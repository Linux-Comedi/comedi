/*
   comedi/drivers/daqboard2000.c
   hardware driver for IOtech DAQboard/2000

   COMEDI - Linux Control and Measurement Device Interface
   Copyright (C) 1999 Anders Blomdell <anders.blomdell@control.lth.se>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

 */
/*
Driver: daqboard2000
Description: IOTech DAQBoard/2000
Author: Anders Blomdell <anders.blomdell@control.lth.se>
Status: works
Updated: Tue, 03 Feb 2026 14:18:10 +0000
Devices: [IOTech] DAQBoard/2000 (daqboard2000)

Much of the functionality of this driver was determined from reading
the source code for the Windows driver.

The FPGA on the board requires initialization code from file
"daqboard2000_firmware.bin" which should be placed in the /lib/firmware/
directory.

The initialization code is available from http://www.comedi.org
in the comedi_nonfree_firmware tarball.

The driver will request the initialization code file from the system
during configuration of the COMEDI device.

Configuration options:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
  If bus/slot is not specified, the first supported
  PCI device found will be used.
*/
/*
   This card was obviously never intended to leave the Windows world,
   since it lacked all kind of hardware documentation (except for cable
   pinouts, plug and pray has something to catch up with yet).

   With some help from our swedish distributor, we got the Windows sourcecode
   for the card, and here are the findings so far.

   1. A good document that describes the PCI interface chip is found at:
      http://plx.plxtech.com/download/9080/databook/9080db-106.pdf

   2. The initialization done so far is:
        a. program the FPGA (windows code sans a lot of error messages)
	b.

   3. Analog out seems to work OK with DAC's disabled, if DAC's are enabled,
      you have to output values to all enabled DAC's until result appears, I
      guess that it has something to do with pacer clocks, but the source
      gives me no clues. I'll keep it simple so far.

   4. Analog in.
        Each channel in the scanlist seems to be controlled by four
	control words:

        Word0:
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          ! | | | ! | | | ! | | | ! | | | !
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

        Word1:
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          ! | | | ! | | | ! | | | ! | | | !
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
	   |             |       | | | | |
           +------+------+       | | | | +-- Digital input (??)
		  |		 | | | +---- 10 us settling time
		  |		 | | +------ Suspend acquisition (last to scan)
		  |		 | +-------- Simultaneous sample and hold
		  |		 +---------- Signed data format
		  +------------------------- Correction offset low

        Word2:
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          ! | | | ! | | | ! | | | ! | | | !
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
           |     | |     | | | | | |     |
           +-----+ +--+--+ +++ +++ +--+--+
              |       |     |   |     +----- Expansion channel
	      |       |     |   +----------- Expansion gain
              |       |     +--------------- Channel (low)
	      |       +--------------------- Correction offset high
	      +----------------------------- Correction gain low
        Word3:
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
          ! | | | ! | | | ! | | | ! | | | !
          +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
           |             | | | |   | | | |
           +------+------+ | | +-+-+ | | +-- Low bank enable
                  |        | |   |   | +---- High bank enable
                  |        | |   |   +------ Hi/low select
		  |    	   | |   +---------- Gain (1,?,2,4,8,16,32,64)
		  |    	   | +-------------- differential/single ended
		  |    	   +---------------- Unipolar
		  +------------------------- Correction gain high



   999. The card seems to have an incredible amount of capabilities, but
        trying to reverse engineer them from the Windows source is beyond my
	patience.


 */

#include <linux/comedidev.h>

#include <linux/delay.h>

#include "comedi_pci.h"
#include "8255.h"

#define DAQBOARD2000_FIRMWARE		"daqboard2000_firmware.bin"

#define DAQBOARD2000_SUBSYSTEM_IDS2 	0x00021616	/* Daqboard/2000 - 2 Dacs */
#define DAQBOARD2000_SUBSYSTEM_IDS4 	0x00041616	/* Daqboard/2000 - 4 Dacs */

#define DAQBOARD2000_DAQ_SIZE 		0x1002
#define DAQBOARD2000_PLX_SIZE 		0x100

// Initialization bits for the Serial EEPROM Control Register
#define DAQBOARD2000_SECRProgPinHi      0x8001767e
#define DAQBOARD2000_SECRProgPinLo      0x8000767e
#define DAQBOARD2000_SECRLocalBusHi     0xc000767e
#define DAQBOARD2000_SECRLocalBusLo     0x8000767e
#define DAQBOARD2000_SECRReloadHi       0xa000767e
#define DAQBOARD2000_SECRReloadLo       0x8000767e

// SECR status bits
#define DAQBOARD2000_EEPROM_PRESENT     0x10000000

// CPLD status bits
#define DAQBOARD2000_CPLD_INIT 		0x0002
#define DAQBOARD2000_CPLD_DONE 		0x0004

// Available ranges
static const comedi_lrange range_daqboard2000_ai = {
	13,
	{
		RANGE(-10, 10),
		RANGE(-5, 5),
		RANGE(-2.5, 2.5),
		RANGE(-1.25, 1.25),
		RANGE(-0.625, 0.625),
		RANGE(-0.3125, 0.3125),
		RANGE(-0.156, 0.156),
		RANGE(0, 10),
		RANGE(0, 5),
		RANGE(0, 2.5),
		RANGE(0, 1.25),
		RANGE(0, 0.625),
		RANGE(0, 0.3125),
	},
};

static const comedi_lrange range_daqboard2000_ao = {
	1,
	{
		RANGE(-10, 10),
	},
};

typedef struct daqboard2000_hw {
	u16 acqControl;			// 0x00
	u16 acqScanListFIFO;		// 0x02
	u32 acqPacerClockDivLow;	// 0x04

	u16 acqScanCounter;		// 0x08
	u16 acqPacerClockDivHigh;	// 0x0a
	u16 acqTriggerCount;		// 0x0c
	u16 fill2;			// 0x0e
	u16 acqResultsFIFO;		// 0x10
	u16 fill3;			// 0x12
	u16 acqResultsShadow;		// 0x14
	u16 fill4;			// 0x16
	u16 acqAdcResult;		// 0x18
	u16 fill5;			// 0x1a
	u16 dacScanCounter;		// 0x1c
	u16 fill6;			// 0x1e

	u16 dacControl;			// 0x20
	u16 fill7;			// 0x22
	s16 dacFIFO;			// 0x24
	u16 fill8[2];			// 0x26
	u16 dacPacerClockDiv;		// 0x2a
	u16 refDacs;			// 0x2c
	u16 fill9;			// 0x2e

	u16 dioControl;			// 0x30
	s16 dioP3hsioData;		// 0x32
	u16 dioP3Control;		// 0x34
	u16 calEepromControl;		// 0x36
	s16 dacSetting[4];		// 0x38
	s16 dioP2ExpansionIO8Bit[32];	// 0x40

	u16 ctrTmrControl;		// 0x80
	u16 fill10[3];			// 0x82
	s16 ctrInput[4];		// 0x88
	u16 fill11[8];			// 0x90
	u16 timerDivisor[2];		// 0xa0
	u16 fill12[6];			// 0xa4

	u16 dmaControl;			// 0xb0
	u16 trigControl;		// 0xb2
	u16 fill13[2];			// 0xb4
	u16 calEeprom;			// 0xb8
	u16 acqDigitalMark;		// 0xba
	u16 trigDacs;			// 0xbc
	u16 fill14;			// 0xbe
	s16 dioP2ExpansionIO16Bit[32];	// 0xc0
} daqboard2000_hw;

/* Scan Sequencer programming */
#define DAQBOARD2000_SeqStartScanList            0x0011
#define DAQBOARD2000_SeqStopScanList             0x0010

// Prepare for acquisition
#define DAQBOARD2000_AcqResetScanListFifo        0x0004
#define DAQBOARD2000_AcqResetResultsFifo         0x0002
#define DAQBOARD2000_AcqResetConfigPipe          0x0001

// Acqusition status bits
#define DAQBOARD2000_AcqResultsFIFOMore1Sample   0x0001
#define DAQBOARD2000_AcqResultsFIFOHasValidData  0x0002
#define DAQBOARD2000_AcqResultsFIFOOverrun       0x0004
#define DAQBOARD2000_AcqLogicScanning            0x0008
#define DAQBOARD2000_AcqConfigPipeFull           0x0010
#define DAQBOARD2000_AcqScanListFIFOEmpty        0x0020
#define DAQBOARD2000_AcqAdcNotReady              0x0040
#define DAQBOARD2000_ArbitrationFailure          0x0080
#define DAQBOARD2000_AcqPacerOverrun             0x0100
#define DAQBOARD2000_DacPacerOverrun             0x0200
#define DAQBOARD2000_AcqHardwareError            0x01c0

// Scan Sequencer programming
#define DAQBOARD2000_SeqStartScanList            0x0011
#define DAQBOARD2000_SeqStopScanList             0x0010

/* Pacer Clock Control */
#define DAQBOARD2000_AdcPacerInternal            0x0030
#define DAQBOARD2000_AdcPacerExternal            0x0032
#define DAQBOARD2000_AdcPacerEnable              0x0031
#define DAQBOARD2000_AdcPacerEnableDacPacer      0x0034
#define DAQBOARD2000_AdcPacerDisable             0x0030
#define DAQBOARD2000_AdcPacerNormalMode          0x0060
#define DAQBOARD2000_AdcPacerCompatibilityMode   0x0061
#define DAQBOARD2000_AdcPacerInternalOutEnable   0x0008
#define DAQBOARD2000_AdcPacerExternalRising      0x0100

// DAC status
#define DAQBOARD2000_DacFull                     0x0001
#define DAQBOARD2000_RefBusy                     0x0002
#define DAQBOARD2000_TrgBusy                     0x0004
#define DAQBOARD2000_CalBusy                     0x0008
#define DAQBOARD2000_Dac0Busy                    0x0010
#define DAQBOARD2000_Dac1Busy                    0x0020
#define DAQBOARD2000_Dac2Busy                    0x0040
#define DAQBOARD2000_Dac3Busy                    0x0080

// DAC control
#define DAQBOARD2000_Dac0Enable                  0x0021
#define DAQBOARD2000_Dac1Enable                  0x0031
#define DAQBOARD2000_Dac2Enable                  0x0041
#define DAQBOARD2000_Dac3Enable                  0x0051
#define DAQBOARD2000_DacEnableBit                0x0001
#define DAQBOARD2000_Dac0Disable                 0x0020
#define DAQBOARD2000_Dac1Disable                 0x0030
#define DAQBOARD2000_Dac2Disable                 0x0040
#define DAQBOARD2000_Dac3Disable                 0x0050
#define DAQBOARD2000_DacResetFifo                0x0004
#define DAQBOARD2000_DacPatternDisable           0x0060
#define DAQBOARD2000_DacPatternEnable            0x0061
#define DAQBOARD2000_DacSelectSignedData         0x0002
#define DAQBOARD2000_DacSelectUnsignedData       0x0000

/* Trigger Control */
#define DAQBOARD2000_TrigAnalog                  0x0000
#define DAQBOARD2000_TrigTTL                     0x0010
#define DAQBOARD2000_TrigTransHiLo               0x0004
#define DAQBOARD2000_TrigTransLoHi               0x0000
#define DAQBOARD2000_TrigAbove                   0x0000
#define DAQBOARD2000_TrigBelow                   0x0004
#define DAQBOARD2000_TrigLevelSense              0x0002
#define DAQBOARD2000_TrigEdgeSense               0x0000
#define DAQBOARD2000_TrigEnable                  0x0001
#define DAQBOARD2000_TrigDisable                 0x0000

// Reference Dac Selection
#define DAQBOARD2000_PosRefDacSelect             0x0100
#define DAQBOARD2000_NegRefDacSelect             0x0000

static int daqboard2000_attach(comedi_device * dev, comedi_devconfig * it);
static int daqboard2000_detach(comedi_device * dev);

static comedi_driver driver_daqboard2000 = {
      .driver_name	= "daqboard2000",
      .module	 	= THIS_MODULE,
      .attach		= daqboard2000_attach,
      .detach		= daqboard2000_detach,
};

typedef struct {
	const char *name;
	u32 id;
} boardtype;

static const boardtype boardtypes[] = {
	{
		.name	= "ids2",
		.id	= DAQBOARD2000_SUBSYSTEM_IDS2,
	},
	{
		.name	= "ids4",
		.id	= DAQBOARD2000_SUBSYSTEM_IDS4
	},
};

#define n_boardtypes (sizeof(boardtypes)/sizeof(boardtype))
#define this_board ((const boardtype *)dev->board_ptr)

static DEFINE_PCI_DEVICE_TABLE(daqboard2000_pci_table) = {
	{0x1616, 0x0409, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0}
};

MODULE_DEVICE_TABLE(pci, daqboard2000_pci_table);

typedef struct {
	struct pci_dev *pci_dev;
	void __iomem *daq;
	void __iomem *plx;
	int got_regions;
	lsampl_t ao_readback[2];
} daqboard2000_private;

#define devpriv ((daqboard2000_private*)dev->private)

static void writeAcqScanListEntry(comedi_device * dev, u16 entry)
{
	daqboard2000_hw __iomem *fpga = devpriv->daq;

//  comedi_udelay(4);
	writew(entry & 0x00ff, &fpga->acqScanListFIFO);
//  comedi_udelay(4);
	writew((entry >> 8) & 0x00ff, &fpga->acqScanListFIFO);
}

static void setup_sampling(comedi_device * dev, int chan, int gain)
{
	u16 word0, word1, word2, word3;

	/* Channel 0-7 diff, channel 8-23 single ended */
	word0 = 0;
	word1 = 0x0004;		/* Last scan */
	word2 = (chan << 6) & 0x00c0;
	switch (chan / 4) {
	case 0:
		word3 = 0x0001;
		break;
	case 1:
		word3 = 0x0002;
		break;
	case 2:
		word3 = 0x0005;
		break;
	case 3:
		word3 = 0x0006;
		break;
	case 4:
		word3 = 0x0041;
		break;
	case 5:
		word3 = 0x0042;
		break;
	default:
		word3 = 0;
		break;
	}
/*
  dev->eeprom.correctionDACSE[i][j][k].offset = 0x800;
  dev->eeprom.correctionDACSE[i][j][k].gain = 0xc00;
*/
	/* These should be read from EEPROM */
	word2 |= 0x0800;
	word3 |= 0xc000;
/*  printk("%d %4.4x %4.4x %4.4x %4.4x\n", chan, word0, word1, word2, word3);*/
	writeAcqScanListEntry(dev, word0);
	writeAcqScanListEntry(dev, word1);
	writeAcqScanListEntry(dev, word2);
	writeAcqScanListEntry(dev, word3);
}

static int daqboard2000_ai_insn_read(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i;
	daqboard2000_hw __iomem *fpga = devpriv->daq;
	int gain, chan, timeout;

	writew((DAQBOARD2000_AcqResetScanListFifo |
		DAQBOARD2000_AcqResetResultsFifo |
		DAQBOARD2000_AcqResetConfigPipe), &fpga->acqControl);

	/* If pacer clock is not set to some high value (> 10 us), we
	   risk multiple samples to be put into the result FIFO. */
	writel(1000000, &fpga->acqPacerClockDivLow);	/* 1 second, should be long enough */
	writew(0, &fpga->acqPacerClockDivHigh);

	gain = CR_RANGE(insn->chanspec);
	chan = CR_CHAN(insn->chanspec);

	/* This doesn't look efficient.  I decided to take the conservative
	 * approach when I did the insn conversion.  Perhaps it would be
	 * better to have broken it completely, then someone would have been
	 * forced to fix it.  --ds */
	for (i = 0; i < insn->n; i++) {
		setup_sampling(dev, chan, gain);
		/* Enable reading from the scanlist FIFO */
		writew(DAQBOARD2000_SeqStartScanList, &fpga->acqControl);
		for (timeout = 0; timeout < 20; timeout++) {
			if (!!(readw(&fpga->acqControl) &
			       DAQBOARD2000_AcqConfigPipeFull)) {
				break;
			}
			//comedi_udelay(2);
		}
		writew(DAQBOARD2000_AdcPacerEnable, &fpga->acqControl);
		for (timeout = 0; timeout < 20; timeout++) {
			if (!!(readw(&fpga->acqControl) &
			       DAQBOARD2000_AcqLogicScanning)) {
				break;
			}
			//comedi_udelay(2);
		}
		for (timeout = 0; timeout < 20; timeout++) {
			if (!!(readw(&fpga->acqControl) &
			       DAQBOARD2000_AcqResultsFIFOHasValidData)) {
				break;
			}
			//comedi_udelay(2);
		}
		data[i] = readw(&fpga->acqResultsFIFO);
		writew(DAQBOARD2000_AdcPacerDisable, &fpga->acqControl);
		writew(DAQBOARD2000_SeqStopScanList, &fpga->acqControl);
	}

	return i;
}

static int daqboard2000_ao_insn_read(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++) {
		data[i] = devpriv->ao_readback[chan];
	}

	return i;
}

static int daqboard2000_ao_insn_write(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);
	daqboard2000_hw __iomem *fpga = devpriv->daq;
	int timeout;

	for (i = 0; i < insn->n; i++) {
		/*
		 * OK, since it works OK without enabling the DAC's, let's keep
		 * it as simple as possible...
		 */
		//fpga->dacControl = (chan + 2) * 0x0010 | 0x0001; comedi_udelay(1000);
		writew(data[i], &fpga->dacSetting[chan]);
		for (timeout = 0; timeout < 20; timeout++) {
			if ((readw(&fpga->dacControl) &
			     ((chan + 1) * 0x0010)) == 0) {
				break;
			}
			//comedi_udelay(2);
		}
		devpriv->ao_readback[chan] = data[i];
		/*
		 * Since we never enabled the DAC's, we don't need to disable it...
		 * fpga->dacControl = (chan + 2) * 0x0010 | 0x0000; comedi_udelay(1000);
		 */
	}

	return i;
}

static void daqboard2000_resetLocalBus(comedi_device * dev)
{
	printk("daqboard2000_resetLocalBus\n");
	writel(DAQBOARD2000_SECRLocalBusHi, devpriv->plx + 0x6c);
	msleep(10);
	writel(DAQBOARD2000_SECRLocalBusLo, devpriv->plx + 0x6c);
	msleep(10);
}

static void daqboard2000_reloadPLX(comedi_device * dev)
{
	printk("daqboard2000_reloadPLX\n");
	writel(DAQBOARD2000_SECRReloadLo, devpriv->plx + 0x6c);
	msleep(10);
	writel(DAQBOARD2000_SECRReloadHi, devpriv->plx + 0x6c);
	msleep(10);
	writel(DAQBOARD2000_SECRReloadLo, devpriv->plx + 0x6c);
	msleep(10);
}

static void daqboard2000_pulseProgPin(comedi_device * dev)
{
	printk("daqboard2000_pulseProgPin 1\n");
	writel(DAQBOARD2000_SECRProgPinHi, devpriv->plx + 0x6c);
	msleep(10);
	writel(DAQBOARD2000_SECRProgPinLo, devpriv->plx + 0x6c);
	msleep(10);	/* Not in the original code, but I like symmetry... */
}

static int daqboard2000_pollCPLD(comedi_device * dev, int mask)
{
	int result = 0;
	int i;
	int cpld;

	/* timeout after 50 tries -> 5ms */
	for (i = 0; i < 50; i++) {
		cpld = readw(devpriv->daq + 0x1000);
		if ((cpld & mask) == mask) {
			result = 1;
			break;
		}
		comedi_udelay(100);
	}
	comedi_udelay(5);
	return result;
}

static int daqboard2000_writeCPLD(comedi_device * dev, int data)
{
	int result = 0;

	comedi_udelay(10);
	writew(data, devpriv->daq + 0x1000);
	if ((readw(devpriv->daq + 0x1000) & DAQBOARD2000_CPLD_INIT) ==
		DAQBOARD2000_CPLD_INIT) {
		result = 1;
	}
	return result;
}

static int initialize_daqboard2000(comedi_device * dev,
	const u8 *cpld_array, size_t len, unsigned long context)
{
	int result = -EIO;
	/* Read the serial EEPROM control register */
	int secr;
	int retry;
	int i;

	/* Check to make sure the serial eeprom is present on the board */
	secr = readl(devpriv->plx + 0x6c);
	if (!(secr & DAQBOARD2000_EEPROM_PRESENT)) {
#ifdef DEBUG_EEPROM
		printk("no serial eeprom\n");
#endif
		return -EIO;
	}

	for (retry = 0; retry < 3; retry++) {
#ifdef DEBUG_EEPROM
		printk("Programming EEPROM try %x\n", retry);
#endif

		daqboard2000_resetLocalBus(dev);
		daqboard2000_reloadPLX(dev);
		daqboard2000_pulseProgPin(dev);
		if (daqboard2000_pollCPLD(dev, DAQBOARD2000_CPLD_INIT)) {
			for (i = 0; i < len; i++) {
				if (cpld_array[i] == 0xff
					&& cpld_array[i + 1] == 0x20) {
#ifdef DEBUG_EEPROM
					printk("Preamble found at %d\n", i);
#endif
					break;
				}
			}
			for (; i < len; i += 2) {
				int data =
					(cpld_array[i] << 8) + cpld_array[i +
					1];
				if (!daqboard2000_writeCPLD(dev, data)) {
					break;
				}
			}
			if (i >= len) {
#ifdef DEBUG_EEPROM
				printk("Programmed\n");
#endif
				daqboard2000_resetLocalBus(dev);
				daqboard2000_reloadPLX(dev);
				result = 0;
				break;
			}
		}
	}
	return result;
}

static void daqboard2000_adcStopDmaTransfer(comedi_device * dev)
{
/*  printk("Implement: daqboard2000_adcStopDmaTransfer\n");*/
}

static void daqboard2000_adcDisarm(comedi_device * dev)
{
	daqboard2000_hw __iomem *fpga = devpriv->daq;

	/* Disable hardware triggers */
	comedi_udelay(2);
	writew(DAQBOARD2000_TrigAnalog | DAQBOARD2000_TrigDisable,
	       &fpga->trigControl);
	comedi_udelay(2);
	writew(DAQBOARD2000_TrigTTL | DAQBOARD2000_TrigDisable,
	       &fpga->trigControl);

	/* Stop the scan list FIFO from loading the configuration pipe */
	comedi_udelay(2);
	writew(DAQBOARD2000_SeqStopScanList, &fpga->acqControl);

	/* Stop the pacer clock */
	comedi_udelay(2);
	writew(DAQBOARD2000_AdcPacerDisable, &fpga->acqControl);

	/* Stop the input dma (abort channel 1) */
	daqboard2000_adcStopDmaTransfer(dev);
}

static void daqboard2000_activateReferenceDacs(comedi_device * dev)
{
	daqboard2000_hw __iomem *fpga = devpriv->daq;
	int timeout;

	// Set the + reference dac value in the FPGA
	writew(0x80 | DAQBOARD2000_PosRefDacSelect, &fpga->refDacs);
	for (timeout = 0; timeout < 20; timeout++) {
		if ((readw(&fpga->dacControl) & DAQBOARD2000_RefBusy) == 0) {
			break;
		}
		comedi_udelay(2);
	}
/*  printk("DAQBOARD2000_PosRefDacSelect %d\n", timeout);*/

	// Set the - reference dac value in the FPGA
	writew(0x80 | DAQBOARD2000_NegRefDacSelect, &fpga->refDacs);
	for (timeout = 0; timeout < 20; timeout++) {
		if ((readw(&fpga->dacControl) & DAQBOARD2000_RefBusy) == 0) {
			break;
		}
		comedi_udelay(2);
	}
/*  printk("DAQBOARD2000_NegRefDacSelect %d\n", timeout);*/
}

static void daqboard2000_initializeCtrs(comedi_device * dev)
{
/*  printk("Implement: daqboard2000_initializeCtrs\n");*/
}

static void daqboard2000_initializeTmrs(comedi_device * dev)
{
/*  printk("Implement: daqboard2000_initializeTmrs\n");*/
}

static void daqboard2000_dacDisarm(comedi_device * dev)
{
/*  printk("Implement: daqboard2000_dacDisarm\n");*/
}

static void daqboard2000_initializeAdc(comedi_device * dev)
{
	daqboard2000_adcDisarm(dev);
	daqboard2000_activateReferenceDacs(dev);
	daqboard2000_initializeCtrs(dev);
	daqboard2000_initializeTmrs(dev);
}

static void daqboard2000_initializeDac(comedi_device * dev)
{
	daqboard2000_dacDisarm(dev);
}

/*
The test command, REMOVE!!:

rmmod daqboard2000 ; rmmod comedi; make install ; modprobe daqboard2000; /usr/sbin/comedi_config /dev/comedi0 daqboard/2000 ; tail -40 /var/log/messages
*/

static int daqboard2000_8255_cb(int dir, int port, int data,
	unsigned long ioaddr)
{
	int result = 0;
	if (dir) {
		writew(data, ((void *)ioaddr) + port * 2);
		result = 0;
	} else {
		result = readw(((void *)ioaddr) + port * 2);
	}
/*
  printk("daqboard2000_8255_cb %x %d %d %2.2x -> %2.2x\n",
        arg, dir, port, data, result);
*/
	return result;
}

static int daqboard2000_attach(comedi_device * dev, comedi_devconfig * it)
{
	int result = 0;
	comedi_subdevice *s;
	struct pci_dev *card = NULL;
	int bus, slot;

	printk("comedi%d: daqboard2000: ", dev->minor);

	bus = it->options[0];
	slot = it->options[1];

	result = alloc_private(dev, sizeof(daqboard2000_private));
	if (result < 0) {
		printk(KERN_CONT "Allocation error\n");
		return -ENOMEM;
	}
	for (card = pci_get_device(0x1616, 0x0409, NULL);
		card != NULL;
		card = pci_get_device(0x1616, 0x0409, card)) {
		if (bus || slot) {
			/* requested particular bus/slot */
			if (card->bus->number != bus ||
				PCI_SLOT(card->devfn) != slot) {
				continue;
			}
		}
		break;  /* found one */
	}
	if (!card) {
		if (bus || slot)
			printk(KERN_CONT "no daqboard2000 found at bus/slot: %d/%d\n",
				bus, slot);
		else
			printk(KERN_CONT "no daqboard2000 found\n");
		return -EIO;
	} else {
		u32 id;
		int i;
		devpriv->pci_dev = card;
		id = ((u32) card->subsystem_device << 16) | card->
			subsystem_vendor;
		for (i = 0; i < n_boardtypes; i++) {
			if (boardtypes[i].id == id) {
				printk(KERN_CONT " %s", boardtypes[i].name);
				dev->board_ptr = boardtypes + i;
			}
		}
		if (!dev->board_ptr) {
			printk(KERN_CONT "unknown subsystem id %08x (pretend it is an ids2) ", id);
			dev->board_ptr = boardtypes;
		}
	}

	if ((result = comedi_pci_enable(card, "daqboard2000")) < 0) {
		printk(KERN_CONT "failed to enable PCI device and request regions\n");
		return -EIO;
	}
	devpriv->got_regions = 1;
	devpriv->plx =
		ioremap(pci_resource_start(card, 0), DAQBOARD2000_PLX_SIZE);
	devpriv->daq =
		ioremap(pci_resource_start(card, 2), DAQBOARD2000_DAQ_SIZE);
	if (!devpriv->plx || !devpriv->daq) {
		printk(KERN_CONT "failed to remap registers\n");
		return -ENOMEM;
	}

	result = alloc_subdevices(dev, 3);
	if (result < 0) {
		printk(KERN_CONT "Allocation error\n");
		goto out;
	}

	readl(devpriv->plx + 0x6c);

	/*
	   u8 interrupt;
	   Windows code does restore interrupts, but since we don't use them...
	   pci_read_config_byte(card, PCI_INTERRUPT_LINE, &interrupt);
	   printk("Interrupt before is: %x\n", interrupt);
	 */

	printk(KERN_CONT "\n");
	result = comedi_load_firmware(dev, &devpriv->pci_dev->dev,
				      DAQBOARD2000_FIRMWARE,
				      initialize_daqboard2000, 0);
	if (result) {
		printk("comedi%d: Failed to load FPGA initialization code, error %d\n",
			dev->minor, result);
		goto out;
	}
	daqboard2000_initializeAdc(dev);
	daqboard2000_initializeDac(dev);
	/*
	   Windows code does restore interrupts, but since we don't use them...
	   pci_read_config_byte(card, PCI_INTERRUPT_LINE, &interrupt);
	   printk("Interrupt after is: %x\n", interrupt);
	 */

	dev->iobase = (unsigned long)devpriv->daq;

	dev->board_name = this_board->name;

	s = dev->subdevices + 0;
	/* ai subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND;
	s->n_chan = 24;
	s->maxdata = 0xffff;
	s->insn_read = daqboard2000_ai_insn_read;
	s->range_table = &range_daqboard2000_ai;

	s = dev->subdevices + 1;
	/* ao subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITABLE;
	s->n_chan = 2;
	s->maxdata = 0xffff;
	s->insn_read = daqboard2000_ao_insn_read;
	s->insn_write = daqboard2000_ao_insn_write;
	s->range_table = &range_daqboard2000_ao;

	s = dev->subdevices + 2;
	result = subdev_8255_init(dev, s, daqboard2000_8255_cb,
		(unsigned long)(dev->iobase + 0x40));

      out:
	return result;
}

static int daqboard2000_detach(comedi_device * dev)
{
	printk("comedi%d: daqboard2000: remove\n", dev->minor);

	if (dev->subdevices)
		subdev_8255_cleanup(dev, dev->subdevices + 2);

	if (dev->irq) {
		free_irq(dev->irq, dev);
	}
	if (devpriv) {
		if (devpriv->daq)
			iounmap(devpriv->daq);
		if (devpriv->plx)
			iounmap(devpriv->plx);
		if (devpriv->pci_dev) {
			if (devpriv->got_regions) {
				comedi_pci_disable(devpriv->pci_dev);
			}
			pci_dev_put(devpriv->pci_dev);
		}
	}
	return 0;
}

COMEDI_PCI_INITCLEANUP(driver_daqboard2000, daqboard2000_pci_table);
