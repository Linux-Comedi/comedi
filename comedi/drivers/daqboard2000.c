/*
   module/daqboard2000.c
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/malloc.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <comedi_module.h>
#include <8255.h>
#include <daqboard2000_fpga.h>

#define DAQBOARD2000_SUBSYSTEM_IDS2 	0x00021616 /* Daqboard/2000 - 2 Dacs */
#define DAQBOARD2000_SUBSYSTEM_IDS4 	0x00041616 /* Daqboard/2000 - 4 Dacs */

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
static comedi_lrange range_daqboard2000_ai = { 13, {
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
  RANGE(0, 0.3125)
}};

static comedi_lrange range_daqboard2000_ao = { 1, {
  RANGE(-10, 10)
}};

typedef struct daqboard2000_hw {
  volatile u16  acqControl;               // 0x00
  volatile u16  acqScanListFIFO;          // 0x02
  volatile u32  acqPacerClockDivLow;      // 0x04

  volatile u16  acqScanCounter;           // 0x08
  volatile u16  acqPacerClockDivHigh;     // 0x0a
  volatile u16  acqTriggerCount;          // 0x0c
  volatile u16  fill2;                    // 0x0e
  volatile s16  acqResultsFIFO;           // 0x10
  volatile u16  fill3;                    // 0x12
  volatile s16  acqResultsShadow;         // 0x14
  volatile u16  fill4;                    // 0x16
  volatile s16  acqAdcResult;             // 0x18
  volatile u16  fill5;                    // 0x1a
  volatile u16  dacScanCounter;           // 0x1c
  volatile u16  fill6;                    // 0x1e

  volatile u16  dacControl;               // 0x20
  volatile u16  fill7;                    // 0x22
  volatile s16  dacFIFO;                  // 0x24
  volatile u16  fill8[2];                 // 0x26
  volatile u16  dacPacerClockDiv;         // 0x2a
  volatile u16  refDacs;                  // 0x2c
  volatile u16  fill9;                    // 0x2e

  volatile u16  dioControl;               // 0x30
  volatile s16  dioP3hsioData;            // 0x32
  volatile u16  dioP3Control;             // 0x34
  volatile u16  calEepromControl;         // 0x36
  volatile s16  dacSetting[4];            // 0x38
  volatile s16  dioP2ExpansionIO8Bit[32]; // 0x40

  volatile u16  ctrTmrControl;            // 0x80
  volatile u16  fill10[3];                // 0x82
  volatile s16  ctrInput[4];              // 0x88
  volatile u16  fill11[8];                // 0x90
  volatile u16  timerDivisor[2];          // 0xa0
  volatile u16  fill12[6];                // 0xa4

  volatile u16  dmaControl;               // 0xb0
  volatile u16  trigControl;              // 0xb2
  volatile u16  fill13[2];                // 0xb4
  volatile u16  calEeprom;                // 0xb8
  volatile u16  acqDigitalMark;           // 0xba
  volatile u16  trigDacs;                 // 0xbc
  volatile u16  fill14;                   // 0xbe
  volatile s16  dioP2ExpansionIO16Bit[32];// 0xc0
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



static void daqboard2000_release_resources(comedi_device * dev);
static int daqboard2000_attach(comedi_device *dev,comedi_devconfig *it);
static int daqboard2000_detach(comedi_device *dev);
static int daqboard2000_recognize(char *name);

comedi_driver driver_daqboard2000 = {
  driver_name:	"daqboard2000",
  module:	THIS_MODULE,
  attach:	daqboard2000_attach,
  detach:	daqboard2000_detach,
  recognize:	daqboard2000_recognize,
};


typedef struct {
  enum {
    card_daqboard_2000
  } card;
  void *daq;
  void *plx;
} daqboard2000_private;

#define devpriv ((daqboard2000_private*)dev->private)

static void writeAcqScanListEntry(comedi_device *dev, u16 entry)
{
  daqboard2000_hw *fpga = devpriv->daq;

  udelay(4);
  fpga->acqScanListFIFO = entry & 0x00ff;
  udelay(4);
  fpga->acqScanListFIFO = (entry >> 8) & 0x00ff;
}

static void setup_sampling(comedi_device *dev, int chan, int gain)
{
  u16 word0, word1, word2, word3;

  /* Channel 0-7 diff, channel 8-23 single ended */
  word0 = 0;
  word1 = 0x0004; /* Last scan */
  word2 = (chan<<6) & 0x00c0;
  switch (chan / 4) { 
    case 0: word3 = 0x0001; break;
    case 1: word3 = 0x0002; break;
    case 2: word3 = 0x0005; break;
    case 3: word3 = 0x0006; break;
    case 4: word3 = 0x0041; break;
    case 5: word3 = 0x0042; break;
    default: word3 = 0; break;
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


static int daqboard2000_ai(comedi_device *dev, comedi_subdevice *s, 
			   comedi_trig *it)
{
  int i;
  daqboard2000_hw *fpga = devpriv->daq;

  fpga->acqControl = DAQBOARD2000_AcqResetScanListFifo;
   fpga->acqControl = DAQBOARD2000_AcqResetResultsFifo | DAQBOARD2000_AcqResetConfigPipe;
  for(i=0 ; i < it->n_chan ; i++) {
    int gain, chan, timeout;

    gain = CR_RANGE(it->chanlist[i]);
    chan = CR_CHAN(it->chanlist[i]);		
    setup_sampling(dev, chan, gain);
    fpga->acqControl = DAQBOARD2000_SeqStartScanList;
    for (timeout = 0 ; timeout < 20 ; timeout++) {
      if (fpga->acqControl & DAQBOARD2000_AcqConfigPipeFull) { break; }
      udelay(2);
    }
    fpga->acqControl = DAQBOARD2000_AdcPacerEnable;
    for (timeout = 0 ; timeout < 20 ; timeout++) {
      if (fpga->acqControl & DAQBOARD2000_AcqLogicScanning) { break; }
      udelay(2);
    }
    for (timeout = 0 ; timeout < 20 ; timeout++) {
      if (fpga->acqControl & DAQBOARD2000_AcqResultsFIFOHasValidData) { break;}
      udelay(2);
    }
    it->data[i] = fpga->acqResultsFIFO;
    fpga->acqControl = DAQBOARD2000_AdcPacerDisable;
    fpga->acqControl = DAQBOARD2000_SeqStopScanList;
  }

  return i;
}

static int daqboard2000_ao(comedi_device *dev, comedi_subdevice *s, 
			   comedi_trig *it)
{
  int i;
  daqboard2000_hw *fpga = devpriv->daq;

  for(i=0 ; i < it->n_chan ; i++) {
    int chan, data, timeout;

    chan = CR_CHAN(it->chanlist[i]);
    data = it->data[i];
/*  
    OK, since it works OK without enabling the DAC's, let's keep
    it as simple as possible...
    fpga->dacControl = (chan + 2) * 0x0010 | 0x0001; udelay(1000);
*/
    fpga->dacSetting[chan] = data;
    for (timeout = 0 ; timeout < 20 ; timeout++) {
      if ((fpga->dacControl & ((chan + 1) * 0x0010)) == 0) { break; }
      udelay(2);
    }
/*  
    Since we never enabled the DAC's, we don't need to disable it...
    fpga->dacControl = (chan + 2) * 0x0010 | 0x0000; udelay(1000);
*/

  }
  return i;
}

#if 0
static int daqboard2000_di(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
#if 0
	unsigned int bits;

	bits = inb(dev->iobase + DAQBOARD2000_Digital_4_Bit);

	di_unpack(bits,it);
#endif
	return it->n_chan;

}

static int daqboard2000_do(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
#if 0
	do_pack(&s->state,it);

       /* the outputs are inverted... dumb... (Is this really true? can't
          find it in my docs...) */
	outb(s->state ^ 0xff, dev->iobase + DAQBOARD2000_Digital_4_Bit);
#endif
	return it->n_chan;
}
#endif

static void daqboard2000_resetLocalBus(comedi_device *dev)
{
/*  printk("daqboard2000_resetLocalBus\n");*/
  writel(DAQBOARD2000_SECRLocalBusHi, devpriv->plx + 0x6c);
  udelay(10000);
  writel(DAQBOARD2000_SECRLocalBusLo, devpriv->plx + 0x6c);
  udelay(10000);
}

static void daqboard2000_reloadPLX(comedi_device *dev)
{
/*  printk("daqboard2000_reloadPLX\n");*/
  writel(DAQBOARD2000_SECRReloadLo, devpriv->plx + 0x6c);
  udelay(10000);
  writel(DAQBOARD2000_SECRReloadHi, devpriv->plx + 0x6c);
  udelay(10000);
  writel(DAQBOARD2000_SECRReloadLo, devpriv->plx + 0x6c);
  udelay(10000);
}

static void daqboard2000_pulseProgPin(comedi_device *dev)
{
/*  printk("daqboard2000_pulseProgPin 1\n");*/
  writel(DAQBOARD2000_SECRProgPinHi, devpriv->plx + 0x6c);
  udelay(10000);
  writel(DAQBOARD2000_SECRProgPinLo, devpriv->plx + 0x6c);
  udelay(10000); /* Not in the original code, but I like symmetry... */
}

static int daqboard2000_pollCPLD(comedi_device *dev, int mask) {
  int result = 0;
  int i;

  for (i = 0 ; i < 50 && !result ; i++) {
    /* timeout after 50 tries -> 5ms */
    int cpld = readw(devpriv->daq + 0x1000);
/*    printk("CPLD: %8.8x\n", cpld);*/
    if ((cpld & mask) == mask) {
      result = 1;
    } else {
      udelay(100);
    }
  }
  udelay(5);
  return result;
}

static int daqboard2000_writeCPLD(comedi_device *dev, int data) {
  int result = 0;

  udelay(10);
  writew(data, devpriv->daq + 0x1000);
  if ((readw(devpriv->daq + 0x1000) & DAQBOARD2000_CPLD_INIT) == 
      DAQBOARD2000_CPLD_INIT) {
    result = 1;
  }
  return result;
}

static int initialize_daqboard2000(comedi_device *dev)
{
  int result = -EIO;
  /* Read the serial EEPROM control register */
  int secr;

  /* Check to make sure the serial eeprom is present on the board */
  secr = readl(devpriv->plx + 0x6c); 
  if (secr & DAQBOARD2000_EEPROM_PRESENT) {
    int retry;

    for (retry = 0 ; retry < 3 ; retry++) {
/*      printk("Programming EEPROM try %x\n", retry);*/

      daqboard2000_resetLocalBus(dev);
      daqboard2000_reloadPLX(dev);
      daqboard2000_pulseProgPin(dev);
      if(daqboard2000_pollCPLD(dev, DAQBOARD2000_CPLD_INIT)) {
	int i;

	for (i = 0 ; i < TopBitSize ; i++) {
	  if (bTopBitArray[i] == 0xff && bTopBitArray[i+1] == 0x20) { 
/*	    printk("Preamble found at %d\n", i);*/
	    break; 
	  }
	}
	for ( ; i < TopBitSize ; i += 2) {
	  int data = (bTopBitArray[i]<<8) + bTopBitArray[i+1];
	  if (!daqboard2000_writeCPLD(dev, data)) {
	    break;
	  }
	}
	if (i >= TopBitSize) {
/*	  printk("Programmed\n");*/
	  daqboard2000_resetLocalBus(dev);
	  daqboard2000_reloadPLX(dev);
	  result = 0;
	  break;
	}
      }
    }
  }
  return result;
}


static void daqboard2000_adcStopDmaTransfer(comedi_device *dev)
{
/*  printk("Implement: daqboard2000_adcStopDmaTransfer\n");*/
}

static void daqboard2000_adcDisarm(comedi_device *dev)
{
  daqboard2000_hw *fpga = devpriv->daq;

  /* Disable hardware triggers */  
  udelay(2);
  fpga->trigControl = DAQBOARD2000_TrigAnalog | DAQBOARD2000_TrigDisable;
  udelay(2);
  fpga->trigControl = DAQBOARD2000_TrigTTL | DAQBOARD2000_TrigDisable; 

  /* Stop the scan list FIFO from loading the configuration pipe */
  udelay(2);
  fpga->acqControl = DAQBOARD2000_SeqStopScanList;

  /* Stop the pacer clock */
  udelay(2);
  fpga->acqControl = DAQBOARD2000_AdcPacerDisable; 
   
  /* Stop the input dma (abort channel 1) */
  daqboard2000_adcStopDmaTransfer(dev);
}

static void daqboard2000_activateReferenceDacs(comedi_device *dev)
{
  daqboard2000_hw *fpga = devpriv->daq;
  int timeout;

  // Set the + reference dac value in the FPGA
  fpga->refDacs = 0x80 | DAQBOARD2000_PosRefDacSelect;
  for (timeout = 0 ; timeout < 20 ; timeout++) {
    if ((fpga->dacControl & DAQBOARD2000_RefBusy) == 0) { break; }
    udelay(2);
  }
/*  printk("DAQBOARD2000_PosRefDacSelect %d\n", timeout);*/

  // Set the - reference dac value in the FPGA
  fpga->refDacs = 0x80 | DAQBOARD2000_NegRefDacSelect;
  for (timeout = 0 ; timeout < 20 ; timeout++) {
    if ((fpga->dacControl & DAQBOARD2000_RefBusy) == 0) { break; }
    udelay(2);
  }
/*  printk("DAQBOARD2000_NegRefDacSelect %d\n", timeout);*/
}

static void daqboard2000_initializeCtrs(comedi_device *dev)
{
/*  printk("Implement: daqboard2000_initializeCtrs\n");*/
}

static void daqboard2000_initializeTmrs(comedi_device *dev)
{
/*  printk("Implement: daqboard2000_initializeTmrs\n");*/
}

static void daqboard2000_dacDisarm(comedi_device *dev)
{
/*  printk("Implement: daqboard2000_dacDisarm\n");*/
}

static void daqboard2000_initializeAdc(comedi_device *dev)
{
  daqboard2000_adcDisarm(dev);
  daqboard2000_activateReferenceDacs(dev);
  daqboard2000_initializeCtrs(dev);
  daqboard2000_initializeTmrs(dev);
}

static void daqboard2000_initializeDac(comedi_device *dev)
{
  daqboard2000_dacDisarm(dev);
}

/*
  options[0]   Board number?? (FIXME: David, please decide)
 */
static int daqboard2000_recognize(char *name)
{
  int result = -1;
  if (strcmp("daqboard/2000", name) == 0) { result = card_daqboard_2000; }
  return result;
}

/*
The test command, REMOVE!!:

rmmod daqboard2000 ; rmmod comedi; make install ; modprobe daqboard2000; /usr/sbin/comedi_config /dev/comedi0 daqboard/2000 ; tail -40 /var/log/messages
*/

static int daqboard2000_8255_cb(int dir, int port, int data, void *arg)
{
  int result = 0;
  int iobase=(int)arg;
  if(dir){
    *((u16*)(iobase+port*2)) = data;
    result = 0;
  }else{
    result = *((u16*)(iobase+port*2));
  }
/*  printk("daqboard2000_8255_cb %x %d %d %2.2x -> %2.2x\n",
        arg, dir, port, data, result);
*/
  return result;
}

static int daqboard2000_attach(comedi_device *dev, comedi_devconfig *it)
{
  int result = 0;
  comedi_subdevice *s;
  struct pci_dev *card = NULL;

  if(result == 0 && !pci_present()) {
    printk("daqboard2000: PCI bus not present!\n");
    result = -EIO;
  }

  if (result == 0) {
    /* FIXME: we should handle multiple cards, have to make David decide 
              how, so we will be consistent among all PCI card drivers... */
    card = pci_find_device(0x1616, 0x0409, NULL);

    if (card) {
      printk("comedi%d: daqboard2000: 0x%8.8x ", dev->minor, (int)card);
    } else {
      printk("no daqboard2000 found\n");
      result = -EIO;
    }
  }

  if (result == 0 && card->hdr_type == PCI_HEADER_TYPE_NORMAL) {
    u32 id;
    pci_read_config_dword(card, PCI_SUBSYSTEM_VENDOR_ID, &id);
    if (id != DAQBOARD2000_SUBSYSTEM_IDS2 &&
	id != DAQBOARD2000_SUBSYSTEM_IDS4) {
      printk("daqboard2000: unknown subsystem vendor %8.8x\n ", id);
      result = -EIO;
    }
  }
  
  if (result == 0) {
    result = alloc_private(dev,sizeof(daqboard2000_private));
  }
  
  if (result == 0) {
    int secr;
#if LINUX_VERSION_CODE < 0x020300
    devpriv->plx = ioremap(card->base_address[0], DAQBOARD2000_PLX_SIZE);
    devpriv->daq = ioremap(card->base_address[2], DAQBOARD2000_DAQ_SIZE);
#else
    devpriv->plx = ioremap(card->resource[0].start, DAQBOARD2000_PLX_SIZE);
    devpriv->daq = ioremap(card->resource[2].start, DAQBOARD2000_DAQ_SIZE);
#endif
    secr = readl(devpriv->plx + 0x6c); 
  }

  if (result == 0) {
    /*
      u8 interrupt;
      Windows code does restore interrupts, but since we don't use them...
      pci_read_config_byte(card, PCI_INTERRUPT_LINE, &interrupt);
      printk("Interrupt before is: %x\n", interrupt);
    */
    result = initialize_daqboard2000(dev);
    daqboard2000_initializeAdc(dev);
    daqboard2000_initializeDac(dev);
    /*
      Windows code does restore interrupts, but since we don't use them...
      pci_read_config_byte(card, PCI_INTERRUPT_LINE, &interrupt);
      printk("Interrupt after is: %x\n", interrupt);
    */
  }

  if (result == 0) {
    dev->iobase = (int)devpriv->daq;
    
    if (check_region((long)devpriv->plx, DAQBOARD2000_PLX_SIZE) < 0 ||
	check_region((long)devpriv->daq, DAQBOARD2000_DAQ_SIZE) < 0) {
      printk("I/O port conflict\n");
      result = -EIO;
    }
  }

  if (result == 0) {
    request_region((long)devpriv->plx, DAQBOARD2000_PLX_SIZE, "daqboard2000");
    request_region((long)devpriv->daq, DAQBOARD2000_DAQ_SIZE, "daqboard2000");
  }
  
  if (result == 0) {
    switch(dev->board){
      case card_daqboard_2000:
      default:
	dev->board_name = "daqboard/2000";
	break;
    }
  }

  if (result == 0) {
    dev->n_subdevices = 3;
    result = alloc_subdevices(dev);
  }

  if (result == 0) {
    s = dev->subdevices + 0;
    /* ai subdevice */
    s->type = COMEDI_SUBD_AI;
    s->subdev_flags = SDF_READABLE|SDF_RT;
    s->n_chan = 24;
    s->maxdata = 0xffff;
    s->trig[0] = daqboard2000_ai;
    s->range_table = &range_daqboard2000_ai;

    s = dev->subdevices + 1;
    /* ao subdevice */
    s->type = COMEDI_SUBD_AO;
    s->subdev_flags = SDF_WRITEABLE|SDF_RT;
    s->n_chan = 2;
    s->maxdata = 0xffff;
    s->trig[0] = daqboard2000_ao;
    s->range_table = &range_daqboard2000_ao;

    s = dev->subdevices + 2;
    result = subdev_8255_init(dev,s,daqboard2000_8255_cb,
                             (void *)(dev->iobase+0x40));
  }
  
  printk("\n");
  return result;
}

static void daqboard2000_release_resources(comedi_device * dev)
{
  if (devpriv && devpriv->daq) {
    release_region((long)devpriv->daq, DAQBOARD2000_DAQ_SIZE);
    iounmap(devpriv->daq);
  }
  if (devpriv && devpriv->plx) {
    release_region((long)devpriv->plx, DAQBOARD2000_PLX_SIZE);
    iounmap(devpriv->plx);
  }
  if (dev->irq) {
    free_irq(dev->irq, dev);
  }
}

static int daqboard2000_detach(comedi_device * dev)
{
  printk("comedi%d: daqboard2000: remove\n", dev->minor);
  daqboard2000_release_resources(dev);
  return 0;
}

#ifdef MODULE
int init_module(void)
{
  comedi_driver_register(&driver_daqboard2000);
  return 0;
}

void cleanup_module(void)
{
  comedi_driver_unregister(&driver_daqboard2000);
}
#endif
