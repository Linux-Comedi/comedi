/*
  comedi/drivers/ni_660x.c
  Hardware driver for NI 660x devices

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
Driver: ni_660x.o
Description: National Instruments 660x counter/timer boards
Devices:
[National Instruments] PCI-6601 (ni_660x), PCI-6602
Author: J.P. Mellor <jpmellor@rose-hulman.edu>,
	Herman.Bruyninckx@mech.kuleuven.ac.be,
	Wim.Meeussen@mech.kuleuven.ac.be,
	Klaas.Gadeyne@mech.kuleuven.ac.be,
	Frank Mori Hess <fmhess@users.sourceforge.net>
Updated: Sun Nov 16 18:46:11 UTC 2003
Status: experimental

Encoders work, but only with instructions, commands are not
supported yet.  PulseGeneration (both single pulse and pulse train)
works. DIO is experimental (8 channels only). Interrupts do not
work.

References:
DAQ 660x Register-Level Programmer Manual  (NI 370505A-01)
DAQ 6601/6602 User Manual (NI 322137B-01)

Things to do:
- Add DMA support (see mite.c and ni_pcidio.c for examples)
- Add commands (copy from ni_pcidio.c ?)
- Add interrupts
- Extend "Application possibilities" for the GPCT subdevice (eg. Time
Measurement, ...)
*/

#include <linux/comedidev.h>
#include "mite.h"

#define CTRS_PER_CHIP 4 // The number of counters per ni-tio chip
#define DATA_1B 0x1 // 1 byte = 8 bits data
#define DATA_2B 0x2 // 2 bytes = 16 bit data
#define DATA_4B 0x4 // 4 bytes = 32 bit data

/* See Register-Level Programmer Manual page 3.1 */
typedef enum
{
	G0InterruptAcknowledge,
	G0StatusRegister,
	G1InterruptAcknowledge,
	G1StatusRegister,
	G01StatusRegister,
	G0CommandRegister,
	G1CommandRegister,
	G0HWSaveRegister,
	G1HWSaveRegister,
	G0SWSaveRegister,
	G1SWSaveRegister,
	G0ModeRegister,
	G01JointStatus1Register,
	G1ModeRegister,
	G0LoadARegister,
	G01JointStatus2Register,
	G0LoadBRegister,
	G1LoadARegister,
	G1LoadBRegister,
	G0InputSelectRegister,
	G1InputSelectRegister,
	G01JointResetRegister,
	G0InterruptEnable,
	G1InterruptEnable,
	G0CountingModeRegister,
	G1CountingModeRegister,
	G0SecondGateRegister,
	G1SecondGateRegister,
	G0DMAConfigRegister,
	G0DMAStatusRegister,
	G1DMAConfigRegister,
	G1DMAStatusRegister,
	G2InterruptAcknowledge,
	G2StatusRegister,
	G3InterruptAcknowledge,
	G3StatusRegister,
	G23StatusRegister,
	G2CommandRegister,
	G3CommandRegister,
	G2HWSaveRegister,
	G3HWSaveRegister,
	G2SWSaveRegister,
	G3SWSaveRegister,
	G2ModeRegister,
	G23JointStatus1Register,
	G3ModeRegister,
	G2LoadARegister,
	G23JointStatus2Register,
	G2LoadBRegister,
	G3LoadARegister,
	G3LoadBRegister,
	G2InputSelectRegister,
	G3InputSelectRegister,
	G23JointResetRegister,
	G2InterruptEnable,
	G3InterruptEnable,
	G2CountingModeRegister,
	G3CountingModeRegister,
	G3SecondGateRegister,
	G2SecondGateRegister,
	G2DMAConfigRegister,
	G2DMAStatusRegister,
	G3DMAConfigRegister,
	G3DMAStatusRegister,
	ClockConfigRegister,
	IOConfigReg0_3,
	IOConfigReg4_7,
	IOConfigReg8_11,
	IOConfigReg12_15,
	IOConfigReg16_19,
	IOConfigReg20_23,
	IOConfigReg24_27,
	IOConfigReg28_31,
	IOConfigReg32_35,
	IOConfigReg36_39,
	STCDIOParallelInput,
	STCDIOOutput,
	STCDIOControl,
	STCDIOSerialInput,
	NumRegisters,
} NI_660xRegisters;
static inline int GxCommandRegister(int counter_channel)
{
	switch(counter_channel)
	{
	case 0:
		return G0CommandRegister;
		break;
	case 1:
		return G1CommandRegister;
		break;
	case 2:
		return G2CommandRegister;
		break;
	case 3:
		return G3CommandRegister;
		break;
	default:
		DPRINTK("ni_660x: bug!, line %i\n", __LINE__);
		break;
	}
	return 0;
}
static inline int GxCountingModeRegister(int counter_channel)
{
	switch(counter_channel)
	{
	case 0:
		return G0CountingModeRegister;
		break;
	case 1:
		return G1CountingModeRegister;
		break;
	case 2:
		return G2CountingModeRegister;
		break;
	case 3:
		return G3CountingModeRegister;
		break;
	default:
		DPRINTK("ni_660x: bug!, line %i\n", __LINE__);
		break;
	}
	return 0;
}
static inline int GxInputSelectRegister(int counter_channel)
{
	switch(counter_channel)
	{
	case 0:
		return G0InputSelectRegister;
		break;
	case 1:
		return G1InputSelectRegister;
		break;
	case 2:
		return G2InputSelectRegister;
		break;
	case 3:
		return G3InputSelectRegister;
		break;
	default:
		DPRINTK("ni_660x: bug!, line %i\n", __LINE__);
		break;
	}
	return 0;
}
static inline int GxxJointResetRegister(int counter_channel)
{
	switch(counter_channel)
	{
	case 0:
	case 1:
		return G01JointResetRegister;
		break;
	case 2:
	case 3:
		return G23JointResetRegister;
		break;
	default:
		DPRINTK("ni_660x.c: bug!, line %i", __LINE__);
		break;
	}
	return 0;
}
static inline int GxLoadARegister(int counter_channel)
{
	switch(counter_channel)
	{
	case 0:
		return G0LoadARegister;
		break;
	case 1:
		return G1LoadARegister;
		break;
	case 2:
		return G2LoadARegister;
		break;
	case 3:
		return G3LoadARegister;
		break;
	default:
		DPRINTK("ni_660x: bug!, line %i\n", __LINE__);
		break;
	}
	return 0;
}
static inline int GxLoadBRegister(int counter_channel)
{
	switch(counter_channel)
	{
	case 0:
		return G0LoadBRegister;
		break;
	case 1:
		return G1LoadBRegister;
		break;
	case 2:
		return G2LoadBRegister;
		break;
	case 3:
		return G3LoadBRegister;
		break;
	default:
		DPRINTK("ni_660x: bug!, line %i\n", __LINE__);
		break;
	}
	return 0;
}
static inline int GxModeRegister(int counter_channel)
{
	switch(counter_channel)
	{
	case 0:
		return G0ModeRegister;
		break;
	case 1:
		return G1ModeRegister;
		break;
	case 2:
		return G2ModeRegister;
		break;
	case 3:
		return G3ModeRegister;
		break;
	default:
		DPRINTK("ni_660x: bug!, line %i\n", __LINE__);
		break;
	}
	return 0;
}
static inline int GxSWSaveRegister(int counter_channel)
{
	switch(counter_channel)
	{
	case 0:
		return G0SWSaveRegister;
		break;
	case 1:
		return G1SWSaveRegister;
		break;
	case 2:
		return G2SWSaveRegister;
		break;
	case 3:
		return G3SWSaveRegister;
		break;
	default:
		DPRINTK("ni_660x: bug!, line %i\n", __LINE__);
		break;
	}
	return 0;
}
static inline int IOConfigReg(int chipset, int counter_channel)
{
	if(chipset == 0)
	{
		switch(counter_channel)
		{
		case 0:
			return IOConfigReg36_39;
			break;
		case 1:
			return IOConfigReg32_35;
			break;
		case 2:
			return IOConfigReg28_31;
			break;
		case 3:
			return IOConfigReg24_27;
			break;
		default:
			DPRINTK("ni_660x: bug!, line %i\n", __LINE__);
			break;
		}
	}else
	{
		switch(counter_channel)
		{
		case 0:
			return IOConfigReg20_23;
			break;
		case 1:
			return IOConfigReg16_19;
			break;
		case 2:
			return IOConfigReg12_15;
			break;
		case 3:
			return IOConfigReg8_11;
			break;
		default:
			DPRINTK("ni_660x: bug!, line %i\n", __LINE__);
			break;
		}
	}
	return 0;
}

typedef struct
{
	char *name; // Register Name
	int offset; // Offset from base address from GPCT chip
	int direction; // read or write, ie INSN_READ or ...
	int size; // 1 byte, 2 bytes, or 4 bytes
} NI_660xRegisterData;

#define INSN_RW 3 // Unused, could be used to check that register can
		  // both be written or read

const NI_660xRegisterData registerData[NumRegisters] =
{
	{"G0 Interrupt Acknowledge", 0x004, INSN_WRITE, DATA_2B},
	{"G0 Status Register", 0x004, INSN_READ, DATA_2B},
	{"G1 Interrupt Acknowledge", 0x006, INSN_WRITE, DATA_2B},
	{"G1 Status Register", 0x006, INSN_READ, DATA_2B},
	{"G01 Status Register ", 0x008, INSN_READ, DATA_2B},
	{"G0 Command Register", 0x00C, INSN_WRITE, DATA_2B},
	{"G1 Command Register", 0x00E, INSN_WRITE, DATA_2B},
	{"G0 HW Save Register", 0x010, INSN_READ, DATA_4B},
	{"G1 HW Save Register", 0x014, INSN_READ, DATA_4B},
	{"G0 SW Save Register", 0x018, INSN_READ, DATA_4B},
	{"G1 SW Save Register", 0x01C, INSN_READ, DATA_4B},
	{"G0 Mode Register", 0x034, INSN_WRITE, DATA_2B},
	{"G01 Joint Status 1 Register", 0x036, INSN_READ, DATA_2B},
	{"G1 Mode Register", 0x036, INSN_WRITE, DATA_2B},
	{"G0 Load A Register", 0x038, INSN_WRITE, DATA_4B},
	{"G01 Joint Status 2 Register", 0x03A, INSN_READ, DATA_2B},
	{"G0 Load B Register", 0x03C, INSN_WRITE, DATA_4B},
	{"G1 Load A Register", 0x040, INSN_WRITE, DATA_4B},
	{"G1 Load B Register", 0x044, INSN_WRITE, DATA_4B},
	{"G0 Input Select Register", 0x048, INSN_WRITE, DATA_2B},
	{"G1 Input Select Register", 0x04A, INSN_WRITE, DATA_2B},
	{"G01 Joint Reset Register", 0x090, INSN_WRITE, DATA_2B},
	{"G0 Interrupt Enable", 0x092, INSN_WRITE, DATA_2B},
	{"G1 Interrupt Enable", 0x096, INSN_WRITE, DATA_2B},
	{"G0 Counting Mode Register", 0x0B0, INSN_WRITE, DATA_2B},
	{"G1 Counting Mode Register", 0x0B2, INSN_WRITE, DATA_2B},
	{"G0 Second Gate Register", 0x0B4, INSN_WRITE, DATA_2B},
	{"G1 Second Gate Register", 0x0B6, INSN_WRITE, DATA_2B},
	{"G0 DMA Config Register", 0x0B8, INSN_WRITE, DATA_2B},
	{"G0 DMA Status Register", 0x0B8, INSN_READ, DATA_2B},
	{"G1 DMA Config Register", 0x0BA, INSN_WRITE, DATA_2B},
	{"G1 DMA Status Register", 0x0BA, INSN_READ, DATA_2B},
	{"G2 Interrupt Acknowledge", 0x104, INSN_WRITE, DATA_2B},
	{"G2 Status Register", 0x104, INSN_READ, DATA_2B},
	{"G3 Interrupt Acknowledge", 0x106, INSN_WRITE, DATA_2B},
	{"G3 Status Register", 0x106, INSN_READ, DATA_2B},
	{"G23 Status Register", 0x108, INSN_READ, DATA_2B},
	{"G2 Command Register", 0x10C, INSN_WRITE, DATA_2B},
	{"G3 Command Register", 0x10E, INSN_WRITE, DATA_2B},
	{"G2 HW Save Register", 0x110, INSN_READ, DATA_4B},
	{"G3 HW Save Register", 0x114, INSN_READ, DATA_4B},
	{"G2 SW Save Register", 0x118, INSN_READ, DATA_4B},
	{"G3 SW Save Register", 0x11C, INSN_READ, DATA_4B},
	{"G2 Mode Register", 0x134, INSN_WRITE, DATA_2B},
	{"G23 Joint Status 1 Register", 0x136, INSN_READ, DATA_2B},
	{"G3 Mode Register", 0x136, INSN_WRITE, DATA_2B},
	{"G2 Load A Register", 0x138, INSN_WRITE, DATA_4B},
	{"G23 Joint Status 2 Register", 0x13A, INSN_READ, DATA_2B},
	{"G2 Load B Register", 0x13C, INSN_WRITE, DATA_4B},
	{"G3 Load A Register", 0x140, INSN_WRITE, DATA_4B},
	{"G3 Load B Register", 0x144, INSN_WRITE, DATA_4B},
	{"G2 Input Select Register", 0x148, INSN_WRITE, DATA_2B},
	{"G3 Input Select Register", 0x14A, INSN_WRITE, DATA_2B},
	{"G23 Joint Reset Register", 0x190, INSN_WRITE, DATA_2B},
	{"G2 Interrupt Enable", 0x192, INSN_WRITE, DATA_2B},
	{"G3 Interrupt Enable", 0x196, INSN_WRITE, DATA_2B},
	{"G2 Counting Mode Register", 0x1B0, INSN_WRITE, DATA_2B},
	{"G3 Counting Mode Register", 0x1B2, INSN_WRITE, DATA_2B},
	{"G3 Second Gate Register", 0x1B6, INSN_WRITE, DATA_2B},
	{"G2 Second Gate Register", 0x1B4, INSN_WRITE, DATA_2B},

	{"G2 DMA Config Register", 0x1B8, INSN_WRITE, DATA_2B},
	{"G2 DMA Status Register", 0x1B8, INSN_READ, DATA_2B},
	{"G3 DMA Config Register", 0x1BA, INSN_WRITE, DATA_2B},
	{"G3 DMA Status Register", 0x1BA, INSN_READ, DATA_2B},
	{"Clock Config Register", 0x73C, INSN_WRITE, DATA_4B},
	{"IO Config Register 0-3", 0x77C, INSN_RW, DATA_4B}, // READWRITE
	{"IO Config Register 4-7", 0x780, INSN_RW, DATA_4B},
	{"IO Config Register 8-11", 0x784, INSN_RW, DATA_4B},
	{"IO Config Register 12-15", 0x788, INSN_RW, DATA_4B},
	{"IO Config Register 16-19", 0x78C, INSN_RW, DATA_4B},
	{"IO Config Register 20-23", 0x790, INSN_RW, DATA_4B},
	{"IO Config Register 24-27", 0x794, INSN_RW, DATA_4B},
	{"IO Config Register 28-31", 0x798, INSN_RW, DATA_4B},
	{"IO Config Register 32-35", 0x79C, INSN_RW, DATA_4B},
	{"IO Config Register 36-39", 0x7A0, INSN_RW, DATA_4B},
	{"STD DIO Parallel Input", 0x00E, INSN_READ, DATA_2B},
	{"STD DIO Output", 0x014, INSN_WRITE, DATA_2B},
	{"STD DIO Control", 0x016, INSN_WRITE, DATA_2B},
	{"STD DIO Serial Input", 0x038, INSN_READ, DATA_2B},
};


/* Different Application Classes for GPCT Subdevices */
/* The list is not exhaustive and needs discussion! */
typedef enum
{
	CountingAndTimeMeasurement,
	SinglePulseGeneration,
	PulseTrainGeneration,
	PositionMeasurement,
	Miscellaneous
} NI_660x_GPCT_AppClass;


/* Config struct for different GPCT subdevice Application Classes and
   their options
*/
typedef struct
{
	NI_660x_GPCT_AppClass App;
	/* Application dependent data, eg. for encoders, See mail Herman Bruyninckx
	<https://cvs.comedi.org/pipermail/comedi/2003-April/004381.html>
	and Adapted by Klaas Gadeyne with real-life experience :-)
	*/
	int data[6];
} NI_660x_GPCT_Config;

#define NI_660X_GPCT_MAXCHANNELS 8 // To avoid dyn. mem. allocation
NI_660x_GPCT_Config ni_660x_gpct_config[NI_660X_GPCT_MAXCHANNELS];

/* Some bits to write in different registers */
#define UpDownDown		0x0<<5 // always count down
#define UpDownUp		0x1<<5 // always count up
#define UpDownHardware		0x1<<6 // up/down depending on
				       // hardware pin
#define UpDownGate		0x3<<5 // depending on hardware
				       // internal GATE
#define Disarm			0x1<<4
#define Load			0x1<<2
#define Arm			0x1<<0

#define IndexPhaseLowLow	0x0<<5 // Index Pulse active when both
				       // A and B are low
#define IndexPhaseLowHigh	0x1<<5 // ...
#define IndexPhaseHighLow	0x2<<5
#define IndexPhaseHighHigh	0x3<<5

#define IndexMode		0x1<<4
// For quadrature encoders
#define CountingModeNormal	0x0<<0
#define CountingModeQuadX1	0x1<<0
#define CountingModeQuadX2	0x2<<0
#define CountingModeQuadX4	0x3<<0

// For 2-pulse encoders
#define CountingModeTwoPulse	0x4<<0
#define CountingModeSynchronous	0x6<<0

#define GateSelectPin38		0x1<<8 // Take internal time-based 20
				       // MHz clockx
#define SourceSelectTimebase1	0x0<<2
#define SourceSelectTimebase2	0x12<<2
#define SourceSelectTimebase3	0x1e<<2
#define GateSelectSource        0x0<<7

#define TriggerModeStartStop	0x0<<3
#define TriggerModeStopStart	0x1<<3
#define TriggerModeStart	0x2<<3
#define TriggerModeNotUsed	0x3<<3
#define GatingModeDisabled	0x0<<0
#define GatingModeLevel		0x1<<0
#define GatingModeRising	0x2<<0
#define GatingModeFalling	0x3<<0
#define G1Reset			0x1<<3
#define G0Reset			0x1<<2
#define G1Armed			0x1<<9
#define G0Armed			0x1<<8
// kind of ENABLE for the second counter
#define CounterSwap             0x1<<21
static inline int GxReset(int counter_channel)
{
	switch(counter_channel)
	{
	case 0:
		return G0Reset;
		break;
	case 1:
		return G1Reset;
		break;
	case 2:
		return G0Reset;
		break;
	case 3:
		return G1Reset;
		break;
	default:
		DPRINTK("ni_660x: bug!, line %i\n", __LINE__);
		break;
	}
	return 0;
}

#define LoadOnTC                0x1<<12

#define OutputIsTC              0x1<<8
#define OutputTogglesOnTC       0x2<<8
#define OutputTogglesOnTCorGate 0x3<<8

#define DisarmAtTCStopsCounting 0x1<<11
#define NoHardwareDisarm        0x0<<11
#define StopOn2ndTC             0x1<<6
#define LoadSourceSelectA       0x0<<7

#define SynchroniseGate         0x1<<8

// For pulse train generation
#define BankSwitchEnable        0x1<<12
#define BankSwitchOnGate        0x0<<11
#define BankSwitchOnSoftware    0x1<<11
#define BankSwitchStart         0x1<<10
#define ReloadSourceSwitching   0x1<<15

// ioconfigreg
/*pin index 0 corresponds to pin A in manual, index 1 is pin B, etc*/
static inline int pin_is_output(int pin_index)
{
	return 0x1 << (24 - 8 * pin_index);
}
static inline int pin_input_select(int pin_index, int input_selection)
{
	input_selection &= 0x7;
	return input_selection << (28 - 8 * pin_index);
}

// For configuring input pins
#define Digital_Filter_A_Is_Off             0x000<<28
#define Digital_Filter_A_Is_Timebase3       0x001<<28
#define Digital_Filter_A_Is_100             0x010<<28
#define Digital_Filter_A_Is_20              0x011<<28
#define Digital_Filter_A_Is_10              0x100<<28
#define Digital_Filter_A_Is_2               0x101<<28
#define Digital_Filter_A_Is_2_Timebase3     0x110<<28



// Offset of the GPCT chips from the base-adress of the card
const int GPCT_OFFSET[2] = {0x0,0x800}; /* First chip is at base-adress +
					   0x00, etc. */

/* Board description*/
typedef struct
{
	unsigned short dev_id; /* `lspci` will show you this */
	char *name;
	int n_ctrs; /* total number of counters */
	int cnt_bits; /* number of bits in each counter */
} ni_660x_board;

static ni_660x_board ni_660x_boards[] =
{
	{
		dev_id       : 0x2c60,
		name         : "PCI-6601",
		n_ctrs       : 1*CTRS_PER_CHIP,
		cnt_bits     : 32,
	},
	{
		dev_id       : 0x1310,
		name         : "PCI-6602",
		n_ctrs       : 2*CTRS_PER_CHIP,
		cnt_bits     : 32,
	},
};

static struct pci_device_id ni_660x_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_NATINST, 0x2c60, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1310, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, ni_660x_pci_table);

#define thisboard ((ni_660x_board *)dev->board_ptr)
/* initialized in ni_660x_find_device() */

typedef struct
{
	struct mite_struct *mite;
	int boardtype;
}ni_660x_private;

#define devpriv ((ni_660x_private *)dev->private)
#define n_ni_660x_boards (sizeof(ni_660x_boards)/sizeof(ni_660x_boards[0]))

static int ni_660x_attach(comedi_device *dev,comedi_devconfig *it);
static int ni_660x_detach(comedi_device *dev);

static comedi_driver driver_ni_660x=
{
	driver_name:	"ni_660x",
	module:		THIS_MODULE,
	attach:		ni_660x_attach,
	detach:		ni_660x_detach,
};

COMEDI_INITCLEANUP(driver_ni_660x);

static int ni_660x_find_device(comedi_device *dev,int bus,int slot);

/* Possible instructions for a GPCT */
static int ni_660x_GPCT_rinsn(comedi_device *dev,
			      comedi_subdevice *s,
			      comedi_insn *insn,
			      lsampl_t *data);
static int ni_660x_GPCT_insn_config(comedi_device *dev,
				    comedi_subdevice *s,
				    comedi_insn *insn,
				    lsampl_t *data);
static int ni_660x_GPCT_winsn(comedi_device *dev,
			      comedi_subdevice *s,
			      comedi_insn *insn,
			      lsampl_t * data);
// Internal triggering
/* Currently only used to stop the pulsegenerator */
static int ni_660x_GPCT_inttrig(comedi_device *dev,
				comedi_subdevice *subdev,
				unsigned int trig_num);

// NYI
static int ni_660x_GPCT_cmdtest(comedi_device *dev,comedi_subdevice *s,
				comedi_cmd *cmd);
static int ni_660x_GPCT_cmd(comedi_device *dev,comedi_subdevice *s);

/* Possible instructions for Digital IO */
static int ni_660x_dio_insn_config(comedi_device *dev,
				   comedi_subdevice *s,
				   comedi_insn *insn,
				   lsampl_t *data);
static int ni_660x_dio_insn_bits(comedi_device *dev,
				 comedi_subdevice *s,
				 comedi_insn *insn,
				 lsampl_t *data);

static int ni_660x_GPCT_cmdtest(comedi_device *dev,comedi_subdevice *s,
				comedi_cmd *cmd)
{
	DPRINTK("NI_660X: COMMANDS not implemented yet for GPCT\n");
	return -EINVAL;
}

static int ni_660x_GPCT_cmd(comedi_device *dev,comedi_subdevice *s)
{
	DPRINTK("NI_660X: COMMANDS not implemented yet for GPCT\n");
	return -EINVAL;
}

static int ni_660x_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret;

	printk("comedi%d: ni_660x: ",dev->minor);

	if ((ret=alloc_private(dev,sizeof(ni_660x_private))) < 0) return ret;

	ret = ni_660x_find_device(dev, it->options[0], it->options[1]);
	if (ret<0) return ret;

	ret = mite_setup(devpriv->mite);
	if (ret < 0) {
	printk("error setting up mite\n");
	return ret;
	}
	dev->iobase = mite_iobase(devpriv->mite);
	dev->board_name = thisboard->name;
	/* we don't support the interrupt yet */
	//dev->irq = mite_irq(devpriv->mite);

	printk(" %s ", dev->board_name);

	/* Currently there is 1 subdevice for the GPCT functionality,
	and another subdevice for DIO */
	dev->n_subdevices = 2;

	if (alloc_subdevices(dev,dev->n_subdevices)<0) return -ENOMEM;

	s=dev->subdevices+0;
	/* GENERAL-PURPOSE COUNTER/TIME (GPCT) */
	s->type         = COMEDI_SUBD_COUNTER;
	s->subdev_flags = SDF_READABLE | SDF_WRITABLE | SDF_LSAMPL;
	/* KG: What does SDF_LSAMPL (see multiq3.c) mean? */
	s->n_chan       = thisboard->n_ctrs;
	s->maxdata      = 0xffffffff; /* 32 bit counter */
	s->insn_read    = ni_660x_GPCT_rinsn;
	s->insn_config  = ni_660x_GPCT_insn_config;
	s->insn_write   = ni_660x_GPCT_winsn;

	/* Command are not implemented yet, however they are necessary to
	allocate the necessary memory for the comedi_async struct (used
	to trigger the GPCT in case of pulsegenerator function */
	s->do_cmd = ni_660x_GPCT_cmd;
	s->do_cmdtest = ni_660x_GPCT_cmdtest;
	//s->cancel = ni_660x_gpct_cancel;

	s=dev->subdevices+1;
	/* DIGITAL I/O SUBDEVICE */
	s->type	  = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE|SDF_WRITABLE;
	s->n_chan       = 8; // Only using 8 bits for now, instead of 32!!
	s->maxdata      = 1;
	s->range_table  = &range_digital;
	/* (Copied from skel.c) DIO devices are slightly special.  Although
	* it is possible to implement the insn_read/insn_write interface,
	* it is much more useful to applications if you implement the
	* insn_bits interface.  This allows packed reading/writing of the
	* DIO channels.  The comedi core can convert between insn_bits and
	* insn_read/write */
	// Not implemented yet
	s->insn_bits    = ni_660x_dio_insn_bits;
	s->insn_config  = ni_660x_dio_insn_config;
	s->io_bits      = 0;     /* all bits default to input */

	printk("attached\n");

	/* What does this "return value" mean?  Is this fixed by API??
	- skel_attach in skel.c returns 1;
	- ni_E_init in ni_mio_common.c returns "0" ... */
	return 1;
}


static int
ni_660x_detach(comedi_device *dev)
{
	printk("comedi%d: ni_660x: remove\n",dev->minor);

	if (dev->private && devpriv->mite)
		mite_unsetup(devpriv->mite);

	/* Free irq */

	if(dev->irq) comedi_free_irq(dev->irq,dev);

	/* Same question as with attach ... */
	return 0;
}

// Help function: Check what chipset the counter channel is on
static int GPCT_check_chipset_from_channel(comedi_device *dev, int channel)
{
	int chipset;
	if ( (channel >= 0) && (channel < CTRS_PER_CHIP) )
	{
		chipset = 0;
	}else if ( (channel >= CTRS_PER_CHIP) && (channel < thisboard->n_ctrs) )
	{
		chipset = 1;
		// DPRINTK("NI_660x: Moving to chipset 1\n");
	}else
	{
		DPRINTK("NI_660x: Channel specification not between limits\n");
		return -EINVAL;
	}
	return chipset;
}
int GPCT_check_counter_channel_from_subdev_channel(int channel)
{
	return channel % CTRS_PER_CHIP;
}

static int
ni_660x_GPCT_rinsn(comedi_device *dev, comedi_subdevice *s,
		   comedi_insn *insn, lsampl_t *data)
{
	int i; // counts the Data
	int subdev_channel = CR_CHAN(insn->chanspec);
	int counter_channel = GPCT_check_counter_channel_from_subdev_channel(subdev_channel);// Unpack chanspec
	int chipset = GPCT_check_chipset_from_channel(dev, subdev_channel);

	/* See Chapter 2.2 Reading Counter Value of the NI Register Level
		Programming Manual: "Reading counter values of armed counters".
		We need to take several measurements to be sure what the counter
		value is
	*/
	int tmpdata[2];
	unsigned long address;

	/* ============================================================ */
	/* 1 subdevice with 8 channels, differentation based on channel */
	DPRINTK("NI_660x: INSN_READ on channel %d\n", subdev_channel);

	// Check what Application of Counter this channel is configured for
	switch(ni_660x_gpct_config[subdev_channel].App)
	{
	case PositionMeasurement:
		// Check if (n > 0)
		if ( insn->n <= 0 )
		{
			DPRINTK("NI_660X: INSN_READ: n should be > 0\n");
			return -EINVAL;
		}
		// Now proceed with reading data
		address = dev->iobase
		+ GPCT_OFFSET[chipset] +
		registerData[GxSWSaveRegister(counter_channel)].offset;
		for ( i=0 ; i < insn->n ; i++ )
		{
			tmpdata[0] = readl(address);
			tmpdata[1] = readl(address);
			if (tmpdata[0] != tmpdata[1])
			{
				// In case they differ, the 3d measurement is the
				// correct value
				data[i] = readl(address);
			}
			// Otherwise, they are the same and the correct counter
			// value
			else data[i] = tmpdata[0];
		}
		return i;
		break;
	case SinglePulseGeneration: case PulseTrainGeneration:
		DPRINTK("NI_660x: INSN_READ irrelevant for this application\n");
		return -EINVAL;
		break;
	default: // The rest is not implemented yet :-)
		DPRINTK("NI_660x: INSN_READ: Functionality not implemented\n");
		return -EINVAL;
		break;
    }
}

static void
enable_chip(comedi_device *dev, int chipset)
{
	/* See P. 3.5 of the Register-Level Programming manual.  This
		bit has to be set, otherwise, you can't use the second chip.
	*/
	if ( chipset == 1)
	{
		writel(CounterSwap,dev->iobase + GPCT_OFFSET[chipset]
		+ registerData[ClockConfigRegister].offset);
	}else
	{
		writel(0x0,dev->iobase + GPCT_OFFSET[chipset]
		+ registerData[ClockConfigRegister].offset);
	}
}



static int
ni_660x_GPCT_insn_config(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int subdev_channel = CR_CHAN(insn->chanspec);// Unpack chanspec
	int chipset = GPCT_check_chipset_from_channel(dev, subdev_channel);
	int counter_channel = GPCT_check_counter_channel_from_subdev_channel(subdev_channel);

	DPRINTK("NI_660x: INSN_CONFIG: Configuring Channel %d\n", subdev_channel);
	enable_chip(dev, chipset);

	// Check what type of Counter the user requested, data[0] contains
	// the Application type
	switch(insn->data[0])
	{
	case INSN_CONFIG_GPCT_QUADRATURE_ENCODER:
		DPRINTK("NI_660x: INSN_CONFIG: Configuring Encoder\n");
		ni_660x_gpct_config[subdev_channel].App = PositionMeasurement;
		/* data[1] contains GPCT_X1, GPCT_X2 or GPCT_X4 */
		switch(insn->data[1])
		{
		case GPCT_X1:
			(ni_660x_gpct_config[subdev_channel]).data[0] = CountingModeQuadX1;
			break;
		case GPCT_X2:
			(ni_660x_gpct_config[subdev_channel]).data[0] = CountingModeQuadX2;
			break;
		case GPCT_X4:
			(ni_660x_gpct_config[subdev_channel]).data[0] = CountingModeQuadX4;
			break;
		default:
			DPRINTK("NI_660x: INSN_CONFIG: Wrong Counting mode\n");
			return -EINVAL;
			break;
		}
			// When to take into account the indexpulse:
		switch(insn->data[2])
		{
		case GPCT_IndexPhaseHighHigh:
			(ni_660x_gpct_config[subdev_channel]).data[1] = IndexPhaseHighHigh;
			break;
		case GPCT_IndexPhaseLowHigh:
			(ni_660x_gpct_config[subdev_channel]).data[1] = IndexPhaseLowHigh;
			break;
		case GPCT_IndexPhaseLowLow:
			(ni_660x_gpct_config[subdev_channel]).data[1] = IndexPhaseLowLow;
			break;
		case GPCT_IndexPhaseHighLow:
			(ni_660x_gpct_config[subdev_channel]).data[1] = IndexPhaseHighLow;
			break;
		default:
			DPRINTK("NI_660x: INSN_CONFIG: Wrong value for taking into account index pulse\n");
			return -EINVAL;
			break;
		}
		// Take into account the index pulse?
		if(insn->data[3] == GPCT_RESET_COUNTER_ON_INDEX)
			(ni_660x_gpct_config[subdev_channel]).data[2] = IndexMode;
		else
			(ni_660x_gpct_config[subdev_channel]).data[2] = 0;

		// Reset the counter
		writew(GxReset(counter_channel),dev->iobase + GPCT_OFFSET[chipset]
		+ registerData[GxxJointResetRegister(counter_channel)].offset);
		// Disarm
		writew(Disarm,dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		// Put 0 as initial counter value in the load register
		writel(0x0,dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxLoadARegister(counter_channel)].offset);
		// Load (latch) this value into the counter
		writew(Load,dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		/* - Set Counting Mode into GPCT_X1 / 2 / 4 (as set by user
		- Take into account Z pulse (index pulse) only when both
		channel A and B are high (arbitrary choice)
		*/
		writew(((ni_660x_gpct_config[subdev_channel]).data[0] |
			(ni_660x_gpct_config[subdev_channel]).data[1] |
			(ni_660x_gpct_config[subdev_channel]).data[1] ),
			dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCountingModeRegister(counter_channel)].offset);
		// Put counter in input mode
		// Not necessary since this is the default ...
		/*  writel(Counter_A_Is_Input, dev->iobase
			+ GPCT_OFFSET[chipset]
			+ registerData[IOConfigReg36_39].offset);
		*/
		// Arm the counter and put it into Hardware UpDown mode (depending
		// on the UP/DOWN IO pin: 0 = down
		writew(UpDownHardware|Arm,dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		break;
	case INSN_CONFIG_GPCT_SINGLE_PULSE_GENERATOR:
		DPRINTK("NI_660x: INSN_CONFIG: Configuring SPG\n");
		ni_660x_gpct_config[subdev_channel].App = SinglePulseGeneration;
		/* data[1] contains the PULSE_WIDTH
		data[2] contains the PULSE_DELAY
		@pre PULSE_WIDTH > 0 && PULSE_DELAY > 0
		The above periods must be expressed as a multiple of the
		pulse frequency on the selected source, see the
		Register-Level Programmer Manual p2-11 (pulse generation)
		*/
		if(insn->data[2] > 1 && insn->data[1] > 1)
		{
			(ni_660x_gpct_config[subdev_channel]).data[0] = insn->data[1];
			(ni_660x_gpct_config[subdev_channel]).data[1] = insn->data[2];
		}else
		{
			DPRINTK("NI_660x: INSN_CONFIG: SPG: Problem with Pulse params\n");
			return -EINVAL;
		}
		// Reset the counter
		writew(GxReset(counter_channel), dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxxJointResetRegister(counter_channel)].offset);
		// Disarm
		writew(Disarm, dev->iobase + GPCT_OFFSET[chipset]
		+ registerData[GxCommandRegister(counter_channel)].offset);
		/* Put PULSE_DELAY as initial counter value into load
		register A */
		writel((ni_660x_gpct_config[subdev_channel]).data[1], dev->iobase
		+ GPCT_OFFSET[chipset]
		+ registerData[GxLoadARegister(counter_channel)].offset);
		// Load (latch) this value into the counter
		writew(Load,dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		// Now Put PULSE_WIDTH in the LOAD register A
		writel((ni_660x_gpct_config[subdev_channel]).data[0],dev->iobase
			+ GPCT_OFFSET[chipset]
			+ registerData[GxLoadARegister(counter_channel)].offset);
		// Put Source input to internal 20 MHz clock
		/* ==================================================
		TODO: MAKE THIS A DATA FIELD!! to allow different clocks
			(See TODO)
		================================================== */
		writew(SourceSelectTimebase1, dev->iobase
			+ GPCT_OFFSET[chipset]
			+ registerData[GxInputSelectRegister(counter_channel)].offset);
		/* Choose to Load on reaching TC and
		Change State of G_OUT on TC (Terminal Count)
		Stop counting after second TC
		Choose Load register A to load from */
		writew(LoadOnTC | OutputTogglesOnTC | StopOn2ndTC | LoadSourceSelectA,
			dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxModeRegister(counter_channel)].offset);
		// Configure Counter for output
		writel(pin_is_output(0), dev->iobase
			+ GPCT_OFFSET[chipset]
			+ registerData[IOConfigReg(chipset, counter_channel)].offset);
	case INSN_CONFIG_GPCT_PULSE_TRAIN_GENERATOR:
		DPRINTK("NI_660x: INSN_CONFIG: PTG linking inttrig\n");
		s->async->inttrig = ni_660x_GPCT_inttrig;
		DPRINTK("NI_660x: INSN_CONFIG: Configuring PTG\n");

		ni_660x_gpct_config[subdev_channel].App = PulseTrainGeneration;

		/* data[1] contains the PULSE_WIDTH
		data[2] contains the PULSE_PERIOD
		@pre PULSE_PERIOD > PULSE_WIDTH > 0
		The above periods must be expressed as a multiple of the
		pulse frequency on the selected source, see the
		Register-Level Programmer Manual p2-11 (pulse generation)
		*/
		if ( (insn->data[2] > insn->data[1]) && (insn->data[1] > 0 ) )
		{
			(ni_660x_gpct_config[subdev_channel]).data[0] = insn->data[1];
			(ni_660x_gpct_config[subdev_channel]).data[1] = insn->data[2];
		}else
		{
			DPRINTK("%d \t %d\n",insn->data[1],insn->data[2]);
			DPRINTK("NI_660x: INSN_CONFIG: PTG: Problem with Pulse params\n");
			return -EINVAL;
		}
		// Reset the counter
		writew(GxReset(counter_channel),dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxxJointResetRegister(counter_channel)].offset);
		// Disarm counter
		writew(Disarm,dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		// Put PULSE_WIDTH as initial counter value into load register A
		writel((ni_660x_gpct_config[subdev_channel]).data[0],dev->iobase
			+ GPCT_OFFSET[chipset]
			+ registerData[GxLoadARegister(counter_channel)].offset);
		// Load (latch) this value into the counter
		writew(Load,dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		// Now Put (PULSE_PERIOD - PULSE_WIDTH) in the load register B
		writel((ni_660x_gpct_config[subdev_channel]).data[1]
			- (ni_660x_gpct_config[subdev_channel]).data[0],
			dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxLoadBRegister(counter_channel)].offset);
		// Put Source input to internal 20 MHz clock
		/* ==================================================
		TODO: MAKE THIS A DATA FIELD!! to allow different clocks
			(See TODO)
		================================================== */
		writew(SourceSelectTimebase1,dev->iobase
			+ GPCT_OFFSET[chipset]
			+ registerData[GxInputSelectRegister(counter_channel)].offset);
		/* Switch between Load registers everytime
		Choose to Load on reaching TC and
		Change State of G_OUT on TC (Terminal Count)
		Choose Load register A to load from */
		writew(ReloadSourceSwitching|LoadOnTC|OutputTogglesOnTC,
			dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxModeRegister(counter_channel)].offset);
		// Configure Counter for output
		writel(pin_is_output(0), dev->iobase
			+ GPCT_OFFSET[chipset]
			+ registerData[IOConfigReg(chipset, counter_channel)].offset);
		// Arm the counter and tell it to count down
		writew(Arm|UpDownDown,dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		break;
	default:
		DPRINTK("NI_660x: unsupported insn_config\n");
		return -EINVAL;
		break;
	}

	return insn->n;
}

static int ni_660x_GPCT_winsn(comedi_device *dev,
			      comedi_subdevice *s,
			      comedi_insn *insn,
			      lsampl_t * data)
{
	int subdev_channel = CR_CHAN(insn->chanspec);// Unpack chanspec
	int chipset = GPCT_check_chipset_from_channel(dev, subdev_channel);
	int counter_channel = GPCT_check_counter_channel_from_subdev_channel(subdev_channel);

	DPRINTK("NI_660X: INSN_WRITE on channel %d\n", subdev_channel);
	// Check what Application of Counter this channel is configured for
	switch(ni_660x_gpct_config[subdev_channel].App)
	{
	case PositionMeasurement:
		// Disarm the counter
		writew(Disarm, dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		// Write the value into the load register
		writel(*data, dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxLoadARegister(counter_channel)].offset);
		// Latch the value into the counter
		writew(Load, dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		// Arm the counter again and put UpDownHardware in!
		writew(UpDownHardware|Arm, dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		break;
	case SinglePulseGeneration:
		DPRINTK("NI_660X: INSN_WRITE: SPG: Arming the counter\n");
		// Tell the counter to count down and arm
		writew(Arm|UpDownDown, dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxCommandRegister(counter_channel)].offset);
		break;
	case PulseTrainGeneration:
	/* data[0] contains the PULSE_WIDTH
	 data[1] contains the PULSE_PERIOD
	 @pre PULSE_PERIOD > PULSE_WIDTH > 0
	 The above periods must be expressed as a multiple of the
	 pulse frequency on the selected source, see the
	 Register-Level Programmer Manual p2-11 (pulse generation)
	*/
		if ( (insn->data[1] > insn->data[0]) && (insn->data[0] > 0 ) )
		{
			(ni_660x_gpct_config[subdev_channel]).data[0] = insn->data[0];
			(ni_660x_gpct_config[subdev_channel]).data[1] = insn->data[1];
		}
		else
		{
			DPRINTK("%d \t %d\n",insn->data[1],insn->data[2]);
			DPRINTK("NI_660x: INSN_WRITE: PTG: Problem with Pulse params\n");
			return -EINVAL;
		}
		// Put PULSE_WIDTH as initial counter value into load register A
		writel((ni_660x_gpct_config[subdev_channel]).data[0],dev->iobase
			+ GPCT_OFFSET[chipset]
			+ registerData[GxLoadARegister(counter_channel)].offset);
		// Put (PULSE_PERIOD - PULSE_WIDTH) in the load register B
		writel(  (ni_660x_gpct_config[subdev_channel]).data[1]
			- (ni_660x_gpct_config[subdev_channel]).data[0],
			dev->iobase + GPCT_OFFSET[chipset]
			+ registerData[GxLoadBRegister(counter_channel)].offset);
		break;
	default: // Impossible
		DPRINTK("NI_660X: INSN_WRITE: Functionality %d not implemented yet\n",
		ni_660x_gpct_config[subdev_channel].App);
		return -EINVAL;
		break;
	}
  // return the number of samples written
  return insn->n ;
}

/* Trigger instruction is currently only used to STOP the
   pulsegenerator
*/
static int ni_660x_GPCT_inttrig(comedi_device *dev,
				comedi_subdevice *subdev,
				unsigned int trig_num)
{
	int subdev_channel = trig_num;
	int chipset = GPCT_check_chipset_from_channel(dev, subdev_channel);
	int counter_channel = GPCT_check_counter_channel_from_subdev_channel(subdev_channel);

	DPRINTK("Triggering channel %d\n", subdev_channel);
  enable_chip(dev, chipset);

	// Reset the counter
	writew(GxReset(counter_channel),dev->iobase + GPCT_OFFSET[chipset]
		+ registerData[GxxJointResetRegister(counter_channel)].offset);
	return 0;
}

static int
ni_660x_find_device(comedi_device *dev, int bus, int slot)
{
  struct mite_struct *mite;
  int i;

  for (mite=mite_devices; mite; mite=mite->next) {
    if (mite->used) continue;
    if (bus || slot) {
      if (bus!=mite->pcidev->bus->number ||
	  slot!=PCI_SLOT(mite->pcidev->devfn)) continue;
    }

    for (i=0; i<n_ni_660x_boards; i++) {
      if (mite_device_id(mite)==ni_660x_boards[i].dev_id) {
	dev->board_ptr=ni_660x_boards+i;
	devpriv->mite=mite;
	return 0;
      }
    }
  }
  printk("no device found\n");
  mite_list_devices();
  return -EIO;
}


static int ni_660x_dio_insn_bits(comedi_device *dev,
				 comedi_subdevice *s,
				 comedi_insn *insn,
				 lsampl_t *data)
{
  if(insn->n!=2)return -EINVAL;
  /* The insn data is a write_mask in data[0] and the new data
   * in data[1], each channel corresponding to a bit. */

  // Check if we have to write some bits
  if(data[0])
    {
      // Copied from skel.c, unverified
      s->state &= ~data[0];
      s->state |= data[0]&data[1];
      /* Write out the new digital output lines */
      /* Check if data < n_chan ?? */
      writew(s->state,dev->iobase + registerData[STCDIOOutput].offset);
    }
  /* on return, data[1] contains the value of the digital
   * input and output lines. */
  data[1]=readw(dev->iobase + registerData[STCDIOParallelInput].offset);
  return 2;
}

static int ni_660x_dio_insn_config(comedi_device *dev,
				   comedi_subdevice *s,
				   comedi_insn *insn,
				   lsampl_t *data)
{
  int chan=CR_CHAN(insn->chanspec);

  if(insn->n!=1)return -EINVAL;

  /* The input or output configuration of each digital line is
   * configured by a special insn_config instruction.  chanspec
   * contains the channel to be changed, and data[0] contains the
   * value COMEDI_INPUT or COMEDI_OUTPUT. */

  if(data[0]==COMEDI_OUTPUT)
    {
      s->io_bits |= 1<<chan;
    }
  else
    {
      s->io_bits &= ~(1<<chan);
    }
  // No GPCT_OFFSET[chipset] offset here??
  writew(s->io_bits,dev->iobase + registerData[STCDIOControl].offset);
  /* Should we do also something with the IO configuration registers,
     see p 3-38 of register level prog. manual
  */

  return 1;
  return -EINVAL;
}


