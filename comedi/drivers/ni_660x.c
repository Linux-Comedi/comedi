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
*/

#include <linux/comedidev.h>
#include "mite.h"
#include "ni_tio.h"

#define CTRS_PER_CHIP 4 // The number of counters per ni-tio chip
#define NUM_PFI_CHANNELS 40

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
	G0AutoincrementRegister,
	G1AutoincrementRegister,
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
	G2AutoincrementRegister,
	G3AutoincrementRegister,
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
	DMAConfigRegister,	// XXX will need to use this according to mite channel used
	IOConfigReg0_1,
	IOConfigReg2_3,
	IOConfigReg4_5,
	IOConfigReg6_7,
	IOConfigReg8_9,
	IOConfigReg10_11,
	IOConfigReg12_13,
	IOConfigReg14_15,
	IOConfigReg16_17,
	IOConfigReg18_19,
	IOConfigReg20_21,
	IOConfigReg22_23,
	IOConfigReg24_25,
	IOConfigReg26_27,
	IOConfigReg28_29,
	IOConfigReg30_31,
	IOConfigReg32_33,
	IOConfigReg34_35,
	IOConfigReg36_37,
	IOConfigReg38_39,
	STCDIOParallelInput,
	STCDIOOutput,
	STCDIOControl,
	STCDIOSerialInput,
	DIO32Input,
	DIO32Output,
	NumRegisters,
} NI_660x_Register;

static inline unsigned IOConfigReg(unsigned pfi_channel)
{
	unsigned reg = IOConfigReg0_1 + pfi_channel / 2;
	BUG_ON(reg > IOConfigReg38_39);
	return reg;
}

enum ni_660x_register_width
{
	DATA_1B,
	DATA_2B,
	DATA_4B
};

enum ni_660x_register_direction
{
	NI_660x_READ,
	NI_660x_WRITE,
	NI_660x_READ_WRITE
};

typedef struct
{
	const char *name; // Register Name
	int offset; // Offset from base address from GPCT chip
	enum ni_660x_register_direction direction;
	enum ni_660x_register_width size; // 1 byte, 2 bytes, or 4 bytes
} NI_660xRegisterData;


static const NI_660xRegisterData registerData[NumRegisters] =
{
	{"G0 Interrupt Acknowledge", 0x004, NI_660x_WRITE, DATA_2B},
	{"G0 Status Register", 0x004, NI_660x_READ, DATA_2B},
	{"G1 Interrupt Acknowledge", 0x006, NI_660x_WRITE, DATA_2B},
	{"G1 Status Register", 0x006, NI_660x_READ, DATA_2B},
	{"G01 Status Register ", 0x008, NI_660x_READ, DATA_2B},
	{"G0 Command Register", 0x00C, NI_660x_WRITE, DATA_2B},
	{"G1 Command Register", 0x00E, NI_660x_WRITE, DATA_2B},
	{"G0 HW Save Register", 0x010, NI_660x_READ, DATA_4B},
	{"G1 HW Save Register", 0x014, NI_660x_READ, DATA_4B},
	{"G0 SW Save Register", 0x018, NI_660x_READ, DATA_4B},
	{"G1 SW Save Register", 0x01C, NI_660x_READ, DATA_4B},
	{"G0 Mode Register", 0x034, NI_660x_WRITE, DATA_2B},
	{"G01 Joint Status 1 Register", 0x036, NI_660x_READ, DATA_2B},
	{"G1 Mode Register", 0x036, NI_660x_WRITE, DATA_2B},
	{"G0 Load A Register", 0x038, NI_660x_WRITE, DATA_4B},
	{"G01 Joint Status 2 Register", 0x03A, NI_660x_READ, DATA_2B},
	{"G0 Load B Register", 0x03C, NI_660x_WRITE, DATA_4B},
	{"G1 Load A Register", 0x040, NI_660x_WRITE, DATA_4B},
	{"G1 Load B Register", 0x044, NI_660x_WRITE, DATA_4B},
	{"G0 Input Select Register", 0x048, NI_660x_WRITE, DATA_2B},
	{"G1 Input Select Register", 0x04A, NI_660x_WRITE, DATA_2B},
	{"G0 Autoincrement Register", 0x088, NI_660x_WRITE, DATA_2B},
	{"G1 Autoincrement Register", 0x08A, NI_660x_WRITE, DATA_2B},
	{"G01 Joint Reset Register", 0x090, NI_660x_WRITE, DATA_2B},
	{"G0 Interrupt Enable", 0x092, NI_660x_WRITE, DATA_2B},
	{"G1 Interrupt Enable", 0x096, NI_660x_WRITE, DATA_2B},
	{"G0 Counting Mode Register", 0x0B0, NI_660x_WRITE, DATA_2B},
	{"G1 Counting Mode Register", 0x0B2, NI_660x_WRITE, DATA_2B},
	{"G0 Second Gate Register", 0x0B4, NI_660x_WRITE, DATA_2B},
	{"G1 Second Gate Register", 0x0B6, NI_660x_WRITE, DATA_2B},
	{"G0 DMA Config Register", 0x0B8, NI_660x_WRITE, DATA_2B},
	{"G0 DMA Status Register", 0x0B8, NI_660x_READ, DATA_2B},
	{"G1 DMA Config Register", 0x0BA, NI_660x_WRITE, DATA_2B},
	{"G1 DMA Status Register", 0x0BA, NI_660x_READ, DATA_2B},
	{"G2 Interrupt Acknowledge", 0x104, NI_660x_WRITE, DATA_2B},
	{"G2 Status Register", 0x104, NI_660x_READ, DATA_2B},
	{"G3 Interrupt Acknowledge", 0x106, NI_660x_WRITE, DATA_2B},
	{"G3 Status Register", 0x106, NI_660x_READ, DATA_2B},
	{"G23 Status Register", 0x108, NI_660x_READ, DATA_2B},
	{"G2 Command Register", 0x10C, NI_660x_WRITE, DATA_2B},
	{"G3 Command Register", 0x10E, NI_660x_WRITE, DATA_2B},
	{"G2 HW Save Register", 0x110, NI_660x_READ, DATA_4B},
	{"G3 HW Save Register", 0x114, NI_660x_READ, DATA_4B},
	{"G2 SW Save Register", 0x118, NI_660x_READ, DATA_4B},
	{"G3 SW Save Register", 0x11C, NI_660x_READ, DATA_4B},
	{"G2 Mode Register", 0x134, NI_660x_WRITE, DATA_2B},
	{"G23 Joint Status 1 Register", 0x136, NI_660x_READ, DATA_2B},
	{"G3 Mode Register", 0x136, NI_660x_WRITE, DATA_2B},
	{"G2 Load A Register", 0x138, NI_660x_WRITE, DATA_4B},
	{"G23 Joint Status 2 Register", 0x13A, NI_660x_READ, DATA_2B},
	{"G2 Load B Register", 0x13C, NI_660x_WRITE, DATA_4B},
	{"G3 Load A Register", 0x140, NI_660x_WRITE, DATA_4B},
	{"G3 Load B Register", 0x144, NI_660x_WRITE, DATA_4B},
	{"G2 Input Select Register", 0x148, NI_660x_WRITE, DATA_2B},
	{"G3 Input Select Register", 0x14A, NI_660x_WRITE, DATA_2B},
	{"G2 Autoincrement Register", 0x188, NI_660x_WRITE, DATA_2B},
	{"G3 Autoincrement Register", 0x18A, NI_660x_WRITE, DATA_2B},
	{"G23 Joint Reset Register", 0x190, NI_660x_WRITE, DATA_2B},
	{"G2 Interrupt Enable", 0x192, NI_660x_WRITE, DATA_2B},
	{"G3 Interrupt Enable", 0x196, NI_660x_WRITE, DATA_2B},
	{"G2 Counting Mode Register", 0x1B0, NI_660x_WRITE, DATA_2B},
	{"G3 Counting Mode Register", 0x1B2, NI_660x_WRITE, DATA_2B},
	{"G3 Second Gate Register", 0x1B6, NI_660x_WRITE, DATA_2B},
	{"G2 Second Gate Register", 0x1B4, NI_660x_WRITE, DATA_2B},
	{"G2 DMA Config Register", 0x1B8, NI_660x_WRITE, DATA_2B},
	{"G2 DMA Status Register", 0x1B8, NI_660x_READ, DATA_2B},
	{"G3 DMA Config Register", 0x1BA, NI_660x_WRITE, DATA_2B},
	{"G3 DMA Status Register", 0x1BA, NI_660x_READ, DATA_2B},
	{"Clock Config Register", 0x73C, NI_660x_WRITE, DATA_4B},
	{"DMA Configuration Register", 0x76c, NI_660x_WRITE, DATA_4B},
	{"IO Config Register 0-1", 0x77C, NI_660x_READ_WRITE, DATA_2B}, // READWRITE
	{"IO Config Register 2-3", 0x77E, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 4-5", 0x780, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 6-7", 0x782, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 8-9", 0x784, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 10-11", 0x786, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 12-13", 0x788, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 14-15", 0x78A, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 16-17", 0x78C, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 18-19", 0x78E, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 20-21", 0x790, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 22-23", 0x792, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 24-25", 0x794, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 26-27", 0x796, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 28-29", 0x798, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 30-31", 0x79A, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 32-33", 0x79C, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 34-35", 0x79E, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 36-37", 0x7A0, NI_660x_READ_WRITE, DATA_2B},
	{"IO Config Register 38-39", 0x7A2, NI_660x_READ_WRITE, DATA_2B},
	{"STD DIO Parallel Input", 0x00E, NI_660x_READ, DATA_2B},
	{"STD DIO Output", 0x014, NI_660x_WRITE, DATA_2B},
	{"STD DIO Control", 0x016, NI_660x_WRITE, DATA_2B},
	{"STD DIO Serial Input", 0x038, NI_660x_READ, DATA_2B},
	{"32 bit Digital Input", 0x414, NI_660x_READ, DATA_4B},
	{"32 bit Digital Output", 0x414, NI_660x_WRITE, DATA_4B}
};

#define GateSelectPin38		0x1<<8 // Take internal time-based 20

// kind of ENABLE for the second counter
#define CounterSwap             0x1<<21

// ioconfigreg
static inline unsigned pfi_output_select_mask(unsigned pfi_channel)
{
	return 0x3 << (8 * (pfi_channel % 2));
}
static inline unsigned pfi_output_select_bits(unsigned pfi_channel, unsigned output_select)
{
	return (output_select & 0x3) << (8 * (pfi_channel % 2));
}
static inline unsigned pfi_input_select_mask(unsigned pfi_channel)
{
	return 0x7 << (4 + 8 * (pfi_channel % 2));
}
static inline unsigned pfi_input_select_bits(unsigned pfi_channel, unsigned input_select)
{
	return (input_select & 0x7) << (4 + 8 * (pfi_channel % 2));
}

// Offset of the GPCT chips from the base-adress of the card
static const unsigned GPCT_OFFSET[2] = {0x0, 0x800}; /* First chip is at base-adress +
					   0x00, etc. */

/* Board description*/
typedef struct
{
	unsigned short dev_id; /* `lspci` will show you this */
	const char *name;
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
#define NI_660X_MAX_NUM_COUNTERS 8

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
	struct ni_gpct_device *counter_dev;
	uint64_t pfi_direction_bits;
}ni_660x_private;

#define devpriv ((ni_660x_private *)dev->private)
#define n_ni_660x_boards (sizeof(ni_660x_boards)/sizeof(ni_660x_boards[0]))

static int ni_660x_attach(comedi_device *dev,comedi_devconfig *it);
static int ni_660x_detach(comedi_device *dev);
static void init_tio_chip(comedi_device *dev, int chipset);

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

static NI_660x_Register ni_gpct_to_660x_register(enum ni_gpct_register reg)
{
	NI_660x_Register ni_660x_register;
	switch(reg)
	{
	case NITIO_G0_Autoincrement_Reg:
		ni_660x_register = G0AutoincrementRegister;
		break;
	case NITIO_G1_Autoincrement_Reg:
		ni_660x_register = G1AutoincrementRegister;
		break;
	case NITIO_G2_Autoincrement_Reg:
		ni_660x_register = G2AutoincrementRegister;
		break;
	case NITIO_G3_Autoincrement_Reg:
		ni_660x_register = G3AutoincrementRegister;
		break;
	case NITIO_G0_Command_Reg:
		ni_660x_register = G0CommandRegister;
		break;
	case NITIO_G1_Command_Reg:
		ni_660x_register = G1CommandRegister;
		break;
	case NITIO_G2_Command_Reg:
		ni_660x_register = G2CommandRegister;
		break;
	case NITIO_G3_Command_Reg:
		ni_660x_register = G3CommandRegister;
		break;
	case NITIO_G0_HW_Save_Reg:
		ni_660x_register = G0HWSaveRegister;
		break;
	case NITIO_G1_HW_Save_Reg:
		ni_660x_register = G1HWSaveRegister;
		break;
	case NITIO_G2_HW_Save_Reg:
		ni_660x_register = G2HWSaveRegister;
		break;
	case NITIO_G3_HW_Save_Reg:
		ni_660x_register = G3HWSaveRegister;
		break;
	case NITIO_G0_SW_Save_Reg:
		ni_660x_register = G0SWSaveRegister;
		break;
	case NITIO_G1_SW_Save_Reg:
		ni_660x_register = G1SWSaveRegister;
		break;
	case NITIO_G2_SW_Save_Reg:
		ni_660x_register = G2SWSaveRegister;
		break;
	case NITIO_G3_SW_Save_Reg:
		ni_660x_register = G3SWSaveRegister;
		break;
	case NITIO_G0_Mode_Reg:
		ni_660x_register = G0ModeRegister;
		break;
	case NITIO_G1_Mode_Reg:
		ni_660x_register = G1ModeRegister;
		break;
	case NITIO_G2_Mode_Reg:
		ni_660x_register = G2ModeRegister;
		break;
	case NITIO_G3_Mode_Reg:
		ni_660x_register = G3ModeRegister;
		break;
	case NITIO_G0_LoadA_Reg:
		ni_660x_register = G0LoadARegister;
		break;
	case NITIO_G1_LoadA_Reg:
		ni_660x_register = G1LoadARegister;
		break;
	case NITIO_G2_LoadA_Reg:
		ni_660x_register = G2LoadARegister;
		break;
	case NITIO_G3_LoadA_Reg:
		ni_660x_register = G3LoadARegister;
		break;
	case NITIO_G0_LoadB_Reg:
		ni_660x_register = G0LoadBRegister;
		break;
	case NITIO_G1_LoadB_Reg:
		ni_660x_register = G1LoadBRegister;
		break;
	case NITIO_G2_LoadB_Reg:
		ni_660x_register = G2LoadBRegister;
		break;
	case NITIO_G3_LoadB_Reg:
		ni_660x_register = G3LoadBRegister;
		break;
	case NITIO_G0_Input_Select_Reg:
		ni_660x_register = G0InputSelectRegister;
		break;
	case NITIO_G1_Input_Select_Reg:
		ni_660x_register = G1InputSelectRegister;
		break;
	case NITIO_G2_Input_Select_Reg:
		ni_660x_register = G2InputSelectRegister;
		break;
	case NITIO_G3_Input_Select_Reg:
		ni_660x_register = G3InputSelectRegister;
		break;
	case NITIO_G01_Status_Reg:
		ni_660x_register = G01StatusRegister;
		break;
	case NITIO_G23_Status_Reg:
		ni_660x_register = G23StatusRegister;
		break;
	case NITIO_G01_Joint_Reset_Reg:
		ni_660x_register = G01JointResetRegister;
		break;
	case NITIO_G23_Joint_Reset_Reg:
		ni_660x_register = G23JointResetRegister;
		break;
	case NITIO_G01_Joint_Status1_Reg:
		ni_660x_register = G01JointStatus1Register;
		break;
	case NITIO_G23_Joint_Status1_Reg:
		ni_660x_register = G23JointStatus1Register;
		break;
	case NITIO_G01_Joint_Status2_Reg:
		ni_660x_register = G01JointStatus2Register;
		break;
	case NITIO_G23_Joint_Status2_Reg:
		ni_660x_register = G23JointStatus2Register;
		break;
	case NITIO_G0_Counting_Mode_Reg:
		ni_660x_register = G0CountingModeRegister;
		break;
	case NITIO_G1_Counting_Mode_Reg:
		ni_660x_register = G1CountingModeRegister;
		break;
	case NITIO_G2_Counting_Mode_Reg:
		ni_660x_register = G2CountingModeRegister;
		break;
	case NITIO_G3_Counting_Mode_Reg:
		ni_660x_register = G3CountingModeRegister;
		break;
	case NITIO_G0_Second_Gate_Reg:
		ni_660x_register = G0SecondGateRegister;
		break;
	case NITIO_G1_Second_Gate_Reg:
		ni_660x_register = G1SecondGateRegister;
		break;
	case NITIO_G2_Second_Gate_Reg:
		ni_660x_register = G2SecondGateRegister;
		break;
	case NITIO_G3_Second_Gate_Reg:
		ni_660x_register = G3SecondGateRegister;
		break;
	default:
		rt_printk("%s: unhandled register 0x%x in switch.\n", __FUNCTION__, reg);
		BUG();
		return 0;
		break;
	}
	return ni_660x_register;
}

static void ni_gpct_write_register(struct ni_gpct *counter, unsigned bits, enum ni_gpct_register reg)
{
	NI_660x_Register ni_660x_register = ni_gpct_to_660x_register(reg);
	comedi_device *dev = counter->counter_dev->dev;
	void * const write_address = devpriv->mite->daq_io_addr + GPCT_OFFSET[counter->chip_index] + registerData[ni_660x_register].offset;

	switch(registerData[ni_660x_register].size)
	{
	case DATA_2B:
		writew(bits, write_address);
		break;
	case DATA_4B:
		writel(bits, write_address);
		break;
	default:
		rt_printk("%s: %s: bug! unhandled case = 0x%x in switch.\n", __FILE__, __FUNCTION__, reg);
		BUG();
		break;
	}
}

static unsigned ni_gpct_read_register(struct ni_gpct *counter, enum ni_gpct_register reg)
{
	NI_660x_Register ni_660x_register = ni_gpct_to_660x_register(reg);
	comedi_device *dev = counter->counter_dev->dev;
	void * const read_address = devpriv->mite->daq_io_addr + GPCT_OFFSET[counter->chip_index] + registerData[ni_660x_register].offset;

	switch(registerData[ni_660x_register].size)
	{
	case DATA_2B:
		return readw(read_address);
		break;
	case DATA_4B:
		return readl(read_address);
		break;
	default:
		rt_printk("%s: %s: bug! unhandled case = 0x%x in switch.\n", __FILE__, __FUNCTION__, reg);
		BUG();
		break;
	}
	return 0;
}

static int ni_660x_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret;
	unsigned i;

	printk("comedi%d: ni_660x: ",dev->minor);

	if ((ret=alloc_private(dev,sizeof(ni_660x_private))) < 0) return ret;

	ret = ni_660x_find_device(dev, it->options[0], it->options[1]);
	if (ret<0) return ret;

	ret = mite_setup(devpriv->mite);
	if (ret < 0) {
	printk("error setting up mite\n");
	return ret;
	}
	dev->board_name = thisboard->name;
	/* we don't support the interrupt yet */
	//dev->irq = mite_irq(devpriv->mite);

	printk(" %s ", dev->board_name);

	dev->n_subdevices = 2 + NI_660X_MAX_NUM_COUNTERS;

	if(alloc_subdevices(dev, dev->n_subdevices) < 0) return -ENOMEM;

	s = dev->subdevices + 0;
	/* Old GENERAL-PURPOSE COUNTER/TIME (GPCT) subdevice, no longer used */
	s->type = COMEDI_SUBD_UNUSED;

	s = dev->subdevices + 1;
	/* DIGITAL I/O SUBDEVICE */
	s->type	  = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE|SDF_WRITABLE;
	s->n_chan       = NUM_PFI_CHANNELS;
	s->maxdata      = 1;
	s->range_table  = &range_digital;
	s->insn_bits    = ni_660x_dio_insn_bits;
	s->insn_config  = ni_660x_dio_insn_config;
	s->io_bits      = 0;     /* all bits default to input */
	// we use the ioconfig registers to control dio direction, so zero output enables in stc dio control reg
	writew(0, devpriv->mite->daq_io_addr + registerData[STCDIOControl].offset);

	devpriv->counter_dev = ni_gpct_device_construct(dev,
		&ni_gpct_write_register, &ni_gpct_read_register,
		ni_gpct_variant_660x, thisboard->n_ctrs);
	if(devpriv->counter_dev == NULL) return -ENOMEM;
	for(i = 0; i < NI_660X_MAX_NUM_COUNTERS; ++i)
	{
		s = dev->subdevices + 2 + i;
		if(i < thisboard->n_ctrs)
		{
			s->type = COMEDI_SUBD_COUNTER;
			s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
			s->n_chan = 3;
			s->maxdata = 0xffffffff;
			s->insn_read = ni_660x_GPCT_rinsn;
			s->insn_write = ni_660x_GPCT_winsn;
			s->insn_config = ni_660x_GPCT_insn_config;
			s->private = &devpriv->counter_dev->counters[i];

			devpriv->counter_dev->counters[i].chip_index = i / CTRS_PER_CHIP;
			devpriv->counter_dev->counters[i].counter_index = i % CTRS_PER_CHIP;
			devpriv->counter_dev->counters[i].clock_period_ps = 0;
			devpriv->counter_dev->counters[i].mite_chan = NULL;
		}else
		{
			s->type = COMEDI_SUBD_UNUSED;
		}
	}
	for(i = 0; i < thisboard->n_ctrs / CTRS_PER_CHIP; ++i)
	{
		init_tio_chip(dev, i);
	}
	for(i = 0; i < thisboard->n_ctrs; ++i)
	{
		ni_tio_init_counter(&devpriv->counter_dev->counters[i]);
	}

	printk("attached\n");
	return 0;
}


static int
ni_660x_detach(comedi_device *dev)
{
	printk("comedi%d: ni_660x: remove\n",dev->minor);

	if(dev->private)
	{
		if(devpriv->counter_dev)
			ni_gpct_device_destroy(devpriv->counter_dev);
		if(devpriv->mite)
			mite_unsetup(devpriv->mite);
	}
	/* Free irq */

	if(dev->irq) comedi_free_irq(dev->irq,dev);

	/* Same question as with attach ... */
	return 0;
}

static int
ni_660x_GPCT_rinsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	struct ni_gpct *counter = s->private;
	return ni_tio_rinsn(counter, insn, data);
}

static void init_tio_chip(comedi_device *dev, int chipset)
{
	/* See P. 3.5 of the Register-Level Programming manual.  The
		CounterSwap bit has to be set on the second chip, otherwise
		it will try to use the same pins as the first chip.
	*/
	if(chipset)
		writel(CounterSwap,devpriv->mite->daq_io_addr + GPCT_OFFSET[1]
			+ registerData[ClockConfigRegister].offset);
	else
		writel(0,devpriv->mite->daq_io_addr + GPCT_OFFSET[0]
			+ registerData[ClockConfigRegister].offset);
}

static int
ni_660x_GPCT_insn_config(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	struct ni_gpct *counter = s->private;
	return ni_tio_insn_config(counter, insn, data);
}

static int ni_660x_GPCT_winsn(comedi_device *dev,
	comedi_subdevice *s,
	comedi_insn *insn,
	lsampl_t * data)
{
	struct ni_gpct *counter = s->private;
	return ni_tio_winsn(counter, insn, data);
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
	unsigned base_bitfield_channel = CR_CHAN(insn->chanspec);

	// Check if we have to write some bits
	if(data[0])
	{
		/* Only the first 8 lines can be read/written.  The rest can
		only have their input/output configuration changed (although
		the user manual implies the first 32 channels can be used as
		general purpose dio, the register manual doesn't tell you how
		this can be accomplished. */
		/*FIXME: use DIO32Input and DIO32Output registers for dio
		to the 32 bit port */
		if((data[0] << base_bitfield_channel) > 0xff)
		{
			return -EINVAL;
		}
		s->state &= ~(data[0] << base_bitfield_channel);
		s->state |= (data[0] & data[1]) << base_bitfield_channel;
		/* Write out the new digital output lines */
		writew(s->state,devpriv->mite->daq_io_addr + registerData[STCDIOOutput].offset);
	}
	/* on return, data[1] contains the value of the digital
	* input and output lines. */
	data[1] = (readw(devpriv->mite->daq_io_addr + registerData[STCDIOParallelInput].offset) >> base_bitfield_channel) &
		(0xff >> base_bitfield_channel);
	return 2;
}

static void ni_660x_select_pfi_output(comedi_device *dev, unsigned pfi_channel, unsigned output_select)
{
	unsigned bits = readw(devpriv->mite->daq_io_addr + registerData[IOConfigReg(pfi_channel)].offset);
	bits &= ~pfi_output_select_mask(pfi_channel);
	bits |= pfi_output_select_bits(pfi_channel, output_select);
	writew(bits, devpriv->mite->daq_io_addr + registerData[IOConfigReg(pfi_channel)].offset);
}

static void ni660x_config_filter(comedi_device *dev, unsigned pfi_channel, enum ni_gpct_filter_select filter)
{
	unsigned bits = readw(devpriv->mite->daq_io_addr + registerData[IOConfigReg(pfi_channel)].offset);
	bits &= ~pfi_input_select_mask(pfi_channel);
	bits |= pfi_input_select_bits(pfi_channel, filter);
	writew(bits, devpriv->mite->daq_io_addr + registerData[IOConfigReg(pfi_channel)].offset);
}

static int ni_660x_dio_insn_config(comedi_device *dev,
	comedi_subdevice *s,
	comedi_insn *insn,
	lsampl_t *data)
{
	int chan=CR_CHAN(insn->chanspec);

	/* The input or output configuration of each digital line is
	* configured by a special insn_config instruction.  chanspec
	* contains the channel to be changed, and data[0] contains the
	* value COMEDI_INPUT or COMEDI_OUTPUT. */

	switch(data[0])
	{
	case INSN_CONFIG_DIO_OUTPUT:
		devpriv->pfi_direction_bits |= ((uint64_t)1) << chan;
		//FIXME: output select 1 is counter output, 2 is digital output
		ni_660x_select_pfi_output(dev, chan, 1);
		break;
	case INSN_CONFIG_DIO_INPUT:
		devpriv->pfi_direction_bits &= ~(((uint64_t)1) << chan);
		ni_660x_select_pfi_output(dev, chan, 0);
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] = (devpriv->pfi_direction_bits & (((uint64_t)1) << chan)) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
	case INSN_CONFIG_FILTER:
		ni660x_config_filter(dev, chan, data[1]);
		return 0;
	default:
		return -EINVAL;
		break;
	};
	return insn->n;
}


