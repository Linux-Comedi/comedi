/*
    ni_labpc.c driver for National Instruments Lab-PC series boards and compatibles
    Copyright (C) 2001 Frank Mori Hess <fmhess@users.sourceforge.net>

    PCMCIA crap at end of file is adapted from dummy_cs.c 1.31 2001/08/24 12:13:13
    from the pcmcia package.
    The initial developer of the pcmcia dummy_cs.c code is David A. Hinds
    <dahinds@users.sourceforge.net>.  Portions created by David A. Hinds
    are Copyright (C) 1999 David A. Hinds.  All Rights Reserved.

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

************************************************************************
*/
/*
Driver: ni_labpc.o
Description: National Instruments Lab-PC (& compatibles)
Author: Frank Mori Hess <fmhess@users.sourceforge.net>
Devices: [National Instruments] DAQCard-1200 (daqcard-1200), Lab-PC-1200 (labpc-1200),
  Lab-PC-1200AI (labpc-1200ai), Lab-PC+ (lab-pc+), PCI-1200 (pci-1200)
Status: works

Tested with lab-pc-1200.  For the older Lab-PC+, not all input ranges
and analog references will work, the available ranges/arefs will
depend on how you have configured the jumpers on your board
(see your owner's manual).

Configuration options - ISA boards:
  [0] - I/O port base address
  [1] - IRQ (optional, required for timed or externally triggered conversions)
  [2] - DMA channel (optional)

Configuration options - PCI boards:
  [0] - bus (optional)
  [1] - slot (optional)

Configuration options - PCMCIA boards:
  none

Lab-pc+ has quirky chanlist when scanning multiple channels.  Scan
sequence must start at highest channel, then decrement down to
channel 0.  1200 series cards can scan down like lab-pc+ or scan
up from channel zero.

*/

/*
TODO:

NI manuals:
341309a (labpc-1200 register manual)
340988a (daqcard-1200)
340914a (pci-1200)
320502b (lab-pc+)

*/

#define LABPC_DEBUG	// enable debugging messages
//#undef LABPC_DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/comedidev.h>
#include <asm/dma.h>
#include "8253.h"
#include "8255.h"
#include "mite.h"

#if defined(CONFIG_PCMCIA) || defined(CONFIG_PCMCIA_MODULE)

//#include <pcmcia/config.h>
//#include <pcmcia/k_compat.h>

#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/system.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ds.h>
#include <pcmcia/bus_ops.h>

/*
   A linked list of "instances" of the dummy device.  Each actual
   PCMCIA card corresponds to one device instance, and is described
   by one dev_link_t structure (defined in ds.h).

   You may not want to use a linked list for this -- for example, the
   memory card driver uses an array of dev_link_t pointers, where minor
   device numbers are used to derive the corresponding array index.
*/

static dev_link_t *pcmcia_dev_list = NULL;

#endif // CONFIG_PCMCIA

#define LABPC_SIZE           32	// size of io region used by board
#define LABPC_TIMER_BASE            500	// 2 MHz master clock
#define EEPROM_SIZE	256	// 256 byte eeprom
#define NUM_AO_CHAN	2	// boards have two analog output channels

/* Registers for the lab-pc+ */

//write-only registers
#define COMMAND1_REG	0x0
#define   ADC_GAIN_MASK	(0x7 << 4)
#define   ADC_CHAN_BITS(x)	((x) & 0x7)
#define   ADC_SCAN_EN_BIT	0x80	// enables multi channel scans
#define COMMAND2_REG	0x1
#define   PRETRIG_BIT	0x1	// enable pretriggering (used in conjunction with SWTRIG)
#define   HWTRIG_BIT	0x2	// enable paced conversions on external trigger
#define   SWTRIG_BIT	0x4	// enable paced conversions
#define   CASCADE_BIT	0x8	// use two cascaded counters for pacing
#define   DAC_PACED_BIT(channel)	(0x40 << ((channel) & 0x1))
#define COMMAND3_REG	0x2
#define   DMA_EN_BIT	0x1	// enable dma transfers
#define   DIO_INTR_EN_BIT	0x2	// enable interrupts for 8255
#define   DMATC_INTR_EN_BIT	0x4	// enable dma terminal count interrupt
#define   TIMER_INTR_EN_BIT	0x8	// enable timer interrupt
#define   ERR_INTR_EN_BIT	0x10	// enable error interrupt
#define   ADC_FNE_INTR_EN_BIT	0x20	// enable fifo not empty interrupt
#define ADC_CONVERT_REG	0x3
#define DAC_LSB_REG(channel)	(0x4 + 2 * ((channel) & 0x1))
#define DAC_MSB_REG(channel)	(0x5 + 2 * ((channel) & 0x1))
#define ADC_CLEAR_REG	0x8
#define DMATC_CLEAR_REG	0xa
#define TIMER_CLEAR_REG	0xc
#define COMMAND6_REG	0xe	// 1200 boards only
#define   ADC_COMMON_BIT	0x1	// select ground or common-mode reference
#define   ADC_UNIP_BIT	0x2	// adc unipolar
#define   DAC_UNIP_BIT(channel)	(0x4 << ((channel) & 0x1))	// dac unipolar
#define   ADC_FHF_INTR_EN_BIT	0x20	// enable fifo half full interrupt
#define   A1_INTR_EN_BIT	0x40	// enable interrupt on end of hardware count
#define   ADC_SCAN_UP_BIT 0x80	// scan up from channel zero instead of down to zero
#define COMMAND4_REG	0xf
#define   EXT_SCAN_MASTER_EN_BIT	0x1	// enables 'interval' scanning
#define   EXT_SCAN_EN_BIT	0x2	// enables external signal on counter b1 output to trigger scan
#define   EXT_CONVERT_OUT_BIT	0x4	// chooses direction (output or input) for EXTCONV* line
#define   ADC_DIFF_BIT	0x8	// chooses differential inputs for adc (in conjunction with board jumper)
#define   EXT_CONVERT_DISABLE_BIT	0x10
#define COMMAND5_REG	0x1c	// 1200 boards only, calibration stuff
#define   EEPROM_WRITE_UNPROTECT_BIT	0x4// enable eeprom for write
#define   DITHER_EN_BIT	0x8	// enable dithering
#define   CALDAC_LOAD_BIT	0x10	// load calibration dac
#define   SCLOCK_BIT	0x20	// serial clock - rising edge writes, falling edge reads
#define   SDATA_BIT	0x40	// serial data bit for writing to eeprom or calibration dacs
#define   EEPROM_EN_BIT	0x80	// enable eeprom for read/write
#define INTERVAL_COUNT_REG	0x1e
#define INTERVAL_LOAD_REG	0x1f
#define   INTERVAL_LOAD_BITS	0x1

// read-only registers
#define STATUS1_REG	0x0
#define   DATA_AVAIL_BIT	0x1	// data is available in fifo
#define   OVERRUN_BIT	0x2	// overrun has occurred
#define   OVERFLOW_BIT	0x4	// fifo overflow
#define   TIMER_BIT	0x8	// timer interrupt has occured
#define   DMATC_BIT	0x10	// dma terminal count has occured
#define   EXT_TRIG_BIT	0x40	// external trigger has occured
#define STATUS2_REG	0x1d	// 1200 boards only
#define   EEPROM_OUT_BIT	0x1	// programmable eeprom serial output
#define   A1_TC_BIT	0x2	// counter A1 terminal count
#define   FNHF_BIT	0x4	// fifo not half full
#define ADC_FIFO_REG	0xa

#define DIO_BASE_REG	0x10
#define COUNTER_A_BASE_REG	0x14
#define COUNTER_A_CONTROL_REG	(COUNTER_A_BASE_REG + 0x3)
#define   INIT_A0_BITS	0x14	// check modes put conversion pacer output in harmless state (a0 mode 2)
#define   INIT_A1_BITS	0x70	// put hardware conversion counter output in harmless state (a1 mode 0)
#define COUNTER_B_BASE_REG	0x18


static int labpc_attach(comedi_device *dev,comedi_devconfig *it);
static int labpc_detach(comedi_device *dev);
static int labpc_cancel(comedi_device *dev, comedi_subdevice *s);
static void labpc_interrupt(int irq, void *d, struct pt_regs *regs);
static int labpc_drain_fifo(comedi_device *dev);
static void labpc_drain_dma(comedi_device *dev);
static void handle_isa_dma(comedi_device *dev);
static void labpc_drain_dregs(comedi_device *dev);
static int labpc_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd);
static int labpc_ai_cmd(comedi_device *dev, comedi_subdevice *s);
static int labpc_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int labpc_ao_winsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int labpc_ao_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int labpc_calib_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int labpc_calib_write_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int labpc_eeprom_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int labpc_eeprom_write_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static unsigned int labpc_suggest_transfer_size(comedi_cmd cmd);
static void labpc_adc_timing(comedi_device *dev, comedi_cmd *cmd);
static struct mite_struct* labpc_find_device(int bus, int slot);
static unsigned int labpc_inb(unsigned int address);
static void labpc_outb(unsigned int byte, unsigned int address);
static unsigned int labpc_readb(unsigned int address);
static void labpc_writeb(unsigned int byte, unsigned int address);
static int labpc_dio_mem_callback(int dir, int port, int data, unsigned long arg);
static void labpc_load_ai_calibration(comedi_device *dev, unsigned int range);
static void labpc_load_ao_calibration(comedi_device *dev, unsigned int channel, unsigned int range);
static void labpc_serial_out(comedi_device *dev, unsigned int value, unsigned int num_bits);
static unsigned int labpc_serial_in(comedi_device *dev);
static unsigned int labpc_eeprom_read(comedi_device *dev, unsigned int address);
static unsigned int labpc_eeprom_read_status(comedi_device *dev);
static unsigned int labpc_eeprom_write(comedi_device *dev, unsigned int address, unsigned int value);
static void __write_caldac(comedi_device *dev, unsigned int channel, unsigned int value);
static void write_caldac(comedi_device *dev, unsigned int channel, unsigned int value);

enum labpc_bustype {isa_bustype, pci_bustype, pcmcia_bustype};
enum labpc_register_layout {labpc_plus_layout, labpc_1200_layout};
enum transfer_type {fifo_not_empty_transfer, fifo_half_full_transfer, isa_dma_transfer};

typedef struct labpc_board_struct{
	char *name;
	int device_id;	// device id for pci and pcmcia boards
	int ai_speed;	// maximum input speed in nanoseconds
	enum labpc_bustype bustype;	// ISA/PCI/etc.
	enum labpc_register_layout register_layout;	// 1200 has extra registers compared to pc+
	int has_ao;	// has analog output true/false
	// function pointers so we can use inb/outb or readb/writeb as appropriate
	unsigned int (*read_byte)(unsigned int address);
	void (*write_byte)(unsigned int byte, unsigned int address);
	comedi_lrange *ai_range_table;
	int *ai_range_code;
	int *ai_range_is_unipolar;
}labpc_board;

//analog input ranges

#define NUM_LABPC_PLUS_AI_RANGES 16
// indicates unipolar ranges
static int labpc_plus_is_unipolar[NUM_LABPC_PLUS_AI_RANGES] =
{
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
};
// map range index to gain bits
static int labpc_plus_ai_gain_bits[NUM_LABPC_PLUS_AI_RANGES] =
{
	0x00,
	0x10,
	0x20,
	0x30,
	0x40,
	0x50,
	0x60,
	0x70,
	0x00,
	0x10,
	0x20,
	0x30,
	0x40,
	0x50,
	0x60,
	0x70,
};
static comedi_lrange range_labpc_plus_ai = {
	NUM_LABPC_PLUS_AI_RANGES,
	{
		BIP_RANGE(5),
		BIP_RANGE(4),
		BIP_RANGE(2.5),
		BIP_RANGE(1),
		BIP_RANGE(0.5),
		BIP_RANGE(0.25),
		BIP_RANGE(0.1),
		BIP_RANGE(0.05),
		UNI_RANGE(10),
		UNI_RANGE(8),
		UNI_RANGE(5),
		UNI_RANGE(2),
		UNI_RANGE(1),
		UNI_RANGE(0.5),
		UNI_RANGE(0.2),
		UNI_RANGE(0.1),
	}
};

#define NUM_LABPC_1200_AI_RANGES 14
// indicates unipolar ranges
static int labpc_1200_is_unipolar[NUM_LABPC_1200_AI_RANGES] =
{
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	1,
	1,
	1,
	1,
	1,
	1,
	1,
};
// map range index to gain bits
static int labpc_1200_ai_gain_bits[NUM_LABPC_1200_AI_RANGES] =
{
	0x00,
	0x20,
	0x30,
	0x40,
	0x50,
	0x60,
	0x70,
	0x00,
	0x20,
	0x30,
	0x40,
	0x50,
	0x60,
	0x70,
};
static comedi_lrange range_labpc_1200_ai = {
	NUM_LABPC_1200_AI_RANGES,
	{
		BIP_RANGE(5),
		BIP_RANGE(2.5),
		BIP_RANGE(1),
		BIP_RANGE(0.5),
		BIP_RANGE(0.25),
		BIP_RANGE(0.1),
		BIP_RANGE(0.05),
		UNI_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(2),
		UNI_RANGE(1),
		UNI_RANGE(0.5),
		UNI_RANGE(0.2),
		UNI_RANGE(0.1),
	}
};

//analog output ranges

#define AO_RANGE_IS_UNIPOLAR 0x1

static comedi_lrange range_labpc_ao = {
	2,
	{
		BIP_RANGE(5),
		UNI_RANGE(10),
	}
};

static labpc_board labpc_boards[] =
{
#if defined(CONFIG_PCMCIA) || defined(CONFIG_PCMCIA_MODULE)
	{
		name:	"daqcard-1200",
		device_id:	0x103,	// 0x10b is manufacturer id, 0x103 is device id
		ai_speed:	10000,
		bustype:	pcmcia_bustype,
		register_layout:	labpc_1200_layout,
		has_ao:	1,
		read_byte:	labpc_inb,
		write_byte:	labpc_outb,
		ai_range_table:	&range_labpc_1200_ai,
		ai_range_code: labpc_1200_ai_gain_bits,
		ai_range_is_unipolar: labpc_1200_is_unipolar,
	},
#endif // CONFIG_PCMCIA
	{
		name:	"lab-pc-1200",
		ai_speed:	10000,
		bustype:	isa_bustype,
		register_layout:	labpc_1200_layout,
		has_ao:	1,
		read_byte:	labpc_inb,
		write_byte:	labpc_outb,
		ai_range_table:	&range_labpc_1200_ai,
		ai_range_code: labpc_1200_ai_gain_bits,
		ai_range_is_unipolar: labpc_1200_is_unipolar,
	},
	{
		name:	"lab-pc-1200ai",
		ai_speed:	10000,
		bustype:	isa_bustype,
		register_layout:	labpc_1200_layout,
		has_ao:	0,
		read_byte:	labpc_inb,
		write_byte:	labpc_outb,
		ai_range_table:	&range_labpc_1200_ai,
		ai_range_code: labpc_1200_ai_gain_bits,
		ai_range_is_unipolar: labpc_1200_is_unipolar,
	},
	{
		name:	"lab-pc+",
		ai_speed:	12000,
		bustype:	isa_bustype,
		register_layout:	labpc_plus_layout,
		has_ao:	1,
		read_byte:	labpc_inb,
		write_byte:	labpc_outb,
		ai_range_table:	&range_labpc_plus_ai,
		ai_range_code: labpc_plus_ai_gain_bits,
		ai_range_is_unipolar: labpc_plus_is_unipolar,
	},
	{
		name:	"pci-1200",
		device_id:	0x161,
		ai_speed:	10000,
		bustype:	pci_bustype,
		register_layout:	labpc_1200_layout,
		has_ao:	1,
		read_byte:	labpc_readb,
		write_byte:	labpc_writeb,
		ai_range_table:	&range_labpc_1200_ai,
		ai_range_code: labpc_1200_ai_gain_bits,
		ai_range_is_unipolar: labpc_1200_is_unipolar,
	},
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((labpc_board *)dev->board_ptr)

static const int dma_buffer_size = 0xff00;	// size in bytes of dma buffer
static const int sample_size = 2;	// 2 bytes per sample

typedef struct{
	struct mite_struct *mite;	// for mite chip on pci-1200
	volatile unsigned int count;  /* number of data points left to be taken */
	unsigned int ai_range;	// current ai range setting
	unsigned int ao_range[NUM_AO_CHAN];	// current ao range settings
	unsigned int ao_value[NUM_AO_CHAN];	// software copy of analog output values
	// software copys of bits written to command registers
	volatile unsigned int command1_bits;
	volatile unsigned int command2_bits;
	volatile unsigned int command3_bits;
	volatile unsigned int command4_bits;
	volatile unsigned int command5_bits;
	volatile unsigned int command6_bits;
	// store last read of board status registers
	volatile unsigned int status1_bits;
	volatile unsigned int status2_bits;
	unsigned int divisor_a0;	/* value to load into board's counter a0 (conversion pacing) for timed conversions */
	unsigned int divisor_b0; 	/* value to load into board's counter b0 (master) for timed conversions */
	unsigned int divisor_b1; 	/* value to load into board's counter b1 (scan pacing) for timed conversions */
	unsigned int dma_chan;	// dma channel to use
	u16 *dma_buffer;	// buffer ai will dma into
	unsigned int dma_transfer_size;	// transfer size in bytes for current transfer
	enum transfer_type current_transfer;	// we are using dma/fifo-half-full/etc.
	unsigned int eeprom_data[EEPROM_SIZE];	// stores contents of board's eeprom
	unsigned int caldac[12];	// stores settings of calibration dacs
}labpc_private;

#define devpriv ((labpc_private *)dev->private)

static comedi_driver driver_labpc={
	driver_name:	"ni_labpc",
	module:		THIS_MODULE,
	attach:		labpc_attach,
	detach:		labpc_detach,
	num_names:	sizeof(labpc_boards) / sizeof(labpc_board),
	board_name:	(char **)labpc_boards,
	offset:		sizeof(labpc_board),
};

static struct pci_device_id labpc_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_NATINST, 0x161, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, labpc_pci_table);

static int labpc_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int iobase = 0;
	int irq = 0;
	int dma_chan = 0;
	int lsb, msb;
	int i;
	unsigned long flags, isr_flags;
	int ret;
#if defined(CONFIG_PCMCIA) || defined(CONFIG_PCMCIA_MODULE)
	dev_link_t *link;
#endif

	/* allocate and initialize dev->private */
	if(alloc_private(dev, sizeof(labpc_private)) < 0)
		return -ENOMEM;

	// get base address, irq etc. based on bustype
	switch(thisboard->bustype)
	{
		case isa_bustype:
			iobase = it->options[0];
			irq = it->options[1];
			dma_chan = it->options[2];
			break;
		case pci_bustype:
			devpriv->mite = labpc_find_device(it->options[0], it->options[1]);
			if(devpriv->mite == NULL)
			{
				return -EIO;
			}
			if(thisboard->device_id != mite_device_id(devpriv->mite))
			{	// this should never happen since this driver only supports one type of pci board
				printk("bug! mite device id does not match boardtype definition\n");
				return -EINVAL;
			}
			ret = mite_setup(devpriv->mite);
			if(ret < 0) return ret;
			iobase = mite_iobase(devpriv->mite);
			irq = mite_irq(devpriv->mite);
			break;
		case pcmcia_bustype:
#if defined(CONFIG_PCMCIA) || defined(CONFIG_PCMCIA_MODULE)
			link = pcmcia_dev_list; /* XXX hack */
			if(!link) return -EIO;
			iobase = link->io.BasePort1;
			irq = link->irq.AssignedIRQ;
#else
			printk(" driver was not compiled with pcmcia support\n");
			return -EINVAL;
#endif // CONFIG_PCMCIA
			break;
		default:
			printk("bug! couldn't determine board type\n");\
			return -EINVAL;
			break;
	}
	printk("comedi%d: ni_labpc: %s, io 0x%x", dev->minor, thisboard->name, iobase);
	if(irq)
	{
		printk(", irq %i", irq);
	}
	if(dma_chan)
	{
		printk(", dma %i", dma_chan);
	}
	printk("\n");

	if(iobase == 0)
	{
		printk("io base address is zero!\n");
		return -EINVAL;
	}

	// request io regions for isa boards
	if(thisboard->bustype == isa_bustype)
	{
		/* check if io addresses are available */
		if(check_region(iobase, LABPC_SIZE) < 0)
		{
			printk("I/O port conflict\n");
			return -EIO;
		}
		request_region(iobase, LABPC_SIZE, driver_labpc.driver_name);
	}
	dev->iobase = iobase;

	// initialize board's command registers
	thisboard->write_byte(devpriv->command1_bits, dev->iobase + COMMAND1_REG);
	thisboard->write_byte(devpriv->command2_bits, dev->iobase + COMMAND2_REG);
	thisboard->write_byte(devpriv->command3_bits, dev->iobase + COMMAND3_REG);
	thisboard->write_byte(devpriv->command4_bits, dev->iobase + COMMAND4_REG);
	if(thisboard->register_layout == labpc_1200_layout)
	{
		thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
		thisboard->write_byte(devpriv->command6_bits, dev->iobase + COMMAND6_REG);
	}

	/* grab our IRQ */
	if(irq < 0)
	{
		printk("irq out of range\n");
		return -EINVAL;
	}
	if(irq)
	{
		isr_flags = 0;
		if((thisboard->bustype == pci_bustype)
#if 0
				// I'm fairly sure the daqcard-1200 interrupt cannot be shared
				|| (thisboard->bustype == pcmcia_bustype)
#endif
				)
			isr_flags |= SA_SHIRQ;
		if(comedi_request_irq( irq, labpc_interrupt, isr_flags, driver_labpc.driver_name, dev))
		{
			printk( "unable to allocate irq %d\n", irq);
			return -EINVAL;
		}
	}
	dev->irq = irq;

	// grab dma channel
	if(dma_chan < 0 || dma_chan > 3)
	{
		printk(" invalid dma channel\n");
		return -EINVAL;
	}else if(dma_chan)
	{
		// allocate dma buffer
		devpriv->dma_buffer = kmalloc(dma_buffer_size, GFP_KERNEL | GFP_DMA);
		if(devpriv->dma_buffer == NULL)
		{
			printk(" failed to allocate dma buffer\n");
			return -ENOMEM;
		}
		if(request_dma(dma_chan, driver_labpc.driver_name))
		{
			printk(" failed to allocate dma channel %i\n", dma_chan);
			return -EINVAL;
		}
		devpriv->dma_chan = dma_chan;
		flags = claim_dma_lock();
		disable_dma(devpriv->dma_chan);
		set_dma_mode(devpriv->dma_chan, DMA_MODE_READ);
		release_dma_lock(flags);
	}

	dev->board_name = thisboard->name;

	dev->n_subdevices = 5;
	if(alloc_subdevices(dev) < 0)
		return -ENOMEM;

	/* analog input subdevice */
	s = dev->subdevices + 0;
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON | SDF_DIFF;
	s->n_chan = 8;
	s->len_chanlist = 8;
	s->maxdata = (1 << 12) - 1;	// 12 bit resolution
	s->range_table = thisboard->ai_range_table;
	s->do_cmd = labpc_ai_cmd;
	s->do_cmdtest = labpc_ai_cmdtest;
	s->insn_read = labpc_ai_rinsn;
	s->cancel = labpc_cancel;

	/* analog output */
	s = dev->subdevices + 1;
	if(thisboard->has_ao)
	{
/* Could provide command support, except it only has a one sample
 * hardware buffer for analog output and no underrun flag. */
		s->type=COMEDI_SUBD_AO;
		s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_GROUND;
		s->n_chan = NUM_AO_CHAN;
		s->maxdata = (1 << 12) - 1;	// 12 bit resolution
		s->range_table = &range_labpc_ao;
		s->insn_read = labpc_ao_rinsn;
		s->insn_write = labpc_ao_winsn;
		/* initialize analog outputs to a known value */
		for(i = 0; i < s->n_chan; i++)
		{
			devpriv->ao_value[i] = s->maxdata / 2;
			lsb = devpriv->ao_value[i] & 0xff;
			msb = (devpriv->ao_value[i] >> 8) & 0xff;
			thisboard->write_byte(lsb, dev->iobase + DAC_LSB_REG(i));
			thisboard->write_byte(msb, dev->iobase + DAC_MSB_REG(i));
		}
	}else
	{
		s->type = COMEDI_SUBD_UNUSED;
	}

	/* 8255 dio */
	s = dev->subdevices + 2;
	// if board uses io memory we have to give a custom callback function to the 8255 driver
	if(thisboard->write_byte == labpc_writeb)
		subdev_8255_init(dev, s, labpc_dio_mem_callback, (unsigned long)(dev->iobase + DIO_BASE_REG));
	else
		subdev_8255_init(dev, s, NULL, dev->iobase + DIO_BASE_REG);

	// calibration subdevices for boards that have one
	s = dev->subdevices + 3;
	if(thisboard->register_layout == labpc_1200_layout)
	{
		s->type=COMEDI_SUBD_CALIB;
		s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_INTERNAL;
		if(thisboard->has_ao)
			s->n_chan = 8;
		else
			s->n_chan = 4;
		s->maxdata = 0xff;
		s->insn_read = labpc_calib_read_insn;
		s->insn_write = labpc_calib_write_insn;
	}else
		s->type = COMEDI_SUBD_UNUSED;

	/* EEPROM */
	s = dev->subdevices + 4;
	if(thisboard->register_layout == labpc_1200_layout)
	{
		s->type = COMEDI_SUBD_MEMORY;
		s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_INTERNAL;
		s->n_chan = EEPROM_SIZE;
		s->maxdata = 0xff;
		s->insn_read = labpc_eeprom_read_insn;
		s->insn_write = labpc_eeprom_write_insn;

#ifdef LABPC_DEBUG
		printk(" eeprom:");
		for(i = 0; i < EEPROM_SIZE; i++)
		{
			devpriv->eeprom_data[i] = labpc_eeprom_read(dev, i);
			printk(" %i:0x%x ", i, devpriv->eeprom_data[i]);
		}
		printk("\n");
#endif
	}else
		s->type = COMEDI_SUBD_UNUSED;

	if(thisboard->register_layout == labpc_1200_layout)
	{
		// load board calibration
		labpc_load_ai_calibration(dev, devpriv->ai_range);
		for(i = 0; i < NUM_AO_CHAN; i++)
			labpc_load_ao_calibration(dev, i, devpriv->ao_range[i]);
	}

	return 0;
};

// adapted from ni_pcimio for finding mite based boards (pc-1200)
static struct mite_struct* labpc_find_device(int bus, int slot)
{
	struct mite_struct *mite;
	int i;
	for(mite = mite_devices; mite; mite = mite->next)
	{
		if(mite->used) continue;
		// if bus/slot are specified then make sure we have the right bus/slot
		if(bus || slot)
		{
			if(bus != mite->pcidev->bus->number || slot != PCI_SLOT(mite->pcidev->devfn)) continue;
		}
		for(i = 0; i < driver_labpc.num_names; i++)
		{
			if(labpc_boards[i].bustype != pci_bustype) continue;
			if(mite_device_id(mite) == labpc_boards[i].device_id)
			{
				 return mite;
			 }
		}
	}
	printk("no device found\n");
	mite_list_devices();
	return NULL;
}

static int labpc_detach(comedi_device *dev)
{
	printk("comedi%d: ni_labpc: remove\n", dev->minor);

	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices + 2);

	/* only free stuff if it has been allocated by _attach */
	if(devpriv->dma_buffer)
		kfree(devpriv->dma_buffer);
	if(devpriv->dma_chan)
		free_dma(devpriv->dma_chan);
	if(thisboard->bustype != pcmcia_bustype &&
		dev->iobase)
		release_region(dev->iobase, LABPC_SIZE);
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);

	return 0;
};

static int labpc_cancel(comedi_device *dev, comedi_subdevice *s)
{
	devpriv->command2_bits &= ~SWTRIG_BIT & ~HWTRIG_BIT & ~PRETRIG_BIT;
	thisboard->write_byte(devpriv->command2_bits, dev->iobase + COMMAND2_REG);
	devpriv->command3_bits = 0;
	thisboard->write_byte(devpriv->command3_bits, dev->iobase + COMMAND3_REG);

	return 0;
}

static int labpc_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err = 0;
	int tmp, tmp2;
	int range;
	int i;
	int scan_up;
	int stop_mask;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src || tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER | TRIG_FOLLOW | TRIG_EXT;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src) err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if(!cmd->convert_src || tmp != cmd->convert_src) err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src) err++;

	tmp=cmd->stop_src;
	stop_mask = TRIG_COUNT | TRIG_NONE;
	if(thisboard->register_layout == labpc_1200_layout)
		stop_mask |= TRIG_EXT;
	cmd->stop_src &= stop_mask;
	if(!cmd->stop_src || tmp!=cmd->stop_src) err++;

	if(err) return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->start_src != TRIG_NOW &&
		cmd->start_src != TRIG_EXT) err++;
	if(cmd->scan_begin_src != TRIG_TIMER &&
		cmd->scan_begin_src != TRIG_FOLLOW &&
		cmd->scan_begin_src != TRIG_EXT) err++;
	if(cmd->convert_src != TRIG_TIMER &&
	   cmd->convert_src != TRIG_EXT) err++;
	if(cmd->stop_src != TRIG_COUNT &&
		cmd->stop_src != TRIG_EXT &&
		cmd->stop_src != TRIG_NONE) err++;

	// can't have external stop and start triggers at once
	if(cmd->start_src == TRIG_EXT &&
		cmd->stop_src == TRIG_EXT) err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg == TRIG_NOW && cmd->start_arg != 0)
	{
		cmd->start_arg = 0;
		err++;
	}

	if(!cmd->chanlist_len)
	{
		err++;
	}
	if(cmd->scan_end_arg != cmd->chanlist_len)
	{
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}

	if(cmd->convert_src == TRIG_TIMER)
	{
		if(cmd->convert_arg < thisboard->ai_speed)
		{
			cmd->convert_arg = thisboard->ai_speed;
			err++;
		}
	}

	// make sure scan timing is not too fast
	if(cmd->scan_begin_src == TRIG_TIMER)
	{
		if(cmd->convert_src == TRIG_TIMER &&
			cmd->scan_begin_arg < cmd->convert_arg * cmd->chanlist_len)
		{
			cmd->scan_begin_arg = cmd->convert_arg * cmd->chanlist_len;
			err++;
		}
		if(cmd->scan_begin_arg < thisboard->ai_speed * cmd->chanlist_len)
		{
			cmd->convert_arg = thisboard->ai_speed * cmd->chanlist_len;
			err++;
		}
	}

	// stop source
	switch(cmd->stop_src)
	{
		case TRIG_COUNT:
			if(!cmd->stop_arg)
			{
				cmd->stop_arg = 1;
				err++;
			}
			break;
		case TRIG_NONE:
			if(cmd->stop_arg != 0)
			{
				cmd->stop_arg = 0;
				err++;
			}
			break;
		// TRIG_EXT doesn't care since it doesn't trigger off a numbered channel
		default:
			break;
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	tmp = cmd->convert_arg;
	tmp2 = cmd->scan_begin_arg;
	labpc_adc_timing(dev, cmd);
	if(tmp != cmd->convert_arg ||
		tmp2 != cmd->scan_begin_arg) err++;

	if(err)return 4;

	// check channel/gain list against card's limitations
	if(cmd->chanlist && cmd->chanlist_len > 1)
	{
		range = CR_RANGE(cmd->chanlist[0]);
		// should the scan list counting up or down?
		scan_up = 0;
		if(thisboard->register_layout == labpc_1200_layout &&
			CR_CHAN(cmd->chanlist[0]) == 0)
		{
			scan_up = 1;
		}
		for(i = 1; i < cmd->chanlist_len; i++)
		{
			if(scan_up == 0)
			{
				if(CR_CHAN(cmd->chanlist[i]) != cmd->chanlist_len - i - 1)
				{
					err++;
				}
			}else
			{
				if(CR_CHAN(cmd->chanlist[i]) != i)
				{
					err++;
				}
			}
			if(err)
				comedi_error(dev, "channel scanning order specified in chanlist is not supported by hardware.\n");
			if(CR_RANGE(cmd->chanlist[i]) != range)
			{
				comedi_error(dev, "entries in chanlist must all have the same gain\n");
				err++;
			}
		}
	}

	if(err)return 5;

	return 0;
}

static int labpc_ai_cmd(comedi_device *dev, comedi_subdevice *s)
{
	int channel, range, aref;
	unsigned long irq_flags;
	int ret;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	int scan_up, scan_enable;
	enum transfer_type xfer;

	if(!dev->irq)
	{
		comedi_error(dev, "no irq assigned, cannot perform command");
		return -1;
	}

	range = CR_RANGE(cmd->chanlist[0]);
	aref = CR_AREF(cmd->chanlist[0]);

	// make sure board is disabled before setting up aquisition
	devpriv->command2_bits &= ~SWTRIG_BIT & ~HWTRIG_BIT & ~PRETRIG_BIT;
	thisboard->write_byte(devpriv->command2_bits, dev->iobase + COMMAND2_REG);
	devpriv->command3_bits = 0;
	thisboard->write_byte(devpriv->command3_bits, dev->iobase + COMMAND3_REG);

	// initialize software conversion count
	if(cmd->stop_src == TRIG_COUNT)
	{
		devpriv->count = cmd->stop_arg * cmd->chanlist_len;
	}
	// setup hardware conversion counter
	if(cmd->stop_src == TRIG_EXT)
	{
		// load counter a1 with count of 3 (pc+ manual says this is minimum allowed) using mode 0
		ret = i8254_load(dev->iobase + COUNTER_A_BASE_REG, 1, 3, 0);
		if(ret < 0)
		{
			comedi_error(dev, "error loading counter a1");
			return -1;
		}
	}else	// otherwise, just put a1 in mode 0 with no count to set its output low
		thisboard->write_byte(INIT_A1_BITS, dev->iobase + COUNTER_A_CONTROL_REG);

	// are we going to use scan mode?
	if(cmd->chanlist_len > 1)
	{
		scan_enable = 1;
		// figure out if we are scanning upwards or downwards through channels
		if(cmd->chanlist_len > 1 &&
			thisboard->register_layout == labpc_1200_layout &&
			CR_CHAN(cmd->chanlist[0]) == 0)
		{
			scan_up = 1;
		}else
			scan_up = 0;
	}else
	{
		scan_enable = 0;
		scan_up = 0;
	}

	// figure out what method we will use to transfer data
	if(devpriv->dma_chan &&	// need a dma channel allocated
		// dma unsafe at RT priority, and too much setup time for TRIG_WAKE_EOS for
		(cmd->flags & (TRIG_WAKE_EOS | TRIG_RT)) == 0 &&
		// only available on the isa boards
		thisboard->bustype == isa_bustype)
	{
		xfer = isa_dma_transfer;
	}else if(thisboard->register_layout == labpc_1200_layout &&	// pc-plus has no fifo-half full interrupt
		// wake-end-of-scan should interrupt on fifo not empty
		(cmd->flags & TRIG_WAKE_EOS) == 0 &&
		// make sure we are taking more than just a few points
		(cmd->stop_src != TRIG_COUNT || devpriv->count > 256))
	{
		xfer = fifo_half_full_transfer;
	}else
		xfer = fifo_not_empty_transfer;
	devpriv->current_transfer = xfer;

	// setup command6 register for 1200 boards
	if(thisboard->register_layout == labpc_1200_layout)
	{
		// reference inputs to ground or common?
		if(aref != AREF_GROUND)
			devpriv->command6_bits |= ADC_COMMON_BIT;
		else
			devpriv->command6_bits &= ~ADC_COMMON_BIT;
		// bipolar or unipolar range?
		if(thisboard->ai_range_is_unipolar[range])
			devpriv->command6_bits |= ADC_UNIP_BIT;
		else
			devpriv->command6_bits &= ~ADC_UNIP_BIT;
		// interrupt on fifo half full?
		if(xfer == fifo_half_full_transfer)
			devpriv->command6_bits |= ADC_FHF_INTR_EN_BIT;
		else
			devpriv->command6_bits &= ~ADC_FHF_INTR_EN_BIT;
		// enable interrupt on counter a1 terminal count?
		if(cmd->stop_src == TRIG_EXT)
			devpriv->command6_bits |= A1_INTR_EN_BIT;
		else
			devpriv->command6_bits &= ~A1_INTR_EN_BIT;
		// are we scanning up or down through channels?
		if(scan_up)
			devpriv->command6_bits |= ADC_SCAN_UP_BIT;
		else
			devpriv->command6_bits &= ~ADC_SCAN_UP_BIT;
		// write to register
		thisboard->write_byte(devpriv->command6_bits, dev->iobase + COMMAND6_REG);

		// if range has changed, update calibration dacs
		if(range != devpriv->ai_range)
		{
			labpc_load_ai_calibration(dev, range);
		}
	}

	/* setup channel list, etc (command1 register) */
	devpriv->command1_bits = 0;
	if(scan_up)
		channel = CR_CHAN(cmd->chanlist[cmd->chanlist_len - 1]);
	else
		channel = CR_CHAN(cmd->chanlist[0]);
	// munge channel bits for differential / scan disabled mode
	if(scan_enable == 0 && aref == AREF_DIFF)
		channel *= 2;
	devpriv->command1_bits |= ADC_CHAN_BITS(channel);
	devpriv->command1_bits |= thisboard->ai_range_code[range];
	thisboard->write_byte(devpriv->command1_bits, dev->iobase + COMMAND1_REG);
	// manual says to set scan enable bit on second pass
	if(scan_enable)
	{
		devpriv->command1_bits |= ADC_SCAN_EN_BIT;
		/* need a brief delay before enabling scan, or scan list will get screwed when you switch
		 * between scan up to scan down mode - dunno why */
		udelay(1);
		thisboard->write_byte(devpriv->command1_bits, dev->iobase + COMMAND1_REG);
	}

	// setup any external triggering/pacing (command4 register)
	devpriv->command4_bits = 0;
	if(cmd->convert_src != TRIG_EXT)
		devpriv->command4_bits |= EXT_CONVERT_DISABLE_BIT;
	switch(cmd->scan_begin_src)
	{
		case TRIG_EXT:
			devpriv->command4_bits |= EXT_SCAN_EN_BIT | EXT_SCAN_MASTER_EN_BIT;
			break;
		case TRIG_TIMER:
			devpriv->command4_bits |= EXT_SCAN_MASTER_EN_BIT;
			break;
		default:
			break;
	}
	if(cmd->scan_begin_src == TRIG_EXT)
		devpriv->command4_bits |= EXT_SCAN_MASTER_EN_BIT | EXT_SCAN_EN_BIT;
	// single-ended/differential
	if(aref == AREF_DIFF)
		devpriv->command4_bits |= ADC_DIFF_BIT;
	thisboard->write_byte(devpriv->command4_bits, dev->iobase + COMMAND4_REG);

	// make sure interval counter register doesn't cause problems
	if(devpriv->command4_bits & EXT_SCAN_MASTER_EN_BIT &&
		cmd->chanlist_len == 1)
	{
		// set count to one
		thisboard->write_byte(0x1, dev->iobase + INTERVAL_COUNT_REG);
		// load count
		thisboard->write_byte(INTERVAL_LOAD_BITS, dev->iobase + INTERVAL_LOAD_REG);
	}

	if(cmd->convert_src == TRIG_TIMER || cmd->scan_begin_src == TRIG_TIMER)
	{
		// set up pacing
		labpc_adc_timing(dev, cmd);
		// load counter b0 in mode 3
		ret = i8254_load(dev->iobase + COUNTER_B_BASE_REG, 0, devpriv->divisor_b0, 3);
		if(ret < 0)
		{
			comedi_error(dev, "error loading counter b0");
			return -1;
		}
		// set up conversion pacing
		if(cmd->convert_src == TRIG_TIMER)
		{
			// load counter a0 in mode 2
			ret = i8254_load(dev->iobase + COUNTER_A_BASE_REG, 0, devpriv->divisor_a0, 2);
			if(ret < 0)
			{
				comedi_error(dev, "error loading counter a0");
				return -1;
			}
		}
		// set up scan pacing
		if(cmd->scan_begin_src == TRIG_TIMER)
		{
			// load counter b1 in mode 2
			ret = i8254_load(dev->iobase + COUNTER_B_BASE_REG, 1, devpriv->divisor_b1, 2);
			if(ret < 0)
			{
				comedi_error(dev, "error loading counter b1");
				return -1;
			}
		}
	}

	// clear adc fifo
	thisboard->write_byte(0x1, dev->iobase + ADC_CLEAR_REG);
	thisboard->read_byte(dev->iobase + ADC_FIFO_REG);
	thisboard->read_byte(dev->iobase + ADC_FIFO_REG);

	// set up dma transfer
	if(xfer == isa_dma_transfer)
	{
		irq_flags = claim_dma_lock();
		disable_dma(devpriv->dma_chan);
		/* clear flip-flop to make sure 2-byte registers for
		* count and address get set correctly */
		clear_dma_ff(devpriv->dma_chan);
		set_dma_addr(devpriv->dma_chan, virt_to_bus(devpriv->dma_buffer));
		// set appropriate size of transfer
		devpriv->dma_transfer_size = labpc_suggest_transfer_size(*cmd);
		if(cmd->stop_src == TRIG_COUNT &&
			devpriv->count * sample_size < devpriv->dma_transfer_size)
		{
			devpriv->dma_transfer_size = devpriv->count * sample_size;
		}
		set_dma_count(devpriv->dma_chan, devpriv->dma_transfer_size);
		enable_dma(devpriv->dma_chan);
		release_dma_lock(irq_flags);
		// enable board's dma
		devpriv->command3_bits |= DMA_EN_BIT | DMATC_INTR_EN_BIT;
	}else
		devpriv->command3_bits &= ~DMA_EN_BIT & ~DMATC_INTR_EN_BIT;

	// enable error interrupts
	devpriv->command3_bits |= ERR_INTR_EN_BIT;
	// enable fifo not empty interrupt?
	if(xfer == fifo_not_empty_transfer)
		devpriv->command3_bits |= ADC_FNE_INTR_EN_BIT;
	else
		devpriv->command3_bits &= ~ADC_FNE_INTR_EN_BIT;
	thisboard->write_byte(devpriv->command3_bits, dev->iobase + COMMAND3_REG);

	// startup aquisition

	// command2 reg
	// use 2 cascaded counters for pacing
	devpriv->command2_bits |= CASCADE_BIT;
	switch(cmd->start_src)
	{
		case TRIG_EXT:
			devpriv->command2_bits |= HWTRIG_BIT;
			devpriv->command2_bits &= ~PRETRIG_BIT & SWTRIG_BIT;
			break;
		case TRIG_NOW:
			devpriv->command2_bits |= SWTRIG_BIT;
			devpriv->command2_bits &= ~PRETRIG_BIT & ~HWTRIG_BIT;
			break;
		default:
			comedi_error(dev, "bug with start_src");
			return -1;
			break;
	}
	switch(cmd->stop_src)
	{
		case TRIG_EXT:
			devpriv->command2_bits |= HWTRIG_BIT | PRETRIG_BIT;
			break;
		case TRIG_COUNT:
		case TRIG_NONE:
			break;
		default:
			comedi_error(dev, "bug with stop_src");
			return -1;
	}
	thisboard->write_byte(devpriv->command2_bits, dev->iobase + COMMAND2_REG);

	return 0;
}

/* interrupt service routine */
static void labpc_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev = d;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async;
	comedi_cmd *cmd;

	if(dev->attached == 0)
	{
		comedi_error(dev, "premature interrupt");
		return;
	}

	async = s->async;
	cmd = &async->cmd;
	async->events = 0;

	// read board status
	devpriv->status1_bits = thisboard->read_byte(dev->iobase + STATUS1_REG);
	if(thisboard->register_layout == labpc_1200_layout)
		devpriv->status2_bits = thisboard->read_byte(dev->iobase + STATUS2_REG);

	if((devpriv->status1_bits & (DMATC_BIT | TIMER_BIT | OVERFLOW_BIT | OVERRUN_BIT | DATA_AVAIL_BIT)) == 0 &&
		(devpriv->status2_bits & A1_TC_BIT) == 0 &&
		(devpriv->status2_bits & FNHF_BIT))
	{
		return;
	}

	if(devpriv->status1_bits & OVERRUN_BIT)
	{
		// clear error interrupt
		thisboard->write_byte(0x1, dev->iobase + ADC_CLEAR_REG);
		async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
		comedi_event(dev, s, async->events);
		comedi_error(dev, "overrun");
		return;
	}

	if(devpriv->current_transfer == isa_dma_transfer)
	{
		// if a dma terminal count of external stop trigger has occurred
		if(devpriv->status1_bits & DMATC_BIT ||
			(thisboard->register_layout == labpc_1200_layout && devpriv->status2_bits & A1_TC_BIT))
		{
			handle_isa_dma(dev);
		}
	}else labpc_drain_fifo(dev);

	if(devpriv->status1_bits & TIMER_BIT)
	{
		comedi_error(dev, "handled timer interrupt?");
		// clear it
		thisboard->write_byte(0x1, dev->iobase + TIMER_CLEAR_REG);
	}

	if(devpriv->status1_bits & OVERFLOW_BIT)
	{
		// clear error interrupt
		thisboard->write_byte(0x1, dev->iobase + ADC_CLEAR_REG);
		async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
		comedi_error(dev, "overflow");
		return;
	}

	// handle external stop trigger
	if(cmd->stop_src == TRIG_EXT)
	{
		if(devpriv->status2_bits & A1_TC_BIT)
		{
			labpc_drain_dregs(dev);
			labpc_cancel(dev, s);
			async->events |= COMEDI_CB_EOA;
		}
	}

	/* TRIG_COUNT end of acquisition */
	if(cmd->stop_src == TRIG_COUNT)
	{
		if(devpriv->count == 0)
		{
			labpc_cancel(dev, s);
			async->events |= COMEDI_CB_EOA;
		}
	}

	comedi_event(dev, s, async->events);
}

// read all available samples from ai fifo
static int labpc_drain_fifo(comedi_device *dev)
{
	unsigned int lsb, msb;
	sampl_t data;
	comedi_async *async = dev->read_subdev->async;
	const int timeout = 10000;
	unsigned int i;

	devpriv->status1_bits = thisboard->read_byte(dev->iobase + STATUS1_REG);

	for(i = 0; (devpriv->status1_bits & DATA_AVAIL_BIT) && i < timeout; i++)
	{
		lsb = thisboard->read_byte(dev->iobase + ADC_FIFO_REG);
		msb = thisboard->read_byte(dev->iobase + ADC_FIFO_REG);
		data = (msb << 8) | lsb;
		comedi_buf_put(async, data);
		// quit if we have all the data we want
		if(async->cmd.stop_src == TRIG_COUNT)
		{
			devpriv->count--;
			if(devpriv->count == 0) break;
		}
		devpriv->status1_bits = thisboard->read_byte(dev->iobase + STATUS1_REG);
	}
	if(i == timeout)
	{
		comedi_error(dev, "ai timeout, fifo never empties");
		async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
		return -1;
	}

	async->events |= COMEDI_CB_BLOCK;

	return 0;
}

static void labpc_drain_dma(comedi_device *dev)
{
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	int status;
	unsigned long flags;
	unsigned int max_points, num_points, residue, leftover;

	status = devpriv->status1_bits;

	flags = claim_dma_lock();
	disable_dma(devpriv->dma_chan);
	/* clear flip-flop to make sure 2-byte registers for
	* count and address get set correctly */
	clear_dma_ff(devpriv->dma_chan);

	// figure out how many points to read
	max_points = devpriv->dma_transfer_size  / sample_size;
	/* residue is the number of points left to be done on the dma
	* transfer.  It should always be zero at this point unless
	* the stop_src is set to external triggering.
	*/
	residue = get_dma_residue(devpriv->dma_chan) / sample_size;
	num_points = max_points - residue;
	if(devpriv->count < num_points &&
		async->cmd.stop_src == TRIG_COUNT)
		num_points = devpriv->count;

	// figure out how many points will be stored next time
	leftover = 0;
	if(async->cmd.stop_src != TRIG_COUNT)
	{
		leftover = devpriv->dma_transfer_size / sample_size;
	}else if(devpriv->count > num_points)
	{
		leftover = devpriv->count - num_points;
		if(leftover > max_points)
			leftover = max_points;
	}

	/* write data to comedi buffer */
	comedi_buf_put_array(async, devpriv->dma_buffer, num_points);
	if(async->cmd.stop_src == TRIG_COUNT) devpriv->count -= num_points;

	// set address and count for next transfer
	set_dma_addr(devpriv->dma_chan, virt_to_bus(devpriv->dma_buffer));
	set_dma_count(devpriv->dma_chan, leftover * sample_size);
	release_dma_lock(flags);

	async->events |= COMEDI_CB_BLOCK;
}

static void handle_isa_dma(comedi_device *dev)
{
	labpc_drain_dma(dev);

	enable_dma(devpriv->dma_chan);

	// clear dma tc interrupt
	thisboard->write_byte(0x1, dev->iobase + DMATC_CLEAR_REG);
}

/* makes sure all data aquired by board is transfered to comedi (used
 * when aquisition is terminated by stop_src == TRIG_EXT). */
static void labpc_drain_dregs(comedi_device *dev)
{
	if(devpriv->current_transfer == isa_dma_transfer)
		labpc_drain_dma(dev);

	labpc_drain_fifo(dev);
}

static int labpc_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
 	int i, n;
	int chan, range;
	int lsb, msb;
	int timeout = 1000;

	// disable timed conversions
	devpriv->command2_bits &= ~SWTRIG_BIT & ~HWTRIG_BIT & ~PRETRIG_BIT;
	thisboard->write_byte(devpriv->command2_bits, dev->iobase + COMMAND2_REG);

	// disable interrupt generation and dma
	devpriv->command3_bits = 0;
	thisboard->write_byte(devpriv->command3_bits, dev->iobase + COMMAND3_REG);

		/* set gain and channel */
	devpriv->command1_bits = 0;
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);
	devpriv->command1_bits |= thisboard->ai_range_code[range];
	// munge channel bits for differential/scan disabled mode
	if(CR_AREF(insn->chanspec) == AREF_DIFF)
		chan *= 2;
	devpriv->command1_bits |= ADC_CHAN_BITS(chan);
	thisboard->write_byte(devpriv->command1_bits, dev->iobase + COMMAND1_REG);

	// setup command6 register for 1200 boards
	if(thisboard->register_layout == labpc_1200_layout)
	{
		// reference inputs to ground or common?
		if(CR_AREF(insn->chanspec) != AREF_GROUND)
			devpriv->command6_bits |= ADC_COMMON_BIT;
		else
			devpriv->command6_bits &= ~ADC_COMMON_BIT;
		// bipolar or unipolar range?
		if(thisboard->ai_range_is_unipolar[range])
			devpriv->command6_bits |= ADC_UNIP_BIT;
		else
			devpriv->command6_bits &= ~ADC_UNIP_BIT;
		// don't interrupt on fifo half full
		devpriv->command6_bits &= ~ADC_FHF_INTR_EN_BIT;
		// don't enable interrupt on counter a1 terminal count?
		devpriv->command6_bits &= ~A1_INTR_EN_BIT;
		// write to register
		thisboard->write_byte(devpriv->command6_bits, dev->iobase + COMMAND6_REG);

		// if range has changed, update calibration dacs
		if(range != devpriv->ai_range)
		{
			labpc_load_ai_calibration(dev, range);
		}
	}

	// setup command4 register
	devpriv->command4_bits = 0;
	devpriv->command4_bits |= EXT_CONVERT_DISABLE_BIT;
	// single-ended/differential
	if(CR_AREF(insn->chanspec) == AREF_DIFF)
		devpriv->command4_bits |= ADC_DIFF_BIT;
	thisboard->write_byte(devpriv->command4_bits, dev->iobase + COMMAND4_REG);

	// initialize pacer counter output to make sure it doesn't cause any problems
	thisboard->write_byte(INIT_A0_BITS, dev->iobase + COUNTER_A_CONTROL_REG);

	// clear adc fifo
	thisboard->write_byte(0x1, dev->iobase + ADC_CLEAR_REG);
	thisboard->read_byte(dev->iobase + ADC_FIFO_REG);
	thisboard->read_byte(dev->iobase + ADC_FIFO_REG);

	// give it a little settling time
	udelay(5);

	for(n = 0; n < insn->n; n++)
	{
		/* trigger conversion */
		thisboard->write_byte(0x1, dev->iobase + ADC_CONVERT_REG);

		for(i = 0; i < timeout; i++)
		{
			if(thisboard->read_byte(dev->iobase + STATUS1_REG) & DATA_AVAIL_BIT)
				break;
		}
		if(i == timeout)
		{
			comedi_error(dev, "timeout");
			return -ETIME;
		}
		lsb = thisboard->read_byte(dev->iobase + ADC_FIFO_REG);
		msb = thisboard->read_byte(dev->iobase + ADC_FIFO_REG);
		data[n] = (msb << 8) | lsb;
	}

	return n;
}

// analog output insn
static int labpc_ao_winsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int channel, range;
	int lsb, msb;

	channel = CR_CHAN(insn->chanspec);

	// turn off pacing of analog output channel
	devpriv->command2_bits &= ~DAC_PACED_BIT(channel);
	thisboard->write_byte(devpriv->command2_bits, dev->iobase + COMMAND2_REG);

	// set range
	if(thisboard->register_layout == labpc_1200_layout)
	{
		range = CR_RANGE(insn->chanspec);
		if(range & AO_RANGE_IS_UNIPOLAR)
			devpriv->command6_bits |= DAC_UNIP_BIT(channel);
		else
			devpriv->command6_bits &= ~DAC_UNIP_BIT(channel);
		// write to register
		thisboard->write_byte(devpriv->command6_bits, dev->iobase + COMMAND6_REG);

		// if range has changed, update calibration dacs
		if(range != devpriv->ao_range[channel])
		{
			labpc_load_ao_calibration(dev, channel, range);
		}
	}

	// send data
	lsb = data[0] & 0xff;
	msb = (data[0] >> 8 ) & 0xff;
	thisboard->write_byte(lsb, dev->iobase + DAC_LSB_REG(channel));
	thisboard->write_byte(msb, dev->iobase + DAC_MSB_REG(channel));

	// remember value for readback
	devpriv->ao_value[channel] = data[0];

	return 1;
}

// analog output readback insn
static int labpc_ao_rinsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	data[0] = devpriv->ao_value[CR_CHAN(insn->chanspec)];

	return 1;
}

static int labpc_calib_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	data[0] = devpriv->caldac[CR_CHAN(insn->chanspec)];

	return 1;
}

static int labpc_calib_write_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int channel = CR_CHAN(insn->chanspec);

	write_caldac(dev, channel, data[0]);

	return 1;
}

static int labpc_eeprom_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	data[0] = devpriv->eeprom_data[CR_CHAN(insn->chanspec)];

	return 1;
}

static int labpc_eeprom_write_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int channel = CR_CHAN(insn->chanspec);
	int ret;

	// only allow writes to user area of eeprom
	if(channel < 16 || channel > 127)
	{
		printk("eeprom writes are only allowed to channels 16 through 127 (the pointer and user areas)");
		return -EINVAL;
	}

	ret = labpc_eeprom_write(dev, channel, data[0]);
	if(ret < 0) return ret;

	return 1;
}

// utility function that suggests a dma transfer size in bytes
static unsigned int labpc_suggest_transfer_size(comedi_cmd cmd)
{
	unsigned int size;
	unsigned int freq;

	if(cmd.convert_src == TRIG_TIMER)
		freq = 1000000000 / cmd.convert_arg;
	// return some default value
	else
		freq = 0xffffffff;

	// make buffer fill in no more than 1/3 second
	size = (freq / 3) * sample_size;

	// set a minimum and maximum size allowed
	if(size > dma_buffer_size)
		size = dma_buffer_size - dma_buffer_size % sample_size;
	else if(size < sample_size)
		size = sample_size;

	return size;
}

// figures out what counter values to use based on command
static void labpc_adc_timing(comedi_device *dev, comedi_cmd *cmd)
{
	const int max_counter_value = 0x10000;  // max value for 16 bit counter in mode 2
	const int min_counter_value = 2;  // min value for 16 bit counter in mode 2
	unsigned int base_period;

	// if both convert and scan triggers are TRIG_TIMER, then they both rely on counter b0
	if(cmd->convert_src == TRIG_TIMER && cmd->scan_begin_src == TRIG_TIMER)
	{
		// pick the lowest b0 divisor value we can (for maximum input clock speed on convert and scan counters)
		devpriv->divisor_b0 = (cmd->scan_begin_arg - 1) / (LABPC_TIMER_BASE * max_counter_value) + 1;
		if(devpriv->divisor_b0 < min_counter_value)
			devpriv->divisor_b0 = min_counter_value;
		if(devpriv->divisor_b0 > max_counter_value)
			devpriv->divisor_b0 = max_counter_value;

		base_period = LABPC_TIMER_BASE * devpriv->divisor_b0;

		// set a0 for conversion frequency and b1 for scan frequency
		switch(cmd->flags & TRIG_ROUND_MASK)
		{
			default:
			case TRIG_ROUND_NEAREST:
				devpriv->divisor_a0 = (cmd->convert_arg + (base_period / 2)) / base_period;
				devpriv->divisor_b1 = (cmd->scan_begin_arg + (base_period / 2)) / base_period;
				break;
			case TRIG_ROUND_UP:
				devpriv->divisor_a0 = (cmd->convert_arg + (base_period - 1)) / base_period;
				devpriv->divisor_b1 = (cmd->scan_begin_arg + (base_period - 1)) / base_period;
				break;
			case TRIG_ROUND_DOWN:
				devpriv->divisor_a0 = cmd->convert_arg  / base_period;
				devpriv->divisor_b1 = cmd->scan_begin_arg  / base_period;
				break;
		}
		// make sure a0 and b1 values are acceptable
		if(devpriv->divisor_a0 < min_counter_value)
			devpriv->divisor_a0 = min_counter_value;
		if(devpriv->divisor_a0 > max_counter_value)
			devpriv->divisor_a0 = max_counter_value;
		if(devpriv->divisor_b1 < min_counter_value)
			devpriv->divisor_b1 = min_counter_value;
		if(devpriv->divisor_b1 > max_counter_value)
			devpriv->divisor_b1 = max_counter_value;
		// write corrected timings to command
		cmd->convert_arg = base_period * devpriv->divisor_a0;
		cmd->scan_begin_arg = base_period * devpriv->divisor_b1;
	// if only one TRIG_TIMER is used, we can employ the generic cascaded timing functions
	}else if(cmd->scan_begin_src == TRIG_TIMER)
	{
		/* calculate cascaded counter values that give desired scan timing */
		i8253_cascade_ns_to_timer_2div(LABPC_TIMER_BASE, &(devpriv->divisor_b1), &(devpriv->divisor_b0),
			&(cmd->scan_begin_arg), cmd->flags & TRIG_ROUND_MASK);
	}else if(cmd->convert_src == TRIG_TIMER)
	{
		/* calculate cascaded counter values that give desired conversion timing */
		i8253_cascade_ns_to_timer_2div(LABPC_TIMER_BASE, &(devpriv->divisor_a0), &(devpriv->divisor_b0),
			&(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
	}
}

/* functions that do inb/outb and readb/writeb so we can use
 * function pointers to decide which to use */
static unsigned int labpc_inb(unsigned int address)
{
	return inb(address);
}

static void labpc_outb(unsigned int byte, unsigned int address)
{
	outb(byte, address);
}

static unsigned int labpc_readb(unsigned int address)
{
	return readb(address);
}

static void labpc_writeb(unsigned int byte, unsigned int address)
{
	writeb(byte, address);
}

static int labpc_dio_mem_callback(int dir, int port, int data, unsigned long iobase)
{
	if(dir)
	{
		writeb(data, iobase + port);
		return 0;
	}else
	{
		return readb(iobase + port);
	}
}

// load analog input caldacs from eeprom values (depend on range used)
static void labpc_load_ai_calibration(comedi_device *dev, unsigned int range)
{
	// caldac channels
	const int coarse_offset_caldac = 0;
	const int fine_offset_caldac = 1;
	const int postgain_offset_caldac = 2;
	const int gain_caldac = 3;

	// points to (end of) analog input bipolar calibration values
	unsigned int *ai_bip_frame = devpriv->eeprom_data + devpriv->eeprom_data[127];
	const int coarse_offset_index = 0;
	const int fine_offset_index = -1;

	// points to (end of) analog input unipolar calibration values
	unsigned int *ai_unip_frame = devpriv->eeprom_data + devpriv->eeprom_data[126];
	// points to (end of) analog input bipolar calibration values
	unsigned int *bip_gain_frame = devpriv->eeprom_data + devpriv->eeprom_data[123];
	// points to (end of) analog input bipolar calibration values
	unsigned int *unip_gain_frame = devpriv->eeprom_data + devpriv->eeprom_data[122];
	// points to (end of) analog input bipolar calibration values
	unsigned int *bip_offset_frame = devpriv->eeprom_data + devpriv->eeprom_data[121];
	// points to (end of) analog input bipolar calibration values
	unsigned int *unip_offset_frame = devpriv->eeprom_data + devpriv->eeprom_data[120];

	unsigned int *ai_frame, *gain_frame, *offset_frame;
	// eeprom offsets by range
	unsigned int range_to_index[NUM_LABPC_1200_AI_RANGES] =
	{
		0,
		-2,
		-3,
		-4,
		-5,
		-6,
		-7,
		0,
		-2,
		-3,
		-4,
		-5,
		-6,
		-7,
	};


	// store new range index in dev->private struct
	devpriv->ai_range = range;

	if(thisboard->ai_range_is_unipolar[range])
	{
		ai_frame = ai_unip_frame;
		gain_frame = unip_gain_frame;
		offset_frame = unip_offset_frame;
	}else
	{
		ai_frame = ai_bip_frame;
		gain_frame = bip_gain_frame;
		offset_frame = bip_offset_frame;
	}

	// load offset
	write_caldac(dev, coarse_offset_caldac, ai_frame[coarse_offset_index]);
	write_caldac(dev, fine_offset_caldac, ai_frame[fine_offset_index]);

	// load gain and postgain offset
	write_caldac(dev, postgain_offset_caldac, offset_frame[range_to_index[range]]);
	write_caldac(dev, gain_caldac, gain_frame[range_to_index[range]]);
}

// load analog output caldacs from eeprom values (depend on range used)
static void labpc_load_ao_calibration(comedi_device *dev, unsigned int channel, unsigned int range)
{
	// caldacs for analog output channels 0 and 1
	const int offset_caldac[NUM_AO_CHAN] = {4, 6};
	const int gain_caldac[NUM_AO_CHAN] = {5, 7};

	// points to (end of) analog output bipolar calibration values
	unsigned int *ao_bip_frame = devpriv->eeprom_data + devpriv->eeprom_data[125];
	// points to (end of) analog output bipolar calibration values
	unsigned int *ao_unip_frame = devpriv->eeprom_data + devpriv->eeprom_data[124];
	const int offset_index[NUM_AO_CHAN] = {0, -2};
	const int gain_index[NUM_AO_CHAN] = {-1, -3};

	// store new range index in dev->private struct
	devpriv->ao_range[channel] = range;

	if(range & AO_RANGE_IS_UNIPOLAR)
	{
		// load offset
		write_caldac(dev, offset_caldac[channel], ao_unip_frame[offset_index[channel]]);
		// load gain calibration
		write_caldac(dev, gain_caldac[channel], ao_unip_frame[gain_index[channel]]);
	}else
	{
		// load offset
		write_caldac(dev, offset_caldac[channel], ao_bip_frame[offset_index[channel]]);
		// load gain calibration
		write_caldac(dev, gain_caldac[channel], ao_bip_frame[gain_index[channel]]);
	}
}

// lowlevel write to eeprom/dac
static void labpc_serial_out(comedi_device *dev, unsigned int value, unsigned int value_width)
{
	int i;

	for(i = 1; i <= value_width; i++)
	{
		// clear serial clock
		devpriv->command5_bits &= ~SCLOCK_BIT;
		// send bits most significant bit first
		if(value & (1 << (value_width - i)))
			devpriv->command5_bits |= SDATA_BIT;
		else
			devpriv->command5_bits &= ~SDATA_BIT;
		udelay(1);
		thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
		// set clock to load bit
		devpriv->command5_bits |= SCLOCK_BIT;
		udelay(1);
		thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
	}
}

// lowlevel read from eeprom
static unsigned int labpc_serial_in(comedi_device *dev)
{
	unsigned int value = 0;
	int i;
	const int value_width = 8;	// number of bits wide values are

	for(i = 1; i <= value_width; i++)
	{
		// set serial clock
		devpriv->command5_bits |= SCLOCK_BIT;
		udelay(1);
		thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
		// clear clock bit
		devpriv->command5_bits &= ~SCLOCK_BIT;
		udelay(1);
		thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
		// read bits most significant bit first
		udelay(1);
		devpriv->status2_bits = thisboard->read_byte(dev->iobase + STATUS2_REG);
		if(devpriv->status2_bits & EEPROM_OUT_BIT)
		{
			value |= 1 << (value_width - i);
		}
	}

	return value;
}

static unsigned int labpc_eeprom_read(comedi_device *dev, unsigned int address)
{
	unsigned int value;
	const int read_instruction = 0x3;	// bits to tell eeprom to expect a read
	const int write_length = 8;	// 8 bit write lengths to eeprom

	// enable read/write to eeprom
	devpriv->command5_bits &= ~EEPROM_EN_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
	devpriv->command5_bits |= EEPROM_EN_BIT | EEPROM_WRITE_UNPROTECT_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);

	// send read instruction
	labpc_serial_out(dev, read_instruction, write_length);
	// send 8 bit address to read from
	labpc_serial_out(dev, address, write_length);
	// read result
	value = labpc_serial_in(dev);

	// disable read/write to eeprom
	devpriv->command5_bits &= ~EEPROM_EN_BIT & ~EEPROM_WRITE_UNPROTECT_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);

	return value;
}

static unsigned int labpc_eeprom_write(comedi_device *dev, unsigned int address, unsigned int value)
{
	const int write_enable_instruction = 0x6;
	const int write_instruction = 0x2;
	const int write_length = 8;	// 8 bit write lengths to eeprom
	const int write_in_progress_bit = 0x1;
	const int timeout = 10000;
	int i;

	// make sure there isn't already a write in progress
	for(i = 0; i < timeout; i++)
	{
		if((labpc_eeprom_read_status(dev) & write_in_progress_bit) == 0)
			break;
	}
	if(i == timeout)
	{
		comedi_error(dev, "eeprom write timed out");
		return -ETIME;
	}

	// update software copy of eeprom
	devpriv->eeprom_data[address] = value;

	// enable read/write to eeprom
	devpriv->command5_bits &= ~EEPROM_EN_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
	devpriv->command5_bits |= EEPROM_EN_BIT | EEPROM_WRITE_UNPROTECT_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);

	// send write_enable instruction
	labpc_serial_out(dev, write_enable_instruction, write_length);
	devpriv->command5_bits &= ~EEPROM_EN_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);


	// send write instruction
	devpriv->command5_bits |= EEPROM_EN_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
	labpc_serial_out(dev, write_instruction, write_length);
	// send 8 bit address to write to
	labpc_serial_out(dev, address, write_length);
	// write value
	labpc_serial_out(dev, value, write_length);
	devpriv->command5_bits &= ~EEPROM_EN_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);

	// disable read/write to eeprom
	devpriv->command5_bits &= ~EEPROM_EN_BIT & ~EEPROM_WRITE_UNPROTECT_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);

	return 0;
}

static unsigned int labpc_eeprom_read_status(comedi_device *dev)
{
	unsigned int value;
	const int read_status_instruction = 0x5;
	const int write_length = 8;	// 8 bit write lengths to eeprom

	// enable read/write to eeprom
	devpriv->command5_bits &= ~EEPROM_EN_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
	devpriv->command5_bits |= EEPROM_EN_BIT | EEPROM_WRITE_UNPROTECT_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);

	// send read status instruction
	labpc_serial_out(dev, read_status_instruction, write_length);
	// read result
	value = labpc_serial_in(dev);

	// disable read/write to eeprom
	devpriv->command5_bits &= ~EEPROM_EN_BIT & ~EEPROM_WRITE_UNPROTECT_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);

	return value;
}

// writes to 8 bit calibration dacs
static void __write_caldac(comedi_device *dev, unsigned int channel, unsigned int value)
{
	unsigned int reordered_channel, i;
	const int num_channel_bits = 4;	// caldacs use 4 bit channel specification

	// clear caldac load bit and make sure we don't write to eeprom
	devpriv->command5_bits &= ~CALDAC_LOAD_BIT & ~EEPROM_EN_BIT & ~EEPROM_WRITE_UNPROTECT_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);

	/* write 4 bit channel, LSB first (NI appears to have gotten confused here
	 * about how the caldac chip works) */
	reordered_channel = 0;
	for(i = 0; i < num_channel_bits; i++)
	{
		if(channel & (1 << i))
			reordered_channel |= 1 << (num_channel_bits - i - 1);
	}
	labpc_serial_out(dev, reordered_channel, 4);
	// write 8 bit caldac value
	labpc_serial_out(dev, value, 8);

	// set and clear caldac bit to load caldac value
	devpriv->command5_bits |= CALDAC_LOAD_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
	devpriv->command5_bits &= ~CALDAC_LOAD_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);
}

// work around NI's screw up on bit order for caldac channels
static void write_caldac(comedi_device *dev, unsigned int channel, unsigned int value)
{
	if(channel > 7)
	{
		comedi_error(dev, "bug!");
		return;
	}

	devpriv->caldac[channel] = value;
	channel += 3;	// first caldac used by boards is number 3

	__write_caldac(dev, channel, value);
	// do some weirdness to make caldacs 3 and 7 work
	if(channel == 3)
		__write_caldac(dev, 14, value);
	if(channel == 7)
		__write_caldac(dev, 13, value);
}

// PCMCIA crap
#if defined(CONFIG_PCMCIA) || defined(CONFIG_PCMCIA_MODULE)

/*
   All the PCMCIA modules use PCMCIA_DEBUG to control debugging.  If
   you do not define PCMCIA_DEBUG at all, all the debug code will be
   left out.  If you compile with PCMCIA_DEBUG=0, the debug code will
   be present but disabled -- but it can then be enabled for specific
   modules at load time with a 'pc_debug=#' option to insmod.
*/
#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
MODULE_PARM(pc_debug, "i");
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
static char *version =
"ni_labpc.c, based on dummy_cs.c 1.31 2001/08/24 12:13:13";
#else
#define DEBUG(n, args...)
#endif

/*====================================================================*/

/* Parameters that can be set with 'insmod' */

/* The old way: bit map of interrupts to choose from */
/* This means pick from 15, 14, 12, 11, 10, 9, 7, 5, 4, and 3 */
static u_int irq_mask = 0xdeb8;
/* Newer, simpler way of listing specific interrupts */
static int irq_list[4] = { -1 };

MODULE_PARM(irq_mask, "i");
MODULE_PARM(irq_list, "1-4i");

/*====================================================================*/

/*
   The event() function is this driver's Card Services event handler.
   It will be called by Card Services when an appropriate card status
   event is received.  The config() and release() entry points are
   used to configure or release a socket, in response to card
   insertion and ejection events.  They are invoked from the dummy
   event handler.
*/

static void labpc_config(dev_link_t *link);
static void labpc_release(u_long arg);
static int labpc_event(event_t event, int priority,
		       event_callback_args_t *args);

/*
   The attach() and detach() entry points are used to create and destroy
   "instances" of the driver, where each instance represents everything
   needed to manage one actual PCMCIA card.
*/

static dev_link_t *labpc_cs_attach(void);
static void labpc_cs_detach(dev_link_t *);

/*
   You'll also need to prototype all the functions that will actually
   be used to talk to your device.  See 'memory_cs' for a good example
   of a fully self-sufficient driver; the other drivers rely more or
   less on other parts of the kernel.
*/

/*
   The dev_info variable is the "key" that is used to match up this
   device driver with appropriate cards, through the card configuration
   database.
*/

static dev_info_t dev_info = "ni_labpc";

/*
   A dev_link_t structure has fields for most things that are needed
   to keep track of a socket, but there will usually be some device
   specific information that also needs to be kept track of.  The
   'priv' pointer in a dev_link_t structure can be used to point to
   a device-specific private data structure, like this.

   To simplify the data structure handling, we actually include the
   dev_link_t structure in the device's private data structure.

   A driver needs to provide a dev_node_t structure for each device
   on a card.  In some cases, there is only one device per card (for
   example, ethernet cards, modems).  In other cases, there may be
   many actual or logical devices (SCSI adapters, memory cards with
   multiple partitions).  The dev_node_t structures need to be kept
   in a linked list starting at the 'dev' field of a dev_link_t
   structure.  We allocate them in the card's private data structure,
   because they generally shouldn't be allocated dynamically.

   In this case, we also provide a flag to indicate if a device is
   "stopped" due to a power management event, or card ejection.  The
   device IO routines can use a flag like this to throttle IO to a
   card that is not ready to accept it.

   The bus_operations pointer is used on platforms for which we need
   to use special socket-specific versions of normal IO primitives
   (inb, outb, readb, writeb, etc) for card IO.
*/

typedef struct local_info_t {
    dev_link_t		link;
    dev_node_t		node;
    int			stop;
    struct bus_operations *bus;
} local_info_t;

/*====================================================================*/

static void cs_error(client_handle_t handle, int func, int ret)
{
    error_info_t err = { func, ret };
    CardServices(ReportError, handle, &err);
}

/*======================================================================

    labpc_cs_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.

======================================================================*/

static dev_link_t *labpc_cs_attach(void)
{
    local_info_t *local;
    dev_link_t *link;
    client_reg_t client_reg;
    int ret, i;

    DEBUG(0, "labpc_cs_attach()\n");

    /* Allocate space for private device-specific data */
    local = kmalloc(sizeof(local_info_t), GFP_KERNEL);
    if (!local) return NULL;
    memset(local, 0, sizeof(local_info_t));
    link = &local->link; link->priv = local;

    /* Initialize the dev_link_t structure */
    link->release.function = &labpc_release;
    link->release.data = (u_long)link;

    /* Interrupt setup */
    link->irq.Attributes = IRQ_TYPE_EXCLUSIVE;
    link->irq.IRQInfo1 = IRQ_INFO2_VALID|IRQ_LEVEL_ID;
    if (irq_list[0] == -1)
	link->irq.IRQInfo2 = irq_mask;
    else
	for (i = 0; i < 4; i++)
	    link->irq.IRQInfo2 |= 1 << irq_list[i];
    link->irq.Handler = NULL;

    /*
      General socket configuration defaults can go here.  In this
      client, we assume very little, and rely on the CIS for almost
      everything.  In most clients, many details (i.e., number, sizes,
      and attributes of IO windows) are fixed by the nature of the
      device, and can be hard-wired here.
    */
    link->conf.Attributes = 0;
    link->conf.Vcc = 50;
    link->conf.IntType = INT_MEMORY_AND_IO;

    /* Register with Card Services */
    link->next = pcmcia_dev_list;
    pcmcia_dev_list = link;
    client_reg.dev_info = &dev_info;
    client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
    client_reg.EventMask =
	CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
	CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
	CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;
    client_reg.event_handler = &labpc_event;
    client_reg.Version = 0x0210;
    client_reg.event_callback_args.client_data = link;
    ret = CardServices(RegisterClient, &link->handle, &client_reg);
    if (ret != CS_SUCCESS) {
	cs_error(link->handle, RegisterClient, ret);
	labpc_cs_detach(link);
	return NULL;
    }

    return link;
} /* labpc_cs_attach */

/*======================================================================

    This deletes a driver "instance".  The device is de-registered
    with Card Services.  If it has been released, all local data
    structures are freed.  Otherwise, the structures will be freed
    when the device is released.

======================================================================*/

static void labpc_cs_detach(dev_link_t *link)
{
    dev_link_t **linkp;

    DEBUG(0, "labpc_cs_detach(0x%p)\n", link);

    /* Locate device structure */
    for (linkp = &pcmcia_dev_list; *linkp; linkp = &(*linkp)->next)
	if (*linkp == link) break;
    if (*linkp == NULL)
	return;

    /*
       If the device is currently configured and active, we won't
       actually delete it yet.  Instead, it is marked so that when
       the release() function is called, that will trigger a proper
       detach().
    */
    if (link->state & DEV_CONFIG) {
#ifdef PCMCIA_DEBUG
	printk(KERN_DEBUG "ni_labpc: detach postponed, '%s' "
	       "still locked\n", link->dev->dev_name);
#endif
	link->state |= DEV_STALE_LINK;
	return;
    }

    /* Break the link with Card Services */
    if (link->handle)
	CardServices(DeregisterClient, link->handle);

    /* Unlink device structure, and free it */
    *linkp = link->next;
    /* This points to the parent local_info_t struct */
    kfree(link->priv);

} /* labpc_cs_detach */

/*======================================================================

    labpc_config() is scheduled to run after a CARD_INSERTION event
    is received, to configure the PCMCIA socket, and to make the
    device available to the system.

======================================================================*/

#define CS_CHECK(fn, args...) \
while ((last_ret=CardServices(last_fn=(fn),args))!=0) goto cs_failed

#define CFG_CHECK(fn, args...) \
if (CardServices(fn, args) != 0) goto next_entry

static void labpc_config(dev_link_t *link)
{
    client_handle_t handle = link->handle;
    local_info_t *dev = link->priv;
    tuple_t tuple;
    cisparse_t parse;
    int last_fn, last_ret;
    u_char buf[64];
    config_info_t conf;
    win_req_t req;
    memreq_t map;
    cistpl_cftable_entry_t dflt = { 0 };

    DEBUG(0, "labpc_config(0x%p)\n", link);

    /*
       This reads the card's CONFIG tuple to find its configuration
       registers.
    */
    tuple.DesiredTuple = CISTPL_CONFIG;
    tuple.Attributes = 0;
    tuple.TupleData = buf;
    tuple.TupleDataMax = sizeof(buf);
    tuple.TupleOffset = 0;
    CS_CHECK(GetFirstTuple, handle, &tuple);
    CS_CHECK(GetTupleData, handle, &tuple);
    CS_CHECK(ParseTuple, handle, &tuple, &parse);
    link->conf.ConfigBase = parse.config.base;
    link->conf.Present = parse.config.rmask[0];

    /* Configure card */
    link->state |= DEV_CONFIG;

    /* Look up the current Vcc */
    CS_CHECK(GetConfigurationInfo, handle, &conf);
    link->conf.Vcc = conf.Vcc;

    /*
      In this loop, we scan the CIS for configuration table entries,
      each of which describes a valid card configuration, including
      voltage, IO window, memory window, and interrupt settings.

      We make no assumptions about the card to be configured: we use
      just the information available in the CIS.  In an ideal world,
      this would work for any PCMCIA card, but it requires a complete
      and accurate CIS.  In practice, a driver usually "knows" most of
      these things without consulting the CIS, and most client drivers
      will only use the CIS to fill in implementation-defined details.
    */
    tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY;
    CS_CHECK(GetFirstTuple, handle, &tuple);
    while (1) {
	cistpl_cftable_entry_t *cfg = &(parse.cftable_entry);
	CFG_CHECK(GetTupleData, handle, &tuple);
	CFG_CHECK(ParseTuple, handle, &tuple, &parse);

	if (cfg->flags & CISTPL_CFTABLE_DEFAULT) dflt = *cfg;
	if (cfg->index == 0) goto next_entry;
	link->conf.ConfigIndex = cfg->index;

	/* Does this card need audio output? */
	if (cfg->flags & CISTPL_CFTABLE_AUDIO) {
	    link->conf.Attributes |= CONF_ENABLE_SPKR;
	    link->conf.Status = CCSR_AUDIO_ENA;
	}

	/* Use power settings for Vcc and Vpp if present */
	/*  Note that the CIS values need to be rescaled */
	if (cfg->vcc.present & (1<<CISTPL_POWER_VNOM)) {
	    if (conf.Vcc != cfg->vcc.param[CISTPL_POWER_VNOM]/10000)
		goto next_entry;
	} else if (dflt.vcc.present & (1<<CISTPL_POWER_VNOM)) {
	    if (conf.Vcc != dflt.vcc.param[CISTPL_POWER_VNOM]/10000)
		goto next_entry;
	}

	if (cfg->vpp1.present & (1<<CISTPL_POWER_VNOM))
	    link->conf.Vpp1 = link->conf.Vpp2 =
		cfg->vpp1.param[CISTPL_POWER_VNOM]/10000;
	else if (dflt.vpp1.present & (1<<CISTPL_POWER_VNOM))
	    link->conf.Vpp1 = link->conf.Vpp2 =
		dflt.vpp1.param[CISTPL_POWER_VNOM]/10000;

	/* Do we need to allocate an interrupt? */
	if (cfg->irq.IRQInfo1 || dflt.irq.IRQInfo1)
	    link->conf.Attributes |= CONF_ENABLE_IRQ;

	/* IO window settings */
	link->io.NumPorts1 = link->io.NumPorts2 = 0;
	if ((cfg->io.nwin > 0) || (dflt.io.nwin > 0)) {
	    cistpl_io_t *io = (cfg->io.nwin) ? &cfg->io : &dflt.io;
	    link->io.Attributes1 = IO_DATA_PATH_WIDTH_AUTO;
	    if (!(io->flags & CISTPL_IO_8BIT))
		link->io.Attributes1 = IO_DATA_PATH_WIDTH_16;
	    if (!(io->flags & CISTPL_IO_16BIT))
		link->io.Attributes1 = IO_DATA_PATH_WIDTH_8;
	    link->io.IOAddrLines = io->flags & CISTPL_IO_LINES_MASK;
	    link->io.BasePort1 = io->win[0].base;
	    link->io.NumPorts1 = io->win[0].len;
	    if (io->nwin > 1) {
		link->io.Attributes2 = link->io.Attributes1;
		link->io.BasePort2 = io->win[1].base;
		link->io.NumPorts2 = io->win[1].len;
	    }
	    /* This reserves IO space but doesn't actually enable it */
	    CFG_CHECK(RequestIO, link->handle, &link->io);
	}

	/*
	  Now set up a common memory window, if needed.  There is room
	  in the dev_link_t structure for one memory window handle,
	  but if the base addresses need to be saved, or if multiple
	  windows are needed, the info should go in the private data
	  structure for this device.

	  Note that the memory window base is a physical address, and
	  needs to be mapped to virtual space with ioremap() before it
	  is used.
	*/
	if ((cfg->mem.nwin > 0) || (dflt.mem.nwin > 0)) {
	    cistpl_mem_t *mem =
		(cfg->mem.nwin) ? &cfg->mem : &dflt.mem;
	    req.Attributes = WIN_DATA_WIDTH_16|WIN_MEMORY_TYPE_CM;
	    req.Attributes |= WIN_ENABLE;
	    req.Base = mem->win[0].host_addr;
	    req.Size = mem->win[0].len;
	    if (req.Size < 0x1000)
		req.Size = 0x1000;
	    req.AccessSpeed = 0;
	    link->win = (window_handle_t)link->handle;
	    CFG_CHECK(RequestWindow, &link->win, &req);
	    map.Page = 0; map.CardOffset = mem->win[0].card_addr;
	    CFG_CHECK(MapMemPage, link->win, &map);
	}
	/* If we got this far, we're cool! */
	break;

    next_entry:
	if (link->io.NumPorts1)
	    CardServices(ReleaseIO, link->handle, &link->io);
	CS_CHECK(GetNextTuple, handle, &tuple);
    }

    /*
       Allocate an interrupt line.  Note that this does not assign a
       handler to the interrupt, unless the 'Handler' member of the
       irq structure is initialized.
    */
    if (link->conf.Attributes & CONF_ENABLE_IRQ)
	CS_CHECK(RequestIRQ, link->handle, &link->irq);

    /*
       This actually configures the PCMCIA socket -- setting up
       the I/O windows and the interrupt mapping, and putting the
       card and host interface into "Memory and IO" mode.
    */
    CS_CHECK(RequestConfiguration, link->handle, &link->conf);

    /*
      At this point, the dev_node_t structure(s) need to be
      initialized and arranged in a linked list at link->dev.
    */
    sprintf(dev->node.dev_name, "daqcard-1200");
    dev->node.major = dev->node.minor = 0;
    link->dev = &dev->node;

    /* Finally, report what we've done */
    printk(KERN_INFO "%s: index 0x%02x: Vcc %d.%d",
	   dev->node.dev_name, link->conf.ConfigIndex,
	   link->conf.Vcc/10, link->conf.Vcc%10);
    if (link->conf.Vpp1)
	printk(", Vpp %d.%d", link->conf.Vpp1/10, link->conf.Vpp1%10);
    if (link->conf.Attributes & CONF_ENABLE_IRQ)
	printk(", irq %d", link->irq.AssignedIRQ);
    if (link->io.NumPorts1)
	printk(", io 0x%04x-0x%04x", link->io.BasePort1,
	       link->io.BasePort1+link->io.NumPorts1-1);
    if (link->io.NumPorts2)
	printk(" & 0x%04x-0x%04x", link->io.BasePort2,
	       link->io.BasePort2+link->io.NumPorts2-1);
    if (link->win)
	printk(", mem 0x%06lx-0x%06lx", req.Base,
	       req.Base+req.Size-1);
    printk("\n");

    link->state &= ~DEV_CONFIG_PENDING;
    return;

cs_failed:
    cs_error(link->handle, last_fn, last_ret);
    labpc_release((u_long)link);

} /* labpc_config */

/*======================================================================

    After a card is removed, labpc_release() will unregister the
    device, and release the PCMCIA configuration.  If the device is
    still open, this will be postponed until it is closed.

======================================================================*/

static void labpc_release(u_long arg)
{
    dev_link_t *link = (dev_link_t *)arg;

    DEBUG(0, "labpc_release(0x%p)\n", link);

    /*
       If the device is currently in use, we won't release until it
       is actually closed, because until then, we can't be sure that
       no one will try to access the device or its data structures.
    */
    if (link->open) {
	DEBUG(1, "ni_labpc: release postponed, '%s' still open\n",
	      link->dev->dev_name);
	link->state |= DEV_STALE_CONFIG;
	return;
    }

    /* Unlink the device chain */
    link->dev = NULL;

    /*
      In a normal driver, additional code may be needed to release
      other kernel data structures associated with this device.
    */

    /* Don't bother checking to see if these succeed or not */
    if (link->win)
	CardServices(ReleaseWindow, link->win);
    CardServices(ReleaseConfiguration, link->handle);
    if (link->io.NumPorts1)
	CardServices(ReleaseIO, link->handle, &link->io);
    if (link->irq.AssignedIRQ)
	CardServices(ReleaseIRQ, link->handle, &link->irq);
    link->state &= ~DEV_CONFIG;

    if (link->state & DEV_STALE_LINK)
	labpc_cs_detach(link);

} /* labpc_release */

/*======================================================================

    The card status event handler.  Mostly, this schedules other
    stuff to run after an event is received.

    When a CARD_REMOVAL event is received, we immediately set a
    private flag to block future accesses to this device.  All the
    functions that actually access the device should check this flag
    to make sure the card is still present.

======================================================================*/

static int labpc_event(event_t event, int priority,
		       event_callback_args_t *args)
{
    dev_link_t *link = args->client_data;
    local_info_t *dev = link->priv;

    DEBUG(1, "labpc_event(0x%06x)\n", event);

    switch (event) {
    case CS_EVENT_CARD_REMOVAL:
	link->state &= ~DEV_PRESENT;
	if (link->state & DEV_CONFIG) {
	    ((local_info_t *)link->priv)->stop = 1;
	    mod_timer(&link->release, jiffies + HZ/20);
	}
	break;
    case CS_EVENT_CARD_INSERTION:
	link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
	dev->bus = args->bus;
	labpc_config(link);
	break;
    case CS_EVENT_PM_SUSPEND:
	link->state |= DEV_SUSPEND;
	/* Fall through... */
    case CS_EVENT_RESET_PHYSICAL:
	/* Mark the device as stopped, to block IO until later */
	dev->stop = 1;
	if (link->state & DEV_CONFIG)
	    CardServices(ReleaseConfiguration, link->handle);
	break;
    case CS_EVENT_PM_RESUME:
	link->state &= ~DEV_SUSPEND;
	/* Fall through... */
    case CS_EVENT_CARD_RESET:
	if (link->state & DEV_CONFIG)
	    CardServices(RequestConfiguration, link->handle, &link->conf);
	dev->stop = 0;
	/*
	  In a normal driver, additional code may go here to restore
	  the device state and restart IO.
	*/
	break;
    }
    return 0;
} /* labpc_event */

/*====================================================================*/

static int __init init_labpc_cs(void)
{
    servinfo_t serv;
    DEBUG(0, "%s\n", version);
    CardServices(GetCardServicesInfo, &serv);
    if (serv.Revision != CS_RELEASE_CODE) {
	printk(KERN_NOTICE "ni_labpc: Card Services release "
	       "does not match!\n");
	return -1;
    }
    register_pccard_driver(&dev_info, &labpc_cs_attach, &labpc_cs_detach);
    return 0;
}

static void __exit exit_labpc_cs(void)
{
    DEBUG(0, "ni_labpc: unloading\n");
    unregister_pccard_driver(&dev_info);
    while (pcmcia_dev_list != NULL) {
	del_timer(&pcmcia_dev_list->release);
	if (pcmcia_dev_list->state & DEV_CONFIG)
	    labpc_release((u_long)pcmcia_dev_list);
	labpc_cs_detach(pcmcia_dev_list);
    }
}

int init_module(void)
{
	int ret;

	ret = init_labpc_cs();
	if(ret < 0)
		return ret;

	return comedi_driver_register(&driver_labpc);
}

void cleanup_module(void)
{
	exit_labpc_cs();
	comedi_driver_unregister(&driver_labpc);
}

#else
COMEDI_INITCLEANUP(driver_labpc);

#endif // CONFIG_PCMCIA
