/*
    ni_labpc.c driver for National Instruments Lab-PC series boards and compatibles
    Copyright (C) 2000 Frank Mori Hess <fmhess@uiuc.edu>

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
  Lab-PC-1200AI (labpc-1200ai), Lab-PC+ (lab-pc+), PCI-1200 (pci-1200,
Status: In development.  For the older Lab-PC+, not all input ranges and analog
  references will work, depending on how you
  have configured the jumpers on your board (see your owner's manual).

Configuration options - ISA boards:
  [0] - I/O port base address
  [1] - IRQ (optional, required for timed or externally triggered conversions)
  [2] - DMA channel (optional)

Configuration options - PCI boards:
  [0] - bus (optional)
  [1] - slot (optional)

Configuration options - PCMCIA boards:
  none

Lab-pc+ has quirky chanlist when scanning multiple channels.  Scan sequence must start
at highest channel, then decrement down to channel 0.  1200 series cards can scan down
like lab-pc+ or scan up from channel zero.

*/

/*
TODO:
	pcmcia
	be more careful about stop trigger with dma transfers
	calibration subdevice
*/

#define LABPC_DEBUG	// enable debugging messages
//#undef LABPC_DEBUG

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
#include <linux/init.h>
#include <asm/io.h>
#include <linux/comedidev.h>
#include <asm/dma.h>
#include "8253.h"
#include "8255.h"
#include "mite.h"

#define LABPC_SIZE           32	// size of io region used by board
#define LABPC_TIMER_BASE            500	// 2 MHz master clock
#define EEPROM_SIZE	256	// 256 byte eeprom

/* Registers for the lab-pc+ */

//write-only registers
#define COMMAND1_REG	0x0
#define   ADC_GAIN_BITS(x)	(((x) & 0x7) << 4)
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
#define   INIT_A1_BITS	0x70
#define COUNTER_B_BASE_REG	0x18


static int labpc_attach(comedi_device *dev,comedi_devconfig *it);
static int labpc_detach(comedi_device *dev);
static int labpc_cancel(comedi_device *dev, comedi_subdevice *s);
static void labpc_interrupt(int irq, void *d, struct pt_regs *regs);
static void handle_isa_dma(comedi_device *dev);
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
static struct mite_struct* pclab_find_device(int bus, int slot);
static unsigned int labpc_inb(unsigned int address);
static void labpc_outb(unsigned int byte, unsigned int address);
static unsigned int labpc_readb(unsigned int address);
static void labpc_writeb(unsigned int byte, unsigned int address);
static int labpc_dio_mem_callback(int dir, int port, int data, void *arg);
static void labpc_load_calibration(comedi_device *dev);
static void labpc_serial_out(comedi_device *dev, unsigned int value, unsigned int num_bits);
static unsigned int labpc_serial_in(comedi_device *dev);
static unsigned int labpc_eeprom_read(comedi_device *dev, unsigned int address);
static unsigned int labpc_eeprom_write(comedi_device *dev, unsigned int address, unsigned int value);
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
}labpc_board;

//analog input ranges

#define AI_RANGE_IS_UNIPOLAR 0x8

static comedi_lrange range_labpc_ai = {
	16,
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
	{
		name:	"daqcard-1200",
		device_id:	0x103,	// 0x10b is manufacturer id, 0x103 is device id
		ai_speed:	10000,
		bustype:	pcmcia_bustype,
		register_layout:	labpc_1200_layout,
		has_ao:	1,
		read_byte:	labpc_inb,
		write_byte:	labpc_outb,
	},
	{
		name:	"lab-pc-1200",
		ai_speed:	10000,
		bustype:	isa_bustype,
		register_layout:	labpc_1200_layout,
		has_ao:	1,
		read_byte:	labpc_inb,
		write_byte:	labpc_outb,
	},
	{
		name:	"lab-pc-1200ai",
		ai_speed:	10000,
		bustype:	isa_bustype,
		register_layout:	labpc_1200_layout,
		has_ao:	0,
		read_byte:	labpc_inb,
		write_byte:	labpc_outb,
	},
	{
		name:	"lab-pc+",
		ai_speed:	12000,
		bustype:	isa_bustype,
		register_layout:	labpc_plus_layout,
		has_ao:	1,
		read_byte:	labpc_inb,
		write_byte:	labpc_outb,
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
	unsigned int ao_value[2];	// software copy of analog output values
	// software copys of bits written to command registers
	unsigned int command1_bits;
	unsigned int command2_bits;
	unsigned int command3_bits;
	unsigned int command4_bits;
	unsigned int command5_bits;
	unsigned int command6_bits;
	// store last read of board status registers
	unsigned int status1_bits;
	unsigned int status2_bits;
	unsigned int divisor1;	/* value to load into board's counter a0 for timed conversions */
	unsigned int divisor2; 	/* value to load into board's counter b0 for timed conversions */
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

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_labpc);

static int labpc_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int iobase = 0;
	int irq = 0;
	int dma_chan = 0;
	int lsb, msb;
	int i;
	unsigned long flags;

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
			devpriv->mite = pclab_find_device(it->options[0], it->options[1]);
			if(devpriv->mite == NULL)
			{
				return -EIO;
			}
			if(thisboard->device_id != mite_device_id(devpriv->mite))
			{	// this should never happen since this driver only supports one type of pci board
				printk("bug! mite device id does not match boardtype definition\n");
				return -EINVAL;
			}
			iobase = mite_setup(devpriv->mite);
			if(iobase < 0) return -EIO;
			irq = mite_irq(devpriv->mite);
			break;
		case pcmcia_bustype:
			// XXX
			printk("TODO: support pcmicia cards\n");
			return -EINVAL;
			break;
		default:
			printk("bug! couldn't determine board type\n");\
			return -EINVAL;
			break;
	}
	printk("comedi%d: ni_labpc: io 0x%x", dev->minor, iobase);
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

	/* grab our IRQ */
	if(irq < 0)
	{
		printk("irq out of range\n");
		return -EINVAL;
	}
	if(irq)
	{
		if(comedi_request_irq( irq, labpc_interrupt, 0, driver_labpc.driver_name, dev))
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
	s->range_table = &range_labpc_ai;
	s->do_cmd = labpc_ai_cmd;
	s->do_cmdtest = labpc_ai_cmdtest;
	s->insn_read = labpc_ai_rinsn;
	s->cancel = labpc_cancel;

	/* analog output */
	s = dev->subdevices + 1;
	if(thisboard->has_ao)
	{
/* XXX could provide command support, except it doesn't have a hardware
 * buffer for analog output and no underrun flag so speed would be very
 * limited unless using RT interrupt */
		s->type=COMEDI_SUBD_AO;
		s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_GROUND;
		s->n_chan = 2;
		s->maxdata = (1 << 12) - 1;	// 12 bit resolution
		s->range_table = &range_labpc_ao;
		s->insn_read = labpc_ao_rinsn;
		s->insn_write = labpc_ao_winsn;
		/* initialize analog outputs to a known value */
		for(i = 0; i < s->n_chan; i++)
		{
			//XXX set to bi polar output mode
			devpriv->ao_value[i] = s->maxdata / 2;	// XXX should init to 0 for unipolar
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
		subdev_8255_init(dev, s, labpc_dio_mem_callback, (void*)(dev->iobase + DIO_BASE_REG));
	else
		subdev_8255_init(dev, s, NULL, (void*)(dev->iobase + DIO_BASE_REG));

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
		labpc_load_calibration(dev);
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

	return 0;
};

// adapted from ni_pcimio for finding mite based boards (pc-1200)
static struct mite_struct* pclab_find_device(int bus, int slot)
{
	struct mite_struct *mite;
	int i;
	for(mite = mite_devices; mite; mite = mite->next)
	{
		if(mite->used) continue;
		// if bus/slot are specified then make sure we have the right bus/slot
		if(bus || slot)
		{
#ifdef PCI_SUPPORT_VER1
			if(bus != mite->pci_bus || slot! = PCI_SLOT(mite->pci_device_fn)) continue;
#else
			if(bus != mite->pcidev->bus->number || slot != PCI_SLOT(mite->pcidev->devfn)) continue;
#endif
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
	if(dev->iobase)
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
	int tmp;
	int gain;
	int i;
	int scan_up;
	int stop_mask;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src || tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW | TRIG_EXT;
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
	if(cmd->scan_begin_src != TRIG_FOLLOW &&
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
	if(cmd->convert_src == TRIG_TIMER)
	{
		if(cmd->convert_arg < thisboard->ai_speed)
		{
			cmd->convert_arg = thisboard->ai_speed;
			err++;
		}
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
		// TRIG_EXT doesn't care since it doesn't trigger of a numbered channel
		default:
			break;
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	if(cmd->convert_src == TRIG_TIMER)
	{
		tmp = cmd->convert_arg;
		/* calculate counter values that give desired timing */
		i8253_cascade_ns_to_timer_2div(LABPC_TIMER_BASE, &(devpriv->divisor1), &(devpriv->divisor2), &(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->convert_arg) err++;
	}

	if(err)return 4;

	// check channel/gain list against card's limitations
	if(cmd->chanlist && cmd->chanlist_len > 1)
	{
		gain = CR_RANGE(cmd->chanlist[0]);
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
			if(CR_RANGE(cmd->chanlist[i]) != gain)
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
	int endChan, range;
	unsigned long irq_flags;
	int ret;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	int scan_up;
	enum transfer_type xfer;

	if(!dev->irq)
	{
		comedi_error(dev, "no irq assigned, cannot perform command");
		return -1;
	}

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

	// figure out if we are scanning upwards or downwards through channels
	scan_up = 0;
	if(cmd->chanlist_len > 1 &&
		thisboard->register_layout == labpc_1200_layout &&
		CR_CHAN(cmd->chanlist[0]) == 0)
	{
		scan_up = 1;
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

#ifdef LABPC_DEBUG
	if(xfer == isa_dma_transfer)
		comedi_error(dev, "using dma transfer");
	else if(xfer == fifo_half_full_transfer)
		comedi_error(dev, "using fifo half full interrupt");
	else if(xfer == fifo_not_empty_transfer)
		comedi_error(dev, "using fifo not empty interrupt");
#endif

	// setup command6 register for 1200 boards
	if(thisboard->register_layout == labpc_1200_layout)
	{
		// reference inputs to ground or common?
		if(CR_AREF(cmd->chanlist[0]) != AREF_GROUND)
			devpriv->command6_bits |= ADC_COMMON_BIT;
		else
			devpriv->command6_bits &= ~ADC_COMMON_BIT;
		// bipolar or unipolar range?
		if(CR_RANGE(cmd->chanlist[0]) & AI_RANGE_IS_UNIPOLAR)
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
	}

	/* setup channel list, etc (command1 register) */
	devpriv->command1_bits = 0;
	if(scan_up)
		endChan = CR_CHAN(cmd->chanlist[cmd->chanlist_len - 1]);
	else
		endChan = CR_CHAN(cmd->chanlist[0]);
	devpriv->command1_bits |= ADC_CHAN_BITS(endChan);
	range = CR_RANGE(cmd->chanlist[0]);
	devpriv->command1_bits |= ADC_GAIN_BITS(range);
	thisboard->write_byte(devpriv->command1_bits, dev->iobase + COMMAND1_REG);
	// manual says to set scan enable bit on second pass
	if(cmd->chanlist_len > 1)
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
	if(cmd->scan_begin_src == TRIG_EXT)
		devpriv->command4_bits |= EXT_SCAN_MASTER_EN_BIT | EXT_SCAN_EN_BIT;
	// single-ended/differential
	if(CR_AREF(cmd->chanlist[0]) == AREF_DIFF)
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

	// set up conversion pacing
	if(cmd->convert_src == TRIG_TIMER)
	{
		/* set conversion frequency */
		i8253_cascade_ns_to_timer_2div(LABPC_TIMER_BASE, &(devpriv->divisor1),
			&(devpriv->divisor2), &(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
		// load counter b0 in mode 3
		ret = i8254_load(dev->iobase + COUNTER_B_BASE_REG, 0, devpriv->divisor2, 3);
		if(ret < 0)
		{
			comedi_error(dev, "error loading counter b0");
			return -1;
		}
		// load counter a0 in mode 2
		ret = i8254_load(dev->iobase + COUNTER_A_BASE_REG, 0, devpriv->divisor1, 2);
		if(ret < 0)
		{
			comedi_error(dev, "error loading counter a0");
			return -1;
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
	int lsb, msb, data;

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
		comedi_error(dev, "spurious interrupt");
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
	}else while(devpriv->status1_bits & DATA_AVAIL_BIT)
	{	// handle fifo-half-full or fifo-not-empty interrupt
		lsb = thisboard->read_byte(dev->iobase + ADC_FIFO_REG);
		msb = thisboard->read_byte(dev->iobase + ADC_FIFO_REG);
		data = (msb << 8) | lsb;
		comedi_buf_put(async, data);
		if(async->cmd.stop_src == TRIG_COUNT)
		{
			devpriv->count--;
			if(devpriv->count == 0) break;
		}
		devpriv->status1_bits = thisboard->read_byte(dev->iobase + STATUS1_REG);
	}

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

static void handle_isa_dma(comedi_device *dev)
{
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	int i;
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

	for(i = 0; i < num_points; i++)
	{
		/* write data point to comedi buffer */
		comedi_buf_put(async, devpriv->dma_buffer[i]);
		if(async->cmd.stop_src == TRIG_COUNT) devpriv->count--;
	}
	// re-enable  dma
	set_dma_addr(devpriv->dma_chan, virt_to_bus(devpriv->dma_buffer));
	set_dma_count(devpriv->dma_chan, leftover * sample_size);
	enable_dma(devpriv->dma_chan);
	release_dma_lock(flags);

	// clear dma tc interrupt
	thisboard->write_byte(0x1, dev->iobase + DMATC_CLEAR_REG);

	async->events |= COMEDI_CB_BLOCK;
}

static int labpc_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
 	int i, n;
	int chan;
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
	devpriv->command1_bits |= ADC_GAIN_BITS(CR_RANGE(insn->chanspec));
	chan = CR_CHAN(insn->chanspec);
	// XXX munge channel bits for differential mode
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
		if(CR_RANGE(insn->chanspec) & AI_RANGE_IS_UNIPOLAR)
			devpriv->command6_bits |= ADC_UNIP_BIT;
		else
			devpriv->command6_bits &= ~ADC_UNIP_BIT;
		// don't interrupt on fifo half full
		devpriv->command6_bits &= ~ADC_FHF_INTR_EN_BIT;
		// don't enable interrupt on counter a1 terminal count?
		devpriv->command6_bits &= ~A1_INTR_EN_BIT;
		// write to register
		thisboard->write_byte(devpriv->command6_bits, dev->iobase + COMMAND6_REG);
	}

	// setup command4 register
	devpriv->command4_bits = 0;
	devpriv->command4_bits |= EXT_CONVERT_DISABLE_BIT;
	// single-ended/differential
	if(CR_AREF(insn->chanspec) == AREF_DIFF)
		devpriv->command4_bits |= ADC_DIFF_BIT;
	thisboard->write_byte(devpriv->command4_bits, dev->iobase + COMMAND4_REG);

	// XXX init counter a0 to high state

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

// analog output insn for pcidas-1602 series
static int labpc_ao_winsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int channel, range;
	int lsb, msb;

	channel = CR_CHAN(insn->chanspec);

	// turn off pacing of analog output channel
	// XXX spinlock access (race with analog input)
	devpriv->command2_bits &= DAC_PACED_BIT(channel);
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

	devpriv->caldac[channel] = data[0];
	write_caldac(dev, channel + 3, data[0]);	// first caldac used by boards is number 3

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
	devpriv->eeprom_data[channel] = data[0];

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

static int labpc_dio_mem_callback(int dir, int port, int data, void *arg)
{
	unsigned long iobase = (int)arg;

	if(dir)
	{
		writeb(data, iobase + port);
		return 0;
	}else
	{
		return readb(iobase + port);
	}
}


static void labpc_load_calibration(comedi_device *dev)
{
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

/* XXX will need some locking if this function is to be called from multiple
 * subdevices */

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
	return 0;
}

// writes to 8 bit calibration dacs
static void write_caldac(comedi_device *dev, unsigned int channel, unsigned int value)
{
	unsigned int reordered_channel, i;
	const int num_channel_bits = 4;	// caldacs use 4 bit channel specification

	// clear caldac load bit and make sure we don't write to eeprom
	devpriv->command5_bits &= ~CALDAC_LOAD_BIT & ~EEPROM_EN_BIT & ~EEPROM_WRITE_UNPROTECT_BIT;
	udelay(1);
	thisboard->write_byte(devpriv->command5_bits, dev->iobase + COMMAND5_REG);

	// write 4 bit channel, LSB first
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

