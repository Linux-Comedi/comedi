/*
    cb_pcidas64.c
    This is a driver for the ComputerBoards/MeasurementComputing PCI-DAS
    64xxx cards.

    Author:  Frank Mori Hess <fmhess@uiuc.edu>

    Options:
    [0] - PCI bus number
    [1] - PCI slot number

    Copyright (C) 2001 Frank Mori Hess <fmhess@uiuc.edu>

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-8 David A. Schleef <ds@stm.lbl.gov>

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

STATUS:
	insn supported

TODO:
	command support
	calibration subdevice
	user counter subdevice
	there are a number of boards this driver could support, but does not since
		I don't know the pci device id numbers
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
#include <linux/comedidev.h>
#include "8253.h"
#include "8255.h"

#define PCIDAS64_DEBUG	// enable debugging code
//#undef PCIDAS64_DEBUG	// disable debugging code

// PCI vendor number of ComputerBoards/MeasurementComputing
#define PCI_VENDOR_CB	0x1307
#define TIMER_BASE 25	// 40MHz master clock

/* PCI-DAS64xxx base addresses */

// indices of base address regions
#define PLX9080_BADRINDEX 0
#define MAIN_BADRINDEX 2
#define DIO_COUNTER_BADRINDEX 3

#define PLX9080_IOSIZE 0xec
#define MAIN_IOSIZE 0x302
#define DIO_COUNTER_IOSIZE 0x29

// plx pci9080 configuration registers
// XXX could steal plx9060 header file from kernel

// devpriv->main_iobase registers
// write-only
#define INTR_ENABLE_REG	0x0	// interrupt enable register
#define HW_CONFIG_REG	0x2	// hardware config register
#define    HW_CONFIG_DUMMY_BITS	0x2400	// bits that don't do anything yet but are given default values
#define ADC_CONTROL0_REG	0x10	// adc control register 0
#define    ADC_ENABLE_BIT	0x8000	// master adc enable
#define ADC_CONTROL1_REG	0x12	// adc control register 1
#define    SW_NOGATE_BIT	0x40	// disables software gate of adc
#define ADC_CONVERT_REG	0x24	// initiates single conversion
#define ADC_QUEUE_CLEAR_REG	0x26	// clears adc queue
#define ADC_QUEUE_LOAD_REG	0x28	// loads adc queue
#define    CHAN_BITS(x)	((x) & 0x3f)
#define    GAIN_BITS(x)	(((x) & 0x3) << 8)	// translates range index to gain bits
#define    UNIP_BIT(x)	(((x) & 0x4) << 11)	// translates range index to unipolar/bipolar bit
#define    SE_BIT	0x1000	// single-ended/ differential bit
#define    QUEUE_EOS_BIT	0x8000	// queue end of scan
#define ADC_BUFFER_CLEAR_REG	0x2a
#define ADC_QUEUE_HIGH_REG	0x2c	// high channel for internal queue, use CHAN_BITS() macro above
#define DAC_CONTROL0_REG	0x50	// dac control register 0
#define    DAC_ENABLE_BIT	0x8000	// dac controller enable bit
#define DAC_CONTROL1_REG	0x52	// dac control register 0
#define    DAC_RANGE_BITS(channel, range)	(((range) & 0x3) << (2 * ((channel) & 0x1)))
#define    DAC_OUTPUT_ENABLE_BIT	0x80	// dac output enable bit
#define DAC_BUFFER_CLEAR_REG 0x66	// clear dac buffer
#define DAC_CONVERT_REG(channel)	((0x70) + (2 * ((channel) & 0x1)))
// read-only
#define HW_STATUS_REG	0x0
#define   ADC_BUSY_BIT	0x8
#define   HW_REVISION(x)	(((x) >> 12) & 0xf)
#define PIPE1_READ_REG	0x4

// devpriv->dio_counter_iobase registers
#define DIO_8255_OFFSET	0x0
#define DO_REG	0x20
#define DI_REG	0x28

// analog input ranges for most boards
static comedi_lrange ai_ranges =
{
	8,
	{
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2.5),
		BIP_RANGE(1.25),
		UNI_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(2.5),
		UNI_RANGE(1.25)
	}
};

// analog output ranges
static comedi_lrange ao_ranges =
{
	4,
	{
		BIP_RANGE(5),
		BIP_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(10),
	}
};

typedef struct pcidas64_board_struct
{
	char *name;
	int device_id;	// pci device id
	int ai_se_chans;	// number of ai inputs in single-ended mode
	int ai_bits;	// analog input resolution
	int ai_speed;	// fastest conversion period in ns
	int ao_nchan;	// number of analog out channels
	int ao_scan_speed;	// analog output speed (for a scan, not conversion)
} pcidas64_board;

static pcidas64_board pcidas64_boards[] =
{
	{
		name:		"pci-das6402/16",
		device_id:	0x1d,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	5000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das6402/12",	// XXX check
		device_id:	0x1e,
		ai_se_chans:	64,
		ai_bits:	12,
		ai_speed:	5000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m1/16",
		device_id:	0x35,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	1000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m2/16",
		device_id:	0x36,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	500,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m3/16",
		device_id:	0x37,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	333,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},

#if 0
	{
		name:		"pci-das6402/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	5000,
		ao_nchan: 0,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m1/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	1000,
		ao_nchan: 0,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m2/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	500,
		ao_nchan: 0,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m3/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	333,
		ao_nchan: 0,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m1/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	1000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m2/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	500,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m3/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	333,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
#endif

};
// Number of boards in cb_pcidas_boards
#define N_BOARDS	(sizeof(pcidas64_boards) / sizeof(pcidas64_board))

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((pcidas64_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct
{
	// base addresses (physical)
	unsigned long plx9080_phys_iobase;
	unsigned long main_phys_iobase;
	unsigned long dio_counter_phys_iobase;
	// base addresses (ioremapped)
	unsigned long plx9080_iobase;
	unsigned long main_iobase;
	unsigned long dio_counter_iobase;
	// divisor of master clock for analog input pacing
	unsigned int ai_divisor;
	volatile unsigned int ai_count;	// number of analog input samples remaining
	// divisors of master clock for analog output pacing
	unsigned int ao_divisor;
	volatile unsigned int ao_count;	// number of analog output samples remaining
	unsigned int ao_value[2];	// remember what the analog outputs are set to, to allow readback
	unsigned int hw_revision;	// stc chip hardware revision number
	unsigned int do_bits;	// remember digital ouput levels
} pcidas64_private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((pcidas64_private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int attach(comedi_device *dev,comedi_devconfig *it);
static int detach(comedi_device *dev);
comedi_driver driver_cb_pcidas={
	driver_name:	"cb_pcidas64",
	module:		THIS_MODULE,
	attach:		attach,
	detach:		detach,
};

static int ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int ao_readback_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
//static int ai_cmd(comedi_device *dev,comedi_subdevice *s);
//static int ai_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd);
//static int ao_cmd(comedi_device *dev,comedi_subdevice *s);
//static int ao_inttrig(comedi_device *dev, comedi_subdevice *subdev, unsigned int trig_num);
//static int ao_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd);
//static void handle_interrupt(int irq, void *d, struct pt_regs *regs);
//static int ai_cancel(comedi_device *dev, comedi_subdevice *s);
//static int ao_cancel(comedi_device *dev, comedi_subdevice *s);
static int dio_callback(int dir, int port, int data, void *arg);
static int di_rbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int do_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_cb_pcidas);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.
 */
static int attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	struct pci_dev* pcidev;
	int index;
	// base addresses
	unsigned long plx9080_iobase;
	unsigned long main_iobase;
	unsigned long dio_counter_iobase;

	printk("comedi%d: cb_pcidas64\n",dev->minor);

/*
 * Allocate the private structure area.
 */
	if(alloc_private(dev,sizeof(pcidas64_private)) < 0)
		return -ENOMEM;

/*
 * Probe the device to determine what device in the series it is.
 */

	pci_for_each_dev(pcidev)
	{
		// is it not a computer boards card?
		if(pcidev->vendor != PCI_VENDOR_CB)
			continue;
#ifdef PCIDAS64_DEBUG
		printk(" found computer boards device id 0x%x on bus %i slot %i\n",
			pcidev->device, pcidev->bus->number, PCI_SLOT(pcidev->devfn));
#endif
		// loop through cards supported by this driver
		for(index = 0; index < N_BOARDS; index++)
		{
			if(pcidas64_boards[index].device_id != pcidev->device)
				continue;
			// was a particular bus/slot requested?
			if(it->options[0] || it->options[1])
			{
				// are we on the wrong bus/slot?
				if(pcidev->bus->number != it->options[0] ||
				   PCI_SLOT(pcidev->devfn) != it->options[1])
				{
					continue;
				}
			}
			dev->board_ptr = pcidas64_boards + index;
			goto found;
		}
	}

	printk("No supported ComputerBoards/MeasurementComputing card found\n");
	return -EIO;

found:

	printk("Found %s on bus %i, slot %i\n", pcidas64_boards[index].name,
		pcidev->bus->number, PCI_SLOT(pcidev->devfn));

	//Initialize dev->board_name
	dev->board_name = thisboard->name;

	/* Initialize devpriv->control_status and devpriv->adc_fifo to point to
	 * their base address.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
	plx9080_iobase =
		pcidev->base_address[PLX9080_BADRINDEX] &
		PCI_BASE_ADDRESS_MEM_MASK;
	main_iobase =
		pcidev->base_address[MAIN_BADRINDEX] &
		PCI_BASE_ADDRESS_MEM_MASK;
	dio_counter_iobase =
		pcidev->base_address[DIO_COUNTER_BADRINDEX] &
		PCI_BASE_ADDRESS_MEM_MASK;
#else
	if(pci_enable_device(pcidev))
		return -EIO;
	pci_set_master(pcidev);
	plx9080_iobase =
		pcidev->resource[PLX9080_BADRINDEX].start &
		PCI_BASE_ADDRESS_MEM_MASK;
	main_iobase =
		pcidev->resource[MAIN_BADRINDEX].start &
		PCI_BASE_ADDRESS_MEM_MASK;
	dio_counter_iobase =
		pcidev->resource[DIO_COUNTER_BADRINDEX].start &
		PCI_BASE_ADDRESS_MEM_MASK;

#if KERNEL_VERSION_CODE >= KERNEL_VERSION(2,3,17)

	if(check_mem_region(plx9080_iobase, PLX9080_IOSIZE))
	{
		/* Couldn't allocate io space */
		printk(KERN_WARNING "couldn't allocate IO space\n");
		return -EIO;
	}
	if(check_mem_region(main_iobase, MAIN_IOSIZE))
	{
		/* Couldn't allocate io space */
		printk(KERN_WARNING "couldn't allocate IO space\n");
		return -EIO;
	}
	if(check_mem_region(dio_counter_iobase, DIO_COUNTER_IOSIZE))
	{
		/* Couldn't allocate io space */
		printk(KERN_WARNING "couldn't allocate IO space\n");
		return -EIO;
	}

	request_mem_region(plx_iobase, PLX9080_IOSIZE, "cb_pcidas64");
	devpriv->plx9080_phys_iobase = dio_counter_iobase;
	request_mem_region(main_iobase, MAIN_IOSIZE, "cb_pcidas64");
	devpriv->main_phys_iobase = dio_counter_iobase;
	request_mem_region(dio_counter_iobase, DIO_COUNTER_IOSIZE, "cb_pcidas64");
	devpriv->dio_counter_phys_iobase = dio_counter_iobase;

#endif
#endif

	// remap, won't work with 2.0 kernels but who cares
	devpriv->plx9080_iobase = (unsigned long)ioremap(plx9080_iobase, PLX9080_IOSIZE);
	devpriv->main_iobase = (unsigned long)ioremap(main_iobase, MAIN_IOSIZE);
	devpriv->dio_counter_iobase = (unsigned long)ioremap(dio_counter_iobase, DIO_COUNTER_IOSIZE);

	devpriv->hw_revision = HW_REVISION(readw(devpriv->main_iobase + HW_STATUS_REG));

	// get irq
/*	if(comedi_request_irq(pcidev->irq, handle_interrupt, SA_SHIRQ, "cb_pcidas64", dev ))
	{
		printk(" unable to allocate irq %d\n", pcidev->irq);
		return -EINVAL;
	}
	dev->irq = pcidev->irq;
*/
#ifdef PCIDAS64_DEBUG

printk(" plx9080 phys io addr 0x%lx\n", devpriv->plx9080_phys_iobase);
printk(" main phys io addr 0x%lx\n", devpriv->main_phys_iobase);
printk(" diocounter phys io addr 0x%lx\n", devpriv->dio_counter_phys_iobase);
printk(" irq %i\n", dev->irq);

printk(" plx9080 virt io addr 0x%lx\n", devpriv->plx9080_iobase);
printk(" main virt io addr 0x%lx\n", devpriv->main_iobase);
printk(" diocounter virt io addr 0x%lx\n", devpriv->dio_counter_iobase);
printk(" irq %i\n", dev->irq);

printk(" stc hardware revision %i\n", devpriv->hw_revision);

#endif


/*
 * Allocate the subdevice structures.
 */
	dev->n_subdevices = 7;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog input subdevice */
//	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON | SDF_DIFF;
	/* WARNING: Number of inputs in differential mode is ignored */
	s->n_chan = thisboard->ai_se_chans;
	s->len_chanlist = 8092;
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = &ai_ranges;
	s->insn_read = ai_rinsn;
//	s->do_cmd = ai_cmd;
//	s->do_cmdtest = ai_cmdtest;
//	s->cancel = ai_cancel;

	/* analog output subdevice */
	s = dev->subdevices + 1;
	if(thisboard->ao_nchan)
	{
	//	dev->write_subdev = s;
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_GROUND;
		s->n_chan = thisboard->ao_nchan;
		// analog out resolution is the same as analog input resolution, so use ai_bits
		s->maxdata = (1 << thisboard->ai_bits) - 1;
		s->range_table = &ao_ranges;
		s->insn_read = ao_readback_insn;
		s->insn_write = ao_winsn;
	//	s->do_cmdtest = ao_cmdtest;
	//	s->do_cmd = ao_cmd;
	//	s->len_chanlist = thisboard->ao_nchan;
	//	s->cancel = ao_cancel;
	} else
	{
		s->type = COMEDI_SUBD_UNUSED;
	}

	// digital input
	s = dev->subdevices + 2;
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = di_rbits;

	// digital output
	s = dev->subdevices + 3;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE | SDF_READABLE;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = do_wbits;

	/* 8255 */
	s = dev->subdevices + 4;
	subdev_8255_init(dev, s, dio_callback,
		(void*) (devpriv->dio_counter_iobase + DIO_8255_OFFSET));

	// user counter subd XXX
	s = dev->subdevices + 5;
	s->type = COMEDI_SUBD_UNUSED;

	// calibration subd XXX
	s = dev->subdevices + 6;
	s->type = COMEDI_SUBD_UNUSED;

	return 0;
}


/*
 * _detach is called to deconfigure a device.  It should deallocate
 * resources.
 * This function is also called when _attach() fails, so it should be
 * careful not to release resources that were not necessarily
 * allocated by _attach().  dev->private and dev->subdevices are
 * deallocated automatically by the core.
 */
static int detach(comedi_device *dev)
{
	printk("comedi%d: cb_pcidas: remove\n",dev->minor);

	if(devpriv)
	{
		if(devpriv->plx9080_iobase)
			iounmap((void*)devpriv->plx9080_iobase);
		if(devpriv->main_iobase)
			iounmap((void*)devpriv->main_iobase);
		if(devpriv->dio_counter_iobase)
			iounmap((void*)devpriv->dio_counter_iobase);
#if KERNEL_VERSION_CODE >= KERNEL_VERSION(2,3,17)
		if(devpriv->plx9080_phys_iobase)
			release_mem_region(devpriv->plx9080_iobase, PLX9080_IOSIZE);
		if(devpriv->main_iobase)
			release_mem_region(devpriv->main_phys_iobase, MAIN_IOSIZE);
		if(devpriv->dio_counter_iobase)
			release_mem_region(devpriv->dio_counter_phys_iobase, DIO_COUNTER_IOSIZE);
#endif
	}
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);
	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices + 4);

	return 0;
}

static int ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	unsigned int bits, n, i;
	const int timeout = 1000;

	// disable card's interrupt sources
	writew(0, devpriv->main_iobase + INTR_ENABLE_REG);

	// use internal queue
	writew(HW_CONFIG_DUMMY_BITS, devpriv->main_iobase + HW_CONFIG_REG);

	/* disable pacing, triggering, etc */
	writew(ADC_ENABLE_BIT, devpriv->main_iobase + ADC_CONTROL0_REG);
	writew(0, devpriv->main_iobase + ADC_CONTROL1_REG);

	// load internal queue
	bits = 0;
	// set channel
	bits |= CHAN_BITS(CR_CHAN(insn->chanspec));
	// set gain
	bits |= GAIN_BITS(CR_RANGE(insn->chanspec));
	// set unipolar / bipolar
	bits |= UNIP_BIT(CR_RANGE(insn->chanspec));
	// set single-ended / differential
	if(CR_AREF(insn->chanspec) != AREF_DIFF)
		bits |= SE_BIT;
	// set stop channel
	writew(CHAN_BITS(CR_CHAN(insn->chanspec)), devpriv->main_iobase + ADC_QUEUE_HIGH_REG);
	// set start channel, and rest of settings
	writew(bits, devpriv->main_iobase + ADC_QUEUE_LOAD_REG);

	// clear adc buffer
	writew(0, devpriv->main_iobase + ADC_BUFFER_CLEAR_REG);

	for(n = 0; n < insn->n; n++)
	{
		/* trigger conversion */
		writew(0, devpriv->main_iobase + ADC_CONVERT_REG);

		// wait for data
		for(i = 0; i < timeout; i++)
		{
			if(!(readw(devpriv->main_iobase + HW_STATUS_REG) & ADC_BUSY_BIT))
				break;
		}
		if(i == timeout)
		{
			comedi_error(dev, " analog input read insn timed out");
			return -ETIME;
		}
		data[n] = readw(devpriv->main_iobase + PIPE1_READ_REG);
	}

	return n;
}

static int ao_winsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int chan = CR_CHAN(insn->chanspec);
	int range = CR_RANGE(insn->chanspec);
	unsigned int bits;

	// do some initializing
	writew(DAC_ENABLE_BIT, devpriv->main_iobase + DAC_CONTROL0_REG);

	// set range
	bits = DAC_OUTPUT_ENABLE_BIT;
	bits |= DAC_RANGE_BITS(chan, range);
	writew(bits, devpriv->main_iobase + DAC_CONTROL1_REG);

	// clear buffer
	writew(0, devpriv->main_iobase + DAC_BUFFER_CLEAR_REG);

	// write to channel
	writew(data[0], devpriv->main_iobase + DAC_CONVERT_REG(chan));

	// remember output value
	devpriv->ao_value[chan] = data[0];

	return 1;
}

static int ao_readback_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	data[0] = devpriv->ao_value[CR_CHAN(insn->chanspec)];

	return 1;
}

static int dio_callback(int dir, int port, int data, void *arg)
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

static int di_rbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	lsampl_t bits;

	bits = readb(devpriv->dio_counter_iobase + DI_REG);
	bits &= 0xf;
	data[1] = bits;
	data[0] = 0;

	return 2;
}

static int do_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	lsampl_t wbits;

	data[0] &= 0xf;
	wbits = devpriv->do_bits;
	// zero bits we are going to change
	wbits &= ~data[0];
	// set new bits
	wbits |= data[0] & data[1];
	devpriv->do_bits = wbits;

	writeb(devpriv->do_bits, devpriv->dio_counter_iobase + DO_REG);

	data[1] = wbits;

	return 2;
}

