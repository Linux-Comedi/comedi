/*
    cb_pcidas64.c
    This is a driver for the ComputerBoards/MeasurementComputing PCI-DAS
    64xxx cards.

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
	quite experimental and unfinished

TODO:
	just about everything
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

// bit in hexadecimal representation of range index that indicates unipolar input range
#define IS_UNIPOLAR 0x4
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
// XXX supports more boards...
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
static int ai_cmd(comedi_device *dev,comedi_subdevice *s);
static int ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);
static int ao_cmd(comedi_device *dev,comedi_subdevice *s);
static int ao_inttrig(comedi_device *dev, comedi_subdevice *subdev, unsigned int trig_num);
static int ao_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);
static void handle_interrupt(int irq, void *d, struct pt_regs *regs);
static int ai_cancel(comedi_device *dev, comedi_subdevice *s);
static int ao_cancel(comedi_device *dev, comedi_subdevice *s);

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

	printk("No supported ComputerBoards/MeasurementComputing card found on "
		"requested position\n");
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

#ifdef PCIDAS64_DEBUG

printk("plx9080 phys io addr 0x%lx\n", devpriv->plx9080_iobase);
printk("main phys io addr 0x%lx\n", devpriv->main_iobase);
printk("diocounter phys io addr 0x%lx\n", devpriv->dio_counter_iobase);
printk("irq %i\n", dev->irq);

#endif

	// remap, won't work with 2.0 kernels but who cares
	devpriv->plx9080_iobase = (unsigned long)ioremap(plx9080_iobase, PLX9080_IOSIZE);
	devpriv->main_iobase = (unsigned long)ioremap(main_iobase, MAIN_IOSIZE);
	devpriv->dio_counter_iobase = (unsigned long)ioremap(dio_counter_iobase, DIO_COUNTER_IOSIZE);

	// get irq
	if(comedi_request_irq(pcidev->irq, handle_interrupt, SA_SHIRQ, "cb_pcidas64", dev ))
	{
		printk(" unable to allocate irq %d\n", pcidev->irq);
		return -EINVAL;
	}
	dev->irq = pcidev->irq;

#ifdef PCIDAS64_DEBUG

printk("plx9080 virt io addr 0x%lx\n", devpriv->plx9080_iobase);
printk("main virt io addr 0x%lx\n", devpriv->main_iobase);
printk("diocounter virt io addr 0x%lx\n", devpriv->dio_counter_iobase);
printk("irq %i\n", dev->irq);

#endif

/*
 * Allocate the subdevice structures.
 */
	dev->n_subdevices = 5;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog input subdevice */
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON | SDF_DIFF;
	/* WARNING: Number of inputs in differential mode is ignored */
	s->n_chan = thisboard->ai_se_chans;
	s->len_chanlist = 8092;
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = &ai_ranges;
//	s->insn_read = ai_rinsn;
//	s->do_cmd = ai_cmd;
//	s->do_cmdtest = ai_cmdtest;
//	s->cancel = ai_cancel;

	/* analog output subdevice */
	s = dev->subdevices + 1;
	dev->write_subdev = s;
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_GROUND;
	s->n_chan = thisboard->ao_nchan;
	// analog out resolution is the same as analog input resolution, so use ai_bits
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = &ao_ranges;
//	s->insn_read = ao_readback_insn;
//	s->insn_write = ao_winsn;
//	s->do_cmdtest = ao_cmdtest;
//	s->do_cmd = ao_cmd;
	s->len_chanlist = thisboard->ao_nchan;
//	s->cancel = ao_cancel;

	// digital input
	s = dev->subdevices + 2;
	s->type = COMEDI_SUBD_UNUSED;

	// digital output
	s = dev->subdevices + 3;
	s->type = COMEDI_SUBD_UNUSED;

	/* 8255 */
	s = dev->subdevices + 4;
	s->type = COMEDI_SUBD_UNUSED;
//	subdev_8255_init(dev, s, );

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
//	if(dev->subdevices)
//		subdev_8255_cleanup(dev,dev->subdevices + 4);

	return 0;
}

static int ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	return -1;
}

static int ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	return -1;
}

static int ao_readback_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	return -1;
}

static int ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	return -1;
}

static int ai_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd)
{
	return -1;
}

static int ao_cmd(comedi_device *dev,comedi_subdevice *s)
{
	return -1;
}

static int ao_inttrig(comedi_device *dev, comedi_subdevice *subdev, unsigned int trig_num)
{
	return -1;
}

static int ao_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd)
{
	return -1;
}

static void handle_interrupt(int irq, void *d, struct pt_regs *regs)
{
}

static int ai_cancel(comedi_device *dev, comedi_subdevice *s)
{
	return -1;
}

static int ao_cancel(comedi_device *dev, comedi_subdevice *s)
{
	return -1;
}



