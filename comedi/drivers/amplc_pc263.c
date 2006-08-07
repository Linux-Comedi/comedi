/*
    comedi/drivers/amplc_pc263.c
    Driver for Amplicon PC263 and PCI263 relay boards.

    Copyright (C) 2002 MEV Ltd. <http://www.mev.co.uk/>

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@schleef.org>

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
Driver: amplc_pc263.o
Description: Amplicon PC263, PCI263
Author: Ian Abbott <abbotti@mev.co.uk>
Devices: [Amplicon] PC263 (pc263), PCI263 (pci263)
Updated: Tue, 20 Aug 2002 11:41:01 +0100
Status: works

Configuration options - PC263:
  [0] - I/O port base address

Configuration options - PCI263:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
  If bus/slot is not specified, the first available PCI device will be
  used.

Each board appears as one subdevice, with 16 digital outputs, each
connected to a reed-relay. Relay contacts are closed when output is 1.
The state of the outputs can be read.
*/

#include <linux/comedidev.h>

#include <linux/pci.h>

#define PC263_DRIVER_NAME	"amplc_pc263"

/* PCI263 PCI configuration register information */
#define PCI_VENDOR_ID_AMPLICON 0x14dc
#define PCI_DEVICE_ID_AMPLICON_PCI263 0x000c


/* PC263 / PCI263 registers */
#define PC263_IO_SIZE	2


/*
 * Board descriptions for Amplicon PC263 / PCI263.
 */

enum pc263_bustype {isa_bustype, pci_bustype};
enum pc263_model {pc263_model, pci263_model};

typedef struct pc263_board_struct{
	char *name;
	char *fancy_name;
	enum pc263_bustype bustype;
	enum pc263_model model;
}pc263_board;
static pc263_board pc263_boards[] = {
	{
	name:		"pc263",
	fancy_name:	"PC263",
	bustype:	isa_bustype,
	model:		pc263_model,
	},
	{
	name:		"pci263",
	fancy_name:	"PCI263",
	bustype:	pci_bustype,
	model:		pci263_model,
	},
};

static struct pci_device_id pc263_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_AMPLICON_PCI263, PCI_ANY_ID, PCI_ANY_ID, 0, 0, pci263_model },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, pc263_pci_table);

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((pc263_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct{
	/* PCI device. */
	struct pci_dev *pci_dev;
}pc263_private;

#define devpriv ((pc263_private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int pc263_attach(comedi_device *dev,comedi_devconfig *it);
static int pc263_detach(comedi_device *dev);
static comedi_driver driver_amplc_pc263={
	driver_name:	PC263_DRIVER_NAME,
	module:		THIS_MODULE,
	attach:		pc263_attach,
	detach:		pc263_detach,
	board_name:	pc263_boards,
	offset:		sizeof(pc263_board),
	num_names:	sizeof(pc263_boards) / sizeof(pc263_board),
};

static int pc263_request_region(unsigned long from, unsigned long extent);
static int pc263_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int pc263_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int pc263_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	struct pci_dev *pci_dev = NULL;
	unsigned long iobase = 0;
	int bus = 0, slot = 0;
	struct pci_device_id *pci_id;
	int ret;

	printk("comedi%d: %s: ",dev->minor, PC263_DRIVER_NAME);
/*
 * Allocate the private structure area.  alloc_private() is a
 * convenient macro defined in comedidev.h.
 */
	if ((ret=alloc_private(dev,sizeof(pc263_private))) < 0) {
		printk("out of memory!\n");
		return ret;
	}
	/* Process options. */
	switch (thisboard->bustype) {
	case isa_bustype:
		iobase = it->options[0];
		break;
	case pci_bustype:
		bus = it->options[0];
		slot = it->options[1];

		/* Look for PCI table entry for this model. */
		for (pci_id = pc263_pci_table; pci_id->vendor != 0; pci_id++) {
			if (pci_id->driver_data == thisboard->model)
				break;
		}
		if (pci_id->vendor == 0) {
			printk("bug! cannot determine board type!\n");
			return -EINVAL;
		}

		/* Look for matching PCI device. */
		for(pci_dev = pci_get_device(pci_id->vendor, pci_id->device,
					NULL); pci_dev != NULL; 
				pci_dev = pci_get_device(pci_id->vendor,
					pci_id->device, pci_dev)) {
			/* If bus/slot specified, check them. */
			if (bus || slot) {
				if (bus != pci_dev->bus->number
						|| slot != PCI_SLOT(pci_dev->devfn))
					continue;
			}
#if 0
			if (pci_id->subvendor != PCI_ANY_ID) {
				if (pci_dev->subsystem_vendor != pci_id->subvendor)
					continue;
			}
			if (pci_id->subdevice != PCI_ANY_ID) {
				if (pci_dev->subsystem_device != pci_id->subdevice)
					continue;
			}
#endif
			if (((pci_dev->class ^ pci_id->class) & pci_id->class_mask) != 0)
				continue;
			/* Found a match. */
			devpriv->pci_dev = pci_dev;
			break;
		}
		if (!pci_dev) {
			printk("no %s found!\n", thisboard->fancy_name);
			return -EIO;
		}
		break;
	default:
		printk("bug! cannot determine board type!\n");
		return -EINVAL;
		break;
	}

/*
 * Initialize dev->board_name.
 */
	dev->board_name = thisboard->name;
	printk("%s ", dev->board_name);

	/* Enable device and reserve I/O spaces. */
	if (pci_dev) {
		if ((ret=pci_enable_device(pci_dev)) < 0) {
			printk("error enabling PCI device!\n");
			return ret;
		}
		if ((ret=pci_request_regions(pci_dev, PC263_DRIVER_NAME)) < 0) {
			printk("I/O port conflict (PCI)!\n");
			return ret;
		}
		iobase = pci_resource_start(pci_dev, 2);
	} else {
		if ((ret=pc263_request_region(iobase, PC263_IO_SIZE)) < 0) {
			return ret;
		}
	}
	dev->iobase = iobase;

/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.
 */
	if ((ret=alloc_subdevices(dev, 1)) < 0) {
		printk("out of memory!\n");
		return -ENOMEM;
	}

	s = dev->subdevices+0;
	/* digital i/o subdevice */
	s->type = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE|SDF_WRITABLE|SDF_RT;
	s->n_chan = 16;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = pc263_dio_insn_bits;
	s->insn_config = pc263_dio_insn_config;
	/* all outputs */
	s->io_bits = 0xffff;
	/* read initial relay state */
	s->state = inb(dev->iobase);
	s->state = s->state | (inb(dev->iobase) << 8);

	if (thisboard->bustype == isa_bustype) {
		printk("(base %#lx) ", iobase);
	} else {
		printk("(pci %s) ", pci_name(pci_dev));
	}
	
	printk("attached\n");

	return 1;
}


/*
 * _detach is called to deconfigure a device.  It should deallocate
 * resources.  
 * This function is also called when _attach() fails, so it should be
 * careful not to release resources that were not necessarily
 * allocated by _attach().  dev->private and dev->subdevices are
 * deallocated automatically by the core.
 */
static int pc263_detach(comedi_device *dev)
{
	printk("comedi%d: %s: remove\n", dev->minor, PC263_DRIVER_NAME);

	if (devpriv) {
		if (devpriv->pci_dev) {
			if(dev->iobase)
			{
				pci_release_regions(devpriv->pci_dev);
				pci_disable_device(devpriv->pci_dev);
			}
			pci_dev_put(devpriv->pci_dev);
		} else if (dev->iobase) {
			release_region(dev->iobase, PC263_IO_SIZE);
		}
	}

	return 0;
}

/*
 * This function checks and requests an I/O region, reporting an error
 * if there is a conflict.
 */
static int pc263_request_region(unsigned long from, unsigned long extent)
{
	if (!request_region(from, extent, PC263_DRIVER_NAME)) {
		printk("I/O port conflict (%#lx,%lu)!\n", from, extent);
		return -EIO;
	}
	return 0;
}

/* DIO devices are slightly special.  Although it is possible to
 * implement the insn_read/insn_write interface, it is much more
 * useful to applications if you implement the insn_bits interface.
 * This allows packed reading/writing of the DIO channels.  The
 * comedi core can convert between insn_bits and insn_read/write */
static int pc263_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=2)return -EINVAL;

	/* The insn data is a mask in data[0] and the new data
	 * in data[1], each channel cooresponding to a bit. */
	if(data[0]){
		s->state &= ~data[0];
		s->state |= data[0]&data[1];
		/* Write out the new digital output lines */
		outb(s->state & 0xFF, dev->iobase);
		outb(s->state >> 8, dev->iobase + 1);
	}

	/* on return, data[1] contains the value of the digital
	 * input and output lines. */
	/* or we could just return the software copy of the output values if
	 * it was a purely digital output subdevice */
	data[1]=s->state;

	return 2;
}

static int pc263_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=1)return -EINVAL;
	return 1;
}

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_amplc_pc263);

