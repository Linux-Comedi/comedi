/*
    comedi/drivers/adl_pci6208.c

    Hardware driver for ADLINK 6208 series cards:
	card	     | voltage output    | current output
	-------------+-------------------+---------------
	PCI-6208V    |  8 channels       | -
	PCI-6216V    | 16 channels       | -
	PCI-6208A    |  8 channels       | 8 channels

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
Driver: adl_pci6208
Description: ADLINK PCI-6216V
Devices: [ADLINK] PCI-6216V (adl_pci6208)
Author: nsyeow <nsyeow@pd.jaring.my>
Updated: Tue, 10 Feb 2015 15:29:55 +0000
Status: untested

Configuration Options:
  none

The driver should work for PCI-6208V, PCI-6208A and PCI-6216V, but all
devices will be treated as a PCI-6216V.

For PCI-6208V and PCI-6208A, only AO channels 0 to 7 are connected and
AO channels 8 to 15 will behave as "phantom" outputs.

The current output ranges for PCI-6208A are not supported.  Only Comedi
sample values 0x8000 to 0xffff should be written to the AO channels on a
PCI-6208A.  Its voltage to current daughter board (EXP-8A) only supports
an input range of 0 to 10 volts and negative voltages may damage the
board.  Comedi sample values 0x0000 to 0x7fff would produce negative
voltages from -10 to 0 volts.

References:
	- ni_660x.c
	- adl_pci9111.c		copied the entire pci setup section
	- adl_pci9118.c
*/
/*
 * These headers should be followed by a blank line, and any comments
 * you wish to say about the driver.  The comment area is the place
 * to put any known bugs, limitations, unsupported features, supported
 * command triggers, whether or not commands are supported on particular
 * subdevices, etc.
 *
 * Somewhere in the comment should be information about configuration
 * options that are used with comedi_config.
 */
#include <linux/comedidev.h>
#include "comedi_pci.h"

#define PCI6208_DRIVER_NAME 	"adl_pci6208"

/* Board descriptions */
typedef struct {
	const char *name;
	unsigned short dev_id;	/* `lspci` will show you this */
	int ao_chans;
} pci6208_board;
static const pci6208_board pci6208_boards[] = {
#if 0
	{
		name:		"pci6208v",
		dev_id:		0x6208,
		ao_chans:	8
	},
#endif
#if 1
	{
		name:		"pci6216v",
		dev_id:		0x6208,
		ao_chans:	16
	},
#endif
#if 0
	{
		name:		"pci6208a",
		dev_id:		0x6208,
		ao_chans:	8
	},
#endif
};

/* This is used by modprobe to translate PCI IDs to drivers.  Should
 * only be used for PCI and ISA-PnP devices */
static DEFINE_PCI_DEVICE_TABLE(pci6208_pci_table) = {
	{PCI_VENDOR_ID_ADLINK, 0x6208, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0}
};

MODULE_DEVICE_TABLE(pci, pci6208_pci_table);

/* Will be initialized in pci6208_find device(). */
#define thisboard ((const pci6208_board *)dev->board_ptr)

typedef struct {
	int data;
	struct pci_dev *pci_dev;	/* for a PCI device */
	lsampl_t ao_readback[2];	/* Used for AO readback */
} pci6208_private;

#define devpriv ((pci6208_private *)dev->private)

static int pci6208_attach(comedi_device * dev, comedi_devconfig * it);
static int pci6208_attach_common(comedi_device * dev);
static int pci6208_auto_attach(comedi_device * dev, unsigned long context);
static int pci6208_detach(comedi_device * dev);

#define pci6208_board_nbr ARRAY_SIZE(pci6208_boards)

static comedi_driver driver_pci6208 = {
      .driver_name = PCI6208_DRIVER_NAME,
      .module = THIS_MODULE,
      .attach =pci6208_attach,
      .auto_attach = pci6208_auto_attach,
      .detach = pci6208_detach,
};

COMEDI_PCI_INITCLEANUP(driver_pci6208, pci6208_pci_table);

static int pci6208_find_device(comedi_device * dev, int bus, int slot);
static int
pci6208_pci_setup(struct pci_dev *pci_dev, unsigned long *io_base_ptr,
	int dev_minor);

/*read/write functions*/
static int pci6208_ao_winsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int pci6208_ao_rinsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int pci6208_di_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data);
static int pci6208_do_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data);

static int pci6208_auto_attach(comedi_device *dev, unsigned long context_model)
{
	struct pci_dev *pci_dev = comedi_to_pci_dev(dev);
	int retval;

	printk("comedi%d: pci6208: attach pci %s\n", dev->minor,
		pci_name(pci_dev));

	retval = alloc_private(dev, sizeof(pci6208_private));
	if (retval < 0)
		return retval;

	/* pci_dev_get() call matches pci_dev_put() in pci6208_detach() */
	devpriv->pci_dev = pci_dev_get(pci_dev);

	if (context_model >= pci6208_board_nbr)
		return -EINVAL;

	dev->board_ptr = &pci6208_boards[context_model];

	return pci6208_attach_common(dev);
}

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int pci6208_attach(comedi_device * dev, comedi_devconfig * it)
{
	int retval;

	printk("comedi%d: pci6208\n", dev->minor);

	retval = alloc_private(dev, sizeof(pci6208_private));
	if (retval < 0)
		return retval;

	retval = pci6208_find_device(dev, it->options[0], it->options[1]);
	if (retval < 0)
		return retval;

	return pci6208_attach_common(dev);
}

static int pci6208_attach_common(comedi_device *dev)
{
	comedi_subdevice *s;
	int retval;
	unsigned long io_base;

	retval = pci6208_pci_setup(devpriv->pci_dev, &io_base, dev->minor);
	if (retval < 0)
		return retval;

	dev->iobase = io_base;
	dev->board_name = thisboard->name;

/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.
 */
	if (alloc_subdevices(dev, 3) < 0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog output subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITABLE;	//anything else to add here??
	s->n_chan = thisboard->ao_chans;
	s->maxdata = 0xffff;	//16-bit DAC
	s->range_table = &range_bipolar10;	//this needs to be checked.
	s->insn_write = pci6208_ao_winsn;
	s->insn_read = pci6208_ao_rinsn;

	s = dev->subdevices + 1;
	/* digital input subdevice */
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = pci6208_di_insn_bits;

	s = dev->subdevices + 2;
	/* digital output subdevice */
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITABLE;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = pci6208_do_insn_bits;
	/* read back initial state of digital outputs */
	s->state = inw(dev->iobase + 0x40) & 0xf;

	printk("comedi: attached\n");

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
static int pci6208_detach(comedi_device * dev)
{
	printk("comedi%d: pci6208: remove\n", dev->minor);

	if (devpriv && devpriv->pci_dev) {
		if (dev->iobase) {
			comedi_pci_disable(devpriv->pci_dev);
		}
		pci_dev_put(devpriv->pci_dev);
	}

	return 0;
}

static int pci6208_ao_winsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i, Data_Read;
	unsigned short chan = CR_CHAN(insn->chanspec);
	lsampl_t invert = 1 << (16 - 1);
	lsampl_t out_value;

	/* Writing a list of values to an AO channel is probably not
	 * very useful, but that's how the interface is defined. */
	for (i = 0; i < insn->n; i++) {
		out_value = data[i];
		/* a typical programming sequence */
		do {
			Data_Read = (inw(dev->iobase) & 1);
		} while (Data_Read);
		outw(out_value ^ invert, dev->iobase + (0x02 * chan));
		devpriv->ao_readback[chan] = out_value;
	}

	/* return the number of samples read/written */
	return i;
}

/* AO subdevices should have a read insn as well as a write insn.
 * Usually this means copying a value stored in devpriv. */
static int pci6208_ao_rinsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++)
		data[i] = devpriv->ao_readback[chan];

	return i;
}

static int pci6208_di_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	data[1] = (inw(dev->iobase + 0x40) & 0xf0) >> 4;
	return insn->n;
}

static int pci6208_do_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	if (data[0]) {
		unsigned int mask = data[0] & 0xf;

		s->state = (s->state & ~mask) | (data[1] & mask);
		outw(s->state, dev->iobase + 0x40);
	}
	data[1] = s->state;
	return insn->n;
}

static int pci6208_find_device(comedi_device * dev, int bus, int slot)
{
	struct pci_dev *pci_dev;
	int i;

	for (pci_dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
		pci_dev != NULL;
		pci_dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pci_dev)) {
		if (pci_dev->vendor == PCI_VENDOR_ID_ADLINK) {
			for (i = 0; i < pci6208_board_nbr; i++) {
				if (pci6208_boards[i].dev_id == pci_dev->device) {
					// was a particular bus/slot requested?
					if ((bus != 0) || (slot != 0)) {
						// are we on the wrong bus/slot?
						if (pci_dev->bus->number
							!= bus ||
							PCI_SLOT(pci_dev->devfn)
							!= slot) {
							continue;
						}
					}
					dev->board_ptr = pci6208_boards + i;
					goto found;
				}
			}
		}
	}

	printk("comedi%d: no supported board found! (req. bus/slot : %d/%d)\n",
		dev->minor, bus, slot);
	return -EIO;

      found:
	printk("comedi%d: found %s (b:s:f=%d:%d:%d) , irq=%d\n",
		dev->minor,
		pci6208_boards[i].name,
		pci_dev->bus->number,
		PCI_SLOT(pci_dev->devfn),
		PCI_FUNC(pci_dev->devfn), pci_dev->irq);

	// TODO: Warn about non-tested boards.
	//switch(board->device_id)
	//{
	//};

	devpriv->pci_dev = pci_dev;

	return 0;
}

static int
pci6208_pci_setup(struct pci_dev *pci_dev, unsigned long *io_base_ptr,
	int dev_minor)
{
	unsigned long io_base, io_range, lcr_io_base, lcr_io_range;

	// Enable PCI device and request regions
	if (comedi_pci_enable(pci_dev, PCI6208_DRIVER_NAME) < 0) {
		printk("comedi%d: Failed to enable PCI device and request regions\n", dev_minor);
		return -EIO;
	}
	// Read local configuration register base address [PCI_BASE_ADDRESS #1].
	lcr_io_base = pci_resource_start(pci_dev, 1);
	lcr_io_range = pci_resource_len(pci_dev, 1);

	printk("comedi%d: local config registers at address 0x%4lx [0x%4lx]\n",
		dev_minor, lcr_io_base, lcr_io_range);

	// Read PCI6208 register base address [PCI_BASE_ADDRESS #2].
	io_base = pci_resource_start(pci_dev, 2);
	io_range = pci_resource_end(pci_dev, 2) - io_base + 1;

	printk("comedi%d: 6208 registers at address 0x%4lx [0x%4lx]\n",
		dev_minor, io_base, io_range);

	*io_base_ptr = io_base;
	//devpriv->io_range = io_range;
	//devpriv->is_valid=0;
	//devpriv->lcr_io_base=lcr_io_base;
	//devpriv->lcr_io_range=lcr_io_range;

	return 0;
}
