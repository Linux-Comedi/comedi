/*
    comedi/drivers/adl_pci7250.c

    Hardware comedi driver for PCI7250 Adlink card
    Copyright (C) 2015 Ian Abbott <abbotti@mev.co.uk>

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
Driver: adl_pci7250
Description: Driver for the Adlink PCI-7250 relay output & digital input card
Devices: [ADLink] PCI-7250 (adl_pci7250) LPCI-7250 LPCIe-7250
Author: Ian Abbott <abbotti@mev.co.uk>
Status: experimental
Updated: Wed, 10 Jun 2015 15:36:20 +0000

The driver assumes that 3 PCI-7251 modules are fitted to the PCI-7250,
giving 32 channels of relay outputs and 32 channels of isolated digital
inputs.  That is also the case for the LPCI-7250 and LPCIe-7250 cards
although they do not physically support the PCI-7251 modules.

Not fitting the PCI-7251 modules shouldn't do any harm, but the extra
inputs and relay outputs won't work!

Configuration Options:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
  If bus/slot is not specified, the first supported
  PCI device found will be used.
*/

#include <linux/comedidev.h>
#include <linux/kernel.h>
#include "comedi_pci.h"

#define PCI_DEVICE_ID_PCI7250 0x7250

static DEFINE_PCI_DEVICE_TABLE(adl_pci7250_pci_table) = {
	{
		PCI_VENDOR_ID_ADLINK,
		PCI_DEVICE_ID_PCI7250,
		PCI_ANY_ID,
		PCI_ANY_ID,
		0,
		0,
		0
	},
	{0}
};

MODULE_DEVICE_TABLE(pci, adl_pci7250_pci_table);

struct adl_pci7250_private {
	struct pci_dev *pci_dev;
};

#define devpriv ((struct adl_pci7250_private *)dev->private)

static int adl_pci7250_attach(comedi_device *dev, comedi_devconfig *it);
static int adl_pci7250_detach(comedi_device *dev);
static comedi_driver driver_adl_pci7250 = {
	.driver_name = "adl_pci7250",
	.module = THIS_MODULE,
	.attach = adl_pci7250_attach,
	.detach = adl_pci7250_detach,
};

static int adl_pci7250_di_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data);

static int adl_pci7250_do_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data);

static int adl_pci7250_attach(comedi_device *dev, comedi_devconfig *it)
{
	struct pci_dev *pcidev;
	comedi_subdevice *s;
	int bus, slot;

	printk(KERN_INFO "comedi%d: adl_pci7250\n", dev->minor);

	dev->board_name = "pci7250";
	bus = it->options[0];
	slot = it->options[1];

	if (alloc_private(dev, sizeof(struct adl_pci7250_private)) < 0)
		return -ENOMEM;

	if (alloc_subdevices(dev, 2) < 0)
		return -ENOMEM;

	for (pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
		pcidev != NULL;
		pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pcidev)) {

		if (pcidev->vendor == PCI_VENDOR_ID_ADLINK &&
			pcidev->device == PCI_DEVICE_ID_PCI7250) {
			if (bus || slot) {
				/* requested particular bus/slot */
				if (pcidev->bus->number != bus ||
					PCI_SLOT(pcidev->devfn) != slot) {
					continue;
				}
			}
			devpriv->pci_dev = pcidev;
			break;
		}
	}
	if (pcidev == NULL) {
		printk(KERN_ERR "comedi%d: no supported board found! (req. bus/slot : %d/%d)\n",
			dev->minor, bus, slot);
		return -EIO;
	}
	if (comedi_pci_enable(pcidev, "adl_pci7250") < 0) {
		printk(KERN_ERR "comedi%d: Failed to enable PCI device and request regions\n",
			dev->minor);
		return -EIO;
	}
	dev->iobase = pci_resource_start(pcidev, 2);
	printk(KERN_DEBUG "comedi: base addr %4lx\n", dev->iobase);

	s = dev->subdevices + 0;
	/* Relay DO */
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITABLE | SDF_GROUND | SDF_COMMON;
	s->n_chan = 32;	/* assume PCI-7251 modules are fitted */
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = adl_pci7250_do_insn_bits;
	/* read initial state of relays */
	s->state = inb(dev->iobase + 0) | (inb(dev->iobase + 2) << 8) |
		(inb(dev->iobase + 4) << 16) | (inb(dev->iobase + 6) << 24);

	s = dev->subdevices + 1;
	/* Isolated DI */
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON;
	s->n_chan = 32;	/* assume PCI-7251 modules are fitted */
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = adl_pci7250_di_insn_bits;

	printk(KERN_DEBUG "comedi: attached\n");

	return 1;
}

static int adl_pci7250_detach(comedi_device *dev)
{
	printk(KERN_DEBUG "comedi%d: pci7250: remove\n", dev->minor);

	if (devpriv && devpriv->pci_dev) {
		if (dev->iobase)
			comedi_pci_disable(devpriv->pci_dev);
		pci_dev_put(devpriv->pci_dev);
	}

	return 0;
}

static int adl_pci7250_do_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	if (data[0]) {
		s->state &= ~data[0];
		s->state |= (data[0] & data[1]);

		if (data[0] & 0xff)
			outb(s->state & 0xff, dev->iobase + 0);
		if (data[0] & 0xff00)
			outb((s->state >> 8) & 0xff, dev->iobase + 2);
		if (data[0] & 0xff0000)
			outb((s->state >> 16) & 0xff, dev->iobase + 4);
		if (data[0] & 0xff000000)
			outb((s->state >> 24) & 0xff, dev->iobase + 6);
	}

	data[1] = s->state;

	return 2;
}

static int adl_pci7250_di_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{

	data[1] = inb(dev->iobase + 1) | (inb(dev->iobase + 3) << 8) |
		(inb(dev->iobase + 5) << 16) | (inb(dev->iobase + 7) << 24);

	return 2;
}

COMEDI_PCI_INITCLEANUP(driver_adl_pci7250, adl_pci7250_pci_table);
