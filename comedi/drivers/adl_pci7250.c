/*
    comedi/drivers/adl_pci7250.c

    Hardware comedi driver for PCI7250 ADLINK card
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
Description: Driver for the ADLINK PCI-7250 relay output & digital input card
Devices: [ADLINK] PCI-7250 (adl_pci7250) LPCI-7250 LPCIe-7250
Author: Ian Abbott <abbotti@mev.co.uk>
Status: works
Updated: Wed, 17 Jun 2015 09:22:58 +0000

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

enum adl_pci7250_regtype { no_regtype, io_regtype, mmio_regtype };
struct adl_pci7250_region {
	union {
		unsigned long iobase;		/* I/O base address */
		unsigned char __iomem *membase;	/* Mapped MMIO base address */
	} u;
	enum adl_pci7250_regtype regtype;
};

struct adl_pci7250_private {
	struct pci_dev *pci_dev;
	struct adl_pci7250_region io;
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
static unsigned char adl_pci7250_read8(comedi_device *dev,
	unsigned int offset);
static void adl_pci7250_write8(comedi_device *dev, unsigned int offset,
	unsigned char val);

static int adl_pci7250_attach(comedi_device *dev, comedi_devconfig *it)
{
	struct pci_dev *pcidev;
	comedi_subdevice *s;
	int bus, slot;
	unsigned int max_chans = 32;	/* Assume PCI-7251 modules fitted. */
	unsigned int i;

	printk(KERN_INFO "comedi%d: adl_pci7250\n", dev->minor);

	dev->board_name = "pci7250";
	bus = it->options[0];
	slot = it->options[1];

	if (alloc_private(dev, sizeof(struct adl_pci7250_private)) < 0)
		return -ENOMEM;

	if (alloc_subdevices(dev, 2) < 0)
		return -ENOMEM;

	devpriv->io.regtype = no_regtype;
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
			if (pcidev->subsystem_device == 0x7000) {
				/*
				 * This is a newer LPCIe-7250 variant, and
				 * neither the LPCI-7250 nor the LPCIe-7250
				 * can have PCI-7251 modules fitted, so we
				 * can limit the number of channels to 8.
				 */
				max_chans = 8;
			}
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
	/* Register mapping. */
	{
		resource_size_t base, len;
		const unsigned bar = 2;

		if ((pci_resource_flags(pcidev, bar) & IORESOURCE_MEM) != 0)
			devpriv->io.regtype = mmio_regtype;
		else
			devpriv->io.regtype = io_regtype;

		base = pci_resource_start(pcidev, bar);
		len = pci_resource_len(pcidev, bar);
		if (len < 8) {
			printk(KERN_ERR
				"comedi%d: PCI region size too small!\n",
				dev->minor);
			return -EIO;
		}
		if (devpriv->io.regtype == mmio_regtype) {
			printk(KERN_DEBUG "comedi%d: base mmio addr %08lx\n",
				dev->minor, (unsigned long)base);
			devpriv->io.u.membase = ioremap(base, len);
			if (!devpriv->io.u.membase) {
				printk(KERN_ERR
					"comedi%d: failed to remap registers!\n",
					dev->minor);
				return -ENOMEM;
			}
		} else {
			printk(KERN_DEBUG "comedi%d: base port addr %04lx\n",
				dev->minor, (unsigned long)base);
			devpriv->io.u.iobase = (unsigned long)base;
		}
	}

	s = dev->subdevices + 0;
	/* Relay DO */
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITABLE | SDF_GROUND | SDF_COMMON;
	s->n_chan = max_chans;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = adl_pci7250_do_insn_bits;
	/* read initial state of relays from the even offset registers */
	s->state = 0;
	for (i = 0; i * 8 < max_chans; i++) {
		s->state |=
			(unsigned int)adl_pci7250_read8(dev, i * 2) << (i * 8);
	}

	s = dev->subdevices + 1;
	/* Isolated DI */
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON;
	s->n_chan = max_chans;
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
		if (devpriv->io.regtype == mmio_regtype)
			iounmap(devpriv->io.u.membase);
		if (devpriv->io.regtype != no_regtype)
			comedi_pci_disable(devpriv->pci_dev);
		pci_dev_put(devpriv->pci_dev);
	}

	return 0;
}

static int adl_pci7250_do_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	if (data[0]) {
		unsigned int state = s->state;
		unsigned int bmask = data[0];
		unsigned int i;

		state &= ~bmask;
		state |= (bmask & data[1]);
		if (s->n_chan < 32)
			state &= ((1u << s->n_chan) - 1);
		s->state = state;
		for (i = 0; i * 8 < s->n_chan; i++) {
			if ((bmask & 0xffu) != 0) {
				/* write relay data to even offset registers */
				adl_pci7250_write8(dev, i * 2, state & 0xffu);
			}
			state >>= 8;
			bmask >>= 8;
		}
	}

	data[1] = s->state;

	return 2;
}

static int adl_pci7250_di_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	unsigned int value = 0;
	unsigned int i;

	for (i = 0; i * 8 < s->n_chan; i++) {
		/* read DI value from odd offset registers */
		value |= (unsigned int)adl_pci7250_read8(dev, i * 2 + 1) <<
			(i * 8);
	}

	data[1] = value;

	return 2;
}

/*
 * Read 8-bit register.
 */
static unsigned char adl_pci7250_read8(comedi_device *dev, unsigned int offset)
{
	if (devpriv->io.regtype == io_regtype)
		return inb(devpriv->io.u.iobase + offset);
	else
		return readb(devpriv->io.u.membase + offset);
}

/*
 * Write 8-bit register.
 */
static void adl_pci7250_write8(comedi_device *dev, unsigned int offset,
	unsigned char val)
{
	if (devpriv->io.regtype == io_regtype)
		outb(val, devpriv->io.u.iobase + offset);
	else
		writeb(val, devpriv->io.u.membase + offset);
}

COMEDI_PCI_INITCLEANUP(driver_adl_pci7250, adl_pci7250_pci_table);
