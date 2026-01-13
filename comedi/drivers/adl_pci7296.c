/*
    comedi/drivers/adl_pci7296.c

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
Driver: adl_pci7296
Description: Driver for the ADLINK PCI-7296/7248/7224 digital I/O boards
Devices: [ADLINK] PCI-7296 (adl_pci7296 or pci7296),
  PCI-7248 (adl_pci7296 or pci7248), PCI-7224 (adl_pci7296 or pci7224)
Author: Jon Grierson <jd@renko.co.uk>
Updated: Tue, 26 Mar 2019 13:26:02 +0000
Status: testing

Configuration Options:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
  If bus/slot is not specified, the first supported
  PCI device found will be used.
*/

#include <linux/comedidev.h>
#include <linux/kernel.h>

#include "comedi_pci.h"
#include "8255.h"
// #include "8253.h"

#define DRIVER_NAME	"adl_pci7296"

#define PCI_DEVICE_ID_PCI7224 0x7224
#define PCI_DEVICE_ID_PCI7248 0x7248
#define PCI_DEVICE_ID_PCI7296 0x7296
#define PCI_DEVICE_ID_INVALID 0xffff

enum adl_pci7296_pci_model {
	pci7224_model,
	pci7248_model,
	pci7296_model,
	wildcard_model
};

static DEFINE_PCI_DEVICE_TABLE(adl_pci7296_pci_table) = {
	{ PCI_VDEVICE(ADLINK, PCI_DEVICE_ID_PCI7224), pci7224_model },
	{ PCI_VDEVICE(ADLINK, PCI_DEVICE_ID_PCI7248), pci7248_model },
	{ PCI_VDEVICE(ADLINK, PCI_DEVICE_ID_PCI7296), pci7296_model },
	{0}
};

MODULE_DEVICE_TABLE(pci, adl_pci7296_pci_table);

typedef struct {
	struct pci_dev *pci_dev;
} adl_pci7296_private;

#define devpriv ((adl_pci7296_private *)dev->private)

/*
 * Board descriptions.
 */
typedef struct adl_pci7296_board_struct {
	const char *name;
	unsigned short dev_id;
	unsigned short n_8255;
} adl_pci7296_board;

#define thisboard ((const adl_pci7296_board *)dev->board_ptr)

static const adl_pci7296_board adl_pci7296_boards[] = {
	[pci7224_model] = {
		.name   = "pci7224",
		.dev_id = PCI_DEVICE_ID_PCI7224,
		.n_8255 = 1,
	},
	[pci7248_model] = {
		.name   = "pci7248",
		.dev_id = PCI_DEVICE_ID_PCI7248,
		.n_8255 = 2,
	},
	[pci7296_model] = {
		.name   = "pci7296",
		.dev_id = PCI_DEVICE_ID_PCI7296,
		.n_8255 = 4,
	},
	[wildcard_model] = {
		/* wildcard */
		name:	DRIVER_NAME,
		dev_id:	PCI_DEVICE_ID_INVALID,
	}
};

static int adl_pci7296_attach(comedi_device * dev, comedi_devconfig * it);
static int adl_pci7296_auto_attach(comedi_device * dev, unsigned long context);
static int adl_pci7296_detach(comedi_device * dev);
static comedi_driver driver_adl_pci7296 = {
      .driver_name = DRIVER_NAME,
      .module = THIS_MODULE,
      .attach = adl_pci7296_attach,
      .auto_attach = adl_pci7296_auto_attach,
      .detach = adl_pci7296_detach,
      .board_name = &adl_pci7296_boards[0].name,
      .offset = sizeof(adl_pci7296_boards[0]),
      .num_names = ARRAY_SIZE(adl_pci7296_boards),
};

static int adl_pci7296_attach_common(comedi_device * dev)
{
	struct pci_dev *pcidev = devpriv->pci_dev;
	int i;
	int ret;

	dev->board_name = thisboard->name;

	/*
	 * Now we know the board type, allocate the subdevices, one
	 * subdevice per 8255 chip.
	 */
	if (alloc_subdevices(dev, thisboard->n_8255) < 0)
		return -ENOMEM;

	if (comedi_pci_enable(pcidev, thisboard->name)) {
		printk("comedi%d: Failed to enable PCI device and request regions\n", dev->minor);
		return -EIO;
	}

	dev->iobase = pci_resource_start(pcidev, 2);
	printk("comedi%d: base addr %4lx\n", dev->minor, dev->iobase);

	/* initialize the subdevices */
	for (i = 0; i < thisboard->n_8255; i++) {
		ret = subdev_8255_init(dev, dev->subdevices + i, NULL,
			dev->iobase + (i * 4));
		if (ret)
			return ret;
	}

	printk("comedi%d: attached\n", dev->minor);
	return 1;
}

static int adl_pci7296_attach(comedi_device * dev, comedi_devconfig * it)
{
	struct pci_dev *pcidev = NULL;
	int bus, slot;
	int i;

	printk("comedi: attempt to attach...\n");
	printk("comedi%d: %s\n", dev->minor, DRIVER_NAME);

	bus = it->options[0];
	slot = it->options[1];

	if (alloc_private(dev, sizeof(adl_pci7296_private)) < 0)
		return -ENOMEM;

	for (pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
		pcidev != NULL;
		pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pcidev)) {

		if (bus || slot) {
			/* requested particular bus/slot */
			if (pcidev->bus->number != bus
				|| PCI_SLOT(pcidev->devfn) != slot) {
				continue;
			}
		}

		if (pcidev->vendor != PCI_VENDOR_ID_ADLINK)
			continue;

		if (thisboard->dev_id == PCI_DEVICE_ID_INVALID) {
			/*
			 * The name was specified as the driver name which is
			 * used to match any supported device.  Replace the
			 * current dev->board_ptr with one that matches the
			 * PCI device ID.
			 */
			for (i = 0; i < ARRAY_SIZE(adl_pci7296_boards); i++) {
				if (pcidev->device ==
					adl_pci7296_boards[i].dev_id) {
					dev->board_ptr = &adl_pci7296_boards[i];
					break;
				}
			}
			if (i < ARRAY_SIZE(adl_pci7296_boards))
				break;
		} else if (thisboard->dev_id == pcidev->device) {
			/*
			 * The name was specified as a specific device name.
			 * The current dev->board_ptr is correct and it matches
			 * the PCI device ID.
			 */
			break;
		}
	}

	if (!pcidev) {
		printk("comedi%d: no %s board found! (req. bus/slot : %d/%d)\n",
			dev->minor, thisboard->name, bus, slot);
		return -EIO;
	}

	devpriv->pci_dev = pcidev;

	printk("comedi%d: found %s on bus %i, slot %i\n", dev->minor,
		thisboard->name, pcidev->bus->number, PCI_SLOT(pcidev->devfn));

	return adl_pci7296_attach_common(dev);
}

static int adl_pci7296_auto_attach(comedi_device *dev, unsigned long context)
{
	struct pci_dev *pcidev = comedi_to_pci_dev(dev);

	printk("comedi%d: %s attempting to attach pci %s\n", dev->minor,
		DRIVER_NAME, pci_name(pcidev));

	if (alloc_private(dev, sizeof(adl_pci7296_private)) < 0)
		return -ENOMEM;

	/* pci_dev_get() call matches pci_dev_put() in adl_pci7296_detach() */
	devpriv->pci_dev = pci_dev_get(pcidev);

	if (context >= ARRAY_SIZE(adl_pci7296_boards))
		return -EINVAL;

	dev->board_ptr = &adl_pci7296_boards[context];

	return adl_pci7296_attach_common(dev);
}

static int adl_pci7296_detach(comedi_device * dev)
{
	printk("comedi%d: %s: remove\n", dev->minor, DRIVER_NAME);

	if (devpriv && devpriv->pci_dev) {
		if (dev->iobase) {
			comedi_pci_disable(devpriv->pci_dev);
		}
		pci_dev_put(devpriv->pci_dev);
	}
	// detach 8255 digital io subdevices
	if (dev->subdevices) {
		int i;

		for (i = 0; i < thisboard->n_8255; i++)
			subdev_8255_cleanup(dev, dev->subdevices + i);
	}

	return 0;
}

COMEDI_PCI_INITCLEANUP(driver_adl_pci7296, adl_pci7296_pci_table);
