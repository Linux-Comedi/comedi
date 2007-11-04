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
Description: Driver for the Adlink PCI-7296 96 ch. digital io board
Devices: [ADLink] PCI-7296 (pci7296)
Author: Jon Grierson <jd@renko.co.uk>
Updated: 2.8.2006
Status: testing

Configuration Options:
  none
*/

#include <linux/comedidev.h>
#include <linux/kernel.h>

#include "comedi_pci.h"
#include "8255.h"
// #include "8253.h"

#define PORT1A 0
#define PORT2A 4
#define PORT3A 8
#define PORT4A 12

#define PCI_DEVICE_ID_PCI7296 0x7296

typedef struct skel_board_struct {
	const char *name;
	int vendor_id;
	int device_id;
} adl_pci7296_board;

static const adl_pci7296_board adl_pci7296_boards[] = {
	{"pci7296", PCI_VENDOR_ID_ADLINK, PCI_DEVICE_ID_PCI7296},
};

static struct pci_device_id adl_pci7296_pci_table[] __devinitdata = {
	{PCI_VENDOR_ID_ADLINK, PCI_DEVICE_ID_PCI7296, PCI_ANY_ID, PCI_ANY_ID, 0,
		0, 0},
	{0}
};

MODULE_DEVICE_TABLE(pci, adl_pci7296_pci_table);

#define thisboard ((const adl_pci7296_board *)dev->board_ptr)

typedef struct {
	int data;
	struct pci_dev *pci_dev;
} adl_pci7296_private;

#define devpriv ((adl_pci7296_private *)dev->private)

static int adl_pci7296_attach(comedi_device * dev, comedi_devconfig * it);
static int adl_pci7296_detach(comedi_device * dev);
static comedi_driver driver_adl_pci7296 = {
      driver_name:"adl_pci7296",
      module:THIS_MODULE,
      attach:adl_pci7296_attach,
      detach:adl_pci7296_detach,

      board_name:&adl_pci7296_boards[0].name,
      offset:sizeof(adl_pci7296_board),
      num_names:sizeof(adl_pci7296_boards) / sizeof(adl_pci7296_board),
};

static int adl_pci7296_attach(comedi_device * dev, comedi_devconfig * it)
{
	struct pci_dev *pcidev;
	comedi_subdevice *s;

	printk("comedi: attempt to attach...\n");
	printk("comedi%d: adl_pci7432: board=%s\n", dev->minor,
		thisboard->name);

	dev->board_name = thisboard->name;

	if (alloc_private(dev, sizeof(adl_pci7296_private)) < 0)
		return -ENOMEM;

	if (alloc_subdevices(dev, 4) < 0)
		return -ENOMEM;

	for (pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
		pcidev != NULL;
		pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pcidev)) {

		if (pcidev->vendor == PCI_VENDOR_ID_ADLINK &&
			pcidev->device == PCI_DEVICE_ID_PCI7296) {
			devpriv->pci_dev = pcidev;
			if (comedi_pci_enable(pcidev, "adl_pci7296") < 0) {
				printk("comedi%d: Failed to enable PCI device and request regions\n", dev->minor);
				return -EIO;
			}

			dev->iobase = pci_resource_start(pcidev, 2);
			printk("comedi: base addr %4lx\n", dev->iobase);

			dev->board_ptr = adl_pci7296_boards + 0;

			// four 8255 digital io subdevices
			s = dev->subdevices + 0;
			subdev_8255_init(dev, s, NULL,
				(unsigned long)(dev->iobase));

			s = dev->subdevices + 1;
			subdev_8255_init(dev, s, NULL,
				(unsigned long)(dev->iobase + PORT2A));

			s = dev->subdevices + 2;
			subdev_8255_init(dev, s, NULL,
				(unsigned long)(dev->iobase + PORT3A));

			s = dev->subdevices + 3;
			subdev_8255_init(dev, s, NULL,
				(unsigned long)(dev->iobase + PORT4A));

			break;
		}
	}

	printk("attached\n");

	return 1;
}

static int adl_pci7296_detach(comedi_device * dev)
{
	printk("comedi%d: pci7432: remove\n", dev->minor);

	if (devpriv && devpriv->pci_dev) {
		if (dev->iobase) {
			comedi_pci_disable(devpriv->pci_dev);
		}
		pci_dev_put(devpriv->pci_dev);
	}
	// detach four 8255 digital io subdevices
	if (dev->subdevices) {
		subdev_8255_cleanup(dev, dev->subdevices + 0);
		subdev_8255_cleanup(dev, dev->subdevices + 1);
		subdev_8255_cleanup(dev, dev->subdevices + 2);
		subdev_8255_cleanup(dev, dev->subdevices + 3);

	}

	return 0;
}

COMEDI_INITCLEANUP(driver_adl_pci7296);
