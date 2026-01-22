/*
    comedi/drivers/cb_pcidio.c
    A Comedi driver for PCI-DIO24H, PCI-DIO48H & PCI-DIO96H of ComputerBoards
    (currently MeasurementComputing)

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
Driver: cb_pcidio
Description: ComputerBoards' DIO boards with PCI interface
Devices: [Measurement Computing] PCI-DIO24 (cb_pcidio), PCI-DIO24H, PCI-DIO48H,
  PCI-DIO96H
Author: Yoshiya Matsuzaka
Updated: Tue, 26 Mar 2019 10:21:18 +0000
Status: experimental

This driver has been modified from skel.c of comedi-0.7.70.

Configuration Options:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
  If bus/slot is not specified, the first available PCI device will
  be used.

Passing a zero for an option is the same as leaving it unspecified.
*/

/*------------------------------ HEADER FILES ---------------------------------*/
#include <linux/comedidev.h>
#include "comedi_pci.h"
#include "8255.h"

/*-------------------------- MACROS and DATATYPES -----------------------------*/
#define PCI_VENDOR_ID_CB	0x1307

/*
 * Board descriptions for two imaginary boards.  Describing the
 * boards in this way is optional, and completely driver-dependent.
 * Some drivers use arrays such as this, other do not.
 */
typedef struct pcidio_board_struct {
	const char *name;	// name of the board
	int dev_id;
	int n_8255;		// number of 8255 chips on board
} pcidio_board;

enum pcidio_model {
	dio24_model,
	dio24h_model,
	dio48h_model,
	dio96h_model,
};

static const pcidio_board pcidio_boards[] = {
	[dio24_model] = {
		.name	= "pci-dio24",
		.dev_id	= 0x0028,
		.n_8255	= 1,
	},
	[dio24h_model] = {
		.name	= "pci-dio24h",
		.dev_id	= 0x0014,
		.n_8255	= 1,
	},
	[dio48h_model] = {
		.name	= "pci-dio48h",
		.dev_id	= 0x000b,
		.n_8255	= 2,
	},
	[dio96h_model] = {
		.name	= "pci-dio96h",
		.dev_id	= 0x0017,
		.n_8255	= 4,
	},
};

/* This is used by modprobe to translate PCI IDs to drivers.  Should
 * only be used for PCI and ISA-PnP devices */
/* Please add your PCI vendor ID to comedidev.h, and it will be forwarded
 * upstream. */
static DEFINE_PCI_DEVICE_TABLE(pcidio_pci_table) = {
	{ PCI_VDEVICE(CB, 0x0028), .driver_data = dio24_model,	},
	{ PCI_VDEVICE(CB, 0x0017), .driver_data = dio96h_model,	},
	{ PCI_VDEVICE(CB, 0x0014), .driver_data = dio24h_model,	},
	{ PCI_VDEVICE(CB, 0x000b), .driver_data = dio48h_model,	},
	{0}
};

MODULE_DEVICE_TABLE(pci, pcidio_pci_table);

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((const pcidio_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct {
	int data;		// curently unused

	/* would be useful for a PCI device */
	struct pci_dev *pci_dev;

	/* used for DO readback, curently unused */
	lsampl_t do_readback[4];	/* up to 4 lsampl_t suffice to hold 96 bits for PCI-DIO96 */

	unsigned long dio_reg_base;	// address of port A of the first 8255 chip on board
} pcidio_private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((pcidio_private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int pcidio_attach(comedi_device * dev, comedi_devconfig * it);
static int pcidio_auto_attach(comedi_device * dev, unsigned long context);
static int pcidio_detach(comedi_device * dev);
static comedi_driver driver_cb_pcidio = {
	.driver_name	= "cb_pcidio",
	.module		= THIS_MODULE,
	.attach		= pcidio_attach,
	.auto_attach	= pcidio_auto_attach,
	.detach		= pcidio_detach,
/* It is not necessary to implement the following members if you are
 * writing a driver for a ISA PnP or PCI card */
	/* Most drivers will support multiple types of boards by
	 * having an array of board structures.  These were defined
	 * in pcidio_boards[] above.  Note that the element 'name'
	 * was first in the structure -- Comedi uses this fact to
	 * extract the name of the board without knowing any details
	 * about the structure except for its length.
	 * When a device is attached (by comedi_config), the name
	 * of the device is given to Comedi, and Comedi tries to
	 * match it by going through the list of board names.  If
	 * there is a match, the address of the pointer is put
	 * into dev->board_ptr and driver->attach() is called.
	 *
	 * Note that these are not necessary if you can determine
	 * the type of board in software.  ISA PnP, PCI, and PCMCIA
	 * devices are such boards.
	 */
// The following fields should NOT be initialized if you are dealing with PCI devices
//	.board_name	= pcidio_boards,
//	.offset		= sizeof(pcidio_board),
//	.num_names	= sizeof(pcidio_boards) / sizeof(pcidio_board),
};

/*------------------------------- FUNCTIONS -----------------------------------*/

static int pcidio_attach_common(comedi_device *dev)
{
	struct pci_dev *pcidev = devpriv->pci_dev;
	int i;

	dev->board_name = thisboard->name;
	if (comedi_pci_enable(pcidev, thisboard->name)) {
		printk("cb_pcidio: failed to enable PCI device and request regions\n");
		return -EIO;
	}
	/*
	 * At some point, they switched from using AMCC S5933 (I think) PCI
	 * interface chip with user registers in BAR 1 to using PLX
	 * PCI9050/PCI9052 chip with user registers in BAR 2, but they kept the
	 * same PCI device ID (although they did set the PCI subvendor and
	 * subdevice ID on the newer cards as well).
	 *
	 * Use BAR 2 if it exists (for PLX), otherwise use BAR 1 (for AMCC).
	 */
	devpriv->dio_reg_base =
		pci_resource_start(pcidev,
			(pci_resource_len(pcidev, 2) ? 2 : 1));

	/*
	 * Allocate the subdevice structures.  alloc_subdevice() is a
	 * convenient macro defined in comedidev.h.
	 */
	if (alloc_subdevices(dev, thisboard->n_8255) < 0)
		return -ENOMEM;

	for (i = 0; i < thisboard->n_8255; i++) {
		subdev_8255_init(dev, dev->subdevices + i,
			NULL, devpriv->dio_reg_base + i * 4);
		printk(" subdev %d: base = 0x%lx\n", i,
			devpriv->dio_reg_base + i * 4);
	}

	printk("attached\n");
	return 1;
}

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int pcidio_attach(comedi_device * dev, comedi_devconfig * it)
{
	struct pci_dev *pcidev = NULL;
	int index;

	printk("comedi%d: cb_pcidio: \n", dev->minor);

	/*
	 * Allocate the private structure area.  alloc_private() is a
	 * convenient macro defined in comedidev.h.
	 */
	if (alloc_private(dev, sizeof(pcidio_private)) < 0)
		return -ENOMEM;
	/*
	 * If you can probe the device to determine what device in a series
	 * it is, this is the place to do it.  Otherwise, dev->board_ptr
	 * should already be initialized.
	 */
	/*
	 * Probe the device to determine what device in the series it is.
	 */

	for (pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
		pcidev != NULL;
		pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pcidev)) {
		// is it not a computer boards card?
		if (pcidev->vendor != PCI_VENDOR_ID_CB)
			continue;
		// loop through cards supported by this driver
		for (index = 0;
			index < ARRAY_SIZE(pcidio_boards);
			index++) {
			if (pcidio_boards[index].dev_id != pcidev->device)
				continue;

			// was a particular bus/slot requested?
			if (it->options[0] || it->options[1]) {
				// are we on the wrong bus/slot?
				if (pcidev->bus->number != it->options[0] ||
					PCI_SLOT(pcidev->devfn) !=
					it->options[1]) {
					continue;
				}
			}
			dev->board_ptr = pcidio_boards + index;
			goto found;
		}
	}

	printk("No supported ComputerBoards/MeasurementComputing card found on "
		"requested position\n");
	return -EIO;

found:

	/*
	 * Note that we can use the "thisboard" macro now,
	 * since we just set dev->board_ptr.
	 */
	devpriv->pci_dev = pcidev;
	printk("Found %s on bus %i, slot %i\n", thisboard->name,
		devpriv->pci_dev->bus->number,
		PCI_SLOT(devpriv->pci_dev->devfn));

	return pcidio_attach_common(dev);
}

static int pcidio_auto_attach(comedi_device *dev, unsigned long context_model)
{
	struct pci_dev *pcidev = comedi_to_pci_dev(dev);

	printk("comedi%d: cb_pcidio: auto-attach PCI %s\n",
	       dev->minor, pci_name(pcidev));

	/*
	 * Allocate the private structure area.  alloc_private() is a
	 * convenient macro defined in comedidev.h.
	 */
	if (alloc_private(dev, sizeof(pcidio_private)) < 0)
		return -ENOMEM;

	/* context_model is the index into pcidio_boards[] */
	if (context_model >= ARRAY_SIZE(pcidio_boards)) {
		printk("comedi%d: cb_pcidio: BUG: bad auto-attach context - %lu\n",
		       dev->minor, context_model);
		return -EINVAL;
	}
	dev->board_ptr = pcidio_boards + context_model;

	/* pci_dev_get() call matches pci_dev_put() in pcidio_detach() */
	devpriv->pci_dev = pci_dev_get(pcidev);

	return pcidio_attach_common(dev);
}

/*
 * _detach is called to deconfigure a device.  It should deallocate
 * resources.
 * This function is also called when _attach() fails, so it should be
 * careful not to release resources that were not necessarily
 * allocated by _attach().  dev->private and dev->subdevices are
 * deallocated automatically by the core.
 */
static int pcidio_detach(comedi_device * dev)
{
	printk("comedi%d: cb_pcidio: remove\n", dev->minor);
	if (devpriv) {
		if (devpriv->pci_dev) {
			if (devpriv->dio_reg_base) {
				comedi_pci_disable(devpriv->pci_dev);
			}
			pci_dev_put(devpriv->pci_dev);
		}
	}
	if (dev->subdevices) {
		int i;
		for (i = 0; i < thisboard->n_8255; i++) {
			subdev_8255_cleanup(dev, dev->subdevices + i);
		}
	}
	return 0;
}

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_PCI_INITCLEANUP(driver_cb_pcidio, pcidio_pci_table);
