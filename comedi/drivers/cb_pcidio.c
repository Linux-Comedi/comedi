/*
    comedi/drivers/cb_pcidio.c
    A Comedi driver for PCI-DIO24H & PCI-DIO48H of ComputerBoards (currently MeasurementComputing)

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
Driver: cb_pcidio.o
Description: ComputerBoards' DIO boards with PCI interface
Devices: [ComputerBoards (currently MeasurementComputing)]
	PCI-DIO24H, PCI-DIO48H
Author: Yoshiya Matsuzaka
Updated: Wed, 25 Jul 2007 15:15:41 +0900
Status: experimental

This driver has been modified from skel.c of comedi-0.7.70.

Configuration Options:
  none
*/

/*
 * The previous block comment is used to automatically generate
 * documentation in Comedi and Comedilib.  The fields:
 *
 * Driver: the name of the driver
 * Description: a short phrase describing the driver.  Don't list boards.
 * Devices: a full list of the boards that attempt to be supported by
 *   the driver.  Format is "(manufacturer) board name [comedi name]",
 *   where comedi_name is the name that is used to configure the board.
 *   See the comment near board_name: in the comedi_driver structure
 *   below.  If (manufacturer) or [comedi name] is missing, the previous
 *   value is used.
 * Author: you
 * Updated: date when the _documentation_ was last updated.  Use 'date -R'
 *   to get a value for this.
 * Status: a one-word description of the status.  Valid values are:
 *   works - driver works correctly on most boards supported, and
 *     passes comedi_test.
 *   unknown - unknown.  Usually put there by ds.
 *   experimental - may not work in any particular release.  Author
 *     probably wants assistance testing it.
 *   bitrotten - driver has not been update in a long time, probably
 *     doesn't work, and probably is missing support for significant
 *     Comedi interface features.
 *   untested - author probably wrote it "blind", and is believed to
 *     work, but no confirmation.
 *
 * These headers should be followed by a blank line, and any comments
 * you wish to say about the driver.  The comment area is the place
 * to put any known bugs, limitations, unsupported features, supported
 * command triggers, whether or not commands are supported on particular
 * subdevices, etc.
 *
 * Somewhere in the comment should be information about configuration
 * options that are used with comedi_config.
 */



/*------------------------------ HEADER FILES ---------------------------------*/
#include <linux/comedidev.h>
#include <linux/pci.h> /* for PCI devices */



/*-------------------------- MACROS and DATATYPES -----------------------------*/
#define PCI_VENDOR_ID_CB	0x1307

/*
 * Board descriptions for two imaginary boards.  Describing the
 * boards in this way is optional, and completely driver-dependent.
 * Some drivers use arrays such as this, other do not.
 */
typedef struct pcidio_board_struct{
	char *name;		// anme of the board
	int n_8255;		// number of 8255 chips on board

	// indices of base address regions	
	int pcicontroler_badrindex;
	int dioregs_badrindex;
} pcidio_board;



static pcidio_board pcidio_boards[] = {
	{
	name:		"pci-dio24h",
	n_8255:		1,
	pcicontroler_badrindex:	1,	
	dioregs_badrindex:		2,
	},
	{
	name:		"pci-dio48h",
	n_8255:		2,
	pcicontroler_badrindex:	0,
	dioregs_badrindex:		1,
	},
};



/* This is used by modprobe to translate PCI IDs to drivers.  Should
 * only be used for PCI and ISA-PnP devices */
/* Please add your PCI vendor ID to comedidev.h, and it will be forwarded
 * upstream. */
static struct pci_device_id pcidio_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_CB, 0x0014, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x000b, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, pcidio_pci_table);



/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((pcidio_board *)dev->board_ptr)



/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct{
	int data;						// curently unused

	/* would be useful for a PCI device */
	struct pci_dev *pci_dev;

	/* used for DO readback, curently unused */
	lsampl_t do_readback[4];	/* up to 4 lsampl_t suffice to hold 96 bits for PCI-DIO96 */
	  
	unsigned long dio_reg_base;		// address of port A of the first 8255 chip on board
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
static int pcidio_attach(comedi_device *dev,comedi_devconfig *it);
static int pcidio_detach(comedi_device *dev);
static comedi_driver driver_cb_pcidio={
	driver_name:	"cb_pcidio",
	module:		THIS_MODULE,
	attach:		pcidio_attach,
	detach:		pcidio_detach,
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
//	board_name:	pcidio_boards,
//	offset:		sizeof(pcidio_board),
//	num_names:	sizeof(pcidio_boards) / sizeof(pcidio_board),
};



/*------------------------------- FUNCTIONS -----------------------------------*/

// The 2 functions below are currently unsed
static int pcidio_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int pcidio_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);



/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int pcidio_attach(comedi_device *dev, comedi_devconfig *it)
{
struct pci_dev* pcidev = NULL;
int index;

	printk("comedi%d: pcidio: \n", dev->minor);
/*
 * If you can probe the device to determine what device in a series
 * it is, this is the place to do it.  Otherwise, dev->board_ptr
 * should already be initialized.
 */
/*
 * Probe the device to determine what device in the series it is.
 */

	for(pcidev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, NULL); pcidev != NULL ;
		pcidev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, pcidev))
	{
		// is it not a computer boards card?
		if(pcidev->vendor != PCI_VENDOR_ID_CB)
			continue;
		// loop through cards supported by this driver
		for(index = 0; index < sizeof pcidio_boards / sizeof(pcidio_board); index++)
		{
			if(pcidio_pci_table[index].device != pcidev->device)
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
			dev->board_ptr = pcidio_boards + index;
			goto found;
		}
	}

	printk("No supported ComputerBoards/MeasurementComputing card found on "
		"requested position\n");
	return -EIO;

found:

/*
 * Allocate the private structure area.  alloc_private() is a
 * convenient macro defined in comedidev.h.
 */
	if(alloc_private(dev, sizeof(pcidio_private)) < 0)
		return -ENOMEM;

	devpriv->pci_dev = pcidev;
	printk("Found %s on bus %i, slot %i\n", pcidio_boards[index].name,
		devpriv->pci_dev->bus->number, PCI_SLOT(devpriv->pci_dev->devfn));
	devpriv->dio_reg_base 
	  = pci_resource_start(devpriv->pci_dev, pcidio_boards[index].dioregs_badrindex);

/*
 * Initialize dev->board_name.  Note that we can use the "thisboard"
 * macro now, since we just initialized it in the last line.
 */
	dev->board_name = thisboard->name;

/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.
 */
	if(alloc_subdevices(dev, thisboard->n_8255) < 0)
		return -ENOMEM;
	
	int i;
	for(i = 0; i < thisboard->n_8255; i++)
	{
		subdev_8255_init(dev, dev->subdevices + i,
			NULL, devpriv->dio_reg_base + i * 4);
		printk(" subdev %d: base = 0x%lx\n", i, devpriv->dio_reg_base + i * 4);
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
static int pcidio_detach(comedi_device *dev)
{
	printk("comedi%d: skel: remove\n",dev->minor);
	return 0;
}



/* DIO devices are slightly special.  Although it is possible to
 * implement the insn_read/insn_write interface, it is much more
 * useful to applications if you implement the insn_bits interface.
 * This allows packed reading/writing of the DIO channels.  The
 * comedi core can convert between insn_bits and insn_read/write */
static int pcidio_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=2)return -EINVAL;

	/* The insn data is a mask in data[0] and the new data
	 * in data[1], each channel cooresponding to a bit. */
	if(data[0]){
		s->state &= ~data[0];
		s->state |= data[0]&data[1];
		/* Write out the new digital output lines */
		//outw(s->state,dev->iobase + SKEL_DIO);
	}

	/* on return, data[1] contains the value of the digital
	 * input and output lines. */
	//data[1]=inw(dev->iobase + SKEL_DIO);
	/* or we could just return the software copy of the output values if
	 * it was a purely digital output subdevice */
	//data[1]=s->state;

	return 2;
}

static int pcidio_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int chan=CR_CHAN(insn->chanspec);

	/* The input or output configuration of each digital line is
	 * configured by a special insn_config instruction.  chanspec
	 * contains the channel to be changed, and data[0] contains the
	 * value COMEDI_INPUT or COMEDI_OUTPUT. */
	switch(data[0])
	{
	case INSN_CONFIG_DIO_OUTPUT:
		s->io_bits |= 1<<chan;
		break;
	case INSN_CONFIG_DIO_INPUT:
		s->io_bits &= ~(1<<chan);
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] = (s->io_bits & (1 << chan)) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
		break;
	}
	//outw(s->io_bits,dev->iobase + SKEL_DIO_CONFIG);

	return insn->n;
}

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_cb_pcidio);

