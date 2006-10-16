/*
    comedi/drivers/adl_pci7432.c

    Hardware comedi driver fot PCI7432 Adlink card
    Copyright (C) 2004 Michel Lachine <mike@mikelachaine.ca>

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
Driver: adl_pci7432.o
Description: Driver for the Adlink PCI-7432 64 ch. isolated digital io board
Devices: [ADLink] PCI-7432 (pci7432)
Author: Michel Lachaine <mike@mikelachaine.ca>
Status: experimental

Configuration Options:
  none
*/

#include <linux/comedidev.h>
#include <linux/kernel.h>
#include <linux/pci.h>

#define PCI7432_DI      0x00
#define PCI7432_DO	    0x00

#define PCI_DEVICE_ID_PCI7432 0x7432

typedef struct {
	char *name;
	int  vendor_id;
	int  device_id;
} adl_pci7432_board;

static adl_pci7432_board adl_pci7432_boards[] = {
	{ "pci7432", PCI_VENDOR_ID_ADLINK, PCI_DEVICE_ID_PCI7432 },
};

static struct pci_device_id adl_pci7432_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_ADLINK, PCI_DEVICE_ID_PCI7432, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, adl_pci7432_pci_table);

#define thisboard ((adl_pci7432_board *)dev->board_ptr)

typedef struct{
	int data;
	struct pci_dev *pci_dev;
} adl_pci7432_private;

#define devpriv ((adl_pci7432_private *)dev->private)

static int adl_pci7432_attach(comedi_device *dev,comedi_devconfig *it);
static int adl_pci7432_detach(comedi_device *dev);
static comedi_driver driver_adl_pci7432={
	driver_name:	"adl_pci7432",
	module:		THIS_MODULE,
	attach:		adl_pci7432_attach,
	detach:		adl_pci7432_detach,
	num_names:  1,
	board_name: (const char**)adl_pci7432_boards,
	offset:     sizeof(adl_pci7432_board),
};

/* Digital IO */

static int adl_pci7432_di_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

static int adl_pci7432_do_insn_bits(comedi_device *dev,comedi_subdevice *s,
        comedi_insn *insn,lsampl_t *data);

/*            */

static int adl_pci7432_attach(comedi_device *dev,comedi_devconfig *it)
{
	struct pci_dev *pcidev;
	comedi_subdevice *s;

	printk("comedi: attempt to attach...\n");
	printk("comedi%d: adl_pci7432: board=%s\n",dev->minor, thisboard->name);

  	dev->board_name = thisboard->name;

	if(alloc_private(dev,sizeof(adl_pci7432_private))<0)
		return -ENOMEM;

	if(alloc_subdevices(dev, 2)<0)
		return -ENOMEM;

	for(pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL); pcidev != NULL ;
		pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pcidev))
	{

		if ( pcidev->vendor == PCI_VENDOR_ID_ADLINK &&
		     pcidev->device == PCI_DEVICE_ID_PCI7432 ) {
			devpriv->pci_dev = pcidev;
			if (pci_enable_device(pcidev) < 0) {
				printk("comedi%d: Failed to enable PCI device\n", dev->minor);
				return -EIO;
			}
			if (pci_request_regions(pcidev, "adl_pci7432") < 0) {
				printk("comedi%d: I/O port conflict\n", dev->minor);
				return -EIO;
			}
			dev->iobase = pci_resource_start ( pcidev, 2 );
			printk ( "comedi: base addr %4lx\n", dev->iobase );

			dev->board_ptr = adl_pci7432_boards + 0;

			s=dev->subdevices+0;
			s->type = COMEDI_SUBD_DI;
			s->subdev_flags = SDF_READABLE|SDF_GROUND|SDF_COMMON;
			s->n_chan = 32;
			s->maxdata = 1;
			s->len_chanlist = 32;
			s->io_bits = 0x00000000;
			s->range_table = &range_digital;
			s->insn_bits = adl_pci7432_di_insn_bits;

			s=dev->subdevices+1;
            s->type = COMEDI_SUBD_DO;
            s->subdev_flags = SDF_WRITABLE|SDF_GROUND|SDF_COMMON;
            s->n_chan = 32;
            s->maxdata = 1	;
		  	s->len_chanlist = 32;
			s->io_bits = 0xffffffff;
            s->range_table = &range_digital;
            s->insn_bits = adl_pci7432_do_insn_bits;

			break;
		}
	}

	printk("comedi: attached\n");

	return 1;
}


static int adl_pci7432_detach(comedi_device *dev)
{
	printk("comedi%d: pci7432: remove\n",dev->minor);

	if (devpriv && devpriv->pci_dev) {
		if (dev->iobase) {
			pci_release_regions(devpriv->pci_dev);
			pci_disable_device(devpriv->pci_dev);
		}
		pci_dev_put(devpriv->pci_dev);
	}

	return 0;
}


static int adl_pci7432_do_insn_bits(comedi_device *dev,comedi_subdevice *s, comedi_insn *insn,lsampl_t *data)
{
	printk ( "comedi: pci7432_do_insn_bits called\n" );
	printk ( "comedi: data0: %8x data1: %8x\n", data[0], data[1] );

	if(insn->n!=2)return -EINVAL;

	if(data[0]){
		s->state &= ~data[0];
  		s->state |= (data[0]&data[1]);

		printk ( "comedi: out: %8x on iobase %4lx\n", s->state, dev->iobase + PCI7432_DO);
		outl(s->state & 0xffffffff, dev->iobase + PCI7432_DO);
	}
	return 2;
}

static int adl_pci7432_di_insn_bits(comedi_device *dev,comedi_subdevice *s, comedi_insn *insn,lsampl_t *data)
{
	printk ( "comedi: pci7432_di_insn_bits called\n" );
	printk ( "comedi: data0: %8x data1: %8x\n", data[0], data[1] );

	if(insn->n!=2)return -EINVAL;

	data[1] = inl(dev->iobase + PCI7432_DI) & 0xffffffff;
	printk ( "comedi: data1 %8x\n", data[1] );

	return 2;
}

COMEDI_INITCLEANUP(driver_adl_pci7432);
