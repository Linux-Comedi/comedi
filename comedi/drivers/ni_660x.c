/*
    comedi/drivers/ni_660x.c
    Hardware driver for NI 660x devices

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
Driver: ni_660x.o
Description: National Instruments 660x
Author: J.P. Mellor <jpmellor@rose-hulman.edu>
Updated: 
Devices: [National Instruments] PCI-6601 (ni_660x), PCI-6602
Status: unknown

Commands are not supported.

*/

/* Things to do:

   Add General-Purpose Counter/Timer as a subdevice
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/errno.h> 
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>

#include <asm/io.h>

#include <linux/comedidev.h>
#include "mite.h"

#define PCI_VENDOR_ID_NATINST	0x1093

#define DATA_1B 0x1
#define DATA_2B 0x2
#define DATA_4B 0x4

/* Board description*/

typedef struct ni_660x_board_struct
{
	unsigned short dev_id;
	char *name;
}ni_660x_board;

static ni_660x_board ni_660x_boards[] = 
{
	{
	dev_id		: 0x2c60,
	name		: "PCI-6601",
	},
	{
	dev_id		: 0x0,			/* ????? */
	name		: "PCI-6602",
	},
};

static struct pci_device_id ni_660x_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_NATINST, 0x2c60, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	//{ PCI_VENDOR_ID_NATINST, 0x0000, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, ni_660x_pci_table);

#define thisboard ((ni_660x_board *)dev->board_ptr)

typedef struct
{
	struct mite_struct *mite;
	int boardtype;
}ni_660x_private;

#define devpriv ((ni_660x_private *)dev->private)
#define n_ni_660x_boards (sizeof(ni_660x_boards)/sizeof(ni_660x_boards[0]))

static int ni_660x_attach(comedi_device *dev,comedi_devconfig *it);
static int ni_660x_detach(comedi_device *dev);

static comedi_driver driver_ni_660x=
{
	driver_name:	"ni_660x",
	module:		THIS_MODULE,
	attach:		ni_660x_attach,
	detach:		ni_660x_detach,
};

COMEDI_INITCLEANUP(driver_ni_660x);

static int ni_660x_find_device(comedi_device *dev,int bus,int slot);

static int ni_660x_tio_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int ni_660x_tio_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int ni_660x_tio_insn_bits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int ni_660x_tio_insn_config(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);


static int ni_660x_attach(comedi_device *dev,comedi_devconfig *it)
{
  comedi_subdevice *s;
  int ret;

  printk("comedi%d: ni_660x: ",dev->minor);

  if ((ret=alloc_private(dev,sizeof(ni_660x_private))) < 0) return ret;

  ret = ni_660x_find_device(dev, it->options[0], it->options[1]);
  if (ret<0) return ret;

  ret = mite_setup(devpriv->mite);
  if (ret < 0) {
    printk("error setting up mite\n");
    return ret;
  }
  dev->iobase = mite_iobase(devpriv->mite);
  dev->board_name = thisboard->name;
  dev->irq = mite_irq(devpriv->mite);
  printk(" %s ", dev->board_name);

  if (alloc_subdevices(dev, 1)<0) return -ENOMEM;

  s=dev->subdevices+0;
  /* general-purpose counter/time ASIC */
  s->type			=	COMEDI_SUBD_COUNTER;
  s->subdev_flags		=	SDF_CMD|SDF_READABLE|SDF_WRITABLE;
  s->n_chan			= 	8;
  s->maxdata			=	1;
  s->range_table		=	&range_digital;
  s->insn_write 		= 	&ni_660x_tio_winsn;
  s->insn_read	 		= 	&ni_660x_tio_rinsn;
  s->insn_bits			= 	ni_660x_tio_insn_bits;
  s->insn_config 		= 	ni_660x_tio_insn_config;

  printk("attached\n");

  return 1;
}


static int
ni_660x_detach(comedi_device *dev)
{
  printk("comedi%d: ni_660x: remove\n",dev->minor);

  if (dev->private && devpriv->mite)
    mite_unsetup(devpriv->mite);

  if(dev->irq)
    comedi_free_irq(dev->irq,dev);

  return 0;
}

static int
ni_660x_tio_winsn(comedi_device *dev, comedi_subdevice *s,
		  comedi_insn *insn, lsampl_t *data)
{
  int i=0;

  switch (insn->chanspec) {

  case DATA_1B:
    for (i=0; i<insn->n; i+=2) {
      writeb(data[i+1], dev->iobase+data[i]);
    }
    break;

  case DATA_2B:
    for (i=0; i<insn->n; i+=2) {
      writew(data[i+1], dev->iobase+data[i]);
    }
    break;

  case DATA_4B:
    for (i=0; i<insn->n; i+=2) {
      writel(data[i+1], dev->iobase+data[i]);
    }
    break;

  default:
    return insn->n;

  }

  return i;
}

static int
ni_660x_tio_rinsn(comedi_device *dev, comedi_subdevice *s,
		  comedi_insn *insn, lsampl_t *data)
{
  int i=0;

  switch (insn->chanspec) {

  case DATA_1B:
    for (i=0; i<insn->n; i+=2) {
      data[i+1] = readb(dev->iobase+data[i]);
    }
    break;

  case DATA_2B:
    for (i=0; i<insn->n; i+=2) {
      data[i+1] = readw(dev->iobase+data[i]);
    }
    break;

  case DATA_4B:
    for (i=0; i<insn->n; i+=2) {
      data[i+1] = readl(dev->iobase+data[i]);
    }
    break;

  default:
    return insn->n;

  }

  return i;
}

// this is being called as part of opening the device
static int
ni_660x_tio_insn_bits(comedi_device *dev, comedi_subdevice *s,
		      comedi_insn *insn, lsampl_t *data)
{
  int i=0;

  switch (insn->chanspec) {

  case DATA_1B:
    for (i=0; i<insn->n; i+=2) {
      data[i+1] = readb(dev->iobase+data[i]);
    }
    break;

  case DATA_2B:
    for (i=0; i<insn->n; i+=2) {
      data[i+1] = readw(dev->iobase+data[i]);
    }
    break;

  case DATA_4B:
    for (i=0; i<insn->n; i+=2) {
      data[i+1] = readl(dev->iobase+data[i]);
    }
    break;

  default:
    return insn->n;

  }

  return i;
}

static int
ni_660x_tio_insn_config(comedi_device *dev, comedi_subdevice *s, 
			comedi_insn *insn, lsampl_t *data)
{
  int i=0;

  switch (insn->chanspec) {

  case DATA_1B:
    for (i=0; i<insn->n; i+=2) {
      writeb(data[i+1], dev->iobase+data[i]);
    }
    break;

  case DATA_2B:
    for (i=0; i<insn->n; i+=2) {
      writew(data[i+1], dev->iobase+data[i]);
    }
    break;

  case DATA_4B:
    for (i=0; i<insn->n; i+=2) {
      writel(data[i+1], dev->iobase+data[i]);
    }
    break;

  default:
    return insn->n;

  }

  return i;
}

static int
ni_660x_find_device(comedi_device *dev, int bus, int slot)
{
  struct mite_struct *mite;
  int i;

  for (mite=mite_devices; mite; mite=mite->next) {
    if (mite->used) continue;
    if (bus || slot) {
      if (bus!=mite->pcidev->bus->number ||
	  slot!=PCI_SLOT(mite->pcidev->devfn)) continue;
    }

    for (i=0; i<n_ni_660x_boards; i++) {
      if (mite_device_id(mite)==ni_660x_boards[i].dev_id) {
	dev->board_ptr=ni_660x_boards+i;
	devpriv->mite=mite;
	return 0;
      }
    }
  }
  printk("no device found\n");
  mite_list_devices();
  return -EIO;
}
