/*
    comedi/drivers/pio1616l.c

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
Driver: pio1616l.o
Description: Driver for Contec PIO1616L digital io board
Devices: PIO1616L
Author: Stefano Rivoir <s.rivoir@gts.it>
Updated: Mon, 18 Mar 2002 15:34:01 -0800
Status: works

Configuration Options:
  none
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <linux/comedidev.h>

typedef enum contec_model {
	PIO1616L	=0,
} contec_model;

typedef struct contec_board {
	char *name;
	int  model;
	int  in_ports;
	int  out_ports;
	int  in_offs;
	int  out_offs;
	int  out_boffs;
} contec_board;
static contec_board contec_boards[] = {
	{ "PIO1616L", PIO1616L, 16, 16, 0, 2, 10 },
};

#define PCI_VENDOR_ID_CONTEC   0x1221
#define PCI_DEVICE_ID_PIO1616L 0x8172
static struct pci_device_id contec_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_CONTEC, PCI_DEVICE_ID_PIO1616L, PCI_ANY_ID, PCI_ANY_ID, 0, 0, PIO1616L },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, contec_pci_table);

#define thisboard ((contec_board *)dev->board_ptr)

typedef struct{
	int data;

	struct pci_dev *pci_dev;

} contec_private;

#define devpriv ((contec_private *)dev->private)

static int contec_attach(comedi_device *dev,comedi_devconfig *it);
static int contec_detach(comedi_device *dev);
static comedi_driver driver_contec={
	driver_name:	"dummy",
	module:		THIS_MODULE,
	attach:		contec_attach,
	detach:		contec_detach,
	board_name:	contec_boards,
	offset:		sizeof(contec_board),
	num_names:	sizeof(contec_boards) / sizeof(contec_board),
};

/* Classic digital IO */
static int contec_dio_insn_read  (comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int contec_dio_insn_write (comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);

static int contec_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int contec_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

static int contec_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);

static int contec_ns_to_timer(unsigned int *ns,int round);

static int contec_attach(comedi_device *dev,comedi_devconfig *it)
{
	struct pci_dev *pcidev;
	comedi_subdevice *s;

	printk("comedi%d: contec: ",dev->minor);
	
	dev->board_name = thisboard->name;

	if(alloc_private(dev,sizeof(contec_private))<0)
		return -ENOMEM;

	dev->n_subdevices=1;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	pci_for_each_dev ( pcidev ) {
		
		if ( pcidev->vendor == PCI_VENDOR_ID_CONTEC && 
		     pcidev->device == PCI_DEVICE_ID_PIO1616L ) {
			dev->iobase = pci_resource_start ( pcidev, 0 );
			printk ( " base addr %x ", dev->iobase );

			s=dev->subdevices+0;

			printk ( "model PIO1616L " );
			s->type = COMEDI_SUBD_DIO;
			s->subdev_flags = SDF_READABLE | SDF_WRITEABLE;
			s->n_chan = 16;
			s->maxdata = 1;
			s->range_table = &range_digital;
			s->insn_read = contec_dio_insn_read;
			s->insn_write= contec_dio_insn_write;
			s->insn_bits = contec_dio_insn_bits;
			s->insn_config = contec_dio_insn_config;
			s->do_cmdtest = contec_cmdtest; 
		}
	}

	printk("attached\n");

	return 1;
}


static int contec_detach(comedi_device *dev)
{
	printk("comedi%d: contec: remove\n",dev->minor);
	
	return 0;
}

static int contec_dio_insn_read(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	unsigned int d1, d2, d;

	d1 = inb ( dev->iobase + thisboard->in_offs + 1 ) & 0x0f;
	d2 = inb ( dev->iobase + thisboard->in_offs + 0 ) & 0x0f;

	d = d1 << 8;
	d += d2;

	data[0] = d;

	return 1;

}

static int contec_dio_insn_write(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	u8 d;

	printk ( "contec_dio_insn_write1 called\n" );
	
	d = data[0] & 0x0f;
	outb ( d, dev->iobase + thisboard->out_offs );
	
	d = (data[0] >> 8) & 0x0f;
	outb ( d, dev->iobase + thisboard->out_offs );

	return 1;

}

static int contec_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd)
{
	printk ( "contec_cmdtest called\n" );
	return 0;
}

static int contec_ns_to_timer(unsigned int *ns,int round)
{
	return *ns;
}

static int contec_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{

	printk ( "contec_dio_insn_bits called\n" );
	printk ( " data: %d %d\n", data[0], data[1] );
	
	if(insn->n!=2)return -EINVAL;

	if(data[0]){
		s->state &= ~data[0];
		s->state |= data[0]&data[1];
		printk ( "  out: %d on %x\n", s->state, dev->iobase+2 );
		outw(s->state, dev->iobase + thisboard->out_offs );
	}
	return 2;
}

static int contec_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int chan=CR_CHAN(insn->chanspec);
	printk ( "contec_dio_insn_config called\n" );

	if(insn->n!=1)return -EINVAL;

	if(data[0]==COMEDI_OUTPUT){
		s->io_bits |= 1<<chan;
	}else{
		s->io_bits &= ~(1<<chan);
	}

	return 1;
}

COMEDI_INITCLEANUP(driver_contec);

