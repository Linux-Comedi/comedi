/*
    module/parport.c
    hardware driver for standard parallel port

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1998 David A. Schleef <ds@stm.lbl.gov>

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

   TODO:

   - support bit mask ioctl
   - EPP/ECP support

   see http://www.beyondlogic.org/ for information.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/malloc.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <comedi_module.h>


#define PARPORT_SIZE 3

#define PARPORT_A 0
#define PARPORT_B 1
#define PARPORT_C 2

static int parport_attach(comedi_device *dev,comedi_devconfig *it);
static int parport_detach(comedi_device *dev);
comedi_driver driver_parport={
	driver_name:	"parport",
	module:		THIS_MODULE,
	attach:		parport_attach,
	detach:		parport_detach,
};

typedef struct parport_private_struct{
	unsigned int a_data;
	unsigned int c_data;
}parport_private;
#define devpriv ((parport_private *)(dev->private))

static int parport_insn_a(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(data[0]){
		devpriv->a_data &= ~data[0];
		devpriv->a_data |= (data[0]&data[1]);

		outb(devpriv->a_data,dev->iobase+PARPORT_A);
	}

	data[1] = devpriv->a_data;

	return 2;
}
	
static int parport_insn_b(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(data[0]){
		// should writes be ignored?
	}

	data[1] = (inb(dev->iobase+PARPORT_B)>>3);

	return 2;
}

static int parport_insn_c(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(data[0]){
		devpriv->c_data &= ~data[0];
		devpriv->c_data |= (data[0]&data[1]);

		outb(devpriv->c_data,dev->iobase+PARPORT_C);
	}

	data[1] = devpriv->c_data;

	return 2;
}
	

static int parport_attach(comedi_device *dev,comedi_devconfig *it)
{
	int ret;
	comedi_subdevice *s;

	dev->iobase=it->options[0];
	printk("comedi%d: parport: 0x%04x ",dev->minor,dev->iobase);
	if(check_region(dev->iobase,PARPORT_SIZE)<0){
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(dev->iobase,PARPORT_SIZE,"parport (comedi)");
	dev->iosize=PARPORT_SIZE;
	dev->irq=0;
	dev->board_name="parport";

	dev->n_subdevices=3;
	if((ret=alloc_subdevices(dev))<0)
		return ret;
	if((ret=alloc_private(dev,sizeof(parport_private)))<0)
		return ret;

	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=8;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->insn_bits = parport_insn_a;

	s=dev->subdevices+1;
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=4;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->insn_bits = parport_insn_b;

	s=dev->subdevices+2;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=4;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->insn_bits = parport_insn_c;

	printk("\n");
	return 1;
}


static int parport_detach(comedi_device *dev)
{
	printk("comedi%d: parport: remove\n",dev->minor);
	
	release_region(dev->iobase,dev->iosize);

	return 0;
}

COMEDI_INITCLEANUP(driver_parport);

