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


static int parport_dio_a(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	outb(it->data[0],dev->iobase+PARPORT_A);

	return 1;
}

static int parport_dio_b(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	it->data[0]=(inb(dev->iobase+PARPORT_B)>>3);

	return 1;
}

static int parport_dio_c(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	outb(it->data[0],dev->iobase+PARPORT_C);

	return 1;
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

	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=8;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->trig[0]=parport_dio_a;

	s=dev->subdevices+1;
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=4;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->trig[0]=parport_dio_b;

	s=dev->subdevices+2;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=4;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->trig[0]=parport_dio_c;

	printk("\n");
	return 1;
}


static int parport_detach(comedi_device *dev)
{
	printk("comedi%d: parport: remove\n",dev->minor);
	
	release_region(dev->iobase,dev->iosize);

	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_parport);

	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_parport);
}
#endif

