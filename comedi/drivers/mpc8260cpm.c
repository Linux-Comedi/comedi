/*
    comedi/drivers/mpc8260.c
    driver for digital I/O pins on the MPC 8260 CPM module

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000,2001 David A. Schleef <ds@schleef.org>

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
#include <linux/comedidev.h>


typedef struct{
	int data;

}mpc8260cpm_private;
#define devpriv ((mpc8260cpm_private *)dev->private)

static int mpc8260cpm_attach(comedi_device *dev,comedi_devconfig *it);
static int mpc8260cpm_detach(comedi_device *dev);
comedi_driver driver_mpc8260cpm={
	driver_name:	"dummy",
	module:		THIS_MODULE,
	attach:		mpc8260cpm_attach,
	detach:		mpc8260cpm_detach,
};

static int mpc8260cpm_dio_config(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int mpc8260cpm_dio_bits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);

static int mpc8260cpm_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int i;

	printk("comedi%d: mpc8260cpm: ",dev->minor);
	
	dev->board_ptr = mpc8260cpm_boards + dev->board;

	dev->board_name = thisboard->name;

	if(alloc_private(dev,sizeof(mpc8260cpm_private))<0)
		return -ENOMEM;

	dev->n_subdevices=4;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	for(i=0;i<4;i++){
		s=dev->subdevices+i;
		s->type=COMEDI_SUBD_DIO;
		s->subdev_flags=SDF_READABLE|SDF_WRITEABLE;
		s->n_chan=32;
		s->maxdata=1;
		s->range_table=&range_digital;
		s->insn_config = &mpc8260cpm_dio_config;
		s->insn_bits = &mpc8260cpm_dio_bits;
	}
	
	return 1;
}

static int mpc8260cpm_detach(comedi_device *dev)
{
	printk("comedi%d: mpc8260cpm: remove\n",dev->minor);
	
	return 0;
}

static int mpc8260cpm_dio_config(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n;
	unsigned int d;


	return 2;
}

static int mpc8260cpm_dio_bits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n;
	unsigned int d;


	return 2;
}

COMEDI_INITCLEANUP(driver_mpc8260cpm);

