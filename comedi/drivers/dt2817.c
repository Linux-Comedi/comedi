/*
    module/dt2817.c
    hardware driver for Data Translation DT2817

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



#define DT2817_SIZE 5

#define DT2817_CR 0
#define DT2817_DATA 1


static int dt2817_attach(comedi_device *dev,comedi_devconfig *it);
static int dt2817_detach(comedi_device *dev);
comedi_driver driver_dt2817={
	driver_name:	"dt2817",
	module:		THIS_MODULE,
	attach:		dt2817_attach,
	detach:		dt2817_detach,
};
COMEDI_INITCLEANUP(driver_dt2817);


static int dt2817_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int mask;
	int chan;
	int oe=0;

	if(insn->n!=1)return -EINVAL;

	chan=CR_CHAN(insn->chanspec);
	if(chan<8){
		mask=0xff;
	}else if(chan<16){
		mask=0xff00;
	}else if(chan<24){
		mask=0xff0000;
	}else mask=0xff000000;
	if(data[0])s->io_bits|=mask;
	else s->io_bits&=~mask;

	if(s->io_bits&0x000000ff)oe|=0x1;
	if(s->io_bits&0x0000ff00)oe|=0x2;
	if(s->io_bits&0x00ff0000)oe|=0x4;
	if(s->io_bits&0xff000000)oe|=0x8;

	outb(oe,dev->iobase + DT2817_CR);

	return 1;
}

static int dt2817_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	unsigned int changed;

	/* It's questionable whether it is more important in
	 * a driver like this to be deterministic or fast. 
	 * We choose fast. */

	if(data[0]){
		changed = s->state;
		s->state &= ~data[0];
		s->state |= (data[0]&data[1]);
		changed ^= s->state;
		changed &= s->io_bits;
		if(changed&0x000000ff)
			outb(s->state&0xff, dev->iobase + DT2817_DATA+0);
		if(changed&0x0000ff00)
			outb((s->state>>8)&0xff, dev->iobase + DT2817_DATA+1);
		if(changed&0x00ff0000)
			outb((s->state>>16)&0xff, dev->iobase + DT2817_DATA+2);
		if(changed&0xff000000)
			outb((s->state>>24)&0xff, dev->iobase + DT2817_DATA+3);
	}
	data[1] = inb(dev->iobase + DT2817_DATA + 0);
	data[1] |= (inb(dev->iobase + DT2817_DATA + 1)<<8);
	data[1] |= (inb(dev->iobase + DT2817_DATA + 2)<<16);
	data[1] |= (inb(dev->iobase + DT2817_DATA + 3)<<24);

	return 2;
}

static int dt2817_attach(comedi_device *dev,comedi_devconfig *it)
{
	int ret;
	comedi_subdevice *s;

	dev->iobase=it->options[0];
	printk("comedi%d: dt2817: 0x%04x ",dev->minor,dev->iobase);
	if(check_region(dev->iobase,DT2817_SIZE)<0){
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(dev->iobase,DT2817_SIZE,"dt2817");
	dev->board_name="dt2817";

	dev->n_subdevices=1;
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	s=dev->subdevices+0;

	s->n_chan=32;
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_READABLE|SDF_WRITEABLE;
	s->range_table=&range_digital;
	s->maxdata=1;
	s->insn_bits=dt2817_dio_insn_bits;
	s->insn_config=dt2817_dio_insn_config;

	s->state=0;
	outb(0,dev->iobase+DT2817_CR);

	printk("\n");

	return 0;
}


static int dt2817_detach(comedi_device *dev)
{
	printk("comedi%d: dt2817: remove\n",dev->minor);
	
	if(dev->iobase)
		release_region(dev->iobase,DT2817_SIZE);

	return 0;
}

