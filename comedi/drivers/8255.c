/*
    module/8255.c
    Driver for 8255

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
Driver: 8255.o
Description: generic 8255 support
Devices: [standard] 8255 (8255)
Author: ds

The classic in digital I/O.  Three channels of 8 bit digital I/O,
each channel is I/O configurable, channels 0 and 1 in 8 bit units,
channel 2 in 4 bit units.  The driver does not support modes 1 or 2
yet, since I don't really understand how they would potentially be used.
(Send me email if you want to use these modes.)  If and when
modes 1 and 2 are supported, there is a strong possibility that the
3rd channel will be split into two 4-bit channels.  (Refer to the
8255 spec for clues as to why.)

You should configure this driver if you plan to use a board that
has an 8255 chip.  For multifunction boards, the main driver will
configure the 8255 subdevice automatically.

This driver also works independently with ISA cards that directly
map the 8255 registers to I/O ports, including cards with multiple
8255 chips.  To configure the driver for such a card, the option
list should be a list of the I/O port bases for each of the 8255
chips.
*/

/*
   This file contains an exported subdevice for driving an 8255.

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



#define _8255_SIZE 4

#define _8255_DATA 0
#define _8255_CR 3

#define CR_C_LO_IO	0x01
#define CR_B_IO		0x02
#define CR_B_MODE	0x04
#define CR_C_HI_IO	0x08
#define CR_A_IO		0x10
#define CR_A_MODE(a)	((a)<<5)
#define CR_CW		0x80

struct subdev_8255_struct{
	void *cb_arg;
	int (*cb_func)(int,int,int,void *);
};

#define CALLBACK_ARG	(((struct subdev_8255_struct *)s->private)->cb_arg)
#define CALLBACK_FUNC	(((struct subdev_8255_struct *)s->private)->cb_func)

static int dev_8255_attach(comedi_device * dev, comedi_devconfig * it);
static int dev_8255_detach(comedi_device * dev);
static comedi_driver driver_8255={
	driver_name:	"8255",
	module:		THIS_MODULE,
	attach:		dev_8255_attach,
	detach:		dev_8255_detach,
};
COMEDI_INITCLEANUP(driver_8255);

static void do_config(comedi_device *dev,comedi_subdevice *s);

static int subdev_8255_cb(int dir,int port,int data,void *arg)
{
	int iobase=(int)arg;

	if(dir){
		outb(data,iobase+port);
		return 0;
	}else{
		return inb(iobase+port);
	}
}

static int subdev_8255_insn(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(data[0]){
		s->state &= ~data[0];
		s->state |= (data[0]&data[1]);
		
		if(data[0]&0xff)
			CALLBACK_FUNC(1,_8255_DATA,s->state&0xff,CALLBACK_ARG);
		if(data[0]&0xff00)
			CALLBACK_FUNC(1,_8255_DATA+1,(s->state>>8)&0xff,CALLBACK_ARG);
		if(data[0]&0xff0000)
			CALLBACK_FUNC(1,_8255_DATA+2,(s->state>>16)&0xff,CALLBACK_ARG);
	}

	data[1]=CALLBACK_FUNC(0,_8255_DATA,0,CALLBACK_ARG);
	data[1]|=(CALLBACK_FUNC(0,_8255_DATA+1,0,CALLBACK_ARG)<<8);
	data[1]|=(CALLBACK_FUNC(0,_8255_DATA+2,0,CALLBACK_ARG)<<16);

	return 2;
}

static int subdev_8255_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	unsigned int mask;
	unsigned int bits;

	mask=1<<CR_CHAN(insn->chanspec);
	if(mask&0x0000ff){
		bits=0x0000ff;
	}else if(mask&0x00ff00){
		bits=0x00ff00;
	}else if(mask&0x0f0000){
		bits=0x0f0000;
	}else{
		bits=0xf00000;
	}

	switch(insn->data[0]){
	case COMEDI_INPUT:
		s->io_bits&=~bits;
		break;
	case COMEDI_OUTPUT:
		s->io_bits|=bits;
		break;
	default:
		return -EINVAL;
	}

	do_config(dev,s);

	return 1;
}

static void do_config(comedi_device *dev,comedi_subdevice *s)
{
	int config;

	config=CR_CW;
	/* 1 in io_bits indicates output, 1 in config indicates input */
	if(!(s->io_bits&0x0000ff))
		config|=CR_A_IO;
	if(!(s->io_bits&0x00ff00))
		config|=CR_B_IO;
	if(!(s->io_bits&0x0f0000))
		config|=CR_C_LO_IO;
	if(!(s->io_bits&0xf00000))
		config|=CR_C_HI_IO;
	CALLBACK_FUNC(1,_8255_CR,config,CALLBACK_ARG);
}


int subdev_8255_init(comedi_device *dev,comedi_subdevice *s,int (*cb)(int,int,int,void *),void *arg)
{
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_READABLE|SDF_WRITEABLE|SDF_RT;
	s->n_chan=24;
	s->range_table=&range_digital;
	s->maxdata=1;

	s->private=kmalloc(sizeof(struct subdev_8255_struct),GFP_KERNEL);
	if(!s->private)return -ENOMEM;

	CALLBACK_ARG=arg;
	if(cb==NULL){
		CALLBACK_FUNC=subdev_8255_cb;
	}else{
		CALLBACK_FUNC=cb;
	}
	s->insn_bits = subdev_8255_insn;
	s->insn_config = subdev_8255_insn_config;

	s->state=0;
	s->io_bits=0;
	do_config(dev,s);
	
	return 0;
}

void subdev_8255_cleanup(comedi_device *dev,comedi_subdevice *s)
{
	if(s->private)
		kfree(s->private);
}

/*

   Start of the 8255 standalone device

 */

static int dev_8255_attach(comedi_device *dev,comedi_devconfig *it)
{
	int ret;
	int iobase;
	int i;

	printk("comedi%d: 8255:",dev->minor);

	dev->board_name="8255";

	for(i=0;i<COMEDI_NDEVCONFOPTS;i++){
		iobase=it->options[i];
		if(!iobase)break;
	}
	if(i==0){
		printk(" no devices specified\n");
		return -EINVAL;
	}
	dev->n_subdevices=i;

	if((ret=alloc_subdevices(dev))<0)
		return ret;

	for(i=0;i<dev->n_subdevices;i++){
		iobase=it->options[i];

		printk(" 0x%04x",iobase);
		if(check_region(iobase,_8255_SIZE)<0){
			printk(" (I/O port conflict)");

			dev->subdevices[i].type=COMEDI_SUBD_UNUSED;
		}else{
			request_region(iobase,_8255_SIZE,"8255");

			subdev_8255_init(dev,dev->subdevices+i,NULL,(void *)iobase);
		}
	}

	printk("\n");

	return 0;
}

static int dev_8255_detach(comedi_device *dev)
{
	int i,iobase;
	comedi_subdevice *s;

	printk("comedi%d: 8255: remove\n",dev->minor);
	
	for(i=0;i<dev->n_subdevices;i++){
		s=dev->subdevices+i;
		if(s->type!=COMEDI_SUBD_UNUSED){
			iobase=(int)CALLBACK_ARG;
			release_region(iobase,_8255_SIZE);
		}
		subdev_8255_cleanup(dev,s);
	}

	return 0;
}


EXPORT_SYMBOL(subdev_8255_init);
EXPORT_SYMBOL(subdev_8255_cleanup);

