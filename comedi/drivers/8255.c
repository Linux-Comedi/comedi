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
#include <comedi_module.h>



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

#define CALLBACK_ARG	((void *)(s->cb_arg))
#define CALLBACK_FUNC	((int (*)(int,int,int,void *))(s->cb_func))

static int dev_8255_attach(comedi_device * dev, comedi_devconfig * it);
static int dev_8255_detach(comedi_device * dev);
comedi_driver driver_8255={
	driver_name:	"8255",
	module:		THIS_MODULE,
	attach:		dev_8255_attach,
	detach:		dev_8255_detach,
};

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

int subdev_8255_dio(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int mask,data_in;
	int i;

	if(it->flags & TRIG_CONFIG){
		int bits;

		for(i=0;i<it->n_chan;i++){
			mask=1<<CR_CHAN(it->chanlist[i]);
			if(mask&0x0000ff){
				bits=0x0000ff;
			}else if(mask&0x00ff00){
				bits=0x00ff00;
			}else if(mask&0x0f0000){
				bits=0x0f0000;
			}else{
				bits=0xf00000;
			}
			if(it->data[i]){
				s->io_bits|=bits;
			}else{
				s->io_bits&=~bits;
			}
		}
		do_config(dev,s);
	}else{
		if(it->flags&TRIG_WRITE){
			do_pack(&s->state,it);

			CALLBACK_FUNC(1,_8255_DATA,s->state&0xff,CALLBACK_ARG);
			CALLBACK_FUNC(1,_8255_DATA+1,(s->state>>8)&0xff,CALLBACK_ARG);
			CALLBACK_FUNC(1,_8255_DATA+2,(s->state>>16)&0xff,CALLBACK_ARG);
		}else{
			data_in=CALLBACK_FUNC(0,_8255_DATA,0,CALLBACK_ARG);
			data_in|=(CALLBACK_FUNC(0,_8255_DATA+1,0,CALLBACK_ARG)<<8);
			data_in|=(CALLBACK_FUNC(0,_8255_DATA+2,0,CALLBACK_ARG)<<16);

			di_unpack(data_in,it);
		}
	}

	return it->n_chan;
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

	/* commandeer range_list */
	CALLBACK_ARG=arg;

	/* same for flaglist */
	if(cb==NULL){
		CALLBACK_FUNC=subdev_8255_cb;
	}else{
		CALLBACK_FUNC=cb;
	}
	s->trig[0]=subdev_8255_dio;

	s->state=0;
	s->io_bits=0;
	do_config(dev,s);

	
	return 0;
}


/*

   Start of the 8255 standalone device

 */

static int dev_8255_attach(comedi_device *dev,comedi_devconfig *it)
{
	int ret;
	int iobase;
	int i;

	if(strcmp("8255",it->board_name))
		return 0;

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
	}

	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_8255);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_8255);
}
#endif
