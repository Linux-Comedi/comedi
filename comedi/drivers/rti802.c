/*
   module/rti802.c
   hardware driver for Analog Devices RTI-802 board

   COMEDI - Linux Control and Measurement Device Interface
   Copyright (C) 1999 Anders Blomdell <anders.blomdell@control.lth.se>

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


#define RTI802_SIZE 4

#define RTI802_SELECT 0
#define RTI802_DATALOW 1
#define RTI802_DATAHIGH 2

static int rti802_attach(comedi_device *dev,comedi_devconfig *it);
static int rti802_detach(comedi_device *dev);
comedi_driver driver_rti802={
	driver_name:	"rti802",
	module:		THIS_MODULE,
	attach:		rti802_attach,
	detach:		rti802_detach,
};

static void rti802_free_resources(comedi_device * dev);

typedef struct {
	enum {
		dac_2comp, dac_straight
	} dac_coding[8];
	comedi_lrange * range_type_list[8];
} rti802_private;

#define devpriv ((rti802_private *)dev->private)

static int rti802_ao(comedi_device * dev, comedi_subdevice *s, comedi_trig * it)
{
  int i;
  for(i=0 ; i < it->n_chan ; i++) {
    int chan = CR_CHAN(it->chanlist[i]);
    int data = it->data[i];

	if (devpriv->dac_coding[chan] == dac_2comp) {
		data ^= 0x800;
	}
	outb(chan, dev->iobase + RTI802_SELECT);
	outb(data & 0xff, dev->iobase + RTI802_DATALOW);
	outb(data >> 8, dev->iobase + RTI802_DATAHIGH);
  }
  return i;
}

/*
   options:
    [0] - i/o base
    [1] - unused
    [2] - dac#0  0=two's comp, 1=straight
    [3] - dac#0  0=bipolar, 1=unipolar
    [4] - dac#1 ...
    ...
    [17] - dac#7 ...
 */

static int rti802_attach(comedi_device * dev, comedi_devconfig * it)
{
	comedi_subdevice *s;
	int i;

	dev->iobase = it->options[0];
	printk("comedi%d: rti802: 0x%04x ", dev->minor, dev->iobase);
	if (check_region(dev->iobase, RTI802_SIZE) < 0) {
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(dev->iobase, RTI802_SIZE, "rti802");

	dev->board_name = "rti802";

	dev->n_subdevices = 1;
	if(alloc_subdevices(dev)<0 || alloc_private(dev,sizeof(rti802_private))){
		return -ENOMEM;
	}

	s=dev->subdevices;
	/* ao subdevice */
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_WRITEABLE;
	s->maxdata=0xfff;
	s->n_chan=8;
	s->trig[0] = rti802_ao;
	s->range_table_list=devpriv->range_type_list;

	for (i = 0; i < 8; i++) {
		devpriv->dac_coding[i] = (it->options[3 + 2 * i])
		    ? (dac_straight)
		    : (dac_2comp);
		devpriv->range_type_list[i] = (it->options[2 + 2 * i])
			? &range_unipolar10
			: &range_bipolar10;
	}

	printk("\n");

	return 0;
}

static void rti802_free_resources(comedi_device * dev)
{
	if(dev->iobase)
		release_region(dev->iobase, RTI802_SIZE);
}

static int rti802_detach(comedi_device * dev)
{
	printk("comedi%d: rti802: remove\n", dev->minor);

	rti802_free_resources(dev);

	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_rti802);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_rti802);
}
#endif
