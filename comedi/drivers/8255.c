/*
    comedi/drivers/8255.c
    Driver for 8255

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1998 David A. Schleef <ds@schleef.org>

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
Driver: 8255
Description: generic 8255 support
Devices: [standard] 8255 (8255)
Author: ds
Status: works
Updated: Fri,  7 Jun 2002 12:56:45 -0700

The classic in digital I/O.  The 8255 appears in Comedi as a single
digital I/O subdevice with 24 channels.  The channel 0 corresponds
to the 8255's port A, bit 0; channel 23 corresponds to port C, bit
7.  Direction configuration is done in blocks, with channels 0-7,
8-15, 16-19, and 20-23 making up the 4 blocks.  The only 8255 mode
supported is mode 0.

You should enable compilation this driver if you plan to use a board
that has an 8255 chip.  For multifunction boards, the main driver will
configure the 8255 subdevice automatically.

This driver also works independently with ISA and PCI cards that
directly map the 8255 registers to I/O ports, including cards with
multiple 8255 chips.  To configure the driver for such a card, the
option list should be a list of the I/O port bases for each of the
8255 chips.  For example,

  comedi_config /dev/comedi0 8255 0x200,0x204,0x208,0x20c

Note that most PCI 8255 boards do NOT work with this driver, and
need a separate driver as a wrapper.  For those that do work, the
I/O port base address can be found in the output of 'lspci -v'.

*/

/*
   This file contains an exported subdevice for driving an 8255.

   To use this subdevice as part of another driver, you need to
   set up the subdevice in the attach function of the driver by
   calling:

     subdev_8255_init(device, subdevice, callback_function, arg)

   device and subdevice are pointers to the device and subdevice
   structures.  callback_function will be called to provide the
   low-level input/output to the device, i.e., actual register
   access.  callback_function will be called with the value of arg
   as the last parameter.  If the 8255 device is mapped as 4
   consecutive I/O ports, you can use NULL for callback_function
   and the I/O port base for arg, and an internal function will
   handle the register access.
 */

#include <linux/comedidev.h>
#include <linux/ioport.h>

#include "8255.h"

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

struct subdev_8255_struct {
	unsigned long cb_arg;
	int (*cb_func) (int, int, int, unsigned long);
};

#define CALLBACK_ARG	(((struct subdev_8255_struct *)s->private)->cb_arg)
#define CALLBACK_FUNC	(((struct subdev_8255_struct *)s->private)->cb_func)
#define subdevpriv	((struct subdev_8255_struct *)s->private)

static int dev_8255_attach(comedi_device * dev, comedi_devconfig * it);
static int dev_8255_detach(comedi_device * dev);
static comedi_driver driver_8255 = {
      driver_name:"8255",
      module:THIS_MODULE,
      attach:dev_8255_attach,
      detach:dev_8255_detach,
};

COMEDI_INITCLEANUP(driver_8255);

static void do_config(comedi_device * dev, comedi_subdevice * s);

static int subdev_8255_cb(int dir, int port, int data, unsigned long arg)
{
	unsigned long iobase = arg;

	if (dir) {
		outb(data, iobase + port);
		return 0;
	} else {
		return inb(iobase + port);
	}
}

static int subdev_8255_insn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	if (data[0]) {
		s->state &= ~data[0];
		s->state |= (data[0] & data[1]);

		if (data[0] & 0xff)
			CALLBACK_FUNC(1, _8255_DATA, s->state & 0xff,
				CALLBACK_ARG);
		if (data[0] & 0xff00)
			CALLBACK_FUNC(1, _8255_DATA + 1, (s->state >> 8) & 0xff,
				CALLBACK_ARG);
		if (data[0] & 0xff0000)
			CALLBACK_FUNC(1, _8255_DATA + 2,
				(s->state >> 16) & 0xff, CALLBACK_ARG);
	}

	data[1] = CALLBACK_FUNC(0, _8255_DATA, 0, CALLBACK_ARG);
	data[1] |= (CALLBACK_FUNC(0, _8255_DATA + 1, 0, CALLBACK_ARG) << 8);
	data[1] |= (CALLBACK_FUNC(0, _8255_DATA + 2, 0, CALLBACK_ARG) << 16);

	return 2;
}

static int subdev_8255_insn_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	unsigned int mask;
	unsigned int bits;

	mask = 1 << CR_CHAN(insn->chanspec);
	if (mask & 0x0000ff) {
		bits = 0x0000ff;
	} else if (mask & 0x00ff00) {
		bits = 0x00ff00;
	} else if (mask & 0x0f0000) {
		bits = 0x0f0000;
	} else {
		bits = 0xf00000;
	}

	switch (data[0]) {
	case INSN_CONFIG_DIO_INPUT:
		s->io_bits &= ~bits;
		break;
	case INSN_CONFIG_DIO_OUTPUT:
		s->io_bits |= bits;
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] = (s->io_bits & bits) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
	}

	do_config(dev, s);

	return 1;
}

static void do_config(comedi_device * dev, comedi_subdevice * s)
{
	int config;

	config = CR_CW;
	/* 1 in io_bits indicates output, 1 in config indicates input */
	if (!(s->io_bits & 0x0000ff))
		config |= CR_A_IO;
	if (!(s->io_bits & 0x00ff00))
		config |= CR_B_IO;
	if (!(s->io_bits & 0x0f0000))
		config |= CR_C_LO_IO;
	if (!(s->io_bits & 0xf00000))
		config |= CR_C_HI_IO;
	CALLBACK_FUNC(1, _8255_CR, config, CALLBACK_ARG);
}
int subdev_8255_init(comedi_device * dev, comedi_subdevice * s, int (*cb) (int,
		int, int, unsigned long), unsigned long arg)
{
	s->type = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
	s->n_chan = 24;
	s->range_table = &range_digital;
	s->maxdata = 1;

	s->private = kmalloc(sizeof(struct subdev_8255_struct), GFP_KERNEL);
	if (!s->private)
		return -ENOMEM;

	CALLBACK_ARG = arg;
	if (cb == NULL) {
		CALLBACK_FUNC = subdev_8255_cb;
	} else {
		CALLBACK_FUNC = cb;
	}
	s->insn_bits = subdev_8255_insn;
	s->insn_config = subdev_8255_insn_config;

	s->state = 0;
	s->io_bits = 0;
	do_config(dev, s);

	return 0;
}

void subdev_8255_cleanup(comedi_device * dev, comedi_subdevice * s)
{
	if (s->private) {
		kfree(s->private);
	}
}

/*

   Start of the 8255 standalone device

 */

static int dev_8255_attach(comedi_device * dev, comedi_devconfig * it)
{
	int ret;
	unsigned long iobase;
	int i;

	printk("comedi%d: 8255:", dev->minor);

	dev->board_name = "8255";

	for (i = 0; i < COMEDI_NDEVCONFOPTS; i++) {
		iobase = it->options[i];
		if (!iobase)
			break;
	}
	if (i == 0) {
		printk(KERN_CONT " no devices specified\n");
		return -EINVAL;
	}

	if ((ret = alloc_subdevices(dev, i)) < 0) {
		printk(KERN_CONT " (error allocating subdevices)\n");
		return ret;
	}

	for (i = 0; i < dev->n_subdevices; i++) {
		comedi_subdevice *s = &dev->subdevices[i];

		iobase = it->options[i];

		printk(KERN_CONT " 0x%04lx", iobase);
		if (!request_region(iobase, _8255_SIZE, "8255")) {
			printk(KERN_CONT " (I/O port conflict)");
			ret = -EIO;
			break;
		}
		ret = subdev_8255_init(dev, s, NULL, iobase);
		if (ret) {
			printk(KERN_CONT " (error %d)", ret);
			release_region(iobase, _8255_SIZE);
			s->type = COMEDI_SUBD_UNUSED;
			break;
		}
	}

	printk(KERN_CONT "\n");

	return ret;
}

static int dev_8255_detach(comedi_device * dev)
{
	int i;
	unsigned long iobase;
	comedi_subdevice *s;

	printk("comedi%d: 8255: remove\n", dev->minor);

	for (i = 0; i < dev->n_subdevices; i++) {
		s = dev->subdevices + i;
		if (s->type != COMEDI_SUBD_UNUSED) {
			iobase = CALLBACK_ARG;
			release_region(iobase, _8255_SIZE);
		}
		subdev_8255_cleanup(dev, s);
	}

	return 0;
}

EXPORT_SYMBOL(subdev_8255_init);
EXPORT_SYMBOL(subdev_8255_cleanup);
