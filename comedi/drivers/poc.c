/*
    comedi/drivers/poc.c
    Mini-drivers for POC (Piece of Crap) boards
    Copyright (C) 2000 Frank Mori Hess <fmhess@uiuc.edu>
    Copyright (C) 2001 David A. Schleef <ds@schleef.org>

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
 * This driver is a kind of miscellaneous driver for very
 * simple ISA boards.  
 */
/*
    dac02 - Keithley DAC-02 analog output board driver

The card's ranges are set by the wiring you use.  The driver only
uses the range specified to decide whether or not to take the
complement of the data before sending it to the card (since the
bipolar outputs of the card are inverted.)

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

/* DAC-02 registers */
#define DAC02_LSB(a)	(2 * a)
#define DAC02_MSB(a)	(2 * a + 1)

static int poc_attach(comedi_device *dev,comedi_devconfig *it);
static int poc_detach(comedi_device *dev);
static int dac02_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int readback_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);

struct boarddef_struct{
	char *name;
	int iosize;
	int (*setup)(comedi_device *);
	int type;
	int n_chan;
	int n_bits;
	int (*winsn)(comedi_device *,comedi_subdevice *,comedi_insn *,lsampl_t *);
	int (*rinsn)(comedi_device *,comedi_subdevice *,comedi_insn *,lsampl_t *);
};
static struct boarddef_struct boards[]={
	{
	name:		"dac02",
	iosize:		8,
	//setup:		dac02_setup,
	type:		COMEDI_SUBD_AO,
	n_chan:		2,
	n_bits:		12,
	winsn:		dac02_ao_winsn,
	rinsn:		readback_insn,
	}
};
#define n_boards (sizeof(boards)/sizeof(boards[0]))
#define this_board ((struct boarddef_struct *)dev->board_ptr)

comedi_driver driver_poc=
{
	driver_name:	"poc",
	module:		THIS_MODULE,
	attach:		poc_attach,
	detach:		poc_detach,
	board_name:	boards,
	num_names:	n_boards,
	offset:		sizeof(boards[0]),
};

// analog output ranges
static comedi_lrange range_dac02 = {
	4,
	{
		RANGE( -5, 5 ),
		RANGE( -10, 10 ),
		RANGE( 0, 5 ),
		RANGE( 0, 10 ),
	}
};

static int poc_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int iosize, iobase;

	iobase = it->options[0];
	printk("comedi%d: poc: using %s iobase 0x%x\n", dev->minor,
		this_board->name, iobase);

	dev->board_name = this_board->name;

	if(iobase == 0)
	{
		printk("io base address required\n");
		return -EINVAL;
	}

	iosize = this_board->iosize;
	/* check if io addresses are available */
	if(check_region(iobase, iosize) < 0)
	{
		printk("I/O port conflict: failed to allocate ports 0x%x to 0x%x\n",
			iobase, iobase + iosize - 1);
		return -EIO;
	}
	request_region(iobase, iosize, "dac02");
	dev->iobase = iobase;

	dev->n_subdevices = 1;
	if(alloc_subdevices(dev) < 0)
		return -ENOMEM;
	if(alloc_private(dev,sizeof(lsampl_t)*this_board->n_chan) < 0)
		return -ENOMEM;

	/* analog output subdevice */
	s=dev->subdevices + 0;
	s->type = this_board->type;
	s->n_chan = this_board->n_chan;
	s->maxdata = (1<<this_board->n_bits)-1;
	s->range_table = &range_dac02; // XXX
	s->insn_write = this_board->winsn;
	s->insn_write = this_board->rinsn;
	if(s->type==COMEDI_SUBD_AO || s->type==COMEDI_SUBD_DO){
		s->subdev_flags = SDF_WRITEABLE;
	}

	return 0;
}

static int poc_detach(comedi_device *dev)
{
	/* only free stuff if it has been allocated by _attach */
	if(dev->iobase)
		release_region(dev->iobase, this_board->iosize);

	printk("comedi%d: dac02: remove\n", dev->minor);

	return 0;
}

static int readback_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int chan;

	chan = CR_CHAN(insn->chanspec);
	data[0]=((lsampl_t *)dev->private)[chan];

	return 1;
}

static int dac02_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int temp;
	int chan;
	int output;

	chan = CR_CHAN(insn->chanspec);
	((lsampl_t *)dev->private)[chan] = data[0];
	output = data[0];
#if wrong
	// convert to complementary binary if range is bipolar
	if((CR_RANGE(insn->chanspec) & 0x2) == 0)
		output = ~output;
#endif
	temp = (output << 4) & 0xf0;
	outb(temp, dev->iobase + DAC02_LSB(chan));
	temp = (output >> 4) & 0xff;
	outb(temp, dev->iobase + DAC02_MSB(chan));

	return 1;
}

COMEDI_INITCLEANUP(driver_poc);

