/*
    comedi/drivers/skel.c
    Skeleton code for a Comedi driver

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


/* Imaginary registers for the imaginary board */

#define SKEL_SIZE 0

#define SKEL_START_AI_CONV	0
#define SKEL_AI_READ		0

/*
 * Board descriptions for two imaginary boards.  Describing the
 * boards in this way is optional, and completely driver-dependent.
 * Some drivers use arrays such as this, other do not.
 */
typedef struct skel_board_struct{
	char *name;
	int ai_chans;
	int ai_bits;
	int have_dio;
}skel_board;
skel_board skel_boards[] = {
	{
	name:		"skel-100",
	ai_chans:	16,
	ai_bits:	12,
	have_dio:	1,
	},
	{
	name:		"skel-200",
	ai_chans:	8,
	ai_bits:	16,
	have_dio:	0,
	},
};
/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((skel_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct{
	int data;

	/* would be useful for a PCI device */
	struct pci_dev *pci_dev;

}skel_private;
/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((skel_private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int skel_attach(comedi_device *dev,comedi_devconfig *it);
static int skel_detach(comedi_device *dev);
static int skel_recognize(char *name);
comedi_driver driver_skel={
	driver_name:	"dummy",
	module:		THIS_MODULE,
	attach:		skel_attach,
	detach:		skel_detach,
	recognize:	skel_recognize,
};

static int skel_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int skel_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);

/*
 * The function skel_recognize() is called when the Comedi core
 * gets a request to configure a device.  If the name of the device
 * being configured matches with one of the devices that this
 * driver can service, then a non-negative index should be returned.
 * This index is put into dev->board, and then _attach() is called.
 */
static int skel_recognize(char *name)
{
	if(!strcmp("skel-100",name))return 0;
	if(!strcmp("skel-200",name))return 1;

	return -1;
}

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  _recognize() has already been called,
 * and dev->board contains whatever _recognize returned.
 */
static int skel_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;

	printk("comedi%d: skel: ",dev->minor);
	
/*
 * If you can probe the device to determine what device in a series
 * it is, this is the place to do it.  Otherwise, you should use the
 * _recognize method, and use the value in dev->board.
 */
	//dev->board = skel_recognize(dev);

/*
 * Initialize dev->board_ptr.  This can point to an element in the
 * skel_boards array, for quick access to board-specific information.
 */
	dev->board_ptr = skel_boards + dev->board;

/*
 * Initialize dev->board_name.  Note that we can use the "thisboard"
 * macro now, since we just initialized it in the last line.
 */
	dev->board_name = thisboard->name;

/*
 * Allocate the private structure area.
 */
	if(alloc_private(dev,sizeof(skel_private))<0)
		return -ENOMEM;

/*
 * Allocate the subdevice structures.
 */
	dev->n_subdevices=3;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s=dev->subdevices+0;
	/* analog input subdevice */
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=thisboard->ai_chans;
	s->maxdata=(1<<thisboard->ai_bits)-1;
	s->range_table=&range_bipolar10;
	s->insn_read = &skel_ai_rinsn;
	
	s=dev->subdevices+1;
	/* analog output subdevice */
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=1;
	s->maxdata=0xffff;
	s->range_table=&range_bipolar5;
	s->insn_write = &skel_ao_winsn;

	s=dev->subdevices+2;
	/* digital i/o subdevice */
	if(thisboard->have_dio){
		s->type=COMEDI_SUBD_DIO;
		s->subdev_flags=SDF_READABLE|SDF_WRITEABLE;
		s->n_chan=16;
		s->maxdata=1;
		s->range_table=&range_digital;
		//s->r_insn = skel_dio_rinsn;
		//s->w_insn = skel_dio_winsn;
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}
	
	return 1;
}


/*
 * _detach is called to deconfigure a device.  It should deallocate
 * resources.  
 * This function is also called when _attach() fails, so it should be
 * careful not to release resources that were not necessarily
 * allocated by _attach().  dev->private and dev->subdevices are
 * deallocated automatically by the core.
 */
static int skel_detach(comedi_device *dev)
{
	printk("comedi%d: skel: remove\n",dev->minor);
	
	return 0;
}

/*
 * "instructions" read/write data in "one-shot" or "software-triggered"
 * mode.
 */
static int skel_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n;
	unsigned int d;

	/* a typical programming sequence */

	/* write channel to multiplexer */
	//outw(chan,dev->iobase + SKEL_MUX);
	
	/* wait for mux to settle */

	/* convert n samples */
	for(n=0;n<insn->n;n++){
		/* trigger conversion */
		//outw(0,dev->iobase + SKEL_CONVERT);

		/* wait for conversion to end */

		/* read data */
		//d = inw(dev->iobase + SKEL_AI_DATA);
		d = 0;

		/* mangle the data as necessary */
		d ^= 1<<(thisboard->ai_bits-1);

		data[n] = d;
	}

	/* return the number of samples read/written */
	return n;
}

static int skel_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	/* a typical programming sequence */

	//outw(data[0],dev->iobase + SKEL_DA0);

	/* return the number of samples read/written */
	return 1;
}


/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_skel);

