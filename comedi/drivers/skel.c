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
#include <linux/comedidev.h>


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
comedi_driver driver_skel={
	driver_name:	"dummy",
	module:		THIS_MODULE,
	attach:		skel_attach,
	detach:		skel_detach,
/* It is not necessary to implement the following members if you are
 * writing a driver for a ISA PnP or PCI card */
	/* Most drivers will support multiple types of boards by
	 * having an array of board structures.  These were defined
	 * in skel_boards[] above.  Note that the element 'name'
	 * was first in the structure -- Comedi uses this fact to
	 * extract the name of the board without knowing any details
	 * about the structure except for its length.
	 * When a device is attached (by comedi_config), the name
	 * of the device is given to Comedi, and Comedi tries to
	 * match it by going through the list of board names.  If
	 * there is a match, the address of the pointer is put
	 * into dev->board_ptr and driver->attach() is called.
	 *
	 * Note that these are not necessary if you can determine
	 * the type of board in software.  ISA PnP, PCI, and PCMCIA
	 * devices are such boards.
	 */
	board_name:	skel_boards,
	offset:		sizeof(skel_board),
	num_names:	sizeof(skel_boards) / sizeof(skel_board),
};

static int skel_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int skel_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int skel_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);
static int skel_ns_to_timer(unsigned int *ns,int round);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int skel_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;

	printk("comedi%d: skel: ",dev->minor);
	
/*
 * If you can probe the device to determine what device in a series
 * it is, this is the place to do it.  Otherwise, dev->board_ptr
 * should already be initialized.
 */
	//dev->board_ptr = skel_probe(dev);

/*
 * Initialize dev->board_name.  Note that we can use the "thisboard"
 * macro now, since we just initialized it in the last line.
 */
	dev->board_name = thisboard->name;

/*
 * Allocate the private structure area.  alloc_private() is a
 * convenient macro defined in comedidev.h.
 */
	if(alloc_private(dev,sizeof(skel_private))<0)
		return -ENOMEM;

/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.  It relies on
 * n_subdevices being set correctly.
 */
	dev->n_subdevices=3;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s=dev->subdevices+0;
	//dev->read_subdev=s;
	/* analog input subdevice */
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=thisboard->ai_chans;
	s->maxdata=(1<<thisboard->ai_bits)-1;
	s->range_table=&range_bipolar10;
	s->insn_read = &skel_ai_rinsn;
	//s->do_cmd = &skel_ai_cmd;
	s->do_cmdtest = &skel_ai_cmdtest;

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
	
	printk("attached\n");

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
		/* return -ETIMEDOUT if there is a timeout */

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

static int skel_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd)
{
	int err=0;
	int tmp;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* step 1: make sure trigger sources are trivially valid */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src && tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER|TRIG_EXT;
	if(!cmd->scan_begin_src && tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER|TRIG_EXT;
	if(!cmd->convert_src && tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src && tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT|TRIG_NONE;
	if(!cmd->stop_src && tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	/* note that mutual compatiblity is not an issue here */
	if(cmd->scan_begin_src!=TRIG_TIMER &&
	   cmd->scan_begin_src!=TRIG_EXT)err++;
	if(cmd->convert_src!=TRIG_TIMER &&
	   cmd->convert_src!=TRIG_EXT)err++;
	if(cmd->stop_src!=TRIG_TIMER &&
	   cmd->stop_src!=TRIG_EXT)err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}

#define MAX_SPEED	10000		/* in nanoseconds */
#define MIN_SPEED	1000000000	/* in nanoseconds */

	if(cmd->scan_begin_src==TRIG_TIMER){
		if(cmd->scan_begin_arg<MAX_SPEED){
			cmd->scan_begin_arg=MAX_SPEED;
			err++;
		}
		if(cmd->scan_begin_arg>MIN_SPEED){
			cmd->scan_begin_arg=MIN_SPEED;
			err++;
		}
	}else{
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		/* should specify multiple external triggers */
		if(cmd->scan_begin_arg>9){
			cmd->scan_begin_arg=9;
			err++;
		}
	}
	if(cmd->convert_src==TRIG_TIMER){
		if(cmd->convert_arg<MAX_SPEED){
			cmd->convert_arg=MAX_SPEED;
			err++;
		}
		if(cmd->convert_arg>MIN_SPEED){
			cmd->convert_arg=MIN_SPEED;
			err++;
		}
	}else{
		/* external trigger */
		/* see above */
		if(cmd->convert_arg>9){
			cmd->convert_arg=9;
			err++;
		}
	}

	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src==TRIG_COUNT){
		if(cmd->stop_arg>0x00ffffff){
			cmd->stop_arg=0x00ffffff;
			err++;
		}
	}else{
		/* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	if(cmd->scan_begin_src==TRIG_TIMER){
		tmp=cmd->scan_begin_arg;
		skel_ns_to_timer(&cmd->scan_begin_arg,cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->scan_begin_arg)err++;
	}
	if(cmd->convert_src==TRIG_TIMER){
		tmp=cmd->convert_arg;
		skel_ns_to_timer(&cmd->convert_arg,cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->convert_arg)err++;
		if(cmd->scan_begin_src==TRIG_TIMER &&
		  cmd->scan_begin_arg<cmd->convert_arg*cmd->scan_end_arg){
			cmd->scan_begin_arg=cmd->convert_arg*cmd->scan_end_arg;
			err++;
		}
	}

	if(err)return 4;

	return 0;
}

/* This function doesn't require a particular form, this is just
 * what happens to be used in some of the drivers.  It should
 * convert ns nanoseconds to a counter value suitable for programming
 * the device.  Also, it should adjust ns so that it cooresponds to
 * the actual time that the device will use. */
static int skel_ns_to_timer(unsigned int *ns,int round)
{
	/* trivial timer */
	return *ns;
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

