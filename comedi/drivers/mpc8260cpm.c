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
	name:		"mpc8260cpm",
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
static int skel_register_boards(void);
comedi_driver driver_skel={
	driver_name:	"dummy",
	module:		THIS_MODULE,
	attach:		skel_attach,
	detach:		skel_detach,
	register_boards:	skel_register_boards,	// replacement for recognize
};

static int skel_dio_config(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int skel_dio_bits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  _recognize() has already been called,
 * and dev->board contains whatever _recognize returned.
 */
static int skel_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int i;

	printk("comedi%d: mpc8260: ",dev->minor);
	
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
		s->insn_config = &skel_dio_config;
		s->insn_bits = &skel_dio_bits;
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
static int skel_dio_config(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n;
	unsigned int d;


	return 2;
}

static int skel_dio_bits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n;
	unsigned int d;


	return 2;
}



/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_skel);

