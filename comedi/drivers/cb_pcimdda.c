/*
    comedi/drivers/cb_pcimdda.c

    Computer Boards PCIM-DDA06-16 Comedi driver

    Author: Calin Culianu <calin@ajvar.org>

    This is a driver for the Computer Boards PCIM-DDA06-16 Analog Output
    card.  This board has a unique register layout and as such probably 
    deserves its own driver file.  

    It is theoretically possible to integrate this board into the cb_pcidda
    file, but since that isn't my code, I didn't want to significantly
    modify that file to support this board (I thought it impolite to do so).

    At any rate, if you feel ambitious, please feel free to take
    the code out of this file and combine it with a more unified driver
    file.

    I would like to thank Timothy Curry <Timothy.Curry@rdec.redstone.army.mil>
    for lending me a board so that I could write this driver.

    -Calin Culianu <calin@ajvar.org>
    
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
/*
Driver: cb_pcimdda.o
Description: A driver for this relatively new and uniquely designed board
Devices: [Computer Boards] PCIM-DDA06-16 (pcimdda06-16)
Author: Calin Culianu <calin@ajvar.org>
Updated: Thu, 20 Jun 2002 16:19:41 -0500
Status: works

All features of the PCIM-DDA06-16 board are supported.  This board
has 6 16-bit AO channels, and the usual 8255 DIO setup.  (24 channels, 
configurable in banks of 8 and 4, etc.).  This board does not support commands.

The board has a peculiar way of specifying AO gain/range settings -- You have
1 jumper bank on the card, which either makes all 6 AO channels either
5 Volt unipolar, 5V bipolar, 10 Volt unipolar or 10V bipolar. 
  
Since there is absolutely _no_ way to tell in software how this jumper is set
(well, at least according  to the rather thin spec. from Measurement Computing
 that comes with the board), the driver assumes the jumper is at its factory 
default setting of +/-5V.

Also of note is the fact that this board features another jumper, whose
state is also completely invisible to software.  It toggles two possible AO
output modes on the board:

  - Update Mode: Writing to an AO channel instantaneously updates the actual
    signal output by the DAC on the board (this is the factory default).
  - Simultaneous XFER Mode: Writing to an AO channel has no effect until
    you read from any one of the AO channels.  This is useful for loading
    all 6 AO values, and then reading from any one of the AO channels on the 
    device to instantly update all 6 AO values in unison.  Useful for some
    control apps, I would assume?  If your jumper is in this setting, then you 
    need to issue your comedi_data_write()s to load all the values you want,
    then issue one comedi_data_read() on any channel on the AO subdevice
    to initiate the simultaneous XFER.
 

Configuration Options:
  Just tell comedi_config that you want to use the cb_pcimdda driver as so:

  comedi_config /dev/comedi0 cb_pcimdda
*/

/*
 * The previous block comment is used to automatically generate
 * documentation in Comedi and Comedilib.  The fields:
 *
 * Driver: the name of the driver
 * Description: a short phrase describing the driver.  Don't list boards.
 * Devices: a full list of the boards that attempt to be supported by
 *   the driver.  Format is "(manufacturer) board name [comedi name]",
 *   where comedi_name is the name that is used to configure the board.
 *   See the comment near board_name: in the comedi_driver structure
 *   below.  If (manufacturer) or [comedi name] is missing, the previous
 *   value is used.
 * Author: you
 * Updated: date when the _documentation_ was last updated.  Use 'date -R'
 *   to get a value for this.
 * Status: a one-word description of the status.  Valid values are:
 *   works - driver works correctly on most boards supported, and
 *     passes comedi_test.
 *   unknown - unknown.  Usually put there by ds.
 *   experimental - may not work in any particular release.  Author
 *     probably wants assistance testing it.
 *   bitrotten - driver has not been update in a long time, probably
 *     doesn't work, and probably is missing support for significant
 *     Comedi interface features.
 *   untested - author probably wrote it "blind", and is believed to
 *     work, but no confirmation.
 *
 * These headers should be followed by a blank line, and any comments
 * you wish to say about the driver.  The comment area is the place
 * to put any known bugs, limitations, unsupported features, supported
 * command triggers, whether or not commands are supported on particular
 * subdevices, etc.
 *
 * Somewhere in the comment should be information about configuration
 * options that are used with comedi_config.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <linux/comedidev.h>
#include "8255.h"


/* device ids of the cards we support -- currently only 1 card supported */
#define PCI_ID_PCIM_DDA06_16 0x0053

/*
 * This is straight from skel.c -- I did this in case this source file
 * will someday support more than 1 board...
 */
typedef struct board_struct {
    char *name;
    unsigned short device_id;
    int ao_chans;
    int ao_bits;
	int ai_chans;
	int ai_bits;
	int dio_chans;
    int dio_method;
    int dio_offset; /* how many bytes into the BADR are the DIO ports */
    int regs_badrindex; /* IO Region for the control, analog output, 
                           and DIO registers */
    int reg_sz;     /* number of bytes of registers in io region */
    comedi_lrange *ai_range_table;
    comedi_lrange *ao_range_table;
} board;

enum DIO_METHODS {
  DIO_NONE = 0,
  DIO_8255,
  DIO_INTERNAL /* unimplemented */
};

static board boards[] = {
    {
        name:		"cb_pcimdda06-16",
        device_id:       PCI_ID_PCIM_DDA06_16,
        ao_chans:	 6,
        ao_bits:         16,
        ai_chans:	 0,   /* No AI on this board */
        ai_bits:	 0,
        dio_chans:	 24,
        dio_method:      DIO_8255,
        dio_offset:      12, 
        regs_badrindex:  3,
        reg_sz:          16,
        ai_range_table:  &range_bipolar5, // dummy, since we have no AI
        /* this board only has one, jumper setable range, +/-5V 
           you can also set it to +/-10V via jumper, but we will 
           assume it is at the factory default, since it is beyond
           documented possibility to detect the state of this jumper through 
           software or to override it through software. (However, it's 
           possible that the Measurement Computing documentation
           is flawed or incomplete.. if you want to experiment by writing
           to random places in this device's IO regions and seeing if it
           changes the ranges settings on the board, please go ahead
           and do so... ). */
        ao_range_table:  &range_bipolar5
    }
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard    ((board *)dev->board_ptr)

/* Number of boards in boards[] */
#define N_BOARDS	(sizeof(boards) / sizeof(board))
#define REG_SZ (thisboard->reg_sz)
#define REGS_BADRINDEX (thisboard->regs_badrindex)

/* This is used by modprobe to translate PCI IDs to drivers.  Should
 * only be used for PCI and ISA-PnP devices */
/* Please add your PCI vendor ID to comedidev.h, and it will be forwarded
 * upstream. */
static struct pci_device_id pci_table[] __devinitdata = {
  { PCI_VENDOR_ID_COMPUTERBOARDS, PCI_ID_PCIM_DDA06_16, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
  { 0 }
};
MODULE_DEVICE_TABLE(pci, pci_table);


/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct {
        int registers; /* set by probe */
        int dio_registers;
        char  attached_to_8255; /* boolean */
        char  attached_successfully; /* boolean */
  /* would be useful for a PCI device */
        struct pci_dev *pci_dev;

#define MAX_AO_READBACK_CHANNELS 6
      /* Used for AO readback */
       lsampl_t ao_readback[MAX_AO_READBACK_CHANNELS];
    
} private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int attach(comedi_device *dev,comedi_devconfig *it);
static int detach(comedi_device *dev);
static comedi_driver cb_pcimdda_driver = {
	driver_name:	"cb_pcimdda",
	module:		THIS_MODULE,
	attach:		attach,
	detach:		detach,
    /* It is not necessary to implement the following members if you are
     * writing a driver for a ISA PnP or PCI card */
	/* Most drivers will support multiple types of boards by
	 * having an array of board structures.  These were defined
	 * in boards[] above.  Note that the element 'name'
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
    //	board_name:	boards,
	//offset:		sizeof(board),
	//num_names:	N_BOARDS
};


static int ao_winsn(comedi_device *dev, comedi_subdevice *s, 
                    comedi_insn *insn,lsampl_t *data);
static int ao_rinsn(comedi_device *dev, comedi_subdevice *s, 
                    comedi_insn *insn,lsampl_t *data);
/* stub... */
static int ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn,
                    lsampl_t *data);


/*---------------------------------------------------------------------------
  HELPER FUNCTION DECLARATIONS 
-----------------------------------------------------------------------------*/

/* returns a maxdata value for a given n_bits */
static inline lsampl_t figure_out_maxdata(int bits);

/* 
 *  Probes for a supported device.
 *
 *  Prerequisite: private be allocated already inside dev
 *   
 *  If the device is found, it returns 0 and has the following side effects:
 *
 *  o  assigns a struct pci_dev * to dev->private->pci_dev 
 *  o  assigns a struct board * to dev->board_ptr
 *  o  sets dev->private->registers
 *  o  sets dev->private->dio_registers
 *
 *  Otherwise, returns a -errno on error
 */
static int probe(comedi_device *dev, const comedi_devconfig *it);


/*---------------------------------------------------------------------------
  FUNCTION DEFINITIONS
-----------------------------------------------------------------------------*/

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int err;
    
/*
 * Allocate the private structure area.  alloc_private() is a
 * convenient macro defined in comedidev.h.
 * if this function fails (returns negative) then the private area is
 * kfree'd by comedi
 */
	if (alloc_private(dev,sizeof(private))<0)
		return -ENOMEM;
	
/*
 * If you can probe the device to determine what device in a series
 * it is, this is the place to do it.  Otherwise, dev->board_ptr
 * should already be initialized.
 */	 
	if ( (err = probe(dev, it)) ) return err;
	

/* Output some info */
	printk("comedi%d: %s: ",dev->minor, thisboard->name);


/*
 * Initialize dev->board_name.  Note that we can use the "thisboard"
 * macro now, since we just initialized it in the last line.
 */	
	dev->board_name = thisboard->name;

/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.  It relies on
 * n_subdevices being set correctly.
 */
	dev->n_subdevices=3;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s = dev->subdevices+0;

	if (thisboard->ai_chans > 0) {

  	    /* analog input subdevice */
	    s->type=COMEDI_SUBD_AI;
	    s->subdev_flags = SDF_READABLE;
	    s->n_chan = thisboard->ai_chans;
	    s->maxdata = figure_out_maxdata(thisboard->ai_bits);
	    s->range_table = &range_bipolar5;
	    s->insn_read = &ai_rinsn;

	} else {
	    /* no AI on this board! */
	    s->type = COMEDI_SUBD_UNUSED;
	}

	s = dev->subdevices+1;

	if (thisboard->ao_chans > 0) {

	    /* analog output subdevice */
	    s->type = COMEDI_SUBD_AO;
	    s->subdev_flags = SDF_WRITABLE | SDF_READABLE;
	    s->n_chan = thisboard->ao_chans;
	    s->maxdata = figure_out_maxdata(thisboard->ao_bits);
	    /* this is hard-coded here */
	    s->range_table = thisboard->ao_range_table;
	    s->insn_write = &ao_winsn;
	    s->insn_read = &ao_rinsn;

	} else {
  	    /* no AO on this board! */
  	    s->type = COMEDI_SUBD_UNUSED;
	}

	s = dev->subdevices+2;
	/* digital i/o subdevice */
    if(thisboard->dio_chans) {
        switch(thisboard->dio_method) {
        case DIO_8255:
           /* this is a straight 8255, so register us with the 8255 driver */
            subdev_8255_init(dev, s, NULL,  
                             (unsigned long)(devpriv->dio_registers));
            devpriv->attached_to_8255 = 1;
            break;
        case DIO_INTERNAL:
        default:
            printk("DIO_INTERNAL not implemented yet!\n");
            return -ENXIO;
            break;
        }
    } else {
      s->type = COMEDI_SUBD_UNUSED;
    }
	
    devpriv->attached_successfully = 1;

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
static int detach(comedi_device *dev)
{
    if (devpriv) {

        if (devpriv->registers && thisboard) {
            release_region(devpriv->registers, REG_SZ);
            devpriv->registers = 0;
        }

        if (dev->subdevices && devpriv->attached_to_8255) {
            /* de-register us from the 8255 driver */
            subdev_8255_cleanup(dev,dev->subdevices + 2);
            devpriv->attached_to_8255 = 0;
        }

        if (devpriv->attached_successfully && thisboard)
            printk("comedi%d: %s: detached\n", dev->minor, thisboard->name);

    }
	
	return 0;
}



static int ao_winsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn,
		    lsampl_t *data)
{
   int i;
   int chan = CR_CHAN(insn->chanspec);
   int offset = devpriv->registers + chan*2;

	/* Writing a list of values to an AO channel is probably not
	 * very useful, but that's how the interface is defined. */
   for(i=0;i<insn->n;i++) {
       /*  first, load the low byte */ 
       outb((char)(data[i] & 0x00ff), offset);
       /*  next, write the high byte -- only after this is written is
           the channel voltage updated in the DAC, unless
           we're in simultaneous xfer mode (jumper on card)
           then a rinsn is necessary to actually update the DAC --
           see ao_rinsn() below... */
       outb((char)(data[i]>>8 & 0x00ff), offset + 1);

       /* for testing only.. the actual rinsn SHOULD do an inw!
          (see the stuff about simultaneous XFER mode on this board) */
       devpriv->ao_readback[chan] = data[i];
   }

   /* return the number of samples read/written */
   return i;
}

/* AO subdevices should have a read insn as well as a write insn.

   Usually this means copying a value stored in devpriv->ao_readback. 
   However, since this board has this jumper setting called "Simultaneous
   Xfer mode" (off by default), we will support it.  Simultaneaous xfer
   mode is accomplished by loading ALL the values you want for AO in all the 
   channels, then READing off one of the AO registers to initiate the 
   instantaneous simultaneous update of all DAC outputs, which makes
   all AO channels update simultaneously.  This is useful for some control
   applications, I would imagine.
*/
static int ao_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn,
		    lsampl_t *data)
{
    int i;
    int chan = CR_CHAN(insn->chanspec);


    for(i=0;i<insn->n;i++) {
      inw(devpriv->registers + chan*2);
      /* should I set data[i] to the result of the actual read on the register
	 or the cached lsampl_t in devpriv->ao_readback[]? */
      data[i] = devpriv->ao_readback[chan];
    }

    return i;
}

/* stub... */
static int ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn,
                    lsampl_t *data)
{
    /* do nothing.. */
    (void) dev; (void) s; (void) insn; (void) data;
    return 0;
}

/*---------------------------------------------------------------------------
  HELPER FUNCTION DEFINITIONS
-----------------------------------------------------------------------------*/


/* 
 *  Probes for a supported device.
 *
 *  Prerequisite: private be allocated already inside dev
 *   
 *  If the device is found, it returns 0 and has the following side effects:
 *
 *  o  assigns a struct pci_dev * to dev->private->pci_dev 
 *  o  assigns a struct board * to dev->board_ptr
 *  o  sets dev->private->registers
 *  o  sets dev->private->dio_registers
 *
 *  Otherwise, returns a -errno on error
 */
static int probe(comedi_device *dev, const comedi_devconfig *it) 
{
    struct pci_dev *pcidev;
	int index, registers;

	pci_for_each_dev(pcidev)
	{
		// is it not a computer boards card?
		if(pcidev->vendor != PCI_VENDOR_ID_COMPUTERBOARDS)
			continue;
		// loop through cards supported by this driver
		for(index = 0; index < N_BOARDS; index++)
		{
			if(boards[index].device_id != pcidev->device)
				continue;
			// was a particular bus/slot requested?
			if(it->options[0] || it->options[1])
			{
				// are we on the wrong bus/slot?
				if(pcidev->bus->number != it->options[0] ||
				   PCI_SLOT(pcidev->devfn) != it->options[1])
				{
					continue;
				}
			}
			/* found ! */

			/* todo: if we support more than 1 board, revise
			   this to be more generic */            
			devpriv->pci_dev = pcidev;
			pci_enable_device(devpriv->pci_dev); /* make sure board is on */
			dev->board_ptr = boards + index;
			registers = pci_resource_start(devpriv->pci_dev, REGS_BADRINDEX);
			if (!request_region(registers, REG_SZ,thisboard->name))
			{
			  printk("cb_pcimdda: "
				 "I/O port conflict failed to allocate ports "
				 "0x%x to 0x%x\n", registers, 
				 registers + REG_SZ - 1);
			  return -EBUSY;
			}
			devpriv->registers = registers;                        
			devpriv->dio_registers 
			  = devpriv->registers + thisboard->dio_offset;
			return 0;
		}
	}

    printk("cb_pcimdda: No supported ComputerBoards/MeasurementComputing "
           "card found at the requested position\n");
	return -ENODEV;
}


/* returns a maxdata value for a given n_bits */
static inline lsampl_t figure_out_maxdata(int bits)
{
    lsampl_t max = 0;
    int i;

    for (i = 0; i < bits; i++) {
      max <<= 1;
      max |=  1U;
    }
    return max;
}


/*----------------------------------------------------------------------------
   LINUX KERNEL MODULE STUFF...
-----------------------------------------------------------------------------*/
MODULE_AUTHOR("Calin A. Culianu <calin@rtlab.org>");             
MODULE_DESCRIPTION("Comedi low-level driver for the Computerboards PCIM-DDA "
                   "series.  Currently only supports PCIM-DDA06-16 (which "
                   "also happens to be the only board in this series. :) ) "); 
MODULE_LICENSE("GPL");                                          

/* Entry point into this module.. */
int init_module(void)             
{ 
  return comedi_driver_register(&cb_pcimdda_driver);
}                  

/* Module Exit/cleanup.. */
void cleanup_module(void)                  
{
  comedi_driver_unregister(&cb_pcimdda_driver);
}


