/*
    cb_pcidda.c
    This intends to be a driver for the ComputerBoards / MeasurementComputing
    PCI-DDA series. SO FAR IT HAS ONLY BEEN TESTED WITH:
    - PCI-DDA08/12
    PLEASE REPORT IF YOU ARE USING IT WITH A DIFFERENT CARD
		<ivanmr@altavista.com>.

		Options:
		[0] - PCI bus number
		[1] - PCI slot number

    Developed by Ivan Martinez <ivanmr@altavista.com>.

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-8 David A. Schleef <ds@stm.lbl.gov>

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
#include <linux/pci.h>
#include <asm/io.h>
#include <linux/comedidev.h>

#define PCI_VENDOR_CB	0x1307	// PCI vendor number of ComputerBoards
#define N_BOARDS	10	// Number of boards in cb_pcidda_boards

/* PCI-DDA base addresses */
#define DIGITALIO_BADRINDEX	2
	// DIGITAL I/O is devpriv->pci_dev->base_address[2]
#define DIGITALIO_SIZE 8
	// DIGITAL I/O uses 8 I/O port addresses
#define DAC_BADRINDEX	3
	// DAC is devpriv->pci_dev->base_address[3]

/* Digital I/O registers */
#define PORT1A 0	// PORT 1A DATA

#define PORT1B 1	// PORT 1B DATA

#define PORT1C 2	// PORT 1C DATA

#define CONTROL1 3	// CONTROL REGISTER 1

#define PORT2A 4	// PORT 2A DATA

#define PORT2B 5	// PORT 2B DATA

#define PORT2C 6	// PORT 2C DATA

#define CONTROL2 7	// CONTROL REGISTER 2

/* DAC registers */
#define DACONTROL	0	// D/A CONTROL REGISTER
#define	SU	0000001	// Simultaneous update enabled
#define NOSU	0000000	// Simultaneous update disabled
#define	ENABLEDAC	0000002	// Enable specified DAC
#define	DISABLEDAC	0000000	// Disable specified DAC
#define RANGE2V5	0000000	// 2.5V
#define RANGE5V	0000200	// 5V
#define RANGE10V	0000300	// 10V
#define UNIP	0000400	// Unipolar outputs
#define BIP	0000000	// Bipolar outputs

#define DACALIBRATION1	4	// D/A CALIBRATION REGISTER 1

#define DACALIBRATION2	6 // D/A CALIBRATION REGISTER 2

#define DADATA	8	// FIRST D/A DATA REGISTER (0)

comedi_lrange cb_pcidda_ranges =
{
	6,
	{
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2.5),
		UNI_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(2.5),
	}
};

/*
 * Board descriptions for two imaginary boards.  Describing the
 * boards in this way is optional, and completely driver-dependent.
 * Some drivers use arrays such as this, other do not.
 */
typedef struct cb_pcidda_board_struct
{
	char *name;
	char status;		// Driver status:
				// 0 - tested
				// 1 - manual read, not tested
				// 2 - manual not read
	unsigned short device_id;
	int ao_chans;
	int ao_bits;
	comedi_lrange *ranges;
} cb_pcidda_board;
static cb_pcidda_board cb_pcidda_boards[] =
{
	{
		name:		"pci-dda02/12",
		status:		1,
		device_id:	0x20,
		ao_chans:	2,
		ao_bits:	12,
		ranges:	&cb_pcidda_ranges,
	},
	{
		name:		"pci-dda04/12",
		status:		1,
		device_id:	0x21,
		ao_chans:	4,
		ao_bits:	12,
		ranges:	&cb_pcidda_ranges,
	},
	{
		name:		"pci-dda08/12",
		status:		0,
		device_id:	0x22,
		ao_chans:	8,
		ao_bits:	12,
		ranges:	&cb_pcidda_ranges,
	},
	{
		name:		"pci-dda02/16",
		status:		2,
		device_id:	0x23,
		ao_chans:	2,
		ao_bits:	16,
		ranges:	&cb_pcidda_ranges,
	},
	{
		name:		"pci-dda04/16",
		status:		2,
		device_id:	0x24,
		ao_chans:	4,
		ao_bits:	16,
		ranges:	&cb_pcidda_ranges,
	},
	{
		name:		"pci-dda08/16",
		status:		2,
		device_id:	0x25,
		ao_chans:	8,
		ao_bits:	16,
		ranges:	&cb_pcidda_ranges,
	},
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((cb_pcidda_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct
{
	int data;

	/* would be useful for a PCI device */
	struct pci_dev *pci_dev;

	unsigned long digitalio;
	unsigned long dac;
	//unsigned long control_status;
	//unsigned long adc_fifo;

} cb_pcidda_private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((cb_pcidda_private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int cb_pcidda_attach(comedi_device *dev,comedi_devconfig *it);
static int cb_pcidda_detach(comedi_device *dev);
comedi_driver driver_cb_pcidda={
	driver_name:	"cb_pcidda",
	module:		THIS_MODULE,
	attach:		cb_pcidda_attach,
	detach:		cb_pcidda_detach,
};

//static int cb_pcidda_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int cb_pcidda_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
//static int cb_pcidda_ai_cmd(comedi_device *dev,comedi_subdevice *s);
static int cb_pcidda_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);
static int cb_pcidda_ns_to_timer(unsigned int *ns,int round);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.
 */
static int cb_pcidda_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	struct pci_dev* pcidev;
	int index;
	unsigned int dac, digitalio;

	printk("comedi%d: cb_pcidda: ",dev->minor);
	
/*
 * Allocate the private structure area.
 */
	if(alloc_private(dev,sizeof(cb_pcidda_private))<0)
		return -ENOMEM;

/*
 * Probe the device to determine what device in the series it is.
 */
	printk("\n");

	pci_for_each_dev(pcidev){
		if(pcidev->vendor==PCI_VENDOR_CB){
			if(it->options[0] || it->options[1]){
				if(pcidev->bus->number==it->options[0] &&
				   PCI_SLOT(pcidev->devfn)==it->options[1]){
					break;
				}
			}else{
				break;
			}
		}
	}

	if(!pcidev){
		printk("Not a ComputerBoards/MeasurementComputing card on requested position\n");
		return -EIO;
	}

	for(index=0;index<N_BOARDS;index++){
		if(cb_pcidda_boards[index].device_id == pcidev->device){
			goto found;
		}
	}
	printk("Not a supported ComputerBoards/MeasurementComputing card on "
		"requested position\n");			
	return -EIO;

found:
	devpriv->pci_dev = pcidev;
	dev->board_ptr = cb_pcidda_boards+index;
	// "thisboard" macro can be used from here.
	printk("Found %s at requested position\n",thisboard->name);

	/*
	 * Initialize devpriv->control_status and devpriv->adc_fifo to point to
	 * their base address.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
	digitalio =
		devpriv->pci_dev->base_address[DIGITALIO_BADRINDEX] &
			PCI_BASE_ADDRESS_IO_MASK;
	dac =
		devpriv->pci_dev->base_address[DAC_BADRINDEX] &
			PCI_BASE_ADDRESS_IO_MASK;
#else
	digitalio =
		devpriv->pci_dev->resource[DIGITALIO_BADRINDEX].start &
			PCI_BASE_ADDRESS_IO_MASK;
	dac =
		devpriv->pci_dev->resource[DAC_BADRINDEX].start &
			PCI_BASE_ADDRESS_IO_MASK;
#endif

/*
 * Allocate the I/O ports.
 */
	if (check_region(digitalio, DIGITALIO_SIZE) < 0)
	{
		printk("I/O port conflict: failed to allocate ports 0x%x to 0x%x\n",
			digitalio, digitalio + DIGITALIO_SIZE - 1);
		return -EIO;
	}
	if (check_region(dac, 8 + thisboard->ao_chans*2) < 0)
	{
		printk("I/O port conflict: failed to allocate ports 0x%x to 0x%x\n",
			dac, dac + 7 + thisboard->ao_chans*2);
		return -EIO;
	}
	request_region(digitalio, DIGITALIO_SIZE, thisboard->name);
	devpriv->digitalio = digitalio;
	request_region(dac, 8 + thisboard->ao_chans*2,
		thisboard->name);
	devpriv->dac = dac;

/*
 * Warn about the status of the driver.
 */
	if (thisboard->status == 2)
		printk("WARNING: DRIVER FOR THIS BOARD NOT CHECKED WITH MANUAL. "
			"WORKS ASSUMING FULL COMPATIBILITY WITH PCI-DDA08/12. "
			"PLEASE REPORT USAGE TO <ivanmr@altavista.com>.\n");

/*
 * Initialize dev->board_name.
 */
	dev->board_name = thisboard->name;

/*
 * Allocate the subdevice structures.
 */
	dev->n_subdevices=1;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog output subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = thisboard->ao_chans;
	s->maxdata = (1 << thisboard->ao_bits) - 1;
	s->range_table = thisboard->ranges;
	s->insn_write = &cb_pcidda_ao_winsn;
//	s->do_cmd = &cb_pcidda_ai_cmd;
	s->do_cmdtest = &cb_pcidda_ai_cmdtest;
	
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
static int cb_pcidda_detach(comedi_device *dev)
{
/*
 * Deallocate the I/O ports.
 */
	if(devpriv)
	{
		if(devpriv->digitalio)
			release_region(devpriv->digitalio, DIGITALIO_SIZE);
		if(devpriv->dac)
			release_region(devpriv->dac, 8 + thisboard->ao_chans*2);
	}
	printk("comedi%d: cb_pcidda: remove\n",dev->minor);

	return 0;
}


/*
 * I will program this later... ;-)
 *
static int cb_pcidda_ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	printk("cb_pcidda_ai_cmd\n");
	printk("subdev: %d\n", cmd->subdev);
	printk("flags: %d\n", cmd->flags);
	printk("start_src: %d\n", cmd->start_src);
	printk("start_arg: %d\n", cmd->start_arg);
	printk("scan_begin_src: %d\n", cmd->scan_begin_src);
	printk("convert_src: %d\n", cmd->convert_src);
	printk("convert_arg: %d\n", cmd->convert_arg);
	printk("scan_end_src: %d\n", cmd->scan_end_src);
	printk("scan_end_arg: %d\n", cmd->scan_end_arg);
	printk("stop_src: %d\n", cmd->stop_src);
	printk("stop_arg: %d\n", cmd->stop_arg);
	printk("chanlist_len: %d\n", cmd->chanlist_len);
}
*/

static int cb_pcidda_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
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

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src && tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER | TRIG_EXT;
	if(!cmd->scan_begin_src && tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if(!cmd->convert_src && tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src && tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if(!cmd->stop_src && tmp != cmd->stop_src)
		err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	/* note that mutual compatiblity is not an issue here */
	if(cmd->scan_begin_src != TRIG_TIMER && cmd->scan_begin_src != TRIG_EXT)
		err++;
	if(cmd->convert_src != TRIG_TIMER && cmd->convert_src != TRIG_EXT)
		err++;
	if(cmd->stop_src != TRIG_TIMER && cmd->stop_src != TRIG_EXT)
		err++;

	if(err) return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0)
	{
		cmd->start_arg=0;
		err++;
	}

#define MAX_SPEED	10000		/* in nanoseconds */
#define MIN_SPEED	1000000000	/* in nanoseconds */

	if (cmd->scan_begin_src == TRIG_TIMER)
	{
		if (cmd->scan_begin_arg < MAX_SPEED)
		{
			cmd->scan_begin_arg = MAX_SPEED;
			err++;
		}
		if (cmd->scan_begin_arg > MIN_SPEED)
		{
			cmd->scan_begin_arg = MIN_SPEED;
			err++;
		}
	}
	else
	{
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		/* should specify multiple external triggers */
		if (cmd->scan_begin_arg > 9)
		{
			cmd->scan_begin_arg = 9;
			err++;
		}
	}
	if (cmd->convert_src == TRIG_TIMER)
	{
		if (cmd->convert_arg < MAX_SPEED)
		{
			cmd->convert_arg = MAX_SPEED;
			err++;
		}
		if (cmd->convert_arg>MIN_SPEED)
		{
			cmd->convert_arg = MIN_SPEED;
			err++;
		}
	}
	else
	{
		/* external trigger */
		/* see above */
		if (cmd->convert_arg > 9)
		{
			cmd->convert_arg = 9;
			err++;
		}
	}

	if(cmd->scan_end_arg != cmd->chanlist_len)
	{
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src == TRIG_COUNT)
	{
		if(cmd->stop_arg > 0x00ffffff)
		{
			cmd->stop_arg = 0x00ffffff;
			err++;
		}
	}
	else
	{
		/* TRIG_NONE */
		if (cmd->stop_arg != 0)
		{
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	if(cmd->scan_begin_src == TRIG_TIMER)
	{
		tmp = cmd->scan_begin_arg;
		cb_pcidda_ns_to_timer(&cmd->scan_begin_arg, cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->scan_begin_arg)
			err++;
	}
	if(cmd->convert_src == TRIG_TIMER)
	{
		tmp=cmd->convert_arg;
		cb_pcidda_ns_to_timer(&cmd->convert_arg, cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->convert_arg)
			err++;
		if(cmd->scan_begin_src == TRIG_TIMER &&
		  cmd->scan_begin_arg < cmd->convert_arg * cmd->scan_end_arg)
		{
			cmd->scan_begin_arg = cmd->convert_arg * cmd->scan_end_arg;
			err++;
		}
	}

	if(err) return 4;

	return 0;
}

/* This function doesn't require a particular form, this is just
 * what happens to be used in some of the drivers.  It should
 * convert ns nanoseconds to a counter value suitable for programming
 * the device.  Also, it should adjust ns so that it cooresponds to
 * the actual time that the device will use. */
static int cb_pcidda_ns_to_timer(unsigned int *ns,int round)
{
	/* trivial timer */
	return *ns;
}

static int cb_pcidda_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	unsigned int command;

	/* output channel configuration */
	command = NOSU | ENABLEDAC;

	/* output channel range */
	switch (CR_RANGE(insn->chanspec))
	{
		case 0:
			command |= BIP | RANGE10V;
			break;
		case 1:
			command |= BIP | RANGE5V;
			break;
		case 2:
			command |= BIP | RANGE2V5;
			break;
		case 3:
			command |= UNIP | RANGE10V;
			break;
		case 4:
			command |= UNIP | RANGE5V;
			break;
		case 5:
			command |= UNIP | RANGE2V5;
			break;
	};

	/* output channel specification */
	command |= CR_CHAN(insn->chanspec) << 2;
	outw(command, devpriv->dac + DACONTROL);

	/* write data */
	outw(data[0], devpriv->dac + DADATA + CR_CHAN(insn->chanspec)*2);

	/* return the number of samples read/written */
	return 1;
}

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_cb_pcidda);

