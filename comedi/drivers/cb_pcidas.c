/*
    cb_pcidas.c
    This intends to be a driver for the ComputerBoards PCI-DAS series.
    SO FAR IT WAS ONLY TESTED WITH PCI-DAS1200. PLEASE REPORT IF YOU ARE
		USING IT WITH A DIFFERENT CARD <ivanmr@altavista.com>.

		Options:
		[0] - PCI bus number
		[1] - PCI slot number

    Copyright (C) 2001 Ivan Martinez <ivanmr@altavista.com>, with
    valuable help from David Schleef, Frank Mori Hess, and the rest of
    the Comedi developers comunity.

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
#define N_BOARDS	10	// Number of boards in cb_pcidas_boards

/* PCI-DAS base addresses */
#define CONT_STAT_BADRINDEX	1
	// CONTROL/STATUS is devpriv->pci_dev->base_address[1]
#define ADC_FIFO_BADRINDEX	2
	// ADC DATA, FIFO CLEAR is devpriv->pci_dev->base_address[2]
/* Control/Status registers */
#define INT_ADCFIFO	0	// INTERRUPT / ADC FIFO register
#define NOINT 0020300	// Clears and disables interrupts

#define ADCMUX_CONT	2	// ADC CHANNEL MUX AND CONTROL register
#define RANGE10V	0000000	// 10V
#define RANGE5V	0000400	// 5V
#define RANGE2V5	0001000	// 2.5V
#define RANGE1V25	0001400	// 1.25V
#define UNIP	0004000	// Analog front-end unipolar for range
#define BIP	0000000	// Analog front-end bipolar for range
#define SE	0002000	// Inputs in single-ended mode
#define DIFF	0000000	// Inputs in differential mode
#define	SWPACER	0000000	// Pacer source is SW convert
#define EOC	0040000	// End of conversion

#define TRIG_CONTSTAT 4 // TRIGGER CONTROL/STATUS register

#define CALIBRATION	6	// CALIBRATION register

/* ADC data, FIFO clear registers */
#define ADCDATA	0	// ADC DATA register
#define BEGSWCONV	0	// Begin software conversion

#define ADCFIFOCLR	2	// ADC FIFO CLEAR
#define CLEARFIFO	0 // Clear fifo

#define BIPRANGES 4	// Number of bipolar ranges in cb_pcidas_ranges

comedi_lrange cb_pcidas_ranges =
{
	8,
	{
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2.5),
		BIP_RANGE(1.25),
		UNI_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(2.5),
		UNI_RANGE(1.25)
	}
};

/*
 * Board descriptions for two imaginary boards.  Describing the
 * boards in this way is optional, and completely driver-dependent.
 * Some drivers use arrays such as this, other do not.
 */
typedef struct cb_pcidas_board_struct
{
	char *name;
	char status;		// Driver status:
				// 0 - tested
				// 1 - manual read, not tested
				// 2 - manual not read
	unsigned short device_id;
	int ai_se_chans;	// Inputs in single-ended mode
	int ai_diff_chans;	// Inputs in differential mode
	int ai_bits;
	comedi_lrange *ranges;
} cb_pcidas_board;
static cb_pcidas_board cb_pcidas_boards[] =
{
	{
		name:		"pci-das1602/16",
		status:		2,
		device_id:	0x1,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	16,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1200",
		status:		0,
		device_id:	0xF,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1602/12",
		status:		2,
		device_id:	0x10,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1200/jr",
		status:		1,
		device_id:	0x19,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1602/16/jr",
		status:		2,
		device_id:	0x1C,
		ai_se_chans:	64,
		ai_diff_chans:	8,
		ai_bits:	16,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das6402/16",
		status:		2,
		device_id:	0x1D,
		ai_se_chans:	64,
		ai_diff_chans:	32,
		ai_bits:	16,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das64/m1/16",
		status:		2,
		device_id:	0x35,
		ai_se_chans:	64,
		ai_diff_chans:	32,
		ai_bits:	16,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das64/m2/16",
		status:		2,
		device_id:	0x36,
		ai_se_chans:	64,
		ai_diff_chans:	32,
		ai_bits:	16,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das64/m3/16",
		status:		2,
		device_id:	0x37,
		ai_se_chans:	64,
		ai_diff_chans:	32,
		ai_bits:	16,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1000",
		status:		2,
		device_id:	0x4C,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ranges:	&cb_pcidas_ranges,
	},
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((cb_pcidas_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct
{
	int data;

	/* would be useful for a PCI device */
	struct pci_dev *pci_dev;

	unsigned long control_status;
	unsigned long adc_fifo;

} cb_pcidas_private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((cb_pcidas_private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int cb_pcidas_attach(comedi_device *dev,comedi_devconfig *it);
static int cb_pcidas_detach(comedi_device *dev);
comedi_driver driver_cb_pcidas={
	driver_name:	"cb_pcidas",
	module:		THIS_MODULE,
	attach:		cb_pcidas_attach,
	detach:		cb_pcidas_detach,
};

static int cb_pcidas_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
//static int cb_pcidas_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
//static int cb_pcidas_ai_cmd(comedi_device *dev,comedi_subdevice *s);
static int cb_pcidas_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);
static int cb_pcidas_ns_to_timer(unsigned int *ns,int round);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.
 */
static int cb_pcidas_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	struct pci_dev* pcidev;
	int index;

	printk("comedi%d: cb_pcidas: ",dev->minor);
	
/*
 * Allocate the private structure area.
 */
	if(alloc_private(dev,sizeof(cb_pcidas_private))<0)
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
		printk("Not a ComputerBoards card on requested position\n");
		return -EIO;
	}

	for(index=0;index<N_BOARDS;index++){
		if(cb_pcidas_boards[index].device_id == pcidev->device){
			goto found;
		}
	}
	printk("Not a supported ComputerBoards card on requested position\n");			
	return -EIO;

found:
	devpriv->pci_dev = pcidev;
	dev->board_ptr = cb_pcidas_boards+index;
	printk("Found %s at requested position\n",cb_pcidas_boards[index].name);

	/*
	 * Initialize devpriv->control_status and devpriv->adc_fifo to point to
	 * their base address.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
	devpriv->control_status =
		devpriv->pci_dev->base_address[CONT_STAT_BADRINDEX] &
			PCI_BASE_ADDRESS_IO_MASK;
	devpriv->adc_fifo =
		devpriv->pci_dev->base_address[ADC_FIFO_BADRINDEX] &
			PCI_BASE_ADDRESS_IO_MASK;
#else
	devpriv->control_status =
		devpriv->pci_dev->resource[CONT_STAT_BADRINDEX].start &
			PCI_BASE_ADDRESS_IO_MASK;
	devpriv->adc_fifo =
		devpriv->pci_dev->resource[ADC_FIFO_BADRINDEX].start &
			PCI_BASE_ADDRESS_IO_MASK;
#endif

/*
 * Warn about the status of the driver.
 */
	if (thisboard->status == 2)
		printk("WARNING: DRIVER FOR THIS BOARD NOT CHECKED WITH MANUAL. "
			"WORKS ASSUMING FULL COMPATIBILITY WITH PCI-DAS1200. "
			"PLEASE REPORT USAGE TO <ivanmr@altavista.com>.\n");

/*
 * Initialize dev->board_name.  Note that we can use the "thisboard"
 * macro now, since we just initialized it in the last line.
 */
	dev->board_name = thisboard->name;

/*
 * Allocate the subdevice structures.
 */
	dev->n_subdevices=1;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog input subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON | SDF_DIFF;
	/* WARNING: Number of inputs in differential mode is ignored */
	s->n_chan = thisboard->ai_se_chans;
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = thisboard->ranges;
	s->insn_read = &cb_pcidas_ai_rinsn;
//	s->do_cmd = &cb_pcidas_ai_cmd;
	s->do_cmdtest = &cb_pcidas_ai_cmdtest;
	
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
static int cb_pcidas_detach(comedi_device *dev)
{
	printk("comedi%d: cb_pcidas: remove\n",dev->minor);

	return 0;
}

/*
 * "instructions" read/write data in "one-shot" or "software-triggered"
 * mode.
 */
static int cb_pcidas_ai_rinsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int n;
	unsigned int d;
	unsigned int command;

	/* a typical programming sequence */

	/* inputs mode */
	if (CR_AREF(insn->chanspec) == AREF_DIFF)
		command = DIFF;
	else
		command = SE;

	/* input signals range */
	if (CR_RANGE(insn->chanspec) < BIPRANGES)
		command |= BIP | (CR_RANGE(insn->chanspec) << 8);
	else
		command |= UNIP | ((CR_RANGE(insn->chanspec) - BIPRANGES) << 8);
		
	/* write channel to multiplexer */
	command |= CR_CHAN(insn->chanspec) | (CR_CHAN(insn->chanspec) << 4);
	
	outw_p(command,	devpriv->control_status + ADCMUX_CONT);

	/* wait for mux to settle */
	/* I suppose I made it with outw_p... */

	/* convert n samples */
	for (n = 0; n < insn->n; n++)
	{
		/* clear fifo */
		outw(CLEARFIFO, devpriv->adc_fifo + ADCFIFOCLR);

		/* trigger conversion */
		outw(BEGSWCONV, devpriv->adc_fifo + ADCDATA);

		/* wait for conversion to end */
		/* return -ETIMEDOUT if there is a timeout */
		while (!(inw(devpriv->control_status + ADCMUX_CONT) & EOC));

		/* read data */
		d = inw(devpriv->adc_fifo + ADCDATA);

		data[n] = d;
	}

	/* return the number of samples read/written */
	return n;
}

/*
 * I will program this later... ;-)
 *
static int cb_pcidas_ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	printk("cb_pcidas_ai_cmd\n");
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

static int cb_pcidas_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
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
		cb_pcidas_ns_to_timer(&cmd->scan_begin_arg, cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->scan_begin_arg)
			err++;
	}
	if(cmd->convert_src == TRIG_TIMER)
	{
		tmp=cmd->convert_arg;
		cb_pcidas_ns_to_timer(&cmd->convert_arg, cmd->flags & TRIG_ROUND_MASK);
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
static int cb_pcidas_ns_to_timer(unsigned int *ns,int round)
{
	/* trivial timer */
	return *ns;
}

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_cb_pcidas);

