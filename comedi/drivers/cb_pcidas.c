/*
    cb_pcidas.c
    This is a driver for the ComputerBoards/MeasurementComputing PCI-DAS
    cards using the AMCC S5933 PCI controller:
    - PCI-DAS1602/12, 1602/16, 1602/16/jr
    - PCI-DAS1200, 1200jr
    - PCI-DAS1000, 1001, 1002


    SO FAR IT WAS ONLY TESTED WITH PCI-DAS1200. PLEASE REPORT IF YOU ARE
    USING IT WITH A DIFFERENT CARD <ivanmr@altavista.com>.

    Options:
    [0] - PCI bus number
    [1] - PCI slot number

    Developed by Ivan Martinez and Frank Mori Hess, with valuable help from
    David Schleef and the rest of the Comedi developers comunity.

    Copyright (C) 2001 Ivan Martinez <ivanmr@altavista.com>
    Copyright (C) 2001 Frank Mori Hess <fmhess@uiuc.edu>

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

************************************************************************

TODO:

add a calibration subdevice

add analog out support
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
#include "8253.h"
#include "8255.h"

#define CB_PCIDAS_DEBUG	// enable debugging code
//#undef CB_PCIDAS_DEBUG	// disable debugging code

// PCI vendor number of ComputerBoards/MeasurementComputing
#define PCI_VENDOR_CB	0x1307
#define TIMER_BASE 100	// 10MHz master clock

/* PCI-DAS base addresses */

// indices of base address regions
#define S5933_BADRINDEX 0
#define CONT_STAT_BADRINDEX 1
#define ADC_FIFO_BADRINDEX 2
#define PACER_BADRINDEX 3
#define AO_BADRINDEX 4
// sizes of io regions
#define S5933_SIZE 64
#define CONT_STAT_SIZE 10
#define ADC_FIFO_SIZE 4
#define PACER_SIZE 12
#define AO_SIZE 4

// amcc s5933 pci configuration registers
#define INTCSR	0x38	// interrupt control/status
#define   OUTBOX_BYTE(x)	((x) & 0x3)
#define   OUTBOX_SELECT(x)	(((x) & 0x3) << 2)
#define   OUTBOX_EMPTY_INT	0x10	// enable outbox empty interrupt
#define   INBOX_BYTE(x)	(((x) & 0x3) << 8)
#define   INBOX_SELECT(x)	(((x) & 0x3) << 10)
#define   INBOX_FULL_INT	0x1000	// enable inbox full interrupt
#define   INBOX_INTR_STATUS	0x20000 // read, or write clear inbox full interrupt

/* Control/Status registers */
#define INT_ADCFIFO	0	// INTERRUPT / ADC FIFO register
#define   INT_EOS 0x1	// interrupt end of scan
#define   INT_FHF 0x2	// interrupt fifo half full
#define   INT_FNE 0x3	// interrupt fifo not empty
#define   INTE 0x4	// interrupt enable
#define   EOACL 0x40	// write-clear for end of acquisition interrupt status
#define   EOAI 0x40	// read end of acq. interrupt status
#define   INTCL 0x80	//	write-clear for EOS/FHF/FNE interrupt status
#define   INT 0x80	// read interrupt status
#define   EOBI 0x200	// read end of burst interrupt status
#define   ADHFI 0x400	// read half-full interrupt status
#define   ADNEI 0x800	// read fifo not empty interrupt latch status
#define   ADNE 0x1000	// fifo not empty (realtime, not latched) status
#define   ADFLCL 0x2000	// write-clear for fifo full status
#define   LADFUL 0x2000	// read fifo overflow

#define ADCMUX_CONT	2	// ADC CHANNEL MUX AND CONTROL register
#define   BEGIN_SCAN(x)	((x) & 0xf)
#define   END_SCAN(x)	(((x) & 0xf) << 4)
#define   GAIN_BITS(x)	(((x) & 0x3) << 8)
#define   UNIP	0004000	// Analog front-end unipolar for range
#define   SE	0002000	// Inputs in single-ended mode
#define   PACER_MASK	0x3000	// pacer source bits
#define   PACER_INT 0x1000	// internal pacer
#define   PACER_EXT_FALL	0x2000	// external falling edge
#define   PACER_EXT_RISE	0x3000	// external rising edge

#define TRIG_CONTSTAT 4 // TRIGGER CONTROL/STATUS register
#define   SW_TRIGGER 0x1	// software start trigger
#define   EXT_TRIGGER 0x2	// external start trigger
#define   TGEN	0x10	// enable external start trigger
#define   BURSTE 0x20	// burst mode enable
#define   XTRCL	0x80	// clear external trigger

#define CALIBRATION	6	// CALIBRATION register

/* ADC data, FIFO clear registers */
#define ADCDATA	0	// ADC DATA register
#define ADCFIFOCLR	2	// ADC FIFO CLEAR

// pacer, counter, dio registers
#define ADC8254 0
#define DIO_8255 4

// bit in hexadecimal representation of range index that indicates unipolar range
#define IS_UNIPOLAR 0x4
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

comedi_lrange cb_pcidas_alt_ranges =
{
	8,
	{
		BIP_RANGE(10),
		BIP_RANGE(1),
		BIP_RANGE(0.1),
		BIP_RANGE(0.01),
		UNI_RANGE(10),
		UNI_RANGE(1),
		UNI_RANGE(0.1),
		UNI_RANGE(0.01)
	}
};

typedef struct cb_pcidas_board_struct
{
	char *name;
	unsigned short device_id;
	int ai_se_chans;	// Inputs in single-ended mode
	int ai_diff_chans;	// Inputs in differential mode
	int ai_bits;	// analog input resolution
	int ai_speed;	// fastest conversion period in ns
	// number of analog outputs
	int ao_nchan;
	comedi_lrange *ranges;
} cb_pcidas_board;

static cb_pcidas_board cb_pcidas_boards[] =
{
	{
		name:		"pci-das1602/16",
		device_id:	0x1,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	16,
		ai_speed:	5000,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1200",
		device_id:	0xF,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	3200,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1602/12",
		device_id:	0x10,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	3200,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1200/jr",
		device_id:	0x19,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	3200,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1602/16/jr",
		device_id:	0x1C,
		ai_se_chans:	64,
		ai_diff_chans:	8,
		ai_bits:	16,
		ai_speed: 5000,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1000",
		device_id:	0x4C,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	4000,
		ao_nchan: 0,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1001",
		device_id:	0x1a,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	6800,
		ao_nchan: 2,
		ranges:	&cb_pcidas_alt_ranges,
	},
	{
		name:		"pci-das1002",
		device_id:	0x1b,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	6800,
		ao_nchan: 2,
		ranges:	&cb_pcidas_ranges,
	},
};
// Number of boards in cb_pcidas_boards
#define N_BOARDS	(sizeof(cb_pcidas_boards) / sizeof(cb_pcidas_board))

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

	// base addresses
	unsigned int s5933_config;
	unsigned int control_status;
	unsigned int adc_fifo;
	unsigned int pacer_counter_dio;
	unsigned int ao_registers;
	// divisors of master clock for pacing
	unsigned int divisor1;
	unsigned int divisor2;
	volatile unsigned int count;	//number of samples remaining
	unsigned int adc_fifo_bits;	// bits to write to interupt/adcfifo register
	unsigned int s5933_intcsr_bits;	// bits to write to amcc s5933 interrupt control/status register
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
static int cb_pcidas_ai_cmd(comedi_device *dev,comedi_subdevice *s);
static int cb_pcidas_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);
static void cb_pcidas_interrupt(int irq, void *d, struct pt_regs *regs);
static int cb_pcidas_cancel(comedi_device *dev, comedi_subdevice *s);
void cb_pcidas_load_counters(comedi_device *dev, unsigned int *ns, int round_flags);
/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.
 */
static int cb_pcidas_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	struct pci_dev* pcidev;
	int index;
	unsigned long s5933_config, control_status, adc_fifo,
		pacer_counter_dio, ao_registers;
	int err;

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
		printk("Not a ComputerBoards/MeasurementComputing card on requested "
			"position\n");
		return -EIO;
	}

	for(index=0;index<N_BOARDS;index++){
		if(cb_pcidas_boards[index].device_id == pcidev->device){
			goto found;
		}
	}
	printk("Not a supported ComputerBoards/MeasurementComputing card on "
		"requested position\n");
	return -EIO;

found:
	devpriv->pci_dev = pcidev;
	dev->board_ptr = cb_pcidas_boards+index;
	// thisboard macro can be used from here

	printk("Found %s at requested position\n",cb_pcidas_boards[index].name);

	// Warn about non-tested features
	switch(thisboard->device_id)
	{
		case 0x1:
		case 0x10:
		case 0x1C:
		case 0x4C:
		case 0x1A:
		case 0x1B:
			printk("DRIVER HASN'T BEEN TESTED WITH THIS CARD. PLEASE REPORT "
				"USAGE TO <ivanmr@altavista.com>\n");
	};


	/*
	 * Initialize devpriv->control_status and devpriv->adc_fifo to point to
	 * their base address.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
	s5933_config =
		devpriv->pci_dev->base_address[S5933_BADRINDEX] &
		PCI_BASE_ADDRESS_IO_MASK;
	control_status =
		devpriv->pci_dev->base_address[CONT_STAT_BADRINDEX] &
		PCI_BASE_ADDRESS_IO_MASK;
	adc_fifo =
		devpriv->pci_dev->base_address[ADC_FIFO_BADRINDEX] &
		PCI_BASE_ADDRESS_IO_MASK;
	pacer_counter_dio =
		devpriv->pci_dev->base_address[PACER_BADRINDEX] &
		PCI_BASE_ADDRESS_IO_MASK;
	ao_registers =
		devpriv->pci_dev->base_address[AO_BADRINDEX] &
		PCI_BASE_ADDRESS_IO_MASK;
#else
	pci_enable_device(devpriv->pci_dev);
	s5933_config =
		devpriv->pci_dev->resource[S5933_BADRINDEX].start &
		PCI_BASE_ADDRESS_IO_MASK;
	control_status =
		devpriv->pci_dev->resource[CONT_STAT_BADRINDEX].start &
		PCI_BASE_ADDRESS_IO_MASK;
	adc_fifo =
		devpriv->pci_dev->resource[ADC_FIFO_BADRINDEX].start &
		PCI_BASE_ADDRESS_IO_MASK;
	pacer_counter_dio =
		devpriv->pci_dev->resource[PACER_BADRINDEX].start &
		PCI_BASE_ADDRESS_IO_MASK;
	ao_registers =
		devpriv->pci_dev->resource[AO_BADRINDEX].start &
		PCI_BASE_ADDRESS_IO_MASK;
#endif

	// reserve io ports
	err = 0;
	if(check_region(s5933_config, S5933_SIZE) < 0)
		err++;
	if(check_region(control_status, CONT_STAT_SIZE) < 0)
		err++;
	if(check_region(adc_fifo, ADC_FIFO_SIZE) < 0)
		err++;
	if(check_region(pacer_counter_dio, PACER_SIZE) < 0)
		err++;
	if(thisboard->ao_nchan)
		if(check_region(ao_registers, AO_SIZE) < 0)
			err++;
	if(err)
	{
		printk(" I/O port conflict\n");
		return -EIO;
	}
	request_region(s5933_config, S5933_SIZE, "cb_pcidas");
	devpriv->s5933_config = s5933_config;
	request_region(control_status, CONT_STAT_SIZE, "cb_pcidas");
	devpriv->control_status = control_status;
	request_region(adc_fifo, ADC_FIFO_SIZE, "cb_pcidas");
	devpriv->adc_fifo = adc_fifo;
	request_region(pacer_counter_dio, PACER_SIZE, "cb_pcidas");
	devpriv->pacer_counter_dio = pacer_counter_dio;
	if(thisboard->ao_nchan)
	{
		request_region(ao_registers, AO_SIZE, "cb_pcidas");
		devpriv->ao_registers = ao_registers;
	}

	// get irq
	if(comedi_request_irq(devpriv->pci_dev->irq, cb_pcidas_interrupt, SA_SHIRQ, "cb_pcidas", dev ))
	{
		printk(" unable to allocate irq %d\n", devpriv->pci_dev->irq);
		return -EINVAL;
	}
	dev->irq = devpriv->pci_dev->irq;

	//Initialize dev->board_name
	dev->board_name = thisboard->name;

/*
 * Allocate the subdevice structures.
 */
	dev->n_subdevices = 3;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog input subdevice */
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON | SDF_DIFF;
	/* WARNING: Number of inputs in differential mode is ignored */
	s->n_chan = thisboard->ai_se_chans;
	s->len_chanlist = thisboard->ai_se_chans;
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = thisboard->ranges;
	s->insn_read = cb_pcidas_ai_rinsn;
	s->do_cmd = cb_pcidas_ai_cmd;
	s->do_cmdtest = cb_pcidas_ai_cmdtest;
	s->cancel = cb_pcidas_cancel;

	/* analog output subdevice */
	s = dev->subdevices + 1;
	if(thisboard->ao_nchan)
	{
		// XXX todo: support analog output
		s->type = COMEDI_SUBD_UNUSED;
	}else
	{
		s->type = COMEDI_SUBD_UNUSED;
	}

	/* 8255 */
	s = dev->subdevices + 2;
	subdev_8255_init(dev, s, NULL,
		(void *)(devpriv->pacer_counter_dio + DIO_8255));

	/* Set bits to enable incoming mailbox interrupts on amcc s5933.
	 * They don't actually get sent here, but in cmd code. */
	devpriv->s5933_intcsr_bits = INBOX_BYTE(3) | INBOX_SELECT(3) | INBOX_FULL_INT;

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

	if(devpriv)
	{
		if(devpriv->s5933_config)
		{
			// disable and clear interrupts on amcc s5933
			outl(INBOX_INTR_STATUS, devpriv->s5933_config + INTCSR);
			rt_printk("detaching, incsr is 0x%x\n", inl(devpriv->s5933_config + INTCSR));
			release_region(devpriv->s5933_config, S5933_SIZE);
		}
		if(devpriv->control_status)
			release_region(devpriv->control_status, CONT_STAT_SIZE);
		if(devpriv->adc_fifo)
			release_region(devpriv->adc_fifo, ADC_FIFO_SIZE);
		if(devpriv->pacer_counter_dio)
			release_region(devpriv->pacer_counter_dio, PACER_SIZE);
		if(devpriv->ao_registers)
			release_region(devpriv->ao_registers, AO_SIZE);
	}
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);
	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices + 2);

	return 0;
}

/*
 * "instructions" read/write data in "one-shot" or "software-triggered"
 * mode.
 */
static int cb_pcidas_ai_rinsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int n,i;
	unsigned int bits;
	static const int timeout = 10000;

	// set mux limits and gain
	bits = BEGIN_SCAN(CR_CHAN(insn->chanspec)) |
		END_SCAN(CR_CHAN(insn->chanspec)) |
		GAIN_BITS(CR_RANGE(insn->chanspec));
	// set unipolar/bipolar
	if(CR_RANGE(insn->chanspec) & IS_UNIPOLAR)
		bits |= UNIP;
	// set singleended/differential
	if(CR_AREF(insn->chanspec) != AREF_DIFF)
		bits |= SE;
	outw_p(bits, devpriv->control_status + ADCMUX_CONT);

	/* wait for mux to settle */
	/* I suppose I made it with outw_p... */

	/* clear fifo */
	outw(0, devpriv->adc_fifo + ADCFIFOCLR);

	/* convert n samples */
	for (n = 0; n < insn->n; n++)
	{
		/* trigger conversion */
		outw(0, devpriv->adc_fifo + ADCDATA);

		/* wait for conversion to end */
		/* return -ETIMEDOUT if there is a timeout */
		for(i = 0; i < timeout; i++)
		{
			if (inw(devpriv->control_status + INT_ADCFIFO) & ADNE)
				break;
		}
		if(i == timeout)
			return -ETIMEDOUT;

		/* read data */
		data[n] = inw(devpriv->adc_fifo + ADCDATA);
	}

	/* return the number of samples read/written */
	return n;
}

static int cb_pcidas_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd)
{
	int err=0;
	int tmp;
	int i, gain, start_chan;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW | TRIG_TIMER | TRIG_EXT;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_NOW | TRIG_EXT;
	if(!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if(!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->start_src != TRIG_NOW &&
		cmd->start_src != TRIG_EXT)
		err++;
	if(cmd->scan_begin_src != TRIG_FOLLOW &&
		cmd->scan_begin_src != TRIG_TIMER &&
		cmd->scan_begin_src != TRIG_EXT)
		err++;
	if(cmd->convert_src != TRIG_TIMER &&
		cmd->convert_src != TRIG_EXT &&
		cmd->convert_src != TRIG_NOW)
		err++;
	if(cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE)
		err++;

	// make sure convert_src and scan_begin_src are compatible
	if(cmd->scan_begin_src == TRIG_FOLLOW &&
		cmd->convert_src == TRIG_NOW)
		err++;
	if(cmd->scan_begin_src != TRIG_FOLLOW &&
		cmd->convert_src != TRIG_NOW)
		err++;

	if(err) return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0)
	{
		cmd->start_arg=0;
		err++;
	}

	if (cmd->scan_begin_src == TRIG_TIMER)
	{
		if (cmd->scan_begin_arg < thisboard->ai_speed * cmd->chanlist_len)
		{
			cmd->scan_begin_arg = thisboard->ai_speed * cmd->chanlist_len;
			err++;
		}
	}
	if (cmd->convert_src == TRIG_TIMER)
	{
		if (cmd->convert_arg < thisboard->ai_speed)
		{
			cmd->convert_arg = thisboard->ai_speed;
			err++;
		}
	}

	if(cmd->scan_end_arg != cmd->chanlist_len)
	{
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src == TRIG_NONE)
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
		i8253_cascade_ns_to_timer_2div(TIMER_BASE,
			&(devpriv->divisor1), &(devpriv->divisor2),
			&(cmd->scan_begin_arg), cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->scan_begin_arg)
			err++;
	}
	if(cmd->convert_src == TRIG_TIMER)
	{
		tmp=cmd->convert_arg;
		i8253_cascade_ns_to_timer_2div(TIMER_BASE,
			&(devpriv->divisor1), &(devpriv->divisor2),
			&(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->convert_arg)
			err++;
	}

	if(err) return 4;

	// check channel/gain list against card's limitations
	if(cmd->chanlist)
	{
		gain = CR_RANGE(cmd->chanlist[0]);
		start_chan = CR_CHAN(cmd->chanlist[0]);
		for(i = 1; i < cmd->chanlist_len; i++)
		{
			if(CR_CHAN(cmd->chanlist[i]) != (start_chan + i) % s->n_chan)
			{
				comedi_error(dev, "entries in chanlist must be consecutive channels, counting upwards\n");
				err++;
			}
			if(CR_RANGE(cmd->chanlist[i]) != gain)
			{
				comedi_error(dev, "entries in chanlist must all have the same gain\n");
				err++;
			}
		}
	}

	if(err) return 5;

	return 0;
}

static int cb_pcidas_ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned int bits;

	// initialize before settings pacer source and count values
	outw(0, devpriv->control_status + TRIG_CONTSTAT);
	// clear fifo
	outw(0, devpriv->adc_fifo + ADCFIFOCLR);

	// set mux limits, gain and pacer source
	bits = BEGIN_SCAN(CR_CHAN(cmd->chanlist[0])) |
		END_SCAN(CR_CHAN(cmd->chanlist[cmd->chanlist_len - 1])) |
		GAIN_BITS(CR_RANGE(cmd->chanlist[0]));
	// set unipolar/bipolar
	if(CR_RANGE(cmd->chanlist[0]) & IS_UNIPOLAR)
		bits |= UNIP;
	// set singleended/differential
	if(CR_AREF(cmd->chanlist[0]) != AREF_DIFF)
		bits |= SE;
	// set pacer source
	if(cmd->convert_src == TRIG_EXT || cmd->scan_begin_src == TRIG_EXT)
		bits |= PACER_EXT_RISE;
	else
		bits |= PACER_INT;
	outw(bits, devpriv->control_status + ADCMUX_CONT);

#ifdef CB_PCIDAS_DEBUG
		rt_printk("comedi: sent 0x%x to adcmux control\n", bits);
#endif

	// load counters
	if(cmd->convert_src == TRIG_TIMER)
		cb_pcidas_load_counters(dev, &cmd->convert_arg, cmd->flags & TRIG_ROUND_MASK);
	else if(cmd->scan_begin_src == TRIG_TIMER)
		cb_pcidas_load_counters(dev, &cmd->scan_begin_arg, cmd->flags & TRIG_ROUND_MASK);

	// set number of conversions
	if(cmd->stop_src == TRIG_COUNT)
	{
		devpriv->count = cmd->chanlist_len * cmd->stop_arg;
	}

	// enable interrupts
	devpriv->adc_fifo_bits = INTE;
	if(cmd->flags & TRIG_WAKE_EOS)
	{
		if(cmd->convert_src == TRIG_NOW && cmd->chanlist_len > 1)
			devpriv->adc_fifo_bits |= INT_EOS;	// interrupt end of burst
		else
			devpriv->adc_fifo_bits |= INT_FNE;	// interrupt fifo not empty
	}else
	{
		devpriv->adc_fifo_bits |= INT_FHF;	//interrupt fifo half full
	}
#ifdef CB_PCIDAS_DEBUG
		rt_printk("comedi: adc_fifo_bits are 0x%x\n", devpriv->adc_fifo_bits);
#endif
	// enable (and clear) interrupts
	outw(devpriv->adc_fifo_bits | EOACL | INTCL | ADFLCL, devpriv->control_status + INT_ADCFIFO);
	// clear s5933 interrupt
	outl(devpriv->s5933_intcsr_bits | INBOX_INTR_STATUS, devpriv->s5933_config + INTCSR);


	// set start trigger and burst mode
	bits = 0;
	if(cmd->start_src == TRIG_NOW)
		bits |= SW_TRIGGER;
	else if(cmd->start_src == TRIG_EXT)
		bits |= EXT_TRIGGER | TGEN | XTRCL;
	else
	{
		comedi_error(dev, "bug!");
		return -1;
	}
	if(cmd->convert_src == TRIG_NOW && cmd->chanlist_len > 1)
		bits |= BURSTE;
	outw(bits, devpriv->control_status + TRIG_CONTSTAT);
#ifdef CB_PCIDAS_DEBUG
		rt_printk("comedi: sent 0x%x to trig control\n", bits);
#endif

	return 0;
}

static void cb_pcidas_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev = (comedi_device*) d;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async;
	int status;
	static const int half_fifo = 512;
	sampl_t data[half_fifo];
	int i;
	static const int timeout = 10000;

	if(dev->attached == 0)
	{
		comedi_error(dev, "premature interrupt");
		return;
	}

	async = s->async;
	async->events = 0;

	status = inw(devpriv->control_status + INT_ADCFIFO);
	if((status & (INT | EOAI | LADFUL)) == 0)
	{
#ifdef CB_PCIDAS_DEBUG
		comedi_error(dev, "spurious interrupt");
#endif
		// clear s5933 interrupt
		outl(devpriv->s5933_intcsr_bits | INBOX_INTR_STATUS, devpriv->s5933_config + INTCSR);
		return;
	}
	// if fifo half-full
	if(status & ADHFI)
	{
		insw(devpriv->adc_fifo, data, half_fifo);
		for(i = 0; i < half_fifo; i++)
		{
			comedi_buf_put(async, data[i]);
			if(async->cmd.stop_src == TRIG_COUNT)
			{
				if(--devpriv->count == 0)
				{		/* end of acquisition */
					cb_pcidas_cancel(dev, s);
					async->events |= COMEDI_CB_EOA;
					break;
				}
			}
		}
		async->events |= COMEDI_CB_BLOCK;
		// clear half-full interrupt latch
		outw(devpriv->adc_fifo_bits | INTCL, devpriv->control_status + INT_ADCFIFO);
	// else if fifo not empty
	}else if(status & (ADNEI | EOBI))
	{
		for(i = 0; i < timeout; i++)
		{
			// break if fifo is empty
			if((ADNE & inw(devpriv->control_status + INT_ADCFIFO)) == 0)
				break;
			data[0] = inw(devpriv->adc_fifo);
			comedi_buf_put(async, data[0]);
			if(async->cmd.stop_src == TRIG_COUNT &&
				--devpriv->count == 0)
			{		/* end of acquisition */
				cb_pcidas_cancel(dev, s);
				async->events |= COMEDI_CB_EOA;
				break;
			}
		}
		async->events |= COMEDI_CB_BLOCK;
		// clear not-empty interrupt latch
		outw(devpriv->adc_fifo_bits | INTCL, devpriv->control_status + INT_ADCFIFO);
	}else if(status & EOAI)
	{
		comedi_error(dev, "bug! encountered end of aquisition interrupt?");
		// clear EOA interrupt latch
		outw(devpriv->adc_fifo_bits | EOACL, devpriv->control_status + INT_ADCFIFO);
	}
	//check for fifo overflow
	if(status & LADFUL)
	{
		comedi_error(dev, "fifo overflow");
		// clear overflow interrupt latch
		outw(devpriv->adc_fifo_bits | ADFLCL, devpriv->control_status + INT_ADCFIFO);
		cb_pcidas_cancel(dev, s);
		async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
	}

	// clear interrupt on amcc s5933
	outl(devpriv->s5933_intcsr_bits | INBOX_INTR_STATUS, devpriv->s5933_config + INTCSR);

	comedi_event(dev, s, async->events);

	return;
}

static int cb_pcidas_cancel(comedi_device *dev, comedi_subdevice *s)
{
	// disable interrupts
	devpriv->adc_fifo_bits = 0;
	outw(devpriv->adc_fifo_bits, devpriv->control_status + INT_ADCFIFO);
	// disable start trigger source and burst mode
	outw(0, devpriv->control_status + TRIG_CONTSTAT);
	// software pacer source
	outw(0, devpriv->control_status + ADCMUX_CONT);


	return 0;
}

void cb_pcidas_load_counters(comedi_device *dev, unsigned int *ns, int rounding_flags)
{
	i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->divisor1),
		&(devpriv->divisor2), ns, rounding_flags & TRIG_ROUND_MASK);

	/* Write the values of ctr1 and ctr2 into counters 1 and 2 */
	i8254_load(devpriv->pacer_counter_dio + ADC8254, 1, devpriv->divisor1, 2);
	i8254_load(devpriv->pacer_counter_dio + ADC8254, 2, devpriv->divisor2, 2);
}


/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_cb_pcidas);

