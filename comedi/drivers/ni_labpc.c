/*
    ni_labpc.c driver for National Instruments Lab-PC series boards and compatibles
    Copyright (C) 2000 Frank Mori Hess <fmhess@uiuc.edu>

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
*/
/*
Driver: ni_labpc.o
Description: National Instruments Lab-PC (& compatibles)
Author: Frank Mori Hess <fmhess@users.sourceforge.net>
Devices: [National Instruments] Lab-PC+ (lab-pc+),
Status: In development, probably doesn't work yet.  Initially
  just working on Lab-PC+.  Will adapt for compatible boards later.  Not
  all input ranges and analog references will work, depending on how you
  have configured the jumpers on your board (see your owner's manual)

Configuration options:
  [0] - I/O port base address
  [1] - IRQ (optional, required for timed or externally triggered conversions)
  [2] - DMA channel (optional, required for timed or externally triggered conversions)
*/

/*
TODO:
	command support
	additional boards
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
#include "8253.h"
#include "8255.h"

#define LABPC_SIZE           32	// size of io region used by board
#define LABPC_TIMER_BASE            500	// 2 MHz master clock

/* Registers for the lab-pc+ */

//write-only registers
#define COMMAND1_REG	0x0
#define   ADC_SCAN_EN_BIT	// enables multi channel scans
#define   ADC_GAIN_BITS(x)	(((x) & 0x7) << 4)
#define   ADC_CHAN_BITS(x)	((x) & 0x7)
#define COMMAND2_REG	0x1
#define   PRETRIG_BIT	0x1	// enable pretriggering (used in conjunction with SWTRIG)
#define   HWTRIG_BIT	0x2	// enable paced conversions on external trigger
#define   SWTRIG_BIT	0x4	// enable paced conversions
#define   CASCADE_BIT	0x8	// use two cascaded counters for pacing
#define   DAC_PACED_BIT(channel)	(0x40 << ((channel) & 0x1))
#define COMMAND3_REG	0x2
#define ADC_CONVERT_REG	0x3
#define DAC_LSB_REG(channel)	(0x4 + 2 * ((channel) & 0x1))
#define DAC_MSB_REG(channel)	(0x5 + 2 * ((channel) & 0x1))
#define ADC_CLEAR_REG	0x8
#define DMATC_CLEAR_REG	0xa
#define COMMAND4_REG            0xf
#define INTERVAL_COUNT_REG	0x1e
#define INTERVAL_LOAD_REG	0x1f

// read-only registers
#define STATUS_REG	0x0
#define   DATA_AVAIL_BIT	0x1
#define   NOT_PCPLUS_BIT	0x80
#define ADC_FIFO_REG	0xa

#define DIO_BASE_REG	0x10
#define COUNTER_A_BASE_REG	0x14
#define COUNTER_B_BASE_REG	0x18

typedef struct labpc_board_struct{
	char *name;
	int ai_speed;	// maximum input speed in nanoseconds
}labpc_board;

//analog input ranges
static comedi_lrange range_labpc_ai = {
	16,
	{
		UNI_RANGE(10),
		UNI_RANGE(8),
		UNI_RANGE(5),
		UNI_RANGE(2),
		UNI_RANGE(1),
		UNI_RANGE(0.5),
		UNI_RANGE(0.2),
		UNI_RANGE(0.1),
		BIP_RANGE(5),
		BIP_RANGE(4),
		BIP_RANGE(2.5),
		BIP_RANGE(1),
		BIP_RANGE(0.5),
		BIP_RANGE(0.25),
		BIP_RANGE(0.1),
		BIP_RANGE(0.05),
	}
};

//analog output ranges
static comedi_lrange range_labpc_ao = {
	2,
	{
		UNI_RANGE(10),
		BIP_RANGE(5),
	}
};

// enum must match labpc_boards array
// XXX i can't think of why i need this
// static enum labpc_board_index {lab_pc_plus};

static labpc_board labpc_boards[] =
{
	{
		name:	"lab-pc+",
		ai_speed:	12000,
	},
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((labpc_board *)dev->board_ptr)

typedef struct{
	volatile unsigned int count;  /* number of data points left to be taken */
	unsigned int ao_value[2];	// software copy of analog output values
	// software copys of bits written to command registers
	unsigned int command1_bits;
	unsigned int command2_bits;
	unsigned int command3_bits;
	unsigned int command4_bits;
	unsigned int divisor1;	/* value to load into board's counter 1 for timed conversions */
	unsigned int divisor2; 	/* value to load into board's counter 2 for timed conversions */
}labpc_private;

#define devpriv ((labpc_private *)dev->private)

static int labpc_attach(comedi_device *dev,comedi_devconfig *it);
static int labpc_detach(comedi_device *dev);
static int labpc_cancel(comedi_device *dev, comedi_subdevice *s);
static void labpc_interrupt(int irq, void *d, struct pt_regs *regs);
static int labpc_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd);
static int labpc_ai_cmd(comedi_device *dev, comedi_subdevice *s);
static int labpc_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int labpc_ao_winsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int labpc_ao_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);

static comedi_driver driver_labpc={
	driver_name:	"ni_labpc",
	module:		THIS_MODULE,
	attach:		labpc_attach,
	detach:		labpc_detach,
	num_names:	sizeof(labpc_boards) / sizeof(labpc_board),
	board_name:	(char **)labpc_boards,
	offset:		sizeof(labpc_board),
};

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_labpc);

/* interrupt service routine */
static void labpc_interrupt(int irq, void *d, struct pt_regs *regs)
{
#if 0
	short i;		/* loop index */
	sampl_t dataPoint = 0;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->read_subdev;	/* analog input subdevice */
	comedi_async *async;
	int status;
	unsigned long irq_flags;
	static const int max_loops = 128;	// half-fifo size for cio-das802/16
	// flags
	int fifo_empty = 0;
	int fifo_overflow = 0;

	status = inb(dev->iobase + DAS800_STATUS);
	/* if interrupt was not generated by board or driver not attached, quit */
	if(!(status & IRQ) || !(dev->attached))
	{
		return;
	}

	/* wait until here to initialize async, since we will get null dereference
	 * if interrupt occurs before driver is fully attached!
	 */
	async = s->async;

	// if hardware conversions are not enabled, then quit
	comedi_spin_lock_irqsave(&dev->spinlock, irq_flags);
	outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select base address + 7 to be STATUS2 register */
	status = inb(dev->iobase + DAS800_STATUS2) & STATUS2_HCEN;
	/* don't release spinlock yet since we want to make sure noone else disables hardware conversions */
	if(status == 0)
	{
		comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);
		return;
	}

	/* loop while card's fifo is not empty (and limit to half fifo for cio-das802/16) */
	for(i = 0; i < max_loops; i++)
	{
		/* read 16 bits from dev->iobase and dev->iobase + 1 */
		dataPoint = inb(dev->iobase + DAS800_LSB);
		dataPoint += inb(dev->iobase + DAS800_MSB) << 8;
		if(thisboard->resolution == 12)
		{
			fifo_empty = dataPoint & FIFO_EMPTY;
			fifo_overflow = dataPoint & FIFO_OVF;
			if(fifo_overflow) break;
		}else
		{
			fifo_empty = 0;	// cio-das802/16 has no fifo empty status bit
		}
		if(fifo_empty)
		{
			break;
		}
		/* strip off extraneous bits for 12 bit cards*/
		if(thisboard->resolution == 12)
			dataPoint = (dataPoint >> 4) & 0xfff;
		/* if there are more data points to collect */
		if(devpriv->count > 0 || devpriv->forever == 1)
		{
			/* write data point to buffer */
			comedi_buf_put(async, dataPoint);
			if(devpriv->count > 0) devpriv->count--;
		}
	}
	async->events |= COMEDI_CB_BLOCK;
	/* check for fifo overflow */
	if(thisboard->resolution == 12)
	{
		fifo_overflow = dataPoint & FIFO_OVF;
	// else cio-das802/16
	}else
	{
		fifo_overflow = inb(dev->iobase + DAS800_GAIN) & CIO_FFOV;
	}
	if(fifo_overflow)
	{
		comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);
		comedi_error(dev, "DAS800 FIFO overflow");
		das800_cancel(dev, dev->subdevices + 0);
		async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
		comedi_event(dev, s, async->events);
		async->events = 0;
		return;
	}
	if(devpriv->count > 0 || devpriv->forever == 1)
	{
		/* Re-enable card's interrupt.
		 * We already have spinlock, so indirect addressing is safe */
		outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be control register 1 */
		outb(CONTROL1_INTE | devpriv->do_bits, dev->iobase + DAS800_CONTROL1);
	/* otherwise, stop taking data */
	} else
	{
		disable_das800(dev);		/* diable hardware triggered conversions */
		async->events |= COMEDI_CB_EOA;
	}
	comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);
	comedi_event(dev, s, async->events);
	async->events = 0;
	return;
#endif
}

static int labpc_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int iobase = it->options[0];
	int irq = it->options[1];
	int dma_chan = it->options[2];
	int status;
	int lsb, msb;
	int i;

	printk("comedi%d: ni_labpc: io 0x%x", dev->minor, iobase);
	if(irq)
	{
		printk(", irq %i", irq);
	}
	if(dma_chan)
	{
		printk(", dma %i", dma_chan);
	}
	printk("\n");

	/* allocate and initialize dev->private */
	if(alloc_private(dev, sizeof(labpc_private)) < 0)
		return -ENOMEM;

	// XXX iobase parameter wont be needed for pcmcia cards
	if(iobase == 0)
	{
		printk("io base address required for lab-pc+\n");
		return -EINVAL;
	}

	/* check if io addresses are available */
	if(check_region(iobase, LABPC_SIZE) < 0)
	{
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase, LABPC_SIZE, driver_labpc.driver_name);
	dev->iobase = iobase;

	// check to see if it's a lab-pc or lab-pc+
	// XXX check against board definitions
	status = inb(dev->iobase + STATUS_REG);
	if(status & NOT_PCPLUS_BIT)
	{
		printk(" status bit indicates lab pc board\n");
	}else
	{
		printk(" status bit indicates lab pc+ board\n");
	}

	/* grab our IRQ */
	// XXX boards other than pc+ may have more flexible irq possibilities
	if(irq == 1 || irq == 2 || irq == 8 || irq > 9 || irq < 0)
	{
		printk("irq out of range\n");
		return -EINVAL;
	}
	if(irq)
	{
		if(comedi_request_irq( irq, labpc_interrupt, 0, driver_labpc.driver_name, dev))
		{
			printk( "unable to allocate irq %d\n", irq);
			return -EINVAL;
		}
	}
	dev->irq = irq;

	dev->board_name = thisboard->name;

	dev->n_subdevices = 3;
	if(alloc_subdevices(dev) < 0)
		return -ENOMEM;

	/* analog input subdevice */
	s = dev->subdevices + 0;
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
//XXX ground/common/differential abilities depend on jumpers
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON | SDF_DIFF;
	s->n_chan = 8;
	s->len_chanlist = 8;
	s->maxdata = (1 << 12) - 1;	// 12 bit resolution
	s->range_table = &range_labpc_ai;
	s->do_cmd = labpc_ai_cmd;
	s->do_cmdtest = labpc_ai_cmdtest;
	s->insn_read = labpc_ai_rinsn;
	s->cancel = labpc_cancel;

	/* analog output */
	s = dev->subdevices + 1;
/* XXX could provide command support, except it doesn't seem to have a hardware
 * buffer for analog output so speed would be very limited unless using RT interrupt */
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_GROUND;
	s->n_chan = 2;
	s->maxdata = (1 << 12) - 1;	// 12 bit resolution
	s->range_table = &range_labpc_ao;
	s->insn_read = labpc_ao_rinsn;
	s->insn_write = labpc_ao_winsn;

	/* initialize analog outputs to a known value */
	for(i = 0; i < s->n_chan; i++)
	{
		devpriv->ao_value[i] = s->maxdata / 2;	// XXX should init to 0 for unipolar
		lsb = devpriv->ao_value[i] & 0xff;
		msb = (devpriv->ao_value[i] >> 8) & 0xff;
		outb(lsb, dev->iobase + DAC_LSB_REG(i));
		outb(msb, dev->iobase + DAC_MSB_REG(i));
	}

	/* 8255 dio */
	s = dev->subdevices + 2;
	subdev_8255_init(dev, s, NULL, (void*)(dev->iobase + DIO_BASE_REG));

	return 0;
};

static int labpc_detach(comedi_device *dev)
{
	printk("comedi%d: ni_labpc: remove\n", dev->minor);

	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices + 2);

	/* only free stuff if it has been allocated by _attach */
	if(dev->iobase)
		release_region(dev->iobase, LABPC_SIZE);
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);

	return 0;
};

static int labpc_cancel(comedi_device *dev, comedi_subdevice *s)
{
	// XXX
	return 0;
}

static int labpc_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
#if 0
	int err = 0;
	int tmp;
	int gain, startChan;
	int i;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src || tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src) err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if(!cmd->convert_src || tmp != cmd->convert_src) err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src) err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src) err++;

	if(err) return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->start_src != TRIG_NOW &&
		cmd->start_src != TRIG_EXT) err++;
	if(cmd->convert_src != TRIG_TIMER &&
	   cmd->convert_src != TRIG_EXT) err++;
	if(cmd->stop_src != TRIG_COUNT &&
		cmd->stop_src != TRIG_NONE) err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg != 0)
	{
		cmd->start_arg = 0;
		err++;
	}
	if(cmd->convert_src == TRIG_TIMER)
	{
		if(cmd->convert_arg < thisboard->ai_speed)
		{
			cmd->convert_arg = thisboard->ai_speed;
			err++;
		}
	}
	if(!cmd->chanlist_len)
	{
		cmd->chanlist_len = 1;
		err++;
	}
	if(cmd->scan_end_arg != cmd->chanlist_len)
	{
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src == TRIG_COUNT)
	{
		if(!cmd->stop_arg)
		{
			cmd->stop_arg = 1;
			err++;
		}
	} else
	{ /* TRIG_NONE */
		if(cmd->stop_arg != 0)
		{
			cmd->stop_arg = 0;
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	if(cmd->convert_src == TRIG_TIMER)
	{
		tmp = cmd->convert_arg;
		/* calculate counter values that give desired timing */
		i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->divisor1), &(devpriv->divisor2), &(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->convert_arg) err++;
	}

	if(err)return 4;

	// check channel/gain list against card's limitations
	if(cmd->chanlist)
	{
		gain = CR_RANGE(cmd->chanlist[0]);
		startChan = CR_CHAN(cmd->chanlist[0]);
		for(i = 1; i < cmd->chanlist_len; i++)
		{
			if(CR_CHAN(cmd->chanlist[i]) != (startChan + i) % N_CHAN_AI)
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

	if(err)return 5;

#endif
	return 0;
}

static int labpc_ai_cmd(comedi_device *dev, comedi_subdevice *s)
{
#if 0
	int startChan, endChan, scan, gain;
	int conv_bits;
	unsigned long irq_flags;
	comedi_async *async = s->async;

	if(!dev->irq)
	{
		comedi_error(dev, "no irq assigned for das-800, cannot do hardware conversions");
		return -1;
	}

	disable_das800(dev);

	/* set channel scan limits */
	startChan = CR_CHAN(async->cmd.chanlist[0]);
	endChan = (startChan + async->cmd.chanlist_len - 1) % 8;
	scan = (endChan << 3) | startChan;

	comedi_spin_lock_irqsave(&dev->spinlock, irq_flags);
	outb(SCAN_LIMITS, dev->iobase + DAS800_GAIN);	/* select base address + 2 to be scan limits register */
	outb(scan, dev->iobase + DAS800_SCAN_LIMITS); /* set scan limits */
	comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);

	/* set gain */
	gain = CR_RANGE(async->cmd.chanlist[0]);
	if( thisboard->resolution == 12 && gain > 0)
		gain += 0x7;
	gain &= 0xf;
	outb(gain, dev->iobase + DAS800_GAIN);

	switch(async->cmd.stop_src)
	{
		case TRIG_COUNT:
			devpriv->count = async->cmd.stop_arg * async->cmd.chanlist_len;
			devpriv->forever = 0;
			break;
		case TRIG_NONE:
			devpriv->forever = 1;
			devpriv->count = 0;
			break;
		default :
			break;
	}

	/* enable auto channel scan, send interrupts on end of conversion
	 * and set clock source to internal or external
	 */
	conv_bits = 0;
	conv_bits |= EACS | IEOC;
	if(async->cmd.start_src == TRIG_EXT)
		conv_bits |= DTEN;
	switch(async->cmd.convert_src)
	{
		case TRIG_TIMER:
			conv_bits |= CASC | ITE;
			/* set conversion frequency */
			i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->divisor1), &(devpriv->divisor2), &(async->cmd.convert_arg), async->cmd.flags & TRIG_ROUND_MASK);
			if(das800_set_frequency(dev) < 0)
			{
				comedi_error(dev, "Error setting up counters");
				return -1;
			}
			break;
		case TRIG_EXT:
			break;
		default:
			break;
	}

	comedi_spin_lock_irqsave(&dev->spinlock, irq_flags);
	outb(CONV_CONTROL, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be conversion control register */
	outb(conv_bits, dev->iobase + DAS800_CONV_CONTROL);
	comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);
	async->events = 0;
	enable_das800(dev);
#endif
	return 0;
}

static int labpc_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
 	int i, n;
	int chan;
	int lsb, msb;
	int timeout = 1000;

	/* set gain and channel */
	devpriv->command1_bits = 0;
	devpriv->command1_bits |= ADC_GAIN_BITS(CR_RANGE(insn->chanspec));
	chan = CR_CHAN(insn->chanspec);
	// munge channel bits for differential mode
	if(CR_AREF(insn->chanspec) == AREF_DIFF)
		chan *= 2;
	devpriv->command1_bits |= ADC_CHAN_BITS(chan);
	outb(devpriv->command1_bits, dev->iobase + COMMAND1_REG);

	// disable timed conversions
	devpriv->command2_bits &= ~SWTRIG_BIT & ~HWTRIG_BIT & ~PRETRIG_BIT;
	outb(devpriv->command2_bits, dev->iobase + COMMAND3_REG);

	// disable interrupt generation and dma
	devpriv->command3_bits = 0;
	outb(devpriv->command3_bits, dev->iobase + COMMAND3_REG);

	// XXX init counter a0 to high state

	// clear adc fifo
	outb(0x1, dev->iobase + ADC_CLEAR_REG);
	inb(dev->iobase + ADC_FIFO_REG);
	inb(dev->iobase + ADC_FIFO_REG);

	// give it a little settling time
	udelay(5);

	for(n = 0; n < insn->n; n++)
	{
		/* trigger conversion */
		outb_p(0x1, dev->iobase + ADC_CONVERT_REG);

		for(i = 0; i < timeout; i++)
		{
			if(inb(dev->iobase + STATUS_REG) & DATA_AVAIL_BIT)
				break;
		}
		if(i == timeout)
		{
			comedi_error(dev, "timeout");
			return -ETIME;
		}
		lsb = inb(dev->iobase + ADC_FIFO_REG);
		msb = inb(dev->iobase + ADC_FIFO_REG);
		data[n] = (msb << 8) | lsb;
	}

	return n;
}

// analog output insn for pcidas-1602 series
static int labpc_ao_winsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int channel;
	int lsb, msb;

	channel = CR_CHAN(insn->chanspec);

	// turn off pacing of analog output channel
	// XXX spinlock access (race with analog input)
	devpriv->command2_bits &= DAC_PACED_BIT(channel);
	outb(devpriv->command2_bits, dev->iobase + COMMAND2_REG);

	// send data
	lsb = data[0] & 0xff;
	msb = (data[0] >> 8 ) & 0xff;
	outb(lsb, dev->iobase + DAC_LSB_REG(channel));
	outb(msb, dev->iobase + DAC_MSB_REG(channel));

	// remember value for readback
	devpriv->ao_value[channel] = data[0];

	return 1;
}

// analog output readback insn
static int labpc_ao_rinsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	data[0] = devpriv->ao_value[CR_CHAN(insn->chanspec)];

	return 1;
}