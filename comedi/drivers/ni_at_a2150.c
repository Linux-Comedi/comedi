/*
    ni_at_a2150.c driver for National Instruments AT-A2150 boards
    Copyright (C) 2000 Frank Mori Hess <fmhess@uiuc.edu>

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

************************************************************************

Yet another driver for obsolete hardware brought to you by Frank Hess.

This driver supports the boards:

AT-A2150C
AT-A2150S

The only difference is their master clock frequencies.

Options:
	[0] - base io address
	[1] - irq
	[2] - dma channel

References (from ftp://ftp.natinst.com/support/manuals):

	   320360.pdf  AT-A2150 User Manual
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
#include <asm/dma.h>
#include <linux/comedidev.h>
#include "8253.h"

#define A2150_SIZE           28
#define A2150_DMA_BUFFER_SIZE	0xff00	// size in bytes of dma buffer

#define A2150_DEBUG	// enable debugging code

/* Registers and bits */
#define CONFIG_REG		0x0
#define   CHANNEL_BITS(x)		((x) & 0x7)
#define   CHANNEL_MASK		0x7
#define   CLOCK_SELECT_BITS(x)		(((x) & 0x3) << 3)
#define   CLOCK_DIVISOR_BITS(x)		(((x) & 0x3) << 5)
#define   CLOCK_MASK		(0xf << 3)
#define   AC0_BIT		0x200	// ac couple channels 0,1
#define   AC1_BIT		0x400	// ac couple channels 2,3
#define   APD_BIT		0x800	// analog power down
#define   DPD_BIT		0x1000	// digital power down
#define TRIGGER_REG		0x2	// trigger config register
#define   DELAY_TRIGGER_BITS		0x3
#define FIFO_START_REG		0x6	// software start aquistion trigger
#define FIFO_RESET_REG		0x8	// clears fifo + fifo flags
#define DMA_TC_CLEAR_REG		0xe	// clear dma terminal count interrupt
#define STATUS_REG		0x12	// read only
#define   FNE_BIT		0x1	// fifo not empty
#define   OVFL_BIT		0x8	// fifo overflow
#define   EDAQ_BIT		0x10	// end of aquisition interrupt
#define   INTR_BIT		0x40	// interrupt has occured
#define   DMA_TC_BIT		0x80	// dma terminal count interrupt has occured
#define   ID_BITS(x)	(((x) >> 8) & 0x3)
#define IRQ_DMA_CNTRL_REG		0x12	// write only
#define   DMA_CHAN_BITS(x)		((x) & 0x7)	// sets dma channel
#define   DMA_EN_BIT		0x8	// enables dma
#define   IRQ_LVL_BITS(x)		(((x) & 0xf) << 4)	// sets irq level
#define   FIFO_INTR_EN_BIT		0x100	// enable fifo interrupts
#define   FIFO_INTR_FHF_BIT		0x200	// interrupt fifo half full
#define   DMA_INTR_EN_BIT 		0x800	// enable interrupt on dma terminal count
#define   DMA_DEM_EN_BIT	0x1000	// enables demand mode dma
#define I8253_BASE_REG		0x14
#define I8253_MODE_REG		0x17

typedef struct a2150_board_struct{
	char *name;
	int clock[4];	// master clock periods, in nanoseconds
	int num_clocks;	// number of available master clock speeds
	int ai_speed;	// maximum conversion rate in nanoseconds
}a2150_board;

//analog input range
static comedi_lrange range_a2150 = {
	1,
	{
		RANGE( -2.828, 2.828 ),
	}
};

// enum must match board indices
enum{a2150_c, a2150_s};
a2150_board a2150_boards[] =
{
	{
		name:	"at-a2150c",
		clock:	{31250,  22676, 20833, 19531},
		num_clocks:	4,
		ai_speed:	19531,
	},
	{
		name:	"at-a2150s",
		clock:	{62500, 50000, 41667, 0},
		num_clocks:	3,
		ai_speed:	41667,
	},
};
/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((a2150_board *)dev->board_ptr)

typedef struct{
	volatile unsigned int count;  /* number of data points left to be taken */
	unsigned int dma;	// dma channel
	s16 *dma_buffer;	// dma buffer
	unsigned int dma_transfer_size;	// size in bytes of dma transfers
	int irq_dma_bits;	// irq/dma register bits
	int config_bits;	// config register bits
}a2150_private;

#define devpriv ((a2150_private *)dev->private)

static int a2150_attach(comedi_device *dev,comedi_devconfig *it);
static int a2150_detach(comedi_device *dev);
static int a2150_cancel(comedi_device *dev, comedi_subdevice *s);

comedi_driver driver_a2150={
	driver_name:	"ni_at_a2150",
	module:		THIS_MODULE,
	attach:		a2150_attach,
	detach:		a2150_detach,
};

static void a2150_interrupt(int irq, void *d, struct pt_regs *regs);
static int a2150_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd);
static int a2150_ai_cmd(comedi_device *dev, comedi_subdevice *s);
static int a2150_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int a2150_get_timing(comedi_device *dev, unsigned int *period, int flags);
static int a2150_probe(comedi_device *dev);
static int a2150_set_chanlist(comedi_device *dev, unsigned int start_channel, unsigned int num_channels);
/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_a2150);

#ifdef A2150_DEBUG

void dump_regs(comedi_device *dev)
{
	rt_printk("config bits 0x%x\n", devpriv->config_bits);
	rt_printk("irq dma bits 0x%x\n", devpriv->irq_dma_bits);
	rt_printk("status bits 0x%x\n", inw(dev->iobase + STATUS_REG));
}

#endif

/* interrupt service routine */
static void a2150_interrupt(int irq, void *d, struct pt_regs *regs)
{
	int i;
	int status;
	unsigned long flags;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async;
	comedi_cmd *cmd;
	unsigned int max_points, num_points, residue, leftover;
	sampl_t dpnt;
	static const int sample_size = sizeof(devpriv->dma_buffer[0]);

	if(dev->attached == 0)
	{
		comedi_error(dev, "premature interrupt");
		return;
	}
	// initialize async here to make sure s is not NULL
	async = s->async;
	cmd = &async->cmd;

	status = inw(dev->iobase + STATUS_REG);

	if((status & INTR_BIT ) == 0)
	{
		comedi_error(dev, "spurious interrupt");
		return;
	}

	if(status & OVFL_BIT)
	{
		comedi_error(dev, "fifo overflow");
		a2150_cancel(dev, s);
		async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
	}

	if((status & DMA_TC_BIT) == 0)
	{
		comedi_error(dev, "caught non-dma interrupt?  Aborting.");
		a2150_cancel(dev, s);
		async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
		comedi_event(dev, s, async->events);
		async->events = 0;
		return;
	}

	flags = claim_dma_lock();
	disable_dma(devpriv->dma);
	/* clear flip-flop to make sure 2-byte registers for
	 * count and address get set correctly */
	clear_dma_ff(devpriv->dma);

	// figure out how many points to read
	max_points = devpriv->dma_transfer_size  / sample_size;
	/* residue is the number of points left to be done on the dma
	 * transfer.  It should always be zero at this point unless
	 * the stop_src is set to external triggering.
	 */
	residue = get_dma_residue(devpriv->dma) / sample_size;
	num_points = max_points - residue;
	if(devpriv->count < num_points &&
		cmd->stop_src == TRIG_COUNT)
		num_points = devpriv->count;

	// figure out how many points will be stored next time
	leftover = 0;
	if(cmd->stop_src == TRIG_NONE)
	{
		leftover = devpriv->dma_transfer_size / sample_size;
	}else if(devpriv->count > max_points)
	{
		leftover = devpriv->count - max_points;
		if(leftover > max_points)
			leftover = max_points;
	}
	/* there should only be a residue if collection was stopped by having
	 * the stop_src set to an external trigger, in which case there
	 * will be no more data
	 */
	if(residue)
		leftover = 0;

	for(i = 0; i < num_points; i++)
	{
		/* write data point to comedi buffer */
		dpnt = devpriv->dma_buffer[i];
		// convert from 2's complement to unsigned coding
		dpnt += 0x8000;
		comedi_buf_put(async, dpnt);
		if(devpriv->count > 0) devpriv->count--;
	}
	// re-enable  dma
	set_dma_addr(devpriv->dma, (unsigned int) devpriv->dma_buffer);
	set_dma_count(devpriv->dma, leftover * sample_size);
	enable_dma(devpriv->dma);
	release_dma_lock(flags);

	async->events |= COMEDI_CB_BLOCK;

	if(devpriv->count == 0)
	{	/* end of acquisition */
		a2150_cancel(dev, s);
		async->events |= COMEDI_CB_EOA;
	}

	comedi_event(dev, s, async->events);
	async->events = 0;

	/* clear interrupt */
	outb(0x00, dev->iobase + DMA_TC_CLEAR_REG);

	return;
}

// probes board type, returns offset
static int a2150_probe(comedi_device *dev)
{
	int status = inw(dev->iobase + STATUS_REG);
	return ID_BITS(status);
}

static int a2150_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int iobase = it->options[0];
	int irq = it->options[1];
	int dma = it->options[2];

	printk("comedi%d: %s: io 0x%x", dev->minor, driver_a2150.driver_name, iobase);
	if(irq)
	{
		printk(", irq %i", irq);
	}else
	{
		printk(", no irq");
	}
	if(dma)
	{
		printk(", dma %i", dma);
	}else
	{
		printk(", no dma");
	}
	printk("\n");

	/* allocate and initialize dev->private */
	if(alloc_private(dev, sizeof(a2150_private)) < 0)
		return -ENOMEM;

	if(iobase == 0)
	{
		printk(" io base address required\n");
		return -EINVAL;
	}

	/* check if io addresses are available */
	if(check_region(iobase, A2150_SIZE) < 0)
	{
		printk(" I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase, A2150_SIZE, driver_a2150.driver_name);
	dev->iobase = iobase;

	/* grab our IRQ */
	if(irq)
	{
		// check that irq is supported
		if(irq < 3 || irq == 8 || irq == 13 || irq > 15)
		{
			printk(" invalid irq line\n");
			return -EINVAL;
		}
		if(comedi_request_irq( irq, a2150_interrupt, 0, driver_a2150.driver_name, dev ))
		{
			printk( "unable to allocate irq %d\n", irq);
			return -EINVAL;
		}
		devpriv->irq_dma_bits |= IRQ_LVL_BITS(irq);
		dev->irq = irq;
	}

	// initialize dma
	if(dma)
	{
		if(dma < 0 || dma == 4 || dma > 7)
		{
			printk(" invalid dma channel %i\n", dma);
			return -EINVAL;
		}
		if(request_dma(dma, driver_a2150.driver_name))
		{
			printk(" failed to allocate dma channel %i\n", dma);
			return -EINVAL;
		}
		devpriv->dma_buffer = kmalloc(A2150_DMA_BUFFER_SIZE, GFP_KERNEL | GFP_DMA);
		if(devpriv->dma_buffer == NULL)
			return -ENOMEM;

		disable_dma(dma);
		set_dma_mode(dma, DMA_MODE_READ);

		devpriv->dma = dma;
		devpriv->irq_dma_bits |= DMA_DEM_EN_BIT | DMA_CHAN_BITS(dma);
	}

	dev->board_ptr = a2150_boards + a2150_probe(dev);
	dev->board_name = thisboard->name;

	dev->n_subdevices = 1;
	if(alloc_subdevices(dev) < 0)
		return -ENOMEM;

	/* analog input subdevice */
	s = dev->subdevices + 0;
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 4;
	s->len_chanlist = 4;
	s->maxdata = 0xffff;
	s->range_table = &range_a2150;
	s->do_cmd = a2150_ai_cmd;
	s->do_cmdtest = a2150_ai_cmdtest;
//	s->insn_read = a2150_ai_rinsn;	XXX
	s->cancel = a2150_cancel;

	// set card's irq and dma levels
	outw(devpriv->irq_dma_bits, dev->iobase + IRQ_DMA_CNTRL_REG);
	// initialize configuration register
	devpriv->config_bits = 0;
	outw(devpriv->config_bits, dev->iobase + CONFIG_REG);

	return 0;
};

static int a2150_detach(comedi_device *dev)
{
	printk("comedi%d: %s: remove\n", dev->minor, driver_a2150.driver_name);

	// put board in power-down mode
	outw(APD_BIT | DPD_BIT, dev->iobase + CONFIG_REG);

	/* only free stuff if it has been allocated by _attach */
	if(dev->iobase)
		release_region(dev->iobase, A2150_SIZE);
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);
	if(devpriv)
	{
		if(devpriv->dma)
			free_dma(devpriv->dma);
		if(devpriv->dma_buffer)
			kfree(devpriv->dma_buffer);
	}

	return 0;
};

static int a2150_cancel(comedi_device *dev, comedi_subdevice *s)
{
	// disable dma on card
	devpriv->irq_dma_bits &= ~DMA_INTR_EN_BIT & ~DMA_EN_BIT;
	outw(devpriv->irq_dma_bits, dev->iobase + IRQ_DMA_CNTRL_REG);

	// disable computer's dma
	disable_dma(devpriv->dma);

	// clear fifo and reset triggering circuitry
	outw(0, dev->iobase + FIFO_RESET_REG);

	return 0;
}

static int a2150_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err = 0;
	int tmp;
	int startChan;
	int i;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW; // | TRIG_EXT;
	if(!cmd->start_src || tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src) err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_NOW;
	if(!cmd->convert_src || tmp != cmd->convert_src) err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src) err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT;// | TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src) err++;

	if(err) return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->start_src != TRIG_NOW &&
		cmd->start_src != TRIG_EXT) err++;
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

	// check channel/gain list against card's limitations
	if(cmd->chanlist)
	{
		startChan = CR_CHAN(cmd->chanlist[0]);
		for(i = 1; i < cmd->chanlist_len; i++)
		{
			if(CR_CHAN(cmd->chanlist[i]) != (startChan + i))
			{
				comedi_error(dev, "entries in chanlist must be consecutive channels, counting upwards\n");
				err++;
			}
		}
		if(cmd->chanlist_len == 2 && CR_CHAN(cmd->chanlist[0]) == 1)
		{
			comedi_error(dev, "length 2 chanlist must be channels 0,1 or channels 2,3");
			err++;
		}
		if(cmd->chanlist_len == 3)
		{
			comedi_error(dev, "chanlist must have 1,2 or 4 channels");
			err++;
		}
		if(CR_AREF(cmd->chanlist[0]) != CR_AREF(cmd->chanlist[1]) ||
			CR_AREF(cmd->chanlist[2]) != CR_AREF(cmd->chanlist[3]))
		{
			comedi_error(dev, "channels 0/1 and 2/3 must have the same analog reference");
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	if(cmd->scan_begin_src == TRIG_TIMER)
	{
		tmp = cmd->scan_begin_arg;
		a2150_get_timing(dev, &cmd->scan_begin_arg, cmd->flags);
		if(tmp != cmd->scan_begin_arg) err++;
	}

	if(err)return 4;

	return 0;
}

static int a2150_ai_cmd(comedi_device *dev, comedi_subdevice *s)
{
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned long lock_flags;

	if(!dev->irq || !devpriv->dma)
	{
		comedi_error(dev, " irq and dma required, cannot do hardware conversions");
		return -1;
	}
	if(cmd->flags & TRIG_RT)
	{
		comedi_error(dev, " dma incompatible with hard real-time interrupt (TRIG_RT), aborting");
		return -1;
	}

	// clear fifo and reset triggering circuitry
	outw(0, dev->iobase + FIFO_RESET_REG);

	/* setup chanlist */
	if(a2150_set_chanlist(dev, CR_CHAN(cmd->chanlist[0]), cmd->chanlist_len) < 0)
		return -1;

	// setup ac/dc coupling
	if(CR_AREF(cmd->chanlist[0]) == AREF_OTHER)
		devpriv->config_bits |= AC0_BIT;
	else
		devpriv->config_bits &= ~AC0_BIT;
	if(CR_AREF(cmd->chanlist[2]) == AREF_OTHER)
		devpriv->config_bits |= AC1_BIT;
	else
		devpriv->config_bits &= ~AC1_BIT;

	// setup timing
	a2150_get_timing(dev, &cmd->scan_begin_arg, cmd->flags);

	// send timing, channel, config bits
	outw(devpriv->config_bits, dev->iobase + CONFIG_REG);

	// initialize number of samples remaining
	devpriv->count = cmd->stop_arg * cmd->chanlist_len;

	// enable computer's dma
	lock_flags = claim_dma_lock();
	disable_dma(devpriv->dma);
	/* clear flip-flop to make sure 2-byte registers for
	 * count and address get set correctly */
	clear_dma_ff(devpriv->dma);
	set_dma_addr(devpriv->dma, (unsigned int) devpriv->dma_buffer);
	// set size of transfer to fill in 1/3 second
	devpriv->dma_transfer_size = sizeof(devpriv->dma_buffer[0]) * cmd->chanlist_len *
		(1000000000.0 / 3.0 ) / cmd->scan_begin_arg;
	if(devpriv->dma_transfer_size > A2150_DMA_BUFFER_SIZE)
		devpriv->dma_transfer_size = A2150_DMA_BUFFER_SIZE;
	if(devpriv->dma_transfer_size < sizeof(devpriv->dma_buffer[0]))
		devpriv->dma_transfer_size = sizeof(devpriv->dma_buffer[0]);
	devpriv->dma_transfer_size -= devpriv->dma_transfer_size % sizeof(devpriv->dma_buffer[0]);
	set_dma_count(devpriv->dma, devpriv->dma_transfer_size);
	enable_dma(devpriv->dma);
	release_dma_lock(lock_flags);

	// enable dma on card
	devpriv->irq_dma_bits |= DMA_INTR_EN_BIT | DMA_EN_BIT;
	outw(devpriv->irq_dma_bits, dev->iobase + IRQ_DMA_CNTRL_REG);

	// manual says you need to do this for software counting of completed conversions
	outw(0x30, dev->iobase + I8253_MODE_REG);

	// need to wait 72 sampling periods after changing timing
	i8254_load(dev->iobase + I8253_BASE_REG, 2, 72, 0);
	// set trigger source to delay trigger
	outw(DELAY_TRIGGER_BITS, dev->iobase + TRIGGER_REG);

	async->events = 0;

	// start aquisition
	outw(0, dev->iobase + FIFO_START_REG);

#ifdef A2150_DEBUG
	dump_regs(dev);
#endif

	return 0;
}

static int a2150_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
// XXX
	return 0;
}

/* sets bits in devpriv->clock_bits to nearest approximation of requested period,
 * adjusts requested period to actual timing. */
static int a2150_get_timing(comedi_device *dev, unsigned int *period, int flags)
{
	int lub, glb, temp;
	int lub_divisor_shift, lub_index, glb_divisor_shift, glb_index;
	int i, j;

	// initialize greatest upper and least lower bounds
	lub_divisor_shift = 3;
	lub_index = 0;
	lub = thisboard->clock[lub_index] * (1 << lub_divisor_shift);
	glb_divisor_shift = 0;
	glb_index = thisboard->num_clocks - 1;
	glb = thisboard->clock[glb_index] * (1 << lub_divisor_shift);

	// make sure period is in available range
	if(*period < glb)
		*period = glb;
	if(*period > lub)
		*period = lub;

	// we can multiply period by 1, 2, 4, or 8, using (1 << i)
	for(i = 0; i < 4; i = i++)
	{
		// there are a maximum of 4 master clocks
		for(j = 0; j < 4; j++)
		{
			if(thisboard->clock[j] == 0) break;
			// temp is the period in nanosec we are evaluating
			temp = thisboard->clock[j] * (1 << i);
			// if it is the best match yet
			if(temp < lub && temp >= *period)
			{
				lub_divisor_shift = i;
				lub_index = j;
				lub = temp;
			}
			if(temp > glb && temp <= *period)
			{
				glb_divisor_shift = i;
				glb_index = j;
				glb = temp;
			}
		}
	}
	flags &= TRIG_ROUND_MASK;
	switch (flags)
	{
		case TRIG_ROUND_NEAREST:
		default:
			// if least upper bound is better approximation
			if(lub - *period < *period - glb)
			{
				*period = lub;
			}else
			{
				*period = glb;
			}
			break;
		case TRIG_ROUND_UP:
				*period = lub;
			break;
		case TRIG_ROUND_DOWN:
				*period = glb;
			break;
	}

	// set clock bits for config register appropriately
	devpriv->config_bits &= ~CLOCK_MASK;
	if(*period == lub)
	{
		devpriv->config_bits |= CLOCK_SELECT_BITS(lub_index) | CLOCK_DIVISOR_BITS(lub_divisor_shift);
	}else
	{
		devpriv->config_bits |= CLOCK_SELECT_BITS(glb_index) | CLOCK_DIVISOR_BITS(glb_divisor_shift);
	}

	return 0;
}

static int a2150_set_chanlist(comedi_device *dev, unsigned int start_channel, unsigned int num_channels)
{
	if(start_channel + num_channels > 4)
		return -1;

	devpriv->config_bits &= ~CHANNEL_MASK;

	switch(num_channels)
	{
		case 1:
			devpriv->config_bits |= CHANNEL_BITS(0x100 | start_channel);
			break;
		case 2:
			if(start_channel == 0)
			{
				devpriv->config_bits |= CHANNEL_BITS(0x010);
			}else if(start_channel == 2)
			{
				devpriv->config_bits |= CHANNEL_BITS(0x011);
			}else
			{
				return -1;
			}
			break;
		case 4:
			devpriv->config_bits |= CHANNEL_BITS(0x001);
			break;
		default:
			return -1;
			break;
	}

	return 0;
}

