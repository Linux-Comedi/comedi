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

Board has quirky chanlist when scanning multiple channels.  Scan sequence must start
at highest channel, then decrement down to channel 0.
*/

/*
TODO:
	command support
	additional boards
*/
//XXX stop_src TRIG_EXT is _not_ supported by the hardware, at least not in combination with dma transfers

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
#include <asm/dma.h>
#include "8253.h"
#include "8255.h"

#define LABPC_SIZE           32	// size of io region used by board
#define LABPC_TIMER_BASE            500	// 2 MHz master clock

/* Registers for the lab-pc+ */

//write-only registers
#define COMMAND1_REG	0x0
#define   ADC_GAIN_BITS(x)	(((x) & 0x7) << 4)
#define   ADC_CHAN_BITS(x)	((x) & 0x7)
#define   ADC_SCAN_EN_BIT	80	// enables multi channel scans
#define COMMAND2_REG	0x1
#define   PRETRIG_BIT	0x1	// enable pretriggering (used in conjunction with SWTRIG)
#define   HWTRIG_BIT	0x2	// enable paced conversions on external trigger
#define   SWTRIG_BIT	0x4	// enable paced conversions
#define   CASCADE_BIT	0x8	// use two cascaded counters for pacing
#define   DAC_PACED_BIT(channel)	(0x40 << ((channel) & 0x1))
#define COMMAND3_REG	0x2
#define   DMA_EN_BIT	0x1	// enable dma transfers
#define   DIO_INTR_EN_BIT	0x2	// enable interrupts for 8255
#define   DMATC_INTR_EN_BIT	0x4	// enable dma terminal count interrupt
#define   TIMER_INTR_EN_BIT	0x8	// enable timer interrupt
#define   ERR_INTR_EN_BIT	0x10	// enable error interrupt
#define   FNE_INTR_EN_BIT	0x20	// enable fifo not empty interrupt
#define ADC_CONVERT_REG	0x3
#define DAC_LSB_REG(channel)	(0x4 + 2 * ((channel) & 0x1))
#define DAC_MSB_REG(channel)	(0x5 + 2 * ((channel) & 0x1))
#define ADC_CLEAR_REG	0x8
#define DMATC_CLEAR_REG	0xa
#define TIMER_CLEAR_REG	0xc
#define COMMAND4_REG            0xf
#define   EXT_SCAN_MASTER_EN_BIT	0x1	// enables 'interval' scanning
#define   EXT_SCAN_EN_BIT	0x2	// enables external signal on counter b1 output to trigger scan
#define   EXT_CONVERT_OUT_BIT	0x4	// chooses direction (output or input) for EXTCONV* line
#define   ADC_DIFF_BIT	0x8	// chooses differential inputs for adc (in conjunction with board jumper)
#define   EXT_CONVERT_DISABLE_BIT	0x10
#define INTERVAL_COUNT_REG	0x1e
#define INTERVAL_LOAD_REG	0x1f

// read-only registers
#define STATUS_REG	0x0
#define   DATA_AVAIL_BIT	0x1	// data is available in fifo
#define   OVERRUN_BIT	0x2	// overrun has occurred
#define   OVERFLOW_BIT	0x4	// fifo overflow
#define   TIMER_BIT	0x8	// timer interrupt has occured
#define   DMATC_BIT	0x10	// dma terminal count has occured
#define   EXT_TRIG_BIT	0x40	// external trigger has occured
#define   NOT_PCPLUS_BIT	0x80	// no a lab-pc+
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

static const int dma_buffer_size = 0xff00;	// size in bytes of dma buffer
const int sample_size = 2;	// 2 bytes per sample

typedef struct{
	volatile unsigned int count;  /* number of data points left to be taken */
	unsigned int ao_value[2];	// software copy of analog output values
	// software copys of bits written to command registers
	unsigned int command1_bits;
	unsigned int command2_bits;
	unsigned int command3_bits;
	unsigned int command4_bits;
	unsigned int divisor1;	/* value to load into board's counter a0 for timed conversions */
	unsigned int divisor2; 	/* value to load into board's counter b0 for timed conversions */
	unsigned int dma_chan;	// dma channel to use
	u16 *dma_buffer;	// buffer ai will dma into
	unsigned int dma_transfer_size;	// transfer size in bytes for current transfer
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
static unsigned int labpc_suggest_transfer_size(comedi_cmd cmd);

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
	int i;
	int status;
	unsigned long flags;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async;
	unsigned int max_points, num_points, residue, leftover;

// XXX deal with stop_src TRIG_EXT

	if(dev->attached == 0)
	{
		comedi_error(dev, "premature interrupt");
		return;
	}
	// initialize async here to make sure s is not NULL
	async = s->async;
	async->events = 0;

	status = inb(dev->iobase + STATUS_REG);

	if((status & (DMATC_BIT | TIMER_BIT | OVERFLOW_BIT | OVERRUN_BIT /*| DATA_AVAIL_BIT*/)) == 0)
	{
		comedi_error(dev, "spurious interrupt");
		return;
	}

	if(status & OVERRUN_BIT)
	{
		// clear error interrupt
		outb(0x1, dev->iobase + ADC_CLEAR_REG);
		async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
		comedi_event(dev, s, async->events);
		comedi_error(dev, "overrun");
		return;
	}

	if(status & DMATC_BIT)
	{
		flags = claim_dma_lock();
		disable_dma(devpriv->dma_chan);
		/* clear flip-flop to make sure 2-byte registers for
		* count and address get set correctly */
		clear_dma_ff(devpriv->dma_chan);

		// figure out how many points to read
		max_points = devpriv->dma_transfer_size  / sample_size;
		/* residue is the number of points left to be done on the dma
		* transfer.  It should always be zero at this point unless
		* the stop_src is set to external triggering.
		*/
		residue = get_dma_residue(devpriv->dma_chan) / sample_size;
		num_points = max_points - residue;
		if(devpriv->count < num_points &&
			async->cmd.stop_src == TRIG_COUNT)
			num_points = devpriv->count;

		// figure out how many points will be stored next time
		leftover = 0;
		if(async->cmd.stop_src != TRIG_COUNT)
		{
			leftover = devpriv->dma_transfer_size / sample_size;
		}else if(devpriv->count > num_points)
		{
			leftover = devpriv->count - num_points;
			if(leftover > max_points)
				leftover = max_points;
		}

		for(i = 0; i < num_points; i++)
		{
			/* write data point to comedi buffer */
			comedi_buf_put(async, devpriv->dma_buffer[i]);
			if(async->cmd.stop_src == TRIG_COUNT) devpriv->count--;
		}
		// re-enable  dma
		set_dma_addr(devpriv->dma_chan, virt_to_bus(devpriv->dma_buffer));
		set_dma_count(devpriv->dma_chan, leftover * sample_size);
		enable_dma(devpriv->dma_chan);
		release_dma_lock(flags);

		async->events |= COMEDI_CB_BLOCK;

		if(devpriv->count == 0)
		{	/* end of acquisition */
			labpc_cancel(dev, s);
			async->events |= COMEDI_CB_EOA;
		}
	}

	if(status & TIMER_BIT)
	{
		comedi_error(dev, "handled timer interrupt?");
		// clear it
		outb(0x1, dev->iobase + TIMER_CLEAR_REG);
	}

	if(status & OVERFLOW_BIT)
	{
		// clear error interrupt
		outb(0x1, dev->iobase + ADC_CLEAR_REG);
		async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
		comedi_error(dev, "overflow");
		return;
	}

	comedi_event(dev, s, async->events);
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
	unsigned long flags;

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

	// grab dma channel
	if(dma_chan < 0 || dma_chan > 3)
	{
		printk(" invalid dma channel\n");
		return -EINVAL;
	}else if(dma_chan)
	{
		// allocate dma buffer
		devpriv->dma_buffer = kmalloc(dma_buffer_size, GFP_KERNEL | GFP_DMA);
		if(devpriv->dma_buffer == NULL)
		{
			printk(" failed to allocate dma buffer\n");
			return -ENOMEM;
		}
		if(request_dma(dma_chan, driver_labpc.driver_name))
		{
			printk(" failed to allocate dma channel %i\n", dma_chan);
			return -EINVAL;
		}
		devpriv->dma_chan = dma_chan;
		flags = claim_dma_lock();
		disable_dma(devpriv->dma_chan);
		set_dma_mode(devpriv->dma_chan, DMA_MODE_READ);
		release_dma_lock(flags);
	}

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
/* XXX could provide command support, except it doesn't have a hardware
 * buffer for analog output and no underrun flag so speed would be very
 * limited unless using RT interrupt */
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
	if(devpriv->dma_buffer)
		kfree(devpriv->dma_buffer);
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
	int err = 0;
	int tmp;
	int gain;
	int i;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src || tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW | TRIG_EXT;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src) err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if(!cmd->convert_src || tmp != cmd->convert_src) err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src) err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_EXT | TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src) err++;

	if(err) return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->start_src != TRIG_NOW &&
		cmd->start_src != TRIG_EXT) err++;
	if(cmd->scan_begin_src != TRIG_FOLLOW &&
	   cmd->scan_begin_src != TRIG_EXT) err++;
	if(cmd->convert_src != TRIG_TIMER &&
	   cmd->convert_src != TRIG_EXT) err++;
	if(cmd->stop_src != TRIG_COUNT &&
		cmd->stop_src != TRIG_EXT &&
		cmd->stop_src != TRIG_NONE) err++;

	// can't have external stop and start triggers at once
	if(cmd->start_src == TRIG_EXT &&
		cmd->stop_src == TRIG_EXT) err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg == TRIG_NOW && cmd->start_arg != 0)
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
		err++;
	}
	if(cmd->scan_end_arg != cmd->chanlist_len)
	{
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}

	// stop source
	switch(cmd->stop_src)
	{
		case TRIG_COUNT:
			if(!cmd->stop_arg)
			{
				cmd->stop_arg = 1;
				err++;
			}
			break;
		case TRIG_NONE:
			if(cmd->stop_arg != 0)
			{
				cmd->stop_arg = 0;
				err++;
			}
			break;
		// TRIG_EXT doesn't care since it doesn't trigger of a numbered channel
		default:
			break;
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	if(cmd->convert_src == TRIG_TIMER)
	{
		tmp = cmd->convert_arg;
		/* calculate counter values that give desired timing */
		i8253_cascade_ns_to_timer_2div(LABPC_TIMER_BASE, &(devpriv->divisor1), &(devpriv->divisor2), &(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->convert_arg) err++;
	}

	if(err)return 4;

	// check channel/gain list against card's limitations
	if(cmd->chanlist && cmd->chanlist_len > 1)
	{
		gain = CR_RANGE(cmd->chanlist[0]);
		for(i = 1; i < cmd->chanlist_len; i++)
		{
			if(CR_CHAN(cmd->chanlist[i]) != cmd->chanlist_len - i - 1)
			{
				comedi_error(dev, "entries in multiple channel chanlist must start high then decrement down to channel 0.\n");
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

	return 0;
}

static int labpc_ai_cmd(comedi_device *dev, comedi_subdevice *s)
{
	int endChan, range;
	unsigned long irq_flags;
	int ret;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;

	if(!dev->irq)
	{
		comedi_error(dev, "no irq assigned, cannot perform command");
		return -1;
	}

	if(!devpriv->dma_chan)
	{
		comedi_error(dev, "no dma channel assigned, cannot perform command");
		return -1;
	}

	if(cmd->flags & TRIG_RT)
	{
		comedi_error(dev, "ISA DMA is unsafe at RT priority (TRIG_RT flag), aborting");
		return -1;
	}

	// make sure board is disabled before setting up aquisition
	devpriv->command2_bits &= ~SWTRIG_BIT & ~HWTRIG_BIT & ~PRETRIG_BIT;
	outb(devpriv->command2_bits, dev->iobase + COMMAND3_REG);
	devpriv->command3_bits = 0;
	outb(devpriv->command3_bits, dev->iobase + COMMAND3_REG);

	/* setup channel list, etc (command1 register) */
	devpriv->command1_bits = 0;
	endChan = CR_CHAN(cmd->chanlist[cmd->chanlist_len - 1]);
	devpriv->command1_bits |= ADC_CHAN_BITS(endChan);
	range = CR_RANGE(cmd->chanlist[0]);
	devpriv->command1_bits |= ADC_GAIN_BITS(range);
	outb(devpriv->command1_bits, dev->iobase + COMMAND1_REG);
	// manual says to set scan enable bit on second pass
	if(cmd->chanlist_len > 1)
	{
		devpriv->command1_bits |= ADC_SCAN_EN_BIT;
		outb(devpriv->command1_bits, dev->iobase + COMMAND1_REG);
	}

	// setup any external triggering/pacing (command4 register)
	devpriv->command4_bits = 0;
	if(cmd->convert_src != TRIG_EXT)
		devpriv->command4_bits |= EXT_CONVERT_DISABLE_BIT;
	if(cmd->scan_begin_src == TRIG_EXT)
		devpriv->command4_bits |= EXT_SCAN_MASTER_EN_BIT | EXT_SCAN_EN_BIT;
	// single-ended/differential
	if(CR_AREF(cmd->chanlist[0]) == AREF_DIFF)
		devpriv->command4_bits |= ADC_DIFF_BIT;
	outb(devpriv->command4_bits, dev->iobase + COMMAND4_REG);
//XXX interval counter register for single channel

	// initialize software conversion count
	if(cmd->stop_src == TRIG_COUNT)
	{
		devpriv->count = cmd->stop_arg * cmd->chanlist_len;
	}

	// set up conversion pacing
	if(cmd->convert_src == TRIG_TIMER)
	{
		/* set conversion frequency */
		i8253_cascade_ns_to_timer_2div(LABPC_TIMER_BASE, &(devpriv->divisor1),
			&(devpriv->divisor2), &(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
		// load counter b0 in mode 2 (manual says mode 3 but I don't see any reason)
		ret = i8254_load(dev->iobase + COUNTER_B_BASE_REG, 0, devpriv->divisor2, 2);
		if(ret < 0)
		{
			comedi_error(dev, "error loading counter");
			return -1;
		}
		// load counter a0 in mode 2
		ret = i8254_load(dev->iobase + COUNTER_A_BASE_REG, 0, devpriv->divisor1, 2);
		if(ret < 0)
		{
			comedi_error(dev, "error loading counter");
			return -1;
		}
	}

	// clear adc fifo
	outb(0x1, dev->iobase + ADC_CLEAR_REG);
	inb(dev->iobase + ADC_FIFO_REG);
	inb(dev->iobase + ADC_FIFO_REG);

	// set up dma transfer
	irq_flags = claim_dma_lock();
	disable_dma(devpriv->dma_chan);
	/* clear flip-flop to make sure 2-byte registers for
	 * count and address get set correctly */
	clear_dma_ff(devpriv->dma_chan);
	set_dma_addr(devpriv->dma_chan, virt_to_bus(devpriv->dma_buffer));
	// set appropriate size of transfer
	devpriv->dma_transfer_size = labpc_suggest_transfer_size(*cmd);
	if(cmd->stop_src == TRIG_COUNT &&
		devpriv->count * sample_size < devpriv->dma_transfer_size)
	{
		devpriv->dma_transfer_size = devpriv->count * sample_size;
	}
	set_dma_count(devpriv->dma_chan, devpriv->dma_transfer_size);
 	enable_dma(devpriv->dma_chan);
	release_dma_lock(irq_flags);

	// enable board's dma and interrupts
	devpriv->command3_bits |= DMA_EN_BIT | DMATC_INTR_EN_BIT | ERR_INTR_EN_BIT;
	// disable fifo not empty interrupt
	devpriv->command3_bits &= ~FNE_INTR_EN_BIT;
	outb(devpriv->command3_bits, dev->iobase + COMMAND3_REG);

// XXX setup counter a1 for external stop trigger, or just to prevent it from messing us up

	// startup aquisition

	// command2 reg
	// use 2 cascaded counters for pacing
	devpriv->command2_bits |= CASCADE_BIT;
	switch(cmd->start_src)
	{
		case TRIG_EXT:
			devpriv->command2_bits |= HWTRIG_BIT;
			devpriv->command2_bits &= ~PRETRIG_BIT & SWTRIG_BIT;
			break;
		case TRIG_NOW:
			devpriv->command2_bits |= SWTRIG_BIT;
			devpriv->command2_bits &= ~PRETRIG_BIT & ~HWTRIG_BIT;
			break;
		default:
			comedi_error(dev, "bug with start_src");
			return -1;
			break;
	}
	switch(cmd->stop_src)
	{
		case TRIG_EXT:
			devpriv->command2_bits |= HWTRIG_BIT | PRETRIG_BIT;
			break;
		case TRIG_COUNT:
		case TRIG_NONE:
			break;
		default:
			comedi_error(dev, "bug with stop_src");
			return -1;
	}
	outb(devpriv->command2_bits, dev->iobase + COMMAND2_REG);

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

	//XXX command4 set se/diff
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

// utility function that suggests a dma transfer size in bytes
static unsigned int labpc_suggest_transfer_size(comedi_cmd cmd)
{
	unsigned int size;
	unsigned int freq;

	if(cmd.convert_src == TRIG_TIMER)
		freq = 1000000000 / cmd.convert_arg;
	// return some default value
	else
		freq = 0xffffffff;

	// XXX this is a hack I should use pio from TRIG_WAKE_EOS
	if(cmd.flags & TRIG_WAKE_EOS)
	{
		size = sample_size * cmd.chanlist_len;
	}else
	{
		// make buffer fill in no more than 1/3 second
		size = (freq / 3) * sample_size;
	}

	// set a minimum and maximum size allowed
	if(size > dma_buffer_size)
		size = dma_buffer_size - dma_buffer_size % sample_size;
	else if(size < sample_size)
		size = sample_size;

	return size;
}
