/*
    das800.c driver for Keitley das800 series boards and compatibles
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

This driver supports the Keithley boards:
das-800
das-801
das-802

and Measurement Computing (Computer Boards) models:
cio-das800
cio-das801
cio-das802

Options:
	[0] - base io address
	[1] - irq

irq can be omitted, although the cmd interface will not work without it.

cmd triggers supported:
	start_src:      TRIG_NOW | TRIG_EXT
	scan_begin_src: TRIG_FOLLOW
	scan_end_src:   TRIG_COUNT
	convert_src:    TRIG_TIMER | TRIG_EXT
	stop_src:       TRIG_NONE | TRIG_COUNT

The number of channels scanned is determined from chanlist_len.  The
starting channel to scan and the gain is determined from chanlist[0].

NOTES:
	I've never tested the gain setting stuff since I only have a
	DAS-800 board with fixed gain.

//FIXME need to protect indirect addressing from interrupt

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
#include <asm/spinlock.h>
#include "das_spinlock.h"

#define DAS800_SIZE           8
#define TIMER_BASE            1000
/* Registers for the das800 */

#define DAS800_LSB            0
#define   FIFO_EMPTY            0x1
#define   FIFO_OVF              0x2
#define DAS800_MSB            1
#define DAS800_CONTROL1       2
#define   CONTROL1_INTE         0x8
#define DAS800_CONV_CONTROL   2
#define   ITE                   0x1
#define   CASC                  0x2
#define   DTEN                  0x4
#define   IEOC                  0x8
#define   EACS                  0x10
#define   CONV_HCEN             0x80
#define DAS800_SCAN_LIMITS    2
#define DAS800_STATUS         2
#define   IRQ                   0x8
#define   BUSY                  0x80
#define DAS800_GAIN           3
#define   CONTROL1              0x80
#define   CONV_CONTROL          0xa0
#define   SCAN_LIMITS           0xc0
#define   ID                    0xe0
#define DAS800_STATUS2        7
#define   STATUS2_HCEN          0x80
#define   STATUS2_INTE          0X20
#define DAS800_ID             7

typedef struct das800_board_struct{
	char *name;
	int ai_speed;
}das800_board;

enum{das800, ciodas800, das801, ciodas801, das802, ciodas802};

das800_board das800_boards[] =
{
	{
		name:	"DAS-800",
		ai_speed:	25000,
	},
	{
		name:	"CIO-DAS800",
		ai_speed:	20000,
	},
	{
		name:		"DAS-801",
		ai_speed:	25000,
	},
	{
		name:	"CIO-DAS801",
		ai_speed:	20000,
	},
	{
		name:		"DAS-802",
		ai_speed:	25000,
	},
	{
		name:	"CIO-DAS802",
		ai_speed:	20000,
	},
};
/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((das800_board *)dev->board_ptr)

typedef struct{
	unsigned long count;  /* number of data points left to be taken */
	int forever;  /* flag indicating whether we should take data forever */
	unsigned int divisor1;	/* value to load into board's counter 1 for timed conversions */
	unsigned int divisor2; 	/* value to load into board's counter 2 for timed conversions */
	int do_bits;	/* digital output bits */
	spinlock_t spinlock;
}das800_private;

#define devpriv ((das800_private *)dev->private)

static comedi_lrange range_das800_ai = {
	1,
	{
		RANGE( -5, 5 ),
	}
};

static comedi_lrange range_das801_ai = {
	9,
	{
		RANGE(-5, 5),
		RANGE(-10, 10),
		RANGE(0, 10),
		RANGE(-0.5, 0.5),
		RANGE(0, 1),
		RANGE(-0.05, 0.05),
		RANGE(0, 0.1),
		RANGE(-0.01, 0.01),
		RANGE(0, 0.02),
	}
};

static comedi_lrange range_cio_das801_ai = {
	9,
	{
		RANGE(-5, 5),
		RANGE(-10, 10),
		RANGE(0, 10),
		RANGE(-0.5, 0.5),
		RANGE(0, 1),
		RANGE(-0.05, 0.05),
		RANGE(0, 0.1),
		RANGE(-0.005, 0.005),
		RANGE(0, 0.01),
	}
};

static comedi_lrange range_das802_ai = {
	9,
	{
		RANGE(-5, 5),
		RANGE(-10, 10),
		RANGE(0, 10),
		RANGE(-2.5, 2.5),
		RANGE(0, 5),
		RANGE(-1.25, 1.25),
		RANGE(0, 2.5),
		RANGE(-0.625, 0.625),
		RANGE(0, 1.25),
	}
};

static comedi_lrange *das800_range_lkup[] = {
	&range_das800_ai,
	&range_das800_ai,
	&range_das801_ai,
	&range_cio_das801_ai,
	&range_das802_ai,
	&range_das802_ai,
};

static int das800_attach(comedi_device *dev,comedi_devconfig *it);
static int das800_detach(comedi_device *dev);
static int das800_recognize(char *name);
static int das800_cancel(comedi_device *dev, comedi_subdevice *s);

comedi_driver driver_das800={
	driver_name:	"das800",
	module:		THIS_MODULE,
	attach:		das800_attach,
	detach:		das800_detach,
	recognize:		das800_recognize,
};

static void das800_interrupt(int irq, void *d, struct pt_regs *regs);
void enable_das800(comedi_device *dev);
void disable_das800(comedi_device *dev);
static int das800_ai_do_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd);
static int das800_ai_do_cmd(comedi_device *dev, comedi_subdevice *s);
static int das800_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int das800_di_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int das800_do_winsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
int das800_probe(comedi_device *dev);
int das800_set_frequency(comedi_device *dev);
int das800_load_counter(unsigned int counterNumber, unsigned int counterValue, comedi_device *dev);

static int das800_recognize(char *name)
{
	if(!strcmp(name, "das-800") || !strcmp(name, "das800"))
		return das800;
	if(!strcmp(name, "cio-das800"))
		return ciodas800;
	if(!strcmp(name, "das-801"))
		return das801;
	if(!strcmp(name, "cio-das801"))
		return ciodas801;
	if(!strcmp(name, "das-802"))
		return das802;
	if(!strcmp(name, "cio-das802"))
		return ciodas802;

	return -1;
}

/* checks and probes das-800 series board type */
int das800_probe(comedi_device *dev)
{
	int id;
	unsigned long irq_flags;

	// 'comedi spin lock irqsave' disables even rt interrupts, we use them to protect indirect addressing
	comedi_spin_lock_irqsave(&devpriv->spinlock, irq_flags);
	outb(ID, dev->iobase + DAS800_GAIN);	/* select base address + 7 to be ID register */
	id = inb(dev->iobase + DAS800_ID) & 0x3; /* get id bits */
	comedi_spin_unlock_irqrestore(&devpriv->spinlock, irq_flags);

	switch(id)
	{
		case 0x0:
			if(dev->board == das800)
			{
				printk(" Board model: DAS-800\n");
				return dev->board;
			}
			if(dev->board == ciodas800)
			{
				printk(" Board model: CIO-DAS800\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-800\n");
			return das800;
			break;
		case 0x2:
			if(dev->board == das801)
			{
				printk(" Board model: DAS-801\n");
				return dev->board;
			}
			if(dev->board == ciodas801)
			{
				printk(" Board model: CIO-DAS801\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-801\n");
			return das801;
			break;
		case 0x3:
			if(dev->board == das802)
			{
				printk(" Board model: DAS-802\n");
				return dev->board;
			}
			if(dev->board == ciodas802)
			{
				printk(" Board model: CIO-DAS802\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-802\n");
			return das802;
			break;
		default :
			printk(" Board model: probe returned 0x%x (unknown)\n", id);
			return -1;
			break;
	}
	return -1;
}

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_das800);

/* interrupt service routine */
static void das800_interrupt(int irq, void *d, struct pt_regs *regs)
{
	short i;		/* loop index */
	sampl_t dataPoint;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + dev->read_subdev;	/* analog input subdevice */
	int status;
	unsigned long irq_flags;

	status = inb(dev->iobase + DAS800_STATUS);
	/* if interupt was not generated by board, quit */
	if(!(status & IRQ) || !(dev->attached))
		return;
  /* read 16 bits from dev->iobase and dev->iobase + 1 */
	dataPoint = inb(dev->iobase + DAS800_LSB);
	dataPoint += inb(dev->iobase + DAS800_MSB) << 8;
	/* loop while card's fifo is not empty (and make sure loop terminates by limiting to 256 iterations) */
	for(i = 0; (dataPoint & FIFO_EMPTY) == 0 && i < 256; i++)
	{
		if( dataPoint & FIFO_OVF )
		{
			comedi_error(dev, "DAS800 FIFO overflow");
			das800_cancel(dev, dev->subdevices + 0);
			comedi_error_done(dev, s);
			return;
		}
		dataPoint = (dataPoint >> 4) & 0xfff;		/* strip off extraneous bits */
		/* if there are more data points to collect */
		if(devpriv->count > 0 || devpriv->forever == 1)
		{
			/* write data point to buffer */
			if(s->buf_int_ptr >= s->prealloc_bufsz )
			{
				s->buf_int_ptr = 0;
				comedi_eobuf(dev, s);
			}
			*((sampl_t *)((void *)s->prealloc_buf + s->buf_int_ptr)) = dataPoint;
			s->cur_chan++;
			if( s->cur_chan >= s->cur_chanlist_len )
			{
				s->cur_chan = 0;
				comedi_eos(dev, s);
			}
			s->buf_int_count += sizeof(sampl_t);
			s->buf_int_ptr += sizeof(sampl_t);
			if(devpriv->count > 0) devpriv->count--;
		}
		/* read 16 bits from dev->iobase and dev->iobase + 1 */
		dataPoint = inb(dev->iobase + DAS800_LSB);
		dataPoint += inb(dev->iobase + DAS800_MSB) << 8;
	}

	/* check for overflow on last data point */
	if( dataPoint & FIFO_OVF )
	{
		comedi_error(dev, "DAS800 FIFO overflow");
		das800_cancel(dev, dev->subdevices + 0);
		comedi_error_done(dev, s);
		return;
	}
	comedi_bufcheck(dev,s);
	if(devpriv->count > 0 || devpriv->forever == 1)
	{
		/* Re-enable card's interrupt */
		comedi_spin_lock_irqsave(&devpriv->spinlock, irq_flags);  // spin lock makes indirect addressing SMP safe
		outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be control register 1 */
		outb(CONTROL1_INTE | devpriv->do_bits, dev->iobase + DAS800_CONTROL1);
		comedi_spin_unlock_irqrestore(&devpriv->spinlock, irq_flags);
	/* otherwise, stop taking data */
	} else
	{
		disable_das800(dev);		/* diable hardware triggered conversions */
		comedi_done(dev, s);
	}
	return;
}

static int das800_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int iobase = it->options[0];
	int irq = it->options[1];
	unsigned long irq_flags;

	printk("comedi%d: das800: io 0x%x", dev->minor, iobase);
	if(irq)
	{
		printk(", irq %i", irq);
	}
	printk("\n");

	if(iobase == 0)
	{
		printk("io base address required for das800\n");
		return -EINVAL;
	}

	/* check if io addresses are available */
	if(check_region(iobase, DAS800_SIZE) < 0)
	{
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase, DAS800_SIZE, "das800");
	dev->iobase = iobase;
	dev->iosize = DAS800_SIZE;

	/* grab our IRQ */
	if(irq == 1 || irq > 7 || irq < 0)
	{
		printk("irq out of range\n");
		return -EINVAL;
	}
	if(irq)
	{
		if(comedi_request_irq( irq, das800_interrupt, 0, "das800", dev ))
		{
			printk( "unable to allocate irq %d\n", irq);
			return -EINVAL;
		}
	}
	dev->irq = irq;

	dev->board = das800_probe(dev);
	if(dev->board < 0)
	{
		printk("unable to determine board type\n");
		return -ENODEV;
	}

	dev->board_ptr = das800_boards + dev->board;
	dev->board_name = thisboard->name;

	/* allocate and initialize dev->private */
	if(alloc_private(dev, sizeof(das800_private)) < 0)
		return -ENOMEM;

	dev->n_subdevices = 3;
	if(alloc_subdevices(dev) < 0)
		return -ENOMEM;

	/* analog input subdevice */
	dev->read_subdev = 0;
	s = dev->subdevices + 0;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 8;
	s->len_chanlist = 8;
	s->maxdata = 0xfff;
	s->range_table = das800_range_lkup[dev->board];
	s->do_cmd = das800_ai_do_cmd;
	s->do_cmdtest = das800_ai_do_cmdtest;
	s->insn_read = das800_ai_rinsn;
	s->cancel = das800_cancel;

	/* di */
	s = dev->subdevices + 1;
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 3;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_read = das800_di_rinsn;

	/* do */
	s = dev->subdevices + 2;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_write = das800_do_winsn;

	disable_das800(dev);

	/* initialize digital out channels */
	comedi_spin_lock_irqsave(&devpriv->spinlock, irq_flags);
	outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be control register 1 */
	outb(CONTROL1_INTE | devpriv->do_bits, dev->iobase + DAS800_CONTROL1);
	comedi_spin_unlock_irqrestore(&devpriv->spinlock, irq_flags);

	return 0;
};

static int das800_detach(comedi_device *dev)
{
	printk("comedi%d: das800: remove\n", dev->minor);

	/* only free stuff if it has been allocated by _attach */
	if(dev->iobase)
		release_region(dev->iobase, DAS800_SIZE);
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);
	return 0;
};

static int das800_cancel(comedi_device *dev, comedi_subdevice *s)
{
	devpriv->forever = 0;
	devpriv->count = 0;
	disable_das800(dev);
	return 0;
}

/* enable_das800 makes the card start taking hardware triggered conversions */
void enable_das800(comedi_device *dev)
{
	unsigned long irq_flags;
	comedi_spin_lock_irqsave(&devpriv->spinlock, irq_flags);
	outb(CONV_CONTROL, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be conversion control register */
	outb(CONV_HCEN, dev->iobase + DAS800_CONV_CONTROL);	/* enable hardware triggering */
	outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be control register 1 */
	outb(CONTROL1_INTE | devpriv->do_bits, dev->iobase + DAS800_CONTROL1);	/* enable card's interrupt */
	comedi_spin_unlock_irqrestore(&devpriv->spinlock, irq_flags);
}

/* disable_das800 stops hardware triggered conversions */
void disable_das800(comedi_device *dev)
{
	unsigned long irq_flags;
	comedi_spin_lock_irqsave(&devpriv->spinlock, irq_flags);
	outb(CONV_CONTROL, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be conversion control register */
	outb(0x0, dev->iobase + DAS800_CONV_CONTROL);	/* disable hardware triggering of conversions */
	comedi_spin_unlock_irqrestore(&devpriv->spinlock, irq_flags);
}

static int das800_ai_do_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err = 0;
	int tmp;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src && tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW;
	if(!cmd->scan_begin_src && tmp != cmd->scan_begin_src) err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if(!cmd->convert_src && tmp != cmd->convert_src) err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src && tmp != cmd->scan_end_src) err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if(!cmd->stop_src && tmp!=cmd->stop_src) err++;

	if(err) return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->start_src != TRIG_NOW &&
		cmd->start_src != TRIG_EXT) err++;
	if(cmd->scan_begin_src != TRIG_FOLLOW) err++;
	if(cmd->convert_src != TRIG_TIMER &&
	   cmd->convert_src != TRIG_EXT) err++;
	if(cmd->scan_end_src != TRIG_COUNT) err++;
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
	if(cmd->chanlist_len > s->len_chanlist)
	{
		cmd->chanlist_len = s->len_chanlist;
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

	return 0;
}

static int das800_ai_do_cmd(comedi_device *dev, comedi_subdevice *s)
{
	int startChan, endChan, scan, gain;
	int conv_bits;
	unsigned long irq_flags;

	if(!dev->irq)
	{
		comedi_error(dev, "no irq assigned for das-800, cannot do hardware conversions");
		return -1;
	}

	disable_das800(dev);

	/* set channel scan limits */
	startChan = CR_CHAN(s->cmd.chanlist[0]);
	endChan = (startChan + s->cmd.chanlist_len - 1) % 8;
	scan = (endChan << 3) | startChan;

	comedi_spin_lock_irqsave(&devpriv->spinlock, irq_flags);
	outb(SCAN_LIMITS, dev->iobase + DAS800_GAIN);	/* select base address + 2 to be scan limits register */
	outb(scan, dev->iobase + DAS800_SCAN_LIMITS); /* set scan limits */
	comedi_spin_unlock_irqrestore(&devpriv->spinlock, irq_flags);

	/* set gain */
	gain = CR_RANGE(s->cmd.chanlist[0]);
	if( gain > 0)
		gain += 0x7;
	gain &= 0xf;
	outb(gain, dev->iobase + DAS800_GAIN);

	switch(s->cmd.stop_src)
	{
		case TRIG_COUNT:
			devpriv->count = s->cmd.stop_arg * s->cmd.chanlist_len;
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
	if(s->cmd.start_src == TRIG_EXT)
		conv_bits |= DTEN;
	switch(s->cmd.convert_src)
	{
		case TRIG_TIMER:
			conv_bits |= CASC | ITE;
			/* set conversion frequency */
			i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->divisor1), &(devpriv->divisor2), &(s->cmd.convert_arg), s->cmd.flags & TRIG_ROUND_MASK);
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

	comedi_spin_lock_irqsave(&devpriv->spinlock, irq_flags);
	outb(CONV_CONTROL, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be conversion control register */
	outb(conv_bits, dev->iobase + DAS800_CONV_CONTROL);
	comedi_spin_unlock_irqrestore(&devpriv->spinlock, irq_flags);

	enable_das800(dev);
	return 0;
}

static int das800_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
 	int i, n;
	int chan;
	int range;
	int lsb, msb;
	int timeout = 1000;
	unsigned long irq_flags;

	disable_das800(dev);	/* disable hardware conversions (enables software conversions) */

	/* set multiplexer */
	chan = CR_CHAN(insn->chanspec);

	comedi_spin_lock_irqsave(&devpriv->spinlock, irq_flags);
	outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be control register 1 */
	outb(chan | devpriv->do_bits, dev->iobase + DAS800_CONTROL1);
	comedi_spin_unlock_irqrestore(&devpriv->spinlock, irq_flags);

	/* set gain / range */
	range = CR_RANGE(insn->chanspec);
	if(range)
		range += 0x7;
	range &= 0xf;
	outb(range, dev->iobase + DAS800_GAIN);

	udelay(5);

	for(n = 0; n < insn->n; n++)
	{
		/* trigger conversion */
		outb_p(0, dev->iobase + DAS800_MSB);

		for(i = 0; i < timeout; i++)
		{
			if(!(inb(dev->iobase + DAS800_STATUS) & BUSY))
				break;
		}
		if(i == timeout)
		{
			comedi_error(dev, "timeout");
			return -ETIME;
		}
		lsb = inb(dev->iobase + DAS800_LSB);
		msb = inb(dev->iobase + DAS800_MSB);
		data[n] = (lsb >> 4) | (msb << 4);
	}

	return n;
}

static int das800_di_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int chan = CR_CHAN(insn->chanspec);
	int ret;

	ret = inb(dev->iobase + DAS800_STATUS) & (1 << (chan + 4));
	if(ret) data[0] = 1;
	else data[0] = 0;

	return 1;
}

static int das800_do_winsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int mux_bits;
	int chan = CR_CHAN(insn->chanspec);
	unsigned long irq_flags;

	mux_bits = inb(dev->iobase + DAS800_STATUS) & 0x7;
	// set channel to 1
	if(data[0])
		devpriv->do_bits |= (1 << (chan + 4)) & 0xf0;
	// set channel to 0
	else
		devpriv->do_bits &= ~(1 << (chan + 4));

	comedi_spin_lock_irqsave(&devpriv->spinlock, irq_flags);
	outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be control register 1 */
	outb(devpriv->do_bits | CONTROL1_INTE | mux_bits, dev->iobase + DAS800_CONTROL1);
	comedi_spin_unlock_irqrestore(&devpriv->spinlock, irq_flags);

	return 1;
}

/* loads counters with divisor1, divisor2 from private structure */
int das800_set_frequency(comedi_device *dev)
{
	int err = 0;

	if(das800_load_counter(1, devpriv->divisor1, dev)) err++;
	if(das800_load_counter(2, devpriv->divisor2, dev)) err++;
	if(err)
		return -1;

	return 0;
}

int das800_load_counter(unsigned int counterNumber, unsigned int counterValue, comedi_device *dev)
{
	unsigned char byte;

	if(counterNumber > 2) return -1;
	if(counterValue == 1 || counterValue > 0xffff) return -1;

  byte = counterNumber << 6;
	byte = byte | 0x30;	// load low then high byte
	byte = byte | 0x4;	// set counter mode 2
	outb(byte, dev->iobase + 0x7);
	byte = counterValue & 0xff;	// lsb of counter value
	outb(byte, dev->iobase + 0x4 + counterNumber);
	byte = counterValue >> 8;	// msb of counter value
	outb(byte, dev->iobase + 0x4 + counterNumber);
	return 0;
}
