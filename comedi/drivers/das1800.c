/*
    das1800.c driver for Keitley das1800st/hr series boards
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

This driver supports the following Keithley boards:

DAS-1801ST
DAS-1802ST
DAS-1802HR

Options:
	[0] - base io address
	[1] - irq (supports shareable interrupts)

cmd triggers supported:
	start_src: TRIG_NOW
	scan_begin_src: TRIG_FOLLOW
	scan_end_src:   TRIG_COUNT
	convert_src:    TRIG_TIMER
	scan_end_src:   TRIG_COUNT
	stop_src:       TRIG_END | TRIG_COUNT

TODO:
	Support unipolar gains and single ended inputs
	Support more cmd triggers
	Speed up interrupt routine
	Support dma transfers
	Add support for cards' digital i/o
	Add support for cards with analog out

NOTES:
Only the DAS-1801ST has been tested by me.

BUGS:
The DAS-1802ST cards are identified as DAS-1801ST cards, since the
two boards have identical id bits.  The only difference between the
cards are their gains.
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
#include <linux/comedidev.h>
#include <asm/io.h>

#define DAS1800_SIZE           16
#define HALFFIFO               512

/* Registers for the das800 */
#define DAS1800_FIFO            0x0
#define DAS1800_QRAM            0x0
#define DAS1800_SELECT          0x2
#define DAS1800_DIGITAL         0x3
#define DAS1800_CONTROL_A       0x4
#define   FFEN                    0x1
#define   CGEN                    0x4
#define DAS1800_CONTROL_B       0x5
#define   FIMD                    0x40
#define DAS1800_CONTROL_C       0X6
#define   BMDE                    0x4
#define   CMEN                    0x8
#define   UQEN                    0x10
#define   SD                      0x40
#define   UB                      0x80
#define DAS1800_STATUS          0x7
#define   INT                     0x1
#define   OVF                     0x10
#define   FHF                     0x20
#define   FNE                     0x40
#define   CVEN                    0x80
#define DAS1800_QRAM_ADDRESS    0xa
#define DAS1800_COUNTER(a)        (0xc + a)
#define DAS1800_COUNTER_CONTROL 0xf

typedef struct das1800_board_struct{
	char *name;
	int ai_speed;
	int resolution;
}das1800_board;

das1800_board das1800_boards[] =
{
	{
		name:	"DAS-1801ST",
		/* Warning: the maximum conversion speeds listed below are
		 * not always achievable depending on board setup (see
		 * user manual.)
		 */
		ai_speed:	3000,
		resolution:	12,
	},
	{
		name:		"DAS-1802ST",
		ai_speed:	3000,
		resolution:	12,
	},
	{
		name:		"DAS-1802HR",
		ai_speed:	10000,
		resolution:	16,
	},
};
/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((das1800_board *)dev->board_ptr)

typedef struct{
	unsigned long count;  /* number of data points left to be taken */
	int forever;  /* flag indicating whether we should take data forever */
	unsigned short divisor1;	/* value to load into board's counter 1 for timed conversions */
	unsigned short divisor2; 	/* value to load into board's counter 2 for timed conversions */
}das1800_private;

#define devpriv ((das1800_private *)dev->private)

static comedi_lrange range_das1801st_ai = {
	4,
	{
		RANGE( -5, 5 ),
		RANGE( -1, 1 ),
		RANGE( -0.1, 0.1 ),
		RANGE( -0.02, 0.02 ),
/*		RANGE( 0, 5 ),
		RANGE( 0, 1 ),
		RANGE( 0, 0.1 ),
		RANGE( 0, 0.02 ),
*/	}
};

static comedi_lrange range_das1802st_ai = {
	4,
	{
		RANGE(-10, 10),
		RANGE(-5, 5),
		RANGE(-2.5, 2.5),
		RANGE(-1.25, 1.25),
/*		RANGE(0, 10),
		RANGE(0, 5),
		RANGE(0, 2.5),
		RANGE(0, 1.25),
*/	}
};

static comedi_lrange range_das1802hr_ai = {
	4,
	{
		RANGE(-10, 10),
		RANGE(-5, 5),
		RANGE(-2.5, 2.5),
		RANGE(-1.25, 1.25),
/*		RANGE(0, 10),
		RANGE(0, 5),
		RANGE(0, 2.5),
		RANGE(0, 1.25),
*/	}
};

static comedi_lrange *das1800_range_lkup[] = {
	&range_das1801st_ai,
	&range_das1802st_ai,
	&range_das1802hr_ai,
};

static int das1800_attach(comedi_device *dev, comedi_devconfig *it);
static int das1800_detach(comedi_device *dev);
static int das1800_cancel(comedi_device *dev, comedi_subdevice *s);

comedi_driver driver_das1800={
	driver_name:	"das1800",
	module:		THIS_MODULE,
	attach:		das1800_attach,
	detach:		das1800_detach,
};

static void das1800_interrupt(int irq, void *d, struct pt_regs *regs);
void enable_das1800(comedi_device *dev);
void disable_das1800(comedi_device *dev);
static int das1800_ai_do_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd);
static int das1800_ai_do_cmd(comedi_device *dev, comedi_subdevice *s);
static int das1800_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
int das1800_probe(comedi_device *dev);
int das1800_set_frequency( unsigned int period, comedi_device *dev);
int das1800_load_counter(unsigned int counterNumber, int counterValue, comedi_device *dev);
unsigned int das1800_find_divisors(unsigned int period, comedi_device *dev);

/* probes das-1800st/hr series board type - stupid boards have incomplete id
	this isn't going to work */
int das1800_probe(comedi_device *dev)
{
	int id;
	id = inb(dev->iobase + DAS1800_DIGITAL) >> 4; /* get id bits */
	switch(id)
	{
		case 0x7:
			printk(" Board model: DAS-1800ST series\n");
			return 0;
			break;
		case 0x3:
			printk(" Board model: DAS-1800ST-DA series\n");
			return 0;
			break;
		case 0x6:
			printk(" Board model: DAS-1802HR\n");
			return 2;
			break;
		case 0x4:
			printk(" Board model: DAS-1802HR-DA\n");
			return 2;
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
COMEDI_INITCLEANUP(driver_das1800);

static void das1800_interrupt(int irq, void *d, struct pt_regs *regs)
{
	int i;		/* loop index */
	int numPoints = HALFFIFO;	/* number of points to read */
	int ret;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;	/* analog input subdevice */
	sampl_t dpnt;

	ret = inb(dev->iobase + DAS1800_STATUS);
	/* if interrupt was not caused by das-1800 */
	if(!(ret & INT))
		return;
	if(ret & FHF)      /* if fifo half full */
	{
		/* if we only need some of the points */
		if( devpriv->forever == 0 && devpriv->count < numPoints)
			numPoints = devpriv->count;
		for( i = 0; i < numPoints; i++)
		{
			/* write data point to buffer */
			if(s->buf_int_ptr + sizeof(sampl_t) > s->cur_trig.data_len )
			{
				s->buf_int_ptr = 0;
				comedi_eobuf(dev, s);
			}
			dpnt = inw(dev->iobase + DAS1800_FIFO);
			/* convert to offset binary */
			dpnt += 1 << (thisboard->resolution - 1); 
			*((sampl_t *)((void *)s->cur_trig.data + s->buf_int_ptr)) = dpnt;
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
	}
	comedi_bufcheck(dev,s);
	if(ret & OVF)
	{
		comedi_error(dev, "DAS1800 FIFO overflow");
		das1800_cancel(dev, dev->subdevices + 0);
		comedi_error_done(dev, s);
		return;
	}
	/* if there are more data points to collect */
	if(devpriv->count > 0 || devpriv->forever == 1)
		/* Re-enable card's interrupt */
		outb(CVEN, dev->iobase + DAS1800_STATUS);
	/* otherwise, stop taking data */
	else
	{
		disable_das1800(dev);		/* diable hardware triggered conversions */
		comedi_done(dev, s);
	}
	return;
}

static int das1800_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	unsigned char byte;
	int iobase = it->options[0];
	int irq = it->options[1];

	printk("comedi%d: das1800: io 0x%x, irq %i\n", dev->minor, iobase, irq);

	/* check if io addresses are available */
	if(check_region(iobase, DAS1800_SIZE) < 0)
	{
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase, DAS1800_SIZE, "das1800");
	dev->iobase = iobase;
	dev->iosize = DAS1800_SIZE;

	/* grab our IRQ */
	byte = FIMD;	/* interrupt on half full fifo */
	switch(irq)
	{
		case 0:
			break;
		case 3:
			byte |= 0x8;
			break;
		case 5:
			byte |= 0x10;
			break;
		case 7:
			byte |= 0x18;
			break;
		case 10:
			byte |= 0x28;
			break;
		case 11:
			byte |= 0x30;
			break;
		case 15:
			byte |= 0x38;
			break;
		default:
			printk("irq out of range\n");
			return -EINVAL;
			break;
	}
	outb(byte, dev->iobase + DAS1800_CONTROL_B); // tell board what irq to use

	if(irq)
	{
		if(comedi_request_irq( irq, das1800_interrupt, SA_SHIRQ, "das1800", dev ))
		{
			printk( "unable to allocate irq %d\n", irq);
			return -EINVAL;
		}
	}
	dev->irq = irq;

	dev->board = das1800_probe(dev);
	if(dev->board < 0)
	{
		printk("unable to determine board type\n");
		return -ENODEV;
	}

	dev->board_ptr = das1800_boards + dev->board;
	dev->board_name = thisboard->name;

	/* allocate and initialize dev->private */
	if(alloc_private(dev, sizeof(das1800_private)) < 0)
		return -ENOMEM;
	devpriv->count = 0;
	devpriv->forever = 0;

	dev->n_subdevices = 1;
	if(alloc_subdevices(dev) < 0)
		return -ENOMEM;

	/* analog input subdevice */
	s = dev->subdevices + 0;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_DIFF;
	s->n_chan = 256;
	s->len_chanlist = 256;
	s->maxdata = (1 << thisboard->resolution) - 1;
	s->range_table = das1800_range_lkup[dev->board];
	s->do_cmd = das1800_ai_do_cmd;
	s->do_cmdtest = das1800_ai_do_cmdtest;
	s->insn_read = das1800_ai_rinsn;
	s->cancel = das1800_cancel;

	return 0;
};

static int das1800_detach(comedi_device *dev)
{
	printk("comedi%d: das1800: remove\n", dev->minor);

	/* only free stuff if it has been allocated by _attach */
	if(dev->iobase)
	{
		release_region(dev->iobase, DAS1800_SIZE);
		dev->iobase = 0;
	}
	if(dev->irq)
	{
		comedi_free_irq(dev->irq, dev);
		dev->irq = 0;
	}
	return 0;
};

static int das1800_cancel(comedi_device *dev, comedi_subdevice *s)
{
	devpriv->forever = 0;
	devpriv->count = 0;
	disable_das1800(dev);
	return 0;
}

/* enable_das1800 makes the card start taking hardware triggered conversions */
void enable_das1800(comedi_device *dev)
{
	outb(CGEN | FFEN, dev->iobase + DAS1800_CONTROL_A);	/* enable fifo and hardware triggering */
	outb(CVEN, dev->iobase + DAS1800_STATUS);	/* enable conversions */
}

/* disable_das1800 stops hardware triggered conversions */
void disable_das1800(comedi_device *dev)
{
	outb(0x0, dev->iobase + DAS1800_STATUS);	/* disable conversions */
	outb(0x0, dev->iobase + DAS1800_CONTROL_A);	/* disable and clear fifo and stop triggering */
}

static int das1800_ai_do_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err = 0;
	int tmp;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src && tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW;
	if(!cmd->scan_begin_src && tmp != cmd->scan_begin_src) err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER;
	if(!cmd->convert_src && tmp != cmd->convert_src) err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src && tmp != cmd->scan_end_src) err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if(!cmd->stop_src && tmp!=cmd->stop_src) err++;

	if(err) return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->start_src != TRIG_NOW) err++;
	if(cmd->scan_begin_src != TRIG_FOLLOW) err++;
	if(cmd->convert_src != TRIG_TIMER) err++;
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
		cmd->convert_arg = das1800_find_divisors(cmd->convert_arg, dev);
		if(tmp != cmd->convert_arg) err++;
	}

	if(err)return 4;

	return 0;
}

static int das1800_ai_do_cmd(comedi_device *dev, comedi_subdevice *s)
{
	int i, n, chan_range;

	if(!dev->irq)
	{
		comedi_error(dev, "no irq assigned for das-1800, cannot do hardware conversions");
		return -1;
	}

	disable_das1800(dev);

	n = s->cmd.chanlist_len;
	outb(0x1, dev->iobase + DAS1800_SELECT); /* select QRAM for baseAddress + 0x0 */
	outb(n - 1, dev->iobase + DAS1800_QRAM_ADDRESS);	/*set QRAM address start */
	for(i = 0; i < n; i++)	/* make channel / gain list */
	{
		chan_range = CR_CHAN(s->cmd.chanlist[i]) | (CR_RANGE(s->cmd.chanlist[i]) << 8);
		outw(chan_range, dev->iobase + DAS1800_QRAM);
	}
	outb(n - 1, dev->iobase + DAS1800_QRAM_ADDRESS);	/*finish write to QRAM */
	outb(0x0, dev->iobase + DAS1800_SELECT);	/* select ADC for baseAddress + 0x0 */

	switch(s->cmd.stop_src)
	{
		case TRIG_COUNT:
			devpriv->count = s->cmd.stop_arg;
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
	switch(s->cmd.convert_src)
	{
		case TRIG_TIMER:
			/* differential, bipolar, pacer clocks are clocks 1 and 2 */
			outb(UQEN | 0x1, dev->iobase + DAS1800_CONTROL_C);
			/* set conversion frequency */
			if(das1800_set_frequency(s->cmd.convert_arg, dev) < 0)
			{
				comedi_error(dev, "Error setting up counters");
				return -1;
			}
			break;
/*		case TRIG_EXT:
			break;
*/		default:
			break;
	}

	enable_das1800(dev);
	return 0;
}

static int das1800_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
 	int i, n;
	int chan, range, chan_range;
	int timeout = 1000;
	short dpnt;

	outb(UQEN, dev->iobase + DAS1800_CONTROL_C);	/* software conversion enabled */
	outb(CVEN, dev->iobase + DAS1800_STATUS);	/* enable conversions */
	outb(0x0, dev->iobase + DAS1800_CONTROL_A);	/* reset fifo */
	outb(FFEN, dev->iobase + DAS1800_CONTROL_A);


	chan = CR_CHAN(insn->chanspec) & 0xff;
	range = CR_RANGE(insn->chanspec) & 0x3;
	chan_range = chan | (range << 8);
	outb(0x1, dev->iobase + DAS1800_SELECT);	/* select QRAM for baseAddress + 0x0 */
	outb(0x0, dev->iobase + DAS1800_QRAM_ADDRESS);	/* set QRAM address start */
	outw(chan_range, dev->iobase + DAS1800_QRAM);
	outb(0x0, dev->iobase + DAS1800_QRAM_ADDRESS);	/*finish write to QRAM */
	outb(0x0, dev->iobase + DAS1800_SELECT);	/* select ADC for baseAddress + 0x0 */

	udelay(5);

	for(n = 0; n < insn->n; n++)
	{
		/* trigger conversion */
		outb(0, dev->iobase + DAS1800_FIFO);
		for(i = 0; i < timeout; i++)
		{
			if(inb(dev->iobase + DAS1800_STATUS) & FNE)
				break;
		}
		if(i == timeout)
		{
			comedi_error(dev, "timeout");
			return -ETIME;
		}
		/* shift data from true binary to offset binary */
		dpnt = inw(dev->iobase + DAS1800_FIFO);
		dpnt += 1 << (thisboard->resolution - 1);
		data[n] = dpnt;
	}

	return n;
}


/* finds best values for cascaded counters to obtain desired frequency,
 * and loads counters with calculated values
 */
int das1800_set_frequency(unsigned int period, comedi_device *dev)
{
	int err = 0;

	das1800_find_divisors(period, dev);
	if(das1800_load_counter(1, devpriv->divisor1, dev)) err++;
	if(das1800_load_counter(2, devpriv->divisor2, dev)) err++;
	if(err)
		return -1;

	return 0;
}

int das1800_load_counter(unsigned int counterNumber, int counterValue, comedi_device *dev)
{
	unsigned char byte;

	if(counterNumber > 2) return -1;
	if(counterValue < 2 || counterValue > 0xffff) return -1;

  byte = counterNumber << 6;
	byte = byte | 0x30;	// load low then high byte
	byte = byte | 0x4;	// set counter mode 2
	outb(byte, dev->iobase + DAS1800_COUNTER_CONTROL);
	byte = counterValue & 0xff;	// lsb of counter value
	outb(byte, dev->iobase + DAS1800_COUNTER(counterNumber));
	byte = counterValue >> 8;	// msb of counter value
	outb(byte, dev->iobase + DAS1800_COUNTER(counterNumber));
	return 0;
}

unsigned int das1800_find_divisors(unsigned int period, comedi_device *dev)
{
	int clock = 200;	// 5 MHz master clock
	int temp, close;
	unsigned short i, j;
	unsigned int max, min;

	if((devpriv->divisor1 * devpriv->divisor2 * clock) == period) return period;

	max = (period / (2 * clock));
	if(max > 0xffff)
		max = 0xffff;
	min = 2;
	close = period;
	for(i = min; i <= max; i++)
	{
		for(j = (period / (i * clock)); j <= (period / (i * clock)) + 1; j++)
		{
			temp = period - clock * i * j;
			if(temp < 0) temp = -temp;
			if(temp < close && j >= min)
			{
				close = temp;
				devpriv->divisor1 = i; devpriv->divisor2 = j;
				if(close == 0) return period;
			}
		}
	}
	return devpriv->divisor1 * devpriv->divisor2 * clock;
}
