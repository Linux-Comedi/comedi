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

This driver supports the Keithley DAS-800, DAS-801, and DAS-802
boards.  It will also work with Measurement Computing (Computer Boards)
boards CIO-DAS800, CIO-DAS801, and CIO-DAS802 boards although the
CIO-DAS801 has a slightly different gain list than the DAS-801.

Options:
	[0] - base io address
	[1] - irq

cmd triggers supported:
	start_src: TRIG_NOW
	scan_begin_src: TRIG_FOLLOW
	scan_end_src:   TRIG_COUNT
	convert_src:    TRIG_TIMER | TRIG_EXT
	scan_end_src:   TRIG_COUNT
	stop_src:       TRIG_END | TRIG_COUNT

The number of channels scanned is determined from chanlist_len.  The
starting channel to scan and the gain is determined from chanlist[0].

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
#include <comedi_module.h>

#define DAS800_SIZE           8
/* Registers for the das800 */

#define DAS800_LSB    0
#define   FIFO_EMPTY            0x1
#define   FIFO_OVF              0x2
#define DAS800_MSB    1
#define DAS800_CONTROL1       2
#define   CONRTOL1_INTE         0x8
#define DAS800_CONV_CONTROL   2
#define   ITE                   0x1
#define   CASC                  0x2
#define   IEOC                  0x8
#define   EACS                  0x10
#define   CONV_HCEN             0x80
#define DAS800_SCAN_LIMITS    2
#define DAS800_STATUS         2
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

das800_board das800_boards[] =
{
	{
		name:	"DAS-800",
		ai_speed:	25000,	/* 40kHz maximum conversion rate (25 microseconds) */
	},
	{
		name:		"DAS-801",
		ai_speed:	25000,
	},
	{
		name:		"DAS-802",
		ai_speed:	25000,
	},
};
/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((das800_board *)dev->board_ptr)

typedef struct{
	unsigned long count;  /* number of data points left to be taken */
	int forever;  /* flag indicating whether we should take data forever */
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
	&range_das801_ai,
	&range_das802_ai,
};

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int das800_attach(comedi_device *dev,comedi_devconfig *it);
static int das800_detach(comedi_device *dev);
static int das800_cancel(comedi_device *dev, comedi_subdevice *s);

comedi_driver driver_das800={
	driver_name:	"das800",
	module:		THIS_MODULE,
	attach:		das800_attach,
	detach:		das800_detach,
};

static void das800_interrupt(int irq, void *d, struct pt_regs *regs);
void enableDAS800(comedi_device *dev);
void disableDAS800(comedi_device *dev);
static int das800_ai_do_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd);
static int das800_ai_do_cmd(comedi_device *dev, comedi_subdevice *s);
static int das800_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
int das800_probe(comedi_device *dev);
int das800_set_frequency( unsigned int period, comedi_device *dev);
int das800_load_counter(unsigned int counterNumber, int counterValue, comedi_device *dev);

int das800_probe(comedi_device *dev)
{
	int id;
	outb(ID, dev->iobase + DAS800_GAIN);	/* select base address + 7 to be ID register */
	id = inb(dev->iobase + DAS800_ID) & 0x3;
	switch(id)
	{
		case 0x0:
			printk(" Board model: DAS-800\n");
			return 0;
			break;
		case 0x2:
			printk(" Board model: DAS-801\n");
			return 1;
			break;
		case 0x3:
			printk(" Board model: DAS-802\n");
			return 2;
			break;
		default :
			printk(" Board model: probe returned 0x%x (unknown)\n", id);
			return 0;
			break;
	}
	return -1;
}

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_das800);

static void das800_interrupt(int irq, void *d, struct pt_regs *regs)
{
	short i;		/* loop index */
	sampl_t dataPoint;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;	/* analog input subdevice */

  /* read 16 bits from dev->iobase and dev->iobase + 1 */
	dataPoint = inb(dev->iobase + DAS800_LSB);
	dataPoint += inb(dev->iobase + DAS800_MSB) << 8;
	/* loop while card's fifo is not empty (and make sure loop terminates by limiting to 256 iterations) */
	for(i = 0; (dataPoint & FIFO_EMPTY) == 0 && i < 256; i++)
	{
		if( dataPoint & FIFO_OVF )
		{
			comedi_error(dev, "DAS800 FIFO overflow");
			comedi_error_done(dev, s);
			devpriv->forever = 0;
			devpriv->count = 0;
			disableDAS800(dev);
			return;
		}
		dataPoint = (dataPoint >> 4) & 0xfff;		/* strip off extraneous bits */
		if(devpriv->count > 0 || devpriv->forever == 1)
		{
			/* write data point to buffer */
			if(s->buf_int_ptr >= s->cur_trig.data_len )
			{
				s->buf_int_ptr=0;
				comedi_eobuf(dev,s);
			}
			*((sampl_t *)((void *)s->cur_trig.data + s->buf_int_ptr)) = dataPoint;
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
	comedi_bufcheck(dev,s);
	if(devpriv->count > 0 || devpriv->forever == 1)
		outb(CONRTOL1_INTE, dev->iobase + DAS800_CONTROL1);	/* re-enable card's interrupt */
	/* otherwise, stop taking data */
	else
	{
		disableDAS800(dev);		/* diable hardware triggered conversions */
		comedi_done(dev, s);
	}
};

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  _recognize() has already been called,
 * and dev->board contains whatever _recognize returned.
 */
static int das800_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int iobase = it->options[0];
	int irq = it->options[1];

	printk("comedi%d: das800: io 0x%x, irq %i\n",dev->minor, iobase, irq);

	/* check if io addresses are available */
	if (check_region(iobase, DAS800_SIZE) < 0)
	{
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase, DAS800_SIZE, "das800");
	dev->iobase = iobase;
	dev->iosize = DAS800_SIZE;

	/* grab our IRQ */
	if(irq < 2 || irq > 7)
	{
		printk("irq out of range\n");
		return -EINVAL;
	}
	if( comedi_request_irq( irq, das800_interrupt, 0, "das800", dev ))
	{
		printk( "unable to allocate irq %d\n", irq);
		return -EINVAL;
	}
	dev->irq = irq;

	dev->board = das800_probe(dev);

/*
 * Initialize dev->board_ptr.  This can point to an element in the
 * das800_boards array, for quick access to board-specific information.
 */
	dev->board_ptr = das800_boards + dev->board;

/*
 * Initialize dev->board_name.  Note that we can use the "thisboard"
 * macro now, since we just initialized it in the last line.
 */
	dev->board_name = thisboard->name;

/*
 * Allocate the private structure area.
 */
	if(alloc_private(dev,sizeof(das800_private))<0)
		return -ENOMEM;

/*
 * Allocate the subdevice structures.
 */
	dev->n_subdevices = 1;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	/* analog input subdevice */
	s = dev->subdevices + 0;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 8;
	s->len_chanlist = 8;
	s->maxdata=(1 << 12) - 1;
	s->range_table = das800_range_lkup[dev->board];
	s->do_cmd = das800_ai_do_cmd;
	s->do_cmdtest = das800_ai_do_cmdtest;
	s->insn_read = das800_ai_rinsn;
	s->cancel = das800_cancel;
	return 0;
};

static int das800_detach(comedi_device *dev)
{

	printk("comedi%d: das800: remove\n", dev->minor);

	/* only free stuff if it has been allocated by _attach */
	if(dev->iobase)
		release_region(dev->iobase, DAS800_SIZE);
	if(dev->irq)
		comedi_free_irq(dev->irq,dev);
	return 0;
};

static int das800_cancel(comedi_device *dev, comedi_subdevice *s)
{
	devpriv->forever = 0;
	devpriv->count = 0;
	disableDAS800(dev);
	return 0;
}

/* enableDAS800 makes the card start taking hardware triggered conversions */
void enableDAS800(comedi_device *dev)
{
	outb(CONV_CONTROL, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be conversion control register */
	outb(CONV_HCEN, dev->iobase + DAS800_CONV_CONTROL);	/* enable hardware triggering */
	outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be control register 1 */
	outb(CONRTOL1_INTE, dev->iobase + DAS800_CONTROL1);	/* enable card's interrupt */
}

/* disableDAS800 stops hardware triggered conversions */
void disableDAS800(comedi_device *dev)
{
	outb(CONV_CONTROL, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be conversion control register */
	outb(0x0, dev->iobase + DAS800_CONV_CONTROL);	/* disable hardware triggering of conversions */
}

static int das800_ai_do_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err = 0;
	int tmp, chan, range;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW;
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

	if(cmd->scan_begin_src != TRIG_FOLLOW) err++;
	if(cmd->convert_src != TRIG_TIMER &&
	   cmd->convert_src != TRIG_EXT) err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0)
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
		cmd->chanlist_len=1;
		err++;
	}
	if(cmd->chanlist_len > 8)
	{
		cmd->chanlist_len = 8;
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

	chan = CR_CHAN(cmd->chanlist[0]);
	range = CR_RANGE(cmd->chanlist[0]);
	if(chan >= 8)
	{
		cmd->chanlist[0] = CR_PACK(chan % 8, range, 0);
		err++;
	}
	if(cmd->convert_src == TRIG_TIMER)
	{
		tmp = cmd->convert_arg;
/* todo: add code to figure out what actual convert_arg card will use
		and store it in tmp

		if(tmp != cmd->convert_arg) err++;
*/
	}

	if(err)return 4;

	return 0;
}

static int das800_ai_do_cmd(comedi_device *dev, comedi_subdevice *s)
{
	int startChan, endChan, scan, gain;

	disableDAS800(dev);

	/* set channel scan limits */
	outb(SCAN_LIMITS, dev->iobase + DAS800_GAIN);	/* select base address + 2 to be scan limits register */
	startChan = CR_CHAN(s->cmd.chanlist[0]);
	endChan = (startChan + s->cmd.chanlist_len - 1) % 8;
	scan = (endChan << 3) | startChan;
	outb(scan, dev->iobase + DAS800_SCAN_LIMITS); /* set scan limits */

	/* set gain */
	gain = CR_RANGE(s->cmd.chanlist[0]);
	if( gain > 0)
		gain += 0x7;
	gain &= 0xf;
	outb(gain, dev->iobase + DAS800_GAIN);

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
			outb(CONV_CONTROL, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be conversion control register */
			outb(EACS | IEOC | CASC| ITE, dev->iobase + DAS800_CONV_CONTROL);
			/* set conversion frequency */
			if( das800_set_frequency( s->cmd.convert_arg, dev) < 0)
			{
				comedi_error(dev, "Error setting up counters");
				return -1;
			}
			break;
		case TRIG_EXT:
			outb(CONV_CONTROL, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be conversion control register */
			outb(EACS | IEOC, dev->iobase + DAS800_CONV_CONTROL);
			break;
		default:
			break;
	}

	enableDAS800(dev);
	return 0;
}

static int das800_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
 	int i,n;
	int chan;
	int range;
	int lsb,msb;

	disableDAS800(dev);

	/* set multiplexer */
	chan = CR_CHAN(insn->chanspec);
	outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be control register 1 */
	outb(chan & 0x7, dev->iobase + DAS800_CONTROL1);

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
		outb(0, dev->iobase);
		udelay(25);
#define TIMEOUT 1000
		for(i = 0; i < TIMEOUT; i++)
		{
			if(!(inb(DAS800_STATUS) & BUSY))
				break;
		}
		if(i == TIMEOUT)
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


/* finds best values for cascaded counters to obtain desired frequency,
 * and loads counters with calculated values
 */
int das800_set_frequency( unsigned int period, comedi_device *dev)
{
	int clock = 1000;
	int countA = 0;
	int countB = 0;
	int temp, close;
	unsigned int i, j, max, min;

	max = (unsigned int) (period / (2 * clock));
	if(max > (1 << 16))
		max = (1 << 16);
	min = 2;
	close = period;
	for(i = min; i <= max; i++)
	{
		for(j = (period / (i * clock)); j <= (period / (i * clock)) + 1; j++)
		{
			temp = period - clock * i * j;
			if(temp < 0) temp = -temp;
			if(temp < close)
			{
				close = temp;
				countA = i; countB = j;
			}
		}
	}

	countA = das800_load_counter(1, countA, dev);
	countB = das800_load_counter(2, countB, dev);
	if(countA < 0 || countB < 0)
		return -1;

	return clock * countA * countB;
}

int das800_load_counter(unsigned int counterNumber, int counterValue, comedi_device *dev)
{
	unsigned char byte;

	if(counterNumber > 2) return -1;
	if(counterValue < 2 || counterValue > 0xffff) return -1;

  byte = counterNumber << 6;
	byte = byte | 0x30;	// load low then high byte
	byte = byte | 0x4;	// set counter mode
	outb(byte, dev->iobase + 0x7);
	byte = counterValue & 0xff;	// lsb of counter value
	outb(byte, dev->iobase + 0x4 + counterNumber);
	byte = counterValue >> 8;	// msb of counter value
	outb(byte, dev->iobase + 0x4 + counterNumber);
	return counterValue;
}
