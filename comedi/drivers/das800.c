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
#define TIMER_BASE            1000
/* Registers for the das800 */

#define DAS800_AI_LSB_READ    0
#define   FIFO_EMPTY            0x1
#define   FIFO_OVF              0x2
#define DAS800_AI_MSB_READ    1
#define DAS800_CONTROL1       2
#define   CONRTOL1_INTE         0x8
#define DAS800_CONV_CONTROL   2
#define   ITE                   0x1
#define   CASC                  0x2
#define   IEOC                  0x8
#define   EACS                  0x10
#define   CONV_HCEN             0x80
#define DAS800_SCAN_LIMITS    2
#define DAS800_GAIN           3
#define   CONTROL1              0x80
#define   CONV_CONTROL          0xa0
#define   SCAN_LIMITS           0xc0
#define   ID                    0xe0
#define DAS800_STATUS2        7
#define   STATUS2_HCEN          0x80
#define   STATUS2_INTE          0X20
#define DAS800_ID             7

/*
 * Board descriptions for two imaginary boards.  Describing the
 * boards in this way is optional, and completely driver-dependent.
 * Some drivers use arrays such as this, other do not.
 */
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

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct{
	unsigned long count;  /* number of data points left to be taken */
	int forever;  /* flag indicating whether we should take data forever */
}das800_private;
/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
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
static int das800_recognize(char *name);

comedi_driver driver_das800={
	driver_name:	"das800",
	module:		THIS_MODULE,
	attach:		das800_attach,
	detach:		das800_detach,
	recognize:	das800_recognize,
};

/* static int das800_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
 * static int das800_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
 */

unsigned int das800_interrupt(unsigned int irq, void *d, struct pt_regs *regs);
void enableDAS800(comedi_device *dev);
void disableDAS800(comedi_device *dev);
static int das800_do_cmd(comedi_device *dev, comedi_subdevice *s);
int das800_probe(comedi_device *dev);
int das800SetFrequency( unsigned int period, comedi_device *dev);
int das800LoadCounter(unsigned int counterNumber, int counterValue, comedi_device *dev);

/*
 * The function das800_recognize() is called when the Comedi core
 * gets a request to configure a device.  If the name of the device
 * being configured matches with one of the devices that this
 * driver can service, then a non-negative index should be returned.
 * This index is put into dev->board, and then _attach() is called.
 */
static int das800_recognize(char *name)
{
	if(!strcmp("das800", name)) return 0;
	if(!strcmp("das801", name)) return 1;
  if(!strcmp("das802", name)) return 2;

	return -1;
}

int das800_probe(comedi_device *dev)
{
	int id;
	outb(ID, dev->iobase + DAS800_GAIN);	/* select base address + 7 to be ID register */
	id = inb(DAS800_ID);
	outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select base address +7 to be status register 2 */
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
			printk(" Board model: probe failed\n");
			return dev->board;
			break;
	}
	return -1;
}

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_das800);

unsigned int das800_interrupt(unsigned int irq, void *d, struct pt_regs *regs)
{
	short i;		/* loop index */
  sampl_t dataPoint;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;	/* analog input subdevice */

  /* read 16 bits from dev->iobase and dev->iobase + 1 */
  dataPoint = inb(dev->iobase + DAS800_AI_LSB_READ);
  dataPoint += inb(dev->iobase + DAS800_AI_MSB_READ) << 8;
	/* loop while card's fifo is not empty (and make sure loop terminates by limiting to 10 iterations) */
	for(i = 0; (dataPoint & FIFO_EMPTY) == 0 && i < 10; i++)
	{
		if( dataPoint & FIFO_OVF )
		{
      comedi_error(dev, "DAS800 FIFO overflow");
			comedi_error_done(dev, s);
			return 1;
		}
		dataPoint = (dataPoint >> 4) & 0xfff;		/* strip off extraneous bits */
		if(devpriv->count > 0 || devpriv->forever == 1)
		{
			/* write data point to buffer */
			if(s->buf_int_ptr + sizeof(sampl_t) >= s->cur_trig.data_len)
			{
				s->buf_int_ptr=0;
				comedi_eobuf(dev,s);
				break;
			}
			*(s->cur_trig.data + s->buf_int_ptr) = dataPoint;
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
	  dataPoint = inb(dev->iobase + DAS800_AI_LSB_READ);
	  dataPoint += inb(dev->iobase + DAS800_AI_MSB_READ) << 8;
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
	return 0;
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

	printk("comedi%d: das800: io 0x%x, irq %i",dev->minor, iobase, irq);

	/* check if io addresses are available */
	if (check_region(iobase, DAS800_SIZE) < 0)
	{
		printk("I/O port conflict\n");
		return -EIO;
	}
	dev->iobase = iobase;
	dev->iosize = DAS800_SIZE;

	/* grab our IRQ */
	if(irq < 2 || irq > 7)
	{
		printk("irq out of range\n");
		return -EINVAL;
	}
	if( comedi_request_irq( irq, das800_interrupt, SA_INTERRUPT, "das800", dev ))
	{
		printk( "unable to allocate irq %d\n", irq);
		return -EINVAL;
	}
	dev->irq = irq;

	/* wait until after irq check before allocating io addresses */
	request_region(dev->iobase, DAS800_SIZE, "das800");

/*
 * If you can probe the device to determine what device in a series
 * it is, this is the place to do it.  Otherwise, you should use the
 * _recognize method, and use the value in dev->board.
 */
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
	s->maxdata=(1 << 12) - 1;
	s->range_table = das800_range_lkup[dev->board];
	s->do_cmd = &das800_do_cmd;

	return 1;
};

/*
 * _detach is called to deconfigure a device.  It should deallocate
 * resources.
 * This function is also called when _attach() fails, so it should be
 * careful not to release resources that were not necessarily
 * allocated by _attach().  dev->private and dev->subdevices are
 * deallocated automatically by the core.
 */
static int das800_detach(comedi_device *dev)
{

	printk("comedi%d: das800: remove\n", dev->minor);

	release_region(dev->iobase, DAS800_SIZE);		/* free general status and control registers' i/o ports */
	comedi_free_irq(dev->irq);		/* free irq */
	return 0;
};

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
	outb(0x0, dev->iobase + DAS800_CONTROL1);	/* disable card's interrupt */
	outb(CONV_CONTROL, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be conversion control register */
	outb(0x0, dev->iobase + DAS800_CONV_CONTROL);	/* disable hardware triggering of conversions */
	outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be control register 1 */
}

static int das800_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err = 0;

	/* check for supported triggers */
	if(cmd->start_src != TRIG_NOW) err++;
	if(cmd->convert_src != TRIG_TIMER && cmd->convert_src != TRIG_EXT) err++;
	if(cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE) err++;
	if(err) return 1;

	/* check arguments */
	if(cmd->convert_src == TRIG_TIMER)
	{
		if(cmd->convert_arg < thisboard->ai_speed)
		{
			cmd->convert_arg = thisboard->ai_speed;
			err++;
		}
	}
	if(cmd->stop_src == TRIG_COUNT)
	{
		if(cmd->stop_arg > 0x00ffffff)
		{
			cmd->stop_arg = 0x00ffffff;
			err++;
		}
	}
	if(cmd->chanlist_len <= 0 || cmd->chanlist_len >8)
	{
		cmd->chanlist_len = 1;
		err++;
	}
	if(err)return 3;

	return 0;
}

static int das800_do_cmd(comedi_device *dev, comedi_subdevice *s)
{
	int gain;
	unsigned int numChannels = s->cmd.chanlist_len;

	/* if cmd is for analog input subdevice */
  if(s == dev->subdevices + 0)
  {
		disableDAS800(dev);

		/* set channel scan limits */
		outb(SCAN_LIMITS, dev->iobase + DAS800_GAIN);	/* select base address + 2 to be scan limits register */
		outb((numChannels - 1) << 3, dev->iobase + DAS800_SCAN_LIMITS); /* set scan limits */
		outb(CONTROL1, dev->iobase + DAS800_GAIN);	/* select dev->iobase + 2 to be control register 1 */

		/* set gain */
		gain = s->cmd.chanlist[0];
		if( gain > 0)
			gain += 0x7;
		gain = gain & 0xf;
		outb(gain, DAS800_GAIN);


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
				outb(EACS | IEOC | CASC| ITE, dev->iobase + DAS800_CONV_CONTROL);
				/* set conversion frequency */
				if( das800SetFrequency( s->cmd.convert_arg, dev) < 0)
				{
					comedi_error(dev, "Error setting up counters");
					return -1;
				}
				break;
			case TRIG_EXT:
				outb(EACS | IEOC, dev->iobase + DAS800_CONV_CONTROL);
				break;
			default:
				break;
		}
	}

	enableDAS800(dev);
	return 0;
}

/* finds best values for cascaded counters to obtain desired frequency,
 * and loads counters with calculated values
 */
int das800SetFrequency( unsigned int period, comedi_device *dev)
{
	int clock = 1000;
	int countA = 0;
	int countB = 0;
	int temp, close;
	unsigned int i, j, max, start;

	max = (unsigned int) (period / (2 * clock)) + 1;
	start  = 1;
	close = period;
	for(i = start; i <= max; i++)
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

	countA = das800LoadCounter(1, countA, dev);
	countB = das800LoadCounter(2, countB, dev);
	if(countA < 0 || countB < 0)
		return -1;

	return clock * countA * countB;
}

int das800LoadCounter(unsigned int counterNumber, int counterValue, comedi_device *dev)
{
	unsigned char byte;

	if(counterNumber > 2) return -1;
	if(counterValue < 1 || counterValue > 0xffff) return -1;

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
