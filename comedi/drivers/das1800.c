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

das-1701st/ao
das-1702st/hr/ao
das-1801st/hc/ao
das-1802st/hr/hc/ao

Options:
	[0] - base io address
	[1] - irq (optional, required for timed or externally triggered conversions)
	[2] - dma0 (optional)
	[3] - dma1 (optional)

irq can be omitted, although the cmd interface will not work without it.

analog input cmd triggers supported:
	start_src:      TRIG_NOW | TRIG_EXT
	scan_begin_src: TRIG_FOLLOW
	scan_end_src:   TRIG_COUNT
	convert_src:    TRIG_TIMER | TRIG_EXT
	scan_end_src:   TRIG_COUNT
	stop_src:       TRIG_COUNT | TRIG_NONE

TODO:
	Support TRIG_TIMER and TRIG_EXT for scan_begin_src (burst mode)
	Add support for cards with analog out
	Support waveform out for 'ao' cards

NOTES:
Only the DAS-1801ST has been tested by me.
Unipolar and bipolar ranges cannot be mixed in the channel/gain list.

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

#define DAS1800_SIZE           16	//uses 16 io addresses
#define HALF_FIFO               512	// 1024 sample fifo
#define TIMER_BASE             200	// 5 Mhz master clock

/* Registers for the das1800 */
#define DAS1800_FIFO            0x0
#define DAS1800_QRAM            0x0
#define DAS1800_SELECT          0x2
#define DAS1800_DIGITAL         0x3
#define DAS1800_CONTROL_A       0x4
#define   FFEN                    0x1
#define   CGEN                    0x4
#define   CGSL                    0x8
#define   TGEN                    0x10
#define   TGSL                    0x20
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
#define   DMATC                   0x2
#define   OVF                     0x10
#define   FHF                     0x20
#define   FNE                     0x40
#define   CVEN                    0x80
#define DAS1800_QRAM_ADDRESS    0xa
#define DAS1800_COUNTER(a)        (0xc + a)
#define DAS1800_COUNTER_CONTROL 0xf

enum{das1701st, das1702st, das1702hr, das1701ao, das1702ao, das1801st, das1802st, das1802hr, das1801hc, das1802hc, das1801ao, das1802ao};

typedef struct das1800_board_struct{
	char *name;
	int ai_speed;	/* max conversion period in nanoseconds */
	int resolution;	/* bits of ai resolution */
	int qram_len;	/* length of card's channel / gain queue */
	int common;	/* supports AREF_COMMON flag */
	int do_n_chan;	/* number of digital output channels */
}das1800_board;

das1800_board das1800_boards[] =
{
	{
		name:	"DAS-1701ST",
		/* Warning: the maximum conversion speeds listed below are
		 * not always achievable depending on board setup (see
		 * user manual.)
		 */
		ai_speed:	6250,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
	},
	{
		name:		"DAS-1702ST",
		ai_speed:	6250,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
	},
	{
		name:		"DAS-1702HR",
		ai_speed:	20000,
		resolution:	16,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
	},
	{
		name:	"DAS-1701AO",
		ai_speed:	6250,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
	},
	{
		name:		"DAS-1702AO",
		ai_speed:	6250,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
	},
	{
		name:	"DAS-1801ST",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
	},
	{
		name:		"DAS-1802ST",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
	},
	{
		name:		"DAS-1802HR",
		ai_speed:	10000,
		resolution:	16,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
	},
	{
		name:		"DAS-1801HC",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	64,
		common:	0,
		do_n_chan:	8,
	},
	{
		name:		"DAS-1802HC",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	64,
		common:	0,
		do_n_chan:	8,
	},
	{
		name:	"DAS-1801AO",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
	},
	{
		name:		"DAS-1802AO",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
	},
};
/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((das1800_board *)dev->board_ptr)

typedef struct{
	unsigned long count;  /* number of data points left to be taken */
	int forever;  /* flag indicating whether we should take data forever */
	unsigned int divisor1;	/* value to load into board's counter 1 for timed conversions */
	unsigned int divisor2; 	/* value to load into board's counter 2 for timed conversions */
	int do_bits;	/* digital output bits */
	int irq_dma_bits;	/* bits for control register b */
	unsigned int dma0;	/* dma channels used */
	unsigned int dma1;
	unsigned int dma_current;	/* dma channel currently in use */
	short *dma0_buf;	/* pointers to dma buffers */
	short *dma1_buf;
	short *dma_current_buf;	/* pointer to dma buffer currently being used */
	unsigned int dma_buf_size;	/* size in bytes of dma buffers */
	int dual_dma;	/* flag that indicates whether we have dual dma */
}das1800_private;

#define devpriv ((das1800_private *)dev->private)

static comedi_lrange range_das1801_ai = {
	8,
	{
		RANGE( -5, 5 ),
		RANGE( -1, 1 ),
		RANGE( -0.1, 0.1 ),
		RANGE( -0.02, 0.02 ),
		RANGE( 0, 5 ),
		RANGE( 0, 1 ),
		RANGE( 0, 0.1 ),
		RANGE( 0, 0.02 ),
	}
};

static comedi_lrange range_das1802_ai = {
	8,
	{
		RANGE(-10, 10),
		RANGE(-5, 5),
		RANGE(-2.5, 2.5),
		RANGE(-1.25, 1.25),
		RANGE(0, 10),
		RANGE(0, 5),
		RANGE(0, 2.5),
		RANGE(0, 1.25),
	}
};

static comedi_lrange *das1800_range_lkup[] = {
	&range_das1801_ai,
	&range_das1802_ai,
	&range_das1802_ai,
	&range_das1801_ai,
	&range_das1802_ai,
	&range_das1801_ai,
	&range_das1802_ai,
	&range_das1801_ai,
	&range_das1802_ai,
	&range_das1802_ai,
	&range_das1801_ai,
	&range_das1802_ai,
};

static int das1800_attach(comedi_device *dev, comedi_devconfig *it);
static int das1800_detach(comedi_device *dev);
static int das1800_recognize(char *name);
static int das1800_cancel(comedi_device *dev, comedi_subdevice *s);

comedi_driver driver_das1800={
	driver_name:	"das1800",
	module:		THIS_MODULE,
	attach:		das1800_attach,
	detach:		das1800_detach,
	recognize:		das1800_recognize,
};

static void das1800_interrupt(int irq, void *d, struct pt_regs *regs);
static void das1800_handle_dma(comedi_device *dev, comedi_subdevice *s);
static void das1800_handle_fifo_half_full(comedi_device *dev, comedi_subdevice *s);
static void das1800_handle_fifo_not_empty(comedi_device *dev, comedi_subdevice *s);
void disable_das1800(comedi_device *dev);
static int das1800_ai_do_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd);
static int das1800_ai_do_cmd(comedi_device *dev, comedi_subdevice *s);
static int das1800_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int das1800_di_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int das1800_do_winsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
int das1800_probe(comedi_device *dev);
int das1800_set_frequency(comedi_device *dev);
int das1800_load_counter(unsigned int counterNumber, unsigned int counterValue, comedi_device *dev);

static int das1800_recognize(char *name)
{
	if(!strcmp(name, "das-1701st"))
		return das1701st;
	if(!strcmp(name, "das-1702st"))
		return das1702st;
	if(!strcmp(name, "das-1702hr"))
		return das1702hr;
	if(!strcmp(name, "das-1701ao"))
		return das1701ao;
	if(!strcmp(name, "das-1702ao"))
		return das1702ao;
	if(!strcmp(name, "das-1801st") || !strcmp(name, "das1800"))
		return das1801st;
	if(!strcmp(name, "das-1802st"))
		return das1802st;
	if(!strcmp(name, "das-1802hr"))
		return das1802hr;
	if(!strcmp(name, "das-1801hc"))
		return das1801hc;
	if(!strcmp(name, "das-1802hc"))
		return das1802hc;
	if(!strcmp(name, "das-1801ao"))
		return das1801ao;
	if(!strcmp(name, "das-1802ao"))
		return das1802ao;

	return -1;
}

/* probes and checks das-1800 series board type
 */
int das1800_probe(comedi_device *dev)
{
	int id;
	id = (inb(dev->iobase + DAS1800_DIGITAL) >> 4) & 0x7; /* get id bits */
	switch(id)
	{
		case 0x3:
			if(dev->board == das1801st)
			{
				printk(" Board model: DAS-1801ST-DA\n");
				return dev->board;
			}
			if(dev->board == das1802st)
			{
				printk(" Board model: DAS-1802ST-DA\n");
				return dev->board;
			}
			if(dev->board == das1701st)
			{
				printk(" Board model: DAS-1701ST-DA\n");
				return dev->board;
			}
			if(dev->board == das1702st)
			{
				printk(" Board model: DAS-1702ST-DA\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-1800ST-DA series\n");
			return das1801st;
			break;
		case 0x4:
			if(dev->board == das1802hr)
			{
				printk(" Board model: DAS-1802HR-DA\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-1802HR-DA\n");
			if(dev->board == das1702hr)
			{
				printk(" Board model: DAS-1702HR-DA\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-1802HR-DA\n");
			return das1802hr;
			break;
		case 0x5:
			if(dev->board == das1801ao)
			{
				printk(" Board model: DAS-1801AO\n");
				return dev->board;
			}
			if(dev->board == das1802ao)
			{
				printk(" Board model: DAS-1802AO\n");
				return dev->board;
			}
			if(dev->board == das1701ao)
			{
				printk(" Board model: DAS-1701AO\n");
				return dev->board;
			}
			if(dev->board == das1702ao)
			{
				printk(" Board model: DAS-1702AO\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-1800AO series\n");
			return das1801ao;
			break;
		case 0x6:
			if(dev->board == das1802hr)
			{
				printk(" Board model: DAS-1802HR\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-1802HR\n");
			if(dev->board == das1702hr)
			{
				printk(" Board model: DAS-1702HR\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-1702HR\n");
			return das1802hr;
			break;
		case 0x7:
			if(dev->board == das1801st)
			{
				printk(" Board model: DAS-1801ST\n");
				return dev->board;
			}
			if(dev->board == das1802st)
			{
				printk(" Board model: DAS-1802ST\n");
				return dev->board;
			}
			if(dev->board == das1701st)
			{
				printk(" Board model: DAS-1701ST\n");
				return dev->board;
			}
			if(dev->board == das1702st)
			{
				printk(" Board model: DAS-1702ST\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-1800ST series\n");
			return das1801st;
			break;
		case 0x8:
			if(dev->board == das1801hc)
			{
				printk(" Board model: DAS-1801HC\n");
				return dev->board;
			}
			if(dev->board == das1802hc)
			{
				printk(" Board model: DAS-1802HC\n");
				return dev->board;
			}
			printk(" Board model (probed): DAS-1800HC series\n");
			return das1801hc;
			break;
		default :
			printk(" Board model: probe returned 0x%x (unknown)\n", id);
			return dev->board;
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
	int status;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;	/* analog input subdevice */

	status = inb(dev->iobase + DAS1800_STATUS);
	/* if interrupt was not caused by das-1800 */
	if(!(status & INT))
	{
		return;
	} else if(devpriv->dma0) 	/* if dma is enabled and generated the interrupt */
	{
		if(status & DMATC)
			das1800_handle_dma(dev, s);
	} else if(status & FHF)
	{
		das1800_handle_fifo_half_full(dev, s);
	} else if(status & FNE)
	{
		das1800_handle_fifo_not_empty(dev, s);
	}
	comedi_bufcheck(dev, s);
	/* if the card's fifo has overflowed */
	if(status & OVF)
	{
		comedi_error(dev, "DAS1800 FIFO overflow");
		das1800_cancel(dev, s);
		comedi_error_done(dev, s);
		return;
	}
	/* stop taking data if appropriate */
	if(devpriv->count == 0 && devpriv->forever == 0)
	{
		disable_das1800(dev);		/* diable hardware conversions */
		comedi_done(dev, s);
	}

	return;
}

static void das1800_handle_dma(comedi_device *dev, comedi_subdevice *s)
{
	unsigned long flags;
	int numPoints;
	short dpnt;
	short *buffer;
	int unipolar;
	int i;

	if(get_dma_residue(devpriv->dma_current))
		return;
	numPoints = devpriv->count;
	if(numPoints > devpriv->dma_buf_size / sizeof(sampl_t))
		numPoints = devpriv->dma_buf_size / sizeof(sampl_t);
	buffer = devpriv->dma_current_buf;
	flags = claim_dma_lock();
	disable_dma(devpriv->dma_current);
	if(devpriv->dual_dma)
	{
		if(devpriv->dma_current == devpriv->dma0)
		{
			devpriv->dma_current = devpriv->dma1;
			devpriv->dma_current_buf = devpriv->dma1_buf;
		}
		else
		{
			devpriv->dma_current = devpriv->dma0;
			devpriv->dma_current_buf = devpriv->dma0_buf;
		}
	}
	set_dma_addr(devpriv->dma_current, (unsigned int) devpriv->dma_current_buf);
	if((devpriv->count - numPoints) * sizeof(short) > devpriv->dma_buf_size || devpriv->forever)
		set_dma_count(devpriv->dma_current, devpriv->dma_buf_size);
	else
		set_dma_count(devpriv->dma_current, (devpriv->count - numPoints) * sizeof(short));

	// if we are doing dual channel dma, enable the second channel immediately
	if(devpriv->dual_dma)
	{
		enable_dma(devpriv->dma_current);
		release_dma_lock(flags);
	}

	unipolar = inb(dev->iobase + DAS1800_CONTROL_C) & UB;

	for( i = 0; i < numPoints; i++)
	{
		/* write data point to buffer */
		if(s->buf_int_ptr + sizeof(sampl_t) > s->cur_trig.data_len )
		{
			s->buf_int_ptr = 0;
			comedi_eobuf(dev, s);
		}
		dpnt = buffer[i];
		/* convert to unsigned type if we are in a bipolar mode */
		if(!unipolar);
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

	// for single channel dma, re-enable after old data has been read
	if(devpriv->dual_dma == 0)
	{
		enable_dma(devpriv->dma_current);
		release_dma_lock(flags);
	}

	/* clear interrupt */
	outb(FNE, dev->iobase + DAS1800_STATUS);

	return;
}

static void das1800_handle_fifo_half_full(comedi_device *dev, comedi_subdevice *s)
{
	int i;		/* loop index */
	int numPoints = 0;	/* number of points to read */
	sampl_t dpnt;
	int unipolar;

	unipolar = inb(dev->iobase + DAS1800_CONTROL_C) & UB;

	numPoints = HALF_FIFO;
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
		/* convert to unsigned type if we are in a bipolar mode */
		if(!unipolar);
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
	/* clear interrupt */
	outb(FNE, dev->iobase + DAS1800_STATUS);
	// if there are just a few points left, switch to interrupt on end of conversion
	if(devpriv->count < HALF_FIFO && devpriv->count > 0 && devpriv->forever == 0)
	{
		devpriv->irq_dma_bits &= ~FIMD;	// interrupt fifo not empty
		outb(devpriv->irq_dma_bits, dev->iobase + DAS1800_CONTROL_B);
	}
	return;
}

static void das1800_handle_fifo_not_empty(comedi_device *dev, comedi_subdevice *s)
{
	sampl_t dpnt;
	int unipolar;

	unipolar = inb(dev->iobase + DAS1800_CONTROL_C) & UB;

	while(inb(dev->iobase + DAS1800_STATUS) & FNE)
	{
		/* write data point to buffer */
		if(s->buf_int_ptr + sizeof(sampl_t) > s->cur_trig.data_len )
		{
			s->buf_int_ptr = 0;
			comedi_eobuf(dev, s);
		}
		dpnt = inw(dev->iobase + DAS1800_FIFO);
		/* convert to unsigned type if we are in a bipolar mode */
		if(!unipolar);
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
		if(devpriv->count == 0 && devpriv->forever == 0)
			break;
	}
	/* clear interrupt */
	outb(FNE, dev->iobase + DAS1800_STATUS);

	return;
}

static int das1800_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	unsigned long flags;
	int iobase = it->options[0];
	int irq = it->options[1];
	int dma0 = it->options[2];
	int dma1 = it->options[3];

	/* allocate and initialize dev->private */
	if(alloc_private(dev, sizeof(das1800_private)) < 0)
		return -ENOMEM;

	printk("comedi%d: das1800: io 0x%x", dev->minor, iobase);
	if(irq)
	{
		printk(", irq %i", irq);
		if(dma0)
		{
			printk(", dma %i", dma0);
			if(dma1) printk(" and %i", dma1);
		}
	}
	printk("\n");

	dev->board = das1800_probe(dev);
	if(dev->board < 0)
	{
		printk("unable to determine board type\n");
		return -ENODEV;
	}

	dev->board_ptr = das1800_boards + dev->board;
	dev->board_name = thisboard->name;

	/* check if io addresses are available */
	if(check_region(iobase, DAS1800_SIZE) < 0)
	{
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase, DAS1800_SIZE, thisboard->name);
	dev->iobase = iobase;
	dev->iosize = DAS1800_SIZE;

	/* grab our IRQ */
	if(irq)
	{
		if(comedi_request_irq( irq, das1800_interrupt, 0, thisboard->name, dev ))
		{
			printk( "unable to allocate irq %d\n", irq);
			return -EINVAL;
		}
	}
	dev->irq = irq;

	// set bits that tell card which irq to use
	switch(irq)
	{
		case 0:
			break;
		case 3:
			devpriv->irq_dma_bits |= 0x8;
			break;
		case 5:
			devpriv->irq_dma_bits |= 0x10;
			break;
		case 7:
			devpriv->irq_dma_bits |= 0x18;
			break;
		case 10:
			devpriv->irq_dma_bits |= 0x28;
			break;
		case 11:
			devpriv->irq_dma_bits |= 0x30;
			break;
		case 15:
			devpriv->irq_dma_bits |= 0x38;
			break;
		default:
			printk("irq out of range\n");
			return -EINVAL;
			break;
	}

//dma stuff
	// need an irq to do dma
	if(irq)
	{
		if(dma0)
		{
			switch(dma0 | (dma1 << 4))
			{
				case 0x5:	// dma0 == 5
					devpriv->irq_dma_bits |= 0x1;
					break;
				case 0x6:	// dma0 == 6
					devpriv->irq_dma_bits |= 0x2;
					break;
				case 0x7:	// dma0 == 7
					devpriv->irq_dma_bits |= 0x3;
					break;
				case 0x65:	// dma0 == 5, dma1 == 6
					devpriv->irq_dma_bits |= 0x5;
					break;
				case 0x76:	// dma0 == 6, dma1 == 7
					devpriv->irq_dma_bits |= 0x6;
					break;
				case 0x57:	// dma0 == 7, dma1 == 5
					devpriv->irq_dma_bits |= 0x7;
					break;
				default:
					printk("%s only supports dma channels 5 through 7\n"
						"  Dual dma only allows the following combinations:\n"
						"  dma 5,6 / 6,7 / or 7,5\n", thisboard->name);
					return -EINVAL;
					break;
			}
			if(request_dma(dma0, thisboard->name))
			{
				printk("failed to allocate dma channel %i\n", dma0);
				return -EINVAL;
			}
			devpriv->dma0 = dma0;
			devpriv->dma_current = dma0;
			if(dma1)
			{
				if(request_dma(dma1, thisboard->name))
				{
					printk("failed to allocate dma channel %i\n", dma1);
					return -EINVAL;
				}
				devpriv->dma1 = dma1;
				devpriv->dual_dma = 1;
			}
			devpriv->dma_buf_size = 0x1ff00;
			devpriv->dma0_buf = kmalloc(devpriv->dma_buf_size, GFP_BUFFER | GFP_DMA);
			if(devpriv->dma0_buf == 0)
				return -ENOMEM;
			devpriv->dma_current_buf = devpriv->dma0_buf;
			if(dma1)
			{
				devpriv->dma1_buf = kmalloc(devpriv->dma_buf_size, GFP_BUFFER | GFP_DMA);
				if(devpriv->dma1_buf == 0)
					return -ENOMEM;
			}
			flags = claim_dma_lock();
			disable_dma(devpriv->dma0);
			clear_dma_ff(devpriv->dma0);
			set_dma_mode(devpriv->dma0, DMA_MODE_READ);
			if(dma1)
			{
				disable_dma(devpriv->dma1);
				clear_dma_ff(devpriv->dma1);
				set_dma_mode(devpriv->dma1, DMA_MODE_READ);
			}
			release_dma_lock(flags);
		}
	}

	dev->n_subdevices = 4;
	if(alloc_subdevices(dev) < 0)
		return -ENOMEM;

	/* analog input subdevice */
	s = dev->subdevices + 0;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_DIFF | SDF_GROUND;
	if(thisboard->common)
		s->subdev_flags |= SDF_COMMON;
	s->n_chan = thisboard->qram_len;
	s->len_chanlist = thisboard->qram_len;
	s->maxdata = (1 << thisboard->resolution) - 1;
	s->range_table = das1800_range_lkup[dev->board];
	s->do_cmd = das1800_ai_do_cmd;
	s->do_cmdtest = das1800_ai_do_cmdtest;
	s->insn_read = das1800_ai_rinsn;
	s->cancel = das1800_cancel;

	/* analog out (TODO!)*/
	s = dev->subdevices + 1;
	s->type = COMEDI_SUBD_UNUSED;

	/* di */
	s = dev->subdevices + 2;
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_read = das1800_di_rinsn;

	/* do */
	s = dev->subdevices + 3;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = thisboard->do_n_chan;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_write = das1800_do_winsn;

	return 0;
};

static int das1800_detach(comedi_device *dev)
{
	printk("comedi%d: das1800: remove\n", dev->minor);

	/* only free stuff if it has been allocated by _attach */
	if(dev->iobase)
		release_region(dev->iobase, DAS1800_SIZE);
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);
	if(devpriv->dma0)
		free_dma(devpriv->dma0);
	if(devpriv->dma0_buf)
		kfree(devpriv->dma0_buf);
	if(devpriv->dma1)
		free_dma(devpriv->dma1);
	if(devpriv->dma1_buf)
		kfree(devpriv->dma1_buf);

	return 0;
};

static int das1800_cancel(comedi_device *dev, comedi_subdevice *s)
{
	devpriv->forever = 0;
	devpriv->count = 0;
	disable_das1800(dev);
	return 0;
}

void disable_das1800(comedi_device *dev)
{
	outb(0x0, dev->iobase + DAS1800_STATUS);	/* disable conversions */
	outb(0x0, dev->iobase + DAS1800_CONTROL_A);	/* disable and clear fifo and stop triggering */
	outb(0x0, dev->iobase + DAS1800_CONTROL_B);	/* disable interrupts and dma */
	if(devpriv->dma0) disable_dma(devpriv->dma0);
	if(devpriv->dma1) disable_dma(devpriv->dma1);
}

static int das1800_ai_do_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
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
	if(!cmd->stop_src && tmp != cmd->stop_src) err++;

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

static int das1800_ai_do_cmd(comedi_device *dev, comedi_subdevice *s)
{
	int i, n, chan_range;
	int aref;
	int conv_flags;
	int control_a;
	int lock_flags;

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
		/* mask off unipolar/bipolar bit from range */
		chan_range = CR_CHAN(s->cmd.chanlist[i]) | ((CR_RANGE(s->cmd.chanlist[i]) & 0x3) << 8);
		outw(chan_range, dev->iobase + DAS1800_QRAM);
	}
	outb(n - 1, dev->iobase + DAS1800_QRAM_ADDRESS);	/*finish write to QRAM */
	outb(0x0, dev->iobase + DAS1800_SELECT);	/* select ADC for baseAddress + 0x0 */

	/* enable auto channel scan, send interrupts on end of conversion,
	 * set clock source to internal or external, select analog reference,
	 * select unipolar / bipolar
	 */
	aref = CR_AREF(s->cmd.chanlist[0]);
	conv_flags = UQEN;
	if(aref != AREF_DIFF)
		conv_flags |= SD;
	if(aref == AREF_COMMON)
		conv_flags |= CMEN;
	/* if a unipolar range was selected */
	if(CR_RANGE(s->cmd.chanlist[0]) & 0x4)
		conv_flags |= UB;
	switch(s->cmd.convert_src)
	{
		case TRIG_TIMER:
			/* trig on cascaded counters */
			outb(conv_flags | 0x1, dev->iobase + DAS1800_CONTROL_C);
			/* set conversion frequency */
			i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->divisor1), &(devpriv->divisor2), &(s->cmd.convert_arg), s->cmd.flags & TRIG_ROUND_MASK);
			if(das1800_set_frequency(dev) < 0)
			{
				comedi_error(dev, "Error setting up counters");
				return -1;
			}
			break;
		case TRIG_EXT:
			/* trig on falling edge */
			outb(conv_flags | 0x3, dev->iobase + DAS1800_CONTROL_C);
			break;
		default:
			break;
	}

	switch(s->cmd.stop_src)
	{
		case TRIG_COUNT:
			devpriv->count = s->cmd.stop_arg;
			devpriv->forever = 0;
			/* set interrupt mode */
			if(s->cmd.stop_arg < HALF_FIFO)
				devpriv->irq_dma_bits &= ~FIMD;	// interrupt fifo not empty
			else
				devpriv->irq_dma_bits |= FIMD;	//interrupt fifo half full
			break;
		case TRIG_NONE:
			devpriv->forever = 1;
			devpriv->count = 0;
			devpriv->irq_dma_bits |= FIMD;
			break;
		default:
			break;
	}
	/* set FIMD bit in control register b */
	outb(devpriv->irq_dma_bits, dev->iobase + DAS1800_CONTROL_B);

	// enable fifo
	control_a = FFEN;
	switch(s->cmd.start_src)
	{
		case TRIG_EXT:
			control_a |= TGEN | CGSL;
			break;
		case TRIG_NOW:
			control_a |= CGEN;
			break;
		default:
			break;
	}

	outb(devpriv->irq_dma_bits, dev->iobase + DAS1800_CONTROL_B); // enable irq/dma
	if(devpriv->dma0)
	{
		lock_flags = claim_dma_lock();
		disable_dma(devpriv->dma0);
		set_dma_addr(devpriv->dma0, (unsigned int) devpriv->dma0_buf);
		if(devpriv->count * sizeof(short) > devpriv->dma_buf_size || devpriv->forever)
			set_dma_count(devpriv->dma0, devpriv->dma_buf_size);
		else
			set_dma_count(devpriv->dma0, devpriv->count * sizeof(short));
		devpriv->dma_current = devpriv->dma0;
		devpriv->dma_current_buf = devpriv->dma0_buf;
		enable_dma(devpriv->dma0);
		release_dma_lock(lock_flags);
	}
	outb(control_a, dev->iobase + DAS1800_CONTROL_A);	/* enable fifo and triggering */
	outb(CVEN, dev->iobase + DAS1800_STATUS);	/* enable conversions */

	return 0;
}

static int das1800_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
 	int i, n;
	int chan, range, aref, chan_range;
	int timeout = 1000;
	short dpnt;
	int conv_flags = 0;

	/* set up analog reference and unipolar / bipolar mode */
	aref = CR_AREF(insn->chanspec);
	conv_flags |= UQEN;
	if(aref != AREF_DIFF)
		conv_flags |= SD;
	if(aref == AREF_COMMON)
		conv_flags |= CMEN;
	/* if a unipolar range was selected */
	if(CR_RANGE(insn->chanspec) & 0x4)
		conv_flags |= UB;

	outb(conv_flags, dev->iobase + DAS1800_CONTROL_C);	/* software conversion enabled */
	outb(CVEN, dev->iobase + DAS1800_STATUS);	/* enable conversions */
	outb(0x0, dev->iobase + DAS1800_CONTROL_A);	/* reset fifo */
	outb(FFEN, dev->iobase + DAS1800_CONTROL_A);


	chan = CR_CHAN(insn->chanspec);
	/* mask of unipolar/bipolar bit from range */
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
		dpnt = inw(dev->iobase + DAS1800_FIFO);
		/* shift data to offset binary for bipolar ranges */
		if((conv_flags & UB) == 0)
			dpnt += 1 << (thisboard->resolution - 1);
		data[n] = dpnt;
	}

	return n;
}

static int das1800_di_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int chan = CR_CHAN(insn->chanspec);
	int ret;

	ret = inb(dev->iobase + DAS1800_DIGITAL) & (1 << chan);
	if(ret) data[0] = 1;
	else data[0] = 0;

	return 1;
}

static int das1800_do_winsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int chan = CR_CHAN(insn->chanspec);

	// set channel to 1
	if(data[0])
		devpriv->do_bits |= (1 << chan);
	// set channel to 0
	else
		devpriv->do_bits &= ~(1 << chan);
	outb(devpriv->do_bits, dev->iobase + DAS1800_DIGITAL);

	return 1;
}

/* loads counters with divisor1, divisor2 from private structure */
int das1800_set_frequency(comedi_device *dev)
{
	int err = 0;

	if(das1800_load_counter(1, devpriv->divisor1, dev)) err++;
	if(das1800_load_counter(2, devpriv->divisor2, dev)) err++;
	if(err)
		return -1;

	return 0;
}

int das1800_load_counter(unsigned int counterNumber, unsigned int counterValue, comedi_device *dev)
{
	unsigned char byte;

	if(counterNumber > 2) return -1;
	if(counterValue == 1 || counterValue > 0xffff) return -1;

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
