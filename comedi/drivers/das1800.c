/*
    das1800.c driver for Keitley das1700/das1800 series boards
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

das-1701st
das-1701st-da
das-1701ao
das-1702st
das-1702st-da
das-1702hr
das-1702hr-da
das-1702ao
das-1801st
das-1801st-da
das-1801hc
das-1801ao
das-1802st
das-1802st-da
das-1802hr
das-1802hr-da
das-1802hc
das-1802ao

Options:
	[0] - base io address
	[1] - irq (optional, required for timed or externally triggered conversions)
	[2] - dma0 (optional, requires irq)
	[3] - dma1 (optional, requires irq and dma0)

irq can be omitted, although the cmd interface will not work without it.

analog input cmd triggers supported:
	start_src:      TRIG_NOW | TRIG_EXT
	scan_begin_src: TRIG_FOLLOW | TRIG_TIMER | TRIG_EXT
	scan_end_src:   TRIG_COUNT
	convert_src:    TRIG_TIMER | TRIG_EXT (TRIG_EXT requires scan_begin_src == TRIG_FOLLOW)
	stop_src:       TRIG_COUNT | TRIG_EXT | TRIG_NONE

scan_begin_src triggers TRIG_TIMER and TRIG_EXT use the card's
'burst mode' which limits the valid conversion time to 64 microseconds
(convert_arg <= 64000).  This limitation does not apply if scan_begin_src
is TRIG_FOLLOW.

If stop_src is TRIG_EXT then stop_src_arg is the number of conversions to take
after receiving the external trigger before stopping conversions.  It must be
at least 1 and no more than 0x10000 == 65536

NOTES:
Only the DAS-1801ST has been tested by me.
Unipolar and bipolar ranges cannot be mixed in the channel/gain list.

TODO:
	Add support for analog out on 'ao' cards.
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

// misc. defines
#define DAS1800_SIZE           16	//uses 16 io addresses
#define HALF_FIFO              512	// 1024 sample fifo
#define TIMER_BASE             200	// 5 Mhz master clock
#define MIN_DMA_TRANSFER       1024	// minimum dma transfer size I want to use
#define UNIPOLAR               0x4	// bit that determines whether input range is uni/bipolar

/* Registers for the das1800 */
#define DAS1800_FIFO            0x0
#define DAS1800_QRAM            0x0
#define DAS1800_DAC             0x0
#define DAS1800_SELECT          0x2
#define   ADC                     0x0
#define   QRAM                    0x1
#define   DAC(a)                  (0x2 + a)
#define DAS1800_DIGITAL         0x3
#define DAS1800_CONTROL_A       0x4
#define   FFEN                    0x1
#define   CGEN                    0x4
#define   CGSL                    0x8
#define   TGEN                    0x10
#define   TGSL                    0x20
#define   ATEN                    0x80
#define DAS1800_CONTROL_B       0x5
#define   DMA_CH5                 0x1
#define   DMA_CH6                 0x2
#define   DMA_CH7                 0x3
#define   DMA_CH5_CH6             0x5
#define   DMA_CH6_CH7             0x6
#define   DMA_CH7_CH5             0x7
#define   DMA_ENABLED             0x3	//mask used to determine if dma is enabled
#define   DMA_DUAL                0x4
#define   IRQ3                    0x8
#define   IRQ5                    0x10
#define   IRQ7                    0x18
#define   IRQ10                   0x28
#define   IRQ11                   0x30
#define   IRQ15                   0x38
#define   FIMD                    0x40
#define DAS1800_CONTROL_C       0X6
#define   IPCLK                   0x1
#define   XPCLK                   0x3
#define   BMDE                    0x4
#define   CMEN                    0x8
#define   UQEN                    0x10
#define   SD                      0x40
#define   UB                      0x80
#define DAS1800_STATUS          0x7
#define   INT                     0x1
#define   DMATC                   0x2
#define   CT0TC                   0x8
#define   OVF                     0x10
#define   FHF                     0x20
#define   FNE                     0x40
#define   CVEN                    0x80
#define DAS1800_BURST_LENGTH    0x8
#define DAS1800_BURST_RATE      0x9
#define DAS1800_QRAM_ADDRESS    0xa
#define DAS1800_COUNTER         0xc

#define IOBASE2                   0x400	//offset of additional ioports used on 'ao' cards

enum{
	das1701st, das1701st_da, das1702st, das1702st_da, das1702hr, das1702hr_da,
	das1701ao, das1702ao, das1801st, das1801st_da, das1802st, das1802st_da,
	das1802hr, das1802hr_da, das1801hc, das1802hc, das1801ao, das1802ao
};

static int das1800_attach(comedi_device *dev, comedi_devconfig *it);
static int das1800_detach(comedi_device *dev);
int das1800_probe(comedi_device *dev);
static int das1800_cancel(comedi_device *dev, comedi_subdevice *s);
static void das1800_interrupt(int irq, void *d, struct pt_regs *regs);
static void das1800_handle_dma(comedi_device *dev, comedi_subdevice *s);
static void das1800_handle_fifo_half_full(comedi_device *dev, comedi_subdevice *s);
static void das1800_handle_fifo_not_empty(comedi_device *dev, comedi_subdevice *s);
void disable_das1800(comedi_device *dev);
static int das1800_ai_do_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd);
static int das1800_ai_do_cmd(comedi_device *dev, comedi_subdevice *s);
static int das1800_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int das1800_ao_winsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int das1800_di_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int das1800_di_rbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int das1800_do_winsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int das1800_do_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);

int das1800_set_frequency(comedi_device *dev);
unsigned int burst_convert_arg(unsigned int convert_arg, int round_mode);
unsigned int suggest_transfer_size(comedi_device *dev, unsigned int ns);

// analog input ranges
static comedi_lrange range_ai_das1801 = {
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

static comedi_lrange range_ai_das1802 = {
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

typedef struct das1800_board_struct{
	char *name;
	int ai_speed;	/* max conversion period in nanoseconds */
	int resolution;	/* bits of ai resolution */
	int qram_len;	/* length of card's channel / gain queue */
	int common;	/* supports AREF_COMMON flag */
	int do_n_chan;	/* number of digital output channels */
	int ao_ability;	/* 0 == no analog out, 1 == basic analog out, 2 == waveform analog out */
	int ao_n_chan;	/* number of analog out channels */
	comedi_lrange *range_ai;	/* available input ranges */
}das1800_board;

/* Warning: the maximum conversion speeds listed below are
 * not always achievable depending on board setup (see
 * user manual.)
 */
das1800_board das1800_boards[] =
{
	{
		name:	"das-1701st",
		ai_speed:	6250,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	0,
		ao_n_chan:	0,
		range_ai:	&range_ai_das1801,
	},
	{
		name:	"das-1701st-da",
		ai_speed:	6250,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	1,
		ao_n_chan:	4,
		range_ai:	&range_ai_das1801,
	},
	{
		name:		"das-1702st",
		ai_speed:	6250,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	0,
		ao_n_chan:	0,
		range_ai:	&range_ai_das1802,
	},
	{
		name:		"das-1702st-da",
		ai_speed:	6250,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	1,
		ao_n_chan:	4,
		range_ai:	&range_ai_das1802,
	},
	{
		name:		"das-1702hr",
		ai_speed:	20000,
		resolution:	16,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	0,
		ao_n_chan:	0,
		range_ai:	&range_ai_das1802,
	},
	{
		name:		"das-1702hr-da",
		ai_speed:	20000,
		resolution:	16,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	1,
		ao_n_chan:	2,
		range_ai:	&range_ai_das1802,
	},
	{
		name:	"das-1701ao",
		ai_speed:	6250,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	2,
		ao_n_chan:	2,
		range_ai:	&range_ai_das1801,
	},
	{
		name:		"das-1702ao",
		ai_speed:	6250,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	2,
		ao_n_chan:	2,
		range_ai:	&range_ai_das1802,
	},
	{
		name:	"das-1801st",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	0,
		ao_n_chan:	0,
		range_ai:	&range_ai_das1801,
	},
	{
		name:	"das-1801st-da",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	0,
		ao_n_chan:	4,
		range_ai:	&range_ai_das1801,
	},
	{
		name:		"das-1802st",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	0,
		ao_n_chan:	0,
		range_ai:	&range_ai_das1802,
	},
	{
		name:		"das-1802st-da",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	1,
		ao_n_chan:	4,
		range_ai:	&range_ai_das1802,
	},
	{
		name:		"das-1802hr",
		ai_speed:	10000,
		resolution:	16,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	0,
		ao_n_chan:	0,
		range_ai:	&range_ai_das1802,
	},
	{
		name:		"das-1802hr-da",
		ai_speed:	10000,
		resolution:	16,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	1,
		ao_n_chan:	2,
		range_ai:	&range_ai_das1802,
	},
	{
		name:		"das-1801hc",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	64,
		common:	0,
		do_n_chan:	8,
		ao_ability:	1,
		ao_n_chan:	2,
		range_ai:	&range_ai_das1801,
	},
	{
		name:		"das-1802hc",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	64,
		common:	0,
		do_n_chan:	8,
		ao_ability:	1,
		ao_n_chan:	2,
		range_ai:	&range_ai_das1802,
	},
	{
		name:	"das-1801ao",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	2,
		ao_n_chan:	2,
		range_ai:	&range_ai_das1801,
	},
	{
		name:		"das-1802ao",
		ai_speed:	3000,
		resolution:	12,
		qram_len:	256,
		common:	1,
		do_n_chan:	4,
		ao_ability:	2,
		ao_n_chan:	2,
		range_ai:	&range_ai_das1802,
	},
};
/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((das1800_board *)dev->board_ptr)

typedef struct{
	volatile unsigned int count;  /* number of data points left to be taken */
	volatile int forever;  /* flag indicating whether we should take data forever */
	unsigned int divisor1;	/* value to load into board's counter 1 for timed conversions */
	unsigned int divisor2; 	/* value to load into board's counter 2 for timed conversions */
	int do_bits;	/* digital output bits */
	int irq_dma_bits;	/* bits for control register b */
	/* dma bits for control register b, stored so that dma can be
	 * turned on and off */
	int dma_bits;
	unsigned int dma0;	/* dma channels used */
	unsigned int dma1;
	volatile unsigned int dma_current;	/* dma channel currently in use */
	short *dma_buf0;	/* pointers to dma buffers */
	short *dma_buf1;
	volatile short *dma_current_buf;	/* pointer to dma buffer currently being used */
	unsigned int dma_buf_max_size;	/* allocated size in bytes of dma buffers */
	unsigned int dma_buf_size;	/* size of buffers currently used, depends on sampling frequency */
	int iobase2;	/* secondary io address used for analog out on 'ao' boards */
	short ao_update_bits; /* remembers the last write to the 'update' dac */
}das1800_private;

#define devpriv ((das1800_private *)dev->private)

// analog out range for boards with basic analog out
static comedi_lrange range_ao_1 = {
	1,
	{
		RANGE(-10, 10),
	}
};

// analog out range for 'ao' boards
/*
static comedi_lrange range_ao_2 = {
	2,
	{
		RANGE(-10, 10),
		RANGE(-5, 5),
	}
};
*/

comedi_driver driver_das1800={
	driver_name:	"das1800",
	module:		THIS_MODULE,
	attach:		das1800_attach,
	detach:		das1800_detach,
	num_names:	sizeof(das1800_boards) / sizeof(das1800_board),
	board_name:	das1800_boards,
	offset:		sizeof(das1800_board),
};

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_das1800);

static int das1800_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	unsigned long flags;
	int iobase = it->options[0];
	int irq = it->options[1];
	int dma0 = it->options[2];
	int dma1 = it->options[3];
	int iobase2;
	int board;

	/* allocate and initialize dev->private */
	if(alloc_private(dev, sizeof(das1800_private)) < 0)
		return -ENOMEM;

	printk("comedi%d: %s: io 0x%x", dev->minor, driver_das1800.driver_name, iobase);
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

	if(iobase == 0)
	{
		printk(" io base address required\n");
		return -EINVAL;
	}

	/* check if io addresses are available */
	if(check_region(iobase, DAS1800_SIZE) < 0)
	{
		printk(" I/O port conflict: failed to allocate ports 0x%x to 0x%x\n",
			iobase, iobase + DAS1800_SIZE - 1);
		return -EIO;
	}
	request_region(iobase, DAS1800_SIZE, driver_das1800.driver_name);
	dev->iobase = iobase;

	board = das1800_probe(dev);
	if(board < 0)
	{
		printk(" unable to determine board type\n");
		return -ENODEV;
	}

	dev->board_ptr = das1800_boards + board;
	dev->board_name = thisboard->name;

	// if it is an 'ao' board with fancy analog out then we need extra io ports
	if(thisboard->ao_ability == 2)
	{
		iobase2 = iobase + IOBASE2;
		if(check_region(iobase2, DAS1800_SIZE) < 0)
		{
			printk(" I/O port conflict: failed to allocate ports 0x%x to 0x%x\n",
				iobase2, iobase2 + DAS1800_SIZE - 1);
			return -EIO;
		}
		request_region(iobase2, DAS1800_SIZE, driver_das1800.driver_name);
		devpriv->iobase2 = iobase2;
	}

	/* grab our IRQ */
	if(irq)
	{
		if(comedi_request_irq( irq, das1800_interrupt, 0, driver_das1800.driver_name, dev ))
		{
			printk(" unable to allocate irq %d\n", irq);
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
			printk(" irq out of range\n");
			return -EINVAL;
			break;
	}

//dma stuff
	// need an irq to do dma
	if(irq)
	{
		if(dma0)
		{
			//encode dma0 and dma1 into 2 digit hexadecimal for switch
			switch((dma0 & 0x7) | (dma1 << 4))
			{
				case 0x5:	// dma0 == 5
					devpriv->dma_bits |= DMA_CH5;
					break;
				case 0x6:	// dma0 == 6
					devpriv->dma_bits |= DMA_CH6;
					break;
				case 0x7:	// dma0 == 7
					devpriv->dma_bits |= DMA_CH7;
					break;
				case 0x65:	// dma0 == 5, dma1 == 6
					devpriv->dma_bits |= DMA_CH5_CH6;
					break;
				case 0x76:	// dma0 == 6, dma1 == 7
					devpriv->dma_bits |= DMA_CH6_CH7;
					break;
				case 0x57:	// dma0 == 7, dma1 == 5
					devpriv->dma_bits |= DMA_CH7_CH5;
					break;
				default:
					printk(" only supports dma channels 5 through 7\n"
						" Dual dma only allows the following combinations:\n"
						" dma 5,6 / 6,7 / or 7,5\n");
					return -EINVAL;
					break;
			}
			if(request_dma(dma0, driver_das1800.driver_name))
			{
				printk(" failed to allocate dma channel %i\n", dma0);
				return -EINVAL;
			}
			devpriv->dma0 = dma0;
			devpriv->dma_current = dma0;
			if(dma1)
			{
				if(request_dma(dma1, driver_das1800.driver_name))
				{
					printk(" failed to allocate dma channel %i\n", dma1);
					return -EINVAL;
				}
				devpriv->dma1 = dma1;
			}
			devpriv->dma_buf_max_size = 0x1ff00;
			devpriv->dma_buf0 = kmalloc(devpriv->dma_buf_max_size, GFP_BUFFER | GFP_DMA);
			if(devpriv->dma_buf0 == NULL)
				return -ENOMEM;
			devpriv->dma_current_buf = devpriv->dma_buf0;
			devpriv->dma_buf1 = kmalloc(devpriv->dma_buf_max_size, GFP_BUFFER | GFP_DMA);
			if(devpriv->dma_buf1 == NULL)
				return -ENOMEM;
			flags = claim_dma_lock();
			disable_dma(devpriv->dma0);
			set_dma_mode(devpriv->dma0, DMA_MODE_READ);
			if(dma1)
			{
				disable_dma(devpriv->dma1);
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
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_DIFF | SDF_GROUND;
	if(thisboard->common)
		s->subdev_flags |= SDF_COMMON;
	s->n_chan = thisboard->qram_len;
	s->len_chanlist = thisboard->qram_len;
	s->maxdata = (1 << thisboard->resolution) - 1;
	s->range_table = thisboard->range_ai;
	s->do_cmd = das1800_ai_do_cmd;
	s->do_cmdtest = das1800_ai_do_cmdtest;
	s->insn_read = das1800_ai_rinsn;
	s->cancel = das1800_cancel;

	/* analog out */
	s = dev->subdevices + 1;
	if(thisboard->ao_ability == 1)
	{
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_WRITEABLE;
		s->n_chan = thisboard->ao_n_chan;
		s->maxdata = (1 << thisboard->resolution) - 1;
		s->range_table = &range_ao_1;
		s->insn_write = das1800_ao_winsn;
	}
	else
	{
		s->type = COMEDI_SUBD_UNUSED;
	}

	/* di */
	s = dev->subdevices + 2;
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_read = das1800_di_rinsn;
	s->insn_bits = das1800_di_rbits;

	/* do */
	s = dev->subdevices + 3;
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = thisboard->do_n_chan;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_write = das1800_do_winsn;
	s->insn_bits = das1800_do_wbits;

	disable_das1800(dev);

	// initialize digital out channels
	outb(devpriv->do_bits, dev->iobase + DAS1800_DIGITAL);

	// initialize analog out channels
	if(thisboard->ao_ability == 1)
	{
		// select 'update' dac channel for baseAddress + 0x0
		outb(DAC(thisboard->ao_n_chan - 1), dev->iobase + DAS1800_SELECT);
		outw(devpriv->ao_update_bits, dev->iobase + DAS1800_DAC);
	}

	return 0;
};

static int das1800_detach(comedi_device *dev)
{
	/* only free stuff if it has been allocated by _attach */
	if(dev->iobase)
		release_region(dev->iobase, DAS1800_SIZE);
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);
	if(dev->private)
	{
		if(devpriv->iobase2)
			release_region(devpriv->iobase2, DAS1800_SIZE);
		if(devpriv->dma0)
			free_dma(devpriv->dma0);
		if(devpriv->dma1)
			free_dma(devpriv->dma1);
		if(devpriv->dma_buf0)
			kfree(devpriv->dma_buf0);
		if(devpriv->dma_buf1)
			kfree(devpriv->dma_buf1);
	}

	printk("comedi%d: %s: remove\n", dev->minor, driver_das1800.driver_name);

	return 0;
};

/* probes and checks das-1800 series board type
 */
int das1800_probe(comedi_device *dev)
{
	int id;
	int board;

	id = (inb(dev->iobase + DAS1800_DIGITAL) >> 4) & 0xf; /* get id bits */
	board = ((das1800_board *)dev->board_ptr) - das1800_boards;

	switch(id)
	{
		// das-1800st-da
		case 0x3:
			if(board == das1801st_da || board == das1802st_da ||
				board == das1701st_da || board == das1702st_da)
			{
				printk(" Board model: %s\n", das1800_boards[board].name);
				return board;
			}
			printk(" Board model (probed, not recommended): das-1800st-da series\n");
			return das1801st;
			break;
		// das-1800hr-da
		case 0x4:
			if(board == das1802hr_da || board == das1702hr_da)
			{
				printk(" Board model: %s\n", das1800_boards[board].name);
				return board;
			}
			printk(" Board model (probed, not recommended): das-1802hr-da\n");
			return das1802hr;
			break;
		case 0x5:
			if(board == das1801ao || board == das1802ao ||
				board == das1701ao || board == das1702ao)
			{
				printk(" Board model: %s\n", das1800_boards[board].name);
				return board;
			}
			printk(" Board model (probed, not recommended): das-1800ao series\n");
			return das1801ao;
			break;
		case 0x6:
			if(board == das1802hr || board == das1702hr)
			{
				printk(" Board model: %s\n", das1800_boards[board].name);
				return board;
			}
			printk(" Board model (probed, not recommended): das-1802hr\n");
			return das1802hr;
			break;
		case 0x7:
			if(board == das1801st || board == das1802st ||
				board == das1701st || board == das1702st)
			{
				printk(" Board model: %s\n", das1800_boards[board].name);
				return board;
			}
			printk(" Board model (probed, not recommended): das-1800st series\n");
			return das1801st;
			break;
		case 0x8:
			if(board == das1801hc || board == das1802hc)
			{
				printk(" Board model: %s\n", das1800_boards[board].name);
				return board;
			}
			printk(" Board model (probed, not recommended): das-1800hc series\n");
			return das1801hc;
			break;
		default :
			printk(" Board model: probe returned 0x%x (unknown)\n", id);
			return board;
			break;
	}
	return -1;
}

static int das1800_cancel(comedi_device *dev, comedi_subdevice *s)
{
	devpriv->forever = 0;
	devpriv->count = 0;
	disable_das1800(dev);
	return 0;
}

static void das1800_interrupt(int irq, void *d, struct pt_regs *regs)
{
	int status;
	unsigned long irq_flags;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;	/* analog input subdevice */
	comedi_async *async;

	status = inb(dev->iobase + DAS1800_STATUS);
	/* if interrupt was not caused by das-1800 */
	if(!(status & INT) || !(dev->attached))
	{
		return;
	}
	async = s->async;
	comedi_spin_lock_irqsave(&dev->spinlock, irq_flags);
	outb(ADC, dev->iobase + DAS1800_SELECT);
	// dma buffer full or about-triggering (stop_src == TRIG_EXT)
	if(devpriv->irq_dma_bits & DMA_ENABLED)
	{
		if(status & (DMATC | CT0TC))
		{
			das1800_handle_dma(dev, s);
		}
	}
	// if fifo half full, and there has not an external stop trigger
	else if((status & FHF) && !(status & CT0TC))
	{
		das1800_handle_fifo_half_full(dev, s);
	} else if(status & FNE)
	{
		das1800_handle_fifo_not_empty(dev, s);
	}
	/* clear interrupt */
	outb(FNE, dev->iobase + DAS1800_STATUS);

	comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);
	async->events |= COMEDI_CB_BLOCK;
	/* if the card's fifo has overflowed */
	if(status & OVF)
	{
		comedi_error(dev, "DAS1800 FIFO overflow");
		das1800_cancel(dev, s);
		async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
		comedi_event(dev, s, async->events);
		async->events = 0;
		return;
	}
	// if stop_src TRIG_EXT has occurred
	if(status & CT0TC) devpriv->forever = 0;
	/* stop taking data if appropriate */
	if(devpriv->count == 0 && devpriv->forever == 0)
	{
		disable_das1800(dev);		/* disable hardware conversions */
		async->events |= COMEDI_CB_EOA;
	}

	comedi_event(dev, s, async->events);
	async->events = 0;

	return;
}

static void das1800_handle_dma(comedi_device *dev, comedi_subdevice *s)
{
	unsigned long flags;
	unsigned long numPoints, leftover;
	long maxPoints, residue;
	short dpnt;
	volatile short *buffer;
	int unipolar;
	int i;
	const int dual_dma = devpriv->irq_dma_bits & DMA_DUAL;

	flags = claim_dma_lock();
	disable_dma(devpriv->dma_current);
 	/* clear flip-flop to make sure 2-byte registers for
	 * count and address get set correctly */
	clear_dma_ff(devpriv->dma_current);

	// figure out how many points to read
	maxPoints = devpriv->dma_buf_size  / sizeof(short);
	/* residue is the number of points left to be done on the dma
	 * transfer.  It should always be zero at this point unless
	 * the stop_src is set to external triggering.
	 */
	residue = get_dma_residue(devpriv->dma_current) / sizeof(short);
	numPoints = maxPoints - residue;
	if(devpriv->forever == 0 && devpriv->count < numPoints)
		numPoints = devpriv->count;

	// figure out how many points will be stored next time
	leftover = 0;
	if(devpriv->forever)
		leftover = maxPoints;
	else
	{
		if(dual_dma)
		{
			if(devpriv->count > 2 * maxPoints)
				leftover = devpriv->count - 2 * maxPoints;
		}else
		{
			if(devpriv->count > maxPoints)
				leftover = devpriv->count - maxPoints;
		}
		if(leftover > maxPoints)
			leftover = maxPoints;
	}
	/* there should only be a residue if collection was stopped by having
	 * the stop_src set to an external trigger, in which case there
	 * will be no more data
	 */
	if(residue)
		leftover = 0;

	// remember buffer before dma_current_buf gets changed
	buffer = devpriv->dma_current_buf;

	// read data from the other buffer next time
	if(devpriv->dma_current_buf == devpriv->dma_buf0)
	{
		devpriv->dma_current_buf = devpriv->dma_buf1;
	}
	else
	{
		devpriv->dma_current_buf = devpriv->dma_buf0;
	}

	// re-enable  dma (single dma)
	if((dual_dma == 0) && leftover)
	{
		set_dma_addr(devpriv->dma_current, (unsigned int) devpriv->dma_current_buf);
		set_dma_count(devpriv->dma_current, leftover * sizeof(short));
		enable_dma(devpriv->dma_current);
		release_dma_lock(flags);
	}

	/* see if card is using a unipolar or bipolar range */
	unipolar = inb(dev->iobase + DAS1800_CONTROL_C) & UB;

	// read data from dma buffer
	for( i = 0; i < numPoints; i++)
	{
		/* write data point to comedi buffer */
		dpnt = buffer[i];
		/* convert to unsigned type if we are in a bipolar mode */
		if(!unipolar);
			dpnt += 1 << (thisboard->resolution - 1);
		comedi_buf_put(s->async, dpnt);
		if(devpriv->count > 0) devpriv->count--;
	}

	// re-enable  dma (dual dma)
	if(dual_dma)
	{
		if(leftover)
		{
			set_dma_addr(devpriv->dma_current, (unsigned int) buffer);
			set_dma_count(devpriv->dma_current, leftover * sizeof(short));
			enable_dma(devpriv->dma_current);
			release_dma_lock(flags);
		}
		// read data from the other channel next time
		if(devpriv->dma_current == devpriv->dma0)
		{
			devpriv->dma_current = devpriv->dma1;
		}
		else
		{
			devpriv->dma_current = devpriv->dma0;
		}
	}

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
		dpnt = inw(dev->iobase + DAS1800_FIFO);
		/* convert to unsigned type if we are in a bipolar mode */
		if(!unipolar);
			dpnt += 1 << (thisboard->resolution - 1);
		comedi_buf_put(s->async, dpnt);
		if(devpriv->count > 0) devpriv->count--;
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
		if(devpriv->count == 0 && devpriv->forever == 0)
			break;
		dpnt = inw(dev->iobase + DAS1800_FIFO);
		/* convert to unsigned type if we are in a bipolar mode */
		if(!unipolar);
			dpnt += 1 << (thisboard->resolution - 1);
		comedi_buf_put(s->async, dpnt);
		if(devpriv->count > 0) devpriv->count--;
	}

	return;
}

void disable_das1800(comedi_device *dev)
{
	outb(0x0, dev->iobase + DAS1800_STATUS);	/* disable conversions */
	outb(0x0, dev->iobase + DAS1800_CONTROL_A);	/* disable and clear fifo and stop triggering */
	outb(0x0, dev->iobase + DAS1800_CONTROL_B);	/* disable interrupts and dma */
	if(devpriv->dma0) disable_dma(devpriv->dma0);
	if(devpriv->dma1) disable_dma(devpriv->dma1);
}

/* test analog input cmd */
static int das1800_ai_do_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err = 0;
	int tmp;
	unsigned int tmp_arg;
	int i;
	int unipolar;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src || tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW | TRIG_TIMER | TRIG_EXT;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src) err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if(!cmd->convert_src || tmp != cmd->convert_src) err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src) err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_EXT | TRIG_NONE;
	if(!cmd->stop_src || tmp != cmd->stop_src) err++;

	if(err) return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	// uniqueness check
	if(cmd->start_src != TRIG_NOW &&
		cmd->start_src != TRIG_EXT) err++;
	if(cmd->scan_begin_src != TRIG_FOLLOW &&
		cmd->scan_begin_src != TRIG_TIMER &&
		cmd->scan_begin_src != TRIG_EXT) err++;
	if(cmd->convert_src != TRIG_TIMER &&
		cmd->convert_src != TRIG_EXT) err++;
	if(cmd->stop_src != TRIG_COUNT &&
		cmd->stop_src != TRIG_NONE &&
		cmd->stop_src != TRIG_EXT) err++;
	//compatibility check
	if(cmd->scan_begin_src != TRIG_FOLLOW &&
		cmd->convert_src != TRIG_TIMER) err++;

	if(err) return 2;

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

	// make sure user is not trying to mix unipolar and bipolar ranges
	unipolar = CR_RANGE(cmd->chanlist[0]) & UNIPOLAR;
	for(i = 1; i < cmd->chanlist_len; i++)
	{
		if(unipolar != (CR_RANGE(cmd->chanlist[i]) & UNIPOLAR))
		{
			comedi_error(dev, "unipolar and bipolar ranges cannot be mixed in the chanlist");
			err++;
			break;
		}
	}

	switch(cmd->stop_src)
	{
		case TRIG_EXT:
			if(cmd->stop_arg > 0x10000)
			{
				cmd->stop_arg = 0x10000;
				err++;
			}
		case TRIG_COUNT:
			if(!cmd->stop_arg)
			{
				cmd->stop_arg = 1;
				err++;
			}
			// this break is for both TRIG_EXT and TRIG_COUNT
			break;
		case TRIG_NONE:
			if(cmd->stop_arg != 0)
			{
				cmd->stop_arg = 0;
				err++;
			}
			break;
		default:
			break;
	}

	if(err) return 3;

	/* step 4: fix up any arguments */

	if(cmd->convert_src == TRIG_TIMER)
	{
		// if we are not in burst mode
		if(cmd->scan_begin_src == TRIG_FOLLOW)
		{
			tmp_arg = cmd->convert_arg;
			/* calculate counter values that give desired timing */
			i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->divisor1), &(devpriv->divisor2), &(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
			if(tmp_arg != cmd->convert_arg) err++;
		}
		// if we are in burst mode
		else
		{
			// check that convert_arg is compatible
			tmp_arg = cmd->convert_arg;
			cmd->convert_arg = burst_convert_arg(cmd->convert_arg, cmd->flags & TRIG_ROUND_MASK);
			if(tmp_arg != cmd->convert_arg) err++;

			if(cmd->scan_begin_src == TRIG_TIMER)
			{
				// if scans are timed faster than conversion rate allows
				if(cmd->convert_arg * cmd->chanlist_len > cmd->scan_begin_arg)
				{
					cmd->scan_begin_arg = cmd->convert_arg * cmd->chanlist_len;
					err++;
				}
				tmp_arg = cmd->scan_begin_arg;
				/* calculate counter values that give desired timing */
				i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->divisor1), &(devpriv->divisor2), &(cmd->scan_begin_arg), cmd->flags & TRIG_ROUND_MASK);
				if(tmp_arg != cmd->scan_begin_arg) err++;
			}
		}
	}

	if(err) return 4;

	return 0;
}

/* analog input cmd interface */

// first, some utility functions used in the main ai_do_cmd()

// returns appropriate bits for control register a, depending on command
int control_a_bits(comedi_cmd cmd)
{
	int control_a;

	control_a = FFEN;	//enable fifo
	if(cmd.stop_src == TRIG_EXT)
	{
		control_a |= ATEN;
	}
	switch(cmd.start_src)
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

	return control_a;
}

// returns appropriate bits for control register c, depending on command
int control_c_bits(comedi_cmd cmd)
{
	int control_c;
	int aref;

	/* set clock source to internal or external, select analog reference,
	 * select unipolar / bipolar
	 */
	aref = CR_AREF(cmd.chanlist[0]);
	control_c = UQEN;	//enable upper qram addresses
	if(aref != AREF_DIFF)
		control_c |= SD;
	if(aref == AREF_COMMON)
		control_c |= CMEN;
	/* if a unipolar range was selected */
	if(CR_RANGE(cmd.chanlist[0]) & UNIPOLAR)
		control_c |= UB;
	switch(cmd.scan_begin_src)
	{
		case TRIG_FOLLOW:	// not in burst mode
			switch(cmd.convert_src)
			{
				case TRIG_TIMER:
					/* trig on cascaded counters */
					control_c |= IPCLK;
					break;
				case TRIG_EXT:
					/* trig on falling edge of external trigger */
					control_c |= XPCLK;
					break;
				default:
					break;
			}
			break;
		case TRIG_TIMER:
			// burst mode with internal pacer clock
			control_c |= BMDE | IPCLK;
			break;
		case TRIG_EXT:
			// burst mode with external trigger
			control_c |= BMDE | XPCLK;
			break;
		default:
			break;
	}

	return control_c;
}

// sets up counters
int setup_counters(comedi_device *dev, comedi_cmd cmd)
{
	// setup cascaded counters for conversion/scan frequency
	switch(cmd.scan_begin_src)
	{
		case TRIG_FOLLOW:	// not in burst mode
			if(cmd.convert_src == TRIG_TIMER)
			{
				/* set conversion frequency */
				i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->divisor1), &(devpriv->divisor2),
					&(cmd.convert_arg), cmd.flags & TRIG_ROUND_MASK);
				if(das1800_set_frequency(dev) < 0)
				{
					return -1;
				}
			}
			break;
		case TRIG_TIMER:	// in burst mode
			/* set scan frequency */
			i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->divisor1), &(devpriv->divisor2),
				&(cmd.scan_begin_arg), cmd.flags & TRIG_ROUND_MASK);
			if(das1800_set_frequency(dev) < 0)
			{
				return -1;
			}
			break;
		default:
			break;
	}

	// setup counter 0 for 'about triggering'
	if(cmd.stop_src == TRIG_EXT)
	{
		// load counter 0 in mode 0
		i8254_load(dev->iobase + DAS1800_COUNTER, 0, cmd.stop_arg, 0);
	}

	return 0;
}

// sets up dma
void setup_dma(comedi_device *dev, comedi_cmd cmd)
{
	unsigned long lock_flags;
	const int dual_dma = devpriv->irq_dma_bits & DMA_DUAL;

	if((devpriv->irq_dma_bits & DMA_ENABLED) == 0)
		return;

	/* determine a reasonable dma transfer size */
	switch(cmd.scan_begin_src)
	{
		case TRIG_FOLLOW:	// not in burst mode
			if(cmd.convert_src == TRIG_TIMER)
				devpriv->dma_buf_size = suggest_transfer_size(dev, cmd.convert_arg);
			break;
		case TRIG_TIMER:
			devpriv->dma_buf_size = suggest_transfer_size(dev, cmd.scan_begin_arg * cmd.chanlist_len);
			break;
		default:
			devpriv->dma_buf_size = devpriv->dma_buf_max_size;
			break;
	}

	lock_flags = claim_dma_lock();
	disable_dma(devpriv->dma0);
	/* clear flip-flop to make sure 2-byte registers for
	 * count and address get set correctly */
	clear_dma_ff(devpriv->dma0);
	set_dma_addr(devpriv->dma0, (unsigned int) devpriv->dma_buf0);
	// set appropriate size of transfer
	if(devpriv->count * sizeof(short) >= devpriv->dma_buf_size || devpriv->forever)
		set_dma_count(devpriv->dma0, devpriv->dma_buf_size);
	else
		set_dma_count(devpriv->dma0, devpriv->count * sizeof(short));
	// set up dual dma if appropriate
	if(dual_dma && (devpriv->count * sizeof(short) > devpriv->dma_buf_size || devpriv->forever))
	{
		disable_dma(devpriv->dma1);
		/* clear flip-flop to make sure 2-byte registers for
		 * count and address get set correctly */
		clear_dma_ff(devpriv->dma1);
		set_dma_addr(devpriv->dma1, (unsigned int) devpriv->dma_buf1);
		// set appropriate size of transfer
		if(devpriv->count * sizeof(short) >= 2 * devpriv->dma_buf_size || devpriv->forever)
			set_dma_count(devpriv->dma1, devpriv->dma_buf_size);
		else
			set_dma_count(devpriv->dma1, devpriv->count * sizeof(short) - devpriv->dma_buf_size);
		enable_dma(devpriv->dma1);
	}
	devpriv->dma_current = devpriv->dma0;
	devpriv->dma_current_buf = devpriv->dma_buf0;
	enable_dma(devpriv->dma0);
	release_dma_lock(lock_flags);

	return;
}

// programs channel/gain list into card
void program_chanlist(comedi_device *dev, comedi_cmd cmd)
{
	int i, n, chan_range;
	unsigned long irq_flags;
	const int range_mask = 0x3; //masks unipolar/bipolar bit off range
	const int range_bitshift = 8;

	n = cmd.chanlist_len;
	comedi_spin_lock_irqsave(&dev->spinlock, irq_flags);
	outb(QRAM, dev->iobase + DAS1800_SELECT); /* select QRAM for baseAddress + 0x0 */
	outb(n - 1, dev->iobase + DAS1800_QRAM_ADDRESS);	/*set QRAM address start */
	/* make channel / gain list */
	for(i = 0; i < n; i++)
	{
		chan_range = CR_CHAN(cmd.chanlist[i]) | ((CR_RANGE(cmd.chanlist[i]) & range_mask) << range_bitshift);
		outw(chan_range, dev->iobase + DAS1800_QRAM);
	}
	outb(n - 1, dev->iobase + DAS1800_QRAM_ADDRESS);	/*finish write to QRAM */
	comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);

	return;
}

// analog input do_cmd
static int das1800_ai_do_cmd(comedi_device *dev, comedi_subdevice *s)
{
	int ret;
	int control_a, control_c;
	comedi_async *async = s->async;
	comedi_cmd cmd = async->cmd;

	if(!dev->irq)
	{
		comedi_error(dev, "no irq assigned for das-1800, cannot do hardware conversions");
		return -1;
	}

	/* disable dma on TRIG_WAKE_EOS (to reduce latency) or TRIG_RT
	 * (because dma in handler is unsafe at hard real-time priority) */
	if(cmd.flags & (TRIG_WAKE_EOS | TRIG_RT))
	{
		devpriv->irq_dma_bits &= ~DMA_ENABLED;
	}else
	{
		devpriv->irq_dma_bits |= devpriv->dma_bits;
	}
	// interrupt on end of conversion for TRIG_WAKE_EOS
	if(cmd.flags & TRIG_WAKE_EOS)
	{
		// interrupt fifo not empty
		devpriv->irq_dma_bits &= ~FIMD;
	}else
	{
		// interrupt fifo half full
		devpriv->irq_dma_bits |= FIMD;
	}
	// determine how many conversions we need
	if(cmd.stop_src == TRIG_COUNT)
	{
		devpriv->count = cmd.stop_arg * cmd.chanlist_len;
		devpriv->forever = 0;
		/* if they want just a few points, interrupt fifo not empty */
		if(cmd.stop_arg * cmd.scan_end_arg < HALF_FIFO)
			devpriv->irq_dma_bits &= ~FIMD;
	}else
	{
		devpriv->forever = 1;
		devpriv->count = 0;
	}

	disable_das1800(dev);

	// determine proper bits for control registers
	control_a = control_a_bits(cmd);
	control_c = control_c_bits(cmd);

	/* setup card and start */
	program_chanlist(dev, cmd);
	ret = setup_counters(dev, cmd);
	if(ret < 0)
	{
		comedi_error(dev, "Error setting up counters");
		return ret;
	}
	setup_dma(dev, cmd);
	async->events = 0;
	outb(control_c, dev->iobase + DAS1800_CONTROL_C);
	// set conversion rate and length for burst mode
	if(control_c & BMDE)
	{
		// program conversion period with number of microseconds minus 1
		outb(cmd.convert_arg / 1000 - 1, dev->iobase + DAS1800_BURST_RATE);
		outb(cmd.chanlist_len - 1, dev->iobase + DAS1800_BURST_LENGTH);
	}
	outb(devpriv->irq_dma_bits, dev->iobase + DAS1800_CONTROL_B); // enable irq/dma
	outb(control_a, dev->iobase + DAS1800_CONTROL_A);	/* enable fifo and triggering */
	outb(CVEN, dev->iobase + DAS1800_STATUS);	/* enable conversions */

	return 0;
}

/* read analog input */
static int das1800_ai_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
 	int i, n;
	int chan, range, aref, chan_range;
	int timeout = 1000;
	short dpnt;
	int conv_flags = 0;
	unsigned long irq_flags;

	/* set up analog reference and unipolar / bipolar mode */
	aref = CR_AREF(insn->chanspec);
	conv_flags |= UQEN;
	if(aref != AREF_DIFF)
		conv_flags |= SD;
	if(aref == AREF_COMMON)
		conv_flags |= CMEN;
	/* if a unipolar range was selected */
	if(CR_RANGE(insn->chanspec) & UNIPOLAR)
		conv_flags |= UB;

	outb(conv_flags, dev->iobase + DAS1800_CONTROL_C);	/* software conversion enabled */
	outb(CVEN, dev->iobase + DAS1800_STATUS);	/* enable conversions */
	outb(0x0, dev->iobase + DAS1800_CONTROL_A);	/* reset fifo */
	outb(FFEN, dev->iobase + DAS1800_CONTROL_A);


	chan = CR_CHAN(insn->chanspec);
	/* mask of unipolar/bipolar bit from range */
	range = CR_RANGE(insn->chanspec) & 0x3;
	chan_range = chan | (range << 8);
	comedi_spin_lock_irqsave(&dev->spinlock, irq_flags);
	outb(QRAM, dev->iobase + DAS1800_SELECT);	/* select QRAM for baseAddress + 0x0 */
	outb(0x0, dev->iobase + DAS1800_QRAM_ADDRESS);	/* set QRAM address start */
	outw(chan_range, dev->iobase + DAS1800_QRAM);
	outb(0x0, dev->iobase + DAS1800_QRAM_ADDRESS);	/*finish write to QRAM */
	outb(ADC, dev->iobase + DAS1800_SELECT);	/* select ADC for baseAddress + 0x0 */

	udelay(2);

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
	comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);

	return n;
}

/* writes to an analog output channel */
static int das1800_ao_winsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int chan = CR_CHAN(insn->chanspec);
//	int range = CR_RANGE(insn->chanspec);
	int update_chan = thisboard->ao_n_chan - 1;
	short output;
	unsigned long irq_flags;

	//  card expects two's complement data
	output = data[0] - (1 << (thisboard->resolution - 1));
	// if the write is to the 'update' channel, we need to remember its value
	if(chan == update_chan)
		devpriv->ao_update_bits = output;
	// write to channel
	comedi_spin_lock_irqsave(&dev->spinlock, irq_flags);
	outb(DAC(chan), dev->iobase + DAS1800_SELECT); /* select dac channel for baseAddress + 0x0 */
	outw(output, dev->iobase + DAS1800_DAC);
	// now we need to write to 'update' channel to update all dac channels
	if(chan != update_chan)
	{
		outb(DAC(update_chan), dev->iobase + DAS1800_SELECT); /* select 'update' channel for baseAddress + 0x0 */
		outw(devpriv->ao_update_bits, dev->iobase + DAS1800_DAC);
	}
	comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);

	return 1;
}

/* reads from a digital input channel */
static int das1800_di_rinsn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int chan = CR_CHAN(insn->chanspec);
	int ret;

	ret = inb(dev->iobase + DAS1800_DIGITAL) & (1 << chan);
	if(ret) data[0] = 1;
	else data[0] = 0;

	return 1;
}

/* reads from digital input channels */
static int das1800_di_rbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{

	data[1] = inb(dev->iobase + DAS1800_DIGITAL) & 0xf;
	data[0] = 0;

	return 2;
}

/* writes to a digital output channel */
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

/* writes to digital output channels */
static int das1800_do_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	lsampl_t wbits;

	// only set bits that have been masked
	data[0] &= (1 << s->n_chan) - 1;
	wbits = devpriv->do_bits;
	wbits &= ~data[0];
	wbits |= data[0] & data[1];
	devpriv->do_bits = wbits;

	outb(devpriv->do_bits, dev->iobase + DAS1800_DIGITAL);

	data[1] = devpriv->do_bits;

	return 2;
}

/* loads counters with divisor1, divisor2 from private structure */
int das1800_set_frequency(comedi_device *dev)
{
	int err = 0;

	// counter 1, mode 2
	if(i8254_load(dev->iobase + DAS1800_COUNTER, 1, devpriv->divisor1, 2)) err++;
	// counter 2, mode 2
	if(i8254_load(dev->iobase + DAS1800_COUNTER, 2, devpriv->divisor2, 2)) err++;
	if(err)
		return -1;

	return 0;
}

/* converts requested conversion timing to timing compatible with
 * hardware, used only when card is in 'burst mode'
 */
unsigned int burst_convert_arg(unsigned int convert_arg, int round_mode)
{
	unsigned int micro_sec;

	// in burst mode, the maximum conversion time is 64 microseconds
	if(convert_arg > 64000)
		convert_arg = 64000;

	// the conversion time must be an integral number of microseconds
	switch(round_mode)
	{
		case TRIG_ROUND_NEAREST:
		default:
			micro_sec = (convert_arg + 500) / 1000;
			break;
		case TRIG_ROUND_DOWN:
			micro_sec = convert_arg / 1000;
			break;
		case TRIG_ROUND_UP:
			micro_sec = (convert_arg - 1) / 1000 + 1;
			break;
	}

	// return number of nanoseconds
	return micro_sec * 1000;
}

// utility function that suggests a dma transfer size based on the conversion period 'ns'
unsigned int suggest_transfer_size(comedi_device *dev, unsigned int ns)
{
	int size;
	unsigned int freq = 1000000000 / ns;
	static const int sample_size = 2;	// size in bytes of one sample from board

	// make buffer fill in no more than 1/3 second
	size = (freq / 3) * sample_size;

	// set a minimum and maximum size allowed
	if(size > devpriv->dma_buf_max_size)
		size = devpriv->dma_buf_max_size - devpriv->dma_buf_max_size % sample_size;
	else if(size < sample_size)
		size = sample_size;

	return size;
}
