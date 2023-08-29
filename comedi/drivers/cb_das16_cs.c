/*
    comedi/drivers/das16cs.c
    Driver for Computer Boards PC-CARD DAS16/16.

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000, 2001, 2002 David A. Schleef <ds@schleef.org>

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
/*
Driver: cb_das16_cs
Description: Computer Boards PC-CARD DAS16/16
Devices: [ComputerBoards] PC-CARD DAS16/16 (cb_das16_cs), PC-CARD DAS16/16-AO
Author: ds
Updated: Mon, 04 Nov 2002 20:04:21 -0800
Status: experimental


*/

#include <linux/comedidev.h>
#include <linux/delay.h>
#include <linux/pci.h>

#ifdef COMEDI_COMPAT_HAVE_CS_TYPES_H
#include <pcmcia/cs_types.h>
#endif
#ifdef COMEDI_COMPAT_HAVE_CS_H
#include <pcmcia/cs.h>
#endif
#include <pcmcia/cistpl.h>
#include <pcmcia/ds.h>

#include "8253.h"

#define DAS16CS_SIZE			18

#define DAS16CS_ADC_DATA		0
#define DAS16CS_DIO_MUX			2
#define DAS16CS_MISC1			4
#define DAS16CS_MISC2			6
#define DAS16CS_CTR0			8
#define DAS16CS_CTR1			10
#define DAS16CS_CTR2			12
#define DAS16CS_CTR_CONTROL		14
#define DAS16CS_DIO			16

typedef struct das16cs_board_struct {
	const char *name;
	int device_id;
	int n_ao_chans;
} das16cs_board;
static const das16cs_board das16cs_boards[] = {
	{
	      device_id:0x0000,/* unknown */
	      name:	"PC-CARD DAS16/16",
	      n_ao_chans:0,
		},
	{
	      device_id:0x0039,
	      name:	"PC-CARD DAS16/16-AO",
	      n_ao_chans:2,
		},
	{
	      device_id:0x4009,
	      name:	"PCM-DAS16s/16",
	      n_ao_chans:0,
		},
};

#define n_boards (sizeof(das16cs_boards)/sizeof(das16cs_boards[0]))
#define thisboard ((const das16cs_board *)dev->board_ptr)

typedef struct {
	struct pcmcia_device *link;

	lsampl_t ao_readback[2];
	unsigned short status1;
	unsigned short status2;
} das16cs_private;
#define devpriv ((das16cs_private *)dev->private)

static int das16cs_attach(comedi_device * dev, comedi_devconfig * it);
static int das16cs_detach(comedi_device * dev);
static comedi_driver driver_das16cs = {
      driver_name:"cb_das16_cs",
      module:THIS_MODULE,
      attach:das16cs_attach,
      detach:das16cs_detach,
};

static struct pcmcia_device *cur_dev = NULL;

static const comedi_lrange das16cs_ai_range = { 4, {
			RANGE(-10, 10),
			RANGE(-5, 5),
			RANGE(-2.5, 2.5),
			RANGE(-1.25, 1.25),
	}
};

static irqreturn_t das16cs_interrupt(int irq, void *d PT_REGS_ARG);
static int das16cs_ai_rinsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int das16cs_ai_cmd(comedi_device * dev, comedi_subdevice * s);
static int das16cs_ai_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd);
static int das16cs_ao_winsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int das16cs_ao_rinsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int das16cs_dio_insn_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int das16cs_dio_insn_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int das16cs_timer_insn_read(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int das16cs_timer_insn_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);

#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
static int get_prodid(comedi_device * dev, struct pcmcia_device *link)
{
	tuple_t tuple;
	u_short buf[128];
	int prodid = 0;

	tuple.TupleData = (cisdata_t *) buf;
	tuple.TupleOffset = 0;
	tuple.TupleDataMax = 255;
	tuple.DesiredTuple = CISTPL_MANFID;
	tuple.Attributes = TUPLE_RETURN_COMMON;
	if ((pcmcia_get_first_tuple(link, &tuple) == 0) &&
		(pcmcia_get_tuple_data(link, &tuple) == 0)) {
		prodid = le16_to_cpu(buf[1]);
	}

	return prodid;
}
#endif

static const das16cs_board *das16cs_probe(comedi_device * dev,
	struct pcmcia_device *link)
{
	int id;
	int i;

#ifdef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	id = link->card_id;
#else
	id = get_prodid(dev, link);
#endif

	for (i = 0; i < n_boards; i++) {
		if (das16cs_boards[i].device_id == id) {
			return das16cs_boards + i;
		}
	}

	printk("unknown board!\n");

	return NULL;
}

static int das16cs_attach(comedi_device * dev, comedi_devconfig * it)
{
	struct pcmcia_device *link;
	comedi_subdevice *s;
	int ret;
	int i;

	printk("comedi%d: cb_das16_cs: ", dev->minor);

	link = cur_dev;		/* XXX hack */
	if (!link)
		return -EIO;

#ifdef COMEDI_COMPAT_HAVE_CS_IO_REQ_T
	dev->iobase = link->io.BasePort1;
#else
	dev->iobase = link->resource[0]->start;
#endif
	printk("I/O base=0x%04lx ", dev->iobase);

	printk("fingerprint:\n");
	for (i = 0; i < 48; i += 2) {
		printk("%04x ", inw(dev->iobase + i));
	}
	printk("\n");

	ret = comedi_request_irq(
#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
		link->irq.AssignedIRQ,
#else
		link->irq,
#endif
		das16cs_interrupt,
		IRQF_SHARED, "cb_das16_cs", dev);
	if (ret < 0) {
		return ret;
	}
#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
	dev->irq = link->irq.AssignedIRQ;
#else
	dev->irq = link->irq;
#endif
	printk("irq=%u ", dev->irq);

	dev->board_ptr = das16cs_probe(dev, link);
	if (!dev->board_ptr)
		return -EIO;

	dev->board_name = thisboard->name;

	if (alloc_private(dev, sizeof(das16cs_private)) < 0)
		return -ENOMEM;

	if (alloc_subdevices(dev, 4) < 0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	dev->read_subdev = s;
	/* analog input subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_DIFF | SDF_CMD_READ;
	s->n_chan = 16;
	s->maxdata = 0xffff;
	s->range_table = &das16cs_ai_range;
	s->len_chanlist = 16;
	s->insn_read = das16cs_ai_rinsn;
	s->do_cmd = das16cs_ai_cmd;
	s->do_cmdtest = das16cs_ai_cmdtest;

	s = dev->subdevices + 1;
	/* analog output subdevice */
	if (thisboard->n_ao_chans) {
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_WRITABLE;
		s->n_chan = thisboard->n_ao_chans;
		s->maxdata = 0xffff;
		s->range_table = &range_bipolar10;
		s->insn_write = &das16cs_ao_winsn;
		s->insn_read = &das16cs_ao_rinsn;
	}

	s = dev->subdevices + 2;
	/* digital i/o subdevice */
	if (1) {
		s->type = COMEDI_SUBD_DIO;
		s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
		s->n_chan = 8;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_bits = das16cs_dio_insn_bits;
		s->insn_config = das16cs_dio_insn_config;
	} else {
		s->type = COMEDI_SUBD_UNUSED;
	}

	s = dev->subdevices + 3;
	/* timer subdevice */
	if (0) {
		s->type = COMEDI_SUBD_TIMER;
		s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
		s->n_chan = 1;
		s->maxdata = 0xff;
		s->range_table = &range_unknown;
		s->insn_read = das16cs_timer_insn_read;
		s->insn_config = das16cs_timer_insn_config;
	} else {
		s->type = COMEDI_SUBD_UNUSED;
	}

	printk("attached\n");

	return 1;
}

static int das16cs_detach(comedi_device * dev)
{
	printk("comedi%d: das16cs: remove\n", dev->minor);

	if (dev->irq) {
		comedi_free_irq(dev->irq, dev);
	}

	return 0;
}

static irqreturn_t das16cs_interrupt(int irq, void *d PT_REGS_ARG)
{
	//comedi_device *dev = d;
	return IRQ_HANDLED;
}

/*
 * "instructions" read/write data in "one-shot" or "software-triggered"
 * mode.
 */
static int das16cs_ai_rinsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i;
	int to;
	int aref;
	int range;
	int chan;
	static int range_bits[] = { 0x800, 0x000, 0x100, 0x200 };

	chan = CR_CHAN(insn->chanspec);
	aref = CR_AREF(insn->chanspec);
	range = CR_RANGE(insn->chanspec);

	outw(chan, dev->iobase + 2);

	devpriv->status1 &= ~0xf320;
	devpriv->status1 |= (aref == AREF_DIFF) ? 0 : 0x0020;
	outw(devpriv->status1, dev->iobase + 4);

	devpriv->status2 &= ~0xff00;
	devpriv->status2 |= range_bits[range];
	outw(devpriv->status2, dev->iobase + 6);

	for (i = 0; i < insn->n; i++) {
		outw(0, dev->iobase);

#define TIMEOUT 1000
		for (to = 0; to < TIMEOUT; to++) {
			if (inw(dev->iobase + 4) & 0x0080)
				break;
		}
		if (to == TIMEOUT) {
			printk("cb_das16_cs: ai timeout\n");
			return -ETIME;
		}
		data[i] = (unsigned short)inw(dev->iobase + 0);
	}

	return i;
}

static int das16cs_ai_cmd(comedi_device * dev, comedi_subdevice * s)
{
	return -EINVAL;
}

static int das16cs_ai_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd)
{
	int err = 0;
	int tmp;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER | TRIG_EXT;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	/* note that mutual compatiblity is not an issue here */
	if (cmd->scan_begin_src != TRIG_TIMER &&
		cmd->scan_begin_src != TRIG_EXT)
		err++;
	if (cmd->convert_src != TRIG_TIMER && cmd->convert_src != TRIG_EXT)
		err++;
	if (cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE)
		err++;

	if (err)
		return 2;

	/* step 3: make sure arguments are trivially compatible */

	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		err++;
	}
#define MAX_SPEED	10000	/* in nanoseconds */
#define MIN_SPEED	1000000000	/* in nanoseconds */

	if (cmd->scan_begin_src == TRIG_TIMER) {
		if (cmd->scan_begin_arg < MAX_SPEED) {
			cmd->scan_begin_arg = MAX_SPEED;
			err++;
		}
		if (cmd->scan_begin_arg > MIN_SPEED) {
			cmd->scan_begin_arg = MIN_SPEED;
			err++;
		}
	} else {
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		/* should specify multiple external triggers */
		if (cmd->scan_begin_arg > 9) {
			cmd->scan_begin_arg = 9;
			err++;
		}
	}
	if (cmd->convert_src == TRIG_TIMER) {
		if (cmd->convert_arg < MAX_SPEED) {
			cmd->convert_arg = MAX_SPEED;
			err++;
		}
		if (cmd->convert_arg > MIN_SPEED) {
			cmd->convert_arg = MIN_SPEED;
			err++;
		}
	} else {
		/* external trigger */
		/* see above */
		if (cmd->convert_arg > 9) {
			cmd->convert_arg = 9;
			err++;
		}
	}

	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	if (cmd->stop_src == TRIG_COUNT) {
		if (cmd->stop_arg > 0x00ffffff) {
			cmd->stop_arg = 0x00ffffff;
			err++;
		}
	} else {
		/* TRIG_NONE */
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
	}

	if (err)
		return 3;

	/* step 4: fix up any arguments */

	if (cmd->scan_begin_src == TRIG_TIMER) {
		unsigned int div1 = 0, div2 = 0;

		tmp = cmd->scan_begin_arg;
		i8253_cascade_ns_to_timer(100, &div1, &div2,
			&cmd->scan_begin_arg, cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->scan_begin_arg)
			err++;
	}
	if (cmd->convert_src == TRIG_TIMER) {
		unsigned int div1 = 0, div2 = 0;

		tmp = cmd->convert_arg;
		i8253_cascade_ns_to_timer(100, &div1, &div2,
			&cmd->scan_begin_arg, cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->convert_arg)
			err++;
		if (cmd->scan_begin_src == TRIG_TIMER &&
			cmd->scan_begin_arg <
			cmd->convert_arg * cmd->scan_end_arg) {
			cmd->scan_begin_arg =
				cmd->convert_arg * cmd->scan_end_arg;
			err++;
		}
	}

	if (err)
		return 4;

	return 0;
}

static int das16cs_ao_winsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);
	unsigned short status1;
	unsigned short d;
	int bit;

	for (i = 0; i < insn->n; i++) {
		devpriv->ao_readback[chan] = data[i];
		d = data[i];

		outw(devpriv->status1, dev->iobase + 4);
		comedi_udelay(1);

		status1 = devpriv->status1 & ~0xf;
		if (chan)
			status1 |= 0x0001;
		else
			status1 |= 0x0008;

/* 		printk("0x%04x\n",status1);*/
		outw(status1, dev->iobase + 4);
		comedi_udelay(1);

		for (bit = 15; bit >= 0; bit--) {
			int b = (d >> bit) & 0x1;
			b <<= 1;
/*			printk("0x%04x\n",status1 | b | 0x0000);*/
			outw(status1 | b | 0x0000, dev->iobase + 4);
			comedi_udelay(1);
/*			printk("0x%04x\n",status1 | b | 0x0004);*/
			outw(status1 | b | 0x0004, dev->iobase + 4);
			comedi_udelay(1);
		}
/*		make high both DAC0CS and DAC1CS to load
		new data and update analog output*/
		outw(status1 | 0x9, dev->iobase + 4);
	}

	return i;
}

/* AO subdevices should have a read insn as well as a write insn.
 * Usually this means copying a value stored in devpriv. */
static int das16cs_ao_rinsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++)
		data[i] = devpriv->ao_readback[chan];

	return i;
}

/* DIO devices are slightly special.  Although it is possible to
 * implement the insn_read/insn_write interface, it is much more
 * useful to applications if you implement the insn_bits interface.
 * This allows packed reading/writing of the DIO channels.  The
 * comedi core can convert between insn_bits and insn_read/write */
static int das16cs_dio_insn_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	if (insn->n != 2)
		return -EINVAL;

	if (data[0]) {
		s->state &= ~data[0];
		s->state |= data[0] & data[1];

		outw(s->state, dev->iobase + 16);
	}

	/* on return, data[1] contains the value of the digital
	 * input and output lines. */
	data[1] = inw(dev->iobase + 16);

	return 2;
}

static int das16cs_dio_insn_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int chan = CR_CHAN(insn->chanspec);
	int bits;

	if (chan < 4)
		bits = 0x0f;
	else
		bits = 0xf0;

	switch (data[0]) {
	case INSN_CONFIG_DIO_OUTPUT:
		s->io_bits |= bits;
		break;
	case INSN_CONFIG_DIO_INPUT:
		s->io_bits &= bits;
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] =
			(s->
			io_bits & (1 << chan)) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
		break;
	}

	devpriv->status2 &= ~0x00c0;
	devpriv->status2 |= (s->io_bits & 0xf0) ? 0x0080 : 0;
	devpriv->status2 |= (s->io_bits & 0x0f) ? 0x0040 : 0;

	outw(devpriv->status2, dev->iobase + 6);

	return insn->n;
}

static int das16cs_timer_insn_read(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	return -EINVAL;
}

static int das16cs_timer_insn_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	return -EINVAL;
}

/* PCMCIA stuff */

/*======================================================================

    The following pcmcia code for the pcm-das08 is adapted from the
    dummy_cs.c driver of the Linux PCMCIA Card Services package.

    The initial developer of the original code is David A. Hinds
    <dahinds@users.sourceforge.net>.  Portions created by David A. Hinds
    are Copyright (C) 1999 David A. Hinds.  All Rights Reserved.

======================================================================*/

/*
   All the PCMCIA modules use PCMCIA_DEBUG to control debugging.  If
   you do not define PCMCIA_DEBUG at all, all the debug code will be
   left out.  If you compile with PCMCIA_DEBUG=0, the debug code will
   be present but disabled -- but it can then be enabled for specific
   modules at load time with a 'pc_debug=#' option to insmod.
*/
#if defined(CONFIG_PCMCIA) || defined(CONFIG_PCMCIA_MODULE)

#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
module_param(pc_debug, int, 0644);
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
static char *version =
	"cb_das16_cs.c pcmcia code (David Schleef), modified from dummy_cs.c 1.31 2001/08/24 12:13:13 (David Hinds)";
#else
#define DEBUG(n, args...)
#endif

/*====================================================================*/

static void das16cs_pcmcia_config(struct pcmcia_device *link);
static void das16cs_pcmcia_release(struct pcmcia_device *link);
static int das16cs_pcmcia_suspend(struct pcmcia_device *p_dev);
static int das16cs_pcmcia_resume(struct pcmcia_device *p_dev);

/*
   The attach() and detach() entry points are used to create and destroy
   "instances" of the driver, where each instance represents everything
   needed to manage one actual PCMCIA card.
*/

static int das16cs_pcmcia_attach(struct pcmcia_device *);
static void das16cs_pcmcia_detach(struct pcmcia_device *);

/*
   You'll also need to prototype all the functions that will actually
   be used to talk to your device.  See 'memory_cs' for a good example
   of a fully self-sufficient driver; the other drivers rely more or
   less on other parts of the kernel.
*/

/*
   The devname variable is the "key" that is used to match up this
   device driver with appropriate cards, through the card configuration
   database.
*/

#ifdef COMEDI_COMPAT_HAVE_CS_TYPES_H
static const dev_info_t devname = "cb_das16_cs";
#else
static const char devname[] = "cb_das16_cs";
#endif

typedef struct local_info_t {
	struct pcmcia_device *link;
#ifdef COMEDI_COMPAT_HAVE_DS_DEV_NODE_T
	dev_node_t node;
#endif
	int stop;
	struct bus_operations *bus;
} local_info_t;

/*======================================================================

    das16cs_pcmcia_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.

======================================================================*/

static int das16cs_pcmcia_attach(struct pcmcia_device *link)
{
	local_info_t *local;

	DEBUG(0, "das16cs_pcmcia_attach()\n");

	/* Allocate space for private device-specific data */
	local = kzalloc(sizeof(local_info_t), GFP_KERNEL);
	if (!local)
		return -ENOMEM;
	local->link = link;
	link->priv = local;

	/* Initialize the pcmcia_device structure */
#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
	/* Interrupt setup */
	link->irq.Attributes = IRQ_TYPE_DYNAMIC_SHARING;
#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	link->irq.IRQInfo1 = IRQ_LEVEL_ID;
#endif
	link->irq.Handler = NULL;
#endif

#ifdef COMEDI_COMPAT_HAVE_CS_H
	link->conf.Attributes = 0;
	link->conf.IntType = INT_MEMORY_AND_IO;
#endif

	cur_dev = link;

	das16cs_pcmcia_config(link);

	return 0;
}				/* das16cs_pcmcia_attach */

static void das16cs_pcmcia_detach(struct pcmcia_device *link)
{
	DEBUG(0, "das16cs_pcmcia_detach(0x%p)\n", link);

#ifdef COMEDI_COMPAT_HAVE_DS_DEV_NODE_T
	if (link->dev_node)
#endif
	{
		((local_info_t *) link->priv)->stop = 1;
		das16cs_pcmcia_release(link);
	}
	/* This points to the parent local_info_t struct */
	if (link->priv)
		kfree(link->priv);
}				/* das16cs_pcmcia_detach */

#ifdef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
#ifdef COMEDI_COMPAT_HAVE_CS_H
static int das16cs_pcmcia_config_loop(struct pcmcia_device *p_dev,
				cistpl_cftable_entry_t *cfg,
				cistpl_cftable_entry_t *dflt,
				unsigned int vcc,
				void *priv_data)
{
	if (cfg->index == 0)
		return -EINVAL;

	/* Do we need to allocate an interrupt? */
#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
	if (cfg->irq.IRQInfo1 || dflt->irq.IRQInfo1)
#endif
	{
		p_dev->conf.Attributes |= CONF_ENABLE_IRQ;
	}

	/* IO window settings */
#ifdef COMEDI_COMPAT_HAVE_CS_IO_REQ_T
	p_dev->io.NumPorts1 = p_dev->io.NumPorts2 = 0;
	if ((cfg->io.nwin > 0) || (dflt->io.nwin > 0)) {
		cistpl_io_t *io = (cfg->io.nwin) ? &cfg->io : &dflt->io;
		p_dev->io.Attributes1 = IO_DATA_PATH_WIDTH_AUTO;
		if (!(io->flags & CISTPL_IO_8BIT))
			p_dev->io.Attributes1 = IO_DATA_PATH_WIDTH_16;
		if (!(io->flags & CISTPL_IO_16BIT))
			p_dev->io.Attributes1 = IO_DATA_PATH_WIDTH_8;
		p_dev->io.IOAddrLines = io->flags & CISTPL_IO_LINES_MASK;
		p_dev->io.BasePort1 = io->win[0].base;
		p_dev->io.NumPorts1 = io->win[0].len;
		if (io->nwin > 1) {
			p_dev->io.Attributes2 = p_dev->io.Attributes1;
			p_dev->io.BasePort2 = io->win[1].base;
			p_dev->io.NumPorts2 = io->win[1].len;
		}
		/* This reserves IO space but doesn't actually enable it */
		return pcmcia_request_io(p_dev, &p_dev->io);
	}
#else
	p_dev->resource[0]->end = p_dev->resource[1]->end = 0;
	if ((cfg->io.nwin > 0) || (dflt->io.nwin > 0)) {
		cistpl_io_t *io = (cfg->io.nwin) ? &cfg->io : &dflt->io;
		p_dev->io_lines = io->flags & CISTPL_IO_LINES_MASK;
		p_dev->resource[0]->flags &= ~IO_DATA_PATH_WIDTH;
		p_dev->resource[0]->flags |=
			pcmcia_io_cfg_data_width(io->flags);
		p_dev->resource[0]->start = io->win[0].base;
		p_dev->resource[0]->end = io->win[0].len;
		if (io->nwin > 1) {
			p_dev->resource[1]->flags = p_dev->resource[0]->flags;
			p_dev->resource[1]->start = io->win[1].base;
			p_dev->resource[1]->end = io->win[1].len;
		}
		/* This reserves IO space but doesn't actually enable it */
		return pcmcia_request_io(p_dev);
	}
#endif

	return 0;
}
#else	/* COMEDI_COMPAT_HAVE_CS_H */
static int das16cs_pcmcia_config_loop(struct pcmcia_device *p_dev,
				void *priv_data)
{
	if (p_dev->config_index == 0)
		return -EINVAL;

	return pcmcia_request_io(p_dev);
}
#endif	/* COMEDI_COMPAT_HAVE_CS_H */
#endif	/* COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE */

static void das16cs_pcmcia_config(struct pcmcia_device *link)
{
#ifdef COMEDI_COMPAT_HAVE_DS_DEV_NODE_T
	local_info_t *dev = link->priv;
#endif
	int last_ret;
#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	tuple_t tuple;
	cisparse_t parse;
	int last_fn;
	u_char buf[64];
	cistpl_cftable_entry_t dflt = { 0 };
#endif

	DEBUG(0, "das16cs_pcmcia_config(0x%p)\n", link);

#ifndef COMEDI_COMPAT_HAVE_CS_H
	/* Do we need to allocate an interrupt? */
	link->config_flags |= CONF_ENABLE_IRQ | CONF_AUTO_SET_IO;
#endif

#ifdef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	last_ret = pcmcia_loop_config(link, das16cs_pcmcia_config_loop, NULL);
	if (last_ret) {
		dev_warn(&link->dev, "no configuration found\n");
		goto cs_failed;
	}
#else
	/*
	   This reads the card's CONFIG tuple to find its configuration
	   registers.
	 */
	tuple.DesiredTuple = CISTPL_CONFIG;
	tuple.Attributes = 0;
	tuple.TupleData = buf;
	tuple.TupleDataMax = sizeof(buf);
	tuple.TupleOffset = 0;
	last_fn = GetFirstTuple;
	if ((last_ret = pcmcia_get_first_tuple(link, &tuple)) != 0)
		goto cs_failed;
	last_fn = GetTupleData;
	if ((last_ret = pcmcia_get_tuple_data(link, &tuple)) != 0)
		goto cs_failed;
	last_fn = ParseTuple;
	if ((last_ret = pcmcia_parse_tuple(&tuple, &parse)) != 0)
		goto cs_failed;
	link->conf.ConfigBase = parse.config.base;
	link->conf.Present = parse.config.rmask[0];

	/*
	   In this loop, we scan the CIS for configuration table entries,
	   each of which describes a valid card configuration, including
	   voltage, IO window, memory window, and interrupt settings.

	   We make no assumptions about the card to be configured: we use
	   just the information available in the CIS.  In an ideal world,
	   this would work for any PCMCIA card, but it requires a complete
	   and accurate CIS.  In practice, a driver usually "knows" most of
	   these things without consulting the CIS, and most client drivers
	   will only use the CIS to fill in implementation-defined details.
	 */
	tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY;
	last_fn = GetFirstTuple;
	if ((last_ret = pcmcia_get_first_tuple(link, &tuple)) != 0)
		goto cs_failed;
	while (1) {
		cistpl_cftable_entry_t *cfg = &(parse.cftable_entry);
		if (pcmcia_get_tuple_data(link, &tuple))
			goto next_entry;
		if (pcmcia_parse_tuple(&tuple, &parse))
			goto next_entry;

		if (cfg->flags & CISTPL_CFTABLE_DEFAULT)
			dflt = *cfg;
		if (cfg->index == 0)
			goto next_entry;
		link->conf.ConfigIndex = cfg->index;

		/* Does this card need audio output? */
/*	if (cfg->flags & CISTPL_CFTABLE_AUDIO) {
		link->conf.Attributes |= CONF_ENABLE_SPKR;
		link->conf.Status = CCSR_AUDIO_ENA;
	}
*/
		/* Do we need to allocate an interrupt? */
#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
		if (cfg->irq.IRQInfo1 || dflt.irq.IRQInfo1)
#endif
		{
			link->conf.Attributes |= CONF_ENABLE_IRQ;
		}

		/* IO window settings */
#ifdef COMEDI_COMPAT_HAVE_CS_IO_REQ_T
		link->io.NumPorts1 = link->io.NumPorts2 = 0;
		if ((cfg->io.nwin > 0) || (dflt.io.nwin > 0)) {
			cistpl_io_t *io = (cfg->io.nwin) ? &cfg->io : &dflt.io;
			link->io.Attributes1 = IO_DATA_PATH_WIDTH_AUTO;
			if (!(io->flags & CISTPL_IO_8BIT))
				link->io.Attributes1 = IO_DATA_PATH_WIDTH_16;
			if (!(io->flags & CISTPL_IO_16BIT))
				link->io.Attributes1 = IO_DATA_PATH_WIDTH_8;
			link->io.IOAddrLines = io->flags & CISTPL_IO_LINES_MASK;
			link->io.BasePort1 = io->win[0].base;
			link->io.NumPorts1 = io->win[0].len;
			if (io->nwin > 1) {
				link->io.Attributes2 = link->io.Attributes1;
				link->io.BasePort2 = io->win[1].base;
				link->io.NumPorts2 = io->win[1].len;
			}
			/* This reserves IO space but doesn't actually enable it */
			if (pcmcia_request_io(link, &link->io))
				goto next_entry;
		}
#else
		link->resource[0]->end = link->resource[1]->end = 0;
		if ((cfg->io.nwin > 0) || (dflt.io.nwin > 0)) {
			cistpl_io_t *io = (cfg->io.nwin) ? &cfg->io : &dflt.io;
			link->io_lines = io->flags & CISTPL_IO_LINES_MASK;
			link->resource[0]->flags &= ~IO_DATA_PATH_WIDTH;
			link->resource[0]->flags |=
				pcmcia_io_cfg_data_width(io->flags);
			link->resource[0]->start = io->win[0].base;
			link->resource[0]->end = io->win[0].len;
			if (io->nwin > 1) {
				link->resource[1]->flags = p_dev->resource[0]->flags;
				link->resource[1]->start = io->win[1].base;
				link->resource[1]->end = io->win[1].len;
			}
			/* This reserves IO space but doesn't actually enable it */
			if (pcmcia_request_io(link))
				goto next_entry;
		}
#endif

		/* If we got this far, we're cool! */
		break;

	      next_entry:
		last_fn = GetNextTuple;
		if ((last_ret = pcmcia_get_next_tuple(link, &tuple)) != 0)
			goto cs_failed;
	}
#endif

#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
	/*
	   Allocate an interrupt line.  Note that this does not assign a
	   handler to the interrupt, unless the 'Handler' member of the
	   irq structure is initialized.
	 */
	if (link->conf.Attributes & CONF_ENABLE_IRQ) {
#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
		last_fn = RequestIRQ;
#endif
		if ((last_ret = pcmcia_request_irq(link, &link->irq)) != 0)
			goto cs_failed;
	}
#else
	/* Check an interrupt line has been allocated. */
	if (!link->irq)
		goto cs_failed;
#endif
	/*
	   This actually configures the PCMCIA socket -- setting up
	   the I/O windows and the interrupt mapping, and putting the
	   card and host interface into "Memory and IO" mode.
	 */
#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	last_fn = RequestConfiguration;
#endif
#ifdef COMEDI_COMPAT_HAVE_CS_H
	last_ret = pcmcia_request_configuration(link, &link->conf);
#else
	last_ret = pcmcia_enable_device(link);
#endif
	if (last_ret)
		goto cs_failed;

#ifdef COMEDI_COMPAT_HAVE_DS_DEV_NODE_T
	/*
	   At this point, the dev_node_t structure(s) need to be
	   initialized and arranged in a linked list at link->dev.
	 */
	sprintf(dev->node.dev_name, "cb_das16_cs");
	dev->node.major = dev->node.minor = 0;
	link->dev_node = &dev->node;
#endif

#ifdef COMEDI_COMPAT_HAVE_CS_H
	/* Finally, report what we've done */
#ifdef COMEDI_COMPAT_HAVE_DS_DEV_NODE_T
	printk(KERN_INFO "%s: index 0x%02x",
		dev->node.dev_name, link->conf.ConfigIndex);
#else
	dev_info(&link->dev, "index 0x%02x", link->conf.ConfigIndex);
#endif
	if (link->conf.Attributes & CONF_ENABLE_IRQ)
		printk(", irq %u",
#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
			link->irq.AssignedIRQ
#else
			link->irq
#endif
			);

#ifdef COMEDI_COMPAT_HAVE_CS_IO_REQ_T
	if (link->io.NumPorts1)
		printk(", io 0x%04x-0x%04x", link->io.BasePort1,
			link->io.BasePort1 + link->io.NumPorts1 - 1);
	if (link->io.NumPorts2)
		printk(" & 0x%04x-0x%04x", link->io.BasePort2,
			link->io.BasePort2 + link->io.NumPorts2 - 1);
#else
	if (link->resource[0])
		printk(", io %pR", link->resource[0]);
	if (link->resource[1])
		printk(" & %pR", link->resource[1]);
#endif
	printk("\n");
#endif	/* COMEDI_COMPAT_HAVE_CS_H */

	return;

      cs_failed:
#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	cs_error(link, last_fn, last_ret);
#endif
	das16cs_pcmcia_release(link);
}				/* das16cs_pcmcia_config */

static void das16cs_pcmcia_release(struct pcmcia_device *link)
{
	DEBUG(0, "das16cs_pcmcia_release(0x%p)\n", link);
	pcmcia_disable_device(link);
}				/* das16cs_pcmcia_release */

static int das16cs_pcmcia_suspend(struct pcmcia_device *link)
{
	local_info_t *local = link->priv;

	/* Mark the device as stopped, to block IO until later */
	local->stop = 1;

	return 0;
}				/* das16cs_pcmcia_suspend */

static int das16cs_pcmcia_resume(struct pcmcia_device *link)
{
	local_info_t *local = link->priv;

	local->stop = 0;
	return 0;
}				/* das16cs_pcmcia_resume */

/*====================================================================*/

static struct pcmcia_device_id das16cs_id_table[] = {
	PCMCIA_DEVICE_MANF_CARD(0x01c5, 0x0039),
	PCMCIA_DEVICE_MANF_CARD(0x01c5, 0x4009),
	PCMCIA_DEVICE_NULL
};

MODULE_DEVICE_TABLE(pcmcia, das16cs_id_table);

struct pcmcia_driver das16cs_driver = {
	.probe = das16cs_pcmcia_attach,
	.remove = das16cs_pcmcia_detach,
	.suspend = das16cs_pcmcia_suspend,
	.resume = das16cs_pcmcia_resume,
	.id_table = das16cs_id_table,
	.owner = THIS_MODULE,
#ifdef COMEDI_COMPAT_HAVE_PCMCIA_DRIVER_NAME
	.name = devname,
#else
	.drv = {
			.name = devname,
		},
#endif
};

static int __init init_das16cs_pcmcia_cs(void)
{
	DEBUG(0, "%s\n", version);
	pcmcia_register_driver(&das16cs_driver);
	return 0;
}

static void __exit exit_das16cs_pcmcia_cs(void)
{
	DEBUG(0, "das16cs_pcmcia_cs: unloading\n");
	pcmcia_unregister_driver(&das16cs_driver);
}

static int __init das16cs_init_module(void)
{
	int ret;

	ret = init_das16cs_pcmcia_cs();
	if (ret < 0)
		return ret;

	return comedi_driver_register(&driver_das16cs);
}

static void __exit das16cs_exit_module(void)
{
	exit_das16cs_pcmcia_cs();
	comedi_driver_unregister(&driver_das16cs);
}

module_init(das16cs_init_module);
module_exit(das16cs_exit_module);
COMEDI_MODULE_MACROS;
#else
COMEDI_INITCLEANUP(driver_das16cs);
#endif //CONFIG_PCMCIA
