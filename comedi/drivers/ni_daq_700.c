/*
 *     comedi/drivers/ni_daq_700.c
 *     Driver for DAQCard-700 DIO only
 *     copied from 8255
 *
 *     COMEDI - Linux Control and Measurement Device Interface
 *     Copyright (C) 1998 David A. Schleef <ds@schleef.org>
 *
 *     This program is free software; you can redistribute it and/or modify
 *     it under the terms of the GNU General Public License as published by
 *     the Free Software Foundation; either version 2 of the License, or
 *     (at your option) any later version.
 *
 *     This program is distributed in the hope that it will be useful,
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *     GNU General Public License for more details.
 *
 *     You should have received a copy of the GNU General Public License
 *     along with this program; if not, write to the Free Software
 *     Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

/*
Driver: ni_daq_700
Description: National Instruments PCMCIA DAQCard-700 DIO only
Author: Fred Brooks <nsaspook@nsaspook.com>,
  based on ni_daq_dio24 by Daniel Vecino Castel <dvecino@able.es>
Devices: [National Instruments] PCMCIA DAQ-Card-700 (ni_daq_700)
Status: works
Updated: Thu, 21 Feb 2008 12:07:20 +0000

The daqcard-700 appears in Comedi as a single digital I/O subdevice with
16 channels.  The channel 0 corresponds to the daqcard-700's output
port, bit 0; channel 8 corresponds to the input port, bit 0.

Direction configuration: channels 0-7 output, 8-15 input (8225 device
emu as port A output, port B input, port C N/A).

IRQ is assigned but not used.
*/

#include <linux/comedidev.h>

#include <linux/ioport.h>
#include <linux/version.h>

#ifdef COMEDI_COMPAT_HAVE_CS_TYPES_H
#include <pcmcia/cs_types.h>
#endif
#ifdef COMEDI_COMPAT_HAVE_CS_H
#include <pcmcia/cs.h>
#endif
#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ds.h>

static struct pcmcia_device *pcmcia_cur_dev = NULL;

#define DIO700_SIZE 8		// size of io region used by board

static int dio700_attach(comedi_device * dev, comedi_devconfig * it);
static int dio700_detach(comedi_device * dev);

enum dio700_bustype { pcmcia_bustype };

typedef struct dio700_board_struct {
	const char *name;
	int device_id;		// device id for pcmcia board
	enum dio700_bustype bustype;	// PCMCIA
	int have_dio;		// have daqcard-700 dio
	// function pointers so we can use inb/outb or readb/writeb
	// as appropriate
	unsigned int (*read_byte) (unsigned int address);
	void (*write_byte) (unsigned int byte, unsigned int address);
} dio700_board;

static const dio700_board dio700_boards[] = {
	{
	      name:	"daqcard-700",
	      device_id:0x4743,// 0x10b is manufacturer id, 0x4743 is device id
	      bustype:	pcmcia_bustype,
	      have_dio:1,
		},
	{
	      name:	"ni_daq_700",
	      device_id:0x4743,// 0x10b is manufacturer id, 0x4743 is device id
	      bustype:	pcmcia_bustype,
	      have_dio:1,
		},
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((const dio700_board *)dev->board_ptr)

typedef struct {
	int data;		/* number of data points left to be taken */
} dio700_private;

#define devpriv ((dio700_private *)dev->private)

static comedi_driver driver_dio700 = {
      driver_name:"ni_daq_700",
      module:THIS_MODULE,
      attach:dio700_attach,
      detach:dio700_detach,
      num_names:sizeof(dio700_boards) / sizeof(dio700_board),
      board_name:&dio700_boards[0].name,
      offset:sizeof(dio700_board),
};

/*	the real driver routines	*/

#define _700_SIZE 8

#define _700_DATA 0

#define DIO_W		0x04
#define DIO_R		0x05

struct subdev_700_struct {
	unsigned long cb_arg;
	int (*cb_func) (int, int, int, unsigned long);
	int have_irq;
};

#define CALLBACK_ARG	(((struct subdev_700_struct *)s->private)->cb_arg)
#define CALLBACK_FUNC	(((struct subdev_700_struct *)s->private)->cb_func)
#define subdevpriv	((struct subdev_700_struct *)s->private)

static void do_config(comedi_device * dev, comedi_subdevice * s);

#ifdef incomplete
static void subdev_700_interrupt(comedi_device * dev, comedi_subdevice * s)
{
	sampl_t d;

	d = CALLBACK_FUNC(0, _700_DATA, 0, CALLBACK_ARG);

	comedi_buf_put(s->async, d);
	s->async->events |= COMEDI_CB_EOS;

	comedi_event(dev, s);
}
#endif

static int subdev_700_cb(int dir, int port, int data, unsigned long arg)
{
	/* port is always A for output and B for input (8255 emu) */
	unsigned long iobase = arg;

	if (dir) {
		outb(data, iobase + DIO_W);
		return 0;
	} else {
		return inb(iobase + DIO_R);
	}
}

static int subdev_700_insn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	if (data[0]) {
		s->state &= ~data[0];
		s->state |= (data[0] & data[1]);

		if (data[0] & 0xff)
			CALLBACK_FUNC(1, _700_DATA, s->state & 0xff,
				CALLBACK_ARG);
	}

	data[1] = s->state & 0xff;
	data[1] |= CALLBACK_FUNC(0, _700_DATA, 0, CALLBACK_ARG) << 8;

	return 2;
}

static int subdev_700_insn_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{

	switch (data[0]) {
	case INSN_CONFIG_DIO_INPUT:
		break;
	case INSN_CONFIG_DIO_OUTPUT:
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] =
			(s->io_bits & (1 << CR_CHAN(insn->
					chanspec))) ? COMEDI_OUTPUT :
			COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
	}

	return 1;
}

static void do_config(comedi_device * dev, comedi_subdevice * s)
{				/* use powerup defaults */
	return;
}

#ifdef incomplete
static int subdev_700_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd)
{
	int err = 0;
	unsigned int tmp;

	/* step 1 */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_EXT;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_FOLLOW;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_NONE;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/* step 2 */

	if (err)
		return 2;

	/* step 3 */

	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		err++;
	}
	if (cmd->scan_begin_arg != 0) {
		cmd->scan_begin_arg = 0;
		err++;
	}
	if (cmd->convert_arg != 0) {
		cmd->convert_arg = 0;
		err++;
	}
	if (cmd->scan_end_arg != 1) {
		cmd->scan_end_arg = 1;
		err++;
	}
	if (cmd->stop_arg != 0) {
		cmd->stop_arg = 0;
		err++;
	}

	if (err)
		return 3;

	/* step 4 */

	if (err)
		return 4;

	return 0;
}
#endif

#ifdef incomplete
static int subdev_700_cmd(comedi_device * dev, comedi_subdevice * s)
{
	/* FIXME */

	return 0;
}
#endif

#ifdef incomplete
static int subdev_700_cancel(comedi_device * dev, comedi_subdevice * s)
{
	/* FIXME */

	return 0;
}
#endif

static int subdev_700_init(comedi_device * dev, comedi_subdevice * s,
	int (*cb) (int, int, int, unsigned long), unsigned long arg)
{
	s->type = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
	s->n_chan = 16;
	s->range_table = &range_digital;
	s->maxdata = 1;

	s->private = kmalloc(sizeof(struct subdev_700_struct), GFP_KERNEL);
	if (!s->private)
		return -ENOMEM;

	CALLBACK_ARG = arg;
	if (cb == NULL) {
		CALLBACK_FUNC = subdev_700_cb;
	} else {
		CALLBACK_FUNC = cb;
	}
	s->insn_bits = subdev_700_insn;
	s->insn_config = subdev_700_insn_config;

	s->state = 0;
	s->io_bits = 0x00ff;
	do_config(dev, s);

	return 0;
}

#ifdef incomplete
static int subdev_700_init_irq(comedi_device * dev, comedi_subdevice * s,
	int (*cb) (int, int, int, unsigned long), unsigned long arg)
{
	int ret;

	ret = subdev_700_init(dev, s, cb, arg);
	if (ret < 0)
		return ret;

	s->do_cmdtest = subdev_700_cmdtest;
	s->do_cmd = subdev_700_cmd;
	s->cancel = subdev_700_cancel;
	s->subdev_flags |= SDF_CMD_READ;

	subdevpriv->have_irq = 1;

	return 0;
}
#endif

static void subdev_700_cleanup(comedi_device * dev, comedi_subdevice * s)
{
	if (s->private) {
		if (subdevpriv->have_irq) {
		}

		kfree(s->private);
	}
}

static int dio700_attach(comedi_device * dev, comedi_devconfig * it)
{
	comedi_subdevice *s;
	unsigned long iobase = 0;
#ifdef incomplete
	unsigned int irq = 0;
#endif
	struct pcmcia_device *link;

	/* allocate and initialize dev->private */
	if (alloc_private(dev, sizeof(dio700_private)) < 0)
		return -ENOMEM;

	// get base address, irq etc. based on bustype
	switch (thisboard->bustype) {
	case pcmcia_bustype:
		link = pcmcia_cur_dev;	/* XXX hack */
		if (!link)
			return -EIO;
#ifdef COMEDI_COMPAT_HAVE_CS_IO_REQ_T
		iobase = link->io.BasePort1;
#else
		iobase = link->resource[0]->start;
#endif
#ifdef incomplete
#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
		irq = link->irq.AssignedIRQ;
#else
		irq = link->irq;
#endif
#endif
		break;
	default:
		printk("bug! couldn't determine board type\n");
		return -EINVAL;
		break;
	}
	printk("comedi%d: ni_daq_700: %s, io 0x%lx", dev->minor,
		thisboard->name, iobase);
#ifdef incomplete
	if (irq) {
		printk(", irq %u", irq);
	}
#endif

	printk("\n");

	if (iobase == 0) {
		printk("io base address is zero!\n");
		return -EINVAL;
	}

	dev->iobase = iobase;

#ifdef incomplete
	/* grab our IRQ */
	dev->irq = irq;
#endif

	dev->board_name = thisboard->name;

	if (alloc_subdevices(dev, 1) < 0)
		return -ENOMEM;

	/* DAQCard-700 dio */
	s = dev->subdevices + 0;
	subdev_700_init(dev, s, NULL, dev->iobase);

	return 0;
};

static int dio700_detach(comedi_device * dev)
{
	printk("comedi%d: ni_daq_700: cs-remove\n", dev->minor);

	if (dev->subdevices)
		subdev_700_cleanup(dev, dev->subdevices + 0);

	if (dev->irq)
		comedi_free_irq(dev->irq, dev);
	if (thisboard->bustype != pcmcia_bustype && dev->iobase)
		release_region(dev->iobase, DIO700_SIZE);

	return 0;
};

/*
   All the PCMCIA modules use PCMCIA_DEBUG to control debugging.  If
   you do not define PCMCIA_DEBUG at all, all the debug code will be
   left out.  If you compile with PCMCIA_DEBUG=0, the debug code will
   be present but disabled -- but it can then be enabled for specific
   modules at load time with a 'pc_debug=#' option to insmod.
*/
#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
module_param(pc_debug, int, 0644);
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
static char *version = "ni_daq_700.c, based on dummy_cs.c";
#else
#define DEBUG(n, args...)
#endif

/*====================================================================*/

static void dio700_config(struct pcmcia_device *link);
static void dio700_release(struct pcmcia_device *link);
static int dio700_cs_suspend(struct pcmcia_device *p_dev);
static int dio700_cs_resume(struct pcmcia_device *p_dev);

/*
   The attach() and detach() entry points are used to create and destroy
   "instances" of the driver, where each instance represents everything
   needed to manage one actual PCMCIA card.
*/

static int dio700_cs_attach(struct pcmcia_device *);
static void dio700_cs_detach(struct pcmcia_device *);

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
static const dev_info_t devname = "ni_daq_700";
#else
static const char devname[] = "ni_daq_700";
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

    dio700_cs_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.

======================================================================*/

static int dio700_cs_attach(struct pcmcia_device *link)
{
	local_info_t *local;

	printk(KERN_INFO "ni_daq_700:  cs-attach\n");

	DEBUG(0, "dio700_cs_attach()\n");

	/* Allocate space for private device-specific data */
	local = kzalloc(sizeof(local_info_t), GFP_KERNEL);
	if (!local)
		return -ENOMEM;
	local->link = link;
	link->priv = local;

#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
	/* Interrupt setup */
	link->irq.Attributes = IRQ_TYPE_DYNAMIC_SHARING;
#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	link->irq.IRQInfo1 = IRQ_LEVEL_ID;
#endif
	link->irq.Handler = NULL;
#endif

#ifdef COMEDI_COMPAT_HAVE_CS_H
	/*
	   General socket configuration defaults can go here.  In this
	   client, we assume very little, and rely on the CIS for almost
	   everything.  In most clients, many details (i.e., number, sizes,
	   and attributes of IO windows) are fixed by the nature of the
	   device, and can be hard-wired here.
	 */
	link->conf.Attributes = 0;
	link->conf.IntType = INT_MEMORY_AND_IO;
#endif

	pcmcia_cur_dev = link;

	dio700_config(link);

	return 0;
}				/* dio700_cs_attach */

/*======================================================================

    This deletes a driver "instance".  The device is de-registered
    with Card Services.  If it has been released, all local data
    structures are freed.  Otherwise, the structures will be freed
    when the device is released.

======================================================================*/

static void dio700_cs_detach(struct pcmcia_device *link)
{

	printk(KERN_INFO "ni_daq_700: cs-detach!\n");

	DEBUG(0, "dio700_cs_detach(0x%p)\n", link);

#ifdef COMEDI_COMPAT_HAVE_DS_DEV_NODE_T
	if (link->dev_node)
#endif
	{
		((local_info_t *) link->priv)->stop = 1;
		dio700_release(link);
	}

	/* This points to the parent local_info_t struct */
	if (link->priv)
		kfree(link->priv);

}				/* dio700_cs_detach */

/*======================================================================

    dio700_config() is scheduled to run after a CARD_INSERTION event
    is received, to configure the PCMCIA socket, and to make the
    device available to the system.

======================================================================*/

#ifdef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
#ifdef COMEDI_COMPAT_HAVE_CS_H
static int dio700_pcmcia_config_loop(struct pcmcia_device *p_dev,
				cistpl_cftable_entry_t *cfg,
				cistpl_cftable_entry_t *dflt,
				unsigned int vcc,
				void *priv_data)
{
	win_req_t *req = priv_data;
#ifdef COMEDI_COMPAT_HAVE_CS_MEMREQ_T
	memreq_t map;
#endif

	if (cfg->index == 0)
		return -ENODEV;

	/* Does this card need audio output? */
	if (cfg->flags & CISTPL_CFTABLE_AUDIO) {
		p_dev->conf.Attributes |= CONF_ENABLE_SPKR;
		p_dev->conf.Status = CCSR_AUDIO_ENA;
	}

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
		if (pcmcia_request_io(p_dev, &p_dev->io) != 0)
			return -ENODEV;
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
		if (pcmcia_request_io(p_dev) != 0)
			return -ENODEV;
	}
#endif

	if ((cfg->mem.nwin > 0) || (dflt->mem.nwin > 0)) {
		cistpl_mem_t *mem =
			(cfg->mem.nwin) ? &cfg->mem : &dflt->mem;
		req->Attributes = WIN_DATA_WIDTH_16 | WIN_MEMORY_TYPE_CM;
		req->Attributes |= WIN_ENABLE;
		req->Base = mem->win[0].host_addr;
		req->Size = mem->win[0].len;
		if (req->Size < 0x1000)
			req->Size = 0x1000;
		req->AccessSpeed = 0;
		if (pcmcia_request_window(p_dev, req, &p_dev->win))
			return -ENODEV;
#ifdef COMEDI_COMPAT_HAVE_CS_MEMREQ_T
		map.Page = 0;
		map.CardOffset = mem->win[0].card_addr;
#endif
		if (pcmcia_map_mem_page(p_dev, p_dev->win,
#ifdef COMEDI_COMPAT_HAVE_CS_MEMREQ_T
					&map
#else
					mem->win[0].card_addr
#endif
					))
			return -ENODEV;
	}
	/* If we got this far, we're cool! */
	return 0;
}
#else	/* COMEDI_COMPAT_HAVE_CS_H */
static int dio700_pcmcia_config_loop(struct pcmcia_device *p_dev,
				void *priv_data)
{
	if (p_dev->config_index == 0)
		return -EINVAL;

	return pcmcia_request_io(p_dev);
}
#endif	/* COMEDI_COMPAT_HAVE_CS_H */
#endif	/* COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE */

static void dio700_config(struct pcmcia_device *link)
{
#ifdef COMEDI_COMPAT_HAVE_DS_DEV_NODE_T
	local_info_t *dev = link->priv;
#endif
	int last_ret;
#ifdef COMEDI_COMPAT_HAVE_CS_H
	win_req_t req;
#endif
#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	tuple_t tuple;
	cisparse_t parse;
	int last_fn;
	u_char buf[64];
#ifdef COMEDI_COMPAT_HAVE_CS_MEMREQ_T
	memreq_t map;
#endif
	cistpl_cftable_entry_t dflt = { 0 };
#endif

	printk(KERN_INFO "ni_daq_700:  cs-config\n");

	DEBUG(0, "dio700_config(0x%p)\n", link);

#ifndef COMEDI_COMPAT_HAVE_CS_H
	link->config_flags |= CONF_ENABLE_IRQ | CONF_AUTO_AUDIO |
		CONF_AUTO_SET_IO;
#endif

#ifdef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	last_ret = pcmcia_loop_config(link, dio700_pcmcia_config_loop,
#ifdef COMEDI_COMPAT_HAVE_CS_H
			&req
#else
			NULL
#endif
			);
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
	if ((last_ret = pcmcia_get_first_tuple(link, &tuple)) != 0) {
		goto cs_failed;
	}
	last_fn = GetTupleData;
	if ((last_ret = pcmcia_get_tuple_data(link, &tuple)) != 0) {
		goto cs_failed;
	}
	last_fn = ParseTuple;
	if ((last_ret = pcmcia_parse_tuple(&tuple, &parse)) != 0) {
		goto cs_failed;
	}
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
	if ((last_ret = pcmcia_get_first_tuple(link, &tuple)) != 0) {
		goto cs_failed;
	}
	while (1) {
		cistpl_cftable_entry_t *cfg = &(parse.cftable_entry);
		if (pcmcia_get_tuple_data(link, &tuple) != 0)
			goto next_entry;
		if (pcmcia_parse_tuple(&tuple, &parse) != 0)
			goto next_entry;

		if (cfg->flags & CISTPL_CFTABLE_DEFAULT)
			dflt = *cfg;
		if (cfg->index == 0)
			goto next_entry;
		link->conf.ConfigIndex = cfg->index;

		/* Does this card need audio output? */
		if (cfg->flags & CISTPL_CFTABLE_AUDIO) {
			link->conf.Attributes |= CONF_ENABLE_SPKR;
			link->conf.Status = CCSR_AUDIO_ENA;
		}

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
			if (pcmcia_request_io(link, &link->io) != 0)
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
			if (pcmcia_request_io(link) != 0)
				goto next_entry;
		}
#endif

		if ((cfg->mem.nwin > 0) || (dflt.mem.nwin > 0)) {
			cistpl_mem_t *mem =
				(cfg->mem.nwin) ? &cfg->mem : &dflt.mem;
			req.Attributes = WIN_DATA_WIDTH_16 | WIN_MEMORY_TYPE_CM;
			req.Attributes |= WIN_ENABLE;
			req.Base = mem->win[0].host_addr;
			req.Size = mem->win[0].len;
			if (req.Size < 0x1000)
				req.Size = 0x1000;
			req.AccessSpeed = 0;
			if (pcmcia_request_window(&link, &req, &link->win))
				goto next_entry;
#ifdef COMEDI_COMPAT_HAVE_CS_MEMREQ_T
			map.Page = 0;
			map.CardOffset = mem->win[0].card_addr;
#endif
			if (pcmcia_map_mem_page(link->win,
#ifdef COMEDI_COMPAT_HAVE_CS_MEMREQ_T
						&map
#else
						mem->win[0].card_addr
#endif
						))
				goto next_entry;
		}
		/* If we got this far, we're cool! */
		break;

	      next_entry:
		last_fn = GetNextTuple;
		if ((last_ret = pcmcia_get_next_tuple(link, &tuple)) != 0) {
			goto cs_failed;
		}
	}
#endif

#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
	/*
	   Allocate an interrupt line.  Note that this does not assign a
	   handler to the interrupt, unless the 'Handler' member of the
	   irq structure is initialized.
	 */
	if (link->conf.Attributes & CONF_ENABLE_IRQ)
#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
		last_fn = RequestIRQ;
#endif
		if ((last_ret = pcmcia_request_irq(link, &link->irq)) != 0) {
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
	sprintf(dev->node.dev_name, "ni_daq_700");
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
	if (link->win)
		printk(", mem 0x%06lx-0x%06lx", req.Base,
			req.Base + req.Size - 1);
	printk("\n");
#endif	/* COMEDI_COMPAT_HAVE_CS_H */

	return;

      cs_failed:
#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	cs_error(link, last_fn, last_ret);
#endif
	printk(KERN_INFO "ni_daq_700 cs failed");
	dio700_release(link);

}				/* dio700_config */

static void dio700_release(struct pcmcia_device *link)
{
	DEBUG(0, "dio700_release(0x%p)\n", link);

	pcmcia_disable_device(link);
}				/* dio700_release */

/*======================================================================

    The card status event handler.  Mostly, this schedules other
    stuff to run after an event is received.

    When a CARD_REMOVAL event is received, we immediately set a
    private flag to block future accesses to this device.  All the
    functions that actually access the device should check this flag
    to make sure the card is still present.

======================================================================*/

static int dio700_cs_suspend(struct pcmcia_device *link)
{
	local_info_t *local = link->priv;

	/* Mark the device as stopped, to block IO until later */
	local->stop = 1;
	return 0;
}				/* dio700_cs_suspend */

static int dio700_cs_resume(struct pcmcia_device *link)
{
	local_info_t *local = link->priv;

	local->stop = 0;
	return 0;
}				/* dio700_cs_resume */

/*====================================================================*/

static struct pcmcia_device_id dio700_cs_ids[] = {
	/* N.B. These IDs should match those in dio700_boards */
	PCMCIA_DEVICE_MANF_CARD(0x010b, 0x4743),	/* daqcard-700 */
	PCMCIA_DEVICE_NULL
};

MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(pcmcia, dio700_cs_ids);

struct pcmcia_driver dio700_cs_driver = {
	.probe = dio700_cs_attach,
	.remove = dio700_cs_detach,
	.suspend = dio700_cs_suspend,
	.resume = dio700_cs_resume,
	.id_table = dio700_cs_ids,
	.owner = THIS_MODULE,
#ifdef COMEDI_COMPAT_HAVE_PCMCIA_DRIVER_NAME
	.name = devname,
#else
	.drv = {
			.name = devname,
		},
#endif
};

static int __init init_dio700_cs(void)
{
	printk("ni_daq_700:  cs-init \n");
	DEBUG(0, "%s\n", version);
	pcmcia_register_driver(&dio700_cs_driver);
	return 0;
}

static void __exit exit_dio700_cs(void)
{
	DEBUG(0, "ni_daq_700: unloading\n");
	pcmcia_unregister_driver(&dio700_cs_driver);
}
static int __init dio700_init_module(void)
{
	int ret;

	ret = init_dio700_cs();
	if (ret < 0)
		return ret;

	return comedi_driver_register(&driver_dio700);
}

static void __exit dio700_exit_module(void)
{
	exit_dio700_cs();
	comedi_driver_unregister(&driver_dio700);
}

module_init(dio700_init_module);
module_exit(dio700_exit_module);
