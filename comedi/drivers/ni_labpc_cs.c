/*
    comedi/drivers/ni_labpc_cs.c
    Driver for National Instruments daqcard-1200 boards
    Copyright (C) 2001, 2002, 2003 Frank Mori Hess <fmhess@users.sourceforge.net>

    PCMCIA stuff is adapted from dummy_cs.c 1.31 2001/08/24 12:13:13
    from the pcmcia package.
    The initial developer of the pcmcia dummy_cs.c code is David A. Hinds
    <dahinds@users.sourceforge.net>.  Portions created by David A. Hinds
    are Copyright (C) 1999 David A. Hinds.

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
Driver: ni_labpc_cs
Description: National Instruments Lab-PC (& compatibles)
Author: Frank Mori Hess <fmhess@users.sourceforge.net>
Devices: [National Instruments] DAQCard-1200 (daqcard-1200)
Status: works

Thanks go to Fredrik Lingvall for much testing and perseverance in
helping to debug daqcard-1200 support.

The 1200 series boards have onboard calibration dacs for correcting
analog input/output offsets and gains.  The proper settings for these
caldacs are stored on the board's eeprom.  To read the caldac values
from the eeprom and store them into a file that can be then be used by
comedilib, use the comedi_calibrate program.

Configuration options:
  none

The daqcard-1200 has quirky chanlist requirements
when scanning multiple channels.  Multiple channel scan
sequence must start at highest channel, then decrement down to
channel 0.  Chanlists consisting of all one channel
are also legal, and allow you to pace conversions in bursts.

*/

/*

NI manuals:
340988a (daqcard-1200)

*/

#undef LABPC_DEBUG
//#define LABPC_DEBUG   // enable debugging messages

#include <linux/comedidev.h>

#include <linux/delay.h>
#include <linux/version.h>

#include "8253.h"
#include "8255.h"
#include "comedi_fc.h"
#include "ni_labpc.h"

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

static int labpc_attach(comedi_device * dev, comedi_devconfig * it);

static const labpc_board labpc_cs_boards[] = {
	{
	      name:	"daqcard-1200",
	      device_id:0x103,	// 0x10b is manufacturer id, 0x103 is device id
	      ai_speed:10000,
	      bustype:	pcmcia_bustype,
	      register_layout:labpc_1200_layout,
	      has_ao:	1,
	      ai_range_table:&range_labpc_1200_ai,
	      ai_range_code:labpc_1200_ai_gain_bits,
	      ai_range_is_unipolar:labpc_1200_is_unipolar,
	      ai_scan_up:0,
	      memory_mapped_io:0,
		},
	/* duplicate entry, to support using alternate name */
	{
	      name:	"ni_labpc_cs",
	      device_id:0x103,
	      ai_speed:10000,
	      bustype:	pcmcia_bustype,
	      register_layout:labpc_1200_layout,
	      has_ao:	1,
	      ai_range_table:&range_labpc_1200_ai,
	      ai_range_code:labpc_1200_ai_gain_bits,
	      ai_range_is_unipolar:labpc_1200_is_unipolar,
	      ai_scan_up:0,
	      memory_mapped_io:0,
		},
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((const labpc_board *)dev->board_ptr)

static comedi_driver driver_labpc_cs = {
	.driver_name = "ni_labpc_cs",
	.module = THIS_MODULE,
	.attach = &labpc_attach,
	.detach = &labpc_common_detach,
	.num_names = sizeof(labpc_cs_boards) / sizeof(labpc_board),
	.board_name = &labpc_cs_boards[0].name,
	.offset = sizeof(labpc_board),
};

static int labpc_attach(comedi_device * dev, comedi_devconfig * it)
{
	unsigned long iobase = 0;
	unsigned int irq = 0;
	struct pcmcia_device *link;

	/* allocate and initialize dev->private */
	if (alloc_private(dev, sizeof(labpc_private)) < 0)
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
#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
		irq = link->irq.AssignedIRQ;
#else
		irq = link->irq;
#endif
		break;
	default:
		printk("bug! couldn't determine board type\n");
		return -EINVAL;
		break;
	}
	return labpc_common_attach(dev, iobase, irq, 0);
}

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
static const char *version =
	"ni_labpc.c, based on dummy_cs.c 1.31 2001/08/24 12:13:13";
#else
#define DEBUG(n, args...)
#endif

/*====================================================================*/

/*
   The event() function is this driver's Card Services event handler.
   It will be called by Card Services when an appropriate card status
   event is received.  The config() and release() entry points are
   used to configure or release a socket, in response to card
   insertion and ejection events.  They are invoked from the dummy
   event handler.

   Kernel version 2.6.16 upwards uses suspend() and resume() functions
   instead of an event() function.
*/

static void labpc_config(struct pcmcia_device *link);
static void labpc_release(struct pcmcia_device *link);
static int labpc_cs_suspend(struct pcmcia_device *p_dev);
static int labpc_cs_resume(struct pcmcia_device *p_dev);

/*
   The attach() and detach() entry points are used to create and destroy
   "instances" of the driver, where each instance represents everything
   needed to manage one actual PCMCIA card.
*/

static int labpc_cs_attach(struct pcmcia_device *);
static void labpc_cs_detach(struct pcmcia_device *);

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
static const dev_info_t devname = "daqcard-1200";
#else
static const char devname[] = "daqcard-1200";
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

    labpc_cs_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.

======================================================================*/

static int labpc_cs_attach(struct pcmcia_device *link)
{
	local_info_t *local;

	DEBUG(0, "labpc_cs_attach()\n");

	/* Allocate space for private device-specific data */
	local = kzalloc(sizeof(local_info_t), GFP_KERNEL);
	if (!local)
		return -ENOMEM;
	local->link = link;
	link->priv = local;

#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
	/* Interrupt setup */
	link->irq.Attributes = IRQ_TYPE_DYNAMIC_SHARING | IRQ_FORCED_PULSE;
#ifndef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	link->irq.IRQInfo1 = IRQ_INFO2_VALID | IRQ_PULSE_ID;
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

	labpc_config(link);

	return 0;
}				/* labpc_cs_attach */

/*======================================================================

    This deletes a driver "instance".  The device is de-registered
    with Card Services.  If it has been released, all local data
    structures are freed.  Otherwise, the structures will be freed
    when the device is released.

======================================================================*/

static void labpc_cs_detach(struct pcmcia_device *link)
{
	DEBUG(0, "labpc_cs_detach(0x%p)\n", link);

	/*
	   If the device is currently configured and active, we won't
	   actually delete it yet.  Instead, it is marked so that when
	   the release() function is called, that will trigger a proper
	   detach().
	 */
#ifdef COMEDI_COMPAT_HAVE_DS_DEV_NODE_T
	if (link->dev_node)
#endif
	{
		((local_info_t *) link->priv)->stop = 1;
		labpc_release(link);
	}

	/* This points to the parent local_info_t struct */
	if (link->priv)
		kfree(link->priv);

}				/* labpc_cs_detach */

/*======================================================================

    labpc_config() is scheduled to run after a CARD_INSERTION event
    is received, to configure the PCMCIA socket, and to make the
    device available to the system.

======================================================================*/

#ifdef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
#ifdef COMEDI_COMPAT_HAVE_CS_H
static int labpc_pcmcia_config_loop(struct pcmcia_device *p_dev,
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
		p_dev->conf.Attributes |= CONF_ENABLE_IRQ;
#else
	p_dev->conf.Attributes |= CONF_ENABLE_IRQ | CONF_ENABLE_PULSE_IRQ;
#endif

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
static int labpc_pcmcia_config_loop(struct pcmcia_device *p_dev,
				void *priv_data)
{
	if (p_dev->config_index == 0)
		return -EINVAL;

	return pcmcia_request_io(p_dev);
}
#endif	/* COMEDI_COMPAT_HAVE_CS_H */
#endif	/* COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE */

static void labpc_config(struct pcmcia_device *link)
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

	DEBUG(0, "labpc_config(0x%p)\n", link);

#ifndef COMEDI_COMPAT_HAVE_CS_H
	link->config_flags |= CONF_ENABLE_IRQ | CONF_ENABLE_PULSE_IRQ |
		CONF_AUTO_AUDIO | CONF_AUTO_SET_IO;
#endif

#ifdef COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE
	last_ret = pcmcia_loop_config(link, labpc_pcmcia_config_loop,
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
	if ((last_ret = pcmcia_get_first_tuple(link, &tuple))) {
		goto cs_failed;
	}
	last_fn = GetTupleData;
	if ((last_ret = pcmcia_get_tuple_data(link, &tuple))) {
		goto cs_failed;
	}
	last_fn = ParseTuple;
	if ((last_ret = pcmcia_parse_tuple(&tuple, &parse))) {
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
	if ((last_ret = pcmcia_get_first_tuple(link, &tuple))) {
		goto cs_failed;
	}
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
		if (cfg->flags & CISTPL_CFTABLE_AUDIO) {
			link->conf.Attributes |= CONF_ENABLE_SPKR;
			link->conf.Status = CCSR_AUDIO_ENA;
		}

		/* Do we need to allocate an interrupt? */
#ifdef COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T
		if (cfg->irq.IRQInfo1 || dflt.irq.IRQInfo1)
			link->conf.Attributes |= CONF_ENABLE_IRQ;
#else
		link->conf.Attributes |= CONF_ENABLE_IRQ | CONF_ENABLE_PULSE_IRQ;
#endif

		/* IO window settings */
#ifdef COMEDI_COMPAT_HAVE_CS_IO_REQ_T
		link->io.NumPorts1 = link->io.NumPorts2 = 0;
		if ((cfg->io.nwin > 0) || (dflt.io.nwin > 0)) {
			cistpl_io_t *io = (cfg->io.nwin) ? &cfg->io : &dflt.io;
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
			link->win = (window_handle_t) link;
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
		if ((last_ret = pcmcia_get_next_tuple(link, &tuple))) {
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
		if ((last_ret = pcmcia_request_irq(link, &link->irq))) {
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
	sprintf(dev->node.dev_name, "daqcard-1200");
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
	labpc_release(link);

}				/* labpc_config */

static void labpc_release(struct pcmcia_device *link)
{
	DEBUG(0, "labpc_release(0x%p)\n", link);

	pcmcia_disable_device(link);
}				/* labpc_release */

/*======================================================================

    The card status event handler.  Mostly, this schedules other
    stuff to run after an event is received.

    When a CARD_REMOVAL event is received, we immediately set a
    private flag to block future accesses to this device.  All the
    functions that actually access the device should check this flag
    to make sure the card is still present.

======================================================================*/

static int labpc_cs_suspend(struct pcmcia_device *link)
{
	local_info_t *local = link->priv;

	/* Mark the device as stopped, to block IO until later */
	local->stop = 1;
	return 0;
}				/* labpc_cs_suspend */

static int labpc_cs_resume(struct pcmcia_device *link)
{
	local_info_t *local = link->priv;

	local->stop = 0;
	return 0;
}				/* labpc_cs_resume */

/*====================================================================*/

static struct pcmcia_device_id labpc_cs_ids[] = {
	/* N.B. These IDs should match those in labpc_cs_boards (ni_labpc.c) */
	PCMCIA_DEVICE_MANF_CARD(0x010b, 0x0103),	/* daqcard-1200 */
	PCMCIA_DEVICE_NULL
};

MODULE_DEVICE_TABLE(pcmcia, labpc_cs_ids);

struct pcmcia_driver labpc_cs_driver = {
	.probe = labpc_cs_attach,
	.remove = labpc_cs_detach,
	.suspend = labpc_cs_suspend,
	.resume = labpc_cs_resume,
	.id_table = labpc_cs_ids,
	.owner = THIS_MODULE,
#ifdef COMEDI_COMPAT_HAVE_PCMCIA_DRIVER_NAME
	.name = devname,
#else
	.drv = {
			.name = devname,
		},
#endif
};

static int __init init_labpc_cs(void)
{
	DEBUG(0, "%s\n", version);
	pcmcia_register_driver(&labpc_cs_driver);
	return 0;
}

static void __exit exit_labpc_cs(void)
{
	DEBUG(0, "ni_labpc: unloading\n");
	pcmcia_unregister_driver(&labpc_cs_driver);
}

static int __init labpc_init_module(void)
{
	int ret;

	ret = init_labpc_cs();
	if (ret < 0)
		return ret;

	return comedi_driver_register(&driver_labpc_cs);
}

static void __exit labpc_exit_module(void)
{
	exit_labpc_cs();
	comedi_driver_unregister(&driver_labpc_cs);
}

MODULE_DESCRIPTION("Comedi driver for National Instruments Lab-PC");
MODULE_AUTHOR("Frank Mori Hess <fmhess@users.sourceforge.net>");
MODULE_LICENSE("GPL");
module_init(labpc_init_module);
module_exit(labpc_exit_module);
