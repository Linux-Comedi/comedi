/*
    ni_daq_dio24.c driver for National Instruments PCMCIA DAQ-Card DIO-24
    Copyright (C) 2002 Daniel Vecino Castel <dvecino@able.es>

    PCMCIA crap at end of file is adapted from dummy_cs.c 1.31 2001/08/24 12:13:13
    from the pcmcia package.
    The initial developer of the pcmcia dummy_cs.c code is David A. Hinds
    <dahinds@users.sourceforge.net>.  Portions created by David A. Hinds
    are Copyright (C) 1999 David A. Hinds.  All Rights Reserved.

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
Driver: ni_daq_dio24.o
Description: National Instruments PCMCIA DAQ-Card DIO-24
Author: Daniel Vecino Castel <dvecino@able.es>
Devices: National Instruments PCMCIA DAQ-Card DIO-24
Status: ?
Updated: Thu, 07 Nov 2002 21:53:06 -0800

This is just a wrapper around the 8255.o driver to properly handle
the PCMCIA interface.
*/


//#define LABPC_DEBUG	// enable debugging messages
#undef LABPC_DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/comedidev.h>
#include "8255.h"

#include <linux/ptrace.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/system.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ds.h>
#include <pcmcia/bus_ops.h>

/*
   A linked list of "instances" of the dummy device.  Each actual
   PCMCIA card corresponds to one device instance, and is described
   by one dev_link_t structure (defined in ds.h).

   You may not want to use a linked list for this -- for example, the
   memory card driver uses an array of dev_link_t pointers, where minor
   device numbers are used to derive the corresponding array index.
*/

static dev_link_t *pcmcia_dev_list = NULL;

#define DIO24_SIZE 4	// size of io region used by board

static int dio24_attach(comedi_device *dev,comedi_devconfig *it);
static int dio24_detach(comedi_device *dev);

enum dio24_bustype {pcmcia_bustype};

typedef struct dio24_board_struct{
	char *name;
	int device_id;	// device id for pcmcia board
	enum dio24_bustype bustype;	// PCMCIA
	int have_dio;	// have 8255 chip
	// function pointers so we can use inb/outb or readb/writeb as appropriate
	unsigned int (*read_byte)(unsigned int address);
	void (*write_byte)(unsigned int byte, unsigned int address);
}dio24_board;


static dio24_board dio24_boards[] =
{
	{
		name:	"daqcard-dio24",
		device_id:	0x475c,	// 0x10b is manufacturer id, 0x475c is device id
		bustype:	pcmcia_bustype,
		have_dio:	1,
	},
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((dio24_board *)dev->board_ptr)

typedef struct{
	int data;  /* number of data points left to be taken */
}dio24_private;

#define devpriv ((dio24_private *)dev->private)

static comedi_driver driver_dio24={
	driver_name:	"ni_daq_dio24",
	module:		THIS_MODULE,
	attach:		dio24_attach,
	detach:		dio24_detach,
	num_names:	sizeof(dio24_boards) / sizeof(dio24_board),
	board_name:	(char **)dio24_boards,
	offset:		sizeof(dio24_board),
};

static int dio24_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int iobase = 0;
	int irq = 0;
	dev_link_t *link;

	/* allocate and initialize dev->private */
	if(alloc_private(dev, sizeof(dio24_private)) < 0)
		return -ENOMEM;

	// get base address, irq etc. based on bustype
	switch(thisboard->bustype)
	{
		case pcmcia_bustype:
			link = pcmcia_dev_list; /* XXX hack */
			if(!link) return -EIO;
			iobase = link->io.BasePort1;
			irq = link->irq.AssignedIRQ;
			break;
		default:
			printk("bug! couldn't determine board type\n");
			return -EINVAL;
			break;
	}
	printk("comedi%d: ni_daq_dio24: %s, io 0x%x", dev->minor, thisboard->name, iobase);
	if(irq)
	{
		printk(", irq %i", irq);
	}

	printk("\n");

	if(iobase == 0)
	{
		printk("io base address is zero!\n");
		return -EINVAL;
	}

	dev->iobase = iobase;

	/* grab our IRQ */
	if(irq < 0)
	{
		printk("irq out of range\n");
		return -EINVAL;
	}
	dev->irq = irq;

	dev->board_name = thisboard->name;

	dev->n_subdevices = 2;	// Antes era 5
	if(alloc_subdevices(dev) < 0)
		return -ENOMEM;

	/* 8255 dio */
	s = dev->subdevices + 0;
	subdev_8255_init(dev, s, NULL, dev->iobase);

	return 0;
};

static int dio24_detach(comedi_device *dev)
{
	printk("comedi%d: ni_daq_dio24: remove\n", dev->minor);

	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices + 2);

	if(thisboard->bustype != pcmcia_bustype &&
		dev->iobase)
		release_region(dev->iobase, DIO24_SIZE);
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);

	return 0;
};



// PCMCIA crap

/*
   All the PCMCIA modules use PCMCIA_DEBUG to control debugging.  If
   you do not define PCMCIA_DEBUG at all, all the debug code will be
   left out.  If you compile with PCMCIA_DEBUG=0, the debug code will
   be present but disabled -- but it can then be enabled for specific
   modules at load time with a 'pc_debug=#' option to insmod.
*/
#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
MODULE_PARM(pc_debug, "i");
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
static char *version =
"ni_daq_dio24.c, based on dummy_cs.c";
#else
#define DEBUG(n, args...)
#endif

/*====================================================================*/

/* Parameters that can be set with 'insmod' */

/* The old way: bit map of interrupts to choose from */
/* This means pick from 15, 14, 12, 11, 10, 9, 7, 5, 4, and 3 */
static u_int irq_mask = 0xdeb8;
/* Newer, simpler way of listing specific interrupts */
static int irq_list[4] = { -1 };

MODULE_PARM(irq_mask, "i");
MODULE_PARM(irq_list, "1-4i");

/*====================================================================*/

/*
   The event() function is this driver's Card Services event handler.
   It will be called by Card Services when an appropriate card status
   event is received.  The config() and release() entry points are
   used to configure or release a socket, in response to card
   insertion and ejection events.  They are invoked from the dummy
   event handler.
*/

static void dio24_config(dev_link_t *link);
static void dio24_release(u_long arg);
static int dio24_event(event_t event, int priority,
		       event_callback_args_t *args);

/*
   The attach() and detach() entry points are used to create and destroy
   "instances" of the driver, where each instance represents everything
   needed to manage one actual PCMCIA card.
*/

static dev_link_t *dio24_cs_attach(void);
static void dio24_cs_detach(dev_link_t *);

/*
   You'll also need to prototype all the functions that will actually
   be used to talk to your device.  See 'memory_cs' for a good example
   of a fully self-sufficient driver; the other drivers rely more or
   less on other parts of the kernel.
*/

/*
   The dev_info variable is the "key" that is used to match up this
   device driver with appropriate cards, through the card configuration
   database.
*/

static dev_info_t dev_info = "ni_daq_dio24";

/*
   A dev_link_t structure has fields for most things that are needed
   to keep track of a socket, but there will usually be some device
   specific information that also needs to be kept track of.  The
   'priv' pointer in a dev_link_t structure can be used to point to
   a device-specific private data structure, like this.

   To simplify the data structure handling, we actually include the
   dev_link_t structure in the device's private data structure.

   A driver needs to provide a dev_node_t structure for each device
   on a card.  In some cases, there is only one device per card (for
   example, ethernet cards, modems).  In other cases, there may be
   many actual or logical devices (SCSI adapters, memory cards with
   multiple partitions).  The dev_node_t structures need to be kept
   in a linked list starting at the 'dev' field of a dev_link_t
   structure.  We allocate them in the card's private data structure,
   because they generally shouldn't be allocated dynamically.

   In this case, we also provide a flag to indicate if a device is
   "stopped" due to a power management event, or card ejection.  The
   device IO routines can use a flag like this to throttle IO to a
   card that is not ready to accept it.

   The bus_operations pointer is used on platforms for which we need
   to use special socket-specific versions of normal IO primitives
   (inb, outb, readb, writeb, etc) for card IO.
*/

typedef struct local_info_t {
    dev_link_t		link;
    dev_node_t		node;
    int			stop;
    struct bus_operations *bus;
} local_info_t;

/*====================================================================*/

static void cs_error(client_handle_t handle, int func, int ret)
{
    error_info_t err = { func, ret };
    CardServices(ReportError, handle, &err);
}

/*======================================================================

    dio24_cs_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.

======================================================================*/

static dev_link_t *dio24_cs_attach(void)
{
    local_info_t *local;
    dev_link_t *link;
    client_reg_t client_reg;
    int ret, i;

    printk(KERN_INFO "ni_daq_dio24: HOLA SOY YO - CS-attach!\n");

    DEBUG(0, "dio24_cs_attach()\n");

    /* Allocate space for private device-specific data */
    local = kmalloc(sizeof(local_info_t), GFP_KERNEL);
    if (!local) return NULL;
    memset(local, 0, sizeof(local_info_t));
    link = &local->link; link->priv = local;

    /* Initialize the dev_link_t structure */
    link->release.function = &dio24_release;
    link->release.data = (u_long)link;

    /* Interrupt setup */
    link->irq.Attributes = IRQ_TYPE_EXCLUSIVE;
    link->irq.IRQInfo1 = IRQ_INFO2_VALID|IRQ_LEVEL_ID;
    if (irq_list[0] == -1)
	link->irq.IRQInfo2 = irq_mask;
    else
	for (i = 0; i < 4; i++)
	    link->irq.IRQInfo2 |= 1 << irq_list[i];
    link->irq.Handler = NULL;

    /*
      General socket configuration defaults can go here.  In this
      client, we assume very little, and rely on the CIS for almost
      everything.  In most clients, many details (i.e., number, sizes,
      and attributes of IO windows) are fixed by the nature of the
      device, and can be hard-wired here.
    */
    link->conf.Attributes = 0;
    link->conf.Vcc = 50;
    link->conf.IntType = INT_MEMORY_AND_IO;

    /* Register with Card Services */
    link->next = pcmcia_dev_list;
    pcmcia_dev_list = link;
    client_reg.dev_info = &dev_info;
    client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
    client_reg.EventMask =
	CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
	CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
	CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;
    client_reg.event_handler = &dio24_event;
    client_reg.Version = 0x0210;
    client_reg.event_callback_args.client_data = link;
    ret = CardServices(RegisterClient, &link->handle, &client_reg);
    if (ret != CS_SUCCESS) {
	cs_error(link->handle, RegisterClient, ret);
	dio24_cs_detach(link);
	return NULL;
    }

    return link;
} /* dio24_cs_attach */

/*======================================================================

    This deletes a driver "instance".  The device is de-registered
    with Card Services.  If it has been released, all local data
    structures are freed.  Otherwise, the structures will be freed
    when the device is released.

======================================================================*/

static void dio24_cs_detach(dev_link_t *link)
{
    dev_link_t **linkp;

    printk(KERN_INFO "ni_daq_dio24: HOLA SOY YO - cs-detach!\n");

    DEBUG(0, "dio24_cs_detach(0x%p)\n", link);

    /* Locate device structure */
    for (linkp = &pcmcia_dev_list; *linkp; linkp = &(*linkp)->next)
	if (*linkp == link) break;
    if (*linkp == NULL)
	return;

    /*
       If the device is currently configured and active, we won't
       actually delete it yet.  Instead, it is marked so that when
       the release() function is called, that will trigger a proper
       detach().
    */
    if (link->state & DEV_CONFIG) {
#ifdef PCMCIA_DEBUG
	printk(KERN_DEBUG "ni_daq_dio24: detach postponed, '%s' "
	       "still locked\n", link->dev->dev_name);
#endif
	link->state |= DEV_STALE_LINK;
	return;
    }

    /* Break the link with Card Services */
    if (link->handle)
	CardServices(DeregisterClient, link->handle);

    /* Unlink device structure, and free it */
    *linkp = link->next;
    /* This points to the parent local_info_t struct */
    kfree(link->priv);

} /* dio24_cs_detach */

/*======================================================================

    dio24_config() is scheduled to run after a CARD_INSERTION event
    is received, to configure the PCMCIA socket, and to make the
    device available to the system.

======================================================================*/

#define CS_CHECK(fn, args...) \
while ((last_ret=CardServices(last_fn=(fn),args))!=0) goto cs_failed

#define CFG_CHECK(fn, args...) \
if (CardServices(fn, args) != 0) goto next_entry

static void dio24_config(dev_link_t *link)
{
    client_handle_t handle = link->handle;
    local_info_t *dev = link->priv;
    tuple_t tuple;
    cisparse_t parse;
    int last_fn, last_ret;
    u_char buf[64];
    config_info_t conf;
    win_req_t req;
    memreq_t map;
    cistpl_cftable_entry_t dflt = { 0 };

    printk(KERN_INFO "ni_daq_dio24: HOLA SOY YO! - config\n");

    DEBUG(0, "dio24_config(0x%p)\n", link);

    /*
       This reads the card's CONFIG tuple to find its configuration
       registers.
    */
    tuple.DesiredTuple = CISTPL_CONFIG;
    tuple.Attributes = 0;
    tuple.TupleData = buf;
    tuple.TupleDataMax = sizeof(buf);
    tuple.TupleOffset = 0;
    CS_CHECK(GetFirstTuple, handle, &tuple);
    CS_CHECK(GetTupleData, handle, &tuple);
    CS_CHECK(ParseTuple, handle, &tuple, &parse);
    link->conf.ConfigBase = parse.config.base;
    link->conf.Present = parse.config.rmask[0];

    /* Configure card */
    link->state |= DEV_CONFIG;

    /* Look up the current Vcc */
    CS_CHECK(GetConfigurationInfo, handle, &conf);
    link->conf.Vcc = conf.Vcc;

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
    CS_CHECK(GetFirstTuple, handle, &tuple);
    while (1) {
	cistpl_cftable_entry_t *cfg = &(parse.cftable_entry);
	CFG_CHECK(GetTupleData, handle, &tuple);
	CFG_CHECK(ParseTuple, handle, &tuple, &parse);

	if (cfg->flags & CISTPL_CFTABLE_DEFAULT) dflt = *cfg;
	if (cfg->index == 0) goto next_entry;
	link->conf.ConfigIndex = cfg->index;

	/* Does this card need audio output? */
	if (cfg->flags & CISTPL_CFTABLE_AUDIO) {
	    link->conf.Attributes |= CONF_ENABLE_SPKR;
	    link->conf.Status = CCSR_AUDIO_ENA;
	}

	/* Use power settings for Vcc and Vpp if present */
	/*  Note that the CIS values need to be rescaled */
	if (cfg->vcc.present & (1<<CISTPL_POWER_VNOM)) {
	    if (conf.Vcc != cfg->vcc.param[CISTPL_POWER_VNOM]/10000)
		goto next_entry;
	} else if (dflt.vcc.present & (1<<CISTPL_POWER_VNOM)) {
	    if (conf.Vcc != dflt.vcc.param[CISTPL_POWER_VNOM]/10000)
		goto next_entry;
	}

	if (cfg->vpp1.present & (1<<CISTPL_POWER_VNOM))
	    link->conf.Vpp1 = link->conf.Vpp2 =
		cfg->vpp1.param[CISTPL_POWER_VNOM]/10000;
	else if (dflt.vpp1.present & (1<<CISTPL_POWER_VNOM))
	    link->conf.Vpp1 = link->conf.Vpp2 =
		dflt.vpp1.param[CISTPL_POWER_VNOM]/10000;

	/* Do we need to allocate an interrupt? */
	if (cfg->irq.IRQInfo1 || dflt.irq.IRQInfo1)
	    link->conf.Attributes |= CONF_ENABLE_IRQ;

	/* IO window settings */
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
	    CFG_CHECK(RequestIO, link->handle, &link->io);
	}

	/*
	  Now set up a common memory window, if needed.  There is room
	  in the dev_link_t structure for one memory window handle,
	  but if the base addresses need to be saved, or if multiple
	  windows are needed, the info should go in the private data
	  structure for this device.

	  Note that the memory window base is a physical address, and
	  needs to be mapped to virtual space with ioremap() before it
	  is used.
	*/
	if ((cfg->mem.nwin > 0) || (dflt.mem.nwin > 0)) {
	    cistpl_mem_t *mem =
		(cfg->mem.nwin) ? &cfg->mem : &dflt.mem;
	    req.Attributes = WIN_DATA_WIDTH_16|WIN_MEMORY_TYPE_CM;
	    req.Attributes |= WIN_ENABLE;
	    req.Base = mem->win[0].host_addr;
	    req.Size = mem->win[0].len;
	    if (req.Size < 0x1000)
		req.Size = 0x1000;
	    req.AccessSpeed = 0;
	    link->win = (window_handle_t)link->handle;
	    CFG_CHECK(RequestWindow, &link->win, &req);
	    map.Page = 0; map.CardOffset = mem->win[0].card_addr;
	    CFG_CHECK(MapMemPage, link->win, &map);
	}
	/* If we got this far, we're cool! */
	break;

    next_entry:
	if (link->io.NumPorts1)
	    CardServices(ReleaseIO, link->handle, &link->io);
	CS_CHECK(GetNextTuple, handle, &tuple);
    }

    /*
       Allocate an interrupt line.  Note that this does not assign a
       handler to the interrupt, unless the 'Handler' member of the
       irq structure is initialized.
    */
    if (link->conf.Attributes & CONF_ENABLE_IRQ)
	CS_CHECK(RequestIRQ, link->handle, &link->irq);

    /*
       This actually configures the PCMCIA socket -- setting up
       the I/O windows and the interrupt mapping, and putting the
       card and host interface into "Memory and IO" mode.
    */
    CS_CHECK(RequestConfiguration, link->handle, &link->conf);

    /*
      At this point, the dev_node_t structure(s) need to be
      initialized and arranged in a linked list at link->dev.
    */
    sprintf(dev->node.dev_name, "ni_daq_dio24");
    dev->node.major = dev->node.minor = 0;
    link->dev = &dev->node;

    /* Finally, report what we've done */
    printk(KERN_INFO "%s: index 0x%02x: Vcc %d.%d",
	   dev->node.dev_name, link->conf.ConfigIndex,
	   link->conf.Vcc/10, link->conf.Vcc%10);
    if (link->conf.Vpp1)
	printk(", Vpp %d.%d", link->conf.Vpp1/10, link->conf.Vpp1%10);
    if (link->conf.Attributes & CONF_ENABLE_IRQ)
	printk(", irq %d", link->irq.AssignedIRQ);
    if (link->io.NumPorts1)
	printk(", io 0x%04x-0x%04x", link->io.BasePort1,
	       link->io.BasePort1+link->io.NumPorts1-1);
    if (link->io.NumPorts2)
	printk(" & 0x%04x-0x%04x", link->io.BasePort2,
	       link->io.BasePort2+link->io.NumPorts2-1);
    if (link->win)
	printk(", mem 0x%06lx-0x%06lx", req.Base,
	       req.Base+req.Size-1);
    printk("\n");

    link->state &= ~DEV_CONFIG_PENDING;
    return;

cs_failed:
    cs_error(link->handle, last_fn, last_ret);
    printk(KERN_INFO "Fallo");
    dio24_release((u_long)link);

} /* dio24_config */

/*======================================================================

    After a card is removed, dio24_release() will unregister the
    device, and release the PCMCIA configuration.  If the device is
    still open, this will be postponed until it is closed.

======================================================================*/

static void dio24_release(u_long arg)
{
    dev_link_t *link = (dev_link_t *)arg;

    DEBUG(0, "dio24_release(0x%p)\n", link);

    /*
       If the device is currently in use, we won't release until it
       is actually closed, because until then, we can't be sure that
       no one will try to access the device or its data structures.
    */
    if (link->open) {
	DEBUG(1, "ni_dio24: release postponed, '%s' still open\n",
	      link->dev->dev_name);
	link->state |= DEV_STALE_CONFIG;
	return;
    }

    /* Unlink the device chain */
    link->dev = NULL;

    /*
      In a normal driver, additional code may be needed to release
      other kernel data structures associated with this device.
    */

    /* Don't bother checking to see if these succeed or not */
    if (link->win)
	CardServices(ReleaseWindow, link->win);
    CardServices(ReleaseConfiguration, link->handle);
    if (link->io.NumPorts1)
	CardServices(ReleaseIO, link->handle, &link->io);
    if (link->irq.AssignedIRQ)
	CardServices(ReleaseIRQ, link->handle, &link->irq);
    link->state &= ~DEV_CONFIG;

    if (link->state & DEV_STALE_LINK)
	dio24_cs_detach(link);

} /* dio24_release */

/*======================================================================

    The card status event handler.  Mostly, this schedules other
    stuff to run after an event is received.

    When a CARD_REMOVAL event is received, we immediately set a
    private flag to block future accesses to this device.  All the
    functions that actually access the device should check this flag
    to make sure the card is still present.

======================================================================*/

static int dio24_event(event_t event, int priority,
		       event_callback_args_t *args)
{
    dev_link_t *link = args->client_data;
    local_info_t *dev = link->priv;

    DEBUG(1, "dio24_event(0x%06x)\n", event);

    switch (event) {
    case CS_EVENT_CARD_REMOVAL:
	link->state &= ~DEV_PRESENT;
	if (link->state & DEV_CONFIG) {
	    ((local_info_t *)link->priv)->stop = 1;
	    mod_timer(&link->release, jiffies + HZ/20);
	}
	break;
    case CS_EVENT_CARD_INSERTION:
	link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
	dev->bus = args->bus;
	dio24_config(link);
	break;
    case CS_EVENT_PM_SUSPEND:
	link->state |= DEV_SUSPEND;
	/* Fall through... */
    case CS_EVENT_RESET_PHYSICAL:
	/* Mark the device as stopped, to block IO until later */
	dev->stop = 1;
	if (link->state & DEV_CONFIG)
	    CardServices(ReleaseConfiguration, link->handle);
	break;
    case CS_EVENT_PM_RESUME:
	link->state &= ~DEV_SUSPEND;
	/* Fall through... */
    case CS_EVENT_CARD_RESET:
	if (link->state & DEV_CONFIG)
	    CardServices(RequestConfiguration, link->handle, &link->conf);
	dev->stop = 0;
	/*
	  In a normal driver, additional code may go here to restore
	  the device state and restart IO.
	*/
	break;
    }
    return 0;
} /* dio24_event */

/*====================================================================*/

static int __init init_dio24_cs(void)
{
    servinfo_t serv;
    printk("ni_daq_dio24: HOLA SOY YO!\n");
    DEBUG(0, "%s\n", version);
    CardServices(GetCardServicesInfo, &serv);
    if (serv.Revision != CS_RELEASE_CODE) {
	printk(KERN_NOTICE "ni_daq_dio24: Card Services release "
	       "does not match! Vaya putada\n");
	return -1;
    }
    register_pccard_driver(&dev_info, &dio24_cs_attach, &dio24_cs_detach);
    return 0;
}

static void __exit exit_dio24_cs(void)
{
    DEBUG(0, "ni_dio24: unloading\n");
    unregister_pccard_driver(&dev_info);
    while (pcmcia_dev_list != NULL) {
	del_timer(&pcmcia_dev_list->release);
	if (pcmcia_dev_list->state & DEV_CONFIG)
	    dio24_release((u_long)pcmcia_dev_list);
	dio24_cs_detach(pcmcia_dev_list);
    }
}

int init_module(void)
{
	int ret;

    	ret = init_dio24_cs();
	if(ret < 0)
		return ret;

	return comedi_driver_register(&driver_dio24);
}

void cleanup_module(void)
{
	exit_dio24_cs();
	comedi_driver_unregister(&driver_dio24);
}

