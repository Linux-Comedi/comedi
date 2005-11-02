/*
    comedi/drivers/das08_cs.c
    DAS08 driver

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@schleef.org>
    Copyright (C) 2001,2002,2003 Frank Mori Hess <fmhess@users.sourceforge.net>

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

*****************************************************************

*/
/*
Driver: das08_cs.o
Description: DAS-08 PCMCIA boards
Author: Warren Jasper, ds, Frank Hess
Devices: [ComputerBoards] PCM-DAS08 (pcm-das08)
Status: works

This is the PCMCIA-specific support split off from the
das08 driver.

Options (for pcm-das08):
        NONE

Command support does not exist, but could be added for this board.
*/

#include <linux/comedidev.h>

#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/version.h>

#include "das08.h"

// pcmcia includes
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/ds.h>
/*
   A linked list of "instances" of the das08_pcmcia device.  Each actual
   PCMCIA card corresponds to one device instance, and is described
   by one dev_link_t structure (defined in ds.h).

   You may not want to use a linked list for this -- for example, the
   memory card driver uses an array of dev_link_t pointers, where minor
   device numbers are used to derive the corresponding array index.
*/
static dev_link_t *dev_list = NULL;

#define thisboard ((struct das08_board_struct *)dev->board_ptr)

static int das08_cs_attach(comedi_device *dev,comedi_devconfig *it);

static comedi_driver driver_das08_cs =
{
	driver_name: "das08_cs",
	module: THIS_MODULE,
	attach: das08_cs_attach,
	detach: das08_common_detach,
	board_name: das08_cs_boards,
	num_names: sizeof(das08_cs_boards) / sizeof(struct das08_board_struct),
	offset: sizeof(struct das08_board_struct),
};

static int das08_cs_attach(comedi_device *dev,comedi_devconfig *it)
{
	int ret;
	unsigned long iobase;
	dev_link_t *link = dev_list;	// XXX hack

	if((ret=alloc_private(dev,sizeof(struct das08_private_struct)))<0)
		return ret;

	printk("comedi%d: das08_cs: ", dev->minor);
	// deal with a pci board

	if(thisboard->bustype == pcmcia)
	{
		if(link == NULL)
		{
			printk(" no pcmcia cards found\n");
			return -EIO;
		}
		iobase = link->io.BasePort1;
	}else{
		printk(" bug! board does not have PCMCIA bustype\n");
		return -EINVAL;
	}

	printk("\n");

	return das08_common_attach( dev, iobase );
}

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

#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
MODULE_PARM(pc_debug, "i");
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
static char *version =
"das08.c pcmcia code (Frank Hess), modified from dummy_cs.c 1.31 2001/08/24 12:13:13 (David Hinds)";
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
   insertion and ejection events.  They are invoked from the das08_pcmcia
   event handler.
*/

static void das08_pcmcia_config(dev_link_t *link);
static void das08_pcmcia_release(u_long arg);
static int das08_pcmcia_event(event_t event, int priority,
		       event_callback_args_t *args);

/*
   The attach() and detach() entry points are used to create and destroy
   "instances" of the driver, where each instance represents everything
   needed to manage one actual PCMCIA card.
*/

static dev_link_t *das08_pcmcia_attach(void);
static void das08_pcmcia_detach(dev_link_t *);

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

static dev_info_t dev_info = "pcm-das08";

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

/*======================================================================

    das08_pcmcia_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.

======================================================================*/

static dev_link_t *das08_pcmcia_attach(void)
{
    local_info_t *local;
    dev_link_t *link;
    client_reg_t client_reg;
    int ret, i;

    DEBUG(0, "das08_pcmcia_attach()\n");

    /* Allocate space for private device-specific data */
    local = kmalloc(sizeof(local_info_t), GFP_KERNEL);
    if (!local) return NULL;
    memset(local, 0, sizeof(local_info_t));
    link = &local->link; link->priv = local;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
    /* Initialize the dev_link_t structure */
    link->release.function = &das08_pcmcia_release;
    link->release.data = (u_long)link;
#endif
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
    link->next = dev_list;
    dev_list = link;
    client_reg.dev_info = &dev_info;
    client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)	
	client_reg.EventMask =
		CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
		CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
		CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;
	client_reg.event_handler = &das08_pcmcia_event;
#endif
    client_reg.Version = 0x0210;
    client_reg.event_callback_args.client_data = link;
    ret = pcmcia_register_client(&link->handle, &client_reg);
    if (ret != CS_SUCCESS) {
	cs_error(link->handle, RegisterClient, ret);
	das08_pcmcia_detach(link);
	return NULL;
    }

    return link;
} /* das08_pcmcia_attach */

/*======================================================================

    This deletes a driver "instance".  The device is de-registered
    with Card Services.  If it has been released, all local data
    structures are freed.  Otherwise, the structures will be freed
    when the device is released.

======================================================================*/

static void das08_pcmcia_detach(dev_link_t *link)
{
	dev_link_t **linkp;

	DEBUG(0, "das08_pcmcia_detach(0x%p)\n", link);

	/* Locate device structure */
	for (linkp = &dev_list; *linkp; linkp = &(*linkp)->next)
	if (*linkp == link) break;
	if (*linkp == NULL)
	return;

	/*
	If the device is currently configured and active, we won't
	actually delete it yet.  Instead, it is marked so that when
	the release() function is called, that will trigger a proper
	detach().
	*/
	if (link->state & DEV_CONFIG)
	{
#ifdef PCMCIA_DEBUG
		printk(KERN_DEBUG "das08: detach postponed, '%s' "
			"still locked\n", link->dev->dev_name);
#endif
		link->state |= DEV_STALE_LINK;
		return;
	}

	/* Break the link with Card Services */
	if (link->handle)
		pcmcia_deregister_client(link->handle);

	/* Unlink device structure, and free it */
	*linkp = link->next;
	/* This points to the parent local_info_t struct */
	kfree(link->priv);

} /* das08_pcmcia_detach */

/*======================================================================

    das08_pcmcia_config() is scheduled to run after a CARD_INSERTION event
    is received, to configure the PCMCIA socket, and to make the
    device available to the system.

======================================================================*/

static void das08_pcmcia_config(dev_link_t *link)
{
	client_handle_t handle = link->handle;
	local_info_t *dev = link->priv;
	tuple_t tuple;
	cisparse_t parse;
	int last_fn, last_ret;
	u_char buf[64];
	config_info_t conf;
	win_req_t req;
	cistpl_cftable_entry_t dflt = { 0 };

	DEBUG(0, "das08_pcmcia_config(0x%p)\n", link);

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
	if((last_ret = pcmcia_get_first_tuple(handle, &tuple)) != 0) goto cs_failed;
	last_fn = GetTupleData;
	if((last_ret = pcmcia_get_tuple_data(handle, &tuple)) != 0) goto cs_failed;
	last_fn = ParseTuple;
	if((last_ret = pcmcia_parse_tuple(handle, &tuple, &parse)) != 0) goto cs_failed;
	link->conf.ConfigBase = parse.config.base;
	link->conf.Present = parse.config.rmask[0];

	/* Configure card */
	link->state |= DEV_CONFIG;

	/* Look up the current Vcc */
	last_fn = GetConfigurationInfo;
	if((last_ret = pcmcia_get_configuration_info(handle, &conf)) != 0) goto cs_failed;
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
	last_fn = GetFirstTuple;
	if((last_ret = pcmcia_get_first_tuple(handle, &tuple)) != 0) goto cs_failed;
	while (1) {
	cistpl_cftable_entry_t *cfg = &(parse.cftable_entry);
	if((last_ret = pcmcia_get_tuple_data(handle, &tuple)) != 0) goto next_entry;
	if((last_ret = pcmcia_parse_tuple(handle, &tuple, &parse)) != 0) goto next_entry;

	if (cfg->flags & CISTPL_CFTABLE_DEFAULT) dflt = *cfg;
	if (cfg->index == 0) goto next_entry;
	link->conf.ConfigIndex = cfg->index;

	/* Does this card need audio output? */
/*	if (cfg->flags & CISTPL_CFTABLE_AUDIO) {
		link->conf.Attributes |= CONF_ENABLE_SPKR;
		link->conf.Status = CCSR_AUDIO_ENA;
	}
*/
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
		if(pcmcia_request_io(link->handle, &link->io) != 0) goto next_entry;
	}

	/* If we got this far, we're cool! */
	break;

	next_entry:
	if (link->io.NumPorts1)
		pcmcia_release_io(link->handle, &link->io);
	last_fn = GetNextTuple;
	if((last_ret = pcmcia_get_next_tuple(handle, &tuple)) != 0) goto cs_failed;
	}

	/*
		Allocate an interrupt line.  Note that this does not assign a
		handler to the interrupt, unless the 'Handler' member of the
		irq structure is initialized.
	*/
	if (link->conf.Attributes & CONF_ENABLE_IRQ)
	{
		last_fn = RequestIRQ;
		if((last_ret = pcmcia_request_irq(handle, &link->irq)) != 0) goto cs_failed;
	}

	/*
		This actually configures the PCMCIA socket -- setting up
		the I/O windows and the interrupt mapping, and putting the
		card and host interface into "Memory and IO" mode.
	*/
	last_fn = RequestConfiguration;
	if((last_ret = pcmcia_request_configuration(link->handle, &link->conf)) != 0) goto cs_failed;

	/*
		At this point, the dev_node_t structure(s) need to be
		initialized and arranged in a linked list at link->dev.
	*/
	sprintf(dev->node.dev_name, "pcm-das08");
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
	das08_pcmcia_release((u_long)link);

} /* das08_pcmcia_config */

/*======================================================================

    After a card is removed, das08_pcmcia_release() will unregister the
    device, and release the PCMCIA configuration.  If the device is
    still open, this will be postponed until it is closed.

======================================================================*/

static void das08_pcmcia_release(u_long arg)
{
	dev_link_t *link = (dev_link_t *)arg;

	DEBUG(0, "das08_pcmcia_release(0x%p)\n", link);

    /*
       If the device is currently in use, we won't release until it
       is actually closed, because until then, we can't be sure that
       no one will try to access the device or its data structures.
    */
	if (link->open)
	{
		DEBUG(1, "das08: release postponed, '%s' still open\n",
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
		pcmcia_release_window(link->win);
	pcmcia_release_configuration(link->handle);
	if (link->io.NumPorts1)
		pcmcia_release_io(link->handle, &link->io);
	if (link->irq.AssignedIRQ)
		pcmcia_release_irq(link->handle, &link->irq);
	link->state &= ~DEV_CONFIG;

	if (link->state & DEV_STALE_LINK)
		das08_pcmcia_detach(link);

} /* das08_pcmcia_release */

/*======================================================================

    The card status event handler.  Mostly, this schedules other
    stuff to run after an event is received.

    When a CARD_REMOVAL event is received, we immediately set a
    private flag to block future accesses to this device.  All the
    functions that actually access the device should check this flag
    to make sure the card is still present.

======================================================================*/

static int das08_pcmcia_event(event_t event, int priority,
	event_callback_args_t *args)
{
	dev_link_t *link = args->client_data;
	local_info_t *dev = link->priv;

	DEBUG(1, "das08_pcmcia_event(0x%06x)\n", event);

	switch (event)
	{
		case CS_EVENT_CARD_REMOVAL:
			link->state &= ~DEV_PRESENT;
			if (link->state & DEV_CONFIG)
			{
				((local_info_t *)link->priv)->stop = 1;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
				mod_timer(&link->release, jiffies + HZ/20);
#else
				das08_pcmcia_release((ulong)link);
#endif
			}
			break;
		case CS_EVENT_CARD_INSERTION:
			link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
			dev->bus = args->bus;
#endif			
			das08_pcmcia_config(link);
			break;
		case CS_EVENT_PM_SUSPEND:
			link->state |= DEV_SUSPEND;
			/* Fall through... */
		case CS_EVENT_RESET_PHYSICAL:
			/* Mark the device as stopped, to block IO until later */
			dev->stop = 1;
			if (link->state & DEV_CONFIG)
				pcmcia_release_configuration(link->handle);
			break;
		case CS_EVENT_PM_RESUME:
			link->state &= ~DEV_SUSPEND;
			/* Fall through... */
		case CS_EVENT_CARD_RESET:
			if (link->state & DEV_CONFIG)
				pcmcia_request_configuration(link->handle, &link->conf);
			dev->stop = 0;
/*
In a normal driver, additional code may go here to restore
the device state and restart IO.
*/
			break;
	}

	return 0;
} /* das08_pcmcia_event */

/*====================================================================*/

struct pcmcia_driver das08_cs_driver =
{
	.attach = &das08_pcmcia_attach,
	.detach = &das08_pcmcia_detach,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
	.event = &das08_pcmcia_event,
#endif
	.owner = THIS_MODULE,
	.drv = {
		.name = dev_info,
	},	
};

static int __init init_das08_pcmcia_cs(void)
{
	DEBUG(0, "%s\n", version);
	pcmcia_register_driver(&das08_cs_driver);
	return 0;
}

static void __exit exit_das08_pcmcia_cs(void)
{
	DEBUG(0, "das08_pcmcia_cs: unloading\n");
	pcmcia_unregister_driver(&das08_cs_driver);
	while (dev_list != NULL)
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 5, 0)
		del_timer(&dev_list->release);
#endif
		if (dev_list->state & DEV_CONFIG)
			das08_pcmcia_release((u_long)dev_list);
		das08_pcmcia_detach(dev_list);
	}
}

static int das08_cs_init_module(void)
{
	int ret;

	ret = init_das08_pcmcia_cs();
	if(ret < 0)
		return ret;

	return comedi_driver_register(&driver_das08_cs);
}

static void das08_cs_exit_module(void)
{
	exit_das08_pcmcia_cs();
	comedi_driver_unregister(&driver_das08_cs);
}

MODULE_LICENSE("GPL");
module_init( das08_cs_init_module );
module_exit( das08_cs_exit_module );

