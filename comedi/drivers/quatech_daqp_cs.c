/*======================================================================

    Quatech DAQP PCMCIA data capture cards COMEDI client driver
    Copyright (C) 2000 Brent Baccala <baccala@freesoft.org>
    The DAQP interface code in this file is released into the public domain.

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1998 David A. Schleef <ds@stm.lbl.gov>

    daqp_cs.c 1.00



    Documentation for the DAQP PCMCIA cards can be found on Quatech's site:

                ftp://ftp.quatech.com/Manuals/daqp-208.pdf

    This manual is for both the DAQP-208 and the DAQP-308.

    This code presently doesn't do D/A conversion; only A/D.
    Also, I've had problems getting interrupts to work reliably,
    the driver currently polls the card.

    Multiple DAPQ cards are handled and can be independently attached
    by specifying a numeric argument to comedi_config.  Cards are
    numbered sequentially from 0 in the order they are inserted/detected.

======================================================================*/


#include <pcmcia/config.h>
#include <pcmcia/k_compat.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/ptrace.h>
#include <linux/malloc.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <asm/system.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/cisreg.h>
#include <pcmcia/ds.h>

#include <linux/comedidev.h>

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
"daqp_cs.c 1.00 2000/10/02 (Brent Baccala)";
#else
#define DEBUG(n, args...)
#endif

/* Maximum number of separate DAQP devices we'll allow */
#define MAX_DEV         4

/* I can't get my DAQP-308 to reliably generate interrupts, so I poll it */
/* #define USE_INTERRUPTS */

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

   We also provide an index into the dev_table
*/
   
typedef struct local_info_t {
    dev_link_t		link;
    dev_node_t		node;
    int			stop;
    int			table_index;
    wait_queue_head_t	endofscan;
} local_info_t;

/* A list of "instances" of the device. */

static local_info_t *dev_table[MAX_DEV] = { NULL, /* ... */ };


/* this is COMEDI's private data structure unique to this hardware driver.
   Not to be confused with PCMCIA's local_info_t (above).
*/

typedef struct {

	int devnum;

} daqp_private;

#define devpriv ((daqp_private *)dev->private)

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

/* The DAQP communicates with the system through a 16 byte I/O window. */

#define DAQP_FIFO_SIZE		4096

#define DAQP_FIFO		0
#define DAQP_SCANLIST		1
#define DAQP_CONTROL		2
#define DAQP_STATUS		2
#define DAQP_DIGITAL_IO		3
#define DAQP_PACER_LOW		4
#define DAQP_PACER_MID		5
#define DAQP_PACER_HIGH		6
#define DAQP_COMMAND		7
#define DAQP_DA			8
#define DAQP_TIMER		10
#define DAQP_AUX		15

#define DAQP_SCANLIST_DIFFERENTIAL	0x4000
#define DAQP_SCANLIST_GAIN(x)		((x)<<12)
#define DAQP_SCANLIST_CHANNEL(x)	((x)<<8)
#define DAQP_SCANLIST_START		0x0080
#define DAQP_SCANLIST_EXT_GAIN(x)	((x)<<4)
#define DAQP_SCANLIST_EXT_CHANNEL(x)	(x)

#define DAQP_CONTROL_PACER_100kHz	0xc0
#define DAQP_CONTROL_PACER_1MHz		0x80
#define DAQP_CONTROL_PACER_5MHz		0x40
#define DAQP_CONTROL_PACER_EXTERNAL	0x00
#define DAQP_CONTORL_EXPANSION		0x20
#define DAQP_CONTROL_EOS_INT_ENABLE	0x10
#define DAQP_CONTROL_FIFO_INT_ENABLE	0x08
#define DAQP_CONTROL_TRIGGER_ONESHOT	0x00
#define DAQP_CONTROL_TRIGGER_CONTINUOUS	0x04
#define DAQP_CONTROL_TRIGGER_INTERNAL	0x00
#define DAQP_CONTROL_TRIGGER_EXTERNAL	0x02
#define DAQP_CONTROL_TRIGGER_RISING	0x00
#define DAQP_CONTROL_TRIGGER_FALLING	0x01

#define DAQP_STATUS_IDLE		0x80
#define DAQP_STATUS_RUNNING		0x40
#define DAQP_STATUS_DATA_LOST		0x20
#define DAQP_STATUS_END_OF_SCAN		0x10
#define DAQP_STATUS_FIFO_THRESHOLD	0x08
#define DAQP_STATUS_FIFO_FULL		0x04
#define DAQP_STATUS_FIFO_NEARFULL	0x02
#define DAQP_STATUS_FIFO_EMPTY		0x01

#define DAQP_COMMAND_ARM		0x80
#define DAQP_COMMAND_RSTF		0x40
#define DAQP_COMMAND_RSTQ		0x20
#define DAQP_COMMAND_STOP		0x10
#define DAQP_COMMAND_LATCH		0x08
#define DAQP_COMMAND_100kHz		0x00
#define DAQP_COMMAND_50kHz		0x02
#define DAQP_COMMAND_25kHz		0x04
#define DAQP_COMMAND_FIFO_DATA		0x01
#define DAQP_COMMAND_FIFO_PROGRAM	0x00

#define DAQP_AUX_TRIGGER_TTL		0x00
#define DAQP_AUX_TRIGGER_ANALOG		0x80
#define DAQP_AUX_TRIGGER_PRETRIGGER	0x40
#define DAQP_AUX_TIMER_INT_ENABLE	0x20
#define DAQP_AUX_TIMER_RELOAD		0x00
#define DAQP_AUX_TIMER_PAUSE		0x08
#define DAQP_AUX_TIMER_GO		0x10
#define DAQP_AUX_TIMER_GO_EXTERNAL	0x18
#define DAQP_AUX_TIMER_EXTERNAL_SRC	0x04
#define DAQP_AUX_TIMER_INTERNAL_SRC	0x00
#define DAQP_AUX_DA_DIRECT		0x00
#define DAQP_AUX_DA_OVERFLOW		0x01
#define DAQP_AUX_DA_EXTERNAL		0x02
#define DAQP_AUX_DA_PACER		0x03

#define DAQP_AUX_RUNNING		0x80
#define DAQP_AUX_TRIGGERED		0x40
#define DAQP_AUX_DA_BUFFER		0x20
#define DAQP_AUX_TIMER_OVERFLOW		0x10
#define DAQP_AUX_CONVERSION		0x08
#define DAQP_AUX_DATA_LOST		0x04
#define DAQP_AUX_FIFO_NEARFULL		0x02
#define DAQP_AUX_FIFO_EMPTY		0x01

static comedi_lrange range_daqp_ai = { 4, {
	BIP_RANGE( 10 ),
	BIP_RANGE( 5 ),
	BIP_RANGE( 2.5 ),
	BIP_RANGE( 1.25 )
}};

/*====================================================================*/

/* comedi interface code */

static int daqp_attach(comedi_device *dev,comedi_devconfig *it);
static int daqp_detach(comedi_device *dev);
comedi_driver driver_daqp={
	driver_name:	"daqp",
	module:		&__this_module,
	attach:		daqp_attach,
	detach:		daqp_detach,
};


static void daqp_dump(comedi_device *dev)
{
	printk("DAQP: status %02x; aux status %02x\n",
	       inb(dev->iobase + DAQP_STATUS), inb(dev->iobase + DAQP_AUX));
}

static int daqp_ai_a(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	local_info_t *local;
	int i;
	int v;
	int data;
	int counter=10000;
#ifdef USE_INTERRUPTS
	int threshold;
	int flags;
	int timeout;
#endif

	if (!dev_table[devpriv->devnum] || dev_table[devpriv->devnum]->stop) {
		return -EIO;
	} else {
		local = dev_table[devpriv->devnum];
	}

	/* Stop any running conversion */
	outb(DAQP_COMMAND_STOP,
	     dev->iobase+DAQP_COMMAND);

	outb(0, dev->iobase+DAQP_AUX);

	/* Reset scan list queue */
	outb(DAQP_COMMAND_RSTQ,
	     dev->iobase+DAQP_COMMAND);

	for (i=0; i < it->n_chan; i++) {

		/* Program one scan list entry */

		v = DAQP_SCANLIST_CHANNEL(CR_CHAN(it->chanlist[i]))
			| DAQP_SCANLIST_GAIN(CR_RANGE(it->chanlist[i]));

		if (CR_AREF(it->chanlist[i]) == AREF_DIFF) {
			v |= DAQP_SCANLIST_DIFFERENTIAL;
		}

		if (i==0) {
			v |= DAQP_SCANLIST_START;
		}

		outb(v & 0xff, dev->iobase + DAQP_SCANLIST);
		outb(v >> 8, dev->iobase + DAQP_SCANLIST);
	}

	/* Reset data FIFO (see page 28 of DAQP User's Manual) */
	outb(DAQP_COMMAND_RSTF,
	     dev->iobase + DAQP_COMMAND);

#ifdef USE_INTERRUPTS
	/* Set FIFO threshold */
	// threshold = DAQP_FIFO_SIZE - 2*it->n_chan;
	threshold = DAQP_FIFO_SIZE - 1;
	outb(0, dev->iobase + DAQP_FIFO);
	outb(0, dev->iobase + DAQP_FIFO);
	outb(threshold & 0xff, dev->iobase + DAQP_FIFO);
	outb(threshold >> 8, dev->iobase + DAQP_FIFO);
#endif

	/* Set trigger */
	v = DAQP_CONTROL_TRIGGER_ONESHOT | DAQP_CONTROL_TRIGGER_INTERNAL
	  | DAQP_CONTROL_PACER_100kHz;
#ifdef USE_INTERRUPTS
	v |= DAQP_CONTROL_FIFO_INT_ENABLE;
#endif
	outb(v, dev->iobase + DAQP_CONTROL);

#ifdef USE_INTERRUPTS
	save_flags(flags);
	cli();
#endif

	/* Start conversion */
	outb(DAQP_COMMAND_ARM | DAQP_COMMAND_FIFO_DATA,
	     dev->iobase + DAQP_COMMAND);

#ifdef USE_INTERRUPTS
	timeout = sleep_on_timeout(&local->endofscan, 1*HZ);

	restore_flags(flags);

	printk("timeout: %d\n", timeout);
#endif

	for (i=0; i < it->n_chan; i++) {

		/* Wait for data in FIFO */
		while (--counter
		       && (inb(dev->iobase + DAQP_STATUS)
			   & DAQP_STATUS_FIFO_EMPTY));

		if (!counter) {
			printk("DAQP FIFO never got data!\n");
			daqp_dump(dev);
			break;
		} else {
			data = inb(dev->iobase + DAQP_FIFO);
			data |= inb(dev->iobase + DAQP_FIFO) << 8;
			data ^= 0x8000;
			it->data[i] = data;
		}
	}

	return i;
}

static void daqp_interrupt(int irq, void * dev_id, struct pt_regs *regs)
{
	local_info_t *local = (local_info_t *)dev_id;

	if (local == NULL) {
		printk(KERN_WARNING "daqp_interrupt(): irq %d for unknown device.\n",
		       irq);
		return;
	}

	wake_up(&local->endofscan);

	printk("daqp_interrupt()\n");
}

/* daqp_attach is called via comedi_config to attach a comedi device
 * to a /dev/comedi*.  Note that this is different from daqp_cs_attach()
 * which is called by the pcmcia subsystem to attach the PCMCIA card
 * when it is inserted.
 */

static int daqp_attach(comedi_device *dev, comedi_devconfig *it)
{
	int ret;
	comedi_subdevice *s;

	if (it->options[0] < 0 || it->options[0] >= MAX_DEV
	    || ! dev_table[it->options[0]]) {
	  printk("comedi%d: No such daqp device %d\n",
		 dev->minor, it->options[0]);
	  return -EIO;
	}

	/* Probably should pull this out of PCMCIA CIS tuples */
	dev->board_name = "Quatech DAQP";

	dev->iobase=dev_table[it->options[0]]->link.io.BasePort1;

	if((ret=alloc_private(dev,sizeof(daqp_private))) < 0)
		return ret;
	devpriv->devnum = it->options[0];

	dev->n_subdevices=1;
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	printk("comedi%d: attaching daqp%d (io 0x%04x)\n",
	       dev->minor, it->options[0], dev->iobase);

	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=8;
	s->maxdata=0xffff;
	s->range_table=&range_daqp_ai;
	s->trig[0]=daqp_ai_a;

	return 1;
}


/* daqp_detach (called from comedi_comdig) does nothing. If the PCMCIA
 * card is removed, daqp_cs_detach() is called by the pcmcia subsystem.
 */

static int daqp_detach(comedi_device *dev)
{
	printk("comedi%d: detaching daqp\n",dev->minor);
	
	return 0;
}

/*====================================================================

    PCMCIA interface code

    The rest of the code in this file is based on dummy_cs.c v1.24
    from the Linux pcmcia_cs distribution v3.1.8 and is subject
    to the following license agreement.

    The remaining contents of this file are subject to the Mozilla Public
    License Version 1.1 (the "License"); you may not use this file
    except in compliance with the License. You may obtain a copy of
    the License at http://www.mozilla.org/MPL/

    Software distributed under the License is distributed on an "AS
    IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
    implied. See the License for the specific language governing
    rights and limitations under the License.

    The initial developer of the original code is David A. Hinds
    <dhinds@pcmcia.sourceforge.org>.  Portions created by David A. Hinds
    are Copyright (C) 1999 David A. Hinds.  All Rights Reserved.

    Alternatively, the contents of this file may be used under the
    terms of the GNU Public License version 2 (the "GPL"), in which
    case the provisions of the GPL are applicable instead of the
    above.  If you wish to allow the use of your version of this file
    only under the terms of the GPL and not to allow others to use
    your version of this file under the MPL, indicate your decision
    by deleting the provisions above and replace them with the notice
    and other provisions required by the GPL.  If you do not delete
    the provisions above, a recipient may use your version of this
    file under either the MPL or the GPL.
    
======================================================================*/

/*
   The event() function is this driver's Card Services event handler.
   It will be called by Card Services when an appropriate card status
   event is received.  The config() and release() entry points are
   used to configure or release a socket, in response to card
   insertion and ejection events.
*/

static void daqp_cs_config(dev_link_t *link);
static void daqp_cs_release(u_long arg);
static int daqp_cs_event(event_t event, int priority,
		       event_callback_args_t *args);

/*
   The attach() and detach() entry points are used to create and destroy
   "instances" of the driver, where each instance represents everything
   needed to manage one actual PCMCIA card.
*/

static dev_link_t *daqp_cs_attach(void);
static void daqp_cs_detach(dev_link_t *);

/*
   The dev_info variable is the "key" that is used to match up this
   device driver with appropriate cards, through the card configuration
   database.
*/

static dev_info_t dev_info = "daqp_cs";

/*====================================================================*/

static void cs_error(client_handle_t handle, int func, int ret)
{
    error_info_t err = { func, ret };
    CardServices(ReportError, handle, &err);
}

/*======================================================================

    daqp_cs_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.
    
======================================================================*/

static dev_link_t *daqp_cs_attach(void)
{
    local_info_t *local;
    dev_link_t *link;
    client_reg_t client_reg;
    int ret, i;
    
    DEBUG(0, "daqp_cs_attach()\n");

    for (i = 0; i < MAX_DEV; i++)
      if (dev_table[i] == NULL) break;
    if (i == MAX_DEV) {
      printk(KERN_NOTICE "daqp_cs: no devices available\n");
      return NULL;
    }
    
    /* Allocate space for private device-specific data */
    local = kmalloc(sizeof(local_info_t), GFP_KERNEL);
    if (!local) return NULL;
    memset(local, 0, sizeof(local_info_t));

    local->table_index = i;
    dev_table[i] = local;
    link = &local->link;
    link->priv = local;

    init_waitqueue(&local->endofscan);

    /* Initialize the dev_link_t structure */
    link->release.function = &daqp_cs_release;
    link->release.data = (u_long)link;

    /* Interrupt setup */
    link->irq.Attributes = IRQ_TYPE_EXCLUSIVE | IRQ_HANDLE_PRESENT;
    link->irq.IRQInfo1 = IRQ_INFO2_VALID|IRQ_LEVEL_ID;
    if (irq_list[0] == -1)
	link->irq.IRQInfo2 = irq_mask;
    else
	for (i = 0; i < 4; i++)
	    link->irq.IRQInfo2 |= 1 << irq_list[i];
    link->irq.Handler = daqp_interrupt;
    link->irq.Instance = local;
    
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
    client_reg.dev_info = &dev_info;
    client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
    client_reg.EventMask =
	CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
	CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
	CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;
    client_reg.event_handler = &daqp_cs_event;
    client_reg.Version = 0x0210;
    client_reg.event_callback_args.client_data = link;
    ret = CardServices(RegisterClient, &link->handle, &client_reg);
    if (ret != CS_SUCCESS) {
	cs_error(link->handle, RegisterClient, ret);
	daqp_cs_detach(link);
	return NULL;
    }

    return link;
} /* daqp_cs_attach */

/*======================================================================

    This deletes a driver "instance".  The device is de-registered
    with Card Services.  If it has been released, all local data
    structures are freed.  Otherwise, the structures will be freed
    when the device is released.

======================================================================*/

static void daqp_cs_detach(dev_link_t *link)
{
    local_info_t *dev = link->priv;

    DEBUG(0, "daqp_cs_detach(0x%p)\n", link);
    
    /*
       If the device is currently configured and active, we won't
       actually delete it yet.  Instead, it is marked so that when
       the release() function is called, that will trigger a proper
       detach().
    */
    if (link->state & DEV_CONFIG) {
#ifdef PCMCIA_DEBUG
	printk(KERN_DEBUG "daqp_cs: detach postponed, '%s' "
	       "still locked\n", link->dev->dev_name);
#endif
	link->state |= DEV_STALE_LINK;
	return;
    }

    /* Break the link with Card Services */
    if (link->handle)
	CardServices(DeregisterClient, link->handle);
    
    /* Unlink device structure, and free it */
    dev_table[dev->table_index] = NULL;
    kfree(dev);
    
} /* daqp_cs_detach */

/*======================================================================

    daqp_cs_config() is scheduled to run after a CARD_INSERTION event
    is received, to configure the PCMCIA socket, and to make the
    device available to the system.
    
======================================================================*/

#define CS_CHECK(fn, args...) \
while ((last_ret=CardServices(last_fn=(fn),args))!=0) goto cs_failed

#define CFG_CHECK(fn, args...) \
if (CardServices(fn, args) != 0) goto next_entry

static void daqp_cs_config(dev_link_t *link)
{
    client_handle_t handle = link->handle;
    local_info_t *dev = link->priv;
    tuple_t tuple;
    cisparse_t parse;
    int last_fn, last_ret;
    u_char buf[64];
    config_info_t conf;
    
    DEBUG(0, "daqp_cs_config(0x%p)\n", link);

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
	cistpl_cftable_entry_t dflt = { 0 };
	cistpl_cftable_entry_t *cfg = &(parse.cftable_entry);
	CFG_CHECK(GetTupleData, handle, &tuple);
	CFG_CHECK(ParseTuple, handle, &tuple, &parse);

	if (cfg->flags & CISTPL_CFTABLE_DEFAULT) dflt = *cfg;
	if (cfg->index == 0) goto next_entry;
	link->conf.ConfigIndex = cfg->index;
	
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
	}

	/* This reserves IO space but doesn't actually enable it */
	CFG_CHECK(RequestIO, link->handle, &link->io);

	/* If we got this far, we're cool! */
	break;
	
    next_entry:
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
    sprintf(dev->node.dev_name, "daqp%d", dev->table_index);
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
    printk("\n");
    
    link->state &= ~DEV_CONFIG_PENDING;
    return;

cs_failed:
    cs_error(link->handle, last_fn, last_ret);
    daqp_cs_release((u_long)link);

} /* daqp_cs_config */

/*======================================================================

    After a card is removed, daqp_cs_release() will unregister the
    device, and release the PCMCIA configuration.  If the device is
    still open, this will be postponed until it is closed.
    
======================================================================*/

static void daqp_cs_release(u_long arg)
{
    dev_link_t *link = (dev_link_t *)arg;

    DEBUG(0, "daqp_cs_release(0x%p)\n", link);

    /*
       If the device is currently in use, we won't release until it
       is actually closed, because until then, we can't be sure that
       no one will try to access the device or its data structures.
    */
    if (link->open) {
	DEBUG(1, "daqp_cs: release postponed, '%s' still open\n",
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

    CardServices(ReleaseConfiguration, link->handle);
    if (link->io.NumPorts1)
	CardServices(ReleaseIO, link->handle, &link->io);
    if (link->irq.AssignedIRQ)
	CardServices(ReleaseIRQ, link->handle, &link->irq);
    link->state &= ~DEV_CONFIG;
    
    if (link->state & DEV_STALE_LINK)
	daqp_cs_detach(link);
    
} /* daqp_cs_release */

/*======================================================================

    The card status event handler.  Mostly, this schedules other
    stuff to run after an event is received.

    When a CARD_REMOVAL event is received, we immediately set a
    private flag to block future accesses to this device.  All the
    functions that actually access the device should check this flag
    to make sure the card is still present.
    
======================================================================*/

static int daqp_cs_event(event_t event, int priority,
		       event_callback_args_t *args)
{
    dev_link_t *link = args->client_data;
    local_info_t *dev = link->priv;
    
    DEBUG(1, "daqp_cs_event(0x%06x)\n", event);
    
    switch (event) {
    case CS_EVENT_CARD_REMOVAL:
	link->state &= ~DEV_PRESENT;
	if (link->state & DEV_CONFIG) {
	    dev->stop = 1;
	    link->release.expires = jiffies + HZ/20;
	    add_timer(&link->release);
	}
	break;
    case CS_EVENT_CARD_INSERTION:
	link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
	daqp_cs_config(link);
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
	break;
    }
    return 0;
} /* daqp_cs_event */

/*====================================================================*/

#ifdef MODULE

int init_module(void)
{
    servinfo_t serv;
    DEBUG(0, "%s\n", version);
    CardServices(GetCardServicesInfo, &serv);
    if (serv.Revision != CS_RELEASE_CODE) {
	printk(KERN_NOTICE "daqp_cs: Card Services release "
	       "does not match!\n");
	return -1;
    }
    register_pccard_driver(&dev_info, &daqp_cs_attach, &daqp_cs_detach);
    comedi_driver_register(&driver_daqp);
    return 0;
}

void cleanup_module(void)
{
    int i;

    DEBUG(0, "daqp_cs: unloading\n");
    comedi_driver_unregister(&driver_daqp);
    unregister_pccard_driver(&dev_info);
    for (i=0; i < MAX_DEV; i++) {
      if (dev_table[i]) {
        if (dev_table[i]->link.state & DEV_CONFIG) {
          daqp_cs_release((u_long)(&dev_table[i]->link));
        }
        daqp_cs_detach(&dev_table[i]->link);
      }
    }
}

#endif
