/*
    comedi/drivers/das16cs.c
    Skeleton code for a Comedi driver

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
Driver: cb_das16_cs.o
Description: Computer Boards PC-CARD DAS16/16
Devices: (ComputerBoards) PC-CARD DAS16/16 [cb_das16_cs], PC-CARD DAS16/16-AO
Author: ds
Updated: Mon, 04 Nov 2002 20:04:21 -0800
Status: experimental


*/

#include <linux/comedidev.h>
#include <linux/delay.h>
#include <linux/pci.h>

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
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


typedef struct das16cs_board_struct{
	char *name;
	int device_id;
	int n_ao_chans;
}das16cs_board;
static das16cs_board das16cs_boards[] = {
	{
	device_id:	0x0000, /* unknown */
	name:		"PC-CARD DAS16/16",
	n_ao_chans:	0,
	},
	{
	device_id:	0x0039,
	name:		"PC-CARD DAS16/16-AO",
	n_ao_chans:	2,
	},
};
#define n_boards (sizeof(das16cs_boards)/sizeof(das16cs_boards[0]))
#define thisboard ((das16cs_board *)dev->board_ptr)

typedef struct{
	dev_link_t *link;

	lsampl_t ao_readback[2];
	unsigned short status1;
	unsigned short status2;
}das16cs_private;
#define devpriv ((das16cs_private *)dev->private)

static int das16cs_attach(comedi_device *dev,comedi_devconfig *it);
static int das16cs_detach(comedi_device *dev);
static comedi_driver driver_das16cs={
	driver_name:	"cb_das16_cs",
	module:		THIS_MODULE,
	attach:		das16cs_attach,
	detach:		das16cs_detach,
};

static dev_link_t *dev_list = NULL;

static comedi_lrange das16cs_ai_range = { 4, {
	RANGE( -10, 10 ),
	RANGE( -5, 5 ),
	RANGE( -2.5, 2.5 ),
	RANGE( -1.25, 1.25 ),
}};


static irqreturn_t das16cs_interrupt(int irq, void *d, struct pt_regs *regs);
static int das16cs_ai_rinsn(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int das16cs_ai_cmd(comedi_device *dev,comedi_subdevice *s);
static int das16cs_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);
static int das16cs_ao_winsn(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int das16cs_ao_rinsn(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int das16cs_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int das16cs_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int das16cs_timer_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int das16cs_timer_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

static int get_prodid(comedi_device *dev, dev_link_t *link)
{
	client_handle_t handle = link->handle;
	tuple_t tuple;
	u_short buf[128];
	int prodid = 0;

	tuple.TupleData = (cisdata_t *) buf;
	tuple.TupleOffset = 0;
	tuple.TupleDataMax = 255;
	tuple.DesiredTuple = CISTPL_MANFID;
	tuple.Attributes = TUPLE_RETURN_COMMON;
	if((CardServices(GetFirstTuple,handle, &tuple) == CS_SUCCESS) &&
	   (CardServices(GetTupleData,handle,&tuple) == CS_SUCCESS)){
		prodid = le16_to_cpu(buf[1]);
	}

	return prodid;
}

static das16cs_board *das16cs_probe(comedi_device *dev, dev_link_t *link)
{
	int id;
	int i;

	id = get_prodid(dev,link);

	for(i=0;i<n_boards;i++){
		if(das16cs_boards[i].device_id==id){
			return das16cs_boards + i;
		}
	}

	printk("unknown board!\n");

	return NULL;
}

static int das16cs_attach(comedi_device *dev,comedi_devconfig *it)
{
	dev_link_t *link;
	comedi_subdevice *s;
	int ret;
	int i;

	printk("comedi%d: cb_das16_cs: ",dev->minor);
	
	link = dev_list; /* XXX hack */
	if(!link)return -EIO;

	dev->iobase = link->io.BasePort1;
	printk("I/O base=0x%04x ",dev->iobase);

	printk("fingerprint:\n");
	for(i=0;i<48;i+=2){
		printk("%04x ",inw(dev->iobase + i));
	}
	printk("\n");

	ret = comedi_request_irq(link->irq.AssignedIRQ, das16cs_interrupt,
		SA_SHIRQ, "cb_das16_cs", dev);
	if(ret<0){
		return ret;
	}
	dev->irq = link->irq.AssignedIRQ;
	printk("irq=%d ",dev->irq);

	dev->board_ptr = das16cs_probe(dev, link);
	if(!dev->board_ptr)return -EIO;

	dev->board_name = thisboard->name;

	if(alloc_private(dev,sizeof(das16cs_private))<0)
		return -ENOMEM;

	if(alloc_subdevices(dev, 4)<0)
		return -ENOMEM;

	s=dev->subdevices+0;
	dev->read_subdev=s;
	/* analog input subdevice */
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE|SDF_GROUND|SDF_DIFF;
	s->n_chan=16;
	s->maxdata=0xffff;
	s->range_table=&das16cs_ai_range;
	s->len_chanlist=16;
	s->insn_read = das16cs_ai_rinsn;
	s->do_cmd = das16cs_ai_cmd;
	s->do_cmdtest = das16cs_ai_cmdtest;

	s=dev->subdevices+1;
	/* analog output subdevice */
	if(thisboard->n_ao_chans){
		s->type=COMEDI_SUBD_AO;
		s->subdev_flags=SDF_WRITABLE;
		s->n_chan=thisboard->n_ao_chans;
		s->maxdata=0xffff;
		s->range_table = &range_bipolar10;
		s->insn_write = &das16cs_ao_winsn;
		s->insn_read = &das16cs_ao_rinsn;
	}

	s=dev->subdevices+2;
	/* digital i/o subdevice */
	if(1){
		s->type=COMEDI_SUBD_DIO;
		s->subdev_flags=SDF_READABLE|SDF_WRITABLE;
		s->n_chan=8;
		s->maxdata=1;
		s->range_table=&range_digital;
		s->insn_bits = das16cs_dio_insn_bits;
		s->insn_config = das16cs_dio_insn_config;
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}
	
	s=dev->subdevices+3;
	/* timer subdevice */
	if(0){
		s->type=COMEDI_SUBD_TIMER;
		s->subdev_flags=SDF_READABLE|SDF_WRITABLE;
		s->n_chan=1;
		s->maxdata=0xff;
		s->range_table = &range_unknown;
		s->insn_read = das16cs_timer_insn_read;
		s->insn_config = das16cs_timer_insn_config;
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}

	printk("attached\n");

	return 1;
}

static int das16cs_detach(comedi_device *dev)
{
	printk("comedi%d: das16cs: remove\n",dev->minor);

	if(dev->irq){
		comedi_free_irq(dev->irq, dev);
	}
	
	return 0;
}


static irqreturn_t das16cs_interrupt(int irq, void *d, struct pt_regs *regs)
{
	//comedi_device *dev = d;
	return IRQ_HANDLED;
}

/*
 * "instructions" read/write data in "one-shot" or "software-triggered"
 * mode.
 */
static int das16cs_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
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
	devpriv->status1 |= (aref==AREF_DIFF)?0:0x0020;
	outw(devpriv->status1, dev->iobase + 4);

	devpriv->status2 &= ~0xff00;
	devpriv->status2 |= range_bits[range];
	outw(devpriv->status2, dev->iobase + 6);

	for(i=0;i<insn->n;i++){
		outw(0, dev->iobase);

#define TIMEOUT 1000
		for(to=0;to<TIMEOUT;to++){
			if(inw(dev->iobase + 4) & 0x0080)break;
		}
		if(to==TIMEOUT){
			printk("cb_das16_cs: ai timeout\n");
			return -ETIME;
		}
		data[i] = (unsigned short)inw(dev->iobase + 0);
	}

	return i;
}

static int das16cs_ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	return -EINVAL;
}

static int das16cs_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd)
{
	int err=0;
	int tmp;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* step 1: make sure trigger sources are trivially valid */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER|TRIG_EXT;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER|TRIG_EXT;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT|TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	/* note that mutual compatiblity is not an issue here */
	if(cmd->scan_begin_src!=TRIG_TIMER &&
	   cmd->scan_begin_src!=TRIG_EXT)err++;
	if(cmd->convert_src!=TRIG_TIMER &&
	   cmd->convert_src!=TRIG_EXT)err++;
	if(cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_NONE)err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}

#define MAX_SPEED	10000		/* in nanoseconds */
#define MIN_SPEED	1000000000	/* in nanoseconds */

	if(cmd->scan_begin_src==TRIG_TIMER){
		if(cmd->scan_begin_arg<MAX_SPEED){
			cmd->scan_begin_arg=MAX_SPEED;
			err++;
		}
		if(cmd->scan_begin_arg>MIN_SPEED){
			cmd->scan_begin_arg=MIN_SPEED;
			err++;
		}
	}else{
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		/* should specify multiple external triggers */
		if(cmd->scan_begin_arg>9){
			cmd->scan_begin_arg=9;
			err++;
		}
	}
	if(cmd->convert_src==TRIG_TIMER){
		if(cmd->convert_arg<MAX_SPEED){
			cmd->convert_arg=MAX_SPEED;
			err++;
		}
		if(cmd->convert_arg>MIN_SPEED){
			cmd->convert_arg=MIN_SPEED;
			err++;
		}
	}else{
		/* external trigger */
		/* see above */
		if(cmd->convert_arg>9){
			cmd->convert_arg=9;
			err++;
		}
	}

	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src==TRIG_COUNT){
		if(cmd->stop_arg>0x00ffffff){
			cmd->stop_arg=0x00ffffff;
			err++;
		}
	}else{
		/* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	if(cmd->scan_begin_src==TRIG_TIMER){
		unsigned int div1, div2;

		tmp=cmd->scan_begin_arg;
		i8253_cascade_ns_to_timer(100, &div1, &div2, 
			&cmd->scan_begin_arg, cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->scan_begin_arg)err++;
	}
	if(cmd->convert_src==TRIG_TIMER){
		unsigned int div1, div2;

		tmp=cmd->convert_arg;
		i8253_cascade_ns_to_timer(100, &div1, &div2, 
			&cmd->scan_begin_arg, cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->convert_arg)err++;
		if(cmd->scan_begin_src==TRIG_TIMER &&
		  cmd->scan_begin_arg<cmd->convert_arg*cmd->scan_end_arg){
			cmd->scan_begin_arg=cmd->convert_arg*cmd->scan_end_arg;
			err++;
		}
	}

	if(err)return 4;

	return 0;
}

static int das16cs_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);
	unsigned short status1;
	unsigned short d;
	int bit;

	for(i=0;i<insn->n;i++){
		devpriv->ao_readback[chan] = data[i];
		d = data[i];

		//devpriv->status1 |= 0x0009;
		outw(devpriv->status1, dev->iobase + 4);
		comedi_udelay(1);

		status1 = devpriv->status1;
		//if(chan)status1 &= ~0x0008;
		//else status1 &= ~0x0001;
		if(chan)status1 |= 0x0008;
		else status1 |= 0x0001;

		printk("0x%04x\n",status1);
		outw(status1, dev->iobase + 4);
		comedi_udelay(1);

		for(bit=15;bit>=0;bit--){
			int b = (d>>bit)&1;

			printk("0x%04x\n",status1 | b | 0x0000);
			outw(status1 | b | 0x0000, dev->iobase + 4);
			comedi_udelay(1);
			printk("0x%04x\n",status1 | b | 0x0004);
			outw(status1 | b | 0x0004, dev->iobase + 4);
			comedi_udelay(1);
		}

		outw(devpriv->status1, dev->iobase + 4);
	}

	return i;
}

/* AO subdevices should have a read insn as well as a write insn.
 * Usually this means copying a value stored in devpriv. */
static int das16cs_ao_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	for(i=0;i<insn->n;i++)
		data[i] = devpriv->ao_readback[chan];

	return i;
}

/* DIO devices are slightly special.  Although it is possible to
 * implement the insn_read/insn_write interface, it is much more
 * useful to applications if you implement the insn_bits interface.
 * This allows packed reading/writing of the DIO channels.  The
 * comedi core can convert between insn_bits and insn_read/write */
static int das16cs_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=2)return -EINVAL;

	if(data[0]){
		s->state &= ~data[0];
		s->state |= data[0]&data[1];

		outw(s->state,dev->iobase + 16);
	}

	/* on return, data[1] contains the value of the digital
	 * input and output lines. */
	data[1]=inw(dev->iobase + 16);

	return 2;
}

static int das16cs_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int chan=CR_CHAN(insn->chanspec);
	int bits;

	if(insn->n!=1)return -EINVAL;

	if(chan<4)bits=0x0f;
	else bits=0xf0;

	if(data[0]==COMEDI_OUTPUT){
		s->io_bits |= bits;
	}else{
		s->io_bits &= bits;
	}

	devpriv->status2 &= ~0x00c0;
	devpriv->status2 |= (s->io_bits&0xf0)?0x0080:0;
	devpriv->status2 |= (s->io_bits&0x0f)?0x0040:0;

	outw(devpriv->status2,dev->iobase + 6);

	return 1;
}

static int das16cs_timer_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	return -EINVAL;
}

static int das16cs_timer_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
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
MODULE_PARM(pc_debug, "i");
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
static char *version =
"cb_das16_cs.c pcmcia code (David Schleef), modified from dummy_cs.c 1.31 2001/08/24 12:13:13 (David Hinds)";
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

static void das16cs_pcmcia_config(dev_link_t *link);
static void das16cs_pcmcia_release(u_long arg);
static int das16cs_pcmcia_event(event_t event, int priority,
		       event_callback_args_t *args);

/*
   The attach() and detach() entry points are used to create and destroy
   "instances" of the driver, where each instance represents everything
   needed to manage one actual PCMCIA card.
*/

static dev_link_t *das16cs_pcmcia_attach(void);
static void das16cs_pcmcia_detach(dev_link_t *);

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

static dev_info_t dev_info = "cb_das16_cs";

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

static void my_cs_error(client_handle_t handle, int func, int ret)
{
    error_info_t err = { func, ret };
    CardServices(ReportError, handle, &err);
}

/*======================================================================

    das16cs_pcmcia_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.

======================================================================*/

static dev_link_t *das16cs_pcmcia_attach(void)
{
    local_info_t *local;
    dev_link_t *link;
    client_reg_t client_reg;
    int ret, i;

    DEBUG(0, "das16cs_pcmcia_attach()\n");

    /* Allocate space for private device-specific data */
    local = kmalloc(sizeof(local_info_t), GFP_KERNEL);
    if (!local) return NULL;
    memset(local, 0, sizeof(local_info_t));
    link = &local->link; link->priv = local;

    /* Initialize the dev_link_t structure */
    link->release.function = &das16cs_pcmcia_release;
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
    link->next = dev_list;
    dev_list = link;
    client_reg.dev_info = &dev_info;
    client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
    client_reg.EventMask =
	CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
	CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
	CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;
    client_reg.event_handler = &das16cs_pcmcia_event;
    client_reg.Version = 0x0210;
    client_reg.event_callback_args.client_data = link;
    ret = CardServices(RegisterClient, &link->handle, &client_reg);
    if (ret != CS_SUCCESS) {
	my_cs_error(link->handle, RegisterClient, ret);
	das16cs_pcmcia_detach(link);
	return NULL;
    }

    return link;
} /* das16cs_pcmcia_attach */

/*======================================================================

    This deletes a driver "instance".  The device is de-registered
    with Card Services.  If it has been released, all local data
    structures are freed.  Otherwise, the structures will be freed
    when the device is released.

======================================================================*/

static void das16cs_pcmcia_detach(dev_link_t *link)
{
	dev_link_t **linkp;

	DEBUG(0, "das16cs_pcmcia_detach(0x%p)\n", link);

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
		printk(KERN_DEBUG "das16cs: detach postponed, '%s' "
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

} /* das16cs_pcmcia_detach */

/*======================================================================

    das16cs_pcmcia_config() is scheduled to run after a CARD_INSERTION event
    is received, to configure the PCMCIA socket, and to make the
    device available to the system.

======================================================================*/

#define CS_CHECK(fn, args...) \
while ((last_ret=CardServices(last_fn=(fn),args))!=0) goto cs_failed

#define CFG_CHECK(fn, args...) \
if (CardServices(fn, args) != 0) goto next_entry

static void das16cs_pcmcia_config(dev_link_t *link)
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

	DEBUG(0, "das16cs_pcmcia_config(0x%p)\n", link);

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
		CFG_CHECK(RequestIO, link->handle, &link->io);
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
	sprintf(dev->node.dev_name, "cb_das16_cs");
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
	my_cs_error(link->handle, last_fn, last_ret);
	das16cs_pcmcia_release((u_long)link);

} /* das16cs_pcmcia_config */

/*======================================================================

    After a card is removed, das16cs_pcmcia_release() will unregister the
    device, and release the PCMCIA configuration.  If the device is
    still open, this will be postponed until it is closed.

======================================================================*/

static void das16cs_pcmcia_release(u_long arg)
{
	dev_link_t *link = (dev_link_t *)arg;

	DEBUG(0, "das16cs_pcmcia_release(0x%p)\n", link);

    /*
       If the device is currently in use, we won't release until it
       is actually closed, because until then, we can't be sure that
       no one will try to access the device or its data structures.
    */
	if (link->open)
	{
		DEBUG(1, "das16cs: release postponed, '%s' still open\n",
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
		das16cs_pcmcia_detach(link);

} /* das16cs_pcmcia_release */

/*======================================================================

    The card status event handler.  Mostly, this schedules other
    stuff to run after an event is received.

    When a CARD_REMOVAL event is received, we immediately set a
    private flag to block future accesses to this device.  All the
    functions that actually access the device should check this flag
    to make sure the card is still present.

======================================================================*/

static int das16cs_pcmcia_event(event_t event, int priority,
	event_callback_args_t *args)
{
	dev_link_t *link = args->client_data;
	local_info_t *dev = link->priv;

	DEBUG(1, "das16cs_pcmcia_event(0x%06x)\n", event);

	switch (event)
	{
		case CS_EVENT_CARD_REMOVAL:
			link->state &= ~DEV_PRESENT;
			if (link->state & DEV_CONFIG)
			{
				((local_info_t *)link->priv)->stop = 1;
				mod_timer(&link->release, jiffies + HZ/20);
			}
			break;
		case CS_EVENT_CARD_INSERTION:
			link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
			dev->bus = args->bus;
			das16cs_pcmcia_config(link);
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
} /* das16cs_pcmcia_event */

/*====================================================================*/

static int __init init_das16cs_pcmcia_cs(void)
{
	servinfo_t serv;
	DEBUG(0, "%s\n", version);
	CardServices(GetCardServicesInfo, &serv);
	if (serv.Revision != CS_RELEASE_CODE)
	{
		printk(KERN_NOTICE "das16cs: Card Services release "
			"does not match!\n");
		//return -1;
	}
	register_pccard_driver(&dev_info, &das16cs_pcmcia_attach, &das16cs_pcmcia_detach);
	return 0;
}

static void __exit exit_das16cs_pcmcia_cs(void)
{
	DEBUG(0, "das16cs_pcmcia_cs: unloading\n");
	unregister_pccard_driver(&dev_info);
	while (dev_list != NULL)
	{
		del_timer(&dev_list->release);
		if (dev_list->state & DEV_CONFIG)
			das16cs_pcmcia_release((u_long)dev_list);
		das16cs_pcmcia_detach(dev_list);
	}
}

int init_module(void)
{
	int ret;

	ret = init_das16cs_pcmcia_cs();
	if(ret < 0)
		return ret;

	return comedi_driver_register(&driver_das16cs);
}

void cleanup_module(void)
{
	exit_das16cs_pcmcia_cs();
	comedi_driver_unregister(&driver_das16cs);
}

#else
COMEDI_INITCLEANUP(driver_das16cs);
#endif //CONFIG_PCMCIA
