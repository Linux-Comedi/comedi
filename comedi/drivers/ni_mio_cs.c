/*
    comedi/drivers/ni_mio_cs.c
    Hardware driver for NI PCMCIA MIO E series cards

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2000 David A. Schleef <ds@stm.lbl.gov>

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
	The real guts of the driver is in ni_mio_common.c, which is
	included by all the E series drivers.

	References for specifications:
	
	   341080a.pdf  DAQCard E Series Register Level Programmer Manual
	
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/malloc.h>
#ifdef CONFIG_COMEDI_RTL
#include <linux/rtl.h>
#endif
#include <linux/comedidev.h>
#include "ni_stc.h"
#include "8255.h"

#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/ds.h>

#undef DEBUG

#define ATMIO 1
#undef PCIMIO

/*
 *  AT specific setup
 */

#define NI_SIZE 0x20

static struct caldac_struct *type1[]={&caldac_mb88341,NULL,NULL};
static struct caldac_struct *type2[]={&caldac_dac8800,&caldac_dac8043,NULL};

#define MAX_N_CALDACS 12

static ni_board ni_boards[]={
	{	device_id:	0x010d,
		name:		"DAQCard-ai-16xe-50",
		n_adchan:	16,
		adbits:		16,
		ai_fifo_depth:	1024,
		alwaysdither:	0,
		gainlkup:	ai_gain_8,
		ai_speed:	5000,
		n_aochan:	0,
		aobits:		0,
		ao_fifo_depth:	0,
		ao_unipolar:	0,
		has_8255:	0,
		caldac:		type2,
	},
	{	device_id:	0x010c,
		name:		"DAQCard-ai-16e-4",
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	1024,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	4000,
		n_aochan:	0,
		aobits:		0,
		ao_fifo_depth:	0,
		ao_unipolar:	0,
		has_8255:	0,
		caldac:		type1, /* or ad8804 */
	},
	{	device_id:	0x02c4,
		name:		"DAQCard-6062E",
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	1024,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	2000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	2048,
		ao_unipolar:	0,
		has_8255:	0,
		caldac:		type2,
	},
	{	device_id:	0x0000,
		name:		"DAQCard-6024E", /* specs incorrect! */
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	1024,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	5000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	0,
		ao_unipolar:	0,
		has_8255:	0,
		caldac:		type2,
	},
};


#define interrupt_pin(a)	0

#define IRQ_POLARITY 1

#define NI_E_IRQ_FLAGS		SA_SHIRQ


/* How we access registers */

#define ni_writew(a,b)		(outw((a),(b)+dev->iobase))
#define ni_readw(a)		(inw((a)+dev->iobase))
#define ni_writeb(a,b)		(outb((a),(b)+dev->iobase))
#define ni_readb(a)		(inb((a)+dev->iobase))
#define ni_writeb_p(a,b)	(outb_p((a),(b)+dev->iobase))
#define ni_readb_p(a)		(inb_p((a)+dev->iobase))


/*
 * this is how we access windowed registers
 */

#define win_out(a,b) (ni_writew((b),Window_Address),ni_writew((a),Window_Data))
#define win_in(b) (ni_writew((b),Window_Address),ni_readw(Window_Data))
#define win_save() (ni_readw(Window_Address))
#define win_restore(a) (ni_writew((a),Window_Address))


typedef struct{
	dev_link_t *link;

	NI_PRIVATE_COMMON
}ni_private;
#define devpriv ((ni_private *)dev->private)

static int mio_cs_attach(comedi_device *dev,comedi_devconfig *it);
static int mio_cs_detach(comedi_device *dev);
comedi_driver driver_ni_mio_cs={
	driver_name:	"ni_mio_cs",
	module:		THIS_MODULE,
	attach:		mio_cs_attach,
	detach:		mio_cs_detach,
};


#include "ni_mio_common.c"


static int ni_getboardtype(comedi_device *dev,dev_link_t *link);

/* clean up allocated resources */
/* called when driver is removed */
static int mio_cs_detach(comedi_device *dev)
{
#if 0
	/* PCMCIA layer does this for us */
	if(dev->iobase)
		release_region(dev->iobase,NI_SIZE);
#endif
	if(dev->irq){
		comedi_free_irq(dev->irq,dev);
	}

	return 0;
}

void mio_cs_config(dev_link_t *link);
static void cs_release(u_long arg);
static void cs_detach(dev_link_t *);
static int irq_mask;

static dev_link_t *dev_list = NULL;
static dev_info_t dev_info = "ni_mio_cs";
static dev_node_t dev_node = {
	"ni_mio_cs",
	COMEDI_MAJOR,0,
	NULL
};
static int mio_cs_event(event_t event, int priority, event_callback_args_t *args);

static void cs_error(client_handle_t handle, int func, int ret)
{
	error_info_t err = { func, ret };

	DPRINTK("cs_error(handle=%p, func=%d, ret=%d)\n",handle,func,ret);

	CardServices(ReportError, handle, &err);
}

static dev_link_t *cs_attach(void)
{
	dev_link_t *link;
	client_reg_t client_reg;
	int ret;

	link=kmalloc(sizeof(*link),GFP_KERNEL);
	if(!link)return NULL;
	memset(link,0,sizeof(*link));

	link->release.function = &cs_release;
	link->release.data = (u_long)link;
	
	link->io.Attributes1 = IO_DATA_PATH_WIDTH_16;
	link->io.NumPorts1 = 16;
	link->irq.Attributes = IRQ_TYPE_EXCLUSIVE;
	link->irq.IRQInfo1 = IRQ_INFO2_VALID|IRQ_LEVEL_ID;
	link->irq.IRQInfo2 = irq_mask;
	link->conf.Attributes = CONF_ENABLE_IRQ;
	link->conf.Vcc = 50;
	link->conf.IntType = INT_MEMORY_AND_IO;
	
	link->next = dev_list;
	dev_list = link;

	client_reg.dev_info = &dev_info;
	client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
	client_reg.EventMask =
		CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
		CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
		CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;
	client_reg.event_handler = &mio_cs_event;
	client_reg.Version = 0x0210;
	client_reg.event_callback_args.client_data = link;
	ret = CardServices(RegisterClient, &link->handle, &client_reg);
	if (ret != CS_SUCCESS) {
		cs_error(link->handle, RegisterClient, ret);
		printk("detaching...\n");
		cs_detach(link);
		return NULL;
	}

	return link;
}

static void cs_release(u_long arg)
{
	dev_link_t *link=(void *)arg;

	CardServices(ReleaseConfiguration, link->handle);
	CardServices(ReleaseIO, link->handle, &link->io);
	CardServices(ReleaseIRQ, link->handle, &link->irq);

	link->state &= ~DEV_CONFIG;
}

static void cs_detach(dev_link_t *link)
{
	dev_link_t **linkp;
	
	DPRINTK("cs_detach(link=%p)\n",link);
	
	for(linkp = &dev_list; *linkp; linkp = &(*linkp)->next)
		if (*linkp == link) break;
	if (*linkp==NULL)
		return;

	//save_flags
	//cli
	if (link->state & DEV_RELEASE_PENDING){
		printk("dev release pending bug\n");
		del_timer(&link->release);
		link->state &= ~DEV_RELEASE_PENDING;
	}
	//restore_flags

	if(link->state & DEV_CONFIG) {
		cs_release((u_long)link);
		if(link->state & DEV_STALE_CONFIG) {
			link->state |= DEV_STALE_LINK;
			return;
		}
	}

	if(link->handle){
		CardServices(DeregisterClient, link->handle);
	}

}

static int mio_cs_event(event_t event, int priority, event_callback_args_t *args)
{
	dev_link_t *link = args->client_data;

	DPRINTK("mio_cs_event(event=%x,priority=%d,args=%p)\n",event,priority,args);

	switch(event){
	case CS_EVENT_CARD_REMOVAL:
		DPRINTK("removal event\n");
		link->state &= ~DEV_PRESENT;
		if(link->state & DEV_CONFIG) {
			link->release.expires = jiffies+HZ/20;
			link->state |= DEV_RELEASE_PENDING;
			add_timer(&link->release);
		}
		/* XXX disable irq here, to get rid of spurious interrupts */
		break;
	case CS_EVENT_CARD_INSERTION:
		DPRINTK("card insertion event\n");
		link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
		mio_cs_config(link);
		break;
	case CS_EVENT_PM_SUSPEND:
		DPRINTK("pm suspend event\n");
		link->state |= DEV_SUSPEND;
		/* fall through */
	case CS_EVENT_RESET_PHYSICAL:
		DPRINTK("reset physical event\n");
		if(link->state & DEV_CONFIG)
			CardServices(ReleaseConfiguration, link->handle);
		break;
	case CS_EVENT_PM_RESUME:
		DPRINTK("pm resume event\n");
		link->state &= ~DEV_SUSPEND;
		/* fall through */
	case CS_EVENT_CARD_RESET:
		DPRINTK("card reset event\n");
		if(DEV_OK(link))
			CardServices(RequestConfiguration, link->handle, &link->conf);
		break;
	default:
		DPRINTK("unknown event (ignored)\n");
	}
	return 0;
}



void mio_cs_config(dev_link_t *link)
{
	client_handle_t handle = link->handle;
	tuple_t tuple;
	u_short buf[128];
	cisparse_t parse;
	int manfid = 0, prodid = 0;
	int ret;
	config_info_t conf;

	DPRINTK("mio_cs_config(link=%p)\n",link);

	tuple.TupleData = (cisdata_t *)buf;
	tuple.TupleOffset = 0;
	tuple.TupleDataMax = 255;
	tuple.Attributes = 0;
	
	tuple.DesiredTuple = CISTPL_CONFIG;
	ret=CardServices(GetFirstTuple, handle, &tuple);
	ret=CardServices(GetTupleData, handle, &tuple);
	ret=CardServices(ParseTuple, handle, &tuple, &parse);
	link->conf.ConfigBase = parse.config.base;
	link->conf.Present = parse.config.rmask[0];

	link->state |= DEV_CONFIG;

	CardServices(GetConfigurationInfo,handle,&conf);
	link->conf.Vcc=conf.Vcc;
#if 0
	tuple.DesiredTuple = CISTPL_LONGLINK_MFC;
	tuple.Attributes = TUPLE_RETURN_COMMON | TUPLE_RETURN_LINK;
	info->multi (first_tuple(handle, &tuple, &parse) == CS_SUCCESS);
#endif

	tuple.DesiredTuple = CISTPL_MANFID;
	tuple.Attributes = TUPLE_RETURN_COMMON;
	if((CardServices(GetFirstTuple,handle, &tuple) == CS_SUCCESS) &&
	   (CardServices(GetTupleData,handle,&tuple) == CS_SUCCESS)){
		manfid = le16_to_cpu(buf[0]);
		prodid = le16_to_cpu(buf[1]);
	}
	//printk("manfid = 0x%04x, 0x%04x\n",manfid,prodid);

	tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY;
	tuple.Attributes = 0;
	ret=CardServices(GetFirstTuple, handle, &tuple);
	ret=CardServices(GetTupleData, handle, &tuple);
	ret=CardServices(ParseTuple, handle, &tuple, &parse);

#if 0
	printk(" index: 0x%x\n",parse.cftable_entry.index);
	printk(" flags: 0x%x\n",parse.cftable_entry.flags);
	printk(" io flags: 0x%x\n",parse.cftable_entry.io.flags);
	printk(" io nwin: 0x%x\n",parse.cftable_entry.io.nwin);
	printk(" io base: 0x%x\n",parse.cftable_entry.io.win[0].base);
	printk(" io len: 0x%x\n",parse.cftable_entry.io.win[0].len);
	printk(" irq1: 0x%x\n",parse.cftable_entry.irq.IRQInfo1);
	printk(" irq2: 0x%x\n",parse.cftable_entry.irq.IRQInfo2);
	printk(" mem flags: 0x%x\n",parse.cftable_entry.mem.flags);
	printk(" mem nwin: 0x%x\n",parse.cftable_entry.mem.nwin);
	printk(" subtuples: 0x%x\n",parse.cftable_entry.subtuples);
#endif


#if 0
	link->io.NumPorts1=0x20;
	link->io.IOAddrLines=5;
	link->io.Attributes1=IO_DATA_PATH_WIDTH_AUTO;
#endif
	link->io.NumPorts1=parse.cftable_entry.io.win[0].len;
	link->io.IOAddrLines=parse.cftable_entry.io.flags & CISTPL_IO_LINES_MASK;
	link->io.NumPorts2=0;

	{
		int base;
		for(base=0x300;base<0x400;base+=0x20){
			link->io.BasePort1=base;
			ret=CardServices(RequestIO, handle, &link->io);
			//printk("RequestIO 0x%02x\n",ret);
			if(!ret)break;
		}
	}

	link->irq.IRQInfo1=parse.cftable_entry.irq.IRQInfo1;
	link->irq.IRQInfo2=parse.cftable_entry.irq.IRQInfo2;
	ret=CardServices(RequestIRQ, handle, &link->irq);
	//printk("RequestIRQ 0x%02x\n",ret);

	link->conf.ConfigIndex=1;

	ret=CardServices(RequestConfiguration, handle, &link->conf);
	//printk("RequestConfiguration %d\n",ret);

	link->dev = &dev_node;
	link->state &= ~DEV_CONFIG_PENDING;
}

static int mio_cs_attach(comedi_device *dev,comedi_devconfig *it)
{
	dev_link_t *link;
	int ret;
	
	DPRINTK("mio_cs_attach(dev=%p,it=%p)\n",dev,it);

	link = dev_list; /* XXX hack */
	if(!link)return 0;

	dev->driver=&driver_ni_mio_cs;
	dev->iobase=link->io.BasePort1;

	dev->irq=link->irq.AssignedIRQ;

	printk("comedi%d: %s: DAQCard: io 0x%04x, irq %d, ",
		dev->minor,dev->driver->driver_name,dev->iobase,
		dev->irq);

#if 0
	{
		int i;

		printk(" board fingerprint:");
		for(i=0;i<32;i+=2){
		printk(" %04x %02x",inw(dev->iobase+i),inb(dev->iobase+i+1));
		}
		printk("\n");
		printk(" board fingerprint (windowed):");
		for(i=0;i<10;i++){
			printk(" 0x%04x",win_in(i));
		}
		printk("\n");
	}
#endif

	dev->board_ptr = ni_boards + ni_getboardtype(dev,link);
	
	printk(" %s",boardtype.name);
	dev->board_name=boardtype.name;

	if( (ret=comedi_request_irq(dev->irq,ni_E_interrupt,NI_E_IRQ_FLAGS,"ni_mio_cs",dev))<0 ){
		printk(" irq not available\n");
		return -EINVAL;
	}
	
	/* allocate private area */
	if((ret=alloc_private(dev,sizeof(ni_private)))<0)
		return ret;
	
	if( (ret=ni_E_init(dev,it))<0 ){
		return ret;
	}

	return 0;
}


static int get_prodid(comedi_device *dev,dev_link_t *link)
{
	client_handle_t handle = link->handle;
	tuple_t tuple;
	u_short buf[128];
	int prodid = 0;

	tuple.TupleData = (cisdata_t *)buf;
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

static int ni_getboardtype(comedi_device *dev,dev_link_t *link)
{
	int id;
	int i;
	
	id = get_prodid(dev,link);

	for(i=0;i<n_ni_boards;i++){
		if(ni_boards[i].device_id==id){
			return i;
		}
	}

	printk("unknown board 0x%04x -- pretend it is a ",id);

	return 0;
}


#ifdef MODULE
int init_module(void)
{
	servinfo_t serv;

	comedi_driver_register(&driver_ni_mio_cs);
	CardServices(GetCardServicesInfo, &serv);
	if(serv.Revision != CS_RELEASE_CODE){
		printk(KERN_NOTICE "mio_cs: Card Services release "
			"does not match!\n");
		return -1;
	}
	register_pccard_driver(&dev_info, &cs_attach, &cs_detach);
	return 0;
}

void cleanup_module(void)
{
	unregister_pccard_driver(&dev_info);
#if 0
	while(dev_list != NULL)
		cs_detach(dev_list);
#endif
	comedi_driver_unregister(&driver_ni_mio_cs);
}
#endif
