/*
    module/atmio-E.c
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
	The real guts of the driver is in ni-E.c, which is included
	both here and in pcimio-E.c

	
	Interrupt support added by Truxton Fulton <trux@truxton.com>

	References for specifications:
	
	   321747b.pdf  Register Level Programmer Manual (obsolete)
	   321747c.pdf  Register Level Programmer Manual (new)
	   DAQ-STC reference manual

	Other possibly relevant info:
	
	   320517c.pdf  User manual (obsolete)
	   320517f.pdf  User manual (new)
	   320889a.pdf  delete
	   320906c.pdf  maximum signal ratings
	   321066a.pdf  about 16x
	   321791a.pdf  discontinuation of at-mio-16e-10 rev. c
	   321808a.pdf  about at-mio-16e-10 rev P
	   321837a.pdf  discontinuation of at-mio-16de-10 rev d
	   321838a.pdf  about at-mio-16de-10 rev N
	
	ISSUES:

	need to deal with external reference for DAC, and other DAC
	properties in board properties
	
	deal with at-mio-16de-10 revision D to N changes, etc.
	
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
#ifdef CONFIG_COMEDI_RT
#include <linux/rtl.h>
#endif
#include <comedi_module.h>
#include <ni_stc.h>
#include <8255.h>

#undef Status

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

static ni_board ni_boards[]={
	{	device_id:	44,
		name:		"xe-50",
		n_adchan:	16,
		adbits:		16,
		ai_fifo_depth:	8192,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	800,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	2048,
		ao_unipolar:	1,
		has_8255:	0,
		caldac:		type1,
	},
};


static int ni_irqpin[]={-1,-1,-1,0,1,2,-1,3,-1,-1,4,5,6,-1,-1,7};

#define interrupt_pin(a)	(ni_irqpin[(a)])

#define IRQ_POLARITY 0


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
	int dio;
	int ao0p,ao1p;
	int lastchan;
	int last_do;
	int rt_irq;
	int irqmask;
	int aimode;

	unsigned short ao_mode1;
	unsigned short ao_mode2;
	unsigned short ao_mode3;
	unsigned short ao_cmd1;
	unsigned short ao_cmd2;
	unsigned short ao_cmd3;
	unsigned short ao_trigger_select;
}ni_private;
#define devpriv ((ni_private *)dev->private)

static int mio_cs_attach(comedi_device *dev,comedi_devconfig *it);
static int mio_cs_detach(comedi_device *dev);
comedi_driver driver_atmio={
	driver_name:	"mio-cs",
	module:		&__this_module,
	attach:		mio_cs_attach,
	detach:		mio_cs_detach,
};


#include "ni_mio_common.c"


static int init_stage2(comedi_device *dev,comedi_devconfig *it);
static int ni_getboardtype(comedi_device *dev);

/* clean up allocated resources */
int atmio_E_free(comedi_device *dev)
{
	if(dev->iobase)
		release_region(dev->iobase,NI_SIZE);
	if(dev->irq){
		comedi_free_irq(dev->irq,dev);
	}

	return 0;
}

/* called when driver is removed */
static int mio_cs_detach(comedi_device *dev)
{
	return atmio_E_free(dev);
}

void mio_cs_config(dev_link_t *link);
static void cs_release(u_long arg);
static void cs_detach(dev_link_t *);
static int irq_mask;

static dev_link_t *dev_list = NULL;
static dev_info_t dev_info = "ni_mio_cs";
static int mio_cs_event(event_t event, int priority, event_callback_args_t *args);

static void cs_error(client_handle_t handle, int func, int ret)
{
	error_info_t err = { func, ret };
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
	
	printk("cs_detach\n");
	
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


	switch(event){
	case CS_EVENT_CARD_REMOVAL:
printk("removal event\n");
		link->state &= ~DEV_PRESENT;
		if(link->state & DEV_CONFIG) {
			link->release.expires = jiffies+HZ/20;
			link->state |= DEV_RELEASE_PENDING;
			add_timer(&link->release);
		}
		break;
	case CS_EVENT_CARD_INSERTION:
printk("insertion event\n");
		link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
		mio_cs_config(link);
		break;
	case CS_EVENT_PM_SUSPEND:
printk("suspend event\n");
		link->state |= DEV_SUSPEND;
		/* fall through */
	case CS_EVENT_RESET_PHYSICAL:
printk("physreset event\n");
		if(link->state & DEV_CONFIG)
			CardServices(ReleaseConfiguration, link->handle);
		break;
	case CS_EVENT_PM_RESUME:
printk("resume event\n");
		link->state &= ~DEV_SUSPEND;
		/* fall through */
	case CS_EVENT_CARD_RESET:
printk("reset event\n");
		if(DEV_OK(link))
			CardServices(RequestConfiguration, link->handle, &link->conf);
		break;
	}
	return 0;
}

static int mio_cs_attach(comedi_device *dev,comedi_devconfig *it)
{
	if(!strcmp("ni_E",it->board_name)){
		printk("comedi: 'ni_E' deprecated.  Use 'atmio-E'\n");
	}else if(!strcmp("atmio-E",it->board_name)){
		;
	}else{
		return 0;
	}

	return init_stage2(dev,it);
}


void mio_cs_config(dev_link_t *link)
{
	client_handle_t handle = link->handle;
	tuple_t tuple;
	u_short buf[128];
	cisparse_t parse;
	int manfid = 0, prodid = 0;
	int ret;

	tuple.TupleData = (cisdata_t *)buf;
	tuple.TupleOffset = 0;
	tuple.TupleDataMax = 255;
	tuple.Attributes = 0;
	tuple.DesiredTuple = CISTPL_CONFIG;

	ret=CardServices(GetFirstTuple, handle,&tuple);
	link->conf.ConfigBase = parse.config.base;
	link->conf.Present = parse.config.rmask[0];

	link->state |= DEV_CONFIG;

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

	tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY;
	tuple.Attributes = 0;
	CardServices(GetFirstTuple, handle, &tuple);
	CardServices(GetTupleData, handle, &tuple);
	CardServices(ParseTuple, handle, &tuple, &parse);
	CardServices(RequestIO, handle, &link->io);
	
	CardServices(RequestIRQ, handle, &link->irq);
	CardServices(RequestConfiguration, handle, &link->conf);

	printk("irq = %d\n",link->irq.AssignedIRQ);
	printk("iobase = 0x%04x\n",link->io.BasePort1);
	printk("manfid = 0x%04x, 0x%04x\n",manfid,prodid);
	
	tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY
	
}

static int init_stage2(comedi_device *dev,comedi_devconfig *it)
{
	int		ret;
	int		iobase;
	int		board;
	int		irq;

	
	/* reserve our I/O region */

	iobase=0x200;
	if(it->options[0])iobase=it->options[0];
	
	printk("comedi%d: ni_E: 0x%04x",dev->minor,iobase);
	if(check_region(iobase,NI_SIZE)<0){
		printk(" I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase,NI_SIZE,"ni_E");

	dev->iobase=iobase;
	dev->iosize=NI_SIZE;
	
	
	/* board existence sanity check */
	
#ifdef DEBUG
       {
               int i;

               printk(" board fingerprint:");
              for(i=0;i<16;i+=2){
                       printk(" %04x %02x",inw(dev->iobase+i),inb(dev->iobase+i+1));
               }
       }
#endif
	
	/* get board type */

	board=ni_getboardtype(dev);
	if(board<0)return -EIO;
	
	printk(" %s",ni_boards[board].name);
	dev->board_name=ni_boards[board].name;

	/* irq stuff */

	irq=it->options[1];
	if(irq!=0){
		if(irq<0 || irq>15 || ni_irqpin[irq]==-1){
			printk(" invalid irq\n");
			return -EINVAL;
		}
		printk(" ( irq = %d )",irq);
		if( (ret=comedi_request_irq(irq,ni_E_interrupt,NI_E_IRQ_FLAGS,"atmio-E",dev))<0 ){
			printk(" irq not available\n");
			return -EINVAL;
		}
		dev->irq=irq;
	}
	
	/* allocate private area */
	
	if((ret=alloc_private(dev,sizeof(ni_private)))<0)
		return ret;
	
	dev->board=board;

	/* generic E series stuff in ni-E.c */

	if( (ret=ni_E_init(dev,it))<0 ){
		return ret;
	}
	
	return 0;
}


static int ni_getboardtype(comedi_device *dev)
{
	int device_id=ni_read_eeprom(dev,511);
	int i;
	
	for(i=0;i<n_ni_boards;i++){
		if(ni_boards[i].device_id==device_id){
			return i;
		}
	}
	if(device_id==255){
		printk(" can't find board\n");
	}else if(device_id == 0){
		printk(" EEPROM read error (?) or device not found\n");
	}else{
		printk(" unknown device ID %d -- contact author\n",device_id);
	}
	return -1;
}


#ifdef MODULE
int init_module(void)
{
	servinfo_t serv;

	comedi_driver_register(&driver_atmio);
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
	comedi_driver_unregister(&driver_atmio);
}
#endif
