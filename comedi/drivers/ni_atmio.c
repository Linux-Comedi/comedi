/*
    module/atmio-E.c
    Hardware driver for NI AT-MIO E series cards

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-8 David A. Schleef <ds@stm.lbl.gov>

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
		name:		"at-mio-16e-1",
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	8192,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	800,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	2048,
		ao_unipolar:	1,
		has_8255:	0,
		n_gpct:         0,
		caldac:		type1,
	},
	{	device_id:	25,
		name:		"at-mio-16e-2",
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	2048,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	2000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	2048,
		ao_unipolar:	1,
		has_8255:	0,
		n_gpct:         0,
		caldac:		type1,
	},
	{	device_id:	36,
		name:		"at-mio-16e-10",
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	512,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	10000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	0,
		ao_unipolar:	1,
		caldac:		type1,
		n_gpct:         2,
		has_8255:	0,
	},
	{	device_id:	37,
		name:		"at-mio-16de-10",
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	512,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	10000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	0,
		ao_unipolar:	1,
		caldac:		type1,
		n_gpct:         0,
		has_8255:	1,
	},
	{	device_id:	38,
		name:		"at-mio-64e-3",
		n_adchan:	64,
		adbits:		12,
		ai_fifo_depth:	2048,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	2000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	2048,
		ao_unipolar:	1,
		has_8255:	0,
		n_gpct:         0,
		caldac:		type1,
	},
	{	device_id:	39,
		name:		"at-mio-16xe-50",
		n_adchan:	16,
		adbits:		16,
		ai_fifo_depth:	512,
		alwaysdither:	1,
		gainlkup:	ai_gain_8,
		ai_speed:	50000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	0,
		ao_unipolar:	0,
		caldac:		type2,
		n_gpct:         0,
		has_8255:	0,
	},
	{	device_id:	50,
		name:		"at-mio-16xe-10",
		n_adchan:	16,
		adbits:		16,
		ai_fifo_depth:	512,
		alwaysdither:	1,	/* unknown */
		gainlkup:	ai_gain_14,
		ai_speed:	10000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	0,	/* unknown */
		ao_unipolar:	0,	/* unknown */
		caldac:		type2,
		n_gpct:         0,
		has_8255:	0,
	},
	{	device_id:	51,
		name:		"at-ai-16xe-10",
		n_adchan:	16,
		adbits:		16,
		ai_fifo_depth:	512,
		alwaysdither:	1,	/* unknown */
		gainlkup:	ai_gain_14,
		ai_speed:	10000,
		n_aochan:	0,
		aobits:		0,
		ao_fifo_depth:	0,
		aorangelkup:	0,
		ao_unipolar:	0,
		caldac:		type2,
		n_gpct:         0,
		has_8255:	0,
	}
};


static int ni_irqpin[]={-1,-1,-1,0,1,2,-1,3,-1,-1,4,5,6,-1,-1,7};

#define interrupt_pin(a)	(ni_irqpin[(a)])

#define IRQ_POLARITY 0

#define NI_E_IRQ_FLAGS		0


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
	unsigned short gpct_mode0;
	unsigned short gpct_mode1;
	unsigned short gpct_command0;
	unsigned short gpct_command1;
	unsigned short gpct_input_select0;
	unsigned short gpct_input_select1;

	unsigned int ai_chanlistptr;
	unsigned short ai_xorlist[512];
}ni_private;
#define devpriv ((ni_private *)dev->private)

static int atmio_attach(comedi_device *dev,comedi_devconfig *it);
static int atmio_detach(comedi_device *dev);
comedi_driver driver_atmio={
	driver_name:	"atmio-E",
	module:		&__this_module,
	attach:		atmio_attach,
	detach:		atmio_detach,
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
static int atmio_detach(comedi_device *dev)
{
	return atmio_E_free(dev);
}

static int atmio_attach(comedi_device *dev,comedi_devconfig *it)
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
	comedi_driver_register(&driver_atmio);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_atmio);
}
#endif
