/*

  DAS-08 adapter

  hack by David Schleef <ds@stm.lbl.gov>

  mostly borrowed from Warren J. Jasper <wjasper@tx.ncsu.edu>

  should be compatible with boards from Keithley Metrabyte and
  Computer Boards.

 * Copyright (C) 1998  Warren Jasper, David Schleef
 * All rights reserved.
 *
 * This software may be freely copied, modified, and redistributed
 * provided that this copyright notice is preserved on all copies.
 *
 * You may not distribute this software, in whole or in part, as part of
 * any commercial product without the express consent of the authors.
 *
 * There is no warranty or other guarantee of fitness of this software
 * for any purpose.  It is provided solely "as is".


*/


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/comedidev.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/malloc.h>
#include <linux/delay.h>
#include <8255.h>

/* general debugging messages */
#define DEBUG

/* specific debugging messages */
#undef DAVE1
#undef DAVE2
#undef DAVE3

/*
	on the non-AO board, the DA registers don't exist
	and the 8255 registers move up by 4
	it also doesn't use the GAIN_REG
*/


#define DAS08_SIZE 0x10

#define LSB_AND_CHNLS	0	/* A/D Data & Channel Register      */
#define MSB_DATA_BYTE	1
#define STATUS_REG	2	/* Channel Mux Scan Limits Register */
#define GAIN_REG	3	/* Programmable Gain Register       */
#define COUNTER_0_DATA	4	/* Counter 0 Register               */
#define COUNTER_1_DATA	5	/* Counter 1 Register               */
#define COUNTER_2_DATA	6	/* Counter 2 Register               */
#define COUNTER_CONTROL	7	/* Conter Control  Register         */
#define DA_CHAN0_LSB	8	/* DAC 0 Low Byte                   */
#define DA_CHAN0_MSB	9	/* DAC 0 High Byte                  */
#define DA_CHAN1_LSB	10	/* DAC 1 Low Byte                   */
#define DA_CHAN1_MSB	11	/* DAC 1 High Byte                  */
#define DIO_PORTA	12	/* Port A 8 bit I/O of 8255         */
#define DIO_PORTB	13	/* Port B 8 bit I/O of 8255         */
#define DIO_PORTC	14	/* Port C 4+4 bit of 8255           */
#define DIO_CNTRL_REG	15	/* Mode and Direction Control Reg.  */

/*

 info provided by Carsten Zerbst about das08:
 agrees with pdf file from www.computerboards.com

port, read, write
0	A/D Bits 3-0(LSB)		Start 8 bit A/D conversion
1	A/D Bits 11-4(MSB)		Start 12 bit A/D conversion
2	EOC,IP1-IP3,IRQ,MUX Address	OP1-OP4, INTE & MUX address
3	not used			not used
4	Read counter 0			Load Counter 0
5	Read counter 1			Load Counter 1
6	Read counter 2			Load Counter 2
7	not used			Counter control
8	Port A input 8255		Port A Output
9	Port B input 			Port A output
10	Port C Input			Port A Output
11	None. No read Back on 8255	Configure 8255


pgh model: gains .5,1,5,10,50,100,500,1000, unipolar and bipolar
pgl model: gains .5,1.2,4,8, unipolar and bipolar

pgh bipolar ranges: +/- 10, +/-5, etc.
pgh unipolar ranges: 0 to { x,10,x,1,x,0.1,x,0.01 }
  (what does x mean?)

pgl bipolar ranges: +/- { 10,5,2.5,1.25,0.625 }
pgl unipolar ranges: 0 to { N/A, 10, 5, 2.5, 1.25 }


das08jr info:

4	DAC0 lsb
5	DAC0 msb
6	DAC1 lsb
7	DAC1 msb



*/



/*************************************************************************
* STATUS_REG         base_reg+2             Status Register              *
**************************************************************************/

/* Read */
#define MUX0  0x01      /* Current multiplexor channel */
#define MUX1  0x02      /* Current multiplexor channel */
#define MUX2  0x04      /* Current multiplexor channel */
#define IRQ   0x08      /* 1 = positive edge detected  */
#define IP1   0x10      /* digial input line IP1       */
#define IP2   0x20      /* digial input line IP2       */
#define IP3   0x40      /* digial input line IP3       */
#define EOC   0x80      /* 1=A/D busy, 0=not busy      */

/* Write */
#define INTE  0x08     /* 1=enable interrupts, 0=disable */
#define OP1   0x10     /* digital output line OP1        */
#define OP2   0x20     /* digital output line OP2        */
#define OP3   0x40     /* digital output line OP3        */
#define OP4   0x80     /* digital output line OP4        */

/*************************************************************************
* Constants for dealing with the 8254 counters                           *
**************************************************************************/

#define MODE0 0x0
#define MODE1 0x2
#define MODE2 0x4
#define MODE3 0x6
#define MODE4 0x8
#define MODE5 0xa

#define C0 0x00
#define C1 0x40
#define C2 0x80

#define _LATCH    0x00		/* LATCH gets caught up with timex.h */
#define LSBONLY  0x10
#define MSBONLY  0x20
#define LSBFIRST 0x30

#define S0       0x00
#define S1       0x02

typedef struct{
	int boardtype;
	int dio;
	int aip[16];
}das08_private;
#define devpriv ((das08_private *)dev->private)

struct boardtype_struct{
	char name[20];
	int ai_chans;
	int ao_chans;
};
static struct boardtype_struct boardtypes[]={
  {"das08",8,0},
  {"das08-aol",8,0},
  {"das08-pgh",8,0},
  {"das08-pgl",8,0}
};
#define this_board (boardtypes[devpriv->boardtype])
#define n_boardtypes (sizeof(boardtypes)/sizeof(boardtypes[0]))

static int das08_attach(comedi_device *dev,comedi_devconfig *it);
static int das08_detach(comedi_device *dev);
comedi_driver driver_das08={
	driver_name:	"das08",
	module:		THIS_MODULE,
	attach:		das08_attach,
	detach:		das08_detach,
	board_name:	boardtypes,
	num_names:	n_boardtypes,
	offset:		sizeof(struct boardtype_struct),
};



static int das08_ai(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int i,lsb,msb;
	int chan;

	chan=CR_CHAN(it->chanlist[0]);
	
#ifndef DAVE1
	/* possibly clears A/D queue */
	inb(dev->iobase+LSB_AND_CHNLS);
	inb(dev->iobase+MSB_DATA_BYTE);
#endif

	/* set multiplexer */
	devpriv->dio &= ~0xf;
	devpriv->dio |= chan;
	outb_p(devpriv->dio,dev->iobase+STATUS_REG);

	/* XXX do we have to wait for MUX to settle?  how long? */
	
	/* how to set gain? */

	/* trigger conversion */
#ifndef DAVE3
	outb_p(0,dev->iobase+LSB_AND_CHNLS);
#else
	outb_p(0,dev->iobase+MSB_DATA_BYTE);
#endif
#ifdef DAVE2
	/* wait for conversion to take place */
	udelay(25);
	
	msb=inb(dev->iobase+MSB_DATA_BYTE);
	lsb=inb(dev->iobase+LSB_AND_CHNLS);
	it->data[0]=(lsb >> 4) | (msb << 4);
	return 1;
#else
	for(i=0;i<200;i++){
		if(!(inb_p(dev->iobase+STATUS_REG) & EOC)){
			msb=inb(dev->iobase+MSB_DATA_BYTE);
			lsb=inb(dev->iobase+LSB_AND_CHNLS);
			it->data[0]=(lsb >> 4) | (msb << 4);
			return 1;
		}
		udelay(5);
	}
#ifdef DEBUG
	rt_printk("das08: ai timeout\n");
#endif
	return -ETIME;
#endif
}


static int das08_ao(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int lsb,msb;

	lsb=it->data[0]&0xff;
	msb=(it->data[0]>>8)&0xf;
	if(CR_CHAN(it->chanlist[0])==0){
		outb(lsb,dev->iobase+DA_CHAN0_LSB);
		outb(msb,dev->iobase+DA_CHAN0_MSB);
	}else{
		outb(lsb,dev->iobase+DA_CHAN1_LSB);
		outb(msb,dev->iobase+DA_CHAN1_MSB);
	}
	return 1;
}

static int das08_do(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	do_pack(&s->state,it);
	
	devpriv->dio &= ~0xf0;
	devpriv->dio |= s->state << 4;
	outb_p(devpriv->dio,dev->iobase+STATUS_REG);

	return it->n_chan;
}

static int das08_di(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	unsigned int bits;
	
	bits=(inb(dev->iobase+STATUS_REG)>>4)&0x7;
	
	return di_unpack(bits,it);
}

static int das08_attach(comedi_device *dev,comedi_devconfig *it)
{
	int i;
	comedi_subdevice *s;
	int ret;
	
	dev->iobase=it->options[0];
	printk("comedi%d: das08: 0x%04x",dev->minor,dev->iobase);
	if(check_region(dev->iobase,DAS08_SIZE)<0){
		printk(" I/O port conflict\n");
		return -EIO;
	}
	dev->board_name="das08";
	dev->iosize=DAS08_SIZE;
	dev->irq=0;
	
#ifdef DEBUG
	printk("\nboard fingerprint:\n");
	for(i=0;i<DAS08_SIZE;i++){
		printk("%02x ",inb_p(dev->iobase+i));
	}
#endif
	dev->n_subdevices=5;
	if((ret=alloc_subdevices(dev))<0)
		return ret;
	if((ret=alloc_private(dev,sizeof(das08_private)))<0)
		return ret;
	
	request_region(dev->iobase,DAS08_SIZE,"das08");
	
	devpriv->boardtype=0;
	
	s=dev->subdevices+0;
	/* ai */
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=8;
	s->maxdata=0xfff;
	s->range_table=&range_unknown;	/* XXX */
	s->trig[0]=das08_ai;

	s=dev->subdevices+1;
	/* ao */
	if(this_board.ao_chans>0){
		s->type=COMEDI_SUBD_AO;
		s->subdev_flags=SDF_WRITEABLE;
		s->n_chan=2;
		s->maxdata=0xfff;
		s->range_table=&range_unknown;	/* XXX */
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}
	s->trig[0]=das08_ao;

	s=dev->subdevices+2;
	subdev_8255_init(dev,s,NULL,(void *)(dev->iobase+DIO_PORTA));

	s=dev->subdevices+3;
	/* ai */
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=3;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->trig[0]=das08_di;

	s=dev->subdevices+4;
	/* ai */
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=4;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->trig[0]=das08_do;

	devpriv->dio=0;
	

	if(this_board.ao_chans>0){
		/* zero AO */
		outb(0,dev->iobase+DA_CHAN0_LSB);
		outb(0,dev->iobase+DA_CHAN0_MSB);
		outb(0,dev->iobase+DA_CHAN1_LSB);
		outb(0,dev->iobase+DA_CHAN1_MSB);
	}
	
	outb_p(0,dev->iobase+STATUS_REG);
	
#if 0
	outb_p(0,dev->iobase+DIO_PORTA);
	outb_p(0,dev->iobase+DIO_PORTB);
	outb_p(0,dev->iobase+DIO_PORTC);
#endif
	
	printk("\n");

	return 0;
}

static int das08_detach(comedi_device *dev)
{
	printk("comedi%d: das08: remove\n",dev->minor);
	
	release_region(dev->iobase,dev->iosize);
	
	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_das08);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_das08);
}
#endif
