/*
   module/das1600.c
   hardware driver for Keithley Metrabyte DAS16, DAS1600 and compatibles

   COMEDI - Linux Control and Measurement Device Interface
   Copyright (C) 1999 Anders Blomdell <anders.blomdell@control.lth.se>

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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/malloc.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <linux/comedidev.h>
#include <8255.h>


#define DAS1600_BASE_SIZE 16
#define DAS1600_DIO_SIZE 8

#define DAS1600_ADC_Low         0x000	// ADC-Low register
#define DAS1600_ADC_High        0x001	// ADC-High register
#define DAS1600_Channel_Mux     0x002	// Channel MUX register
#define DAS1600_Digital_4_Bit   0x003	// 4-bit digital in/out
#define DAS1600_DA0_Low         0x004	// DA0-Low register
#define DAS1600_DA0_High        0x005	// DA0-High register
#define DAS1600_DA1_Low         0x006	// DA1-Low register
#define DAS1600_DA1_High        0x007	// DA1-High register
#define DAS1600_Status          0x008	// Status register
#define DAS1600_Control         0x009	// DMA, interrupt and trigger control
#define DAS1600_Pacer_Control   0x00a	// Pacer clock control
#define DAS1600_Gain_Control    0x00b	// Gain control
#define DAS1600_Counter_0       0x00c	// 8254 counter 0 data
#define DAS1600_Counter_1       0x00d	// 8254 counter 1 data
#define DAS1600_Counter_2       0x00e	// 8254 counter 2 data
#define DAS1600_Counter_Control 0x00f	// 8254 counter control
#define DAS1600_Port_A          0x400	// 8255 port A
#define DAS1600_Port_B          0x401	// 8255 port B
#define DAS1600_Port_C          0x402	// 8255 port C
#define DAS1600_Control_8255    0x403	// 8255 control
#define DAS1600_Convert_Disable 0x404	// Disable AD conversion
#define DAS1600_Mode_Enable     0x405	// Enable DAS1600 mode
#define DAS1600_Burst_Enable    0x406	// Enable burst mode
#define DAS1600_Burst_Status    0x407	// Burst mode status

/* Some (or all?) K. M. das16/1600 boards have a bug, apparently due to the 
 * sample/hold amplifier being slower than the a/d converter.  This causes rapid 
 * succesive a/d conversions on different channels to return wrong values.
 * Sampling each channel twice and discarding the first reading seems to get
 * around this.  Define SLOW_SH_BUG to use this work-around.
 *
 * It is also reported to be helpful to ground any ai channels that are not in
 * use. 
 *
 * Thanks to Anders Blomdell for pointing this out.*/

#define SLOW_SH_BUG

/* Old, 1980's vintage K. M. das16 and das16F cards do not have software
 * adjustable input gains.  Defining NO_SOFT_RANGE will prevent the driver from
 * attmpting to set the gain.
 */

/*#define NO_SOFT_RANGE*/

static void das1600_release_resources(comedi_device * dev);

static comedi_lrange range_das1601_ai_10_bipolar = { 4, {
	RANGE( -10,	10 ),
	RANGE( -1,	1 ),
	RANGE( -0.1,	0.1 ),
	RANGE( -0.01,	0.01 )
}};
static comedi_lrange range_das1601_ai_10_unipolar = { 4, {
	RANGE( 0,	10 ),
	RANGE( 0,	1 ),
	RANGE( 0,	0.1 ),
	RANGE( 0,	0.01 )
}};
static comedi_lrange range_das1602_ai_10_bipolar = { 4, {
	RANGE( -10,	10 ),
	RANGE( -5,	5 ),
	RANGE( -2.5,	2.5 ),
	RANGE( -1.25,	1.25 )
}};
static comedi_lrange range_das1602_ai_10_unipolar = { 4, {
	RANGE( 0,	10 ),
	RANGE( 0,	5 ),
	RANGE( 0,	2.5 ),
	RANGE( 0,	1.25 )
}};
static comedi_lrange range_das1600_ao_extern_bipolar = { 1, {
	RANGE_ext( -1,	1 )
}};
static comedi_lrange range_das1600_ao_extern_unipolar = { 1, {
	RANGE_ext( -1,	1 )
}};
static comedi_lrange _range_bipolar10 = { 1, {
	RANGE( -10,	10 )
}};
static comedi_lrange _range_bipolar5 = { 1, {
	RANGE( -5,	5 )
}};
static comedi_lrange _range_unipolar10 = { 1, {
	RANGE( 0,	10 )
}};
static comedi_lrange _range_unipolar5 = { 1, {
	RANGE( 0,	5 )
}};

typedef struct{
	char *name;
	int ai_bits;
	comedi_lrange *bip_range;
	comedi_lrange *unip_range;
}boardtype;
static boardtype boardtypes[]={
	{ "das1601/12", 12,
	bip_range:	&range_das1601_ai_10_bipolar,
	unip_range:	&range_das1601_ai_10_unipolar,
	},
	{ "das1602/12", 12,
	bip_range:	&range_das1602_ai_10_bipolar,
	unip_range:	&range_das1602_ai_10_unipolar,
	},
	{ "das1602/16", 16,
	bip_range:	&range_das1602_ai_10_bipolar,
	unip_range:	&range_das1602_ai_10_unipolar,
	},
};
#define n_boardtypes sizeof(boardtypes)/sizeof(boardtype)
#define this_board ((boardtype *)dev->board_ptr)

static int das1600_attach(comedi_device *dev,comedi_devconfig *it);
static int das1600_detach(comedi_device *dev);
comedi_driver driver_das1600={
	driver_name:	"das1600",
	module:		THIS_MODULE,
	attach:		das1600_attach,
	detach:		das1600_detach,
	board_name:	boardtypes,
	num_names:	n_boardtypes,
	offset:		sizeof(boardtype),
};


typedef struct {
	int dma;
	int crystal;
	enum {
		adc_diff, adc_singleended
	} adc_mux;
	enum {
		adc_bipolar10, adc_unipolar10
	} adc_range;
	enum {
		dac_bipolar10, dac_bipolar5, dac_bipolaruser,
		dac_unipolar10, dac_unipolar5, dac_unipolaruser,
	} dac_range[2];
	comedi_lrange *range_type_list[2];
} das1600_private;

#define devpriv ((das1600_private *)dev->private)

static comedi_lrange *range_types[] =
{
	&_range_bipolar10,
	&_range_bipolar5,
	&range_das1600_ao_extern_bipolar,
	&_range_unipolar10,
	&_range_unipolar5,
	&range_das1600_ao_extern_unipolar
};

#define DAS1600_TIMEOUT 100


static int das1600_ai(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
  int i;

#ifdef SLOW_SH_BUG
  int j;
#endif
  
  for(i=0 ; i < it->n_chan ; i++) {
    int t, gain, hi, lo, chan;

	if (it->mode != 0)
		return -EINVAL;

#ifndef NO_SOFT_RANGE 
	gain = CR_RANGE(it->chanlist[i]);
	outb(gain, dev->iobase + DAS1600_Gain_Control);
#endif

	chan = CR_CHAN(it->chanlist[i]);
	
#ifdef SLOW_SH_BUG
	
	for (j=0 ; j < 2 ; j++) {
		outb(chan + (chan << 4), dev->iobase + DAS1600_Channel_Mux);
		outb(0, dev->iobase + DAS1600_ADC_Low);
   		for (t = 0; t < DAS1600_TIMEOUT; t++) {
			if ((inb(dev->iobase + DAS1600_Status) & 0x80) == 0) {
				break;
			}
		}
		lo = inb(dev->iobase + DAS1600_ADC_Low);
		hi = inb(dev->iobase + DAS1600_ADC_High);
	}
	
#else
	
	outb(chan + (chan << 4), dev->iobase + DAS1600_Channel_Mux);
	outb(0, dev->iobase + DAS1600_ADC_Low);
	for (t = 0; t < DAS1600_TIMEOUT; t++) {
		if ((inb(dev->iobase + DAS1600_Status) & 0x80) == 0) {
			break;
		}
	}
	lo = inb(dev->iobase + DAS1600_ADC_Low);
	hi = inb(dev->iobase + DAS1600_ADC_High); 

#endif

	if (t == DAS1600_TIMEOUT) {
		rt_printk("das1600: timeout\n");
	} 

	if(this_board->ai_bits==12){
		it->data[i] = (hi << 4) | (lo >> 4);
	}else{
		it->data[i] = (hi << 8) | lo;
	}
  }
  return i;
}


static int das1600_ao(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
  int i;
  for(i=0 ; i < it->n_chan ; i++) {
	int chan;
	int data;

		chan = CR_CHAN(it->chanlist[i]);
		data = it->data[i];

    outb((data & 0x00f) << 4, dev->iobase + ((chan) ? DAS1600_DA1_Low : DAS1600_DA0_Low));
    outb((data & 0xff0) >> 4, dev->iobase + ((chan) ? DAS1600_DA1_High : DAS1600_DA0_High));
	}
	return i;
}

#if 1

static int das1600_di(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	unsigned int bits;

	bits = inb(dev->iobase + DAS1600_Digital_4_Bit);

	di_unpack(bits,it);

	return it->n_chan;
}

static int das1600_do(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	do_pack(&s->state,it);

       /* the outputs are inverted... dumb... (Is this really true? can't
          find it in my docs...) */
	outb(s->state ^ 0xff, dev->iobase + DAS1600_Digital_4_Bit);

	return it->n_chan;
}

#else

static int das1600_di(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int data;
	int chan;
	int i;

	data= inb(dev->iobase + DAS1600_DI);
	for(i=0;i<it->n_chan;i++){
		chan=CR_CHAN(it->chanlist[i]);
		it->data[i]=(i>>chan)&1;
	}

	return i;
}

static int das1600_do(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int data;
	int chan;
	int mask;
	int i;

	data=devpriv->last_do;

	for(i=0;i<it->n_chan;i++){
		chan=CR_CHAN(it->chanlist[i]);
		mask=1<<chan;
		data &= ~mask;
		if(it->data[i])
			data |=mask;
	}
	devpriv->last_do=data;

	/* why are outputs inverted?  dumb... */
	outb(data ^ 0xff, dev->iobase + DAS1600_DO);

	return i;
}

#endif


/*
  options[0]   Board base address
  options[1]   IRQ
  options[2]   DMA level select configuration
                 0 == no DMA
                 1 == DMA level 1
                 3 == DMA level 3
  options[3]   Crystal select configuration
                 0 == 10 MHz
                 1 == 1 MHz
                 10 == 10 MHz
  options[4]   Input configuration
                 0 == differential
                 1 == single-ended
  options[5]   Analog input range configuration
                 0 == bipolar 10V  (-10V -- +10V)
                 1 == unipolar 10V  (0V -- +10V)
  options[6]   Analog output 0 range configuration
                 0 == bipolar 10V  (-10V -- +10V)
                 1 == bipolar 5V  (-5V -- +5V)
                 2 == bipolar user supplied  (-xV -- +xV)
                 3 == unipolar 10V  (0V -- +10V)
                 4 == unipolar 5V  (0V -- +5V)
                 5 == unipolar user supplied  (0V -- +xV)
  options[7]   Analog output 1 range configuration
                 0 == bipolar 10V  (-10V -- +10V)
                 1 == bipolar 5V  (-5V -- +5V)
                 2 == bipolar user supplied  (-xV -- +xV)
                 3 == unipolar 10V  (0V -- +10V)
                 4 == unipolar 5V  (0V -- +5V)
                 5 == unipolar user supplied  (0V -- +xV)
 */

static int das1600_attach(comedi_device * dev, comedi_devconfig * it)
{
	int result = 1;
	comedi_subdevice *s;

	dev->iobase = it->options[0];

	printk("comedi%d: das1600: 0x%04x ", dev->minor, dev->iobase);
	if (check_region(dev->iobase, DAS1600_BASE_SIZE) < 0 ||
	    check_region(dev->iobase + 0x400, DAS1600_DIO_SIZE) < 0) {
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(dev->iobase, DAS1600_BASE_SIZE, "das1600");
	request_region(dev->iobase + 0x400, DAS1600_DIO_SIZE, "das1600");

	dev->board_name=*(char **)dev->board_ptr;

	dev->n_subdevices=5;

	if((result=alloc_private(dev,sizeof(das1600_private)))<0)
		return result;
	if((result=alloc_subdevices(dev))<0)
		return result;

	devpriv->dma = it->options[2];
	devpriv->crystal = it->options[3];
	devpriv->adc_mux = (it->options[4] == 1) ? adc_singleended : adc_diff;
	devpriv->adc_range = it->options[5];
	devpriv->dac_range[0] = it->options[6];
	devpriv->dac_range[1] = it->options[7];

	s = dev->subdevices + 0;
	/* ai subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE|SDF_RT;
	s->n_chan = (devpriv->adc_mux == adc_singleended) ? 16 : 8;
	s->maxdata = (1<<this_board->ai_bits)-1;
	s->trig[0] = das1600_ai;
	switch (devpriv->adc_range) {
	case adc_bipolar10:
		s->range_table = this_board->bip_range;
		break;
	case adc_unipolar10:
		s->range_table = this_board->unip_range;
		break;
	}

	s++;
	/* ao subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE|SDF_RT;
	s->n_chan = 2;
	s->maxdata = 0xfff;
	s->trig[0] = das1600_ao;
	s->range_table_list = devpriv->range_type_list;
	devpriv->range_type_list[0] = range_types[devpriv->dac_range[0]];
	devpriv->range_type_list[1] = range_types[devpriv->dac_range[1]];

	s++;
	/* di subdevice */
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE|SDF_RT;
	s->trig[0] = das1600_di;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;

	s++;
	/* do subdevice */
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE|SDF_RT;
	s->trig[0] = das1600_do;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;

	s++;
	/* 8255 subdevice */
	subdev_8255_init(dev,s,NULL,(void *)(dev->iobase+DAS1600_Port_A));

	printk("\n");
	return 0;
}

static void das1600_release_resources(comedi_device * dev)
{
	if (dev->iobase) {
		release_region(dev->iobase, DAS1600_BASE_SIZE);
		release_region(dev->iobase + 0x400, DAS1600_DIO_SIZE);
	}
	if (dev->irq) {
		free_irq(dev->irq, dev);
	}
}

static int das1600_detach(comedi_device * dev)
{
	printk("comedi%d: das1600: remove\n", dev->minor);

	das1600_release_resources(dev);

	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_das1600);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_das1600);
}
#endif
