/*
    comedi/drivers/das16.c
    DAS16 driver

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@stm.lbl.gov>

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
#include <comedi_module.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/malloc.h>
#include <linux/delay.h>
#include <8255.h>


/*
    cio-das16.pdf

    "das16"
    "das16/f"

  0	a/d bits 0-3		start 12 bit
  1	a/d bits 4-11		unused
  2	mux read		mux set
  3	di 4 bit		do 4 bit
  4	unused			ao0_lsb
  5	unused			ao0_msb
  6	unused			ao1_lsb
  7	unused			ao1_msb
  8	status eoc uni/bip	interrupt reset
  9	dma, int, trig ctrl	set dma, int
  a	pacer control		unused
  b	reserved		reserved
  cdef	8254
  0123	8255

*/

/*
    cio-das16jr.pdf

    "das16jr"

  0	a/d bits 0-3		start 12 bit
  1	a/d bits 4-11		unused
  2	mux read		mux set
  3	di 4 bit		do 4 bit
  4567	unused			unused
  8	status eoc uni/bip	interrupt reset
  9	dma, int, trig ctrl	set dma, int
  a	pacer control		unused
  b	gain status		gain control
  cdef	8254

*/

/*
    cio-das160x-1x.pdf

    "das1601/12"
    "das1602/12"
    "das1602/16"

  0	a/d bits 0-3		start 12 bit
  1	a/d bits 4-11		unused
  2	mux read		mux set
  3	di 4 bit		do 4 bit
  4	unused			ao0_lsb
  5	unused			ao0_msb
  6	unused			ao1_lsb
  7	unused			ao1_msb
  8	status eoc uni/bip	interrupt reset
  9	dma, int, trig ctrl	set dma, int
  a	pacer control		unused
  b	gain status		gain control
  cdef	8254
  400	8255
  404	unused			conversion enable
  405	unused			burst enable
  406	unused			das1600 enable
  407	status

*/

#define DAS16_TRIG		0
#define DAS16_AI_LSB		0
#define DAS16_AI_MSB		1
#define DAS16_MUX		2
#define DAS16_DIO		3
#define DAS16_AO_LSB(x)	((x)?6:4)
#define DAS16_AO_MSB(x)	((x)?7:5)
#define DAS16_STATUS		8
#define   DAS16_EOC			(1<<7)
#define   DAS16_UB			(1<<6)
#define   DAS16_MUXbit			(1<<5)
#define   DAS16_INT			(1<<4)
#define   DAS16_INT			(1<<4)
#define DAS16_CONTROL		9
#define   DAS16_INTE			(1<<7)
#define   DAS16_IRQ(x)			((x)<<4)
#define   DAS16_DMA			(1<<2)
#define   DAS16_TS(x)			((x)<<0)
#define DAS16_PACER		0x0a
#define   DAS16_CTR0			(1<<1)
#define   DAS16_TRIG0			(1<<0)
#define DAS16_GAIN		0x0b

#define DAS1600_CONV		0x404
#define DAS1600_BURST		0x405
#define   DAS1600_BURST_VAL		0x40
#define DAS1600_ENABLE		0x406
#define   DAS1600_ENABLE_VAL		0x40
#define DAS1600_BURST_STATUS	0x407
#define   DAS1600_BME			(1<<7)
#define   DAS1600_ME			(1<<6)
#define   DAS1600_CD			(1<<5)
#define   DAS1600_WS			(1<<1)
#define   DAS1600_CLK			(1<<0)


static int das16jr_gainlist[] = { 8, 0, 1, 2, 3, 4, 5, 6, 7 };
static int das1600_gainlist[] = { 0, 1, 2, 3 };

static int das16_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn);
static int das16_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn);
static int das16_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn);
static int das16_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn);
static int das08jr16_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn);



struct das_board_struct{
	char		*name;
	void		*ai;
	unsigned int	ai_nbits;
	void		*ao;
	unsigned int	ao_nbits;
	void		*di;
	void		*do_;
	unsigned int ai_type;

	unsigned int i8255_offset;
	unsigned int i8254_offset;
};
static struct das_board_struct boards[]={
	{
	name:		"das16",	// cio-das16.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x10,
	i8254_offset:	0x0c,
	},
	{
	name:		"das16/jr",	// cio-das16jr.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ao:		NULL,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	},
	{
	name:		"das1401/12",	// cio-das1400_series.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ao:		NULL,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	},
	{
	name:		"das1402/12",	// cio-das1400_series.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ao:		NULL,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	},
	{
	name:		"das1402/16",	// cio-das1400_series.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	16,
	ao:		NULL,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	},
	{
	name:		"das1601/12",	// cio-das160x-1x.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x400,
	i8254_offset:	0x0c,
	},
	{
	name:		"das1602/12",	// cio-das160x-1x.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x400,
	i8254_offset:	0x0c,
	},
	{
	name:		"das1602/16",	// cio-das160x-1x.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	16,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x400,
	i8254_offset:	0x0c,
	},
	{
	name:		"das16/330",	// ?
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	},
	{
	name:		"das16/330i",	// ?
	},
	{
	name:		"das16/f",	// das16.pdf
	// faster version of das16
	},
	{
	name:		"das16/jr/ctr5", // ?
	},
	{
	name:		"das16/n1",	// cio-das-m1-16.pdf ?
	},
	{
	name:		"das1601/12-p5",//
	},
	{
	name:		"das1602/12-p5",//
	},
};

#define TIMEOUT 1000

static comedi_lrange range_das1x01_bip = { 4, {
	BIP_RANGE( 10 ),
	BIP_RANGE( 1 ),
	BIP_RANGE( 0.1 ),
	BIP_RANGE( 0.01 ),
}};
static comedi_lrange range_das1x01_unip = { 4, {
	UNI_RANGE( 10 ),
	UNI_RANGE( 1 ),
	UNI_RANGE( 0.1 ),
	UNI_RANGE( 0.01 ),
}};
static comedi_lrange range_das1x02_bip = { 4, {
	BIP_RANGE( 10 ),
	BIP_RANGE( 5 ),
	BIP_RANGE( 2.5 ),
	BIP_RANGE( 1.25 ),
}};
static comedi_lrange range_das1x02_unip = { 4, {
	UNI_RANGE( 10 ),
	UNI_RANGE( 5 ),
	UNI_RANGE( 2.5 ),
	UNI_RANGE( 1.25 ),
}};
static comedi_lrange range_das16jr = { 9, {
	// also used by 16/330
	BIP_RANGE( 10 ),
	BIP_RANGE( 5 ),
	BIP_RANGE( 2.5 ),
	BIP_RANGE( 1.25 ),
	BIP_RANGE( 0.625 ),
	UNI_RANGE( 10 ),
	UNI_RANGE( 5 ),
	UNI_RANGE( 2.5 ),
	UNIP_RANGE( 1.25 ),
}};


static int das08jr16_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn)
{
	int n;
	int lsb,msb;
	int chan;

	lsb=insn->data[0]&0xff;
	msb=(insn->data[0]>>8)&0xff;

	chan=CR_CHAN(insn->chanspec);

	for(n=0;n<insn->n;n++){
#if 0
		outb(lsb,dev->iobase+devpriv->ao_offset_lsb[chan]);
		outb(msb,dev->iobase+devpriv->ao_offset_msb[chan]);
#else
		outb(lsb,dev->iobase+DAS08AO_AO_LSB(chan));
		outb(msb,dev->iobase+DAS08AO_AO_MSB(chan));
#endif

		/* load DACs */
		inb(dev->iobase+DAS08AO_UPDATE);

		/* XXX */
		break;
	}

	return n;
}

static int das16_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn)
{
	int i,n;

	/* clear crap */
	inb(dev->iobase+DAS16_LSB);
	inb(dev->iobase+DAS16_MSB);

	/* set multiplexer */
	outb_p(chan,dev->iobase+DAS16_MUX);
	
	/* set gain */
	if(board->have_pg){
		range = CR_RANGE(insn->chanspec);
		outb(das16_gainlist[range],dev->iobase+DAS16_GAIN);
	}

	/* How long should we wait for MUX to settle? */
	//udelay(5);

	for(n=0;n<insn->n;n++){
		/* trigger conversion */
		outb_p(0,dev->iobase+DAS16_TRIG);

		for(i=0;i<TIMEOUT;i++){
			if(!(inb(DAS16_STATUS)&DAS16_EOC))
				break;
		}
		if(i==TIMEOUT){
			rt_printk("das08: timeout\n");
			return -ETIME;
		}
		msb = inb(dev->iobase + DAS16_MSB);
		lsb = inb(dev->iobase + DAS16_LSB);
		if(board->ai_nbits==12){
			insn->data[n] = (lsb>>4) | (msb << 4);
		}else{
			insn->data[n] = lsb | (msb << 8);
		}
	}

	return n;
}

static void das16_init(comedi_device *dev)
{
	outb(DAS16_IRQ(dev->irq),dev->iobase+DAS16_CONTROL);
	outb(0,DAS16_PACER);

}

static int das16_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn)
{
	insn->data[0]=inb(dev->iobase+DAS08JR_DIO)&0xf;

	return 1;
}

static int das16_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn)
{
	outb(insn->data[0],dev->iobase+DAS08JR_DIO);

	return 1;
}

static int das16_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn)
{
	int n;
	int lsb,msb;
	int chan;

	if(board->ao_nbits==12){
		lsb=(insn->data[0]<<4)&0xff;
		msb=(insn->data[0]>>4)&0xff;
	}else{
		lsb=insn->data[0]&0xff;
		msb=(insn->data[0]>>8)&0xff;
	}

	chan=CR_CHAN(insn->chanspec);

	for(n=0;n<insn->n;n++){
#if 0
		outb(lsb,dev->iobase+devpriv->ao_offset_lsb[chan]);
		outb(msb,dev->iobase+devpriv->ao_offset_msb[chan]);
#else
		outb(lsb,dev->iobase+DAS16_AO_LSB(chan));
		outb(msb,dev->iobase+DAS16_AO_MSB(chan));
#endif
		/* XXX */
		break;
	}

	return n;
}

static void das1600_init(comedi_device *dev)
{
	int status, extstatus;

	status = inb(dev->iobase + DAS16_STATUS);
	extstatus = inb(dev->iobase + DAS1600_BURST_STATUS);

	if((extstatus & 0xfc)==0x10){
		/* probably a 1600 board */
	}

	if((extstatus & DAS1600_CLK)){
		/* clock is 10 Mhz */
	}else{
		/* clock is 1 Mhz */
	}

	outb(DAS1600_ENABLE_VAL,dev->iobase+DAS1600_ENABLE);

	if((status & DAS16_UB)){
		/* unipolar */
	}else{
		/* bipolar */
	}

	if((status & DAS16_MUX)){
		/* single ended */
	}else{
		/* differential */
	}

	printk("4 bit: 0x%02x\n",inb(dev->iobase + DAS16_DIO));
	/* 0x80: das16 */
	/* 0x00: das16jr, das16/330 */
	/* 0xc0: das1600, das1400 */

}


