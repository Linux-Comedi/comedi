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
#include <linuxcomedidev.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/malloc.h>
#include <linux/delay.h>
#include <8255.h>


#define DAS16_SIZE 20

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
	UNI_RANGE( 1.25 ),
}};


static int das16jr_gainlist[] = { 8, 0, 1, 2, 3, 4, 5, 6, 7 };
static int das1600_gainlist[] = { 0, 1, 2, 3 };
enum {
	das16_pg_none = 0,
	das16_pg_16jr,
	das16_pg_1601,
	das16_pg_1602,
};
static int *das16_gainlists[] = {
	NULL,
	das16jr_gainlist,
	das1600_gainlist,
	das1600_gainlist,
};
static comedi_lrange *das16_ai_uni_lranges[]={
	&range_bipolar10, /* XXX guess */
	&range_das16jr,
	&range_das1x01_unip,
	&range_das1x02_unip,
};
static comedi_lrange *das16_ai_bip_lranges[]={
	&range_unipolar10, /* XXX guess */
	&range_das16jr,
	&range_das1x01_bip,
	&range_das1x02_bip,
};

static int das16_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das16_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das16_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das16_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);

struct das16_board_struct{
	char		*name;
	void		*ai;
	unsigned int	ai_nbits;
	unsigned int	ai_pg;
	void		*ao;
	unsigned int	ao_nbits;
	void		*di;
	void		*do_;

	unsigned int    i8255_offset;
	unsigned int    i8254_offset;

	unsigned int	size;
};
enum{	/* must match following array */
	das16_board_das16,
	das16_board_das16f,
	das16_board_das16jr,
	das16_board_das1401_12,
	das16_board_das1402_12,
	das16_board_das1402_16,
	das16_board_das1601_12,
	das16_board_das1602_12,
	das16_board_das1602_16,
	das16_board_das16_330,
};
static struct das16_board_struct das16_boards[]={
	{
	name:		"das16",	// cio-das16.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das16_pg_none,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x10,
	i8254_offset:	0x0c,
	size:		0x14,
	},
	{
	name:		"das16/f",	// das16.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das16_pg_none,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x10,
	i8254_offset:	0x0c,
	size:		0x14,
	},
	{
	name:		"das16/jr",	// cio-das16jr.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das16_pg_16jr,
	ao:		NULL,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x10,
	},
	{
	name:		"das1401/12",	// cio-das1400_series.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das16_pg_1601,
	ao:		NULL,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	},
	{
	name:		"das1402/12",	// cio-das1400_series.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das16_pg_1602,
	ao:		NULL,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	},
	{
	name:		"das1402/16",	// cio-das1400_series.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	16,
	ai_pg:		das16_pg_1602,
	ao:		NULL,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	},
	{
	name:		"das1601/12",	// cio-das160x-1x.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das16_pg_1601,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x400,
	i8254_offset:	0x0c,
	size:		0x408,
	},
	{
	name:		"das1602/12",	// cio-das160x-1x.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das16_pg_1602,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x400,
	i8254_offset:	0x0c,
	size:		0x408,
	},
	{
	name:		"das1602/16",	// cio-das160x-1x.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	16,
	ai_pg:		das16_pg_1602,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x400,
	i8254_offset:	0x0c,
	size:		0x408,
	},
	{
	name:		"das16/330",	// ?
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das16_pg_16jr,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	},
#if 0
	{
	name:		"das16/330i",	// ?
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
#endif
};
#define n_das16_boards ((sizeof(das16_boards))/(sizeof(das16_boards[0])))

static int das16_attach(comedi_device *dev,comedi_devconfig *it);
static int das16_detach(comedi_device *dev);
static int das16_recognize(char *name);
comedi_driver driver_das16={
	driver_name:	"das16",
	module:		THIS_MODULE,
	attach:		das16_attach,
	detach:		das16_detach,
	recognize:	das16_recognize,
};


#define TIMEOUT 1000


struct das16_private_struct {
	unsigned int	ai_unipolar;
	unsigned int	ai_singleended;
	unsigned int	clockbase;
};
#define devpriv ((struct das16_private_struct *)(dev->private))
#define thisboard ((struct das16_board_struct *)(dev->board_ptr))


static int das16_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i,n;
	int range;
	int chan;
	int msb,lsb;

	/* clear crap */
	inb(dev->iobase+DAS16_AI_LSB);
	inb(dev->iobase+DAS16_AI_MSB);

	/* set multiplexer */
	chan = CR_CHAN(insn->chanspec);
	outb_p(chan,dev->iobase+DAS16_MUX);
	
	/* set gain */
	if(thisboard->ai_pg != das16_pg_none){
		range = CR_RANGE(insn->chanspec);
		outb((das16_gainlists[thisboard->ai_pg])[range],
			dev->iobase+DAS16_GAIN);
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
			rt_printk("das16: timeout\n");
			return -ETIME;
		}
		msb = inb(dev->iobase + DAS16_AI_MSB);
		lsb = inb(dev->iobase + DAS16_AI_LSB);
		if(thisboard->ai_nbits==12){
			data[n] = (lsb>>4) | (msb << 4);
		}else{
			data[n] = lsb | (msb << 8);
		}
	}

	return n;
}

static int das16_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	data[0]=inb(dev->iobase+DAS16_DIO)&0xf;

	return 1;
}

static int das16_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	outb(data[0],dev->iobase+DAS16_DIO);

	return 1;
}

static int das16_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n;
	int lsb,msb;
	int chan;

	if(thisboard->ao_nbits==12){
		lsb=(data[0]<<4)&0xff;
		msb=(data[0]>>4)&0xff;
	}else{
		lsb=data[0]&0xff;
		msb=(data[0]>>8)&0xff;
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




static int detect_ao(comedi_device *dev)
{
	int in_aa;
	int in_55;

	outb(0xaa,dev->iobase+DAS16_AO_LSB(0));
	in_aa = inb(dev->iobase+DAS16_AO_LSB(0));

	outb(0x55,dev->iobase+DAS16_AO_LSB(0));
	in_55 = inb(dev->iobase+DAS16_AO_LSB(0));

	printk("detect_ao: 0xaa -> 0x%02x, 0x55 -> 0x%02x\n",in_aa,in_55);

	if((in_aa == 0xaa) && (in_55 == 0x55)){
		printk("possibly 16 bit ao?\n");
	}

	if(((in_aa & 0xf0) == 0xa0) && ((in_55 & 0xf0) == 0x50)){
		printk("ao test positive\n");
		return 1;
	}

	printk("ao test negative\n");

	return 0;
}

static int das16_probe(comedi_device *dev)
{
	int status;
	int burststatus;
	int diobits;

	/* status is available on all boards */

	status = inb(dev->iobase + DAS16_STATUS);

	if((status & DAS16_UB)){
		devpriv->ai_unipolar = 1;
	}else{
		devpriv->ai_unipolar = 0;
	}

	if((status & DAS16_MUX)){
		devpriv->ai_singleended = 1;
	}else{
		devpriv->ai_singleended = 0;
	}

	/* diobits indicates boards */

	diobits = inb(dev->iobase + DAS16_DIO) & 0xf0;

printk("diobits 0x%02x",diobits);
	switch(diobits){
	case 0x80:
		printk(" das16 or das16/f");
		/* only difference is speed, so not an issue yet */
		return das16_board_das16;
	case 0x00:
		printk(" das16jr or das16/330");
		/* the 330 has ao, 16jr does not */

		/* we can write the low 4 bits without updating DAC */
		if(detect_ao(dev)){
			return das16_board_das16_330;
		}else{
			return das16_board_das16jr;
		}

		break;
	default:
		printk(" unknown board");
		return -1;
	case 0xc0:
		printk(" das1600 or das1400");
		break;
	}

	/* burststatus is available on 1600, 1400 */

	burststatus = inb(dev->iobase + DAS1600_BURST_STATUS);

	if((burststatus & 0xfc)==0x10){
		/* true for 1400, 1600 */
	}

	if((burststatus & DAS1600_CLK)){
		devpriv->clockbase = 100;
	}else{
		devpriv->clockbase = 1000;
	}

	outb(DAS1600_ENABLE_VAL,dev->iobase+DAS1600_ENABLE);

	if(detect_ao(dev)){
		printk("das1600 series\n");
		return das16_board_das1601_12;
	}else{
		printk("das1400 series\n");
		return das16_board_das1401_12;
	}

}


static int das16_recognize(char *name)
{
	int i;

	for(i=0;i<n_das16_boards;i++){
		if(!strcmp(das16_boards[i].name,name))
			return i;
	}

	return -1;
}


/*
 *
 * Options list:
 *   0  I/O base
 *   1  IRQ
 */

static int das16_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret;

	dev->iobase = it->options[0];

	printk("comedi%d: das16:",dev->minor);

	if((ret=alloc_private(dev,sizeof(struct das16_private_struct)))<0)
		return ret;

	dev->board = das16_probe(dev);

	dev->board_ptr = das16_boards + dev->board;
	dev->board_name = thisboard->name;

	if(thisboard->size<0x400){
		printk(" 0x%04x-0x%04x",
			dev->iobase,dev->iobase+thisboard->size);
		if(check_region(dev->iobase,thisboard->size)<0){
			printk(" I/O port conflict\n");
			return -EIO;
		}
	}else{
		printk(" 0x%04x-0x%04x 0x%04x-0x%04x",
			dev->iobase,dev->iobase+0x0f,
			dev->iobase+0x400,dev->iobase+(thisboard->size&0x3ff));
		if(check_region(dev->iobase,0x10)<0 ||
		   check_region(dev->iobase+0x400,thisboard->size&0x3ff)<0){
			printk(" I/O port conflict\n");
			return -EIO;
		}
	}

	dev->n_subdevices = 5;
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	if(thisboard->size<0x400){
		request_region(dev->iobase,thisboard->size,"das16");
	}else{
		request_region(dev->iobase,0x10,"das16");
		request_region(dev->iobase,thisboard->size&0x3ff,"das16");
	}

	s=dev->subdevices+0;
	/* ai */
	if(thisboard->ai){
		s->type = COMEDI_SUBD_AI;
		s->subdev_flags = SDF_READABLE;
		if(devpriv->ai_singleended){
			s->n_chan = 16;
			s->subdev_flags |= SDF_GROUND;	/* XXX ? */
		}else{
			s->n_chan = 8;
			s->subdev_flags |= SDF_DIFF;
		}
		s->maxdata = (1<<thisboard->ai_nbits)-1;
		if(devpriv->ai_unipolar){
			s->range_table = das16_ai_uni_lranges[thisboard->ai_pg];
		}else{
			s->range_table = das16_ai_bip_lranges[thisboard->ai_pg];
		}
		s->insn_read = thisboard->ai;
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	s = dev->subdevices + 1;
	/* ao */
	if(thisboard->ao){
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_WRITEABLE;
		s->n_chan = 2;
		s->maxdata = (1<<thisboard->ao_nbits)-1;
		s->range_table = &range_unknown; /* XXX */
		s->insn_write = thisboard->ao;
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}

	s = dev->subdevices + 2;
	/* di */
	if(thisboard->di){
		s->type = COMEDI_SUBD_DI;
		s->subdev_flags = SDF_READABLE;
		s->n_chan = 4;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_read = thisboard->di;
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}

	s = dev->subdevices + 3;
	/* do */
	if(thisboard->do_){
		s->type = COMEDI_SUBD_DO;
		s->subdev_flags = SDF_WRITEABLE;
		s->n_chan = 4;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_write = thisboard->do_;
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}

	s = dev->subdevices + 4;
	/* 8255 */
	if(thisboard->i8255_offset!=0){
		subdev_8255_init(dev,s,NULL,(void *)(dev->iobase+
			thisboard->i8255_offset));
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}

	outb(DAS16_IRQ(dev->irq),dev->iobase+DAS16_CONTROL);
	outb(0,DAS16_PACER);

	return 0;
}


static int das16_detach(comedi_device *dev)
{

	return 0;
}

COMEDI_INITCLEANUP(driver_das16);

