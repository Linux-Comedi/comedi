/*
    comedi/drivers/das.c
    DAS08 driver

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


#define DAS08_SIZE 16

/*
    cio-das08.pdf

  "das08"

  0	a/d bits 0-3		start 8 bit
  1	a/d bits 4-11		start 12 bit
  2	eoc, ip1-3, irq, mux	op1-4, inte, mux
  3	unused			unused
  4567	8254
  89ab	8255

  requires hard-wiring for async ai

*/

#define DAS08_LSB		0
#define DAS08_MSB		1
#define DAS08_TRIG_12BIT	1
#define DAS08_STATUS		2
#define	  DAS08_EOC			(1<<7)
#define   DAS08_IRQ			(1<<3)
#define   DAS08_IP(x)			(((x)>>4)&0x7)
#define DAS08_CONTROL		2
#define	  DAS08_MUX(x)			((x)<<0)
#define   DAS08_INTE			(1<<3)
#define   DAS08_OP(x)			((x)<<4)

/*
    cio-das08jr.pdf

  "das08/jr-ao"

  0	a/d bits 0-3		unused
  1	a/d bits 4-11		start 12 bit
  2	eoc, mux		mux
  3	di			do
  4	unused			ao0_lsb
  5	unused			ao0_msb
  6	unused			ao1_lsb
  7	unused			ao1_msb

*/

#define DAS08JR_DIO		3
#define DAS08JR_AO_LSB(x)	((x)?6:4)
#define DAS08JR_AO_MSB(x)	((x)?7:5)

/*
    cio-das08_aox.pdf

  "das08-aoh"
  "das08-aol"
  "das08-aom"

  0	a/d bits 0-3		start 8 bit
  1	a/d bits 4-11		start 12 bit
  2	eoc, ip1-3, irq, mux	op1-4, inte, mux
  3	mux, gain status	gain control
  4567	8254
  8	unused			ao0_lsb
  9	unused			ao0_msb
  a	unused			ao1_lsb
  b	unused			ao1_msb
  89ab	
  cdef	8255
*/

#define DAS08AO_GAIN_CONTROL	3
#define DAS08AO_GAIN_STATUS	3

#define DAS08AO_AO_LSB(x)	((x)?0xa:8)
#define DAS08AO_AO_MSB(x)	((x)?0xb:9)
#define DAS08AO_AO_UPDATE	8

/* gainlist same as _pgx_ below */

/*
    cio-das08pgx.pdf

    "das08pgx"

  0	a/d bits 0-3		start 8 bit
  1	a/d bits 4-11		start 12 bit
  2	eoc, ip1-3, irq, mux	op1-4, inte, mux
  3	mux, gain status	gain control
  4567	8254

*/

static int das08_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08jr_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08jr_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08jr_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08ao_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);

static comedi_lrange range_das08_pgl = { 9, {
	BIP_RANGE(10),
	BIP_RANGE(5),
	BIP_RANGE(2.5),
	BIP_RANGE(1.25),
	BIP_RANGE(0.625),
	UNI_RANGE(10),
	UNI_RANGE(5),
	UNI_RANGE(2.5),
	UNI_RANGE(1.25)
}};
static comedi_lrange range_das08_pgh = { 12, {
	BIP_RANGE(10),
	BIP_RANGE(5),
	BIP_RANGE(1),
	BIP_RANGE(0.5),
	BIP_RANGE(0.1),
	BIP_RANGE(0.05),
	BIP_RANGE(0.01),
	BIP_RANGE(0.005),
	UNI_RANGE(10),
	UNI_RANGE(1),
	UNI_RANGE(0.1),
	UNI_RANGE(0.01),
}};
static comedi_lrange range_das08_pgm = { 9, {
	BIP_RANGE(10),
	BIP_RANGE(5),
	BIP_RANGE(0.5),
	BIP_RANGE(0.05),
	BIP_RANGE(0.01),
	UNI_RANGE(10),
	UNI_RANGE(1),
	UNI_RANGE(0.1),
	UNI_RANGE(0.01)
}};

enum { das08_pg_none, das08_pgh, das08_pgl, das08_pgm };

static comedi_lrange *das08_ai_lranges[]={
	&range_bipolar10, /* XXX guess */
	&range_das08_pgh,
	&range_das08_pgl,
	&range_das08_pgm,
};

static int das08_pgh_gainlist[] = { 8, 0, 10, 2, 12, 4, 14, 6, 1, 3, 5, 7 };
static int das08_pgl_gainlist[] = { 8, 0, 2, 4, 6, 1, 3, 5, 7 };
static int das08_pgm_gainlist[] = { 8, 0, 10, 12, 14, 9, 11, 13, 15 };

static int *das08_gainlists[] = {
	NULL,
	das08_pgh_gainlist,
	das08_pgl_gainlist,
	das08_pgm_gainlist,
};

struct das08_board_struct{
	char		*name;
	void		*ai;
	unsigned int	ai_nbits;
	unsigned int	ai_pg;
	void		*ao;
	unsigned int	ao_nbits;
	void		*di;
	void		*do_;
	unsigned int	i8255_offset;
	unsigned int	i8254_offset;
};
static struct das08_board_struct das08_boards[]={
	{
	name:		"das08",		// cio-das08.pdf
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pg_none,
	ao:		NULL,
	ao_nbits:	12,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	i8255_offset:	8,
	i8254_offset:	4,
	},
	{
	name:		"das08-pgm",		// cio-das08pgx.pdf
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgm,
	ao:		NULL,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x04,
	},
	{
	name:		"das08-pgh",		// cio-das08pgx.pdf
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgh,
	ao:		NULL,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x04,
	},
	{
	name:		"das08-pgl",		// cio-das08pgx.pdf
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgl,
	ao:		NULL,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x04,
	},
	{
	name:		"das08-aoh",		// cio-das08_aox.pdf
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgh,
	ao:		das08ao_ao_winsn,	// 8
	ao_nbits:	12,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	i8255_offset:	0x0c,
	i8254_offset:	0x04,
	},
	{
	name:		"das08-aol",		// cio-das08_aox.pdf
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgl,
	ao:		das08ao_ao_winsn,	// 8
	ao_nbits:	12,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	i8255_offset:	0x0c,
	i8254_offset:	0x04,
	},
	{
	name:		"das08-aom",		// cio-das08_aox.pdf
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgm,
	ao:		das08ao_ao_winsn,	// 8
	ao_nbits:	12,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	i8255_offset:	0x0c,
	i8254_offset:	0x04,
	},
	{
	name:		"das08/jr-ao",		// cio-das08-jr-ao.pdf
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pg_none,
	ao:		das08jr_ao_winsn,
	ao_nbits:	12,
	di:		das08jr_di_rbits,
	do_:		das08jr_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0,
	},
	{
	name:		"das08jr-16-ao",	// cio-das08jr-16-ao.pdf
	ai:		das08_ai_rinsn,
	ai_nbits:	16,
	ai_pg:		das08_pg_none,
	ao:		das08jr_ao_winsn,
	ao_nbits:	16,
	di:		das08jr_di_rbits,
	do_:		das08jr_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x04,
	},
#if 0
	{
	name:		"das08/f",
	},
	{
	name:		"das08jr",
	},
	{
	name:		"das08jr/16",
	},
	{
	name:		"das48-pga",		// cio-das48-pga.pdf
	},
	{
	name:		"das08-pga-g2",		// a KM board
	},
#endif
};
static int n_boardtypes = sizeof(das08_boards)/sizeof(das08_boards[0]);


struct das08_private_struct{
	unsigned int	do_bits;
	unsigned int	*pg_gainlist;
};

#define devpriv ((struct das08_private_struct *)dev->private)
#define thisboard ((struct das08_board_struct *)dev->board)

#define TIMEOUT 1000

static int das08_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i,n;
	int chan;
	int range;
	int lsb,msb;

	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);

	/* clear crap */
	inb(dev->iobase+DAS08_LSB);
	inb(dev->iobase+DAS08_MSB);

	/* set multiplexer */
	outb_p(DAS08_MUX(chan) | devpriv->do_bits,dev->iobase+DAS08_CONTROL);
	
	if(thisboard->ai_pg!=das08_pg_none){
		/* set gain/range */
		range = CR_RANGE(insn->chanspec);
		outb_p(devpriv->pg_gainlist[range],dev->iobase+DAS08AO_GAIN_CONTROL);
	}

	/* How long should we wait for MUX to settle? */
	//udelay(5);

	for(n=0;n<insn->n;n++){
		/* trigger conversion */
		outb_p(0,dev->iobase+DAS08_TRIG_12BIT);

		for(i=0;i<TIMEOUT;i++){
			if(!(inb(DAS08_STATUS)&DAS08_EOC))
				break;
		}
		if(i==TIMEOUT){
			rt_printk("das08: timeout\n");
			return -ETIME;
		}
		msb = inb(dev->iobase + DAS08_MSB);
		lsb = inb(dev->iobase + DAS08_LSB);
		if(thisboard->ai_nbits==12){
			insn->data[n] = (lsb>>4) | (msb << 4);
		}else{
			insn->data[n] = lsb | (msb << 8);
		}
	}

	return n;
}

static int das08_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	insn->data[0]=DAS08_IP(inb(dev->iobase+DAS08_STATUS));

	return 1;
}

static int das08_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	/* XXX race with ai */

	devpriv->do_bits = DAS08_OP(insn->data[0]);

	outb(devpriv->do_bits,dev->iobase+DAS08_CONTROL);

	return 1;
}

static int das08jr_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	insn->data[0]=inb(dev->iobase+DAS08JR_DIO);

	return 1;
}

static int das08jr_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	outb(insn->data[0],dev->iobase+DAS08JR_DIO);

	return 1;
}

static int das08jr_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n;
	int lsb,msb;
	int chan;

	lsb=insn->data[0]&0xff;
	msb=(insn->data[0]>>8)&0xf;

	chan=CR_CHAN(insn->chanspec);

	for(n=0;n<insn->n;n++){
#if 0
		outb(lsb,dev->iobase+devpriv->ao_offset_lsb[chan]);
		outb(msb,dev->iobase+devpriv->ao_offset_msb[chan]);
#else
		outb(lsb,dev->iobase+DAS08JR_AO_LSB(chan));
		outb(msb,dev->iobase+DAS08JR_AO_MSB(chan));
#endif

		/* load DACs */
		inb(dev->iobase+DAS08JR_DIO);

		/* XXX */
		break;
	}

	return n;
}

/*
 *
 * The -aox boards have the DACs at a different offset and use
 * a different method to force an update.
 *
 */
static int das08ao_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n;
	int lsb,msb;
	int chan;

	lsb=insn->data[0]&0xff;
	msb=(insn->data[0]>>8)&0xf;

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
		inb(dev->iobase+DAS08AO_AO_UPDATE);

		/* XXX */
		break;
	}

	return n;
}

static int das08_attach(comedi_device *dev,comedi_devconfig *it);
static int das08_detach(comedi_device *dev);
static int das08_recognize(char *name);

comedi_driver driver_das08={
	driver_name:	"das08",
	module:		THIS_MODULE,
	attach:		das08_attach,
	detach:		das08_detach,
	recognize:	das08_recognize,
};

static int das08_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret;

	dev->iobase=it->options[0];
	printk("comedi%d: das08: 0x%04x",dev->minor,dev->iobase);
	if(check_region(dev->iobase,DAS08_SIZE)<0){
		printk(" I/O port conflict\n");
		return -EIO;
	}

	dev->board_ptr = das08_boards+dev->board;
	dev->board_name = thisboard->name;
	dev->iosize = DAS08_SIZE;

	dev->n_subdevices=5;
	if((ret=alloc_subdevices(dev))<0)
		return ret;
	if((ret=alloc_private(dev,sizeof(struct das08_private_struct)))<0)
		return ret;

	request_region(dev->iobase,DAS08_SIZE,"das08");

	s=dev->subdevices+0;
	/* ai */
	if(thisboard->ai){
		s->type=COMEDI_SUBD_AI;
		s->subdev_flags = SDF_READABLE;
		s->n_chan = 8;
		s->maxdata = (1<<thisboard->ai_nbits)-1;
		s->range_table = das08_ai_lranges[thisboard->ai_pg];
		s->insn_read = thisboard->ai;
		devpriv->pg_gainlist = das08_gainlists[thisboard->ai_pg];
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	s=dev->subdevices+1;
	/* ao */
	if(thisboard->ao){
		s->type=COMEDI_SUBD_AO;
		s->subdev_flags = SDF_WRITEABLE;
		s->n_chan = 2;
		s->maxdata = (1<<thisboard->ao_nbits)-1;
		s->range_table = &range_unknown; /* XXX */
		s->insn_write = thisboard->ao;
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	s=dev->subdevices+2;
	/* di */
	if(thisboard->di){
		s->type=COMEDI_SUBD_DI;
		s->subdev_flags = SDF_READABLE;
		s->n_chan = (thisboard->di==das08_di_rbits)?3:8;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_read = thisboard->di; /* XXX */
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	s=dev->subdevices+3;
	/* do */
	if(thisboard->do_){
		s->type=COMEDI_SUBD_DO;
		s->subdev_flags = SDF_WRITEABLE;
		s->n_chan = (thisboard->do_==das08_do_wbits)?4:8;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_write = thisboard->do_; /* XXX */
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	s=dev->subdevices+4;
	/* 8255 */
	if(thisboard->i8255_offset!=0){
		subdev_8255_init(dev,s,NULL,(void *)(dev->iobase+
			thisboard->i8255_offset));
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	return 0;
}

static int das08_detach(comedi_device *dev)
{
	printk(KERN_INFO "comedi%d: das08: remove\n",dev->minor);

	release_region(dev->iobase,DAS08_SIZE);

	return 0;
}

static int das08_recognize(char *name)
{
	int i;

	for(i=0;i<n_boardtypes;i++){
		if(!strcmp(das08_boards[i].name,name))return i;
	}

	return -1;
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

