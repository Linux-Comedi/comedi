/*
    comedi/drivers/das16.c
    DAS16 driver

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@stm.lbl.gov>
	Copyright (C) 2000 Chris R. Baugher <baugher@enteract.com>

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

#undef DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/comedidev.h>
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
#define   DAS16_MUXBIT			(1<<5)
#define   DAS16_INT			(1<<4)
#define DAS16_CONTROL		9
#define   DAS16_INTE			(1<<7)
#define   DAS16_IRQ(x)			((x)<<4)
#define   DAS16_DMA			(1<<2)
#define   DAS16_CTR2		0x03
#define   DAS16_IP0			0x02
#define   DAS16_SOFT		0x00
#define DAS16_PACER		0x0A
#define   DAS16_CTR0			(1<<1)
#define   DAS16_TRIG0			(1<<0)
#define DAS16_GAIN		0x0B
#define DAS16_CNTR0_DATA		0x0C
#define DAS16_CNTR1_DATA		0x0D
#define DAS16_CNTR2_DATA		0x0E
#define DAS16_CNTR_CONTROL	0x0F
#define   DAS16_TERM_CNT	0x00
#define   DAS16_ONE_SHOT	0x02
#define   DAS16_RATE_GEN	0x04
#define   DAS16_CNTR_LSB_MSB	0x30
#define   DAS16_CNTR0		0x00
#define   DAS16_CNTR1		0x40
#define   DAS16_CNTR2		0x80

#define DAS1600_CONV		0x404
#define   DAS1600_CONV_DISABLE	0x40
#define DAS1600_BURST		0x405
#define   DAS1600_BURST_VAL		0x40
#define DAS1600_ENABLE		0x406
#define   DAS1600_ENABLE_VAL		0x40
#define DAS1600_STATUS_B	0x407
#define   DAS1600_BME		0x40
#define   DAS1600_ME		0x20
#define   DAS1600_CD			0x10
#define   DAS1600_WS			0x02
#define   DAS1600_CLK_10MHZ		0x01

#define DAS16_SLOWEST_TIMER		0xffffffff
#define DAS16_FASTEST_TIMER		10000

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

static int das16_cmd_test(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd);
static int das16_cmd_exec(comedi_device *dev,comedi_subdevice *s);
static int das16_cancel(comedi_device *dev, comedi_subdevice *s);

static void das16_reset(comedi_device *dev);
static void das16_interrupt(int irq, void *d, struct pt_regs *regs);

static float das16_set_pacer(comedi_device *dev, unsigned int ns);
static int das1600_mode_detect(comedi_device *dev);
#ifdef DEBUG
static void reg_dump(comedi_device *dev);
#endif

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
comedi_driver driver_das16={
	driver_name:	"das16",
	module:		THIS_MODULE,
	attach:		das16_attach,
	detach:		das16_detach,
/* Should be able to autodetect boards */
#if 0
	board_name:	das16_boards,
	num_names:	n_das16_boards,
	offset:		sizeof(das16_boards[0]),
#endif
};


#define DAS16_TIMEOUT 1000


struct das16_private_struct {
	unsigned int	ai_unipolar;
	unsigned int	ai_singleended;
	unsigned int	clockbase;
	unsigned char	control_state;
	volatile unsigned char	cmd_go;
	unsigned int	adc_count;
};
#define devpriv ((struct das16_private_struct *)(dev->private))
#define thisboard ((struct das16_board_struct *)(dev->board_ptr))

static int das16_cmd_test(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd)
{
	int err=0, tmp;
	
	/* make sure triggers are valid */
	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW|TRIG_TIMER;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	/* XXX This must be TRIG_FOLLOW until I figure out a way to *
	 * time the individual conversions.                         */
	tmp=cmd->convert_src;
	//cmd->convert_src &= TRIG_TIMER;
	cmd->convert_src &= TRIG_FOLLOW;
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
	if(cmd->scan_begin_src!=TRIG_FOLLOW &&
	   cmd->scan_begin_src!=TRIG_EXT &&
	   cmd->scan_begin_src!=TRIG_TIMER)err++;
	if(cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_NONE)err++;

	if(err)return 2;
	
	/* step 3: make sure arguments are trivially compatible */
	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}

#if 0
	if(cmd->scan_begin_src==TRIG_FOLLOW){
		/* internal trigger */
		if(cmd->scan_begin_arg!=0){
			cmd->scan_begin_arg=0;
			err++;
		}
	}else{
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		if(cmd->scan_begin_arg!=0){
			cmd->scan_begin_arg=0;
			err++;
		}
	}
#endif
	
	if(cmd->scan_begin_arg<DAS16_FASTEST_TIMER){
		cmd->scan_begin_arg=DAS16_FASTEST_TIMER;
		err++;
	}

	if(cmd->scan_begin_arg>DAS16_SLOWEST_TIMER){
		cmd->scan_begin_arg=DAS16_SLOWEST_TIMER;
		err++;
	}

	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	
	if(cmd->stop_src==TRIG_COUNT){
		/* any count is allowed */
	}else{
		/* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;
	
	return 0;
}

static int das16_cmd_exec(comedi_device *dev,comedi_subdevice *s)
{
	comedi_cmd *cmd = &s->async->cmd;
	char byte;
	float freq;
	
	devpriv->adc_count = cmd->stop_arg*cmd->chanlist_len;
	devpriv->cmd_go = 1;
	
	/* check if we are scanning multiple channels */
	if(cmd->chanlist_len < 2) {	/* one channel */
		byte = CR_CHAN(cmd->chanlist[0]);
		outb(byte, dev->iobase+DAS16_MUX);
	} else {	/* multiple channels */
		byte = CR_CHAN(cmd->chanlist[0]) +
			(CR_CHAN(cmd->chanlist[cmd->chanlist_len-1]<<4));
		outb(byte, dev->iobase+DAS16_MUX);
	}

	/* enable pacer clocked conversions */
	devpriv->control_state |= DAS16_CTR2;
	outb(devpriv->control_state,dev->iobase+DAS16_CONTROL);
	
	/* set counter mode and counts */
	freq = das16_set_pacer(dev, cmd->scan_begin_arg);

	printk("pacer frequency: %d\n", (int)freq);
	/* enable counters */
	outb(0x00,dev->iobase+DAS16_PACER);
	/* clear interrupt bit */
	outb(0x00,dev->iobase+DAS16_STATUS);
	/* enable interrupts */
	devpriv->control_state |= DAS16_INTE;
	outb(devpriv->control_state,dev->iobase+DAS16_CONTROL);

	return 0;
}

static int das16_cancel(comedi_device *dev, comedi_subdevice *s)
{
	devpriv->cmd_go = 0;
	
	return 0;
}

static void das16_reset(comedi_device *dev)
{
	outb(0,dev->iobase+DAS16_STATUS);
	outb(0,dev->iobase+DAS16_CONTROL);
	outb(0,dev->iobase+DAS16_PACER);
	outb(0,dev->iobase+DAS16_CNTR_CONTROL);
}

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

		for(i=0;i<DAS16_TIMEOUT;i++){
			if(!(inb(dev->iobase + DAS16_STATUS) & DAS16_EOC))
				break;
		}
		if(i==DAS16_TIMEOUT){
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
	int i;
	int lsb,msb;
	int chan;

	chan=CR_CHAN(insn->chanspec);

	for(i=0;i<insn->n;i++){
		if(thisboard->ao_nbits==12){
			lsb=(data[i]<<4)&0xff;
			msb=(data[i]>>4)&0xff;
		}else{
			lsb=data[i]&0xff;
			msb=(data[i]>>8)&0xff;
		}

#if 0
		outb(lsb,dev->iobase+devpriv->ao_offset_lsb[chan]);
		outb(msb,dev->iobase+devpriv->ao_offset_msb[chan]);
#else
		outb(lsb,dev->iobase+DAS16_AO_LSB(chan));
		outb(msb,dev->iobase+DAS16_AO_MSB(chan));
#endif
	}

	return i;
}


static void das16_interrupt(int irq, void *d, struct pt_regs *regs)
{
	static int i, j;
	static char lsb, msb;
	
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices;
	
	if(devpriv->cmd_go)	{	/* Are we supposed to be here? */
		/* check for missed conversions */
	//	if( CR_CHAN(s->async->cur_chanlist[0]) != inb() )
	//		printk("Channel MUX sync error\n");
		
		for(i=0;i<s->async->cur_chanlist_len;++i) {
			
			if(i) {		/* performing multiple conversions */			
				outb(0x00, dev->iobase+DAS16_TRIG);	/* force a conversion */
			
				for(j=0;j<DAS16_TIMEOUT;++j)	/* wait until it's ready */
					if( !(inb(dev->iobase+DAS16_STATUS)&DAS16_EOC) )
						break;
				if(j==DAS16_TIMEOUT)
					printk("das16 EOC timeout!!");
			}
			
			lsb = inb(dev->iobase+DAS16_AI_LSB);
			msb = inb(dev->iobase+DAS16_AI_MSB);
			
			/* all this just to put data into a buffer! */
			*(sampl_t *)(s->async->data+s->async->buf_int_ptr) =
				(lsb>>4) + (msb<<4);

			s->async->buf_int_ptr += sizeof(sampl_t);
			s->async->buf_int_count += sizeof(sampl_t);

			if(s->async->buf_int_ptr >= s->async->data_len) {	/* buffer rollover */
				s->async->buf_int_ptr = 0;
				comedi_eobuf(dev, s);
			}
		
			if( !(s->async->cmd.flags&TRIG_WAKE_EOS) )
				comedi_bufcheck(dev, s);	/* wakeup user's read() */
		
			if(--devpriv->adc_count <= 0) {		/* end of acquisition */
				printk("End of acquisition\n");
				devpriv->cmd_go = 0;
				devpriv->control_state &= ~DAS16_INTE;
				outb(devpriv->control_state, dev->iobase+DAS16_CONTROL);
				comedi_done(dev, s);
				break;
			}

		}	/* end of scan loop */
		
		comedi_eos(dev, s);
		
	} else {
		printk("Acquisition canceled OR Stray interrupt!\n");
		devpriv->control_state &= ~DAS16_INTE;
		outb(devpriv->control_state, dev->iobase+DAS16_CONTROL);
		comedi_error_done(dev, s);
	}
	
	/* clear interrupt */
	outb(0x00,dev->iobase+DAS16_STATUS);
}

/* This function takes a time in nanoseconds and sets the     *
 * 2 pacer clocks to the closest frequency possible. It also  *
 * returns the actual sampling rate.                          */
static float das16_set_pacer(comedi_device *dev, unsigned int ns)
{
	short unsigned ctr1, ctr2;
	unsigned int mask;
	long product, error;

	/* divide 10Mhz by frequency */
	product = ( devpriv->clockbase/(1000000000.0/ns) ) + 0.5;
	/* Now the job is to find two 16 bit numbers, that when multiplied
	   together are approximately equal to product.  Start by setting
	   one of them, ctr1 to 2 (minimum settable value) and increment until
	   the error is minimized and ctr2 is less than 32768.

	   NOTE: In Mode 2, a value of 1 is illegal! Therefore, crt1 and crt2
	   can never be 1.

	 */

	printk("product: %ld\n", product);
	ctr1 = product / 32768;
	if (ctr1 < 2)
		ctr1 = 2;
	ctr2 = product / ctr1;
	error = abs(product - (long) ctr2 * (long) ctr1);

	while (error && ctr1 < 32768 && ctr2 > 1) {
		ctr1++;
		ctr2 = product / ctr1;
		error = abs(product - (long) ctr2 * (long) ctr1);
	}

	/* the frequency is prime, add 1 to it */
	if (error) {
		product++;
		ctr1 = product / 32768;
		if (ctr1 < 2)
			ctr1 = 2;
		ctr2 = product / ctr1;
		error = abs(product - (long) ctr2 * (long) ctr1);

		while (error && ctr1 < 32768 && ctr2 > 1) {
			ctr1++;
			ctr2 = product / ctr1;
			error = abs(product - (long) ctr2 * (long) ctr1);
		}
	}
	/* we can't have ctr2 equal to 1, or system hangs */
	if (ctr2 == 1) {
		ctr2++;
		ctr1 /= 2;
	}
	printk("ctr1: %d, ctr2: %d\n", ctr1, ctr2);
	/* Write the values of ctr1 and ctr2 into counters 1 and 2 */
	mask = DAS16_CNTR2 | DAS16_RATE_GEN | DAS16_CNTR_LSB_MSB;
	outb(mask, dev->iobase+DAS16_CNTR_CONTROL);

	outb(ctr2 & 0xFF , dev->iobase + DAS16_CNTR2_DATA);
	outb(ctr2 >> 8, dev->iobase + DAS16_CNTR2_DATA);

	mask = DAS16_CNTR1 | DAS16_RATE_GEN | DAS16_CNTR_LSB_MSB;
	outb(mask, dev->iobase+DAS16_CNTR_CONTROL);

	outb(ctr1 & 0xFF, dev->iobase + DAS16_CNTR1_DATA);
	outb(ctr1 >> 8, dev->iobase + DAS16_CNTR1_DATA);
	
//	printk("SetPacerFreq: Pacer Register set to %#x\n", BoardData.pacerReg);
	
	return devpriv->clockbase / ((long) ctr1 * (long) ctr2) + 0.5;
}

#ifdef DEBUG
static void reg_dump(comedi_device *dev)
{
	printk("********DAS1600 REGISTER DUMP********\n");
	printk("DAS16_MUX: %x\n", inb(dev->iobase+DAS16_MUX) );
	printk("DAS16_DIO: %x\n", inb(dev->iobase+DAS16_DIO) );
	printk("DAS16_STATUS: %x\n", inb(dev->iobase+DAS16_STATUS) );
	printk("DAS16_CONTROL: %x\n", inb(dev->iobase+DAS16_CONTROL) );
	printk("DAS16_PACER: %x\n", inb(dev->iobase+DAS16_PACER) );
	printk("DAS16_GAIN: %x\n", inb(dev->iobase+DAS16_GAIN) );
	printk("DAS16_CNTR_CONTROL: %x\n", inb(dev->iobase+DAS16_CNTR_CONTROL) );
	printk("DAS1600_CONV: %x\n", inb(dev->iobase+DAS1600_CONV) );
	printk("DAS1600_BURST: %x\n", inb(dev->iobase+DAS1600_BURST) );
	printk("DAS1600_ENABLE: %x\n", inb(dev->iobase+DAS1600_ENABLE) );
	printk("DAS1600_STATUS_B: %x\n", inb(dev->iobase+DAS1600_STATUS_B) );	
}
#endif

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

static int das16_probe(comedi_device *dev, comedi_devconfig *it)
{
	int status;
	int diobits;

	/* status is available on all boards */

	status = inb(dev->iobase + DAS16_STATUS);

	if((status & DAS16_UB)){
		devpriv->ai_unipolar = 1;
	}else{
		devpriv->ai_unipolar = 0;
	}

	if((status & DAS16_MUXBIT)){
		devpriv->ai_singleended = 1;
	}else{
		devpriv->ai_singleended = 0;
	}

	/* diobits indicates boards */

	diobits = inb(dev->iobase + DAS16_DIO) & 0xf0;

	printk(" diobits 0x%02x",diobits);

	switch(diobits){
	case 0x80:
		printk(" das16 or das16/f");
		/* only difference is speed, so not an issue yet */
		//devpriv->clockbase = it->options[4];
		devpriv->clockbase = 10000000;
		return das16_board_das16;
	case 0x00:
		printk(" das16jr or das16/330");
		//devpriv->clockbase = it->options[4];
		devpriv->clockbase = 10000000;
		/* the 330 has ao, 16jr does not */

		/* we can write the low 4 bits without updating DAC */
		if(detect_ao(dev)){
			return das16_board_das16_330;
		}else{
			return das16_board_das16jr;
		}
	case 0xC0:
		printk(" das1600\n");
		das1600_mode_detect(dev);
		return das16_board_das1601_12;
	case 0xE0:
		printk(" das1400\n");
		das1600_mode_detect(dev);
		return das16_board_das1401_12;
	default:
		printk(" unknown board\n");
		return -1;
	}
}

static int das1600_mode_detect(comedi_device *dev)
{
	int status=0;
	
	status = inb(dev->iobase + DAS1600_STATUS_B);
	
	if(status & DAS1600_CLK_10MHZ) {
		devpriv->clockbase = 10000000;
		printk(" 10MHz pacer clock\n");
	} else {
		devpriv->clockbase = 1000000;
		printk(" 1MHz pacer clock\n");
	}

	/* enable das1400/1600 mode */
//	outb(DAS1600_ENABLE_VAL, dev->iobase+DAS1600_ENABLE);
#ifdef DETECT	
	/* burststatus is available on 1600, 1400 */
	if((burststatus & 0xfc)==0x10){
		/* true for 1400, 1600 */
	}

	if((burststatus & DAS1600_CLK)){
		devpriv->clockbase = 100;
	}else{
		devpriv->clockbase = 1000;
	}

	if(detect_ao(dev)){
		printk("das1600 series\n");
		return das16_board_das1601_12;
	}else{
		printk("das1400 series\n");
		return das16_board_das1401_12;
	}
#endif

	return 0;
}


/*
 *
 * Options list:
 *   0  I/O base
 *   1  IRQ
 *   2  DMA
 *   3  Clock speed
 */

static int das16_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret, irq;
	int iobase;

	iobase = it->options[0];

	printk("comedi%d: das16:",dev->minor);

	if((ret=alloc_private(dev,sizeof(struct das16_private_struct)))<0)
		return ret;

	dev->board_ptr = das16_boards + das16_probe(dev, it);
	dev->board_name = thisboard->name;

	if(thisboard->size<0x400){
		printk(" 0x%04x-0x%04x\n", iobase, iobase+thisboard->size);
		if(check_region(iobase,thisboard->size)<0){
			printk(" I/O port conflict\n");
			return -EIO;
		}
	}else{
		printk(" 0x%04x-0x%04x 0x%04x-0x%04x\n",
			   iobase,iobase+0x0f,
			   iobase+0x400,iobase+0x400+(thisboard->size&0x3ff));
		if(check_region(iobase,0x10) < 0) {
			printk(" I/O port conflict:  0x%04x-0x%04x\n",
				   iobase,iobase+0x0f);
			return -EIO;
		}
		if(check_region(iobase+0x400,thisboard->size&0x3ff)<0){
			printk(" I/O port conflict:  0x%04x-0x%04x\n",
				   iobase+0x400,
				   iobase+0x400+(thisboard->size&0x3ff));
			return -EIO;
		}
	}

	dev->n_subdevices = 5;
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	if(thisboard->size<0x400){
		request_region(iobase,thisboard->size,"das16");
	}else{
		request_region(iobase,0x10,"das16");
		request_region(iobase+0x400,thisboard->size&0x3ff,"das16");
	}

	dev->iobase = iobase;

	/* now for the irq */
	irq=it->options[1];
	if(irq>0){
		if((ret=comedi_request_irq(irq,das16_interrupt,0,"das16",dev))<0)
			return ret;
		dev->irq=irq;
		printk(" ( irq = %d )\n",irq);
	} else if(irq == 0){
		printk(" ( no irq )\n");
	}
	
	s=dev->subdevices+0;
	dev->read_subdev=s;
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
		s->len_chanlist = 16;
		s->maxdata = (1<<thisboard->ai_nbits)-1;
		if(devpriv->ai_unipolar){
			s->range_table = das16_ai_uni_lranges[thisboard->ai_pg];
		}else{
			s->range_table = das16_ai_bip_lranges[thisboard->ai_pg];
		}
		s->insn_read = thisboard->ai;
		s->do_cmdtest = das16_cmd_test;
		s->do_cmd = das16_cmd_exec;
		s->cancel = das16_cancel;
		s->async->cmd.flags |= TRIG_WAKE_EOS;
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

	das16_reset(dev);
	/* set the interrupt level,enable pacer clock */
	devpriv->control_state = DAS16_IRQ(dev->irq);
	outb(devpriv->control_state,dev->iobase+DAS16_CONTROL);

	return 0;
}


static int das16_detach(comedi_device *dev)
{
	printk("comedi%d: das16: remove\n", dev->minor);
	
	das16_reset(dev);
	
	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices+4);

	if(dev->irq)
		free_irq(dev->irq, dev);
	
	if(thisboard->size<0x400){
		release_region(dev->iobase,thisboard->size);
	}else{
		release_region(dev->iobase,0x10);
		release_region(dev->iobase+0x400,thisboard->size&0x3ff);
	}
	
	return 0;
}

COMEDI_INITCLEANUP(driver_das16);

