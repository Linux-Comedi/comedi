/*
    comedi/drivers/das16.c
    DAS16 driver

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@stm.lbl.gov>
    Copyright (C) 2000 Chris R. Baugher <baugher@enteract.com>
    Copyright (C) 2001 Frank Mori Hess <fmhess@uiuc.edu>

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

************************************************************************

Testing and debugging help provided by Daniel Koch.

Options:
	[0] - base io address
	[1] - irq (optional)
	[2] - dma (optional)
	[3] - master clock speed in MHz (1 or 10, ignored if board can
		probe clock speed, defaults to 1 otherwise)

Both an irq line and dma channel are required for timed or externally
triggered conversions.

Keithley Manuals:
	2309.PDF (das16)
	4923.PDF (das1200, 1400, 1600)

Computer boards manuals also available from their website www.measurementcomputing.com

*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/comedidev.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/malloc.h>
#include <linux/delay.h>
#include <asm/dma.h>
#include "8253.h"
#include "8255.h"

#undef DEBUG

#define DAS16_SIZE 20	// number of ioports
#define DAS16_DMA_SIZE 0xff00 // size in bytes of allocated dma buffer

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

static const int sample_size = 2;	// size in bytes of a sample from board

#define DAS16_TRIG		0
#define DAS16_AI_LSB		0
#define DAS16_AI_MSB		1
#define DAS16_MUX		2
#define DAS16_DIO		3
#define DAS16_AO_LSB(x)	((x)?6:4)
#define DAS16_AO_MSB(x)	((x)?7:5)
#define DAS16_STATUS		8
#define   BUSY			(1<<7)
#define   UNIPOLAR			(1<<6)
#define   DAS16_MUXBIT			(1<<5)
#define   DAS16_INT			(1<<4)
#define DAS16_CONTROL		9
#define   DAS16_INTE			(1<<7)
#define   DAS16_IRQ(x)			(((x) & 0x7) << 4)
#define   DMA_ENABLE			(1<<2)
#define   INT_PACER		0x03
#define   EXT_PACER			0x02
#define   DAS16_SOFT		0x00
#define DAS16_PACER		0x0A
#define   DAS16_CTR0			(1<<1)
#define   DAS16_TRIG0			(1<<0)
#define   BURST_LEN_BITS(x)			(((x) & 0xf) << 4)
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
#define   DAS1600_CONV_DISABLE		0x40
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
	&range_unknown,
	&range_das16jr,
	&range_das1x01_unip,
	&range_das1x02_unip,
};
static comedi_lrange *das16_ai_bip_lranges[]={
	&range_unknown,
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

static unsigned int das16_set_pacer(comedi_device *dev, unsigned int ns, int flags);
static int das1600_mode_detect(comedi_device *dev);
static unsigned int das16_suggest_transfer_size(comedi_cmd cmd);

#ifdef DEBUG
static void reg_dump(comedi_device *dev);
#endif

typedef struct das16_board_struct{
	char		*name;
	void		*ai;
	unsigned int	ai_nbits;
	unsigned int	ai_speed;	// max conversion speed in nanosec
	unsigned int	ai_pg;
	void		*ao;
	unsigned int	ao_nbits;
	void		*di;
	void		*do_;

	unsigned int    i8255_offset;
	unsigned int    i8254_offset;

	unsigned int	size;
	unsigned int id;
} das16_board;

static struct das16_board_struct das16_boards[]={
	{
	name:		"das-16",
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	15000,
	ai_pg:		das16_pg_none,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x10,
	i8254_offset:	0x0c,
	size:		0x14,
	id:	0x00,
	},
	{
	name:		"das-16g",
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	15000,
	ai_pg:		das16_pg_none,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x10,
	i8254_offset:	0x0c,
	size:		0x14,
	id:	0x00,
	},
	{
	name:		"das-16f",
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	8500,
	ai_pg:		das16_pg_none,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x10,
	i8254_offset:	0x0c,
	size:		0x14,
	id:	0x00,
	},
	{
	name:		"cio-das16",	// cio-das16.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	20000,
	ai_pg:		das16_pg_none,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x10,
	i8254_offset:	0x0c,
	size:		0x14,
	id:	0x80,
	},
	{
	name:		"cio-das16/f",	// das16.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	10000,
	ai_pg:		das16_pg_none,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x10,
	i8254_offset:	0x0c,
	size:		0x14,
	id:	0x80,
	},
	{
	name:		"cio-das16/jr",	// cio-das16jr.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	7692,
	ai_pg:		das16_pg_16jr,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x10,
	id:	0x00,
	},
	{
	name:		"pc104-das16jr",	// pc104-das16jr_xx.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	6667,
	ai_pg:		das16_pg_16jr,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x10,
	id:	0x00,
	},
	{
	name:		"pc104-das16jr/16",	// pc104-das16jr_xx.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	16,
	ai_speed:	10000,
	ai_pg:		das16_pg_16jr,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x10,
	id:	0x00,
	},
	{
	name:		"das-1201",	// 4924.pdf (keithley user's manual)
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	20000,
	ai_pg:		das16_pg_none,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0x20,
	},
	{
	name:		"das-1202",	// 4924.pdf (keithley user's manual)
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	10000,
	ai_pg:		das16_pg_none,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0x20,
	},
	{
	name:		"das-1401",	// 4922.pdf (keithley user's manual)
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	10000,
	ai_pg:		das16_pg_1601,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0xc0
	},
	{
	name:		"das-1402",	// 4922.pdf (keithley user's manual)
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	10000,
	ai_pg:		das16_pg_1602,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0xc0
	},
	{
	name:		"das-1601",
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	10000,
	ai_pg:		das16_pg_1601,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0xc0
	},
	{
	name:		"das-1602",
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	10000,
	ai_pg:		das16_pg_1602,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0xc0
	},
	{
	name:		"cio-das1401/12",	// cio-das1400_series.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	6250,
	ai_pg:		das16_pg_1601,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0xc0
	},
	{
	name:		"cio-das1402/12",	// cio-das1400_series.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	6250,
	ai_pg:		das16_pg_1602,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0xc0
	},
	{
	name:		"cio-das1402/16",	// cio-das1400_series.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	16,
	ai_speed:	10000,
	ai_pg:		das16_pg_1602,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0xc0
	},
	{
	name:		"cio-das1601/12",	// cio-das160x-1x.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	6250,
	ai_pg:		das16_pg_1601,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x400,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0xc0
	},
	{
	name:		"cio-das1602/12",	// cio-das160x-1x.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	10000,
	ai_pg:		das16_pg_1602,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x400,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0xc0
	},
	{
	name:		"cio-das1602/16",	// cio-das160x-1x.pdf
	ai:		das16_ai_rinsn,
	ai_nbits:	16,
	ai_speed:	10000,
	ai_pg:		das16_pg_1602,
	ao:		das16_ao_winsn,
	ao_nbits:	12,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0x400,
	i8254_offset:	0x0c,
	size:		0x408,
	id:	0xc0
	},
	{
	name:		"cio-das16/330",	// ?
	ai:		das16_ai_rinsn,
	ai_nbits:	12,
	ai_speed:	3030,
	ai_pg:		das16_pg_16jr,
	ao:		NULL,
	di:		das16_di_rbits,
	do_:		das16_do_wbits,
	i8255_offset:	0,
	i8254_offset:	0x0c,
	size:		0x14,
	id:	0x00
	},
#if 0
	{
	name:		"das16/330i",	// ?
	},
	{
	name:		"das16/jr/ctr5", // ?
	},
	{
	name:		"cio-das16/m1/16",	// cio-das16_m1_16.pdf, this board is a bit quirky, no dma
	},
	{
#endif
};
#define n_das16_boards ((sizeof(das16_boards))/(sizeof(das16_board)))

static int das16_attach(comedi_device *dev,comedi_devconfig *it);
static int das16_detach(comedi_device *dev);
comedi_driver driver_das16={
	driver_name:	"das16",
	module:		THIS_MODULE,
	attach:		das16_attach,
	detach:		das16_detach,
	board_name:	das16_boards,
	num_names:	n_das16_boards,
	offset:		sizeof(das16_boards[0]),
};


#define DAS16_TIMEOUT 1000


struct das16_private_struct {
	unsigned int	ai_unipolar;	// unipolar flag
	unsigned int	ai_singleended;	// single ended flag
	unsigned int	clockbase;	// master clock speed in ns
	unsigned int	control_state;	// dma, interrupt and trigger control bits
	unsigned int	adc_count;	// number of samples remaining
	unsigned int	do_bits;	// digital output bits
	unsigned int divisor1;	// divisor dividing master clock to get conversion frequency
	unsigned int divisor2;	// divisor dividing master clock to get conversion frequency
	unsigned int dma_chan;	// dma channel
	sampl_t *dma_buffer;
	unsigned int dma_transfer_size;	// number of bytes transfered per dma shot
};
#define devpriv ((struct das16_private_struct *)(dev->private))
#define thisboard ((struct das16_board_struct *)(dev->board_ptr))

static int das16_cmd_test(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd)
{
	int err=0, tmp;
	int gain, start_chan, i;
	int mask;

	/* make sure triggers are valid */
	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	mask = TRIG_FOLLOW;
	// if board supports burst mode
	if(thisboard->size > 0x400)
		mask |= TRIG_TIMER | TRIG_EXT;
	cmd->scan_begin_src &= mask;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	mask = TRIG_TIMER | TRIG_EXT;
	// if board supports burst mode
	if(thisboard->size > 0x400)
		mask |= TRIG_NOW;
	cmd->convert_src &= mask;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */
	if(cmd->scan_begin_src != TRIG_TIMER &&
		cmd->scan_begin_src != TRIG_EXT &&
		cmd->scan_begin_src != TRIG_FOLLOW) err++;
	if(cmd->convert_src != TRIG_TIMER &&
		cmd->convert_src != TRIG_EXT &&
		cmd->convert_src != TRIG_NOW) err++;
	if(cmd->stop_src != TRIG_NONE &&
		cmd->stop_src != TRIG_COUNT) err++;

	// make sure scan_begin_src and convert_src dont conflict
	if(cmd->scan_begin_src == TRIG_FOLLOW &&
		cmd->convert_src == TRIG_NOW) err++;
	if(cmd->scan_begin_src != TRIG_FOLLOW &&
		cmd->convert_src != TRIG_NOW) err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */
	if(cmd->start_arg!=0)
	{
		cmd->start_arg=0;
		err++;
	}

	if(cmd->scan_begin_src==TRIG_FOLLOW)
	{
		/* internal trigger */
		if(cmd->scan_begin_arg!=0)
		{
			cmd->scan_begin_arg=0;
			err++;
		}
	}

	if(cmd->scan_end_arg != cmd->chanlist_len)
	{
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}

	// check against maximum frequency
	if(cmd->scan_begin_src == TRIG_TIMER)
	{
		if(cmd->scan_begin_arg < thisboard->ai_speed * cmd->chanlist_len)
		{
			cmd->scan_begin_arg = thisboard->ai_speed * cmd->chanlist_len;
			err++;
		}
	}
	if(cmd->convert_src == TRIG_TIMER)
	{
		if(cmd->convert_arg < thisboard->ai_speed)
		{
			cmd->convert_arg = thisboard->ai_speed;
			err++;
		}
	}

	if(cmd->stop_src == TRIG_NONE)
	{
		if(cmd->stop_arg != 0)
		{
			cmd->stop_arg = 0;
			err++;
		}
	}
	if(err)return 3;

	// step 4: fix up arguments
	if(cmd->scan_begin_src == TRIG_TIMER)
	{
		unsigned int tmp = cmd->scan_begin_arg;
		// set divisors, correct timing arguments
		i8253_cascade_ns_to_timer_2div(devpriv->clockbase,
			&(devpriv->divisor1), &(devpriv->divisor2),
			&(cmd->scan_begin_arg), cmd->flags & TRIG_ROUND_MASK);
		err += (tmp!=cmd->scan_begin_arg);
	}
	if(cmd->convert_src == TRIG_TIMER)
	{
		unsigned int tmp = cmd->convert_arg;
		// set divisors, correct timing arguments
		i8253_cascade_ns_to_timer_2div(devpriv->clockbase,
			&(devpriv->divisor1), &(devpriv->divisor2),
			&(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
		err += (tmp!=cmd->convert_arg);
	}
	if(err)return 4;

	// check channel/gain list against card's limitations
	if(cmd->chanlist){
		gain = CR_RANGE(cmd->chanlist[0]);
		start_chan = CR_CHAN(cmd->chanlist[0]);
		for(i = 1; i < cmd->chanlist_len; i++)
		{
			if(CR_CHAN(cmd->chanlist[i]) != (start_chan + i) % s->n_chan)
			{
				comedi_error(dev, "entries in chanlist must be consecutive channels, counting upwards\n");
				err++;
			}
			if(CR_RANGE(cmd->chanlist[i]) != gain)
			{
				comedi_error(dev, "entries in chanlist must all have the same gain\n");
				err++;
			}
		}
	}
	if(err)return 5;

	return 0;
}

static int das16_cmd_exec(comedi_device *dev,comedi_subdevice *s)
{
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned int byte;
	unsigned long flags;
	int range;

	if(dev->irq == 0 || devpriv->dma_chan == 0)
	{
		comedi_error(dev, "irq and dma required to execute comedi_cmd");
		return -1;
	}
	if(cmd->flags & TRIG_RT)
	{
		comedi_error(dev, "dma transfers cannot be performed with TRIG_RT, aborting");
		return -1;
	}

	devpriv->adc_count = cmd->stop_arg * cmd->chanlist_len;

	// disable conversions for das1600 mode
	if(thisboard->size > 0x400)
	{
		outb(DAS1600_CONV_DISABLE, dev->iobase + DAS1600_CONV);
	}

	// set scan limits
	byte = CR_CHAN(cmd->chanlist[0]);
	byte |= CR_CHAN(cmd->chanlist[cmd->chanlist_len - 1]) << 4;
	outb(byte, dev->iobase + DAS16_MUX);

	/* set gain (this is also burst rate register but according to
	 * computer boards manual, burst rate does nothing, even on keithley cards) */
	if(thisboard->ai_pg != das16_pg_none){
		range = CR_RANGE(cmd->chanlist[0]);
		outb((das16_gainlists[thisboard->ai_pg])[range],
			dev->iobase+DAS16_GAIN);
	}

	/* set counter mode and counts */
	cmd->convert_arg = das16_set_pacer(dev, cmd->convert_arg, cmd->flags & TRIG_ROUND_MASK);
	rt_printk("pacer period: %d ns\n", cmd->convert_arg);

	/* enable counters */
	byte = 0;
	/* Enable burst mode if appropriate. */
	if(thisboard->size > 0x400)
	{
		if(cmd->convert_src == TRIG_NOW)
		{
			outb(DAS1600_BURST_VAL, dev->iobase + DAS1600_BURST);
			// set burst length
			byte |= BURST_LEN_BITS(cmd->chanlist_len - 1);
		}else
		{
			outb(0, dev->iobase + DAS1600_BURST);
		}
	}
	outb(byte, dev->iobase + DAS16_PACER);


	// set up dma transfer
	flags = claim_dma_lock();
	disable_dma(devpriv->dma_chan);
	/* clear flip-flop to make sure 2-byte registers for
	 * count and address get set correctly */
	clear_dma_ff(devpriv->dma_chan);
	set_dma_addr(devpriv->dma_chan, virt_to_bus(devpriv->dma_buffer));
	// set appropriate size of transfer
	devpriv->dma_transfer_size = das16_suggest_transfer_size(*cmd);
	if(devpriv->adc_count * sample_size > devpriv->dma_transfer_size)
		set_dma_count(devpriv->dma_chan, devpriv->dma_transfer_size);
	else
		set_dma_count(devpriv->dma_chan, devpriv->adc_count * sample_size);
 	enable_dma(devpriv->dma_chan);
	release_dma_lock(flags);

	/* clear interrupt bit */
	outb(0x00, dev->iobase + DAS16_STATUS);
	async->events = 0;
	/* enable interrupts, dma and pacer clocked conversions */
	devpriv->control_state |= DAS16_INTE | DMA_ENABLE;
	if(cmd->convert_src == TRIG_EXT)
		devpriv->control_state |= EXT_PACER;
	else
		devpriv->control_state |= INT_PACER;
	outb(devpriv->control_state, dev->iobase + DAS16_CONTROL);

	/* Enable conversions if using das1600 mode */
	if(thisboard->size > 0x400)
	{
		outb(0, dev->iobase + DAS1600_CONV);
	}

	return 0;
}

static int das16_cancel(comedi_device *dev, comedi_subdevice *s)
{
	/* disable interrupts, dma and pacer clocked conversions */
	devpriv->control_state &= ~DAS16_INTE & ~INT_PACER & ~DMA_ENABLE;
	outb(devpriv->control_state, dev->iobase + DAS16_CONTROL);
	if(devpriv->dma_chan)
		disable_dma(devpriv->dma_chan);

	/* disable burst mode */
	if(thisboard->size > 0x400)
	{
		outb(0, dev->iobase + DAS1600_BURST);
	}

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

	/* set multiplexer */
	chan = CR_CHAN(insn->chanspec);
	chan |= CR_CHAN(insn->chanspec) << 4;
	outb(chan,dev->iobase+DAS16_MUX);

	/* set gain */
	if(thisboard->ai_pg != das16_pg_none){
		range = CR_RANGE(insn->chanspec);
		outb((das16_gainlists[thisboard->ai_pg])[range],
			dev->iobase+DAS16_GAIN);
	}

	for(n=0;n<insn->n;n++){
		/* trigger conversion */
		outb_p(0,dev->iobase+DAS16_TRIG);

		for(i=0;i<DAS16_TIMEOUT;i++){
			if(!(inb(dev->iobase + DAS16_STATUS) & BUSY))
				break;
		}
		if(i==DAS16_TIMEOUT){
			rt_printk("das16: timeout\n");
			return -ETIME;
		}
		msb = inb(dev->iobase + DAS16_AI_MSB);
		lsb = inb(dev->iobase + DAS16_AI_LSB);
		if(thisboard->ai_nbits==12){
			data[n] = ((lsb >> 4) & 0xf) | (msb << 4);
		}else{
			data[n] = lsb | (msb << 8);
		}
	}

	return n;
}

static int das16_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	lsampl_t bits;

	bits = inb(dev->iobase+DAS16_DIO) & 0xf;
	data[1] = bits;
	data[0] = 0;

	return 2;
}

static int das16_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	lsampl_t wbits;

	// only set bits that have been masked
	data[0] &= 0xf;
	wbits = devpriv->do_bits;
	// zero bits that have been masked
	wbits &= ~data[0];
	// set masked bits
	wbits |= data[0] & data[1];
	devpriv->do_bits = wbits;
	data[1] = wbits;

	outb(devpriv->do_bits, dev->iobase + DAS16_DIO);

	return 2;
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
	int i;
	int status;
	unsigned long flags;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async;
	unsigned int max_points, num_points, residue, leftover;
	sampl_t dpnt;

	if(dev->attached == 0)
	{
		comedi_error(dev, "premature interrupt");
		return;
	}
	// initialize async here to make sure s is not NULL
	async = s->async;

	status = inb(dev->iobase + DAS16_STATUS);

	if((status & DAS16_INT ) == 0)
	{
		comedi_error(dev, "spurious interrupt");
		return;
	}

	flags = claim_dma_lock();
	disable_dma(devpriv->dma_chan);
	/* clear flip-flop to make sure 2-byte registers for
	 * count and address get set correctly */
	clear_dma_ff(devpriv->dma_chan);

	// figure out how many points to read
	max_points = devpriv->dma_transfer_size  / sample_size;
	/* residue is the number of points left to be done on the dma
	 * transfer.  It should always be zero at this point unless
	 * the stop_src is set to external triggering.
	 */
	residue = get_dma_residue(devpriv->dma_chan) / sample_size;
	num_points = max_points - residue;
	if(devpriv->adc_count < num_points &&
		async->cmd.stop_src == TRIG_COUNT)
		num_points = devpriv->adc_count;

	// figure out how many points will be stored next time
	leftover = 0;
	if(async->cmd.stop_src == TRIG_NONE)
	{
		leftover = devpriv->dma_transfer_size / sample_size;
	}else if(devpriv->adc_count > max_points)
	{
		leftover = devpriv->adc_count - max_points;
		if(leftover > max_points)
			leftover = max_points;
	}
	/* there should only be a residue if collection was stopped by having
	 * the stop_src set to an external trigger, in which case there
	 * will be no more data
	 */
	if(residue)
		leftover = 0;

	for(i = 0; i < num_points; i++)
	{
		/* write data point to comedi buffer */
		dpnt = devpriv->dma_buffer[i];
		if(thisboard->ai_nbits == 12)
			dpnt = (dpnt >> 4) & 0xfff;
		comedi_buf_put(async, dpnt);
		if(devpriv->adc_count > 0) devpriv->adc_count--;
	}
	// re-enable  dma
	set_dma_addr(devpriv->dma_chan, virt_to_bus(devpriv->dma_buffer));
	set_dma_count(devpriv->dma_chan, leftover * sample_size);
	enable_dma(devpriv->dma_chan);
	release_dma_lock(flags);

	async->events |= COMEDI_CB_BLOCK;

	if(devpriv->adc_count == 0)
	{	/* end of acquisition */
		rt_printk("End of acquisition\n");
		das16_cancel(dev, s);
		async->events |= COMEDI_CB_EOA;
	}

	comedi_event(dev, s, async->events);
	async->events = 0;

	/* clear interrupt */
	outb(0x00, dev->iobase + DAS16_STATUS);
}

static unsigned int das16_set_pacer(comedi_device *dev, unsigned int ns, int rounding_flags)
{
	i8253_cascade_ns_to_timer_2div(devpriv->clockbase, &(devpriv->divisor1),
		&(devpriv->divisor2), &ns, rounding_flags & TRIG_ROUND_MASK);

	/* Write the values of ctr1 and ctr2 into counters 1 and 2 */
	i8254_load(dev->iobase + DAS16_CNTR0_DATA, 1, devpriv->divisor1, 2);
	i8254_load(dev->iobase + DAS16_CNTR0_DATA, 2, devpriv->divisor2, 2);

	return ns;
}

#ifdef DEBUG
static void reg_dump(comedi_device *dev)
{
	rt_printk("********DAS1600 REGISTER DUMP********\n");
	rt_printk("DAS16_MUX: %x\n", inb(dev->iobase+DAS16_MUX) );
	rt_printk("DAS16_DIO: %x\n", inb(dev->iobase+DAS16_DIO) );
	rt_printk("DAS16_STATUS: %x\n", inb(dev->iobase+DAS16_STATUS) );
	rt_printk("DAS16_CONTROL: %x\n", inb(dev->iobase+DAS16_CONTROL) );
	rt_printk("DAS16_PACER: %x\n", inb(dev->iobase+DAS16_PACER) );
	rt_printk("DAS16_GAIN: %x\n", inb(dev->iobase+DAS16_GAIN) );
	rt_printk("DAS16_CNTR_CONTROL: %x\n", inb(dev->iobase+DAS16_CNTR_CONTROL) );
	rt_printk("DAS1600_CONV: %x\n", inb(dev->iobase+DAS1600_CONV) );
	rt_printk("DAS1600_BURST: %x\n", inb(dev->iobase+DAS1600_BURST) );
	rt_printk("DAS1600_ENABLE: %x\n", inb(dev->iobase+DAS1600_ENABLE) );
	rt_printk("DAS1600_STATUS_B: %x\n", inb(dev->iobase+DAS1600_STATUS_B) );
}
#endif

static int das16_probe(comedi_device *dev, comedi_devconfig *it)
{
	int status;
	int diobits;

	/* status is available on all boards */

	status = inb(dev->iobase + DAS16_STATUS);

	if((status & UNIPOLAR)){
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

	printk(" id bits are 0x%02x\n",diobits);
	if(thisboard->id != diobits)
	{
		printk(" requested board's id bits are 0x%x\n", thisboard->id);
		return -1;
	}
	return 0;
}

static int das1600_mode_detect(comedi_device *dev)
{
	int status=0;

	status = inb(dev->iobase + DAS1600_STATUS_B);

	if(status & DAS1600_CLK_10MHZ) {
		devpriv->clockbase = 100;
		printk(" 10MHz pacer clock\n");
	} else {
		devpriv->clockbase = 1000;
		printk(" 1MHz pacer clock\n");
	}

	return 0;
}


/*
 *
 * Options list:
 *   0  I/O base
 *   1  IRQ
 *   2  DMA
 *   3  Clock speed (in MHz)
 */

static int das16_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret, irq;
	int iobase;
	int dma_chan;
	unsigned long flags;

	iobase = it->options[0];

	printk("comedi%d: das16:",dev->minor);

	// check that clock setting is valid
	if(it->options[3])
	{
		if(it->options[3] != 0 &&
			it->options[3] != 1 &&
			it->options[3] != 10)
		{
			printk("\n Invalid option.  Master clock must be set to 1 or 10 (MHz)\n");
			return -EINVAL;
		}
	}

	if((ret=alloc_private(dev,sizeof(struct das16_private_struct)))<0)
		return ret;

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

	if(thisboard->size < 0x400){
		request_region(iobase,thisboard->size,"das16");
	}else{
		request_region(iobase,0x10,"das16");
		request_region(iobase+0x400,thisboard->size&0x3ff,"das16");
	}

	dev->iobase = iobase;

	// probe id bits to make sure they are consistent
	if(das16_probe(dev, it))
	{
		printk(" id bits do not match selected board, aborting\n");
		return -EINVAL;
	}
	dev->board_name = thisboard->name;

	// get master clock speed
	if(thisboard->size < 0x400)
	{
		if(it->options[3])
			devpriv->clockbase = 1000 / it->options[3];
		else
			devpriv->clockbase = 1000;	// 1 MHz default
	}else
	{
		das1600_mode_detect(dev);
	}

	/* now for the irq */
	irq=it->options[1];
	if(irq > 1 && irq < 8)
	{
		if((ret=comedi_request_irq(irq,das16_interrupt,0,"das16",dev))<0)
			return ret;
		dev->irq=irq;
		printk(" ( irq = %d )",irq);
	}else if(irq == 0){
		printk(" ( no irq )");
	}else
	{
		printk(" invalid irq\n");
		return -EINVAL;
	}

	// initialize dma
	dma_chan = it->options[2];
	if(dma_chan == 1 || dma_chan == 3)
	{
		// allocate dma buffer
		devpriv->dma_buffer = kmalloc(DAS16_DMA_SIZE, GFP_KERNEL | GFP_DMA);
		if(devpriv->dma_buffer == NULL)
			return -ENOMEM;
		if(request_dma(dma_chan, "das16"))
		{
			printk(" failed to allocate dma channel %i\n", dma_chan);
			return -EINVAL;
		}
		devpriv->dma_chan = dma_chan;
		flags = claim_dma_lock();
		disable_dma(devpriv->dma_chan);
		set_dma_mode(devpriv->dma_chan, DMA_MODE_READ);
		release_dma_lock(flags);
		printk(" ( dma = %d)\n", dma_chan);
	}else if(dma_chan == 0){
		printk(" ( no dma )\n");
	}else
	{
		printk(" invalid dma channel\n");
		return -EINVAL;
	}

	dev->n_subdevices = 5;
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	s=dev->subdevices+0;
	dev->read_subdev=s;
	/* ai */
	if(thisboard->ai){
		s->type = COMEDI_SUBD_AI;
		s->subdev_flags = SDF_READABLE;
		if(devpriv->ai_singleended){
			s->n_chan = 16;
			s->len_chanlist = 16;
			s->subdev_flags |= SDF_GROUND;
		}else{
			s->n_chan = 8;
			s->len_chanlist = 8;
			s->subdev_flags |= SDF_DIFF;
		}
		s->maxdata = (1 << thisboard->ai_nbits) - 1;
		if(devpriv->ai_unipolar){
			s->range_table = das16_ai_uni_lranges[thisboard->ai_pg];
		}else{
			s->range_table = das16_ai_bip_lranges[thisboard->ai_pg];
		}
		s->insn_read = thisboard->ai;
		s->do_cmdtest = das16_cmd_test;
		s->do_cmd = das16_cmd_exec;
		s->cancel = das16_cancel;
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	s = dev->subdevices + 1;
	/* ao */
	if(thisboard->ao){
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_WRITEABLE;
		s->n_chan = 2;
		s->maxdata = (1 << thisboard->ao_nbits) - 1;
		s->range_table = &range_unknown;
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
		s->insn_bits = thisboard->di;
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}

	s = dev->subdevices + 3;
	/* do */
	if(thisboard->do_){
		s->type = COMEDI_SUBD_DO;
		s->subdev_flags = SDF_WRITEABLE | SDF_READABLE;
		s->n_chan = 4;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_bits = thisboard->do_;
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
	// initialize digital output lines
	outb(devpriv->do_bits, dev->iobase + DAS16_DIO);
	/* set the interrupt level,enable pacer clock */
	devpriv->control_state = DAS16_IRQ(dev->irq);
	outb(devpriv->control_state, dev->iobase + DAS16_CONTROL);

	// turn on das1600 mode if available
	if(thisboard->size > 0x400)
	{
		outb(DAS1600_ENABLE_VAL, dev->iobase + DAS1600_ENABLE);
		outb(0, dev->iobase + DAS1600_CONV);
		outb(0, dev->iobase + DAS1600_BURST);
	}

	return 0;
}


static int das16_detach(comedi_device *dev)
{
	printk("comedi%d: das16: remove\n", dev->minor);

	das16_reset(dev);
	
	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices+4);

	if(dev->irq)
		comedi_free_irq(dev->irq, dev);

	if(thisboard->size<0x400){
		release_region(dev->iobase,thisboard->size);
	}else{
		release_region(dev->iobase,0x10);
		release_region(dev->iobase+0x400,thisboard->size&0x3ff);
	}

	if(devpriv)
	{
		if(devpriv->dma_buffer)
			kfree(devpriv->dma_buffer);
		if(devpriv->dma_chan)
			free_dma(devpriv->dma_chan);
	}

	return 0;
}

COMEDI_INITCLEANUP(driver_das16);

// utility function that suggests a dma transfer size in bytes
static unsigned int das16_suggest_transfer_size(comedi_cmd cmd)
{
	unsigned int size;
	unsigned int freq;

	if(cmd.convert_src == TRIG_TIMER)
		freq = 1000000000 / cmd.convert_arg;
	else if(cmd.scan_begin_src == TRIG_TIMER)
		freq = (1000000000 / cmd.scan_begin_arg) * cmd.chanlist_len;
	// return some default value
	else
		freq = 0xffffffff;

	if(cmd.flags & TRIG_WAKE_EOS)
	{
		size = sample_size * cmd.chanlist_len;
	}else
	{
		// make buffer fill in no more than 1/3 second
		size = (freq / 3) * sample_size;
	}

	// set a minimum and maximum size allowed
	if(size > DAS16_DMA_SIZE)
		size = DAS16_DMA_SIZE - DAS16_DMA_SIZE % sample_size;
	else if(size < sample_size)
		size = sample_size;

	return size;
}

