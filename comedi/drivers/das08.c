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

*****************************************************************

*/
/*
Driver: das08.o
Description: DAS-08 compatible boards
Authors: Warren Jasper, ds, Frank Hess
Devices: [ComputerBoards] DAS08 (das08), DAS08-PGM (das08-pgm),
  DAS08-PGH (das08-pgh), DAS08-PGL (das08-pgl), DAS08-AOH (das08-aoh),
  DAS08-AOL (das08-aol), DAS08-AOM (das08-aom), DAS08/JR-AO (das08/jr-ao),
  DAS08/JR-16-AO (das08jr-16-ao), PCI-DAS08 (pci-das08), PCM-DAS08 (pcm-das08),
  PC104-DAS08 (pc104-das08), DAS08/JR/16 (das08jr/16)
Status: works

This is a rewrite of the das08 and das08jr drivers.

Options (for ISA cards):
        [0] - base io address

Options (for pci-das08):
        [0] - bus  (optional)
        [1] = slot (optional)
Use the name 'pci-das08' for the pci-das08, NOT 'das08'.

Options (for pcm-das08):
        NONE

The das08 driver doesn't support asynchronous commands, since
the cheap das08 hardware doesn't really support them (except for
pcm-das08).  The
comedi_rt_timer driver can be used to emulate commands for this
driver.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/comedidev.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <asm/io.h>

#include "8255.h"

// pcmcia includes
#ifdef CONFIG_PCMCIA
#include <pcmcia/version.h>
#include <pcmcia/cs_types.h>
#include <pcmcia/cs.h>
#include <pcmcia/cistpl.h>
#include <pcmcia/ds.h>
#endif	// CONFIG_PCMCIA

#define PCI_VENDOR_ID_COMPUTERBOARDS 0x1307
#define PCI_DEVICE_ID_PCIDAS08 0x29
#define PCIDAS08_SIZE 0x54

// pci configuration registers
#define INTCSR               0x4c
#define   INTR1_ENABLE         0x1
#define   INTR1_HIGH_POLARITY  0x2
#define   PCI_INTR_ENABLE      0x40
#define   INTR1_EDGE_TRIG      0x100	// requires high polarity
#define CNTRL                0x50
#define   CNTRL_DIR            0x2
#define   CNTRL_INTR           0x4


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
#define   DAS08_EOC			(1<<7)
#define   DAS08_IRQ			(1<<3)
#define   DAS08_IP(x)			(((x)>>4)&0x7)
#define DAS08_CONTROL		2
#define   DAS08_MUX_MASK	0x7
#define   DAS08_MUX(x)		((x) & DAS08_MUX_MASK)
#define   DAS08_INTE			(1<<3)
#define   DAS08_DO_MASK		0xf0
#define   DAS08_OP(x)		(((x) << 4) & DAS08_DO_MASK)

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

static int das08_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08jr_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08jr_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08jr_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int das08ao_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);

#ifdef CONFIG_PCMCIA
/*
   A linked list of "instances" of the das08_pcmcia device.  Each actual
   PCMCIA card corresponds to one device instance, and is described
   by one dev_link_t structure (defined in ds.h).

   You may not want to use a linked list for this -- for example, the
   memory card driver uses an array of dev_link_t pointers, where minor
   device numbers are used to derive the corresponding array index.
*/
static dev_link_t *dev_list = NULL;
#endif

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
}};/*
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


enum das08_lrange {das08_pg_none, das08_bipolar5, das08_pgh, das08_pgl, das08_pgm};

static comedi_lrange *das08_ai_lranges[]={
	&range_unknown,
	&range_bipolar5,
	&range_das08_pgh,
	&range_das08_pgl,
	&range_das08_pgm,
};

static int das08_pgh_gainlist[] = { 8, 0, 10, 2, 12, 4, 14, 6, 1, 3, 5, 7 };
static int das08_pgl_gainlist[] = { 8, 0, 2, 4, 6, 1, 3, 5, 7 };
static int das08_pgm_gainlist[] = { 8, 0, 10, 12, 14, 9, 11, 13, 15 };

static int *das08_gainlists[] = {
	NULL,
	NULL,
	das08_pgh_gainlist,
	das08_pgl_gainlist,
	das08_pgm_gainlist,
};

enum das08_bustype {isa, pci, pcmcia, pc104};
// different ways ai data is encoded in first two registers
enum das08_ai_encoding {das08_encode12, das08_encode16, das08_pcm_encode12};

typedef struct das08_board_struct{
	char		*name;
	unsigned int	id;	// id for pci/pcmcia boards
	enum das08_bustype	bustype;
	void		*ai;
	unsigned int	ai_nbits;
	enum das08_lrange	ai_pg;
	enum das08_ai_encoding	ai_encoding;
	void		*ao;
	unsigned int	ao_nbits;
	void		*di;
	void		*do_;
	unsigned int	do_nchan;
	unsigned int	i8255_offset;
	unsigned int	i8254_offset;
	unsigned int	iosize;	// number of ioports used
} das08_board;

static struct das08_board_struct das08_boards[]={
	{
	name:		"das08",		// cio-das08.pdf
	bustype:	isa,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pg_none,
	ai_encoding:	das08_encode12,
	ao:		NULL,
	ao_nbits:	12,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	do_nchan:	4,
	i8255_offset:	8,
	i8254_offset:	4,
	iosize:	16, // unchecked
	},
	{
	name:		"das08-pgm",		// cio-das08pgx.pdf
	bustype:	isa,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgm,
	ai_encoding:	das08_encode12,
	ao:		NULL,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	do_nchan:	4,
	i8255_offset:	0,
	i8254_offset:	0x04,
	iosize:	16, // unchecked
	},
	{
	name:		"das08-pgh",		// cio-das08pgx.pdf
	bustype:	isa,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgh,
	ai_encoding:	das08_encode12,
	ao:		NULL,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	do_nchan:	4,
	i8255_offset:	0,
	i8254_offset:	0x04,
	iosize:	16, // unchecked
	},
	{
	name:		"das08-pgl",		// cio-das08pgx.pdf
	bustype:	isa,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgl,
	ai_encoding:	das08_encode12,
	ao:		NULL,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	do_nchan:	4,
	i8255_offset:	0,
	i8254_offset:	0x04,
	iosize:	16, // unchecked
	},
	{
	name:		"das08-aoh",		// cio-das08_aox.pdf
	bustype:	isa,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgh,
	ai_encoding:	das08_encode12,
	ao:		das08ao_ao_winsn,	// 8
	ao_nbits:	12,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	do_nchan:	4,
	i8255_offset:	0x0c,
	i8254_offset:	0x04,
	iosize:	16, // unchecked
	},
	{
	name:		"das08-aol",		// cio-das08_aox.pdf
	bustype:	isa,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgl,
	ai_encoding:	das08_encode12,
	ao:		das08ao_ao_winsn,	// 8
	ao_nbits:	12,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	do_nchan:	4,
	i8255_offset:	0x0c,
	i8254_offset:	0x04,
	iosize:	16, // unchecked
	},
	{
	name:		"das08-aom",		// cio-das08_aox.pdf
	bustype:	isa,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pgm,
	ai_encoding:	das08_encode12,
	ao:		das08ao_ao_winsn,	// 8
	ao_nbits:	12,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	do_nchan:	4,
	i8255_offset:	0x0c,
	i8254_offset:	0x04,
	iosize:	16, // unchecked
	},
	{
	name:		"das08/jr-ao",		// cio-das08-jr-ao.pdf
	bustype:	isa,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pg_none,
	ai_encoding:	das08_encode12,
	ao:		das08jr_ao_winsn,
	ao_nbits:	12,
	di:		das08jr_di_rbits,
	do_:		das08jr_do_wbits,
	do_nchan:	8,
	i8255_offset:	0,
	i8254_offset:	0,
	iosize:	16, // unchecked
	},
	{
	name:		"das08jr-16-ao",	// cio-das08jr-16-ao.pdf
	bustype:	isa,
	ai:		das08_ai_rinsn,
	ai_nbits:	16,
	ai_pg:		das08_pg_none,
	ai_encoding:	das08_encode12,
	ao:		das08jr_ao_winsn,
	ao_nbits:	16,
	di:		das08jr_di_rbits,
	do_:		das08jr_do_wbits,
	do_nchan:	8,
	i8255_offset:	0,
	i8254_offset:	0x04,
	iosize:	16, // unchecked
	},
	{
	name:		"pci-das08",
	id:	PCI_DEVICE_ID_PCIDAS08,
	bustype:	pci,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_bipolar5,
	ai_encoding:	das08_encode12,
	ao:		NULL,
	ao_nbits:	0,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	do_nchan:	4,
	i8255_offset:	0,
	i8254_offset:	4,
	iosize:	8,
	},
#ifdef CONFIG_PCMCIA
	{
	name:		"pcm-das08",
	id:	0x0,	// XXX
	bustype:	pcmcia,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_bipolar5,
	ai_encoding:	das08_pcm_encode12,
	ao:		NULL,
	ao_nbits:	0,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	do_nchan:	3,
	i8255_offset:	0,
	i8254_offset:	0,
	iosize:	16, // unchecked
	},
#endif // CONFIG_PCMCIA
	{
	name:		"pc104-das08",
	bustype:	pc104,
	ai:		das08_ai_rinsn,
	ai_nbits:	12,
	ai_pg:		das08_pg_none,
	ai_encoding:	das08_encode12,
	ao:		NULL,
	ao_nbits:	0,
	di:		das08_di_rbits,
	do_:		das08_do_wbits,
	do_nchan:	4,
	i8255_offset:	0,
	i8254_offset:	4,
	iosize:	16, // unchecked
	},
#if 0
	{
	name:		"das08/f",
	},
	{
	name:		"das08jr",
	},
#endif
	{
	name:		"das08jr/16",
	bustype:	isa,
	ai:		das08_ai_rinsn,
	ai_nbits:	16,
	ai_pg:		das08_pg_none,
	ai_encoding:	das08_encode16,
	ao:		NULL,
	ao_nbits:	0,
	di:		das08jr_di_rbits,
	do_:		das08jr_do_wbits,
	do_nchan:	8,
	i8255_offset:	0,
	i8254_offset:	0,
	iosize:	16, // unchecked
	},
#if 0
	{
	name:		"das48-pga",		// cio-das48-pga.pdf
	},
	{
	name:		"das08-pga-g2",		// a KM board
	},
#endif
};
#define n_boardtypes sizeof(das08_boards)/sizeof(das08_boards[0])


struct das08_private_struct{
	unsigned int	do_mux_bits;	// bits for do/mux register on boards without seperate do register
	unsigned int	do_bits;	// bits for do register on boards with register dedicated to digital out only
	unsigned int	*pg_gainlist;
	struct pci_dev *pdev;	// struct for pci-das08
	unsigned int	pci_iobase;	// additional base address for pci-das08
};

static struct pci_device_id das08_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_COMPUTERBOARDS, PCI_DEVICE_ID_PCIDAS08, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, das08_pci_table);

#define devpriv ((struct das08_private_struct *)dev->private)
#define thisboard ((struct das08_board_struct *)dev->board_ptr)

#define TIMEOUT 100000

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
	spin_lock(&dev->spinlock);	// lock to prevent race with digital output
	devpriv->do_mux_bits &= ~DAS08_MUX_MASK;
	devpriv->do_mux_bits |= DAS08_MUX(chan);
	outb(devpriv->do_mux_bits,dev->iobase+DAS08_CONTROL);
	spin_unlock(&dev->spinlock);

	if(s->range_table->length > 1){
		/* set gain/range */
		range = CR_RANGE(insn->chanspec);
		outb(devpriv->pg_gainlist[range],dev->iobase+DAS08AO_GAIN_CONTROL);
	}

	/* wait for MUX to settle */
	udelay(2);

	for(n=0;n<insn->n;n++){
		/* clear over-range bits for 16-bit boards */
		if (thisboard->ai_nbits == 16)
			if (inb(dev->iobase + DAS08_MSB) & 0x80)
				rt_printk("das08: over-range\n");

		/* trigger conversion */
		outb_p(0,dev->iobase+DAS08_TRIG_12BIT);

		for(i=0;i<TIMEOUT;i++){
			if(!(inb(dev->iobase+DAS08_STATUS)&DAS08_EOC))
				break;
		}
		if(i==TIMEOUT){
			rt_printk("das08: timeout\n");
			return -ETIME;
		}
		msb = inb(dev->iobase + DAS08_MSB);
		lsb = inb(dev->iobase + DAS08_LSB);
		if(thisboard->ai_encoding == das08_encode12){
			data[n] = (lsb>>4) | (msb << 4);
		}else if(thisboard->ai_encoding == das08_pcm_encode12){
			data[n] = (msb << 8) + lsb;
		}else if(thisboard->ai_encoding == das08_encode16){
			/* FPOS 16-bit boards are sign-magnitude */
			if (msb & 0x80)
				data[n] = (1 << 15) | lsb | ((msb & 0x7f) << 8);
			else
				data[n] = (1 << 15) - (lsb | (msb & 0x7f) << 8);
		}else{
			comedi_error(dev, "bug! unknown ai encoding");
			return -1;
		}
	}

	return n;
}

static int das08_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	data[0] = 0;
	data[1] = DAS08_IP(inb(dev->iobase+DAS08_STATUS));

	return 2;
}

static int das08_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int wbits;

	// get current settings of digital output lines
	wbits = (devpriv->do_mux_bits >> 4) & 0xf;
	// null bits we are going to set
	wbits &= ~data[0];
	// set new bit values
	wbits |= data[0] & data[1];
	// remember digital output bits
	spin_lock(&dev->spinlock);	// prevent race with setting of analog input mux
	devpriv->do_mux_bits &= ~DAS08_DO_MASK;
	devpriv->do_mux_bits |= DAS08_OP(wbits);
	outb(devpriv->do_mux_bits, dev->iobase + DAS08_CONTROL);
	spin_unlock(&dev->spinlock);

	data[1] = wbits;

	return 2;
}

static int das08jr_di_rbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	data[0] = 0;
	data[1] = inb(dev->iobase + DAS08JR_DIO);

	return 2;
}

static int das08jr_do_wbits(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	// null bits we are going to set
	devpriv->do_bits &= ~data[0];
	// set new bit values
	devpriv->do_bits |= data[0] & data[1];
	outb(devpriv->do_bits, dev->iobase + DAS08JR_DIO);

	data[1] = devpriv->do_bits;

	return 2;
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
	}

	return n;
}

static int das08_attach(comedi_device *dev,comedi_devconfig *it);
static int das08_detach(comedi_device *dev);

static comedi_driver driver_das08={
	driver_name:	"das08",
	module:		THIS_MODULE,
	attach:		das08_attach,
	detach:		das08_detach,
	board_name:	das08_boards,
	num_names:	sizeof(das08_boards)/sizeof(struct das08_board_struct),
	offset:		sizeof(struct das08_board_struct),
};

static int das08_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret;
	int iobase, pci_iobase = 0;
	struct pci_dev *pdev;
#ifdef CONFIG_PCMCIA
	dev_link_t *link = dev_list;	// XXX hack
#endif

	if((ret=alloc_private(dev,sizeof(struct das08_private_struct)))<0)
		return ret;

	printk("comedi%d: das08: ", dev->minor);
	// deal with a pci board
	if(thisboard->bustype == pci)
	{
		if(it->options[0] || it->options[1]){
			printk("bus %i slot %i ",
				it->options[0], it->options[1]);
		}
		printk("\n");
		// find card
		pci_for_each_dev(pdev){
			if(pdev->vendor == PCI_VENDOR_ID_COMPUTERBOARDS &&
				pdev->device == PCI_DEVICE_ID_PCIDAS08){
				if(it->options[0] || it->options[1]){
					if(pdev->bus->number == it->options[0] &&
					   PCI_SLOT(pdev->devfn) == it->options[1]){
						break;
					}
				}else{
					break;
				}
			}
		}
		if(!pdev){
			printk("No pci das08 cards found\n");
			return -EIO;
		}
		devpriv->pdev = pdev;
		// read base addresses
		if(pci_enable_device(pdev))
			return -EIO;
		pci_iobase = pci_resource_start(pdev, 1) & PCI_BASE_ADDRESS_IO_MASK;
		iobase = pci_resource_start(pdev, 2) & PCI_BASE_ADDRESS_IO_MASK;
		printk("pcibase 0x%x ", pci_iobase);
		// reserve io ports for 9052 pci chip
		if(check_region(pci_iobase,PCIDAS08_SIZE)<0){
			printk(" I/O port conflict\n");
			return -EIO;
		}
		request_region(pci_iobase,PCIDAS08_SIZE,"das08");
		devpriv->pci_iobase = pci_iobase;

#if 0
/* We could enable to pci-das08's interrupt here to make it possible
 * to do timed input in this driver, but there is little point since
 * conversions would have to be started by the interrupt handler
 * so you might as well use comedi_rt_timer to emulate commands
 */
		/* set source of interrupt trigger to counter2 output */
		outb(CNTRL_INTR | CNTRL_DIR, pci_iobase + CNTRL);
		/* Enable local interrupt 1 and pci interrupt */
		outw(INTR1_ENABLE | PCI_INTR_ENABLE, pci_iobase + INTCSR );
#endif

#ifdef CONFIG_PCMCIA
	}else if(thisboard->bustype == pcmcia)
	{
		if(link == NULL)
		{
			printk(" no pcmcia cards found\n");
			return -EIO;
		}
		iobase = link->io.BasePort1;
#endif // CONFIG_PCMCIA
	}else{
		iobase = it->options[0];
	}

	// allocate ioports for non-pcmcia boards
	if(thisboard->bustype != pcmcia)
	{
		printk("iobase 0x%x ", iobase);
		if(check_region(iobase, thisboard->iosize)<0){
			printk(" I/O port conflict\n");
			return -EIO;
		}
		request_region(iobase, thisboard->iosize,"das08");
	}
	dev->iobase = iobase;

	printk("\n");

	dev->board_name = thisboard->name;

	dev->n_subdevices=5;
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	s=dev->subdevices+0;
	/* ai */
	if(thisboard->ai){
		s->type=COMEDI_SUBD_AI;
	/* XXX some boards actually have differential inputs instead of single ended.
	 *  The driver does nothing with arefs though, so it's no big deal. */
		s->subdev_flags = SDF_READABLE | SDF_GROUND;
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
// XXX lacks read-back insn
		s->subdev_flags = SDF_WRITEABLE;
		s->n_chan = 2;
		s->maxdata = (1<<thisboard->ao_nbits)-1;
		s->range_table = &range_bipolar5;
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
		s->insn_bits = thisboard->di;
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	s=dev->subdevices+3;
	/* do */
	if(thisboard->do_){
		s->type=COMEDI_SUBD_DO;
		s->subdev_flags = SDF_WRITEABLE | SDF_READABLE;
		s->n_chan = thisboard->do_nchan;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_bits = thisboard->do_;
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	s=dev->subdevices+4;
	/* 8255 */
	if(thisboard->i8255_offset!=0){
		subdev_8255_init(dev,s,NULL,(unsigned long)(dev->iobase+
			thisboard->i8255_offset));
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	return 0;
}

static int das08_detach(comedi_device *dev)
{
	printk(KERN_INFO "comedi%d: das08: remove\n",dev->minor);

	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices+4);

	// deallocate ioports for non-pcmcia boards
	if(thisboard->bustype != pcmcia)
	{
		if(dev->iobase)
			release_region(dev->iobase, thisboard->iosize);
	}

	if(devpriv){
		if(devpriv->pci_iobase){
			release_region(devpriv->pci_iobase, PCIDAS08_SIZE);
		}
	}

	return 0;
}

/*======================================================================

    The following pcmcia code for the pcm-das08 is adapted from the
    dummy_cs.c driver of the Linux PCMCIA Card Services package.

    The initial developer of the original code is David A. Hinds
    <dahinds@users.sourceforge.net>.  Portions created by David A. Hinds
    are Copyright (C) 1999 David A. Hinds.  All Rights Reserved.

======================================================================*/

/*
   All the PCMCIA modules use PCMCIA_DEBUG to control debugging.  If
   you do not define PCMCIA_DEBUG at all, all the debug code will be
   left out.  If you compile with PCMCIA_DEBUG=0, the debug code will
   be present but disabled -- but it can then be enabled for specific
   modules at load time with a 'pc_debug=#' option to insmod.
*/
#ifdef CONFIG_PCMCIA

#ifdef PCMCIA_DEBUG
static int pc_debug = PCMCIA_DEBUG;
MODULE_PARM(pc_debug, "i");
#define DEBUG(n, args...) if (pc_debug>(n)) printk(KERN_DEBUG args)
static char *version =
"das08.c pcmcia code (Frank Hess), modified from dummy_cs.c 1.31 2001/08/24 12:13:13 (David Hinds)";
#else
#define DEBUG(n, args...)
#endif

/*====================================================================*/

/* Parameters that can be set with 'insmod' */

/* The old way: bit map of interrupts to choose from */
/* This means pick from 15, 14, 12, 11, 10, 9, 7, 5, 4, and 3 */
static u_int irq_mask = 0xdeb8;
/* Newer, simpler way of listing specific interrupts */
static int irq_list[4] = { -1 };

MODULE_PARM(irq_mask, "i");
MODULE_PARM(irq_list, "1-4i");

/*====================================================================*/

/*
   The event() function is this driver's Card Services event handler.
   It will be called by Card Services when an appropriate card status
   event is received.  The config() and release() entry points are
   used to configure or release a socket, in response to card
   insertion and ejection events.  They are invoked from the das08_pcmcia
   event handler.
*/

static void das08_pcmcia_config(dev_link_t *link);
static void das08_pcmcia_release(u_long arg);
static int das08_pcmcia_event(event_t event, int priority,
		       event_callback_args_t *args);

/*
   The attach() and detach() entry points are used to create and destroy
   "instances" of the driver, where each instance represents everything
   needed to manage one actual PCMCIA card.
*/

static dev_link_t *das08_pcmcia_attach(void);
static void das08_pcmcia_detach(dev_link_t *);

/*
   You'll also need to prototype all the functions that will actually
   be used to talk to your device.  See 'memory_cs' for a good example
   of a fully self-sufficient driver; the other drivers rely more or
   less on other parts of the kernel.
*/

/*
   The dev_info variable is the "key" that is used to match up this
   device driver with appropriate cards, through the card configuration
   database.
*/

static dev_info_t dev_info = "das08";

/*
   A dev_link_t structure has fields for most things that are needed
   to keep track of a socket, but there will usually be some device
   specific information that also needs to be kept track of.  The
   'priv' pointer in a dev_link_t structure can be used to point to
   a device-specific private data structure, like this.

   To simplify the data structure handling, we actually include the
   dev_link_t structure in the device's private data structure.

   A driver needs to provide a dev_node_t structure for each device
   on a card.  In some cases, there is only one device per card (for
   example, ethernet cards, modems).  In other cases, there may be
   many actual or logical devices (SCSI adapters, memory cards with
   multiple partitions).  The dev_node_t structures need to be kept
   in a linked list starting at the 'dev' field of a dev_link_t
   structure.  We allocate them in the card's private data structure,
   because they generally shouldn't be allocated dynamically.

   In this case, we also provide a flag to indicate if a device is
   "stopped" due to a power management event, or card ejection.  The
   device IO routines can use a flag like this to throttle IO to a
   card that is not ready to accept it.

   The bus_operations pointer is used on platforms for which we need
   to use special socket-specific versions of normal IO primitives
   (inb, outb, readb, writeb, etc) for card IO.
*/

typedef struct local_info_t {
    dev_link_t		link;
    dev_node_t		node;
    int			stop;
    struct bus_operations *bus;
} local_info_t;

/*====================================================================*/

static void cs_error(client_handle_t handle, int func, int ret)
{
    error_info_t err = { func, ret };
    CardServices(ReportError, handle, &err);
}

/*======================================================================

    das08_pcmcia_attach() creates an "instance" of the driver, allocating
    local data structures for one device.  The device is registered
    with Card Services.

    The dev_link structure is initialized, but we don't actually
    configure the card at this point -- we wait until we receive a
    card insertion event.

======================================================================*/

static dev_link_t *das08_pcmcia_attach(void)
{
    local_info_t *local;
    dev_link_t *link;
    client_reg_t client_reg;
    int ret, i;

    DEBUG(0, "das08_pcmcia_attach()\n");

    /* Allocate space for private device-specific data */
    local = kmalloc(sizeof(local_info_t), GFP_KERNEL);
    if (!local) return NULL;
    memset(local, 0, sizeof(local_info_t));
    link = &local->link; link->priv = local;

    /* Initialize the dev_link_t structure */
    link->release.function = &das08_pcmcia_release;
    link->release.data = (u_long)link;

    /* Interrupt setup */
    link->irq.Attributes = IRQ_TYPE_EXCLUSIVE;
    link->irq.IRQInfo1 = IRQ_INFO2_VALID|IRQ_LEVEL_ID;
    if (irq_list[0] == -1)
	link->irq.IRQInfo2 = irq_mask;
    else
	for (i = 0; i < 4; i++)
	    link->irq.IRQInfo2 |= 1 << irq_list[i];
    link->irq.Handler = NULL;

    /*
      General socket configuration defaults can go here.  In this
      client, we assume very little, and rely on the CIS for almost
      everything.  In most clients, many details (i.e., number, sizes,
      and attributes of IO windows) are fixed by the nature of the
      device, and can be hard-wired here.
    */
    link->conf.Attributes = 0;
    link->conf.Vcc = 50;
    link->conf.IntType = INT_MEMORY_AND_IO;

    /* Register with Card Services */
    link->next = dev_list;
    dev_list = link;
    client_reg.dev_info = &dev_info;
    client_reg.Attributes = INFO_IO_CLIENT | INFO_CARD_SHARE;
    client_reg.EventMask =
	CS_EVENT_CARD_INSERTION | CS_EVENT_CARD_REMOVAL |
	CS_EVENT_RESET_PHYSICAL | CS_EVENT_CARD_RESET |
	CS_EVENT_PM_SUSPEND | CS_EVENT_PM_RESUME;
    client_reg.event_handler = &das08_pcmcia_event;
    client_reg.Version = 0x0210;
    client_reg.event_callback_args.client_data = link;
    ret = CardServices(RegisterClient, &link->handle, &client_reg);
    if (ret != CS_SUCCESS) {
	cs_error(link->handle, RegisterClient, ret);
	das08_pcmcia_detach(link);
	return NULL;
    }

    return link;
} /* das08_pcmcia_attach */

/*======================================================================

    This deletes a driver "instance".  The device is de-registered
    with Card Services.  If it has been released, all local data
    structures are freed.  Otherwise, the structures will be freed
    when the device is released.

======================================================================*/

static void das08_pcmcia_detach(dev_link_t *link)
{
	dev_link_t **linkp;

	DEBUG(0, "das08_pcmcia_detach(0x%p)\n", link);

	/* Locate device structure */
	for (linkp = &dev_list; *linkp; linkp = &(*linkp)->next)
	if (*linkp == link) break;
	if (*linkp == NULL)
	return;

	/*
	If the device is currently configured and active, we won't
	actually delete it yet.  Instead, it is marked so that when
	the release() function is called, that will trigger a proper
	detach().
	*/
	if (link->state & DEV_CONFIG)
	{
#ifdef PCMCIA_DEBUG
		printk(KERN_DEBUG "das08: detach postponed, '%s' "
			"still locked\n", link->dev->dev_name);
#endif
		link->state |= DEV_STALE_LINK;
		return;
	}

	/* Break the link with Card Services */
	if (link->handle)
		CardServices(DeregisterClient, link->handle);

	/* Unlink device structure, and free it */
	*linkp = link->next;
	/* This points to the parent local_info_t struct */
	kfree(link->priv);

} /* das08_pcmcia_detach */

/*======================================================================

    das08_pcmcia_config() is scheduled to run after a CARD_INSERTION event
    is received, to configure the PCMCIA socket, and to make the
    device available to the system.

======================================================================*/

#define CS_CHECK(fn, args...) \
while ((last_ret=CardServices(last_fn=(fn),args))!=0) goto cs_failed

#define CFG_CHECK(fn, args...) \
if (CardServices(fn, args) != 0) goto next_entry

static void das08_pcmcia_config(dev_link_t *link)
{
	client_handle_t handle = link->handle;
	local_info_t *dev = link->priv;
	tuple_t tuple;
	cisparse_t parse;
	int last_fn, last_ret;
	u_char buf[64];
	config_info_t conf;
	win_req_t req;
	cistpl_cftable_entry_t dflt = { 0 };

	DEBUG(0, "das08_pcmcia_config(0x%p)\n", link);

	/*
		This reads the card's CONFIG tuple to find its configuration
		registers.
	*/
	tuple.DesiredTuple = CISTPL_CONFIG;
	tuple.Attributes = 0;
	tuple.TupleData = buf;
	tuple.TupleDataMax = sizeof(buf);
	tuple.TupleOffset = 0;
	CS_CHECK(GetFirstTuple, handle, &tuple);
	CS_CHECK(GetTupleData, handle, &tuple);
	CS_CHECK(ParseTuple, handle, &tuple, &parse);
	link->conf.ConfigBase = parse.config.base;
	link->conf.Present = parse.config.rmask[0];

	/* Configure card */
	link->state |= DEV_CONFIG;

	/* Look up the current Vcc */
	CS_CHECK(GetConfigurationInfo, handle, &conf);
	link->conf.Vcc = conf.Vcc;

	/*
	In this loop, we scan the CIS for configuration table entries,
	each of which describes a valid card configuration, including
	voltage, IO window, memory window, and interrupt settings.

	We make no assumptions about the card to be configured: we use
	just the information available in the CIS.  In an ideal world,
	this would work for any PCMCIA card, but it requires a complete
	and accurate CIS.  In practice, a driver usually "knows" most of
	these things without consulting the CIS, and most client drivers
	will only use the CIS to fill in implementation-defined details.
	*/
	tuple.DesiredTuple = CISTPL_CFTABLE_ENTRY;
	CS_CHECK(GetFirstTuple, handle, &tuple);
	while (1) {
	cistpl_cftable_entry_t *cfg = &(parse.cftable_entry);
	CFG_CHECK(GetTupleData, handle, &tuple);
	CFG_CHECK(ParseTuple, handle, &tuple, &parse);

	if (cfg->flags & CISTPL_CFTABLE_DEFAULT) dflt = *cfg;
	if (cfg->index == 0) goto next_entry;
	link->conf.ConfigIndex = cfg->index;

	/* Does this card need audio output? */
/*	if (cfg->flags & CISTPL_CFTABLE_AUDIO) {
		link->conf.Attributes |= CONF_ENABLE_SPKR;
		link->conf.Status = CCSR_AUDIO_ENA;
	}
*/
	/* Use power settings for Vcc and Vpp if present */
	/*  Note that the CIS values need to be rescaled */
	if (cfg->vcc.present & (1<<CISTPL_POWER_VNOM)) {
		if (conf.Vcc != cfg->vcc.param[CISTPL_POWER_VNOM]/10000)
		goto next_entry;
	} else if (dflt.vcc.present & (1<<CISTPL_POWER_VNOM)) {
		if (conf.Vcc != dflt.vcc.param[CISTPL_POWER_VNOM]/10000)
		goto next_entry;
	}

	if (cfg->vpp1.present & (1<<CISTPL_POWER_VNOM))
		link->conf.Vpp1 = link->conf.Vpp2 =
		cfg->vpp1.param[CISTPL_POWER_VNOM]/10000;
	else if (dflt.vpp1.present & (1<<CISTPL_POWER_VNOM))
		link->conf.Vpp1 = link->conf.Vpp2 =
		dflt.vpp1.param[CISTPL_POWER_VNOM]/10000;

	/* Do we need to allocate an interrupt? */
	if (cfg->irq.IRQInfo1 || dflt.irq.IRQInfo1)
		link->conf.Attributes |= CONF_ENABLE_IRQ;

	/* IO window settings */
	link->io.NumPorts1 = link->io.NumPorts2 = 0;
	if ((cfg->io.nwin > 0) || (dflt.io.nwin > 0)) {
		cistpl_io_t *io = (cfg->io.nwin) ? &cfg->io : &dflt.io;
		link->io.Attributes1 = IO_DATA_PATH_WIDTH_AUTO;
		if (!(io->flags & CISTPL_IO_8BIT))
		link->io.Attributes1 = IO_DATA_PATH_WIDTH_16;
		if (!(io->flags & CISTPL_IO_16BIT))
		link->io.Attributes1 = IO_DATA_PATH_WIDTH_8;
		link->io.IOAddrLines = io->flags & CISTPL_IO_LINES_MASK;
		link->io.BasePort1 = io->win[0].base;
		link->io.NumPorts1 = io->win[0].len;
		if (io->nwin > 1) {
		link->io.Attributes2 = link->io.Attributes1;
		link->io.BasePort2 = io->win[1].base;
		link->io.NumPorts2 = io->win[1].len;
		}
		/* This reserves IO space but doesn't actually enable it */
		CFG_CHECK(RequestIO, link->handle, &link->io);
	}

	/* If we got this far, we're cool! */
	break;

	next_entry:
	if (link->io.NumPorts1)
		CardServices(ReleaseIO, link->handle, &link->io);
	CS_CHECK(GetNextTuple, handle, &tuple);
	}

	/*
		Allocate an interrupt line.  Note that this does not assign a
		handler to the interrupt, unless the 'Handler' member of the
		irq structure is initialized.
	*/
	if (link->conf.Attributes & CONF_ENABLE_IRQ)
	CS_CHECK(RequestIRQ, link->handle, &link->irq);

	/*
		This actually configures the PCMCIA socket -- setting up
		the I/O windows and the interrupt mapping, and putting the
		card and host interface into "Memory and IO" mode.
	*/
	CS_CHECK(RequestConfiguration, link->handle, &link->conf);

	/*
		At this point, the dev_node_t structure(s) need to be
		initialized and arranged in a linked list at link->dev.
	*/
	sprintf(dev->node.dev_name, "skel0");
	dev->node.major = dev->node.minor = 0;
	link->dev = &dev->node;

	/* Finally, report what we've done */
	printk(KERN_INFO "%s: index 0x%02x: Vcc %d.%d",
		dev->node.dev_name, link->conf.ConfigIndex,
		link->conf.Vcc/10, link->conf.Vcc%10);
	if (link->conf.Vpp1)
	printk(", Vpp %d.%d", link->conf.Vpp1/10, link->conf.Vpp1%10);
	if (link->conf.Attributes & CONF_ENABLE_IRQ)
	printk(", irq %d", link->irq.AssignedIRQ);
	if (link->io.NumPorts1)
	printk(", io 0x%04x-0x%04x", link->io.BasePort1,
			link->io.BasePort1+link->io.NumPorts1-1);
	if (link->io.NumPorts2)
	printk(" & 0x%04x-0x%04x", link->io.BasePort2,
			link->io.BasePort2+link->io.NumPorts2-1);
	if (link->win)
	printk(", mem 0x%06lx-0x%06lx", req.Base,
			req.Base+req.Size-1);
	printk("\n");

	link->state &= ~DEV_CONFIG_PENDING;
	return;

cs_failed:
	cs_error(link->handle, last_fn, last_ret);
	das08_pcmcia_release((u_long)link);

} /* das08_pcmcia_config */

/*======================================================================

    After a card is removed, das08_pcmcia_release() will unregister the
    device, and release the PCMCIA configuration.  If the device is
    still open, this will be postponed until it is closed.

======================================================================*/

static void das08_pcmcia_release(u_long arg)
{
	dev_link_t *link = (dev_link_t *)arg;

	DEBUG(0, "das08_pcmcia_release(0x%p)\n", link);

    /*
       If the device is currently in use, we won't release until it
       is actually closed, because until then, we can't be sure that
       no one will try to access the device or its data structures.
    */
	if (link->open)
	{
		DEBUG(1, "das08: release postponed, '%s' still open\n",
			link->dev->dev_name);
		link->state |= DEV_STALE_CONFIG;
		return;
	}

	/* Unlink the device chain */
	link->dev = NULL;

    /*
      In a normal driver, additional code may be needed to release
      other kernel data structures associated with this device.
    */

    /* Don't bother checking to see if these succeed or not */
	if (link->win)
		CardServices(ReleaseWindow, link->win);
	CardServices(ReleaseConfiguration, link->handle);
	if (link->io.NumPorts1)
		CardServices(ReleaseIO, link->handle, &link->io);
	if (link->irq.AssignedIRQ)
		CardServices(ReleaseIRQ, link->handle, &link->irq);
	link->state &= ~DEV_CONFIG;

	if (link->state & DEV_STALE_LINK)
		das08_pcmcia_detach(link);

} /* das08_pcmcia_release */

/*======================================================================

    The card status event handler.  Mostly, this schedules other
    stuff to run after an event is received.

    When a CARD_REMOVAL event is received, we immediately set a
    private flag to block future accesses to this device.  All the
    functions that actually access the device should check this flag
    to make sure the card is still present.

======================================================================*/

static int das08_pcmcia_event(event_t event, int priority,
	event_callback_args_t *args)
{
	dev_link_t *link = args->client_data;
	local_info_t *dev = link->priv;

	DEBUG(1, "das08_pcmcia_event(0x%06x)\n", event);

	switch (event)
	{
		case CS_EVENT_CARD_REMOVAL:
			link->state &= ~DEV_PRESENT;
			if (link->state & DEV_CONFIG)
			{
				((local_info_t *)link->priv)->stop = 1;
				mod_timer(&link->release, jiffies + HZ/20);
			}
			break;
		case CS_EVENT_CARD_INSERTION:
			link->state |= DEV_PRESENT | DEV_CONFIG_PENDING;
			dev->bus = args->bus;
			das08_pcmcia_config(link);
			break;
		case CS_EVENT_PM_SUSPEND:
			link->state |= DEV_SUSPEND;
			/* Fall through... */
		case CS_EVENT_RESET_PHYSICAL:
			/* Mark the device as stopped, to block IO until later */
			dev->stop = 1;
			if (link->state & DEV_CONFIG)
			CardServices(ReleaseConfiguration, link->handle);
			break;
		case CS_EVENT_PM_RESUME:
			link->state &= ~DEV_SUSPEND;
			/* Fall through... */
		case CS_EVENT_CARD_RESET:
			if (link->state & DEV_CONFIG)
				CardServices(RequestConfiguration, link->handle, &link->conf);
			dev->stop = 0;
/*
In a normal driver, additional code may go here to restore
the device state and restart IO.
*/
			break;
	}

	return 0;
} /* das08_pcmcia_event */

/*====================================================================*/

static int __init init_das08_pcmcia_cs(void)
{
	servinfo_t serv;
	DEBUG(0, "%s\n", version);
	CardServices(GetCardServicesInfo, &serv);
	if (serv.Revision != CS_RELEASE_CODE)
	{
		printk(KERN_NOTICE "das08: Card Services release "
			"does not match!\n");
		return -1;
	}
	register_pccard_driver(&dev_info, &das08_pcmcia_attach, &das08_pcmcia_detach);
	return 0;
}

static void __exit exit_das08_pcmcia_cs(void)
{
	DEBUG(0, "das08_pcmcia_cs: unloading\n");
	unregister_pccard_driver(&dev_info);
	while (dev_list != NULL)
	{
		del_timer(&dev_list->release);
		if (dev_list->state & DEV_CONFIG)
			das08_pcmcia_release((u_long)dev_list);
		das08_pcmcia_detach(dev_list);
	}
}

int init_module(void)
{
	int ret;

	ret = init_das08_pcmcia_cs();
	if(ret < 0)
		return ret;

	return comedi_driver_register(&driver_das08);
}

void cleanup_module(void)
{
	exit_das08_pcmcia_cs();
	comedi_driver_unregister(&driver_das08);
}

#else
COMEDI_INITCLEANUP(driver_das08);
#endif //CONFIG_PCMCIA
