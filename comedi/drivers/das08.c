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

Support for pci-das08 card added by Frank M. Hess

*/
/*
Driver: das08.o
Description: DAS-08 compatible boards
Authors: Warren Jasper, ds
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

The das08 driver doesn't support asynchronous commands, since
the cheap das08 hardware doesn't really support them.  The
comedi_rt_timer driver can be used to emulate commands for this
driver.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/comedidev.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/malloc.h>
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
#define PCIDAS08_SIZE 128

// pci configuration registers
#define INTCSR               0x4c
#define   INTR1_ENABLE         0x1
#define   INTR1_HIGH_POLARITY  0x2
#define   PCI_INTR_ENABLE      0x40
#define   INTR1_EDGE_TRIG      0x100	// requires high polarity
#define CNTRL                0x50
#define   CNTRL_DIR            0x2
#define   CNTRL_INTR           0x4


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

	if((ret=alloc_private(dev,sizeof(struct das08_private_struct)))<0)
		return ret;

	printk("comedi%d: das08", dev->minor);
	// deal with a pci board
	if(thisboard->bustype == pci)
	{
		if(it->options[0] || it->options[1]){
			printk(": bus %i, slot %i",
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
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
		pci_iobase = pdev->base_address[1] & PCI_BASE_ADDRESS_IO_MASK;
		iobase = pdev->base_address[2] & PCI_BASE_ADDRESS_IO_MASK;
#else
		if(pci_enable_device(pdev))
			return -EIO;
		pci_iobase = pdev->resource[1].start & PCI_BASE_ADDRESS_IO_MASK;
		iobase = pdev->resource[2].start & PCI_BASE_ADDRESS_IO_MASK;
#endif

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
		// XXX deal with pcmcia board
#endif // CONFIG_PCMCIA
	}else{
		iobase = it->options[0];
		printk(": 0x%04x\n", iobase);
	}


	if(check_region(iobase,DAS08_SIZE)<0){
		printk(" I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase,DAS08_SIZE,"das08");
	dev->iobase = iobase;

	dev->board_name = thisboard->name;

	dev->n_subdevices=5;
	if((ret=alloc_subdevices(dev))<0)
		return ret;

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

	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices+4);

	if(dev->iobase)
		release_region(dev->iobase,DAS08_SIZE);

	if(devpriv){
		if(devpriv->pci_iobase){
			release_region(devpriv->pci_iobase, PCIDAS08_SIZE);
		}
	}

	return 0;
}

COMEDI_INITCLEANUP(driver_das08);

