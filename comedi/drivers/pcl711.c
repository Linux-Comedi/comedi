/*
   module/pcl711.c
   hardware driver for PC-LabCard PCL-711 and AdSys ACL-8112
   and compatibles

   COMEDI - Linux Control and Measurement Device Interface
   Copyright (C) 1998 David A. Schleef <ds@stm.lbl.gov>
   Janne Jalkanen <jalkanen@cs.hut.fi>
   Eric Bunn <ebu@cs.hut.fi>

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
   Dave Andruczyk <dave@tech.buffalostate.edu> also wrote a
   driver for the PCL-711.  I used a few ideas from his driver
   here.  His driver also has more comments, if you are
   interested in understanding how this driver works.
   http://tech.buffalostate.edu/~dave/driver/

   The ACL-8112 driver was hacked from the sources of the PCL-711
   driver (the 744 chip used on the 8112 is almost the same as
   the 711b chip, but it has more I/O channels) by
   Janne Jalkanen (jalkanen@cs.hut.fi) and
   Erik Bunn (ebu@cs.hut.fi).  Remerged with the PCL-711 driver
   by ds.

   [acl-8112]
   This driver supports both TRIGNOW and TRIGCLK,
   but does not yet support DMA transfers.  It also supports
   both high (HG) and low (DG) versions of the card, though
   the HG version has been untested.

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
#include "8253.h"



#define PCL711_SIZE 16

#define PCL711_CTR0 0
#define PCL711_CTR1 1
#define PCL711_CTR2 2
#define PCL711_CTRCTL 3
#define PCL711_AD_LO 4
#define PCL711_DA0_LO 4
#define PCL711_AD_HI 5
#define PCL711_DA0_HI 5
#define PCL711_DI_LO 6
#define PCL711_DA1_LO 6
#define PCL711_DI_HI 7
#define PCL711_DA1_HI 7
#define PCL711_CLRINTR 8
#define PCL711_GAIN 9
#define PCL711_MUX 10
#define PCL711_MODE 11
#define PCL711_SOFTTRIG 12
#define PCL711_DO_LO 13
#define PCL711_DO_HI 14

static comedi_lrange range_pcl711b_ai = { 5, {
        BIP_RANGE( 5 ),
        BIP_RANGE( 2.5 ),
        BIP_RANGE( 1.25 ),
        BIP_RANGE( 0.625 ),
        BIP_RANGE( 0.3125 )
}};
static comedi_lrange range_acl8112hg_ai = { 12, {
	BIP_RANGE( 5 ),
	BIP_RANGE( 0.5 ),
	BIP_RANGE( 0.05 ),
	BIP_RANGE( 0.005 ),
	UNI_RANGE( 10 ),
	UNI_RANGE( 1 ),
	UNI_RANGE( 0.1 ),
	UNI_RANGE( 0.01 ),
	BIP_RANGE( 10 ),
	BIP_RANGE( 1 ),
	BIP_RANGE( 0.1 ),
	BIP_RANGE( 0.01 )
}};
static comedi_lrange range_acl8112dg_ai = { 9, {
        BIP_RANGE( 5 ),
        BIP_RANGE( 2.5 ),
        BIP_RANGE( 1.25 ),
        BIP_RANGE( 0.625 ),
	UNI_RANGE( 10 ),
	UNI_RANGE( 5 ),
	UNI_RANGE( 2.5 ),
	UNI_RANGE( 1.25 ),
	BIP_RANGE( 10 )
}};

typedef int bool;

/*
 * flags
 */

#define PCL711_TIMEOUT 100
#define PCL711_DRDY 0x10

int i8253_osc_base = 500;	/* 2 Mhz */

typedef struct {
	char *name;
	int is_pcl711b;
	int is_8112;
	int is_dg;
	int n_ranges;
	int n_aichan;
	int n_aochan;
	int maxirq;
	comedi_lrange * ai_range_type;
} boardtype;

static boardtype boardtypes[] =
{
	{"pcl711", 0, 0, 0, 5, 8, 1, 0, &range_bipolar5},
	{"pcl711b", 1, 0, 0, 5, 8, 1, 7, &range_pcl711b_ai},
	{"acl8112hg", 0, 1, 0, 12, 16, 2, 15, &range_acl8112hg_ai},
	{"acl8112dg", 0, 1, 1, 9, 16, 2, 15, &range_acl8112dg_ai},
};
#define n_boardtypes (sizeof(boardtypes)/sizeof(boardtype))
#define this_board ((boardtype *)dev->board_ptr)

static int pcl711_attach(comedi_device *dev,comedi_devconfig *it);
static int pcl711_detach(comedi_device *dev);
comedi_driver driver_pcl711={
	driver_name:	"pcl711",
	module:		THIS_MODULE,
	attach:		pcl711_attach,
	detach:		pcl711_detach,
	board_name:	boardtypes,
	num_names:	n_boardtypes,
	offset:		sizeof(boardtype),
};
COMEDI_INITCLEANUP(driver_pcl711);

typedef struct {
	int board;
	int adchan;
	int ntrig;
	int aip[8];
	int mode;
	lsampl_t ao_readback[2];
} pcl711_private;

#define devpriv ((pcl711_private *)dev->private)

static void pcl711_interrupt(int irq, void *d, struct pt_regs *regs)
{
	int lo, hi;
	int data;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;

	hi = inb(dev->iobase + PCL711_AD_HI);
	lo = inb(dev->iobase + PCL711_AD_LO);
	outb(0, dev->iobase + PCL711_CLRINTR);

	data = (hi << 8) | lo;

	if (!(--devpriv->ntrig)) {
		if (this_board->is_8112) {
			outb(1, dev->iobase + PCL711_MODE);
		} else {
			outb(0, dev->iobase + PCL711_MODE);
		}

		comedi_done(dev,s);
	}
}

static void pcl711_set_changain(comedi_device * dev, int chan)
{
	int chan_register;

	outb(CR_RANGE(chan), dev->iobase + PCL711_GAIN);
	
	chan_register=CR_CHAN(chan);

	if (this_board->is_8112) {

		/*
		 *  Set the correct channel.  The two channel banks are switched
		 *  using the mask value.
		 *  NB: To use differential channels, you should use mask = 0x30,
		 *  but I haven't written the support for this yet. /JJ
		 */

		if (chan_register >= 8){
			chan_register = 0x20 | (chan_register & 0x7);
		}else{
			chan_register |= 0x10;
		}
	} else {
		outb(chan_register, dev->iobase + PCL711_MUX);
	}
}

static int pcl711_ai_insn(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int i,n;
	int hi,lo;

	pcl711_set_changain(dev,insn->chanspec);

	/*
	   a sensible precaution to wait for the mux to
	   settle here.  is 10us enough?
	*/
	udelay(10);

	for(n=0;n<insn->n;n++){
		/*
		 *  Write the correct mode (software polling) and start polling by writing
		 *  to the trigger register
		 */
		outb(1, dev->iobase + PCL711_MODE);

		if (this_board->is_8112) {
		}else{
			outb(0, dev->iobase + PCL711_SOFTTRIG);
		}

		i=PCL711_TIMEOUT;
		while(--i){
			hi = inb(dev->iobase + PCL711_AD_HI);
			if (!(hi & PCL711_DRDY))
				goto ok;
		}
		rt_printk("comedi%d: pcl711: A/D timeout\n", dev->minor);
		return -ETIME;
	
ok:
		lo = inb(dev->iobase + PCL711_AD_LO);

		data[n] = ((hi & 0xf) << 8) | lo;
	}

	return n;
}

#ifdef CONFIG_COMEDI_TRIG
static int pcl711_ai_mode4(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	if (!this_board->is_pcl711b || dev->irq == 0)
		return -EINVAL;

	pcl711_set_changain(dev,it->chanlist[0]);

	/*
	 *  Set mode to "no internal trigger"
	 */
	outb(devpriv->mode | 3, dev->iobase + PCL711_MODE);

	return 0;
}
#endif


#ifdef CONFIG_COMEDI_TRIG
static int pcl711_ai_mode1(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int timer1,timer2;

	if (!dev->irq)
		return -EINVAL;

	pcl711_set_changain(dev,it->chanlist[0]);

	/*
	 *  Set timers
	 *	timer chip is an 8253, with timers 1 and 2
	 *	cascaded
	 *  0x74 = Select Counter 1 | LSB/MSB | Mode=2 | Binary
	 *        Mode 2 = Rate generator
	 *
	 *  0xb4 = Select Counter 2 | LSB/MSB | Mode=2 | Binary
	 */

	i8253_cascade_ns_to_timer(i8253_osc_base,&timer1,&timer2,&it->trigvar,TRIG_ROUND_NEAREST);

	outb(0x74, dev->iobase + PCL711_CTRCTL);
	outb(timer1 & 0xff, dev->iobase + PCL711_CTR1);
	outb((timer1 >> 8) & 0xff, dev->iobase + PCL711_CTR1);
	outb(0xb4, dev->iobase + PCL711_CTRCTL);
	outb(timer2 & 0xff, dev->iobase + PCL711_CTR2);
	outb((timer2 >> 8) & 0xff, dev->iobase + PCL711_CTR2);

	/* clear pending interrupts (just in case) */
	outb(0, dev->iobase + PCL711_CLRINTR);

	/*
	 *  Set mode to IRQ transfer
	 */
	outb(devpriv->mode | 6, dev->iobase + PCL711_MODE);

	return 0;
}
#endif

/*
   analog output
*/
static int pcl711_ao_insn(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int n;
	int chan = CR_CHAN(insn->chanspec);

	for(n=0;n<insn->n;n++){
		outb((data[n] & 0xff), dev->iobase + (chan ? PCL711_DA1_LO : PCL711_DA0_LO));
		outb((data[n] >> 8), dev->iobase + (chan ? PCL711_DA1_HI : PCL711_DA0_HI));

		devpriv->ao_readback[chan] = data[n];
	}

	return n;
}

static int pcl711_ao_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int n;
	int chan = CR_CHAN(insn->chanspec);

	for(n=0;n<insn->n;n++){
		data[n] = devpriv->ao_readback[chan];
	}

	return n;

}

/* Digital port read - Untested on 8112 */
static int pcl711_di_insn_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=2)return -EINVAL;

	data[1] = inb(dev->iobase + PCL711_DI_LO) |
	    (inb(dev->iobase + PCL711_DI_HI) << 8);

	return 2;
}

/* Digital port write - Untested on 8112 */
static int pcl711_do_insn_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=2)return -EINVAL;

	if(data[0]){
		s->state &= ~data[0];
		s->state |= data[0]&data[1];
	}
	if(data[0]&0x00ff)
		outb(s->state & 0xff, dev->iobase + PCL711_DO_LO);
	if(data[0]&0xff00)
		outb((s->state >> 8), dev->iobase + PCL711_DO_HI);

	data[1]=s->state;

	return 2;
}

/*  Free any resources that we have claimed  */
static int pcl711_detach(comedi_device * dev)
{
	printk("comedi%d: pcl711: remove\n", dev->minor);

	if (dev->irq)
		free_irq(dev->irq, dev);

	if (dev->iobase)
		release_region(dev->iobase, PCL711_SIZE);

	return 0;
}

/*  Initialization */
static int pcl711_attach(comedi_device * dev, comedi_devconfig * it)
{
	int ret;
	int iobase;
	int irq;
	comedi_subdevice *s;

	/* claim our I/O space */

	iobase = it->options[0];
	printk("comedi%d: pcl711: 0x%04x ", dev->minor, iobase);
	if (check_region(iobase, PCL711_SIZE) < 0) {
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase, PCL711_SIZE, "pcl711");
	dev->iobase = iobase;

	/* there should be a sanity check here */

	/* set up some name stuff */
	dev->board_name = this_board->name;

	/* grab our IRQ */
	irq = it->options[1];
	if (irq < 0 || irq > this_board->maxirq) {
		printk("irq out of range\n");
		return -EINVAL;
	}
	if (irq) {
		if (comedi_request_irq(irq, pcl711_interrupt, 0, "pcl711", dev)) {
			printk("unable to allocate irq %d\n", irq);
			return -EINVAL;
		} else {
			printk("( irq = %d )\n", irq);
		}
	}
	dev->irq = irq;

	dev->n_subdevices = 4;
	if((ret=alloc_subdevices(dev))<0)
		return ret;
	if((ret=alloc_private(dev,sizeof(pcl711_private)))<0)
		return ret;

	s = dev->subdevices + 0;
	/* AI subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = this_board->n_aichan;
	s->maxdata = 0xfff;
	s->len_chanlist = 1;
	s->range_table = this_board->ai_range_type;
	s->insn_read = pcl711_ai_insn;
#ifdef CONFIG_COMEDI_TRIG
	s->trig[1] = pcl711_ai_mode1;
	s->trig[4] = pcl711_ai_mode4;
#endif

	s++;
	/* AO subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = this_board->n_aochan;
	s->maxdata = 0xfff;
	s->len_chanlist = 1;
	s->range_table = &range_bipolar5;
	s->insn_write = pcl711_ao_insn;
	s->insn_read = pcl711_ao_insn_read;

	s++;
	/* 16-bit digital input */
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 16;
	s->maxdata = 1;
	s->len_chanlist = 16;
	s->range_table = &range_digital;
	s->insn_bits = pcl711_di_insn_bits;

	s++;
	/* 16-bit digital out */
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = 16;
	s->maxdata = 1;
	s->len_chanlist = 16;
	s->range_table = &range_digital;
	s->state=0;
	s->insn_bits = pcl711_do_insn_bits;

	/*
	   this is the "base value" for the mode register, which is
	   used for the irq on the PCL711
	 */
	if(this_board->is_pcl711b){
		devpriv->mode=(dev->irq<<4);
	}

	/* clear DAC */
	outb(0, dev->iobase + PCL711_DA0_LO);
	outb(0, dev->iobase + PCL711_DA0_HI);
	outb(0, dev->iobase + PCL711_DA1_LO);
	outb(0, dev->iobase + PCL711_DA1_HI);

	printk("\n");

	return 0;
}

