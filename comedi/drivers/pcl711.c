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
#include <comedi_module.h>



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

/*
--BEGIN-RANGE-DEFS--
RANGE_pcl711b_ai
        -5      5
        -2.5    2.5
        -1.25   1.25
        -0.625  0.625
        -0.3125 0.3125
RANGE_acl8112hg_ai
        -5      5
        -0.5    0.5
        -0.05   0.05
        -0.005  0.005
        0       10
        0       1
        0       0.1
        0       0.01
        -10     10
        -1      1
        -0.1    0.1
        -0.01   0.01
RANGE_acl8112dg_ai
        -5      5
        -2.5    2.5
        -1.25   1.25
        -0.625  0.625
        0       10
        0       5
        0       2.5
        0       1.25
        -10     10
---END-RANGE-DEFS---
*/

static int pcl711_attach(comedi_device *dev,comedi_devconfig *it);
static int pcl711_detach(comedi_device *dev);
static int pcl711_recognize(char *name);
comedi_driver driver_pcl711={
	driver_name:	"pcl711",
	module:		&__this_module,
	attach:		pcl711_attach,
	detach:		pcl711_detach,
	recognize:	pcl711_recognize,
};

typedef int bool;

/*
 * flags
 */

#define PCL711_TIMEOUT 100
#define PCL711_DRDY 0x10


typedef struct {
	char *name;
	int is_pcl711b;
	int is_8112;
	int is_dg;
	int n_ranges;
	int n_aichan;
	int n_aochan;
	int maxirq;
	int ai_range_type;
	int ai_timer_type;
} boardtype;

static boardtype boardtypes[] =
{
	{"pcl711", 0, 0, 0, 5, 8, 1, 0, RANGE_bipolar5, 0},
	{"pcl711b", 1, 0, 0, 5, 8, 1, 7, RANGE_pcl711b_ai,
	 TIMER_acl8112},
	{"acl8112hg", 0, 1, 0, 12, 16, 2, 15, RANGE_acl8112hg_ai,
	 TIMER_acl8112},
	{"acl8112dg", 0, 1, 1, 9, 16, 2, 15, RANGE_acl8112dg_ai,
	 TIMER_acl8112},
};
#define n_boardtypes (sizeof(boardtypes)/sizeof(boardtype))

typedef struct {
	int board;
	int adchan;
	int ntrig;
	int aip[8];
	int mode;
} pcl711_private;

#define devpriv ((pcl711_private *)dev->private)
#define this_board (boardtypes+dev->board)

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
		s->busy = 0;

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

static int pcl711_ai_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int hi, lo, i;
	int nmax=40;	/* 1000 us / 25 us */
	int n;

	if(it->n<=nmax)nmax=it->n;

	pcl711_set_changain(dev,it->chanlist[0]);
	
	/*
	   a sensible precaution to wait for the mux to
	   settle here.  is 10us enough?
	*/
	udelay(10);

for(n=0;n<nmax;n++){
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
		udelay(5);
	}
	rt_printk("comedi%d: pcl711: A/D timeout\n", dev->minor);
	return -ETIME;
	
ok:
	lo = inb(dev->iobase + PCL711_AD_LO);

	it->data[n] = ((hi & 0xf) << 8) | lo;
}

	return n;
}

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


static int pcl711_ai_mode1(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
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


	outb(0x74, dev->iobase + PCL711_CTRCTL);
	outb((it->trigvar >> 16) & 0xff, dev->iobase + PCL711_CTR1);
	outb((it->trigvar >> 24) & 0xff, dev->iobase + PCL711_CTR1);
	outb(0xb4, dev->iobase + PCL711_CTRCTL);
	outb(it->trigvar & 0xff, dev->iobase + PCL711_CTR2);
	outb((it->trigvar >> 8) & 0xff, dev->iobase + PCL711_CTR2);

	/* clear pending interrupts (just in case) */
	outb(0, dev->iobase + PCL711_CLRINTR);

	/*
	 *  Set mode to IRQ transfer
	 */
	outb(devpriv->mode | 6, dev->iobase + PCL711_MODE);

	return 0;
}

/*
   analog output
*/
static int pcl711_ao(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int chan = CR_CHAN(it->chanlist[0]);
	sampl_t data = it->data[0];

	outb((data & 0xff), dev->iobase + (chan ? PCL711_DA1_LO : PCL711_DA0_LO));
	outb((data >> 8), dev->iobase + (chan ? PCL711_DA1_HI : PCL711_DA0_HI));

	return 0;
}

/* Digital port read - Untested on 8112 */
static int pcl711_di(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int data;
	int chan;
	int i;

	data = inb(dev->iobase + PCL711_DI_LO) |
	    (inb(dev->iobase + PCL711_DI_HI) << 8);

	for(i=0;i<it->n_chan;i++){
		chan=CR_CHAN(it->chanlist[i]);
		it->data[i]=(data>>chan)&1;
	}

	return it->n_chan;
}

/* Digital port write - Untested on 8112 */
static int pcl711_do(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int mask, data;
	int chan;
	int i;

	data=s->state;
	for(i=0;i<it->n_chan;i++){
		chan=CR_CHAN(it->chanlist[i]);
		mask=(1<<chan);
		data &= ~mask;
		if(it->data[i])
			data |= mask;
	}
	outb(data & 0xff, dev->iobase + PCL711_DO_LO);
	outb((data >> 8), dev->iobase + PCL711_DO_HI);
	s->state = data;

	return it->n_chan;
}

/*  Free any resources that we have claimed  */
static void free_resources(comedi_device * dev)
{
	if (dev->irq)
		free_irq(dev->irq, dev);

	if (dev->iobase)
		release_region(dev->iobase, dev->iosize);
}

static int pcl711_recognize(char *name)
{
	int i;

	for (i = 0; i < n_boardtypes; i++) {
		if (!strcmp(boardtypes[i].name, name)) {
			return i;
		}
	}
	return -1;
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
	request_region(dev->iobase, PCL711_SIZE, "pcl711");
	dev->iobase = iobase;
	dev->iosize = PCL711_SIZE;

	/* there should be a sanity check here */

	/* set up some name stuff */
	dev->board_name = boardtypes[dev->board].name;

	/* grab our IRQ */
	irq = it->options[1];
	if (irq < 0 || irq > boardtypes[dev->board].maxirq) {
		printk("irq out of range\n");
		free_resources(dev);
		return -EINVAL;
	}
	if (irq) {
		if (request_irq(irq, pcl711_interrupt, SA_INTERRUPT, "pcl711", dev)) {
			printk("unable to allocate irq %d\n", irq);
			free_resources(dev);
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
	s->timer_type = this_board->ai_timer_type;
	s->range_type = this_board->ai_range_type;
	s->trig[0] = pcl711_ai_mode0;
	s->trig[1] = pcl711_ai_mode1;
	s->trig[4] = pcl711_ai_mode4;

	s++;
	/* AO subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = this_board->n_aochan;
	s->maxdata = 0xfff;
	s->len_chanlist = 1;
	s->range_type = RANGE_bipolar5;
	s->trig[0] = pcl711_ao;

	s++;
	/* 16-bit digital input */
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 16;
	s->maxdata = 1;
	s->len_chanlist = 16;
	s->range_type = RANGE_digital;
	s->trig[0] = pcl711_di;

	s++;
	/* 16-bit digital out */
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = 16;
	s->maxdata = 1;
	s->len_chanlist = 16;
	s->range_type = RANGE_digital;
	s->state=0;
	s->trig[0] = pcl711_do;

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


/*
 *  Removes device
 */

static int pcl711_detach(comedi_device * dev)
{
	printk("comedi%d: pcl711: remove\n", dev->minor);

	free_resources(dev);

	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_pcl711);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_pcl711);
}
#endif
