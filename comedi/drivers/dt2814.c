/*
    module/dt2814.c
    hardware driver for Data Translation DT2814

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1998 David A. Schleef <ds@stm.lbl.gov>

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
#include <linuxcomedidev.h>



#define DT2814_SIZE 2

#define DT2814_CSR 0
#define DT2814_DATA 1

/*
 * flags
 */

#define DT2814_FINISH 0x80
#define DT2814_ERR 0x40
#define DT2814_BUSY 0x20
#define DT2814_ENB 0x10
#define DT2814_CHANMASK 0x0f

static int dt2814_attach(comedi_device *dev,comedi_devconfig *it);
static int dt2814_detach(comedi_device *dev);
comedi_driver driver_dt2814={
	driver_name:	"dt2814",
	module:		THIS_MODULE,
	attach:		dt2814_attach,
	detach:		dt2814_detach,
};

static void dt2814_interrupt(int irq,void *dev,struct pt_regs * regs);

typedef struct{
	int ntrig;
	int curadchan;
}dt2814_private;
#define devpriv ((dt2814_private *)dev->private)


#define DT2814_TIMEOUT 100

static int dt2814_ai_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int i,hi,lo;
	int chan;

	chan=CR_CHAN(it->chanlist[0]);

	outb(chan,dev->iobase+DT2814_CSR);
	for(i=0;i<DT2814_TIMEOUT;i++){
		if(inb(dev->iobase+DT2814_CSR)&DT2814_FINISH)
			break;
	}
	hi=inb(dev->iobase+DT2814_DATA);
	lo=inb(dev->iobase+DT2814_DATA);

	it->data[0]=(hi<<4)|(lo>>4);
	
	return 1;
}

static int dt2814_ai_mode1(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int chan;

	chan=CR_CHAN(it->chanlist[0]);

	devpriv->ntrig=it->n;
	outb(chan|DT2814_ENB|(it->trigvar<<5),
		dev->iobase+DT2814_CSR);
	
	return 0;
}

static int dt2814_attach(comedi_device *dev,comedi_devconfig *it)
{
	int i,irq;
	int ret;
	comedi_subdevice *s;

	dev->iobase=it->options[0];
	printk("comedi%d: dt2814: 0x%04x ",dev->minor,dev->iobase);
	if(check_region(dev->iobase,DT2814_SIZE)<0){
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(dev->iobase,DT2814_SIZE,"dt2814");
	dev->iobase=dev->iobase;
	dev->iosize=DT2814_SIZE;

	outb(0,dev->iobase+DT2814_CSR);
	udelay(100);
	if(inb(dev->iobase+DT2814_CSR)&DT2814_ERR){
		printk("reset error (fatal)\n");
		return -EIO;
	}
	i=inb(dev->iobase+DT2814_DATA);
	i=inb(dev->iobase+DT2814_DATA);

	irq=it->options[1];
#if 0
	if(irq<0){
		save_flags(flags);
		sti();
		irqs=probe_irq_on();
	
		outb(0,dev->iobase+DT2814_CSR);

		udelay(100);

		irq=probe_irq_off(irqs);
		restore_flags(flags);
		if(inb(dev->iobase+DT2814_CSR)&DT2814_ERR){
			printk("error probing irq (bad) \n");
		}

		i=inb(dev->iobase+DT2814_DATA);
		i=inb(dev->iobase+DT2814_DATA);
	}
#endif
	dev->irq=0;
	if(irq>0){
		printk("( irq = %d )\n",irq);
		request_irq(irq,dt2814_interrupt,0*SA_INTERRUPT,"dt2814",dev);
		dev->irq=irq;
	}else if(irq==0){
		printk("(no irq)\n");
	}else{
		printk("(probe returned multiple irqs--bad)\n");
	}
	
	dev->n_subdevices=1;
	if((ret=alloc_subdevices(dev))<0)
		return ret;
	if((ret=alloc_private(dev,sizeof(dt2814_private)))<0)
		return ret;

	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=16;			/* XXX */
	s->len_chanlist=1;
	s->trig[0]=dt2814_ai_mode0;
	s->trig[1]=dt2814_ai_mode1;
	s->maxdata=0xfff;
	s->range_table=&range_unknown;	/* XXX */

	return 0;
}


static int dt2814_detach(comedi_device *dev)
{
	printk("comedi%d: dt2814: remove\n",dev->minor);
	
	if(dev->irq){
		free_irq(dev->irq,dev);
	}
	release_region(dev->iobase,dev->iosize);

	return 0;
}


static void dt2814_interrupt(int irq,void *d,struct pt_regs * regs)
{
        int lo,hi;
        comedi_device *dev=d;
	int data;

        hi=inb(dev->iobase+DT2814_DATA);
        lo=inb(dev->iobase+DT2814_DATA);

        data=(hi<<4)|(lo>>4);

	if(!(--devpriv->ntrig)){
		int flags,i;

		outb(0,dev->iobase+DT2814_CSR);
		/* note: turning off timed mode triggers another
			sample. */

		save_flags(flags);
		cli();	/* FIXME */
		for(i=0;i<DT2814_TIMEOUT;i++){
			if(inb(dev->iobase+DT2814_CSR)&DT2814_FINISH)
				break;
		}
		inb(dev->iobase+DT2814_DATA);
		inb(dev->iobase+DT2814_DATA);
		restore_flags(flags);
	
		comedi_done(dev,dev->subdevices);
	}
}


#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_dt2814);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_dt2814);
}
#endif
