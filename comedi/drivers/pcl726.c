/*
    module/pcl726.c
    hardware driver for PC-LabCard PCL-726 and compatibles
      ACL-6126

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


/*
    Thanks to Circuit Specialists for having programming info (!) on
    their web page.  (http://www.cir.com/)
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


#undef PCL726_IRQ	/* no interrupt support (yet) */


#define PCL726_SIZE 16

#define PCL726_DAC0_HI 0
#define PCL726_DAC0_LO 1

#define PCL726_DO_HI 12
#define PCL726_DO_LO 13
#define PCL726_DI_HI 14
#define PCL726_DI_LO 15


static int pcl726_attach(comedi_device *dev,comedi_devconfig *it);
static int pcl726_detach(comedi_device *dev);
comedi_driver driver_pcl726={
	driver_name:	"pcl726",
	module:		THIS_MODULE,
	attach:		pcl726_attach,
	detach:		pcl726_detach,
};


typedef struct{
	int bipolar[6];
}pcl726_private;
#define devpriv ((pcl726_private *)dev->private)


static int pcl726_ao(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int hi,lo;
	int chan=CR_CHAN(it->chanlist[0]);

	lo=it->data[0]&0xff;
	hi=(it->data[0]>>8)&0xf;
	if(devpriv->bipolar[chan])hi^=0x8;
/*
	the programming info did not say which order to write bytes.
	switch the order of the next two lines if you get glitches.
*/
	outb(hi,dev->iobase+PCL726_DAC0_HI + 2*chan);
	outb(lo,dev->iobase+PCL726_DAC0_LO + 2*chan);
	
	return 1;
}

static int pcl726_di(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	unsigned int bits;
	
	bits=inb(dev->iobase+PCL726_DI_LO)|
		(inb(dev->iobase+PCL726_DI_HI)<<8);
	
	return di_unpack(bits,it);
}

static int pcl726_do(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	do_pack(&s->state,it);
	
	outb(s->state&0xff,dev->iobase+PCL726_DO_LO);
	outb((s->state>>8),dev->iobase+PCL726_DO_HI);
	
	return it->n_chan;
}

static int pcl726_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret;
	
	dev->iobase=it->options[0];
	printk("comedi%d: pcl726: 0x%04x ",dev->minor,dev->iobase);
	if(check_region(dev->iobase,PCL726_SIZE)<0){
		printk("I/O port conflict\n");
		return -EIO;
	}
	
	dev->board_name="pcl726";
	
#ifdef PCL726_IRQ
	irq=dev->options[1];
	if(!is_b)irq=0;
	if(irq<0 || irq==1 || irq>7){
		printk("irq out of range\n");
		return -EIO;
	}
	if(irq){
		if(request_irq(irq,pcl726_interrupt,SA_INTERRUPT,"pcl726",dev)){
			printk("unable to allocate irq %d\n",irq);
			irq=0;
		}else{
			printk("( irq = %d )\n",irq);
		}
	}
	dev->irq=irq;
#endif
	
	request_region(dev->iobase,PCL726_SIZE,"pcl726");
	dev->iosize=PCL726_SIZE;

	if((ret=alloc_private(dev,sizeof(pcl726_private)))<0)
		return -ENOMEM;

	dev->n_subdevices=3;
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	s=dev->subdevices+0;
	/* ao */
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=6;
	s->maxdata=0xfff;
	s->trig[0]=pcl726_ao;
	s->range_table=&range_unknown;	/* XXX */

	s=dev->subdevices+1;
	/* di */
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=16;
	s->maxdata=1;
	s->trig[0]=pcl726_di;
	s->range_table=&range_digital;

	s=dev->subdevices+2;
	/* do */
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=16;
	s->maxdata=1;
	s->trig[0]=pcl726_do;
	s->range_table=&range_digital;

	printk("\n");
	
	return 0;
}


static int pcl726_detach(comedi_device *dev)
{
	printk("comedi%d: pcl726: remove\n",dev->minor);
	
#ifdef PCL726_IRQ
	if(dev->irq){
		free_irq(dev->irq,dev);
	}
#endif

	release_region(dev->iobase,dev->iosize);

	return 0;
}



#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_pcl726);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_pcl726);
}
#endif
