/*
 * Driver for PCL725 and clones
 * David A. Schleef
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



#define PCL725_SIZE 2

#define PCL725_DO 0
#define PCL725_DI 1

static int pcl725_attach(comedi_device *dev,comedi_devconfig *it);
static int pcl725_detach(comedi_device *dev);
comedi_driver driver_pcl725={
	driver_name:	"pcl725",
	module:		THIS_MODULE,
	attach:		pcl725_attach,
	detach:		pcl725_detach,
};

static int pcl725_do(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int pcl725_di(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);

static int pcl725_do(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	do_pack(&s->state,it);
	
	outb(s->state,dev->iobase+PCL725_DO);

	return it->n_chan;
}

static int pcl725_di(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	unsigned int bits;
	
	bits=inb(dev->iobase+PCL725_DI);
	
	return di_unpack(bits,it);
}

static int pcl725_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;

	dev->iobase=it->options[0];
	printk("comedi%d: pcl725: 0x%04x ",dev->minor,dev->iobase);
	if(check_region(dev->iobase,PCL725_SIZE)<0){
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(dev->iobase,PCL725_SIZE,"pcl725");
	dev->board_name="pcl725";
	dev->iobase=dev->iobase;
	dev->iosize=PCL725_SIZE;
	dev->irq=0;

	dev->n_subdevices=2;

	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s=dev->subdevices+0;
	/* do */
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->maxdata=1;
	s->n_chan=8;
	s->trig[0]=pcl725_do;
	s->range_table=&range_digital;

	s=dev->subdevices+1;
	/* do */
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->maxdata=1;
	s->n_chan=8;
	s->trig[0]=pcl725_di;
	s->range_table=&range_digital;

	printk("\n");

	return 0;
}


static int pcl725_detach(comedi_device *dev)
{
	printk("comedi%d: pcl725: remove\n",dev->minor);
	
	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_pcl725);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_pcl725);
}
#endif
