/*
 * Driver for PCM3730 and clones
 * Blaine Lee
 * from pcl725 by David S.
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



#define PCM3730_SIZE 4   // consecutive io port addresses

#define PCM3730_DOA 0    // offsets for each port
#define PCM3730_DOB 2
#define PCM3730_DOC 3
#define PCM3730_DIA 0
#define PCM3730_DIB 2
#define PCM3730_DIC 3

static int pcm3730_attach(comedi_device *dev,comedi_devconfig *it);
static int pcm3730_detach(comedi_device *dev);
comedi_driver driver_pcm3730={
	driver_name:	"pcm3730",
	module:		THIS_MODULE,
	attach:		pcm3730_attach,
	detach:		pcm3730_detach,
};

static int pcm3730_doa(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int pcm3730_dob(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int pcm3730_doc(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int pcm3730_dia(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int pcm3730_dib(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int pcm3730_dic(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);

static int pcm3730_doa(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	do_pack(&s->state,it);
	outb(s->state,dev->iobase+PCM3730_DOA);
	return it->n_chan;
}

static int pcm3730_dob(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	do_pack(&s->state,it);
	outb(s->state,dev->iobase+PCM3730_DOB);
	return it->n_chan;
}

static int pcm3730_doc(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	do_pack(&s->state,it);
	outb(s->state,dev->iobase+PCM3730_DOC);
	return it->n_chan;
}

static int pcm3730_dia(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	unsigned int bits;
	bits=inb(dev->iobase+PCM3730_DIA);
	return di_unpack(bits,it);
}

static int pcm3730_dib(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	unsigned int bits;
	bits=inb(dev->iobase+PCM3730_DIB);
	return di_unpack(bits,it);
}

static int pcm3730_dic(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	unsigned int bits;
	bits=inb(dev->iobase+PCM3730_DIC);
	return di_unpack(bits,it);
}

static int pcm3730_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;

	dev->iobase=it->options[0];
	printk("comedi%d: pcm3730: 0x%04x ",dev->minor,dev->iobase);
	if(check_region(dev->iobase,PCM3730_SIZE)<0){
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(dev->iobase,PCM3730_SIZE,"pcm3730");
	dev->board_name="pcm3730";
	dev->iobase=dev->iobase;
	dev->iosize=PCM3730_SIZE;
	dev->irq=0;

	dev->n_subdevices=6;

	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->maxdata=1;
	s->n_chan=8;
	s->trig[0]=pcm3730_doa;
	s->range_table=&range_digital;

	s=dev->subdevices+1;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->maxdata=1;
	s->n_chan=8;
	s->trig[0]=pcm3730_dob;
	s->range_table=&range_digital;

	s=dev->subdevices+2;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->maxdata=1;
	s->n_chan=8;
	s->trig[0]=pcm3730_doc;
	s->range_table=&range_digital;

	s=dev->subdevices+3;
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->maxdata=1;
	s->n_chan=8;
	s->trig[0]=pcm3730_dia;
	s->range_table=&range_digital;

	s=dev->subdevices+4;
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->maxdata=1;
	s->n_chan=8;
	s->trig[0]=pcm3730_dib;
	s->range_table=&range_digital;

	s=dev->subdevices+5;
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->maxdata=1;
	s->n_chan=8;
	s->trig[0]=pcm3730_dic;
	s->range_table=&range_digital;

	printk("\n");

	return 0;
}


static int pcm3730_detach(comedi_device *dev)
{
	printk("comedi%d: pcm3730: remove\n",dev->minor);
	
	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_pcm3730);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_pcm3730);
}
#endif
