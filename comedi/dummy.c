/*
 * dummy driver
 * David A. Schleef
 */

#include <linux/kernel.h>
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

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct{
	int data;
}dummy_private;
#define devpriv ((dummy_private *)dev->private)

static int dummy_attach(comedi_device *dev,comedi_devconfig *it);
static int dummy_detach(comedi_device *dev);
static int dummy_recognize(char *name);
comedi_driver driver_dummy={
	driver_name:	"dummy",
	attach:		dummy_attach,
	detach:		dummy_detach,
	recognize:	dummy_recognize,
};

static int dummy_ai(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int dummy_ao(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);

static int dummy_recognize(char *name)
{
	if(!strcmp("dummy",name))return 0;
	if(!strcmp("example",name))return 1;

	return -1;
}

static int dummy_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;

/* there is a *_init() function for each hardware driver installed.
   Each one is called in sequence.
   if it->name is a device that this hardware driver supports, we try
   to initialize it.  If everything works, return a 1.  If there is
   an error, return the error.  If we do not match it->name, return a
   0 so that the next *_init() function can be tried. */

/* *dev is zeroed except for dev->minor, which contains the minor
   number being used.  *it has configuration information (not really
   used here--see other drivers.)  this function is responsible for
   filling the rest of *dev.
 
   no longer true
 */

	printk("comedi%d: dummy: ",dev->minor);
	
	/* this is an appropriate place to put probe and initialization
	   code.  remember to fill in dev->iobase and friends. */
	dev->board_name="dummy";

	if(alloc_private(dev,sizeof(dummy_private))<0)
		return -ENOMEM;
#if DEBUG
	printk("malloc ok\n");
#endif

	dev->n_subdevices=2;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	/* analog input subdevice */
	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=1;
	s->maxdata=0xffff;
	s->range_table=&range_unknown;
	s->trig[0]=dummy_ai;
	
	/* analog output subdevice */
	s=dev->subdevices+1;
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=1;
	s->maxdata=0xffff;
	s->range_table=&range_unknown;
	s->trig[0]=dummy_ao;
	
	return 1;
}


static int dummy_detach(comedi_device *dev)
{
	printk("comedi%d: dummy: remove\n",dev->minor);
	
	return 0;
}

static int dummy_ai(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	it->data[0]=devpriv->data;

	return 0;
}

static int dummy_ao(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	devpriv->data=it->data[0];

	return 0;
}


#if 0
#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_dummy);

	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_dummy);
}
#endif
#endif

