/*
   The comedi version of this driver was modified from the
   original by ds.

 * das08jr
 * =======
 *
 * Linux device driver for Computer Boards CIO-DAS08/Jr-AO board.
 *
 * This board has 8 analog input lines and 2 analog output lines. Additionally
 * there are 8 digital outputs.
 * These lines are mapped onto minor 0 (digital output), 1 - 8 for analog
 * input, and 1 + 2 for analog output.
 *
 * Copyright (C) 1998 Jochen Küpper
 *
 * $Id$
 */


#include <comedi_module.h>

#include <linux/module.h>
#include <linux/config.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/version.h>

#include <asm/io.h>
#include <asm/system.h>


/* delay for A/D- and D/A-conversions [us] */
#define DAS08JR_DELAY ( 30 )


/* Module Name */
#define DAS08JR_NAME "das08jr"




/* driver version */
#if 0
static const char das08jr_version[] = "PCI-DAS08Jr-AO driver ($Id$)";
#endif





/* Port Offsets */
#define DAS08JR_SIZE                            ( 8 )	/* number of ports */
#define DAS08JR_ANALOG_IN_LSB                   ( 0 )	/* read */
#define DAS08JR_ANALOG_IN_MSB                   ( 1 )	/* read */
#define DAS08JR_START                           ( 1 )
#define DAS08JR_STATUS                          ( 2 )	/* read */
#define DAS08JR_MUX                             ( 2 )
#define DAS08JR_CONTROL                         ( 2 )
#define DAS08JR_DIGITAL_IN                      ( 3 )	/* read */
#define DAS08JR_DIGITAL_OUT                     ( 3 )
#define DAS08JR_ANALOG_OUT_LSB0                 ( 4 )
#define DAS08JR_ANALOG_OUT_MSB0                 ( 5 )
#define DAS08JR_ANALOG_OUT_LSB1                 ( 6 )
#define DAS08JR_ANALOG_OUT_MSB1                 ( 7 )



/* number of available lines */
#define DAS08JR_ANALOG_INPUTS   ( 8 )
#define DAS08JR_ANALOG_OUTPUTS  ( 2 )
#define DAS08JR_DIGITAL_LINES   ( 1 )
#define DAS08JR_MAX_LINE        ( 8 )	/* we have nine lines starting at zero */


static int das08jr_attach(comedi_device *dev,comedi_devconfig *it);
static int das08jr_detach(comedi_device *dev);
comedi_driver driver_das08jr={
	driver_name:	"das08jr",
	module:		&__this_module,
	attach:		das08jr_attach,
	detach:		das08jr_detach,
};


typedef struct{
	int last_do;
}das08jr_priv;
#define devpriv ((das08jr_priv *)dev->private)

static void release_resources(comedi_device *dev);



/**
 * Read digital lines
 *
 * @author Jochen Küpper
 * @version $Id$
 */
static int das08jr_di(comedi_device * dev,comedi_subdevice *s, comedi_trig * it)
{
	int chan;
	int data;
	int i;

	data = inb(dev->iobase + DAS08JR_DIGITAL_IN);

	for(i=0;i<it->n_chan;i++){
		chan=CR_CHAN(it->chanlist[i]);
		it->data[i]=(data>>chan)&1;
	}

	return i;
}



/**
 * write digital lines
 *
 * @author Jochen Küpper
 * @version $Id$
 */
static int das08jr_do(comedi_device * dev,comedi_subdevice *s, comedi_trig * it)
{
	int data;
	int chan;
	int mask;
	int i;

	data=devpriv->last_do;
	for(i=0;i<it->n_chan;i++){
		chan=CR_CHAN(it->chanlist[i]);
		mask=1<<chan;
		data&=~mask;
		if(it->data[i])
			data|=mask;
	}
	devpriv->last_do=data;

	outb(data, dev->iobase + DAS08JR_DIGITAL_OUT);

	return i;
}



/**
 * Read from analog line
 *
 * @author Jochen Küpper
 * @version $Id$
 */
static int das08jr_ai(comedi_device * dev, comedi_subdevice *s, comedi_trig * it)
{
	unsigned char lsb, msb;

	/* we alway read one short value */

	/* start daq */

	/* XXX do something here to trigger it */

	/* busy waiting for board to finish */
	udelay( DAS08JR_DELAY );

	/* read data and put it into user space */

	lsb = inb(DAS08JR_ANALOG_IN_LSB);
	msb = inb(DAS08JR_ANALOG_IN_MSB);

	it->data[0] = (((int) msb) << 4) + (lsb >> 4);

	return 1;
}


/*------------------------------*/

/**
 * Write to writeable analog line.
 *
 * We do not need to check wether this really is an writeable line, since open
 * cares about that an we only get called on lines 1 or 2.
 *
 * We simply put one value to the output ports, start D/A conversion, and return.
 *
 * @return On success the number of written bytes ( always 2 ),
 *         a negative error code otherwise.
 *
 * @author Jochen Küpper
 * @version $Id$
 */
static int das08jr_ao(comedi_device * dev, comedi_subdevice *s, comedi_trig * it)
{
	unsigned short val = it->data[0];
	int chan=CR_CHAN(it->chanlist[0]);

	if (chan){
		outb((unsigned char) val >> 4, DAS08JR_ANALOG_OUT_MSB1);
		outb((unsigned char) val << 4, DAS08JR_ANALOG_OUT_LSB1);
	} else {
		outb((unsigned char) val >> 4, DAS08JR_ANALOG_OUT_MSB0);
		outb((unsigned char) val << 4, DAS08JR_ANALOG_OUT_LSB0);
	}

	/* what does this do? */
	outb(0xff, DAS08JR_START);

	return 1;
}




/**
 * Register ports,
 *
 * @author Jochen Küpper
 * @version $Id$
 */
static int das08jr_attach(comedi_device * dev, comedi_devconfig * it)
{
	int result = 0;
	int io;
	comedi_subdevice *s;

	printk("comedi%d: das08jr: ", dev->minor);

	io = it->options[0];
#if 0
	if (0 == io) {
		/* should autoprobe here */
		io = DAS08JR_PORT;
	}
#endif
	if ((result = check_region(io, DAS08JR_SIZE)) < 0) {
		printk("Cannot register port memory at %x.\n", io);
		return result;
	}
	request_region(io, DAS08JR_SIZE, DAS08JR_NAME);

	dev->board_name = DAS08JR_NAME;
	dev->iobase = io;

	printk("Copyright (C) 1998 Jochen K\"upper.\n");

	dev->n_subdevices = 3;
	if((result=alloc_subdevices(dev))<0)
		return result;
	if((result=alloc_private(dev,sizeof(das08jr_priv)))<0)
		return result;

	s = dev->subdevices + 0;
	/* ai subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 8;
	s->maxdata = 0xfff;
	s->range_type = RANGE_unknown;
	s->trig[0] = das08jr_ai;

	s = dev->subdevices + 1;
	/* ao subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = 2;
	s->maxdata = 0xfff;
	s->range_type = RANGE_unknown;
	s->trig[0] = das08jr_ao;

	s = dev->subdevices + 2;
	/* di subdevice */
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 8;
	s->maxdata = 1;
	s->range_type = RANGE_digital;
	s->trig[0] = das08jr_di;

	s = dev->subdevices + 3;
	/* do subdevice */
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = 8;
	s->maxdata = 1;
	s->range_type = RANGE_digital;
	s->trig[0] = das08jr_do;

	return 0;
}


static void release_resources(comedi_device *dev)
{
	if (dev->iobase)
		release_region(dev->iobase, DAS08JR_SIZE);
}

/**
 * @author Jochen Küpper
 * @version $Id$
 */
static int das08jr_detach(comedi_device *dev)
{
	printk("comedi%d: das08jr: remove\n", dev->minor);

	release_resources(dev);

	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_das08jr);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_das08jr);
}
#endif
