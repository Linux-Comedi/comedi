/*
 *	ii_pci20kc.c
 *	driver for Intelligent Instruments PCI-20001C carrier board
 *	and modules
 *
 * 	COMEDI - Linux Control and Measurement Device Interface
 *	Copyright (C) 2000 Markus Kempf <kempf@matsci.uni-sb.de>
 *			   25.2.2000	
 *
 *	Linux device driver for COMEDI
 *	Intelligent Instrumentation
 *	PCI-20001 C-2A Carrier Board
 *	PCI-20341 M-1A 16-Bit analog input module
 *				- differential
 *				- range (-5V - +5V)
 *				- 16 bit
 *	PCI-20006 M-2 16-Bit analog output module
 *				- ranges (-10V - +10V) (0V - +10V) (-5V - +5V)
 *				- 16 bit
 *	
 *	only ONE PCI-20341 module possible
 * 	only ONE PCI-20006 module possible
 *	no extern trigger implemented
 *	only 4 on-board differential channels supported
 *	only ONE di-port and ONE do-port supported instead of 4 digital ports
 *  	di-port == Port 0
 *	do-port == Port 1
 *
 *	The state of this driver is only a starting point for a complete
 *	COMEDI-driver. The final driver should support all features of the 
 *	carrier board and modules.
 *
 *	The test configuration:
 *
 *	kernel 2.2.13 with RTAI v1.1 (realtime not yet tested)
 *	COMEDI 0.7.39
 *	COMEDILIB 0.7.8
 *
 */

#include <linux/module.h>	/* modularer Kernel */
#include <linux/kernel.h>	/* printk() */
#include <linux/fs.h>		/* character device definition */
#include <linux/errno.h>	/* error codes */

#include <asm/io.h>			/* readb() ... */
#include <asm/uaccess.h>	/* copy_to/from_user */

#include "comedi_module.h"


#define PCI20000_ID				0x1d
#define PCI20341_ID    			0x77
#define PCI20006_ID      		0xe3
#define PCI20000_OFFSET 		0x100
#define PCI20000_MODULES		3
#define PCI20341_OPTION			0x00
#define PCI20000_DI_PORT		0	/* only ONE di-port is supported */
#define PCI20000_DO_PORT		1	/* only ONE do-port is supported */
#define PCI20341_REPMODE		0x00	/* single shot mode */
#define PCI20341_PACER			0x00	/* Hardware Pacer disabled */
#define PCI20341_CHAN_NR		2	/* number of input channels */
#define CHANNEL_READ			0x01
#define CHANNEL_WRITE			0x01



typedef struct { 
		unsigned int ai_mod_pos;			/* block address of ai module */
		unsigned int ao_mod_pos;			/* block address of ao module */
		comedi_lrange *ao_range_list[2];		/* range of channels of ao module */
		int dio_port[PCI20000_MODULES+1];	/* port address */
		int timebase;						/* timebase ai module */
		int settling_time;					/* settling time ai module */
		int ai_gain;						/* analog input gain */
		}pci20xxx_private;
		
#define devpriv ((pci20xxx_private *)dev->private)
#define CHAN (CR_CHAN(it->chanlist[0]))

static int pci20xxx_attach(comedi_device *dev, comedi_devconfig *it);
static int pci20xxx_detach(comedi_device *dev);
static int pci20xxx_recognize(char *name);

comedi_driver driver_pci20xxx={
		driver_name:	"pci20xxx",
		module:		&__this_module,
		attach:		pci20xxx_attach,
		detach:		pci20xxx_detach,
		recognize:	pci20xxx_recognize,
		};
		
static int pci20xxx_ai_mode0(comedi_device *dev, comedi_subdevice *s, comedi_trig *it);
static int pci20xxx_ao(comedi_device *dev, comedi_subdevice *s, comedi_trig *it);		
static int pci20xxx_di(comedi_device *dev, comedi_subdevice *s, comedi_trig *it);
static int pci20xxx_do(comedi_device *dev, comedi_subdevice *s, comedi_trig *it);

static int pci20xxx_init(comedi_device *dev);
static int pci20xxx_ai_init(comedi_device *dev);
static int pci20xxx_di_init(comedi_device *dev);
static int pci20xxx_do_init(comedi_device *dev);

static int pci20xxx_recognize(char *name)
	{
	if(!strcmp("pci20xxx",name))
		return 0;
	return -1;
	}

/*
  options[0]   Board base address
  options[1]   IRQ
  options[2]   Analog input gain configuration
                1
       		10
       		100
       		200
  options[3]   Analog output channel 0 range configuration
                 0 == bipolar 10  (-10V -- +10V)
                 1 == unipolar 10V  (0V -- +10V)
                 2 == bipolar 5V  (-5V -- +5V)
  options[4]   Analog output channel 1 range configuration
                 0 == bipolar 10  (-10V -- +10V)
                 1 == unipolar 10V  (0V -- +10V)
                 2 == bipolar 5V  (-5V -- +5V)
*/
static int pci20xxx_attach(comedi_device * dev, comedi_devconfig * it)
	{
	unsigned char i;
	int ret;
	int isthere = 0;
	comedi_subdevice *s;

	dev->n_subdevices = 4;
	if ((ret = alloc_subdevices(dev)) < 0)
		return ret;
	if ((ret = alloc_private(dev, sizeof(pci20xxx_private))) < 0)
		return ret;

	dev->iobase = it->options[0];
	
  	/* Check PCI-20001 C-2A Carrier Board ID */	
  	if((readb(dev->iobase) & PCI20000_ID) != PCI20000_ID)
		{
		printk("comedi%d: \n", dev->minor);
		printk("PCI-20001 C-2A Carrier Board at base=0x%05x not found !\n", dev->iobase);
		return -EINVAL;
		}

	/* looking for input module PCI-20341 M-1A*/	
	for (i=1; i <= PCI20000_MODULES; i++)
		{
		if ( readb(dev->iobase+i*PCI20000_OFFSET) == PCI20341_ID)
			{
			devpriv->ai_mod_pos = dev->iobase + i*PCI20000_OFFSET;			
			printk("comedi%d: module PCI-20341 opened!\n", dev->minor);
			isthere = 1;
			}	
		}
	if (!isthere)
		{
		printk("comedi%d: module PCI-20341 not found!\n", dev->minor);
		return -EINVAL;
		}

	/* look for output module PCI-20006 M-2 */
	for (i=1; i<= PCI20000_MODULES; i++)
		{
		if ( readb(dev->iobase+i*PCI20000_OFFSET) == PCI20006_ID)
			{
			devpriv->ao_mod_pos = dev->iobase + i*PCI20000_OFFSET;
			printk("comedi%d: module PCI-20006 opened!\n", dev->minor);
			isthere = 1;
			}
		}
	if (!isthere)
		{
		printk("comedi%d: module PCI-20006 not found!\n", dev->minor);
		return -EINVAL;
		}


	printk("comedi%d: pci20xxx: base=0x%05x\n", dev->minor, dev->iobase);

	dev->board_name = "pci20xxx";

	/* initialize pci20xxx_private */
	pci20xxx_init(dev);	

	/* options handling */
	switch (it->options[2])		/* gain */
		{
		case 1: 	devpriv->ai_gain = 0; 			/* gain 1 */
					devpriv->timebase = 0x00;		/* timebase ai-module */
					devpriv->settling_time = 0x58;	/* settling time ai-modlue */				
					break;
		case 10: 	devpriv->ai_gain = 1; 			/* gain 10 */
					devpriv->timebase = 0x00;		/* timebase ai-module */
					devpriv->settling_time = 0x58;	/* settling time ai-modlue */				
					break;		
		case 100:	devpriv->ai_gain = 2; 			/* gain 100 */
					devpriv->timebase = 0x00;		/* timebase ai-module */
					devpriv->settling_time = 0x93;	/* settling time ai-modlue */				
					break;		
		case 200: 	devpriv->ai_gain = 3; 			/* gain 200 */
					devpriv->timebase = 0x04;		/* timebase ai-module */
					devpriv->settling_time = 0x99;	/* settling time ai-modlue */				
					break;
		default: 	devpriv->ai_gain = 0; 			/* gain 1 */
					devpriv->timebase = 0x00;		/* timebase ai-module */
					devpriv->settling_time = 0x58;	/* settling time ai-modlue */				
					printk("comedi%d: ai default gain = 1 !\n", dev->minor);
					break;
		}
	switch	(it->options[3])
		{
		case 0: devpriv->ao_range_list[0] = &range_bipolar10; break;
		case 1: devpriv->ao_range_list[0] = &range_unipolar10; break;
		case 2: devpriv->ao_range_list[0] = &range_bipolar5; break;
		default:	devpriv->ao_range_list[0] = &range_bipolar10; break;
		}
	switch	(it->options[4])
		{
		case 0: devpriv->ao_range_list[1] = &range_bipolar10; break;
		case 1: devpriv->ao_range_list[1] = &range_unipolar10; break;
		case 2: devpriv->ao_range_list[1] = &range_bipolar5; break;
		default:	devpriv->ao_range_list[1] = &range_bipolar10; break;
		}		
		
	/* initialize the analog subdevices */
	/* ai subdevice */
	s = dev->subdevices + 0;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 4;
	s->trig[0] = pci20xxx_ai_mode0;
	s->maxdata = 0xffff;
	s->range_table = &range_bipolar5;
	pci20xxx_ai_init(dev);

	/* ao subdevice */	
	s = dev->subdevices + 1;
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = 2;
	s->trig[0] = pci20xxx_ao;
	s->maxdata = 0xffff;
	s->range_table_list = devpriv->ao_range_list;

	/* di subdevice */
	s = dev->subdevices + 2;
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 8;			/* every bit of a port is one channel */
	s->trig[0] = pci20xxx_di;
	s->maxdata = 1;
	s->range_table = &range_digital;
	pci20xxx_di_init(dev);

	/* do subdevice */
	s = dev->subdevices + 3;
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = 8;			/* every bit of a port is one channel */
	s->trig[0] = pci20xxx_do;
	s->maxdata = 1;
	s->state = 0;		
	s->range_table = &range_digital;
	pci20xxx_do_init(dev);

	return 1;
	}

static int pci20xxx_detach(comedi_device * dev)
	{
	printk("comedi%d: pci20xxx: remove\n", dev->minor);

	return 0;
	}

/* initialize pci20xxx_private */
static int pci20xxx_init(comedi_device *dev)
	{
	devpriv->dio_port[0] = 0x80;
	devpriv->dio_port[1] = 0x81;
	devpriv->dio_port[2] = 0xc0;
	devpriv->dio_port[3] = 0xc1;

	return 0;
	}


/* initialize digital output */
static int pci20xxx_do_init(comedi_device *dev)
	{
	unsigned char port;

	/* only ONE do port supported: port1 ! */
	switch (PCI20000_DO_PORT)
		{
		case 0:	port = readb(dev->iobase + 0x83);
			writeb( (port & 0x82)| 0x80, dev->iobase + 0x83);			/* port direction */
			writeb( 0xff, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);	/* write 0xff to port */
			port = readb(dev->iobase + 0x82);
			writeb( (port & 0xfe)| 0x04, dev->iobase + 0x82);			/* enable port */
			break;
		case 1:	port = readb(dev->iobase + 0x83);
			writeb( (port & 0x90)| 0x80, dev->iobase + 0x83);
			writeb( 0xff, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);
			port = readb(dev->iobase + 0x82);
			writeb( (port & 0xfd)| 0x08, dev->iobase + 0x82);
			break;
		case 2:	port = readb(dev->iobase + 0xc3);
			writeb( (port & 0x82)| 0x80, dev->iobase + 0xc3);
			writeb( 0xff, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);
			port = readb(dev->iobase + 0x82);
			writeb( (port & 0xef)| 0x40, dev->iobase + 0x82);
			break;
		case 3:	port = readb(dev->iobase + 0xc3);
			writeb( (port & 0x90)| 0x80, dev->iobase + 0xc3);
			writeb( 0xff, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);
			port = readb(dev->iobase + 0x82);
			writeb( (port & 0xdf)| 0x80, dev->iobase + 0x82);
			break;
		default: printk("comedi%d: wrong DO-port in pci20xxx_do_init()", dev->minor);
				return -EINVAL;
		}
	writeb( 0xff, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);		
	
	return 0;
	}

/* initialize digital input */
static int pci20xxx_di_init(comedi_device *dev)
	{
	unsigned char port;

	/* only ONE di port supported: port0 ! */
	switch (PCI20000_DI_PORT)
		{
		case 0:	port = readb(dev->iobase + 0x83);
			writeb( port|0x90, dev->iobase + 0x83);
			port = readb(dev->iobase + 0x82);
			writeb( port & 0xfa, dev->iobase + 0x82);
			break;
		case 1:	port = readb(dev->iobase + 0x83);
			writeb( port|0x82, dev->iobase + 0x83);
			port = readb(dev->iobase + 0x82);
			writeb( port & 0xf5, dev->iobase + 0x82);
			break;
		case 2:	port = readb(dev->iobase + 0xc3);
			writeb( port|0x90, dev->iobase + 0xc3);
			port = readb(dev->iobase + 0x82);
			writeb( port & 0xaf, dev->iobase + 0x82);
			break;
		case 3:	port = readb(dev->iobase + 0xc3);
			writeb( port|0x82, dev->iobase + 0xc3);
			port = readb(dev->iobase + 0x82);
			writeb( port & 0x5f, dev->iobase + 0x82);
			break;
		default: printk("comedi%d: wrong DI-port in pci20xxx_di_init()", dev->minor);
				return -EINVAL;
		}

	return 0;
	}

/* initialize analog input */
static int pci20xxx_ai_init(comedi_device *dev)
	{
	int option = PCI20341_OPTION;		/* depends on gain, trigger, repetition mode */

	option = devpriv->timebase | PCI20341_REPMODE;	
		
	writeb( 0x04, devpriv->ai_mod_pos + 0x10);						/* initialize Module */
	writeb( PCI20341_PACER, devpriv->ai_mod_pos + 0x01);				/* set Pacer */
	writeb( option, devpriv->ai_mod_pos + 0x11);					/* option register */
	writeb( devpriv->settling_time, devpriv->ai_mod_pos + 0x15); 	/* settling time counter */
	/* trigger not implemented */
	/* Burst mode disabled, only one channel is read */
	writeb( CHANNEL_READ, devpriv->ai_mod_pos + 0x13);				/* write number of input channels */

	return 0;
	}

static int pci20xxx_ao(comedi_device *dev, comedi_subdevice *s, comedi_trig *it)
	{
	int hi, lo;
	
	lo = (it->data[0] & 0xff);
	hi = ((it->data[0] >>8) & 0xff);
	 
	switch (CHAN)
		{
		case 0:	writeb( lo, devpriv->ao_mod_pos + 0x0d);		
				writeb( hi, devpriv->ao_mod_pos + 0x0d + 1);
				writeb( 0x00, devpriv->ao_mod_pos + 0x0b);
				break;
		case 1:	writeb( lo, devpriv->ao_mod_pos + 0x15);		
				writeb( hi, devpriv->ao_mod_pos + 0x15 + 1);
				writeb( 0x00, devpriv->ao_mod_pos + 0x13);
				break;		
		default: printk(" comedi%d: pci20xxx: ao channel Error!\n", dev->minor);
				 return -EINVAL;
		}				
	return CHANNEL_WRITE;	/* return value == number of written datas */
	}

static int pci20xxx_do(comedi_device *dev, comedi_subdevice *s, comedi_trig *it)
	{
	do_pack(&s->state, it);

	writeb(s->state, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);

	return it->n_chan;
	}

static int pci20xxx_ai_mode0(comedi_device *dev, comedi_subdevice *s, comedi_trig *it)
	{
	int	i=0, 
		j=0; 
	int lo, hi;
	writeb( (0x24|(devpriv->ai_gain<<3)|CHAN), devpriv->ai_mod_pos + 0x80);	
	writeb( 0x00, devpriv->ai_mod_pos + 0x1b);						/* reset settling time counter and trigger delay counter */
	writeb( 0x00, devpriv->ai_mod_pos + 0x19);	

	i = readb( devpriv->ai_mod_pos + 0x04);	/* generate Pacer */

	/* data polling isn't the niciest way to get the data, I know,
	 * but there are only 6 cycles (mean) and it is easier than
	 * the whole interrupt stuff 
	 */
	while ( (i<0x80) && j<100)		/* poll Interrupt Flag */
		{
		j++;
		i = readb(devpriv->ai_mod_pos + 0x12);
		}
	if (j>=100)
		{
		printk("comedi%d:  pci20xxx: AI interrupt polling exit !\n", dev->minor);
		return -EINVAL;
		}
	lo = readb(devpriv->ai_mod_pos + 0x02);
	hi = readb(devpriv->ai_mod_pos + 0x03);

	it->data[0] = lo + 0x100 * hi;
	
	return CHANNEL_READ;	/* return value == number of read datas !!*/
	}		

static int pci20xxx_di(comedi_device *dev, comedi_subdevice *s, comedi_trig *it)
	{
	unsigned int bits;

	bits = readb(dev->iobase + devpriv->dio_port[PCI20000_DI_PORT]);
	
	return di_unpack(bits, it);
	}	

#ifdef MODULE
int init_module(void)
	{
	comedi_driver_register(&driver_pci20xxx);

	return 0;
	}

void cleanup_module(void)
	{
	comedi_driver_unregister(&driver_pci20xxx);
	}
#endif	



