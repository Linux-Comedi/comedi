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

#include <comedi_module.h>

#include <linux/module.h>	/* modularer Kernel */
#include <linux/kernel.h>	/* printk() */
#include <linux/errno.h>	/* error codes */

#include <asm/io.h>		/* readb() ... */


#define PCI20000_ID				0x1d
#define PCI20341_ID    			0x77
#define PCI20006_ID      		0xe3
#define PCI20xxx_EMPTY_ID		0

#define PCI20000_OFFSET 		0x100
#define PCI20000_MODULES		3

#define PCI20000_DI_PORT		0	/* only ONE di-port is supported */
#define PCI20000_DO_PORT		1	/* only ONE do-port is supported */
#define PCI20000_DIO_0			0x80
#define PCI20000_DIO_1			0x81
#define PCI20000_DIO_2			0xc0
#define PCI20000_DIO_3			0xc1

#define PCI20341_OPTION			0x00
#define PCI20341_REPMODE		0x00	/* single shot mode */
#define PCI20341_PACER			0x00	/* Hardware Pacer disabled */
#define PCI20341_CHAN_NR		2	/* number of input channels */



typedef union {
	int iobase;
	struct {
		int iobase;
		comedi_lrange *ao_range_list[2];	/* range of channels of ao module */
	}pci20006;
	struct {
		int iobase;
		int timebase;
		int settling_time;
		int ai_gain;
	}pci20341;
} pci20xxx_subdev_private;

typedef struct {
	pci20xxx_subdev_private subdev_private[PCI20000_MODULES];
} pci20xxx_private;

#define devpriv ((pci20xxx_private *)dev->private)
#define CHAN (CR_CHAN(it->chanlist[0]))

static int pci20xxx_attach(comedi_device * dev, comedi_devconfig * it);
static int pci20xxx_detach(comedi_device * dev);

comedi_driver driver_pci20xxx = {
	driver_name:"pci20xxx",
	module:&__this_module,
	attach:pci20xxx_attach,
	detach:pci20xxx_detach,
};

static int pci20006_init(comedi_device * dev,comedi_subdevice *s,
	int opt0,int opt1);
static int pci20341_init(comedi_device * dev,comedi_subdevice *s,
	int opt0,int opt1);
static int pci20xxx_dio_init(comedi_device * dev,comedi_subdevice *s);

/*
  options[0]   Board base address
  options[1]   IRQ
  options[2]   first option for module 1
  options[3]   second option for module 1
  options[4]   first option for module 2
  options[5]   second option for module 2
  options[6]   first option for module 3
  options[7]   second option for module 3

  options for PCI-20341M:
  first         Analog input gain configuration
                1
       		10
       		100
       		200

  options for PCI-20006M:
  first        Analog output channel 0 range configuration
                 0 == bipolar 10  (-10V -- +10V)
                 1 == unipolar 10V  (0V -- +10V)
                 2 == bipolar 5V  (-5V -- +5V)
  second       Analog output channel 1 range configuration
                 0 == bipolar 10  (-10V -- +10V)
                 1 == unipolar 10V  (0V -- +10V)
                 2 == bipolar 5V  (-5V -- +5V)
*/
static int pci20xxx_attach(comedi_device * dev, comedi_devconfig * it)
{
	unsigned char i;
	int ret;
	int id;
	comedi_subdevice *s;

	dev->n_subdevices = 4;
	if ((ret = alloc_subdevices(dev)) < 0)
		return ret;
	if ((ret = alloc_private(dev, sizeof(pci20xxx_private))) < 0)
		return ret;

	dev->iobase = it->options[0];
	dev->board_name = "pci20kc";

	/* Check PCI-20001 C-2A Carrier Board ID */
	if ((readb(dev->iobase) & PCI20000_ID) != PCI20000_ID) {
		printk("comedi%d: ii_pci20kc", dev->minor);
		printk(" PCI-20001 C-2A Carrier Board at base=0x%05x not found !\n", dev->iobase);
		return -EINVAL;
	}
	printk("comedi%d: pci20xxx: base=0x%05x\n", dev->minor, dev->iobase);

	for (i = 0; i < PCI20000_MODULES; i++) {
		s = dev->subdevices + i;
		id = readb(dev->iobase + (i+1) * PCI20000_OFFSET);
		s->private = devpriv->subdev_private + i;
		switch(id){
		case PCI20006_ID:
			pci20006_init(dev,s,it->options[2*i+2],it->options[2*i+3]);
			break;
		case PCI20341_ID:
			pci20341_init(dev,s,it->options[2*i+2],it->options[2*i+3]);
			break;
		default:
			printk("ii_pci20kc: unknown module code 0x%02x in slot %d: module disabled\n",
				id,i);
			/* fall through */
		case PCI20xxx_EMPTY_ID:
			s->type = COMEDI_SUBD_UNUSED;
			break;
		}
	}
//devpriv->ai_mod_pos = dev->iobase + i * PCI20000_OFFSET;

	/* initialize pci20xxx_private */
	pci20xxx_dio_init(dev,dev->subdevices + PCI20000_MODULES);

	return 1;
}

static int pci20xxx_detach(comedi_device * dev)
{
	printk("comedi%d: pci20xxx: remove\n", dev->minor);

	return 0;
}

/* pci20006m */

static int pci20006_ao(comedi_device * dev, comedi_subdevice * s, comedi_trig * it);

comedi_lrange *pci20006_range_list[] = {
	&range_bipolar10,
	&range_unipolar10,
	&range_bipolar5,
	&range_unipolar5
};

static int pci20006_init(comedi_device * dev,comedi_subdevice *s,
	int opt0,int opt1)
{
	pci20xxx_subdev_private *sdp = s->private;

	if(opt0<0 || opt0>=3)opt0=0;
	if(opt1<0 || opt1>=3)opt1=0;

	sdp->pci20006.ao_range_list[0] = pci20006_range_list[opt0];
	sdp->pci20006.ao_range_list[1] = pci20006_range_list[opt1];

	/* ao subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = 2;
	s->trig[0] = pci20006_ao;
	s->maxdata = 0xffff;
	s->range_table_list = sdp->pci20006.ao_range_list;

	return 0;
}

static int pci20006_ao(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	pci20xxx_subdev_private *sdp = s->private;
	int hi, lo;

	lo = (it->data[0] & 0xff);
	hi = ((it->data[0] >> 8) & 0xff);

	switch (CHAN) {
	case 0:
		writeb(lo, sdp->iobase + 0x0d);
		writeb(hi, sdp->iobase + 0x0d + 1);
		writeb(0x00, sdp->iobase + 0x0b);
		break;
	case 1:
		writeb(lo, sdp->iobase + 0x15);
		writeb(hi, sdp->iobase + 0x15 + 1);
		writeb(0x00, sdp->iobase + 0x13);
		break;
	default:
		printk(" comedi%d: pci20xxx: ao channel Error!\n", dev->minor);
		return -EINVAL;
	}
	return 1;
}

/* PCI20341M */

static int pci20341_mode0(comedi_device * dev, comedi_subdevice * s,
	comedi_trig * it);

static int pci20341_timebase[] = { 0x00, 0x00, 0x00, 0x04 };
static int pci20341_settling_time[] = { 0x58, 0x58, 0x93, 0x99 };

#ifdef SOFTWARE_SELECT_GAIN
/* use this if you want to set the AI gain in the mode0 function */
static comedi_lrange pci20341_range = { 4, {
	BIP_RANGE(5),
	BIP_RANGE(0.5),
	BIP_RANGE(0.05),
	BIP_RANGE(0.025),
}};
#else
static comedi_lrange range_bipolar0_5 = { 1, { BIP_RANGE(0.5) }};
static comedi_lrange range_bipolar0_05 = { 1, { BIP_RANGE(0.05) }};
static comedi_lrange range_bipolar0_025 = { 1, { BIP_RANGE(0.025) }};

static comedi_lrange *pci20341_ranges[] = {
	&range_bipolar5,
	&range_bipolar0_5,
	&range_bipolar0_05,
	&range_bipolar0_025,
};
#endif

static int pci20341_init(comedi_device * dev,comedi_subdevice *s,
	int opt0,int opt1)
{
	pci20xxx_subdev_private *sdp = s->private;
	int option = PCI20341_OPTION;	/* depends on gain, trigger, repetition mode */

	/* options handling */
	if(opt0<0 || opt0>3)opt0=0;
	sdp->pci20341.timebase = pci20341_timebase[opt0];
	sdp->pci20341.settling_time = pci20341_settling_time[opt0];

	/* ai subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 4;
	s->trig[0] = pci20341_mode0;
	s->maxdata = 0xffff;
	s->range_table = pci20341_ranges[opt0];

	option = sdp->pci20341.timebase | PCI20341_REPMODE;

	writeb(0x04, sdp->iobase + 0x10);	/* initialize Module */
	writeb(PCI20341_PACER, sdp->iobase + 0x01);	/* set Pacer */
	writeb(option, sdp->iobase + 0x11);	/* option register */
	writeb(sdp->pci20341.settling_time, sdp->iobase + 0x15);	/* settling time counter */
	/* trigger not implemented */
	/* Burst mode disabled, only one channel is read */
	writeb(1, sdp->iobase + 0x13);	/* write number of input channels */

	return 0;
}

static int pci20341_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	pci20xxx_subdev_private *sdp = s->private;
	int i = 0, j = 0;
	int lo, hi;

	writeb((0x24 | (sdp->pci20341.ai_gain << 3) | CHAN), sdp->iobase + 0x80);
	writeb(0x00, sdp->iobase + 0x1b);	/* reset settling time counter and trigger delay counter */
	writeb(0x00, sdp->iobase + 0x19);

	i = readb(sdp->iobase + 0x04);	/* generate Pacer */

	/* data polling isn't the niciest way to get the data, I know,
	 * but there are only 6 cycles (mean) and it is easier than
	 * the whole interrupt stuff 
	 */
	while ((i < 0x80) && j < 100) {	/* poll Interrupt Flag */
		j++;
		i = readb(sdp->iobase + 0x12);
	}
	if (j >= 100) {
		printk("comedi%d:  pci20xxx: AI interrupt polling exit !\n", dev->minor);
		return -EINVAL;
	}
	lo = readb(sdp->iobase + 0x02);
	hi = readb(sdp->iobase + 0x03);

	it->data[0] = lo + 0x100 * hi;

	return 1;
}

/* native DIO */

static void pci20xxx_dio_config(comedi_device * dev,comedi_subdevice *s);
static void pci20xxx_do(comedi_device * dev,comedi_subdevice *s);
static unsigned int pci20xxx_di(comedi_device * dev,comedi_subdevice *s);
static int pci20xxx_dio(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);

/* initialize pci20xxx_private */
static int pci20xxx_dio_init(comedi_device * dev,comedi_subdevice *s)
{

	s->type = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE | SDF_WRITEABLE;
	s->n_chan = 32;
	s->trig[0] = pci20xxx_dio;
	s->maxdata = 1;
	s->len_chanlist = 32;
	s->range_table = &range_digital;
	s->io_bits = 0;

	/* XXX digital I/O lines default to input on board reset. */
	pci20xxx_dio_config(dev,s);

	return 0;
}

static int pci20xxx_dio(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int mask,data_in;
	int i;

	if(it->flags & TRIG_CONFIG){
		int bits;

		for(i=0;i<it->n_chan;i++){
			mask = 1<<CR_CHAN(it->chanlist[i]);
			if(mask&0x000000ff){
				bits = 0x000000ff;
			}else if(mask & 0x0000ff00){
				bits = 0x0000ff00;
			}else if(mask & 0x00ff0000){
				bits = 0x00ff0000;
			}else {
				bits = 0xff000000;
			}
			if(it->data[i]){
				s->io_bits |= bits;
			}else{
				s->io_bits &= ~bits;
			}
		}
		pci20xxx_dio_config(dev,s);
	}else{
		if(it->flags&TRIG_WRITE){
			do_pack(&s->state,it);

			pci20xxx_do(dev,s);
		}else{
			data_in = pci20xxx_di(dev,s);

			di_unpack(data_in,it);
		}
	}

	return it->n_chan;
}

static void pci20xxx_dio_config(comedi_device * dev,comedi_subdevice *s)
{
	/* XXX it is not clear from the d[oi]_init functions how to
	   configure a group of bits for input or output */

	if(s->io_bits & 0x000000ff ){
		/* output */
	}else{
		/* input */
	}
	if(s->io_bits & 0x0000ff00 ){

	}else{

	}
	if(s->io_bits & 0x00ff0000 ){

	}else{

	}
	if(s->io_bits & 0xff000000 ){

	}else{

	}
}

#if 0
/* initialize digital output */
static int pci20xxx_do_init(comedi_device * dev)
{
	unsigned char port;

	/* only ONE do port supported: port1 ! */
	switch (PCI20000_DO_PORT) {
	case 0:
		port = readb(dev->iobase + 0x83);
		writeb((port & 0x82) | 0x80, dev->iobase + 0x83);	/* port direction */
		writeb(0xff, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);	/* write 0xff to port */
		port = readb(dev->iobase + 0x82);
		writeb((port & 0xfe) | 0x04, dev->iobase + 0x82);	/* enable port */
		break;
	case 1:
		port = readb(dev->iobase + 0x83);
		writeb((port & 0x90) | 0x80, dev->iobase + 0x83);
		writeb(0xff, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);
		port = readb(dev->iobase + 0x82);
		writeb((port & 0xfd) | 0x08, dev->iobase + 0x82);
		break;
	case 2:
		port = readb(dev->iobase + 0xc3);
		writeb((port & 0x82) | 0x80, dev->iobase + 0xc3);
		writeb(0xff, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);
		port = readb(dev->iobase + 0x82);
		writeb((port & 0xef) | 0x40, dev->iobase + 0x82);
		break;
	case 3:
		port = readb(dev->iobase + 0xc3);
		writeb((port & 0x90) | 0x80, dev->iobase + 0xc3);
		writeb(0xff, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);
		port = readb(dev->iobase + 0x82);
		writeb((port & 0xdf) | 0x80, dev->iobase + 0x82);
		break;
	default:
		printk("comedi%d: wrong DO-port in pci20xxx_do_init()", dev->minor);
		return -EINVAL;
	}
	writeb(0xff, dev->iobase + devpriv->dio_port[PCI20000_DO_PORT]);

	return 0;
}

/* initialize digital input */
static int pci20xxx_di_init(comedi_device * dev)
{
	unsigned char port;

	/* only ONE di port supported: port0 ! */
	switch (PCI20000_DI_PORT) {
	case 0:
		port = readb(dev->iobase + 0x83);
		writeb(port | 0x90, dev->iobase + 0x83);
		port = readb(dev->iobase + 0x82);
		writeb(port & 0xfa, dev->iobase + 0x82);
		break;
	case 1:
		port = readb(dev->iobase + 0x83);
		writeb(port | 0x82, dev->iobase + 0x83);
		port = readb(dev->iobase + 0x82);
		writeb(port & 0xf5, dev->iobase + 0x82);
		break;
	case 2:
		port = readb(dev->iobase + 0xc3);
		writeb(port | 0x90, dev->iobase + 0xc3);
		port = readb(dev->iobase + 0x82);
		writeb(port & 0xaf, dev->iobase + 0x82);
		break;
	case 3:
		port = readb(dev->iobase + 0xc3);
		writeb(port | 0x82, dev->iobase + 0xc3);
		port = readb(dev->iobase + 0x82);
		writeb(port & 0x5f, dev->iobase + 0x82);
		break;
	default:
		printk("comedi%d: wrong DI-port in pci20xxx_di_init()", dev->minor);
		return -EINVAL;
	}

	return 0;
}
#endif

static void pci20xxx_do(comedi_device * dev, comedi_subdevice * s)
{
	/* XXX if the channel is configured for input, does this
	   do bad things? */
	/* XXX it would be a good idea to only update the registers
	   that _need_ to be updated.  This requires changes to
	   comedi, however. */
	writeb((s->state>>0)&0xff, dev->iobase + PCI20000_DIO_0 );
	writeb((s->state>>8)&0xff, dev->iobase + PCI20000_DIO_1 );
	writeb((s->state>>16)&0xff, dev->iobase + PCI20000_DIO_2 );
	writeb((s->state>>24)&0xff, dev->iobase + PCI20000_DIO_3 );
}

static unsigned int pci20xxx_di(comedi_device * dev, comedi_subdevice * s)
{
	/* XXX same note as above */
	unsigned int bits;
	
	bits = readb(dev->iobase + PCI20000_DIO_0 );
	bits |= readb(dev->iobase + PCI20000_DIO_1 )<<8;
	bits |= readb(dev->iobase + PCI20000_DIO_2 )<<16;
	bits |= readb(dev->iobase + PCI20000_DIO_3 )<<24;

	return bits;
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
