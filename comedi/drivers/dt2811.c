/*
   module/dt2811.c
   hardware driver for Data Translation DT2811

   COMEDI - Linux Control and Measurement Device Interface
   History:
   Base Version  - David A. Schleef <ds@stm.lbl.gov>
   December 1998 - Updated to work.  David does not have a DT2811
   board any longer so this was suffering from bitrot.
   Updated performed by ...

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
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <asm/io.h>
#include <comedi_module.h>

static char *driver_name = "dt2811";

comedi_lrange range_dt2811_pgh_ai_5_unipolar = { 4, {
	RANGE( 0,	5 ),
	RANGE( 0,	2.5 ),
	RANGE( 0,	1.25 ),
	RANGE( 0,	0.625 )
}};
comedi_lrange range_dt2811_pgh_ai_2_5_bipolar = { 4, {
	RANGE( -2.5,	2.5 ),
	RANGE( -1.25,	1.25 ),
	RANGE( -0.625,	0.625 ),
	RANGE( -0.3125,	0.3125 )
}};
comedi_lrange range_dt2811_pgh_ai_5_bipolar = { 4, {
	RANGE( -5,	5 ),
	RANGE( -2.5,	2.5 ),
	RANGE( -1.25,	1.25 ),
	RANGE( -0.625,	0.625 )
}};
comedi_lrange range_dt2811_pgl_ai_5_unipolar = { 4, {
	RANGE( 0,	5 ),
	RANGE( 0,	0.5 ),
	RANGE( 0,	0.05 ),
	RANGE( 0,	0.01 )
}};
comedi_lrange range_dt2811_pgl_ai_2_5_bipolar = { 4, {
	RANGE( -2.5,	2.5 ),
	RANGE( -0.25,	0.25 ),
	RANGE( -0.025,	0.025 ),
	RANGE( -0.005,	0.005 )
}};
comedi_lrange range_dt2811_pgl_ai_5_bipolar = { 4, {
	RANGE( -5,	5 ),
	RANGE( -0.5,	0.5 ),
	RANGE( -0.05,	0.05 ),
	RANGE( -0.01,	0.01 )
}};

/*

   0x00    ADCSR R/W  A/D Control/Status Register
   bit 7 - (R) 1 indicates A/D conversion done
   reading ADDAT clears bit
   (W) ignored
   bit 6 - (R) 1 indicates A/D error
   (W) ignored
   bit 5 - (R) 1 indicates A/D busy, cleared at end
   of conversion
   (W) ignored
   bit 4 - (R) 0
   (W)
   bit 3 - (R) 0
   bit 2 - (R/W) 1 indicates interrupts enabled
   bits 1,0 - (R/W) mode bits
   00  single conversion on ADGCR load
   01  continuous conversion, internal clock,
   (clock enabled on ADGCR load)
   10  continuous conversion, internal clock,
   external trigger
   11  continuous conversion, external clock,
   external trigger

   0x01    ADGCR R/W A/D Gain/Channel Register
   bit 6,7 - (R/W) gain select
   00  gain=1, both PGH, PGL models
   01  gain=2 PGH, 10 PGL
   10  gain=4 PGH, 100 PGL
   11  gain=8 PGH, 500 PGL
   bit 4,5 - reserved
   bit 3-0 - (R/W) channel select
   channel number from 0-15

   0x02,0x03 (R) ADDAT A/D Data Register
   (W) DADAT0 D/A Data Register 0
   0x02 low byte
   0x03 high byte

   0x04,0x05 (W) DADAT0 D/A Data Register 1

   0x06 (R) DIO0 Digital Input Port 0
   (W) DIO1 Digital Output Port 1

   0x07 TMRCTR (R/W) Timer/Counter Register
   bits 6,7 - reserved
   bits 5-3 - Timer frequency control (mantissa)
   543  divisor  freqency (kHz)
   000  1        600
   001  10       60
   010  2        300
   011  3        200
   100  4        150
   101  5        120
   110  6        100
   111  12       50
   bits 2-0 - Timer frequency control (exponent)
   210  multiply divisor/divide frequency by
   000  1
   001  10
   010  100
   011  1000
   100  10000
   101  100000
   110  1000000
   111  10000000

 */

#define DT2811_SIZE 8

#define DT2811_ADCSR 0
#define DT2811_ADGCR 1
#define DT2811_ADDATLO 2
#define DT2811_ADDATHI 3
#define DT2811_DADAT0LO 2
#define DT2811_DADAT0HI 3
#define DT2811_DADAT1LO 4
#define DT2811_DADAT1HI 5
#define DT2811_DIO 6
#define DT2811_TMRCTR 7

/*
 * flags
 */

/* ADCSR */

#define DT2811_ADDONE   0x80
#define DT2811_ADERROR  0x40
#define DT2811_ADBUSY   0x20
#define DT2811_CLRERROR 0x10
#define DT2811_INTENB   0x04
#define DT2811_ADMODE   0x03

static int dt2811_attach(comedi_device *dev,comedi_devconfig *it);
static int dt2811_detach(comedi_device *dev);
static int dt2811_recognize(char *name);
comedi_driver driver_dt2811={
	driver_name:	"dt2811",
	module:		&__this_module,
	attach:		dt2811_attach,
	detach:		dt2811_detach,
	recognize:	dt2811_recognize,
};

static int dt2811_ai(comedi_device * dev, comedi_subdevice * s, comedi_trig * it);
#if 0
static int dt2811_ai_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it);
#endif
static int dt2811_ao(comedi_device * dev, comedi_subdevice * s, comedi_trig * it);
static int dt2811_di(comedi_device * dev, comedi_subdevice * s, comedi_trig * it);
static int dt2811_do(comedi_device * dev, comedi_subdevice * s, comedi_trig * it);

enum { card_2811_pgh, card_2811_pgl };
typedef struct {
	int ntrig;
	int curadchan;
        enum {
	  adc_singleended, adc_diff, adc_pseudo_diff
        } adc_mux;
        enum {
	  adc_bipolar_5, adc_bipolar_2_5, adc_unipolar_5
        } adc_range;
        enum {
	  dac_bipolar_5, dac_bipolar_2_5, dac_unipolar_5
	} dac_range[2];
        comedi_lrange * range_type_list[2];
} dt2811_private;

#define devpriv ((dt2811_private *)dev->private)

static comedi_lrange *adc_range_types[][2] =
{
  /*     dt2811-pgh                       dt2811-pgl */
  { &range_dt2811_pgh_ai_5_bipolar,   &range_dt2811_pgl_ai_5_bipolar },
  { &range_dt2811_pgh_ai_2_5_bipolar, &range_dt2811_pgl_ai_2_5_bipolar },
  { &range_dt2811_pgh_ai_5_unipolar,  &range_dt2811_pgl_ai_5_unipolar }
};
static comedi_lrange *dac_range_types[] =
{
  &range_bipolar5,
  &range_bipolar2_5,
  &range_unipolar5
};

#define DT2811_TIMEOUT 5

static void dt2811_interrupt(int irq, void *d, struct pt_regs *regs)
{
	int lo, hi;
	int data;
	comedi_device *dev = d;

	lo = inb(dev->iobase + DT2811_ADDATLO);
	hi = inb(dev->iobase + DT2811_ADDATHI);

	data = lo + (hi << 8);

	if (!(--devpriv->ntrig)) {
		/* XXX */
		/* how to turn off acquisition */
		comedi_done(dev, dev->subdevices + 0);
	}
}

/*
  options[0]   Board base address
  options[1]   IRQ
  options[2]   Input configuration
                 0 == single-ended
                 1 == differential
                 2 == pseudo-differential
  options[3]   Analog input range configuration
                 0 == bipolar 5  (-5V -- +5V)
                 1 == bipolar 2.5V  (-2.5V -- +2.5V)
                 2 == unipolar 5V  (0V -- +5V)
  options[4]   Analog output 0 range configuration
                 0 == bipolar 5  (-5V -- +5V)
                 1 == bipolar 2.5V  (-2.5V -- +2.5V)
                 2 == unipolar 5V  (0V -- +5V)
  options[5]   Analog output 1 range configuration
                 0 == bipolar 5  (-5V -- +5V)
                 1 == bipolar 2.5V  (-2.5V -- +2.5V)
                 2 == unipolar 5V  (0V -- +5V)
*/
static int dt2811_recognize(char *name)
{
	if(!strcmp("dt2811-pgh", name))return card_2811_pgh;
	if(!strcmp("dt2811-pgl", name))return card_2811_pgl;

	return -1;
}

static int dt2811_attach(comedi_device * dev, comedi_devconfig * it)
{
	int i, irq, irqs;
	long flags;
	int ret;
	comedi_subdevice *s;
        int board = -1;

	dev->iobase = it->options[0];

	printk("comedi%d: dt2811: base=0x%04x\n", dev->minor, dev->iobase);

	if (check_region(dev->iobase, DT2811_SIZE) < 0) {
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(dev->iobase, DT2811_SIZE, driver_name);
	if (board == card_2811_pgh) {
	  dev->board_name = "dt2811-pgh";
        } else if (board == card_2811_pgl) {
	  dev->board_name = "dt2811-pgl";
        }
	dev->iosize = DT2811_SIZE;

#if 0
	outb(0, dev->iobase + DT2811_ADCSR);
	udelay(100);
	i = inb(dev->iobase + DT2811_ADDATLO);
	i = inb(dev->iobase + DT2811_ADDATHI);
#endif

#if 1
	irq = it->options[1];
	if (irq < 0) {
		save_flags(flags);
		sti();
		irqs = probe_irq_on();

		outb(DT2811_CLRERROR | DT2811_INTENB, dev->iobase + DT2811_ADCSR);
		outb(0, dev->iobase + DT2811_ADGCR);

		udelay(100);

		irq = probe_irq_off(irqs);
		restore_flags(flags);

		/*outb(DT2811_CLRERROR|DT2811_INTENB,dev->iobase+DT2811_ADCSR); */

		if (inb(dev->iobase + DT2811_ADCSR) & DT2811_ADERROR) {
			printk("error probing irq (bad) \n");
		}
		dev->irq = 0;
		if (irq > 0) {
			i = inb(dev->iobase + DT2811_ADDATLO);
			i = inb(dev->iobase + DT2811_ADDATHI);
			printk("(irq = %d)\n", irq);
			request_irq(irq, dt2811_interrupt, 0 * SA_INTERRUPT, driver_name, dev);
			dev->irq = irq;
		} else if (irq == 0) {
			printk("(no irq)\n");
		} else {
			printk("( multiple irq's -- this is bad! )\n");
		}
	}
#endif

	dev->n_subdevices = 4;
	if ((ret = alloc_subdevices(dev)) < 0)
		return ret;
	if ((ret = alloc_private(dev, sizeof(dt2811_private))) < 0)
		return ret;
	switch (it->options[2]) {
	  case 0: devpriv->adc_mux = adc_singleended; break;
	  case 1: devpriv->adc_mux = adc_diff; break;
	  case 2: devpriv->adc_mux = adc_pseudo_diff; break;
	  default:devpriv->adc_mux = adc_singleended; break;
	}
	switch (it->options[3]) {
	  case 0: devpriv->adc_range = adc_bipolar_5; break;
	  case 1: devpriv->adc_range = adc_bipolar_2_5; break;
	  case 2: devpriv->adc_range = adc_unipolar_5; break;
	  default:devpriv->adc_range = adc_bipolar_5; break;
	}
	switch (it->options[4]) {
	  case 0: devpriv->dac_range[0] = dac_bipolar_5; break;
	  case 1: devpriv->dac_range[0] = dac_bipolar_2_5; break;
	  case 2: devpriv->dac_range[0] = dac_unipolar_5; break;
	  default:devpriv->dac_range[0] = dac_bipolar_5; break;
	}
	switch (it->options[5]) {
	  case 0: devpriv->dac_range[1] = dac_bipolar_5; break;
	  case 1: devpriv->dac_range[1] = dac_bipolar_2_5; break;
	  case 2: devpriv->dac_range[1] = dac_unipolar_5; break;
	  default:devpriv->dac_range[1] = dac_bipolar_5; break;
	}

	s = dev->subdevices + 0;
	/* initialize the ADC subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = devpriv->adc_mux == adc_diff ? 8 : 16;
	s->trig[0] = dt2811_ai;
	s->maxdata = 0xfff;
	s->range_table = adc_range_types[devpriv->adc_range][board];

	s = dev->subdevices + 1;
	/* ao subdevice */
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = 2;
	s->trig[0] = dt2811_ao;
	s->maxdata = 0xfff;
        s->range_table_list = devpriv->range_type_list;
        devpriv->range_type_list[0] = dac_range_types[devpriv->dac_range[0]];
        devpriv->range_type_list[1] = dac_range_types[devpriv->dac_range[1]];

	s = dev->subdevices + 2;
	/* di subdevice */
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 8;
	s->trig[0] = dt2811_di;
	s->maxdata = 1;
	s->range_table = &range_digital;

	s = dev->subdevices + 3;
	/* do subdevice */
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE;
	s->n_chan = 8;
	s->trig[0] = dt2811_do;
	s->maxdata = 1;
	s->state = 0;
	s->range_table = &range_digital;

	return 0;
}


static int dt2811_detach(comedi_device * dev)
{
	printk("comedi%d: dt2811: remove\n", dev->minor);

	if (dev->irq) {
		free_irq(dev->irq, dev);
	}
	release_region(dev->iobase, dev->iosize);

	return 0;
}


static int dt2811_ai(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
  int i;
  for(i=0 ; i < it->n_chan ; i++) {
    int lo, hi;
    int chan;

    chan = CR_CHAN(it->chanlist[i]);

    outb(chan, dev->iobase + DT2811_ADGCR);
    while (inb(dev->iobase + DT2811_ADCSR) & DT2811_ADBUSY)
      udelay(25);
    lo = inb(dev->iobase + DT2811_ADDATLO);
    hi = inb(dev->iobase + DT2811_ADDATHI);

    it->data[i] = lo + 0x100 * hi;
  }
  return i;
}

#if 0
static int dt2811_ai(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{

	switch (it->mode) {
	case 0:
		return dt2811_ai_mode0(dev, s, it);
	case 1:
#if defined(FROM_DT2814)
		outb(it->chan | DT2814_ENB | (it->trigvar << 5), dev->iobase + DT2814_CSR);
#endif
	default:
		return -EINVAL;
	}
}

static int dt2811_ai_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int lo, hi;
	int chan;

	chan = CR_CHAN(it->chanlist[0]);

	outb(chan, dev->iobase + DT2811_ADGCR);
	while (inb(dev->iobase + DT2811_ADCSR) & DT2811_ADBUSY)
		udelay(25);
	lo = inb(dev->iobase + DT2811_ADDATLO);
	hi = inb(dev->iobase + DT2811_ADDATHI);

	it->data[0] = lo + 0x100 * hi;

	return 0;
}
#endif


#if 0
int dt2811_adtrig(kdev_t minor, comedi_adtrig * adtrig)
{
	comedi_device *dev = comedi_devices + minor;

	if (adtrig->n < 1)
		return 0;
	dev->curadchan = adtrig->chan;
	switch (dev->i_admode) {
	case COMEDI_MDEMAND:
		dev->ntrig = adtrig->n - 1;
		/*printk("dt2811: AD soft trigger\n"); */
		/*outb(DT2811_CLRERROR|DT2811_INTENB,dev->iobase+DT2811_ADCSR); *//* not neccessary */
		outb(dev->curadchan, dev->iobase + DT2811_ADGCR);
		do_gettimeofday(&trigtime);
		break;
	case COMEDI_MCONTS:
		dev->ntrig = adtrig->n;
		break;
	}

	return 0;
}
#endif

static int dt2811_ao(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
  int i;
  for(i=0 ; i < it->n_chan ; i++) {
	int lo, hi;
	int chan;

    chan = CR_CHAN(it->chanlist[i]);

    lo = (it->data[i] & 0xff);
    hi = (it->data[i] >> 8) & 0x0f;

	switch (chan) {
	case 0:
		outb(lo, dev->iobase + DT2811_DADAT0LO);
		outb(hi, dev->iobase + DT2811_DADAT0HI);
		break;
	case 1:
		outb(lo, dev->iobase + DT2811_DADAT1LO);
		outb(hi, dev->iobase + DT2811_DADAT1HI);
		break;
	}
  }
  return i;
}

static int dt2811_di(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	unsigned int bits;
	
	bits=inb(dev->iobase + DT2811_DIO);

	return di_unpack(bits,it);
}

static int dt2811_do(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	do_pack(&s->state,it);
	
	outb(s->state, dev->iobase + DT2811_DIO);

	return it->n_chan;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_dt2811);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_dt2811);
}
#endif
