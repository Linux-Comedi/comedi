/*
   module/multiq3.c
   hardware driver for Quanser Consulting MultiQ-3 board

   COMEDI - Linux Control and Measurement Device Interface
   Copyright (C) 1999 Anders Blomdell <anders.blomdell@control.lth.se>

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
#include <linux/comedidev.h>


#define MULTIQ3_SIZE 16

/*
 * MULTIQ-3 port offsets
 */
#define MULTIQ3_DIGIN_PORT 0
#define MULTIQ3_DIGOUT_PORT 0
#define MULTIQ3_DAC_DATA 2
#define MULTIQ3_AD_DATA 4
#define MULTIQ3_AD_CS 4
#define MULTIQ3_STATUS 6
#define MULTIQ3_CONTROL 6
#define MULTIQ3_CLK_DATA 8
#define MULTIQ3_ENC_DATA 12
#define MULTIQ3_ENC_CONTROL 14

/*
 * flags for CONTROL register
 */
#define MULTIQ3_AD_MUX_EN      0x0040
#define MULTIQ3_AD_AUTOZ       0x0080
#define MULTIQ3_AD_AUTOCAL     0x0100
#define MULTIQ3_AD_SH          0x0200
#define MULTIQ3_AD_CLOCK_4M    0x0400
#define MULTIQ3_DA_LOAD                0x1800

#define MULTIQ3_CONTROL_MUST    0x0600

/*
 * flags for STATUS register
 */
#define MULTIQ3_STATUS_EOC      0x008
#define MULTIQ3_STATUS_EOC_I    0x010

/*
 * flags for encoder control
 */
#define MULTIQ3_CLOCK_DATA      0x00
#define MULTIQ3_CLOCK_SETUP     0x18
#define MULTIQ3_INPUT_SETUP     0x41
#define MULTIQ3_QUAD_X4         0x38
#define MULTIQ3_BP_RESET        0x01
#define MULTIQ3_CNTR_RESET      0x02
#define MULTIQ3_TRSFRPR_CTR     0x08
#define MULTIQ3_TRSFRCNTR_OL    0x10
#define MULTIQ3_EFLAG_RESET     0x06

#define MULTIQ3_TIMEOUT 30

static int multiq3_attach(comedi_device *dev,comedi_devconfig *it);
static int multiq3_detach(comedi_device *dev);
comedi_driver driver_multiq3={
	driver_name:	"multiq3",
	module:		THIS_MODULE,
	attach:		multiq3_attach,
	detach:		multiq3_detach,
};


static int multiq3_ai(comedi_device *dev, comedi_subdevice *s, comedi_trig *it)
{
  int i, hi, lo;
  int chan;
  int data;
  int status, control;

  chan = CR_CHAN(it->chanlist[0]);
  control = MULTIQ3_CONTROL_MUST | MULTIQ3_AD_MUX_EN | (chan<<3);
  outw(control, dev->iobase+MULTIQ3_CONTROL);
  for (i = 0; i < MULTIQ3_TIMEOUT; i++) {
    status = inw(dev->iobase+MULTIQ3_STATUS);
    if(status & MULTIQ3_STATUS_EOC) {
      break;
    }
    udelay(10);
  }
  if(i == MULTIQ3_TIMEOUT){
    rt_printk("multiq3: timeout\n");
    return -ETIME;
  }
  outw(0, dev->iobase+MULTIQ3_AD_CS);
  for (i = 0; i < MULTIQ3_TIMEOUT; i++) {
    status = inw(dev->iobase+MULTIQ3_STATUS);
    if(status & MULTIQ3_STATUS_EOC_I) {
      break;
    }
    udelay(10);
  }
  hi = inb(dev->iobase + MULTIQ3_AD_CS) &0xff;
  lo = inb(dev->iobase + MULTIQ3_AD_CS) &0xff;

  data = (((hi << 8) | lo) + 0x1000) & 0x1fff;
  it->data[0]=data;

  return 1;
}

static int multiq3_ao(comedi_device *dev, comedi_subdevice *s, comedi_trig *it)
{
  int chan, control;

  chan=CR_CHAN(it->chanlist[0]);
  control = MULTIQ3_CONTROL_MUST | MULTIQ3_DA_LOAD | chan;
  outw(control, dev->iobase+MULTIQ3_CONTROL);
  outw(it->data[0], dev->iobase+MULTIQ3_DAC_DATA);
  control = MULTIQ3_CONTROL_MUST;
  outw(control, dev->iobase+MULTIQ3_CONTROL);
  return 1;
}

static int multiq3_di(comedi_device *dev, comedi_subdevice *s, comedi_trig *it)
{
  unsigned int bits;

  bits = inw(dev->iobase + MULTIQ3_DIGIN_PORT);

  return di_unpack(bits,it);
}

static int multiq3_do(comedi_device *dev, comedi_subdevice *s, comedi_trig *it)
{
  do_pack(&s->state,it);

  outw(s->state, dev->iobase + MULTIQ3_DIGOUT_PORT);

  return it->n_chan;
}

static int multiq3_ei(comedi_device *dev, comedi_subdevice *s, comedi_trig *it)
{
  int b1, b2, b3;
  int chan;
  int data;
  int control;

  chan = CR_CHAN(it->chanlist[0]);
  control = MULTIQ3_CONTROL_MUST | MULTIQ3_AD_MUX_EN | (chan<<3);
  outw(control, dev->iobase+MULTIQ3_CONTROL);
  outb(MULTIQ3_BP_RESET, dev->iobase+MULTIQ3_ENC_CONTROL);
  outb(MULTIQ3_TRSFRCNTR_OL, dev->iobase+MULTIQ3_ENC_CONTROL);
  b1 = inb(dev->iobase+MULTIQ3_ENC_DATA);
  b2 = inb(dev->iobase+MULTIQ3_ENC_DATA);
  b3 = inb(dev->iobase+MULTIQ3_ENC_DATA);

  data =  (((b3<<16) | (b2 << 8) | (b1)) + 0x800000) & 0xffffff;
  ((lsampl_t*)(it->data))[0] = data;

  return 1;
}

static void encoder_reset(comedi_device *dev) {
  int chan;
  for (chan = 0 ; chan < dev->subdevices[4].n_chan ; chan++) {
    int control = MULTIQ3_CONTROL_MUST | MULTIQ3_AD_MUX_EN | (chan<<3);
    outw(control, dev->iobase+MULTIQ3_CONTROL);
    outb(MULTIQ3_EFLAG_RESET, dev->iobase+MULTIQ3_ENC_CONTROL);
    outb(MULTIQ3_BP_RESET, dev->iobase+MULTIQ3_ENC_CONTROL);
    outb(MULTIQ3_CLOCK_DATA, dev->iobase+MULTIQ3_ENC_DATA);
    outb(MULTIQ3_CLOCK_SETUP, dev->iobase+MULTIQ3_ENC_CONTROL);
    outb(MULTIQ3_INPUT_SETUP, dev->iobase+MULTIQ3_ENC_CONTROL);
    outb(MULTIQ3_QUAD_X4, dev->iobase+MULTIQ3_ENC_CONTROL);
    outb(MULTIQ3_CNTR_RESET, dev->iobase+MULTIQ3_ENC_CONTROL);
  }
}

/*
   options[0] - I/O port
   options[1] - irq
   options[2] - number of encoder chips installed
 */

static int multiq3_attach(comedi_device * dev, comedi_devconfig * it)
{
  int result = 0;
  int iobase;
      int irq;
      comedi_subdevice *s;

    iobase = it->options[0];
    printk("comedi%d: multiq3: 0x%04x ", dev->minor, iobase);
    if (check_region(iobase, MULTIQ3_SIZE) < 0) {
      printk("comedi%d: I/O port conflict\n", dev->minor);
      return -EIO;
    }

      request_region(dev->iobase, MULTIQ3_SIZE, "multiq3");
      dev->iobase = iobase;

      irq = it->options[1];
      if (irq > 0) {
       printk("comedi%d: irq = %d ignored\n", dev->minor, irq);
      } else if(irq == 0) {
       printk("comedi%d: no irq\n", dev->minor);
      }
      dev->board_name = "multiq3";
      dev->n_subdevices = 5;
      result = alloc_subdevices(dev);
      if(result<0)return result;

      s = dev->subdevices + 0;
      /* ai subdevice */
      s->type = COMEDI_SUBD_AI;
      s->subdev_flags = SDF_READABLE;
      s->n_chan = 8;
      s->trig[0] = multiq3_ai;
      s->maxdata = 0x1fff;
      s->range_table = &range_bipolar5;

      s = dev->subdevices + 1;
      /* ao subdevice */
      s->type = COMEDI_SUBD_AO;
      s->subdev_flags = SDF_WRITEABLE;
      s->n_chan = 8;
      s->trig[0] = multiq3_ao;
      s->maxdata = 0xfff;
      s->range_table = &range_bipolar5;

      s = dev->subdevices + 2;
      /* di subdevice */
      s->type = COMEDI_SUBD_DI;
      s->subdev_flags = SDF_READABLE;
      s->n_chan = 16;
      s->trig[0] = multiq3_di;
      s->maxdata = 1;
      s->range_table = &range_digital;

      s = dev->subdevices + 3;
      /* do subdevice */
      s->type = COMEDI_SUBD_DO;
      s->subdev_flags = SDF_WRITEABLE;
      s->n_chan = 16;
      s->trig[0] = multiq3_do;
      s->maxdata = 1;
      s->range_table = &range_digital;
      s->state = 0;

      s = dev->subdevices + 4;
      /* encoder (counter) subdevice */
      s->type = COMEDI_SUBD_COUNTER;
      s->subdev_flags = SDF_READABLE | SDF_LSAMPL;
      s->n_chan = it->options[2] * 2;
      s->trig[0] = multiq3_ei;
      s->maxdata = 0xffffff;
      s->range_table = &range_unknown;

      encoder_reset(dev);

  return 0;
}


static int multiq3_detach(comedi_device * dev)
{
  printk("comedi%d: multiq3: remove\n", dev->minor);

  if (dev->iobase) { release_region(dev->iobase, MULTIQ3_SIZE); }
  if (dev->irq) { free_irq(dev->irq,dev); }

  return 0;
}


#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_multiq3);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_multiq3);
}
#endif
