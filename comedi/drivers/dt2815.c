/*
   module/dt2815.c
   hardware driver for Data Translation DT2815

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


static comedi_lrange range_dt2815_ao_32_current = { 1, {
	RANGE( 0,	32 )	/* XXX mA */
}};
static comedi_lrange range_dt2815_ao_20_current = { 1, {
	RANGE( 4,	20 )
}};

#define DT2815_SIZE 2

#define DT2815_DATA 0
#define DT2815_STATUS 1

static int dt2815_attach(comedi_device *dev,comedi_devconfig *it);
static int dt2815_detach(comedi_device *dev);
comedi_driver driver_dt2815={
	driver_name:	"dt2815",
	module:		THIS_MODULE,
	attach:		dt2815_attach,
	detach:		dt2815_detach,
};
COMEDI_INITCLEANUP(driver_dt2815);

static void dt2815_free_resources(comedi_device * dev);

typedef struct {
  comedi_lrange * range_type_list[8];
} dt2815_private;

#define devpriv ((dt2815_private *)dev->private)

static int dt2815_ao(comedi_device * dev, comedi_subdevice *s, comedi_trig * it)
{
  int i;
  int t;
  int chan;
  int data;
  unsigned int status;
  unsigned int lo, hi;

  for(i=0 ; i < it->n_chan ; i++) {
    chan = CR_CHAN(it->chanlist[i]);
    data = it->data[i];

    lo = ((data & 0x0f) << 4) | (chan << 1) | 0x01;
    hi = (data & 0xff0) >> 4;
    status = inb(dev->iobase + DT2815_STATUS);
    for (t = 0 ; t < 30 ; t++) {
      if (status == 0x00) break;
      udelay(10);
      status = inb(dev->iobase + DT2815_STATUS);
    }
    if (status == 0x00) {
      outb(lo, dev->iobase + DT2815_DATA);
    } else {
      rt_printk("dt2815: failed to write low byte on %d reason %x, %d\n",
	     chan, status, t);
      return -EBUSY;
    }
    status = inb(dev->iobase + DT2815_STATUS);
    for (t = 0 ; t < 30 ; t++) {
      if (status == 0x10) break;
      udelay(10);
      status = inb(dev->iobase + DT2815_STATUS);
    }
    if (status == 0x10) {
      outb(hi, dev->iobase + DT2815_DATA);
    } else {
      rt_printk("dt2815: failed to write high byte on %d reason %x, %d\n",
	     chan, status, t);
      return -EBUSY;
    }
  }
  return i;
}

/*
  options[0]   Board base address
  options[1]   IRQ (not applicable)
  options[2]   Voltage unipolar/bipolar configuration
                 0 == unipolar 5V  (0V -- +5V)
		 1 == bipolar 5V  (-5V -- +5V)
  options[3]   Current offset configuration
                 0 == disabled  (0mA -- +32mAV)
                 1 == enabled  (+4mA -- +20mAV)
  options[4]   Firmware program configuration
                 0 == program 1 (see manual table 5-4)
                 1 == program 2 (see manual table 5-4)
                 2 == program 3 (see manual table 5-4)
                 3 == program 4 (see manual table 5-4)
  options[5]   Analog output 0 range configuration
                 0 == voltage
                 1 == current
  options[6]   Analog output 1 range configuration
  ...
  options[12]   Analog output 7 range configuration
                 0 == voltage
                 1 == current
 */

static int dt2815_attach(comedi_device * dev, comedi_devconfig * it)
{
  comedi_subdevice *s;
  int i;
  comedi_lrange *current_range_type, *voltage_range_type;

  dev->iobase = it->options[0];
  printk("comedi%d: dt2815: 0x%04x ", dev->minor, dev->iobase);
  if (check_region(dev->iobase, DT2815_SIZE) < 0) {
    printk("I/O port conflict\n");
    return -EIO;
  }
  request_region(dev->iobase, DT2815_SIZE, "dt2815");

  dev->board_name = "dt2815";

  dev->n_subdevices = 1;
  if(alloc_subdevices(dev)<0)
    return -ENOMEM;
  if(alloc_private(dev,sizeof(dt2815_private))<0)
    return -ENOMEM;

  s=dev->subdevices;
  /* ao subdevice */
  s->type=COMEDI_SUBD_AO;
  s->subdev_flags=SDF_WRITEABLE;
  s->maxdata=0xfff;
  s->n_chan=8;
  s->trig[0] = dt2815_ao;
  s->range_table_list=devpriv->range_type_list;

  current_range_type = (it->options[3])
	      ? &range_dt2815_ao_20_current
	      : &range_dt2815_ao_32_current;
  voltage_range_type = (it->options[2])
	      ? &range_bipolar5
	      : &range_unipolar5;
  for (i = 0; i < 8; i++) {
    devpriv->range_type_list[i] = (it->options[5+i])
	    ? current_range_type
	    : voltage_range_type;
  }

  /* Init the 2815 */
  outb(0x00, dev->iobase + DT2815_STATUS);
  for (i = 0 ; i < 100 ; i++) {
    /* This is incredibly slow (approx 20 ms) */
    unsigned int status;

    udelay(1000);
    status = inb(dev->iobase + DT2815_STATUS);
    if (status == 4) {
      unsigned int program;
      program = (it->options[4] & 0x3) << 3 | 0x7;
      outb(program, dev->iobase + DT2815_DATA);
      printk(", program: 0x%x (@t=%d)\n", program, i);
      break;
    } else if (status != 0x00) {
      printk("dt2815: unexpected status 0x%x (@t=%d)\n", status, i);
      if (status & 0x60) {
	outb(0x00, dev->iobase + DT2815_STATUS);
      }
    }
  }


  printk("\n");

  return 0;
}

static void dt2815_free_resources(comedi_device * dev)
{
  if(dev->iobase)
    release_region(dev->iobase, DT2815_SIZE);
}

static int dt2815_detach(comedi_device * dev)
{
  printk("comedi%d: dt2815: remove\n", dev->minor);

  dt2815_free_resources(dev);

  return 0;
}

