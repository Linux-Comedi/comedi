/*
    module/vd_dds.c
    virtual driver for direct digital signal synthesis

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1999 David A. Schleef <ds@stm.lbl.gov>

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
#ifdef CONFIG_COMEDI_RTL_V1
#include <rtl_sched.h>
#endif
#ifdef CONFIG_COMEDI_RTL
#include <rtl.h>
#include <rtl_sched.h>
#include <rtl_compat.h>
#endif
#ifdef CONFIG_COMEDI_RTAI
#include <rtai.h>
#include <rtai_sched.h>
#endif

static int dds_attach(comedi_device *dev,comedi_devconfig *it);
static int dds_detach(comedi_device *dev);
comedi_driver driver_={
	driver_name:		"dds",
	attach:		dds_attach,
	detach:		dds_detach,
};

static void dds_interrupt(int irq,void *dev,struct pt_regs * regs);

typedef struct{
	int device;
	int subd;
	comedi_device *dev;
	comedi_subdevice *s;
	RT_TASK rt_task;

	unsigned int accumulator;
	unsigned int args[3];

	sampl_t data[2];

	comedi_trig trig;
	int soft_irq;
}dds_private;
#define devpriv ((dds_private *)dev->private)


static comedi_device *broken_rtl_dev;

static void dds_interrupt(int irq,void *d,struct pt_regs * regs)
{
	comedi_device *dev=broken_rtl_dev;

	comedi_done(dev,dev->subdevices+0);

	comedi_unlock_ioctl(devpriv->device,devpriv->subd);
}

static inline void buf_add(comedi_device *dev,comedi_subdevice *s,sampl_t x)
{
	*(sampl_t *)(((void *)(s->cur_trig.data))+s->buf_int_ptr)=x&0xfff;
	s->buf_int_ptr+=sizeof(sampl_t);
	if(s->buf_int_ptr>=s->cur_trig.data_len){
		s->buf_int_ptr=0;
		comedi_eobuf(dev,s);
	}
	s->buf_int_count+=sizeof(sampl_t);
}


static void dds_ao_task_func(int d)
{
	comedi_device *dev=(comedi_device *)d;
	comedi_subdevice *s=dev->subdevices+0;
	comedi_trig *it=&devpriv->trig;
	comedi_trig *my_trig=&s->cur_trig;
	int ret;

	it->n_chan=1;
	it->data=devpriv->data;
	it->chanlist=my_trig->chanlist;
	while(1){
		ret=comedi_trig_ioctl(devpriv->device,devpriv->subd,it);

		if(ret<0){
			/* eek! */
		}
#ifdef CONFIG_COMEDI_RTL
		rt_task_wait();
#endif
#ifdef CONFIG_COMEDI_RTAI
		rt_task_yield();
#endif
	}
}

static int dds_cntrl_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int i,chan;

	for(i=0;i<it->n_chan;i++){
		chan=CR_CHAN(it->chanlist[i]);

		devpriv->args[chan]=it->data[i];
	}
	return i;
}


static int dds_ao_mode2(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int ret;
	RTIME now,period;
	struct timespec ts;

	//if(it->trigvar1!=0)return -EINVAL;
	if(it->trigvar<100000)return -EINVAL;	/* 10 khz */
	
	ret=comedi_lock_ioctl(devpriv->device,devpriv->subd);
	if(ret<0)goto out;

#if 0
	struct timespec ts;

	ts.tv_sec=0;
	ts.tv_nsec=it->trigvar;
#endif

	devpriv->trig.subdev=devpriv->subd;
	devpriv->trig.mode=TRIG_WRITE;
	devpriv->trig.flags=0;
	devpriv->trig.n=1;

	ts.tv_sec=0;
	ts.tv_nsec=it->trigvar;

#ifdef CONFIG_COMEDI_RTAI
	//period=timespec_to_RTIME(ts);
	//rt_task_init(&devpriv->rt_task,dds_ao_task_func,(int)dev,3000,4);
#endif
#ifdef CONFIG_COMEDI_RTL
	period=timespec_to_RTIME(ts);
	rt_task_init(&devpriv->rt_task,dds_ao_task_func,(int)dev,3000,4);
#endif

	now=rt_get_time();
	rt_task_make_periodic(&devpriv->rt_task,now+period,period);

	return 0;

unlock:
	comedi_unlock_ioctl(devpriv->device,devpriv->subd);
out:
	return ret;
}

int dds_ao_cancel(comedi_device *dev,comedi_subdevice *s)
{
	rt_task_delete(&devpriv->rt_task);

	comedi_unlock_ioctl(devpriv->device,devpriv->subd);

	return 0;
}

int dds_attach(comedi_device *dev,comedi_devconfig *it)
{
	int ret;
	comedi_subdevice *s;

	printk("comedi%d: dds: ",dev->minor);
	dev->board_name="dds";

	dev->n_subdevices=1;
	if((ret=alloc_subdevices(dev))<0)
		return ret;
	if((ret=alloc_private(dev,sizeof(dds_private)))<0)
		return ret;

	devpriv->device=it->options[0];
	devpriv->subd=it->options[1];

	devpriv->dev=comedi_get_device_by_minor(devpriv->device);
	devpriv->s=devpriv->dev->subdevices+devpriv->subd;

	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=devpriv->s->n_chan;
	s->len_chanlist=1024;
	s->trig[2]=dds_ao_mode2;
	s->cancel=dds_ao_cancel;
	s->maxdata=devpriv->s->maxdata;
	s->range_table=devpriv->s->range_table;
	s->range_table_list=devpriv->s->range_table_list;

#ifdef CONFIG_COMEDI_RTAI
#endif
#ifdef CONFIG_COMEDI_RTL
	devpriv->soft_irq=rtl_get_soft_irq(dds_interrupt,"dds");
#endif
	broken_rtl_dev=dev;

	printk("\n");

	return 1;
}


static int dds_detach(comedi_device *dev)
{
	printk("comedi%d: dds: remove\n",dev->minor);
	
	free_irq(devpriv->soft_irq,NULL);

	return 0;
}


