/*
    module/vd_timer.c
    virtual driver for using RTL timing sources

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
#include <asm/rt_irq.h>
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

static int timer_attach(comedi_device *dev,comedi_devconfig *it);
static int timer_detach(comedi_device *dev);
comedi_driver driver_timer={
	driver_name:	"timer",
	attach:		timer_attach,
	detach:		timer_detach,
};


static void timer_interrupt(int irq,void *dev,struct pt_regs * regs);

typedef struct{
	int device;
	int subd;
	comedi_device *dev;
	comedi_subdevice *s;
	RT_TASK rt_task;
	sampl_t *data;
	comedi_trig trig;
	int soft_irq;
}timer_private;
#define devpriv ((timer_private *)dev->private)


static comedi_device *broken_rtl_dev;

static void timer_interrupt(int irq,void *d,struct pt_regs * regs)
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


static void timer_ai_task_func(int d)
{
	comedi_device *dev=(comedi_device *)d;
	comedi_subdevice *s=dev->subdevices+0;
	comedi_trig *it=&devpriv->trig;
	comedi_trig *my_trig=&s->cur_trig;
	int i,n,ret;
	int n_chan;

	n_chan=s->cur_trig.n_chan;

	for(n=0;n<my_trig->n;n++){
		for(i=0;i<n_chan;i++){
			it->n_chan=1;
			it->data=devpriv->data+i;
			it->chanlist=my_trig->chanlist+i;

			ret=comedi_trig_ioctl(devpriv->device,devpriv->subd,it);

			if(ret<0){
				/* eek! */
			}
		}
		for(i=0;i<n_chan;i++){
			buf_add(dev,s,devpriv->data[i]);
		}
		rt_task_wait();
	}
	rtl_global_pend_irq(devpriv->soft_irq);

	rt_task_delete(&devpriv->rt_task);

	/* eek! */
}

static int timer_ai_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	return comedi_trig_ioctl(devpriv->device,devpriv->subd,it);
}

static int timer_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	if(cmd->scan_start_arg<100000)	/* 10 khz */
		cmd->scan_start_arg=100000;
	if(cmd->scan_start_arg>1e9)	/* 1 hz */
		cmd->scan_start_arg=1e9;

}

static int timer_cmd(comedi_device *dev,comedi_subdevice *s)
{
	int ret;
	RTIME now,period;
	struct timespec ts;

	ret=comedi_lock_ioctl(devpriv->device,devpriv->subd);
	if(ret<0)return ret;

	/* XXX this does not get freed */
	devpriv->data=kmalloc(sizeof(sampl_t)*cmd->chanlist_len,GFP_KERNEL);
	if(!devpriv->data){
		ret=-ENOMEM;
		goto unlock;
	}

	devpriv->trig.subdev=devpriv->subd;
	devpriv->trig.mode=0;
	devpriv->trig.flags=0;
	devpriv->trig.n=1;

	ts.tv_sec=0;
	ts.tv_nsec=it->trigvar;
	period=timespec_to_RTIME(ts);

	rt_task_init(&devpriv->rt_task,timer_ai_task_func,(int)dev,3000,4);

	now=rt_get_time();
	rt_task_make_periodic(&devpriv->rt_task,now+period,period);

	return 0;

unlock:
	comedi_unlock_ioctl(devpriv->device,devpriv->subd);
	return ret;
}

static int timer_ai_mode2(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
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

	/* XXX this does not get freed */
	devpriv->data=kmalloc(sizeof(sampl_t)*it->n_chan,GFP_KERNEL);
	if(!devpriv->data){
		ret=-ENOMEM;
		goto unlock;
	}

	devpriv->trig.subdev=devpriv->subd;
	devpriv->trig.mode=0;
	devpriv->trig.flags=0;
	devpriv->trig.n=1;

	ts.tv_sec=0;
	ts.tv_nsec=it->trigvar;
	period=timespec_to_RTIME(ts);

	rt_task_init(&devpriv->rt_task,timer_ai_task_func,(int)dev,3000,4);

	now=rt_get_time();
	rt_task_make_periodic(&devpriv->rt_task,now+period,period);

	return 0;

unlock:
	comedi_unlock_ioctl(devpriv->device,devpriv->subd);
out:
	return ret;
}

int timer_cancel(comedi_device *dev,comedi_subdevice *s)
{
	rt_task_delete(&devpriv->rt_task);

	comedi_unlock_ioctl(devpriv->device,devpriv->subd);

	return 0;
}

static int timer_attach(comedi_device *dev,comedi_devconfig *it)
{
	int ret;
	comedi_subdevice *s;

	printk("comedi%d: timer: ",dev->minor);
	dev->board_name="timer";

	dev->n_subdevices=1;
	if((ret=alloc_subdevices(dev))<0)
		return ret;
	if((ret=alloc_private(dev,sizeof(timer_private)))<0)
		return ret;

	devpriv->device=it->options[0];
	devpriv->subd=it->options[1];

	devpriv->dev=comedi_get_device_by_minor(devpriv->device);
	devpriv->s=devpriv->dev->subdevices+devpriv->subd;

	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=devpriv->s->n_chan;
	s->len_chanlist=1024;
	s->trig[0]=timer_ai_mode0;
	s->trig[2]=timer_ai_mode2;
	s->do_cmd=timer_cmd;
	s->do_cmdtest=timer_cmdtest;
	s->cancel=timer_cancel;
	s->maxdata=devpriv->s->maxdata;
	s->range_table=devpriv->s->range_table;
	s->range_table_list=devpriv->s->range_table_list;

	devpriv->soft_irq=rtl_get_soft_irq(timer_interrupt,"timer");
	broken_rtl_dev=dev;

	printk("\n");

	return 1;
}


static int timer_detach(comedi_device *dev)
{
	printk("comedi%d: timer: remove\n",dev->minor);
	
	free_irq(devpriv->soft_irq,NULL);

	return 0;
}


