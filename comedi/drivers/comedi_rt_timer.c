/*
    module/vd_timer.c
    virtual driver for using RTL timing sources

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1999,2001 David A. Schleef <ds@schleef.org>

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
#include <asm/io.h>
#include <linux/comedidev.h>
#include <linux/comedilib.h>
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

/* Change this if you need more channels */
#define N_CHANLIST 16


static int timer_attach(comedi_device *dev,comedi_devconfig *it);
static int timer_detach(comedi_device *dev);
static comedi_driver driver_timer={
	module:		THIS_MODULE,
	driver_name:	"comedi_rt_timer",
	attach:		timer_attach,
	detach:		timer_detach,
};
COMEDI_INITCLEANUP(driver_timer);



typedef struct{
	int device;
	int subd;
	comedi_device *dev;
	comedi_subdevice *s;
	RT_TASK rt_task;
	int soft_irq;
	int chanlist[N_CHANLIST];
}timer_private;
#define devpriv ((timer_private *)dev->private)


static comedi_device *broken_rt_dev;

#ifdef CONFIG_COMEDI_RTL
static void timer_interrupt(int irq,void *d,struct pt_regs * regs)
#endif
#ifdef CONFIG_COMEDI_RTAI
static void timer_interrupt(void)
#endif
{
	comedi_device *dev=broken_rt_dev;

	//comedi_done(dev,dev->subdevices+0);

	comedi_unlock(devpriv->device,devpriv->subd);
}

static inline void buf_add(comedi_device *dev,comedi_subdevice *s,sampl_t x)
{
	comedi_async *async = s->async;

	*(sampl_t *)(async->data+async->buf_int_ptr)=x;
	async->buf_int_ptr+=sizeof(sampl_t);
	if(async->buf_int_ptr>=async->data_len){
		async->buf_int_ptr=0;
		async->events |= COMEDI_CB_EOBUF;
	}
	async->buf_int_count+=sizeof(sampl_t);
}


static void timer_ai_task_func(int d)
{
	comedi_device *dev=(comedi_device *)d;
	comedi_subdevice *s=dev->subdevices+0;
	comedi_cmd *cmd=&s->async->cmd;
	int i,n,ret;
	lsampl_t data;

	for(n=0;n<cmd->stop_arg;n++){
		for(i=0;i<cmd->scan_end_arg;i++){
			ret = comedi_data_read(devpriv->device,devpriv->subd,
				CR_CHAN(devpriv->chanlist[i]),
				CR_RANGE(devpriv->chanlist[i]),
				CR_AREF(devpriv->chanlist[i]),
				&data);
			if(ret<0){
				/* eek! */
			}
			buf_add(dev,s,data);
		}
		s->async->events |= COMEDI_CB_EOS;
		comedi_event(dev,s,s->async->events);
#ifdef CONFIG_COMEDI_RTL
		rt_task_wait();
#endif
#ifdef CONFIG_COMEDI_RTAI
		rt_task_wait_period();
#endif
	}
	s->async->events |= COMEDI_CB_EOA;
	comedi_event(dev,s,s->async->events);
#ifdef CONFIG_COMEDI_RTL
	rtl_global_pend_irq(devpriv->soft_irq);
#endif
#ifdef CONFIG_COMEDI_RTAI
	rt_pend_linux_srq(devpriv->soft_irq);
#endif

	rt_task_delete(&devpriv->rt_task);

	/* eek! */
}

static int timer_ai_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	comedi_insn xinsn = *insn;

	xinsn.data = data;
	xinsn.subdev = devpriv->subd;

	return comedi_do_insn(devpriv->device,&xinsn);
}

static int cmdtest_helper(comedi_cmd *cmd,
	unsigned int start_src,
	unsigned int scan_begin_src,
	unsigned int convert_src,
	unsigned int scan_end_src,
	unsigned int stop_src)
{
	int err = 0;
	int tmp;

	tmp = cmd->start_src;
	cmd->start_src &= start_src;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= scan_begin_src;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= convert_src;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= scan_end_src;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= stop_src;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	return err;
}

static int timer_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err = 0;
	//int tmp;

	err = cmdtest_helper(cmd,
		TRIG_NOW,	/* start_src */
		TRIG_TIMER,	/* scan_begin_src */
		TRIG_NOW,	/* convert_src */
		TRIG_COUNT,	/* scan_end_src */
		TRIG_COUNT);	/* stop_src */
	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually
	 * compatible */

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->scan_begin_arg<100000){	/* 10 khz */
		cmd->scan_begin_arg=100000;
		err++;
	}
	if(cmd->scan_begin_arg>1e9){	/* 1 hz */
		cmd->scan_begin_arg=1e9;
		err++;
	}
	if(err)return 3;

	/* step 4: fix up and arguments */

	/* XXX we don't do this yet, but we should */

	return 0;
}

static int timer_cmd(comedi_device *dev,comedi_subdevice *s)
{
	int ret;
	RTIME now,period;
	struct timespec ts;
	comedi_cmd *cmd = &s->async->cmd;

	/* hack attack: drivers are not supposed to do this: */
	dev->rt = 1;

	ret=comedi_lock(devpriv->device,devpriv->subd);
	if(ret<0)return ret;

	ts.tv_sec=0;
	ts.tv_nsec=cmd->scan_begin_arg;

#ifdef CONFIG_COMEDI_RTL
	period=HRT_TO_8254(cmd->scan_begin_arg);
	rt_task_init(&devpriv->rt_task,timer_ai_task_func,(int)dev,3000,4);
#endif
#ifdef CONFIG_COMEDI_RTAI
	period = start_rt_timer(nano2count(cmd->scan_begin_arg));
	rt_task_init(&devpriv->rt_task,timer_ai_task_func,(int)dev,3000,0,0,0);
#endif

	now=rt_get_time();
	rt_task_make_periodic(&devpriv->rt_task,now+period,period);

	return 0;
}

static int timer_cancel(comedi_device *dev,comedi_subdevice *s)
{
	rt_task_delete(&devpriv->rt_task);

	comedi_unlock(devpriv->device,devpriv->subd);

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

	printk("device %d, subdevice %d\n", devpriv->device, devpriv->subd);

	devpriv->dev=comedi_get_device_by_minor(devpriv->device);
	devpriv->s=devpriv->dev->subdevices+devpriv->subd;

	s=dev->subdevices+0;
	dev->read_subdev = s;
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=devpriv->s->n_chan;
	s->len_chanlist=1024;
	s->insn_read=timer_ai_insn_read;
	s->do_cmd=timer_cmd;
	s->do_cmdtest=timer_cmdtest;
	s->cancel=timer_cancel;
	s->maxdata=devpriv->s->maxdata;
	s->range_table=devpriv->s->range_table;
	s->range_table_list=devpriv->s->range_table_list;

#ifdef CONFIG_COMEDI_RTL
	devpriv->soft_irq=rtl_get_soft_irq(timer_interrupt,"timer");
	broken_rt_dev=dev;
#endif
#ifdef CONFIG_COMEDI_RTAI
	devpriv->soft_irq=rt_request_srq(0,timer_interrupt,NULL);
	broken_rt_dev=dev;
#endif

	printk("\n");

	return 1;
}


static int timer_detach(comedi_device *dev)
{
	printk("comedi%d: timer: remove\n",dev->minor);
	
#ifdef CONFIG_COMEDI_RTL
	if(devpriv->soft_irq)
		free_irq(devpriv->soft_irq,NULL);
#endif
#ifdef CONFIG_COMEDI_RTAI
	if(devpriv->soft_irq)
		rt_free_srq(devpriv->soft_irq);
#endif

	return 0;
}


