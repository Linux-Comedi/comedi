/*
    module/comedi_rt_timer.c
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

// begin hack to workaround broken HRT_TO_8254() function on rtlinux
// this function sole purpose is to divide a long long by 838
static inline long long nano2count(long long ns)
{
	unsigned long denom = 838;	// divisor
	long ms32 = ns >> 32;	// most significant 32 bits
	unsigned long ms32rem = ms32 % denom;	// remainder of ms32 / denom
	unsigned long ls32 = ns;	// least significant 32 bits
	unsigned long ls32rem = ls32 % denom;
	unsigned long big = 0xffffffff;
	unsigned long big_rem = big % denom;
	unsigned long rem_rem;

	// divide most significant bits
	ns = ms32 / denom;
	ns = ns << 32;
	// add corrections due to rounding errors
	ns += ms32rem * (big / denom) + (ms32rem * (big_rem + 1)) / denom;
	// divide least significant bits
	ns += ls32 / denom;
	// add really small correction
	rem_rem = (ms32rem * (big_rem + 1)) % denom;
	ns += (ls32rem + rem_rem) / denom;

	return ns;
}

inline RTIME rt_get_time(void)
{
        return nano2count(gethrtime());
}

// end hack

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

	comedi_unlock(devpriv->device,devpriv->subd);
}

// writes a data point to comedi's buffer, used for input
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

// reads a data point from comedi's buffer, used for output
static inline int buf_remove(comedi_device *dev,comedi_subdevice *s)
{
	comedi_async *async = s->async;
	sampl_t data;

	if(async->buf_int_ptr >= async->buf_user_ptr)
		return -1;

	data = *(sampl_t *)(async->data+async->buf_int_ptr);
	async->buf_int_ptr+=sizeof(sampl_t);
	if(async->buf_int_ptr>=async->data_len){
		async->buf_int_ptr=0;
		async->events |= COMEDI_CB_EOBUF;
	}
	async->buf_int_count+=sizeof(sampl_t);

	return data;
}

static void timer_ai_task_func(int d)
{
	comedi_device *dev=(comedi_device *)d;
	comedi_subdevice *s=dev->read_subdev;
	comedi_cmd *cmd=&s->async->cmd;
	int i,n,ret;
	lsampl_t data;

	s->async->events = 0;

	for(n=0;n<cmd->stop_arg;n++){
		/* pause goes at beginning so task can not be interrupted between
		 * writing last point to buffer (buf_add()) and comedi_done()
		 */
		if(n != 0){
#ifdef CONFIG_COMEDI_RTL
			rt_task_wait();
#endif
#ifdef CONFIG_COMEDI_RTAI
			rt_task_wait_period();
#endif
		}
		for(i=0;i<cmd->scan_end_arg;i++){
			ret = comedi_data_read(devpriv->device,devpriv->subd,
				CR_CHAN(cmd->chanlist[i]),
				CR_RANGE(cmd->chanlist[i]),
				CR_AREF(cmd->chanlist[i]),
				&data);
			if(ret<0){
				/* eek! */
			}
			buf_add(dev,s,data);
		}
		s->async->events |= COMEDI_CB_EOS;
		comedi_event(dev,s,s->async->events);
		s->async->events = 0;
	}
	comedi_done(dev,s);
#ifdef CONFIG_COMEDI_RTL
	rtl_global_pend_irq(devpriv->soft_irq);
#endif
#ifdef CONFIG_COMEDI_RTAI
	rt_pend_linux_srq(devpriv->soft_irq);
#endif

	rt_task_delete(&devpriv->rt_task);

	/* eek! */
}

static void timer_ao_task_func(int d)
{
	comedi_device *dev=(comedi_device *)d;
	comedi_subdevice *s=dev->write_subdev;
	comedi_cmd *cmd=&s->async->cmd;
	int i,n,ret;
	int data;
	int ao_repeat_flag = 0;

	if(cmd->stop_src == TRIG_NONE)
		ao_repeat_flag = 1;

	s->async->events = 0;

	do{
		for(n=0;n<cmd->stop_arg;n++){
			for(i=0;i<cmd->scan_end_arg;i++){
				data = buf_remove(dev,s);
				if(data < 0) {
					/* eek! */
				}
				ret = comedi_data_write(devpriv->device,devpriv->subd,
					CR_CHAN(cmd->chanlist[i]),
					CR_RANGE(cmd->chanlist[i]),
					CR_AREF(cmd->chanlist[i]),
					data);
				if(ret<0){
					/* eek! */
				}
			}
			s->async->events |= COMEDI_CB_EOS;
			comedi_event(dev,s,s->async->events);
			s->async->events = 0;
#ifdef CONFIG_COMEDI_RTL
			rt_task_wait();
#endif
#ifdef CONFIG_COMEDI_RTAI
			rt_task_wait_period();
#endif
		}
	}while(ao_repeat_flag);
	comedi_done(dev,s);
#ifdef CONFIG_COMEDI_RTL
	rtl_global_pend_irq(devpriv->soft_irq);
#endif
#ifdef CONFIG_COMEDI_RTAI
	rt_pend_linux_srq(devpriv->soft_irq);
#endif

	rt_task_delete(&devpriv->rt_task);

	/* eek! */
}

static int timer_insn(comedi_device *dev,comedi_subdevice *s,
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
	int stop_src = TRIG_COUNT;

// stop_src TRIG_NONE does not work yet
//	if(s == dev->write_subdev)
//		stop_src |= TRIG_NONE;

	err = cmdtest_helper(cmd,
		TRIG_NOW,	/* start_src */
		TRIG_TIMER,	/* scan_begin_src */
		TRIG_NOW,	/* convert_src */
		TRIG_COUNT,	/* scan_end_src */
		stop_src);	/* stop_src */
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
	RTIME now,period,delay;
	comedi_cmd *cmd = &s->async->cmd;

	/* hack attack: drivers are not supposed to do this: */
	dev->rt = 1;

	ret = comedi_lock(devpriv->device,devpriv->subd);
	if(ret<0)
	{
		comedi_error(dev, "failed to obtain lock");
		return ret;
	}

	delay = nano2count(cmd->start_arg);
#ifdef CONFIG_COMEDI_RTL
	period = nano2count(cmd->scan_begin_arg);
	if(s == dev->read_subdev)
		ret = rt_task_init(&devpriv->rt_task,timer_ai_task_func,(int)dev,3000,1);
	else
		ret = rt_task_init(&devpriv->rt_task,timer_ao_task_func,(int)dev,3000,1);
#endif
#ifdef CONFIG_COMEDI_RTAI
	period = start_rt_timer(nano2count(cmd->scan_begin_arg));
	if(s == dev->read_subdev)
		ret = rt_task_init(&devpriv->rt_task,timer_ai_task_func,(int)dev,3000,0,0,0);
	else
		ret = rt_task_init(&devpriv->rt_task,timer_ao_task_func,(int)dev,3000,0,0,0);
#endif
	if(ret < 0)
	{
		comedi_error(dev, "error initalizing task");
		return ret;
	}
	now=rt_get_time();
	ret = rt_task_make_periodic(&devpriv->rt_task,now+delay,period);
	if(ret < 0)
	{
		comedi_error(dev, "error starting task");
		return ret;
	}

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

	dev->n_subdevices=2;
	if((ret=alloc_subdevices(dev))<0)
		return ret;
	if((ret=alloc_private(dev,sizeof(timer_private)))<0)
		return ret;

	devpriv->device=it->options[0];
	devpriv->subd=it->options[1];

	printk("device %d, subdevice %d\n", devpriv->device, devpriv->subd);

	devpriv->dev=comedi_get_device_by_minor(devpriv->device);
	devpriv->s=devpriv->dev->subdevices+devpriv->subd;

	if(devpriv->s->type != COMEDI_SUBD_AI
		&& devpriv->s->type != COMEDI_SUBD_AO)
	{
		printk("cannot emulate subdevice type\n");
		return -EINVAL;
	}

	// input subdevice
	s=dev->subdevices+0;
	if(devpriv->s->subdev_flags & SDF_READABLE)
	{
		s->type=devpriv->s->type;
		s->subdev_flags = SDF_READABLE;
		s->n_chan=devpriv->s->n_chan;
		s->len_chanlist=1024;
		s->do_cmd=timer_cmd;
		s->do_cmdtest=timer_cmdtest;
		s->cancel=timer_cancel;
		s->maxdata=devpriv->s->maxdata;
		s->range_table=devpriv->s->range_table;
		s->range_table_list=devpriv->s->range_table_list;
		s->insn_read=timer_insn;
		dev->read_subdev = s;
	}else {
		s->type=COMEDI_SUBD_UNUSED;
	}

	// output subdevice
	s=dev->subdevices+1;
	if(devpriv->s->subdev_flags & SDF_WRITEABLE)
	{
		s->type=devpriv->s->type;
		s->subdev_flags = SDF_WRITEABLE;
		s->n_chan=devpriv->s->n_chan;
		s->len_chanlist=1024;
		s->do_cmd=timer_cmd;
		s->do_cmdtest=timer_cmdtest;
		s->cancel=timer_cancel;
		s->maxdata=devpriv->s->maxdata;
		s->range_table=devpriv->s->range_table;
		s->range_table_list=devpriv->s->range_table_list;
		s->insn_write=timer_insn;
		dev->write_subdev = s;
	}else {
		s->type=COMEDI_SUBD_UNUSED;
	}

#ifdef CONFIG_COMEDI_RTL
	devpriv->soft_irq=rtl_get_soft_irq(timer_interrupt,"timer");
#endif
#ifdef CONFIG_COMEDI_RTAI
	devpriv->soft_irq=rt_request_srq(0,timer_interrupt,NULL);
#endif
	if(devpriv->soft_irq < 0)
	{
		return devpriv->soft_irq;
	}
	broken_rt_dev=dev;

	printk("\n");

	return 1;
}

// free allocated resources
static int timer_detach(comedi_device *dev)
{
	printk("comedi%d: timer: remove\n",dev->minor);

	// make sure dev->private was sucessfully allocated
	if(devpriv)
	{
		if(devpriv->soft_irq > 0)
		{
#ifdef CONFIG_COMEDI_RTL
			rtl_free_soft_irq(devpriv->soft_irq);
#endif
#ifdef CONFIG_COMEDI_RTAI
			rt_free_srq(devpriv->soft_irq);
#endif
		}
	}

	return 0;
}


