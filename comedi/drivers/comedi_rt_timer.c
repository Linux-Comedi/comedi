/*
    module/comedi_rt_timer.c
    virtual driver for using RTL timing sources

    Authors: David A. Schleef, Frank M. Hess

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


**************************************************************************

Options:
	[0] - minor number of device you wish to emulate commands for
	[1] - subdevice number you wish to emulate commands for

TODO:
	Support for digital io commands could be added, except I can't see why
		anyone would want to use them
	What happens if device we are emulating for is de-configured?

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
static inline RTIME nano2count(long long ns)
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

#define rt_get_time() nano2count(gethrtime())
// end hack

// rtl-rtai compatibility
#define rt_task_wait_period() rt_task_wait()
#define rt_pend_linux_srq(irq) rtl_global_pend_irq(irq)
#define rt_free_srq(irq) rtl_free_soft_irq(irq)
#define rt_request_srq(x,y,z) rtl_get_soft_irq(y,"timer")
#define rt_task_init(a,b,c,d,e,f,g) rt_task_init(a,b,c,d,(e)+1)

#endif
#ifdef CONFIG_COMEDI_RTAI
#include <rtai.h>
#include <rtai_sched.h>
#endif

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
}timer_private;
#define devpriv ((timer_private *)dev->private)


static void timer_interrupt(int unused, void *device)
{
	comedi_device *dev = (comedi_device*)device;

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

	if(async->buf_int_count == async->buf_user_count)
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

	for(n=0;n<cmd->stop_arg || cmd->stop_src == TRIG_NONE; n++){
		/* pause goes at beginning so task can not be interrupted between
		 * writing last point to buffer (buf_add()) and comedi_done()
		 */
		if(cmd->scan_begin_src == TRIG_TIMER && n != 0){
			rt_task_wait_period();
		}
		for(i = 0; i < cmd->scan_end_arg; i++){
			ret = comedi_data_read(devpriv->device,devpriv->subd,
				CR_CHAN(cmd->chanlist[i]),
				CR_RANGE(cmd->chanlist[i]),
				CR_AREF(cmd->chanlist[i]),
				&data);
			if(ret<0){
				comedi_error(dev, "read error");
				comedi_error_done(dev, s);
				goto cleanup;
			}
			if(cmd->convert_src == TRIG_TIMER)
				rt_task_wait_period();
			buf_add(dev,s,data);
		}
		s->async->events |= COMEDI_CB_EOS | COMEDI_CB_BLOCK;
		comedi_event(dev,s,s->async->events);
		s->async->events = 0;
	}
	comedi_done(dev,s);

cleanup:

	rt_pend_call(timer_interrupt, 0, dev);
	// we are deleting ourself here, no lines afterwards will be executed!
	rt_task_delete(&devpriv->rt_task);
}

static void timer_ao_task_func(int d)
{
	comedi_device *dev=(comedi_device *)d;
	comedi_subdevice *s=dev->write_subdev;
	comedi_cmd *cmd=&s->async->cmd;
	int i,n,ret;
	int data;

	s->async->events = 0;

	for(n = 0; n < cmd->stop_arg || cmd->stop_src == TRIG_NONE; n++){
		for(i=0;i<cmd->scan_end_arg;i++){
			data = buf_remove(dev,s);
			if(data < 0) {
				comedi_error(dev, "buffer underrun");
				comedi_error_done(dev, s);
				goto cleanup;
			}
			ret = comedi_data_write(devpriv->device,devpriv->subd,
				CR_CHAN(cmd->chanlist[i]),
				CR_RANGE(cmd->chanlist[i]),
				CR_AREF(cmd->chanlist[i]),
				data);
			if(ret<0){
				comedi_error(dev, "write error");
				comedi_error_done(dev, s);
				goto cleanup;
			}
			if(cmd->convert_src == TRIG_TIMER)
				rt_task_wait_period();
		}
		s->async->events |= COMEDI_CB_EOS | COMEDI_CB_BLOCK;
		comedi_event(dev,s,s->async->events);
		s->async->events = 0;
		if(cmd->scan_begin_src == TRIG_TIMER)
			rt_task_wait_period();
	}
	comedi_done(dev,s);

cleanup:

	rt_pend_call(timer_interrupt, 0, dev);
	// we are deleting ourself here, no lines afterwards will be executed!
	rt_task_delete(&devpriv->rt_task);
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

	err = cmdtest_helper(cmd,
		TRIG_NOW,	/* start_src */
		TRIG_TIMER | TRIG_FOLLOW,	/* scan_begin_src */
		TRIG_NOW | TRIG_TIMER,	/* convert_src */
		TRIG_COUNT,	/* scan_end_src */
		TRIG_COUNT | TRIG_NONE);	/* stop_src */
	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually
	 * compatible */

	if(cmd->scan_begin_src != TRIG_TIMER &&
		cmd->scan_begin_src != TRIG_FOLLOW)
		err++;
	if(cmd->convert_src != TRIG_TIMER &&
		cmd->convert_src != TRIG_NOW)
		err++;
	if(cmd->stop_src != TRIG_COUNT &&
		cmd->stop_src != TRIG_NONE)
		err++;
	/* we dont yet support TRIG_TIMER for scan_begin_src and convert_src
	 * simultaneously */
	if(cmd->scan_begin_src == TRIG_TIMER && cmd->convert_src == TRIG_TIMER)
		err++;
	if(cmd->scan_begin_src == TRIG_FOLLOW && cmd->convert_src != TRIG_TIMER)
		err++;
	if(cmd->convert_src == TRIG_NOW && cmd->scan_begin_src != TRIG_TIMER)
		err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	// limit frequency, this is fairly arbitrary
	if(cmd->scan_begin_src == TRIG_TIMER){
		if(cmd->scan_begin_arg<100000){	/* 10 khz */
			cmd->scan_begin_arg=100000;
			err++;
		}
		if(cmd->scan_begin_arg>1e9){	/* 1 hz */
			cmd->scan_begin_arg=1e9;
			err++;
		}
	}
	if(cmd->convert_src == TRIG_TIMER){
		if(cmd->convert_arg<100000){	/* 10 khz */
			cmd->convert_arg=100000;
			err++;
		}
		if(cmd->convert_arg>1e9){	/* 1 hz */
			cmd->convert_arg=1e9;
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up and arguments */

	if(err)return 4;

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
#ifdef CONFIG_COMEDI_RTAI
	start_rt_timer(1);
#endif
	delay = nano2count(cmd->start_arg);
	if(cmd->scan_begin_src == TRIG_TIMER)
		period = nano2count(cmd->scan_begin_arg);
	else if(cmd->convert_src == TRIG_TIMER)
		period = nano2count(cmd->convert_arg);
	else {
		comedi_error(dev, "bug!");
		return -1;
	}
	if(s == dev->read_subdev)
		ret = rt_task_init(&devpriv->rt_task,timer_ai_task_func,(int)dev,3000,0,0,0);
	else
		ret = rt_task_init(&devpriv->rt_task,timer_ao_task_func,(int)dev,3000,0,0,0);
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
	rt_pend_call(timer_interrupt, 0, dev);

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

	return 1;
}

// free allocated resources
static int timer_detach(comedi_device *dev)
{
	printk("comedi%d: timer: remove\n",dev->minor);

	return 0;
}


