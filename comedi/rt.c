/*
    comedi/rt.c
    comedi kernel module

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>

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

#undef DEBUG

#include <linux/comedidev.h>

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <asm/io.h>

#include "rt_pend_tq.h"

#ifdef CONFIG_COMEDI_RTAI
#include <rtai.h>

#define RT_protect()	hard_cli()
#define RT_unprotect()	hard_sti()
#define RT_spin_lock_irq(x)	rt_spin_lock_irq(x)
#define RT_spin_unlock_irq(x)	rt_spin_unlock_irq(x)
#endif

#ifdef CONFIG_COMEDI_RTL
#include <rtl_core.h>
#include <rtl_sync.h>

#define RT_protect()	rtl_make_rt_system_active()
#define RT_unprotect()	rtl_make_rt_system_idle()
/* RTL doesn't have the necessary primitives, so we have to hack
 * it, dealing with the race */
#define RT_spin_lock_irq(x)	do{RT_protect();rtl_spin_lock(x);}while(0)
#define RT_spin_unlock_irq(x)	do{rtl_spin_unlock(x);RT_unprotect();}while(0)
#endif

#ifdef CONFIG_PRIORITY_IRQ
#define RT_protect()	__local_irq_disable()
#define RT_unprotect()	__local_irq_enable()
#define RT_spin_lock_irq(x)	spin_lock_hard_irq(x)
#define RT_spin_unlock_irq(x)	spin_unlock_hard_irq(x)
#endif

struct comedi_irq_struct {
	int rt;
	int irq;
	void (*handler)(int irq,void *dev_id,struct pt_regs *regs);
	unsigned long flags;
	const char *device;
	void *dev_id;
};

static int rt_get_irq(struct comedi_irq_struct *it);
static int rt_release_irq(struct comedi_irq_struct *it);


static struct comedi_irq_struct *comedi_irqs[NR_IRQS];


int comedi_request_irq(unsigned irq,void (*handler)(int, void *,struct pt_regs *),
		unsigned long flags,const char *device,void *dev_id)
{
	struct comedi_irq_struct *it;
	int ret;

	it=kmalloc(sizeof(*it),GFP_KERNEL);
	if(!it)
		return -ENOMEM;
	memset(it,0,sizeof(*it));

	it->handler=handler;
	it->irq=irq;
	it->dev_id=dev_id;
	it->device=device;

	/* null shared interrupt flag, since rt interrupt handlers do not
	 * support it, and this version of comedi_request_irq() is only
	 * called for kernels with rt support */
	it->flags = flags & ~SA_SHIRQ;

	ret=request_irq(irq,handler,it->flags,device,dev_id);
	if(ret<0){
		// we failed, so fall back on allowing shared interrupt
		if(flags & SA_SHIRQ)
		{
			it->flags = flags;
			ret=request_irq(irq,handler,it->flags,device,dev_id);
		}
		if(ret<0){
			kfree(it);
			return ret;
		}
	}

	comedi_irqs[irq]=it;

	return 0;
}

void comedi_free_irq(unsigned int irq,void *dev_id)
{
	struct comedi_irq_struct *it;

	it=comedi_irqs[irq];
	if(!it)return;

	comedi_irqs[irq]=NULL;

	if(it->rt){
		printk("real-time IRQ allocated at board removal (ignore)\n");
		rt_release_irq(it);
	}

	free_irq(it->irq,it->dev_id);

	kfree(it);
}

void comedi_switch_to_rt(comedi_device *dev)
{
	struct comedi_irq_struct *it=comedi_irqs[dev->irq];
	unsigned long flags;

	/* drivers might not be using an interrupt for commands */
	if(it == NULL)return;

	/* rt interrupts and shared interrupts don't mix */
	if(it->flags & SA_SHIRQ){
		printk("comedi: cannot switch shared interrupt to real time priority\n");
		return;
	}

	comedi_spin_lock_irqsave( &dev->spinlock, flags );

	if(!dev->rt)
		rt_get_irq(it);

	dev->rt++;
	it->rt=1;

	comedi_spin_unlock_irqrestore( &dev->spinlock, flags );
}

void comedi_switch_to_non_rt(comedi_device *dev)
{
	struct comedi_irq_struct *it=comedi_irqs[dev->irq];
	unsigned long flags;

	if(it == NULL)
		return;

	/* rt interrupts and shared interrupts don't mix */
	if(it->flags & SA_SHIRQ)
		return;

	comedi_spin_lock_irqsave( &dev->spinlock, flags );

	dev->rt--;
	if(!dev->rt)
		rt_release_irq(it);

	it->rt=0;

	comedi_spin_unlock_irqrestore( &dev->spinlock, flags );
}

void wake_up_int_handler(int arg1, void * arg2)
{
	wake_up_interruptible((wait_queue_head_t*)arg2);
}

void comedi_rt_pend_wakeup(wait_queue_head_t *q)
{
	rt_pend_call(wake_up_int_handler,0,q);
}


/* RTAI section */
#ifdef CONFIG_COMEDI_RTAI

#ifdef CONFIG_PPC
#define HAVE_RT_REQUEST_IRQ_WITH_ARG
#endif
#ifndef HAVE_RT_REQUEST_IRQ_WITH_ARG
#define DECLARE_VOID_IRQ(irq) \
static void handle_void_irq_ ## irq (void){ handle_void_irq(irq);}

static void handle_void_irq(int irq)
{
	struct comedi_irq_struct *it=comedi_irqs[irq];
	it->handler(irq,it->dev_id,NULL);
}

DECLARE_VOID_IRQ(0);
DECLARE_VOID_IRQ(1);
DECLARE_VOID_IRQ(2);
DECLARE_VOID_IRQ(3);
DECLARE_VOID_IRQ(4);
DECLARE_VOID_IRQ(5);
DECLARE_VOID_IRQ(6);
DECLARE_VOID_IRQ(7);
DECLARE_VOID_IRQ(8);
DECLARE_VOID_IRQ(9);
DECLARE_VOID_IRQ(10);
DECLARE_VOID_IRQ(11);
DECLARE_VOID_IRQ(12);
DECLARE_VOID_IRQ(13);
DECLARE_VOID_IRQ(14);
DECLARE_VOID_IRQ(15);
DECLARE_VOID_IRQ(16);
DECLARE_VOID_IRQ(17);
DECLARE_VOID_IRQ(18);
DECLARE_VOID_IRQ(19);
DECLARE_VOID_IRQ(20);
DECLARE_VOID_IRQ(21);
DECLARE_VOID_IRQ(22);
DECLARE_VOID_IRQ(23);

typedef void (*V_FP_V)(void);
static V_FP_V handle_void_irq_ptrs[]={
	handle_void_irq_0,
	handle_void_irq_1,
	handle_void_irq_2,
	handle_void_irq_3,
	handle_void_irq_4,
	handle_void_irq_5,
	handle_void_irq_6,
	handle_void_irq_7,
	handle_void_irq_8,
	handle_void_irq_9,
	handle_void_irq_10,
	handle_void_irq_11,
	handle_void_irq_12,
	handle_void_irq_13,
	handle_void_irq_14,
	handle_void_irq_15,
	handle_void_irq_16,
	handle_void_irq_17,
	handle_void_irq_18,
	handle_void_irq_19,
	handle_void_irq_20,
	handle_void_irq_21,
	handle_void_irq_22,
	handle_void_irq_23,
};
/* if you need more, fix it yourself... */

static int rt_get_irq(struct comedi_irq_struct *it)
{
	rt_request_global_irq(it->irq,handle_void_irq_ptrs[it->irq]);
	rt_startup_irq(it->irq);
	
	return 0;
}

static int rt_release_irq(struct comedi_irq_struct *it)
{
	rt_shutdown_irq(it->irq);
	rt_free_global_irq(it->irq);
	return 0;
}
#else

static int rt_get_irq(struct comedi_irq_struct *it)
{
	int ret;

	ret = rt_request_global_irq_arg(it->irq,it->handler,it->flags,
			it->device,it->dev_id);
	if(ret<0){
		rt_printk("rt_request_global_irq_arg() returned %d\n",ret);
		return ret;
	}
	rt_startup_irq(it->irq);
	
	return 0;
}

static int rt_release_irq(struct comedi_irq_struct *it)
{
	rt_shutdown_irq(it->irq);
	rt_free_global_irq(it->irq);
	return 0;
}
#endif


void comedi_rt_init(void)
{
	rt_mount_rtai();
	rt_pend_tq_init();
}

void comedi_rt_cleanup(void)
{
	rt_umount_rtai();
	rt_pend_tq_cleanup();
}

#endif


/* RTLinux section */
#ifdef CONFIG_COMEDI_RTL

static unsigned int handle_rtl_irq(unsigned int irq,struct pt_regs *regs)
{
	struct comedi_irq_struct *it=comedi_irqs[irq];
	it->handler(irq,it->dev_id,regs);
	rtl_hard_enable_irq(irq);
	return 0;
}

static int rt_get_irq(struct comedi_irq_struct *it)
{
	rtl_request_global_irq(it->irq,handle_rtl_irq);
	return 0;
}

static int rt_release_irq(struct comedi_irq_struct *it)
{
	rtl_free_global_irq(it->irq);
	return 0;
}

void comedi_rt_init(void)
{
	rt_pend_tq_init();
}

void comedi_rt_cleanup(void)
{
	rt_pend_tq_cleanup();
}

#endif

#ifdef CONFIG_COMEDI_PIRQ
static int rt_get_irq(struct comedi_irq_struct *it)
{
	int ret;

	free_irq(it->irq,it->dev_id);
	ret=request_irq(it->irq,it->handler,it->flags|SA_PRIORITY,
		it->device,it->dev_id);

	return ret;
}

static int rt_release_irq(struct comedi_irq_struct *it)
{
	int ret;

	free_irq(it->irq,it->dev_id);
	ret=request_irq(it->irq,it->handler,it->flags,
		it->device,it->dev_id);

	return ret;
}

void comedi_rt_init(void)
{
	//rt_pend_tq_init();
}

void comedi_rt_cleanup(void)
{
	//rt_pend_tq_cleanup();
}
#endif

