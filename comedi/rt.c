/*
    comedi/rt.c
    comedi kernel module

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2000 David A. Schleef <ds@stm.lbl.gov>

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

#include <comedi_module.h>

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/malloc.h>
#include <asm/io.h>



static struct comedi_irq_struct *comedi_irqs;

int comedi_request_irq(unsigned irq,void (*handler)(int, void *,struct pt_regs *),
		unsigned long flags,const char *device,void *dev_id)
{
	struct comedi_irq_struct *it;
	int ret;

	it=kmalloc(sizeof(*it),GFP_KERNEL);
	if(!it)
		return -ENOMEM;

	it->handler=handler;
	it->irq=irq;
	it->dev_id=dev_id;
	it->flags=flags;
	it->device=device;

	ret=request_irq(irq,handler,flags&~SA_PRIORITY,device,dev_id);
	if(ret<0){
		kfree(it);
		return ret;
	}

	if(flags&SA_PRIORITY){
		get_priority_irq(it);
	}

	it->next=comedi_irqs;
	comedi_irqs=it;

	return 0;
}

int comedi_change_irq_flags(unsigned int irq,void *dev_id,unsigned long flags)
{
	struct comedi_irq_struct *it;
	int ret;

	it=get_irq_struct(irq);
	if(it){
		if((it->flags&~SA_PRIORITY)!=(flags&~SA_PRIORITY))
			return -EINVAL;

		if((it->flags&SA_PRIORITY)==(flags&SA_PRIORITY))
			return 0;

		it->flags=flags;
		if(flags&SA_PRIORITY){
			free_irq(it->irq,it->dev_id);
			return get_priority_irq(it);
		}else{
			ret=free_priority_irq(it);
			request_irq(it->irq,it->handler,it->flags,it->device,it->dev_id);
			return ret;
		}
	}

	return -EINVAL;
}

void comedi_free_irq(unsigned int irq,void *dev_id)
{
	struct comedi_irq_struct *it,*prev;

	prev=NULL;
	for(it=comedi_irqs;it;it=it->next){
		if(it->irq==irq){
			break;
		}
		prev=it;
	}
	if(it->flags&SA_PRIORITY)
		free_priority_irq(it);

	free_irq(it->irq,it->dev_id);

	if(prev) prev->next=it->next;
	else comedi_irqs=it->next;

	kfree(it);
}

struct comedi_irq_struct *get_irq_struct(unsigned int irq)
{
	struct comedi_irq_struct *it;

	for(it=comedi_irqs;it;it=it->next){
		if(it->irq==irq){
			return it;
		}
	}
	return NULL;
}

#ifdef HAVE_RT_PEND_TQ
void wake_up_int_handler(int arg1, void * arg2)
{
	wake_up_interruptible((wait_queue_head_t*)arg2);
}
#endif
