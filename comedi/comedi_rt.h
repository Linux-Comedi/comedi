/*
    module/comedi_rt.h
    header file for real-time structures, variables, and constants

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

#ifndef _COMEDI_RT_H
#define _COMEDI_RT_H

#include <linux/version.h>
#include <linux/config.h>
#include <linux/kdev_t.h>
#include <linux/config.h>
#include <linux/malloc.h>
#include <linux/errno.h>
#include <comedi.h>


#include <kern_compat.h>


struct comedi_irq_struct{
	struct comedi_irq_struct *next;

	int irq;
	void *dev_id;
	unsigned long flags;
	void (*handler)(int,void *,struct pt_regs *);
	const char *device;
};

int get_priority_irq(struct comedi_irq_struct *);
int free_priority_irq(struct comedi_irq_struct *);
struct comedi_irq_struct * get_irq_struct(unsigned int);

#ifndef SA_PRIORITY
#define SA_PRIORITY 0x08000000
#endif

#ifdef CONFIG_COMEDI_RTL
void comedi_rtl_init(void);
void comedi_rtl_cleanup(void);
#define comedi_rt_init		comedi_rtl_init
#define comedi_rt_cleanup	comedi_rtl_cleanup

#include <rtl_printf.h>

#define NEED_RT_PEND_TQ
#endif

#ifdef CONFIG_COMEDI_RTAI
void comedi_rtai_init(void);
void comedi_rtai_cleanup(void);
#define comedi_rt_init		comedi_rtai_init
#define comedi_rt_cleanup	comedi_rtai_cleanup

#define NEED_RT_PEND_TQ
#define NEED_RT_PRINTK
#endif

#ifdef CONFIG_COMEDI_RTL_V1
void comedi_rtl_v1_init(void);
void comedi_rtl_v1_cleanup(void);
#define comedi_rt_init		comedi_rtl_v1_init
#define comedi_rt_cleanup	comedi_rtl_v1_cleanup

/* we do not have sort IRQs (unless my v1 patch is used) T.M. */

#define NEED_RT_PRINTK
#endif

#ifdef NEED_RT_PEND_TQ
#include <rt_pend_tq.h>
extern void wake_up_int_handler(int arg1, void * arg2);
#endif

#ifdef NEED_RT_PRINTK
#define rt_printk(format,args...)	printk(format,##args)
#define rt_printk_init()		
#define rt_printk_cleanup()		
#endif

int comedi_request_irq(unsigned int irq,void (*handler)(int,void *,
	struct pt_regs *regs),unsigned long flags,const char *device,
	void *dev_id);
int comedi_change_irq_flags(unsigned int irq,void *dev_id,
	unsigned long new_flags);
void comedi_free_irq(unsigned int irq,void *dev_id);


#endif

