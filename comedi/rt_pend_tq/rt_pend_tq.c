/* rt_pend_tq.c */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include "rt_pend_tq.h"
#ifdef CONFIG_COMEDI_RTAI
#include <rtai.h>
#endif
#ifdef CONFIG_COMEDI_RTL
#include <rtl.h>
#include <rtl_core.h>
#endif

#ifdef standalone
#define rt_pend_tq_init init_module
#define rt_pend_tq_cleanup cleanup_module
#endif

volatile static struct rt_pend_tq rt_pend_tq[RT_PEND_TQ_SIZE]; 
volatile static struct rt_pend_tq * volatile rt_pend_head= rt_pend_tq,
	* volatile rt_pend_tail = rt_pend_tq;
int rt_pend_tq_irq=0;

// WARNING: following code not checked against race conditions yet.
#define INC_CIRCULAR_PTR(ptr,begin,size) do {if(++(ptr)>=(begin)+(size)) (ptr)=(begin); } while(0)
#define DEC_CIRCULAR_PTR(ptr,begin,size) do {if(--(ptr)<(begin)) (ptr)=(begin)+(size)-1; } while(0)

int rt_pend_call(void (*func)(int arg1, void * arg2), int arg1, void * arg2)
{
	if(func==NULL)
		return -EINVAL;
	if(rt_pend_tq_irq<=0)
		return -ENODEV;
// FIXME: grab RT spinlock/cli
	INC_CIRCULAR_PTR(rt_pend_head,rt_pend_tq,RT_PEND_TQ_SIZE);
	if(rt_pend_head==rt_pend_tail) {
		// overflow, we just refuse to take this request
		DEC_CIRCULAR_PTR(rt_pend_head,rt_pend_tq,RT_PEND_TQ_SIZE);
// FIXME: release RT spinlock/restore
		return -EAGAIN;
	}
// FIXME: release RT spinlock/restore
	rt_pend_head->func=func;
	rt_pend_head->arg1=arg1;
	rt_pend_head->arg2=arg2;
#ifdef CONFIG_COMEDI_RTAI
	rt_pend_linux_srq(rt_pend_tq_irq);
#endif
#ifdef CONFIG_COMEDI_RTL
	rtl_global_pend_irq(rt_pend_tq_irq);
#endif
	return 0;
}

#ifdef CONFIG_COMEDI_RTAI
void rt_pend_irq_handler(void)
#endif
#ifdef CONFIG_COMEDI_RTL
void rt_pend_irq_handler(int irq, void *dev, struct pt_regs * regs)
#endif
{
	while(rt_pend_head!=rt_pend_tail) {
		INC_CIRCULAR_PTR(rt_pend_tail,rt_pend_tq,RT_PEND_TQ_SIZE);
		rt_pend_tail->func(rt_pend_tail->arg1,rt_pend_tail->arg2);
	}
}

int rt_pend_tq_init(void)
{
	rt_pend_head=rt_pend_tail=rt_pend_tq;
#ifdef CONFIG_COMEDI_RTAI
	rt_pend_tq_irq=rt_request_srq(0,rt_pend_irq_handler,NULL);
#endif
#ifdef CONFIG_COMEDI_RTL
	rt_pend_tq_irq=rtl_get_soft_irq(rt_pend_irq_handler,"rt_pend_irq");
	if(rt_pend_tq_irq>0) 
		printk("rt_pend_tq: RT bottom half scheduler initialized OK\n");
	else
		printk("rt_pend_tq: rtl_get_soft_irq failed\n");
#endif
	return 0;
}

void rt_pend_tq_cleanup(void)
{
	printk("rt_pend_tq: unloading\n");
	free_irq(rt_pend_tq_irq,NULL);
}

