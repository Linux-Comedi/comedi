/*
 * RTAI support rooutines
 *
 */

#include <linux/comedidev.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/stddef.h>

#include <asm/irq.h>
#include <asm/ptrace.h>

#include <rtai/rtai.h>


extern void rt_unmask_irq(unsigned int irq);

static struct comedi_irq_struct *rtai_irq;

static void handle_rtai_irq(void)
{
	struct comedi_irq_struct *it=rtai_irq;

	if(it)
		it->handler(it->irq,it->dev_id,NULL);

	rt_unmask_irq(it->irq);
}

int get_priority_irq(struct comedi_irq_struct *it)
{
	rtai_irq=it;

	//free_irq(it->irq,it->dev_id);
	rt_request_global_irq(it->irq,handle_rtai_irq);
	rt_startup_irq(it->irq); // rtai 1.3

	return 0;
}

int free_priority_irq(struct comedi_irq_struct *it)
{
	rt_free_global_irq(it->irq);
	//request_irq(it->irq,it->handler,it->flags&~SA_PRIORITY,"fixme",it->dev_id);

	return 0;
}

#ifdef NEED_RT_PEND_TQ

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
	INC_CIRCULAR_PTR(rt_pend_head,rt_pend_tq,RT_PEND_TQ_SIZE);
	if(rt_pend_head==rt_pend_tail) {
		// overflow, we just refuse to take this request
		DEC_CIRCULAR_PTR(rt_pend_head,rt_pend_tq,RT_PEND_TQ_SIZE);
		return -EAGAIN;
	}
	rt_pend_head->func=func;
	rt_pend_head->arg1=arg1;
	rt_pend_head->arg2=arg2;
	rt_pend_linux_srq(rt_pend_tq_irq);
	return 0;
}

void rt_pend_irq_handler(void)
{
	while(rt_pend_head!=rt_pend_tail) {
		INC_CIRCULAR_PTR(rt_pend_tail,rt_pend_tq,RT_PEND_TQ_SIZE);
		rt_pend_tail->func(rt_pend_tail->arg1,rt_pend_tail->arg2);
	}
}

#define random_pointer ((void *)&rt_pend_tq_init)

int rt_pend_tq_init(void)
{
	rt_pend_head=rt_pend_tail=rt_pend_tq;
	rt_pend_tq_irq=rt_request_srq(0,rt_pend_irq_handler,random_pointer);
	if(rt_pend_tq_irq<0){
		rt_pend_tq_irq=0;
		printk("rt_pend_tq_init(): couldn't alloc srq\n");
		return -EINVAL;
	}else{
		printk("rt_pend_tq_init(): irq=%d\n",rt_pend_tq_irq);
	}
	return 0;
}

void rt_pend_tq_cleanup(void)
{
	if(rt_pend_tq_irq){
		free_irq(rt_pend_tq_irq,random_pointer);
	}
}
#endif


void comedi_rtai_init(void)
{
	rt_mount_rtai();
	rt_pend_tq_init();
}

void comedi_rtai_cleanup(void)
{
	rt_pend_tq_cleanup();
	rt_umount_rtai();
}

