/*
 * RTAI support rooutines
 *
 */

#include <comedi_module.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <asm/irq.h>
#include <asm/ptrace.h>
#include <linux/string.h>
#include <linux/errno.h>

#include <rtai.h>


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
	rt_enable_irq(it->irq);

	return 0;
}

int free_priority_irq(struct comedi_irq_struct *it)
{
	rt_free_global_irq(it->irq);
	//request_irq(it->irq,it->handler,it->flags&~SA_PRIORITY,"fixme",it->dev_id);

	return 0;
}


void comedi_rtai_init(void)
{

}

void comedi_rtai_cleanup(void)
{

}

