/*
 * RTLinux v1 support rooutines
 *
 */

#include <linux/comedidev.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/irq.h>
#include <asm/ptrace.h>
#include <linux/string.h>
#include <linux/errno.h>


static struct comedi_irq_struct *rtlv1_irq;

static void handle_rtlv1_irq(void)
{
	struct comedi_irq_struct *it=rtlv1_irq;

	if(it)
		it->handler(it->irq,it->dev_id,NULL);
}

int get_priority_irq(struct comedi_irq_struct *it)
{
	rtlv1_irq=it;
	request_RTirq(it->irq,handle_rtlv1_irq);

	return 0;
}

int free_priority_irq(struct comedi_irq_struct *it)
{
	free_RTirq(it->irq);
	rtlv1_irq=NULL;

	return 0;
}



void comedi_rtl_v1_init(void)
{

}

void comedi_rtl_v1_cleanup(void)
{

}



