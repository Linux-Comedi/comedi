/*
 * RTLinux support rooutines
 *
 */

#define __NO_VERSION__

#include <linux/comedidev.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sched.h>
//#include <linux/rtl.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/irq.h>

#include <asm/ptrace.h>

#include <rtl_core.h>
#if 0
#include <rtlinux/rtl_posixio.h>
#endif



#if 0
/* rt_printk() section */

#define BUF_LEN	(16384)

static char rt_printk_buf[BUF_LEN];
static char buf[1024];

static int buf_front,buf_back;

static int rt_printk_irq;

/* this is rather bogus, but it will catch most errors */
static volatile int rt_printk_lock;

static void rt_printk_interrupt(int irq,void *junk,struct pt_regs *regs);

int rt_printk(const char *fmt, ...)
{
	va_list args;
	int len,i;

	if(rtl_rt_system_is_idle()){
		va_start(args, fmt);
		len=printk(fmt,args);
		va_end(args);
		return len;
	}

	if(rt_printk_lock){
		/* force a panic */
		*(int *)0=0;
	}
	rt_printk_lock=1;

	va_start(args, fmt);
	len = vsprintf(buf, fmt, args); /* hopefully i < sizeof(buf)-1 */
	va_end(args);

	if(buf_front+len>=BUF_LEN){
		i=BUF_LEN-buf_front;
		memcpy(rt_printk_buf+buf_front,buf,i);
		memcpy(rt_printk_buf,buf+i,len-i);
		buf_front=len-i;
	}else{
		memcpy(rt_printk_buf+buf_front,buf,len);
		buf_front+=len;
	}

	rt_printk_lock=0;

	rtl_global_pend_irq(rt_printk_irq);

	return len;
}

void rt_printk_cleanup(void)
{
	free_irq(rt_printk_irq,NULL);
	rt_printk_irq=0;
}

int rt_printk_init(void)
{
	rt_printk_irq=rtl_get_soft_irq(rt_printk_interrupt,"rt_printk");
	if(rt_printk_irq<0){
		printk("rt_printk: can't allocate soft irq\n");
		return -EIO;
	}

	return 0;
}

void rt_printk_interrupt(int irq,void *junk,struct pt_regs *regs)
{
	int tmp;

	for(;;){
		tmp=buf_front;
		if(buf_back>tmp){
			printk("%.*s",BUF_LEN-buf_back,rt_printk_buf+buf_back);
			buf_back=0;
		}
		if(buf_back==tmp)break;
		printk("%.*s",tmp-buf_back,rt_printk_buf+buf_back);
		buf_back=tmp;
	}
}
#endif


#if 0

static ssize_t comedi_rtl_read(struct rtl_file *file,char *buf,
	size_t len,loff_t *p)
{
	return -EIO;
}

static ssize_t comedi_rtl_write(struct rtl_file *file,const char *buf,
	size_t len,loff_t *p)
{
	return -EIO;
}

static int comedi_rtl_ioctl(struct rtl_file *file,unsigned int cmd,
	unsigned long arg)
{
	switch(cmd){
	case COMEDI_TRIG:
	  {
		comedi_trig *it=(comedi_trig *)arg;
		int minor=file->f_minor;
		int subdev=it->subdev;

		return comedi_trig_ioctl(minor,subdev,it);
	  }
	case COMEDI_LOCK:
	  {
		int minor=file->f_minor;
		int subdev=(int)arg;

		return comedi_lock_ioctl(minor,subdev);
	  }
	case COMEDI_UNLOCK:
	  {
		int minor=file->f_minor;
		int subdev=(int)arg;

		return comedi_unlock_ioctl(minor,subdev);
	  }
	case COMEDI_CANCEL:
	  {
		int minor=file->f_minor;
		int subdev=(int)arg;

		return comedi_cancel_ioctl(minor,subdev);
	  }
	}

	return -EIO;
}

static int comedi_rtl_open(struct rtl_file *file)
{
	return 0;
}

static int comedi_rtl_release(struct rtl_file *file)
{
	return 0;
}


struct rtl_file_operations comedi_rtl_fops={
	read:		comedi_rtl_read,
	write:		comedi_rtl_write,
	ioctl:		comedi_rtl_ioctl,
	open:		comedi_rtl_open,
	release:	comedi_rtl_release,
};

#endif

static unsigned int handle_rtl_irq(unsigned int irq,struct pt_regs *regs)
{
	struct comedi_irq_struct *it;

	it=get_irq_struct(irq);
	if(it)
		it->handler(irq,it->dev_id,regs);
	rtl_hard_enable_irq(irq);
	return 0;
}

int get_priority_irq(struct comedi_irq_struct *it)
{
	rtl_request_global_irq(it->irq,handle_rtl_irq);

	return 0;
}

int free_priority_irq(struct comedi_irq_struct *it)
{
	rtl_free_global_irq(it->irq);

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
	rtl_global_pend_irq(rt_pend_tq_irq);
	return 0;
}

void rt_pend_irq_handler(int irq, void *dev, struct pt_regs * regs)
{
	while(rt_pend_head!=rt_pend_tail) {
		INC_CIRCULAR_PTR(rt_pend_tail,rt_pend_tq,RT_PEND_TQ_SIZE);
		rt_pend_tail->func(rt_pend_tail->arg1,rt_pend_tail->arg2);
	}
}

int rt_pend_tq_init()
{
	rt_pend_head=rt_pend_tail=rt_pend_tq;
	return rt_pend_tq_irq=rtl_get_soft_irq(rt_pend_irq_handler,"rt_pend_irq");
}

void rt_pend_tq_cleanup()
{
	free_irq(rt_pend_tq_irq,NULL);
}
#endif

void comedi_rtl_init(void)
{
	//rt_printk_init();
	//rtl_register_chardev(COMEDI_MAJOR,"comedi",&comedi_rtl_fops);
	rt_pend_tq_init();
}

void comedi_rtl_cleanup(void)
{
	//rt_printk_cleanup();
	//rtl_unregister_chardev(COMEDI_MAJOR);
	rt_pend_tq_cleanup();
}

