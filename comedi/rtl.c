/*
 * RTLinux support rooutines
 *
 */

#include <comedi_module.h>
#include "rtl.h"

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <asm/irq.h>
#include <asm/ptrace.h>
#include <linux/rtl.h>
#include <linux/string.h>
#include <linux/errno.h>

#include <rtl_core.h>
#if 0
#include <rtlinux/rtl_posixio.h>
#endif



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

		return comedi_lock_ioctl(minor,subdev);
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

static struct comedi_irq_struct rtl_irq_struct;

struct comedi_irq_struct * get_irq_struct(unsigned int irq)
{
	return &rtl_irq_struct;
}

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
	rtl_free_irq(it->irq);

	return 0;
}


void comedi_rtl_init(void)
{
	rt_printk_init();
	//rtl_register_chardev(COMEDI_MAJOR,"comedi",&comedi_rtl_fops);
}

void comedi_rtl_cleanup(void)
{
	rt_printk_cleanup();
	//rtl_unregister_chardev(COMEDI_MAJOR);
}

