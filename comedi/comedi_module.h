/*
    module/comedi_module.h
    header file for kernel-only structures, variables, and constants

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-8 David A. Schleef <ds@stm.lbl.gov>

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

#ifndef _COMEDI_MODULE_H
#define _COMEDI_MODULE_H

#include <linux/version.h>
#include <linux/config.h>
#include <linux/kdev_t.h>
#include <config.h>
#include <linux/malloc.h>
#include <linux/errno.h>
#include <comedi.h>


#include <kern_compat.h>

#ifdef CONFIG_COMEDI_RT

struct comedi_irq_struct{
	struct comedi_irq_struct *next;

	int irq;
	void *dev_id;
	unsigned long flags;
	void (*handler)(int,void *,struct pt_regs *);
	char *device;
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
#endif

#ifdef CONFIG_COMEDI_RTAI
void comedi_rtai_init(void);
void comedi_rtai_cleanup(void);
#define rt_printk(format,args...)	printk(format,##args)
#define rt_printk_init()		
#define rt_printk_cleanup()		
#endif

#ifdef CONFIG_COMEDI_RTL_V1
void comedi_rtl_v1_init(void);
void comedi_rtl_v1_cleanup(void);
#define rt_printk(format,args...)	printk(format,##args)
#define rt_printk_init()		
#define rt_printk_cleanup()		
#endif

#else /* !CONFIG_COMEDI_RT */

#define rt_printk(format,args...)	printk(format,##args)
#define rt_printk_init()		
#define rt_printk_cleanup()		

#endif

#ifdef CONFIG_COMEDI_DEBUG
#define DPRINTK(format, args...)	printk("comedi: " format , ## args )
#else
#define DPRINTK(format, args...)	/* */
#endif


typedef struct comedi_device_struct comedi_device;
typedef struct comedi_subdevice_struct comedi_subdevice;
typedef struct comedi_driver_struct comedi_driver;


struct comedi_subdevice_struct{
	int type;
	int n_chan;
	int subdev_flags;
	int timer_type;
	int len_chanlist;		/* length of channel/gain list, if available */

	void		*prealloc_buf;		/* pre-allocated buffer */
	unsigned int	prealloc_bufsz;		/* buffer size, in bytes */

	void *lock;
	void *busy;
	
	int io_bits;
	
	lsampl_t maxdata;		/* if maxdata==0, use list */
	lsampl_t *maxdata_list;		/* list is channel specific */
	
	unsigned int flags;
	unsigned int *flaglist;

	unsigned int range_type;
	unsigned int *range_type_list;
	
	unsigned int *chanlist;		/* driver-owned chanlist (not used) */
	
	comedi_trig	cur_trig;	/* current trig structure */
	comedi_cmd	cmd;
	
	volatile unsigned int buf_int_ptr;	/* buffer marker for interrupt */
	unsigned int buf_user_ptr;		/* buffer marker for read() and write() */
	volatile unsigned int buf_int_count;	/* byte count for interrupt */
	unsigned int buf_user_count;		/* byte count for read() and write() */
	unsigned int cur_chan;		/* useless channel marker for interrupt */
	
#if 0
	unsigned int *range_list;	/* is this necessary? */
#endif
	
	int (*trig[5])(comedi_device *,comedi_subdevice *,comedi_trig *);

	int (*do_cmd)(comedi_device *,comedi_subdevice *);
	int (*poll)(comedi_device *,comedi_subdevice *);
	int (*cancel)(comedi_device *,comedi_subdevice *);
	int (*do_lock)(comedi_device *,comedi_subdevice *);
	int (*do_unlock)(comedi_device *,comedi_subdevice *);

	unsigned int cb_mask;
	int (*cb_func)(unsigned int flags,void *);
	void *cb_arg;

	unsigned int state;
};

struct comedi_driver_struct{
	struct comedi_driver_struct *next;

	char *driver_name;
	struct module *module;
	int (*attach)(comedi_device *,comedi_devconfig *);
	int (*detach)(comedi_device *);
	int (*recognize)(char *name);
};

struct comedi_device_struct{
	int use_count;
	comedi_driver *driver;
	void *private;
	kdev_t minor;
	//char *driver_name;
	char *board_name;
	int board;
	void *board_ptr;
	int attached;

	int n_subdevices;
	comedi_subdevice *subdevices;
	int options[COMEDI_NDEVCONFOPTS];

	/* dumb */
	int iobase;
	int iosize;
	int irq;

	wait_queue_head_t wait;
};



extern comedi_device *comedi_devices;

/*
 * function prototypes
 */

void comedi_error(comedi_device *dev,const char *s);
void comedi_done(comedi_device *dev,comedi_subdevice *s);
void comedi_eos(comedi_device *dev,comedi_subdevice *s);
void comedi_eobuf(comedi_device *dev,comedi_subdevice *s);
void comedi_bufcheck(comedi_device *dev,comedi_subdevice *s);

int comedi_device_detach(comedi_device *dev);
int comedi_device_attach(comedi_device *dev,comedi_devconfig *it);
int comedi_driver_register(comedi_driver *);
int comedi_driver_unregister(comedi_driver *);

void init_polling(void);
void cleanup_polling(void);
void start_polling(comedi_device *);
void stop_polling(comedi_device *);

void comedi_proc_init(void);
void comedi_proc_cleanup(void);

int di_unpack(unsigned int bits,comedi_trig *it);
int do_pack(unsigned int *bits,comedi_trig *it);

#ifdef CONFIG_COMEDI_RT
int comedi_request_irq(unsigned int irq,void (*handler)(int,void *,struct pt_regs *),
		unsigned long flags,const char *device,void *dev_id);
int comedi_change_irq_flags(unsigned int irq,void *dev_id,unsigned long new_flags);
void comedi_free_irq(unsigned int irq,void *dev_id);
#else
#define comedi_request_irq		request_irq
#define comedi_change_irq_flags(a,b,c)	/* */
#define comedi_free_irq			free_irq
#endif



/*
   various internal comedi functions
 */

int do_rangeinfo_ioctl(comedi_device *dev,comedi_rangeinfo *arg);
int check_chanlist(comedi_subdevice *s,int n,unsigned int *chanlist);

/* range stuff */

#include <range.h>

#define RANGE_digital			RANGE_unipolar5

extern unsigned int comedi_max_range;
extern comedi_krange comedi_kranges[];


/* timer types */
/* these *must* agree with lib/timer.c */

#define TIMER_dt282x			1
#define TIMER_dt2814			2
#define TIMER_atmio			3
#define TIMER_acl8112			4
#define TIMER_nanosec			5


/* some silly little inline functions */

static inline int alloc_subdevices(comedi_device *dev)
{
	int size=sizeof(comedi_subdevice)*dev->n_subdevices;

	dev->subdevices=kmalloc(size,GFP_KERNEL);
	if(!dev->subdevices)
		return -ENOMEM;
	memset(dev->subdevices,0,size);
	return 0;
}

static inline int alloc_private(comedi_device *dev,int size)
{
	dev->private=kmalloc(size,GFP_KERNEL);
	if(!dev->private)
		return -ENOMEM;
	memset(dev->private,0,size);
	return 0;
}


#ifdef LINUX_V20
#ifdef CONFIG_EXPORT
extern struct symbol_table comedi_syms;
#endif
#endif


#endif /* _COMEDI_MODULE_H */




