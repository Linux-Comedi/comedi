/*
    include/linux/comedidev.h
    header file for kernel-only structures, variables, and constants

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

#ifndef _COMEDIDEV_H
#define _COMEDIDEV_H

#include <linux/version.h>
#include <linux/config.h>
#include <linux/kdev_t.h>
#include <linux/malloc.h>
#include <linux/errno.h>
#include <linux/comedi.h>


#include "kern_compat.h"

#ifdef CONFIG_COMEDI_DEBUG
#define DPRINTK(format, args...)	printk("comedi: " format , ## args )
#else
#define DPRINTK(format, args...)	/* */
#endif

#ifndef COMEDI_VERSION_CODE
#define COMEDI_VERSION_CODE 0x10000
#define COMEDI_VERSION "1.0.0"
#endif

#ifdef MODULE
#define COMEDI_INITCLEANUP(x)						\
	int init_module(void){return comedi_driver_register(&(x));}	\
	void cleanup_module(void){comedi_driver_unregister(&(x));}
#else
#define COMEDI_INITCLEANUP(x)
#endif
typedef struct comedi_device_struct comedi_device;
typedef struct comedi_subdevice_struct comedi_subdevice;
typedef struct comedi_async_struct comedi_async;
typedef struct comedi_driver_struct comedi_driver;
typedef struct comedi_lrange_struct comedi_lrange;


struct comedi_subdevice_struct{
	int type;
	int n_chan;
	int subdev_flags;
	int len_chanlist;		/* length of channel/gain list, if available */

	void		*private;

	comedi_async *async;

	void *lock;
	void *busy;
	unsigned int runflags;

	int io_bits;

	lsampl_t maxdata;		/* if maxdata==0, use list */
	lsampl_t *maxdata_list;		/* list is channel specific */

	unsigned int flags;
	unsigned int *flaglist;

	comedi_lrange *range_table;
	comedi_lrange **range_table_list;

	unsigned int *chanlist;		/* driver-owned chanlist (not used) */

#ifdef CONFIG_COMEDI_MODE_CORE
	comedi_trig	cur_trig;	/* current trig structure */
	int (*trig[5])(comedi_device *,comedi_subdevice *,comedi_trig *);
#endif

	int (*insn_read)(comedi_device *,comedi_subdevice *,comedi_insn *,lsampl_t *);
	int (*insn_write)(comedi_device *,comedi_subdevice *,comedi_insn *,lsampl_t *);
	int (*insn_bits)(comedi_device *,comedi_subdevice *,comedi_insn *,lsampl_t *);
	int (*insn_config)(comedi_device *,comedi_subdevice *,comedi_insn *,lsampl_t *);

	int (*do_cmd)(comedi_device *,comedi_subdevice *);
	int (*do_cmdtest)(comedi_device *,comedi_subdevice *,comedi_cmd *);
	int (*poll)(comedi_device *,comedi_subdevice *);
	int (*cancel)(comedi_device *,comedi_subdevice *);
	int (*do_lock)(comedi_device *,comedi_subdevice *);
	int (*do_unlock)(comedi_device *,comedi_subdevice *);

	unsigned int state;
};

struct comedi_async_struct{
	void		*prealloc_buf;		/* pre-allocated buffer */
	unsigned int	prealloc_bufsz;		/* buffer size, in bytes */
	unsigned int	max_bufsize;		/* maximum buffer size, bytes */
	unsigned int	mmap_count;	/* current number of mmaps of prealloc_buf */

	volatile unsigned int buf_int_ptr;	/* buffer marker for interrupt */
	unsigned int buf_user_ptr;		/* buffer marker for read() and write() */
	volatile unsigned int buf_int_count;	/* byte count for interrupt */
	unsigned int buf_user_count;		/* byte count for read() and write() */
	unsigned int cur_chan;		/* useless channel marker for interrupt */
	unsigned int cur_chanlist_len;

	void		*data;
	unsigned int	data_len;

	unsigned int	events;		/* events that have occurred */

	comedi_cmd cmd;

	// callback stuff
	unsigned int cb_mask;
	int (*cb_func)(unsigned int flags,void *);
	void *cb_arg;
};

struct comedi_driver_struct{
	struct comedi_driver_struct *next;

	char *driver_name;
	struct module *module;
	int (*attach)(comedi_device *,comedi_devconfig *);
	int (*detach)(comedi_device *);

	// number of elements in board_name and board_id arrays
	unsigned int num_names;
	void *board_name;
	// offset in bytes from one board name pointer to the next
	int offset;
};

struct comedi_device_struct{
	int use_count;
	comedi_driver *driver;
	void *private;
	kdev_t minor;
	char *board_name;
	//int board;
	void *board_ptr;
	int attached;
	int rt;
	spinlock_t spinlock;

	int n_subdevices;
	comedi_subdevice *subdevices;
	int options[COMEDI_NDEVCONFOPTS];

	/* dumb */
	int iobase;
	int irq;

	comedi_subdevice *read_subdev;
	wait_queue_head_t read_wait;

	comedi_subdevice *write_subdev;
	wait_queue_head_t write_wait;

	struct fasync_struct *async_queue;
};



extern comedi_device *comedi_devices;

/*
 * function prototypes
 */

void comedi_event(comedi_device *dev,comedi_subdevice *s,unsigned int mask);
void comedi_error(comedi_device *dev,const char *s);
void comedi_done(comedi_device *dev,comedi_subdevice *s);
void comedi_error_done(comedi_device *dev,comedi_subdevice *s);
void comedi_eos(comedi_device *dev,comedi_subdevice *s);
void comedi_eobuf(comedi_device *dev,comedi_subdevice *s);
void comedi_bufcheck(comedi_device *dev,comedi_subdevice *s);

comedi_device * comedi_get_device_by_minor(kdev_t minor);

extern inline comedi_device * comedi_get_device_by_minor(kdev_t minor)
{
	return comedi_devices+minor;
}

int comedi_device_detach(comedi_device *dev);
int comedi_device_attach(comedi_device *dev,comedi_devconfig *it);
int comedi_driver_register(comedi_driver *);
int comedi_driver_unregister(comedi_driver *);

comedi_device *comedi_allocate_dev(comedi_driver *);
void comedi_deallocate_dev(comedi_device *);

void init_polling(void);
void cleanup_polling(void);
void start_polling(comedi_device *);
void stop_polling(comedi_device *);

void comedi_proc_init(void);
void comedi_proc_cleanup(void);

#ifdef CONFIG_COMEDI_MODE_CORE
int di_unpack(unsigned int bits,comedi_trig *it);
int do_pack(unsigned int *bits,comedi_trig *it);
#endif


#define SRF_USER		0x00000001
#define SRF_RT			0x00000002


/*
   various internal comedi functions
 */

int do_rangeinfo_ioctl(comedi_device *dev,comedi_rangeinfo *arg);
int check_chanlist(comedi_subdevice *s,int n,unsigned int *chanlist);

extern volatile int rtcomedi_lock_semaphore;

/* range stuff */


#define RANGE(a,b)		{(a)*1e6,(b)*1e6,0}
#define RANGE_ext(a,b)		{(a)*1e6,(b)*1e6,RF_EXTERNAL}
#define RANGE_mA(a,b)		{(a)*1e6,(b)*1e6,UNIT_mA}
#define RANGE_unitless(a,b)	{(a)*1e6,(b)*1e6,0}	/* XXX */
#define BIP_RANGE(a)		{-(a)*1e6,(a)*1e6,0}
#define UNI_RANGE(a)		{0,(a)*1e6,0}

extern comedi_lrange range_bipolar10;
extern comedi_lrange range_bipolar5;
extern comedi_lrange range_bipolar2_5;
extern comedi_lrange range_unipolar10;
extern comedi_lrange range_unipolar5;
extern comedi_lrange range_unknown;

#define range_digital		range_unipolar5

struct comedi_lrange_struct{
	int length;
	comedi_krange range[0];
};



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
extern struct symbol_table comedi_syms;
#endif

//#ifdef CONFIG_COMEDI_RT
#include <linux/comedi_rt.h>
//#endif


#endif /* _COMEDIDEV_H */




