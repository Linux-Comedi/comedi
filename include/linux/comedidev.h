/*
    include/linux/comedidev.h
    header file for kernel-only structures, variables, and constants

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>

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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/config.h>
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/interrupt.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#include <linux/comedi.h>

#include <config.h>


#define DPRINTK(format, args...)	do{				\
	if(comedi_debug)printk("comedi: " format , ## args );		\
} while(0)

#define COMEDI_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#define COMEDI_VERSION_CODE COMEDI_VERSION(COMEDI_MAJORVERSION,COMEDI_MINORVERSION,COMEDI_MICROVERSION)
#define COMEDI_RELEASE VERSION

#define COMEDI_INITCLEANUP_NOMODULE(x)					\
	static int __init x ## _init_module(void)			\
		{return comedi_driver_register(&(x));}			\
	static void __exit x ## _cleanup_module(void)			\
		{comedi_driver_unregister(&(x));} 			\
	module_init(x ## _init_module);					\
	module_exit(x ## _cleanup_module);					\

#define COMEDI_INITCLEANUP(x)						\
	MODULE_AUTHOR("David A. Schleef <ds@schleef.org>");		\
	MODULE_DESCRIPTION("Comedi low-level driver");			\
	MODULE_LICENSE("GPL");						\
	static int __init x ## _init_module(void)			\
		{return comedi_driver_register(&(x));}			\
	static void __exit x ## _cleanup_module(void)			\
		{comedi_driver_unregister(&(x));} 			\
	module_init(x ## _init_module);					\
	module_exit(x ## _cleanup_module);					\


#define PCI_VENDOR_ID_INOVA		0x104c
#define PCI_VENDOR_ID_NATINST		0x1093
#define PCI_VENDOR_ID_DATX		0x1116
#define PCI_VENDOR_ID_COMPUTERBOARDS	0x1307
#define PCI_VENDOR_ID_ADVANTECH		0x13fe
#define PCI_VENDOR_ID_RTD		0x1435
#define PCI_VENDOR_ID_AMPLICON		0x14dc
#define PCI_VENDOR_ID_ADLINK		0x144a
#define PCI_VENDOR_ID_ICP		0x104c
#define PCI_VENDOR_ID_CONTEC		0x1221
#define PCI_VENDOR_ID_MEILHAUS		0x1402


typedef struct comedi_device_struct comedi_device;
typedef struct comedi_subdevice_struct comedi_subdevice;
typedef struct comedi_async_struct comedi_async;
typedef struct comedi_driver_struct comedi_driver;
typedef struct comedi_lrange_struct comedi_lrange;


struct comedi_subdevice_struct{
	int type;
	int n_chan;
	volatile int subdev_flags;
	int len_chanlist;		/* maximum length of channel/gain list */

	void		*private;

	comedi_async *async;

	void *lock;
	void *busy;
	volatile unsigned int runflags;

	int io_bits;

	lsampl_t maxdata;		/* if maxdata==0, use list */
	lsampl_t *maxdata_list;		/* list is channel specific */

	unsigned int flags;
	unsigned int *flaglist;

	unsigned int settling_time_0;

	comedi_lrange *range_table;
	comedi_lrange **range_table_list;

	unsigned int *chanlist;		/* driver-owned chanlist (not used) */

	int (*insn_read)(comedi_device *,comedi_subdevice *,comedi_insn *,lsampl_t *);
	int (*insn_write)(comedi_device *,comedi_subdevice *,comedi_insn *,lsampl_t *);
	int (*insn_bits)(comedi_device *,comedi_subdevice *,comedi_insn *,lsampl_t *);
	int (*insn_config)(comedi_device *,comedi_subdevice *,comedi_insn *,lsampl_t *);

	int (*do_cmd)(comedi_device *,comedi_subdevice *);
	int (*do_cmdtest)(comedi_device *,comedi_subdevice *,comedi_cmd *);
	int (*poll)(comedi_device *,comedi_subdevice *);
	int (*cancel)(comedi_device *,comedi_subdevice *);
	//int (*do_lock)(comedi_device *,comedi_subdevice *);
	//int (*do_unlock)(comedi_device *,comedi_subdevice *);

	/* called when the buffer changes */
	int (*buf_change)(comedi_device *,comedi_subdevice *s,unsigned long new_size);

	void (*munge)( comedi_device *, comedi_subdevice *s, void *data,
		unsigned int num_bytes, unsigned int start_chan_index );

	unsigned int state;
};

struct comedi_async_struct{
	void		*prealloc_buf;		/* pre-allocated buffer */
	unsigned int	prealloc_bufsz;		/* buffer size, in bytes */
	unsigned long	*buf_page_list;		/* physical address of each page */
	unsigned int	max_bufsize;		/* maximum buffer size, bytes */
	unsigned int	mmap_count;	/* current number of mmaps of prealloc_buf */

	volatile unsigned int buf_write_count;	/* byte count for writer (write completed) */
	volatile unsigned int buf_write_alloc_count;	/* byte count for writer (allocated for writing) */
	volatile unsigned int buf_read_count;	/* byte count for reader (read completed)*/

	unsigned int buf_write_ptr;	/* buffer marker for writer */
	unsigned int buf_read_ptr;	/* buffer marker for reader */

	unsigned int cur_chan;		/* useless channel marker for interrupt */
	/* number of bytes that have been received for current scan */
	unsigned int scan_progress;
	/* keeps track of where we are in chanlist as for munging */
	unsigned int munge_chan;
	/* number of bytes that have been munged */
	unsigned int munge_count;
	/* buffer marker for munging */
	unsigned int munge_ptr;

	unsigned int	events;		/* events that have occurred */

	comedi_cmd cmd;

	// callback stuff
	unsigned int cb_mask;
	int (*cb_func)(unsigned int flags,void *);
	void *cb_arg;

	int (*inttrig)(comedi_device *dev,comedi_subdevice *s,unsigned int x);
};

struct comedi_driver_struct{
	struct comedi_driver_struct *next;

	char *driver_name;
	struct module *module;
	int (*attach)(comedi_device *,comedi_devconfig *);
	int (*detach)(comedi_device *);

	/* number of elements in board_name and board_id arrays */
	unsigned int num_names;
	void *board_name;
	/* offset in bytes from one board name pointer to the next */
	int offset;
};

struct comedi_device_struct{
	int use_count;
	comedi_driver *driver;
	void *private;
	unsigned int minor;
	char *board_name;
	const void * board_ptr;
	int attached;
	int rt;
	spinlock_t spinlock;
	int in_request_module;

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

	void (*open)(comedi_device *dev);
	void (*close)(comedi_device *dev);
};



extern comedi_device *comedi_devices;
extern spinlock_t big_comedi_lock;

#ifdef CONFIG_COMEDI_DEBUG
extern int comedi_debug;
#else
static const int comedi_debug = 0;
#endif

/*
 * function prototypes
 */

void comedi_event(comedi_device *dev,comedi_subdevice *s,unsigned int mask);
void comedi_error(const comedi_device *dev,const char *s);

static inline comedi_device * comedi_get_device_by_minor( unsigned int minor )
{
	return comedi_devices + minor;
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

int comedi_buf_alloc(comedi_device *dev, comedi_subdevice *s, unsigned long
	new_size);

#ifdef CONFIG_PROC_FS
void comedi_proc_init(void);
void comedi_proc_cleanup(void);
#else
static inline void comedi_proc_init(void) {}
static inline void comedi_proc_cleanup(void) {}
#endif

/* subdevice runflags */
#define SRF_USER		0x00000001
#define SRF_RT			0x00000002
/* indicates an COMEDI_CB_ERROR event has occurred since the last command was started */
#define SRF_ERROR		0x00000004

/*
   various internal comedi functions
 */

int do_rangeinfo_ioctl(comedi_device *dev,comedi_rangeinfo *arg);
int check_chanlist(comedi_subdevice *s,int n,unsigned int *chanlist);

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

#if __GNUC__ >= 3
#define GCC_ZERO_LENGTH_ARRAY
#else
#define GCC_ZERO_LENGTH_ARRAY 0
#endif

struct comedi_lrange_struct{
	int length;
	comedi_krange range[ GCC_ZERO_LENGTH_ARRAY ];
};



/* some silly little inline functions */

static inline int alloc_subdevices(comedi_device *dev, unsigned int num_subdevices)
{
	int size=sizeof(comedi_subdevice)*num_subdevices;

	dev->n_subdevices = num_subdevices;
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

static inline unsigned int bytes_per_sample( const comedi_subdevice *subd )
{
	if( subd->flags & SDF_LSAMPL )
		return sizeof( lsampl_t );
	else
		return sizeof( sampl_t );
}

int comedi_buf_put(comedi_async *async, sampl_t x);
int comedi_buf_get(comedi_async *async, sampl_t *x);

unsigned int comedi_buf_write_n_available(comedi_subdevice *s);
unsigned int comedi_buf_write_alloc(comedi_async *async, unsigned int nbytes);
unsigned int comedi_buf_write_alloc_strict(comedi_async *async, unsigned int nbytes);
void comedi_buf_write_free(comedi_async *async, unsigned int nbytes);
void comedi_buf_read_free(comedi_async *async, unsigned int nbytes);
unsigned int comedi_buf_read_n_available(comedi_subdevice *s);
void comedi_buf_memcpy_to( comedi_async *async, unsigned int offset, const void *source,
	unsigned int num_bytes );
void comedi_buf_memcpy_from( comedi_async *async, unsigned int offset, void *destination,
	unsigned int num_bytes );
unsigned int comedi_buf_munge( comedi_device *dev, comedi_subdevice *s,
	unsigned int num_bytes );

//#ifdef CONFIG_COMEDI_RT
#include <linux/comedi_rt.h>
//#endif


#endif /* _COMEDIDEV_H */




