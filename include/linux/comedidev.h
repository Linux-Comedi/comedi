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
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
#include <linux/config.h>
#endif
#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/mutex.h>
#include <linux/kref.h>
#include <linux/wait.h>
#include <linux/mm.h>
#include <linux/init.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <asm/io.h>

#include <linux/comedi.h>

#include <linux/comedi-config.h>

#define DPRINTK(format, args...)	do{				\
	if(comedi_debug)printk("comedi: " format , ## args );		\
} while(0)

#define COMEDI_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#define COMEDI_VERSION_CODE COMEDI_VERSION(COMEDI_MAJORVERSION,COMEDI_MINORVERSION,COMEDI_MICROVERSION)

#define COMEDI_INITCLEANUP_NOMODULE(x)					\
	static int __init x ## _init_module(void)			\
		{return comedi_driver_register(&(x));}			\
	static void __exit x ## _cleanup_module(void)			\
		{comedi_driver_unregister(&(x));} 			\
	module_init(x ## _init_module);					\
	module_exit(x ## _cleanup_module);					\

#define COMEDI_MODULE_MACROS						\
	MODULE_AUTHOR("Comedi http://www.comedi.org");		\
	MODULE_DESCRIPTION("Comedi low-level driver");			\
	MODULE_LICENSE("GPL");						\
	MODULE_VERSION(COMEDI_RELEASE);

#define COMEDI_INITCLEANUP(x)						\
	COMEDI_MODULE_MACROS		\
	COMEDI_INITCLEANUP_NOMODULE(x)

#define COMEDI_PCI_INITCLEANUP_NOMODULE(comedi_driver, pci_id_table) \
	static int __devinit comedi_driver ## _pci_probe(struct pci_dev *dev, \
		const struct pci_device_id *ent) \
	{ \
		return comedi_pci_auto_config(dev, &(comedi_driver), \
			(ent)->driver_data); \
	} \
	static void __devexit comedi_driver ## _pci_remove(struct pci_dev *dev) \
	{ \
		comedi_pci_auto_unconfig(dev); \
	} \
	static struct pci_driver comedi_driver ## _pci_driver = \
	{ \
		.id_table = pci_id_table, \
		.probe = & comedi_driver ## _pci_probe, \
		.remove = __devexit_p(& comedi_driver ## _pci_remove) \
	}; \
	static int __init comedi_driver ## _init_module(void) \
	{ \
		int retval; \
		retval = comedi_driver_register(& comedi_driver); \
		if(retval < 0) return retval; \
		comedi_driver ## _pci_driver.name = (char*)comedi_driver.driver_name; \
		return pci_register_driver(& comedi_driver ## _pci_driver); \
	} \
	static void __exit comedi_driver ## _cleanup_module(void) \
	{ \
		pci_unregister_driver(& comedi_driver ## _pci_driver); \
		comedi_driver_unregister(& comedi_driver); \
	} \
	module_init(comedi_driver ## _init_module); \
	module_exit(comedi_driver ## _cleanup_module);

#define COMEDI_PCI_INITCLEANUP(comedi_driver, pci_id_table) \
	COMEDI_MODULE_MACROS \
	COMEDI_PCI_INITCLEANUP_NOMODULE(comedi_driver, pci_id_table)

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

#define COMEDI_NUM_MINORS 0x100
#define COMEDI_NUM_BOARD_MINORS 0x30

/* callback stuff */
#define COMEDI_CB_EOS		1	/* end of scan */
#define COMEDI_CB_EOA		2	/* end of acquisition/output */
#define COMEDI_CB_BLOCK		4	/* new data has arrived: wakes up write()/read() */
#define COMEDI_CB_EOBUF		8	/* DEPRECATED: end of buffer */
#define COMEDI_CB_ERROR		16	/* card error during acquisition */
#define COMEDI_CB_OVERFLOW	32	/* buffer overflow/underflow */

typedef struct comedi_device_struct comedi_device;
typedef struct comedi_subdevice_struct comedi_subdevice;
typedef struct comedi_async_struct comedi_async;
typedef struct comedi_driver_struct comedi_driver;
typedef struct comedi_lrange_struct comedi_lrange;

struct comedi_subdevice_struct {
	comedi_device *device;
	int type;
	int n_chan;
	volatile int subdev_flags;
	int len_chanlist;	/* maximum length of channel/gain list */

	void *private;

	comedi_async *async;

	void *lock;
	void *busy;
	unsigned runflags;
	spinlock_t spin_lock;

	int io_bits;

	lsampl_t maxdata;	/* if maxdata==0, use list */
	const lsampl_t *maxdata_list;	/* list is channel specific */

	const comedi_lrange *range_table;
	const comedi_lrange *const *range_table_list;

	unsigned int *chanlist;	/* driver-owned chanlist (not used) */

	int (*insn_read) (comedi_device *, comedi_subdevice *, comedi_insn *,
		lsampl_t *);
	int (*insn_write) (comedi_device *, comedi_subdevice *, comedi_insn *,
		lsampl_t *);
	int (*insn_bits) (comedi_device *, comedi_subdevice *, comedi_insn *,
		lsampl_t *);
	int (*insn_config) (comedi_device *, comedi_subdevice *, comedi_insn *,
		lsampl_t *);

	int (*do_cmd) (comedi_device *, comedi_subdevice *);
	int (*do_cmdtest) (comedi_device *, comedi_subdevice *, comedi_cmd *);
	int (*poll) (comedi_device *, comedi_subdevice *);
	int (*cancel) (comedi_device *, comedi_subdevice *);
	//int (*do_lock)(comedi_device *,comedi_subdevice *);
	//int (*do_unlock)(comedi_device *,comedi_subdevice *);

	/* called when the buffer changes */
	int (*buf_change) (comedi_device * dev, comedi_subdevice * s,
		unsigned long new_size);

	void (*munge) (comedi_device * dev, comedi_subdevice * s, void *data,
		unsigned int num_bytes, unsigned int start_chan_index);
	enum dma_data_direction async_dma_dir;

	unsigned int state;

	comedi_device_create_t *class_dev;
	int minor;
};

struct comedi_buf_page {
	void *virt_addr;
	dma_addr_t dma_addr;
};

struct comedi_async_struct {
	comedi_subdevice *subdevice;

	unsigned int prealloc_bufsz;	/* buffer size, in bytes */
	struct comedi_buf_page *buf_page_list;	/* virtual and dma address of each page */
	unsigned n_buf_pages;	/* num elements in buf_page_list */

	unsigned int max_bufsize;	/* maximum buffer size, bytes */
	unsigned int mmap_count;	/* current number of mmaps of prealloc_buf */

	unsigned int buf_write_count;	/* byte count for writer (write completed) */
	unsigned int buf_write_alloc_count;	/* byte count for writer (allocated for writing) */
	unsigned int buf_read_count;	/* byte count for reader (read completed) */
	unsigned int buf_read_alloc_count;	/* byte count for reader (allocated for reading) */

	unsigned int buf_write_ptr;	/* buffer marker for writer */
	unsigned int buf_read_ptr;	/* buffer marker for reader */

	unsigned int cur_chan;	/* useless channel marker for interrupt */
	/* number of bytes that have been received for current scan */
	unsigned int scan_progress;
	/* keeps track of where we are in chanlist as for munging */
	unsigned int munge_chan;
	/* number of bytes that have been munged */
	unsigned int munge_count;
	/* buffer marker for munging */
	unsigned int munge_ptr;

	unsigned int events;	/* events that have occurred */

	comedi_cmd cmd;

	wait_queue_head_t wait_head;

	// callback stuff
	unsigned int cb_mask;
	int (*cb_func) (unsigned int flags, void *);
	void *cb_arg;

	int (*inttrig) (comedi_device * dev, comedi_subdevice * s,
		unsigned int x);
};

struct comedi_driver_struct {
	struct comedi_driver_struct *next;

	const char *driver_name;
	struct module *module;
	int (*attach) (comedi_device *, comedi_devconfig *);
	int (*detach) (comedi_device *);
	int (*auto_attach)(comedi_device *, unsigned long);

	/* number of elements in board_name and board_id arrays */
	unsigned int num_names;
	const char *const *board_name;
	/* offset in bytes from one board name pointer to the next */
	int offset;
};

struct comedi_device_struct {
	int use_count;
	comedi_driver *driver;
	void *private;

	comedi_device_create_t *class_dev;
	int minor;
	unsigned int detach_count;
	/* hw_dev is passed to dma_alloc_coherent when allocating async buffers for subdevices
	   that have async_dma_dir set to something other than DMA_NONE */
	struct device *hw_dev;

	const char *board_name;
	const void *board_ptr;
	int attached;
	int rt;
	spinlock_t spinlock;
	struct mutex mutex;
	struct rw_semaphore attach_lock;
	struct kref refcount;
	int in_request_module;

	int n_subdevices;
	comedi_subdevice *subdevices;

	/* dumb */
	unsigned long iobase;
	unsigned int irq;

	comedi_subdevice *read_subdev;
	comedi_subdevice *write_subdev;

	struct fasync_struct *async_queue;

	int (*open) (comedi_device * dev);
	void (*close) (comedi_device * dev);
};

#ifdef COMEDI_CONFIG_DEBUG
extern int comedi_debug;
#else
static const int comedi_debug = 0;
#endif

/*
 * function prototypes
 */

void comedi_event(comedi_device * dev, comedi_subdevice * s);
void comedi_error(const comedi_device * dev, const char *s);

comedi_device *comedi_dev_get_from_minor(unsigned minor);
int comedi_dev_put(comedi_device *dev);

int comedi_driver_register(comedi_driver *);
void comedi_driver_unregister(comedi_driver *);

/* subdevice runflags */
enum subdevice_runflags {
	SRF_USER = 0x00000001,
	SRF_RT = 0x00000002,
	/* indicates an COMEDI_CB_ERROR event has occurred since the last command was started */
	SRF_ERROR = 0x00000004,
	SRF_RUNNING = 0x08000000
};

/*
   various internal comedi functions
 */

int check_chanlist(comedi_subdevice * s, int n, unsigned int *chanlist);
void comedi_update_subdevice_runflags(comedi_subdevice * s, unsigned mask,
	unsigned bits);
unsigned comedi_get_subdevice_runflags(comedi_subdevice * s);
bool comedi_is_subdevice_running(comedi_subdevice *s);

/* range stuff */

#define RANGE(a,b)		{(a)*1e6,(b)*1e6,0}
#define RANGE_ext(a,b)		{(a)*1e6,(b)*1e6,RF_EXTERNAL}
#define RANGE_mA(a,b)		{(a)*1e6,(b)*1e6,UNIT_mA}
#define RANGE_unitless(a,b)	{(a)*1e6,(b)*1e6,0}	/* XXX */
#define BIP_RANGE(a)		{-(a)*1e6,(a)*1e6,0}
#define UNI_RANGE(a)		{0,(a)*1e6,0}

extern const comedi_lrange range_bipolar10;
extern const comedi_lrange range_bipolar5;
extern const comedi_lrange range_bipolar2_5;
extern const comedi_lrange range_unipolar10;
extern const comedi_lrange range_unipolar5;
extern const comedi_lrange range_unknown;

#define range_digital		range_unipolar5

#if __GNUC__ >= 3
#define GCC_ZERO_LENGTH_ARRAY
#else
#define GCC_ZERO_LENGTH_ARRAY 0
#endif

struct comedi_lrange_struct {
	int length;
	comedi_krange range[GCC_ZERO_LENGTH_ARRAY];
};

/* some silly little inline functions */

static inline int alloc_subdevices(comedi_device * dev,
	unsigned int num_subdevices)
{
	unsigned i;

	dev->n_subdevices = num_subdevices;
	dev->subdevices =
		kcalloc(num_subdevices, sizeof(comedi_subdevice), GFP_KERNEL);
	if (!dev->subdevices)
		return -ENOMEM;
	for (i = 0; i < num_subdevices; ++i) {
		dev->subdevices[i].device = dev;
		dev->subdevices[i].async_dma_dir = DMA_NONE;
		spin_lock_init(&dev->subdevices[i].spin_lock);
		dev->subdevices[i].minor = -1;
	}
	return 0;
}

static inline int alloc_private(comedi_device * dev, int size)
{
	dev->private = kzalloc(size, GFP_KERNEL);
	if (!dev->private)
		return -ENOMEM;
	return 0;
}

static inline unsigned int comedi_bytes_per_sample(const comedi_subdevice *subd)
{
	return subd->subdev_flags & SDF_LSAMPL ? sizeof(lsampl_t)
					       : sizeof(sampl_t);
}

/* to be removed */
static inline unsigned int bytes_per_sample(const comedi_subdevice *subd)
{
	return comedi_bytes_per_sample(subd);
}

static inline unsigned int comedi_sample_shift(const comedi_subdevice *subd)
{
	return subd->subdev_flags & SDF_LSAMPL ? 2 : 1;
}

static inline unsigned int comedi_bytes_to_samples(const comedi_subdevice *subd,
	unsigned int nbytes)
{
	return nbytes >> comedi_sample_shift(subd);
}

static inline unsigned int comedi_samples_to_bytes(const comedi_subdevice *subd,
	unsigned int nsamples)
{
	return nsamples << comedi_sample_shift(subd);
}

/* must be used in attach to set dev->hw_dev if you wish to dma directly
into comedi's buffer */
int comedi_set_hw_dev(comedi_device * dev, struct device *hw_dev);

int comedi_buf_put(comedi_async * async, sampl_t x);
int comedi_buf_putl(comedi_async * async, lsampl_t x);
int comedi_buf_get(comedi_async * async, sampl_t * x);
int comedi_buf_getl(comedi_async * async, lsampl_t * x);

unsigned int comedi_buf_write_n_available(comedi_async * async);
unsigned int comedi_buf_write_alloc(comedi_async * async, unsigned int nbytes);
unsigned comedi_buf_write_free(comedi_async * async, unsigned int nbytes);
unsigned comedi_buf_read_alloc(comedi_async * async, unsigned nbytes);
unsigned comedi_buf_read_free(comedi_async * async, unsigned int nbytes);
unsigned int comedi_buf_read_n_available(comedi_async * async);
void comedi_buf_memcpy_to(comedi_async * async, unsigned int offset,
	const void *source, unsigned int num_bytes);
void comedi_buf_memcpy_from(comedi_async * async, unsigned int offset,
	void *destination, unsigned int num_bytes);
static inline unsigned comedi_buf_write_n_allocated(comedi_async * async)
{
	return async->buf_write_alloc_count - async->buf_write_count;
}
static inline unsigned comedi_buf_read_n_allocated(comedi_async * async)
{
	return async->buf_read_alloc_count - async->buf_read_count;
}

void comedi_reset_async_buf(comedi_async * async);

static inline void *comedi_aux_data(int options[], int n)
{
	unsigned long address;
	unsigned long addressLow;
	int bit_shift;
	if (sizeof(int) >= sizeof(void *))
		address = options[COMEDI_DEVCONF_AUX_DATA_LO];
	else {
		address = options[COMEDI_DEVCONF_AUX_DATA_HI];
		bit_shift = sizeof(int) * 8;
		address <<= bit_shift;
		addressLow = options[COMEDI_DEVCONF_AUX_DATA_LO];
		addressLow &= (1UL << bit_shift) - 1;
		address |= addressLow;
	}
	if (n >= 1)
		address += options[COMEDI_DEVCONF_AUX_DATA0_LENGTH];
	if (n >= 2)
		address += options[COMEDI_DEVCONF_AUX_DATA1_LENGTH];
	if (n >= 3)
		address += options[COMEDI_DEVCONF_AUX_DATA2_LENGTH];
	BUG_ON(n > 3);
	return (void *)address;
}

int comedi_auto_config(struct device *hardware_device, comedi_driver *driver,
		       unsigned long context);
void comedi_auto_unconfig(struct device *hardware_device);
int comedi_pci_auto_config(struct pci_dev *pcidev, comedi_driver *driver,
	unsigned long context);
void comedi_pci_auto_unconfig(struct pci_dev *pcidev);
struct usb_interface;	// forward declaration
int comedi_usb_auto_config(struct usb_interface *intf, comedi_driver *driver,
	unsigned long context);
void comedi_usb_auto_unconfig(struct usb_interface *intf);

//#ifdef COMEDI_CONFIG_RT
#include <linux/comedi_rt.h>
//#endif

#endif /* _COMEDIDEV_H */
