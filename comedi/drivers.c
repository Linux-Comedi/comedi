/*
    module/drivers.c
    functions for manipulating drivers

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


#define __NO_VERSION__
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/comedidev.h>

#include <asm/io.h>

#include "kvmem.h"

static int postconfig(comedi_device *dev);
static int insn_rw_emulate_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int insn_inval(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static void *comedi_recognize(comedi_driver *driv, const char *name);
static void comedi_report_boards(comedi_driver *driv);
static int poll_invalid(comedi_device *dev,comedi_subdevice *s);
static int buf_alloc(comedi_device *dev, comedi_subdevice *s,
	unsigned long new_size);

comedi_driver *comedi_drivers;

int comedi_modprobe(kdev_t minor)
{
	return -EINVAL;
}

int comedi_device_detach(comedi_device *dev)
{
	int i;
	comedi_subdevice *s;

	if(!dev->attached)
		return 0;

	/* this is not correct for the kmod case */
	if(dev->driver->module)
		__MOD_DEC_USE_COUNT(dev->driver->module);

	dev->attached=0;

	for(i=0;i<dev->n_subdevices;i++){
		s=dev->subdevices+i;
		if(s->async){
			if(s->buf_alloc) s->buf_alloc(dev,s,0);
			kfree(s->async);
		}
	}

	if(dev->driver){
		dev->driver->detach(dev);
	}else{
		printk("BUG: dev->driver=NULL in comedi_device_detach()\n");
	}

	if(dev->subdevices)kfree(dev->subdevices);
	if(dev->private)kfree(dev->private);

	return 0;
}

int comedi_device_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_driver *driv;
	int ret;
	int minor;
	int use_count;

	if(dev->attached)
		return -EBUSY;

	minor = dev->minor;
	use_count = dev->use_count;
	memset(dev,0,sizeof(comedi_device));
	spin_lock_init(&dev->spinlock);
	dev->minor=minor;
	dev->use_count = use_count;

	for(driv=comedi_drivers;driv;driv=driv->next){
		if(driv->num_names){
			dev->board_ptr=comedi_recognize(driv, it->board_name);
			if(dev->board_ptr==NULL) continue;
		}else{
			if(strcmp(driv->driver_name,it->board_name))
				continue;
		}
		//initialize dev->driver here so comedi_error() can be called from attach
		dev->driver=driv;
		ret=driv->attach(dev,it);
		if(ret<0){
			driv->detach(dev);
			if(dev->subdevices)kfree(dev->subdevices);
			if(dev->private)kfree(dev->private);

			return ret;
		}
		goto attached;
	}

	// recognize has failed if we get here
	// report valid board names before returning error
	for(driv=comedi_drivers;driv;driv=driv->next){
		comedi_report_boards(driv);
	}
	return -EIO;

attached:
	/* do a little post-config cleanup */
	ret = postconfig(dev);
	if(ret < 0)
	{
		driv->detach(dev);
		if(dev->subdevices)kfree(dev->subdevices);
		if(dev->private)kfree(dev->private);

		return ret;
	}

	init_waitqueue_head(&dev->read_wait);
	init_waitqueue_head(&dev->write_wait);

	if(!dev->board_name){
		printk("BUG: dev->board_name=<%p>\n",dev->board_name);
		dev->board_name="BUG";
	}

	dev->attached=1;

	if(driv->module)
		__MOD_INC_USE_COUNT(driv->module);

	return 0;
}

int comedi_driver_register(comedi_driver *driver)
{
	driver->next=comedi_drivers;
	comedi_drivers=driver;

	return 0;
}

int comedi_driver_unregister(comedi_driver *driver)
{
	comedi_driver *prev;
	int i;

	/* check for devices using this driver */
	for(i=0;i<COMEDI_NDEVICES;i++){
		comedi_device *dev;

		dev=comedi_get_device_by_minor(i);
		if(dev->attached && dev->driver==driver){
			if(dev->use_count)
				printk("BUG! detaching device with use_count=%d\n",dev->use_count);
			comedi_device_detach(dev);
		}
	}

	if(comedi_drivers==driver){
		comedi_drivers=driver->next;
		return 0;
	}

	for(prev=comedi_drivers;prev->next;prev=prev->next){
		if(prev->next==driver){
			prev->next=driver->next;
			return 0;
		}
	}
	return -EINVAL;
}

comedi_device *comedi_allocate_dev(comedi_driver *driver)
{
	comedi_device *dev;

	// XXX we need to do actual allocation here.
	
	dev=comedi_get_device_by_minor(0);

	dev->driver=driver;

	return dev;
}

void comedi_deallocate_dev(comedi_device *dev)
{

}

static int postconfig(comedi_device *dev)
{
	int i;
	comedi_subdevice *s;
	comedi_async *async = NULL;

	for(i=0;i<dev->n_subdevices;i++){
		s=dev->subdevices+i;

		if(s->type==COMEDI_SUBD_UNUSED)
			continue;

		if(s->len_chanlist==0)
			s->len_chanlist=1; 

		if(s->do_cmd){
			async = kmalloc(sizeof(comedi_async), GFP_KERNEL);
			if(async == NULL)
			{
				printk("failed to allocate async struct\n");
				return -ENOMEM;
			}
			memset(async, 0, sizeof(comedi_async));
			s->async = async;

#define DEFAULT_BUF_MAXSIZE (64*1024)
#define DEFAULT_BUF_SIZE (16*1024)

			async->max_bufsize = DEFAULT_BUF_MAXSIZE;
			if(!s->buf_alloc) s->buf_alloc = buf_alloc;

			async->prealloc_buf = NULL;
			async->prealloc_bufsz = 0;
			if(s->buf_alloc(dev,s,DEFAULT_BUF_SIZE) < 0){
				printk("Buffer allocation failed\n");
				return -ENOMEM;
			}
		}

		if(!s->range_table && !s->range_table_list)
			s->range_table=&range_unknown;

		if(!s->insn_read && s->insn_bits)
			s->insn_read = insn_rw_emulate_bits;
		if(!s->insn_write && s->insn_bits)
			s->insn_write = insn_rw_emulate_bits;

		if(!s->insn_read)s->insn_read = insn_inval;
		if(!s->insn_write)s->insn_write = insn_inval;
		if(!s->insn_bits)s->insn_bits = insn_inval;

		if(!s->poll)s->poll=poll_invalid;
	}

	return 0;
}

// generic recognize function for drivers that register their supported board names
void *comedi_recognize(comedi_driver *driv, const char *name)
{
	unsigned int i = 0;
	void *name_ptr;

	name_ptr=driv->board_name;
	for(i = 0; i < driv->num_names; i++)
	{
		if(strcmp(*(char **)name_ptr, name) == 0)
			return name_ptr;
		name_ptr += driv->offset;
	}

	return NULL;
}

void comedi_report_boards(comedi_driver *driv)
{
	unsigned int i;
	void *name_ptr;

	printk("comedi: valid board names for %s driver are:\n", driv->driver_name);

	name_ptr=driv->board_name;
	for(i = 0; i < driv->num_names; i++)
	{
		printk(" %s\n", *(char **)name_ptr);
		name_ptr += driv->offset;
	}

	if(driv->num_names == 0)
		printk(" %s\n", driv->driver_name);
}

static int poll_invalid(comedi_device *dev,comedi_subdevice *s)
{
	return -EINVAL;
}

static int insn_inval(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	return -EINVAL;
}

static int insn_rw_emulate_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	comedi_insn new_insn;
	int ret;
	lsampl_t new_data[2];
	unsigned int chan;

	chan = CR_CHAN(insn->chanspec);

	memset(&new_insn,0,sizeof(new_insn));
	new_insn.insn = INSN_BITS;
	new_insn.n = 2;
	new_insn.data = new_data;
	new_insn.subdev = insn->subdev;

	if(insn->insn == INSN_WRITE){
		if(!(s->subdev_flags & SDF_WRITABLE))
			return -EINVAL;
		new_data[0] = 1<<chan; /* mask */
		new_data[1] = data[0]?(1<<chan):0; /* bits */
	}else {
		new_data[0] = 0;
		new_data[1] = 0;
	}

	ret = s->insn_bits(dev,s,&new_insn,new_data);
	if(ret<0)return ret;

	if(insn->insn == INSN_READ){
		data[0] = (new_data[1]>>chan)&1;
	}

	return 1;
}

/*
 * Default allocator for Comedi buffers
 * Override function should allocate a new buffer of size new_size,
 * and update async->prealloc_buf and async->prealloc_bufsz.  Set
 * NULL and 0 if the allocation fails.  If new_size is 0, deallocate
 * the old buffer and don't allocate a new one.
 */
static int buf_alloc(comedi_device *dev, comedi_subdevice *s,
	unsigned long new_size)
{
	comedi_async *async = s->async;

	/* if no change is required, do nothing */
	if(async->prealloc_buf && async->prealloc_bufsz == new_size){
		return 0;
	}

	if(async->prealloc_bufsz){
		rvfree(async->prealloc_buf, async->prealloc_bufsz);
		async->prealloc_buf = NULL;
	}

	if(new_size){
		async->prealloc_buf = rvmalloc(new_size);
		if(async->prealloc_buf == NULL){
			async->prealloc_bufsz = 0;
			return -ENOMEM;
		}
	}
	async->prealloc_bufsz = new_size;

	return 0;
}


unsigned int comedi_buf_write_alloc(comedi_async *async, unsigned int nbytes)
{
	unsigned int free_end = async->buf_read_count + async->data_len;

	if((int)(async->buf_free_count + nbytes - free_end) > 0){
		nbytes = free_end - async->buf_free_count;
	}

	async->buf_free_count += nbytes;

	return nbytes;
}

unsigned int comedi_buf_write_alloc_strict(comedi_async *async,
	unsigned int nbytes)
{
	unsigned int free_end = async->buf_read_count + async->data_len;

	if((int)(async->buf_free_count + nbytes - free_end) > 0){
		nbytes = 0;
	}

	async->buf_free_count += nbytes;

	return nbytes;
}

/* transfers control of a chunk from writer to reader */
void comedi_buf_write_free(comedi_async *async, unsigned int nbytes)
{
	async->buf_write_count += nbytes;
	async->buf_write_ptr += nbytes;
	if(async->buf_write_ptr >= async->data_len){
		async->buf_write_ptr -= async->data_len;
		async->events |= COMEDI_CB_EOBUF;
	}
}

/* transfers control of a chunk from reader to free area */
void comedi_buf_read_free(comedi_async *async, unsigned int nbytes)
{
	async->buf_read_count += nbytes;
	async->buf_read_ptr += nbytes;
	if(async->buf_read_ptr >= async->data_len){
		async->buf_read_ptr -= async->data_len;
	}
}

void comedi_buf_memcpy_to( comedi_async *async, unsigned int offset, const void *data,
	unsigned int num_bytes )
{
	unsigned int write_ptr = async->buf_write_ptr + offset;

	if( write_ptr >= async->data_len )
		write_ptr -= async->data_len;

	while( num_bytes )
	{
		unsigned int block_size;

		if( write_ptr + num_bytes > async->data_len)
			block_size = async->data_len - write_ptr;
		else
			block_size = num_bytes;

		memcpy( async->data + write_ptr, data, block_size );

		data += block_size;
		num_bytes -= block_size;

		write_ptr = 0;
	}
}

void comedi_buf_memcpy_from(comedi_async *async, unsigned int offset,
	void *dest, unsigned int nbytes)
{
	void *src;
	unsigned int read_ptr = async->buf_read_ptr + offset;

	if( read_ptr >= async->data_len )
		read_ptr -= async->data_len;

	while( nbytes )
	{
		unsigned int block_size;

		src = async->data + read_ptr;

		if( nbytes >= async->data_len - read_ptr )
			block_size = async->data_len - read_ptr;
		else
			block_size = nbytes;

		memcpy(dest, src, block_size );
		nbytes -= block_size;
		dest += block_size;
		read_ptr = 0;
	}
}

unsigned int comedi_buf_read_n_available(comedi_async *async)
{
	unsigned int read_end = async->buf_write_count;

	return read_end - async->buf_read_count;
}

int comedi_buf_get(comedi_async *async, sampl_t *x)
{
	unsigned int n = comedi_buf_read_n_available(async);

	if(n<sizeof(sampl_t))return 0;
	*x = *(sampl_t *)(async->prealloc_buf + async->buf_read_ptr);
	comedi_buf_read_free(async, sizeof(sampl_t));
	return 1;
}

int comedi_buf_put(comedi_async *async, sampl_t x)
{
	unsigned int n = comedi_buf_write_alloc_strict(async, sizeof(sampl_t));

	if(n<sizeof(sampl_t)){
		async->events |= COMEDI_CB_ERROR;
		return 0;
	}
	*(sampl_t *)(async->prealloc_buf + async->buf_write_ptr) = x;
	comedi_buf_write_free(async, sizeof(sampl_t));
	return 1;
}

