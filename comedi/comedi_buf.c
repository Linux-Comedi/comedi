/*
    module/comedi_buf.c
    functions for manipulating data buffers for asynchronous commands

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


#include <linux/device.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/comedidev.h>
#include <linux/vmalloc.h>
#include <linux/dma-mapping.h>

#include <asm/barrier.h>

#include "comedi_internal.h"


int comedi_buf_alloc(comedi_device * dev, comedi_subdevice * s,
	unsigned long new_size)
{
	comedi_async *async = s->async;
	struct comedi_buf_page *buf;

	/* Round up new_size to multiple of PAGE_SIZE */
	new_size = (new_size + PAGE_SIZE - 1) & PAGE_MASK;

	/* if no change is required, do nothing */
	if (async->prealloc_bufsz == new_size) {
		return 0;
	}
	// deallocate old buffer
	async->prealloc_bufsz = 0;
	if (async->buf_page_list) {
		unsigned i;
		for (i = 0; i < async->n_buf_pages; ++i) {
			buf = async->buf_page_list + i;
			if (buf->virt_addr) {
				if (s->async_dma_dir != DMA_NONE) {
					dma_free_coherent(dev->hw_dev,
						PAGE_SIZE, buf->virt_addr,
						buf->dma_addr);
				} else {
					ClearPageReserved(
						virt_to_page(buf->virt_addr));
					free_page((unsigned long)buf->
						virt_addr);
				}
			}
		}
		vfree(async->buf_page_list);
		async->buf_page_list = NULL;
		async->n_buf_pages = 0;
	}
	// allocate new buffer
	if (new_size) {
		unsigned i = 0;
		unsigned n_pages = new_size >> PAGE_SHIFT;

		async->buf_page_list =
			vmalloc(sizeof(struct comedi_buf_page) * n_pages);
		if (async->buf_page_list) {
			memset(async->buf_page_list, 0,
				sizeof(struct comedi_buf_page) * n_pages);
			for (i = 0; i < n_pages; i++) {
				buf = async->buf_page_list + i;
				if (s->async_dma_dir != DMA_NONE) {
					buf->virt_addr =
						dma_alloc_coherent(dev->hw_dev,
							PAGE_SIZE,
							&buf->dma_addr,
							GFP_KERNEL);
				} else {
					buf->virt_addr =
						(void *)
						get_zeroed_page(GFP_KERNEL);
				}
				if (buf->virt_addr == NULL) {
					break;
				}
				if (s->async_dma_dir == DMA_NONE) {
					SetPageReserved(
						virt_to_page(buf->virt_addr));
				}
			}
		}
		if (i < n_pages) {
			/* Some allocation failed above. */
			if (async->buf_page_list) {
				for (i = 0; i < n_pages; i++) {
					buf = async->buf_page_list + i;
					if (buf->virt_addr == NULL) {
						break;
					}
					if (s->async_dma_dir != DMA_NONE) {
						dma_free_coherent(dev->hw_dev,
							PAGE_SIZE,
							buf->virt_addr,
							buf->dma_addr);
					} else {
						ClearPageReserved(virt_to_page(
							buf->virt_addr));
						free_page((unsigned long)buf->
							virt_addr);
					}
				}
				vfree(async->buf_page_list);
				async->buf_page_list = NULL;
			}
			return -ENOMEM;
		}
		async->n_buf_pages = n_pages;
	}
	async->prealloc_bufsz = new_size;

	return 0;
}

/* munging is applied to data by core as it passes between user
 * and kernel space */
static unsigned int comedi_buf_munge(comedi_async * async,
	unsigned int num_bytes)
{
	comedi_subdevice *s = async->subdevice;
	unsigned int count = 0;
	const unsigned num_sample_bytes = bytes_per_sample(s);

	if (s->munge == NULL || (async->cmd.flags & CMDF_RAWDATA)) {
		async->munge_count += num_bytes;
		if ((int)(async->munge_count - async->buf_write_count) > 0)
			BUG();
		return num_bytes;
	}
	/* don't munge partial samples */
	num_bytes -= num_bytes % num_sample_bytes;
	while (count < num_bytes) {
		unsigned int block_page = async->munge_ptr >> PAGE_SHIFT;
		unsigned int block_page_offset = async->munge_ptr & ~PAGE_MASK;
		unsigned int block_page_left = PAGE_SIZE - block_page_offset;
		unsigned int block_size;

		block_size = num_bytes - count;
		/*
		 * Only munge to the end of the page this iteration.
		 * Note that prealloc_bufsz is a multiple of the page size.
		 */
		if (block_size > block_page_left) {
			block_size = block_page_left;
		}
		s->munge(s->device, s,
			async->buf_page_list[block_page].virt_addr +
				block_page_offset, block_size,
			async->munge_chan);

		smp_wmb();	//barrier insures data is munged in buffer before munge_count is incremented

		async->munge_chan += block_size / num_sample_bytes;
		async->munge_chan %= async->cmd.chanlist_len;
		async->munge_count += block_size;
		async->munge_ptr += block_size;
		async->munge_ptr %= async->prealloc_bufsz;
		count += block_size;
	}
	if ((int)(async->munge_count - async->buf_write_count) > 0)
		BUG();
	return count;
}

unsigned int comedi_buf_write_n_available(comedi_async * async)
{
	unsigned int free_end;
	unsigned int nbytes;

	if (async == NULL)
		return 0;

	free_end = async->buf_read_count + async->prealloc_bufsz;
	nbytes = free_end - async->buf_write_alloc_count;
	nbytes -= nbytes % bytes_per_sample(async->subdevice);
	/* barrier insures the read of buf_read_count in this
	   query occurs before any following writes to the buffer which
	   might be based on the return value from this query.
	 */
	smp_mb();
	return nbytes;
}

/* allocates chunk for the writer from free buffer space */
unsigned int comedi_buf_write_alloc(comedi_async * async, unsigned int nbytes)
{
	unsigned int free_end = async->buf_read_count + async->prealloc_bufsz;

	if ((int)(async->buf_write_alloc_count + nbytes - free_end) > 0) {
		nbytes = free_end - async->buf_write_alloc_count;
	}
	async->buf_write_alloc_count += nbytes;
	/* barrier insures the read of buf_read_count above occurs before
	   we write data to the write-alloc'ed buffer space */
	smp_mb();
	return nbytes;
}

/* allocates nothing unless it can completely fulfill the request */
unsigned int comedi_buf_write_alloc_strict(comedi_async * async,
	unsigned int nbytes)
{
	unsigned int free_end = async->buf_read_count + async->prealloc_bufsz;

	if ((int)(async->buf_write_alloc_count + nbytes - free_end) > 0) {
		nbytes = 0;
	}
	async->buf_write_alloc_count += nbytes;
	/* barrier insures the read of buf_read_count above occurs before
	   we write data to the write-alloc'ed buffer space */
	smp_mb();
	return nbytes;
}

/* transfers a chunk from writer to filled buffer space */
unsigned comedi_buf_write_free(comedi_async * async, unsigned int nbytes)
{
	if ((int)(async->buf_write_count + nbytes -
			async->buf_write_alloc_count) > 0) {
		rt_printk
			("comedi: attempted to write-free more bytes than have been write-allocated.\n");
		nbytes = async->buf_write_alloc_count - async->buf_write_count;
	}
	async->buf_write_count += nbytes;
	async->buf_write_ptr += nbytes;
	comedi_buf_munge(async, async->buf_write_count - async->munge_count);
	if (async->buf_write_ptr >= async->prealloc_bufsz) {
		async->buf_write_ptr %= async->prealloc_bufsz;
	}
	return nbytes;
}

/* allocates a chunk for the reader from filled (and munged) buffer space */
unsigned comedi_buf_read_alloc(comedi_async * async, unsigned nbytes)
{
	if ((int)(async->buf_read_alloc_count + nbytes - async->munge_count) >
		0) {
		nbytes = async->munge_count - async->buf_read_alloc_count;
	}
	async->buf_read_alloc_count += nbytes;
	/* barrier insures read of munge_count occurs before we actually read
	   data out of buffer */
	smp_rmb();
	return nbytes;
}

/* transfers control of a chunk from reader to free buffer space */
unsigned comedi_buf_read_free(comedi_async * async, unsigned int nbytes)
{
	// barrier insures data has been read out of buffer before read count is incremented
	smp_mb();
	if ((int)(async->buf_read_count + nbytes -
			async->buf_read_alloc_count) > 0) {
		rt_printk
			("comedi: attempted to read-free more bytes than have been read-allocated.\n");
		nbytes = async->buf_read_alloc_count - async->buf_read_count;
	}
	async->buf_read_count += nbytes;
	async->buf_read_ptr += nbytes;
	async->buf_read_ptr %= async->prealloc_bufsz;
	return nbytes;
}

void comedi_buf_memcpy_to(comedi_async * async, unsigned int offset,
	const void *data, unsigned int num_bytes)
{
	unsigned int write_ptr = async->buf_write_ptr + offset;

	if (write_ptr >= async->prealloc_bufsz)
		write_ptr %= async->prealloc_bufsz;

	while (num_bytes) {
		unsigned int block_page = write_ptr >> PAGE_SHIFT;
		unsigned int block_page_offset = write_ptr & ~PAGE_MASK;
		unsigned int block_page_left = PAGE_SIZE - block_page_offset;
		unsigned int block_size;

		if (num_bytes > block_page_left)
			block_size = block_page_left;
		else
			block_size = num_bytes;

		memcpy(async->buf_page_list[block_page].virt_addr +
				block_page_offset, data, block_size);

		data += block_size;
		num_bytes -= block_size;
		write_ptr += block_size;
		if (write_ptr == async->prealloc_bufsz)
			write_ptr = 0;
	}
}

void comedi_buf_memcpy_from(comedi_async * async, unsigned int offset,
	void *dest, unsigned int nbytes)
{
	unsigned int read_ptr = async->buf_read_ptr + offset;

	if (read_ptr >= async->prealloc_bufsz)
		read_ptr %= async->prealloc_bufsz;

	while (nbytes) {
		unsigned int block_page = read_ptr >> PAGE_SHIFT;
		unsigned int block_page_offset = read_ptr & ~PAGE_MASK;
		unsigned int block_page_left = PAGE_SIZE - block_page_offset;
		unsigned int block_size;

		if (nbytes > block_page_left)
			block_size = block_page_left;
		else
			block_size = nbytes;

		memcpy(dest, async->buf_page_list[block_page].virt_addr +
				block_page_offset, block_size);
		nbytes -= block_size;
		dest += block_size;
		read_ptr += block_size;
		if (read_ptr == async->prealloc_bufsz)
			read_ptr = 0;
	}
}

unsigned int comedi_buf_read_n_available(comedi_async * async)
{
	unsigned num_bytes;

	if (async == NULL)
		return 0;
	num_bytes = async->munge_count - async->buf_read_count;
	/* barrier insures the read of munge_count in this
	   query occurs before any following reads of the buffer which
	   might be based on the return value from this query.
	 */
	smp_rmb();
	return num_bytes;
}

int comedi_buf_get(comedi_async * async, sampl_t * x)
{
	unsigned int n = comedi_buf_read_n_available(async);
	unsigned int buf_page;
	unsigned int buf_page_offset;

	if (n < sizeof(sampl_t))
		return 0;
	comedi_buf_read_alloc(async, sizeof(sampl_t));
	buf_page = async->buf_read_ptr >> PAGE_SHIFT;
	buf_page_offset = async->buf_read_ptr & ~PAGE_MASK;
	*x = *(sampl_t *) (async->buf_page_list[buf_page].virt_addr +
			buf_page_offset);
	comedi_buf_read_free(async, sizeof(sampl_t));
	return 1;
}

int comedi_buf_getl(comedi_async * async, lsampl_t * x)
{
	unsigned int n = comedi_buf_read_n_available(async);
	unsigned int buf_page;
	unsigned int buf_page_offset;

	if (n < sizeof(lsampl_t))
		return 0;
	comedi_buf_read_alloc(async, sizeof(lsampl_t));
	buf_page = async->buf_read_ptr >> PAGE_SHIFT;
	buf_page_offset = async->buf_read_ptr & ~PAGE_MASK;
	*x = *(lsampl_t *) (async->buf_page_list[buf_page].virt_addr +
			buf_page_offset);
	comedi_buf_read_free(async, sizeof(lsampl_t));
	return 1;
}

int comedi_buf_put(comedi_async * async, sampl_t x)
{
	unsigned int n = comedi_buf_write_alloc_strict(async, sizeof(sampl_t));
	unsigned int buf_page;
	unsigned int buf_page_offset;

	if (n < sizeof(sampl_t)) {
		async->events |= COMEDI_CB_ERROR;
		return 0;
	}
	buf_page = async->buf_write_ptr >> PAGE_SHIFT;
	buf_page_offset = async->buf_write_ptr & ~PAGE_MASK;
	*(sampl_t *) (async->buf_page_list[buf_page].virt_addr +
			buf_page_offset) = x;
	comedi_buf_write_free(async, sizeof(sampl_t));
	return 1;
}

int comedi_buf_putl(comedi_async * async, lsampl_t x)
{
	unsigned int n = comedi_buf_write_alloc_strict(async, sizeof(lsampl_t));
	unsigned int buf_page;
	unsigned int buf_page_offset;

	if (n < sizeof(lsampl_t)) {
		async->events |= COMEDI_CB_ERROR;
		return 0;
	}
	buf_page = async->buf_write_ptr >> PAGE_SHIFT;
	buf_page_offset = async->buf_write_ptr & ~PAGE_MASK;
	*(lsampl_t *) (async->buf_page_list[buf_page].virt_addr +
			buf_page_offset) = x;
	comedi_buf_write_free(async, sizeof(lsampl_t));
	return 1;
}

void comedi_reset_async_buf(comedi_async * async)
{
	async->buf_write_alloc_count = 0;
	async->buf_write_count = 0;
	async->buf_read_alloc_count = 0;
	async->buf_read_count = 0;

	async->buf_write_ptr = 0;
	async->buf_read_ptr = 0;

	async->cur_chan = 0;
	async->scan_progress = 0;
	async->munge_chan = 0;
	async->munge_count = 0;
	async->munge_ptr = 0;

	async->events = 0;
}
