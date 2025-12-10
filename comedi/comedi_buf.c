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


static void comedi_buf_map_kref_release(struct kref *kref)
{
	struct comedi_buf_map *bm =
		container_of(kref, struct comedi_buf_map, refcount);
	struct comedi_buf_page *buf;
	unsigned int i;

	if (bm->page_list) {
		if (bm->dma_dir != DMA_NONE) {
			for (i =0; i < bm->n_pages; i++) {
				buf = &bm->page_list[i];
				dma_free_coherent(bm->dma_hw_dev, PAGE_SIZE,
					buf->virt_addr, buf->dma_addr);
			}
		} else {
			for (i = 0; i < bm->n_pages; i++) {
				buf = &bm->page_list[i];
				ClearPageReserved(virt_to_page(buf->virt_addr));
				free_page((unsigned long)buf->virt_addr);
			}
		}
		vfree(bm->page_list);
	}
	if (bm->dma_dir != DMA_NONE)
		put_device(bm->dma_hw_dev);
	kfree(bm);
}

static void __comedi_buf_free(comedi_device *dev, comedi_subdevice *s)
{
	comedi_async *async = s->async;
	struct comedi_buf_map *bm;
	unsigned long flags;

	async->prealloc_bufsz = 0;
	spin_lock_irqsave(&s->spin_lock, flags);
	bm = async->buf_map;
	async->buf_map = NULL;
	spin_unlock_irqrestore(&s->spin_lock, flags);
	comedi_buf_map_put(bm);
}

static struct comedi_buf_map *
comedi_buf_map_alloc(comedi_device *dev, enum dma_data_direction dma_dir,
	unsigned int n_pages)
{
	struct comedi_buf_map *bm;
	struct comedi_buf_page *buf;
	unsigned int i;

	bm = kzalloc(sizeof(*bm), GFP_KERNEL);
	if (!bm)
		return NULL;

	kref_init(&bm->refcount);
	bm->dma_dir = dma_dir;
	if (bm->dma_dir != DMA_NONE) {
		/* Need ref to hardware device to free buffer later. */
		bm->dma_hw_dev = get_device(dev->hw_dev);
	}

	bm->page_list = vzalloc(sizeof(*buf) * n_pages);
	if (!bm->page_list)
		goto err;

	if (bm->dma_dir != DMA_NONE) {
		for (i = 0; i < n_pages; i++) {
			buf = &bm->page_list[i];
			buf->virt_addr =
				dma_alloc_coherent(bm->dma_hw_dev, PAGE_SIZE,
					&buf->dma_addr, GFP_KERNEL);
			if (!buf->virt_addr)
				break;
		}
	} else {
		for (i = 0; i < n_pages; i++) {
			buf = &bm->page_list[i];
			buf->virt_addr = (void *)get_zeroed_page(GFP_KERNEL);
			if (!buf->virt_addr)
				break;

			SetPageReserved(virt_to_page(buf->virt_addr));
		}
	}
	bm->n_pages = i;
	if (i < n_pages)
		goto err;

	return bm;

err:
	comedi_buf_map_put(bm);
	return NULL;
}

static void __comedi_buf_alloc(comedi_device *dev, comedi_subdevice *s,
	unsigned int n_pages)
{
	comedi_async *async = s->async;
	struct comedi_buf_map *bm;
	unsigned long flags;

	bm = comedi_buf_map_alloc(dev, s->async_dma_dir, n_pages);
	if (!bm)
		return;

	spin_lock_irqsave(&s->spin_lock, flags);
	async->buf_map = bm;
	spin_unlock_irqrestore(&s->spin_lock, flags);
	async->prealloc_bufsz = n_pages << PAGE_SHIFT;
}

void comedi_buf_map_get(struct comedi_buf_map *bm)
{
	if (bm)
		kref_get(&bm->refcount);
}

int comedi_buf_map_put(struct comedi_buf_map *bm)
{
	if (bm)
		return kref_put(&bm->refcount, comedi_buf_map_kref_release);
	return 1;
}

/* helper for "access" vm operation */
int comedi_buf_map_access(struct comedi_buf_map *bm, unsigned long offset,
	void *buf, int len, int write)
{
	unsigned int pgoff = offset_in_page(offset);
	unsigned long pg = offset >> PAGE_SHIFT;
	int done = 0;

	while (done < len && pg < bm->n_pages) {
		int l = min_t(int, len - done, PAGE_SIZE - pgoff);
		void *b = bm->page_list[pg].virt_addr + pgoff;

		if (write)
			memcpy(b, buf, l);
		else
			memcpy(buf, b, l);
		buf += l;
		done += l;
		pg++;
		pgoff = 0;
	}
	return done;
}

/* returns s->async->buf_map and increments its kref refcount */
struct comedi_buf_map *
comedi_buf_map_from_subdev_get(comedi_subdevice *s)
{
	comedi_async *async = s->async;
	struct comedi_buf_map *bm = NULL;
	unsigned long flags;

	if (!async)
		return NULL;

	spin_lock_irqsave(&s->spin_lock, flags);
	bm = async->buf_map;
	/* only want it if buffer pages allocated */
	if (bm && bm->n_pages)
		comedi_buf_map_get(bm);
	else
		bm = NULL;
	spin_unlock_irqrestore(&s->spin_lock, flags);

	return bm;
}

bool comedi_buf_is_mmapped(comedi_subdevice *s)
{
	struct comedi_buf_map *bm = s->async->buf_map;

	return bm && (kref_read(&bm->refcount) > 1);
}

int comedi_buf_alloc(comedi_device * dev, comedi_subdevice * s,
	unsigned long new_size)
{
	comedi_async *async = s->async;

	/* Round up new_size to multiple of PAGE_SIZE */
	new_size = (new_size + PAGE_SIZE - 1) & PAGE_MASK;

	/* if no change is required, do nothing */
	if (async->prealloc_bufsz == new_size) {
		return 0;
	}
	/* deallocate old buffer */
	__comedi_buf_free(dev, s);

	/* allocate new buffer */
	if (new_size) {
		unsigned int n_pages = new_size >> PAGE_SHIFT;

		__comedi_buf_alloc(dev, s, n_pages);
		if (!async->prealloc_bufsz)
			return -ENOMEM;
	}

	return 0;
}

/* munging is applied to data by core as it passes between user
 * and kernel space */
static unsigned int comedi_buf_munge(comedi_subdevice *s,
	unsigned int num_bytes)
{
	comedi_async *async = s->async;
	struct comedi_buf_page *buf_page_list = async->buf_map->page_list;
	unsigned int count = 0;
	const unsigned num_sample_bytes = comedi_bytes_per_sample(s);

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
			buf_page_list[block_page].virt_addr +
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

static unsigned int comedi_buf_write_n_unalloc(const comedi_subdevice *s)
{
	const comedi_async *async = s->async;
	unsigned int free_end = async->buf_read_count + async->prealloc_bufsz;

	return free_end - async->buf_write_alloc_count;
}

unsigned int comedi_buf_write_n_available(const comedi_subdevice *s)
{
	const comedi_async *async = s->async;
	unsigned int free_end;
	unsigned int nbytes;

	if (async == NULL)
		return 0;

	free_end = async->buf_read_count + async->prealloc_bufsz;
	nbytes = free_end - async->buf_write_count;
	/* barrier insures the read of buf_read_count in this
	   query occurs before any following writes to the buffer which
	   might be based on the return value from this query.
	 */
	smp_mb();
	return nbytes;
}

static unsigned int __comedi_buf_write_alloc(comedi_subdevice *s,
	unsigned int nbytes, bool strict)
{
	comedi_async *async = s->async;
	unsigned int unalloc = comedi_buf_write_n_unalloc(s);

	if (nbytes > unalloc)
		nbytes = strict ? 0 : unalloc;

	async->buf_write_alloc_count += nbytes;
	/*
	 * barrier ensures the async buffer 'counts' are read and updated
	 * before we write data to the write-alloc'ed buffer space
	 */
	smp_mb();
	return nbytes;
}

/* allocates chunk for the writer from free buffer space */
unsigned int comedi_buf_write_alloc(comedi_subdevice *s, unsigned int nbytes)
{
	return __comedi_buf_write_alloc(s, nbytes, false);
}

/* allocates nothing unless it can completely fulfill the request */
unsigned int comedi_buf_write_alloc_strict(comedi_subdevice *s,
	unsigned int nbytes)
{
	return __comedi_buf_write_alloc(s, nbytes, true);
}

/* transfers a chunk from writer to filled buffer space */
unsigned comedi_buf_write_free(comedi_subdevice *s, unsigned int nbytes)
{
	comedi_async *async = s->async;
	unsigned int allocated = comedi_buf_write_n_allocated(s);

	if (nbytes > allocated) {
		rt_printk("comedi: attempted to write-free more bytes than have been write-allocated.\n");
		nbytes = allocated;
	}

	async->buf_write_count += nbytes;
	async->buf_write_ptr += nbytes;
	comedi_buf_munge(s, async->buf_write_count - async->munge_count);
	if (async->buf_write_ptr >= async->prealloc_bufsz) {
		async->buf_write_ptr %= async->prealloc_bufsz;
	}
	return nbytes;
}

/* allocates a chunk for the reader from filled (and munged) buffer space */
unsigned comedi_buf_read_alloc(comedi_subdevice *s, unsigned nbytes)
{
	comedi_async *async = s->async;
	unsigned int available =
		async->munge_count - async->buf_read_alloc_count;

	if (nbytes > available)
		nbytes = available;

	async->buf_read_alloc_count += nbytes;
	/* barrier insures read of munge_count occurs before we actually read
	   data out of buffer */
	smp_rmb();
	return nbytes;
}

/* transfers control of a chunk from reader to free buffer space */
unsigned comedi_buf_read_free(comedi_subdevice *s, unsigned int nbytes)
{
	comedi_async *async = s->async;
	unsigned int allocated;

	// barrier insures data has been read out of buffer before read count is incremented
	smp_mb();

	allocated = comedi_buf_read_n_allocated(s);
	if (nbytes > allocated) {
		rt_printk("comedi: attempted to read-free more bytes than have been read-allocated.\n");
		nbytes = allocated;
	}

	async->buf_read_count += nbytes;
	async->buf_read_ptr += nbytes;
	async->buf_read_ptr %= async->prealloc_bufsz;
	return nbytes;
}

void comedi_buf_memcpy_to(comedi_subdevice *s, unsigned int offset,
	const void *data, unsigned int num_bytes)
{
	comedi_async *async = s->async;
	struct comedi_buf_page *buf_page_list = async->buf_map->page_list;
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

		memcpy(buf_page_list[block_page].virt_addr +
				block_page_offset, data, block_size);

		data += block_size;
		num_bytes -= block_size;
		write_ptr += block_size;
		if (write_ptr == async->prealloc_bufsz)
			write_ptr = 0;
	}
}

void comedi_buf_memcpy_from(comedi_subdevice *s, unsigned int offset,
	void *dest, unsigned int nbytes)
{
	comedi_async *async = s->async;
	struct comedi_buf_page *buf_page_list = async->buf_map->page_list;
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

		memcpy(dest, buf_page_list[block_page].virt_addr +
				block_page_offset, block_size);
		nbytes -= block_size;
		dest += block_size;
		read_ptr += block_size;
		if (read_ptr == async->prealloc_bufsz)
			read_ptr = 0;
	}
}

unsigned int comedi_buf_read_n_available(const comedi_subdevice *s)
{
	const comedi_async *async = s->async;
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

int comedi_buf_get(comedi_subdevice *s, sampl_t *x)
{
	comedi_async *async = s->async;
	unsigned int n = comedi_buf_read_n_available(s);
	unsigned int buf_page;
	unsigned int buf_page_offset;

	if (n < sizeof(sampl_t))
		return 0;
	comedi_buf_read_alloc(s, sizeof(sampl_t));
	buf_page = async->buf_read_ptr >> PAGE_SHIFT;
	buf_page_offset = async->buf_read_ptr & ~PAGE_MASK;
	*x = *(sampl_t *) (async->buf_map->page_list[buf_page].virt_addr +
			buf_page_offset);
	comedi_buf_read_free(s, sizeof(sampl_t));
	return 1;
}

int comedi_buf_getl(comedi_subdevice *s, lsampl_t *x)
{
	comedi_async *async = s->async;
	unsigned int n = comedi_buf_read_n_available(s);
	unsigned int buf_page;
	unsigned int buf_page_offset;

	if (n < sizeof(lsampl_t))
		return 0;
	comedi_buf_read_alloc(s, sizeof(lsampl_t));
	buf_page = async->buf_read_ptr >> PAGE_SHIFT;
	buf_page_offset = async->buf_read_ptr & ~PAGE_MASK;
	*x = *(lsampl_t *) (async->buf_map->page_list[buf_page].virt_addr +
			buf_page_offset);
	comedi_buf_read_free(s, sizeof(lsampl_t));
	return 1;
}

int comedi_buf_put(comedi_subdevice *s, sampl_t x)
{
	comedi_async *async = s->async;
	unsigned int n = comedi_buf_write_alloc_strict(s, sizeof(sampl_t));
	unsigned int buf_page;
	unsigned int buf_page_offset;

	if (n < sizeof(sampl_t)) {
		async->events |= COMEDI_CB_ERROR;
		return 0;
	}
	buf_page = async->buf_write_ptr >> PAGE_SHIFT;
	buf_page_offset = async->buf_write_ptr & ~PAGE_MASK;
	*(sampl_t *) (async->buf_map->page_list[buf_page].virt_addr +
			buf_page_offset) = x;
	comedi_buf_write_free(s, sizeof(sampl_t));
	return 1;
}

int comedi_buf_putl(comedi_subdevice *s, lsampl_t x)
{
	comedi_async *async = s->async;
	unsigned int n = comedi_buf_write_alloc_strict(s, sizeof(lsampl_t));
	unsigned int buf_page;
	unsigned int buf_page_offset;

	if (n < sizeof(lsampl_t)) {
		async->events |= COMEDI_CB_ERROR;
		return 0;
	}
	buf_page = async->buf_write_ptr >> PAGE_SHIFT;
	buf_page_offset = async->buf_write_ptr & ~PAGE_MASK;
	*(lsampl_t *) (async->buf_map->page_list[buf_page].virt_addr +
			buf_page_offset) = x;
	comedi_buf_write_free(s, sizeof(lsampl_t));
	return 1;
}

void comedi_buf_reset(comedi_subdevice *s)
{
	comedi_async *async = s->async;

	async->buf_write_alloc_count = 0;
	async->buf_write_count = 0;
	async->buf_read_alloc_count = 0;
	async->buf_read_count = 0;

	async->buf_write_ptr = 0;
	async->buf_read_ptr = 0;

	async->cur_chan = 0;
	async->scans_done = 0;
	async->scan_progress = 0;
	async->munge_chan = 0;
	async->munge_count = 0;
	async->munge_ptr = 0;

	async->events = 0;
}

/*
 * comedi_buf_write_samples() - Write sample data to COMEDI buffer
 * @s: COMEDI subdevice.
 * @data: Pointer to source samples.
 * @nsamples: Number of samples to write.
 *
 * Write up to @nsamples samples to the COMEDI acquisition data buffer
 * associated with the subdevice, mark it as written and update the
 * acquisition scan progress.  If there is not enough room for the specified
 * number of samples, the number of samples written is limited to the number
 * that will fit and the %COMEDI_CB_OVERFLOW event flag is set to cause the
 * acquisition to terminate with an overrun error.  Set the %COMEDI_CB_BLOCK
 * event flag if any samples are written to cause waiting tasks to be woken
 * when the event flags are processed.
 *
 * Return: The amount of data written in bytes.
 */
unsigned int comedi_buf_write_samples(comedi_subdevice *s, const void *data,
	unsigned int nsamples)
{
	unsigned int max_samples;
	unsigned int nbytes;

	/*
	 * Make sure there is enough room in the buffer for all the samples.
	 * If not, clamp the nsamples to the number that will fit, flag the
	 * buffer overrun and add the samples that fit.
	 */
	max_samples = comedi_bytes_to_samples(s, comedi_buf_write_n_unalloc(s));
	if (nsamples > max_samples) {
		rt_printk("comedi: buffer overrun\n");
		s->async->events |= COMEDI_CB_OVERFLOW;
		nsamples = max_samples;
	}

	if (nsamples == 0)
		return 0;

	nbytes = comedi_buf_write_alloc(s,
					comedi_samples_to_bytes(s, nsamples));
	comedi_buf_memcpy_to(s, 0, data, nbytes);
	comedi_buf_write_free(s, nbytes);
	comedi_inc_scan_progress(s, nbytes);
	s->async->events |= COMEDI_CB_BLOCK;

	return nbytes;
}

/*
 * comedi_buf_read_samples() - Read sample data from COMEDI buffer
 * @s: COMEDI subdevice.
 * @data: Pointer to destination.
 * @nsamples: Maximum number of samples to read.
 *
 * Read up to @nsamples samples from the COMEDI acquisition data buffer
 * associated with the subdevice, mark it as read and update the acquisition
 * scan progress.  Limit the number of samples read to the number available.
 * Set the %COMEDI_CB_BLOCK event flag if any samples are read to cause waiting
 * tasks to be woken when the event flags are processed.
 *
 * Return: The amount of data read in bytes.
 */
unsigned int comedi_buf_read_samples(comedi_subdevice *s, void *data,
	unsigned int nsamples)
{
	unsigned int max_samples;
	unsigned int nbytes;

	/* clamp nsamples to the number of full samples available */
	max_samples = comedi_bytes_to_samples(s,
					      comedi_buf_read_n_available(s));
	if (nsamples > max_samples)
		nsamples = max_samples;

	if (nsamples == 0)
		return 0;

	nbytes = comedi_buf_read_alloc(s,
				       comedi_samples_to_bytes(s, nsamples));
	comedi_buf_memcpy_from(s, 0, data, nbytes);
	comedi_buf_read_free(s, nbytes);
	comedi_inc_scan_progress(s, nbytes);
	s->async->events |= COMEDI_CB_BLOCK;

	return nbytes;
}
