/*
    comedi/comedi_fops.c
    comedi kernel module

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

#undef DEBUG

#define __NO_VERSION__
#include "comedi_fops.h"
#include "comedi_compat32.h"

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/kmod.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/comedidev.h>
#include <linux/cdev.h>

#include <asm/io.h>
#include <asm/uaccess.h>

//#include "kvmem.h"

MODULE_AUTHOR("David Schleef <ds@schleef.org>");
MODULE_DESCRIPTION("Comedi core module");
MODULE_LICENSE("GPL");

#ifdef CONFIG_COMEDI_DEBUG
int comedi_debug;
module_param(comedi_debug, int, 0644);
#endif

comedi_device *comedi_devices;
spinlock_t big_comedi_lock;	/* Dynamic initialization */

static int do_devconfig_ioctl(comedi_device * dev, comedi_devconfig * arg);
static int do_bufconfig_ioctl(comedi_device * dev, void *arg);
static int do_devinfo_ioctl(comedi_device * dev, comedi_devinfo * arg,
	struct file *file);
static int do_subdinfo_ioctl(comedi_device * dev, comedi_subdinfo * arg,
	void *file);
static int do_chaninfo_ioctl(comedi_device * dev, comedi_chaninfo * arg);
static int do_bufinfo_ioctl(comedi_device * dev, void *arg);
static int do_cmd_ioctl(comedi_device * dev, void *arg, void *file);
static int do_lock_ioctl(comedi_device * dev, unsigned int arg, void *file);
static int do_unlock_ioctl(comedi_device * dev, unsigned int arg, void *file);
static int do_cancel_ioctl(comedi_device * dev, unsigned int arg, void *file);
static int do_cmdtest_ioctl(comedi_device * dev, void *arg, void *file);
static int do_insnlist_ioctl(comedi_device * dev, void *arg, void *file);
static int do_insn_ioctl(comedi_device * dev, void *arg, void *file);
static int do_poll_ioctl(comedi_device * dev, unsigned int subd, void *file);

void do_become_nonbusy(comedi_device * dev, comedi_subdevice * s);
static int do_cancel(comedi_device * dev, comedi_subdevice * s);

static int comedi_fasync(int fd, struct file *file, int on);

static int comedi_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	const unsigned minor = iminor(inode);
	comedi_device *dev = comedi_get_device_by_minor(minor);

	/* Device config is special, because it must work on
	 * an unconfigured device. */
	if (cmd == COMEDI_DEVCONFIG) {
		return do_devconfig_ioctl(dev, (void *)arg);
	}

	if (!dev->attached) {
		DPRINTK("no driver configured on /dev/comedi%i\n", dev->minor);
		return -ENODEV;
	}

	switch (cmd) {
	case COMEDI_BUFCONFIG:
		return do_bufconfig_ioctl(dev, (void *)arg);
	case COMEDI_DEVINFO:
		return do_devinfo_ioctl(dev, (void *)arg, file);
	case COMEDI_SUBDINFO:
		return do_subdinfo_ioctl(dev, (void *)arg, file);
	case COMEDI_CHANINFO:
		return do_chaninfo_ioctl(dev, (void *)arg);
	case COMEDI_RANGEINFO:
		return do_rangeinfo_ioctl(dev, (void *)arg);
	case COMEDI_BUFINFO:
		return do_bufinfo_ioctl(dev, (void *)arg);
	case COMEDI_LOCK:
		return do_lock_ioctl(dev, arg, file);
	case COMEDI_UNLOCK:
		return do_unlock_ioctl(dev, arg, file);
	case COMEDI_CANCEL:
		return do_cancel_ioctl(dev, arg, file);
	case COMEDI_CMD:
		return do_cmd_ioctl(dev, (void *)arg, file);
	case COMEDI_CMDTEST:
		return do_cmdtest_ioctl(dev, (void *)arg, file);
	case COMEDI_INSNLIST:
		return do_insnlist_ioctl(dev, (void *)arg, file);
	case COMEDI_INSN:
		return do_insn_ioctl(dev, (void *)arg, file);
	case COMEDI_POLL:
		return do_poll_ioctl(dev, arg, file);
	default:
		return -ENOTTY;
	}
}

/*
	COMEDI_DEVCONFIG
	device config ioctl

	arg:
		pointer to devconfig structure

	reads:
		devconfig structure at arg

	writes:
		none
*/
static int do_devconfig_ioctl(comedi_device * dev, comedi_devconfig * arg)
{
	comedi_devconfig it;
	int ret;
	unsigned char *aux_data = NULL;
	int aux_len;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (arg == NULL) {
		return comedi_device_detach(dev);
	}

	if (copy_from_user(&it, arg, sizeof(comedi_devconfig)))
		return -EFAULT;

	it.board_name[COMEDI_NAMELEN - 1] = 0;

	if (comedi_aux_data(it.options, 0) &&
		it.options[COMEDI_DEVCONF_AUX_DATA_LENGTH]) {
		int bit_shift;
		aux_len = it.options[COMEDI_DEVCONF_AUX_DATA_LENGTH];
		if (aux_len < 0)
			return -EFAULT;

		aux_data = vmalloc(aux_len);
		if (!aux_data)
			return -ENOMEM;

		if (copy_from_user(aux_data,
				comedi_aux_data(it.options, 0), aux_len)) {
			vfree(aux_data);
			return -EFAULT;
		}
		it.options[COMEDI_DEVCONF_AUX_DATA_LO] =
			(unsigned long)aux_data;
		if (sizeof(void *) > sizeof(int)) {
			bit_shift = sizeof(int) * 8;
			it.options[COMEDI_DEVCONF_AUX_DATA_HI] =
				((unsigned long)aux_data) >> bit_shift;
		} else
			it.options[COMEDI_DEVCONF_AUX_DATA_HI] = 0;
	}

	ret = comedi_device_attach(dev, &it);

	if (aux_data)
		vfree(aux_data);

	return ret;
}

/*
	COMEDI_BUFCONFIG
	buffer configuration ioctl

	arg:
		pointer to bufconfig structure

	reads:
		bufconfig at arg

	writes:
		modified bufconfig at arg

*/
static int do_bufconfig_ioctl(comedi_device * dev, void *arg)
{
	comedi_bufconfig bc;
	comedi_async *async;
	comedi_subdevice *s;
	int ret = 0;

	if (copy_from_user(&bc, arg, sizeof(comedi_bufconfig)))
		return -EFAULT;

	if (bc.subdevice >= dev->n_subdevices || bc.subdevice < 0)
		return -EINVAL;

	s = dev->subdevices + bc.subdevice;
	async = s->async;

	if (!async) {
		DPRINTK("subdevice does not have async capability\n");
		bc.size = 0;
		bc.maximum_size = 0;
		goto copyback;
	}

	if (bc.maximum_size) {
		if (!capable(CAP_SYS_ADMIN))
			return -EPERM;

		async->max_bufsize = bc.maximum_size;
	}

	if (bc.size) {
		if (bc.size > async->max_bufsize)
			return -EPERM;

		if (s->busy) {
			DPRINTK("subdevice is busy, cannot resize buffer\n");
			return -EBUSY;
		}
		if (async->mmap_count) {
			DPRINTK("subdevice is mmapped, cannot resize buffer\n");
			return -EBUSY;
		}

		if (!async->prealloc_buf)
			return -EINVAL;

		/* make sure buffer is an integral number of pages
		 * (we round up) */
		bc.size = (bc.size + PAGE_SIZE - 1) & PAGE_MASK;

		ret = comedi_buf_alloc(dev, s, bc.size);
		if (ret < 0)
			return ret;

		if (s->buf_change) {
			ret = s->buf_change(dev, s, bc.size);
			if (ret < 0)
				return ret;
		}

		DPRINTK("comedi%i subd %d buffer resized to %i bytes\n",
			dev->minor, bc.subdevice, async->prealloc_bufsz);
	}

	bc.size = async->prealloc_bufsz;
	bc.maximum_size = async->max_bufsize;

      copyback:
	if (copy_to_user(arg, &bc, sizeof(comedi_bufconfig)))
		return -EFAULT;

	return 0;
}

/*
	COMEDI_DEVINFO
	device info ioctl

	arg:
		pointer to devinfo structure

	reads:
		none

	writes:
		devinfo structure

*/
static int do_devinfo_ioctl(comedi_device * dev, comedi_devinfo * arg,
	struct file *file)
{
	comedi_devinfo devinfo;
	const unsigned minor = iminor(file->f_dentry->d_inode);
	comedi_subdevice *read_subdev = comedi_get_read_subdevice(dev, minor);
	comedi_subdevice *write_subdev = comedi_get_write_subdevice(dev, minor);

	memset(&devinfo, 0, sizeof(devinfo));

	/* fill devinfo structure */
	devinfo.version_code = COMEDI_VERSION_CODE;
	devinfo.n_subdevs = dev->n_subdevices;
	memcpy(devinfo.driver_name, dev->driver->driver_name, COMEDI_NAMELEN);
	memcpy(devinfo.board_name, dev->board_name, COMEDI_NAMELEN);

	if (read_subdev) {
		devinfo.read_subdevice = read_subdev - dev->subdevices;
	} else {
		devinfo.read_subdevice = -1;
	}
	if (write_subdev) {
		devinfo.write_subdevice = write_subdev - dev->subdevices;
	} else {
		devinfo.write_subdevice = -1;
	}

	if (copy_to_user(arg, &devinfo, sizeof(comedi_devinfo)))
		return -EFAULT;

	return 0;
}

/*
	COMEDI_SUBDINFO
	subdevice info ioctl

	arg:
		pointer to array of subdevice info structures

	reads:
		none

	writes:
		array of subdevice info structures at arg

*/
static int do_subdinfo_ioctl(comedi_device * dev, comedi_subdinfo * arg,
	void *file)
{
	int ret, i;
	comedi_subdinfo *tmp, *us;
	comedi_subdevice *s;

	tmp = kcalloc(dev->n_subdevices, sizeof(comedi_subdinfo), GFP_KERNEL);
	if (!tmp)
		return -ENOMEM;

	/* fill subdinfo structs */
	for (i = 0; i < dev->n_subdevices; i++) {
		s = dev->subdevices + i;
		us = tmp + i;

		us->type = s->type;
		us->n_chan = s->n_chan;
		us->subd_flags = s->subdev_flags;
		if (comedi_get_subdevice_runflags(s) & SRF_RUNNING)
			us->subd_flags |= SDF_RUNNING;
#define TIMER_nanosec 5		/* backwards compatibility */
		us->timer_type = TIMER_nanosec;
		us->len_chanlist = s->len_chanlist;
		us->maxdata = s->maxdata;
		if (s->range_table) {
			us->range_type =
				(dev->
				minor << 28) | (i << 24) | (0 << 16) | (s->
				range_table->length);
		} else {
			us->range_type = 0;	/* XXX */
		}
		us->flags = s->flags;

		if (s->busy)
			us->subd_flags |= SDF_BUSY;
		if (s->busy == file)
			us->subd_flags |= SDF_BUSY_OWNER;
		if (s->lock)
			us->subd_flags |= SDF_LOCKED;
		if (s->lock == file)
			us->subd_flags |= SDF_LOCK_OWNER;
		if (!s->maxdata && s->maxdata_list)
			us->subd_flags |= SDF_MAXDATA;
		if (s->flaglist)
			us->subd_flags |= SDF_FLAGS;
		if (s->range_table_list)
			us->subd_flags |= SDF_RANGETYPE;
		if (s->do_cmd)
			us->subd_flags |= SDF_CMD;

		us->settling_time_0 = s->settling_time_0;
	}

	ret = copy_to_user(arg, tmp,
		dev->n_subdevices * sizeof(comedi_subdinfo));

	kfree(tmp);

	return ret ? -EFAULT : 0;
}

/*
	COMEDI_CHANINFO
	subdevice info ioctl

	arg:
		pointer to chaninfo structure

	reads:
		chaninfo structure at arg

	writes:
		arrays at elements of chaninfo structure

*/
static int do_chaninfo_ioctl(comedi_device * dev, comedi_chaninfo * arg)
{
	comedi_subdevice *s;
	comedi_chaninfo it;

	if (copy_from_user(&it, arg, sizeof(comedi_chaninfo)))
		return -EFAULT;

	if (it.subdev >= dev->n_subdevices)
		return -EINVAL;
	s = dev->subdevices + it.subdev;

	if (it.maxdata_list) {
		if (s->maxdata || !s->maxdata_list)
			return -EINVAL;
		if (copy_to_user(it.maxdata_list, s->maxdata_list,
				s->n_chan * sizeof(lsampl_t)))
			return -EFAULT;
	}

	if (it.flaglist) {
		if (!s->flaglist)
			return -EINVAL;
		if (copy_to_user(it.flaglist, s->flaglist,
				s->n_chan * sizeof(unsigned int)))
			return -EFAULT;
	}

	if (it.rangelist) {
		int i;

		if (!s->range_table_list)
			return -EINVAL;
		for (i = 0; i < s->n_chan; i++) {
			int x;

			x = (dev->minor << 28) | (it.subdev << 24) | (i << 16) |
				(s->range_table_list[i]->length);
			put_user(x, it.rangelist + i);
		}
		//if(copy_to_user(it.rangelist,s->range_type_list,s->n_chan*sizeof(unsigned int)))
		//      return -EFAULT;
	}

	return 0;
}

 /*
    COMEDI_BUFINFO
    buffer information ioctl

    arg:
    pointer to bufinfo structure

    reads:
    bufinfo at arg

    writes:
    modified bufinfo at arg

  */
static int do_bufinfo_ioctl(comedi_device * dev, void *arg)
{
	comedi_bufinfo bi;
	comedi_subdevice *s;
	comedi_async *async;

	if (copy_from_user(&bi, arg, sizeof(comedi_bufinfo)))
		return -EFAULT;

	if (bi.subdevice >= dev->n_subdevices || bi.subdevice < 0)
		return -EINVAL;

	s = dev->subdevices + bi.subdevice;
	async = s->async;

	if (!async) {
		DPRINTK("subdevice does not have async capability\n");
		bi.buf_write_ptr = 0;
		bi.buf_read_ptr = 0;
		bi.buf_write_count = 0;
		bi.buf_read_count = 0;
		goto copyback;
	}

	if (bi.bytes_read && (s->subdev_flags & SDF_CMD_READ)) {
		bi.bytes_read = comedi_buf_read_alloc(async, bi.bytes_read);
		comedi_buf_read_free(async, bi.bytes_read);

		if (!(comedi_get_subdevice_runflags(s) & (SRF_ERROR |
					SRF_RUNNING))
			&& async->buf_write_count == async->buf_read_count) {
			do_become_nonbusy(dev, s);
		}
	}

	if (bi.bytes_written && (s->subdev_flags & SDF_CMD_WRITE)) {
		bi.bytes_written =
			comedi_buf_write_alloc(async, bi.bytes_written);
		comedi_buf_write_free(async, bi.bytes_written);
	}

	bi.buf_write_count = async->buf_write_count;
	bi.buf_write_ptr = async->buf_write_ptr;
	bi.buf_read_count = async->buf_read_count;
	bi.buf_read_ptr = async->buf_read_ptr;

      copyback:
	if (copy_to_user(arg, &bi, sizeof(comedi_bufinfo)))
		return -EFAULT;

	return 0;
}

static int parse_insn(comedi_device * dev, comedi_insn * insn, lsampl_t * data,
	void *file);
/*
 * 	COMEDI_INSNLIST
 * 	synchronous instructions
 *
 * 	arg:
 * 		pointer to sync cmd structure
 *
 * 	reads:
 * 		sync cmd struct at arg
 * 		instruction list
 * 		data (for writes)
 *
 * 	writes:
 * 		data (for reads)
 */
/* arbitrary limits */
#define MAX_SAMPLES 256
static int do_insnlist_ioctl(comedi_device * dev, void *arg, void *file)
{
	comedi_insnlist insnlist;
	comedi_insn *insns = NULL;
	lsampl_t *data = NULL;
	int i = 0;
	int ret = 0;

	if (copy_from_user(&insnlist, arg, sizeof(comedi_insnlist)))
		return -EFAULT;

	data = kmalloc(sizeof(lsampl_t) * MAX_SAMPLES, GFP_KERNEL);
	if (!data) {
		DPRINTK("kmalloc failed\n");
		ret = -ENOMEM;
		goto error;
	}

	insns = kmalloc(sizeof(comedi_insn) * insnlist.n_insns, GFP_KERNEL);
	if (!insns) {
		DPRINTK("kmalloc failed\n");
		ret = -ENOMEM;
		goto error;
	}

	if (copy_from_user(insns, insnlist.insns,
			sizeof(comedi_insn) * insnlist.n_insns)) {
		DPRINTK("copy_from_user failed\n");
		ret = -EFAULT;
		goto error;
	}

	for (i = 0; i < insnlist.n_insns; i++) {
		if (insns[i].n > MAX_SAMPLES) {
			DPRINTK("number of samples too large\n");
			ret = -EINVAL;
			goto error;
		}
		if (insns[i].insn & INSN_MASK_WRITE) {
			if (copy_from_user(data, insns[i].data,
					insns[i].n * sizeof(lsampl_t))) {
				DPRINTK("copy_from_user failed\n");
				ret = -EFAULT;
				goto error;
			}
		}
		ret = parse_insn(dev, insns + i, data, file);
		if (ret < 0)
			goto error;
		if (insns[i].insn & INSN_MASK_READ) {
			if (copy_to_user(insns[i].data, data,
					insns[i].n * sizeof(lsampl_t))) {
				DPRINTK("copy_to_user failed\n");
				ret = -EFAULT;
				goto error;
			}
		}
		if (need_resched())
			schedule();
	}

      error:
	if (insns)
		kfree(insns);
	if (data)
		kfree(data);

	if (ret < 0)
		return ret;
	return i;
}

static int check_insn_config_length(comedi_insn * insn, lsampl_t * data)
{
	switch (data[0]) {
	case INSN_CONFIG_DIO_OUTPUT:
	case INSN_CONFIG_DIO_INPUT:
	case INSN_CONFIG_DISARM:
	case INSN_CONFIG_RESET:
		if (insn->n == 1)
			return 0;
		break;
	case INSN_CONFIG_ARM:
	case INSN_CONFIG_DIO_QUERY:
	case INSN_CONFIG_BLOCK_SIZE:
	case INSN_CONFIG_FILTER:
	case INSN_CONFIG_SERIAL_CLOCK:
	case INSN_CONFIG_BIDIRECTIONAL_DATA:
	case INSN_CONFIG_ALT_SOURCE:
	case INSN_CONFIG_SET_COUNTER_MODE:
	case INSN_CONFIG_8254_READ_STATUS:
	case INSN_CONFIG_SET_ROUTING:
	case INSN_CONFIG_GET_ROUTING:
		if (insn->n == 2)
			return 0;
		break;
	case INSN_CONFIG_SET_GATE_SRC:
	case INSN_CONFIG_GET_GATE_SRC:
	case INSN_CONFIG_SET_CLOCK_SRC:
	case INSN_CONFIG_GET_CLOCK_SRC:
	case INSN_CONFIG_SET_OTHER_SRC:
	case INSN_CONFIG_GET_COUNTER_STATUS:
		if (insn->n == 3)
			return 0;
		break;
	case INSN_CONFIG_PWM_OUTPUT:
	case INSN_CONFIG_ANALOG_TRIG:
		if (insn->n == 5)
			return 0;
		break;
		//by default we allow the insn since we don't have checks for all possible cases yet
	default:
		rt_printk
			("comedi: no check for data length of config insn id %i is implemented.\n"
			" Add a check to %s in %s.\n"
			" Assuming n=%i is correct.\n", data[0], __FUNCTION__,
			__FILE__, insn->n);
		return 0;
		break;
	}
	return -EINVAL;
}

static int parse_insn(comedi_device * dev, comedi_insn * insn, lsampl_t * data,
	void *file)
{
	comedi_subdevice *s;
	int ret = 0;
	int i;

	if (insn->insn & INSN_MASK_SPECIAL) {
		/* a non-subdevice instruction */

		switch (insn->insn) {
		case INSN_GTOD:
			{
				struct timeval tv;

				if (insn->n != 2) {
					ret = -EINVAL;
					break;
				}

				do_gettimeofday(&tv);
				data[0] = tv.tv_sec;
				data[1] = tv.tv_usec;
				ret = 2;

				break;
			}
		case INSN_WAIT:
			if (insn->n != 1 || data[0] >= 100000) {
				ret = -EINVAL;
				break;
			}
			udelay(data[0] / 1000);
			ret = 1;
			break;
		case INSN_INTTRIG:
			if (insn->n != 1) {
				ret = -EINVAL;
				break;
			}
			if (insn->subdev >= dev->n_subdevices) {
				DPRINTK("%d not usable subdevice\n",
					insn->subdev);
				ret = -EINVAL;
				break;
			}
			s = dev->subdevices + insn->subdev;
			if (!s->async) {
				DPRINTK("no async\n");
				ret = -EINVAL;
				break;
			}
			if (!s->async->inttrig) {
				DPRINTK("no inttrig\n");
				ret = -EAGAIN;
				break;
			}
			ret = s->async->inttrig(dev, s, insn->data[0]);
			if (ret >= 0)
				ret = 1;
			break;
		default:
			DPRINTK("invalid insn\n");
			ret = -EINVAL;
			break;
		}
	} else {
		/* a subdevice instruction */
		if (insn->subdev >= dev->n_subdevices) {
			DPRINTK("subdevice %d out of range\n", insn->subdev);
			ret = -EINVAL;
			goto out;
		}
		s = dev->subdevices + insn->subdev;

		if (s->type == COMEDI_SUBD_UNUSED) {
			DPRINTK("%d not usable subdevice\n", insn->subdev);
			ret = -EIO;
			goto out;
		}

		/* are we locked? (ioctl lock) */
		if (s->lock && s->lock != file) {
			DPRINTK("device locked\n");
			ret = -EACCES;
			goto out;
		}

		if ((ret = check_chanlist(s, 1, &insn->chanspec)) < 0) {
			ret = -EINVAL;
			DPRINTK("bad chanspec\n");
			goto out;
		}

		if (s->busy) {
			ret = -EBUSY;
			goto out;
		}
		/* This looks arbitrary.  It is. */
		s->busy = &parse_insn;
		switch (insn->insn) {
		case INSN_READ:
			ret = s->insn_read(dev, s, insn, data);
			break;
		case INSN_WRITE:
			for (i = 0; i < insn->n; ++i) {
				if (data[i] > s->maxdata)
					ret = -EINVAL;
			}
			if (ret == 0)
				ret = s->insn_write(dev, s, insn, data);
			break;
		case INSN_BITS:
			if (insn->n != 2) {
				ret = -EINVAL;
				break;
			}
			ret = s->insn_bits(dev, s, insn, data);
			break;
		case INSN_CONFIG:
			ret = check_insn_config_length(insn, data);
			if (ret)
				break;
			ret = s->insn_config(dev, s, insn, data);
			break;
		default:
			ret = -EINVAL;
			break;
		}

		s->busy = NULL;
	}

      out:
	return ret;
}

/*
 * 	COMEDI_INSN
 * 	synchronous instructions
 *
 * 	arg:
 * 		pointer to insn
 *
 * 	reads:
 * 		comedi_insn struct at arg
 * 		data (for writes)
 *
 * 	writes:
 * 		data (for reads)
 */
static int do_insn_ioctl(comedi_device * dev, void *arg, void *file)
{
	comedi_insn insn;
	lsampl_t *data = NULL;
	int ret = 0;

	data = kmalloc(sizeof(lsampl_t) * MAX_SAMPLES, GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto error;
	}

	if (copy_from_user(&insn, arg, sizeof(comedi_insn))) {
		ret = -EFAULT;
		goto error;
	}

	/* This is where the behavior of insn and insnlist deviate. */
	if (insn.n > MAX_SAMPLES)
		insn.n = MAX_SAMPLES;
	if (insn.insn & INSN_MASK_WRITE) {
		if (copy_from_user(data, insn.data, insn.n * sizeof(lsampl_t))) {
			ret = -EFAULT;
			goto error;
		}
	}
	ret = parse_insn(dev, &insn, data, file);
	if (ret < 0)
		goto error;
	if (insn.insn & INSN_MASK_READ) {
		if (copy_to_user(insn.data, data, insn.n * sizeof(lsampl_t))) {
			ret = -EFAULT;
			goto error;
		}
	}
	ret = insn.n;

      error:
	if (data)
		kfree(data);

	return ret;
}

/*
	COMEDI_CMD
	command ioctl

	arg:
		pointer to cmd structure

	reads:
		cmd structure at arg
		channel/range list

	writes:
		modified cmd structure at arg

*/
static int do_cmd_ioctl(comedi_device * dev, void *arg, void *file)
{
	comedi_cmd user_cmd;
	comedi_subdevice *s;
	comedi_async *async;
	int ret = 0;
	unsigned int *chanlist_saver = NULL;

	if (copy_from_user(&user_cmd, arg, sizeof(comedi_cmd))) {
		DPRINTK("bad cmd address\n");
		return -EFAULT;
	}
	// save user's chanlist pointer so it can be restored later
	chanlist_saver = user_cmd.chanlist;

	if (user_cmd.subdev >= dev->n_subdevices) {
		DPRINTK("%d no such subdevice\n", user_cmd.subdev);
		return -ENODEV;
	}

	s = dev->subdevices + user_cmd.subdev;
	async = s->async;

	if (s->type == COMEDI_SUBD_UNUSED) {
		DPRINTK("%d not valid subdevice\n", user_cmd.subdev);
		return -EIO;
	}

	if (!s->do_cmd || !s->do_cmdtest || !s->async) {
		DPRINTK("subdevice %i does not support commands\n",
			user_cmd.subdev);
		return -EIO;
	}

	/* are we locked? (ioctl lock) */
	if (s->lock && s->lock != file) {
		DPRINTK("subdevice locked\n");
		return -EACCES;
	}

	/* are we busy? */
	if (s->busy) {
		DPRINTK("subdevice busy\n");
		return -EBUSY;
	}
	s->busy = file;

	/* make sure channel/gain list isn't too long */
	if (user_cmd.chanlist_len > s->len_chanlist) {
		DPRINTK("channel/gain list too long %d > %d\n",
			user_cmd.chanlist_len, s->len_chanlist);
		ret = -EINVAL;
		goto cleanup;
	}

	if (async->cmd.chanlist)
		kfree(async->cmd.chanlist);
	async->cmd = user_cmd;
	async->cmd.data = NULL;
	/* load channel/gain list */
	async->cmd.chanlist =
		kmalloc(async->cmd.chanlist_len * sizeof(int), GFP_KERNEL);
	if (!async->cmd.chanlist) {
		DPRINTK("allocation failed\n");
		ret = -ENOMEM;
		goto cleanup;
	}

	if (copy_from_user(async->cmd.chanlist, user_cmd.chanlist,
			async->cmd.chanlist_len * sizeof(int))) {
		DPRINTK("fault reading chanlist\n");
		ret = -EFAULT;
		goto cleanup;
	}

	/* make sure each element in channel/gain list is valid */
	if ((ret = check_chanlist(s, async->cmd.chanlist_len,
				async->cmd.chanlist)) < 0) {
		DPRINTK("bad chanlist\n");
		goto cleanup;
	}

	ret = s->do_cmdtest(dev, s, &async->cmd);

	if (async->cmd.flags & TRIG_BOGUS || ret) {
		DPRINTK("test returned %d\n", ret);
		user_cmd = async->cmd;
		// restore chanlist pointer before copying back
		user_cmd.chanlist = chanlist_saver;
		user_cmd.data = NULL;
		if (copy_to_user(arg, &user_cmd, sizeof(comedi_cmd))) {
			DPRINTK("fault writing cmd\n");
			ret = -EFAULT;
			goto cleanup;
		}
		ret = -EAGAIN;
		goto cleanup;
	}

	if (!async->prealloc_bufsz) {
		ret = -ENOMEM;
		DPRINTK("no buffer (?)\n");
		goto cleanup;
	}

	comedi_reset_async_buf(async);

	async->cb_mask =
		COMEDI_CB_EOA | COMEDI_CB_BLOCK | COMEDI_CB_ERROR |
		COMEDI_CB_OVERFLOW;
	if (async->cmd.flags & TRIG_WAKE_EOS) {
		async->cb_mask |= COMEDI_CB_EOS;
	}

	comedi_set_subdevice_runflags(s, ~0, SRF_USER | SRF_RUNNING);

#ifdef CONFIG_COMEDI_RT
	if (async->cmd.flags & TRIG_RT) {
		comedi_switch_to_rt(dev);
		comedi_set_subdevice_runflags(s, SRF_RT, SRF_RT);
	}
#endif

	ret = s->do_cmd(dev, s);
	if (ret == 0)
		return 0;

      cleanup:
	do_become_nonbusy(dev, s);

	return ret;
}

/*
	COMEDI_CMDTEST
	command testing ioctl

	arg:
		pointer to cmd structure

	reads:
		cmd structure at arg
		channel/range list

	writes:
		modified cmd structure at arg

*/
static int do_cmdtest_ioctl(comedi_device * dev, void *arg, void *file)
{
	comedi_cmd user_cmd;
	comedi_subdevice *s;
	int ret = 0;
	unsigned int *chanlist = NULL;
	unsigned int *chanlist_saver = NULL;

	if (copy_from_user(&user_cmd, arg, sizeof(comedi_cmd))) {
		DPRINTK("bad cmd address\n");
		return -EFAULT;
	}
	// save user's chanlist pointer so it can be restored later
	chanlist_saver = user_cmd.chanlist;

	if (user_cmd.subdev >= dev->n_subdevices) {
		DPRINTK("%d no such subdevice\n", user_cmd.subdev);
		return -ENODEV;
	}

	s = dev->subdevices + user_cmd.subdev;
	if (s->type == COMEDI_SUBD_UNUSED) {
		DPRINTK("%d not valid subdevice\n", user_cmd.subdev);
		return -EIO;
	}

	if (!s->do_cmd || !s->do_cmdtest) {
		DPRINTK("subdevice %i does not support commands\n",
			user_cmd.subdev);
		return -EIO;
	}

	/* make sure channel/gain list isn't too long */
	if (user_cmd.chanlist_len > s->len_chanlist) {
		DPRINTK("channel/gain list too long %d > %d\n",
			user_cmd.chanlist_len, s->len_chanlist);
		ret = -EINVAL;
		goto cleanup;
	}

	/* load channel/gain list */
	if (user_cmd.chanlist) {
		chanlist =
			kmalloc(user_cmd.chanlist_len * sizeof(int),
			GFP_KERNEL);
		if (!chanlist) {
			DPRINTK("allocation failed\n");
			ret = -ENOMEM;
			goto cleanup;
		}

		if (copy_from_user(chanlist, user_cmd.chanlist,
				user_cmd.chanlist_len * sizeof(int))) {
			DPRINTK("fault reading chanlist\n");
			ret = -EFAULT;
			goto cleanup;
		}

		/* make sure each element in channel/gain list is valid */
		if ((ret = check_chanlist(s, user_cmd.chanlist_len,
					chanlist)) < 0) {
			DPRINTK("bad chanlist\n");
			goto cleanup;
		}

		user_cmd.chanlist = chanlist;
	}

	ret = s->do_cmdtest(dev, s, &user_cmd);

	// restore chanlist pointer before copying back
	user_cmd.chanlist = chanlist_saver;

	if (copy_to_user(arg, &user_cmd, sizeof(comedi_cmd))) {
		DPRINTK("bad cmd address\n");
		ret = -EFAULT;
		goto cleanup;
	}
      cleanup:
	if (chanlist)
		kfree(chanlist);

	return ret;
}

/*
	COMEDI_LOCK
	lock subdevice

	arg:
		subdevice number

	reads:
		none

	writes:
		none

*/

static int do_lock_ioctl(comedi_device * dev, unsigned int arg, void *file)
{
	int ret = 0;
	unsigned long flags;
	comedi_subdevice *s;

	if (arg >= dev->n_subdevices)
		return -EINVAL;
	s = dev->subdevices + arg;

	comedi_spin_lock_irqsave(&big_comedi_lock, flags);
	if (s->busy) {
		comedi_spin_unlock_irqrestore(&big_comedi_lock, flags);
		return -EBUSY;
	}
	if (s->lock) {
		ret = -EBUSY;
	} else {
		s->lock = file;
	}
	comedi_spin_unlock_irqrestore(&big_comedi_lock, flags);

	if (ret < 0)
		return ret;

#if 0
	if (s->lock_f)
		ret = s->lock_f(dev, s);
#endif

	return ret;
}

/*
	COMEDI_UNLOCK
	unlock subdevice

	arg:
		subdevice number

	reads:
		none

	writes:
		none

	This function isn't protected by the semaphore, since
	we already own the lock.
*/
static int do_unlock_ioctl(comedi_device * dev, unsigned int arg, void *file)
{
	comedi_subdevice *s;

	if (arg >= dev->n_subdevices)
		return -EINVAL;
	s = dev->subdevices + arg;

	if (s->busy)
		return -EBUSY;

	if (s->lock && s->lock != file)
		return -EACCES;

	if (s->lock == file) {
#if 0
		if (s->unlock)
			s->unlock(dev, s);
#endif

		s->lock = NULL;
	}

	return 0;
}

/*
	COMEDI_CANCEL
	cancel acquisition ioctl

	arg:
		subdevice number

	reads:
		nothing

	writes:
		nothing

*/
static int do_cancel_ioctl(comedi_device * dev, unsigned int arg, void *file)
{
	comedi_subdevice *s;

	if (arg >= dev->n_subdevices)
		return -EINVAL;
	s = dev->subdevices + arg;
	if (s->async == NULL)
		return -EINVAL;

	if (s->lock && s->lock != file)
		return -EACCES;

	if (!s->busy)
		return 0;

	if (s->busy != file)
		return -EBUSY;

	return do_cancel(dev, s);
}

/*
	COMEDI_POLL ioctl
	instructs driver to synchronize buffers

	arg:
		subdevice number

	reads:
		nothing

	writes:
		nothing

*/
static int do_poll_ioctl(comedi_device * dev, unsigned int arg, void *file)
{
	comedi_subdevice *s;

	if (arg >= dev->n_subdevices)
		return -EINVAL;
	s = dev->subdevices + arg;

	if (s->lock && s->lock != file)
		return -EACCES;

	if (!s->busy)
		return 0;

	if (s->busy != file)
		return -EBUSY;

	if (s->poll)
		return s->poll(dev, s);

	return -EINVAL;
}

static int do_cancel(comedi_device * dev, comedi_subdevice * s)
{
	int ret = 0;

	if ((comedi_get_subdevice_runflags(s) & SRF_RUNNING) && s->cancel)
		ret = s->cancel(dev, s);

	do_become_nonbusy(dev, s);

	return ret;
}

void comedi_unmap(struct vm_area_struct *area)
{
	comedi_async *async;

	async = area->vm_private_data;

	async->mmap_count--;
}

static struct vm_operations_struct comedi_vm_ops = {
      close:comedi_unmap,
};

static int comedi_mmap(struct file *file, struct vm_area_struct *vma)
{
	const unsigned minor = iminor(file->f_dentry->d_inode);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	comedi_async *async = NULL;
	unsigned long start = vma->vm_start;
	unsigned long size;
	int n_pages;
	int i;
	comedi_subdevice *s;

	if (!dev->attached) {
		DPRINTK("no driver configured on comedi%i\n", dev->minor);
		return -ENODEV;
	}
	if (vma->vm_flags & VM_WRITE) {
		s = comedi_get_write_subdevice(dev, minor);
	} else {
		s = comedi_get_read_subdevice(dev, minor);
	}
	if (s == NULL)
		return -EINVAL;
	async = s->async;
	if (async == NULL) {
		return -EINVAL;
	}

	if (vma->vm_pgoff != 0) {
		DPRINTK("comedi: mmap() offset must be 0.\n");
		return -EINVAL;
	}

	size = vma->vm_end - vma->vm_start;
	if (size > async->prealloc_bufsz)
		return -EFAULT;
	if (size & (~PAGE_MASK))
		return -EFAULT;

	n_pages = size >> PAGE_SHIFT;
	for (i = 0; i < n_pages; ++i) {
		if (remap_pfn_range(vma, start,
				page_to_pfn(virt_to_page(async->
						buf_page_list[i].virt_addr)),
				PAGE_SIZE, PAGE_SHARED)) {
			return -EAGAIN;
		}
		start += PAGE_SIZE;
	}

	vma->vm_ops = &comedi_vm_ops;
	vma->vm_private_data = async;

	async->mmap_count++;

	return 0;
}

static unsigned int comedi_poll(struct file *file, poll_table * wait)
{
	unsigned int mask;
	const unsigned minor = iminor(file->f_dentry->d_inode);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	comedi_subdevice *read_subdev = comedi_get_read_subdevice(dev, minor);
	comedi_subdevice *write_subdev = comedi_get_write_subdevice(dev, minor);

	if (!dev->attached) {
		DPRINTK("no driver configured on comedi%i\n", dev->minor);
		return -ENODEV;
	}

	mask = 0;
	if (read_subdev) {
		poll_wait(file, &read_subdev->async->wait_head, wait);
		if (!read_subdev->busy
			|| comedi_buf_read_n_available(read_subdev->async) > 0
			|| !(comedi_get_subdevice_runflags(read_subdev) &
				SRF_RUNNING)) {
			mask |= POLLIN | POLLRDNORM;
		}
	}
	if (write_subdev) {
		poll_wait(file, &write_subdev->async->wait_head, wait);
		if (!write_subdev->busy
			|| !(comedi_get_subdevice_runflags(write_subdev) &
				SRF_RUNNING)
			|| comedi_buf_write_n_available(write_subdev->async) >
			0) {
			mask |= POLLOUT | POLLWRNORM;
		}
	}

	return mask;
}

static ssize_t comedi_write(struct file *file, const char *buf, size_t nbytes,
	loff_t * offset)
{
	comedi_device *dev;
	comedi_subdevice *s;
	comedi_async *async;
	int n, m, count = 0, retval = 0;
	DECLARE_WAITQUEUE(wait, current);
	const unsigned minor = iminor(file->f_dentry->d_inode);

	dev = comedi_get_device_by_minor(minor);
	if (!dev->attached) {
		DPRINTK("no driver configured on comedi%i\n", dev->minor);
		return -ENODEV;
	}

	s = comedi_get_write_subdevice(dev, minor);
	if (s == NULL)
		return -EIO;
	async = s->async;

	if (!nbytes)
		return 0;

	if (!s->busy) {
		return 0;
	}

	if (s->busy != file)
		return -EACCES;
	add_wait_queue(&async->wait_head, &wait);
	while (nbytes > 0 && !retval) {
		set_current_state(TASK_INTERRUPTIBLE);

		n = nbytes;

		m = n;
		if (async->buf_write_ptr + m > async->prealloc_bufsz) {
			m = async->prealloc_bufsz - async->buf_write_ptr;
		}
		comedi_buf_write_alloc(async, async->prealloc_bufsz);
		if (m > comedi_buf_write_n_allocated(async)) {
			m = comedi_buf_write_n_allocated(async);
		}
		if (m < n)
			n = m;

		if (n == 0) {
			if (file->f_flags & O_NONBLOCK) {
				retval = -EAGAIN;
				break;
			}
			if (!(comedi_get_subdevice_runflags(s) & SRF_RUNNING)) {
				if (comedi_get_subdevice_runflags(s) &
					SRF_ERROR) {
					retval = -EPIPE;
				} else {
					retval = 0;
				}
				do_become_nonbusy(dev, s);
				break;
			}
			if (signal_pending(current)) {
				retval = -ERESTARTSYS;
				break;
			}
			schedule();
			continue;
		}

		m = copy_from_user(async->prealloc_buf + async->buf_write_ptr,
			buf, n);
		if (m) {
			n -= m;
			retval = -EFAULT;
		}
		comedi_buf_write_free(async, n);

		count += n;
		nbytes -= n;

		buf += n;
		break;		/* makes device work like a pipe */
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&async->wait_head, &wait);

	return (count ? count : retval);
}

static ssize_t comedi_read(struct file *file, char *buf, size_t nbytes,
	loff_t * offset)
{
	comedi_subdevice *s;
	comedi_async *async;
	int n, m, count = 0, retval = 0;
	DECLARE_WAITQUEUE(wait, current);
	const unsigned minor = iminor(file->f_dentry->d_inode);
	comedi_device *dev = comedi_get_device_by_minor(minor);

	if (!dev->attached) {
		DPRINTK("no driver configured on comedi%i\n", dev->minor);
		return -ENODEV;
	}

	s = comedi_get_read_subdevice(dev, minor);
	if (s == NULL)
		return -EIO;
	async = s->async;
	if (!nbytes)
		return 0;

	if (!s->busy)
		return 0;

	if (s->busy != file)
		return -EACCES;

	add_wait_queue(&async->wait_head, &wait);
	while (nbytes > 0 && !retval) {
		set_current_state(TASK_INTERRUPTIBLE);

		n = nbytes;

		m = comedi_buf_read_n_available(async);
//printk("%d available\n",m);
		if (async->buf_read_ptr + m > async->prealloc_bufsz) {
			m = async->prealloc_bufsz - async->buf_read_ptr;
		}
//printk("%d contiguous\n",m);
		if (m < n)
			n = m;

		if (n == 0) {
			if (file->f_flags & O_NONBLOCK) {
				retval = -EAGAIN;
				break;
			}
			if (!(comedi_get_subdevice_runflags(s) & SRF_RUNNING)) {
				do_become_nonbusy(dev, s);
				if (comedi_get_subdevice_runflags(s) &
					SRF_ERROR) {
					retval = -EPIPE;
				} else {
					retval = 0;
				}
				break;
			}
			if (signal_pending(current)) {
				retval = -ERESTARTSYS;
				break;
			}
			schedule();
			continue;
		}
		m = copy_to_user(buf, async->prealloc_buf +
			async->buf_read_ptr, n);
		if (m) {
			n -= m;
			retval = -EFAULT;
		}

		comedi_buf_read_alloc(async, n);
		comedi_buf_read_free(async, n);

		count += n;
		nbytes -= n;

		buf += n;
		break;		/* makes device work like a pipe */
	}
	if (!(comedi_get_subdevice_runflags(s) & (SRF_ERROR | SRF_RUNNING)) &&
		async->buf_read_count - async->buf_write_count == 0) {
		do_become_nonbusy(dev, s);
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&async->wait_head, &wait);

	return (count ? count : retval);
}

/*
   This function restores a subdevice to an idle state.
 */
void do_become_nonbusy(comedi_device * dev, comedi_subdevice * s)
{
	comedi_async *async = s->async;

	comedi_set_subdevice_runflags(s, SRF_RUNNING, 0);
#ifdef CONFIG_COMEDI_RT
	if (comedi_get_subdevice_runflags(s) & SRF_RT) {
		comedi_switch_to_non_rt(dev);
		comedi_set_subdevice_runflags(s, SRF_RT, 0);
	}
#endif
	if (async) {
		comedi_reset_async_buf(async);
		async->inttrig = NULL;
	} else {
		printk("BUG: (?) do_become_nonbusy called with async=0\n");
	}

	s->busy = NULL;
}

static int comedi_open(struct inode *inode, struct file *file)
{
	char mod[32];
	const unsigned minor = iminor(inode);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	if (dev == NULL) {
		DPRINTK("invalid minor number\n");
		return -ENODEV;
	}

	/* This is slightly hacky, but we want module autoloading
	 * to work for root.
	 * case: user opens device, attached -> ok
	 * case: user opens device, unattached, in_request_module=0 -> autoload
	 * case: user opens device, unattached, in_request_module=1 -> fail
	 * case: root opens device, attached -> ok
	 * case: root opens device, unattached, in_request_module=1 -> ok
	 *   (typically called from modprobe)
	 * case: root opens device, unattached, in_request_module=0 -> autoload
	 *
	 * The last could be changed to "-> ok", which would deny root
	 * autoloading.
	 */
	if (dev->attached)
		goto ok;
	if (!capable(CAP_SYS_MODULE) && dev->in_request_module) {
		DPRINTK("in request module\n");
		return -ENODEV;
	}
	if (capable(CAP_SYS_MODULE) && dev->in_request_module)
		goto ok;

	dev->in_request_module = 1;

	sprintf(mod, "char-major-%i-%i", COMEDI_MAJOR, dev->minor);
#ifdef CONFIG_KMOD
	request_module(mod);
#endif

	dev->in_request_module = 0;

	if (!dev->attached && !capable(CAP_SYS_MODULE)) {
		DPRINTK("not attached and not CAP_SYS_MODULE\n");
		return -ENODEV;
	}
      ok:
	if (!try_module_get(THIS_MODULE))
		return -ENOSYS;

	if (dev->attached) {
		try_module_get(dev->driver->module);
	}

	if (dev->attached && dev->use_count == 0 && dev->open) {
		dev->open(dev);
	}

	dev->use_count++;

	return 0;
}

static int comedi_close(struct inode *inode, struct file *file)
{
	const unsigned minor = iminor(inode);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	comedi_subdevice *s = NULL;
	int i;

	if (dev->subdevices) {
		for (i = 0; i < dev->n_subdevices; i++) {
			s = dev->subdevices + i;

			if (s->busy == file) {
				do_cancel(dev, s);
			}
			if (s->lock == file) {
				s->lock = NULL;
			}
		}
	}
	if (dev->attached && dev->use_count == 1 && dev->close) {
		dev->close(dev);
	}

	module_put(THIS_MODULE);
	if (dev->attached) {
		module_put(dev->driver->module);
	}

	dev->use_count--;

	if (file->f_flags & FASYNC) {
		comedi_fasync(-1, file, 0);
	}

	return 0;
}

static int comedi_fasync(int fd, struct file *file, int on)
{
	const unsigned minor = iminor(file->f_dentry->d_inode);
	comedi_device *dev = comedi_get_device_by_minor(minor);

	return fasync_helper(fd, file, on, &dev->async_queue);
}

const struct file_operations comedi_fops = {
      owner:THIS_MODULE,
      ioctl:comedi_ioctl,
#ifdef HAVE_COMPAT_IOCTL
      compat_ioctl:comedi_compat_ioctl,
#endif
      open:comedi_open,
      release:comedi_close,
      read:comedi_read,
      write:comedi_write,
      mmap:comedi_mmap,
      poll:comedi_poll,
      fasync:comedi_fasync,
};

struct class *comedi_class = NULL;
static struct cdev comedi_cdev;

static int __init comedi_init(void)
{
	int i;
	int retval;

	printk("comedi: version " COMEDI_RELEASE
		" - David Schleef <ds@schleef.org>\n");
	spin_lock_init(&big_comedi_lock);
	retval = register_chrdev_region(MKDEV(COMEDI_MAJOR, 0),
		COMEDI_NUM_MINORS, "comedi");
	if (retval)
		return -EIO;
	cdev_init(&comedi_cdev, &comedi_fops);
	comedi_cdev.owner = THIS_MODULE;
	kobject_set_name(&comedi_cdev.kobj, "comedi");
	if (cdev_add(&comedi_cdev, MKDEV(COMEDI_MAJOR, 0), COMEDI_NUM_MINORS)) {
		unregister_chrdev_region(MKDEV(COMEDI_MAJOR, 0),
			COMEDI_NUM_MINORS);
		return -EIO;
	}
	comedi_class = class_create(THIS_MODULE, "comedi");
	if (IS_ERR(comedi_class)) {
		printk("comedi: failed to create class");
		unregister_chrdev_region(MKDEV(COMEDI_MAJOR, 0),
			COMEDI_NUM_MINORS);
		cdev_del(&comedi_cdev);
		return PTR_ERR(comedi_class);
	}
	comedi_devices =
		kcalloc(COMEDI_NDEVICES, sizeof(comedi_device), GFP_KERNEL);
	if (!comedi_devices) {
		unregister_chrdev_region(MKDEV(COMEDI_MAJOR, 0),
			COMEDI_NUM_MINORS);
		cdev_del(&comedi_cdev);
		class_destroy(comedi_class);
		return -ENOMEM;
	}

	/* XXX requires /proc interface */
	comedi_proc_init();

	for (i = 0; i < COMEDI_NDEVICES; i++) {
		char name[20];
		sprintf(name, "comedi%d", i);
		comedi_devices[i].minor = i;
		comedi_devices[i].class_dev =
			COMEDI_CLASS_DEVICE_CREATE(comedi_class, 0,
			MKDEV(COMEDI_MAJOR, i), NULL, "comedi%i", i);
		spin_lock_init(&comedi_devices[i].spinlock);
	}

	comedi_rt_init();

	comedi_register_ioctl32();

	return 0;
}

static void __exit comedi_cleanup(void)
{
	int i;

	if (MOD_IN_USE)
		printk("comedi: module in use -- remove delayed\n");

	for (i = 0; i < COMEDI_NDEVICES; i++) {
		comedi_device *dev;

		dev = comedi_devices + i;
		if (dev->attached)
			comedi_device_detach(dev);
	}

	for (i = 0; i < COMEDI_NDEVICES; i++) {
		char name[20];
		class_device_destroy(comedi_class, MKDEV(COMEDI_MAJOR, i));
		sprintf(name, "comedi%d", i);
	}
	class_destroy(comedi_class);
	cdev_del(&comedi_cdev);
	unregister_chrdev_region(MKDEV(COMEDI_MAJOR, 0), COMEDI_NUM_MINORS);

	comedi_proc_cleanup();

	kfree(comedi_devices);

	comedi_rt_cleanup();

	comedi_unregister_ioctl32();
}

module_init(comedi_init);
module_exit(comedi_cleanup);

void comedi_error(const comedi_device * dev, const char *s)
{
	rt_printk("comedi%d: %s: %s\n", dev->minor, dev->driver->driver_name,
		s);
}

void comedi_event(comedi_device * dev, comedi_subdevice * s)
{
	comedi_async *async = s->async;
	unsigned runflags = 0;
	unsigned runflags_mask = 0;

	//DPRINTK("comedi_event 0x%x\n",mask);

	if ((comedi_get_subdevice_runflags(s) & SRF_RUNNING) == 0)
		return;

	if (s->async->
		events & (COMEDI_CB_EOA | COMEDI_CB_ERROR | COMEDI_CB_OVERFLOW))
	{
		runflags_mask |= SRF_RUNNING;
	}
	/* remember if an error event has occured, so an error
	 * can be returned the next time the user does a read() */
	if (s->async->events & (COMEDI_CB_ERROR | COMEDI_CB_OVERFLOW)) {
		runflags_mask |= SRF_ERROR;
		runflags |= SRF_ERROR;
	}
	if (runflags_mask) {
		/*sets SRF_ERROR and SRF_RUNNING together atomically */
		comedi_set_subdevice_runflags(s, runflags_mask, runflags);
	}

	if (async->cb_mask & s->async->events) {
		if (comedi_get_subdevice_runflags(s) & SRF_USER) {

			if (dev->rt) {
#ifdef CONFIG_COMEDI_RT
				// pend wake up
				comedi_rt_pend_wakeup(&async->wait_head);
#else
				printk("BUG: comedi_event() code unreachable\n");
#endif
			} else {
				wake_up_interruptible(&async->wait_head);
				if (s->subdev_flags & SDF_CMD_READ) {
					kill_fasync(&dev->async_queue, SIGIO,
						POLL_IN);
				}
				if (s->subdev_flags & SDF_CMD_WRITE) {
					kill_fasync(&dev->async_queue, SIGIO,
						POLL_OUT);
				}
			}
		} else {
			if (async->cb_func)
				async->cb_func(s->async->events, async->cb_arg);
			/* XXX bug here.  If subdevice A is rt, and
			 * subdevice B tries to callback to a normal
			 * linux kernel function, it will be at the
			 * wrong priority.  Since this isn't very
			 * common, I'm not going to worry about it. */
		}
	}
	s->async->events = 0;
}

void comedi_set_subdevice_runflags(comedi_subdevice * s, unsigned mask,
	unsigned bits)
{
	unsigned long flags;

	comedi_spin_lock_irqsave(&s->runflags_lock, flags);
	s->runflags &= ~mask;
	s->runflags |= (bits & mask);
	comedi_spin_unlock_irqrestore(&s->runflags_lock, flags);
}

unsigned comedi_get_subdevice_runflags(comedi_subdevice * s)
{
	unsigned long flags;
	unsigned runflags;

	comedi_spin_lock_irqsave(&s->runflags_lock, flags);
	runflags = s->runflags;
	comedi_spin_unlock_irqrestore(&s->runflags_lock, flags);
	return runflags;
}
