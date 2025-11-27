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
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched/signal.h>
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
#include <linux/stat.h>
#include <linux/uaccess.h>
#include <asm/io.h>

#include "comedi_internal.h"
#include "comedi_compat32.h"

//#include "kvmem.h"

MODULE_AUTHOR("http://www.comedi.org");
MODULE_DESCRIPTION("Comedi core module");
MODULE_LICENSE("GPL");
MODULE_VERSION(COMEDI_RELEASE);

struct comedi_file {
	comedi_device *dev;
	comedi_subdevice * volatile read_subdev;
	comedi_subdevice * volatile write_subdev;
	unsigned int last_detach_count;
	int last_attached;
};

#define COMEDI_NUM_SUBDEVICE_MINORS	\
	(COMEDI_NUM_MINORS - COMEDI_NUM_BOARD_MINORS)

#ifdef COMEDI_CONFIG_DEBUG
int comedi_debug;
module_param(comedi_debug, int, 0644);
#endif

COMEDI_MODULE_PARAM_BOOL_T comedi_autoconfig = 1;
module_param(comedi_autoconfig, bool, 0444);

static int comedi_num_legacy_minors = 0;
module_param(comedi_num_legacy_minors, int, 0444);

static DEFINE_SPINLOCK(comedi_board_minor_table_lock);
static comedi_device *comedi_board_minor_table[COMEDI_NUM_BOARD_MINORS];

static DEFINE_SPINLOCK(comedi_subdevice_minor_table_lock);
static comedi_subdevice *comedi_subdevice_minor_table[COMEDI_NUM_SUBDEVICE_MINORS];

static void comedi_free_board_dev(comedi_device *dev);
static int do_devconfig_ioctl(comedi_device * dev, comedi_devconfig __user * arg);
static int do_bufconfig_ioctl(comedi_device * dev, comedi_bufconfig __user *arg);
static int do_devinfo_ioctl(comedi_device * dev, comedi_devinfo __user * arg,
	struct file *file);
static int do_subdinfo_ioctl(comedi_device * dev, comedi_subdinfo __user * arg,
	void *file);
static int do_chaninfo_ioctl(comedi_device * dev, comedi_chaninfo __user * arg);
static int do_chaninfo_i(comedi_device * dev, comedi_chaninfo * ci);
static int do_rangeinfo_ioctl(comedi_device * dev, comedi_rangeinfo __user * arg);
static int do_bufinfo_ioctl(comedi_device * dev, comedi_bufinfo __user *arg, void *file);
static int do_cmd_ioctl(comedi_device * dev, comedi_cmd __user *arg, void *file);
static int do_cmd_i(comedi_device * dev, comedi_cmd *cmd, int *copy, void *file);
static int do_lock_ioctl(comedi_device * dev, unsigned int arg, void *file);
static int do_unlock_ioctl(comedi_device * dev, unsigned int arg, void *file);
static int do_cancel_ioctl(comedi_device * dev, unsigned int arg, void *file);
static int do_cmdtest_ioctl(comedi_device * dev, comedi_cmd __user *arg, void *file);
static int do_cmdtest_i(comedi_device * dev, comedi_cmd *cmd, int *copy, void *file);
static int do_insnlist_ioctl(comedi_device * dev, comedi_insnlist __user *arg, void *file);
static int do_insnlist_i(comedi_device * dev, comedi_insn *insns, unsigned int n_insns, void *file);
static int do_insn_ioctl(comedi_device * dev, comedi_insn __user *arg, void *file);
static int do_insn_i(comedi_device * dev, comedi_insn *insn, void *file);
static int do_poll_ioctl(comedi_device * dev, unsigned int subd, void *file);
static int do_setrsubd_ioctl(comedi_device *dev, unsigned long subd, struct file *file);
static int do_setwsubd_ioctl(comedi_device *dev, unsigned long subd, struct file *file);

static void do_become_nonbusy(comedi_device * dev, comedi_subdevice * s);
static int do_cancel(comedi_device * dev, comedi_subdevice * s);

static int comedi_fasync(int fd, struct file *file, int on);

static int is_device_busy(comedi_device * dev);
static int resize_async_buffer(comedi_device *dev,
	comedi_subdevice *s, comedi_async *async, unsigned new_size);

/* sysfs attribute files */
static COMEDI_DECLARE_ATTR_SHOW(show_driver_name, dev, buf);
static comedi_device_attribute_t dev_attr_driver_name =
{
	.attr = {
			.name = "driver_name",
			.mode = S_IRUGO
		},
	.show = &show_driver_name,
};

static COMEDI_DECLARE_ATTR_SHOW(show_board_name, dev, buf);
static comedi_device_attribute_t dev_attr_board_name =
{
	.attr = {
			.name = "board_name",
			.mode = S_IRUGO
		},
	.show = &show_board_name,
};

static COMEDI_DECLARE_ATTR_SHOW(show_bydrivername_index, dev, buf);
static comedi_device_attribute_t dev_attr_bydrivername_index =
{
	.attr = {
			.name = "bydrivername_index",
			.mode = S_IRUGO
		},
	.show = &show_bydrivername_index,
};

static COMEDI_DECLARE_ATTR_SHOW(show_byboardname_index, dev, buf);
static comedi_device_attribute_t dev_attr_byboardname_index =
{
	.attr = {
			.name = "byboardname_index",
			.mode = S_IRUGO
		},
	.show = &show_byboardname_index,
};

static COMEDI_DECLARE_ATTR_SHOW(show_max_read_buffer_kb, dev, buf);
static COMEDI_DECLARE_ATTR_STORE(store_max_read_buffer_kb, dev, buf, count);
static comedi_device_attribute_t dev_attr_max_read_buffer_kb =
{
	.attr = {
			.name = "max_read_buffer_kb",
			.mode = S_IRUGO | S_IWUSR
		},
	.show = &show_max_read_buffer_kb,
	.store = &store_max_read_buffer_kb
};

static COMEDI_DECLARE_ATTR_SHOW(show_read_buffer_kb, dev, buf);
static COMEDI_DECLARE_ATTR_STORE(store_read_buffer_kb, dev, buf, count);
static comedi_device_attribute_t dev_attr_read_buffer_kb =
{
	.attr = {
			.name = "read_buffer_kb",
			.mode = S_IRUGO | S_IWUSR | S_IWGRP
		},
	.show = &show_read_buffer_kb,
	.store = &store_read_buffer_kb
};

static COMEDI_DECLARE_ATTR_SHOW(show_max_write_buffer_kb, dev, buf);
static COMEDI_DECLARE_ATTR_STORE(store_max_write_buffer_kb, dev, buf, count);
static comedi_device_attribute_t dev_attr_max_write_buffer_kb =
{
	.attr = {
			.name = "max_write_buffer_kb",
			.mode = S_IRUGO | S_IWUSR
		},
	.show = &show_max_write_buffer_kb,
	.store = &store_max_write_buffer_kb
};

static COMEDI_DECLARE_ATTR_SHOW(show_write_buffer_kb, dev, buf);
static COMEDI_DECLARE_ATTR_STORE(store_write_buffer_kb, dev, buf, count);
static comedi_device_attribute_t dev_attr_write_buffer_kb =
{
	.attr = {
			.name = "write_buffer_kb",
			.mode = S_IRUGO | S_IWUSR | S_IWGRP
		},
	.show = &show_write_buffer_kb,
	.store = &store_write_buffer_kb
};

static comedi_device *comedi_clear_board_minor(unsigned minor)
{
	comedi_device *dev;
	unsigned long flags;

	comedi_spin_lock_irqsave(&comedi_board_minor_table_lock, flags);
	dev = comedi_board_minor_table[minor];
	comedi_board_minor_table[minor] = NULL;
	comedi_spin_unlock_irqrestore(&comedi_board_minor_table_lock, flags);
	return dev;
}

static comedi_subdevice*
comedi_subdevice_from_minor(const comedi_device *dev, unsigned minor)
{
	comedi_subdevice *s;
	unsigned long flags;
	unsigned i = minor - COMEDI_NUM_BOARD_MINORS;

	comedi_spin_lock_irqsave(&comedi_subdevice_minor_table_lock, flags);
	s = comedi_subdevice_minor_table[i];
	if (s && s->device != dev)
		s = NULL;
	comedi_spin_unlock_irqrestore(&comedi_subdevice_minor_table_lock, flags);
	return s;
}

static comedi_device* comedi_get_device_by_board_minor(unsigned minor)
{
	comedi_device *dev;
	unsigned long flags;

	comedi_spin_lock_irqsave(&comedi_board_minor_table_lock, flags);
	dev = comedi_board_minor_table[minor];
	comedi_spin_unlock_irqrestore(&comedi_board_minor_table_lock, flags);
	return dev;
}

static comedi_device* comedi_get_device_by_subdevice_minor(unsigned minor)
{
	comedi_device *dev;
	comedi_subdevice *s;
	unsigned long flags;
	unsigned i = minor - COMEDI_NUM_BOARD_MINORS;

	comedi_spin_lock_irqsave(&comedi_subdevice_minor_table_lock, flags);
	s = comedi_subdevice_minor_table[i];
	dev = s ? s->device : NULL;
	comedi_spin_unlock_irqrestore(&comedi_subdevice_minor_table_lock, flags);
	return dev;
}

static comedi_subdevice*
comedi_read_subdevice(const comedi_device *dev, unsigned minor)
{
	comedi_subdevice *s;

	if (minor >= COMEDI_NUM_BOARD_MINORS) {
		s = comedi_subdevice_from_minor(dev, minor);
		if (!s || (s->subdev_flags & SDF_CMD_READ))
			return s;
	}
	return dev->read_subdev;
}

static comedi_subdevice*
comedi_write_subdevice(const comedi_device *dev, unsigned minor)
{
	comedi_subdevice *s;

	if (minor >= COMEDI_NUM_BOARD_MINORS) {
		s = comedi_subdevice_from_minor(dev, minor);
		if (!s || (s->subdev_flags & SDF_CMD_WRITE))
			return s;
	}
	return dev->write_subdev;
}

static void comedi_file_reset(struct file *file)
{
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;
	comedi_subdevice *s;
	comedi_subdevice *read_s;
	comedi_subdevice *write_s;
	unsigned minor = iminor(file_inode(file));

	read_s = dev->read_subdev;
	write_s = dev->write_subdev;
	if (minor >= COMEDI_NUM_BOARD_MINORS) {
		s = comedi_subdevice_from_minor(dev, minor);
		if (!s || (s->subdev_flags & SDF_CMD_READ))
			read_s = s;
		if (!s || (s->subdev_flags & SDF_CMD_WRITE))
			write_s = s;
	}
	cfp->last_attached = dev->attached;
	cfp->last_detach_count = dev->detach_count;
	cfp->read_subdev = read_s;
	cfp->write_subdev = write_s;
}

static void comedi_file_check(struct file *file)
{
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;

	if (cfp->last_attached != dev->attached || cfp->last_detach_count != dev->detach_count)
		comedi_file_reset(file);
}

static comedi_subdevice* comedi_file_read_subdevice(struct file *file)
{
	struct comedi_file *cfp = file->private_data;

	comedi_file_check(file);
	return cfp->read_subdev;
}

static comedi_subdevice* comedi_file_write_subdevice(struct file *file)
{
	struct comedi_file *cfp = file->private_data;

	comedi_file_check(file);
	return cfp->write_subdev;
}

static long comedi_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;
	int rc;

	mutex_lock(&dev->mutex);

	/* Device config is special, because it must work on
	 * an unconfigured device. */
	if (cmd == COMEDI_DEVCONFIG) {
		if (iminor(file_inode(file)) >= COMEDI_NUM_BOARD_MINORS) {
			/* Device config not appropriate on non-board minors. */
			rc =-ENOTTY;
			goto done;
		}
		rc = do_devconfig_ioctl(dev, (comedi_devconfig __user *)arg);
		if (rc == 0) {
			if (arg == 0 &&
			    dev->minor >= comedi_num_legacy_minors) {
				/* Successfully unconfigured a dynamically
				 * allocated device.  Try and remove it. */
				comedi_device *devr;

				devr = comedi_clear_board_minor(dev->minor);
				if (dev == devr) {
					mutex_unlock(&dev->mutex);
					comedi_free_board_dev(dev);
					return rc;
				}
			}
		}
		goto done;
	}

	if (!dev->attached) {
		DPRINTK("no driver configured on /dev/comedi%i\n", dev->minor);
		rc = -ENODEV;
		goto done;
	}

	switch (cmd) {
	case COMEDI_BUFCONFIG:
		rc = do_bufconfig_ioctl(dev, (comedi_bufconfig __user *)arg);
		break;
	case COMEDI_DEVINFO:
		rc = do_devinfo_ioctl(dev, (comedi_devinfo __user *)arg, file);
		break;
	case COMEDI_SUBDINFO:
		rc = do_subdinfo_ioctl(dev, (comedi_subdinfo __user *)arg, file);
		break;
	case COMEDI_CHANINFO:
		rc = do_chaninfo_ioctl(dev, (comedi_chaninfo __user *)arg);
		break;
	case COMEDI_RANGEINFO:
		rc = do_rangeinfo_ioctl(dev, (comedi_rangeinfo __user *)arg);
		break;
	case COMEDI_BUFINFO:
		rc = do_bufinfo_ioctl(dev, (comedi_bufinfo __user *)arg, file);
		break;
	case COMEDI_LOCK:
		rc = do_lock_ioctl(dev, arg, file);
		break;
	case COMEDI_UNLOCK:
		rc = do_unlock_ioctl(dev, arg, file);
		break;
	case COMEDI_CANCEL:
		rc = do_cancel_ioctl(dev, arg, file);
		break;
	case COMEDI_CMD:
		rc = do_cmd_ioctl(dev, (comedi_cmd __user *)arg, file);
		break;
	case COMEDI_CMDTEST:
		rc = do_cmdtest_ioctl(dev, (comedi_cmd __user *)arg, file);
		break;
	case COMEDI_INSNLIST:
		rc = do_insnlist_ioctl(dev, (comedi_insnlist __user *)arg, file);
		break;
	case COMEDI_INSN:
		rc = do_insn_ioctl(dev, (comedi_insn __user *)arg, file);
		break;
	case COMEDI_POLL:
		rc = do_poll_ioctl(dev, arg, file);
		break;
	case COMEDI_SETRSUBD:
		rc = do_setrsubd_ioctl(dev, arg, file);
		break;
	case COMEDI_SETWSUBD:
		rc = do_setwsubd_ioctl(dev, arg, file);
		break;
	default:
		rc = -ENOTTY;
		break;
	}

      done:
	mutex_unlock(&dev->mutex);
	return rc;
}

#ifndef HAVE_UNLOCKED_IOCTL
static int comedi_ioctl(struct inode *inode, struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return comedi_unlocked_ioctl(file, cmd, arg);
}
#endif /* ifndef HAVE_UNLOCKED_IOCTL */

#ifdef CONFIG_COMPAT

/* Handle 32-bit COMEDI_CHANINFO ioctl. */
static long compat_chaninfo(struct file *file, unsigned long arg)
{
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;
	comedi_chaninfo chaninfo;
	comedi32_chaninfo __user *chaninfo32;
	int err;
	compat_uptr_t temp_uptr = 0;

	chaninfo32 = compat_ptr(arg);
	memset(&chaninfo, 0, sizeof(chaninfo));

	/* Copy chaninfo structure.  Ignore unused members. */
	if (!comedi_access_ok(chaninfo32, sizeof(*chaninfo32))) {
		return -EFAULT;
	}
	err = 0;
	err |= __get_user(chaninfo.subdev, &chaninfo32->subdev);
	err |= __get_user(temp_uptr, &chaninfo32->maxdata_list);
	chaninfo.maxdata_list = (void __force *)compat_ptr(temp_uptr);
	err |= __get_user(temp_uptr, &chaninfo32->flaglist);
	chaninfo.flaglist = (void __force *)compat_ptr(temp_uptr);
	err |= __get_user(temp_uptr, &chaninfo32->rangelist);
	chaninfo.rangelist = (void __force *)compat_ptr(temp_uptr);
	if (err) {
		return -EFAULT;
	}

	mutex_lock(&dev->mutex);
	err = do_chaninfo_i(dev, &chaninfo);
	mutex_unlock(&dev->mutex);
	return err;
}

/* Handle 32-bit COMEDI_RANGEINFO ioctl. */
static long compat_rangeinfo(struct file *file, unsigned long arg)
{
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;
	comedi_rangeinfo rangeinfo;
	comedi32_rangeinfo __user *rangeinfo32;
	int err;
	compat_uptr_t temp_uptr = 0;

	rangeinfo32 = compat_ptr(arg);
	memset(&rangeinfo, 0, sizeof(rangeinfo));

	/* Copy rangeinfo structure. */
	if (!comedi_access_ok(rangeinfo32, sizeof(*rangeinfo32))) {
		return -EFAULT;
	}
	err = 0;
	err |= __get_user(rangeinfo.range_type, &rangeinfo32->range_type);
	err |= __get_user(temp_uptr, &rangeinfo32->range_ptr);
	rangeinfo.range_ptr = (void __force *)compat_ptr(temp_uptr);
	if (err) {
		return -EFAULT;
	}

	mutex_lock(&dev->mutex);
	err = do_rangeinfo_i(dev, &rangeinfo);
	mutex_unlock(&dev->mutex);
	return err;
}

/* Copy 32-bit cmd structure to native cmd structure. */
static int get_compat_cmd(comedi_cmd *cmd,
		comedi32_cmd __user *cmd32)
{
	int err;
	compat_uptr_t temp_uptr;

	/* Copy cmd structure. */
	if (!comedi_access_ok(cmd32, sizeof(*cmd32))) {
		return -EFAULT;
	}
	memset(cmd, 0, sizeof(*cmd));
	err = 0;
	err |= __get_user(cmd->subdev, &cmd32->subdev);
	err |= __get_user(cmd->flags, &cmd32->flags);
	err |= __get_user(cmd->start_src, &cmd32->start_src);
	err |= __get_user(cmd->start_arg, &cmd32->start_arg);
	err |= __get_user(cmd->scan_begin_src, &cmd32->scan_begin_src);
	err |= __get_user(cmd->scan_begin_arg, &cmd32->scan_begin_arg);
	err |= __get_user(cmd->convert_src, &cmd32->convert_src);
	err |= __get_user(cmd->convert_arg, &cmd32->convert_arg);
	err |= __get_user(cmd->scan_end_src, &cmd32->scan_end_src);
	err |= __get_user(cmd->scan_end_arg, &cmd32->scan_end_arg);
	err |= __get_user(cmd->stop_src, &cmd32->stop_src);
	err |= __get_user(cmd->stop_arg, &cmd32->stop_arg);
	err |= __get_user(temp_uptr, &cmd32->chanlist);
	cmd->chanlist = (void __force *)compat_ptr(temp_uptr);
	err |= __get_user(cmd->chanlist_len, &cmd32->chanlist_len);
	err |= __get_user(temp_uptr, &cmd32->data);
	cmd->data = (void __force *)compat_ptr(temp_uptr);
	err |= __get_user(cmd->data_len, &cmd32->data_len);
	return err ? -EFAULT : 0;
}

/* Copy native cmd structure to 32-bit cmd structure. */
static int put_compat_cmd(comedi32_cmd __user *cmd32, comedi_cmd *cmd)
{
	int err;

	/* Copy back most of cmd structure. */
	/* Assume the pointer values are already valid. */
	/* (Could use ptr_to_compat() to set them, but that wasn't implemented
	 * until kernel version 2.6.11.) */
	if (!comedi_access_ok(cmd32, sizeof(*cmd32))) {
		return -EFAULT;
	}
	err = 0;
	err |= __put_user(cmd->subdev, &cmd32->subdev);
	err |= __put_user(cmd->flags, &cmd32->flags);
	err |= __put_user(cmd->start_src, &cmd32->start_src);
	err |= __put_user(cmd->start_arg, &cmd32->start_arg);
	err |= __put_user(cmd->scan_begin_src, &cmd32->scan_begin_src);
	err |= __put_user(cmd->scan_begin_arg, &cmd32->scan_begin_arg);
	err |= __put_user(cmd->convert_src, &cmd32->convert_src);
	err |= __put_user(cmd->convert_arg, &cmd32->convert_arg);
	err |= __put_user(cmd->scan_end_src, &cmd32->scan_end_src);
	err |= __put_user(cmd->scan_end_arg, &cmd32->scan_end_arg);
	err |= __put_user(cmd->stop_src, &cmd32->stop_src);
	err |= __put_user(cmd->stop_arg, &cmd32->stop_arg);
	/* Assume chanlist pointer is unchanged. */
	err |= __put_user(cmd->chanlist_len, &cmd32->chanlist_len);
	/* Assume data pointer is unchanged. */
	err |= __put_user(cmd->data_len, &cmd32->data_len);
	return err ? -EFAULT : 0;
}

/* Handle 32-bit COMEDI_CMD ioctl. */
static long compat_cmd(struct file *file, unsigned long arg)
{
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;
	comedi_cmd cmd;
	comedi32_cmd __user *cmd32;
	int copy = 0;
	long rc;
	int err;

	cmd32 = compat_ptr(arg);

	err = get_compat_cmd(&cmd, cmd32);
	if (err) {
		return err;
	}

	mutex_lock(&dev->mutex);
	rc = do_cmd_i(dev, &cmd, &copy, file);
	mutex_unlock(&dev->mutex);

	if (rc && copy) {
		/* Special case: copy cmd back to user. */
		err = put_compat_cmd(cmd32, &cmd);
		if (err) {
			rc = err;
		}
	}
	return rc;
}

/* Handle 32-bit COMEDI_CMDTEST ioctl. */
static long compat_cmdtest(struct file *file, unsigned long arg)
{
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;
	comedi_cmd cmd;
	comedi32_cmd __user *cmd32;
	int copy = 0;
	long rc;
	int err;

	cmd32 = compat_ptr(arg);

	err = get_compat_cmd(&cmd, cmd32);
	if (err) {
		return err;
	}

	mutex_lock(&dev->mutex);
	rc = do_cmdtest_i(dev, &cmd, &copy, file);
	mutex_unlock(&dev->mutex);

	if (copy) {
		err = put_compat_cmd(cmd32, &cmd);
		if (err) {
			rc = err;
		}
	}
	return rc;
}

/* Copy 32-bit insn structure to native insn structure. */
static int get_compat_insn(comedi_insn *insn,
		comedi32_insn __user *insn32)
{
	int err;
	compat_uptr_t temp_uptr;

	/* Copy insn structure.  Ignore the unused members. */
	err = 0;
	if (!comedi_access_ok(insn32, sizeof(*insn32))) {
		return -EFAULT;
	}
	err |= __get_user(insn->insn, &insn32->insn);
	err |= __get_user(insn->n, &insn32->n);
	err |= __get_user(temp_uptr, &insn32->data);
	insn->data = (void __force *)compat_ptr(temp_uptr);
	err |= __get_user(insn->subdev, &insn32->subdev);
	err |= __get_user(insn->chanspec, &insn32->chanspec);
	return err ? -EFAULT : 0;
}

/* Handle 32-bit COMEDI_INSNLIST ioctl. */
static long compat_insnlist(struct file *file, unsigned long arg)
{
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;
	comedi32_insnlist __user *insnlist32;
	comedi32_insn __user *insn32;
	comedi_insn *insns;
	compat_uptr_t uptr;
	unsigned int n_insns, n;
	int err;

	insnlist32 = compat_ptr(arg);

	/* Get 32-bit insnlist structure.  */
	if (!comedi_access_ok(insnlist32, sizeof(*insnlist32))) {
		return -EFAULT;
	}
	err = 0;
	err |= __get_user(n_insns, &insnlist32->n_insns);
	err |= __get_user(uptr, &insnlist32->insns);
	insn32 = compat_ptr(uptr);
	if (err) {
		return -EFAULT;
	}

	/* Allocate memory to copy insns into. */
	insns = kcalloc(n_insns, sizeof(*insns), GFP_KERNEL);
	if (!insns) {
		DPRINTK("kmalloc failed\n");
		return -ENOMEM;
	}

	/* Copy insn structures. */
	for (n = 0; n < n_insns; n++) {
		err = get_compat_insn(&insns[n], &insn32[n]);
		if (err) {
			goto out;
		}
	}

	mutex_lock(&dev->mutex);
	err = do_insnlist_i(dev, insns, n_insns, file);
	mutex_unlock(&dev->mutex);

out:
	kfree(insns);
	return err;
}

/* Handle 32-bit COMEDI_INSN ioctl. */
static long compat_insn(struct file *file, unsigned long arg)
{
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;
	comedi_insn insn;
	comedi32_insn __user *insn32;
	int err;

	insn32 = compat_ptr(arg);

	err = get_compat_insn(&insn, insn32);
	if (err) {
		return err;
	}

	mutex_lock(&dev->mutex);
	err = do_insn_i(dev, &insn, file);
	mutex_unlock(&dev->mutex);

	return err;
}

/* compat_ioctl file operation. */
/* Returns -ENOIOCTLCMD for unrecognised ioctl codes. */
#ifdef HAVE_COMPAT_IOCTL
static
#endif
long comedi_compat_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	long rc;

#ifndef HAVE_COMPAT_IOCTL
	/*
	 * Called from mapped ioctl handler (for kernel 2.6.10 or earlier).
	 * Do some sanity checking.
	 */
	if (!file->f_op) {
		/* File has no operation handlers. */
		return -ENOTTY;
	}
#ifdef HAVE_UNLOCKED_IOCTL
	/*
	 * N.B. Should not get here because HAVE_COMPAT_IOCTL and
	 * HAVE_UNLOCKED_IOCTL should be both defined or both undefined.
	 */
	if (file->f_op->unlocked_ioctl != comedi_unlocked_ioctl) {
		/* File is using unknown unlocked_ioctl handler. */
		return -ENOTTY;
	}
#else
	if (file->f_op->ioctl != comedi_ioctl) {
		/* File is using unknown ioctl handler. */
		return -ENOTTY;
	}
	/* Do not bother locking the Big Kernel Lock. */
#endif	/* HAVE_UNLOCKED_IOCTL */
#endif	/* ifndef HAVE_COMPAT_IOCTL */

	switch (cmd) {
	case COMEDI_DEVCONFIG:
	case COMEDI_DEVINFO:
	case COMEDI_SUBDINFO:
	case COMEDI_BUFCONFIG:
	case COMEDI_BUFINFO:
		/* Just need to translate the pointer argument. */
		arg = (unsigned long)compat_ptr(arg);
		rc = comedi_unlocked_ioctl(file, cmd, arg);
		break;
	case COMEDI_LOCK:
	case COMEDI_UNLOCK:
	case COMEDI_CANCEL:
	case COMEDI_POLL:
	case COMEDI_SETRSUBD:
	case COMEDI_SETWSUBD:
		/* No translation needed. */
		rc = comedi_unlocked_ioctl(file, cmd, arg);
		break;
	case COMEDI32_CHANINFO:
		rc = compat_chaninfo(file, arg);
		break;
	case COMEDI32_RANGEINFO:
		rc = compat_rangeinfo(file, arg);
		break;
	case COMEDI32_CMD:
		rc = compat_cmd(file, arg);
		break;
	case COMEDI32_CMDTEST:
		rc = compat_cmdtest(file, arg);
		break;
	case COMEDI32_INSNLIST:
		rc = compat_insnlist(file, arg);
		break;
	case COMEDI32_INSN:
		rc = compat_insn(file, arg);
		break;
	default:
		rc = -ENOIOCTLCMD;
		break;
	}
	return rc;
}

#endif /* CONFIG_COMPAT */

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
static int do_devconfig_ioctl(comedi_device * dev,
	comedi_devconfig __user * arg)
{
	comedi_devconfig it;
	int ret;
	unsigned char *aux_data = NULL;
	int aux_len;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	if (arg == NULL) {
		if (is_device_busy(dev))
			return -EBUSY;
		if(dev->attached)
		{
			struct module *driver_module = dev->driver->module;
			comedi_device_detach(dev);
			module_put(driver_module);
		}
		return 0;
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
				(void __user *)comedi_aux_data(it.options, 0),
				aux_len)) {
			ret = -EFAULT;
			goto out_free_aux_data;
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

	if (dev->minor >= comedi_num_legacy_minors) {
		/* Don't re-use dynamically allocated comedi devices. */
		ret = -EBUSY;
		goto out_free_aux_data;
	}

	/* This increments the driver module count on success. */
	ret = comedi_device_attach(dev, &it);

out_free_aux_data:
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
static int do_bufconfig_ioctl(comedi_device * dev, comedi_bufconfig __user *arg)
{
	comedi_bufconfig bc;
	comedi_async *async;
	comedi_subdevice *s;
	int retval = 0;

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
		retval = resize_async_buffer(dev, s, async, bc.size);
		if(retval < 0) return retval;
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
static int do_devinfo_ioctl(comedi_device * dev, comedi_devinfo __user * arg,
	struct file *file)
{
	comedi_devinfo devinfo;
	comedi_subdevice *s;

	memset(&devinfo, 0, sizeof(devinfo));

	/* fill devinfo structure */
	devinfo.version_code = COMEDI_VERSION_CODE;
	devinfo.n_subdevs = dev->n_subdevices;
	strscpy(devinfo.driver_name, dev->driver->driver_name, COMEDI_NAMELEN);
	strscpy(devinfo.board_name, dev->board_name, COMEDI_NAMELEN);

	s = comedi_file_read_subdevice(file);
	if (s) {
		devinfo.read_subdevice = s - dev->subdevices;
	} else {
		devinfo.read_subdevice = -1;
	}
	s = comedi_file_write_subdevice(file);
	if (s) {
		devinfo.write_subdevice = s - dev->subdevices;
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
static int do_subdinfo_ioctl(comedi_device * dev, comedi_subdinfo __user * arg,
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
				(i << 24) | (0 << 16) | (s->
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

		if (s->insn_bits != &insn_inval)
			us->insn_bits_support = COMEDI_SUPPORTED;
		else
			us->insn_bits_support = COMEDI_UNSUPPORTED;

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
static int do_chaninfo_ioctl(comedi_device * dev, comedi_chaninfo __user * arg)
{
	comedi_chaninfo ci;

	if (copy_from_user(&ci, arg, sizeof(ci)))
		return -EFAULT;

	return do_chaninfo_i(dev, &ci);
}

static int do_chaninfo_i(comedi_device * dev, comedi_chaninfo *ci)
{
	comedi_subdevice *s;

	if (ci->subdev >= dev->n_subdevices)
		return -EINVAL;
	s = dev->subdevices + ci->subdev;

	if (ci->maxdata_list) {
		if (s->maxdata || !s->maxdata_list)
			return -EINVAL;
		if (copy_to_user((lsampl_t __user *)ci->maxdata_list,
				s->maxdata_list, s->n_chan * sizeof(lsampl_t)))
			return -EFAULT;
	}

	if (ci->flaglist) {
		if (!s->flaglist)
			return -EINVAL;
		if (copy_to_user((unsigned int __user *)ci->flaglist,
				s->flaglist, s->n_chan * sizeof(unsigned int)))
			return -EFAULT;
	}

	if (ci->rangelist) {
		int i;

		if (!s->range_table_list)
			return -EINVAL;
		for (i = 0; i < s->n_chan; i++) {
			int x;

			x = (dev->minor << 28) | (ci->subdev << 24) |
				(i << 16) | (s->range_table_list[i]->length);
			if (put_user(x, (unsigned int __user *)ci->rangelist + i))
				return -EFAULT;
		}
		//if(copy_to_user((unsigned int __user *)ci->rangelist,s->range_type_list,s->n_chan*sizeof(unsigned int)))
		//      return -EFAULT;
	}

	return 0;
}

/*
	COMEDI_RANGEINFO
	range information ioctl

	arg:
		pointer to rangeinfo structure

	reads:
		range info structure

	writes:
		n comedi_krange structures to rangeinfo->range_ptr
*/
static int do_rangeinfo_ioctl(comedi_device * dev,
		comedi_rangeinfo __user * arg)
{
	comedi_rangeinfo ri;

	if (copy_from_user(&ri, arg, sizeof(ri)))
		return -EFAULT;

	return do_rangeinfo_i(dev, &ri);
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
static int do_bufinfo_ioctl(comedi_device * dev, comedi_bufinfo __user *arg,
	void *file)
{
	comedi_bufinfo bi;
	comedi_subdevice *s;
	comedi_async *async;
	unsigned int runflags;
	int retval = 0;
	int become_nonbusy = 0;

	if (copy_from_user(&bi, arg, sizeof(comedi_bufinfo)))
		return -EFAULT;

	if (bi.subdevice >= dev->n_subdevices || bi.subdevice < 0)
		return -EINVAL;

	s = dev->subdevices + bi.subdevice;

	if (s->lock && s->lock != file)
		return -EACCES;

	async = s->async;
	if (!async || s->busy != file)
		return -EINVAL;

	runflags = comedi_get_subdevice_runflags(s);
	if (!(async->cmd.flags & CMDF_WRITE)) {
		/* command was set up in "read" direction */
		if (bi.bytes_read) {
			bi.bytes_read = comedi_buf_read_alloc(async,
					bi.bytes_read);
			bi.bytes_read = comedi_buf_read_free(async,
					bi.bytes_read);
		}
		/*
		 * If nothing left to read, and command has stopped, and
		 * {"read" position not updated or command stopped normally},
		 * then become non-busy.
		 */
		if (comedi_buf_read_n_available(async) == 0 &&
			!(runflags & SRF_RUNNING) &&
			(bi.bytes_read == 0 || !(runflags & SRF_ERROR))) {
			become_nonbusy = 1;
			if ((runflags & SRF_ERROR) != 0) {
				retval = -EPIPE;
			}
		}
		bi.bytes_written = 0;
	} else {
		/* command was set up in "write" direction */
		if (!(runflags & SRF_RUNNING)) {
			bi.bytes_written = 0;
			become_nonbusy = 1;
			if ((runflags & SRF_ERROR) != 0) {
				retval = -EPIPE;
			}
		} else if (bi.bytes_written) {
			bi.bytes_written = comedi_buf_write_alloc(async,
					bi.bytes_written);
			bi.bytes_written = comedi_buf_write_free(async,
					bi.bytes_written);
		}
		bi.bytes_read = 0;
	}

	bi.buf_write_count = async->buf_write_count;
	bi.buf_write_ptr = async->buf_write_ptr;
	bi.buf_read_count = async->buf_read_count;
	bi.buf_read_ptr = async->buf_read_ptr;

	if (become_nonbusy) {
		do_become_nonbusy(dev, s);
	}

	if (retval)
		return retval;

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
static int do_insnlist_ioctl(comedi_device * dev, comedi_insnlist __user *arg,
	void *file)
{
	comedi_insnlist insnlist;
	comedi_insn *insns = NULL;
	int ret = 0;

	if (copy_from_user(&insnlist, arg, sizeof(insnlist)))
		return -EFAULT;

	insns = kcalloc(insnlist.n_insns, sizeof(*insns), GFP_KERNEL);
	if (!insns) {
		DPRINTK("kmalloc failed\n");
		return -ENOMEM;
	}

	if (copy_from_user(insns, (comedi_insn __user *)insnlist.insns,
			sizeof(*insns) * insnlist.n_insns)) {
		DPRINTK("copy_from_user failed\n");
		ret = -EFAULT;
	} else {
		ret = do_insnlist_i(dev, insns, insnlist.n_insns, file);
	}
	kfree(insns);
	return ret;
}

static int do_insnlist_i(comedi_device * dev, comedi_insn *insns,
	unsigned int n_insns, void *file)
{
	lsampl_t *data = NULL;
	unsigned int max_samples;
	int i;
	int ret = 0;

	max_samples = 0;
	for (i = 0; i < n_insns; i++) {
		if (max_samples < insns[i].n)
			max_samples = insns[i].n;
	}

	if (max_samples) {
		data = kmalloc_array(max_samples, sizeof(*data), GFP_KERNEL);
		if (!data) {
			DPRINTK("kmalloc failed\n");
			ret = -ENOMEM;
			goto error;
		}
	}

	for (i = 0; i < n_insns; i++) {
		if (insns[i].insn & INSN_MASK_WRITE) {
			if (copy_from_user(data,
					(lsampl_t __user *)insns[i].data,
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
			if (copy_to_user((lsampl_t __user *)insns[i].data, data,
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
	if (data)
		kfree(data);

	if (ret < 0)
		return ret;
	return i;
}

static int check_insn_config_length(comedi_insn * insn, lsampl_t * data)
{
	if(insn->n < 1) return -EINVAL;

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
	case INSN_CONFIG_GET_PWM_STATUS:
	case INSN_CONFIG_PWM_SET_PERIOD:
	case INSN_CONFIG_PWM_GET_PERIOD:
		if (insn->n == 2)
			return 0;
		break;
	case INSN_CONFIG_SET_GATE_SRC:
	case INSN_CONFIG_GET_GATE_SRC:
	case INSN_CONFIG_SET_CLOCK_SRC:
	case INSN_CONFIG_GET_CLOCK_SRC:
	case INSN_CONFIG_SET_OTHER_SRC:
	case INSN_CONFIG_GET_COUNTER_STATUS:
	case INSN_CONFIG_PWM_SET_H_BRIDGE:
	case INSN_CONFIG_PWM_GET_H_BRIDGE:
	case INSN_CONFIG_GET_HARDWARE_BUFFER_SIZE:
		if (insn->n == 3)
			return 0;
		break;
	case INSN_CONFIG_PWM_OUTPUT:
	case INSN_CONFIG_ANALOG_TRIG:
		if (insn->n == 5)
			return 0;
		break;
	case INSN_CONFIG_DIGITAL_TRIG:
		if (insn->n == 6)
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
			ret = s->async->inttrig(dev, s, data[0]);
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
		lsampl_t maxdata;

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
			maxdata = s->maxdata_list
				? s->maxdata_list[CR_CHAN(insn->chanspec)]
				: s->maxdata;
			for (i = 0; i < insn->n; ++i) {
				if (data[i] > maxdata) {
					ret = -EINVAL;
					DPRINTK("bad data value(s)\n");
					break;
				}
			}
			if (ret == 0)
				ret = s->insn_write(dev, s, insn, data);
			break;
		case INSN_BITS:
			if (insn->n != 2) {
				ret = -EINVAL;
			} else {
				/* Most drivers ignore the base channel in
				 * insn->chanspec.  Deal with it here if
				 * the subdevice has <= 32 channels. */
				unsigned int shift;
				lsampl_t orig_mask;

				orig_mask = data[0];
				if (s->n_chan <= 32) {
					shift = CR_CHAN(insn->chanspec);
					if (shift > 0) {
						insn->chanspec = 0;
						data[0] <<= shift;
						data[1] <<= shift;
					}
				} else {
					shift = 0;
				}
				ret = s->insn_bits(dev, s, insn, data);
				data[0] = orig_mask;
				if (shift > 0)
					data[1] >>= shift;
			}
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
static int do_insn_ioctl(comedi_device * dev, comedi_insn __user *arg,
	void *file)
{
	comedi_insn insn;

	if (copy_from_user(&insn, arg, sizeof(insn)))
		return -EFAULT;

	return do_insn_i(dev, &insn, file);
}

static int do_insn_i(comedi_device * dev, comedi_insn *insn, void *file)
{
	lsampl_t *data = NULL;
	int ret = 0;

	if (insn->n) {
		data = kmalloc_array(insn->n, sizeof(*data), GFP_KERNEL);
		if (!data) {
			ret = -ENOMEM;
			goto error;
		}
	}

	if (insn->insn & INSN_MASK_WRITE) {
		if (copy_from_user(data, (lsampl_t __user *)insn->data,
				insn->n * sizeof(lsampl_t))) {
			ret = -EFAULT;
			goto error;
		}
	}
	ret = parse_insn(dev, insn, data, file);
	if (ret < 0)
		goto error;
	if (insn->insn & INSN_MASK_READ) {
		if (copy_to_user((lsampl_t __user *)insn->data, data,
				insn->n * sizeof(lsampl_t))) {
			ret = -EFAULT;
			goto error;
		}
	}
	ret = insn->n;

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
static int do_cmd_ioctl(comedi_device * dev, comedi_cmd __user *arg, void *file)
{
	comedi_cmd cmd;
	int copy = 0;
	int rc;

	if (copy_from_user(&cmd, arg, sizeof(cmd))) {
		DPRINTK("bad cmd address\n");
		return -EFAULT;
	}
	rc = do_cmd_i(dev, &cmd, &copy, file);
	if (rc && copy && copy_to_user(arg, &cmd, sizeof(cmd))) {
		DPRINTK("fault writing cmd\n");
		rc = -EFAULT;
	}
	return rc;
}

static int do_cmd_i(comedi_device * dev, comedi_cmd *cmd, int *copy, void *file)
{
	comedi_subdevice *s;
	comedi_async *async;
	int ret = 0;
	unsigned int *chanlist_saver = NULL;

	*copy = 0;

	// save user's chanlist pointer so it can be restored later
	chanlist_saver = cmd->chanlist;

	if (cmd->subdev >= dev->n_subdevices) {
		DPRINTK("%d no such subdevice\n", cmd->subdev);
		return -ENODEV;
	}

	s = dev->subdevices + cmd->subdev;
	async = s->async;

	if (s->type == COMEDI_SUBD_UNUSED) {
		DPRINTK("%d not valid subdevice\n", cmd->subdev);
		return -EIO;
	}

	if (!s->do_cmd || !s->do_cmdtest || !s->async) {
		DPRINTK("subdevice %i does not support commands\n",
			cmd->subdev);
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

	/* make sure channel/gain list isn't too long */
	if (cmd->chanlist_len > s->len_chanlist) {
		DPRINTK("channel/gain list too long %u > %d\n",
			cmd->chanlist_len, s->len_chanlist);
		return -EINVAL;
	}

	/* make sure channel/gain list isn't too short */
	if (cmd->chanlist_len < 1) {
		DPRINTK("channel/gain list too short %u < 1\n",
			cmd->chanlist_len);
		return -EINVAL;
	}

	async->cmd = *cmd;
	async->cmd.data = NULL;
	/* load channel/gain list */
	async->cmd.chanlist =
		kmalloc(async->cmd.chanlist_len * sizeof(int), GFP_KERNEL);
	if (!async->cmd.chanlist) {
		DPRINTK("allocation failed\n");
		return -ENOMEM;
	}

	if (copy_from_user(async->cmd.chanlist,
			(unsigned int __user *)cmd->chanlist,
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
		*cmd = async->cmd;
		// restore chanlist pointer before copying back
		cmd->chanlist = chanlist_saver;
		cmd->data = NULL;
		*copy = 1;
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

#ifdef COMEDI_CONFIG_RT
	if (async->cmd.flags & TRIG_RT) {
		if (comedi_switch_to_rt(dev) == 0)
			comedi_set_subdevice_runflags(s, SRF_RT, SRF_RT);
	}
#endif

	s->busy = file;
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
static int do_cmdtest_ioctl(comedi_device * dev, comedi_cmd __user *arg,
	void *file)
{
	comedi_cmd cmd;
	int copy = 0;
	int rc;

	if (copy_from_user(&cmd, arg, sizeof(cmd))) {
		DPRINTK("bad cmd address\n");
		return -EFAULT;
	}
	rc = do_cmdtest_i(dev, &cmd, &copy, file);
	if (copy && copy_to_user(arg, &cmd, sizeof(cmd))) {
		DPRINTK("bad cmd address\n");
		rc = -EFAULT;
	}
	return rc;
}

static int do_cmdtest_i(comedi_device * dev, comedi_cmd *cmd, int *copy,
	void *file)
{
	comedi_subdevice *s;
	int ret = 0;
	unsigned int *chanlist = NULL;
	unsigned int *chanlist_saver = NULL;

	*copy = 0;

	// save user's chanlist pointer so it can be restored later
	chanlist_saver = cmd->chanlist;

	if (cmd->subdev >= dev->n_subdevices) {
		DPRINTK("%d no such subdevice\n", cmd->subdev);
		return -ENODEV;
	}

	s = dev->subdevices + cmd->subdev;
	if (s->type == COMEDI_SUBD_UNUSED) {
		DPRINTK("%d not valid subdevice\n", cmd->subdev);
		return -EIO;
	}

	if (!s->do_cmd || !s->do_cmdtest) {
		DPRINTK("subdevice %i does not support commands\n",
			cmd->subdev);
		return -EIO;
	}

	/* make sure channel/gain list isn't too long */
	if (cmd->chanlist_len > s->len_chanlist) {
		DPRINTK("channel/gain list too long %d > %d\n",
			cmd->chanlist_len, s->len_chanlist);
		ret = -EINVAL;
		goto cleanup;
	}

	/* load channel/gain list */
	if (cmd->chanlist) {
		chanlist =
			kmalloc(cmd->chanlist_len * sizeof(int),
			GFP_KERNEL);
		if (!chanlist) {
			DPRINTK("allocation failed\n");
			ret = -ENOMEM;
			goto cleanup;
		}

		if (copy_from_user(chanlist,
				(unsigned int __user *)cmd->chanlist,
				cmd->chanlist_len * sizeof(int))) {
			DPRINTK("fault reading chanlist\n");
			ret = -EFAULT;
			goto cleanup;
		}

		/* make sure each element in channel/gain list is valid */
		if ((ret = check_chanlist(s, cmd->chanlist_len,
					chanlist)) < 0) {
			DPRINTK("bad chanlist\n");
			goto cleanup;
		}

		cmd->chanlist = chanlist;
	}

	ret = s->do_cmdtest(dev, s, cmd);

	// restore chanlist pointer before copying back
	cmd->chanlist = chanlist_saver;

	*copy = 1;

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

	comedi_spin_lock_irqsave(&s->spin_lock, flags);
	if (s->busy || s->lock) {
		ret = -EBUSY;
	} else {
		s->lock = file;
	}
	comedi_spin_unlock_irqrestore(&s->spin_lock, flags);

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
	int ret;

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

	ret = do_cancel(dev, s);

	if (comedi_get_subdevice_runflags(s) & SRF_USER) {
		/* wake up waiters */
		comedi_async *async = s->async;

		if (dev->rt) {
#ifdef COMEDI_CONFIG_RT
			// pend wake up
			comedi_rt_pend_wakeup(&async->wait_head);
#else
			printk("BUG: do_cancel_ioctl() code unreachable\n");
#endif
		} else {
			wake_up_interruptible(&async->wait_head);
		}
	}

	return ret;
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

/*
	COMEDI_SETRSUBD ioctl
	sets the current "read" subdevice on a per-file basis

	arg:
		subdevice number

	reads:
		nothing

	writes:
		nothing

*/
static int do_setrsubd_ioctl(comedi_device *dev, unsigned long arg,
	struct file *file)
{
	struct comedi_file *cfp = file->private_data;
	comedi_subdevice *s_old, *s_new;

	if (arg >= dev->n_subdevices)
		return -EINVAL;

	s_new = dev->subdevices + arg;
	s_old = comedi_file_read_subdevice(file);
	if (s_old == s_new)
		return 0;	/* no change */

	if (!(s_new->subdev_flags & SDF_CMD_READ))
		return -EINVAL;

	/*
	 * Check the file isn't still busy handling a "read" command on the
	 * old subdevice (if any).
	 */
	if (s_old && s_old->busy == file && s_old->async &&
		!(s_old->async->cmd.flags & CMDF_WRITE)) {
		return -EBUSY;
	}

	cfp->read_subdev = s_new;
	return 0;
}

/*
	COMEDI_SETWSUBD ioctl
	sets the current "write" subdevice on a per-file basis

	arg:
		subdevice number

	reads:
		nothing

	writes:
		nothing

*/
static int do_setwsubd_ioctl(comedi_device *dev, unsigned long arg,
	struct file *file)
{
	struct comedi_file *cfp = file->private_data;
	comedi_subdevice *s_old, *s_new;

	if (arg >= dev->n_subdevices)
		return -EINVAL;

	s_new = dev->subdevices + arg;
	s_old = comedi_file_write_subdevice(file);
	if (s_old == s_new)
		return 0;	/* no change */

	if (!(s_new->subdev_flags & SDF_CMD_WRITE))
		return -EINVAL;

	/*
	 * Check the file isn't still busy handling a "write" command on the
	 * old subdevice (if any).
	 */
	if (s_old && s_old->busy == file && s_old->async &&
		(s_old->async->cmd.flags & CMDF_WRITE)) {
		return -EBUSY;
	}

	cfp->write_subdev = s_new;
	return 0;
}

static int do_cancel(comedi_device * dev, comedi_subdevice * s)
{
	int ret = 0;

	if ((comedi_get_subdevice_runflags(s) & SRF_RUNNING) && s->cancel)
		ret = s->cancel(dev, s);

	do_become_nonbusy(dev, s);

	return ret;
}

static void comedi_vm_open(struct vm_area_struct *area)
{
	comedi_async *async;
	comedi_device *dev;

	async = area->vm_private_data;
	dev = async->subdevice->device;

	mutex_lock(&dev->mutex);
	async->mmap_count++;
	mutex_unlock(&dev->mutex);
}

static void comedi_vm_close(struct vm_area_struct *area)
{
	comedi_async *async;
	comedi_device *dev;

	async = area->vm_private_data;
	dev = async->subdevice->device;

	mutex_lock(&dev->mutex);
	async->mmap_count--;
	mutex_unlock(&dev->mutex);
}

static struct vm_operations_struct comedi_vm_ops = {
	.open = comedi_vm_open,
	.close = comedi_vm_close,
};

static int comedi_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct comedi_file *cfp = file->private_data;
	comedi_async *async = NULL;
	unsigned long vm_start = vma->vm_start;
	unsigned long vm_end = vma->vm_end;
	unsigned long start = vm_start;
	unsigned long size;
	int n_pages;
	int i;
	int retval = 0;
	comedi_subdevice *s;
	comedi_device *dev = cfp->dev;

	mutex_lock(&dev->mutex);
	if (!dev->attached) {
		DPRINTK("no driver configured on comedi%i\n", dev->minor);
		retval = -ENODEV;
		goto done;
	}
	if (vma->vm_flags & VM_WRITE) {
		s = comedi_file_write_subdevice(file);
	} else {
		s = comedi_file_read_subdevice(file);
	}
	if (s == NULL) {
		retval = -EINVAL;
		goto done;
	}
	async = s->async;
	if (async == NULL) {
		retval = -EINVAL;
		goto done;
	}

	if (vma->vm_pgoff != 0) {
		DPRINTK("comedi: mmap() offset must be 0.\n");
		retval = -EINVAL;
		goto done;
	}

	size = vma->vm_end - vma->vm_start;
	if (size > async->prealloc_bufsz) {
		retval = -EFAULT;
		goto done;
	}
	if (size & (~PAGE_MASK)) {
		retval = -EFAULT;
		goto done;
	}

	n_pages = size >> PAGE_SHIFT;
	for (i = 0; i < n_pages; ++i) {
		if (s->async_dma_dir != DMA_NONE) {
			/*
			 * Temporarily modify VMA start and end addresses
			 * for calling dma_mmap_coherent()
			 */
			vma->vm_start = start;
			vma->vm_end = start + PAGE_SIZE;
			if (dma_mmap_coherent(dev->hw_dev, vma,
					async->buf_page_list[i].virt_addr,
					async->buf_page_list[i].dma_addr,
					PAGE_SIZE)) {
				break;
			}
		} else {
			if (remap_pfn_range(vma, start,
					page_to_pfn(virt_to_page(async->
							buf_page_list[i].
							virt_addr)),
					PAGE_SIZE, PAGE_SHARED)) {
				break;
			}
		}
		start += PAGE_SIZE;
	}
	/* Restore VMA start and end address */
	vma->vm_start = vm_start;
	vma->vm_end = vm_end;
	if (i < n_pages) {
		/* Mapping failed (at least partially) */
		retval = -EAGAIN;
		goto done;
	}

	vma->vm_ops = &comedi_vm_ops;
	vma->vm_private_data = async;

	async->mmap_count++;

	retval = 0;
      done:
	mutex_unlock(&dev->mutex);
	return retval;
}

static comedi_poll_t comedi_poll(struct file *file, poll_table * wait)
{
	comedi_poll_t mask = 0;
	struct comedi_file *cfp = file->private_data;
	comedi_subdevice *read_subdev;
	comedi_subdevice *write_subdev;
	comedi_device *dev = cfp->dev;

	mutex_lock(&dev->mutex);
	if (!dev->attached) {
		DPRINTK("no driver configured on comedi%i\n", dev->minor);
		mutex_unlock(&dev->mutex);
		return 0;
	}

	mask = 0;
	read_subdev = comedi_file_read_subdevice(file);
	if (read_subdev && read_subdev->async) {
		poll_wait(file, &read_subdev->async->wait_head, wait);
		if (!read_subdev->busy
			|| comedi_buf_read_n_available(read_subdev->async) > 0
			|| !(comedi_get_subdevice_runflags(read_subdev) &
				SRF_RUNNING)) {
			mask |= COMEDI_EPOLLIN | COMEDI_EPOLLRDNORM;
		}
	}
	write_subdev = comedi_file_write_subdevice(file);
	if (write_subdev && write_subdev->async) {
		if (write_subdev != read_subdev) {
			poll_wait(file, &write_subdev->async->wait_head, wait);
		}
		comedi_buf_write_alloc(write_subdev->async, write_subdev->async->prealloc_bufsz);
		if (!write_subdev->busy
			|| !(comedi_get_subdevice_runflags(write_subdev) &
				SRF_RUNNING)
			|| comedi_buf_write_n_allocated(write_subdev->async) >=
			bytes_per_sample(write_subdev->async->subdevice)) {
			mask |= COMEDI_EPOLLOUT | COMEDI_EPOLLWRNORM;
		}
	}

	mutex_unlock(&dev->mutex);
	return mask;
}

static unsigned int comedi_buf_copy_to_user(comedi_async * async,
	void __user *dest, unsigned int src_offset, unsigned int n)
{
	unsigned int src_page = src_offset >> PAGE_SHIFT;
	unsigned int src_page_offset = src_offset & ~PAGE_MASK;

	while (n) {
		unsigned int copy_amount = PAGE_SIZE - src_page_offset;
		unsigned int uncopied;

		if (copy_amount > n)
			copy_amount = n;

		uncopied = copy_to_user(dest,
				async->buf_page_list[src_page].virt_addr +
					src_page_offset, copy_amount);
		copy_amount -= uncopied;
		n -= copy_amount;
		if (uncopied) {
			break;
		}

		dest += copy_amount;
		src_page++;
		src_page_offset = 0;
	}
	return n;
}

static unsigned int comedi_buf_copy_from_user(comedi_async * async,
	unsigned int dst_offset, const void __user *src, unsigned int n)
{
	unsigned int dst_page = dst_offset >> PAGE_SHIFT;
	unsigned int dst_page_offset = dst_offset & ~PAGE_MASK;

	while (n) {
		unsigned int copy_amount = PAGE_SIZE - dst_page_offset;
		unsigned int uncopied;

		if (copy_amount > n)
			copy_amount = n;

		uncopied = copy_from_user(async->buf_page_list[dst_page].
				virt_addr + dst_page_offset, src, copy_amount);
		copy_amount -= uncopied;
		n -= copy_amount;
		if (uncopied) {
			break;
		}

		src += copy_amount;
		dst_page++;
		dst_page_offset = 0;
	}
	return n;
}

static ssize_t comedi_write(struct file *file, const char __user *buf,
	size_t nbytes, loff_t * offset)
{
	comedi_subdevice *s;
	comedi_async *async;
	int n, m, count = 0, retval = 0;
	DECLARE_WAITQUEUE(wait, current);
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;

	if (!dev->attached) {
		DPRINTK("no driver configured on comedi%i\n", dev->minor);
		retval = -ENODEV;
		goto done;
	}

	s = comedi_file_write_subdevice(file);
	if (s == NULL) {
		retval = -EIO;
		goto done;
	}
	async = s->async;
	if (async == NULL) {
		retval = -EIO;
		goto done;
	}

	if (!nbytes) {
		retval = 0;
		goto done;
	}
	if (!s->busy) {
		retval = 0;
		goto done;
	}
	if (s->busy != file) {
		retval = -EACCES;
		goto done;
	}
	add_wait_queue(&async->wait_head, &wait);
	while (nbytes > 0 && !retval) {
		set_current_state(TASK_INTERRUPTIBLE);

		if (!(comedi_get_subdevice_runflags(s) & SRF_RUNNING)) {
			if (count == 0) {
				set_current_state(TASK_RUNNING);
				mutex_lock(&dev->mutex);
				if (comedi_get_subdevice_runflags(s) &
					SRF_ERROR) {
					retval = -EPIPE;
				} else {
					retval = 0;
				}
				do_become_nonbusy(dev, s);
				mutex_unlock(&dev->mutex);
			}
			break;
		}

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
			schedule();
			if (signal_pending(current)) {
				retval = -ERESTARTSYS;
				break;
			}
			if (!s->busy) {
				break;
			}
			if (s->busy != file) {
				retval = -EACCES;
				break;
			}
			continue;
		}

		set_current_state(TASK_RUNNING);
		m = comedi_buf_copy_from_user(async, async->buf_write_ptr,
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

done:
	return (count ? count : retval);
}

static ssize_t comedi_read(struct file *file, char __user *buf, size_t nbytes,
	loff_t * offset)
{
	comedi_subdevice *s;
	comedi_async *async;
	int n, m, count = 0, retval = 0;
	DECLARE_WAITQUEUE(wait, current);
	struct comedi_file *cfp = file->private_data;
	comedi_device *dev = cfp->dev;

	if (!dev->attached) {
		DPRINTK("no driver configured on comedi%i\n", dev->minor);
		retval = -ENODEV;
		goto done;
	}

	s = comedi_file_read_subdevice(file);
	if (s == NULL) {
		retval = -EIO;
		goto done;
	}
	async = s->async;
	if (async == NULL) {
		retval = -EIO;
		goto done;
	}
	if (!nbytes) {
		retval = 0;
		goto done;
	}
	if (!s->busy) {
		retval = 0;
		goto done;
	}
	if (s->busy != file) {
		retval = -EACCES;
		goto done;
	}

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
			if (!(comedi_get_subdevice_runflags(s) & SRF_RUNNING)) {
				set_current_state(TASK_RUNNING);
				mutex_lock(&dev->mutex);
				if (comedi_get_subdevice_runflags(s) &
					SRF_ERROR) {
					retval = -EPIPE;
				} else {
					retval = 0;
				}
				do_become_nonbusy(dev, s);
				mutex_unlock(&dev->mutex);
				break;
			}
			if (file->f_flags & O_NONBLOCK) {
				retval = -EAGAIN;
				break;
			}
			schedule();
			if (signal_pending(current)) {
				retval = -ERESTARTSYS;
				break;
			}
			if (!s->busy) {
				retval = 0;
				break;
			}
			if (s->busy != file) {
				retval = -EACCES;
				break;
			}
			continue;
		}
		set_current_state(TASK_RUNNING);
		m = comedi_buf_copy_to_user(async, buf, async->buf_read_ptr, n);
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
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&async->wait_head, &wait);
	if (!(comedi_get_subdevice_runflags(s) & (SRF_ERROR | SRF_RUNNING))) {
		mutex_lock(&dev->mutex);
		if (async->buf_read_count - async->buf_write_count == 0)
			do_become_nonbusy(dev, s);
		mutex_unlock(&dev->mutex);
	}

done:
	return (count ? count : retval);
}

/*
   This function restores a subdevice to an idle state.
 */
static void do_become_nonbusy(comedi_device * dev, comedi_subdevice * s)
{
	comedi_async *async = s->async;

	comedi_set_subdevice_runflags(s, SRF_RUNNING, 0);
#ifdef COMEDI_CONFIG_RT
	if (comedi_get_subdevice_runflags(s) & SRF_RT) {
		comedi_switch_to_non_rt(dev);
		comedi_set_subdevice_runflags(s, SRF_RT, 0);
	}
#endif
	if (async) {
		comedi_reset_async_buf(async);
		async->inttrig = NULL;
		kfree(async->cmd.chanlist);
		async->cmd.chanlist = NULL;
	} else {
		printk("BUG: (?) do_become_nonbusy called with async=0\n");
	}

	s->busy = NULL;
}

static int comedi_open(struct inode *inode, struct file *file)
{
	const unsigned minor = iminor(inode);
	struct comedi_file *cfp;
	comedi_device *dev = comedi_get_device_by_minor(minor);
	int rc;

	if (dev == NULL) {
		DPRINTK("invalid minor number\n");
		return -ENODEV;
	}

	cfp = kzalloc(sizeof(*cfp), GFP_KERNEL);
	if (!cfp) {
		return -ENOMEM;
	}

	cfp->dev = dev;

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
	mutex_lock(&dev->mutex);
	if (dev->attached)
		goto ok;
	if (!capable(CAP_SYS_MODULE) && dev->in_request_module) {
		DPRINTK("in request module\n");
		rc = -ENODEV;
		goto out;
	}
	if (capable(CAP_SYS_MODULE) && dev->in_request_module)
		goto ok;

	dev->in_request_module = 1;

#ifdef CONFIG_KMOD
	mutex_unlock(&dev->mutex);
	request_module("char-major-%i-%i", COMEDI_MAJOR, dev->minor);
	mutex_lock(&dev->mutex);
#endif

	dev->in_request_module = 0;

	if (!dev->attached && !capable(CAP_SYS_MODULE)) {
		DPRINTK("not attached and not CAP_SYS_MODULE\n");
		rc = -ENODEV;
		goto out;
	}
ok:
	__module_get(THIS_MODULE);

	if (dev->attached) {
		if (!try_module_get(dev->driver->module)) {
			module_put(THIS_MODULE);
			rc = -ENOSYS;
			goto out;
		}
	}

	if (dev->attached && dev->use_count == 0 && dev->open) {
		rc = dev->open(dev);
		if (rc < 0) {
			module_put(dev->driver->module);
			module_put(THIS_MODULE);
			goto out;
		}
	}

	dev->use_count++;
	file->private_data = cfp;
	comedi_file_reset(file);
	rc = 0;

out:
	mutex_unlock(&dev->mutex);
	if (rc) {
		kfree(cfp);
	}
	return rc;
}

static int comedi_close(struct inode *inode, struct file *file)
{
	struct comedi_file *cfp = file->private_data;
	comedi_subdevice *s = NULL;
	int i;
	comedi_device *dev = cfp->dev;

	mutex_lock(&dev->mutex);

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

	mutex_unlock(&dev->mutex);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
	if (file->f_flags & FASYNC) {
		comedi_fasync(-1, file, 0);
	}
#endif

	kfree(cfp);

	return 0;
}

static int comedi_fasync(int fd, struct file *file, int on)
{
	const unsigned minor = iminor(file_inode(file));
	comedi_device *dev = comedi_get_device_by_minor(minor);

	if (dev==NULL) return -ENODEV;

	return fasync_helper(fd, file, on, &dev->async_queue);
}

static const struct file_operations comedi_fops = {
      owner:THIS_MODULE,
#ifdef HAVE_UNLOCKED_IOCTL
      unlocked_ioctl:comedi_unlocked_ioctl,
#else
      ioctl:comedi_ioctl,
#endif
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL)
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

static struct class *comedi_class = NULL;
static struct cdev comedi_cdev;

static void comedi_cleanup_legacy_minors(void)
{
	unsigned i;
	for (i = 0; i < comedi_num_legacy_minors; i++) {
		comedi_free_board_minor(i);
	}
}

static int __init comedi_init(void)
{
	int i;
	int retval;

	printk("comedi: version " COMEDI_RELEASE
		" - http://www.comedi.org\n");

	if(comedi_num_legacy_minors < 0 || comedi_num_legacy_minors > COMEDI_NUM_BOARD_MINORS)
	{
		printk("comedi:  error: invalid value for module parameter \"comedi_num_legacy_minors\".  Valid "
			"values are 0 through %i.\n", COMEDI_NUM_BOARD_MINORS);
		return -EINVAL;
	}
	/* comedi is unusable if both comedi_autoconfig and comedi_num_legacy_minors are zero,
		so we might as well adjust the defaults in that case */
	if(comedi_autoconfig == 0 && comedi_num_legacy_minors == 0)
	{
		comedi_num_legacy_minors = 16;
	}

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
	comedi_class = class_create("comedi");
	if (IS_ERR(comedi_class)) {
		printk("comedi: failed to create class\n");
		cdev_del(&comedi_cdev);
		unregister_chrdev_region(MKDEV(COMEDI_MAJOR, 0),
			COMEDI_NUM_MINORS);
		return PTR_ERR(comedi_class);
	}

	// create devices files for legacy/manual use
	for (i = 0; i < comedi_num_legacy_minors; i++) {
		comedi_device *dev;
		dev = comedi_alloc_board_minor(NULL);
		if(IS_ERR(dev)) {
			comedi_cleanup_legacy_minors();
			class_destroy(comedi_class);
			cdev_del(&comedi_cdev);
			unregister_chrdev_region(MKDEV(COMEDI_MAJOR, 0),
				COMEDI_NUM_MINORS);
			return PTR_ERR(dev);
		} else {
			/* comedi_alloc_board_minor() locked the mutex */
			mutex_unlock(&dev->mutex);
		}
	}

	/* XXX requires /proc interface */
	comedi_proc_init();

	comedi_rt_init();

	comedi_register_ioctl32();

	return 0;
}

static void __exit comedi_cleanup(void)
{
	int i;

	comedi_cleanup_legacy_minors();
	for(i = 0; i < COMEDI_NUM_BOARD_MINORS; ++i)
	{
		BUG_ON(comedi_board_minor_table[i]);
	}
	for(i = 0; i < COMEDI_NUM_SUBDEVICE_MINORS; ++i)
	{
		BUG_ON(comedi_subdevice_minor_table[i]);
	}

	class_destroy(comedi_class);
	cdev_del(&comedi_cdev);
	unregister_chrdev_region(MKDEV(COMEDI_MAJOR, 0), COMEDI_NUM_MINORS);

	comedi_proc_cleanup();

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
#ifdef COMEDI_CONFIG_RT
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

	comedi_spin_lock_irqsave(&s->spin_lock, flags);
	s->runflags &= ~mask;
	s->runflags |= (bits & mask);
	comedi_spin_unlock_irqrestore(&s->spin_lock, flags);
}

unsigned comedi_get_subdevice_runflags(comedi_subdevice * s)
{
	unsigned long flags;
	unsigned runflags;

	comedi_spin_lock_irqsave(&s->spin_lock, flags);
	runflags = s->runflags;
	comedi_spin_unlock_irqrestore(&s->spin_lock, flags);
	return runflags;
}

static int is_device_busy(comedi_device * dev)
{
	comedi_subdevice *s;
	int i;

	if (!dev->attached)
		return 0;

	for (i = 0; i < dev->n_subdevices; i++) {
		s = dev->subdevices + i;
		if (s->busy)
			return 1;
		if (s->async && s->async->mmap_count)
			return 1;
	}

	return 0;
}

static void comedi_device_init(comedi_device *dev)
{
	memset(dev, 0, sizeof(comedi_device));
	spin_lock_init(&dev->spinlock);
	mutex_init(&dev->mutex);
	dev->minor = -1;
}

static void comedi_device_cleanup(comedi_device *dev)
{
	if(dev == NULL) return;
	mutex_lock(&dev->mutex);
	comedi_device_detach(dev);
	mutex_unlock(&dev->mutex);
	mutex_destroy(&dev->mutex);
}

static void comedi_free_board_dev(comedi_device *dev)
{
	if(dev) {
		comedi_device_cleanup(dev);
		if(dev->class_dev) {
			COMEDI_DEVICE_DESTROY(comedi_class,
				MKDEV(COMEDI_MAJOR, dev->minor));
		}
		kfree(dev);
	}
}

/* Note: the ->mutex is pre-locked on successful return */
comedi_device *comedi_alloc_board_minor(struct device *hardware_device)
{
	unsigned long flags;
	comedi_device *dev;
	comedi_device_create_t *csdev;
	unsigned i;
	int retval;
	static comedi_device_attribute_t * const dev_attrs[] = {
		&dev_attr_driver_name,
		&dev_attr_board_name,
		&dev_attr_bydrivername_index,
		&dev_attr_byboardname_index,
		&dev_attr_max_read_buffer_kb,
		&dev_attr_read_buffer_kb,
		&dev_attr_max_write_buffer_kb,
		&dev_attr_write_buffer_kb,
	};
	unsigned at;

	dev = kzalloc(sizeof(comedi_device), GFP_KERNEL);
	if(dev == NULL) {
		return ERR_PTR(-ENOMEM);
	}
	comedi_device_init(dev);
	comedi_set_hw_dev(dev, hardware_device);
	mutex_lock(&dev->mutex);
	comedi_spin_lock_irqsave(&comedi_board_minor_table_lock, flags);
	for(i = 0; i < COMEDI_NUM_BOARD_MINORS; ++i) {
		if (comedi_board_minor_table[i] == NULL) {
			comedi_board_minor_table[i] = dev;
			break;
		}
	}
	comedi_spin_unlock_irqrestore(&comedi_board_minor_table_lock, flags);
	if(i == COMEDI_NUM_BOARD_MINORS) {
		printk("comedi: error: ran out of minor numbers for board device files.\n");
		mutex_unlock(&dev->mutex);
		comedi_device_cleanup(dev);
		kfree(dev);
		return ERR_PTR(-EBUSY);
	}
	dev->minor = i;
	csdev = COMEDI_DEVICE_CREATE(comedi_class, NULL,
		MKDEV(COMEDI_MAJOR, i), NULL, hardware_device, "comedi%i", i);
	if(!IS_ERR(csdev)) {
		dev->class_dev = csdev;
	}
	/* Using minor device number as the drvdata. */
	COMEDI_DEV_SET_DRVDATA(csdev, (void *)(unsigned long)i);
	for (at = 0; at < ARRAY_SIZE(dev_attrs); at++) {
		retval = COMEDI_DEVICE_CREATE_FILE(csdev, dev_attrs[at]);
		if (retval) {
			printk(KERN_ERR "comedi: failed to create sysfs attribute file \"%s\".\n",
			       dev_attrs[at]->attr.name);
			mutex_unlock(&dev->mutex);
			comedi_free_board_minor(i);
			return ERR_PTR(retval);
		}
	}
	/* Note: dev->mutex needs to be unlocked by the caller. */
	return dev;
}

void comedi_free_board_minor(unsigned minor)
{
	BUG_ON(minor >= COMEDI_NUM_BOARD_MINORS);
	comedi_free_board_dev(comedi_clear_board_minor(minor));
}

int comedi_alloc_subdevice_minor(comedi_device *dev, comedi_subdevice *s)
{
	unsigned long flags;
	comedi_device_create_t *csdev;
	unsigned i;
	int retval;
	static comedi_device_attribute_t * const dev_attrs[] = {
		&dev_attr_max_read_buffer_kb,
		&dev_attr_read_buffer_kb,
		&dev_attr_max_write_buffer_kb,
		&dev_attr_write_buffer_kb,
	};
	unsigned at;

	comedi_spin_lock_irqsave(&comedi_subdevice_minor_table_lock, flags);
	for(i = 0; i < COMEDI_NUM_SUBDEVICE_MINORS; ++i)
	{
		if(comedi_subdevice_minor_table[i] == NULL)
		{
			comedi_subdevice_minor_table[i] = s;
			break;
		}
	}
	comedi_spin_unlock_irqrestore(&comedi_subdevice_minor_table_lock, flags);
	if(i == COMEDI_NUM_SUBDEVICE_MINORS)
	{
		printk("comedi: error: ran out of minor numbers for subdevice files.\n");
		return -EBUSY;
	}
	i += COMEDI_NUM_BOARD_MINORS;
	s->minor = i;
	csdev = COMEDI_DEVICE_CREATE(comedi_class, dev->class_dev,
		MKDEV(COMEDI_MAJOR, i), NULL, NULL, "comedi%i_subd%i", dev->minor, (int)(s - dev->subdevices));
	if(!IS_ERR(csdev))
	{
		s->class_dev = csdev;
	}
	/* Using minor device number as the drvdata. */
	COMEDI_DEV_SET_DRVDATA(csdev, (void *)(unsigned long)i);
	for (at = 0; at < ARRAY_SIZE(dev_attrs); at++) {
		retval = COMEDI_DEVICE_CREATE_FILE(csdev, dev_attrs[at]);
		if(retval) {
			printk(KERN_ERR "comedi: failed to create sysfs attribute file \"%s\".\n",
				dev_attrs[at]->attr.name);
			comedi_free_subdevice_minor(s);
			return retval;
		}
	}
	return i;
}

void comedi_free_subdevice_minor(comedi_subdevice *s)
{
	unsigned long flags;
	unsigned int i;

	if(s == NULL)
		return;
	if(s->minor < COMEDI_NUM_BOARD_MINORS || s->minor >= COMEDI_NUM_MINORS)
		return;

	i = s->minor - COMEDI_NUM_BOARD_MINORS;
	comedi_spin_lock_irqsave(&comedi_subdevice_minor_table_lock, flags);
	if (s == comedi_subdevice_minor_table[i])
		comedi_subdevice_minor_table[i] = NULL;
	comedi_spin_unlock_irqrestore(&comedi_subdevice_minor_table_lock, flags);

	if(s->class_dev)
	{
		COMEDI_DEVICE_DESTROY(comedi_class,
			MKDEV(COMEDI_MAJOR, s->minor));
		s->class_dev = NULL;
	}
}

comedi_device *comedi_get_device_by_minor(unsigned minor)
{
	BUG_ON(minor >= COMEDI_NUM_MINORS);
	if (minor < COMEDI_NUM_BOARD_MINORS)
		return comedi_get_device_by_board_minor(minor);

	return comedi_get_device_by_subdevice_minor(minor);
}

static int resize_async_buffer(comedi_device *dev,
	comedi_subdevice *s, comedi_async *async, unsigned new_size)
{
	int retval;

	if (new_size > async->max_bufsize)
		return -EPERM;

	if (s->busy) {
		DPRINTK("subdevice is busy, cannot resize buffer\n");
		return -EBUSY;
	}
	if (async->mmap_count) {
		DPRINTK("subdevice is mmapped, cannot resize buffer\n");
		return -EBUSY;
	}

	/* make sure buffer is an integral number of pages
		* (we round up) */
	new_size = (new_size + PAGE_SIZE - 1) & PAGE_MASK;

	retval = comedi_buf_alloc(dev, s, new_size);
	if (retval < 0)
		return retval;

	if (s->buf_change) {
		retval = s->buf_change(dev, s, new_size);
		if (retval < 0)
			return retval;
	}

	DPRINTK("comedi%i subd %d buffer resized to %i bytes\n",
		dev->minor, (int)(s - dev->subdevices), async->prealloc_bufsz);
	return 0;
}

// sysfs attribute file functions

static const unsigned bytes_per_kibi = 1024;

static COMEDI_DECLARE_ATTR_SHOW(show_driver_name, csdev, buf)
{
	ssize_t retval;
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	retval = snprintf(buf, PAGE_SIZE, "%s\n", dev->driver->driver_name);

	return retval;
}

static COMEDI_DECLARE_ATTR_SHOW(show_board_name, csdev, buf)
{
	ssize_t retval;
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	retval = snprintf(buf, PAGE_SIZE, "%s\n", dev->board_name);

	return retval;
}

static COMEDI_DECLARE_ATTR_SHOW(show_bydrivername_index, csdev, buf)
{
	ssize_t retval;
	int i;
	int result = 0;
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	for (i = 0; i < COMEDI_NUM_BOARD_MINORS; i++) {
		comedi_device *iter_dev = comedi_get_device_by_minor(i);
		if (iter_dev == NULL)
			continue;
		if (iter_dev == dev)
			break;
		if (iter_dev->driver == dev->driver)
			++result;
	}
	retval = snprintf(buf, PAGE_SIZE, "%d\n", result);

	return retval;
}

static COMEDI_DECLARE_ATTR_SHOW(show_byboardname_index, csdev, buf)
{
	ssize_t retval;
	int i;
	int result = 0;
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	for (i = 0; i < COMEDI_NUM_BOARD_MINORS; i++) {
		comedi_device *iter_dev = comedi_get_device_by_minor(i);
		if (iter_dev == NULL)
			continue;
		if (iter_dev == dev)
			break;
		if (strncmp(iter_dev->board_name, dev->board_name, PAGE_SIZE) == 0)
			++result;
	}
	retval = snprintf(buf, PAGE_SIZE, "%d\n", result);

	return retval;
}

static COMEDI_DECLARE_ATTR_SHOW(show_max_read_buffer_kb, csdev, buf)
{
	ssize_t retval;
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	unsigned max_buffer_size_kb = 0;
	comedi_subdevice * const read_subdevice = comedi_read_subdevice(dev, minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	mutex_lock(&dev->mutex);
	if(read_subdevice &&
		(read_subdevice->subdev_flags & SDF_CMD_READ) &&
		read_subdevice->async)
	{
		max_buffer_size_kb = read_subdevice->async->max_bufsize / bytes_per_kibi;
	}
	retval =  snprintf(buf, PAGE_SIZE, "%i\n", max_buffer_size_kb);
	mutex_unlock(&dev->mutex);

	return retval;
}

static COMEDI_DECLARE_ATTR_STORE(store_max_read_buffer_kb, csdev, buf, count)
{
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	unsigned long new_max_size_kb;
	uint64_t new_max_size;
	comedi_subdevice * const read_subdevice = comedi_read_subdevice(dev, minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	if(kstrtoul(buf, 10, &new_max_size_kb))
	{
		return -EINVAL;
	}
	if(new_max_size_kb != (uint32_t)new_max_size_kb) return -EINVAL;
	new_max_size = ((uint64_t)new_max_size_kb) * bytes_per_kibi;
	if(new_max_size != (uint32_t)new_max_size) return -EINVAL;

	mutex_lock(&dev->mutex);
	if(read_subdevice == NULL ||
		(read_subdevice->subdev_flags & SDF_CMD_READ) == 0 ||
		read_subdevice->async == NULL)
	{
		mutex_unlock(&dev->mutex);
		return -EINVAL;
	}
	read_subdevice->async->max_bufsize = new_max_size;
	mutex_unlock(&dev->mutex);

	return count;
}

static COMEDI_DECLARE_ATTR_SHOW(show_read_buffer_kb, csdev, buf)
{
	ssize_t retval;
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	unsigned buffer_size_kb = 0;
	comedi_subdevice * const read_subdevice = comedi_read_subdevice(dev, minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	mutex_lock(&dev->mutex);
	if(read_subdevice &&
		(read_subdevice->subdev_flags & SDF_CMD_READ) &&
		read_subdevice->async)
	{
		buffer_size_kb = read_subdevice->async->prealloc_bufsz / bytes_per_kibi;
	}
	retval =  snprintf(buf, PAGE_SIZE, "%i\n", buffer_size_kb);
	mutex_unlock(&dev->mutex);

	return retval;
}

static COMEDI_DECLARE_ATTR_STORE(store_read_buffer_kb, csdev, buf, count)
{
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	unsigned long new_size_kb;
	uint64_t new_size;
	int retval;
	comedi_subdevice * const read_subdevice = comedi_read_subdevice(dev, minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	if(kstrtoul(buf, 10, &new_size_kb))
	{
		return -EINVAL;
	}
	if(new_size_kb != (uint32_t)new_size_kb) return -EINVAL;
	new_size = ((uint64_t)new_size_kb) * bytes_per_kibi;
	if(new_size != (uint32_t)new_size) return -EINVAL;

	mutex_lock(&dev->mutex);
	if(read_subdevice == NULL ||
		(read_subdevice->subdev_flags & SDF_CMD_READ) == 0 ||
		read_subdevice->async == NULL)
	{
		mutex_unlock(&dev->mutex);
		return -EINVAL;
	}
	retval = resize_async_buffer(dev, read_subdevice,
		read_subdevice->async, new_size);
	mutex_unlock(&dev->mutex);

	if(retval < 0) return retval;
	return count;
}

static COMEDI_DECLARE_ATTR_SHOW(show_max_write_buffer_kb, csdev, buf)
{
	ssize_t retval;
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	unsigned max_buffer_size_kb = 0;
	comedi_subdevice * const write_subdevice = comedi_write_subdevice(dev, minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	mutex_lock(&dev->mutex);
	if(write_subdevice &&
		(write_subdevice->subdev_flags & SDF_CMD_WRITE) &&
		write_subdevice->async)
	{
		max_buffer_size_kb = write_subdevice->async->max_bufsize / bytes_per_kibi;
	}
	retval =  snprintf(buf, PAGE_SIZE, "%i\n", max_buffer_size_kb);
	mutex_unlock(&dev->mutex);

	return retval;
}

static COMEDI_DECLARE_ATTR_STORE(store_max_write_buffer_kb, csdev, buf, count)
{
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	unsigned long new_max_size_kb;
	uint64_t new_max_size;
	comedi_subdevice * const write_subdevice = comedi_write_subdevice(dev, minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	if(kstrtoul(buf, 10, &new_max_size_kb))
	{
		return -EINVAL;
	}
	if(new_max_size_kb != (uint32_t)new_max_size_kb) return -EINVAL;
	new_max_size = ((uint64_t)new_max_size_kb) * bytes_per_kibi;
	if(new_max_size != (uint32_t)new_max_size) return -EINVAL;

	mutex_lock(&dev->mutex);
	if(write_subdevice == NULL ||
		(write_subdevice->subdev_flags & SDF_CMD_WRITE) == 0 ||
		write_subdevice->async == NULL)
	{
		mutex_unlock(&dev->mutex);
		return -EINVAL;
	}
	write_subdevice->async->max_bufsize = new_max_size;
	mutex_unlock(&dev->mutex);

	return count;
}

static COMEDI_DECLARE_ATTR_SHOW(show_write_buffer_kb, csdev, buf)
{
	ssize_t retval;
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	unsigned buffer_size_kb = 0;
	comedi_subdevice * const write_subdevice = comedi_write_subdevice(dev, minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	mutex_lock(&dev->mutex);
	if(write_subdevice &&
		(write_subdevice->subdev_flags & SDF_CMD_WRITE) &&
		write_subdevice->async)
	{
		buffer_size_kb = write_subdevice->async->prealloc_bufsz / bytes_per_kibi;
	}
	retval =  snprintf(buf, PAGE_SIZE, "%i\n", buffer_size_kb);
	mutex_unlock(&dev->mutex);

	return retval;
}

static COMEDI_DECLARE_ATTR_STORE(store_write_buffer_kb, csdev, buf, count)
{
	const unsigned minor = (unsigned long)COMEDI_DEV_GET_DRVDATA(csdev);
	comedi_device *dev = comedi_get_device_by_minor(minor);
	unsigned long new_size_kb;
	uint64_t new_size;
	int retval;
	comedi_subdevice * const write_subdevice = comedi_write_subdevice(dev, minor);

	if (dev == NULL) {
		return -ENODEV;
	}

	if(kstrtoul(buf, 10, &new_size_kb))
	{
		return -EINVAL;
	}
	if(new_size_kb != (uint32_t)new_size_kb) return -EINVAL;
	new_size = ((uint64_t)new_size_kb) * bytes_per_kibi;
	if(new_size != (uint32_t)new_size) return -EINVAL;

	mutex_lock(&dev->mutex);
	if(write_subdevice == NULL ||
		(write_subdevice->subdev_flags & SDF_CMD_WRITE) == 0 ||
		write_subdevice->async == NULL)
	{
		mutex_unlock(&dev->mutex);
		return -EINVAL;
	}
	retval = resize_async_buffer(dev, write_subdevice,
		write_subdevice->async, new_size);
	mutex_unlock(&dev->mutex);

	if(retval < 0) return retval;
	return count;
}
