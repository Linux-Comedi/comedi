/*
    comedi/comedi_compat32.c
    32-bit ioctl compatibility for 64-bit comedi kernel module.

    Author: Ian Abbott, MEV Ltd. <abbotti@mev.co.uk>
    Copyright (C) 2007 MEV Ltd. <http://www.mev.co.uk/>

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2007 David A. Schleef <ds@schleef.org>

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
#include <linux/version.h>
#include <linux/comedi.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
#include <linux/smp_lock.h>
#endif
#include <linux/uaccess.h>

#include "comedi_compat32.h"

#ifdef CONFIG_COMPAT

#ifndef HAVE_COMPAT_IOCTL
#include <linux/ioctl32.h>	/* for (un)register_ioctl32_conversion */
#endif

#define COMEDI32_CHANINFO _IOR(CIO,3,comedi32_chaninfo)
#define COMEDI32_RANGEINFO _IOR(CIO,8,comedi32_rangeinfo)
/* N.B. COMEDI32_CMD and COMEDI_CMD ought to use _IOWR, not _IOR.
 * It's too late to change it now, but it only affects the command number. */
#define COMEDI32_CMD _IOR(CIO,9,comedi32_cmd)
/* N.B. COMEDI32_CMDTEST and COMEDI_CMDTEST ought to use _IOWR, not _IOR.
 * It's too late to change it now, but it only affects the command number. */
#define COMEDI32_CMDTEST _IOR(CIO,10,comedi32_cmd)
#define COMEDI32_INSNLIST _IOR(CIO,11,comedi32_insnlist)
#define COMEDI32_INSN _IOR(CIO,12,comedi32_insn)

typedef struct comedi32_chaninfo_struct {
	unsigned int subdev;
	compat_uptr_t maxdata_list;	/* 32-bit 'lsampl_t *' */
	compat_uptr_t flaglist;		/* 32-bit 'unsigned int *' */
	compat_uptr_t rangelist;	/* 32-bit 'unsigned int *' */
	unsigned int unused[4];
} comedi32_chaninfo;

typedef struct comedi32_rangeinfo_struct {
	unsigned int range_type;
	compat_uptr_t range_ptr;	/* 32-bit 'void *' */
} comedi32_rangeinfo;

typedef struct comedi32_cmd_struct {
	unsigned int subdev;
	unsigned int flags;
	unsigned int start_src;
	unsigned int start_arg;
	unsigned int scan_begin_src;
	unsigned int scan_begin_arg;
	unsigned int convert_src;
	unsigned int convert_arg;
	unsigned int scan_end_src;
	unsigned int scan_end_arg;
	unsigned int stop_src;
	unsigned int stop_arg;
	compat_uptr_t chanlist;		/* 32-bit 'unsigned int *' */
	unsigned int chanlist_len;
	compat_uptr_t data;		/* 32-bit 'sampl_t *' */
	unsigned int data_len;
} comedi32_cmd;

typedef struct comedi32_insn_struct {
	unsigned int insn;
	unsigned int n;
	compat_uptr_t data;		/* 32-bit 'lsampl_t *' */
	unsigned int subdev;
	unsigned int chanspec;
	unsigned int unused[3];
} comedi32_insn;

typedef struct comedi32_insnlist_struct {
	unsigned int n_insns;
	compat_uptr_t insns;		/* 32-bit 'comedi_insn *' */
} comedi32_insnlist;

/* Handle translated ioctl. */
static int translated_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	if (!file->f_op) {
		return -ENOTTY;
	}
#ifdef HAVE_UNLOCKED_IOCTL
	if (file->f_op->unlocked_ioctl) {
		int rc = (int)(*file->f_op->unlocked_ioctl)(file, cmd, arg);
		if (rc == -ENOIOCTLCMD) {
			rc = -ENOTTY;
		}
		return rc;
	}
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
	if (file->f_op->ioctl) {
		int rc;
		lock_kernel();
		rc = (*file->f_op->ioctl)(file->f_dentry->d_inode,
				file, cmd, arg);
		unlock_kernel();
		return rc;
	}
#endif
	return -ENOTTY;
}

/* Handle 32-bit COMEDI_CHANINFO ioctl. */
static int compat_chaninfo(struct file *file, unsigned long arg)
{
	comedi_chaninfo __user *chaninfo;
	comedi32_chaninfo __user *chaninfo32;
	int err;
	union {
		unsigned int uint;
		compat_uptr_t uptr;
	} temp;

	chaninfo32 = compat_ptr(arg);
	chaninfo = compat_alloc_user_space(sizeof(*chaninfo));

	/* Copy chaninfo structure.  Ignore unused members. */
	if (!access_ok(chaninfo32, sizeof(*chaninfo32))
			|| !access_ok(chaninfo, sizeof(*chaninfo))) {
		return -EFAULT;
	}
	err = 0;
	err |= __get_user(temp.uint, &chaninfo32->subdev);
	err |= __put_user(temp.uint, &chaninfo->subdev);
	err |= __get_user(temp.uptr, &chaninfo32->maxdata_list);
	err |= __put_user(compat_ptr(temp.uptr), &chaninfo->maxdata_list);
	err |= __get_user(temp.uptr, &chaninfo32->flaglist);
	err |= __put_user(compat_ptr(temp.uptr), &chaninfo->flaglist);
	err |= __get_user(temp.uptr, &chaninfo32->rangelist);
	err |= __put_user(compat_ptr(temp.uptr), &chaninfo->rangelist);
	if (err) {
		return -EFAULT;
	}

	return translated_ioctl(file, COMEDI_CHANINFO, (unsigned long)chaninfo);
}

/* Handle 32-bit COMEDI_RANGEINFO ioctl. */
static int compat_rangeinfo(struct file *file, unsigned long arg)
{
	comedi_rangeinfo __user *rangeinfo;
	comedi32_rangeinfo __user *rangeinfo32;
	int err;
	union {
		unsigned int uint;
		compat_uptr_t uptr;
	} temp;

	rangeinfo32 = compat_ptr(arg);
	rangeinfo = compat_alloc_user_space(sizeof(*rangeinfo));

	/* Copy rangeinfo structure. */
	if (!access_ok(rangeinfo32, sizeof(*rangeinfo32))
			|| !access_ok(rangeinfo, sizeof(*rangeinfo))) {
		return -EFAULT;
	}
	err = 0;
	err |= __get_user(temp.uint, &rangeinfo32->range_type);
	err |= __put_user(temp.uint, &rangeinfo->range_type);
	err |= __get_user(temp.uptr, &rangeinfo32->range_ptr);
	err |= __put_user(compat_ptr(temp.uptr), &rangeinfo->range_ptr);
	if (err) {
		return -EFAULT;
	}

	return translated_ioctl(file, COMEDI_RANGEINFO,
			(unsigned long)rangeinfo);
}

/* Copy 32-bit cmd structure to native cmd structure. */
static int get_compat_cmd(comedi_cmd __user *cmd,
		comedi32_cmd __user *cmd32)
{
	int err;
	union {
		unsigned int uint;
		compat_uptr_t uptr;
	} temp;

	/* Copy cmd structure. */
	if (!access_ok(cmd32, sizeof(*cmd32))
			|| !access_ok(cmd, sizeof(*cmd))) {
		return -EFAULT;
	}
	err = 0;
	err |= __get_user(temp.uint, &cmd32->subdev);
	err |= __put_user(temp.uint, &cmd->subdev);
	err |= __get_user(temp.uint, &cmd32->flags);
	err |= __put_user(temp.uint, &cmd->flags);
	err |= __get_user(temp.uint, &cmd32->start_src);
	err |= __put_user(temp.uint, &cmd->start_src);
	err |= __get_user(temp.uint, &cmd32->start_arg);
	err |= __put_user(temp.uint, &cmd->start_arg);
	err |= __get_user(temp.uint, &cmd32->scan_begin_src);
	err |= __put_user(temp.uint, &cmd->scan_begin_src);
	err |= __get_user(temp.uint, &cmd32->scan_begin_arg);
	err |= __put_user(temp.uint, &cmd->scan_begin_arg);
	err |= __get_user(temp.uint, &cmd32->convert_src);
	err |= __put_user(temp.uint, &cmd->convert_src);
	err |= __get_user(temp.uint, &cmd32->convert_arg);
	err |= __put_user(temp.uint, &cmd->convert_arg);
	err |= __get_user(temp.uint, &cmd32->scan_end_src);
	err |= __put_user(temp.uint, &cmd->scan_end_src);
	err |= __get_user(temp.uint, &cmd32->scan_end_arg);
	err |= __put_user(temp.uint, &cmd->scan_end_arg);
	err |= __get_user(temp.uint, &cmd32->stop_src);
	err |= __put_user(temp.uint, &cmd->stop_src);
	err |= __get_user(temp.uint, &cmd32->stop_arg);
	err |= __put_user(temp.uint, &cmd->stop_arg);
	err |= __get_user(temp.uptr, &cmd32->chanlist);
	err |= __put_user(compat_ptr(temp.uptr), &cmd->chanlist);
	err |= __get_user(temp.uint, &cmd32->chanlist_len);
	err |= __put_user(temp.uint, &cmd->chanlist_len);
	err |= __get_user(temp.uptr, &cmd32->data);
	err |= __put_user(compat_ptr(temp.uptr), &cmd->data);
	err |= __get_user(temp.uint, &cmd32->data_len);
	err |= __put_user(temp.uint, &cmd->data_len);
	return err ? -EFAULT : 0;
}

/* Copy native cmd structure to 32-bit cmd structure. */
static int put_compat_cmd(comedi32_cmd __user *cmd32, comedi_cmd __user *cmd)
{
	int err;
	unsigned int temp;

	/* Copy back most of cmd structure. */
	/* Assume the pointer values are already valid. */
	/* (Could use ptr_to_compat() to set them, but that wasn't implemented
	 * until kernel version 2.6.11.) */
	if (!access_ok(cmd, sizeof(*cmd))
			|| !access_ok(cmd32, sizeof(*cmd32))) {
		return -EFAULT;
	}
	err = 0;
	err |= __get_user(temp, &cmd->subdev);
	err |= __put_user(temp, &cmd32->subdev);
	err |= __get_user(temp, &cmd->flags);
	err |= __put_user(temp, &cmd32->flags);
	err |= __get_user(temp, &cmd->start_src);
	err |= __put_user(temp, &cmd32->start_src);
	err |= __get_user(temp, &cmd->start_arg);
	err |= __put_user(temp, &cmd32->start_arg);
	err |= __get_user(temp, &cmd->scan_begin_src);
	err |= __put_user(temp, &cmd32->scan_begin_src);
	err |= __get_user(temp, &cmd->scan_begin_arg);
	err |= __put_user(temp, &cmd32->scan_begin_arg);
	err |= __get_user(temp, &cmd->convert_src);
	err |= __put_user(temp, &cmd32->convert_src);
	err |= __get_user(temp, &cmd->convert_arg);
	err |= __put_user(temp, &cmd32->convert_arg);
	err |= __get_user(temp, &cmd->scan_end_src);
	err |= __put_user(temp, &cmd32->scan_end_src);
	err |= __get_user(temp, &cmd->scan_end_arg);
	err |= __put_user(temp, &cmd32->scan_end_arg);
	err |= __get_user(temp, &cmd->stop_src);
	err |= __put_user(temp, &cmd32->stop_src);
	err |= __get_user(temp, &cmd->stop_arg);
	err |= __put_user(temp, &cmd32->stop_arg);
	/* Assume chanlist pointer is unchanged. */
	err |= __get_user(temp, &cmd->chanlist_len);
	err |= __put_user(temp, &cmd32->chanlist_len);
	/* Assume data pointer is unchanged. */
	err |= __get_user(temp, &cmd->data_len);
	err |= __put_user(temp, &cmd32->data_len);
	return err ? -EFAULT : 0;
}

/* Handle 32-bit COMEDI_CMD ioctl. */
static int compat_cmd(struct file *file, unsigned long arg)
{
	comedi_cmd __user *cmd;
	comedi32_cmd __user *cmd32;
	int rc, err;

	cmd32 = compat_ptr(arg);
	cmd = compat_alloc_user_space(sizeof(*cmd));

	rc = get_compat_cmd(cmd, cmd32);
	if (rc) {
		return rc;
	}

	rc = translated_ioctl(file, COMEDI_CMD, (unsigned long)cmd);
	if (rc == -EAGAIN) {
		/* Special case: copy cmd back to user. */
		err = put_compat_cmd(cmd32, cmd);
		if (err) {
			rc = err;
		}
	}
	return rc;
}

/* Handle 32-bit COMEDI_CMDTEST ioctl. */
static int compat_cmdtest(struct file *file, unsigned long arg)
{
	comedi_cmd __user *cmd;
	comedi32_cmd __user *cmd32;
	int rc, err;

	cmd32 = compat_ptr(arg);
	cmd = compat_alloc_user_space(sizeof(*cmd));

	rc = get_compat_cmd(cmd, cmd32);
	if (rc) {
		return rc;
	}

	rc = translated_ioctl(file, COMEDI_CMDTEST, (unsigned long)cmd);
	if (rc < 0) {
		return rc;
	}

	err = put_compat_cmd(cmd32, cmd);
	if (err) {
		rc = err;
	}
	return rc;
}

/* Copy 32-bit insn structure to native insn structure. */
static int get_compat_insn(comedi_insn __user *insn,
		comedi32_insn __user *insn32)
{
	int err;
	union {
		unsigned int uint;
		compat_uptr_t uptr;
	} temp;

	/* Copy insn structure.  Ignore the unused members. */
	err = 0;
	if (!access_ok(insn32, sizeof(*insn32))
			|| !access_ok(insn, sizeof(*insn))) {
		return -EFAULT;
	}
	err |= __get_user(temp.uint, &insn32->insn);
	err |= __put_user(temp.uint, &insn->insn);
	err |= __get_user(temp.uint, &insn32->n);
	err |= __put_user(temp.uint, &insn->n);
	err |= __get_user(temp.uptr, &insn32->data);
	err |= __put_user(compat_ptr(temp.uptr), &insn->data);
	err |= __get_user(temp.uint, &insn32->subdev);
	err |= __put_user(temp.uint, &insn->subdev);
	err |= __get_user(temp.uint, &insn32->chanspec);
	err |= __put_user(temp.uint, &insn->chanspec);
	return err ? -EFAULT : 0;
}

/* Handle 32-bit COMEDI_INSNLIST ioctl. */
static int compat_insnlist(struct file *file, unsigned long arg)
{
	struct combined_insnlist {
		comedi_insnlist insnlist;
		comedi_insn insn[1];
	} __user *s;
	comedi32_insnlist __user *insnlist32;
	comedi32_insn __user *insn32;
	compat_uptr_t uptr;
	unsigned int n_insns, n;
	int err, rc;

	insnlist32 = compat_ptr(arg);

	/* Get 32-bit insnlist structure.  */
	if (!access_ok(insnlist32, sizeof(*insnlist32))) {
		return -EFAULT;
	}
	err = 0;
	err |= __get_user(n_insns, &insnlist32->n_insns);
	err |= __get_user(uptr, &insnlist32->insns);
	insn32 = compat_ptr(uptr);
	if (err) {
		return -EFAULT;
	}

	/* Allocate user memory to copy insnlist and insns into. */
	s = compat_alloc_user_space(offsetof(struct combined_insnlist,
				insn[n_insns]));

	/* Set native insnlist structure. */
	if (!access_ok(&s->insnlist, sizeof(s->insnlist))) {
		return -EFAULT;
	}
	err |= __put_user(n_insns, &s->insnlist.n_insns);
	err |= __put_user(&s->insn[0], &s->insnlist.insns);
	if (err) {
		return -EFAULT;
	}

	/* Copy insn structures. */
	for (n = 0; n < n_insns; n++) {
		rc = get_compat_insn(&s->insn[n], &insn32[n]);
		if (rc) {
			return rc;
		}
	}

	return translated_ioctl(file, COMEDI_INSNLIST,
			(unsigned long)&s->insnlist);
}

/* Handle 32-bit COMEDI_INSN ioctl. */
static int compat_insn(struct file *file, unsigned long arg)
{
	comedi_insn __user *insn;
	comedi32_insn __user *insn32;
	int rc;

	insn32 = compat_ptr(arg);
	insn = compat_alloc_user_space(sizeof(*insn));

	rc = get_compat_insn(insn, insn32);
	if (rc) {
		return rc;
	}

	return translated_ioctl(file, COMEDI_INSN, (unsigned long)insn);
}

/* Process untranslated ioctl. */
/* Returns -ENOIOCTLCMD for unrecognised ioctl codes. */
static inline int raw_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc;

	switch (cmd) {
	case COMEDI_DEVCONFIG:
	case COMEDI_DEVINFO:
	case COMEDI_SUBDINFO:
	case COMEDI_BUFCONFIG:
	case COMEDI_BUFINFO:
		/* Just need to translate the pointer argument. */
		arg = (unsigned long)compat_ptr(arg);
		rc = translated_ioctl(file, cmd, arg);
		break;
	case COMEDI_LOCK:
	case COMEDI_UNLOCK:
	case COMEDI_CANCEL:
	case COMEDI_POLL:
	case COMEDI_SETRSUBD:
	case COMEDI_SETWSUBD:
		/* No translation needed. */
		rc = translated_ioctl(file, cmd, arg);
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

#ifdef HAVE_COMPAT_IOCTL	/* defined in <linux/fs.h> 2.6.11 onwards */

/* compat_ioctl file operation. */
/* Returns -ENOIOCTLCMD for unrecognised ioctl codes. */
long comedi_compat_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	return raw_ioctl(file, cmd, arg);
}

#else /* HAVE_COMPAT_IOCTL */

/*
 * Brain-dead ioctl compatibility for 2.6.10 and earlier.
 *
 * It's brain-dead because cmd numbers need to be unique system-wide!
 * The comedi driver could end up attempting to execute ioctls for non-Comedi
 * devices because it registered the system-wide cmd code first.  Similarly,
 * another driver could end up attempting to execute ioctls for a Comedi
 * device because it registered the cmd code first.  Chaos ensues.
 */

/* Handler for all 32-bit ioctl codes registered by this driver. */
static int mapped_ioctl(unsigned int fd, unsigned int cmd, unsigned long arg,
		struct file *file)
{
	int rc;

	/* Make sure we are dealing with a Comedi device. */
	if (imajor(file->f_dentry->d_inode) != COMEDI_MAJOR) {
		return -ENOTTY;
	}
	rc = raw_ioctl(file, cmd, arg);
	/* Do not return -ENOIOCTLCMD. */
	if (rc == -ENOIOCTLCMD) {
		rc = -ENOTTY;
	}
	return rc;
}

struct ioctl32_map {
	unsigned int cmd;
	int (*handler)(unsigned int, unsigned int, unsigned long,
			struct file *);
	int registered;
};

static struct ioctl32_map comedi_ioctl32_map[] = {
	{ COMEDI_DEVCONFIG, mapped_ioctl, 0 },
	{ COMEDI_DEVINFO, mapped_ioctl, 0 },
	{ COMEDI_SUBDINFO, mapped_ioctl, 0 },
	{ COMEDI_BUFCONFIG, mapped_ioctl, 0 },
	{ COMEDI_BUFINFO, mapped_ioctl, 0 },
	{ COMEDI_LOCK, mapped_ioctl, 0 },
	{ COMEDI_UNLOCK, mapped_ioctl, 0 },
	{ COMEDI_CANCEL, mapped_ioctl, 0 },
	{ COMEDI_POLL, mapped_ioctl, 0 },
	{ COMEDI_SETRSUBD, mapped_ioctl, 0 },
	{ COMEDI_SETRWUBD, mapped_ioctl, 0 },
	{ COMEDI32_CHANINFO, mapped_ioctl, 0 },
	{ COMEDI32_RANGEINFO, mapped_ioctl, 0 },
	{ COMEDI32_CMD, mapped_ioctl, 0 },
	{ COMEDI32_CMDTEST, mapped_ioctl, 0 },
	{ COMEDI32_INSNLIST, mapped_ioctl, 0 },
	{ COMEDI32_INSN, mapped_ioctl, 0 },
};

#define NUM_IOCTL32_MAPS ARRAY_SIZE(comedi_ioctl32_map)

/* Register system-wide 32-bit ioctl handlers. */
void comedi_register_ioctl32(void)
{
	int n, rc;

	for (n = 0; n < NUM_IOCTL32_MAPS; n++) {
		rc = register_ioctl32_conversion(comedi_ioctl32_map[n].cmd,
				comedi_ioctl32_map[n].handler);
		if (rc) {
			printk(KERN_WARNING
					"comedi: failed to register 32-bit "
					"compatible ioctl handler for 0x%X - "
					"expect bad things to happen!\n",
					comedi_ioctl32_map[n].cmd);
		}
		comedi_ioctl32_map[n].registered = !rc;
	}
}

/* Unregister system-wide 32-bit ioctl translations. */
void comedi_unregister_ioctl32(void)
{
	int n, rc;

	for (n = 0; n < NUM_IOCTL32_MAPS; n++) {
		if (comedi_ioctl32_map[n].registered) {
			rc = unregister_ioctl32_conversion(
					comedi_ioctl32_map[n].cmd,
					comedi_ioctl32_map[n].handler);
			if (rc) {
				printk(KERN_ERR
					"comedi: failed to unregister 32-bit "
					"compatible ioctl handler for 0x%X - "
					"expect kernel Oops!\n",
					comedi_ioctl32_map[n].cmd);
			} else {
				comedi_ioctl32_map[n].registered = 0;
			}
		}
	}
}

#endif	/* HAVE_COMPAT_IOCTL */

#endif	/* CONFIG_COMPAT */
