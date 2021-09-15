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

#ifndef HAVE_COMPAT_IOCTL	/* defined in <linux/fs.h> 2.6.11 onwards */

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
	rc = comedi_compat_ioctl(file, cmd, arg);
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

#endif	/* ifndef HAVE_COMPAT_IOCTL */

#endif	/* CONFIG_COMPAT */
