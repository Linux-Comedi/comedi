/*
 * linux/device.h compatibility header
 */
/*
    Copyright (C) 2004-2006 Frank Mori Hess <fmhess@users.sourceforge.net>
    Copyright (C) 2005-2006 Ian Abbott

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
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#ifndef __COMPAT_LINUX_DEVICE_H_
#define __COMPAT_LINUX_DEVICE_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#include <pcmcia/cs_types.h>

struct device_driver
{
	char *name;
};

struct class;
struct class_device;
struct device;

static inline struct class *class_create(struct module *owner, char *name)
{
	return NULL;
};

static inline void class_destroy(struct class *cs)
{};

static inline struct class_device *COMEDI_CLASS_DEVICE_CREATE(struct class *cls,
	struct class_device *parent, dev_t devt, struct device *device,
	char *fmt, ...)
{
	return NULL;
};

static inline void class_device_destroy(struct class *cs, dev_t dev)
{};

#else

#include_next <linux/device.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

#define class_create(owner, name) \
	(struct class *)class_simple_create(owner, name)
#define class_destroy(cs) \
	class_simple_destroy((struct class_simple *)(cs))
#define COMEDI_CLASS_DEVICE_CREATE(cs, parent, dev, device, fmt...) \
	class_simple_device_add((struct class_simple *)(cs), \
		dev, device, fmt)
#define class_device_destroy(cs, dev) \
	class_simple_device_remove(dev)

#else

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)

#define COMEDI_CLASS_DEVICE_CREATE(cs, parent, dev, device, fmt...) \
	class_device_create(cs, dev, device, fmt)

#else

#define COMEDI_CLASS_DEVICE_CREATE(cs, parent, dev, device, fmt...) \
	class_device_create(cs, parent, dev, device, fmt)

#endif	// LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)

#endif	// LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

#endif	// LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)

#endif // __COMPAT_LINUX_DEVICE_H_

