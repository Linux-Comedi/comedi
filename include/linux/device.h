/*
 * linux/device.h compatibility header
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

typedef void class_simple;

struct device
{
	void *dummy;
};

static inline struct class_simple *class_simple_create(struct module *owner, char *name)
{
	return NULL;
};

static inline void class_simple_destroy(struct class_simple *cs)
{};

static inline struct class_device *class_simple_device_add(struct class_simple *cs, 
	dev_t dev, struct device *device, const char *fmt, ...)
{
	return NULL;
};

static inline void class_simple_device_remove(dev_t dev)
{};

#else
#include_next <linux/device.h>
#endif

#endif // __COMPAT_LINUX_DEVICE_H_

