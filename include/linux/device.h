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
	dev_info_t name;
};
#else
#include_next <linux/device.h>
#endif

#endif // __COMPAT_LINUX_DEVICE_H_

