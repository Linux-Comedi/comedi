/*
 * linux/uaccess.h compatibility header
 */

#ifndef __COMPAT_LINUX_UACCESS_H_
#define __COMPAT_LINUX_UACCESS_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#include <asm/uaccess.h>
#else
#include_next <linux/uaccess.h>
#endif

#endif /* __COMPAT_LINUX_UACCESS_H_ */
