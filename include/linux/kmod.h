/*
 * linux/kmod.h compatibility header
 */

#ifndef __COMPAT_LINUX_KMOD_H_
#define __COMPAT_LINUX_KMOD_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,0)

#else
#include_next <linux/kmod.h>
#endif

#endif

