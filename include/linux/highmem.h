
/*
 * linux/highmem.h compatibility header
 */

#ifndef __COMPAT_LINUX_DEVFS_FS_KERNEL_H_
#define __COMPAT_LINUX_DEVFS_FS_KERNEL_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 4, 0)
#else
#include_next <linux/highmem.h>
#endif

#endif

