/*
 * linux/vmalloc.h compatibility header
 */

#ifndef __COMPAT_LINUX_VMALLOC_H_
#define __COMPAT_LINUX_VMALLOC_H_

#include <linux/version.h>


#if LINUX_VERSION_CODE < 0x020200

#else
#include_next <linux/vmalloc.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 2, 18)
extern inline void * vmalloc_32(unsigned long size)
{
        return vmalloc(size);
}
#endif	// 2.2.18

#endif

#endif

