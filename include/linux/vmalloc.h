/*
 * linux/vmalloc.h compatibility header
 */

#ifndef __COMPAT_LINUX_VMALLOC_H_
#define __COMPAT_LINUX_VMALLOC_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020300
#define vmalloc_32(x) vmalloc((x))
#endif

#if LINUX_VERSION_CODE < 0x020200

#else
#include_next <linux/vmalloc.h>
#endif

#endif

