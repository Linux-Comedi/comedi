/*
 * linux/slab.h compatibility header
 */

#ifndef __COMPAT_LINUX_SLAB_H_
#define __COMPAT_LINUX_SLAB_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020200
#include <linux/malloc.h>
#else
#include_next <linux/slab.h>
#endif

#endif

