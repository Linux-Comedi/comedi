/*
 * linux/kmod.h compatibility header
 */

#ifndef __COMPAT_LINUX_KMOD_H_
#define __COMPAT_LINUX_KMOD_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020200

#else
#include_next <linux/kmod.h>
#endif

#endif

