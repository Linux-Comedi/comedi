/*
 * linux/init.h compatibility header
 */

#ifndef __COMPAT_LINUX_INIT_H_
#define __COMPAT_LINUX_INIT_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020400
#define __devinitdata __initdata
#define __exit
#endif
#if LINUX_VERSION_CODE < 0x020200
#define __init
#else
#include_next <linux/init.h>
#endif

#endif

