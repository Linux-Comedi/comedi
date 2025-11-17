/*
 * linux/module.h compatibility header
 */

#ifndef __COMPAT_LINUX_MODULE_H_
#define __COMPAT_LINUX_MODULE_H_

#include_next <linux/module.h>

#ifndef MODULE_VERSION
/* Early 2.6.x kernels are missing the MODULE_VERSION() macro. */
#define MODULE_VERSION(version)
#endif

#endif
