/*
 * linux/fisapnp.h compatibility header
 */

#ifndef __COMPAT_LINUX_ISAPNP_H_
#define __COMPAT_LINUX_ISAPNP_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,3,0)
#include_next <linux/isapnp.h>
#else
#include <linux/pci.h>
#endif

#endif

