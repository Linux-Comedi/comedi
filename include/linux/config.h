#ifndef __MULTI_CONFIG_H_
#define __MULTI_CONFIG_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 19)

#include <config.h>
#include_next <linux/config.h>

#endif

#endif
