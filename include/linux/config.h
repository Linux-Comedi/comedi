#ifndef __MULTI_CONFIG_H_
#define __MULTI_CONFIG_H_

#include <linux/version.h>

#include <config.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 15)
#include_next <linux/config.h>
#endif

#endif
