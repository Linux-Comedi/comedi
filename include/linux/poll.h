/*
 * linux/poll.h compatibility header
 */

#ifndef __COMPAT_LINUX_POLL_H_
#define __COMPAT_LINUX_POLL_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,0)

#else
#include_next <linux/poll.h>
#endif

#endif

