/*
 * linux/fs.h compatibility header
 */

#ifndef __COMPAT_LINUX_POLL_H_
#define __COMPAT_LINUX_POLL_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020200
#define signal_pending(x)	(((x)->signal) & (~(x)->blocked))
#endif

#include_next <linux/sched.h>

#endif

