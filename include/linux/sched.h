/*
 * linux/sched.h compatibility header
 */

#ifndef __COMPAT_LINUX_SCHED_H_
#define __COMPAT_LINUX_SCHED_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,0)
#define signal_pending(x)	(((x)->signal) & (~(x)->blocked))
#endif

#include_next <linux/sched.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,20)
static inline int need_resched(void)
{
	return current->need_resched;
}
#endif

#endif

