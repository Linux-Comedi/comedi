/*
 * linux/fs.h compatibility header
 */

#ifndef __COMPAT_LINUX_SCHED_H_
#define __COMPAT_LINUX_SCHED_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,0)
#define signal_pending(x)	(((x)->signal) & (~(x)->blocked))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,20) /* not sure exactly when need_resched() was added */
static inline int need_resched(void)
{
	return current->need_resched;
}
#endif

#include_next <linux/sched.h>

#endif

