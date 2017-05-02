/*
 * linux/sched/signal.h compatibility header
 */

#ifndef __COMPAT_LINUX_SCHED_SIGNAL_H_
#define __COMPAT_LINUX_SCHED_SIGNAL_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
#include <linux/sched.h>
#else
#include_next <linux/sched/signal.h>
#endif

#endif /* __COMPAT_LINUX_SCHED_SIGNAL_H_ */
