
#ifndef __COMPAT_LINUX_KTIME_H
#define __COMPAT_LINUX_KTIME_H

#include <linux/version.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,16)
#include_next <linux/ktime.h>
#endif

/*
 * do_gettimeofday was removed in kernel version 4.20.
 * Add a temporary version here for now until we fix the code that calls it.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,20,0)
static inline void do_gettimeofday(struct timeval *tv)
{
	struct timespec64 now;

	ktime_get_real_ts64(&now);
	tv->tv_sec = now.tv_sec;
	tv->tv_usec = now.tv_nsec / 1000;
}
#endif

#endif
