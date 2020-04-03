
#ifndef __COMPAT_LINUX_TIME_H
#define __COMPAT_LINUX_TIME_H

#include <linux/version.h>

#include_next <linux/time.h>

#ifndef MSEC_PER_SEC
#define MSEC_PER_SEC	1000L
#endif

#ifndef USEC_PER_MSEC
#define USEC_PER_MSEC	1000L
#endif

#ifndef NSEC_PER_USEC
#define NSEC_PER_USEC	1000L
#endif

#ifndef NSEC_PER_MSEC
#define NSEC_PER_MSEC	1000000L
#endif

#ifndef USEC_PER_SEC
#define USEC_PER_SEC	1000000L
#endif

#ifndef NSEC_PER_SEC
#define NSEC_PER_SEC	1000000000L
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,29) || \
    (LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0) && \
     LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7))
static inline unsigned int jiffies_to_msecs(const unsigned long j)
{
	if ((HZ <= MSEC_PER_SEC) && !(MSEC_PER_SEC % HZ)) {
		return (MSEC_PER_SEC / HZ) * j;
	} else if ((HZ > MSEC_PER_SEC) && !(HZ % MSEC_PER_SEC)) {
		/* gcc emits 'warning: division by zero' here, but this code
		 * is unreachable in that case. */
		return (j + (HZ / MSEC_PER_SEC) - 1) / (HZ / MSEC_PER_SEC);
	} else {
		return (j * MSEC_PER_SEC) / HZ;
	}
}

static inline unsigned long msecs_to_jiffies(const unsigned int m)
{
	if (m > jiffies_to_msecs(MAX_JIFFY_OFFSET)) {
		return MAX_JIFFY_OFFSET;
	}
	if ((HZ <= MSEC_PER_SEC) && !(MSEC_PER_SEC % HZ)) {
		return (m + (MSEC_PER_SEC / HZ) - 1) / (MSEC_PER_SEC / HZ);
	} else if ((HZ > MSEC_PER_SEC) && !(HZ % MSEC_PER_SEC)) {
		return m * (HZ / MSEC_PER_SEC);
	} else {
		return (m * HZ + MSEC_PER_SEC - 1) / MSEC_PER_SEC;
	}
};
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,29) || \
    (LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0) && \
     LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9))
static inline unsigned int jiffies_to_usecs(const unsigned long j)
{
	if ((HZ <= USEC_PER_SEC) && !(USEC_PER_SEC % HZ)) {
		return (USEC_PER_SEC / HZ) * j;
	} else if ((HZ > USEC_PER_SEC) && !(HZ % USEC_PER_SEC)) {
		/* gcc emits 'warning: division by zero' here, but this code
		 * is unreachable in that case. */
		return (j + (HZ / USEC_PER_SEC) - 1) / (HZ / USEC_PER_SEC);
	} else {
		return (j * USEC_PER_SEC) / HZ;
	}
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
static inline unsigned long usecs_to_jiffies(const unsigned int u)
{
	if (u > jiffies_to_usecs(MAX_JIFFY_OFFSET)) {
		return MAX_JIFFY_OFFSET;
	}
	if ((HZ <= USEC_PER_SEC) && !(USEC_PER_SEC % HZ)) {
		return (u + (USEC_PER_SEC / HZ) - 1) / (USEC_PER_SEC / HZ);
	} else if ((HZ > USEC_PER_SEC) && !(HZ % USEC_PER_SEC)) {
		return u * (HZ / USEC_PER_SEC);
	} else {
		return (u * HZ + USEC_PER_SEC - 1) / USEC_PER_SEC;
	}
}
#endif

/*
 * struct timeval was removed in kernel version 5.6.
 * Add a temporary version here for now until we fix the code that uses it.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,6,0)
struct timeval {
	long tv_sec;
	long tv_usec;
};
#endif

#endif
