/*
 * linux/timer.h compatibility header
 */

#ifndef __COMPAT_LINUX_TIMER_H_
#define __COMPAT_LINUX_TIMER_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,0)

#define mod_timer(a,b)	do{del_timer((a));(a)->expires=(b);add_timer((a));}while(0)
#endif

#include_next <linux/timer.h>

#endif

