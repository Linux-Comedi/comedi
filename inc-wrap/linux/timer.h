/*
 * linux/timer.h compatibility header
 */

#ifndef __COMPAT_LINUX_TIMER_H_
#define __COMPAT_LINUX_TIMER_H_

#include <linux/version.h>

#include_next <linux/timer.h>

#ifndef from_timer

static inline void timer_setup(struct timer_list *timer,
			       void (*callback)(struct timer_list *),
			       unsigned int flags)
{
	/* Comedi doesn't use the 'flags' parameter yet, so ignore it! */
	timer->function = (void (*)(unsigned long))callback;
	timer->data = (unsigned long)timer;
	init_timer(timer);
}

#define from_timer(var, callback_timer, timer_fieldname) \
	container_of(callback_timer, typeof(*var), timer_fieldname)

#endif /* from_timer */

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,2,0)

#undef timer_delete
#define timer_delete(timer) del_timer(timer)

#undef timer_delete_sync
#define timer_delete_sync(timer) del_timer_sync(timer)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(6,2,0) */

#endif /* __COMPAT_LINUX_TIMER_H_ */
