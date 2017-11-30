/*
 * linux/timer.h compatibility header
 */

#ifndef __COMPAT_LINUX_TIMER_H_
#define __COMPAT_LINUX_TIMER_H_

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

#endif /* __COMPAT_LINUX_TIMER_H_ */
