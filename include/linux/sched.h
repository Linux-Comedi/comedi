/*
 * linux/sched.h compatibility header
 */

#ifndef __COMPAT_LINUX_SCHED_H_
#define __COMPAT_LINUX_SCHED_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,0)
#define signal_pending(x)	(((x)->signal) & (~(x)->blocked))
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,3)
#define __wait_event(wq, condition) 					\
do {									\
	struct wait_queue __wait;					\
									\
	__wait.task = current;						\
	add_wait_queue(&wq, &__wait);					\
	for (;;) {							\
		current->state = TASK_UNINTERRUPTIBLE;			\
		mb();							\
		if (condition)						\
			break;						\
		schedule();						\
	}								\
	current->state = TASK_RUNNING;					\
	remove_wait_queue(&wq, &__wait);				\
} while (0)

#define wait_event(wq, condition) 					\
do {									\
	if (condition)	 						\
		break;							\
	__wait_event(wq, condition);					\
} while (0)

#define __wait_event_interruptible(wq, condition, ret)			\
do {									\
	struct wait_queue __wait;					\
									\
	__wait.task = current;						\
	add_wait_queue(&wq, &__wait);					\
	for (;;) {							\
		current->state = TASK_INTERRUPTIBLE;			\
		mb();							\
		if (condition)						\
			break;						\
		if (!signal_pending(current)) {				\
			schedule();					\
			continue;					\
		}							\
		ret = -ERESTARTSYS;					\
		break;							\
	}								\
	current->state = TASK_RUNNING;					\
	remove_wait_queue(&wq, &__wait);				\
} while (0)
	
#define wait_event_interruptible(wq, condition)				\
({									\
	int __ret = 0;							\
	if (!(condition))						\
		__wait_event_interruptible(wq, condition, __ret);	\
	__ret;								\
})
#endif

#include_next <linux/sched.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,20)
static inline int need_resched(void)
{
	return current->need_resched;
}
#endif

#endif

