/*
 * linux/delay.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__DELAY_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__DELAY_H__INCLUDED__

#include_next <linux/delay.h>
#include <linux/version.h>
#include <linux/time.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,7)
static inline void comedi_msleep(unsigned int msecs)
{
	unsigned long timeout = msecs_to_jiffies(msecs);

	while (timeout) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		timeout = schedule_timeout(timeout);
	}
}
#undef msleep
#define msleep(msecs) comedi_msleep(msecs)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
static inline unsigned long comedi_msleep_interruptible(unsigned int msecs)
{
	unsigned long timeout = msecs_to_jiffies(msecs);

	while (timeout && !signal_pending(current)) {
		set_current_state(TASK_INTERRUPTIBLE);
		timeout = schedule_timeout(timeout);
	}
	return jiffies_to_msecs(timeout);
}
#undef msleep_interruptible
#define msleep_interruptible(msecs) comedi_msleep_interruptible(msecs)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
static inline void comedi_ssleep(unsigned int seconds)
{
	msleep(seconds * 1000);
}
#undef ssleep
#define ssleep(seconds) comedi_ssleep(seconds)
#endif

#endif
