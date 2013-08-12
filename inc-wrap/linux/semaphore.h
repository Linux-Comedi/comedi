/*
 * linux/semaphore.h compatibility header
 */

#ifndef __COMPAT_LINUX_SEMAPHORE_H_
#define __COMPAT_LINUX_SEMAPHORE_H_

#include <linux/config.h>

#ifdef CONFIG_COMEDI_HAVE_LINUX_SEMAPHORE_H

#include_next <linux/semaphore.h>

#else /* HAVE_LINUX_SEMAPHORE_H */

#include <asm/semaphore.h>

#endif /* HAVE_LINUX_SEMAPHORE_H */

#endif /* __COMPAT_LINUX_SEMAPHORE_H_ */
