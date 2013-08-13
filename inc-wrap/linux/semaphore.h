/*
 * linux/semaphore.h compatibility header
 */

#ifndef __COMPAT_LINUX_SEMAPHORE_H_
#define __COMPAT_LINUX_SEMAPHORE_H_

#include <linux/comedi-config.h>

#ifdef COMEDI_COMPAT_HAVE_LINUX_SEMAPHORE_H

#include_next <linux/semaphore.h>

#else /* COMEDI_COMPAT_HAVE_LINUX_SEMAPHORE_H */

#include <asm/semaphore.h>

#endif /* COMEDI_COMPAT_HAVE_LINUX_SEMAPHORE_H */

#endif /* __COMPAT_LINUX_SEMAPHORE_H_ */
