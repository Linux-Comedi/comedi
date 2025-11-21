/*
 * linux/mutex.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__MUTEX_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__MUTEX_H__INCLUDED__

#include <linux/comedi-config.h>

#ifdef COMEDI_COMPAT_HAVE_MUTEX_H

#include_next <linux/mutex.h>

#include <linux/version.h>
#include <linux/config.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
#ifndef CONFIG_DEBUG_MUTEXES
#ifndef mutex_destroy
/* Some Redhat kernels include a backported mutex.h, lacking mutex_destroy */
#define mutex_destroy(m) do {} while (0)
#endif
#endif
#endif

#else /* COMEDI_COMPAT_HAVE_MUTEX_H */

#include <asm/semaphore.h>

#define DEFINE_MUTEX(m) DECLARE_MUTEX(m)
#define mutex_init(m) init_MUTEX(m)
#define mutex_destroy(m) do {} while (0)
#define mutex_lock(m) down(m)
#define mutex_lock_interruptible(m) down_interruptible(m)
#define mutex_trylock(m) (!down_trylock(m))
#define mutex_unlock(m) up(m)
/* There is some unfortunate name-space pollution in the following macro, so any
 * code using 'mutex' as an identifier has to be careful with include order. */
#define mutex semaphore		/* "struct mutex" becomes "struct semaphore" */

#endif /* COMEDI_COMPAT_HAVE_MUTEX_H */

#endif
