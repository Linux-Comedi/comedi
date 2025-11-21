/*
 * linux/spinlock.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__SPINLOCK_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__SPINLOCK_H__INCLUDED__

#include_next <linux/spinlock.h>

#ifndef DEFINE_SPINLOCK
/* Early 2.6.x kernels are missing the DEFINE_SPINLOCK(x) macro. */
#define DEFINE_SPINLOCK(x) spinlock_t x = SPIN_LOCK_UNLOCKED
#endif

#ifndef DEFINE_RWLOCK
/* Early 2.6.x kernels are missing the DEFINE_RWLOCK(x) macro. */
#define DEFINE_RWLOCK(x) rwlock_t x = RW_LOCK_UNLOCKED
#endif

#endif
