/*
 * linux/spinlock.h compatibility header
 */

#ifndef _COMPAT_SPINLOCK_H
#define _COMPAT_SPINLOCK_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020100
/* no spinlocks */
typedef int spinlock_t;

#define spin_lock_init(lock)    do { } while(0)
#define spin_lock(lock)         (void)(lock) /* Not "unused variable". */
#define spin_trylock(lock)      (1)
#define spin_unlock_wait(lock)  do { } while(0)
#define spin_unlock(lock)       do { } while(0)
#define spin_lock_irq(lock)     cli()
#define spin_unlock_irq(lock)   sti()

#define spin_lock_irqsave(lock, flags) \
	do { save_flags(flags); cli(); } while (0)
#define spin_unlock_irqrestore(lock, flags) \
	restore_flags(flags)

#define SPIN_LOCK_UNLOCKED 0

#else
#if LINUX_VERSION_CODE < 0x020300
#include <asm/spinlock.h>
#else
#include_next <linux/spinlock.h>
#endif
#endif


#endif /* _COMPAT_SPINLOCK_H */




