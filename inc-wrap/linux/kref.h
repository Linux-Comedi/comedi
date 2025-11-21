/*    
 * linux/kref.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__KREF_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__KREF_H__INCLUDED__

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5)

#include <linux/types.h>
#include <asm/atomic.h>

struct kref {
	atomic_t refcount;
};

static inline void kref_init(struct kref *kref)
{
	atomic_set(&kref->refcount, 1);
}

static inline void kref_get(struct kref *kref)
{
	atomic_inc(&kref->refcount);
}

static inline int kref_put(struct kref *kref,
	void (*release) (struct kref * kref))
{
	if (atomic_dec_and_test(&kref->refcount)) {
		release(kref);
		return 1;
	}
	return 0;
}

#else

#include_next <linux/kref.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
#include <asm/bug.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)

/* Dummy release function should never be called. */
static void comedi_dummy_kref_release(struct kref *kref)
{
	BUG();
}

/* Redefine kref_init to remove 'release' parameter. */
static inline void comedi_internal_kref_init(struct kref *kref)
{
	kref_init(kref, comedi_dummy_kref_release);
}
#undef kref_init
#define kref_init(kref) comedi_internal_kref_init(kref)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9) */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12)

/* Redefine kref_put to add 'release' parameter and return a result. */
static inline int comedi_internal_kref_put(struct kref *kref,
	void (*release) (struct kref * kref))
{
	if (atomic_dec_and_test(&kref->refcount)) {
		release(kref);
		return 1;
	}
	return 0;
}
#undef kref_put
#define kref_put(kref, release) comedi_internal_kref_put(kref, release)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12) */

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,5) */

#endif
