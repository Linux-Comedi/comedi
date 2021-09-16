
#ifndef __COMPAT_LINUX_SLAB_H
#define __COMPAT_LINUX_SLAB_H

#include <linux/version.h>
#include <linux/string.h>

#include_next <linux/slab.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)

/* Some RHEL4 2.6.9 kernels have kzalloc.  Redefine to avoid warnings
   about static declaration following non-static declaration. */
#undef kzalloc
#define kzalloc comedi_kzalloc
static inline void *comedi_kzalloc(size_t size, unsigned int flags)
{
	void *ret = kmalloc(size, flags);
	if (ret)
		memset(ret, 0, size);
	return ret;
}

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)

static inline void *kcalloc(size_t n, size_t size, int flags)
{
	if (n != 0 && size > INT_MAX / n)
		return NULL;
	return kzalloc(n * size, flags);
}

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,2,0)

/* Some RHEL 2.6.32 kernels have kmalloc_array.  Redefine to avoid warnings
   about static declaration following non-static declaration. */
#undef kmalloc_array
#define kmalloc_array comedi_kmalloc_array
static inline void *comedi_kmalloc_array(size_t n, size_t size, unsigned int flags)
{
	if (n != 0 && size > ULONG_MAX / n)
		return NULL;
	return kmalloc(n * size, flags);
}

#endif

#endif

