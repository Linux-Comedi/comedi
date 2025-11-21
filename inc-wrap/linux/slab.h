/*
 * linux/slab.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__SLAB_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__SLAB_H__INCLUDED__

#include_next <linux/slab.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/string.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)

/* Some RHEL4 2.6.9 kernels have kzalloc.  Redefine to avoid warnings
   about static declaration following non-static declaration. */
#undef kzalloc
#define kzalloc(size, flags) comedi_kzalloc(size, flags)
static inline void *comedi_kzalloc(size_t size, gfp_t flags)
{
	void *ret = kmalloc(size, flags);
	if (ret)
		memset(ret, 0, size);
	return ret;
}

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)

#undef kcalloc
#define kcalloc(n, size, flags) comedi_kcalloc(n, size, flags)
static inline void *comedi_kcalloc(size_t n, size_t size, gfp_t flags)
{
	if (n != 0 && size > INT_MAX / n)
		return NULL;
	return kzalloc(n * size, flags);
}

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)

/* Some RHEL 2.6.32 kernels have kmalloc_array.  Redefine to avoid warnings
   about static declaration following non-static declaration. */
#undef kmalloc_array
#define kmalloc_array(n, size, flags) comedi_kmalloc_array(n, size, flags)
static inline void *comedi_kmalloc_array(size_t n, size_t size, gfp_t flags)
{
	if (n != 0 && size > ULONG_MAX / n)
		return NULL;
	return kmalloc(n * size, flags);
}

#endif

#endif

