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

/*
 * The following compability functions really belong in our linux/string.h
 * compatibility header, but adding #include <linux/slab.h> to our
 * linux/string.h compatibility header produces all sorts of problems, at
 * least when building for early 2.6.x kernels.
 */

/*
 * kstrdup() was introduced in Linux kernel 2.6.13.  Define a compatibility
 * function for earlier kernels.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

#undef kstrdup
#define kstrdup(src, gfp) comedi_kstrdup(src, gfp)
static inline char *comedi_kstrdup(const char *s, gfp_t gfp)
{
	size_t len;
	char *buf;

	if (!s)
		return NULL;

	len = strlen(s) + 1;
	buf = kmalloc(len, gfp);
	if (buf)
		memcpy(buf, s, len);
	return buf;
}

#endif

/*
 * kstrndup() was introduced in Linux kernel 2.6.23.  Define a compatibility
 * function for earlier kernels.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,23)

#undef kstrndup
#define kstrndup(src, max, gfp) comedi_kstrndup(src, max, gfp)
static inline char *comedi_kstrndup(const void *s, size_t max, gfp_t gfp)
{
	size_t len;
	char *buf;

	if (!s)
		return NULL;

	len = strnlen(s, max);
	buf = kmalloc(len + 1, gfp);
	if (buf) {
		memcpy(buf, s, len);
		buf[len] = '\0';
	}
	return buf;
}

#endif

/*
 * kmemdup() was introduced in Linux kernel 2.6.19.  Define a compatibility
 * function for earlier kernels.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)

#undef kmemdup
#define kmemdup(src, len, gfp) comedi_kmemdup(src, len, gfp)
static inline void *comedi_kmemdup(const void *src, size_t len, gfp_t gfp)
{
	void *p = kmalloc(len, gfp);

	if (p)
		memcpy(p, src, len);
	return p;
}

#endif

#endif

