// linux/ioport.h compatibility header

#ifndef _COMPAT_IOPORT_H
#define _COMPAT_IOPORT_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,20)

#define request_mem_region(start,n,name) ((void*) 1)
#define release_mem_region(start,n)

#endif

#include_next <linux/ioport.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,11)

/* 'struct resource' not defined.  Define a dummy version. */
struct resource {
	int dummy;
};

/* Define a compatible version of request_region that returns a pointer
 * to a non-NULL value on success.  */
static inline struct resource *compat__request_region(unsigned long from,
		unsigned long extent, const char *name)
{
	if (check_region(from, extent) < 0) {
		return (struct resource *)0;
	}
	request_region(from, extent, name);
	return ((struct resource *)0 + 1);	/* Should be non-NULL */
}

/* Replace existing request_region macro/function. */
#undef request_region
#define request_region(f,e,n)	compat__request_region(f,e,n)

#endif

#endif // _COMPAT_IOPORT_H

