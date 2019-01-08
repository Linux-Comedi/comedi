/*
 * linux/uaccess.h compatibility header
 */

#ifndef __COMPAT_LINUX_UACCESS_H_
#define __COMPAT_LINUX_UACCESS_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#include <asm/uaccess.h>
#else
#include_next <linux/uaccess.h>
#endif

/*
 * Kernel 5.0 removed the verification type (first argument) of
 * access_ok(type,addr,size) and also removed VERIFY_READ and VERIFY_WRITE.
 *
 * The verification type has been completely ignored in the 2.6 kernel onwards.
 */
#ifdef VERIFY_READ
static inline int _comedi_access_ok(unsigned long addr, size_t size)
{
	/* The verification type is ignored by the kernel (2.6 onwards). */
	return access_ok(0, addr, size);
}
#undef access_ok
#define access_ok(addr, size) _comedi_access_ok((unsigned long)(addr), (size))
#endif

#endif /* __COMPAT_LINUX_UACCESS_H_ */
