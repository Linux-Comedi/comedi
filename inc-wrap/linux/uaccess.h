/*
 * linux/uaccess.h compatibility header
 */

#ifndef __COMPAT_LINUX_UACCESS_H_
#define __COMPAT_LINUX_UACCESS_H_

#include <linux/version.h>
#include <linux/compiler.h> /* for __user */

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
static inline int comedi_access_ok(const void __user *addr, size_t size)
{
	/* Always use VERIFY_WRITE.  Most architectures ignore it. */
	return access_ok(
#ifdef VERIFY_WRITE
			 VERIFY_WRITE,
#endif
			 addr, size);
}

#endif /* __COMPAT_LINUX_UACCESS_H_ */
