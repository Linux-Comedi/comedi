/*
 * linux/uaccess.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__UACCESS_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__UACCESS_H__INCLUDED__

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
#include <asm/uaccess.h>
#else
#include_next <linux/uaccess.h>
#endif

#include <linux/compiler.h> /* for __user */

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

#endif
