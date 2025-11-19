/*
 * linux/module.h compatibility header
 */

#ifndef __COMPAT_LINUX_MODULE_H_
#define __COMPAT_LINUX_MODULE_H_

#include <linux/version.h>
#include_next <linux/module.h>

#ifndef MODULE_VERSION
/* Early 2.6.x kernels are missing the MODULE_VERSION() macro. */
#define MODULE_VERSION(version)
#endif

/*
 * The KBUILD_MODNAME macro expands to a string since kernel version 2.6.16,
 * and to an identifier before 2.6.16.
 *
 * Define COMEDI_MODNAME macro to expand to KBUILD_MODNAME since kernel
 * version 2.6.16, or to a stringified version of KBUILD_MODNAME for kernel
 * versions before 2.6.16.
 *
 * This might not be the correct header to do this since KBUILD_MODNAME is
 * defined on the compiler command line, but it will do.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
#define COMEDI_MODNAME __stringify(KBUILD_MODNAME)
#else
#define COMEDI_MODNAME KBUILD_MODNAME
#endif

#endif
