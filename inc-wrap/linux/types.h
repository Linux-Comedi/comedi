/*
 * linux/types.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__TYPES_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__TYPES_H__INCLUDED__

#include_next <linux/types.h>

#include <linux/version.h>
#include <linux/compiler.h>
#include <linux/comedi-config.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18)
typedef unsigned long resource_size_t;
#else
/* resource_size_t is either u32 or u64, depending on CONFIG_RESOURCES_64BIT */
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,14)
typedef unsigned __nocast gfp_t;
#endif

#ifndef COMEDI_COMPAT_HAVE_GENERIC_BOOL_TYPE
#if __GNUC__ >= 3
typedef _Bool bool;
#else
/*
 * The compiler does not have a type with C99 _Bool semantics, so use char
 * instead.  Things might break if they rely on the _Bool semantics.
 */
typedef char bool;
#endif
#endif

#endif
