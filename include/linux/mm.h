/*
 * linux/mm.h compatibility header
 */

#ifndef _COMPAT_MM_H
#define _COMPAT_MM_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020300
#define VM_OFFSET(a)	((a)->vm_offset)
#else
#define VM_OFFSET(a)	((a)->vm_pgoff * PAGE_SIZE)
#endif

#include_next <linux/mm.h>

#endif /* _COMPAT_MM_H */




