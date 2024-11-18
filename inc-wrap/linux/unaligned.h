/*
 * linux/unaligned.h compatibility header
 */

#ifndef _COMEDI_COMPAT_LINUX_UNALIGNED_H
#define _COMEDI_COMPAT_LINUX_UNALIGNED_H

#include <linux/version.h>

/* The unaligned.h header moved from asm/ to linux/ in the 6.12 kernel. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
#include_next <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#endif /* _COMEDI_COMPAT_LINUX_UNALIGNED_H */
