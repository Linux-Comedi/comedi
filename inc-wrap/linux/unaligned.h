/*
 * linux/unaligned.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__UNALIGNED_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__UNALIGNED_H__INCLUDED__

#include <linux/version.h>

/* The unaligned.h header moved from asm/ to linux/ in the 6.12 kernel. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,12,0)
#include_next <linux/unaligned.h>
#else
#include <asm/unaligned.h>
#endif

#endif /* _COMEDI_COMPAT_LINUX_UNALIGNED_H */
