/*
 * linux/irq.h compatibility header
 */

#ifndef _COMPAT_IRQ_H
#define _COMPAT_IRQ_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020300
#include <asm/irq.h>
#else
#include_next <linux/irq.h>
#endif


#endif /* _COMPAT_IRQ_H */




