/*
 * linux/irq.h compatibility header
 */

#ifndef _COMPAT_IRQ_H
#define _COMPAT_IRQ_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
#include <asm/irq.h>
#else
#include_next <linux/irq.h>
#endif


#endif /* _COMPAT_IRQ_H */




