/*
 * asm/barrier.h compatibility header
 */

#ifndef __COMPAT_ASM_BARRIER_H_
#define __COMPAT_ASM_BARRIER_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#else
#include_next <asm/barrier.h>
#endif

#endif /* __COMPAT_ASM_BARRIER_H_ */
