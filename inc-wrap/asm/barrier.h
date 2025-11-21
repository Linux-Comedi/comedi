/*
 * asm/barrier.h compatibility header
 */

#ifndef LINUX_COMPAT__ASM__BARRIER_H__INCLUDED__
#define LINUX_COMPAT__ASM__BARRIER_H__INCLUDED__

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#else
#include_next <asm/barrier.h>
#endif

#endif
