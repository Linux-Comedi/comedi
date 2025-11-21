/*
 * asm/io.h compatibility header
 */

#ifndef COMEDI_COMPAT__ASM__IO_H__INCLUDED__
#define COMEDI_COMPAT__ASM__IO_H__INCLUDED__

#include_next <asm/io.h>
#include <linux/version.h>
#include <linux/config.h>

#ifndef mmiowb			/* Defined in 2.6.10 */

#if defined(CONFIG_MIPS)

/* Depends on MIPS II instruction set */
#define mmiowb() asm volatile ("sync" ::: "memory")

#elif defined(CONFIG_IA64)

/* IA64 */
#ifdef CONFIG_IA64_SGI_SN2
#include <asm/sn/io.h>
#define platform_mmiowb sn_mmiob
#endif

#ifndef platform_mmiowb
#define platform_mmiowb ia64_mfa
#endif

#define mmiowb() platform_mmiowb()

#else

/* Other architectures */
#define mmiowb()

#endif

#endif /* mmiowb */

/*
 * Try and define isa_virt_to_bus() if not already defined as a macro or
 * an inline function, just for building purposes.  The actual modules that
 * use it are unlikely to work on anything that does not have an ISA bus.
 */
#if !defined(isa_virt_to_bus) && \
    !(defined(CONFIG_MIPS) && \
      LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,74)) && \
    !(defined(CONFIG_PARISC) && \
      LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,23)) && \
    !(defined(CONFIG_X86) && \
      LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,4))
#define isa_virt_to_bus virt_to_phys
#endif

#endif
