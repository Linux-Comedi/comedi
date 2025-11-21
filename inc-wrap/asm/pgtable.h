/*
 * asm/pgtable.h compatibility header
 */

#ifndef COMEDI_COMPAT__ASM__PGTABLE_H__INCLUDED__
#define COMEDI_COMPAT__ASM__PGTABLE_H__INCLUDED__

#include_next <asm/pgtable.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11)
#define pud_t pgd_t
#define pud_offset(pgd, start)	(pgd)
#endif

#if !defined(pte_offset_kernel) && LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
#define pte_offset_kernel(dir,address) pte_offset(dir,address)
#endif

#endif
