/*
 * linux/mm.h compatibility header
 */

#ifndef _COMPAT_MM_H
#define _COMPAT_MM_H

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
#define VM_OFFSET(a)	((a)->vm_offset)
#define page_address(page) page
#else
#define VM_OFFSET(a)	((a)->vm_pgoff * PAGE_SIZE)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,10) \
			  && LINUX_VERSION_CODE < KERNEL_VERSION(2,5,3)
#include <asm/tlb.h>	/* look for tlb_vma() macro for "statm" patch */
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,3) && !defined(tlb_vma)
#define REMAP_PAGE_RANGE(a,b,c,d,e) remap_page_range(b,c,d,e)
#else
#define REMAP_PAGE_RANGE(a,b,c,d,e) remap_page_range(a,b,c,d,e)
#endif

#include_next <linux/mm.h>

#endif /* _COMPAT_MM_H */




