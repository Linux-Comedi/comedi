/*
 * linux/mm.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__MM_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__MM_H__INCLUDED__

#include_next <linux/mm.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
static inline int remap_pfn_range(struct vm_area_struct *vma,
	unsigned long from, unsigned long pfn, unsigned long size,
	pgprot_t prot)
{
	return remap_page_range(vma, from, pfn << PAGE_SHIFT, size, prot);
};

#endif

#endif
