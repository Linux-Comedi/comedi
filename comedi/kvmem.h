/*******************************/
/* Memory management functions */
/*******************************/

#ifndef _KVMEM_H
#define _KVMEM_H
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/highmem.h>  /* for SuSE brokenness */
#include <linux/wrapper.h>
#include <asm/pgtable.h>
#include <asm/io.h>

/* convert virtual user memory address to physical address */
/* (virt_to_phys only works for kmalloced kernel memory) */

#if LINUX_VERSION_CODE < 0x020300
static inline unsigned long uvirt_to_phys(unsigned long adr)
{
	pgd_t *pgd;
	pmd_t *pmd;
	pte_t *ptep, pte;
  
	pgd = pgd_offset(current->mm, adr);
	if (pgd_none(*pgd))
		return 0;
	pmd = pmd_offset(pgd, adr);
	if (pmd_none(*pmd))
		return 0;
	ptep = pte_offset(pmd, adr/*&(~PGDIR_MASK)*/);
	pte = *ptep;
	if(pte_present(pte))
		return virt_to_phys((void *)(pte_page(pte)|(adr&(PAGE_SIZE-1))));
	return 0;
}

static inline unsigned long uvirt_to_bus(unsigned long adr) 
{
	return virt_to_bus(phys_to_virt(uvirt_to_phys(adr)));
}

/* convert virtual kernel memory address to physical address */
/* (virt_to_phys only works for kmalloced kernel memory) */

static inline unsigned long kvirt_to_phys(unsigned long adr) 
{
	return uvirt_to_phys(VMALLOC_VMADDR(adr));
}

static inline unsigned long kvirt_to_bus(unsigned long adr) 
{
	return uvirt_to_bus(VMALLOC_VMADDR(adr));
}

#else

static inline unsigned long uvirt_to_kva(pgd_t *pgd, unsigned long adr)
{
	unsigned long ret = 0UL;
	pmd_t *pmd;
	pte_t *ptep, pte;

	if(!pgd_none(*pgd)) {
		pmd = pmd_offset(pgd, adr);
		if (!pmd_none(*pmd)) {
			ptep = pte_offset(pmd, adr);
			pte = *ptep;
			if(pte_present(pte)){
				ret = (unsigned long) page_address(pte_page(pte));
				ret |= (adr&(PAGE_SIZE-1));
			}
		}
	}
	return ret;
}

static inline unsigned long uvirt_to_bus(unsigned long adr)
{
	unsigned long kva, ret;

	kva = uvirt_to_kva(pgd_offset(current->mm, adr), adr);
	ret = virt_to_bus((void *)kva);

	return ret;
}

static inline unsigned long kvirt_to_bus(unsigned long adr)
{
	unsigned long va, kva, ret;

	va = VMALLOC_VMADDR(adr);
	kva = uvirt_to_kva(pgd_offset_k(va), va);
	ret = virt_to_bus((void *)kva);

	return ret;
}

static inline unsigned long kvirt_to_pa(unsigned long adr)
{
	unsigned long va, kva, ret;

	va = VMALLOC_VMADDR(adr);
	kva = uvirt_to_kva(pgd_offset_k(va), va);
	ret = __pa(kva);

	return ret;
}


#endif


extern void * rvmalloc(unsigned long size);
extern void rvfree(void * mem, unsigned long size);
extern int rvmmap(void *mem, unsigned memsize, struct vm_area_struct *vma);
#endif
