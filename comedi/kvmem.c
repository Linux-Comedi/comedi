
#include <linux/kernel.h>
#include <linux/config.h>
#include <linux/vmalloc.h>
#include "kvmem.h"

/* allocate user space mmapable block of memory in the kernel space */
void * rvmalloc(unsigned long size)
{
	void * mem;
	unsigned long adr, page;
        
#if LINUX_VERSION_CODE < 0x020300
	mem=vmalloc(size);
#else
	mem=vmalloc_32(size);
#endif
	if (mem) 
	{
		memset(mem, 0, size); /* Clear the ram out, no junk to the user */
	        adr=(unsigned long) mem;
		while (size > 0) 
                {
#if LINUX_VERSION_CODE < 0x020300
	                page = kvirt_to_phys(adr);
			mem_map_reserve(MAP_NR(phys_to_virt(page)));
#else
	                page = kvirt_to_pa(adr);
			mem_map_reserve(virt_to_page(__va(page)));
#endif
			adr+=PAGE_SIZE;
			size-=PAGE_SIZE;
		}
	}
	return mem;
}

void rvfree(void * mem, unsigned long size)
{
        unsigned long adr, page;
        
	if (mem) 
	{
	        adr=(unsigned long) mem;
		while (size > 0) 
                {
#if LINUX_VERSION_CODE < 0x020300
	                page = kvirt_to_phys(adr);
			mem_map_unreserve(MAP_NR(phys_to_virt(page)));
#else
	                page = kvirt_to_pa(adr);
			mem_map_unreserve(virt_to_page(__va(page)));
#endif
			adr+=PAGE_SIZE;
			size-=PAGE_SIZE;
		}
		vfree(mem);
	}
}

/* this function will map (fragment of) rvmalloc'ed memory area to user space */
int rvmmap(void *mem, unsigned memsize, struct vm_area_struct *vma) {
	unsigned long pos, size, start=vma->vm_start;
#if LINUX_VERSION_CODE < 0x20300
	unsigned long offset = vma->vm_offset;
#else
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
#endif
	/* this is not time critical code, so we check the arguments */
	/* vma->vm_offset HAS to be checked (and is checked)*/
	if (offset<0)
		return -EFAULT;
	size = vma->vm_end - vma->vm_start;
	if (size + offset > memsize)
		return -EFAULT;
	pos = (unsigned long) mem + offset;
	if (pos%PAGE_SIZE || start%PAGE_SIZE || size%PAGE_SIZE)
		return -EFAULT;
		
	while (size>0) {
#if LINUX_VERSION_CODE < 0x020300
		if (remap_page_range(start,kvirt_to_phys(pos), PAGE_SIZE, 
			vma->vm_page_prot )) {
			/* FIXME: what should we do here to unmap previous pages ?*/
			printk(KERN_ERR "rvmmap failed: vm_start=0x%lx, vm_end=0x%lx, size=0x%lx, pos=0x%lx; please report to motyl@stan.chemie.unibas.ch\n",vma->vm_start,vma->vm_end,size,pos);
			return -EFAULT;
		}
#else
		if (remap_page_range(start, kvirt_to_pa(pos),
		    PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;
#endif
		pos+=PAGE_SIZE;
		start+=PAGE_SIZE;
		size-=PAGE_SIZE;
	}
	return 0;
}
