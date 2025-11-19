/*
 * linux/dma-mapping.h compatibility header
 */

#ifndef __COMPAT_LINUX_DMA_MAPPING_H_
#define __COMPAT_LINUX_DMA_MAPPING_H_

#include <linux/version.h>
#include <linux/config.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9)
/*
 * <linux/dma-mapping.h> includes <asm/dma-mapping.h> which may dereference
 * pointers to struct device, so include <linux/device.h>.
 */
#include <linux/device.h>
#endif

#include_next <linux/dma-mapping.h>

#if !defined(dma_mmap_coherent) && !defined(CONFIG_ARC) && \
    !defined(CONFIG_METAG) && \
    (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0) || \
     !defined(CONFIG_AVR32) && !defined(CONFIG_BLACKFIN) && \
     !defined(CONFIG_CRIS) && !defined(CONFIG_M68K)) && \
    (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39) || \
     !defined(CONFIG_PPC32) && !defined(CONFIG_PPC64)) && \
    (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,11) || !defined(CONFIG_ARM))

#include <linux/mm.h>
#include <asm/io.h>

/*
 * Provide a version of dma_mmap_coherent() for compatibility.
 * We assume the CPU address is in the linear mapped range of addresses.
 */
static inline int
comedi_dma_mmap_coherent(struct device *dev, struct vm_area_struct *vma,
			 void *cpu_addr, dma_addr_t dma_addr, size_t size)
{
	return remap_pfn_range(vma, (unsigned long)cpu_addr,
			       virt_to_phys(cpu_addr) >> PAGE_SHIFT, size,
			       vma->vm_page_prot);
}

#undef dma_mmap_coherent
#define dma_mmap_coherent(dev, vma, cpu_addr, dma_addr, size) \
	comedi_dma_mmap_coherent(dev, vma, cpu_addr, dma_addr, size)

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)

/*
 * Provide a version of dma_set_coherent_mask() for compatibility.
 */
static inline int comedi_dma_set_coherent_mask(struct device *dev, u64 mask)
{
	if (!dma_supported(dev, mask))
		return -EIO;
	dev->coherent_dma_mask = mask;
	return 0;
}

#undef dma_set_coherent_mask
#define dma_set_coherent_mask(dev, mask) comedi_dma_set_coherent_mask(dev, mask)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34) */

#endif
