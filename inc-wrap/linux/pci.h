/*
 * linux/pci.h compatibility header
 */

#ifndef _COMPAT_PCI_H
#define _COMPAT_PCI_H

#include <linux/version.h>

#include_next <linux/pci.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#define pci_get_device pci_find_device
#define pci_get_subsys pci_find_subsys
#define pci_dev_get(x)	(x)
#define pci_dev_put(x)
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,22)
static inline char *pci_name(struct pci_dev *pdev)
{
	return pdev->slot_name;
}
#endif

#ifndef DEFINE_PCI_DEVICE_TABLE
#define DEFINE_PCI_DEVICE_TABLE(_table) \
	struct pci_device_id _table[]
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,53)
/*
 * Since kernel version 2.5.53, the PCI DMA API is just a set of inline wrapper
 * functions around the DMA API.  These wrappers have been removed for kernel
 * version 5.18.  Add them here for convenience if missing.
 */
#ifndef PCI_DMA_NONE
#include <linux/dma-mapping.h>

/* This defines the direction arg to the DMA mapping routines. */
#define PCI_DMA_BIDIRECTIONAL	DMA_BIDIRECTIONAL
#define PCI_DMA_TODEVICE	DMA_TO_DEVICE
#define PCI_DMA_FROMDEVICE	DMA_FROM_DEVICE
#define PCI_DMA_NONE		DMA_NONE

static inline void *
pci_alloc_consistent(struct pci_dev *hwdev, size_t size,
		     dma_addr_t *dma_handle)
{
	return dma_alloc_coherent(&hwdev->dev, size, dma_handle, GFP_ATOMIC);
}

/*
 * Note that dma_zalloc_coherent() was removed in kernel 5.0 because
 * dma_alloc_coherent() already cleared the memory.
 */
static inline void *
pci_zalloc_consistent(struct pci_dev *hwdev, size_t size,
		      dma_addr_t *dma_handle)
{
	return dma_alloc_coherent(&hwdev->dev, size, dma_handle, GFP_ATOMIC);
}

static inline void
pci_free_consistent(struct pci_dev *hwdev, size_t size,
		    void *vaddr, dma_addr_t dma_handle)
{
	dma_free_coherent(&hwdev->dev, size, vaddr, dma_handle);
}

static inline dma_addr_t
pci_map_single(struct pci_dev *hwdev, void *ptr, size_t size, int direction)
{
	return dma_map_single(&hwdev->dev, ptr, size, (enum dma_data_direction)direction);
}

static inline void
pci_unmap_single(struct pci_dev *hwdev, dma_addr_t dma_addr,
		 size_t size, int direction)
{
	dma_unmap_single(&hwdev->dev, dma_addr, size, (enum dma_data_direction)direction);
}

static inline dma_addr_t
pci_map_page(struct pci_dev *hwdev, struct page *page,
	     unsigned long offset, size_t size, int direction)
{
	return dma_map_page(&hwdev->dev, page, offset, size, (enum dma_data_direction)direction);
}

static inline void
pci_unmap_page(struct pci_dev *hwdev, dma_addr_t dma_address,
	       size_t size, int direction)
{
	dma_unmap_page(&hwdev->dev, dma_address, size, (enum dma_data_direction)direction);
}

static inline int
pci_map_sg(struct pci_dev *hwdev, struct scatterlist *sg,
	   int nents, int direction)
{
	return dma_map_sg(&hwdev->dev, sg, nents, (enum dma_data_direction)direction);
}

static inline void
pci_unmap_sg(struct pci_dev *hwdev, struct scatterlist *sg,
	     int nents, int direction)
{
	dma_unmap_sg(&hwdev->dev, sg, nents, (enum dma_data_direction)direction);
}

static inline void
pci_dma_sync_single_for_cpu(struct pci_dev *hwdev, dma_addr_t dma_handle,
		    size_t size, int direction)
{
	dma_sync_single_for_cpu(&hwdev->dev, dma_handle, size, (enum dma_data_direction)direction);
}

static inline void
pci_dma_sync_single_for_device(struct pci_dev *hwdev, dma_addr_t dma_handle,
		    size_t size, int direction)
{
	dma_sync_single_for_device(&hwdev->dev, dma_handle, size, (enum dma_data_direction)direction);
}

static inline void
pci_dma_sync_sg_for_cpu(struct pci_dev *hwdev, struct scatterlist *sg,
		int nelems, int direction)
{
	dma_sync_sg_for_cpu(&hwdev->dev, sg, nelems, (enum dma_data_direction)direction);
}

static inline void
pci_dma_sync_sg_for_device(struct pci_dev *hwdev, struct scatterlist *sg,
		int nelems, int direction)
{
	dma_sync_sg_for_device(&hwdev->dev, sg, nelems, (enum dma_data_direction)direction);
}

static inline int
pci_dma_mapping_error(struct pci_dev *pdev, dma_addr_t dma_addr)
{
	return dma_mapping_error(&pdev->dev, dma_addr);
}

#ifdef CONFIG_PCI
static inline int pci_set_dma_mask(struct pci_dev *dev, u64 mask)
{
	return dma_set_mask(&dev->dev, mask);
}

static inline int pci_set_consistent_dma_mask(struct pci_dev *dev, u64 mask)
{
	return dma_set_coherent_mask(&dev->dev, mask);
}
#else
static inline int pci_set_dma_mask(struct pci_dev *dev, u64 mask)
{ return -EIO; }
static inline int pci_set_consistent_dma_mask(struct pci_dev *dev, u64 mask)
{ return -EIO; }
#endif

#endif /* PCI_DMA_NONE */
#endif	/* LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,53) */

#endif /* _COMPAT_PCI_H */
