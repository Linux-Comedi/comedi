/*
 * linux/pci.h compatibility header
 */

#ifndef _COMPAT_PCI_H
#define _COMPAT_PCI_H

#include <linux/version.h>


#if LINUX_VERSION_CODE < 0x020155
#include <linux/bios32.h>
#define PCI_SUPPORT_VER1
#else
#include_next <linux/pci.h>
#define PCI_SUPPORT_VER2

#if LINUX_VERSION_CODE < 0x020300 /* XXX */

#define pci_for_each_dev(x)	\
	for((x)=pci_devices;(x);(x)=(x)->next)

#define pci_enable_device(x) 0

#define PCI_ANY_ID (~0)

struct pci_device_id {
	unsigned int vendor, device;
	unsigned int subvendor, subdevice;
	unsigned int class, class_mask;
	unsigned long driver_data;
};

// stuff for allocating pci dma buffers
#include <asm/io.h>
#include <linux/malloc.h>
#define PCI_DMA_FROMDEVICE              0
#define PCI_DMA_TODEVICE                0

extern inline void *pci_alloc_consistent(struct pci_dev *hwdev, size_t size,
	dma_addr_t *dma_handle)
{
        void *ret;
        int gfp = GFP_KERNEL;

        if (hwdev == NULL)
                gfp |= GFP_DMA;
        ret = (void *) __get_free_pages(gfp, get_order(size));

        if (ret != NULL) 
	{
                memset(ret, 0, size);
                *dma_handle = virt_to_bus(ret);
        }
        return ret;
}

extern inline void pci_free_consistent(struct pci_dev *hwdev, size_t size,
	void *vaddr, dma_addr_t dma_handle)
{
	free_pages((unsigned long)vaddr, get_order(size));
}

#define pci_map_single(cookie, address, size, dir)      virt_to_bus(address)
#define pci_unmap_single(cookie, address, size, dir)
#define pci_dma_sync_single(cookie, address, size, dir)

// for getting base addresses
#define pci_resource_start(dev,bar)   ((dev)->base_address[(bar)])

#else

#endif

#endif

#endif /* _COMPAT_PCI_H */




