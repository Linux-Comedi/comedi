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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
/* we should get rid of this, as it has been dropped from 2.6 */
#define pci_for_each_dev(x)	\
	for((x) = pci_dev_g(pci_devices.next), prefetch((x)->global_list.next); \
		x->global_list.next != &pci_devices; (x) = pci_dev_g((x)->global_list.next), \
		prefetch((x)->global_list.next))
#endif

#if LINUX_VERSION_CODE < 0x020300 /* XXX */

#define pci_for_each_dev(x)	\
	for((x)=pci_devices;(x);(x)=(x)->next)

static inline int pci_enable_device(struct pci_dev *dev){return 0;}
#define pci_disable_device(x)

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
#include <linux/ioport.h>
#define PCI_DMA_FROMDEVICE              0
#define PCI_DMA_TODEVICE                0

static inline void *pci_alloc_consistent(struct pci_dev *hwdev, size_t size,
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

static inline void pci_free_consistent(struct pci_dev *hwdev, size_t size,
	void *vaddr, dma_addr_t dma_handle)
{
	free_pages((unsigned long)vaddr, get_order(size));
}

#define pci_map_single(cookie, address, size, dir)      virt_to_bus(address)
#define pci_unmap_single(cookie, address, size, dir)
#define pci_dma_sync_single(cookie, address, size, dir)

// for getting base addresses
static inline unsigned long pci_resource_start(struct pci_dev *dev, unsigned int bar)
{
	if(dev->base_address[bar] & PCI_BASE_ADDRESS_SPACE_IO)
		return dev->base_address[bar] & PCI_BASE_ADDRESS_IO_MASK;
	return dev->base_address[bar] & PCI_BASE_ADDRESS_MEM_MASK;
}

static inline unsigned long pci_resource_end(struct pci_dev *dev, unsigned int bar)
{
	return pci_resource_start(dev, bar);
}

static inline int pci_request_regions(struct pci_dev *dev, char *name)
{
	const int max_num_base_addr = 6;
	static const int fake_length = 1;
	int i;
	int retval = 0;

	for(i = 0; i < max_num_base_addr; i++)
	{
		if(dev->base_address[i])
		{
			if(dev->base_address[i] & PCI_BASE_ADDRESS_SPACE_IO)
				retval = check_region(pci_resource_start(dev, i),
					fake_length);
			if( retval )
				break;
		}
	}

	if(retval) return retval;

	for(i = 0; i < max_num_base_addr; i++)
	{
		if(dev->base_address[i])
		{
			if(dev->base_address[i] & PCI_BASE_ADDRESS_SPACE_IO)
				request_region(pci_resource_start(dev, i),
					fake_length, name);
		}
	}

	return 0;
}

static inline void pci_release_regions(struct pci_dev *dev)
{
	static const int max_num_base_addr = 6;
	static const int fake_length = 1;
	int i;

	for(i = 0; i < max_num_base_addr; i++)
	{
		if(dev->base_address[i])
		{
			if(dev->base_address[i] & PCI_BASE_ADDRESS_SPACE_IO)
				release_region(pci_resource_start(dev, i),
					fake_length);
		}
	}
}

static inline struct pci_dev* pci_find_subsys( unsigned int vendor, unsigned int device,
	unsigned int ss_vendor, unsigned int ss_device,
	const struct pci_dev *from)
{
	rt_printk( "pci_find_subsys() not supported in kernels older than 2.4!\n" );
	return NULL;
}

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,14)
static inline void pci_set_master(struct pci_dev *dev)
{ return; }
#endif	// 2.2.14

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,18)
static inline unsigned long pci_resource_len (struct pci_dev *dev, int n_base)
{ return 0; }
#endif	// 2.2.18

#endif

#endif /* _COMPAT_PCI_H */




