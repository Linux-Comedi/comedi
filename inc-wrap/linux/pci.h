/*
 * linux/pci.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__PCI_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__PCI_H__INCLUDED__

#include_next <linux/pci.h>
#include <linux/version.h>

#ifndef DEFINE_PCI_DEVICE_TABLE
#define DEFINE_PCI_DEVICE_TABLE(_table) \
	struct pci_device_id _table[]
#endif

/*
 * Prior to kernel version 2.6.17, the const qualifier is missing from the
 * res_name parameter of pci_request_region() and pci_request_regions().
 * Redefine them as macros discarding the const qualifier for these kernel
 * versions.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,17)

static inline int
comedi_pci_request_region(struct pci_dev *pdev, int bar, const char *res_name)
{
	return pci_request_region(pdev, bar, (char *)res_name);
}
#undef pci_request_region
#define pci_request_region(pdev, bar, res_name) \
	comedi_pci_request_region(pdev, bar, res_name)

static inline int
comedi_pci_request_regions(struct pci_dev *pdev, const char *res_name)
{
	return pci_request_regions(pdev, (char *)res_name);
}
#undef pci_request_regions
#define pci_request_regions(pdev, res_name) \
	comedi_pci_request_regions(pdev, res_name)

#endif

#endif
