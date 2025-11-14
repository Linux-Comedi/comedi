/*
 * linux/pci.h compatibility header
 */

#ifndef _COMPAT_PCI_H
#define _COMPAT_PCI_H

#include_next <linux/pci.h>

#ifndef DEFINE_PCI_DEVICE_TABLE
#define DEFINE_PCI_DEVICE_TABLE(_table) \
	struct pci_device_id _table[]
#endif

#endif /* _COMPAT_PCI_H */
