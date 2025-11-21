/*
 * linux/pci.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__PCI_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__PCI_H__INCLUDED__

#include_next <linux/pci.h>

#ifndef DEFINE_PCI_DEVICE_TABLE
#define DEFINE_PCI_DEVICE_TABLE(_table) \
	struct pci_device_id _table[]
#endif

#endif
