/*
 * linux/pci_ids.h compatibility header
 */

#ifndef LINUX_COMPAT__LINUX__PCI_IDS_H__INCLUDED__
#define LINUX_COMPAT__LINUX__PCI_IDS_H__INCLUDED__

#include_next <linux/pci_ids.h>

#ifndef PCI_VENDOR_ID_AMCC
#define PCI_VENDOR_ID_AMCC	0x10e8
#endif

#ifndef PCI_VENDOR_ID_CBOARDS
#define PCI_VENDOR_ID_CBOARDS	0x1307
#endif

#ifndef PCI_VENDOR_ID_QUANCOM
#define PCI_VENDOR_ID_QUANCOM	0x8008
#endif

#ifndef PCI_DEVICE_ID_QUANCOM_GPIB
#define PCI_DEVICE_ID_QUANCOM_GPIB	0x3302
#endif

#endif
