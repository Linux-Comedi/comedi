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

#else

#endif
#endif


#endif /* _COMPAT_PCI_H */




