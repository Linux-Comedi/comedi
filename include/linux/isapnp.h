/*
 * linux/fisapnp.h compatibility header
 */

#ifndef __COMPAT_LINUX_ISAPNP_H_
#define __COMPAT_LINUX_ISAPNP_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE >= 0x020300
#include_next <linux/isapnp.h>
#else
/* lowlevel configuration */
static inline int isapnp_present(void) { return 0; }
static inline int isapnp_cfg_begin(int csn, int device) { return -ENODEV; }
static inline int isapnp_cfg_end(void) { return -ENODEV; }
static inline unsigned char isapnp_read_byte(unsigned char idx) { return 0xff; }
static inline unsigned short isapnp_read_word(unsigned char idx) { return 0xffff; }
static inline unsigned int isapnp_read_dword(unsigned char idx) { return 0xffffffff; }
static inline void isapnp_write_byte(unsigned char idx, unsigned char val) { ; }
static inline void isapnp_write_word(unsigned char idx, unsigned short val) { ; }
static inline void isapnp_write_dword(unsigned char idx, unsigned int val) { ; }
static inline void isapnp_wake(unsigned char csn) { ; }
static inline void isapnp_device(unsigned char device) { ; }
static inline void isapnp_activate(unsigned char device) { ; }
static inline void isapnp_deactivate(unsigned char device) { ; }
/* manager */
static inline struct pci_bus *isapnp_find_card(unsigned short vendor,
	unsigned short device,
	struct pci_bus *from) { return NULL; }
static inline struct pci_dev *isapnp_find_dev(struct pci_bus *card,
	unsigned short vendor,
	unsigned short function,
	struct pci_dev *from) { return NULL; }
static inline int isapnp_probe_cards(const struct isapnp_card_id *ids,
	int (*probe)(struct pci_bus *card,
	const struct isapnp_card_id *id)) { return -ENODEV; }
static inline int isapnp_probe_devs(const struct isapnp_device_id *ids,
	int (*probe)(struct pci_dev *dev,
	const struct isapnp_device_id *id)) { return -ENODEV; }
static inline void isapnp_resource_change(struct resource *resource,
	unsigned long start,
	unsigned long size) { ; }
static inline int isapnp_activate_dev(struct pci_dev *dev, const char *name) { return -ENODEV; }
static inline int isapnp_register_driver(struct isapnp_driver *drv) { return 0; }
static inline void isapnp_unregister_driver(struct isapnp_driver *drv) { }
#endif

#endif

