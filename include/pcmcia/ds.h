/*
 * pcmcia/ds.h compatibility header
 */

#ifndef __COMPAT_PCMCIA_DS_H_
#define __COMPAT_PCMCIA_DS_H_

#include <linux/version.h>
#include_next <pcmcia/ds.h>
#include <linux/device.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
struct pcmcia_driver {
	dev_link_t              *(*attach)(void);
	void                    (*detach)(dev_link_t *);
	struct module           *owner;
	struct device_driver    drv;
};

/* driver registration */
static inline int pcmcia_register_driver(struct pcmcia_driver *driver)
{
	return register_pccard_driver(driver->drv.name, driver->attach, driver->detach);
}

static void inline pcmcia_unregister_driver(struct pcmcia_driver *driver)
{
	unregister_pccard_driver(driver->drv.name);
}


#endif

#endif // __COMPAT_PCMCIA_DS_H_

