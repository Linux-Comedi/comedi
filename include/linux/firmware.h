/*
 * linux/firmware.h compatibility header
 */

#ifndef __COMPAT_LINUX_FIRMWARE_H

#include <linux/version.h>

#include_next <linux/firmware.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)

/* Redefine request_firmware_nowait() to add gfp parameter.  This will be
 * ignored for kernel versions prior to 2.6.33. */
static inline int comedi_internal_request_firmware_nowait(
	struct module *module, int uevent,
	const char *name, struct device *device, unsigned gfp, void *context,
	void (*cont)(const struct firmware *fw, void *context))
{
	return request_firmware_nowait(
			module, uevent, name, device, context, cont);
}

#undef request_firmware_nowait
#define request_firmware_nowait(module, uevent, name, device, gfp, context, cont) \
	comedi_internal_request_firmware_nowait( \
			module, uevent, name, device, gfp, context, cont)

#endif

#endif
