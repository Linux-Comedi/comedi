/*
 * linux/firmware.h compatibility header
 */

#ifndef __COMPAT_LINUX_FIRMWARE_H

#include <linux/version.h>

#include_next <linux/firmware.h>

#ifdef CONFIG_COMEDI_REQUEST_FIRMWARE_NOWAIT_HAS_GFP

/* Define COMEDI_RELEASE_FIRMWARE_NOWAIT(fw) for use in the callback function
 * of request_firmware_nowait().  This version just calls
 * release_firmware(fw). */
#define COMEDI_RELEASE_FIRMWARE_NOWAIT(fw)	release_firmware(fw)

#else

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

/* Define COMEDI_RELEASE_FIRMWARE_NOWAIT(fw) for use in the callback function
 * of request_firmware_nowait().  This version does nothing. */
#define COMEDI_RELEASE_FIRMWARE_NOWAIT(fw)	do; while (0)
#endif

#endif
