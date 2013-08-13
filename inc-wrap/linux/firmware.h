/*
 * linux/firmware.h compatibility header
 */

#ifndef __COMPAT_LINUX_FIRMWARE_H

#include <linux/comedi-config.h>

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
#ifdef FW_ACTION_HOTPLUG
	return request_firmware_nowait(
			module, uevent, name, device, context, cont);
#else
	return request_firmware_nowait(
			module, name, device, context, cont);
#endif
}

#undef request_firmware_nowait
#define request_firmware_nowait(module, uevent, name, device, gfp, context, cont) \
	comedi_internal_request_firmware_nowait( \
			module, uevent, name, device, gfp, context, cont)

/* Define COMEDI_RELEASE_FIRMWARE_NOWAIT(fw) for use in the callback function
 * of request_firmware_nowait().  This version does nothing. */
#define COMEDI_RELEASE_FIRMWARE_NOWAIT(fw)	do; while (0)
#endif

/*
 * FW_ACTION_HOTPLUG and FW_ACTION_NOHOTPLUG and the uevent/hotplug parameter
 * of request_firmware_nowait() were added in vanilla kernel 2.6.14.  We've
 * already dealt with the missing parameter above, but define the missing
 * values below.  Comedi drivers would normally use FW_ACTION_HOTPLUG.
 */
#ifndef FW_ACTION_HOTPLUG
#define FW_ACTION_NOHOTPLUG 0
#define FW_ACTION_HOTPLUG 1
#endif

#endif
