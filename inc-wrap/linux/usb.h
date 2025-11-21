/*
 * linux/usb.h compatibility header
*/

#ifndef COMEDI_COMPAT__LINUX__USB_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__USB_H__INCLUDED__

#include_next <linux/usb.h>

#include <linux/version.h>
#include <linux/time.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12)
static inline int USB_CONTROL_MSG(struct usb_device *dev, unsigned int pipe,
	__u8 request, __u8 requesttype, __u16 value, __u16 index,
	void *data, __u16 size, int millisec_timeout)
{
	return usb_control_msg(dev, pipe, request, requesttype, value, index,
			       data, size,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12)
			       msecs_to_jiffies(millisec_timeout)
#else
			       millisec_timeout
#endif			    
		);
}
static inline int USB_BULK_MSG(struct usb_device *usb_dev, unsigned int pipe,
	void *data, int len, int *actual_length, int millisec_timeout)
{
	return usb_bulk_msg(usb_dev, pipe, data, len, actual_length,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12)
			    msecs_to_jiffies(millisec_timeout)
#else
			    millisec_timeout
#endif			    
		);
}
#else
#define USB_CONTROL_MSG usb_control_msg
#define USB_BULK_MSG usb_bulk_msg
#endif

/*
 * Determine whether we need the "owner" member of struct usb_driver and
 * define COMEDI_COMPAT_HAVE_USB_DRIVER_OWNER if we need it.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
#define COMEDI_COMPAT_HAVE_USB_DRIVER_OWNER
#endif

#endif
