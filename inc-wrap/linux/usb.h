/*
 * linux/usb.h compatibility header
*/

#ifndef COMEDI_COMPAT__LINUX__USB_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__USB_H__INCLUDED__

#include_next <linux/usb.h>

#include <linux/version.h>
#include <linux/time.h>
#include <linux/errno.h>

/*
 * usb_kill_urb() was added in kernel version 2.6.8.  Define a compatible
 * version for earlier kernels.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)

static inline void comedi_usb_kill_urb(struct urb *urb)
{
	if (!urb)
		return;
	urb->transfer_flags &= ~URB_ASYNC_UNLINK;
	usb_unlink_urb(urb);
}
#undef usb_kill_urb
#define usb_kill_urb comedi_usb_kill_urb

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8) */

/*
 * Since kernel version 2.6.12, USB timeouts have been specified in
 * milliseconds rather than jiffies.
 *
 * For older kernels, redefine usb_control_msg() and usb_bulk_msg() to 
 * use a timeout specified in milliseconds, and redefine the
 * USB_CTRL_GET_TIMEOUT and USB_CTRL_SET_TIMEOUT macros to be specified
 * in milliseconds.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12)
static inline int
comedi_usb_control_msg(struct usb_device *dev, unsigned int pipe,
	__u8 request, __u8 requesttype, __u16 value, __u16 index,
	void *data, __u16 size, int millisec_timeout)
{
	return usb_control_msg(dev, pipe, request, requesttype, value, index,
			       data, size, msecs_to_jiffies(millisec_timeout));
}
#undef usb_control_msg
#define usb_control_msg(dev, pipe, req, rt, val, i, dat, s, tout) \
	comedi_usb_control_msg(dev, pipe, req, rt, val, i, dat, s, tout)

static inline int
comedi_usb_bulk_msg(struct usb_device *usb_dev, unsigned int pipe,
	void *data, int len, int *actual_length, int millisec_timeout)
{
	return usb_bulk_msg(usb_dev, pipe, data, len, actual_length,
			    msecs_to_jiffies(millisec_timeout));
}
#undef usb_bulk_msg
#define usb_bulk_msg(dev, pipe, dat, len, actlen, tout) \
	comedi_usb_bulk_msg(dev, pipe, dat, len, actlen, tout)

#undef USB_CTRL_GET_TIMEOUT
#define USB_CTRL_GET_TIMEOUT 5000

#undef USB_CTRL_SET_TIMEOUT
#define USB_CTRL_SET_TIMEOUT 5000

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,12) */

/*
 * Determine whether we need the "owner" member of struct usb_driver and
 * define COMEDI_COMPAT_HAVE_USB_DRIVER_OWNER if we need it.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16)
#define COMEDI_COMPAT_HAVE_USB_DRIVER_OWNER
#endif

#endif
