/*
 * linux/usb/ch9.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__USB__CH9_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__USB__CH9_H__INCLUDED__

/*
 * This compatibility header is included by our linux/usb.h compatibility
 * header, but could be included from other places.
 */

#include <linux/version.h>

/*
 * The <linux/usb/ch9.h> header was added in kernel version 2.6.21.  Before
 * then, there was a <linux/usb_ch9.h> header.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,21)
#include_next <linux/usb/ch9.h>
#else
#include <linux/usb_ch9.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)

static inline int
comedi_usb_endpoint_dir_in(const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN);
}
#undef usb_endpoint_dir_in
#define usb_endpoint_dir_in(epd) comedi_usb_endpoint_dir_in(epd)

static inline int
comedi_usb_endpoint_dir_out(const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_OUT);
}
#undef usb_endpoint_dir_out
#define usb_endpoint_dir_out(epd) comedi_usb_endpoint_dir_out(epd)

static inline int
comedi_usb_endpoint_xfer_bulk(const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		USB_ENDPOINT_XFER_BULK);
}
#undef usb_endpoint_xfer_bulk
#define usb_endpoint_xfer_bulk(epd) comedi_usb_endpoint_xfer_bulk(epd)

static inline int
comedi_usb_endpoint_xfer_int(const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		USB_ENDPOINT_XFER_INT);
}
#undef usb_endpoint_xfer_int
#define usb_endpoint_xfer_int(epd) comedi_usb_endpoint_xfer_int(epd)

static inline int
comedi_usb_endpoint_xfer_isoc(const struct usb_endpoint_descriptor *epd)
{
	return ((epd->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		USB_ENDPOINT_XFER_ISOC);
}
#undef usb_endpoint_xfer_isoc
#define usb_endpoint_xfer_isoc(epd) comedi_usb_endpoint_xfer_isoc(epd)

static inline int
comedi_usb_endpoint_is_bulk_in(const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_bulk(epd) && usb_endpoint_dir_in(epd));
}
#undef usb_endpoint_is_bulk_in
#define usb_endpoint_is_bulk_in(epd) comedi_usb_endpoint_is_bulk_in(epd)

static inline int
comedi_usb_endpoint_is_bulk_out(const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_bulk(epd) && usb_endpoint_dir_out(epd));
}
#undef usb_endpoint_is_bulk_out
#define usb_endpoint_is_bulk_out(epd) comedi_usb_endpoint_is_bulk_out(epd)

static inline int
comedi_usb_endpoint_is_int_in(const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_int(epd) && usb_endpoint_dir_in(epd));
}
#undef usb_endpoint_is_int_in
#define usb_endpoint_is_int_in(epd) comedi_usb_endpoint_is_int_in(epd)

static inline int
comedi_usb_endpoint_is_int_out(const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_int(epd) && usb_endpoint_dir_out(epd));
}
#undef usb_endpoint_is_int_out
#define usb_endpoint_is_int_out(epd) comedi_usb_endpoint_is_int_out(epd)

static inline int
comedi_usb_endpoint_is_isoc_in(const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_isoc(epd) && usb_endpoint_dir_in(epd));
}
#undef usb_endpoint_is_isoc_in
#define usb_endpoint_is_isoc_in(epd) comedi_usb_endpoint_is_isoc_in(epd)

static inline int
comedi_usb_endpoint_is_isoc_out(const struct usb_endpoint_descriptor *epd)
{
	return (usb_endpoint_xfer_isoc(epd) && usb_endpoint_dir_out(epd));
}
#undef usb_endpoint_is_isoc_out
#define usb_endpoint_is_isoc_out(epd) comedi_usb_endpoint_is_isoc_out(epd)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19) */

#endif
