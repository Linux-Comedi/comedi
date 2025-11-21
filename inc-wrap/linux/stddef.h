/*
 * linux/stddef.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__STDDEF_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__STDDEF_H__INCLUDED__

#include_next <linux/stddef.h>
#include <linux/comedi-config.h>

#ifndef COMEDI_COMPAT_HAVE_GENERIC_BOOL_TYPE
enum {
	false,
	true
};
#endif

#endif
