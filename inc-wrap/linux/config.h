/*
 * linux/config.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__CONFIG_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__CONFIG_H__INCLUDED__

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)
#include_next <linux/config.h>
#endif

#endif
