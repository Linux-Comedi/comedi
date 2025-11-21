/*
 * linux/semaphore.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__SEMAPHORE_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__SEMAPHORE_H__INCLUDED__

#include <linux/comedi-config.h>

#ifdef COMEDI_COMPAT_HAVE_LINUX_SEMAPHORE_H

#include_next <linux/semaphore.h>

#else /* COMEDI_COMPAT_HAVE_LINUX_SEMAPHORE_H */

#include <asm/semaphore.h>

#endif /* COMEDI_COMPAT_HAVE_LINUX_SEMAPHORE_H */

#endif
