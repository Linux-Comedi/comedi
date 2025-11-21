/*
 * linux/sched/signal.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__SCHED__SIGNAL_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__SCHED__SIGNAL_H__INCLUDED__

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,11,0)
#include <linux/sched.h>
#else
#include_next <linux/sched/signal.h>
#endif

#endif
