/*
 * linux/poll.h compatibility header
 */

#ifndef __COMPAT_LINUX_POLL_H_
#define __COMPAT_LINUX_POLL_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020200

#else
#include_next <linux/poll.h>
#endif

#endif

