/*
 * linux/fs.h compatibility header
 */

#ifndef __COMPAT_LINUX_FS_H_
#define __COMPAT_LINUX_FS_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020400
/* no fasync */
#define KILL_FASYNC(a,b,c)
#else
#if LINUX_VERSION_CODE < 0x020400
#define KILL_FASYNC(a,b,c)	kill_fasync((a),(c))
#else
#define KILL_FASYNC(a,b,c)	kill_fasync(&(a),(b),(c))
#endif
#endif

#include_next <linux/fs.h>

#endif

