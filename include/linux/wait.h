
#ifndef __COMPAT_LINUX_WAIT_H
#define __COMPAT_LINUX_WAIT_H

#include <linux/version.h>

#include_next <linux/wait.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,18)
typedef struct wait_queue *wait_queue_head_t;
#define DECLARE_WAITQUEUE(x,y) struct wait_queue x={y,NULL}
#define init_waitqueue_head(x)
#endif

#endif

