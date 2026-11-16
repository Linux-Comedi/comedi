/*
 * linux/wait.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__WAIT_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__WAIT_H__INCLUDED__

#include_next <linux/wait.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,5,0)

#undef wq_has_sleeper
#define wq_has_sleeper(wq_head) comedi_wq_has_sleeper(wq_head)
static inline bool comedi_wq_has_sleeper(wait_queue_head_t *wq_head)
{
	smp_mb();
	return waitqueue_active(wq_head);
}

#endif

#endif
