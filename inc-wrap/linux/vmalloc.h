/*
 * linux/vmalloc.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__VMALLOC_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__VMALLOC_H__INCLUDED__

#include_next <linux/vmalloc.h>
#include <linux/version.h>
#include <linux/string.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)

#undef vzalloc
#define vzalloc(size) comedi_vzalloc(size)
static inline void *comedi_vzalloc(size_t size)
{
	void *ret = vmalloc(size);
	if (ret)
		memset(ret, 0, size);
	return ret;
}
#endif

#endif
