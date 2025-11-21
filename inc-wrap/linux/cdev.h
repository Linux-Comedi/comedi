/*
 * linux/cdev.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__CDEV_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__CDEV_H__INCLUDED__

#include_next <linux/cdev.h>
#include <linux/version.h>

/*
 * The const qualifier was missing from *fops in cdev_init() prior to kernel
 * version 2.6.19.  Redefine it as a macro for those kernels.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)

static inline void
comedi_cdev_init(struct cdev *cdev, const struct file_operations *fops)
{
	return cdev_init(cdev, (struct file_operations *)fops);
}
#undef cdev_init
#define cdev_init(cdev, fops) comedi_cdev_init(cdev, fops)
#endif

#endif
