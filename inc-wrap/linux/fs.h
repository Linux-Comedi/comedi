/*
 * linux/fs.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__FS_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__FS_H__INCLUDED__

#include_next <linux/fs.h>
#include <linux/version.h>

/*
 * The HAVE_COMPAT_IOCTL and HAVE_UNLOCKED_IOCTL macros were defined in
 * 2.6.11 but removed in 5.9 because nothing in the kernel source uses them,
 * but we do.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
#undef HAVE_COMPAT_IOCTL
#define HAVE_COMPAT_IOCTL 1
#undef HAVE_UNLOCKED_IOCTL
#define HAVE_UNLOCKED_IOCTL 1
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)

#undef file_inode
#define file_inode(f) comedi_file_inode(f)
static inline struct inode *comedi_file_inode(struct file *f)
{
	return f->f_dentry->d_inode;
}

#endif

#endif
