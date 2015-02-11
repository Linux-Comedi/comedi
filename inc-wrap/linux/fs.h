
#ifndef __COMPAT_LINUX_FS_H
#define __COMPAT_LINUX_FS_H

#include <linux/version.h>
#include_next <linux/fs.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)

#undef file_inode
#define file_inode(f) comedi_file_inode(f)
static inline struct inode *comedi_file_inode(struct file *f)
{
	return f->f_dentry->d_inode;
}

#endif

#endif
