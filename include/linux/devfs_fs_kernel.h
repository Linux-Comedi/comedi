/*
 * linux/devfs_fs_kernel.h compatibility header
 */

#ifndef __COMPAT_LINUX_DEVFS_FS_KERNEL_H_
#define __COMPAT_LINUX_DEVFS_FS_KERNEL_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 2, 18)

#define DEVFS_FL_DEFAULT 0
#define DEVFS_SPECIAL_CHR 0

typedef struct devfs_entry * devfs_handle_t;

static inline int devfs_register_chrdev (unsigned int major, const char *name,
	struct file_operations *fops)
{
	return register_chrdev (major, name, fops);
}   

static inline int devfs_unregister_chrdev (unsigned int major,const char *name)
{
	return unregister_chrdev (major, name);
}

static inline devfs_handle_t devfs_register (devfs_handle_t dir,
	const char *name, unsigned int flags, unsigned int major,
	unsigned int minor, umode_t mode, void *ops, void *info)
{
	return NULL;
}

static inline void devfs_unregister (devfs_handle_t de)
{
}

static inline devfs_handle_t devfs_get_handle (devfs_handle_t dir,
	const char *name, unsigned int major, unsigned int minor,
	char type, int traverse_symlinks)
{
	    return NULL;
}

static inline devfs_handle_t devfs_find_handle (devfs_handle_t dir,
	const char *name, unsigned int major, unsigned int minor,
	char type, int traverse_symlinks)
{
	    return NULL;
}

#else
#include_next <linux/devfs_fs_kernel.h>
#endif

#endif

