/*
 * linux/device.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__DEVICE_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__DEVICE_H__INCLUDED__

#include_next <linux/device.h>
#include <linux/version.h>

/*
 * Notes:
 *
 * Call COMEDI_DEVICE_CREATE() instead of device_create() for compatibility
 * with kernel versions prior to 2.6.27.  This returns a PTR_ERR() value or a
 * 'comedi_device_create_t *' which will be a 'struct class_device *' for
 * kernel versions prior to 2.6.19, and a 'struct device *' for kernel version
 * 2.6.19 onwards.  Call COMEDI_DEVICE_DESTROY() to destroy it.
 *
 * The result of COMEDI_DEVICE_CREATE() can be used to set or get a private
 * data pointer, using COMEDI_DEV_SET_DRVDATA() and COMEDI_DEV_GET_DRVDATA().
 *
 * The result of COMEDI_DEVICE_CREATE() can be used to create or remove
 * sysfs attributes, using COMEDI_DECLARE_ATTR_SHOW() to declare the "show
 * attribute" function, COMEDI_DECLARE_ATTR_STORE() macro to declare the
 * "store attribute" function, COMEDI_DEVICE_CREATE_FILE() to create the
 * attribute, and COMEDI_DEVICE_REMOVE_FILE() to remove it.
 *
 * The 'drvdata' parameter of COMEDI_DEVICE_CREATE() doesn't work for kernel
 * versions prior to 2.6.26, so please don't use it!
 *
 * The API of class_create() changed in kernel version 6.4 to remove the
 * first parameter of type 'struct module *owner'. This compatibility
 * layer redefines class_create() to expect a single parameter.  For kernel
 * versions prior to 6.4, it uses 'THIS_MODULE' as the owner.
 *
 *
 * None of the above is currently supported for 2.4 kernels!
 */

#include <linux/module.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

#define class_create(name) \
	(struct class *)class_simple_create(THIS_MODULE, name)
#define class_destroy(cs) \
	class_simple_destroy((struct class_simple *)(cs))

#else // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,4,0)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)

static inline struct class *
comedi_internal_class_create(struct module *owner, const char *name)
{
	/*
	 * Cast away the const qualifier to match function prototype for
	 * kernel versions prior to 2.6.19.
	 */
	return class_create(owner, (char *)name);
}

/* Also define it as a macro, tested for later. */
#define comedi_internal_class_create(owner, name)	\
	comedi_internal_class_create(owner, name)

#else // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)

/*
 * Red Hat back-ported the 6.4 version of class_create() to their later
 * 5.14 kernels.  Detect this by the fact that the class_create macro
 * will not be defined as a result of the back-port.
 */
#ifdef class_create

/* class_create was not back-ported from 6.4. */
#define comedi_internal_class_create(owner, name)	\
({							\
	static struct lock_class_key __key;		\
	__class_create(owner, name, &__key);		\
})

#endif // ifdef class_create

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)

/*
 * If comedi_internal_class_create() was defined above, redefine
 * class_create() to use single parameter and use THIS_MODULE as
 * the owner for earlier kernels.
 */
#ifdef comedi_internal_class_create

#undef class_create
#define class_create(name) comedi_internal_class_create(THIS_MODULE, name)

#endif // ifdef comedi_internal_class_create

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(6,4,0)

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)

/* Use 'struct class_device' before kernel 2.6.19. */

typedef struct class_device comedi_device_create_t;
typedef struct class_device_attribute comedi_device_attribute_t;
#define COMEDI_DEV_SET_DRVDATA(csdev, data)	class_set_devdata(csdev, data)
#define COMEDI_DEV_GET_DRVDATA(csdev)		class_get_devdata(csdev)
#define COMEDI_DEVICE_CREATE_FILE(csdev, attr)	class_device_create_file(csdev, attr)
#define COMEDI_DEVICE_REMOVE_FILE(csdev, attr)	class_device_remove_file(csdev, attr)
#define COMEDI_DECLARE_ATTR_SHOW(func, dev, buf) \
ssize_t func(struct class_device *dev, char *buf)
#define COMEDI_DECLARE_ATTR_STORE(func, dev, buf, count) \
ssize_t func(struct class_device *dev, const char *buf, size_t count)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

#define COMEDI_DEVICE_CREATE(cs, parent, devt, drvdata, device, fmt...) \
	class_simple_device_add((struct class_simple *)(cs), \
		devt, device, fmt)
#define COMEDI_DEVICE_DESTROY(cs, devt) \
	class_simple_device_remove(devt)

#else // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)

#define COMEDI_DEVICE_CREATE(cs, parent, devt, drvdata, device, fmt...) \
	class_device_create(cs, devt, device, fmt)
#define COMEDI_DEVICE_DESTROY(cs, devt) \
	class_device_destroy(cs, devt)

#else // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)

#define COMEDI_DEVICE_CREATE(cs, parent, devt, drvdata, device, fmt...) \
	class_device_create(cs, parent, devt, device, fmt)
#define COMEDI_DEVICE_DESTROY(cs, devt) \
	class_device_destroy(cs, devt)

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15)

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,13)

#else // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)

/* Use 'struct device' from kernel 2.6.19 onwards. */

typedef struct device comedi_device_create_t;
typedef struct device_attribute comedi_device_attribute_t;
#define COMEDI_DEV_SET_DRVDATA(csdev, data)	dev_set_drvdata(csdev, data)
#define COMEDI_DEV_GET_DRVDATA(csdev)		dev_get_drvdata(csdev)
#define COMEDI_DEVICE_CREATE_FILE(csdev, attr)	device_create_file(csdev, attr)
#define COMEDI_DEVICE_REMOVE_FILE(csdev, attr)	device_remove_file(csdev, attr)
#define COMEDI_DECLARE_ATTR_SHOW(func, dev, buf) \
ssize_t func(struct device *dev, struct device_attribute *_attr, char *buf)
#define COMEDI_DECLARE_ATTR_STORE(func, dev, buf, count) \
ssize_t func(struct device *dev, struct device_attribute *_attr, \
	const char *buf, size_t count)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)

#define COMEDI_DEVICE_CREATE(cs, parent, devt, drvdata, device, fmt...) \
	device_create(cs, ((parent) ? (parent) : (device)), devt, fmt)
#define COMEDI_DEVICE_DESTROY(cs, devt) \
	device_destroy(cs, devt)

#else // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)

#define COMEDI_DEVICE_CREATE(cs, parent, devt, drvdata, device, fmt...) \
	device_create_drvdata(cs, ((parent) ? (parent) : (device)), devt, drvdata, fmt)
#define COMEDI_DEVICE_DESTROY(cs, devt) \
	device_destroy(cs, devt)

#else // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)

#define COMEDI_DEVICE_CREATE(cs, parent, devt, drvdata, device, fmt...) \
	device_create(cs, ((parent) ? (parent) : (device)), devt, drvdata, fmt)
#define COMEDI_DEVICE_DESTROY(cs, devt) \
	device_destroy(cs, devt)

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)

/*
 * Since Linux kernel version 2.6.10, put_device(dev) has accepted a null
 * pointer.  Support that for earlier kernel versions.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
static inline void comedi_compat_put_device(struct device *dev)
{
	if (dev)
		put_device(dev);
}
#undef put_device
#define put_device(dev) comedi_compat_put_device(dev)

#endif // LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)

#endif
