/*
 * moduleparam.h compatibility header
 */

#ifndef _COMPAT_MODULEPARAM_H
#define _COMPAT_MODULEPARAM_H

#include <linux/version.h>
#include_next <linux/moduleparam.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10)
/*
 * 2.6 kernels before 2.6.10 use an unsigned int lvalue for third parameter of
 * module_param_array() instead of the pointer to unsigned int used in
 * 2.6.10 onwards.  Comedi code goes with 2.6.10, so cope with the change here.
 * To make things more difficult, the pointer can be null and we need to
 * substitute that with a pointer to a unique lvalue.
 */

#undef module_param_array_named
#define module_param_array_named(name, array, type, nump, perm)		\
	static unsigned int __missing_num_##name;			\
	static struct kparam_array __param_arr_##name			\
	= { ARRAY_SIZE(array), nump ?: &__missing_num_##name,		\
	    param_set_##type, param_get_##type,				\
	    sizeof(array[0]), array };					\
	module_param_call(name, param_array_set, param_array_get,	\
		&__param_arr_##name, perm)

#undef module_param_array
#define module_param_array(name, type, nump, perm)		\
	module_param_array_named(name, name, type, nump, perm)

#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10) */

/*
 * Define a Comedi-specific macro MODULE_PARAM_ARRAY(name,type,len,perm)
 * supported on 2.4 and 2.6 kernels, because
 * module_param_array(name,type,nump,perm) * is only available for 2.6.
 *
 * name - Name of module parameter and array variable.
 * type - Module parameter type: byte, short, ushort, int, uint, long, ulong,
 *        bool, charp.  Note that invbool is not supported.
 * len  - Length of array variable; needed for 2.4 kernel; ignored for 2.6
 *        kernel.  Note that this must be specified as a sequence of decimal
 *        digits or a macro that expands to a sequence of decimal digits.
 *        Using ARRAY_SIZE() or sizeof() is not allowed.
 * perm - Permissions for module parameter in sysfs, e.g. 0444.  Only used
 *        for 2.6 kernels.
 */
#ifdef module_param_array
#define MODULE_PARAM_ARRAY(name,type,len,perm) \
	module_param_array(name,type,0,perm)
#else
#define MODULE_PARAM_ARRAY(name,type,len,perm) \
	static inline void *__check_existence_##name(void) \
		{ return &name[0]; } \
	MODULE_PARM(name, "1-" __MODULE_STRING(len) _MODULE_PARM_STRING_##type)
#endif /* module_param_array */

/*
 * Define a type for 'bool' parameters....
 */
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31))
/* Need to use 'int' or 'unsigned int' for 'bool' module parameter. */
typedef int COMEDI_MODULE_PARAM_BOOL_T;
#else
/* Can use 'int', 'unsigned int' or 'bool' for 'bool' module parameter, but
 * only 'bool' will be allowed for kernel version 3.4 onwards.  Using 'int'
 * or 'unsigned int' results in a warning for kernel version 3.3. */
typedef bool COMEDI_MODULE_PARAM_BOOL_T;
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,31)) */

#endif /* _COMPAT_MODULEPARAM_H */
