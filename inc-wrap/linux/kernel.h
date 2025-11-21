/*
 * linux/kernel.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX___KERNEL_H__INCLUDED__
#define COMEDI_COMPAT__LINUX___KERNEL_H__INCLUDED__

#include_next <linux/kernel.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25)
/* Add missing strict_strtox functions. */
#include <linux/string.h>
#include <linux/errno.h>

static inline int comedi_strict_strtoul(const char *cp, unsigned int base,
	unsigned long *res)
{
	char *tail;
	unsigned long val;
	size_t len;

	*res = 0;
	len = strlen(cp);
	if (len == 0)
		return -EINVAL;

	val = simple_strtoul(cp, &tail, base);
	if (tail == cp)
		return -EINVAL;
	if ((*tail == '\0') ||
		((len == (size_t)(tail - cp) + 1) && (*tail == '\n'))) {
		*res = val;
		return 0;
	}

	return -EINVAL;
}

#undef strict_strtoul
#define strict_strtoul(cp, base, res) comedi_strict_strtoul(cp, base, res)

static inline int comedi_strict_strtol(const char *cp, unsigned int base,
	long *res)
{
	int ret;
	if (*cp == '-') {
		ret = comedi_strict_strtoul(cp + 1, base, (unsigned long *)res);
		if (!ret)
			*res = -(*res);
	} else {
		ret = comedi_strict_strtoul(cp, base, (unsigned long *)res);
	}

	return ret;
}

#undef strict_strtol
#define strict_strtol(cp, base, res) comedi_strict_strtoul(cp, base, res)

static inline int comedi_strict_strtoull(const char *cp, unsigned int base,
	unsigned long long *res)
{
	char *tail;
	unsigned long long val;
	size_t len;

	*res = 0;
	len = strlen(cp);
	if (len == 0)
		return -EINVAL;

	val = simple_strtoull(cp, &tail, base);
	if (tail == cp)
		return -EINVAL;
	if ((*tail == '\0') ||
		((len == (size_t)(tail - cp) + 1) && (*tail == '\n'))) {
		*res = val;
		return 0;
	}

	return -EINVAL;
}

#undef strict_strtoull
#define strict_strtoull(cp, base, res) comedi_strict_strtoul(cp, base, res)

static inline int comedi_strict_strtoll(const char *cp, unsigned int base,
	long long *res)
{
	int ret;
	if (*cp == '-') {
		ret = comedi_strict_strtoull(cp + 1, base, (unsigned long long *)res);
		if (!ret)
			*res = -(*res);
	} else {
		ret = comedi_strict_strtoull(cp, base, (unsigned long long *)res);
	}

	return ret;
}

#undef strict_strtoll
#define strict_strtoll(cp, base, res) comedi_strict_strtoul(cp, base, res)

#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39)
#undef kstrtol
#define kstrtol(s, base, res)	strict_strtol(s, base, res)
#undef kstrtoul
#define kstrtoul(s, base, res)	strict_strtoul(s, base, res)
#undef kstrtoll
#define kstrtoll(s, base, res)	strict_strtoll(s, base, res)
#undef kstrtoull
#define kstrtoull(s, base, res)	strict_strtoull(s, base, res)
#endif

#ifndef KERN_CONT
#define KERN_CONT	""
#endif

#ifndef KERN_DEFAULT
#define KERN_DEFAULT	""
#endif

#endif
