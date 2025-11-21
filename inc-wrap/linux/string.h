/*
 * linux/string.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__STRING_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__STRING_H__INCLUDED__

#include_next <linux/string.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/errno.h>

/*
 * strscpy() was introduced in Linux kernel 4.3.0.  Define a compatibility
 * function for earlier kernels.
 *
 * Note that callers of strscpy between kernel versions 4.3.0 and 4.15.x
 * will get a warning about ignoring the return value of strscpy, but that
 * warning was dropped from later kernel versions.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,3,0)

#undef strscpy
#define strscpy(d, s, n) comedi_strscpy(d, s, n)

static inline ssize_t comedi_strscpy(char *d, const char *s, size_t n)
{
	const char *s_orig = s;
	char ch = 1;

	if (n) {
		for (ch = *s; --n && ch; ch = *s) {
			*d++ = ch;
			s++;
		}
		*d = '\0';
	}
	return ch ? -E2BIG : (ssize_t)(s - s_orig);
}

#endif

#endif
