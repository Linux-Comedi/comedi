/*
 * linux/printk.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__PRINTK_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__PRINTK_H__INCLUDED__

/*
 * This compatibility header is included by our linux/kernel.h compatibility
 * header, but could be included from other places.
 */

#include <linux/version.h>

/* The <linux/printk.h> header was added in kernel version 2.6.37. */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37)
#include_next <linux/printk.h>
#endif

#include <linux/compiler.h>

#ifndef KERN_CONT
#define KERN_CONT	""
#endif

#ifndef KERN_DEFAULT
#define KERN_DEFAULT	""
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static inline __printf(1, 2)
int comedi_no_printk(const char *fmt, ...)
{
	return 0;
}
#undef no_printk
#define no_printk comedi_no_printk
#endif

/* Driver code can define pr_fmt(fmt) itself to override this. */
#ifndef pr_fmt
#define pr_fmt(fmt) fmt
#endif

/*
 * Undefine the pr_ macros that use pr_fmt(fmt) for older kernel versions
 * so they will be redefined to use pr_fmt(fmt) below.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,28)
#undef pr_emerg
#undef pr_alert
#undef pr_crit
#undef pr_err
#undef pr_warning
#undef pr_warn
#undef pr_notice
#undef pr_info
#undef pr_devel
#undef pr_debug
#undef pr_emerg_once
#undef pr_alert_once
#undef pr_crit_once
#undef pr_err_once
#undef pr_warn_once
#undef pr_info_once
#undef pr_devel_once
#undef pr_debug_once
#endif

/* Note: using old GCC variadic macro syntax for the following... */
#ifndef pr_emerg
#define pr_emerg(fmt, args...) \
	printk(KERN_EMERG pr_fmt(fmt), ##args)
#endif

#ifndef pr_alert
#define pr_alert(fmt, args...) \
	printk(KERN_ALERT pr_fmt(fmt), ##args)
#endif

#ifndef pr_crit
#define pr_crit(fmt, args...) \
	printk(KERN_CRIT pr_fmt(fmt), ##args)
#endif

#ifndef pr_err
#define pr_err(fmt, args...) \
	printk(KERN_ERR pr_fmt(fmt), ##args)
#endif

#ifndef pr_warning
#define pr_warning(fmt, args...) \
	printk(KERN_WARNING pr_fmt(fmt), ##args)
#endif

#ifndef pr_warn
#define pr_warn pr_warning
#endif

#ifndef pr_notice
#define pr_notice(fmt, args...) \
	printk(KERN_NOTICE pr_fmt(fmt), ##args)
#endif

#ifndef pr_info
#define pr_info(fmt, args...) \
	printk(KERN_INFO pr_fmt(fmt), ##args)
#endif

#ifndef pr_cont
/* This doesn't use pr_fmt(fmt) as it's a continuation of previous message. */
#define pr_cont(fmt, args...) \
	printk(KERN_CONT fmt, ##args)
#endif

#ifndef pr_devel
/* pr_devel() should produce zero code unless DEBUG is defined */
#ifdef DEBUG
#define pr_devel(fmt, args...) \
	printk(KERN_DEBUG pr_fmt(fmt), ##args)
#else
#define pr_devel(fmt, args...) \
	no_printk(KERN_DEBUG pr_fmt(fmt), ##args)
#endif
#endif

#ifndef pr_debug
#define pr_debug pr_devel
#endif

#ifndef printk_once
#define printk_once(fmt, args...)		\
({						\
	static int __print_once;		\
						\
	if (!__print_once) {			\
		__print_once = 1;		\
		printk(fmt, ##args);		\
	}					\
})
#endif

#ifndef pr_emerg_once
#define pr_emerg_once(fmt, args...) \
	printk_once(KERN_EMERG pr_fmt(fmt), ##args)
#endif

#ifndef pr_alert_once
#define pr_alert_once(fmt, args...) \
	printk_once(KERN_ALERT pr_fmt(fmt), ##args)
#endif

#ifndef pr_crit_once
#define pr_crit_once(fmt, args...) \
	printk_once(KERN_CRIT pr_fmt(fmt), ##args)
#endif

#ifndef pr_err_once
#define pr_err_once(fmt, args...) \
	printk_once(KERN_ERR pr_fmt(fmt), ##args)
#endif

#ifndef pr_warn_once
#define pr_warn_once(fmt, args...) \
	printk_once(KERN_WARNING pr_fmt(fmt), ##args)
#endif

#ifndef pr_notice_once
#define pr_notice_once(fmt, args...) \
	printk_once(KERN_NOTICE pr_fmt(fmt), ##args)
#endif

#ifndef pr_info_once
#define pr_info_once(fmt, args...) \
	printk_once(KERN_INFO pr_fmt(fmt), ##args)
#endif

#ifndef pr_cont_once
/* This doesn't use pr_fmt(fmt) as it's a continuation of previous message. */
#define pr_cont_once(fmt, args...) \
	printk_once(KERN_CONT fmt, ##args)
#endif

#ifndef pr_devel_once
/* pr_devel_once() should produce zero code unless DEBUG is defined */
#ifdef DEBUG
#define pr_devel_once(fmt, args...) \
	printk_once(KERN_DEBUG pr_fmt(fmt), ##args)
#else
#define pr_devel_once(fmt, args...) \
	no_printk(KERN_DEBUG pr_fmt(fmt), ##args)
#endif
#endif

#ifndef pr_debug_once
#define pr_debug_once pr_devel_once
#endif

#endif
