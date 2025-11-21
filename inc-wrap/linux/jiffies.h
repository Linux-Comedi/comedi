/*
 * linux/jiffies.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__JIFFIES_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__JIFFIES_H__INCLUDED__

#include_next <linux/jiffies.h>

#ifndef time_in_range
#define time_in_range(a, b, c)	(time_after_eq(a, b) && time_before_eq(a, c))
#endif

#ifndef time_in_range_open
#define time_in_range_open(a, b, c) (time_after_eq(a, b) && time_before(a, c))
#endif

#ifndef time_after64
#define time_after64(a, b)	((__s64)(b) - (__s64)(a) < 0)
#endif

#ifndef time_before64
#define time_before64(a, b)	time_after64(b, a)
#endif

#ifndef time_after_eq64
#define time_after_eq64(a, b)	((__s64)(a) - (__s64)(b) >= 0)
#endif

#ifndef time_before_eq64
#define time_before_eq64(a, b)	time_after_eq64(b, a)
#endif

#ifndef time_in_range64
#define time_in_range64(a, b, c) (time_after_eq64(a, b) && time_before_eq(a, c))
#endif


#ifndef time_is_before_jiffies
#define time_is_before_jiffies(a)	time_after(jiffies, a)
#endif

#ifndef time_is_before_jiffies64
#define time_is_before_jiffies64(a)	time_after64(get_jiffies_64(), a)
#endif

#ifndef time_is_after_jiffies
#define time_is_after_jiffies(a)	time_before(jiffies, a)
#endif

#ifndef time_is_after_jiffies64
#define time_is_after_jiffies64(a)	time_before64(get_jiffies_64(), a)
#endif

#ifndef time_is_before_eq_jiffies
#define time_is_before_eq_jiffies(a)	time_after_eq(jiffies, a)
#endif

#ifndef time_is_before_eq_jiffies64
#define time_is_before_eq_jiffies64(a)	time_after_eq64(get_jiffies_64(), a)
#endif

#ifndef time_is_after_eq_jiffies
#define time_is_after_eq_jiffies(a)	time_before_eq(jiffies, a)
#endif

#ifndef time_is_after_eq_jiffies64
#define time_is_after_eq_jiffies64(a)	time_before_eq64(get_jiffies_64(), a)
#endif

#endif
