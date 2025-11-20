/*
 * linux/compiler.h compatibility header
 */

#ifndef _COMPAT_COMPILER_H
#define _COMPAT_COMPILER_H

#include_next <linux/compiler.h>

#ifndef __force
#define __force
#endif

#ifndef __iomem
#define __iomem
#endif

#ifndef __nocast
#define __nocast
#endif

#ifndef fallthrough
#define fallthrough	do {} while (0)	/* fallthrough */
#endif

#endif /* _COMPAT_COMPILER_H */
