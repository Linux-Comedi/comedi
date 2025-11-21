/*
 * linux/compiler.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__COMPILER_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__COMPILER_H__INCLUDED__

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

#endif
