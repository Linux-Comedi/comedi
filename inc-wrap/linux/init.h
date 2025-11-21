/*
 * linux/init.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__INIT_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__INIT_H__INCLUDED__

#include_next <linux/init.h>

#ifndef __devinit
#define __devinit
#endif

#ifndef __devinitdata
#define __devinitdata
#endif

#ifndef __devinitconst
#define __devinitconst __devinitdata
#endif

#ifndef __devexit
#define __devexit
#endif

#ifndef __devexitdata
#define __devexitdata
#endif

#ifndef __devexitconst
#define __devexitconst __devexitdata
#endif

#ifndef __devexit_p
#define __devexit_p(x) x
#endif

#endif
