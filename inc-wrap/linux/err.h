/*
 * linux/err.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__ERR_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__ERR_H__INCLUDED__

/*
 * Debian patches for 2.6.8 kernels modify the IS_ERR() function to use the 
 * unlikely(x) macro, but it does not include <linux/compiler.h> to define
 * the macro.  Do so here to avoid declaring unlikely(x) as a function
 * implicitly.
 */
#include <linux/compiler.h>
#include_next <linux/err.h>

#endif
