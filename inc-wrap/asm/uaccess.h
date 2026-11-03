/*
 * asm/uaccess.h compatibility header
 */

#ifndef COMEDI_COMPAT__ASM__UACCESS_H__INCLUDED__
#define COMEDI_COMPAT__ASM__UACCESS_H__INCLUDED__

#include_next <asm/uaccess.h>

/*
 * The __get_user() and __put_user() macros were removed in kernel version 7.4.
 * If they are not defined, define them to call the get_user() and put_user()
 * macros.
 */
#ifndef __get_user
#define __get_user(x, ptr) get_user(x, ptr)
#endif
#ifndef __put_user
#define __put_user(x, ptr) put_user(x, ptr)
#endif

#endif
