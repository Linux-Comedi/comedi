/*
    linux/types.h compatibility header

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __COMPAT_LINUX_TYPES_H_
#define __COMPAT_LINUX_TYPES_H_

#include <linux/version.h>
#include <linux/comedi-config.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
typedef unsigned long resource_size_t;
#else
/* resource_size_t is either u32 or u64, depending on CONFIG_RESOURCES_64BIT */
#endif

#ifndef COMEDI_COMPAT_HAVE_GENERIC_BOOL_TYPE
typedef _Bool bool;
#endif

#include_next <linux/types.h>

#endif
