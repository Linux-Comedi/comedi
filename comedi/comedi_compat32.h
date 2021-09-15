/*
    comedi/comedi_compat32.h
    32-bit ioctl compatibility for 64-bit comedi kernel module.

    Author: Ian Abbott, MEV Ltd. <abbotti@mev.co.uk>
    Copyright (C) 2007 MEV Ltd. <http://www.mev.co.uk/>

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2007 David A. Schleef <ds@schleef.org>

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

#ifndef _COMEDI_COMPAT32_H
#define _COMEDI_COMPAT32_H

#include <linux/compat.h>
#include <linux/fs.h>	/* For HAVE_COMPAT_IOCTL and HAVE_UNLOCKED_IOCTL */

#ifdef CONFIG_COMPAT

#define COMEDI32_CHANINFO _IOR(CIO,3,comedi32_chaninfo)
#define COMEDI32_RANGEINFO _IOR(CIO,8,comedi32_rangeinfo)
/* N.B. COMEDI32_CMD and COMEDI_CMD ought to use _IOWR, not _IOR.
 * It's too late to change it now, but it only affects the command number. */
#define COMEDI32_CMD _IOR(CIO,9,comedi32_cmd)
/* N.B. COMEDI32_CMDTEST and COMEDI_CMDTEST ought to use _IOWR, not _IOR.
 * It's too late to change it now, but it only affects the command number. */
#define COMEDI32_CMDTEST _IOR(CIO,10,comedi32_cmd)
#define COMEDI32_INSNLIST _IOR(CIO,11,comedi32_insnlist)
#define COMEDI32_INSN _IOR(CIO,12,comedi32_insn)

typedef struct comedi32_chaninfo_struct {
	unsigned int subdev;
	compat_uptr_t maxdata_list;	/* 32-bit 'lsampl_t *' */
	compat_uptr_t flaglist;		/* 32-bit 'unsigned int *' */
	compat_uptr_t rangelist;	/* 32-bit 'unsigned int *' */
	unsigned int unused[4];
} comedi32_chaninfo;

typedef struct comedi32_rangeinfo_struct {
	unsigned int range_type;
	compat_uptr_t range_ptr;	/* 32-bit 'void *' */
} comedi32_rangeinfo;

typedef struct comedi32_cmd_struct {
	unsigned int subdev;
	unsigned int flags;
	unsigned int start_src;
	unsigned int start_arg;
	unsigned int scan_begin_src;
	unsigned int scan_begin_arg;
	unsigned int convert_src;
	unsigned int convert_arg;
	unsigned int scan_end_src;
	unsigned int scan_end_arg;
	unsigned int stop_src;
	unsigned int stop_arg;
	compat_uptr_t chanlist;		/* 32-bit 'unsigned int *' */
	unsigned int chanlist_len;
	compat_uptr_t data;		/* 32-bit 'sampl_t *' */
	unsigned int data_len;
} comedi32_cmd;

typedef struct comedi32_insn_struct {
	unsigned int insn;
	unsigned int n;
	compat_uptr_t data;		/* 32-bit 'lsampl_t *' */
	unsigned int subdev;
	unsigned int chanspec;
	unsigned int unused[3];
} comedi32_insn;

typedef struct comedi32_insnlist_struct {
	unsigned int n_insns;
	compat_uptr_t insns;		/* 32-bit 'comedi_insn *' */
} comedi32_insnlist;

#ifdef HAVE_COMPAT_IOCTL

#define comedi_register_ioctl32() do{}while(0)
#define comedi_unregister_ioctl32() do{}while(0)

#else /* HAVE_COMPAT_IOCTL */

extern long comedi_compat_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg);

extern void comedi_register_ioctl32(void);
extern void comedi_unregister_ioctl32(void);

#endif /* HAVE_COMPAT_IOCTL */

#else /* CONFIG_COMPAT */

#define comedi_register_ioctl32() do{}while(0)
#define comedi_unregister_ioctl32() do{}while(0)

#endif /* CONFIG_COMPAT */

#endif /* _COMEDI_COMPAT32_H */
