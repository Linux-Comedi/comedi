/*
    comedi/kcomedilib/ksyms.c
    a comedlib interface for kernel modules

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2001 David A. Schleef <ds@stm.lbl.gov>

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



#include <linux/comedi.h>
#include <linux/comedidev.h>
#include <linux/comedilib.h>

#include <linux/module.h>

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/malloc.h>


#ifdef LINUX_V22

/* functions specific to kcomedilib */

#ifdef CONFIG_COMEDI_TRIG
EXPORT_SYMBOL(__comedi_trigger);
#endif
EXPORT_SYMBOL(comedi_register_callback);
EXPORT_SYMBOL(comedi_get_subdevice_flags);
EXPORT_SYMBOL(comedi_get_len_chanlist);
EXPORT_SYMBOL(comedi_get_krange);
EXPORT_SYMBOL(comedi_get_buf_head_pos);
EXPORT_SYMBOL(comedi_set_user_int_count);

/* er?  why not in user-space? */
EXPORT_SYMBOL(comedi_get_version_code);
EXPORT_SYMBOL(comedi_get_driver_name);
EXPORT_SYMBOL(comedi_get_board_name);
EXPORT_SYMBOL(comedi_get_subdevice_type);

/* This list comes from user-space comedilib, to show which
 * functions are not ported yet. */

EXPORT_SYMBOL(comedi_open);
EXPORT_SYMBOL(comedi_close);
EXPORT_SYMBOL(comedi_poll);
//EXPORT_SYMBOL(comedi_loglevel);
//EXPORT_SYMBOL(comedi_perror);
//EXPORT_SYMBOL(comedi_strerror);
//EXPORT_SYMBOL(comedi_errno);
//EXPORT_SYMBOL(comedi_fileno);
EXPORT_SYMBOL(comedi_get_n_subdevices);
EXPORT_SYMBOL(comedi_find_subdevice_by_type);
EXPORT_SYMBOL(comedi_get_n_channels);
EXPORT_SYMBOL(comedi_get_maxdata);
EXPORT_SYMBOL(comedi_get_rangetype);
//EXPORT_SYMBOL(comedi_get_range);
//EXPORT_SYMBOL(comedi_find_range);
EXPORT_SYMBOL(comedi_get_n_ranges);
//EXPORT_SYMBOL(comedi_range_is_chan_specific);
//EXPORT_SYMBOL(comedi_maxdata_is_chan_specific);
EXPORT_SYMBOL(comedi_cancel);
#ifdef CONFIG_COMEDI_TRIG
EXPORT_SYMBOL(comedi_trigger);
#endif
EXPORT_SYMBOL(comedi_command);
EXPORT_SYMBOL(comedi_command_test);
//EXPORT_SYMBOL(comedi_do_insnlist);
EXPORT_SYMBOL(comedi_do_insn);
EXPORT_SYMBOL(comedi_lock);
EXPORT_SYMBOL(comedi_unlock);
//EXPORT_SYMBOL(comedi_to_phys);
//EXPORT_SYMBOL(comedi_from_phys);
EXPORT_SYMBOL(comedi_data_read);
EXPORT_SYMBOL(comedi_data_write);
//EXPORT_SYMBOL(comedi_sv_init);
//EXPORT_SYMBOL(comedi_sv_update);
//EXPORT_SYMBOL(comedi_sv_measure);
EXPORT_SYMBOL(comedi_dio_config);
EXPORT_SYMBOL(comedi_dio_read);
EXPORT_SYMBOL(comedi_dio_write);
EXPORT_SYMBOL(comedi_dio_bitfield);
//EXPORT_SYMBOL(comedi_get_timer);
//EXPORT_SYMBOL(comedi_timed_1chan);
//EXPORT_SYMBOL(comedi_set_global_oor_behavior);
EXPORT_SYMBOL(comedi_map);
EXPORT_SYMBOL(comedi_unmap);


#endif

