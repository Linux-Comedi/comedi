/*
    module/exp_ioctl.c
    exported comedi functions

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-8 David A. Schleef <ds@schleef.org>

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


#include <linux/comedidev.h>

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>




#if LINUX_VERSION_CODE >= 0x020200

/* for drivers */
EXPORT_SYMBOL(comedi_driver_register);
EXPORT_SYMBOL(comedi_driver_unregister);
EXPORT_SYMBOL(comedi_allocate_dev);
EXPORT_SYMBOL(comedi_deallocate_dev);
//EXPORT_SYMBOL(comedi_bufcheck);
//EXPORT_SYMBOL(comedi_done);
//EXPORT_SYMBOL(comedi_error_done);
EXPORT_SYMBOL(comedi_error);
//EXPORT_SYMBOL(comedi_eobuf);
//EXPORT_SYMBOL(comedi_eos);
EXPORT_SYMBOL(comedi_event);
EXPORT_SYMBOL(range_bipolar10);
EXPORT_SYMBOL(range_bipolar5);
EXPORT_SYMBOL(range_bipolar2_5);
EXPORT_SYMBOL(range_unipolar10);
EXPORT_SYMBOL(range_unipolar5);
EXPORT_SYMBOL(range_unknown);
#ifdef CONFIG_COMEDI_RT
EXPORT_SYMBOL(comedi_free_irq);
EXPORT_SYMBOL(comedi_request_irq);
EXPORT_SYMBOL(comedi_switch_to_rt);
EXPORT_SYMBOL(comedi_switch_to_non_rt);
EXPORT_SYMBOL(rt_pend_call);
#endif

/* for kcomedilib */
EXPORT_SYMBOL(comedi_devices);
EXPORT_SYMBOL(rtcomedi_lock_semaphore);
EXPORT_SYMBOL(check_chanlist);

#endif



