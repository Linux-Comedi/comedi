/*
    cfc.c

    This is a place for code driver writers wish to share between
    two or more drivers.  cfc is short
    for comedi-frank-common.

    Author:  Frank Mori Hess <fmhess@users.sourceforge.net>
    Copyright (C) 2002 Frank Mori Hess

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

************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include "comedi_fc.h"

static inline unsigned int bytes_per_sample( comedi_subdevice *subd )
{
	if( subd->flags & SDF_LSAMPL )
		return sizeof( lsampl_t );
	else
		return sizeof( sampl_t );
}

static inline unsigned int bytes_per_scan( comedi_subdevice *subd )
{
	return subd->async->cmd.chanlist_len * bytes_per_sample( subd );
}

static void increment_scan_progress( comedi_subdevice *subd, unsigned int num_bytes )
{
	comedi_async *async = subd->async;
	unsigned int scan_length = bytes_per_scan( subd );

	async->scan_progress += num_bytes;
	if( async->scan_progress >= scan_length )
	{
		async->scan_progress %= scan_length;
		async->events |= COMEDI_CB_EOS;
	}
}

/* Writes an array of data points to comedi's buffer */
unsigned int cfc_write_array_to_buffer( comedi_subdevice *subd, void *data,
	unsigned int num_bytes )
{
	comedi_async *async = subd->async;
	unsigned int retval;

	retval = comedi_buf_write_alloc( async, num_bytes );
	if( retval != num_bytes )
	{
		async->events |= COMEDI_CB_ERROR;
		return 0;
	}

	comedi_buf_memcpy_to( async, 0, data, num_bytes);

	comedi_buf_write_free( async, num_bytes );

	increment_scan_progress( subd, num_bytes );

	if( num_bytes ) async->events |= COMEDI_CB_BLOCK;

	return num_bytes;
}

unsigned int cfc_read_array_from_buffer( comedi_subdevice *subd, void *data,
	unsigned int num_bytes )
{
	comedi_async *async = subd->async;
	unsigned int bytes_available;

	bytes_available = comedi_buf_read_n_available( async );
	if( bytes_available < num_bytes )
	{
		num_bytes = bytes_available;
	}

	comedi_buf_memcpy_from( async, 0, data, num_bytes);

	comedi_buf_read_free( async, num_bytes );

	increment_scan_progress( subd, num_bytes );

	if( num_bytes ) async->events |= COMEDI_CB_BLOCK;

	return num_bytes;
}

unsigned int cfc_handle_events( comedi_device *dev, comedi_subdevice *subd )
{
	unsigned int events = subd->async->events;

	if( events & COMEDI_CB_ERROR )
		events |= COMEDI_CB_EOA;

	if( events & COMEDI_CB_EOA )
		subd->cancel( dev, subd );

	comedi_event( dev, subd, events );

	return events;
}

MODULE_AUTHOR("Frank Mori Hess <fmhess@users.sourceforge.net>");
MODULE_DESCRIPTION("Shared functoins for Comedi low-level drivers");
MODULE_LICENSE("GPL");

static int __init comedi_fc_init_module(void)
{
	return 0;
}
static void __exit comedi_fc_cleanup_module(void)
{
}
module_init(comedi_fc_init_module);
module_exit(comedi_fc_cleanup_module);

EXPORT_SYMBOL( cfc_write_array_to_buffer );
EXPORT_SYMBOL( cfc_read_array_from_buffer );
EXPORT_SYMBOL( cfc_handle_events );

