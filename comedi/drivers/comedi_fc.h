/*
    cfc.h

    This is a place for code driver writers wish to share between
    two or more drivers.
    If it ever grows beyond a couple inline functions, I'll make
    a cfc.o module to go with this header file.  cfc is short
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

#include <linux/comedidev.h>

/* Writes an array of data points to comedi's buffer */
extern unsigned int cfc_write_array_to_buffer( comedi_subdevice *subd, void *data,
	unsigned int num_bytes );

static inline unsigned int cfc_write_to_buffer( comedi_subdevice *subd, sampl_t data )
{
	return cfc_write_array_to_buffer( subd, &data, sizeof( data ) );
};

static inline unsigned int cfc_write_long_to_buffer( comedi_subdevice *subd, lsampl_t data )
{
	return cfc_write_array_to_buffer( subd, &data, sizeof( data ) );
};

extern unsigned int cfc_read_array_from_buffer( comedi_subdevice *subd, void *data,
	unsigned int num_bytes );

extern unsigned int cfc_handle_events( comedi_device *dev, comedi_subdevice *subd );

