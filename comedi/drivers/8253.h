/*
    comedi/drivers/8253.h
    Header file for 8253

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@stm.lbl.gov>

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

#ifndef _8253_H
#define _8253_H

#define i8253_cascade_ns_to_timer i8253_cascade_ns_to_timer_power

static inline void i8253_cascade_ns_to_timer_2div(int i8253_osc_base, unsigned int *d1, unsigned int *d2, unsigned int *nanosec, int round_mode)
{
	int divider;
	int div1, div2;
	int div1_glb, div2_glb, ns_glb;
	int div1_lub, div2_lub, ns_lub;
	int ns;

	divider = (*nanosec + i8253_osc_base / 2) / i8253_osc_base;

	/* find 2 integers 1<={x,y}<=65536 such that x*y is
	   close to divider */

	div1_lub = div2_lub = 0;
	div1_glb = div2_glb = 0;

	ns_glb = 0;
	ns_lub = 0xffffffff;

	div2 = 0x10000;
	for (div1 = divider / 65536 + 1; div1 < div2; div1++) {
		div2 = divider / div1;

		ns = i8253_osc_base * div1 * div2;
		if (ns <= *nanosec && ns > ns_glb) {
			ns_glb = ns;
			div1_glb = div1;
			div2_glb = div2;
		}

		div2++;
		if (div2 <= 65536) {
			ns = i8253_osc_base * div1 * div2;
			if (ns > *nanosec && ns < ns_lub) {
				ns_lub = ns;
				div1_lub = div1;
				div2_lub = div2;
			}
		}
	}

	*nanosec = div1_lub * div2_lub * i8253_osc_base;
	*d1 = div1_lub;
	*d2 = div2_lub;
	return;
}

static inline void i8253_cascade_ns_to_timer_power(int i8253_osc_base, unsigned int *d1, unsigned int *d2, unsigned int *nanosec, int round_mode)
{
	int div1, div2;
	int base;

	for (div1 = 1; div1 <= (1 << 16); div1 <<= 1) {
		base = i8253_osc_base * div1;
		switch (round_mode) {
		case TRIG_ROUND_NEAREST:
		default:
			div2 = (*nanosec + base / 2) / base;
			break;
		case TRIG_ROUND_DOWN:
			div2 = (*nanosec) / base;
			break;
		case TRIG_ROUND_UP:
			div2 = (*nanosec + base - 1) / base;
			break;
		}
		if (div2 <= 65536) {
			*nanosec = div2 * base;
			*d1 = div1 & 0xffff;
			*d2 = div2 & 0xffff;
			return;
		}
	}

	/* shouldn't get here */
	div1 = 0x10000;
	div2 = 0x10000;
	*nanosec = div1 * div2 * i8253_osc_base;
	*d1 = div1 & 0xffff;
	*d2 = div2 & 0xffff;
}


#endif

