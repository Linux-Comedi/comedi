/*
    module/ni_pcimio.c
    Hardware driver for NI PCI-MIO E series cards

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
/*
Driver: ni_pcimio.o
Description: National Instruments PCI-MIO-E series (all boards)
Author: ds
Status: works
Devices: [National Instruments] PCI-MIO-16XE-50 (ni_pcimio),
  PCI-MIO-16XE-10, PXI-6030E, PCI-MIO-16E-1, PCI-MIO-16E-4, PCI-6040E,
  PXI-6040E, PCI-6031E, PCI-6032E, PCI-6033E, PCI-6071E, PCI-6023E,
  PCI-6024E, PCI-6025E, PXI-6025E, PCI-6034E, PCI-6035E, PCI-6052E,
  PCI-6110, PCI-6111, PCI-6711, PCI-6713, PXI-6071E, PXI-6070E,
  PXI-6052E, PCI-6036E, PCI-6731, PCI-6733
Updated: Sat, 16 Mar 2002 17:34:48 -0800

These boards are almost identical to the AT-MIO E series, except that
they use the PCI bus instead of ISA (i.e., AT).  See the notes for
the ni_atmio.o driver for additional information about these boards.

Autocalibration is supported on many of the devices, using the
calibration utility in Comedilib.

By default, the driver uses DMA to transfer analog input data to
memory.  When DMA is enabled, not all triggering features are
supported.

Streaming analog output is not supported on PCI-671x and PCI-673x.

PCI IDs are not known for PCI-6731 and PCI-6733.  Digital I/O may not
work on 673x.

Information (number of channels, bits, etc.) for some devices may be
incorrect.  Please check this and submit a bug if there are problems
for your device.

Bugs:
 - When DMA is enabled, COMEDI_EV_SCAN_END and COMEDI_EV_CONVERT do
   not work correctly.
 - There are reported problems with the 61xx and 67xx boards.

*/
/*
	The PCI-MIO E series driver was originally written by
	Tomasz Motylewski <...>, and ported to comedi by ds.


	References:
	
	   341079b.pdf  PCI E Series Register-Level Programmer Manual
	   340934b.pdf  DAQ-STC reference manual

	   322080b.pdf  6711/6713/6715 User Manual

	   320945c.pdf  PCI E Series User Manual
	   322138a.pdf  PCI-6052E and DAQPad-6052E User Manual
	
	ISSUES:

	need to deal with external reference for DAC, and other DAC
	properties in board properties
	
	deal with at-mio-16de-10 revision D to N changes, etc.
	
	need to add other CALDAC type
	
	need to slow down DAC loading.  I don't trust NI's claim that
	two writes to the PCI bus slows IO enough.  I would prefer to
	use udelay().  Timing specs: (clock)
		AD8522		30ns
		DAC8043		120ns
		DAC8800		60ns
		MB88341		?

*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/comedidev.h>
#include <linux/init.h>

#include <asm/io.h>

#include "ni_stc.h"
#include "mite.h"

//#define PCI_DEBUG

#define PCIDMA

#define PCIMIO 1
#undef ATMIO

#define MAX_N_CALDACS (16+16+2)

/* The following two tables must be in the same order */
static struct pci_device_id ni_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_NATINST, 0x0162, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1170, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x11d0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1180, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1190, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x11c0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1330, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1270, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1340, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1350, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2a60, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2a70, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2a80, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2ab0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2ca0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2c80, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x18b0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x14e0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x14f0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1880, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1870, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x15b0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x11b0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x18c0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1580, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x2890, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, ni_pci_table);

static ni_board ni_boards[]={
	{       device_id:      0x0162, // NI also says 0x1620.  typo?
		name:           "pci-mio-16xe-50",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  2048,
		alwaysdither:   1,
		gainlkup:       ai_gain_8,
		ai_speed:	50000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		caldac:         {dac8800,dac8043},
		has_8255:       0,
	},
	{       device_id:      0x1170,
		name:           "pci-mio-16xe-10",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  2048,
		ao_unipolar:    1,
		caldac:         {dac8800,dac8043,ad8522},
		has_8255:       0,
	},
	{       device_id:      0x11d0,
		name:           "pxi-6030e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  2048,
		ao_unipolar:    1,
		caldac:         {dac8800,dac8043,ad8522},
		has_8255:       0,
	},

	{       device_id:      0x1180,
		name:           "pci-mio-16e-1",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_16,
		ai_speed:	800,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  2048,
		ao_unipolar:    1,
		caldac:         {mb88341},
		has_8255:       0,
	},
	{       device_id:      0x1190,
		name:           "pci-mio-16e-4", /* aka pci-6040e */
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_16,
		/* Note: there have been reported problems with full speed
		 * on this board */
		ai_speed:	2000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  512,
		ao_unipolar:    1,
		caldac:         {ad8804_debug}, // doc says mb88341
		has_8255:       0,
	},
	{       device_id:      0x11c0,
		name:           "pxi-6040e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_16,
		ai_speed:	2000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  512,
		ao_unipolar:    1,
		caldac:         {mb88341},
		has_8255:       0,
	},

	{       device_id:      0x1330,
		name:           "pci-6031e",
		n_adchan:       64,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  2048,
		ao_unipolar:    1,
		caldac:         {dac8800,dac8043,ad8522},
		has_8255:       0,
	},
	{       device_id:      0x1270,
		name:           "pci-6032e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		ao_unipolar:    1,
		caldac:         {dac8800,dac8043,ad8522},
		has_8255:       0,
	},
	{       device_id:      0x1340,
		name:           "pci-6033e",
		n_adchan:       64,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		ao_unipolar:    1,
		caldac:         {dac8800,dac8043,ad8522},
		has_8255:       0,
	},
	{       device_id:      0x1350,
		name:           "pci-6071e",
		n_adchan:       64,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:	800,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  2048,
		ao_unipolar:    1,
		caldac:         {mb88341},
		has_8255:       0,
	},
	{       device_id:      0x2a60,
		name:           "pci-6023e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       0,
		aobits:         0,
		ao_unipolar:    0,
		caldac:         {ad8804}, /* manual is wrong */
		has_8255:	0,
	},
	{       device_id:      0x2a70,
		name:           "pci-6024e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		caldac:         {ad8804}, /* manual is wrong */
		has_8255:	0,
	},
	{       device_id:      0x2a80,
		name:           "pci-6025e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		caldac:         {ad8804}, /* manual is wrong */
		has_8255:	1,
	},
	{       device_id:      0x2ab0,
		name:           "pxi-6025e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		ao_unipolar:    1,
		caldac:         {ad8804}, /* manual is wrong */
		has_8255:	1,
	},

	{       device_id:      0x2ca0,
		name:           "pci-6034e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		caldac:         {mb88341},
		has_8255:	0,
	},
	{       device_id:      0x2c80,
		name:           "pci-6035e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		caldac:         {ad8804_debug},
		has_8255:	0,
	},
	{       device_id:      0x18b0,
		name:           "pci-6052e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:	3000,
		n_aochan:       2,
		aobits:         16,
		ao_unipolar:    1,
		ao_fifo_depth:  2048,
		caldac:         {ad8804,mb88341,ad8522}, /* manual is wrong */
	},
#if 0
	{       device_id:      0x0000, /* unknown */
		name:           "pci-6053e",
		n_adchan:       64,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:	3000,
		n_aochan:       2,
		aobits:         16,
		ao_unipolar:    1,
		ao_fifo_depth:  2048,
		caldac:         {ad8804,mb88341,ad8522}, /* manual is wrong */
	},
#endif
	{       device_id:      0x14e0,
		name:           "pci-6110",
		n_adchan:       4, 
		adbits:         12,
		ai_fifo_depth:  8192,
		alwaysdither:   0,
		gainlkup:       ai_gain_611x,
		ai_speed:	200,
		n_aochan:       2,
		aobits:         16,
		ao_671x:	1,
		ao_unipolar:    0,
		ao_fifo_depth:  2048,
		reg_611x:	1,
		caldac:         {ad8804_debug,ad8804_debug,ad8804_debug},/* XXX */
	},
	{       device_id:      0x14f0,
		name:           "pci-6111",
		n_adchan:       2,
		adbits:         12,
		ai_fifo_depth:  8192,
		alwaysdither:   0,
		gainlkup:       ai_gain_611x,
		ai_speed:	200,
		n_aochan:       2,
		aobits:         16,
		ao_671x:	1,
		ao_unipolar:    0,
		ao_fifo_depth:  2048,
		reg_611x:	1,
		caldac:         {ad8804_debug,ad8804_debug,ad8804_debug},/* XXX */
	},
#if 0
	/* The 6115 boards probably need their own driver */
	{       device_id:      0x2ed0,
		name:           "pci-6115",
		n_adchan:       4,
		adbits:         12,
		ai_fifo_depth:  8192,
		alwaysdither:   0,
		gainlkup:       ai_gain_611x,
		ai_speed:	100,
		n_aochan:       2,
		aobits:         16,
		ao_671x:	1,
		ao_unipolar:    0,
		ao_fifo_depth:  2048,
		reg_611x:	1,
		caldac:         {ad8804_debug,ad8804_debug,ad8804_debug},/* XXX */
	},
#endif
#if 0
	{       device_id:      0x0000,
		name:           "pxi-6115",
		n_adchan:       4,
		adbits:         12,
		ai_fifo_depth:  8192,
		alwaysdither:   0,
		gainlkup:       ai_gain_611x,
		ai_speed:	100,
		n_aochan:       2,
		aobits:         16,
		ao_671x:	1,
		ao_unipolar:    0,
		ao_fifo_depth:  2048,
		reg_611x:	1,
		caldac:         {ad8804_debug,ad8804_debug,ad8804_debug},/* XXX */
	},
#endif
	{       device_id:      0x1880,
		name:           "pci-6711",
		n_adchan:       0, /* no analog input */
		n_aochan:	4,
		aobits:         12,
		ao_unipolar:    0,
		ao_fifo_depth:  8192,
		ao_671x:	1,
		caldac:         {mb88341,mb88341},/* XXX */
	},
	{       device_id:      0x1870,
		name:           "pci-6713",
		n_adchan:       0, /* no analog input */
		n_aochan:	8,
		aobits:         12,
		ao_unipolar:    0,
		ao_fifo_depth:  16384,
		ao_671x:	1,
		caldac:         {mb88341,mb88341},/* XXX */
	},
#if 0
	{       device_id:      0x1880,
		name:           "pci-6731",
		n_adchan:       0, /* no analog input */
		n_aochan:	4,
		aobits:         16,
		ao_unipolar:    0,
		ao_fifo_depth:  8192,
		ao_671x:	1,
		caldac:         {mb88341,mb88341},/* XXX */
	},
	{       device_id:      0x1870,
		name:           "pci-6733",
		n_adchan:       0, /* no analog input */
		n_aochan:	8,
		aobits:         16,
		ao_unipolar:    0,
		ao_fifo_depth:  16384,
		ao_671x:	1,
		caldac:         {mb88341,mb88341},/* XXX */
	},
#endif
        {       device_id:      0x15b0,
                name:           "pxi-6071e",
                n_adchan:       64,
                adbits:         12,
                ai_fifo_depth:  512,
                alwaysdither:   1,
                gainlkup:       ai_gain_16,
                ai_speed:       800,
                n_aochan:       2,
                aobits:         12,
                ao_fifo_depth:  2048,
                ao_unipolar:    1,
		caldac:         {mb88341},
                has_8255:       0,
        },

        {       device_id:      0x11b0,
                name:           "pxi-6070e",
                n_adchan:       16,
                adbits:         12,
                ai_fifo_depth:  512,
                alwaysdither:   1,
                gainlkup:       ai_gain_16,
                ai_speed:       800,
                n_aochan:       2,
                aobits:         12,
                ao_fifo_depth:  2048,
                ao_unipolar:    1,
		caldac:         {mb88341},
                has_8255:       0,
        },
 	{	device_id:      0x18c0,
		name:           "pxi-6052e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:	3000,
		n_aochan:       2,
		aobits:         16,
		ao_unipolar:    1,
		ao_fifo_depth:  2048,
		caldac:         {mb88341,mb88341,ad8522},
	},
 	{	device_id:      0x1580,
		name:           "pxi-6031e",
		n_adchan:       64,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  2048,
		ao_unipolar:    1,
		caldac:         {dac8800,dac8043,ad8522},
	},
	{       device_id:      0x2890,
		name:           "pci-6036e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_4,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		caldac:         {mb88341},
		has_8255:	0,
	},
};
#define n_pcimio_boards ((sizeof(ni_boards)/sizeof(ni_boards[0])))

static int pcimio_attach(comedi_device *dev,comedi_devconfig *it);
static int pcimio_detach(comedi_device *dev);
static comedi_driver driver_pcimio={
	driver_name:	"ni_pcimio",
	module:		THIS_MODULE,
	attach:		pcimio_attach,
	detach: 	pcimio_detach,
};
COMEDI_INITCLEANUP(driver_pcimio);


/* How we access registers */

#define ni_writel(a,b)		(writel((a),dev->iobase+(b)))
#define ni_readl(a)		(readl(dev->iobase+(a)))
#define ni_writew(a,b)		(writew((a),dev->iobase+(b)))
#define ni_readw(a)		(readw(dev->iobase+(a)))
#define ni_writeb(a,b)		(writeb((a),dev->iobase+(b)))
#define ni_readb(a)		(readb(dev->iobase+(a)))

/* How we access STC registers */

/* We automatically take advantage of STC registers that can be
 * read/written directly in the I/O space of the board.  Most
 * PCIMIO devices map the low 8 STC registers to iobase+addr*2.
 * The 611x devices map the write registers to iobase+addr*2, and
 * the read registers to iobase+(addr-1)*2. */
/* However, the 611x boards still aren't working, so I'm disabling
 * non-windowed STC access temporarily */

#if 0
#define win_out(data,addr) do{ \
	if((addr)<8){ \
		ni_writew((data),(addr)*2); \
	}else{ \
		ni_writew((addr),Window_Address); \
		ni_writew((data),Window_Data); \
	} \
}while(0)
#else
#define win_out(data,addr) do{ \
	ni_writew((addr),Window_Address); \
	ni_writew((data),Window_Data); \
}while(0)
#endif

#define win_out2(data,addr) do{ \
	win_out((data)>>16, (addr)); \
	win_out((data)&0xffff, (addr)+1); \
}while(0)

#if 0
#define win_in(addr) ( \
	((addr)<7) \
	? (ni_readw(((addr) - boardtype.reg_611x)*2)) \
	: (ni_writew((addr),Window_Address),ni_readw(Window_Data)))
#else
#define win_in(addr) (ni_writew((addr),Window_Address),ni_readw(Window_Data))
#endif

#define win_save() (ni_readw(Window_Address))
#define win_restore(a) (ni_writew((a),Window_Address))

#define ao_win_out(a,b) do{ \
	ni_writew((b),AO_Window_Address_671x); \
	ni_writew((a),AO_Window_Data_671x); \
}while(0)



#define interrupt_pin(a)	0
#define IRQ_POLARITY 1

#define NI_E_IRQ_FLAGS		SA_SHIRQ


typedef struct{
	struct mite_struct *mite;

	NI_PRIVATE_COMMON
	
	dma_addr_t ai_dma_handle;
}ni_private;
#define devpriv ((ni_private *)dev->private)


#include "ni_mio_common.c"


static int pcimio_find_device(comedi_device *dev,int bus,int slot);
static int pcimio_ai_alloc(comedi_device *dev, comedi_subdevice *s,
	unsigned long new_size);


/* cleans up allocated resources */
static int pcimio_detach(comedi_device *dev)
{
	mio_common_detach(dev);

	if(dev->private && devpriv->mite)
		mite_unsetup(devpriv->mite);
	
	if(dev->irq){
		comedi_free_irq(dev->irq,dev);
	}

	return 0;
}

static int pcimio_attach(comedi_device *dev,comedi_devconfig *it)
{
	int		ret;
	
	printk("comedi%d: ni_pcimio:",dev->minor);
	
	ret=alloc_private(dev,sizeof(ni_private));
	if(ret<0)return ret;

	ret=pcimio_find_device(dev,it->options[0],it->options[1]);
	if(ret<0)return ret;

	printk(" %s",boardtype.name);
	dev->board_name=boardtype.name;
	
	ret = mite_setup(devpriv->mite);
	if(ret < 0)
	{
		printk(" error setting up mite\n");
		return ret;
	}
	dev->iobase = mite_iobase(devpriv->mite);

	dev->irq=mite_irq(devpriv->mite);

        if(dev->irq==0){
		printk(" unknown irq (bad)\n");
	}else{
        	printk(" ( irq = %d )",dev->irq);
        	if( (ret=comedi_request_irq(dev->irq,ni_E_interrupt,NI_E_IRQ_FLAGS,"ni_pcimio",dev))<0 ){
                	printk(" irq not available\n");
			dev->irq=0;
        	}
	}

	ret = ni_E_init(dev,it);
	if(ret<0)return ret;

	dev->subdevices[0].buf_alloc = pcimio_ai_alloc;

	return ret;
}


static int pcimio_find_device(comedi_device *dev,int bus,int slot)
{
	struct mite_struct *mite;
	int i;

	for(mite=mite_devices;mite;mite=mite->next){
		if(mite->used)continue;
		if(bus || slot){
			if(bus!=mite->pcidev->bus->number ||
			   slot!=PCI_SLOT(mite->pcidev->devfn))
				continue;
		}

		for(i=0;i<n_pcimio_boards;i++){
			if(mite_device_id(mite)==ni_boards[i].device_id){
				dev->board_ptr=ni_boards+i;
				devpriv->mite=mite;

				return 0;
			}
		}
	}
	printk("no device found\n");
	mite_list_devices();
	return -EIO;
}

/* This needs to be fixed before it can be used for AO, since it
 * uses devpriv->ai_dma_handle */
static int pcimio_ai_alloc(comedi_device *dev, comedi_subdevice *s,
	unsigned long new_size)
{
	int ret;

	ret = mite_buf_alloc(devpriv->mite, s->async, new_size);
	if(ret<0)return ret;

	return 0;
#if 0
	comedi_async *async = s->async;

	if(async->prealloc_buf && async->prealloc_bufsz == new_size){
		return 0;
	}

	if(async->prealloc_bufsz){
		pci_free_consistent(devpriv->mite->pcidev,
			async->prealloc_bufsz, async->prealloc_buf,
			devpriv->ai_dma_handle);
		async->prealloc_buf = NULL;
		async->prealloc_bufsz = 0;
	}

	if(new_size){
		async->prealloc_buf = pci_alloc_consistent(devpriv->mite->pcidev,
			new_size, &devpriv->ai_dma_handle);
		if(async->prealloc_buf == NULL){
			async->prealloc_bufsz = 0;
			return -ENOMEM;
		}
	}
	async->prealloc_bufsz = new_size;

	return 0;
#endif
}


