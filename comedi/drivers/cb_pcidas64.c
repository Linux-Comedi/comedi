/*
    cb_pcidas64.c
    This is a driver for the ComputerBoards/MeasurementComputing PCI-DAS
    64xx, 60xx, and 4020 cards.

    Author:  Frank Mori Hess <fmhess@uiuc.edu>
    Copyright (C) 2001, 2002 Frank Mori Hess <fmhess@uiuc.edu>

    Thanks go to Steve Rosenbluth for providing the source code for
    his pci-das6402 driver, and source code for working QNX pci-6402
    drivers by Greg Laird and Mariusz Bogacz.  None of the code was
    used directly here, but was useful as an additional source of
    documentation on how to program the boards.

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

************************************************************************/

/*

Driver: cb_pcidas64.o
Description: Driver for the ComputerBoards/MeasurementComputing
   PCI-DAS64xx, 60XX, and 4020 series with the PLX 9080 PCI controller.
Author: Frank Mori Hess <fmhess@users.sourceforge.net>
Status: in development, pci-das6025e analog input works
Updated: 2002-03-30
Devices: [Measurement Computing] PCI-DAS6402/16 (cb_pcidas64),
  PCI-DAS6402/12, PCI-DAS64/M1/16, PCI-DAS64/M2/16,
  PCI-DAS64/M3/16, PCI-DAS6402/16/JR, PCI-DAS64/M1/16/JR,
  PCI-DAS64/M2/16/JR, PCI-DAS64/M3/16/JR, PCI-DAS64/M1/14,
  PCI-DAS64/M2/14, PCI-DAS64/M3/14, PCI-DAS6025E, PCI-DAS6034E,
  PCI-DAS6035E, PCI-DAS4020/12

Configuration options:
   [0] - PCI bus of device (optional)
   [1] - PCI slot of device (optional)

Calibration is not yet supported!
Feel free to send and success/failure reports to Frank Hess.

Some devices are not identified because the PCI device IDs are not known.
*/

/*

TODO:
	command support for ao
	calibration subdevice
	user counter subdevice
	there are a number of boards this driver will support when they are
		fully released, but does not yet since the pci device id numbers
		are not yet available.
	need to take care to prevent ai and ao from affecting each other's register bits
	support prescaled 100khz clock for slow pacing (not available on 6000 series?)
	figure out cause of intermittent lockups (pci dma?)
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <asm/io.h>
#include <linux/comedidev.h>
#include "8253.h"
#include "8255.h"
#include "plx9080.h"

#undef PCIDAS64_DEBUG	// disable debugging code
//#define PCIDAS64_DEBUG	// enable debugging code

#ifdef PCIDAS64_DEBUG
#define DEBUG_PRINT(format, args...)  printk("comedi: " format , ## args )
#else
#define DEBUG_PRINT(format, args...)
#endif

#define TIMER_BASE 25	// 40MHz master clock
#define PRESCALED_TIMER_BASE	10000	// 100kHz 'prescaled' clock for slow aquisition, maybe I'll support this someday
#define QUARTER_AI_FIFO_SIZE 2048	// 1/4 analog input fifo size
// size in bytes of transfers used for dma transfers, also size of buffers that make up dma ring
#define DMA_TRANSFER_SIZE 0x1000
// number of dma transfers we will chain together into a ring (and the number of dma buffers we maintain)
#define DMA_RING_COUNT 32

/* PCI-DAS64xxx base addresses */

// indices of base address regions
#define PLX9080_BADRINDEX 0
#define MAIN_BADRINDEX 2
#define DIO_COUNTER_BADRINDEX 3

// private(dev)->main_iobase registers
// write-only
#define INTR_ENABLE_REG	0x0	// interrupt enable register
#define    ADC_INTR_SRC_MASK	0x3	// bits that set adc interrupt source
#define    ADC_INTR_QFULL_BITS	0x0	// interrupt fifo quater full
#define    ADC_INTR_EOC_BITS	0x1	// interrupt end of conversion
#define    ADC_INTR_EOSCAN_BITS	0x2	// interrupt end of scan
#define    ADC_INTR_EOSEQ_BITS	0x3	// interrupt end of sequence (probably wont use this it's pretty fancy)
#define    EN_ADC_INTR_SRC_BIT	0x4	// enable adc interrupt source
#define    EN_ADC_DONE_INTR_BIT	0x8	// enable adc aquisition done interrupt
#define    EN_DAC_INTR_SRC_BIT	0x40	// enable dac interrupt source
#define    EN_ADC_ACTIVE_INTR_BIT	0x200	// enable adc active interrupt
#define    EN_ADC_STOP_INTR_BIT	0x400	// enable adc stop trigger interrupt
#define    EN_DAC_ACTIVE_INTR_BIT	0x800	// enable dac active interrupt
#define    EN_DAC_UNDERRUN_BIT	0x4000	// enable dac underrun status bit
#define    EN_ADC_OVERRUN_BIT	0x8000	// enable adc overrun status bit
#define HW_CONFIG_REG	0x2	// hardware config register
#define    MASTER_CLOCK_4020_MASK	0x3	// bits that specify master clock source for 4020
#define    INTERNAL_CLOCK_4020_BITS	0x1	// user 40 MHz internal master clock for 4020
#define    EXT_QUEUE_BIT	0x200	// use external channel/gain queue (more versatile than internal queue)
#define    SLOW_DAC_BIT	0x400	// use 225 nanosec strobe when loading dac instead of 50 nanosec
#define    HW_CONFIG_DUMMY_BITS	0x2000	// bit with unknown function yet given as default value in pci-das64 manual
#define FIFO_SIZE_REG	0x4	// allows adjustment of fifo sizes, we will always use maximum
#define    ADC_FIFO_SIZE_MASK	0x7f	// bits that set adc fifo size
#define    ADC_FIFO_60XX_BITS	0x78	// 8 kilosample adc fifo for 60xx boards
#define    ADC_FIFO_64XX_BITS	0x38	// 8 kilosample adc fifo for 64xx boards
#define    ADC_FIFO_4020_BITS	0x0	// dual 64 kilosample adc fifo for 4020
#define    DAC_FIFO_SIZE_MASK	0xff00	// bits that set dac fifo size
#define    DAC_FIFO_BITS 0xf000
#define DAQ_SYNC_REG	0xc
#define ADC_CONTROL0_REG	0x10	// adc control register 0
#define    ADC_GATE_SRC_MASK	0x3	// bits that select gate
#define    ADC_SOFT_GATE_BITS	0x1	// software gate
#define    ADC_EXT_GATE_BITS	0x2	// external digital gate
#define    ADC_ANALOG_GATE_BITS	0x3	// analog level gate
#define    ADC_GATE_LEVEL_BIT	0x4	// level-sensitive gate (for digital)
#define    ADC_GATE_POLARITY_BIT	0x8	// gate active low
#define    ADC_START_TRIG_SOFT_BITS	0x10
#define    ADC_START_TRIG_EXT_BITS	0x20
#define    ADC_START_TRIG_ANALOG_BITS	0x30
#define    ADC_START_TRIG_MASK	0x30
#define    ADC_START_TRIG_FALLING_BIT	0x40	// trig 1 uses falling edge
#define    ADC_EXT_CONV_FALLING_BIT	0x800	// external pacing uses falling edge
#define    ADC_DMA_DISABLE_BIT	0x4000	// disables dma
#define    ADC_ENABLE_BIT	0x8000	// master adc enable
#define ADC_CONTROL1_REG	0x12	// adc control register 1
#define    ADC_QUEUE_CONFIG_BIT	0x1	// should be set for boards with > 16 channels
#define    CONVERT_POLARITY_BIT	0x10
#define    EOC_POLARITY_BIT	0x20
#define    SW_GATE_BIT	0x40	// software gate of adc
#define    ADC_DITHER_BIT	0x200	// turn on extra noise for dithering
#define    RETRIGGER_BIT	0x800
#define    LO_CHANNEL_4020_BITS(x)	(((x) & 0x3) << 8)	// low channel for 4020 two channel mode
#define    HI_CHANNEL_4020_BITS(x)	(((x) & 0x3) << 10)	// high channel for 4020 two channel mode
#define    TWO_CHANNEL_4020_BITS	0x1000	// two channel mode for 4020
#define    FOUR_CHANNEL_4020_BITS	0x2000	// four channel mode for 4020
#define    ADC_MODE_BITS(x)	(((x) & 0xf) << 12)
#define CALIBRATION_REG	0x14
#define    SELECT_8800_BIT	0x1
#define    SELECT_1590_BIT	0x2	// XXX 60xx only
/* calibration sources for 6025 appear to be:
 *  0 : ground
 *  1 : 10V
 *  2 : 5V
 *  3 : 0.5V
 *  4 : 0.05V
 *  5 : ground
 *  6 : dac channel 0
 *  7 : dac channel 1
 */
#define    CAL_SRC_BITS(x)	(((x) & 0xf) << 3)	// XXX 60xx only
#define    SERIAL_DATA_IN_BIT	0x80
#define    SERIAL_CLOCK_BIT	0x100
#define    CAL_GAIN_BIT	0x800
#define    CAL_EN_BIT	0x200	// calibration enable XXX 0x40 for 6402?
#define ADC_SAMPLE_INTERVAL_LOWER_REG	0x16	// lower 16 bits of sample interval counter
#define ADC_SAMPLE_INTERVAL_UPPER_REG	0x18	// upper 8 bits of sample interval counter
#define ADC_DELAY_INTERVAL_LOWER_REG	0x1a	// lower 16 bits of delay interval counter
#define ADC_DELAY_INTERVAL_UPPER_REG	0x1c	// upper 8 bits of delay interval counter
#define ADC_COUNT_LOWER_REG	0x1e	// lower 16 bits of hardware conversion/scan counter
#define ADC_COUNT_UPPER_REG	0x20	// upper 8 bits of hardware conversion/scan counter
#define ADC_START_REG	0x22	// software trigger to start aquisition
#define ADC_CONVERT_REG	0x24	// initiates single conversion
#define    ADC_CONVERT_CHANNEL_4020_BITS(x) (((x) & 0x3) << 8)
#define ADC_QUEUE_CLEAR_REG	0x26	// clears adc queue
#define ADC_QUEUE_LOAD_REG	0x28	// loads adc queue
#define    CHAN_BITS(x)	((x) & 0x3f)
#define    UNIP_BIT	0x800	// unipolar/bipolar bit
#define    ADC_DIFFERENTIAL_BIT	0x1000	// single-ended/ differential bit
#define    ADC_COMMON_BIT	0x2000	// non-referenced single-ended (common-mode input)
#define    QUEUE_EOSEQ_BIT	0x4000	// queue end of sequence
#define    QUEUE_EOSCAN_BIT	0x8000	// queue end of scan
#define ADC_BUFFER_CLEAR_REG	0x2a
#define ADC_QUEUE_HIGH_REG	0x2c	// high channel for internal queue, use CHAN_BITS() macro above
#define DAC_CONTROL0_REG	0x50	// dac control register 0
#define    DAC_ENABLE_BIT	0x8000	// dac controller enable bit
#define DAC_CONTROL1_REG	0x52	// dac control register 0
#define    DAC_RANGE_BITS(channel, code)	(((code) & 0x3) << (2 * ((channel) & 0x1)))
#define    DAC_OUTPUT_ENABLE_BIT	0x80	// dac output enable bit
#define DAC_BUFFER_CLEAR_REG 0x66	// clear dac buffer
#define DAC_CONVERT_REG(channel)	((0x70) + (2 * ((channel) & 0x1)))
// read-only
#define HW_STATUS_REG	0x0	// hardware status register, reading this apparently clears pending interrupts as well
#define   DAC_UNDERRUN_BIT	0x1
#define   ADC_OVERRUN_BIT 0x2
#define   DAC_ACTIVE_BIT	0x4
#define   ADC_ACTIVE_BIT	0x8
#define   DAC_INTR_PENDING_BIT	0x10
#define   ADC_INTR_PENDING_BIT	0x20
#define   DAC_DONE_BIT	0x40
#define   ADC_DONE_BIT	0x80
#define   EXT_INTR_PENDING_BIT	0x100
#define   ADC_STOP_BIT	0x200
#define   PIPE_FULL_BITS(x)	(((x) >> 10) & 0x3)
#define   HW_REVISION(x)	(((x) >> 12) & 0xf)
#define PIPE1_READ_REG	0x4
#define ADC_READ_PNTR_REG	0x8
#define LOWER_XFER_REG	0x10
#define ADC_WRITE_PNTR_REG	0xc
#define PREPOST_REG	0x14
#define   ADC_UPP_READ_PNTR_CODE(x)	(((x) >> 12) & 0x3)
#define   ADC_UPP_WRITE_PNTR_CODE(x)	(((x) >> 14) & 0x3)
#define   CHAIN_FLAG_BITS(x)	(((x) >> 6) & 0x3)
// read-write
#define I8255_4020_REG 0x48	// 8255 offset, for 4020 only
#define ADC_QUEUE_FIFO_REG	0x100	// external channel/gain queue, uses same bits as ADC_QUEUE_LOAD_REG
#define ADC_FIFO_REG 0x200	// adc data fifo

// private(dev)->dio_counter_iobase registers
// XXX board dependent
#define DIO_8255_OFFSET	0x0
#define DO_REG	0x20
#define DI_REG	0x28
#define DIO_DIRECTION_60XX_REG	0x40
#define DIO_DATA_60XX_REG	0x48

// I2C addresses for 4020
#define RANGE_CAL_I2C_ADDR	0x40
#define CALDAC0_I2C_ADDR	0x18
#define CALDAC1_I2C_ADDR	0x1a
// XXX fix ranges for 60xx

// analog input ranges for 64xx boards
static comedi_lrange ai_ranges_64xx =
{
	8,
	{
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2.5),
		BIP_RANGE(1.25),
		UNI_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(2.5),
		UNI_RANGE(1.25)
	}
};
static int ai_range_bits_64xx[] = {
	0x000,
	0x100,
	0x200,
	0x300,
	0x800,
	0x900,
	0xa00,
	0xb00,
};

// analog input ranges for 60xx boards
static comedi_lrange ai_ranges_60xx =
{
	4,
	{
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(0.5),
		BIP_RANGE(0.05),
	}
};
static int ai_range_bits_60xx[] = {
	0x000,
	0x100,
	0x400,
	0x700,
};

// analog input ranges for 4020 board
static comedi_lrange ai_ranges_4020 =
{
	2,
	{
		BIP_RANGE(5),
		BIP_RANGE(1),
	}
};

// analog output ranges
static comedi_lrange ao_ranges_64xx =
{
	4,
	{
		BIP_RANGE(5),
		BIP_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(10),
	}
};
static int ao_range_code_64xx[] =
{
	0x0,
	0x1,
	0x2,
	0x3,
};

static comedi_lrange ao_ranges_60xx =
{
	1,
	{
		BIP_RANGE(10),
		UNI_RANGE(10),
	}
};
static int ao_range_code_60xx[] =
{
	0x0,
	0x2,
};

static comedi_lrange ao_ranges_4020 =
{
	2,
	{
		BIP_RANGE(5),
		BIP_RANGE(10),
	}
};
static int ao_range_code_4020[] =
{
	0x1,
	0x0,
};

enum register_layout
{
	LAYOUT_60XX,
	LAYOUT_64XX,
	LAYOUT_4020,
};

typedef struct pcidas64_board_struct
{
	char *name;
	int device_id;	// pci device id
	int ai_se_chans;	// number of ai inputs in single-ended mode
	int ai_bits;	// analog input resolution
	int ai_speed;	// fastest conversion period in ns
	comedi_lrange *ai_range_table;
	int *ai_range_bits;
	int ao_nchan;	// number of analog out channels
	int ao_scan_speed;	// analog output speed (for a scan, not conversion)
	comedi_lrange *ao_range_table;
	int *ao_range_code;
	int fifo_depth;	// number of entries in fifo (may be 32 bit or 16 bit entries)
	enum register_layout layout;	// different board families have slightly different registers
} pcidas64_board;

static pcidas64_board pcidas64_boards[] =
{
	{
		name:		"pci-das6402/16",
		device_id:	0x1d,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	5000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ao_range_table:	&ao_ranges_64xx,
		ao_range_code:	ao_range_code_64xx,
	},
	{
		name:		"pci-das6402/12",	// XXX check
		device_id:	0x1e,
		ai_se_chans:	64,
		ai_bits:	12,
		ai_speed:	5000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ao_range_table:	&ao_ranges_64xx,
		ao_range_code:	ao_range_code_64xx,
	},
	{
		name:		"pci-das64/m1/16",
		device_id:	0x35,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	1000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ao_range_table:	&ao_ranges_64xx,
		ao_range_code:	ao_range_code_64xx,
	},
	{
		name:		"pci-das64/m2/16",
		device_id:	0x36,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	500,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ao_range_table:	&ao_ranges_64xx,
		ao_range_code:	ao_range_code_64xx,
	},
	{
		name:		"pci-das64/m3/16",
		device_id:	0x37,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	333,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ao_range_table:	&ao_ranges_64xx,
		ao_range_code:	ao_range_code_64xx,
	},
	{
		name:		"pci-das6025",
		device_id:	0x5e,
		ai_se_chans:	16,
		ai_bits:	12,
		ai_speed:	5000,
		ao_nchan: 2,
		ao_scan_speed:	100000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_60XX,
		ai_range_table:	&ai_ranges_60xx,
		ai_range_bits:	ai_range_bits_60xx,
		ao_range_table:	&ao_ranges_60xx,
		ao_range_code:	ao_range_code_60xx,
	},
	{
		name:		"pci-das6034",
		device_id:	0x63,
		ai_se_chans:	16,
		ai_bits:	16,
		ai_speed:	5000,
		ao_nchan: 0,
		ao_scan_speed:	0,
		fifo_depth: 0x2000,
		layout:	LAYOUT_60XX,
		ai_range_table:	&ai_ranges_60xx,
		ai_range_bits:	ai_range_bits_60xx,
	},
	{
		name:		"pci-das6035",
		device_id:	0x64,
		ai_se_chans:	16,
		ai_bits:	16,
		ai_speed:	5000,
		ao_nchan:	2,
		ao_scan_speed:	100000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_60XX,
		ai_range_table:	&ai_ranges_60xx,
		ai_range_bits:	ai_range_bits_60xx,
		ao_range_table:	&ao_ranges_60xx,
		ao_range_code:	ao_range_code_60xx,
	},
	{
		name:	"pci-das4020/12",
		device_id:	0x52,
		ai_se_chans:	4,
		ai_bits:	12,
		ai_speed:	50,
		ao_nchan:	2,
		ao_scan_speed:	0,	// no hardware pacing on ao
		fifo_depth: 0x8000,	// 32K 32bit entries = 64K samples
		layout:	LAYOUT_4020,
		ai_range_table:	&ai_ranges_4020,
		ai_range_bits:	NULL,
		ao_range_table:	&ao_ranges_4020,
		ao_range_code:	ao_range_code_4020,
	},
#if 0
	{
		name:		"pci-das6402/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	5000,
		ao_nchan: 0,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
	},
	{
		name:		"pci-das64/m1/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	1000,
		ao_nchan: 0,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
	},
	{
		name:		"pci-das64/m2/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	500,
		ao_nchan: 0,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
	},
	{
		name:		"pci-das64/m3/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	333,
		ao_nchan: 0,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
	},
	{
		name:		"pci-das64/m1/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	1000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
	},
	{
		name:		"pci-das64/m2/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	500,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
	},
	{
		name:		"pci-das64/m3/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	333,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		fifo_depth: 0x2000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
	},
#endif

};
// Number of boards in cb_pcidas_boards
#define N_BOARDS	(sizeof(pcidas64_boards) / sizeof(pcidas64_board))

static struct pci_device_id pcidas64_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x001d, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x001e, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x0035, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x0036, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x0037, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x005e, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x0063, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x0064, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x0052, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, pcidas64_pci_table);

/*
 * Useful for shorthand access to the particular board structure
 */
extern inline pcidas64_board* board(comedi_device *dev)
{
	return (pcidas64_board *)dev->board_ptr;
}

/* this structure is for data unique to this hardware driver. */
typedef struct
{
	struct pci_dev *hw_dev;	// pointer to board's pci_dev struct
	// base addresses (physical)
	unsigned long plx9080_phys_iobase;
	unsigned long main_phys_iobase;
	unsigned long dio_counter_phys_iobase;
	// base addresses (ioremapped)
	unsigned long plx9080_iobase;
	unsigned long main_iobase;
	unsigned long dio_counter_iobase;
	// local address (used by dma controller)
	uint32_t local0_iobase;
	uint32_t local1_iobase;
	volatile unsigned int ai_count;	// number of analog input samples remaining
	uint16_t *ai_buffer[DMA_RING_COUNT];	// dma buffers for analog input
	dma_addr_t ai_buffer_phys_addr[DMA_RING_COUNT];	// physical addresses of ai dma buffers
	struct plx_dma_desc *dma_desc;	// array of dma descriptors read by plx9080, allocated to get proper alignment
	dma_addr_t dma_desc_phys_addr;	// physical address of dma descriptor array
	volatile unsigned int dma_index;	// index of the dma descriptor/buffer that is currently being used
	volatile unsigned int ao_count;	// number of analog output samples remaining
	volatile unsigned int ao_value[2];	// remember what the analog outputs are set to, to allow readback
	unsigned int hw_revision;	// stc chip hardware revision number
	volatile unsigned int intr_enable_bits;	// bits to send to INTR_ENABLE_REG register
	volatile uint16_t adc_control1_bits;	// bits to send to ADC_CONTROL1_REG register
	volatile uint16_t fifo_size_bits;	// bits to send to FIFO_SIZE_REG register
	volatile uint32_t plx_control_bits;	// bits written to plx9080 control register
	volatile int calibration_source;	// index of calibration source readable through ai ch0
} pcidas64_private;

/* inline function that makes it easier to
 * access the private structure.
 */
extern inline pcidas64_private* private(comedi_device *dev)
{
	return dev->private;
}

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int attach(comedi_device *dev,comedi_devconfig *it);
static int detach(comedi_device *dev);
static comedi_driver driver_cb_pcidas={
	driver_name:	"cb_pcidas64",
	module:		THIS_MODULE,
	attach:		attach,
	detach:		detach,
};

static int ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int ai_config_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int ao_readback_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int ai_cmd(comedi_device *dev,comedi_subdevice *s);
static int ai_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd);
//static int ao_cmd(comedi_device *dev,comedi_subdevice *s);
//static int ao_inttrig(comedi_device *dev, comedi_subdevice *subdev, unsigned int trig_num);
//static int ao_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd);
static void handle_interrupt(int irq, void *d, struct pt_regs *regs);
static int ai_cancel(comedi_device *dev, comedi_subdevice *s);
//static int ao_cancel(comedi_device *dev, comedi_subdevice *s);
static int dio_callback(int dir, int port, int data, unsigned long arg);
static int dio_callback_4020(int dir, int port, int data, unsigned long arg);
static int di_rbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int do_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int dio_60xx_config_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int dio_60xx_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int calib_write_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int eeprom_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static void check_adc_timing(comedi_cmd *cmd);
static unsigned int get_divisor(unsigned int ns, unsigned int flags);
static int caldac_8800_write(comedi_device *dev, unsigned int address, uint8_t value);
//static int dac_1590_write(comedi_device *dev, unsigned int dac_a, unsigned int dac_b);

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_cb_pcidas);

// initialize plx9080 chip
void init_plx9080(comedi_device *dev)
{
	uint32_t bits;
	unsigned long plx_iobase = private(dev)->plx9080_iobase;

	private(dev)->plx_control_bits = readl(private(dev)->plx9080_iobase + PLX_CONTROL_REG);

	// plx9080 dump
	DEBUG_PRINT(" plx interrupt status 0x%x\n", readl(plx_iobase + PLX_INTRCS_REG));
	DEBUG_PRINT(" plx id bits 0x%x\n", readl(plx_iobase + PLX_ID_REG));
	DEBUG_PRINT(" plx control reg 0x%x\n", private(dev)->plx_control_bits);

	DEBUG_PRINT(" plx revision 0x%x\n", readl(plx_iobase + PLX_REVISION_REG));
	DEBUG_PRINT(" plx dma channel 0 mode 0x%x\n", readl(plx_iobase + PLX_DMA0_MODE_REG));
	DEBUG_PRINT(" plx dma channel 1 mode 0x%x\n", readl(plx_iobase + PLX_DMA1_MODE_REG));
	DEBUG_PRINT(" plx dma channel 0 pci address 0x%x\n", readl(plx_iobase + PLX_DMA0_PCI_ADDRESS_REG));
	DEBUG_PRINT(" plx dma channel 0 local address 0x%x\n", readl(plx_iobase + PLX_DMA0_LOCAL_ADDRESS_REG));
	DEBUG_PRINT(" plx dma channel 0 transfer size 0x%x\n", readl(plx_iobase + PLX_DMA0_TRANSFER_SIZE_REG));
	DEBUG_PRINT(" plx dma channel 0 descriptor 0x%x\n", readl(plx_iobase + PLX_DMA0_DESCRIPTOR_REG));
	DEBUG_PRINT(" plx dma channel 0 command status 0x%x\n", readb(plx_iobase + PLX_DMA0_CS_REG));
	DEBUG_PRINT(" plx dma channel 0 threshold 0x%x\n", readl(plx_iobase + PLX_DMA0_THRESHOLD_REG));

	DEBUG_PRINT(" plx queue control/status 0x%x\n", readl(plx_iobase + PLX_QUEUE_SC_REG));
	DEBUG_PRINT(" plx queue configuration 0x%x\n", readl(plx_iobase + PLX_QUEUE_CONFIG_REG));

	// disable interrupts
	writel(0, plx_iobase + PLX_INTRCS_REG);

	// disable dma channels
	writeb(0, plx_iobase + PLX_DMA0_CS_REG);
	writeb(0, plx_iobase + PLX_DMA1_CS_REG);

	// configure dma0 mode
	bits = 0;
	// enable ready input, not sure if this is necessary
	bits |= PLX_DMA_EN_READYIN_BIT;
	// enable BTERM# input, not sure if this is necessary
	bits |= PLX_EN_BTERM_BIT;
	// enable dma chaining
	bits |= PLX_EN_CHAIN_BIT;
	// enable interrupt on dma done (probably don't need this, since chain never finishes)
	bits |= PLX_EN_DMA_DONE_INTR_BIT;
	// don't increment local address during transfers (we are transferring from a fixed fifo register)
	bits |= PLX_LOCAL_ADDR_CONST_BIT;
	// route dma interrupt to pci bus
	bits |= PLX_DMA_INTR_PCI_BIT;
	// enable demand mode
	bits |= PLX_DEMAND_MODE_BIT;
	// enable local burst mode
	bits |= PLX_DMA_LOCAL_BURST_EN_BIT;
	// 4020 uses 32 bit dma
	if(board(dev)->layout == LAYOUT_4020)
		bits |= PLX_LOCAL_BUS_32_WIDE_BITS;
	else
		// localspace0 bus is 16 bits wide
		bits |= PLX_LOCAL_BUS_16_WIDE_BITS;
	writel(bits, plx_iobase + PLX_DMA0_MODE_REG);
}

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.
 */
static int attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	struct pci_dev* pcidev;
	int index;
	// base addresses
	unsigned long plx9080_iobase;
	unsigned long main_iobase;
	unsigned long dio_counter_iobase;
	uint32_t local_range, local_decode;
	unsigned long dio_8255_iobase;

	printk("comedi%d: cb_pcidas64\n",dev->minor);

/*
 * Allocate the private structure area.
 */
	if(alloc_private(dev,sizeof(pcidas64_private)) < 0)
		return -ENOMEM;

/*
 * Probe the device to determine what device in the series it is.
 */

	pci_for_each_dev(pcidev)
	{
		// is it not a computer boards card?
		if(pcidev->vendor != PCI_VENDOR_ID_COMPUTERBOARDS)
			continue;
#ifdef PCIDAS64_DEBUG
		printk(" found computer boards device id 0x%x on bus %i slot %i\n",
			pcidev->device, pcidev->bus->number, PCI_SLOT(pcidev->devfn));
#endif
		// loop through cards supported by this driver
		for(index = 0; index < N_BOARDS; index++)
		{
			if(pcidas64_boards[index].device_id != pcidev->device)
				continue;
			// was a particular bus/slot requested?
			if(it->options[0] || it->options[1])
			{
				// are we on the wrong bus/slot?
				if(pcidev->bus->number != it->options[0] ||
				   PCI_SLOT(pcidev->devfn) != it->options[1])
				{
					continue;
				}
			}
			dev->board_ptr = pcidas64_boards + index;
			goto found;
		}
	}

	printk("No supported ComputerBoards/MeasurementComputing card found\n");
	return -EIO;

found:

	printk("Found %s on bus %i, slot %i\n", pcidas64_boards[index].name,
		pcidev->bus->number, PCI_SLOT(pcidev->devfn));
	private(dev)->hw_dev = pcidev;

	//Initialize dev->board_name
	dev->board_name = board(dev)->name;

	if(pci_enable_device(pcidev))
		return -EIO;
	pci_set_master(pcidev);

	plx9080_iobase = pci_resource_start(pcidev, PLX9080_BADRINDEX);
	main_iobase = pci_resource_start(pcidev, MAIN_BADRINDEX);
	dio_counter_iobase = pci_resource_start(pcidev, DIO_COUNTER_BADRINDEX);

	if(pci_request_regions(pcidev, driver_cb_pcidas.driver_name))
	{
		/* Couldn't allocate io space */
		printk(KERN_WARNING " failed to allocate io memory\n");
		return -EIO;
	}

	private(dev)->plx9080_phys_iobase = plx9080_iobase;
	private(dev)->main_phys_iobase = main_iobase;
	private(dev)->dio_counter_phys_iobase = dio_counter_iobase;

	// remap, won't work with 2.0 kernels but who cares
	private(dev)->plx9080_iobase = (unsigned long)ioremap(plx9080_iobase, pci_resource_len(pcidev, PLX9080_BADRINDEX));
	private(dev)->main_iobase = (unsigned long)ioremap(main_iobase, pci_resource_len(pcidev, PLX9080_BADRINDEX));
	private(dev)->dio_counter_iobase = (unsigned long)ioremap(dio_counter_iobase, pci_resource_len(pcidev, PLX9080_BADRINDEX));

	DEBUG_PRINT(" plx9080 remapped to 0x%lx\n", private(dev)->plx9080_iobase);
	DEBUG_PRINT(" main remapped to 0x%lx\n", private(dev)->main_iobase);
	DEBUG_PRINT(" diocounter remapped to 0x%lx\n", private(dev)->dio_counter_iobase);

	// figure out what local addresses are
	local_range = readl(private(dev)->plx9080_iobase + PLX_LAS0RNG_REG) & LRNG_MEM_MASK;
	local_decode = readl(private(dev)->plx9080_iobase + PLX_LAS0MAP_REG) & local_range & LMAP_MEM_MASK ;
	private(dev)->local0_iobase = (private(dev)->main_phys_iobase & ~local_range) | local_decode;
	local_range = readl(private(dev)->plx9080_iobase + PLX_LAS1RNG_REG) & LRNG_MEM_MASK;
	local_decode = readl(private(dev)->plx9080_iobase + PLX_LAS1MAP_REG) & local_range & LMAP_MEM_MASK ;
	private(dev)->local1_iobase = (private(dev)->dio_counter_phys_iobase & ~local_range) | local_decode;

	DEBUG_PRINT(" local 0 io addr 0x%x\n", private(dev)->local0_iobase);
	DEBUG_PRINT(" local 1 io addr 0x%x\n", private(dev)->local1_iobase);

	private(dev)->hw_revision = HW_REVISION(readw(private(dev)->main_iobase + HW_STATUS_REG));
	if( board(dev)->layout == LAYOUT_4020)
		private(dev)->hw_revision >>= 1;

	printk(" stc hardware revision %i\n", private(dev)->hw_revision);

	init_plx9080(dev);

	// get irq
	if(comedi_request_irq(pcidev->irq, handle_interrupt, SA_SHIRQ, "cb_pcidas64", dev ))
	{
		printk(" unable to allocate irq %d\n", pcidev->irq);
		return -EINVAL;
	}
	dev->irq = pcidev->irq;

	printk(" irq %i\n", dev->irq);

	// alocate pci dma buffers
	for(index = 0; index < DMA_RING_COUNT; index++)
	{
		private(dev)->ai_buffer[index] =
			pci_alloc_consistent(private(dev)->hw_dev, DMA_TRANSFER_SIZE, &private(dev)->ai_buffer_phys_addr[index]);
	}
	// allocate dma descriptors
	private(dev)->dma_desc =
		pci_alloc_consistent(private(dev)->hw_dev, sizeof(struct plx_dma_desc) * DMA_RING_COUNT,
		&private(dev)->dma_desc_phys_addr);
	// initialize dma descriptors
	for(index = 0; index < DMA_RING_COUNT; index++)
	{
		private(dev)->dma_desc[index].pci_start_addr = private(dev)->ai_buffer_phys_addr[index];
		if(board(dev)->layout == LAYOUT_4020)
			private(dev)->dma_desc[index].local_start_addr = private(dev)->local1_iobase + ADC_FIFO_REG;
		else
			private(dev)->dma_desc[index].local_start_addr = private(dev)->local0_iobase + ADC_FIFO_REG;
		private(dev)->dma_desc[index].transfer_size = DMA_TRANSFER_SIZE;
		private(dev)->dma_desc[index].next = (private(dev)->dma_desc_phys_addr + ((index + 1) % (DMA_RING_COUNT)) * sizeof(private(dev)->dma_desc[0])) |
			PLX_DESC_IN_PCI_BIT | PLX_INTR_TERM_COUNT | PLX_XFER_LOCAL_TO_PCI;
	}

/*
 * Allocate the subdevice structures.
 */
	dev->n_subdevices = 9;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog input subdevice */
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND;
	if(board(dev)->layout != LAYOUT_4020)
		s->subdev_flags |= SDF_COMMON | SDF_DIFF;
	/* XXX Number of inputs in differential mode is ignored */
	s->n_chan = board(dev)->ai_se_chans;
	s->len_chanlist = 0x2000;
	s->maxdata = (1 << board(dev)->ai_bits) - 1;
	s->range_table = board(dev)->ai_range_table;
	s->insn_read = ai_rinsn;
	s->insn_config = ai_config_insn;
	s->do_cmd = ai_cmd;
	s->do_cmdtest = ai_cmdtest;
	s->cancel = ai_cancel;

	/* analog output subdevice */
	s = dev->subdevices + 1;
	if(board(dev)->ao_nchan)
	{
	//	dev->write_subdev = s;
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_GROUND;
		s->n_chan = board(dev)->ao_nchan;
		// analog out resolution is the same as analog input resolution, so use ai_bits
		s->maxdata = (1 << board(dev)->ai_bits) - 1;
		s->range_table = board(dev)->ao_range_table;
		s->insn_read = ao_readback_insn;
		s->insn_write = ao_winsn;
//XXX 4020 can't do paced analog output
	//	s->do_cmdtest = ao_cmdtest;
	//	s->do_cmd = ao_cmd;
	//	s->len_chanlist = board(dev)->ao_nchan;
	//	s->cancel = ao_cancel;
	} else
	{
		s->type = COMEDI_SUBD_UNUSED;
	}

	// digital input
	s = dev->subdevices + 2;
	if(board(dev)->layout == LAYOUT_64XX)
	{
		s->type = COMEDI_SUBD_DI;
		s->subdev_flags = SDF_READABLE;
		s->n_chan = 4;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_bits = di_rbits;
	} else
		s->type = COMEDI_SUBD_UNUSED;

	// digital output
	if(board(dev)->layout == LAYOUT_64XX)
	{
		s = dev->subdevices + 3;
		s->type = COMEDI_SUBD_DO;
		s->subdev_flags = SDF_WRITEABLE | SDF_READABLE;
		s->n_chan = 4;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_bits = do_wbits;
	} else
		s->type = COMEDI_SUBD_UNUSED;

	/* 8255 */
	s = dev->subdevices + 4;
	if(board(dev)->layout == LAYOUT_4020)
	{
		dio_8255_iobase = private(dev)->main_iobase + I8255_4020_REG;
		subdev_8255_init(dev, s, dio_callback_4020, dio_8255_iobase);
	} else
	{
		dio_8255_iobase = private(dev)->dio_counter_iobase + DIO_8255_OFFSET;
		subdev_8255_init(dev, s, dio_callback, dio_8255_iobase);
	}

	// 8 channel dio for 60xx
	s = dev->subdevices + 5;
	if(board(dev)->layout == LAYOUT_60XX)
	{
		s->type = COMEDI_SUBD_DIO;
		s->subdev_flags = SDF_WRITEABLE | SDF_READABLE;
		s->n_chan = 8;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_config = dio_60xx_config_insn;
		s->insn_bits = dio_60xx_wbits;
	} else
		s->type = COMEDI_SUBD_UNUSED;

	// calibration subd XXX
	s = dev->subdevices + 6;
	if(board(dev)->layout == LAYOUT_60XX)
	{
		s->type=COMEDI_SUBD_CALIB;
		s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_INTERNAL;
		s->n_chan = 8;	// XXX
		s->maxdata = 0xff;
//		s->insn_read = calib_read_insn;
		s->insn_write = calib_write_insn;
	}else
		s->type = COMEDI_SUBD_UNUSED;

	//serial EEPROM, if present
	s = dev->subdevices + 7;
	if(private(dev)->plx_control_bits & CTL_EECHK)
	{
		s->type = COMEDI_SUBD_MEMORY;
		s->subdev_flags = SDF_READABLE | SDF_INTERNAL;
		s->n_chan = 128;
		s->maxdata = 0xffff;
		s->insn_read = eeprom_read_insn;
	} else
		s->type = COMEDI_SUBD_UNUSED;
	// user counter subd XXX
	s = dev->subdevices + 8;
	s->type = COMEDI_SUBD_UNUSED;


	// manual says to set this bit for boards with > 16 channels
//	if(board(dev)->ai_se_chans > 16)
	if(1)	// bit should be set for 6025, although docs say 6034 and 6035 should be cleared XXX
		private(dev)->adc_control1_bits |= ADC_QUEUE_CONFIG_BIT;
	writew(private(dev)->adc_control1_bits, private(dev)->main_iobase + ADC_CONTROL1_REG);

	// initialize various registers
	writew(0, private(dev)->main_iobase + DAQ_SYNC_REG);
	writew(0, private(dev)->main_iobase + CALIBRATION_REG);

	// set fifos to maximum size
	private(dev)->fifo_size_bits = DAC_FIFO_BITS;
	switch(board(dev)->layout)
	{
		case LAYOUT_64XX:
			private(dev)->fifo_size_bits |= ADC_FIFO_64XX_BITS;
			break;
		case LAYOUT_60XX:
			private(dev)->fifo_size_bits |= ADC_FIFO_60XX_BITS;
			break;
		case LAYOUT_4020:
			private(dev)->fifo_size_bits |= ADC_FIFO_4020_BITS;
			break;
		default:
			printk(" bug! unknown register layout\n");
			return -1;
			break;
	}
	writew(private(dev)->fifo_size_bits, private(dev)->main_iobase + FIFO_SIZE_REG);

#if 0
{
int i;
for(i = 0; i < 8; i++)
	if(i == 0)
		caldac_8800_write(dev, i, 128);
	else
		caldac_8800_write(dev, i, 128);
}
#endif

	return 0;
}

/*
 * _detach is called to deconfigure a device.  It should deallocate
 * resources.
 * This function is also called when _attach() fails, so it should be
 * careful not to release resources that were not necessarily
 * allocated by _attach().  dev->private and dev->subdevices are
 * deallocated automatically by the core.
 */
static int detach(comedi_device *dev)
{
	unsigned int i;

	printk("comedi%d: cb_pcidas: remove\n",dev->minor);

	if(dev->irq)
		comedi_free_irq(dev->irq, dev);
	if(private(dev))
	{
		if(private(dev)->plx9080_iobase)
			iounmap((void*)private(dev)->plx9080_iobase);
		if(private(dev)->main_iobase)
			iounmap((void*)private(dev)->main_iobase);
		if(private(dev)->dio_counter_iobase)
			iounmap((void*)private(dev)->dio_counter_iobase);
		if(private(dev)->plx9080_phys_iobase ||
			private(dev)->main_phys_iobase || private(dev)->dio_counter_phys_iobase)
			pci_release_regions(private(dev)->hw_dev);
		// free pci dma buffers
		for(i = 0; i < DMA_RING_COUNT; i++)
		{
			if(private(dev)->ai_buffer[i])
				pci_free_consistent(private(dev)->hw_dev, DMA_TRANSFER_SIZE,
					private(dev)->ai_buffer[i], private(dev)->ai_buffer_phys_addr[i]);
		}
		// free dma descriptors
		if(private(dev)->dma_desc)
			pci_free_consistent(private(dev)->hw_dev, sizeof(struct plx_dma_desc) * DMA_RING_COUNT,
				private(dev)->dma_desc, private(dev)->dma_desc_phys_addr);
	}
	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices + 4);

	return 0;
}

static int ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	unsigned int bits = 0, n, i;
	const int timeout = 100;

	DEBUG_PRINT("chanspec 0x%x\n", insn->chanspec);

	// disable interrupts on plx 9080 XXX
	writel(0, private(dev)->plx9080_iobase + PLX_INTRCS_REG);

	// disable card's analog input interrupt sources
	private(dev)->intr_enable_bits &= ~EN_ADC_INTR_SRC_BIT & ~EN_ADC_DONE_INTR_BIT &
		~EN_ADC_ACTIVE_INTR_BIT & ~EN_ADC_STOP_INTR_BIT & ~EN_ADC_OVERRUN_BIT;
	writew(private(dev)->intr_enable_bits, private(dev)->main_iobase + INTR_ENABLE_REG);

	/* disable pacing, enable software triggering, etc */
	writew(ADC_DMA_DISABLE_BIT, private(dev)->main_iobase + ADC_CONTROL0_REG);
	private(dev)->adc_control1_bits &= ADC_QUEUE_CONFIG_BIT;
	if(insn->chanspec & CR_DITHER)
		private(dev)->adc_control1_bits |= ADC_DITHER_BIT;
	writew(private(dev)->adc_control1_bits, private(dev)->main_iobase + ADC_CONTROL1_REG);

	if(board(dev)->layout != LAYOUT_4020)
	{
		// use internal queue
		writew(SLOW_DAC_BIT, private(dev)->main_iobase + HW_CONFIG_REG);

		// load internal queue
		bits = 0;
		// set gain
		bits |= board(dev)->ai_range_bits[CR_RANGE(insn->chanspec)];
		// set single-ended / differential
		if(CR_AREF(insn->chanspec) == AREF_DIFF)
			bits |= ADC_DIFFERENTIAL_BIT;
		if(CR_AREF(insn->chanspec) == AREF_COMMON)
			bits |= ADC_COMMON_BIT;
		// ALT_SOURCE is internal calibration reference
		if(insn->chanspec & CR_ALT_SOURCE)
		{
			DEBUG_PRINT("reading calibration source\n");
			// internal reference reads on channel 0
			bits |= CHAN_BITS(0);
			writew(CHAN_BITS(0), private(dev)->main_iobase + ADC_QUEUE_HIGH_REG);
			// channel selects internal reference source to connect to channel 0
			writew(CAL_EN_BIT | CAL_SRC_BITS(private(dev)->calibration_source),
				private(dev)->main_iobase + CALIBRATION_REG);
		} else	// set channel
		{
			bits |= CHAN_BITS(CR_CHAN(insn->chanspec));
			// set stop channel
			writew(CHAN_BITS(CR_CHAN(insn->chanspec)), private(dev)->main_iobase + ADC_QUEUE_HIGH_REG);
			// make sure internal calibration source is turned off
			writew(0, private(dev)->main_iobase + CALIBRATION_REG);
		}
		// set start channel, and rest of settings
		writew(bits, private(dev)->main_iobase + ADC_QUEUE_LOAD_REG);
	}else
	{
		/* 4020 requires sample interval register to be set before writing to convert register.
		 * Using somewhat arbitrary setting of 4 master clock ticks = 0.1 usec */
		writew(0, private(dev)->main_iobase + ADC_SAMPLE_INTERVAL_UPPER_REG);
		writew(2, private(dev)->main_iobase + ADC_SAMPLE_INTERVAL_LOWER_REG);
	}

	// clear adc buffer
	writew(0, private(dev)->main_iobase + ADC_BUFFER_CLEAR_REG);

	for(n = 0; n < insn->n; n++)
	{
		/* trigger conversion, bits sent only matter for 4020 */
		writew(ADC_CONVERT_CHANNEL_4020_BITS(CR_CHAN(insn->chanspec)), private(dev)->main_iobase + ADC_CONVERT_REG);

		// wait for data
		for(i = 0; i < timeout; i++)
		{
			bits = readw(private(dev)->main_iobase + HW_STATUS_REG);
			DEBUG_PRINT(" pipe bits 0x%x\n", PIPE_FULL_BITS(bits));
			if(PIPE_FULL_BITS(bits))
				break;
			udelay(1);
			if(board(dev)->layout == LAYOUT_4020) break;	// XXX
		}
		DEBUG_PRINT(" looped %i times waiting for data\n", i);
		if(i == timeout)
		{
			comedi_error(dev, " analog input read insn timed out");
			printk(" status 0x%x\n", bits);
			return -ETIME;
		}
		if(board(dev)->layout == LAYOUT_4020)
			data[n] = readl(private(dev)->dio_counter_iobase + ADC_FIFO_REG) & 0xffff;
		else
			data[n] = readw(private(dev)->main_iobase + PIPE1_READ_REG);
	}

	return n;
}

static int ai_config_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int id = data[0];
	int source = data[1];
	static const int num_calibration_sources = 8;

	if(id != INSN_CONFIG_ALT_SOURCE) return -EINVAL;

	if(source >= num_calibration_sources)
		return -EINVAL;

	DEBUG_PRINT("setting calibration source to %i\n", source);
	private(dev)->calibration_source = source;

	return 1;
}

static int ai_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd)
{	int err = 0;
	int tmp;
	unsigned int tmp_arg, tmp_arg2;
	int i;
	int aref;
	unsigned int triggers;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src || tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	triggers = TRIG_TIMER;
	if(board(dev)->layout != LAYOUT_4020)
		triggers |= TRIG_FOLLOW;;
	cmd->scan_begin_src &= triggers;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src) err++;

	tmp = cmd->convert_src;
	if(board(dev)->layout == LAYOUT_4020)
		triggers = TRIG_NOW;
	else
		triggers = TRIG_TIMER | TRIG_EXT;
	cmd->convert_src &= triggers;
	if(!cmd->convert_src || tmp != cmd->convert_src) err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src) err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_EXT | TRIG_NONE;
	if(!cmd->stop_src || tmp != cmd->stop_src) err++;

	if(err) return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	// uniqueness check
	if(cmd->start_src != TRIG_NOW &&
		cmd->start_src != TRIG_EXT) err++;
	if(cmd->scan_begin_src != TRIG_TIMER &&
		cmd->scan_begin_src != TRIG_FOLLOW) err++;
	if(cmd->convert_src != TRIG_TIMER &&
		cmd->convert_src != TRIG_EXT &&
		cmd->convert_src != TRIG_NOW) err++;
	if(cmd->stop_src != TRIG_COUNT &&
		cmd->stop_src != TRIG_NONE &&
		cmd->stop_src != TRIG_EXT) err++;

	// compatibility check
	if(cmd->convert_src == TRIG_EXT &&
		cmd->scan_begin_src == TRIG_TIMER)
		err++;

	if(err) return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg != 0)
	{
		cmd->start_arg = 0;
		err++;
	}
	if(cmd->convert_src == TRIG_TIMER)
	{
		if(cmd->convert_arg < board(dev)->ai_speed)
		{
			cmd->convert_arg = board(dev)->ai_speed;
			err++;
		}
		if(cmd->scan_begin_src == TRIG_TIMER)
		{
			// if scans are timed faster than conversion rate allows
			if(cmd->convert_arg * cmd->chanlist_len > cmd->scan_begin_arg)
			{
				cmd->scan_begin_arg = cmd->convert_arg * cmd->chanlist_len;
				err++;
			}
		}
	}

	if(!cmd->chanlist_len)
	{
		cmd->chanlist_len = 1;
		err++;
	}
	if(cmd->scan_end_arg != cmd->chanlist_len)
	{
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}

	switch(cmd->stop_src)
	{
		case TRIG_EXT:
			if(cmd->stop_arg)
			{
				cmd->stop_arg = 0;
				err++;
			}
			break;
		case TRIG_COUNT:
			if(!cmd->stop_arg)
			{
				cmd->stop_arg = 1;
				err++;
			}
			break;
		case TRIG_NONE:
			if(cmd->stop_arg != 0)
			{
				cmd->stop_arg = 0;
				err++;
			}
			break;
		default:
			break;
	}

	if(err) return 3;

	/* step 4: fix up any arguments */

	if(cmd->convert_src == TRIG_TIMER)
	{
		tmp_arg = cmd->convert_arg;
		tmp_arg2 = cmd->scan_begin_arg;
		check_adc_timing(cmd);
		if(tmp_arg != cmd->convert_arg) err++;
		if(tmp_arg2 != cmd->scan_begin_arg) err++;
	}

	if(err) return 4;

	// make sure user is doesn't change analog reference mid chanlist
	if(cmd->chanlist)
	{
		aref = CR_AREF(cmd->chanlist[0]);
		for(i = 1; i < cmd->chanlist_len; i++)
		{
			if(aref != CR_AREF(cmd->chanlist[i]))
			{
				comedi_error(dev, "all elements in chanlist must use the same analog reference");
				err++;
				break;
			}
		}
		// XXX check 4020 chanlist
	}

	if(err) return 5;

	return 0;
}

static int ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	u32 bits;
	unsigned int convert_counter_value = 0;
	unsigned int scan_counter_value = 0;
	unsigned int i;

	// disable card's analog input interrupt sources
	private(dev)->intr_enable_bits &= ~EN_ADC_INTR_SRC_BIT & ~EN_ADC_DONE_INTR_BIT &
		~EN_ADC_ACTIVE_INTR_BIT & ~EN_ADC_STOP_INTR_BIT & ~EN_ADC_OVERRUN_BIT &
		~ADC_INTR_SRC_MASK;
	writew(private(dev)->intr_enable_bits, private(dev)->main_iobase + INTR_ENABLE_REG);

	/* disable pacing, triggering, etc */
	writew(ADC_DMA_DISABLE_BIT, private(dev)->main_iobase + ADC_CONTROL0_REG);
	private(dev)->adc_control1_bits &= ADC_QUEUE_CONFIG_BIT;
	writew(private(dev)->adc_control1_bits, private(dev)->main_iobase + ADC_CONTROL1_REG);

	// make sure internal calibration source is turned off
	writew(0, private(dev)->main_iobase + CALIBRATION_REG);

	// set hardware configuration register
	bits = 0;
	if(board(dev)->layout == LAYOUT_4020)
		bits |= INTERNAL_CLOCK_4020_BITS;
	// use external queue
	else bits |= EXT_QUEUE_BIT | SLOW_DAC_BIT;
	writew(bits, private(dev)->main_iobase + HW_CONFIG_REG);

	// set conversion pacing
	check_adc_timing(cmd);
	if(cmd->convert_src == TRIG_TIMER)
	{
		// supposed to load counter with desired divisor minus 3
		convert_counter_value = cmd->convert_arg / TIMER_BASE - 3;

		// set scan pacing
		if(cmd->scan_begin_src == TRIG_TIMER)
		{
			// figure out how long we need to delay at end of scan
			scan_counter_value = (cmd->scan_begin_arg - (cmd->convert_arg * (cmd->chanlist_len - 1)))
				/ TIMER_BASE;
		}else if(cmd->scan_begin_src == TRIG_FOLLOW)
		{
			scan_counter_value = cmd->convert_arg / TIMER_BASE;
		}
	// 4020 pacing
	}else if(cmd->convert_src == TRIG_NOW && cmd->scan_begin_src == TRIG_TIMER)
	{
		// supposed to load counter with desired divisor minus 2 for 4020
		convert_counter_value = cmd->scan_begin_arg / TIMER_BASE - 2;
		scan_counter_value = 0;
	}
	// load lower 16 bits if convert interval
	writew(convert_counter_value & 0xffff, private(dev)->main_iobase + ADC_SAMPLE_INTERVAL_LOWER_REG);
	DEBUG_PRINT("convert counter 0x%x\n", convert_counter_value);
	// load upper 8 bits of convert interval
	writew((convert_counter_value >> 16) & 0xff, private(dev)->main_iobase + ADC_SAMPLE_INTERVAL_UPPER_REG);
	// load lower 16 bits of scan delay
	writew(scan_counter_value & 0xffff, private(dev)->main_iobase + ADC_DELAY_INTERVAL_LOWER_REG);
	// load upper 8 bits of scan delay
	writew((scan_counter_value >> 16) & 0xff, private(dev)->main_iobase + ADC_DELAY_INTERVAL_UPPER_REG);
	DEBUG_PRINT("scan counter 0x%x\n", scan_counter_value);

	// load hardware conversion counter with non-zero value so it doesn't mess with us
	writew(1, private(dev)->main_iobase + ADC_COUNT_LOWER_REG);

	// set software count
	if(cmd->stop_src == TRIG_COUNT)
		private(dev)->ai_count = cmd->stop_arg * cmd->chanlist_len;

	if(board(dev)->layout != LAYOUT_4020)
	{
		/* XXX cannot write to queue fifo while dac fifo is being written to
		* ( need spinlock, or try to use internal queue instead */
		// clear queue pointer
		writew(0, private(dev)->main_iobase + ADC_QUEUE_CLEAR_REG);
		// load external queue
		for(i = 0; i < cmd->chanlist_len; i++)
		{
			bits = 0;
			// set channel
			bits |= CHAN_BITS(CR_CHAN(cmd->chanlist[i]));
			// set gain
			bits |= board(dev)->ai_range_bits[CR_RANGE(cmd->chanlist[i])];
			// set single-ended / differential
			if(CR_AREF(cmd->chanlist[i]) == AREF_DIFF)
				bits |= ADC_DIFFERENTIAL_BIT;
			if(CR_AREF(cmd->chanlist[i]) == AREF_COMMON)
				bits |= ADC_COMMON_BIT;
			// mark end of queue
			if(i == cmd->chanlist_len - 1)
				bits |= QUEUE_EOSCAN_BIT | QUEUE_EOSEQ_BIT;
			writew(bits, private(dev)->main_iobase + ADC_QUEUE_FIFO_REG);
		}
		// prime queue holding register
		writew(0, private(dev)->main_iobase + ADC_QUEUE_LOAD_REG);
	}

	// clear adc buffer
	writew(0, private(dev)->main_iobase + ADC_BUFFER_CLEAR_REG);

	private(dev)->dma_index = 0;

	// enable interrupts on plx 9080
	// XXX enabling more interrupt sources than are actually used
	bits = ICS_PIE | ICS_PLIE | ICS_PAIE | ICS_PDIE | ICS_LIE | ICS_LDIE | ICS_DMA0_E | ICS_DMA1_E | ICS_MBIE
		| ICS_RAE;	// XXX
	writel(bits, private(dev)->plx9080_iobase + PLX_INTRCS_REG);

	// enable interrupts
	private(dev)->intr_enable_bits |= EN_ADC_OVERRUN_BIT |
		EN_ADC_DONE_INTR_BIT | EN_ADC_ACTIVE_INTR_BIT;
	if(cmd->stop_src == TRIG_EXT)
		private(dev)->intr_enable_bits |= EN_ADC_STOP_INTR_BIT;
	// Use pio transfer and interrupt on end of conversion if TRIG_WAKE_EOS flag is set.
	if(cmd->flags & TRIG_WAKE_EOS)
	{
		if(board(dev)->layout == LAYOUT_4020)
			private(dev)->intr_enable_bits |= EN_ADC_INTR_SRC_BIT;	// XXX
		else
			private(dev)->intr_enable_bits |= ADC_INTR_EOSCAN_BITS | EN_ADC_INTR_SRC_BIT;
	}
	writew(private(dev)->intr_enable_bits, private(dev)->main_iobase + INTR_ENABLE_REG);
	DEBUG_PRINT("intr enable bits 0x%x\n", private(dev)->intr_enable_bits);

	/* set mode, allow conversions through software gate */
	private(dev)->adc_control1_bits |= SW_GATE_BIT;
	if(board(dev)->layout != LAYOUT_4020)
	{
		if(cmd->convert_src == TRIG_EXT)
			private(dev)->adc_control1_bits |= ADC_MODE_BITS(13);	// good old mode 13
		else
			private(dev)->adc_control1_bits |= ADC_MODE_BITS(8);	// mode 8.  What else could you need?
	} else
	{
		if(cmd->chanlist_len == 4)
			private(dev)->adc_control1_bits |= FOUR_CHANNEL_4020_BITS;
		else
			private(dev)->adc_control1_bits |= TWO_CHANNEL_4020_BITS;
		private(dev)->adc_control1_bits |= LO_CHANNEL_4020_BITS(CR_CHAN(cmd->chanlist[0]));
		private(dev)->adc_control1_bits |= HI_CHANNEL_4020_BITS(CR_CHAN(cmd->chanlist[cmd->chanlist_len - 1]));
		// this causes interrupt on end of scan to be disabled on 60xx?
		if(cmd->flags & TRIG_WAKE_EOS)
			private(dev)->adc_control1_bits |= ADC_DMA_DISABLE_BIT;
	}
	writew(private(dev)->adc_control1_bits, private(dev)->main_iobase + ADC_CONTROL1_REG);
	DEBUG_PRINT("control1 bits 0x%x\n", private(dev)->adc_control1_bits);

	if(cmd->flags & TRIG_WAKE_EOS)
		writeb(0, private(dev)->plx9080_iobase + PLX_DMA0_CS_REG);
	else
	{
		// give location of first dma descriptor
		bits = private(dev)->dma_desc_phys_addr | PLX_DESC_IN_PCI_BIT | PLX_INTR_TERM_COUNT | PLX_XFER_LOCAL_TO_PCI;;
		writel(bits, private(dev)->plx9080_iobase + PLX_DMA0_DESCRIPTOR_REG);
		// enable dma transfer
		writeb(PLX_DMA_EN_BIT | PLX_DMA_START_BIT, private(dev)->plx9080_iobase + PLX_DMA0_CS_REG);
	}

	/* enable pacing, triggering, etc */
	bits = ADC_ENABLE_BIT | ADC_SOFT_GATE_BITS | ADC_GATE_LEVEL_BIT;
	// set start trigger
	if(cmd->start_src == TRIG_EXT)
		bits |= ADC_START_TRIG_EXT_BITS;
	else if(cmd->start_src == TRIG_NOW)
		bits |= ADC_START_TRIG_SOFT_BITS;
	writew(bits, private(dev)->main_iobase + ADC_CONTROL0_REG);
	DEBUG_PRINT("control0 bits 0x%x\n", bits);

	// start aquisition
	writew(0, private(dev)->main_iobase + ADC_START_REG);
	DEBUG_PRINT("soft trig\n");

	DEBUG_PRINT("trying to start, hw status is 0x%x\n", readw(private(dev)->main_iobase + HW_STATUS_REG));

	return 0;
}

// read num_samples from 16 bit wide ai fifo
static void pio_drain_ai_fifo_16(comedi_device *dev, unsigned int num_samples)
{
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	unsigned int i;

	DEBUG_PRINT(" read %i samples from fifo\n", num_samples);

	private(dev)->ai_count -= num_samples;

	for(i = 0; i < num_samples; i++)
	{
		comedi_buf_put(async, readw(private(dev)->main_iobase + ADC_FIFO_REG));
	}
}

// read num_samples from 32 bit wide ai fifo of 4020
static void pio_drain_ai_fifo_32(comedi_device *dev, unsigned int num_samples)
{
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	unsigned int i;
	u32 fifo_data;

	private(dev)->ai_count -= num_samples;

	for(i = 0; i < num_samples / 2; i++)
	{
		fifo_data = readl(private(dev)->dio_counter_iobase + ADC_FIFO_REG);
		comedi_buf_put(async, fifo_data & 0xffff);
		comedi_buf_put(async, (fifo_data >> 16) & 0xffff);
	}
}

// figure out how many samples are in fifo and read them
static void pio_drain_ai_fifo(comedi_device *dev)
{
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	int read_segment, read_index, write_segment, write_index;
	const int num_fifo_segments = 4;
	unsigned int num_samples;

	// figure out how many samples we should read from board's fifo

	do
	{
		/* Get most significant bits.  Different boards encode the meaning of these bits
		* differently, so use a scheme that doesn't depend on encoding */
		read_segment = ADC_UPP_READ_PNTR_CODE(readw(private(dev)->main_iobase + PREPOST_REG));
		write_segment = ADC_UPP_WRITE_PNTR_CODE(readw(private(dev)->main_iobase + PREPOST_REG));
		// get least significant 15 bits
		read_index = readw(private(dev)->main_iobase + ADC_READ_PNTR_REG) & 0x7fff;
		write_index = readw(private(dev)->main_iobase + ADC_WRITE_PNTR_REG) & 0x7fff;

		/* if read and write pointers are not on the same fifo segment, read to the
		* end of the read segment */
		if(read_segment != write_segment)
			num_samples = (board(dev)->fifo_depth / num_fifo_segments) - read_index;
		else
			num_samples = write_index - read_index;

		// 4020 stores two samples per 32 bit fifo entry
		if(board(dev)->layout == LAYOUT_4020)
			num_samples *= 2;

		if(cmd->stop_src == TRIG_COUNT)
		{
			if(num_samples > private(dev)->ai_count)
			{
				num_samples = private(dev)->ai_count;
			}
		}

		if(board(dev)->layout == LAYOUT_4020)
			pio_drain_ai_fifo_32(dev, num_samples);
		else
			pio_drain_ai_fifo_16(dev, num_samples);

		if(cmd->stop_src == TRIG_COUNT && private(dev)->ai_count <= 0)
		{
			break;
		}

	} while (read_segment != write_segment);

	async->events |= COMEDI_CB_BLOCK;
}

static void handle_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev = d;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	int num_samples = 0;
	unsigned int i;
	unsigned int status;
	uint32_t plx_status;
	uint32_t plx_bits;
	unsigned int dma0_status;

	plx_status = readl(private(dev)->plx9080_iobase + PLX_INTRCS_REG);
	status = readw(private(dev)->main_iobase + HW_STATUS_REG);
	dma0_status = readb(private(dev)->plx9080_iobase + PLX_DMA0_CS_REG);

	DEBUG_PRINT(" isr hw status 0x%x\n", status);
	DEBUG_PRINT(" plx status 0x%x\n", plx_status);
	DEBUG_PRINT(" user counter 0x%x\n", readw(private(dev)->main_iobase + LOWER_XFER_REG));

	if((status &
		(ADC_INTR_PENDING_BIT | ADC_DONE_BIT | ADC_STOP_BIT |
		DAC_INTR_PENDING_BIT | DAC_DONE_BIT | EXT_INTR_PENDING_BIT)) == 0 &&
		(plx_status & (ICS_DMA0_A | ICS_DMA1_A | ICS_LDIA | ICS_LIA | ICS_PAIA | ICS_PDIA |
		ICS_MBIA(0) | ICS_MBIA(1) |ICS_MBIA(2) | ICS_MBIA(3))) == 0)
	{
		return;
	}

	async->events = 0;

	// check for fifo overrun
	if(status & ADC_OVERRUN_BIT)
	{
		ai_cancel(dev, s);
		async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
		comedi_error(dev, "fifo overrun");
	}

	if(plx_status & ICS_DMA0_A)
	{	// dma chan 0 interrupt
		DEBUG_PRINT("dma0 status 0x%x\n", dma0_status);
		// XXX possible race
		writeb((dma0_status & PLX_DMA_EN_BIT) | PLX_CLEAR_DMA_INTR_BIT, private(dev)->plx9080_iobase + PLX_DMA0_CS_REG);

		if(dma0_status & PLX_DMA_EN_BIT)
		{
			uint32_t next_transfer_addr;
			static const int timeout = 1000;
			int j;
			// loop until we have read all the transferred data
			for(next_transfer_addr = readl(private(dev)->plx9080_iobase + PLX_DMA0_PCI_ADDRESS_REG), j = 0;
				next_transfer_addr != private(dev)->ai_buffer_phys_addr[private(dev)->dma_index] && j < timeout;
				j++ )
			{
				// transfer data from dma buffer to comedi buffer
				num_samples = DMA_TRANSFER_SIZE / sizeof(private(dev)->ai_buffer[0][0]);
				if(cmd->stop_src == TRIG_COUNT)
				{
					if(num_samples > private(dev)->ai_count)
						num_samples = private(dev)->ai_count;
					private(dev)->ai_count -= num_samples;
				}
				for(i = 0; i < num_samples; i++)
				{
					comedi_buf_put(async, private(dev)->ai_buffer[private(dev)->dma_index][i]);
				}
				private(dev)->dma_index = (private(dev)->dma_index + 1) % DMA_RING_COUNT;
				DEBUG_PRINT("next buffer addr 0x%x\n", private(dev)->ai_buffer_phys_addr[private(dev)->dma_index]);
				DEBUG_PRINT("pci addr reg 0x%x\n", next_transfer_addr);
			}
			async->events |= COMEDI_CB_BLOCK;
		}
		DEBUG_PRINT(" cleared dma ch0 interrupt\n");
	}

	// pio transfer
	if((status & ADC_INTR_PENDING_BIT) && (dma0_status & PLX_DMA_EN_BIT) == 0)
	{
		pio_drain_ai_fifo(dev);
	}

	// clear possible plx9080 interrupt sources
	if(plx_status & ICS_LDIA)
	{ // clear local doorbell interrupt
		plx_bits = readl(private(dev)->plx9080_iobase + PLX_DBR_OUT_REG);
		writel(plx_bits, private(dev)->plx9080_iobase + PLX_DBR_OUT_REG);
		DEBUG_PRINT(" cleared local doorbell bits 0x%x\n", plx_bits);
	}
	if(plx_status & ICS_DMA1_A)
	{	// dma chan 1 interrupt
		writeb(PLX_CLEAR_DMA_INTR_BIT, private(dev)->plx9080_iobase + PLX_DMA1_CS_REG);
		DEBUG_PRINT(" cleared dma ch1 interrupt\n");
	}

	// if we are have all the data, then quit
	if((cmd->stop_src == TRIG_COUNT && private(dev)->ai_count <= 0) ||
		(cmd->stop_src == TRIG_EXT && (status & ADC_STOP_BIT)))
	{
		ai_cancel(dev, s);
		async->events |= COMEDI_CB_EOA;
	}

	comedi_event(dev, s, async->events);
	return;
}

static int ai_cancel(comedi_device *dev, comedi_subdevice *s)
{
	const int timeout = 10000;
	unsigned int dma_status, i;

	// disable ai interrupts
	private(dev)->intr_enable_bits &= ~EN_ADC_INTR_SRC_BIT & ~EN_ADC_DONE_INTR_BIT &
		~EN_ADC_ACTIVE_INTR_BIT & ~EN_ADC_STOP_INTR_BIT & ~EN_ADC_OVERRUN_BIT &
		~ADC_INTR_SRC_MASK;
	writew(private(dev)->intr_enable_bits, private(dev)->main_iobase + INTR_ENABLE_REG);

	/* disable pacing, triggering, etc */
	writew(ADC_DMA_DISABLE_BIT, private(dev)->main_iobase + ADC_CONTROL0_REG);
	private(dev)->adc_control1_bits &= ADC_QUEUE_CONFIG_BIT;
	writew(private(dev)->adc_control1_bits, private(dev)->main_iobase + ADC_CONTROL1_REG);

	// abort dma transfer if necessary XXX
	dma_status = readb(private(dev)->plx9080_iobase + PLX_DMA0_CS_REG);
	if((dma_status & PLX_DMA_EN_BIT) == 0)
		return 0;

	// wait to make sure done bit is zero
	for(i = 0; (dma_status & PLX_DMA_DONE_BIT) && i < timeout; i++)
	{
		dma_status = readb(private(dev)->plx9080_iobase + PLX_DMA0_CS_REG);
		udelay(1);
	}
	if(i == timeout)
	{
		comedi_error(dev, "cancel() timed out waiting for dma done clear");
		return 0;
	}
	// disable channel
	writeb(0, private(dev)->plx9080_iobase + PLX_DMA0_CS_REG);
	// abort channel
	writeb(PLX_DMA_ABORT_BIT, private(dev)->plx9080_iobase + PLX_DMA0_CS_REG);
	// wait for dma done bit
	dma_status = readb(private(dev)->plx9080_iobase + PLX_DMA0_CS_REG);
	for(i = 0; (dma_status & PLX_DMA_DONE_BIT) == 0 && i < timeout; i++)
	{
		udelay(1);
		dma_status = readb(private(dev)->plx9080_iobase + PLX_DMA0_CS_REG);
	}
	if(i == timeout)
		comedi_error(dev, "cancel() timed out waiting for dma done set");

	return 0;
}

static int ao_winsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int chan = CR_CHAN(insn->chanspec);
	int range = CR_RANGE(insn->chanspec);
	unsigned int bits;

	// do some initializing
	writew(DAC_ENABLE_BIT, private(dev)->main_iobase + DAC_CONTROL0_REG);

	// set range
	bits = DAC_OUTPUT_ENABLE_BIT;
	bits |= DAC_RANGE_BITS(chan, board(dev)->ao_range_code[range]);
	writew(bits, private(dev)->main_iobase + DAC_CONTROL1_REG);

	// clear buffer
	writew(0, private(dev)->main_iobase + DAC_BUFFER_CLEAR_REG);

	// write to channel
	writew(data[0], private(dev)->main_iobase + DAC_CONVERT_REG(chan));

	// remember output value
	private(dev)->ao_value[chan] = data[0];

	return 1;
}

static int ao_readback_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	data[0] = private(dev)->ao_value[CR_CHAN(insn->chanspec)];

	return 1;
}

static int dio_callback(int dir, int port, int data, unsigned long iobase)
{
	if(dir)
	{
		writeb(data, iobase + port);
		DEBUG_PRINT("wrote 0x%x to port %i\n", data, port);
		return 0;
	}else
	{
		return readb(iobase + port);
	}
}

static int dio_callback_4020(int dir, int port, int data, unsigned long iobase)
{
	if(dir)
	{
		writew(data, iobase + 2 * port);
		return 0;
	}else
	{
		return readw(iobase + 2 * port);
	}
}

static int di_rbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	lsampl_t bits;

	bits = readb(private(dev)->dio_counter_iobase + DI_REG);
	bits &= 0xf;
	data[1] = bits;
	data[0] = 0;

	return 2;
}

static int do_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	data[0] &= 0xf;
	// zero bits we are going to change
	s->state &= ~data[0];
	// set new bits
	s->state |= data[0] & data[1];

	writeb(s->state, private(dev)->dio_counter_iobase + DO_REG);

	data[1] = s->state;

	return 2;
}

static int dio_60xx_config_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	unsigned int mask;

	mask = 1 << CR_CHAN(insn->chanspec);

	switch(insn->data[0])
	{
		case COMEDI_INPUT:
			s->io_bits &= ~mask;
			break;
		case COMEDI_OUTPUT:
			s->io_bits |= mask;
			break;
		default:
			return -EINVAL;
	}

	writeb(s->io_bits, private(dev)->dio_counter_iobase + DIO_DIRECTION_60XX_REG);

	return 1;
}

static int dio_60xx_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	if(data[0])
	{
		s->state &= ~data[0];
		s->state |= (data[0] & data[1]);
		writeb(s->state, private(dev)->dio_counter_iobase + DIO_DATA_60XX_REG);
	}

	data[1] = readb( private(dev)->dio_counter_iobase + DIO_DATA_60XX_REG );

	return 2;
}

static int calib_write_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int channel = CR_CHAN(insn->chanspec);

	caldac_8800_write(dev, channel, data[0]);

	return 1;

}

// XXX
#if 0
static int calib_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
        data[0] = ;

        return 1;
}
#endif

static uint16_t read_eeprom(comedi_device *dev, uint8_t address)
{
	static const int bitstream_length = 11;
	static const int read_command = 0x6;
	unsigned int bitstream = (read_command << 8) | address;
	unsigned int bit;
	const int plx_control_addr = private(dev)->plx9080_iobase + PLX_CONTROL_REG;
	uint16_t value;
	static const int value_length = 16;

	udelay(1);
	private(dev)->plx_control_bits &= ~CTL_EE_CLK & ~CTL_EE_CS;
	writel(private(dev)->plx_control_bits, plx_control_addr);
	// activate serial eeprom
	udelay(1);
	private(dev)->plx_control_bits |= CTL_EE_CS;
	writel(private(dev)->plx_control_bits, plx_control_addr);

	// write read command and desired memory address
	for(bit = 1 << (bitstream_length - 1); bit; bit >>= 1)
	{
		// set bit to be written
		udelay(1);
		if(bitstream & bit)
			private(dev)->plx_control_bits |= CTL_EE_W;
		else
			private(dev)->plx_control_bits &= ~CTL_EE_W;
		writel(private(dev)->plx_control_bits, plx_control_addr);
		// clock in bit
		udelay(1);
		private(dev)->plx_control_bits |= CTL_EE_CLK;
		writel(private(dev)->plx_control_bits, plx_control_addr);
		udelay(1);
		private(dev)->plx_control_bits &= ~CTL_EE_CLK;
		writel(private(dev)->plx_control_bits, plx_control_addr);
	}
	// read back value from eeprom memory location
	value = 0;
	for(bit = 1 << (value_length - 1); bit; bit >>= 1)
	{
		// clock out bit
		udelay(1);
		private(dev)->plx_control_bits |= CTL_EE_CLK;
		writel(private(dev)->plx_control_bits, plx_control_addr);
		udelay(1);
		private(dev)->plx_control_bits &= ~CTL_EE_CLK;
		writel(private(dev)->plx_control_bits, plx_control_addr);
		udelay(1);
		if(readl(plx_control_addr) & CTL_EE_R)
			value |= bit;
	}

	// deactivate eeprom serial input
	udelay(1);
	private(dev)->plx_control_bits &= ~CTL_EE_CS;
	writel(private(dev)->plx_control_bits, plx_control_addr);

	return value;
}

static int eeprom_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
        data[0] = read_eeprom(dev, CR_CHAN(insn->chanspec));

        return 1;
}

/* utility function that rounds desired timing to an achievable time, and
 * sets cmd members appropriately.
 * adc paces conversions from master clock by dividing by (x + 3) where x is 24 bit number
 */
static void check_adc_timing(comedi_cmd *cmd)
{
	unsigned int convert_divisor = 0, scan_divisor;
	static const int max_counter_value = 0xffffff;	// board uses 24 bit counters for pacing
	static const int min_convert_divisor = 3;
	static const int max_convert_divisor = max_counter_value + min_convert_divisor;
	static const int min_scan_divisor_4020 = 2;
	unsigned long long max_scan_divisor, min_scan_divisor;

	if(cmd->convert_src == TRIG_TIMER)
	{
		convert_divisor = get_divisor(cmd->convert_arg, cmd->flags);
		if(convert_divisor > max_convert_divisor) convert_divisor = max_convert_divisor;
		if(convert_divisor < min_convert_divisor) convert_divisor = min_convert_divisor;
		cmd->convert_arg = convert_divisor * TIMER_BASE;
	}else if(cmd->convert_src == TRIG_NOW)
		cmd->convert_arg = 0;


	if(cmd->scan_begin_src == TRIG_TIMER)
	{
		scan_divisor = get_divisor(cmd->scan_begin_arg, cmd->flags);
		if(cmd->convert_src == TRIG_TIMER)
		{
			// XXX check for integer overflows
			min_scan_divisor = convert_divisor * cmd->chanlist_len;
			max_scan_divisor = (convert_divisor * cmd->chanlist_len - 1) + max_counter_value;
		}else
		{
			min_scan_divisor = min_scan_divisor_4020;
			max_scan_divisor = max_counter_value + min_scan_divisor;
		}
		if(scan_divisor > max_scan_divisor) scan_divisor = max_scan_divisor;
		if(scan_divisor < min_scan_divisor) scan_divisor = min_scan_divisor;
		cmd->scan_begin_arg = scan_divisor * TIMER_BASE;
	}

	return;
}

/* Gets nearest achievable timing given master clock speed, does not
 * take into account possible minimum/maximum divisor values.  Used
 * by other timing checking functions. */
static unsigned int get_divisor(unsigned int ns, unsigned int flags)
{
	unsigned int divisor;

	switch(flags & TRIG_ROUND_MASK)
	{
		case TRIG_ROUND_UP:
			divisor = (ns + TIMER_BASE - 1) / TIMER_BASE;
			break;
		case TRIG_ROUND_DOWN:
			divisor = ns / TIMER_BASE;
			break;
		case TRIG_ROUND_NEAREST:
		default:
			divisor = (ns + TIMER_BASE / 2) / TIMER_BASE;
			break;
	}

	return divisor;
}

/* pci-6025 8800 caldac:
 * address 0 == dac channel 0 offset
 * address 1 == dac channel 0 gain
 * address 2 == dac channel 1 offset
 * address 3 == dac channel 1 gain
 * address 4 == fine adc offset
 * address 5 == coarse adc offset
 * address 6 == coarse adc gain
 * address 7 == fine adc gain
 */
static int caldac_8800_write(comedi_device *dev, unsigned int address, uint8_t value)
{
	const int num_caldac_channels = 8;
	const int bitstream_length = 11;
	unsigned int bitstream = ((address & 0x7) << 8) | value;
	unsigned int bit, register_bits;

	if(address >= num_caldac_channels)
	{
		comedi_error(dev, "illegal caldac channel");
		return -1;
	}

	for(bit = 1 << (bitstream_length - 1); bit; bit >>= 1)
	{
		register_bits = SERIAL_CLOCK_BIT;
		if(bitstream & bit)
			register_bits |= SERIAL_DATA_IN_BIT;
		udelay(1);
		writew(register_bits, private(dev)->main_iobase + CALIBRATION_REG);
        }

	udelay(1);
	writew(SELECT_8800_BIT, private(dev)->main_iobase + CALIBRATION_REG);
	udelay(1);
	writew(0, private(dev)->main_iobase + CALIBRATION_REG);

	return 0;
}

#if 0
// 1590 doesn't seem to do anything.  Perhaps it is the actual primary ao chip.
static int dac_1590_write(comedi_device *dev, unsigned int dac_a, unsigned int dac_b)
{
	const int bitstream_length = 24;
	const int max_caldac_value = 0xfff;
	unsigned int bitstream = ((dac_a & 0xfff) << 12) | (dac_b & 0xfff);
	unsigned int bit, register_bits;

	if(dac_a > max_caldac_value || dac_b > max_caldac_value)
	{
		comedi_error(dev, "illegal 1590 caldac channel");
		return -1;
	}

        for(bit = 1 << (bitstream_length - 1); bit; bit >>= 1)
	{
		register_bits = SELECT_1590_BIT | SERIAL_CLOCK_BIT;
		if(bitstream & bit)
			register_bits |= SERIAL_DATA_IN_BIT;
		udelay(1);
		writew(register_bits, private(dev)->main_iobase + CALIBRATION_REG);
        }

	udelay(1);
	writew(0, private(dev)->main_iobase + CALIBRATION_REG);

	return 0;
}
#endif
