/*
    cb_pcidas64.c
    This is a driver for the ComputerBoards/MeasurementComputing PCI-DAS
    64xx, 60xx, and 4020 cards.

    Author:  Frank Mori Hess <fmhess@uiuc.edu>
    Copyright (C) 2001, 2002 Frank Mori Hess <fmhess@users.sourceforge.net>

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
Status: works, but no streaming analog output yet
Updated: 2002-06-30
Devices: [Measurement Computing] PCI-DAS6402/16 (cb_pcidas64),
  PCI-DAS6402/12, PCI-DAS64/M1/16, PCI-DAS64/M2/16,
  PCI-DAS64/M3/16, PCI-DAS6402/16/JR, PCI-DAS64/M1/16/JR,
  PCI-DAS64/M2/16/JR, PCI-DAS64/M3/16/JR, PCI-DAS64/M1/14,
  PCI-DAS64/M2/14, PCI-DAS64/M3/14, PCI-DAS6023E, PCI-DAS6025E, PCI-DAS6034E,
  PCI-DAS6035E, PCI-DAS4020/12

Configuration options:
   [0] - PCI bus of device (optional)
   [1] - PCI slot of device (optional)

Feel free to send and success/failure reports to Frank Hess.

Some devices are not identified because the PCI device IDs are not known.
*/

/*

TODO:
	command support for ao
	user counter subdevice
	there are a number of boards this driver will support when they are
		fully released, but does not yet since the pci device id numbers
		are not yet available.
	support prescaled 100khz clock for slow pacing (not available on 6000 series?)

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
#define DEBUG_PRINT(format, args...)  rt_printk(format , ## args )
#else
#define DEBUG_PRINT(format, args...)
#endif

#define TIMER_BASE 25	// 40MHz master clock
#define PRESCALED_TIMER_BASE	10000	// 100kHz 'prescaled' clock for slow aquisition, maybe I'll support this someday
#define DMA_BUFFER_SIZE 0x1000
/* maximum number of dma transfers we will chain together into a ring
 * (and the maximum number of dma buffers we maintain) */
#define DMA_RING_COUNT 64

/* PCI-DAS64xxx base addresses */

// indices of base address regions
#define PLX9080_BADRINDEX 0
#define MAIN_BADRINDEX 2
#define DIO_COUNTER_BADRINDEX 3

// priv(dev)->main_iobase registers
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
#define    DMA_CH_SELECT_BIT	0x8000	// bit selects channels 1/0 for analog input/output, otherwise 0/1
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
#define    ADC_SAMPLE_COUNTER_EN_BIT	0x1000	// enable hardware scan counter
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
#define    SELECT_8402_64XX_BIT	0x2
#define    SELECT_1590_60XX_BIT	0x2
/* calibration sources for 6025 are:
 *  0 : ground
 *  1 : 10V
 *  2 : 5V
 *  3 : 0.5V
 *  4 : 0.05V
 *  5 : ground or 0.005V?
 *  6 : dac channel 0
 *  7 : dac channel 1
 */
#define    CAL_SRC_BITS(x)	(((x) & 0xf) << 3)
#define    CAL_EN_64XX_BIT	0x40	// calibration enable for 64xx series
#define    SERIAL_DATA_IN_BIT	0x80
#define    SERIAL_CLOCK_BIT	0x100
#define    CAL_EN_60XX_BIT	0x200	// calibration enable for 60xx series
#define    CAL_GAIN_BIT	0x800
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
#define    ADC_SE_DIFF_BIT	0x1000	// single-ended/ differential bit
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
#define DAC_LSB_4020_REG( channel )	((0x70) + (4 * ((channel) & 0x1)))
#define DAC_MSB_4020_REG( channel )	((0x72) + (4 * ((channel) & 0x1)))
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

// I2C addresses for 4020
#define RANGE_CAL_I2C_ADDR	0x20
#define   ADC_SRC_BITS(x)	(((x) << 4) & ADC_SRC_MASK)	// input source
#define   ADC_SRC_MASK	0x70	// bits that set what source the adc converter measures
#define   ATTENUATE_BIT(channel)	(1 << ((channel) & 0x3))	// attenuate channel (+-5V input range)
#define CALDAC0_I2C_ADDR	0xc
#define CALDAC1_I2C_ADDR	0xd

#define I8255_4020_REG 0x48	// 8255 offset, for 4020 only
#define ADC_QUEUE_FIFO_REG	0x100	// external channel/gain queue, uses same bits as ADC_QUEUE_LOAD_REG
#define ADC_FIFO_REG 0x200	// adc data fifo

// priv(dev)->dio_counter_iobase registers
#define DIO_8255_OFFSET	0x0
#define DO_REG	0x20
#define DI_REG	0x28
#define DIO_DIRECTION_60XX_REG	0x40
#define DIO_DATA_60XX_REG	0x48

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
static int ai_range_bits_4020[] = 
{
	0x1,
	0x0,
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

typedef struct hw_fifo_info_struct
{
	unsigned int num_segments;
	unsigned int max_segment_length;
	unsigned int sample_packing_ratio;
	uint16_t fifo_size_reg_mask;
} hw_fifo_info_t;

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
	const hw_fifo_info_t *const ai_fifo;
	enum register_layout layout;	// different board families have slightly different registers
} pcidas64_board;

static const hw_fifo_info_t ai_fifo_4020 =
{
	num_segments: 2,
	max_segment_length: 0x8000,
	sample_packing_ratio: 2,
	fifo_size_reg_mask: 0x7f,
};

static const hw_fifo_info_t ai_fifo_64xx =
{
	num_segments: 4,
	max_segment_length: 0x800,
	sample_packing_ratio: 1,
	fifo_size_reg_mask: 0x3f,
};

static const hw_fifo_info_t ai_fifo_60xx =
{
	num_segments: 4,
	max_segment_length: 0x800,
	sample_packing_ratio: 1,
	fifo_size_reg_mask: 0x7f,
};

static const pcidas64_board pcidas64_boards[] =
{
	{
		name:		"pci-das6402/16",
		device_id:	0x1d,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	5000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ao_range_table:	&ao_ranges_64xx,
		ao_range_code:	ao_range_code_64xx,
		ai_fifo:	&ai_fifo_64xx,
	},
	{
		name:		"pci-das6402/12",	// XXX check
		device_id:	0x1e,
		ai_se_chans:	64,
		ai_bits:	12,
		ai_speed:	5000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ao_range_table:	&ao_ranges_64xx,
		ao_range_code:	ao_range_code_64xx,
		ai_fifo:	&ai_fifo_64xx,
	},
	{
		name:		"pci-das64/m1/16",
		device_id:	0x35,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	1000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ao_range_table:	&ao_ranges_64xx,
		ao_range_code:	ao_range_code_64xx,
		ai_fifo:	&ai_fifo_64xx,
	},
	{
		name:		"pci-das64/m2/16",
		device_id:	0x36,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	500,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ao_range_table:	&ao_ranges_64xx,
		ao_range_code:	ao_range_code_64xx,
		ai_fifo:	&ai_fifo_64xx,
	},
	{
		name:		"pci-das64/m3/16",
		device_id:	0x37,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	333,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ao_range_table:	&ao_ranges_64xx,
		ao_range_code:	ao_range_code_64xx,
		ai_fifo:	&ai_fifo_64xx,
	},
	{
		name:		"pci-das6023",
		device_id:	0x5d,
		ai_se_chans:	16,
		ai_bits:	12,
		ai_speed:	5000,
		ao_nchan: 0,
		ao_scan_speed:	100000,
		layout:	LAYOUT_60XX,
		ai_range_table:	&ai_ranges_60xx,
		ai_range_bits:	ai_range_bits_60xx,
		ao_range_table:	&ao_ranges_60xx,
		ao_range_code:	ao_range_code_60xx,
		ai_fifo:	&ai_fifo_60xx,
	},
	{
		name:		"pci-das6025",
		device_id:	0x5e,
		ai_se_chans:	16,
		ai_bits:	12,
		ai_speed:	5000,
		ao_nchan: 2,
		ao_scan_speed:	100000,
		layout:	LAYOUT_60XX,
		ai_range_table:	&ai_ranges_60xx,
		ai_range_bits:	ai_range_bits_60xx,
		ao_range_table:	&ao_ranges_60xx,
		ao_range_code:	ao_range_code_60xx,
		ai_fifo:	&ai_fifo_60xx,
	},
	{
		name:		"pci-das6034",
		device_id:	0x63,
		ai_se_chans:	16,
		ai_bits:	16,
		ai_speed:	5000,
		ao_nchan: 0,
		ao_scan_speed:	0,
		layout:	LAYOUT_60XX,
		ai_range_table:	&ai_ranges_60xx,
		ai_range_bits:	ai_range_bits_60xx,
		ai_fifo:	&ai_fifo_60xx,
	},
	{
		name:		"pci-das6035",
		device_id:	0x64,
		ai_se_chans:	16,
		ai_bits:	16,
		ai_speed:	5000,
		ao_nchan:	2,
		ao_scan_speed:	100000,
		layout:	LAYOUT_60XX,
		ai_range_table:	&ai_ranges_60xx,
		ai_range_bits:	ai_range_bits_60xx,
		ao_range_table:	&ao_ranges_60xx,
		ao_range_code:	ao_range_code_60xx,
		ai_fifo:	&ai_fifo_60xx,
	},
	{
		name:	"pci-das4020/12",
		device_id:	0x52,
		ai_se_chans:	4,
		ai_bits:	12,
		ai_speed:	50,
		ao_nchan:	2,
		ao_scan_speed:	0,	// no hardware pacing on ao
		layout:	LAYOUT_4020,
		ai_range_table:	&ai_ranges_4020,
		ai_range_bits:	NULL,
		ao_range_table:	&ao_ranges_4020,
		ao_range_code:	ao_range_code_4020,
		ai_fifo:	&ai_fifo_4020,
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
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ai_fifo:	ai_fifo_64xx,
	},
	{
		name:		"pci-das64/m1/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	1000,
		ao_nchan: 0,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ai_fifo:	ai_fifo_64xx,
	},
	{
		name:		"pci-das64/m2/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	500,
		ao_nchan: 0,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ai_fifo:	ai_fifo_64xx,
	},
	{
		name:		"pci-das64/m3/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	333,
		ao_nchan: 0,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ai_fifo:	ai_fifo_64xx,
	},
	{
		name:		"pci-das64/m1/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	1000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ai_fifo:	ai_fifo_64xx,
	},
	{
		name:		"pci-das64/m2/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	500,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ai_fifo:	ai_fifo_64xx,
	},
	{
		name:		"pci-das64/m3/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	333,
		ao_nchan: 2,
		ao_scan_speed:	10000,
		layout:	LAYOUT_64XX,
		ai_range_table:	&ai_ranges_64xx,
		ai_range_bits:	ai_range_bits_64xx,
		ai_fifo:	ai_fifo_64xx,
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
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x005d, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
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
	volatile unsigned int intr_enable_bits;	// last bits sent to INTR_ENABLE_REG register
	volatile uint16_t adc_control1_bits;	// last bits sent to ADC_CONTROL1_REG register
	volatile uint16_t fifo_size_bits;	// last bits sent to FIFO_SIZE_REG register
	volatile uint16_t hw_config_bits;	// last bits sent to HW_CONFIG_REG register
	volatile uint32_t plx_control_bits;	// last bits written to plx9080 control register
	volatile uint32_t plx_intcsr_bits;	// last bits written to plx interrupt control and status register
	volatile int calibration_source;	// index of calibration source readable through ai ch0
	volatile uint8_t i2c_cal_range_bits;	// bits written to i2c calibration/range register
	volatile unsigned int ext_trig_falling;	// configure digital triggers to trigger on falling edge
	// states of various devices stored to enable read-back
	unsigned int ad8402_state[2];
	unsigned int caldac_state[8];
	volatile unsigned ai_cmd_running : 1;
	unsigned int ai_fifo_segment_length;
} pcidas64_private;

/* inline function that makes it easier to
 * access the private structure.
 */
extern inline pcidas64_private* priv(comedi_device *dev)
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
static int calib_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int calib_write_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int ad8402_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int ad8402_write_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int eeprom_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static void check_adc_timing(comedi_cmd *cmd);
static unsigned int get_divisor(unsigned int ns, unsigned int flags);
static void i2c_write(comedi_device *dev, unsigned int address, const uint8_t *data, unsigned int length);
static int caldac_8800_write(comedi_device *dev, unsigned int address, uint8_t value);
//static int dac_1590_write(comedi_device *dev, unsigned int dac_a, unsigned int dac_b);
static int caldac_i2c_write(comedi_device *dev, unsigned int caldac_channel, unsigned int value);
static void abort_dma(comedi_device *dev, unsigned int channel);
static void disable_plx_interrupts( comedi_device *dev );
static int set_ai_fifo_size( comedi_device *dev, unsigned int num_samples );
static unsigned int ai_fifo_size( comedi_device *dev );
static int set_ai_fifo_segment_length( comedi_device *dev, unsigned int num_entries );

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_cb_pcidas);

// initialize plx9080 chip
static void init_plx9080(comedi_device *dev)
{
	uint32_t bits;
	unsigned long plx_iobase = priv(dev)->plx9080_iobase;

	priv(dev)->plx_control_bits = readl(priv(dev)->plx9080_iobase + PLX_CONTROL_REG);

	// plx9080 dump
	DEBUG_PRINT(" plx interrupt status 0x%x\n", readl(plx_iobase + PLX_INTRCS_REG));
	DEBUG_PRINT(" plx id bits 0x%x\n", readl(plx_iobase + PLX_ID_REG));
	DEBUG_PRINT(" plx control reg 0x%x\n", priv(dev)->plx_control_bits);

	DEBUG_PRINT(" plx revision 0x%x\n", readl(plx_iobase + PLX_REVISION_REG));
	DEBUG_PRINT(" plx dma channel 0 mode 0x%x\n", readl(plx_iobase + PLX_DMA0_MODE_REG));
	DEBUG_PRINT(" plx dma channel 1 mode 0x%x\n", readl(plx_iobase + PLX_DMA1_MODE_REG));
	DEBUG_PRINT(" plx dma channel 0 pci address 0x%x\n", readl(plx_iobase + PLX_DMA0_PCI_ADDRESS_REG));
	DEBUG_PRINT(" plx dma channel 0 local address 0x%x\n", readl(plx_iobase + PLX_DMA0_LOCAL_ADDRESS_REG));
	DEBUG_PRINT(" plx dma channel 0 transfer size 0x%x\n", readl(plx_iobase + PLX_DMA0_TRANSFER_SIZE_REG));
	DEBUG_PRINT(" plx dma channel 0 descriptor 0x%x\n", readl(plx_iobase + PLX_DMA0_DESCRIPTOR_REG));
	DEBUG_PRINT(" plx dma channel 0 command status 0x%x\n", readb(plx_iobase + PLX_DMA0_CS_REG));
	DEBUG_PRINT(" plx dma channel 0 threshold 0x%x\n", readl(plx_iobase + PLX_DMA0_THRESHOLD_REG));

	disable_plx_interrupts( dev );

	abort_dma(dev, 0);
	abort_dma(dev, 1);

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
	{
		bits |= PLX_LOCAL_BUS_32_WIDE_BITS;
	}else
	{	// localspace0 bus is 16 bits wide
		bits |= PLX_LOCAL_BUS_16_WIDE_BITS;
	}
	writel(bits, plx_iobase + PLX_DMA1_MODE_REG);
}

/* Allocate and initialize the subdevice structures.
 */
static int setup_subdevices(comedi_device *dev)
{
	comedi_subdevice *s;
	unsigned long dio_8255_iobase;

	dev->n_subdevices = 10;
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
	if(board(dev)->layout == LAYOUT_4020)
	{
		unsigned int i;
		uint8_t data;
		// set adc to read from inputs (not internal calibration sources)
		priv(dev)->i2c_cal_range_bits = ADC_SRC_BITS(4);
		// set channels to +-5 volt input ranges
		for( i = 0; i < s->n_chan; i++)
			priv(dev)->i2c_cal_range_bits |= ATTENUATE_BIT(i);
		data = priv(dev)->i2c_cal_range_bits;
		i2c_write(dev, RANGE_CAL_I2C_ADDR, &data, sizeof(data));
	}

	/* analog output subdevice */
	s = dev->subdevices + 1;
	if(board(dev)->ao_nchan)
	{
	//	dev->write_subdev = s;
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_READABLE | SDF_WRITABLE | SDF_GROUND;
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
		s->subdev_flags = SDF_WRITABLE | SDF_READABLE;
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
		dio_8255_iobase = priv(dev)->main_iobase + I8255_4020_REG;
		subdev_8255_init(dev, s, dio_callback_4020, dio_8255_iobase);
	} else
	{
		dio_8255_iobase = priv(dev)->dio_counter_iobase + DIO_8255_OFFSET;
		subdev_8255_init(dev, s, dio_callback, dio_8255_iobase);
	}

	// 8 channel dio for 60xx
	s = dev->subdevices + 5;
	if(board(dev)->layout == LAYOUT_60XX)
	{
		s->type = COMEDI_SUBD_DIO;
		s->subdev_flags = SDF_WRITABLE | SDF_READABLE;
		s->n_chan = 8;
		s->maxdata = 1;
		s->range_table = &range_digital;
		s->insn_config = dio_60xx_config_insn;
		s->insn_bits = dio_60xx_wbits;
	} else
		s->type = COMEDI_SUBD_UNUSED;

	// caldac
	s = dev->subdevices + 6;
	s->type=COMEDI_SUBD_CALIB;
	s->subdev_flags = SDF_READABLE | SDF_WRITABLE | SDF_INTERNAL;
	s->n_chan = 8;
	if(board(dev)->layout == LAYOUT_4020)
		s->maxdata = 0xfff;
	else
		s->maxdata = 0xff;
	s->insn_read = calib_read_insn;
	s->insn_write = calib_write_insn;

	// 2 channel ad8402 potentiometer
	s = dev->subdevices + 7;
	if(board(dev)->layout == LAYOUT_64XX)
	{
		s->type = COMEDI_SUBD_CALIB;
		s->subdev_flags = SDF_READABLE | SDF_WRITABLE | SDF_INTERNAL;
		s->n_chan = 2;
		s->insn_read = ad8402_read_insn;
		s->insn_write = ad8402_write_insn;
		s->maxdata = 0xff;
	} else
		s->type = COMEDI_SUBD_UNUSED;

	//serial EEPROM, if present
	s = dev->subdevices + 8;
	if(priv(dev)->plx_control_bits & CTL_EECHK)
	{
		s->type = COMEDI_SUBD_MEMORY;
		s->subdev_flags = SDF_READABLE | SDF_INTERNAL;
		s->n_chan = 128;
		s->maxdata = 0xffff;
		s->insn_read = eeprom_read_insn;
	} else
		s->type = COMEDI_SUBD_UNUSED;

	// user counter subd XXX
	s = dev->subdevices + 9;
	s->type = COMEDI_SUBD_UNUSED;

	return 0;
}

static void disable_plx_interrupts( comedi_device *dev )
{
	priv(dev)->plx_intcsr_bits = 0;
	writel(priv(dev)->plx_intcsr_bits, priv(dev)->plx9080_iobase + PLX_INTRCS_REG);
}

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.
 */
static int attach(comedi_device *dev, comedi_devconfig *it)
{
	struct pci_dev* pcidev;
	int index;
	uint32_t local_range, local_decode;
	int retval;

	printk("comedi%d: cb_pcidas64\n",dev->minor);

/*
 * Allocate the private structure area.
 */
	if(alloc_private(dev,sizeof(pcidas64_private)) < 0)
		return -ENOMEM;

/*
 * Probe the device to determine what device in the series it is.
 */

	pci_for_each_dev( pcidev )
	{
		// is it not a computer boards card?
		if( pcidev->vendor != PCI_VENDOR_ID_COMPUTERBOARDS )
			continue;
		// loop through cards supported by this driver
		for(index = 0; index < N_BOARDS; index++)
		{
			if( pcidas64_boards[index].device_id != pcidev->device )
				continue;
			// was a particular bus/slot requested?
			if( it->options[0] || it->options[1] )
			{
				// are we on the wrong bus/slot?
				if( pcidev->bus->number != it->options[0] ||
				   PCI_SLOT( pcidev->devfn ) != it->options[1] )
				{
					continue;
				}
			}
			dev->board_ptr = pcidas64_boards + index;
			break;
		}
		if( dev->board_ptr ) break;
	}

	if( dev->board_ptr == NULL )
	{
		printk("No supported ComputerBoards/MeasurementComputing card found\n");
		return -EIO;
	}

	printk("Found %s on bus %i, slot %i\n", board(dev)->name,
		pcidev->bus->number, PCI_SLOT(pcidev->devfn));

	if( pci_enable_device( pcidev ) )
		return -EIO;
	pci_set_master( pcidev );

	priv(dev)->hw_dev = pcidev;

	//Initialize dev->board_name
	dev->board_name = board(dev)->name;

	if( pci_request_regions( pcidev, driver_cb_pcidas.driver_name ) )
	{
		/* Couldn't allocate io space */
		printk(KERN_WARNING " failed to allocate io memory\n");
		return -EIO;
	}

	priv(dev)->plx9080_phys_iobase = pci_resource_start(pcidev, PLX9080_BADRINDEX);
	priv(dev)->main_phys_iobase = pci_resource_start(pcidev, MAIN_BADRINDEX);
	priv(dev)->dio_counter_phys_iobase = pci_resource_start(pcidev, DIO_COUNTER_BADRINDEX);

	// remap, won't work with 2.0 kernels but who cares
	priv(dev)->plx9080_iobase = (unsigned long)ioremap(priv(dev)->plx9080_phys_iobase,
		pci_resource_len(pcidev, PLX9080_BADRINDEX));
	priv(dev)->main_iobase = (unsigned long)ioremap(priv(dev)->main_phys_iobase,
		pci_resource_len(pcidev, PLX9080_BADRINDEX));
	priv(dev)->dio_counter_iobase = (unsigned long)ioremap(priv(dev)->dio_counter_phys_iobase,
		pci_resource_len(pcidev, PLX9080_BADRINDEX));

	DEBUG_PRINT(" plx9080 remapped to 0x%lx\n", priv(dev)->plx9080_iobase);
	DEBUG_PRINT(" main remapped to 0x%lx\n", priv(dev)->main_iobase);
	DEBUG_PRINT(" diocounter remapped to 0x%lx\n", priv(dev)->dio_counter_iobase);

	// get irq
	if(comedi_request_irq(pcidev->irq, handle_interrupt, SA_SHIRQ, "cb_pcidas64", dev ))
	{
		printk(" unable to allocate irq %d\n", pcidev->irq);
		return -EINVAL;
	}
	dev->irq = pcidev->irq;

	printk(" irq %i\n", dev->irq);

	// figure out what local addresses are
	local_range = readl(priv(dev)->plx9080_iobase + PLX_LAS0RNG_REG) & LRNG_MEM_MASK;
	local_decode = readl(priv(dev)->plx9080_iobase + PLX_LAS0MAP_REG) & local_range & LMAP_MEM_MASK ;
	priv(dev)->local0_iobase = (priv(dev)->main_phys_iobase & ~local_range) | local_decode;
	local_range = readl(priv(dev)->plx9080_iobase + PLX_LAS1RNG_REG) & LRNG_MEM_MASK;
	local_decode = readl(priv(dev)->plx9080_iobase + PLX_LAS1MAP_REG) & local_range & LMAP_MEM_MASK ;
	priv(dev)->local1_iobase = (priv(dev)->dio_counter_phys_iobase & ~local_range) | local_decode;

	DEBUG_PRINT(" local 0 io addr 0x%x\n", priv(dev)->local0_iobase);
	DEBUG_PRINT(" local 1 io addr 0x%x\n", priv(dev)->local1_iobase);

	priv(dev)->hw_revision = HW_REVISION(readw(priv(dev)->main_iobase + HW_STATUS_REG));
	if( board(dev)->layout == LAYOUT_4020)
		priv(dev)->hw_revision >>= 1;

	printk(" stc hardware revision %i\n", priv(dev)->hw_revision);

	init_plx9080(dev);

	// alocate pci dma buffers
	for(index = 0; index < DMA_RING_COUNT; index++)
	{
		priv(dev)->ai_buffer[index] =
			pci_alloc_consistent(priv(dev)->hw_dev, DMA_BUFFER_SIZE, &priv(dev)->ai_buffer_phys_addr[index]);
	}
	// allocate dma descriptors
	priv(dev)->dma_desc =
		pci_alloc_consistent(priv(dev)->hw_dev, sizeof(struct plx_dma_desc) * DMA_RING_COUNT,
		&priv(dev)->dma_desc_phys_addr);
	// initialize dma descriptors
	for(index = 0; index < DMA_RING_COUNT; index++)
	{
		priv(dev)->dma_desc[index].pci_start_addr = priv(dev)->ai_buffer_phys_addr[index];
		if(board(dev)->layout == LAYOUT_4020)
			priv(dev)->dma_desc[index].local_start_addr = priv(dev)->local1_iobase + ADC_FIFO_REG;
		else
			priv(dev)->dma_desc[index].local_start_addr = priv(dev)->local0_iobase + ADC_FIFO_REG;
//XXX
//		priv(dev)->dma_desc[index].transfer_size = DMA_TRANSFER_SIZE;
		priv(dev)->dma_desc[index].next = (priv(dev)->dma_desc_phys_addr + ((index + 1) % (DMA_RING_COUNT)) * sizeof(priv(dev)->dma_desc[0])) |
			PLX_DESC_IN_PCI_BIT | PLX_INTR_TERM_COUNT | PLX_XFER_LOCAL_TO_PCI;
	}

	retval = setup_subdevices(dev);
	if(retval < 0)
	{
		return retval;
	}

	// initialize various registers

	// manual says to set this bit for boards with > 16 channels
//	if(board(dev)->ai_se_chans > 16)
	if(1)	// bit should be set for 6025, although docs say 6034 and 6035 should be cleared XXX
		priv(dev)->adc_control1_bits |= ADC_QUEUE_CONFIG_BIT;
	writew(priv(dev)->adc_control1_bits, priv(dev)->main_iobase + ADC_CONTROL1_REG);

	priv(dev)->hw_config_bits = SLOW_DAC_BIT | DMA_CH_SELECT_BIT;
	if(board(dev)->layout == LAYOUT_4020)
		priv(dev)->hw_config_bits |= INTERNAL_CLOCK_4020_BITS;
	writew(priv(dev)->hw_config_bits, priv(dev)->main_iobase + HW_CONFIG_REG);

	writew(0, priv(dev)->main_iobase + DAQ_SYNC_REG);
	writew(0, priv(dev)->main_iobase + CALIBRATION_REG);

	// set fifos to maximum size
	priv(dev)->fifo_size_bits = DAC_FIFO_BITS;
	set_ai_fifo_segment_length( dev, board(dev)->ai_fifo->max_segment_length );

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
	if(priv(dev))
	{
		if( priv(dev)->hw_dev )
		{
			if(priv(dev)->plx9080_iobase)
			{
				disable_plx_interrupts( dev );
				iounmap((void*)priv(dev)->plx9080_iobase);
			}
			if(priv(dev)->main_iobase)
				iounmap((void*)priv(dev)->main_iobase);
			if(priv(dev)->dio_counter_iobase)
				iounmap((void*)priv(dev)->dio_counter_iobase);
			if(priv(dev)->plx9080_phys_iobase ||
				priv(dev)->main_phys_iobase || priv(dev)->dio_counter_phys_iobase)
				pci_release_regions( priv(dev)->hw_dev );
			// free pci dma buffers
			for(i = 0; i < DMA_RING_COUNT; i++)
			{
				if( priv(dev)->ai_buffer[i] )
					pci_free_consistent( priv(dev)->hw_dev, DMA_BUFFER_SIZE,
						priv(dev)->ai_buffer[i], priv(dev)->ai_buffer_phys_addr[i] );
			}
			// free dma descriptors
			if(priv(dev)->dma_desc)
				pci_free_consistent( priv(dev)->hw_dev, sizeof( struct plx_dma_desc ) * DMA_RING_COUNT,
					priv(dev)->dma_desc, priv(dev)->dma_desc_phys_addr );
			pci_disable_device( priv(dev)->hw_dev );
		}
	}
	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices + 4);

	return 0;
}

static int ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	unsigned int bits = 0, n, i;
	const int timeout = 100;
	unsigned int channel, range, aref;

	DEBUG_PRINT("chanspec 0x%x\n", insn->chanspec);
	channel = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);
	aref = CR_AREF(insn->chanspec);

	// disable card's analog input interrupt sources
	// 4020 generates dac done interrupts even though they are disabled
	priv(dev)->intr_enable_bits &= ~EN_ADC_INTR_SRC_BIT & ~EN_ADC_DONE_INTR_BIT &
		~EN_ADC_ACTIVE_INTR_BIT & ~EN_ADC_STOP_INTR_BIT & ~EN_ADC_OVERRUN_BIT;
	writew(priv(dev)->intr_enable_bits, priv(dev)->main_iobase + INTR_ENABLE_REG);

	/* disable pacing, enable software triggering, etc */
	writew(ADC_DMA_DISABLE_BIT, priv(dev)->main_iobase + ADC_CONTROL0_REG);
	priv(dev)->adc_control1_bits &= ADC_QUEUE_CONFIG_BIT;
	if(insn->chanspec & CR_DITHER)
		priv(dev)->adc_control1_bits |= ADC_DITHER_BIT;
	writew(priv(dev)->adc_control1_bits, priv(dev)->main_iobase + ADC_CONTROL1_REG);

	if(board(dev)->layout != LAYOUT_4020)
	{
		// use internal queue
		priv(dev)->hw_config_bits &= ~EXT_QUEUE_BIT;
		writew(priv(dev)->hw_config_bits, priv(dev)->main_iobase + HW_CONFIG_REG);

		// load internal queue
		bits = 0;
		// set gain
		bits |= board(dev)->ai_range_bits[CR_RANGE(insn->chanspec)];
		// set single-ended / differential
		if( ( board(dev)->layout == LAYOUT_64XX && aref != AREF_DIFF ) ||
			( board(dev)->layout == LAYOUT_60XX && aref == AREF_DIFF ) )
			bits |= ADC_SE_DIFF_BIT;
		if( aref == AREF_COMMON)
			bits |= ADC_COMMON_BIT;
		// ALT_SOURCE is internal calibration reference
		if(insn->chanspec & CR_ALT_SOURCE)
		{
			unsigned int cal_en_bit;

			DEBUG_PRINT("reading calibration source\n");
			if( board(dev)->layout == LAYOUT_60XX)
				cal_en_bit = CAL_EN_60XX_BIT;
			else
				cal_en_bit = CAL_EN_64XX_BIT;
			// select internal reference source to connect to channel 0
			writew(cal_en_bit | CAL_SRC_BITS(priv(dev)->calibration_source),
				priv(dev)->main_iobase + CALIBRATION_REG);
		} else
		{
			// make sure internal calibration source is turned off
			writew(0, priv(dev)->main_iobase + CALIBRATION_REG);
		}
		bits |= CHAN_BITS(channel);
		// set start channel, and rest of settings
		writew(bits, priv(dev)->main_iobase + ADC_QUEUE_LOAD_REG);
		// set stop channel
		writew(CHAN_BITS(channel), priv(dev)->main_iobase + ADC_QUEUE_HIGH_REG);
	}else
	{
		uint8_t old_cal_range_bits = priv(dev)->i2c_cal_range_bits;

		priv(dev)->i2c_cal_range_bits &= ~ADC_SRC_MASK;
		if(insn->chanspec & CR_ALT_SOURCE)
		{
			DEBUG_PRINT("reading calibration source\n");
			priv(dev)->i2c_cal_range_bits |= ADC_SRC_BITS(priv(dev)->calibration_source);
		} else
		{	//select BNC inputs
			priv(dev)->i2c_cal_range_bits |= ADC_SRC_BITS(4);
		}
		// select range
		if(ai_range_bits_4020[range])
			priv(dev)->i2c_cal_range_bits |= ATTENUATE_BIT(channel);
		else
			priv(dev)->i2c_cal_range_bits &= ~ATTENUATE_BIT(channel);
		// update calibration/range i2c register only if necessary, as it is very slow
		if(old_cal_range_bits != priv(dev)->i2c_cal_range_bits)
		{
			uint8_t i2c_data = priv(dev)->i2c_cal_range_bits;
			i2c_write(dev, RANGE_CAL_I2C_ADDR, &i2c_data, sizeof(i2c_data));
		}

		/* 4020 manual asks that sample interval register to be set before writing to convert register.
		 * Using somewhat arbitrary setting of 4 master clock ticks = 0.1 usec */
		writew(0, priv(dev)->main_iobase + ADC_SAMPLE_INTERVAL_UPPER_REG);
		writew(2, priv(dev)->main_iobase + ADC_SAMPLE_INTERVAL_LOWER_REG);
	}

	for(n = 0; n < insn->n; n++)
	{

		// clear adc buffer (inside loop for 4020 sake)
		writew(0, priv(dev)->main_iobase + ADC_BUFFER_CLEAR_REG);

		/* trigger conversion, bits sent only matter for 4020 */
		writew(ADC_CONVERT_CHANNEL_4020_BITS(CR_CHAN(insn->chanspec)), priv(dev)->main_iobase + ADC_CONVERT_REG);

		// wait for data
		for(i = 0; i < timeout; i++)
		{
			bits = readw(priv(dev)->main_iobase + HW_STATUS_REG);
			DEBUG_PRINT(" pipe bits 0x%x\n", PIPE_FULL_BITS(bits));
			if(board(dev)->layout == LAYOUT_4020)
			{
				if(readw(priv(dev)->main_iobase + ADC_WRITE_PNTR_REG))
					break;
			}else
			{
				if(PIPE_FULL_BITS(bits))
					break;
			}
			udelay(1);
		}
		DEBUG_PRINT(" looped %i times waiting for data\n", i);
		if(i == timeout)
		{
			comedi_error(dev, " analog input read insn timed out");
			rt_printk(" status 0x%x\n", bits);
			return -ETIME;
		}
		if(board(dev)->layout == LAYOUT_4020)
			data[n] = readl(priv(dev)->dio_counter_iobase + ADC_FIFO_REG) & 0xffff;
		else
			data[n] = readw(priv(dev)->main_iobase + PIPE1_READ_REG);
	}

	return n;
}

static int ai_config_calibration_source( comedi_device *dev, lsampl_t *data )
{
	static const int num_calibration_sources = 8;
	lsampl_t source = data[1];

	if(source >= num_calibration_sources)
	{
		printk( "invalid calibration source: %i\n", source );
		return -EINVAL;
	}

	DEBUG_PRINT("setting calibration source to %i\n", source);
	priv(dev)->calibration_source = source;

	return 2;
}

static int ai_config_block_size( comedi_device *dev, lsampl_t *data )
{
	int fifo_size;
	const hw_fifo_info_t *const fifo = board(dev)->ai_fifo;
	unsigned int block_size, requested_block_size;
	int retval;
	static const int bytes_per_sample = 2;

	requested_block_size = data[ 1 ];

	if( requested_block_size )
	{
		fifo_size = requested_block_size * fifo->num_segments / bytes_per_sample;

		retval = set_ai_fifo_size( dev, fifo_size );
		if( retval < 0 ) return retval;

	}

	block_size = ai_fifo_size( dev ) / fifo->num_segments * bytes_per_sample;

	data[ 1 ] = block_size;

	return 2;
}

static int ai_config_insn( comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int id = data[0];

	switch( id )
	{
		case INSN_CONFIG_ALT_SOURCE:
			return ai_config_calibration_source( dev, data );
			break;
		case INSN_CONFIG_BLOCK_SIZE:
			return ai_config_block_size( dev, data );
			break;
		default:
			return -EINVAL;
			break;
	}
	return -EINVAL;
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
		// check 4020 chanlist
		if( board(dev)->layout == LAYOUT_4020 )
		{
			unsigned int first_channel = CR_CHAN( cmd->chanlist[0] );
			for( i = 1; i < cmd->chanlist_len; i++ )
			{
				if( CR_CHAN( cmd->chanlist[ i ] ) != first_channel + i )
				{
					comedi_error( dev, "chanlist must use consecutive channels" );
					err++;
					break;
				}
			}
			if( cmd->chanlist_len == 3 )
			{
				comedi_error( dev, "chanlist cannot be 3 channels long, use 1, 2, or 4 channels" );
				err++;
			}
		}
	}

	if(err) return 5;

	return 0;
}

static int use_hw_sample_counter( comedi_cmd *cmd )
{
	static const int max_hardware_count = 0xffffff;

// disable for now until I work out a race
return 0;

	if( cmd->stop_src == TRIG_COUNT &&
		cmd->stop_arg <= max_hardware_count )
		return 1;
	else
		return 0;
}

static void setup_sample_counters( comedi_device *dev, comedi_cmd *cmd )
{
	if( cmd->stop_src == TRIG_COUNT )
	{
		// set software count
		priv(dev)->ai_count = cmd->stop_arg * cmd->chanlist_len;
	}

	// load hardware conversion counter
	if( use_hw_sample_counter( cmd ) )
	{
		writew( cmd->stop_arg & 0xffff, priv(dev)->main_iobase + ADC_COUNT_LOWER_REG);
		writew( ( cmd->stop_arg >> 16 ) & 0xff,
			priv(dev)->main_iobase + ADC_COUNT_UPPER_REG);
	} else
	{
		writew( 1, priv(dev)->main_iobase + ADC_COUNT_LOWER_REG);
	}
}

static inline unsigned int dma_transfer_size( comedi_device *dev )
{
	unsigned int num_samples;

	num_samples = priv(dev)->ai_fifo_segment_length * board(dev)->ai_fifo->sample_packing_ratio;
	if( num_samples > DMA_BUFFER_SIZE ) num_samples = DMA_BUFFER_SIZE;

	return num_samples;
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
	priv(dev)->intr_enable_bits &= ~EN_ADC_INTR_SRC_BIT & ~EN_ADC_DONE_INTR_BIT &
		~EN_ADC_ACTIVE_INTR_BIT & ~EN_ADC_STOP_INTR_BIT & ~EN_ADC_OVERRUN_BIT &
		~ADC_INTR_SRC_MASK;
	writew(priv(dev)->intr_enable_bits, priv(dev)->main_iobase + INTR_ENABLE_REG);

	/* disable pacing, triggering, etc */
	writew(ADC_DMA_DISABLE_BIT, priv(dev)->main_iobase + ADC_CONTROL0_REG);
	priv(dev)->adc_control1_bits &= ADC_QUEUE_CONFIG_BIT;
	writew(priv(dev)->adc_control1_bits, priv(dev)->main_iobase + ADC_CONTROL1_REG);

	// make sure internal calibration source is turned off
	writew(0, priv(dev)->main_iobase + CALIBRATION_REG);
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
	writew(convert_counter_value & 0xffff, priv(dev)->main_iobase + ADC_SAMPLE_INTERVAL_LOWER_REG);
	DEBUG_PRINT("convert counter 0x%x\n", convert_counter_value);
	// load upper 8 bits of convert interval
	writew((convert_counter_value >> 16) & 0xff, priv(dev)->main_iobase + ADC_SAMPLE_INTERVAL_UPPER_REG);
	// load lower 16 bits of scan delay
	writew(scan_counter_value & 0xffff, priv(dev)->main_iobase + ADC_DELAY_INTERVAL_LOWER_REG);
	// load upper 8 bits of scan delay
	writew((scan_counter_value >> 16) & 0xff, priv(dev)->main_iobase + ADC_DELAY_INTERVAL_UPPER_REG);
	DEBUG_PRINT("scan counter 0x%x\n", scan_counter_value);

	setup_sample_counters( dev, cmd );

	if(board(dev)->layout != LAYOUT_4020)
	{
		// use external queue
		priv(dev)->hw_config_bits |= EXT_QUEUE_BIT;
		writew(priv(dev)->hw_config_bits, priv(dev)->main_iobase + HW_CONFIG_REG);

		/* XXX cannot write to queue fifo while dac fifo is being written to
		* ( need spinlock, or try to use internal queue instead */
		// clear queue pointer
		writew(0, priv(dev)->main_iobase + ADC_QUEUE_CLEAR_REG);
		// load external queue
		for(i = 0; i < cmd->chanlist_len; i++)
		{
			bits = 0;
			// set channel
			bits |= CHAN_BITS(CR_CHAN(cmd->chanlist[i]));
			// set gain
			bits |= board(dev)->ai_range_bits[CR_RANGE(cmd->chanlist[i])];
			// set single-ended / differential
			if( ( board(dev)->layout == LAYOUT_64XX && CR_AREF(cmd->chanlist[i]) != AREF_DIFF ) ||
				( board(dev)->layout == LAYOUT_60XX && CR_AREF(cmd->chanlist[i]) == AREF_DIFF ) )
				bits |= ADC_SE_DIFF_BIT;
			if(CR_AREF(cmd->chanlist[i]) == AREF_COMMON)
				bits |= ADC_COMMON_BIT;
			// mark end of queue
			if(i == cmd->chanlist_len - 1)
				bits |= QUEUE_EOSCAN_BIT | QUEUE_EOSEQ_BIT;
			writew(bits, priv(dev)->main_iobase + ADC_QUEUE_FIFO_REG);
		}
		// prime queue holding register
		writew(0, priv(dev)->main_iobase + ADC_QUEUE_LOAD_REG);
	}else
	{
		uint8_t old_cal_range_bits = priv(dev)->i2c_cal_range_bits;

		priv(dev)->i2c_cal_range_bits &= ~ADC_SRC_MASK;
		//select BNC inputs
		priv(dev)->i2c_cal_range_bits |= ADC_SRC_BITS(4);
		// select ranges
		for(i = 0; i < cmd->chanlist_len; i++)
		{
			unsigned int channel = CR_CHAN(cmd->chanlist[i]);
			unsigned int range = CR_RANGE(cmd->chanlist[i]);

			if(ai_range_bits_4020[range])
				priv(dev)->i2c_cal_range_bits |= ATTENUATE_BIT(channel);
			else
				priv(dev)->i2c_cal_range_bits &= ~ATTENUATE_BIT(channel);
		}
		// update calibration/range i2c register only if necessary, as it is very slow
		if(old_cal_range_bits != priv(dev)->i2c_cal_range_bits)
		{
			uint8_t i2c_data = priv(dev)->i2c_cal_range_bits;
			i2c_write(dev, RANGE_CAL_I2C_ADDR, &i2c_data, sizeof(i2c_data));
		}
	}

	// clear adc buffer
	writew(0, priv(dev)->main_iobase + ADC_BUFFER_CLEAR_REG);

	priv(dev)->dma_index = 0;

	// enable interrupts on plx 9080
	priv(dev)->plx_intcsr_bits |= ICS_AERR | ICS_PERR | ICS_PIE | ICS_PLIE | ICS_PAIE | ICS_LIE | ICS_DMA1_E;
	writel(priv(dev)->plx_intcsr_bits, priv(dev)->plx9080_iobase + PLX_INTRCS_REG);

	// enable interrupts
	priv(dev)->intr_enable_bits |= EN_ADC_OVERRUN_BIT |
		EN_ADC_DONE_INTR_BIT | EN_ADC_ACTIVE_INTR_BIT;
	if(cmd->stop_src == TRIG_EXT)
		priv(dev)->intr_enable_bits |= EN_ADC_STOP_INTR_BIT;
	// Use pio transfer and interrupt on end of conversion if TRIG_WAKE_EOS flag is set.
	if(cmd->flags & TRIG_WAKE_EOS)
	{
		if(board(dev)->layout != LAYOUT_4020)
			priv(dev)->intr_enable_bits |= ADC_INTR_EOSCAN_BITS | EN_ADC_INTR_SRC_BIT;
		// 4020 doesn't support pio transfers except for fifo dregs
	}
	writew(priv(dev)->intr_enable_bits, priv(dev)->main_iobase + INTR_ENABLE_REG);
	DEBUG_PRINT("intr enable bits 0x%x\n", priv(dev)->intr_enable_bits);

	/* set mode, allow conversions through software gate */
	priv(dev)->adc_control1_bits |= SW_GATE_BIT;
	if(board(dev)->layout != LAYOUT_4020)
	{
		if(cmd->convert_src == TRIG_EXT)
			priv(dev)->adc_control1_bits |= ADC_MODE_BITS(13);	// good old mode 13
		else
			priv(dev)->adc_control1_bits |= ADC_MODE_BITS(8);	// mode 8.  What else could you need?
#if 0
		// this causes interrupt on end of scan to be disabled on 60xx?
		if(cmd->flags & TRIG_WAKE_EOS)
			priv(dev)->adc_control1_bits |= ADC_DMA_DISABLE_BIT;
#endif
	} else
	{
		if(cmd->chanlist_len == 4)
			priv(dev)->adc_control1_bits |= FOUR_CHANNEL_4020_BITS;
		else if(cmd->chanlist_len == 2)
			priv(dev)->adc_control1_bits |= TWO_CHANNEL_4020_BITS;
		priv(dev)->adc_control1_bits |= LO_CHANNEL_4020_BITS(CR_CHAN(cmd->chanlist[0]));
		priv(dev)->adc_control1_bits |= HI_CHANNEL_4020_BITS(CR_CHAN(cmd->chanlist[cmd->chanlist_len - 1]));
	}

	writew(priv(dev)->adc_control1_bits, priv(dev)->main_iobase + ADC_CONTROL1_REG);
	DEBUG_PRINT("control1 bits 0x%x\n", priv(dev)->adc_control1_bits);

	abort_dma(dev, 1);
	if((cmd->flags & TRIG_WAKE_EOS) == 0 ||
		board(dev)->layout == LAYOUT_4020)
	{
		unsigned long flags;

		// set dma transfer size
		for( i = 0; i < DMA_RING_COUNT; i++)
			priv(dev)->dma_desc[ i ].transfer_size = dma_transfer_size( dev );
		// give location of first dma descriptor
		bits = priv(dev)->dma_desc_phys_addr | PLX_DESC_IN_PCI_BIT | PLX_INTR_TERM_COUNT | PLX_XFER_LOCAL_TO_PCI;;
		writel(bits, priv(dev)->plx9080_iobase + PLX_DMA1_DESCRIPTOR_REG);

		// spinlock for plx dma control/status reg
		comedi_spin_lock_irqsave( &dev->spinlock, flags );
		// enable dma transfer
		writeb(PLX_DMA_EN_BIT | PLX_DMA_START_BIT | PLX_CLEAR_DMA_INTR_BIT, priv(dev)->plx9080_iobase + PLX_DMA1_CS_REG);
		comedi_spin_unlock_irqrestore( &dev->spinlock, flags );
	}

	/* enable pacing, triggering, etc */
	bits = ADC_ENABLE_BIT | ADC_SOFT_GATE_BITS | ADC_GATE_LEVEL_BIT;
	// set start trigger
	if(cmd->start_src == TRIG_EXT)
		bits |= ADC_START_TRIG_EXT_BITS;
	else if(cmd->start_src == TRIG_NOW)
		bits |= ADC_START_TRIG_SOFT_BITS;
	if( cmd->start_arg & CR_INVERT )
		bits |= ADC_START_TRIG_FALLING_BIT;
	if( use_hw_sample_counter( cmd ) )
		bits |= ADC_SAMPLE_COUNTER_EN_BIT;
	writew(bits, priv(dev)->main_iobase + ADC_CONTROL0_REG);
	DEBUG_PRINT("control0 bits 0x%x\n", bits);

	// start aquisition
	writew(0, priv(dev)->main_iobase + ADC_START_REG);
	DEBUG_PRINT("soft trig\n");

	priv(dev)->ai_cmd_running = 1;

	return 0;
}

// read num_samples from 16 bit wide ai fifo
static void pio_drain_ai_fifo_16(comedi_device *dev)
{
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned int i;
	uint16_t prepost_bits;
	int read_segment, read_index, write_segment, write_index;
	int num_samples;

	do
	{
		/* Get most significant bits (grey code).  Different boards use different code
		* so use a scheme that doesn't depend on encoding */
		prepost_bits = readw(priv(dev)->main_iobase + PREPOST_REG);
		// get least significant 15 bits
		read_index = readw(priv(dev)->main_iobase + ADC_READ_PNTR_REG) & 0x7fff;
		write_index = readw(priv(dev)->main_iobase + ADC_WRITE_PNTR_REG) & 0x7fff;

		/* if read and write pointers are not on the same fifo segment, read to the
		* end of the read segment */
		read_segment = ADC_UPP_READ_PNTR_CODE(prepost_bits);
		write_segment = ADC_UPP_WRITE_PNTR_CODE(prepost_bits);
		if(read_segment != write_segment)
			num_samples = priv(dev)->ai_fifo_segment_length - read_index;
		else
			num_samples = write_index - read_index;

		if(cmd->stop_src == TRIG_COUNT)
		{
			if(priv(dev)->ai_count == 0) break;
			if(num_samples > priv(dev)->ai_count)
			{
				num_samples = priv(dev)->ai_count;
			}
			priv(dev)->ai_count -= num_samples;
		}

		if(num_samples < 0)
		{
			rt_printk(" cb_pcidas64: bug! num_samples < 0\n");
			break;
		}

		DEBUG_PRINT(" read %i samples from fifo\n", num_samples);

		for(i = 0; i < num_samples; i++)
		{
			comedi_buf_put(async, readw(priv(dev)->main_iobase + ADC_FIFO_REG));
		}

	} while (read_segment != write_segment);
}

/* Read from 32 bit wide ai fifo of 4020 - deal with insane grey coding of pointers.
 * The pci-4020 hardware only supports
 * dma transfers (it only supports the use of pio for draining the last remaining
 * points from the fifo when a data aquisition operation has completed).
 */
static void pio_drain_ai_fifo_32(comedi_device *dev)
{
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned int i;
	unsigned int max_transfer = 1e5;
	uint32_t fifo_data;
	int write_code = readw(priv(dev)->main_iobase + ADC_WRITE_PNTR_REG) & 0x7fff;
	int read_code = readw(priv(dev)->main_iobase + ADC_READ_PNTR_REG) & 0x7fff;

	if(cmd->stop_src == TRIG_COUNT)
	{
		if(max_transfer > priv(dev)->ai_count)
		{
			max_transfer = priv(dev)->ai_count;
		}
	}
	for(i = 0; read_code != write_code && i < max_transfer; )
	{
		fifo_data = readl(priv(dev)->dio_counter_iobase + ADC_FIFO_REG);
		comedi_buf_put(async, fifo_data & 0xffff);
		i++;
		if(i < max_transfer)
		{
			comedi_buf_put(async, (fifo_data >> 16) & 0xffff);
			i++;
		}
		read_code = readw(priv(dev)->main_iobase + ADC_READ_PNTR_REG) & 0x7fff;
	}
	priv(dev)->ai_count -= i;
}

// empty fifo
static void pio_drain_ai_fifo(comedi_device *dev)
{
	if(board(dev)->layout == LAYOUT_4020)
	{
		pio_drain_ai_fifo_32(dev);
	}else
		pio_drain_ai_fifo_16(dev);
}

static void drain_dma_buffers(comedi_device *dev, unsigned int channel)
{
	comedi_async *async = dev->read_subdev->async;
	uint32_t next_transfer_addr;
	int j;
	int num_samples = 0;
	unsigned long pci_addr_reg;

	if(channel)
		pci_addr_reg = priv(dev)->plx9080_iobase + PLX_DMA1_PCI_ADDRESS_REG;
	else
		pci_addr_reg = priv(dev)->plx9080_iobase + PLX_DMA0_PCI_ADDRESS_REG;

	// loop until we have read all the full buffers
	j = 0;
	for(next_transfer_addr = readl(pci_addr_reg);
		(next_transfer_addr < priv(dev)->ai_buffer_phys_addr[priv(dev)->dma_index] ||
		next_transfer_addr >= priv(dev)->ai_buffer_phys_addr[priv(dev)->dma_index] + DMA_BUFFER_SIZE) &&
		j < DMA_RING_COUNT;
		j++ )
	{
		// transfer data from dma buffer to comedi buffer
		num_samples = dma_transfer_size( dev );
		if(async->cmd.stop_src == TRIG_COUNT)
		{
			if(num_samples > priv(dev)->ai_count)
				num_samples = priv(dev)->ai_count;
			priv(dev)->ai_count -= num_samples;
		}
		comedi_buf_put_array(async, priv(dev)->ai_buffer[priv(dev)->dma_index], num_samples);
		priv(dev)->dma_index = (priv(dev)->dma_index + 1) % DMA_RING_COUNT;

		DEBUG_PRINT("next buffer addr 0x%lx\n", (unsigned long) priv(dev)->ai_buffer_phys_addr[priv(dev)->dma_index]);
		DEBUG_PRINT("pci addr reg 0x%x\n", next_transfer_addr);
	}
	// XXX check for buffer overrun somehow
}

static void handle_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev = d;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned int status;
	uint32_t plx_status;
	uint32_t plx_bits;
	uint8_t dma0_status, dma1_status;
	unsigned long flags;

	plx_status = readl(priv(dev)->plx9080_iobase + PLX_INTRCS_REG);
	status = readw(priv(dev)->main_iobase + HW_STATUS_REG);

	DEBUG_PRINT("cb_pcidas64: hw status 0x%x ", status);
	DEBUG_PRINT("plx status 0x%x\n", plx_status);

	async->events = 0;

	// check for fifo overrun
	if(status & ADC_OVERRUN_BIT)
	{
		comedi_error(dev, "fifo overrun");
		async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
	}

	// spin lock makes sure noone else changes plx dma control reg
	comedi_spin_lock_irqsave( &dev->spinlock, flags );
	dma0_status = readb(priv(dev)->plx9080_iobase + PLX_DMA0_CS_REG);
	if(plx_status & ICS_DMA0_A)
	{	// dma chan 0 interrupt
		// XXX possible race
		writeb((dma0_status & PLX_DMA_EN_BIT) | PLX_CLEAR_DMA_INTR_BIT, priv(dev)->plx9080_iobase + PLX_DMA0_CS_REG);

		DEBUG_PRINT("dma0 status 0x%x\n", dma0_status);

		DEBUG_PRINT(" cleared dma ch0 interrupt\n");
	}
	comedi_spin_unlock_irqrestore( &dev->spinlock, flags );

	// spin lock makes sure noone else changes plx dma control reg
	comedi_spin_lock_irqsave( &dev->spinlock, flags );
	dma1_status = readb(priv(dev)->plx9080_iobase + PLX_DMA1_CS_REG);
	if(plx_status & ICS_DMA1_A)	// XXX
	{	// dma chan 1 interrupt
		// XXX possible race
		writeb((dma1_status & PLX_DMA_EN_BIT) | PLX_CLEAR_DMA_INTR_BIT, priv(dev)->plx9080_iobase + PLX_DMA1_CS_REG);
		DEBUG_PRINT("dma1 status 0x%x\n", dma1_status);

		if(dma1_status & PLX_DMA_EN_BIT)
		{
			drain_dma_buffers(dev, 1);
		}
		DEBUG_PRINT(" cleared dma ch1 interrupt\n");
	}
	comedi_spin_unlock_irqrestore( &dev->spinlock, flags );

	// clear possible plx9080 interrupt sources
	if(plx_status & ICS_LDIA)
	{ // clear local doorbell interrupt
		plx_bits = readl(priv(dev)->plx9080_iobase + PLX_DBR_OUT_REG);
		writel(plx_bits, priv(dev)->plx9080_iobase + PLX_DBR_OUT_REG);
		DEBUG_PRINT(" cleared local doorbell bits 0x%x\n", plx_bits);
	}

	// clean up dregs if we were stopped by hardware sample counter
	if( status & ADC_DONE_BIT )
	{
		DEBUG_PRINT("adc done interrupt\n");
		if( priv(dev)->ai_cmd_running )
			pio_drain_ai_fifo(dev);
	}

	// if we are have all the data, then quit
	if((cmd->stop_src == TRIG_COUNT && priv(dev)->ai_count <= 0) ||
		(cmd->stop_src == TRIG_EXT && (status & ADC_STOP_BIT)))
	{
		async->events |= COMEDI_CB_EOA;
	}

	if( ( async->events & COMEDI_CB_EOA) )
		ai_cancel(dev, s);

	comedi_event(dev, s, async->events);

	DEBUG_PRINT("exiting handler\n");

	return;
}

void abort_dma(comedi_device *dev, unsigned int channel)
{
	unsigned long dma_cs_addr;
	uint8_t dma_status;
	const int timeout = 10000;
	unsigned int i;
	unsigned long flags;

	if(channel)
		dma_cs_addr = priv(dev)->plx9080_iobase + PLX_DMA1_CS_REG;
	else
		dma_cs_addr = priv(dev)->plx9080_iobase + PLX_DMA0_CS_REG;

	// spinlock for plx dma control/status reg
	comedi_spin_lock_irqsave( &dev->spinlock, flags );

	// abort dma transfer if necessary
	dma_status = readb(dma_cs_addr);
	if((dma_status & PLX_DMA_EN_BIT) == 0)
	{
		comedi_spin_unlock_irqrestore( &dev->spinlock, flags );
		return;
	}

	// wait to make sure done bit is zero
	for(i = 0; (dma_status & PLX_DMA_DONE_BIT) && i < timeout; i++)
	{
		dma_status = readb(dma_cs_addr);
		udelay(1);
	}
	if(i == timeout)
	{
		rt_printk("cb_pcidas64: cancel() timed out waiting for dma %i done clear\n", channel);
		comedi_spin_unlock_irqrestore( &dev->spinlock, flags );
		return;
	}
	// disable channel
	writeb(0, dma_cs_addr);
	udelay(1);
	// abort channel
	writeb(PLX_DMA_ABORT_BIT, dma_cs_addr);
	// wait for dma done bit
	dma_status = readb(dma_cs_addr);
	for(i = 0; (dma_status & PLX_DMA_DONE_BIT) == 0 && i < timeout; i++)
	{
		udelay(1);
		dma_status = readb(dma_cs_addr);
	}
	if(i == timeout)
		rt_printk("cb_pcidas64: cancel() timed out waiting for dma %i done set\n", channel);

	comedi_spin_unlock_irqrestore( &dev->spinlock, flags );
}

static int ai_cancel(comedi_device *dev, comedi_subdevice *s)
{
	if( priv(dev)->ai_cmd_running == 0 ) return 0;

	priv(dev)->ai_cmd_running = 0;

	// disable ai interrupts
	priv(dev)->intr_enable_bits &= ~EN_ADC_INTR_SRC_BIT & ~EN_ADC_DONE_INTR_BIT &
		~EN_ADC_ACTIVE_INTR_BIT & ~EN_ADC_STOP_INTR_BIT & ~EN_ADC_OVERRUN_BIT &
		~ADC_INTR_SRC_MASK;
	writew(priv(dev)->intr_enable_bits, priv(dev)->main_iobase + INTR_ENABLE_REG);

	abort_dma(dev, 1);

	/* disable pacing, triggering, etc */
	writew(ADC_DMA_DISABLE_BIT, priv(dev)->main_iobase + ADC_CONTROL0_REG);
	priv(dev)->adc_control1_bits &= ADC_QUEUE_CONFIG_BIT;
	writew(priv(dev)->adc_control1_bits, priv(dev)->main_iobase + ADC_CONTROL1_REG);

	return 0;
}

static int ao_winsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int chan = CR_CHAN(insn->chanspec);
	int range = CR_RANGE(insn->chanspec);
	unsigned int bits;

	// do some initializing
	writew(0, priv(dev)->main_iobase + DAC_CONTROL0_REG);

	// set range
	bits = DAC_OUTPUT_ENABLE_BIT;
	bits |= DAC_RANGE_BITS(chan, board(dev)->ao_range_code[range]);
	writew(bits, priv(dev)->main_iobase + DAC_CONTROL1_REG);

	// clear buffer
	writew(0, priv(dev)->main_iobase + DAC_BUFFER_CLEAR_REG);

	// write to channel
	if( board(dev)->layout == LAYOUT_4020 )
	{
		writew( data[0] & 0xff , priv(dev)->main_iobase + DAC_LSB_4020_REG(chan));
		writew( (data[0] >> 8) & 0xf , priv(dev)->main_iobase + DAC_MSB_4020_REG(chan));
	}else
	{
		writew(data[0], priv(dev)->main_iobase + DAC_CONVERT_REG(chan));
	}

	// remember output value
	priv(dev)->ao_value[chan] = data[0];

	return 1;
}

static int ao_readback_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	data[0] = priv(dev)->ao_value[CR_CHAN(insn->chanspec)];

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

	bits = readb(priv(dev)->dio_counter_iobase + DI_REG);
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

	writeb(s->state, priv(dev)->dio_counter_iobase + DO_REG);

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

	writeb(s->io_bits, priv(dev)->dio_counter_iobase + DIO_DIRECTION_60XX_REG);

	return 1;
}

static int dio_60xx_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	if(data[0])
	{
		s->state &= ~data[0];
		s->state |= (data[0] & data[1]);
		writeb(s->state, priv(dev)->dio_counter_iobase + DIO_DATA_60XX_REG);
	}

	data[1] = readb( priv(dev)->dio_counter_iobase + DIO_DATA_60XX_REG );

	return 2;
}

static int calib_write_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	int channel = CR_CHAN(insn->chanspec);

	priv(dev)->caldac_state[channel] = data[0];

	switch(board(dev)->layout)
	{
		case LAYOUT_60XX:
		case LAYOUT_64XX:
			caldac_8800_write(dev, channel, data[0]);
			break;
		case LAYOUT_4020:
			caldac_i2c_write(dev, channel, data[0]);
			break;
		default:
			break;
	}

	return 1;
}

static int calib_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	unsigned int channel = CR_CHAN(insn->chanspec);

        data[0] = priv(dev)->caldac_state[channel];

        return 1;
}

/* for pci-das6402/16, channel 0 is analog input gain and channel 1 is offset */
static int ad8402_write_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	static const int bitstream_length = 10;
	int channel = CR_CHAN(insn->chanspec);
	unsigned int bitstream = ((channel & 0x3) << 8) | (data[0] & 0xff);
	unsigned int bit, register_bits;
	static const int ad8402_udelay = 1;

	priv(dev)->ad8402_state[channel] = data[0];

	register_bits = SELECT_8402_64XX_BIT;
	udelay(ad8402_udelay);
	writew(register_bits, priv(dev)->main_iobase + CALIBRATION_REG);

	for(bit = 1 << (bitstream_length - 1); bit; bit >>= 1)
	{
		if(bitstream & bit)
			register_bits |= SERIAL_DATA_IN_BIT;
		else
			register_bits &= ~SERIAL_DATA_IN_BIT;
		udelay(ad8402_udelay);
		writew(register_bits, priv(dev)->main_iobase + CALIBRATION_REG);
		udelay(ad8402_udelay);
		writew(register_bits | SERIAL_CLOCK_BIT, priv(dev)->main_iobase + CALIBRATION_REG);
        }

	udelay(ad8402_udelay);
	writew(0, priv(dev)->main_iobase + CALIBRATION_REG);

	return 1;
}

static int ad8402_read_insn(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	unsigned int channel = CR_CHAN(insn->chanspec);

	data[0] = priv(dev)->ad8402_state[channel];

        return 1;
}

static uint16_t read_eeprom(comedi_device *dev, uint8_t address)
{
	static const int bitstream_length = 11;
	static const int read_command = 0x6;
	unsigned int bitstream = (read_command << 8) | address;
	unsigned int bit;
	const int plx_control_addr = priv(dev)->plx9080_iobase + PLX_CONTROL_REG;
	uint16_t value;
	static const int value_length = 16;
	static const int eeprom_udelay = 1;

	udelay(eeprom_udelay);
	priv(dev)->plx_control_bits &= ~CTL_EE_CLK & ~CTL_EE_CS;
	// make sure we don't send anything to the i2c bus on 4020
	priv(dev)->plx_control_bits |= CTL_USERO;
	writel(priv(dev)->plx_control_bits, plx_control_addr);
	// activate serial eeprom
	udelay(eeprom_udelay);
	priv(dev)->plx_control_bits |= CTL_EE_CS;
	writel(priv(dev)->plx_control_bits, plx_control_addr);

	// write read command and desired memory address
	for(bit = 1 << (bitstream_length - 1); bit; bit >>= 1)
	{
		// set bit to be written
		udelay(eeprom_udelay);
		if(bitstream & bit)
			priv(dev)->plx_control_bits |= CTL_EE_W;
		else
			priv(dev)->plx_control_bits &= ~CTL_EE_W;
		writel(priv(dev)->plx_control_bits, plx_control_addr);
		// clock in bit
		udelay(eeprom_udelay);
		priv(dev)->plx_control_bits |= CTL_EE_CLK;
		writel(priv(dev)->plx_control_bits, plx_control_addr);
		udelay(eeprom_udelay);
		priv(dev)->plx_control_bits &= ~CTL_EE_CLK;
		writel(priv(dev)->plx_control_bits, plx_control_addr);
	}
	// read back value from eeprom memory location
	value = 0;
	for(bit = 1 << (value_length - 1); bit; bit >>= 1)
	{
		// clock out bit
		udelay(eeprom_udelay);
		priv(dev)->plx_control_bits |= CTL_EE_CLK;
		writel(priv(dev)->plx_control_bits, plx_control_addr);
		udelay(eeprom_udelay);
		priv(dev)->plx_control_bits &= ~CTL_EE_CLK;
		writel(priv(dev)->plx_control_bits, plx_control_addr);
		udelay(eeprom_udelay);
		if(readl(plx_control_addr) & CTL_EE_R)
			value |= bit;
	}

	// deactivate eeprom serial input
	udelay(eeprom_udelay);
	priv(dev)->plx_control_bits &= ~CTL_EE_CS;
	writel(priv(dev)->plx_control_bits, plx_control_addr);

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

// adjusts the size of hardware fifo (which determines block size for dma xfers)
static int set_ai_fifo_size( comedi_device *dev, unsigned int num_samples )
{
	unsigned int num_fifo_entries;
	int retval;
	const hw_fifo_info_t *const fifo = board(dev)->ai_fifo;

	num_fifo_entries = num_samples / fifo->sample_packing_ratio;

	retval = set_ai_fifo_segment_length( dev, num_fifo_entries / fifo->num_segments );
	if( retval < 0 ) return retval;

	num_samples = retval * fifo->num_segments * fifo->sample_packing_ratio;

	DEBUG_PRINT( "set hardware fifo size to %i\n", num_samples );

	return num_samples;
}

// query length of fifo
static unsigned int ai_fifo_size( comedi_device *dev )
{
	return priv(dev)->ai_fifo_segment_length * board(dev)->ai_fifo->num_segments *
		board(dev)->ai_fifo->sample_packing_ratio;
}

static int set_ai_fifo_segment_length( comedi_device *dev, unsigned int num_entries )
{
	static const int increment_size = 0x100;
	const hw_fifo_info_t *const fifo = board(dev)->ai_fifo;
	unsigned int num_increments;
	uint16_t bits;

	if( num_entries < increment_size ) num_entries = increment_size;
	if( num_entries > fifo->max_segment_length ) num_entries = fifo->max_segment_length;

	// 0 -- 256 entries, 1 == 512 entries, etc
	num_increments = ( num_entries - increment_size / 2 ) / increment_size;

	bits = (~num_entries) & fifo->fifo_size_reg_mask;
	priv(dev)->fifo_size_bits &= ~fifo->fifo_size_reg_mask;
	priv(dev)->fifo_size_bits |= bits;
	writew( priv(dev)->fifo_size_bits, priv(dev)->main_iobase + FIFO_SIZE_REG );

	priv(dev)->ai_fifo_segment_length = num_increments * increment_size;

	DEBUG_PRINT( "set hardware fifo segment length to %i\n", priv(dev)->ai_fifo_segment_length );

	return priv(dev)->ai_fifo_segment_length;
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
/* pci-6402/16 uses all 8 channels for dac:
 * address 0 == dac channel 0 fine gain
 * address 1 == dac channel 0 coarse gain
 * address 2 == dac channel 0 coarse offset
 * address 3 == dac channel 1 coarse offset
 * address 4 == dac channel 1 fine gain
 * address 5 == dac channel 1 coarse gain
 * address 6 == dac channel 0 fine offset
 * address 7 == dac channel 1 fine offset
*/

static int caldac_8800_write(comedi_device *dev, unsigned int address, uint8_t value)
{
	static const int num_caldac_channels = 8;
	static const int bitstream_length = 11;
	unsigned int bitstream = ((address & 0x7) << 8) | value;
	unsigned int bit, register_bits;
	static const int caldac_8800_udelay = 1;

	if(address >= num_caldac_channels)
	{
		comedi_error(dev, "illegal caldac channel");
		return -1;
	}

	for(bit = 1 << (bitstream_length - 1); bit; bit >>= 1)
	{
		register_bits = 0;
		if(bitstream & bit)
			register_bits |= SERIAL_DATA_IN_BIT;
		udelay(caldac_8800_udelay);
		writew(register_bits, priv(dev)->main_iobase + CALIBRATION_REG);
		register_bits |= SERIAL_CLOCK_BIT;
		udelay(caldac_8800_udelay);
		writew(register_bits, priv(dev)->main_iobase + CALIBRATION_REG);
        }

	udelay(caldac_8800_udelay);
	writew(SELECT_8800_BIT, priv(dev)->main_iobase + CALIBRATION_REG);
	udelay(caldac_8800_udelay);
	writew(0, priv(dev)->main_iobase + CALIBRATION_REG);

	return 0;
}

// 4020 caldacs
static int caldac_i2c_write(comedi_device *dev, unsigned int caldac_channel, unsigned int value)
{
	uint8_t serial_bytes[3];
	uint8_t i2c_addr;
	enum pointer_bits
	{
		// manual has gain and offset bits switched
		OFFSET_0_2 = 0x1,
		GAIN_0_2 = 0x2,
		OFFSET_1_3 = 0x4,
		GAIN_1_3 = 0x8,
	};
	enum data_bits
	{
		NOT_CLEAR_REGISTERS = 0x20,
	};

	switch(caldac_channel)
	{
		case 0:	// chan 0 offset
			i2c_addr = CALDAC0_I2C_ADDR;
			serial_bytes[0] = OFFSET_0_2;
			break;
		case 1:	// chan 1 offset
			i2c_addr = CALDAC0_I2C_ADDR;
			serial_bytes[0] = OFFSET_1_3;
			break;
		case 2:	// chan 2 offset
			i2c_addr = CALDAC1_I2C_ADDR;
			serial_bytes[0] = OFFSET_0_2;
			break;
		case 3:	// chan 3 offset
			i2c_addr = CALDAC1_I2C_ADDR;
			serial_bytes[0] = OFFSET_1_3;
			break;
		case 4:	// chan 0 gain
			i2c_addr = CALDAC0_I2C_ADDR;
			serial_bytes[0] = GAIN_0_2;
			break;
		case 5:	// chan 1 gain
			i2c_addr = CALDAC0_I2C_ADDR;
			serial_bytes[0] = GAIN_1_3;
			break;
		case 6:	// chan 2 gain
			i2c_addr = CALDAC1_I2C_ADDR;
			serial_bytes[0] = GAIN_0_2;
			break;
		case 7:	// chan 3 gain
			i2c_addr = CALDAC1_I2C_ADDR;
			serial_bytes[0] = GAIN_1_3;
			break;
		default:
			comedi_error(dev, "invalid caldac channel\n");
			return -1;
			break;
	}
	serial_bytes[1] = NOT_CLEAR_REGISTERS | ((value >> 8) & 0xf);
	serial_bytes[2] = value & 0xff;
	i2c_write(dev, i2c_addr, serial_bytes, 3);
	return 0;
}

// Their i2c requires a huge delay on setting clock or data high for some reason
static const int i2c_high_udelay = 1000;
static const int i2c_low_udelay = 10;

// set i2c data line high or low
static void i2c_set_sda(comedi_device *dev, int state)
{
	static const int data_bit = CTL_EE_W;
	unsigned long plx_control_addr = priv(dev)->plx9080_iobase + PLX_CONTROL_REG;

	if(state)
	{
		// set data line high
		priv(dev)->plx_control_bits &= ~data_bit;
		writel(priv(dev)->plx_control_bits, plx_control_addr);
		udelay(i2c_high_udelay);
	}else // set data line low
	{
		priv(dev)->plx_control_bits |= data_bit;
		writel(priv(dev)->plx_control_bits, plx_control_addr);
		udelay(i2c_low_udelay);
	}
}

// set i2c clock line high or low
static void i2c_set_scl(comedi_device *dev, int state)
{
	static const int clock_bit = CTL_USERO;
	unsigned long plx_control_addr = priv(dev)->plx9080_iobase + PLX_CONTROL_REG;

	if(state)
	{
		// set clock line high
		priv(dev)->plx_control_bits &= ~clock_bit;
		writel(priv(dev)->plx_control_bits, plx_control_addr);
		udelay(i2c_high_udelay);
	}else // set clock line low
	{
		priv(dev)->plx_control_bits |= clock_bit;
		writel(priv(dev)->plx_control_bits, plx_control_addr);
		udelay(i2c_low_udelay);
	}
}

static void i2c_write_byte(comedi_device *dev, uint8_t byte)
{
	uint8_t bit;
	unsigned int num_bits = 8;

	DEBUG_PRINT("writing to i2c byte 0x%x\n", byte);

	for(bit = 1 << (num_bits - 1); bit; bit >>= 1)
	{
		i2c_set_scl(dev, 0);
		if((byte & bit))
			i2c_set_sda(dev, 1);
		else
			i2c_set_sda(dev, 0);
		i2c_set_scl(dev, 1);
	}
}

// we can't really read the lines, so fake it
static int i2c_read_ack(comedi_device *dev)
{
	i2c_set_scl(dev, 0);
	i2c_set_sda(dev, 1);
	i2c_set_scl(dev, 1);

	return 0;	// return fake acknowledge bit
}

// send start bit
static void i2c_start(comedi_device *dev)
{
	i2c_set_scl(dev, 1);
	i2c_set_sda(dev, 1);
	i2c_set_sda(dev, 0);
}

// send stop bit
static void i2c_stop(comedi_device *dev)
{
	i2c_set_scl(dev, 0);
	i2c_set_sda(dev, 0);
	i2c_set_scl(dev, 1);
	i2c_set_sda(dev, 1);
}

static void i2c_write(comedi_device *dev, unsigned int address, const uint8_t *data, unsigned int length)
{
	unsigned int i;
	uint8_t bitstream;
	static const int read_bit = 0x1;

//XXX need mutex to prevent simultaneous attempts to access eeprom and i2c bus

	// make sure we dont send anything to eeprom
	priv(dev)->plx_control_bits &= ~CTL_EE_CS;

	i2c_stop(dev);
	i2c_start(dev);

	// send address and write bit
	bitstream = (address << 1) & ~read_bit;
	i2c_write_byte(dev, bitstream);

	// get acknowledge
	if(i2c_read_ack(dev) != 0)
	{
		comedi_error(dev, "i2c write failed: no acknowledge");
		i2c_stop(dev);
		return;
	}

	// write data bytes
	for(i = 0; i < length; i++)
	{
		i2c_write_byte(dev, data[i]);
		if(i2c_read_ack(dev) != 0)
		{
			comedi_error(dev, "i2c write failed: no acknowledge");
			i2c_stop(dev);
			return;
		}
	}
	i2c_stop(dev);
}

