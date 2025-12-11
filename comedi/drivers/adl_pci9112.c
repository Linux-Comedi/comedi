/*

   comedi/drivers/adl_pci9112.c

   Hardware driver for PCI9112 ADLINK cards:

     PCI-9112

   Copyright (C) 2002-2005 Emmanuel Pacaud <emmanuel.pacaud@univ-poitiers.fr>

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
Driver: adl_pci9112
Description: ADLINK PCI-9112
Author: Pascal Berthou <berthou@laas.fr> Emmanuel Pacaud <emmanuel.pacaud@univ-poitiers.fr>
Devices: [ADLINK] PCI-9112 (adl_pci9112)
Status: experimental

Supports:

  - ai_insn read
  - ao_insn read/write
  - di_insn read
  - do_insn read/write

  Following command mode are not tested
  - ai_do_cmd mode with the following sources:

    - start_src 		TRIG_NOW
    - scan_begin_src 		TRIG_FOLLOW	TRIG_TIMER	TRIG_EXT
    - convert_src				TRIG_TIMER	TRIG_EXT
    - scan_end_src		TRIG_COUNT
    - stop_src			TRIG_COUNT	TRIG_NONE

    The scanned channels must be consecutive and start from 0. They must
    all have the same range and aref.

Configuration options:

    [0] - PCI bus number (optional)
    [1] - PCI slot number (optional)

    If bus/slot is not specified, the first available PCI
    device will be used.

*/

/*
CHANGELOG:

  2014/09/9  Port to PCI 9112

TODO:

  - Really test implemented functionality and command mode.
  - Add support for the PCI-9112 with a probe routine to identify the card type
    (perhaps with the help of the channel number readback of the A/D Data register).
  - Add external multiplexer support.

*/

#include <linux/comedidev.h>

#include <linux/delay.h>

#include "8253.h"
#include "comedi_pci.h"

#define PCI_VENDOR_ID_ADLINK_9112 0x10e8 
#define PCI9112_DEVICE_ID       0x80d7

#define PCI9112_DRIVER_NAME 	"adl_pci9112"

// Ajout pci9112_ad_da
typedef struct {
	int data;
	struct pci_dev *pci_dev;	/* for a PCI device */
	lsampl_t ao_readback[2];	/* Used for AO readback */
} pci9112_private;

#define devpriv ((pci9112_private *)dev->private)
// fin ajout pci9112_ad_da

// TODO: Add other pci9112 board id

#define PCI9112_IO_RANGE 	0x0100

#define PCI9112_FIFO_HALF_SIZE	512

#define PCI9112_AI_CHANNEL_NBR			16

#define PCI9112_AI_RESOLUTION			12
#define PCI9112_AI_RESOLUTION_MASK		0x0FFF
#define PCI9112_AI_RESOLUTION_2_CMP_BIT		0x0800

#define PCI9112_HR_AI_RESOLUTION		16
#define PCI9112_HR_AI_RESOLUTION_MASK		0xFFFF
#define PCI9112_HR_AI_RESOLUTION_2_CMP_BIT	0x8000

#define PCI9112_AI_ACQUISITION_PERIOD_MIN_NS	10000
#define PCI9112_AO_CHANNEL_NBR			2
#define	PCI9112_AO_RESOLUTION			12
#define PCI9112_AO_RESOLUTION_MASK		0x0FFF
#define PCI9112_DI_CHANNEL_NBR			16
#define	PCI9112_DO_CHANNEL_NBR			16
#define PCI9112_DO_MASK				0xFFFF

#define PCI9112_RANGE_SETTING_DELAY		10
#define PCI9112_AI_INSTANT_READ_UDELAY_US	2
#define PCI9112_AI_INSTANT_READ_TIMEOUT		100

#define PCI9112_8254_CLOCK_PERIOD_NS		500

#define PCI9112_8254_COUNTER_0			0x00
#define PCI9112_8254_COUNTER_1			0x40
#define PCI9112_8254_COUNTER_2			0x80
#define PCI9112_8254_COUNTER_LATCH		0x00
#define PCI9112_8254_READ_LOAD_LSB_ONLY		0x10
#define PCI9112_8254_READ_LOAD_MSB_ONLY		0x20
#define PCI9112_8254_READ_LOAD_LSB_MSB		0x30
#define PCI9112_8254_MODE_0			0x00
#define PCI9112_8254_MODE_1			0x02
#define PCI9112_8254_MODE_2			0x04
#define PCI9112_8254_MODE_3			0x06
#define PCI9112_8254_MODE_4			0x08
#define PCI9112_8254_MODE_5			0x0A
#define PCI9112_8254_BINARY_COUNTER		0x00
#define PCI9112_8254_BCD_COUNTER		0x01

/* IO address map */

#define PCI9112_REGISTER_AD_FIFO_VALUE 			0x18	// 10 ? 
#define PCI9112_REGISTER_DA_OUTPUT 			0x10	// 
#define PCI9112_REGISTER_DIGITAL_IO 			0x1C    //
#define PCI9112_REGISTER_EXTENDED_IO_PORTS 		0x04    // --
#define PCI9112_REGISTER_AD_CHANNEL_CONTROL 		0x18	// 
#define PCI9112_REGISTER_AD_CHANNEL_READBACK 		0x18    // 
#define PCI9112_REGISTER_INPUT_SIGNAL_RANGE 		0x08	// --
#define PCI9112_REGISTER_RANGE_STATUS_READBACK 		0x08    // --
#define PCI9112_REGISTER_TRIGGER_MODE_CONTROL 		0x0A	// --
#define PCI9112_REGISTER_AD_MODE_INTERRUPT_READBACK 	0x0A	// --
#define PCI9112_REGISTER_SOFTWARE_TRIGGER 		0x20	//
#define PCI9112_REGISTER_INTERRUPT_CONTROL 		0x0C	// --
#define PCI9112_REGISTER_8254_COUNTER_0			0x00	//
#define PCI9112_REGISTER_8254_COUNTER_1			0x04	//
#define PCI9112_REGISTER_8254_COUNTER_2 		0X08	//
#define PCI9112_REGISTER_8254_CONTROL			0x0C	//
#define PCI9112_REGISTER_INTERRUPT_CLEAR 		0x48	// --


#define PCI9112_TRIGGER_MASK 				0x0F
#define PCI9112_PTRG_OFF 				(0 << 3)
#define PCI9112_PTRG_ON 				(1 << 3)
#define PCI9112_EITS_EXTERNAL				(1 << 2)
#define PCI9112_EITS_INTERNAL				(0 << 2)
#define PCI9112_TPST_SOFTWARE_TRIGGER			(0 << 1)
#define PCI9112_TPST_TIMER_PACER			(1 << 1)
#define PCI9112_ASCAN_ON				(1 << 0)
#define PCI9112_ASCAN_OFF				(0 << 0)

#define PCI9112_ISC0_SET_IRQ_ON_ENDING_OF_AD_CONVERSION (0 << 0)
#define PCI9112_ISC0_SET_IRQ_ON_FIFO_HALF_FULL		(1 << 0)
#define PCI9112_ISC1_SET_IRQ_ON_TIMER_TICK  		(0 << 1)
#define PCI9112_ISC1_SET_IRQ_ON_EXT_TRG 		(1 << 1)
#define PCI9112_FFEN_SET_FIFO_ENABLE 			(0 << 2)
#define PCI9112_FFEN_SET_FIFO_DISABLE			(1 << 2)

#define PCI9112_CHANNEL_MASK				0x0F  

#define PCI9112_RANGE_MASK				0x07
#define PCI9112_FIFO_EMPTY_MASK				0x10
#define PCI9112_FIFO_HALF_FULL_MASK			0x20
#define PCI9112_FIFO_FULL_MASK				0x40
#define PCI9112_AD_BUSY_MASK				0x80

#define PCI9112_IO_BASE dev->iobase

/*
 * Define inlined function
 */

#define pci9112_trigger_and_autoscan_get() \
  (inb(PCI9112_IO_BASE+PCI9112_REGISTER_AD_MODE_INTERRUPT_READBACK)&0x0F)

#define pci9112_trigger_and_autoscan_set(flags) \
  outb(flags,PCI9112_IO_BASE+PCI9112_REGISTER_TRIGGER_MODE_CONTROL)

#define pci9112_interrupt_and_fifo_get() \
  ((inb(PCI9112_IO_BASE+PCI9112_REGISTER_AD_MODE_INTERRUPT_READBACK) >> 4) &0x03)

#define pci9112_interrupt_and_fifo_set(flags) \
  outb(flags,PCI9112_IO_BASE+PCI9112_REGISTER_INTERRUPT_CONTROL)

#define pci9112_interrupt_clear() \
  outb(0,PCI9112_IO_BASE+PCI9112_REGISTER_INTERRUPT_CLEAR)

#define pci9112_software_trigger() \
  outb(0,PCI9112_IO_BASE+PCI9112_REGISTER_SOFTWARE_TRIGGER)

#define pci9112_fifo_reset() \
  outb(PCI9112_FFEN_SET_FIFO_ENABLE,PCI9112_IO_BASE+PCI9112_REGISTER_INTERRUPT_CONTROL); \
  outb(PCI9112_FFEN_SET_FIFO_DISABLE,PCI9112_IO_BASE+PCI9112_REGISTER_INTERRUPT_CONTROL); \
  outb(PCI9112_FFEN_SET_FIFO_ENABLE,PCI9112_IO_BASE+PCI9112_REGISTER_INTERRUPT_CONTROL)

#define pci9112_is_fifo_full() \
  ((inb(PCI9112_IO_BASE+PCI9112_REGISTER_RANGE_STATUS_READBACK)& \
    PCI9112_FIFO_FULL_MASK)==0)

#define pci9112_is_fifo_half_full() \
  ((inb(PCI9112_IO_BASE+PCI9112_REGISTER_RANGE_STATUS_READBACK)& \
    PCI9112_FIFO_HALF_FULL_MASK)==0)

#define pci9112_is_fifo_empty() \
  ((inb(PCI9112_IO_BASE+PCI9112_REGISTER_RANGE_STATUS_READBACK)& \
    PCI9112_FIFO_EMPTY_MASK)==0)

#define pci9112_ai_channel_set(channel) \
  outb((channel)&PCI9112_CHANNEL_MASK,PCI9112_IO_BASE+PCI9112_REGISTER_AD_CHANNEL_CONTROL)

#define pci9112_ai_channel_get() \
  inb(PCI9112_IO_BASE+PCI9112_REGISTER_AD_CHANNEL_READBACK)&PCI9112_CHANNEL_MASK

#define pci9112_ai_range_set(range) \
  outb((range)&PCI9112_RANGE_MASK,PCI9112_IO_BASE+PCI9112_REGISTER_INPUT_SIGNAL_RANGE)

#define pci9112_ai_range_get() \
  inb(PCI9112_IO_BASE+PCI9112_REGISTER_RANGE_STATUS_READBACK)&PCI9112_RANGE_MASK

/*
#define pci9112_ai_get_data() \
  ((inw(PCI9112_IO_BASE+PCI9112_REGISTER_AD_FIFO_VALUE)>>4)&PCI9112_AI_RESOLUTION_MASK) \
  ^ PCI9112_AI_RESOLUTION_2_CMP_BIT
*/
#define pci9112_hr_ai_get_data() \
  (inw(PCI9112_IO_BASE+PCI9112_REGISTER_AD_FIFO_VALUE) & PCI9112_HR_AI_RESOLUTION_MASK) \
  ^ PCI9112_HR_AI_RESOLUTION_2_CMP_BIT

#define pci9112_ao_set_data(data) \
  outw(data&PCI9112_AO_RESOLUTION_MASK,PCI9112_IO_BASE+PCI9112_REGISTER_DA_OUTPUT)

#define pci9112_di_get_bits() \
  inw(PCI9112_IO_BASE+PCI9112_REGISTER_DIGITAL_IO)

#define pci9112_do_set_bits(bits) \
  outw(bits,PCI9112_IO_BASE+PCI9112_REGISTER_DIGITAL_IO)

#define pci9112_8254_control_set(flags) \
  outb(flags,PCI9112_IO_BASE+PCI9112_REGISTER_8254_CONTROL)

#define pci9112_8254_counter_0_set(data) \
  outb(data & 0xFF, PCI9112_IO_BASE+PCI9112_REGISTER_8254_COUNTER_0); \
  outb( (data >> 8) & 0xFF, PCI9112_IO_BASE+PCI9112_REGISTER_8254_COUNTER_0)

#define pci9112_8254_counter_1_set(data) \
  outb(data & 0xFF, PCI9112_IO_BASE+PCI9112_REGISTER_8254_COUNTER_1); \
  outb( (data >> 8) & 0xFF, PCI9112_IO_BASE+PCI9112_REGISTER_8254_COUNTER_1)

#define pci9112_8254_counter_2_set(data) \
  outb(data & 0xFF, PCI9112_IO_BASE+PCI9112_REGISTER_8254_COUNTER_2); \
  outb( (data >> 8) & 0xFF, PCI9112_IO_BASE+PCI9112_REGISTER_8254_COUNTER_2)

// Ajout ADLINK_PCI9112_AD_DA
#define pci9112_ai_get_data() \
  ((inw(dev->iobase+0x10))>>4)&0x0fff;

#define pci9112_ai_set_channel(channel) \
    outw((channel &0x0f)<<5,dev->iobase+0x18)

#define pci9112_ai_status() \
  (inw(dev->iobase+0x18))
// Fin Ajout ADLINK_PCI9112_AD_DA

//
// Function prototypes
//

static int pci9112_attach(comedi_device * dev, comedi_devconfig * it);
static int pci9112_detach(comedi_device * dev);
static void pci9112_ai_munge(comedi_device * dev, comedi_subdevice * s,
	void *data, unsigned int num_bytes, unsigned int start_chan_index);

static const comedi_lrange pci9112_hr_ai_range = {
	5,
	{
			BIP_RANGE(10),
			BIP_RANGE(5),
			BIP_RANGE(2.5),
			BIP_RANGE(1.25),
			BIP_RANGE(0.625)
		}
};

static DEFINE_PCI_DEVICE_TABLE(pci9112_pci_table) = {
	{PCI_VENDOR_ID_ADLINK_9112, PCI9112_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0,
		0, 0},
	//{ PCI_VENDOR_ID_ADLINK, PCI9112_HG_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{0}
};

MODULE_DEVICE_TABLE(pci, pci9112_pci_table);

//
// Board specification structure
//

typedef struct {
	const char *name;	// driver name
	int device_id;
	int ai_channel_nbr;	// num of A/D chans
	int ao_channel_nbr;	// num of D/A chans
	int ai_resolution;	// resolution of A/D
	int ai_resolution_mask;
	int ao_resolution;	// resolution of D/A
	int ao_resolution_mask;
	const comedi_lrange *ai_range_list;	// rangelist for A/D
	const comedi_lrange *ao_range_list;	// rangelist for D/A
	unsigned int ai_acquisition_period_min_ns;
} pci9112_board_struct;

static const pci9112_board_struct pci9112_boards[] = {
	{
	      name:	"pci9112",
	      device_id:PCI9112_DEVICE_ID,
	      ai_channel_nbr:PCI9112_AI_CHANNEL_NBR,
	      ao_channel_nbr:PCI9112_AO_CHANNEL_NBR,
	      ai_resolution:PCI9112_AI_RESOLUTION,
	      ai_resolution_mask:PCI9112_AI_RESOLUTION_MASK,
	      ao_resolution:PCI9112_AO_RESOLUTION,
	      ao_resolution_mask:PCI9112_AO_RESOLUTION_MASK,
	      ai_range_list:&pci9112_hr_ai_range, //pci9112_hr_ai_range
	      ao_range_list:&range_bipolar10,
      ai_acquisition_period_min_ns:PCI9112_AI_ACQUISITION_PERIOD_MIN_NS}
};

#define pci9112_board_nbr \
  (sizeof(pci9112_boards)/sizeof(pci9112_board_struct))

static comedi_driver pci9112_driver = {
      driver_name:PCI9112_DRIVER_NAME,
      module:THIS_MODULE,
      attach:pci9112_attach,
      detach:pci9112_detach,
};

COMEDI_PCI_INITCLEANUP(pci9112_driver, pci9112_pci_table);

//
// Private data structure
//

typedef struct {
	struct pci_dev *pci_device;
	unsigned long io_range;	// PCI6503 io range

	unsigned long lcr_io_base;	// Local configuration register base address
	unsigned long lcr_io_range;

	int stop_counter;
	int stop_is_none;

	unsigned int scan_delay;
	unsigned int chanlist_len;
	unsigned int chunk_counter;
	unsigned int chunk_num_samples;

	int ao_readback;	// Last written analog output data

	int timer_divisor_1;	// Divisor values for the 8254 timer pacer
	int timer_divisor_2;

	int is_valid;		// Is device valid

	sampl_t ai_bounce_buffer[2 * PCI9112_FIFO_HALF_SIZE];
} pci9112_private_data_struct;

#define dev_private 	((pci9112_private_data_struct *)dev->private)

// ------------------------------------------------------------------
//
// PLX9050 SECTION
//
// ------------------------------------------------------------------

#define PLX9050_REGISTER_INTERRUPT_CONTROL 0x4c

#define PLX9050_LINTI1_ENABLE		(1 << 0)
#define PLX9050_LINTI1_ACTIVE_HIGH	(1 << 1)
#define PLX9050_LINTI1_STATUS		(1 << 2)
#define PLX9050_LINTI2_ENABLE		(1 << 3)
#define PLX9050_LINTI2_ACTIVE_HIGH	(1 << 4)
#define PLX9050_LINTI2_STATUS		(1 << 5)
#define PLX9050_PCI_INTERRUPT_ENABLE	(1 << 6)
#define PLX9050_SOFTWARE_INTERRUPT	(1 << 7)

static void plx9050_interrupt_control(unsigned long io_base,
	bool LINTi1_enable,
	bool LINTi1_active_high,
	bool LINTi2_enable, bool LINTi2_active_high, bool interrupt_enable)
{
	int flags = 0;

	if (LINTi1_enable)
		flags |= PLX9050_LINTI1_ENABLE;
	if (LINTi1_active_high)
		flags |= PLX9050_LINTI1_ACTIVE_HIGH;
	if (LINTi2_enable)
		flags |= PLX9050_LINTI2_ENABLE;
	if (LINTi2_active_high)
		flags |= PLX9050_LINTI2_ACTIVE_HIGH;

	if (interrupt_enable)
		flags |= PLX9050_PCI_INTERRUPT_ENABLE;

	outb(flags, io_base + PLX9050_REGISTER_INTERRUPT_CONTROL);
}

// ------------------------------------------------------------------
//
// MISCELLANEOUS SECTION
//
// ------------------------------------------------------------------

//
// 8254 timer
//

static void pci9112_timer_set(comedi_device * dev)
{
	pci9112_8254_control_set(PCI9112_8254_COUNTER_0 |
		PCI9112_8254_READ_LOAD_LSB_MSB |
		PCI9112_8254_MODE_0 | PCI9112_8254_BINARY_COUNTER);

	pci9112_8254_control_set(PCI9112_8254_COUNTER_1 |
		PCI9112_8254_READ_LOAD_LSB_MSB |
		PCI9112_8254_MODE_2 | PCI9112_8254_BINARY_COUNTER);

	pci9112_8254_control_set(PCI9112_8254_COUNTER_2 |
		PCI9112_8254_READ_LOAD_LSB_MSB |
		PCI9112_8254_MODE_2 | PCI9112_8254_BINARY_COUNTER);

	comedi_udelay(1);

	pci9112_8254_counter_2_set(dev_private->timer_divisor_2);
	pci9112_8254_counter_1_set(dev_private->timer_divisor_1);
}

typedef enum {
	software,
	timer_pacer,
	external
} pci9112_trigger_sources;

static void pci9112_trigger_source_set(comedi_device * dev,
	pci9112_trigger_sources source)
{
	int flags;

	flags = pci9112_trigger_and_autoscan_get() & 0x09;

	switch (source) {
	case software:
		flags |= PCI9112_EITS_INTERNAL | PCI9112_TPST_SOFTWARE_TRIGGER;
		break;

	case timer_pacer:
		flags |= PCI9112_EITS_INTERNAL | PCI9112_TPST_TIMER_PACER;
		break;

	case external:
		flags |= PCI9112_EITS_EXTERNAL;
		break;
	}

	pci9112_trigger_and_autoscan_set(flags);
}

static void pci9112_pretrigger_set(comedi_device * dev, bool pretrigger)
{
	int flags;

	flags = pci9112_trigger_and_autoscan_get() & 0x07;

	if (pretrigger)
		flags |= PCI9112_PTRG_ON;

	pci9112_trigger_and_autoscan_set(flags);
}

static void pci9112_autoscan_set(comedi_device * dev, bool autoscan)
{
	int flags;

	flags = pci9112_trigger_and_autoscan_get() & 0x0e;

	if (autoscan)
		flags |= PCI9112_ASCAN_ON;

	pci9112_trigger_and_autoscan_set(flags);
}

typedef enum {
	irq_on_eoc,
	irq_on_fifo_half_full
} pci9112_ISC0_sources;

typedef enum {
	irq_on_timer_tick,
	irq_on_external_trigger
} pci9112_ISC1_sources;

static void pci9112_interrupt_source_set(comedi_device * dev,
	pci9112_ISC0_sources irq_0_source, pci9112_ISC1_sources irq_1_source)
{
	int flags;

	flags = pci9112_interrupt_and_fifo_get() & 0x04;

	if (irq_0_source == irq_on_fifo_half_full)
		flags |= PCI9112_ISC0_SET_IRQ_ON_FIFO_HALF_FULL;

	if (irq_1_source == irq_on_external_trigger)
		flags |= PCI9112_ISC1_SET_IRQ_ON_EXT_TRG;

	pci9112_interrupt_and_fifo_set(flags);
}

// ------------------------------------------------------------------
//
// HARDWARE TRIGGERED ANALOG INPUT SECTION
//
// ------------------------------------------------------------------

//
// Cancel analog input autoscan
//

#undef AI_DO_CMD_DEBUG

static int pci9112_ai_cancel(comedi_device * dev, comedi_subdevice * s)
{
	// Disable interrupts

	plx9050_interrupt_control(dev_private->lcr_io_base, true, true, true,
		true, false);

	pci9112_trigger_source_set(dev, software);

	pci9112_autoscan_set(dev, false);

	pci9112_fifo_reset();

#ifdef AI_DO_CMD_DEBUG
	printk(PCI9112_DRIVER_NAME ": ai_cancel\n");
#endif

	return 0;
}

//
// Test analog input command
//

#define pci9112_check_trigger_src(src,flags) \
  tmp = src; \
  src &= flags; \
  if (!src || tmp != src) error++

static int
pci9112_ai_do_cmd_test(comedi_device * dev,
	comedi_subdevice * s, comedi_cmd * cmd)
{
	int tmp;
	int error = 0;
	int range, reference;
	int i;
	pci9112_board_struct *board = (pci9112_board_struct *) dev->board_ptr;

	// Step 1 : check if trigger are trivialy valid

	pci9112_check_trigger_src(cmd->start_src, TRIG_NOW);
	pci9112_check_trigger_src(cmd->scan_begin_src,
		TRIG_TIMER | TRIG_FOLLOW | TRIG_EXT);
	pci9112_check_trigger_src(cmd->convert_src, TRIG_TIMER | TRIG_EXT);
	pci9112_check_trigger_src(cmd->scan_end_src, TRIG_COUNT);
	pci9112_check_trigger_src(cmd->stop_src, TRIG_COUNT | TRIG_NONE);

	if (error)
		return 1;

	// step 2 : make sure trigger sources are unique and mutually compatible

	if (cmd->start_src != TRIG_NOW)
		error++;

	if ((cmd->scan_begin_src != TRIG_TIMER) &&
		(cmd->scan_begin_src != TRIG_FOLLOW) &&
		(cmd->scan_begin_src != TRIG_EXT))
		error++;

	if ((cmd->convert_src != TRIG_TIMER) && (cmd->convert_src != TRIG_EXT)) {
		error++;
	}
	if ((cmd->convert_src == TRIG_TIMER) &&
		!((cmd->scan_begin_src == TRIG_TIMER) ||
			(cmd->scan_begin_src == TRIG_FOLLOW))) {
		error++;
	}
	if ((cmd->convert_src == TRIG_EXT) &&
		!((cmd->scan_begin_src == TRIG_EXT) ||
			(cmd->scan_begin_src == TRIG_FOLLOW))) {
		error++;
	}

	if (cmd->scan_end_src != TRIG_COUNT)
		error++;
	if ((cmd->stop_src != TRIG_COUNT) && (cmd->stop_src != TRIG_NONE))
		error++;

	if (error)
		return 2;

	// Step 3 : make sure arguments are trivialy compatible

	if (cmd->chanlist_len < 1) {
		cmd->chanlist_len = 1;
		error++;
	}

	if (cmd->chanlist_len > board->ai_channel_nbr) {
		cmd->chanlist_len = board->ai_channel_nbr;
		error++;
	}

	if ((cmd->start_src == TRIG_NOW) && (cmd->start_arg != 0)) {
		cmd->start_arg = 0;
		error++;
	}

	if ((cmd->convert_src == TRIG_TIMER) &&
		(cmd->convert_arg < board->ai_acquisition_period_min_ns)) {
		cmd->convert_arg = board->ai_acquisition_period_min_ns;
		error++;
	}
	if ((cmd->convert_src == TRIG_EXT) && (cmd->convert_arg != 0)) {
		cmd->convert_arg = 0;
		error++;
	}

	if ((cmd->scan_begin_src == TRIG_TIMER) &&
		(cmd->scan_begin_arg < board->ai_acquisition_period_min_ns)) {
		cmd->scan_begin_arg = board->ai_acquisition_period_min_ns;
		error++;
	}
	if ((cmd->scan_begin_src == TRIG_FOLLOW) && (cmd->scan_begin_arg != 0)) {
		cmd->scan_begin_arg = 0;
		error++;
	}
	if ((cmd->scan_begin_src == TRIG_EXT) && (cmd->scan_begin_arg != 0)) {
		cmd->scan_begin_arg = 0;
		error++;
	}

	if ((cmd->scan_end_src == TRIG_COUNT) &&
		(cmd->scan_end_arg != cmd->chanlist_len)) {
		cmd->scan_end_arg = cmd->chanlist_len;
		error++;
	}

	if ((cmd->stop_src == TRIG_COUNT) && (cmd->stop_arg < 1)) {
		cmd->stop_arg = 1;
		error++;
	}
	if ((cmd->stop_src == TRIG_NONE) && (cmd->stop_arg != 0)) {
		cmd->stop_arg = 0;
		error++;
	}

	if (error)
		return 3;

	// Step 4 : fix up any arguments

	if (cmd->convert_src == TRIG_TIMER) {
		tmp = cmd->convert_arg;
		i8253_cascade_ns_to_timer_2div(PCI9112_8254_CLOCK_PERIOD_NS,
			&(dev_private->timer_divisor_1),
			&(dev_private->timer_divisor_2),
			&(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->convert_arg)
			error++;
	}
	// There's only one timer on this card, so the scan_begin timer must
	// be a multiple of chanlist_len*convert_arg

	if (cmd->scan_begin_src == TRIG_TIMER) {

		unsigned int scan_begin_min;
		unsigned int scan_begin_arg;
		unsigned int scan_factor;

		scan_begin_min = cmd->chanlist_len * cmd->convert_arg;

		if (cmd->scan_begin_arg != scan_begin_min) {
			if (scan_begin_min < cmd->scan_begin_arg) {
				scan_factor =
					cmd->scan_begin_arg / scan_begin_min;
				scan_begin_arg = scan_factor * scan_begin_min;
				if (cmd->scan_begin_arg != scan_begin_arg) {
					cmd->scan_begin_arg = scan_begin_arg;
					error++;
				}
			} else {
				cmd->scan_begin_arg = scan_begin_min;
				error++;
			}
		}
	}

	if (error)
		return 4;

	// Step 5 : check channel list

	if (cmd->chanlist) {

		range = CR_RANGE(cmd->chanlist[0]);
		reference = CR_AREF(cmd->chanlist[0]);

		if (cmd->chanlist_len > 1) {
			for (i = 0; i < cmd->chanlist_len; i++) {
				if (CR_CHAN(cmd->chanlist[i]) != i) {
					comedi_error(dev,
						"entries in chanlist must be consecutive "
						"channels,counting upwards from 0\n");
					error++;
				}
				if (CR_RANGE(cmd->chanlist[i]) != range) {
					comedi_error(dev,
						"entries in chanlist must all have the same gain\n");
					error++;
				}
				if (CR_AREF(cmd->chanlist[i]) != reference) {
					comedi_error(dev,
						"entries in chanlist must all have the same reference\n");
					error++;
				}
			}
		} else {
			if ((CR_CHAN(cmd->chanlist[0]) >
					(board->ai_channel_nbr - 1))
				|| (CR_CHAN(cmd->chanlist[0]) < 0)) {
				comedi_error(dev,
					"channel number is out of limits\n");
				error++;
			}
		}
	}

	if (error)
		return 5;

	return 0;

}

//
// Analog input command
//

static int pci9112_ai_do_cmd(comedi_device * dev, comedi_subdevice * subdevice)
{
	comedi_cmd *async_cmd = &subdevice->async->cmd;

	if (!dev->irq) {
		comedi_error(dev,
			"no irq assigned for PCI9112, cannot do hardware conversion");
		return -1;
	}
	// Set channel scan limit
	//
	// PCI9112 allows only scanning from channel 0 to channel n
	//
	// TODO: handle the case of an external multiplexer
	//

	if (async_cmd->chanlist_len > 1) {
		pci9112_ai_channel_set((async_cmd->chanlist_len) - 1);
		pci9112_autoscan_set(dev, true);
	} else {
		pci9112_ai_channel_set(CR_CHAN(async_cmd->chanlist[0]));
		pci9112_autoscan_set(dev, false);
	}

	// Set gain
	//
	// This is the same gain on every channel
	//

	pci9112_ai_range_set(CR_RANGE(async_cmd->chanlist[0]));

	/* Set counter */

	switch (async_cmd->stop_src) {
	case TRIG_COUNT:
		dev_private->stop_counter =
			async_cmd->stop_arg * async_cmd->chanlist_len;
		dev_private->stop_is_none = 0;
		break;

	case TRIG_NONE:
		dev_private->stop_counter = 0;
		dev_private->stop_is_none = 1;
		break;

	default:
		comedi_error(dev, "Invalid stop trigger");
		return -1;
	}

	// Set timer pacer

	dev_private->scan_delay = 0;
	switch (async_cmd->convert_src) {
	case TRIG_TIMER:
		i8253_cascade_ns_to_timer_2div(PCI9112_8254_CLOCK_PERIOD_NS,
			&(dev_private->timer_divisor_1),
			&(dev_private->timer_divisor_2),
			&(async_cmd->convert_arg),
			async_cmd->flags & TRIG_ROUND_MASK);
#ifdef AI_DO_CMD_DEBUG
		printk(PCI9112_DRIVER_NAME ": divisors = %d, %d\n",
			dev_private->timer_divisor_1,
			dev_private->timer_divisor_2);
#endif

		pci9112_trigger_source_set(dev, software);
		pci9112_timer_set(dev);
		pci9112_fifo_reset();
		pci9112_interrupt_source_set(dev, irq_on_fifo_half_full,
			irq_on_timer_tick);
		pci9112_trigger_source_set(dev, timer_pacer);
		plx9050_interrupt_control(dev_private->lcr_io_base, true, true,
			false, true, true);

		if (async_cmd->scan_begin_src == TRIG_TIMER) {
			dev_private->scan_delay =
				(async_cmd->scan_begin_arg /
				 (async_cmd->convert_arg *
				  async_cmd->chanlist_len)) - 1;
		}

		break;

	case TRIG_EXT:

		pci9112_trigger_source_set(dev, external);
		pci9112_fifo_reset();
		pci9112_interrupt_source_set(dev, irq_on_fifo_half_full,
			irq_on_timer_tick);
		plx9050_interrupt_control(dev_private->lcr_io_base, true, true,
			false, true, true);

		break;

	default:
		comedi_error(dev, "Invalid convert trigger");
		return -1;
	}

	dev_private->stop_counter *= (1 + dev_private->scan_delay);
	dev_private->chanlist_len = async_cmd->chanlist_len;
	dev_private->chunk_counter = 0;
	dev_private->chunk_num_samples =
		dev_private->chanlist_len * (1 + dev_private->scan_delay);

#ifdef AI_DO_CMD_DEBUG
	printk(PCI9112_DRIVER_NAME ": start interruptions!\n");
	printk(PCI9112_DRIVER_NAME ": trigger source = %2x\n",
		pci9112_trigger_and_autoscan_get());
	printk(PCI9112_DRIVER_NAME ": irq source     = %2x\n",
		pci9112_interrupt_and_fifo_get());
	printk(PCI9112_DRIVER_NAME ": ai_do_cmd\n");
	printk(PCI9112_DRIVER_NAME ": stop counter   = %d\n",
		dev_private->stop_counter);
	printk(PCI9112_DRIVER_NAME ": scan delay     = %d\n",
		dev_private->scan_delay);
	printk(PCI9112_DRIVER_NAME ": chanlist_len   = %d\n",
		dev_private->chanlist_len);
	printk(PCI9112_DRIVER_NAME ": chunk num samples = %d\n",
		dev_private->chunk_num_samples);
#endif

	return 0;
}

static void pci9112_ai_munge(comedi_device * dev, comedi_subdevice * s,
	void *data, unsigned int num_bytes, unsigned int start_chan_index)
{
	unsigned int i, num_samples = num_bytes / sizeof(sampl_t);
	sampl_t *array = data;
	int resolution =
		((pci9112_board_struct *) dev->board_ptr)->ai_resolution;

	for (i = 0; i < num_samples; i++) {
		if (resolution == PCI9112_HR_AI_RESOLUTION)
			array[i] =
				(array[i] & PCI9112_HR_AI_RESOLUTION_MASK) ^
				PCI9112_HR_AI_RESOLUTION_2_CMP_BIT;
		else
			array[i] =
				((array[i] >> 4) & PCI9112_AI_RESOLUTION_MASK) ^
				PCI9112_AI_RESOLUTION_2_CMP_BIT;
	}
}

// ------------------------------------------------------------------
//
// INTERRUPT SECTION
//
// ------------------------------------------------------------------

#undef INTERRUPT_DEBUG

static irqreturn_t pci9112_interrupt(int irq, void *p_device PT_REGS_ARG)
{
	comedi_device *dev = p_device;
	comedi_subdevice *subdevice = dev->read_subdev;
	comedi_async *async;
	unsigned long irq_flags;
	unsigned char intcsr;

	if (!dev->attached) {
		// Ignore interrupt before device fully attached.
		// Might not even have allocated subdevices yet!
		return IRQ_NONE;
	}

	async = subdevice->async;

	comedi_spin_lock_irqsave(&dev->spinlock, irq_flags);

	// Check if we are source of interrupt
	intcsr = inb(dev_private->lcr_io_base +
		PLX9050_REGISTER_INTERRUPT_CONTROL);
	if (!(((intcsr & PLX9050_PCI_INTERRUPT_ENABLE) != 0)
			&& (((intcsr & (PLX9050_LINTI1_ENABLE |
							PLX9050_LINTI1_STATUS))
					==
					(PLX9050_LINTI1_ENABLE |
						PLX9050_LINTI1_STATUS))
				|| ((intcsr & (PLX9050_LINTI2_ENABLE |
							PLX9050_LINTI2_STATUS))
					==
					(PLX9050_LINTI2_ENABLE |
						PLX9050_LINTI2_STATUS))))) {
		// Not the source of the interrupt.
		// (N.B. not using PLX9050_SOFTWARE_INTERRUPT)
		comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);
		return IRQ_NONE;
	}

	if ((intcsr & (PLX9050_LINTI1_ENABLE | PLX9050_LINTI1_STATUS)) ==
		(PLX9050_LINTI1_ENABLE | PLX9050_LINTI1_STATUS)) {
		// Interrupt comes from fifo_half-full signal

		if (pci9112_is_fifo_full()) {
			comedi_spin_unlock_irqrestore(&dev->spinlock,
				irq_flags);
			comedi_error(dev, PCI9112_DRIVER_NAME " fifo overflow");
			pci9112_interrupt_clear();
			pci9112_ai_cancel(dev, subdevice);
			async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
			comedi_event(dev, subdevice);

			return IRQ_HANDLED;
		}

		if (pci9112_is_fifo_half_full()) {
			unsigned int num_samples;
			unsigned int bytes_written = 0;

#ifdef INTERRUPT_DEBUG
			printk(PCI9112_DRIVER_NAME ": fifo is half full\n");
#endif

			num_samples =
				PCI9112_FIFO_HALF_SIZE >
				dev_private->stop_counter
				&& !dev_private->stop_is_none ? dev_private->
				stop_counter : PCI9112_FIFO_HALF_SIZE;
			insw(PCI9112_IO_BASE + PCI9112_REGISTER_AD_FIFO_VALUE,
				dev_private->ai_bounce_buffer, num_samples);

			if (dev_private->scan_delay < 1) {
				bytes_written =
					comedi_buf_write_samples(subdevice,
					dev_private->ai_bounce_buffer,
					num_samples);
			} else {
				int position = 0;
				int to_read;

				while (position < num_samples) {
					if (dev_private->chunk_counter <
						dev_private->chanlist_len) {
						to_read =
							dev_private->
							chanlist_len -
							dev_private->
							chunk_counter;

						if (to_read >
							num_samples - position)
							to_read =
								num_samples -
								position;

						bytes_written +=
							comedi_buf_write_samples
							(subdevice,
							dev_private->
							ai_bounce_buffer +
							position,
							to_read);
					} else {
						to_read =
							dev_private->
							chunk_num_samples -
							dev_private->
							chunk_counter;
						if (to_read >
							num_samples - position)
							to_read =
								num_samples -
								position;

						bytes_written +=
							sizeof(sampl_t) *
							to_read;
					}

					position += to_read;
					dev_private->chunk_counter += to_read;

					if (dev_private->chunk_counter >=
						dev_private->chunk_num_samples)
						dev_private->chunk_counter = 0;
				}
			}

			dev_private->stop_counter -=
				bytes_written / sizeof(sampl_t);
		}
	}

	if ((dev_private->stop_counter == 0) && (!dev_private->stop_is_none)) {
		async->events |= COMEDI_CB_EOA;
		pci9112_ai_cancel(dev, subdevice);
	}

	/* Very important, otherwise another interrupt request will be inserted
	 * and will cause driver hangs on processing interrupt event. */

	pci9112_interrupt_clear();

	comedi_spin_unlock_irqrestore(&dev->spinlock, irq_flags);

	comedi_event(dev, subdevice);

	return IRQ_HANDLED;
}

// ------------------------------------------------------------------
//
// INSTANT ANALOG INPUT OUTPUT SECTION
//
// ------------------------------------------------------------------

//
// analog instant input
//

// VERSION PCI9112_AD_DA
static int pci9112_ai_rinsn(comedi_device * dev, comedi_subdevice * subdevice, comedi_insn * insn, lsampl_t * data)
{

	int timeout, i;
	unsigned int channel;

	//printk("PCI9112_AD_DA_a_read\n");
	channel=CR_CHAN(insn->chanspec);
//	channel=channel<<5;
//	pci9112_ai_set_channel(CR_CHAN(insn->chanspec));
//	mode=
	outw((channel & 0xf)<<5,dev->iobase+0x18);
	outb(0,dev->iobase+0x20);
	for (i = 0; i < insn->n; i++) {
		timeout =100;
		while (timeout--) {
		  if ((pci9112_ai_status() & 0x01)!=0)
				goto conversion_done;
		}

		comedi_error(dev, "A/D read timeout");
		data[i] = 0;
		return -ETIME;

	      conversion_done:

		data[i] = pci9112_ai_get_data();
		
	}

	return i;
}


#undef AI_INSN_DEBUG
//#define AI_INSN_DEBUG 1
/*
static int pci9112_ai_insn_read(comedi_device * dev,
	comedi_subdevice * subdevice, comedi_insn * insn, lsampl_t * data)
{
	int resolution =
		((pci9112_board_struct *) dev->board_ptr)->ai_resolution;

	int timeout, i;

#ifdef AI_INSN_DEBUG
	printk(PCI9112_DRIVER_NAME ": ai_insn set c/r/n = %2x/%2x/%2x\n",
		CR_CHAN((&insn->chanspec)[0]),
		CR_RANGE((&insn->chanspec)[0]), insn->n);
#endif

	pci9112_ai_channel_set(CR_CHAN((&insn->chanspec)[0]));

	if ((pci9112_ai_range_get()) != CR_RANGE((&insn->chanspec)[0])) {
		pci9112_ai_range_set(CR_RANGE((&insn->chanspec)[0]));
	}

	pci9112_fifo_reset();

	for (i = 0; i < insn->n; i++) {
		pci9112_software_trigger();

		timeout = PCI9112_AI_INSTANT_READ_TIMEOUT;

		while (timeout--) {
			if (!pci9112_is_fifo_empty())
				goto conversion_done;
		}

		comedi_error(dev, "A/D read timeout");
		data[i] = 0;
		pci9112_fifo_reset();
		return -ETIME;

	      conversion_done:

		if (resolution == PCI9112_HR_AI_RESOLUTION) {
			data[i] = pci9112_hr_ai_get_data();
		} else {
			data[i] = pci9112_ai_get_data();
		}
	}

#ifdef AI_INSN_DEBUG
	printk(PCI9112_DRIVER_NAME ": ai_insn get c/r/t = %2x/%2x/%2x\n",
		pci9112_ai_channel_get(),
		pci9112_ai_range_get(), pci9112_trigger_and_autoscan_get());
#endif

	return i;
}
*/




//
// Analog instant output
//

static int
pci9112_ao_insn_write(comedi_device * dev,
	comedi_subdevice * s, comedi_insn * insn, lsampl_t * data)
{
	int i;

	for (i = 0; i < insn->n; i++) {
		pci9112_ao_set_data(data[i]);
		dev_private->ao_readback = data[i];
	}

	return i;
}

//
// Analog output readback
//

static int pci9112_ao_insn_read(comedi_device * dev,
	comedi_subdevice * s, comedi_insn * insn, lsampl_t * data)
{
	int i;

	for (i = 0; i < insn->n; i++) {
		data[i] = dev_private->ao_readback & PCI9112_AO_RESOLUTION_MASK;
	}


	return i;
}

// ------------------------------------------------------------------
//
// DIGITAL INPUT OUTPUT SECTION
//
// ------------------------------------------------------------------

//
// Digital inputs
//

static int pci9112_di_insn_bits(comedi_device * dev,
	comedi_subdevice * subdevice, comedi_insn * insn, lsampl_t * data)
{
	lsampl_t bits;

	bits = pci9112_di_get_bits();
	data[1] = bits;

	return 2;
}

//
// Digital outputs
//

static int pci9112_do_insn_bits(comedi_device * dev,
	comedi_subdevice * subdevice, comedi_insn * insn, lsampl_t * data)
{
	lsampl_t bits;

	// Only set bits that have been masked
	// data[0] = mask
	// data[1] = bit state

	data[0] &= PCI9112_DO_MASK;

	bits = subdevice->state;
	bits &= ~data[0];
	bits |= data[0] & data[1];
	subdevice->state = bits;

	pci9112_do_set_bits(bits);

	data[1] = bits;

	return 2;
}

// ------------------------------------------------------------------
//
// INITIALISATION SECTION
//
// ------------------------------------------------------------------

//
// Reset device
//

static int pci9112_reset(comedi_device * dev)
{
	// Set trigger source to software

	plx9050_interrupt_control(dev_private->lcr_io_base, true, true, true,
		true, false);

	pci9112_trigger_source_set(dev, software);
	pci9112_pretrigger_set(dev, false);
	pci9112_autoscan_set(dev, false);

	// Reset 8254 chip

	dev_private->timer_divisor_1 = 0;
	dev_private->timer_divisor_2 = 0;

	pci9112_timer_set(dev);

	return 0;
}

//
// Attach
//
//      - Register PCI device
//      - Declare device driver capability
//

static int pci9112_attach(comedi_device * dev, comedi_devconfig * it)
{
	comedi_subdevice *subdevice;
	unsigned long io_base, io_range, lcr_io_base, lcr_io_range;
	struct pci_dev *pci_device;
	int error, i;
	const pci9112_board_struct *board;

	if (alloc_private(dev, sizeof(pci9112_private_data_struct)) < 0) {
		return -ENOMEM;
	}
	//
	// Probe the device to determine what device in the series it is.
	//

	printk("comedi%d: " PCI9112_DRIVER_NAME " driver\n", dev->minor);

	for (pci_device = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
		pci_device != NULL;
		pci_device =
		pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pci_device)) {
		//printk("Vendor id:%d %d\n",pci_device->vendor,PCI_VENDOR_ID_ADLINK);
		if (pci_device->vendor == PCI_VENDOR_ID_ADLINK_9112) {
			for (i = 0; i < pci9112_board_nbr; i++) {
				//printk("deviceid %d:\n",pci_device->device);
				if (pci9112_boards[i].device_id ==
					pci_device->device) {
					// was a particular bus/slot requested?
					if ((it->options[0] != 0)
						|| (it->options[1] != 0)) {
						// are we on the wrong bus/slot?
						if (pci_device->bus->number !=
							it->options[0]
							|| PCI_SLOT(pci_device->
								devfn) !=
							it->options[1]) {
							continue;
						}
					}

					dev->board_ptr = pci9112_boards + i;
					board = (pci9112_board_struct *) dev->
						board_ptr;
					dev_private->pci_device = pci_device;
					goto found;
				}
			}
		}
	}

	printk("comedi%d: no supported board found! (req. bus/slot : %d/%d)\n",
		dev->minor, it->options[0], it->options[1]);
	return -EIO;

      found:

	printk("comedi%d: found %s (b:s:f=%d:%d:%d) , irq=%d\n",
		dev->minor,
		pci9112_boards[i].name,
		pci_device->bus->number,
		PCI_SLOT(pci_device->devfn),
		PCI_FUNC(pci_device->devfn), pci_device->irq);

	// TODO: Warn about non-tested boards.

	switch (board->device_id) {
	};

	// Read local configuration register base address [PCI_BASE_ADDRESS #1].

	lcr_io_base = pci_resource_start(pci_device, 0); //1 dans 9111
	lcr_io_range = pci_resource_len(pci_device, 0); //0 dans 9112

	//lcr_io_base -=lcr_io_range; //pb

	printk("comedi%d: local configuration registers at address 0x%4lx [0x%4lx]\n", dev->minor, lcr_io_base, lcr_io_range);

	// Enable PCI device and request regions
	if (comedi_pci_enable(pci_device, PCI9112_DRIVER_NAME) < 0) {
		printk("comedi%d: Failed to enable PCI device and request regions\n", dev->minor);
		return -EIO;
	}
	// Read PCI6308 register base address [PCI_BASE_ADDRESS #2].

	io_base = pci_resource_start(pci_device, 2);//1 dans 9111
	//io_range = pci_resource_end(pci_device,2) - io_base+1; //pb
	io_range = pci_resource_len(pci_device, 2);   

	printk("comedi%d: 6503 registers at address 0x%4lx [0x%4lx]\n",
		dev->minor, io_base, io_range);

	dev->iobase = io_base;

	dev->board_name = board->name;
	dev_private->io_range = io_range;
	dev_private->is_valid = 0;
	dev_private->lcr_io_base = lcr_io_base;
	dev_private->lcr_io_range = lcr_io_range;

	pci9112_reset(dev);

	// Irq setup

	dev->irq = 0;
	if (pci_device->irq > 0) {
		if (comedi_request_irq(pci_device->irq,
				pci9112_interrupt,
				IRQF_SHARED, PCI9112_DRIVER_NAME, dev) != 0) {
			printk("comedi%d: unable to allocate irq  %u\n",
				dev->minor, pci_device->irq);
			return -EINVAL;
		}
	}
	dev->irq = pci_device->irq;

	//
	// TODO: Add external multiplexer setup (according to option[2]).
	//

	if ((error = alloc_subdevices(dev, 4)) < 0)
		return error;

	subdevice = dev->subdevices + 0;
	dev->read_subdev = subdevice;

	subdevice->type = COMEDI_SUBD_AI;
	subdevice->subdev_flags = SDF_READABLE | SDF_COMMON | SDF_CMD_READ;

	//
	// TODO: Add external multiplexer data
	//
	//    if (devpriv->usemux) { subdevice->n_chan = devpriv->usemux; }
	//    else { subdevice->n_chan = this_board->n_aichan; }
	//

	subdevice->n_chan = board->ai_channel_nbr;
	subdevice->maxdata = board->ai_resolution_mask;
	subdevice->len_chanlist = board->ai_channel_nbr;
	subdevice->range_table = board->ai_range_list;
	subdevice->cancel = pci9112_ai_cancel;
//	subdevice->insn_read = pci9112_ai_insn_read; // Version PCI9111
	subdevice->insn_read = pci9112_ai_rinsn;     // Version PCI9112_AD_DA
	subdevice->do_cmdtest = pci9112_ai_do_cmd_test;
	subdevice->do_cmd = pci9112_ai_do_cmd;
	subdevice->munge = pci9112_ai_munge;

	subdevice = dev->subdevices + 1;
	subdevice->type = COMEDI_SUBD_AO;
	subdevice->subdev_flags = SDF_WRITABLE | SDF_COMMON;
	subdevice->n_chan = board->ao_channel_nbr;
	subdevice->maxdata = board->ao_resolution_mask;
	subdevice->len_chanlist = board->ao_channel_nbr;
	subdevice->range_table = board->ao_range_list;
	subdevice->insn_write = pci9112_ao_insn_write;
	subdevice->insn_read = pci9112_ao_insn_read;

	subdevice = dev->subdevices + 2;
	subdevice->type = COMEDI_SUBD_DI;
	subdevice->subdev_flags = SDF_READABLE;
	subdevice->n_chan = PCI9112_DI_CHANNEL_NBR;
	subdevice->maxdata = 1;
	subdevice->range_table = &range_digital;
	subdevice->insn_bits = pci9112_di_insn_bits;

	subdevice = dev->subdevices + 3;
	subdevice->type = COMEDI_SUBD_DO;
	subdevice->subdev_flags = SDF_READABLE | SDF_WRITABLE;
	subdevice->n_chan = PCI9112_DO_CHANNEL_NBR;
	subdevice->maxdata = 1;
	subdevice->range_table = &range_digital;
	subdevice->insn_bits = pci9112_do_insn_bits;

	dev_private->is_valid = 1;

	return 0;
}

//
// Detach
//

static int pci9112_detach(comedi_device * dev)
{
	// Reset device

	if (dev->private != 0) {
		if (dev_private->is_valid)
			pci9112_reset(dev);

	}
	// Release previously allocated irq

	if (dev->irq != 0) {
		comedi_free_irq(dev->irq, dev);
	}

	if (dev_private != 0 && dev_private->pci_device != 0) {
		if (dev->iobase) {
			comedi_pci_disable(dev_private->pci_device);
		}
		pci_dev_put(dev_private->pci_device);
	}

	return 0;
}
