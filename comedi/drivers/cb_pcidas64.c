/*
    cb_pcidas64.c
    This is a driver for the ComputerBoards/MeasurementComputing PCI-DAS
    64xxx cards.

    Author:  Frank Mori Hess <fmhess@uiuc.edu>
    Copyright (C) 2001 Frank Mori Hess <fmhess@uiuc.edu>

    Thanks go to Steve Rosenbluth for providing the source code for
    his pci-das6402 driver, and source code for working QNX pci-6402
    drivers by Greg Laird and Mariusz Bogacz.  None of the code was
    used directly here, but was useful as an additional source of
    documentation on how to program the boards.

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-8 David A. Schleef <ds@stm.lbl.gov>

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
   PCI-DAS64xxx series with the PLX 9080 PCI controller.
Author: Frank Mori Hess <fmhess@uiuc.edu>
Status: Experimental
Updated: 2001-9-19
Devices: [Measurement Computing] PCI-DAS6402/16 (cb_pcidas64),
  PCI-DAS6402/12, PCI-DAS64/M1/16, PCI-DAS64/M2/16,
  PCI-DAS64/M3/16, PCI-DAS6402/16/JR, PCI-DAS64/M1/16/JR,
  PCI-DAS64/M2/16/JR, PCI-DAS64/M3/16/JR, PCI-DAS64/M1/14,
  PCI-DAS64/M2/14, PCI-DAS64/M3/14
Configuration options:
   [0] - PCI bus of device (optional)
   [1] - PCI slot of device (optional)

Basic insn support should work, but untested as far as I know.
Has command support for analog input, which may also work.  Support
for pci dma transfers can be enabled by editing the source to #define
PCIDMA instead of #undef'ing it. This driver is in need of stout-hearted
testers who aren't afraid to crash their computers in the name of progress.
Feel free to send and success/failure reports to author.

Some devices are not identified because the PCI device IDs are not known.
*/

/*

TODO:
	needs to be tested and debugged
	command support for ao
	calibration subdevice
	user counter subdevice
	there are a number of boards this driver will support when they are
		fully released, but does not yet since the pci device id numbers
		are not yet available.
	need to take care to prevent ai and ao from affecting each others register bits
	support prescaled 100khz clock for slow pacing
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/malloc.h>
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

#define PCIDAS64_DEBUG	// enable debugging code
//#undef PCIDAS64_DEBUG	// disable debugging code
//#define PCIDMA	// enable pcidma code
#undef PCIDMA	// disable pcidma code

// PCI vendor number of ComputerBoards/MeasurementComputing
#define PCI_VENDOR_ID_CB	0x1307
#define TIMER_BASE 25	// 40MHz master clock
#define PRESCALED_TIMER_BASE	10000	// 100kHz 'prescaled' clock for slow aquisition, maybe I'll support this someday
#define QUARTER_AI_FIFO_SIZE 2048	// 1/4 analog input fifo size
// size in bytes of transfers used for dma transfers, also size of buffers that make up dma ring
#define DMA_TRANSFER_SIZE 0x4000
// number of dma transfers we will chain together into a ring (and the number of dma buffers we maintain)
#define DMA_RING_COUNT 10

/* PCI-DAS64xxx base addresses */

// indices of base address regions
#define PLX9080_BADRINDEX 0
#define MAIN_BADRINDEX 2
#define DIO_COUNTER_BADRINDEX 3
// size on bytes of various memory io regions
#define PLX9080_IOSIZE 0xec
#define MAIN_IOSIZE 0x302
#define DIO_COUNTER_IOSIZE 0x29

// devpriv->main_iobase registers
// write-only
#define INTR_ENABLE_REG	0x0	// interrupt enable register
#define    ADC_INTR_SRC_MASK	0x3	// bits that set adc interrupt source
#define    ADC_INTR_QFULL_BITS	0x0	// interrupt fifo quater full
#define    ADC_INTR_EOC_BITS	0x1	// interrupt end of conversion
#define    ADC_INTR_EOSCAN_BITS	0x2	// interrupt end of scan
#define    ADC_INTR_EOSEQ_BITS	0x3	// interrupt end of sequence (probably wont use this it's pretty fancy)
#define    EN_ADC_INTR_SRC_BIT	0x4	// enable adc interrupt source
#define    EN_ADC_DONE_INTR_BIT	0x8	// enable adc aquisition done interrupt
#define    EN_ADC_ACTIVE_INTR_BIT	0x200	// enable adc active interrupt
#define    EN_ADC_STOP_INTR_BIT	0x400	// enable adc stop trigger interrupt
#define    EN_DAC_ACTIVE_INTR_BIT	0x800	// enable dac active interrupt
#define    EN_DAC_UNDERRUN_BIT	0x4000	// enable dac underrun status bit
#define    EN_ADC_OVERRUN_BIT	0x8000	// enable adc overrun status bit
#define HW_CONFIG_REG	0x2	// hardware config register
#define    HW_CONFIG_DUMMY_BITS	0x2400	// bits that don't do anything yet but are given default values
#define    HW_CONFIG_DUMMY_BITS_6402	0x0400	// dummy bits in 6402 manual are slightly different, probably doesn't matter
#define    EXT_QUEUE	0x200	// use external channel/gain queue (more versatile than internal queue)
#define FIFO_SIZE_REG	0x4	// allows adjustment of fifo sizes, we will always use maximum
#define    FIFO_SIZE_DUMMY_BITS	0xf038	// bits that don't do anything yet but are given default values
#define    ADC_FIFO_SIZE_MASK	0x7	// bits that set adc fifo size
#define    ADC_FIFO_8K_BITS	0x0	// 8 kilosample adc fifo
#define    DAC_FIFO_SIZE_MASK	0xf00	// bits that set dac fifo size
#define    DAC_FIFO_16K_BITS 0x0
#define ADC_CONTROL0_REG	0x10	// adc control register 0
#define    ADC_START_TRIG_FALLING_BIT	0x20	// trig 1 uses falling edge
#define    ADC_START_TRIG_SOFT_BITS	0x10
#define    ADC_START_TRIG_EXT_BITS	0x20
#define    ADC_START_TRIG_ANALOG_BITS	0x30
#define    ADC_START_TRIG_MASK	0x30
#define    ADC_EXT_CONV_FALLING_BIT	0x800	// external pacing uses falling edge
#define    ADC_ENABLE_BIT	0x8000	// master adc enable
#define ADC_CONTROL1_REG	0x12	// adc control register 1
#define    ADC_CONTROL1_DUMMY_BITS	0x1	// dummy bits for adc control register 1
#define    SW_NOGATE_BIT	0x40	// disables software gate of adc
#define    ADC_MODE_BITS(x)	(((x) & 0xf) << 12)
#define ADC_SAMPLE_INTERVAL_LOWER_REG	0x16	// lower 16 bits of sample interval counter
#define ADC_SAMPLE_INTERVAL_UPPER_REG	0x18	// upper 8 bits of sample interval counter
#define ADC_DELAY_INTERVAL_LOWER_REG	0x1a	// lower 16 bits of delay interval counter
#define ADC_DELAY_INTERVAL_UPPER_REG	0x1c	// upper 8 bits of delay interval counter
#define ADC_COUNT_LOWER_REG	0x1e	// lower 16 bits of hardware conversion/scan counter
#define ADC_COUNT_UPPER_REG	0x20	// upper 8 bits of hardware conversion/scan counter
#define ADC_START_REG	0x22	// software trigger to start aquisition
#define ADC_CONVERT_REG	0x24	// initiates single conversion
#define ADC_QUEUE_CLEAR_REG	0x26	// clears adc queue
#define ADC_QUEUE_LOAD_REG	0x28	// loads adc queue
#define    CHAN_BITS(x)	((x) & 0x3f)
#define    GAIN_BITS(x)	(((x) & 0x3) << 8)	// translates range index to gain bits
#define    UNIP_BIT(x)	(((x) & 0x4) << 11)	// translates range index to unipolar/bipolar bit
#define    SE_BIT	0x1000	// single-ended/ differential bit
#define    QUEUE_EOSEQ_BIT	0x4000	// queue end of sequence
#define    QUEUE_EOSCAN_BIT	0x8000	// queue end of scan
#define ADC_BUFFER_CLEAR_REG	0x2a
#define ADC_QUEUE_HIGH_REG	0x2c	// high channel for internal queue, use CHAN_BITS() macro above
#define DAC_CONTROL0_REG	0x50	// dac control register 0
#define    DAC_ENABLE_BIT	0x8000	// dac controller enable bit
#define DAC_CONTROL1_REG	0x52	// dac control register 0
#define    DAC_RANGE_BITS(channel, range)	(((range) & 0x3) << (2 * ((channel) & 0x1)))
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
#define   PIPE_FULL_BIT(x)	(0x400 << ((x) & 0x1))
#define   HW_REVISION(x)	(((x) >> 12) & 0xf)
#define PIPE1_READ_REG	0x4
#define ADC_READ_PNTR_REG	0x8
#define ADC_WRITE_PNTR_REG	0xc
#define PREPOST_REG	0x14
#define   ADC_UPP_READ_PNTR_CODE(x)	(((x) >> 12) & 0x3)
#define   ADC_UPP_WRITE_PNTR_CODE(x)	(((x) >> 14) & 0x3)
// read-write
#define ADC_QUEUE_FIFO_REG	0x100	// external channel/gain queue, uses same bits as ADC_QUEUE_LOAD_REG
#define ADC_FIFO_REG 0x200	// adc data fifo

// devpriv->dio_counter_iobase registers
#define DIO_8255_OFFSET	0x0
#define DO_REG	0x20
#define DI_REG	0x28

// analog input ranges for most boards
static comedi_lrange ai_ranges =
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

// analog output ranges
static comedi_lrange ao_ranges =
{
	4,
	{
		BIP_RANGE(5),
		BIP_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(10),
	}
};

typedef struct pcidas64_board_struct
{
	char *name;
	int device_id;	// pci device id
	int ai_se_chans;	// number of ai inputs in single-ended mode
	int ai_bits;	// analog input resolution
	int ai_speed;	// fastest conversion period in ns
	int ao_nchan;	// number of analog out channels
	int ao_scan_speed;	// analog output speed (for a scan, not conversion)
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
	},
	{
		name:		"pci-das6402/12",	// XXX check
		device_id:	0x1e,
		ai_se_chans:	64,
		ai_bits:	12,
		ai_speed:	5000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m1/16",
		device_id:	0x35,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	1000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m2/16",
		device_id:	0x36,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	500,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m3/16",
		device_id:	0x37,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	333,
		ao_nchan: 2,
		ao_scan_speed:	10000,
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
	},
	{
		name:		"pci-das64/m1/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	1000,
		ao_nchan: 0,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m2/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	500,
		ao_nchan: 0,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m3/16/jr",
		device_id:	0 // XXX,
		ai_se_chans:	64,
		ai_bits:	16,
		ai_speed:	333,
		ao_nchan: 0,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m1/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	1000,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m2/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	500,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
	{
		name:		"pci-das64/m3/14",
		device_id:	0,	// XXX
		ai_se_chans:	64,
		ai_bits:	14,
		ai_speed:	333,
		ao_nchan: 2,
		ao_scan_speed:	10000,
	},
#endif

};
// Number of boards in cb_pcidas_boards
#define N_BOARDS	(sizeof(pcidas64_boards) / sizeof(pcidas64_board))

static struct pci_device_id pcidas64_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_CB, 0x001d, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x001e, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x0035, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x0036, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x0037, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, pcidas64_pci_table);

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((pcidas64_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
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
	u32 local_main_iobase;
	volatile unsigned int ai_count;	// number of analog input samples remaining
	u16 *ai_buffer[DMA_RING_COUNT];	// dma buffers for analog input
	dma_addr_t ai_buffer_phys_addr[DMA_RING_COUNT];	// physical addresses of ai dma buffers
	struct plx_dma_desc *dma_desc;	// array of dma descriptors read by plx9080, allocated to get proper alignment
	dma_addr_t dma_desc_phys_addr;	// physical address of dma descriptor array
	volatile unsigned int dma_index;	// index of the dma descriptor/buffer that is currently being used
	volatile unsigned int ao_count;	// number of analog output samples remaining
	unsigned int ao_value[2];	// remember what the analog outputs are set to, to allow readback
	unsigned int hw_revision;	// stc chip hardware revision number
	unsigned int do_bits;	// remember digital ouput levels
	volatile unsigned int intr_enable_bits;	// bits to send to INTR_ENABLE_REG register
} pcidas64_private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((pcidas64_private *)dev->private)

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
static int dio_callback(int dir, int port, int data, void *arg);
static int di_rbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static int do_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data);
static void check_adc_timing(comedi_cmd *cmd);
static unsigned int get_divisor(unsigned int ns, unsigned int flags);
static int grey2int(int code);

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_cb_pcidas);

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
	u32 local_range, local_decode;
#ifdef PCIDMA
	unsigned int bits;
#endif

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
		if(pcidev->vendor != PCI_VENDOR_ID_CB)
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
	devpriv->hw_dev = pcidev;

	//Initialize dev->board_name
	dev->board_name = thisboard->name;

	/* Initialize devpriv->control_status and devpriv->adc_fifo to point to
	 * their base address.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
	plx9080_iobase =
		pcidev->base_address[PLX9080_BADRINDEX] &
		PCI_BASE_ADDRESS_MEM_MASK;
	main_iobase =
		pcidev->base_address[MAIN_BADRINDEX] &
		PCI_BASE_ADDRESS_MEM_MASK;
	dio_counter_iobase =
		pcidev->base_address[DIO_COUNTER_BADRINDEX] &
		PCI_BASE_ADDRESS_MEM_MASK;
#else
	if(pci_enable_device(pcidev))
		return -EIO;
#ifdef PCIDMA
	pci_set_master(pcidev);
#endif
	plx9080_iobase =
		pcidev->resource[PLX9080_BADRINDEX].start &
		PCI_BASE_ADDRESS_MEM_MASK;
	main_iobase =
		pcidev->resource[MAIN_BADRINDEX].start &
		PCI_BASE_ADDRESS_MEM_MASK;
	dio_counter_iobase =
		pcidev->resource[DIO_COUNTER_BADRINDEX].start &
		PCI_BASE_ADDRESS_MEM_MASK;

	if(check_mem_region(plx9080_iobase, PLX9080_IOSIZE))
	{
		/* Couldn't allocate io space */
		printk(KERN_WARNING "couldn't allocate IO space\n");
		return -EIO;
	}
	if(check_mem_region(main_iobase, MAIN_IOSIZE))
	{
		/* Couldn't allocate io space */
		printk(KERN_WARNING "couldn't allocate IO space\n");
		return -EIO;
	}
	if(check_mem_region(dio_counter_iobase, DIO_COUNTER_IOSIZE))
	{
		/* Couldn't allocate io space */
		printk(KERN_WARNING "couldn't allocate IO space\n");
		return -EIO;
	}

	request_mem_region(plx_iobase, PLX9080_IOSIZE, "cb_pcidas64");
	devpriv->plx9080_phys_iobase = dio_counter_iobase;
	request_mem_region(main_iobase, MAIN_IOSIZE, "cb_pcidas64");
	devpriv->main_phys_iobase = dio_counter_iobase;
	request_mem_region(dio_counter_iobase, DIO_COUNTER_IOSIZE, "cb_pcidas64");
	devpriv->dio_counter_phys_iobase = dio_counter_iobase;

#endif

	// remap, won't work with 2.0 kernels but who cares
	devpriv->plx9080_iobase = (unsigned long)ioremap(plx9080_iobase, PLX9080_IOSIZE);
	devpriv->main_iobase = (unsigned long)ioremap(main_iobase, MAIN_IOSIZE);
	devpriv->dio_counter_iobase = (unsigned long)ioremap(dio_counter_iobase, DIO_COUNTER_IOSIZE);

	// figure out what local address is for main_iobase
	local_range = readl(devpriv->plx9080_iobase + PLX_LAS0RNG_REG) & LRNG_MEM_MASK;
	local_decode = readl(devpriv->plx9080_iobase + PLX_LAS0MAP_REG) & local_range & LMAP_MEM_MASK ;
	devpriv->local_main_iobase = (devpriv->main_phys_iobase & ~local_range) | local_decode;

	devpriv->hw_revision = HW_REVISION(readw(devpriv->main_iobase + HW_STATUS_REG));

	// get irq
	if(comedi_request_irq(pcidev->irq, handle_interrupt, SA_SHIRQ, "cb_pcidas64", dev ))
	{
		printk(" unable to allocate irq %d\n", pcidev->irq);
		return -EINVAL;
	}
	dev->irq = pcidev->irq;

#ifdef PCIDAS64_DEBUG

printk(" plx9080 phys io addr 0x%lx\n", devpriv->plx9080_phys_iobase);
printk(" main phys io addr 0x%lx\n", devpriv->main_phys_iobase);
printk(" diocounter phys io addr 0x%lx\n", devpriv->dio_counter_phys_iobase);
printk(" irq %i\n", dev->irq);

printk(" plx9080 virt io addr 0x%lx\n", devpriv->plx9080_iobase);
printk(" main virt io addr 0x%lx\n", devpriv->main_iobase);
printk(" diocounter virt io addr 0x%lx\n", devpriv->dio_counter_iobase);
printk(" irq %i\n", dev->irq);

printk(" local main io addr 0x%ulx\n", devpriv->local_main_iobase);

printk(" stc hardware revision %i\n", devpriv->hw_revision);

// plx9080 dump
printk(" plx interrupt status 0x%x\n", readl(devpriv->plx9080_iobase + PLX_INTRCS_REG));
printk(" plx id bits 0x%x\n", readl(devpriv->plx9080_iobase + PLX_ID_REG));
printk(" plx hardware revision 0x%x\n", readl(devpriv->plx9080_iobase + PLX_REVISION_REG));
printk(" plx dma channel 0 mode 0x%x\n", readl(devpriv->plx9080_iobase + PLX_DMA0_MODE_REG));
printk(" plx dma channel 0 pci address 0x%x\n", readl(devpriv->plx9080_iobase + PLX_DMA0_PCI_ADDRESS_REG));
printk(" plx dma channel 0 local address 0x%x\n", readl(devpriv->plx9080_iobase + PLX_DMA0_LOCAL_ADDRESS_REG));
printk(" plx dma channel 0 transfer size 0x%x\n", readl(devpriv->plx9080_iobase + PLX_DMA0_TRANSFER_SIZE_REG));
printk(" plx dma channel 0 descriptor 0x%x\n", readl(devpriv->plx9080_iobase + PLX_DMA0_DESCRIPTOR_REG));
printk(" plx dma channel 0 command status 0x%x\n", readl(devpriv->plx9080_iobase + PLX_DMA0_CS_REG));
printk(" plx dma channel 0 threshold 0x%x\n", readl(devpriv->plx9080_iobase + PLX_DMA0_THRESHOLD_REG));

#endif

#ifdef PCIDMA
	// alocate pci dma buffers
	for(index = 0; index < DMA_RING_COUNT; index++)
	{
		devpriv->ai_buffer[index] =
			pci_alloc_consistent(devpriv->hw_dev, DMA_TRANSFER_SIZE, &devpriv->ai_buffer_phys_addr[index]);
	}
	// allocate dma descriptors
	devpriv->dma_desc =
		pci_alloc_consistent(devpriv->hw_dev, sizeof(struct plx_dma_desc) * DMA_RING_COUNT,
		&devpriv->dma_desc_phys_addr);
	// initialize dma descriptors
	for(index = 0; index < DMA_RING_COUNT; index++)
	{
		devpriv->dma_desc[index].pci_start_addr = virt_to_bus(devpriv->ai_buffer[index]);
		devpriv->dma_desc[index].local_start_addr = devpriv->local_main_iobase + ADC_FIFO_REG;
		devpriv->dma_desc[index].transfer_size = DMA_TRANSFER_SIZE;
		devpriv->dma_desc[index].next = virt_to_bus(&devpriv->dma_desc[(index + 1) % (DMA_RING_COUNT)]) |
			PLX_DESC_IN_PCI_BIT | PLX_INTR_TERM_COUNT | PLX_XFER_LOCAL_TO_PCI;
	}
	// do some initialization of plx9080 dma registers
	/* XXX there are some other bits that can be set here that might
	 * improve performance, but I just want to get it working */
	bits = 0;
	// localspace0 bus is 16 bits wide
	bits |= PLX_LOCAL_BUS_16_WIDE_BITS;
	// enable ready input, not sure if this is necessary
	bits |= PLX_DMA_EN_READYIN_BIT;
	// enable dma chaining
	bits |= PLX_EN_CHAIN_BIT;
	// enable interrupt on dma done (probably don't need this, since chain never finishes)
	bits |= PLX_EN_DMA_DONE_INTR_BIT;
	// don't increment local address during transfers (we are transferring from a fixed fifo register)
	bits |= PLX_LOCAL_ADDR_CONST_BIT;
	// route dma interrupt to pci bus
	bits |= PLX_DMA_INTR_PCI_BIT;
	writel(bits, devpriv->plx9080_iobase + PLX_DMA0_MODE_REG);

#endif

	// disable dma transfers
	writeb(0, devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
	writeb(0, devpriv->plx9080_iobase + PLX_DMA1_CS_REG);

/*
 * Allocate the subdevice structures.
 */
	dev->n_subdevices = 7;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog input subdevice */
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON | SDF_DIFF;
	/* XXX Number of inputs in differential mode is ignored */
	s->n_chan = thisboard->ai_se_chans;
	s->len_chanlist = 8092;
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = &ai_ranges;
	s->insn_read = ai_rinsn;
	s->do_cmd = ai_cmd;
	s->do_cmdtest = ai_cmdtest;
	s->cancel = ai_cancel;

	/* analog output subdevice */
	s = dev->subdevices + 1;
	if(thisboard->ao_nchan)
	{
	//	dev->write_subdev = s;
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_GROUND;
		s->n_chan = thisboard->ao_nchan;
		// analog out resolution is the same as analog input resolution, so use ai_bits
		s->maxdata = (1 << thisboard->ai_bits) - 1;
		s->range_table = &ao_ranges;
		s->insn_read = ao_readback_insn;
		s->insn_write = ao_winsn;
	//	s->do_cmdtest = ao_cmdtest;
	//	s->do_cmd = ao_cmd;
	//	s->len_chanlist = thisboard->ao_nchan;
	//	s->cancel = ao_cancel;
	} else
	{
		s->type = COMEDI_SUBD_UNUSED;
	}

	// digital input
	s = dev->subdevices + 2;
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = di_rbits;

	// digital output
	s = dev->subdevices + 3;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE | SDF_READABLE;
	s->n_chan = 4;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = do_wbits;

	/* 8255 */
	s = dev->subdevices + 4;
	subdev_8255_init(dev, s, dio_callback,
		(void*) (devpriv->dio_counter_iobase + DIO_8255_OFFSET));

	// user counter subd XXX
	s = dev->subdevices + 5;
	s->type = COMEDI_SUBD_UNUSED;

	// calibration subd XXX
	s = dev->subdevices + 6;
	s->type = COMEDI_SUBD_UNUSED;

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
#ifdef PCIDMA
	unsigned int i;
#endif

	printk("comedi%d: cb_pcidas: remove\n",dev->minor);

	if(dev->irq)
		comedi_free_irq(dev->irq, dev);
	if(devpriv)
	{
		if(devpriv->plx9080_iobase)
			iounmap((void*)devpriv->plx9080_iobase);
		if(devpriv->main_iobase)
			iounmap((void*)devpriv->main_iobase);
		if(devpriv->dio_counter_iobase)
			iounmap((void*)devpriv->dio_counter_iobase);
		if(devpriv->plx9080_phys_iobase)
			release_mem_region(devpriv->plx9080_iobase, PLX9080_IOSIZE);
		if(devpriv->main_iobase)
			release_mem_region(devpriv->main_phys_iobase, MAIN_IOSIZE);
		if(devpriv->dio_counter_iobase)
			release_mem_region(devpriv->dio_counter_phys_iobase, DIO_COUNTER_IOSIZE);
#ifdef PCIDMA
		// free pci dma buffers
		for(i = 0; i < DMA_RING_COUNT; i++)
		{
			if(devpriv->ai_buffer[i])
				pci_free_consistent(devpriv->hw_dev, DMA_BUFFER_SIZE,
					devpriv->ai_buffer[i], devpriv->ai_buffer_phys_addr[i]);
		}
		// free dma descriptors
		if(devpriv->dma_desc)
			pci_free_consistent(devpriv->hw_dev, sizeof(struct plx_dma_desc) * DMA_RING_COUNT,
				devpriv->dma_desc, devpriv->dma_desc_phys_addr);
#endif
	}
	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices + 4);

	return 0;
}

static int ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	unsigned int bits, n, i;
	const int timeout = 1000;

	// disable card's analog input interrupt sources
	devpriv->intr_enable_bits &= ~EN_ADC_INTR_SRC_BIT & ~EN_ADC_DONE_INTR_BIT &
		~EN_ADC_ACTIVE_INTR_BIT & ~EN_ADC_STOP_INTR_BIT & ~EN_ADC_OVERRUN_BIT;
	writew(devpriv->intr_enable_bits, devpriv->main_iobase + INTR_ENABLE_REG);

	/* disable pacing, triggering, etc */
	writew(ADC_ENABLE_BIT, devpriv->main_iobase + ADC_CONTROL0_REG);
	writew(ADC_CONTROL1_DUMMY_BITS, devpriv->main_iobase + ADC_CONTROL1_REG);

	// use internal queue
	writew(HW_CONFIG_DUMMY_BITS, devpriv->main_iobase + HW_CONFIG_REG);

	// load internal queue
	bits = 0;
	// set channel
	bits |= CHAN_BITS(CR_CHAN(insn->chanspec));
	// set gain
	bits |= GAIN_BITS(CR_RANGE(insn->chanspec));
	// set unipolar / bipolar
	bits |= UNIP_BIT(CR_RANGE(insn->chanspec));
	// set single-ended / differential
	if(CR_AREF(insn->chanspec) != AREF_DIFF)
		bits |= SE_BIT;
	// set stop channel
	writew(CHAN_BITS(CR_CHAN(insn->chanspec)), devpriv->main_iobase + ADC_QUEUE_HIGH_REG);
	// set start channel, and rest of settings
	writew(bits, devpriv->main_iobase + ADC_QUEUE_LOAD_REG);

	// clear adc buffer
	writew(0, devpriv->main_iobase + ADC_BUFFER_CLEAR_REG);

	for(n = 0; n < insn->n; n++)
	{
		/* trigger conversion */
		writew(0, devpriv->main_iobase + ADC_CONVERT_REG);

		// wait for data
		for(i = 0; i < timeout; i++)
		{
			if(!(readw(devpriv->main_iobase + HW_STATUS_REG) & ADC_ACTIVE_BIT))
				break;
		}
		if(i == timeout)
		{
			comedi_error(dev, " analog input read insn timed out");
			return -ETIME;
		}
		data[n] = readw(devpriv->main_iobase + PIPE1_READ_REG);
	}

	return n;
}

static int ai_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd)
{	int err = 0;
	int tmp;
	unsigned int tmp_arg, tmp_arg2;
	int i;
	int aref;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src || tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER | TRIG_FOLLOW;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src) err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
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
		cmd->convert_src != TRIG_EXT) err++;
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
		if(cmd->convert_arg < thisboard->ai_speed)
		{
			cmd->convert_arg = thisboard->ai_speed;
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
	}

	if(err) return 5;

	return 0;
}

static int ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	u32 bits;
	unsigned int convert_counter_value;
	unsigned int scan_counter_value;
	unsigned int i;

	// disable card's interrupt sources
	// disable card's analog input interrupt sources
	devpriv->intr_enable_bits &= ~EN_ADC_INTR_SRC_BIT & ~EN_ADC_DONE_INTR_BIT &
		~EN_ADC_ACTIVE_INTR_BIT & ~EN_ADC_STOP_INTR_BIT & ~EN_ADC_OVERRUN_BIT;
	writew(devpriv->intr_enable_bits, devpriv->main_iobase + INTR_ENABLE_REG);

	/* disable pacing, triggering, etc */
	writew(0, devpriv->main_iobase + ADC_CONTROL0_REG);
	writew(ADC_CONTROL1_DUMMY_BITS, devpriv->main_iobase + ADC_CONTROL1_REG);

	// use external queue
	writew(EXT_QUEUE | HW_CONFIG_DUMMY_BITS, devpriv->main_iobase + HW_CONFIG_REG);

	// set fifo size
	// XXX this sets dac fifo size too
	writew(ADC_FIFO_8K_BITS | FIFO_SIZE_DUMMY_BITS, devpriv->main_iobase + FIFO_SIZE_REG);

	// set conversion pacing
	if(cmd->convert_src == TRIG_TIMER)
	{
		check_adc_timing(cmd);
		// supposed to load counter with desired divisor minus 3
		convert_counter_value = cmd->convert_arg / TIMER_BASE - 3;
		// load lower 16 bits
		writew(convert_counter_value & 0xffff, devpriv->main_iobase + ADC_SAMPLE_INTERVAL_LOWER_REG);
		// load upper 8 bits
		writew((convert_counter_value >> 16) & 0xff, devpriv->main_iobase + ADC_SAMPLE_INTERVAL_UPPER_REG);
		// set scan pacing
		scan_counter_value = 0;
		if(cmd->scan_begin_src == TRIG_TIMER)
		{
			// figure out how long we need to delay at end of scan
			scan_counter_value = (cmd->scan_begin_arg - (cmd->convert_arg * (cmd->chanlist_len - 1)))
				/ TIMER_BASE;
		}else if(cmd->scan_begin_src == TRIG_FOLLOW)
		{
			scan_counter_value = cmd->convert_arg / TIMER_BASE;
		}
		// load lower 16 bits
		writew(scan_counter_value & 0xffff, devpriv->main_iobase + ADC_DELAY_INTERVAL_LOWER_REG);
		// load upper 8 bits
		writew((scan_counter_value >> 16) & 0xff, devpriv->main_iobase + ADC_DELAY_INTERVAL_UPPER_REG);
	}

	// load hardware conversion counter with non-zero value so it doesn't mess with us
	writew(~0, devpriv->main_iobase + ADC_COUNT_LOWER_REG);

	// set software count
	if(cmd->stop_src == TRIG_COUNT)
		devpriv->ai_count = cmd->stop_arg * cmd->chanlist_len;

	/* XXX cannot write to queue fifo while dac fifo is being written to
	 * ( need spinlock, or try to use internal queue instead */
	// clear queue pointer
	writew(0, devpriv->main_iobase + ADC_QUEUE_CLEAR_REG);
	// load external queue
	for(i = 0; i < cmd->chanlist_len; i++)
	{
		bits = 0;
		// set channel
		bits |= CHAN_BITS(CR_CHAN(cmd->chanlist[i]));
		// set gain
		bits |= GAIN_BITS(CR_RANGE(cmd->chanlist[i]));
		// set unipolar / bipolar
		bits |= UNIP_BIT(CR_RANGE(cmd->chanlist[i]));
		// set single-ended / differential
		if(CR_AREF(cmd->chanlist[i]) != AREF_DIFF)
			bits |= SE_BIT;
		// mark end of queue
		if(i == cmd->chanlist_len - 1)
			bits |= QUEUE_EOSCAN_BIT | QUEUE_EOSEQ_BIT;
		writew(bits, devpriv->main_iobase + ADC_QUEUE_FIFO_REG);
	}
	// prime queue holding register
	writew(0, devpriv->main_iobase + ADC_QUEUE_LOAD_REG);

	// clear adc buffer
	writew(0, devpriv->main_iobase + ADC_BUFFER_CLEAR_REG);

#ifdef PCIDMA
	devpriv->dma_index = 0;
	// give location of first dma descriptor
	bits = virt_to_bus(devpriv->dma_desc);
	bits |= PLX_DESC_IN_PCI_BIT | PLX_INTR_TERM_COUNT | PLX_XFER_LOCAL_TO_PCI;
	writel(bits, devpriv->plx9080_iobase + PLX_DMA0_DESCRIPTOR_REG);
	// enable dma transfer
	writeb(PLX_DMA_EN_BIT | PLX_DMA_START_BIT, devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
#else
	writeb(0, devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
#endif

	// enable interrupts
	devpriv->intr_enable_bits |= EN_ADC_OVERRUN_BIT | EN_ADC_DONE_INTR_BIT;
	if(cmd->stop_src == TRIG_EXT)
		devpriv->intr_enable_bits |= EN_ADC_STOP_INTR_BIT;
	if(cmd->flags & TRIG_WAKE_EOS)
		devpriv->intr_enable_bits |= ADC_INTR_EOSCAN_BITS;
	else
		devpriv->intr_enable_bits |= ADC_INTR_QFULL_BITS;	// for clairity only, since quarter-full bits are zero
	writew(devpriv->intr_enable_bits, devpriv->main_iobase + INTR_ENABLE_REG);
	// enable interrupts on plx 9080
	// XXX enabling more interrupt sources than are actually used
	bits = ICS_PIE | ICS_PLIE | ICS_PAIE | ICS_PDIE | ICS_LIE | ICS_LDIE | ICS_DMA0_E | ICS_DMA1_E | ICS_MBIE;
	writel(bits, devpriv->plx9080_iobase + PLX_INTRCS_REG);

	/* set mode, disable software conversion gate */
	bits = ADC_CONTROL1_DUMMY_BITS | SW_NOGATE_BIT;
	if(cmd->convert_src == TRIG_EXT)
		bits |= ADC_MODE_BITS(13);	// good old mode 13
	else
		bits |= ADC_MODE_BITS(8);	// mode 8.  What else could you need?
	writew(bits, devpriv->main_iobase + ADC_CONTROL1_REG);

	/* enable pacing, triggering, etc */
	bits = ADC_ENABLE_BIT;
	// set start trigger
	if(cmd->start_src == TRIG_EXT)
		bits |= ADC_START_TRIG_EXT_BITS;
	else if(cmd->start_src == TRIG_NOW)
		bits |= ADC_START_TRIG_SOFT_BITS;
	writew(bits, devpriv->main_iobase + ADC_CONTROL0_REG);

	// start aquisition
	writew(0, devpriv->main_iobase + ADC_START_REG);

	return 0;
}

static void handle_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev = d;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	int num_samples = 0;
	unsigned int i;
	u16 data;
	unsigned int status;
	u32 plx_status;
	u32 plx_bits;
	unsigned int dma_status;
#ifdef PCIDAS64_DEBUG
	static unsigned int intr_count = 0;
	const int debug_count = 10;
#endif

	status = readw(devpriv->main_iobase + HW_STATUS_REG);
	plx_status = readl(devpriv->plx9080_iobase + PLX_INTRCS_REG);
	if((status &
		(ADC_INTR_PENDING_BIT | ADC_DONE_BIT | ADC_STOP_BIT |
		DAC_INTR_PENDING_BIT | DAC_DONE_BIT | EXT_INTR_PENDING_BIT)) == 0 &&
		(plx_status & (ICS_DMA0_A | ICS_DMA1_A | ICS_LDIA | ICS_LIA | ICS_PAIA | ICS_PDIA |
		ICS_MBIA(0) | ICS_MBIA(1) |ICS_MBIA(2) | ICS_MBIA(3))) == 0)
	{
#ifdef PCIDAS64_DEBUG
		intr_count++;
		rt_printk(" cb_pcidas64 spurious interrupt");
#endif
		return;
	}
#ifdef PCIDAS64_DEBUG
	intr_count++;
	if(intr_count < debug_count)
	{
		rt_printk(" cb_pcidas64 interrupt status 0x%x\n", status);
		rt_printk(" plx status 0x%x\n", plx_status);
	}
#endif

	async->events = 0;

	// check for fifo overrun
	if(status & ADC_OVERRUN_BIT)
	{
		ai_cancel(dev, s);
		async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
		comedi_error(dev, "fifo overrun");
	}

	// if interrupt was due to analog input data being available
	if(status & ADC_INTR_PENDING_BIT)
	{
		// figure out how many samples we should read from board's fifo
		int read_index, write_index;
		// get most significant bits
		read_index = grey2int(ADC_UPP_READ_PNTR_CODE(readw(devpriv->main_iobase + PREPOST_REG)));
		write_index = grey2int(ADC_UPP_WRITE_PNTR_CODE(readw(devpriv->main_iobase + PREPOST_REG)));
		read_index <<= 15;
		write_index <<= 15;
		// get least significant 15 bits
		read_index += readw(devpriv->main_iobase + ADC_READ_PNTR_REG) & 0x7fff;
		write_index += readw(devpriv->main_iobase + ADC_WRITE_PNTR_REG) & 0x7fff;
		num_samples = write_index - read_index;
		if(num_samples < 0)
			num_samples += 4 * QUARTER_AI_FIFO_SIZE;
		if(cmd->stop_src == TRIG_COUNT)
		{
			if(num_samples > devpriv->ai_count)
				num_samples = devpriv->ai_count;
			devpriv->ai_count -= num_samples;
		}
#ifdef PCIDAS64_DEBUG
		if(intr_count < debug_count)
			rt_printk(" reading %i samples from fifo\n", num_samples);
#endif
		// read samples
		for(i = 0; i < num_samples; i++)
		{
			data = readw(devpriv->main_iobase + ADC_FIFO_REG);
			comedi_buf_put(async, data);
		}
		async->events |= COMEDI_CB_BLOCK | COMEDI_CB_EOS;
	}

	// clear possible plx9080 interrupt sources
	if(plx_status & ICS_LDIA)
	{ // clear local doorbell interrupt
		plx_bits = readl(devpriv->plx9080_iobase + PLX_DBR_OUT_REG);
		writel(plx_bits, devpriv->plx9080_iobase + PLX_DBR_OUT_REG);
#ifdef PCIDAS64_DEBUG
		if(intr_count < debug_count)
			rt_printk(" cleared local doorbell bits 0x%x\n", plx_bits);
#endif
	}
	if(plx_status & ICS_DMA0_A)
	{	// dma chan 0 interrupt
		dma_status = readb(devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
		if(dma_status & PLX_DMA_EN_BIT)
		{
			// transfer data from dma buffer to comedi buffer
			num_samples = DMA_TRANSFER_SIZE / sizeof(devpriv->ai_buffer[0][0]);
			if(cmd->stop_src == TRIG_COUNT)
			{
				if(num_samples > devpriv->ai_count)
					num_samples = devpriv->ai_count;
				devpriv->ai_count -= num_samples;
			}
			for(i = 0; i < num_samples; i++)
			{
				comedi_buf_put(async, devpriv->ai_buffer[devpriv->dma_index][i]);
			}
			devpriv->dma_index = (devpriv->dma_index + 1) % DMA_RING_COUNT;
		}
		writeb(PLX_CLEAR_DMA_INTR_BIT, devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
#ifdef PCIDAS64_DEBUG
		if(intr_count < debug_count)
			rt_printk(" cleared dma ch0 interrupt\n");
#endif
	}
	if(plx_status & ICS_DMA1_A)
	{	// dma chan 1 interrupt
		writeb(PLX_CLEAR_DMA_INTR_BIT, devpriv->plx9080_iobase + PLX_DMA1_CS_REG);
#ifdef PCIDAS64_DEBUG
		if(intr_count < debug_count)
			rt_printk(" cleared dma ch1 interrupt\n");
#endif
	}

	// if we are have all the data, then quit
	if(cmd->stop_src == TRIG_COUNT ||
		(cmd->stop_src == TRIG_EXT && (status & ADC_STOP_BIT)))
	{
		if(devpriv->ai_count <= 0)
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

	/* disable pacing, triggering, etc */
	writew(0, devpriv->main_iobase + ADC_CONTROL0_REG);
	writew(ADC_CONTROL1_DUMMY_BITS, devpriv->main_iobase + ADC_CONTROL1_REG);

	// abort dma transfer if necessary
	dma_status = readb(devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
	if(dma_status & PLX_DMA_EN_BIT)
	{
		// wait to make sure done bit is zero
		for(i = 0; (dma_status & PLX_DMA_DONE_BIT) && i < timeout; i++)
		{
			udelay(1);
			dma_status = readb(devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
		}
		if(i == timeout)
			comedi_error(dev, "cancel() timed out waiting for dma done clear");
		// disable channel
		writeb(0, devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
		// abort channel
		writeb(PLX_DMA_ABORT_BIT, devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
		// wait for dma done bit
		dma_status = readb(devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
		for(i = 0; (dma_status & PLX_DMA_DONE_BIT) == 0 && i < timeout; i++)
		{
			udelay(1);
			dma_status = readb(devpriv->plx9080_iobase + PLX_DMA0_CS_REG);
		}
		if(i == timeout)
			comedi_error(dev, "cancel() timed out waiting for dma done set");
	}

	return 0;
}

static int ao_winsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int chan = CR_CHAN(insn->chanspec);
	int range = CR_RANGE(insn->chanspec);
	unsigned int bits;

	// do some initializing
	writew(DAC_ENABLE_BIT, devpriv->main_iobase + DAC_CONTROL0_REG);

	// set range
	bits = DAC_OUTPUT_ENABLE_BIT;
	bits |= DAC_RANGE_BITS(chan, range);
	writew(bits, devpriv->main_iobase + DAC_CONTROL1_REG);

	// clear buffer
	writew(0, devpriv->main_iobase + DAC_BUFFER_CLEAR_REG);

	// write to channel
	writew(data[0], devpriv->main_iobase + DAC_CONVERT_REG(chan));

	// remember output value
	devpriv->ao_value[chan] = data[0];

	return 1;
}

static int ao_readback_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	data[0] = devpriv->ao_value[CR_CHAN(insn->chanspec)];

	return 1;
}

static int dio_callback(int dir, int port, int data, void *arg)
{
	unsigned long iobase = (int)arg;

	if(dir)
	{
		writeb(data, iobase + port);
		return 0;
	}else
	{
		return readb(iobase + port);
	}
}

static int di_rbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	lsampl_t bits;

	bits = readb(devpriv->dio_counter_iobase + DI_REG);
	bits &= 0xf;
	data[1] = bits;
	data[0] = 0;

	return 2;
}

static int do_wbits(comedi_device *dev, comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	lsampl_t wbits;

	data[0] &= 0xf;
	wbits = devpriv->do_bits;
	// zero bits we are going to change
	wbits &= ~data[0];
	// set new bits
	wbits |= data[0] & data[1];
	devpriv->do_bits = wbits;

	writeb(devpriv->do_bits, devpriv->dio_counter_iobase + DO_REG);

	data[1] = wbits;

	return 2;
}

/* utility function that rounds desired timing to an achievable time, and
 * sets cmd members appropriately.
 * adc paces conversions from master clock by dividing by (x + 3) where x is 24 bit number
 */
static void check_adc_timing(comedi_cmd *cmd)
{
	unsigned int convert_divisor, scan_divisor;
	const int max_counter_value = 0xffffff;	// board uses 24 bit counters for pacing
	const int min_convert_divisor = 3;
	const int max_convert_divisor = max_counter_value + min_convert_divisor;
	unsigned long long max_scan_divisor, min_scan_divisor;

	if(cmd->convert_src == TRIG_TIMER)
	{
		convert_divisor = get_divisor(cmd->convert_arg, cmd->flags);
		if(convert_divisor > max_convert_divisor) convert_divisor = max_convert_divisor;
		if(convert_divisor < min_convert_divisor) convert_divisor = min_convert_divisor;
		cmd->convert_arg = convert_divisor * TIMER_BASE;

		if(cmd->scan_begin_src == TRIG_TIMER)
		{
			scan_divisor = get_divisor(cmd->scan_begin_arg, cmd->flags);
			// XXX check for integer overflows
			min_scan_divisor = convert_divisor * cmd->chanlist_len;
			max_scan_divisor = (convert_divisor * cmd->chanlist_len - 1) + max_counter_value;
			if(scan_divisor > max_scan_divisor) scan_divisor = max_scan_divisor;
			if(scan_divisor < min_scan_divisor) scan_divisor = min_scan_divisor;
			cmd->scan_begin_arg = scan_divisor * TIMER_BASE;
		}
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

// decodes encoding of 2 most significant bits of read/write pointer addresses coming from board
static int grey2int(int code)
{
	switch(code)
	{
		case 0:
			return 0;
			break;
		case 1:
			return 1;
			break;
		case 2:
			return 3;
			break;
		case 3:
			return 2;
			break;
		default:
			break;
	}

	return -1;
}
