/*
    cb_pcidas.c
    This is a driver for the ComputerBoards/MeasurementComputing PCI-DAS
    cards using the AMCC S5933 PCI controller:
    - PCI-DAS1602/12, 1602/16, 1602/16/jr
    - PCI-DAS1200, 1200jr
    - PCI-DAS1000, 1001, 1002


    SO FAR IT WAS ONLY TESTED WITH PCI-DAS1200. PLEASE REPORT IF YOU ARE
    USING IT WITH A DIFFERENT CARD <ivanmr@altavista.com>.

    Options:
    [0] - PCI bus number
    [1] - PCI slot number

    Developed by Ivan Martinez and Frank Mori Hess, with valuable help from
    David Schleef and the rest of the Comedi developers comunity.

    Copyright (C) 2001 Ivan Martinez <ivanmr@altavista.com>
    Copyright (C) 2001 Frank Mori Hess <fmhess@uiuc.edu>

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

************************************************************************
*/
/*
Driver: cb_pcidas.o
Description: Driver for the ComputerBoards/MeasurementComputing cards
  of the PCI-DAS series with the AMCC S5933 PCI controller.
Author: Ivan Martinez <ivanmr@altavista.com>,
  Frank Mori Hess <fmhess@uiuc.edu>
Status:
  - PCI-DAS1602/16, 16jr: Driver should work, but untested.  Please
      report usage.
  - PCI-DAS1602/12: Same as above.
  - PCI-DAS1200, 1200jr: Tested, works.
  - PCI-DAS1000, 1001, 1002: Should work, but untested.  Please report
    usage.
Updated: 2001-8-27
Devices: [Measurement Computing] PCI-DAS1602/16 (cb_pcidas),
  PCI-DAS1602/16jr, PCI-DAS1602/12, PCI-DAS1200, PCI-DAS1200jr,
  PCI-DAS1000, PCI-DAS1001, PCI_DAS1002

This driver originally intended to support the whole PCI-DAS series,
but cards based on the PLX PCI controller are excluded for being
too different.

Configuration options:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
  If bus/slot is not specified, the first available PCI
  device will be used.

For commands, the scanned channels must be consecutive
(i.e. 4-5-6-7, 2-3-4,...), and must all have the same
range and aref.
*/
/*

TODO:

add a calibration subdevice

analog triggering on 1602 series
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

#define CB_PCIDAS_DEBUG	// enable debugging code
//#undef CB_PCIDAS_DEBUG	// disable debugging code

// PCI vendor number of ComputerBoards/MeasurementComputing
#define PCI_VENDOR_ID_CB	0x1307
#define TIMER_BASE 100	// 10MHz master clock
static const int max_fifo_size = 1024;	// maximum fifo size of any supported board

/* PCI-DAS base addresses */

// indices of base address regions
#define S5933_BADRINDEX 0
#define CONT_STAT_BADRINDEX 1
#define ADC_FIFO_BADRINDEX 2
#define PACER_BADRINDEX 3
#define AO_BADRINDEX 4
// sizes of io regions
#define S5933_SIZE 64
#define CONT_STAT_SIZE 10
#define ADC_FIFO_SIZE 4
#define PACER_SIZE 12
#define AO_SIZE 4

// amcc s5933 pci configuration registers
#define INTCSR	0x38	// interrupt control/status
#define   OUTBOX_BYTE(x)	((x) & 0x3)
#define   OUTBOX_SELECT(x)	(((x) & 0x3) << 2)
#define   OUTBOX_EMPTY_INT	0x10	// enable outbox empty interrupt
#define   INBOX_BYTE(x)	(((x) & 0x3) << 8)
#define   INBOX_SELECT(x)	(((x) & 0x3) << 10)
#define   INBOX_FULL_INT	0x1000	// enable inbox full interrupt
#define   INBOX_INTR_STATUS	0x20000 // read, or write clear inbox full interrupt

/* Control/Status registers */
#define INT_ADCFIFO	0	// INTERRUPT / ADC FIFO register
#define   INT_EOS 0x1	// interrupt end of scan
#define   INT_FHF 0x2	// interrupt fifo half full
#define   INT_FNE 0x3	// interrupt fifo not empty
#define   INT_MASK 0x3	// mask of interrupt select bits
#define   INTE 0x4	// interrupt enable
#define   DAHFIE 0x8	// dac half full interrupt enable
#define   EOAIE	0x10	// end of aquisition interrupt enable
#define   DAHFI	0x20	// dac half full read status / write interrupt clear
#define   EOAI 0x40	// read end of acq. interrupt status / write clear
#define   INT 0x80	// read interrupt status / write clear
#define   EOBI 0x200	// read end of burst interrupt status
#define   ADHFI 0x400	// read half-full interrupt status
#define   ADNEI 0x800	// read fifo not empty interrupt latch status
#define   ADNE 0x1000	// read, fifo not empty (realtime, not latched) status
#define   DAEMIE	0x1000	// write, dac empty interrupt enable
#define   LADFUL 0x2000	// read fifo overflow / write clear
#define   DAEMI 0x4000	// dac fifo empty interrupt status / write clear

#define ADCMUX_CONT	2	// ADC CHANNEL MUX AND CONTROL register
#define   BEGIN_SCAN(x)	((x) & 0xf)
#define   END_SCAN(x)	(((x) & 0xf) << 4)
#define   GAIN_BITS(x)	(((x) & 0x3) << 8)
#define   UNIP	0004000	// Analog front-end unipolar for range
#define   SE	0002000	// Inputs in single-ended mode
#define   PACER_MASK	0x3000	// pacer source bits
#define   PACER_INT 0x1000	// internal pacer
#define   PACER_EXT_FALL	0x2000	// external falling edge
#define   PACER_EXT_RISE	0x3000	// external rising edge
#define   EOC	0x4000	// adc not busy

#define TRIG_CONTSTAT 4 // TRIGGER CONTROL/STATUS register
#define   SW_TRIGGER 0x1	// software start trigger
#define   EXT_TRIGGER 0x2	// external start trigger
#define   TGEN	0x10	// enable external start trigger
#define   BURSTE 0x20	// burst mode enable
#define   XTRCL	0x80	// clear external trigger

#define CALIBRATION	6	// CALIBRATION register

#define DAC_CSR	0x8	// dac control and status register
#define   DACEN	0x2	// dac enable
#define   DAC_RANGE(channel, range)	(((range) & 0x3) << (8 + 2 * channel) )	// dac range
// bits for 1602 series only
#define   DAC_EMPTY	0x1	// dac fifo empty, read, write clear
#define   DAC_START	0x4	// start/arm dac fifo operations
#define   DAC_PACER_MASK	0x18	// bits that set dac pacer source
#define   DAC_PACER_INT	0x8	// dac internal pacing
#define   DAC_PACER_EXT_FALL	0x10	// dac external pacing, falling edge
#define   DAC_PACER_EXT_RISE	0x18	// dac external pacing, rising edge
#define   DAC_CHAN_EN(x)		(1 << (5 + ((x) & 0x1)))	// enable channel 0 or 1

/* analog input fifo */
#define ADCDATA	0	// ADC DATA register
#define ADCFIFOCLR	2	// ADC FIFO CLEAR

// pacer, counter, dio registers
#define ADC8254 0
#define DIO_8255 4
#define DAC8254 8

// analog output registers for 100x, 1200 series
#define DAC_DATA_REG(channel)	((channel) & 0x1)
/* analog output registers for 1602 series*/
#define DACDATA	0	// DAC DATA register
#define DACFIFOCLR	2	// DAC FIFO CLEAR

// bit in hexadecimal representation of range index that indicates unipolar input range
#define IS_UNIPOLAR 0x4
// analog input ranges for most boards
static comedi_lrange cb_pcidas_ranges =
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

// pci-das1001 input ranges
static comedi_lrange cb_pcidas_alt_ranges =
{
	8,
	{
		BIP_RANGE(10),
		BIP_RANGE(1),
		BIP_RANGE(0.1),
		BIP_RANGE(0.01),
		UNI_RANGE(10),
		UNI_RANGE(1),
		UNI_RANGE(0.1),
		UNI_RANGE(0.01)
	}
};

// analog output ranges
static comedi_lrange cb_pcidas_ao_ranges =
{
	4,
	{
		BIP_RANGE(5),
		BIP_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(10),
	}
};

typedef struct cb_pcidas_board_struct
{
	char *name;
	unsigned short device_id;
	int ai_se_chans;	// Inputs in single-ended mode
	int ai_diff_chans;	// Inputs in differential mode
	int ai_bits;	// analog input resolution
	int ai_speed;	// fastest conversion period in ns
	int ao_nchan;	// number of analog out channels
	int has_ao_fifo;	// analog output has fifo
	int ao_scan_speed;	// analog output speed for 1602 series (for a scan, not conversion)
	int fifo_size;	// number of samples fifo can hold
	comedi_lrange *ranges;
} cb_pcidas_board;

static cb_pcidas_board cb_pcidas_boards[] =
{
	{
		name:		"pci-das1602/16",
		device_id:	0x1,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	16,
		ai_speed:	5000,
		ao_nchan: 2,
		has_ao_fifo:	1,
		ao_scan_speed:	10000,
		fifo_size:	512,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1200",
		device_id:	0xF,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	3200,
		ao_nchan: 2,
		has_ao_fifo:	0,
		fifo_size:	1024,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1602/12",
		device_id:	0x10,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	3200,
		ao_nchan: 2,
		has_ao_fifo:	1,
		ao_scan_speed:	4000,
		fifo_size:	1024,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1200/jr",
		device_id:	0x19,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	3200,
		ao_nchan: 0,
		has_ao_fifo:	0,
		fifo_size:	1024,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1602/16/jr",
		device_id:	0x1C,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	16,
		ai_speed: 5000,
		ao_nchan: 0,
		has_ao_fifo:	0,
		fifo_size:	512,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1000",
		device_id:	0x4C,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	4000,
		ao_nchan: 0,
		has_ao_fifo:	0,
		fifo_size:	1024,
		ranges:	&cb_pcidas_ranges,
	},
	{
		name:		"pci-das1001",
		device_id:	0x1a,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	6800,
		ao_nchan: 2,
		has_ao_fifo:	0,
		fifo_size:	1024,
		ranges:	&cb_pcidas_alt_ranges,
	},
	{
		name:		"pci-das1002",
		device_id:	0x1b,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	12,
		ai_speed:	6800,
		ao_nchan: 2,
		has_ao_fifo:	0,
		fifo_size:	1024,
		ranges:	&cb_pcidas_ranges,
	},
};
// Number of boards in cb_pcidas_boards
#define N_BOARDS	(sizeof(cb_pcidas_boards) / sizeof(cb_pcidas_board))

static struct pci_device_id cb_pcidas_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_CB, 0x0001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x000f, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x0010, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x0019, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x001c, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x004c, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x001a, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_CB, 0x001b, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, cb_pcidas_pci_table);

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((cb_pcidas_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct
{
	/* would be useful for a PCI device */
	struct pci_dev *pci_dev;
	// base addresses
	unsigned int s5933_config;
	unsigned int control_status;
	unsigned int adc_fifo;
	unsigned int pacer_counter_dio;
	unsigned int ao_registers;
	// divisors of master clock for analog input pacing
	unsigned int divisor1;
	unsigned int divisor2;
	volatile unsigned int count;	// number of analog input samples remaining
	unsigned int adc_fifo_bits;	// bits to write to interupt/adcfifo register
	unsigned int s5933_intcsr_bits;	// bits to write to amcc s5933 interrupt control/status register
	unsigned int ao_control_bits;	// bits to write to ao control and status register
	// divisors of master clock for analog output pacing
	unsigned int ao_divisor1;
	unsigned int ao_divisor2;
	volatile unsigned int ao_count;	// number of analog output samples remaining
	int ao_value[2];	// remember what the analog outputs are set to, to allow readback
} cb_pcidas_private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((cb_pcidas_private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int cb_pcidas_attach(comedi_device *dev,comedi_devconfig *it);
static int cb_pcidas_detach(comedi_device *dev);
comedi_driver driver_cb_pcidas={
	driver_name:	"cb_pcidas",
	module:		THIS_MODULE,
	attach:		cb_pcidas_attach,
	detach:		cb_pcidas_detach,
};

static int cb_pcidas_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int cb_pcidas_ao_nofifo_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int cb_pcidas_ao_fifo_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int cb_pcidas_ao_readback_insn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int cb_pcidas_ai_cmd(comedi_device *dev,comedi_subdevice *s);
static int cb_pcidas_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);
static int cb_pcidas_ao_cmd(comedi_device *dev,comedi_subdevice *s);
static int cb_pcidas_ao_inttrig(comedi_device *dev, comedi_subdevice *subdev, unsigned int trig_num);
static int cb_pcidas_ao_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);
static void cb_pcidas_interrupt(int irq, void *d, struct pt_regs *regs);
static void handle_ao_interrupt(comedi_device *dev, unsigned int status);
static int cb_pcidas_cancel(comedi_device *dev, comedi_subdevice *s);
static int cb_pcidas_ao_cancel(comedi_device *dev, comedi_subdevice *s);
static void cb_pcidas_load_counters(comedi_device *dev, unsigned int *ns, int round_flags);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.
 */
static int cb_pcidas_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
	struct pci_dev* pcidev;
	int index;
	unsigned long s5933_config, control_status, adc_fifo,
		pacer_counter_dio, ao_registers;
	int err;

	printk("comedi%d: cb_pcidas: ",dev->minor);

/*
 * Allocate the private structure area.
 */
	if(alloc_private(dev,sizeof(cb_pcidas_private))<0)
		return -ENOMEM;

/*
 * Probe the device to determine what device in the series it is.
 */
	printk("\n");

	pci_for_each_dev(pcidev)
	{
		// is it not a computer boards card?
		if(pcidev->vendor != PCI_VENDOR_ID_CB)
			continue;
		// loop through cards supported by this driver
		for(index = 0; index < N_BOARDS; index++)
		{
			if(cb_pcidas_boards[index].device_id != pcidev->device)
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
			devpriv->pci_dev = pcidev;
			dev->board_ptr = cb_pcidas_boards + index;
			goto found;
		}
	}

	printk("No supported ComputerBoards/MeasurementComputing card found on "
		"requested position\n");
	return -EIO;

found:

	printk("Found %s on bus %i, slot %i\n", cb_pcidas_boards[index].name,
		devpriv->pci_dev->bus->number, PCI_SLOT(devpriv->pci_dev->devfn));

	// Warn about non-tested features
	switch(thisboard->device_id)
	{
		case 0x1:
		case 0x10:
		case 0x1C:
		case 0x4C:
		case 0x1A:
		case 0x1B:
			printk("DRIVER HASN'T BEEN TESTED WITH THIS CARD. PLEASE REPORT "
				"USAGE TO <ivanmr@altavista.com>\n");
	};


	/*
	 * Initialize devpriv->control_status and devpriv->adc_fifo to point to
	 * their base address.
	 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
	s5933_config =
		devpriv->pci_dev->base_address[S5933_BADRINDEX] &
		PCI_BASE_ADDRESS_IO_MASK;
	control_status =
		devpriv->pci_dev->base_address[CONT_STAT_BADRINDEX] &
		PCI_BASE_ADDRESS_IO_MASK;
	adc_fifo =
		devpriv->pci_dev->base_address[ADC_FIFO_BADRINDEX] &
		PCI_BASE_ADDRESS_IO_MASK;
	pacer_counter_dio =
		devpriv->pci_dev->base_address[PACER_BADRINDEX] &
		PCI_BASE_ADDRESS_IO_MASK;
	ao_registers =
		devpriv->pci_dev->base_address[AO_BADRINDEX] &
		PCI_BASE_ADDRESS_IO_MASK;
#else
	if(pci_enable_device(devpriv->pci_dev))
		return -EIO;
	s5933_config =
		devpriv->pci_dev->resource[S5933_BADRINDEX].start &
		PCI_BASE_ADDRESS_IO_MASK;
	control_status =
		devpriv->pci_dev->resource[CONT_STAT_BADRINDEX].start &
		PCI_BASE_ADDRESS_IO_MASK;
	adc_fifo =
		devpriv->pci_dev->resource[ADC_FIFO_BADRINDEX].start &
		PCI_BASE_ADDRESS_IO_MASK;
	pacer_counter_dio =
		devpriv->pci_dev->resource[PACER_BADRINDEX].start &
		PCI_BASE_ADDRESS_IO_MASK;
	ao_registers =
		devpriv->pci_dev->resource[AO_BADRINDEX].start &
		PCI_BASE_ADDRESS_IO_MASK;
#endif

	// reserve io ports
	err = 0;
	if(check_region(s5933_config, S5933_SIZE) < 0)
		err++;
	if(check_region(control_status, CONT_STAT_SIZE) < 0)
		err++;
	if(check_region(adc_fifo, ADC_FIFO_SIZE) < 0)
		err++;
	if(check_region(pacer_counter_dio, PACER_SIZE) < 0)
		err++;
	if(thisboard->ao_nchan)
		if(check_region(ao_registers, AO_SIZE) < 0)
			err++;
	if(err)
	{
		printk(" I/O port conflict\n");
		return -EIO;
	}
	request_region(s5933_config, S5933_SIZE, "cb_pcidas");
	devpriv->s5933_config = s5933_config;
	request_region(control_status, CONT_STAT_SIZE, "cb_pcidas");
	devpriv->control_status = control_status;
	request_region(adc_fifo, ADC_FIFO_SIZE, "cb_pcidas");
	devpriv->adc_fifo = adc_fifo;
	request_region(pacer_counter_dio, PACER_SIZE, "cb_pcidas");
	devpriv->pacer_counter_dio = pacer_counter_dio;
	if(thisboard->ao_nchan)
	{
		request_region(ao_registers, AO_SIZE, "cb_pcidas");
		devpriv->ao_registers = ao_registers;
	}

	// get irq
	if(comedi_request_irq(devpriv->pci_dev->irq, cb_pcidas_interrupt, SA_SHIRQ, "cb_pcidas", dev ))
	{
		printk(" unable to allocate irq %d\n", devpriv->pci_dev->irq);
		return -EINVAL;
	}
	dev->irq = devpriv->pci_dev->irq;

	//Initialize dev->board_name
	dev->board_name = thisboard->name;

/*
 * Allocate the subdevice structures.
 */
	dev->n_subdevices = 3;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog input subdevice */
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_COMMON | SDF_DIFF;
	/* WARNING: Number of inputs in differential mode is ignored */
	s->n_chan = thisboard->ai_se_chans;
	s->len_chanlist = thisboard->ai_se_chans;
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = thisboard->ranges;
	s->insn_read = cb_pcidas_ai_rinsn;
	s->do_cmd = cb_pcidas_ai_cmd;
	s->do_cmdtest = cb_pcidas_ai_cmdtest;
	s->cancel = cb_pcidas_cancel;

	/* analog output subdevice */
	s = dev->subdevices + 1;
	if(thisboard->ao_nchan)
	{
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_READABLE | SDF_WRITEABLE | SDF_GROUND;
		s->n_chan = thisboard->ao_nchan;
		// analog out resolution is the same as analog input resolution, so use ai_bits
		s->maxdata = (1 << thisboard->ai_bits) - 1;
		s->range_table = &cb_pcidas_ao_ranges;
		s->insn_read = cb_pcidas_ao_readback_insn;
		if(thisboard->has_ao_fifo)
		{
			dev->write_subdev = s;
			s->insn_write = cb_pcidas_ao_fifo_winsn;
			s->do_cmdtest = cb_pcidas_ao_cmdtest;
			s->do_cmd = cb_pcidas_ao_cmd;
			s->len_chanlist = thisboard->ao_nchan;
			s->cancel = cb_pcidas_ao_cancel;
		}else
		{
			s->insn_write = cb_pcidas_ao_nofifo_winsn;
		}
	}else
	{
		s->type = COMEDI_SUBD_UNUSED;
	}

	/* 8255 */
	s = dev->subdevices + 2;
	subdev_8255_init(dev, s, NULL,
		(void *)(devpriv->pacer_counter_dio + DIO_8255));

	/* Set bits to enable incoming mailbox interrupts on amcc s5933.
	 * They don't actually get sent here, but in cmd code. */
	devpriv->s5933_intcsr_bits = INBOX_BYTE(3) | INBOX_SELECT(3) | INBOX_FULL_INT;

	return 1;
}


/*
 * _detach is called to deconfigure a device.  It should deallocate
 * resources.
 * This function is also called when _attach() fails, so it should be
 * careful not to release resources that were not necessarily
 * allocated by _attach().  dev->private and dev->subdevices are
 * deallocated automatically by the core.
 */
static int cb_pcidas_detach(comedi_device *dev)
{
	printk("comedi%d: cb_pcidas: remove\n",dev->minor);

	if(devpriv)
	{
		if(devpriv->s5933_config)
		{
			// disable and clear interrupts on amcc s5933
			outl(INBOX_INTR_STATUS, devpriv->s5933_config + INTCSR);
			rt_printk("detaching, incsr is 0x%x\n", inl(devpriv->s5933_config + INTCSR));
			release_region(devpriv->s5933_config, S5933_SIZE);
		}
		if(devpriv->control_status)
			release_region(devpriv->control_status, CONT_STAT_SIZE);
		if(devpriv->adc_fifo)
			release_region(devpriv->adc_fifo, ADC_FIFO_SIZE);
		if(devpriv->pacer_counter_dio)
			release_region(devpriv->pacer_counter_dio, PACER_SIZE);
		if(devpriv->ao_registers)
			release_region(devpriv->ao_registers, AO_SIZE);
	}
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);
	if(dev->subdevices)
		subdev_8255_cleanup(dev,dev->subdevices + 2);

	return 0;
}

/*
 * "instructions" read/write data in "one-shot" or "software-triggered"
 * mode.
 */
static int cb_pcidas_ai_rinsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int n,i;
	unsigned int bits;
	static const int timeout = 10000;

	// set mux limits and gain
	bits = BEGIN_SCAN(CR_CHAN(insn->chanspec)) |
		END_SCAN(CR_CHAN(insn->chanspec)) |
		GAIN_BITS(CR_RANGE(insn->chanspec));
	// set unipolar/bipolar
	if(CR_RANGE(insn->chanspec) & IS_UNIPOLAR)
		bits |= UNIP;
	// set singleended/differential
	if(CR_AREF(insn->chanspec) != AREF_DIFF)
		bits |= SE;
	outw_p(bits, devpriv->control_status + ADCMUX_CONT);

	/* wait for mux to settle */
	/* I suppose I made it with outw_p... */

	/* clear fifo */
	outw(0, devpriv->adc_fifo + ADCFIFOCLR);

	/* convert n samples */
	for (n = 0; n < insn->n; n++)
	{
		/* trigger conversion */
		outw(0, devpriv->adc_fifo + ADCDATA);

		/* wait for conversion to end */
		/* return -ETIMEDOUT if there is a timeout */
		for(i = 0; i < timeout; i++)
		{
			if (inw(devpriv->control_status + ADCMUX_CONT) & EOC)
				break;
		}
		if(i == timeout)
			return -ETIMEDOUT;

		/* read data */
		data[n] = inw(devpriv->adc_fifo + ADCDATA);
	}

	/* return the number of samples read/written */
	return n;
}

// analog output insn for pcidas-1000 and 1200 series
static int cb_pcidas_ao_nofifo_winsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int bits, channel;

	// set channel and range
	channel = CR_CHAN(insn->chanspec);
	bits = DACEN;
	bits |= DAC_RANGE(channel, CR_RANGE(insn->chanspec));
	outw(bits, devpriv->control_status + DAC_CSR);

	// remember value for readback
	devpriv->ao_value[channel] = data[0];
	// send data
	outw(data[0], devpriv->ao_registers + DAC_DATA_REG(channel));

	return 1;
}

// analog output insn for pcidas-1602 series
static int cb_pcidas_ao_fifo_winsn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	int bits, channel;

	// clear dac fifo
	outw(0, devpriv->ao_registers + DACFIFOCLR);

	// set channel and range
	channel = CR_CHAN(insn->chanspec);
	bits = DACEN;
	bits |= DAC_RANGE(channel, CR_RANGE(insn->chanspec));
	bits |= DAC_CHAN_EN(channel);
	bits |= DAC_START;	// not sure if this is necessary
	outw(bits, devpriv->control_status + DAC_CSR);

	// remember value for readback
	devpriv->ao_value[channel] = data[0];
	// send data
	outw(data[0], devpriv->ao_registers + DACDATA);

	return 1;
}

// analog output readback insn
// XXX loses track of analog output value back after an analog ouput command is executed
static int cb_pcidas_ao_readback_insn(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	data[0] = devpriv->ao_value[CR_CHAN(insn->chanspec)];

	return 1;
}

static int cb_pcidas_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd)
{
	int err=0;
	int tmp;
	int i, gain, start_chan;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_FOLLOW | TRIG_TIMER | TRIG_EXT;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_NOW | TRIG_EXT;
	if(!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if(!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->start_src != TRIG_NOW &&
		cmd->start_src != TRIG_EXT)
		err++;
	if(cmd->scan_begin_src != TRIG_FOLLOW &&
		cmd->scan_begin_src != TRIG_TIMER &&
		cmd->scan_begin_src != TRIG_EXT)
		err++;
	if(cmd->convert_src != TRIG_TIMER &&
		cmd->convert_src != TRIG_EXT &&
		cmd->convert_src != TRIG_NOW)
		err++;
	if(cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE)
		err++;

	// make sure trigger sources are compatible with each other
	if(cmd->scan_begin_src == TRIG_FOLLOW &&
		cmd->convert_src == TRIG_NOW)
		err++;
	if(cmd->scan_begin_src != TRIG_FOLLOW &&
		cmd->convert_src != TRIG_NOW)
		err++;
	if(cmd->start_src == TRIG_EXT &&
		(cmd->convert_src == TRIG_EXT || cmd->scan_begin_src == TRIG_EXT))
		err++;

	if(err) return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0)
	{
		cmd->start_arg=0;
		err++;
	}

	if (cmd->scan_begin_src == TRIG_TIMER)
	{
		if (cmd->scan_begin_arg < thisboard->ai_speed * cmd->chanlist_len)
		{
			cmd->scan_begin_arg = thisboard->ai_speed * cmd->chanlist_len;
			err++;
		}
	}
	if (cmd->convert_src == TRIG_TIMER)
	{
		if (cmd->convert_arg < thisboard->ai_speed)
		{
			cmd->convert_arg = thisboard->ai_speed;
			err++;
		}
	}

	if(cmd->scan_end_arg != cmd->chanlist_len)
	{
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src == TRIG_NONE)
	{
		/* TRIG_NONE */
		if (cmd->stop_arg != 0)
		{
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	if(cmd->scan_begin_src == TRIG_TIMER)
	{
		tmp = cmd->scan_begin_arg;
		i8253_cascade_ns_to_timer_2div(TIMER_BASE,
			&(devpriv->divisor1), &(devpriv->divisor2),
			&(cmd->scan_begin_arg), cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->scan_begin_arg)
			err++;
	}
	if(cmd->convert_src == TRIG_TIMER)
	{
		tmp=cmd->convert_arg;
		i8253_cascade_ns_to_timer_2div(TIMER_BASE,
			&(devpriv->divisor1), &(devpriv->divisor2),
			&(cmd->convert_arg), cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->convert_arg)
			err++;
	}

	if(err) return 4;

	// check channel/gain list against card's limitations
	if(cmd->chanlist)
	{
		gain = CR_RANGE(cmd->chanlist[0]);
		start_chan = CR_CHAN(cmd->chanlist[0]);
		for(i = 1; i < cmd->chanlist_len; i++)
		{
			if(CR_CHAN(cmd->chanlist[i]) != (start_chan + i) % s->n_chan)
			{
				comedi_error(dev, "entries in chanlist must be consecutive channels, counting upwards\n");
				err++;
			}
			if(CR_RANGE(cmd->chanlist[i]) != gain)
			{
				comedi_error(dev, "entries in chanlist must all have the same gain\n");
				err++;
			}
		}
	}

	if(err) return 5;

	return 0;
}

static int cb_pcidas_ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned int bits;

	// initialize before settings pacer source and count values
	outw(0, devpriv->control_status + TRIG_CONTSTAT);
	// clear fifo
	outw(0, devpriv->adc_fifo + ADCFIFOCLR);

	// set mux limits, gain and pacer source
	bits = BEGIN_SCAN(CR_CHAN(cmd->chanlist[0])) |
		END_SCAN(CR_CHAN(cmd->chanlist[cmd->chanlist_len - 1])) |
		GAIN_BITS(CR_RANGE(cmd->chanlist[0]));
	// set unipolar/bipolar
	if(CR_RANGE(cmd->chanlist[0]) & IS_UNIPOLAR)
		bits |= UNIP;
	// set singleended/differential
	if(CR_AREF(cmd->chanlist[0]) != AREF_DIFF)
		bits |= SE;
	// set pacer source
	if(cmd->convert_src == TRIG_EXT || cmd->scan_begin_src == TRIG_EXT)
		bits |= PACER_EXT_RISE;
	else
		bits |= PACER_INT;
	outw(bits, devpriv->control_status + ADCMUX_CONT);

#ifdef CB_PCIDAS_DEBUG
		rt_printk("comedi: sent 0x%x to adcmux control\n", bits);
#endif

	// load counters
	if(cmd->convert_src == TRIG_TIMER)
		cb_pcidas_load_counters(dev, &cmd->convert_arg, cmd->flags & TRIG_ROUND_MASK);
	else if(cmd->scan_begin_src == TRIG_TIMER)
		cb_pcidas_load_counters(dev, &cmd->scan_begin_arg, cmd->flags & TRIG_ROUND_MASK);

	// set number of conversions
	if(cmd->stop_src == TRIG_COUNT)
	{
		devpriv->count = cmd->chanlist_len * cmd->stop_arg;
	}

	// enable interrupts
	devpriv->adc_fifo_bits |= INTE;
	devpriv->adc_fifo_bits &= ~INT_MASK;
	if(cmd->flags & TRIG_WAKE_EOS)
	{
		if(cmd->convert_src == TRIG_NOW && cmd->chanlist_len > 1)
			devpriv->adc_fifo_bits |= INT_EOS;	// interrupt end of burst
		else
			devpriv->adc_fifo_bits |= INT_FNE;	// interrupt fifo not empty
	}else
	{
		devpriv->adc_fifo_bits |= INT_FHF;	//interrupt fifo half full
	}
#ifdef CB_PCIDAS_DEBUG
		rt_printk("comedi: adc_fifo_bits are 0x%x\n", devpriv->adc_fifo_bits);
#endif
	// enable (and clear) interrupts
	outw(devpriv->adc_fifo_bits | EOAI | INT | LADFUL, devpriv->control_status + INT_ADCFIFO);
	// enable s5933 interrupt
	outl(devpriv->s5933_intcsr_bits, devpriv->s5933_config + INTCSR);


	// set start trigger and burst mode
	bits = 0;
	if(cmd->start_src == TRIG_NOW)
		bits |= SW_TRIGGER;
	else if(cmd->start_src == TRIG_EXT)
		bits |= EXT_TRIGGER | TGEN | XTRCL;
	else
	{
		comedi_error(dev, "bug!");
		return -1;
	}
	if(cmd->convert_src == TRIG_NOW && cmd->chanlist_len > 1)
		bits |= BURSTE;
	outw(bits, devpriv->control_status + TRIG_CONTSTAT);
#ifdef CB_PCIDAS_DEBUG
		rt_printk("comedi: sent 0x%x to trig control\n", bits);
#endif

	return 0;
}

static int cb_pcidas_ao_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd)
{
	int err=0;
	int tmp;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_INT;
	if(!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER | TRIG_EXT;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_NOW;
	if(!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if(!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->scan_begin_src != TRIG_TIMER &&
		cmd->scan_begin_src != TRIG_EXT)
		err++;
	if(cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE)
		err++;

	if(err) return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0)
	{
		cmd->start_arg=0;
		err++;
	}

	if (cmd->scan_begin_src == TRIG_TIMER)
	{
		if (cmd->scan_begin_arg < thisboard->ao_scan_speed)
		{
			cmd->scan_begin_arg = thisboard->ao_scan_speed;
			err++;
		}
	}

	if(cmd->scan_end_arg != cmd->chanlist_len)
	{
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src == TRIG_NONE)
	{
		/* TRIG_NONE */
		if (cmd->stop_arg != 0)
		{
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	if(cmd->scan_begin_src == TRIG_TIMER)
	{
		tmp = cmd->scan_begin_arg;
		i8253_cascade_ns_to_timer_2div(TIMER_BASE,
			&(devpriv->ao_divisor1), &(devpriv->ao_divisor2),
			&(cmd->scan_begin_arg), cmd->flags & TRIG_ROUND_MASK);
		if(tmp != cmd->scan_begin_arg)
			err++;
	}

	if(err) return 4;

	// check channel/gain list against card's limitations
	if(cmd->chanlist &&
		cmd->chanlist_len > 1)
	{
		if(CR_CHAN(cmd->chanlist[0]) != 0 ||
			CR_CHAN(cmd->chanlist[1]) != 1)
		{
			comedi_error(dev, "channels must be ordered channel 0, channel 1 in chanlist\n");
			err++;
		}
	}

	if(err) return 5;

	return 0;
}

static int cb_pcidas_ao_cmd(comedi_device *dev,comedi_subdevice *s)
{
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned int ao_control_bits = 0;
	unsigned int i;

	// set channel limits, gain
	for(i = 0; i < cmd->chanlist_len; i++)
	{
		// enable channel
		ao_control_bits |= DAC_CHAN_EN(CR_CHAN(cmd->chanlist[i]));
		// set range
		ao_control_bits |= DAC_RANGE(CR_CHAN(cmd->chanlist[i]),
			CR_RANGE(cmd->chanlist[i]));
	}

	// disable analog out before settings pacer source and count values
	outw(ao_control_bits, devpriv->control_status + DAC_CSR);
	// clear fifo
	outw(0, devpriv->ao_registers + DACFIFOCLR);

	// load counters
	if(cmd->scan_begin_src == TRIG_TIMER)
	{
		i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->ao_divisor1),
			&(devpriv->ao_divisor2), &(cmd->scan_begin_arg),
			cmd->flags);

		/* Write the values of ctr1 and ctr2 into counters 1 and 2 */
		i8254_load(devpriv->pacer_counter_dio + DAC8254, 1, devpriv->ao_divisor1, 2);
		i8254_load(devpriv->pacer_counter_dio + DAC8254, 2, devpriv->ao_divisor2, 2);
	}

	// set number of conversions
	if(cmd->stop_src == TRIG_COUNT)
	{
		devpriv->ao_count = cmd->chanlist_len * cmd->stop_arg;
	}

	// set pacer source
	switch(cmd->scan_begin_src)
	{
		case TRIG_TIMER:
			ao_control_bits |= DAC_PACER_INT;
			break;
		case TRIG_EXT:
			ao_control_bits |= DAC_PACER_EXT_RISE;
			break;
		default:
			comedi_error(dev, "error setting dac pacer source");
			return -1;
			break;
	}

	devpriv->ao_control_bits = ao_control_bits;
	async->inttrig = cb_pcidas_ao_inttrig;

	return 0;
}

static int cb_pcidas_ao_inttrig(comedi_device *dev, comedi_subdevice *s, unsigned int trig_num)
{
	unsigned int i, num_points = thisboard->fifo_size;
	sampl_t data[max_fifo_size];
	comedi_async *async = s->async;
	comedi_cmd *cmd = &s->async->cmd;

	if(trig_num != 0)
		return -EINVAL;

	// load up fifo
	if(cmd->stop_src == TRIG_COUNT &&
		devpriv->ao_count < num_points)
		num_points = devpriv->ao_count;
	for(i = 0; i < num_points; i++)
	{
		if(comedi_buf_get(async, &data[i]))
			break;
	}
	if(cmd->stop_src == TRIG_COUNT)
	{
		devpriv->ao_count -= i;
	}
	// write data to board's fifo
	outsw(devpriv->ao_registers + DACDATA, data, i);

	// enable dac half-full and empty interrupts
	devpriv->adc_fifo_bits |= DAEMIE | DAHFIE;
#ifdef CB_PCIDAS_DEBUG
	rt_printk("comedi: adc_fifo_bits are 0x%x\n", devpriv->adc_fifo_bits);
#endif
	// enable and clear interrupts
	outw(devpriv->adc_fifo_bits | DAEMI | DAHFI, devpriv->control_status + INT_ADCFIFO);
	// enable s5933 interrupt
	outl(devpriv->s5933_intcsr_bits, devpriv->s5933_config + INTCSR);

	// start dac
	devpriv->ao_control_bits |= DAC_START | DACEN | DAC_EMPTY;
	outw(devpriv->ao_control_bits, devpriv->control_status + DAC_CSR);
#ifdef CB_PCIDAS_DEBUG
	rt_printk("comedi: sent 0x%x to dac control\n", devpriv->ao_control_bits);
#endif

	async->inttrig = NULL;

	return 0;
}

static void cb_pcidas_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev = (comedi_device*) d;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async;
	int status;
	int half_fifo = thisboard->fifo_size / 2;
	static const int max_half_fifo = 512;	// maximum possible half-fifo size
	sampl_t data[max_half_fifo];
	int i;
	static const int timeout = 10000;

#ifdef CB_PCIDAS_DEBUG
	if(dev->attached == 0)
	{
		comedi_error(dev, "premature interrupt");
		return;
	}
#endif

	async = s->async;
	async->events = 0;

	status = inw(devpriv->control_status + INT_ADCFIFO);
	if((status & (INT | EOAI | LADFUL | DAHFI | DAEMI)) == 0)
	{
#ifdef CB_PCIDAS_DEBUG
		comedi_error(dev, "spurious interrupt");
#endif
		// clear s5933 interrupt
		outl(devpriv->s5933_intcsr_bits | INBOX_INTR_STATUS, devpriv->s5933_config + INTCSR);
		return;
	}

	// check for analog output interrupt
	if(status & (DAHFI | DAEMI))
	{
		handle_ao_interrupt(dev, status);
	}

	// check for analog input interrupts
	// if fifo half-full
	if(status & ADHFI)
	{
		// read data
		insw(devpriv->adc_fifo + ADCDATA, data, half_fifo);
		for(i = 0; i < half_fifo; i++)
		{
			comedi_buf_put(async, data[i]);
			if(async->cmd.stop_src == TRIG_COUNT)
			{
				if(--devpriv->count == 0)
				{		/* end of acquisition */
					cb_pcidas_cancel(dev, s);
					async->events |= COMEDI_CB_EOA;
					break;
				}
			}
		}
		async->events |= COMEDI_CB_BLOCK;
		// clear half-full interrupt latch
		outw(devpriv->adc_fifo_bits | INT, devpriv->control_status + INT_ADCFIFO);
	// else if fifo not empty
	}else if(status & (ADNEI | EOBI))
	{
		for(i = 0; i < timeout; i++)
		{
			// break if fifo is empty
			if((ADNE & inw(devpriv->control_status + INT_ADCFIFO)) == 0)
				break;
			data[0] = inw(devpriv->adc_fifo);
			comedi_buf_put(async, data[0]);
			if(async->cmd.stop_src == TRIG_COUNT &&
				--devpriv->count == 0)
			{		/* end of acquisition */
				cb_pcidas_cancel(dev, s);
				async->events |= COMEDI_CB_EOA;
				break;
			}
		}
		async->events |= COMEDI_CB_BLOCK;
		// clear not-empty interrupt latch
		outw(devpriv->adc_fifo_bits | INT, devpriv->control_status + INT_ADCFIFO);
	}else if(status & EOAI)
	{
		comedi_error(dev, "bug! encountered end of aquisition interrupt?");
		// clear EOA interrupt latch
		outw(devpriv->adc_fifo_bits | EOAI, devpriv->control_status + INT_ADCFIFO);
	}
	//check for fifo overflow
	if(status & LADFUL)
	{
		comedi_error(dev, "fifo overflow");
		// clear overflow interrupt latch
		outw(devpriv->adc_fifo_bits | LADFUL, devpriv->control_status + INT_ADCFIFO);
		cb_pcidas_cancel(dev, s);
		async->events |= COMEDI_CB_EOA | COMEDI_CB_ERROR;
	}

	// clear interrupt on amcc s5933
	outl(devpriv->s5933_intcsr_bits | INBOX_INTR_STATUS, devpriv->s5933_config + INTCSR);

	comedi_event(dev, s, async->events);

	return;
}

static void handle_ao_interrupt(comedi_device *dev, unsigned int status)
{
	comedi_subdevice *s = dev->write_subdev;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned int half_fifo = thisboard->fifo_size / 2;
	static const int max_half_fifo = max_fifo_size / 2;	// maximum possible half-fifo size
	sampl_t data[max_half_fifo];
	unsigned int num_points, i;

	async->events = 0;

	if(status & DAEMI)
	{
		// clear dac empty interrupt latch
		outw(devpriv->adc_fifo_bits | DAEMI, devpriv->control_status + INT_ADCFIFO);
		if(inw(devpriv->ao_registers + DAC_CSR) & DAC_EMPTY)
		{
			if(cmd->stop_src == TRIG_NONE ||
				(cmd->stop_src == TRIG_COUNT && devpriv->ao_count))
			{
				comedi_error(dev, "dac fifo underflow");
				cb_pcidas_ao_cancel(dev, s);
				async->events |= COMEDI_CB_ERROR;
			}
			async->events |= COMEDI_CB_EOA;
			return;
		}
	}
	if(status & DAHFI)
	{
		// figure out how many points we are writing to fifo
		num_points = half_fifo;
		if(cmd->stop_src == TRIG_COUNT &&
			devpriv->ao_count < num_points)
			num_points = devpriv->ao_count;
		for(i = 0; i < num_points; i++)
		{
			if(comedi_buf_get(async, &data[i]))
				break;
		}
		if(async->cmd.stop_src == TRIG_COUNT)
		{
			devpriv->ao_count -= i;
		}
		// write data to board's fifo
		outsw(devpriv->ao_registers + DACDATA, data, i);
		async->events |= COMEDI_CB_BLOCK;
		// clear half-full interrupt latch
		outw(devpriv->adc_fifo_bits | DAHFI, devpriv->control_status + INT_ADCFIFO);
	}

	comedi_event(dev, s, async->events);
}

// cancel analog input command
static int cb_pcidas_cancel(comedi_device *dev, comedi_subdevice *s)
{
	// disable interrupts
	devpriv->adc_fifo_bits &= ~INTE & ~EOAIE;
	outw(devpriv->adc_fifo_bits, devpriv->control_status + INT_ADCFIFO);
	// disable start trigger source and burst mode
	outw(0, devpriv->control_status + TRIG_CONTSTAT);
	// software pacer source
	outw(0, devpriv->control_status + ADCMUX_CONT);


	return 0;
}

// cancel analog output command
static int cb_pcidas_ao_cancel(comedi_device *dev, comedi_subdevice *s)
{
	// disable interrupts
	devpriv->adc_fifo_bits &= ~DAHFIE & ~DAEMIE;
	outw(devpriv->adc_fifo_bits, devpriv->control_status + INT_ADCFIFO);
	// disable output
	devpriv->ao_control_bits &= ~DACEN & ~DAC_PACER_MASK;
	outw(devpriv->ao_control_bits, devpriv->control_status + DAC_CSR);

	return 0;
}

static void cb_pcidas_load_counters(comedi_device *dev, unsigned int *ns, int rounding_flags)
{
	i8253_cascade_ns_to_timer_2div(TIMER_BASE, &(devpriv->divisor1),
		&(devpriv->divisor2), ns, rounding_flags & TRIG_ROUND_MASK);

	/* Write the values of ctr1 and ctr2 into counters 1 and 2 */
	i8254_load(devpriv->pacer_counter_dio + ADC8254, 1, devpriv->divisor1, 2);
	i8254_load(devpriv->pacer_counter_dio + ADC8254, 2, devpriv->divisor2, 2);
}


/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_cb_pcidas);

