 /*
    comedi/drivers/amplc_pci230.c
    Driver for Amplicon PCI230 and PCI260 Multifunction I/O boards.

    Copyright (C) 2001 Allan Willcox <allanwillcox@ozemail.com.au>

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@schleef.org>

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
Driver: amplc_pci230
Description: Amplicon PCI230, PCI260 Multifunction I/O boards
Author: Allan Willcox <allanwillcox@ozemail.com.au>,
  Steve D Sharples <steve.sharples@nottingham.ac.uk>
Updated: Wed, 31 Oct 2007 12:58:49 +0000
Devices: [Amplicon] PCI230 (pci230 or amplc_pci230), PCI230+ (pci230+),
  PCI260 (pci260 or amplc_pci230), PCI260+ (pci260+)
Status: works

Configuration options:
  [0] - PCI bus of device (optional).
  [1] - PCI slot of device (optional).
          If bus/slot is not specified, the first available PCI device
          will be used.
*/
/*
extra triggered scan functionality, interrupt bug-fix added by Steve Sharples
*/
#include <linux/comedidev.h>

#include <linux/delay.h>

#include "comedi_pci.h"
#include "8253.h"
#include "8255.h"

/* PCI230 PCI configuration register information */
#define PCI_VENDOR_ID_AMPLICON 0x14dc
#define PCI_DEVICE_ID_PCI230 0x0000
#define PCI_DEVICE_ID_PCI260 0x0006
#define PCI_DEVICE_ID_INVALID 0xffff

#define PCI230_IO1_SIZE 32	/* Size of I/O space 1 */
#define PCI230_IO2_SIZE 16	/* Size of I/O space 2 */

/* PCI230 i/o space 1 registers. */
#define PCI230_PPI_X_BASE	0x00	/* User PPI (82C55) base */
#define PCI230_PPI_X_A		0x00	/* User PPI (82C55) port A */
#define PCI230_PPI_X_B		0x01	/* User PPI (82C55) port B */
#define PCI230_PPI_X_C		0x02	/* User PPI (82C55) port C */
#define PCI230_PPI_X_CMD	0x03	/* User PPI (82C55) control word */
#define PCI230_Z2_CT_BASE	0x14	/* 82C54 counter/timer base */
#define PCI230_Z2_CT0		0x14	/* 82C54 counter/timer 0 */
#define PCI230_Z2_CT1		0x15	/* 82C54 counter/timer 1 */
#define PCI230_Z2_CT2		0x16	/* 82C54 counter/timer 2 */
#define PCI230_Z2_CTC		0x17	/* 82C54 counter/timer control word */
#define PCI230_ZCLK_SCE		0x1A	/* Group Z Clock Configuration */
#define PCI230_ZGAT_SCE		0x1D	/* Group Z Gate Configuration */
#define PCI230_INT_SCE		0x1E	/* Interrupt source mask (w) */
#define PCI230_INT_STAT		0x1E	/* Interrupt status (r) */

/* PCI230 i/o space 2 registers. */
#define PCI230_DACCON		0x00	/* DAC control */
#define PCI230_DACOUT1		0x02	/* DAC channel 0 (w) */
#define PCI230_DACOUT2		0x04	/* DAC channel 1 (w) */
#define PCI230_DACOUT3		0x06	/* reserved */
#define PCI230_ADCDATA		0x08	/* ADC data (r) */
#define PCI230_ADCSWTRIG	0x08	/* ADC software trigger (w) */
#define PCI230_ADCCON		0x0A	/* ADC control */
#define PCI230_ADCEN		0x0C	/* ADC channel enable bits */
#define PCI230_ADCG		0x0E	/* ADC gain control bits */
/* PCI230+ i/o space 2 additional registers. */
#define PCI230P_ADCTRIG		0x10	/* ADC start acquisition trigger */
#define PCI230P_ADCTH		0x12	/* ADC analog trigger threshold */
#define PCI230P_ADCFFTH		0x14	/* ADC FIFO interrupt threshold */
#define PCI230P_ADCFFLEV	0x16	/* ADC FIFO level (r) */
#define PCI230P_ADCPTSC		0x18	/* ADC pre-trigger sample count (r) */
#define PCI230P_ADCHYST		0x1A	/* ADC analog trigger hysteresys */
#define PCI230P_EXTFUNC		0x1C	/* Extended functions */
#define PCI230P_HWVER		0x1E	/* Hardware version (r) */

/* Convertor related constants. */
#define PCI230_DAC_SETTLE 5	/* Analogue output settling time in µs */
				/* (DAC itself is 1µs nominally). */
#define PCI230_ADC_SETTLE 1	/* Analogue input settling time in µs */
				/* (ADC itself is 1.6µs nominally but we poll
				 * anyway). */
#define PCI230_MUX_SETTLE 10	/* ADC MUX settling time in µS */
				/* - 10µs for se, 20µs de. */

/* DACCON write values. */
#define PCI230_ADC_OR_UNI		(0<<0)	/* Output range unipolar */
#define PCI230_ADC_OR_BIP		(1<<0)	/* Output range bipolar */
#define PCI230_ADC_OR_MASK		(1<<0)

/* DACCON read values. */
#define PCI230_DAC_BUSY			(1<<1)	/* DAC busy. */

/* ADCCON write values. */
#define PCI230_ADC_TRIG_NONE		(0<<0)	/* No trigger */
#define PCI230_ADC_TRIG_SW		(1<<0)	/* Software trigger trigger */
#define PCI230_ADC_TRIG_EXTP		(2<<0)	/* EXTTRIG +ve edge trigger */
#define PCI230_ADC_TRIG_EXTN		(3<<0)	/* EXTTRIG -ve edge trigger */
#define PCI230_ADC_TRIG_Z2CT0		(4<<0)	/* CT0-OUT +ve edge trigger */
#define PCI230_ADC_TRIG_Z2CT1		(5<<0)	/* CT1-OUT +ve edge trigger */
#define PCI230_ADC_TRIG_Z2CT2		(6<<0)	/* CT2-OUT +ve edge trigger */
#define PCI230_ADC_TRIG_MASK		(7<<0)
#define PCI230_ADC_IR_UNI		(0<<3)	/* Input range unipolar */
#define PCI230_ADC_IR_BIP		(1<<3)	/* Input range bipolar */
#define PCI230_ADC_IR_MASK		(1<<3)
#define PCI230_ADC_IM_SE		(0<<4)	/* Input mode single ended */
#define PCI230_ADC_IM_DIF		(1<<4)	/* Input mode differential */
#define PCI230_ADC_IM_MASK		(1<<4)
#define PCI230_ADC_FIFO_EN		(1<<8)	/* FIFO enable */
#define PCI230_ADC_INT_FIFO_EMPTY	(0<<9)
#define PCI230_ADC_INT_FIFO_NEMPTY	(1<<9)	/* FIFO interrupt not empty */
#define PCI230_ADC_INT_FIFO_NHALF	(2<<9)
#define PCI230_ADC_INT_FIFO_HALF	(3<<9)	/* FIFO interrupt half full */
#define PCI230_ADC_INT_FIFO_NFULL	(4<<9)
#define PCI230_ADC_INT_FIFO_FULL	(5<<9)	/* FIFO interrupt full */
#define PCI230_ADC_INT_FIFO_MASK	(7<<9)
#define PCI230_ADC_FIFO_RESET		(1<<12)	/* FIFO reset */
#define PCI230_ADC_GLOB_RESET		(1<<13)	/* Global reset */
#define PCI230_ADC_CONV			0xffff
			/* Value to write to ADCSWTRIG to trigger ADC conversion
			 * in software trigger mode.  Can be anything.  */

/* ADCCON read values. */
#define PCI230_ADC_BUSY			(1<<15)	/* ADC busy */
#define PCI230_ADC_FIFO_EMPTY		(1<<12)	/* FIFO empty */
#define PCI230_ADC_FIFO_FULL		(1<<13)	/* FIFO full */
#define PCI230_ADC_FIFO_HALF		(1<<14)	/* FIFO half full */
#define PCI230_ADC_FIFO_FULL_LATCHED	(1<<5)	/* Indicates overrun occurred */

/* PCI230 ADC FIFO levels. */
#define PCI230_ADC_FIFOLEVEL_HALFFULL	2049	/* Value for FIFO half full */
#define PCI230_ADC_FIFOLEVEL_FULL	4096	/* FIFO size */

/* PCI230+ EXTFUNC values. */
#define PCI230P_EXTFUNC_GAT_EXTTRIG	(1<<0)
			/* Route EXTTRIG pin to external gate inputs. */

/*
 * Counter/timer clock input configuration sources.
 */
#define CLK_CLK		0	/* reserved (channel-specific clock) */
#define CLK_10MHZ	1	/* internal 10 MHz clock */
#define CLK_1MHZ	2	/* internal 1 MHz clock */
#define CLK_100KHZ	3	/* internal 100 kHz clock */
#define CLK_10KHZ	4	/* internal 10 kHz clock */
#define CLK_1KHZ	5	/* internal 1 kHz clock */
#define CLK_OUTNM1	6	/* output of channel-1 modulo total */
#define CLK_EXT		7	/* external clock */
/* Macro to construct clock input configuration register value. */
#define CLK_CONFIG(chan, src)	((((chan) & 3) << 3) | ((src) & 7))
/* Timebases in ns. */
#define TIMEBASE_10MHZ		100
#define TIMEBASE_1MHZ		1000
#define TIMEBASE_100KHZ		10000
#define TIMEBASE_10KHZ		100000
#define TIMEBASE_1KHZ		1000000

/*
 * Counter/timer gate input configuration sources.
 */
#define GAT_VCC		0	/* VCC (i.e. enabled) */
#define GAT_GND		1	/* GND (i.e. disabled) */
#define GAT_EXT		2	/* external gate input (PPCn on PCI230) */
#define GAT_NOUTNM2	3	/* inverted output of channel-2 modulo total */
/* Macro to construct gate input configuration register value. */
#define GAT_CONFIG(chan, src)	((((chan) & 3) << 3) | ((src) & 7))

/*
 * Summary of CLK_OUTNM1 and GAT_NOUTNM2 connections for PCI230 and PCI260:
 *
 *              Channel's       Channel's
 *              clock input     gate input
 * Channel      CLK_OUTNM1      GAT_NOUTNM2
 * -------      ----------      -----------
 * Z2-CT0       Z2-CT2-OUT      /Z2-CT1-OUT
 * Z2-CT1       Z2-CT0-OUT      /Z2-CT2-OUT
 * Z2-CT2       Z2-CT1-OUT      /Z2-CT0-OUT
 */

/* Interrupt enables/status register values. */
#define PCI230_INT_DISABLE		0
#define PCI230_INT_PPI_C0		1
#define PCI230_INT_PPI_C3		2
#define PCI230_INT_ADC			4
#define PCI230_INT_ZCLK_CT1		32

#define PCI230_TEST_BIT(val, n)	((val>>n)&1)
			/* Assumes bits numbered with zero offset, ie. 0-15 */

/* (Potentially) shared resources and their owners */
enum {
	RES_Z2CT0,		/* Z2-CT0 */
	RES_Z2CT1,		/* Z2-CT1 */
	RES_Z2CT2,		/* Z2-CT2 */
	NUM_RESOURCES		/* Number of (potentially) shared resources. */
};

enum {
	OWNER_NONE,		/* Not owned */
	OWNER_AICMD,		/* Owned by AI command */
	OWNER_AOCMD		/* Owned by AO command */
};

/*
 * Handy macros.
 */

/* Combine old and new bits. */
#define COMBINE(old, new, mask)	(((old) & ~(mask)) | ((new) & (mask)))

/* A generic null function pointer value.  */
#define NULLFUNC	0

/* Current CPU.  XXX should this be hard_smp_processor_id()? */
#define THISCPU		smp_processor_id()

/*
 * Board descriptions for the two boards supported.
 */

typedef struct pci230_board_struct {
	const char *name;
	unsigned short id;
	int ai_chans;
	int ai_bits;
	int ao_chans;
	int ao_bits;
	int have_dio;
	unsigned int min_hwver;	/* Minimum hardware version supported. */
} pci230_board;
static const pci230_board pci230_boards[] = {
	{
	      name:	"pci230",
	      id:	PCI_DEVICE_ID_PCI230,
	      ai_chans:16,
	      ai_bits:	12,
	      ao_chans:2,
	      ao_bits:	12,
	      have_dio:1,
		},
	{
	      name:	"pci260",
	      id:	PCI_DEVICE_ID_PCI260,
	      ai_chans:16,
	      ai_bits:	12,
	      ao_chans:0,
	      ao_bits:	0,
	      have_dio:0,
		},
	{
	      name:	"amplc_pci230",	/* Legacy name matches any above */
	      id:	PCI_DEVICE_ID_INVALID,
		},
	/*
	 * The '+' versions of the above models have the same PCI device ID.
	 * They are backwards compatible with the above models.
	 */
	{
	      name:	"pci230+",
	      id:	PCI_DEVICE_ID_PCI230,
	      ai_chans:16,
	      ai_bits:	16,
	      ao_chans:2,
	      ao_bits:	12,
	      have_dio:1,
	      min_hwver:1,
		},
	{
	      name:	"pci260+",
	      id:	PCI_DEVICE_ID_PCI260,
	      ai_chans:16,
	      ai_bits:	16,
	      ao_chans:0,
	      ao_bits:	0,
	      have_dio:0,
	      min_hwver:1,
		},
};

static struct pci_device_id pci230_pci_table[] __devinitdata = {
	{PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_PCI230, PCI_ANY_ID, PCI_ANY_ID,
		0, 0, 0},
	{PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_PCI260, PCI_ANY_ID, PCI_ANY_ID,
		0, 0, 0},
	{0}
};

MODULE_DEVICE_TABLE(pci, pci230_pci_table);
/*
 * Useful for shorthand access to the particular board structure
 */
#define n_pci230_boards (sizeof(pci230_boards)/sizeof(pci230_boards[0]))
#define thisboard ((const pci230_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
struct pci230_private {
	struct pci_dev *pci_dev;
	spinlock_t isr_spinlock;	/* Interrupt spin lock */
	spinlock_t ai_inttrig_spinlock;	/* AI command inttrig spin lock */
	spinlock_t ao_inttrig_spinlock;	/* AO command inttrig spin lock */
	spinlock_t res_spinlock;	/* Shared resources spin lock */
	unsigned long iobase1;	/* PCI230's I/O space 1 */
	lsampl_t ao_readback[2];	/* Used for AO readback */
	unsigned int ai_scan_count;	/* Number of analogue input scans
					 * remaining.  */
	unsigned int ai_scan_pos;	/* Current position within analogue
					 * input scan */
	unsigned int ao_scan_count;	/* Number of analogue output scans
					 * remaining.  */
	int intr_cpuid;		/* ID of CPU running interrupt routine. */
	unsigned short hwver;	/* Hardware version (for '+' models). */
	unsigned short adccon;	/* ADCCON register value. */
	unsigned short adcg;	/* ADCG register value. */
	unsigned char int_en;	/* Interrupt enables bits. */
	unsigned char ai_continuous;	/* Flag set when cmd->stop_src ==
					 * TRIG_NONE - user chooses to stop
					 * continuous conversion by
					 * cancelation. */
	unsigned char ao_continuous;	/* Flag set when cmd->stop_src ==
					 * TRIG_NONE - user chooses to stop
					 * continuous conversion by
					 * cancelation. */
	unsigned char ai_bipolar;	/* Set if bipolar input range so we
					 * know to mangle it. */
	unsigned char ao_bipolar;	/* Set if bipolar output range so we
					 * know to mangle it. */
	unsigned char ier;	/* Copy of interrupt enables/status register. */
	unsigned char intr_running;	/* Flag set in interrupt routine. */
	unsigned char res_owner[NUM_RESOURCES];	/* Shared resource owners. */
};

#define devpriv ((struct pci230_private *)dev->private)

/* PCI230 clock source periods in ns */
static const unsigned int pci230_timebase[8] = {
	[CLK_10MHZ] = TIMEBASE_10MHZ,
	[CLK_1MHZ] = TIMEBASE_1MHZ,
	[CLK_100KHZ] = TIMEBASE_100KHZ,
	[CLK_10KHZ] = TIMEBASE_10KHZ,
	[CLK_1KHZ] = TIMEBASE_1KHZ,
};

/* PCI230 analogue input range table */
static const comedi_lrange pci230_ai_range = { 7, {
			BIP_RANGE(10),
			BIP_RANGE(5),
			BIP_RANGE(2.5),
			BIP_RANGE(1.25),
			UNI_RANGE(10),
			UNI_RANGE(5),
			UNI_RANGE(2.5)
	}
};

/* PCI230 analogue gain bits for each input range. */
static const unsigned char pci230_ai_gain[7] = { 0, 1, 2, 3, 1, 2, 3 };

/* PCI230 adccon bipolar flag for each analogue input range. */
static const unsigned char pci230_ai_bipolar[7] = { 1, 1, 1, 1, 0, 0, 0 };

/* PCI230 analogue output range table */
static const comedi_lrange pci230_ao_range = { 2, {
			UNI_RANGE(10),
			BIP_RANGE(10)
	}
};

/* PCI230 daccon bipolar flag for each analogue output range. */
static const unsigned char pci230_ao_bipolar[2] = { 0, 1 };

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int pci230_attach(comedi_device * dev, comedi_devconfig * it);
static int pci230_detach(comedi_device * dev);
static comedi_driver driver_amplc_pci230 = {
      driver_name:"amplc_pci230",
      module:THIS_MODULE,
      attach:pci230_attach,
      detach:pci230_detach,
      board_name:&pci230_boards[0].name,
      offset:sizeof(pci230_boards[0]),
      num_names:sizeof(pci230_boards) / sizeof(pci230_boards[0]),
};

COMEDI_INITCLEANUP(driver_amplc_pci230);

static int pci230_ai_rinsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int pci230_ao_winsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int pci230_ao_rinsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static void pci230_ct_setup_ns_mode(comedi_device * dev, unsigned int ct,
	unsigned int mode, uint64_t ns, unsigned int round);
static void pci230_ns_to_single_timer(unsigned int *ns, unsigned int round);
static void pci230_cancel_ct(comedi_device * dev, unsigned int ct);
static irqreturn_t pci230_interrupt(int irq, void *d PT_REGS_ARG);
static int pci230_ao_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd);
static int pci230_ao_cmd(comedi_device * dev, comedi_subdevice * s);
static int pci230_ao_cancel(comedi_device * dev, comedi_subdevice * s);
static void pci230_ao_stop(comedi_device * dev, comedi_subdevice * s);
static void pci230_handle_ao(comedi_device * dev, comedi_subdevice * s);
static int pci230_ai_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd);
static int pci230_ai_cmd(comedi_device * dev, comedi_subdevice * s);
static int pci230_ai_cancel(comedi_device * dev, comedi_subdevice * s);
static void pci230_ai_stop(comedi_device * dev, comedi_subdevice * s);
static void pci230_handle_ai(comedi_device * dev, comedi_subdevice * s);

static sampl_t pci230_ai_read(comedi_device * dev)
{
	/* Read sample. */
	sampl_t data = (sampl_t) inw(dev->iobase + PCI230_ADCDATA);

	/* PCI230 is 12 bit - stored in upper bits of 16 bit register (lower
	 * four bits reserved for expansion). */
	/* PCI230+ is 16 bit AI. */
	data = data >> (16 - thisboard->ai_bits);

	/* If a bipolar range was specified, mangle it (twos
	 * complement->straight binary). */
	if (devpriv->ai_bipolar) {
		data ^= 1 << (thisboard->ai_bits - 1);
	}
	return data;
}

static void pci230_ao_write(comedi_device * dev, sampl_t data, int chan)
{
	/* If a bipolar range was specified, mangle it (straight binary->twos
	 * complement). */
	if (devpriv->ao_bipolar) {
		data ^= 1 << (thisboard->ao_bits - 1);
	}

	/* PCI230 is 12 bit - stored in upper bits of 16 bit register (lower
	 * four bits reserved for expansion). */
	/* PCI230+ is also 12 bit AO. */
	data = data << (16 - thisboard->ao_bits);

	/* Write data. */
	outw((unsigned int)data, dev->iobase + (((chan) == 0)
			? PCI230_DACOUT1 : PCI230_DACOUT2));
}

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int pci230_attach(comedi_device * dev, comedi_devconfig * it)
{
	comedi_subdevice *s;
	unsigned long iobase1, iobase2;
	/* PCI230's I/O spaces 1 and 2 respectively. */
	struct pci_dev *pci_dev;
	int i = 0, irq_hdl, rc;

	printk("comedi%d: amplc_pci230: attach %s %d,%d\n", dev->minor,
		thisboard->name, it->options[0], it->options[1]);

	/* Allocate the private structure area using alloc_private().
	 * Macro defined in comedidev.h - memsets struct fields to 0. */
	if ((alloc_private(dev, sizeof(struct pci230_private))) < 0) {
		return -ENOMEM;
	}
	spin_lock_init(&devpriv->isr_spinlock);
	spin_lock_init(&devpriv->ai_inttrig_spinlock);
	spin_lock_init(&devpriv->ao_inttrig_spinlock);
	spin_lock_init(&devpriv->res_spinlock);
	/* Find card */
	for (pci_dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
		pci_dev != NULL;
		pci_dev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pci_dev)) {
		if (it->options[0] || it->options[1]) {
			/* Match against bus/slot options. */
			if (it->options[0] != pci_dev->bus->number ||
				it->options[1] != PCI_SLOT(pci_dev->devfn))
				continue;
		}
		if (pci_dev->vendor != PCI_VENDOR_ID_AMPLICON)
			continue;
		if (thisboard->id == PCI_DEVICE_ID_INVALID) {
			/* The name was specified as "amplc_pci230" which is
			 * used to match any supported device.  Replace the
			 * current dev->board_ptr with one that matches the
			 * PCI device ID. */
			for (i = 0; i < n_pci230_boards; i++) {
				if (pci_dev->device == pci230_boards[i].id) {
					/* Change board_ptr to matched board */
					dev->board_ptr = &pci230_boards[i];
					break;
				}
			}
			if (i < n_pci230_boards)
				break;
		} else {
			/* The name was specified as a specific device name.
			 * The current dev->board_ptr is correct.  Check
			 * whether it matches the PCI device ID. */
			if (thisboard->id == pci_dev->device) {
				/* Check minimum hardware version. */
				if (thisboard->min_hwver > 0) {
					/* Looking for a '+' model.  First
					 * check length of registers. */
					if (pci_resource_len(pci_dev, 3) < 32) {
						/* Not a '+' model. */
						continue;
					}
					/* TODO: temporarily enable the PCI
					 * device and read the hardware version
					 * register.  For now, assume it's
					 * okay. */
					break;
				} else {
					break;
				}
			}
		}
	}
	if (!pci_dev) {
		printk("comedi%d: No %s card found\n", dev->minor,
			thisboard->name);
		return -EIO;
	}
	devpriv->pci_dev = pci_dev;

	/*
	 * Initialize dev->board_name.
	 */
	dev->board_name = thisboard->name;

	/* Enable PCI device and reserve I/O spaces. */
	if (comedi_pci_enable(pci_dev, "amplc_pci230") < 0) {
		printk("comedi%d: failed to enable PCI device "
			"and request regions\n", dev->minor);
		return -EIO;
	}

	/* Read base addresses of the PCI230's two I/O regions from PCI
	 * configuration register. */
	iobase1 = pci_resource_start(pci_dev, 2);
	iobase2 = pci_resource_start(pci_dev, 3);

	printk("comedi%d: %s I/O region 1 0x%04lx I/O region 2 0x%04lx\n",
		dev->minor, dev->board_name, iobase1, iobase2);

	devpriv->iobase1 = iobase1;
	dev->iobase = iobase2;

	/* Read hardware version register and set extended function register
	 * if they exist. */
	if (pci_resource_len(pci_dev, 3) >= 32) {
		unsigned short extfunc = 0;

		devpriv->hwver = inw(dev->iobase + PCI230P_HWVER);
		if (devpriv->hwver < thisboard->min_hwver) {
			printk("comedi%d: %s - bad hardware version "
				"- got %u, need %u\n", dev->minor,
				dev->board_name, devpriv->hwver,
				thisboard->min_hwver);
			return -EIO;
		}
		if (devpriv->hwver > 0) {
			if (!thisboard->have_dio) {
				/* No DIO ports.  Route counters' external gates
				 * to the EXTTRIG signal (PCI260+ pin 17).
				 * (Otherwise, they would be routed to DIO
				 * inputs PC0, PC1 and PC2 which don't exist
				 * on PCI260[+].) */
				extfunc |= PCI230P_EXTFUNC_GAT_EXTTRIG;
			}
		}
		outw(extfunc, dev->iobase + PCI230P_EXTFUNC);
	}

	/* Disable board's interrupts. */
	outb(0, devpriv->iobase1 + PCI230_INT_SCE);

	/* Set ADC to a reasonable state. */
	devpriv->adcg = 0;
	devpriv->adccon = PCI230_ADC_TRIG_NONE | PCI230_ADC_IM_SE
		| PCI230_ADC_IR_BIP;
	outw(1 << 0, dev->iobase + PCI230_ADCEN);
	outw(devpriv->adcg, dev->iobase + PCI230_ADCG);
	outw(devpriv->adccon | PCI230_ADC_FIFO_RESET,
		dev->iobase + PCI230_ADCCON);

	/* Register the interrupt handler. */
	irq_hdl = comedi_request_irq(devpriv->pci_dev->irq, pci230_interrupt,
		IRQF_SHARED, "amplc_pci230", dev);
	if (irq_hdl < 0) {
		printk("comedi%d: unable to register irq, "
			"commands will not be available %d\n", dev->minor,
			devpriv->pci_dev->irq);
	} else {
		dev->irq = devpriv->pci_dev->irq;
		printk("comedi%d: registered irq %u\n", dev->minor,
			devpriv->pci_dev->irq);
	}

	/*
	 * Allocate the subdevice structures.  alloc_subdevice() is a
	 * convenient macro defined in comedidev.h.
	 */
	if (alloc_subdevices(dev, 3) < 0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	/* analog input subdevice */
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_DIFF | SDF_GROUND;
	s->n_chan = thisboard->ai_chans;
	s->maxdata = (1 << thisboard->ai_bits) - 1;
	s->range_table = &pci230_ai_range;
	s->insn_read = &pci230_ai_rinsn;
	s->len_chanlist = 256;	/* but there are restrictions. */
	/* Only register commands if the interrupt handler is installed. */
	if (irq_hdl == 0) {
		dev->read_subdev = s;
		s->subdev_flags |= SDF_CMD_READ;
		s->do_cmd = &pci230_ai_cmd;
		s->do_cmdtest = &pci230_ai_cmdtest;
		s->cancel = pci230_ai_cancel;
	}

	s = dev->subdevices + 1;
	/* analog output subdevice */
	if (thisboard->ao_chans > 0) {
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_WRITABLE | SDF_GROUND;
		s->n_chan = thisboard->ao_chans;;
		s->maxdata = (1 << thisboard->ao_bits) - 1;
		s->range_table = &pci230_ao_range;
		s->insn_write = &pci230_ao_winsn;
		s->insn_read = &pci230_ao_rinsn;
		s->len_chanlist = thisboard->ao_chans;
		/* Only register commands if the interrupt handler is
		 * installed. */
		if (irq_hdl == 0) {
			dev->write_subdev = s;
			s->subdev_flags |= SDF_CMD_WRITE;
			s->do_cmd = &pci230_ao_cmd;
			s->do_cmdtest = &pci230_ao_cmdtest;
			s->cancel = pci230_ao_cancel;
		}
	} else {
		s->type = COMEDI_SUBD_UNUSED;
	}

	s = dev->subdevices + 2;
	/* digital i/o subdevice */
	if (thisboard->have_dio) {
		rc = subdev_8255_init(dev, s, NULL,
			(devpriv->iobase1 + PCI230_PPI_X_BASE));
		if (rc < 0)
			return rc;
	} else {
		s->type = COMEDI_SUBD_UNUSED;
	}

	printk("comedi%d: attached\n", dev->minor);

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
static int pci230_detach(comedi_device * dev)
{
	printk("comedi%d: amplc_pci230: remove\n", dev->minor);

	if (dev->subdevices && thisboard->have_dio)
		/* Clean up dio subdevice. */
		subdev_8255_cleanup(dev, dev->subdevices + 2);

	if (dev->irq)
		comedi_free_irq(dev->irq, dev);

	if (devpriv) {
		if (devpriv->pci_dev) {
			if (dev->iobase) {
				comedi_pci_disable(devpriv->pci_dev);
			}
			pci_dev_put(devpriv->pci_dev);
		}
	}

	return 0;
}

static int get_resources(comedi_device * dev, unsigned int res_mask,
	unsigned char owner)
{
	int ok;
	unsigned int i;
	unsigned int b;
	unsigned int claimed;
	unsigned long irqflags;

	ok = 1;
	claimed = 0;
	comedi_spin_lock_irqsave(&devpriv->res_spinlock, irqflags);
	for (b = 1, i = 0; (i < NUM_RESOURCES)
		&& (res_mask != 0); b <<= 1, i++) {
		if ((res_mask & b) != 0) {
			res_mask &= ~b;
			if (devpriv->res_owner[i] == OWNER_NONE) {
				devpriv->res_owner[i] = owner;
				claimed |= b;
			} else if (devpriv->res_owner[i] != owner) {
				for (b = 1, i = 0; claimed != 0; b <<= 1, i++) {
					if ((claimed & b) != 0) {
						devpriv->res_owner[i]
							= OWNER_NONE;
						claimed &= ~b;
					}
				}
				ok = 0;
				break;
			}
		}
	}
	comedi_spin_unlock_irqrestore(&devpriv->res_spinlock, irqflags);
	return ok;
}

static inline int get_one_resource(comedi_device * dev, unsigned int resource,
	unsigned char owner)
{
	return get_resources(dev, (1U << resource), owner);
}

static void put_resources(comedi_device * dev, unsigned int res_mask,
	unsigned char owner)
{
	unsigned int i;
	unsigned int b;
	unsigned long irqflags;

	comedi_spin_lock_irqsave(&devpriv->res_spinlock, irqflags);
	for (b = 1, i = 0; (i < NUM_RESOURCES)
		&& (res_mask != 0); b <<= 1, i++) {
		if ((res_mask & b) != 0) {
			res_mask &= ~b;
			if (devpriv->res_owner[i] == owner) {
				devpriv->res_owner[i] = OWNER_NONE;
			}
		}
	}
	comedi_spin_unlock_irqrestore(&devpriv->res_spinlock, irqflags);
}

static inline void put_one_resource(comedi_device * dev, unsigned int resource,
	unsigned char owner)
{
	put_resources(dev, (1U << resource), owner);
}

static inline void put_all_resources(comedi_device * dev, unsigned char owner)
{
	put_resources(dev, (1U << NUM_RESOURCES) - 1, owner);
}

/*
 *  COMEDI_SUBD_AI instruction;
 */
static int pci230_ai_rinsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	unsigned int n, i;
	unsigned int chan, range, aref;
	unsigned int gainshift;
	unsigned int status;
	unsigned short adccon, adcen;

	/* Unpack channel and range. */
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);
	aref = CR_AREF(insn->chanspec);

	adccon = PCI230_ADC_TRIG_SW;
	devpriv->ai_bipolar = pci230_ai_bipolar[range];
	if (aref == AREF_DIFF) {
		/* Differential. */
		if (chan >= s->n_chan / 2) {
			DPRINTK("comedi%d: amplc_pci230: ai_rinsn: "
				"differential channel number out of range "
				"0 to %u\n", dev->minor, (s->n_chan / 2) - 1);
			return -EINVAL;
		}
		gainshift = chan * 2;
		if (devpriv->hwver == 0) {
			/* Original PCI230/260 expects both inputs of the
			 * differential channel to be enabled. */
			adcen = 3 << gainshift;
		} else {
			/* PCI230+/260+ expects only one input of the
			 * differential channel to be enabled. */
			adcen = 1 << gainshift;
		}
		adccon |= PCI230_ADC_IM_DIF;
	} else {
		/* Single ended. */
		adcen = 1 << chan;
		gainshift = chan & ~1;
		adccon |= PCI230_ADC_IM_SE;
	}
	devpriv->adcg = (devpriv->adcg & ~(3 << gainshift))
		| (pci230_ai_gain[range] << gainshift);
	if (devpriv->ai_bipolar) {
		adccon |= PCI230_ADC_IR_BIP;
	} else {
		adccon |= PCI230_ADC_IR_UNI;
	}

	/* Enable only this channel in the scan list - otherwise by default
	 * we'll get one sample from each channel. */
	outw(adcen, dev->iobase + PCI230_ADCEN);

	/* Set gain for channel. */
	outw(devpriv->adcg, dev->iobase + PCI230_ADCG);

	/* Specify uni/bip, se/diff, s/w conversion, and reset FIFO (even
	 * though we're not using it - MEV says so). */
	devpriv->adccon = adccon;
	outw(adccon | PCI230_ADC_FIFO_RESET, dev->iobase + PCI230_ADCCON);

	/* Convert n samples */
	for (n = 0; n < insn->n; n++) {
		/* trigger conversion */
		outw(PCI230_ADC_CONV, dev->iobase + PCI230_ADCSWTRIG);

#define TIMEOUT 100
		/* wait for conversion to end */
		for (i = 0; i < TIMEOUT; i++) {
			status = inw(dev->iobase + PCI230_ADCCON);
			if (!(status & PCI230_ADC_BUSY))
				break;
		}
		if (i == TIMEOUT) {
			/* rt_printk() should be used instead of printk()
			 * whenever the code can be called from real-time. */
			rt_printk("timeout\n");
			return -ETIMEDOUT;
		}

		/* read data */
		data[n] = pci230_ai_read(dev);
	}

	/* return the number of samples read/written */
	return n;
}

/*
 *  COMEDI_SUBD_AO instructions;
 */
static int pci230_ao_winsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i;
	int chan, range;

	/* Unpack channel and range. */
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);

	/* Set range - see analogue output range table; 0 => unipolar 10V,
	 * 1 => bipolar +/-10V range scale */
	devpriv->ao_bipolar = pci230_ao_bipolar[range];
	outw(range, dev->iobase + PCI230_DACCON);

	/* Writing a list of values to an AO channel is probably not
	 * very useful, but that's how the interface is defined. */
	for (i = 0; i < insn->n; i++) {
		/* Store the value to be written to the DAC in our
		 * pci230_private struct before mangling it. */
		devpriv->ao_readback[chan] = data[i];

		/* Write value to DAC. */
		pci230_ao_write(dev, data[i], chan);
	}

	/* return the number of samples read/written */
	return i;
}

/* AO subdevices should have a read insn as well as a write insn.
 * Usually this means copying a value stored in devpriv. */
static int pci230_ao_rinsn(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	for (i = 0; i < insn->n; i++)
		data[i] = devpriv->ao_readback[chan];

	return i;
}

static int pci230_ao_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd)
{
	int err = 0;
	unsigned int tmp;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* Step 1: make sure trigger sources are trivially valid.
	 * "invalid source" returned by comedilib to user mode process
	 * if this fails. */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_INT;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_NOW;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/* Step 2: make sure trigger sources are unique and mutually compatible
	 * "source conflict" returned by comedilib to user mode process
	 * if this fails. */

	/* these tests are true if more than one _src bit is set */
	if ((cmd->start_src & (cmd->start_src - 1)) != 0)
		err++;
	if ((cmd->scan_begin_src & (cmd->scan_begin_src - 1)) != 0)
		err++;
	if ((cmd->convert_src & (cmd->convert_src - 1)) != 0)
		err++;
	if ((cmd->scan_end_src & (cmd->scan_end_src - 1)) != 0)
		err++;
	if ((cmd->stop_src & (cmd->stop_src - 1)) != 0)
		err++;

	if (err)
		return 2;

	/* Step 3: make sure arguments are trivially compatible.
	 * "invalid argument" returned by comedilib to user mode process
	 * if this fails. */

	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		err++;
	}
#define MAX_SPEED_AO	8000	/* 8000 ns => 125 kHz */
#define MIN_SPEED_AO	4294967295u	/* 4294967295ns = 4.29s */
			/*- Comedi limit due to unsigned int cmd.  Driver limit
			 * = 2^16 (16bit * counter) * 1000000ns (1kHz onboard
			 * clock) = 65.536s */

	if (cmd->scan_begin_src == TRIG_TIMER) {
		if (cmd->scan_begin_arg < MAX_SPEED_AO) {
			cmd->scan_begin_arg = MAX_SPEED_AO;
			err++;
		}
		if (cmd->scan_begin_arg > MIN_SPEED_AO) {
			cmd->scan_begin_arg = MIN_SPEED_AO;
			err++;
		}
	}

	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	if (cmd->stop_src == TRIG_NONE) {
		/* TRIG_NONE */
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
	}

	if (err)
		return 3;

	/* Step 4: fix up any arguments.
	 * "argument conflict" returned by comedilib to user mode process
	 * if this fails. */

	if (cmd->scan_begin_src == TRIG_TIMER) {
		tmp = cmd->scan_begin_arg;
		pci230_ns_to_single_timer(&cmd->scan_begin_arg,
			cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->scan_begin_arg)
			err++;
	}

	if (err)
		return 4;

	/* Step 5: check channel list if it exists. */

	if (cmd->chanlist && cmd->chanlist_len > 0) {
		enum {
			seq_err = (1 << 0),
			range_err = (1 << 1)
		};
		unsigned int errors;
		unsigned int n;
		unsigned int chan, prev_chan;
		unsigned int range, first_range;

		prev_chan = CR_CHAN(cmd->chanlist[0]);
		first_range = CR_RANGE(cmd->chanlist[0]);
		errors = 0;
		for (n = 1; n < cmd->chanlist_len; n++) {
			chan = CR_CHAN(cmd->chanlist[n]);
			range = CR_RANGE(cmd->chanlist[n]);
			/* Channel numbers must strictly increase. */
			if (chan < prev_chan) {
				errors |= seq_err;
			}
			/* Ranges must be the same. */
			if (range != first_range) {
				errors |= range_err;
			}
			prev_chan = chan;
		}
		if (errors != 0) {
			err++;
			if ((errors & seq_err) != 0) {
				DPRINTK("comedi%d: amplc_pci230: ao_cmdtest: "
					"channel numbers must increase\n",
					dev->minor);
			}
			if ((errors & range_err) != 0) {
				DPRINTK("comedi%d: amplc_pci230: ao_cmdtest: "
					"channels must have the same range\n",
					dev->minor);
			}
		}
	}

	if (err)
		return 5;

	return 0;
}

static void pci230_ao_start(comedi_device * dev, comedi_subdevice * s)
{
	unsigned long irqflags;

	if (!devpriv->ao_continuous && (devpriv->ao_scan_count == 0)) {
		/* An empty acquisition! */
		s->async->events |= COMEDI_CB_EOA;
		pci230_ao_stop(dev, s);
		comedi_event(dev, s);
	} else {
		/* Enable DAC interrupt. */
		comedi_spin_lock_irqsave(&devpriv->isr_spinlock, irqflags);
		devpriv->int_en |= PCI230_INT_ZCLK_CT1;
		devpriv->ier |= PCI230_INT_ZCLK_CT1;
		outb(devpriv->ier, devpriv->iobase1 + PCI230_INT_SCE);
		comedi_spin_unlock_irqrestore(&devpriv->isr_spinlock, irqflags);
	}
}

static int pci230_ao_inttrig_start(comedi_device * dev, comedi_subdevice * s,
	unsigned int trig_num)
{
	unsigned long irqflags;

	if (trig_num != 0)
		return -EINVAL;

	comedi_spin_lock_irqsave(&devpriv->ao_inttrig_spinlock, irqflags);
	if (s->async->inttrig) {
		s->async->inttrig = NULLFUNC;
		comedi_spin_unlock_irqrestore(&devpriv->ao_inttrig_spinlock,
			irqflags);
		pci230_ao_start(dev, s);
	} else {
		comedi_spin_unlock_irqrestore(&devpriv->ao_inttrig_spinlock,
			irqflags);
	}

	return 1;
}

static int pci230_ao_cmd(comedi_device * dev, comedi_subdevice * s)
{
	unsigned long irqflags;
	int range;

	/* Get the command. */
	comedi_cmd *cmd = &s->async->cmd;

	/* Claim Z2-CT1. */
	if (!get_one_resource(dev, RES_Z2CT1, OWNER_AOCMD)) {
		return -EBUSY;
	}

	/* Get number of scans required. */
	if (cmd->stop_src == TRIG_COUNT) {
		devpriv->ao_scan_count = cmd->stop_arg;
		devpriv->ao_continuous = 0;
	} else {
		/* TRIG_NONE, user calls cancel. */
		devpriv->ao_scan_count = 0;
		devpriv->ao_continuous = 1;
	}

	/* Set range - see analogue output range table; 0 => unipolar 10V,
	 * 1 => bipolar +/-10V range scale */
	range = CR_RANGE(cmd->chanlist[0]);
	devpriv->ao_bipolar = pci230_ao_bipolar[range];
	outw(range, dev->iobase + PCI230_DACCON);

	/* Set the counter timer 1 to the specified scan frequency. */
	/* cmd->scan_begin_arg is sampling period in ns */
	pci230_ct_setup_ns_mode(dev, 1, I8254_MODE3, cmd->scan_begin_arg,
		cmd->flags & TRIG_ROUND_MASK);

	/* N.B. cmd->start_src == TRIG_INT */
	comedi_spin_lock_irqsave(&devpriv->ao_inttrig_spinlock, irqflags);
	s->async->inttrig = pci230_ao_inttrig_start;
	comedi_spin_unlock_irqrestore(&devpriv->ao_inttrig_spinlock, irqflags);

	return 0;
}

static int pci230_ai_check_scan_period(comedi_cmd * cmd)
{
	unsigned int min_scan_period, chanlist_len;
	int err = 0;

	chanlist_len = cmd->chanlist_len;
	if (cmd->chanlist_len == 0) {
		chanlist_len = 1;
	}
	min_scan_period = chanlist_len * cmd->convert_arg;
	if ((min_scan_period < chanlist_len)
		|| (min_scan_period < cmd->convert_arg)) {
		/* Arithmetic overflow. */
		min_scan_period = UINT_MAX;
		err++;
	}
	if (cmd->scan_begin_arg < min_scan_period) {
		cmd->scan_begin_arg = min_scan_period;
		err++;
	}

	return !err;
}

static int pci230_ai_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd)
{
	int err = 0;
	unsigned int tmp;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4,5 or 0, depending on which tests
	 * the command passes. */

	/* Step 1: make sure trigger sources are trivially valid.
	 * "invalid source" returned by comedilib to user mode process
	 * if this fails. */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_INT;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	/* Unfortunately, we cannot trigger a scan off an external source
	 * on the PCI260 board, since it uses the PPI0 (DIO) input, which
	 * isn't present on the PCI260 */
	if (thisboard->have_dio) {
		cmd->scan_begin_src &= TRIG_FOLLOW | TRIG_TIMER | TRIG_INT
			| TRIG_EXT;
	} else {
		cmd->scan_begin_src &= TRIG_FOLLOW | TRIG_TIMER | TRIG_INT;
	}
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_INT | TRIG_EXT;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/* Step 2: make sure trigger sources are unique and mutually compatible
	 * "source conflict" returned by comedilib to user mode process
	 * if this fails. */

	/* these tests are true if more than one _src bit is set */
	if ((cmd->start_src & (cmd->start_src - 1)) != 0)
		err++;
	if ((cmd->scan_begin_src & (cmd->scan_begin_src - 1)) != 0)
		err++;
	if ((cmd->convert_src & (cmd->convert_src - 1)) != 0)
		err++;
	if ((cmd->scan_end_src & (cmd->scan_end_src - 1)) != 0)
		err++;
	if ((cmd->stop_src & (cmd->stop_src - 1)) != 0)
		err++;

	/* If scan_begin_src is not TRIG_FOLLOW, then a monostable will be
	 * set up to generate a fixed number of timed conversion pulses. */
	if ((cmd->scan_begin_src != TRIG_FOLLOW)
		&& (cmd->convert_src != TRIG_TIMER))
		err++;

	if (err)
		return 2;

	/* Step 3: make sure arguments are trivially compatible.
	 * "invalid argument" returned by comedilib to user mode process
	 * if this fails. */

	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		err++;
	}
#define MAX_SPEED_AI_SE		3200	/* PCI230 SE:   3200 ns => 312.5 kHz */
#define MAX_SPEED_AI_DIFF	8000	/* PCI230 DIFF: 8000 ns => 125 kHz */
#define MAX_SPEED_AI_PLUS	4000	/* PCI230+:     4000 ns => 250 kHz */
#define MIN_SPEED_AI	4294967295u	/* 4294967295ns = 4.29s */
			/*- Comedi limit due to unsigned int cmd.  Driver limit
			 * = 2^16 (16bit * counter) * 1000000ns (1kHz onboard
			 * clock) = 65.536s */

	if (cmd->convert_src == TRIG_TIMER) {
		unsigned int max_speed_ai;

		if (devpriv->hwver == 0) {
			/* PCI230 or PCI260.  Max speed depends whether
			 * single-ended or pseudo-differential. */
			if (cmd->chanlist && (cmd->chanlist_len > 0)) {
				/* Peek analogue reference of first channel. */
				if (CR_AREF(cmd->chanlist[0]) == AREF_DIFF) {
					max_speed_ai = MAX_SPEED_AI_DIFF;
				} else {
					max_speed_ai = MAX_SPEED_AI_SE;
				}
			} else {
				/* No channel list.  Assume single-ended. */
				max_speed_ai = MAX_SPEED_AI_SE;
			}
		} else {
			/* PCI230+ or PCI260+. */
			max_speed_ai = MAX_SPEED_AI_PLUS;
		}

		if (cmd->convert_arg < max_speed_ai) {
			cmd->convert_arg = max_speed_ai;
			err++;
		}
		if (cmd->convert_arg > MIN_SPEED_AI) {
			cmd->convert_arg = MIN_SPEED_AI;
			err++;
		}
	} else if (cmd->convert_src == TRIG_EXT) {
		/*
		 * external trigger
		 *
		 * convert_arg == (CR_EDGE | 0)
		 *                => trigger on +ve edge.
		 * convert_arg == (CR_EDGE | CR_INVERT | 0)
		 *                => trigger on -ve edge.
		 */
		if ((cmd->convert_arg & CR_FLAGS_MASK) != 0) {
			/* Trigger number must be 0. */
			if ((cmd->convert_arg & ~CR_FLAGS_MASK) != 0) {
				cmd->convert_arg = COMBINE(cmd->convert_arg, 0,
					~CR_FLAGS_MASK);
				err++;
			}
			/* The only flags allowed are CR_INVERT and CR_EDGE.
			 * CR_EDGE is required. */
			if ((cmd->convert_arg & (CR_FLAGS_MASK & ~CR_INVERT))
				!= CR_EDGE) {
				/* Set CR_EDGE, preserve CR_INVERT. */
				cmd->convert_arg =
					COMBINE(cmd->start_arg, (CR_EDGE | 0),
					CR_FLAGS_MASK & ~CR_INVERT);
				err++;
			}
		} else {
			/* Backwards compatibility with previous versions. */
			/* convert_arg == 0 => trigger on -ve edge. */
			/* convert_arg == 1 => trigger on +ve edge. */
			if (cmd->convert_arg > 1) {
				/* Default to trigger on +ve edge. */
				cmd->convert_arg = 1;
				err++;
			}
		}
	} else {
		if (cmd->convert_arg != 0) {
			cmd->convert_arg = 0;
			err++;
		}
	}

	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}

	if (cmd->stop_src == TRIG_NONE) {
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
	}

	if (cmd->scan_begin_src == TRIG_EXT) {
		/* external "trigger" to begin each scan
		 * scan_begin_arg==0 => use PPC0 input -> gate of CT0 -> gate
		 * of CT2 (sample convert trigger is CT2) */
		if ((cmd->scan_begin_arg & ~CR_FLAGS_MASK) != 0) {
			cmd->scan_begin_arg = COMBINE(cmd->scan_begin_arg, 0,
				~CR_FLAGS_MASK);
		}
		/* The only flag allowed is CR_EDGE, which is ignored. */
		if ((cmd->scan_begin_arg & CR_FLAGS_MASK & ~CR_EDGE) != 0) {
			cmd->scan_begin_arg = COMBINE(cmd->scan_begin_arg, 0,
				CR_FLAGS_MASK & ~CR_EDGE);
			err++;
		}
	} else if (cmd->scan_begin_src == TRIG_TIMER) {
		/* N.B. cmd->convert_arg is also TRIG_TIMER */
		if (!pci230_ai_check_scan_period(cmd)) {
			err++;
		}
	} else {
		if (cmd->scan_begin_arg != 0) {
			cmd->scan_begin_arg = 0;
			err++;
		}
	}

	if (err)
		return 3;

	/* Step 4: fix up any arguments.
	 * "argument conflict" returned by comedilib to user mode process
	 * if this fails. */

	if (cmd->convert_src == TRIG_TIMER) {
		tmp = cmd->convert_arg;
		pci230_ns_to_single_timer(&cmd->convert_arg,
			cmd->flags & TRIG_ROUND_MASK);
		if (tmp != cmd->convert_arg)
			err++;
	}

	if (cmd->scan_begin_src == TRIG_TIMER) {
		/* N.B. cmd->convert_arg is also TRIG_TIMER */
		tmp = cmd->scan_begin_arg;
		pci230_ns_to_single_timer(&cmd->scan_begin_arg,
			cmd->flags & TRIG_ROUND_MASK);
		if (!pci230_ai_check_scan_period(cmd)) {
			/* Was below minimum required.  Round up. */
			pci230_ns_to_single_timer(&cmd->scan_begin_arg,
				TRIG_ROUND_UP);
			pci230_ai_check_scan_period(cmd);
		}
		if (tmp != cmd->scan_begin_arg)
			err++;
	}

	if (err)
		return 4;

	/* Step 5: check channel list if it exists. */

	if (cmd->chanlist && cmd->chanlist_len > 0) {
		enum {
			seq_err = 1 << 0,
			rangepair_err = 1 << 1,
			polarity_err = 1 << 2,
			aref_err = 1 << 3,
			diffchan_err = 1 << 4,
			buggy_chan0_err = 1 << 5
		};
		unsigned int errors;
		unsigned int chan, prev_chan;
		unsigned int range, prev_range;
		unsigned int polarity, prev_polarity;
		unsigned int aref, prev_aref;
		unsigned int subseq_len;
		unsigned int n;

		subseq_len = 0;
		errors = 0;
		prev_chan = prev_aref = prev_range = prev_polarity = 0;
		for (n = 0; n < cmd->chanlist_len; n++) {
			chan = CR_CHAN(cmd->chanlist[n]);
			range = CR_RANGE(cmd->chanlist[n]);
			aref = CR_AREF(cmd->chanlist[n]);
			polarity = pci230_ai_bipolar[range];
			/* Only the first half of the channels are available if
			 * differential.  (These are remapped in software.  In
			 * hardware, only the even channels are available.) */
			if ((aref == AREF_DIFF)
				&& (chan >= (s->n_chan / 2))) {
				errors |= diffchan_err;
			}
			if (n > 0) {
				/* Channel numbers must strictly increase or
				 * subsequence must repeat exactly. */
				if ((chan <= prev_chan)
					&& (subseq_len == 0)) {
					subseq_len = n;
				}
				if ((subseq_len > 0)
					&& (cmd->chanlist[n] !=
						cmd->chanlist[n %
							subseq_len])) {
					errors |= seq_err;
				}
				/* Channels must have same AREF. */
				if (aref != prev_aref) {
					errors |= aref_err;
				}
				/* Channel ranges must have same polarity. */
				if (polarity != prev_polarity) {
					errors |= polarity_err;
				}
				/* Single-ended channel pairs must have same
				 * range.  */
				if ((aref != AREF_DIFF)
					&& (((chan ^ prev_chan) & ~1) == 0)
					&& (range != prev_range)) {
					errors |= rangepair_err;
				}
			}
			prev_chan = chan;
			prev_range = range;
			prev_aref = aref;
			prev_polarity = polarity;
		}
		if (subseq_len == 0) {
			/* Subsequence is whole sequence. */
			subseq_len = n;
		}
		/* If channel list is a repeating subsequence, need a whole
		 * number of repeats. */
		if ((n % subseq_len) != 0) {
			errors |= seq_err;
		}
		if ((devpriv->hwver > 0) && (devpriv->hwver < 4)) {
			/*
			 * Buggy PCI230+ or PCI260+ requires channel 0 to be
			 * (first) in the sequence if the sequence contains
			 * more than one channel.  Hardware versions 1 and 2
			 * have the bug.  There is no hardware version 3.
			 *
			 * Actually, there are two firmwares that report
			 * themselves as hardware version 1 (the boards
			 * have different ADC chips with slightly different
			 * timing requirements, which was supposed to be
			 * invisible to software).  The first one doesn't
			 * seem to have the bug, but the second one
			 * does, and we can't tell them apart!
			 */
			if ((subseq_len > 1)
				&& (CR_CHAN(cmd->chanlist[0]) != 0)) {
				errors |= buggy_chan0_err;
			}
		}
		if (errors != 0) {
			err++;
			if ((errors & seq_err) != 0) {
				DPRINTK("comedi%d: amplc_pci230: ai_cmdtest: "
					"channel numbers must increase or "
					"sequence must repeat exactly\n",
					dev->minor);
			}
			if ((errors & rangepair_err) != 0) {
				DPRINTK("comedi%d: amplc_pci230: ai_cmdtest: "
					"single-ended channel pairs must "
					"have the same range\n", dev->minor);
			}
			if ((errors & polarity_err) != 0) {
				DPRINTK("comedi%d: amplc_pci230: ai_cmdtest: "
					"channel sequence ranges must be all "
					"bipolar or all unipolar\n",
					dev->minor);
			}
			if ((errors & aref_err) != 0) {
				DPRINTK("comedi%d: amplc_pci230: ai_cmdtest: "
					"channel sequence analogue references "
					"must be all the same (single-ended "
					"or differential)\n", dev->minor);
			}
			if ((errors & diffchan_err) != 0) {
				DPRINTK("comedi%d: amplc_pci230: ai_cmdtest: "
					"differential channel number out of "
					"range 0 to %u\n", dev->minor,
					(s->n_chan / 2) - 1);
			}
			if ((errors & buggy_chan0_err) != 0) {
				/* Use printk instead of DPRINTK here. */
				printk("comedi: comedi%d: amplc_pci230: "
					"ai_cmdtest: Buggy PCI230+/260+ "
					"h/w version %u requires first channel "
					"of multi-channel sequence to be 0 "
					"(corrected in h/w version 4)\n",
					dev->minor, devpriv->hwver);
			}
		}
	}

	if (err)
		return 5;

	return 0;
}

static void pci230_ai_update_fifo_trigger_level(comedi_device * dev,
	comedi_subdevice * s)
{
	comedi_cmd *cmd = &s->async->cmd;
	unsigned int scanlen = cmd->scan_end_arg;
	unsigned int wake;
	unsigned short triglev;
	unsigned short adccon;

	if ((cmd->flags & TRIG_WAKE_EOS) != 0) {
		/* Wake at end of scan. */
		wake = scanlen - devpriv->ai_scan_pos;
	} else {
		if (devpriv->ai_continuous
			|| (devpriv->ai_scan_count
				>= PCI230_ADC_FIFOLEVEL_HALFFULL)
			|| (scanlen >= PCI230_ADC_FIFOLEVEL_HALFFULL)) {
			wake = PCI230_ADC_FIFOLEVEL_HALFFULL;
		} else {
			wake = (devpriv->ai_scan_count * scanlen)
				- devpriv->ai_scan_pos;
		}
	}
	if (wake >= PCI230_ADC_FIFOLEVEL_HALFFULL) {
		triglev = PCI230_ADC_INT_FIFO_HALF;
	} else {
		triglev = PCI230_ADC_INT_FIFO_NEMPTY;
	}
	adccon = (devpriv->adccon & ~PCI230_ADC_INT_FIFO_MASK) | triglev;
	if (adccon != devpriv->adccon) {
		devpriv->adccon = adccon;
		outw(adccon, dev->iobase + PCI230_ADCCON);
	}
}

static int pci230_ai_inttrig_convert(comedi_device * dev, comedi_subdevice * s,
	unsigned int trig_num)
{
	unsigned long irqflags;

	if (trig_num != 0)
		return -EINVAL;

	comedi_spin_lock_irqsave(&devpriv->ai_inttrig_spinlock, irqflags);
	if (s->async->inttrig) {
		comedi_spin_unlock_irqrestore(&devpriv->ai_inttrig_spinlock,
			irqflags);
		/* Trigger conversion. */
		outw(PCI230_ADC_CONV, dev->iobase + PCI230_ADCSWTRIG);
	} else {
		comedi_spin_unlock_irqrestore(&devpriv->ai_inttrig_spinlock,
			irqflags);
	}

	return 1;
}

static int pci230_ai_inttrig_scan_begin(comedi_device * dev,
	comedi_subdevice * s, unsigned int trig_num)
{
	unsigned long irqflags;
	unsigned char zgat;

	if (trig_num != 0)
		return -EINVAL;

	comedi_spin_lock_irqsave(&devpriv->ai_inttrig_spinlock, irqflags);
	if (s->async->inttrig) {
		comedi_spin_unlock_irqrestore(&devpriv->ai_inttrig_spinlock,
			irqflags);
		/* Trigger scan by waggling CT0 gate source. */
		zgat = GAT_CONFIG(0, GAT_GND);
		outb(zgat, devpriv->iobase1 + PCI230_ZGAT_SCE);
		zgat = GAT_CONFIG(0, GAT_VCC);
		outb(zgat, devpriv->iobase1 + PCI230_ZGAT_SCE);
	} else {
		comedi_spin_unlock_irqrestore(&devpriv->ai_inttrig_spinlock,
			irqflags);
	}

	return 1;
}

static void pci230_ai_start(comedi_device * dev, comedi_subdevice * s)
{
	unsigned long irqflags;
	unsigned short conv;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;

	if (!devpriv->ai_continuous && (devpriv->ai_scan_count == 0)) {
		/* An empty acquisition! */
		async->events |= COMEDI_CB_EOA;
		pci230_ai_stop(dev, s);
		comedi_event(dev, s);
	} else {
		/* Enable ADC FIFO trigger level interrupt. */
		comedi_spin_lock_irqsave(&devpriv->isr_spinlock, irqflags);
		devpriv->int_en |= PCI230_INT_ADC;
		devpriv->ier |= PCI230_INT_ADC;
		outb(devpriv->ier, devpriv->iobase1 + PCI230_INT_SCE);
		comedi_spin_unlock_irqrestore(&devpriv->isr_spinlock, irqflags);

		/* Update conversion trigger source which is currently set
		 * to CT2 output, which is currently stuck high. */
		switch (cmd->convert_src) {
		default:
			conv = PCI230_ADC_TRIG_NONE;
			break;
		case TRIG_TIMER:
			/* Using CT2 output. */
			conv = PCI230_ADC_TRIG_Z2CT2;
			break;
		case TRIG_EXT:
			if ((cmd->convert_arg & CR_EDGE) != 0) {
				if ((cmd->convert_arg & CR_INVERT) == 0) {
					/* Trigger on +ve edge. */
					conv = PCI230_ADC_TRIG_EXTP;
				} else {
					/* Trigger on -ve edge. */
					conv = PCI230_ADC_TRIG_EXTN;
				}
			} else {
				/* Backwards compatibility. */
				if (cmd->convert_arg != 0) {
					/* Trigger on +ve edge. */
					conv = PCI230_ADC_TRIG_EXTP;
				} else {
					/* Trigger on -ve edge. */
					conv = PCI230_ADC_TRIG_EXTN;
				}
			}
			break;
		case TRIG_INT:
			conv = PCI230_ADC_TRIG_SW;
			comedi_spin_lock_irqsave(&devpriv->ai_inttrig_spinlock,
				irqflags);
			async->inttrig = pci230_ai_inttrig_convert;
			/* ai_inttrig_spinlock is unlocked after ADCCON is
			 * written below. */
			break;
		}
		devpriv->adccon = (devpriv->adccon & ~PCI230_ADC_TRIG_MASK)
			| conv;
		outw(devpriv->adccon, dev->iobase + PCI230_ADCCON);
		if (cmd->convert_src == TRIG_INT) {
			comedi_spin_unlock_irqrestore(&devpriv->
				ai_inttrig_spinlock, irqflags);
		}
		/* Update FIFO interrupt trigger level, which is currently
		 * set to "full".  */
		pci230_ai_update_fifo_trigger_level(dev, s);
		if (cmd->convert_src == TRIG_TIMER) {
			/* Update timer gates. */
			unsigned char zgat;

			if (cmd->scan_begin_src != TRIG_FOLLOW) {
				/* Conversion timer CT2 needs to be gated by
				 * inverted output of monostable CT2. */
				zgat = GAT_CONFIG(2, GAT_NOUTNM2);
			} else {
				/* Conversion timer CT2 needs to be gated on
				 * continuously. */
				zgat = GAT_CONFIG(2, GAT_VCC);
			}
			outb(zgat, devpriv->iobase1 + PCI230_ZGAT_SCE);
			if (cmd->scan_begin_src != TRIG_FOLLOW) {
				/* Set monostable CT0 trigger source. */
				switch (cmd->scan_begin_src) {
				default:
					zgat = GAT_CONFIG(0, GAT_VCC);
					break;
				case TRIG_EXT:
					/*
					 * For CT0 on PCI230, the external
					 * trigger (gate) signal comes from
					 * PPC0, which is channel 16 of the DIO
					 * subdevice.  The application needs to
					 * configure this as an input in order
					 * to use it as an external scan
					 * trigger.
					 */
					zgat = GAT_CONFIG(0, GAT_EXT);
					break;
				case TRIG_TIMER:
					/*
					 * Monostable CT0 triggered by rising
					 * edge on inverted output of CT1
					 * (falling edge on CT1).
					 */
					zgat = GAT_CONFIG(0, GAT_NOUTNM2);
					break;
				case TRIG_INT:
					/*
					 * Monostable CT0 is triggered by
					 * inttrig function waggling the CT0
					 * gate source.
					 */
					zgat = GAT_CONFIG(0, GAT_VCC);
					break;
				}
				outb(zgat, devpriv->iobase1 + PCI230_ZGAT_SCE);
				switch (cmd->scan_begin_src) {
				case TRIG_TIMER:
					/* Scan period timer CT1 needs to be
					 * gated on to start counting. */
					zgat = GAT_CONFIG(1, GAT_VCC);
					outb(zgat, devpriv->iobase1
						+ PCI230_ZGAT_SCE);
					break;
				case TRIG_INT:
					comedi_spin_lock_irqsave(&devpriv->
						ai_inttrig_spinlock, irqflags);
					async->inttrig =
						pci230_ai_inttrig_scan_begin;
					comedi_spin_unlock_irqrestore(&devpriv->
						ai_inttrig_spinlock, irqflags);
					break;
				}
			}
		} else {
			/* No longer need Z2-CT2. */
			put_one_resource(dev, RES_Z2CT2, OWNER_AICMD);
		}
	}
}

static int pci230_ai_inttrig_start(comedi_device * dev, comedi_subdevice * s,
	unsigned int trig_num)
{
	unsigned long irqflags;

	if (trig_num != 0)
		return -EINVAL;

	comedi_spin_lock_irqsave(&devpriv->ai_inttrig_spinlock, irqflags);
	if (s->async->inttrig) {
		s->async->inttrig = NULLFUNC;
		comedi_spin_unlock_irqrestore(&devpriv->ai_inttrig_spinlock,
			irqflags);
		pci230_ai_start(dev, s);
	} else {
		comedi_spin_unlock_irqrestore(&devpriv->ai_inttrig_spinlock,
			irqflags);
	}

	return 1;
}

static int pci230_ai_cmd(comedi_device * dev, comedi_subdevice * s)
{
	unsigned int i, chan, range, diff;
	unsigned int res_mask;
	unsigned short adccon, adcen;
	unsigned char zgat;

	/* Get the command. */
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;

	/*
	 * Determine which shared resources are needed.
	 */
	res_mask = 0;
	/* Need Z2-CT2 to supply a conversion trigger source at a high
	 * logic level, even if not doing timed conversions. */
	res_mask |= (1U << RES_Z2CT2);
	if (cmd->scan_begin_src != TRIG_FOLLOW) {
		/* Using Z2-CT0 monostable to gate Z2-CT2 conversion timer */
		res_mask |= (1U << RES_Z2CT0);
		if (cmd->scan_begin_src == TRIG_TIMER) {
			/* Using Z2-CT1 for scan frequency */
			res_mask |= (1U << RES_Z2CT1);
		}
	}
	/* Claim resources. */
	if (!get_resources(dev, res_mask, OWNER_AICMD)) {
		return -EBUSY;
	}

	/* Get number of scans required. */
	if (cmd->stop_src == TRIG_COUNT) {
		devpriv->ai_scan_count = cmd->stop_arg;
		devpriv->ai_continuous = 0;
	} else {
		/* TRIG_NONE, user calls cancel. */
		devpriv->ai_scan_count = 0;
		devpriv->ai_continuous = 1;
	}
	devpriv->ai_scan_pos = 0;	/* Position within scan. */

	/* Steps;
	 * - Set channel scan list.
	 * - Set channel gains.
	 * - Enable and reset FIFO, specify uni/bip, se/diff, and set
	 *   start conversion source to point to something at a high logic
	 *   level (we use the output of counter/timer 2 for this purpose.
	 * - PAUSE to allow things to settle down.
	 * - Reset the FIFO again because it needs resetting twice and there
	 *   may have been a false conversion trigger on some versions of
	 *   PCI230/260 due to the start conversion source being set to a
	 *   high logic level.
	 * - Enable ADC FIFO level interrupt.
	 * - Set actual conversion trigger source and FIFO interrupt trigger
	 *   level.
	 * - If convert_src is TRIG_TIMER, set up the timers.
	 */

	adccon = PCI230_ADC_FIFO_EN;
	adcen = 0;

	if (CR_AREF(cmd->chanlist[0]) == AREF_DIFF) {
		/* Differential - all channels must be differential. */
		diff = 1;
		adccon |= PCI230_ADC_IM_DIF;
	} else {
		/* Single ended - all channels must be single-ended. */
		diff = 0;
		adccon |= PCI230_ADC_IM_SE;
	}

	range = CR_RANGE(cmd->chanlist[0]);
	devpriv->ai_bipolar = pci230_ai_bipolar[range];
	if (devpriv->ai_bipolar) {
		adccon |= PCI230_ADC_IR_BIP;
	} else {
		adccon |= PCI230_ADC_IR_UNI;
	}
	for (i = 0; i < cmd->chanlist_len; i++) {
		unsigned int gainshift;

		chan = CR_CHAN(cmd->chanlist[i]);
		range = CR_RANGE(cmd->chanlist[i]);
		if (diff) {
			gainshift = 2 * chan;
			if (devpriv->hwver == 0) {
				/* Original PCI230/260 expects both inputs of
				 * the differential channel to be enabled. */
				adcen |= 3 << gainshift;
			} else {
				/* PCI230+/260+ expects only one input of the
				 * differential channel to be enabled. */
				adcen |= 1 << gainshift;
			}
		} else {
			gainshift = (chan & ~1);
			adcen |= 1 << chan;
		}
		devpriv->adcg = (devpriv->adcg & ~(3 << gainshift))
			| (pci230_ai_gain[range] << gainshift);
	}

	/* Set channel scan list. */
	outw(adcen, dev->iobase + PCI230_ADCEN);

	/* Set channel gains. */
	outw(devpriv->adcg, dev->iobase + PCI230_ADCG);

	/* Set counter/timer 2 output high for use as the initial start
	 * conversion source. */
	i8254_set_mode(devpriv->iobase1 + PCI230_Z2_CT_BASE, 0, 2, I8254_MODE1);

	/* Temporarily use CT2 output as conversion trigger source and
	 * temporarily set FIFO interrupt trigger level to 'full'. */
	adccon |= PCI230_ADC_INT_FIFO_FULL | PCI230_ADC_TRIG_Z2CT2;

	/* Enable and reset FIFO, specify FIFO trigger level full, specify
	 * uni/bip, se/diff, and temporarily set the start conversion source
	 * to CT2 output.  Note that CT2 output is currently high, and this
	 * will produce a false conversion trigger on some versions of the
	 * PCI230/260, but that will be dealt with later. */
	devpriv->adccon = adccon;
	outw(adccon | PCI230_ADC_FIFO_RESET, dev->iobase + PCI230_ADCCON);

	/* Delay */
	/* Failure to include this will result in the first few channels'-worth
	 * of data being corrupt, normally manifesting itself by large negative
	 * voltages. It seems the board needs time to settle between the first
	 * FIFO reset (above) and the second FIFO reset (below). Setting the
	 * channel gains and scan list _before_ the first FIFO reset also
	 * helps, though only slightly. */
	comedi_udelay(25);

	/* Reset FIFO again. */
	outw(adccon | PCI230_ADC_FIFO_RESET, dev->iobase + PCI230_ADCCON);

	if (cmd->convert_src == TRIG_TIMER) {
		/* Set up CT2 as conversion timer, but gate it off for now.
		 * Note, counter/timer output 2 can be monitored on the
		 * connector: PCI230 pin 21, PCI260 pin 18. */
		zgat = GAT_CONFIG(2, GAT_GND);
		outb(zgat, devpriv->iobase1 + PCI230_ZGAT_SCE);
		/* Set counter/timer 2 to the specified conversion period. */
		pci230_ct_setup_ns_mode(dev, 2, I8254_MODE3, cmd->convert_arg,
			cmd->flags & TRIG_ROUND_MASK);
		if (cmd->scan_begin_src != TRIG_FOLLOW) {
			/*
			 * Set up monostable on CT0 output for scan timing.  A
			 * rising edge on the trigger (gate) input of CT0 will
			 * trigger the monostable, causing its output to go low
			 * for the configured period.  The period depends on
			 * the conversion period and the number of conversions
			 * in the scan.
			 *
			 * Set the trigger high before setting up the
			 * monostable to stop it triggering.  The trigger
			 * source will be changed later.
			 */
			zgat = GAT_CONFIG(0, GAT_VCC);
			outb(zgat, devpriv->iobase1 + PCI230_ZGAT_SCE);
			pci230_ct_setup_ns_mode(dev, 0, I8254_MODE1,
				((uint64_t) cmd->convert_arg
					* cmd->scan_end_arg), TRIG_ROUND_UP);
			if (cmd->scan_begin_src == TRIG_TIMER) {
				/*
				 * Monostable on CT0 will be triggered by
				 * output of CT1 at configured scan frequency.
				 *
				 * Set up CT1 but gate it off for now.
				 */
				zgat = GAT_CONFIG(1, GAT_GND);
				outb(zgat, devpriv->iobase1 + PCI230_ZGAT_SCE);
				pci230_ct_setup_ns_mode(dev, 1, I8254_MODE3,
					cmd->scan_begin_arg,
					cmd->flags & TRIG_ROUND_MASK);
			}
		}
	}

	if (cmd->start_src == TRIG_INT) {
		unsigned long irqflags;

		comedi_spin_lock_irqsave(&devpriv->ai_inttrig_spinlock,
			irqflags);
		s->async->inttrig = pci230_ai_inttrig_start;
		comedi_spin_unlock_irqrestore(&devpriv->ai_inttrig_spinlock,
			irqflags);

	} else {
		/* TRIG_NOW */
		pci230_ai_start(dev, s);
	}

	return 0;
}

static unsigned int divide_ns(uint64_t ns, unsigned int timebase,
	unsigned int round_mode)
{
	uint64_t div;
	unsigned int rem;

	div = ns;
	rem = do_div(div, timebase);
	round_mode &= TRIG_ROUND_MASK;
	switch (round_mode) {
	default:
	case TRIG_ROUND_NEAREST:
		div += (rem + (timebase / 2)) / timebase;
		break;
	case TRIG_ROUND_DOWN:
		break;
	case TRIG_ROUND_UP:
		div += (rem + timebase - 1) / timebase;
		break;
	}
	return div > UINT_MAX ? UINT_MAX : (unsigned int)div;
}

/* Given desired period in ns, returns the required internal clock source
 * and gets the initial count. */
static unsigned int pci230_choose_clk_count(uint64_t ns, unsigned int *count,
	unsigned int round_mode)
{
	unsigned int clk_src, cnt;

	for (clk_src = CLK_10MHZ;; clk_src++) {
		cnt = divide_ns(ns, pci230_timebase[clk_src], round_mode);
		if ((cnt <= 65536) || (clk_src == CLK_1KHZ)) {
			break;
		}
	}
	*count = cnt;
	return clk_src;
}

static void pci230_ns_to_single_timer(unsigned int *ns, unsigned int round)
{
	unsigned int count;
	unsigned int clk_src;

	clk_src = pci230_choose_clk_count(*ns, &count, round);
	*ns = count * pci230_timebase[clk_src];
	return;
}

static void pci230_ct_setup_ns_mode(comedi_device * dev, unsigned int ct,
	unsigned int mode, uint64_t ns, unsigned int round)
{
	unsigned int clk_src;
	unsigned int count;

	/* Set mode. */
	i8254_set_mode(devpriv->iobase1 + PCI230_Z2_CT_BASE, 0, ct, mode);
	/* Determine clock source and count. */
	clk_src = pci230_choose_clk_count(ns, &count, round);
	/* Program clock source. */
	outb(CLK_CONFIG(ct, clk_src), devpriv->iobase1 + PCI230_ZCLK_SCE);
	/* Set initial count. */
	if (count >= 65536) {
		count = 0;
	}
	i8254_write(devpriv->iobase1 + PCI230_Z2_CT_BASE, 0, ct, count);
}

static void pci230_cancel_ct(comedi_device * dev, unsigned int ct)
{
	i8254_load(devpriv->iobase1 + PCI230_Z2_CT_BASE, 0, ct, 0, 0);
	/* Counter ct, divisor 0, 8254 mode 0. */
}

/* Interrupt handler */
static irqreturn_t pci230_interrupt(int irq, void *d PT_REGS_ARG)
{
	unsigned char status_int, valid_status_int;
	comedi_device *dev = (comedi_device *) d;
	comedi_subdevice *s;
	unsigned long irqflags;

	/* Read interrupt status/enable register. */
	status_int = inb(devpriv->iobase1 + PCI230_INT_STAT);

	if (status_int == PCI230_INT_DISABLE) {
		return IRQ_NONE;
	}

	comedi_spin_lock_irqsave(&devpriv->isr_spinlock, irqflags);
	valid_status_int = devpriv->int_en & status_int;
	/* Disable triggered interrupts.
	 * (Only those interrupts that need re-enabling, are, later in the
	 * handler).  */
	devpriv->ier = devpriv->int_en & ~status_int;
	outb(devpriv->ier, devpriv->iobase1 + PCI230_INT_SCE);
	devpriv->intr_running = 1;
	devpriv->intr_cpuid = THISCPU;
	comedi_spin_unlock_irqrestore(&devpriv->isr_spinlock, irqflags);

	/*
	 * Check the source of interrupt and handle it.
	 * The PCI230 can cope with concurrent ADC, DAC, PPI C0 and C3
	 * interrupts.  However, at present (Comedi-0.7.60) does not allow
	 * concurrent execution of commands, instructions or a mixture of the
	 * two.
	 */

	if ((valid_status_int & PCI230_INT_ZCLK_CT1) != 0) {
		s = dev->write_subdev;
		pci230_handle_ao(dev, s);
		comedi_event(dev, s);
	}

	if ((valid_status_int & PCI230_INT_ADC) != 0) {
		s = dev->read_subdev;
		pci230_handle_ai(dev, s);
		comedi_event(dev, s);
	}

	/* Reenable interrupts. */
	comedi_spin_lock_irqsave(&devpriv->isr_spinlock, irqflags);
	if (devpriv->ier != devpriv->int_en) {
		devpriv->ier = devpriv->int_en;
		outb(devpriv->ier, devpriv->iobase1 + PCI230_INT_SCE);
	}
	devpriv->intr_running = 0;
	comedi_spin_unlock_irqrestore(&devpriv->isr_spinlock, irqflags);

	return IRQ_HANDLED;
}

static void pci230_handle_ao(comedi_device * dev, comedi_subdevice * s)
{
	sampl_t data;
	int i, ret;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;

	for (i = 0; i < cmd->chanlist_len; i++) {
		/* Read sample from Comedi's circular buffer. */
		ret = comedi_buf_get(s->async, &data);
		if (ret == 0) {
			s->async->events |= COMEDI_CB_OVERFLOW;
			pci230_ao_stop(dev, s);
			comedi_error(dev, "AO buffer underrun");
			return;
		}
		/* Write value to DAC. */
		pci230_ao_write(dev, data, CR_CHAN(cmd->chanlist[i]));
	}

	async->events |= COMEDI_CB_BLOCK | COMEDI_CB_EOS;
	if (!devpriv->ao_continuous) {
		devpriv->ao_scan_count--;
		if (devpriv->ao_scan_count == 0) {
			/* End of acquisition. */
			async->events |= COMEDI_CB_EOA;
			pci230_ao_stop(dev, s);
		}
	}
}

static void pci230_handle_ai(comedi_device * dev, comedi_subdevice * s)
{
	unsigned int events = 0;
	unsigned int status_fifo;
	unsigned int i;
	unsigned int todo;
	unsigned int fifoamount;
	comedi_async *async = s->async;
	unsigned int scanlen = async->cmd.scan_end_arg;

	/* Determine number of samples to read. */
	if (devpriv->ai_continuous) {
		todo = PCI230_ADC_FIFOLEVEL_HALFFULL;
	} else if (devpriv->ai_scan_count == 0) {
		todo = 0;
	} else if ((devpriv->ai_scan_count > PCI230_ADC_FIFOLEVEL_HALFFULL)
		|| (scanlen > PCI230_ADC_FIFOLEVEL_HALFFULL)) {
		todo = PCI230_ADC_FIFOLEVEL_HALFFULL;
	} else {
		todo = (devpriv->ai_scan_count * scanlen)
			- devpriv->ai_scan_pos;
		if (todo > PCI230_ADC_FIFOLEVEL_HALFFULL) {
			todo = PCI230_ADC_FIFOLEVEL_HALFFULL;
		}
	}

	fifoamount = 0;
	for (i = 0; i < todo; i++) {
		if (fifoamount == 0) {
			/* Read FIFO state. */
			status_fifo = inw(dev->iobase + PCI230_ADCCON);

			if ((status_fifo & PCI230_ADC_FIFO_FULL_LATCHED) != 0) {
				/* Report error otherwise FIFO overruns will go
				 * unnoticed by the caller. */
				comedi_error(dev, "AI FIFO overrun");
				events |= COMEDI_CB_OVERFLOW | COMEDI_CB_ERROR;
				break;
			} else if ((status_fifo & PCI230_ADC_FIFO_EMPTY) != 0) {
				/* FIFO empty. */
				break;
			} else if ((status_fifo & PCI230_ADC_FIFO_HALF) != 0) {
				/* FIFO half full. */
				fifoamount = PCI230_ADC_FIFOLEVEL_HALFFULL;
			} else {
				/* FIFO not empty. */
				fifoamount = 1;
			}
		}

		/* Read sample and store in Comedi's circular buffer. */
		if (comedi_buf_put(async, pci230_ai_read(dev)) == 0) {
			events |= COMEDI_CB_ERROR | COMEDI_CB_OVERFLOW;
			comedi_error(dev, "AI buffer overflow");
			break;
		}
		fifoamount--;
		devpriv->ai_scan_pos++;
		if (devpriv->ai_scan_pos == scanlen) {
			/* End of scan. */
			devpriv->ai_scan_pos = 0;
			devpriv->ai_scan_count--;
			async->events |= COMEDI_CB_EOS;
		}
	}

	if (!devpriv->ai_continuous && (devpriv->ai_scan_count == 0)) {
		/* End of acquisition. */
		events |= COMEDI_CB_EOA;
	} else {
		/* More samples required, tell Comedi to block. */
		events |= COMEDI_CB_BLOCK;
	}
	async->events |= events;

	if ((async->events & (COMEDI_CB_EOA | COMEDI_CB_ERROR |
				COMEDI_CB_OVERFLOW)) != 0) {
		/* disable hardware conversions */
		pci230_ai_stop(dev, s);
	} else {
		/* update FIFO interrupt trigger level */
		pci230_ai_update_fifo_trigger_level(dev, s);
	}
}

static void pci230_ao_stop(comedi_device * dev, comedi_subdevice * s)
{
	unsigned long irqflags;

	/* Stop counter/timer. */
	pci230_cancel_ct(dev, 1);

	comedi_spin_lock_irqsave(&devpriv->ao_inttrig_spinlock, irqflags);
	/* Disable internal trigger. */
	s->async->inttrig = NULLFUNC;
	comedi_spin_unlock_irqrestore(&devpriv->ao_inttrig_spinlock, irqflags);
	/* Disable interrupt and wait for interrupt routine to finish running
	 * unless we are called from the interrupt routine. */
	comedi_spin_lock_irqsave(&devpriv->isr_spinlock, irqflags);
	devpriv->int_en &= ~PCI230_INT_ZCLK_CT1;	/* Disable interrupt. */
	while (devpriv->intr_running && devpriv->intr_cpuid != THISCPU) {
		comedi_spin_unlock_irqrestore(&devpriv->isr_spinlock, irqflags);
		comedi_spin_lock_irqsave(&devpriv->isr_spinlock, irqflags);
	}
	if (devpriv->ier != devpriv->int_en) {
		devpriv->ier = devpriv->int_en;
		outb(devpriv->ier, devpriv->iobase1 + PCI230_INT_SCE);
	}
	comedi_spin_unlock_irqrestore(&devpriv->isr_spinlock, irqflags);

	/* Release Z2-CT1. */
	put_one_resource(dev, RES_Z2CT1, OWNER_AOCMD);

	/* No longer running AO command. */
	devpriv->ao_scan_count = 0;
	devpriv->ao_continuous = 0;
}

static int pci230_ao_cancel(comedi_device * dev, comedi_subdevice * s)
{
	pci230_ao_stop(dev, s);
	return 0;
}

static void pci230_ai_stop(comedi_device * dev, comedi_subdevice * s)
{
	unsigned long irqflags;
	comedi_cmd *cmd = &s->async->cmd;

	if (cmd->convert_src == TRIG_TIMER) {
		/* Stop conversion rate generator. */
		pci230_cancel_ct(dev, 2);
	}
	if (cmd->scan_begin_src != TRIG_FOLLOW) {
		/* Stop scan period monostable. */
		pci230_cancel_ct(dev, 0);
	}

	comedi_spin_lock_irqsave(&devpriv->ai_inttrig_spinlock, irqflags);
	/* Disable internal trigger. */
	s->async->inttrig = NULLFUNC;
	comedi_spin_unlock_irqrestore(&devpriv->ai_inttrig_spinlock, irqflags);
	comedi_spin_lock_irqsave(&devpriv->isr_spinlock, irqflags);
	/* Disable ADC interrupt and wait for interrupt routine to finish
	 * running unless we are called from the interrupt routine. */
	devpriv->int_en &= ~PCI230_INT_ADC;
	while (devpriv->intr_running && devpriv->intr_cpuid != THISCPU) {
		comedi_spin_unlock_irqrestore(&devpriv->isr_spinlock, irqflags);
		comedi_spin_lock_irqsave(&devpriv->isr_spinlock, irqflags);
	}
	if (devpriv->ier != devpriv->int_en) {
		devpriv->ier = devpriv->int_en;
		outb(devpriv->ier, devpriv->iobase1 + PCI230_INT_SCE);
	}
	comedi_spin_unlock_irqrestore(&devpriv->isr_spinlock, irqflags);

	/* Reset FIFO, disable FIFO and set start conversion source to none.
	 * Keep se/diff and bip/uni settings */
	devpriv->adccon = (devpriv->adccon & (PCI230_ADC_IR_MASK
			| PCI230_ADC_IM_MASK)) | PCI230_ADC_TRIG_NONE;
	outw(devpriv->adccon | PCI230_ADC_FIFO_RESET,
		dev->iobase + PCI230_ADCCON);

	/* Release resources. */
	put_all_resources(dev, OWNER_AICMD);

	devpriv->ai_scan_count = 0;
	devpriv->ai_scan_pos = 0;
	devpriv->ai_continuous = 0;
}

static int pci230_ai_cancel(comedi_device * dev, comedi_subdevice * s)
{
	pci230_ai_stop(dev, s);
	return 0;
}
