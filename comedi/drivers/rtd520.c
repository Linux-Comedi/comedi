/*
    comedi/drivers/rtd520.c
    Comedi driver for Real Time Devices (RTD) PCI4520/DM7520
    
    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2001 David A. Schleef <ds@schleef.org>

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
    Created by Dan Christian, NASA Ames Research Center.

    The PCI4520 is a PCI card.  The DM7520 is a PC/104-plus card.
    Both have:
    8/16 12 bit ADC with FIFO and channel gain table
    8 bits high speed digital out (for external MUX) (or 8 in or 8 out)
    8 bits high speed digital in with FIFO and interrupt on change (or 8 IO)
    2 12 bit DACs with FIFOs
    2 bits output
    2 bits input
    bus mastering DMA
    timers: ADC sample, pacer, burst, about, delay, DA1, DA2
    sample counter
    3 user timer/counters (8254)
    external interrupt

    The DM7520 has slightly fewer features (fewer gain steps).

    These boards can support external multiplexors and multi-board
    synchronization, but this driver doesn't support that.

    Board docs: http://www.rtdusa.com/dm7520.htm
    Data sheet: http://www.rtdusa.com/pdf/dm7520.pdf
    Example source: http://www.rtdusa.com/examples/dm/dm7520.zip
    Call them and ask for the register level manual.
    PCI chip: http://www.plxtech.com/products/toolbox/9080.htm

    Notes:
    This board is (almost) completely memory mapped.

    I use a pretty loose naming style within the driver (rtd_blah).
    All externally visible names should be rtd520_blah.
    I use camelCase in and for structures.
    I may also use upper CamelCase for function names (old habit).

    This board is somewhat related to the PCI4400 board.  

    I borrowed heavily from the ni_mio_common, ni_atmio16d, and das1800,
    since they have the best documented code.

*/

/*
  driver status:

  Analog in supports instruction and command mode.  I can do 400Khz
  mutli-channel sampling on 400Mhz K6-2 with 58% idle.

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


/*======================================================================
  Board specific stuff
======================================================================*/

/* registers  */
#define PCI_VENDOR_ID_RTD	0x1435
/*
  The board has three memory windows: las0, las1, and lcfg (the PCI chip)
  Las1 has the data and can be burst DMAed 32bits at a time.
*/
#define LCFG_PCIINDEX	0
/* PCI region 1 is a 256 byte IO space mapping.  Use??? */
#define LAS0_PCIINDEX	2		/* PCI memory resources */
#define LAS1_PCIINDEX	3
#define LCFG_PCISIZE	0x100
#define LAS0_PCISIZE	0x200
#define LAS1_PCISIZE	0x10

#define RTD_CLOCK_RATE	8000000		/* 8Mhz onboard clock */
#define RTD_CLOCK_BASE	125		/* clock period in ns */

#define RTD_MAX_SPEED	1600		/* in nanoseconds */
#define RTD_MIN_SPEED	1000000000	/* in nanoseconds ??? */
#define RTD_SETTLE_DELAY	1	/* in usec */
#define RTD_ADC_TIMEOUT	1000		/* in usec */
#define RTD_DAC_DELAY	1		/* in usec */

#include "rtd520.h"

/*======================================================================
  Comedi specific stuff
======================================================================*/

/*
  The board has 3 input modes and the gains of 1,2,4,...32 (, 64, 128)
*/
static comedi_lrange rtd_ai_7520_range = { 6, {
    BIP_RANGE(5.0),
    BIP_RANGE(5.0/2),
    BIP_RANGE(5.0/4),
    BIP_RANGE(5.0/8),
    BIP_RANGE(5.0/16),
    BIP_RANGE(5.0/32),
#if 0					/* until we can handle 10V mode */
    UNI_RANGE(10.0),
    UNI_RANGE(10.0/2),
    UNI_RANGE(10.0/4),
    UNI_RANGE(10.0/8),
    UNI_RANGE(10.0/16),
    UNI_RANGE(10.0/32),
#endif
}};

/* PCI4520 has two more gains (6 more entries) */
static comedi_lrange rtd_ai_4520_range = { 8, {
    BIP_RANGE(5.0),
    BIP_RANGE(5.0/2),
    BIP_RANGE(5.0/4),
    BIP_RANGE(5.0/8),
    BIP_RANGE(5.0/16),
    BIP_RANGE(5.0/32),
    BIP_RANGE(5.0/64),
    BIP_RANGE(5.0/128),
#if 0					/* until we can handle 10V mode */
    UNI_RANGE(10.0),
    UNI_RANGE(10.0/2),
    UNI_RANGE(10.0/4),
    UNI_RANGE(10.0/8),
    UNI_RANGE(10.0/16),
    UNI_RANGE(10.0/32),
    UNI_RANGE(10.0/64),
    UNI_RANGE(10.0/128),
#endif
}};

/* Table order matches range values */
static comedi_lrange rtd_ao_range = { 4, {
    RANGE(0, 5),
    RANGE(0, 10),
    RANGE(-5, 5),
    RANGE(-10, 10),
}};

/*
  Board descriptions
 */
typedef struct rtdBoard_struct{
    char	*name;			/* must be first */
    int		device_id;
    int		aiChans;
    int		aiBits;
    int		aiMaxGain;
    int		fifoLen;
} rtdBoard;

static rtdBoard rtd520Boards[] = {
    {
	name:		"DM7520",
	device_id:	0x7520,
	aiChans:	16,
	aiBits:		12,
	aiMaxGain:	32,
	fifoLen:	1024,
    },
    {
	name:		"DM7520-8",
	device_id:	0x7520,
	aiChans:	16,
	aiBits:		12,
	aiMaxGain:	32,
	fifoLen:	8192,
    },
    {
	name:		"PCI4520",
	device_id:	0x4520,
	aiChans:	16,
	aiBits:		12,
	aiMaxGain:	128,
	fifoLen:	1024,
    },
    {
	name:		"PCI4520-8",
	device_id:	0x4520,
	aiChans:	16,
	aiBits:		12,
	aiMaxGain:	128,
	fifoLen:	8192,
    },
};

static struct pci_device_id rtd520_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_RTD, 0x7520, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_RTD, 0x4520, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, rtd520_pci_table);

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((rtdBoard *)dev->board_ptr)

/*
   This structure is for data unique to this hardware driver.
   This is also unique for each board in the system.
*/
typedef struct{
					/* memory mapped board structures */
    void	*las0;
    void	*las1;
    void	*lcfg;

    unsigned long	intCount;	/* interrupt count */
    unsigned long	aiCount;	/* total transfer size (samples) */
    unsigned long	aiExtraInt;	/* ints but no data */
    int			aboutWrap;

					/* PCI device info */
    struct pci_dev *pci_dev;

    /* read back data */
    lsampl_t	aoValue[2];		/* Used for AO read back */

    /* timer gate (when enabled) */
    u8		utcGate[4];		/* 1 extra allows simple range check */

    /* shadow registers affect other registers, but cant be read back */
    /* The macros below update these on writes */
    u16		intMask;		/* interrupt mask */
    u16		intClearMask;		/* interrupt clear mask */
    u8		utcCtrl[4];		/* crtl mode for 3 utc + read back */
    u8		dioStatus;		/* could be read back (dio0Ctrl) */

} rtdPrivate;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((rtdPrivate *)dev->private)


/* Macros to access registers */

/* Reset board */
#define RtdResetBoard(dev) \
    writel (0, devpriv->las0+LAS0_BOARD_RESET)

/* Reset channel gain table read pointer */
#define RtdResetCGT(dev) \
    writel (0, devpriv->las0+LAS0_CGT_RESET)

/* Reset channel gain table read and write pointers */
#define RtdClearCGT(dev) \
    writel (0, devpriv->las0+LAS0_CGT_CLEAR)

/* Reset channel gain table read and write pointers */
#define RtdEnableCGT(dev,v) \
    writel ((v > 0) ? 1 : 0, devpriv->las0+LAS0_CGT_ENABLE)

/* Write channel gain table entry */
#define RtdWriteCGTable(dev,v) \
    writel (v, devpriv->las0+LAS0_CGT_WRITE)

/* Write Channel Gain Latch */
#define RtdWriteCGLatch(dev,v) \
    writel (v, devpriv->las0+LAS0_CGL_WRITE)

/* Reset ADC FIFO */
#define RtdAdcClearFifo(dev) \
    writel (0, devpriv->las0+LAS0_ADC_FIFO_CLEAR)

/* Set ADC start conversion source select (write only) */
#define RtdAdcConversionSource(dev,v) \
    writel (v, devpriv->las0+LAS0_ADC_CONVERSION)

/* Set burst start source select (write only) */
#define RtdBurstStartSource(dev,v) \
    writel (v, devpriv->las0+LAS0_BURST_START)

/* Set Pacer start source select (write only) */
#define RtdPacerStartSource(dev,v) \
    writel (v, devpriv->las0+LAS0_PACER_START)

/* Set Pacer stop source select (write only) */
#define RtdPacerStopSource(dev,v) \
    writel (v, devpriv->las0+LAS0_PACER_STOP)

/* Set Pacer clock source select (write only) 0=external 1=internal */
#define RtdPacerClockSource(dev,v) \
    writel ((v > 0) ? 1 : 0, devpriv->las0+LAS0_PACER_SELECT)

/* Set sample counter source select (write only) */
#define RtdAdcSampleCounterSource(dev,v) \
    writel (v, devpriv->las0+LAS0_ADC_SCNT_SRC)

/* Set Pacer trigger mode select (write only) 0=single cycle, 1=repeat */
#define RtdPacerTriggerMode(dev,v) \
    writel ((v > 0) ? 1 : 0, devpriv->las0+LAS0_PACER_REPEAT)

/* Set About counter stop enable (write only) */
#define RtdAboutStopEnable(dev,v) \
    writel ((v > 0) ? 1 : 0, devpriv->las0+LAS0_ACNT_STOP_ENABLE)

/* Set external trigger polarity (write only) 0=positive edge, 1=negative */
#define RtdTriggerPolarity(dev,v) \
    writel ((v > 0) ? 1 : 0, devpriv->las0+LAS0_ETRG_POLARITY)

/* Start single ADC conversion */
#define RtdAdcStart(dev) \
    writew (0, devpriv->las0+LAS0_ADC)

/* Read one ADC data value (12bit (with sign extend) as 16bit) */
/* Note: matches what DMA would get.  Actual value >> 3 */
#define RtdAdcFifoGet(dev) \
    readw (devpriv->las1+LAS1_ADC_FIFO)

/* Read two ADC data values (DOESNT WORK) */
#define RtdAdcFifoGet2(dev) \
    readl (devpriv->las1+LAS1_ADC_FIFO)

/* FIFO status */
#define RtdFifoStatus(dev) \
    readl (devpriv->las0+LAS0_ADC)

/* pacer start/stop read=start, write=stop*/
#define RtdPacerStart(dev) \
    readl (devpriv->las0+LAS0_PACER)
#define RtdPacerStop(dev) \
    writel (0, devpriv->las0+LAS0_PACER)

/* Interrupt status */
#define RtdInterruptStatus(dev) \
    readw (devpriv->las0+LAS0_IT)

/* Interrupt mask */
#define RtdInterruptMask(dev,v) \
    writew ((devpriv->intMask = (v)),devpriv->las0+LAS0_IT)

/* Interrupt status clear (only bits set in mask) */
#define RtdInterruptClear(dev) \
    readw (devpriv->las0+LAS0_CLEAR)

/* Interrupt clear mask */
#define RtdInterruptClearMask(dev,v) \
    writew ((devpriv->intClearMask = (v)), devpriv->las0+LAS0_CLEAR)

/* Interrupt overrun status */
#define RtdInterruptOverrunStatus(dev) \
    readl (devpriv->las0+LAS0_OVERRUN)

/* Interrupt overrun clear */
#define RtdInterruptOverrunClear(dev) \
    writel (0, devpriv->las0+LAS0_OVERRUN)

/* Pacer counter, 24bit */
#define RtdPacerCount(dev) \
    readl (devpriv->las0+LAS0_PCLK)
#define RtdPacerCounter(dev,v) \
    writel ((v) & 0xffffff,devpriv->las0+LAS0_PCLK)

/* Burst counter, 10bit */
#define RtdBurstCount(dev) \
    readl (devpriv->las0+LAS0_BCLK)
#define RtdBurstCounter(dev,v) \
    writel ((v) & 0x3ff,devpriv->las0+LAS0_BCLK)

/* Delay counter, 16bit */
#define RtdDelayCount(dev) \
    readl (devpriv->las0+LAS0_DCLK)
#define RtdDelayCounter(dev,v) \
    writel ((v) & 0xffff, devpriv->las0+LAS0_DCLK)

/* About counter, 16bit */
#define RtdAboutCount(dev) \
    readl (devpriv->las0+LAS0_ACNT)
#define RtdAboutCounter(dev,v) \
    writel ((v) & 0xffff, devpriv->las0+LAS0_ACNT)

/* ADC sample counter, 10bit */
#define RtdAdcSampleCount(dev) \
    readl (devpriv->las0+LAS0_ADC_SCNT)
#define RtdAdcSampleCounter(dev,v) \
    writel ((v) & 0x3ff, devpriv->las0+LAS0_ADC_SCNT)


/* User Timer/Counter (8254) */
#define RtdUtcCounterGet(dev,n) \
    readb (devpriv->las0 \
        + ((n <= 0) ? LAS0_UTC0 : ((1 == n) ? LAS0_UTC1 : LAS0_UTC2)))

#define RtdUtcCounterPut(dev,n,v) \
    writeb ((v) & 0xff, devpriv->las0 \
        + ((n <= 0) ? LAS0_UTC0 : ((1 == n) ? LAS0_UTC1 : LAS0_UTC2)))

/* Set UTC (8254) control byte  */
#define RtdUtcCtrlPut(dev,n,v) \
    writeb (devpriv->utcCtrl[(n) & 3] = (((n) & 3) << 6) | ((v) & 0x3f), \
      devpriv->las0 + LAS0_UTC_CTRL)

/* Set UTCn clock source (write only) */
#define RtdUtcClockSource(dev,n,v) \
    writew (v, devpriv->las0 \
        + ((n <= 0) ? LAS0_UTC0_CLOCK : \
           ((1 == n) ? LAS0_UTC1_CLOCK : LAS0_UTC2_CLOCK)))

/* Set UTCn gate source (write only) */
#define RtdUtcGateSource(dev,n,v) \
    writew (v, devpriv->las0 \
        + ((n <= 0) ? LAS0_UTC0_GATE : \
           ((1 == n) ? LAS0_UTC1_GATE : LAS0_UTC2_GATE)))


/* User output N source select (write only) */
#define RtdUsrOutSource(dev,n,v) \
    writel (v,devpriv->las0+((n <= 0) ? LAS0_UOUT0_SELECT : LAS0_UOUT1_SELECT))


/* PLX9080 interrupt mask and status */
#define RtdPLXInterruptRead(dev) \
    readl (devpriv->lcfg+LCFG_ITCSR)
#define RtdPLXInterruptWrite(dev,v) \
    writel (v, devpriv->lcfg+LCFG_ITCSR)


/* Digital IO */
#define RtdDio0Read(dev) \
    (readw (devpriv->las0+LAS0_DIO0) & 0xff)
#define RtdDio0Write(dev,v) \
    writew ((v) & 0xff, devpriv->las0+LAS0_DIO0)

#define RtdDio1Read(dev) \
    (readw (devpriv->las0+LAS0_DIO1) & 0xff)
#define RtdDio1Write(dev,v) \
    writew ((v) & 0xff, devpriv->las0+LAS0_DIO1)

#define RtdDioStatusRead(dev) \
    (readw (devpriv->las0+LAS0_DIO_STATUS) & 0xff)
#define RtdDioStatusWrite(dev,v) \
    writew ((devpriv->dioStatus = (v)), devpriv->las0+LAS0_DIO_STATUS)

#define RtdDio0CtrlRead(dev) \
    (readw (devpriv->las0+LAS0_DIO0_CTRL) & 0xff)
#define RtdDio0CtrlWrite(dev,v) \
    writew ((v) & 0xff, devpriv->las0+LAS0_DIO0_CTRL)


/* Digital to Analog converter */
/* Write one data value (sign + 12bit + marker bits) */
/* Note: matches what DMA would put.  Actual value << 3 */
#define RtdDacFifoPut(dev,n,v) \
    writew ((v), devpriv->las1 +(((n) == 0) ? LAS1_DAC1_FIFO : LAS1_DAC2_FIFO))

/* Start single DAC conversion */
#define RtdDacUpdate(dev,n) \
    writew (0, devpriv->las0 +(((n) == 0) ? LAS0_DAC1 : LAS0_DAC2))

/* Start single DAC conversion on both DACs */
#define RtdDacBothUpdate(dev) \
    writew (0, devpriv->las0+LAS0_DAC)

/* Set DAC output type and range */
#define RtdDacRange(dev,n,v) \
    writew ((v) & 7, devpriv->las0 \
	+(((n) == 0) ? LAS0_DAC1_CTRL : LAS0_DAC2_CTRL))

/* Reset DAC FIFO */
#define RtdDacClearFifo(dev,n) \
    writel (0, devpriv->las0+(((n) == 0) ? LAS0_DAC1_RESET : LAS0_DAC2_RESET))

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int rtd_attach (comedi_device *dev, comedi_devconfig *it);
static int rtd_detach (comedi_device *dev);

static comedi_driver rtd520Driver={
    driver_name:	"rtd520",
    module:		THIS_MODULE,
    attach:		rtd_attach,
    detach:		rtd_detach,
    board_name:		rtd520Boards,
    offset:		sizeof(rtdBoard),
    num_names:		sizeof(rtd520Boards) / sizeof(rtdBoard),
};

static int rtd_ai_rinsn (comedi_device *dev, comedi_subdevice *s,
			 comedi_insn *insn, lsampl_t *data);
static int rtd_ao_winsn (comedi_device *dev, comedi_subdevice *s,
			 comedi_insn *insn, lsampl_t *data);
static int rtd_ao_rinsn (comedi_device *dev, comedi_subdevice *s,
			 comedi_insn *insn, lsampl_t *data);
static int rtd_dio_insn_bits (comedi_device *dev, comedi_subdevice *s,
			      comedi_insn *insn, lsampl_t *data);
static int rtd_dio_insn_config (comedi_device *dev, comedi_subdevice *s,
				comedi_insn *insn, lsampl_t *data);
static int rtd_ai_cmdtest (comedi_device *dev,comedi_subdevice *s,
			   comedi_cmd *cmd);
static int rtd_ai_cmd ( comedi_device *dev, comedi_subdevice *s);
static int rtd_ai_cancel ( comedi_device *dev, comedi_subdevice *s);
static int rtd_ns_to_timer (unsigned int *ns, int roundMode);
static void rtd_interrupt ( int irq, void *d, struct pt_regs *regs);


/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int rtd_attach (
    comedi_device *dev,
    comedi_devconfig *it)		/* board name and options flags */
{
    comedi_subdevice *s;
    struct pci_dev* pcidev;
    int index;
    int	ret;
    unsigned long physLas0;		/* configuation */
    unsigned long physLas1;		/* data area */
    unsigned long physLcfg;		/* PLX9080 */

    printk ("comedi%d: rtd520 attaching.\n", dev->minor);

    /*
     * Allocate the private structure area.  alloc_private() is a
     * convenient macro defined in comedidev.h.
     */
    if (alloc_private (dev, sizeof(rtdPrivate))<0)
	return -ENOMEM;

    /*
     * Probe the device to determine what device in the series it is.
     */
    pci_for_each_dev (pcidev) {
	if (pcidev->vendor == PCI_VENDOR_ID_RTD) {
	    if (it->options[0] || it->options[1]) {
		if (pcidev->bus->number == it->options[0]
		    && PCI_SLOT(pcidev->devfn) == it->options[1]) {
		    DPRINTK("rtd520: found bus=%d slot=%d\n",
			   it->options[0], it->options[1]);
		    break;		/* found it */
		}
	    } else {		/* specific board/slot not specified */
		break;		/* found one */
	    }
	}
    }

    if (!pcidev) {
	if (it->options[0] && it->options[1]) {
	    printk ("No RTD card at bus=%d slot=%d.\n",
		    it->options[0], it->options[1]);
	} else {
	    printk ("No RTD card found.\n");
	}
	return -EIO;
    }

    /* See if this is a model that we know about */
    for (index=0; index < rtd520Driver.num_names; index++){
	if (rtd520Boards[index].device_id == pcidev->device) {
	    break;
	}
    }
    if (index >= rtd520Driver.num_names) {
	printk ("Found an RTD card, but not a supported type (%x).\n",
		pcidev->device);
	return -EIO;
    } else {
	devpriv->pci_dev = pcidev;
	dev->board_ptr = rtd520Boards+index;
    }
    /*
     * Initialize dev->board_name.  Note that we can use the "thisboard"
     * macro now, since we just initialized it in the last line.
     */
    dev->board_name = thisboard->name;

    /*
     * Initialize base addresses
     */
    /* Get the physical address from PCI config */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
    physLas0 = devpriv->pci_dev->base_address[LAS0_PCIINDEX];
    physLas1 = devpriv->pci_dev->base_address[LAS1_PCIINDEX];
    physLcfg = devpriv->pci_dev->base_address[LCFG_PCIINDEX];
#else
    physLas0 = devpriv->pci_dev->resource[LAS0_PCIINDEX].start;
    physLas1 = devpriv->pci_dev->resource[LAS1_PCIINDEX].start;
    physLcfg = devpriv->pci_dev->resource[LCFG_PCIINDEX].start;
#endif
    /* Now have the kernel map this into memory */
					/* ASSUME page aligned */
    devpriv->las0 = ioremap(physLas0, LAS0_PCISIZE);
    devpriv->las1 = ioremap(physLas1, LAS1_PCISIZE);
    devpriv->lcfg = ioremap(physLcfg, LCFG_PCISIZE);

    printk ("%s: ", dev->board_name);
    /*printk ("%s: LAS0=%lx, LAS1=%lx, CFG=%lx.\n", dev->board_name,
      physLas0, physLas1, physLcfg);*/

    /*
     * Allocate the subdevice structures.  alloc_subdevice() is a
     * convenient macro defined in comedidev.h.  It relies on
     * n_subdevices being set correctly.
     */
    dev->n_subdevices=4;
    if (alloc_subdevices(dev)<0)
	return -ENOMEM;

    s=dev->subdevices+0;
    dev->read_subdev=s;
    /* analog input subdevice */
    s->type=COMEDI_SUBD_AI;
    s->subdev_flags=SDF_READABLE;
    s->n_chan=thisboard->aiChans;
    s->maxdata=(1<<thisboard->aiBits)-1;
    if (thisboard->aiMaxGain <= 32) {
	s->range_table = &rtd_ai_7520_range;
    } else {
	s->range_table = &rtd_ai_4520_range;
    }
    s->len_chanlist = thisboard->fifoLen;
    s->insn_read = rtd_ai_rinsn;
    s->do_cmd = rtd_ai_cmd;
    s->do_cmdtest = rtd_ai_cmdtest;
    s->cancel = rtd_ai_cancel;

    s=dev->subdevices+1;
    /* analog output subdevice */
    s->type=COMEDI_SUBD_AO;
    s->subdev_flags=SDF_WRITEABLE;
    s->n_chan = 2;
    s->maxdata =(1<<thisboard->aiBits)-1;
    s->range_table = &rtd_ao_range;
    s->insn_write = rtd_ao_winsn;
    s->insn_read = rtd_ao_rinsn;

    s=dev->subdevices+2;
    /* digital i/o subdevice */
    s->type=COMEDI_SUBD_DIO;
    s->subdev_flags=SDF_READABLE|SDF_WRITEABLE;
    /* we only support port 0 right now.  Ignoring port 1 and user IO */
    s->n_chan=8;
    s->maxdata=1;
    s->range_table=&range_digital;
    s->insn_bits = rtd_dio_insn_bits;
    s->insn_config = rtd_dio_insn_config;

    /* timer/counter subdevices */
    s=dev->subdevices+3;
    s->type = COMEDI_SUBD_COUNTER;
    s->subdev_flags=SDF_READABLE|SDF_WRITEABLE;
    //s->insn_read=  rtd_gpct_insn_read;
    //s->insn_write= rtd_gpct_insn_write;
    //s->insn_config=rtd_gpct_insn_config;
    s->n_chan=3;
    s->maxdata=0xffff;

    /* check if our interrupt is available and get it */
    dev->irq = devpriv->pci_dev->irq;
    if(dev->irq>0){
	if((ret=comedi_request_irq (dev->irq, rtd_interrupt,
				    0, "rtd520", dev))<0)
	    return ret;
	printk("( irq = %d )\n", dev->irq);
    } else {
	printk("( NO IRQ )");
    }
	
					/* initialize board, per RTD spec */
					/* also, initialize shadow registers */
    RtdResetBoard (dev);
    RtdInterruptMask (dev,0);		/* and sets shadow */
    RtdInterruptClearMask (dev,~0);	/* and sets shadow */
    RtdInterruptClear(dev);		/* clears bits set by mask */
    RtdInterruptOverrunClear(dev);
    RtdClearCGT (dev);
    RtdAdcClearFifo (dev);
    RtdDacClearFifo (dev,0);
    RtdDacClearFifo (dev,1);
					/* clear digital IO fifo*/
    RtdDioStatusWrite (dev, 0);		/* safe state, set shadow */
    RtdUtcCtrlPut (dev, 0, 0x30);	/* safe state, set shadow */
    RtdUtcCtrlPut (dev, 1, 0x30);	/* safe state, set shadow */
    RtdUtcCtrlPut (dev, 2, 0x30);	/* safe state, set shadow */
    RtdUtcCtrlPut (dev, 3, 0);		/* safe state, set shadow */
    /* todo: set user out source ??? */

    if (dev->irq) {			/* enable interrupt controller */
	RtdPLXInterruptWrite (dev,
			      RtdPLXInterruptRead (dev) | (0x0800));
    }

    printk("comedi%d: rtd520 driver attached.\n", dev->minor);

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
static int rtd_detach (
    comedi_device *dev)
{
    DPRINTK("comedi%d: rtd520: removing (%ld ints, %ld extra ai)\n(int status 0x%x, overrun status 0x%x, fifo status 0x%x)...\n",
	   dev->minor, devpriv->intCount, devpriv->aiExtraInt,
	   0xffff & RtdInterruptStatus (dev),
	   0xffff & RtdInterruptOverrunStatus (dev),
	   0xffff & RtdFifoStatus (dev));
    if (devpriv) {
	/* Shut down any board ops by resetting it */
	RtdResetBoard (dev);
	RtdInterruptMask (dev, 0);
	RtdInterruptClearMask (dev,~0);
	RtdInterruptClear(dev);		/* clears bits set by mask */

	/* release DMA */

	/* release IRQ */
	if (dev->irq) {
	    /* disable interrupt controller */
	    RtdPLXInterruptWrite (dev,
				  RtdPLXInterruptRead (dev) & ~(0x0800));
	    comedi_free_irq (dev->irq, dev);
	}

	/* release all regions that were allocated */
	if (devpriv->las0) {
	    iounmap (devpriv->las0);
	}
	if (devpriv->las1) {
	    iounmap (devpriv->las1);
	}
	if (devpriv->lcfg) {
	    iounmap (devpriv->lcfg);
	}
    }

    printk("comedi%d: rtd520: removed.\n",dev->minor);
	
    return 0;
}

/*
  Convert a single comedi channel-gain entry to a RTD520 table entry
*/
static unsigned short rtdConvertChanGain (
    unsigned int	comediChan)
{
    unsigned int chan, range, aref;
    unsigned short r=0;

    chan = CR_CHAN (comediChan);
    range = CR_RANGE (comediChan);
    aref = CR_AREF (comediChan);

    r |= chan & 0xf;

    /* TODO: Should also be able to switch into +-=10 range */
    /* HACK!!! should not use a constant here */
    if (range < 6) {		/* first 6 are bipolar */
	r |= 0x000;			/* +-5 range */
	r |= (range & 0x7) << 4;	/* gain */
    } else {
	r |= 0x200;			/* +10 range */
	r |= ((range-6) & 0x7) << 4;	/* gain */
    }

    switch (aref) {
    case AREF_GROUND:
	break;

    case AREF_COMMON:
	r |= 0x80;			/* ref external analog common */
	break;

    case AREF_DIFF:
	r |= 0x400;			/* DIFF */
	break;

    case AREF_OTHER:		/* ??? */
	break;
    }
    /*printk ("chan=%d r=%d a=%d -> 0x%x\n",
      chan, range, aref, r);*/
    return r;
}

/*
  Setup the channel-gain table from a comedi list
*/
static void rtd_load_channelgain_list (
    comedi_device *dev,
    unsigned int n_chan,
    unsigned int *list)
{
    if (n_chan > 1) {			/* setup channel gain table */
	int	ii;
	RtdClearCGT (dev);
	RtdEnableCGT(dev, 1);		/* enable table */
	for(ii=0; ii < n_chan; ii++){
	    RtdWriteCGTable (dev, rtdConvertChanGain (list[ii]));
	}
    } else {				/* just use the channel gain latch */
	RtdEnableCGT(dev, 0);		/* disable table, enable latch */
	RtdWriteCGLatch (dev, rtdConvertChanGain (list[0]));
    }
}

/*
  "instructions" read/write data in "one-shot" or "software-triggered"
  mode (simplest case).
  This doesnt use interrupts.
 */
static int rtd_ai_rinsn (
    comedi_device *dev,
    comedi_subdevice *s,
    comedi_insn *insn,
    lsampl_t *data)
{
    int n, ii;
    int	stat;

    /* write channel to multiplexer and clear channel gain table */ 
    rtd_load_channelgain_list (dev, 1, &insn->chanspec);
	
					/* set conversion source */
    RtdAdcConversionSource (dev, 0);	/* software */

					/* wait for mux to settle */
    udelay (RTD_SETTLE_DELAY);

					/* clear any old fifo data */
    RtdAdcClearFifo (dev);

    stat = RtdFifoStatus (dev);		/* DEBUG */
    if (stat & FS_ADC_EMPTY) {		/* 1 -> not empty */
	printk ("rtd520: Warning: fifo didn't seem to clear! FifoStatus=0x%x\n",
		stat);
    }

    /* convert n samples */
    for (n=0; n < insn->n; n++) {
	s16 d;
	/* trigger conversion */
	RtdAdcStart (dev);

	/* wait for conversion to end */
	udelay ((2*RTD_MAX_SPEED + RTD_MAX_SPEED - 1)/1000);
	for (ii = 0; ii < RTD_ADC_TIMEOUT; ++ii) {
	    /* by delaying here, we try to reduce system electrical noise */
	    udelay (1);
	    stat = RtdFifoStatus (dev);
	    if (stat & FS_ADC_EMPTY)	/* 1 -> not empty */
		break;
	}
	if (ii >= RTD_ADC_TIMEOUT) {
	    printk ("rtd520: Error: Never got ADC done flag! FifoStatus=0x%x\n",
		    stat);
	    return -ETIMEDOUT;
	}

	/* read data */
	d = RtdAdcFifoGet (dev);	/* get 2s comp value */
	/*printk ("rtd520: Got 0x%x after %d usec\n", d, ii+1);*/
	d = d >> 3;			/* low 3 bits are marker lines */
	data[n] = d + 2048;		/* convert to comedi unsigned data */
    }

    /* return the number of samples read/written */
    return n;
}

/*
  Fifo is a least half full.  Get what we know is there.... Fast!
  This uses 1/2 the bus cycles of read_dregs (below).

  The manual claims that we can do a lword read, but it doesn't work here.
*/
static void ai_read_half_fifo (
    comedi_device *dev,
    comedi_subdevice *s)
{
    int ii;

    for (ii = 0; ii < thisboard->fifoLen / 2; ii++) {
	s16 d = RtdAdcFifoGet (dev);	/* get 2s comp value */
	d = d >> 3;			/* low 3 bits are marker lines */

	if (devpriv->aiCount <= 0) {
					/* should never happen */
	    devpriv->aiExtraInt++;
	    continue;
	}

					/* check and deal with buffer wrap */
	if (s->async->buf_int_ptr >= s->async->data_len) {
	    s->async->buf_int_ptr = 0;
	    s->async->events |= COMEDI_CB_EOBUF;
	}
					/* write into buffer */
	*((sampl_t *)((void *)s->async->data + s->async->buf_int_ptr))
	    = d + 2048;		/* convert to comedi unsigned data */
	s->async->buf_int_count += sizeof(sampl_t);
	s->async->buf_int_ptr += sizeof(sampl_t);
	devpriv->aiCount--;
    }
}

/*
  unknown amout of data is waiting in fifo.
*/
static void ai_read_dregs (
    comedi_device *dev,
    comedi_subdevice *s)
{
    while (RtdFifoStatus (dev) & FS_ADC_EMPTY) { /* 1 -> not empty */
	s16 d = RtdAdcFifoGet (dev); /* get 2s comp value */

	d = d >> 3;		/* low 3 bits are marker lines */

	if (devpriv->aiCount <= 0) {
					/* should never happen */
	    devpriv->aiExtraInt++;
	    continue;
	}

					/* check and deal with buffer wrap */
	if (s->async->buf_int_ptr >= s->async->data_len) {
	    s->async->buf_int_ptr = 0;
	    s->async->events |= COMEDI_CB_EOBUF;
	}
					/* write into buffer */
	*((sampl_t *)((void *)s->async->data + s->async->buf_int_ptr))
	    = d + 2048;		/* convert to comedi unsigned data */
	s->async->buf_int_count += sizeof(sampl_t);
	s->async->buf_int_ptr += sizeof(sampl_t);
	devpriv->aiCount--;
    }
}

/*
  Handle all rtd520 interrupts.  
  Runs atomically and is never re-entered.
  This is a "slow handler";  other interrupts may be active.
  The data conversion may someday happen in a "bottom half".
*/
static void rtd_interrupt (
    int irq,				/* interrupt number (ignored) */
    void *d,				/* our data */
    struct pt_regs *regs)		/* cpu context (ignored) */
{
    comedi_device *dev = d;		/* must be called "dev" for devpriv */
    u16 status = RtdInterruptStatus (dev);

    devpriv->intCount++;

    /* if interrupt was not caused by our board */
    /* needed??? we dont claim to share interrupt lines */
    if ((0 == status)
	|| !(dev->attached)) {
	return;
    }

    /* Check for analog in */
    /* Either end if a sequence (about), or time to flush the fifo (sample) */
    /* Future: process DMA transfer */
    if (status & (IRQM_ADC_ABOUT_CNT | IRQM_ADC_SAMPLE_CNT)) {
	comedi_subdevice *s = dev->subdevices + 0; /* analog in subdevice */

	/* Check for any ready data */
	if (RtdFifoStatus (dev) & FS_ADC_HEMPTY) { /* read 1/2 fifo worth */
	    ai_read_half_fifo (dev, s);
	    /*comedi_bufcheck (dev, s);	*/
	    s->async->events |= COMEDI_CB_BLOCK; /* signal something there */
	} else {
	    /* for slow transfers, we should read whatever is there */
	    /*s->async->events |= COMEDI_CB_EOS;*/
	}

	if (0 == devpriv->aiCount) { /* done! stop! */
	    RtdInterruptMask (dev, 0);		/* mask out ABOUT and SAMPLE */
	    RtdPacerStop (dev);			/* Stop PACER */
	    /*comedi_done (dev, s);*/
	    s->async->events |= COMEDI_CB_EOA;/* signal end to comedi */
	} else if (status & IRQM_ADC_ABOUT_CNT) { /* about cnt terminated */
	    if (devpriv->aboutWrap) { /* multi-count wraps */
		if (devpriv->aiCount < devpriv->aboutWrap) {
		    RtdAboutStopEnable (dev, 0); /* enable stop */
		    devpriv->aboutWrap = 0;
		}
	    } else {			/* done */
		/* TODO: allow multiple interrupt sources */
		RtdInterruptMask (dev, 0);/* mask out ABOUT and SAMPLE */
		RtdPacerStop (dev);	/* Stop PACER */
		ai_read_dregs (dev, s);	/* ready anything in FIFO */

		/*comedi_done (dev, s);*/
		s->async->events |= COMEDI_CB_EOA;/* signal end to comedi */
	    }
	}

	/* check for fifo over-run */
	
	if (s->async->events != 0) {	/* signal any events */
	    comedi_event (dev, s, s->async->events);
	    s->async->events = 0;
	}
    }

					/* clear the interrupt */
    RtdInterruptClearMask (dev, status);
    RtdInterruptClear (dev);
}

/*
  cmdtest tests a particular command to see if it is valid.
  Using the cmdtest ioctl, a user can create a valid cmd
  and then have it executed by the cmd ioctl (asyncronously).
 
  cmdtest returns 1,2,3,4 or 0, depending on which tests
  the command passes.
*/

static int rtd_ai_cmdtest (
    comedi_device *dev,
    comedi_subdevice *s,
    comedi_cmd *cmd)
{
    int err=0;
    int tmp;

    /* step 1: make sure trigger sources are trivially valid */

    tmp = cmd->start_src;
    cmd->start_src &= TRIG_NOW;
    if (!cmd->start_src || tmp != cmd->start_src) {
	err++;
    }

    tmp=cmd->scan_begin_src;
    cmd->scan_begin_src &= TRIG_TIMER|TRIG_EXT;
    if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src) {
	err++;
    }

    tmp=cmd->convert_src;
    cmd->convert_src &= TRIG_TIMER|TRIG_EXT;
    if (!cmd->convert_src || tmp != cmd->convert_src) {
	err++;
    }

    tmp=cmd->scan_end_src;
    cmd->scan_end_src &= TRIG_COUNT;
    if (!cmd->scan_end_src || tmp != cmd->scan_end_src) {
	err++;
    }

    tmp=cmd->stop_src;
    cmd->stop_src &= TRIG_COUNT|TRIG_NONE;
    if (!cmd->stop_src || tmp != cmd->stop_src) {
	err++;
    }

    if (err)
	return 1;

    /* step 2: make sure trigger sources are unique
       and mutually compatible */
    /* note that mutual compatiblity is not an issue here */
    if (cmd->scan_begin_src !=TRIG_TIMER &&
	cmd->scan_begin_src !=TRIG_EXT) {
	err++;
    }
    if (cmd->convert_src !=TRIG_TIMER &&
	cmd->convert_src !=TRIG_EXT) {
	err++;
    }
    if (cmd->stop_src != TRIG_COUNT &&
	cmd->stop_src != TRIG_NONE) {
	err++;
    }

    if (err) {
	return 2;
    }

    /* step 3: make sure arguments are trivially compatible */

    if (cmd->start_arg != 0) {
	cmd->start_arg = 0;
	DPRINTK ("rtd520: cmdtest: start_arg not 0\n");
	err++;
    }

    if (cmd->scan_begin_src == TRIG_TIMER){
	if (cmd->scan_begin_arg < RTD_MAX_SPEED) {
	    cmd->scan_begin_arg = RTD_MAX_SPEED;
	    DPRINTK ("rtd520: cmdtest: scan rate greater than max.\n");
	    err++;
	}
	if (cmd->scan_begin_arg > RTD_MIN_SPEED) {
	    cmd->scan_begin_arg = RTD_MIN_SPEED;
	    DPRINTK ("rtd520: cmdtest: scan rate lower than min.\n");
	    err++;
	}
    } else {
	/* external trigger */
	/* should be level/edge, hi/lo specification here */
	/* should specify multiple external triggers */
	if (cmd->scan_begin_arg > 9) {
	    cmd->scan_begin_arg = 9;
	    DPRINTK ("rtd520: cmdtest: scan_begin_arg out of range\n");
	    err++;
	}
    }
    if (cmd->convert_src==TRIG_TIMER) {
	if (cmd->convert_arg < RTD_MAX_SPEED) {
	    cmd->convert_arg = RTD_MAX_SPEED;
	    DPRINTK ("rtd520: cmdtest: convert rate greater than max.\n");
	    err++;
	}
	if (cmd->convert_arg > RTD_MIN_SPEED) {
	    cmd->convert_arg = RTD_MIN_SPEED;
	    DPRINTK ("rtd520: cmdtest: convert rate lower than min.\n");
	    err++;
	}
    } else {
	/* external trigger */
	/* see above */
	if (cmd->convert_arg > 9) {
	    cmd->convert_arg = 9;
	    DPRINTK ("rtd520: cmdtest: convert_arg out of range\n");
	    err++;
	}
    }

#if 0
    if (cmd->scan_end_arg != cmd->chanlist_len) {
	cmd->scan_end_arg = cmd->chanlist_len;
	err++;
    }
#endif
    if (cmd->stop_src==TRIG_COUNT) {
	/* TODO check for rounding error due to counter wrap */

    } else {
	/* TRIG_NONE */
	if (cmd->stop_arg!=0) {
	    cmd->stop_arg=0;
	    DPRINTK ("rtd520: cmdtest: stop_arg not 0\n");
	    err++;
	}
    }

    if (err) {
	DPRINTK ("rtd520: cmdtest error! Some argument compatibility test failed.\n");
	return 3;
    }

    /* step 4: fix up any arguments */

    if (cmd->scan_begin_src == TRIG_TIMER) {
	tmp=cmd->scan_begin_arg;
	rtd_ns_to_timer(&cmd->scan_begin_arg,
			cmd->flags&TRIG_ROUND_MASK);
	if (tmp!=cmd->scan_begin_arg) {
	    err++;
	}
    }
    if (cmd->convert_src == TRIG_TIMER){
	tmp=cmd->convert_arg;
	rtd_ns_to_timer(&cmd->convert_arg,
			cmd->flags&TRIG_ROUND_MASK);
	if (tmp!=cmd->convert_arg) {
	    err++;
	}
	if (cmd->scan_begin_src == TRIG_TIMER
	    && (cmd->scan_begin_arg
		< (cmd->convert_arg * cmd->scan_end_arg))) {
	    cmd->scan_begin_arg=cmd->convert_arg*cmd->scan_end_arg;
	    err++;
	}
    }

    if (err) {
	DPRINTK ("rtd520: cmdtest error! Some timer value was altered.\n");
	return 4;
    }

    return 0;
}

/*
  Execute a analog in command with many possible triggering options.
  The data get stored in the async structure of the subdevice.
  This is usually done by an interrupt handler.
  Userland gets to the data using read calls.
*/
static int rtd_ai_cmd (
    comedi_device *dev,
    comedi_subdevice *s)
{
    comedi_cmd *cmd=&s->async->cmd;
    int timer;
    int	justPoll = 0;			/* can we do a simple poll */

					/* stop anything currently running */
    RtdPacerStop (dev);			/* Stop PACER */

    /* start configuration */
    /* load channel list and reset CGT */
    rtd_load_channelgain_list (dev, cmd->chanlist_len, cmd->chanlist);

    /* setup the common case and override if needed */
    if (cmd->chanlist_len > 1) {
	/*DPRINTK ("rtd520: Multi channel setup\n");*/
	RtdPacerStartSource (dev, 0);	/* software triggers pacer */
	RtdBurstStartSource (dev, 1);	/* PACER triggers burst */
	RtdAdcConversionSource (dev, 2); /* BURST triggers ADC */
    } else {				/* single channel */
	/*DPRINTK ("rtd520: single channel setup\n");*/
	RtdPacerStartSource (dev, 0);	/* software triggers pacer */
	RtdAdcConversionSource (dev, 1); /* PACER triggers ADC */
    }

    RtdAdcSampleCounter (dev,		/* setup a periodic interrupt */
			 (thisboard->fifoLen > 1024) ? 1023 : 511);
    RtdPacerStopSource (dev, 3);	/* stop on ABOUT count down*/
    RtdAboutStopEnable (dev, 0);	/* actually stop (see below) */
    RtdPacerClockSource (dev, 1);	/* use INTERNAL 8Mhz clock source */
    RtdAdcSampleCounterSource (dev, 1);	/* count samples, not scans */

    /* BUG??? these look like enumerated values, but they are bit fields */

    /* First, setup when to stop */
    switch(cmd->stop_src){
    case TRIG_COUNT:			/* stop after N scans */
	if ((cmd->chanlist_len <= 1) 	/* no scanning to do */
	    && (cmd->stop_arg <= 1)) {
	    justPoll = 1;
	    RtdAdcConversionSource (dev, 0); /* SOFTWARE trigger */
	} else {
	    int	n = cmd->stop_arg * cmd->chanlist_len;
	    /* load about counter (16bit) with number of SAMPLES */
	    devpriv->aiCount = n;
	    if (n <= 0x10000) {
		/* Note: stop on underflow.  Load with N-1 */
		DPRINTK ("rtd520: loading %d into about\n", n - 1);
		devpriv->aboutWrap = 0;
		RtdAboutCounter (dev, n - 1);
	    } else {			/* multiple counter wraps */
		int	mm, dd, ii;

		/*DPRINTK ("rtd520: multi-wrap count %d = %d x %d \n",
		  n, cmd->chanlist_len, cmd->stop_arg);*/
		/* interrupt on ABOUT wrap, until last wrap */
		/* we can run long, aiCount handles the excess */
		dd = (n + 0xffff-1) / 0xffff;/* find divisor, round up */
		mm = n / dd;
		/* dd * mm >= n, mm < 0xffff */

					/* try to find good divisor */
		for (ii = 0; ((mm*dd) < n) && (ii < 6); ++ii) {
		    dd++;
		    mm = n / dd;
		}

		if ((mm*dd) < n) {	/* just run long */
		    ++mm;
		    /* test case: try asking for 3 x 65537 samples */
		}
		/* dd * mm >= n, mm < 0xffff */
		DPRINTK ("rtd520: multi-wrap count (%d) %d = (%d) x %d\n",
			n, mm*dd, mm, dd);
		devpriv->aboutWrap = mm;
		RtdAboutCounter (dev, mm-1);
		RtdAboutStopEnable (dev, 1); /* just interrupt */
	    }
	}

	break;

    case TRIG_NONE:			/* stop when cancel is called */
	RtdPacerStopSource (dev, 0);	/* stop on SOFTWARE stop */
	break;

    default:
	DPRINTK ("rtd520: Warning! ignoring stop_src mode %d\n",
		cmd->stop_src);
    }


    /* Scan timing */
    switch (cmd->scan_begin_src) {
    case TRIG_TIMER:			/* periodic scanning */
	timer=rtd_ns_to_timer(&cmd->scan_begin_arg,TRIG_ROUND_NEAREST);
	/* set PACER clock */
	/*DPRINTK ("rtd520: loading %d into pacer\n", timer);*/
	RtdPacerCounter (dev, timer);

	break;

    case TRIG_EXT:
	RtdPacerStartSource (dev, 1);	/* EXTERNALy trigger pacer */
	break;

    default:
	DPRINTK ("rtd520: Warning! ignoring scan_begin_src mode %d\n",
		cmd->scan_begin_src);
    }

    /* Sample timing within a scan */
    switch(cmd->convert_src){
    case TRIG_TIMER:			/* periodic */
	if (cmd->chanlist_len > 1) {	/* only needed for multi-channel */
	    timer=rtd_ns_to_timer(&cmd->convert_arg,TRIG_ROUND_NEAREST);
	    /* setup BURST clock */
	    /*DPRINTK ("rtd520: loading %d into burst\n", timer);*/
	    RtdBurstCounter (dev, timer);
	}

	break;

    case TRIG_EXT:			/* external */
	RtdBurstStartSource (dev, 2);	/* EXTERNALy trigger burst */
	break;

    default:
	DPRINTK ("rtd520: Warning! ignoring convert_src mode %d\n",
		cmd->convert_src);
    }


    /* end configuration */

    /* start_src is ASSUMED to be TRIG_NOW */
    /* initial settling */
    udelay (RTD_SETTLE_DELAY);
					/* clear any old data */
    RtdAdcClearFifo (dev);

    /* see if we can do a simple polled input */
    if (justPoll) {
	int stat = RtdFifoStatus (dev);		/* DEBUG */
	s16 d;
	int ii;
	
					/* DEBUG */
	if (stat & FS_ADC_EMPTY) {		/* 1 -> not empty */
	    DPRINTK ("rtd520: ai_cmd Warning: fifo didn't seem to clear! FifoStatus=0x%x\n",
		    stat);
	} else {
	    DPRINTK ("rtd520: ai_cmd: polling for sample.\n");
	}

	/* trigger conversion */
	RtdAdcStart (dev);

	udelay ((2*RTD_MAX_SPEED + RTD_MAX_SPEED - 1)/1000);
	/* right now, this means just 1 sample. emulate ai_rinsn */
	for (ii = 0; ii < RTD_ADC_TIMEOUT; ++ii) {
	    /* by delaying here, we try to reduce system electrical noise */
	    udelay (1);
	    stat = RtdFifoStatus (dev);
	    if (stat & FS_ADC_EMPTY)	/* 1 -> not empty */
		break;
	}
	if (ii >= RTD_ADC_TIMEOUT) {
	    DPRINTK ("rtd520: ai_cmd Error: Never got data in FIFO! FifoStatus=0x%x\n",
		    stat);
	    return -ETIMEDOUT;
	}

	/* read data */
	d = RtdAdcFifoGet (dev);	/* get 2s comp value */
	/*DPRINTK ("rtd520: Got 0x%x after %d usec\n", d, ii+1);*/
	d = d >> 3;		/* low 3 bits are marker lines */

					/* write into buffer */
	*((sampl_t *)((void *)s->async->data + s->async->buf_int_ptr))
	    = d + 2048;			/* convert to comedi unsigned data */
	s->async->buf_int_count += sizeof(sampl_t);
	s->async->buf_int_ptr += sizeof(sampl_t);
	comedi_done (dev, s);
    } else {
	/* interrupt setup */
	if (! dev->irq) {
	    DPRINTK ("rtd520: ERROR! No interrupt available!\n");
	    return -ENXIO;
	}
	
	DPRINTK("rtd520: using interrupts. (%ld ints, %ld extra ai)\n(int status 0x%x, overrun status 0x%x, fifo status 0x%x)\n",
	       devpriv->intCount, devpriv->aiExtraInt,
	       0xffff & RtdInterruptStatus (dev),
	       0xffff & RtdInterruptOverrunStatus (dev),
	       0xffff & RtdFifoStatus (dev));

	RtdInterruptClearMask (dev, ~0); /* clear any existing flags */
	RtdInterruptClear (dev);
	/*DEBUG RtdInterruptOverrunClear(dev);*/

	/* TODO: allow multiple interrupt sources */
	if (devpriv->aiCount > 512) {
	    RtdInterruptMask (dev, IRQM_ADC_ABOUT_CNT | IRQM_ADC_SAMPLE_CNT );
	} else {
	    RtdInterruptMask (dev, IRQM_ADC_ABOUT_CNT);
	}

	RtdPacerStart (dev);		/* Start PACER */
    }
    return 0;
}

/*
  Stop a running data aquisition.
*/
static int rtd_ai_cancel (
    comedi_device *dev,
    comedi_subdevice *s)
{
					/* more is probably needed here */
    RtdPacerStop (dev);			/* Stop PACER */
    RtdAdcConversionSource (dev, 0);	/* software trigger only */
    return 0;
}

/*
  Given a desired period and the clock period (both in ns),
  return the proper counter value (divider-1).
  Sets the original period to be the true value.
  Note: you have to check if the value is larger than the counter range!
*/
static int rtd_ns_to_timer_base (
    unsigned int *nanosec,		/* desired period (in ns) */
    int round_mode,
    double base)			/* clock period (in ns) */
{
    int divider;

    switch(round_mode){
    case TRIG_ROUND_NEAREST:
    default:
	divider=(*nanosec+base/2)/base;
	break;
    case TRIG_ROUND_DOWN:
	divider=(*nanosec)/base;
	break;
    case TRIG_ROUND_UP:
	divider=(*nanosec+base-1)/base;
	break;
    }
    if (divider < 2) divider = 2;	/* min is divide by 2 */

    /* Note: we don't check for max, because different timers
       have different ranges */

    *nanosec=base*divider;
    return divider - 1;			/* countdown is divisor+1 */
}

/*
  Given a desired period (in ns),
  return the proper counter value (divider-1) for the internal clock.
  Sets the original period to be the true value.
*/
static int rtd_ns_to_timer (
    unsigned int *ns,
    int round_mode)
{
    return rtd_ns_to_timer_base (ns, round_mode, RTD_CLOCK_BASE);
}

/*
  Output one (or more) analog values to a single port as fast as possible.
*/
static int rtd_ao_winsn (
    comedi_device *dev,
    comedi_subdevice *s,
    comedi_insn *insn,
    lsampl_t *data)
{
    int i;
    int chan = CR_CHAN (insn->chanspec);
    int range = CR_RANGE (insn->chanspec);

    /* Configure the output range (table index matches the range values) */
    RtdDacRange (dev, chan, range);

    /* Writing a list of values to an AO channel is probably not
     * very useful, but that's how the interface is defined. */
    for (i=0; i < insn->n; ++i){
	int	val = data[i] << 3;

	/* VERIFY: comedi range and offset conversions */

	if ((range > 1)			/* bipolar */
	    && (data[i] < 2048)) {
					/* offset and sign extend */
	    val = (((int)data[i]) - 2048) << 3;
	} else {			/* unipolor */
	    val = data[i] << 3;
	}

	DPRINTK("comedi: rtd520 DAC chan=%d range=%d writing %d as 0x%x\n",
	       chan, range, data[i], val);

	/* a typical programming sequence */
	RtdDacFifoPut (dev, chan, val); /* put the value in */
	RtdDacUpdate (dev, chan);	/* trigger the conversion */

	devpriv->aoValue[chan] = data[i]; /* save for read back */

	if (insn->n > 1) {		/* let DAC finish (TODO poll) */
	    udelay (RTD_DAC_DELAY);
	}
    }

    /* return the number of samples read/written */
    return i;
}

/* AO subdevices should have a read insn as well as a write insn.
 * Usually this means copying a value stored in devpriv. */
static int rtd_ao_rinsn (
    comedi_device *dev,
    comedi_subdevice *s,
    comedi_insn *insn,
    lsampl_t *data)
{
    int i;
    int chan = CR_CHAN(insn->chanspec);

    for (i=0; i < insn->n; i++) {
	data[i] = devpriv->aoValue[chan];
    }

    return i;
}

/* 
   Write a masked set of bits and the read back the port.
   We track what the bits should be (i.e. we don't read the port first).
   
   DIO devices are slightly special.  Although it is possible to
 * implement the insn_read/insn_write interface, it is much more
 * useful to applications if you implement the insn_bits interface.
 * This allows packed reading/writing of the DIO channels.  The
 * comedi core can convert between insn_bits and insn_read/write
 */
static int rtd_dio_insn_bits (
    comedi_device *dev,
    comedi_subdevice *s,
    comedi_insn *insn,
    lsampl_t *data)
{
    if (insn->n!=2) return -EINVAL;

    /* The insn data is a mask in data[0] and the new data
     * in data[1], each channel cooresponding to a bit. */
    if (data[0]) {
	s->state &= ~data[0];
	s->state |= data[0]&data[1];
	
	/* Write out the new digital output lines */
	RtdDio0Write (dev, s->state);
    }
    /* on return, data[1] contains the value of the digital
     * input lines. */
    data[1] = RtdDio0Read (dev);

    /*DPRINTK("rtd520:port_0 wrote: 0x%x read: 0x%x\n", s->state, data[1]);*/

    return 2;
}

/*
  Configure one bit on a IO port as Input or Output (hence the name :-).
*/
static int rtd_dio_insn_config (
    comedi_device *dev,
    comedi_subdevice *s,
    comedi_insn *insn,
    lsampl_t *data)
{
    int chan=CR_CHAN(insn->chanspec);

    if (insn->n!=1) return -EINVAL;

    /* The input or output configuration of each digital line is
     * configured by a special insn_config instruction.  chanspec
     * contains the channel to be changed, and data[0] contains the 
     * value COMEDI_INPUT or COMEDI_OUTPUT. */
	
    if (data[0]==COMEDI_OUTPUT) {
	s->io_bits |= 1<<chan;		/* 1 means Out */
    } else {
	s->io_bits &= ~(1<<chan);
    }

    DPRINTK("rtd520: port_0_direction=0x%x (1 means out)\n", s->io_bits);
    /* TODO support digital match interrupts and strobes */
    RtdDioStatusWrite (dev, 0x01);	/* make Dio0Ctrl point to direction */
    RtdDio0CtrlWrite (dev, s->io_bits);	/* set direction 1 means Out */
    RtdDioStatusWrite (dev, 0);		/* make Dio0Ctrl clear interrupts */

    /* port1 can only be all input or all output */

    /* there are also 2 user input lines and 2 user output lines */

    return 1;
}


/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(rtd520Driver);

