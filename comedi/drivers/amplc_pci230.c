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

************************************************************************

TODO:
fix pci autodetection, add support for bus/slot config options

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
#include <asm/io.h>
#include <linux/comedidev.h>
#include <linux/pci.h>
#include "8253.h"
#include "8255.h"


/* PCI230 PCI configuration register information */

#define PCI_VENDOR_ID_AMPLICON 0x14dc
#define PCI_DEVICE_ID_PCI230 0x0000
#define PCI_DEVICE_ID_PCI260 0x0006

#define PCI230_IO1_SIZE 32		/* Size of I/O space 1 */
#define PCI230_IO2_SIZE 32		/* Size of I/O space 2 - Check this!!! only appears to be 8 16bit words = 16 bytes long*/

/* PCI230 I/O SPACE 1 REGISTERS */
#define PCI230_PPI_X_A   0x00   /* User PPI port A */
#define PCI230_PPI_X_B   0x01   /* User PPI port B */
#define PCI230_PPI_X_C   0x02   /* User PPI port C */
#define PCI230_PPI_X_CMD 0x03   /* User PPI control word */
#define PCI230_Z2_CT0    0x14   /* 82C54 counter/timer 0 */
#define PCI230_Z2_CT1    0x15   /* 82C54 counter/timer 1 */
#define PCI230_Z2_CT2    0x16   /* 82C54 counter/timer 2 */
#define PCI230_Z2_CTC    0x17   /* 82C54 counter/timer control word */
#define PCI230_ZCLK_SCE  0x1A   /* Group Z Clock Configuration Register */
#define PCI230_ZGAT_SCE  0x1D   /* Group Z Gate Configuration Register */
#define PCI230_INT_SCE   0x1E   /* ISR Interrupt source mask register/Interrupt status */
  
/* PCI230 I/O SPACE 2 REGISTERS */
#define PCI230_DACCON  0x00
#define PCI230_DACOUT1 0x02
#define PCI230_DACOUT2 0x04
#define PCI230_DACOUT3 0x06
#define PCI230_ADCDATA 0x08
#define PCI230_ADCCON  0x0A
#define PCI230_ADCEN   0x0C
#define PCI230_ADCG    0x0E

/* CONVERTOR RELATED CONSTANTS */
#define PCI230_DAC_SETTLE 5		/* Analogue output settling time in µS (DAC itself is 1µS nominally) */  
#define PCI230_ADC_SETTLE 1		/* Analogue input settling time in µS (ADC itself is 1.6µS nominally but we poll anyway) */  
#define PCI230_MUX_SETTLE 1		/* ADC MUX settling time in µS - guess */

/* DACCON VALUES */
#define PCI230_DAC_BUSY_BIT		1
#define PCI230_DAC_BIP_BIT		0

/* ADCCON WRITE VALUES */
#define PCI230_ADC_TRIG_NONE	0
#define PCI230_ADC_TRIG_SW 		1
#define PCI230_ADC_TRIG_EXTP	2
#define PCI230_ADC_TRIG_EXTN	3
#define PCI230_ADC_TRIG_Z2CT0	4
#define PCI230_ADC_TRIG_Z2CT1	5
#define PCI230_ADC_TRIG_Z2CT2	6
#define PCI230_ADC_IR_UNI		(0<<3)	/* Input range unipolar */
#define PCI230_ADC_IR_BIP		(1<<3)	/* Input range bipolar */
#define PCI230_ADC_IM_SE		(0<<4)	/* Input mode single ended */
#define PCI230_ADC_IM_DIF		(1<<4)	/* Input mode differential */
#define PCI230_ADC_FIFO_EN		(1<<8)
#define PCI230_ADC_INT_FIFO_EMPTY	0
#define PCI230_ADC_INT_FIFO_NEMPTY	(1<<9)
#define PCI230_ADC_INT_FIFO_NHALF	(2<<9)
#define PCI230_ADC_INT_FIFO_HALF	(3<<9)
#define PCI230_ADC_INT_FIFO_NFULL	(4<<9)
#define PCI230_ADC_INT_FIFO_FULL	(5<<9)
#define PCI230_ADC_FIFO_RESET	(1<<12)
#define PCI230_ADC_GLOB_RESET	(1<<13)
#define PCI230_ADC_CONV	0xffff				/* Value to write to ADCDATA to trigger ADC conversion in sotware trigger mode */
#define PCI230_ADC_SW_CHAN(n)	((n)<<12)	/* ADCCON software trigger channel selection - bit shift channel into upper 4 bit nibble of word */

/* ADCCON READ VALUES */
#define PCI230_ADC_BUSY_BIT		15
#define PCI230_ADC_FIFO_EMPTY	(1<<12)
#define PCI230_ADC_FIFO_FULL	(1<<13)
#define PCI230_ADC_FIFO_HALF	(1<<14)

/* GROUP Z CLOCK CONFIGURATION REGISTER VALUES */
#define PCI230_ZCLK_CT0			0
#define PCI230_ZCLK_CT1			8
#define PCI230_ZCLK_CT2			16
#define PCI230_ZCLK_RES			24
#define PCI230_ZCLK_SRC_PPCN	0 		/* The counter/timer's CLK input from the SK1 connector. */
#define PCI230_ZCLK_SRC_10MHZ	1		/* The internal 10MHz clock. */ 	
#define PCI230_ZCLK_SRC_1MHZ	2		/* The internal 1MHz clock. */ 	 	
#define PCI230_ZCLK_SRC_100KHZ	3		/* The internal 100kHz clock. */ 	 	
#define PCI230_ZCLK_SRC_10KHZ	4		/* The internal 10kHz clock. */  	
#define PCI230_ZCLK_SRC_1KHZ	5		/* The internal 1kHz clock. */  	
#define PCI230_ZCLK_SRC_OUTNM1	6		/* The output of the preceding counter/timer channel (OUT n-1). */ 
#define PCI230_ZCLK_SRC_EXTCLK	7		/* The dedicated external clock input for the group (X1/X2, Y1/Y2, Z1/Z2). */ 

#define PCI230_TIMEBASE_10MHZ	100		/* 10MHz is 100ns. */

/* INTERRUPT ENABLES/STATUS REGISTER VALUES */
#define PCI230_INT_DISABLE		0
#define PCI230_INT_PPI_C0		1
#define PCI230_INT_PPI_C3		2
#define PCI230_INT_ADC_DAC		4
#define PCI230_INT_ZCLK_CT0		32

#define PCI230_TEST_BIT(val, n)	((val>>n)&1)	/* Assumes you number bit with zero offset, ie. 0-15 */

/*
 * Board descriptions for two imaginary boards.  Describing the
 * boards in this way is optional, and completely driver-dependent.
 * Some drivers use arrays such as this, other do not.
 */
typedef struct pci230_board_struct{
	char *name;
	unsigned short id;
	int ai_chans;
	int ai_bits;
	int have_ao;
	int ao_chans;
	int ao_bits;	
	int have_dio;
}pci230_board;
pci230_board pci230_boards[] = {
	{
	name:		"Amplicon PCI230",
	id:		PCI_DEVICE_ID_PCI230,
	ai_chans:	16,
	ai_bits:	12,
	have_ao:	1,
	ao_chans:	2,
	ao_bits:	12,	
	have_dio:	1,
	},
	{
	name:		"Amplicon PCI260",
	id:		PCI_DEVICE_ID_PCI260,
	ai_chans:	16,
	ai_bits:	12,
	have_ao:	0,
	ao_chans:	0,
	ao_bits:	0,
	have_dio:	0,
	},
};
/*
 * Useful for shorthand access to the particular board structure
 */
#define n_pci230_boards (sizeof(pci230_boards)/sizeof(pci230_boards[0]))
#define thisboard ((pci230_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
struct pci230_private{
//	int data;
	struct pci_dev *pci_dev;
	lsampl_t ao_readback[2];  		/* Used for AO readback */
	unsigned int pci_iobase;		/* PCI230's I/O space 1 */	
	/* Divisors for 8254 counter/timer. */    
	unsigned int divisor0;
	unsigned int divisor1;
	unsigned int divisor2;
	unsigned int int_en;			/* Interrupt Enables bits. */	
	volatile unsigned int count;	/* Number of samples remaining. */
	unsigned int bipolar;			/* Set if bipolar range so we know to mangle it in interrupt handler. */
};

#define devpriv ((struct pci230_private *)dev->private)

/* PCI230 analogue input range table */ 
static comedi_lrange pci230_ai_range = { 7, {
	BIP_RANGE(10),
	BIP_RANGE(5),
	BIP_RANGE(2.5),
	BIP_RANGE(1.25),
	UNI_RANGE(10),
	UNI_RANGE(5),
	UNI_RANGE(2.5)
}};

/* PCI230 analogue output range table */ 
static comedi_lrange pci230_ao_range = { 2, {
	UNI_RANGE(10),
	BIP_RANGE(10)
}};

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int pci230_attach(comedi_device *dev,comedi_devconfig *it);
static int pci230_detach(comedi_device *dev);
comedi_driver driver_amplc_pci230={
	driver_name:	"amplc_pci230",
	module:		THIS_MODULE,
	attach:		pci230_attach,
	detach:		pci230_detach,
};
COMEDI_INITCLEANUP(driver_amplc_pci230);

static int pci230_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int pci230_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int pci230_ao_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int pci230_ai_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd);
static int pci230_ai_cmd(comedi_device *dev, comedi_subdevice *s);
static int pci230_ns_to_timer(unsigned int *ns,int round);
static int pci230_z2_ct0(comedi_device *dev, unsigned int *ns,int round);
static int pci230_z2_ct1(comedi_device *dev, unsigned int *ns,int round);
static int pci230_z2_ct2(comedi_device *dev, unsigned int *ns,int round);
static void pci230_interrupt(int irq, void *d, struct pt_regs *regs);
static int pci230__cancel(comedi_device *dev, comedi_subdevice *s);
 
/* print out string and val as bits - up to 16 */
static void printb(int val) {
	printk("\t%d%d%d%d ",PCI230_TEST_BIT(val, 15), PCI230_TEST_BIT(val, 14), PCI230_TEST_BIT(val, 13), PCI230_TEST_BIT(val, 12));
	printk("%d%d%d%d ",	PCI230_TEST_BIT(val, 11), PCI230_TEST_BIT(val, 10), PCI230_TEST_BIT(val,  9), PCI230_TEST_BIT(val,  8));
	printk("%d%d%d%d ",	PCI230_TEST_BIT(val,  7), PCI230_TEST_BIT(val,  6), PCI230_TEST_BIT(val,  5), PCI230_TEST_BIT(val,  4));
	printk("%d%d%d%d\n",PCI230_TEST_BIT(val,  3), PCI230_TEST_BIT(val,  2), PCI230_TEST_BIT(val,  1), PCI230_TEST_BIT(val,  0));
}

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int pci230_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int pci_iobase, iobase = 0;		/* PCI230's I/O spaces 1 and 2 */
	struct pci_dev *pci_dev;
	int i=0,ret;

	printk("comedi%d: amplc_pci230\n",dev->minor);

	/* Find card */
	pci_for_each_dev(pci_dev){
		if(pci_dev->vendor != PCI_VENDOR_ID_AMPLICON)
			continue;
		for(i=0;i<n_pci230_boards;i++){
			if(pci_dev->device == pci230_boards[i].id)break;
		}
		if(i<n_pci230_boards)break;
 		printk("comedi%d: found an unknown Amplicon board, dev id=0x%04x\n",
			dev->minor,pci_dev->device);
	}
	if(!pci_dev){
		printk("comedi%d: amplc_pci230: No PCI230 found\n",dev->minor);
		return -EIO;
	}
	dev->board_ptr = pci230_boards+i;
	
	/* Read base addressses of the PCI230's two I/O regions from PCI configuration register */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,3,0)
	pci_iobase = pci_dev->base_address[2] & PCI_BASE_ADDRESS_IO_MASK;
	iobase = pci_dev->base_address[3] & PCI_BASE_ADDRESS_IO_MASK;
#else
	if(pci_enable_device(pci_dev))
		return -EIO;
	pci_iobase = pci_dev->resource[2].start & PCI_BASE_ADDRESS_IO_MASK;
	iobase = pci_dev->resource[3].start & PCI_BASE_ADDRESS_IO_MASK;
#endif

	printk("comedi%d: amplc_pci230: I/O region 1 0x%04x I/O region 2 0x%04x\n",dev->minor, pci_iobase, iobase);

	/* Allocate the private structure area using alloc_private() (macro defined in comedidev.h.) */
	if((alloc_private(dev,sizeof(struct pci230_private)))<0)
		return -ENOMEM;
	devpriv->pci_dev = pci_dev;

	/* Reserve I/O space 1. */
	if(check_region(pci_iobase,PCI230_IO1_SIZE)<0){
		printk("comedi%d: amplc_pci230: I/O space 1 conflict\n",dev->minor);
		return -EIO;
	}
	request_region(pci_iobase,PCI230_IO1_SIZE,"PCI230");
	devpriv->pci_iobase = pci_iobase;

	/* Reserve I/O space 2. */
	if(check_region(iobase,PCI230_IO2_SIZE)<0){
		printk("comedi%d: amplc_pci230: I/O space 2 conflict\n",dev->minor);
		return -EIO;
	}
	request_region(iobase,PCI230_IO2_SIZE,"PCI230");
	dev->iobase = iobase;

/*
 * Initialize dev->board_name.  Note that we can use the "thisboard"
 * macro now, since we just initialized it in the last line.
 */
	dev->board_name = thisboard->name;


/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.  It relies on
 * n_subdevices being set correctly.
 */
	dev->n_subdevices=3;
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;

	s=dev->subdevices+0;
	dev->read_subdev=s;
	/* analog input subdevice */
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=thisboard->ai_chans;
	s->maxdata=(1<<thisboard->ai_bits)-1;
	s->range_table=&pci230_ai_range;
	s->insn_read = &pci230_ai_rinsn;
	s->do_cmd = &pci230_ai_cmd;
	s->len_chanlist = thisboard->ai_chans;
	s->do_cmdtest = &pci230_ai_cmdtest;

	s=dev->subdevices+1;
	/* analog output subdevice */
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=thisboard->ao_chans;;
	s->maxdata=(1<<thisboard->ao_bits)-1;
	s->range_table=&pci230_ao_range;
	s->insn_write = &pci230_ao_winsn;
	s->insn_read = &pci230_ao_rinsn;

	s=dev->subdevices+2;
	/* digital i/o subdevice */
	if(thisboard->have_dio){
		subdev_8255_init(dev,s,NULL,(void *)(devpriv->pci_iobase + PCI230_PPI_X_A));
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}

	/* Register the interrupt handler. */
	ret = comedi_request_irq(devpriv->pci_dev->irq, pci230_interrupt, 0, "PCI230", dev );
	if(ret<0)
	{
		printk("comedi%d: amplc_pci230: unable to register irq %d\n", dev->minor, devpriv->pci_dev->irq);
		return ret;
	}
	dev->irq = devpriv->pci_dev->irq;
	printk("comedi%d: amplc_pci230: registered irq %d\n", dev->minor, devpriv->pci_dev->irq);

	printk("attached\n");

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
static int pci230_detach(comedi_device *dev)
{
	printk("comedi%d: amplc_pci230: remove\n",dev->minor);

	if(dev->subdevices && thisboard->have_dio)
		subdev_8255_cleanup(dev,dev->subdevices + 2);	/* Clean up dio subdevice. */

	if(dev->iobase)
		release_region(dev->iobase,PCI230_IO2_SIZE);

	if(dev->irq)
		comedi_free_irq(dev->irq, dev);

	if(devpriv){
		if(devpriv->pci_iobase){
			release_region(devpriv->pci_iobase, PCI230_IO1_SIZE);
		}
	}
	
	return 0;
}

/*
 * "instructions" read/write data in "one-shot" or "software-triggered"
 * mode.
 */
static int pci230_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n,i;
	int chan, range;
	unsigned int d;
	unsigned int status;
	unsigned int adccon, adcen, adcg;
	unsigned int bipolar;

	/* Unpack channel and range. */
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);
	
	printk("\ncomedi%d: amplc_pci230::pci230_ai_rinsn() chan %d range %d\n",dev->minor, chan, range);

	/* If bit 2 of range unset, range is referring to bipolar element in range table */
	bipolar = !PCI230_TEST_BIT(range, 2);
	adccon = PCI230_ADC_IM_SE | PCI230_ADC_TRIG_SW | PCI230_ADC_SW_CHAN(chan);
	if (bipolar) {
		adccon = adccon | PCI230_ADC_IR_BIP;
		adcg = range<<(chan-chan%2);
	}
	else {
		adccon = adccon | PCI230_ADC_IR_UNI;
		adcg = ((range&(~4))+1)<<(chan-chan%2);
	}
	adcen = 1<<chan;

	/* Specify uni/bip, se/diff, s/w conversion and channel. */
	outw_p(adccon, dev->iobase + PCI230_ADCCON);
	printk("comedi%d: amplc_pci230::pci230_ai_rinsn() wrote PCI230_ADCCON",dev->minor); printb(adccon);

	/* Enable only this channel in the scan list - otherwise by default we'll get one sample from each channel. */
	outw_p(adcen, dev->iobase + PCI230_ADCEN);
	printk("comedi%d: amplc_pci230::pci230_ai_rinsn() wrote PCI230_ADCEN", dev->minor); printb(adcen);

	/* Set gain for channel. */
	outw_p(adcg, dev->iobase + PCI230_ADCG);
	printk("comedi%d: amplc_pci230::pci230_ai_rinsn() wrote PCI230_ADCG", dev->minor); printb(adcg);

	/* Wait for mux to settle */
	udelay(PCI230_MUX_SETTLE);

	/* Convert n samples */
	for(n=0;n<insn->n;n++){
		/* trigger conversion */
		outw_p(PCI230_ADC_CONV,dev->iobase + PCI230_ADCDATA);

#define TIMEOUT 100
		/* wait for conversion to end */
		for(i=0;i<TIMEOUT;i++){
			status = inw(dev->iobase + PCI230_ADCCON);
			printk("comedi%d: amplc_pci230::pci230_ai_rinsn() read PCI230_ADCCON",dev->minor); printb(status);
			if(PCI230_TEST_BIT(status, PCI230_ADC_BUSY_BIT))break;
		}
		if(i==TIMEOUT){
			/* rt_printk() should be used instead of printk()
			 * whenever the code can be called from real-time. */
			rt_printk("timeout\n");
			return -ETIMEDOUT;
		}

		/* read data */
		d = inw(dev->iobase + PCI230_ADCDATA);
		printk("comedi%d: amplc_pci230::pci230_ai_rinsn() read PCI230_ADCDATA 0x%04x\n",dev->minor, d);

		/* PCI230 is 12 bit - stored in upper bits of 16 bit register (lower four bits reserved for expansion). */
		d = d>>4;

		/* If a bipolar range was specified, mangle it (twos complement->straight binary). */ 
		if (bipolar) {
			d ^= 1<<(thisboard->ai_bits-1);
		}
		data[n] = d;
	}

	/* return the number of samples read/written */
	return n;
}

static int pci230_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd)
{
	int err=0;
	int tmp;

	printk("comedi%d: amplc_pci230::pci230_ai_cmdtest()\n",dev->minor);

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* Step 1: make sure trigger sources are trivially valid.
	 * "invalid source" returned by comedilib to user mode process 
	 * if this fails. */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER | TRIG_EXT;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT | TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* Step 2: make sure trigger sources are unique and mutually compatible
	 * "source conflict" returned by comedilib to user mode process 
	 * if this fails. */

	if(cmd->start_src!=TRIG_NOW &&
	   cmd->start_src!=TRIG_EXT)err++;
	if(cmd->scan_begin_src!=TRIG_TIMER &&
	   cmd->scan_begin_src!=TRIG_EXT)err++;
	if(cmd->convert_src!=TRIG_TIMER &&
	   cmd->convert_src!=TRIG_EXT)err++;
	if(cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_NONE)err++;
	
	/* XXX Should check here if we aren't able to have multiple
	 * sources as TRIG_EXT */

	if(err)return 2;

	/* Step 3: make sure arguments are trivially compatible.
	 * "invalid argument" returned by comedilib to user mode process 
	 * if this fails. */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}

#define MAX_SPEED	10000		/* in nanoseconds */
#define MIN_SPEED	1000000000	/* in nanoseconds */

	if(cmd->scan_begin_src==TRIG_TIMER){
		if(cmd->scan_begin_arg<MAX_SPEED){
			cmd->scan_begin_arg=MAX_SPEED;
			err++;
		}
		if(cmd->scan_begin_arg>MIN_SPEED){
			cmd->scan_begin_arg=MIN_SPEED;
			err++;
		}
	}else{
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		/* should specify multiple external triggers */
		/* XXX Do you really have 10 available triggering channels? */
		if(cmd->scan_begin_arg>9){
			cmd->scan_begin_arg=9;
			err++;
		}
	}
	if(cmd->convert_src==TRIG_TIMER){
		if(cmd->convert_arg<MAX_SPEED){
			cmd->convert_arg=MAX_SPEED;
			err++;
		}
		if(cmd->convert_arg>MIN_SPEED){
			cmd->convert_arg=MIN_SPEED;
			err++;
		}
	}else{
		/* external trigger */
		/* see above */
		if(cmd->convert_arg>9){
			cmd->convert_arg=9;
			err++;
		}
	}

	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src==TRIG_COUNT){
		if(cmd->stop_arg>0x00ffffff){
			cmd->stop_arg=0x00ffffff;
			err++;
		}
	}else{
		/* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;

	/* Step 4: fix up any arguments.
	 * "argument conflict" returned by comedilib to user mode process 
	 * if this fails. */

	if(cmd->scan_begin_src==TRIG_TIMER){
		tmp=cmd->scan_begin_arg;
		pci230_ns_to_timer(&cmd->scan_begin_arg,cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->scan_begin_arg)err++;
	}
	if(cmd->convert_src==TRIG_TIMER){
		tmp=cmd->convert_arg;
		pci230_ns_to_timer(&cmd->convert_arg,cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->convert_arg)err++;
		if(cmd->scan_begin_src==TRIG_TIMER &&
		  cmd->scan_begin_arg<cmd->convert_arg*cmd->scan_end_arg){
			cmd->scan_begin_arg=cmd->convert_arg*cmd->scan_end_arg;
			err++;
		}
	}

	if(err)return 4;

	return 0;
}

static int pci230_ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	int i, chan, range, aref;
	unsigned int adccon, adcen, adcg;

	/* Get the command. */
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;

	/* Print the command */
	printk("comedi%d: amplc_pci230::pci230_ai_cmd()\n",dev->minor);
	printk("\tflags \t\t %d\n", cmd->flags);
	printk("\tstart \t\t src %d arg %d\n", cmd->start_src, cmd->start_arg);
	printk("\tscan_begin \t src %d arg %d\n", cmd->scan_begin_src, cmd->scan_begin_arg);
	printk("\tconvert \t src %d arg %d\n", cmd->convert_src, cmd->convert_arg);
	printk("\tscan_end \t src %d arg %d\n", cmd->scan_end_src, cmd->scan_end_arg);
	printk("\tstop \t\t src %d arg %d\n", cmd->stop_src, cmd->stop_arg);
	for (chan = 0; chan < cmd->chanlist_len; chan++) {
		printk("\tchannel %d\t range %d\t aref %d\n", CR_CHAN(cmd->chanlist[chan]), CR_RANGE(cmd->chanlist[chan]), CR_AREF(cmd->chanlist[chan]));
	}

	if(!dev->irq)
	{
		/* XXX Instead of checking here, just don't set do_cmd in
		 * _attach() */
		comedi_error(dev, "no irq assigned for PCI230, cannot do hardware conversions");
		return -1;
	}

	/* Calculate number of conversions required. */
	if(cmd->stop_src == TRIG_COUNT) {
		devpriv->count = cmd->stop_arg * cmd->chanlist_len;
		printk("comedi%d: amplc_pci230::pci230_ai_cmd() total number of conversions %d\n",dev->minor, devpriv->count);
	}
	else {
		devpriv->count = 0;
	}

	/* Steps;
	 * - Disable board interrupts.
	 * - Reset FIFO, specify uni/bip, se/diff, and start conversion source to none.
	 * - Set channel scan list.
	 * - Set channel gains.
	 * - Set the counter timers to the specified sampling frequency.
	 * - Enable conversion complete interrupt.
	 * - Enable FIFO, set FIFO interrupt trigger level, set start conversion source to counter 1.
	 */

	/* Disable interrupts. */
	outb(PCI230_INT_DISABLE, devpriv->pci_iobase + PCI230_INT_SCE);
	printk("comedi%d: amplc_pci230::pci230_ai_cmd wrote PCI230_INT_SCE",dev->minor); printb(PCI230_INT_DISABLE);

	adccon = PCI230_ADC_FIFO_RESET | PCI230_ADC_IM_SE | PCI230_ADC_TRIG_NONE;
	adcg = 0;
	adcen = 0;

	/* If bit 2 of range unset, range is referring to bipolar element in range table */
	/* XXX set range? */
	range = CR_RANGE(cmd->chanlist[0]);
	devpriv->bipolar = !PCI230_TEST_BIT(range, 2);	
	if (devpriv->bipolar) {
		adccon |= PCI230_ADC_IR_BIP;
		for (i = 0; i < cmd->chanlist_len; i++) {
			chan = CR_CHAN(cmd->chanlist[i]);
			range = CR_RANGE(cmd->chanlist[i]);
			adcg |= range<<(chan-chan%2);
			adcen |= 1<<chan;
		}
	}
	else {
		adccon |= PCI230_ADC_IR_UNI;
		for (i = 0; i < cmd->chanlist_len; i++) {
			chan = CR_CHAN(cmd->chanlist[i]);
			range = CR_RANGE(cmd->chanlist[i]);
			adcg |= ((range&(~4))+1)<<(chan-chan%2);
			adcen |= 1<<chan;
		}
	}

	/* Reset FIFO, specify uni/bip, se/diff, and start conversion source to none. */
	outw_p(adccon, dev->iobase + PCI230_ADCCON);
	printk("comedi%d: amplc_pci230::pci230_ai_cmd wrote PCI230_ADCCON",dev->minor); printb(adccon);

	/* Set channel scan list. */
	outw_p(adcen, dev->iobase + PCI230_ADCEN);
	printk("comedi%d: amplc_pci230::pci230_ai_cmd wrote PCI230_ADCEN", dev->minor); printb(adcen);

	/* Set channel gains. */
	outw_p(adcg, dev->iobase + PCI230_ADCG);
	printk("comedi%d: amplc_pci230::pci230_ai_cmd wrote PCI230_ADCG", dev->minor); printb(adcg);

	/* Set the counter timers to the specified sampling frequency. */
	pci230_z2_ct1(dev, &cmd->convert_arg, cmd->flags & TRIG_ROUND_MASK);

	/* Enable conversion complete interrupt. */
	outb(PCI230_INT_ADC_DAC, devpriv->pci_iobase + PCI230_INT_SCE);
	printk("comedi%d: amplc_pci230::pci230_ai_cmd wrote PCI230_INT_SCE",dev->minor); printb(PCI230_INT_ADC_DAC);

	/* Enable FIFO, set FIFO interrupt trigger level, set start conversion source to counter 1. */
	adccon = (adccon & ~PCI230_ADC_FIFO_RESET) | PCI230_ADC_FIFO_EN | PCI230_ADC_TRIG_Z2CT1;
	if (devpriv->count < 2048) {
		adccon = adccon | PCI230_ADC_INT_FIFO_NEMPTY;
	}
	else {
		adccon = adccon | PCI230_ADC_INT_FIFO_HALF;
	}
	outw_p(adccon, dev->iobase + PCI230_ADCCON);
	printk("comedi%d: amplc_pci230::pci230_ai_cmd wrote PCI230_ADCCON",dev->minor); printb(adccon);

	return 0;
}


/* This function doesn't require a particular form, this is just
 * what happens to be used in some of the drivers.  It should
 * convert ns nanoseconds to a counter value suitable for programming
 * the device.  Also, it should adjust ns so that it cooresponds to
 * the actual time that the device will use. */
static int pci230_ns_to_timer(unsigned int *ns,int round)
{
	unsigned int divisor0, divisor1;
	i8253_cascade_ns_to_timer_2div(PCI230_TIMEBASE_10MHZ, &divisor0, &divisor1, ns, TRIG_ROUND_MASK);
	printk("comedi: amplc_pci230::pci230_ns_to_timer divisor0 %d divisor1 %d ns %d\n",divisor0, divisor1, *ns);
	return *ns;
}

/* 
 *  Set ZCLK_CT0 to square wave mode with period of ns.
 *  Default clk source for DAC.
 */
static int pci230_z2_ct0(comedi_device *dev, unsigned int *ns,int round)
{
	/* For two cascaded counter/timers, calculate the divide ratios required to give a square wave of period ns. */
	i8253_cascade_ns_to_timer_2div(PCI230_TIMEBASE_10MHZ, &devpriv->divisor2, &devpriv->divisor0, ns, TRIG_ROUND_MASK);
	printk("comedi%d: amplc_pci230::pci230_z2_ct0() divisor2 %d divisor0 %d ns %d\n",dev->minor, devpriv->divisor2, devpriv->divisor0, *ns);

    /* Generic i8254_load calls; program counters' divide ratios. */
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 2, devpriv->divisor2, 2);	/* Counter 2, divisor2, square wave (8254 mode 2). */
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 0, devpriv->divisor0, 2);	/* Counter 0, divisor0, square wave (8254 mode 2). */

	/* PCI 230 specific - ties up counter clk inputs with clk sources */
	outb(PCI230_ZCLK_CT2 | PCI230_ZCLK_SRC_10MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 2's input clock source. */
	outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_OUTNM1, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */

	return *ns;
}

/* 
 *  Set ZCLK_CT1 to square wave mode with period of ns.
 *  Default clk source for ADC.
 */
static int pci230_z2_ct1(comedi_device *dev, unsigned int *ns,int round)
{
	/* For two cascaded counter/timers, calculate the divide ratios required to give a square wave of period ns. */
	i8253_cascade_ns_to_timer_2div(PCI230_TIMEBASE_10MHZ, &devpriv->divisor0, &devpriv->divisor1, ns, TRIG_ROUND_MASK);
	printk("comedi%d: amplc_pci230::pci230_z2_ct1() divisor0 %d divisor1 %d ns %d\n",dev->minor, devpriv->divisor0, devpriv->divisor1, *ns);

    /* Generic i8254_load calls; program counters' divide ratios. */
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 0, devpriv->divisor0, 2);	/* Counter 0, divisor0, square wave (8254 mode 2). */
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 1, devpriv->divisor1, 2);	/* Counter 1, divisor1, square wave (8254 mode 2). */

	/* PCI 230 specific - ties up counter clk inputs with clk sources */
	outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_10MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
	outb(PCI230_ZCLK_CT1 | PCI230_ZCLK_SRC_OUTNM1, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 1's input clock source. */

	return *ns;
}

/* 
 *  Set ZCLK_CT2 to square wave mode with period of ns.
 */
static int pci230_z2_ct2(comedi_device *dev, unsigned int *ns,int round)
{
	/* For two cascaded counter/timers, calculate the divide ratios required to give a square wave of period ns. */
	i8253_cascade_ns_to_timer_2div(PCI230_TIMEBASE_10MHZ, &devpriv->divisor1, &devpriv->divisor2, ns, TRIG_ROUND_MASK);
	printk("comedi%d: amplc_pci230::pci230_z2_ct2() divisor1 %d divisor2 %d ns %d\n",dev->minor, devpriv->divisor1, devpriv->divisor2, *ns);

    /* Generic i8254_load calls; program counters' divide ratios. */
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 1, devpriv->divisor1, 2);	/* Counter 1, divisor1, square wave (8254 mode 2). */
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 2, devpriv->divisor2, 2);	/* Counter 2, divisor2, square wave (8254 mode 2). */

	/* PCI 230 specific - ties up counter clk inputs with clk sources */
	outb(PCI230_ZCLK_CT1 | PCI230_ZCLK_SRC_10MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 1's input clock source. */
	outb(PCI230_ZCLK_CT2 | PCI230_ZCLK_SRC_OUTNM1, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 2's input clock source. */

	return *ns;
}

static int pci230_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i;
	int chan, range;
	unsigned int d;
	unsigned int bipolar;
	
	/* Unpack channel and range. */
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);

	/* Set range - see analogue output range table; 0 => unipolar 10V, 1 => bipolar +/-10V range scale */
	bipolar = PCI230_TEST_BIT(range, PCI230_DAC_BIP_BIT);
	outw(range, dev->iobase + PCI230_DACCON);

	/* Writing a list of values to an AO channel is probably not
	 * very useful, but that's how the interface is defined. */
	for(i=0;i<insn->n;i++){
		d = data[i];

		/* Store the value to be written to the DAC in our pci230_private struct before mangling it. */
		devpriv->ao_readback[chan] = d;

		/* If a bipolar range was specified, mangle it (straight binary->twos complement). */ 
		if (bipolar) {
			d ^= 1<<(thisboard->ai_bits-1);
		}
		
		/* PCI230 is 12 bit - stored in upper bits of 16 bit register (lower four bits reserved for expansion). */
		d = d<<4;
		
		/* Write data. */
   		outw(d, dev->iobase + (((chan) == 0) ? PCI230_DACOUT1 : PCI230_DACOUT2));
		printk("comedi%d: amplc_pci230::pci230_ao_rinsn() wrote PCI230_DACOUTx 0x%04x\n",dev->minor, d);

#if 0
		/* XXX screw the user.  Only do this if the board gets upset if you don't */
		/* If we're writing more than one sample, wait for output to settle between successive writes */
		if (insn->n > 1) {
	    	udelay(PCI230_DAC_SETTLE);
		}
#endif
	}

	/* return the number of samples read/written */
	return i;
}

/* AO subdevices should have a read insn as well as a write insn.
 * Usually this means copying a value stored in devpriv. */
static int pci230_ao_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	for(i=0;i<insn->n;i++)
		data[i] = devpriv->ao_readback[chan];

	return i;
}

/* Interrupt handler */
static void pci230_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev = (comedi_device*) d;
	comedi_subdevice *s = dev->read_subdev;
	comedi_async *async;
	int status_int, status_fifo;
	int i;
	unsigned int data;

	/* 
	 * Check to see whether this driver has been configured yet.
	 * This must be done first, because if the board is asserting
	 * interrupts and our driver's attach fn. hasn't been called
	 * we can't even talk to the board (base addresses for native
	 * IO regions aren't set...)
	 */

	/* XXX doesn't make sense, since interrupt is allocated in the
	 * _attach function, _after_ the IO regions are allocated */

	/* XXX I (Frank Hess) do this in drivers to prevent a null
	 * dereference when the handler tries to use async.  This
	 * probably won't help too much here as it is though, since
	 * it doesn't clear the interrupt before returning and so
	 * will lock up the (single cpu) computer. */

	if(dev->attached == 0) {
		comedi_error(dev, "premature interrupt");
		return;
	}

	printk("comedi%d: amplc_pci230::pci230_interrupt executing interrupt handler\n", dev->minor);

	/* Read interrupt status/enable register. */
	status_int = inb(devpriv->pci_iobase + PCI230_INT_SCE);
	printk("comedi%d: amplc_pci230::pci230_interrupt read PCI230_INT_SCE",dev->minor); printb(status_int);

	/* Disable board interrupts. */
	outb(PCI230_INT_DISABLE, devpriv->pci_iobase + PCI230_INT_SCE);
	printk("comedi%d: amplc_pci230::pci230_interrupt wrote PCI230_INT_SCE",dev->minor); printb(PCI230_INT_DISABLE);

	/* 
	 * Check to see whether this board emitted the interrupt, and if it did,  
	 * whether it is for a supported function.
	 */
	if (status_int == PCI230_INT_DISABLE) {
		printk("comedi%d: amplc_pci230::pci230_interrupt spurious interrupt",dev->minor);
		return;
	}
	else if (status_int & PCI230_INT_PPI_C0) {
		printk("comedi%d: amplc_pci230::pci230_interrupt PPI C0 interrupt, not implemented",dev->minor);
		return;
	}
	else if (status_int & PCI230_INT_PPI_C3) {
		printk("comedi%d: amplc_pci230::pci230_interrupt PPI C3 interrupt, not implemented",dev->minor);
		return;
	}
	else if (status_int & PCI230_INT_ZCLK_CT0) {
		printk("comedi%d: amplc_pci230::pci230_interrupt ZCLK CT0 interrupt, not implemented",dev->minor);
		return;
	}

	/* Read FIFO state. */
	status_fifo = inw(dev->iobase + PCI230_ADCCON);
	printk("comedi%d: amplc_pci230::pci230_interrupt read PCI230_ADCCON",dev->minor); printb(status_fifo);
	
	/* Check to see whether FIFO enabled. */
	if (!(status_fifo & PCI230_ADC_FIFO_EN)) {
		printk("comedi%d: amplc_pci230::pci230_interrupt FIFO not enabled",dev->minor);
		return;
	}

	/* Ok., only reason we're here is because;
	 * - Board has raised a conversion complete interrupt.
	 * - FIFO is enabled.
	 */

	async = s->async;
	async->events = 0;

	if (status_fifo & PCI230_ADC_FIFO_FULL) {
		/* 
		 * Report error and return - if we didn´t do this, but instead handled it in a similar
		 * manner to the half full FIFO case, FIFO overruns would go unnoticed by the caller.
		 */
		printk("comedi%d: amplc_pci230::pci230_interrupt FIFO overflow",dev->minor);

		/* Cancel sampled conversion. */
		pci230__cancel(dev, s);	
		comedi_error(dev, "FIFO overrun");
		async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
		comedi_event(dev, s, async->events);
		async->events = 0;
		return;
	}
	else if (status_fifo & PCI230_ADC_FIFO_HALF) {
		/* FIFO is at least half full */
		printk("comedi%d: amplc_pci230::pci230_interrupt FIFO half full\n",dev->minor);
		for (i = 0; i < 2048; i++) {
			/* Read sample. */
			data = inw(dev->iobase + PCI230_ADCDATA);

			/* PCI230 is 12 bit - stored in upper bits of 16 bit register (lower four bits reserved for expansion). */
			data = data>>4;

			/* If a bipolar range was specified, mangle it (twos complement->straight binary). */ 
			if (devpriv->bipolar) {
				data ^= 1<<(thisboard->ai_bits-1);
			}

			/* Store in Comedi's circular buffer. */
			comedi_buf_put(async, (sampl_t) data);

			if(async->cmd.stop_src == TRIG_COUNT)
			{
				if(--devpriv->count == 0) {
					/* Acquisition complete. */
					printk("comedi%d: amplc_pci230::pci230_interrupt acquisition complete\n",dev->minor);
					pci230__cancel(dev, s);
					async->events |= COMEDI_CB_EOA;
					break;
				}
			}
		}
		/* More samples required, tell Comedi to block, and enable boards interrupt (don´t change trigger level). */
		async->events |= COMEDI_CB_BLOCK;
		outb(PCI230_INT_ADC_DAC, devpriv->pci_iobase + PCI230_INT_SCE);
		printk("comedi%d: amplc_pci230::pci230_interrupt wrote PCI230_INT_SCE\n",dev->minor); printb(PCI230_INT_ADC_DAC);
	}
	else if (status_fifo & PCI230_ADC_FIFO_EMPTY) {
		/* FIFO empty but we got an interrupt */
		printk("comedi%d: amplc_pci230::pci230_interrupt FIFO empty - spurious interrupt\n",dev->minor);
	}
	else {
		/* FIFO is less than half full, but not empty. */
		/* We should only ever get here if no. samples to read < half fifo size. */
		printk("comedi%d: amplc_pci230::pci230_interrupt FIFO less than half full, but not empty\n",dev->minor);
		while (devpriv->count != 0) {
			if (inw(dev->iobase + PCI230_ADCCON) & PCI230_ADC_FIFO_EMPTY) {
				/* The FIFO is empty, block. */
				printk("comedi%d: amplc_pci230::pci230_interrupt FIFO now empty, want %d samples\n",dev->minor, devpriv->count);

				/* More samples required, tell Comedi to block, and enable boards interrupt (don´t change trigger level). */
				outb(PCI230_INT_ADC_DAC, devpriv->pci_iobase + PCI230_INT_SCE);
				printk("comedi%d: amplc_pci230::pci230_interrupt wrote PCI230_INT_SCE\n",dev->minor); printb(PCI230_INT_ADC_DAC);
				async->events |= COMEDI_CB_BLOCK;
				comedi_event(dev, s, async->events);
				return;
			}
			/* There are sample(s) to read from FIFO, read one. */
			data = inw(dev->iobase + PCI230_ADCDATA);

			/* PCI230 is 12 bit - stored in upper bits of 16 bit register (lower four bits reserved for expansion). */
			data = data>>4;

			/* If a bipolar range was specified, mangle it (twos complement->straight binary). */ 
			if (devpriv->bipolar) {
				data ^= 1<<(thisboard->ai_bits-1);
			}

			/* Store in Comedi's circular buffer. */
			comedi_buf_put(async, (sampl_t) data);

			if(devpriv->count > 0) devpriv->count--;
		}

		/* Acquisition complete. */
		printk("comedi%d: amplc_pci230::pci230_interrupt acquisition complete\n",dev->minor);
		pci230__cancel(dev, s);
		async->events |= COMEDI_CB_EOA;
	}

	comedi_event(dev, s, async->events);
	async->events = 0;

	return;
}

static int pci230__cancel(comedi_device *dev, comedi_subdevice *s)
{
	/* Disable interrupts. */
	outb(PCI230_INT_DISABLE, devpriv->pci_iobase + PCI230_INT_SCE);
	printk("comedi%d: amplc_pci230::pci230__cancel disabled interrupts\n",dev->minor);

	/* Reset FIFO and set start conversion source to none. */
	outw_p(PCI230_ADC_FIFO_RESET | PCI230_ADC_TRIG_NONE, dev->iobase + PCI230_ADCCON);
	printk("comedi%d: amplc_pci230::pci230__cancel reset FIFO and set conv src to none\n", dev->minor);

	/* Clear channel scan list. */
	outw_p(0x0000, dev->iobase + PCI230_ADCEN);
	printk("comedi%d: amplc_pci230::pci230__cancel cleared scan list\n", dev->minor);

	/* Clear channel gains. */
	outw_p(0x0000, dev->iobase + PCI230_ADCG);
	printk("comedi%d: amplc_pci230::pci230__cancel cleared channel gains\n", dev->minor);

	return 0;
}


