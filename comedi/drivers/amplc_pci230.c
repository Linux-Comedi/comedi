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
Driver: amplc_pci230.o
Description: Driver for Amplicom PCI230 and PCI260 Multifunction I/O boards
Author: Allan Willcox <allanwillcox@ozemail.com.au>, Steve D Sharples <steve.sharples@nottingham.ac.uk>
Updated: Fri,  30 April 2004
Devices: [Amplicon] PCI230 (amplc_pci230), PCI260
Status: works

*/
/*
extra triggered scan functionality, interrupt bug-fix added by Steve Sharples 
*/
#include <linux/comedidev.h>

#include <linux/delay.h>
#include <linux/pci.h>

#include "8253.h"
#include "8255.h"


/* PCI230 PCI configuration register information */
#define PCI_VENDOR_ID_AMPLICON 0x14dc
#define PCI_DEVICE_ID_PCI230 0x0000
#define PCI_DEVICE_ID_PCI260 0x0006

#define PCI230_IO1_SIZE 32		/* Size of I/O space 1 */
#define PCI230_IO2_SIZE 16		/* Size of I/O space 2 */

/* PCI230 i/o space 1 registers. */
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
  
/* PCI230 i/o space 2 registers. */
#define PCI230_DACCON  0x00
#define PCI230_DACOUT1 0x02
#define PCI230_DACOUT2 0x04
#define PCI230_DACOUT3 0x06
#define PCI230_ADCDATA 0x08
#define PCI230_ADCCON  0x0A
#define PCI230_ADCEN   0x0C
#define PCI230_ADCG    0x0E

/* Convertor related constants. */
#define PCI230_DAC_SETTLE 5		/* Analogue output settling time in 탎 (DAC itself is 1탎 nominally). */  
#define PCI230_ADC_SETTLE 1		/* Analogue input settling time in 탎 (ADC itself is 1.6탎 nominally but we poll anyway). */  
#define PCI230_MUX_SETTLE 10	/* ADC MUX settling time in 킪 - 10탎 for se, 20탎 de. */

/* DACCON values. */
#define PCI230_DAC_BUSY_BIT		1
#define PCI230_DAC_BIP_BIT		0

/* ADCCON write values. */
#define PCI230_ADC_TRIG_NONE		0
#define PCI230_ADC_TRIG_SW 		1
#define PCI230_ADC_TRIG_EXTP		2
#define PCI230_ADC_TRIG_EXTN		3
#define PCI230_ADC_TRIG_Z2CT0		4
#define PCI230_ADC_TRIG_Z2CT1		5
#define PCI230_ADC_TRIG_Z2CT2		6
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
#define PCI230_ADC_FIFO_RESET		(1<<12)
#define PCI230_ADC_GLOB_RESET		(1<<13)
#define PCI230_ADC_CONV				0xffff		/* Value to write to ADCDATA to trigger ADC conversion in sotware trigger mode */

/* ADCCON read values. */
#define PCI230_ADC_BUSY_BIT		15
#define PCI230_ADC_FIFO_EMPTY		(1<<12)
#define PCI230_ADC_FIFO_FULL		(1<<13)
#define PCI230_ADC_FIFO_HALF		(1<<14)

/* Group Z clock configuration register values. */
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

/* Group Z gate configuration register values. */
#define	PCI230_ZGAT_CT0			0
#define	PCI230_ZGAT_CT1			8
#define	PCI230_ZGAT_CT2			16
#define	PCI230_ZGAT_RES			24
#define	PCI230_ZGAT_SRC_VCC	0		/* The counter/timer's GAT input is VCC (ie enabled) */
#define	PCI230_ZGAT_SRC_GND	1		/* GAT input is GND (ie disabled) */
#define	PCI230_ZGAT_SRC_PPCN	2		/* GAT input is DIO port Cn, where n is the number of the counter/timer */
#define	PCI230_ZGAT_SRC_OUTNP1	3		/* GAT input is the output of the next counter/timer channel (OUT n+1) */


#define PCI230_TIMEBASE_10MHZ	100		/* 10MHz  =>     100ns. */
#define PCI230_TIMEBASE_1MHZ	1000		/* 1MHz   =>    1000ns. */
#define PCI230_TIMEBASE_100KHZ	10000		/* 100kHz =>   10000ns. */
#define PCI230_TIMEBASE_10KHZ	100000		/* 10kHz  =>  100000ns. */
#define PCI230_TIMEBASE_1KHZ	1000000		/* 1kHz   => 1000000ns. */


/* Interrupt enables/status register values. */
#define PCI230_INT_DISABLE		0
#define PCI230_INT_PPI_C0		1
#define PCI230_INT_PPI_C3		2
#define PCI230_INT_ADC			4
#define PCI230_INT_ZCLK_CT1		32

#define PCI230_TEST_BIT(val, n)	((val>>n)&1)	/* Assumes bits numbered with zero offset, ie. 0-15 */

/*
 * Board descriptions for the two boards supported.
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
static pci230_board pci230_boards[] = {
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

static struct pci_device_id pci230_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_PCI230, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_PCI260, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, pci230_pci_table);
/*
 * Useful for shorthand access to the particular board structure
 */
#define n_pci230_boards (sizeof(pci230_boards)/sizeof(pci230_boards[0]))
#define thisboard ((pci230_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
struct pci230_private{
	struct pci_dev *pci_dev;
	lsampl_t ao_readback[2];  	/* Used for AO readback */
	unsigned int pci_iobase;	/* PCI230's I/O space 1 */	
	/* Divisors for 8254 counter/timer. */    
	unsigned int clk_src0;		/* which clock to use for the counter/timers: 10MHz, 1MHz, 100kHz etc */
	unsigned int clk_src1;
	unsigned int clk_src2;
	unsigned int divisor0;
	unsigned int divisor1;
	unsigned int divisor2;
	unsigned int int_en;		/* Interrupt enables bits. */	
	unsigned int ai_count;		/* Number of analogue input samples remaining. */
	unsigned int ao_count;		/* Number of analogue output samples remaining. */
	unsigned int ai_stop;  		/* Flag set when cmd->stop_src == TRIG_NONE - user chooses to stop continuous conversion by cancelation. */
	unsigned int ao_stop;  		/* Flag set when cmd->stop_src == TRIG_NONE - user chooses to stop continuous conversion by cancelation. */
	unsigned int ai_bipolar;	/* Set if bipolar input range so we know to mangle it. */
	unsigned int ao_bipolar;	/* Set if bipolar output range so we know to mangle it. */
	unsigned int ier;		/* Copy of interrupt enables/status register. */
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
static comedi_driver driver_amplc_pci230={
	driver_name:	"amplc_pci230",
	module:		THIS_MODULE,
	attach:		pci230_attach,
	detach:		pci230_detach,
};
COMEDI_INITCLEANUP(driver_amplc_pci230);

static int pci230_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int pci230_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int pci230_ao_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int pci230_ct_insn_config(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int pci230_ct_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static void pci230_ns_to_timer(unsigned int *ns,int round);
static void pci230_ns_to_single_timer(unsigned int *ns,int round);
static void i8253_single_ns_to_timer(unsigned int i8253_osc_base, unsigned int *d, unsigned int *nanosec, int round_mode);
static void pci230_setup_monostable_ct0(comedi_device *dev, unsigned int ns, unsigned int n_chan);
static void pci230_z2_ct0(comedi_device *dev, unsigned int *ns,int round);
static void pci230_z2_ct1(comedi_device *dev, unsigned int *ns,int round);
static void pci230_z2_ct2(comedi_device *dev, unsigned int *ns,int round);
static void pci230_cancel_ct0(comedi_device *dev);
static void pci230_cancel_ct1(comedi_device *dev);
static void pci230_cancel_ct2(comedi_device *dev);
static irqreturn_t pci230_interrupt(int irq, void *d, struct pt_regs *regs);
static int pci230_ao_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd);
static int pci230_ao_cmd(comedi_device *dev, comedi_subdevice *s);
static int pci230_ao_cancel(comedi_device *dev, comedi_subdevice *s);
static void pci230_handle_ao(comedi_device *dev, comedi_subdevice *s);
static int pci230_ai_cmdtest(comedi_device *dev,comedi_subdevice *s, comedi_cmd *cmd);
static int pci230_ai_cmd(comedi_device *dev, comedi_subdevice *s);
static int pci230_ai_cancel(comedi_device *dev, comedi_subdevice *s);
static void pci230_handle_ai(comedi_device *dev, comedi_subdevice *s);
static void pci230_handle_fifo_half_full(comedi_device *dev, comedi_subdevice *s);
static void pci230_handle_fifo_not_empty(comedi_device *dev, comedi_subdevice *s);
 
static sampl_t pci230_ai_read(comedi_device *dev)
{
	/* Read sample. */
	sampl_t data = (sampl_t) inw(dev->iobase + PCI230_ADCDATA);

	/* PCI230 is 12 bit - stored in upper bits of 16 bit register (lower four bits reserved for expansion). */
	data = data>>4;

	/* If a bipolar range was specified, mangle it (twos complement->straight binary). */ 
	if (devpriv->ai_bipolar) {
		data ^= 1<<(thisboard->ai_bits-1);
	}
	return data;
}

static void pci230_ao_write(comedi_device *dev, sampl_t data, int chan)
{
	/* If a bipolar range was specified, mangle it (straight binary->twos complement). */ 
	if (devpriv->ao_bipolar) {
		data ^= 1<<(thisboard->ao_bits-1);
	}
		
	/* PCI230 is 12 bit - stored in upper bits of 16 bit register (lower four bits reserved for expansion). */
	data = data<<4;
		
	/* Write data. */
	outw((unsigned int) data, dev->iobase + (((chan) == 0) ? PCI230_DACOUT1 : PCI230_DACOUT2));
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
	int pci_iobase, iobase = 0;		/* PCI230's I/O spaces 1 and 2 respectively. */
	struct pci_dev *pci_dev;
	int i=0,irq_hdl;

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
	
	/* Read base addressses of the PCI230's two I/O regions from PCI configuration register. */
	if(pci_enable_device(pci_dev)<0)return -EIO;

	pci_iobase = pci_resource_start(pci_dev, 2);
	iobase = pci_resource_start(pci_dev, 3);

	printk("comedi%d: amplc_pci230: I/O region 1 0x%04x I/O region 2 0x%04x\n",dev->minor, pci_iobase, iobase);

	/* Allocate the private structure area using alloc_private().
	 * Macro defined in comedidev.h - memsets struct fields to 0. */
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

	/* Register the interrupt handler. */
	irq_hdl = comedi_request_irq(devpriv->pci_dev->irq, pci230_interrupt, SA_SHIRQ, "amplc_pci230", dev);
	if(irq_hdl<0) {
		printk("comedi%d: amplc_pci230: unable to register irq, commands will not be available %d\n", dev->minor, devpriv->pci_dev->irq);
	}
	else {
		dev->irq = devpriv->pci_dev->irq;
		printk("comedi%d: amplc_pci230: registered irq %d\n", dev->minor, devpriv->pci_dev->irq);
	}

/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.
 */
	if(alloc_subdevices(dev, 4)<0)
		return -ENOMEM;

	s=dev->subdevices+0;
	/* analog input subdevice */
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE|SDF_DIFF|SDF_GROUND;
	s->n_chan=thisboard->ai_chans;
	s->maxdata=(1<<thisboard->ai_bits)-1;
	s->range_table=&pci230_ai_range;
	s->insn_read = &pci230_ai_rinsn;
	s->len_chanlist = thisboard->ai_chans;
	/* Only register commands if the interrupt handler is installed. */
	if(irq_hdl==0) {
		dev->read_subdev=s;
		s->do_cmd = &pci230_ai_cmd;
		s->do_cmdtest = &pci230_ai_cmdtest;
		s->cancel = pci230_ai_cancel;
	}

	s=dev->subdevices+1;
	/* analog output subdevice */
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_WRITABLE;
	s->n_chan=thisboard->ao_chans;;
	s->maxdata=(1<<thisboard->ao_bits)-1;
	s->range_table=&pci230_ao_range;
	s->insn_write = &pci230_ao_winsn;
	s->insn_read = &pci230_ao_rinsn;
	s->len_chanlist = thisboard->ao_chans;
	/* Only register commands if the interrupt handler is installed. */
	if(irq_hdl==0) {
		dev->write_subdev=s;
		s->do_cmd = &pci230_ao_cmd;
		s->do_cmdtest = &pci230_ao_cmdtest;
		s->cancel = pci230_ao_cancel;
	}

	s=dev->subdevices+2;
	/* digital i/o subdevice */
	if(thisboard->have_dio){
		subdev_8255_init(dev,s,NULL,(devpriv->pci_iobase + PCI230_PPI_X_A));
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}

	s=dev->subdevices+3;
	/* timer subdevice */
	s->type=COMEDI_SUBD_TIMER;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=1;
	s->maxdata=0xffff;
	s->range_table=&range_digital;
	s->insn_config = pci230_ct_insn_config;
	s->insn_read = &pci230_ct_rinsn;

	printk("comedi%d: attached\n",dev->minor);

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
 *  COMEDI_SUBD_AI instruction; 
 */  
static int pci230_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n,i;
	int chan, range, aref;
	unsigned int status;
	unsigned int adccon, adcen, adcg;

	/* Unpack channel and range. */
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);
	aref = CR_AREF(insn->chanspec);

	/* If bit 2 of range unset, range is referring to bipolar element in range table */
	adccon = PCI230_ADC_TRIG_SW | PCI230_ADC_FIFO_RESET;
	devpriv->ai_bipolar = !PCI230_TEST_BIT(range, 2);
	if (aref==AREF_DIFF) {
		/* Differential. */
		adcen = 3<<2*chan;
		adccon |= PCI230_ADC_IM_DIF;	
		if (devpriv->ai_bipolar) {
			adccon |= PCI230_ADC_IR_BIP;
			adcg = range<<(2*chan-2*chan%2);
		}
		else {
			adccon |= PCI230_ADC_IR_UNI;
			adcg = ((range&(~4))+1)<<(2*chan-2*chan%2);
		}
	}
	else {
		/* Single ended. */
		adcen = 1<<chan;
		adccon |= PCI230_ADC_IM_SE;	
		if (devpriv->ai_bipolar) {
			adccon |= PCI230_ADC_IR_BIP;
			adcg = range<<(chan-chan%2);
		}
		else {
			adccon |= PCI230_ADC_IR_UNI;
			adcg = ((range&(~4))+1)<<(chan-chan%2);
		}
	}

	/* Enable only this channel in the scan list - otherwise by default we'll get one sample from each channel. */
	outw_p(adcen, dev->iobase + PCI230_ADCEN);

	/* Set gain for channel. */
	outw_p(adcg, dev->iobase + PCI230_ADCG);

	/* Specify uni/bip, se/diff, s/w conversion, and reset FIFO (even though we're not using it - MEV says so). */
	outw_p(adccon, dev->iobase + PCI230_ADCCON);

	/* Convert n samples */
	for(n=0;n<insn->n;n++){
		/* trigger conversion */
		outw_p(PCI230_ADC_CONV,dev->iobase + PCI230_ADCDATA);

#define TIMEOUT 100
		/* wait for conversion to end */
		for(i=0;i<TIMEOUT;i++){
			status = inw(dev->iobase + PCI230_ADCCON);
			if(PCI230_TEST_BIT(status, PCI230_ADC_BUSY_BIT))break;
		}
		if(i==TIMEOUT){
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
static int pci230_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i;
	int chan, range;
	
	/* Unpack channel and range. */
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);

	/* Set range - see analogue output range table; 0 => unipolar 10V, 1 => bipolar +/-10V range scale */
	devpriv->ao_bipolar = PCI230_TEST_BIT(range, PCI230_DAC_BIP_BIT);
	outw(range, dev->iobase + PCI230_DACCON);

	/* Writing a list of values to an AO channel is probably not
	 * very useful, but that's how the interface is defined. */
	for(i=0;i<insn->n;i++){
		/* Store the value to be written to the DAC in our pci230_private struct before mangling it. */
		devpriv->ao_readback[chan] = data[i];

		/* Write value to DAC. */
		pci230_ao_write(dev, data[i], chan);
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

/*
 *  COMEDI_SUBD_TIMER instructions; 
 *  
 *  insn_config allows user to start and stop counter/timer 2 (SK1 pin 21).
 *  Period specified in ns.
 *
 *  rinsn returns counter/timer's actual period in ns.  
 */
static int pci230_ct_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	unsigned int ns;
	
	if(insn->n!=1)return -EINVAL;

	ns = data[0];
	if (ns == 0) {
		//Stop counter/timer 2.
		pci230_cancel_ct2(dev);
	}
	else {
		//Start conter/timer 2 with period ns.
		pci230_z2_ct2(dev, &ns, TRIG_ROUND_MASK);
	}

	return 1;
}

static int pci230_ct_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=1)return -EINVAL;
	
	/* Return the actual period set in ns. */
	data[0] = PCI230_TIMEBASE_10MHZ*devpriv->divisor1*devpriv->divisor2;
	return 1;
}

static int pci230_ao_cmdtest(comedi_device *dev,comedi_subdevice *s,
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

	/* Step 1: make sure trigger sources are trivially valid.
	 * "invalid source" returned by comedilib to user mode process 
	 * if this fails. */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_INT;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_NOW;
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

	if(cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_NONE)err++;

	if(err)return 2;

	/* Step 3: make sure arguments are trivially compatible.
	 * "invalid argument" returned by comedilib to user mode process 
	 * if this fails. */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}

#define MAX_SPEED	3200		/* 3200ns => 312.5kHz */
#define MIN_SPEED	4294967295u	/* 4294967295ns = 4.29s - Comedi limit due to unsigned int cmd.  Driver limit = 2^16 (16bit counter) * 1000000ns (1kHz onboard clock) = 65.536s */

	if(cmd->scan_begin_src==TRIG_TIMER){
		if(cmd->scan_begin_arg<MAX_SPEED){
			cmd->scan_begin_arg=MAX_SPEED;
			err++;
		}
		if(cmd->scan_begin_arg>MIN_SPEED){
			cmd->scan_begin_arg=MIN_SPEED;
			err++;
		}
	}

	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src==TRIG_NONE){
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
		pci230_ns_to_single_timer(&cmd->scan_begin_arg,cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->scan_begin_arg)err++;
	}

	if(err)return 4;

	return 0;
}

static int pci230_ao_inttrig(comedi_device *dev,comedi_subdevice *s,
		unsigned int trig_num)
{
	if(trig_num != 0)
		return -EINVAL;

	/* Enable DAC interrupt. */
	devpriv->ier |= PCI230_INT_ZCLK_CT1;
	outb(devpriv->ier, devpriv->pci_iobase + PCI230_INT_SCE);

	s->async->inttrig=NULL;

	return 1;
}

static int pci230_ao_cmd(comedi_device *dev,comedi_subdevice *s)
{
	int range;

	/* Get the command. */
	comedi_cmd *cmd=&s->async->cmd;

	/* Calculate number of conversions required. */
	if(cmd->stop_src == TRIG_COUNT) {
		devpriv->ao_count = cmd->stop_arg * cmd->chanlist_len;
		devpriv->ao_stop = 0;
	}
	else {
		/* TRIG_NONE, user calls cancel. */
		devpriv->ao_count = 0;
		devpriv->ao_stop = 1;
	}
	
	/* Disable DAC interrupt. */
	devpriv->ier &= ~PCI230_INT_ZCLK_CT1;
	outb(devpriv->ier, devpriv->pci_iobase + PCI230_INT_SCE);

	/* Set range - see analogue output range table; 0 => unipolar 10V, 1 => bipolar +/-10V range scale */
	range = CR_RANGE(cmd->chanlist[0]);
	devpriv->ao_bipolar = PCI230_TEST_BIT(range, PCI230_DAC_BIP_BIT);
	outw(range, dev->iobase + PCI230_DACCON);

	/* Set the counter timers to the specified sampling frequency.
	 * TODO - when Comedi supports concurrent commands, this must be
	 * changed; using ct0 and ct1 for DAC will screw up ADC pacer
	 * which uses ct2 and ct0.  Change to only use ct1 for DAC?
	 *
	 * <sds>: we may as well do this now, in the transition to using single
	 * counters for the analogue input (to accommodate triggered scans).
	 */
	pci230_z2_ct1(dev, &cmd->scan_begin_arg, cmd->flags & TRIG_ROUND_MASK);	/* cmd->convert_arg is sampling period in ns */

	s->async->inttrig=pci230_ao_inttrig;

	return 0;
}

static int pci230_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,
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

	/* Step 1: make sure trigger sources are trivially valid.
	 * "invalid source" returned by comedilib to user mode process 
	 * if this fails. */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	/* Unfortunately, we cannot trigger a scan off an external source
	 * on the PCI260 board, since it uses the PPI0 (DIO) input, which
	 * isn't present on the PCI260 */
	if(thisboard->have_dio){
		cmd->scan_begin_src &= TRIG_FOLLOW | TRIG_EXT;
		}
	else{
		cmd->scan_begin_src &= TRIG_FOLLOW;
		}
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

	if(cmd->start_src!=TRIG_NOW)err++;
	if(cmd->scan_begin_src!=TRIG_FOLLOW &&
	   cmd->scan_begin_src!=TRIG_EXT)err++;
	if(cmd->convert_src!=TRIG_TIMER &&
	   cmd->convert_src!=TRIG_EXT)err++;
	if(cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_NONE)err++;
	
	/* Although the scan and convert triggers come from different sources, the
	 * driver relies on the knowledge of convert rate to trigger the correct number
	 * of channels. If the is not known (ie if convert_src==TRIG_EXT) then the 
	 * scan trigger will not work correctly.
	 * The convert trigger is the input into the "EXT TRIG" line (pin 25), whilst
	 * the scan trigger is "PPC0" (pin 49). */
	if(cmd->scan_begin_src==TRIG_EXT &&
	   cmd->convert_src==TRIG_EXT)err++;

	if(err)return 2;

	/* Step 3: make sure arguments are trivially compatible.
	 * "invalid argument" returned by comedilib to user mode process 
	 * if this fails. */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}

#define MAX_SPEED	3200		/* 3200ns => 312.5kHz */
#define MIN_SPEED	4294967295u	/* 4294967295ns = 4.29s - Comedi limit due to unsigned int cmd.  Driver limit = 2^16 (16bit counter) * 1000000ns (1kHz onboard clock) = 65.536s */

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
		/* convert_arg == 0 => trigger on -ve edge. */
		/* convert_arg == 1 => trigger on +ve edge. */
		if(cmd->convert_arg>1){
			cmd->convert_arg=1;			/* Default to trigger on +ve edge. */
			err++;
		}
	}

	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src==TRIG_NONE){
		/* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}


	if(cmd->scan_begin_src==TRIG_EXT){
		/* external "trigger" to begin each scan                               *
		 * scan_begin_arg==0 => use PPC0 input -> gate of CT0 -> gate of CT2   *
		 *			(sample convert trigger is CT2)                */
		if(cmd->scan_begin_arg!=0){
			cmd->scan_begin_arg=0;	/* default option, so you can monitor CT2 (only CT with an external output) */
			err++;
		}
	}

	if(err)return 3;

	/* Step 4: fix up any arguments.
	 * "argument conflict" returned by comedilib to user mode process 
	 * if this fails. */

	if(cmd->convert_src==TRIG_TIMER){
		tmp=cmd->convert_arg;
		pci230_ns_to_single_timer(&cmd->convert_arg,cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->convert_arg)err++;
	}

	if(err)return 4;

	return 0;
}

static int pci230_ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	int i, chan, range, diff;
	unsigned int adccon, adcen, adcg;
	unsigned int zgat;

	/* Get the command. */
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;

	/* Calculate number of conversions required. */
	if(cmd->stop_src == TRIG_COUNT) {
		devpriv->ai_count = cmd->stop_arg * cmd->chanlist_len;
		devpriv->ai_stop = 0;
	}
	else {
		/* TRIG_NONE, user calls cancel. */
		devpriv->ai_count = 0;
		devpriv->ai_stop = 1;
	}

	/* Steps;
	 * - Disable ADC interrupts.
	 * - Set channel scan list.
	 * - Set channel gains.
	 * - Enable and reset FIFO, specify uni/bip, se/diff, and start conversion source to none.
	 * - PAUSE (25us) - failure to do this leads to "dodgy data" for the first few channels at high convert rates.
	 * - Enable conversion complete interrupt.
	 * - Set the counter timers to the specified sampling frequency.
	 * - Enable AND RESET FIFO (yes you do need to do this twice), set FIFO interrupt trigger level, set start conversion source to counter 2. 
	 */

	/* Disable ADC interrupt. */
	devpriv->ier &= ~PCI230_INT_ADC;
	outb(devpriv->ier, devpriv->pci_iobase + PCI230_INT_SCE);

	if (CR_AREF(cmd->chanlist[0])==AREF_DIFF) {
		/* Differential - all channels must be differential. */
		diff = 1;
		adccon = PCI230_ADC_IM_DIF;	
	}
	else {
		/* Single ended - all channels must be single-ended. */
		diff = 0;
		adccon = PCI230_ADC_IM_SE;	
	}

	adccon |= PCI230_ADC_FIFO_RESET | PCI230_ADC_FIFO_EN;
	//adccon |= PCI230_ADC_FIFO_RESET;
	adcg = 0;
	adcen = 0;

	/* If bit 2 of range unset, range is referring to bipolar element in range table */
	range = CR_RANGE(cmd->chanlist[0]);
	devpriv->ai_bipolar = !PCI230_TEST_BIT(range, 2);	
	if (devpriv->ai_bipolar) {
		adccon |= PCI230_ADC_IR_BIP;
		for (i = 0; i < cmd->chanlist_len; i++) {
			chan = CR_CHAN(cmd->chanlist[i]);
			range = CR_RANGE(cmd->chanlist[i]);
			if (diff) {
				adcg |= range<<(2*chan-2*chan%2);
				adcen |= 3<<2*chan;
			}
			else {
				adcg |= range<<(chan-chan%2);
				adcen |= 1<<chan;
			}
		}
	}
	else {
		adccon |= PCI230_ADC_IR_UNI;
		for (i = 0; i < cmd->chanlist_len; i++) {
			chan = CR_CHAN(cmd->chanlist[i]);
			range = CR_RANGE(cmd->chanlist[i]);
			if (diff) {
				adcg |= ((range&(~4))+1)<<(2*chan-2*chan%2);
				adcen |= 3<<2*chan;
			}
			else {
				adcg |= ((range&(~4))+1)<<(chan-chan%2);
				adcen |= 1<<chan;
			}
		}
	}

	/* Set channel scan list. */
	outw(adcen, dev->iobase + PCI230_ADCEN);

	/* Set channel gains. */
	outw(adcg, dev->iobase + PCI230_ADCG);

	/* Enable and reset FIFO, specify FIFO trigger level full, specify uni/bip, se/diff, and start conversion source to none. */
	outw(adccon | PCI230_ADC_INT_FIFO_FULL | PCI230_ADC_TRIG_NONE, dev->iobase + PCI230_ADCCON);

	/* Delay */
	/* Failure to include this will result in the first few channels'-worth of data being corrupt,
	 * normally manifesting itself by large negative voltages. It seems the board needs time to
	 * settle between the first FIFO reset (above) and the second FIFO reset (below). Setting
	 * the channel gains and scan list _before_ the first FIFO reset also helps, though only
	 * slightly. */
	comedi_udelay(25);

	/* Enable ADC (conversion complete) interrupt. */
	devpriv->ier |= PCI230_INT_ADC;
	outb(devpriv->ier, devpriv->pci_iobase + PCI230_INT_SCE);

	/* Set up the scan_begin_src (if it's NOT set to TRIG_FOLLOW) */
	if(cmd->scan_begin_src == TRIG_EXT){
		/* use PPC0 -> gate of CT0 (as monostable) -> gate of CT2
		 *
		 * Note that the PCI230 card does not support "native" triggered scans,
		 * although the _hardware_ can be set up to achieve this. This is _not_
		 * software emulation within the driver, it is merely using the on-board
		 * counters and one of the digital inputs to achieve the same thing.
		 * The idea is to use a rising edge of a digital input (in this case PPC0)
		 * to trigger a counter (set up as a monostable). This produces a pulse of
		 * exactly the same length as the number of conversion pulses in your scan.
		 * This pulse is then used as to gate the counter responsible for your
		 * convert source.
		 *
		 * So, if your conversion rate is set to 100kHz (10us/conversion) and you 
		 * have 8 channels in your channel list, a positive edge on PPC0 will
		 * trigger a pulse of length 80us (8 x 10us). Because the two counters
		 * involved have the same clock source, the monostable pulse will always
		 * be exactly the right length.
		 *
		 * Previous versions of this driver used two cascaded counters to achieve
		 * different conversion rates (the output of the first counter is the
		 * clock input of the second counter). The use of the triggered scan
		 * functionality necessitated using only one counter/timer for dividing
		 * the internal clock down to the convert rate. In order to allow
		 * relatively low convert rates (upto the comedi limit of 4.29s), the
		 * driver now uses not only the 10MHz input clock, but also (depending on
		 * the desired convert rate) a choice of 1MHz, 100kHz or 10kHz clocks.
		 *                                                 - sds, 30 April 2004 */

		/* initialise the gates to sensible settings while we set everything up */
		zgat = PCI230_ZGAT_CT0 | PCI230_ZGAT_SRC_GND;
		outb(zgat, devpriv->pci_iobase + PCI230_ZGAT_SCE);

		zgat = PCI230_ZGAT_CT2 | PCI230_ZGAT_SRC_GND;
		outb(zgat, devpriv->pci_iobase + PCI230_ZGAT_SCE);

		pci230_setup_monostable_ct0(dev, cmd->convert_arg, cmd->chanlist_len);

		/* now set the gates up so that we can begin triggering */
		zgat = PCI230_ZGAT_CT0 | PCI230_ZGAT_SRC_PPCN;
		outb(zgat, devpriv->pci_iobase + PCI230_ZGAT_SCE);

		zgat = PCI230_ZGAT_CT2 | PCI230_ZGAT_SRC_OUTNP1;
		outb(zgat, devpriv->pci_iobase + PCI230_ZGAT_SCE);
	}
	else{	/* must be using "TRIG_FOLLOW", so need to "ungate" CT2 */
		zgat = PCI230_ZGAT_CT2 | PCI230_ZGAT_SRC_VCC;
		outb(zgat, devpriv->pci_iobase + PCI230_ZGAT_SCE);
	}

	/* Set start conversion source. */
	if(cmd->convert_src == TRIG_TIMER) {
		/* Onboard counter/timer 2. */
		adccon = adccon | PCI230_ADC_TRIG_Z2CT2;

		/* Set the counter timers to the specified sampling frequency. */
		pci230_z2_ct2(dev, &cmd->convert_arg, cmd->flags & TRIG_ROUND_MASK);	/* cmd->convert_arg is sampling period in ns */
	}
	else {
		/* TRIG_EXT - external trigger. */
		if (cmd->convert_arg) {
			/* Trigger on +ve edge. */
			adccon = adccon | PCI230_ADC_TRIG_EXTP;
		}
		else {
			/* Trigger on -ve edge. */
			adccon = adccon | PCI230_ADC_TRIG_EXTN;
		}	
	}


	/* Set FIFO interrupt trigger level. */
	if(cmd->stop_src == TRIG_COUNT) {
		if (devpriv->ai_count < 2048) {
			adccon = adccon | PCI230_ADC_INT_FIFO_NEMPTY;
		}
		else {
			adccon = adccon | PCI230_ADC_INT_FIFO_HALF;
		}
	}
	else {
		/* TRIG_NONE - trigger on half-full FIFO. */
		adccon = adccon | PCI230_ADC_INT_FIFO_HALF;
	}

	//adccon = adccon | PCI230_ADC_FIFO_EN;
	outw(adccon, dev->iobase + PCI230_ADCCON);

	return 0;
}


/* This function doesn't require a particular form, this is just
 * what happens to be used in some of the drivers.  It should
 * convert ns nanoseconds to a counter value suitable for programming
 * the device.  Also, it should adjust ns so that it cooresponds to
 * the actual time that the device will use. */
static void pci230_ns_to_timer(unsigned int *ns,int round)
{
	unsigned int divisor0, divisor1;
	i8253_cascade_ns_to_timer_2div(PCI230_TIMEBASE_10MHZ, &divisor0, &divisor1, ns, TRIG_ROUND_MASK);
	return;
}


/* This function is used for analogue input. Only one counter/timer can be used,
 * because for the triggered scan functionality to work, 2 counters with
 * identical clock inputs and divide ratios are required, so that the
 * correct number of channels are converted in each scan.
 * This is a very "noddy" way of doing this... apologies. (sds) */
static unsigned int pci230_choose_clk_src(unsigned int ns)
{
	unsigned int clk_src=0;

	if(ns <   6553600)			clk_src=PCI230_TIMEBASE_10MHZ;
	if(ns >=  6553600 && ns <  65536000)	clk_src=PCI230_TIMEBASE_1MHZ;
	if(ns >= 65536000 && ns < 655360000)	clk_src=PCI230_TIMEBASE_100KHZ;
	if(ns >=655360000 && ns <4294967295u)	clk_src=PCI230_TIMEBASE_10KHZ; /* maximum limited by comedi = 4.29s */

	if(clk_src == 0){
		printk("comedi: dodgy clock source chosen, using 10MHz\n");
		clk_src=PCI230_TIMEBASE_10MHZ;
		}
	return clk_src;
}

static void pci230_ns_to_single_timer(unsigned int *ns,int round)
{
	unsigned int divisor;
	unsigned int clk_src;

	clk_src=pci230_choose_clk_src(*ns);
	i8253_single_ns_to_timer(clk_src, &divisor, ns, TRIG_ROUND_MASK);
	return;
}

static void i8253_single_ns_to_timer(unsigned int i8253_osc_base, unsigned int *d, unsigned int *nanosec, int round_mode)
{
        int divider;
        unsigned int div;

        /* exit early if everything is already correct (this can save time
         * since this function may be called repeatedly during command tests
         * and execution) */
        if(*d * i8253_osc_base == *nanosec &&
                *d > 1 && *d < 0x10000)
        {
                return;
        }

	round_mode &= TRIG_ROUND_MASK;
	switch (round_mode) {
	case TRIG_ROUND_NEAREST:
	default:
		div=(*nanosec + i8253_osc_base / 2) / i8253_osc_base;
		break;
	case TRIG_ROUND_UP:
		div=(*nanosec)  / i8253_osc_base;
		break;
	case TRIG_ROUND_DOWN:
		div=(*nanosec + i8253_osc_base + 1 )  / i8253_osc_base;
		break;	
	}

	*nanosec = div * i8253_osc_base;
	*d = div & 0xffff;    // masking is done since counter maps zero to 0x10000

	return;
}

static void pci230_setup_monostable_ct0(comedi_device *dev, unsigned int ns, unsigned int n_chan)
{
	/* ns is the convert period, n_chan is the no of channels per scan.
	 * We must make the output of the counter equal in time to the amount of time
	 * it takes to acquire n_chan*ns */

	unsigned int pulse_duration;

	pulse_duration = ns*n_chan;
	devpriv->clk_src0=pci230_choose_clk_src(pulse_duration); /* let's hope that if devpriv->clk_src0 != devpriv->clk_src2, then one is divided down from the other! */

	devpriv->divisor0=pulse_duration/devpriv->clk_src0;

	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 0, devpriv->divisor0, 1);	/* Counter 1, mode 1 */

	/* PCI 230 specific - ties up counter clk input with correct clk source */
	switch (devpriv->clk_src0) {
	case PCI230_TIMEBASE_10MHZ:
	default:
		outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_10MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
		break;
	case PCI230_TIMEBASE_1MHZ:
		outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_1MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
		break;
	case PCI230_TIMEBASE_100KHZ:
		outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_100KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
		break;
	case PCI230_TIMEBASE_10KHZ:
		outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_10KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
		break;
	case PCI230_TIMEBASE_1KHZ:
		outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_1KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
		break;
	}

	return;
}


/* 
 *  Set ZCLK_CT0 to square wave mode with period of ns.
 */
static void pci230_z2_ct0(comedi_device *dev, unsigned int *ns,int round)
{
	devpriv->clk_src0=pci230_choose_clk_src(*ns);	/* choose a suitable clock source from the range available, given the desired period in ns */
	i8253_single_ns_to_timer(devpriv->clk_src0, &devpriv->divisor0, ns, TRIG_ROUND_MASK);

	/* Generic i8254_load calls; program counters' divide ratios. */
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 0, devpriv->divisor0, 3);	/* Counter 0, divisor0, square wave (8254 mode 3). */

	/* PCI 230 specific - ties up counter clk input with clk source */
	switch (devpriv->clk_src0) {
	case PCI230_TIMEBASE_10MHZ:
	default:
		outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_10MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
		break;
	case PCI230_TIMEBASE_1MHZ:
		outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_1MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
		break;
	case PCI230_TIMEBASE_100KHZ:
		outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_100KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
		break;
	case PCI230_TIMEBASE_10KHZ:
		outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_10KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
		break;
	case PCI230_TIMEBASE_1KHZ:
		outb(PCI230_ZCLK_CT0 | PCI230_ZCLK_SRC_1KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 0's input clock source. */
		break;
	}
	return;
}

static void pci230_cancel_ct0(comedi_device *dev)
{
	devpriv->divisor0 = 0;
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 0, devpriv->divisor0, 0);	/* Counter 0, divisor0, 8254 mode 0. */
}

/* 
 *  Set ZCLK_CT1 to square wave mode with period of ns.
 *  Default clk source for DAC.
 */
static void pci230_z2_ct1(comedi_device *dev, unsigned int *ns,int round)
{
	devpriv->clk_src1=pci230_choose_clk_src(*ns);	/* choose a suitable clock source from the range available, given the desired period in ns */
	i8253_single_ns_to_timer(devpriv->clk_src1, &devpriv->divisor1, ns, TRIG_ROUND_MASK);

	/* Generic i8254_load calls; program counters' divide ratios. */
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 1, devpriv->divisor1, 3);	/* Counter 1, divisor1, square wave (8254 mode 3). */

	/* PCI 230 specific - ties up counter clk input with clk source */
	switch (devpriv->clk_src1) {
	case PCI230_TIMEBASE_10MHZ:
	default:
		outb(PCI230_ZCLK_CT1 | PCI230_ZCLK_SRC_10MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 1's input clock source. */
		break;
	case PCI230_TIMEBASE_1MHZ:
		outb(PCI230_ZCLK_CT1 | PCI230_ZCLK_SRC_1MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 1's input clock source. */
		break;
	case PCI230_TIMEBASE_100KHZ:
		outb(PCI230_ZCLK_CT1 | PCI230_ZCLK_SRC_100KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 1's input clock source. */
		break;
	case PCI230_TIMEBASE_10KHZ:
		outb(PCI230_ZCLK_CT1 | PCI230_ZCLK_SRC_10KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 1's input clock source. */
		break;
	case PCI230_TIMEBASE_1KHZ:
		outb(PCI230_ZCLK_CT1 | PCI230_ZCLK_SRC_1KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 1's input clock source. */
		break;
	}
	return;
}

static void pci230_cancel_ct1(comedi_device *dev)
{
	devpriv->divisor1 = 0;
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 1, devpriv->divisor1, 0);	/* Counter 1, divisor1, 8254 mode 0. */
}

/* 
 *  Set ZCLK_CT2 to square wave mode with period of ns.
 *  Default clk source for ADC.
 */
static void pci230_z2_ct2(comedi_device *dev, unsigned int *ns,int round)
{
	devpriv->clk_src2=pci230_choose_clk_src(*ns);	/* choose a suitable clock source from the range available, given the desired period in ns */
	i8253_single_ns_to_timer(devpriv->clk_src2, &devpriv->divisor2, ns, TRIG_ROUND_MASK);

	/* Generic i8254_load calls; program counters' divide ratios. */
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 2, devpriv->divisor2, 3);	/* Counter 2, divisor2, square wave (8254 mode 3). */

	/* PCI 230 specific - ties up counter clk input with clk source */
	switch (devpriv->clk_src2) {
	case PCI230_TIMEBASE_10MHZ:
	default:
		outb(PCI230_ZCLK_CT2 | PCI230_ZCLK_SRC_10MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 2's input clock source. */
		break;
	case PCI230_TIMEBASE_1MHZ:
		outb(PCI230_ZCLK_CT2 | PCI230_ZCLK_SRC_1MHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 2's input clock source. */
		break;
	case PCI230_TIMEBASE_100KHZ:
		outb(PCI230_ZCLK_CT2 | PCI230_ZCLK_SRC_100KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 2's input clock source. */
		break;
	case PCI230_TIMEBASE_10KHZ:
		outb(PCI230_ZCLK_CT2 | PCI230_ZCLK_SRC_10KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 2's input clock source. */
		break;
	case PCI230_TIMEBASE_1KHZ:
		outb(PCI230_ZCLK_CT2 | PCI230_ZCLK_SRC_1KHZ, devpriv->pci_iobase + PCI230_ZCLK_SCE);	/* Program counter 2's input clock source. */
		break;
	}
	return;
}

static void pci230_cancel_ct2(comedi_device *dev)
{
	devpriv->divisor2 = 0;
	i8254_load(devpriv->pci_iobase + PCI230_Z2_CT0, 2, devpriv->divisor2, 0);	/* Counter 2, divisor2, 8254 mode 0. */
}

/* Interrupt handler */
static irqreturn_t pci230_interrupt(int irq, void *d, struct pt_regs *regs)
{
	int status_int;
	comedi_device *dev = (comedi_device*) d;
	comedi_subdevice *s;
	int retval = 1;

	/* Read interrupt status/enable register. */
	status_int = inb(devpriv->pci_iobase + PCI230_INT_SCE);

	if (status_int == PCI230_INT_DISABLE) {
		return IRQ_NONE;
	}

	/* Disable all of board's interrupts.
	 * (Only those interrrupts that need re-enabling, are, later in the handler).  */
	devpriv->ier = PCI230_INT_DISABLE; 
	outb(devpriv->ier, devpriv->pci_iobase + PCI230_INT_SCE);

	/* 
	 * Check the source of interrupt and handle it.
	 * The PCI230 can cope with concurrent ADC, DAC, PPI C0 and C3 interrupts.
	 * However, at present (Comedi-0.7.60) does not allow concurrent
	 * execution of commands, instructions or a mixture of the two.
	 */
	
	if (status_int & PCI230_INT_ZCLK_CT1) {
		s = dev->write_subdev;
		s->async->events = 0;
		pci230_handle_ao(dev, s);
		comedi_event(dev, s, s->async->events);
		s->async->events = 0;
	}

	if (status_int & PCI230_INT_ADC) {
		s = dev->read_subdev;
		s->async->events = 0;
		pci230_handle_ai(dev, s);
		comedi_event(dev, s, s->async->events);
		s->async->events = 0;
	}

	return IRQ_RETVAL(retval);
}

static void pci230_handle_ao(comedi_device *dev, comedi_subdevice *s) {
	sampl_t data;
	int i, ret;
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;

	for (i = 0; i < cmd->chanlist_len; i++) {
		/* Read sample from Comedi's circular buffer. */
		ret = comedi_buf_get(s->async, &data);
		if(ret < 0) {
			comedi_error(dev, "buffer underrun");
			return;										// XXX does comedi_buf_get set s->async->events with appropriate flags in this instance?
		}
		/* Write value to DAC. */
		pci230_ao_write(dev, data, cmd->chanlist[i]);

		if(async->cmd.stop_src == TRIG_COUNT) {
			if(devpriv->ao_count > 0) devpriv->ao_count--;
			if(devpriv->ao_count == 0) break;
		}
	}

	if(devpriv->ao_count == 0 && devpriv->ao_stop == 0) {
		/* End of DAC. */
		async->events |= COMEDI_CB_EOA;
		pci230_ao_cancel(dev, s);
	}
	else {
		/* More samples required, tell Comedi to block. */
		async->events |= COMEDI_CB_BLOCK;
		/* Enable DAC (conversion complete) interrupt (and leave any other enabled interrupts as they are). */
		devpriv->ier |= PCI230_INT_ZCLK_CT1;
		outb(devpriv->ier, devpriv->pci_iobase + PCI230_INT_SCE);
	}
	return;
}

static void pci230_handle_ai(comedi_device *dev, comedi_subdevice *s) {
	int error = 0;
	int status_fifo;
	
	/* Read FIFO state. */
	status_fifo = inw(dev->iobase + PCI230_ADCCON);

	if (status_fifo & PCI230_ADC_FIFO_FULL) {
		/* Report error otherwise FIFO overruns will go unnoticed by the caller. */
		comedi_error(dev, "FIFO overrun");
		error++;
	}
	else if (status_fifo & PCI230_ADC_FIFO_HALF) {
		/* FIFO is at least half full. */
		pci230_handle_fifo_half_full(dev, s);
	}
	else if (status_fifo & PCI230_ADC_FIFO_EMPTY) {
		/* FIFO empty but we got an interrupt. */
		printk("comedi%d: amplc_pci230::pci230_handle_ai FIFO empty - spurious interrupt\n",dev->minor);
	}
	else {
		/* FIFO is less than half full, but not empty. */
		pci230_handle_fifo_not_empty(dev, s);
	}

	if (error) {
		/* Cancel sampled conversion. */
		s->async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
		pci230_ai_cancel(dev, s);	
	}
	if(devpriv->ai_count == 0 && devpriv->ai_stop == 0) {
		/* Acquisition complete. */
		s->async->events |= COMEDI_CB_EOA;
		pci230_ai_cancel(dev, s);			/* disable hardware conversions */
	}
	else {
		/* More samples required, tell Comedi to block. */
		s->async->events |= COMEDI_CB_BLOCK;

		/* Enable ADC (conversion complete) interrupt (and leave any other enabled interrupts as they are). */
		devpriv->ier |= PCI230_INT_ADC;
		outb(devpriv->ier, devpriv->pci_iobase + PCI230_INT_SCE);
	}
	return;
}

static void pci230_handle_fifo_half_full(comedi_device *dev, comedi_subdevice *s) {
	int i;

	for (i = 0; i < 2048; i++) {
		/* Read sample and store in Comedi's circular buffer. */
		comedi_buf_put(s->async, pci230_ai_read(dev));

		if(s->async->cmd.stop_src == TRIG_COUNT)
		{
			if(--devpriv->ai_count == 0) {
				/* Acquisition complete. */
				return;
			}
		}
	}
	/* More samples required. */
	return;
}

static void pci230_handle_fifo_not_empty(comedi_device *dev, comedi_subdevice *s) {
	while (devpriv->ai_count != 0) {
		if (inw(dev->iobase + PCI230_ADCCON) & PCI230_ADC_FIFO_EMPTY) {
			/* The FIFO is empty, block. */
			return;
		}
		/* There are sample(s) to read from FIFO, read one and store in Comedi's circular buffer. */
		comedi_buf_put(s->async, pci230_ai_read(dev));

		if(devpriv->ai_count > 0) devpriv->ai_count--;
	}
	/* Acquisition complete. */
	return;
}


static int pci230_ao_cancel(comedi_device *dev, comedi_subdevice *s) {
	devpriv->ao_count = 0;
	devpriv->ao_stop = 0;

	/* Stop counter/timers. */
	pci230_cancel_ct1(dev);

	/* Disable DAC interrupt. */
	devpriv->ier &= ~PCI230_INT_ZCLK_CT1;
	outb(devpriv->ier, devpriv->pci_iobase + PCI230_INT_SCE);

	return 0;
}

static int pci230_ai_cancel(comedi_device *dev, comedi_subdevice *s) {
	devpriv->ai_count = 0;
	devpriv->ai_stop = 0;

	/* Stop counter/timers. */
	pci230_cancel_ct2(dev);

	/* Disable ADC interrupt. */
	devpriv->ier &= ~PCI230_INT_ADC;
	outb(devpriv->ier, devpriv->pci_iobase + PCI230_INT_SCE);

	/* Reset FIFO and set start conversion source to none. */
	outw(PCI230_ADC_FIFO_RESET | PCI230_ADC_TRIG_NONE, dev->iobase + PCI230_ADCCON);

	/* Clear channel scan list. */
	outw(0x0000, dev->iobase + PCI230_ADCEN);

	/* Clear channel gains. */
	outw(0x0000, dev->iobase + PCI230_ADCG);

	return 0;
}


