/*
    comedi/drivers/cb_pcimdas.c
    Comedi driver for Computer Boards PCIM-DAS1602/16

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
Driver: cb_pcimdas.o
Description: Computer Boards PCI Migration series boards
Devices: (Computer Boards) PCIM-DAS1602/16 [cb_pcimdas]
Author: Richard Bytheway
Updated: Wed, 13 Nov 2002 12:34:56 +0000
Status: experimental

Written to support the PCIM-DAS1602/16 on a 2.4 series kernel.

Configuration Options:
    [0] - PCI bus number
    [1] - PCI slot number

Developed from cb_pcidas and skel by Richard Bytheway (mocelet@sucs.org). 
Only supports DIO, AO and simple AI in it's present form.
No interrupts, multi channel or FIFO AI, although the card looks like it could support this.
See http://www.measurementcomputing.com/PDFManuals/pcim-das1602_16.pdf for more details.
*/

#include <linux/comedidev.h>

#include <linux/pci.h>
#include <linux/delay.h>

#include "plx9052.h"

//#define CBPCIMDAS_DEBUG
#undef CBPCIMDAS_DEBUG

/* Registers for the PCIM-DAS1602/16 */

// sizes of io regions (bytes)
#define BADR0_SIZE 2  //??
#define BADR1_SIZE 4
#define BADR2_SIZE 6
#define BADR3_SIZE 16
#define BADR4_SIZE 4

//DAC Offsets
#define ADC_TRIG 0
#define DAC0_OFFSET 2
#define DAC1_OFFSET 4

//AI and Counter Constants
#define MUX_LIMITS 0
#define MAIN_CONN_DIO 1
#define ADC_STAT 2
#define ADC_CONV_STAT 3
#define ADC_INT 4
#define ADC_PACER 5
#define BURST_MODE 6
#define PROG_GAIN 7
#define CLK8254_1_DATA 8
#define CLK8254_2_DATA 9
#define CLK8254_3_DATA 10
#define CLK8254_CONTROL 11
#define USER_COUNTER 12
#define RESID_COUNT_H 13
#define RESID_COUNT_L 14

//DIO Offsets
#define PORT_A 0
#define PORT_B 1
#define PORT_C 2
#define DIO_CONFIG 3

//DIO Constants
#define ALL_OUTPUT 128
#define PORT_A_IN 16
#define PORT_B_IN 2
#define PORT_CH_IN 8
#define PORT_CL_IN 1
#define PORT_A_MASK 0x0000ff
#define PORT_B_MASK 0x00ff00
#define PORT_C_MASK 0xff0000
#define PORT_CL_MASK 0x0f0000
#define PORT_CH_MASK 0xf00000


/* Board description */
typedef struct cb_pcimdas_board_struct
{
	char *name;
	unsigned short device_id;
	int ai_se_chans;	// Inputs in single-ended mode
	int ai_diff_chans;	// Inputs in differential mode
	int ai_bits;	// analog input resolution
	int ai_speed;	// fastest conversion period in ns
	int ao_nchan;	// number of analog out channels
	int ao_bits;	// analogue output resolution
	int has_ao_fifo;	// analog output has fifo
	int ao_scan_speed;	// analog output speed for 1602 series (for a scan, not conversion)
	int fifo_size;	// number of samples fifo can hold
	int dio_bits;  // number of dio bits
	int has_dio;	// has DIO
	comedi_lrange *ranges;
} cb_pcimdas_board;

static cb_pcimdas_board cb_pcimdas_boards[] =
{
	{
		name:		"PCIM-DAS1602/16",
		device_id:	0x56,
		ai_se_chans:	16,
		ai_diff_chans:	8,
		ai_bits:	16,
		ai_speed:	10000, //??
		ao_nchan: 	2,     
		ao_bits:	12,
		has_ao_fifo:	0,     //??
		ao_scan_speed:	10000, //??
		fifo_size:	1024,   
		dio_bits:	24,
		has_dio:	1,
//		ranges:		&cb_pcimdas_ranges,
	},
};

/* This is used by modprobe to translate PCI IDs to drivers.  Should
 * only be used for PCI and ISA-PnP devices */
static struct pci_device_id cb_pcimdas_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_COMPUTERBOARDS, 0x0056, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, cb_pcimdas_pci_table);

#define N_BOARDS 1 // Max number of boards supported

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((cb_pcimdas_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct{
	int data;

	// would be useful for a PCI device 
	struct pci_dev *pci_dev;

	//base addresses
	unsigned int BADR0;
	unsigned int BADR1;
	unsigned int BADR2;
	unsigned int BADR3;
	unsigned int BADR4;

	/* Used for AO readback */
	lsampl_t ao_readback[2];

	// Used for DIO
	unsigned short int port_a;   // copy of BADR4+0
	unsigned short int port_b;   // copy of BADR4+1
	unsigned short int port_c;   // copy of BADR4+2
	unsigned short int dio_mode; // copy of BADR4+3
	
}cb_pcimdas_private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((cb_pcimdas_private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int cb_pcimdas_attach(comedi_device *dev,comedi_devconfig *it);
static int cb_pcimdas_detach(comedi_device *dev);
static comedi_driver driver_cb_pcimdas={
	driver_name:	"cb_pcimdas",
	module:		THIS_MODULE,
	attach:		cb_pcimdas_attach,
	detach:		cb_pcimdas_detach,
};

static int cb_pcimdas_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int cb_pcimdas_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int cb_pcimdas_ao_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int cb_pcimdas_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int cb_pcimdas_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int cb_pcimdas_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	struct pci_dev* pcidev;
	int index;
	//unsigned long BADR0;
	unsigned long BADR1, BADR2, BADR3, BADR4;
	int err;
	//int i;

	printk("comedi%d: cb_pcimdas: ",dev->minor);	

/*
 * Allocate the private structure area.
 */
	if(alloc_private(dev,sizeof(cb_pcimdas_private))<0)
		return -ENOMEM;

/*
 * Probe the device to determine what device in the series it is.
 */
	printk("\n");

	pci_for_each_dev(pcidev)
	{
		// is it not a computer boards card?
		if(pcidev->vendor != PCI_VENDOR_ID_COMPUTERBOARDS)
			continue;
		// loop through cards supported by this driver
		for(index = 0; index < N_BOARDS; index++)
		{
			if(cb_pcimdas_boards[index].device_id != pcidev->device)
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
			dev->board_ptr = cb_pcimdas_boards + index;
			goto found;
		}
	}

	printk("No supported ComputerBoards/MeasurementComputing card found on "
		"requested position\n");
	return -EIO;

found:

	printk("Found %s on bus %i, slot %i\n", cb_pcimdas_boards[index].name,
		devpriv->pci_dev->bus->number, PCI_SLOT(devpriv->pci_dev->devfn));

	// Warn about non-tested features
	switch(thisboard->device_id)
	{
		case 0x56:
			break;
		default:
			printk( "THIS CARD IS UNSUPPORTED.\n"
				"PLEASE REPORT USAGE TO <mocelet@sucs.org>\n");
	};

	if(pci_enable_device(devpriv->pci_dev))
		return -EIO;

// Since I don't know how large BADR0 is, lets not request it for now.
// It doesn't appear to be needed for operation of the Board.
//
//	BADR0 = pci_resource_start(devpriv->pci_dev, 0); 
	BADR1 = pci_resource_start(devpriv->pci_dev, 1); 
	BADR2 = pci_resource_start(devpriv->pci_dev, 2); 
	BADR3 = pci_resource_start(devpriv->pci_dev, 3); 
	BADR4 = pci_resource_start(devpriv->pci_dev, 4); 

	// reserve io ports
	err = 0;
//	if(check_mem_region(BADR0, BADR0_SIZE) < 0)
//		err+=1;
	if(check_region(BADR1, BADR1_SIZE) < 0)
		err+=2;
	if(check_region(BADR2, BADR2_SIZE) < 0)
		err+=4;
	if(check_region(BADR3, BADR3_SIZE) < 0)
		err+=8;
	if(check_region(BADR4, BADR4_SIZE) < 0)
		err+=16;
	if(err)
	{
		printk(" I/O port conflict %d\n",err);
		return -EIO;
	}
//	request_mem_region(BADR0, BADR0_SIZE, "cb_pcimdas");
//	devpriv->BADR0 = BADR0;
	request_region(BADR1, BADR1_SIZE, "cb_pcimdas");
	devpriv->BADR1 = BADR1;
	request_region(BADR2, BADR2_SIZE, "cb_pcimdas");
	devpriv->BADR2 = BADR2;
	request_region(BADR3, BADR3_SIZE, "cb_pcimdas");
	devpriv->BADR3 = BADR3;
	request_region(BADR4, BADR4_SIZE, "cb_pcimdas");
	devpriv->BADR4 = BADR4;

#ifdef CBPCIMDAS_DEBUG
	printk("devpriv->BADR0 = %d\n",devpriv->BADR0);
	printk("devpriv->BADR1 = %d\n",devpriv->BADR1);
	printk("devpriv->BADR2 = %d\n",devpriv->BADR2);
	printk("devpriv->BADR3 = %d\n",devpriv->BADR3);
	printk("devpriv->BADR4 = %d\n",devpriv->BADR4);
#endif


// Dont support IRQ yet
//	// get irq
//	if(comedi_request_irq(devpriv->pci_dev->irq, cb_pcimdas_interrupt, SA_SHIRQ, "cb_pcimdas", dev ))
//	{
//		printk(" unable to allocate irq %d\n", devpriv->pci_dev->irq);
//		return -EINVAL;
//	}
//	dev->irq = devpriv->pci_dev->irq;

	//Initialize dev->board_name
	dev->board_name = thisboard->name;


/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.
 */
	if(alloc_subdevices(dev, 3)<0)
		return -ENOMEM;

	s=dev->subdevices+0;
	//dev->read_subdev=s;
	// analog input subdevice 
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE|SDF_GROUND;
	s->n_chan=thisboard->ai_se_chans;
	s->maxdata=(1<<thisboard->ai_bits)-1;
	s->range_table=&range_unknown;
	s->len_chanlist=1;  // This is the maximum chanlist length that
			     //	the board can handle 
	s->insn_read = cb_pcimdas_ai_rinsn;

	s=dev->subdevices+1;
	// analog output subdevice
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_WRITABLE;
	s->n_chan=thisboard->ao_nchan;
	s->maxdata=1<<thisboard->ao_bits;
	s->range_table=&range_unknown; //ranges are hardware settable, but not software readable.
	s->insn_write = &cb_pcimdas_ao_winsn;
	s->insn_read = &cb_pcimdas_ao_rinsn;

	s=dev->subdevices+2;
	/* digital i/o subdevice */
	if(thisboard->has_dio){
		s->type=COMEDI_SUBD_DIO;
		s->subdev_flags=SDF_READABLE|SDF_WRITABLE;
		s->n_chan=thisboard->dio_bits;
		s->maxdata=1;
		s->range_table=&range_digital;
		s->insn_bits = cb_pcimdas_dio_insn_bits;
		s->insn_config = cb_pcimdas_dio_insn_config;
	}else{
		s->type = COMEDI_SUBD_UNUSED;
	}
	
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
static int cb_pcimdas_detach(comedi_device *dev)
{
#ifdef CBPCIMDAS_DEBUG
	printk("devpriv->BADR0 = %d\n",devpriv->BADR0);
	printk("devpriv->BADR1 = %d\n",devpriv->BADR1);
	printk("devpriv->BADR2 = %d\n",devpriv->BADR2);
	printk("devpriv->BADR3 = %d\n",devpriv->BADR3);
	printk("devpriv->BADR4 = %d\n",devpriv->BADR4);
#endif
	printk("comedi%d: cb_pcimdas: remove\n",dev->minor);
		if(devpriv->BADR0)
			release_mem_region(devpriv->BADR0, BADR0_SIZE);
		if(devpriv->BADR1)
			release_region(devpriv->BADR1, BADR1_SIZE);
		if(devpriv->BADR2)
			release_region(devpriv->BADR2, BADR2_SIZE);
		if(devpriv->BADR3)
			release_region(devpriv->BADR3, BADR3_SIZE);
		if(devpriv->BADR4)
			release_region(devpriv->BADR4, BADR4_SIZE);
	
	if(dev->irq)
		comedi_free_irq(dev->irq, dev);

	return 0;
}

/*
 * "instructions" read/write data in "one-shot" or "software-triggered"
 * mode.
 */
static int cb_pcimdas_ai_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n,i;
	unsigned int d;
	unsigned int busy;
	int chan = CR_CHAN(insn->chanspec);
	unsigned short chanlims;
	int maxchans;

	// only support sw initiated reads from a single channel

	//check channel number
	if ((inb(devpriv->BADR3+2) & 0x20)==0) //differential mode
		maxchans=thisboard->ai_diff_chans;
	else
		maxchans=thisboard->ai_se_chans;

	if (chan>(maxchans-1)) 
		return -ETIMEDOUT;  //*** Wrong error code. Fixme.

	//configure for sw initiated read
	d=inb(devpriv->BADR3+5);
	if ((d & 0x03)>0) { 			//only reset if needed.
		d=d & 0xfd;
		outb(d,devpriv->BADR3+5);
	}
	outb(0x01,devpriv->BADR3+6); //set bursting off, conversions on
	outb(0x00,devpriv->BADR3+7); //set range to 10V. UP/BP is controlled by a switch on the board

	// write channel limits to multiplexer, set Low (bits 0-3) and High (bits 4-7) channels to chan.
	chanlims=chan | (chan<<4);
	outb(chanlims,devpriv->BADR3+0);

	/* convert n samples */
	for(n=0;n<insn->n;n++){
		/* trigger conversion */
		outw(0,devpriv->BADR2+0);

#define TIMEOUT 1000 	//typically takes 5 loops on a lightly loaded Pentium 100MHz,
			//this is likely to be 100 loops on a 2GHz machine, so set 1000 as the limit.

		/* wait for conversion to end */
		for(i=0;i<TIMEOUT;i++){
			busy = inb(devpriv->BADR3+2)&0x80;
			if(!busy)break;
		}
		if(i==TIMEOUT){
			printk("timeout\n");
			return -ETIMEDOUT;
		}
		/* read data */
		d = inw(devpriv->BADR2+0);

		/* mangle the data as necessary */
		//d ^= 1<<(thisboard->ai_bits-1); // 16 bit data from ADC, so no mangle needed.

		data[n] = d;
	}

	/* return the number of samples read/written */
	return n;
}


static int cb_pcimdas_ao_winsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	/* Writing a list of values to an AO channel is probably not
	 * very useful, but that's how the interface is defined. */
	for(i=0;i<insn->n;i++){
		switch ( chan ) {
		case 0:	 
			outw(data[i] & 0x0FFF,devpriv->BADR2+DAC0_OFFSET);
			break;
		case 1:
			outw(data[i] & 0x0FFF,devpriv->BADR2+DAC1_OFFSET);
			break;
		default:
			return -1;
		}
		devpriv->ao_readback[chan] = data[i];
	}

	/* return the number of samples read/written */
	return i;
}

/* AO subdevices should have a read insn as well as a write insn.
 * Usually this means copying a value stored in devpriv. */
static int cb_pcimdas_ao_rinsn(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i;
	int chan = CR_CHAN(insn->chanspec);

	for(i=0;i<insn->n;i++)
		data[i] = devpriv->ao_readback[chan];

	return i;
}

/* DIO devices are slightly special.  Although it is possible to
 * implement the insn_read/insn_write interface, it is much more
 * useful to applications if you implement the insn_bits interface.
 * This allows packed reading/writing of the DIO channels.  The
 * comedi core can convert between insn_bits and insn_read/write */
static int cb_pcimdas_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
// Much of this based on the 8255 driver.

	if(data[0]){
		s->state &= ~data[0];
		s->state |= (data[0]&data[1]);
		
		if(data[0]&PORT_A_MASK) {
			devpriv->port_a=s->state&0xff;
			outb(devpriv->port_a,devpriv->BADR4+PORT_A);
		}
		if(data[0]&PORT_B_MASK) {
			devpriv->port_b=(s->state>>8)&0xff;
			outb(devpriv->port_b,devpriv->BADR4+PORT_B);
		}
		if(data[0]&PORT_C_MASK) {
			devpriv->port_c=(s->state>>16)&0xff;
			outb(devpriv->port_c,devpriv->BADR4+PORT_C);
		}
	}

	data[1]=inb(devpriv->BADR4+PORT_A);
	data[1]|=(inb(devpriv->BADR4+PORT_B)<<8);
	data[1]|=(inb(devpriv->BADR4+PORT_C)<<16);

	return 2; //should this be 2?
}

static int cb_pcimdas_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
// DIO config on this card is per port (or half port for port C), not per line.

	unsigned int mask;
	unsigned int bits;
	int config;

	mask=1<<CR_CHAN(insn->chanspec);
	if(mask&PORT_A_MASK){
		bits=PORT_A_MASK;
	}else if(mask&PORT_B_MASK){
		bits=PORT_B_MASK;
	}else if(mask&PORT_CL_MASK){
		bits=PORT_CL_MASK;
	}else{
		bits=PORT_CH_MASK;
	}

	switch(data[0]){
	case COMEDI_INPUT:
		s->io_bits&=~bits;
		break;
	case COMEDI_OUTPUT:
		s->io_bits|=bits;
		break;
	default:
		return -EINVAL;
	}

	config=ALL_OUTPUT;
	/* 0 in io_bits indicates output, 1 in config indicates input */
	if(!(s->io_bits&PORT_A_MASK))
		config|=PORT_A_IN;
	if(!(s->io_bits&PORT_B_MASK))
		config|=PORT_B_IN;
	if(!(s->io_bits&PORT_CL_MASK))
		config|=PORT_CL_IN;
	if(!(s->io_bits&PORT_CH_MASK))
		config|=PORT_CH_IN;

	outb(config,devpriv->BADR4+DIO_CONFIG);
	devpriv->dio_mode=config;

	return 1;
}
 

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver_cb_pcimdas);


