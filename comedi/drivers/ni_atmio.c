/*
    comedi/drivers/ni_atmio.c
    Hardware driver for NI AT-MIO E series cards

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2001 David A. Schleef <ds@schleef.org>

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
Driver: ni_atmio.o
Description: National Instruments AT-MIO-E series
Author: ds
Devices: [National Instruments] AT-MIO-16E-1 (ni_atmio),
  AT-MIO-16E-2, AT-MIO-16E-10, AT-MIO-16DE-10, AT-MIO-64E-3,
  AT-MIO-16XE-50, AT-MIO-16XE-10, AT-AI-16XE-10
Status: works
Updated: Sat, 16 Mar 2002 17:34:48 -0800

The isapnptools package is required to use this board.  Use isapnp to
configure the I/O base for the board, and then pass the same value as
a parameter in comedi_config.  A sample isapnp.conf file is included
in the etc/ directory of Comedilib.

Comedilib includes a utility to autocalibrate these boards.  The
boards seem to boot into a state where the all calibration DACs
are at one extreme of their range, thus the default calibration
is terrible.  Calibration at boot is strongly encouraged.

To use the extended digital I/O on some of the boards, enable the
8255 driver when configuring the Comedi source tree.

External triggering is supported for some events.  The channel index
(scan_begin_arg, etc.) maps to PFI0 - PFI9.

Some of the more esoteric triggering possibilities of these boards
are not supported.
*/
/*
	The real guts of the driver is in ni_mio_common.c, which is included
	both here and in ni_pcimio.c

	
	Interrupt support added by Truxton Fulton <trux@truxton.com>

	References for specifications:
	
	   340747b.pdf  Register Level Programmer Manual (obsolete)
	   340747c.pdf  Register Level Programmer Manual (new)
	   DAQ-STC reference manual

	Other possibly relevant info:
	
	   320517c.pdf  User manual (obsolete)
	   320517f.pdf  User manual (new)
	   320889a.pdf  delete
	   320906c.pdf  maximum signal ratings
	   321066a.pdf  about 16x
	   321791a.pdf  discontinuation of at-mio-16e-10 rev. c
	   321808a.pdf  about at-mio-16e-10 rev P
	   321837a.pdf  discontinuation of at-mio-16de-10 rev d
	   321838a.pdf  about at-mio-16de-10 rev N
	
	ISSUES:

	need to deal with external reference for DAC, and other DAC
	properties in board properties
	
	deal with at-mio-16de-10 revision D to N changes, etc.
	
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/slab.h>
#include <linux/comedidev.h>
#ifdef HAVE_ISAPNP
#include <linux/isapnp.h>
#include <linux/pci.h>
#endif
#include "ni_stc.h"
#include "8255.h"

#undef DEBUG

#define ATMIO 1
#undef PCIMIO

/*
 *  AT specific setup
 */

#define NI_SIZE 0x20

#define MAX_N_CALDACS 12

static ni_board ni_boards[]={
	{	device_id:	44,
		name:		"at-mio-16e-1",
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	8192,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	800,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	2048,
		ao_unipolar:	1,
		has_8255:	0,
		caldac:		{mb88341},
	},
	{	device_id:	25,
		name:		"at-mio-16e-2",
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	2048,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	2000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	2048,
		ao_unipolar:	1,
		has_8255:	0,
		caldac:		{mb88341},
	},
	{	device_id:	36,
		name:		"at-mio-16e-10",
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	512,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	10000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	0,
		ao_unipolar:	1,
		caldac:		{mb88341},
		has_8255:	0,
	},
	{	device_id:	37,
		name:		"at-mio-16de-10",
		n_adchan:	16,
		adbits:		12,
		ai_fifo_depth:	512,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	10000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	0,
		ao_unipolar:	1,
		caldac:		{mb88341},
		has_8255:	1,
	},
	{	device_id:	38,
		name:		"at-mio-64e-3",
		n_adchan:	64,
		adbits:		12,
		ai_fifo_depth:	2048,
		alwaysdither:	0,
		gainlkup:	ai_gain_16,
		ai_speed:	2000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	2048,
		ao_unipolar:	1,
		has_8255:	0,
		caldac:		{mb88341},
	},
	{	device_id:	39,
		name:		"at-mio-16xe-50",
		n_adchan:	16,
		adbits:		16,
		ai_fifo_depth:	512,
		alwaysdither:	1,
		gainlkup:	ai_gain_8,
		ai_speed:	50000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	0,
		ao_unipolar:	0,
		caldac:		{ad8804,dac8043},
		has_8255:	0,
	},
	{	device_id:	50,
		name:		"at-mio-16xe-10",
		n_adchan:	16,
		adbits:		16,
		ai_fifo_depth:	512,
		alwaysdither:	1,	/* unknown */
		gainlkup:	ai_gain_14,
		ai_speed:	10000,
		n_aochan:	2,
		aobits:		12,
		ao_fifo_depth:	0,	/* unknown */
		ao_unipolar:	0,	/* unknown */
		caldac:		{ad8804,dac8043},
		has_8255:	0,
	},
	{	device_id:	51,
		name:		"at-ai-16xe-10",
		n_adchan:	16,
		adbits:		16,
		ai_fifo_depth:	512,
		alwaysdither:	1,	/* unknown */
		gainlkup:	ai_gain_14,
		ai_speed:	10000,
		n_aochan:	0,
		aobits:		0,
		ao_fifo_depth:	0,
		aorangelkup:	0,
		ao_unipolar:	0,
		caldac:		{dac8800,dac8043},
		has_8255:	0,
	}
};


static int ni_irqpin[]={-1,-1,-1,0,1,2,-1,3,-1,-1,4,5,6,-1,-1,7};

#define interrupt_pin(a)	(ni_irqpin[(a)])

#define IRQ_POLARITY 0

#define NI_E_IRQ_FLAGS		0


/* How we access registers */

#define ni_writel(a,b)		(outl((a),(b)+dev->iobase))
#define ni_readl(a)		(inl((a)+dev->iobase))
#define ni_writew(a,b)		(outw((a),(b)+dev->iobase))
#define ni_readw(a)		(inw((a)+dev->iobase))
#define ni_writeb(a,b)		(outb((a),(b)+dev->iobase))
#define ni_readb(a)		(inb((a)+dev->iobase))

/* How we access windowed registers */

/* We automatically take advantage of STC registers that can be
 * read/written directly in the I/O space of the board.  The
 * AT-MIO devices map the low 8 STC registers to iobase+addr*2. */

#define win_out(data,addr) do{ \
	if((addr)<8){ \
		ni_writew((data),(addr)*2); \
	}else{ \
		ni_writew((addr),Window_Address); \
		ni_writew((data),Window_Data); \
	} \
}while(0)

#define win_out2(data,addr) do{ \
	win_out((data)>>16, (addr)); \
	win_out((data)&0xffff, (addr)+1); \
}while(0)

#define win_in(addr) ( \
	((addr)<8) \
	? (ni_readw(((addr))*2)) \
	: (ni_writew((addr),Window_Address),ni_readw(Window_Data)))

#define win_save() (ni_readw(Window_Address))
#define win_restore(a) (ni_writew((a),Window_Address))

#define ao_win_out(a,b) do{ \
	ni_writew((b),AO_Window_Address_671x); \
	ni_writew((a),AO_Window_Data_671x); \
}while(0)



#ifdef HAVE_ISAPNP
static struct isapnp_device_id device_ids[] = {
	{ ISAPNP_DEVICE_SINGLE('N','I','C',0x1900,'N','I','C',0x0000), },
	{ ISAPNP_DEVICE_SINGLE_END, },
};
MODULE_DEVICE_TABLE(isapnp, device_ids);
#endif

typedef struct{
#ifdef HAVE_ISAPNP
	struct pci_dev *pcidev;
#endif
	NI_PRIVATE_COMMON
}ni_private;
#define devpriv ((ni_private *)dev->private)

static int ni_atmio_attach(comedi_device *dev,comedi_devconfig *it);
static int ni_atmio_detach(comedi_device *dev);
static comedi_driver driver_atmio={
	driver_name:	"ni_atmio",
	module:		THIS_MODULE,
	attach:		ni_atmio_attach,
	detach:		ni_atmio_detach,
};

COMEDI_INITCLEANUP(driver_atmio);

#include "ni_mio_common.c"

static int ni_getboardtype(comedi_device *dev);

/* clean up allocated resources */
static int ni_atmio_detach(comedi_device *dev)
{
	mio_common_detach(dev);

#ifdef HAVE_ISAPNP
	if(devpriv->pcidev)
		devpriv->pcidev->deactivate(devpriv->pcidev);
#else
	if(dev->iobase)
		release_region(dev->iobase,NI_SIZE);
	if(dev->irq){
		comedi_free_irq(dev->irq,dev);
	}
#endif

	return 0;
}

static int ni_atmio_attach(comedi_device *dev,comedi_devconfig *it)
{
#ifdef HAVE_ISAPNP
	struct pci_dev *pcidev;
#endif
	int		ret;
	int		iobase;
	int		board;
	int		irq;


#ifdef HAVE_ISAPNP
	pcidev = isapnp_find_dev(NULL,
		ISAPNP_VENDOR('N','I','C'),
		ISAPNP_FUNCTION(0x1900),
		NULL);

	if(!pcidev)
		return -ENODEV;

	if(pcidev->active)
		return -EBUSY;

	if(pcidev->prepare(pcidev)<0)
		return -EAGAIN;

	if(!(pcidev->resource[0].flags & IORESOURCE_IO))
		return -ENODEV;

	if(!pcidev->ro){
		/* override resource */
		if(it->options[0] != 0){
			isapnp_resource_change(&pcidev->resource[0],
				it->options[0], 1);
		}
	}
	if(pcidev->activate(pcidev)<0){
		printk("isapnp configure failed!\n");
		return -ENOMEM;
	}
	iobase = pcidev->resource[0].start;
	irq = pcidev->irq;
#else
	iobase=0x200;
	if(it->options[0])iobase=it->options[0];
	irq=it->options[1];
#endif
	
	/* reserve our I/O region */
	
	printk("comedi%d: ni_atmio: 0x%04x",dev->minor,iobase);
	if(check_region(iobase,NI_SIZE)<0){
		printk(" I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase,NI_SIZE,"ni_atmio");

	dev->iobase=iobase;
	
#ifdef DEBUG
	/* board existence sanity check */
       {
               int i;

               printk(" board fingerprint:");
              for(i=0;i<16;i+=2){
                       printk(" %04x %02x",inw(dev->iobase+i),inb(dev->iobase+i+1));
               }
       }
#endif
	
	/* get board type */

	board=ni_getboardtype(dev);
	if(board<0)return -EIO;

	dev->board_ptr=ni_boards + board;
	
	printk(" %s",boardtype.name);
	dev->board_name=boardtype.name;

	/* irq stuff */

	if(irq!=0){
		if(irq<0 || irq>15 || ni_irqpin[irq]==-1){
			printk(" invalid irq\n");
			return -EINVAL;
		}
		printk(" ( irq = %d )",irq);
		if( (ret=comedi_request_irq(irq,ni_E_interrupt,NI_E_IRQ_FLAGS,"ni_atmio",dev))<0 ){
			printk(" irq not available\n");
			return -EINVAL;
		}
		dev->irq=irq;
	}
	
	/* allocate private area */
	
	if((ret=alloc_private(dev,sizeof(ni_private)))<0)
		return ret;

	/* generic E series stuff in ni_mio_common.c */

	if( (ret=ni_E_init(dev,it))<0 ){
		return ret;
	}
	
	return 0;
}


static int ni_getboardtype(comedi_device *dev)
{
	int device_id=ni_read_eeprom(dev,511);
	int i;
	
	for(i=0;i<n_ni_boards;i++){
		if(ni_boards[i].device_id==device_id){
			return i;
		}
	}
	if(device_id==255){
		printk(" can't find board\n");
	}else if(device_id == 0){
		printk(" EEPROM read error (?) or device not found\n");
	}else{
		printk(" unknown device ID %d -- contact author\n",device_id);
	}
	return -1;
}

