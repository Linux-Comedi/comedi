/*
    module/ni_pcimio.c
    Hardware driver for NI PCI-MIO E series cards

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

*/

/*
	The PCI-MIO E series driver was originally written by
	Tomasz Motylewski <...>, and ported to comedi by ds.


	References for specifications:
	
	   321747b.pdf  Register Level Programmer Manual (obsolete)
	   321747c.pdf  Register Level Programmer Manual (new)
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
	
	need to add other CALDAC type
	
	need to slow down DAC loading.  I don't trust NI's claim that
	two writes to the PCI bus slows IO enough.  I would prefer to
	use udelay().  Timing specs: (clock)
		AD8522		30ns
		DAC8043		120ns
		DAC8800		60ns
		MB88341		?

*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/malloc.h>
#include <linux/comedidev.h>
#include <ni_stc.h>
#include <mite.h>

#undef DEBUG
#define PCI_DEBUG

//#define PCIDMA

#define PCIMIO 1
#undef ATMIO

static struct caldac_struct *type1[]={&caldac_dac8800,&caldac_dac8043,NULL};
static struct caldac_struct *type2[]={&caldac_dac8800,&caldac_dac8043,&caldac_ad8522};
static struct caldac_struct *type3[]={&caldac_mb88341,NULL,NULL};
static struct caldac_struct *type4[]={&caldac_mb88341,&caldac_mb88341,&caldac_ad8522};


static ni_board ni_boards[]={
	{       device_id:      0x0162, // NI also says 0x1620.  typo?
		name:           "pci-mio-16xe-50",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  2048,
		alwaysdither:   1,
		gainlkup:       ai_gain_8,
		ai_speed:	50000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		caldac:         type1,
		has_8255:       0,
	},
	{       device_id:      0x1170,
		name:           "pci-mio-16xe-10",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  2048,
		ao_unipolar:    1,
		caldac:         type2,
		has_8255:       0,
	},
	{       device_id:      0x1180,
		name:           "pci-mio-16e-1",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_16,
		ai_speed:	800,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  2048,
		ao_unipolar:    1,
		caldac:         type3,
		has_8255:       0,
	},
	{       device_id:      0x1190,
		name:           "pci-mio-16e-4",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_16,
		ai_speed:	500,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  512,
		ao_unipolar:    1,
		caldac:         type3,
		has_8255:       0,
	},
	{       device_id:      0x1330,
		name:           "pci-6031e",
		n_adchan:       64,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       2,
		aobits:         16,
		ao_fifo_depth:  2048,
		ao_unipolar:    1,
		caldac:         type2,
		has_8255:       0,
	},
	{       device_id:      0x1270,
		name:           "pci-6032e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		ao_unipolar:    1,
		caldac:         type2,
		has_8255:       0,
	},
	{       device_id:      0x1340,
		name:           "pci-6033e",
		n_adchan:       64,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_14,
		ai_speed:	10000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		ao_unipolar:    1,
		caldac:         type2,
		has_8255:       0,
	},
	{       device_id:      0x1350,
		name:           "pci-6071e",
		n_adchan:       64,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:	800,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  2048,
		ao_unipolar:    1,
		caldac:         type3,
		has_8255:       0,
	},
	{       device_id:      0x2a60,
		name:           "pci-6023e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_8_602x,
		ai_speed:	5000,
		n_aochan:       0,
		aobits:         0,
		ao_unipolar:    0,
		caldac:         type3,
		has_8255:	0,
	},
	{       device_id:      0x2a70,
		name:           "pci-6024e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_8_602x,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		ao_unipolar:    1,
		caldac:         type3,
		has_8255:	0,
	},
	{       device_id:      0x2a80,
		name:           "pci-6025e",
		n_adchan:       16,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_8_602x,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		ao_unipolar:    1,
		caldac:         type3,
		has_8255:	1,
	},
	{       device_id:      0x2ca0,
		name:           "pci-6034e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_4_603x,
		ai_speed:	5000,
		n_aochan:       0,
		aobits:         0,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		caldac:         type3,
		has_8255:	0,
	},
	{       device_id:      0x2c80,
		name:           "pci-6035e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_4_603x,
		ai_speed:	5000,
		n_aochan:       2,
		aobits:         12,
		ao_fifo_depth:  0,
		ao_unipolar:    0,
		caldac:         type3,
		has_8255:	0,
	},
	{       device_id:      0x18b0,
		name:           "pci-6052e",
		n_adchan:       16,
		adbits:         16,
		ai_fifo_depth:  512,
		alwaysdither:   1,
		gainlkup:       ai_gain_16,
		ai_speed:	3000,
		n_aochan:       2,
		aobits:         16,
		ao_unipolar:    1,
		ao_fifo_depth:  2048,
		caldac:         type4,
	},
	{       device_id:      0x14e0,		/* unknown */
		name:           "pci-6110e",
		n_adchan:       4, 
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_14,	/* wrong */
		ai_speed:	200,
		aobits:         16,
		ao_unipolar:    0,
		ao_fifo_depth:  2048,
		caldac:         type4,		/* XXX */
	},
	{       device_id:      0x14f0,		/* unknown */
		name:           "pci-6111e",
		n_adchan:       2,
		adbits:         12,
		ai_fifo_depth:  512,
		alwaysdither:   0,
		gainlkup:       ai_gain_14,
		ai_speed:	200,
		n_aochan:       2,
		aobits:         12,
		ao_unipolar:    0,
		ao_fifo_depth:  2048,
		caldac:         type4,		/* XXX */
	},
#if 0
	{       device_id:      0x0000,		/* unknown */
		name:           "pci-6711",
		n_adchan:       0,
		adbits:         0,
		ai_fifo_depth:  0,
		alwaysdither:   0,
		gainlkup:       0,
		ai_speed:	0,
		n_aochan:	4,
		aobits:         12,
		ao_unipolar:    0,
		ao_fifo_depth:  8192,
		caldac:         type4,		/* XXX */
	},
#endif
	{       device_id:      0x1870,
		name:           "pci-6713",
		n_adchan:       0,
		adbits:         0,
		ai_fifo_depth:  0,
		alwaysdither:   0,
		gainlkup:       0,
		ai_speed:	0,
		n_aochan:	8,
		aobits:         12,
		ao_unipolar:    0,
		ao_fifo_depth:  16384,
		caldac:         type4,		/* XXX */
	},
#if 0
	{       device_id:      0x0000,		/* unknown */
		name:           "pci-6040e",
		ai_speed:	800,
	},
	{       device_id:      0x0000,		/* unknown */
		name:           "pci-6041e",
		ai_speed:	800,
	},
	{       device_id:      0x0000,		/* unknown */
		name:           "pci-1200",
		ai_speed:	10000,
	},
	{       device_id:      0x0000,		/* unknown */
		name:           "ni-5911 for PCI",
		ai_speed:	10000,
	},
	{       device_id:      0x0000,		/* unknown */
		name:           "ni-5102 for PCI",
		ai_speed:	10000,
	},
#endif
};
#define n_pcimio_boards ((sizeof(ni_boards)/sizeof(ni_boards[0])))

static int pcimio_attach(comedi_device *dev,comedi_devconfig *it);
static int pcimio_detach(comedi_device *dev);
comedi_driver driver_pcimio={
	driver_name:	"ni_pcimio",
	module:		THIS_MODULE,
	attach:		pcimio_attach,
	detach: 	pcimio_detach,
};
COMEDI_INITCLEANUP(driver_pcimio);


/* How we access registers */


#define ni_writew(a,b)		(writew((a),dev->iobase+(b)))
#define ni_readw(a)		(readw(dev->iobase+(a)))
#define ni_writeb(a,b)		(writeb((a),dev->iobase+(b)))
#define ni_readb(a)		(readb(dev->iobase+(a)))
#define ni_writeb_p(a,b)	(ni_writeb(a,b),ni_writeb(a,b))
#define ni_readb_p(a)		(ni_readb(a),ni_readb(a))


/*
 * this is how we access windowed registers
 */
#define win_out(a,b) (ni_writew((b),Window_Address),ni_writew((a),Window_Data))
#define win_in(b) (ni_writew((b),Window_Address),ni_readw(Window_Data))
#define win_save() (ni_readw(Window_Address))
#define win_restore(a) (ni_writew((a),Window_Address))

/*
   If interrupts _still_ don't work, play with the
   following two values.
 */
#define interrupt_pin(a)	0
#define IRQ_POLARITY 1

#define NI_E_IRQ_FLAGS		SA_SHIRQ


typedef struct{
	struct mite_struct *mite;

	NI_PRIVATE_COMMON
}ni_private;
#define devpriv ((ni_private *)dev->private)


#include "ni_mio_common.c"


static int pcimio_find_device(comedi_device *dev,int bus,int slot);


/* cleans up allocated resources */
static int pcimio_detach(comedi_device *dev)
{
	if(dev->private && devpriv->mite)
		mite_unsetup(devpriv->mite);
	
	if(dev->irq){
		comedi_free_irq(dev->irq,dev);
	}

	return 0;
}

static int pcimio_attach(comedi_device *dev,comedi_devconfig *it)
{
	int		ret;
	
	printk("comedi%d: ni_pcimio:",dev->minor);
	
	ret=alloc_private(dev,sizeof(ni_private));
	if(ret<0)return ret;

	ret=pcimio_find_device(dev,it->options[0],it->options[1]);
	if(ret<0)return ret;

	printk(" %s",boardtype.name);
	dev->board_name=boardtype.name;
	
	dev->iobase=mite_setup(devpriv->mite);

	dev->irq=mite_irq(devpriv->mite);

        if(dev->irq==0){
		printk(" unknown irq (bad)\n");
	}else{
        	printk(" ( irq = %d )",dev->irq);
        	if( (ret=comedi_request_irq(dev->irq,ni_E_interrupt,NI_E_IRQ_FLAGS,"ni_pcimio",dev))<0 ){
                	printk(" irq not available\n");
			dev->irq=0;
        	}
	}
	return ni_E_init(dev,it);
}


static int pcimio_find_device(comedi_device *dev,int bus,int slot)
{
	struct mite_struct *mite;
	int i;

	for(mite=mite_devices;mite;mite=mite->next){
		if(mite->used)continue;
		if(bus || slot){
#ifdef PCI_SUPPORT_VER1
			if(bus!=mite->pci_bus ||
			   slot!=PCI_SLOT(mite->pci_device_fn))
				continue;
#else
			if(bus!=mite->pcidev->bus->number ||
			   slot!=PCI_SLOT(mite->pcidev->devfn))
				continue;
#endif
		}

		for(i=0;i<n_pcimio_boards;i++){
			if(mite_device_id(mite)==ni_boards[i].device_id){
				dev->board_ptr=ni_boards+i;
				devpriv->mite=mite;

				return 0;
			}
		}
	}
	printk("no device found\n");
	mite_list_devices();
	return -EIO;
}

