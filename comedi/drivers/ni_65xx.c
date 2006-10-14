/*
    comedi/drivers/ni_6514.c
    driver for National Instruments PCI-6514

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1999,2002,2003 David A. Schleef <ds@schleef.org>

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
Driver: ni_65xx.o
Description: National Instruments 65xx static dio boards
Author: Jon Grierson <jd@renko.co.uk>
Status: testing
Devices: [National Instruments] PCI-6514 (ni6514), PXI-6514
Updated: Mon, 17 Jul 2006 16:40:10 +0100

Based on the PCI-6527 driver by ds.
Should be easily modified for 6509, 651x, 6520, 6521 and 6528

*/

/*
   Manuals (available from ftp://ftp.natinst.com/support/manuals)

	370106b.pdf	6514 Register Level Programmer Manual

 */

#define DEBUG 1
#define DEBUG_FLAGS

#include <linux/comedidev.h>

#include "mite.h"


#define NI6514_DIO_SIZE 4096
#define NI6514_MITE_SIZE 4096


#define Port_Register(x)			(0x40+(x))
#define ID_Register				0x00

#define Clear_Register				0x01
#define ClrEdge				0x08
#define ClrOverflow			0x04

#define Filter_Interval			0x08
#define Filter_Enable(x)			(0x44+(x))

#define Change_Status				0x02
#define MasterInterruptStatus		0x04
#define Overflow			0x02
#define EdgeStatus			0x01

#define Master_Interrupt_Control		0x03
#define FallingEdgeIntEnable		0x10
#define RisingEdgeIntEnable		0x08
#define MasterInterruptEnable		0x04
#define OverflowIntEnable		0x02
#define EdgeIntEnable			0x01

#define Rising_Edge_Detection_Enable(x)		(0x42+(x))
#define Falling_Edge_Detection_Enable(x)	(0x43+(x))



static int ni6514_attach(comedi_device *dev,comedi_devconfig *it);
static int ni6514_detach(comedi_device *dev);
static comedi_driver driver_ni6514={
	driver_name:	"ni6514",
	module:		THIS_MODULE,
	attach:		ni6514_attach,
	detach:		ni6514_detach,
};
COMEDI_INITCLEANUP(driver_ni6514);

typedef struct{
	int dev_id;
	char *name;
}ni6514_board;
static ni6514_board ni6514_boards[] = {
	{
	dev_id:		0x7088,
	name:		"pci-6514",
	},
	{
	dev_id:		0x70CD,
	name:		"pxi-6514",
	},
};

#define n_ni6514_boards (sizeof(ni6514_boards)/sizeof(ni6514_boards[0]))
#define this_board ((ni6514_board *)dev->board_ptr)

static struct pci_device_id ni6514_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_NATINST, 0x7088, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x70CD, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, ni6514_pci_table);

typedef struct{
	struct mite_struct *mite;
	unsigned int filter_interval;
	unsigned int filter_enable;
}ni6514_private;
#define devpriv ((ni6514_private *)dev->private)

static int ni6514_find_device(comedi_device *dev,int bus,int slot);


static int ni6514_di_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int chan = CR_CHAN(insn->chanspec);
	unsigned int interval;

	if(insn->n!=2)return -EINVAL;

	if(data[0] != INSN_CONFIG_FILTER)return -EINVAL;

	if(data[1]){
		interval = (data[1]+100)/200;
		data[1] = interval*200;

		if(interval!=devpriv->filter_interval){
			writeb(interval&0x000fffff, devpriv->mite->daq_io_addr + Filter_Interval);  /* FIXME 32 bit reg - mask possibly wrong??? */

			devpriv->filter_interval = interval;
		}

		devpriv->filter_enable |= 1<<chan;
	}else{
		devpriv->filter_enable &= ~(1<<chan);
	}
	
	writeb(devpriv->filter_enable, devpriv->mite->daq_io_addr + Filter_Enable(0));
	writeb(devpriv->filter_enable>>8, devpriv->mite->daq_io_addr + Filter_Enable(0x10));
	writeb(devpriv->filter_enable>>16, devpriv->mite->daq_io_addr + Filter_Enable(0x20));
	writeb(devpriv->filter_enable>>24, devpriv->mite->daq_io_addr + Filter_Enable(0x30));

	return 2;
}

static int ni6514_di_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=2)return -EINVAL;

	data[1] = readb(devpriv->mite->daq_io_addr+Port_Register(0));
	data[1] |= readb(devpriv->mite->daq_io_addr+Port_Register(0x10))<<8;
	data[1] |= readb(devpriv->mite->daq_io_addr+Port_Register(0x20))<<16;
	data[1] |= readb(devpriv->mite->daq_io_addr+Port_Register(0x30))<<24;

	return 2;
}

static int ni6514_do_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=2)return -EINVAL;
	if(data[0]){
		s->state &= ~data[0];
		s->state |= (data[0]&data[1]);

		/* The open relay state on the board cooresponds to 1,
		 * but in Comedi, it is represented by 0. */
		if(data[0]&0x000000ff){
			writeb((s->state^0xff),devpriv->mite->daq_io_addr+Port_Register(0x40));
		}
		if(data[0]&0x0000ff00){
			writeb((s->state>>8)^0xff,devpriv->mite->daq_io_addr+Port_Register(0x50));
		}
		if(data[0]&0x00ff0000){
			writeb((s->state>>16)^0xff,devpriv->mite->daq_io_addr+Port_Register(0x60));
		}
		if(data[0]&0xff000000){
			writeb((s->state>>24)^0xff,devpriv->mite->daq_io_addr+Port_Register(0x70));
		}
	}
	data[1] = s->state;

	return 2;
}

static irqreturn_t ni6514_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 2;
	unsigned int status;

	status = readb(devpriv->mite->daq_io_addr + Change_Status);
	if((status & MasterInterruptStatus) == 0) return IRQ_NONE;
	if((status & EdgeStatus) == 0) return IRQ_NONE;

	writeb(ClrEdge | ClrOverflow, devpriv->mite->daq_io_addr + Clear_Register);

	comedi_buf_put(s->async, 0);
	s->async->events |= COMEDI_CB_EOS;
	comedi_event(dev,s,s->async->events);
	return IRQ_HANDLED;
}

static int ni6514_intr_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd)
{
	int err=0;
	int tmp;

	/* step 1: make sure trigger sources are trivially valid */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_OTHER;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_FOLLOW;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}
	if(cmd->scan_begin_arg!=0){
		cmd->scan_begin_arg = 0;
		err++;
	}
	if(cmd->convert_arg!=0){
		cmd->convert_arg = 0;
		err++;
	}

	if(cmd->scan_end_arg!=1){
		cmd->scan_end_arg=1;
		err++;
	}
	if(cmd->stop_arg!=0){
		cmd->stop_arg=0;
		err++;
	}

	if(err)return 3;

	/* step 4: fix up any arguments */
	
	if(err)return 4;

	return 0;
}

static int ni6514_intr_cmd(comedi_device *dev,comedi_subdevice *s)
{
	//comedi_cmd *cmd = &s->async->cmd;
	
	writeb(ClrEdge|ClrOverflow, devpriv->mite->daq_io_addr + Clear_Register);
	writeb(FallingEdgeIntEnable|RisingEdgeIntEnable|
		MasterInterruptEnable|EdgeIntEnable,
		devpriv->mite->daq_io_addr + Master_Interrupt_Control);
	
	return 0;
}

static int ni6514_intr_cancel(comedi_device *dev,comedi_subdevice *s)
{
	writeb(0x00, devpriv->mite->daq_io_addr + Master_Interrupt_Control);

	return 0;
}

static int ni6514_intr_insn_bits(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	if(insn->n < 1)return -EINVAL;

	data[1] = 0;
	return 2;
}

static int ni6514_intr_insn_config(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	if(insn->n < 1)return -EINVAL;
	if(data[0] != INSN_CONFIG_CHANGE_NOTIFY)return -EINVAL;

	writeb(data[1], devpriv->mite->daq_io_addr + Rising_Edge_Detection_Enable(0));
	writeb(data[1]>>8, devpriv->mite->daq_io_addr + Rising_Edge_Detection_Enable(0x10));
	writeb(data[1]>>16, devpriv->mite->daq_io_addr + Rising_Edge_Detection_Enable(0x20));
	writeb(data[1]>>24, devpriv->mite->daq_io_addr + Rising_Edge_Detection_Enable(0x30));

	writeb(data[2], devpriv->mite->daq_io_addr + Falling_Edge_Detection_Enable(0));
	writeb(data[2]>>8, devpriv->mite->daq_io_addr + Falling_Edge_Detection_Enable(0x10));
	writeb(data[2]>>16, devpriv->mite->daq_io_addr + Falling_Edge_Detection_Enable(0x20));
	writeb(data[2]>>24, devpriv->mite->daq_io_addr + Falling_Edge_Detection_Enable(0x30));

	return 2;
}


static int ni6514_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret;
	
	printk("comedi%d: ni6514:",dev->minor);

	if((ret=alloc_private(dev,sizeof(ni6514_private)))<0)
		return ret;
	
	ret=ni6514_find_device(dev,it->options[0],it->options[1]);
	if(ret<0)return ret;

	ret = mite_setup(devpriv->mite);
	if(ret < 0)
	{
		printk("error setting up mite\n");
		return ret;
	}

	dev->board_name=this_board->name;
	dev->irq=mite_irq(devpriv->mite);
	printk(" %s",dev->board_name);

	printk(" ID=0x%02x", readb(devpriv->mite->daq_io_addr + ID_Register));

	if((ret=alloc_subdevices(dev,3))<0)
		return ret;

	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=32;
	s->range_table=&range_digital;
	s->maxdata=1;
	s->insn_config = ni6514_di_insn_config;
	s->insn_bits = ni6514_di_insn_bits;

	s=dev->subdevices+1;
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_READABLE|SDF_WRITABLE;
	s->n_chan=32;
	s->range_table=&range_digital;
	s->maxdata=1;
	s->insn_bits = ni6514_do_insn_bits;

	s=dev->subdevices + 2;
	dev->read_subdev = s;
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=1;
	s->range_table=&range_unknown;
	s->maxdata=1;
	s->do_cmdtest = ni6514_intr_cmdtest;
	s->do_cmd = ni6514_intr_cmd;
	s->cancel = ni6514_intr_cancel;
	s->insn_bits = ni6514_intr_insn_bits;
	s->insn_config = ni6514_intr_insn_config;

	writeb(0x00, devpriv->mite->daq_io_addr + Filter_Enable(0));
	writeb(0x00, devpriv->mite->daq_io_addr + Filter_Enable(0x10));
	writeb(0x00, devpriv->mite->daq_io_addr + Filter_Enable(0x20));
	writeb(0x00, devpriv->mite->daq_io_addr + Filter_Enable(0x30));

	writeb(0x00,devpriv->mite->daq_io_addr+Port_Register(0x40));
	writeb(0x00,devpriv->mite->daq_io_addr+Port_Register(0x50));
	writeb(0x00,devpriv->mite->daq_io_addr+Port_Register(0x60));
	writeb(0x00,devpriv->mite->daq_io_addr+Port_Register(0x70));

	writeb(ClrEdge|ClrOverflow,
		devpriv->mite->daq_io_addr + Clear_Register);
	writeb(0x00, devpriv->mite->daq_io_addr + Master_Interrupt_Control);
	
	/* Set filter interval to 0  (32bit reg) */
	writeb(0x00000000, devpriv->mite->daq_io_addr + Filter_Interval);
	
	ret=comedi_request_irq(dev->irq,ni6514_interrupt,SA_SHIRQ,"ni6514",dev);
	if(ret<0){
		dev->irq=0;
		printk(" irq not available");
	}

	printk("\n");

	return 0;
}

static int ni6514_detach(comedi_device *dev)
{
	if(devpriv && devpriv->mite && devpriv->mite->daq_io_addr){
		writeb(0x00, devpriv->mite->daq_io_addr + Master_Interrupt_Control);
	}

	if(dev->irq){
		comedi_free_irq(dev->irq,dev);
	}

	if(devpriv && devpriv->mite){
		mite_unsetup(devpriv->mite);
	}

	return 0;
}

static int ni6514_find_device(comedi_device *dev,int bus,int slot)
{
	struct mite_struct *mite;
	int i;
	
	for(mite=mite_devices;mite;mite=mite->next){
		if(mite->used)continue;
		if(bus || slot){
			if(bus!=mite->pcidev->bus->number ||
			   slot!=PCI_SLOT(mite->pcidev->devfn))
				continue;
		}
		for(i=0;i<n_ni6514_boards;i++){
			if(mite_device_id(mite)==ni6514_boards[i].dev_id){
				dev->board_ptr = ni6514_boards + i;
				devpriv->mite=mite;
				return 0;
			}
		}
	}
	printk("no device found\n");
	mite_list_devices();
	return -EIO;
}

