/*
    comedi/drivers/amplc_pc236.c
    Driver for Amplicon PC36AT and PCI236 DIO boards.

    Copyright (C) 2002 MEV Ltd. <http://www.mev.co.uk/>

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
Driver: amplc_pc236.o
Description: Driver for Amplicon PC36AT and PCI236 DIO boards
Author: Ian Abbott <abbotti@mev.co.uk>
Devices: [Amplicon] PC36AT (pc36at), PCI236 (pci236)
Updated: Fri, 23 Aug 2002 11:41:11 +0100
Status: works

Configuration options - PC36AT:
  [0] - I/O port base address
  [1] - IRQ (optional)

Configuration options - PCI236:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
  If bus/slot is not specified, the first available PCI device will be
  used.

The PC36AT ISA board and PCI236 PCI board have a single 8255 appearing
as subdevice 0.

Subdevice 1 pretends to be a digital input device, but it always returns
0 when read. However, if you run a command with scan_begin_src=TRIG_EXT,
a rising edge on port C bit 7 acts as an external trigger, which can be
used to wake up tasks.  This is like the comedi_parport device, but the
only way to physically disable the interrupt on the PC36AT is to remove
the IRQ jumper.  If no interrupt is connected, then subdevice 1 is
unused.
*/

#include <linux/comedidev.h>

#include <linux/pci.h>

#include "8255.h"
#include "plx9052.h"

#define PC236_DRIVER_NAME	"amplc_pc236"

/* PCI236 PCI configuration register information */
#define PCI_VENDOR_ID_AMPLICON 0x14dc
#define PCI_DEVICE_ID_AMPLICON_PCI236 0x0009


/* PC36AT / PCI236 registers */

#define PC236_IO_SIZE		4
#define PC236_LCR_IO_SIZE	128


/*
 * Board descriptions for Amplicon PC36AT and PCI236.
 */

enum pc236_bustype {isa_bustype, pci_bustype};
enum pc236_model {pc36at_model, pci236_model};

typedef struct pc236_board_struct{
	char *name;
	char *fancy_name;
	enum pc236_bustype bustype;
	enum pc236_model model;
}pc236_board;
static pc236_board pc236_boards[] = {
	{
	name:		"pc36at",
	fancy_name:	"PC36AT",
	bustype:	isa_bustype,
	model:		pc36at_model,
	},
	{
	name:		"pci236",
	fancy_name:	"PCI236",
	bustype:	pci_bustype,
	model:		pci236_model,
	},
};

static struct pci_device_id pc236_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_AMPLICON_PCI236, PCI_ANY_ID, PCI_ANY_ID, 0, 0, pci236_model },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, pc236_pci_table);

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((pc236_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct{
	/* PCI device */
	struct pci_dev *pci_dev;
	int lcr_iobase;	/* PLX PCI9052 config registers in PCIBAR1 */
	int enable_irq;
	int share_irq;
}pc236_private;

#define devpriv ((pc236_private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int pc236_attach(comedi_device *dev,comedi_devconfig *it);
static int pc236_detach(comedi_device *dev);
static comedi_driver driver_amplc_pc236={
	driver_name:	PC236_DRIVER_NAME,
	module:		THIS_MODULE,
	attach:		pc236_attach,
	detach:		pc236_detach,
	board_name:	pc236_boards,
	offset:		sizeof(pc236_board),
	num_names:	sizeof(pc236_boards) / sizeof(pc236_board),
};
COMEDI_INITCLEANUP(driver_amplc_pc236);


static int pc236_request_region(unsigned long from, unsigned long extent);
static void pc236_intr_disable(comedi_device *dev);
static void pc236_intr_enable(comedi_device *dev);
static int pc236_intr_check(comedi_device *dev);
static int pc236_intr_insn(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int pc236_intr_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd);
static int pc236_intr_cmd(comedi_device *dev,comedi_subdevice *s);
static int pc236_intr_cancel(comedi_device *dev,comedi_subdevice *s);
static irqreturn_t pc236_interrupt(int irq,void *d,struct pt_regs *regs);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int pc236_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	struct pci_dev *pci_dev = NULL;
	int iobase = 0, irq = 0;
	int lcr_iobase = 0;
	int bus = 0, slot = 0;
	struct pci_device_id *pci_id;
	int share_irq = 0;
	int ret;

	printk("comedi%d: %s: ",dev->minor, PC236_DRIVER_NAME);

	/* Get card bus position and base address. */
	switch (thisboard->bustype) {
	case isa_bustype:
		iobase = it->options[0];
		irq = it->options[1];
		share_irq = 0;
		break;
	case pci_bustype:
		bus = it->options[0];
		slot = it->options[1];
		share_irq = 1;

		/* Look for PCI table entry for this model. */
		for (pci_id = pc236_pci_table; pci_id->vendor != 0; pci_id++) {
			if (pci_id->driver_data == thisboard->model)
				break;
		}
		if (pci_id->vendor == 0) {
			printk("bug! cannot determine board type!\n");
			return -EINVAL;
		}

		/* Look for matching PCI device. */
		for(pci_dev = pci_get_device(pci_id->vendor, pci_id->device,
					NULL); pci_dev != NULL; 
				pci_dev = pci_get_device(pci_id->vendor,
					pci_id->device, pci_dev)) {
			/* If bus/slot specified, check them. */
			if (bus || slot) {
				if (bus != pci_dev->bus->number
						|| slot != PCI_SLOT(pci_dev->devfn))
					continue;
			}
#if 0
			if (pci_id->subvendor != PCI_ANY_ID) {
				if (pci_dev->subsystem_vendor != pci_id->subvendor)
					continue;
			}
			if (pci_id->subdevice != PCI_ANY_ID) {
				if (pci_dev->subsystem_device != pci_id->subdevice)
					continue;
			}
#endif
			if (((pci_dev->class ^ pci_id->class) & pci_id->class_mask) != 0)
				continue;
			/* Found a match. */
			break;
		}
		if (!pci_dev) {
			printk("no %s found!\n", thisboard->fancy_name);
			return -EIO;
		}
		if ((ret=pci_enable_device(pci_dev)) < 0) {
			printk("error enabling PCI device!\n");
			pci_dev_put(pci_dev);
			return ret;
		}
		lcr_iobase = pci_resource_start(pci_dev, 1);
		iobase = pci_resource_start(pci_dev, 2);
		irq = pci_dev->irq;
		break;
	default:
		printk("bug! cannot determine board type!\n");
		return -EINVAL;
		break;
	}

/*
 * Initialize dev->board_name.
 */
	dev->board_name = thisboard->name;
	printk("%s ", dev->board_name);

/*
 * Allocate the private structure area.  alloc_private() is a
 * convenient macro defined in comedidev.h.
 */
	if ((ret=alloc_private(dev,sizeof(pc236_private))) < 0) {
		printk("out of memory!\n");
		if (pci_dev)
			pci_dev_put(pci_dev);
		return ret; 
	}

	devpriv->pci_dev = pci_dev;
	devpriv->share_irq = share_irq;

	/* Reserve I/O spaces. */
	if (lcr_iobase) {
		if ((ret=pc236_request_region(lcr_iobase, PC236_LCR_IO_SIZE)) < 0) {
			return ret;
		}
		devpriv->lcr_iobase = lcr_iobase;
	}
	if ((ret=pc236_request_region(iobase, PC236_IO_SIZE)) < 0) {
		return ret;
	}
	dev->iobase = iobase;

/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.
 */
	if ((ret=alloc_subdevices(dev, 2)) < 0) {
		printk("out of memory!\n");
		return ret;
	}

	s = dev->subdevices+0;
	/* digital i/o subdevice (8255) */
	if ((ret=subdev_8255_init(dev, s, NULL, iobase)) < 0) {
		printk("out of memory!\n");
		return ret;
	}
	s = dev->subdevices+1;
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_UNUSED;
	pc236_intr_disable(dev);
	if (irq) {
		unsigned long flags = share_irq ? SA_SHIRQ : 0;

		if (comedi_request_irq(irq, pc236_interrupt, flags,
					PC236_DRIVER_NAME, dev) >= 0) {
			dev->irq = irq;
			s->type = COMEDI_SUBD_DI;
			s->subdev_flags = SDF_READABLE;
			s->n_chan = 1;
			s->maxdata = 1;
			s->range_table = &range_digital;
			s->insn_bits = pc236_intr_insn;
			s->do_cmdtest = pc236_intr_cmdtest;
			s->do_cmd = pc236_intr_cmd;
			s->cancel = pc236_intr_cancel;
		}
	}
	if (thisboard->bustype == isa_bustype) {
		printk("(base %#x) ", iobase);
	} else {
		printk("(pci %02x:%02x.%x) ", pci_dev->bus->number,
				PCI_SLOT(pci_dev->devfn),
				PCI_FUNC(pci_dev->devfn));
	}
	if (irq) {
		printk("(irq %d%s) ", irq, (dev->irq ? "" : " UNAVAILABLE"));
	} else {
		printk("(no irq) ");
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
static int pc236_detach(comedi_device *dev)
{
	printk("comedi%d: %s: remove\n", dev->minor, PC236_DRIVER_NAME);
	if (devpriv) {
		pc236_intr_disable(dev);
	}
	if (dev->irq) comedi_free_irq(dev->irq, dev);
	if (dev->subdevices) {
		subdev_8255_cleanup(dev, dev->subdevices+0);
	}
	if (dev->iobase) {
		release_region(dev->iobase, PC236_IO_SIZE);
	}
	if (devpriv) {
		if (devpriv->lcr_iobase)
			release_region(devpriv->lcr_iobase, PC236_LCR_IO_SIZE);
		if (devpriv->pci_dev)
			pci_dev_put(devpriv->pci_dev);
	}
	
	return 0;
}

/*
 * This function checks and requests an I/O region, reporting an error
 * if there is a conflict.
 */
static int pc236_request_region(unsigned long from, unsigned long extent)
{
	if (check_region(from, extent) < 0) {
		printk("I/O port conflict (%#lx,%lu)!\n", from, extent);
		return -EIO;
	}
	request_region(from, extent, PC236_DRIVER_NAME);
	return 0;
}

/*
 * This function is called to mark the interrupt as disabled (no command
 * configured on subdevice 1) and to physically disable the interrupt
 * (not possible on the PC36AT, except by removing the IRQ jumper!).
 */
static void pc236_intr_disable(comedi_device *dev)
{
	unsigned long flags;

	comedi_spin_lock_irqsave(&dev->spinlock, flags);
	devpriv->enable_irq = 0;
	if (devpriv->lcr_iobase)
		outl(PCI236_INTR_DISABLE, devpriv->lcr_iobase+PLX9052_INTCSR);
	comedi_spin_unlock_irqrestore(&dev->spinlock, flags);
}

/*
 * This function is called to mark the interrupt as enabled (a command
 * configured on subdevice 1) and to physically enable the interrupt
 * (not possible on the PC36AT, except by (re)connecting the IRQ jumper!).
 */
static void pc236_intr_enable(comedi_device *dev)
{
	unsigned long flags;

	comedi_spin_lock_irqsave(&dev->spinlock, flags);
	devpriv->enable_irq = 1;
	if (devpriv->lcr_iobase)
		outl(PCI236_INTR_ENABLE, devpriv->lcr_iobase+PLX9052_INTCSR);
	comedi_spin_unlock_irqrestore(&dev->spinlock, flags);
}

/*
 * This function is called when an interrupt occurs to check whether
 * the interrupt has been marked as enabled and was generated by the
 * board.  If so, the function prepares the hardware for the next
 * interrupt.
 * Returns 0 if the interrupt should be ignored.
 */
static int pc236_intr_check(comedi_device *dev)
{
	int retval = 0;
	unsigned long flags;

	comedi_spin_lock_irqsave(&dev->spinlock, flags);
	if (devpriv->enable_irq) {
		retval = 1;
		if (devpriv->lcr_iobase) {
			if ((inl(devpriv->lcr_iobase+PLX9052_INTCSR)
						& PLX9052_INTCSR_LI1STAT_MASK)
					== PLX9052_INTCSR_LI1STAT_SHIFTED_INACTIVE) {
				retval = 0;
			} else {
				/* Clear interrupt and keep it enabled. */
				outl(PCI236_INTR_ENABLE, devpriv->lcr_iobase+PLX9052_INTCSR);
			}
		}
	}
	comedi_spin_unlock_irqrestore(&dev->spinlock, flags);

	return retval;
}

/*
 * Input from subdevice 1.
 * Copied from the comedi_parport driver.
 */
static int pc236_intr_insn(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	data[1] = 0;
	return 2;
}

/*
 * Subdevice 1 command test.
 * Copied from the comedi_parport driver.
 */
static int pc236_intr_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd)
{
	int err=0;
	int tmp;

	/* step 1 */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_EXT;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_FOLLOW;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: ignored */

	if(err)return 2;

	/* step 3: */

	if(cmd->start_arg!=0){
		cmd->start_arg = 0;
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
		cmd->scan_end_arg = 1;
		err++;
	}
	if(cmd->stop_arg!=0){
		cmd->stop_arg = 0;
		err++;
	}

	if(err)return 3;

	/* step 4: ignored */

	if(err)return 4;

	return 0;
}

/*
 * Subdevice 1 command.
 */
static int pc236_intr_cmd(comedi_device *dev,comedi_subdevice *s)
{
	pc236_intr_enable(dev);

	return 0;
}

/*
 * Subdevice 1 cancel command.
 */
static int pc236_intr_cancel(comedi_device *dev,comedi_subdevice *s)
{
	pc236_intr_disable(dev);

	return 0;
}

/*
 * Interrupt service routine.
 * Based on the comedi_parport driver.
 */
static irqreturn_t pc236_interrupt(int irq,void *d,struct pt_regs *regs)
{
	comedi_device *dev=d;
	comedi_subdevice *s=dev->subdevices+1;
	int retval = 1;

	if(!pc236_intr_check(dev))
		return IRQ_RETVAL(retval);

	comedi_buf_put(s->async,0);
	s->async->events |= COMEDI_CB_BLOCK | COMEDI_CB_EOS;

	comedi_event(dev,s,s->async->events);
	return IRQ_RETVAL(retval);
}

