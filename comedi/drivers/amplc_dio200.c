/*
    comedi/drivers/amplc_dio200.c
    Driver for Amplicon PC272E and PCI272 DIO boards.
    (Support for other boards in Amplicon 200 series may be added at
    a later date, e.g. PCI215.)

    Copyright (C) 2005 MEV Ltd. <http://www.mev.co.uk/>

    Includes parts of the 8255 driver
    Copyright (C) 1998 David A. Schleef <ds@schleef.org>

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1998,2000 David A. Schleef <ds@schleef.org>

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
Driver: amplc_dio200.o
Description: Amplicon PC272E, PCI272
Author: Ian Abbott <abbotti@mev.co.uk>
Devices: [Amplicon] PC272E (pc272e), PCI272 (pci272)
Updated: Fri, 11 Feb 2005 13:13:13 +0000
Status: works

Configuration options - PC272E:
  [0] - I/O port base address
  [1] - IRQ (optional, but commands won't work without it)

Configuration options - PCI272:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
  If bus/slot is not specified, the first available PCI device will
  be used.

Passing a zero for an option is the same as leaving it unspecified.


SUBDEVICES

                 PC272E/PCI272
                 -------------
  Subdevices           4
   0                 PPI-X
   1                 PPI-Y
   2                 PPI-Z
   3               INTERRUPT


Each PPI is a 8255 chip providing 24 DIO channels.  The DIO channels
are configurable as inputs or outputs in four groups:

  Port A  - channels  0 to  7
  Port B  - channels  8 to 15
  Port CL - channels 16 to 19
  Port CH - channels 20 to 23

Only mode 0 of the 8255 chips is supported.

The 'INTERRUPT' subdevice pretends to be a digital input subdevice.
The digital inputs come from the interrupt status register. The number
of channels matches the number of interrupt sources.


INTERRUPT SOURCES

                 PC272E/PCI272
                 -------------
  Sources              6
   0               PPI-X-C0
   1               PPI-X-C3
   2               PPI-Y-C0
   3               PPI-Y-C3
   4               PPI-Z-C0
   5               PPI-Z-C3

When an interrupt source is enabled in the interrupt source enable
register, a rising edge on the source signal latches the corresponding
bit to 1 in the interrupt status register.

When the interrupt status register value as a whole (actually, just the
6 least significant bits) goes from zero to non-zero, the board will
generate an interrupt.  For level-triggered hardware interrupts (PCI
card), the interrupt will remain asserted until the interrupt status
register is cleared to zero.  For edge-triggered hardware interrupts
(ISA card), no further interrupts will occur until the interrupt status
register is cleared to zero.  To clear a bit to zero in the interrupt
status register, the corresponding interrupt source must be disabled
in the interrupt source enable register (there is no separate interrupt
clear register).


COMMANDS

The driver supports a read streaming acquisition command on the
'INTERRUPT' subdevice.  The channel list selects the interrupt sources
to be enabled.  All channels will be sampled together (convert_src ==
TRIG_NOW).  The scan begins a short time after the hardware interrupt
occurs, subject to interrupt latencies (scan_begin_src == TRIG_EXT,
scan_begin_arg == 0).  The value read from the interrupt status register
is packed into a sampl_t value, one bit per requested channel, in the
order they appear in the channel list.


TODO LIST

Support for PC212E, PC215E, PCI215 and possibly PC218E should be added.
Apart from the PC218E, these consist of a mixture of 8255 DIO chips and
8254 counter chips with software configuration of the clock and gate
sources for the 8254 chips.  (The PC218E has 6 8254 counter chips but
no 8255 DIO chips.)
*/

#include <linux/comedidev.h>

#include <linux/pci.h>

#include "8255.h"

#define DIO200_DRIVER_NAME	"amplc_dio200"

/* PCI IDs */
/* #define PCI_VENDOR_ID_AMPLICON 0x14dc */
#define PCI_DEVICE_ID_AMPLICON_PCI272 0x000a

/* 200 series registers */
#define DIO200_IO_SIZE		0x20
#define DIO200_INT_SCE		0x1e	/* Interrupt enable/status register */

/*
 * Board descriptions.
 */

enum dio200_bustype	{ isa_bustype, pci_bustype };
enum dio200_model	{ pc272e_model, pci272_model };
enum dio200_layout	{ pc272_layout };

typedef struct dio200_board_struct {
	char *name;
	enum dio200_bustype bustype;
	enum dio200_model model;
	enum dio200_layout layout;
} dio200_board;

static dio200_board dio200_boards[] = {
	{
	name:		"pc272e",
	bustype:	isa_bustype,
	model:		pc272e_model,
	layout:		pc272_layout,
	},
	{
	name:		"pci272",
	bustype:	pci_bustype,
	model:		pci272_model,
	layout:		pc272_layout,
	},
};

/*
 * Layout descriptions - some ISA and PCI board descriptions share the same
 * layout.
 */

enum dio200_sdtype	{ sd_none, sd_intr, sd_8255 };

#define DIO200_MAX_SUBDEVS	4
#define DIO200_MAX_ISNS		6

typedef struct dio200_layout_struct {
	unsigned short n_subdevs;	/* number of subdevices */
	unsigned char sdtype[DIO200_MAX_SUBDEVS]; /* enum dio200_sdtype */
	unsigned char sdinfo[DIO200_MAX_SUBDEVS]; /* depends on sdtype */
	short read_sd;			/* 'read' subdevice' if >= 0 */
} dio200_layout;

static dio200_layout dio200_layouts[] = {
	[pc272_layout] = {
		n_subdevs:	4,
		sdtype:		{ sd_8255, sd_8255, sd_8255, sd_intr },
		sdinfo:		{ 0x00, 0x08, 0x10, 0x3F },
	},
};

/*
 * PCI driver table.
 */

static struct pci_device_id dio200_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_AMPLICON_PCI272,
	  PCI_ANY_ID, PCI_ANY_ID, 0, 0, pci272_model },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, dio200_pci_table);

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((dio200_board *)dev->board_ptr)
#define thislayout (&dio200_layouts[((dio200_board *)dev->board_ptr)->layout])

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct {
	struct pci_dev *pci_dev;	/* PCI device */
	int share_irq;
	int intr_sd;
} dio200_private;

#define devpriv ((dio200_private *)dev->private)

typedef struct {
	unsigned long iobase;
	spinlock_t spinlock;
	int active;
	unsigned int valid_isns;
	unsigned int enabled_isns;
	unsigned int stopcount;
	int continuous;
} dio200_subdev_intr;


/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int dio200_attach(comedi_device *dev,comedi_devconfig *it);
static int dio200_detach(comedi_device *dev);
static comedi_driver driver_amplc_dio200 = {
	driver_name:	DIO200_DRIVER_NAME,
	module:		THIS_MODULE,
	attach:		dio200_attach,
	detach:		dio200_detach,
	board_name:	dio200_boards,
	offset:		sizeof(dio200_board),
	num_names:	sizeof(dio200_boards) / sizeof(dio200_board),
};
COMEDI_INITCLEANUP(driver_amplc_dio200);

/*
 * This function looks for a PCI device matching the requested board name,
 * bus and slot.
 */
static int
dio200_find_pci(comedi_device *dev, int bus, int slot,
		struct pci_dev **pci_dev_p)
{
	struct pci_dev *pci_dev = NULL;
	struct pci_device_id *pci_id;

	*pci_dev_p = NULL;

	/* Look for PCI table entry for this model. */
	for (pci_id = dio200_pci_table; pci_id->vendor != 0; pci_id++) {
		if (pci_id->driver_data == thisboard->model)
			break;
	}
	if (pci_id->vendor == 0) {
		printk(KERN_ERR "comedi%d: %s: BUG! cannot determine board type!\n",
				dev->minor, DIO200_DRIVER_NAME);
		return -EINVAL;
	}

	/* Look for matching PCI device. */
	for(pci_dev = pci_get_device(pci_id->vendor, pci_id->device, NULL);
			pci_dev != NULL ; 
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
		*pci_dev_p = pci_dev;
		return 0;
	}
	/* No match found. */
	if (bus || slot) {
		printk(KERN_ERR "comedi%d: error! no %s found at pci %02x:%02x!\n",
				dev->minor, thisboard->name,
				bus, slot);
	} else {
		printk(KERN_ERR "comedi%d: error! no %s found!\n",
				dev->minor, thisboard->name);
	}
	return -EIO;
}

/*
 * This function checks and requests an I/O region, reporting an error
 * if there is a conflict.
 */
static int
dio200_request_region(unsigned minor, unsigned long from, unsigned long extent)
{
	if (!request_region(from, extent, DIO200_DRIVER_NAME)) {
		printk(KERN_ERR "comedi%d: I/O port conflict (%#lx,%lu)!\n",
				minor, from, extent);
		return -EIO;
	}
	return 0;
}

/*
 * 'insn_bits' function for an 'INTERRUPT' subdevice.
 */
static int
dio200_subdev_intr_insn_bits(comedi_device *dev, comedi_subdevice *s,
		comedi_insn *insn, lsampl_t *data)
{
	dio200_subdev_intr *subpriv = s->private;

	/* Just read the interrupt status register.  */
	data[1] = inb(subpriv->iobase) & subpriv->valid_isns;

	return 2;
}

/*
 * Called to stop acquisition for an 'INTERRUPT' subdevice.
 */
static void
dio200_stop_intr(comedi_device *dev, comedi_subdevice *s)
{
	dio200_subdev_intr *subpriv = s->private;

	s->async->inttrig = 0;
	subpriv->active = 0;
	subpriv->enabled_isns = 0;
	outb(0, subpriv->iobase);
}

/*
 * Called to start acquisition for an 'INTERRUPT' subdevice.
 */
static int
dio200_start_intr(comedi_device *dev, comedi_subdevice *s)
{
	unsigned int n;
	unsigned isn_bits;
	dio200_subdev_intr *subpriv = s->private;
	comedi_cmd *cmd = &s->async->cmd;
	int retval = 0;

	if (!subpriv->continuous && subpriv->stopcount == 0) {
		/* An empty acquisition! */
		s->async->events |= COMEDI_CB_EOA;
		subpriv->active = 0;
		retval = 1;
	} else {
		/* Determine interrupt sources to enable. */
		isn_bits = 0;
		if (cmd->chanlist) {
			for (n = 0; n < cmd->chanlist_len; n++) {
				isn_bits |= (1U << CR_CHAN(cmd->chanlist[n]));
			}
		}
		isn_bits &= subpriv->valid_isns;
		/* Enable interrupt sources. */
		subpriv->enabled_isns = isn_bits;
		outb(isn_bits, subpriv->iobase);
	}

	return retval;
}

/*
 * Internal trigger function to start acquisition for an 'INTERRUPT' subdevice.
 */
static int
dio200_inttrig_start_intr(comedi_device *dev, comedi_subdevice *s,
		unsigned int trignum)
{
	dio200_subdev_intr *subpriv;
	unsigned long flags;
	int event = 0;

	if (trignum != 0) return -EINVAL;

	subpriv = s->private;

	comedi_spin_lock_irqsave(&subpriv->spinlock, flags);
	s->async->inttrig = 0;
	if (subpriv->active) {
		event = dio200_start_intr(dev, s);
	}
	comedi_spin_unlock_irqrestore(&subpriv->spinlock, flags);

	if (event) {
		comedi_event(dev, s, s->async->events);
	}

	return 1;
}

/*
 * This is called from the interrupt service routine to handle a read
 * scan on an 'INTERRUPT' subdevice.
 */
static int
dio200_handle_read_intr(comedi_device *dev, comedi_subdevice *s)
{
	dio200_subdev_intr *subpriv = s->private;
	unsigned triggered;
	unsigned intstat;
	unsigned cur_enabled;
	unsigned int oldevents;
	unsigned long flags;

	triggered = 0;

	comedi_spin_lock_irqsave(&subpriv->spinlock, flags);
	oldevents = s->async->events;
	/*
	 * Collect interrupt sources that have triggered and disable them
	 * temporarily.  Loop around until no extra interrupt sources have
	 * triggered, at which point, the valid part of the interrupt status
	 * register will read zero, clearing the cause of the interrupt.
	 */
	cur_enabled = subpriv->enabled_isns;
	while ((intstat = (inb(subpriv->iobase) & subpriv->valid_isns)) != 0) {
		triggered |= intstat;
		cur_enabled &= ~triggered;
		outb(cur_enabled, subpriv->iobase);
	}
	
	if (triggered) {
		/*
		 * Some interrupt sources have triggered and have been
		 * temporarily disabled to clear the cause of the interrupt.
		 *
		 * Reenable them NOW to minimize the time they are disabled.
		 */
		cur_enabled = subpriv->enabled_isns;
		outb(cur_enabled, subpriv->iobase);
		
		if (subpriv->active) {
			/*
			 * The command is still active.
			 *
			 * Ignore interrupt sources that the command isn't
			 * interested in (just in case there's a race
			 * condition).
			 */
			if (triggered & subpriv->enabled_isns) {
				/* Collect scan data. */
				sampl_t val;
				unsigned int n, ch, len;

				val = 0;
				len = s->async->cmd.chanlist_len;
				for (n = 0; n < len; n++) {
					ch = CR_CHAN(s->async->cmd.chanlist[n]);
					if (triggered & (1U << ch)) {
						val |= (1U << n);
					}
				}
				/* Write the scan to the buffer. */
				if (comedi_buf_put(s->async, val)) {
					s->async->events |= (COMEDI_CB_BLOCK |
							     COMEDI_CB_EOS);
				} else {
					/* Error!  Stop acquisition.  */
					dio200_stop_intr(dev, s);
				}

				/* Check for end of acquisition. */
				if (!subpriv->continuous) {
					/* stop_src == TRIG_COUNT */
					if (subpriv->stopcount > 0) {
						subpriv->stopcount--;
						if (subpriv->stopcount == 0) {
							s->async->events |=
								COMEDI_CB_EOA;
							dio200_stop_intr(dev, s);
						}
					}
				}
			}
		}
	}
	comedi_spin_unlock_irqrestore(&subpriv->spinlock, flags);

	if (oldevents != s->async->events) {
		comedi_event(dev, s, s->async->events);
	}

	return (triggered != 0);
}

/*
 * 'cancel' function for an 'INTERRUPT' subdevice.
 */
static int
dio200_subdev_intr_cancel(comedi_device *dev, comedi_subdevice *s)
{
	dio200_subdev_intr *subpriv = s->private;
	unsigned long flags;

	comedi_spin_lock_irqsave(&subpriv->spinlock, flags);
	if (subpriv->active) {
		dio200_stop_intr(dev, s);
	}
	comedi_spin_unlock_irqrestore(&subpriv->spinlock, flags);

	return 0;
}

/*
 * 'do_cmdtest' function for an 'INTERRUPT' subdevice.
 */
static int
dio200_subdev_intr_cmdtest(comedi_device *dev, comedi_subdevice *s,
		comedi_cmd *cmd)
{
	int err = 0;
	unsigned int tmp;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= (TRIG_NOW | TRIG_INT);
	if (!cmd->start_src || tmp != cmd->start_src) err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_EXT;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src) err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_NOW;
	if (!cmd->convert_src || tmp != cmd->convert_src) err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;  
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src) err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= (TRIG_COUNT | TRIG_NONE);
	if (!cmd->stop_src || tmp != cmd->stop_src) err++;

	if (err) return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	/* these tests are true if more than one _src bit is set */
	if ((cmd->start_src & (cmd->start_src - 1)) != 0) err++;
	if ((cmd->scan_begin_src & (cmd->scan_begin_src - 1)) != 0) err++;
	if ((cmd->convert_src & (cmd->convert_src - 1)) != 0) err++;
	if ((cmd->scan_end_src & (cmd->scan_end_src - 1)) != 0) err++;
	if ((cmd->stop_src & (cmd->stop_src - 1)) != 0) err++;

	if (err) return 2;

	/* step 3: make sure arguments are trivially compatible */

	/* cmd->start_src == TRIG_NOW || cmd->start_src == TRIG_INT */
	if (cmd->start_arg != 0) {
		cmd->start_arg = 0;
		err++;
	}

	/* cmd->scan_begin_src == TRIG_EXT */
	if (cmd->scan_begin_arg != 0) {
		cmd->scan_begin_arg = 0;
		err++;
	}

	/* cmd->convert_src == TRIG_NOW */
	if (cmd->convert_arg != 0) {
		cmd->convert_arg = 0;
		err++;
	}

	/* cmd->scan_end_src == TRIG_COUNT */
	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}

	switch (cmd->stop_src) {
	case TRIG_COUNT:
		/* any count allowed */
		break;
	case TRIG_NONE:
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
		break;
	default:
		break;
	}

	if (err) return 3;

	/* step 4: fix up any arguments */

	/* if (err) return 4; */

	return 0;
}

/*
 * 'do_cmd' function for an 'INTERRUPT' subdevice.
 */
static int
dio200_subdev_intr_cmd(comedi_device *dev, comedi_subdevice *s)
{
	comedi_cmd *cmd = &s->async->cmd;
	dio200_subdev_intr *subpriv = s->private;
	unsigned long flags;
	int event = 0;

	comedi_spin_lock_irqsave(&subpriv->spinlock, flags);
	subpriv->active = 1;

	/* Set up end of acquisition. */
	switch (cmd->stop_src) {
	case TRIG_COUNT:
		subpriv->continuous = 0;
		subpriv->stopcount = cmd->stop_arg;
		break;
	default:
		/* TRIG_NONE */
		subpriv->continuous = 1;
		subpriv->stopcount = 0;
		break;
	}

	/* Set up start of acquisition. */
	switch (cmd->start_src) {
	case TRIG_INT:
		s->async->inttrig = dio200_inttrig_start_intr;
		break;
	default:
		/* TRIG_NOW */
		event = dio200_start_intr(dev, s);
		break;
	}
	comedi_spin_unlock_irqrestore(&subpriv->spinlock, flags);

	if (event) {
		comedi_event(dev, s, s->async->events);
	}

	return 0;
}

/*
 * This function initializes an 'INTERRUPT' subdevice.
 */
static int
dio200_subdev_intr_init(comedi_device *dev, comedi_subdevice *s,
		unsigned long iobase, unsigned valid_isns)
{
	dio200_subdev_intr *subpriv;

	subpriv = kmalloc(sizeof(*subpriv), GFP_KERNEL);
	if (!subpriv) {
		printk(KERN_ERR "comedi%d: error! out of memory!\n", dev->minor);
		return -ENOMEM;
	}
	memset(subpriv, 0, sizeof(*subpriv));
	subpriv->iobase = iobase;
	subpriv->valid_isns = valid_isns;
	spin_lock_init(&subpriv->spinlock);

	outb(0, subpriv->iobase);	/* Disable interrupt sources. */

	s->private = subpriv;
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan = DIO200_MAX_ISNS;
	s->len_chanlist = DIO200_MAX_ISNS;
	s->range_table = &range_digital;
	s->maxdata = 1;
	s->insn_bits = dio200_subdev_intr_insn_bits;
	s->do_cmdtest = dio200_subdev_intr_cmdtest;
	s->do_cmd = dio200_subdev_intr_cmd;
	s->cancel = dio200_subdev_intr_cancel;

	return 0;
}

/*
 * This function cleans up an 'INTERRUPT' subdevice.
 */
static void
dio200_subdev_intr_cleanup(comedi_device *dev, comedi_subdevice *s)
{
	dio200_subdev_intr *subpriv = s->private;

	if (subpriv) {
		kfree(subpriv);
	}
}

/*
 * Interrupt service routine.
 */
static irqreturn_t
dio200_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev=d;
	int handled;

	if (devpriv->intr_sd >= 0) {
		handled = dio200_handle_read_intr(dev,
				dev->subdevices + devpriv->intr_sd);
	} else {
		handled = 0;
	}

	return IRQ_RETVAL(handled);
}

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int
dio200_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	struct pci_dev *pci_dev = NULL;
	int iobase = 0, irq = 0;
	int bus = 0, slot = 0;
	dio200_layout *layout;
	int share_irq = 0;
	int sdx;
	unsigned n;
	int ret;

	printk(KERN_DEBUG "comedi%d: %s: attach\n", dev->minor,
			DIO200_DRIVER_NAME);

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

		if ((ret=dio200_find_pci(dev, bus, slot, &pci_dev)) < 0)
			return ret;

		if ((ret=pci_enable_device(pci_dev)) < 0) {
			printk(KERN_ERR "comedi%d: error! cannot enable PCI device!\n",
					dev->minor);
			pci_dev_put(pci_dev);
			return ret;
		}
		iobase = pci_resource_start(pci_dev, 2);
		irq = pci_dev->irq;
		break;
	default:
		printk(KERN_ERR "comedi%d: %s: BUG! cannot determine board type!\n",
				dev->minor, DIO200_DRIVER_NAME);
		return -EINVAL;
		break;
	}

	if ((ret=alloc_private(dev,sizeof(dio200_private))) < 0) {
		printk(KERN_ERR "comedi%d: error! out of memory!\n", dev->minor);
		if (pci_dev) {
			pci_dev_put(pci_dev);
		}
		return ret; 
	}

	devpriv->pci_dev = pci_dev;
	devpriv->share_irq = share_irq;
	devpriv->intr_sd = -1;

	/* Reserve I/O spaces. */
	ret = dio200_request_region(dev->minor, iobase, DIO200_IO_SIZE);
	if (ret < 0) {
		return ret;
	}
	dev->iobase = iobase;

	layout = thislayout;
	if ((ret=alloc_subdevices(dev, layout->n_subdevs)) < 0) {
		printk(KERN_ERR "comedi%d: error! out of memory!\n", dev->minor);
		return ret;
	}

	for (n = 0; n < dev->n_subdevices; n++) {
		s = &dev->subdevices[n];
		switch (layout->sdtype[n]) {
		case sd_8255:
			/* digital i/o subdevice (8255) */
			ret = subdev_8255_init(dev, s, 0,
					iobase + layout->sdinfo[n]);
			if (ret < 0) {
				return ret;
			}
			break;
		case sd_intr:
			/* 'INTERRUPT' subdevice */
			if (irq) {
				ret = dio200_subdev_intr_init(dev, s,
						iobase + DIO200_INT_SCE,
						layout->sdinfo[n]);
				if (ret < 0) {
					return ret;
				}
				devpriv->intr_sd = n;
			} else {
				s->type = COMEDI_SUBD_UNUSED;
			}
			break;
		default:
			s->type = COMEDI_SUBD_UNUSED;
			break;
		}
	}

	sdx = devpriv->intr_sd;
	if (sdx >= 0 && sdx < dev->n_subdevices) {
		dev->read_subdev = &dev->subdevices[sdx];
	}

	dev->board_name = thisboard->name;

	if (irq) {
		unsigned long flags = share_irq ? SA_SHIRQ : 0;

		if (comedi_request_irq(irq, dio200_interrupt, flags,
					DIO200_DRIVER_NAME, dev) >= 0) {
			dev->irq = irq;
		} else {
			printk(KERN_WARNING "comedi%d: warning! irq %d unavailable!\n",
					dev->minor, irq);
		}
	}

	printk(KERN_INFO "comedi%d: %s ", dev->minor, dev->board_name);
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
static int
dio200_detach(comedi_device *dev)
{
	dio200_layout *layout;
	unsigned n;

	printk(KERN_DEBUG "comedi%d: %s: detach\n", dev->minor,
			DIO200_DRIVER_NAME);

	if (dev->irq) {
		comedi_free_irq(dev->irq, dev);
	}
	if (dev->subdevices) {
		layout = thislayout;
		for (n = 0; n < dev->n_subdevices; n++) {
			comedi_subdevice *s = &dev->subdevices[n];
			switch (layout->sdtype[n]) {
			case sd_8255:
				subdev_8255_cleanup(dev, s);
				break;
			case sd_intr:
				dio200_subdev_intr_cleanup(dev, s);
				break;
			default:
				break;
			}
		}
	}
	if (dev->iobase) {
		release_region(dev->iobase, DIO200_IO_SIZE);
	}
	if (devpriv && devpriv->pci_dev) {
		pci_dev_put(devpriv->pci_dev);
	}
	if (dev->board_name) {
		printk(KERN_INFO "comedi%d: %s removed\n",
				dev->minor, dev->board_name);
	}
	
	return 0;
}

