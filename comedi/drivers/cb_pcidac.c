/*
    comedi/drivers/cb_pcidac.c
    Comedi driver for Computer Boards PCI-DAC6702

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
Driver: cb_pcidac
Description: Measurement Computing PCI Migration series boards
Devices: [ComputerBoards] PCI-DAC6702 (cb_pcidac)
Author: Oliver Gause
Updated: Fr, 31 Aug 2012
Status: works

Written to support the PCI-DAC6702. Should be trivial to add support for
the PCI-DAC6703, it has just 16 ao channels instead of 8.

Configuration Options:
    [0] - PCI bus number
    [1] - PCI slot number

Developed from cb_pcidas64, cb_pcimdas and skel. The register values are
taken from the register map of Measurement Computing.

Supports DIO, AO in its present form.

*/

#include <linux/comedidev.h>

#include <linux/delay.h>

#include "comedi_pci.h"

#define CBPCIDAC_DEBUG
//#undef CBPCIDAC_DEBUG


//DAC Offsets (16bit)
#define DAC_CHSELECT 0x08 // write only, number of channel to be updated
#define DAC_DATA 0x0a	// write only, data to be written to the selected channel
#define DAC_STARTUP 0x00 // write only, if bit 7 is set, the present channel values will be written to nonvolatile memory
#define DAC_UPDATE_MODE 0x02 // write only, each bit represents the update mode of the respective channel: 0 immediate update, 1 delayed update
#define DAC_UPDATE 0x06	// write only, default mode is immediate update, all writes trigger the delayed update
#define DAC_READOUT 0x04 // read only, readout of the selected channel

//DIO OFFSETS Read/Write (8bit)

#define DIO_DIRECTION 0x00 // DIO Data Direction Bits
#define DIO_DATA 0x01	// DIO Data Bits


// indices of base address regions
enum base_address_regions {
	PLX9030_BADDRINDEX = 0,
	MAIN_BADDRINDEX = 2,
	DIO_COUNTER_BADDRINDEX = 3,
};

//AI and Counter Constants

/* Board description */
typedef struct cb_pcidac_board_struct {
  const char *name;
  unsigned short device_id;
  int ao_nchan;		// number of analog out channels
  int ao_bits;		// analogue output resolution
  int has_ao_fifo;	// analog output has fifo
  int ao_scan_speed;	// analog output speed for 1602 series (for a scan, not conversion)
  int dio_bits;		// number of dio bits
  int dio_nchan;	// number of dio channels
  int has_dio;		// has DIO
  const comedi_lrange *ranges;
} cb_pcidac_board;

static const cb_pcidac_board cb_pcidac_boards[] = {
  {
  name:	"PCI-DAC6702",
  device_id:0x70,
  ao_nchan:8,
  ao_bits:16,
  has_ao_fifo:0,
  dio_bits:8,
  dio_nchan:8,
  has_dio:1,
  },
};

/* This is used by modprobe to translate PCI IDs to drivers.  Should
 * only be used for PCI and ISA-PnP devices */
static DEFINE_PCI_DEVICE_TABLE(cb_pcidac_pci_table) = {
  {PCI_VENDOR_ID_COMPUTERBOARDS, 0x0070 , PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
  {0}
};

MODULE_DEVICE_TABLE(pci, cb_pcidac_pci_table);

#define N_BOARDS 1		// Max number of boards supported

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((const cb_pcidac_board *)dev->board_ptr)

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct {
  int data;

  // would be useful for a PCI device
  struct pci_dev *pci_dev;

  /* Used for AO readback */
  lsampl_t ao_readback[2];

  // base addresses (physical)
  resource_size_t plx9030_phys_iobase;
  resource_size_t main_phys_iobase;
  resource_size_t dio_counter_phys_iobase;
  // base addresses (ioremapped)
  void *plx9030_iobase;
  void *main_iobase;
  void *dio_counter_iobase;

} cb_pcidac_private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((cb_pcidac_private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int cb_pcidac_attach(comedi_device * dev, comedi_devconfig * it);
static int cb_pcidac_detach(comedi_device * dev);
static comedi_driver driver_cb_pcidac = {
 driver_name:"cb_pcidac",
 module:THIS_MODULE,
 attach:cb_pcidac_attach,
 detach:cb_pcidac_detach,
};

static int cb_pcidac_ao_winsn(comedi_device * dev, comedi_subdevice * s,
			      comedi_insn * insn, lsampl_t * data);
static int cb_pcidac_ao_rinsn(comedi_device * dev, comedi_subdevice * s,
			      comedi_insn * insn, lsampl_t * data);
static int cb_pcidac_dio_insn_bits(comedi_device * dev, comedi_subdevice * s,
			      comedi_insn * insn, lsampl_t * data);
static int cb_pcidac_dio_insn_config(comedi_device * dev, comedi_subdevice * s,
			      comedi_insn * insn, lsampl_t * data);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int cb_pcidac_attach(comedi_device * dev, comedi_devconfig * it)
{
  comedi_subdevice *s;
  struct pci_dev *pcidev;
  int index;
  //int i;

  printk("comedi%d: cb_pcidac: ", dev->minor);

  /*
   * Allocate the private structure area.
   */
  if (alloc_private(dev, sizeof(cb_pcidac_private)) < 0)
    return -ENOMEM;

  /*
   * Probe the device to determine what device in the series it is.
   */
  printk("\n");

  for (pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, NULL);
       pcidev != NULL;
       pcidev = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pcidev)) {
    // is it not a computer boards card?
    if (pcidev->vendor != PCI_VENDOR_ID_COMPUTERBOARDS)
      continue;
    // loop through cards supported by this driver
    for (index = 0; index < N_BOARDS; index++) {
      if (cb_pcidac_boards[index].device_id !=
	  pcidev->device)
	continue;
      // was a particular bus/slot requested?
      if (it->options[0] || it->options[1]) {
	// are we on the wrong bus/slot?
	if (pcidev->bus->number != it->options[0] ||
	    PCI_SLOT(pcidev->devfn) !=
	    it->options[1]) {
	  continue;
	}
      }
      devpriv->pci_dev = pcidev;
      dev->board_ptr = cb_pcidac_boards + index;
      goto found;
    }
  }

  printk("No supported ComputerBoards/MeasurementComputing card found on "
	 "requested position\n");
  return -EIO;

 found:

  printk("Found %s on bus %i, slot %i\n", cb_pcidac_boards[index].name,
	 pcidev->bus->number, PCI_SLOT(pcidev->devfn));

  // Warn about non-tested features
  switch (thisboard->device_id) {
  case 0x70:
    break;
  default:
    printk("THIS CARD IS UNSUPPORTED.\n"
	   "PLEASE REPORT USAGE TO <mocelet@sucs.org>\n");
  };

  if (comedi_pci_enable(pcidev, "cb_pcidac")) {
    printk(" Failed to enable PCI device and request regions\n");
    return -EIO;
  }




  // Dont support IRQ yet
  //      // get irq
  //      if(comedi_request_irq(devpriv->pci_dev->irq, cb_pcidac_interrupt, IRQF_SHARED, "cb_pcidac", dev ))
  //      {
  //              printk(" unable to allocate irq %u\n", devpriv->pci_dev->irq);
  //              return -EINVAL;
  //      }
  //      dev->irq = devpriv->pci_dev->irq;

  //Initialize dev->board_name
	dev->board_name = thisboard->name;

  	//Initialize dev->board_name


	devpriv->plx9030_phys_iobase =
		pci_resource_start(pcidev, PLX9030_BADDRINDEX);
	devpriv->main_phys_iobase =
		pci_resource_start(pcidev, MAIN_BADDRINDEX);
	devpriv->dio_counter_phys_iobase =
		pci_resource_start(pcidev, DIO_COUNTER_BADDRINDEX);
		

	// remap, won't work with 2.0 kernels but who cares
	devpriv->plx9030_iobase = ioremap(devpriv->plx9030_phys_iobase,
		pci_resource_len(pcidev, PLX9030_BADDRINDEX));
	devpriv->main_iobase = ioremap(devpriv->main_phys_iobase,
		pci_resource_len(pcidev, MAIN_BADDRINDEX));
	devpriv->dio_counter_iobase =
		ioremap(devpriv->dio_counter_phys_iobase,
		pci_resource_len(pcidev, DIO_COUNTER_BADDRINDEX));

	if (!devpriv->plx9030_iobase || !devpriv->main_iobase
		|| !devpriv->dio_counter_iobase) {
		printk(" failed to remap io memory\n");
		return -ENOMEM;
	}
 
  
  /*
   * Allocate the subdevice structures.  alloc_subdevice() is a
   * convenient macro defined in comedidev.h.
   */
  if (alloc_subdevices(dev, 2) < 0)
    return -ENOMEM;

  s = dev->subdevices + 0;
  // analog output subdevice
  s->type = COMEDI_SUBD_AO;
  s->subdev_flags = SDF_WRITABLE;
  s->n_chan = thisboard->ao_nchan;
  s->maxdata = 1 << thisboard->ao_bits;
  s->range_table = &range_bipolar10;
  s->insn_write = &cb_pcidac_ao_winsn;
  s->insn_read = &cb_pcidac_ao_rinsn;

  s = dev->subdevices + 1;
  /* digital i/o subdevice */
	if (thisboard->has_dio) {
	  s->type = COMEDI_SUBD_DIO;
	  s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
	  s->n_chan = thisboard->dio_nchan;
	  s->maxdata = 1;
	  s->range_table = &range_digital;
	  s->insn_bits = cb_pcidac_dio_insn_bits;
	  s->insn_config = cb_pcidac_dio_insn_config;
	  }
	  else {
	  s->type = COMEDI_SUBD_UNUSED;
	}
  
  printk("attached\n");
#ifdef CBPCIDAC_DEBUG
      printk("devpriv->BADR%d = 0x%lx\n", PLX9030_BADDRINDEX,(unsigned long)devpriv->plx9030_phys_iobase);
      printk("devpriv->BADR%d = 0x%lx\n", MAIN_BADDRINDEX,(unsigned long)devpriv->main_phys_iobase);
      printk("devpriv->BADR%d = 0x%lx\n", DIO_COUNTER_BADDRINDEX, (unsigned long)devpriv->dio_counter_phys_iobase);
      printk(" plx9030 remapped to 0x%lx\n", (unsigned long)devpriv->plx9030_iobase);
      printk(" main remapped to 0x%lx\n", (unsigned long)devpriv->main_iobase);
      printk(" diocounter remapped to 0x%lx\n", (unsigned long)devpriv->dio_counter_iobase);
#endif
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
static int cb_pcidac_detach(comedi_device * dev)
{
#ifdef CBPCIDAC_DEBUG
  if (devpriv) {
    printk("devpriv->plx9030_iobase = 0x%lx\n", (unsigned long)devpriv->plx9030_iobase);
    printk("devpriv->plx9030_phys_iobase = 0x%lx\n", (unsigned long)devpriv->plx9030_phys_iobase);
    printk("devpriv->main_iobase = 0x%lx\n", (unsigned long)devpriv->main_iobase);
    printk("devpriv->main_phys_iobase = 0x%lx\n", (unsigned long)devpriv->main_phys_iobase);
     }
#endif
  printk("comedi%d: cb_pcidac: remove\n", dev->minor);
  
  
  if (devpriv) { 
    if (devpriv->plx9030_iobase)
      iounmap((void *)devpriv->plx9030_iobase);
    if (devpriv->main_iobase)
      iounmap((void *)devpriv->main_iobase);
    if (devpriv->dio_counter_iobase)
      iounmap((void *)devpriv->dio_counter_iobase);       
    if (devpriv->pci_dev) {
      if (devpriv->plx9030_phys_iobase) {
	comedi_pci_disable(devpriv->pci_dev);
      }
      pci_dev_put(devpriv->pci_dev);
    }
  }

  return 0;
}

/*
 * "instructions" read/write data in "one-shot" or "software-triggered"
 * mode.
 */

static int cb_pcidac_ao_winsn(comedi_device * dev, comedi_subdevice * s,
			      comedi_insn * insn, lsampl_t * data)
{
  int i;
  int chan = CR_CHAN(insn->chanspec);

  /* Writing a list of values to an AO channel is probably not
   * very useful, but that's how the interface is defined. */
  for (i = 0; i < insn->n; i++) {
      iowrite16(chan, devpriv->main_iobase + DAC_CHSELECT);
      iowrite16(data[i], devpriv->main_iobase + DAC_DATA); 
  }

  /* return the number of samples read/written */
  return i;
}

/* AO subdevices should have a read insn as well as a write insn.
 * Usually this means copying a value stored in devpriv. */
static int cb_pcidac_ao_rinsn(comedi_device * dev, comedi_subdevice * s,
			      comedi_insn * insn, lsampl_t * data)
{
  int i;
  int chan = CR_CHAN(insn->chanspec);

  for (i = 0; i < insn->n; i++)
    {
    //data[i] = devpriv->ao_readback[chan];
      iowrite16(chan, devpriv->main_iobase + DAC_CHSELECT);
      data[i] = ioread16(devpriv->main_iobase + DAC_READOUT);        
    }
    return i;
    
}
/* DIO devices are slightly special.  Although it is possible to
 * implement the insn_read/insn_write interface, it is much more
 * useful to applications if you implement the insn_bits interface.
 * This allows packed reading/writing of the DIO channels.  The
 * comedi core can convert between insn_bits and insn_read/write */
static int cb_pcidac_dio_insn_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	if (insn->n != 2)
		return -EINVAL;

	/* The insn data is a mask in data[0] and the new data
	 * in data[1], each channel cooresponding to a bit. */
	if (data[0]) {
		s->state &= ~data[0];
		s->state |= data[0] & data[1];
		/* Write out the new digital output lines */
		iowrite8(s->state,devpriv->dio_counter_iobase + DIO_DATA);
	}

	/* on return, data[1] contains the value of the digital
	 * input and output lines. */
	//data[1]=inw(dev->iobase + SKEL_DIO);
	/* or we could just return the software copy of the output values if
	 * it was a purely digital output subdevice */
	//data[1]=s->state;

	return 2;
}

static int cb_pcidac_dio_insn_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int chan = CR_CHAN(insn->chanspec);

	/* The input or output configuration of each digital line is
	 * configured by a special insn_config instruction.  chanspec
	 * contains the channel to be changed, and data[0] contains the
	 * value COMEDI_INPUT or COMEDI_OUTPUT. */
	switch (data[0]) {
	case INSN_CONFIG_DIO_OUTPUT:
		s->io_bits |= 1 << chan;
		break;
	case INSN_CONFIG_DIO_INPUT:
		s->io_bits &= ~(1 << chan);
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] =
			(s->
			io_bits & (1 << chan)) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
		break;
	}
	iowrite8(s->io_bits,devpriv->dio_counter_iobase + DIO_DIRECTION);

	return insn->n;
}

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_PCI_INITCLEANUP(driver_cb_pcidac, cb_pcidac_pci_table);
