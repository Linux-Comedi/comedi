/*
    comedi/drivers/pcmuio.c
    Driver for Winsystems PC-104 based 48-channel and 96-channel DIO boards.

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2006 Calin A. Culianu <calin@ajvar.org>

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
Driver: pcmuio.o
Description: A driver for the PCM-UIO48A and PCM-UIO96A boards from Winsystems.
Devices: (Winsystems) PCM-UIO48A [pcmuio48], (Winsystems) PCM-UIO96A [pcmuio96]
Author: Calin Culianu <calin@ajvar.org>
Updated: Fri, 13 Jan 2006 12:01:01 -0500
Status: works

A driver for the relatively straightforward-to-program PCM-UIO48A and
PCM-UIO96A boards from Winsystems.  These boards use either one or two
(in the 96-DIO version) WS16C48 ASIC HighDensity I/O Chips (HDIO).
This chip is interesting in that each I/O line is individually
programmable for INPUT or OUTPUT (thus comedi_dio_config can be done
on a per-channel basis).  Also, each chip supports edge-triggered
interrupts for the first 24 I/O lines.  Of course, since the
96-channel version of the board has two ASICs, it can detect polarity
changes on up to 48 I/O lines.  Since this is essentially an (non-PnP)
ISA board, I/O Address and IRQ selection are done through jumpers on
the board.  You need to pass that information to this driver as the
first and second comedi_config option, respectively.  Note that the
48-channel version uses 16 bytes of IO memory and the 96-channel
version uses 32-bytes (in case you are worried about conflicts).  The
48-channel board is split into two comedi subdevices, a 32-channel and
a 16-channel subdevice (so that comedi_dio_bitfield works correctly
for all channels).  The 96-channel board is split into 3 32-channel
DIO subdevices.  Note that IRQ support hasn't been added yet.  Let me know
if you need it and I may get around to doing it.

Configuration Options:
  [0] - I/O port base address
  [1] - IRQ (not yet supported)
  [2] - IRQ for second ASIC (pcmuio96 only) (not yet supported)
*/


#include <linux/comedidev.h>

#include <linux/pci.h> /* for PCI devices */

#define MIN(a,b) ( ((a) < (b)) ? (a) : (b) )
#define CHANS_PER_PORT   8
#define PORTS_PER_ASIC   6
#define INTR_PORTS_PER_ASIC   3
#define MAX_CHANS_PER_SUBDEV 32 /* number of channels per comedi subdevice */
#define PORTS_PER_SUBDEV (MAX_CHANS_PER_SUBDEV/CHANS_PER_PORT)
#define CHANS_PER_ASIC (CHANS_PER_PORT*PORTS_PER_ASIC)
#define INTR_CHANS_PER_ASIC 24
#define INTR_PORTS_PER_SUBDEV (INTR_CHANS_PER_ASIC/CHANS_PER_PORT)
#define MAX_DIO_CHANS   (PORTS_PER_ASIC*2*CHANS_PER_PORT)
#define MAX_ASICS       (MAX_DIO_CHANS/CHANS_PER_ASIC)
#define SDEV_NO ((int)(s - dev->subdevices))
#define CALC_N_SUBDEVS(nchans) ((nchans)/32 + (!!((nchans)%32)) /*+ (nchans > INTR_CHANS_PER_ASIC ? 2 : 1)*/)
/* IO Memory sizes */
#define ASIC_IOSIZE 16
#define PCMUIO48_IOSIZE ASIC_IOSIZE
#define PCMUIO96_IOSIZE (ASIC_IOSIZE*2)

/* Some offsets - these are all in the 16byte IO memory offset from
   the base address.  Note that there is a paging scheme to swap out
   offsets 0x8-0xA using the PAGELOCK register.  See the table below.

  Register(s)       Pages        R/W?        Description
  --------------------------------------------------------------
  REG_PORTx         All          R/W         Read/Write/Configure IO
  REG_INT_PENDING   All          ReadOnly    Quickly see which INT_IDx has int.
  REG_PAGELOCK      All          WriteOnly   Select a page 
  REG_POLx          Pg. 1 only   WriteOnly   Select edge-detection polarity
  REG_ENABx         Pg. 2 only   WriteOnly   Enable/Disable edge-detect. int.
  REG_INT_IDx       Pg. 3 only   R/W         See which ports/bits have ints.
 */
#define REG_PORT0 0x0
#define REG_PORT1 0x1
#define REG_PORT2 0x2
#define REG_PORT3 0x3
#define REG_PORT4 0x4
#define REG_PORT5 0x5
#define REG_INT_PENDING 0x6
#define REG_PAGELOCK 0x7 /* page selector register, upper 2 bits select a page
                            and bits 0-5 are used to 'lock down' a particular
                            port above to make it readonly.  */
#define REG_POL0 0x8 
#define REG_POL1 0x9
#define REG_POL2 0xA
#define REG_ENAB0 0x8 
#define REG_ENAB1 0x9
#define REG_ENAB2 0xA
#define REG_INT_ID0 0x8
#define REG_INT_ID1 0x9
#define REG_INT_ID2 0xA

#define NUM_PAGED_REGS 3
#define NUM_PAGES 4
#define FIRST_PAGED_REG 0x8
#define REG_PAGE_BITOFFSET 6
#define REG_LOCK_BITOFFSET 0
#define REG_PAGE_MASK (~((0x1<<REG_PAGE_BITOFFSET)-1))
#define REG_LOCK_MASK ~(REG_PAGE_MASK)
#define PAGE_POL 1
#define PAGE_ENAB 2
#define PAGE_INT_ID 3

/*
 * Board descriptions for two imaginary boards.  Describing the
 * boards in this way is optional, and completely driver-dependent.
 * Some drivers use arrays such as this, other do not.
 */
typedef struct pcmuio_board_struct
{
	char * const name;
	const int num_asics;
	const int num_channels_per_port;
	const int num_ports;
} pcmuio_board;

static pcmuio_board pcmuio_boards[] = 
{
	{
		name:                   "pcmuio48",
		num_asics:              1,
		num_ports:              6,
	},
	{
		name:		"pcmuio96",
		num_asics:              2,
		num_ports:              12,
	},
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((pcmuio_board *)dev->board_ptr)

/* this structure is for data unique to this subdevice.  */
typedef struct 
{
    /* mapping of halfwords (bytes) in port/chanarray to iobase */
    unsigned long iobases[PORTS_PER_SUBDEV];
  
    /* The below is only used for intr subdevices 
    struct {
      int asic;
      int active;
      int stop_count;
      spinlock_t spin_lock;
    } intr;
    */
} pcmuio_subdev_private;

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct
{  
  struct 
  {
    unsigned char pagelock; /* current page and lock*/
    unsigned char pol [NUM_PAGED_REGS]; /* shadow of POLx registers */
    unsigned char enab[NUM_PAGED_REGS]; /* shadow of ENABx registers */
    int num;
    unsigned long iobase;
    int irq;
  } asics[MAX_ASICS];  
  pcmuio_subdev_private *sprivs;
} pcmuio_private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((pcmuio_private *)dev->private)
#define subpriv ((pcmuio_subdev_private *)s->private)
/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int pcmuio_attach(comedi_device *dev, comedi_devconfig *it);
static int pcmuio_detach(comedi_device *dev);

static comedi_driver driver = 
{
	driver_name:	"pcmuio",
	module:		THIS_MODULE,
	attach:		pcmuio_attach,
	detach:		pcmuio_detach,
/* It is not necessary to implement the following members if you are
 * writing a driver for a ISA PnP or PCI card */
	/* Most drivers will support multiple types of boards by
	 * having an array of board structures.  These were defined
	 * in pcmuio_boards[] above.  Note that the element 'name'
	 * was first in the structure -- Comedi uses this fact to
	 * extract the name of the board without knowing any details
	 * about the structure except for its length.
	 * When a device is attached (by comedi_config), the name
	 * of the device is given to Comedi, and Comedi tries to
	 * match it by going through the list of board names.  If
	 * there is a match, the address of the pointer is put
	 * into dev->board_ptr and driver->attach() is called.
	 *
	 * Note that these are not necessary if you can determine
	 * the type of board in software.  ISA PnP, PCI, and PCMCIA
	 * devices are such boards.
	 */
	board_name:	pcmuio_boards,
	offset:		sizeof(pcmuio_board),
	num_names:	sizeof(pcmuio_boards) / sizeof(pcmuio_board),
};

static int pcmuio_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
                                comedi_insn *insn,lsampl_t *data);
static int pcmuio_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
                                  comedi_insn *insn,lsampl_t *data);
/*
static int pcmuio_intr_insn_bits(comedi_device *dev,comedi_subdevice *s,
                                comedi_insn *insn,lsampl_t *data);
static int pcmuio_intr_insn_config(comedi_device *dev,comedi_subdevice *s,
                                   comedi_insn *insn,lsampl_t *data);
static int pcmuio_intr_cmdtest(comedi_device *dev, comedi_subdevice *s,
                               comedi_cmd *cmd);
static int pcmuio_intr_cmd(comedi_device *dev, comedi_subdevice *s,
                           comedi_cmd *cmd);
*/

/* some helper functions to deal with specifics of this device's registers */
static void init_asics(comedi_device *dev); /* sets up/clears ASIC chips to defaults */
static void switch_page(comedi_device *dev, int asic, int page);
static void lock_port(comedi_device *dev, int asic, int port);
static void unlock_port(comedi_device *dev, int asic, int port);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int pcmuio_attach(comedi_device *dev, comedi_devconfig *it)
{
	comedi_subdevice *s;
    int sdev_no, chans_left, n_subdevs, iobase, irq[MAX_ASICS], port, asic;

    iobase = it->options[0];
    irq[0] = it->options[1];
    irq[1] = it->options[2];
    

printk("comedi%d: %s: io: %x ", dev->minor, driver.driver_name, iobase);
	
    dev->iobase = iobase;
    
    if ( !iobase || !request_region(iobase, 
                                    thisboard->num_asics*ASIC_IOSIZE, 
                                    driver.driver_name) ) {
      printk("I/O port conflict\n");
      return -EIO;
    }

    if (irq[0]) {
      printk("irq: %x ", irq[0]);
      if (irq[1] && thisboard->num_asics == 2) 
        printk("second ASIC irq: %x ", irq[1]);
    } else {
      printk("(IRQ mode disabled) ");
    }
    

    if (irq[0]) {
      /* TODO: irq request/handling here.. */
    }
    /* dev->irq = irq[0]; */

/*
 * Initialize dev->board_name.  Note that we can use the "thisboard"
 * macro now, since we just initialized it in the last line.
 */
	dev->board_name = thisboard->name;

/*
 * Allocate the private structure area.  alloc_private() is a
 * convenient macro defined in comedidev.h.
 */
	if (alloc_private(dev,sizeof(pcmuio_private)) < 0) {
      printk("cannot allocate private data structure\n");
      return -ENOMEM;
    }
    for (asic = 0; asic < MAX_ASICS; ++asic) {
      devpriv->asics[asic].num = asic;
      devpriv->asics[asic].iobase = dev->iobase + asic*ASIC_IOSIZE;
      devpriv->asics[asic].irq = irq[asic];
    }      
    
    chans_left = CHANS_PER_ASIC * thisboard->num_asics;    
    n_subdevs = CALC_N_SUBDEVS(chans_left);
    devpriv->sprivs = (pcmuio_subdev_private *)kmalloc(sizeof(pcmuio_subdev_private) * n_subdevs, GFP_KERNEL);
    if (!devpriv->sprivs) {
        printk("cannot allocate subdevice private data structures\n");
        return -ENOMEM;
    }
    memset(devpriv->sprivs, 0, sizeof(pcmuio_subdev_private) * n_subdevs);
      /*
       * Allocate the subdevice structures.  alloc_subdevice() is a
       * convenient macro defined in comedidev.h.
       *
       * Allocate 2 subdevs (32 + 16 DIO lines) or 3 32 DIO subdevs for the 
       * 96-channel version of the board.
       */
	if ( alloc_subdevices(dev, n_subdevs ) < 0 ) {
        printk("cannot allocate subdevice data structures\n");
        return -ENOMEM;
    }
    port = 0;
    asic = 0;
    for (sdev_no = 0; sdev_no < (int)dev->n_subdevices; ++sdev_no)
    {      
      int byte_no;

      s = dev->subdevices + sdev_no;
      s->private = devpriv->sprivs + sdev_no;
      s->maxdata = 1;
      s->range_table = &range_digital;
      
/*       if (!chans_left) { /\* INTERRUPT subdevice(s) *\/ */
        
/*         s->subdev_flags = SDF_READABLE|SDF_PACKED; */
/*         s->type = COMEDI_SUBD_DI; */
/*         s->do_cmdtest = pcmuio_intr_cmdtest; */
/*         s->do_cmd = pcmuio_intr_cmd; */
/*         s->insn_bits = pcmuio_intr_insn_bits; */
/*         s->insn_config = pcmuio_intr_insn_config; */
/*         s->n_chan = INTR_PORTS_PER_ASIC * CHANS_PER_PORT; */
        
/*         /\* save the ioport address for each 'port' of 8 channels in the  */
/*            interrupt subdevice *\/ */
/*         for (byte_no = 0; byte_no < INTR_PORTS_PER_ASIC; ++byte_no) */
/*           subpriv->iobases[byte_no] = devpriv->asics[asic].iobase + port++; */
        
/*         subpriv->intr.asic = asic; */
/*         asic++; */
/*         port = 0; */

/*       } else { */

      s->subdev_flags = SDF_READABLE|SDF_WRITABLE|SDF_PACKED;
      s->type = COMEDI_SUBD_DIO;
      s->insn_bits = pcmuio_dio_insn_bits;
      s->insn_config = pcmuio_dio_insn_config;
      s->n_chan = MIN(chans_left, MAX_CHANS_PER_SUBDEV);
      /* save the ioport address for each 'port' of 8 channels in the 
         subdevice */         
      for (byte_no = 0; byte_no < PORTS_PER_SUBDEV; ++byte_no) {
          if (port >= PORTS_PER_ASIC) {
            port = 0;
            asic++;
          }
          subpriv->iobases[byte_no] = devpriv->asics[asic].iobase + port++;
      }
      chans_left -= s->n_chan;

      if (!chans_left) {
        asic = 0; /* reset the asic to our first asic, to do intr subdevs */
        port = 0;
      }
      
        /*}*/
    }

	init_asics(dev); /* clear out all the registers, basically */

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
static int pcmuio_detach(comedi_device *dev)
{
    printk("comedi%d: %s: remove\n", dev->minor, driver.driver_name);
    if (dev->iobase)
      release_region(dev->iobase, ASIC_IOSIZE * thisboard->num_asics);
    if (dev->irq)
      free_irq(dev->irq, dev);
    if (devpriv && devpriv->sprivs)
      kfree(devpriv->sprivs);
    return 0;
}


/* DIO devices are slightly special.  Although it is possible to
 * implement the insn_read/insn_write interface, it is much more
 * useful to applications if you implement the insn_bits interface.
 * This allows packed reading/writing of the DIO channels.  The
 * comedi core can convert between insn_bits and insn_read/write */
static int pcmuio_dio_insn_bits(comedi_device *dev, comedi_subdevice *s,
                                comedi_insn *insn, lsampl_t *data)
{
    if (insn->n != 2) return -EINVAL;

     /* NOTE:
        reading a 0 means this channel was high
        reading a 1 means this channel was low 

        therefore s->state is always inverted.  it also contains
        forced-zeros for all channels that were configured as INPUT 
        (s->iobit == 0 for a bit means it is INPUT) */

	/* The insn data is a mask in data[0] and the new data
	 * in data[1], each channel cooresponding to a bit. */
    int byte_no;
     s->state &= ~(data[0] & s->io_bits);  /* clear bits for  
                                              write_mask&output_bits.  */
    s->state |= (data[0] & s->io_bits & data[1]); /* set the bits for write_mask&output_bits with the data. */
    data[1] = 0; /* clear what we will return */
    for (byte_no = 0; byte_no < s->n_chan/CHANS_PER_PORT; byte_no++) {
        int ioaddr = subpriv->iobases[byte_no];
        /* Write out the new digital output lines */
        outb(s->state >> byte_no*8, ioaddr);
         /* read back the digital input lines.. note they come in inverted! */ 
        data[1] |= (unsigned int)~inb(ioaddr) << byte_no*8; 
    }

	return 2;
}

static int pcmuio_dio_insn_config(comedi_device *dev, comedi_subdevice *s,
                                  comedi_insn *insn, lsampl_t *data)
{
    int chan = CR_CHAN(insn->chanspec), byte_no = chan/8, ioaddr;
    unsigned char byte;

    /* Compute ioaddr for this channel */
    ioaddr = subpriv->iobases[byte_no];

     /* NOTE:
        writing a 0 an IO channel's bit sets the channel to INPUT 
        and pulls the line high as well
        
        writing a 1 to an IO channel's  bit means set this channel to OUTPUT 
        and it pulls the line low as well
        
        note: s->state is the last real bitpattern written
        to the device! */

	/* The input or output configuration of each digital line is
	 * configured by a special insn_config instruction.  chanspec
	 * contains the channel to be changed, and data[0] contains the
	 * value COMEDI_INPUT or COMEDI_OUTPUT. */
	switch(data[0])
	{
	case INSN_CONFIG_DIO_OUTPUT:
      /* save to shadow registers */
		s->io_bits |= 1<<chan;
        s->state |= 1<<chan; /* reflect the change in the state. */
		break;
	case INSN_CONFIG_DIO_INPUT:
      /* save to shadow registers */
		s->io_bits &= ~(1<<chan);
        s->state &= ~(1<<chan); /* reflect the change in the state. */
		break;
	case INSN_CONFIG_DIO_QUERY:
      /* retreive from shadow registers */
		data[1] = (s->io_bits & (1 << chan)) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
		break;
	}    
    /* now write out the saved shadow register to reconfigure */
    byte = (s->state >> byte_no*8) & 0xff;

	outb(byte, ioaddr);


	return insn->n;    
}

static void init_asics(comedi_device *dev) /* sets up an 
                                              ASIC chip to defaults */
{
  int asic;
  
  for (asic = 0; asic < thisboard->num_asics; ++asic)
  {
    int port, page;
    int baseaddr = dev->iobase + asic*ASIC_IOSIZE;

    /* first, clear all the DIO port bits */
    for (port = 0; port < PORTS_PER_ASIC; ++port) 
      outb(0, baseaddr + REG_PORT0 + port);

    /* Next, clear all the paged registers for each page */
    for (page = 1; page < NUM_PAGES; ++page)
    {
      int reg;
      /* now clear all the paged registers*/
      switch_page(dev, asic, page);
      for (reg = FIRST_PAGED_REG; reg < FIRST_PAGED_REG+NUM_PAGED_REGS; ++reg)
        outb(0, baseaddr + reg);
    }
    switch_page(dev, asic,  0); /* switch back to default page 0 */
  }
}


static void switch_page(comedi_device *dev, int asic, int page)
{
  if (asic < 0 || asic >= thisboard->num_asics) return; /* paranoia */
  if (page < 0 || page >= NUM_PAGES) return; /* more paranoia */
  devpriv->asics[asic].pagelock &= ~REG_PAGE_MASK;
  devpriv->asics[asic].pagelock |= page<<REG_PAGE_BITOFFSET;

  /* now write out the shadow register */
  outb(devpriv->asics[asic].pagelock, 
       dev->iobase + ASIC_IOSIZE*asic + REG_PAGELOCK);
}

static void lock_port(comedi_device *dev, int asic, int port)
{
  if (asic < 0 || asic >= thisboard->num_asics) return; /* paranoia */
  if (port < 0 || port >= PORTS_PER_ASIC) return; /* more paranoia */
  devpriv->asics[asic].pagelock |= 0x1<<port;  
  /* now write out the shadow register */
  outb(devpriv->asics[asic].pagelock, dev->iobase + ASIC_IOSIZE*asic + REG_PAGELOCK);
  return;
  (void)lock_port(dev, asic, port); /* not reached, suppress compiler warnings*/
}

static void unlock_port(comedi_device *dev, int asic, int port)
{
  if (asic < 0 || asic >= thisboard->num_asics) return; /* paranoia */
  if (port < 0 || port >= PORTS_PER_ASIC) return; /* more paranoia */
  devpriv->asics[asic].pagelock &= ~(0x1<<port) | REG_LOCK_MASK;  
  /* now write out the shadow register */
  outb(devpriv->asics[asic].pagelock, dev->iobase + ASIC_IOSIZE*asic + REG_PAGELOCK);
  (void)unlock_port(dev, asic, port); /* not reached, suppress compiler warnings*/
}

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_INITCLEANUP(driver);


