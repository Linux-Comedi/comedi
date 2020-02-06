/*
    comedi/drivers/amplc_dio200.c
    Driver for various Amplicon 200 series DIO boards.
    (Support for other boards in Amplicon 200 series may be added at
    a later date, e.g. PCI215.)

    Copyright (C) 2005-2012 MEV Ltd. <http://www.mev.co.uk/>

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
Driver: amplc_dio200
Description: Amplicon 200 Series Digital I/O
Author: Ian Abbott <abbotti@mev.co.uk>
Devices: [Amplicon] PC212E (pc212e), PC214E (pc214e), PC215E (pc215e),
  PCI215 (pci215 or amplc_dio200), PCIe215 (pcie215 or amplc_dio200),
  PC218E (pc218e), PCIe236 (pcie236 or amplc_dio200), PC272E (pc272e),
  PCI272 (pci272 or amplc_dio200), PCIe296 (pcie296 or amplc_dio200)
Updated: Wed, 03 Oct 2012 17:54:05 +0100
Status: works

Configuration options - PC212E, PC214E, PC215E, PC218E, PC272E:
  [0] - I/O port base address
  [1] - IRQ (optional, but commands won't work without it)

Configuration options - PCI215, PCIe215, PCIe236, PCI272, PCIe296:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
  If bus/slot is not specified, the first available PCI device will
  be used.

Passing a zero for an option is the same as leaving it unspecified.

SUBDEVICES

                    PC212E         PC214E      PC215E/PCI215
                 -------------  -------------  -------------
  Subdevices           6              4              5
   0                 PPI-X          PPI-X          PPI-X
   1                 CTR-Y1         PPI-Y          PPI-Y
   2                 CTR-Y2         CTR-Z1*        CTR-Z1
   3                 CTR-Z1       INTERRUPT*       CTR-Z2
   4                 CTR-Z2                      INTERRUPT
   5               INTERRUPT

                    PCIe215        PC218E         PCIe236
                 -------------  -------------  -------------
  Subdevices           8              7              8
   0                 PPI-X          CTR-X1         PPI-X
   1                 UNUSED         CTR-X2         UNUSED
   2                 PPI-Y          CTR-Y1         UNUSED
   3                 UNUSED         CTR-Y2         UNUSED
   4                 CTR-Z1         CTR-Z1         CTR-Z1
   5                 CTR-Z2         CTR-Z2         CTR-Z2
   6                 TIMER        INTERRUPT        TIMER
   7               INTERRUPT                     INTERRUPT

                 PC272E/PCI272     PCIe296
                 -------------  -------------
  Subdevices           4              8
   0                 PPI-X          PPI-X1
   1                 PPI-Y          PPI-X2
   2                 PPI-Z          PPI-Y1
   3               INTERRUPT        PPI-Y2
   4                                CTR-Z1
   5                                CTR-Z2
   6                                TIMER
   7                              INTERRUPT

Each PPI is a 8255 chip providing 24 DIO channels.  The DIO channels
are configurable as inputs or outputs in four groups:

  Port A  - channels  0 to  7
  Port B  - channels  8 to 15
  Port CL - channels 16 to 19
  Port CH - channels 20 to 23

Only mode 0 of the 8255 chips is supported.

Each CTR is a 8254 chip providing 3 16-bit counter channels.  Each
channel is configured individually with INSN_CONFIG instructions.  The
specific type of configuration instruction is specified in data[0].
Some configuration instructions expect an additional parameter in
data[1]; others return a value in data[1].  The following configuration
instructions are supported:

  INSN_CONFIG_SET_COUNTER_MODE.  Sets the counter channel's mode and
    BCD/binary setting specified in data[1].

  INSN_CONFIG_8254_READ_STATUS.  Reads the status register value for the
    counter channel into data[1].

  INSN_CONFIG_SET_CLOCK_SRC.  Sets the counter channel's clock source as
    specified in data[1] (this is a hardware-specific value).  Not
    supported on PC214E.  For the other boards, valid clock sources are
    0 to 7 as follows:

      0.  CLK n, the counter channel's dedicated CLK input from the SK1
        connector.  (N.B. for other values, the counter channel's CLKn
        pin on the SK1 connector is an output!)
      1.  Internal 10 MHz clock.
      2.  Internal 1 MHz clock.
      3.  Internal 100 kHz clock.
      4.  Internal 10 kHz clock.
      5.  Internal 1 kHz clock.
      6.  OUT n-1, the output of counter channel n-1 (see note 1 below).
      7.  Ext Clock, the counter chip's dedicated Ext Clock input from
        the SK1 connector.  This pin is shared by all three counter
        channels on the chip.

  INSN_CONFIG_GET_CLOCK_SRC.  Returns the counter channel's current
    clock source in data[1].  For internal clock sources, data[2] is set
    to the period in ns.

  INSN_CONFIG_SET_GATE_SRC.  Sets the counter channel's gate source as
    specified in data[2] (this is a hardware-specific value).  Not
    supported on PC214E.  For the other boards, valid gate sources are 0
    to 7 as follows:

      0.  VCC (internal +5V d.c.), i.e. gate permanently enabled.
      1.  GND (internal 0V d.c.), i.e. gate permanently disabled.
      2.  GAT n, the counter channel's dedicated GAT input from the SK1
        connector.  (N.B. for other values, the counter channel's GATn
        pin on the SK1 connector is an output!)
      3.  /OUT n-2, the inverted output of counter channel n-2 (see note
        2 below).
      4.  Reserved.
      5.  Reserved.
      6.  Reserved.
      7.  Reserved.

  INSN_CONFIG_GET_GATE_SRC.  Returns the counter channel's current gate
    source in data[2].

Clock and gate interconnection notes:

  1.  Clock source OUT n-1 is the output of the preceding channel on the
  same counter subdevice if n > 0, or the output of channel 2 on the
  preceding counter subdevice (see note 3) if n = 0.

  2.  Gate source /OUT n-2 is the inverted output of channel 0 on the
  same counter subdevice if n = 2, or the inverted output of channel n+1
  on the preceding counter subdevice (see note 3) if n < 2.

  3.  The counter subdevices are connected in a ring, so the highest
  counter subdevice precedes the lowest.

The 'TIMER' subdevice is a free-running 32-bit timer subdevice.

The 'INTERRUPT' subdevice pretends to be a digital input subdevice.  The
digital inputs come from the interrupt status register.  The number of
channels matches the number of interrupt sources.  The PC214E does not
have an interrupt status register; see notes on 'INTERRUPT SOURCES'
below.

INTERRUPT SOURCES

                    PC212E         PC214E      PC215E/PCI215
                 -------------  -------------  -------------
  Sources              6              1              6
   0               PPI-X-C0       JUMPER-J5      PPI-X-C0
   1               PPI-X-C3                      PPI-X-C3
   2              CTR-Y1-OUT1                    PPI-Y-C0
   3              CTR-Y2-OUT1                    PPI-Y-C3
   4              CTR-Z1-OUT1                   CTR-Z1-OUT1
   5              CTR-Z2-OUT1                   CTR-Z2-OUT1

                    PCIe215        PC218E         PCIe236
                 -------------  -------------  -------------
  Sources              6              6              6
   0               PPI-X-C0      CTR-X1-OUT1     PPI-X-C0
   1               PPI-X-C3      CTR-X2-OUT1     PPI-X-C3
   2               PPI-Y-C0      CTR-Y1-OUT1      unused
   3               PPI-Y-C3      CTR-Y2-OUT1      unused
   4              CTR-Z1-OUT1    CTR-Z1-OUT1    CTR-Z1-OUT1
   5              CTR-Z2-OUT1    CTR-Z2-OUT1    CTR-Z2-OUT1

                 PC272E/PCI272     PCIe296
                 -------------  -------------
  Sources              6              6
   0               PPI-X-C0       PPI-X1-C0
   1               PPI-X-C3       PPI-X1-C3
   2               PPI-Y-C0       PPI-Y1-C0
   3               PPI-Y-C3       PPI-Y1-C3
   4               PPI-Z-C0      CTR-Z1-OUT1
   5               PPI-Z-C3      CTR-Z2-OUT1

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

The PC214E does not have an interrupt source enable register or an
interrupt status register; its 'INTERRUPT' subdevice has a single
channel and its interrupt source is selected by the position of jumper
J5.

COMMANDS

The driver supports a read streaming acquisition command on the
'INTERRUPT' subdevice.  The channel list selects the interrupt sources
to be enabled.  All channels will be sampled together (convert_src ==
TRIG_NOW).  The scan begins a short time after the hardware interrupt
occurs, subject to interrupt latencies (scan_begin_src == TRIG_EXT,
scan_begin_arg == 0).  The value read from the interrupt status register
is packed into a sampl_t value, one bit per requested channel, in the
order they appear in the channel list.
*/

#include <linux/comedidev.h>

#include "comedi_pci.h"

#include "8255.h"
#include "8253.h"

#define DIO200_DRIVER_NAME	"amplc_dio200"

/* PCI IDs */
/* #define PCI_VENDOR_ID_AMPLICON 0x14dc */
#define PCI_DEVICE_ID_AMPLICON_PCI272 0x000a
#define PCI_DEVICE_ID_AMPLICON_PCI215 0x000b
#define PCI_DEVICE_ID_AMPLICON_PCIE236 0x0011
#define PCI_DEVICE_ID_AMPLICON_PCIE215 0x0012
#define PCI_DEVICE_ID_AMPLICON_PCIE296 0x0014
#define PCI_DEVICE_ID_INVALID 0xffff

/* 8255 control register bits */
#define CR_C_LO_IO	0x01
#define CR_B_IO		0x02
#define CR_B_MODE	0x04
#define CR_C_HI_IO	0x08
#define CR_A_IO		0x10
#define CR_A_MODE(a)	((a)<<5)
#define CR_CW		0x80

/* 200 series registers */
#define DIO200_IO_SIZE		0x20
#define DIO200_PCIE_IO_SIZE	0x4000
#define DIO200_XCLK_SCE		0x18	/* Group X clock selection register */
#define DIO200_YCLK_SCE		0x19	/* Group Y clock selection register */
#define DIO200_ZCLK_SCE		0x1a	/* Group Z clock selection register */
#define DIO200_XGAT_SCE		0x1b	/* Group X gate selection register */
#define DIO200_YGAT_SCE		0x1c	/* Group Y gate selection register */
#define DIO200_ZGAT_SCE		0x1d	/* Group Z gate selection register */
#define DIO200_INT_SCE		0x1e	/* Interrupt enable/status register */
/* Extra registers for new PCIe boards */
#define DIO200_ENHANCE		0x20	/* 1 to enable enhanced features */
#define DIO200_VERSION		0x24	/* Hardware version */
#define DIO200_TS_CONFIG	0x600	/* Timestamp timer config register */
#define DIO200_TS_COUNT		0x602	/* Timestamp timer count register */

/*
 * Macros for constructing value for DIO_200_?CLK_SCE and
 * DIO_200_?GAT_SCE registers:
 *
 * 'which' is: 0 for CTR-X1, CTR-Y1, CTR-Z1; 1 for CTR-X2, CTR-Y2 or CTR-Z2.
 * 'chan' is the channel: 0, 1 or 2.
 * 'source' is the signal source: 0 to 7, or 0 to 31 for "enhanced" boards.
 */
#define CLK_SCE(which, chan, source) (((which) << 5) | ((chan) << 3) | \
		(((source) & 030) << 3) | ((source) & 007))
#define GAT_SCE(which, chan, source) (((which) << 5) | ((chan) << 3) | \
		(((source) & 030) << 3) | ((source) & 007))

/*
 * Timestamp timer configuration register (for new PCIe boards).
 */
#define TS_CONFIG_RESET		0x100	/* Reset counter to zero. */
#define TS_CONFIG_CLK_SRC_MASK	0x0FF	/* Clock source. */
#define TS_CONFIG_MAX_CLK_SRC	2	/* Maximum clock source value. */

/*
 * Periods of the timestamp timer clock sources in nanoseconds.
 */
static const unsigned ts_clock_period[TS_CONFIG_MAX_CLK_SRC + 1] = {
	1,			/* 1 nanosecond (but with 20 ns granularity). */
	1000,			/* 1 microsecond. */
	1000000,		/* 1 millisecond. */
};

/*
 * Periods of the internal counter clock sources in nanoseconds.
 */
static const unsigned clock_period[32] = {
	0,			/* dedicated clock input/output pin */
	100,			/* 10 MHz */
	1000,			/* 1 MHz */
	10000,			/* 100 kHz */
	100000,			/* 10 kHz */
	1000000,		/* 1 kHz */
	0,			/* OUT N-1 */
	0,			/* group clock input pin */
	0,			/* HIGH (VCC) (enhanced) */
	0,			/* LOW (GND) (enhanced) */
	0,			/* pattern present (enhanced) */
	50,			/* 20 MHz (enhanced) */
	/* remaining clock sources reserved (enhanced) */
};

/*
 * Register region.
 */
enum dio200_regtype { no_regtype, io_regtype, mmio_regtype };
struct dio200_region {
	union {
		unsigned long iobase;		/* I/O base address */
		unsigned char __iomem *membase;	/* Mapped MMIO base address */
	} u;
	unsigned char regtype;
	unsigned char regshift;
};

/*
 * Board descriptions.
 */

enum dio200_bustype { isa_bustype, pci_bustype };

enum dio200_model {
	pc212e_model,
	pc214e_model,
	pc215e_model, pci215_model, pcie215_model,
	pc218e_model,
	pcie236_model,
	pc272e_model, pci272_model,
	pcie296_model,
	anypci_model
};

enum dio200_layout {
	pc212_layout,
	pc214_layout,
	pc215_layout,
	pc218_layout,
	pc272_layout,
#ifdef COMEDI_CONFIG_PCI
	pcie215_layout,
	pcie236_layout,
	pcie296_layout,
#endif
	num_layouts
};

typedef struct dio200_board_struct {
	const char *name;
	unsigned short devid;
	enum dio200_bustype bustype;
	enum dio200_model model;
	enum dio200_layout layout;
	unsigned char mainbar;
	unsigned char mainshift;
	unsigned int mainsize;
} dio200_board;

static const dio200_board dio200_boards[] = {
	{
	      name:	"pc212e",
	      bustype:	isa_bustype,
	      model:	pc212e_model,
	      layout:	pc212_layout,
	      mainsize:	DIO200_IO_SIZE,
		},
	{
	      name:	"pc214e",
	      bustype:	isa_bustype,
	      model:	pc214e_model,
	      layout:	pc214_layout,
	      mainsize:	DIO200_IO_SIZE,
		},
	{
	      name:	"pc215e",
	      bustype:	isa_bustype,
	      model:	pc215e_model,
	      layout:	pc215_layout,
	      mainsize:	DIO200_IO_SIZE,
		},
#ifdef COMEDI_CONFIG_PCI
	{
	      name:	"pci215",
	      devid:	PCI_DEVICE_ID_AMPLICON_PCI215,
	      bustype:	pci_bustype,
	      model:	pci215_model,
	      layout:	pc215_layout,
	      mainsize:	DIO200_IO_SIZE,
	      mainbar:	2,
		},
#endif
#ifdef COMEDI_CONFIG_PCI
	{
	      name:	"pcie215",
	      devid:	PCI_DEVICE_ID_AMPLICON_PCIE215,
	      bustype:	pci_bustype,
	      model:	pcie215_model,
	      layout:	pcie215_layout,
	      mainsize:	DIO200_PCIE_IO_SIZE,
	      mainbar:	1,
	      mainshift: 3,
		},
#endif
	{
	      name:	"pc218e",
	      bustype:	isa_bustype,
	      model:	pc218e_model,
	      layout:	pc218_layout,
	      mainsize:	DIO200_IO_SIZE,
		},
#ifdef COMEDI_CONFIG_PCI
	{
	      name:	"pcie236",
	      devid:	PCI_DEVICE_ID_AMPLICON_PCIE236,
	      bustype:	pci_bustype,
	      model:	pcie236_model,
	      layout:	pcie236_layout,
	      mainsize:	DIO200_PCIE_IO_SIZE,
	      mainbar:	1,
	      mainshift: 3,
		},
#endif
	{
	      name:	"pc272e",
	      bustype:	isa_bustype,
	      model:	pc272e_model,
	      layout:	pc272_layout,
	      mainsize:	DIO200_IO_SIZE,
		},
#ifdef COMEDI_CONFIG_PCI
	{
	      name:	"pci272",
	      devid:	PCI_DEVICE_ID_AMPLICON_PCI272,
	      bustype:	pci_bustype,
	      model:	pci272_model,
	      layout:	pc272_layout,
	      mainsize:	DIO200_IO_SIZE,
	      mainbar:	2,
		},
#endif
#ifdef COMEDI_CONFIG_PCI
	{
	      name:	"pcie296",
	      devid:	PCI_DEVICE_ID_AMPLICON_PCIE296,
	      bustype:	pci_bustype,
	      model:	pcie296_model,
	      layout:	pcie296_layout,
	      mainsize:	DIO200_PCIE_IO_SIZE,
	      mainbar:	1,
	      mainshift: 3,
		},
#endif
#ifdef COMEDI_CONFIG_PCI
	{
	      name:	DIO200_DRIVER_NAME,
	      devid:	PCI_DEVICE_ID_INVALID,
	      bustype:	pci_bustype,
	      model:	anypci_model,	/* wildcard */
		},
#endif
};

/*
 * Layout descriptions - some ISA and PCI board descriptions share the same
 * layout.
 */

enum dio200_sdtype { sd_none, sd_intr, sd_8255, sd_8254, sd_timer };

#define DIO200_MAX_SUBDEVS	8
#define DIO200_MAX_ISNS		6

typedef struct dio200_layout_struct {
	unsigned short n_subdevs;	/* number of subdevices */
	unsigned char sdtype[DIO200_MAX_SUBDEVS];	/* enum dio200_sdtype */
	unsigned char sdinfo[DIO200_MAX_SUBDEVS];	/* depends on sdtype */
	char has_int_sce;	/* has interrupt enable/status register */
	char has_clk_gat_sce;	/* has clock/gate selection registers */
	char has_enhancements;	/* has enhanced features */
} dio200_layout;

static const dio200_layout dio200_layouts[] = {
	[pc212_layout] = {
	      n_subdevs:6,
	      sdtype:	{sd_8255, sd_8254, sd_8254, sd_8254,
					sd_8254,
				sd_intr},
	      sdinfo:	{0x00, 0x08, 0x0C, 0x10, 0x14,
				0x3F},
	      has_int_sce:1,
	      has_clk_gat_sce:1,
	      has_enhancements:0,
		},
	[pc214_layout] = {
	      n_subdevs:4,
	      sdtype:	{sd_8255, sd_8255, sd_8254,
				sd_intr},
	      sdinfo:	{0x00, 0x08, 0x10, 0x01},
	      has_int_sce:0,
	      has_clk_gat_sce:0,
	      has_enhancements:0,
		},
	[pc215_layout] = {
	      n_subdevs:5,
	      sdtype:	{sd_8255, sd_8255, sd_8254,
					sd_8254,
				sd_intr},
	      sdinfo:	{0x00, 0x08, 0x10, 0x14, 0x3F},
	      has_int_sce:1,
	      has_clk_gat_sce:1,
	      has_enhancements:0,
		},
	[pc218_layout] = {
	      n_subdevs:7,
	      sdtype:	{sd_8254, sd_8254, sd_8255, sd_8254,
					sd_8254,
				sd_intr},
	      sdinfo:	{0x00, 0x04, 0x08, 0x0C, 0x10,
					0x14,
				0x3F},
	      has_int_sce:1,
	      has_clk_gat_sce:1,
	      has_enhancements:0,
		},
	[pc272_layout] = {
	      n_subdevs:4,
	      sdtype:	{sd_8255, sd_8255, sd_8255,
				sd_intr},
	      sdinfo:	{0x00, 0x08, 0x10, 0x3F},
	      has_int_sce:1,
	      has_clk_gat_sce:0,
	      has_enhancements:0,
		},
#ifdef COMEDI_CONFIG_PCI
	[pcie215_layout] = {
	      n_subdevs:8,
	      sdtype:	{sd_8255, sd_none, sd_8255, sd_none, sd_8254, sd_8254,
				sd_timer, sd_intr},
	      sdinfo:	{0x00, 0x00, 0x08, 0x00, 0x10, 0x14, 0x00, 0x3F},
	      has_int_sce:1,
	      has_clk_gat_sce:1,
	      has_enhancements:1,
		},
	[pcie236_layout] = {
	      n_subdevs:8,
	      sdtype:	{sd_8255, sd_none, sd_none, sd_none, sd_8254, sd_8254,
				sd_timer, sd_intr},
	      sdinfo:	{0x00, 0x00, 0x00, 0x00, 0x10, 0x14, 0x00, 0x3F},
	      has_int_sce:1,
	      has_clk_gat_sce:1,
	      has_enhancements:1,
		},
	[pcie296_layout] = {
	      n_subdevs:8,
	      sdtype:	{sd_8255, sd_8255, sd_8255, sd_8255, sd_8254, sd_8254,
				sd_timer, sd_intr},
	      sdinfo:	{0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x00, 0x3F},
	      has_int_sce:1,
	      has_clk_gat_sce:1,
	      has_enhancements:1,
		},
#endif
};

/*
 * PCI driver table.
 */

#ifdef COMEDI_CONFIG_PCI
static DEFINE_PCI_DEVICE_TABLE(dio200_pci_table) = {
	{PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_AMPLICON_PCI215,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_AMPLICON_PCI272,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_AMPLICON_PCIE236,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_AMPLICON_PCIE215,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{PCI_VENDOR_ID_AMPLICON, PCI_DEVICE_ID_AMPLICON_PCIE296,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0}
};

MODULE_DEVICE_TABLE(pci, dio200_pci_table);
#endif /* COMEDI_CONFIG_PCI */

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((const dio200_board *)dev->board_ptr)
#define thislayout (&dio200_layouts[((dio200_board *)dev->board_ptr)->layout])

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
typedef struct {
#ifdef COMEDI_CONFIG_PCI
	struct pci_dev *pci_dev;	/* PCI device */
#endif
	struct dio200_region io;	/* Register region */
	int intr_sd;
} dio200_private;

#define devpriv ((dio200_private *)dev->private)

typedef struct {
	unsigned int ofs;		/* Counter base offset */
	unsigned int clk_sce_ofs;	/* CLK_SCE base offset */
	unsigned int gat_sce_ofs;	/* GAT_SCE base offset */
	int which;			/* Bit 5 of CLK_SCE or GAT_SCE */
	unsigned int clock_src[3];	/* Current clock sources */
	unsigned int gate_src[3];	/* Current gate sources */
	spinlock_t spinlock;
} dio200_subdev_8254;

typedef struct {
	unsigned int ofs;		/* DIO base offset */
} dio200_subdev_8255;

typedef struct {
	unsigned int ofs;
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
static int dio200_attach(comedi_device * dev, comedi_devconfig * it);
static int dio200_detach(comedi_device * dev);
static comedi_driver driver_amplc_dio200 = {
      driver_name:DIO200_DRIVER_NAME,
      module:THIS_MODULE,
      attach:dio200_attach,
      detach:dio200_detach,
      board_name:&dio200_boards[0].name,
      offset:sizeof(dio200_board),
      num_names:sizeof(dio200_boards) / sizeof(dio200_board),
};

#ifdef COMEDI_CONFIG_PCI
COMEDI_PCI_INITCLEANUP(driver_amplc_dio200, dio200_pci_table);
#else
COMEDI_INITCLEANUP(driver_amplc_dio200);
#endif

/*
 * Read 8-bit register.
 */
static unsigned char dio200_read8(comedi_device * dev, unsigned int offset)
{
	offset <<= devpriv->io.regshift;
	if (devpriv->io.regtype == io_regtype)
		return inb(devpriv->io.u.iobase + offset);
	else
		return readb(devpriv->io.u.membase + offset);
}

/*
 * Write 8-bit register.
 */
static void dio200_write8(comedi_device * dev, unsigned int offset,
		unsigned char val)
{
	offset <<= devpriv->io.regshift;
	if (devpriv->io.regtype == io_regtype)
		outb(val, devpriv->io.u.iobase + offset);
	else
		writeb(val, devpriv->io.u.membase + offset);
}

/*
 * Read 32-bit register.
 */
#ifdef COMEDI_CONFIG_PCI
static unsigned int dio200_read32(comedi_device * dev, unsigned int offset)
{
	offset <<= devpriv->io.regshift;
	if (devpriv->io.regtype == io_regtype)
		return inl(devpriv->io.u.iobase + offset);
	else
		return readl(devpriv->io.u.membase + offset);
}
#endif

/*
 * Write 32-bit register.
 */
#ifdef COMEDI_CONFIG_PCI
static void dio200_write32(comedi_device * dev, unsigned int offset,
		unsigned int val)
{
	offset <<= devpriv->io.regshift;
	if (devpriv->io.regtype == io_regtype)
		outl(val, devpriv->io.u.iobase + offset);
	else
		writel(val, devpriv->io.u.membase + offset);
}
#endif

/*
 * This function looks for a PCI device matching the requested board name,
 * bus and slot.
 */
#ifdef COMEDI_CONFIG_PCI
static int
dio200_find_pci(comedi_device * dev, int bus, int slot,
	struct pci_dev **pci_dev_p)
{
	struct pci_dev *pci_dev = NULL;

	*pci_dev_p = NULL;

	/* Look for matching PCI device. */
	for (pci_dev = pci_get_device(PCI_VENDOR_ID_AMPLICON, PCI_ANY_ID, NULL);
		pci_dev != NULL;
		pci_dev = pci_get_device(PCI_VENDOR_ID_AMPLICON,
			PCI_ANY_ID, pci_dev)) {
		/* If bus/slot specified, check them. */
		if (bus || slot) {
			if (bus != pci_dev->bus->number
				|| slot != PCI_SLOT(pci_dev->devfn))
				continue;
		}
		if (thisboard->model == anypci_model) {
			/* Match any supported model. */
			int i;

			for (i = 0; i < ARRAY_SIZE(dio200_boards); i++) {
				if (dio200_boards[i].bustype != pci_bustype)
					continue;
				if (pci_dev->device == dio200_boards[i].devid) {
					/* Change board_ptr to matched board. */
					dev->board_ptr = &dio200_boards[i];
					break;
				}
			}
			if (i == ARRAY_SIZE(dio200_boards))
				continue;
		} else {
			/* Match specific model name. */
			if (pci_dev->device != thisboard->devid)
				continue;
		}

		/* Found a match. */
		*pci_dev_p = pci_dev;
		return 0;
	}
	/* No match found. */
	if (bus || slot) {
		printk(KERN_ERR
			"comedi%d: error! no %s found at pci %02x:%02x!\n",
			dev->minor, thisboard->name, bus, slot);
	} else {
		printk(KERN_ERR "comedi%d: error! no %s found!\n",
			dev->minor, thisboard->name);
	}
	return -EIO;
}
#endif

/*
 * This function checks and requests an I/O region, reporting an error
 * if there is a conflict.
 */
static int
dio200_request_region(unsigned minor, unsigned long from, unsigned long extent)
{
	if (!from || !request_region(from, extent, DIO200_DRIVER_NAME)) {
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
dio200_subdev_intr_insn_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	dio200_subdev_intr *subpriv = s->private;

	if (thislayout->has_int_sce) {
		/* Just read the interrupt status register.  */
		data[1] = dio200_read8(dev, subpriv->ofs) & subpriv->valid_isns;
	} else {
		/* No interrupt status register. */
		data[0] = 0;
	}

	return 2;
}

/*
 * Called to stop acquisition for an 'INTERRUPT' subdevice.
 */
static void dio200_stop_intr(comedi_device * dev, comedi_subdevice * s)
{
	dio200_subdev_intr *subpriv = s->private;

	subpriv->active = 0;
	subpriv->enabled_isns = 0;
	if (thislayout->has_int_sce) {
		dio200_write8(dev, subpriv->ofs, 0);
	}
}

/*
 * Called to start acquisition for an 'INTERRUPT' subdevice.
 */
static int dio200_start_intr(comedi_device * dev, comedi_subdevice * s)
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
		if (thislayout->has_int_sce) {
			dio200_write8(dev, subpriv->ofs, isn_bits);
		}
	}

	return retval;
}

/*
 * Internal trigger function to start acquisition for an 'INTERRUPT' subdevice.
 */
static int
dio200_inttrig_start_intr(comedi_device * dev, comedi_subdevice * s,
	unsigned int trignum)
{
	dio200_subdev_intr *subpriv;
	unsigned long flags;
	int event = 0;

	if (trignum != 0)
		return -EINVAL;

	subpriv = s->private;

	comedi_spin_lock_irqsave(&subpriv->spinlock, flags);
	s->async->inttrig = 0;
	if (subpriv->active) {
		event = dio200_start_intr(dev, s);
	}
	comedi_spin_unlock_irqrestore(&subpriv->spinlock, flags);

	if (event) {
		comedi_event(dev, s);
	}

	return 1;
}

/*
 * This is called from the interrupt service routine to handle a read
 * scan on an 'INTERRUPT' subdevice.
 */
static int dio200_handle_read_intr(comedi_device * dev, comedi_subdevice * s)
{
	dio200_subdev_intr *subpriv = s->private;
	unsigned triggered;
	unsigned intstat;
	unsigned cur_enabled;
	unsigned int oldevents;
	unsigned long flags;
	unsigned int int_sce_ofs;

	int_sce_ofs = subpriv->ofs;
	triggered = 0;

	comedi_spin_lock_irqsave(&subpriv->spinlock, flags);
	oldevents = s->async->events;
	if (thislayout->has_int_sce) {
		/*
		 * Collect interrupt sources that have triggered and disable
		 * them temporarily.  Loop around until no extra interrupt
		 * sources have triggered, at which point, the valid part of
		 * the interrupt status register will read zero, clearing the
		 * cause of the interrupt.
		 *
		 * Mask off interrupt sources already seen to avoid infinite
		 * loop in case of misconfiguration.
		 */
		cur_enabled = subpriv->enabled_isns;
		while ((intstat = (dio200_read8(dev, int_sce_ofs)
					& subpriv->valid_isns & ~triggered))
				!= 0) {
			triggered |= intstat;
			cur_enabled &= ~triggered;
			dio200_write8(dev, int_sce_ofs, cur_enabled);
		}
	} else {
		/*
		 * No interrupt status register.  Assume the single interrupt
		 * source has triggered.
		 */
		triggered = subpriv->enabled_isns;
	}

	if (triggered) {
		/*
		 * Some interrupt sources have triggered and have been
		 * temporarily disabled to clear the cause of the interrupt.
		 *
		 * Reenable them NOW to minimize the time they are disabled.
		 */
		cur_enabled = subpriv->enabled_isns;
		if (thislayout->has_int_sce) {
			dio200_write8(dev, int_sce_ofs, cur_enabled);
		}

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
					s->async->events |= COMEDI_CB_ERROR
						| COMEDI_CB_OVERFLOW;
					comedi_error(dev, "buffer overflow");
				}

				/* Check for end of acquisition. */
				if (!subpriv->continuous) {
					/* stop_src == TRIG_COUNT */
					if (subpriv->stopcount > 0) {
						subpriv->stopcount--;
						if (subpriv->stopcount == 0) {
							s->async->events |=
								COMEDI_CB_EOA;
							dio200_stop_intr(dev,
								s);
						}
					}
				}
			}
		}
	}
	comedi_spin_unlock_irqrestore(&subpriv->spinlock, flags);

	if (oldevents != s->async->events) {
		comedi_event(dev, s);
	}

	return (triggered != 0);
}

/*
 * 'cancel' function for an 'INTERRUPT' subdevice.
 */
static int dio200_subdev_intr_cancel(comedi_device * dev, comedi_subdevice * s)
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
dio200_subdev_intr_cmdtest(comedi_device * dev, comedi_subdevice * s,
	comedi_cmd * cmd)
{
	int err = 0;
	unsigned int tmp;

	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= (TRIG_NOW | TRIG_INT);
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_EXT;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_NOW;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	cmd->stop_src &= (TRIG_COUNT | TRIG_NONE);
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	/* these tests are true if more than one _src bit is set */
	if ((cmd->start_src & (cmd->start_src - 1)) != 0)
		err++;
	if ((cmd->scan_begin_src & (cmd->scan_begin_src - 1)) != 0)
		err++;
	if ((cmd->convert_src & (cmd->convert_src - 1)) != 0)
		err++;
	if ((cmd->scan_end_src & (cmd->scan_end_src - 1)) != 0)
		err++;
	if ((cmd->stop_src & (cmd->stop_src - 1)) != 0)
		err++;

	if (err)
		return 2;

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

	if (err)
		return 3;

	/* step 4: fix up any arguments */

	/* if (err) return 4; */

	return 0;
}

/*
 * 'do_cmd' function for an 'INTERRUPT' subdevice.
 */
static int dio200_subdev_intr_cmd(comedi_device * dev, comedi_subdevice * s)
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
		comedi_event(dev, s);
	}

	return 0;
}

/*
 * This function initializes an 'INTERRUPT' subdevice.
 */
static int
dio200_subdev_intr_init(comedi_device * dev, comedi_subdevice * s,
	unsigned int offset, unsigned valid_isns)
{
	dio200_subdev_intr *subpriv;

	subpriv = kzalloc(sizeof(*subpriv), GFP_KERNEL);
	if (!subpriv) {
		printk(KERN_ERR "comedi%d: error! out of memory!\n",
			dev->minor);
		return -ENOMEM;
	}
	subpriv->ofs = offset;
	subpriv->valid_isns = valid_isns;
	spin_lock_init(&subpriv->spinlock);

	if (thislayout->has_int_sce) {
		/* Disable interrupt sources. */
		dio200_write8(dev, subpriv->ofs, 0);
	}

	s->private = subpriv;
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE | SDF_CMD_READ;
	if (thislayout->has_int_sce) {
		s->n_chan = DIO200_MAX_ISNS;
		s->len_chanlist = DIO200_MAX_ISNS;
	} else {
		/* No interrupt source register.  Support single channel. */
		s->n_chan = 1;
		s->len_chanlist = 1;
	}
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
dio200_subdev_intr_cleanup(comedi_device * dev, comedi_subdevice * s)
{
	dio200_subdev_intr *subpriv = s->private;

	if (subpriv) {
		kfree(subpriv);
	}
}

/*
 * Interrupt service routine.
 */
static irqreturn_t dio200_interrupt(int irq, void *d PT_REGS_ARG)
{
	comedi_device *dev = d;
	int handled;

	if (!dev->attached) {
		return IRQ_NONE;
	}

	if (devpriv->intr_sd >= 0) {
		handled = dio200_handle_read_intr(dev,
			dev->subdevices + devpriv->intr_sd);
	} else {
		handled = 0;
	}

	return IRQ_RETVAL(handled);
}

/*
 * Read an '8254' counter subdevice channel.
 */
static inline unsigned int
dio200_subdev_8254_read_chan(comedi_device * dev, comedi_subdevice * s,
	unsigned int chan)
{
	unsigned int i8254_ofs = ((dio200_subdev_8254 *)s->private)->ofs;
	unsigned int val;

	/* latch counter */
	val = chan << 6;
	dio200_write8(dev, i8254_ofs + i8254_control_reg, val);

	/* read lsb */
	val = dio200_read8(dev, i8254_ofs + chan);
	/* read msb */
	val += dio200_read8(dev, i8254_ofs + chan) << 8;

	return val;
}

/*
 * Write an '8254' counter subdevice channel.
 */
static inline void
dio200_subdev_8254_write_chan(comedi_device * dev, comedi_subdevice * s,
	unsigned int chan, unsigned int count)
{
	unsigned int i8254_ofs = ((dio200_subdev_8254 *)s->private)->ofs;

	/* write lsb */
	dio200_write8(dev, i8254_ofs + chan, count & 0xff);
	/* write msb */
	dio200_write8(dev, i8254_ofs + chan, (count >> 8) & 0xff);
}

/*
 * Set mode of an '8254' counter subdevice channel.
 */
static inline void
dio200_subdev_8254_set_mode(comedi_device * dev, comedi_subdevice * s,
	unsigned int chan, unsigned int mode)
{
	unsigned int i8254_ofs = ((dio200_subdev_8254 *)s->private)->ofs;
	unsigned int byte;

	byte = chan << 6;
	byte |= 0x30;		/* load lsb then msb */
	byte |= (mode & 0xf);	/* set counter mode and BCD|binary */
	dio200_write8(dev, i8254_ofs + i8254_control_reg, byte);
}

/*
 * Read status byte of an '8254' counter subdevice channel.
 */
static inline unsigned int
dio200_subdev_8254_status(comedi_device * dev, comedi_subdevice * s,
	unsigned int chan)
{
	unsigned int i8254_ofs = ((dio200_subdev_8254 *)s->private)->ofs;

	dio200_write8(dev, i8254_ofs + i8254_control_reg, (0xE0 | 2 << chan));
	return dio200_read8(dev, i8254_ofs + chan);
}

/*
 * Handle 'insn_read' for an '8254' counter subdevice.
 */
static int
dio200_subdev_8254_read(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	dio200_subdev_8254 *subpriv = s->private;
	int chan = CR_CHAN(insn->chanspec);
	unsigned long flags;

	if (insn->n == 0)
		return 0;

	comedi_spin_lock_irqsave(&subpriv->spinlock, flags);
	data[0] = dio200_subdev_8254_read_chan(dev, s, chan);
	comedi_spin_unlock_irqrestore(&subpriv->spinlock, flags);

	return 1;
}

/*
 * Handle 'insn_write' for an '8254' counter subdevice.
 */
static int
dio200_subdev_8254_write(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	dio200_subdev_8254 *subpriv = s->private;
	int chan = CR_CHAN(insn->chanspec);
	unsigned long flags;

	if (insn->n == 0)
		return 0;

	comedi_spin_lock_irqsave(&subpriv->spinlock, flags);
	dio200_subdev_8254_write_chan(dev, s, chan, data[0]);
	comedi_spin_unlock_irqrestore(&subpriv->spinlock, flags);

	return 1;
}

/*
 * Set gate source for an '8254' counter subdevice channel.
 */
static int
dio200_subdev_8254_set_gate_src(comedi_device * dev, comedi_subdevice *s,
	unsigned int counter_number, unsigned int gate_src)
{
	dio200_subdev_8254 *subpriv = s->private;
	unsigned char byte;

	if (!thislayout->has_clk_gat_sce)
		return -1;
	if (counter_number > 2)
		return -1;
	if (gate_src > (thislayout->has_enhancements ? 31 : 7))
		return -1;

	subpriv->gate_src[counter_number] = gate_src;
	byte = GAT_SCE(subpriv->which, counter_number, gate_src);
	dio200_write8(dev, subpriv->gat_sce_ofs, byte);

	return 0;
}

/*
 * Get gate source for an '8254' counter subdevice channel.
 */
static int
dio200_subdev_8254_get_gate_src(comedi_device * dev, comedi_subdevice *s,
	unsigned int counter_number)
{
	dio200_subdev_8254 *subpriv = s->private;

	if (!thislayout->has_clk_gat_sce)
		return -1;
	if (counter_number > 2)
		return -1;

	return subpriv->gate_src[counter_number];
}

/*
 * Set clock source for an '8254' counter subdevice channel.
 */
static int
dio200_subdev_8254_set_clock_src(comedi_device * dev, comedi_subdevice *s,
	unsigned int counter_number, unsigned int clock_src)
{
	dio200_subdev_8254 *subpriv = s->private;
	unsigned char byte;

	if (!thislayout->has_clk_gat_sce)
		return -1;
	if (counter_number > 2)
		return -1;
	if (clock_src > (thislayout->has_enhancements ? 31 : 7))
		return -1;

	subpriv->clock_src[counter_number] = clock_src;
	byte = CLK_SCE(subpriv->which, counter_number, clock_src);
	dio200_write8(dev, subpriv->clk_sce_ofs, byte);

	return 0;
}

/*
 * Get clock source for an '8254' counter subdevice channel.
 */
static int
dio200_subdev_8254_get_clock_src(comedi_device * dev, comedi_subdevice *s,
	unsigned int counter_number, lsampl_t * period_ns)
{
	dio200_subdev_8254 *subpriv = s->private;
	unsigned clock_src;

	if (!thislayout->has_clk_gat_sce)
		return -1;
	if (counter_number > 2)
		return -1;

	clock_src = subpriv->clock_src[counter_number];
	*period_ns = clock_period[clock_src];
	return clock_src;
}

/*
 * Handle 'insn_config' for an '8254' counter subdevice.
 */
static int
dio200_subdev_8254_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	dio200_subdev_8254 *subpriv = s->private;
	int ret = 0;
	int chan = CR_CHAN(insn->chanspec);
	unsigned long flags;

	comedi_spin_lock_irqsave(&subpriv->spinlock, flags);
	switch (data[0]) {
	case INSN_CONFIG_SET_COUNTER_MODE:
		if (data[1] > (I8254_MODE5 | I8254_BCD))
			ret = -EINVAL;
		else
			dio200_subdev_8254_set_mode(dev, s, chan, data[1]);
		break;
	case INSN_CONFIG_8254_READ_STATUS:
		data[1] = dio200_subdev_8254_status(dev, s, chan);
		break;
	case INSN_CONFIG_SET_GATE_SRC:
		ret = dio200_subdev_8254_set_gate_src(dev, s, chan, data[2]);
		if (ret < 0)
			ret = -EINVAL;
		break;
	case INSN_CONFIG_GET_GATE_SRC:
		ret = dio200_subdev_8254_get_gate_src(dev, s, chan);
		if (ret < 0) {
			ret = -EINVAL;
			break;
		}
		data[2] = ret;
		break;
	case INSN_CONFIG_SET_CLOCK_SRC:
		ret = dio200_subdev_8254_set_clock_src(dev, s, chan, data[1]);
		if (ret < 0)
			ret = -EINVAL;
		break;
	case INSN_CONFIG_GET_CLOCK_SRC:
		ret = dio200_subdev_8254_get_clock_src(dev, s, chan, &data[2]);
		if (ret < 0) {
			ret = -EINVAL;
			break;
		}
		data[1] = ret;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	comedi_spin_unlock_irqrestore(&subpriv->spinlock, flags);
	return ret < 0 ? ret : insn->n;
}

/*
 * This function initializes an '8254' counter subdevice.
 *
 * offset is the offset to the 8254 chip.
 */
static int
dio200_subdev_8254_init(comedi_device * dev, comedi_subdevice * s,
	unsigned int offset)
{
	dio200_subdev_8254 *subpriv;
	unsigned int chan;

	subpriv = kzalloc(sizeof(*subpriv), GFP_KERNEL);
	if (!subpriv) {
		printk(KERN_ERR "comedi%d: error! out of memory!\n",
			dev->minor);
		return -ENOMEM;
	}

	s->private = subpriv;
	s->type = COMEDI_SUBD_COUNTER;
	s->subdev_flags = SDF_WRITABLE | SDF_READABLE;
	s->n_chan = 3;
	s->maxdata = 0xFFFF;
	s->insn_read = dio200_subdev_8254_read;
	s->insn_write = dio200_subdev_8254_write;
	s->insn_config = dio200_subdev_8254_config;

	spin_lock_init(&subpriv->spinlock);
	subpriv->ofs = offset;
	if (thislayout->has_clk_gat_sce) {
		/* Derive CLK_SCE and GAT_SCE register offsets from
		 * 8254 offset. */
		subpriv->clk_sce_ofs =
			DIO200_XCLK_SCE + (offset >> 3);
		subpriv->gat_sce_ofs =
			DIO200_XGAT_SCE + (offset >> 3);
		subpriv->which = (offset >> 2) & 1;
	}

	/* Initialize channels. */
	for (chan = 0; chan < 3; chan++) {
		dio200_subdev_8254_set_mode(dev, s, chan,
			(I8254_MODE0 | I8254_BINARY));
		if (thislayout->has_clk_gat_sce) {
			/* Gate source 0 is VCC (logic 1). */
			dio200_subdev_8254_set_gate_src(dev, s, chan, 0);
			/* Clock source 0 is the dedicated clock input. */
			dio200_subdev_8254_set_clock_src(dev, s, chan, 0);
		}
	}

	return 0;
}

/*
 * This function cleans up an '8254' counter subdevice.
 */
static void
dio200_subdev_8254_cleanup(comedi_device * dev, comedi_subdevice * s)
{
	dio200_subdev_8254 *subpriv = s->private;

	if (subpriv) {
		kfree(subpriv);
	}
}

/*
 * This function sets I/O directions for an '8255' DIO subdevice.
 */
static void
dio200_subdev_8255_set_dir(comedi_device * dev, comedi_subdevice * s)
{
	dio200_subdev_8255 *subpriv = s->private;
	int config;

	config = CR_CW;
	/* 1 in io_bits indicates output, 1 in config indicates input */
	if (!(s->io_bits & 0x0000ff))
		config |= CR_A_IO;
	if (!(s->io_bits & 0x00ff00))
		config |= CR_B_IO;
	if (!(s->io_bits & 0x0f0000))
		config |= CR_C_LO_IO;
	if (!(s->io_bits & 0xf00000))
		config |= CR_C_HI_IO;

	dio200_write8(dev, subpriv->ofs + 3, config);
}

/*
 * Handle 'insn_bits' for an '8255' DIO subdevice.
 */
static int
dio200_subdev_8255_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	unsigned int i8255_ofs = ((dio200_subdev_8255 *)s->private)->ofs;

	if (data[0]) {
		s->state &= ~data[0];
		s->state |= (data[0] & data[1]);

		if (data[0] & 0xff) {
			dio200_write8(dev, i8255_ofs, s->state & 0xff);
		}
		if (data[0] & 0xff00) {
			dio200_write8(dev, i8255_ofs + 1,
				(s->state >> 8) & 0xff);
		}
		if (data[0] & 0xff0000) {
			dio200_write8(dev, i8255_ofs + 2,
				(s->state >> 16) & 0xff);
		}
	}

	data[1] = dio200_read8(dev, i8255_ofs);
	data[1] |= dio200_read8(dev, i8255_ofs + 1) << 8;
	data[1] |= dio200_read8(dev, i8255_ofs + 2) << 16;

	return 2;
}

/*
 * Handle 'insn_config' for an '8255' DIO subdevice.
 */
static int
dio200_subdev_8255_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	unsigned int mask;
	unsigned int bits;

	mask = 1 << CR_CHAN(insn->chanspec);
	if (mask & 0x0000ff) {
		bits = 0x0000ff;
	} else if (mask & 0x00ff00) {
		bits = 0x00ff00;
	} else if (mask & 0x0f0000) {
		bits = 0x0f0000;
	} else {
		bits = 0xf00000;
	}

	switch (data[0]) {
	case INSN_CONFIG_DIO_INPUT:
		s->io_bits &= ~bits;
		break;
	case INSN_CONFIG_DIO_OUTPUT:
		s->io_bits |= bits;
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] = (s->io_bits & bits) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
	}

	dio200_subdev_8255_set_dir(dev, s);

	return 1;
}

/*
 * This function initializes an '8255' DIO subdevice.
 *
 * offset is the offset to the 8255 chip.
 */
static int
dio200_subdev_8255_init(comedi_device * dev, comedi_subdevice * s,
	unsigned int offset)
{
	dio200_subdev_8255 *subpriv;

	subpriv = kzalloc(sizeof(*subpriv), GFP_KERNEL);
	if (!subpriv) {
		printk(KERN_ERR "comedi%d: error! out of memory!\n",
			dev->minor);
		return -ENOMEM;
	}

	subpriv->ofs = offset;

	s->private = subpriv;
	s->type = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
	s->n_chan = 24;
	s->range_table = &range_digital;
	s->maxdata = 1;
	s->insn_bits = dio200_subdev_8255_bits;
	s->insn_config = dio200_subdev_8255_config;

	s->state = 0;
	s->io_bits = 0;
	dio200_subdev_8255_set_dir(dev, s);

	return 0;
}

/*
 * This function cleans up an '8255' DIO subdevice.
 */
static void
dio200_subdev_8255_cleanup(comedi_device * dev, comedi_subdevice * s)
{
	dio200_subdev_8255 *subpriv = s->private;

	if (subpriv) {
		kfree(subpriv);
	}
}

/*
 * Handle 'insn_read' for a timer subdevice.
 */
#ifdef COMEDI_CONFIG_PCI
static int
dio200_subdev_timer_read(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int n;

	for (n = 0; n < insn->n; n++) {
		data[n] = dio200_read32(dev, DIO200_TS_COUNT);
	}
	return n;
}
#endif

/*
 * Reset timer subdevice.
 */
#ifdef COMEDI_CONFIG_PCI
static void
dio200_subdev_timer_reset(comedi_device * dev, comedi_subdevice * s)
{
	unsigned int clock;

	clock = dio200_read32(dev, DIO200_TS_CONFIG) & TS_CONFIG_CLK_SRC_MASK;
	dio200_write32(dev, DIO200_TS_CONFIG, clock | TS_CONFIG_RESET);
	dio200_write32(dev, DIO200_TS_CONFIG, clock);
}
#endif

/*
 * Get timer subdevice clock source and period.
 */
#ifdef COMEDI_CONFIG_PCI
static void
dio200_subdev_timer_get_clock_src(comedi_device * dev, comedi_subdevice * s,
		lsampl_t *src, lsampl_t *period)
{
	unsigned int clk;

	clk = dio200_read32(dev, DIO200_TS_CONFIG) & TS_CONFIG_CLK_SRC_MASK;
	*src = clk;
	*period = (clk < ARRAY_SIZE(ts_clock_period))
		? ts_clock_period[clk] : 0;
}
#endif

/*
 * Set timer subdevice clock source.
 */
#ifdef COMEDI_CONFIG_PCI
static int
dio200_subdev_timer_set_clock_src(comedi_device * dev, comedi_subdevice * s,
		lsampl_t src)
{
	if (src > TS_CONFIG_MAX_CLK_SRC) {
		return -EINVAL;
	}
	dio200_write32(dev, DIO200_TS_CONFIG, src);
	return 0;
}
#endif

/*
 * Handle 'insn_config' for a timer subdevice.
 */
#ifdef COMEDI_CONFIG_PCI
static int
dio200_subdev_timer_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	int ret = 0;

	switch (data[0]) {
	case INSN_CONFIG_RESET:
		dio200_subdev_timer_reset(dev, s);
		break;
	case INSN_CONFIG_SET_CLOCK_SRC:
		ret = dio200_subdev_timer_set_clock_src(dev, s, data[1]);
		if (ret < 0)
			ret = -EINVAL;
		break;
	case INSN_CONFIG_GET_CLOCK_SRC:
		dio200_subdev_timer_get_clock_src(dev, s, &data[1], &data[2]);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret < 0 ? ret : insn->n;
}
#endif

/*
 * This function initializes a timer subdevice.
 *
 * Uses the timestamp timer registers.  There is only one timestamp timer.
 */
#ifdef COMEDI_CONFIG_PCI
static int
dio200_subdev_timer_init(comedi_device * dev, comedi_subdevice * s)
{
	s->type = COMEDI_SUBD_TIMER;
	s->subdev_flags = SDF_READABLE | SDF_LSAMPL;
	s->n_chan = 1;
	s->maxdata = 0xFFFFFFFF;
	s->insn_read = dio200_subdev_timer_read;
	s->insn_config = dio200_subdev_timer_config;
	return 0;
}
#endif

/*
 * This function cleans up a timer subdevice.
 */
#ifdef COMEDI_CONFIG_PCI
static void
dio200_subdev_timer_cleanup(comedi_device * dev, comedi_subdevice * s)
{
	/* Nothing to do. */
}
#endif

#ifdef COMEDI_CONFIG_PCI
/*
 * This function does some special set-up for the PCIe boards
 * PCIe215, PCIe236, PCIe296.
 */
static int
dio200_pcie_board_setup(comedi_device * dev)
{
	struct pci_dev *pci_dev = devpriv->pci_dev;
	unsigned char __iomem *brbase;
	resource_size_t brlen;

	/*
	 * The board uses Altera Cyclone IV with PCI-Express hard IP.
	 * The FPGA configuration has the PCI-Express Avalon-MM Bridge
	 * Control registers in PCI BAR 0, offset 0, and the length of
	 * these registers is 0x4000.
	 *
	 * We need to write 0x80 to the "Avalon-MM to PCI-Express Interrupt
	 * Enable" register at offset 0x50 to allow generation of PCIe
	 * interrupts when RXmlrq_i is asserted in the SOPC Builder system.
	 */
	brlen = pci_resource_len(pci_dev, 0);
	if (brlen < 0x4000 ||
			!(pci_resource_flags(pci_dev, 0) & IORESOURCE_MEM)) {
		printk(KERN_ERR "comedi%d: error! bad PCI region!\n",
			dev->minor);
		return -EINVAL;
	}
	brbase = ioremap(pci_resource_start(pci_dev, 0), brlen);
	if (!brbase) {
		printk(KERN_ERR "comedi%d: error! failed to map registers!\n",
			dev->minor);
		return -ENOMEM;
	}
	writel(0x80, brbase + 0x50);
	iounmap(brbase);
	/*
	 * Enable "enhanced" features of board.
	 */
	dio200_write8(dev, DIO200_ENHANCE, 1);
	return 0;
}
#endif

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int dio200_attach(comedi_device * dev, comedi_devconfig * it)
{
	comedi_subdevice *s;
	unsigned long iobase = 0;
	unsigned int irq = 0;
#ifdef COMEDI_CONFIG_PCI
	struct pci_dev *pci_dev = NULL;
	int bus = 0, slot = 0;
#endif
	const dio200_layout *layout;
	int share_irq = 0;
	int sdx;
	unsigned n;
	int ret;

	printk(KERN_DEBUG "comedi%d: %s: attach\n", dev->minor,
		DIO200_DRIVER_NAME);

	if ((ret = alloc_private(dev, sizeof(dio200_private))) < 0) {
		printk(KERN_ERR "comedi%d: error! out of memory!\n",
			dev->minor);
		return ret;
	}

	/* Process options. */
	switch (thisboard->bustype) {
	case isa_bustype:
		iobase = it->options[0];
		irq = it->options[1];
		share_irq = 0;
		break;
#ifdef COMEDI_CONFIG_PCI
	case pci_bustype:
		bus = it->options[0];
		slot = it->options[1];
		share_irq = 1;

		if ((ret = dio200_find_pci(dev, bus, slot, &pci_dev)) < 0)
			return ret;
		devpriv->pci_dev = pci_dev;
		break;
#endif
	default:
		printk(KERN_ERR
			"comedi%d: %s: BUG! cannot determine board type!\n",
			dev->minor, DIO200_DRIVER_NAME);
		return -EINVAL;
		break;
	}

	devpriv->intr_sd = -1;
	devpriv->io.regtype = no_regtype;
	devpriv->io.regshift = thisboard->mainshift;

	/* Enable device and reserve I/O spaces. */
#ifdef COMEDI_CONFIG_PCI
	if (pci_dev) {
		resource_size_t base, len;
		unsigned int bar;

		ret = comedi_pci_enable(pci_dev, DIO200_DRIVER_NAME);
		if (ret < 0) {
			printk(KERN_ERR
				"comedi%d: error! cannot enable PCI device and request regions!\n",
				dev->minor);
			return ret;
		}
		bar = thisboard->mainbar;
		base = pci_resource_start(pci_dev, bar);
		len = pci_resource_len(pci_dev, bar);
		if (len < thisboard->mainsize) {
			printk(KERN_ERR
				"comedi%d: error! PCI region size too small!\n",
				dev->minor);
			return -EINVAL;
		}
		if ((pci_resource_flags(pci_dev, bar) & IORESOURCE_MEM) != 0) {
			devpriv->io.u.membase = ioremap(base, len);
			if (!devpriv->io.u.membase) {
				printk(KERN_ERR
					"comedi%d: error! cannot remap registers!\n",
					dev->minor);
				return -ENOMEM;
			}
			devpriv->io.regtype = mmio_regtype;
		} else {
			devpriv->io.u.iobase = (unsigned long)base;
			devpriv->io.regtype = io_regtype;
		}
		irq = pci_dev->irq;
	} else
#endif
	{
		ret = dio200_request_region(dev->minor, iobase,
			thisboard->mainsize);
		if (ret < 0) {
			return ret;
		}
		devpriv->io.u.iobase = iobase;
		devpriv->io.regtype = io_regtype;
	}

	switch (thisboard->model) {
#ifdef COMEDI_CONFIG_PCI
	case pcie215_model:
	case pcie236_model:
	case pcie296_model:
		ret = dio200_pcie_board_setup(dev);
		if (ret < 0) {
			return ret;
		}
		break;
#endif
	default:
		break;
	}

	layout = thislayout;
	if ((ret = alloc_subdevices(dev, layout->n_subdevs)) < 0) {
		printk(KERN_ERR "comedi%d: error! out of memory!\n",
			dev->minor);
		return ret;
	}

	for (n = 0; n < dev->n_subdevices; n++) {
		s = &dev->subdevices[n];
		switch (layout->sdtype[n]) {
		case sd_8254:
			/* counter subdevice (8254) */
			ret = dio200_subdev_8254_init(dev, s,
				layout->sdinfo[n]);
			if (ret < 0) {
				return ret;
			}
			break;
		case sd_8255:
			/* digital i/o subdevice (8255) */
			ret = dio200_subdev_8255_init(dev, s,
				layout->sdinfo[n]);
			if (ret < 0) {
				return ret;
			}
			break;
		case sd_intr:
			/* 'INTERRUPT' subdevice */
			if (irq) {
				ret = dio200_subdev_intr_init(dev, s,
					DIO200_INT_SCE, layout->sdinfo[n]);
				if (ret < 0) {
					return ret;
				}
				devpriv->intr_sd = n;
			} else {
				s->type = COMEDI_SUBD_UNUSED;
			}
			break;
		case sd_timer:
			/* timer subdevice */
#ifdef COMEDI_CONFIG_PCI
			ret = dio200_subdev_timer_init(dev, s);
			if (ret < 0) {
				return ret;
			}
#else
			s->type = COMEDI_SUBD_UNUSED;
#endif
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
		unsigned long flags = share_irq ? IRQF_SHARED : 0;

		if (comedi_request_irq(irq, dio200_interrupt, flags,
				DIO200_DRIVER_NAME, dev) >= 0) {
			dev->irq = irq;
		} else {
			printk(KERN_WARNING
				"comedi%d: warning! irq %u unavailable!\n",
				dev->minor, irq);
		}
	}

	printk(KERN_INFO "comedi%d: %s ", dev->minor, dev->board_name);
	if (thisboard->bustype == isa_bustype) {
		printk("(base %#lx) ", iobase);
	} else {
#ifdef COMEDI_CONFIG_PCI
		printk("(pci %s) ", pci_name(pci_dev));
#endif
	}
	if (irq) {
		printk("(irq %u%s) ", irq, (dev->irq ? "" : " UNAVAILABLE"));
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
static int dio200_detach(comedi_device * dev)
{
	const dio200_layout *layout;
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
			case sd_8254:
				dio200_subdev_8254_cleanup(dev, s);
				break;
			case sd_8255:
				dio200_subdev_8255_cleanup(dev, s);
				break;
			case sd_intr:
				dio200_subdev_intr_cleanup(dev, s);
				break;
			case sd_timer:
#ifdef COMEDI_CONFIG_PCI
				dio200_subdev_timer_cleanup(dev, s);
#endif
				break;
			default:
				break;
			}
		}
	}
	if (devpriv) {
		if (devpriv->io.regtype == mmio_regtype) {
			iounmap(devpriv->io.u.membase);
		}
#ifdef COMEDI_CONFIG_PCI
		if (devpriv->pci_dev) {
			if (devpriv->io.regtype != no_regtype) {
				comedi_pci_disable(devpriv->pci_dev);
			}
			pci_dev_put(devpriv->pci_dev);
		} else
#endif
		{
			if (devpriv->io.regtype == io_regtype) {
				release_region(devpriv->io.u.iobase,
					thisboard->mainsize);
			}
		}
	}
	if (dev->board_name) {
		printk(KERN_INFO "comedi%d: %s removed\n",
			dev->minor, dev->board_name);
	}

	return 0;
}
