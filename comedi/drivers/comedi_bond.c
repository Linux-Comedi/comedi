/*
    comedi/drivers/comedi_bond.c
    A Comedi driver to 'bond' or merge multiple drivers and devices as one.

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@schleef.org>
    Copyright (C) 2005 Calin A. Culianu <calin@ajvar.org>

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
Driver: comedi_bond
Description: A driver to 'bond' (merge) multiple subdevices from multiple devices together as one.
Devices:
Author: ds
Updated: Mon, 10 Oct 00:18:25 -0500
Status: works

This driver allows you to 'bond' (merge) multiple comedi subdevices
(coming from possibly difference boards and/or drivers) together.  For
example, if you had a board with 2 different DIO subdevices, and
another with 1 DIO subdevice, you could 'bond' them with this driver
so that they look like one big fat DIO subdevice.  This makes writing
applications slightly easier as you don't have to worry about managing
different subdevices in the application -- you just worry about
indexing one linear array of channel id's.

Right now only DIO subdevices are supported as that's the personal itch
I am scratching with this driver.  If you want to add support for AI and AO
subdevs, go right on ahead and do so!

Commands aren't supported -- although it would be cool if they were.

Configuration Options:
  List of comedi-minors to bond.  All subdevices of the same type
  within each minor will be concatenated together in the order given here.
*/

/*
 * The previous block comment is used to automatically generate
 * documentation in Comedi and Comedilib.  The fields:
 *
 *  Driver: the name of the driver
 *  Description: a short phrase describing the driver.  Don't list boards.
 *  Devices: a full list of the boards that attempt to be supported by
 *    the driver.  Format is "(manufacturer) board name [comedi name]",
 *    where comedi_name is the name that is used to configure the board.
 *    See the comment near board_name: in the comedi_driver structure
 *    below.  If (manufacturer) or [comedi name] is missing, the previous
 *    value is used.
 *  Author: you
 *  Updated: date when the _documentation_ was last updated.  Use 'date -R'
 *    to get a value for this.
 *  Status: a one-word description of the status.  Valid values are:
 *    works - driver works correctly on most boards supported, and
 *      passes comedi_test.
 *    unknown - unknown.  Usually put there by ds.
 *    experimental - may not work in any particular release.  Author
 *      probably wants assistance testing it.
 *    bitrotten - driver has not been update in a long time, probably
 *      doesn't work, and probably is missing support for significant
 *      Comedi interface features.
 *    untested - author probably wrote it "blind", and is believed to
 *      work, but no confirmation.
 *
 * These headers should be followed by a blank line, and any comments
 * you wish to say about the driver.  The comment area is the place
 * to put any known bugs, limitations, unsupported features, supported
 * command triggers, whether or not commands are supported on particular
 * subdevices, etc.
 *
 * Somewhere in the comment should be information about configuration
 * options that are used with comedi_config.
 */

#include <linux/comedilib.h>
#include <linux/comedidev.h>
#include <linux/string.h>

#define MODULE_NAME "comedi_bond"
MODULE_LICENSE("GPL");
MODULE_VERSION(COMEDI_RELEASE);

#ifndef STR
#  define STR1(x) #x
#  define STR(x) STR1(x)
#endif

int debug = 0;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug,
	"If true, print extra cryptic debugging output useful only to developers probably.");

#define LOG_MSG(x...) printk(KERN_INFO MODULE_NAME": "x)
#define DEBUG(x...)  do { if(debug) printk(KERN_DEBUG MODULE_NAME": DEBUG: "x); } while(0)
#define WARNING(x...)  printk(KERN_WARNING MODULE_NAME ": WARNING: "x)
#define ERROR(x...)  printk(KERN_ERR MODULE_NAME ": INTERNAL ERROR: "x)
MODULE_AUTHOR("Calin A. Culianu");
MODULE_DESCRIPTION(MODULE_NAME
	": A driver for COMEDI to bond multiple COMEDI devices together as one.  In the words of John Lennon: 'And the world will live as one...'");

/*
 * Board descriptions for two imaginary boards.  Describing the
 * boards in this way is optional, and completely driver-dependent.
 * Some drivers use arrays such as this, other do not.
 */
struct BondingBoard {
	const char *name;
};
typedef struct BondingBoard BondingBoard;

static const BondingBoard bondingBoards[] = {
	{
		.name	= MODULE_NAME,
	},
};

/*
 * Useful for shorthand access to the particular board structure
 */
#define thisboard ((const BondingBoard *)dev->board_ptr)

struct BondedDevice {
	comedi_t *dev;
	unsigned minor;
	unsigned subdev;
	unsigned subdev_type;
	unsigned nchans;
};
typedef struct BondedDevice BondedDevice;

/* this structure is for data unique to this hardware driver.  If
   several hardware drivers keep similar information in this structure,
   feel free to suggest moving the variable to the comedi_device struct.  */
struct Private {
# define MAX_BOARD_NAME 256
	char name[MAX_BOARD_NAME];
	struct BondedDevice **devs;
	unsigned ndevs;
	unsigned nchans;
};
typedef struct Private Private;

/*
 * most drivers define the following macro to make it easy to
 * access the private structure.
 */
#define devpriv ((Private *)dev->private)

/*
 * The comedi_driver structure tells the Comedi core module
 * which functions to call to configure/deconfigure (attach/detach)
 * the board, and also about the kernel module that contains
 * the device code.
 */
static int bonding_attach(comedi_device * dev, comedi_devconfig * it);
static int bonding_detach(comedi_device * dev);
/** Build Private array of all devices.. */
static int doDevConfig(comedi_device * dev, comedi_devconfig * it);
static void doDevUnconfig(comedi_device * dev);

static comedi_driver driver_bonding = {
	.driver_name	= MODULE_NAME,
	.module		= THIS_MODULE,
	.attach		= bonding_attach,
	.detach		= bonding_detach,
	.board_name	= &bondingBoards[0].name,
	.offset		= sizeof(BondingBoard),
	.num_names	= ARRAY_SIZE(bondingBoards),
};

static int bonding_dio_insn_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static int bonding_dio_insn_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);

/*
 * Attach is called by the Comedi core to configure the driver
 * for a particular board.  If you specified a board_name array
 * in the driver structure, dev->board_ptr contains that
 * address.
 */
static int bonding_attach(comedi_device * dev, comedi_devconfig * it)
{
	comedi_subdevice *s;
	int err;

	LOG_MSG("comedi%d\n", dev->minor);

/*
 * Allocate the private structure area.  alloc_private() is a
 * convenient macro defined in comedidev.h.
 */
	if (alloc_private(dev, sizeof(Private)) < 0)
		return -ENOMEM;

/*
 * Setup our bonding from config params.. sets up our Private struct..
 */
	err = doDevConfig(dev, it);
	if (err)
		return err;

/*
 * Initialize dev->board_name.  Note that we can use the "thisboard"
 * macro now, since we just initialized it in the last line.
 */
	dev->board_name = devpriv->name;

/*
 * Allocate the subdevice structures.  alloc_subdevice() is a
 * convenient macro defined in comedidev.h.
 */
	if (alloc_subdevices(dev, 1) < 0)
		return -ENOMEM;

	s = dev->subdevices + 0;
	s->type = COMEDI_SUBD_DIO;
	s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
	s->n_chan = devpriv->nchans;
	s->maxdata = 1;
	s->range_table = &range_digital;
	s->insn_bits = bonding_dio_insn_bits;
	s->insn_config = bonding_dio_insn_config;

	LOG_MSG("attached with %u DIO channels coming from %u different subdevices all bonded together.  John Lennon would be proud!\n", devpriv->nchans, devpriv->ndevs);

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
static int bonding_detach(comedi_device * dev)
{
	LOG_MSG("comedi%d: remove\n", dev->minor);
	doDevUnconfig(dev);
	return 0;
}

/* DIO devices are slightly special.  Although it is possible to
 * implement the insn_read/insn_write interface, it is much more
 * useful to applications if you implement the insn_bits interface.
 * This allows packed reading/writing of the DIO channels.  The
 * comedi core can convert between insn_bits and insn_read/write */
static int bonding_dio_insn_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	unsigned int n_left, n_done, base_chan;
	unsigned int write_mask, data_bits;
	BondedDevice **devs;

	write_mask = data[0];
	data_bits = data[1];
	base_chan = CR_CHAN(insn->chanspec);
	/* Do a maximum of 32 channels, starting from base_chan. */
	n_left = devpriv->nchans - base_chan;
	if (n_left > 32)
		n_left = 32;

	n_done = 0;
	devs = devpriv->devs;
	do {
		BondedDevice *bdev = *devs++;

		if (base_chan < bdev->nchans) {
			/* Base channel falls within bonded device. */
			unsigned int b_chans, b_mask, b_write_mask, b_data_bits;
			int ret;

			/*
			 * Get num channels to do for bonded device and set up
			 * mask and data bits for bonded device.
			 */
			b_chans = bdev->nchans - base_chan;
			if (b_chans > n_left)
				b_chans = n_left;
			b_mask = (b_chans < 32) ? ((1U << b_chans) - 1)
						: 0xffffffffu;
			b_write_mask = (write_mask >> n_done) & b_mask;
			b_data_bits = (data_bits >> n_done) & b_mask;
			/* Read/Write the new digital lines. */
			ret = comedi_dio_bitfield2(bdev->dev, bdev->subdev,
					b_write_mask, &b_data_bits, base_chan);
			if (ret < 0)
				return ret;
			/* Place read bits into data[1]. */
			data[1] &= ~(b_mask << n_done);
			data[1] |= (b_data_bits & b_mask) << n_done;
			/*
			 * Set up for following bonded device (if still have
			 * channels to read/write).
			 */
			base_chan = 0;
			n_done += b_chans;
			n_left -= b_chans;
		} else {
			/* Skip bonded devices before base channel. */
			base_chan -= bdev->nchans;
		}
	} while (n_left);

	return insn->n;
}

static int bonding_dio_insn_config(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	unsigned int chan = CR_CHAN(insn->chanspec);
	int ret;
	BondedDevice *bdev;
	BondedDevice **devs;
	unsigned int io;

	/*
	 * Locate bonded subdevice and adjust channel.
	 */
	devs = devpriv->devs;
	for (bdev = *devs++; chan >= bdev->nchans; bdev = *devs++)
		chan -= bdev->nchans;

	/*
	 * The input or output configuration of each digital line is
	 * configured by a special insn_config instruction.  chanspec
	 * contains the channel to be changed, and data[0] contains the
	 * configuration instruction INSN_CONFIG_DIO_OUTPUT,
	 * INSN_CONFIG_DIO_INPUT or INSN_CONFIG_DIO_QUERY.
	 *
	 * Note than INSN_CONFIG_DIO_OUTPUT == COMEDI_OUTPUT, and
	 * INSN_CONFIG_DIO_INPUT == COMEDI_INPUT.  This is deliberate ;)
	 */
	switch (data[0]) {
	case INSN_CONFIG_DIO_OUTPUT:
	case INSN_CONFIG_DIO_INPUT:
		ret = comedi_dio_config(bdev->dev, bdev->subdev, chan, data[0]);
		break;
	case INSN_CONFIG_DIO_QUERY:
		ret = comedi_dio_get_config(bdev->dev, bdev->subdev, chan, &io);
		if (ret >= 0)
			data[1] = io;
		break;
	default:
		ret = -EINVAL;
		break;
	}
	if (ret >= 0)
		ret = insn->n;
	return ret;
}

static int doDevConfig(comedi_device * dev, comedi_devconfig * it)
{
	int i;
	DECLARE_BITMAP(devs_opened, COMEDI_NUM_BOARD_MINORS) = { 0 };

	devpriv->name[0] = 0;
	/* Loop through all comedi devices specified on the command-line,
	   building our device list */
	for (i = 0; i < COMEDI_NDEVCONFOPTS && (!i || it->options[i]); ++i) {
		int minor = it->options[i];
		comedi_t *d;
		int sdev = -1;
		int nope = 1;

		if (minor < 0 || minor >= COMEDI_NUM_BOARD_MINORS) {
			ERROR("Minor %d is invalid!\n", minor);
			return -EINVAL;
		}
		if (minor == dev->minor) {
			ERROR("Cannot bond this driver to itself!\n");
			return -EINVAL;
		}
		if (test_and_set_bit(minor, devs_opened)) {
			ERROR("Minor %d specified more than once!\n", minor);
			return -EINVAL;
		}

		d = comedi_open_minor_from(minor, dev->minor);
		if (!d) {
			ERROR("Minor %u could not be opened\n", minor);
			return -ENODEV;
		}

		/* Do DIO, as that's all we support now.. */
		while ((sdev = comedi_find_subdevice_by_type(d, COMEDI_SUBD_DIO,
					sdev + 1)) > -1) {
			BondedDevice *bdev;
			BondedDevice **devs;
			int nchans;

			if ((nchans = comedi_get_n_channels(d, sdev)) <= 0) {
				ERROR("comedi_get_n_channels() returned %d on minor %u subdev %d!\n", nchans, minor, sdev);
				nope = -EINVAL;
				break;
			}
			bdev = kmalloc(sizeof(*bdev), GFP_KERNEL);
			if (!bdev) {
				ERROR("Out of memory.\n");
				nope = -ENOMEM;
				break;
			}
			bdev->dev = d;
			bdev->minor = minor;
			bdev->subdev = sdev;
			bdev->subdev_type = COMEDI_SUBD_DIO;
			bdev->nchans = nchans;

			/* Now put bdev pointer at end of devpriv->devs array list.. */

			/* ergh.. ugly.. we need to realloc :(  */
			devs = krealloc(devpriv->devs,
					(devpriv->ndevs + 1) * sizeof(*devs),
					GFP_KERNEL);
			if (!devs) {
				ERROR("Could not allocate memory. Out of memory?\n");
				kfree(bdev);
				nope = -ENOMEM;
				break;
			}
			devpriv->devs = devs;
			devpriv->devs[devpriv->ndevs++] = bdev;
			{
	/** Append dev:subdev to devpriv->name */
				char buf[20];
				int left =
					MAX_BOARD_NAME - strlen(devpriv->name) -
					1;
				snprintf(buf, sizeof(buf), "%u:%u ",
					bdev->minor, bdev->subdev);
				buf[sizeof(buf) - 1] = 0;
				strncat(devpriv->name, buf, left);
			}
			devpriv->nchans += nchans;
			nope = 0;	/* keeps the device open */
		}

		if (nope) {
			/* Opened device cannot be used, so close it. */
			comedi_close_from(d, dev->minor);
			if (nope < 0)
				return nope;
		}
	}

	if (!devpriv->nchans) {
		ERROR("No channels found!\n");
		return -EINVAL;
	}

	return 0;
}

static void doDevUnconfig(comedi_device * dev)
{
	if (devpriv) {
		DECLARE_BITMAP(devs_closed, COMEDI_NUM_BOARD_MINORS) = { 0 };

		while (devpriv->ndevs-- && devpriv->devs) {
			BondedDevice *bdev = devpriv->devs[devpriv->ndevs];
			if (!bdev)
				continue;
			if (!test_and_set_bit(bdev->minor, devs_closed))
				comedi_close_from(bdev->dev, dev->minor);
			kfree(bdev);
		}
		kfree(devpriv->devs);
		kfree(devpriv);
		dev->private = 0;
	}
}

static int __init init(void)
{
	return comedi_driver_register(&driver_bonding);
}

static void __exit cleanup(void)
{
	comedi_driver_unregister(&driver_bonding);
}

module_init(init);
module_exit(cleanup);
