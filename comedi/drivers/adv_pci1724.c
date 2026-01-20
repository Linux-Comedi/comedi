/*******************************************************************************
   comedi/drivers/adv_pci1724.c
   This is a driver for the Advantech PCI-1724U card.

   Author:  Frank Mori Hess <fmh6jj@gmail.com>
   Copyright (C) 2013 GnuBIO Inc

   COMEDI - Linux Control and Measurement Device Interface
   Copyright (C) 1997-8 David A. Schleef <ds@schleef.org>

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

*******************************************************************************/

/*
Driver: adv_pci1724
Description: Advantech PCI-1724U
Devices: [Advantech] PCI-1724U (adv_pci1724)
Author: Frank Mori Hess <fmh6jj@gmail.com>
Updated: Wed, 06 Nov 2019 13:36:28 +0000
Status: works

Configuration Options:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)

  If bus/slot is not specified, the first supported
  PCI device found will be used.


Subdevice 0 is the analog output.
Subdevice 1 is the offset calibration for the analog output.
Subdevice 2 is the gain calibration for the analog output.

The calibration offset and gains have quite a large effect on the
analog output, so it is possible to adjust the analog output to
have an output range significantly different from the board's
nominal output ranges. For a calibrated +/-10V range, the analog
output's offset will be set somewhere near mid-range (0x2000) and
its gain will be near maximum (0x3fff).

There is really no difference between the board's documented 0-20mA
versus 4-20mA output ranges. To pick one or the other is simply a
matter of adjusting the offset and gain calibration until the board
outputs in the desired range.

Subdevice 0 supports a synchronized output mode.  In this mode,
values written to the analog output channels will not appear on the
outputs until synchronized output is triggered.

An INSN_CONFIG_ARM configuration instruction is used to enable
synchronized output mode and optionally trigger synchronized output.
If data[1] is non-zero, synchronized output will be triggered.

    comedi_insn insn;
    lsampl_t data[2];
    int ret;
    memset(&insn, 0, sizeof(insn));
    insn.insn = INSN_CONFIG;
    insn.subdev = 0;
    insn.chanspec = 0;
    insn.data = data;
    insn.n = 2;
    data[0] = INSN_CONFIG_ARM;
    data[1] = trigger; // 0 = only enable; 1 = trigger now
    ret = comedi_do_insn(d, &insn);

An INSN_CONFIG_DISARM configuration instruction is used to disable
synchronized output mode.

    comedi_insn insn;
    lsampl_t data[1];
    int ret;
    memset(&insn, 0, sizeof(insn));
    insn.insn = INSN_CONFIG;
    insn.subdev = 0;
    insn.chanspec = 0;
    insn.data = data;
    insn.n = 1;
    data[0] = INSN_CONFIG_DISARM;
    ret = comedi_do_insn(d, &insn);
*/

#include <linux/comedidev.h>

#include "comedi_pci.h"


/*
 * PCI bar 2 Register I/O map (dev->iobase)
 */
#define PCI1724_DAC_CTRL_REG		0x00
#define PCI1724_DAC_CTRL_GX(x)		(1U << (20 + ((x) / 8)))
#define PCI1724_DAC_CTRL_CX(x)		(((x) % 8) << 16)
#define PCI1724_DAC_CTRL_MODE(x)	(((x) & 0x3) << 14)
#define PCI1724_DAC_CTRL_MODE_GAIN	PCI1724_DAC_CTRL_MODE(1)
#define PCI1724_DAC_CTRL_MODE_OFFSET	PCI1724_DAC_CTRL_MODE(2)
#define PCI1724_DAC_CTRL_MODE_NORMAL	PCI1724_DAC_CTRL_MODE(3)
#define PCI1724_DAC_CTRL_MODE_MASK	PCI1724_DAC_CTRL_MODE(3)
#define PCI1724_DAC_CTRL_DATA(x)	(((x) & 0x3fff) << 0)
#define PCI1724_SYNC_CTRL_REG		0x04
#define PCI1724_SYNC_CTRL_DACSTAT	(1U << 1)
#define PCI1724_SYNC_CTRL_SYN		(1U << 0)
#define PCI1724_EEPROM_CTRL_REG		0x08
#define PCI1724_SYNC_TRIG_REG		0x0c  /* any value works */
#define PCI1724_BOARD_ID_REG		0x10
#define PCI1724_BOARD_ID_MASK		(0xf << 0)

typedef struct {
	unsigned short mode;
	unsigned short syn;
	sampl_t readback[32];
} pci1724_subdev_private;

typedef struct {
	struct pci_dev *pcidev;
	pci1724_subdev_private spriv[3];
} pci1724_private;

/*the following macros to make it easy to
* access the private structure.
*/
#define devpriv ((pci1724_private *)dev->private)
#define subpriv ((pci1724_subdev_private *)s->private)

static const comedi_lrange adv_pci1724_ao_ranges = {
	4, {
		BIP_RANGE(10),
		RANGE_mA(0, 20),
		RANGE_mA(4, 20),
		RANGE_unitless(0, 1)
	}
};

static int adv_pci1724_insn_read(comedi_device *dev, comedi_subdevice *s,
				 comedi_insn *insn, lsampl_t *data)
{
	unsigned int chan = CR_CHAN(insn->chanspec);
	int i;

	for (i = 0; i < insn->n; i++) {
		data[i] = subpriv->readback[chan];
	}

	return insn->n;
}

static int adv_pci1724_wait_dac(comedi_device *dev, comedi_subdevice *s)
{
	int timeout;
	unsigned int status = 0;

	for (timeout = 1000; timeout > 0; timeout--) {
		status = inl(dev->iobase + PCI1724_SYNC_CTRL_REG);
		if ((status & PCI1724_SYNC_CTRL_DACSTAT) == 0)
			break;
		comedi_udelay(1);
	}
	if (timeout <= 0)
		return -ETIMEDOUT;

	/* Enable or disable synchronous output mode, if changed. */
	if (subpriv->syn != (status & PCI1724_SYNC_CTRL_SYN))
		outl(subpriv->syn, dev->iobase + PCI1724_SYNC_CTRL_REG);

	return 0;
}

static int adv_pci1724_insn_write(comedi_device *dev, comedi_subdevice *s,
				  comedi_insn *insn, lsampl_t *data)
{
	unsigned int mode = subpriv->mode;
	unsigned int chan = CR_CHAN(insn->chanspec);
	unsigned int ctrl;
	int i;

	ctrl = PCI1724_DAC_CTRL_GX(chan) | PCI1724_DAC_CTRL_CX(chan) | mode;

	for (i = 0; i < insn->n; ++i) {
		lsampl_t val = data[i];
		int ret;

		ret = adv_pci1724_wait_dac(dev, s);
		if (ret)
			return ret;

		outl(ctrl | PCI1724_DAC_CTRL_DATA(val),
		     dev->iobase + PCI1724_DAC_CTRL_REG);

		subpriv->readback[chan] = val;
	}

	return insn->n;
}

static int adv_pci1724_sync_mode(comedi_device *dev, comedi_subdevice *s,
				 int enable, int trigger)
{
	int ret;

	/* Enable or disable synchronized output mode. */
	subpriv->syn = enable ? PCI1724_SYNC_CTRL_SYN : 0;
	/* Wait until DAC ready, and update the SYN bit. */
	ret = adv_pci1724_wait_dac(dev, s);
	if (ret)
		return ret;

	if (enable && trigger) {
		/* Trigger synchronous output. */
		outl(0, dev->iobase + PCI1724_SYNC_TRIG_REG);
	}
	return 0;
}

static int adv_pci1724_insn_config(comedi_device *dev, comedi_subdevice *s,
				   comedi_insn *insn, lsampl_t *data)
{
	int ret;

	switch (data[0]) {
	case INSN_CONFIG_ARM:
		ret = adv_pci1724_sync_mode(dev, s, 1, data[1]);
		break;
	case INSN_CONFIG_DISARM:
		ret = adv_pci1724_sync_mode(dev, s, 0, 0);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret < 0 ? ret : insn->n;
}

static int adv_pci1724_attach_common(comedi_device *dev, struct pci_dev *pcidev)
{
	comedi_subdevice *s;
	unsigned int board_id;
	unsigned char pci_bus, pci_slot, pci_func;
	unsigned int iobase;
	int ret;

	pci_bus = pcidev->bus->number;
	pci_slot = PCI_SLOT(pcidev->devfn);
	pci_func = PCI_FUNC(pcidev->devfn);
	iobase = pci_resource_start(pcidev, 2);

	board_id = inl(dev->iobase + PCI1724_BOARD_ID_REG) &
		PCI1724_BOARD_ID_MASK;

	rt_printk(KERN_CONT ", b:s:f=%d:%d:%d, io=0x%4x, board_id=%d",
		pci_bus, pci_slot, pci_func, iobase, board_id);

	dev->iobase = iobase;
	dev->board_name = "pci1724u";
	devpriv->pcidev = pcidev;

	ret = alloc_subdevices(dev, 3);
	if (ret) {
		rt_printk(KERN_CONT " - Allocation failed!\n");
		return ret;
	}

	/* Analog Output subdevice */
	s = &dev->subdevices[0];
	s->type		= COMEDI_SUBD_AO;
	s->subdev_flags	= SDF_READABLE | SDF_WRITABLE | SDF_GROUND;
	s->n_chan	= 32;
	s->maxdata	= 0x3fff;
	s->range_table	= &adv_pci1724_ao_ranges;
	s->insn_read	= adv_pci1724_insn_read;
	s->insn_write	= adv_pci1724_insn_write;
	s->insn_config	= adv_pci1724_insn_config;
	s->private	= &devpriv->spriv[0];
	subpriv->mode	= PCI1724_DAC_CTRL_MODE_NORMAL;

	/* Offset Calibration subdevice */
	s = &dev->subdevices[1];
	s->type		= COMEDI_SUBD_CALIB;
	s->subdev_flags	= SDF_READABLE | SDF_WRITABLE | SDF_INTERNAL;
	s->n_chan	= 32;
	s->maxdata	= 0x3fff;
	s->insn_read	= adv_pci1724_insn_read;
	s->insn_write	= adv_pci1724_insn_write;
	s->private	= &devpriv->spriv[1];
	subpriv->mode	= PCI1724_DAC_CTRL_MODE_OFFSET;

	/* Gain Calibration subdevice */
	s = &dev->subdevices[2];
	s->type		= COMEDI_SUBD_CALIB;
	s->subdev_flags	= SDF_READABLE | SDF_WRITABLE | SDF_INTERNAL;
	s->n_chan	= 32;
	s->maxdata	= 0x3fff;
	s->insn_read	= adv_pci1724_insn_read;
	s->insn_write	= adv_pci1724_insn_write;
	s->private	= &devpriv->spriv[2];
	subpriv->mode	= PCI1724_DAC_CTRL_MODE_GAIN;

	rt_printk(KERN_CONT " - attached\n");
	return 0;
}

static int adv_pci1724_attach(comedi_device *dev, comedi_devconfig *it)
{
	struct pci_dev *pcidev;
	int opt_bus, opt_slot;
	int ret;
	const char *errstr;

	rt_printk("comedi%d: adv_pci1724", dev->minor);

	opt_bus = it->options[0];
	opt_slot = it->options[1];

	if ((ret = alloc_private(dev, sizeof(pci1724_private))) < 0) {
		rt_printk(KERN_CONT " - Allocation failed!\n");
		return -ENOMEM;
	}

	/* Look for matching PCI device */
	errstr = "not found!";
	pcidev = NULL;
	while (NULL != (pcidev =
			pci_get_device(PCI_VENDOR_ID_ADVANTECH,
				0x1724, pcidev))) {
		/* Found matching vendor/device. */
		if (opt_bus || opt_slot) {
			/* Check bus/slot. */
			if (opt_bus != pcidev->bus->number
				|| opt_slot != PCI_SLOT(pcidev->devfn))
				continue;	/* no match */
		}
		/*
		 * Look for device that isn't in use.
		 * Enable PCI device and request regions.
		 */
		if (comedi_pci_enable(pcidev, "adv_pci1724")) {
			errstr = "failed to enable PCI device and request regions!";
			continue;
		}
		break;
	}

	if (!pcidev) {
		if (opt_bus || opt_slot) {
			rt_printk(KERN_CONT " - Card at b:s %d:%d %s\n",
				opt_bus, opt_slot, errstr);
		} else {
			rt_printk(KERN_CONT " - Card %s\n", errstr);
		}
		return -EIO;
	}

	return adv_pci1724_attach_common(dev, pcidev);
}

static int adv_pci1724_auto_attach(comedi_device *dev, unsigned long context)
{
	struct pci_dev *pcidev = comedi_to_pci_dev(dev);
	int ret;

	rt_printk("comedi%d: adv_pci1724", dev->minor);

	if ((ret = alloc_private(dev, sizeof(pci1724_private))) < 0) {
		rt_printk(KERN_CONT " - Allocation failed!\n");
		return -ENOMEM;
	}

	/*
	 * adv_pci1724_attach() calls comedi_pci_enable() before
	 * adv_pci1724_attach_common(), so do the same.
	 */
	ret = comedi_pci_enable(pcidev, "adv_pci1724");
	if (ret) {
		rt_printk(KERN_CONT " - failed to enable PCI device and request regions!\n");
		return ret;
	}

	/* pci_dev_get() call matches pci_dev_put() in adv_pci1724_detach() */
	pci_dev_get(pcidev);

	return adv_pci1724_attach_common(dev, pcidev);
}

static int adv_pci1724_detach(comedi_device *dev)
{
	printk("comedi%d: adv_pci1724: remove\n", dev->minor);

	if (dev->private) {
		if (devpriv->pcidev) {
			if (dev->iobase) {
				comedi_pci_disable(devpriv->pcidev);
			}
			pci_dev_put(devpriv->pcidev);
		}
	}

	return 0;
}

static comedi_driver adv_pci1724_driver = {
	.driver_name	= "adv_pci1724",
	.module		= THIS_MODULE,
	.attach		= adv_pci1724_attach,
	.auto_attach	= adv_pci1724_auto_attach,
	.detach		= adv_pci1724_detach,
};

static const struct pci_device_id adv_pci1724_pci_table[] = {
	{PCI_VENDOR_ID_ADVANTECH, 0x1724, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, adv_pci1724_pci_table);

/*
 * A convenient macro that defines init_module() and cleanup_module(),
 * as necessary.
 */
COMEDI_PCI_INITCLEANUP(adv_pci1724_driver, adv_pci1724_pci_table);

MODULE_AUTHOR("Frank Mori Hess <fmh6jj@gmail.com>");
MODULE_DESCRIPTION("Advantech PCI-1724U Comedi driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(COMEDI_RELEASE);
