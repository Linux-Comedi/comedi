/*
    module/drivers.c
    functions for manipulating drivers

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>

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

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/usb.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/comedidev.h>
#include <linux/cdev.h>
#include <linux/firmware.h>

#include <asm/io.h>

#include "comedi_internal.h"

struct comedi_device_attach_driver_context {
	comedi_devconfig *it;
	bool *matched;
};

/*
 * Set hardware device associated with comedi device.
 * For example, this is needed for devices that need to DMA directly into
 * the COMEDI data acquisition buffer (which will have been allocated from
 * DMA coherent memory).
 */
int comedi_set_hw_dev(comedi_device * dev, struct device *hw_dev)
{
	if (hw_dev == dev->hw_dev)
		return 0;
	if (dev->hw_dev)
		return -EEXIST;
	dev->hw_dev = get_device(hw_dev);
	return 0;
}

static void comedi_clear_hw_dev(comedi_device * dev)
{
	put_device(dev->hw_dev);
	dev->hw_dev = NULL;
}

void *comedi_alloc_devpriv(comedi_device *dev, size_t size)
{
	dev->private = kzalloc(size, GFP_KERNEL);
	return dev->private;
}

int comedi_alloc_subdevices(comedi_device *dev, unsigned int num_subdevices)
{
	unsigned i;

	dev->n_subdevices = num_subdevices;
	dev->subdevices =
		kcalloc(num_subdevices, sizeof(comedi_subdevice), GFP_KERNEL);
	if (!dev->subdevices)
		return -ENOMEM;
	for (i = 0; i < num_subdevices; ++i) {
		dev->subdevices[i].device = dev;
		dev->subdevices[i].async_dma_dir = DMA_NONE;
		spin_lock_init(&dev->subdevices[i].spin_lock);
		dev->subdevices[i].minor = -1;
	}
	return 0;
}

static int postconfig(comedi_device * dev);
static int insn_rw_emulate_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
static void *comedi_recognize(comedi_driver * driv, const char *name);
static void comedi_report_boards(comedi_driver * driv);
static int poll_invalid(comedi_device * dev, comedi_subdevice * s);

comedi_driver *comedi_drivers;
DEFINE_MUTEX(comedi_drivers_list_lock);

static void cleanup_device(comedi_device * dev)
{
	int i;
	comedi_subdevice *s;

	if (dev->subdevices) {
		for (i = 0; i < dev->n_subdevices; i++) {
			s = dev->subdevices + i;
			comedi_free_subdevice_minor(s);
			if (s->async) {
				comedi_buf_alloc(dev, s, 0);
				kfree(s->async);
			}
		}
		kfree(dev->subdevices);
		dev->subdevices = NULL;
		dev->n_subdevices = 0;
	}
	if (dev->private) {
		kfree(dev->private);
		dev->private = NULL;
	}
	dev->driver = 0;
	dev->board_name = NULL;
	dev->board_ptr = NULL;
	dev->iobase = 0;
	dev->irq = 0;
	dev->read_subdev = NULL;
	dev->write_subdev = NULL;
	dev->open = NULL;
	dev->close = NULL;
	comedi_clear_hw_dev(dev);
}

void comedi_device_detach_locked(comedi_device *dev)
{
	comedi_device_cancel_all(dev);
	dev->attached = 0;
	if (dev->driver) {
		dev->detach_count++;
		if (dev->driver->detach)
			dev->driver->detach(dev);
	}
	cleanup_device(dev);
}

void comedi_device_detach(comedi_device * dev)
{
	down_write(&dev->attach_lock);
	comedi_device_detach_locked(dev);
	up_write(&dev->attach_lock);
}

/*
 * Do a little post-config cleanup.
 */
static int comedi_device_postconfig(comedi_device *dev)
{
	int ret = postconfig(dev);

	if (ret < 0)
		return ret;

	down_write(&dev->attach_lock);
	dev->attached = 1;
	up_write(&dev->attach_lock);
	return 0;
}

static int comedi_device_attach_driver_wrapper(comedi_device *dev,
	comedi_driver *driv, void *context)
{
	struct comedi_device_attach_driver_context *cadc = context;
	comedi_devconfig *it = cadc->it;
	bool *matched = cadc->matched;

	if (driv->num_names) {
		/* Look for board entry matching it->board_name */
		dev->board_ptr = comedi_recognize(driv, it->board_name);
		if (dev->board_ptr == NULL)
			return -EINVAL;
	} else {
		/* Look for driver name matching it->board_name */
		if (strcmp(driv->driver_name, it->board_name))
			return -EINVAL;
	}
	/* Driver matched. */
	*matched = true;
	if (!driv->attach) {
		printk("comedi: BUG! driver '%s' has no attach handler!\n",
		       driv->driver_name);
		return -EINVAL;
	}
	dev->driver = driv;
	dev->board_name = dev->board_ptr ? *(const char * const *)dev->board_ptr
					 : dev->driver->driver_name;
	return driv->attach(dev, it);
}

static int comedi_device_attach_driver(comedi_device *dev, comedi_driver *driv,
	comedi_devconfig *it, bool *matched)
{
	struct comedi_device_attach_driver_context cadc;
	int ret = 0;

	if (!try_module_get(driv->module)) {
		printk("comedi: failed to increment module count, skipping\n");
		return -EIO;
	}
	cadc.it = it;
	cadc.matched = matched;
	/* This sets dev->driver if the driver's attach function is called. */
	ret = comedi_device_attach_driver_wrapper(dev, driv, &cadc);
	if (ret >= 0) {
		/* Do a little post-config cleanup. */
		ret = comedi_device_postconfig(dev);
	}
	if (ret < 0) {
		comedi_device_detach(dev);
		module_put(driv->module);
		return ret;
	}
	/* On success, the driver module count has been incremented. */
	return ret;
}

int comedi_device_attach(comedi_device * dev, comedi_devconfig * it)
{
	comedi_driver *driv;
	bool matched = false;
	int ret = -EIO;

	if (dev->attached)
		return -EBUSY;

	mutex_lock(&comedi_drivers_list_lock);
	for (driv = comedi_drivers; ret && !matched && driv; driv = driv->next)
		ret = comedi_device_attach_driver(dev, driv, it, &matched);

	if (ret && !matched) {
		// recognize has failed if we get here
		// report valid board names before returning error
		for (driv = comedi_drivers; driv; driv = driv->next) {
			if (!try_module_get(driv->module)) {
				printk("comedi: failed to increment module count\n");
				continue;
			}
			comedi_report_boards(driv);
			module_put(driv->module);
		}
		ret = -EIO;
	}
	mutex_unlock(&comedi_drivers_list_lock);
	return ret;
}

int comedi_driver_register(comedi_driver * driver)
{
	mutex_lock(&comedi_drivers_list_lock);
	driver->next = comedi_drivers;
	comedi_drivers = driver;
	mutex_unlock(&comedi_drivers_list_lock);

	return 0;
}

void comedi_driver_unregister(comedi_driver * driver)
{
	comedi_driver **link;
	comedi_driver *elem;
	int i;

	/* Unlink the driver. */
	mutex_lock(&comedi_drivers_list_lock);
	link = &comedi_drivers;
	while ((elem = *link) != NULL) {
		if (elem == driver) {
			*link = elem->next;
			break;
		}
		link = &elem->next;
	}
	mutex_unlock(&comedi_drivers_list_lock);

	/* Check for devices using this driver. */
	for (i = 0; i < COMEDI_NUM_BOARD_MINORS; i++) {
		comedi_device *dev = comedi_dev_get_from_minor(i);

		if(dev == NULL) continue;

		mutex_lock(&dev->mutex);
		if (dev->attached && dev->driver == driver) {
			if (dev->use_count)
				printk("BUG! detaching device with use_count=%d\n", dev->use_count);
			comedi_device_detach(dev);
		}
		mutex_unlock(&dev->mutex);
		comedi_dev_put(dev);
	}
}

int comedi_load_firmware(comedi_device *dev, struct device *hw_dev,
	const char *name, int (*cb)(comedi_device *dev, const u8 *data,
				    size_t size, unsigned long context),
	unsigned long context)
{
	const struct firmware *fw;
	int ret;

	if (!cb)
		return -EINVAL;

	ret = request_firmware(&fw, name, hw_dev);
	if (ret == 0) {
		ret = cb(dev, fw->data, fw->size, context);
		release_firmware(fw);
	}
	if (ret < 0)
		return ret;
	return 0;
}

static int postconfig(comedi_device * dev)
{
	int i;
	comedi_subdevice *s;
	comedi_async *async = NULL;
	int ret;

	for (i = 0; i < dev->n_subdevices; i++) {
		s = dev->subdevices + i;

		if (s->type == COMEDI_SUBD_UNUSED)
			continue;

		if (s->len_chanlist == 0)
			s->len_chanlist = 1;

		if (s->do_cmd) {
			BUG_ON((s->subdev_flags & (SDF_CMD_READ |
				SDF_CMD_WRITE)) == 0);
			BUG_ON(!s->do_cmdtest);

			async = kzalloc(sizeof(comedi_async), GFP_KERNEL);
			if (async == NULL) {
				printk("failed to allocate async struct\n");
				return -ENOMEM;
			}
			init_waitqueue_head(&async->wait_head);
			s->async = async;

#define DEFAULT_BUF_MAXSIZE (64*1024)
#define DEFAULT_BUF_SIZE (64*1024)

			async->max_bufsize = DEFAULT_BUF_MAXSIZE;

			async->prealloc_bufsz = 0;
			if (comedi_buf_alloc(dev, s, DEFAULT_BUF_SIZE) < 0) {
				printk("Buffer allocation failed\n");
				return -ENOMEM;
			}
			if (s->buf_change) {
				ret = s->buf_change(dev, s, DEFAULT_BUF_SIZE);
				if (ret < 0)
					return ret;
			}
			comedi_alloc_subdevice_minor(dev, s);
		}

		if (!s->range_table && !s->range_table_list)
			s->range_table = &range_unknown;

		if (!s->insn_read && s->insn_bits)
			s->insn_read = insn_rw_emulate_bits;
		if (!s->insn_write && s->insn_bits)
			s->insn_write = insn_rw_emulate_bits;

		if (!s->insn_read)
			s->insn_read = insn_inval;
		if (!s->insn_write)
			s->insn_write = insn_inval;
		if (!s->insn_bits)
			s->insn_bits = insn_inval;
		if (!s->insn_config)
			s->insn_config = insn_inval;

		if (!s->poll)
			s->poll = poll_invalid;
	}

	return 0;
}

// generic recognize function for drivers that register their supported board names
static void *comedi_recognize(comedi_driver * driv, const char *name)
{
	unsigned i;
	const char *const *name_ptr = driv->board_name;
	for (i = 0; i < driv->num_names; i++) {
		if (strcmp(*name_ptr, name) == 0)
			return (void *)name_ptr;
		name_ptr =
			(const char *const *)((const char *)name_ptr +
			driv->offset);
	}

	return NULL;
}

static void comedi_report_boards(comedi_driver * driv)
{
	unsigned int i;
	const char *const *name_ptr;

	printk("comedi: valid board names for %s driver are:\n",
		driv->driver_name);

	name_ptr = driv->board_name;
	for (i = 0; i < driv->num_names; i++) {
		printk(" %s\n", *name_ptr);
		name_ptr = (const char **)((char *)name_ptr + driv->offset);
	}

	if (driv->num_names == 0)
		printk(" %s\n", driv->driver_name);
}

static int poll_invalid(comedi_device * dev, comedi_subdevice * s)
{
	return -EINVAL;
}

int insn_inval(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	return -EINVAL;
}

static int insn_rw_emulate_bits(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	comedi_insn new_insn;
	int ret;
	static const unsigned channels_per_bitfield = 32;
	unsigned chan = CR_CHAN(insn->chanspec);
	const unsigned base_bitfield_channel =
		(chan < channels_per_bitfield) ? 0 : chan;
	lsampl_t new_data[2];
	unsigned int mask;
	unsigned int i;

	if ((insn->insn == INSN_WRITE) && !(s->subdev_flags & SDF_WRITABLE))
		return -EINVAL;

	if (insn->n == 0) 
		return 0;

	memset(new_data, 0, sizeof(new_data));
	memset(&new_insn, 0, sizeof(new_insn));
	new_insn.insn = INSN_BITS;
	new_insn.chanspec = base_bitfield_channel;
	new_insn.n = 2;
	new_insn.data = new_data;
	new_insn.subdev = insn->subdev;
	mask = 1U << (chan - base_bitfield_channel);

	for (i = 0; i < insn->n; i++) {
		if (insn->insn == INSN_WRITE) {
			/* new_data[0] = mask, new_data[1] = bits */
			new_data[0] = mask;
			new_data[1] = data[i] ? mask : 0;
		}

		ret = s->insn_bits(dev, s, &new_insn, new_data);
		if (ret < 0)
			return ret;

		if (insn->insn == INSN_READ) {
			data[i] = !!(new_data[1] & mask);
		}
	}

	return insn->n;
}

unsigned int comedi_handle_events(comedi_device *dev, comedi_subdevice *subd)
{
	unsigned int events = subd->async->events;

	if (events == 0)
		return events;

	if (events & COMEDI_CB_CANCEL_MASK)
		subd->cancel(dev, subd);

	comedi_event(dev, subd);

	return events;
}

unsigned int comedi_bytes_per_scan_cmd(const comedi_subdevice *s,
				       const comedi_cmd *cmd)
{
	unsigned int num_samples;
	unsigned int bits_per_sample;

	switch (s->type) {
	case COMEDI_SUBD_DI:
	case COMEDI_SUBD_DO:
	case COMEDI_SUBD_DIO:
		bits_per_sample = 8 * comedi_bytes_per_sample(s);
		num_samples = DIV_ROUND_UP(cmd->scan_end_arg, bits_per_sample);
		break;
	default:
		num_samples = cmd->scan_end_arg;
		break;
	}
	return comedi_samples_to_bytes(s, num_samples);
}

unsigned int comedi_bytes_per_scan(const comedi_subdevice *s)
{
	return comedi_bytes_per_scan_cmd(s, &s->async->cmd);
}

static unsigned int __comedi_nscans_left(const comedi_subdevice *s,
	unsigned int nscans)
{
	const comedi_async *async = s->async;
	const comedi_cmd *cmd = &async->cmd;

	if (cmd->stop_src == TRIG_COUNT) {
		unsigned int scans_left = 0;

		if (async->scans_done < cmd->stop_arg)
			scans_left = cmd->stop_arg - async->scans_done;

		if (nscans > scans_left)
			nscans = scans_left;
	}
	return nscans;
}

/*
 * Return number of scans left in the command up to a limit.
 *
 * If nscans is 0, it is set to the number of scans available in the async
 * buffer.  If nscans is non-zero, it is an upper limit on the number of scans
 * available.
 */
unsigned int comedi_nscans_left(const comedi_subdevice *s, unsigned int nscans)
{
	if (nscans == 0) {
		unsigned int nbytes = comedi_buf_read_n_available(s);

		nscans = nbytes / comedi_bytes_per_scan(s);
	}
	return __comedi_nscans_left(s, nscans);
}

/*
 * Return number of samples left in the command up to a limit.
 *
 * nsamples is an upper limit on the return value.
 */
unsigned int comedi_nsamples_left(const comedi_subdevice *s,
	unsigned int nsamples)
{
	const comedi_async *async = s->async;
	const comedi_cmd *cmd = &async->cmd;
	unsigned long long scans_left;
	unsigned long long samples_left;

	if (cmd->stop_src != TRIG_COUNT)
		return nsamples;

	scans_left = __comedi_nscans_left(s, cmd->stop_arg);
	if (!scans_left)
		return 0;

	samples_left = scans_left * cmd->scan_end_arg -
		comedi_bytes_to_samples(s, async->scan_progress);

	if (samples_left < nsamples)
		return samples_left;
	return nsamples;
}

void comedi_inc_scan_progress(comedi_subdevice *s, unsigned int num_bytes)
{
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	unsigned int scan_length = comedi_bytes_per_scan(s);

	/* track the 'cur_chan' for non-SDF_PACKED subdevices */
	if (!(s->subdev_flags & SDF_PACKED)) {
		async->cur_chan += comedi_bytes_to_samples(s, num_bytes);
		async->cur_chan %= cmd->chanlist_len;
	}

	async->scan_progress += num_bytes;
	if (async->scan_progress >= scan_length) {
		unsigned int nscans = async->scan_progress / scan_length;

		if (async->scans_done < (UINT_MAX - nscans))
			async->scans_done += nscans;
		else
			async->scans_done = UINT_MAX;

		async->scan_progress %= scan_length;
		async->events |= COMEDI_CB_EOS;
	}
}

static int comedi_auto_config_helper(struct device *hardware_device,
	comedi_driver *driv,
	int (*attach_wrapper)(comedi_device *, comedi_driver *, void *),
	void *context)
{
	int ret = 0;
	comedi_device *comedi_dev;

	if (!comedi_autoconfig)
		return 0;

	comedi_dev = comedi_alloc_board_minor(hardware_device);
	if (IS_ERR(comedi_dev))
		return PTR_ERR(comedi_dev);
	/* Note: comedi_alloc_board_minor() locked comedi_dev->mutex */

	/* This may set comedi_dev->driver. */
	ret = attach_wrapper(comedi_dev, driv, context);
	if (ret >= 0) {
		/* Do a little post-config cleanup. */
		ret = comedi_device_postconfig(comedi_dev);
	}
	mutex_unlock(&comedi_dev->mutex);
	if (ret < 0)
		comedi_release_hardware_device(hardware_device);
	return ret;
}

static int comedi_old_auto_config(struct device *hardware_device,
	comedi_driver *driver, const int *options, unsigned num_options)
{
	struct comedi_device_attach_driver_context cadc;
	comedi_devconfig it;
	int retval = 0;
	bool matched = false;

	memset(&it, 0, sizeof(it));
	strncpy(it.board_name, driver->driver_name, COMEDI_NAMELEN);
	it.board_name[COMEDI_NAMELEN - 1] = '\0';
	BUG_ON(num_options > COMEDI_NDEVCONFOPTS);
	memcpy(it.options, options, num_options * sizeof(int));

	cadc.it = &it;
	cadc.matched = &matched;
	retval = comedi_auto_config_helper(hardware_device, driver,
			comedi_device_attach_driver_wrapper, &cadc);
	if (retval && !matched) {
		printk("comedi: auto config failed to match '%s'\n",
		       it.board_name);
	}
	return retval;
}

static int comedi_auto_config_wrapper(comedi_device *dev, comedi_driver *driv,
	void *context)
{
	unsigned long driver_context = *(unsigned long *)context;

	if (!driv->auto_attach) {
		printk("comedi: BUG! driver '%s' has no auto_attach handler!\n",
			driv->driver_name);
		return -EINVAL;
	}
	dev->driver = driv;
	dev->board_name = dev->driver->driver_name;
	return driv->auto_attach(dev, driver_context);
}

int comedi_auto_config(struct device *hardware_device, comedi_driver *driver,
	unsigned long context)
{
	return comedi_auto_config_helper(hardware_device, driver,
		comedi_auto_config_wrapper, &context);
}

void comedi_auto_unconfig(struct device *hardware_device)
{
	if (hardware_device == NULL)
		return;
	comedi_release_hardware_device(hardware_device);
}

static int comedi_new_pci_auto_config(struct pci_dev *pcidev,
	comedi_driver *driver, unsigned long context)
{
	return comedi_auto_config(&pcidev->dev, driver, context);
}

static int comedi_old_pci_auto_config(struct pci_dev *pcidev,
	comedi_driver *driver, unsigned long context)
{
	int options[2];

	// pci bus
	options[0] = pcidev->bus->number;
	// pci slot
	options[1] = PCI_SLOT(pcidev->devfn);

	return comedi_old_auto_config(&pcidev->dev, driver,
		options, ARRAY_SIZE(options));
}

int comedi_pci_auto_config(struct pci_dev *pcidev, comedi_driver *driver,
	unsigned long context)
{
	if (driver->auto_attach)
		return comedi_new_pci_auto_config(pcidev, driver, context);
	else
		return comedi_old_pci_auto_config(pcidev, driver, context);
}

void comedi_pci_auto_unconfig(struct pci_dev *pcidev)
{
	comedi_auto_unconfig(&pcidev->dev);
}

static int comedi_usb_new_auto_config(struct usb_interface *intf,
	comedi_driver *driver, unsigned long context)
{
	return comedi_auto_config(&intf->dev, driver, context);
}

static int comedi_usb_old_auto_config(struct usb_interface *intf,
	comedi_driver *driver, unsigned long context)
{
	return comedi_old_auto_config(&intf->dev, driver, NULL, 0);
}

int comedi_usb_auto_config(struct usb_interface *intf, comedi_driver *driver,
	unsigned long context)
{
	BUG_ON(intf == NULL);
	if (driver->auto_attach)
		return comedi_usb_new_auto_config(intf, driver, context);
	else
		return comedi_usb_old_auto_config(intf, driver, context);
}

void comedi_usb_auto_unconfig(struct usb_interface *intf)
{
	BUG_ON(intf == NULL);
	comedi_auto_unconfig(&intf->dev);
}

struct pci_dev *comedi_to_pci_dev(comedi_device *dev)
{
	return dev->hw_dev ? to_pci_dev(dev->hw_dev) : NULL;
}

struct usb_interface *comedi_to_usb_interface(comedi_device *dev)
{
	return dev->hw_dev ? to_usb_interface(dev->hw_dev) : NULL;
}

struct usb_device *comedi_to_usb_dev(comedi_device *dev)
{
	struct usb_interface *intf = comedi_to_usb_interface(dev);

	return intf ? interface_to_usbdev(intf) : NULL;
}
