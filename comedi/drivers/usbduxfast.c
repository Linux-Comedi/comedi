#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "Bernd Porr, BerndPorr@f2s.com"
#define DRIVER_DESC "USB-DUXfast, BerndPorr@f2s.com"
/*
   comedi/drivers/usbduxfast.c
   Copyright (C) 2004 Bernd Porr, Bernd.Porr@f2s.com

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
Driver: usbduxfast
Description: Driver for USB-DUX-FAST of INCITE Technology Limited
Devices: [ITL] USB-DUX-FAST (usbduxfast)
Author: Bernd Porr <tech@linux-usb-daq.co.uk>
Updated: 14 May 2012
Status: stable

The device has one subdevice for analogue input.
  - subdevice: 0
    number of channels: 16
    max data value: 4096
    ranges:
      all channelss: 
        range = 0 : [-0.75 V,0.75 V] 
        range = 1 : [-0.5 V,0.5 V]
    command:
      The channel-list allows 1,2,3 and 16 channels.
      start: now|ext|int (external trigger via pin at HD-D connector)
      scan_begin: follow|timer|ext
      convert: timer|ext (contains the sampling interval. Min interval
                          for single channel acquisition is 33us 
                          and for multiplexed acquisition 300us)
      scan_end: count
      stop: none|count

Configuration options:
  The device requires firmware which is usually
  uploaded automatically by udev/hotplug at the moment
  the driver module is being loaded.
  In case udev/hotplug is not enabled you need to upload 
  the firmware with comedi_config -i usbduxfast_firmware.bin
  The firmware is usually installed under /lib/firmware
  or can be downloaded form http://www.linux-usb-daq.co.uk.
*/

/*
 * I must give credit here to Chris Baugher who
 * wrote the driver for AT-MIO-16d. I used some parts of this
 * driver. I also must give credits to David Brownell
 * who supported me with the USB development.
 *
 * Bernd Porr
 *
 *
 * Revision history:
 * 0.9: Dropping the first data packet which seems to be from the last transfer.
 *      Buffer overflows in the FX2 are handed over to comedi.
 * 0.92: Dropping now 4 packets. The quad buffer has to be emptied.
 *       Added insn command basically for testing. Sample rate is 1MHz/16ch=62.5kHz
 * 0.99: Ian Abbott pointed out a bug which has been corrected. Thanks!
 * 0.99a: added external trigger.
 * 1.00: added firmware kernel request to the driver which fixed
 *       udev coldplug problem
 */

#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/fcntl.h>
#include <linux/compiler.h>
#include <linux/comedidev.h>
#include <linux/usb.h>

// (un)comment this if you want to have debug info.
//#define COMEDI_CONFIG_DEBUG
#undef  COMEDI_CONFIG_DEBUG

#define BOARDNAME "usbduxfast"

// timeout for the USB-transfer
#define BULK_TIMEOUT 1000

// constants for "firmware" upload and download
#define USBDUXFASTSUB_FIRMWARE 0xA0
#define VENDOR_DIR_IN  0xC0
#define VENDOR_DIR_OUT 0x40

// internal adresses of the 8051 processor
#define USBDUXFASTSUB_CPUCS 0xE600

// max lenghth of the transfer-buffer for software upload
#define TB_LEN 0x2000

// Input endpoint number
#define BULKINEP           6

// Endpoint for the A/D channellist: bulk OUT
#define CHANNELLISTEP     4

// Number of channels
#define NUMCHANNELS       32

// size of the waveform descriptor
#define WAVESIZE          0x20

// Size of one A/D value
#define SIZEADIN          ((sizeof(int16_t)))

// Size of the input-buffer IN BYTES
#define SIZEINBUF         512

// 16 bytes.
#define SIZEINSNBUF       512

// Size of the buffer for the dux commands
#define SIZEOFDUXBUFFER    256	// bytes

// Number of in-URBs which receive the data: min=5
#define NUMOFINBUFFERSHIGH     10

// Total number of usbduxfast devices
#define NUMUSBDUXFAST             16

// Number of subdevices
#define N_SUBDEVICES          1

// Analogue in subdevice
#define SUBDEV_AD             0

// min delay steps for more than one channel
// basically when the mux gives up. ;-)
#define MIN_SAMPLING_PERIOD 9	// steps at 30MHz in the FX2

// Max number of 1/30MHz delay steps:
#define MAX_SAMPLING_PERIOD 500

// Number of received packets to ignore before we start handing data over to comedi.
// It's quad buffering and we have to ignore 4 packets.
#define PACKETS_TO_IGNORE 4

/////////////////////////////////////////////
// comedi constants
static const comedi_lrange range_usbduxfast_ai_range = { 2, {
			BIP_RANGE(0.75),
			BIP_RANGE(0.5),
	}
};

/*
 * private structure of one subdevice
 */

// This is the structure which holds all the data of this driver
// one sub device just now: A/D
typedef struct {
	// attached?
	int attached;
	// is it associated with a subdevice?
	int probed;
	// pointer to the usb-device
	struct usb_device *usbdev;
	// BULK-transfer handling: urb
	struct urb *urbIn;
	int8_t *transfer_buffer;
	// input buffer for single insn
	int16_t *insnBuffer;
	// interface number
	int ifnum;
	// interface structure in 2.6
	struct usb_interface *interface;
	// comedi device for the interrupt context
	comedi_device *comedidev;
	// asynchronous command is running
	short int ai_cmd_running;
	// continous aquisition
	short int ai_continous;
	// number of samples to aquire
	long int ai_sample_count;
	// commands
	uint8_t *dux_commands;
	// counter which ignores the first buffers
	int ignore;
	struct mutex mutex;
} usbduxfastsub_t;

// The pointer to the private usb-data of the driver
// is also the private data for the comedi-device.
// This has to be global as the usb subsystem needs
// global variables. The other reason is that this
// structure must be there _before_ any comedi
// command is issued. The usb subsystem must be
// initialised before comedi can access it.
static usbduxfastsub_t usbduxfastsub[NUMUSBDUXFAST];

static DEFINE_MUTEX(start_stop_mutex);

static comedi_driver driver_usbduxfast; /* See below for initializer. */

// bulk transfers to usbduxfast

#define SENDADCOMMANDS            0
#define SENDINITEP6               1

static int send_dux_commands(usbduxfastsub_t * this_usbduxfastsub, int cmd_type)
{
	int result, nsent;
	this_usbduxfastsub->dux_commands[0] = cmd_type;
#ifdef COMEDI_CONFIG_DEBUG
	int i;
	printk("comedi%d: usbduxfast: dux_commands: ",
		this_usbduxfastsub->comedidev->minor);
	for (i = 0; i < SIZEOFDUXBUFFER; i++) {
		printk(KERN_CONT " %02x", this_usbduxfastsub->dux_commands[i]);
	}
	printk(KERN_CONT "\n");
#endif
	result = usb_bulk_msg(this_usbduxfastsub->usbdev,
		usb_sndbulkpipe(this_usbduxfastsub->usbdev,
			CHANNELLISTEP),
		this_usbduxfastsub->dux_commands,
		SIZEOFDUXBUFFER, &nsent, BULK_TIMEOUT);
	if (result < 0) {
		printk("comedi%d: could not transmit dux_commands to the usb-device, err=%d\n", this_usbduxfastsub->comedidev->minor, result);
	}
	return result;
}

// Stops the data acquision
// It should be safe to call this function from any context
static int usbduxfastsub_unlink_InURBs(usbduxfastsub_t * usbduxfastsub_tmp)
{
	int j = 0;
	int err = 0;

	if (usbduxfastsub_tmp && usbduxfastsub_tmp->urbIn) {
		usbduxfastsub_tmp->ai_cmd_running = 0;
		// waits until a running transfer is over
		usb_kill_urb(usbduxfastsub_tmp->urbIn);
		j = 0;
	}
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi: usbduxfast: unlinked InURB: res=%d\n", j);
#endif
	return err;
}

/* This will stop a running acquisition operation */
// Is called from within this driver from both the
// interrupt context and from comedi
static int usbduxfast_ai_stop(usbduxfastsub_t * this_usbduxfastsub,
	int do_unlink)
{
	int ret = 0;

	if (!this_usbduxfastsub) {
		printk("comedi?: usbduxfast_ai_stop: this_usbduxfastsub=NULL!\n");
		return -EFAULT;
	}
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi: usbduxfast_ai_stop\n");
#endif

	this_usbduxfastsub->ai_cmd_running = 0;

	if (do_unlink) {
		// stop aquistion
		ret = usbduxfastsub_unlink_InURBs(this_usbduxfastsub);
	}

	return ret;
}

// This will cancel a running acquisition operation.
// This is called by comedi but never from inside the
// driver.
static int usbduxfast_ai_cancel(comedi_device * dev, comedi_subdevice * s)
{
	usbduxfastsub_t *this_usbduxfastsub;
	int res = 0;

	// force unlink of all urbs
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi: usbduxfast_ai_cancel\n");
#endif
	this_usbduxfastsub = dev->private;
	if (!this_usbduxfastsub) {
		printk("comedi: usbduxfast_ai_cancel: this_usbduxfastsub=NULL\n");
		return -EFAULT;
	}
	mutex_lock(&this_usbduxfastsub->mutex);
	if (!(this_usbduxfastsub->probed)) {
		mutex_unlock(&this_usbduxfastsub->mutex);
		return -ENODEV;
	}
	// unlink
	res = usbduxfast_ai_stop(this_usbduxfastsub, 1);
	mutex_unlock(&this_usbduxfastsub->mutex);

	return res;
}

// analogue IN
// interrupt service routine
static void usbduxfastsub_ai_Irq(struct urb *urb PT_REGS_ARG)
{
	int n, err;
	usbduxfastsub_t *this_usbduxfastsub;
	comedi_device *this_comedidev;
	comedi_subdevice *s;
	uint16_t *p;

	// sanity checks
	// is the urb there?
	if (!urb) {
		printk("comedi_: usbduxfast_: ao int-handler called with urb=NULL!\n");
		return;
	}
	// the context variable points to the subdevice
	this_comedidev = urb->context;
	if (!this_comedidev) {
		printk("comedi_: usbduxfast_: urb context is a NULL pointer!\n");
		return;
	}
	// the private structure of the subdevice is usbduxfastsub_t
	this_usbduxfastsub = this_comedidev->private;
	if (!this_usbduxfastsub) {
		printk("comedi_: usbduxfast_: private of comedi subdev is a NULL pointer!\n");
		return;
	}
	// are we running a command?
	if (unlikely(!(this_usbduxfastsub->ai_cmd_running))) {
		// not running a command
		// do not continue execution if no asynchronous command is running
		// in particular not resubmit
		return;
	}

	if (unlikely(!(this_usbduxfastsub->attached))) {
		// no comedi device there
		return;
	}
	// subdevice which is the AD converter
	s = this_comedidev->subdevices + SUBDEV_AD;

	// first we test if something unusual has just happened
	switch (urb->status) {
	case 0:
		break;

		// happens after an unlink command or when the device is plugged out
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
	case -ECONNABORTED:
		// tell this comedi
		s->async->events |= COMEDI_CB_EOA;
		s->async->events |= COMEDI_CB_ERROR;
		comedi_event(this_usbduxfastsub->comedidev, s);
		// stop the transfer w/o unlink
		usbduxfast_ai_stop(this_usbduxfastsub, 0);
		return;

	default:
		printk("comedi%d: usbduxfast: non-zero urb status received in ai intr context: %d\n", this_usbduxfastsub->comedidev->minor, urb->status);
		s->async->events |= COMEDI_CB_EOA;
		s->async->events |= COMEDI_CB_ERROR;
		comedi_event(this_usbduxfastsub->comedidev, s);
		usbduxfast_ai_stop(this_usbduxfastsub, 0);
		return;
	}

	p = urb->transfer_buffer;
	if (!this_usbduxfastsub->ignore) {
		if (!(this_usbduxfastsub->ai_continous)) {
			// not continous, fixed number of samples
			n = urb->actual_length / sizeof(uint16_t);
			if (unlikely(this_usbduxfastsub->ai_sample_count < n)) {
				// we have send only a fraction of the bytes received
				comedi_buf_write_samples(s,
					urb->transfer_buffer,
					this_usbduxfastsub->ai_sample_count);
				usbduxfast_ai_stop(this_usbduxfastsub, 0);
				// say comedi that the acquistion is over
				s->async->events |= COMEDI_CB_EOA;
				comedi_event(this_usbduxfastsub->comedidev, s);
				return;
			}
			this_usbduxfastsub->ai_sample_count -= n;
		}
		// write the full buffer to comedi
		err = comedi_buf_write_samples(s,
			urb->transfer_buffer,
			urb->actual_length / sizeof(uint16_t));

		if (unlikely(err == 0)) {
			/* buffer overflow */
			usbduxfast_ai_stop(this_usbduxfastsub, 0);
			return;
		}

		// tell comedi that data is there
		comedi_event(this_usbduxfastsub->comedidev, s);

	} else {
		// ignore this packet
		this_usbduxfastsub->ignore--;
	}

	// command is still running
	// resubmit urb for BULK transfer
	urb->dev = this_usbduxfastsub->usbdev;
	urb->status = 0;
	if ((err = usb_submit_urb(urb, GFP_ATOMIC)) < 0) {
		printk("comedi%d: usbduxfast: urb resubm failed: %d\n",
			this_usbduxfastsub->comedidev->minor, err);
		s->async->events |= COMEDI_CB_EOA;
		s->async->events |= COMEDI_CB_ERROR;
		comedi_event(this_usbduxfastsub->comedidev, s);
		usbduxfast_ai_stop(this_usbduxfastsub, 0);
	}
}

static int usbduxfastsub_start(usbduxfastsub_t * usbduxfastsub)
{
	int errcode = 0;
	unsigned char *local_transfer_buffer;

	local_transfer_buffer = kmalloc(16, GFP_KERNEL);
	if (!local_transfer_buffer)
		return -ENOMEM;

	// 7f92 to zero
	local_transfer_buffer[0] = 0;
	errcode = usb_control_msg(usbduxfastsub->usbdev,
				  // create a pipe for a control transfer
				  usb_sndctrlpipe(usbduxfastsub->usbdev, 0),
				  // bRequest, "Firmware"
				  USBDUXFASTSUB_FIRMWARE,
				  // bmRequestType
				  VENDOR_DIR_OUT,
				  // Value
				  USBDUXFASTSUB_CPUCS,
				  // Index
				  0x0000,
				  // address of the transfer buffer
				  local_transfer_buffer,
				  // Length
				  1,
				  // Timeout
				  BULK_TIMEOUT);
	if (errcode < 0)
		printk("comedi_: usbduxfast_: control msg failed (start)\n");

	kfree(local_transfer_buffer);
	return errcode;
}

static int usbduxfastsub_stop(usbduxfastsub_t * usbduxfastsub)
{
	int errcode = 0;
	unsigned char *local_transfer_buffer;

	local_transfer_buffer = kmalloc(16, GFP_KERNEL);
	if (!local_transfer_buffer)
		return -ENOMEM;

	// 7f92 to one
	local_transfer_buffer[0] = 1;
	errcode = usb_control_msg
		(usbduxfastsub->usbdev,
		 usb_sndctrlpipe(usbduxfastsub->usbdev, 0),
		 // bRequest, "Firmware"
		 USBDUXFASTSUB_FIRMWARE,
		 // bmRequestType
		 VENDOR_DIR_OUT,
		 // Value
		 USBDUXFASTSUB_CPUCS,
		 // Index
		 0x0000, local_transfer_buffer,
		 // Length
		 1,
		 // Timeout
		 BULK_TIMEOUT);
	if (errcode < 0)
		printk("comedi_: usbduxfast: control msg failed (stop)\n");

	kfree(local_transfer_buffer);
	return errcode;
}

static int usbduxfastsub_upload(usbduxfastsub_t * usbduxfastsub,
	unsigned char *local_transfer_buffer,
	unsigned int startAddr, unsigned int len)
{
	int errcode;

	errcode = usb_control_msg
		(usbduxfastsub->usbdev,
		 usb_sndctrlpipe(usbduxfastsub->usbdev, 0),
		 // brequest, firmware
		 USBDUXFASTSUB_FIRMWARE,
		 // bmRequestType
		 VENDOR_DIR_OUT,
		 // value
		 startAddr,
		 // index
		 0x0000,
		 // our local safe buffer
		 local_transfer_buffer,
		 // length
		 len,
		 // timeout
		 BULK_TIMEOUT);
	if (errcode < 0) {
		printk("comedi_: usbduxfast: uppload failed\n");
		return errcode;
	}
	return 0;
}

static int firmwareUpload(usbduxfastsub_t * usbduxfastsub,
	unsigned char *firmwareBinary, int sizeFirmware)
{
	int ret;

	if (!firmwareBinary) {
		return 0;
	}
	ret = usbduxfastsub_stop(usbduxfastsub);
	if (ret < 0) {
		printk("comedi_: usbduxfast: can not stop firmware\n");
		return ret;
	}
	ret = usbduxfastsub_upload(usbduxfastsub,
		firmwareBinary, 0, sizeFirmware);
	if (ret < 0) {
		printk("comedi_: usbduxfast: firmware upload failed\n");
		return ret;
	}
	ret = usbduxfastsub_start(usbduxfastsub);
	if (ret < 0) {
		printk("comedi_: usbduxfast: can not start firmware\n");
		return ret;
	}
	return 0;
}

static int usbduxfastsub_submit_InURBs(usbduxfastsub_t * usbduxfastsub)
{
	int errFlag;

	if (!usbduxfastsub) {
		return -EFAULT;
	}
	usb_fill_bulk_urb(usbduxfastsub->urbIn,
		usbduxfastsub->usbdev,
		usb_rcvbulkpipe(usbduxfastsub->usbdev, BULKINEP),
		usbduxfastsub->transfer_buffer,
		SIZEINBUF, usbduxfastsub_ai_Irq, usbduxfastsub->comedidev);

#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi%d: usbduxfast: submitting in-urb: %x,%x\n",
		usbduxfastsub->comedidev->minor,
		(int)(usbduxfastsub->urbIn->context),
		(int)(usbduxfastsub->urbIn->dev));
#endif
	errFlag = usb_submit_urb(usbduxfastsub->urbIn, GFP_ATOMIC);
	if (errFlag) {
		printk("comedi_: usbduxfast: ai: ");
		printk(KERN_CONT "usb_submit_urb");
		printk(KERN_CONT " error %d\n", errFlag);
		return errFlag;
	}
	return 0;
}

static int usbduxfast_ai_cmdtest(comedi_device * dev,
	comedi_subdevice * s, comedi_cmd * cmd)
{
	int err = 0, stop_mask = 0;
	long int steps, tmp = 0;
	int minSamplPer;
	usbduxfastsub_t *this_usbduxfastsub = dev->private;
	if (!(this_usbduxfastsub->probed)) {
		return -ENODEV;
	}
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi%d: usbduxfast_ai_cmdtest\n", dev->minor);
	printk("comedi%d: usbduxfast: convert_arg=%u scan_begin_arg=%u\n",
		dev->minor, cmd->convert_arg, cmd->scan_begin_arg);
#endif
	/* step 1: make sure trigger sources are trivially valid */

	tmp = cmd->start_src;
	cmd->start_src &= TRIG_NOW | TRIG_EXT | TRIG_INT;
	if (!cmd->start_src || tmp != cmd->start_src)
		err++;

	tmp = cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER | TRIG_FOLLOW | TRIG_EXT;
	if (!cmd->scan_begin_src || tmp != cmd->scan_begin_src)
		err++;

	tmp = cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER | TRIG_EXT;
	if (!cmd->convert_src || tmp != cmd->convert_src)
		err++;

	tmp = cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if (!cmd->scan_end_src || tmp != cmd->scan_end_src)
		err++;

	tmp = cmd->stop_src;
	stop_mask = TRIG_COUNT | TRIG_NONE;
	cmd->stop_src &= stop_mask;
	if (!cmd->stop_src || tmp != cmd->stop_src)
		err++;

	if (err)
		return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if (cmd->start_src != TRIG_NOW &&
		cmd->start_src != TRIG_EXT && cmd->start_src != TRIG_INT)
		err++;
	if (cmd->scan_begin_src != TRIG_TIMER &&
		cmd->scan_begin_src != TRIG_FOLLOW &&
		cmd->scan_begin_src != TRIG_EXT)
		err++;
	if (cmd->convert_src != TRIG_TIMER && cmd->convert_src != TRIG_EXT)
		err++;
	if (cmd->stop_src != TRIG_COUNT &&
		cmd->stop_src != TRIG_EXT && cmd->stop_src != TRIG_NONE)
		err++;

	// can't have external stop and start triggers at once
	if (cmd->start_src == TRIG_EXT && cmd->stop_src == TRIG_EXT)
		err++;

	if (err)
		return 2;

	/* step 3: make sure arguments are trivially compatible */

	if (cmd->start_src == TRIG_NOW && cmd->start_arg != 0) {
		cmd->start_arg = 0;
		err++;
	}

	if (!cmd->chanlist_len) {
		err++;
	}
	if (cmd->scan_end_arg != cmd->chanlist_len) {
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}

	if (cmd->chanlist_len == 1) {
		minSamplPer = 1;
	} else {
		minSamplPer = MIN_SAMPLING_PERIOD;
	}

	if (cmd->convert_src == TRIG_TIMER) {
		steps = cmd->convert_arg * 30;
		if (steps < (minSamplPer * 1000)) {
			steps = minSamplPer * 1000;
		}
		if (steps > (MAX_SAMPLING_PERIOD * 1000)) {
			steps = MAX_SAMPLING_PERIOD * 1000;
		}
		// calc arg again
		tmp = steps / 30;
		if (cmd->convert_arg != tmp) {
			cmd->convert_arg = tmp;
			err++;
		}
	}

	if (cmd->scan_begin_src == TRIG_TIMER) {
		err++;
	}
	// stop source
	switch (cmd->stop_src) {
	case TRIG_COUNT:
		if (!cmd->stop_arg) {
			cmd->stop_arg = 1;
			err++;
		}
		break;
	case TRIG_NONE:
		if (cmd->stop_arg != 0) {
			cmd->stop_arg = 0;
			err++;
		}
		break;
		// TRIG_EXT doesn't care since it doesn't trigger off a numbered channel
	default:
		break;
	}

	if (err)
		return 3;

	/* step 4: fix up any arguments */

	return 0;

}

static int usbduxfast_ai_inttrig(comedi_device * dev,
	comedi_subdevice * s, unsigned int trignum)
{
	int ret;
	usbduxfastsub_t *this_usbduxfastsub = dev->private;
	if (!this_usbduxfastsub) {
		return -EFAULT;
	}
	mutex_lock(&this_usbduxfastsub->mutex);
	if (!(this_usbduxfastsub->probed)) {
		mutex_unlock(&this_usbduxfastsub->mutex);
		return -ENODEV;
	}
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi%d: usbduxfast_ai_inttrig\n", dev->minor);
#endif

	if (trignum != 0) {
		printk("comedi%d: usbduxfast_ai_inttrig: invalid trignum\n",
			dev->minor);
		mutex_unlock(&this_usbduxfastsub->mutex);
		return -EINVAL;
	}
	if (!(this_usbduxfastsub->ai_cmd_running)) {
		this_usbduxfastsub->ai_cmd_running = 1;
		ret = usbduxfastsub_submit_InURBs(this_usbduxfastsub);
		if (ret < 0) {
			printk("comedi%d: usbduxfast_ai_inttrig: urbSubmit: err=%d\n", dev->minor, ret);
			this_usbduxfastsub->ai_cmd_running = 0;
			mutex_unlock(&this_usbduxfastsub->mutex);
			return ret;
		}
		s->async->inttrig = NULL;
	} else {
		printk("comedi%d: ai_inttrig but acqu is already running\n",
			dev->minor);
	}
	mutex_unlock(&this_usbduxfastsub->mutex);
	return 1;
}

// offsets for the GPIF bytes
// the first byte is the command byte
#define LENBASE 1+0x00
#define OPBASE  1+0x08
#define OUTBASE 1+0x10
#define LOGBASE 1+0x18

static int usbduxfast_ai_cmd(comedi_device * dev, comedi_subdevice * s)
{
	comedi_cmd *cmd = &s->async->cmd;
	unsigned int chan, gain, rngmask = 0xff;
	int i, j, ret;
	usbduxfastsub_t *this_usbduxfastsub = dev->private;
	int result;
	long steps, steps_tmp;

#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi%d: usbduxfast_ai_cmd\n", dev->minor);
#endif
	if (!this_usbduxfastsub) {
		return -EFAULT;
	}
	mutex_lock(&this_usbduxfastsub->mutex);
	if (!(this_usbduxfastsub->probed)) {
		mutex_unlock(&this_usbduxfastsub->mutex);
		return -ENODEV;
	}
	if (this_usbduxfastsub->ai_cmd_running) {
		printk("comedi%d: ai_cmd not possible. Another ai_cmd is running.\n", dev->minor);
		mutex_unlock(&this_usbduxfastsub->mutex);
		return -EBUSY;
	}
	// set current channel of the running aquisition to zero
	s->async->cur_chan = 0;

	// ignore the first buffers from the device if there is an error condition
	this_usbduxfastsub->ignore = PACKETS_TO_IGNORE;

	if (cmd->chanlist_len > 0) {
		gain = CR_RANGE(cmd->chanlist[0]);
		for (i = 0; i < cmd->chanlist_len; ++i) {
			chan = CR_CHAN(cmd->chanlist[i]);
			if (chan != i) {
				printk("comedi%d: cmd is accepting only consecutive channels.\n", dev->minor);
				mutex_unlock(&this_usbduxfastsub->mutex);
				return -EINVAL;
			}
			if ((gain != CR_RANGE(cmd->chanlist[i]))
				&& (cmd->chanlist_len > 3)) {
				printk("comedi%d: the gain must be the same for all channels.\n", dev->minor);
				mutex_unlock(&this_usbduxfastsub->mutex);
				return -EINVAL;
			}
			if (i >= NUMCHANNELS) {
				printk("comedi%d: channel list too long\n",
					dev->minor);
				break;
			}
		}
	}
	steps = 0;
	if (cmd->scan_begin_src == TRIG_TIMER) {
		printk("comedi%d: usbduxfast: scan_begin_src==TRIG_TIMER not valid.\n", dev->minor);
		mutex_unlock(&this_usbduxfastsub->mutex);
		return -EINVAL;
	}
	if (cmd->convert_src == TRIG_TIMER) {
		steps = (cmd->convert_arg * 30) / 1000;
	}
	if ((steps < MIN_SAMPLING_PERIOD) && (cmd->chanlist_len != 1)) {
		printk("comedi%d: usbduxfast: ai_cmd: steps=%ld, scan_begin_arg=%d. Not properly tested by cmdtest?\n", dev->minor, steps, cmd->scan_begin_arg);
		mutex_unlock(&this_usbduxfastsub->mutex);
		return -EINVAL;
	}
	if (steps > MAX_SAMPLING_PERIOD) {
		printk("comedi%d: usbduxfast: ai_cmd: sampling rate too low.\n",
			dev->minor);
		mutex_unlock(&this_usbduxfastsub->mutex);
		return -EINVAL;
	}
	if ((cmd->start_src == TRIG_EXT) && (cmd->chanlist_len != 1)
		&& (cmd->chanlist_len != 16)) {
		printk("comedi%d: usbduxfast: ai_cmd: TRIG_EXT only with 1 or 16 channels possible.\n", dev->minor);
		mutex_unlock(&this_usbduxfastsub->mutex);
		return -EINVAL;
	}
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi%d: usbduxfast: steps=%ld, convert_arg=%u, ai_timer=%u\n",
		dev->minor,
		steps, cmd->convert_arg, this_usbduxfastsub->ai_timer);
#endif

	switch (cmd->chanlist_len) {
		// one channel
	case 1:
		if (CR_RANGE(cmd->chanlist[0]) > 0)
			rngmask = 0xff - 0x04;
		else
			rngmask = 0xff;

		// for external trigger: looping in this state until the RDY0 pin
		// becomes zero
		if (cmd->start_src == TRIG_EXT) {	// we loop here until ready has been set
			this_usbduxfastsub->dux_commands[LENBASE + 0] = 0x01;	// branch back to state 0
			this_usbduxfastsub->dux_commands[OPBASE + 0] = 0x01;	// deceision state w/o data
			this_usbduxfastsub->dux_commands[OUTBASE + 0] =
				0xFF & rngmask;
			this_usbduxfastsub->dux_commands[LOGBASE + 0] = 0x00;	// RDY0 = 0
		} else {	// we just proceed to state 1
			this_usbduxfastsub->dux_commands[LENBASE + 0] = 1;
			this_usbduxfastsub->dux_commands[OPBASE + 0] = 0;
			this_usbduxfastsub->dux_commands[OUTBASE + 0] =
				0xFF & rngmask;
			this_usbduxfastsub->dux_commands[LOGBASE + 0] = 0;
		}

		if (steps < MIN_SAMPLING_PERIOD) {
			// for fast single channel aqu without mux
			if (steps <= 1) {
				// we just stay here at state 1 and rexecute the same state
				// this gives us 30MHz sampling rate
				this_usbduxfastsub->dux_commands[LENBASE + 1] = 0x89;	// branch back to state 1
				this_usbduxfastsub->dux_commands[OPBASE + 1] = 0x03;	// deceision state with data
				this_usbduxfastsub->dux_commands[OUTBASE + 1] =
					0xFF & rngmask;
				this_usbduxfastsub->dux_commands[LOGBASE + 1] = 0xFF;	// doesn't matter
			} else {
				// we loop through two states: data and delay: max rate is 15Mhz
				this_usbduxfastsub->dux_commands[LENBASE + 1] =
					steps - 1;
				this_usbduxfastsub->dux_commands[OPBASE + 1] = 0x02;	// data
				this_usbduxfastsub->dux_commands[OUTBASE + 1] =
					0xFF & rngmask;
				this_usbduxfastsub->dux_commands[LOGBASE + 1] = 0;	// doesn't matter

				this_usbduxfastsub->dux_commands[LENBASE + 2] = 0x09;	// branch back to state 1
				this_usbduxfastsub->dux_commands[OPBASE + 2] = 0x01;	// deceision state w/o data
				this_usbduxfastsub->dux_commands[OUTBASE + 2] =
					0xFF & rngmask;
				this_usbduxfastsub->dux_commands[LOGBASE + 2] = 0xFF;	// doesn't matter
			}
		} else {
			// we loop through 3 states: 2x delay and 1x data. This gives a min
			// sampling rate of 60kHz.

			// we have 1 state with duration 1
			steps = steps - 1;

			// do the first part of the delay
			this_usbduxfastsub->dux_commands[LENBASE + 1] =
				steps / 2;
			this_usbduxfastsub->dux_commands[OPBASE + 1] = 0;
			this_usbduxfastsub->dux_commands[OUTBASE + 1] =
				0xFF & rngmask;
			this_usbduxfastsub->dux_commands[LOGBASE + 1] = 0;

			// and the second part
			this_usbduxfastsub->dux_commands[LENBASE + 2] =
				steps - steps / 2;
			this_usbduxfastsub->dux_commands[OPBASE + 2] = 0;
			this_usbduxfastsub->dux_commands[OUTBASE + 2] =
				0xFF & rngmask;
			this_usbduxfastsub->dux_commands[LOGBASE + 2] = 0;

			// get the data and branch back
			this_usbduxfastsub->dux_commands[LENBASE + 3] = 0x09;	// branch back to state 1
			this_usbduxfastsub->dux_commands[OPBASE + 3] = 0x03;	// deceision state w data
			this_usbduxfastsub->dux_commands[OUTBASE + 3] =
				0xFF & rngmask;
			this_usbduxfastsub->dux_commands[LOGBASE + 3] = 0xFF;	// doesn't matter
		}
		break;

	case 2:
		// two channels
		// commit data to the FIFO
		if (CR_RANGE(cmd->chanlist[0]) > 0)
			rngmask = 0xff - 0x04;
		else
			rngmask = 0xff;
		this_usbduxfastsub->dux_commands[LENBASE + 0] = 1;
		this_usbduxfastsub->dux_commands[OPBASE + 0] = 0x02;	// data
		this_usbduxfastsub->dux_commands[OUTBASE + 0] = 0xFF & rngmask;
		this_usbduxfastsub->dux_commands[LOGBASE + 0] = 0;

		// we have 1 state with duration 1: state 0
		steps_tmp = steps - 1;

		if (CR_RANGE(cmd->chanlist[1]) > 0)
			rngmask = 0xff - 0x04;
		else
			rngmask = 0xff;
		// do the first part of the delay
		this_usbduxfastsub->dux_commands[LENBASE + 1] = steps_tmp / 2;
		this_usbduxfastsub->dux_commands[OPBASE + 1] = 0;
		this_usbduxfastsub->dux_commands[OUTBASE + 1] = 0xFE & rngmask;	//count
		this_usbduxfastsub->dux_commands[LOGBASE + 1] = 0;

		// and the second part
		this_usbduxfastsub->dux_commands[LENBASE + 2] =
			steps_tmp - steps_tmp / 2;
		this_usbduxfastsub->dux_commands[OPBASE + 2] = 0;
		this_usbduxfastsub->dux_commands[OUTBASE + 2] = 0xFF & rngmask;
		this_usbduxfastsub->dux_commands[LOGBASE + 2] = 0;

		this_usbduxfastsub->dux_commands[LENBASE + 3] = 1;
		this_usbduxfastsub->dux_commands[OPBASE + 3] = 0x02;	// data
		this_usbduxfastsub->dux_commands[OUTBASE + 3] = 0xFF & rngmask;
		this_usbduxfastsub->dux_commands[LOGBASE + 3] = 0;

		// we have 2 states with duration 1: step 6 and the IDLE state
		steps_tmp = steps - 2;

		if (CR_RANGE(cmd->chanlist[0]) > 0)
			rngmask = 0xff - 0x04;
		else
			rngmask = 0xff;
		// do the first part of the delay
		this_usbduxfastsub->dux_commands[LENBASE + 4] = steps_tmp / 2;
		this_usbduxfastsub->dux_commands[OPBASE + 4] = 0;
		this_usbduxfastsub->dux_commands[OUTBASE + 4] = (0xFF - 0x02) & rngmask;	//reset
		this_usbduxfastsub->dux_commands[LOGBASE + 4] = 0;

		// and the second part
		this_usbduxfastsub->dux_commands[LENBASE + 5] =
			steps_tmp - steps_tmp / 2;
		this_usbduxfastsub->dux_commands[OPBASE + 5] = 0;
		this_usbduxfastsub->dux_commands[OUTBASE + 5] = 0xFF & rngmask;
		this_usbduxfastsub->dux_commands[LOGBASE + 5] = 0;

		this_usbduxfastsub->dux_commands[LENBASE + 6] = 1;
		this_usbduxfastsub->dux_commands[OPBASE + 6] = 0;
		this_usbduxfastsub->dux_commands[OUTBASE + 6] = 0xFF & rngmask;
		this_usbduxfastsub->dux_commands[LOGBASE + 6] = 0;
		break;

	case 3:
		// three channels
		for (j = 0; j < 1; j++) {
			if (CR_RANGE(cmd->chanlist[j]) > 0)
				rngmask = 0xff - 0x04;
			else
				rngmask = 0xff;
			// commit data to the FIFO and do the first part of the delay
			this_usbduxfastsub->dux_commands[LENBASE + j * 2] =
				steps / 2;
			this_usbduxfastsub->dux_commands[OPBASE + j * 2] = 0x02;	// data
			this_usbduxfastsub->dux_commands[OUTBASE + j * 2] = 0xFF & rngmask;	// no change
			this_usbduxfastsub->dux_commands[LOGBASE + j * 2] = 0;

			if (CR_RANGE(cmd->chanlist[j + 1]) > 0)
				rngmask = 0xff - 0x04;
			else
				rngmask = 0xff;
			// do the second part of the delay
			this_usbduxfastsub->dux_commands[LENBASE + j * 2 + 1] =
				steps - steps / 2;
			this_usbduxfastsub->dux_commands[OPBASE + j * 2 + 1] = 0;	// no data
			this_usbduxfastsub->dux_commands[OUTBASE + j * 2 + 1] = 0xFE & rngmask;	//count
			this_usbduxfastsub->dux_commands[LOGBASE + j * 2 + 1] =
				0;
		}

		// 2 steps with duration 1: the idele step and step 6:
		steps_tmp = steps - 2;
		// commit data to the FIFO and do the first part of the delay
		this_usbduxfastsub->dux_commands[LENBASE + 4] = steps_tmp / 2;
		this_usbduxfastsub->dux_commands[OPBASE + 4] = 0x02;	// data
		this_usbduxfastsub->dux_commands[OUTBASE + 4] = 0xFF & rngmask;	// no change
		this_usbduxfastsub->dux_commands[LOGBASE + 4] = 0;

		if (CR_RANGE(cmd->chanlist[0]) > 0)
			rngmask = 0xff - 0x04;
		else
			rngmask = 0xff;
		// do the second part of the delay
		this_usbduxfastsub->dux_commands[LENBASE + 5] =
			steps_tmp - steps_tmp / 2;
		this_usbduxfastsub->dux_commands[OPBASE + 5] = 0;	// no data
		this_usbduxfastsub->dux_commands[OUTBASE + 5] = (0xFF - 0x02) & rngmask;	// reset
		this_usbduxfastsub->dux_commands[LOGBASE + 5] = 0;

		this_usbduxfastsub->dux_commands[LENBASE + 6] = 1;
		this_usbduxfastsub->dux_commands[OPBASE + 6] = 0;
		this_usbduxfastsub->dux_commands[OUTBASE + 6] = 0xFF & rngmask;
		this_usbduxfastsub->dux_commands[LOGBASE + 6] = 0;
		break;

	case 16:
		if (CR_RANGE(cmd->chanlist[0]) > 0)
			rngmask = 0xff - 0x04;
		else
			rngmask = 0xff;
		if (cmd->start_src == TRIG_EXT) {	// we loop here until ready has been set
			this_usbduxfastsub->dux_commands[LENBASE + 0] = 0x01;	// branch back to state 0
			this_usbduxfastsub->dux_commands[OPBASE + 0] = 0x01;	// deceision state w/o data
			this_usbduxfastsub->dux_commands[OUTBASE + 0] = (0xFF - 0x02) & rngmask;	// reset
			this_usbduxfastsub->dux_commands[LOGBASE + 0] = 0x00;	// RDY0 = 0
		} else {	// we just proceed to state 1
			this_usbduxfastsub->dux_commands[LENBASE + 0] = 255;	// 30us reset pulse
			this_usbduxfastsub->dux_commands[OPBASE + 0] = 0;
			this_usbduxfastsub->dux_commands[OUTBASE + 0] = (0xFF - 0x02) & rngmask;	// reset
			this_usbduxfastsub->dux_commands[LOGBASE + 0] = 0;
		}

		// commit data to the FIFO
		this_usbduxfastsub->dux_commands[LENBASE + 1] = 1;
		this_usbduxfastsub->dux_commands[OPBASE + 1] = 0x02;	// data
		this_usbduxfastsub->dux_commands[OUTBASE + 1] = 0xFF & rngmask;
		this_usbduxfastsub->dux_commands[LOGBASE + 1] = 0;

		// we have 2 states with duration 1
		steps = steps - 2;

		// do the first part of the delay
		this_usbduxfastsub->dux_commands[LENBASE + 2] = steps / 2;
		this_usbduxfastsub->dux_commands[OPBASE + 2] = 0;
		this_usbduxfastsub->dux_commands[OUTBASE + 2] = 0xFE & rngmask;
		this_usbduxfastsub->dux_commands[LOGBASE + 2] = 0;

		// and the second part
		this_usbduxfastsub->dux_commands[LENBASE + 3] =
			steps - steps / 2;
		this_usbduxfastsub->dux_commands[OPBASE + 3] = 0;
		this_usbduxfastsub->dux_commands[OUTBASE + 3] = 0xFF & rngmask;
		this_usbduxfastsub->dux_commands[LOGBASE + 3] = 0;

		this_usbduxfastsub->dux_commands[LENBASE + 4] = 0x09;	// branch back to state 1
		this_usbduxfastsub->dux_commands[OPBASE + 4] = 0x01;	// deceision state w/o data
		this_usbduxfastsub->dux_commands[OUTBASE + 4] = 0xFF & rngmask;
		this_usbduxfastsub->dux_commands[LOGBASE + 4] = 0xFF;	// doesn't matter

		break;

	default:
		printk("comedi %d: unsupported combination of channels\n",
			dev->minor);
		mutex_unlock(&this_usbduxfastsub->mutex);
		return -EFAULT;
	}

#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi %d: sending commands to the usb device\n", dev->minor);
#endif
	// 0 means that the AD commands are sent
	result = send_dux_commands(this_usbduxfastsub, SENDADCOMMANDS);
	if (result < 0) {
		printk("comedi%d: adc command could not be submitted. Aborting...\n", dev->minor);
		mutex_unlock(&this_usbduxfastsub->mutex);
		return result;
	}
	if (cmd->stop_src == TRIG_COUNT) {
		this_usbduxfastsub->ai_sample_count =
			(cmd->stop_arg) * (cmd->scan_end_arg);
		if (usbduxfastsub->ai_sample_count < 1) {
			printk("comedi%d: (cmd->stop_arg)*(cmd->scan_end_arg)<1, aborting.\n", dev->minor);
			mutex_unlock(&this_usbduxfastsub->mutex);
			return -EFAULT;
		}
		this_usbduxfastsub->ai_continous = 0;
	} else {
		// continous aquisition
		this_usbduxfastsub->ai_continous = 1;
		this_usbduxfastsub->ai_sample_count = 0;
	}

	if ((cmd->start_src == TRIG_NOW) || (cmd->start_src == TRIG_EXT)) {
		// enable this acquisition operation
		this_usbduxfastsub->ai_cmd_running = 1;
		ret = usbduxfastsub_submit_InURBs(this_usbduxfastsub);
		if (ret < 0) {
			this_usbduxfastsub->ai_cmd_running = 0;
			// fixme: unlink here??
			mutex_unlock(&this_usbduxfastsub->mutex);
			return ret;
		}
		s->async->inttrig = NULL;
	} else {
		/* TRIG_INT */
		// don't enable the acquision operation
		// wait for an internal signal
		s->async->inttrig = usbduxfast_ai_inttrig;
	}
	mutex_unlock(&this_usbduxfastsub->mutex);

	return 0;
}

/* Mode 0 is used to get a single conversion on demand */
static int usbduxfast_ai_insn_read(comedi_device * dev,
	comedi_subdevice * s, comedi_insn * insn, lsampl_t * data)
{
	int i, j, n, actual_length;
	int chan, range, rngmask;
	int err;
	usbduxfastsub_t *usbduxfastsub = dev->private;

	if (!usbduxfastsub) {
		printk("comedi%d: ai_insn_read: no usb dev.\n", dev->minor);
		return -ENODEV;
	}
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi%d: ai_insn_read, insn->n=%d, insn->subdev=%d\n",
		dev->minor, insn->n, insn->subdev);
#endif
	mutex_lock(&usbduxfastsub->mutex);
	if (!(usbduxfastsub->probed)) {
		mutex_unlock(&usbduxfastsub->mutex);
		return -ENODEV;
	}
	if (usbduxfastsub->ai_cmd_running) {
		printk("comedi%d: ai_insn_read not possible. Async Command is running.\n", dev->minor);
		mutex_unlock(&usbduxfastsub->mutex);
		return -EBUSY;
	}
	// sample one channel
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);
	// set command for the first channel

	if (range > 0)
		rngmask = 0xff - 0x04;
	else
		rngmask = 0xff;
	// commit data to the FIFO
	usbduxfastsub->dux_commands[LENBASE + 0] = 1;
	usbduxfastsub->dux_commands[OPBASE + 0] = 0x02;	// data
	usbduxfastsub->dux_commands[OUTBASE + 0] = 0xFF & rngmask;
	usbduxfastsub->dux_commands[LOGBASE + 0] = 0;

	// do the first part of the delay
	usbduxfastsub->dux_commands[LENBASE + 1] = 12;
	usbduxfastsub->dux_commands[OPBASE + 1] = 0;
	usbduxfastsub->dux_commands[OUTBASE + 1] = 0xFE & rngmask;
	usbduxfastsub->dux_commands[LOGBASE + 1] = 0;

	usbduxfastsub->dux_commands[LENBASE + 2] = 1;
	usbduxfastsub->dux_commands[OPBASE + 2] = 0;
	usbduxfastsub->dux_commands[OUTBASE + 2] = 0xFE & rngmask;
	usbduxfastsub->dux_commands[LOGBASE + 2] = 0;

	usbduxfastsub->dux_commands[LENBASE + 3] = 1;
	usbduxfastsub->dux_commands[OPBASE + 3] = 0;
	usbduxfastsub->dux_commands[OUTBASE + 3] = 0xFE & rngmask;
	usbduxfastsub->dux_commands[LOGBASE + 3] = 0;

	usbduxfastsub->dux_commands[LENBASE + 4] = 1;
	usbduxfastsub->dux_commands[OPBASE + 4] = 0;
	usbduxfastsub->dux_commands[OUTBASE + 4] = 0xFE & rngmask;
	usbduxfastsub->dux_commands[LOGBASE + 4] = 0;

	// second part
	usbduxfastsub->dux_commands[LENBASE + 5] = 12;
	usbduxfastsub->dux_commands[OPBASE + 5] = 0;
	usbduxfastsub->dux_commands[OUTBASE + 5] = 0xFF & rngmask;
	usbduxfastsub->dux_commands[LOGBASE + 5] = 0;

	usbduxfastsub->dux_commands[LENBASE + 6] = 1;
	usbduxfastsub->dux_commands[OPBASE + 6] = 0;
	usbduxfastsub->dux_commands[OUTBASE + 6] = 0xFF & rngmask;
	usbduxfastsub->dux_commands[LOGBASE + 0] = 0;

#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi %d: sending commands to the usb device\n", dev->minor);
#endif
	// 0 means that the AD commands are sent
	err = send_dux_commands(usbduxfastsub, SENDADCOMMANDS);
	if (err < 0) {
		printk("comedi%d: adc command could not be submitted. Aborting...\n", dev->minor);
		mutex_unlock(&usbduxfastsub->mutex);
		return err;
	}
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi%d: usbduxfast: submitting in-urb: %x,%x\n",
		usbduxfastsub->comedidev->minor,
		(int)(usbduxfastsub->urbIn->context),
		(int)(usbduxfastsub->urbIn->dev));
#endif
	for (i = 0; i < PACKETS_TO_IGNORE; i++) {
		err = usb_bulk_msg(usbduxfastsub->usbdev,
			usb_rcvbulkpipe(usbduxfastsub->usbdev, BULKINEP),
			usbduxfastsub->transfer_buffer,
			SIZEINBUF, &actual_length, BULK_TIMEOUT);
		if (err < 0) {
			printk("comedi%d: insn timeout. No data.\n",
				dev->minor);
			mutex_unlock(&usbduxfastsub->mutex);
			return err;
		}
	}
	// data points
	for (i = 0; i < insn->n;) {
		err = usb_bulk_msg(usbduxfastsub->usbdev,
			usb_rcvbulkpipe(usbduxfastsub->usbdev, BULKINEP),
			usbduxfastsub->transfer_buffer,
			SIZEINBUF, &actual_length, BULK_TIMEOUT);
		if (err < 0) {
			printk("comedi%d: insn data error: %d\n",
				dev->minor, err);
			mutex_unlock(&usbduxfastsub->mutex);
			return err;
		}
		n = actual_length / sizeof(uint16_t);
		if ((n % 16) != 0) {
			printk("comedi%d: insn data packet corrupted.\n",
				dev->minor);
			mutex_unlock(&usbduxfastsub->mutex);
			return -EINVAL;
		}
		for (j = chan; (j < n) && (i < insn->n); j = j + 16) {
			data[i] =
				((uint16_t *) (usbduxfastsub->
					transfer_buffer))[j];
			i++;
		}
	}
	mutex_unlock(&usbduxfastsub->mutex);
	return i;
}

static unsigned hex2unsigned(char *h)
{
	unsigned hi, lo;
	if (h[0] > '9') {
		hi = h[0] - 'A' + 0x0a;
	} else {
		hi = h[0] - '0';
	}
	if (h[1] > '9') {
		lo = h[1] - 'A' + 0x0a;
	} else {
		lo = h[1] - '0';
	}
	return hi * 0x10 + lo;
}

// for FX2
#define FIRMWARE_MAX_LEN 0x2000

// taken from David Brownell's fxload and adjusted for this driver
static int read_firmware(usbduxfastsub_t * usbduxfastsub,
			 const void *firmwarePtr,
			 long size)
{
	int i = 0;
	unsigned char *fp = (char *)firmwarePtr;
	unsigned char *firmwareBinary = NULL;
	int res = 0;
	int maxAddr = 0;

	firmwareBinary = kmalloc(FIRMWARE_MAX_LEN, GFP_KERNEL);
	if (!firmwareBinary) {
		printk("comedi_: usbduxfast: mem alloc for firmware failed\n");
		return -ENOMEM;
	}

	for (;;) {
		char buf[256], *cp;
		char type;
		int len;
		int idx, off;
		int j = 0;

		// get one line
		while ((i < size) && (fp[i] != 13) && (fp[i] != 10)) {
			buf[j] = fp[i];
			i++;
			j++;
			if (j >= sizeof(buf)) {
				printk("comedi_: usbduxfast: bogus firmware file!\n");
				return -1;
			}
		}
		// get rid of LF/CR/...
		while ((i < size) && ((fp[i] == 13) || (fp[i] == 10)
				|| (fp[i] == 0))) {
			i++;
		}

		buf[j] = 0;
		//printk("comedi_: buf=%s\n",buf);

		/* EXTENSION: "# comment-till-end-of-line", for copyrights etc */
		if (buf[0] == '#')
			continue;

		if (buf[0] != ':') {
			printk("comedi_: usbduxfast: upload: not an ihex record: %s\n", buf);
			return -EFAULT;
		}

		/* Read the length field (up to 16 bytes) */
		len = hex2unsigned(buf + 1);

		/* Read the target offset */
		off = (hex2unsigned(buf + 3) * 0x0100) + hex2unsigned(buf + 5);

		if ((off + len) > maxAddr) {
			maxAddr = off + len;
		}

		if (maxAddr >= FIRMWARE_MAX_LEN) {
			printk("comedi_: usbduxfast: firmware upload goes beyond FX2 RAM boundaries.\n");
			return -EFAULT;
		}
		//printk("comedi_: usbduxfast: off=%x, len=%x:\n",off,len);

		/* Read the record type */
		type = hex2unsigned(buf + 7);

		/* If this is an EOF record, then make it so. */
		if (type == 1) {
			break;
		}

		if (type != 0) {
			printk("comedi_: usbduxfast: unsupported record type: %u\n", type);
			return -EFAULT;
		}

		for (idx = 0, cp = buf + 9; idx < len; idx += 1, cp += 2) {
			firmwareBinary[idx + off] = hex2unsigned(cp);
			//printk("%02x ",firmwareBinary[idx+off]);
		}
		//printk("\n");

		if (i >= size) {
			printk("comedi_: usbduxfast: unexpected end of hex file\n");
			break;
		}

	}
	res = firmwareUpload(usbduxfastsub, firmwareBinary, maxAddr + 1);
	kfree(firmwareBinary);
	return res;
}

static void tidy_up(usbduxfastsub_t * usbduxfastsub_tmp)
{
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi_: usbduxfast: tiding up\n");
#endif
	if (!usbduxfastsub_tmp) {
		return;
	}
	// shows the usb subsystem that the driver is down
	if (usbduxfastsub_tmp->interface) {
		usb_set_intfdata(usbduxfastsub_tmp->interface, NULL);
	}

	usbduxfastsub_tmp->probed = 0;

	if (usbduxfastsub_tmp->urbIn) {
		// waits until a running transfer is over
		// thus, under 2.4 hotplugging while a command
		// is running is not safe
		usb_kill_urb(usbduxfastsub_tmp->urbIn);
		if (usbduxfastsub_tmp->transfer_buffer) {
			kfree(usbduxfastsub_tmp->transfer_buffer);
			usbduxfastsub_tmp->transfer_buffer = NULL;
		}
		usb_free_urb(usbduxfastsub_tmp->urbIn);
		usbduxfastsub_tmp->urbIn = NULL;
	}
	if (usbduxfastsub_tmp->insnBuffer) {
		kfree(usbduxfastsub_tmp->insnBuffer);
		usbduxfastsub_tmp->insnBuffer = NULL;
	}
	if (usbduxfastsub_tmp->dux_commands) {
		kfree(usbduxfastsub_tmp->dux_commands);
		usbduxfastsub_tmp->dux_commands = NULL;
	}
	usbduxfastsub_tmp->ai_cmd_running = 0;
}

static void usbduxfast_firmware_request_complete_handler(
	const struct firmware *fw,
	void *context)
{
	usbduxfastsub_t * usbduxfastsub_tmp = (usbduxfastsub_t *)context;
	struct usb_device *usbdev = usbduxfastsub_tmp->usbdev;
	int ret;

	if(fw == NULL) {
		return;
	}

	// we need to upload the firmware here because fw will be
	// freed one we've left this function
	ret=read_firmware(usbduxfastsub_tmp,
			  fw->data,
			  fw->size);

	if (ret) {
		dev_err(&usbdev->dev,
			"Could not upload firmware (err=%d)\n",
			ret);
		goto out;
	}

	comedi_usb_auto_config(usbduxfastsub_tmp->interface,
			       &driver_usbduxfast, 0);
out:
	/*
	 * in more recent versions the completion handler
	 * had to release the firmware whereas in older
	 * versions this has been done by the caller
	 */
	COMEDI_RELEASE_FIRMWARE_NOWAIT(fw);
}

// allocate memory for the urbs and initialise them
static int usbduxfastsub_probe(struct usb_interface *uinterf,
	const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(uinterf);
	int i;
	int index;
	int ret;

	if (udev->speed != USB_SPEED_HIGH) {
		printk("comedi_: usbduxfast_: This driver needs USB 2.0 to operate. Aborting...\n");
		return -ENODEV;
	}
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi_: usbduxfast_: finding a free structure for the usb-device\n");
#endif
	mutex_lock(&start_stop_mutex);
	// look for a free place in the usbduxfast array
	index = -1;
	for (i = 0; i < NUMUSBDUXFAST; i++) {
		if (!(usbduxfastsub[i].probed)) {
			index = i;
			break;
		}
	}

	// no more space
	if (index == -1) {
		printk("Too many usbduxfast-devices connected.\n");
		mutex_unlock(&start_stop_mutex);
		return -EMFILE;
	}
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi_: usbduxfast: usbduxfastsub[%d] is ready to connect to comedi.\n", index);
#endif

	mutex_init(&(usbduxfastsub[index].mutex));
	// save a pointer to the usb device
	usbduxfastsub[index].usbdev = udev;

	// 2.6: save the interface itself
	usbduxfastsub[index].interface = uinterf;
	// get the interface number from the interface
	usbduxfastsub[index].ifnum = uinterf->altsetting->desc.bInterfaceNumber;
	// hand the private data over to the usb subsystem
	// will be needed for disconnect
	usb_set_intfdata(uinterf, &(usbduxfastsub[index]));

#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi_: usbduxfast: ifnum=%d\n", usbduxfastsub[index].ifnum);
#endif
	// create space for the commands going to the usb device
	usbduxfastsub[index].dux_commands = kmalloc(SIZEOFDUXBUFFER,
		GFP_KERNEL);
	if (!usbduxfastsub[index].dux_commands) {
		printk("comedi_: usbduxfast: error alloc space for dac commands\n");
		tidy_up(&(usbduxfastsub[index]));
		mutex_unlock(&start_stop_mutex);
		return -ENOMEM;
	}
	// create space of the instruction buffer
	usbduxfastsub[index].insnBuffer = kmalloc(SIZEINSNBUF, GFP_KERNEL);
	if (!(usbduxfastsub[index].insnBuffer)) {
		printk("comedi_: usbduxfast: could not alloc space for insnBuffer\n");
		tidy_up(&(usbduxfastsub[index]));
		mutex_unlock(&start_stop_mutex);
		return -ENOMEM;
	}
	// setting to alternate setting 1: enabling bulk ep
	i = usb_set_interface(usbduxfastsub[index].usbdev,
		usbduxfastsub[index].ifnum, 1);
	if (i < 0) {
		printk("comedi_: usbduxfast%d: could not switch to alternate setting 1.\n", index);
		tidy_up(&(usbduxfastsub[index]));
		mutex_unlock(&start_stop_mutex);
		return -ENODEV;
	}
	usbduxfastsub[index].urbIn = usb_alloc_urb(0, GFP_KERNEL);
	if (usbduxfastsub[index].urbIn == NULL) {
		printk("comedi_: usbduxfast%d: Could not alloc. urb\n", index);
		tidy_up(&(usbduxfastsub[index]));
		mutex_unlock(&start_stop_mutex);
		return -ENOMEM;
	}
	usbduxfastsub[index].transfer_buffer = kmalloc(SIZEINBUF, GFP_KERNEL);
	if (!(usbduxfastsub[index].transfer_buffer)) {
		printk("comedi_: usbduxfast%d: could not alloc. transb.\n",
			index);
		tidy_up(&(usbduxfastsub[index]));
		mutex_unlock(&start_stop_mutex);
		return -ENOMEM;
	}
	// we've reached the bottom of the function
	usbduxfastsub[index].probed = 1;
	mutex_unlock(&start_stop_mutex);

	ret = request_firmware_nowait(THIS_MODULE,
				      FW_ACTION_HOTPLUG,
				      "usbduxfast_firmware.hex",
				      &udev->dev,
				      GFP_KERNEL,
				      usbduxfastsub + index,
				      usbduxfast_firmware_request_complete_handler);

	if (ret) {
		dev_err(&udev->dev,
			"could not load firmware (err=%d)\n",
			ret);
		return ret;
	}

	printk("comedi_: usbduxfast%d has been successfully initialized.\n",
	       index);

	// success
	return 0;
}

static void usbduxfastsub_disconnect(struct usb_interface *intf)
{
	usbduxfastsub_t *usbduxfastsub_tmp = usb_get_intfdata(intf);
	if (!usbduxfastsub_tmp) {
		printk("comedi_: usbduxfast: disconnect called with null pointer.\n");
		return;
	}
	if (usbduxfastsub_tmp->interface != intf) {
		printk("comedi_: usbduxfast: BUG! called with wrong ptr!!!\n");
		return;
	}

	comedi_usb_auto_unconfig(intf);

	mutex_lock(&start_stop_mutex);
	mutex_lock(&usbduxfastsub_tmp->mutex);
	tidy_up(usbduxfastsub_tmp);
	mutex_unlock(&usbduxfastsub_tmp->mutex);
	mutex_unlock(&start_stop_mutex);
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi_: usbduxfast: disconnected from the usb\n");
#endif
}

// is called when comedi-config is called
static int usbduxfast_attach(comedi_device * dev, comedi_devconfig * it)
{
	int ret;
	int index;
	int i;
	comedi_subdevice *s = NULL;
	dev->private = NULL;

	mutex_lock(&start_stop_mutex);
	// find a valid device which has been detected by the probe function of the usb
	index = -1;
	for (i = 0; i < NUMUSBDUXFAST; i++) {
		if ((usbduxfastsub[i].probed) && (!usbduxfastsub[i].attached)) {
			index = i;
			break;
		}
	}

	if (index < 0) {
		printk("comedi%d: usbduxfast: error: attach failed, no usbduxfast devs connected to the usb bus.\n", dev->minor);
		mutex_unlock(&start_stop_mutex);
		return -ENODEV;
	}

	mutex_lock(&(usbduxfastsub[index].mutex));
	// pointer back to the corresponding comedi device
	usbduxfastsub[index].comedidev = dev;

	// trying to upload the firmware into the chip
	if (comedi_aux_data(it->options, 0) &&
	    it->options[COMEDI_DEVCONF_AUX_DATA_LENGTH]) {
		read_firmware(&usbduxfastsub[index],
			      comedi_aux_data(it->options, 0),
			      it->options[COMEDI_DEVCONF_AUX_DATA_LENGTH]);
	}

	dev->board_name = BOARDNAME;

	/* set number of subdevices */
	dev->n_subdevices = N_SUBDEVICES;

	// allocate space for the subdevices
	if ((ret = alloc_subdevices(dev, N_SUBDEVICES)) < 0) {
		printk("comedi%d: usbduxfast: error alloc space for subdev\n",
			dev->minor);
		mutex_unlock(&start_stop_mutex);
		return ret;
	}

	printk("comedi%d: usbduxfast: usb-device %d is attached to comedi.\n",
		dev->minor, index);
	// private structure is also simply the usb-structure
	dev->private = usbduxfastsub + index;
	// the first subdevice is the A/D converter
	s = dev->subdevices + SUBDEV_AD;
	// the URBs get the comedi subdevice
	// which is responsible for reading
	// this is the subdevice which reads data
	dev->read_subdev = s;
	// the subdevice receives as private structure the
	// usb-structure
	s->private = NULL;
	// analog input
	s->type = COMEDI_SUBD_AI;
	// readable and ref is to ground
	s->subdev_flags = SDF_READABLE | SDF_GROUND | SDF_CMD_READ;
	// 16 channels
	s->n_chan = 16;
	// length of the channellist
	s->len_chanlist = 16;
	// callback functions
	s->insn_read = usbduxfast_ai_insn_read;
	s->do_cmdtest = usbduxfast_ai_cmdtest;
	s->do_cmd = usbduxfast_ai_cmd;
	s->cancel = usbduxfast_ai_cancel;
	// max value from the A/D converter (12bit+1 bit for overflow)
	s->maxdata = 0x1000;
	// range table to convert to physical units
	s->range_table = &range_usbduxfast_ai_range;

	// finally decide that it's attached
	usbduxfastsub[index].attached = 1;

	mutex_unlock(&(usbduxfastsub[index].mutex));

	mutex_unlock(&start_stop_mutex);

	printk("comedi%d: successfully attached to usbduxfast.\n", dev->minor);

	return 0;
}

static int usbduxfast_detach(comedi_device * dev)
{
	usbduxfastsub_t *usbduxfastsub_tmp;

#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi%d: usbduxfast: detach usb device\n", dev->minor);
#endif

	if (!dev) {
		printk("comedi?: usbduxfast: detach without dev variable...\n");
		return -EFAULT;
	}

	usbduxfastsub_tmp = dev->private;
	if (!usbduxfastsub_tmp) {
		printk("comedi?: usbduxfast: detach without ptr to usbduxfastsub[]\n");
		return -EFAULT;
	}

	mutex_lock(&usbduxfastsub_tmp->mutex);
	mutex_lock(&start_stop_mutex);
	// Don't allow detach to free the private structure
	// It's one entry of of usbduxfastsub[]
	dev->private = NULL;
	usbduxfastsub_tmp->attached = 0;
	usbduxfastsub_tmp->comedidev = NULL;
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi%d: usbduxfast: detach: successfully removed\n",
		dev->minor);
#endif
	mutex_unlock(&start_stop_mutex);
	mutex_unlock(&usbduxfastsub_tmp->mutex);
	return 0;
}

/* main driver struct */
static comedi_driver driver_usbduxfast = {
      driver_name:"usbduxfast",
      module:THIS_MODULE,
      attach:usbduxfast_attach,
      detach:usbduxfast_detach,
};

static void init_usb_devices(void)
{
	int index;
#ifdef COMEDI_CONFIG_DEBUG
	printk("comedi_: usbduxfast: setting all possible devs to invalid\n");
#endif
	// all devices entries are invalid to begin with
	// they will become valid by the probe function
	// and then finally by the attach-function
	for (index = 0; index < NUMUSBDUXFAST; index++) {
		memset(&(usbduxfastsub[index]), 0x00,
			sizeof(usbduxfastsub[index]));
		mutex_init(&(usbduxfastsub[index].mutex));
	}
}

static void uninit_usb_devices(void)
{
	int index;

	for (index = 0; index < NUMUSBDUXFAST; index++) {
		mutex_destroy(&(usbduxfastsub[index].mutex));
	}
}

// Table with the USB-devices: just now only testing IDs
static struct usb_device_id usbduxfastsub_table[] = {
	//        { USB_DEVICE(0x4b4, 0x8613), //testing
	//        },
	{USB_DEVICE(0x13d8, 0x0010)	//real ID
		},
	{USB_DEVICE(0x13d8, 0x0011)	//real ID
		},
	{}			/* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, usbduxfastsub_table);

// The usbduxfastsub-driver
static struct usb_driver usbduxfastsub_driver = {
#ifdef COMEDI_COMPAT_HAVE_USB_DRIVER_OWNER
      owner:THIS_MODULE,
#endif
      name:BOARDNAME,
      probe:usbduxfastsub_probe,
      disconnect:usbduxfastsub_disconnect,
      id_table:usbduxfastsub_table,
};

// Can't use the nice macro as I have also to initialise the USB
// subsystem:
// registering the usb-system _and_ the comedi-driver
static int init_usbduxfast(void)
{
	printk(KERN_INFO COMEDI_MODNAME ": "
	       DRIVER_VERSION ":" DRIVER_DESC "\n");
	init_usb_devices();
	usb_register(&usbduxfastsub_driver);
	comedi_driver_register(&driver_usbduxfast);
	return 0;
}

// deregistering the comedi driver and the usb-subsystem
static void exit_usbduxfast(void)
{
	comedi_driver_unregister(&driver_usbduxfast);
	usb_deregister(&usbduxfastsub_driver);
	uninit_usb_devices();
}

module_init(init_usbduxfast);
module_exit(exit_usbduxfast);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_VERSION(COMEDI_RELEASE);
