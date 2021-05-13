/*
    comedi/drivers/vmk80xx.c
    Velleman USB Board Low-Level Driver

    Ported from the Linux kernel sources by Ian Abbott <abbotti@mev.co.uk>.

    Copyright (C) 2009 Manuel Gebele <forensixs@gmx.de>, Germany

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
Driver: vmk80xx
Description: Velleman K8055/VM110, K8061/VM140 USB boards
Devices: [Velleman] K8055 (K8055/VM110), K8061 (K8061/VM140),
  VM110 (K8055/VM110), VM140 (K8061/VM140)
Author: Manuel Gebele <forensixs@gmx.de>
Updated: Thu, 13 May 2021 17:46:07 +0100
Status: works

Supports:
 - analog input
 - analog output
 - digital input
 - digital output
 - counter
 - pwm
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/usb.h>
#include <linux/uaccess.h>

#include <linux/comedidev.h>

#define BOARDNAME "vmk80xx"

#define VMK8055_DI_REG		0x00
#define VMK8055_DO_REG		0x01
#define VMK8055_AO1_REG		0x02
#define VMK8055_AO2_REG		0x03
#define VMK8055_AI1_REG		0x02
#define VMK8055_AI2_REG		0x03
#define VMK8055_CNT1_REG	0x04
#define VMK8055_CNT2_REG	0x06

#define VMK8061_CH_REG		0x01
#define VMK8061_DI_REG		0x01
#define VMK8061_DO_REG		0x01
#define VMK8061_PWM_REG1	0x01
#define VMK8061_PWM_REG2	0x02
#define VMK8061_CNT_REG		0x02
#define VMK8061_AO_REG		0x02
#define VMK8061_AI_REG1		0x02
#define VMK8061_AI_REG2		0x03

#define VMK8055_CMD_RST		0x00
#define VMK8055_CMD_DEB1_TIME	0x01
#define VMK8055_CMD_DEB2_TIME	0x02
#define VMK8055_CMD_RST_CNT1	0x03
#define VMK8055_CMD_RST_CNT2	0x04
#define VMK8055_CMD_WRT_AD	0x05

#define VMK8061_CMD_RD_AI	0x00
#define VMK8061_CMR_RD_ALL_AI	0x01	/* !non-active! */
#define VMK8061_CMD_SET_AO	0x02
#define VMK8061_CMD_SET_ALL_AO	0x03	/* !non-active! */
#define VMK8061_CMD_OUT_PWM	0x04
#define VMK8061_CMD_RD_DI	0x05
#define VMK8061_CMD_DO		0x06	/* !non-active! */
#define VMK8061_CMD_CLR_DO	0x07
#define VMK8061_CMD_SET_DO	0x08
#define VMK8061_CMD_RD_CNT	0x09	/* TODO: completely pointless? */
#define VMK8061_CMD_RST_CNT	0x0a	/* TODO: completely pointless? */
#define VMK8061_CMD_RD_VERSION	0x0b	/* internal usage */
#define VMK8061_CMD_RD_JMP_STAT	0x0c	/* TODO: not implemented yet */
#define VMK8061_CMD_RD_PWR_STAT	0x0d	/* internal usage */
#define VMK8061_CMD_RD_DO	0x0e
#define VMK8061_CMD_RD_AO	0x0f
#define VMK8061_CMD_RD_PWM	0x10

#define VMK80XX_MAX_BOARDS	COMEDI_NUM_BOARD_MINORS

#define IC3_VERSION		(1 << 0)
#define IC6_VERSION		(1 << 1)

#define CONFIG_VMK80XX_DEBUG
#undef CONFIG_VMK80XX_DEBUG

#ifdef CONFIG_VMK80XX_DEBUG
static int dbgvm = 1;
#else
static int dbgvm;
#endif

#ifdef CONFIG_COMEDI_DEBUG
static int dbgcm = 1;
#else
static int dbgcm;
#endif

#define dbgvm(fmt, arg...)                     \
do {                                           \
	if (dbgvm)                             \
		printk(KERN_DEBUG fmt, ##arg); \
} while (0)

#define dbgcm(fmt, arg...)                     \
do {                                           \
	if (dbgcm)                             \
		printk(KERN_DEBUG fmt, ##arg); \
} while (0)

enum vmk80xx_model {
	VMK8055_MODEL,
	VMK8061_MODEL
};

struct firmware_version {
	unsigned char ic3_vers[32];	/* USB-Controller */
	unsigned char ic6_vers[32];	/* CPU */
};

static const comedi_lrange vmk8061_range = {
	2, {
		UNI_RANGE(5),
		UNI_RANGE(10)
	}
};

struct vmk80xx_board {
	const char *name;
	const comedi_lrange *range;
	unsigned short ai_nchans;
	sampl_t ai_maxdata;
	unsigned short ao_nchans;
	unsigned short di_nchans;
	sampl_t cnt_maxdata;
	unsigned short pwm_nchans;
	sampl_t pwm_maxdata;
};

static const struct vmk80xx_board vmk80xx_boardinfo[] = {
	[VMK8055_MODEL] = {
		.name		= "K8055 (VM110)",
		.range		= &range_unipolar5,
		.ai_nchans	= 2,
		.ai_maxdata	= 0x00ff,
		.ao_nchans	= 2,
		.di_nchans	= 6,
		.cnt_maxdata	= 0xffff,
	},
	[VMK8061_MODEL] = {
		.name		= "K8061 (VM140)",
		.range		= &vmk8061_range,
		.ai_nchans	= 8,
		.ai_maxdata	= 0x03ff,
		.ao_nchans	= 8,
		.di_nchans	= 8,
		.cnt_maxdata	= 0,	/* unknown, device is not writeable */
		.pwm_nchans	= 1,
		.pwm_maxdata	= 0x03ff,
	},
};

struct vmk80xx_private {
	struct usb_device *usb;
	struct usb_interface *intf;
	struct usb_endpoint_descriptor *ep_rx;
	struct usb_endpoint_descriptor *ep_tx;
	struct firmware_version fw;
	struct semaphore limit_sem;
	unsigned char *usb_rx_buf;
	unsigned char *usb_tx_buf;
	int probed;
	int attached;
	int count;
	enum vmk80xx_model model;
};

static struct vmk80xx_private vmb[VMK80XX_MAX_BOARDS];

static DEFINE_MUTEX(glb_mutex);

static int vmk80xx_check_data_link(struct vmk80xx_private *devpriv)
{
	struct usb_device *usb = devpriv->usb;
	unsigned int tx_pipe;
	unsigned int rx_pipe;
	unsigned char *tx;
	unsigned char *rx;
	int ret;

	dbgvm("vmk80xx: %s\n", __func__);

	tx_pipe = usb_sndbulkpipe(usb, 0x01);
	rx_pipe = usb_rcvbulkpipe(usb, 0x81);

	tx = kzalloc(1, GFP_KERNEL);
	if (!tx)
		return 0;

	rx = kzalloc(2, GFP_KERNEL);
	if (!rx) {
		kfree(tx);
		return 0;
	}

	tx[0] = VMK8061_CMD_RD_PWR_STAT;

	/*
	 * Check that IC6 (PIC16F871) is powered and
	 * running and the data link between IC3 and
	 * IC6 is working properly
	 */
	usb_bulk_msg(usb, tx_pipe, tx, 1, NULL, devpriv->ep_tx->bInterval);
	usb_bulk_msg(usb, rx_pipe, rx, 2, NULL, HZ * 10);

	ret = (int)rx[1];

	kfree(tx);
	kfree(rx);

	return ret;
}

static void vmk80xx_read_eeprom(struct vmk80xx_private *devpriv, int flag)
{
	struct usb_device *usb = devpriv->usb;
	unsigned int tx_pipe;
	unsigned int rx_pipe;
	unsigned char *tx;
	unsigned char *rx;
	int cnt = 0;

	dbgvm("vmk80xx: %s\n", __func__);

	tx_pipe = usb_sndbulkpipe(usb, 0x01);
	rx_pipe = usb_rcvbulkpipe(usb, 0x81);

	tx = kzalloc(1, GFP_KERNEL);
	if (!tx)
		return;

	rx = kzalloc(64 + 1, GFP_KERNEL);
	if (!rx) {
		kfree(tx);
		return;
	}

	tx[0] = VMK8061_CMD_RD_VERSION;

	/*
	 * Read the firmware version info of IC3 and
	 * IC6 from the internal EEPROM of the IC
	 */
	usb_bulk_msg(usb, tx_pipe, tx, 1, NULL, devpriv->ep_tx->bInterval);
	usb_bulk_msg(usb, rx_pipe, rx, 64, &cnt, HZ * 10);

	rx[cnt] = '\0';

	if (flag & IC3_VERSION)
		strncpy(devpriv->fw.ic3_vers, rx + 1, 24);
	else			/* IC6_VERSION */
		strncpy(devpriv->fw.ic6_vers, rx + 25, 24);

	kfree(tx);
	kfree(rx);
}

static void vmk80xx_do_bulk_msg(struct vmk80xx_private *devpriv)
{
	struct usb_device *usb = devpriv->usb;
	__u8 tx_addr;
	__u8 rx_addr;
	unsigned int tx_pipe;
	unsigned int rx_pipe;
	size_t size;

	dbgvm("vmk80xx: %s\n", __func__);

	tx_addr = devpriv->ep_tx->bEndpointAddress;
	rx_addr = devpriv->ep_rx->bEndpointAddress;
	tx_pipe = usb_sndbulkpipe(usb, tx_addr);
	rx_pipe = usb_rcvbulkpipe(usb, rx_addr);

	/*
	 * The max packet size attributes of the K8061
	 * input/output endpoints are identical
	 */
	size = le16_to_cpu(devpriv->ep_tx->wMaxPacketSize);

	usb_bulk_msg(usb, tx_pipe, devpriv->usb_tx_buf,
		     size, NULL, devpriv->ep_tx->bInterval);
	usb_bulk_msg(usb, rx_pipe, devpriv->usb_rx_buf, size, NULL, HZ * 10);
}

static int vmk80xx_read_packet(struct vmk80xx_private *devpriv)
{
	struct usb_device *usb;
	struct usb_endpoint_descriptor *ep;
	unsigned int pipe;

	dbgvm("vmk80xx: %s\n", __func__);

	if (!devpriv->intf)
		return -ENODEV;

	if (devpriv->model == VMK8061_MODEL) {
		vmk80xx_do_bulk_msg(devpriv);
		return 0;
	}

	usb = devpriv->usb;
	ep = devpriv->ep_rx;
	pipe = usb_rcvintpipe(usb, ep->bEndpointAddress);
	return usb_interrupt_msg(usb, pipe, devpriv->usb_rx_buf,
				 le16_to_cpu(ep->wMaxPacketSize), NULL,
				 HZ * 10);
}

static int vmk80xx_write_packet(struct vmk80xx_private *devpriv, int cmd)
{
	struct usb_device *usb;
	struct usb_endpoint_descriptor *ep;
	unsigned int pipe;

	dbgvm("vmk80xx: %s\n", __func__);

	if (!devpriv->intf)
		return -ENODEV;

	devpriv->usb_tx_buf[0] = cmd;

	if (devpriv->model == VMK8061_MODEL) {
		vmk80xx_do_bulk_msg(devpriv);
		return 0;
	}

	usb = devpriv->usb;
	ep = devpriv->ep_tx;
	pipe = usb_sndintpipe(usb, ep->bEndpointAddress);
	return usb_interrupt_msg(usb, pipe, devpriv->usb_tx_buf,
				 le16_to_cpu(ep->wMaxPacketSize), NULL,
				 HZ * 10);
}

static int vmk80xx_reset_device(struct vmk80xx_private *devpriv)
{
	size_t size;
	int retval;

	dbgvm("vmk80xx: %s\n", __func__);

	size = le16_to_cpu(devpriv->ep_tx->wMaxPacketSize);
	memset(devpriv->usb_tx_buf, 0, size);
	retval = vmk80xx_write_packet(devpriv, VMK8055_CMD_RST);
	if (retval)
		return retval;
	/* set outputs to known state as we cannot read them */
	return vmk80xx_write_packet(devpriv, VMK8055_CMD_WRT_AD);
}

static int vmk80xx_ai_insn_read(comedi_device *dev, comedi_subdevice *s,
				comedi_insn *insn, lsampl_t *data)
{
	struct vmk80xx_private *devpriv = dev->private;
	int chan;
	int reg[2];
	int n;

	dbgvm("vmk80xx: %s\n", __func__);

	down(&devpriv->limit_sem);
	chan = CR_CHAN(insn->chanspec);

	switch (devpriv->model) {
	case VMK8055_MODEL:
		if (!chan)
			reg[0] = VMK8055_AI1_REG;
		else
			reg[0] = VMK8055_AI2_REG;
		break;
	case VMK8061_MODEL:
	default:
		reg[0] = VMK8061_AI_REG1;
		reg[1] = VMK8061_AI_REG2;
		devpriv->usb_tx_buf[0] = VMK8061_CMD_RD_AI;
		devpriv->usb_tx_buf[VMK8061_CH_REG] = chan;
		break;
	}

	for (n = 0; n < insn->n; n++) {
		if (vmk80xx_read_packet(devpriv))
			break;

		if (devpriv->model == VMK8055_MODEL) {
			data[n] = devpriv->usb_rx_buf[reg[0]];
			continue;
		}

		/* VMK8061_MODEL */
		data[n] = devpriv->usb_rx_buf[reg[0]] + 256 *
		    devpriv->usb_rx_buf[reg[1]];
	}

	up(&devpriv->limit_sem);

	return n;
}

static int vmk80xx_ao_insn_write(comedi_device *dev, comedi_subdevice *s,
				 comedi_insn *insn, lsampl_t *data)
{
	struct vmk80xx_private *devpriv = dev->private;
	int chan;
	int cmd;
	int reg;
	int n;

	dbgvm("vmk80xx: %s\n", __func__);

	down(&devpriv->limit_sem);
	chan = CR_CHAN(insn->chanspec);

	switch (devpriv->model) {
	case VMK8055_MODEL:
		cmd = VMK8055_CMD_WRT_AD;
		if (!chan)
			reg = VMK8055_AO1_REG;
		else
			reg = VMK8055_AO2_REG;
		break;
	default:		/* NOTE: avoid compiler warnings */
		cmd = VMK8061_CMD_SET_AO;
		reg = VMK8061_AO_REG;
		devpriv->usb_tx_buf[VMK8061_CH_REG] = chan;
		break;
	}

	for (n = 0; n < insn->n; n++) {
		devpriv->usb_tx_buf[reg] = data[n];

		if (vmk80xx_write_packet(devpriv, cmd))
			break;
	}

	up(&devpriv->limit_sem);

	return n;
}

static int vmk80xx_ao_insn_read(comedi_device *dev, comedi_subdevice *s,
				comedi_insn *insn, lsampl_t *data)
{
	struct vmk80xx_private *devpriv = dev->private;
	int chan;
	int reg;
	int n;

	dbgvm("vmk80xx: %s\n", __func__);

	down(&devpriv->limit_sem);
	chan = CR_CHAN(insn->chanspec);

	reg = VMK8061_AO_REG - 1;

	devpriv->usb_tx_buf[0] = VMK8061_CMD_RD_AO;

	for (n = 0; n < insn->n; n++) {
		if (vmk80xx_read_packet(devpriv))
			break;

		data[n] = devpriv->usb_rx_buf[reg + chan];
	}

	up(&devpriv->limit_sem);

	return n;
}

static int vmk80xx_di_insn_bits(comedi_device *dev, comedi_subdevice *s,
				comedi_insn *insn, lsampl_t *data)
{
	struct vmk80xx_private *devpriv = dev->private;
	unsigned char *rx_buf;
	int reg;
	int retval;

	dbgvm("vmk80xx: %s\n", __func__);

	down(&devpriv->limit_sem);

	rx_buf = devpriv->usb_rx_buf;

	if (devpriv->model == VMK8061_MODEL) {
		reg = VMK8061_DI_REG;
		devpriv->usb_tx_buf[0] = VMK8061_CMD_RD_DI;
	} else {
		reg = VMK8055_DI_REG;
	}

	retval = vmk80xx_read_packet(devpriv);

	if (!retval) {
		if (devpriv->model == VMK8055_MODEL)
			data[1] = (((rx_buf[reg] >> 4) & 0x03) |
				   ((rx_buf[reg] << 2) & 0x04) |
				   ((rx_buf[reg] >> 3) & 0x18));
		else
			data[1] = rx_buf[reg];

		retval = 2;
	}

	up(&devpriv->limit_sem);

	return retval;
}

static int vmk80xx_do_insn_bits(comedi_device *dev, comedi_subdevice *s,
				comedi_insn *insn, lsampl_t *data)
{
	struct vmk80xx_private *devpriv = dev->private;
	unsigned char *rx_buf;
	unsigned char *tx_buf;
	int reg;
	int cmd;
	int ret = 0;

	dbgvm("vmk80xx: %s\n", __func__);

	if (devpriv->model == VMK8061_MODEL) {
		reg = VMK8061_DO_REG;
		cmd = VMK8061_CMD_DO;
	} else {
		/* VMK8055_MODEL */
		reg = VMK8055_DO_REG;
		cmd = VMK8055_CMD_WRT_AD;
	}

	down(&devpriv->limit_sem);

	rx_buf = devpriv->usb_rx_buf;
	tx_buf = devpriv->usb_tx_buf;

	if (data[0]) {
		tx_buf[reg] &= ~data[0];
		tx_buf[reg] |= (data[0] & data[1]);

		ret = vmk80xx_write_packet(devpriv, cmd);
		if (ret)
			goto out;
	}

	if (devpriv->model == VMK8061_MODEL) {
		tx_buf[0] = VMK8061_CMD_RD_DO;

		ret = vmk80xx_read_packet(devpriv);
		if (ret)
			goto out;

		data[1] = rx_buf[reg];
	} else {
		data[1] = tx_buf[reg];
	}

out:
	up(&devpriv->limit_sem);

	return ret ? ret : insn->n;
}

static int vmk80xx_cnt_insn_read(comedi_device *dev, comedi_subdevice *s,
				 comedi_insn *insn, lsampl_t *data)
{
	struct vmk80xx_private *devpriv = dev->private;
	int chan;
	int reg[2];
	int n;

	dbgvm("vmk80xx: %s\n", __func__);

	down(&devpriv->limit_sem);
	chan = CR_CHAN(insn->chanspec);

	switch (devpriv->model) {
	case VMK8055_MODEL:
		if (!chan)
			reg[0] = VMK8055_CNT1_REG;
		else
			reg[0] = VMK8055_CNT2_REG;
		break;
	case VMK8061_MODEL:
	default:
		reg[0] = VMK8061_CNT_REG;
		reg[1] = VMK8061_CNT_REG;
		devpriv->usb_tx_buf[0] = VMK8061_CMD_RD_CNT;
		break;
	}

	for (n = 0; n < insn->n; n++) {
		if (vmk80xx_read_packet(devpriv))
			break;

		if (devpriv->model == VMK8055_MODEL) {
			data[n] = devpriv->usb_rx_buf[reg[0]];
		} else {
			/* VMK8061_MODEL */
			data[n] = devpriv->usb_rx_buf[reg[0] * (chan + 1) + 1]
			    + 256 * devpriv->usb_rx_buf[reg[1] * 2 + 2];
		}
	}

	up(&devpriv->limit_sem);

	return n;
}

static int vmk80xx_cnt_insn_config(comedi_device *dev, comedi_subdevice *s,
				   comedi_insn *insn, lsampl_t *data)
{
	struct vmk80xx_private *devpriv = dev->private;
	unsigned int chan = CR_CHAN(insn->chanspec);
	int cmd;
	int reg;
	int ret;

	dbgvm("vmk80xx: %s\n", __func__);

	down(&devpriv->limit_sem);

	switch (data[0]) {
	case INSN_CONFIG_RESET:
		if (devpriv->model == VMK8055_MODEL) {
			if (!chan) {
				cmd = VMK8055_CMD_RST_CNT1;
				reg = VMK8055_CNT1_REG;
			} else {
				cmd = VMK8055_CMD_RST_CNT2;
				reg = VMK8055_CNT2_REG;
			}
			devpriv->usb_tx_buf[reg] = 0x00;
		} else {
			cmd = VMK8061_CMD_RST_CNT;
		}
		ret = vmk80xx_write_packet(devpriv, cmd);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	up(&devpriv->limit_sem);

	return ret ? ret : insn->n;
}

static int vmk80xx_cnt_insn_write(comedi_device *dev, comedi_subdevice *s,
				  comedi_insn *insn, lsampl_t *data)
{
	struct vmk80xx_private *devpriv = dev->private;
	unsigned long debtime;
	unsigned long val;
	int chan, cmd;
	int n;

	dbgvm("vmk80xx: %s\n", __func__);

	down(&devpriv->limit_sem);
	chan = CR_CHAN(insn->chanspec);

	if (!chan)
		cmd = VMK8055_CMD_DEB1_TIME;
	else
		cmd = VMK8055_CMD_DEB2_TIME;

	for (n = 0; n < insn->n; n++) {
		debtime = data[n];
		if (debtime == 0)
			debtime = 1;

		/* TODO: Prevent overflows */
		if (debtime > 7450)
			debtime = 7450;

		val = int_sqrt(debtime * 1000 / 115);
		if (((val + 1) * val) < debtime * 1000 / 115)
			val += 1;

		devpriv->usb_tx_buf[6 + chan] = val;

		if (vmk80xx_write_packet(devpriv, cmd))
			break;
	}

	up(&devpriv->limit_sem);

	return n;
}

static int vmk80xx_pwm_insn_read(comedi_device *dev, comedi_subdevice *s,
				 comedi_insn *insn, lsampl_t *data)
{
	struct vmk80xx_private *devpriv = dev->private;
	unsigned char *tx_buf;
	unsigned char *rx_buf;
	int reg[2];
	int n;

	dbgvm("vmk80xx: %s\n", __func__);

	down(&devpriv->limit_sem);

	tx_buf = devpriv->usb_tx_buf;
	rx_buf = devpriv->usb_rx_buf;

	reg[0] = VMK8061_PWM_REG1;
	reg[1] = VMK8061_PWM_REG2;

	tx_buf[0] = VMK8061_CMD_RD_PWM;

	for (n = 0; n < insn->n; n++) {
		if (vmk80xx_read_packet(devpriv))
			break;

		data[n] = rx_buf[reg[0]] + 4 * rx_buf[reg[1]];
	}

	up(&devpriv->limit_sem);

	return n;
}

static int vmk80xx_pwm_insn_write(comedi_device *dev, comedi_subdevice *s,
				  comedi_insn *insn, lsampl_t *data)
{
	struct vmk80xx_private *devpriv = dev->private;
	unsigned char *tx_buf;
	int reg[2];
	int cmd;
	int n;

	dbgvm("vmk80xx: %s\n", __func__);

	down(&devpriv->limit_sem);

	tx_buf = devpriv->usb_tx_buf;

	reg[0] = VMK8061_PWM_REG1;
	reg[1] = VMK8061_PWM_REG2;

	cmd = VMK8061_CMD_OUT_PWM;

	/*
	 * The followin piece of code was translated from the inline
	 * assembler code in the DLL source code.
	 *
	 * asm
	 *   mov eax, k  ; k is the value (data[n])
	 *   and al, 03h ; al are the lower 8 bits of eax
	 *   mov lo, al  ; lo is the low part (tx_buf[reg[0]])
	 *   mov eax, k
	 *   shr eax, 2  ; right shift eax register by 2
	 *   mov hi, al  ; hi is the high part (tx_buf[reg[1]])
	 * end;
	 */
	for (n = 0; n < insn->n; n++) {
		tx_buf[reg[0]] = (unsigned char)(data[n] & 0x03);
		tx_buf[reg[1]] = (unsigned char)(data[n] >> 2) & 0xff;

		if (vmk80xx_write_packet(devpriv, cmd))
			break;
	}

	up(&devpriv->limit_sem);

	return n;
}

static int vmk80xx_alloc_usb_buffers(struct vmk80xx_private *devpriv)
{
	size_t size;

	size = le16_to_cpu(devpriv->ep_rx->wMaxPacketSize);
	devpriv->usb_rx_buf = kzalloc(size, GFP_KERNEL);
	if (!devpriv->usb_rx_buf)
		return -ENOMEM;

	size = le16_to_cpu(devpriv->ep_tx->wMaxPacketSize);
	devpriv->usb_tx_buf = kzalloc(size, GFP_KERNEL);
	if (!devpriv->usb_tx_buf) {
		kfree(devpriv->usb_rx_buf);
		devpriv->usb_rx_buf = NULL;
		return -ENOMEM;
	}

	return 0;
}

static int vmk80xx_attach(comedi_device *dev, comedi_devconfig *it)
{
	const struct vmk80xx_board *board;
	int i;
	struct vmk80xx_private *devpriv;
	int n_subd;
	comedi_subdevice *s;
	int minor;
	int ret;

	dbgvm("vmk80xx: %s\n", __func__);

	mutex_lock(&glb_mutex);

	for (i = 0; i < VMK80XX_MAX_BOARDS; i++)
		if (vmb[i].probed && !vmb[i].attached)
			break;

	if (i == VMK80XX_MAX_BOARDS) {
		mutex_unlock(&glb_mutex);
		return -ENODEV;
	}

	devpriv = &vmb[i];

	down(&devpriv->limit_sem);

	board = &vmk80xx_boardinfo[devpriv->model];
	dev->board_ptr = board;
	dev->board_name = board->name;
	dev->private = devpriv;

	if (devpriv->model == VMK8055_MODEL)
		n_subd = 5;
	else
		n_subd = 6;

	ret = alloc_subdevices(dev, n_subd);
	if (ret) {
		up(&devpriv->limit_sem);
		mutex_unlock(&glb_mutex);
		return ret;
	}

	/* Analog input subdevice */
	s = &dev->subdevices[0];
	s->type         = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE | SDF_GROUND;
	s->n_chan       = board->ai_nchans;
	s->maxdata      = board->ai_maxdata;
	s->range_table  = board->range;
	s->insn_read    = vmk80xx_ai_insn_read;

	/* Analog output subdevice */
	s = &dev->subdevices[1];
	s->type         = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITABLE | SDF_GROUND;
	s->n_chan       = board->ao_nchans;
	s->maxdata      = 0x00ff;
	s->range_table  = board->range;
	s->insn_write   = vmk80xx_ao_insn_write;

	if (devpriv->model == VMK8061_MODEL) {
		s->subdev_flags |= SDF_READABLE;
		s->insn_read    = vmk80xx_ao_insn_read;
	}

	/* Digital input subdevice */
	s = &dev->subdevices[2];
	s->type         = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE;
	s->n_chan       = board->di_nchans;
	s->maxdata      = 1;
	s->insn_bits    = vmk80xx_di_insn_bits;

	/* Digital output subdevice */
	s = &dev->subdevices[3];
	s->type         = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITABLE;
	s->n_chan       = 8;
	s->maxdata      = 1;
	s->insn_bits    = vmk80xx_do_insn_bits;

	if (devpriv->model == VMK8061_MODEL)
		s->subdev_flags |= SDF_READABLE;

	/* Counter subdevice */
	s = &dev->subdevices[4];
	s->type         = COMEDI_SUBD_COUNTER;
	s->subdev_flags = SDF_READABLE;
	s->n_chan       = 2;
	s->maxdata      = board->cnt_maxdata;
	s->insn_read    = vmk80xx_cnt_insn_read;
	s->insn_config  = vmk80xx_cnt_insn_config;

	if (devpriv->model == VMK8055_MODEL) {
		s->subdev_flags |= SDF_WRITABLE;
		s->insn_write   = vmk80xx_cnt_insn_write;
	}

	/* PWM subdevice */
	if (devpriv->model == VMK8061_MODEL) {
		s = &dev->subdevices[5];
		s->type         = COMEDI_SUBD_PWM;
		s->subdev_flags = SDF_READABLE | SDF_WRITABLE;
		s->n_chan       = board->pwm_nchans;
		s->maxdata      = board->pwm_maxdata;
		s->insn_read    = vmk80xx_pwm_insn_read;
		s->insn_write   = vmk80xx_pwm_insn_write;
	}

	devpriv->attached = 1;

	minor = dev->minor;

	printk(KERN_INFO
	       "comedi%d: vmk80xx: board #%d [%s] attached to comedi\n",
	       minor, devpriv->count, board->name);

	up(&devpriv->limit_sem);
	mutex_unlock(&glb_mutex);

	return 0;
}

static int vmk80xx_detach(comedi_device *dev)
{
	struct vmk80xx_private *devpriv;
	int minor;

	dbgvm("vmk80xx: %s\n", __func__);

	devpriv = dev->private;
	if (!devpriv)
		return 0;

	down(&devpriv->limit_sem);
	dev->private = NULL;
	devpriv->attached = 0;

	minor = dev->minor;

	printk(KERN_INFO
	       "comedi%d: vmk80xx: board #%d [%s] detached from comedi\n",
	       minor, devpriv->count, dev->board_name);

	up(&devpriv->limit_sem);

	return 0;
}

static comedi_driver driver_vmk80xx = {
	.module = THIS_MODULE,
	.driver_name = "vmk80xx",
	.attach = vmk80xx_attach,
	.detach = vmk80xx_detach
};

static int vmk80xx_find_usb_endpoints(struct vmk80xx_private *devpriv,
				      struct usb_interface *intf)
{
	struct usb_host_interface *iface_desc = intf->cur_altsetting;
	struct usb_endpoint_descriptor *ep_desc;
	int i;

	if (iface_desc->desc.bNumEndpoints != 2)
		return -ENODEV;

	if (le16_to_cpu(devpriv->ep_rx->wMaxPacketSize) == 0 ||
	    le16_to_cpu(devpriv->ep_tx->wMaxPacketSize) == 0)
		return -EINVAL;


	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		ep_desc = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_int_in(ep_desc) ||
		    usb_endpoint_is_bulk_in(ep_desc)) {
			if (!devpriv->ep_rx)
				devpriv->ep_rx = ep_desc;
			continue;
		}

		if (usb_endpoint_is_int_out(ep_desc) ||
		    usb_endpoint_is_bulk_out(ep_desc)) {
			if (!devpriv->ep_tx)
				devpriv->ep_tx = ep_desc;
			continue;
		}
	}

	if (!devpriv->ep_rx || !devpriv->ep_tx)
		return -ENODEV;

	return 0;
}

static int vmk80xx_usb_probe(struct usb_interface *intf,
			     const struct usb_device_id *id)
{
	const struct vmk80xx_board *board;
	struct vmk80xx_private *devpriv;
	int ret;
	int i;

	dbgvm("vmk80xx: %s\n", __func__);

	mutex_lock(&glb_mutex);

	for (i = 0; i < VMK80XX_MAX_BOARDS; i++)
		if (!vmb[i].probed)
			break;

	if (i == VMK80XX_MAX_BOARDS) {
		ret = -EMFILE;
		goto fail;
	}

	devpriv = &vmb[i];

	memset(devpriv, 0x00, sizeof(*devpriv));

	devpriv->count = i;

	ret = vmk80xx_find_usb_endpoints(devpriv, intf);
	if (ret)
		goto error;

	ret = vmk80xx_alloc_usb_buffers(devpriv);
	if (ret)
		goto error;

	devpriv->usb = interface_to_usbdev(intf);
	devpriv->intf = intf;

	sema_init(&devpriv->limit_sem, 8);

	usb_set_intfdata(intf, devpriv);

	devpriv->model = id->driver_info;
	board = &vmk80xx_boardinfo[devpriv->model];

	if (devpriv->model == VMK8061_MODEL) {
		vmk80xx_read_eeprom(devpriv, IC3_VERSION);
		dev_info(&intf->dev, "%s\n", devpriv->fw.ic3_vers);

		if (vmk80xx_check_data_link(devpriv)) {
			vmk80xx_read_eeprom(devpriv, IC6_VERSION);
			dev_info(&intf->dev, "%s\n", devpriv->fw.ic6_vers);
		} else {
			dbgcm("comedi#: vmk80xx: no conn. to CPU\n");
		}
	}

	if (devpriv->model == VMK8055_MODEL)
		vmk80xx_reset_device(devpriv);

	devpriv->probed = 1;

	dev_info(&intf->dev, "board #%d [%s] now attached\n",
		 devpriv->count, board->name);

	mutex_unlock(&glb_mutex);

	comedi_usb_auto_config(devpriv->usb, BOARDNAME);

	return 0;

error:
	/*
	 * Since 'devpriv' points to an element of the static vmb array
	 * we can't kfree it. Instead memset it to all '0' so subsequent
	 * usb probes don't find any garbage in it.
	 */
	memset(devpriv, 0x00, sizeof(*devpriv));
fail:
	mutex_unlock(&glb_mutex);
	return ret;
}

static void vmk80xx_usb_disconnect(struct usb_interface *intf)
{
	struct vmk80xx_private *devpriv = usb_get_intfdata(intf);
	const struct vmk80xx_board *board;

	dbgvm("vmk80xx: %s\n", __func__);

	if (!devpriv)
		return;

	board = &vmk80xx_boardinfo[devpriv->model];

	comedi_usb_auto_unconfig(devpriv->usb);

	mutex_lock(&glb_mutex);
	down(&devpriv->limit_sem);

	devpriv->probed = 0;
	usb_set_intfdata(devpriv->intf, NULL);

	kfree(devpriv->usb_rx_buf);
	kfree(devpriv->usb_tx_buf);

	dev_info(&intf->dev, "board #%d [%s] now detached\n",
		 devpriv->count, board->name);

	up(&devpriv->limit_sem);
	mutex_unlock(&glb_mutex);
}

static const struct usb_device_id vmk80xx_usb_id_table[] = {
	{USB_DEVICE(0x10cf, 0x5500), .driver_info = VMK8055_MODEL},
	{USB_DEVICE(0x10cf, 0x5501), .driver_info = VMK8055_MODEL},
	{USB_DEVICE(0x10cf, 0x5502), .driver_info = VMK8055_MODEL},
	{USB_DEVICE(0x10cf, 0x5503), .driver_info = VMK8055_MODEL},
	{USB_DEVICE(0x10cf, 0x8061), .driver_info = VMK8061_MODEL},
	{USB_DEVICE(0x10cf, 0x8062), .driver_info = VMK8061_MODEL},
	{USB_DEVICE(0x10cf, 0x8063), .driver_info = VMK8061_MODEL},
	{USB_DEVICE(0x10cf, 0x8064), .driver_info = VMK8061_MODEL},
	{USB_DEVICE(0x10cf, 0x8065), .driver_info = VMK8061_MODEL},
	{USB_DEVICE(0x10cf, 0x8066), .driver_info = VMK8061_MODEL},
	{USB_DEVICE(0x10cf, 0x8067), .driver_info = VMK8061_MODEL},
	{USB_DEVICE(0x10cf, 0x8068), .driver_info = VMK8061_MODEL},
	{}			/* terminating entry */
};
MODULE_DEVICE_TABLE(usb, vmk80xx_usb_id_table);

/* TODO: Add support for suspend, resume, pre_reset, * post_reset and flush */
static struct usb_driver vmk80xx_usb_driver = {
	.name		= "vmk80xx",
	.probe		= vmk80xx_usb_probe,
	.disconnect	= vmk80xx_usb_disconnect,
	.id_table	= vmk80xx_usb_id_table
};

static int __init vmk80xx_init(void)
{
	int retval;

	printk(KERN_INFO "vmk80xx: version 0.8.01 "
	       "Manuel Gebele <forensixs@gmx.de>\n");
	retval = comedi_driver_register(&driver_vmk80xx);
	if (retval < 0)
		return retval;

	retval = usb_register(&vmk80xx_usb_driver);
	if (retval < 0) {
		comedi_driver_unregister(&driver_vmk80xx);
		return retval;
	}

	return 0;
}

static void __exit vmk80xx_exit(void)
{
	comedi_driver_unregister(&driver_vmk80xx);
	usb_deregister(&vmk80xx_usb_driver);
}

module_init(vmk80xx_init);
module_exit(vmk80xx_exit);

MODULE_AUTHOR("Manuel Gebele <forensixs@gmx.de>");
MODULE_DESCRIPTION("Velleman USB Board Low-Level Driver");
MODULE_LICENSE("GPL");
