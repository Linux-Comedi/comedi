/*
    comedi/drivers/contec_fit.c

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
Driver:	contec_fit

Description:	Contec DAI12-4(FIT)GY Analog Output F&eIT Module
		Contec ADI16-4(FIT)GY Analog Input F&eIT Module
		Contec DIO-8/8(FIT)GY Digital I/O F&eIT Module

Devices:	[Contec] DAI12-4(FIT)GY (contec_fit)
		[Contec] ADI16-4(FIT)GY (contec_fit)
		[Contec] DIO-8/8(FIT)GY (contec_fit)

Author:	Contec Co.,Ltd

Updated:	Fri, 21 Apr 2017 14:30:00 +0900

Status:	works

Configuration Options:
  [0] - DeviceID of module (optional)
  If DeviceID is not specified, DeviceID 0 will be used.
*/

#include <linux/kthread.h>
#include <linux/comedidev.h>

#include "comedi_fc.h"

typedef struct {
	const char	*name;
	int		input_channels;
	int		output_channels;
	int		resolution;
} contec_fit;

#define thisboard ((const contec_board *)dev->board_ptr)

typedef struct {
	struct task_struct *fit_kthread;
} contec_fit_private;

#define devpriv ((contec_fit_private *)dev->private)

static int contec_attach(comedi_device * dev, comedi_devconfig * it);
static int contec_detach(comedi_device * dev);
static comedi_driver driver_contec = {
      driver_name:"contec_fit",
      module:THIS_MODULE,
      attach:contec_attach,
      detach:contec_detach,
};

/* Classic digital IO */
static int contec_ai_insn_read(comedi_device * dev, comedi_subdevice * s, comedi_insn * insn, lsampl_t * data);
static int contec_ao_insn_write(comedi_device * dev, comedi_subdevice * s, comedi_insn * insn, lsampl_t * data);
static int contec_di_insn_bits(comedi_device * dev, comedi_subdevice * s, comedi_insn * insn, lsampl_t * data);
static int contec_do_insn_bits(comedi_device * dev, comedi_subdevice * s, comedi_insn * insn, lsampl_t * data);

static	int contec_ai_cmdtest(comedi_device * dev, comedi_subdevice * s, comedi_cmd * cmd);
static int contec_ai_cmd(comedi_device * dev, comedi_subdevice * s);
static int contec_ai_sampling_thread(void * dev);
static int contec_ai_cancel(comedi_device * dev, comedi_subdevice * s);

static	int contec_ao_cmdtest(comedi_device * dev, comedi_subdevice * s, comedi_cmd * cmd);
static int contec_ao_cmd(comedi_device * dev, comedi_subdevice * s);
static int contec_ao_sampling_thread(void * dev);
static int contec_ao_cancel(comedi_device * dev, comedi_subdevice * s);
static int contec_ao_cmd_inttrig(comedi_device *dev, comedi_subdevice *s, unsigned int trignum);

static void contec_check_sampling_clock(unsigned int *sampling_clock, unsigned int *channel_num, int flags);

//#define DEBUG				//Output debug code

//START ADDRESS
#define ADDRESS_BASE						0x0800
#define ADDRESS_PER_ID					0x1000

//CONTEC FIT SIZE
#define CONTEC_FIT_SIZE					0x20

//COMMON IO
#define IO_PRODUCT_CATEGORY					0x00
#define IO_PRODUCT_ID_NUMBER				0x01
#define IO_COMMAND_DATA					0x18
#define IO_SETUP_DATA0					0x1C
#define IO_SETUP_DATA1					0x1D
#define IO_SETUP_DATA2					0x1E
#define IO_SETUP_DATA3					0x1F

//DAI12-4(FIT)GY IO
#define IO_DAI124_OUTPUTDATA_LOWER				0x10
#define IO_DAI124_OUTPUTDATA_UPPER				0x11
#define IO_DAI124_CHANNELDATA				0x12
#define IO_DAI124_STATUS					0x16

//ADI16-4(FIT)GY IO
#define IO_ADI164_INPUTDATA_LOWER				0x10
#define IO_ADI164_INPUTDATA_UPPER				0x11
#define IO_ADI164_CHANNELDATA				0x12
#define IO_ADI164_STATUS0					0x16
#define IO_ADI164_STATUS1					0x17

//DIO-8/8(FIT)GY IO
#define IO_DIO88_INPUT0					0x10
#define IO_DIO88_OUTPUT0					0x14

//DAI12-4(FIT)GY COMMAND
#define COMMAND_DAI124_INIT					0x00
#define COMMAND_DAI124_DASETUP				0x02
#define COMMAND_DAI124_OUTPUT_RANGE			0x03
#define COMMAND_DAI124_PASERCLOCK				0x04
#define COMMAND_DAI124_TIMER_START				0x05
#define COMMAND_DAI124_TIMER_STOP				0x06

//ADI16-4(FIT)GY COMMAND
#define COMMAND_ADI164_INIT					0x00
#define COMMAND_ADI164_SAMPLING_SETUP			0x02
#define COMMAND_ADI164_INPUT_RANGE				0x03
#define COMMAND_ADI164_SAMPLING_CLOCK			0x04
#define COMMAND_ADI164_TIMER_START				0x05
#define COMMAND_ADI164_TIMER_STOP				0x06
#define COMMAND_ADI164_FIFO_FLAG				0x07

//CATEGORY NUMBER
#define CATEGORY_AIO						0x20
#define CATEGORY_DIO						0x10

//PRODUCT ID NUMBER
#define PRODUCTID_DAI124					0x01
#define PRODUCTID_ADI164					0x02
#define PRODUCTID_DIO88					0x00

//SUBDEVICE DAI12-4(FIT)GY
#define SUBDEVICE_DAI124_AO					0x00

//SUBDEVICE ADI16-4(FIT)GY
#define SUBDEVICE_ADI164_AI					0x00

//SUBDEVICE DIO-8/8(FIT)GY
#define SUBDEVICE_DIO88_DI					0x00
#define SUBDEVICE_DIO88_DO					0x01

//RANGE
#define RANGE_BIPOLAR_10					0x00

//STATUS DAI12-4(FIT)GY
#define STATUS_DAI124_DSB					0x01
#define STATUS_DAI124_EOC					0x02
#define STATUS_DAI124_PCI					0x10
#define STATUS_DAI124_PCE					0x20

//STATUS ADI16-4(FIT)GY
#define STATUS_ADI164_DRE					0x01
#define STATUS_ADI164_DOE					0x04
#define STATUS_ADI164_SCE					0x20

//SAMPLING SETUP ADI16-4(FIT)GY
#define SAMPLING_SETUP_ADI164_CHANNELMODE_MULTI		0x04
#define SAMPLING_SETUP_ADI164_SAMPLINGCLOCK_INTERNAL	0x00
#define SAMPLING_SETUP_ADI164_SAMPLINGMODE_CLOCK		0x01

//FIFO FLAG ADI16-4(FIT)GY
#define FIFOFLAG_ADI164_DATACOUNT_1			0x01

//DACONVERT SETUP DAI12-4(FIT)GY
#define DACONVERT_SETUP_DAI124_SAMPLINGMODE_CLOCK	0x01
#define DACONVERT_SETUP_DAI124_OUTPUTMODE_SYNCOUT	0x04

//CHANNEL DATA DAI12-4(FIT)GY
#define CHANNELDATA_DAI124_ENDCHANNEL			0x40 

static int contec_attach(comedi_device * dev, comedi_devconfig * it)
{
	comedi_subdevice		*s;
	int				deviceid, device_address;
	int				product_category, product_id_number;

#ifdef DEBUG
	printk("comedi%d: contec: ", dev->minor);
#endif
	if (alloc_private(dev, sizeof(contec_fit_private)) < 0){
		return -ENOMEM;
	}

	deviceid = it->options[0];
	if(deviceid < 0 || deviceid > 7){
#ifdef DEBUG
		printk("comedi%d: contec: ID %d out of range\n", dev->minor, deviceid);
#endif
		return -EINVAL;
	}
	device_address	= ADDRESS_BASE + (ADDRESS_PER_ID * deviceid);
	if(!request_region(device_address, CONTEC_FIT_SIZE, "cotnec_fit")){
#ifdef DEBUG
		printk("comedi%d: contec: 0x%04x I/O port conflict\n", dev->minor, device_address);
#endif
		return -EIO;
	}
	product_category	= inb(device_address + IO_PRODUCT_CATEGORY);
	product_id_number	= inb(device_address + IO_PRODUCT_ID_NUMBER);
	dev->iobase		= device_address;
	devpriv->fit_kthread	= NULL;

	switch(product_category & 0xf0){
	case CATEGORY_AIO:
		switch(product_id_number){
		case PRODUCTID_DAI124:
			if (alloc_subdevices(dev, 1) < 0){
				return -ENOMEM;
			}
			dev->board_name	= "DAI12-4(FIT)GY";

			s			= dev->subdevices + SUBDEVICE_DAI124_AO;
			dev->write_subdev	= s;
			s->type		= COMEDI_SUBD_AO;
			s->subdev_flags	= SDF_WRITEABLE | SDF_CMD_WRITE;
			s->len_chanlist	= 4;
			s->n_chan		= 4;
			s->maxdata		= 0xfff;
			s->range_table	= &range_bipolar10;
			s->insn_write		= contec_ao_insn_write;
			s->do_cmdtest		= contec_ao_cmdtest;
			s->do_cmd		= &contec_ao_cmd;
			s->cancel		= contec_ao_cancel;

			return 1;
		case PRODUCTID_ADI164:
			if (alloc_subdevices(dev, 1) < 0){
				return -ENOMEM;
			}
			dev->board_name	= "ADI16-4(FIT)GY";

			s			= dev->subdevices + SUBDEVICE_ADI164_AI;
			dev->read_subdev	= s;
			s->type		= COMEDI_SUBD_AI;
			s->subdev_flags	= SDF_READABLE | SDF_CMD_READ;
			s->len_chanlist	= 4;
			s->n_chan		= 4;
			s->maxdata		= 0xffff;
			s->range_table	= &range_bipolar10;
			s->insn_read		= contec_ai_insn_read;
			s->do_cmdtest		= contec_ai_cmdtest;
			s->do_cmd		= contec_ai_cmd;
			s->cancel		= contec_ai_cancel;

			return 1;
		}
		break;
	case CATEGORY_DIO:
		switch(product_id_number){
		case PRODUCTID_DIO88:
			if (alloc_subdevices(dev, 2) < 0){
				return -ENOMEM;
			}
			dev->board_name	= "DIO-8/8(FIT)GY";

			s			= dev->subdevices + SUBDEVICE_DIO88_DI;
			s->type		= COMEDI_SUBD_DI;
			s->subdev_flags	= SDF_READABLE;
			s->n_chan		= 8;
			s->maxdata		= 1;
			s->range_table	= &range_digital;
			s->insn_bits		= contec_di_insn_bits;

			s			= dev->subdevices + SUBDEVICE_DIO88_DO;
			s->type		= COMEDI_SUBD_DO;
			s->subdev_flags	= SDF_WRITEABLE;
			s->n_chan		= 8;
			s->maxdata		= 1;
			s->range_table	= &range_digital;
			s->insn_bits		= contec_do_insn_bits;
			s->state		= inb(dev->iobase + IO_DIO88_OUTPUT0);
		
			return 1;
		}
		break;
	}			

	return -EIO;
}

static int contec_detach(comedi_device * dev)
{
#ifdef DEBUG
	printk("comedi%d: contec: remove\n", dev->minor);
#endif
	if(dev->iobase){
		release_region(dev->iobase, CONTEC_FIT_SIZE);
	}
	return 0;
}

static int contec_ai_insn_read(comedi_device * dev, comedi_subdevice * s, comedi_insn * insn, lsampl_t * data)
{
	int	lcount;
	int	i;
	int	channel;

#ifdef DEBUG
	printk("comedi%d: contec: contec_ai_insn_read called\n", dev->minor);
#endif
	channel = CR_CHAN(insn->chanspec);

	outb(COMMAND_ADI164_INIT, dev->iobase + IO_COMMAND_DATA);

	for(i = 0; i < insn->n; i++){
		outb(channel, dev->iobase + IO_ADI164_CHANNELDATA);
		lcount = 0;
		while(!(inb(dev->iobase + IO_ADI164_STATUS0) & STATUS_ADI164_DRE)){
			lcount++;
			if(lcount > 50000){
				return -EIO;
			}
			comedi_udelay(1);
		}
		data[i] = inb(dev->iobase + IO_ADI164_INPUTDATA_LOWER) + (inb(dev->iobase + IO_ADI164_INPUTDATA_UPPER) << 8);
	}

	return i;
}

static int contec_ao_insn_write(comedi_device * dev, comedi_subdevice * s, comedi_insn * insn, lsampl_t * data)
{
	int	lcount;
	int	i;
	int	channel;

#ifdef DEBUG
	printk("comedi%d: contec: contec_ao_insn_write called\n", dev->minor);
#endif
	channel = CR_CHAN(insn->chanspec);

	outb(COMMAND_DAI124_INIT, dev->iobase + IO_COMMAND_DATA);
	
	for(i = 0; i < insn->n; i++){
		lcount = 0;
		while((inb(dev->iobase + IO_DAI124_STATUS) & STATUS_DAI124_DSB)){
			lcount++;
			if(lcount > 50000){
				return -EIO;
			}
			comedi_udelay(1);
		}
		outb(channel, dev->iobase + IO_DAI124_CHANNELDATA);
		outb(data[i] & 0xff, dev->iobase + IO_DAI124_OUTPUTDATA_LOWER);
		outb(((data[i] & 0xff00) >> 8), dev->iobase + IO_DAI124_OUTPUTDATA_UPPER);
		lcount = 0;
		while(!(inb(dev->iobase + IO_DAI124_STATUS) & STATUS_DAI124_EOC)){
			lcount++;
			if(lcount > 50000){
				return -EIO;
			}
			comedi_udelay(1);
		}
		outb(STATUS_DAI124_EOC, dev->iobase + IO_DAI124_STATUS);
	}

	return i;
}

static int contec_ai_cmdtest(comedi_device * dev, comedi_subdevice * s, comedi_cmd * cmd)
{
	int	err = 0;
	int	tmp, i;
	
#ifdef DEBUG
	printk("comedi%d: contec: contec_ai_cmdtest called\n", dev->minor);
#endif
	tmp			= cmd->start_src;
	cmd->start_src	&= TRIG_NOW;
	if(!cmd->start_src || tmp != cmd->start_src){
		err++;
	}
	
	tmp			= cmd->scan_begin_src;
	cmd->scan_begin_src	&= TRIG_TIMER;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src){
		err++;
	}
	
	tmp			= cmd->convert_src;
	cmd->convert_src	&= TRIG_NOW;
	if(!cmd->convert_src || tmp != cmd->convert_src){
		err++;
	}
	
	tmp			= cmd->stop_src;
	cmd->stop_src		&= TRIG_COUNT | TRIG_NONE;
	if(!cmd->stop_src || tmp != cmd->stop_src){
		err++;
	}
	
	tmp			= cmd->scan_end_src;
	cmd->scan_end_src	&= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src){
		err++;
	}
	
	if(err){
		return 1;
	}
	
	if(cmd->stop_src != TRIG_COUNT && cmd->stop_src != TRIG_NONE){
		err++;
	}
	
	if(err){
		return 2;
	}

	if(cmd->start_arg != 0){
		cmd->start_arg = 0;
		err++;
	}
	
	if(!cmd->chanlist_len){
		cmd->chanlist_len = 1;
		err++;
	}else if(cmd->chanlist_len > 4){
		cmd->chanlist_len = 4;
		err++;
	}
	if(cmd->scan_begin_arg < 10000 * cmd->chanlist_len + 20000){
		cmd->scan_begin_arg = 10000 * cmd->chanlist_len + 20000;
		err++;
	}
	
	if(cmd->convert_arg != 0){
		cmd->convert_arg = 0;
		err++;
	}
	if(cmd->stop_src == TRIG_COUNT)
	{
		if(!cmd->stop_arg){
			cmd->stop_arg = 1;
			err++;
		}
	}else{
		if(cmd->stop_arg != 0){
			cmd->stop_arg = 0;
			err++;
		}
	}
	
	if(cmd->scan_end_arg != cmd->chanlist_len){
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	
	if(err){
		return 3;
	}
	
	tmp = cmd->scan_begin_arg;
	contec_check_sampling_clock(&cmd->scan_begin_arg, &cmd->chanlist_len, cmd->flags);
	if(tmp != cmd->scan_begin_arg){
		err++;
	}
	
	if(err){
		return 4;
	}

	if(cmd->chanlist){
		for(i = 0; i < cmd->chanlist_len; i++){
			if(CR_CHAN(cmd->chanlist[i]) != i){
				err++;
				break;
			}
		}
	}
	
	if(err){
		return 5;
	}

	return 0;
}

static int contec_ao_cmdtest(comedi_device * dev, comedi_subdevice * s, comedi_cmd * cmd)
{
	int	err = 0;
	int	tmp, i;
	
#ifdef DEBUG
	printk("comedi%d: contec: contec_ao_cmdtest called\n", dev->minor);
#endif
	tmp			= cmd->start_src;
	cmd->start_src	&= TRIG_INT;
	if(!cmd->start_src || tmp != cmd->start_src){
		err++;
	}
	
	tmp			= cmd->scan_begin_src;
	cmd->scan_begin_src	&= TRIG_TIMER;
	if(!cmd->scan_begin_src || tmp != cmd->scan_begin_src){
		err++;
	}
	
	tmp			= cmd->convert_src;
	cmd->convert_src	&= TRIG_NOW;
	if(!cmd->convert_src || tmp != cmd->convert_src){
		err++;
	}
	
	tmp			= cmd->stop_src;
	cmd->stop_src		&= TRIG_COUNT;
	if(!cmd->stop_src || tmp != cmd->stop_src){
		err++;
	}
	
	tmp			= cmd->scan_end_src;
	cmd->scan_end_src	&= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp != cmd->scan_end_src){
		err++;
	}
	
	if(err){
		return 1;
	}
	
	if(cmd->stop_src != TRIG_COUNT){
		err++;
	}
	
	if(err){
		return 2;
	}
	if(cmd->start_arg != 0){
		cmd->start_arg = 0;
		err++;
	}
	
	if(!cmd->chanlist_len){
		cmd->chanlist_len = 1;
		err++;
	}else if(cmd->chanlist_len > 4){
		cmd->chanlist_len = 4;
		err++;
	}
	if(cmd->scan_begin_arg < 10000 * cmd->chanlist_len + 20000){
		cmd->scan_begin_arg = 10000 * cmd->chanlist_len + 20000;
		err++;
	}
	
	if(cmd->convert_arg != 0){
		cmd->convert_arg = 0;
		err++;
	}
	if(cmd->stop_src == TRIG_COUNT)
	{
		if(!cmd->stop_arg){
			cmd->stop_arg = 1;
			err++;
		}
	}else{
		if(cmd->stop_arg != 0){
			cmd->stop_arg = 0;
			err++;
		}
	}
	
	if(cmd->scan_end_arg != cmd->chanlist_len){
		cmd->scan_end_arg = cmd->chanlist_len;
		err++;
	}
	
	if(err){
		return 3;
	}
	
	tmp = cmd->scan_begin_arg;
	contec_check_sampling_clock(&cmd->scan_begin_arg, &cmd->chanlist_len, cmd->flags);
	if(tmp != cmd->scan_begin_arg){
		err++;
	}
	
	if(err){
		return 4;
	}

	if(cmd->chanlist){
		for(i = 0; i < cmd->chanlist_len; i++){
			if(CR_CHAN(cmd->chanlist[i]) != i){
				err++;
				break;
			}
		}
	}
	
	if(err){
		return 5;
	}

	return 0;
}

static int contec_ai_cmd(comedi_device * dev, comedi_subdevice * s)
{
	comedi_async	*async	= s->async;
	comedi_cmd	*cmd	= &async->cmd;
	long		clockdata;
#ifdef DEBUG
	printk("comedi%d: contec: contec_ai_cmd colled\n", dev->minor);
#endif
	outb(COMMAND_ADI164_INIT, dev->iobase + IO_COMMAND_DATA);
	
	outb(COMMAND_ADI164_SAMPLING_SETUP, dev->iobase + IO_COMMAND_DATA);
	outb(	SAMPLING_SETUP_ADI164_CHANNELMODE_MULTI | 
		SAMPLING_SETUP_ADI164_SAMPLINGCLOCK_INTERNAL | 
		SAMPLING_SETUP_ADI164_SAMPLINGMODE_CLOCK, 
		dev->iobase + IO_SETUP_DATA0);
	
	outb(COMMAND_ADI164_FIFO_FLAG, dev->iobase + IO_COMMAND_DATA);
	outb(FIFOFLAG_ADI164_DATACOUNT_1, dev->iobase + IO_SETUP_DATA0);
	
	contec_check_sampling_clock(&cmd->scan_begin_arg, &cmd->chanlist_len, cmd->flags);

	clockdata = cmd->scan_begin_arg / 250 - 1;

	outb(COMMAND_ADI164_SAMPLING_CLOCK, dev->iobase + IO_COMMAND_DATA);
	outb(clockdata & 0xff, dev->iobase + IO_SETUP_DATA0);
	outb((clockdata >> 8) & 0xff, dev->iobase + IO_SETUP_DATA1);
	outb((clockdata >> 16) & 0xff, dev->iobase + IO_SETUP_DATA2);
	outb((clockdata >> 24) & 0xff, dev->iobase + IO_SETUP_DATA3);
	
	devpriv->fit_kthread = kthread_run(contec_ai_sampling_thread,  (void *)dev, "fit_ai");
	if(IS_ERR(devpriv->fit_kthread)){
		devpriv->fit_kthread	= NULL;
		return -EIO;
	}

	return 0;
}

static int contec_ao_cmd(comedi_device * dev, comedi_subdevice * s)
{
	comedi_async	*async	= s->async;
	comedi_cmd	*cmd	= &async->cmd;
	long		clockdata;
	
#ifdef DEBUG
	printk("comedi%d: contec: contec_ao_cmd called\n", dev->minor);
#endif
	outb(COMMAND_DAI124_INIT, dev->iobase + IO_COMMAND_DATA);
	
	outb(COMMAND_DAI124_DASETUP, dev->iobase + IO_COMMAND_DATA);
	outb(DACONVERT_SETUP_DAI124_SAMPLINGMODE_CLOCK | DACONVERT_SETUP_DAI124_OUTPUTMODE_SYNCOUT, dev->iobase + IO_SETUP_DATA0);
	
	contec_check_sampling_clock(&cmd->scan_begin_arg, &cmd->chanlist_len, cmd->flags);

	clockdata = cmd->scan_begin_arg / 250 - 1;
	outb(COMMAND_DAI124_PASERCLOCK, dev->iobase + IO_COMMAND_DATA);
	outb(clockdata & 0xff, dev->iobase + IO_SETUP_DATA0);
	outb((clockdata >> 8) & 0xff, dev->iobase + IO_SETUP_DATA1);
	outb((clockdata >> 16) & 0xff, dev->iobase + IO_SETUP_DATA2);
	outb((clockdata >> 24) & 0xff, dev->iobase + IO_SETUP_DATA3);

	if(cmd->start_src == TRIG_INT){
		s->async->inttrig = contec_ao_cmd_inttrig;
	}

	return 0;
}

static int contec_ao_cmd_inttrig(comedi_device *dev, comedi_subdevice *s, unsigned int trignum)
{
	if(trignum != 0){
		return -EINVAL;
	}

	s->async->inttrig = 0;
	devpriv->fit_kthread = kthread_run(contec_ao_sampling_thread,  (void *)dev, "fit_ao");
	if(IS_ERR(devpriv->fit_kthread)){
		devpriv->fit_kthread	= NULL;
		return -EIO;
	}

	return 1;
}

static int contec_do_insn_bits(comedi_device * dev, comedi_subdevice * s, comedi_insn * insn, lsampl_t * data)
{
#ifdef DEBUG
	printk("comedi%d: contec: contec_do_insn_bits called\n", dev->minor);
#endif
	if (insn->n != 2)
		return -EINVAL;

	if (data[0]){
		s->state &= ~data[0];
		s->state |= data[0] & data[1];
		outw(s->state, dev->iobase + IO_DIO88_OUTPUT0);
	}
	data[1] = s->state;

	return 2;
}

static int contec_di_insn_bits(comedi_device * dev, comedi_subdevice * s, comedi_insn * insn, lsampl_t * data)
{
#ifdef DEBUG
	rt_printk("comedi%d: contec: contec_di_insn_bits called\n", dev->minor);
#endif
	if (insn->n != 2)
		return -EINVAL;

	data[1] = inw(dev->iobase + IO_DIO88_INPUT0);

	return 2;
}

static void contec_check_sampling_clock(unsigned int *sampling_clock, unsigned int *channel_num, int flags)
{
	int clock_tmp;
	
	if(*channel_num < 1){
		*channel_num = 1;
	}else if(*channel_num > 4){
		*channel_num = 4;
	}
	if(*sampling_clock < ((10000 * *channel_num) + 20000)){
		*sampling_clock = ((10000 * *channel_num) + 20000);
	}
	
	clock_tmp = *sampling_clock % 250;
	if(clock_tmp != 0){
		switch(flags & TRIG_ROUND_MASK){
		case TRIG_ROUND_UP:
			*sampling_clock += 250 - clock_tmp;
			break;
		case TRIG_ROUND_DOWN:
			*sampling_clock -= clock_tmp;
			break;
		case TRIG_ROUND_NEAREST:
		default:
			if(clock_tmp < 126){
				*sampling_clock -= clock_tmp;
			}else{
				*sampling_clock += 250 - clock_tmp;
			}
			break;
		}
	}
}

static int contec_ai_sampling_thread(void * dev)
{
	comedi_device		*dev_tmp	= (comedi_device *)dev;
	comedi_subdevice	*s		= dev_tmp->read_subdev;
	comedi_async		*async		= s->async;
	comedi_cmd		*cmd		= &async->cmd;
	int			sampling_count;
	int			status;

#ifdef DEBUG
	printk("comedi%d: contec: contec_ai_sampling_thread called\n", dev->minor);
#endif
	outb((cmd->chanlist_len - 1), dev_tmp->iobase + IO_ADI164_CHANNELDATA);

	outb(COMMAND_ADI164_TIMER_START, dev_tmp->iobase + IO_COMMAND_DATA);
	
	sampling_count = 0;
	while(!kthread_should_stop()){
		schedule();
		async->events = 0;
		status = inb(dev_tmp->iobase + IO_ADI164_STATUS0);
		if(status & STATUS_ADI164_DRE){
			if(!cfc_write_to_buffer(s, (inb(dev_tmp->iobase + IO_ADI164_INPUTDATA_LOWER) + (inb(dev_tmp->iobase + IO_ADI164_INPUTDATA_UPPER) << 8)))){
				async->events = COMEDI_CB_ERROR | COMEDI_CB_EOA;
				break;
			}
			sampling_count++;
			if(cmd->stop_src == TRIG_COUNT){
				if((cmd->stop_arg * cmd->chanlist_len) <= sampling_count){
					async->events |= COMEDI_CB_EOA;
#ifdef DEBUG
					printk("comedi%d: contec: contec_ai_sampling_thread TRIG_COUNT\n", dev->minor);
#endif
					break;
				}
			}
			if(!(sampling_count % cmd->chanlist_len)){
				async->events = COMEDI_CB_BLOCK;
				comedi_event(dev_tmp, s);
			}
			continue;
		}
		if(status & (STATUS_ADI164_DOE | STATUS_ADI164_SCE)){
			async->events = COMEDI_CB_ERROR | COMEDI_CB_EOA;
#ifdef DEBUG
			printk("comedi%d: contec: contec_ai_sampling_thread ERROR = %d\n", dev->minor, status);
#endif
			break;
		}
	}

	outb(COMMAND_ADI164_TIMER_STOP, dev_tmp->iobase + IO_COMMAND_DATA);

	comedi_event(dev_tmp, s);
	
	return 0;
}

static int contec_ao_sampling_thread(void * dev)
{
	comedi_device		*dev_tmp	= (comedi_device *)dev;
	comedi_subdevice	*s		= dev_tmp->write_subdev;
	comedi_async		*async		= s->async;
	comedi_cmd		*cmd		= &async->cmd;
	int			sampling_count;
	int			channel_num;
	sampl_t		data;
	int			status;
	int			i;
	int			count;

#ifdef DEBUG
	printk("comedi%d: contec: contec_ao_sampling_thread called\n", dev->minor);
#endif

	sampling_count = 0;
	channel_num = 0;
	count = 0;

	status = inb(dev_tmp->iobase + IO_DAI124_STATUS);
	while((status & STATUS_DAI124_DSB)){
		count++;
		if(count > 50000){
			async->events |= COMEDI_CB_EOA;
			comedi_event(dev_tmp, s);
			return 0;
		}
	}

	for(i = 0; i < cmd->chanlist_len; i++){
		if(!comedi_buf_get(async, &data)){
			async->events = COMEDI_CB_ERROR | COMEDI_CB_EOA;
			comedi_event(dev_tmp, s);

			return 0;
		}
		if(i < (cmd->chanlist_len - 1)){
			outb(i, dev_tmp->iobase + IO_DAI124_CHANNELDATA);
		}else{
			outb(i | CHANNELDATA_DAI124_ENDCHANNEL, dev_tmp->iobase + IO_DAI124_CHANNELDATA);
		}
		outb(data & 0xff, dev_tmp->iobase + IO_DAI124_OUTPUTDATA_LOWER);
		outb(((data & 0xf00) >> 8), dev_tmp->iobase + IO_DAI124_OUTPUTDATA_UPPER);
	}

	outb(COMMAND_DAI124_TIMER_START, dev_tmp->iobase + IO_COMMAND_DATA);

	while(!kthread_should_stop()){
		schedule();
		async->events = 0;
		status = inb(dev_tmp->iobase + IO_DAI124_STATUS);
		if((status & STATUS_DAI124_EOC)){
			sampling_count++;
			if(cmd->stop_src == TRIG_COUNT){
				if(cmd->stop_arg <= sampling_count){
					async->events |= COMEDI_CB_EOA;
#ifdef DEBUG
					printk("comedi%d: contec: contec_ai_sampling_thread TRIG_COUNT\n", dev->minor);
#endif
					break;
				}
			}
			for(i = 0; i < cmd->chanlist_len; i++){
				if(!comedi_buf_get(async, &data)){
					async->events = COMEDI_CB_ERROR | COMEDI_CB_EOA;
					break;
				}
				if(i < (cmd->chanlist_len - 1)){
					outb(i, dev_tmp->iobase + IO_DAI124_CHANNELDATA);
				}else{
					outb(i | CHANNELDATA_DAI124_ENDCHANNEL, dev_tmp->iobase + IO_DAI124_CHANNELDATA);
				}
				outb(data & 0xff, dev_tmp->iobase + IO_DAI124_OUTPUTDATA_LOWER);
				outb(((data & 0xf00) >> 8), dev_tmp->iobase + IO_DAI124_OUTPUTDATA_UPPER);
			}
			outb(status & (STATUS_DAI124_EOC | STATUS_DAI124_PCI), dev_tmp->iobase + IO_DAI124_STATUS);
			async->events = COMEDI_CB_BLOCK;
			comedi_event(dev_tmp, s);

			continue;
		}
		if(status & STATUS_DAI124_PCE){
			async->events = COMEDI_CB_ERROR | COMEDI_CB_EOA;
#ifdef DEBUG
			printk("comedi%d: contec: contec_ai_sampling_thread ERROR = %d\n", dev->minor, status);
#endif
			break;
		}
	}

	outb(COMMAND_DAI124_TIMER_STOP, dev_tmp->iobase + IO_COMMAND_DATA);

	comedi_event(dev_tmp, s);
	
	return 0;
}

static int contec_ai_cancel(comedi_device * dev, comedi_subdevice * s)
{
#ifdef DEBUG
	printk("comedi%d: contec: contec_ai_cancel called\n", dev->minor);
#endif
	if(devpriv->fit_kthread != NULL){
		kthread_stop(devpriv->fit_kthread);
		devpriv->fit_kthread	= NULL;
	}

	return 0;
}

static int contec_ao_cancel(comedi_device * dev, comedi_subdevice * s)
{
#ifdef DEBUG
	printk("comedi%d: contec: contec_ao_cancel called\n", dev->minor);
#endif
	if(devpriv->fit_kthread != NULL){
		kthread_stop(devpriv->fit_kthread);
		devpriv->fit_kthread	= NULL;
	}

	return 0;
}

COMEDI_INITCLEANUP(driver_contec);
