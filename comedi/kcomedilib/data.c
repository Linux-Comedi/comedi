/*
    kcomedilib/data.c
    implements comedi_data_*() functions

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@stm.lbl.gov>

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



#include <comedi_module.h>

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/malloc.h>
#include <asm/io.h>
#ifdef LINUX_V22
#include <asm/uaccess.h>
#endif

#define USE_INSN

extern volatile int rtcomedi_lock_semaphore;

int comedi_data_write(unsigned int dev,unsigned int subdev,unsigned int chan,
	unsigned int range,unsigned int aref,lsampl_t data)
{
#ifdef USE_INSN
	comedi_insn insn;
	
	memset(&insn,0,sizeof(insn));
	insn.insn = INSN_WRITE;
	insn.n = 1;
	insn.data = &data;
	insn.subdev = subdev;
	insn.chanspec = CR_PACK(chan,range,aref);

	return comedi_do_insn(dev,&insn);
#else
	comedi_trig cmd;
	sampl_t sdata = data;

	memset(&cmd,0,sizeof(cmd));

	cmd.flags = TRIG_WRITE;
	cmd.n_chan = 1;
	cmd.n = 1;

	chan = CR_PACK(chan,range,aref);
	cmd.subdev = subdev;
	cmd.data = &sdata;
	cmd.data_len = sizeof(sampl_t);

	cmd.chanlist = &chan;

	return comedi_trig_ioctl(dev,subdev,&cmd);
#endif
}

int comedi_data_read(unsigned int dev,unsigned int subdev,unsigned int chan,
	unsigned int range,unsigned int aref,lsampl_t *data)
{
#ifdef USE_INSN
	comedi_insn insn;
	
	memset(&insn,0,sizeof(insn));
	insn.insn = INSN_READ;
	insn.n = 1;
	insn.data = data;
	insn.subdev = subdev;
	insn.chanspec = CR_PACK(chan,range,aref);

	return comedi_do_insn(dev,&insn);
#else
	comedi_trig cmd;
	int ret;
	sampl_t sdata;

	memset(&cmd,0,sizeof(cmd));

	cmd.n_chan = 1;
	cmd.n = 1;

	chan = CR_PACK(chan,range,aref);
	cmd.subdev = subdev;
	cmd.data = &sdata;
	cmd.data_len = sizeof(sampl_t);

	cmd.chanlist = &chan;

	ret = comedi_trig_ioctl(dev,subdev,&cmd);

	*data = sdata;

	return ret;
#endif
}

