/*
    kcomedilib/dio.c
    implements comedi_dio_*() functions

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



#include <linux/comedidev.h>

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

int comedi_dio_config(unsigned int dev,unsigned int subdev,unsigned int chan,
	unsigned int io)
{
#if 0
	comedi_insn insn;

	memset(&insn,0,sizeof(insn));
	insn.insn = INSN_CONFIG;
	insn.n = 1;
	insn.data = &io;
	insn.subdev = subdev;
	insn.chanspec = CR_PACK(chan,0,0);

	return comedi_do_insn(dev,&insn);
#else
	comedi_trig cmd;
	sampl_t sdata = io;

	memset(&cmd,0,sizeof(cmd));

	cmd.flags = TRIG_CONFIG|TRIG_WRITE;
	cmd.n_chan = 1;
	cmd.n = 1;

	cmd.subdev = subdev;
	cmd.data = &sdata;
	cmd.data_len = sizeof(sampl_t);

	cmd.chanlist = &chan;

	return comedi_trigger(dev,subdev,&cmd);
#endif
}

int comedi_dio_read(unsigned int dev,unsigned int subdev,unsigned int chan,
	unsigned int *val)
{
#ifdef USE_INSN
	comedi_insn insn;

	memset(&insn,0,sizeof(insn));
	insn.insn = INSN_READ;
	insn.n = 1;
	insn.data = val;
	insn.subdev = subdev;
	insn.chanspec = CR_PACK(chan,0,0);

	return comedi_do_insn(dev,&insn);
#else
	comedi_trig cmd;
	sampl_t sdata;
	int ret;

	memset(&cmd,0,sizeof(cmd));

	cmd.n_chan = 1;
	cmd.n = 1;
	cmd.subdev = subdev;
	cmd.data = &sdata;
	cmd.data_len = sizeof(sampl_t);
	cmd.chanlist = &chan;

	ret = comedi_trigger(dev,subdev,&cmd);

	*val = sdata;

	return ret;
#endif
}

int comedi_dio_write(unsigned int dev,unsigned int subdev,unsigned int chan,
	unsigned int val)
{
#ifdef USE_INSN
	comedi_insn insn;

	memset(&insn,0,sizeof(insn));
	insn.insn = INSN_WRITE;
	insn.n = 1;
	insn.data = &val;
	insn.subdev = subdev;
	insn.chanspec = CR_PACK(chan,0,0);

	return comedi_do_insn(dev,&insn);
#else
	comedi_trig cmd;
	sampl_t sdata=val;
	int ret;

	memset(&cmd,0,sizeof(cmd));

	cmd.flags = TRIG_WRITE;
	cmd.n_chan = 1;
	cmd.n = 1;
	cmd.subdev = subdev;
	cmd.data = &sdata;
	cmd.data_len = sizeof(sampl_t);
	cmd.chanlist = &chan;

	ret = comedi_trigger(dev,subdev,&cmd);

	return ret;
#endif
}

int comedi_dio_bitfield(unsigned int minor,unsigned int subdev,unsigned int mask,
	unsigned int *bits)
{
#ifdef USE_INSN
	comedi_insn insn;
	lsampl_t data[2];
	int ret;

	memset(&insn,0,sizeof(insn));
	insn.insn = INSN_BITS;
	insn.n = 2;
	insn.data = data;
	insn.subdev = subdev;

	data[0] = mask;
	data[1] = *bits;

	ret = comedi_do_insn(minor,&insn);

	*bits = data[1];

	return ret;
#else
	int ret;
	unsigned int i,n_chan;
	unsigned int m,bit;
	comedi_subdevice *s;

	s=comedi_get_device_by_minor(minor)->subdevices+subdev;

	n_chan=s->n_chan;
	if(n_chan>32)n_chan=32;

	for(i=0,m=1;i<n_chan;i++,m<<=1){
		if(mask&m){
			bit=(*bits&m)?1:0;
			ret=comedi_dio_write(minor,subdev,i,bit);
		}else{
			ret=comedi_dio_read(minor,subdev,i,&bit);
			if(bit) *bits|=m;
			else (*bits)&=~m;
		}
		if(ret<0)return ret;
	}

	return (int)n_chan;
#endif
}



