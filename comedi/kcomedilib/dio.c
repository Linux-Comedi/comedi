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


extern volatile int rtcomedi_lock_semaphore;

int comedi_dio_config(unsigned int dev,unsigned int subdev,unsigned int chan,
	unsigned int io)
{
	comedi_trig cmd;
	sampl_t sdata = io;

	memset(&cmd,0,sizeof(cmd));

	cmd.flags = TRIG_CONFIG|TRIG_WRITE;
	cmd.n_chan = 1;
	cmd.n = 1;

	cmd.subdev = subdev;
	cmd.data = &sdata;

	cmd.chanlist = &chan;

	return comedi_trig_ioctl(dev,subdev,&cmd);
}

int comedi_dio_read(unsigned int dev,unsigned int subdev,unsigned int chan,
	unsigned int *val)
{
	comedi_trig cmd;
	sampl_t sdata;
	int ret;

	memset(&cmd,0,sizeof(cmd));

	cmd.n_chan = 1;
	cmd.n = 1;
	cmd.subdev = subdev;
	cmd.data = &sdata;
	cmd.chanlist = &chan;

	ret = comedi_trig_ioctl(dev,subdev,&cmd);

	*val = sdata;

	return ret;
}

int comedi_dio_write(unsigned int dev,unsigned int subdev,unsigned int chan,
	unsigned int val)
{
	comedi_trig cmd;
	sampl_t sdata=val;
	int ret;

	memset(&cmd,0,sizeof(cmd));

	cmd.flags = TRIG_WRITE;
	cmd.n_chan = 1;
	cmd.n = 1;
	cmd.subdev = subdev;
	cmd.data = &sdata;
	cmd.chanlist = &chan;

	ret = comedi_trig_ioctl(dev,subdev,&cmd);

	return ret;
}

int comedi_dio_bitfield(unsigned int dev,unsigned int subdev,unsigned int mask,
	unsigned int *bits)
{
	int ret;
	unsigned int i,n_chan;
	unsigned int m,bit;
	comedi_subdevice *s;

	s=comedi_devices[dev].subdevices+subdev;

	n_chan=s->n_chan;
	if(n_chan>32)n_chan=32;

	for(i=0,m=1;i<n_chan;i++,m<<=1){
		if(mask&m){
			bit=(*bits&m)?1:0;
			ret=comedi_dio_write(dev,subdev,i,bit);
		}else{
			ret=comedi_dio_read(dev,subdev,i,&bit);
			if(bit) *bits|=m;
			else (*bits)&=~m;
		}
		if(ret<0)return ret;
	}

	return (int)n_chan;
}



