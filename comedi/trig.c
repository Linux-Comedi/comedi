/*
    module/trig.c
    back-compatibility functions

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2001 David A. Schleef <ds@schleef.org>

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
#include <kvmem.h>

#define __NO_VERSION__
#include <linux/module.h>

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/malloc.h>
#include <asm/io.h>

int command_trig(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
int mode_to_command(comedi_cmd *cmd,comedi_trig *it);
int mode0_emulate(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig);
int mode0_emulate_config(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig);



int mode0_emulate(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig)
{
	comedi_insn insn;
	lsampl_t ldata;
	int ret;

	insn.subdev=trig->subdev;
	insn.data=&ldata;
	insn.n=1;
	insn.chanspec=trig->chanlist[0];

	if(trig->flags&TRIG_CONFIG)
		return mode0_emulate_config(dev,s,trig);

	if(s->subdev_flags & SDF_WRITEABLE){
		if(s->subdev_flags & SDF_READABLE){
			if(trig->flags&TRIG_WRITE){
				insn.insn=INSN_WRITE;
			}else{
				insn.insn=INSN_READ;
			}
		}else{
			insn.insn=INSN_WRITE;
		}
	}else{
		insn.insn=INSN_READ;
	}

	switch(insn.insn){
	case INSN_READ:
		ret=s->insn_read(dev,s,&insn,&ldata);
		if(s->subdev_flags&SDF_LSAMPL){
			*(lsampl_t *)trig->data=ldata;
		}else{
			trig->data[0]=ldata;
		}
		return ret;
	case INSN_WRITE:
		if(s->subdev_flags&SDF_LSAMPL){
			ldata=*(lsampl_t *)trig->data;
		}else{
			ldata=trig->data[0];
		}
		return s->insn_write(dev,s,&insn,&ldata);
	default:
	}

	return -EINVAL;
}

int mode0_emulate_config(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig)
{
	comedi_insn insn;
	lsampl_t ldata;

	insn.subdev=trig->subdev;
	insn.data=&ldata;
	insn.n=1;
	insn.chanspec=trig->chanlist[0];

	ldata = trig->data[0];

	return s->insn_config(dev,s,&insn,&ldata);
}

int command_trig(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int ret;
	comedi_async *async = s->async;

	ret=mode_to_command(&async->cmd,it);
	if(ret)return ret;

	ret=s->do_cmdtest(dev,s,&async->cmd);
	if(ret)return -EINVAL;

	ret=s->do_cmd(dev,s);
	if(ret>0)return -EINVAL;
	return ret;
}

int mode_to_command(comedi_cmd *cmd,comedi_trig *it)
{
	memset(cmd,0,sizeof(comedi_cmd));
	cmd->subdev=it->subdev;
	cmd->chanlist_len=it->n_chan;
	cmd->chanlist=it->chanlist;
	cmd->data=it->data;
	cmd->data_len=it->data_len;

	cmd->start_src=TRIG_NOW;

	switch(it->mode){
	case 1:
		cmd->scan_begin_src=TRIG_FOLLOW;
		cmd->convert_src=TRIG_TIMER;
		cmd->convert_arg=it->trigvar;
		cmd->scan_end_src=TRIG_COUNT;
		cmd->scan_end_arg=it->n_chan;
		cmd->stop_src=TRIG_COUNT;
		cmd->stop_arg=it->n;
		
		break;
	case 2:
		cmd->scan_begin_src=TRIG_TIMER;
		cmd->scan_begin_arg=it->trigvar;
		cmd->convert_src=TRIG_TIMER;
		cmd->convert_arg=it->trigvar1;
		cmd->scan_end_src=TRIG_COUNT;
		cmd->scan_end_arg=it->n_chan;
		cmd->stop_src=TRIG_COUNT;
		cmd->stop_arg=it->n;
		
		break;
	case 3:
		cmd->scan_begin_src=TRIG_FOLLOW;
		cmd->convert_src=TRIG_EXT;
		cmd->convert_arg=it->trigvar;
		cmd->scan_end_src=TRIG_COUNT;
		cmd->scan_end_arg=it->n_chan;
		cmd->stop_src=TRIG_COUNT;
		cmd->stop_arg=it->n;

		break;
	case 4:
		cmd->scan_begin_src=TRIG_EXT;
		cmd->scan_begin_arg=it->trigvar;
		cmd->convert_src=TRIG_TIMER;
		cmd->convert_arg=it->trigvar1;
		cmd->scan_end_src=TRIG_COUNT;
		cmd->scan_end_arg=it->n_chan;
		cmd->stop_src=TRIG_COUNT;
		cmd->stop_arg=it->n;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

