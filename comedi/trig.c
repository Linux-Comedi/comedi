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
#include <asm/uaccess.h>
#include <asm/io.h>

int command_trig(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
int mode_to_command(comedi_cmd *cmd,comedi_trig *it);
int mode0_emulate(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig);
int mode0_emulate_config(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig);

void do_become_nonbusy(comedi_device *dev, comedi_subdevice *s);

/*
	COMEDI_TRIG
	trigger ioctl
	
	arg:
		pointer to trig structure
	
	reads:
		trig structure at arg
		channel/range list
	
	writes:
		modified trig structure at arg
		data list

	this function is too complicated
*/
static int do_trig_ioctl_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *user_trig);
static int do_trig_ioctl_modeN(comedi_device *dev,comedi_subdevice *s,comedi_trig *user_trig);
int do_trig_ioctl(comedi_device *dev,void *arg,void *file)
{
	comedi_trig user_trig;
	comedi_subdevice *s;

#if 0
DPRINTK("entering do_trig_ioctl()\n");
#endif
	if(copy_from_user(&user_trig,arg,sizeof(comedi_trig))){
		DPRINTK("bad trig address\n");
		return -EFAULT;
	}

#if 0
	/* this appears to be the only way to check if we are allowed
	   to write to an area. */
	if(copy_to_user(arg,&user_trig,sizeof(comedi_trig)))
		return -EFAULT;
#endif

	if(user_trig.subdev>=dev->n_subdevices){
		DPRINTK("%d no such subdevice\n",user_trig.subdev);
		return -ENODEV;
	}

	s=dev->subdevices+user_trig.subdev;
	if(s->type==COMEDI_SUBD_UNUSED){
		DPRINTK("%d not useable subdevice\n",user_trig.subdev);
		return -EIO;
	}

	/* are we locked? (ioctl lock) */
	if(s->lock && s->lock!=file){
		DPRINTK("device locked\n");
		return -EACCES;
	}

	/* are we busy? */
	if(s->busy){
		DPRINTK("device busy\n");
		return -EBUSY;
	}
	s->busy=file;

	s->cur_trig=user_trig;
	s->cur_trig.chanlist=NULL;
	s->cur_trig.data=NULL;

	if(user_trig.mode == 0){
		return do_trig_ioctl_mode0(dev,s,&user_trig);
	}else{
		return do_trig_ioctl_modeN(dev,s,&user_trig);
	}
}

static int do_trig_ioctl_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *user_trig)
{
	int reading;
	int bufsz;
	int ret=0,i;
//	comedi_async *async = s->async;

	/* make sure channel/gain list isn't too long */
	if(user_trig->n_chan > s->len_chanlist){
		DPRINTK("channel/gain list too long %d > %d\n",user_trig->n_chan,s->len_chanlist);
		ret = -EINVAL;
		goto cleanup;
	}

	/* load channel/gain list */
	s->cur_trig.chanlist=kmalloc(s->cur_trig.n_chan*sizeof(int),GFP_KERNEL);
	if(!s->cur_trig.chanlist){
		DPRINTK("allocation failed\n");
		ret = -ENOMEM;
		goto cleanup;
	}

	if(copy_from_user(s->cur_trig.chanlist,user_trig->chanlist,s->cur_trig.n_chan*sizeof(int))){
		DPRINTK("fault reading chanlist\n");
		ret = -EFAULT;
		goto cleanup;
	}
	
	/* make sure each element in channel/gain list is valid */
	if((ret=check_chanlist(s,s->cur_trig.n_chan,s->cur_trig.chanlist))<0){
		DPRINTK("bad chanlist\n");
		goto cleanup;
	}

	/* allocate temporary buffer */
	if(s->subdev_flags&SDF_LSAMPL){
		bufsz=s->cur_trig.n*s->cur_trig.n_chan*sizeof(lsampl_t);
	}else{
		bufsz=s->cur_trig.n*s->cur_trig.n_chan*sizeof(sampl_t);
	}

	if(!(s->cur_trig.data=kmalloc(bufsz,GFP_KERNEL))){
		DPRINTK("failed to allocate buffer\n");
		ret=-ENOMEM;
		goto cleanup;
	}

// this stuff isn't used in mode0
//	async->buf_int_ptr=0;
//	async->buf_int_count=0;
//	if(s->subdev_flags & SDF_READABLE){
//		async->buf_user_ptr=0;
//		async->buf_user_count=0;
//	}

	if(s->subdev_flags & SDF_WRITEABLE){
		if(s->subdev_flags & SDF_READABLE){
			/* bidirectional, so we defer to trig structure */
			if(user_trig->flags&TRIG_WRITE){
				reading=0;
			}else{
				reading=1;
			}
		}else{
			reading=0;
		}
	}else{
		/* subdev is read-only */
		reading=1;
	}
	if(!reading && user_trig->data){
		if(s->subdev_flags&SDF_LSAMPL){
			i=s->cur_trig.n*s->cur_trig.n_chan*sizeof(lsampl_t);
		}else{
			i=s->cur_trig.n*s->cur_trig.n_chan*sizeof(sampl_t);
		}
		if(copy_from_user(s->cur_trig.data,user_trig->data,i)){
			DPRINTK("bad address %p,%p (%d)\n",s->cur_trig.data,user_trig->data,i);
			ret=-EFAULT;
			goto cleanup;
		}
	}

	ret = mode0_emulate(dev,s,&s->cur_trig);

	if(ret<0)goto cleanup;

	if(s->subdev_flags&SDF_LSAMPL){
		i=ret*sizeof(lsampl_t);
	}else{
		i=ret*sizeof(sampl_t);
	}
	if(i>bufsz){
		printk("comedi: (bug) trig returned too many samples\n");
		i=bufsz;
	}
	if(reading){
		if(copy_to_user(user_trig->data,s->cur_trig.data,i)){
			ret=-EFAULT;
			goto cleanup;
		}
	}
cleanup:

	s->busy=NULL;
	//do_become_nonbusy(dev,s);

	return ret;
}

static int do_trig_ioctl_modeN(comedi_device *dev,comedi_subdevice *s,comedi_trig *user_trig)
{
	int ret=0;
	comedi_async *async = s->async;

	if(async == NULL)
	{
		DPRINTK("subdevice has no buffer, trig failed\n");
		return -ENODEV;
		goto cleanup;
	}

	if(s->cur_trig.mode>=5 || s->trig[s->cur_trig.mode]==NULL){
		DPRINTK("bad mode %d\n",s->cur_trig.mode);
		ret=-EINVAL;
		goto cleanup;
	}

	/* make sure channel/gain list isn't too long */
	if(user_trig->n_chan > s->len_chanlist){
		DPRINTK("channel/gain list too long %d > %d\n",user_trig->n_chan,s->len_chanlist);
		ret = -EINVAL;
		goto cleanup;
	}

	/* load channel/gain list */
	s->cur_trig.chanlist=kmalloc(s->cur_trig.n_chan*sizeof(int),GFP_KERNEL);
	if(!s->cur_trig.chanlist){
		DPRINTK("allocation failed\n");
		ret = -ENOMEM;
		goto cleanup;
	}

	if(copy_from_user(s->cur_trig.chanlist,user_trig->chanlist,s->cur_trig.n_chan*sizeof(int))){
		DPRINTK("fault reading chanlist\n");
		ret = -EFAULT;
		goto cleanup;
	}

	/* make sure each element in channel/gain list is valid */
	if((ret=check_chanlist(s,s->cur_trig.n_chan,s->cur_trig.chanlist))<0){
		DPRINTK("bad chanlist\n");
		goto cleanup;
	}

	if(!s->async->prealloc_buf){
		printk("comedi: bug: s->async->prealloc_buf==NULL\n");
	}

//	s->cur_trig.data=async->prealloc_buf;
//	s->cur_trig.data_len=async->prealloc_bufsz;
	async->data=async->prealloc_buf;
	async->data_len=async->prealloc_bufsz;

	async->buf_int_ptr=0;
	async->buf_int_count=0;
	if(s->subdev_flags & SDF_READABLE){
		async->buf_user_ptr=0;
		async->buf_user_count=0;
	}

	async->cur_chan=0;
	async->cur_chanlist_len=s->cur_trig.n_chan;

	async->cb_mask=COMEDI_CB_EOA|COMEDI_CB_BLOCK|COMEDI_CB_ERROR;
	if(s->cur_trig.flags & TRIG_WAKE_EOS){
		async->cb_mask|=COMEDI_CB_EOS;
	}

	s->runflags=SRF_USER;

	s->subdev_flags|=SDF_RUNNING;

	ret=s->trig[s->cur_trig.mode](dev,s,&s->cur_trig);

	if(ret==0)return 0;

cleanup:
	do_become_nonbusy(dev,s);

	return ret;
}

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

