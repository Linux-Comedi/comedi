/*
    kcomedilib/kcomedilib.c
    a comedlib interface for kernel modules

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2000 David A. Schleef <ds@stm.lbl.gov>

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
#ifdef LINUX_V22
#include <asm/uaccess.h>
#endif

#include <linux/comedidev.h>
#include <linux/comedi.h>
#include <linux/comedilib.h>


extern volatile int rtcomedi_lock_semaphore;





static inline int minor_to_dev(unsigned int minor,comedi_device **dev)
{
	if(minor>=COMEDI_NDEVICES)
		return -ENODEV;

	*dev=comedi_get_device_by_minor(minor);

	if(!(*dev)->attached)
		return -ENODEV;

	return 0;
}


/* this is strange */
static inline int minor_to_subdev(unsigned int minor,unsigned int subdevice,comedi_device **dev,comedi_subdevice **s)
{
	if ((minor_to_dev(minor, dev))!=0) 
		return -ENODEV;

        if (subdevice>(*dev)->n_subdevices)
		return -ENODEV;

	*s=(*dev)->subdevices+subdevice;

	return 0;
}


/* this is strange */
static inline int minor_to_subdevchan(unsigned int minor,unsigned int subdevice,comedi_device **dev,comedi_subdevice **s,unsigned int chan)
{
	int ret;
	
	if ((ret=minor_to_subdev(minor,subdevice,dev,s))!=0) 
		return ret;

	if (chan>=(*s)->n_chan)
		return -EINVAL;

	return 0;
}

int comedi_open(unsigned int minor)
{
	comedi_device *dev;

	if(minor>=COMEDI_NDEVICES)
		return -ENODEV;

	dev=comedi_get_device_by_minor(minor);

	if(!dev->attached)
		return -ENODEV;

	return minor;
}

void comedi_close(unsigned int minor)
{
}

/*
*/
int comedi_get_n_subdevices(unsigned int minor)
{
	comedi_device *dev;
	int ret;
	
	if ((ret=minor_to_dev(minor, &dev))!=0) 
		return ret;

	return dev->n_subdevices;
}

/*
*/
int comedi_get_version_code(unsigned int minor)
{
	comedi_device *dev;
	int ret;

	if ((ret=minor_to_dev(minor, &dev))!=0) 
		return ret;

	return COMEDI_VERSION_CODE;
}

/*
*/
char *comedi_get_driver_name(unsigned int minor)
{
	comedi_device *dev;
	int ret;

	if ((ret=minor_to_dev(minor, &dev))!=0) 
		return NULL;

	return dev->driver->driver_name;
}

/*
*/
char *comedi_get_board_name(unsigned int minor)
{
	comedi_device *dev;
	int ret;

	if ((ret=minor_to_dev(minor, &dev))!=0) 
		return NULL;

	return dev->board_name;
}

/*
*/
int comedi_get_subdevice_type(unsigned int minor,unsigned int subdevice)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret;

	if ((ret=minor_to_subdev(minor,subdevice,&dev,&s))!=0) 
		return ret;
		
	return s->type;
}

/*
 * ALPHA function
*/
unsigned int comedi_get_subdevice_flags(unsigned int minor,unsigned int subdevice)
{
	comedi_device *dev;
	comedi_subdevice *s;

	if (minor_to_subdev(minor,subdevice,&dev,&s)!=0)
		return 0;
		
	return s->subdev_flags;
}

/*
*/
int comedi_find_subdevice_by_type(unsigned int minor,int type,unsigned int subd)
{
	comedi_device *dev;
	int ret;

	if ((ret=minor_to_dev(minor, &dev))!=0)
		return ret;

        if (subd>dev->n_subdevices)
		return -ENODEV;

	for(;subd<dev->n_subdevices;subd++){
		if(dev->subdevices[subd].type==type)
			return subd;
	}
	return -1;
}

/*
*/
int comedi_get_n_channels(unsigned int minor,unsigned int subdevice)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret;

	if ((ret=minor_to_subdev(minor,subdevice,&dev,&s))!=0) 
		return ret;

	return s->n_chan;
}

/*
 * ALPHA function
*/
int comedi_get_len_chanlist(unsigned int minor,unsigned int subdevice)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret;

	if ((ret=minor_to_subdev(minor,subdevice,&dev,&s))!=0) 
		return ret;

	return s->len_chanlist;
}

/*
*/
lsampl_t comedi_get_maxdata(unsigned int minor,unsigned int subdevice,unsigned int chan)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret;

	if ((ret=minor_to_subdevchan(minor,subdevice,&dev,&s,chan))!=0) 
		return ret;
		
	if (s->maxdata_list)
		return s->maxdata_list[chan];
		
	return s->maxdata;
}

/*
 * DEPRECATED
*/
int comedi_get_rangetype(unsigned int minor,unsigned int subdevice,unsigned int chan)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret;

	if ((ret=minor_to_subdevchan(minor,subdevice,&dev,&s,chan))!=0) 
		return ret;
		
	if (s->range_table_list) {
		ret=s->range_table_list[chan]->length;
	} else {
		ret=s->range_table->length;
	}
	
	ret=ret|(minor<<28)|(subdevice<<24)|(chan<<16);

	return ret;
}

/*
*/
int comedi_get_n_ranges(unsigned int minor,unsigned int subdevice,unsigned int chan)
{
	int ret;

	if ((ret=comedi_get_rangetype(minor, subdevice, chan))<0)
		return ret;
		
	return RANGE_LENGTH(ret);
}

/*
 * ALPHA (non-portable)
*/
int comedi_get_krange(unsigned int minor,unsigned int subdevice,unsigned int chan,unsigned int range,comedi_krange *krange)
{
	comedi_device *dev;
	comedi_subdevice *s;
	comedi_lrange *lr;
	int ret;

	if ((ret=minor_to_subdevchan(minor,subdevice,&dev,&s,chan))!=0) 
		return ret;
		
	if (s->range_table_list) {
		lr=s->range_table_list[chan];
	} else {
		lr=s->range_table;
	}
	if (range>=lr->length) {
		return -EINVAL;
	}
	memcpy(krange,lr->range+range,sizeof(comedi_krange));

	return 0;
}

/*
 * ALPHA (may be renamed)
*/
unsigned int comedi_get_buf_head_pos(unsigned int minor,unsigned int subdevice)
{
	comedi_device *dev;
	comedi_subdevice *s;
	comedi_async *async;

	if (minor_to_subdev(minor,subdevice,&dev,&s)!=0)
		return 0;

	async = s->async;
	if(async == NULL) return 0;

	return async->buf_int_count;
}

/*
 * ALPHA (not necessary)
*/
int comedi_set_user_int_count(unsigned int minor,unsigned int subdevice,unsigned int buf_user_count)
{
	comedi_device *dev;
	comedi_subdevice *s;
	comedi_async *async;
	int ret;

	if ((ret=minor_to_subdev(minor,subdevice,&dev,&s))!=0)
		return ret;

	async = s->async;
	async->buf_user_count = buf_user_count;

	return 0;
}

int comedi_command(unsigned int minor,comedi_cmd *cmd)
{
	comedi_device *dev;
	comedi_subdevice *s;
	comedi_async *async;
	int ret;

	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;

	if(cmd->subdev>=dev->n_subdevices)
		return -ENODEV;

	s=dev->subdevices+cmd->subdev;
	async = s->async;
	if(s->type==COMEDI_SUBD_UNUSED)
		return -EIO;

	if(async == NULL)
		return -ENODEV;

	async->cb_mask = COMEDI_CB_EOA|COMEDI_CB_BLOCK|COMEDI_CB_ERROR;

	async->cmd=*cmd;

#if 0
	s->runflags=0;
#else
	s->runflags=SRF_RT;
	comedi_switch_to_rt(dev);
#endif

	s->subdev_flags |= SDF_RUNNING;

	async->buf_user_ptr=0;
	async->buf_user_count=0;
	async->buf_int_ptr=0;
	async->buf_int_count=0;

//	cur_trig deprecated, looks like all drivers have abandoned it by now - FMH
//	s->cur_trig.data_len = async->cmd.data_len;
//	s->cur_trig.data = async->cmd.data;

	async->data = cmd->data;
	async->data_len = cmd->data_len;

	return s->do_cmd(dev,s);
}

int comedi_command_test(unsigned int minor,comedi_cmd *cmd)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret;

	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;

	if(cmd->subdev>=dev->n_subdevices)
		return -ENODEV;

	s=dev->subdevices+cmd->subdev;
	if(s->type==COMEDI_SUBD_UNUSED)
		return -EIO;

	if(s->async == NULL)
		return -ENODEV;

	return s->do_cmdtest(dev,s,cmd);
}

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
static int comedi_trig_ioctl_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int comedi_trig_ioctl_modeN(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
int comedi_trigger(unsigned int minor,unsigned int subdev,comedi_trig *it)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret;

	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;

	if(it->subdev>=dev->n_subdevices)
		return -ENODEV;

	s=dev->subdevices+it->subdev;
	if(s->type==COMEDI_SUBD_UNUSED)
		return -EIO;

	if(it->mode==0)
		return comedi_trig_ioctl_mode0(dev,s,it);

	if(s->async == NULL)
		return -ENODEV;

	return comedi_trig_ioctl_modeN(dev,s,it);
}

static int comedi_trig_ioctl_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int ret=0;

	/* are we locked? (ioctl lock) */
	if(s->lock && s->lock!=&rtcomedi_lock_semaphore)
		return -EACCES;

	/* are we busy? */
	if(s->busy)
		return -EBUSY;
	s->busy=(void *)&rtcomedi_lock_semaphore;

	/* make sure channel/gain list isn't too long */
	if(it->n_chan > s->len_chanlist){
		ret = -EINVAL;
		goto cleanup;
	}

	/* make sure each element in channel/gain list is valid */
	if((ret=check_chanlist(s,it->n_chan,it->chanlist))<0)
		goto cleanup;

	if(it->data==NULL){
		ret=-EINVAL;
		goto cleanup;
	}

	if(!it->data_len){
#if 0
		ret=-EINVAL;
		goto cleanup;
#else
		it->data_len=it->n_chan*it->n*sizeof(sampl_t);
		rt_printk("comedi: warning: trig->data_len not set\n");
#endif
	}

	//s->cur_trig=*it;

	ret=s->trig[0](dev,s,it);

	if(ret>it->n*it->n_chan){
		rt_printk("comedi: (bug) trig returned too many samples\n");
	}

cleanup:
	s->busy=NULL;

	return ret;
}

static int comedi_trig_ioctl_modeN(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int ret=0;
	comedi_async *async = s->async;

	/* are we locked? (ioctl lock) */
	if(s->lock && s->lock!=&rtcomedi_lock_semaphore)
		return -EACCES;

	/* are we busy? */
	if(s->busy)
		return -EBUSY;
	s->busy=(void *)&rtcomedi_lock_semaphore;

	/* make sure channel/gain list isn't too long */
	if(it->n_chan > s->len_chanlist){
		ret = -EINVAL;
		goto cleanup;
	}

	if(async == NULL)
		return -ENODEV;

	/* make sure each element in channel/gain list is valid */
	if((ret=check_chanlist(s,it->n_chan,it->chanlist))<0)
		goto cleanup;

	async->buf_user_ptr=0;
	async->buf_user_count=0;
	async->buf_int_ptr=0;
	async->buf_int_count=0;

	if(it->data==NULL){
		ret=-EINVAL;
		goto cleanup;
	}

	if(!it->data_len){
#if 0
		ret=-EINVAL;
		goto cleanup;
#else
		it->data_len=it->n_chan*it->n*sizeof(sampl_t);
		rt_printk("comedi: warning: trig->data_len not set\n");
#endif
	}

	if(it->mode>=5 || s->trig[it->mode]==NULL){
		ret=-EINVAL;
		goto cleanup;
	}

//	s->cur_trig=*it;
	async->data = it->data;
	async->data_len = it->data_len;

	ret=s->trig[it->mode](dev,s,it);

	if(ret==0)return 0;

cleanup:
	s->busy=NULL;

	return ret;
}

/*
   This function does the same as above, but without any sanity
   checks.  Any insanity in code calling this function must be
   assumed by the writer.
 */
int __comedi_trigger(unsigned int minor,unsigned int subdev,comedi_trig *it)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret=0;

	dev=comedi_get_device_by_minor(minor);
	s=dev->subdevices+subdev;

//	s->cur_trig=*it;

	ret=s->trig[it->mode](dev,s,it);

	return ret;
}

/*
 *	COMEDI_INSN
 *	perform an instruction
 */
int comedi_do_insn(unsigned int minor,comedi_insn *insn)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret=0;

	dev=comedi_get_device_by_minor(minor);

	if(insn->insn&INSN_MASK_SPECIAL){
		switch(insn->insn){
		case INSN_GTOD:
		{
			struct timeval tv;
			lsampl_t data[2];

			do_gettimeofday(&tv);
			data[0] = tv.tv_sec;
			data[1] = tv.tv_usec;
			ret = 2;

			break;
		}
		case INSN_WAIT:
			if(insn->n<1 || insn->data[0]>=100){
				ret = -EINVAL;
				break;
			}
			udelay(insn->data[0]);
			ret=1;
			break;
		default:
			ret = -EINVAL;
		}
	}else{
		/* a subdevice instruction */
		if(insn->subdev>=dev->n_subdevices){
			ret = -EINVAL;
			goto error;
		}
		s = dev->subdevices+insn->subdev;

		if(s->type==COMEDI_SUBD_UNUSED){
			rt_printk("%d not useable subdevice\n",insn->subdev);
			goto error;
		}

		/* XXX check lock */

		if((ret=check_chanlist(s,1,&insn->chanspec))<0){
			rt_printk("bad chanspec\n");
			goto error;
		}

		if(s->busy){
			ret = -EBUSY;
			goto error;
		}
		s->busy = (void *)&rtcomedi_lock_semaphore;

		switch(insn->insn){
			case INSN_READ:
				ret = s->insn_read(dev,s,insn,insn->data);
				break;
			case INSN_WRITE:
				ret = s->insn_write(dev,s,insn,insn->data);
				break;
			case INSN_BITS:
				ret = s->insn_bits(dev,s,insn,insn->data);
				break;
			default:
				ret=-EINVAL;
				break;
		}

		s->busy = NULL;
	}
	if(ret<0)goto error;
	if(ret!=insn->n){
		rt_printk("BUG: result of insn != insn.n\n");
		ret = -EINVAL;
		goto error;
	}
error:
	
	return ret;
}

/*
	COMEDI_LOCK
	lock subdevice
	
	arg:
		subdevice number

	reads:
		none
	
	writes:
		none

	non-RT linux always controls rtcomedi_lock_semaphore.  If an
	RT-linux process wants the lock, it first checks rtcomedi_lock_semaphore.
	If it is 1, it knows it is pre-empting this function, and fails.
	Obviously, if RT-linux fails to get a lock, it *must* allow
	linux to run, since that is the only way to free the lock.
	
	This function is not SMP compatible.

	necessary locking:
	- ioctl/rt lock  (this type)
	- lock while subdevice busy
	- lock while subdevice being programmed
	
*/
int comedi_lock(unsigned int minor,unsigned int subdev)
{
	int ret=0;
	comedi_subdevice *s;
	comedi_device *dev;

	if(rtcomedi_lock_semaphore)
		return -EBUSY;
	
	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;
	
	if(subdev>=dev->n_subdevices)
		return -EINVAL;
	s=dev->subdevices+subdev;
	
	if(s->busy)
		return -EBUSY;

	/* &rtcomedi_lock_semaphore is just a convenient address */

	if(s->lock && s->lock!=&rtcomedi_lock_semaphore){
		ret=-EACCES;
	}else{
		__MOD_INC_USE_COUNT(dev->driver->module);
		s->lock=(void *)&rtcomedi_lock_semaphore;

		if(s->do_lock)
			s->do_lock(dev,s);
	}
	
	return ret;
}


/*
	COMEDI_UNLOCK
	unlock subdevice
	
	arg:
		subdevice number
	
	reads:
		none
	
	writes:
		none

*/
int comedi_unlock(unsigned int minor,unsigned int subdev)
{
	int ret=0;
	comedi_subdevice *s;
	comedi_device *dev;
	comedi_async *async;

	if(rtcomedi_lock_semaphore)
		return -EBUSY;

	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;

	if(subdev>=dev->n_subdevices)
		return -EINVAL;
	s=dev->subdevices+subdev;
	async = s->async;

	if(s->busy)
		return -EBUSY;

	if(s->lock && s->lock!=&rtcomedi_lock_semaphore)
		return -EACCES;

	if(s->do_unlock)
		s->do_unlock(dev,s);

	if(s->lock==&rtcomedi_lock_semaphore){
		s->lock=NULL;

		if(async){
			async->cb_mask=0;
			async->cb_func=NULL;
			async->cb_arg=NULL;
		}

		__MOD_DEC_USE_COUNT(dev->driver->module);
	}

	return 0;
}

/*
	COMEDI_CANCEL
	cancel acquisition ioctl

	arg:
		subdevice number

	reads:
		nothing

	writes:
		nothing

*/
int comedi_cancel(unsigned int minor,unsigned int subdev)
{
	int ret=0;
	comedi_subdevice *s;
	comedi_device *dev;

	if(rtcomedi_lock_semaphore)
		return -EBUSY;
	
	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;
	
	if(subdev>=dev->n_subdevices)
		return -EINVAL;
	s=dev->subdevices+subdev;
	
	if(s->lock && s->lock!=&rtcomedi_lock_semaphore)
		return -EACCES;
	
#if 0
	if(!s->busy)
		return 0;

	if(s->busy!=&rtcomedi_lock_semaphore)
		return -EBUSY;
#endif

	if(!s->cancel)
		return -EINVAL;

	if((ret=s->cancel(dev,s)))
		return ret;

	s->busy=NULL;

	return 0;
}
	
/*
   registration of callback functions
 */
int comedi_register_callback(unsigned int minor,unsigned int subdev,
		unsigned int mask,int (*cb)(unsigned int,void *),void *arg)
{
	comedi_device *dev;
	comedi_subdevice *s;
	comedi_async *async;
	int ret;

	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;

	if(subdev>=dev->n_subdevices)
		return -ENODEV;

	s=dev->subdevices+subdev;
	async = s->async;
	if(s->type==COMEDI_SUBD_UNUSED)
		return -EIO;

	/* are we locked? (ioctl lock) */
	if(s->lock && s->lock!=&rtcomedi_lock_semaphore)
		return -EACCES;

	/* are we busy? */
	if(s->busy)
		return -EBUSY;

	if(!mask){
		async->cb_mask=0;
		async->cb_func=NULL;
		async->cb_arg=NULL;
	}else{
		async->cb_mask=mask;
		async->cb_func=cb;
		async->cb_arg=arg;
	}

	return 0;
}


int comedi_poll(unsigned int minor, unsigned int subdev)
{
	comedi_device *dev;
	comedi_subdevice *s;
	comedi_async *async;
	int ret;

	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;

	if(subdev>=dev->n_subdevices)
		return -ENODEV;

	s=dev->subdevices+subdev;
	async = s->async;
	if(s->type==COMEDI_SUBD_UNUSED || !async)
		return -EIO;

	/* are we locked? (ioctl lock) */
	if(s->lock && s->lock!=&rtcomedi_lock_semaphore)
		return -EACCES;

	/* are we running? XXX wrong? */
	if(!s->busy)
		return -EIO;

	return s->poll(dev,s);
}


/* WARNING: not portable */
int comedi_map(unsigned int minor, unsigned int subdev, void **ptr)
{
	comedi_device *dev;
	comedi_subdevice *s;

	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;

	if(subdev>=dev->n_subdevices)
		return -ENODEV;

	s=dev->subdevices+subdev;
	if(!s->async)
		return -EINVAL;

	if(ptr)*ptr=s->async->prealloc_buf;

	/* XXX no reference counting */

	return 0;
}

/* WARNING: not portable */
int comedi_unmap(unsigned int minor, unsigned int subdev)
{
	comedi_device *dev;
	comedi_subdevice *s;

	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;

	if(subdev>=dev->n_subdevices)
		return -ENODEV;

	s=dev->subdevices+subdev;
	if(!s->async)
		return -EINVAL;

	/* XXX no reference counting */

	return 0;
}

