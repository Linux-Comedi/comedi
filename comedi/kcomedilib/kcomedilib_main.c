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



#include <comedi_module.h>

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


extern volatile int rtcomedi_lock_semaphore;


#if 0
/* need more thot */
int comedi_devinfo_ioctl(unsigned int minor,comedi_devinfo *arg);
int comedi_subdinfo_ioctl(unsigned int minor,comedi_subdinfo *arg,void *file);
int comedi_chaninfo_ioctl(unsigned int minor,comedi_chaninfo *arg);
#endif



static inline int minor_to_dev(unsigned int minor,comedi_device **dev)
{
	if(minor>=COMEDI_NDEVICES)
		return -ENODEV;

	*dev=comedi_get_device_by_minor(minor);

	if(!(*dev)->attached)
		return -ENODEV;

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
   These functions are #if 0'd because they aren't appropriate
   inside RTLinux, at least, not in this form.  Interface needs
   thot.
 */
#if 0

/*
	COMEDI_DEVINFO
	device info ioctl
	
	arg:
		pointer to devinfo structure
	
	reads:
		none
	
	writes:
		devinfo structure
		
*/
static int do_devinfo_ioctl(comedi_device *dev,comedi_devinfo *arg)
{
	comedi_devinfo devinfo;
	
	
	/* fill devinfo structure */
	devinfo.version_code=COMEDI_VERSION_CODE;
	devinfo.n_subdevs=dev->n_subdevices;
	memcpy(devinfo.driver_name,dev->driver_name,COMEDI_NAMELEN);
	memcpy(devinfo.board_name,dev->board_name,COMEDI_NAMELEN);
	memcpy(devinfo.options,dev->options,COMEDI_NDEVCONFOPTS*sizeof(int));
	

	if(copy_to_user(arg,&devinfo,sizeof(comedi_devinfo)))
		return -EFAULT;

	return 0;
}


/*
	COMEDI_SUBDINFO
	subdevice info ioctl
	
	arg:
		pointer to array of subdevice info structures
	
	reads:
		none
	
	writes:
		array of subdevice info structures at arg
		
*/
static int do_subdinfo_ioctl(comedi_device *dev,comedi_subdinfo *arg,void *file)
{
	int ret,i;
	comedi_subdinfo *tmp,*us;
	comedi_subdevice *s;
	

	tmp=kmalloc(dev->n_subdevices*sizeof(comedi_subdinfo),GFP_KERNEL);
	if(!tmp)
		return -ENOMEM;
	
	/* fill subdinfo structs */
	for(i=0;i<dev->n_subdevices;i++){
		s=dev->subdevices+i;
		us=tmp+i;
		
		us->type		= s->type;
		us->n_chan		= s->n_chan;
		us->subd_flags		= s->subdev_flags;
		us->timer_type		= s->timer_type;
		us->len_chanlist	= s->len_chanlist;
		us->maxdata		= s->maxdata;
		us->range_type		= s->range_type;
		
		if(s->busy)
			us->subd_flags |= SDF_BUSY;
		if(s->busy == file)
			us->subd_flags |= SDF_BUSY_OWNER;
		if(s->lock)
			us->subd_flags |= SDF_LOCKED;
		if(s->lock == file)
			us->subd_flags |= SDF_LOCK_OWNER;
		if(s->maxdata_list)
			us->subd_flags |= SDF_MAXDATA;
		if(s->flaglist)
			us->subd_flags |= SDF_FLAGS;
		if(s->range_type_list)
			us->subd_flags |= SDF_RANGETYPE;

	}
	
	ret=copy_to_user(arg,tmp,dev->n_subdevices*sizeof(comedi_subdinfo));
	
	kfree(tmp);
	
	return ret?-EFAULT:0;
}


/*
	COMEDI_CHANINFO
	subdevice info ioctl
	
	arg:
		pointer to chaninfo structure
	
	reads:
		chaninfo structure at arg
	
	writes:
		arrays at elements of chaninfo structure
	
*/
static int do_chaninfo_ioctl(comedi_device *dev,comedi_chaninfo *arg)
{
	comedi_subdevice *s;
	comedi_chaninfo it;
	int ret;
	
	if(copy_from_user(&it,arg,sizeof(comedi_chaninfo)))
		return -EFAULT;
	
	if(it.subdev>=dev->n_subdevices)
		return -EINVAL;
	s=dev->subdevices+it.subdev;
	
	if(it.flaglist){
		if(s->subdev_flags & SDF_FLAGS)
			ret=copy_to_user(it.flaglist,s->flaglist,s->n_chan*sizeof(unsigned int));
		else
			ret=clear_user(it.flaglist,s->n_chan*sizeof(unsigned int));
		if(ret)return -EFAULT;
	}
			
	if(it.rangelist){
		if(s->subdev_flags & SDF_FLAGS)
			ret=copy_to_user(it.rangelist,s->range_type_list,s->n_chan*sizeof(unsigned int));
		else
			ret=clear_user(it.rangelist,s->n_chan*sizeof(unsigned int));
		if(ret)return -EFAULT;
	}
	
	return 0;
}
#endif


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
int comedi_trig_ioctl(unsigned int minor,unsigned int subdev,comedi_trig *it)
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
		it->data_len=it->n_chan*it->n*sizeof(sample_t);
		rt_printk("comedi: warning: trig->data_len not set\n");
#endif
	}

	s->cur_trig=*it;
	
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

	/* is subdevice RT capable? (important!) */
	if(!(s->subdev_flags&SDF_RT)){
		ret=-EINVAL;
		goto cleanup;
	}

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
	
	s->buf_user_ptr=0;
	s->buf_user_count=0;
	s->buf_int_ptr=0;
	s->buf_int_count=0;

	if(it->data==NULL){
		ret=-EINVAL;
		goto cleanup;
	}

	if(!it->data_len){
#if 0
		ret=-EINVAL;
		goto cleanup;
#else
		it->data_len=it->n_chan*it->n;
		rt_printk("comedi: warning: trig->data_len not set\n");
#endif
	}

	if(it->mode>=5 || s->trig[it->mode]==NULL){
		ret=-EINVAL;
		goto cleanup;
	}

	s->cur_trig=*it;
	
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
int __comedi_trig_ioctl(unsigned int minor,unsigned int subdev,comedi_trig *it)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret=0;

	dev=comedi_get_device_by_minor(minor);
	s=dev->subdevices+subdev;

	s->cur_trig=*it;
	
	ret=s->trig[it->mode](dev,s,it);
	
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
int comedi_lock_ioctl(unsigned int minor,unsigned int subdev)
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
int comedi_unlock_ioctl(unsigned int minor,unsigned int subdev)
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

	if(s->lock && s->lock!=&rtcomedi_lock_semaphore)
		return -EACCES;
	
	if(s->do_unlock)
		s->do_unlock(dev,s);

	if(s->lock==&rtcomedi_lock_semaphore){
		s->lock=NULL;

		s->cb_mask=0;
		s->cb_func=NULL;
		s->cb_arg=NULL;
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
int comedi_cancel_ioctl(unsigned int minor,unsigned int subdev)
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
	
	if(!s->busy)
		return 0;

	if(s->busy!=&rtcomedi_lock_semaphore)
		return -EBUSY;

	if(!s->cancel)
		return -EINVAL;

	if((ret=s->cancel(dev,s)))
		return ret;

	s->busy=NULL;

	return 0;
}
	
/*
   registration of callback functions

   XXX - this needs additional work.  Specifically, being SDF_RT is _not_ a
   sufficient condition for being able to do callbacks.
 */
int comedi_register_callback(unsigned int minor,unsigned int subdev,
		unsigned int mask,int (*cb)(unsigned int,void *),void *arg)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int ret;

	if((ret=minor_to_dev(minor,&dev))<0)
		return ret;
	
	if(subdev>=dev->n_subdevices)
		return -ENODEV;

	s=dev->subdevices+subdev;
	if(s->type==COMEDI_SUBD_UNUSED)
		return -EIO;
	
	/* is subdevice RT capable? (important!) */
	if(!(s->subdev_flags&SDF_RT))
		return -EINVAL;

	/* are we locked? (ioctl lock) */
	if(s->lock && s->lock!=&rtcomedi_lock_semaphore)
		return -EACCES;

	/* are we busy? */
	if(s->busy)
		return -EBUSY;

	if(!mask){
		s->cb_mask=0;
		s->cb_func=NULL;
		s->cb_arg=NULL;
	}else{
		s->cb_mask=mask;
		s->cb_func=cb;
		s->cb_arg=arg;
	}

	return 0;
}


