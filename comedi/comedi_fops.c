/*
    comedi/comedi_fops.c
    comedi kernel module

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

#undef DEBUG

#include <linux/comedidev.h>

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
#include <linux/kmod.h>
#include <asm/uaccess.h>
#endif
#if LINUX_VERSION_CODE >= 0x020100
#include <linux/poll.h>
#endif
#include <kvmem.h>

comedi_device *comedi_devices;

static int do_devconfig_ioctl(comedi_device *dev,comedi_devconfig *arg,kdev_t minor);
static int do_bufconfig_ioctl(comedi_device *dev,void *arg);
static int do_devinfo_ioctl(comedi_device *dev,comedi_devinfo *arg);
static int do_subdinfo_ioctl(comedi_device *dev,comedi_subdinfo *arg,void *file);
static int do_chaninfo_ioctl(comedi_device *dev,comedi_chaninfo *arg);
#ifdef CONFIG_COMEDI_MODE_CORE
static int do_trig_ioctl(comedi_device *dev,void *arg,void *file);
#endif
static int do_cmd_ioctl(comedi_device *dev,void *arg,void *file);
static int do_lock_ioctl(comedi_device *dev,unsigned int arg,void * file);
static int do_unlock_ioctl(comedi_device *dev,unsigned int arg,void * file);
static int do_cancel_ioctl(comedi_device *dev,unsigned int arg,void *file);
static int do_cmdtest_ioctl(comedi_device *dev,void *arg,void *file);
static int do_insnlist_ioctl(comedi_device *dev,void *arg,void *file);

static void do_become_nonbusy(comedi_device *dev,comedi_subdevice *s);
int resize_buf(comedi_device *dev,comedi_subdevice *s, unsigned int size);

static int comedi_ioctl(struct inode * inode,struct file * file,unsigned int cmd,unsigned long arg)
{
	kdev_t minor=MINOR(inode->i_rdev);
	comedi_device *dev=comedi_get_device_by_minor(minor);
	
	switch(cmd)
	{
	case COMEDI_DEVCONFIG:
		return do_devconfig_ioctl(dev,(void *)arg,minor);
	case COMEDI_BUFCONFIG:
		return do_bufconfig_ioctl(dev,(void*)arg);
	case COMEDI_DEVINFO:
		return do_devinfo_ioctl(dev,(void *)arg);
	case COMEDI_SUBDINFO:
		return do_subdinfo_ioctl(dev,(void *)arg,file);
	case COMEDI_CHANINFO:
		return do_chaninfo_ioctl(dev,(void *)arg);
	case COMEDI_RANGEINFO:
		return do_rangeinfo_ioctl(dev,(void *)arg);
#ifdef CONFIG_COMEDI_MODE_CORE
	case COMEDI_TRIG:
		return do_trig_ioctl(dev,(void *)arg,file);
#endif
	case COMEDI_LOCK:
		return do_lock_ioctl(dev,arg,file);
	case COMEDI_UNLOCK:
		return do_unlock_ioctl(dev,arg,file);
	case COMEDI_CANCEL:
		return do_cancel_ioctl(dev,arg,file);
	case COMEDI_CMD:
		return do_cmd_ioctl(dev,(void *)arg,file);
	case COMEDI_CMDTEST:
		return do_cmdtest_ioctl(dev,(void *)arg,file);
	case COMEDI_INSNLIST:
		return do_insnlist_ioctl(dev,(void *)arg,file);
	default:
		return -EIO;
	}
}


/*
	COMEDI_DEVCONFIG
	device config ioctl
	
	arg:
		pointer to devconfig structure
	
	reads:
		devconfig structure at arg
	
	writes:
		none
*/
static int do_devconfig_ioctl(comedi_device *dev,comedi_devconfig *arg,kdev_t minor)
{
	comedi_devconfig it;
	
	if(!suser())
		return -EPERM;

	if(arg==NULL){
		return comedi_device_detach(dev);
	}

	if(copy_from_user(&it,arg,sizeof(comedi_devconfig)))
		return -EFAULT;
	
	it.board_name[COMEDI_NAMELEN-1]=0;

	return comedi_device_attach(dev,&it);
}

/*
	COMEDI_BUFCONFIG
	buffer configuration ioctl

	arg:
		pointer to bufconfig structure

	reads:
		bufconfig at arg

	writes:
		modified bufconfig at arg

*/
static int do_bufconfig_ioctl(comedi_device *dev,void *arg)
{
	comedi_bufconfig bc;
	comedi_subdevice *rsd=NULL, *wsd=NULL;
	int read_ret = 0, write_ret = 0;

	if(!suser())
		return -EPERM;

	// perform sanity checks
	if(!dev->attached)
		return -EINVAL;

	if(copy_from_user(&bc,arg,sizeof(comedi_bufconfig)))
		return -EFAULT;

	// Should check to see if buffer is memory mapped and avoid
	// changing buffer if it is.  (Have to wait until a mapped flag
	// gets added to subdevice struct.)

	if(bc.read_size){
		rsd = &dev->subdevices[dev->read_subdev];

		if(rsd->busy)
			return -EBUSY;

		if(!rsd->prealloc_buf)
			return -EINVAL;
	}

	if(bc.write_size && dev->read_subdev != dev->write_subdev){
		wsd = &dev->subdevices[dev->write_subdev];

		if(wsd->busy)
			return -EBUSY;

		if(!wsd->prealloc_buf)
			return -EINVAL;
	}

	// resize buffers
	if(rsd){
		read_ret = resize_buf(dev,rsd,bc.read_size);
		bc.read_size = rsd->prealloc_bufsz;
		DPRINTK("dev %i read buffer resized to %i bytes\n", dev->minor, bc.read_size);
	}

	if(wsd){
		write_ret = resize_buf(dev,wsd,bc.write_size);
		bc.write_size = wsd->prealloc_bufsz;
		DPRINTK("dev %i write buffer resized to %i bytes\n", dev->minor, bc.write_size);
	}
	else bc.write_size = 0;


	if(copy_to_user(arg,&bc,sizeof(comedi_bufconfig)))
		return -EFAULT;

	if(read_ret < 0 || write_ret < 0)
		return -ENOMEM;

	return 0;
}

/* utility function that resizes the prealloc_buf for
 * a subdevice
 */
int resize_buf(comedi_device *dev, comedi_subdevice *s, unsigned int size)
{
	void *old_buf;

	// make sure buffer is an integral number of pages (we round up)
	size = ((size + PAGE_SIZE - 1) / PAGE_SIZE) * PAGE_SIZE;

	// if no change is required, do nothing
	if(s->prealloc_buf && s->prealloc_bufsz){
		if(s->prealloc_bufsz == size)
			return 0;
	}

	old_buf = s->prealloc_buf;
	s->prealloc_buf = rvmalloc(size);
	// restore old buffer on error
	if(s->prealloc_buf == 0){
		s->prealloc_buf = old_buf;
		return -ENOMEM;
	}

	rvfree(old_buf, s->prealloc_bufsz);
	s->prealloc_bufsz = size;

	return 0;
}

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
	memcpy(devinfo.driver_name,dev->driver->driver_name,COMEDI_NAMELEN);
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
	
	memset(tmp,0,sizeof(comedi_subdinfo)*dev->n_subdevices);

	/* fill subdinfo structs */
	for(i=0;i<dev->n_subdevices;i++){
		s=dev->subdevices+i;
		us=tmp+i;
		
		us->type		= s->type;
		us->n_chan		= s->n_chan;
		us->subd_flags		= s->subdev_flags;
#define TIMER_nanosec 5	/* backwards compatibility */
		us->timer_type		= TIMER_nanosec;
		us->len_chanlist	= s->len_chanlist;
		us->maxdata		= s->maxdata;
		if(s->range_table){
			us->range_type	= (dev->minor<<28)|(i<<24)|(0<<16)|
					(s->range_table->length);
		}else{
			us->range_type	= 0; /* XXX */
		}
		us->flags		= s->flags;
		
		if(s->busy)
			us->subd_flags |= SDF_BUSY;
		if(s->busy == file)
			us->subd_flags |= SDF_BUSY_OWNER;
		if(s->lock)
			us->subd_flags |= SDF_LOCKED;
		if(s->lock == file)
			us->subd_flags |= SDF_LOCK_OWNER;
		if(!s->maxdata && s->maxdata_list)
			us->subd_flags |= SDF_MAXDATA;
		if(s->flaglist)
			us->subd_flags |= SDF_FLAGS;
		if(s->range_table_list)
			us->subd_flags |= SDF_RANGETYPE;
#ifdef CONFIG_COMEDI_MODE_CORE
		if(s->trig[0])
			us->subd_flags |= SDF_MODE0;
		if(s->trig[1])
			us->subd_flags |= SDF_MODE1;
		if(s->trig[2])
			us->subd_flags |= SDF_MODE2;
		if(s->trig[3])
			us->subd_flags |= SDF_MODE3;
		if(s->trig[4])
			us->subd_flags |= SDF_MODE4;
#endif
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
	
	if(copy_from_user(&it,arg,sizeof(comedi_chaninfo)))
		return -EFAULT;
	
	if(it.subdev>=dev->n_subdevices)
		return -EINVAL;
	s=dev->subdevices+it.subdev;
	
	if(it.maxdata_list){
		if(s->maxdata || !s->maxdata_list)
			return -EINVAL;
		if(copy_to_user(it.maxdata_list,s->maxdata_list,s->n_chan*sizeof(lsampl_t)))
			return -EFAULT;
	}

	if(it.flaglist){
		if(!s->flaglist)return -EINVAL;
		if(copy_to_user(it.flaglist,s->flaglist,s->n_chan*sizeof(unsigned int)))
			return -EFAULT;
	}
			
	if(it.rangelist){
		int i;

		if(!s->range_table_list)return -EINVAL;
		for(i=0;i<s->n_chan;i++){
			int x;

			x=(dev->minor<<28)|(it.subdev<<24)|(i<<16)|
				(s->range_table_list[i]->length);
			put_user(x,it.rangelist+i);
		}
		//if(copy_to_user(it.rangelist,s->range_type_list,s->n_chan*sizeof(unsigned int)))
		//	return -EFAULT;
	}
	
	return 0;
}


#ifdef CONFIG_COMEDI_MODE_CORE
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
static int do_trig_ioctl(comedi_device *dev,void *arg,void *file)
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

	s->buf_int_ptr=0;
	s->buf_int_count=0;
	if(s->subdev_flags & SDF_READABLE){
		s->buf_user_ptr=0;
		s->buf_user_count=0;
	}

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

	ret=s->trig[0](dev,s,&s->cur_trig);

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

	do_become_nonbusy(dev,s);
	
	return ret;
}

static int do_trig_ioctl_modeN(comedi_device *dev,comedi_subdevice *s,comedi_trig *user_trig)
{
	int ret=0;

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

	if(!s->prealloc_buf){
		printk("comedi: bug: s->prealloc_buf=NULL\n");
	}
	s->cur_trig.data=s->prealloc_buf;
	s->cur_trig.data_len=s->prealloc_bufsz;

	s->buf_int_ptr=0;
	s->buf_int_count=0;
	if(s->subdev_flags & SDF_READABLE){
		s->buf_user_ptr=0;
		s->buf_user_count=0;
	}

	s->cur_chan=0;
	s->cur_chanlist_len=s->cur_trig.n_chan;

	s->cb_mask=COMEDI_CB_EOA|COMEDI_CB_BLOCK|COMEDI_CB_ERROR;
	if(s->cur_trig.flags & TRIG_WAKE_EOS){
		s->cb_mask|=COMEDI_CB_EOS;
	}

	s->runflags=SRF_USER;

	s->subdev_flags|=SDF_RUNNING;

	ret=s->trig[s->cur_trig.mode](dev,s,&s->cur_trig);

	if(ret==0)return 0;

cleanup:
	do_become_nonbusy(dev,s);
	
	return ret;
}
#endif


/*
 * 	COMEDI_INSNLIST
 * 	synchronous instructions
 *
 * 	arg:
 * 		pointer to sync cmd structure
 *
 * 	reads:
 * 		sync cmd struct at arg
 * 		instruction list
 * 		data (for writes)
 *
 * 	writes:
 * 		data (for reads)
 */
static int do_insnlist_ioctl(comedi_device *dev,void *arg,void *file)
{
	comedi_insnlist insnlist;
	comedi_insn	insn;
	comedi_subdevice *s;
	lsampl_t	*data;
	int i;
	int ret=0;

	if(copy_from_user(&insnlist,arg,sizeof(comedi_insnlist)))
		return -EFAULT;
	
	if(insnlist.n_insns>=10)	/* XXX */
		return -EINVAL;

	data=kmalloc(sizeof(lsampl_t)*256,GFP_KERNEL);
	if(!data)
		return -ENOMEM;

	for(i=0;i<insnlist.n_insns;i++){
		if(copy_from_user(&insn,insnlist.insns+i,sizeof(comedi_insn))){
			ret=-EFAULT;
			goto error;
		}
		if(insn.n>256){
			ret=-EINVAL;
			goto error;
		}
		if(insn.insn&INSN_MASK_WRITE){
			if(copy_from_user(data,insn.data,insn.n*sizeof(lsampl_t))){
				ret=-EFAULT;
				goto error;
			}
		}
		if(insn.insn&INSN_MASK_SPECIAL){
			/* a non-subdevice instruction */

			switch(insn.insn){
			case INSN_GTOD:
			{
				struct timeval tv;

				do_gettimeofday(&tv);
				data[0]=tv.tv_sec;
				data[1]=tv.tv_usec;
				ret=2;
				
				break;
			}
			case INSN_WAIT:
				if(insn.n<1 || data[0]>=100){
					ret=-EINVAL;
					break;
				}
				udelay(data[0]);
				ret=1;
				break;
			default:
				ret=-EINVAL;
			}
		}else{
			/* a subdevice instruction */
			if(insn.subdev>=dev->n_subdevices){
				ret=-EINVAL;
				goto error;
			}
			s=dev->subdevices+insn.subdev;
	
			if(s->type==COMEDI_SUBD_UNUSED){
				DPRINTK("%d not useable subdevice\n",insn.subdev);
				ret = -EIO;
				goto error;
			}
		
			/* are we locked? (ioctl lock) */
			if(s->lock && s->lock!=file){
				DPRINTK("device locked\n");
				ret = -EACCES;
				goto error;
			}
	
			if((ret=check_chanlist(s,1,&insn.chanspec))<0){
				ret=-EINVAL;
				DPRINTK("bad chanspec\n");
				goto error;
			}

			if(s->busy){
				ret=-EBUSY;
				goto error;
			}
			s->busy=file;

			switch(insn.insn){
				case INSN_READ:
					ret=s->insn_read(dev,s,&insn,data);
					break;
				case INSN_WRITE:
					ret=s->insn_write(dev,s,&insn,data);
					break;
				case INSN_BITS:
					ret=s->insn_bits(dev,s,&insn,data);
					break;
				case INSN_CONFIG:
					ret=s->insn_config(dev,s,&insn,data);
					break;
				default:
					ret=-EINVAL;
					break;
			}

			s->busy=NULL;
		}
		if(ret<0)goto error;
		if(ret!=insn.n){
			printk("BUG: result of insn != insn.n\n");
			ret=-EINVAL;
			goto error;
		}
		if(insn.insn&INSN_MASK_READ){
			if(copy_to_user(insn.data,data,insn.n*sizeof(lsampl_t))){
				ret=-EFAULT;
				goto error;
			}
		}
	}

error:
	kfree(data);

	if(i==0)return ret;
	return i;
}

/*
	COMEDI_CMD
	command ioctl
	
	arg:
		pointer to cmd structure
	
	reads:
		cmd structure at arg
		channel/range list
	
	writes:
		modified cmd structure at arg

*/
static int do_cmd_ioctl(comedi_device *dev,void *arg,void *file)
{
	comedi_cmd user_cmd;
	comedi_subdevice *s;
	int ret=0;
	
	if(copy_from_user(&user_cmd,arg,sizeof(comedi_cmd))){
		DPRINTK("bad cmd address\n");
		return -EFAULT;
	}
	
	if(user_cmd.subdev>=dev->n_subdevices){
		DPRINTK("%d no such subdevice\n",user_cmd.subdev);
		return -ENODEV;
	}

	s=dev->subdevices+user_cmd.subdev;
	if(s->type==COMEDI_SUBD_UNUSED){
		DPRINTK("%d not valid subdevice\n",user_cmd.subdev);
		return -EIO;
	}
	
	if(!s->do_cmd){
		DPRINTK("subdevice does not support commands\n");
		return -EIO;
	}
	
	/* are we locked? (ioctl lock) */
	if(s->lock && s->lock!=file){
		DPRINTK("subdevice locked\n");
		return -EACCES;
	}

	/* are we busy? */
	if(s->busy){
		DPRINTK("subdevice busy\n");
		return -EBUSY;
	}
	s->busy=file;

	/* make sure channel/gain list isn't too long */
	if(user_cmd.chanlist_len > s->len_chanlist){
		DPRINTK("channel/gain list too long %d > %d\n",user_cmd.chanlist_len,s->len_chanlist);
		ret = -EINVAL;
		goto cleanup;
	}

	s->cmd=user_cmd;
	s->cmd.chanlist=NULL;
	s->cmd.data=NULL;

	/* load channel/gain list */
	/* we should have this already allocated */
	s->cmd.chanlist=kmalloc(s->cmd.chanlist_len*sizeof(int),GFP_KERNEL);
	if(!s->cmd.chanlist){
		DPRINTK("allocation failed\n");
		ret = -ENOMEM;
		goto cleanup;
	}
	
	if(copy_from_user(s->cmd.chanlist,user_cmd.chanlist,s->cmd.chanlist_len*sizeof(int))){
		DPRINTK("fault reading chanlist\n");
		ret = -EFAULT;
		goto cleanup;
	}
	
	/* make sure each element in channel/gain list is valid */
	if((ret=check_chanlist(s,s->cmd.chanlist_len,s->cmd.chanlist))<0){
		DPRINTK("bad chanlist\n");
		goto cleanup;
	}
	
	ret=s->do_cmdtest(dev,s,&s->cmd);

	if(s->cmd.flags&TRIG_BOGUS || ret){
		DPRINTK("test returned %d\n",ret);
		user_cmd=s->cmd;
		user_cmd.chanlist = NULL;
		user_cmd.data = NULL;
		if(copy_to_user(arg,&user_cmd,sizeof(comedi_cmd))){
			DPRINTK("fault writing cmd\n");
			ret = -EFAULT;
			goto cleanup;
		}
		ret = -EAGAIN;
		goto cleanup;
	}

	if(!s->prealloc_bufsz){
		ret=-ENOMEM;
		DPRINTK("no buffer (?)\n");
		goto cleanup;
	}
	s->cmd.data = s->prealloc_buf;
	s->cmd.data_len=s->prealloc_bufsz;

#ifdef CONFIG_COMEDI_MODE_CORE
	s->cur_trig.data=s->prealloc_buf;	/* XXX */
	s->cur_trig.data_len=s->prealloc_bufsz;
#endif

	s->buf_int_ptr=0;
	s->buf_int_count=0;
	if(s->subdev_flags & SDF_READABLE){
		s->buf_user_ptr=0;
		s->buf_user_count=0;
	}

	s->cur_chan = 0;
	s->cur_chanlist_len = s->cmd.chanlist_len;
	
	s->cb_mask = COMEDI_CB_EOA|COMEDI_CB_BLOCK|COMEDI_CB_ERROR;
	if(s->cmd.flags & TRIG_WAKE_EOS){
		s->cb_mask |= COMEDI_CB_EOS;
	}

	s->runflags=SRF_USER;

	s->subdev_flags|=SDF_RUNNING;

#ifdef CONFIG_COMEDI_RT
	if(s->cmd.flags&TRIG_RT){
		comedi_switch_to_rt(dev);
		s->runflags |= SRF_RT;
	}
#endif

	ret=s->do_cmd(dev,s);
	
	if(ret==0)return 0;

cleanup:
	do_become_nonbusy(dev,s);
	
	return ret;
}

/*
	COMEDI_CMDTEST
	command testing ioctl
	
	arg:
		pointer to cmd structure
	
	reads:
		cmd structure at arg
		channel/range list
	
	writes:
		modified cmd structure at arg

*/
static int do_cmdtest_ioctl(comedi_device *dev,void *arg,void *file)
{
	comedi_cmd user_cmd;
	comedi_subdevice *s;
	int ret=0;
	unsigned int *chanlist=NULL;
	
	if(copy_from_user(&user_cmd,arg,sizeof(comedi_cmd))){
		DPRINTK("bad cmd address\n");
		return -EFAULT;
	}
	
	if(user_cmd.subdev>=dev->n_subdevices){
		DPRINTK("%d no such subdevice\n",user_cmd.subdev);
		return -ENODEV;
	}

	s=dev->subdevices+user_cmd.subdev;
	if(s->type==COMEDI_SUBD_UNUSED){
		DPRINTK("%d not valid subdevice\n",user_cmd.subdev);
		return -EIO;
	}
	
	if(!s->do_cmd){
		DPRINTK("subdevice does not support commands\n");
		return -EIO;
	}
	
	/* make sure channel/gain list isn't too long */
	if(user_cmd.chanlist_len > s->len_chanlist){
		DPRINTK("channel/gain list too long %d > %d\n",user_cmd.chanlist_len,s->len_chanlist);
		ret = -EINVAL;
		goto cleanup;
	}

	/* load channel/gain list */
	if(user_cmd.chanlist){
		chanlist=kmalloc(user_cmd.chanlist_len*sizeof(int),GFP_KERNEL);
		if(!chanlist){
			DPRINTK("allocation failed\n");
			ret = -ENOMEM;
			goto cleanup;
		}
	
		if(copy_from_user(chanlist,user_cmd.chanlist,user_cmd.chanlist_len*sizeof(int))){
			DPRINTK("fault reading chanlist\n");
			ret = -EFAULT;
			goto cleanup;
		}
	
		/* make sure each element in channel/gain list is valid */
		if((ret=check_chanlist(s,user_cmd.chanlist_len,chanlist))<0){
			DPRINTK("bad chanlist\n");
			goto cleanup;
		}

		user_cmd.chanlist=chanlist;
	}

	ret=s->do_cmdtest(dev,s,&user_cmd);
	
	if(copy_to_user(arg,&user_cmd,sizeof(comedi_cmd))){
		DPRINTK("bad cmd address\n");
		ret=-EFAULT;
		goto cleanup;
	}
cleanup:
	if(chanlist)
		kfree(chanlist);
	
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

volatile int rtcomedi_lock_semaphore=0;

static int do_lock_ioctl(comedi_device *dev,unsigned int arg,void * file)
{
	int ret=0;
	comedi_subdevice *s;
	
	if(arg>=dev->n_subdevices)
		return -EINVAL;
	s=dev->subdevices+arg;
	
	if(s->busy)
		return -EBUSY;

	rtcomedi_lock_semaphore=1;
	
	if(s->lock && s->lock!=file){
		ret=-EACCES;
	}else{
		s->lock=file;
	}
	
	rtcomedi_lock_semaphore=0;

	if(ret<0)
		return ret;

#if 0
	if(s->lock_f)
		ret=s->lock_f(dev,s);
#endif

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

	This function isn't protected by the semaphore, since
	we already own the lock.
*/
static int do_unlock_ioctl(comedi_device *dev,unsigned int arg,void * file)
{
	comedi_subdevice *s;
	
	if(arg>=dev->n_subdevices)
		return -EINVAL;
	s=dev->subdevices+arg;
	
	if(s->busy)
		return -EBUSY;

	if(s->lock && s->lock!=file)
		return -EACCES;
	
	if(s->lock==file){
#if 0
		if(s->unlock)
			s->unlock(dev,s);
#endif

		s->lock=NULL;
	}

	return 0;
}

static int do_cancel(comedi_device *dev,comedi_subdevice *s);
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
static int do_cancel_ioctl(comedi_device *dev,unsigned int arg,void *file)
{
	comedi_subdevice *s;
	
	if(arg>=dev->n_subdevices)
		return -EINVAL;
	s=dev->subdevices+arg;
	
	if(s->lock && s->lock!=file)
		return -EACCES;
	
	if(!s->busy)
		return 0;

	if(s->busy!=file)
		return -EBUSY;

	return do_cancel(dev,s);
}

static int do_cancel(comedi_device *dev,comedi_subdevice *s)
{
	int ret=0;

	if((s->subdev_flags&SDF_RUNNING) && s->cancel)
		ret=s->cancel(dev,s);

	do_become_nonbusy(dev,s);

	return ret;
}

#ifdef LINUX_V22
/*
   comedi_mmap_v22

   issues:
      what happens when the underlying buffer gets changed?
      
 */
static int comedi_mmap_v22(struct file * file, struct vm_area_struct *vma)
{
	kdev_t minor=MINOR(RDEV_OF_FILE(file));
	comedi_device *dev=comedi_get_device_by_minor(minor);
	comedi_subdevice *s;
	int subdev;

	if(vma->vm_flags & VM_WRITE){
		subdev=dev->write_subdev;
	}else{
		subdev=dev->read_subdev;
	}
	if(subdev<0){
		return -EINVAL;
	}
	s=dev->subdevices+subdev;

	if(VM_OFFSET(vma) != 0){
		DPRINTK("comedi: mmap() offset must be 0.\n");
		return -EINVAL;
	}

	rvmmap(s->prealloc_buf,s->prealloc_bufsz,vma);
	
	//vma->vm_file = file;
	//vma->vm_ops = &comedi_vm_ops;
	//file_atomic_inc(&file->f_count);

	/* XXX mark subdev as mapped */
	
	return 0;
}
#endif

#if LINUX_VERSION_CODE >= 0x020100

static unsigned int comedi_poll_v22(struct file *file, poll_table * wait)
{
	comedi_device *dev;
	comedi_subdevice *s;
	unsigned int mask;

	dev=comedi_get_device_by_minor(MINOR(RDEV_OF_FILE(file)));

	poll_wait(file, &dev->read_wait, wait);
	poll_wait(file, &dev->write_wait, wait);
	mask = 0;
	if(dev->read_subdev>=0){
		s=dev->subdevices+dev->read_subdev;
		if(!(s->subdev_flags&SDF_RUNNING) ||
		   (s->buf_user_count < s->buf_int_count))
			mask |= POLLIN | POLLRDNORM;
	}
	if(dev->write_subdev>=0){
		s=dev->subdevices+dev->write_subdev;
		if((!s->subdev_flags&SDF_RUNNING) ||
		   (s->buf_user_count < s->buf_int_count + s->prealloc_bufsz))
			mask |= POLLOUT | POLLWRNORM;
	}

	return mask;
}
#endif

static ssize_t comedi_write_v22(struct file *file,const char *buf,size_t nbytes,loff_t *offset)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int n,m,count=0,retval=0;
	DECLARE_WAITQUEUE(wait,current);
	int sample_size;
	void *buf_ptr;
	unsigned int buf_len;

	dev=comedi_get_device_by_minor(MINOR(RDEV_OF_FILE(file)));
	if(dev->write_subdev<0)return -EIO;
	s=dev->subdevices+dev->write_subdev;

	if(s->subdev_flags&SDF_LSAMPL){
		sample_size=sizeof(lsampl_t);
	}else{
		sample_size=sizeof(sampl_t);
	}
	if(nbytes%sample_size)
		nbytes-=nbytes%sample_size;

	if(!nbytes)return 0;

	if(!(s->subdev_flags&SDF_WRITEABLE))
		return -EIO;

	if(!s->busy){
		buf_ptr=s->prealloc_buf;
		buf_len=s->prealloc_bufsz;
	}else{
		if(s->busy != file)
			return -EACCES;

#ifdef CONFIG_COMEDI_MODE_CORE
		buf_ptr=s->cur_trig.data; /* XXX */
		buf_len=s->cur_trig.data_len;
#endif
	}

	if(!buf_ptr)
		return -EIO;

	add_wait_queue(&dev->write_wait,&wait);
	while(nbytes>0 && !retval){
		current->state=TASK_INTERRUPTIBLE;

		n=nbytes;

		m=buf_len-(s->buf_user_count-s->buf_int_count);
		if(s->buf_user_ptr+m > buf_len){
			m=buf_len - s->buf_user_ptr;
		}
		if(m<n)n=m;

		if(n==0){
			if(file->f_flags&O_NONBLOCK){
				retval=-EAGAIN;
				break;
			}
			if(signal_pending(current)){
				retval=-ERESTARTSYS;
				break;
			}
			if(!(s->subdev_flags&SDF_RUNNING)){
				do_become_nonbusy(dev,s);
				break;
			}
			schedule();
			continue;
		}
		m=copy_from_user(buf_ptr+s->buf_user_ptr,buf,n);
		if(m) retval=-EFAULT;
		n-=m;
		
		count+=n;
		nbytes-=n;
		s->buf_user_ptr+=n;
		s->buf_user_count+=n;

		if(s->buf_user_ptr>=buf_len ){
			s->buf_user_ptr=0;
		}

		buf+=n;
		break;	/* makes device work like a pipe */
	}
	current->state=TASK_RUNNING;
	remove_wait_queue(&dev->write_wait,&wait);

	return (count ? count : retval);
}


static ssize_t comedi_read_v22(struct file * file,char *buf,size_t nbytes,loff_t *offset)
{
	comedi_device *dev;
	comedi_subdevice *s;
	int n,m,count=0,retval=0;
	DECLARE_WAITQUEUE(wait,current);
	int sample_size;

	dev=comedi_get_device_by_minor(MINOR(RDEV_OF_FILE(file)));
	if(dev->read_subdev<0)return -EIO;
	s=dev->subdevices+dev->read_subdev;

	if(s->subdev_flags&SDF_LSAMPL){
		sample_size=sizeof(lsampl_t);
	}else{
		sample_size=sizeof(sampl_t);
	}
	if(nbytes%sample_size)
		nbytes-=nbytes%sample_size;

	if(!nbytes)return 0;

	if(!s->busy)
		return 0;

#ifdef CONFIG_COMEDI_MODE_CORE
	if(!s->cur_trig.data || !(s->subdev_flags&SDF_READABLE))	/* XXX */
		return -EIO;
#endif

	if(s->busy != file)
		return -EACCES;

	add_wait_queue(&dev->read_wait,&wait);
	while(nbytes>0 && !retval){
		current->state=TASK_INTERRUPTIBLE;

		n=nbytes;

		m=s->buf_int_count-s->buf_user_count;

		if(s->buf_user_ptr+m > s->cur_trig.data_len){ /* XXX MODE */
			m=s->cur_trig.data_len - s->buf_user_ptr;
#if 0
printk("m is %d\n",m);
#endif
		}
		if(m<n)n=m;

		if(n==0){
			if(!(s->subdev_flags&SDF_RUNNING)){
				do_become_nonbusy(dev,s);
				retval=-EINVAL;
				break;
			}
			if(file->f_flags&O_NONBLOCK){
				retval=-EAGAIN;
				break;
			}
			if(signal_pending(current)){
				retval=-ERESTARTSYS;
				break;
			}
			schedule();
			continue;
		}
		m=copy_to_user(buf,((void *)(s->cur_trig.data))+s->buf_user_ptr,n);
		if(m) retval=-EFAULT;
		n-=m;

		// check for buffer overrun
		if(s->buf_int_count - s->buf_user_count > s->cur_trig.data_len){	/* XXX MODE */
			s->buf_user_count = s->buf_int_count;
			s->buf_user_ptr = s->buf_int_ptr;
			retval=-EINVAL;
			do_cancel_ioctl(dev, dev->read_subdev, file);
			DPRINTK("buffer overrun\n");
			break;
		}

		count+=n;
		nbytes-=n;
		s->buf_user_ptr+=n;
		s->buf_user_count+=n;

		if(s->buf_user_ptr>=s->cur_trig.data_len ){
			s->buf_user_ptr=0;
		}

		buf+=n;
		break;	/* makes device work like a pipe */
	}
	if(!(s->subdev_flags&SDF_RUNNING) && s->buf_int_count-s->buf_user_count==0){
		do_become_nonbusy(dev,s);
	}
	current->state=TASK_RUNNING;
	remove_wait_queue(&dev->read_wait,&wait);

	return (count ? count : retval);
}

/*
   This function restores a subdevice to an idle state.
 */
static void do_become_nonbusy(comedi_device *dev,comedi_subdevice *s)
{
#if 0
	printk("becoming non-busy\n");
#endif
	/* we do this because it's useful for the non-standard cases */
	s->subdev_flags &= ~SDF_RUNNING;

#ifdef CONFIG_COMEDI_RT
	if(s->runflags&SRF_RT){
		comedi_switch_to_non_rt(dev);
		s->runflags &= ~SRF_RT;
	}
#endif

	if(s->cur_trig.chanlist){		/* XXX wrong? */
		kfree(s->cur_trig.chanlist);
		s->cur_trig.chanlist=NULL;
	}

	if(s->cur_trig.data){
		if(s->cur_trig.data != s->prealloc_buf)
			kfree(s->cur_trig.data);

		s->cur_trig.data=NULL;
	}

	s->buf_user_ptr=0;
	s->buf_int_ptr=0;
	s->buf_user_count=0;
	s->buf_int_count=0;

	s->busy=NULL;
}

/* no chance that these will change soon */
#define SEEK_SET 0
#define SEEK_CUR 1
#define SEEK_END 2

static loff_t comedi_lseek_v22(struct file *file,loff_t offset,int origin)
{
	comedi_device *dev;
	loff_t new_offset;
	
	dev=comedi_get_device_by_minor(MINOR(RDEV_OF_FILE(file)));

	switch(origin){
	case SEEK_SET:
		new_offset = offset;
		break;
	case SEEK_CUR:
		new_offset = file->f_pos + offset;
		break;
	case SEEK_END:
		new_offset = dev->n_subdevices + offset;
		break;
	default:
		return -EINVAL;
	}
	if(new_offset<0 || new_offset >= dev->n_subdevices)
		return -EINVAL;

	return file->f_pos=new_offset;
}

static int comedi_fop_open(struct inode *inode,struct file *file)
{
	kdev_t minor=MINOR(inode->i_rdev);
	comedi_device *dev;
	static int in_comedi_open=0;
	char mod[32];

	if(minor>=COMEDI_NDEVICES)return -ENODEV;

	dev=comedi_get_device_by_minor(minor);
	if(dev->attached)
		goto ok;
	if(in_comedi_open && suser())
		goto ok;

	in_comedi_open=1;

	sprintf(mod,"char-major-%i-%i",COMEDI_MAJOR,minor);
#ifdef CONFIG_KMOD
	request_module(mod);
#endif

	in_comedi_open=0;

	if(dev->attached || suser())
		goto ok;
	return -ENODEV;

ok:
	MOD_INC_USE_COUNT;
	if(dev->attached && dev->driver->module){
		__MOD_INC_USE_COUNT(dev->driver->module);
	}
	dev->use_count++;

	return 0;
}

static int comedi_close_v22(struct inode *inode,struct file *file)
{
	comedi_device *dev=comedi_get_device_by_minor(MINOR(inode->i_rdev));
	comedi_subdevice *s;
	int i;

	for(i=0;i<dev->n_subdevices;i++){
		s=dev->subdevices+i;

		if(s->busy==file){
			do_cancel(dev,s);
		}
		if(s->lock==file){
			s->lock=NULL;
		}
	}

	MOD_DEC_USE_COUNT;
	if(dev->attached && dev->driver->module){
		__MOD_DEC_USE_COUNT(dev->driver->module);
	}

	dev->use_count--;
	
	return 0;
}


/*
	kernel compatibility
*/

#ifdef LINUX_V20

static int comedi_write_v20(struct inode *inode,struct file *file,const char *buf,int nbytes)
{
	return comedi_write_v22(file,buf,nbytes,NULL);
}

static int comedi_read_v20(struct inode *inode,struct file *file,char *buf,int nbytes)
{
	return comedi_read_v22(file,buf,nbytes,NULL);
}

static int comedi_lseek_v20(struct inode * inode,struct file *file,off_t offset,int origin)
{
	return comedi_lseek_v22(file,offset,origin);
}

static void comedi_close_v20(struct inode *inode,struct file *file)
{
	comedi_close_v22(inode,file);
}

#define comedi_ioctl_v20 comedi_ioctl
#define comedi_open_v20 comedi_fop_open

static struct file_operations comedi_fops={
	lseek		: comedi_lseek_v20,
	ioctl		: comedi_ioctl_v20,
	open		: comedi_open_v20,
	release		: comedi_close_v20,
	read		: comedi_read_v20,
	write		: comedi_write_v20,
};

#endif

#ifdef LINUX_V22

#define comedi_ioctl_v22 comedi_ioctl
#define comedi_open_v22 comedi_fop_open

static struct file_operations comedi_fops={
#if LINUX_VERSION_CODE >= 0x020400
	owner		: THIS_MODULE,
#endif
	llseek		: comedi_lseek_v22,
	ioctl		: comedi_ioctl_v22,
	open		: comedi_open_v22,
	release		: comedi_close_v22,
	read		: comedi_read_v22,
	write		: comedi_write_v22,
	mmap		: comedi_mmap_v22,
	poll		: comedi_poll_v22,
};
#endif

void mite_init(void);
void mite_cleanup(void);
void init_drivers(void);


int comedi_init(void)
{
	int i;

	printk("comedi: version " COMEDI_RELEASE " - David Schleef <ds@schleef.org>\n");
	if(register_chrdev(COMEDI_MAJOR,"comedi",&comedi_fops)){
		printk("comedi: unable to get major %d\n",COMEDI_MAJOR);
		return -EIO;
	}
	comedi_devices=(comedi_device *)kmalloc(sizeof(comedi_device)*COMEDI_NDEVICES,GFP_KERNEL);
	if(!comedi_devices)
		return -ENOMEM;
	memset(comedi_devices,0,sizeof(comedi_device)*COMEDI_NDEVICES);
	for(i=0;i<COMEDI_NDEVICES;i++){
		comedi_devices[i].minor=i;
	}
#if 0
	init_polling();
#endif

	/* XXX requires /proc interface */
	comedi_proc_init();
	
#ifdef CONFIG_COMEDI_RT
//	comedi_rt_init();
#endif
	init_drivers();

	return 0;
}

void comedi_cleanup(void)
{
	int i;

	if(MOD_IN_USE)
		printk("comedi: module in use -- remove delayed\n");

	unregister_chrdev(COMEDI_MAJOR,"comedi");

	comedi_proc_cleanup();
#if 0
	comedi_polling_cleanup();
#endif
	for(i=0;i<COMEDI_NDEVICES;i++){
		comedi_device *dev;

		dev=comedi_get_device_by_minor(i);
		if(dev->attached)
			comedi_device_detach(dev);
	}
	kfree(comedi_devices);

#ifdef CONFIG_COMEDI_RT
//	comedi_rt_cleanup();
#endif

}

#ifdef MODULE
int init_module(void)
{
	return comedi_init();
}

void cleanup_module(void)
{
	comedi_cleanup();
}
#endif

void comedi_error(comedi_device *dev,const char *s)
{
	rt_printk("comedi%d: %s: %s\n",dev->minor,dev->driver->driver_name,s);
}

void comedi_event(comedi_device *dev,comedi_subdevice *s,unsigned int mask)
{
	if(s->cb_mask&mask){
		if(s->runflags&SRF_USER){
			unsigned int subdev;

			subdev = s - dev->subdevices;
			if(dev->rt){
#ifdef CONFIG_COMEDI_RT
				// pend wake up
				if(subdev==dev->read_subdev)
					comedi_rt_pend_wakeup(&dev->read_wait);
				if(subdev==dev->write_subdev)
					comedi_rt_pend_wakeup(&dev->write_wait);
#endif
			}else{
				if(subdev==dev->read_subdev)
					wake_up_interruptible(&dev->read_wait);
				if(subdev==dev->write_subdev)
					wake_up_interruptible(&dev->write_wait);
			}
		}else{
			if(s->runflags&SRF_RT){
				s->cb_func(mask,s->cb_arg);
			}else{
			/* XXX bug here.  If subdevice A is rt, and
			 * subdevice B tries to callback to a normal
			 * linux kernel function, it will be at the
			 * wrong priority.  Since this isn't very
			 * common, I'm not going to worry about it. */
			}
		}
	}
	
	if(mask&COMEDI_CB_EOA){
		s->subdev_flags &= ~SDF_RUNNING;
	}
}

/*
   this function should be called by your interrupt routine
   at the end of acquisition
 */
void comedi_done(comedi_device *dev,comedi_subdevice *s)
{
	comedi_event(dev,s,COMEDI_CB_EOA);
}

/*
   this function should be called by your interrupt routine
   at errors causing termination of acquisition
 */
void comedi_error_done(comedi_device *dev,comedi_subdevice *s)
{
	comedi_event(dev,s,COMEDI_CB_ERROR|COMEDI_CB_EOA);
}

/*
   this function should be called by your interrupt routine
   at convenient block sizes
 */
void comedi_bufcheck(comedi_device *dev,comedi_subdevice *s)
{
	comedi_event(dev,s,COMEDI_CB_BLOCK);
}

/*
   this function should be called by your interrupt routine
   at end-of-scan events
 */
void comedi_eos(comedi_device *dev,comedi_subdevice *s)
{
	comedi_event(dev,s,COMEDI_CB_EOS);
}

/*
   this function should be called by your interrupt routine
   at buffer rollover events
 */
void comedi_eobuf(comedi_device *dev,comedi_subdevice *s)
{
	comedi_event(dev,s,COMEDI_CB_EOBUF);
}

