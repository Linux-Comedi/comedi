/*
    kcomedilib/kcomedilib.c
    a comedlib interface for kernel modules

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2000 David A. Schleef <ds@schleef.org>

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
#include <linux/slab.h>
#include <asm/io.h>

#include <linux/comedidev.h>
#include <linux/comedi.h>
#include <linux/comedilib.h>


MODULE_AUTHOR("David Schleef <ds@schleef.org>");
MODULE_DESCRIPTION("Comedi kernel library");
MODULE_LICENSE("GPL");



comedi_t *comedi_open(const char *filename)
{
	comedi_device *dev;
	unsigned int minor;

	if(strncmp(filename,"/dev/comedi",11) != 0)
		return NULL;

	minor = simple_strtoul(filename+11,NULL,0);

	if(minor >= COMEDI_NDEVICES)
		return NULL;

	dev = comedi_get_device_by_minor(minor);

	if(!dev->attached)
		return NULL;

	if(!try_module_get(dev->driver->module))
		return NULL;

	return (comedi_t *)dev;
}

comedi_t *comedi_open_old(unsigned int minor)
{
	comedi_device *dev;

	if(minor>=COMEDI_NDEVICES)
		return NULL;

	dev = comedi_get_device_by_minor(minor);

	if(!dev->attached)
		return NULL;

	return (comedi_t *)dev;
}

int comedi_close(comedi_t *d)
{
	comedi_device *dev = (comedi_device *) d;

	module_put(dev->driver->module);

	return 0;
}

int comedi_loglevel(int newlevel)
{
	return 0;
}

void comedi_perror(const char *message)
{
	rt_printk("%s: unknown error\n",message);
}

char *comedi_strerror(int err)
{
	return "unknown error";
}

int comedi_fileno(comedi_t *d)
{
	comedi_device *dev = (comedi_device *)d;

	/* return something random */
	return dev->minor;
}

static void init_async_buf( comedi_async *async )
{
	async->buf_read_count = 0;
	async->buf_write_count = 0;
	async->buf_write_alloc_count = 0;
	async->buf_read_ptr = 0;
	async->buf_write_ptr = 0;
	async->cur_chan = 0;
	async->scan_progress = 0;
	async->munge_chan = 0;
	async->munge_count = 0;
	async->munge_ptr = 0;
	async->events = 0;
}

int comedi_command(comedi_t *d,comedi_cmd *cmd)
{
	comedi_device *dev = (comedi_device *)d;
	comedi_subdevice *s;
	comedi_async *async;

	if(cmd->subdev>=dev->n_subdevices)
		return -ENODEV;

	s=dev->subdevices+cmd->subdev;
	if(s->type==COMEDI_SUBD_UNUSED)
		return -EIO;

	async = s->async;
	if(async == NULL)
		return -ENODEV;

	if(s->busy)
		return -EBUSY;
	s->busy = d;

	if(async->cb_mask & COMEDI_CB_EOS)
		cmd->flags |= TRIG_WAKE_EOS;

	async->cmd=*cmd;

	s->runflags |= SRF_RT;
	comedi_switch_to_rt(dev);

	s->subdev_flags |= SDF_RUNNING;

	init_async_buf( async );

	return s->do_cmd(dev,s);
}

int comedi_command_test(comedi_t *d,comedi_cmd *cmd)
{
	comedi_device *dev = (comedi_device *)d;
	comedi_subdevice *s;

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
 *	COMEDI_INSN
 *	perform an instruction
 */
int comedi_do_insn(comedi_t *d,comedi_insn *insn)
{
	comedi_device *dev = (comedi_device *)d;
	comedi_subdevice *s;
	int ret=0;

	if(insn->insn&INSN_MASK_SPECIAL){
		switch(insn->insn){
		case INSN_GTOD:
		{
			struct timeval tv;

			do_gettimeofday(&tv);
			insn->data[0] = tv.tv_sec;
			insn->data[1] = tv.tv_usec;
			ret = 2;

			break;
		}
		case INSN_WAIT:
			if(insn->n<1 || insn->data[0]>=100){
				ret = -EINVAL;
				break;
			}
			comedi_udelay(insn->data[0]);
			ret=1;
			break;
		case INSN_INTTRIG:
			if(insn->n!=1){
				ret=-EINVAL;
				break;
			}
			if(insn->subdev>=dev->n_subdevices){
				rt_printk("%d not usable subdevice\n",insn->subdev);
				ret=-EINVAL;
				break;
			}
			s=dev->subdevices+insn->subdev;
			if(!s->async){
				rt_printk("no async\n");
				ret=-EINVAL;
				break;
			}
			if(!s->async->inttrig){
				rt_printk("no inttrig\n");
				ret=-EAGAIN;
				break;
			}
			ret = s->async->inttrig(dev,s,insn->data[0]);
			if(ret>=0)ret = 1;
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
			/* XXX no return value is set! ret = ? */
			goto error;
		}

		/* XXX check lock */

		if((ret=check_chanlist(s,1,&insn->chanspec))<0){
			rt_printk("bad chanspec\n");
			/* XXX no return value is set! ret = ? */
			goto error;
		}

		if(s->busy){
			ret = -EBUSY;
			goto error;
		}
		s->busy = d;

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
			case INSN_CONFIG:
				ret = s->insn_config(dev,s,insn,insn->data);
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

	necessary locking:
	- ioctl/rt lock  (this type)
	- lock while subdevice busy
	- lock while subdevice being programmed

*/
int comedi_lock(comedi_t *d,unsigned int subdevice)
{
	comedi_device *dev = (comedi_device *)d;
	comedi_subdevice *s = dev->subdevices + subdevice;
	unsigned long flags;
	int ret=0;

	comedi_spin_lock_irqsave(&big_comedi_lock,flags);

	if(s->busy){
		ret = -EBUSY;
	}else{
		if(s->lock && s->lock!=d){
			ret = -EACCES;
		}else{
			s->lock = d;
		}
	}

	comedi_spin_unlock_irqrestore(&big_comedi_lock,flags);

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
int comedi_unlock(comedi_t *d,unsigned int subdevice)
{
	comedi_device *dev = (comedi_device *)d;
	comedi_subdevice *s = dev->subdevices + subdevice;
	unsigned long flags;
	comedi_async *async;

	async = s->async;

	comedi_spin_lock_irqsave(&big_comedi_lock,flags);

	if(s->busy){
		comedi_spin_unlock_irqrestore(&big_comedi_lock,flags);
		return -EBUSY;
	}

	if(s->lock && s->lock!=(void *)d){
		comedi_spin_unlock_irqrestore(&big_comedi_lock,flags);
		return -EACCES;
	}

	s->lock=NULL;

	if(async){
		async->cb_mask=0;
		async->cb_func=NULL;
		async->cb_arg=NULL;
	}

	comedi_spin_unlock_irqrestore(&big_comedi_lock,flags);

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
int comedi_cancel(comedi_t *d,unsigned int subdevice)
{
	comedi_device *dev = (comedi_device *)d;
	comedi_subdevice *s = dev->subdevices + subdevice;
	int ret=0;

	if(s->lock && s->lock!=d)
		return -EACCES;

#if 0
	if(!s->busy)
		return 0;

	if(s->busy!=d)
		return -EBUSY;
#endif

	if(!s->cancel)
		return -EINVAL;

	if((ret=s->cancel(dev,s)))
		return ret;

	if( s->runflags & SRF_RT )
	{
		// XXX race
		s->runflags &= ~SRF_RT;
		comedi_switch_to_non_rt(dev);
	}

	s->busy=NULL;

	return 0;
}

/*
   registration of callback functions
 */
int comedi_register_callback(comedi_t *d,unsigned int subdevice,
		unsigned int mask,int (*cb)(unsigned int,void *),void *arg)
{
	comedi_device *dev = (comedi_device *)d;
	comedi_subdevice *s = dev->subdevices + subdevice;
	comedi_async *async;

	async = s->async;
	if(s->type==COMEDI_SUBD_UNUSED)
		return -EIO;

	/* are we locked? (ioctl lock) */
	if(s->lock && s->lock!=d)
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


int comedi_poll(comedi_t *d, unsigned int subdevice)
{
	comedi_device *dev = (comedi_device *)d;
	comedi_subdevice *s = dev->subdevices + subdevice;
	comedi_async *async;

	async = s->async;
	if(s->type==COMEDI_SUBD_UNUSED || !async)
		return -EIO;

	/* are we locked? (ioctl lock) */
	if(s->lock && s->lock!=d)
		return -EACCES;

	/* are we running? XXX wrong? */
	if(!s->busy)
		return -EIO;

	return s->poll(dev,s);
}


/* WARNING: not portable */
int comedi_map(comedi_t *d, unsigned int subdevice, void *ptr)
{
	comedi_device *dev = (comedi_device *)d;
	comedi_subdevice *s = dev->subdevices + subdevice;

	if(!s->async)
		return -EINVAL;

	if(ptr)
	{
		*((void **) ptr) = s->async->prealloc_buf;
	}

	/* XXX no reference counting */

	return 0;
}

/* WARNING: not portable */
int comedi_unmap(comedi_t *d, unsigned int subdevice)
{
	comedi_device *dev = (comedi_device *)d;
	comedi_subdevice *s = dev->subdevices + subdevice;

	if(!s->async)
		return -EINVAL;

	/* XXX no reference counting */

	return 0;
}

