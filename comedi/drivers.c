/*
    module/drivers.c
    functions for manipulating drivers

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

#ifdef CONFIG_COMEDI_TRIG
/* in comedi/trig.c */
int command_trig(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
int mode_to_command(comedi_cmd *cmd,comedi_trig *it);
int mode0_emulate(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig);
int mode0_emulate_config(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig);
#endif

static int postconfig(comedi_device *dev);
static int insn_rw_emulate_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int insn_inval(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static void *comedi_recognize(comedi_driver *driv, const char *name);
static void comedi_report_boards(comedi_driver *driv);
static int poll_invalid(comedi_device *dev,comedi_subdevice *s);

comedi_driver *comedi_drivers;

int comedi_modprobe(kdev_t minor)
{
	return -EINVAL;
}

int comedi_device_detach(comedi_device *dev)
{
	int i;
	comedi_subdevice *s;

	if(!dev->attached)
		return 0;

	/* this is not correct for the kmod case */
	if(dev->driver->module)
		__MOD_DEC_USE_COUNT(dev->driver->module);

	dev->attached=0;

	for(i=0;i<dev->n_subdevices;i++){
		s=dev->subdevices+i;
		if(s->async){
			rvfree(s->async->prealloc_buf,s->async->prealloc_bufsz);
			kfree(s->async);
		}
	}

	if(dev->driver){
		dev->driver->detach(dev);
	}else{
		printk("BUG: dev->driver=NULL in comedi_device_detach()\n");
	}

	if(dev->subdevices)kfree(dev->subdevices);
	if(dev->private)kfree(dev->private);

	return 0;
}

int comedi_device_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_driver *driv;
	int ret;
	int minor;
	int use_count;

	if(dev->attached)
		return -EBUSY;

	minor = dev->minor;
	use_count = dev->use_count;
	memset(dev,0,sizeof(comedi_device));
	spin_lock_init(&dev->spinlock);
	dev->minor=minor;
	dev->use_count = use_count;

	for(driv=comedi_drivers;driv;driv=driv->next){
		if(driv->num_names){
			dev->board_ptr=comedi_recognize(driv, it->board_name);
			if(dev->board_ptr==NULL) continue;
		}else{
			if(strcmp(driv->driver_name,it->board_name))
				continue;
		}
		//initialize dev->driver here so comedi_error() can be called from attach
		dev->driver=driv;
		ret=driv->attach(dev,it);
		if(ret<0){
			driv->detach(dev);
			if(dev->subdevices)kfree(dev->subdevices);
			if(dev->private)kfree(dev->private);

			return ret;
		}
		goto attached;
	}

	// recognize has failed if we get here
	// report valid board names before returning error
	for(driv=comedi_drivers;driv;driv=driv->next){
		comedi_report_boards(driv);
	}
	return -EIO;

attached:
	/* do a little post-config cleanup */
	ret = postconfig(dev);
	if(ret < 0)
	{
		driv->detach(dev);
		if(dev->subdevices)kfree(dev->subdevices);
		if(dev->private)kfree(dev->private);

		return ret;
	}

	init_waitqueue_head(&dev->read_wait);
	init_waitqueue_head(&dev->write_wait);

	if(!dev->board_name){
		printk("BUG: dev->board_name=<%p>\n",dev->board_name);
		dev->board_name="BUG";
	}

	dev->attached=1;

	if(driv->module)
		__MOD_INC_USE_COUNT(driv->module);

	return 0;
}

int comedi_driver_register(comedi_driver *driver)
{
	driver->next=comedi_drivers;
	comedi_drivers=driver;

	return 0;
}

int comedi_driver_unregister(comedi_driver *driver)
{
	comedi_driver *prev;
	int i;

	/* check for devices using this driver */
	for(i=0;i<COMEDI_NDEVICES;i++){
		comedi_device *dev;

		dev=comedi_get_device_by_minor(i);
		if(dev->attached && dev->driver==driver){
			if(dev->use_count)
				printk("BUG! detaching device with use_count=%d\n",dev->use_count);
			comedi_device_detach(dev);
		}
	}

	if(comedi_drivers==driver){
		comedi_drivers=driver->next;
		return 0;
	}

	for(prev=comedi_drivers;prev->next;prev=prev->next){
		if(prev->next==driver){
			prev->next=driver->next;
			return 0;
		}
	}
	return -EINVAL;
}

comedi_device *comedi_allocate_dev(comedi_driver *driver)
{
	comedi_device *dev;

	// XXX we need to do actual allocation here.
	
	dev=comedi_get_device_by_minor(0);

	dev->driver=driver;

	return dev;
}

void comedi_deallocate_dev(comedi_device *dev)
{

}

static int postconfig(comedi_device *dev)
{
	int i;
	comedi_subdevice *s;
	comedi_async *async = NULL;

	for(i=0;i<dev->n_subdevices;i++){
		s=dev->subdevices+i;

		if(s->type==COMEDI_SUBD_UNUSED)
			continue;

		if(s->len_chanlist==0)
			s->len_chanlist=1; 
#ifdef CONFIG_COMEDI_TRIG
		if(s->do_cmd){
			s->trig[1]=command_trig;
			s->trig[2]=command_trig;
			s->trig[3]=command_trig;
			s->trig[4]=command_trig;
		}
#endif

		if(s->do_cmd
#ifdef CONFIG_COMEDI_TRIG
		    || s->trig[1] || s->trig[2] || s->trig[3] ||s->trig[4]
#endif
		    ){
			async = kmalloc(sizeof(comedi_async), GFP_KERNEL);
			if(async == NULL)
			{
				printk("failed to allocate async struct\n");
				return -ENOMEM;
			}
			memset(async, 0, sizeof(comedi_async));
			s->async = async;
			async->max_bufsize=64*1024;
			async->prealloc_bufsz=16*1024;
			async->prealloc_buf=rvmalloc(async->prealloc_bufsz);
			if(!async->prealloc_buf){
				printk("ENOMEM\n");
				kfree(async);
				return -ENOMEM;
			}
		}

		if(!s->range_table && !s->range_table_list)
			s->range_table=&range_unknown;

		if(!s->insn_read && s->insn_bits)
			s->insn_read = insn_rw_emulate_bits;
		if(!s->insn_write && s->insn_bits)
			s->insn_write = insn_rw_emulate_bits;

		if(!s->insn_read)s->insn_read = insn_inval;
		if(!s->insn_write)s->insn_write = insn_inval;
		if(!s->insn_bits)s->insn_bits = insn_inval;

		if(!s->poll)s->poll=poll_invalid;
	}

	return 0;
}

// generic recognize function for drivers that register their supported board names
void *comedi_recognize(comedi_driver *driv, const char *name)
{
	unsigned int i = 0;
	void *name_ptr;

	name_ptr=driv->board_name;
	for(i = 0; i < driv->num_names; i++)
	{
		if(strcmp(*(char **)name_ptr, name) == 0)
			return name_ptr;
		name_ptr += driv->offset;
	}

	return NULL;
}

void comedi_report_boards(comedi_driver *driv)
{
	unsigned int i;
	void *name_ptr;

	printk("comedi: valid board names for %s driver are:\n", driv->driver_name);

	name_ptr=driv->board_name;
	for(i = 0; i < driv->num_names; i++)
	{
		printk(" %s\n", *(char **)name_ptr);
		name_ptr += driv->offset;
	}

	if(driv->num_names == 0)
		printk(" %s\n", driv->driver_name);
}

static int poll_invalid(comedi_device *dev,comedi_subdevice *s)
{
	return -EINVAL;
}

static int insn_inval(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	return -EINVAL;
}

static int insn_rw_emulate_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	comedi_insn new_insn;
	int ret;
	lsampl_t new_data[2];
	unsigned int chan;

	chan = CR_CHAN(insn->chanspec);

	memset(&new_insn,0,sizeof(new_insn));
	new_insn.insn = INSN_BITS;
	new_insn.n = 2;
	new_insn.data = new_data;
	new_insn.subdev = insn->subdev;

	if(insn->insn == INSN_WRITE){
		if(!(s->subdev_flags & SDF_WRITEABLE))
			return -EINVAL;
		new_data[0] = 1<<chan; /* mask */
		new_data[1] = data[0]?(1<<chan):0; /* bits */
	}else {
		new_data[0] = 0;
		new_data[1] = 0;
	}

	ret = s->insn_bits(dev,s,&new_insn,new_data);
	if(ret<0)return ret;

	if(insn->insn == INSN_READ){
		if(!(s->subdev_flags & SDF_READABLE))
			return -EINVAL;
		data[0] = (new_data[1]>>chan)&1;
	}

	return 1;
}

#define REG(x) {extern comedi_driver (x);comedi_driver_register(&(x));}

void init_drivers(void)
{
#ifdef CONFIG_COMEDI_DT282x
	REG(driver_dt282x);
#endif
#ifdef CONFIG_COMEDI_NI_PCIMIO
	REG(driver_pcimio);
#endif
#ifdef CONFIG_COMEDI_NI_ATMIO
	REG(driver_atmio);
#endif
#ifdef CONFIG_COMEDI_DT2801
	REG(driver_dt2801);
#endif
#ifdef CONFIG_COMEDI_DT2811
	REG(driver_dt2811);
#endif
#ifdef CONFIG_COMEDI_DT2814
	REG(driver_dt2814);
#endif
#ifdef CONFIG_COMEDI_DT2817
	REG(driver_dt2817);
#endif
#ifdef CONFIG_COMEDI_DT3000
	REG(driver_dt3000);
#endif
#ifdef CONFIG_COMEDI_8255
	REG(driver_8255);
#endif
#ifdef CONFIG_NI_PCIDIO
	REG(driver_nidio);
#endif
#ifdef CONFIG_COMEDI_DAS08
	REG(driver_das08);
#endif
#ifdef CONFIG_COMEDI_PCL711
	REG(driver_pcl711);
#endif
#ifdef CONFIG_COMEDI_PCL725
	REG(driver_pcl725);
#endif
#ifdef CONFIG_COMEDI_PCL726
	REG(driver_pcl726);
#endif
#ifdef CONFIG_COMEDI_RTI800
	REG(driver_rti800);
#endif
#ifdef CONFIG_COMEDI_RTI802
	REG(driver_rti802);
#endif
#ifdef CONFIG_COMEDI_RTI860
	REG(driver_rti860);
#endif
#ifdef CONFIG_COMEDI_PARPORT
	REG(driver_parport);
#endif
#ifdef CONFIG_COMEDI_DAS08JR
	REG(driver_das08jr);
#endif
#ifdef CONFIG_COMEDI_DAS1600
	REG(driver_das1600);
#endif
#ifdef CONFIG_COMEDI_DAS6402
	REG(driver_das6402);
#endif
#ifdef CONFIG_COMEDI_MULTIQ3
	REG(driver_multiq3);
#endif
#ifdef CONFIG_COMEDI_DT2815
	REG(driver_dt2815);
#endif
#ifdef CONFIG_COMEDI_DAS16
	REG(driver_das16);
#endif
#ifdef CONFIG_COMEDI_PCL812
	REG(driver_pcl812);
#endif
#ifdef CONFIG_COMEDI_DAQBOARD2000
	REG(driver_daqboard2000);
#endif
}

