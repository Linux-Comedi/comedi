/*
    module/drivers.c
    functions for manipulating drivers

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-8 David A. Schleef <ds@stm.lbl.gov>

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

static void postconfig(comedi_device *dev);
#ifdef CONFIG_COMEDI_VER08
static int command_trig(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int mode_to_command(comedi_cmd *cmd,comedi_trig *it);
#endif

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
	__MOD_DEC_USE_COUNT(dev->driver->module);

	dev->attached=0;

	for(i=0;i<dev->n_subdevices;i++){
		s=dev->subdevices+i;
		if(s->prealloc_buf)
			rvfree(s->prealloc_buf,s->prealloc_bufsz);
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
	int i,ret;

	if(dev->attached)
		return -EBUSY;

	for(driv=comedi_drivers;driv;driv=driv->next){
		if(driv->recognize){
			i=driv->recognize(it->board_name);
			if(i<0)continue;
		}else{
			if(strcmp(driv->driver_name,it->board_name))
				continue;
		}
		ret=driv->attach(dev,it);
		if(ret<0){
			driv->detach(dev);
			if(dev->subdevices)kfree(dev->subdevices);
			if(dev->private)kfree(dev->private);

			return ret;
		}
		goto attached;
	}

	return -EIO;

attached:
	init_waitqueue_head(&dev->wait);

	/* do a little post-config cleanup */
	postconfig(dev);

	dev->attached=1;
	dev->driver=driv;

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

		dev=comedi_devices+i;
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


static void postconfig(comedi_device *dev)
{
	int i;
	int have_trig;
	comedi_subdevice *s;

	for(i=0;i<dev->n_subdevices;i++){
		s=dev->subdevices+i;

		if(s->type==COMEDI_SUBD_UNUSED)
			continue;

		if(s->len_chanlist==0)
			s->len_chanlist=1;

		have_trig=0;
		if(s->trig[1] || s->trig[2] || s->trig[3] ||s->trig[4])
			have_trig=1;

		if(s->do_cmd && !have_trig){
			s->trig[1]=command_trig;
			s->trig[2]=command_trig;
			s->trig[3]=command_trig;
			s->trig[4]=command_trig;
		}
		if(s->do_cmd || have_trig){
			s->prealloc_bufsz=1024*128;
		}else{
			s->prealloc_bufsz=0;
		}

		if(s->prealloc_bufsz){
			/* XXX */
			s->prealloc_buf=rvmalloc(s->prealloc_bufsz*sizeof(sampl_t));
			if(!s->prealloc_buf){
				printk("ENOMEM\n");
			}
		}

		if(!s->range_table && !s->range_table_list)
			s->range_table=&range_unknown;
	}

}

/* helper functions for drivers */

int di_unpack(unsigned int bits,comedi_trig *it)
{
	int chan;
	int i;

	for(i=0;i<it->n_chan;i++){
		chan=CR_CHAN(it->chanlist[i]);
		it->data[i]=(bits>>chan)&1;
	}

	return i;
}

int do_pack(unsigned int *bits,comedi_trig *it)
{
	int chan;
	int mask;
	int i;

	for(i=0;i<it->n_chan;i++){
		chan=CR_CHAN(it->chanlist[i]);
		mask=1<<chan;
		(*bits) &= ~mask;
		if(it->data[i])
			(*bits) |=mask;
	}

	return i;
}

#ifdef CONFIG_COMEDI_VER08
static int command_trig(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int ret;

	ret=mode_to_command(&s->cmd,it);
	if(ret)return ret;

	ret=s->do_cmdtest(dev,s,&s->cmd);
	if(ret)return ret;

	ret=s->do_cmd(dev,s);
	if(ret)return ret;

	return ret;
}

static int mode_to_command(comedi_cmd *cmd,comedi_trig *it)
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
#endif


#define REG(x) {extern comedi_driver (x);comedi_driver_register(&(x));}

void init_drivers(void)
{
	REG(driver_dummy);
#if 0
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
#endif
}

