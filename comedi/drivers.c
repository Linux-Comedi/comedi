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
static int command_trig(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int mode_to_command(comedi_cmd *cmd,comedi_trig *it);
static int insn_emulate(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int insn_emulate_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int insn_inval(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static int mode0_emulate(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig);

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
	int ret;
	int i=0;

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
		dev->board = i;
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
	init_waitqueue_head(&dev->read_wait);
	init_waitqueue_head(&dev->write_wait);

	/* do a little post-config cleanup */
	postconfig(dev);

	if(!dev->board_name){
		printk("BUG: dev->board_name=<%p>\n",dev->board_name);
		dev->board_name="BUG";
	}

	dev->attached=1;
	dev->driver=driv;

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
			s->prealloc_buf=rvmalloc(s->prealloc_bufsz);
			if(!s->prealloc_buf){
				printk("ENOMEM\n");
			}
		}

		if(!s->range_table && !s->range_table_list)
			s->range_table=&range_unknown;

#define EMULATE_INSN_WITH_TRIG
#ifdef EMULATE_INSN_WITH_TRIG
		if(!s->insn_read){
			if(s->insn_bits){
				s->insn_read = insn_emulate_bits;
			}else if(s->subdev_flags & SDF_READABLE){
				s->insn_read=insn_emulate;
			}else{
				s->insn_read=insn_inval;
			}
		}
		if(!s->insn_write){
			if(s->insn_bits){
				s->insn_write = insn_emulate_bits;
			}else if(s->subdev_flags & SDF_WRITEABLE){
				s->insn_write=insn_emulate;
			}else{
				s->insn_write=insn_inval;
			}
		}
		if(!s->insn_config){
			if(s->type==COMEDI_SUBD_DIO){
				s->insn_config = insn_emulate;
			}else{
				s->insn_config = insn_inval;
			}
		}

		if(!s->trig[0]){
			s->trig[0]=mode0_emulate;
		}
#else
		if(!s->insn_read)s->insn_read = insn_inval;
		if(!s->insn_write)s->insn_write = insn_inval;
		if(!s->insn_bits)s->insn_bits = insn_inval;
#endif
		
		if(!s->insn_bits){
			s->insn_bits = insn_inval;
		}
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

static int insn_inval(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	return -EINVAL;
}

#ifdef EMULATE_INSN_WITH_TRIG
static int insn_emulate(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	comedi_trig trig;
	int i;
	int ret;

	switch(insn->insn){
	case INSN_CONFIG:
		trig.flags=TRIG_CONFIG;
		break;
	case INSN_WRITE:
		if(!(s->subdev_flags & SDF_WRITEABLE))
			return -EINVAL;
		trig.flags=TRIG_WRITE;
		break;
	case INSN_READ:
		if(!(s->subdev_flags & SDF_READABLE))
			return -EINVAL;
		trig.flags=0;
		break;
	default:
		return -EINVAL;
	}

	trig.subdev=insn->subdev;
	trig.mode=0;
	trig.n_chan=1;
	trig.chanlist=&insn->chanspec;
	trig.n=1;

	if(s->subdev_flags & SDF_LSAMPL){
		for(i=0;i<insn->n;i++){
			trig.data=(void *)(data+i);
			ret=s->trig[0](dev,s,&trig);
			if(ret<0)return ret;
		}
	}else{
		sampl_t sdata;

		trig.data=&sdata;
		for(i=0;i<insn->n;i++){
			sdata=data[i];
			ret=s->trig[0](dev,s,&trig);
			if(ret<0)return ret;
			data[i]=sdata;
		}
	}

	return insn->n;
}
#endif

static int insn_emulate_bits(comedi_device *dev,comedi_subdevice *s,
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
	}else{
		new_data[0] = 0;
		new_data[1] = 0;
	}

	ret = s->insn_bits(dev,s,&new_insn,new_data);
	if(ret<0)return ret;

	if(insn->insn == INSN_READ){
		if(!(s->subdev_flags & SDF_WRITEABLE))
			return -EINVAL;
		data[0] = (new_data[1]>>chan)&1;
	}

	return 1;
}

#define SUPPORT_TRIG
#define SUPPORT_TRIG0
#ifdef SUPPORT_TRIG
static int mode0_emulate(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig)
{
	comedi_insn insn;
	lsampl_t ldata;
	int ret;

	insn.subdev=trig->subdev;
	insn.data=&ldata;
	insn.n=1;
	insn.chanspec=trig->chanlist[0];

	if(trig->flags&TRIG_CONFIG)
		return mode0_emulate_config();

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
#endif

#ifdef SUPPORT_TRIG0
static int mode0_emulate_config(comedi_device *dev,comedi_subdevice *s,comedi_trig *trig)
{
	comedi_insn insn;
	lsampl_t ldata;
	int ret;

	insn.subdev=trig->subdev;
	insn.data=&ldata;
	insn.n=1;
	insn.chanspec=trig->chanlist[0];

	ldata = trig->data[0];

	return s->insn_config(dev,s,&insn,&ldata);
}
#endif

#ifdef SUPPORT_TRIG
static int command_trig(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int ret;

	ret=mode_to_command(&s->cmd,it);
	if(ret)return ret;

	ret=s->do_cmdtest(dev,s,&s->cmd);
	if(ret)return -EINVAL;

	ret=s->do_cmd(dev,s);
	if(ret>0)return -EINVAL;
	return ret;
}
#endif

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

