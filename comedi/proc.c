/*
    module/proc.c
    /proc interface for comedi

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1998 David A. Schleef <ds@stm.lbl.gov>

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

/*
	This is some serious bloatware.
	
	Taken from Dave A.'s PCL-711 driver, 'cuz I thought it
	was cool.
*/


#include <linux/comedidev.h>
#include <linux/proc_fs.h>
#include <linux/string.h>


#if LINUX_VERSION_CODE < 0x020100
int comedi_read_procmem(char *buf,char **start,off_t offset,int len,int unused);

struct proc_dir_entry comedi_proc_entry =
{
	0,
	6, "comedi",
	S_IFREG | S_IRUGO,
	1, 0, 0,
	0, NULL,
	&comedi_read_procmem,
};

#else
int comedi_read_procmem(char *buf,char **start,off_t offset,int len,int *eof,void *data);



#endif

extern comedi_driver *comedi_drivers;

#if LINUX_VERSION_CODE < 0x020100
int comedi_read_procmem(char *buf,char **start,off_t offset,int len,int unused)
#else
int comedi_read_procmem(char *buf,char **start,off_t offset,int len,int *eof,void *data)
#endif
{
	int i;
	int devices_q=0;
	int l=0;
	comedi_driver *driv;
	
	l+=sprintf(buf+l,
		"comedi version " COMEDI_RELEASE "\n"
		"format string: %s\n",
		"\"%2d: %-20s %-20s %4d\",i,driver_name,board_name,n_subdevices");

	for(i=0;i<COMEDI_NDEVICES;i++){
		comedi_device *dev;

		dev=comedi_get_device_by_minor(i);
		if(dev->attached){
			devices_q=1;
			l+=sprintf(buf+l,"%2d: %-20s %-20s %4d\n",
				i,
				dev->driver->driver_name,
				dev->board_name,
				dev->n_subdevices
				);
		}
	}
	if(!devices_q){
		l+=sprintf(buf+l,"no devices\n");
	}

	for(driv=comedi_drivers;driv;driv=driv->next){
		l += sprintf(buf+l,"%s:\n",driv->driver_name);
		for(i=0;i<driv->num_names;i++){
			l+=sprintf(buf+l," %s\n",
				*(char **)(driv->board_name+i*driv->offset));
		}
		if(!driv->num_names){
			l+=sprintf(buf+l," %s\n",driv->driver_name);
		}
	}

	return l;
}


void comedi_proc_init(void)
{
#if LINUX_VERSION_CODE < 0x020100
	proc_register_dynamic(&proc_root,&comedi_proc_entry);
#else
	struct proc_dir_entry *comedi_proc;
	
	comedi_proc = create_proc_entry("comedi",S_IFREG | S_IRUGO,0);
	if(comedi_proc)
		comedi_proc->read_proc = comedi_read_procmem;
#endif
}

void comedi_proc_cleanup(void)
{
#if LINUX_VERSION_CODE < 0x020100
	proc_unregister(&proc_root,comedi_proc_entry.low_ino);
#else
	remove_proc_entry("comedi",0);
#endif
}


