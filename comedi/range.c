/*
    module/range.c
    comedi routines for voltage ranges

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

#define RANGE_C

#include <comedi_module.h>
#ifdef LINUX_V22
#include <asm/uaccess.h>
#endif



/*
--BEGIN-RANGE-DEFS--
RANGE_bipolar10
	-10	10
RANGE_bipolar5
	-5	5
RANGE_bipolar2_5
	-2.5	2.5
RANGE_unipolar10
	0	10
RANGE_unipolar5
	0	5
RANGE_unknown
	0	1	unitless
---END-RANGE-DEFS---
*/

/*
   	COMEDI_RANGEINFO
	range information ioctl

	arg:
		pointer to rangeinfo structure

	reads:
		range info structure
	
	writes:
		n comedi_krange structures to rangeinfo->range_ptr
*/
int do_rangeinfo_ioctl(comedi_device *dev,comedi_rangeinfo *arg)
{
	comedi_rangeinfo it;

	if(copy_from_user(&it,arg,sizeof(comedi_rangeinfo)))
		return -EFAULT;

	if(RANGE_OFFSET(it.range_type)+RANGE_LENGTH(it.range_type)>comedi_max_range)
		return -EINVAL;

	if(copy_to_user(it.range_ptr,comedi_kranges+RANGE_OFFSET(it.range_type),
		sizeof(comedi_krange)*RANGE_LENGTH(it.range_type)))
		return -EFAULT;
	
	return 0;
}


/*
   This function checks each element in a channel/gain list to make
   make sure it is valid.
*/
int check_chanlist(comedi_subdevice *s,int n,unsigned int *chanlist)
{
	int i;
	int chan;


	if(s->range_type){
		for(i=0;i<n;i++)
		   	if(CR_CHAN(chanlist[i])>=s->n_chan || 
			   CR_RANGE(chanlist[i])>=RANGE_LENGTH(s->range_type)){
				rt_printk("bad chanlist[%d]=0x%08x n_chan=%d range length=%d\n",
					i,chanlist[i],s->n_chan,RANGE_LENGTH(s->range_type));
#if 0
for(i=0;i<n;i++){
	printk("[%d]=0x%08x\n",i,chanlist[i]);
}
#endif
				return -EINVAL;
			}
	}else if(s->range_type_list){
		for(i=0;i<n;i++){
			chan=CR_CHAN(chanlist[i]);
			if(chan>=s->n_chan ||
			   CR_RANGE(chanlist[i])>=RANGE_LENGTH(s->range_type_list[chan])){
				rt_printk("bad chanlist[%d]=0x%08x\n",i,chanlist[i]);
				return -EINVAL;
			}
		}
	}else{
		rt_printk("comedi: (bug) no range type list!\n");
		return -EINVAL;
	}
	return 0;
}


