/*
    module/pcmad
    hardware driver for Winsystems PCM-A/D12 and PCM-A/D16

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 2000 David A. Schleef <ds@stm.lbl.gov>

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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/malloc.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <comedi_module.h>


#define PCMAD_SIZE		4

#define PCMAD_STATUS		0
#define PCMAD_LSB		1
#define PCMAD_MSB		2
#define PCMAD_CONVERT		1

static int pcmad_attach(comedi_device *dev,comedi_devconfig *it);
static int pcmad_detach(comedi_device *dev);
static int pcmad_recognize(char *name);
comedi_driver driver_pcmad={
	driver_name:	"pcmad",
	module:		THIS_MODULE,
	attach:		pcmad_attach,
	detach:		pcmad_detach,
	recognize:	pcmad_recognize,
};

struct pcmad_board_struct{
	char *name;
	int n_ai_bits;
};
struct pcmad_board_struct pcmad_boards[]={
	{
	name:		"pcmad12",
	n_ai_bits:	12,
	},
	{
	name:		"pcmad16",
	n_ai_bits:	16,
	},
};
#define this_board ((struct pcmad_board_struct *)(dev->board_ptr))
static int n_pcmad_boards=(sizeof(pcmad_boards)/sizeof(pcmad_boards[0]));

struct pcmad_priv_struct{
	int differential;
	int twos_comp;
};
#define devpriv ((struct pcmad_priv_struct *)dev->private)


#define TIMEOUT	100

static int pcmad_ai_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int i,msb,lsb;
	int chan;
	int data;

	chan=CR_CHAN(it->chanlist[0]);

	outb(chan,dev->iobase+PCMAD_CONVERT);

	for(i=0;i<TIMEOUT;i++){
		if((inb(dev->iobase+PCMAD_STATUS)&0x3) == 0x3)
			break;
	}
	lsb=inb(dev->iobase+PCMAD_LSB);
	msb=inb(dev->iobase+PCMAD_MSB);

	data=(msb<<8)|(lsb);

	if(devpriv->twos_comp){
		data ^= (1<<(this_board->n_ai_bits-1));
	}
	it->data[0]=data;
	
	return 1;
}

static int pcmad_recognize(char *name)
{
	int i;

	for(i=0;i<n_pcmad_boards;i++){
		if(!strcmp(pcmad_boards[i].name,name))
			return i;
	}

	return -1;
}

/*
 * options:
 * 0	i/o base
 * 1	unused
 * 2	0=single ended 1=differential
 * 3	0=straight binary 1=two's comp
 */
static int pcmad_attach(comedi_device *dev,comedi_devconfig *it)
{
	int ret;
	comedi_subdevice *s;

	dev->iobase=it->options[0];
	printk("comedi%d: pcmad: 0x%04x ",dev->minor,dev->iobase);
	if(check_region(dev->iobase,PCMAD_SIZE)<0){
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(dev->iobase,PCMAD_SIZE,"pcmad");
	dev->iobase=dev->iobase;
	dev->iosize=PCMAD_SIZE;

	dev->n_subdevices=1;
	if((ret=alloc_subdevices(dev))<0)
		return ret;
	if((ret=alloc_private(dev,sizeof(struct pcmad_priv_struct)))<0)
		return ret;

	dev->board_ptr = pcmad_boards+dev->board;

	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=16;			/* XXX */
	s->len_chanlist=1;
	s->trig[0]=pcmad_ai_mode0;
	s->maxdata=(1<<this_board->n_ai_bits)-1;
	s->range_table=&range_unknown;

	return 0;
}


static int pcmad_detach(comedi_device *dev)
{
	printk("comedi%d: pcmad: remove\n",dev->minor);
	
	if(dev->irq){
		free_irq(dev->irq,dev);
	}
	release_region(dev->iobase,dev->iosize);

	return 0;
}


COMEDI_INITCLEANUP(driver_pcmad);

