/*
    module/ni-dio.c
    driver for National Instruments PCI-DIO-96/PCI-6508
               National Instruments PCI-DIO-32HS
               National Instruments PCI-6503

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1999 David A. Schleef <ds@stm.lbl.gov>

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
   This driver is for both the NI PCI-DIO-32HS and the PCI-DIO-96,
   which have very different architectures.  But, since the '96 is
   so simple, it is included here.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <comedi_module.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/malloc.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <mite.h>
#include <8255.h>

/* general debugging messages */
#define DEBUG


#define PCI_VENDOR_ID_NATINST	0x1093

#define PCI_DIO_SIZE 4096
#define PCI_MITE_SIZE 4096

/* defines for the PCI-DIO-96 */

#define NIDIO_8255_BASE(x)	((x)*4)
#define NIDIO_A 0
#define NIDIO_B 4
#define NIDIO_C 8
#define NIDIO_D 12

/* defines for the PCI-DIO-32HS */

#define Chip_ID_D			24
#define Chip_ID_I			25
#define Chip_ID_O			26
#define Chip_Version			27

#define Port_IO(x)			(28+(x))
#define Port_Pin_Directions(x)		(32+(x))
#define Port_Pin_Mask(x)		(36+(x))
#define Port_Pin_Polarities(x)		(40+(x))
#define Port_Pattern(x)			(48+(x))

#define Master_DMA_And_Interrupt_Control 5
#define InterruptLine				3
#define OpenInt					4

#define Interrupt_And_Window_Status	4
#define Group_Status			5
#define DataLeft				(1<<0)
#define Req					(1<<2)
#define StopTrig				(1<<3)

#define Group_Flags			6
#define TransferReady				(1<<0)
#define CountExpired				(1<<1)
#define Waited					(1<<5)
#define PrimaryTC				(1<<6)
#define SecondaryTC				(1<<7)

#define Group_First_Clear		6
#define ClearWaited				(1<<3)
#define ClearPrimaryTC				(1<<4)
#define ClearSecondaryTC			(1<<5)
#define DMAReset				(1<<6)
#define FIFOReset				(1<<7)
#define ClearAll				0xf8

#define Group_FIFO			8
#define Transfer_Count			20
#define Data_Path			64
#define FIFO_Control			72
#define Interrupt_Enable		75
#define DMA_Line_Control		76
#define Transfer_Size_Control		77

#define Protocol_Register_1		65
#define Protocol_Register_2		66
#define Protocol_Register_3		67
#define Protocol_Register_4		70
#define Protocol_Register_5		71
#define Protocol_Register_6		73
#define Protocol_Register_7		74
#define Protocol_Register_8		88
#define Protocol_Register_9		82
#define Protocol_Register_10		83
#define Protocol_Register_11		84
#define Protocol_Register_12		85
#define Protocol_Register_13		86
#define Protocol_Register_14		68
#define Protocol_Register_15		79

#define OpMode				Protocol_Register_1
#define ClockReg			Protocol_Register_2
#define Sequence			Protocol_Register_3
#define ReqReg				Protocol_Register_4
#define BlockMode			Protocol_Register_5
#define LinePolarities			Protocol_Register_6
#define AckSer				Protocol_Register_7
#define StartDelay			Protocol_Register_8
#define ReqDelay			Protocol_Register_9
#define ReqNotDelay			Protocol_Register_10
#define AckDelay			Protocol_Register_11
#define AckNotDelay			Protocol_Register_12
#define Data1Delay			Protocol_Register_13
#define ClockSpeed			Protocol_Register_14
#define DAQOptions			Protocol_Register_15

static int nidio_attach(comedi_device *dev,comedi_devconfig *it);
static int nidio_detach(comedi_device *dev);
comedi_driver driver_pcidio={
	driver_name:	"nidio",
	module:		&__this_module,
	attach:		nidio_attach,
	detach:		nidio_detach,
};

typedef struct{
	int dev_id;
	char *name;
	int n_8255;
	int is_diodaq;
}nidio_board;
static nidio_board nidio_boards[]={
	{
	dev_id:		0x1150,
	name:		"pci-dio-32hs",
	n_8255:		0,
	is_diodaq:	1,
	},
	{
	dev_id:		0x0160,
	name:		"pci-dio-96",
	n_8255:		4,
	is_diodaq:	0,
	},
	{
	dev_id:		0x17d0,
	name:		"pci-6503",
	n_8255:		1,
	is_diodaq:	0,
	},
};
#define n_nidio_boards (sizeof(nidio_boards)/sizeof(nidio_boards[0]))

typedef struct{
	struct mite_struct *mite;
	int boardtype;
	int dio;
}nidio96_private;
#define devpriv ((nidio96_private *)dev->private)

static int nidio_find_device(comedi_device *dev,comedi_devconfig *it);
#if 0
static int setup_mite(comedi_device *dev);
#endif


static int nidio96_8255_cb(int dir,int port,int data,void *arg)
{
	int iobase=(int)arg;

	if(dir){
		writeb(data,iobase+port);
		return 0;
	}else{
		return readb(iobase+port);
	}
}

static void nidio_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev=d;
	//comedi_subdevice *s=dev->subdevices;
	int a,b;
	static int n_int=0;
	//int i;
	struct timeval tv;

	do_gettimeofday(&tv);
	a=readb(dev->iobase+Group_Status);
	b=readb(dev->iobase+Group_Flags);
	printk("status 0x%02x flags 0x%02x time %06d\n",a,b,(int)tv.tv_usec);

	while(b&1){
		writew(0xff,dev->iobase+Group_FIFO);
		b=readb(dev->iobase+Group_Flags);
	}

	b=readb(dev->iobase+Group_Flags);
	printk("new status 0x%02x\n",b);

	n_int++;
	if(n_int==10)
		disable_irq(dev->irq);
}

static int nidio_dio(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	if(it->flags & TRIG_CONFIG){
		do_pack(&s->io_bits,it);
		writel(s->io_bits,dev->iobase+Port_Pin_Directions(0));
	}else{
		if(it->flags&TRIG_WRITE){
			do_pack(&s->state,it);
			writel(s->state,dev->iobase+Port_IO(0));
		}else{
			unsigned int data;

			data=readl(dev->iobase+Port_IO(0));
			di_unpack(data,it);
		}
	}

	return it->n_chan;
}

static int nidio_dio_mode2(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int i;

	writeb(  0 ,dev->iobase+OpMode);

	writel(0xffff,dev->iobase+Port_Pin_Directions(0));

	/* choose chan A,B */
	writeb(0x83,dev->iobase+Data_Path);

	/* set transfer width */
	writeb(0x03,dev->iobase+Transfer_Size_Control);

#if 0
	/* protocol configuration */
	writeb(0x10,dev->iobase+ClockReg);
	writeb(  0 ,dev->iobase+Sequence);
	writeb(0x20,dev->iobase+ReqReg);
	writeb(  4 ,dev->iobase+BlockMode);
	writeb(  0 ,dev->iobase+LinePolarities);
	writeb(0x60,dev->iobase+AckSer);
	writeb(  1 ,dev->iobase+StartDelay);
	writeb(  1 ,dev->iobase+ReqDelay);
	writeb(  1 ,dev->iobase+ReqNotDelay);
	writeb(  1 ,dev->iobase+AckDelay);
	writeb(  1 ,dev->iobase+AckNotDelay);
	writeb(  1 ,dev->iobase+Data1Delay);
	writew(  0 ,dev->iobase+ClockSpeed);
	writeb(  0 ,dev->iobase+DAQOptions);
#else
	/* protocol configuration */
	writeb(  0 ,dev->iobase+ClockReg);
	writeb(  1 ,dev->iobase+Sequence);
	writeb(  4 ,dev->iobase+ReqReg);
	writeb(  0 ,dev->iobase+BlockMode);
	writeb(  3 ,dev->iobase+LinePolarities);
	writeb(224 ,dev->iobase+AckSer);
	writeb(100 ,dev->iobase+StartDelay);
	writeb(  1 ,dev->iobase+ReqDelay);
	writeb(  1 ,dev->iobase+ReqNotDelay);
	writeb(  1 ,dev->iobase+AckDelay);
	writeb( 11 ,dev->iobase+AckNotDelay);
	writeb(  1 ,dev->iobase+Data1Delay);
	writew(1000,dev->iobase+ClockSpeed);
	writeb(100 ,dev->iobase+DAQOptions);
#endif

	/* ReadyLevel */
	writeb(  0 ,dev->iobase+FIFO_Control);

	for(i=0;i<16;i++){
		writew(i,dev->iobase+Group_FIFO);
	}

	writel(10000 ,dev->iobase+Transfer_Count);

#define USEDMA 0
	if(USEDMA){
		writeb(0x05,dev->iobase+DMA_Line_Control);
		writeb(0x30,dev->iobase+Group_First_Clear);

		mite_dma_prep(devpriv->mite,s);
	}else{
		writeb(0x00,dev->iobase+DMA_Line_Control);
	}

	/* clear and enable interrupts */
	writeb(0xff,dev->iobase+Group_First_Clear);
	writeb(0xc3,dev->iobase+Interrupt_Enable);
	writeb(0x03,dev->iobase+Master_DMA_And_Interrupt_Control);

	/* start */

	writeb(0x0f,dev->iobase+OpMode);

	return 0;
}

static int nidio_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int i;
	int ret;
	
	printk("comedi%d: nidio:",dev->minor);

	if((ret=alloc_private(dev,sizeof(nidio96_private)))<0)
		return ret;
	
	ret=nidio_find_device(dev,it);
	if(ret<0)return ret;

	dev->iobase=mite_setup(devpriv->mite);

	dev->board_name=nidio_boards[dev->board].name;
	dev->irq=mite_irq(devpriv->mite);
	printk(" %s",dev->board_name);

	if(!nidio_boards[dev->board].is_diodaq){
		dev->n_subdevices=4;
	}else{
		dev->n_subdevices=1;
	}
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	if(!nidio_boards[dev->board].is_diodaq){
		for(i=0;i<nidio_boards[dev->board].n_8255;i++){
			subdev_8255_init(dev,dev->subdevices+i,
				nidio96_8255_cb,(void *)(dev->iobase+NIDIO_8255_BASE(i)));
		}
	}else{

		printk(" rev=%d",readb(dev->iobase+Chip_Version));
	
		s=dev->subdevices+0;

		s->type=COMEDI_SUBD_DIO;
		s->subdev_flags=SDF_READABLE|SDF_WRITEABLE|SDF_RT;
		s->n_chan=32;
		s->range_table=&range_digital;
		s->maxdata=1;
		s->trig[0]=nidio_dio;
		s->trig[2]=nidio_dio_mode2;
		s->len_chanlist=32;		/* XXX */

		writel(0,dev->iobase+Port_IO(0));
		writel(0,dev->iobase+Port_Pin_Directions(0));
		writel(0,dev->iobase+Port_Pin_Mask(0));

		/* disable interrupts on board */
		writeb(0x00,dev->iobase+Master_DMA_And_Interrupt_Control);

		ret=comedi_request_irq(dev->irq,nidio_interrupt,0,"nidio",dev);
		if(ret<0){
			dev->irq=0;
			printk(" irq not available");
		}
	}

	printk("\n");

	return 0;
}

static int nidio_detach(comedi_device *dev)
{
	if(dev->irq)
		comedi_free_irq(dev->irq,dev);

	if(devpriv && devpriv->mite)
		mite_unsetup(devpriv->mite);

	return 0;
}


static int nidio_find_device(comedi_device *dev,comedi_devconfig *it)
{
	struct mite_struct *mite;
	int i;
	
	for(mite=mite_devices;mite;mite=mite->next){
		for(i=0;i<n_nidio_boards;i++){
			if(mite_device_id(mite)==nidio_boards[i].dev_id){
				dev->board=i;
				devpriv->mite=mite;

				return 0;
			}
		}
	}
	printk("no device found\n");
	return -EIO;
}


#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_pcidio);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_pcidio);
}
#endif
