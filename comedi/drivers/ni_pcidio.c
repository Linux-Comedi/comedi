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
Driver: ni_pcidio.o
Description: National Instruments PCI-DIO32HS, PCI-DIO96, PCI-6533, PCI-6503
Author: ds
Status: works in immediate mode
Devices: [National Instruments] PCI-DIO-32HS (ni_pcidio), PXI-6533,
  PCI-DIO-96, PCI-DIO-96B, PXI-6508, PCI-6503, PCI-6503B, PCI-6503X,
  PXI-6503

The DIO-96 appears as four 8255 subdevices.  See the 8255
driver notes for details.

The DIO32HS board appears as one subdevice, with 32 channels.
Each channel is individually I/O configurable.  The channel order,
as one might guess, is 0=A0, 1=A1, 2=A2, ... 8=B0, 16=C0, 24=D0.

DMA is halfway completed, but not operational, for the PCI-DIO32HS.

This driver could be easily modified to support AT-MIO32HS and
AT-MIO96.
*/

/*
   This driver is for both the NI PCI-DIO-32HS and the PCI-DIO-96,
   which have very different architectures.  But, since the '96 is
   so simple, it is included here.

   Manuals (available from ftp://ftp.natinst.com/support/manuals)

	320938c.pdf	PCI-DIO-96/PXI-6508/PCI-6503 User Manual
	321464b.pdf	AT/PCI-DIO-32HS User Manual
	341329A.pdf	PCI-6533 Register-Level Programmer Manual
	341330A.pdf	DAQ-DIO Technical Reference Manual

 */

//#define USE_CMD 1

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/comedidev.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/init.h>

#include <asm/io.h>
#include "mite.h"
#include "8255.h"

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

#define IntEn 3

static int nidio_attach(comedi_device *dev,comedi_devconfig *it);
static int nidio_detach(comedi_device *dev);
static comedi_driver driver_pcidio={
	driver_name:	"ni_pcidio",
	module:		THIS_MODULE,
	attach:		nidio_attach,
	detach:		nidio_detach,
};
COMEDI_INITCLEANUP(driver_pcidio);

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
	dev_id:		0x1320,
	name:		"pxi-6533",
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
	dev_id:		0x1630,
	name:		"pci-dio-96b",
	n_8255:		4,
	is_diodaq:	0,
	},
	{
	dev_id:		0x13c0,
	name:		"pxi-6508",
	n_8255:		4,
	is_diodaq:	0,
	},
	{
	dev_id:		0x0400,
	name:		"pci-6503",
	n_8255:		1,
	is_diodaq:	0,
	},
	{
	dev_id:		0x1250,
	name:		"pci-6503b",
	n_8255:		1,
	is_diodaq:	0,
	},
	{
	dev_id:		0x17d0,
	name:		"pci-6503x",
	n_8255:		1,
	is_diodaq:	0,
	},
	{
	dev_id:		0x1800,
	name:		"pxi-6503",
	n_8255:		1,
	is_diodaq:	0,
	},
};
#define n_nidio_boards (sizeof(nidio_boards)/sizeof(nidio_boards[0]))
#define this_board ((nidio_board *)dev->board_ptr)

static struct pci_device_id ni_pcidio_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_NATINST, 0x1150, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1320, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x0160, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1630, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x13c0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x0400, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1250, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x17d0, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_NATINST, 0x1800, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, ni_pcidio_pci_table);

typedef struct{
	struct mite_struct *mite;
	int boardtype;
	int dio;
}nidio96_private;
#define devpriv ((nidio96_private *)dev->private)

#ifdef USE_CMD
static int ni_pcidio_cmdtest(comedi_device *dev,comedi_subdevice *s,
				  comedi_cmd *cmd);
static int ni_pcidio_cmd(comedi_device *dev,comedi_subdevice *s);
#endif
static int nidio_find_device(comedi_device *dev,int bus,int slot);

static int nidio96_8255_cb(int dir,int port,int data,unsigned long iobase)
{
	if(dir){
		writeb(data,iobase+port);
		return 0;
	}else{
		return readb(iobase+port);
	}
}

#ifdef USE_CMD
static void nidio_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev=d;
	comedi_subdevice *s=dev->subdevices;
	int a,b;
	static int n_int=0;
	int i;
	struct timeval tv;
	//int tag=0;
	long int AuxData = 0;
	sampl_t MyData1 = 0;
	sampl_t MyData2 = 0;
	comedi_async *async;
	int Flags;
	int Status;
	int j;


	//writeb(0x00,dev->iobase+Master_DMA_And_Interrupt_Control);
	

	async = s->async;
	async->events = 0;	
	
	//printk("%d ",async->buf_int_count);
       
	//writeb(0x00,dev->iobase+Master_DMA_And_Interrupt_Control);

	Flags = readb(dev->iobase+Group_Flags);
	
	//IntEn es un parametre!!!!
	
	Status = readb(dev->iobase+Interrupt_And_Window_Status);
	
	//interrupcions parasites
	 if(dev->attached == 0)
	   {
	     comedi_error(dev,"premature interrupt");
	     async->events |= COMEDI_CB_ERROR|COMEDI_CB_EOA;	
	   }
	 
	 if((Flags & (TransferReady | CountExpired | Waited | PrimaryTC | SecondaryTC))==0)
	   {
	     comedi_error(dev,"spurious interrupt");
	     async->events |= COMEDI_CB_ERROR|COMEDI_CB_EOA;	
	   }
	 // printk("IntEn=%d,Flags=%d,Status=%d\n",IntEn,Flags,Status);
	 
	 while(Status&1)
	   {
	     //printk("1)IntEn=%d,Flags=%d,Status=%d\n",IntEn,Flags,Status);
	     //printk("hola11\n");
	    
	    if(Flags & IntEn & CountExpired)
	      {
		printk("count expired ");
		writeb(CountExpired,dev->iobase+Group_First_Clear);
		async->events |= COMEDI_CB_EOA;	
		async->events |= COMEDI_CB_BLOCK;
		//for(i=0;i<16;i++)
		//{
		    AuxData = readl(dev->iobase+Group_FIFO);
		    MyData1 = AuxData & 0xffff;
		    MyData2 = (AuxData & 0xffff0000) >> 16;
		    comedi_buf_put(async,MyData1);
		    comedi_buf_put(async,MyData2);
		    //printk("lectura:%d, %d\n",MyData1,MyData2);
		    //}
		
		writeb(0x00,dev->iobase+OpMode);
		writeb(0x00,dev->iobase+Master_DMA_And_Interrupt_Control);
		writeb(0xff,dev->iobase+Group_First_Clear);
		writeb(0xff,dev->iobase+Group_First_Clear);
		break;
	      }
	    
	    else if(Flags & IntEn & TransferReady)
	      {
		//aixo no surt en el manual de NI
		writeb(TransferReady,dev->iobase+Group_First_Clear);
		//printk("transfer ready 1º");
		//for(i=0;i<16;i++)
		//{
		/*Flags = 0;
		    j=0;
		    while(!(Flags & TransferReady) && j<320)
		       {
			Flags = readb(dev->iobase+Group_Flags);
			//printk("Flags:%d ",Flags);
			j++;
			}*/
		    AuxData = readl(dev->iobase+Group_FIFO);
		    MyData1 = AuxData & 0xffff;
		    MyData2 = (AuxData & 0xffff0000) >> 16;
		    comedi_buf_put(async,MyData1);
		    comedi_buf_put(async,MyData2);
		    //printk("lectura:%d, %d\n",MyData1,MyData2);
		    //Flags = readb(dev->iobase+Group_Flags);
		    //}
		//  Flags = readb(dev->iobase+Group_Flags);
		// }
		//printk("%d ",async->buf_int_count);
		//printk("1)IntEn=%d,Flags=%d,Status=%d\n",IntEn,Flags,Status);
		async->events |= COMEDI_CB_BLOCK;
	      }
	    
	    else if(Flags & IntEn & Waited)
	      {
		printk("waited\n");
		writeb(Waited,dev->iobase+Group_First_Clear);
		writeb(0x00,dev->iobase+Master_DMA_And_Interrupt_Control);
	      }
	    
	    else if(Flags & IntEn & PrimaryTC)
	      {
		printk("primaryTC\n");
		writeb(PrimaryTC,dev->iobase+Group_First_Clear);
		async->events |= COMEDI_CB_EOA;
		writeb(0x00,dev->iobase+Master_DMA_And_Interrupt_Control);
	      }
	    
	    else if(Flags & IntEn & SecondaryTC)
	      {
		printk("secondaryTC\n");
		writeb(SecondaryTC,dev->iobase+Group_First_Clear);
		async->events |= COMEDI_CB_EOA;
		writeb(0x00,dev->iobase+Master_DMA_And_Interrupt_Control);
	      }
	    
	    /*else
	      {
		printk("interrupcio desconeguda\n");
		async->events |= COMEDI_CB_ERROR|COMEDI_CB_EOA;
		writeb(0x00,dev->iobase+Master_DMA_And_Interrupt_Control);
		}*/
	
	    Flags = readb(dev->iobase+Group_Flags);
	    Status = readb(dev->iobase+Interrupt_And_Window_Status);
	    //printk("2) IntEn=%d,Flags=%d,Status=%d\n",IntEn,Flags,Status);
	   }

	comedi_event(dev,s,async->events);
	async->events = 0;
	    
	//printk("hola2\n");

	//if  (!tag) 
	//  writeb(0x03,dev->iobase+Master_DMA_And_Interrupt_Control);

	/*

	do_gettimeofday(&tv);
	a=readb(dev->iobase+Group_Status);
	b=readb(dev->iobase+Group_Flags);

	if(n_int < 10){
		DPRINTK("status 0x%02x flags 0x%02x time %06d\n",a,b,(int)tv.tv_usec);
	}

	while(b&1){
		writew(0xff,dev->iobase+Group_FIFO);
		b=readb(dev->iobase+Group_Flags);
	}

	b=readb(dev->iobase+Group_Flags);

	if(n_int < 10){
		DPRINTK("new status 0x%02x\n",b);
		n_int++;
		}*/
	return;
}
#endif

static int ni_pcidio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=1)return -EINVAL;
	switch(data[0]){
	case COMEDI_OUTPUT:
		s->io_bits |= 1<<CR_CHAN(insn->chanspec);
		break;
	case COMEDI_INPUT:
		s->io_bits &= ~(1<<CR_CHAN(insn->chanspec));
		break;
	default:
		return -EINVAL;
	}
	writel(s->io_bits,dev->iobase+Port_Pin_Directions(0));

	return 1;
}

static int ni_pcidio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=2)return -EINVAL;
	if(data[0]){
		s->state &= ~data[0];
		s->state |= (data[0]&data[1]);
		writel(s->state,dev->iobase+Port_IO(0));
	}
	data[1] = readl(dev->iobase+Port_IO(0));

	return 2;
}

#ifdef USE_CMD
static int ni_pcidio_cmdtest(comedi_device *dev,comedi_subdevice *s,
	comedi_cmd *cmd)
{
	int err=0;
	int tmp;

	/* cmdtest tests a particular command to see if it is valid.
	 * Using the cmdtest ioctl, a user can create a valid cmd
	 * and then have it executes by the cmd ioctl.
	 *
	 * cmdtest returns 1,2,3,4 or 0, depending on which tests
	 * the command passes. */

	/* step 1: make sure trigger sources are trivially valid */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER|TRIG_EXT;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER|TRIG_EXT;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT|TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	/* note that mutual compatiblity is not an issue here */
	if(cmd->scan_begin_src!=TRIG_TIMER &&
	   cmd->scan_begin_src!=TRIG_EXT)err++;
	if(cmd->convert_src!=TRIG_TIMER &&
	   cmd->convert_src!=TRIG_EXT)err++;
	if(cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_NONE)err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}

#define MAX_SPEED	10000		/* in nanoseconds */
#define MIN_SPEED	1000000000	/* in nanoseconds */

	if(cmd->scan_begin_src==TRIG_TIMER){
		if(cmd->scan_begin_arg<MAX_SPEED){
			cmd->scan_begin_arg=MAX_SPEED;
			err++;
		}
		if(cmd->scan_begin_arg>MIN_SPEED){
			cmd->scan_begin_arg=MIN_SPEED;
			err++;
		}
	}else{
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		/* should specify multiple external triggers */
		if(cmd->scan_begin_arg>9){
			cmd->scan_begin_arg=9;
			err++;
		}
	}
	if(cmd->convert_src==TRIG_TIMER){
		if(cmd->convert_arg<MAX_SPEED){
			cmd->convert_arg=MAX_SPEED;
			err++;
		}
		if(cmd->convert_arg>MIN_SPEED){
			cmd->convert_arg=MIN_SPEED;
			err++;
		}
	}else{
		/* external trigger */
		/* see above */
		if(cmd->convert_arg>9){
			cmd->convert_arg=9;
			err++;
		}
	}

	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src==TRIG_COUNT){
		if(cmd->stop_arg>0x00ffffff){
			cmd->stop_arg=0x00ffffff;
			err++;
		}
	}else{
		/* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up any arguments */
	

	if(err)return 4;

	return 0;
}


static int ni_pcidio_cmd(comedi_device *dev,comedi_subdevice *s)
{
  int i;

  writeb(  0 ,dev->iobase+OpMode);

  /*ho vui per entrada(els ports)*/
  writel(0x0000,dev->iobase+Port_Pin_Directions(0));
  
  /* choose chan A,B (grup in,i 4 fifos sense funneling) */
  writeb(0x0F,dev->iobase+Data_Path);
  
  /* set transfer width a 32 bits*/
  writeb(0x00,dev->iobase+Transfer_Size_Control);


  /* protocol configuration */
  writeb(0x00,dev->iobase+ClockReg);
  writeb(  0 ,dev->iobase+Sequence);
  writeb(0x00,dev->iobase+ReqReg);
  writeb(  4 ,dev->iobase+BlockMode);
  writeb(  0 ,dev->iobase+LinePolarities);
  writeb(0x00,dev->iobase+AckSer);
  writeb(  1 ,dev->iobase+StartDelay);
  writeb(  1 ,dev->iobase+ReqDelay);
  writeb(  1 ,dev->iobase+ReqNotDelay);
  writeb(  1 ,dev->iobase+AckDelay);
  writeb(0x0C,dev->iobase+AckNotDelay);
  writeb(0x10,dev->iobase+Data1Delay);
  writew(  0 ,dev->iobase+ClockSpeed);
  writeb(0x60,dev->iobase+DAQOptions);

  /* ReadyLevel */
  /*writeb(  0 ,dev->iobase+FIFO_Control);*/
  
  /*for(i=0;i<16;i++){
    writew(i,dev->iobase+Group_FIFO);
    }*/
  //printk("arg:%d\n",s->async->cmd.stop_arg);
  writel(s->async->cmd.stop_arg,dev->iobase+Transfer_Count);
  
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
  /* interrupcions degudas a FIFO ple, s'ha fet totes les transferencies
     primaryTC=1??? secondaryTC=1???*/
  writeb(IntEn,dev->iobase+Interrupt_Enable);
  writeb(0x03,dev->iobase+Master_DMA_And_Interrupt_Control);
  
  /* start */
  /*amb aixo ja s'esta dient numbered=1 + runmode=7*/
  writeb(0x0f,dev->iobase+OpMode);
  
  printk("\nhas configurat la tarja chaval\n");
  return 0;
}
#endif

#ifdef unused
static int nidio_dio_cmd(comedi_device *dev,comedi_subdevice *s)
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
#endif

static int nidio_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int i;
	int ret;
	
	printk("comedi%d: nidio:",dev->minor);

	if((ret=alloc_private(dev,sizeof(nidio96_private)))<0)
		return ret;
	
	ret=nidio_find_device(dev,it->options[0],it->options[1]);
	if(ret<0)return ret;

	ret = mite_setup(devpriv->mite);
	if(ret < 0)
	{
		printk("error setting up mite\n");
		return ret;
	}
	dev->iobase = mite_iobase(devpriv->mite);

	dev->board_name=this_board->name;
	dev->irq=mite_irq(devpriv->mite);
	printk(" %s",dev->board_name);

	if(!this_board->is_diodaq){
		dev->n_subdevices=this_board->n_8255;
	}else{
		dev->n_subdevices=1;
	}
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	if(!this_board->is_diodaq){
		for(i=0;i<this_board->n_8255;i++){
			subdev_8255_init(dev,dev->subdevices+i,
				nidio96_8255_cb,(unsigned long)(dev->iobase+NIDIO_8255_BASE(i)));
		}
	}else{

		printk(" rev=%d",readb(dev->iobase+Chip_Version));
	
		s=dev->subdevices+0;

		dev->read_subdev = s;
		s->type=COMEDI_SUBD_DIO;
		s->subdev_flags=SDF_READABLE|SDF_WRITEABLE|SDF_RT;
		s->n_chan=32;
		s->range_table=&range_digital;
		s->maxdata=1;
		s->insn_config = ni_pcidio_insn_config;
		s->insn_bits = ni_pcidio_insn_bits;
#ifdef USE_CMD
		s->do_cmd = ni_pcidio_cmd;
		s->do_cmdtest = ni_pcidio_cmdtest;
#endif
		s->len_chanlist=32;		/* XXX */

		writel(0,dev->iobase+Port_IO(0));
		writel(0,dev->iobase+Port_Pin_Directions(0));
		writel(0,dev->iobase+Port_Pin_Mask(0));

#ifdef USE_CMD
		/* disable interrupts on board */
		writeb(0x00,dev->iobase+Master_DMA_And_Interrupt_Control);

		ret=comedi_request_irq(dev->irq,nidio_interrupt,SA_SHIRQ,"nidio",dev);
		if(ret<0){
			dev->irq=0;
			printk(" irq not available");
		}
#endif
	}

	printk("\n");

	return 0;
}

static int nidio_detach(comedi_device *dev)
{
	int i;

	if(this_board && !this_board->is_diodaq){
		for(i=0;i<this_board->n_8255;i++){
			subdev_8255_cleanup(dev,dev->subdevices+i);
		}
	}

	if(dev->irq)
		comedi_free_irq(dev->irq,dev);

	if(devpriv && devpriv->mite)
		mite_unsetup(devpriv->mite);

	return 0;
}


static int nidio_find_device(comedi_device *dev,int bus,int slot)
{
	struct mite_struct *mite;
	int i;
	
	for(mite=mite_devices;mite;mite=mite->next){
		if(mite->used)continue;
		if(bus || slot){
#ifdef PCI_SUPPORT_VER1
			if(bus!=(mite->pci_bus<<8) ||
			   slot!=PCI_SLOT(mite->pci_device_fn))
				continue;
#else
			if(bus!=(mite->pcidev->bus->number<<8) ||
			   slot!=PCI_SLOT(mite->pcidev->devfn))
				continue;
#endif
		}
		for(i=0;i<n_nidio_boards;i++){
			if(mite_device_id(mite)==nidio_boards[i].dev_id){
				dev->board_ptr=nidio_boards+i;
				devpriv->mite=mite;

				return 0;
			}
		}
	}
	printk("no device found\n");
	mite_list_devices();
	return -EIO;
}

