/*
    module/dt3000.c
    Data Translation DT3000 series driver

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
   The DT3000 series is Data Translation's attempt to make a PCI
   data acquisition board.  The design of this series is very nice,
   since each board has an on-board DSP (Texas Instruments TMS320C52).
   However, a few details are a little annoying.  The boards lack
   bus-mastering DMA, which eliminates them from serious work.
   They also are not capable of autocalibration, which is a common
   feature in modern hardware.  The default firmware is pretty bad,
   making it nearly impossible to write an RT compatible driver.
   It would make an interesting project to write a decent firmware
   for these boards.

   Data Translation originally wanted an NDA for the documentation
   for the 3k series.  However, if you ask nicely, they might send
   you the docs without one, also.
*/

#include <linux/comedidev.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/init.h>

#include <asm/io.h>

#define PCI_VENDOR_ID_DT	0x1116

static comedi_lrange range_dt3000_ai = { 4, {
	RANGE( -10,	10 ),
	RANGE( -5,	5 ),
	RANGE( -2.5,	2.5 ),
	RANGE( -1.25,	1.25 )
}};
static comedi_lrange range_dt3000_ai_pgl = { 4, {
	RANGE( -10,	10 ),
	RANGE( -1,	1 ),
	RANGE( -0.1,	0.1 ),
	RANGE( -0.02,	0.02 )
}};

typedef struct{
	char *name;
	unsigned int device_id;
	int adchan;
	int adbits;
	comedi_lrange *adrange;
	int dachan;
	int dabits;
}dt3k_boardtype;

static dt3k_boardtype dt3k_boardtypes[]={
	{	name:		"dt3001",
		device_id:	0x22,
		adchan:		16,
		adbits:		12,
		adrange:	&range_dt3000_ai,
		dachan:		2,
		dabits:		12,
	},
	{	name:		"dt3001-pgl",
		device_id:	0x27,
		adchan:		16,
		adbits:		12,
		adrange:	&range_dt3000_ai_pgl,
		dachan:		2,
		dabits:		12,
	},
	{	name:		"dt3002",
		device_id:	0x23,
		adchan:		32,
		adbits:		12,
		adrange:	&range_dt3000_ai,
		dachan:		0,
		dabits:		0,
	},
	{	name:		"dt3003",
		device_id:	0x24,
		adchan:		64,
		adbits:		12,
		adrange:	&range_dt3000_ai,
		dachan:		2,
		dabits:		12,
	},
	{	name:		"dt3003-pgl",
		device_id:	0x28,
		adchan:		64,
		adbits:		12,
		adrange:	&range_dt3000_ai_pgl,
		dachan:		2,
		dabits:		12,
	},
	{	name:		"dt3004",
		device_id:	0x25,
		adchan:		16,
		adbits:		16,
		adrange:	&range_dt3000_ai,
		dachan:		2,
		dabits:		12,
	},
	{	name:		"dt3005",	/* a.k.a. 3004-200 */
		device_id:	0x26,
		adchan:		16,
		adbits:		16,
		adrange:	&range_dt3000_ai,
		dachan:		2,
		dabits:		12,
	},
};
#define n_dt3k_boards sizeof(dt3k_boardtypes)/sizeof(dt3k_boardtype)
#define this_board ((dt3k_boardtype *)dev->board_ptr)

static struct pci_device_id dt3k_pci_table[] __devinitdata = {
	{ PCI_VENDOR_ID_DT, 0x0022, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_DT, 0x0027, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_DT, 0x0023, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_DT, 0x0024, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_DT, 0x0028, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_DT, 0x0025, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ PCI_VENDOR_ID_DT, 0x0026, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, dt3k_pci_table);

#define DT3000_SIZE		(4*0x1000)

/* dual-ported RAM location definitions */

#define DPR_DAC_buffer		(4*0x000)
#define DPR_ADC_buffer		(4*0x800)
#define DPR_Command		(4*0xfd3)
#define DPR_SubSys		(4*0xfd3)
#define DPR_Encode		(4*0xfd4)
#define DPR_Params(a)		(4*(0xfd5+(a)))
#define DPR_Tick_Reg_Lo		(4*0xff5)
#define DPR_Tick_Reg_Hi		(4*0xff6)
#define DPR_DA_Buf_Front	(4*0xff7)
#define DPR_DA_Buf_Rear		(4*0xff8)
#define DPR_AD_Buf_Front	(4*0xff9)
#define DPR_AD_Buf_Rear		(4*0xffa)
#define DPR_Int_Mask		(4*0xffb)
#define DPR_Intr_Flag		(4*0xffc)
#define DPR_Response_Mbx	(4*0xffe)
#define DPR_Command_Mbx		(4*0xfff)

/* command list */

#define CMD_GETBRDINFO		0
#define CMD_CONFIG		1
#define CMD_GETCONFIG		2
#define CMD_START		3
#define CMD_STOP		4
#define CMD_READSINGLE		5
#define CMD_WRITESINGLE		6
#define CMD_CALCCLOCK		7
#define CMD_READEVENTS		8
#define CMD_WRITECTCTRL		16
#define CMD_READCTCTRL		17
#define CMD_WRITECT		18
#define CMD_READCT		19
#define CMD_WRITEDATA		32
#define CMD_READDATA		33
#define CMD_WRITEIO		34
#define CMD_READIO		35
#define CMD_WRITECODE		36
#define CMD_READCODE		37
#define CMD_EXECUTE		38
#define CMD_HALT		48

#define SUBS_AI		0
#define SUBS_AO		1
#define SUBS_DIN	2
#define SUBS_DOUT	3
#define SUBS_MEM	4
#define SUBS_CT		5

/* interrupt flags */
#define DT3000_CMDONE		0x80
#define DT3000_CTDONE		0x40
#define DT3000_DAHWERR		0x20
#define DT3000_DASWERR		0x10
#define DT3000_DAEMPTY		0x08
#define DT3000_ADHWERR		0x04
#define DT3000_ADSWERR		0x02
#define DT3000_ADFULL		0x01

#define DT3000_COMPLETION_MASK	0xff00
#define DT3000_COMMAND_MASK	0x00ff
#define DT3000_NOTPROCESSED	0x0000
#define DT3000_NOERROR		0x5500
#define DT3000_ERROR		0xaa00
#define DT3000_NOTSUPPORTED	0xff00


#define DT3000_EXTERNAL_CLOCK	1
#define DT3000_RISING_EDGE	2

#define TMODE_MASK		0x1c

#define DT3000_AD_TRIG_INTERNAL		(0<<2)
#define DT3000_AD_TRIG_EXTERNAL		(1<<2)
#define DT3000_AD_RETRIG_INTERNAL	(2<<2)
#define DT3000_AD_RETRIG_EXTERNAL	(3<<2)
#define DT3000_AD_EXTRETRIG		(4<<2)

#define DT3000_CHANNEL_MODE_SE		0
#define DT3000_CHANNEL_MODE_DI		1

typedef struct{
	struct pci_dev *pci_dev;
	unsigned long phys_addr;
	void *io_addr;
	unsigned int lock;
	lsampl_t ao_readback[2];
}dt3k_private;
#define devpriv ((dt3k_private *)dev->private)

static int dt3000_attach(comedi_device *dev,comedi_devconfig *it);
static int dt3000_detach(comedi_device *dev);
comedi_driver driver_dt3000={
	driver_name:	"dt3000",
	module:		THIS_MODULE,
	attach:		dt3000_attach,
	detach:		dt3000_detach,
};
COMEDI_INITCLEANUP(driver_dt3000);


#define TIMEOUT 100

static int dt3k_send_cmd(comedi_device *dev,unsigned int cmd)
{
	int i;
	unsigned int status;
	
	/* XXX my gcc has a bug that causes a warning if the following
	 * is not there */
	status = 0;

	writew(cmd,dev->iobase+DPR_Command_Mbx);
	
	for(i=0;i<TIMEOUT;i++){
		status=readw(dev->iobase+DPR_Command_Mbx);
		if((status&DT3000_COMPLETION_MASK)!=DT3000_NOTPROCESSED)
			break;
		udelay(1);
	}
	if((status&DT3000_COMPLETION_MASK)==DT3000_NOERROR){
		return 0;
	}

	printk("dt3k_send_cmd() timeout/error status=0x%04x\n",status);
	
	return -ETIME;
}

static unsigned int dt3k_readsingle(comedi_device *dev,unsigned int subsys,
	unsigned int chan,unsigned int gain)
{
	writew(subsys,dev->iobase+DPR_SubSys);
	
	writew(chan,dev->iobase+DPR_Params(0));
	writew(gain,dev->iobase+DPR_Params(1));
	
	dt3k_send_cmd(dev,CMD_READSINGLE);

	return readw(dev->iobase+DPR_Params(2));
}

static void dt3k_writesingle(comedi_device *dev,unsigned int subsys,
	unsigned int chan,unsigned int data)
{
	writew(subsys,dev->iobase+DPR_SubSys);
	
	writew(chan,dev->iobase+DPR_Params(0));
	writew(0,dev->iobase+DPR_Params(1));
	writew(data,dev->iobase+DPR_Params(2));
	
	dt3k_send_cmd(dev,CMD_WRITESINGLE);
}



#if 0
static int dt3k_ai_config(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int i;
	unsigned int chan,range,aref;
	
	for(i=0;i<it->n_chan;i++){
		chan=CR_CHAN(it->chanlist[i]);
		range=CR_RANGE(it->chanlist[i]);
		
		writew((range<<6)|chan,dev->iobase+DPR_ADC_buffer+i);
	}
	aref=CR_AREF(it->chanlist[0]);
	
	writew(it->n_chan,dev->iobase+DPR_Params(0));
#if 0
	writew(clkprescale,dev->iobase+DPR_Params(1));
	writew(clkdivider,dev->iobase+DPR_Params(2));
	writew(tscanprescale,dev->iobase+DPR_Params(3));
	writew(tscandiv,dev->iobase+DPR_Params(4));
	writew(triggerclockmode,dev->iobase+DPR_Params(5));
#endif
	writew(aref==AREF_DIFF,dev->iobase+DPR_Params(6));
	writew(0,dev->iobase+DPR_Params(7));
	
	return dt3k_send_cmd(dev,CMD_CONFIG);
}
#endif
	
static int dt3k_ai_insn(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int i;
	unsigned int chan,gain,aref;

	chan=CR_CHAN(insn->chanspec);
	gain=CR_RANGE(insn->chanspec);
	/* XXX docs don't explain how to select aref */
	aref=CR_AREF(insn->chanspec);

	for(i=0;i<insn->n;i++){
		data[i]=dt3k_readsingle(dev,SUBS_AI,chan,gain);
	}

	return i;
}

static int dt3k_ao_insn(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int i;
	unsigned int chan;

	chan=CR_CHAN(insn->chanspec);
	for(i=0;i<insn->n;i++){
		dt3k_writesingle(dev,SUBS_AO,chan,data[i]);
		devpriv->ao_readback[chan]=data[i];
	}

	return i;
}

static int dt3k_ao_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int i;
	unsigned int chan;

	chan=CR_CHAN(insn->chanspec);
	for(i=0;i<insn->n;i++){
		data[i]=devpriv->ao_readback[chan];
	}

	return i;
}

static void dt3k_dio_config(comedi_device *dev,int bits)
{
	/* XXX */
	writew(SUBS_DOUT,dev->iobase+DPR_SubSys);
	
	writew(bits,dev->iobase+DPR_Params(0));
#if 0
	/* don't know */
	writew(0,dev->iobase+DPR_Params(1));
	writew(0,dev->iobase+DPR_Params(2));
#endif
	
	dt3k_send_cmd(dev,CMD_CONFIG);
}

static int dt3k_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int mask;

	if(insn->n!=1)return -EINVAL;

	mask=(CR_CHAN(insn->chanspec)<4)?0x0f:0xf0;
	if(data[0]==COMEDI_OUTPUT)s->io_bits|=mask;
	else s->io_bits&=~mask;

	mask=(s->io_bits&0x01)|((s->io_bits&0x10)>>3);
	dt3k_dio_config(dev,mask);

	return 1;
}

static int dt3k_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=2)return -EINVAL;

	if(data[0]){
		s->state &= ~data[0];
		s->state |= data[1]&data[0];
		dt3k_writesingle(dev,SUBS_DOUT,0,s->state);
	}
	data[1]=dt3k_readsingle(dev,SUBS_DIN,0,0);

	return 2;
}

static int dt3k_mem_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	unsigned int addr=CR_CHAN(insn->chanspec);
	int i;

	for(i=0;i<insn->n;i++){
		writew(SUBS_MEM,dev->iobase+DPR_SubSys);
		writew(addr,dev->iobase+DPR_Params(0));
		writew(1,dev->iobase+DPR_Params(1));

		dt3k_send_cmd(dev,CMD_READCODE);

		data[i]=readw(dev->iobase+DPR_Params(2));
	}
	
	return i;
}

static int dt_pci_probe(comedi_device *dev);

static int dt3000_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret=0;
	
	printk("dt3000:");
#ifdef PCI_SUPPORT_VER1
	if(!pcibios_present()){
#else
	if(!pci_present()){
#endif
		printk(" no PCI bus\n");
		return -EINVAL;
	}
	
	if((ret=alloc_private(dev,sizeof(dt3k_private)))<0)
		return ret;

	ret=dt_pci_probe(dev);
	if(ret<0)return ret;
	if(ret==0){
		printk(" no DT board found\n");
		return -ENODEV;
	}

	dev->board_name=this_board->name;

	dev->n_subdevices=4;
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	s=dev->subdevices;

	/* ai subdevice */
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=this_board->adchan;
	s->insn_read=dt3k_ai_insn;
	s->maxdata=(1<<this_board->adbits)-1;
	s->len_chanlist=512;
	s->range_table=&range_dt3000_ai; /* XXX */

	s++;
	/* ao subsystem */
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=2;
	s->insn_read=dt3k_ao_insn_read;
	s->insn_write=dt3k_ao_insn;
	s->maxdata=(1<<this_board->dabits)-1;
	s->len_chanlist=1;
	s->range_table=&range_bipolar10;

	s++;
	/* dio subsystem */
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_READABLE|SDF_WRITEABLE;
	s->n_chan=8;
	s->insn_config=dt3k_dio_insn_config;
	s->insn_bits=dt3k_dio_insn_bits;
	s->maxdata=1;
	s->len_chanlist=8;
	s->range_table=&range_digital;

	s++;
	/* mem subsystem */
	s->type=COMEDI_SUBD_MEMORY;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=0x1000;
	s->insn_read=dt3k_mem_insn_read;
	s->maxdata=0xff;
	s->len_chanlist=1;
	s->range_table=&range_unknown;

#if 0
	s++;
	/* proc subsystem */
	s->type=COMEDI_SUBD_PROC;
#endif

	return 0;
}

static int dt3000_detach(comedi_device *dev)
{
	if(dev->irq)free_irq(dev->irq,dev);

	/* XXX */

	return 0;
}


#ifdef PCI_SUPPORT_VER1
static int dt_pci_find_device(comedi_device *dev);
static int setup_pci(comedi_device *dev);

static int dt_pci_probe(comedi_device *dev)
{
	int board;
	int ret;

	ret=dt_pci_find_device(NULL,&board);
	if(ret==0)
		return 0;

	setup_pci(dev);

	return 1;
}

static int dt_pci_find_device(comedi_device *dev)
{
	int i;
	unsigned char	pci_bus;
	unsigned char	pci_dev_fn;
	
	for(i=0;i<n_dt3k_boards;i++){
		if(pcibios_find_device(PCI_VENDOR_ID_DT,
			dt3k_boardtypes[i].device_id,
			0,
			&pci_bus,
			&pci_dev_fn) == PCIBIOS_SUCCESSFUL)
		{
			devpriv->pci_bus=pci_bus;
			devpriv->pci_dev_fn=pci_dev_fn;
			dev->board_ptr=dt3k_boards+i;
			return 1;
		}
	}
	return 0;
}

static int setup_pci(comedi_device *dev)
{
	unsigned long		offset;
	u32			addr;

	pcibios_read_config_dword(devpriv->pci_bus,devpriv->pci_device_fn,
			PCI_BASE_ADDRESS_0,&addr);
	devpriv->phys_addr=addr;
	offset=devpriv->phys_addr & ~PAGE_MASK;
	devpriv->io_addr=ioremap(devpriv->phys_addr&PAGE_MASK,DT3000_SIZE+offset)
		+offset;
#if DEBUG
	printk("0x%08lx mapped to %p, ",devpriv->phys_addr,devpriv->io_addr);
#endif

	dev->iobase = (int)devpriv->io_addr;

	return 0;
}

#else

static struct pci_dev *dt_pci_find_device(struct pci_dev *from,int *board);
static int setup_pci(comedi_device *dev);

static int dt_pci_probe(comedi_device *dev)
{
	int board;

	devpriv->pci_dev=dt_pci_find_device(NULL,&board);
	dev->board_ptr=dt3k_boardtypes+board;

	if(!devpriv->pci_dev)
		return 0;

	setup_pci(dev);

	return 1;
}

static int setup_pci(comedi_device *dev)
{
	unsigned long		offset;
	u32			addr;

#if LINUX_VERSION_CODE < 0x020300
	addr=devpriv->pci_dev->base_address[0];
#else
	addr=devpriv->pci_dev->resource[0].start;
#endif
	devpriv->phys_addr=addr;
	offset = devpriv->phys_addr & ~PAGE_MASK;
	devpriv->io_addr = ioremap(devpriv->phys_addr & PAGE_MASK, DT3000_SIZE + offset )
		+ offset;
#if DEBUG
	printk("0x%08lx mapped to %p, ",devpriv->phys_addr,devpriv->io_addr);
#endif

	dev->iobase = (int)devpriv->io_addr;

	return 0;
}

#if LINUX_VERSION_CODE < 0x020300
static struct pci_dev *dt_pci_find_device(struct pci_dev *from,int *board)
{
	int i;
	
	if(!from){
		from=pci_devices;
	}else{
		from=from->next;
	}
	while(from){
		if(from->vendor == PCI_VENDOR_ID_DT){
			for(i=0;i<n_dt3k_boards;i++){
				if(from->device == dt3k_boardtypes[i].device_id){
					*board=i;
					return from;
				}
			}
			printk("unknown Data Translation PCI device found with device_id=0x%04x\n",from->device);
		}
		from=from->next;
	}
	*board=-1;
	return from;
}

#else

static struct pci_dev *dt_pci_find_device(struct pci_dev *from,int *board)
{
	int i;
	
	if(!from){
		from=(struct pci_dev *)(pci_devices.next);
	}else{
		from=(struct pci_dev *)(from->global_list.next);
	}
	while(from){
		if(from->vendor == PCI_VENDOR_ID_DT){
			for(i=0;i<n_dt3k_boards;i++){
				if(from->device == dt3k_boardtypes[i].device_id){
					*board=i;
					return from;
				}
			}
			printk("unknown Data Translation PCI device found with device_id=0x%04x\n",from->device);
		}
		from=(struct pci_dev *)(from->global_list.next);
	}
	*board=-1;
	return from;
}

#endif
#endif /* PCI_SUPPORT_VER1 */

