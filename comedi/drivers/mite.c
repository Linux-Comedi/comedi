/*
    module/mite.c
    Hardware driver for NI Mite PCI interface chip

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

/*
	The PCI-MIO E series driver was originally written by
	Tomasz Motylewski <...>, and ported to comedi by ds.


	References for specifications:
	
	   321747b.pdf  Register Level Programmer Manual (obsolete)
	   321747c.pdf  Register Level Programmer Manual (new)
	   DAQ-STC reference manual

	Other possibly relevant info:
	
	   320517c.pdf  User manual (obsolete)
	   320517f.pdf  User manual (new)
	   320889a.pdf  delete
	   320906c.pdf  maximum signal ratings
	   321066a.pdf  about 16x
	   321791a.pdf  discontinuation of at-mio-16e-10 rev. c
	   321808a.pdf  about at-mio-16e-10 rev P
	   321837a.pdf  discontinuation of at-mio-16de-10 rev d
	   321838a.pdf  about at-mio-16de-10 rev N
	
	ISSUES:

*/

#include <linux/comedidev.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <linux/malloc.h>
#include <mite.h>
#include <kvmem.h>


#define PCI_MITE_SIZE		4096
#define PCI_DAQ_SIZE		4096


struct mite_struct *mite_devices = NULL;
	
#define TOP_OF_PAGE(x) ((x)|(~(PAGE_MASK)))
#define min(a,b) (((a)<(b))?(a):(b))

#ifdef PCI_SUPPORT_VER1
/* routines for the old PCI code (before 2.1.55) */

void mite_init(void)
{
	struct mite_struct *mite;
	int pci_index;
	unsigned char pci_bus, pci_device_fn;
	u16 vendor;
	u16 device_id;

	for(pci_index=0;pci_index<0xff;pci_index++){
		if(pcibios_find_class(PCI_CLASS_OTHERS << 8,
			pci_index,&pci_bus,&pci_device_fn)!=PCIBIOS_SUCCESSFUL)
				break;

		pcibios_read_config_word(pci_bus,pci_device_fn,PCI_VENDOR_ID,&vendor);
		if(vendor==PCI_VENDOR_ID_NATINST){
			mite=kmalloc(sizeof(*mite),GFP_KERNEL);
			if(!mite){
				printk("mite: allocation failed\n");
				return;
			}
			memset(mite,0,sizeof(*mite));

			mite->pci_bus=pci_bus;
			mite->pci_device_fn=pci_device_fn;

			pcibios_read_config_word(pci_bus,pci_device_fn,PCI_DEVICE_ID,&device_id);
			mite->device_id=device_id;

			mite->next=mite_devices;
			mite_devices=mite;
		}
	}
}

#else

/* functions for the new PCI code (after 2.1.55) */

void mite_init(void)
{
	struct pci_dev *pcidev;
	struct mite_struct *mite;

	pci_for_each_dev(pcidev){
		if(pcidev->vendor==PCI_VENDOR_ID_NATINST){
			mite=kmalloc(sizeof(*mite),GFP_KERNEL);
			if(!mite){
				printk("mite: allocation failed\n");
				return;
			}
			memset(mite,0,sizeof(*mite));

			mite->pcidev=pcidev;

			mite->next=mite_devices;
			mite_devices=mite;
		}
	}
}

#endif

int mite_setup(struct mite_struct *mite)
{
	unsigned long			offset;
	u32				addr;
	int i;

#ifdef PCI_SUPPORT_VER1
	pcibios_read_config_dword(mite->pci_bus,mite->pci_device_fn,PCI_BASE_ADDRESS_0,&addr);
#else

#if LINUX_VERSION_CODE < 0x020300
	addr=mite->pcidev->base_address[0];
#else
	pci_enable_device(mite->pcidev);
	pci_set_master(mite->pcidev);
	addr=mite->pcidev->resource[0].start;
#endif
#endif
	mite->mite_phys_addr=addr;
	offset = mite->mite_phys_addr & ~PAGE_MASK;
	mite->mite_io_addr = ioremap(mite->mite_phys_addr & PAGE_MASK, PCI_MITE_SIZE + offset ) + offset;
	printk("MITE:0x%08lx mapped to %p ",mite->mite_phys_addr,mite->mite_io_addr);
	
#ifdef PCI_SUPPORT_VER1
	pcibios_read_config_dword(mite->pci_bus,mite->pci_device_fn,PCI_BASE_ADDRESS_1,&addr);
#else
#if LINUX_VERSION_CODE < 0x020300
	addr=mite->pcidev->base_address[1];
#else
	addr=mite->pcidev->resource[1].start;
#endif
#endif
	mite->daq_phys_addr=addr;
	offset = mite->daq_phys_addr & ~PAGE_MASK;
	mite->daq_io_addr = ioremap(mite->daq_phys_addr & PAGE_MASK, PCI_DAQ_SIZE + offset ) + offset;
	printk("DAQ:0x%08lx mapped to %p, ",mite->daq_phys_addr,mite->daq_io_addr);

	/* XXX don't know what the 0xc0 and 0x80 mean */
	/* It must be here for the driver to work though */
	writel(mite->daq_phys_addr | 0x80 , mite->mite_io_addr + 0xc0 );

#ifdef PCI_SUPPORT_VER1
	{
		unsigned char irq;
		pcibios_read_config_byte(mite->pci_bus,mite->pci_device_fn,PCI_INTERRUPT_LINE,&irq);
		mite->irq=irq;
	}
#endif

	/* DMA setup */
	for(i=0;i<MITE_RING_SIZE-1;i++){
		mite->ring[i].next=virt_to_bus(mite->ring+i+1);

	}
	mite->ring[i].next=0;

	mite->used = 1;

	return (int) mite->daq_io_addr;
}


void mite_cleanup(void)
{
	struct mite_struct *mite,*next;

	for(mite=mite_devices;mite;mite=next){
		next=mite->next;
		kfree(mite);
	}
}

void mite_unsetup(struct mite_struct *mite)
{
	if(!mite)return;

	if(mite->mite_io_addr){
		iounmap(mite->mite_io_addr);
		mite->mite_io_addr=NULL;
	}
	if(mite->daq_io_addr){
		iounmap(mite->daq_io_addr);
		mite->daq_io_addr=NULL;
	}

	mite->used = 0;
}


void mite_list_devices(void)
{
	struct mite_struct *mite,*next;
	
	printk("Available NI device IDs:");
	if(mite_devices)for(mite=mite_devices;mite;mite=next){
		next=mite->next;
		printk(" 0x%04x",mite_device_id(mite));
		if(mite->used)printk("(used)");
	}
	printk("\n");

}

int mite_kvmem_segment_load(struct mite_struct *mite,int i,char *kvmem,unsigned int len)
{
	int count,offset;

	offset=((int)kvmem)&(PAGE_SIZE-1);

	mite->ring[i].addr = kvirt_to_bus((int)kvmem);

	count=PAGE_SIZE-offset;
	if(count>len)count=len;
	mite->ring[i].count = count;

	return count;
}

void mite_dma_prep(struct mite_struct *mite,comedi_subdevice *s)
{
	int i,n;
	int chor,chcr,mcr,dcr,lkcr;

	for(i=0;i<MITE_RING_SIZE;i++){
		n=s->async->prealloc_bufsz-s->async->buf_int_ptr;
		n=mite_kvmem_segment_load(mite,i,s->async->data+s->async->buf_int_ptr,n);
		s->async->buf_int_ptr+=n;
		if(s->async->buf_int_ptr>=s->async->prealloc_bufsz)
			s->async->buf_int_ptr=0;
	}

	chor = CHOR_DMARESET | CHOR_FRESET;
	writel(chor,mite->mite_io_addr+MITE_CHOR+CHAN_OFFSET(0));

	// Should this be LINKLONG for 32HS?
	chcr = CHCR_LINKSHORT | CHCR_DEV_TO_MEM;
	writel(chcr,mite->mite_io_addr+MITE_CHCR+CHAN_OFFSET(0));

	mcr = CR_RL64 | CR_ASEQxP1 | CR_PSIZEHALF;
	writel(mcr,mite->mite_io_addr+MITE_MCR+CHAN_OFFSET(0));

	dcr = CR_RL64 | CR_PSIZEHALF | CR_PORTIO | CR_AMDEVICE;
	dcr |= CR_REQSDRQ0;
	writel(dcr,mite->mite_io_addr+MITE_DCR+CHAN_OFFSET(0));

	lkcr = CR_RL64 | CR_ASEQUP | CR_PSIZEWORD;
	writel(lkcr,mite->mite_io_addr+MITE_LKCR+CHAN_OFFSET(0));

	writel(virt_to_bus(mite->ring),mite->mite_io_addr+MITE_LKAR+CHAN_OFFSET(0));
}



/*
 * Create the short linkchaining MITE linked list using kernel memory
 * a drop in replacement for mite_ll_from_user( ) 
 */
unsigned long mite_ll_from_kvmem(struct mite_struct *mite,comedi_async *async,int reqlen)
{
	int i,size_so_far, continuous_aq, len;
	unsigned long nup;
	unsigned long prealloc_buf,prealloc_bufsz; 
	//comedi_subdevice *s;
	//struct mite_struct *mite=NULL;
	//comedi_async *async=NULL;
	//int len;
		
	MDPRINTK("mite_ll_from_kvmem\n");
	//s=dev->subdevices+cmd->subdev;
	//mite=devpriv->mite;
	prealloc_buf=(unsigned long)async->prealloc_buf;
	prealloc_bufsz=async->prealloc_bufsz;
	
	continuous_aq = (async->cmd.stop_src == TRIG_NONE? 1:0);
	if(continuous_aq) {
		len = prealloc_bufsz;
	}else{
		len = (reqlen>prealloc_bufsz?prealloc_bufsz:reqlen);
	}
	
	//if(async->data_len<len) {
	//	printk("<1>Comedi Error: preallocated DMA buffer is too small to hold the samples.");
	//}
	
	//find the kernel's memory pages.
	nup = (unsigned long)async->data;
	i=0;
	size_so_far=0;
	MDPRINTK("buf=0x%08lx bufsz=0x%08lx\n",
		(unsigned long)prealloc_buf,prealloc_bufsz);
	
	while(((void*)nup < (async->data+len))&&(i<(MITE_RING_SIZE-1))) {
		mite->ring[i].addr =kvirt_to_bus(nup);// it's already a kernel address :-)
		mite->ring[i].count=min(1+TOP_OF_PAGE(nup)-nup,len-size_so_far);
		mite->ring[i].next=virt_to_bus(mite->ring+i+1);
		size_so_far += mite->ring[i].count;
		nup += mite->ring[i].count;
		i++;
	}
	
	/*End the mite->ring by setting the last element's count to 0.
	To make a looping ring for continuous acquisition, 
	mite->ring[i-1].next = virt_to_bus(mite->ring);
	*/
	//mite->ring[i].count=0;
	
	if (continuous_aq&&(i>0)) {
		mite->ring[i-1].next = virt_to_bus(mite->ring+0);
		mite->DMA_CheckNearEnd = 0;
	}else if (prealloc_bufsz < reqlen) {
		mite->ring[i-1].next = virt_to_bus(mite->ring+0);
		mite->DMA_CheckNearEnd = 1;
	}	
	else {
		mite->ring[i].count=0;
		mite->DMA_CheckNearEnd = 0;
	}

	
	MDPRINTK("i was %d, size_so_far was %d\n",i,size_so_far);
	if(size_so_far<len) {
		printk("<1>Comedi Error: MITE_RING_SIZE is too small to hold the needed buffer\n");
	}
	#if 0 //#ifdef DEBUG_MITE
	for(i=0; i<MITE_RING_SIZE;i++){
		printk("i=%3d, addr=0x%08x, next=0x%08x, count=0x%08x\n",i,mite->ring[i].addr,
			mite->ring[i].next, mite->ring[i].count);
	}
	#endif
	MDPRINTK("exit mite_ll_from_kvmem\n");
	return virt_to_bus(&(mite->ring[0]));
}

/* This function would be used to DMA directly into user memory.
Since Comedi won't support that for a while, this should probably be removed. --Tim Ousley
unsigned long mite_ll_from_user(comedi_device *dev, comedi_cmd *cmd)
{
	int i,size_so_far,len;
	unsigned long nup;
		
	comedi_subdevice *s;
	struct mite_struct *mite;
		
	MDPRINTK("mite_ll_from_user\n");
	s=dev->subdevices+cmd->subdev;
	//mite=devpriv->mite;
	len = cmd->data_len;
	
	//find the users memory pages.
	nup = (unsigned long)cmd->data; i=0; size_so_far=0;
	MDPRINTK("buf=0x%08lx bufsz=0x%08x\n",
		(unsigned long)cmd->data,cmd->data_len);
	while(((void*)nup < ((void*)cmd->data+len))&&(i<(MITE_RING_SIZE-1))) {
		mite->ring[i].addr =uvirt_to_bus(nup);
		mite->ring[i].count=min(1+TOP_OF_PAGE(nup)-nup,len-size_so_far);
		mite->ring[i].next=virt_to_bus(mite->ring+i+1);
		size_so_far += mite->ring[i].count;
		nup += mite->ring[i].count;
		i++;
	}
	mite->ring[i].count=0;
	MDPRINTK("i was %d, size_so_far was %d\n",i,size_so_far);
	if(size_so_far<len) {
		printk("<1>Comedi Error: MITE_RING_SIZE is too small to hold the needed buffer\n");
		printk("<1>Comedi Error: MITE_RING_SIZE is %d, buffer is %d bytes\n",
			MITE_RING_SIZE, len);
	}
	
	#if 0 //#ifdef DEBUG_MITE
	for(i=0; i<MITE_RING_SIZE;i++){
		printk("i=%3d, addr=0x%08x, next=0x%08x, count=0x%08x\n",i,mite->ring[i].addr,
			mite->ring[i].next, mite->ring[i].count);
	}
	#endif
	MDPRINTK("exit mite_ll_from_user\n");
	return virt_to_bus(&(mite->ring[0]));
}
*/

void mite_dma_arm(struct mite_struct *mite)
{
	int chor;

	/* arm */
	chor = CHOR_START;
	writel(chor,mite->mite_io_addr+CHAN_OFFSET(0)+MITE_CHOR);

	mite_dma_tcr(mite);
}


void mite_setregs(struct mite_struct *mite,unsigned long ll_start,int chan,int dir)
{
	//*mite is the mite to work with
	//ll_start is the beginning of the linkshort mite linked list
	//chan is the DMA channel to use on the MITE (0,1,2,3)
	//dir is the direction of the transfer COMEDI_INPUT or COMEDI_OUTPUT
	int chor,chcr,mcr,dcr,lkcr;
	
	MDPRINTK("mite_setregs\n");
	
	chor = CHOR_DMARESET | CHOR_FRESET; //reset DMA and FIFO
	writel(chor,mite->mite_io_addr+MITE_CHOR+CHAN_OFFSET(chan));
	
	//short link chaining mode
	chcr = CHCR_SET_DMA_IE| CHCR_LINKSHORT | CHCR_SET_DONE_IE;
	/*Link Complete Interrupt: interrupt every time a link in MITE_RING
	is completed. This can generate a lot of extra interrupts, but right now we
	update the values of buf_int_ptr and buf_int_count at each interrupt.  A 
	better method is to poll the MITE before each user "read()" to calculate the
	number of bytes available.  mite_bytes_transferred() is provided to get the
	number of bytes transferred to system memory so far.
	*/
	chcr |= CHCR_SET_LC_IE; 
	
	if(dir == COMEDI_INPUT){
		chcr |= CHCR_DEV_TO_MEM;
	}
	writel(chcr,mite->mite_io_addr+MITE_CHCR+CHAN_OFFSET(chan));
	
	//16 bits, to memory
	mcr = CR_RL64 | CR_ASEQxP1 | CR_PSIZEHALF; 
	writel(mcr,mite->mite_io_addr+MITE_MCR+CHAN_OFFSET(chan));

	//16 bits, from STC
	dcr = CR_RL64 |  CR_ASEQx(1) | CR_PSIZEHALF;
	dcr |= CR_PORTIO | CR_AMDEVICE | CR_REQS(0x4+chan);
	writel(dcr,mite->mite_io_addr+MITE_DCR+CHAN_OFFSET(chan));
	
	//reset the DAR
	writel(0,mite->mite_io_addr+MITE_DAR+CHAN_OFFSET(chan));
	
	//the link is 32bits
	lkcr = CR_RL64 | CR_ASEQUP | CR_PSIZEWORD;
	writel(lkcr,mite->mite_io_addr+MITE_LKCR+CHAN_OFFSET(chan));

	//starting address for link chaining
	writel(ll_start,mite->mite_io_addr+MITE_LKAR+CHAN_OFFSET(chan));
	
	MDPRINTK("exit mite_setregs\n");
}

int mite_bytes_transferred(struct mite_struct *mite, int chan)
{
	int dar, fcr;
	
	dar = readl(mite->mite_io_addr+MITE_DAR+CHAN_OFFSET(chan));
	fcr = readl(mite->mite_io_addr+MITE_FCR+CHAN_OFFSET(chan)) & 0x000000FF;
	return dar-fcr;
}

int mite_dma_tcr(struct mite_struct *mite)
{
	int tcr;
	int lkar;

	lkar=readl(mite->mite_io_addr+CHAN_OFFSET(0)+MITE_LKAR);
	tcr=readl(mite->mite_io_addr+CHAN_OFFSET(0)+MITE_TCR);
	MDPRINTK("lkar=0x%08x tcr=%d\n",lkar,tcr);

	return tcr;
}

void mite_dma_disarm(struct mite_struct *mite)
{
	int chor;

	/* disarm */
	chor = CHOR_ABORT;
	writel(chor,mite->mite_io_addr+CHAN_OFFSET(0)+MITE_CHOR);
}

void mite_dump_regs(struct mite_struct *mite)
{
	unsigned long mite_io_addr = (unsigned long) mite->mite_io_addr;
	unsigned long addr=0;
	unsigned long temp=0;

	printk("mite address is  =0x%08lx\n",mite_io_addr);
		
	addr = mite_io_addr+MITE_CHOR+CHAN_OFFSET(0);
	printk("mite status[CHOR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_CHOR_strings,temp);
	addr = mite_io_addr+MITE_CHCR+CHAN_OFFSET(0);
	printk("mite status[CHCR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_CHCR_strings,temp);
	addr = mite_io_addr+MITE_TCR+CHAN_OFFSET(0);
	printk("mite status[TCR] at 0x%08lx =0x%08x\n",addr, readl(addr));
	addr = mite_io_addr+MITE_MCR+CHAN_OFFSET(0);
	printk("mite status[MCR] at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_MCR_strings,temp);
	
	addr = mite_io_addr+MITE_MAR+CHAN_OFFSET(0);
	printk("mite status[MAR] at 0x%08lx =0x%08x\n",addr, readl(addr));
	addr = mite_io_addr+MITE_DCR+CHAN_OFFSET(0);
	printk("mite status[DCR] at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_CR_strings,temp);
	addr = mite_io_addr+MITE_DAR+CHAN_OFFSET(0);
	printk("mite status[DAR] at 0x%08lx =0x%08x\n",addr, readl(addr));
	addr = mite_io_addr+MITE_LKCR+CHAN_OFFSET(0);
	printk("mite status[LKCR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_CR_strings,temp);
	addr = mite_io_addr+MITE_LKAR+CHAN_OFFSET(0);
	printk("mite status[LKAR]at 0x%08lx =0x%08x\n",addr, readl(addr));

	addr = mite_io_addr+MITE_CHSR+CHAN_OFFSET(0);
	printk("mite status[CHSR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_CHSR_strings,temp);
	addr = mite_io_addr+MITE_FCR+CHAN_OFFSET(0);
	printk("mite status[FCR] at 0x%08lx =0x%08x\n\n",addr, readl(addr));
}




#ifdef MODULE
int init_module(void)
{
	mite_init();
	mite_list_devices();

	return 0;
}

void cleanup_module(void)
{
	mite_cleanup();
}

#ifdef LINUX_V20

struct symbol_table mite_syms = {
#include <linux/symtab_begin.h>
	X(mite_dma_arm),
	X(mite_dma_disarm),
	X(mite_dma_prep),
	X(mite_setup),
	X(mite_unsetup),
	X(mite_kvmem_segment_load),
	X(mite_devices),
	X(mite_list_devices),
#include <linux/symtab_end.h>
};

#endif

#ifdef LINUX_V22

EXPORT_SYMBOL(mite_dma_tcr);
EXPORT_SYMBOL(mite_dma_arm);
EXPORT_SYMBOL(mite_dma_disarm);
EXPORT_SYMBOL(mite_dma_prep);
EXPORT_SYMBOL(mite_setup);
EXPORT_SYMBOL(mite_unsetup);
EXPORT_SYMBOL(mite_kvmem_segment_load);
EXPORT_SYMBOL(mite_devices);
EXPORT_SYMBOL(mite_list_devices);

//Tim's debugging function
EXPORT_SYMBOL(mite_dump_regs);
//EXPORT_SYMBOL(mite_ll_from_user); //obsolete
EXPORT_SYMBOL(mite_ll_from_kvmem);
EXPORT_SYMBOL(mite_setregs);
EXPORT_SYMBOL(mite_bytes_transferred);

#endif

#endif

