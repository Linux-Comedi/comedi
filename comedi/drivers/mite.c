/*
    module/mite.c
    Hardware driver for NI Mite PCI interface chip

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2002 David A. Schleef <ds@schleef.org>

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

//#define USE_KMALLOC

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
#include <linux/slab.h>

#include "mite.h"
#include "../kvmem.h"


#define PCI_MITE_SIZE		4096
#define PCI_DAQ_SIZE		4096


struct mite_struct *mite_devices = NULL;

#define TOP_OF_PAGE(x) ((x)|(~(PAGE_MASK)))


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


int mite_setup(struct mite_struct *mite)
{
	unsigned long			offset, start, length;
	u32				addr;

	if(pci_enable_device(mite->pcidev)){
		printk("error enabling mite\n");
		return -EIO;
	}
	pci_set_master(mite->pcidev);

	addr = pci_resource_start(mite->pcidev, 0);
	mite->mite_phys_addr = addr;
	offset = mite->mite_phys_addr & ~PAGE_MASK;
	start = mite->mite_phys_addr & PAGE_MASK;
	length = PCI_MITE_SIZE + offset;
	mite->mite_io_addr = ioremap(start, length) + offset;
	printk("MITE:0x%08lx mapped to %p ",mite->mite_phys_addr,mite->mite_io_addr);

	addr=pci_resource_start(mite->pcidev, 1);
	mite->daq_phys_addr=addr;
	offset = mite->daq_phys_addr & ~PAGE_MASK;
	start = mite->daq_phys_addr & PAGE_MASK;
	length = PCI_DAQ_SIZE + offset;
	mite->daq_io_addr = ioremap(start, length) + offset;
	printk("DAQ:0x%08lx mapped to %p\n",mite->daq_phys_addr,mite->daq_io_addr);

	/* XXX don't know what the 0xc0 and 0x80 mean */
	/* It must be here for the driver to work though */
	writel(mite->daq_phys_addr | 0x80 , mite->mite_io_addr + 0xc0 );

	writel(CHOR_DMARESET, mite->mite_io_addr + MITE_CHOR + CHAN_OFFSET(0));
	writel(CHOR_DMARESET, mite->mite_io_addr + MITE_CHOR + CHAN_OFFSET(1));

	/* disable interrupts */
	writel(0,mite->mite_io_addr+MITE_CHCR+CHAN_OFFSET(0));
	writel(0,mite->mite_io_addr+MITE_CHCR+CHAN_OFFSET(1));

	mite->used = 1;

	return 0;
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
	unsigned long offset, start, length;

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

#if 0
int mite_kvmem_segment_load(struct mite_struct *mite,int i,char *kvmem,unsigned int len)
{
	int count,offset;

	offset=((int)kvmem)&(PAGE_SIZE-1);

	mite->ring[i].addr = cpu_to_le32(kvirt_to_bus((int)kvmem));

	count=PAGE_SIZE-offset;
	if(count>len)count=len;
	mite->ring[i].count = cpu_to_le32(count);

	return count;
}


/*
 * Create the short linkchaining MITE linked list using kernel memory
 */
unsigned long mite_ll_from_kvmem(struct mite_struct *mite,comedi_async *async,int reqlen)
{
	int i,size_so_far, continuous_aq, len;
	unsigned long nup;
	unsigned long prealloc_buf,prealloc_bufsz; 
		
	MDPRINTK("mite_ll_from_kvmem\n");
	prealloc_buf=(unsigned long)async->prealloc_buf;
	prealloc_bufsz=async->prealloc_bufsz;
	
	continuous_aq = (async->cmd.stop_src == TRIG_NONE? 1:0);
	if(continuous_aq) {
		len = prealloc_bufsz;
	}else{
		len = (reqlen>prealloc_bufsz?prealloc_bufsz:reqlen);
	}
	
	//find the kernel's memory pages.
	nup = (unsigned long)async->data;
	i=0;
	size_so_far=0;
	MDPRINTK("buf=0x%08lx bufsz=0x%08lx\n",
		(unsigned long)prealloc_buf,prealloc_bufsz);
	
	while(((void*)nup < (async->data+len))&&(i<(MITE_RING_SIZE-1))) {
		int count;
		count = 1+TOP_OF_PAGE(nup)-nup;
		if(count>len-size_so_far) count = len-size_so_far;
		// it's already a kernel address :-)
		mite->ring[i].addr = cpu_to_le32(kvirt_to_bus(nup));
		mite->ring[i].count = cpu_to_le32(count);
		mite->ring[i].next = cpu_to_le32(virt_to_bus(mite->ring+i+1));
		size_so_far += count;
		nup += count;
		i++;
	}
	
	/*End the mite->ring by setting the last element's count to 0.
	To make a looping ring for continuous acquisition, 
	mite->ring[i-1].next = cpu_to_le32(virt_to_bus(mite->ring));
	*/
	//mite->ring[i].count=0;
	
	if (continuous_aq&&(i>0)) {
		mite->ring[i-1].next = cpu_to_le32(virt_to_bus(mite->ring+0));
		mite->DMA_CheckNearEnd = 0;
	}else if (prealloc_bufsz < reqlen) {
		mite->ring[i-1].next = cpu_to_le32(virt_to_bus(mite->ring+0));
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
#ifdef DEBUG_MITE
	for(i=0; i<MITE_RING_SIZE;i++){
		printk("i=%3d, addr=0x%08x, next=0x%08x, count=0x%08x\n",i,mite->ring[i].addr,
			mite->ring[i].next, mite->ring[i].count);
	}
#endif
	MDPRINTK("exit mite_ll_from_kvmem\n");
	return virt_to_bus(&(mite->ring[0]));
}
#endif


void mite_dma_arm(struct mite_struct *mite)
{
	int chor;

	/* arm */
	chor = CHOR_START;
	writel(chor,mite->mite_io_addr+CHAN_OFFSET(mite->chan)+MITE_CHOR);

	mite_dma_tcr(mite);
}


#if 0
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
#endif


/**************************************/

int mite_load_buffer(struct mite_struct *mite, comedi_async *async)
{
	unsigned int n_links;
	int i;

	MDPRINTK("mite_load_buffer\n");

	if(mite->ring){
		kfree(mite->ring);
	}

	if(async->prealloc_bufsz==0){
		return 0;
	}

	n_links = async->prealloc_bufsz >> PAGE_SHIFT;

	MDPRINTK("buf=%p bufsz=0x%08x n_links=0x%04x\n",
		async->prealloc_buf, async->prealloc_bufsz, n_links);

	mite->ring = kmalloc(n_links*sizeof(struct mite_dma_chain),GFP_KERNEL);
	if(!mite->ring){
		printk("mite: ring buffer allocation failed\n");
		return -ENOMEM;
	}
	mite->n_links = n_links;
	mite->current_link = 0;

	for(i=0;i<n_links;i++){
		mite->ring[i].count = cpu_to_le32(PAGE_SIZE);
#ifdef USE_KMALLOC
		mite->ring[i].addr = cpu_to_le32(
			virt_to_bus(async->prealloc_buf + i*PAGE_SIZE));
#else
		mite->ring[i].addr = cpu_to_le32(
			kvirt_to_bus((unsigned long)async->prealloc_buf + i*PAGE_SIZE));
#endif
		mite->ring[i].next = cpu_to_le32(virt_to_bus(mite->ring+i+1));
	}

	mite->ring[n_links-1].next = cpu_to_le32(virt_to_bus(mite->ring));

	return 0;
}

int mite_buf_alloc(struct mite_struct *mite, comedi_async *async,
	unsigned long new_size)
{
	MDPRINTK("mite_buf_alloc\n");

	if(async->prealloc_buf && async->prealloc_bufsz == new_size){
		return 0;
	}

	if(async->prealloc_bufsz){
#ifdef USE_KMALLOC
		kfree(async->prealloc_buf);
#else
		vfree(async->prealloc_buf);
#endif
		async->prealloc_buf = NULL;
		async->prealloc_bufsz = 0;
	}

	if(new_size){
#ifdef USE_KMALLOC
		async->prealloc_buf = kmalloc(new_size, GFP_KERNEL);
#else
		async->prealloc_buf = vmalloc_32(new_size);
#endif
		if(async->prealloc_buf == NULL){
			async->prealloc_bufsz = 0;
			return -ENOMEM;
		}
	}
	async->prealloc_bufsz = new_size;

	mite_load_buffer(mite,async);

	return 0;
}

void mite_prep_dma(struct mite_struct *mite)
{
	unsigned int chor,chcr,mcr,dcr,lkcr;
	int chan = mite->chan;
	
	MDPRINTK("mite_prep_dma\n");
	
	/* reset DMA and FIFO */
	chor = CHOR_DMARESET | CHOR_FRESET;
	writel(chor,mite->mite_io_addr+MITE_CHOR+CHAN_OFFSET(chan));
	
	/* short link chaining mode */
	chcr = CHCR_SET_DMA_IE| CHCR_LINKSHORT | CHCR_SET_DONE_IE;
	/*
	 * Link Complete Interrupt: interrupt every time a link
	 * in MITE_RING is completed. This can generate a lot of
	 * extra interrupts, but right now we update the values
	 * of buf_int_ptr and buf_int_count at each interrupt.  A
	 * better method is to poll the MITE before each user
	 * "read()" to calculate the number of bytes available.
	 * mite_bytes_transferred() is provided to get the number
	 * of bytes transferred to system memory so far.
	 */
	chcr |= CHCR_SET_LC_IE; 
	
	if(mite->dir == COMEDI_INPUT){
		chcr |= CHCR_DEV_TO_MEM;
	}
	writel(chcr,mite->mite_io_addr+MITE_CHCR+CHAN_OFFSET(chan));
	
	/* 16 bits, to memory */
	mcr = CR_RL64 | CR_ASEQxP1 | CR_PSIZEHALF; 
	writel(mcr,mite->mite_io_addr+MITE_MCR+CHAN_OFFSET(chan));

	/* 16 bits, from device */
	dcr = CR_RL64 |  CR_ASEQx(1) | CR_PSIZEHALF;
	dcr |= CR_PORTIO | CR_AMDEVICE | CR_REQS(0x4+chan);
	writel(dcr,mite->mite_io_addr+MITE_DCR+CHAN_OFFSET(chan));
	
	/* reset the DAR */
	writel(0,mite->mite_io_addr+MITE_DAR+CHAN_OFFSET(chan));
	
	/* the link is 32bits */
	lkcr = CR_RL64 | CR_ASEQUP | CR_PSIZEWORD;
	writel(lkcr,mite->mite_io_addr+MITE_LKCR+CHAN_OFFSET(chan));

	/* starting address for link chaining */
	writel(virt_to_bus(mite->ring),
		mite->mite_io_addr+MITE_LKAR+CHAN_OFFSET(chan));
	
	MDPRINTK("exit mite_prep_dma\n");
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

	lkar=readl(mite->mite_io_addr+CHAN_OFFSET(mite->chan)+MITE_LKAR);
	tcr=readl(mite->mite_io_addr+CHAN_OFFSET(mite->chan)+MITE_TCR);
	MDPRINTK("lkar=0x%08x tcr=%d\n",lkar,tcr);

	return tcr;
}

void mite_dma_disarm(struct mite_struct *mite)
{
	int chor;

	/* disarm */
	chor = CHOR_ABORT;
	writel(chor,mite->mite_io_addr+CHAN_OFFSET(mite->chan)+MITE_CHOR);
}

#ifdef DEBUG_MITE
void mite_dump_regs(struct mite_struct *mite)
{
	unsigned long mite_io_addr = (unsigned long) mite->mite_io_addr;
	unsigned long addr=0;
	unsigned long temp=0;

	printk("mite address is  =0x%08lx\n",mite_io_addr);
		
	addr = mite_io_addr+MITE_CHOR+CHAN_OFFSET(mite->chan);
	printk("mite status[CHOR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_CHOR_strings,temp);
	addr = mite_io_addr+MITE_CHCR+CHAN_OFFSET(mite->chan);
	printk("mite status[CHCR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_CHCR_strings,temp);
	addr = mite_io_addr+MITE_TCR+CHAN_OFFSET(mite->chan);
	printk("mite status[TCR] at 0x%08lx =0x%08x\n",addr, readl(addr));
	addr = mite_io_addr+MITE_MCR+CHAN_OFFSET(mite->chan);
	printk("mite status[MCR] at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_MCR_strings,temp);
	
	addr = mite_io_addr+MITE_MAR+CHAN_OFFSET(mite->chan);
	printk("mite status[MAR] at 0x%08lx =0x%08x\n",addr, readl(addr));
	addr = mite_io_addr+MITE_DCR+CHAN_OFFSET(mite->chan);
	printk("mite status[DCR] at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_CR_strings,temp);
	addr = mite_io_addr+MITE_DAR+CHAN_OFFSET(mite->chan);
	printk("mite status[DAR] at 0x%08lx =0x%08x\n",addr, readl(addr));
	addr = mite_io_addr+MITE_LKCR+CHAN_OFFSET(mite->chan);
	printk("mite status[LKCR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	//mite_decode(mite_CR_strings,temp);
	addr = mite_io_addr+MITE_LKAR+CHAN_OFFSET(mite->chan);
	printk("mite status[LKAR]at 0x%08lx =0x%08x\n",addr, readl(addr));

	addr = mite_io_addr+MITE_CHSR+CHAN_OFFSET(mite->chan);
	printk("mite status[CHSR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	mite_print_chsr(temp);
	//mite_decode(mite_CHSR_strings,temp);
	addr = mite_io_addr+MITE_FCR+CHAN_OFFSET(mite->chan);
	printk("mite status[FCR] at 0x%08lx =0x%08x\n\n",addr, readl(addr));
}


static char *chsr_strings[] = {
	"d.err0", "d.err1", "m.err0", "m.err1",
	"l.err0", "l.err1", "drq0", "drq1",
	"end", "xferr", "operr0", "operr1",
	"stops", "habort", "sabort", "error",
	"16", "conts_rb", "18", "linkc",
	"20", "drdy", "22", "mrdy",
	"24", "done", "26", "sars",
	"28", "lpauses", "30", "int",
};
void mite_print_chsr(unsigned int bits)
{
	int i;

	printk("chsr:");
	for(i=31;i>=0;i--){
		if(bits&(1<<i)){
			printk(" %s",chsr_strings[i]);
		}
	}
	printk("\n");
}
#endif


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

EXPORT_SYMBOL(mite_dma_tcr);
EXPORT_SYMBOL(mite_dma_arm);
EXPORT_SYMBOL(mite_dma_disarm);
EXPORT_SYMBOL(mite_setup);
EXPORT_SYMBOL(mite_unsetup);
#if 0
EXPORT_SYMBOL(mite_kvmem_segment_load);
EXPORT_SYMBOL(mite_ll_from_kvmem);
EXPORT_SYMBOL(mite_setregs);
#endif
EXPORT_SYMBOL(mite_devices);
EXPORT_SYMBOL(mite_list_devices);
EXPORT_SYMBOL(mite_prep_dma);
EXPORT_SYMBOL(mite_buf_alloc);
EXPORT_SYMBOL(mite_bytes_transferred);
#ifdef DEBUG_MITE
EXPORT_SYMBOL(mite_print_chsr);
EXPORT_SYMBOL(mite_dump_regs);
#endif

#endif

