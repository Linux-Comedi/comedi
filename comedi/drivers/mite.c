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

#include <comedi_module.h>
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
#include <ni_stc.h>
#include <kvmem.h>


#define PCI_MITE_SIZE		4096
#define PCI_DAQ_SIZE		4096


struct mite_struct *mite_devices;




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

#if LINUX_VERSION_CODE < 0x020300

/* functions for the new PCI code (after 2.1.55) */

void mite_init(void)
{
	struct pci_dev *pcidev;
	struct mite_struct *mite;

	for(pcidev=pci_devices;pcidev;pcidev=pcidev->next){
		if(pcidev->vendor==PCI_VENDOR_ID_NATINST){
			mite=kmalloc(sizeof(*mite),GFP_KERNEL);
			memset(mite,0,sizeof(*mite));

			mite->pcidev=pcidev;

			mite->next=mite_devices;
			mite_devices=mite;
		}
	}
}

#else

/* And after the pci_devices change */

void mite_init(void)
{
	struct pci_dev *pcidev;
	struct mite_struct *mite;

	pci_for_each_dev(pcidev){
		if(pcidev->vendor==PCI_VENDOR_ID_NATINST){
			mite=kmalloc(sizeof(*mite),GFP_KERNEL);
			memset(mite,0,sizeof(*mite));

			mite->pcidev=pcidev;

			mite->next=mite_devices;
			mite_devices=mite;
		}
	}
}


#endif
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
		mite->ring[i].unused=0x1c; /* eh? */
	}
	mite->ring[i].next=0;
	mite->ring[i].unused=0x1c; /* eh? */

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
		n=s->prealloc_bufsz-s->buf_int_ptr;
		n=mite_kvmem_segment_load(mite,i,((void *)s->cur_trig.data)+s->buf_int_ptr,n);
		s->buf_int_ptr+=n;
		if(s->buf_int_ptr>=s->prealloc_bufsz)
			s->buf_int_ptr=0;
	}

	chor = CHOR_DMARESET | CHOR_FRESET;
	writel(chor,mite->mite_io_addr+MITE_CHOR+CHAN_OFFSET(0));

	chcr = CHCR_LINKLONG | CHCR_DEV_TO_MEM;
	//chcr = CHCR_LINKLONG | CHCR_MEM_TO_DEV;
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

void mite_dma_arm(struct mite_struct *mite)
{
	int chor;

	/* arm */
	chor = CHOR_START;
	writel(chor,mite->mite_io_addr+CHAN_OFFSET(0)+MITE_CHOR);

	mite_dma_tcr(mite);
}

int mite_dma_tcr(struct mite_struct *mite)
{
	int tcr;
	int lkar;

	lkar=readl(mite->mite_io_addr+CHAN_OFFSET(0)+MITE_LKAR);
	tcr=readl(mite->mite_io_addr+CHAN_OFFSET(0)+MITE_TCR);
	printk("lkar=0x%08x tcr=%d\n",lkar,tcr);

	return tcr;
}

void mite_dma_disarm(struct mite_struct *mite)
{
	int chor;

	/* disarm */
	chor = CHOR_ABORT;
	writel(chor,mite->mite_io_addr+CHAN_OFFSET(0)+MITE_CHOR);
}

#ifdef MODULE
int init_module(void)
{
	mite_init();

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

#endif

#endif

