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

#include <linux/pci.h>
#include <asm/system.h>

#include "mite.h"

#define PCI_MITE_SIZE		4096
#define PCI_DAQ_SIZE		4096

MODULE_LICENSE("GPL");

struct mite_struct *mite_devices = NULL;

#define TOP_OF_PAGE(x) ((x)|(~(PAGE_MASK)))


void mite_init(void)
{
	struct pci_dev *pcidev;
	struct mite_struct *mite;

	for(pcidev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, NULL); pcidev != NULL ; 
		pcidev = pci_find_device(PCI_ANY_ID, PCI_ANY_ID, pcidev)) {
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
	int i;

	if(pci_enable_device(mite->pcidev)){
		printk("error enabling mite\n");
		return -EIO;
	}
	pci_set_master(mite->pcidev);
	if( pci_request_regions( mite->pcidev, "mite" ) ) {
		printk("failed to request mite io regions\n");
		return -EIO;
	};

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

	writel(mite->daq_phys_addr | WENAB , mite->mite_io_addr + MITE_IODWBSR);

	for( i = 0; i < NUM_MITE_DMA_CHANNELS; i++ ) {
		writel(CHOR_DMARESET, mite->mite_io_addr + MITE_CHOR(i));
		/* disable interrupts */
		writel(0, mite->mite_io_addr + MITE_CHCR(i));
	}
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
	//unsigned long offset, start, length;

	if(!mite)return;

	if(mite->mite_io_addr){
		iounmap(mite->mite_io_addr);
		mite->mite_io_addr=NULL;
	}
	if(mite->daq_io_addr){
		iounmap(mite->daq_io_addr);
		mite->daq_io_addr=NULL;
	}
	if( mite->used )
		pci_release_regions( mite->pcidev );

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

void mite_dma_arm( struct mite_struct *mite, unsigned int channel )
{
	int chor;

	MDPRINTK("mite_dma_arm ch%i\n", channel);
	/* arm */
	chor = CHOR_START;
	writel(chor, mite->mite_io_addr + MITE_CHOR(channel));
	mite_dma_tcr(mite, channel);
}


/**************************************/

int mite_load_buffer(struct mite_struct *mite, unsigned int channel, comedi_async *async)
{
	unsigned int n_links;
	int i;
	struct mite_channel *mite_chan = &mite->channels[ channel ];

	MDPRINTK("mite_load_buffer ch%i\n", channel);

	if(mite_chan->ring){
		kfree(mite_chan->ring);
		mite_chan->ring = NULL;
	}

	if(async->prealloc_bufsz==0){
		return 0;
	}

	n_links = async->prealloc_bufsz >> PAGE_SHIFT;

	MDPRINTK("buf=%p buf(bus)=%08lx bufsz=0x%08x n_links=0x%04x\n",
		async->prealloc_buf, virt_to_bus(async->prealloc_buf), async->prealloc_bufsz, n_links);

	mite_chan->ring = kmalloc(n_links * sizeof(struct mite_dma_chain), GFP_KERNEL);
	if(!mite_chan->ring){
		printk("mite: ring buffer allocation failed\n");
		return -ENOMEM;
	}
	mite_chan->n_links = n_links;
	mite_chan->current_link = 0;

	for(i=0;i<n_links;i++){
		mite_chan->ring[i].count = cpu_to_le32(PAGE_SIZE);
		mite_chan->ring[i].addr = cpu_to_le32(virt_to_bus(
				(void *)async->buf_page_list[i]));
		mite_chan->ring[i].next = cpu_to_le32(virt_to_bus(
				mite_chan->ring+i+1));
	}

	mite_chan->ring[n_links-1].next = cpu_to_le32(virt_to_bus(
				mite_chan->ring));

	return 0;
}

int mite_buf_change(struct mite_struct *mite, unsigned int channel,
	comedi_async *async, unsigned long new_size)
{
	MDPRINTK("mite_buf_change ch%i\n", channel);

	mite_load_buffer(mite, channel, async);

	return 0;
}

void mite_prep_dma( struct mite_struct *mite, unsigned int channel,
	unsigned int num_device_bits, unsigned int num_memory_bits )
{
	unsigned int chor,chcr,mcr,dcr,lkcr;
	struct mite_channel *mite_chan = &mite->channels[ channel ];

	MDPRINTK("mite_prep_dma ch%i\n", channel );

	/* reset DMA and FIFO */
	chor = CHOR_DMARESET | CHOR_FRESET;
	writel(chor, mite->mite_io_addr + MITE_CHOR(channel));

	/* short link chaining mode */
	chcr = CHCR_SET_DMA_IE| CHCR_LINKSHORT | CHCR_SET_DONE_IE;
	/*
	 * Link Complete Interrupt: interrupt every time a link
	 * in MITE_RING is completed. This can generate a lot of
	 * extra interrupts, but right now we update the values
	 * of buf_int_ptr and buf_int_count at each interrupt.  A
	 * better method is to poll the MITE before each user
	 * "read()" to calculate the number of bytes available.
	 * mite_bytes_transferred(), mite_bytes_read(), and
	 * mite_bytes_in_transit() are provided to get the number
	 * of bytes transferred by the mite so far.
	 */
	chcr |= CHCR_SET_LC_IE;

	if(mite_chan->dir == COMEDI_INPUT){
		chcr |= CHCR_DEV_TO_MEM;
	}
	writel(chcr, mite->mite_io_addr + MITE_CHCR(channel));

	/* to/from memory */
	mcr = CR_RL(64) | CR_ASEQUP;
	switch( num_memory_bits ){
		case 8:
			mcr |= CR_PSIZE8;
			break;
		case 16:
			mcr |= CR_PSIZE16;
			break;
		case 32:
			mcr |= CR_PSIZE32;
			break;
		default:
			rt_printk( "mite: bug! invalid mem bit width for dma transfer\n" );
			break;
	}
	writel(mcr, mite->mite_io_addr + MITE_MCR(channel));

	/* from/to device */
	dcr = CR_RL(64) |  CR_ASEQUP;
	dcr |= CR_PORTIO | CR_AMDEVICE | CR_REQSDRQ(channel);
	switch( num_device_bits ){
		case 8:
			dcr |= CR_PSIZE8;
			break;
		case 16:
			dcr |= CR_PSIZE16;
			break;
		case 32:
			dcr |= CR_PSIZE32;
			break;
		default:
			rt_printk( "mite: bug! invalid dev bit width for dma transfer\n" );
			break;
	}
	writel(dcr, mite->mite_io_addr + MITE_DCR(channel));

	/* reset the DAR */
	writel(0, mite->mite_io_addr + MITE_DAR(channel));

	/* the link is 32bits */
	lkcr = CR_RL(64) | CR_ASEQUP | CR_PSIZE32;
	writel(lkcr, mite->mite_io_addr + MITE_LKCR(channel));

	/* starting address for link chaining */
	writel(virt_to_bus(mite_chan->ring),
		mite->mite_io_addr + MITE_LKAR(channel));

	MDPRINTK("exit mite_prep_dma\n");
}

unsigned int mite_bytes_read(struct mite_struct *mite, unsigned int chan)
{
       return readl(mite->mite_io_addr+MITE_DAR(chan));
}

unsigned int mite_bytes_in_transit(struct mite_struct *mite, unsigned int chan)
{
	return readl(mite->mite_io_addr + MITE_FCR(chan)) & 0x000000FF;
}

unsigned int mite_bytes_transferred(struct mite_struct *mite, unsigned int chan)
{
	unsigned int bytes_read;

	bytes_read = mite_bytes_read( mite, chan );
	/* to avoid race, we want to read bytes read before reading bytes
	 * in transit */
	rmb();
	return bytes_read - mite_bytes_in_transit( mite, chan );
}

int mite_dma_tcr(struct mite_struct *mite, unsigned int channel)
{
	int tcr;
	int lkar;

	lkar = readl(mite->mite_io_addr + MITE_LKAR(channel));
	tcr = readl(mite->mite_io_addr + MITE_TCR(channel));
	MDPRINTK("mite_dma_tcr ch%i, lkar=0x%08x tcr=%d\n", channel, lkar, tcr);

	return tcr;
}

void mite_dma_disarm(struct mite_struct *mite, unsigned int channel)
{
	int chor;

	/* disarm */
	chor = CHOR_ABORT;
	writel(chor, mite->mite_io_addr + MITE_CHOR(channel));
}

#ifdef DEBUG_MITE

static void mite_decode(char **bit_str, unsigned int bits);

/* names of bits in mite registers */

static char *mite_CHOR_strings[] = {
	"start", "cont", "stop", "abort",
	"freset", "clrlc", "clrrb", "clrdone",
	"clr_lpause", "set_lpause", "clr_send_tc",
	"set_send_tc", "12", "13", "14",
	"15", "16", "17", "18",
	"19", "20", "21", "22",
	"23", "24", "25", "26",
	"27", "28", "29", "30",
	"dmareset",
};

static char *mite_CHCR_strings[] = {
	"continue", "ringbuff", "2", "3",
	"4", "5", "6", "7",
	"8", "9", "10", "11",
	"12", "13", "bursten", "fifodis",
	"clr_cont_rb_ie", "set_cont_rb_ie", "clr_lc_ie", "set_lc_ie",
	"clr_drdy_ie", "set_drdy_ie", "clr_mrdy_ie", "set_mrdy_ie",
	"clr_done_ie", "set_done_ie", "clr_sar_ie", "set_sar_ie",
	"clr_linkp_ie", "set_linkp_ie", "clr_dma_ie", "set_dma_ie",
};

static char *mite_MCR_strings[] = {
	"amdevice", "1", "2", "3",
	"4", "5", "portio", "portvxi",
	"psizebyte", "psizehalf (byte & half = word)", "aseqxp1", "11",
	"12", "13", "blocken", "berhand",
	"reqsintlim/reqs0", "reqs1", "reqs2", "rd32",
	"rd512", "rl1", "rl2", "rl8",
	"24", "25", "26", "27",
	"28", "29", "30", "stopen",
};

static char *mite_DCR_strings[] = {
	"amdevice", "1", "2", "3",
	"4", "5", "portio", "portvxi",
	"psizebyte", "psizehalf (byte & half = word)", "aseqxp1", "aseqxp2",
	"aseqxp8", "13", "blocken", "berhand",
	"reqsintlim", "reqs1", "reqs2", "rd32",
	"rd512", "rl1", "rl2", "rl8",
	"23", "24", "25", "27",
	"28", "wsdevc", "wsdevs", "rwdevpack",
};

static char *mite_LKCR_strings[] = {
	"amdevice", "1", "2", "3",
	"4", "5", "portio", "portvxi",
	"psizebyte", "psizehalf (byte & half = word)", "asequp", "aseqdown",
	"12", "13", "14", "berhand",
	"16", "17", "18", "rd32",
	"rd512", "rl1", "rl2", "rl8",
	"24", "25", "26", "27",
	"28", "29", "30", "chngend",
};


static char *mite_CHSR_strings[] = {
	"d.err0", "d.err1", "m.err0", "m.err1",
	"l.err0", "l.err1", "drq0", "drq1",
	"end", "xferr", "operr0", "operr1",
	"stops", "habort", "sabort", "error",
	"16", "conts_rb", "18", "linkc",
	"20", "drdy", "22", "mrdy",
	"24", "done", "26", "sars",
	"28", "lpauses", "30", "int",
};

void mite_dump_regs(struct mite_struct *mite, int channel)
{
	unsigned long mite_io_addr = (unsigned long) mite->mite_io_addr;
	unsigned long addr=0;
	unsigned long temp=0;

	printk("mite_dump_regs ch%i\n", channel);
	printk("mite address is  =0x%08lx\n",mite_io_addr);

	addr = mite_io_addr+MITE_CHOR(channel);
	printk("mite status[CHOR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	mite_decode(mite_CHOR_strings,temp);
	addr = mite_io_addr+MITE_CHCR(channel);
	printk("mite status[CHCR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	mite_decode(mite_CHCR_strings,temp);
	addr = mite_io_addr+MITE_TCR(channel);
	printk("mite status[TCR] at 0x%08lx =0x%08x\n",addr, readl(addr));
	addr = mite_io_addr+MITE_MCR(channel);
	printk("mite status[MCR] at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	mite_decode(mite_MCR_strings,temp);

	addr = mite_io_addr+MITE_MAR(channel);
	printk("mite status[MAR] at 0x%08lx =0x%08x\n",addr, readl(addr));
	addr = mite_io_addr+MITE_DCR(channel);
	printk("mite status[DCR] at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	mite_decode(mite_DCR_strings,temp);
	addr = mite_io_addr+MITE_DAR(channel);
	printk("mite status[DAR] at 0x%08lx =0x%08x\n",addr, readl(addr));
	addr = mite_io_addr+MITE_LKCR(channel);
	printk("mite status[LKCR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	mite_decode(mite_LKCR_strings,temp);
	addr = mite_io_addr + MITE_LKAR(channel);
	printk("mite status[LKAR]at 0x%08lx =0x%08x\n",addr, readl(addr));

	addr = mite_io_addr+MITE_CHSR(channel);
	printk("mite status[CHSR]at 0x%08lx =0x%08lx\n",addr, temp=readl(addr));
	mite_decode(mite_CHSR_strings,temp);
	addr = mite_io_addr+MITE_FCR(channel);
	printk("mite status[FCR] at 0x%08lx =0x%08x\n\n",addr, readl(addr));
}

static void mite_decode(char **bit_str, unsigned int bits)
{
	int i;

	for(i=31;i>=0;i--){
		if(bits&(1<<i)){
			printk(" %s", bit_str[i]);
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
EXPORT_SYMBOL(mite_buf_change);
EXPORT_SYMBOL(mite_bytes_transferred);
EXPORT_SYMBOL(mite_bytes_read);
EXPORT_SYMBOL(mite_bytes_in_transit);
#ifdef DEBUG_MITE
EXPORT_SYMBOL(mite_decode);
EXPORT_SYMBOL(mite_dump_regs);
#endif

#endif

