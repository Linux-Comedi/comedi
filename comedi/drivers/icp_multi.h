/*
    comedi/drivers/icp_multi.h

    Stuff for ICP Multi
    
    Author: Anne Smorthit <anne.smorthit@sfwte.ch>

*/

#ifndef _ICP_MULTI_H_
#define _ICP_MULTI_H_

#include <linux/pci.h>
#include <linux/comedidev.h>



/****************************************************************************/

struct pcilst_struct{
	struct 		pcilst_struct *next;
	int 		used;
	struct pci_dev 	*pcidev;
	unsigned short	vendor;
	unsigned short	device;
	unsigned int	master;
	unsigned char	pci_bus;
	unsigned char	pci_slot;
	unsigned char	pci_func;
	unsigned long	io_addr[5];
	unsigned int	irq;
};

struct pcilst_struct *inova_devices;	// ptr to root list of all Inova devices

/****************************************************************************/

static void pci_card_list_init(unsigned short pci_vendor, char display);
static void pci_card_list_cleanup(unsigned short pci_vendor);
static struct pcilst_struct *find_free_pci_card_by_device(unsigned short vendor_id, unsigned short device_id);
static int find_free_pci_card_by_position(unsigned short vendor_id, unsigned short device_id, unsigned short pci_bus, unsigned short pci_slot, struct pcilst_struct **card);
static struct pcilst_struct *select_and_alloc_pci_card(unsigned short vendor_id, unsigned short device_id, unsigned short pci_bus, unsigned short pci_slot);

static int pci_card_alloc(struct pcilst_struct *amcc);
static int pci_card_free(struct pcilst_struct *amcc);
static void pci_card_list_display(void);
static int pci_card_data(struct pcilst_struct *amcc,
	unsigned char *pci_bus, unsigned char *pci_slot, unsigned char *pci_func,
	unsigned long *io_addr, unsigned short *irq, unsigned short *master);

/****************************************************************************/

/* build list of Inova cards in this system */
static void pci_card_list_init(unsigned short pci_vendor, char display)
{
	struct pci_dev *pcidev;
	struct pcilst_struct *inova,*last;
	int i;

	inova_devices=NULL;
	last=NULL;
	
#if LINUX_VERSION_CODE < 0x020300
	for(pcidev=pci_devices;pcidev;pcidev=pcidev->next){
#else
	pci_for_each_dev(pcidev){
#endif
		if(pcidev->vendor==pci_vendor){
			inova=kmalloc(sizeof(*inova),GFP_KERNEL);
			memset(inova,0,sizeof(*inova));

			inova->pcidev=pcidev;
			if (last) { last->next=inova; }
			     else { inova_devices=inova; }
			last=inova;
			
			inova->vendor=pcidev->vendor;		
			inova->device=pcidev->device;
#if LINUX_VERSION_CODE < 0x020300
			inova->master=pcidev->master;
#else
inova->master = 1;	//XXX
#endif
			inova->pci_bus=pcidev->bus->number;
			inova->pci_slot=PCI_SLOT(pcidev->devfn);
			inova->pci_func=PCI_FUNC(pcidev->devfn);
			for (i=0;i<5;i++)
				inova->io_addr[i]=pci_resource_start(pcidev, i);
			inova->irq=pcidev->irq;
		}
	}

	if (display) pci_card_list_display();
}

/****************************************************************************/
/* free up list of amcc cards in this system */
static void pci_card_list_cleanup(unsigned short pci_vendor)
{
	struct pcilst_struct *inova,*next;

	for(inova=inova_devices; inova; inova=next){
		next=inova->next;
		kfree(inova);
	}
	
	inova_devices=NULL;
}

/****************************************************************************/
/* find first unused card with this device_id */
static struct pcilst_struct *find_free_pci_card_by_device(unsigned short vendor_id, unsigned short device_id)
{
	struct pcilst_struct *inova,*next;

	for (inova=inova_devices; inova; inova=next) {
		next=inova->next;
		if ((!inova->used)&&(inova->device==device_id)&&(inova->vendor==vendor_id)) return inova;
		
	}

	return NULL;
}

/****************************************************************************/
/* find card on requested position */
static int find_free_pci_card_by_position(unsigned short vendor_id, unsigned short device_id, unsigned short pci_bus, unsigned short pci_slot, struct pcilst_struct **card)
{
	struct pcilst_struct *inova,*next;

	*card=NULL;
	for (inova=inova_devices; inova; inova=next) {
		next=inova->next;
		if ((inova->vendor==vendor_id)&&(inova->device==device_id)&&(inova->pci_bus==pci_bus)&&(inova->pci_slot==pci_slot)) {
			if (!(inova->used)) {
				*card=inova;
				return 0;	// ok, card is found
			} else {
				return 2;	// card exist but is used
			}
		}
	}

        return 1; // no card found
}

/****************************************************************************/
/* mark card as used */
static int pci_card_alloc(struct pcilst_struct *inova)
{
	if (!inova) return -1;

	if (inova->used) return 1;
	if(pci_enable_device(inova->pcidev)) return -1;
	inova->used=1;
	return 0;
}

/****************************************************************************/
/* mark card as free */
static int pci_card_free(struct pcilst_struct *inova)
{
	if (!inova) return -1;

	if (!inova->used) return 1;
	inova->used=0;
	return 0;
}

/****************************************************************************/
/* display list of found cards */
static void pci_card_list_display(void)
{
	struct pcilst_struct *inova, *next;

	printk("Anne's List of pci cards\n");
	printk("bus:slot:func vendor device master io_inova io_daq irq used\n");

	for (inova=inova_devices; inova; inova=next) {
		next=inova->next;
		printk("%2d   %2d   %2d  0x%4x 0x%4x   %3s   0x%8lx 0x%8lx  %2d  %2d\n",
			inova->pci_bus,inova->pci_slot,inova->pci_func,inova->vendor,inova->device,inova->master?"yes":"no",
			inova->io_addr[0],inova->io_addr[2],inova->irq,inova->used);
		
	}
}

/****************************************************************************/
/* return all card information for driver */
static int pci_card_data(struct pcilst_struct *inova,
	unsigned char *pci_bus, unsigned char *pci_slot, unsigned char *pci_func,
	unsigned long *io_addr, unsigned short *irq, unsigned short *master)
{
	int	i;
	
	if (!inova) return -1;
	*pci_bus=inova->pci_bus;
	*pci_slot=inova->pci_slot;
	*pci_func=inova->pci_func;
	for (i=0;i<5;i++)
		io_addr[i]=inova->io_addr[i];
	*irq=inova->irq;
	*master=inova->master;
	return 0;
}

/****************************************************************************/
/* select and alloc card */
static struct pcilst_struct *select_and_alloc_pci_card(unsigned short vendor_id, unsigned short device_id, unsigned short pci_bus, unsigned short pci_slot)
{
	struct pcilst_struct *card;
	
	if ((pci_bus<1)&(pci_slot<1)) { // use autodetection
		if ((card=find_free_pci_card_by_device(vendor_id,device_id))==NULL) {
			rt_printk(" - Unused card not found in system!\n");
			return NULL;
		}
	} else {
		switch (find_free_pci_card_by_position(vendor_id,device_id,pci_bus,pci_slot,&card)) {
		case 1:
			rt_printk(" - Card not found on requested position b:s %d:%d!\n",pci_bus,pci_slot);
			return NULL;
		case 2:
			rt_printk(" - Card on requested position is used b:s %d:%d!\n",pci_bus,pci_slot);
			return NULL;
		}
	}


	if (pci_card_alloc(card)!=0) {
		rt_printk(" - Can't allocate card!\n");
		return NULL;
	}

	return card;
}

#endif
