/*
    comedi/drivers/amcc_s5933.h

    Stuff for AMCC S5933 PCI Controller
    
    Author: Michal Dobes <majkl@tesnet.cz>

    Inspirated from general-purpose AMCC S5933 PCI Matchmaker driver  
    made by Andrea Cisternino  <acister@pcape1.pi.infn.it>
    and as result of espionage from MITE code made by David A. Schleef.
    Thanks to AMCC for their on-line documentation and bus master DMA
    example.
*/

#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/pci.h>
#include <linux/comedidev.h>

#include "amcc_s5933.h"

#ifdef PCI_SUPPORT_VER1
#error    Sorry, no support for 2.1.55 and older! :-((((
#endif


/****************************************************************************/

/* build list of amcc cards in this system */
void pci_card_list_init(unsigned short pci_vendor, char display)
{
	struct pci_dev *pcidev;
	struct pcilst_struct *amcc,*last;
	int i;

	amcc_devices=NULL;
	last=NULL;

	pci_for_each_dev(pcidev){
		if(pcidev->vendor==pci_vendor){
			amcc=kmalloc(sizeof(*amcc),GFP_KERNEL);
			memset(amcc,0,sizeof(*amcc));

			amcc->pcidev=pcidev;
			if (last) { last->next=amcc; }
			     else { amcc_devices=amcc; }
			last=amcc;

			amcc->vendor=pcidev->vendor;
			amcc->device=pcidev->device;
			amcc->pci_bus=pcidev->bus->number;
			amcc->pci_slot=PCI_SLOT(pcidev->devfn);
			amcc->pci_func=PCI_FUNC(pcidev->devfn);
			for (i=0;i<5;i++)
				amcc->io_addr[i]=pci_resource_start(pcidev, i) & PCI_BASE_ADDRESS_IO_MASK;
			amcc->irq=pcidev->irq;
#if LINUX_VERSION_CODE < 0x020300
			amcc->master=pcidev->master;
#else
//			amcc->master=pcidev->master; // how get this information under 2.4 kernels?
#endif
			
		}
	}

	if (display) pci_card_list_display();
}

/****************************************************************************/
/* free up list of amcc cards in this system */
void pci_card_list_cleanup(unsigned short pci_vendor)
{
	struct pcilst_struct *amcc,*next;

	for(amcc=amcc_devices;amcc;amcc=next){
		next=amcc->next;
		kfree(amcc);
	}
	
	amcc_devices=NULL;
}

/****************************************************************************/
/* find first unused card with this device_id */
struct pcilst_struct *find_free_pci_card_by_device(unsigned short vendor_id, unsigned short device_id)
{
	struct pcilst_struct *amcc,*next;

	for (amcc=amcc_devices;amcc;amcc=next) {
		next=amcc->next;
		if ((!amcc->used)&&(amcc->device==device_id)&&(amcc->vendor==vendor_id)) return amcc;
		
	}

	return NULL;
}

/****************************************************************************/
/* find card on requested position */
int find_free_pci_card_by_position(unsigned short vendor_id, unsigned short device_id, unsigned short pci_bus, unsigned short pci_slot, struct pcilst_struct **card)
{
	struct pcilst_struct *amcc,*next;

	*card=NULL;
	for (amcc=amcc_devices;amcc;amcc=next) {
		next=amcc->next;
		if ((amcc->vendor==vendor_id)&&(amcc->device==device_id)&&(amcc->pci_bus==pci_bus)&&(amcc->pci_slot==pci_slot)) {
			if (!(amcc->used)) {
				*card=amcc;
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
int pci_card_alloc(struct pcilst_struct *amcc)
{
	if (!amcc) return -1;

	if (amcc->used) return 1;
	amcc->used=1;
	return 0;
}

/****************************************************************************/
/* mark card as free */
int pci_card_free(struct pcilst_struct *amcc)
{
	if (!amcc) return -1;

	if (!amcc->used) return 1;
	amcc->used=0;
	return 0;
}

/****************************************************************************/
/* display list of found cards */
void pci_card_list_display(void)
{
	struct pcilst_struct *amcc,*next;

	printk("List of pci cards\n");
	printk("bus:slot:func vendor device master io_amcc io_daq irq used\n");

	for (amcc=amcc_devices;amcc;amcc=next) {
		next=amcc->next;
		printk("%2d   %2d   %2d  0x%4x 0x%4x   %3s   0x%4x 0x%4x  %2d  %2d\n",
			amcc->pci_bus,amcc->pci_slot,amcc->pci_func,amcc->vendor,amcc->device,amcc->master?"yes":"no",
			amcc->io_addr[0],amcc->io_addr[2],amcc->irq,amcc->used);
		
	}
}

/****************************************************************************/
/* return all card information for driver */
int pci_card_data(struct pcilst_struct *amcc,
	unsigned char *pci_bus, unsigned char *pci_slot, unsigned char *pci_func,
	unsigned short *io_addr, unsigned short *irq, unsigned short *master)
{
	int	i;
	
	if (!amcc) return -1;
	*pci_bus=amcc->pci_bus;
	*pci_slot=amcc->pci_slot;
	*pci_func=amcc->pci_func;
	for (i=0;i<5;i++)
		io_addr[i]=amcc->io_addr[i];
	*irq=amcc->irq;
	*master=amcc->master;
	return 0;
}

/****************************************************************************/
/* select and alloc card */
struct pcilst_struct *select_and_alloc_pci_card(unsigned short vendor_id, unsigned short device_id, unsigned short pci_bus, unsigned short pci_slot)
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

