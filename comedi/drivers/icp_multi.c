/*
 * comedi/drivers/icp_multi.c
 *
 * Author: Anne Smorthit <anne.smorthit@sfwte.ch>
 *
 *
 *  hardware driver for Inova cards:
 *   card:   ICP_MULTI
 *   driver: icp_multi
 *
 * Options:
 *  [0] - PCI bus number - if bus number and slot number are 0, 
 *                         then driver search for first unused card
 *  [1] - PCI slot number 
 * 
*/
/*
Driver: icp_multi.o
Description: Inova ICP Multi
Author: Anne Smorthit <anne.smorthit@sfwte.ch>
Devices: [Inova] ICP Multi (icp_multi)

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
#include <linux/pci.h>
#include <asm/io.h>
#include <linux/comedidev.h>
#include "icp_multi.h"


#define VENDOR_ID	0x104C	/* PCI vendor ID */
#define DEVICE_ID	0x8000	/* Device ID */

#define ICP_MULTI_EXTDEBUG

// Hardware types of the cards
#define TYPE_ICP_MULTI	0

#define IORANGE_ICP_MULTI 	32		

#define ICP_MULTI_ADC_CSR	0	/* R/W:	ADC command/status register */
#define ICP_MULTI_AI		2	/* R:	Analogue input data */
#define ICP_MULTI_DAC_CSR	4	/* R/W:	DAC command/status register */
#define ICP_MULTI_AO		6	/* R/W:	Analogue output data */
#define ICP_MULTI_DI		8	/* R/W:	Digital inouts */
#define ICP_MULTI_DO		0x0A	/* R/W:	Digital outputs */
#define ICP_MULTI_INT_EN	0x0C	/* R/W:	Interrupt enable register */
#define ICP_MULTI_INT_STAT	0x0E	/* R/W:	Interrupt status register */
#define ICP_MULTI_CNTR0		0x10	/* R/W:	Counter 0 */
#define ICP_MULTI_CNTR1		0x12	/* R/W:	counter 1 */
#define ICP_MULTI_CNTR2		0x14	/* R/W:	Counter 2 */
#define ICP_MULTI_CNTR3		0x16	/* R/W:	Counter 3 */

#define ICP_MULTI_PCI_MEM_WINDOW_SIZE		0x10000		/* 64K */

// Define bits from ADC command/status register
#define	ADC_ST		0x0001		/* Start ADC */
#define	ADC_BSY		0x0001		/* ADC busy */
#define ADC_BI		0x0010		/* Bipolar input range 1 = bipolar */
#define ADC_RA		0x0020		/* Input range 0 = 5V, 1 = 10V */
#define	ADC_DI		0x0040		/* Differential input mode 1 = differential */

// Define bits from DAC command/status register
#define	DAC_ST		0x0001		/* Start DAC */
#define DAC_BSY		0x0001		/* DAC busy */
#define	DAC_BI		0x0010		/* Bipolar input range 1 = bipolar */
#define	DAC_RA		0x0020		/* Input range 0 = 5V, 1 = 10V */

// Define bits from interrupt enable/status registers
#define	ADC_READY	0x0001		/* A/d conversion ready interrupt */
#define	DAC_READY	0x0002		/* D/a conversion ready interrupt */
#define	DOUT_ERROR	0x0004		/* Digital output error interrupt */
#define	DIN_STATUS	0x0008		/* Digital input status change interrupt */
#define	CIE0		0x0010		/* Counter 0 overrun interrupt */
#define	CIE1		0x0020		/* Counter 1 overrun interrupt */
#define	CIE2		0x0040		/* Counter 2 overrun interrupt */
#define	CIE3		0x0080		/* Counter 3 overrun interrupt */

// Useful definitions
#define	Status_IRQ	0x00ff		// All interrupts


// Define analogue range
comedi_lrange range_analog={ 4, {
	UNI_RANGE(5),
	UNI_RANGE(10),
	BIP_RANGE(5),
	BIP_RANGE(10)
	}
};

static char range_codes_analog[]={0x00, 0x20, 0x10, 0x30};


/*
==============================================================================
	Forward declarations
==============================================================================
*/
static int icp_multi_attach(comedi_device *dev, comedi_devconfig *it);
static int icp_multi_detach(comedi_device *dev);


/*
==============================================================================
	Data & Structure declarations
==============================================================================
*/
static unsigned short	pci_list_builded=0;	/*=1 list of card is know */

typedef struct {
	char 		*name;		// driver name
	int		vendor_id;	// PCI vendor a device ID of card
	int		device_id;
	int		iorange;	// I/O range len
	char		have_irq;	// 1=card support IRQ
	char		cardtype;	// 0=ICP Multi
	int 		n_aichan;	// num of A/D chans
	int 		n_aichand;	// num of A/D chans in diff mode
	int 		n_aochan;	// num of D/A chans
	int 		n_dichan;	// num of DI chans
	int 		n_dochan;	// num of DO chans
	int		n_ctrs;		// num of counters
	int		ai_maxdata;	// resolution of A/D
	int		ao_maxdata;	// resolution of D/A
	comedi_lrange	*rangelist_ai;	// rangelist for A/D
	char		*rangecode;	// range codes for programming
	comedi_lrange	*rangelist_ao;	// rangelist for D/A
} boardtype;

static boardtype boardtypes[] =
{
	{"icp_multi",		// Driver name
	 VENDOR_ID,		// PCI vendor ID
	 DEVICE_ID,		// PCI device ID
	 IORANGE_ICP_MULTI,	// I/O range length
	 1,			// 1=Card supports interrupts
	 TYPE_ICP_MULTI,	// Card type = ICP MULTI
	 16,			// Num of A/D channels
	 8,			// Num of A/D channels in diff mode
	 4,			// Num of D/A channels
	 16,			// Num of digital inputs
	 8,			// Num of digital outputs
	 4,			// Num of counters
	 0x0fff,		// Resolution of A/D
	 0x0fff,		// Resolution of D/A
	 &range_analog,		// Rangelist for A/D
	 range_codes_analog,	// Range codes for programming
	 &range_analog },	// Rangelist for D/A
};

#define n_boardtypes (sizeof(boardtypes)/sizeof(boardtype))

static comedi_driver driver_icp_multi={
	driver_name:	"icp_multi",
	module:		THIS_MODULE,
	attach:		icp_multi_attach,
	detach:		icp_multi_detach,
	num_names:	n_boardtypes,
	board_name:	boardtypes,
	offset:		sizeof(boardtype),
};

typedef struct{
	char			valid;			// card is usable
	char			neverending_ai;		// we do unlimited AI
	unsigned int		AdcCmdStatus;		// ADC Command/Status register
	unsigned int		DacCmdStatus;		// DAC Command/Status register
	unsigned int		IntEnable;		// Interrupt Enable register
	unsigned int		IntStatus;		// Interrupt Status register
	unsigned int		ai_do;			// what do AI? 0=nothing, 1 to 4 mode
	unsigned int		ai_act_scan;		// how many scans we finished
	unsigned int		ai_act_chan;		// actual position in actual scan
	unsigned int 		ai_buf_ptr;		// data buffer ptr in samples
	unsigned char		ai_eos;			// 1=EOS wake up
	unsigned int		act_chanlist[32];	// list of scaned channel
	unsigned char		act_chanlist_len;	// len of scanlist
	unsigned char 		act_chanlist_pos;	// actual position in MUX list
	unsigned int		ai_scans;		// len of scanlist
	unsigned int		ai_n_chan;		// how many channels is measured	
	unsigned int		*ai_chanlist;		// actaul chanlist
	unsigned int		ai_flags;		// flaglist
	unsigned int		ai_data_len;		// len of data buffer
	sampl_t			*ai_data;		// data buffer
	sampl_t			ao_data[4];		// data output buffer
	sampl_t			di_data;		// Digital input data
	sampl_t			do_data;		// Digital output data
} icp_multi_private;

#define devpriv ((icp_multi_private *)dev->private)
#define this_board ((boardtype *)dev->board_ptr)

/* 
==============================================================================
	More forward declarations
==============================================================================
*/

int check_channel_list(comedi_device * dev, comedi_subdevice * s, unsigned int *chanlist, unsigned int n_chan);
void setup_channel_list(comedi_device * dev, comedi_subdevice * s, unsigned int *chanlist, unsigned int n_chan);
static int icp_multi_reset(comedi_device *dev);


/* 
==============================================================================
	Functions
==============================================================================
*/


/*
==============================================================================

	Name:	icp_multi_insn_read_ai

	Description:
		This function reads a single analogue input.

	Parameters:
		comedi_device *dev	Pointer to current device structure
		comedi_subdevice *s	Pointer to current subdevice structure
		comedi_insn *insn	Pointer to current comedi instruction
		lsampl_t *data		Pointer to analogue input data

	Returns:int			Nmuber of instructions executed

==============================================================================
*/
int icp_multi_insn_read_ai(comedi_device * dev, comedi_subdevice * s, comedi_insn *insn, lsampl_t *data)
{
	int n,timeout;

#ifdef ICP_MULTI_EXTDEBUG
	rt_printk("icp multi EDBG: BGN: icp_multi_insn_read_ai(...)\n");
#endif
	// Disable A/D conversion ready interrupt
	devpriv->IntEnable &= ~ADC_READY;
	outw(devpriv->IntEnable,dev->iobase + ICP_MULTI_INT_EN);
	
	// Clear interrupt status
	devpriv->IntStatus |= ADC_READY;
	outw(devpriv->IntStatus,dev->iobase + ICP_MULTI_INT_STAT);

	// Set up appropriate channel, mode and range data, for specified channel
	setup_channel_list(dev, s, &insn->chanspec, 1);

#ifdef ICP_MULTI_EXTDEBUG
	rt_printk("icp_multi A ST=%4x IO=%x\n",inw(dev->iobase+ICP_MULTI_ADC_CSR), dev->iobase+ICP_MULTI_ADC_CSR);
#endif

	for (n=0; n<insn->n; n++) {
		// Set start ADC bit
		devpriv->AdcCmdStatus |= ADC_ST;
		outw(devpriv->AdcCmdStatus, dev->iobase+ICP_MULTI_ADC_CSR);
		devpriv->AdcCmdStatus &= ~ADC_ST;

#ifdef ICP_MULTI_EXTDEBUG
    		rt_printk("icp multi B n=%d ST=%4x\n",n,inw(dev->iobase+ICP_MULTI_ADC_CSR));
#endif

		udelay(1);

#ifdef ICP_MULTI_EXTDEBUG
    		rt_printk("icp multi C n=%d ST=%4x\n",n,inw(dev->iobase+ICP_MULTI_ADC_CSR));
#endif

		// Wait for conversion to complete, or get fed up waiting
    		timeout=100;
    		while (timeout--) {
			if (!(inw(dev->iobase+ICP_MULTI_ADC_CSR) & ADC_BSY))
				goto conv_finish;

#ifdef ICP_MULTI_EXTDEBUG
			if (!(timeout%10))
				rt_printk("icp multi D n=%d tm=%d ST=%4x\n",n,timeout,inw(dev->iobase+ICP_MULTI_ADC_CSR));
#endif

			udelay(1);
    		}

		// If we reach here, a timeout has occurred
    		comedi_error(dev,"A/D insn timeout");

		// Disable interrupt
		devpriv->IntEnable &= ~ADC_READY;
		outw(devpriv->IntEnable,dev->iobase + ICP_MULTI_INT_EN);

		// Clear interrupt status
		devpriv->IntStatus |= ADC_READY;
		outw(devpriv->IntStatus,dev->iobase + ICP_MULTI_INT_STAT);

		// Clear data received
    		data[n]=0;

#ifdef ICP_MULTI_EXTDEBUG
		rt_printk("icp multi EDBG: END: icp_multi_insn_read_ai(...) n=%d\n",n);
#endif
    		return -ETIME;

conv_finish:
		data[n] = (inw(dev->iobase+ICP_MULTI_AI) >> 4 ) & 0x0fff;
	}
	
	// Disable interrupt
	devpriv->IntEnable &= ~ADC_READY;
	outw(devpriv->IntEnable,dev->iobase + ICP_MULTI_INT_EN);

	// Clear interrupt status
	devpriv->IntStatus |= ADC_READY;
	outw(devpriv->IntStatus,dev->iobase + ICP_MULTI_INT_STAT);

#ifdef ICP_MULTI_EXTDEBUG
		rt_printk("icp multi EDBG: END: icp_multi_insn_read_ai(...) n=%d\n",n);
#endif
	return n;
}

/*
==============================================================================
	
	Name:	icp_multi_insn_write_ao

	Description:
		This function writes a single analogue output.

	Parameters:
		comedi_device *dev	Pointer to current device structure
		comedi_subdevice *s	Pointer to current subdevice structure
		comedi_insn *insn	Pointer to current comedi instruction
		lsampl_t *data		Pointer to analogue output data

	Returns:int			Nmuber of instructions executed

==============================================================================
*/
int icp_multi_insn_write_ao(comedi_device * dev, comedi_subdevice * s, comedi_insn *insn, lsampl_t *data)
{
	int n, chan, range, timeout;

#ifdef ICP_MULTI_EXTDEBUG
	rt_printk("icp multi EDBG: BGN: icp_multi_insn_write_ao(...)\n");
#endif
	// Disable D/A conversion ready interrupt
	devpriv->IntEnable &= ~DAC_READY;
	outw(devpriv->IntEnable,dev->iobase + ICP_MULTI_INT_EN);
	
	// Clear interrupt status
	devpriv->IntStatus |= DAC_READY;
	outw(devpriv->IntStatus,dev->iobase + ICP_MULTI_INT_STAT);

	// Get channel number and range
	chan = CR_CHAN(insn->chanspec);
	range = CR_RANGE(insn->chanspec);

	// Set up range and channel data
	// Bit 4 = 1 : Bipolar
	// Bit 5 = 0 : 5V
	// Bit 5 = 1 : 10V
	// Bits 8-9 : Channel number
	devpriv->DacCmdStatus &= 0xfccf;
	devpriv->DacCmdStatus |= this_board->rangecode[range];
	devpriv->DacCmdStatus |= (chan << 8);
	
	outw(devpriv->DacCmdStatus, dev->iobase+ICP_MULTI_DAC_CSR);

	for (n=0; n<insn->n; n++) {
		// Wait for analogue output data register to be ready for new data, or get fed up waiting
    		timeout=100;
    		while (timeout--) {
			if (!(inw(dev->iobase+ICP_MULTI_DAC_CSR) & DAC_BSY))
				goto dac_ready;

#ifdef ICP_MULTI_EXTDEBUG
			if (!(timeout%10))
				rt_printk("icp multi A n=%d tm=%d ST=%4x\n",n,timeout,inw(dev->iobase+ICP_MULTI_DAC_CSR));
#endif

			udelay(1);
    		}

		// If we reach here, a timeout has occurred
    		comedi_error(dev,"D/A insn timeout");

		// Disable interrupt
		devpriv->IntEnable &= ~DAC_READY;
		outw(devpriv->IntEnable,dev->iobase + ICP_MULTI_INT_EN);

		// Clear interrupt status
		devpriv->IntStatus |= DAC_READY;
		outw(devpriv->IntStatus,dev->iobase + ICP_MULTI_INT_STAT);

		// Clear data received
    		devpriv->ao_data[chan]=0;

#ifdef ICP_MULTI_EXTDEBUG
		rt_printk("icp multi EDBG: END: icp_multi_insn_write_ao(...) n=%d\n",n);
#endif
    		return -ETIME;

dac_ready:
		// Write data to analogue output data register
		outw(data[n], dev->iobase + ICP_MULTI_AO);

		// Set DAC_ST bit to write the data to selected channel
		devpriv->DacCmdStatus |= DAC_ST;
		outw(devpriv->DacCmdStatus, dev->iobase+ICP_MULTI_DAC_CSR);
		devpriv->DacCmdStatus &= ~DAC_ST;

		// Save analogue output data
		devpriv->ao_data[chan]=data[n];
	}

#ifdef ICP_MULTI_EXTDEBUG
		rt_printk("icp multi EDBG: END: icp_multi_insn_write_ao(...) n=%d\n",n);
#endif
		return n;
}

/*
==============================================================================
	
	Name:	icp_multi_insn_read_ao

	Description:
		This function reads a single analogue output.

	Parameters:
		comedi_device *dev	Pointer to current device structure
		comedi_subdevice *s	Pointer to current subdevice structure
		comedi_insn *insn	Pointer to current comedi instruction
		lsampl_t *data		Pointer to analogue output data

	Returns:int			Nmuber of instructions executed

==============================================================================
*/
int icp_multi_insn_read_ao(comedi_device * dev, comedi_subdevice * s, comedi_insn *insn, lsampl_t *data)
{
	int n,chan;

	// Get channel number	
	chan = CR_CHAN(insn->chanspec);

	// Read analogue outputs
	for (n=0; n<insn->n; n++) 
		data[n]=devpriv->ao_data[chan];

	return n;
}

/*
==============================================================================
	
	Name:	icp_multi_insn_bits_di

	Description:
		This function reads the digital inputs.

	Parameters:
		comedi_device *dev	Pointer to current device structure
		comedi_subdevice *s	Pointer to current subdevice structure
		comedi_insn *insn	Pointer to current comedi instruction
		lsampl_t *data		Pointer to analogue output data

	Returns:int			Nmuber of instructions executed

==============================================================================
*/
int icp_multi_insn_bits_di(comedi_device *dev,comedi_subdevice *s, comedi_insn *insn, lsampl_t *data)
{
	data[1] = inw(dev->iobase + ICP_MULTI_DI);

	return 2;
}

/*
==============================================================================
	
	Name:	icp_multi_insn_bits_do

	Description:
		This function writes the appropriate digital outputs.

	Parameters:
		comedi_device *dev	Pointer to current device structure
		comedi_subdevice *s	Pointer to current subdevice structure
		comedi_insn *insn	Pointer to current comedi instruction
		lsampl_t *data		Pointer to analogue output data

	Returns:int			Nmuber of instructions executed

==============================================================================
*/
int icp_multi_insn_bits_do(comedi_device *dev,comedi_subdevice *s, comedi_insn *insn,lsampl_t *data)
{
	if(data[0]){
		s->state &= ~data[0];
		s->state |= (data[0]&data[1]);
		outw(s->state, dev->iobase + ICP_MULTI_DO);
	}
	data[1] = inw(dev->iobase + ICP_MULTI_DI);

	return 2;
}


/*
==============================================================================

	Name:	icp_multi_insn_read_ctr

	Description:
		This function reads the specified counter.

	Parameters:
		comedi_device *dev	Pointer to current device structure
		comedi_subdevice *s	Pointer to current subdevice structure
		comedi_insn *insn	Pointer to current comedi instruction
		lsampl_t *data		Pointer to counter data

	Returns:int			Nmuber of instructions executed

==============================================================================
*/
int icp_multi_insn_read_ctr(comedi_device * dev, comedi_subdevice * s, comedi_insn *insn, lsampl_t *data)
{
        return 0;
}


/*
==============================================================================

	Name:	icp_multi_insn_write_ctr

	Description:
		This function write to the specified counter.

	Parameters:
		comedi_device *dev	Pointer to current device structure
		comedi_subdevice *s	Pointer to current subdevice structure
		comedi_insn *insn	Pointer to current comedi instruction
		lsampl_t *data		Pointer to counter data

	Returns:int			Nmuber of instructions executed

==============================================================================
*/
int icp_multi_insn_write_ctr(comedi_device * dev, comedi_subdevice * s, comedi_insn *insn, lsampl_t *data)
{
        return 0;
}



/*
==============================================================================

	Name:	interrupt_service_icp_multi

	Description:
		This function is the interrupt service routine for all
		interrupts generated by the icp multi board.

	Parameters:
		int irq
		void *d			Pointer to current device
		struct pt_regs *regs	Pointer to

	Returns:int			Nmuber of instructions executed

==============================================================================
*/
static void interrupt_service_icp_multi(int irq, void *d, struct pt_regs *regs)
{
        comedi_device *dev = d;
	int	int_no;
	
#ifdef ICP_MULTI_EXTDEBUG
	rt_printk("icp multi EDBG: BGN: interrupt_service_icp_multi(%d,...)\n",irq);
#endif

	// Is this interrupt from our board?
	int_no = inw(dev->iobase + ICP_MULTI_INT_STAT) & Status_IRQ;
	if (!int_no) 	
		// No, exit
		return;

#ifdef ICP_MULTI_EXTDEBUG
	rt_printk("icp multi EDBG: interrupt_service_icp_multi() ST: %4x\n",inw(dev->iobase + ICP_MULTI_INT_STAT));
#endif

	// Determine which interrupt is active & handle it
	switch(int_no)
	{
		case ADC_READY:
				break;
		case DAC_READY:
				break;
		case DOUT_ERROR:
				break;
		case DIN_STATUS:
				break;
		case CIE0:
				break;
		case CIE1:
				break;
		case CIE2:
				break;
		case CIE3:
				break;
		default:
				break;

	}

#ifdef ICP_MULTI_EXTDEBUG
	rt_printk("icp multi EDBG: END: interrupt_service_icp_multi(...)\n");
#endif
}


/*
==============================================================================

	Name:	check_channel_list
	
	Description:
		This function checks if the channel list, provided by user
		is built correctly

	Parameters:
		comedi_device *dev	Pointer to current sevice structure
		comedi_subdevice *s	Pointer to current subdevice structure
		unsigned int *chanlist	Pointer to packed channel list
		unsigned int n_chan	Number of channels to scan

	Returns:int 0 = failure
		    1 = success

==============================================================================
*/
int check_channel_list(comedi_device * dev, comedi_subdevice * s, unsigned int *chanlist, unsigned int n_chan)
{
        unsigned int i;
    
#ifdef ICP_MULTI_EXTDEBUG
	rt_printk("icp multi EDBG:  check_channel_list(...,%d)\n",n_chan);
#endif
    	// Check that we at least have one channel to check
	if (n_chan<1) {
		comedi_error(dev,"range/channel list is empty!");
		return 0;
        }

	// Check all channels
	for (i=0; i<n_chan; i++) {
		// Check that channel number is < maximum
		if (CR_AREF(chanlist[i])==AREF_DIFF) {
			if (CR_CHAN(chanlist[i]) > this_board->n_aichand) {
				comedi_error(dev,"Incorrect differential ai channel number");
				return 0;
			}
                }
		else {
			if (CR_CHAN(chanlist[i]) > this_board->n_aichan) {
				comedi_error(dev,"Incorrect ai channel number");
				return 0;
			}
                }
	}
	return 1;
}



/*
==============================================================================

	Name:	setup_channel_list

	Description:
		This function sets the appropriate channel selection,
		differential input mode and range bits in the ADC Command/
		Status register.

	Parameters:
		comedi_device *dev	Pointer to current sevice structure
		comedi_subdevice *s	Pointer to current subdevice structure
		unsigned int *chanlist	Pointer to packed channel list
		unsigned int n_chan	Number of channels to scan

	Returns:Void

==============================================================================
*/
void setup_channel_list(comedi_device * dev, comedi_subdevice * s, unsigned int *chanlist,
			unsigned int n_chan)
{
	unsigned int i, range, chanprog;
	unsigned int diff;

#ifdef ICP_MULTI_EXTDEBUG
	rt_printk("icp multi EDBG:  setup_channel_list(...,%d)\n",n_chan);
#endif
	devpriv->act_chanlist_len=n_chan;
	devpriv->act_chanlist_pos=0;

 	for (i=0; i<n_chan; i++) {
		// Get channel
		chanprog=CR_CHAN(chanlist[i]);

		// Determine if it is a differential channel (Bit 15  = 1)
		if (CR_AREF(chanlist[i])==AREF_DIFF) {
			diff = 1;
			chanprog &= 0x0007;
		}
		else {
			diff = 0;
			chanprog &= 0x000f;
		}

		// Clear channel, range and input mode bits in A/D command/status register
		devpriv->AdcCmdStatus &= 0xf00f;

		// Set channel number and differential mode status bit
		if (diff) {
			// Set channel number, bits 9-11 & mode, bit 6
			devpriv->AdcCmdStatus |= (chanprog << 9);
			devpriv->AdcCmdStatus |= ADC_DI;
                }
		else
			// Set channel number, bits 8-11
			devpriv->AdcCmdStatus |= (chanprog << 8);

		// Get range for current channel
		range=this_board->rangecode[CR_RANGE(chanlist[i])];
		// Set range. bits 4-5
		devpriv->AdcCmdStatus |= range;

		/* Output channel, range, mode to ICP Multi*/
		outw(devpriv->AdcCmdStatus, dev->iobase+ICP_MULTI_ADC_CSR);

#ifdef ICP_MULTI_EXTDEBUG
		rt_printk("GS: %2d. [%4x]=%4x %4x\n", i, chanprog, range, devpriv->act_chanlist[i]);
#endif
	}

}


/*
==============================================================================

	Name:	icp_multi_reset

	Description:
		This function resets the icp multi device to a 'safe' state
	
	Parameters:
		comedi_device *dev	Pointer to current sevice structure

	Returns:int	0 = success

==============================================================================
*/
static int icp_multi_reset(comedi_device *dev)
{
        unsigned int    i;

#ifdef ICP_MULTI_EXTDEBUG
	rt_printk("icp_multi EDBG: BGN: icp_multi_reset(...)\n");
#endif
	// Clear INT enables and requests
	outw(0, dev->iobase + ICP_MULTI_INT_EN);
	outw(0x00ff, dev->iobase + ICP_MULTI_INT_STAT);

	if (this_board->n_aochan)
		// Set DACs to 0..5V range and 0V output
		for (i =0; i < this_board->n_aochan; i++) {
			devpriv->DacCmdStatus &= 0xfcce;

			// Set channel number
			devpriv->DacCmdStatus |= (i << 8);

			// Output 0V
			outw(0, dev->iobase+ICP_MULTI_AO);

			// Set start conversion bit
			devpriv->DacCmdStatus |= DAC_ST;

			// Output to command / status register
			outw(devpriv->DacCmdStatus, dev->iobase+ICP_MULTI_DAC_CSR);
			
			// Delay to allow DAC time to recover
			udelay(1);
		}
	
	// Digital outputs to 0
	outw(0, dev->iobase + ICP_MULTI_DO);

#ifdef ICP_MULTI_EXTDEBUG
	rt_printk("icp multi EDBG: END: icp_multi_reset(...)\n");
#endif
	return 0;
}

/*
==============================================================================

	Name:	icp_multi_attach
	
	Description:
		This function sets up all the appropriate data for the current
		device.

	Parameters:
		comedi_device *dev	Pointer to current device structure
		comedi_devconfig *it	Pointer to current device configuration

	Returns:int	0 = success

==============================================================================
*/
static int icp_multi_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret, subdev;
	unsigned short io_addr[5], master,irq;
	struct pcilst_struct *card=NULL;
        unsigned int iobase;
	unsigned char pci_bus, pci_slot, pci_func;
	
	if (!pci_list_builded) {
		pci_card_list_init(VENDOR_ID,
#ifdef ICP_MULTI_EXTDEBUG
						    1);
#else
						    0);
#endif
		pci_list_builded=1;
	}

	printk("comedi%d: icp_multi: board=%s",dev->minor,this_board->name);

	if ((card=select_and_alloc_pci_card(VENDOR_ID, this_board->device_id, it->options[0], it->options[1]))==NULL)
		return -EIO;
	
	if ((pci_card_data(card, &pci_bus, &pci_slot, &pci_func, &io_addr[0], &irq, &master))<0) {
		pci_card_free(card);
		printk(" - Can't get configuration data!\n");
		return -EIO;
	}
	
	iobase=io_addr[2];

	printk(", b:s:f=%d:%d:%d, io=0x%4x \n", pci_bus, pci_slot, pci_func, iobase);
	
        if (check_region(iobase, this_board->iorange) < 0) {
		pci_card_free(card);
		printk("I/O port conflict\n");
		return -EIO;
        }

        request_region(iobase, this_board->iorange, "Inova Icp Multi");
        dev->iobase=iobase;
    
	dev->board_name = this_board->name;

	if((ret=alloc_private(dev, sizeof(icp_multi_private)))<0) {
    		release_region(dev->iobase, this_board->iorange);
		pci_card_free(card);
		return -ENOMEM;
	}
		
        dev->n_subdevices = 0;
	if (this_board->n_aichan) dev->n_subdevices++;
	if (this_board->n_aochan) dev->n_subdevices++;
	if (this_board->n_dichan) dev->n_subdevices++;
	if (this_board->n_dochan) dev->n_subdevices++;
	if (this_board->n_ctrs)	  dev->n_subdevices++;
	
        if((ret=alloc_subdevices(dev))<0) {
    		release_region(dev->iobase, this_board->iorange);
		pci_card_free(card);
    		return ret;
	}

	if (this_board->have_irq) {
		if (irq)  {
			if (comedi_request_irq(irq, interrupt_service_icp_multi, SA_SHIRQ, "Inova Icp Multi", dev)) {
				printk(", unable to allocate IRQ %d, DISABLING IT", irq);
				irq=0; /* Can't use IRQ */
			} else {
				printk(", irq=%d", irq);
			}    
		} else {
			printk(", IRQ disabled");
		}
	} else {
		irq=0;
	}
	
        dev->irq = irq;

	printk(".\n");

	subdev=0;
	
	if (this_board->n_aichan) {
		s = dev->subdevices + subdev;
		dev->read_subdev = s;
		s->type = COMEDI_SUBD_AI;
		s->subdev_flags = SDF_READABLE|SDF_COMMON|SDF_GROUND;
		if (this_board->n_aichand) s->subdev_flags |= SDF_DIFF;
		s->n_chan = this_board->n_aichan;
		s->maxdata = this_board->ai_maxdata;
		s->len_chanlist = this_board->n_aichan;
		s->range_table = this_board->rangelist_ai;
		s->insn_read=icp_multi_insn_read_ai;
		subdev++;
	}
	
	if (this_board->n_aochan) {
		s = dev->subdevices + subdev;
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_WRITEABLE|SDF_GROUND|SDF_COMMON;
		s->n_chan = this_board->n_aochan;
		s->maxdata = this_board->ao_maxdata;
		s->len_chanlist = this_board->n_aochan;
		s->range_table = this_board->rangelist_ao;
		s->insn_write=icp_multi_insn_write_ao;
		s->insn_read=icp_multi_insn_read_ao;
		subdev++;
	}

	if (this_board->n_dichan) {
		s = dev->subdevices + subdev;
		s->type = COMEDI_SUBD_DI;
		s->subdev_flags = SDF_READABLE|SDF_GROUND|SDF_COMMON;
		s->n_chan = this_board->n_dichan;
		s->maxdata = 1;
		s->len_chanlist = this_board->n_dichan;
		s->range_table = &range_digital;
		s->io_bits=0;		/* all bits input */
		s->insn_bits=icp_multi_insn_bits_di;
		subdev++;
	}

	if (this_board->n_dochan) {
		s = dev->subdevices + subdev;
		s->type = COMEDI_SUBD_DO;
		s->subdev_flags = SDF_WRITEABLE|SDF_GROUND|SDF_COMMON;
		s->n_chan = this_board->n_dochan;
		s->maxdata = 1;
		s->len_chanlist = this_board->n_dochan;
		s->range_table = &range_digital;
		s->io_bits=(1 << this_board->n_dochan)-1;	/* all bits output */
		s->state=0;
		s->insn_bits=icp_multi_insn_bits_do;
		subdev++;
	}

	if (this_board->n_ctrs) {
		s = dev->subdevices + subdev;
		s->type = COMEDI_SUBD_COUNTER;
		s->subdev_flags = SDF_WRITEABLE|SDF_GROUND|SDF_COMMON;
		s->n_chan = this_board->n_ctrs;
		s->maxdata = 0xffff;
		s->len_chanlist = this_board->n_ctrs;
		s->state=0;
		s->insn_read=icp_multi_insn_read_ctr;
		s->insn_write=icp_multi_insn_write_ctr;
		subdev++;
	}
	
	devpriv->valid=1;

	icp_multi_reset(dev);

	return 0;
}

/*
==============================================================================

	Name:	icp_multi_detach
	
	Description:
		This function releases all the resources used by the current
		device.
	
	Parameters:
		comedi_device *dev	Pointer to current device structure

	Returns:int	0 = success

==============================================================================
*/
static int icp_multi_detach(comedi_device *dev)
{

	if (dev->private) 
		if (devpriv->valid)
			icp_multi_reset(dev);
	
	if (dev->irq)
		comedi_free_irq(dev->irq,dev);

	if (dev->iobase)
		release_region(dev->iobase,this_board->iorange);

	if (pci_list_builded) {
	    	pci_card_list_cleanup(VENDOR_ID);
		pci_list_builded=0;
	}


	return 0;
}

/* 
==============================================================================
*/
COMEDI_INITCLEANUP(driver_icp_multi);
/* 
==============================================================================
*/
