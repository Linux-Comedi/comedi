/*
 *  comedi/drivers/adl_pci9118.c
 *
 *  hardware driver for ADLink cards:
 *   card:   PCI-9118DG, PCI-9118HG, PCI-9118HR
 *   driver: pci9118dg,  pci9118hg,  pci9118hr
 *
 * Author: Michal Dobes <majkl@tesnet.cz>
 *
 * Options:
 *  [0] - PCI bus number - if bus number and slot number are 0, 
 *                         then driver search for first unused card
 *  [1] - PCI slot number 
 *  [2] - A/D channels - card have 16 channels, but supports external multiplexor,
 *                       you can select from 1 to 256 channels (0=16 SE/8 DIFF channels)
 * 
*/
/*
Driver: adl_pci9118.o
Description: Adlink PCI-9118DG, PCI-9118HG, PCI-9118HR
Author: Michal Dobes <majkl@tesnet.cz>
Devices: [ADLink] PCI-9118DG (pci9118dg), PCI-9118HG (pci9118hg),
  PCI-9118HR (pci9118hr)
Status: works

This driver supports AI, AO, DI and DO subdevices.
AI subdevice supports cmd and insn interface,
other subdevices support only insn interface.
For AI:
- If cmd->scan_begin_src=TRIG_EXT then trigger input is TGIN (pin 46).
- If cmd->convert_src=TRIG_EXT then trigger input is EXTTRG (pin 44).
- If cmd->start_src/stop_src=TRIG_EXT then trigger input is TGIN (pin 46).
- It is not neccessary to have cmd.scan_end_arg=cmd.chanlist_len but
  cmd.scan_end_arg modulo cmd.chanlist_len must by 0.
- If return value of cmdtest is 5 then you've bad channel list
  (it isn't possible mixture S.E. and DIFF inputs or bipolar and unipolar
  ranges).
There is know problem with this driver:
- If you use scan_begin_src=TRIG_EXT & convert_src=TRIG_TIMER
  then this mode sometimes discards some samples. :-((

Configuration options:
  [0] - PCI bus of device (optional)
  [1] - PCI slot of device (optional)
          If bus/slot is not specified, the first available PCI
          device will be used.

If you have an external multiplexer, the third option in the option
list should be used to indicate the number of channels in the
multiplexer.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <linux/comedidev.h>
#include "amcc_s5933.h"
#include "8253.h"

#define PCL9118_PARANOIDCHECK		/* if defined, then is used code which control correct channel number on every 12 bit sample */

#define IORANGE_9118 	64		/* XXX: I hope */
#define PCI9118_CHANLEN	255		/* len of chanlist, some source say 256, but reality is 255 :-( */

#define PCI9118_CNT0	0x00		/* R/W: 8254 couter 0 */
#define PCI9118_CNT1	0x04		/* R/W: 8254 couter 0 */
#define PCI9118_CNT2	0x08		/* R/W: 8254 couter 0 */
#define PCI9118_CNTCTRL	0x0c		/* W:   8254 counter control */
#define PCI9118_AD_DATA	0x10		/* R:   A/D data */
#define PCI9118_DA1	0x10		/* W:   D/A registers */
#define PCI9118_DA2	0x14
#define PCI9118_ADSTAT	0x18		/* R:   A/D status register */
#define PCI9118_ADCNTRL	0x18		/* W:   A/D control register */
#define PCI9118_DI	0x1c		/* R:   digi input register */
#define PCI9118_DO	0x1c		/* W:   digi output register */
#define PCI9118_SOFTTRG	0x20		/* W:   soft trigger for A/D */
#define PCI9118_GAIN	0x24		/* W:   A/D gain/channel register */
#define PCI9118_BURST	0x28		/* W:   A/D burst number register */
#define PCI9118_SCANMOD	0x2c		/* W:   A/D auto scan mode */
#define PCI9118_ADFUNC	0x30		/* W:   A/D function register */
#define PCI9118_DELFIFO	0x34		/* W:   A/D data FIFO reset */
#define PCI9118_INTSRC	0x38		/* R:   interrupt reason register */
#define PCI9118_INTCTRL	0x38		/* W:   interrupt control register */

// bits from A/D control register (PCI9118_ADCNTRL)
#define AdControl_UniP	0x80		/* 1=bipolar, 0=unipolar */
#define AdControl_Diff	0x40		/* 1=differential, 0= single end inputs */
#define AdControl_SoftG	0x20		/* 1=8254 counter works, 0=counter stops */
#define	AdControl_ExtG	0x10		/* 1=8254 countrol controlled by TGIN(pin 46), 0=controled by SoftG */
#define AdControl_ExtM	0x08		/* 1=external hardware trigger (pin 44), 0=internal trigger */
#define AdControl_TmrTr	0x04		/* 1=8254 is iternal trigger source, 0=software trigger is source (register PCI9118_SOFTTRG) */
#define AdControl_Int	0x02		/* 1=enable INT, 0=disable */
#define AdControl_Dma	0x01		/* 1=enable DMA, 0=disable */

// bits from A/D function register (PCI9118_ADFUNC)
#define AdFunction_PDTrg	0x80		/* 1=positive, 0=negative digital trigger (only positive is correct) */
#define AdFunction_PETrg	0x40		/* 1=positive, 0=negative external trigger (only positive is correct) */
#define AdFunction_BSSH		0x20		/* 1=with sample&hold, 0=without */
#define AdFunction_BM		0x10		/* 1=burst mode, 0=normal mode */
#define AdFunction_BS		0x08		/* 1=burst mode start, 0=burst mode stop */
#define AdFunction_PM		0x04		/* 1=post trigger mode, 0=not post trigger */
#define AdFunction_AM		0x02		/* 1=about trigger mode, 0=not about trigger */
#define AdFunction_Start	0x01		/* 1=trigger start, 0=trigger stop */

// bits from A/D status register (PCI9118_ADSTAT)
#define AdStatus_nFull	0x100		/* 0=FIFO full (fatal), 1=not full */
#define AdStatus_nHfull	0x080		/* 0=FIFO half full, 1=FIFO not half full */
#define AdStatus_nEpty	0x040		/* 0=FIFO empty, 1=FIFO not empty */
#define AdStatus_Acmp	0x020		/*  */
#define AdStatus_DTH	0x010		/* 1=external digital trigger */
#define AdStatus_Bover	0x008		/* 1=burst mode overrun (fatal) */
#define AdStatus_ADOS	0x004		/* 1=A/D over speed (warning) */
#define AdStatus_ADOR	0x002		/* 1=A/D overrun (fatal) */
#define AdStatus_ADrdy	0x001		/* 1=A/D already ready, 0=not ready */

// bits for interrupt reason and control (PCI9118_INTSRC, PCI9118_INTCTRL)
// 1=interrupt occur, enable source,  0=interrupt not occur, disable source
#define Int_Timer	0x08		/* timer interrupt */
#define Int_About	0x04		/* about trigger complete */
#define Int_Hfull	0x02		/* A/D FIFO hlaf full */
#define Int_DTrg	0x01		/* external digital trigger */

#define START_AI_EXT	0x01		/* start measure on external trigger */
#define STOP_AI_EXT	0x02		/* stop measure on external trigger */

comedi_lrange range_pci9118dg_hr={ 8, {
	BIP_RANGE(5),
	BIP_RANGE(2.5),
	BIP_RANGE(1.25),
	BIP_RANGE(0.625),
	UNI_RANGE(10),
	UNI_RANGE(5),
	UNI_RANGE(2.5),
	UNI_RANGE(1.25)
	}
};

comedi_lrange range_pci9118hg={ 8, {
	BIP_RANGE(5),
	BIP_RANGE(0.5),
	BIP_RANGE(0.05),
	BIP_RANGE(0.005),
	UNI_RANGE(10),
	UNI_RANGE(1),
	UNI_RANGE(0.1),
	UNI_RANGE(0.01)
	}
};

#define PCI9118_BIPOLAR_RANGES	4	/* used for test on mixture of BIP/UNI ranges */

static int pci9118_attach(comedi_device *dev,comedi_devconfig *it);
static int pci9118_detach(comedi_device *dev);

static unsigned short	pci_list_builded=0;	/*=1 list of card is know */

typedef struct {
	char 		*name;		// driver name
	int		vendor_id;	// PCI vendor a device ID of card
	int		device_id;
	int		iorange_amcc;	// iorange for own S5933 region
	int		iorange_9118;	// pass thru card region size
	int 		n_aichan;	// num of A/D chans
	int 		n_aichand;	// num of A/D chans in diff mode
	int		mux_aichan;	// num of A/D chans with external multiplexor
	int		n_aichanlist;	// len of chanlist
	int 		n_aochan;	// num of D/A chans
	int		ai_maxdata;	// resolution of A/D
	int		ao_maxdata;	// resolution of D/A
	comedi_lrange	*rangelist_ai;	// rangelist for A/D
	comedi_lrange	*rangelist_ao;	// rangelist for D/A
	unsigned int	ai_ns_min;	// max sample speed of card v ns
	unsigned int	ai_pacer_min;	// minimal pacer value (c1*c2 or c1 in burst)
	
} boardtype;

static boardtype boardtypes[] =
{
	{"pci9118dg", PCI_VENDOR_ID_AMCC, 0x80d9,
	 AMCC_OP_REG_SIZE, IORANGE_9118,
	 16, 8, 256, PCI9118_CHANLEN, 2, 0x0fff, 0x0fff,
	 &range_pci9118dg_hr, &range_bipolar10,
	 3000, 12 },
	{"pci9118hg", PCI_VENDOR_ID_AMCC, 0x80d9,
	 AMCC_OP_REG_SIZE, IORANGE_9118,
	 16, 8, 256, PCI9118_CHANLEN, 2, 0x0fff, 0x0fff, 
	 &range_pci9118hg, &range_bipolar10,
	 3000, 12 },
	{"pci9118hr", PCI_VENDOR_ID_AMCC, 0x80d9,
	 AMCC_OP_REG_SIZE, IORANGE_9118,
	 16, 8, 256, PCI9118_CHANLEN, 2, 0xffff, 0x0fff, 
	 &range_pci9118dg_hr,    &range_bipolar10,
	 10000, 40 },
};

#define n_boardtypes (sizeof(boardtypes)/sizeof(boardtype))

comedi_driver driver_pci9118={
	driver_name:	"adl_pci9118",
	module:		THIS_MODULE,
	attach:		pci9118_attach,
	detach:		pci9118_detach,
	num_names:	n_boardtypes,
	board_name:	boardtypes,
	offset:		sizeof(boardtype),
};
COMEDI_INITCLEANUP(driver_pci9118);

typedef struct{
	int			iobase_a;	// base+size for AMCC chip
	struct pcilst_struct	*amcc;		// ptr too AMCC data
	unsigned int		master;		// master capable
	unsigned char		allocated;	// we have blocked card
	unsigned int		usemux;		// we want to use external multiplexor!
#ifdef PCL9118_PARANOIDCHECK
	unsigned short		chanlist[PCI9118_CHANLEN]; // list of scaned channel
	unsigned char		chanlistlen;	// number of scanlist
#endif
	unsigned char		AdControlReg;	// A/D control register
	unsigned char		IntControlReg;	// Interrupt control register
	unsigned char		AdFunctionReg;	// A/D function register
	char			valid;		// driver is ok
	char			neverending_ai;	// we do unlimited AI
	unsigned int		i8254_osc_base;	// frequence of onboard oscilator
	unsigned int		ai_do;		// what do AI? 0=nothing, 1 to 4 mode
	unsigned int		ai1234_act_scan;// how many scans we finished
	unsigned int 		ai1234_buf_ptr;	// data buffer ptr in samples
	unsigned int 		ai1234_n_chan;// how many channels is measured
	unsigned int 		ai1234_n_scanlen;// len of actual scanlist
	unsigned int 		ai1234_act_scanpos; // position in actual scan
	unsigned int		*ai1234_chanlist;// actaul chanlist
	unsigned int		ai1234_timer1;	
	unsigned int		ai1234_timer2;
	unsigned int		ai1234_flags;
	char			ai12_startstop;	// measure can start/stop on external trigger
	unsigned int		ai1234_divisor1,ai1234_divisor2; // divisors for start of measure on external start
	unsigned int		ai1234_data_len;
	sampl_t 		*ai1234_data;
	sampl_t			ao_data[2];	// data output buffer
	unsigned int		ai1234_scans;	// number of scans to do
	unsigned char		ai4_status;	// 0=wait for ext trg 1=sampling
	unsigned char		ai4_cntrl0;	// control register for status 0 and 1
	unsigned char		ai4_cntrl1;
	char			dma_doublebuf;	// we can use double buffring
	unsigned int		dma_actbuf;	// which buffer is used now
	unsigned long 		dmabuf_virt[2];	// pointers to begin of DMA buffer
	unsigned long 		dmabuf_hw[2];	// hw address of DMA buff
	unsigned int		dmabuf_size[2];	// size of dma buffer in bytes
	unsigned int		dmabuf_use_size[2];	// which size e may now used for transfer
	unsigned int		dmabuf_samples[2];	// size in samples
	int			dmabuf_pages[2];	// number of pages in buffer
	unsigned char		cnt0_users;	// bit field of 8254 CNT0 users (0-unused, 1-AO, 2-DI, 3-DO)
	unsigned char		exttrg_users;	// bit field of external trigger users (0-AI, 1-AO, 2-DI, 3-DO)
	unsigned int		cnt0_divisor;	// actual CNT0 divisor
}pci9118_private;

#define devpriv ((pci9118_private *)dev->private)
#define this_board ((boardtype *)dev->board_ptr)

/*
==============================================================================
*/

int check_channel_list(comedi_device * dev, comedi_subdevice * s, int n_chan,
	unsigned int *chanlist);
int setup_channel_list(comedi_device * dev, comedi_subdevice * s, int n_chan,
	unsigned int *chanlist,int rot);
void start_pacer(comedi_device * dev, int mode, unsigned int divisor1, unsigned int divisor2);
static int pci9118_reset(comedi_device *dev);
int pci9118_exttrg_add(comedi_device * dev, unsigned char source);
int pci9118_exttrg_del(comedi_device * dev, unsigned char source);
int pci9118_ai_cancel(comedi_device * dev, comedi_subdevice * s);

/* 
==============================================================================
*/
static void interrupt_pci9118_ai_mode4_switch(void *d) 
{
	comedi_device 	*dev = d;

	if (devpriv->ai4_status) {
		pci9118_exttrg_add(dev,0);		// activate EXT trigger
		devpriv->ai4_status=0;			// next int start pacer!
		outl(inl(devpriv->iobase_a+AMCC_OP_REG_INTCSR)&(~AINT_WRITE_COMPL), devpriv->iobase_a+AMCC_OP_REG_INTCSR);	// stop dma irq
		outl(devpriv->ai4_cntrl0, dev->iobase+PCI9118_ADCNTRL);
		start_pacer(dev, 0, 0, 0);		// stop pacer
	} else {
		pci9118_exttrg_del(dev,0);		// deactivate EXT trigger
		outl(inl(devpriv->iobase_a+AMCC_OP_REG_INTCSR)|(AINT_WRITE_COMPL), devpriv->iobase_a+AMCC_OP_REG_INTCSR);	// enable dma irq
		//outw(0, dev->iobase+PCI9118_SOFTTRG); /* start 1. conversion */
  		devpriv->ai4_status++;			// next int wi will have data!
		outl(devpriv->ai4_cntrl1, dev->iobase+PCI9118_ADCNTRL);
		start_pacer(dev, 4, devpriv->ai1234_divisor1, devpriv->ai1234_divisor2);	// start pacer
	}
}

/* 
==============================================================================
*/
static void move_block_from_dma_12bit(comedi_device *dev,comedi_subdevice *s,sampl_t *dma,sampl_t *data,int n)
{
	unsigned int i,j,m;

	j=s->async->cur_chan;
	m=devpriv->ai1234_act_scanpos;
	for(i=0;i<n;i++){
#ifdef PCL9118_PARANOIDCHECK
		if ((*dma & 0x0f00)!=devpriv->chanlist[j]) { // data dropout!
	    		rt_printk("comedi: A/D  DMA - data dropout: received channel %d, expected %d!\n",(*dma & 0x0f00)>>8,devpriv->chanlist[j]>>8);
	    		pci9118_ai_cancel(dev,s);
			comedi_error_done(dev,s);
			return;
		}
#endif
		*data=((*dma & 0xff)<<4)|((*dma & 0xf000)>>12); // get one sample
		data++; dma++; 
		j++;
		if(j>=devpriv->ai1234_n_chan) {
			m+=j;
			j=0;
			if(m>=devpriv->ai1234_n_scanlen) {
				m=0;
			        devpriv->ai1234_act_scan++;
				if (devpriv->ai1234_flags & TRIG_WAKE_EOS) 
					comedi_eos(dev,s);  
			}
		}
	}
	devpriv->ai1234_act_scanpos=m;
	s->async->cur_chan=j;
}

/* 
==============================================================================
*/
static void move_block_from_dma_16bit(comedi_device *dev,comedi_subdevice *s,sampl_t *dma,sampl_t *data,int n)
{
	int i,j,m;

	j=s->async->cur_chan;
	m=devpriv->ai1234_act_scanpos;
	for(i=0;i<n;i++){
		*data=((*dma & 0xff)<<8)|((*dma & 0xff00)>>8); // get one sample
		data++; dma++;
		j++;
		if(j>=devpriv->ai1234_n_chan) {
			m+=j;
			j=0;
			if(m>=devpriv->ai1234_n_scanlen) {
				m=0;
			        devpriv->ai1234_act_scan++;
				if (devpriv->ai1234_flags & TRIG_WAKE_EOS) 
					comedi_eos(dev,s);  
			}
		}
	}
	devpriv->ai1234_act_scanpos=m;
	s->async->cur_chan=j;
}

/* 
==============================================================================
*/
static void interrupt_pci9118_ai_dma(int irq, void *d, struct pt_regs *regs) 
{
	comedi_device 	*dev = d;
	comedi_subdevice *s = dev->subdevices + 0;
        sampl_t 	*ptr;
	unsigned int	next_dma_buf, samplesinbuf, m;

	if (devpriv->ai_do==4) 
		interrupt_pci9118_ai_mode4_switch(d); 
	
	samplesinbuf=devpriv->dmabuf_use_size[devpriv->dma_actbuf]-inl(devpriv->iobase_a+AMCC_OP_REG_MWTC);	// how many bytes is in DMA buffer?

	if (samplesinbuf<devpriv->dmabuf_use_size[devpriv->dma_actbuf]) {
		comedi_error(dev,"Interrupted DMA transfer!");
	}

	if (samplesinbuf & 1) {
		comedi_error(dev,"Odd count of bytes in DMA ring!");
	        pci9118_ai_cancel(dev,s);
		comedi_error_done(dev,s);
		return;
	}

	m=inw(dev->iobase+PCI9118_ADSTAT);
	if (m & 0x10e) {
		if (m & 0x100)
			comedi_error(dev,"A/D FIFO Full status (Fatal Error!)");
		if (m & 0x008)
			comedi_error(dev,"A/D Burst Mode Overrun Status (Fatal Error!)");
		if (m & 0x004)
			comedi_error(dev,"A/D Over Speed Status (Warning!)");
		if (m & 0x002)
			comedi_error(dev,"A/D Overrun Status (Fatal Error!)");
		if (m & 0x10a) {
		        pci9118_ai_cancel(dev,s);
			comedi_error_done(dev,s);
			return;
		}
	}

	samplesinbuf=samplesinbuf>>1;	// number of received samples
		
	if (devpriv->dma_doublebuf) {	// switch DMA buffers if is used double buffering
    		next_dma_buf=1-devpriv->dma_actbuf;
		outl(devpriv->dmabuf_hw[next_dma_buf], devpriv->iobase_a+AMCC_OP_REG_MWAR);
		outl(devpriv->dmabuf_use_size[next_dma_buf], devpriv->iobase_a+AMCC_OP_REG_MWTC);
	}
	
        ptr=(sampl_t *)devpriv->dmabuf_virt[devpriv->dma_actbuf];

	if(s->async->buf_int_ptr+samplesinbuf*sizeof(sampl_t)>=devpriv->ai1234_data_len){
		m=(devpriv->ai1234_data_len-s->async->buf_int_ptr)/sizeof(sampl_t);
		if (this_board->ai_maxdata==0xfff) { move_block_from_dma_12bit(dev,s,(void *)ptr,((void *)(devpriv->ai1234_data))+s->async->buf_int_ptr,m); }
					    else   { move_block_from_dma_16bit(dev,s,(void *)ptr,((void *)(devpriv->ai1234_data))+s->async->buf_int_ptr,m); }
		s->async->buf_int_count+=m*sizeof(sampl_t);
		ptr+=m*sizeof(sampl_t);
		samplesinbuf-=m;
		s->async->buf_int_ptr=0;

		comedi_eobuf(dev,s);
	}
	
	if (samplesinbuf) {
		if (this_board->ai_maxdata==0xfff) { move_block_from_dma_12bit(dev,s,(void *)ptr,((void *)(devpriv->ai1234_data))+s->async->buf_int_ptr,samplesinbuf); }
					    else   { move_block_from_dma_16bit(dev,s,(void *)ptr,((void *)(devpriv->ai1234_data))+s->async->buf_int_ptr,samplesinbuf); }
		s->async->buf_int_count+=samplesinbuf*sizeof(sampl_t);
		s->async->buf_int_ptr+=samplesinbuf*sizeof(sampl_t);
		if (!(devpriv->ai1234_flags & TRIG_WAKE_EOS)) {
			comedi_bufcheck(dev,s);
		}
	}

	if (!devpriv->neverending_ai)
    		if ( devpriv->ai1234_act_scan>=devpriv->ai1234_scans ) { /* all data sampled */
		        pci9118_ai_cancel(dev,s);
			comedi_done(dev,s); 
			return;
		}
	

	if (devpriv->dma_doublebuf) { // switch dma buffers
		devpriv->dma_actbuf=1-devpriv->dma_actbuf; 
	} else { // restart DMA if is not used double buffering
		outl(devpriv->dmabuf_hw[0], devpriv->iobase_a+AMCC_OP_REG_MWAR);
		outl(devpriv->dmabuf_use_size[0], devpriv->iobase_a+AMCC_OP_REG_MWTC);
	}
}

/* 
==============================================================================
*/
static void interrupt_pci9118(int irq, void *d, struct pt_regs *regs) 
{
	comedi_device *dev = d;
	unsigned int	int_daq,int_amcc,int_adstat;
		
	int_daq=inl(dev->iobase+PCI9118_INTSRC) & 0xf;		// get IRQ reasons
	int_amcc=inl(devpriv->iobase_a+AMCC_OP_REG_INTCSR);	// get AMCC INT register

#if 0
	rt_printk("INT daq=0x%01x amcc=0x%08x MWAR=0x%08x MWTC=0x%08x ADSTAT=0x%02x\n", int_daq, int_amcc,
		inl(devpriv->iobase_a+AMCC_OP_REG_MWAR), inl(devpriv->iobase_a+AMCC_OP_REG_MWTC),
		int_adstat);
#endif

	if ((!int_daq)&&(!(int_amcc&ANY_S593X_INT))) {
#if 0
    		comedi_error(dev,"IRQ from unknow source");
#endif
		return;
	}
	outl(int_amcc|0x00ff0000, devpriv->iobase_a+AMCC_OP_REG_INTCSR);	// shutdown IRQ reasons in AMCC

	if (int_amcc&MASTER_ABORT_INT)
    		comedi_error(dev,"AMCC IRQ - MASTER DMA ABORT!");
	if (int_amcc&TARGET_ABORT_INT)
    		comedi_error(dev,"AMCC IRQ - TARGET DMA ABORT!");


	if (devpriv->ai_do) {

		int_adstat=inw(dev->iobase+PCI9118_ADSTAT)&0x1ff;	// get STATUS register

		if ((int_adstat&AdStatus_DTH)&&(int_daq&Int_DTrg)&&(devpriv->ai12_startstop)) {	// start stop of measure
			if (devpriv->ai12_startstop&START_AI_EXT) {
				devpriv->ai12_startstop&=~START_AI_EXT;
				if (!(devpriv->ai12_startstop&STOP_AI_EXT))
					pci9118_exttrg_del(dev,0);		// deactivate EXT trigger
				start_pacer(dev, devpriv->ai_do, devpriv->ai1234_divisor1, devpriv->ai1234_divisor2);	// start pacer
			} else {
				if (devpriv->ai12_startstop&STOP_AI_EXT) {
					devpriv->ai12_startstop&=~STOP_AI_EXT;
					pci9118_exttrg_del(dev,0);		// deactivate EXT trigger
					devpriv->neverending_ai=0;
				}
			}
		}

		if (int_amcc&WRITE_TC_INT)
			if (devpriv->master) {
				interrupt_pci9118_ai_dma(irq,d,regs); // do some data transfer
			}

		if ((int_adstat&AdStatus_DTH)&&(int_daq&Int_DTrg)&&(int_amcc&IN_MB_INT)&&(devpriv->ai_do==4)) {
			interrupt_pci9118_ai_mode4_switch(d); // AI mode 4, scan begin EXT interrupt
		}
	}
	
}

/* 
==============================================================================
*/
static int pci9118_ai_docmd_and_mode(int mode, comedi_device * dev, comedi_subdevice * s, char startstop) 
{
        unsigned int divisor1, divisor2;
	unsigned int	dmalen0,dmalen1;
	char	use_bssh=0;
	
	start_pacer(dev, -1, 0, 0); // stop pacer

	devpriv->AdControlReg=0; 	// bipolar, S.E., use 8254, stop 8354, internal trigger, soft trigger, disable DMA

	if (!check_channel_list(dev, s, devpriv->ai1234_n_chan,
		devpriv->ai1234_chanlist)) return -EINVAL;
	if (!setup_channel_list(dev, s, devpriv->ai1234_n_chan,
		devpriv->ai1234_chanlist, 8)) return -EINVAL;

	outl(devpriv->AdControlReg,dev->iobase+PCI9118_ADCNTRL);
	devpriv->AdFunctionReg=AdFunction_PDTrg|AdFunction_PETrg;	// positive triggers, no S&H, no burst, burst stop, no post trigger, no about trigger, trigger stop
	outl(devpriv->AdFunctionReg,dev->iobase+PCI9118_ADFUNC);
	udelay(1);
	outl(0,dev->iobase+PCI9118_DELFIFO); // flush FIFO
	inl(dev->iobase+PCI9118_ADSTAT); // flush A/D status register
	inl(dev->iobase+PCI9118_INTSRC);

        devpriv->ai1234_act_scan=0;
	devpriv->ai1234_act_scanpos=0;
        s->async->cur_chan=0;
        devpriv->ai1234_buf_ptr=0;
        devpriv->neverending_ai=0;
	devpriv->ai12_startstop=startstop;
	devpriv->dma_actbuf=0;
	if ((devpriv->ai1234_scans==0)||(devpriv->ai1234_scans==-1)) devpriv->neverending_ai=1; //well, user want neverending
	if (startstop&STOP_AI_EXT)
		devpriv->neverending_ai=1;	// stop on external account

  
        switch (mode) {
	case 1:
		if (devpriv->ai1234_timer1<this_board->ai_ns_min) devpriv->ai1234_timer1=this_board->ai_ns_min;
		i8253_cascade_ns_to_timer(devpriv->i8254_osc_base,&divisor1,&divisor2,&devpriv->ai1234_timer1,TRIG_ROUND_NEAREST);
		// rt_printk("OSC base=%u div1=%u div2=%u timer=%u\n",devpriv->i8254_osc_base,divisor1,divisor2,devpriv->ai1234_timer1);
		break;
	case 2:
		if (devpriv->ai1234_timer2==0) use_bssh=1;	// use S&H
		if (devpriv->ai1234_timer2<this_board->ai_ns_min) devpriv->ai1234_timer2=this_board->ai_ns_min;
		divisor1=(devpriv->ai1234_timer2+devpriv->i8254_osc_base/2)/devpriv->i8254_osc_base; // minor timer
		if (divisor1<this_board->ai_pacer_min) divisor1=this_board->ai_pacer_min;
		divisor2=(devpriv->ai1234_timer1+devpriv->i8254_osc_base/2)/devpriv->i8254_osc_base;
		divisor2=divisor2/divisor1;						// major timer is c1*c2
		if (divisor2<devpriv->ai1234_n_chan) divisor2=devpriv->ai1234_n_chan;
		if (use_bssh) 
			if (divisor2<(devpriv->ai1234_n_chan+2))
				 divisor2=devpriv->ai1234_n_chan+2;
		devpriv->ai1234_timer2=divisor1*devpriv->i8254_osc_base;
		devpriv->ai1234_timer1=divisor1*divisor2*devpriv->i8254_osc_base;
		break;
	case 4:
		if (devpriv->ai1234_timer2<this_board->ai_ns_min) devpriv->ai1234_timer2=this_board->ai_ns_min;
		i8253_cascade_ns_to_timer(devpriv->i8254_osc_base,&divisor1,&divisor2,&devpriv->ai1234_timer2,TRIG_ROUND_NEAREST);
		devpriv->ai4_status=0;
		break;
	}
   
	devpriv->ai1234_divisor1=divisor1;
	devpriv->ai1234_divisor2=divisor2;
		
        if (devpriv->master) {  // bus master DMA
		
		dmalen0=devpriv->dmabuf_size[0];
		dmalen1=devpriv->dmabuf_size[1];
		
		if (!devpriv->neverending_ai) {
			if (dmalen0>(devpriv->ai1234_scans*devpriv->ai1234_n_scanlen*2)) {	// must we fill full first buffer?
				dmalen0=devpriv->ai1234_scans*devpriv->ai1234_n_scanlen*2;
			} else
				if (dmalen1>(devpriv->ai1234_scans*devpriv->ai1234_n_scanlen*2-dmalen0)) 	// and must we fill full second buffer when first is once filled?
					dmalen1=devpriv->ai1234_scans*devpriv->ai1234_n_scanlen*2-dmalen0;
		}

		if (devpriv->ai1234_flags & TRIG_WAKE_EOS) {	// don't we want wake up every scan?
			if (dmalen0>(devpriv->ai1234_n_scanlen*2)) {
				dmalen0=devpriv->ai1234_n_scanlen*2;
				if (devpriv->ai1234_n_scanlen&1) dmalen0+=2;
			}
			if (dmalen1>(devpriv->ai1234_n_scanlen*2)) {
				dmalen1=devpriv->ai1234_n_scanlen*2;
				if (devpriv->ai1234_n_scanlen&1) dmalen1-=2;
				if (dmalen1<4) dmalen1=4;
			}
		} else {				// isn't output buff smaller that our DMA buff?
			if (dmalen0>(devpriv->ai1234_data_len)) {
				dmalen0=devpriv->ai1234_data_len;
			}
			if (dmalen1>(devpriv->ai1234_data_len)) {
				dmalen1=devpriv->ai1234_data_len;
			}
		}
		
		devpriv->dmabuf_use_size[0]=dmalen0;
		devpriv->dmabuf_use_size[1]=dmalen1;
		outl(devpriv->dmabuf_hw[0], devpriv->iobase_a+AMCC_OP_REG_MWAR);
		outl(devpriv->dmabuf_use_size[0], devpriv->iobase_a+AMCC_OP_REG_MWTC);

		outl(0x02000000|AINT_WRITE_COMPL, devpriv->iobase_a+AMCC_OP_REG_INTCSR);
		outl(inl(devpriv->iobase_a+AMCC_OP_REG_MCSR)|RESET_A2P_FLAGS|A2P_HI_PRIORITY|EN_A2P_TRANSFERS, devpriv->iobase_a+AMCC_OP_REG_MCSR);

		switch (mode) {
		case 1: 
			devpriv->AdControlReg|=((AdControl_SoftG|AdControl_TmrTr|AdControl_Dma) & 0xff);
			break;
		case 2: 
			devpriv->AdControlReg|=((AdControl_SoftG|AdControl_TmrTr|AdControl_Dma) & 0xff);
			devpriv->AdFunctionReg|=AdFunction_BM;
			if (use_bssh) {	outl(devpriv->ai1234_n_chan+1, dev->iobase+PCI9118_BURST); devpriv->AdFunctionReg|=AdFunction_BSSH; }
				else  {	outl(devpriv->ai1234_n_chan, dev->iobase+PCI9118_BURST); }
			break;
		case 3: 
			devpriv->AdControlReg|=((AdControl_ExtM|AdControl_Dma) & 0xff);
			break;
		case 4: 
			devpriv->ai4_cntrl1=devpriv->AdControlReg|((AdControl_SoftG|AdControl_TmrTr|AdControl_Dma) & 0xff);
			devpriv->AdControlReg|=((AdControl_Int) & 0xff);
			devpriv->ai4_cntrl0=devpriv->AdControlReg;
			pci9118_exttrg_add(dev,0);	// activate EXT trigger
			break;
		}; 
		devpriv->ai_do=mode;

		if (devpriv->ai12_startstop) {
			pci9118_exttrg_add(dev,0);	// activate EXT trigger
			devpriv->AdControlReg|=AdControl_Int;
		}
			
		outl(devpriv->AdFunctionReg,dev->iobase+PCI9118_ADFUNC);
		outl(devpriv->AdControlReg, dev->iobase+PCI9118_ADCNTRL);

		if (!(devpriv->ai12_startstop&START_AI_EXT)) 
			if ((mode==1)||(mode==2))
    				start_pacer(dev, mode, divisor1, divisor2);

		if (mode==2) {
			devpriv->AdFunctionReg=AdFunction_PDTrg|AdFunction_PETrg|AdFunction_BM|AdFunction_BS;
			if (use_bssh) devpriv->AdFunctionReg|=AdFunction_BSSH;
			outl(devpriv->AdFunctionReg,dev->iobase+PCI9118_ADFUNC);
		}

        } else {
	         return -EINVAL;
	}

	return 0;
}


/*
==============================================================================
*/
int pci9118_insn_read_ai(comedi_device *dev,comedi_subdevice *s, comedi_insn *insn,lsampl_t *data)
{

	int n,timeout;

	devpriv->AdControlReg=AdControl_Int & 0xff;
	devpriv->AdFunctionReg=AdFunction_PDTrg|AdFunction_PETrg;
	outl(devpriv->AdFunctionReg,dev->iobase+PCI9118_ADFUNC);// positive triggers, no S&H, no burst, burst stop, no post trigger, no about trigger, trigger stop

	if (!setup_channel_list(dev,s,1,&insn->chanspec, 0))  return -EINVAL;

	outl(0,dev->iobase+PCI9118_DELFIFO); // flush FIFO

	for (n=0; n<insn->n; n++) {
		outw(0, dev->iobase+PCI9118_SOFTTRG); /* start conversion */
		udelay(2);
    		timeout=100;
    		while (timeout--) {
			if (inl(dev->iobase+PCI9118_ADSTAT) & AdStatus_ADrdy) goto conv_finish;
			udelay(1);
    		}

    		comedi_error(dev,"A/D insn timeout");
    		insn->data[n]=0;
		outl(0,dev->iobase+PCI9118_DELFIFO); // flush FIFO
    		return -ETIME;

conv_finish:
		if (this_board->ai_maxdata==0xfff) {
    			data[n] = (inw(dev->iobase+PCI9118_AD_DATA)>>4) & 0xfff; 
		} else {
    			data[n] = inl(dev->iobase+PCI9118_AD_DATA) & 0xffff;
		}
	}

	outl(0,dev->iobase+PCI9118_DELFIFO); // flush FIFO
	return n;

}

/*
==============================================================================
*/
int pci9118_insn_write_ao(comedi_device *dev,comedi_subdevice *s, comedi_insn *insn,lsampl_t *data)
{
	int n,chanreg,ch;
	
	ch=CR_CHAN(insn->chanspec);
	if (ch) { chanreg=PCI9118_DA2;} 
	    else { chanreg=PCI9118_DA1; } 

	for (n=0; n<insn->n; n++) {
		outl(data[n], dev->iobase + chanreg);
		devpriv->ao_data[ch]=data[n];
	}

	return n;
}

/* 
==============================================================================
*/
int pci9118_insn_read_ao(comedi_device * dev, comedi_subdevice * s, comedi_insn *insn, lsampl_t *data) 
{
	int n,chan;
	
	chan=CR_CHAN(insn->chanspec);
	for (n=0; n<insn->n; n++) 
		data[n]=devpriv->ao_data[chan];

	return n;
}

/*
==============================================================================
*/
int pci9118_insn_bits_di(comedi_device *dev,comedi_subdevice *s, comedi_insn *insn,lsampl_t *data)
{
	data[1] = inl(dev->iobase + PCI9118_DI) & 0xf;

	return 2;
}

/*
==============================================================================
*/
int pci9118_insn_bits_do(comedi_device *dev,comedi_subdevice *s, comedi_insn *insn,lsampl_t *data)
{
	if(data[0]){
		s->state &= ~data[0];
		s->state |= (data[0]&data[1]);
		outl(s->state & 0x0f, dev->iobase + PCI9118_DO);
	}
	data[1] = s->state;

	return 2;
}

/*
==============================================================================
*/
static int pci9118_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err=0;
	int tmp,divisor1,divisor2;

	/* step 1: make sure trigger sources are trivially valid */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW|TRIG_EXT;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER|TRIG_EXT|TRIG_FOLLOW;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER|TRIG_EXT;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT|TRIG_NONE|TRIG_EXT;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err) return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->start_src!=TRIG_NOW &&
	   cmd->start_src!=TRIG_EXT) {
		err++;
	}

	if(cmd->scan_begin_src!=TRIG_TIMER &&
	   cmd->scan_begin_src!=TRIG_EXT &&
	   cmd->scan_begin_src!=TRIG_FOLLOW) err++;

	if(cmd->convert_src!=TRIG_TIMER &&
	   cmd->convert_src!=TRIG_EXT) err++;

	if((cmd->scan_begin_src&(TRIG_TIMER|TRIG_EXT)) &&
	   cmd->convert_src!=TRIG_TIMER) {
		cmd->convert_src=TRIG_TIMER;
		err++;
	}
		
	if(cmd->scan_end_src!=TRIG_COUNT) {
		cmd->scan_end_src=TRIG_COUNT;
		err++;
	}

	if(cmd->stop_src!=TRIG_NONE &&
	   cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_EXT) err++;

	if(cmd->start_src==TRIG_EXT &&
	  cmd->scan_begin_src==TRIG_EXT) {
		cmd->start_src=TRIG_NOW;
		err++;
	}

	if(cmd->stop_src==TRIG_EXT &&
	  cmd->scan_begin_src==TRIG_EXT) err++;

	if(err) { return 2;}

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}
	if(cmd->scan_begin_src==TRIG_TIMER){
		if(cmd->scan_begin_arg<this_board->ai_ns_min){
			cmd->scan_begin_arg=this_board->ai_ns_min;
			err++;
		}
	}
	if(cmd->convert_src==TRIG_TIMER){
		if (cmd->scan_begin_src==TRIG_TIMER) {
			if((cmd->convert_arg)&&(cmd->convert_arg<this_board->ai_ns_min)){  // convert_arg=0 -> use S&H
				cmd->convert_arg=this_board->ai_ns_min;
				err++;
			}		
		} else {
			if(cmd->convert_arg<this_board->ai_ns_min){
				cmd->convert_arg=this_board->ai_ns_min;
				err++;

			}
		}
	}

	if(!cmd->chanlist_len){
		cmd->chanlist_len=1;
		err++;
	}
	if(cmd->chanlist_len>this_board->n_aichanlist){
		cmd->chanlist_len=this_board->n_aichanlist;
		err++;
	}
	if(cmd->scan_end_arg<cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	if((cmd->scan_end_arg % cmd->chanlist_len)){
		cmd->scan_end_arg=cmd->scan_end_arg/cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src==TRIG_COUNT){
		if(!cmd->stop_arg){
			cmd->stop_arg=1;
			err++;
		}
	} else { /* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err) return 3;

	/* step 4: fix up any arguments */

	if(cmd->scan_begin_src==TRIG_TIMER){
		tmp=cmd->scan_begin_arg;
		i8253_cascade_ns_to_timer(devpriv->i8254_osc_base,&divisor1,&divisor2,&cmd->scan_begin_arg,cmd->flags&TRIG_ROUND_MASK);
		if(cmd->scan_begin_arg<this_board->ai_ns_min)
			cmd->scan_begin_arg=this_board->ai_ns_min;
		if(tmp!=cmd->scan_begin_arg)err++;
	} 
	if(cmd->convert_src==TRIG_TIMER){
		tmp=cmd->convert_arg;
		i8253_cascade_ns_to_timer(devpriv->i8254_osc_base,&divisor1,&divisor2,&cmd->convert_arg,cmd->flags&TRIG_ROUND_MASK);
		if(cmd->scan_begin_arg<this_board->ai_ns_min)
			cmd->scan_begin_arg=this_board->ai_ns_min;
		if(tmp!=cmd->convert_arg)err++;
		if(cmd->scan_begin_src==TRIG_TIMER) {
			if (cmd->convert_arg==0) {
				if (cmd->scan_begin_arg<this_board->ai_ns_min*(cmd->scan_end_arg+2)) {
					cmd->scan_begin_arg=this_board->ai_ns_min*(cmd->scan_end_arg+2);
					err++;
				}
			} else {
	    			if (cmd->scan_begin_arg<cmd->convert_arg*cmd->scan_end_arg){
					cmd->scan_begin_arg=cmd->convert_arg*cmd->scan_end_arg;
					err++;
				}
			}
		}
	}

	if(err) return 4;

	if (cmd->chanlist)
		if (!check_channel_list(dev, s, cmd->chanlist_len,
			cmd->chanlist)) return 5; // incorrect channels list

	return 0;
}

/*
==============================================================================
*/
static int pci9118_ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	comedi_cmd *cmd=&s->async->cmd;
	char startstop;
	
	startstop=0;
	devpriv->ai1234_flags=cmd->flags;
	devpriv->ai1234_n_chan=cmd->chanlist_len;
	devpriv->ai1234_n_scanlen=cmd->scan_end_arg;
	devpriv->ai1234_chanlist=cmd->chanlist;
	devpriv->ai1234_data=s->async->data;
	devpriv->ai1234_data_len=s->async->data_len;
	if (cmd->stop_src==TRIG_COUNT) { devpriv->ai1234_scans=cmd->stop_arg; }
				else   { devpriv->ai1234_scans=0; }
	devpriv->ai1234_timer1=0;
	devpriv->ai1234_timer2=0;
	if (cmd->start_src==TRIG_EXT) startstop|=START_AI_EXT;
	if (cmd->stop_src==TRIG_EXT) startstop|=STOP_AI_EXT;
	
	if(cmd->scan_begin_src==TRIG_FOLLOW){ // mode 1 or 3
		if (cmd->convert_src==TRIG_TIMER) { // mode 1
			devpriv->ai1234_timer1=cmd->convert_arg;
			return pci9118_ai_docmd_and_mode(1,dev,s,startstop);
		}
		if (cmd->convert_src==TRIG_EXT) { // mode 3
			return pci9118_ai_docmd_and_mode(3,dev,s,startstop);
		}
	}

	if((cmd->scan_begin_src==TRIG_TIMER)&&(cmd->convert_src==TRIG_TIMER)){ // mode 2
		devpriv->ai1234_timer1=cmd->scan_begin_arg;
		devpriv->ai1234_timer2=cmd->convert_arg;
		return pci9118_ai_docmd_and_mode(2,dev,s,startstop);
	}
		
	if((cmd->scan_begin_src==TRIG_EXT)&&(cmd->convert_src==TRIG_TIMER)){ // mode 4
		devpriv->ai1234_timer2=cmd->convert_arg;
		return pci9118_ai_docmd_and_mode(4,dev,s,startstop);
	}
		
        return -1;
}

/*
==============================================================================
*/
int check_channel_list(comedi_device * dev, comedi_subdevice * s,
	int n_chan, unsigned int *chanlist)
{
        unsigned int i, differencial=0, bipolar=0;
	    
	/* correct channel and range number check itself comedi/range.c */
	if (n_chan<1) {
		comedi_error(dev,"range/channel list is empty!");
		return 0;             
        }

	if (CR_AREF(chanlist[0])==AREF_DIFF)
		differencial=1;  // all input must be diff
	if (CR_RANGE(chanlist[0])<PCI9118_BIPOLAR_RANGES)
		bipolar=1;  // all input must be bipolar
        if (n_chan > 1)
		for (i=1 ; i<n_chan; i++) { // check S.E/diff
			if ((CR_AREF(chanlist[i])==AREF_DIFF)!=(differencial)) {
			    	comedi_error(dev,"Differencial and single ended inputs cann't be mixtured!");
				return 0;             
			}
			if ((CR_RANGE(chanlist[i])<PCI9118_BIPOLAR_RANGES)!=(bipolar)) {
			    	comedi_error(dev,"Bipolar and unipolar ranges cann't be mixtured!");
				return 0;             
			}
			if ((!devpriv->usemux)&(differencial)&(CR_CHAN(chanlist[i])>=this_board->n_aichand)) {
			    	comedi_error(dev,"If AREF_DIFF is used then is available only 8 channels!");
				return 0;             
			}
		}
		
	return 1;
}

int setup_channel_list(comedi_device * dev, comedi_subdevice * s, int n_chan,
	unsigned int *chanlist,int rot)
{
        unsigned int i, differencial=0, bipolar=0;
	unsigned int scanquad, gain;

	if (CR_AREF(chanlist[0])==AREF_DIFF)
		differencial=1;  // all input must be diff
	if (CR_RANGE(chanlist[0])<PCI9118_BIPOLAR_RANGES)
		bipolar=1;  // all input must be bipolar

	// All is ok, so we can setup channel/range list

	if (!bipolar) {
	    devpriv->AdControlReg|=AdControl_UniP;	// enable bipolar
	} else {
	    devpriv->AdControlReg&=((~AdControl_UniP)&0xff);	// set unibipolar
	}

	if (differencial) {
	    devpriv->AdControlReg|=AdControl_UniP;	// enable diff inputs
	} else {
	    devpriv->AdControlReg&=((~AdControl_Diff)&0xff);	// set single ended inputs
	}

        outl(devpriv->AdControlReg, dev->iobase+PCI9118_ADCNTRL); // setup mode

	outl(2,dev->iobase+PCI9118_SCANMOD); // gods know why this sequence (disasembled from DOS driver)!
	outl(0,dev->iobase+PCI9118_SCANMOD);
	outl(1,dev->iobase+PCI9118_SCANMOD);
	
#ifdef PCL9118_PARANOIDCHECK
	devpriv->chanlistlen=n_chan;
#endif
	
	for (i=0; i<n_chan; i++) {  // store range list to card
		scanquad=CR_CHAN(chanlist[i]);	// get channel number;
#ifdef PCL9118_PARANOIDCHECK
		devpriv->chanlist[i]=(scanquad & 0xf)<<rot;
#endif
		gain=CR_RANGE(chanlist[i]);		// get gain number
		scanquad|=((gain & 0x03)<<8);
		outl(scanquad,dev->iobase+PCI9118_GAIN);
	}

	outl(0,dev->iobase+PCI9118_SCANMOD);	// close scan queue
//	udelay(100);				// important delay, or first sample will be cripled

	return 1; // we can serve this with scan logic
}


/*
==============================================================================
*/
void start_pacer(comedi_device * dev, int mode, unsigned int divisor1, unsigned int divisor2) 
{
        outl(0x74, dev->iobase + PCI9118_CNTCTRL);
	outl(0x30, dev->iobase + PCI9118_CNTCTRL);
        udelay(1);
  
        if ((mode==1)||(mode==2)||(mode==4)) {
		outl(divisor2 & 0xff, dev->iobase + PCI9118_CNT2);
		outl((divisor2 >> 8) & 0xff, dev->iobase + PCI9118_CNT2);
		outl(divisor1  & 0xff, dev->iobase + PCI9118_CNT1);
		outl((divisor1 >> 8) & 0xff, dev->iobase + PCI9118_CNT1);
        }
}

/*
==============================================================================
*/
int pci9118_exttrg_add(comedi_device * dev, unsigned char source)
{
	if (source>3) return -1; // incorrect source
	devpriv->exttrg_users|=(1<<source);
	devpriv->IntControlReg|=Int_DTrg;
	outl(devpriv->IntControlReg,dev->iobase+PCI9118_INTCTRL);
	outl(inl(devpriv->iobase_a+AMCC_OP_REG_INTCSR)|0x1f00, devpriv->iobase_a+AMCC_OP_REG_INTCSR); // allow INT in AMCC
	return 0;
}

/* 
==============================================================================
*/
int pci9118_exttrg_del(comedi_device * dev, unsigned char source)
{
	if (source>3) return -1; // incorrect source
	devpriv->exttrg_users&=~(1<<source);
	if (!devpriv->exttrg_users) { // shutdown ext trg intterrupts
		devpriv->IntControlReg&=~Int_DTrg;
		if (!devpriv->IntControlReg) // all IRQ disabled
			outl(inl(devpriv->iobase_a+AMCC_OP_REG_INTCSR)&(~0x00001f00), devpriv->iobase_a+AMCC_OP_REG_INTCSR); // disable int in AMCC
		outl(devpriv->IntControlReg,dev->iobase+PCI9118_INTCTRL);
	}
	return 0;
}

/* 
==============================================================================
*/
int pci9118_ai_cancel(comedi_device * dev, comedi_subdevice * s)
{
	outl(inl(devpriv->iobase_a+AMCC_OP_REG_INTCSR)&(~AINT_WRITE_COMPL), devpriv->iobase_a+AMCC_OP_REG_INTCSR);	// stop amcc irqs
	outl(inl(devpriv->iobase_a+AMCC_OP_REG_MCSR)&(~EN_A2P_TRANSFERS), devpriv->iobase_a+AMCC_OP_REG_MCSR); // stop DMA
	pci9118_exttrg_del(dev,0);
	start_pacer(dev,0,0,0);			// stop 8254 counters
	devpriv->AdFunctionReg=AdFunction_PDTrg|AdFunction_PETrg;
	outl(devpriv->AdFunctionReg,dev->iobase+PCI9118_ADFUNC);// positive triggers, no S&H, no burst, burst stop, no post trigger, no about trigger, trigger stop
	devpriv->AdControlReg=AdControl_Int;
	outl(devpriv->AdControlReg,dev->iobase+PCI9118_ADCNTRL);// bipolar, S.E., use 8254, stop 8354, internal trigger, soft trigger, disable INT and DMA
	outl(0,dev->iobase+PCI9118_BURST);
	outl(1,dev->iobase+PCI9118_SCANMOD);
	outl(2,dev->iobase+PCI9118_SCANMOD);// reset scan queue
	outl(0,dev->iobase+PCI9118_DELFIFO); // flush FIFO

	devpriv->ai_do=0;
        devpriv->ai1234_act_scan=0;
        devpriv->ai1234_act_scanpos=0;
        s->async->cur_chan=0;
        devpriv->ai1234_buf_ptr=0;
        devpriv->neverending_ai=0;
	devpriv->ai4_status=0;
	devpriv->dma_actbuf=0;
  
	return 0;
}

/* 
==============================================================================
*/
static int pci9118_reset(comedi_device *dev)
{
	devpriv->IntControlReg=0;
	devpriv->exttrg_users=0;
	inl(dev->iobase+PCI9118_INTCTRL);
	outl(devpriv->IntControlReg,dev->iobase+PCI9118_INTCTRL);// disable interrupts source
        outl(0xb4, dev->iobase + PCI9118_CNTCTRL);
	start_pacer(dev,0,0,0);			// stop 8254 counters
	devpriv->AdControlReg=0;
	outl(devpriv->AdControlReg,dev->iobase+PCI9118_ADCNTRL);// bipolar, S.E., use 8254, stop 8354, internal trigger, soft trigger, disable INT and DMA
	outl(0,dev->iobase+PCI9118_BURST);
	outl(1,dev->iobase+PCI9118_SCANMOD);
	outl(2,dev->iobase+PCI9118_SCANMOD);// reset scan queue
	devpriv->AdFunctionReg=AdFunction_PDTrg|AdFunction_PETrg;
	outl(devpriv->AdFunctionReg,dev->iobase+PCI9118_ADFUNC);// positive triggers, no S&H, no burst, burst stop, no post trigger, no about trigger, trigger stop

	outl(2047,dev->iobase+PCI9118_DA1);// reset A/D outs to 0V
	outl(2047,dev->iobase+PCI9118_DA2);
	outl(0,dev->iobase+PCI9118_DO);	// reset digi outs to L
	udelay(10);
	inl(dev->iobase+PCI9118_AD_DATA);
	outl(0,dev->iobase+PCI9118_DELFIFO); // flush FIFO
	outl(0,dev->iobase+PCI9118_INTSRC); // remove INT requests
	inl(dev->iobase+PCI9118_ADSTAT); // flush A/D status register
	inl(dev->iobase+PCI9118_INTSRC); // flush INT requests
	devpriv->AdControlReg=0; 
	outl(devpriv->AdControlReg,dev->iobase+PCI9118_ADCNTRL);// bipolar, S.E., use 8254, stop 8354, internal trigger, soft trigger, disable INT and DMA

	devpriv->cnt0_users=0;
	devpriv->exttrg_users=0;
	
	return 0;
}

/* 
==============================================================================
*/
static int pci9118_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret,pages,i;
	unsigned short io_addr[5],master,irq;
        unsigned int iobase_a,iobase_9;
	struct pcilst_struct *card=NULL;
	unsigned char pci_bus,pci_slot,pci_func;
	
	if (!pci_list_builded) {
		pci_card_list_init(PCI_VENDOR_ID_AMCC,0);
		pci_list_builded=1;
	}

	rt_printk("comedi%d: adl_pci9118: board=%s",dev->minor,this_board->name);

	if ((card=select_and_alloc_pci_card(PCI_VENDOR_ID_AMCC, this_board->device_id, it->options[0], it->options[1]))==NULL) 
		return -EIO;
	
	if ((pci_card_data(card,&pci_bus,&pci_slot,&pci_func,
	                    &io_addr[0],&irq,&master))<0) {
		pci_card_free(card);
		rt_printk(" - Can't get AMCC data!\n");
		return -EIO;
	}

	iobase_a=io_addr[0];
	iobase_9=io_addr[2];


	rt_printk(", b:s:f=%d:%d:%d, io=0x%4x, 0x%4x",pci_bus,pci_slot,pci_func,iobase_9,iobase_a);
	
        if (check_region(iobase_9, this_board->iorange_9118) < 0) {
		rt_printk("I/O port conflict\n");
		return -EIO;
        }
        if (check_region(iobase_a, this_board->iorange_amcc) < 0) {
		rt_printk("I/O port conflict\n");
		return -EIO;
        }

        request_region(iobase_9, this_board->iorange_9118, "ADLink PCI-9118");

        dev->iobase=iobase_9;
	dev->board_name = this_board->name;

	if((ret=alloc_private(dev,sizeof(pci9118_private)))<0)
		return -ENOMEM;

	devpriv->amcc=card;
	devpriv->master=master;
        request_region(iobase_a, this_board->iorange_amcc, "ADLink PCI-9118");
	devpriv->iobase_a=iobase_a;
	
	if (irq>0)  {
		if (comedi_request_irq(irq, interrupt_pci9118, SA_SHIRQ, "ADLink PCI-9118", dev)) {
			rt_printk(", unable to allocate IRQ %d, DISABLING IT", irq);
			irq=0; /* Can't use IRQ */
		} else {
			rt_printk(", irq=%d", irq);
		}    
	} else {
			rt_printk(", IRQ disabled");
	}

        dev->irq = irq;

	if (master) {	// alloc DMA buffers
		devpriv->dma_doublebuf=0;
		for (i=0; i<2; i++) {
			for (pages=4; pages>=0; pages--)
				if ((devpriv->dmabuf_virt[i]=__get_free_pages(GFP_KERNEL,4)))
					break;
			if (devpriv->dmabuf_virt[i]) {
				devpriv->dmabuf_pages[i]=pages;
				devpriv->dmabuf_size[i]=PAGE_SIZE*pages;
				devpriv->dmabuf_samples[i]=devpriv->dmabuf_size[i]>>1;
				devpriv->dmabuf_hw[i]=virt_to_bus((void *)devpriv->dmabuf_virt[i]);
			}
		}
		if (!devpriv->dmabuf_virt[0]) {
			rt_printk(", Can't allocate DMA buffer, DMA disabled!");
			master=0;
		}

		if (devpriv->dmabuf_virt[1])
			devpriv->dma_doublebuf=1;

	}
	
	if ((devpriv->master=master)) {
		rt_printk(", bus master");
	} else {
		rt_printk(", no bus master");
	}

	devpriv->usemux=0;
	if (it->options[2]>0)	{
		devpriv->usemux=it->options[2];
		if (devpriv->usemux>256) devpriv->usemux=256; // max 256 channels!
		if (it->options[3]>0)
			if (devpriv->usemux>128) {
				devpriv->usemux=128; // max 128 channels with softare S&H!
			}
		rt_printk(", ext. mux %d channels",devpriv->usemux);
	}

	printk(".\n");

        dev->n_subdevices = 4;
        if((ret=alloc_subdevices(dev))<0)
    		return ret;

	s = dev->subdevices + 0;
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE|SDF_COMMON|SDF_GROUND|SDF_DIFF;
	if (devpriv->usemux) { s->n_chan = devpriv->usemux; }
			else { s->n_chan = this_board->n_aichan; }
	s->maxdata = this_board->ai_maxdata;
	s->len_chanlist = this_board->n_aichanlist;
	s->range_table = this_board->rangelist_ai;
	s->cancel=pci9118_ai_cancel;
	s->insn_read=pci9118_insn_read_ai;
	s->do_cmdtest=pci9118_ai_cmdtest;
	s->do_cmd=pci9118_ai_cmd;

	s = dev->subdevices + 1;
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE|SDF_GROUND|SDF_COMMON;
	s->n_chan = this_board->n_aochan;
	s->maxdata = this_board->ao_maxdata;
	s->len_chanlist = this_board->n_aochan;
	s->range_table = this_board->rangelist_ao;
	s->insn_write=pci9118_insn_write_ao;
	s->insn_read=pci9118_insn_read_ao;

	s = dev->subdevices + 2;
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE|SDF_GROUND|SDF_COMMON;
	s->n_chan = 4;
	s->maxdata = 1;
	s->len_chanlist = 4;
	s->range_table = &range_digital;
	s->io_bits=0;		/* all bits input */
	s->insn_bits=pci9118_insn_bits_di;

	s = dev->subdevices + 3;
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_WRITEABLE|SDF_GROUND|SDF_COMMON;
	s->n_chan = 4;
	s->maxdata = 1;
	s->len_chanlist = 4;
	s->range_table = &range_digital;
	s->io_bits=0xf;		/* all bits output */
	s->insn_bits=pci9118_insn_bits_do;

	pci9118_reset(dev);

	devpriv->valid=1;
	devpriv->i8254_osc_base=250;	// 250ns=4MHz

	return 0;
}


/* 
==============================================================================
*/
static int pci9118_detach(comedi_device *dev)
{
	if (dev->private) {
		if (devpriv->valid) pci9118_reset(dev);
		release_region(devpriv->iobase_a,this_board->iorange_amcc);
		if (devpriv->allocated)	pci_card_free(devpriv->amcc);
		if (devpriv->dmabuf_virt[0]) free_pages(devpriv->dmabuf_virt[0],devpriv->dmabuf_pages[0]);
		if (devpriv->dmabuf_virt[1]) free_pages(devpriv->dmabuf_virt[1],devpriv->dmabuf_pages[1]);
	}

	if(dev->irq) free_irq(dev->irq,dev);

	if(dev->iobase)	release_region(dev->iobase,this_board->iorange_9118);
		
	if (pci_list_builded) {
	    	pci_card_list_cleanup(PCI_VENDOR_ID_AMCC);
		pci_list_builded=0;
	}

	return 0;
}

/*
==============================================================================
*/

/* a unknow future:
 *  [3] - sample&hold signal - card can generate hold signal for external S&H board
 *            0=use SSHO (pin 45) signal with onboard hardware S&H logic
 *            1=use ADCHN7 (pin 23) signal and use software for timing
 *              (in this case external multiplexor can serve only 128 A/D channels)
*/
