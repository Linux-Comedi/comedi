/*
   module/pcl818.c

   Author:  Michal Dobes <majkl@tesnet.cz>  
   
   hardware driver for Advantech cards:
    card:   PCL-818L, PCL-818H, PCL-818HD, PCL-818HG, PCL-818, PCL-718
    driver: pcl818l,  pcl818h,  pcl818hd,  pcl818hg,  pcl818,  pcl718

   Options for PCL-818L:
    [0] - IO Base
    [1] - IRQ	(0=disable, 2, 3, 4, 5, 6, 7)
    [2] - DMA	(0=disable, 1, 3)
    [3] - 0, 10=10MHz clock for 8254
              1= 1MHz clock for 8254
    [4] - 0,  5=A/D input  -5V.. +5V
          1, 10=A/D input -10V..+10V
    [5] - 0,  5=D/A output 0-5V  (internal reference -5V)
          1, 10=D/A output 0-10V (internal reference -10V)
	  2    =D/A output unknow (external reference)
       
   Options for PCL-818, PCL-818H:
    [0] - IO Base
    [1] - IRQ	(0=disable, 2, 3, 4, 5, 6, 7)
    [2] - DMA	(0=disable, 1, 3)
    [3] - 0, 10=10MHz clock for 8254
              1= 1MHz clock for 8254
    [4] - 0,  5=D/A output 0-5V  (internal reference -5V)
          1, 10=D/A output 0-10V (internal reference -10V)
	  2    =D/A output unknow (external reference)

   Options for PCL-818HD, PCL-818HG:
    [0] - IO Base
    [1] - IRQ	(0=disable, 2, 3, 4, 5, 6, 7)
    [2] - DMA/FIFO  (-1=use FIFO, 0=disable both FIFO and DMA, 
                      1=use DMA ch 1, 3=use DMA ch 3)
    [3] - 0, 10=10MHz clock for 8254
              1= 1MHz clock for 8254
    [4] - 0,  5=D/A output 0-5V  (internal reference -5V)
          1, 10=D/A output 0-10V (internal reference -10V)
   	  2    =D/A output unknow (external reference)

   Options for PCL-718:
    [0] - IO Base
    [1] - IRQ	(0=disable, 2, 3, 4, 5, 6, 7)
    [2] - DMA	(0=disable, 1, 3)
    [3] - 0, 10=10MHz clock for 8254
              1= 1MHz clock for 8254
    [4] -     0=A/D Range is +/-10V
	      1=             +/-5V
	      2=             +/-2.5V
	      3=             +/-1V
	      4=             +/-0.5V 
	      5=  	     user defined bipolar
	      6=	     0-10V
	      7=	     0-5V
 	      8=	     0-2V
	      9=	     0-1V
	     10=	     user defined unipolar
    [5] - 0,  5=D/A outputs 0-5V  (internal reference -5V)
          1, 10=D/A outputs 0-10V (internal reference -10V)
	      2=D/A outputs unknow (external reference)
    [6] - 0, 60=max  60kHz A/D sampling
          1,100=max 100kHz A/D sampling (PCL-718 with Option 001 installed)

*/

#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/mc146818rtc.h>
#include <asm/dma.h>
#include <linux/comedidev.h>
#include <8253.h>

// #define PCL818_MODE13_AO 1

// boards constants

#define boardPCL818L 0
#define boardPCL818H 1
#define boardPCL818HD 2
#define boardPCL818HG 3
#define boardPCL818 4
#define boardPCL718 5

// IO space len
#define PCLx1x_RANGE 16
// IO space len if we use FIFO
#define PCLx1xFIFO_RANGE 32

// W: clear INT request
#define PCL818_CLRINT 8
// R: return status byte
#define PCL818_STATUS 8
// R: A/D high byte W: A/D range control
#define PCL818_RANGE 1
// R: next mux scan channel W: mux scan channel & range control pointer
#define PCL818_MUX 2
// R/W: operation control register
#define PCL818_CONTROL 9
// W: counter enable
#define PCL818_CNTENABLE 10

// R: low byte of A/D W: soft A/D trigger
#define PCL818_AD_LO 0
// R: high byte of A/D W: A/D range control
#define PCL818_AD_HI 1
// W: D/A low&high byte
#define PCL818_DA_LO 4
#define PCL818_DA_HI 5
// R: low&high byte of DI 
#define PCL818_DI_LO 3
#define PCL818_DI_HI 11
// W: low&high byte of DO
#define PCL818_DO_LO 3
#define PCL818_DO_HI 11
// W: PCL718 second D/A
#define PCL718_DA2_LO 6
#define PCL718_DA2_HI 7
// counters
#define PCL818_CTR0 12
#define PCL818_CTR1 13
#define PCL818_CTR2 14
// W: counter control
#define PCL818_CTRCTL 15

// W: fifo enable/disable
#define PCL818_FI_ENABLE 6
// W: fifo interrupt clear
#define PCL818_FI_INTCLR 20
// W: fifo interrupt clear
#define PCL818_FI_FLUSH 25
// R: fifo status
#define PCL818_FI_STATUS 25
// R: one record from FIFO
#define PCL818_FI_DATALO 23
#define PCL818_FI_DATAHI 23

// type of interrupt handler
#define INT_TYPE_AI1_INT 1
#define INT_TYPE_AI1_DMA 2 
#define INT_TYPE_AI1_FIFO 3
#define INT_TYPE_AI3_INT 4
#define INT_TYPE_AI3_DMA 5
#define INT_TYPE_AI3_FIFO 6
#ifdef PCL818_MODE13_AO
#define INT_TYPE_AO1_INT 7
#define INT_TYPE_AO3_INT 8
#endif
#define INT_TYPE_AI1_DMA_RTC 9
#define INT_TYPE_AI3_DMA_RTC 10

// RTC stuff...
#define RTC_IRQ 	8
#define RTC_IO_EXTENT	0x10

#define MAGIC_DMA_WORD 0x5a5a

static comedi_lrange range_pcl818h_ai = { 9, {
	BIP_RANGE(5),
	BIP_RANGE(2.5),
	BIP_RANGE(1.25),
	BIP_RANGE(0.625),
	UNI_RANGE(10),
	UNI_RANGE(5),
	UNI_RANGE(2.5),
	UNI_RANGE(1.25),
	BIP_RANGE(10),
}};

static comedi_lrange range_pcl818hg_ai = { 10, {
	BIP_RANGE(5),
	BIP_RANGE(0.5),
	BIP_RANGE(0.05),
	BIP_RANGE(0.005),
	UNI_RANGE(10),
	UNI_RANGE(1),
	UNI_RANGE(0.1),
	UNI_RANGE(0.01),
	BIP_RANGE(10),
	BIP_RANGE(1),
	BIP_RANGE(0.1),
	BIP_RANGE(0.01),
}};

static comedi_lrange range_pcl818l_l_ai = { 4, {
	BIP_RANGE(5),
	BIP_RANGE(2.5),
	BIP_RANGE(1.25),
	BIP_RANGE(0.625),
}};

static comedi_lrange range_pcl818l_h_ai = { 4, {
	BIP_RANGE(10),
	BIP_RANGE(5),
	BIP_RANGE(2.5),
	BIP_RANGE(1.25),
}};

static comedi_lrange range718_bipolar1 = { 1, { BIP_RANGE(1), }};
static comedi_lrange range718_bipolar0_5 = { 1, { BIP_RANGE(0.5), }};
static comedi_lrange range718_unipolar2 = { 1, { UNI_RANGE(2), }};
static comedi_lrange range718_unipolar1 = { 1, { BIP_RANGE(1), }};

static int pcl818_attach(comedi_device *dev,comedi_devconfig *it);
static int pcl818_detach(comedi_device *dev);

static int RTC_lock = 0;		/* RTC lock */
static int RTC_timer_lock = 0;		/* RTC int lock */

typedef struct {
	char 		*name;		// driver name
	int 		n_ranges;	// len of range list
	int 		n_aichan_se;	// num of A/D chans in single ended  mode
	int 		n_aichan_diff;	// num of A/D chans in diferencial mode
	unsigned int 	ns_min;		// minimal alllowed delay between samples (in ns)
	int 		n_aochan;	// num of D/A chans
	int 		n_dichan;	// num of DI chans
	int 		n_dochan;	// num of DO chans
	comedi_lrange 	*ai_range_type;	// default A/D rangelist
	comedi_lrange 	*ao_range_type;	// dafault D/A rangelist
	int 		io_range;	// len of IO space
	unsigned int 	IRQbits;	// allowed interrupts
	unsigned int 	DMAbits; 	// allowed DMA chans
	int 		ai_maxdata;	// maxdata for A/D
	int 		ao_maxdata;	// maxdata for D/A           
	int 		ai_chanlist;	// allowed len of channel list A/D
	int 		ao_chanlist;	// allowed len of channel list D/A
	unsigned char 	fifo;		// 1=board've FIFO
	int		is_818;
} boardtype;

static boardtype boardtypes[] =
{
	{"pcl818l",   4, 16, 8, 25000, 1, 16, 16, &range_pcl818l_l_ai, &range_unipolar5, PCLx1x_RANGE, 0x00fc, 
	  0x0a, 0xfff, 0xfff, 1024, 1, 0, 1 },
	{"pcl818h",   9, 16, 8, 10000, 1, 16, 16, &range_pcl818h_ai,   &range_unipolar5, PCLx1x_RANGE, 0x00fc, 
	  0x0a, 0xfff, 0xfff, 1024, 1, 0, 1 },
	{"pcl818hd",  9, 16, 8, 10000, 1, 16, 16, &range_pcl818h_ai,   &range_unipolar5, PCLx1x_RANGE, 0x00fc, 
	  0x0a, 0xfff, 0xfff, 1024, 1, 1, 1 },
	{"pcl818hg", 12, 16, 8, 10000, 1, 16, 16, &range_pcl818hg_ai,  &range_unipolar5, PCLx1x_RANGE, 0x00fc, 
	  0x0a, 0xfff, 0xfff, 1024, 1, 1, 1 },
	{"pcl818",    9, 16, 8, 10000, 2, 16, 16, &range_pcl818h_ai,   &range_unipolar5, PCLx1x_RANGE, 0x00fc, 
	  0x0a, 0xfff, 0xfff, 1024, 2, 0, 1 },
	{"pcl718",    1, 16, 8, 16000, 2, 16, 16, &range_unipolar5,    &range_unipolar5, PCLx1x_RANGE, 0x00fc, 
	  0x0a, 0xfff, 0xfff, 1024, 2, 0, 0 },
	/* pcm3718 */
	{"pcm3718",   9, 16, 8, 10000, 0, 16, 16, &range_pcl818h_ai,   &range_unipolar5, PCLx1x_RANGE, 0x00fc, 
	  0x0a, 0xfff, 0xfff, 1024, 0, 0, 1 /* XXX ? */ },
};

#define n_boardtypes (sizeof(boardtypes)/sizeof(boardtype))

comedi_driver driver_pcl818={
	driver_name:	"pcl818",
	module:		THIS_MODULE,
	attach:		pcl818_attach,
	detach:		pcl818_detach,
	board_name:	boardtypes,
	num_names:	n_boardtypes,
	offset:		sizeof(boardtype),
};
COMEDI_INITCLEANUP(driver_pcl818);


typedef struct {
	int 		dma;		// used DMA, 0=don't use DMA
	int 		dma_rtc;	// 1=RTC used with DMA, 0=no RTC alloc
	unsigned int 	rtc_iobase;	// RTC port region
	unsigned int 	rtc_iosize;
	unsigned int 	rtc_irq;
	unsigned long 	dmabuf[2];	// pointers to begin of DMA buffers
	unsigned int 	dmapages[2];	// len of DMA buffers in PAGE_SIZEs
	unsigned int 	hwdmaptr[2];	// hardware address of DMA buffers
	unsigned int 	hwdmasize[2]; 	// len of DMA buffers in Bytes
	unsigned int 	dmasamplsize;	// size in samples hwdmasize[0]/2
	unsigned int 	last_top_dma; 	// DMA pointer in last RTC int
	int 		next_dma_buf;	// which DMA buffer will be used next round
	long 		dma_runs_to_end;// how many we must permorm DMA transfer to end of record
	unsigned long 	last_dma_run;	// how many bytes we must transfer on last DMA page
	unsigned char 	neverending_ai;	// if=1, then we do neverending record (you must use cancel())
	unsigned int 	ns_min;		// manimal alllowed delay between samples (in us) for actual card
	int 		i8253_osc_base;	// 1/frequency of on board oscilator in ns
	int 		irq_free;	// 1=have allocated IRQ
	int 		irq_blocked;	// 1=IRQ now uses any subdev
	int 		rtc_irq_blocked;// 1=we now do AI with DMA&RTC
	int 		irq_was_now_closed;// when IRQ finish, there's stored int818_mode for last interrupt
	int 		int818_mode; 	// who now uses IRQ - 1=AI1 int, 2=AI1 dma, 3=AI3 int, 4AI3 dma 
	comedi_subdevice *last_int_sub;	// ptr to subdevice which now finish
	int 		int13_act_scan;	// how many scans we finished
	int 		int13_act_chan;	// actual position in actual scan
	unsigned int 	act_chanlist[16];// MUX setting for actual AI operations
	unsigned int 	act_chanlist_len;// how long is actual MUX list
	unsigned int 	act_chanlist_pos;// actual position in MUX list
	unsigned int 	buf_ptr;	// data buffer ptr in samples
	comedi_subdevice *sub_ai;	// ptr to AI subdevice
	unsigned char 	usefifo;	// 1=use fifo
	struct timer_list rtc_irq_timer;// timer for RTC sanity check
	unsigned long 	rtc_freq;	// RTC int freq
} pcl818_private;


static unsigned int muxonechan[] ={ 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, // used for gain list programming
                                    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};

#define devpriv ((pcl818_private *)dev->private)
#define this_board ((boardtype *)dev->board_ptr)

/* 
==============================================================================
*/
int check_and_setup_channel_list(comedi_device * dev, comedi_subdevice * s, comedi_trig * it);
int pcl818_ai_cancel(comedi_device * dev, comedi_subdevice * s);
void start_pacer(comedi_device * dev, int mode, unsigned int divisor1, unsigned int divisor2);
int set_rtc_irq_bit(unsigned char bit);
void rtc_dropped_irq(unsigned long data);
int rtc_setfreq_irq(int freq);

/* 
==============================================================================
   ANALOG INPUT MODE0, 818 cards, slow version
*/
static int pcl818_ai_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        int timeout; 

        outb(0, dev->iobase+PCL818_CONTROL); /* software trigger, DMA and INT off */
        outb(0, dev->iobase+PCL818_CLRINT); /* clear INT (conversion end) flag */
        outb(muxonechan[CR_CHAN(it->chanlist[0])], dev->iobase+PCL818_MUX); /* select channel */
        outb(CR_RANGE(it->chanlist[0]), dev->iobase+PCL818_RANGE); /* select gain */
        udelay(5);
	outb(0, dev->iobase+PCL818_AD_LO); /* start conversion */
        timeout=100;
        while (timeout--) {
		if (inb(dev->iobase + PCL818_STATUS) & 0x10) goto conv_finish;
		udelay(1);
        }
        comedi_error(dev,"A/D mode0 timeout");
        it->data[0]=0;
        outb(0, dev->iobase+PCL818_CLRINT); /* clear INT (conversion end) flag */
        return -ETIME;

conv_finish:
        it->data[0] = ((inb(dev->iobase + PCL818_AD_HI) << 4) | (inb(dev->iobase + PCL818_AD_LO) >> 4));
  
        outb(0, dev->iobase+PCL818_STATUS); /* clear INT (conversion end) flag */
        return 1;
}

/* 
==============================================================================
   ANALOG OUTPUT MODE0, 818 cards
   only one sample per call is supported
*/
static int pcl818_ao_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        int chan,i;
        sampl_t data;
 
	for (i=0;i<it->n_chan;i++){
    		data=it->data[i];
		chan=CR_CHAN(it->chanlist[i]);
		if (!chan) {
			outb((data & 0x000f) << 4, dev->iobase+PCL818_DA_LO);
			outb((data & 0x0ff0) >> 4, dev->iobase+PCL818_DA_HI);
		} else {
			outb((data & 0x000f) << 4, dev->iobase+PCL718_DA2_LO);
			outb((data & 0x0ff0) >> 4, dev->iobase+PCL718_DA2_HI);
		} 
        }    

        return 0;
}

/* 
==============================================================================
   DIGITAL INPUT MODE0, 818 cards
   
   only one sample per call is supported
*/
static int pcl818_di_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{ 
        int data;
        int chan;
        int i;

	data = inb(dev->iobase + PCL818_DI_LO) |
	        (inb(dev->iobase + PCL818_DI_HI) << 8);

        for(i=0;i<it->n_chan;i++) {
		chan=CR_CHAN(it->chanlist[i]);
		it->data[i]=(data>>chan)&1;
        }

	return it->n_chan;
}

/* 
==============================================================================
   DIGITAL OUTPUT MODE0, 818 cards
   
   only one sample per call is supported
*/
static int pcl818_do_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        int mask, data;
        int chan;
        int i;

	data=s->state;
        for(i=0;i<it->n_chan;i++) {
		chan=CR_CHAN(it->chanlist[i]);
		mask=(1<<chan);
		data &= ~mask;
		if(it->data[i])
	        data |= mask;
        }
	outb(data & 0xff, dev->iobase + PCL818_DO_LO);
        outb((data >> 8), dev->iobase + PCL818_DO_HI);
        s->state = data;

	return it->n_chan;
}

/* 
==============================================================================
   analog input interrupt mode 1 & 3, 818 cards
   one sample per interrupt version   
*/
static void interrupt_pcl818_ai_mode13_int(int irq, void *d, struct pt_regs *regs) 
{
        comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;
        int low;
        int timeout=50; /* wait max 50us */

	while (timeout--) {
		if (inb(dev->iobase + PCL818_STATUS) & 0x10) goto conv_finish;
		udelay(1);
        }
        outb(0,dev->iobase+PCL818_STATUS); /* clear INT request */
        comedi_error(dev,"A/D mode1/3 IRQ without DRDY!");
	pcl818_ai_cancel(dev,s);
        comedi_error_done(dev,s);
	return;

conv_finish:
        low=inb(dev->iobase + PCL818_AD_LO);
	*(sampl_t *)(s->async->data+devpriv->buf_ptr)=((inb(dev->iobase + PCL818_AD_HI) << 4) | (low >> 4)); // get one sample
	devpriv->buf_ptr+=sizeof(sampl_t);
        outb(0,dev->iobase+PCL818_CLRINT); /* clear INT request */

        if ((low & 0xf)!=devpriv->act_chanlist[devpriv->act_chanlist_pos]) { // dropout!
		rt_printk("comedi: A/D mode1/3 IRQ - channel dropout %x!=%x !\n",(low & 0xf),devpriv->act_chanlist[devpriv->act_chanlist_pos]);
		pcl818_ai_cancel(dev,s);
		comedi_error_done(dev,s);
		return;
        }
        s->async->buf_int_ptr+=sizeof(sampl_t);
        s->async->buf_int_count+=sizeof(sampl_t);

	s->async->cur_chan++;
        if (s->async->cur_chan>=s->async->cur_chanlist_len){
		s->async->cur_chan=0;
#if 0
		if (devpriv->cur_flags & TRIG_WAKE_EOS){
			comedi_eos(dev,s);
		} else {
			comedi_bufcheck(dev,s);
		}
#else
		comedi_bufcheck(dev,s);
#endif
		// rt_printk("E");
		devpriv->int13_act_scan--;
        }

	if (s->async->buf_int_ptr>=s->async->data_len) { /* buffer rollover */
		s->async->buf_int_ptr=0;
		devpriv->buf_ptr=0;
		//printk("B ");
		comedi_eobuf(dev,s);
        }

	if (!devpriv->neverending_ai)
		if ( devpriv->int13_act_scan == 0 ) { /* all data sampled */
			pcl818_ai_cancel(dev,s);
	        	comedi_done(dev,s);
			return;
		}
}

/*
==============================================================================
   analog input dma mode 1 & 3, 818 cards
*/
static void interrupt_pcl818_ai_mode13_dma(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;
	int i,len,bufptr;
	unsigned long flags;
        sampl_t *ptr;

        disable_dma(devpriv->dma);
        devpriv->next_dma_buf=1-devpriv->next_dma_buf;
        if ((devpriv->dma_runs_to_end)>-1) {  // switch dma bufs
		set_dma_mode(devpriv->dma, DMA_MODE_READ);
		flags=claim_dma_lock();
		set_dma_addr(devpriv->dma, devpriv->hwdmaptr[devpriv->next_dma_buf]);
		if (devpriv->dma_runs_to_end) { set_dma_count(devpriv->dma, devpriv->hwdmasize[devpriv->next_dma_buf]); }
					else { set_dma_count(devpriv->dma, devpriv->last_dma_run); }
		release_dma_lock(flags);
		enable_dma(devpriv->dma);
        }

        devpriv->dma_runs_to_end--;
        outb(0,dev->iobase+PCL818_CLRINT); /* clear INT request */
        ptr=(sampl_t *)devpriv->dmabuf[1-devpriv->next_dma_buf];

        len=devpriv->hwdmasize[0] >> 1;
        bufptr=0;

        for (i=0;i<len;i++) {
		if ((ptr[bufptr] & 0xf)!=devpriv->act_chanlist[devpriv->act_chanlist_pos]) { // dropout!
		        rt_printk("comedi: A/D mode1/3 DMA - channel dropout %d!=%d !\n",(ptr[bufptr] & 0xf),devpriv->act_chanlist[devpriv->act_chanlist_pos]);
		        pcl818_ai_cancel(dev,s);
			comedi_error_done(dev,s);
			return;
		}

		*(sampl_t *)(s->async->data+s->async->buf_int_ptr)=
			ptr[bufptr++] >> 4; // get one sample
		devpriv->buf_ptr++;

		s->async->buf_int_ptr+=sizeof(sampl_t);
		s->async->buf_int_count+=sizeof(sampl_t);
		devpriv->act_chanlist_pos++;

		s->async->cur_chan++;
		if(s->async->cur_chan>=s->async->cur_chanlist_len){
			s->async->cur_chan=0;
		}

		if (s->async->buf_int_ptr>=s->async->data_len) { /* buffer rollover */
	    		s->async->buf_int_ptr=0;
			devpriv->buf_ptr=0;
			comedi_eobuf(dev,s);
		}

		if (!devpriv->neverending_ai)
	    		if ( devpriv->int13_act_scan == 0 ) { /* all data sampled */
				pcl818_ai_cancel(dev,s);
				comedi_done(dev,s);
				// printk("done int ai13 dma\n");
				return;
			}
	}

	if (len>0)  comedi_bufcheck(dev,s);
}

/*
==============================================================================
   analog input dma mode 1 & 3 over RTC, 818 cards
*/
static void interrupt_pcl818_ai_mode13_dma_rtc(int irq, void *d, struct pt_regs *regs)
{
        comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;
        unsigned long tmp;
        unsigned int top1,top2,i,bufptr;
        long ofs_dats;
        sampl_t *dmabuf=(sampl_t *)devpriv->dmabuf[0];

        //outb(2,0x378);
        switch(devpriv->int818_mode) {
	case INT_TYPE_AI1_DMA_RTC:
	case INT_TYPE_AI3_DMA_RTC:
		tmp = (CMOS_READ(RTC_INTR_FLAGS) & 0xF0);
		mod_timer(&devpriv->rtc_irq_timer, jiffies + HZ/devpriv->rtc_freq + 2*HZ/100);

		for (i=0; i<10; i++) {
			top1=get_dma_residue(devpriv->dma);
			top2=get_dma_residue(devpriv->dma);
			if (top1==top2) break;
		}

		if (top1!=top2) return;
		top1=devpriv->hwdmasize[0]-top1; // where is now DMA in buffer
		top1>>=1;
		ofs_dats=top1-devpriv->last_top_dma;  // new samples from last call
		if (ofs_dats<0) ofs_dats=(devpriv->dmasamplsize)+ofs_dats;
		if (!ofs_dats) return; // exit=no new samples from last call
		// obsluz data
		i=devpriv->last_top_dma-1;
		i&=(devpriv->dmasamplsize-1);

		if (dmabuf[i]!=MAGIC_DMA_WORD) { // DMA overflow!
			comedi_error(dev,"A/D mode1/3 DMA buffer overflow!");
			//rt_printk("I %d dmabuf[i] %d %d\n",i,dmabuf[i],devpriv->dmasamplsize);
			pcl818_ai_cancel(dev,s);
			comedi_error_done(dev,s);
			return;
		}
		//rt_printk("r %ld ",ofs_dats);

		bufptr=devpriv->last_top_dma;

		for (i=0; i<ofs_dats; i++) {
			if ((dmabuf[bufptr] & 0xf)!=devpriv->act_chanlist[devpriv->act_chanlist_pos]) { // dropout!
				rt_printk("comedi: A/D mode1/3 DMA - channel dropout %d!=%d !\n",(dmabuf[bufptr] & 0xf),devpriv->act_chanlist[devpriv->act_chanlist_pos]);
				pcl818_ai_cancel(dev,s);
				comedi_error_done(dev,s);
				return;
			}

			*(sampl_t *)(s->async->data+s->async->buf_int_ptr)=
				dmabuf[bufptr++] >> 4; // get one sample
			devpriv->buf_ptr++;
			bufptr&=(devpriv->dmasamplsize-1);

			s->async->buf_int_ptr+=sizeof(sampl_t);
			s->async->buf_int_count+=sizeof(sampl_t);

			s->async->cur_chan++;
			if(s->async->cur_chan>=s->async->cur_chanlist_len){
				s->async->cur_chan++;
				devpriv->int13_act_scan--;
			}

			if (s->async->buf_int_ptr>=s->async->data_len) { /* buffer rollover */
				s->async->buf_int_ptr=0;
				devpriv->buf_ptr=0;
				comedi_eobuf(dev,s);
			}

			if (!devpriv->neverending_ai)
				if ( devpriv->int13_act_scan == 0 ) { /* all data sampled */
					pcl818_ai_cancel(dev,s);
					comedi_done(dev,s);
					//printk("done int ai13 dma\n");
					return;
				}
		}

		devpriv->last_top_dma=bufptr;
		bufptr--;
		bufptr&=(devpriv->dmasamplsize-1);
		dmabuf[bufptr]=MAGIC_DMA_WORD;
		comedi_bufcheck(dev,s);
		//outb(0,0x378);
		return;
	}

	//outb(0,0x378);

}

/*
==============================================================================
   analog input interrupt mode 1 & 3, 818HD/HG cards
*/
static void interrupt_pcl818_ai_mode13_fifo(int irq, void *d, struct pt_regs *regs)
{
        comedi_device *dev = d;
        comedi_subdevice *s = dev->subdevices + 0;
        int i,len,lo;

        outb(0, dev->iobase + PCL818_FI_INTCLR);  // clear fifo int request

        lo=inb(dev->iobase + PCL818_FI_STATUS);

        if (lo&4) {
		comedi_error(dev,"A/D mode1/3 FIFO overflow!");
		pcl818_ai_cancel(dev,s);
		comedi_error_done(dev,s);
		return;
        }

        if (lo&1) {
		comedi_error(dev,"A/D mode1/3 FIFO interrupt without data!");
		pcl818_ai_cancel(dev,s);
		comedi_error_done(dev,s);
		return;
	}

        if (lo&2) { len=512; }
	    else { len=0; }

        for (i=0;i<len;i++) {
		lo=inb(dev->iobase + PCL818_FI_DATALO);
		if ((lo & 0xf)!=devpriv->act_chanlist[devpriv->act_chanlist_pos]) { // dropout!
			rt_printk("comedi: A/D mode1/3 FIFO - channel dropout %d!=%d !\n",(lo & 0xf),devpriv->act_chanlist[devpriv->act_chanlist_pos]);
			pcl818_ai_cancel(dev,s);
			comedi_error_done(dev,s);
			return;
		}

		*(sampl_t *)(s->async->data+s->async->buf_int_ptr)=
			(lo >> 4)|(inb(dev->iobase + PCL818_FI_DATAHI) << 4); // get one sample
		devpriv->buf_ptr++;
		s->async->buf_int_ptr+=sizeof(sampl_t);
		s->async->buf_int_count+=sizeof(sampl_t);

		s->async->cur_chan++;
		if(s->async->cur_chan>=s->async->cur_chanlist_len){
			s->async->cur_chan = 0;
			devpriv->int13_act_scan--;
		}

		if (s->async->buf_int_ptr>=s->async->data_len) { /* buffer rollover */
			s->async->buf_int_ptr=0;
			devpriv->buf_ptr=0;
    			comedi_eobuf(dev,s);    
		}
     
		if (!devpriv->neverending_ai)
			if ( devpriv->int13_act_scan == 0 ) { /* all data sampled */
				comedi_bufcheck(dev,s);
				pcl818_ai_cancel(dev,s);
				comedi_done(dev,s); 
				return;
			}
	}

	if (len>0)  comedi_bufcheck(dev,s);
}

/* 
==============================================================================
    INT procedure
*/
static void interrupt_pcl818(int irq, void *d, struct pt_regs *regs) 
{
        comedi_device *dev = d;

        //rt_printk("I\n");
 
        switch (devpriv->int818_mode) {
	case INT_TYPE_AI1_DMA:
	case INT_TYPE_AI3_DMA:
		interrupt_pcl818_ai_mode13_dma(irq, d, regs);
		return; 
	case INT_TYPE_AI1_INT:
	case INT_TYPE_AI3_INT:
		interrupt_pcl818_ai_mode13_int(irq, d, regs);
		return;
	case INT_TYPE_AI1_FIFO:
	case INT_TYPE_AI3_FIFO:
		interrupt_pcl818_ai_mode13_fifo(irq, d, regs);
		return;
#ifdef PCL818_MODE13_AO
	case INT_TYPE_AO1_INT:
	case INT_TYPE_AO3_INT:
		interrupt_pcl818_ao_mode13_int(irq, d, regs);
		return;
#endif
	}

	outb(0,dev->iobase+PCL818_CLRINT); /* clear INT request */

	if ((!dev->irq)|(!devpriv->irq_free)|(!devpriv->irq_blocked)|(!devpriv->int818_mode)) {
		if (devpriv->irq_was_now_closed) {
			devpriv->irq_was_now_closed=0;
			// comedi_error(dev,"last IRQ..");
			return;
		} 
		comedi_error(dev,"bad IRQ!");
		return;
	}

        comedi_error(dev,"IRQ from unknow source!");
}

/* 
==============================================================================
   ANALOG INPUT MODE 1 or 3 DMA , 818 cards
*/
void pcl818_ai_mode13dma_int(int mode, comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        unsigned int flags;
        unsigned int bytes;

        bytes=devpriv->hwdmasize[0];
        if (!devpriv->neverending_ai) {
		bytes=it->n_chan*it->n*sizeof(sampl_t); // how many 
		devpriv->dma_runs_to_end=bytes / devpriv->hwdmasize[0]; // how many DMA pages we must fiil
		devpriv->last_dma_run=bytes % devpriv->hwdmasize[0]; //on last dma transfer must be moved
		devpriv->dma_runs_to_end--;
		if (devpriv->dma_runs_to_end>=0) bytes=devpriv->hwdmasize[0];
		//rt_printk("%d %d %d\n",devpriv->dma_runs_to_end,devpriv->last_dma_run,bytes);
	} 

        devpriv->next_dma_buf=0;
	set_dma_mode(devpriv->dma, DMA_MODE_READ);
        flags=claim_dma_lock();
        clear_dma_ff(devpriv->dma);
        set_dma_addr(devpriv->dma, devpriv->hwdmaptr[0]);
        set_dma_count(devpriv->dma, bytes);
        release_dma_lock(flags);
        enable_dma(devpriv->dma);

        if (mode==1) { 
		devpriv->int818_mode=INT_TYPE_AI1_DMA;
		outb(0x87 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);  /* Pacer+IRQ+DMA */ 
        } else { 
		devpriv->int818_mode=INT_TYPE_AI3_DMA;
		outb(0x86 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);  /* Ext trig+IRQ+DMA */ 
        };
}

/* 
==============================================================================
   ANALOG INPUT MODE 1 or 3 DMA rtc, 818 cards
*/
void pcl818_ai_mode13dma_rtc(int mode, comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        unsigned int flags;
        sampl_t *pole;
 
        set_dma_mode(devpriv->dma, DMA_MODE_READ|DMA_AUTOINIT);
        flags=claim_dma_lock();
        clear_dma_ff(devpriv->dma);
        set_dma_addr(devpriv->dma, devpriv->hwdmaptr[0]);
        set_dma_count(devpriv->dma, devpriv->hwdmasize[0]);
        release_dma_lock(flags);
        enable_dma(devpriv->dma);
        devpriv->last_top_dma=0; //devpriv->hwdmasize[0];
        pole=(sampl_t *)devpriv->dmabuf[0];
        devpriv->dmasamplsize=devpriv->hwdmasize[0]/2;
        pole[devpriv->dmasamplsize-1]=MAGIC_DMA_WORD;
        devpriv->rtc_freq=rtc_setfreq_irq(2048);
        devpriv->rtc_irq_timer.expires=jiffies + HZ/devpriv->rtc_freq + 2*HZ/100;
        devpriv->rtc_irq_timer.data=(unsigned long)dev;
        devpriv->rtc_irq_timer.function=rtc_dropped_irq;
    
        add_timer(&devpriv->rtc_irq_timer);
    
        if (mode==1) { 
		devpriv->int818_mode=INT_TYPE_AI1_DMA_RTC;
		outb(0x07 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);  /* Pacer+DMA */ 
        } else { 
		devpriv->int818_mode=INT_TYPE_AI3_DMA_RTC;
		outb(0x06 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);  /* Ext trig+DMA */ 
    };
}

/* 
==============================================================================
   ANALOG INPUT MODE 1 or 3, 818 cards
*/
static int pcl818_ai_mode13(int mode, comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        int divisor1, divisor2;

        if ((!dev->irq)&&(!devpriv->dma_rtc)) {
		comedi_error(dev,"IRQ not defined!");
		return -EINVAL;
        }

        if (devpriv->irq_blocked)
		return -EBUSY;

	start_pacer(dev, -1, 0, 0); // stop pacer

        if (!check_and_setup_channel_list(dev, s, it)) return -EINVAL;
	udelay(1);

        devpriv->int13_act_scan=it->n;
        devpriv->int13_act_chan=0;
	devpriv->irq_blocked=1;
        devpriv->irq_was_now_closed=0;
        devpriv->neverending_ai=0;
        devpriv->act_chanlist_pos=0;
        devpriv->buf_ptr=0;
  
	if ((it->n==0)||(it->n==-1)) devpriv->neverending_ai=1; //well, user want neverending
  
        if (mode==1) {
		if (it->trigvar<devpriv->ns_min) it->trigvar=devpriv->ns_min;
		i8253_cascade_ns_to_timer(devpriv->i8253_osc_base,&divisor1,&divisor2,&it->trigvar,TRIG_ROUND_NEAREST);
		if (divisor1==1) {	/* PCL718/818 crash if any divisor is set to 1 */
		        divisor1=2;
			divisor2/=2;
		}
		if (divisor2==1) {
			divisor2=2;
			divisor1/=2;
		}
	}
   
        outb(0 , dev->iobase + PCL818_CNTENABLE); /* enable pacer */

        switch (devpriv->dma>0) {
	case 1:		// DMA
	case 3:
	        if (devpriv->dma_rtc==0) { pcl818_ai_mode13dma_int(mode, dev, s, it); }
				else { pcl818_ai_mode13dma_rtc(mode, dev, s, it); }
		break;
	case 0:		// IRQ
		// rt_printk("IRQ\n");
		if (mode==1) { 
			devpriv->int818_mode=INT_TYPE_AI1_INT;
			outb(0x83 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);  /* Pacer+IRQ */ 
		} else { 
			devpriv->int818_mode=INT_TYPE_AI3_INT;
			outb(0x82 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);  /* Ext trig+IRQ */ 
		};
		break;
	case -1:		// FIFO
		outb(1, dev->iobase + PCL818_FI_ENABLE);	// enable FIFO
		if (mode==1) { 
			devpriv->int818_mode=INT_TYPE_AI1_FIFO;
			outb(0x03, dev->iobase + PCL818_CONTROL);  /* Pacer */ 
		} else { 
			devpriv->int818_mode=INT_TYPE_AI3_FIFO;
			outb(0x02, dev->iobase + PCL818_CONTROL); 
		}; /* Ext trig */ 
		break;
        }

        start_pacer(dev, mode, divisor1, divisor2);
    
	switch(devpriv->int818_mode) {
	case INT_TYPE_AI1_DMA_RTC:
	case INT_TYPE_AI3_DMA_RTC:
		set_rtc_irq_bit(1); /* start RTC */
		break;
	}
       
	return 0;
}

/* 
==============================================================================
   ANALOG INPUT MODE 1, 818 cards
*/
static int pcl818_ai_mode1(comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        return pcl818_ai_mode13(1, dev, s, it);
}

/* 
==============================================================================
   ANALOG INPUT MODE 3, 818 cards
*/
static int pcl818_ai_mode3(comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        return pcl818_ai_mode13(3, dev, s, it);
}

/* 
==============================================================================
   ANALOG OUTPUT MODE 1 or 3, 818 cards
*/
#ifdef PCL818_MODE13_AO
static int pcl818_ao_mode13(int mode, comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        int divisor1, divisor2;
 

	if (!dev->irq) {
		comedi_error(dev,"IRQ not defined!");
		return -EINVAL;
        }

        if (devpriv->irq_blocked)
		return -EBUSY;

        start_pacer(dev, -1, 0, 0); // stop pacer

	devpriv->int13_act_scan=it->n;
        devpriv->int13_act_chan=0;
        devpriv->irq_blocked=1;
        devpriv->irq_was_now_closed=0;
        devpriv->neverending_ai=0;
        devpriv->act_chanlist_pos=0;
        devpriv->buf_ptr=0;
      
        if (mode==1) {
		i8253_cascade_ns_to_timer(devpriv->i8253_osc_base,&divisor1,&divisor2,&it->trigvar,TRIG_ROUND_NEAREST);
		if (divisor1==1) {	/* PCL818 crash if any divisor is set to 1 */
			divisor1=2;
			divisor2/=2;
		}
		if (divisor2==1) {
			divisor2=2;
			divisor1/=2;
		}
	}

	outb(0 , dev->iobase + PCL818_CNTENABLE); /* enable pacer */
	if (mode==1) { 
		devpriv->int818_mode=INT_TYPE_AO1_INT;
		outb(0x83 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);  /* Pacer+IRQ */ 
	} else { 
		devpriv->int818_mode=INT_TYPE_AO3_INT;
		outb(0x82 | (dev->irq << 4), dev->iobase + PCL818_CONTROL);  /* Ext trig+IRQ */
	};

        start_pacer(dev, mode, divisor1, divisor2);

        return 0;
}

/* 
==============================================================================
   ANALOG OUTPUT MODE 1, 818 cards
*/
static int pcl818_ao_mode1(comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        return pcl818_ao_mode13(1, dev, s, it);
}

/* 
==============================================================================
   ANALOG OUTPUT MODE 3, 818 cards
*/
static int pcl818_ao_mode3(comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        return pcl818_ao_mode13(3, dev, s, it);
}
#endif

/*
==============================================================================
 Start/stop pacer onboard pacer
*/
void start_pacer(comedi_device * dev, int mode, unsigned int divisor1, unsigned int divisor2) 
{
        outb(0xb4, dev->iobase + PCL818_CTRCTL);
        outb(0x74, dev->iobase + PCL818_CTRCTL);
	outb(0x30, dev->iobase + PCL818_CTRCTL);
        udelay(1);
  
        if (mode==1) {
		outb(divisor2 & 0xff, dev->iobase + PCL818_CTR2);
		outb((divisor2 >> 8) & 0xff, dev->iobase + PCL818_CTR2);
		outb(divisor1  & 0xff, dev->iobase + PCL818_CTR1);
		outb((divisor1 >> 8) & 0xff, dev->iobase + PCL818_CTR1);
        }
}

/*
==============================================================================
 Check if channel list from user is builded correctly 
 If it's ok, then program scan/gain logic
*/
int check_and_setup_channel_list(comedi_device * dev, comedi_subdevice * s, comedi_trig * it) 
{
        unsigned int chansegment[16];  
        unsigned int i, nowmustbechan, seglen, segpos;
    
    /* correct channel and range number check itself comedi/range.c */
	if (it->n_chan<1) {
		comedi_error(dev,"range/channel list is empty!");
		return 0;             
        }

        if (it->n_chan > 1) {
		// first channel is everytime ok
		chansegment[0]=it->chanlist[0];
		// build part of chanlist
		for (i=1, seglen=1; i<it->n_chan; i++, seglen++) {
			// rt_printk("%d. %d %d\n",i,CR_CHAN(it->chanlist[i]),CR_RANGE(it->chanlist[i]));
			// we detect loop, this must by finish
			if (it->chanlist[0]==it->chanlist[i]) break;
			nowmustbechan=(CR_CHAN(chansegment[i-1])+1) % s->n_chan;
			// channel list isn't continous :-(
			if (nowmustbechan!=CR_CHAN(it->chanlist[i])) {
				rt_printk("comedi%d: pcl818: channel list must be continous! chanlist[%i]=%d but must be %d or %d!\n",
					    dev->minor,i,CR_CHAN(it->chanlist[i]),
					    nowmustbechan,CR_CHAN(it->chanlist[0]) );
				return 0;             
			}
			// well, this is next correct channel in list
			chansegment[i]=it->chanlist[i];
		}

		// check whole chanlist
		for (i=0, segpos=0; i<it->n_chan; i++) {
			//rt_printk("%d %d=%d %d\n",CR_CHAN(chansegment[i%seglen]),CR_RANGE(chansegment[i%seglen]),CR_CHAN(it->chanlist[i]),CR_RANGE(it->chanlist[i]));
			if (it->chanlist[i]!=chansegment[i%seglen]) {
				rt_printk("comedi%d: pcl818: bad channel or range number! chanlist[%i]=%d,%d,%d and not %d,%d,%d!\n",
				  dev->minor,i,CR_CHAN(chansegment[i]),
				  CR_RANGE(chansegment[i]),
				  CR_AREF(chansegment[i]),
				  CR_CHAN(it->chanlist[i%seglen]),
				  CR_RANGE(it->chanlist[i%seglen]),
				  CR_AREF(chansegment[i%seglen]));
				return 0; // chan/gain list is strange
			}
		}
	} else {
		seglen=1;
	}
    
	devpriv->act_chanlist_len=seglen;
	devpriv->act_chanlist_pos=0;

	for (i=0; i<seglen; i++) {  // store range list to card
		devpriv->act_chanlist[i]=CR_CHAN(it->chanlist[i]);
		outb(muxonechan[CR_CHAN(it->chanlist[i])], dev->iobase+PCL818_MUX); /* select channel */
		outb(CR_RANGE(it->chanlist[i]), dev->iobase+PCL818_RANGE); /* select gain */
	}

	udelay(1);

	/* select channel interval to sca n*/
	outb(devpriv->act_chanlist[0] | (devpriv->act_chanlist[seglen-1] << 4),
		dev->iobase+PCL818_MUX);
	// printk(" MUX %x\n",devpriv->act_chanlist[0] | (devpriv->act_chanlist[seglen-1] << 4));
	
	// we can serve this with MUX logic
	return 1;
}

/* 
==============================================================================
 Check if board is switched to SE (1) or DIFF(0) mode
*/
int check_single_ended(unsigned int port) 
{
        if (inb(port+PCL818_STATUS)&0x20) { return 1; }
					else { return 0; }
}

/* 
==============================================================================
 cancel any mode 1-4 AI
*/
int pcl818_ai_cancel(comedi_device * dev, comedi_subdevice * s) 
{
        if (devpriv->irq_blocked>0) {
		// rt_printk("pcl818_ai_cancel()\n");
		switch (devpriv->int818_mode) {
		case INT_TYPE_AI1_DMA_RTC:
		case INT_TYPE_AI3_DMA_RTC:
			set_rtc_irq_bit(0); // stop RTC
			del_timer(&devpriv->rtc_irq_timer);
		case INT_TYPE_AI1_DMA:
		case INT_TYPE_AI3_DMA:
			disable_dma(devpriv->dma);
		case INT_TYPE_AI1_INT:
		case INT_TYPE_AI3_INT:
		case INT_TYPE_AI1_FIFO:
		case INT_TYPE_AI3_FIFO:
#ifdef PCL818_MODE13_AO
		case INT_TYPE_AO1_INT:
		case INT_TYPE_AO3_INT:
#endif
			outb(inb(dev->iobase+PCL818_CONTROL)& 0x73, dev->iobase+PCL818_CONTROL); /* Stop A/D */
			udelay(1);
			outb(0, dev->iobase+PCL818_CONTROL); /* Stop A/D */
			outb(0xb4, dev->iobase + PCL818_CTRCTL);/* Stop pacer */
			outb(0x74, dev->iobase + PCL818_CTRCTL);
			outb(0, dev->iobase+PCL818_AD_LO);
			inb(dev->iobase+PCL818_AD_LO);
			inb(dev->iobase+PCL818_AD_HI);
			outb(0, dev->iobase+PCL818_CLRINT); /* clear INT request */
			outb(0, dev->iobase+PCL818_CONTROL); /* Stop A/D */
			if (devpriv->usefifo) { // FIFO shutdown
			        outb(0, dev->iobase + PCL818_FI_INTCLR);
				outb(0, dev->iobase + PCL818_FI_FLUSH);
				outb(0, dev->iobase + PCL818_FI_ENABLE);
			}
			devpriv->irq_blocked=0;
			devpriv->irq_was_now_closed=devpriv->int818_mode;
			devpriv->int818_mode=0;
			devpriv->last_int_sub=s;
			s->busy = 0;
			break;
		}
        }     

        //rt_printk("pcl818_ai_cancel() end\n");
        return 0;
}

/* 
==============================================================================
 chech for PCL818
*/
int pcl818_check(int iobase) 
{
        outb(0x00, iobase + PCL818_MUX);
        udelay(1); 
        if (inb(iobase + PCL818_MUX)!=0x00) return 1; //there isn't card
        outb(0x55, iobase + PCL818_MUX);
        udelay(1); 
        if (inb(iobase + PCL818_MUX)!=0x55) return 1; //there isn't card
        outb(0x00, iobase + PCL818_MUX);
        udelay(1); 
        outb(0x18, iobase + PCL818_CONTROL); 
        udelay(1); 
        if (inb(iobase + PCL818_CONTROL)!=0x18) return 1; //there isn't card
        return 0; // ok, card exist
}

/* 
==============================================================================
 reset whole PCL-818 cards
*/
void pcl818_reset(comedi_device * dev) 
{
        if (devpriv->usefifo) { // FIFO shutdown
		outb(0, dev->iobase + PCL818_FI_INTCLR);
		outb(0, dev->iobase + PCL818_FI_FLUSH);
		outb(0, dev->iobase + PCL818_FI_ENABLE);
        }
        outb(0, dev->iobase + PCL818_DA_LO); // DAC=0V
        outb(0, dev->iobase + PCL818_DA_HI);
        udelay(1);
        outb(0, dev->iobase + PCL818_DO_HI); // DO=$0000
        outb(0, dev->iobase + PCL818_DO_LO);
        udelay(1);
        outb(0, dev->iobase + PCL818_CONTROL);
        outb(0, dev->iobase + PCL818_CNTENABLE);
        outb(0, dev->iobase + PCL818_MUX);
        outb(0, dev->iobase + PCL818_CLRINT);
        outb(0xb0, dev->iobase + PCL818_CTRCTL);/* Stop pacer */
        outb(0x70, dev->iobase + PCL818_CTRCTL);
        outb(0x30, dev->iobase + PCL818_CTRCTL);
	if(this_board->is_818){
    	        outb(0, dev->iobase + PCL818_RANGE);
	}else{
		outb(0, dev->iobase + PCL718_DA2_LO);
		outb(0, dev->iobase + PCL718_DA2_HI);
	}
}

/* 
==============================================================================
  Enable(1)/disable(0) periodic interrupts from RTC
*/
int set_rtc_irq_bit(unsigned char bit) 
{
        unsigned char val;
        unsigned long flags;
 
        if (bit==1) {
		RTC_timer_lock++;
		if (RTC_timer_lock>1) return 0;
	} else {
		RTC_timer_lock--;
		if (RTC_timer_lock<0) RTC_timer_lock=0;
		if (RTC_timer_lock>0) return 0;
	}
  
        save_flags(flags);
        cli();
        val = CMOS_READ(RTC_CONTROL);
        if (bit) { val |= RTC_PIE; }
    	    else { val &=  ~RTC_PIE; }
        CMOS_WRITE(val, RTC_CONTROL);
        CMOS_READ(RTC_INTR_FLAGS);
        restore_flags(flags);
        return 0;
}

/* 
==============================================================================
  Restart RTC if something stop it (xntpd every 11 mins or large IDE transfers)
*/
void rtc_dropped_irq(unsigned long data) 
{
        comedi_device *dev = (void *)data;
        unsigned long flags,tmp;

	switch(devpriv->int818_mode) {
    	case INT_TYPE_AI1_DMA_RTC:
	case INT_TYPE_AI3_DMA_RTC:
	        mod_timer(&devpriv->rtc_irq_timer, jiffies + HZ/devpriv->rtc_freq + 2*HZ/100);
		save_flags(flags);
		cli();
		tmp=(CMOS_READ(RTC_INTR_FLAGS) & 0xF0);	/* restart */
		restore_flags(flags);
		break;
        };     
}


/* 
==============================================================================
  Set frequency of interrupts from RTC
*/
int rtc_setfreq_irq(int freq) 
{
        int tmp = 0;
        int rtc_freq;
        unsigned char val;
        unsigned long flags;

        if (freq<2) freq=2;
        if (freq>8192) freq=8192;

        while (freq>(1<<tmp))
		tmp++;

        rtc_freq = 1<<tmp;

        save_flags(flags);
        cli();
	val = CMOS_READ(RTC_FREQ_SELECT) & 0xf0;
        val |= (16 - tmp);
        CMOS_WRITE(val, RTC_FREQ_SELECT);
        restore_flags(flags);
        return rtc_freq;
}

/* 
==============================================================================
  Free any resources that we have claimed  
*/
static void free_resources(comedi_device * dev) 
{
        //rt_printk("free_resource()\n");
        if(dev->private)  {
		pcl818_ai_cancel(dev, devpriv->sub_ai);
		pcl818_reset(dev);
		if (devpriv->dma) free_dma(devpriv->dma);
		if (devpriv->dmabuf[0]) free_pages(devpriv->dmabuf[0], devpriv->dmapages[0]);
		if (devpriv->dmabuf[1]) free_pages(devpriv->dmabuf[1], devpriv->dmapages[1]);
		if (devpriv->rtc_irq) free_irq(devpriv->rtc_irq, dev);
		if ((devpriv->dma_rtc)&&(RTC_lock==1)) {
			if (devpriv->rtc_iobase) 
				release_region(devpriv->rtc_iobase, devpriv->rtc_iosize);
		} 
	} 

	if (dev->irq) free_irq(dev->irq, dev);
	if (dev->iobase) release_region(dev->iobase, this_board->io_range);
	//rt_printk("free_resource() end\n");
}

/* 
==============================================================================

   Initialization 

*/
static int pcl818_attach(comedi_device * dev, comedi_devconfig * it) 
{
        int ret;
        int iobase;
        int irq,dma;
        unsigned long pages;
        int io_range;
        comedi_subdevice *s;
	
        /* claim our I/O space */
        iobase = it->options[0];
        printk("comedi%d: pcl818:  board=%s, ioport=0x%03x",
		dev->minor, this_board->name, iobase);
        io_range=this_board->io_range;
        if ((this_board->fifo)&&(it->options[2]==0)) // we've board with FIFO and we want to use FIFO
		io_range=PCLx1xFIFO_RANGE;
        if (check_region(iobase, io_range) < 0) {
		rt_printk("I/O port conflict\n");
		return -EIO;
        }

        request_region(iobase, io_range, "pcl818");
        dev->iobase=iobase;
    
        if (pcl818_check(iobase)) {
		rt_printk(", I can't detect board. FAIL!\n");
		return -EIO;
        }

        if((ret=alloc_private(dev,sizeof(pcl818_private)))<0)
		return ret; /* Can't alloc mem */

        /* set up some name stuff */
	dev->board_name = this_board->name;
        if (io_range==PCLx1xFIFO_RANGE) devpriv->usefifo=1; 
        /* grab our IRQ */
        irq=0;
        if (this_board->IRQbits!=0) { /* board support IRQ */
		irq=it->options[1];
		if (irq>0)  {/* we want to use IRQ */
		        if (((1<<irq)&this_board->IRQbits)==0) {
				rt_printk(", IRQ %d is out of allowed range, DISABLING IT",irq);
				irq=0; /* Bad IRQ */
			} else { 
				if (comedi_request_irq(irq, interrupt_pcl818, 0, "pcl818", dev)) {
					rt_printk(", unable to allocate IRQ %d, DISABLING IT", irq);
					irq=0; /* Can't use IRQ */
				} else {
					rt_printk(", irq=%d", irq);
				}    
			}  
		}
	}

        dev->irq = irq;
        if (irq) { devpriv->irq_free=1; } /* 1=we have allocated irq */
	    else { devpriv->irq_free=0; } 
        devpriv->irq_blocked=0; /* number of subdevice which use IRQ */
        devpriv->int818_mode=0; /* mode of irq */

        /* grab RTC for DMA operations */
        devpriv->dma_rtc=0;
        if (it->options[2]>0) { // we want to use DMA
		if (RTC_lock==0) {
			if (check_region(RTC_PORT(0), RTC_IO_EXTENT) < 0) goto no_rtc;
			request_region(RTC_PORT(0), RTC_IO_EXTENT, "pcl818 (RTC)");
		} 
		devpriv->rtc_iobase=RTC_PORT(0);
		devpriv->rtc_iosize=RTC_IO_EXTENT;
		RTC_lock++;
		if (!comedi_request_irq(RTC_IRQ, interrupt_pcl818_ai_mode13_dma_rtc, 0, "pcl818 DMA (RTC)", dev)) {
			devpriv->dma_rtc=1;
			devpriv->rtc_irq=RTC_IRQ;
			rt_printk(", dma_irq=%d", devpriv->rtc_irq);
		} else {
			RTC_lock--;
			if (RTC_lock==0) {
    				if (devpriv->rtc_iobase) release_region(devpriv->rtc_iobase, devpriv->rtc_iosize);
			}
			devpriv->rtc_iobase=0;
			devpriv->rtc_iosize=0;
		}
	} 

no_rtc:
        /* grab our DMA */
        dma=0;
        devpriv->dma=dma;
        if ((devpriv->irq_free==0)&&(devpriv->dma_rtc==0)) goto no_dma; /* if we haven't IRQ, we can't use DMA */
        if (this_board->DMAbits!=0) { /* board support DMA */
		dma=it->options[2];
		if (dma<1) goto no_dma; /* DMA disabled */
		if (((1<<dma)&this_board->DMAbits)==0) {
		        rt_printk(", DMA is out of allowed range, FAIL!\n");
			return -EINVAL; /* Bad DMA */
		} 
		ret=request_dma(dma, "pcl818");
		if (ret) {
			rt_printk(", unable to allocate DMA %d, FAIL!\n",dma);
			return -EBUSY; /* DMA isn't free */
		} 
		devpriv->dma=dma;
		rt_printk(", dma=%d", dma);
		pages=2; /* we need 16KB */
		devpriv->dmabuf[0]=__get_dma_pages(GFP_KERNEL, pages);
		if (!devpriv->dmabuf[0]) {
			rt_printk(", unable to allocate DMA buffer, FAIL!\n");
			/* maybe experiment with try_to_free_pages() will help .... */
			return -EBUSY; /* no buffer :-( */
		}
		devpriv->dmapages[0]=pages;
		devpriv->hwdmaptr[0] = virt_to_bus((void *)devpriv->dmabuf[0]);
		devpriv->hwdmasize[0]=(1<<pages)*PAGE_SIZE;
		//rt_printk("%d %d %ld, ",devpriv->dmapages[0],devpriv->hwdmasize[0],PAGE_SIZE);
		if (devpriv->dma_rtc==0) { // we must do duble buff :-(
			devpriv->dmabuf[1]=__get_dma_pages(GFP_KERNEL, pages);
			if (!devpriv->dmabuf[1]) {
				rt_printk(", unable to allocate DMA buffer, FAIL!\n");
				return -EBUSY;
			}
			devpriv->dmapages[1]=pages;
			devpriv->hwdmaptr[1] = virt_to_bus((void *)devpriv->dmabuf[1]);
			devpriv->hwdmasize[1]=(1<<pages)*PAGE_SIZE;
		} 
	}

no_dma:
   
        dev->n_subdevices = 4;
        if((ret=alloc_subdevices(dev))<0) return ret;

	s = dev->subdevices + 0;
	if(!this_board->n_aichan_se){
		s->type = COMEDI_SUBD_UNUSED;
	}else{
		s->type = COMEDI_SUBD_AI;
		devpriv->sub_ai=s;
		s->subdev_flags = SDF_READABLE|SDF_RT;
		if (check_single_ended(dev->iobase)) {
			s->n_chan = this_board->n_aichan_se;
			s->subdev_flags|=SDF_COMMON|SDF_GROUND;
			rt_printk(", %dchans S.E. DAC",s->n_chan);
		} else {
			s->n_chan = this_board->n_aichan_diff;
			s->subdev_flags|=SDF_DIFF;
			rt_printk(", %dchans DIFF DAC",s->n_chan);
		}
		s->maxdata = this_board->ai_maxdata;
		s->len_chanlist = this_board->ai_chanlist;
		s->range_table = this_board->ai_range_type;
		s->cancel=pcl818_ai_cancel;
		s->trig[0] = pcl818_ai_mode0;
		if ((irq)||(devpriv->dma_rtc)) {
			s->trig[1] = pcl818_ai_mode1;
			s->trig[3] = pcl818_ai_mode3;
		} 
		if(this_board->is_818){
			if ((it->options[4]==1)||(it->options[4]==10)) 
				s->range_table=&range_pcl818l_h_ai; // secondary range list jumper selectable
		}else{
			switch (it->options[4]) {
			case 0: s->range_table=&range_bipolar10; break;
			case 1: s->range_table=&range_bipolar5; break;
			case 2: s->range_table=&range_bipolar2_5; break;
			case 3: s->range_table=&range718_bipolar1; break;
			case 4: s->range_table=&range718_bipolar0_5; break;
			case 6: s->range_table=&range_unipolar10; break;
			case 7: s->range_table=&range_unipolar5; break;
			case 8: s->range_table=&range718_unipolar2; break;
			case 9: s->range_table=&range718_unipolar1; break;
			default: s->range_table=&range_unknown; break;
			}
		}
	}

	s = dev->subdevices + 1;
	if(!this_board->n_aochan){
		s->type = COMEDI_SUBD_UNUSED;
	}else{
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_WRITEABLE|SDF_GROUND|SDF_RT;
		s->n_chan = this_board->n_aochan;
		s->maxdata = this_board->ao_maxdata;
		s->len_chanlist = this_board->ao_chanlist;
		s->range_table = this_board->ao_range_type;
		s->trig[0] = pcl818_ao_mode0;
#ifdef PCL818_MODE13_AO
		if (irq) {
		        s->trig[1] = pcl818_ao_mode1;
			s->trig[3] = pcl818_ao_mode3;
		} 
#endif
		if(this_board->is_818){
			if ((it->options[4]==1)||(it->options[4]==10)) 
				s->range_table=&range_unipolar10; 
			if (it->options[4]==2) 
				s->range_table=&range_unknown; 
		}else{
			if ((it->options[5]==1)||(it->options[5]==10)) 
				s->range_table=&range_unipolar10; 
			if (it->options[5]==2) 
				s->range_table=&range_unknown; 
		}	
	}

	s = dev->subdevices + 2;
	if(!this_board->n_dichan){
		s->type = COMEDI_SUBD_UNUSED;
	}else{
		s->type = COMEDI_SUBD_DI;
		s->subdev_flags = SDF_READABLE|SDF_RT;
		s->n_chan = this_board->n_dichan;
		s->maxdata = 1;
		s->len_chanlist = this_board->n_dichan;
		s->range_table = &range_digital;
		s->trig[0] = pcl818_di_mode0;
	}

	s = dev->subdevices + 3;
	if(!this_board->n_dochan){
		s->type = COMEDI_SUBD_UNUSED;
	}else{
		s->type = COMEDI_SUBD_DO;
		s->subdev_flags = SDF_WRITEABLE|SDF_RT;
		s->n_chan = this_board->n_dochan;
		s->maxdata = 1;
		s->len_chanlist = this_board->n_dochan;
		s->range_table = &range_digital;
		s->trig[0] = pcl818_do_mode0;
	}

	/* select 1/10MHz oscilator */
        if ((it->options[3]==0)||(it->options[3]==10)) { 
		devpriv->i8253_osc_base= 100; 
	} else { 
		devpriv->i8253_osc_base=1000; 
	}	

         /* max sampling speed */
        devpriv->ns_min=this_board->ns_min;
	  
	if(!this_board->is_818){
		if ((it->options[6]==1)||(it->options[6]==100)) 
			devpriv->ns_min=10000;  /* extended PCL718 to 100kHz DAC */
	}

	pcl818_reset(dev); 

	rt_printk("\n");

	return 0;
}


/*
==============================================================================
  Removes device
 */
static int pcl818_detach(comedi_device * dev) 
{
        //  rt_printk("comedi%d: pcl818: remove\n", dev->minor);
        free_resources(dev);
        if (devpriv->dma_rtc) 
		RTC_lock--;
        return 0;
}

