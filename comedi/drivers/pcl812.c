// *INDENT-OFF*
/*
 * module/pcl812.c
 * hardware driver for Advantech cards
 *  card:   PCL-812PG, PCL-813B
 *  driver: pcl812pg,  pcl813b
 *  
 * Michal Dobes <majkl@tesnet.cz>  
 * Based on 711.c 
 * 
 * Options for PCL-812PG:
 *  [0] - IO Base
 *  [1] - IRQ  (0=disable, 2, 3, 4, 5, 6, 7; 10, 11, 12, 14, 15)
 *  [2] - 0=trigger source is internal 8253 with 2MHz clock
 *        1=trigger source is external 
 *  [3] - 0=A/D have max +/-5V input
 *        1=A/D have max +/-10V input
 *  [4] - 0=D/A outputs 0-5V  (internal reference -5V)
 *        1=D/A outputs 0-10V (internal reference -10V)
 *        2=D/A outputs unknow (external reference)
 *
 * Options for PCL-813B:
 *  [0] - IO Base
 *  [1] - 0= bipolar inputs
 *        1= unipolar inputs
 *  [2] - max number of samples in ai_mode0 (defaul=1scan)
 *
 *     
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/malloc.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <asm/dma.h>
#include <linux/comedidev.h>
#include <8253.h>

/* #define MD_DEBUG */

#define boardPCL812PG 0
#define boardPCL813B 1

#define PCLx1x_RANGE 16

#define PCL812_CLRINT 8
#define PCL812_GAIN 9
#define PCL812_MUX 10
#define PCL812_MODE 11
#define PCL812_CNTENABLE 10
#define PCL812_SOFTTRIG 12
#define PCL812_CTR0 0
#define PCL812_CTR1 1
#define PCL812_CTR2 2
#define PCL812_CTRCTL 3

#define PCL812_AD_LO 4
#define PCL812_AD_HI 5
#define PCL812_DA1_LO 4
#define PCL812_DA1_HI 5
#define PCL812_DA2_LO 6
#define PCL812_DA2_HI 7
#define PCL812_DI_LO 6
#define PCL812_DI_HI 7
#define PCL812_DO_LO 13
#define PCL812_DO_HI 14

#define PCL812_DRDY 0x10

/*
  For PCL-813B: 
  I don't know if timeouts which are specified at a documentation 
  are miliseconds or microseconds. If your card don't work properly then
  undef next #define.
*/
#define PCL813_MICROSECS

#define INT_TYPE_AI1_INT 1
/* #define INT_TYPE_AI1_DMA 2 */
#define INT_TYPE_AI3_INT 3
/* #define INT_TYPE_AI3_DMA 4 */

static comedi_lrange range_pcl812pg_ai = { 5, {
	BIP_RANGE(5),
	BIP_RANGE(2.5),
	BIP_RANGE(1.25),
	BIP_RANGE(0.625),
	BIP_RANGE(0.3125),
}};
static comedi_lrange range_pcl812pg2_ai = { 5, {
	BIP_RANGE(10),
	BIP_RANGE(5),
	BIP_RANGE(2.5),
	BIP_RANGE(1.25),
	BIP_RANGE(0.625),
}};
static comedi_lrange range_pcl813b_ai = { 4, {
	BIP_RANGE(5),
	BIP_RANGE(2.5),
	BIP_RANGE(1.25),
	BIP_RANGE(0.625),
}};
static comedi_lrange range_pcl813b2_ai = { 4, {
	UNI_RANGE(10),
	UNI_RANGE(5),
	UNI_RANGE(2.5),
	UNI_RANGE(1.25),
}};

static int pcl812_attach(comedi_device *dev,comedi_devconfig *it);
static int pcl812_detach(comedi_device *dev);
static int pcl812_recognize(char *name);

static int i8253_osc_base = 500;	/* 2 Mhz */


comedi_driver driver_pcl812={
	driver_name:	"pcl812",
	module:		THIS_MODULE,
	attach:		pcl812_attach,
	detach:		pcl812_detach,
	recognize:	pcl812_recognize,
};

typedef struct {
	char *name;
	int n_ranges;
	int n_aichan;
	int ai_maxsample;
	int n_aochan;
	int n_dichan;
	int n_dochan;
	comedi_lrange *ai_range_type;
	comedi_lrange *ao_range_type;
	int io_range;
	unsigned int IRQbits;
#ifdef USE_DMA
	unsigned int DMAbits;
#endif
/*	void *ai_mode[4];*/
	
} boardtype;

static boardtype boardtypes[] =
{
	{
	name:		"pcl812pg",
	n_ranges:	5,
	n_aichan:	16,
	ai_maxsample:	30,
	n_aochan:	2,
	n_dichan:	16,
	n_dochan:	16,
	ai_range_type:	&range_pcl812pg_ai,
	ao_range_type:	&range_unipolar5,
	io_range:	PCLx1x_RANGE,
	IRQbits:	0xdcfc,
#ifdef USE_DMA
	DMAbits:	0x00,
#endif
	},
	{
	name:		"pcl813b",
	n_ranges:	4,
	n_aichan:	32,
	ai_maxsample:	25,
	n_aochan:	0,
	n_dichan:	0,
	n_dochan:	0,
	ai_range_type:	&range_pcl813b_ai,
	ao_range_type:	NULL,
	io_range:	PCLx1x_RANGE,
	IRQbits:	0x0000
#ifdef USE_DMA
	DMAbits:	0x00,
#endif
	},
};

#define n_boardtypes (sizeof(boardtypes)/sizeof(boardtype))

typedef struct {
#ifdef USE_DMA
	int dma;
	unsigned long dmabuf[2];
	unsigned int dmapages[2];
	unsigned int hwdmaptr[2];
	unsigned int hwdmasize[2];
	int next_dma_buf;
	unsigned long dma_runs_to_end;
#endif
	int irq_free;
	int irq_blocked;
	int irq_was_now_closed;
	int max_812_ai_mode0_samples;
	int max_812_ai_mode0_rangewait;
	int max_812_ai_mode0_chanset;
	int max_812_ai_mode0_convstart;
	int int812_mode; /*1=AI1 int, 2=AI1 dma, 3=AI3 int, 4AI3 dma */
	//int int13_act_ptr;
	int int13_act_scan;
	int int13_act_chan;
} pcl812_private;

#define devpriv ((pcl812_private *)dev->private)
#define this_board (boardtypes+dev->board)

// *INDENT-ON*
/* 
==============================================================================
   ANALOG INPUT MODE0, 812pg and 813b card
*/
static int pcl812_ai_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int nmax;
	int i, n, p;
	int timeout, hi;

	nmax = devpriv->max_812_ai_mode0_samples;	/* block for max cca 1ms  (812) */

	if ((it->n * it->n_chan) <= nmax)
		nmax = it->n * it->n_chan;

	nmax = nmax / it->n_chan;
	if (!nmax)
		nmax++;

	outb(1, dev->iobase + PCL812_MODE);	/* select software trigger */

	p = 0;			/* ptr to buff */

	for (n = 0; n < nmax; n++) {
		for (i = 0; i < it->n_chan; i++) {
			outb(CR_RANGE(it->chanlist[i]), dev->iobase + PCL812_GAIN);	/* select gain */
			udelay(devpriv->max_812_ai_mode0_rangewait);
			outb(CR_CHAN(it->chanlist[i]), dev->iobase + PCL812_MUX);	/* select channel */
			udelay(devpriv->max_812_ai_mode0_chanset);
			outb(255, dev->iobase + PCL812_SOFTTRIG);	/* start conversion */
			udelay(devpriv->max_812_ai_mode0_convstart);
			timeout = 20;	/* wait max 100us, it must finish under 33us */
			while (timeout--) {
				hi = inb(dev->iobase + PCL812_AD_HI);
				if (!(hi & PCL812_DRDY))
					goto conv_finish;
				udelay(5);
			}
			rt_printk("comedi%d: pcl812: (%s at 0x%x) A/D mode0 timeout\n", dev->minor, dev->board_name, dev->iobase);
			it->data[p++] = 0;
			outb(0, dev->iobase + PCL812_MODE);
			return -ETIME;

		      conv_finish:
			it->data[p++] = ((hi & 0xf) << 8) | inb(dev->iobase + PCL812_AD_LO);;
		}
	}

	outb(0, dev->iobase + PCL812_MODE);
	return p;
}

/* 
==============================================================================
   ANALOG OUTPUT MODE0, 812pg card
   only one sample per call is supported
*/
static int pcl812_ao_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int chan;
	sampl_t data;
	int i;

	for(i=0;i<it->n_chan;i++){
		chan = CR_CHAN(it->chanlist[i]);
		data = it->data[i];

		outb((data & 0xff), dev->iobase + (chan ? PCL812_DA2_LO : PCL812_DA1_LO));
		outb((data >> 8) & 0x0f, dev->iobase + (chan ? PCL812_DA2_HI : PCL812_DA1_HI));
	}

	return i;
}

/* 
==============================================================================
   DIGITAL INPUT MODE0, 812pg card
   
   only one sample per call is supported
*/
static int pcl812_di_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int data;

	data = inb(dev->iobase + PCL812_DI_LO) | (inb(dev->iobase + PCL812_DI_HI) << 8);

	return di_unpack(data,it);
}

/* 
==============================================================================
   DIGITAL OUTPUT MODE0, 812pg card
   
   only one sample per call is supported
*/
static int pcl812_do_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	do_pack(&s->state,it);
	outb(s->state & 0xff, dev->iobase + PCL812_DO_LO);
	outb((s->state >> 8), dev->iobase + PCL812_DO_HI);

	return it->n_chan;
}

/* 
==============================================================================
   analog input interrupt mode 1 & 3, 812pg card
   one sample per interrupt version   
*/
static void interrupt_pcl812_ai_mode13_int(int irq, void *d, struct pt_regs *regs)
{

	int hi;
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;

	int timeout = 20;	/* wait max 100us, it must finish under 33us */
	while (timeout--) {
		hi = inb(dev->iobase + PCL812_AD_HI);
		if (!(hi & PCL812_DRDY))
			goto conv_finish;
		udelay(5);
	}
	hi = inb(dev->iobase + PCL812_AD_LO);
	outb(0, dev->iobase + PCL812_CLRINT);	/* clear INT request */
	rt_printk("comedi%d: pcl812: (%s at 0x%x) A/D mode1/3 IRQ without DRDY!\n", dev->minor, dev->board_name, dev->iobase);
	comedi_done(dev,s);
	return;

      conv_finish:

	s->cur_trig.data[s->buf_int_ptr++] = ((hi << 8) | inb(dev->iobase + PCL812_AD_LO)) & 0xfff;

	outb(0, dev->iobase + PCL812_CLRINT);	/* clear INT request */

	s->buf_int_count += sizeof(sampl_t);

	if ((++devpriv->int13_act_chan) >= s->cur_trig.n_chan) {	/* one scan done */
		devpriv->int13_act_chan = 0;
		outb(CR_RANGE(s->cur_trig.chanlist[devpriv->int13_act_chan]), dev->iobase + PCL812_GAIN);	/* select next gain */
		outb(CR_CHAN(s->cur_trig.chanlist[devpriv->int13_act_chan]), dev->iobase + PCL812_MUX);	/* select next channel */
		if (s->cur_trig.flags & TRIG_WAKE_EOS) {
			comedi_eos(dev, s);
		} else {
			comedi_bufcheck(dev, s);
		}
		devpriv->int13_act_scan++;
	} else {
		outb(CR_RANGE(s->cur_trig.chanlist[devpriv->int13_act_chan]), dev->iobase + PCL812_GAIN);	/* select next gain */
		outb(CR_CHAN(s->cur_trig.chanlist[devpriv->int13_act_chan]), dev->iobase + PCL812_MUX);	/* select next channel */
	}

	if (s->buf_int_ptr >= s->cur_trig.data_len) {	/* buffer rollover */
		s->buf_int_ptr = 0;
		//devpriv->int13_act_ptr=0;
		comedi_eobuf(dev, s);
	}

	if (devpriv->int13_act_scan >= s->cur_trig.n) {	/* all data sampled */
		outb(0, dev->iobase + PCL812_MODE);	/* Stop A/D */
		outb(0, dev->iobase + PCL812_CLRINT);	/* clear INT request */
		if (devpriv->int812_mode == 1) {
			/* Stop pacer */
			outb(0xb4, dev->iobase + PCL812_CTRCTL);
			outb(0x74, dev->iobase + PCL812_CTRCTL);
		}
		s->busy = 0;
		devpriv->irq_blocked = 0;
		devpriv->int812_mode = 0;
		devpriv->irq_was_now_closed = 1;
		/* printk("comedi_done\n"); */
		comedi_done(dev, s);
	}
}


/* 
==============================================================================
    INT procedure
*/
static void interrupt_pcl812(int irq, void *d, struct pt_regs *regs)
{

	comedi_device *dev = d;

	if ((!dev->irq) | (!devpriv->irq_free) | (!devpriv->irq_blocked) | (!devpriv->int812_mode)) {
		if (devpriv->irq_was_now_closed) {
			devpriv->irq_was_now_closed = 0;
			outb(0, dev->iobase + PCL812_CLRINT);	/* clear INT request */
			rt_printk("comedi%d: pcl812: (%s at 0x%x) too much IRQs!\n", dev->minor, dev->board_name, dev->iobase);
			return;
		}
		rt_printk("comedi%d: pcl812: (%s at 0x%x) bad IRQ!\n", dev->minor, dev->board_name, dev->iobase);
		return;
	}

	switch (devpriv->int812_mode) {
	case INT_TYPE_AI1_INT:
		interrupt_pcl812_ai_mode13_int(irq, d, regs);
		return;
	case INT_TYPE_AI3_INT:
		interrupt_pcl812_ai_mode13_int(irq, d, regs);
		return;
#if USE_DMA
	case INT_TYPE_AI1_DMA:
		interrupt_pcl812_ai_mode13_dma(irq, d, regs);
		return;
	case INT_TYPE_AI3_DMA:
		interrupt_pcl812_ai_mode13_dma(irq, d, regs);
		return;
#endif
	}
}

/* 
==============================================================================
   ANALOG INPUT MODE 1, 812pg card
   interrupt pacer pooling
*/
static int pcl812_ai_mode1_int(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int timer1,timer2;

	/*
	 *  Set timers
	 *    timer chip is an 8253, with timers 1 and 2
	 *    cascaded
	 *  0x74 = Select Counter 1 | LSB/MSB | Mode=2 | Binary
	 *        Mode 2 = Rate generator
	 *
	 *  0xb4 = Select Counter 2 | LSB/MSB | Mode=2 | Binary
	 */

	i8253_cascade_ns_to_timer(i8253_osc_base,&timer1,&timer2,&it->trigvar,TRIG_ROUND_NEAREST);

	outb(0x74, dev->iobase + PCL812_CTRCTL);
	outb((timer1) & 0xff, dev->iobase + PCL812_CTR1);
	outb((timer1 >> 8) & 0xff, dev->iobase + PCL812_CTR1);
	outb(0xb4, dev->iobase + PCL812_CTRCTL);
	outb(timer2 & 0xff, dev->iobase + PCL812_CTR2);
	outb((timer2 >> 8) & 0xff, dev->iobase + PCL812_CTR2);

	/* clear pending interrupts (just in case) */
	outb(0, dev->iobase + PCL812_CLRINT);

	//devpriv->int13_act_ptr=0;
	devpriv->int13_act_scan = 0;
	devpriv->int13_act_chan = 0;
	devpriv->int812_mode = INT_TYPE_AI1_INT;	/* analog in, mode 0, int driven */
	devpriv->irq_blocked = 1;
	devpriv->irq_was_now_closed = 0;

	outb(6, dev->iobase + PCL812_MODE);	/* Pacer+IRQ */

	return 0;
}

/* 
==============================================================================
   ANALOG INPUT MODE 1, 812pg card
*/
static int pcl812_ai_mode1(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{

	if (!dev->irq)
		return -EINVAL;
	if (devpriv->irq_blocked)
		return -EBUSY;
	if (it->n_chan < 0)
		return -1;

#ifdef USE_DMA
	if (devpriv->dma) {	/* check if we can use DMA? */
		if (it->n_chan == 1) {
			return pcl812_ai_mode1_dma(dev, s, it);	/* we scanning only one chan, we can */
		} else {
			fst = it->chanlist[0];
			for (i = 1; i < it->n_chan; i++) {
				if (fst != it->chanlist[0]) {
					i = -1;
					break;
				}
			}
			if (i == -1) {
				return pcl812_ai_mode1_int(dev, s, it);
			} else {
				return pcl812_ai_mode1_dma(dev, s, it);
			}
		}
	}
#endif
	return pcl812_ai_mode1_int(dev, s, it);	/* no, we can only int driven */
}

/* 
==============================================================================
   ANALOG INPUT MODE 3, 812pg card
*/
static int pcl812_ai_mode3_int(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{

	if (!dev->irq)
		return -EINVAL;
	if (devpriv->irq_blocked)
		return -EBUSY;

	/* clear pending interrupts (just in case) */
	outb(0, dev->iobase + PCL812_CLRINT);

	//devpriv->int13_act_ptr=0;
	devpriv->int13_act_scan = 0;
	devpriv->int13_act_chan = 0;
	devpriv->int812_mode = 3;	/* analog in, mode 3, int driven */
	devpriv->irq_blocked = 1;

	outb(6, dev->iobase + PCL812_MODE);	/* external trigger+IRQ */

	return 0;
}

/* 
==============================================================================
  Free any resources that we have claimed  
*/
static void free_resources(comedi_device * dev)
{

	if (dev->irq)
		free_irq(dev->irq, dev);
	if (dev->iobase)
		release_region(dev->iobase, dev->iosize);
#ifdef USE_DMA
	if (dev->private) {
		if (devpriv->dmabuf[0])
			free_pages(devpriv->dmabuf[0], devpriv->dmapages[0]);
		if (devpriv->dmabuf[1])
			free_pages(devpriv->dmabuf[1], devpriv->dmapages[1]);
		if (devpriv->dma)
			free_dma(devpriv->dma);
	}
#endif
}

/* 
==============================================================================
 reset whole PCL-812 or PCL-813 
*/
static void pcl812_reset(comedi_device * dev)
{
	if (dev->board == boardPCL812PG) {
		outb(0, dev->iobase + PCL812_DA1_LO);
		outb(0, dev->iobase + PCL812_DA1_HI);
		outb(0, dev->iobase + PCL812_DA2_LO);
		outb(0, dev->iobase + PCL812_DA2_HI);
		outb(0, dev->iobase + PCL812_DO_HI);
		outb(0, dev->iobase + PCL812_DO_LO);
		outb(0, dev->iobase + PCL812_MODE);
		outb(0, dev->iobase + PCL812_CLRINT);
	}
	outb(0, dev->iobase + PCL812_GAIN);
	outb(0, dev->iobase + PCL812_MUX);
	udelay(5);
	if (dev->board == boardPCL813B) {
#ifdef PCL813_MICROSECS
		udelay(5);
#else
		udelay(5000);
#endif
	}
}


/* 
==============================================================================

   Initialization 

*/
static int pcl812_attach(comedi_device * dev, comedi_devconfig * it)
{

	int ret;
	int iobase;
	int irq;
#ifdef USE_DMA
	int dma;
	unsigned long pages;
#endif
	int board;
	comedi_subdevice *s;
	int num_of_subdevs, subdevs[5];

	board = dev->board;	/* inicialized from pcl812_recognize()? */

	/* claim our I/O space */
	iobase = it->options[0];
	printk("comedi%d: pcl812:  board=%s, ioport=0x%03x", dev->minor, boardtypes[board].name, iobase);
	if (check_region(iobase, boardtypes[board].io_range) < 0) {
		printk("I/O port conflict\n");
		return -EIO;
	}
	request_region(iobase, boardtypes[board].io_range, "pcl812");
	dev->iobase = iobase;
	dev->iosize = boardtypes[board].io_range;

	/* there should be a sanity check here */

	if ((ret = alloc_private(dev, sizeof(pcl812_private))) < 0)
		return ret;	/* Can't alloc mem */

	/* set up some name stuff */
	dev->board_name = boardtypes[board].name;

	/* grab our IRQ */
	irq = 0;
	if (boardtypes[board].IRQbits != 0) {	/* board support IRQ */
		irq = it->options[1];
		if (irq) {	/* we want to use IRQ */
			if (((1 << irq) & boardtypes[board].IRQbits) == 0) {
				printk(", IRQ %d is out of allowed range, DISABLING IT", irq);
				irq = 0;	/* Bad IRQ */
			} else {
				if (request_irq(irq, interrupt_pcl812, SA_INTERRUPT, "pcl812", dev)) {
					printk(", unable to allocate IRQ %d, DISABLING IT", irq);
					irq = 0;	/* Can't use IRQ */
				} else {
					printk(", irq=%d", irq);
				}
			}
		}
	}

	dev->irq = irq;
	if (irq) {
		devpriv->irq_free = 1;
	} /* 1=we have allocated irq */
	else {
		devpriv->irq_free = 0;
	}
	devpriv->irq_blocked = 0;	/* number of subdevice which use IRQ */
	devpriv->int812_mode = 0;	/* mode of irq */

#ifdef USE_DMA
	/* grab our DMA */
	dma = 0;
	devpriv->dma = dma;
	if (!devpriv->irq_free)
		goto no_dma;	/* if we haven't IRQ, we can't use DMA */
	if (boardtypes[board].DMAbits != 0) {	/* board support DMA */
		dma = it->options[2];
		if (((1 << dma) & boardtypes[board].DMAbits) == 0) {
			printk(", DMA is out of allowed range, FAIL!\n");
			return -EINVAL;	/* Bad DMA */
		}
		ret = request_dma(dma, "pcl812");
		if (ret) {
			printk(", unable to allocate DMA %d, FAIL!\n", dma);
			return -EBUSY;	/* DMA isn't free */
		}
		devpriv->dma = dma;
		printk(", dma=%d", dma);
		pages = 1;	/* we need 8KB */
		devpriv->dmabuf[0] = __get_dma_pages(GFP_KERNEL, pages);
		if (!devpriv->dmabuf[0]) {
			printk(", unable to allocate DMA buffer, FAIL!\n");
			/* maybe experiment with try_to_free_pages() will help .... */
			return -EBUSY;	/* no buffer :-( */
		}
		devpriv->dmapages[0] = pages;
		devpriv->hwdmaptr[0] = virt_to_bus((void *) devpriv->dmabuf[0]);
		devpriv->hwdmasize[0] = PAGE_SIZE * 2;
		devpriv->dmabuf[1] = __get_dma_pages(GFP_KERNEL, pages);
		if (!devpriv->dmabuf[1]) {
			printk(", unable to allocate DMA buffer, FAIL!\n");
			return -EBUSY;
		}
		devpriv->dmapages[1] = pages;
		devpriv->hwdmaptr[1] = virt_to_bus((void *) devpriv->dmabuf[1]);
		devpriv->hwdmasize[1] = PAGE_SIZE * 2;
	}
      no_dma:
#endif

	num_of_subdevs = 0;

	/*if (!((board==boardPCL812PG)&&(it->options[3]==1))) { */
	subdevs[num_of_subdevs++] = COMEDI_SUBD_AI;
	/* } */
	if (this_board->n_aochan > 0)
		subdevs[num_of_subdevs++] = COMEDI_SUBD_AO;
	if (this_board->n_dichan > 0)
		subdevs[num_of_subdevs++] = COMEDI_SUBD_DI;
	if (this_board->n_dochan > 0)
		subdevs[num_of_subdevs++] = COMEDI_SUBD_DO;

	dev->n_subdevices = 4;
	if ((ret = alloc_subdevices(dev)) < 0)
		return ret;

	/* analog input */
	s = dev->subdevices + 0;
	if (this_board->n_aichan == 0) {
		s->type = COMEDI_SUBD_UNUSED;
	} else {
		s->type = COMEDI_SUBD_AI;
		s->subdev_flags = SDF_READABLE;
		s->n_chan = this_board->n_aichan;
		s->maxdata = 0xfff;
		s->len_chanlist = 1024;
		s->range_table = this_board->ai_range_type;
		switch (board) {
		case boardPCL812PG:
			s->subdev_flags |= SDF_GROUND;
			s->trig[0] = pcl812_ai_mode0;
			if (it->options[3] == 1)
				s->range_table = &range_pcl812pg2_ai;
			if (dev->irq) {
				if (it->options[2] != 1) {
					s->trig[1] = pcl812_ai_mode1;
				} else {
					s->trig[3] = pcl812_ai_mode3_int;
				}
			}
			break;
		case boardPCL813B:
			s->subdev_flags |= SDF_GROUND;
			if (it->options[1] == 1)
				s->range_table = &range_pcl813b2_ai;
			s->trig[0] = pcl812_ai_mode0;
			break;
		}
	}

	/* analog output */
	s = dev->subdevices + 1;
	if (this_board->n_aochan == 0) {
		s->type = COMEDI_SUBD_UNUSED;
	} else {
		s->type = COMEDI_SUBD_AO;
		s->subdev_flags = SDF_WRITEABLE;
		s->n_chan = this_board->n_aochan;
		s->maxdata = 0xfff;
		s->len_chanlist = 1;
		s->range_table = this_board->ao_range_type;
		switch (board) {
		case boardPCL812PG:
			s->subdev_flags |= SDF_GROUND;
			s->trig[0] = pcl812_ao_mode0;
			//s->trig[1] = pcl812_ao_mode1;
			if (it->options[4] == 1)
				s->range_table = &range_unipolar5;
			if (it->options[4] == 2)
				s->range_table = &range_unknown;
			break;
		}
	}

	/* digital input */
	s = dev->subdevices + 2;
	if (this_board->n_dichan == 0) {
		s->type = COMEDI_SUBD_UNUSED;
	} else {
		s->type = COMEDI_SUBD_DI;
		s->subdev_flags = SDF_READABLE;
		s->n_chan = this_board->n_dichan;
		s->maxdata = 1;
		s->len_chanlist = this_board->n_dichan;
		s->range_table = &range_digital;
		switch (board) {
		case boardPCL812PG:
			s->trig[0] = pcl812_di_mode0;
			break;
		}
	}

	/* digital output */
	s = dev->subdevices + 3;
	if (this_board->n_dochan == 0) {
		s->type = COMEDI_SUBD_UNUSED;
	} else {
		s->type = COMEDI_SUBD_DO;
		s->subdev_flags = SDF_WRITEABLE;
		s->n_chan = this_board->n_dochan;
		s->maxdata = 1;
		s->len_chanlist = this_board->n_dochan;
		s->range_table = &range_digital;
		switch (board) {
		case boardPCL812PG:
			s->trig[0] = pcl812_do_mode0;
			break;
		}
	}

	/*dev->rem = pcl812_rem; */

	switch (dev->board) {
	case boardPCL812PG:
		pcl812_reset(dev);
		devpriv->max_812_ai_mode0_samples = 32;
		devpriv->max_812_ai_mode0_rangewait = 1;
		devpriv->max_812_ai_mode0_chanset = 1;
		devpriv->max_812_ai_mode0_convstart = 5;
		break;
	case boardPCL813B:
		pcl812_reset(dev);
		if (it->options[2] < 2) {
			devpriv->max_812_ai_mode0_samples = 1;
		} else {
			devpriv->max_812_ai_mode0_samples = it->options[2];
		}
#ifdef PCL813_MICROSECS
		devpriv->max_812_ai_mode0_rangewait = 1;	/* maybe there must by greatest timeout */
		devpriv->max_812_ai_mode0_chanset = 5;
		devpriv->max_812_ai_mode0_convstart = 20;
#else
		devpriv->max_812_ai_mode0_rangewaint = 1;
		devpriv->max_812_ai_mode0_chanset = 5000;
		devpriv->max_812_ai_mode0_convstart = 20000;
#endif
		break;
	}
	printk("\n");
	return 0;
}


/*
==============================================================================
  Removes device
 */
static int pcl812_detach(comedi_device * dev)
{

#ifdef MD_DEBUG
	printk("comedi%d: pcl812: remove\n", dev->minor);
#endif
	free_resources(dev);
	return 0;
}

static int pcl812_recognize(char *name)
{

	int i;

#ifdef MD_DEBUG
	printk("comedi: pcl812: recognize code '%s'\n", name);
#endif
	for (i = 0; i < n_boardtypes; i++) {
		if (!strcmp(boardtypes[i].name, name)) {
#ifdef MD_DEBUG
			printk("comedi: pcl812: recognize found '%s'\n", boardtypes[i].name);
#endif
			return i;
		}
	}
	return -1;
}

/*  
==============================================================================
*/
#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_pcl812);

	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_pcl812);
}

#endif


#if 0
/* @@Kluvi: magic crystaline sphere error correction, I hope
   that work identicaly under C and TP :-) */
static void Zisti_Div8254(long celk, long *d1, long *d2)
{
	long minch, chyba, mini, i;

	minch = 32000;
	mini = 1;
	if (celk <= (65536 * 2)) {
		i = 2;
	} else {
		i = celk / 65536;
	}
	do {
		*d1 = i;
		*d2 = celk / *d1;
		if (*d2 < 65536) {
			chyba = celk - *d1 * *d2;
			if (chyba < 0) {
				chyba = -chyba;
			}
			if (chyba < minch) {
				minch = chyba;
				mini = i;
				if (chyba == 0)
					break;
			}
		}
		i++;
	} while ((i < 65536) && (*d2 > 2) && (*d2 > *d1));
	i = mini;
	*d1 = i;
	*d2 = celk / *d1;
}

static int pcl812_timer(double freq, unsigned int *trigvar, double *actual_freq)
{
	long divisor1, divisor2, divid;
	double Oscilator = 2e6;

	if (freq < 0.0004) {
		freq = 0.0004;
	}
	if (freq > 30000) {
		freq = 30000;
	}
	divid = rint(Oscilator / freq);
	Zisti_Div8254(divid, &divisor1, &divisor2);
	divid = rint(Oscilator / freq + 0.5 * (divid - divisor1 * divisor2));
	Zisti_Div8254(divid, &divisor1, &divisor2);
	*actual_freq = Oscilator / (divisor1 * divisor2);

	*trigvar = (divisor1 << 16) | divisor2;
	return 0;
}

#endif


