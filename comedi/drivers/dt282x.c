/*
   modules/dt282x.c
   hardware driver for Data Translation DT2821 series

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


#include <comedi_module.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/malloc.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timex.h>
#include <linux/timer.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/dma.h>

#define DEBUG

#define DT2821_TIMEOUT		100	/* 500 us */
#define DT2821_SIZE 0x10

/*
 *    Registers in the DT282x
 */

#define DT2821_ADCSR	0x00	/* A/D Control/Status             */
#define DT2821_CHANCSR	0x02	/* Channel Control/Status */
#define DT2821_ADDAT	0x04	/* A/D data                       */
#define DT2821_DACSR	0x06	/* D/A Control/Status             */
#define DT2821_DADAT	0x08	/* D/A data                       */
#define DT2821_DIODAT	0x0a	/* digital data                   */
#define DT2821_SUPCSR	0x0c	/* Supervisor Control/Status      */
#define DT2821_TMRCTR	0x0e	/* Timer/Counter          */

/*
 *  At power up, some registers are in a well-known state.  The
 *  masks and values are as follows:
 */

#define DT2821_ADCSR_MASK 0xfff0
#define DT2821_ADCSR_VAL 0x7c00

#define DT2821_CHANCSR_MASK 0xf0f0
#define DT2821_CHANCSR_VAL 0x70f0

#define DT2821_DACSR_MASK 0x7c93
#define DT2821_DACSR_VAL 0x7c90

#define DT2821_SUPCSR_MASK 0xf8ff
#define DT2821_SUPCSR_VAL 0x0000

#define DT2821_TMRCTR_MASK 0xff00
#define DT2821_TMRCTR_VAL 0xf000

/*
 *    Bit fields of each register
 */

/* ADCSR */

#define DT2821_ADERR	0x8000	/* (R)   1 for A/D error  */
#define DT2821_ADCLK	0x0200	/* (R/W) A/D clock enable */
		/*      0x7c00           read as 1's            */
#define DT2821_MUXBUSY	0x0100	/* (R)   multiplexer busy */
#define DT2821_ADDONE	0x0080	/* (R)   A/D done         */
#define DT2821_IADDONE	0x0040	/* (R/W) interrupt on A/D done    */
		/*      0x0030           gain select            */
		/*      0x000f           channel select         */

/* CHANCSR */

#define DT2821_LLE	0x8000	/* (R/W) Load List Enable */
		/*      0x7000           read as 1's            */
		/*      0x0f00     (R)   present address        */
		/*      0x00f0           read as 1's            */
		/*      0x000f     (R)   number of entries - 1  */

/* DACSR */

#define DT2821_DAERR	0x8000	/* (R)   D/A error                */
#define DT2821_YSEL	0x0200	/* (R/W) DAC 1 select             */
#define DT2821_SSEL	0x0100	/* (R/W) single channel select    */
#define DT2821_DACRDY	0x0080	/* (R)   DAC ready                */
#define DT2821_IDARDY	0x0040	/* (R/W) interrupt on DAC ready   */
#define DT2821_DACLK	0x0020	/* (R/W) D/A clock enable */
#define DT2821_HBOE	0x0002	/* (R/W) DIO high byte output enable      */
#define DT2821_LBOE	0x0001	/* (R/W) DIO low byte output enable       */

/* SUPCSR */

#define DT2821_DMAD	0x8000	/* (R)   DMA done                 */
#define DT2821_ERRINTEN	0x4000	/* (R/W) interrupt on error               */
#define DT2821_CLRDMADNE 0x2000	/* (W)   clear DMA done                   */
#define DT2821_DDMA	0x1000	/* (R/W) dual DMA                 */
#define DT2821_DS1	0x0800	/* (R/W) DMA select 1                     */
#define DT2821_DS0	0x0400	/* (R/W) DMA select 0                     */
#define DT2821_BUFFB	0x0200	/* (R/W) buffer B selected                */
#define DT2821_SCDN	0x0100	/* (R)   scan done                        */
#define DT2821_DACON	0x0080	/* (W)   DAC single conversion            */
#define DT2821_ADCINIT	0x0040	/* (W)   A/D initialize                   */
#define DT2821_DACINIT	0x0020	/* (W)   D/A initialize                   */
#define DT2821_PRLD	0x0010	/* (W)   preload multiplexer              */
#define DT2821_STRIG	0x0008	/* (W)   software trigger         */
#define DT2821_XTRIG	0x0004	/* (R/W) external trigger enable  */
#define DT2821_XCLK	0x0002	/* (R/W) external clock enable            */
#define DT2821_BDINIT	0x0001	/* (W)   initialize board         */

/*
--BEGIN-RANGE-DEFS--
RANGE_dt282x_ai_lo_bipolar
        -10     10
        -5      5
        -2.5    2.5
        -1.25   1.25
RANGE_dt282x_ai_lo_unipolar
        0       10
        0       5
        0       2.5
        0       1.25
RANGE_dt282x_ai_5_bipolar
       -5      5
       -2.5    2.5
       -1.25   1.25
       -0.625  0.625
RANGE_dt282x_ai_5_unipolar
        0       5
        0       2.5
        0       1.25
        0       0.625
RANGE_dt282x_ai_hi_bipolar
        -10     10
        -1      1
        -0.1    0.1
        -0.02   0.02
RANGE_dt282x_ai_hi_unipolar
        0       10
        0       1
        0       0.1
        0       0.02
RANGE_bipolar2_5
	-2.5	2.5
---END-RANGE-DEFS---
*/

typedef struct {
	char *name;
	int adbits;
	int adchan_se;
	int adchan_di;
	int ispgl;
	int dachan;
	int dabits;
} boardtype_t;

static boardtype_t boardtypes[] =
{
	{	name:		"dt2821",
		adbits:		12,
		adchan_se:	16,
		adchan_di:	8,
		ispgl:		0,
		dachan:		2,
		dabits:		12,
	},
	{	name:		"dt2823",
		adbits:		16,
		adchan_se:	0,
		adchan_di:	4,
		ispgl:		0,
		dachan:		2,
		dabits:		16,
	},
	{	name:		"dt2824-pgh",
		adbits:		16,
		adchan_se:	16,
		adchan_di:	8,
		ispgl:		0,
		dachan:		0,
		dabits:		0,
	},
	{	name:		"dt2824-pgl",
		adbits:		16,
		adchan_se:	16,
		adchan_di:	8,
		ispgl:		1,
		dachan:		0,
		dabits:		0,
	},
	{	name:		"dt2825",
		adbits:		12,
		adchan_se:	16,
		adchan_di:	8,
		ispgl:		0,
		dachan:		2,
		dabits:		12,
	},
	{	name:		"dt2827",
		adbits:		16,
		adchan_se:	0,
		adchan_di:	4,
		ispgl:		0,
		dachan:		2,
		dabits:		12,
	},
	{	name:		"dt2828",
		adbits:		12,
		adchan_se:	4,
		adchan_di:	0,
		ispgl:		0,
		dachan:		2,
		dabits:		12,
	},
	{	name:		"dt21-ez",
		adbits:		12,
		adchan_se:	16,
		adchan_di:	8,
		ispgl:		0,
		dachan:		2,
		dabits:		12,
	},
	{	name:		"dt23-ez",
		adbits:		16,
		adchan_se:	16,
		adchan_di:	8,
		ispgl:		0,
		dachan:		0,
		dabits:		0,
	},
	{	name:		"dt24-ez",
		adbits:		12,
		adchan_se:	16,
		adchan_di:	8,
		ispgl:		0,
		dachan:		0,
		dabits:		0,
	},
	{	name:		"dt24-ez-pgl",
		adbits:		12,
		adchan_se:	16,
		adchan_di:	8,
		ispgl:		1,
		dachan:		0,
		dabits:		0,
	},
};
static int n_boardtypes = sizeof(boardtypes)/sizeof(boardtype_t);


typedef struct {
	int ad_2scomp;		/* we have 2's comp jumper set  */
	int da0_2scomp;		/* same, for DAC0               */
	int da1_2scomp;		/* same, for DAC1               */

	int darangelist[2];

	int dacsr;		/* software copies of registers */
	int adcsr;
	int supcsr;

	int ntrig;
	int nread;

	struct{
		int chan;
		short *buf;	/* DMA buffer */
		int size;	/* size of current transfer */
	}dma[2];
	int dma_maxsize;	/* max size of DMA transfer     */
	int usedma;		/* driver uses DMA              */
	int current_dma_chan;
	int dma_dir;
} dt282x_private;

#define devpriv ((dt282x_private *)dev->private)
#define boardtype (*(boardtype_t *)dev->board_ptr)

/*
 *    Some useless abstractions
 */
#define chan_to_DAC(a)	((a)&1)
#define update_dacsr(a)	outw(devpriv->dacsr|(a),dev->iobase+DT2821_DACSR)
#define update_adcsr(a)	outw(devpriv->adcsr|(a),dev->iobase+DT2821_ADCSR)
#define mux_busy() (inw(dev->iobase+DT2821_ADCSR)&DT2821_MUXBUSY)
#define ad_done() (inw(dev->iobase+DT2821_ADCSR)&DT2821_ADDONE)
#define update_supcsr(a)	outw(devpriv->supcsr|(a),dev->iobase+DT2821_SUPCSR)

/*
 *    danger! macro abuse... a is the expression to wait on, and b is
 *      the statement(s) to execute if it doesn't happen.
 */
#define wait_for(a,b)	 				\
	do{						\
		int _i;					\
		for(_i=0;_i<DT2821_TIMEOUT;_i++){	\
			if(a){_i=0;break;}		\
			udelay(5);			\
		}					\
		if(_i){b}				\
	}while(0)

static int dt282x_attach(comedi_device * dev, comedi_devconfig * it);
static int dt282x_detach(comedi_device * dev);
static int dt282x_recognize(char *name);
comedi_driver driver_dt282x={
	driver_name:	"dt282x",
	module:		&__this_module,
	attach:		dt282x_attach,
	detach:		dt282x_detach,
	recognize:	dt282x_recognize,
};

static void free_resources(comedi_device *dev);
static int prep_ai_dma(comedi_device * dev,int chan,int size);
static int prep_ao_dma(comedi_device * dev,int chan,int size);
static int dt282x_ai_cancel(comedi_device * dev, comedi_subdevice * s);
static int dt282x_ao_cancel(comedi_device * dev, comedi_subdevice * s);
static int dt282x_ns_to_timer(int *nanosec);


static int dt282x_grab_dma(comedi_device *dev,int dma1,int dma2);

static void dt282x_cleanup_buffer(comedi_device *dev,unsigned short *buf,unsigned int len)
{
	unsigned int i;
	unsigned short mask=(1<<boardtype.adbits)-1;
	unsigned short sign=1<<(boardtype.adbits-1);

	if(devpriv->ad_2scomp){
		for(i=0;i<len;i++){
			buf[i]&=mask;
			buf[i]^=sign;
		}
	}else{
		for(i=0;i<len;i++){
			buf[i]&=mask;
		}
	}
}

static void copy_to_buf(comedi_device *dev,comedi_subdevice *s,void *buf,unsigned int n_bytes)
{
	unsigned int n;

	n=n_bytes;
	if(s->buf_int_ptr+n >= s->cur_trig.data_len){
		n=s->cur_trig.data_len-s->buf_int_ptr;
		memcpy(((void *)(s->cur_trig.data))+s->buf_int_ptr,buf,n);
		buf+=n;
		s->buf_int_count+=n;
		s->buf_int_ptr=0;

		n=n_bytes-n;
	}
	memcpy(((void *)(s->cur_trig.data))+s->buf_int_ptr,buf,n);
	buf+=n;
	s->buf_int_count+=n;
	s->buf_int_ptr+=n;
}

static int copy_from_buf(comedi_device *dev,comedi_subdevice *s,void *buf,unsigned int n_bytes)
{
	unsigned int n,m;

	n=s->buf_int_count-s->buf_user_count;
	if(n==0)return 0;
	if(n>n_bytes)
		n=n_bytes;

	n_bytes=n;
	if(s->buf_int_ptr+n >= s->cur_trig.data_len){
		m=s->cur_trig.data_len-s->buf_int_ptr;
		memcpy(buf,((void *)(s->cur_trig.data))+s->buf_int_ptr,m);
		buf+=m;
		s->buf_int_count+=m;
		s->buf_int_ptr=0;

		n-=m;
	}
	memcpy(buf,((void *)(s->cur_trig.data))+s->buf_int_ptr,n);
	s->buf_int_count+=n;
	s->buf_int_ptr+=n;

	return n_bytes;
}


static void dt282x_ao_dma_interrupt(comedi_device * dev)
{
	void *ptr;
	int size;
	int i;
	comedi_subdevice *s=dev->subdevices+1;

	update_supcsr(DT2821_CLRDMADNE);

	if(!s->cur_trig.data){
		printk("cur_trig.data disappeared.  dang!\n");
		return;
	}

	i=devpriv->current_dma_chan;
	ptr=devpriv->dma[i].buf;

	disable_dma(devpriv->dma[i].chan);

	devpriv->current_dma_chan=1-i;

	size=copy_from_buf(dev,s,ptr,devpriv->dma_maxsize*2);
	if(!size){
		printk("dt282x: AO underrun\n");
		dt282x_ao_cancel(dev,s);
		comedi_done(dev,s);
		return;
	}
	prep_ao_dma(dev,i,size/2);

	enable_dma(devpriv->dma[i].chan);

	comedi_bufcheck(dev,s);
	return;
}

static void dt282x_ai_dma_interrupt(comedi_device * dev)
{
	void *ptr;
	int size;
	int i;
	comedi_subdevice *s=dev->subdevices;

	update_supcsr(DT2821_CLRDMADNE);

	if(!s->cur_trig.data){
		printk("cur_trig.data disappeared.  dang!\n");
		return;
	}

	i=devpriv->current_dma_chan;
	ptr=devpriv->dma[i].buf;
	size=devpriv->dma[i].size;

	disable_dma(devpriv->dma[i].chan);

	devpriv->current_dma_chan=1-i;
	dt282x_cleanup_buffer(dev,ptr,size);
	copy_to_buf(dev,s,ptr,size*2);
	devpriv->nread-=size;

	if(devpriv->nread<0){
		printk("dt282x: off by one\n");
		devpriv->nread=0;
	}
	if (!devpriv->nread) {
		devpriv->adcsr=0;
		update_adcsr(0);

		/* this eliminates "extra sample" issues */
		devpriv->supcsr = 0;
		update_supcsr(DT2821_ADCINIT);

		comedi_done(dev,s);

		return;
	}else{
		comedi_bufcheck(dev,s);
	}

#if 1
	/* clear the dual dma flag, making this the last dma segment */
	/* XXX probably wrong */
	if(!devpriv->ntrig){
		devpriv->supcsr &= ~(DT2821_DDMA);
		update_supcsr(0);
	}
#endif
	/* restart the channel */
	prep_ai_dma(dev,i,0);

	enable_dma(devpriv->dma[i].chan);

	return;
}

static int prep_ai_dma(comedi_device * dev,int chan,int n)
{
	int dma_chan;
	unsigned long dma_ptr;
	unsigned long flags;

	if(!devpriv->ntrig)
		return 0;

	if(n==0)
		n = devpriv->dma_maxsize;
	if (n >= devpriv->ntrig)
		n = devpriv->ntrig;
	devpriv->ntrig -= n;

	devpriv->dma[chan].size = n;
	dma_chan = devpriv->dma[chan].chan;
	dma_ptr = virt_to_bus(devpriv->dma[chan].buf);

	set_dma_mode(dma_chan, DMA_MODE_READ);
	flags=claim_dma_lock();
	set_dma_addr(dma_chan, dma_ptr);
	set_dma_count(dma_chan, n << 1);
	release_dma_lock(flags);

	return n;
}

static int prep_ao_dma(comedi_device * dev,int chan,int n)
{
	int dma_chan;
	unsigned long dma_ptr;
	unsigned long flags;

	devpriv->dma[chan].size = n;
	dma_chan = devpriv->dma[chan].chan;
	dma_ptr = virt_to_bus(devpriv->dma[chan].buf);

	set_dma_mode(dma_chan, DMA_MODE_WRITE);
	flags=claim_dma_lock();
	set_dma_addr(dma_chan, dma_ptr);
	set_dma_count(dma_chan, n*2 );
	release_dma_lock(flags);

	return n;
}

static void dt282x_interrupt(int irq, void *d, struct pt_regs *regs)
{
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices+0;
	unsigned int supcsr, adcsr, dacsr;
	sampl_t data;

	adcsr=inw(dev->iobase + DT2821_ADCSR);
	if (adcsr & DT2821_ADERR) {
		comedi_error(dev, "A/D error");
		dt282x_ai_cancel(dev,s);
		comedi_done(dev,s);
		return;
	}
	supcsr = inw(dev->iobase + DT2821_SUPCSR);
	/*printk("supcsr=%02x\n",supcsr);*/
	if (supcsr & DT2821_DMAD) {
		if(devpriv->dma_dir==DMA_MODE_READ)
			dt282x_ai_dma_interrupt(dev);
		else
			dt282x_ao_dma_interrupt(dev);
		return;
	}
	if ((dacsr = inw(dev->iobase + DT2821_DACSR)) & DT2821_DAERR) {
#if 0
		static int warn = 5;
		if(--warn<=0){
			disable_irq(dev->irq);
			printk("disabling irq\n");
		}
#endif
		comedi_error(dev, "D/A error");
		dt282x_ao_cancel(dev,s);
		comedi_done(dev,s);
		return;
	}
	if (adcsr & DT2821_ADDONE) {
		data = (sampl_t) inw(dev->iobase + DT2821_ADDAT);
		data&=(1<<boardtype.adbits)-1;
		if(devpriv->ad_2scomp){
			data^=1<<(boardtype.adbits-1);
		}
		s->cur_trig.data[s->buf_int_ptr++]=data;

		devpriv->nread--;
		if(!devpriv->nread){
			comedi_done(dev,s);
		}else{
			if(supcsr&DT2821_SCDN)
				update_supcsr(DT2821_STRIG);
		}

		return;
	}
}


static void dt282x_load_changain(comedi_device * dev, int n, unsigned int *chanlist)
{
	unsigned int i;
	unsigned int chan, range;

	outw(DT2821_LLE | (n - 1), dev->iobase + DT2821_CHANCSR);
	for (i = 0; i < n; i++) {
		chan = CR_CHAN(chanlist[i]);
		range = CR_RANGE(chanlist[i]);
		update_adcsr((range << 4) | (chan));
	}
	outw(n - 1, dev->iobase + DT2821_CHANCSR);
}


/*
 *    Performs a single A/D conversion.
 *      - Put channel/gain into channel-gain list
 *      - preload multiplexer
 *      - trigger conversion and wait for it to finish
 */
static int dt282x_ai_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	devpriv->adcsr = DT2821_ADCLK;
	update_adcsr(0);

	dt282x_load_changain(dev, 1, it->chanlist);

	update_supcsr(DT2821_PRLD);
	wait_for(!mux_busy(),
		 comedi_error(dev, "timeout\n");
		 return -ETIME;
	    );

	update_supcsr(DT2821_STRIG);
	wait_for(ad_done(),
		 comedi_error(dev, "timeout\n");
		 return -ETIME;
	    );

	it->data[0] = inw(dev->iobase + DT2821_ADDAT) & ((1<<boardtype.adbits)-1);
	if (devpriv->ad_2scomp)
		it->data[0] ^= (1 << (boardtype.adbits - 1));

	return 1;
}

#if 0
static int dt282x_ai_cmd(comedi_device * dev, comedi_subdevice * s)
{
	int err=0;
	comedi_cmd *cmd=&s->cur_cmd;

	if(cmd->start_src!=TRIG_NOW ||
	   cmd->start_arg!=0 |||
	   cmd->scan_begin_arg!=0 ||
	   cmd->convert_src!=TRIG_TIMER ||
	   cmd->scan_end_src!=TRIG_COUNT ||
	   cmd->scan_end_arg!=cmd->chanlist_len){
		err=1;
		cmd->start_src=TRIG_NOW;
		cmd->start_arg=0;
		cmd->scan_begin_arg=0;
		cmd->convert_src=TRIG_TIMER;
		cmd->scan_end_src=TRIG_COUNT;
	   	cmd->scan_end_arg=cmd->chanlist_len;
	}
	if(cmd->scan_begin_src!=TRIG_FOLLOW && cmd->scan_begin_src!=TRIG_EXT){
		err=1;
		cmd->scan_begin_src=TRIG_INVAL;
	}
	if(cmd->convert_arg<4000){
		err=1;
		cmd->convert_arg=4000;
	}
	if(cmd->stop_src!=TRIG_COUNT && cmd->stop_src!=TRIG_NONE){
		err=1;

	}

	return -EINVAL;
}
#endif

static int dt282x_ai_mode1(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int timer;

	if (!devpriv->usedma) {
		dt282x_load_changain(dev,it->n_chan,it->chanlist);

		devpriv->ntrig=it->n*it->n_chan;
		devpriv->nread=devpriv->ntrig;

		timer=dt282x_ns_to_timer(&it->trigvar);
		outw(timer, dev->iobase + DT2821_TMRCTR);

		devpriv->adcsr = DT2821_ADCLK | DT2821_IADDONE;
		update_adcsr(0);

		devpriv->supcsr = DT2821_ERRINTEN ;

		update_supcsr(DT2821_PRLD);
		wait_for(!mux_busy(),
			 comedi_error(dev, "timeout\n");
			 return -ETIME;
		    );
		update_supcsr(DT2821_STRIG);

		return 0;
	} else {
		timer=dt282x_ns_to_timer(&it->trigvar);
		outw(timer, dev->iobase + DT2821_TMRCTR);

		devpriv->supcsr = DT2821_ERRINTEN | DT2821_DS0;
		update_supcsr(DT2821_CLRDMADNE | DT2821_BUFFB | DT2821_ADCINIT);
		devpriv->adcsr = 0;

		devpriv->ntrig=it->n*it->n_chan;
		devpriv->nread=devpriv->ntrig;

		devpriv->dma_dir=DMA_MODE_READ;
		devpriv->current_dma_chan=0;
		prep_ai_dma(dev,0,0);
		enable_dma(devpriv->dma[0].chan);
		if(devpriv->ntrig){
			prep_ai_dma(dev,1,0);
			enable_dma(devpriv->dma[1].chan);
			devpriv->supcsr |= DT2821_DDMA;
			update_supcsr(0);
		}

		devpriv->adcsr = DT2821_ADCLK | DT2821_IADDONE;
		update_adcsr(0);

		dt282x_load_changain(dev,it->n_chan,it->chanlist);

		devpriv->adcsr = DT2821_ADCLK | DT2821_IADDONE;
		update_adcsr(0);

		update_supcsr(DT2821_PRLD);
		wait_for(!mux_busy(),
			 comedi_error(dev, "timeout\n");
			 return -ETIME;
		    );
		update_supcsr(DT2821_STRIG);

		return 0;
	}
}

static int dt282x_ai_mode4(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int timer;

	if (!devpriv->usedma) {
		dt282x_load_changain(dev,it->n_chan,it->chanlist);

		devpriv->ntrig=it->n*it->n_chan;
		devpriv->nread=devpriv->ntrig;

		timer=dt282x_ns_to_timer(&it->trigvar1);
		outw(timer, dev->iobase + DT2821_TMRCTR);

		devpriv->adcsr = DT2821_ADCLK | DT2821_IADDONE;
		update_adcsr(0);

		devpriv->supcsr = DT2821_ERRINTEN ;

		update_supcsr(DT2821_PRLD);
		wait_for(!mux_busy(),
			 comedi_error(dev, "timeout\n");
			 return -ETIME;
		    );
		update_supcsr(DT2821_STRIG);

		return 0;
	} else {
		timer=dt282x_ns_to_timer(&it->trigvar1);
		outw(timer, dev->iobase + DT2821_TMRCTR);

		devpriv->supcsr = DT2821_ERRINTEN | DT2821_DS0 | DT2821_DS1;
		update_supcsr(DT2821_CLRDMADNE | DT2821_BUFFB | DT2821_ADCINIT);
		devpriv->adcsr = 0;

		devpriv->ntrig=it->n*it->n_chan;
		devpriv->nread=devpriv->ntrig;

		devpriv->dma_dir=DMA_MODE_READ;
		devpriv->current_dma_chan=0;
		prep_ai_dma(dev,0,0);
		enable_dma(devpriv->dma[0].chan);
		if(devpriv->ntrig){
			prep_ai_dma(dev,1,0);
			enable_dma(devpriv->dma[1].chan);
			devpriv->supcsr |= DT2821_DDMA;
			update_supcsr(0);
		}

		devpriv->adcsr = DT2821_ADCLK | DT2821_IADDONE;
		update_adcsr(0);

		dt282x_load_changain(dev,it->n_chan,it->chanlist);

		devpriv->adcsr = DT2821_ADCLK | DT2821_IADDONE;
		update_adcsr(0);

		update_supcsr(DT2821_PRLD);
		wait_for(!mux_busy(),
			 comedi_error(dev, "timeout\n");
			 return -ETIME;
		    );
		devpriv->supcsr |= DT2821_XTRIG;
		update_supcsr(0);

		return 0;
	}
}

static int dt282x_ai_cancel(comedi_device * dev, comedi_subdevice * s)
{
	devpriv->adcsr=0;
	update_adcsr(0);

	devpriv->supcsr = 0;
	update_supcsr(DT2821_ADCINIT);

	return 0;
}


static int dt282x_ns_to_timer(int *nanosec)
{
	int prescale,base,divider;

	for(prescale=0;prescale<16;prescale++){
		if(prescale==1)continue;
		base=250*(1<<prescale);
		divider=(*nanosec+base/2)/base;
		if(divider<256){
			*nanosec=divider*base;
			return (prescale<<8)|(255-divider);
		}
	}
	base=250*(1<<15);
	divider=255;
	*nanosec=divider*base;
	return (15<<8)|(255-divider);
}


/*
 *    Analog output routine.  Selects single channel conversion,
 *      selects correct channel, converts from 2's compliment to
 *      offset binary if necessary, loads the data into the DAC
 *      data register, and performs the conversion.
 */
static int dt282x_ao(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	sampl_t data;
	unsigned int chan;

	data = it->data[0];
	chan = CR_CHAN(it->chanlist[0]);

	devpriv->dacsr |= DT2821_SSEL;

	if (chan) {
		/* select channel */
		devpriv->dacsr |= DT2821_YSEL;
		if (devpriv->da0_2scomp)
			data ^= (1<<(boardtype.dabits-1));
	} else {
		devpriv->dacsr &= ~DT2821_YSEL;
		if (devpriv->da1_2scomp)
			data ^= (1<<(boardtype.dabits-1));
	}

	update_dacsr(0);

	outw(data, dev->iobase + DT2821_DADAT);

	update_supcsr(DT2821_DACON);

	return 1;
}

static int dt282x_ao_mode2(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int size;
	int timer;

	devpriv->supcsr = DT2821_ERRINTEN | DT2821_DS1 | DT2821_DDMA;
	update_supcsr(DT2821_CLRDMADNE | DT2821_BUFFB | DT2821_DACINIT);

	devpriv->ntrig=it->n*it->n_chan;
	devpriv->nread=devpriv->ntrig;

	devpriv->dma_dir=DMA_MODE_WRITE;
	devpriv->current_dma_chan=0;

	size=copy_from_buf(dev,s,devpriv->dma[0].buf,devpriv->dma_maxsize*2);
	prep_ao_dma(dev,0,size/2);
	enable_dma(devpriv->dma[0].chan);

	size=copy_from_buf(dev,s,devpriv->dma[1].buf,devpriv->dma_maxsize*2);
	prep_ao_dma(dev,1,size/2);
	enable_dma(devpriv->dma[1].chan);
	
	timer=dt282x_ns_to_timer(&it->trigvar);
	outw(timer, dev->iobase + DT2821_TMRCTR);

	devpriv->dacsr = DT2821_SSEL| DT2821_DACLK | DT2821_IDARDY;
	update_dacsr(0);

	update_supcsr(DT2821_STRIG);

	return 0;
}

static int dt282x_ao_cancel(comedi_device * dev, comedi_subdevice * s)
{
	devpriv->dacsr=0;
	update_dacsr(0);

	devpriv->supcsr = 0;
	update_supcsr(DT2821_DACINIT);

	return 0;
}

static int dt282x_dio(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	if(it->flags&TRIG_CONFIG){
		int mask,i;

		for(i=0;i<it->n_chan;i++){
			mask=(CR_CHAN(it->chanlist[i])<8)?0x00ff:0xff00;
			if(it->data[i])s->io_bits|=mask;
			else s->io_bits&=~mask;
		}
		if(s->io_bits&0x00ff)devpriv->dacsr|=DT2821_LBOE;
		else devpriv->dacsr&=~DT2821_LBOE;
		if(s->io_bits&0xff00)devpriv->dacsr|=DT2821_HBOE;
		else devpriv->dacsr&=~DT2821_HBOE;

		outw(devpriv->dacsr, dev->iobase + DT2821_DACSR);
	}else{
		unsigned int data;

		if(it->flags&TRIG_WRITE){
			do_pack(&s->state,it);
			outw(s->state, dev->iobase + DT2821_DIODAT);
		}else{
			data = inw(dev->iobase + DT2821_DIODAT);
			di_unpack(data,it);
		}
	}

	return it->n_chan;
}


static int ai_range_table[]={ RANGE_dt282x_ai_lo_bipolar,
	RANGE_dt282x_ai_lo_unipolar, RANGE_dt282x_ai_5_bipolar,
	RANGE_dt282x_ai_5_unipolar };
static int ai_range_pgl_table[]={ RANGE_dt282x_ai_hi_bipolar,
	RANGE_dt282x_ai_hi_unipolar };
static inline int opt_ai_range_lkup(int ispgl,int x)
{
	if(ispgl){
		if(x<0 || x>=2)return RANGE_unknown;
		return ai_range_pgl_table[x];
	}else{
		if(x<0 || x>=4)return RANGE_unknown;
		return ai_range_table[x];
	}
}
static int ao_range_table[]={ RANGE_bipolar10, RANGE_unipolar10, RANGE_bipolar5,
	RANGE_unipolar5, RANGE_bipolar2_5 };
static inline int opt_ao_range_lkup(int x)
	{ if(x<0)x=0; if(x>=5)x=0; return ao_range_table[x]; }

enum{	opt_iobase=0, opt_irq, opt_dma1, opt_dma2,	/* i/o base, irq, dma channels */
	opt_diff,					/* differential */
	opt_ai_twos, opt_ao0_twos, opt_ao1_twos, 	/* twos comp */
	opt_ai_range, opt_ao0_range, opt_ao1_range,	/* range */
};


static int dt282x_recognize(char *name)
{
	int i;

	for(i=0;i<n_boardtypes;i++){
		if(!strcmp(boardtypes[i].name,name))
			return i;
	}

	return -1;
}

/*
   options:
   0	i/o base
   1	irq
   2	dma1
   3	dma2
   4	0=single ended, 1=differential
   5	ai 0=straight binary, 1=2's comp
   6	ao0 0=straight binary, 1=2's comp
   7	ao1 0=straight binary, 1=2's comp
   8	ai 0=±10 V, 1=0-10 V, 2=±5 V, 3=0-5 V
   9	ao0 0=±10 V, 1=0-10 V, 2=±5 V, 3=0-5 V, 4=±2.5 V
   10	ao1 0=±10 V, 1=0-10 V, 2=±5 V, 3=0-5 V, 4=±2.5 V
 */
static int dt282x_attach(comedi_device * dev, comedi_devconfig * it)
{
	int i, irqs, irq;
	long flags;
	int ret;
	comedi_subdevice *s;

	dev->board_ptr = boardtypes+dev->board;
	dev->board_name = boardtypes[dev->board].name;

	if (it->options[opt_iobase])
		dev->iobase = it->options[opt_iobase];
	else
		dev->iobase = 0x240;

	printk("comedi%d: dt282x: 0x%04x", dev->minor, dev->iobase);
	if (check_region(dev->iobase, DT2821_SIZE) < 0) {
		printk(" I/O port conflict\n");
		return -EBUSY;
	}
	request_region(dev->iobase, DT2821_SIZE, "dt282x");
	dev->iosize = DT2821_SIZE;

	outw(DT2821_BDINIT, dev->iobase + DT2821_SUPCSR);
	i = inw(dev->iobase + DT2821_ADCSR);
#ifdef DEBUG
	printk(" fingerprint=%x,%x,%x,%x,%x",
	       inw(dev->iobase + DT2821_ADCSR),
	       inw(dev->iobase + DT2821_CHANCSR),
	       inw(dev->iobase + DT2821_DACSR),
	       inw(dev->iobase + DT2821_SUPCSR),
	       inw(dev->iobase + DT2821_TMRCTR));
#endif

	if (
		   ((inw(dev->iobase + DT2821_ADCSR) & DT2821_ADCSR_MASK)
		    != DT2821_ADCSR_VAL) ||
	       ((inw(dev->iobase + DT2821_CHANCSR) & DT2821_CHANCSR_MASK)
		!= DT2821_CHANCSR_VAL) ||
		   ((inw(dev->iobase + DT2821_DACSR) & DT2821_DACSR_MASK)
		    != DT2821_DACSR_VAL) ||
		 ((inw(dev->iobase + DT2821_SUPCSR) & DT2821_SUPCSR_MASK)
		  != DT2821_SUPCSR_VAL) ||
		 ((inw(dev->iobase + DT2821_TMRCTR) & DT2821_TMRCTR_MASK)
		  != DT2821_TMRCTR_VAL)) {
		printk(" board not found");
		return -EIO;
	}
	/* should do board test */

	irq = it->options[opt_irq];
	if (irq < 0) {
		save_flags(flags);
		sti();
		irqs = probe_irq_on();

		/* trigger interrupt */

		udelay(100);

		irq = probe_irq_off(irqs);
		restore_flags(flags);
		if (0 /* error */ ) {
			printk(" error probing irq (bad)");
		}
	}
	dev->irq = 0;
	if (irq > 0) {
		printk(" ( irq = %d )", irq);
		request_irq(irq, dt282x_interrupt, SA_INTERRUPT, "dt282x", dev);
		dev->irq = irq;
	} else if (irq == 0) {
		printk(" (no irq)");
	} else {
		printk(" (probe returned multiple irqs--bad)");
	}

	if((ret=alloc_private(dev,sizeof(dt282x_private)))<0)
		return ret;

	ret=dt282x_grab_dma(dev,it->options[opt_dma1],it->options[opt_dma2]);
	if(ret<0)
		return ret;

	dev->n_subdevices = 3;
	if((ret=alloc_subdevices(dev))<0)
		return ret;

	s=dev->subdevices+0;

	/* ai subdevice */
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE|((it->options[opt_diff])?SDF_DIFF:SDF_COMMON);
	s->n_chan=(it->options[opt_diff])?boardtype.adchan_di:boardtype.adchan_se;
	s->trig[0]=dt282x_ai_mode0;
	s->trig[1]=dt282x_ai_mode1;
	s->trig[4]=dt282x_ai_mode4;
#if 0
	s->do_cmd=dt282x_ai_cmd;
#endif
	s->cancel=dt282x_ai_cancel;
	s->maxdata=(1<<boardtype.adbits)-1;
	s->len_chanlist=16;
	s->range_type = opt_ai_range_lkup(boardtype.ispgl,it->options[opt_ai_range]);
	s->timer_type=TIMER_nanosec;
	devpriv->ad_2scomp=it->options[opt_ai_twos];

	s++;
	if((s->n_chan=boardtype.dachan)){
		/* ao subsystem */
		s->type=COMEDI_SUBD_AO;
		s->subdev_flags=SDF_WRITEABLE;
		s->trig[0]=dt282x_ao;
		s->trig[2]=dt282x_ao_mode2;
		s->cancel=dt282x_ao_cancel;
		s->maxdata=(1<<boardtype.dabits)-1;
		s->len_chanlist=1;			/* XXX could do 2 */
		s->range_type_list=devpriv->darangelist;
		s->timer_type=TIMER_nanosec;
		devpriv->darangelist[0]=
			opt_ao_range_lkup(it->options[opt_ao0_range]);
		devpriv->darangelist[1]=
			opt_ao_range_lkup(it->options[opt_ao1_range]);
		devpriv->da0_2scomp=it->options[opt_ao0_twos];
		devpriv->da1_2scomp=it->options[opt_ao1_twos];
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	s++;
	/* dio subsystem */
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_READABLE|SDF_WRITEABLE;
	s->n_chan=16;
	s->trig[0]=dt282x_dio;
	s->maxdata=1;
	s->range_type = RANGE_digital;

	printk("\n");

	return 0;
}


static void free_resources(comedi_device *dev)
{
	if (dev->irq) {
		free_irq(dev->irq, dev);
	}
	if(dev->iobase)
		release_region(dev->iobase, dev->iosize);
	if(dev->private){
		if (devpriv->dma[0].chan)
			free_dma(devpriv->dma[0].chan);
		if (devpriv->dma[1].chan)
			free_dma(devpriv->dma[1].chan);
		if (devpriv->dma[0].buf)
			free_page((unsigned long) devpriv->dma[0].buf);
		if (devpriv->dma[1].buf)
			free_page((unsigned long) devpriv->dma[1].buf);
	}
}

static int dt282x_detach(comedi_device * dev)
{
	printk("comedi%d: dt282x: remove\n", dev->minor);

	free_resources(dev);

	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_dt282x);

	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_dt282x);
}

#endif

static int dt282x_grab_dma(comedi_device *dev,int dma1,int dma2)
{
	int ret;

	devpriv->usedma=0;

	if(!dma1 && !dma2){
		printk(" (no dma)");
		return 0;
	}

	if(dma1==dma2 || dma1<5 || dma2<5 || dma1>7 || dma2>7)
		return -EINVAL;

	if(dma2<dma1){
		int i;i=dma1;dma1=dma2;dma2=i;
	}

	ret = request_dma(dma1, "dt282x A");
	if (ret)
		return -EBUSY;
	devpriv->dma[0].chan=dma1;

	ret = request_dma(dma2, "dt282x B");
	if (ret)
		return -EBUSY;
	devpriv->dma[1].chan=dma2;

	devpriv->dma_maxsize = PAGE_SIZE >> 1;
	devpriv->dma[0].buf = (void *) get_free_page(GFP_KERNEL | GFP_DMA);
	devpriv->dma[1].buf = (void *) get_free_page(GFP_KERNEL | GFP_DMA);
	if (!devpriv->dma[0].buf || !devpriv->dma[1].buf) {
		printk(" can't get DMA memory");
		return -ENOMEM;
	}

	printk(" (dma=%d,%d)",dma1,dma2);

	devpriv->usedma=1;

	return 0;
}

