/*
    comedi/drivers/ni_mio_common.c
    Hardware driver for DAQ-STC based boards

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-2001 David A. Schleef <ds@schleef.org>
    Copyright (C) 2002, 2003 Frank Mori Hess <fmhess@users.sourceforge.net

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
	This file is meant to be included by another file, e.g.,
	ni_atmio.c or ni_pcimio.c.

	Interrupt support originally added by Truxton Fulton
	<trux@truxton.com>

	References (from ftp://ftp.natinst.com/support/manuals):

	   340747b.pdf  AT-MIO E series Register Level Programmer Manual
	   341079b.pdf  PCI E Series RLPM
	   340934b.pdf  DAQ-STC reference manual
	67xx and 611x registers (from http://www.ni.com/pdf/daq/us)
	release_ni611x.pdf
	release_ni67xx.pdf
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

	 - the interrupt routine needs to be cleaned up
	 - many printk's need to be changed to rt_printk()

	2006-02-07: S-Series PCI-6143: Support has been added but is not
		fully tested as yet. Terry Barnaby, BEAM Ltd.
*/

//#define DEBUG_INTERRUPT
//#define DEBUG_STATUS_A
//#define DEBUG_STATUS_B

#include "8255.h"
#include "mite.h"
#include "comedi_fc.h"

#ifndef MDPRINTK
#define MDPRINTK(format,args...)
#endif

/* A timeout count */

#define NI_TIMEOUT 1000

/* Note: this table must match the ai_gain_* definitions */
static short ni_gainlkup[][16]={
	/* ai_gain_16 */
	{ 0, 1, 2, 3, 4, 5, 6, 7, 0x100, 0x101, 0x102, 0x103, 0x104, 0x105,
		0x106, 0x107 },
	/* ai_gain_8 */
	{ 1, 2, 4, 7, 0x101, 0x102, 0x104, 0x107 },
	/* ai_gain_14 */
	{ 1, 2, 3, 4, 5, 6, 7, 0x101, 0x102, 0x103, 0x104, 0x105, 0x106,
		0x107 },
	/* ai_gain_4 */
	{ 0, 1, 4, 7 },
	/* ai_gain_611x */
	{ 0x00a, 0x00b, 0x001, 0x002, 0x003, 0x004, 0x005, 0x006 },
	/* ai_gain_622x */
	{ 0, 1, 4, 5},
	/* ai_gain_628x */
	{ 1, 2, 3, 4, 5, 6, 7},
	/* ai_gain_6143 */
	{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },
};

static comedi_lrange range_ni_E_ai={	16, {
	RANGE( -10,	10	),
	RANGE( -5,	5	),
	RANGE( -2.5,	2.5	),
	RANGE( -1,	1	),
	RANGE( -0.5,	0.5	),
	RANGE( -0.25,	0.25	),
	RANGE( -0.1,	0.1	),
	RANGE( -0.05,	0.05	),
	RANGE( 0,	20	),
	RANGE( 0,	10	),
	RANGE( 0,	5	),
	RANGE( 0,	2	),
	RANGE( 0,	1	),
	RANGE( 0,	0.5	),
	RANGE( 0,	0.2	),
	RANGE( 0,	0.1	),
}};
static comedi_lrange range_ni_E_ai_limited={	8, {
	RANGE( -10,	10	),
	RANGE( -5,	5	),
	RANGE( -1,	1	),
	RANGE( -0.1,	0.1	),
	RANGE( 0,	10	),
	RANGE( 0,	5	),
	RANGE( 0,	1	),
	RANGE( 0,	0.1	),
}};
static comedi_lrange range_ni_E_ai_limited14={	14, {
	RANGE( -10,	10	),
	RANGE( -5,	5	),
	RANGE( -2,	2	),
	RANGE( -1,	1	),
	RANGE( -0.5,	0.5	),
	RANGE( -0.2,	0.2	),
	RANGE( -0.1,	0.1	),
	RANGE( 0,	10	),
	RANGE( 0,	5	),
	RANGE( 0,	2	),
	RANGE( 0,	1	),
	RANGE( 0,	0.5	),
	RANGE( 0,	0.2	),
	RANGE( 0,	0.1	),
}};
static comedi_lrange range_ni_E_ai_bipolar4={ 4, {
	RANGE( -10,	10	),
	RANGE( -5,	5	),
	RANGE( -0.5,	0.5	),
	RANGE( -0.05,	0.05	),
}};
static comedi_lrange range_ni_E_ai_611x={ 8, {
	RANGE( -50,	50	),
	RANGE( -20,	20	),
	RANGE( -10,	10	),
	RANGE( -5,	5	),
	RANGE( -2,	2	),
	RANGE( -1,	1	),
	RANGE( -0.5,	0.5	),
	RANGE( -0.2,	0.2	),
}};
static comedi_lrange range_ni_M_ai_622x={ 4, {
	RANGE(-10, 10),
	RANGE(-5, 5),
	RANGE(-1, 1),
	RANGE(-0.2, 0.2),
}};
static comedi_lrange range_ni_M_ai_628x={ 7, {
	RANGE( -10,	10	),
	RANGE( -5,	5	),
	RANGE( -2,	2	),
	RANGE( -1,	1	),
	RANGE( -0.5,	0.5	),
	RANGE( -0.2,	0.2	),
	RANGE( -0.1,	0.1	),
}};
static comedi_lrange range_ni_S_ai_6143 = { 1, {
	RANGE( -5,	+5	),
}};
static comedi_lrange range_ni_E_ao_ext = { 4, {
	RANGE( -10,	10	),
	RANGE( 0,	10	),
	RANGE_ext( -1,	1	),
	RANGE_ext( 0,	1	),
}};

static comedi_lrange *ni_range_lkup[]={
	&range_ni_E_ai,
	&range_ni_E_ai_limited,
	&range_ni_E_ai_limited14,
	&range_ni_E_ai_bipolar4,
	&range_ni_E_ai_611x,
	&range_ni_M_ai_622x,
	&range_ni_M_ai_628x,
	&range_ni_S_ai_6143
};



static int ni_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int ni_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

static int ni_serial_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int ni_serial_hw_readwrite8(comedi_device *dev,comedi_subdevice *s,
	unsigned char data_out, unsigned char *data_in);
static int ni_serial_sw_readwrite8(comedi_device *dev,comedi_subdevice *s,
	unsigned char data_out, unsigned char *data_in);

static int ni_calib_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int ni_calib_insn_write(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

static int ni_eeprom_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int ni_m_series_eeprom_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

static int ni_pfi_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int ni_pfi_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

static void ni_rtsi_init(comedi_device *dev);
static int ni_rtsi_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int ni_rtsi_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

static void caldac_setup(comedi_device *dev,comedi_subdevice *s);
static int ni_read_eeprom(comedi_device *dev,int addr);

#ifdef DEBUG_STATUS_A
static void ni_mio_print_status_a(int status);
#else
#define ni_mio_print_status_a(a)
#endif
#ifdef DEBUG_STATUS_B
static void ni_mio_print_status_b(int status);
#else
#define ni_mio_print_status_b(a)
#endif

static int ni_ai_reset(comedi_device *dev,comedi_subdevice *s);
#ifndef PCIDMA
static void ni_handle_fifo_half_full(comedi_device *dev);
static int ni_ao_fifo_half_empty(comedi_device *dev,comedi_subdevice *s);
#endif
static void ni_handle_fifo_dregs(comedi_device *dev);
static int ni_ai_inttrig(comedi_device *dev,comedi_subdevice *s,
	unsigned int trignum);
static void ni_load_channelgain_list(comedi_device *dev,unsigned int n_chan,
	unsigned int *list);
static void shutdown_ai_command( comedi_device *dev );

static int ni_ao_inttrig(comedi_device *dev,comedi_subdevice *s,
	unsigned int trignum);

static int ni_ao_reset(comedi_device *dev,comedi_subdevice *s);

static int ni_8255_callback(int dir,int port,int data,unsigned long arg);

static int ni_ns_to_timer(int *nanosec,int round_mode);


/*GPCT function def's*/
static int GPCT_G_Watch(comedi_device *dev, int chan);

static void GPCT_Reset(comedi_device *dev, int chan);
static void GPCT_Gen_Cont_Pulse(comedi_device *dev, int chan, unsigned int length);
static void GPCT_Gen_Single_Pulse(comedi_device *dev, int chan, unsigned int length);
static void GPCT_Period_Meas(comedi_device *dev, int chan);
static void GPCT_Pulse_Width_Meas(comedi_device *dev, int chan);
static void GPCT_Event_Counting(comedi_device *dev,int chan);
static int GPCT_Set_Direction(comedi_device *dev,int chan,int direction);
static int GPCT_Set_Gate(comedi_device *dev,int chan ,int gate);
static int GPCT_Set_Source(comedi_device *dev,int chan ,int source);

static int ni_gpct_insn_write(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int ni_gpct_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int ni_gpct_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

static int init_cs5529(comedi_device *dev);
static int cs5529_do_conversion(comedi_device *dev, unsigned short *data);
static int cs5529_ai_insn_read(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
static unsigned int cs5529_config_read(comedi_device *dev, unsigned int reg_select_bits);
static void cs5529_config_write(comedi_device *dev, unsigned int value, unsigned int reg_select_bits);

static int ni_m_series_pwm_config(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
static int ni_6143_pwm_config(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data);
	
enum aimodes
{
	AIMODE_NONE = 0,
	AIMODE_HALF_FULL = 1,
	AIMODE_SCAN = 2,
	AIMODE_SAMPLE = 3,
};

#define SERIAL_DISABLED		0
#define SERIAL_600NS		600
#define SERIAL_1_2US		1200
#define SERIAL_10US			10000

static const int num_adc_stages_611x = 3;

static void handle_a_interrupt(comedi_device *dev,unsigned short status,
	unsigned int m_status);
static void handle_b_interrupt(comedi_device *dev,unsigned short status,
	unsigned int m_status);
static void get_last_sample_611x( comedi_device *dev );
static void get_last_sample_6143( comedi_device *dev );
#ifdef PCIDMA
//static void mite_handle_interrupt(comedi_device *dev,unsigned int status);
static int ni_ai_drain_dma(comedi_device *dev );
#endif

static void ni_flush_ai_fifo(comedi_device *dev){
	if(boardtype.reg_type == ni_reg_6143){
		// Flush the 6143 data FIFO
		ni_writel(0x10, AIFIFO_Control_6143);		// Flush fifo
		ni_writel(0x00, AIFIFO_Control_6143);		// Flush fifo
		while(ni_readl(AIFIFO_Status_6143) & 0x10);	// Wait for complete
	}
	else {
		devpriv->stc_writew(dev, 1,ADC_FIFO_Clear);
	}
}

static void win_out2(comedi_device *dev, uint32_t data, int reg)
{
	devpriv->stc_writew(dev, data >> 16, reg);
	devpriv->stc_writew(dev, data & 0xffff, reg + 1);
}

#define ao_win_out(data,addr) ni_ao_win_outw(dev,data,addr)
static inline void ni_ao_win_outw( comedi_device *dev, uint16_t data, int addr )
{
	unsigned long flags;

	comedi_spin_lock_irqsave(&devpriv->window_lock,flags);
	ni_writew(addr,AO_Window_Address_611x);
	ni_writew(data,AO_Window_Data_611x);
	comedi_spin_unlock_irqrestore(&devpriv->window_lock,flags);
}

static inline void ni_ao_win_outl(comedi_device *dev, uint32_t data, int addr)
{
	unsigned long flags;

	comedi_spin_lock_irqsave(&devpriv->window_lock,flags);
	ni_writew(addr,AO_Window_Address_611x);
	ni_writel(data,AO_Window_Data_611x);
	comedi_spin_unlock_irqrestore(&devpriv->window_lock,flags);
}

static inline unsigned short ni_ao_win_inw( comedi_device *dev, int addr )
{
	unsigned long flags;
	unsigned short data;

	comedi_spin_lock_irqsave(&devpriv->window_lock,flags);
	ni_writew(addr, AO_Window_Address_611x);
	data = ni_readw(AO_Window_Data_611x);
	comedi_spin_unlock_irqrestore(&devpriv->window_lock,flags);
	return data;
}

/* ni_set_bits( ) allows different parts of the ni_mio_common driver to
* share registers (such as Interrupt_A_Register) without interfering with
* each other.
*
* NOTE: the switch/case statements are optimized out for a constant argument
* so this is actually quite fast---  If you must wrap another function around this
* make it inline to avoid a large speed penalty.
*
* value should only be 1 or 0.
*/
static inline void ni_set_bits(comedi_device *dev, int reg, int bits, int value)
{
	unsigned long flags;

	comedi_spin_lock_irqsave( &devpriv->window_lock, flags );
	switch (reg){
		case Interrupt_A_Enable_Register:
			if(value)
				devpriv->int_a_enable_reg |= bits;
			else
				devpriv->int_a_enable_reg &= ~bits;
			comedi_spin_unlock_irqrestore( &devpriv->window_lock, flags );
			devpriv->stc_writew(dev, devpriv->int_a_enable_reg,Interrupt_A_Enable_Register);
			break;
		case Interrupt_B_Enable_Register:
			if(value)
				devpriv->int_b_enable_reg |= bits;
			else
				devpriv->int_b_enable_reg &= ~bits;
			comedi_spin_unlock_irqrestore( &devpriv->window_lock, flags );
			devpriv->stc_writew(dev, devpriv->int_b_enable_reg,Interrupt_B_Enable_Register);
			break;
		case IO_Bidirection_Pin_Register:
			if(value)
				devpriv->io_bidirection_pin_reg |= bits;
			else
				devpriv->io_bidirection_pin_reg &= ~bits;
			comedi_spin_unlock_irqrestore( &devpriv->window_lock, flags );
			devpriv->stc_writew(dev, devpriv->io_bidirection_pin_reg,IO_Bidirection_Pin_Register);
			break;
		default:
			printk("Warning ni_set_bits() called with invalid arguments\n");
			printk("reg is %d\n",reg);
			comedi_spin_unlock_irqrestore( &devpriv->window_lock, flags );
			break;
	}
}


static irqreturn_t ni_E_interrupt(int irq,void *d,struct pt_regs * regs)
{
	comedi_device *dev=d;
	unsigned short a_status;
	unsigned short b_status;
	unsigned int m0_status;
	unsigned int m1_status;
	unsigned long flags;
#ifdef PCIDMA
	struct mite_struct *mite = devpriv->mite;
#endif

	if(dev->attached == 0) return IRQ_NONE;
	// lock to avoid race with comedi_poll
	comedi_spin_lock_irqsave(&dev->spinlock, flags);
	a_status=devpriv->stc_readw(dev, AI_Status_1_Register);
	b_status=devpriv->stc_readw(dev, AO_Status_1_Register);
#ifdef PCIDMA
	m0_status=readl(mite->mite_io_addr + MITE_CHSR(AI_DMA_CHAN));
	m1_status=readl(mite->mite_io_addr + MITE_CHSR(AO_DMA_CHAN));
#else
	m0_status = 0;
	m1_status = 0;
#endif

	if(a_status&Interrupt_A_St || m0_status & CHSR_INT )
		handle_a_interrupt(dev, a_status, m0_status);
	if(b_status&Interrupt_B_St || m1_status & CHSR_INT )
		handle_b_interrupt(dev, b_status, m1_status);
	comedi_spin_unlock_irqrestore(&dev->spinlock, flags);
	return IRQ_HANDLED;
}

#ifdef PCIDMA
static void ni_sync_ai_dma(struct mite_struct *mite, comedi_device *dev)
{
	int count;
	comedi_subdevice *s = dev->subdevices + 0;
	comedi_async *async = s->async;
	unsigned int nbytes, old_alloc_count;
	unsigned int bytes_per_scan = 2 * async->cmd.chanlist_len;

	old_alloc_count = async->buf_write_alloc_count;
	// write alloc as much as we can
	comedi_buf_write_alloc(s->async, s->async->prealloc_bufsz);

	nbytes = mite_bytes_written_to_memory_lb(mite, AI_DMA_CHAN);
	rmb();
	if( (int)(mite_bytes_written_to_memory_ub(mite, AI_DMA_CHAN) - old_alloc_count) > 0 ){
		printk("ni_mio_common: DMA overwrite of free area\n");
		ni_ai_reset(dev,s);
		async->events |= COMEDI_CB_OVERFLOW;
		return;
	}

	count = nbytes - async->buf_write_count;
	if( count <= 0 ){
		/* it's possible count will be negative due to
		 * conservative value returned by mite_bytes_transferred */
		return;
	}
	comedi_buf_write_free(async, count);

	async->scan_progress += count;
	if( async->scan_progress >= bytes_per_scan )
	{
		async->scan_progress %= bytes_per_scan;
		async->events |= COMEDI_CB_EOS;
	}
	async->events |= COMEDI_CB_BLOCK;
}

static void mite_handle_b_linkc(struct mite_struct *mite, comedi_device *dev)
{
	int count;
	comedi_subdevice *s = dev->subdevices + 1;
	comedi_async *async = s->async;
	u32 nbytes_ub, nbytes_lb;
	unsigned int new_write_count;
	u32 stop_count = async->cmd.stop_arg * sizeof(sampl_t);
	
	writel(CHOR_CLRLC, mite->mite_io_addr + MITE_CHOR(AO_DMA_CHAN));

	new_write_count = async->buf_write_count;
	mb();
	nbytes_lb = mite_bytes_read_from_memory_lb(mite, AO_DMA_CHAN);
	if(async->cmd.stop_src == TRIG_COUNT &&
		(int) (nbytes_lb - stop_count) > 0)
		nbytes_lb = stop_count;
	mb();
	nbytes_ub = mite_bytes_read_from_memory_ub(mite, AO_DMA_CHAN);
	if(async->cmd.stop_src == TRIG_COUNT &&
		(int) (nbytes_ub - stop_count) > 0)
		nbytes_ub = stop_count;
	if((int)(nbytes_ub - devpriv->last_buf_write_count) > 0){
		rt_printk("ni_mio_common: DMA underrun\n");
		ni_ao_reset(dev,s);
		async->events |= COMEDI_CB_OVERFLOW;
		return;
	}
	mb();
	devpriv->last_buf_write_count = new_write_count;

	count = nbytes_lb - async->buf_read_count;
	if(count < 0){
		return;
	}
	comedi_buf_read_free(async, count);

	async->events |= COMEDI_CB_BLOCK;
}

static int ni_ao_wait_for_dma_load( comedi_device *dev )
{
	static const int timeout = 10000;
	int i;

	for(i = 0; i < timeout; i++)
	{
		unsigned short b_status;

		b_status = devpriv->stc_readw(dev,  AO_Status_1_Register );
		if( b_status & AO_FIFO_Half_Full_St )
			break;
		/* if we poll too often, the pci bus activity seems
		 to slow the dma transfer down */
		comedi_udelay(10);
	}
	if( i == timeout )
	{
		comedi_error(dev, "timed out waiting for dma load");
		return -EPIPE;
	}
	return 0;
}

#endif //PCIDMA
static void ni_handle_eos(comedi_device *dev, comedi_subdevice *s)
{
	if(devpriv->aimode == AIMODE_SCAN)
	{
#ifdef PCIDMA
		static const int timeout = 10;
		int i;

		for(i = 0; i < timeout; i++)
		{
			ni_sync_ai_dma(devpriv->mite, dev);
			if((s->async->events & COMEDI_CB_EOS)) break;
			comedi_udelay(1);
		}
#else
		ni_handle_fifo_dregs(dev);
		s->async->events |= COMEDI_CB_EOS;
#endif
	}
	/* handle special case of single scan using AI_End_On_End_Of_Scan */
	if((devpriv->ai_cmd2 & AI_End_On_End_Of_Scan)){
		shutdown_ai_command( dev );
		ni_ai_reset(dev, s);
	}
}

static void shutdown_ai_command( comedi_device *dev )
{
	comedi_subdevice *s = dev->subdevices + 0;

#ifdef PCIDMA
	ni_ai_drain_dma( dev );
	mite_dma_disarm(devpriv->mite, AI_DMA_CHAN);
#endif
	ni_handle_fifo_dregs(dev);
	get_last_sample_611x(dev);
	get_last_sample_6143(dev);

	ni_set_bits(dev, Interrupt_A_Enable_Register,
		AI_SC_TC_Interrupt_Enable | AI_START1_Interrupt_Enable|
		AI_START2_Interrupt_Enable| AI_START_Interrupt_Enable|
		AI_STOP_Interrupt_Enable| AI_Error_Interrupt_Enable|
		AI_FIFO_Interrupt_Enable,0);

	s->async->events |= COMEDI_CB_EOA;
}

static void handle_a_interrupt(comedi_device *dev,unsigned short status,
	unsigned int m_status)
{
	comedi_subdevice *s=dev->subdevices+0;
	unsigned short ack=0;

	s->async->events = 0;

#ifdef DEBUG_INTERRUPT
	rt_printk("ni_mio_common: interrupt: a_status=%04x m0_status=%08x\n",
		status, m_status);
	ni_mio_print_status_a(status);
#endif


#ifdef PCIDMA
	/* Currently, mite.c requires us to handle LINKC and DONE */
	if(m_status & CHSR_LINKC){
		writel(CHOR_CLRLC, devpriv->mite->mite_io_addr + MITE_CHOR(AI_DMA_CHAN));
		ni_sync_ai_dma(devpriv->mite, dev);
	}

	if(m_status & CHSR_DONE){
		writel(CHOR_CLRDONE, devpriv->mite->mite_io_addr + MITE_CHOR(AI_DMA_CHAN));
	}

	if(m_status & ~(CHSR_INT | CHSR_LINKC | CHSR_DONE | CHSR_MRDY | CHSR_DRDY | CHSR_DRQ1 | CHSR_DRQ0 | CHSR_ERROR | CHSR_SABORT | CHSR_XFERR | CHSR_LxERR_mask)){
		printk("unknown mite interrupt, ack! (m_status=%08x)\n", m_status);
		//mite_print_chsr(m_status);
		mite_dma_disarm(devpriv->mite, AI_DMA_CHAN );
		writel(CHOR_DMARESET, devpriv->mite->mite_io_addr + MITE_CHOR(AI_DMA_CHAN));
		//disable_irq(dev->irq);
	}
#endif

	/* test for all uncommon interrupt events at the same time */
	if(status&(AI_Overrun_St|AI_Overflow_St|AI_SC_TC_Error_St|AI_SC_TC_St|AI_START1_St)){
		if(status==0xffff){
			rt_printk("ni_mio_common: a_status=0xffff.  Card removed?\n");
			/* we probably aren't even running a command now,
			 * so it's a good idea to be careful. */
			if(s->subdev_flags&SDF_RUNNING){
				s->async->events |= COMEDI_CB_ERROR | COMEDI_CB_EOA;
				//comedi_event(dev,s,s->async->events);
			}
			return;
		}
		if(status&(AI_Overrun_St|AI_Overflow_St|AI_SC_TC_Error_St)){
			rt_printk("ni_mio_common: ai error a_status=%04x\n",
				status);
			ni_mio_print_status_a(status);

			ni_ai_reset(dev,dev->subdevices);


			shutdown_ai_command( dev );

			s->async->events |= COMEDI_CB_ERROR;
			if(status & (AI_Overrun_St | AI_Overflow_St))
				s->async->events |= COMEDI_CB_OVERFLOW;
			
			comedi_event(dev,s,s->async->events);

			return;
		}
		if(status&AI_SC_TC_St){
#ifdef DEBUG_INTERRUPT
			rt_printk("ni_mio_common: SC_TC interrupt\n");
#endif
			if(!devpriv->ai_continuous){
				shutdown_ai_command( dev );
			}
			ack|=AI_SC_TC_Interrupt_Ack;
		}
		if(status&AI_START1_St){
			ack|=AI_START1_Interrupt_Ack;
		}
	}
#ifndef PCIDMA
	if(status&AI_FIFO_Half_Full_St){
		int i;
		static const int timeout = 10;
		/* pcmcia cards (at least 6036) seem to stop producing interrupts if we 
		 *fail to get the fifo less than half full, so loop to be sure.*/
		for(i = 0; i < timeout; ++i)
		{
			ni_handle_fifo_half_full(dev);
			if((devpriv->stc_readw(dev, AI_Status_1_Register) & AI_FIFO_Half_Full_St) == 0)
				break;
		}
	}
#endif // !PCIDMA

	if( (status & AI_STOP_St) ){
		ni_handle_eos(dev, s);
		/* we need to ack the START, also */
		ack |= AI_STOP_Interrupt_Ack|AI_START_Interrupt_Ack;
	}
#if 0
	if(devpriv->aimode==AIMODE_SAMPLE){
		ni_handle_fifo_dregs(dev);

		//s->async->events |= COMEDI_CB_SAMPLE;
	}
#endif
	if(ack) devpriv->stc_writew(dev, ack,Interrupt_A_Ack_Register);

	comedi_event(dev,s,s->async->events);

#ifdef DEBUG_INTERRUPT
	status=devpriv->stc_readw(dev, AI_Status_1_Register);
	if(status&Interrupt_A_St){
		printk("handle_a_interrupt: didn't clear interrupt? status=0x%x\n", status);
	}
#endif
}

static void handle_b_interrupt(comedi_device *dev,unsigned short b_status, unsigned int m_status)
{
	comedi_subdevice *s=dev->subdevices+1;
	//unsigned short ack=0;
#ifdef DEBUG_INTERRUPT
	rt_printk("ni_mio_common: interrupt: b_status=%04x m1_status=%08x\n",
		b_status,m_status);
	ni_mio_print_status_b(b_status);
#endif


#ifdef PCIDMA
	/* Currently, mite.c requires us to handle LINKC and DONE */
	if(m_status & CHSR_LINKC){
		mite_handle_b_linkc(devpriv->mite, dev);
	}

	if(m_status & CHSR_DONE){
		writel(CHOR_CLRDONE, devpriv->mite->mite_io_addr + MITE_CHOR(AO_DMA_CHAN));
	}

	if(m_status & ~(CHSR_INT | CHSR_LINKC | CHSR_DONE | CHSR_MRDY | CHSR_DRDY | CHSR_DRQ1 | CHSR_DRQ0 | CHSR_ERROR | CHSR_SABORT | CHSR_XFERR | CHSR_LxERR_mask)){
		printk("unknown mite interrupt, ack! (m_status=%08x)\n", m_status);
		//mite_print_chsr(m_status);
		mite_dma_disarm(devpriv->mite, AO_DMA_CHAN );
		writel(CHOR_DMARESET, devpriv->mite->mite_io_addr + MITE_CHOR(AO_DMA_CHAN));
       }
#endif

	if(b_status==0xffff)return;
	if(b_status&AO_Overrun_St){
		rt_printk("ni_mio_common: AO FIFO underrun status=0x%04x status2=0x%04x\n",b_status,devpriv->stc_readw(dev, AO_Status_2_Register));
		ni_ao_reset(dev,s);
		s->async->events |= COMEDI_CB_OVERFLOW;
	}

	if(b_status&AO_BC_TC_St){
		MDPRINTK("ni_mio_common: AO BC_TC status=0x%04x status2=0x%04x\n",b_status,devpriv->stc_readw(dev, AO_Status_2_Register));
		ni_ao_reset(dev,s);
		s->async->events |= COMEDI_CB_EOA;
	}

#ifndef PCIDMA
	if(b_status&AO_FIFO_Request_St){
		int ret;

		ret = ni_ao_fifo_half_empty(dev,s);
		if(!ret){
			rt_printk("ni_mio_common: AO buffer underrun\n");
			ni_set_bits(dev, Interrupt_B_Enable_Register,
				AO_FIFO_Interrupt_Enable|AO_Error_Interrupt_Enable, 0);
			s->async->events |= COMEDI_CB_OVERFLOW;
		}
	}
#endif

	b_status=devpriv->stc_readw(dev, AO_Status_1_Register);
	if(b_status&Interrupt_B_St){
		if(b_status&AO_FIFO_Request_St){
			rt_printk("ni_mio_common: AO buffer underrun\n");
		}
		rt_printk("Ack! didn't clear AO interrupt. b_status=0x%04x\n",b_status);
		ni_set_bits(dev,Interrupt_B_Enable_Register,~0,0);
		ni_ao_reset(dev,s);
		s->async->events |= COMEDI_CB_OVERFLOW;
	}

	comedi_event(dev,s,s->async->events);
}

#ifdef DEBUG_STATUS_A
static char *status_a_strings[]={
	"passthru0","fifo","G0_gate","G0_TC",
	"stop","start","sc_tc","start1",
	"start2","sc_tc_error","overflow","overrun",
	"fifo_empty","fifo_half_full","fifo_full","interrupt_a"
};

static void ni_mio_print_status_a(int status)
{
	int i;

	rt_printk("A status:");
	for(i=15;i>=0;i--){
		if(status&(1<<i)){
			rt_printk(" %s",status_a_strings[i]);
		}
	}
	rt_printk("\n");
}
#endif

#ifdef DEBUG_STATUS_B
static char *status_b_strings[]={
	"passthru1","fifo","G1_gate","G1_TC",
	"UI2_TC","UPDATE","UC_TC","BC_TC",
	"start1","overrun","start","bc_tc_error",
	"fifo_empty","fifo_half_full","fifo_full","interrupt_b"
};

static void ni_mio_print_status_b(int status)
{
	int i;

	rt_printk("B status:");
	for(i=15;i>=0;i--){
		if(status&(1<<i)){
			rt_printk(" %s",status_b_strings[i]);
		}
	}
	rt_printk("\n");
}
#endif

#ifndef PCIDMA

static void ni_ao_fifo_load(comedi_device *dev,comedi_subdevice *s, int n)
{
	comedi_async *async = s->async;
	comedi_cmd *cmd = &async->cmd;
	int chan;
	int i;
	sampl_t d;
	u32 packed_data;
	int range;
	int err = 1;

	chan = async->cur_chan;
	for(i=0;i<n;i++){
		err &= comedi_buf_get(async, &d);
		if(err == 0) break;

		range = CR_RANGE(cmd->chanlist[chan]);

		if(boardtype.reg_type & ni_reg_6xxx_mask)
		{
			packed_data = d & 0xffff;
			/* 6711 only has 16 bit wide ao fifo */
			if(boardtype.reg_type != ni_reg_6711)
			{
				err &= comedi_buf_get(async, &d);
				if(err == 0) break;
				chan++;
				i++;
				packed_data |= ( d << 16 ) & 0xffff0000;
			}
			ni_writel( packed_data, DAC_FIFO_Data_611x );
		}else{
			ni_writew(d, DAC_FIFO_Data);
		}
		chan++;
		chan %= cmd->chanlist_len;
	}
	async->cur_chan = chan;
	if(err==0){
		async->events |= COMEDI_CB_OVERFLOW;
	}
}

/*
 *  There's a small problem if the FIFO gets really low and we
 *  don't have the data to fill it.  Basically, if after we fill
 *  the FIFO with all the data available, the FIFO is _still_
 *  less than half full, we never clear the interrupt.  If the
 *  IRQ is in edge mode, we never get another interrupt, because
 *  this one wasn't cleared.  If in level mode, we get flooded
 *  with interrupts that we can't fulfill, because nothing ever
 *  gets put into the buffer.
 *
 *  This kind of situation is recoverable, but it is easier to
 *  just pretend we had a FIFO underrun, since there is a good
 *  chance it will happen anyway.  This is _not_ the case for
 *  RT code, as RT code might purposely be running close to the
 *  metal.  Needs to be fixed eventually.
 */
static int ni_ao_fifo_half_empty(comedi_device *dev,comedi_subdevice *s)
{
	int n;

	n = comedi_buf_read_n_available(s);
	if(n==0){
		s->async->events |= COMEDI_CB_OVERFLOW;
		return 0;
	}

	n /= sizeof(sampl_t);
	if(n > boardtype.ao_fifo_depth / 2)
		n = boardtype.ao_fifo_depth / 2;

	ni_ao_fifo_load(dev,s,n);

	s->async->events |= COMEDI_CB_BLOCK;

	return 1;
}

static int ni_ao_prep_fifo(comedi_device *dev,comedi_subdevice *s)
{
	int n;

	/* reset fifo */
	devpriv->stc_writew(dev, 1,DAC_FIFO_Clear);
	if(boardtype.reg_type & ni_reg_6xxx_mask)
		ni_ao_win_outl(dev, 0x6, AO_FIFO_Offset_Load_611x);

	/* load some data */
	n = comedi_buf_read_n_available(s);
	if(n==0)return 0;

	n /= sizeof(sampl_t);
	if(n > boardtype.ao_fifo_depth)
		n = boardtype.ao_fifo_depth;

	ni_ao_fifo_load(dev,s,n);

	return n;
}

static void ni_ai_fifo_read(comedi_device *dev,comedi_subdevice *s,
	int n)
{
	comedi_async *async = s->async;
	int i;

	if(boardtype.reg_type == ni_reg_611x){
		sampl_t data[2];
		u32 dl;

		for( i = 0; i < n / 2; i++ ){
			dl=ni_readl(ADC_FIFO_Data_611x);
			/* This may get the hi/lo data in the wrong order */
			data[0] = (dl>>16) & 0xffff;
			data[1] = dl & 0xffff;
			cfc_write_array_to_buffer(s, data, sizeof(data));
		}
		/* Check if there's a single sample stuck in the FIFO */
		if( n % 2){
			dl=ni_readl(ADC_FIFO_Data_611x);
			data[0] = dl & 0xffff;
			cfc_write_to_buffer(s, data[0]);
		}
	} else if(boardtype.reg_type == ni_reg_6143){
		sampl_t	data[2];
		u32	dl;

		// This just reads the FIFO assuming the data is present, no checks on the FIFO status are performed
		for(i = 0; i < n / 2; i++){
			dl = ni_readl(AIFIFO_Data_6143);

			data[0] = (dl >> 16) & 0xffff;
			data[1] = dl & 0xffff;
			cfc_write_array_to_buffer(s, data, sizeof(data));
		}
		if(n % 2){
			/* Assume there is a single sample stuck in the FIFO */
			ni_writel(0x01, AIFIFO_Control_6143);	// Get stranded sample into FIFO
			dl = ni_readl(AIFIFO_Data_6143);
			data[0] = (dl >> 16) & 0xffff;
			cfc_write_to_buffer(s, data[0]);
		}
	} else{
		if( n > sizeof(devpriv->ai_fifo_buffer) / sizeof(devpriv->ai_fifo_buffer[0]))
		{
			comedi_error( dev, "bug! ai_fifo_buffer too small" );
			async->events |= COMEDI_CB_ERROR;
			return;
		}
		for(i = 0; i < n; i++){
			devpriv->ai_fifo_buffer[i] = ni_readw(ADC_FIFO_Data_Register);
		}
		cfc_write_array_to_buffer( s, devpriv->ai_fifo_buffer,
			n * sizeof(devpriv->ai_fifo_buffer[0]) );
	}
}

static void ni_handle_fifo_half_full(comedi_device *dev)
{
	int n;
	comedi_subdevice *s=dev->subdevices+0;

	n=boardtype.ai_fifo_depth/2;

	ni_ai_fifo_read(dev,s,n);
}
#endif

#ifdef PCIDMA
static int ni_ai_drain_dma(comedi_device *dev )
{
	struct mite_struct *mite = devpriv->mite;
	int i;
	static const int timeout = 10000;

	for( i = 0; i < timeout; i++ )
	{
		if((devpriv->stc_readw(dev, AI_Status_1_Register) & AI_FIFO_Empty_St) &&
			mite_bytes_in_transit(mite, AI_DMA_CHAN) == 0)
			break;
		comedi_udelay(2);
	}
	if(i == timeout)
	{
		rt_printk("ni_mio_common: wait for dma drain timed out\n");
		rt_printk("mite_bytes_in_transit=%i, AI_Status1_Register=0x%x\n",
			mite_bytes_in_transit(mite, AI_DMA_CHAN), devpriv->stc_readw(dev, AI_Status_1_Register));
		return -1;
	}

	ni_sync_ai_dma( mite, dev );

	return 0;
}
#endif
/*
   Empties the AI fifo
*/
static void ni_handle_fifo_dregs(comedi_device *dev)
{
	comedi_subdevice *s=dev->subdevices+0;
	sampl_t data[2];
	u32 dl;
	short fifo_empty;
	int i;

	if(boardtype.reg_type == ni_reg_611x){
		while((devpriv->stc_readw(dev, AI_Status_1_Register)&AI_FIFO_Empty_St) == 0){
			dl=ni_readl(ADC_FIFO_Data_611x);

			/* This may get the hi/lo data in the wrong order */
			data[0] = (dl>>16);
			data[1] = (dl&0xffff);
			cfc_write_array_to_buffer(s, data, sizeof(data));
		}
	}else if(boardtype.reg_type == ni_reg_6143){
		i = 0;
		while(ni_readl(AIFIFO_Status_6143) & 0x04){
			dl = ni_readl(AIFIFO_Data_6143);

			/* This may get the hi/lo data in the wrong order */
			data[0] = (dl >> 16);
			data[1] = (dl & 0xffff);
			cfc_write_array_to_buffer(s, data, sizeof(data));
			i += 2;
		}
		// Check if stranded sample is present
		if(ni_readl(AIFIFO_Status_6143) & 0x01){
			ni_writel(0x01, AIFIFO_Control_6143);	// Get stranded sample into FIFO
			dl = ni_readl(AIFIFO_Data_6143);
			data[0] = (dl >> 16) & 0xffff;
			cfc_write_to_buffer(s, data[0]);
		}

	}else{
		fifo_empty = devpriv->stc_readw(dev, AI_Status_1_Register) & AI_FIFO_Empty_St;
		while(fifo_empty == 0)
		{
			for(i = 0; i < sizeof(devpriv->ai_fifo_buffer) / sizeof(devpriv->ai_fifo_buffer[0]); i++)
			{
				fifo_empty = devpriv->stc_readw(dev, AI_Status_1_Register) & AI_FIFO_Empty_St;
				if(fifo_empty) break;
				devpriv->ai_fifo_buffer[i] = ni_readw(ADC_FIFO_Data_Register);
			}
			cfc_write_array_to_buffer( s, devpriv->ai_fifo_buffer,
				i * sizeof(devpriv->ai_fifo_buffer[0]) );
		}
	}
}

static void get_last_sample_611x( comedi_device *dev )
{
	comedi_subdevice *s=dev->subdevices+0;
	sampl_t data;
	u32 dl;

	if(boardtype.reg_type != ni_reg_611x) return;

	/* Check if there's a single sample stuck in the FIFO */
	if(ni_readb(XXX_Status)&0x80){
		dl=ni_readl(ADC_FIFO_Data_611x);
		data = (dl&0xffff);
		cfc_write_to_buffer(s, data);
	}
}

static void get_last_sample_6143(comedi_device* dev)
{
	comedi_subdevice*	s = dev->subdevices + 0;
	sampl_t			data;
	u32			dl;

	if(boardtype.reg_type != ni_reg_6143) return;

	/* Check if there's a single sample stuck in the FIFO */
	if(ni_readl(AIFIFO_Status_6143) & 0x01){
		ni_writel(0x01, AIFIFO_Control_6143);	// Get stranded sample into FIFO
		dl = ni_readl(AIFIFO_Data_6143);

		/* This may get the hi/lo data in the wrong order */
		data = (dl >> 16) & 0xffff;
		cfc_write_to_buffer(s, data);
	}
}

static void ni_ai_munge(comedi_device *dev, comedi_subdevice *s,
	void *data, unsigned int num_bytes, unsigned int chan_index )
{
	comedi_async *async = s->async;
	unsigned int i;
	unsigned int length = num_bytes / bytes_per_sample(s);
	sampl_t *array = data;
	lsampl_t *larray = data;
	
	for(i = 0; i < length; i++)
	{
#ifdef PCIDMA
		if(s->subdev_flags & SDF_LSAMPL)
			larray[i] = le32_to_cpu(larray[i]);
		else
			array[i] = le16_to_cpu(array[i]);
#endif
		if(s->subdev_flags & SDF_LSAMPL)
			larray[i] += devpriv->ai_offset[chan_index];
		else
			array[i] += devpriv->ai_offset[chan_index];
		chan_index++;
		chan_index %= async->cmd.chanlist_len;
	}
}

#ifdef PCIDMA

static void ni_ai_setup_MITE_dma(comedi_device *dev,comedi_cmd *cmd)
{
	struct mite_struct *mite = devpriv->mite;
	struct mite_channel *mite_chan = &mite->channels[ AI_DMA_CHAN ];
	comedi_subdevice *s = dev->subdevices + 0;

	/* write alloc the entire buffer */
	comedi_buf_write_alloc(s->async, s->async->prealloc_bufsz);

	mite_chan->current_link = 0;
	mite_chan->dir = COMEDI_INPUT;
	switch(boardtype.reg_type)
	{
	case ni_reg_611x:
	case ni_reg_6143:
		mite_prep_dma(mite, AI_DMA_CHAN, 32, 16);
		break;
	case ni_reg_m_series:
		if(boardtype.adbits > 16)
			mite_prep_dma(mite, AI_DMA_CHAN, 32, 32);
		else
			mite_prep_dma(mite, AI_DMA_CHAN, 16, 16);	//guess
		break;
	default:
		mite_prep_dma(mite, AI_DMA_CHAN, 16, 16);
		break;
	};
	/*start the MITE*/
	mite_dma_arm(mite, AI_DMA_CHAN);
}

static void ni_ao_setup_MITE_dma(comedi_device *dev,comedi_cmd *cmd)
{
	struct mite_struct *mite = devpriv->mite;
	struct mite_channel *mite_chan = &mite->channels[ AO_DMA_CHAN ];
	comedi_subdevice *s = dev->subdevices + 1;

	devpriv->last_buf_write_count = s->async->buf_write_count;

	mite_chan->current_link = 0;
	mite_chan->dir = COMEDI_OUTPUT;
	if(boardtype.reg_type & (ni_reg_611x | ni_reg_6713))
		mite_prep_dma(mite, AO_DMA_CHAN, 32, 32);
	else
		mite_prep_dma(mite, AO_DMA_CHAN, 16, 16);

	/*start the MITE*/
	mite_dma_arm(mite, AO_DMA_CHAN);
}

#endif // PCIDMA

/*
   used for both cancel ioctl and board initialization

   this is pretty harsh for a cancel, but it works...
 */

static int ni_ai_reset(comedi_device *dev,comedi_subdevice *s)
{
#ifdef PCIDMA
	mite_dma_disarm(devpriv->mite, AI_DMA_CHAN);
#endif
	/* ai configuration */
	devpriv->stc_writew(dev, AI_Configuration_Start | AI_Reset, Joint_Reset_Register);

	ni_set_bits(dev, Interrupt_A_Enable_Register,
		AI_SC_TC_Interrupt_Enable | AI_START1_Interrupt_Enable|
		AI_START2_Interrupt_Enable| AI_START_Interrupt_Enable|
		AI_STOP_Interrupt_Enable|   AI_Error_Interrupt_Enable|
		AI_FIFO_Interrupt_Enable,0);

	ni_flush_ai_fifo(dev);

	if(boardtype.reg_type != ni_reg_6143)
		ni_writeb(0, Misc_Command);

	devpriv->stc_writew(dev, AI_Disarm, AI_Command_1_Register); /* reset pulses */
	devpriv->stc_writew(dev, AI_Start_Stop | AI_Mode_1_Reserved /*| AI_Trigger_Once */,
		AI_Mode_1_Register);
	devpriv->stc_writew(dev, 0x0000,AI_Mode_2_Register);
	/* generate FIFO interrupts on non-empty */
	devpriv->stc_writew(dev, (0<<6)|0x0000,AI_Mode_3_Register);
	if(boardtype.reg_type == ni_reg_611x){
		devpriv->stc_writew(dev, AI_SHIFTIN_Pulse_Width |
			AI_SOC_Polarity |
			AI_LOCALMUX_CLK_Pulse_Width, AI_Personal_Register);
		devpriv->stc_writew(dev, AI_SCAN_IN_PROG_Output_Select(3) |
			AI_EXTMUX_CLK_Output_Select(0) |
			AI_LOCALMUX_CLK_Output_Select(2) |
			AI_SC_TC_Output_Select(3) |
			AI_CONVERT_Output_Select(3),AI_Output_Control_Register);
	}else if(boardtype.reg_type == ni_reg_6143){
		devpriv->stc_writew(dev, AI_SHIFTIN_Pulse_Width |
			AI_SOC_Polarity |
			AI_LOCALMUX_CLK_Pulse_Width, AI_Personal_Register);
		devpriv->stc_writew(dev, AI_SCAN_IN_PROG_Output_Select(3) |
			AI_EXTMUX_CLK_Output_Select(0) |
			AI_LOCALMUX_CLK_Output_Select(2) |
			AI_SC_TC_Output_Select(3) |
			AI_CONVERT_Output_Select(2),AI_Output_Control_Register);
	}else{
		devpriv->stc_writew(dev, AI_SHIFTIN_Pulse_Width |
			AI_SOC_Polarity |
			AI_CONVERT_Pulse_Width |
			AI_LOCALMUX_CLK_Pulse_Width, AI_Personal_Register);
		devpriv->stc_writew(dev, AI_SCAN_IN_PROG_Output_Select(3) |
			AI_EXTMUX_CLK_Output_Select(0) |
			AI_LOCALMUX_CLK_Output_Select(2) |
			AI_SC_TC_Output_Select(3) |
			AI_CONVERT_Output_Select(2),AI_Output_Control_Register);
	}
	/* the following registers should not be changed, because there
	 * are no backup registers in devpriv.  If you want to change
	 * any of these, add a backup register and other appropriate code:
	 *	AI_Mode_1_Register
	 *	AI_Mode_3_Register
	 *	AI_Personal_Register
	 *	AI_Output_Control_Register
	*/
	devpriv->stc_writew(dev, AI_SC_TC_Error_Confirm | AI_START_Interrupt_Ack |
		AI_START2_Interrupt_Ack | AI_START1_Interrupt_Ack |
		AI_SC_TC_Interrupt_Ack | AI_Error_Interrupt_Ack |
		AI_STOP_Interrupt_Ack, Interrupt_A_Ack_Register); /* clear interrupts */

	devpriv->stc_writew(dev, AI_Configuration_End,Joint_Reset_Register);

	return 0;
}

static int ni_ai_poll(comedi_device *dev,comedi_subdevice *s)
{
	unsigned long flags = 0;
	int count;

	// lock to avoid race with interrupt handler
	if(in_interrupt() == 0)
		comedi_spin_lock_irqsave(&dev->spinlock, flags);
#ifndef PCIDMA
	ni_handle_fifo_dregs(dev);
#else
	ni_sync_ai_dma(devpriv->mite, dev);
#endif
	count = s->async->buf_write_count - s->async->buf_read_count;
	if(in_interrupt() == 0)
		comedi_spin_unlock_irqrestore(&dev->spinlock, flags);

	return count;
}


static int ni_ai_insn_read(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i,n;
	unsigned int mask;
	unsigned signbits;
	unsigned short d;
	unsigned long dl;

	ni_load_channelgain_list(dev,1,&insn->chanspec);

	ni_flush_ai_fifo(dev);

	mask=(1<<boardtype.adbits)-1;
	signbits=devpriv->ai_offset[0];
	if(boardtype.reg_type == ni_reg_611x){
		for(n=0; n < num_adc_stages_611x; n++){
			devpriv->stc_writew(dev, AI_CONVERT_Pulse, AI_Command_1_Register);
			comedi_udelay(1);
		}
		for(n=0; n<insn->n; n++){
			devpriv->stc_writew(dev, AI_CONVERT_Pulse, AI_Command_1_Register);
			/* The 611x has screwy 32-bit FIFOs. */
			d = 0;
			for(i=0; i<NI_TIMEOUT; i++){
				if(ni_readb(XXX_Status)&0x80)
				{
					d = ( ni_readl(ADC_FIFO_Data_611x) >> 16 ) & 0xffff;
					break;
				}
				if(!(devpriv->stc_readw(dev, AI_Status_1_Register)&AI_FIFO_Empty_St))
				{
					d = ni_readl(ADC_FIFO_Data_611x) & 0xffff;
					break;
				}
			}
			if(i==NI_TIMEOUT){
				rt_printk("ni_mio_common: timeout in 611x ni_ai_insn_read\n");
				return -ETIME;
			}
			d += signbits;
			data[ n ] = d;
		}
	}else if(boardtype.reg_type == ni_reg_6143){
		for(n = 0; n < insn->n; n++){
			devpriv->stc_writew(dev, AI_CONVERT_Pulse, AI_Command_1_Register);

			/* The 6143 has 32-bit FIFOs. You need to strobe a bit to move a single 16bit stranded sample into the FIFO */
			dl = 0;
			for(i = 0; i < NI_TIMEOUT; i++){
				if(ni_readl(AIFIFO_Status_6143) & 0x01)
				{
					ni_writel(0x01, AIFIFO_Control_6143);	// Get stranded sample into FIFO
					dl = ni_readl(AIFIFO_Data_6143);
					break;
				}
			}
			if(i == NI_TIMEOUT){
				rt_printk("ni_mio_common: timeout in 6143 ni_ai_insn_read\n");
				return -ETIME;
			}
			data[n] = (((dl >> 16) & 0xFFFF) + signbits) & 0xFFFF;
		}
	}else{
		for(n=0;n<insn->n;n++){
			devpriv->stc_writew(dev, AI_CONVERT_Pulse, AI_Command_1_Register);
			for(i=0;i<NI_TIMEOUT;i++){
				if(!(devpriv->stc_readw(dev, AI_Status_1_Register)&AI_FIFO_Empty_St))
			 		break;
			}
			if(i==NI_TIMEOUT){
				rt_printk("ni_mio_common: timeout in ni_ai_insn_read\n");
				return -ETIME;
			}
			if(boardtype.reg_type == ni_reg_m_series)
			{
				data[n] = ni_readl(M_Offset_AI_FIFO_Data);
				data[n] += signbits;
			}else
			{
				d = ni_readw(ADC_FIFO_Data_Register);
				d += signbits; /* subtle: needs to be short addition */
				data[n] = d;
			}
		}
	}
	return insn->n;
}

void ni_prime_channelgain_list(comedi_device *dev)
{
	int i;
	devpriv->stc_writew(dev, AI_CONVERT_Pulse, AI_Command_1_Register);
	for(i = 0; i < NI_TIMEOUT; ++i)
	{
		if(!(devpriv->stc_readw(dev, AI_Status_1_Register) & AI_FIFO_Empty_St))
		{
			devpriv->stc_writew(dev, 1, ADC_FIFO_Clear);
			return;
		}
		comedi_udelay(1);
	}
	rt_printk("ni_mio_common: timeout loading channel/gain list\n");
}

/* m series boards have new ai_config_fifo_data register which replaces
 * old configuration_memory_low and high registers.  But the old registers still
 * seem to work, so I'm not going to bother to change it yet. */
static void ni_m_series_load_channelgain_list(comedi_device *dev,unsigned int n_chan,
	unsigned int *list)
{
	unsigned int chan, range, aref;
	unsigned int i;
	unsigned int hi,lo;
	unsigned offset;
	unsigned int dither;
	unsigned int use_alt_src;
	unsigned range_code;
	
	devpriv->stc_writew(dev, 1, Configuration_Memory_Clear);

	offset = 1 << (boardtype.adbits - 1);
	for(i = 0; i < n_chan; i++)
	{
		chan = CR_CHAN(list[i]);
		aref = CR_AREF(list[i]);
		range = CR_RANGE(list[i]);
		dither = ((list[i] & CR_ALT_FILTER) != 0);
		use_alt_src = ((list[i] & CR_ALT_SOURCE) != 0);
		
		range_code = ni_gainlkup[boardtype.gainlkup][range];
		devpriv->ai_offset[i] = offset;

		hi = 0;
		if(use_alt_src)
		{
			unsigned bypass_bits = MSeries_AI_Bypass_Config_FIFO_Bit;
			bypass_bits |= chan;
			bypass_bits |= (devpriv->ai_calib_source) & (MSeries_AI_Bypass_Cal_Sel_Pos_Mask |
				MSeries_AI_Bypass_Cal_Sel_Neg_Mask | MSeries_AI_Bypass_Mode_Mux_Mask |
				MSeries_AO_Bypass_AO_Cal_Sel_Mask);
			bypass_bits |= MSeries_AI_Bypass_Gain_Bits(range_code);
			if(dither)
				bypass_bits |= MSeries_AI_Bypass_Dither_Bit;
			ni_writel(bypass_bits, M_Offset_AI_Config_FIFO_Bypass);
		}else
		{
			ni_writel(0, M_Offset_AI_Config_FIFO_Bypass);
			switch( aref )
			{
				case AREF_DIFF:
					hi |= AI_DIFFERENTIAL;
					break;
				case AREF_COMMON:
					hi |= AI_COMMON;
					break;
				case AREF_GROUND:
					hi |= AI_GROUND;
					break;
				case AREF_OTHER:
					break;
			}
		}
		hi |= AI_CONFIG_CHANNEL( chan );

		ni_writew(hi,Configuration_Memory_High);

		lo = range_code;
		if(i == n_chan - 1) lo |= AI_LAST_CHANNEL;
		if( dither ) lo |= AI_DITHER;

		ni_writew(lo,Configuration_Memory_Low);
	}
	ni_prime_channelgain_list(dev);
}

/*
 * Notes on the 6110 and 6111:
 * These boards a slightly different than the rest of the series, since
 * they have multiple A/D converters.
 * From the driver side, the configuration memory is a
 * little different.
 * Configuration Memory Low:
 *   bits 15-9: same
 *   bit 8: unipolar/bipolar (should be 0 for bipolar)
 *   bits 0-3: gain.  This is 4 bits instead of 3 for the other boards
 *       1001 gain=0.1 (+/- 50)
 *       1010 0.2
 *       1011 0.1
 *       0001 1
 *       0010 2
 *       0011 5
 *       0100 10
 *       0101 20
 *       0110 50
 * Configuration Memory High:
 *   bits 12-14: Channel Type
 *       001 for differential
 *       000 for calibration
 *   bit 11: coupling  (this is not currently handled)
 *       1 AC coupling
 *       0 DC coupling
 *   bits 0-2: channel
 *       valid channels are 0-3
 */
static void ni_load_channelgain_list(comedi_device *dev,unsigned int n_chan,
	unsigned int *list)
{
	unsigned int chan,range,aref;
	unsigned int i;
	unsigned int hi,lo;
	unsigned offset;
	unsigned int dither;

	if(boardtype.reg_type == ni_reg_m_series)
	{
		ni_m_series_load_channelgain_list(dev, n_chan, list);
		return;
	}
	if(n_chan == 1 && (boardtype.reg_type != ni_reg_611x) && (boardtype.reg_type != ni_reg_6143)){
		if(devpriv->changain_state && devpriv->changain_spec==list[0]){
			// ready to go.
			return;
		}
		devpriv->changain_state=1;
		devpriv->changain_spec=list[0];
	}else{
		devpriv->changain_state=0;
	}

	devpriv->stc_writew(dev, 1,Configuration_Memory_Clear);

	// Set up Calibration mode if required
	if(boardtype.reg_type == ni_reg_6143){
		if((list[0] & CR_ALT_SOURCE) && !devpriv->ai_calib_source_enabled){
			// Strobe Relay enable bit
			ni_writew(devpriv->ai_calib_source | Calibration_Channel_6143_RelayOn, Calibration_Channel_6143);
			ni_writew(devpriv->ai_calib_source, Calibration_Channel_6143);
			devpriv->ai_calib_source_enabled = 1;
			msleep_interruptible(100);	// Allow relays to change
		}
		else if(!(list[0] & CR_ALT_SOURCE) && devpriv->ai_calib_source_enabled){
			// Strobe Relay disable bit
			ni_writew(devpriv->ai_calib_source | Calibration_Channel_6143_RelayOff, Calibration_Channel_6143);
			ni_writew(devpriv->ai_calib_source, Calibration_Channel_6143);
			devpriv->ai_calib_source_enabled = 0;
			msleep_interruptible(100);	// Allow relays to change
		}
	}

	offset=1<<(boardtype.adbits-1);
	for(i=0;i<n_chan;i++){
		if((boardtype.reg_type != ni_reg_6143) && (list[i] & CR_ALT_SOURCE)){
			chan=devpriv->ai_calib_source;
		}else{
			chan=CR_CHAN(list[i]);
		}
		aref=CR_AREF(list[i]);
		range=CR_RANGE(list[i]);
		dither=((list[i]&CR_ALT_FILTER)!=0);

		/* fix the external/internal range differences */
		range = ni_gainlkup[boardtype.gainlkup][range];
		if(boardtype.reg_type == ni_reg_611x)
			devpriv->ai_offset[i] = offset;
		else
			devpriv->ai_offset[i] = (range&0x100)?0:offset;

		hi = 0;
		if( ( list[i] & CR_ALT_SOURCE ) )
		{
			if(boardtype.reg_type == ni_reg_611x)
				ni_writew(CR_CHAN(list[i])&0x0003, Calibration_Channel_Select_611x);
		}else
		{
			if(boardtype.reg_type == ni_reg_611x)
				aref = AREF_DIFF;
			else if(boardtype.reg_type == ni_reg_6143)
				aref = AREF_OTHER;
			switch( aref )
			{
				case AREF_DIFF:
					hi |= AI_DIFFERENTIAL;
					break;
				case AREF_COMMON:
					hi |= AI_COMMON;
					break;
				case AREF_GROUND:
					hi |= AI_GROUND;
					break;
				case AREF_OTHER:
					break;
			}
		}
		hi |= AI_CONFIG_CHANNEL( chan );

		ni_writew(hi,Configuration_Memory_High);

		if(boardtype.reg_type != ni_reg_6143){
			lo = range;
			if(i == n_chan - 1) lo |= AI_LAST_CHANNEL;
			if( dither ) lo |= AI_DITHER;

			ni_writew(lo,Configuration_Memory_Low);
		}
	}

	/* prime the channel/gain list */
	if((boardtype.reg_type != ni_reg_611x) && (boardtype.reg_type != ni_reg_6143)){
		ni_prime_channelgain_list(dev);
	}
}

#define TIMER_BASE 50 /* 20 Mhz base */

static int ni_ns_to_timer(int *nanosec,int round_mode)
{
	int divider,base;

	base=TIMER_BASE;

	switch(round_mode){
	case TRIG_ROUND_NEAREST:
	default:
		divider=(*nanosec+base/2)/base;
		break;
	case TRIG_ROUND_DOWN:
		divider=(*nanosec)/base;
		break;
	case TRIG_ROUND_UP:
		divider=(*nanosec+base-1)/base;
		break;
	}

	*nanosec=base*divider;
	return divider-1;
}

static int ni_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err=0;
	int tmp;
	int sources;

	/* step 1: make sure trigger sources are trivially valid */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW|TRIG_INT|TRIG_EXT;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER|TRIG_EXT;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	sources = TRIG_TIMER | TRIG_EXT;
	if((boardtype.reg_type == ni_reg_611x) || (boardtype.reg_type == ni_reg_6143)) sources |= TRIG_NOW;
	cmd->convert_src &= sources;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT|TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	/* note that mutual compatiblity is not an issue here */
	if(cmd->start_src!=TRIG_NOW &&
	   cmd->start_src!=TRIG_INT &&
	   cmd->start_src!=TRIG_EXT)err++;
	if(cmd->scan_begin_src!=TRIG_TIMER &&
	   cmd->scan_begin_src!=TRIG_EXT &&
	   cmd->scan_begin_src!=TRIG_OTHER)err++;
	if(cmd->convert_src!=TRIG_TIMER &&
	   cmd->convert_src!=TRIG_EXT &&
	   cmd->convert_src!=TRIG_NOW)err++;
	if(cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_NONE)err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_src==TRIG_EXT){
		/* external trigger */
		unsigned int tmp = CR_CHAN(cmd->start_arg);

		if(tmp > 16) tmp = 16;
		tmp |= (cmd->start_arg & (CR_INVERT | CR_EDGE));
		if(cmd->start_arg != tmp){
			cmd->start_arg = tmp;
			err++;
		}
	}else{
		if(cmd->start_arg!=0){
			/* true for both TRIG_NOW and TRIG_INT */
			cmd->start_arg=0;
			err++;
		}
	}
	if(cmd->scan_begin_src==TRIG_TIMER){
		if(cmd->scan_begin_arg<boardtype.ai_speed){
			cmd->scan_begin_arg=boardtype.ai_speed;
			err++;
		}
		if(cmd->scan_begin_arg>TIMER_BASE*0xffffff){
			cmd->scan_begin_arg=TIMER_BASE*0xffffff;
			err++;
		}
	}else if(cmd->scan_begin_src==TRIG_EXT){
		/* external trigger */
		unsigned int tmp = CR_CHAN(cmd->scan_begin_arg);

		if(tmp>16)tmp=16;
		tmp |= (cmd->scan_begin_arg & (CR_INVERT | CR_EDGE));
		if(cmd->scan_begin_arg!=tmp){
			cmd->scan_begin_arg = tmp;
			err++;
		}
	}else{ /* TRIG_OTHER */
		if(cmd->scan_begin_arg){
			cmd->scan_begin_arg=0;
			err++;
		}
	}
	if(cmd->convert_src==TRIG_TIMER){
		if((boardtype.reg_type == ni_reg_611x) || (boardtype.reg_type == ni_reg_6143)){
			if(cmd->convert_arg != 0){
				cmd->convert_arg = 0;
				err++;
			}
		}else{
			if(cmd->convert_arg<boardtype.ai_speed){
				cmd->convert_arg=boardtype.ai_speed;
				err++;
			}
			if(cmd->convert_arg>TIMER_BASE*0xffff){
				cmd->convert_arg=TIMER_BASE*0xffff;
				err++;
			}
		}
	}else if(cmd->convert_src == TRIG_EXT){
		/* external trigger */
		unsigned int tmp = CR_CHAN(cmd->convert_arg);

		if(tmp>16)tmp=16;
		tmp |= (cmd->convert_arg&(CR_ALT_FILTER|CR_INVERT));
		if(cmd->convert_arg!=tmp){
			cmd->convert_arg = tmp;
			err++;
		}
	}else if(cmd->convert_src == TRIG_NOW){
		if(cmd->convert_arg != 0){
			cmd->convert_arg = 0;
			err++;
		}
	}

	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src==TRIG_COUNT){
		unsigned int max_count = 0x01000000;

		if(boardtype.reg_type == ni_reg_611x )
			max_count -= num_adc_stages_611x;
		if(cmd->stop_arg > max_count){
			cmd->stop_arg = max_count;
			err++;
		}
		if(cmd->stop_arg < 1){
			cmd->stop_arg = 1;
			err++;
		}
	}else{
		/* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	if(cmd->scan_begin_src==TRIG_TIMER){
		tmp=cmd->scan_begin_arg;
		ni_ns_to_timer(&cmd->scan_begin_arg,cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->scan_begin_arg)err++;
	}
	if(cmd->convert_src==TRIG_TIMER){
		if((boardtype.reg_type != ni_reg_611x) && (boardtype.reg_type != ni_reg_6143)){
			tmp=cmd->convert_arg;
			ni_ns_to_timer(&cmd->convert_arg,cmd->flags&TRIG_ROUND_MASK);
			if(tmp!=cmd->convert_arg)err++;
			if(cmd->scan_begin_src==TRIG_TIMER &&
			cmd->scan_begin_arg<cmd->convert_arg*cmd->scan_end_arg){
				cmd->scan_begin_arg=cmd->convert_arg*cmd->scan_end_arg;
				err++;
			}
		}
	}

	if(err)return 4;

	return 0;
}

static int ni_ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	comedi_cmd *cmd=&s->async->cmd;
	int timer;
	int mode1=0; /* mode1 is needed for both stop and convert */
	int mode2=0;
	int start_stop_select=0;
	unsigned int stop_count;
	int interrupt_a_enable=0;

	MDPRINTK("ni_ai_cmd\n");
	if(dev->irq == 0)
	{
		comedi_error(dev, "cannot run command without an irq");
		return -EIO;
	}
	ni_flush_ai_fifo(dev);

	ni_load_channelgain_list(dev,cmd->chanlist_len,cmd->chanlist);

	/* start configuration */
	devpriv->stc_writew(dev, AI_Configuration_Start,Joint_Reset_Register);

	/* disable analog triggering for now, since it
	 * interferes with the use of pfi0 */
	devpriv->an_trig_etc_reg &= ~Analog_Trigger_Enable;
	devpriv->stc_writew(dev, devpriv->an_trig_etc_reg, Analog_Trigger_Etc_Register);

	switch(cmd->start_src){
		case TRIG_INT:
		case TRIG_NOW:
			devpriv->stc_writew(dev, AI_START2_Select(0)|
				AI_START1_Sync|AI_START1_Edge|AI_START1_Select(0),
				AI_Trigger_Select_Register);
			break;
		case TRIG_EXT:
		{
			int chan = CR_CHAN(cmd->start_arg);
			unsigned int bits = AI_START2_Select(0)|
				AI_START1_Sync |
				AI_START1_Select(chan + 1);
				
			if(cmd->start_arg & CR_INVERT)
				bits |= AI_START1_Polarity;
			if(cmd->start_arg & CR_EDGE)
				bits |= AI_START1_Edge;
			devpriv->stc_writew(dev, bits, AI_Trigger_Select_Register);
			break;
		}
	}

	mode2 &= ~AI_Pre_Trigger;
	mode2 &= ~AI_SC_Initial_Load_Source;
	mode2 &= ~AI_SC_Reload_Mode;
	devpriv->stc_writew(dev, mode2, AI_Mode_2_Register);

	if(cmd->chanlist_len == 1 || (boardtype.reg_type == ni_reg_611x) || (boardtype.reg_type == ni_reg_6143)){
		start_stop_select |= AI_STOP_Polarity;
		start_stop_select |= AI_STOP_Select( 31 ); // logic low
		start_stop_select |= AI_STOP_Sync;
	}else
	{
		start_stop_select |= AI_STOP_Select(19); // ai configuration memory
	}
	devpriv->stc_writew(dev, start_stop_select, AI_START_STOP_Select_Register);

	devpriv->ai_cmd2 = 0;
	switch(cmd->stop_src){
	case TRIG_COUNT:
		stop_count = cmd->stop_arg - 1;

		if(boardtype.reg_type == ni_reg_611x){
			// have to take 3 stage adc pipeline into account
			stop_count += num_adc_stages_611x;
		}
		/* stage number of scans */
		devpriv->stc_writel(dev,  stop_count, AI_SC_Load_A_Registers);

		mode1 |= AI_Start_Stop | AI_Mode_1_Reserved | AI_Trigger_Once;
		devpriv->stc_writew(dev, mode1,AI_Mode_1_Register);
		/* load SC (Scan Count) */
		devpriv->stc_writew(dev, AI_SC_Load,AI_Command_1_Register);

		devpriv->ai_continuous = 0;
		if( stop_count == 0 ){
			devpriv->ai_cmd2 |= AI_End_On_End_Of_Scan;
			interrupt_a_enable |= AI_STOP_Interrupt_Enable;
			// this is required to get the last sample for chanlist_len > 1, not sure why
			if(cmd->chanlist_len > 1)
				start_stop_select |= AI_STOP_Polarity | AI_STOP_Edge;
		}
		break;
	case TRIG_NONE:
		/* stage number of scans */
		devpriv->stc_writel(dev, 0,AI_SC_Load_A_Registers);

		mode1 |= AI_Start_Stop | AI_Mode_1_Reserved | AI_Continuous;
		devpriv->stc_writew(dev, mode1,AI_Mode_1_Register);

		/* load SC (Scan Count) */
		devpriv->stc_writew(dev, AI_SC_Load,AI_Command_1_Register);

		devpriv->ai_continuous = 1;

		break;
	}

	switch(cmd->scan_begin_src){
	case TRIG_TIMER:
		/*
			stop bits for non 611x boards
			AI_SI_Special_Trigger_Delay=0
			AI_Pre_Trigger=0
			AI_START_STOP_Select_Register:
			AI_START_Polarity=0 (?)	rising edge
			AI_START_Edge=1		edge triggered
			AI_START_Sync=1 (?)
			AI_START_Select=0		SI_TC
			AI_STOP_Polarity=0		rising edge
			AI_STOP_Edge=0		level
			AI_STOP_Sync=1
			AI_STOP_Select=19		external pin (configuration mem)
		 */
		start_stop_select |= AI_START_Edge | AI_START_Sync;
		devpriv->stc_writew(dev, start_stop_select, AI_START_STOP_Select_Register);

		mode2 |= AI_SI_Reload_Mode(0);
		/* AI_SI_Initial_Load_Source=A */
		mode2 &= ~AI_SI_Initial_Load_Source;
		//mode2 |= AI_SC_Reload_Mode;
		devpriv->stc_writew(dev, mode2, AI_Mode_2_Register);

		/* load SI */
		timer=ni_ns_to_timer(&cmd->scan_begin_arg,TRIG_ROUND_NEAREST);
		devpriv->stc_writel(dev, timer,AI_SI_Load_A_Registers);
		devpriv->stc_writew(dev, AI_SI_Load,AI_Command_1_Register);
		break;
	case TRIG_EXT:
		if( cmd->scan_begin_arg & CR_EDGE )
			start_stop_select |= AI_START_Edge;
		/* AI_START_Polarity==1 is falling edge */
		if( cmd->scan_begin_arg & CR_INVERT )
			start_stop_select |= AI_START_Polarity;
		if( cmd->scan_begin_src != cmd->convert_src ||
			( cmd->scan_begin_arg & ~CR_EDGE ) != ( cmd->convert_arg & ~CR_EDGE ) )
			start_stop_select |= AI_START_Sync;
		start_stop_select |= AI_START_Select(1 + CR_CHAN(cmd->scan_begin_arg));
		devpriv->stc_writew(dev, start_stop_select, AI_START_STOP_Select_Register);
		break;
	}

	switch(cmd->convert_src){
	case TRIG_TIMER:
	case TRIG_NOW:
		if( cmd->convert_arg == 0 || cmd->convert_src == TRIG_NOW )
			timer = 1;
		else
			timer=ni_ns_to_timer(&cmd->convert_arg, TRIG_ROUND_NEAREST);
		devpriv->stc_writew(dev, 1,AI_SI2_Load_A_Register); /* 0,0 does not work. */
		devpriv->stc_writew(dev, timer,AI_SI2_Load_B_Register);

		/* AI_SI2_Reload_Mode = alternate */
		/* AI_SI2_Initial_Load_Source = A */
		mode2 &= ~AI_SI2_Initial_Load_Source;
		mode2 |= AI_SI2_Reload_Mode;
		devpriv->stc_writew(dev,  mode2, AI_Mode_2_Register);

		/* AI_SI2_Load */
		devpriv->stc_writew(dev, AI_SI2_Load,AI_Command_1_Register);

		mode2 |= AI_SI2_Reload_Mode; // alternate
		mode2 |= AI_SI2_Initial_Load_Source; // B

		devpriv->stc_writew(dev, mode2,AI_Mode_2_Register);
		break;
	case TRIG_EXT:
		mode1 |= AI_CONVERT_Source_Select(1+cmd->convert_arg);
		if( ( cmd->convert_arg & CR_INVERT ) == 0 )
			mode1 |= AI_CONVERT_Source_Polarity;
		devpriv->stc_writew(dev, mode1,AI_Mode_1_Register);

		mode2 |= AI_Start_Stop_Gate_Enable | AI_SC_Gate_Enable;
		devpriv->stc_writew(dev, mode2, AI_Mode_2_Register);

		break;
	}

	if(dev->irq){

		/* interrupt on FIFO, errors, SC_TC */
		interrupt_a_enable |= AI_Error_Interrupt_Enable|
			AI_SC_TC_Interrupt_Enable;

#ifndef PCIDMA
		interrupt_a_enable|=AI_FIFO_Interrupt_Enable;
#endif

		if(cmd->flags & TRIG_WAKE_EOS || (devpriv->ai_cmd2 & AI_End_On_End_Of_Scan)){
			/* wake on end-of-scan */
			devpriv->aimode=AIMODE_SCAN;
		}else{
			devpriv->aimode=AIMODE_HALF_FULL;
		}

		switch(devpriv->aimode){
		case AIMODE_HALF_FULL:
			/*generate FIFO interrupts and DMA requests on half-full */
#ifdef PCIDMA
			devpriv->stc_writew(dev, AI_FIFO_Mode_HF_to_E, AI_Mode_3_Register);
#else
			devpriv->stc_writew(dev, AI_FIFO_Mode_HF, AI_Mode_3_Register);
#endif
			break;
		case AIMODE_SAMPLE:
			/*generate FIFO interrupts on non-empty */
			devpriv->stc_writew(dev, AI_FIFO_Mode_NE, AI_Mode_3_Register);
			break;
		case AIMODE_SCAN:
#ifdef PCIDMA
			devpriv->stc_writew(dev, AI_FIFO_Mode_NE, AI_Mode_3_Register);
#else
			devpriv->stc_writew(dev, AI_FIFO_Mode_HF, AI_Mode_3_Register);
#endif
			interrupt_a_enable |= AI_STOP_Interrupt_Enable;
			break;
		default:
			break;
		}

		devpriv->stc_writew(dev, 0x3f80,Interrupt_A_Ack_Register); /* clear interrupts */

		ni_set_bits(dev, Interrupt_A_Enable_Register, interrupt_a_enable, 1);

		MDPRINTK("Interrupt_A_Enable_Register = 0x%04x\n",devpriv->int_a_enable_reg);
	}else{
		/* interrupt on nothing */
		ni_set_bits(dev, Interrupt_A_Enable_Register, ~0, 0);

		/* XXX start polling if necessary */
		MDPRINTK("interrupting on nothing\n");
	}

	/* end configuration */
	devpriv->stc_writew(dev, AI_Configuration_End,Joint_Reset_Register);

	switch(cmd->scan_begin_src){
	case TRIG_TIMER:
		devpriv->stc_writew(dev, AI_SI2_Arm | AI_SI_Arm | AI_DIV_Arm | AI_SC_Arm,
			AI_Command_1_Register);
		break;
	case TRIG_EXT:
		/* XXX AI_SI_Arm? */
		devpriv->stc_writew(dev, AI_SI2_Arm | AI_SI_Arm | AI_DIV_Arm | AI_SC_Arm,
			AI_Command_1_Register);
		break;
	}

#ifdef PCIDMA
	ni_ai_setup_MITE_dma(dev,cmd);
	//mite_dump_regs(devpriv->mite);
#endif

	switch(cmd->start_src){
	case TRIG_NOW:
		/* AI_START1_Pulse */
		devpriv->stc_writew(dev,  AI_START1_Pulse | devpriv->ai_cmd2, AI_Command_2_Register );
		s->async->inttrig=NULL;
		break;
	case TRIG_EXT:
		s->async->inttrig=NULL;
		break;
	case TRIG_INT:
		s->async->inttrig=ni_ai_inttrig;
		break;
	}

	MDPRINTK("exit ni_ai_cmd\n");

	return 0;
}

static int ni_ai_inttrig(comedi_device *dev,comedi_subdevice *s,
	unsigned int trignum)
{
	if(trignum!=0)return -EINVAL;

	devpriv->stc_writew(dev,  AI_START1_Pulse | devpriv->ai_cmd2, AI_Command_2_Register );
	s->async->inttrig=NULL;

	return 1;
}

static int ni_ai_config_analog_trig(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data);

static int ni_ai_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	if(insn->n<1)return -EINVAL;

	switch(data[0]){
	case INSN_CONFIG_ANALOG_TRIG:
		return ni_ai_config_analog_trig(dev,s,insn,data);
	case INSN_CONFIG_ALT_SOURCE:
		if(boardtype.reg_type == ni_reg_m_series)
		{
			if(data[1] & ~(MSeries_AI_Bypass_Cal_Sel_Pos_Mask |
				MSeries_AI_Bypass_Cal_Sel_Neg_Mask | MSeries_AI_Bypass_Mode_Mux_Mask |
				MSeries_AO_Bypass_AO_Cal_Sel_Mask))
			{
				return -EINVAL;
			}
			devpriv->ai_calib_source = data[1];
		} else if(boardtype.reg_type == ni_reg_6143)
		{
			unsigned int calib_source;

			calib_source = data[1] & 0xf;
	
	
			if(calib_source > 0xF)
				return -EINVAL;
			
			devpriv->ai_calib_source = calib_source;
			ni_writew(calib_source, Calibration_Channel_6143);
		}else
		{
			unsigned int calib_source;
			unsigned int calib_source_adjust;
	
			calib_source = data[1] & 0xf;
			calib_source_adjust = ( data[1] >> 4 ) & 0xff;
	
			if(calib_source >= 8)
				return -EINVAL;
			devpriv->ai_calib_source = calib_source;
			if(boardtype.reg_type == ni_reg_611x){
				ni_writeb( calib_source_adjust, Cal_Gain_Select_611x );
			}
		}
		return 2;
	default:
		break;
	}

	return -EINVAL;
}

static int ni_ai_config_analog_trig(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	unsigned int a,b,modebits;
	int err=0;

	/* data[1] is flags
	 * data[2] is analog line
	 * data[3] is set level
	 * data[4] is reset level */
	if(!boardtype.has_analog_trig)return -EINVAL;
	if(insn->n!=5)return -EINVAL;
	if((data[1]&0xffff0000) != COMEDI_EV_SCAN_BEGIN){
		data[1]&=~(COMEDI_EV_SCAN_BEGIN&0xffff);
		err++;
	}
	if(data[2]>=boardtype.n_adchan){
		data[2]=boardtype.n_adchan-1;
		err++;
	}
	if(data[3]>255){ /* a */
		data[3]=255;
		err++;
	}
	if(data[4]>255){ /* b */
		data[4]=255;
		err++;
	}
	/*
	 * 00 ignore
	 * 01 set
	 * 10 reset
	 *
	 * modes:
	 *   1 level:			 +b-   +a-
	 *     high mode		00 00 01 10
	 *     low mode			00 00 10 01
	 *   2 level: (a<b)
	 *     hysteresis low mode	10 00 00 01
	 *     hysteresis high mode	01 00 00 10
	 *     middle mode		10 01 01 10
	 */

	a=data[3];
	b=data[4];
	modebits=data[1]&0xff;
	if(modebits&0xf0){
		/* two level mode */
		if(b<a){
			/* swap order */
			a=data[4];
			b=data[3];
			modebits=((data[1]&0xf)<<4)|((data[1]&0xf0)>>4);
		}
		devpriv->atrig_low = a;
		devpriv->atrig_high = b;
		switch(modebits){
		case 0x81:	/* low hysteresis mode */
			devpriv->atrig_mode = 6;
			break;
		case 0x42:	/* high hysteresis mode */
			devpriv->atrig_mode = 3;
			break;
		case 0x96:	/* middle window mode */
			devpriv->atrig_mode = 2;
			break;
		default:
			data[1]&=~0xff;
			err++;
		}
	}else{
		/* one level mode */
		if(b!=0){
			data[4]=0;
			err++;
		}
		switch(modebits){
		case 0x06:	/* high window mode */
			devpriv->atrig_high = a;
			devpriv->atrig_mode = 0;
			break;
		case 0x09:	/* low window mode */
			devpriv->atrig_low = a;
			devpriv->atrig_mode = 1;
			break;
		default:
			data[1]&=~0xff;
			err++;
		}
	}
	if(err)return -EAGAIN;
	return 5;
}

/* munge data from unsigned to 2's complement for analog output bipolar modes */
static void ni_ao_munge(comedi_device *dev, comedi_subdevice *s,
	void *data, unsigned int num_bytes, unsigned int chan_index )
{
	comedi_async *async = s->async;
	unsigned int range;
	unsigned int i;
	unsigned int offset;
	unsigned int length = num_bytes / sizeof( sampl_t );
	sampl_t *array = data;

	offset = 1 << (boardtype.aobits - 1);
	for(i = 0; i < length; i++)
	{
		range = CR_RANGE( async->cmd.chanlist[ chan_index ] );
		if(boardtype.ao_unipolar == 0 || (range & 1) == 0 )
			array[i] -= offset;
#ifdef PCIDMA
		array[i] = cpu_to_le16( array[i] );
#endif
		chan_index++;
		chan_index %= async->cmd.chanlist_len;
	}
}

static int ni_ao_config_chanlist(comedi_device *dev, comedi_subdevice *s,
	unsigned int chanspec[], unsigned int n_chans)
{
	unsigned int range;
	unsigned int chan;
	unsigned int conf;
	int i;
	int invert = 0;

	for(i=0;i<n_chans;i++){
		chan = CR_CHAN(chanspec[i]);
		range = CR_RANGE(chanspec[i]);
		if(boardtype.reg_type == ni_reg_m_series)
		{
 			comedi_krange *krange = s->range_table->range + range;
			invert = 0;
			conf = 0;
			switch(krange->max - krange->min)
			{
			case 20000000:
				conf |= MSeries_AO_DAC_Reference_10V_Internal_Bits;
				ni_writeb(0, M_Offset_AO_Reference_Attenuation(chan));
				break;
			case 10000000:
				conf |= MSeries_AO_DAC_Reference_5V_Internal_Bits;
				ni_writeb(0, M_Offset_AO_Reference_Attenuation(chan));
				break;
			case 4000000:
				conf |= MSeries_AO_DAC_Reference_10V_Internal_Bits;
				ni_writeb(MSeries_Attenuate_x5_Bit, M_Offset_AO_Reference_Attenuation(chan));
				break;
			case 2000000:
				conf |= MSeries_AO_DAC_Reference_5V_Internal_Bits;
				ni_writeb(MSeries_Attenuate_x5_Bit, M_Offset_AO_Reference_Attenuation(chan));
				break;
			default:
				rt_printk("%s: bug! unhandled ao reference voltage\n", __FUNCTION__);
				break;
			}
			switch(krange->max + krange->min)
			{
			case 0:
				conf |= MSeries_AO_DAC_Offset_0V_Bits;
				break;
			case 10000000:
				conf |= MSeries_AO_DAC_Offset_5V_Bits;
				break;
			default:
				rt_printk("%s: bug! unhandled ao offset voltage\n", __FUNCTION__);
				break;
			}
			ni_writeb(conf, M_Offset_AO_Config_Bank(chan));
		}else
		{
			conf = AO_Channel(chan);
	
			if(boardtype.ao_unipolar){
				if((range&1) == 0){
					conf |= AO_Bipolar;
					invert = (1<<(boardtype.aobits-1));
				}else{
					invert = 0;
				}
				if(range&2)
					conf |= AO_Ext_Ref;
			}else{
				conf |= AO_Bipolar;
				invert = (1<<(boardtype.aobits-1));
			}
	
			/* not all boards can deglitch, but this shouldn't hurt */
			if(chanspec[i] & CR_DEGLITCH)
				conf |= AO_Deglitch;
	
			/* analog reference */
			/* AREF_OTHER connects AO ground to AI ground, i think */
			conf |= (CR_AREF(chanspec[i])==AREF_OTHER)? AO_Ground_Ref : 0;

			ni_writew(conf,AO_Configuration);
		}
		devpriv->ao_conf[chan] = conf;
	}
	return invert;
}

static int ni_ao_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	data[0] = devpriv->ao[CR_CHAN(insn->chanspec)];

	return 1;
}

static int ni_ao_insn_write(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	unsigned int chan = CR_CHAN(insn->chanspec);
	unsigned int invert;

	invert = ni_ao_config_chanlist(dev,s,&insn->chanspec,1);

	devpriv->ao[chan] = data[0];

	if(boardtype.reg_type == ni_reg_m_series)
	{
		ni_writew(data[0], M_Offset_DAC_Direct_Data(chan));
	}
	else
		ni_writew(data[0] ^ invert,(chan)? DAC1_Direct_Data : DAC0_Direct_Data);

	return 1;
}

static int ni_ao_insn_write_671x(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	unsigned int chan = CR_CHAN(insn->chanspec);
	unsigned int invert;

	ao_win_out(1 << chan, AO_Immediate_671x);
	invert = 1 << (boardtype.aobits - 1);

	ni_ao_config_chanlist(dev,s,&insn->chanspec,1);

	devpriv->ao[chan] = data[0];
	ao_win_out(data[0] ^ invert, DACx_Direct_Data_671x(chan));

	return 1;
}

static int ni_ao_inttrig(comedi_device *dev,comedi_subdevice *s,
	unsigned int trignum)
{
	int ret;
	int interrupt_b_bits;
	int i;
	static const int timeout = 1000;
	
	if(trignum!=0)return -EINVAL;

	ni_set_bits(dev, Interrupt_B_Enable_Register, AO_FIFO_Interrupt_Enable | AO_Error_Interrupt_Enable, 0);
	interrupt_b_bits = AO_Error_Interrupt_Enable;
#ifdef PCIDMA
	devpriv->stc_writew(dev, 1, DAC_FIFO_Clear);
	if(boardtype.reg_type & ni_reg_6xxx_mask)
		ni_ao_win_outl(dev, 0x6, AO_FIFO_Offset_Load_611x);
	ni_ao_setup_MITE_dma(dev, &s->async->cmd);
	ret = ni_ao_wait_for_dma_load(dev);
	if(ret < 0) return ret;

#else
	ret = ni_ao_prep_fifo(dev,s);
	if(ret==0)return -EPIPE;

	interrupt_b_bits |= AO_FIFO_Interrupt_Enable;
#endif
	
	devpriv->stc_writew(dev, devpriv->ao_mode3|AO_Not_An_UPDATE,AO_Mode_3_Register);
	devpriv->stc_writew(dev, devpriv->ao_mode3,AO_Mode_3_Register);
	/* wait for DACs to be loaded */
	for(i = 0; i < timeout; i++)
	{
		comedi_udelay(10);
		if((devpriv->stc_readw(dev, Joint_Status_2_Register) & AO_TMRDACWRs_In_Progress_St) == 0)
			break;
	}
	if(i == timeout)
	{
		comedi_error(dev, "timed out waiting for AO_TMRDACWRs_In_Progress_St to clear");
		return -EIO;
	}
	// stc manual says we are need to clear error interrupt after AO_TMRDACWRs_In_Progress_St clears
	devpriv->stc_writew(dev, AO_Error_Interrupt_Ack, Interrupt_B_Ack_Register);
	
	ni_set_bits(dev, Interrupt_B_Enable_Register, interrupt_b_bits, 1);

	devpriv->stc_writew(dev, devpriv->ao_cmd1|AO_UI_Arm|AO_UC_Arm|AO_BC_Arm|AO_DAC1_Update_Mode|AO_DAC0_Update_Mode,
		AO_Command_1_Register);

	devpriv->stc_writew(dev, devpriv->ao_cmd2|AO_START1_Pulse,AO_Command_2_Register);

	s->async->inttrig=NULL;

	return 0;
}

static int ni_ao_cmd(comedi_device *dev,comedi_subdevice *s)
{
	comedi_cmd *cmd = &s->async->cmd;
	int trigvar;
	int bits;
	int i;
	
	if(dev->irq == 0)
	{
		comedi_error(dev, "cannot run command without an irq");
		return -EIO;
	}
	trigvar = ni_ns_to_timer(&cmd->scan_begin_arg,TRIG_ROUND_NEAREST);

	devpriv->stc_writew(dev, AO_Configuration_Start,Joint_Reset_Register);

	devpriv->stc_writew(dev, AO_Disarm,AO_Command_1_Register);

	if(boardtype.reg_type & ni_reg_6xxx_mask)
	{
		ao_win_out(CLEAR_WG, AO_Misc_611x);

		bits = 0;
		for(i = 0; i < cmd->chanlist_len; i++)
		{
			int chan;

			chan = CR_CHAN(cmd->chanlist[i]);
			bits |= 1 << chan;
			ao_win_out(chan, AO_Waveform_Generation_611x);
		}
		ao_win_out(bits, AO_Timed_611x);
	}

	ni_ao_config_chanlist(dev,s,cmd->chanlist,cmd->chanlist_len);

	if(cmd->stop_src==TRIG_NONE){
		devpriv->ao_mode1|=AO_Continuous;
		devpriv->ao_mode1&=~AO_Trigger_Once;
	}else{
		devpriv->ao_mode1&=~AO_Continuous;
		devpriv->ao_mode1|=AO_Trigger_Once;
	}
	devpriv->stc_writew(dev, devpriv->ao_mode1,AO_Mode_1_Register);
	devpriv->ao_trigger_select&=~(AO_START1_Polarity|AO_START1_Select(-1));
	devpriv->ao_trigger_select|=AO_START1_Edge|AO_START1_Sync;
	devpriv->stc_writew(dev, devpriv->ao_trigger_select,AO_Trigger_Select_Register);
	devpriv->ao_mode3&=~AO_Trigger_Length;
	devpriv->stc_writew(dev, devpriv->ao_mode3,AO_Mode_3_Register);

	devpriv->stc_writew(dev, devpriv->ao_mode1,AO_Mode_1_Register);
	devpriv->ao_mode2&=~AO_BC_Initial_Load_Source;
	devpriv->stc_writew(dev, devpriv->ao_mode2,AO_Mode_2_Register);
	if(cmd->stop_src==TRIG_NONE){
		devpriv->stc_writel(dev, 0xffffff,AO_BC_Load_A_Register);
	}else{
		devpriv->stc_writel(dev, 0,AO_BC_Load_A_Register);
	}
	devpriv->stc_writew(dev, AO_BC_Load,AO_Command_1_Register);
	devpriv->ao_mode2&=~AO_UC_Initial_Load_Source;
	devpriv->stc_writew(dev, devpriv->ao_mode2,AO_Mode_2_Register);
	switch(cmd->stop_src){
	case TRIG_COUNT:
		devpriv->stc_writel(dev, cmd->stop_arg,AO_UC_Load_A_Register);
		devpriv->stc_writew(dev, AO_UC_Load,AO_Command_1_Register);
		devpriv->stc_writel(dev, cmd->stop_arg - 1,AO_UC_Load_A_Register);
		break;
	case TRIG_NONE:
		devpriv->stc_writel(dev, 0xffffff,AO_UC_Load_A_Register);
		devpriv->stc_writew(dev, AO_UC_Load,AO_Command_1_Register);
		devpriv->stc_writel(dev, 0xffffff,AO_UC_Load_A_Register);
		break;
	default:
		devpriv->stc_writel(dev, 0,AO_UC_Load_A_Register);
		devpriv->stc_writew(dev, AO_UC_Load,AO_Command_1_Register);
		devpriv->stc_writel(dev, cmd->stop_arg,AO_UC_Load_A_Register);
	}

	devpriv->ao_cmd2&=~AO_BC_Gate_Enable;
	devpriv->stc_writew(dev, devpriv->ao_cmd2,AO_Command_2_Register);
	devpriv->ao_mode1&=~(AO_UI_Source_Select(0x1f)|AO_UI_Source_Polarity);
	devpriv->stc_writew(dev, devpriv->ao_mode1,AO_Mode_1_Register);
	devpriv->ao_mode2&=~(AO_UI_Reload_Mode(3)|AO_UI_Initial_Load_Source);
	devpriv->stc_writew(dev, devpriv->ao_mode2,AO_Mode_2_Register);
	devpriv->stc_writel(dev, 1,AO_UI_Load_A_Register);
	devpriv->stc_writew(dev, AO_UI_Load,AO_Command_1_Register);
	devpriv->stc_writel(dev, trigvar,AO_UI_Load_A_Register);

	if((boardtype.reg_type & ni_reg_6xxx_mask) == 0){
		if(cmd->scan_end_arg>1){
			devpriv->ao_mode1|=AO_Multiple_Channels;
			devpriv->stc_writew(dev, AO_Number_Of_Channels(cmd->scan_end_arg-1)|
				AO_UPDATE_Output_Select(1),
				AO_Output_Control_Register);
		}else{
			devpriv->ao_mode1&=~AO_Multiple_Channels;
			devpriv->stc_writew(dev, AO_Number_Of_Channels(CR_CHAN(cmd->chanlist[0]))|
				AO_UPDATE_Output_Select(1),
				AO_Output_Control_Register);
		}
		devpriv->stc_writew(dev, devpriv->ao_mode1,AO_Mode_1_Register);
	}

	devpriv->stc_writew(dev, AO_DAC0_Update_Mode|AO_DAC1_Update_Mode,AO_Command_1_Register);

	devpriv->ao_mode3|=AO_Stop_On_Overrun_Error;
	devpriv->stc_writew(dev, devpriv->ao_mode3,AO_Mode_3_Register);

	devpriv->ao_mode2 &= AO_FIFO_Mode_Mask;
#ifdef PCIDMA
	devpriv->ao_mode2 |= AO_FIFO_Mode_HF_to_F;
#else
	devpriv->ao_mode2 |= AO_FIFO_Mode_HF;
#endif
	devpriv->ao_mode2 &= ~AO_FIFO_Retransmit_Enable;
	devpriv->stc_writew(dev, devpriv->ao_mode2,AO_Mode_2_Register);

	bits = AO_BC_Source_Select | AO_UPDATE_Pulse_Width |
		AO_TMRDACWR_Pulse_Width;
	if( boardtype.ao_fifo_depth )
		bits |= AO_FIFO_Enable;
	else
		bits |= AO_DMA_PIO_Control;
	devpriv->stc_writew(dev, bits, AO_Personal_Register);
	// enable sending of ao dma requests
	devpriv->stc_writew(dev, AO_AOFREQ_Enable, AO_Start_Select_Register);

	devpriv->stc_writew(dev, AO_Configuration_End,Joint_Reset_Register);

	if(cmd->stop_src==TRIG_COUNT) {
		devpriv->stc_writew(dev, AO_BC_TC_Interrupt_Ack,Interrupt_B_Ack_Register);
		ni_set_bits(dev, Interrupt_B_Enable_Register,
			AO_BC_TC_Interrupt_Enable, 1);
	}

	s->async->inttrig=ni_ao_inttrig;

	return 0;
}

static int ni_ao_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err=0;
	int tmp;

	/* step 1: make sure trigger sources are trivially valid */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_INT;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_NOW;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT|TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	if(cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_NONE)err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}
#if 0
	/* XXX need ao_speed */
	if(cmd->scan_begin_arg<boardtype.ao_speed){
		cmd->scan_begin_arg=boardtype.ao_speed;
		err++;
	}
#endif
	if(cmd->scan_begin_arg>TIMER_BASE*0xffffff){ /* XXX check */
		cmd->scan_begin_arg=TIMER_BASE*0xffffff;
		err++;
	}
	if(cmd->convert_arg!=0){
		cmd->convert_arg=0;
		err++;
	}
	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src==TRIG_COUNT){ /* XXX check */
		if(cmd->stop_arg>0x00ffffff){
			cmd->stop_arg=0x00ffffff;
			err++;
		}
	}else{
		/* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;

	/* step 4: fix up any arguments */

	tmp = cmd->scan_begin_arg;
	ni_ns_to_timer(&cmd->scan_begin_arg,cmd->flags&TRIG_ROUND_MASK);
	if(tmp!=cmd->scan_begin_arg)err++;

	if(err)return 4;

	/* step 5: fix up chanlist */

	if(err)return 5;

	return 0;
}


static int ni_ao_reset(comedi_device *dev,comedi_subdevice *s)
{
	//devpriv->ao0p=0x0000;
	//ni_writew(devpriv->ao0p,AO_Configuration);

	//devpriv->ao1p=AO_Channel(1);
	//ni_writew(devpriv->ao1p,AO_Configuration);

#ifdef PCIDMA
	mite_dma_disarm(devpriv->mite, AO_DMA_CHAN);
	writel(CHOR_DMARESET | CHOR_FRESET, devpriv->mite->mite_io_addr + MITE_CHOR(AO_DMA_CHAN));
#endif

	devpriv->stc_writew(dev, AO_Configuration_Start,Joint_Reset_Register);
	devpriv->stc_writew(dev, AO_Disarm,AO_Command_1_Register);
	ni_set_bits(dev,Interrupt_B_Enable_Register,~0,0);
	devpriv->stc_writew(dev, AO_BC_Source_Select, AO_Personal_Register);
	devpriv->stc_writew(dev, 0x3f98,Interrupt_B_Ack_Register);
	devpriv->stc_writew(dev, AO_BC_Source_Select | AO_UPDATE_Pulse_Width |
		AO_TMRDACWR_Pulse_Width, AO_Personal_Register);
	devpriv->stc_writew(dev, 0,AO_Output_Control_Register);
	devpriv->stc_writew(dev, 0,AO_Start_Select_Register);
	devpriv->ao_cmd1=0;
	devpriv->stc_writew(dev, devpriv->ao_cmd1,AO_Command_1_Register);
	devpriv->ao_cmd2=0;
	devpriv->ao_mode1=0;
	devpriv->ao_mode2=0;
	if(boardtype.reg_type == ni_reg_m_series)
		devpriv->ao_mode3 = AO_Last_Gate_Disable;
	else
		devpriv->ao_mode3 = 0;
	devpriv->stc_writew(dev, devpriv->ao_mode3, AO_Mode_3_Register);
	devpriv->ao_trigger_select=0;
	if(boardtype.reg_type & ni_reg_6xxx_mask){
		ao_win_out(0x3, AO_Immediate_671x);
		ao_win_out(CLEAR_WG, AO_Misc_611x);
	}
	devpriv->stc_writew(dev, AO_Configuration_End, Joint_Reset_Register);

	return 0;
}

static int ni_dio_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
#ifdef DEBUG_DIO
	printk("ni_dio_insn_config() chan=%d io=%d\n",
		CR_CHAN(insn->chanspec),data[0]);
#endif
	switch(data[0]){
	case INSN_CONFIG_DIO_OUTPUT:
		s->io_bits |= 1<<CR_CHAN(insn->chanspec);
		break;
	case INSN_CONFIG_DIO_INPUT:
		s->io_bits &= ~(1<<CR_CHAN(insn->chanspec));
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] = (s->io_bits & (1<<CR_CHAN(insn->chanspec))) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
	}

	devpriv->dio_control &= ~DIO_Pins_Dir_Mask;
	devpriv->dio_control |= DIO_Pins_Dir(s->io_bits);
	devpriv->stc_writew(dev, devpriv->dio_control,DIO_Control_Register);

	return 1;
}

static int ni_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
#ifdef DEBUG_DIO
	printk("ni_dio_insn_bits() mask=0x%x bits=0x%x\n",data[0],data[1]);
#endif
	if(insn->n!=2)return -EINVAL;
	if(data[0]){
		/* Perform check to make sure we're not using the
		   serial part of the dio */
		if((data[0] & (DIO_SDIN | DIO_SDOUT)) && devpriv->serial_interval_ns)
			return -EBUSY;

		s->state &= ~data[0];
		s->state |= (data[0]&data[1]);
		devpriv->dio_output &= ~DIO_Parallel_Data_Mask;
		devpriv->dio_output |= DIO_Parallel_Data_Out(s->state);
		devpriv->stc_writew(dev, devpriv->dio_output,DIO_Output_Register);
	}
	data[1] = devpriv->stc_readw(dev, DIO_Parallel_Input_Register);

	return 2;
}

static int ni_serial_insn_config(comedi_device *dev,comedi_subdevice *s,
				 comedi_insn *insn,lsampl_t *data)
{
	int err = insn->n;
	unsigned char byte_out, byte_in;

	if(insn->n!=2)return -EINVAL;

	switch(data[0]) {
	case INSN_CONFIG_SERIAL_CLOCK:

#ifdef DEBUG_DIO
		printk("SPI serial clock Config cd\n", data[1]);
#endif
		devpriv->serial_hw_mode = 1;
		devpriv->dio_control |= DIO_HW_Serial_Enable;

		if(data[1] == SERIAL_DISABLED) {
			devpriv->serial_hw_mode = 0;
			devpriv->dio_control &= ~(DIO_HW_Serial_Enable |
						  DIO_Software_Serial_Control);
			data[1] = SERIAL_DISABLED;
			devpriv->serial_interval_ns = data[1];
		}
		else if(data[1] <= SERIAL_600NS) {
			/* Warning: this clock speed is too fast to reliably
			control SCXI. */
			devpriv->dio_control &= ~DIO_HW_Serial_Timebase;
			devpriv->clock_and_fout |= Slow_Internal_Timebase;
			devpriv->clock_and_fout &= ~DIO_Serial_Out_Divide_By_2;
			data[1] = SERIAL_600NS;
			devpriv->serial_interval_ns = data[1];
		}
		else if(data[1] <= SERIAL_1_2US) {
			devpriv->dio_control &= ~DIO_HW_Serial_Timebase;
			devpriv->clock_and_fout |= Slow_Internal_Timebase |
				DIO_Serial_Out_Divide_By_2;
			data[1] = SERIAL_1_2US;
			devpriv->serial_interval_ns = data[1];
		}
		else if(data[1] <= SERIAL_10US) {
			devpriv->dio_control |= DIO_HW_Serial_Timebase;
			devpriv->clock_and_fout |= Slow_Internal_Timebase |
				DIO_Serial_Out_Divide_By_2;
			/* Note: DIO_Serial_Out_Divide_By_2 only affects
			600ns/1.2us. If you turn divide_by_2 off with the
			slow clock, you will still get 10us, except then
			all your delays are wrong. */
			data[1] = SERIAL_10US;
			devpriv->serial_interval_ns = data[1];
		}
		else {
			devpriv->dio_control &= ~(DIO_HW_Serial_Enable |
						  DIO_Software_Serial_Control);
			devpriv->serial_hw_mode = 0;
			data[1] = (data[1] / 1000) * 1000;
			devpriv->serial_interval_ns = data[1];
		}

		devpriv->stc_writew(dev, devpriv->dio_control,DIO_Control_Register);
		devpriv->stc_writew(dev, devpriv->clock_and_fout,Clock_and_FOUT_Register);
		return 1;

	break;

	case INSN_CONFIG_BIDIRECTIONAL_DATA:

		if(devpriv->serial_interval_ns == 0) {
			return -EINVAL;
		}

		byte_out = data[1] & 0xFF;

		if(devpriv->serial_hw_mode) {
			err = ni_serial_hw_readwrite8(dev,s,byte_out,&byte_in);
		} else if(devpriv->serial_interval_ns > 0) {
			err = ni_serial_sw_readwrite8(dev,s,byte_out,&byte_in);
		} else {
			printk("ni_serial_insn_config: serial disabled!\n");
			return -EINVAL;
		}
		if(err < 0) return err;
		data[1] = byte_in & 0xFF;
		return insn->n;

	break;
	default:
		return -EINVAL;
	}

}

static int ni_serial_hw_readwrite8(comedi_device *dev,comedi_subdevice *s,
				   unsigned char data_out,
				   unsigned char *data_in)
{
	unsigned int status1;
	int err = 0, count = 20;

#ifdef DEBUG_DIO
	printk("ni_serial_hw_readwrite8: outputting 0x%x\n", data_out);
#endif

	devpriv->dio_output &= ~DIO_Serial_Data_Mask;
	devpriv->dio_output |= DIO_Serial_Data_Out(data_out);
	devpriv->stc_writew(dev, devpriv->dio_output,DIO_Output_Register);

	status1 = devpriv->stc_readw(dev, Joint_Status_1_Register);
	if(status1 & DIO_Serial_IO_In_Progress_St) {
		err = -EBUSY;
		goto Error;
	}

	devpriv->dio_control |= DIO_HW_Serial_Start;
	devpriv->stc_writew(dev, devpriv->dio_control,DIO_Control_Register);
	devpriv->dio_control &= ~DIO_HW_Serial_Start;

	/* Wait until STC says we're done, but don't loop infinitely. */
	while((status1 = devpriv->stc_readw(dev, Joint_Status_1_Register)) & DIO_Serial_IO_In_Progress_St) {
		/* Delay one bit per loop */
		comedi_udelay((devpriv->serial_interval_ns + 999) / 1000);
		if(--count < 0) {
			printk("ni_serial_hw_readwrite8: SPI serial I/O didn't finish in time!\n");
			err = -ETIME;
			goto Error;
		}
	}

	/* Delay for last bit. This delay is absolutely necessary, because
	   DIO_Serial_IO_In_Progress_St goes high one bit too early. */
	comedi_udelay((devpriv->serial_interval_ns + 999) / 1000);

	if(data_in != NULL) {
		*data_in = devpriv->stc_readw(dev, DIO_Serial_Input_Register);
#ifdef DEBUG_DIO
		printk("ni_serial_hw_readwrite8: inputted 0x%x\n", *data_in);
#endif
	}

 Error:
	devpriv->stc_writew(dev, devpriv->dio_control,DIO_Control_Register);

	return err;
}

static int ni_serial_sw_readwrite8(comedi_device *dev,comedi_subdevice *s,
				   unsigned char data_out,
				   unsigned char *data_in)
{
	unsigned char mask, input = 0;

#ifdef DEBUG_DIO
	printk("ni_serial_sw_readwrite8: outputting 0x%x\n", data_out);
#endif

	/* Wait for one bit before transfer */
	comedi_udelay((devpriv->serial_interval_ns + 999) / 1000);

	for(mask = 0x80; mask; mask >>= 1) {
		/* Output current bit; note that we cannot touch s->state
	   because it is a per-subdevice field, and serial is
		   a separate subdevice from DIO. */
		devpriv->dio_output &= ~DIO_SDOUT;
		if(data_out & mask) {
			devpriv->dio_output |= DIO_SDOUT;
		}
		devpriv->stc_writew(dev, devpriv->dio_output,DIO_Output_Register);

		/* Assert SDCLK (active low, inverted), wait for half of
		   the delay, deassert SDCLK, and wait for the other half. */
		devpriv->dio_control |= DIO_Software_Serial_Control;
		devpriv->stc_writew(dev, devpriv->dio_control,DIO_Control_Register);

		comedi_udelay((devpriv->serial_interval_ns + 999) / 2000);

		devpriv->dio_control &= ~DIO_Software_Serial_Control;
		devpriv->stc_writew(dev, devpriv->dio_control,DIO_Control_Register);

		comedi_udelay((devpriv->serial_interval_ns + 999) / 2000);

		/* Input current bit */
		if(devpriv->stc_readw(dev, DIO_Parallel_Input_Register) & DIO_SDIN) {
/*			printk("DIO_P_I_R: 0x%x\n", devpriv->stc_readw(dev, DIO_Parallel_Input_Register)); */
			input |= mask;
		}
	}
#ifdef DEBUG_DIO
	printk("ni_serial_sw_readwrite8: inputted 0x%x\n", input);
#endif
	if(data_in) *data_in = input;

	return 0;
}

static void mio_common_detach(comedi_device *dev)
{
	if(dev->subdevices && boardtype.has_8255)
		subdev_8255_cleanup(dev,dev->subdevices+3);
}

static void init_ao_67xx(comedi_device *dev, comedi_subdevice *s)
{
	int i;

	for(i = 0; i < s->n_chan; i++)
		ni_ao_win_outw(dev, AO_Channel(i) | 0x0, AO_Configuration_2_67xx);
}

static int ni_alloc_private(comedi_device *dev)
{
	int ret;

	ret = alloc_private(dev, sizeof(ni_private));
	if(ret < 0) return ret;

	spin_lock_init(&devpriv->window_lock);

	return 0;
};

static int ni_E_init(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int bits;

	if(boardtype.n_aochan > MAX_N_AO_CHAN)
	{
		printk("bug! boardtype.n_aochan > MAX_N_AO_CHAN\n");
		return -EINVAL;
	}
	
	if(alloc_subdevices(dev, 11) < 0)
		return -ENOMEM;

	/* analog input subdevice */

	s=dev->subdevices+0;
	dev->read_subdev=s;
	if(boardtype.n_adchan){
		s->type=COMEDI_SUBD_AI;
		s->subdev_flags=SDF_READABLE | SDF_DIFF | SDF_DITHER;
		if(boardtype.reg_type != ni_reg_611x)
			s->subdev_flags |= SDF_GROUND | SDF_COMMON | SDF_OTHER;
		if(boardtype.adbits > 16)
			s->subdev_flags |= SDF_LSAMPL;
		s->n_chan=boardtype.n_adchan;
		s->len_chanlist=512;
		s->maxdata=(1<<boardtype.adbits)-1;
		s->range_table=ni_range_lkup[boardtype.gainlkup];
		s->insn_read=ni_ai_insn_read;
		s->insn_config=ni_ai_insn_config;
		s->do_cmdtest=ni_ai_cmdtest;
		s->do_cmd=ni_ai_cmd;
		s->cancel=ni_ai_reset;
		s->poll=ni_ai_poll;
		s->munge=ni_ai_munge;
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}
		
	/* analog output subdevice */

	s=dev->subdevices+1;
	if(boardtype.n_aochan){
		dev->write_subdev=s;
		s->type=COMEDI_SUBD_AO;
		s->subdev_flags=SDF_WRITABLE|SDF_DEGLITCH|SDF_GROUND;
		s->n_chan=boardtype.n_aochan;
		s->maxdata=(1<<boardtype.aobits)-1;
		s->range_table = boardtype.ao_range_table;
		s->insn_read=ni_ao_insn_read;
		if(boardtype.reg_type & ni_reg_6xxx_mask){
			s->insn_write=ni_ao_insn_write_671x;
		}else{
			s->insn_write=ni_ao_insn_write;
		}
#ifdef PCIDMA
		if(boardtype.n_aochan){
#else		
		if(boardtype.ao_fifo_depth){ 
#endif
			s->do_cmd=ni_ao_cmd;
			s->do_cmdtest=ni_ao_cmdtest;
			s->len_chanlist = boardtype.n_aochan;
			if(boardtype.reg_type != ni_reg_m_series)
				s->munge=ni_ao_munge;
		}
		s->cancel=ni_ao_reset;
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}
	if((boardtype.reg_type & ni_reg_67xx_mask))
		init_ao_67xx(dev, s);

	/* digital i/o subdevice */

	s=dev->subdevices+2;
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_WRITABLE|SDF_READABLE;
	s->n_chan=8;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->io_bits=0;		/* all bits input */
	s->insn_bits=ni_dio_insn_bits;
	s->insn_config=ni_dio_insn_config;

	/* dio setup */
	devpriv->dio_control = DIO_Pins_Dir(s->io_bits);
	devpriv->stc_writew(dev, devpriv->dio_control,DIO_Control_Register);
	
	/* 8255 device */
	s=dev->subdevices+3;
	if(boardtype.has_8255){
		subdev_8255_init(dev,s,ni_8255_callback,(unsigned long)dev);
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}

	/* general purpose counter/timer device */
	s=dev->subdevices+4;
	s->type=COMEDI_SUBD_COUNTER;
	s->subdev_flags=SDF_READABLE|SDF_WRITABLE;
	s->insn_read=  ni_gpct_insn_read;
	s->insn_write= ni_gpct_insn_write;
	s->insn_config=ni_gpct_insn_config;
	s->n_chan=2;
	s->maxdata=1;
	devpriv->an_trig_etc_reg = 0;
	GPCT_Reset(dev,0);
	GPCT_Reset(dev,1);

	/* calibration subdevice -- ai and ao */
	s=dev->subdevices+5;
	s->type=COMEDI_SUBD_CALIB;
	if(boardtype.reg_type == ni_reg_m_series)
	{
		// internal PWM analog output used for AI nonlinearity calibration
		s->subdev_flags = SDF_INTERNAL;
		s->insn_config = &ni_m_series_pwm_config;
		s->n_chan = 1;
		s->maxdata = 0;
		ni_writel(0x0, M_Offset_Cal_PWM);
	} else if(boardtype.reg_type == ni_reg_6143)
	{
		// internal PWM analog output used for AI nonlinearity calibration
		s->subdev_flags = SDF_INTERNAL;
		s->insn_config = &ni_6143_pwm_config;
		s->n_chan = 1;
		s->maxdata = 0;
	}else
	{
		s->subdev_flags = SDF_WRITABLE | SDF_INTERNAL;
		s->insn_read = &ni_calib_insn_read;
		s->insn_write = &ni_calib_insn_write;
		caldac_setup(dev, s);
	}
	
	/* EEPROM */
	s=dev->subdevices+6;
	s->type=COMEDI_SUBD_MEMORY;
	s->subdev_flags=SDF_READABLE|SDF_INTERNAL;
	s->maxdata=0xff;
	if(boardtype.reg_type == ni_reg_m_series)
	{
		s->n_chan = M_SERIES_EEPROM_SIZE;
		s->insn_read = &ni_m_series_eeprom_insn_read;
	}else
	{
		s->n_chan = 512;
		s->insn_read = &ni_eeprom_insn_read;
	}
	/* PFI */
	s=dev->subdevices+7;
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_READABLE|SDF_WRITABLE|SDF_INTERNAL;
	s->n_chan=10;
	s->maxdata=1;
	s->insn_bits = ni_pfi_insn_bits;
	s->insn_config = ni_pfi_insn_config;
	ni_set_bits(dev, IO_Bidirection_Pin_Register, ~0, 0);

	/* cs5529 calibration adc */
	s = dev->subdevices + 8;
	if(boardtype.reg_type & ni_reg_67xx_mask)
	{
		s->type = COMEDI_SUBD_AI;
		s->subdev_flags = SDF_READABLE | SDF_DIFF | SDF_INTERNAL;
		// one channel for each analog output channel
		s->n_chan = boardtype.n_aochan;
		s->maxdata = (1 << 16) - 1;
		s->range_table = &range_unknown; /* XXX */
		s->insn_read=cs5529_ai_insn_read;
		s->insn_config=NULL;
		init_cs5529(dev);
	}else
	{
		s->type=COMEDI_SUBD_UNUSED;
	}

	/* Serial */
	s=dev->subdevices+9;
	s->type=COMEDI_SUBD_SERIAL;
	s->subdev_flags=SDF_READABLE|SDF_WRITABLE|SDF_INTERNAL;
	s->n_chan=1;
	s->maxdata=0xff;
	s->insn_config = ni_serial_insn_config;
	devpriv->serial_interval_ns = 0;
	devpriv->serial_hw_mode = 0;

	/* RTSI */
	s=dev->subdevices+10;
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_READABLE|SDF_WRITABLE|SDF_INTERNAL;
	s->n_chan=8;
	s->maxdata=1;
	s->insn_bits = ni_rtsi_insn_bits;
	s->insn_config = ni_rtsi_insn_config;
	ni_rtsi_init(dev);

	/* ai configuration */
	ni_ai_reset(dev,dev->subdevices+0);
	if((boardtype.reg_type & ni_reg_6xxx_mask) == 0){
		// BEAM is this needed for PCI-6143 ??
		devpriv->clock_and_fout =
			Slow_Internal_Time_Divide_By_2 |
			Slow_Internal_Timebase |
			Clock_To_Board_Divide_By_2 |
			Clock_To_Board |
			AI_Output_Divide_By_2 |
			AO_Output_Divide_By_2;
	}else{
		devpriv->clock_and_fout =
			Slow_Internal_Time_Divide_By_2 |
			Slow_Internal_Timebase |
			Clock_To_Board_Divide_By_2 |
			Clock_To_Board;
	}
	devpriv->stc_writew(dev, devpriv->clock_and_fout, Clock_and_FOUT_Register);

	/* analog output configuration */
	ni_ao_reset(dev,dev->subdevices + 1);

	if(dev->irq){
		devpriv->stc_writew(dev, (IRQ_POLARITY?Interrupt_Output_Polarity:0) |
			(Interrupt_Output_On_3_Pins&0) |
			Interrupt_A_Enable |
			Interrupt_B_Enable |
			Interrupt_A_Output_Select(interrupt_pin(dev->irq)) |
			Interrupt_B_Output_Select(interrupt_pin(dev->irq)),
			Interrupt_Control_Register
		);
	}

	/* DMA setup */
	/* tell the STC which dma channels to use for AI and AO */
	bits = 1 << ( AI_DMA_CHAN );
	bits |= 1 << ( AO_DMA_CHAN + 4 );
	ni_writeb( bits, AI_AO_Select);
	/* tell the STC which dma channels to use for
	 * General purpose counters 0 and 1 */
	bits = 1 << ( GPC0_DMA_CHAN );
	bits |= 1 << ( GPC1_DMA_CHAN + 4 );
	ni_writeb( bits, G0_G1_Select);

	if(boardtype.reg_type & ni_reg_6xxx_mask)
	{
		ni_writeb( 0, Magic_611x );
	}else if(boardtype.reg_type == ni_reg_m_series)
	{
		int channel;
		for(channel = 0; channel < boardtype.n_aochan; ++channel)
		{
			ni_writeb(0xf, M_Offset_AO_Waveform_Order(channel));
			ni_writeb(0x0, M_Offset_AO_Reference_Attenuation(channel));
		}
		ni_writeb(0x0, M_Offset_AO_Calibration);
	}

	printk("\n");

	return 0;
}



static int ni_8255_callback(int dir,int port,int data,unsigned long arg)
{
	comedi_device *dev=(comedi_device *)arg;

	if(dir){
		ni_writeb(data,Port_A+2*port);
		return 0;
	}else{
		return ni_readb(Port_A+2*port);
	}
}

/*
	presents the EEPROM as a subdevice
*/

static int ni_eeprom_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	data[0]=ni_read_eeprom(dev,CR_CHAN(insn->chanspec));

	return 1;
}

/*
	reads bytes out of eeprom
*/

static int ni_read_eeprom(comedi_device *dev,int addr)
{
	int bit;
	int bitstring;

	bitstring=0x0300|((addr&0x100)<<3)|(addr&0xff);
	ni_writeb(0x04,Serial_Command);
	for(bit=0x8000;bit;bit>>=1){
		ni_writeb(0x04|((bit&bitstring)?0x02:0),Serial_Command);
		ni_writeb(0x05|((bit&bitstring)?0x02:0),Serial_Command);
	}
	bitstring=0;
	for(bit=0x80;bit;bit>>=1){
		ni_writeb(0x04,Serial_Command);
		ni_writeb(0x05,Serial_Command);
		bitstring|=((ni_readb(XXX_Status)&PROMOUT)?bit:0);
	}
	ni_writeb(0x00,Serial_Command);

	return bitstring;
}

static int ni_m_series_eeprom_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	data[0] = devpriv->eeprom_buffer[CR_CHAN(insn->chanspec)];

	return 1;
}

static int ni_get_pwm_config(comedi_device *dev, lsampl_t *data)
{
	data[1] = devpriv->pwm_up_count * TIMER_BASE;
	data[2] = devpriv->pwm_down_count * TIMER_BASE;
	return 3;
}

static int ni_m_series_pwm_config(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	unsigned up_count, down_count;
	switch(data[0])
	{
	case INSN_CONFIG_PWM_OUTPUT:
		switch(data[1])
		{
		case TRIG_ROUND_NEAREST:
			up_count = (data[2] + TIMER_BASE / 2) / TIMER_BASE; 
			break;
		case TRIG_ROUND_DOWN:
			up_count = data[2] / TIMER_BASE;
			break;
		case TRIG_ROUND_UP:
			up_count = (data[2] + TIMER_BASE - 1) / TIMER_BASE;
			break;
		default:
			return -EINVAL;
			break;
		}
		switch(data[3])
		{
		case TRIG_ROUND_NEAREST:
			down_count = (data[4] + TIMER_BASE / 2) / TIMER_BASE;
			break;
		case TRIG_ROUND_DOWN:
			down_count = data[4] / TIMER_BASE;
			break;
		case TRIG_ROUND_UP:
			down_count = (data[4] + TIMER_BASE - 1) / TIMER_BASE;
			break;
		default:
			return -EINVAL;
			break;
		}
		if(up_count * TIMER_BASE != data[2] ||
			down_count * TIMER_BASE != data[4])
		{
			data[2] = up_count * TIMER_BASE;
			data[4] = down_count * TIMER_BASE;
			return -EAGAIN;
		}
		ni_writel(MSeries_Cal_PWM_High_Time_Bits(up_count) | MSeries_Cal_PWM_Low_Time_Bits(down_count), M_Offset_Cal_PWM);
		devpriv->pwm_up_count = up_count;
		devpriv->pwm_down_count = down_count;
		return 5;
		break;
	case INSN_CONFIG_GET_PWM_OUTPUT:
		return ni_get_pwm_config(dev, data);
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

static int ni_6143_pwm_config(comedi_device *dev, comedi_subdevice *s,
	comedi_insn *insn, lsampl_t *data)
{
	unsigned up_count, down_count;
	switch(data[0])
	{
	case INSN_CONFIG_PWM_OUTPUT:
		switch(data[1])
		{
		case TRIG_ROUND_NEAREST:
			up_count = (data[2] + TIMER_BASE / 2) / TIMER_BASE; 
			break;
		case TRIG_ROUND_DOWN:
			up_count = data[2] / TIMER_BASE;
			break;
		case TRIG_ROUND_UP:
			up_count = (data[2] + TIMER_BASE - 1) / TIMER_BASE;
			break;
		default:
			return -EINVAL;
			break;
		}
		switch(data[3])
		{
		case TRIG_ROUND_NEAREST:
			down_count = (data[4] + TIMER_BASE / 2) / TIMER_BASE;
			break;
		case TRIG_ROUND_DOWN:
			down_count = data[4] / TIMER_BASE;
			break;
		case TRIG_ROUND_UP:
			down_count = (data[4] + TIMER_BASE - 1) / TIMER_BASE;
			break;
		default:
			return -EINVAL;
			break;
		}
		if(up_count * TIMER_BASE != data[2] ||
			down_count * TIMER_BASE != data[4])
		{
			data[2] = up_count * TIMER_BASE;
			data[4] = down_count * TIMER_BASE;
			return -EAGAIN;
		}
		ni_writel(up_count, Calibration_HighTime_6143);
		devpriv->pwm_up_count = up_count;
		ni_writel(down_count, Calibration_LowTime_6143);
		devpriv->pwm_down_count = down_count;
		return 5;
		break;
	case INSN_CONFIG_GET_PWM_OUTPUT:
		return ni_get_pwm_config(dev, data);
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

static void ni_write_caldac(comedi_device *dev,int addr,int val);
/*
	calibration subdevice
*/
static int ni_calib_insn_write(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	ni_write_caldac(dev,CR_CHAN(insn->chanspec),data[0]);

	return 1;
}

static int ni_calib_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	data[0] = devpriv->caldacs[CR_CHAN(insn->chanspec)];

	return 1;
}

static int pack_mb88341(int addr,int val,int *bitstring);
static int pack_dac8800(int addr,int val,int *bitstring);
static int pack_dac8043(int addr,int val,int *bitstring);
static int pack_ad8522(int addr,int val,int *bitstring);
static int pack_ad8804(int addr,int val,int *bitstring);
static int pack_ad8842(int addr,int val,int *bitstring);

struct caldac_struct{
	int n_chans;
	int n_bits;
	int (*packbits)(int,int,int *);
};

static struct caldac_struct caldacs[] = {
	[mb88341] = { 12, 8, pack_mb88341 },
	[dac8800] = { 8, 8, pack_dac8800 },
	[dac8043] = { 1, 12, pack_dac8043 },
	[ad8522]  = { 2, 12, pack_ad8522 },
	[ad8804] = { 12, 8, pack_ad8804 },
	[ad8842] = { 8, 8, pack_ad8842 },
	[ad8804_debug] = { 16, 8, pack_ad8804 },
};

static void caldac_setup(comedi_device *dev,comedi_subdevice *s)
{
	int i,j;
	int n_dacs;
	int n_chans=0;
	int n_bits;
	int diffbits=0;
	int type;
	int chan;

	type = boardtype.caldac[0];
	if(type==caldac_none)return;
	n_bits=caldacs[type].n_bits;
	for(i=0;i<3;i++){
		type = boardtype.caldac[i];
		if(type==caldac_none)break;
		if(caldacs[type].n_bits!=n_bits)diffbits=1;
		n_chans+=caldacs[type].n_chans;
	}
	n_dacs=i;
	s->n_chan=n_chans;

	if(diffbits){

		if(n_chans>MAX_N_CALDACS){
			printk("BUG! MAX_N_CALDACS too small\n");
		}
		s->maxdata_list=devpriv->caldac_maxdata_list;
		chan=0;
		for(i=0;i<n_dacs;i++){
			type = boardtype.caldac[i];
			for(j=0;j<caldacs[type].n_chans;j++){
				s->maxdata_list[chan]=
					(1<<caldacs[type].n_bits)-1;
				chan++;
			}
		}

		for( chan = 0; chan < s->n_chan; chan++ )
			ni_write_caldac( dev, i, s->maxdata_list[ i ] / 2 );
	}else{
		type = boardtype.caldac[0];
		s->maxdata=(1<<caldacs[type].n_bits)-1;

		for( chan = 0; chan < s->n_chan; chan++ )
			ni_write_caldac( dev, i, s->maxdata / 2 );
	}
}

static void ni_write_caldac(comedi_device *dev,int addr,int val)
{
	unsigned int loadbit=0,bits=0,bit,bitstring=0;
	int i;
	int type;

	//printk("ni_write_caldac: chan=%d val=%d\n",addr,val);
	if( devpriv->caldacs[ addr ] == val ) return;
	devpriv->caldacs[ addr ] = val;

	for(i=0;i<3;i++){
		type = boardtype.caldac[i];
		if(type==caldac_none)break;
		if(addr<caldacs[type].n_chans){
			bits=caldacs[type].packbits(addr,val,&bitstring);
			loadbit=SerDacLd(i);
			//printk("caldac: using i=%d addr=%d %x\n",i,addr,bitstring);
			break;
		}
		addr-=caldacs[type].n_chans;
	}

	for(bit=1<<(bits-1);bit;bit>>=1){
		ni_writeb(((bit&bitstring)?0x02:0),Serial_Command);
		comedi_udelay(1);
		ni_writeb(1|((bit&bitstring)?0x02:0),Serial_Command);
		comedi_udelay(1);
	}
	ni_writeb(loadbit,Serial_Command);
	comedi_udelay(1);
	ni_writeb(0,Serial_Command);
}



static int pack_mb88341(int addr,int val,int *bitstring)
{
	/*
	   Fujitsu MB 88341
	   Note that address bits are reversed.  Thanks to
	   Ingo Keen for noticing this.

	   Note also that the 88341 expects address values from
	   1-12, whereas we use channel numbers 0-11.  The NI
	   docs use 1-12, also, so be careful here.
	*/
	addr++;
	*bitstring=((addr&0x1)<<11) |
		  ((addr&0x2)<<9)  |
		  ((addr&0x4)<<7)  |
		  ((addr&0x8)<<5)  |
		  (val&0xff);
	return 12;
}

static int pack_dac8800(int addr,int val,int *bitstring)
{
	*bitstring=((addr&0x7)<<8)|(val&0xff);
	return 11;
}

static int pack_dac8043(int addr,int val,int *bitstring)
{
	*bitstring=val&0xfff;
	return 12;
}
	
static int pack_ad8522(int addr,int val,int *bitstring)
{
	*bitstring=(val&0xfff)|(addr ? 0xc000:0xa000);
	return 16;
}

static int pack_ad8804(int addr,int val,int *bitstring)
{
	*bitstring=((addr&0xf)<<8) | (val&0xff);
	return 12;
}

static int pack_ad8842(int addr,int val,int *bitstring)
{
	*bitstring=((addr+1)<<8) | (val&0xff);
	return 12;
}





/*
 *
 *  General Purpose Counter/Timer section
 *
 */

/*
 * Low level stuff...Each STC counter has two 24 bit load registers
 * (A&B).  Just make it easier to access them.
 *
 * These are inlined _only_ because they are used once in subsequent
 * code.  Otherwise they should not be inlined.
 */
static inline void GPCT_Load_A(comedi_device *dev, int chan, unsigned int value)
{
	devpriv->stc_writel(dev,  value & 0x00ffffff, G_Load_A_Register(chan));
}

static inline void GPCT_Load_B(comedi_device *dev, int chan, unsigned int value)
{
	devpriv->stc_writel(dev,  value & 0x00ffffff, G_Load_B_Register(chan));
}

/*  Load a value into the counter, using register A as the intermediate step.
*  You might use GPCT_Load_Using_A to load a 0x000000 into a counter
*  reset its value.
*/
static void GPCT_Load_Using_A(comedi_device *dev, int chan, unsigned int value)
{
	devpriv->gpct_mode[chan] &= (~G_Load_Source_Select);
	devpriv->stc_writew(dev,  devpriv->gpct_mode[chan],G_Mode_Register(chan));
	GPCT_Load_A(dev,chan,value);
	devpriv->stc_writew(dev,  devpriv->gpct_command[chan]|G_Load,G_Command_Register(chan));
}

/*
 *	Read the GPCTs current value.  
 */
static int GPCT_G_Watch(comedi_device *dev, int chan)
{
	unsigned int hi1,hi2,lo;
	
	devpriv->gpct_command[chan] &= ~G_Save_Trace;
	devpriv->stc_writew(dev,  devpriv->gpct_command[chan],G_Command_Register(chan));
	
	devpriv->gpct_command[chan] |= G_Save_Trace;
	devpriv->stc_writew(dev,  devpriv->gpct_command[chan], G_Command_Register(chan));

	/* This procedure is used because the two registers cannot
	 * be read atomically. */
	do{
		hi1 = devpriv->stc_readw(dev,  G_Save_Register_High(chan));
		lo = devpriv->stc_readw(dev, G_Save_Register_Low(chan));
		hi2 = devpriv->stc_readw(dev,  G_Save_Register_High(chan));
	}while(hi1!=hi2);

	return (hi1<<16)|lo;
}


static int GPCT_Disarm(comedi_device *dev, int chan)
{
	devpriv->stc_writew(dev,  devpriv->gpct_command[chan] | G_Disarm,G_Command_Register(chan));
	return 0;
}


static int GPCT_Arm(comedi_device *dev, int chan)
{
	devpriv->stc_writew(dev,  devpriv->gpct_command[chan] | G_Arm,G_Command_Register(chan));
	/* If the counter is doing pulse width measurement, then make
	 sure that the counter did not start counting right away.  This would
	 indicate that we started acquiring the pulse after it had already 
	 started and our measurement would be inaccurate */
	if(devpriv->gpct_cur_operation[chan] == GPCT_SINGLE_PW){
		int g_status; 

		g_status=devpriv->stc_readw(dev, G_Status_Register);
		
		if(chan == 0){
			//TIM 5/2/01 possible error with very short pulses
			if((G0_Counting_St & g_status)|| !(G0_Armed_St&g_status)) {
				//error: we missed the beginning of the pulse
				return -EINVAL; //there is probably a more accurate error code...
			}
		}else{
			if((G1_Counting_St & g_status)|| !(G1_Armed_St&g_status)) {
				//error: we missed the beginning of the pulse
				return -EINVAL;
			}
		}
	}
	return 0;
}

static int GPCT_Set_Source(comedi_device *dev,int chan ,int source)
{
	//printk("GPCT_Set_Source...");
	devpriv->gpct_input_select[chan] &= ~G_Source_Select(0x1f);//reset gate to 0
	switch(source) {
		case GPCT_INT_CLOCK:
		devpriv->gpct_input_select[chan] |= G_Source_Select(0);//INT_TIMEBASE
		break;
	case GPCT_EXT_PIN:
		if(chan==0)
			devpriv->gpct_input_select[chan] |= G_Source_Select(9);//PFI8
		else
			devpriv->gpct_input_select[chan] |= G_Source_Select(4);//PFI3
		break;
	default:
		return -EINVAL;
	}
	devpriv->stc_writew(dev, devpriv->gpct_input_select[chan], G_Input_Select_Register(chan));
	//printk("exit GPCT_Set_Source\n");
	return 0;
}

static int GPCT_Set_Gate(comedi_device *dev,int chan ,int gate)
{
	//printk("GPCT_Set_Gate...");
	devpriv->gpct_input_select[chan] &= ~G_Gate_Select(0x1f);//reset gate to 0
	switch(gate) {
	case GPCT_NO_GATE:
		devpriv->gpct_input_select[chan] |= G_Gate_Select(31);//Low
		devpriv->gpct_mode[chan] |= G_Gate_Polarity;
		break;
	case GPCT_EXT_PIN:
		devpriv->gpct_mode[chan] &= ~G_Gate_Polarity;
		if(chan==0){
			devpriv->gpct_input_select[chan] |= G_Gate_Select(10);//PFI9
		}else{
			devpriv->gpct_input_select[chan] |= G_Gate_Select(5);//PFI4
		}
		break;
	default:
		return -EINVAL;
	}
	devpriv->stc_writew(dev, devpriv->gpct_input_select[chan], G_Input_Select_Register(chan));
	devpriv->stc_writew(dev, devpriv->gpct_mode[chan], G_Mode_Register(chan));
	//printk("exit GPCT_Set_Gate\n");
	return 0;
}

static int GPCT_Set_Direction(comedi_device *dev,int chan,int direction)
{
	//printk("GPCT_Set_Direction...");
	
	devpriv->gpct_command[chan] &= ~G_Up_Down(0x3);
	switch (direction) {
		case GPCT_UP:
			devpriv->gpct_command[chan] |= G_Up_Down(1);
			break;
		case GPCT_DOWN:
			devpriv->gpct_command[chan] |= G_Up_Down(0);
			break;
		case GPCT_HWUD:
			devpriv->gpct_command[chan] |= G_Up_Down(2);
			break;
		default:
			printk("Error direction=0x%08x..",direction);
			return -EINVAL;
	}
	devpriv->stc_writew(dev, devpriv->gpct_command[chan], G_Command_Register(chan));
	//TIM 4/23/01 devpriv->stc_writew(dev, devpriv->gpct_mode[chan], G_Mode_Register(chan));
	//printk("exit GPCT_Set_Direction\n");
	return 0;
}

static void GPCT_Event_Counting(comedi_device *dev,int chan)
{

	//NOTE: possible residual bits from multibit masks can corrupt
	//If you config for several measurements between Resets, watch out!
	
	//printk("GPCT_Event_Counting...");
	
	devpriv->gpct_cur_operation[chan] = GPCT_SIMPLE_EVENT;
	
	// Gating_Mode = 1
	devpriv->gpct_mode[chan] &= ~(G_Gating_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Gating_Mode(1);
	
	// Trigger_Mode_For_Edge_Gate = 1
	devpriv->gpct_mode[chan] &= ~(G_Trigger_Mode_For_Edge_Gate(0x3));
	devpriv->gpct_mode[chan] |= G_Trigger_Mode_For_Edge_Gate(2);

	devpriv->stc_writew(dev,  devpriv->gpct_mode[chan],G_Mode_Register(chan));
	//printk("exit GPCT_Event_Counting\n");
}

static void GPCT_Period_Meas(comedi_device *dev, int chan)
{
	//printk("GPCT_Period_Meas...");
	
	devpriv->gpct_cur_operation[chan] = GPCT_SINGLE_PERIOD;

	
	//NOTE: possible residual bits from multibit masks can corrupt
	//If you config for several measurements between Resets, watch out!	
	devpriv->gpct_mode[chan] &= ~G_OR_Gate;
	devpriv->gpct_mode[chan] &= ~G_Gate_Select_Load_Source;
	
	// Output_Mode = 3 
	devpriv->gpct_mode[chan] &= ~(G_Output_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Output_Mode(3);
	
	
	//Gating Mode=2
	devpriv->gpct_mode[chan] &= ~(G_Gating_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Gating_Mode(2);
	
	// Trigger_Mode_For_Edge_Gate=0
	devpriv->gpct_mode[chan] &= ~(G_Trigger_Mode_For_Edge_Gate(0x3));
	devpriv->gpct_mode[chan] |= G_Trigger_Mode_For_Edge_Gate(0);

	devpriv->gpct_mode[chan] |= G_Reload_Source_Switching;
	devpriv->gpct_mode[chan] &= ~G_Loading_On_Gate;
	devpriv->gpct_mode[chan] &= ~G_Loading_On_TC;
	devpriv->gpct_mode[chan] &= ~G_Gate_On_Both_Edges;

	// Stop_Mode = 2
	devpriv->gpct_mode[chan] &= ~(G_Stop_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Stop_Mode(0);
	
	// Counting_Once = 2 
	devpriv->gpct_mode[chan] &= ~(G_Counting_Once(0x3));
	devpriv->gpct_mode[chan] |= G_Counting_Once(2);

	// Up_Down = 1 
	devpriv->gpct_command[chan] &= ~(G_Up_Down(0x3));
	devpriv->gpct_command[chan] |= G_Up_Down(1);

	devpriv->stc_writew(dev,  devpriv->gpct_mode[chan],G_Mode_Register(chan));
	devpriv->stc_writew(dev,  devpriv->gpct_command[chan],G_Command_Register(chan));
	//printk("exit GPCT_Period_Meas\n");
}

static void GPCT_Pulse_Width_Meas(comedi_device *dev, int chan)
{
	//printk("GPCT_Pulse_Width_Meas...");

	devpriv->gpct_cur_operation[chan] = GPCT_SINGLE_PW;

	devpriv->gpct_mode[chan] &= ~G_OR_Gate;
	devpriv->gpct_mode[chan] &= ~G_Gate_Select_Load_Source;

	// Output_Mode = 3 
	devpriv->gpct_mode[chan] &= ~(G_Output_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Output_Mode(3);
	
	//Gating Mode=1
	devpriv->gpct_mode[chan] &= ~(G_Gating_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Gating_Mode(1);//TIM 4/24/01 was 2
	
	// Trigger_Mode_For_Edge_Gate=2
	devpriv->gpct_mode[chan] &= ~(G_Trigger_Mode_For_Edge_Gate(0x3));
	devpriv->gpct_mode[chan] |= G_Trigger_Mode_For_Edge_Gate(2);//TIM 4/24/01 was 0


	devpriv->gpct_mode[chan] |= G_Reload_Source_Switching;//TIM 4/24/01 was 1
	devpriv->gpct_mode[chan] &= ~G_Loading_On_Gate;//TIM 4/24/01 was 0

	devpriv->gpct_mode[chan] &= ~G_Loading_On_TC;
	devpriv->gpct_mode[chan] &= ~G_Gate_On_Both_Edges;

	// Stop_Mode = 0
	devpriv->gpct_mode[chan] &= ~(G_Stop_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Stop_Mode(0);
	
	// Counting_Once = 2 
	devpriv->gpct_mode[chan] &= ~(G_Counting_Once(0x3));
	devpriv->gpct_mode[chan] |= G_Counting_Once(2);

	// Up_Down = 1 
	devpriv->gpct_command[chan] &= ~(G_Up_Down(0x3));
	devpriv->gpct_command[chan] |= G_Up_Down(1);

	devpriv->stc_writew(dev,  devpriv->gpct_mode[chan],G_Mode_Register(chan));
	devpriv->stc_writew(dev,  devpriv->gpct_command[chan],G_Command_Register(chan));

	//printk("exit GPCT_Pulse_Width_Meas\n");
}

/* GPCT_Gen_Single_Pulse() creates pulse of length pulsewidth which starts after the Arm
signal is sent.  The pulse is delayed by the value already in the counter.  This function could
be modified to send a pulse in response to a trigger event at its gate.*/
static void GPCT_Gen_Single_Pulse(comedi_device *dev, int chan, unsigned int length)
{
	//printk("GPCT_Gen_Cont...");

	devpriv->gpct_cur_operation[chan] = GPCT_SINGLE_PULSE_OUT;

	// Set length of the pulse
	GPCT_Load_B(dev,chan, length-1);

	//Load next time using B, This is reset by GPCT_Load_Using_A()
	devpriv->gpct_mode[chan] |= G_Load_Source_Select;
	
	devpriv->gpct_mode[chan] &= ~G_OR_Gate;
	devpriv->gpct_mode[chan] &= ~G_Gate_Select_Load_Source;

	// Output_Mode = 3 
	devpriv->gpct_mode[chan] &= ~(G_Output_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Output_Mode(2); //TIM 4/26/01 was 3
	
	//Gating Mode=0 for untriggered single pulse
	devpriv->gpct_mode[chan] &= ~(G_Gating_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Gating_Mode(0); //TIM 4/25/01 was 1
	
	// Trigger_Mode_For_Edge_Gate=0
	devpriv->gpct_mode[chan] &= ~(G_Trigger_Mode_For_Edge_Gate(0x3));
	devpriv->gpct_mode[chan] |= G_Trigger_Mode_For_Edge_Gate(2);


	devpriv->gpct_mode[chan] |= G_Reload_Source_Switching;
	devpriv->gpct_mode[chan] &= ~G_Loading_On_Gate;
	devpriv->gpct_mode[chan] |= G_Loading_On_TC; //TIM 4/25/01
	devpriv->gpct_mode[chan] &= ~G_Gate_On_Both_Edges;

	// Stop_Mode = 2
	devpriv->gpct_mode[chan] &= ~(G_Stop_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Stop_Mode(2); //TIM 4/25/01
	
	// Counting_Once = 2 
	devpriv->gpct_mode[chan] &= ~(G_Counting_Once(0x3));
	devpriv->gpct_mode[chan] |= G_Counting_Once(1); //TIM 4/25/01

	// Up_Down = 1 
	devpriv->gpct_command[chan] &= ~(G_Up_Down(0x3));
	devpriv->gpct_command[chan] |= G_Up_Down(0); //TIM 4/25/01 was 1

	devpriv->stc_writew(dev,  devpriv->gpct_mode[chan],G_Mode_Register(chan));
	devpriv->stc_writew(dev,  devpriv->gpct_command[chan],G_Command_Register(chan));

	//printk("exit GPCT_Gen_Cont\n");
}

static void GPCT_Gen_Cont_Pulse(comedi_device *dev, int chan, unsigned int length)
{
	//printk("GPCT_Gen_Cont...");

	devpriv->gpct_cur_operation[chan] = GPCT_CONT_PULSE_OUT;

	// Set length of the pulse
	GPCT_Load_B(dev,chan, length-1);

	//Load next time using B, This is reset by GPCT_Load_Using_A()
	devpriv->gpct_mode[chan] |= G_Load_Source_Select;
	
	devpriv->gpct_mode[chan] &= ~G_OR_Gate;
	devpriv->gpct_mode[chan] &= ~G_Gate_Select_Load_Source;

	// Output_Mode = 3 
	devpriv->gpct_mode[chan] &= ~(G_Output_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Output_Mode(2); //TIM 4/26/01 was 3
	
	//Gating Mode=0 for untriggered single pulse
	devpriv->gpct_mode[chan] &= ~(G_Gating_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Gating_Mode(0); //TIM 4/26/01 was 0
	
	// Trigger_Mode_For_Edge_Gate=0
	devpriv->gpct_mode[chan] &= ~(G_Trigger_Mode_For_Edge_Gate(0x3));
	devpriv->gpct_mode[chan] |= G_Trigger_Mode_For_Edge_Gate(2);


	devpriv->gpct_mode[chan] |= G_Reload_Source_Switching;
	devpriv->gpct_mode[chan] &= ~G_Loading_On_Gate;
	devpriv->gpct_mode[chan] |= G_Loading_On_TC; 
	devpriv->gpct_mode[chan] &= ~G_Gate_On_Both_Edges;

	// Stop_Mode = 2
	devpriv->gpct_mode[chan] &= ~(G_Stop_Mode(0x3));
	devpriv->gpct_mode[chan] |= G_Stop_Mode(0); //TIM 4/26/01
	
	// Counting_Once = 2 
	devpriv->gpct_mode[chan] &= ~(G_Counting_Once(0x3));
	devpriv->gpct_mode[chan] |= G_Counting_Once(0); //TIM 4/26/01

	// Up_Down = 1 
	devpriv->gpct_command[chan] &= ~(G_Up_Down(0x3));
	devpriv->gpct_command[chan] |= G_Up_Down(0); 

	//TIM 4/26/01
	//This seems pretty unsafe since I don't think it is cleared anywhere.
	//I don't think this is working
	//devpriv->gpct_command[chan] &= ~G_Bank_Switch_Enable;
	//devpriv->gpct_command[chan] &= ~G_Bank_Switch_Mode;
	

	devpriv->stc_writew(dev,  devpriv->gpct_mode[chan],G_Mode_Register(chan));
	devpriv->stc_writew(dev,  devpriv->gpct_command[chan],G_Command_Register(chan));

	//printk("exit GPCT_Gen_Cont\n");
}

static void GPCT_Reset(comedi_device *dev, int chan)
{
	int temp_ack_reg=0;
	
	//printk("GPCT_Reset...");
	devpriv->gpct_cur_operation[chan] = GPCT_RESET;

	switch (chan) {
		case 0:
			devpriv->stc_writew(dev, G0_Reset,Joint_Reset_Register);
			ni_set_bits(dev,Interrupt_A_Enable_Register,G0_TC_Interrupt_Enable,  0);
			ni_set_bits(dev,Interrupt_A_Enable_Register,G0_Gate_Interrupt_Enable,0);
			temp_ack_reg |= G0_Gate_Error_Confirm;
			temp_ack_reg |= G0_TC_Error_Confirm;
			temp_ack_reg |= G0_TC_Interrupt_Ack;
			temp_ack_reg |= G0_Gate_Interrupt_Ack;
			devpriv->stc_writew(dev, temp_ack_reg,Interrupt_A_Ack_Register);
		
			//problem...this interferes with the other ctr...
			devpriv->an_trig_etc_reg |= GPFO_0_Output_Enable;
			devpriv->stc_writew(dev, devpriv->an_trig_etc_reg, Analog_Trigger_Etc_Register);
			break;
		case 1:
			devpriv->stc_writew(dev, G1_Reset,Joint_Reset_Register);
			ni_set_bits(dev,Interrupt_B_Enable_Register,G1_TC_Interrupt_Enable,  0);
			ni_set_bits(dev,Interrupt_B_Enable_Register,G0_Gate_Interrupt_Enable,0);
			temp_ack_reg |= G1_Gate_Error_Confirm;
			temp_ack_reg |= G1_TC_Error_Confirm;
			temp_ack_reg |= G1_TC_Interrupt_Ack;
			temp_ack_reg |= G1_Gate_Interrupt_Ack;
			devpriv->stc_writew(dev, temp_ack_reg,Interrupt_B_Ack_Register);
		
			devpriv->an_trig_etc_reg |= GPFO_1_Output_Enable;
			devpriv->stc_writew(dev, devpriv->an_trig_etc_reg, Analog_Trigger_Etc_Register);
			break;
	};
	
	devpriv->gpct_mode[chan] = 0;
	devpriv->gpct_input_select[chan] = 0;
	devpriv->gpct_command[chan] = 0;
	
	devpriv->gpct_command[chan] |= G_Synchronized_Gate;
	
	devpriv->stc_writew(dev,  devpriv->gpct_mode[chan],G_Mode_Register(chan));
	devpriv->stc_writew(dev,  devpriv->gpct_input_select[chan],G_Input_Select_Register(chan));
	devpriv->stc_writew(dev,  0,G_Autoincrement_Register(chan));
		
	//printk("exit GPCT_Reset\n");
}

static int ni_gpct_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	int retval=0;
	//printk("data[0] is 0x%08x, data[1] is 0x%08x\n",data[0],data[1]);
	switch(data[0]){
	case GPCT_RESET:
		if(insn->n!=1)return -EINVAL;
		GPCT_Reset(dev,insn->chanspec);
		break;
	case GPCT_SET_SOURCE:
		if(insn->n!=2)return -EINVAL;
		retval=GPCT_Set_Source(dev,insn->chanspec,data[1]);
		break;
	case GPCT_SET_GATE:
		if(insn->n!=2)return -EINVAL;
		retval=GPCT_Set_Gate(dev,insn->chanspec,data[1]);
		break;
	case GPCT_SET_DIRECTION:
		if(insn->n!=2) return -EINVAL;
		retval=GPCT_Set_Direction(dev,insn->chanspec,data[1]);
		break;
	case GPCT_GET_INT_CLK_FRQ:
		if(insn->n!=2) return -EINVAL;
		//There are actually 2 internal clocks on the STC, we always
		//use the fast 20MHz one at this time.  Tim  Ousley 5/1/01
		//NOTE: This is not the final interface, ideally the user
		//will never need to know the int. clk. freq.
		data[1]=50;//50ns = 20MHz = internal timebase of STC
		break;
	case GPCT_SET_OPERATION:
		//TIM 5/1/01 if((insn->n<2)||(insn->n>3))return -EINVAL;
		switch(data[1]){
			case GPCT_SIMPLE_EVENT:
				GPCT_Event_Counting(dev,insn->chanspec);
				break;
			case GPCT_SINGLE_PERIOD:
				GPCT_Period_Meas(dev,insn->chanspec);
				break;
			case GPCT_SINGLE_PW:
				GPCT_Pulse_Width_Meas(dev,insn->chanspec);
				break;
			case GPCT_SINGLE_PULSE_OUT:
				GPCT_Gen_Single_Pulse(dev,insn->chanspec,data[2]);
				break;
			case GPCT_CONT_PULSE_OUT:
				GPCT_Gen_Cont_Pulse(dev,insn->chanspec,data[2]);
				break;
			default:
				printk("unsupported GPCT operation!\n");
				return -EINVAL;
		}
		break;
	case GPCT_ARM:
		if(insn->n!=1)return -EINVAL;
		retval=GPCT_Arm(dev,insn->chanspec);
		break;
	case GPCT_DISARM:
		if(insn->n!=1)return -EINVAL;
		retval=GPCT_Disarm(dev,insn->chanspec);
		break;
	default:
		return -EINVAL;
	}

	//catch any errors from return values
	if(retval==0){
		return insn->n;
	}else{
		if(data[0]!=GPCT_ARM){ 
			printk("error: retval was %d\n",retval);
			printk("data[0] is 0x%08x, data[1] is 0x%08x\n",data[0],data[1]);
		}

		return retval;
	}
}

static int ni_gpct_insn_read(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data) {

	int chan=insn->chanspec;
	int cur_op = devpriv->gpct_cur_operation[chan];

	//printk("in ni_gpct_insn_read, n=%d, data[0]=%d\n",insn->chanspec,data[0]);
	if(insn->n!=1)return -EINVAL;
		
	data[0] = GPCT_G_Watch(dev,insn->chanspec);
		
	/* for certain modes (period and pulse width measurment), the value
	in the counter is not valid until the counter stops.  If the value is 
	invalid, return a 0 */
	if((cur_op == GPCT_SINGLE_PERIOD) || (cur_op == GPCT_SINGLE_PW)){
		/* is the counter still running? */
		if(devpriv->stc_readw(dev, G_Status_Register) & (chan?G1_Counting_St:G0_Counting_St))
			data[0]=0;
	}
	return 1;
}

static int ni_gpct_insn_write(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data) {

	//printk("in ni_gpct_insn_write");
	if(insn->n!=1)return -EINVAL;
	GPCT_Load_Using_A(dev,insn->chanspec,data[0]);
	return 1;
}


/*
 *
 *  Programmable Function Inputs
 *
 */

static int ni_pfi_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n!=2)return -EINVAL;

	data[1] = 0;

	return 2;
}

static int ni_pfi_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	unsigned int chan;

	if(insn->n < 1)return -EINVAL;

	chan = CR_CHAN(insn->chanspec);
	if(chan>10)return -EINVAL;

	switch(data[0]){
	case COMEDI_OUTPUT:
		ni_set_bits(dev, IO_Bidirection_Pin_Register, 1<<chan, 1);
		break;
	case COMEDI_INPUT:
		ni_set_bits(dev, IO_Bidirection_Pin_Register, 1<<chan, 0);
		break;
	case INSN_CONFIG_DIO_QUERY:
		data[1] = (devpriv->io_bidirection_pin_reg & (1<<chan)) ? COMEDI_OUTPUT : COMEDI_INPUT;
		return insn->n;
		break;
	default:
		return -EINVAL;
	}

	return 1;
}

/*
 *
 *  NI RTSI Bus Functions
 *
 */
static void ni_rtsi_init(comedi_device *dev)
{
	// Initialises the RTSI bus signal switch to a default state
	
	// Set clock mode to internal
	devpriv->stc_writew(dev, COMEDI_RTSI_CLOCK_MODE_INTERNAL, RTSI_Trig_Direction_Register);

	// Standard internal lines are routed to standard RTSI bus lines
	devpriv->stc_writew(dev, 0x3210, RTSI_Trig_A_Output_Register);
	devpriv->stc_writew(dev, 0x0654, RTSI_Trig_B_Output_Register);
	
	// Sets the source and direction of the 4 on board lines
//	devpriv->stc_writew(dev, 0x0000, RTSI_Board_Register);
}

static int ni_rtsi_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(insn->n != 2) return -EINVAL;

	data[1] = 0;

	return 2;
}

static int ni_rtsi_insn_config(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	unsigned int chan;
	unsigned int bit;

	if(insn->n < 1) return -EINVAL;

	if(data[0] == INSN_CONFIG_SET_RTSI_CLOCK_MODE){
		if(data[1] > 3)
			return -EINVAL;

		devpriv->rtsi_trig_direction_reg &= ~0x03;
		devpriv->rtsi_trig_direction_reg |= data[1];
		devpriv->stc_writew(dev, devpriv->rtsi_trig_direction_reg, RTSI_Trig_Direction_Register);
	}
	else {
		chan = CR_CHAN(insn->chanspec);
		if(chan > 6) return -EINVAL;
		
		bit = 9 + chan;

		switch(data[0]){
		case INSN_CONFIG_DIO_OUTPUT:
			devpriv->rtsi_trig_direction_reg |= (1 << bit);
			devpriv->stc_writew(dev, devpriv->rtsi_trig_direction_reg, RTSI_Trig_Direction_Register);
			break;
		case INSN_CONFIG_DIO_INPUT:
			devpriv->rtsi_trig_direction_reg &= ~(1 << bit);
			devpriv->stc_writew(dev, devpriv->rtsi_trig_direction_reg, RTSI_Trig_Direction_Register);
			break;
		case INSN_CONFIG_DIO_QUERY:
			data[1] = (devpriv->rtsi_trig_direction_reg & (1<<bit)) ? INSN_CONFIG_DIO_OUTPUT : INSN_CONFIG_DIO_INPUT;
			return 2;
			break;
		default:
			return -EINVAL;
		}
	}

	return 1;
}

static int cs5529_wait_for_idle(comedi_device *dev)
{
	unsigned short status;
	const int timeout = HZ;
	int i;

	for(i = 0; i < timeout; i++)
	{
		status = ni_ao_win_inw(dev, CAL_ADC_Status_67xx);
		if((status & CSS_ADC_BUSY) == 0)
		{
			break;
		}
		set_current_state(TASK_INTERRUPTIBLE);
		if(schedule_timeout(1))
		{
			return -EIO;
		}
	}
//printk("looped %i times waiting for idle\n", i);
	if(i == timeout)
	{
		rt_printk("%s: %s: timeout\n", __FILE__, __FUNCTION__);
		return -ETIME;
	}
	return 0;
}

static void cs5529_command(comedi_device *dev, unsigned short value)
{
	static const int timeout = 100;
	int i;

	ni_ao_win_outw(dev, value, CAL_ADC_Command_67xx);
	/* give time for command to start being serially clocked into cs5529.
	 * this insures that the CSS_ADC_BUSY bit will get properly
	 * set before we exit this function.
	*/
	for(i = 0; i < timeout; i++)
	{
		if((ni_ao_win_inw(dev, CAL_ADC_Status_67xx) & CSS_ADC_BUSY))
			break;
		comedi_udelay(1);
	}
//printk("looped %i times writing command to cs5529\n", i);
	if(i == timeout)
	{
		comedi_error(dev, "possible problem - never saw adc go busy?");
	}
}

/* write to cs5529 register */
static void cs5529_config_write(comedi_device *dev, unsigned int value, unsigned int reg_select_bits)
{
	ni_ao_win_outw(dev, ((value >> 16) & 0xff), CAL_ADC_Config_Data_High_Word_67xx);
	ni_ao_win_outw(dev, (value & 0xffff), CAL_ADC_Config_Data_Low_Word_67xx);
	reg_select_bits &= CSCMD_REGISTER_SELECT_MASK;
	cs5529_command(dev, CSCMD_COMMAND | reg_select_bits);
	if(cs5529_wait_for_idle(dev))
		comedi_error(dev, "time or signal in cs5529_config_write()");
}

/* read from cs5529 register */
static unsigned int cs5529_config_read(comedi_device *dev, unsigned int reg_select_bits)
{
	unsigned int value;

	reg_select_bits &= CSCMD_REGISTER_SELECT_MASK;
	cs5529_command(dev, CSCMD_COMMAND | CSCMD_READ | reg_select_bits);
	if(cs5529_wait_for_idle(dev))
		comedi_error(dev, "timeout or signal in cs5529_config_read()");
	value = (ni_ao_win_inw(dev, CAL_ADC_Config_Data_High_Word_67xx) << 16) & 0xff0000;
	value |= ni_ao_win_inw(dev, CAL_ADC_Config_Data_Low_Word_67xx) & 0xffff;
	return value;
}

static int cs5529_do_conversion(comedi_device *dev, unsigned short *data)
{
	int retval;
	unsigned short status;

	cs5529_command(dev, CSCMD_COMMAND | CSCMD_SINGLE_CONVERSION);
	retval = cs5529_wait_for_idle(dev);
	if(retval)
	{
		comedi_error(dev, "timeout or signal in cs5529_do_conversion()");
		return -ETIME;
	}
	status = ni_ao_win_inw(dev, CAL_ADC_Status_67xx);
	if(status & CSS_OSC_DETECT)
	{
		rt_printk("ni_mio_common: cs5529 conversion error, status CSS_OSC_DETECT\n");
		return -EIO;
	}
	if(status & CSS_OVERRANGE)
	{
		rt_printk("ni_mio_common: cs5529 conversion error, overrange (ignoring)\n");
	}
	if(data)
	{
		*data = ni_ao_win_inw(dev, CAL_ADC_Data_67xx);
		/* cs5529 returns 16 bit signed data in bipolar mode */
		*data ^= (1 << 15);
	}
	return 0;
}

static int cs5529_ai_insn_read(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int n, retval;
	unsigned short sample;
	unsigned int channel_select;
	const unsigned int INTERNAL_REF = 0x1000;

	/* Set calibration adc source.  Docs lie, reference select bits 8 to 11
	 * do nothing. bit 12 seems to chooses internal reference voltage, bit
	 * 13 causes the adc input to go overrange (maybe reads external reference?) */
	if(insn->chanspec & CR_ALT_SOURCE)
		channel_select = INTERNAL_REF;
	else
		channel_select = CR_CHAN(insn->chanspec);
	ni_ao_win_outw(dev, channel_select, AO_Calibration_Channel_Select_67xx);

	for(n = 0; n < insn->n; n++)
	{
		retval = cs5529_do_conversion(dev, &sample);
		if(retval < 0) return retval;
		data[n] = sample;
	}
	return insn->n;
}

static int init_cs5529(comedi_device *dev)
{
	unsigned int config_bits = CSCFG_PORT_MODE | CSCFG_WORD_RATE_2180_CYCLES;

#if 1
	/* do self-calibration */
	cs5529_config_write(dev, config_bits | CSCFG_SELF_CAL_OFFSET_GAIN, CSCMD_CONFIG_REGISTER);
	/* need to force a conversion for calibration to run */
	cs5529_do_conversion(dev, NULL);
#else
	/* force gain calibration to 1 */
	cs5529_config_write(dev, 0x400000, CSCMD_GAIN_REGISTER);
	cs5529_config_write(dev, config_bits | CSCFG_SELF_CAL_OFFSET, CSCMD_CONFIG_REGISTER);
	if(cs5529_wait_for_idle(dev))
		comedi_error(dev, "timeout or signal in init_cs5529()\n");
#endif
	if(0)
	{
		rt_printk("config: 0x%x\n", cs5529_config_read(dev, CSCMD_CONFIG_REGISTER));
		rt_printk("gain: 0x%x\n", cs5529_config_read(dev, CSCMD_GAIN_REGISTER));
		rt_printk("offset: 0x%x\n", cs5529_config_read(dev, CSCMD_OFFSET_REGISTER));
	}
	return 0;
}


