/*
    module/ni-E.c
    Hardware driver for NI E series cards

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1997-9 David A. Schleef <ds@stm.lbl.gov>

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
	This file needs to be included by another file, e.g.,
	atmio-E.c.

	Interrupt support added by Truxton Fulton <trux@truxton.com>

	References (from ftp://ftp.natinst.com/support/manuals):
	
	   340747b.pdf  AT-MIO E series Register Level Programmer Manual
	   341079b.pdf  PCI E Series RLPM
	   340934b.pdf  DAQ-STC reference manual

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

	deal with at-mio-16de-10 revision D to N changes, etc.
	
*/

#include <8255.h>


/* reference: ground, common, differential, other */
static int ni_modebits1[4]={ 0x3000, 0x2000, 0x1000, 0 };
static int ni_modebits2[4]={ 0x3f, 0x3f, 0x37, 0x37 };

static int ni_gainlkup[][16]={
	{ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
	{ 1, 2, 4, 7, 9, 10, 12, 15, 0,0,0,0,0,0,0,0 },
	{ 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 0,0 },
	{ 0, 1, 4, 7, 8, 9, 12, 15, 0, 0, 0, 0, 0, 0, 0, 0 },
	{ 0, 1, 4, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
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
	RANGE( 0,	20	),
	RANGE( 0,	10	),
	RANGE( 0,	2	),
	RANGE( 0,	0.2	),
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
static comedi_lrange range_ni_E_ai_limited_602x={ 8, {
	RANGE( -10,	10	),
	RANGE( -5,	5	),
	RANGE( -0.5,	0.5	),
	RANGE( -0.05,	0.05	),
	RANGE( 0,	20	),
	RANGE( 0,	10	),
	RANGE( 0,	1	),
	RANGE( 0,	0.1	),
}};
static comedi_lrange range_ni_E_ai_603x={ 4, {
	RANGE( -10,	10	),
	RANGE( -5,	5	),
	RANGE( -0.5,	0.5	),
	RANGE( -0.05,	0.05	),
}};
#if 0
static comedi_lrange range_ni_E_ao = { 2, {
	RANGE( -10,	10	),
	RANGE( 0,	10	),
}};
#endif
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
	&range_ni_E_ai_limited_602x,
	&range_ni_E_ai_603x,
};



static int ni_dio(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int ni_eeprom(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int ni_calib(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);

static void caldac_setup(comedi_device *dev,comedi_subdevice *s);
static int ni_read_eeprom(comedi_device *dev,int addr);


static void ni_handle_fifo_half_full(comedi_device *dev);
static void ni_handle_fifo_dregs(comedi_device *dev);

static int ni_ao_fifo_half_empty(comedi_device *dev,comedi_subdevice *s);

static int ni_8255_callback(int dir,int port,int data,void *arg);

static int ni_ns_to_timer(int *nanosec,int round_mode);

static int gpct_setup(comedi_device *dev,comedi_subdevice *s);
static int ni_gpct(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);

#undef DEBUG

#define AIMODE_NONE		0
#define AIMODE_HALF_FULL	1
#define AIMODE_SCAN		2
#define AIMODE_SAMPLE		3

#ifdef CONFIG_COMEDI_RT

#define RTirqmask_AI 1
#define RTirqmask_AO 2

static int ni_ai_lock(comedi_device *dev,comedi_subdevice *s)
{
	comedi_change_irq_flags(dev->irq,dev,NI_E_IRQ_FLAGS | SA_PRIORITY );

	return 0;
}

static int ni_ai_unlock(comedi_device *dev,comedi_subdevice *s)
{
	comedi_change_irq_flags(dev->irq,dev,NI_E_IRQ_FLAGS );

	return 0;
}

#else

#define ni_ai_lock NULL
#define ni_ai_unlock NULL

#endif

static void ni_E_interrupt(int irq,void *d,struct pt_regs * regs)
{
	comedi_device *dev=d;
	comedi_subdevice *s=dev->subdevices;
	int status;
	unsigned short b_status;
	int ack=0;
	int wsave;

/*
    If you want to use windowed registers in an interrupt, it is
    important that you restore the window address register.  If
    you change certain modes, e.g., AI_Configuration_Start/End,
    you need to set up software flags for non-interrupt routines.
*/
	wsave=win_save();
	
	b_status=ni_readw(AO_Status_1);
	status=ni_readw(AI_Status_1);
#ifdef PCIDMA
printk("status=0x%04x,0x%04x\n",status,b_status);
#ifdef PCIDMA
printk("mite status=0x%08x\n",readw(devpriv->mite->mite_io_addr+0x14));
#endif
#endif

#ifdef PCIDMA
mite_dma_tcr(devpriv->mite);
#endif
	if(status&(AI_Overrun_St|AI_Overflow_St)){
		rt_printk("ni_E: overrun/overflow status=0x%04x\n",status);
		win_out(0x0000,Interrupt_A_Enable_Register);
		comedi_done(dev,s);
		return;
	}

	if(status&AI_SC_TC_St){
#ifdef DEBUG
rt_printk("ni-E: SC_TC interrupt\n");
#endif
		if(s->cur_trig.n){	/* XXX fix */
			ni_handle_fifo_dregs(dev);
			win_out(0x0000,Interrupt_A_Enable_Register);
			comedi_done(dev,s);
		}

		ack|=AI_SC_TC_Interrupt_Ack;
	}
	switch(devpriv->aimode){
	default:
		break;
	case AIMODE_HALF_FULL:
		if(status&AI_FIFO_Half_Full_St){
			ni_handle_fifo_half_full(dev);
		}
		break;
	case AIMODE_SCAN:
#if 0
		if(status&AI_START_St){
			/* just ack it */
			ack|=AI_START_Interrupt_Ack;
		}
#endif
		if(status&AI_STOP_St){
			ni_handle_fifo_dregs(dev);
	
			comedi_eos(dev,dev->subdevices+0);
	
			/* we need to ack the START, also */
			ack|=AI_STOP_Interrupt_Ack|AI_START_Interrupt_Ack;
		}
		break;
	case AIMODE_SAMPLE:
		ni_handle_fifo_dregs(dev);
#if 0
		if(s->event_mask&COMEDI_CB_EOS){
			comedi_eos(dev,dev->subdevices+0);
		}
#endif
		break;
	}

	if(b_status&AO_Overrun_St){
		printk("ni-E: AO FIFO underrun status=0x%04x status2=0x%04x\n",b_status,ni_readw(AO_Status_2));
	}

	if(b_status&AO_BC_TC_St){
		printk("ni-E: AO BC_TC status=0x%04x status2=0x%04x\n",b_status,ni_readw(AO_Status_2));
	}

	if(b_status&AO_FIFO_Request_St)
		ni_ao_fifo_half_empty(dev,dev->subdevices+1);

	b_status=ni_readw(AO_Status_1);
	if(b_status&Interrupt_B_St){
		if(b_status&AO_FIFO_Request_St){
			printk("AO buffer underrun\n");
		}
		printk("Ack! didn't clear AO interrupt. b_status=0x%04x\n",b_status);
		win_out(0,Interrupt_B_Enable_Register);
	}

	if(ack){
		ack|=AI_START1_Interrupt_Ack;
		ni_writew(ack,Interrupt_A_Ack);
	}

	win_restore(wsave);
}

static void ni_ai_fifo_read(comedi_device *dev,comedi_subdevice *s,
		sampl_t *data,int n)
{
	int i;
	sampl_t d;
	int j;
	unsigned int mask;

	mask=(1<<boardtype.adbits)-1;
	j=s->cur_chan;
	for(i=0;i<n;i++){
		d=ni_readw(ADC_FIFO_Data_Register);
		d^=devpriv->ai_xorlist[j];
		d&=mask;
		data[i]=d;
		j++;
		if(j>=s->cur_chanlist_len){
			j=0;
			//s->event_mask |= COMEDI_CB_EOS;
		}
	}
	s->cur_chan=j;
}


static void ni_handle_fifo_half_full(comedi_device *dev)
{
	int n,m;
	comedi_subdevice *s=dev->subdevices+0;

	/*
	   if we got a fifo_half_full interrupt, we can transfer fifo/2
	   samples without checking the empty flag.  It doesn't matter if
	   we transfer the rest of the samples, the performance trade-off
	   is minimal (checking empty flag for a few samples vs. having
	   1% more interrupts.)  At really high speeds, it's better to
	   ignore them.

	*/
	
	n=boardtype.ai_fifo_depth/2;

	/* this makes the assumption that the buffer length is
	   greater than the half-fifo depth. */

	if(s->buf_int_ptr+n*sizeof(sampl_t)>=s->cur_trig.data_len){
		m=(s->cur_trig.data_len-s->buf_int_ptr)/sizeof(sampl_t);
		ni_ai_fifo_read(dev,s,((void *)(s->cur_trig.data))+s->buf_int_ptr,m);
		s->buf_int_count+=m*sizeof(sampl_t);
		n-=m;
		s->buf_int_ptr=0;

		comedi_eobuf(dev,s);
	}
	ni_ai_fifo_read(dev,s,((void *)(s->cur_trig.data))+s->buf_int_ptr,n);
	s->buf_int_count+=n*sizeof(sampl_t);
	s->buf_int_ptr+=n*sizeof(sampl_t);

	comedi_bufcheck(dev,s);
}

/*
   Empties the AI fifo
*/
static void ni_handle_fifo_dregs(comedi_device *dev)
{
	comedi_subdevice *s=dev->subdevices+0;
	sampl_t *data,d;
	int i,n;
	int j;
	unsigned int mask;

	/*
	   Too bad NI didn't have the foresight to return a
	   "fifo empty" value if we read past the end of the
	   FIFO.  It would have made this procedure twice
	   as fast.

	   We can calculate how many samples to transfer.
	   This would save a lot of time.
	*/

	mask=(1<<boardtype.adbits)-1;
	j=s->cur_chan;
	data=((void *)s->cur_trig.data)+s->buf_int_ptr;
	while(1){
		n=(s->cur_trig.data_len-s->buf_int_ptr)/sizeof(sampl_t);
		for(i=0;i<n;i++){
			if(ni_readw(AI_Status_1)&AI_FIFO_Empty_St){
				s->cur_chan=j;
				return;
			}
			d=ni_readw(ADC_FIFO_Data_Register);
			d^=devpriv->ai_xorlist[j];
			d&=mask;
			*data=d;
			j++;
			if(j>=s->cur_chanlist_len){
				j=0;
				//s->event_mask |= COMEDI_CB_EOS;
			}
			data++;
			s->buf_int_ptr+=sizeof(sampl_t);
			s->buf_int_count+=sizeof(sampl_t);
		}
		s->buf_int_ptr=0;
		data=s->cur_trig.data;
		comedi_eobuf(dev,s);
	}
}

/*
   used for both cancel ioctl and board initialization

   this is pretty harsh for a cancel, but it works...
 */
static int ni_ai_reset(comedi_device *dev,comedi_subdevice *s)
{
#ifdef PCIDMA
	mite_dma_disarm(devpriv->mite);
#endif
	win_out(0x0000,Interrupt_A_Enable_Register);

	win_out(AI_Reset,Joint_Reset_Register);

	win_out(1,ADC_FIFO_Clear);

	/* ai configuration */

	win_out(AI_Configuration_Start,Joint_Reset_Register);

	win_out(0x0000,AI_Command_1_Register); /* reset pulses */
	win_out(0x000d,AI_Mode_1_Register);
	win_out(0x0000,AI_Mode_2_Register);
#if 0
	win_out((1<<6)|0x0000,AI_Mode_3_Register); /* generate FIFO interrupts on half full */
#else
	win_out((0<<6)|0x0000,AI_Mode_3_Register); /* generate FIFO interrupts on non-empty */
#endif
	win_out(0xa4a0,AI_Personal_Register); /* ? */
	win_out(0x032e,AI_Output_Control_Register);
	win_out(0x0060,AI_Trigger_Select_Register); /* trigger source */

	/* this should be done in _ai_modeX() */
	win_out(0x29e0,AI_START_STOP_Select_Register);

	/* the following registers should not be changed:
		Clock_and_FOUT_Register
		AI_Mode_1_Register
		AI_Mode_3_Register
		AI_Personal_Register
		AI_Output_Control_Register
		AI_Trigger_Select_Register
	*/

	win_out(0x3f80,Interrupt_A_Ack_Register); /* clear interrupts */

	win_out(AI_Configuration_End,Joint_Reset_Register);

	return 0;
}

static void ni_load_channelgain_list(comedi_device *dev,unsigned int n_chan,unsigned int *list,int dither);


/*
	Mode 0 is immediate
*/
static int ni_ai_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int i;
	int chan;
	int wsave;
	
	wsave=win_save();

	win_out(1,ADC_FIFO_Clear);

	/* interrupt on errors */
	win_out(0x0020,Interrupt_A_Enable_Register) ;
	
	for(chan=0;chan<it->n_chan;chan++){
		ni_load_channelgain_list(dev,1,it->chanlist+chan,(it->flags&TRIG_DITHER)==TRIG_DITHER);
#if 0
	/* needs start configuration */
	win_out(AI_START_Edge|AI_START_Sync|
		AI_STOP_Select(19)|AI_STOP_Sync,
		AI_START_STOP_Select_Register);
#endif


		win_out(1,AI_Command_1_Register);
	
		/* I don't know how long it takes to access the bus,
		   so shorter loops might cause timeouts */
#define NI_TIMEOUT 1000
		for(i=0;i<NI_TIMEOUT;i++){
			if(!(ni_readw(AI_Status_1)&AI_FIFO_Empty_St)){
				it->data[chan]=ni_readw(ADC_FIFO_Data_Register);
				it->data[chan]^=devpriv->ai_xorlist[0];
				it->data[chan]&=(1<<boardtype.adbits)-1;
				break;
			}
			/*udelay(25);*/
		}
		if(i==NI_TIMEOUT)goto timeout;
	}
	win_restore(wsave);
	return chan;

timeout:
	rt_printk("ni_E: timeout 2\n");
	win_restore(wsave);
	return -ETIME;
}


static int ni_ai_insn_read(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{
	int i,n;
	int wsave;
	unsigned int mask,sign;
	
	wsave=win_save();

	win_out(1,ADC_FIFO_Clear);

	/* interrupt on errors */
	win_out(0x0020,Interrupt_A_Enable_Register) ;
	
	//ni_load_channelgain_list(dev,1,&insn->chanspec,(insn->flags&TRIG_DITHER)==TRIG_DITHER);
	ni_load_channelgain_list(dev,1,&insn->chanspec,0);

#if 0
#define NI_TIMEOUT 1000
#endif
	mask=(1<<boardtype.adbits)-1;
	sign=devpriv->ai_xorlist[0];
	for(n=0;n<insn->n;n++){
		win_out(1,AI_Command_1_Register);
		for(i=0;i<NI_TIMEOUT;i++){
			if(!(ni_readw(AI_Status_1)&AI_FIFO_Empty_St))
				break;
		}
		if(i==NI_TIMEOUT){
			rt_printk("ni_E: timeout 2\n");
			win_restore(wsave);
			return -ETIME;
		}
		data[n]=(ni_readw(ADC_FIFO_Data_Register)&mask)^sign;
	}
	win_restore(wsave);
	return insn->n;
}


static void ni_load_channelgain_list(comedi_device *dev,unsigned int n_chan,unsigned int *list,int dither)
{
	unsigned int chan,range,aref;
	unsigned int i;
	unsigned int hi,lo;
	unsigned short sign;

	if(n_chan==1){
		if(devpriv->changain_state && devpriv->changain_spec==list[0]){
			// ready to go.
			return;
		}
		devpriv->changain_state=1;
		devpriv->changain_spec=list[0];
	}else{
		devpriv->changain_state=0;
	}
	
	win_out(1,Configuration_Memory_Clear);

	sign=1<<(boardtype.adbits-1);
	for(i=0;i<n_chan;i++){
		chan=CR_CHAN(list[i]);
		range=CR_RANGE(list[i]);
		aref=CR_AREF(list[i]);
		
		/* fix the external/internal range differences */
		range=ni_gainlkup[boardtype.gainlkup][range];
		devpriv->ai_xorlist[i]=(range<8)?sign:0;

		hi=ni_modebits1[aref]|(chan&ni_modebits2[aref]);
		ni_writew(hi,Configuration_Memory_High);

		lo=((i==n_chan-1)?0x8000:0) | ((range&0x8)<<5) | (range&0x7) | (dither<<9);
		ni_writew(lo,Configuration_Memory_Low);
	}

	/* prime the channel/gain list */

	win_out(1,AI_Command_1_Register);
	for(i=0;i<1000;i++){
		if(!(ni_readw(AI_Status_1)&AI_FIFO_Empty_St)){
			win_out(1,ADC_FIFO_Clear);
			return;
		}
		//udelay(25);
	}
	rt_printk("ni_E: timeout 1\n");
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
	return divider;
}

static int ni_ai_cmdtest(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{
	int err=0;
	int tmp;

	/* step 1: make sure trigger sources are trivially valid */

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW;
	if(!cmd->start_src && tmp!=cmd->start_src)err++;

	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER|TRIG_EXT;
	if(!cmd->scan_begin_src && tmp!=cmd->scan_begin_src)err++;

	tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER|TRIG_EXT;
	if(!cmd->convert_src && tmp!=cmd->convert_src)err++;

	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src && tmp!=cmd->scan_end_src)err++;

	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT|TRIG_NONE;
	if(!cmd->stop_src && tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */

	/* note that mutual compatiblity is not an issue here */
	if(cmd->scan_begin_src!=TRIG_TIMER &&
	   cmd->scan_begin_src!=TRIG_EXT)err++;
	if(cmd->convert_src!=TRIG_TIMER &&
	   cmd->convert_src!=TRIG_EXT)err++;

	if(err)return 2;

	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
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
	}else{
		/* external trigger */
		/* should be level/edge, hi/lo specification here */
		/* should specify multiple external triggers */
		if(cmd->scan_begin_arg>9){
			cmd->scan_begin_arg=9;
			err++;
		}
	}
	if(cmd->convert_src==TRIG_TIMER){
		if(cmd->convert_arg<boardtype.ai_speed){
			cmd->convert_arg=boardtype.ai_speed;
			err++;
		}
		if(cmd->convert_arg>TIMER_BASE*0xffff){
			cmd->convert_arg=TIMER_BASE*0xffff;
			err++;
		}
	}else{
		/* external trigger */
		/* see above */
		if(cmd->convert_arg>9){
			cmd->convert_arg=9;
			err++;
		}
	}

	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}
	if(cmd->stop_src==TRIG_COUNT){
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

	if(cmd->scan_begin_src==TRIG_TIMER){
		tmp=cmd->scan_begin_arg;
		ni_ns_to_timer(&cmd->scan_begin_arg,cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->scan_begin_arg)err++;
	}
	if(cmd->convert_src==TRIG_TIMER){
		tmp=cmd->convert_arg;
		ni_ns_to_timer(&cmd->convert_arg,cmd->flags&TRIG_ROUND_MASK);
		if(tmp!=cmd->convert_arg)err++;
		if(cmd->scan_begin_src==TRIG_TIMER &&
		  cmd->scan_begin_arg<cmd->convert_arg*cmd->scan_end_arg){
			cmd->scan_begin_arg=cmd->convert_arg*cmd->scan_end_arg;
			err++;
		}
	}

	if(err)return 4;

	return 0;
}

static int ni_ai_cmd(comedi_device *dev,comedi_subdevice *s)
{
	int wsave;
	comedi_cmd *cmd=&s->cmd;
	int timer;
	int mode1=0; /* mode1 is needed for both stop and convert */
	int mode2=0;

	wsave = win_save();

	win_out(1,ADC_FIFO_Clear);

	ni_load_channelgain_list(dev,cmd->chanlist_len,cmd->chanlist,
		(cmd->flags&TRIG_DITHER)==TRIG_DITHER);

	/* start configuration */
	win_out(AI_Configuration_Start,Joint_Reset_Register);

	switch(cmd->stop_src){
	case TRIG_COUNT:
		/* stage number of scans */
		win_out((cmd->stop_arg-1)>>16,AI_SC_Load_A_Registers);
		win_out((cmd->stop_arg-1)&0xffff,AI_SC_Load_A_Registers+1);

		//mode1 |= AI_Start_Stop | AI_Mode_1_Reserved | AI_Trigger_Once;
		mode1 |= 0xd;
		win_out(mode1,AI_Mode_1_Register);
	
		/* load SC (Scan Count) */
		win_out(AI_SC_Load,AI_Command_1_Register);

s->cur_trig.n=1; /* XXX */

		break;
	case TRIG_NONE:
		/* stage number of scans */
		win_out(0,AI_SC_Load_A_Registers);
		win_out(0,AI_SC_Load_A_Registers+1);
	
		//mode1 |= AI_Start_Stop | AI_Mode_1_Reserved | AI_Continuous;
		mode1 |= 0xe;
		win_out(mode1,AI_Mode_1_Register);

		/* load SC (Scan Count) */
		win_out(AI_SC_Load,AI_Command_1_Register);

s->cur_trig.n=0; /* XXX */

		break;
	}

	switch(cmd->scan_begin_src){
	case TRIG_TIMER:
		/*
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
		win_out(AI_START_Edge|AI_START_Sync|
			AI_STOP_Select(19)|AI_STOP_Sync,
			AI_START_STOP_Select_Register);

		timer=ni_ns_to_timer(&cmd->scan_begin_arg,TRIG_ROUND_NEAREST);
		win_out((timer>>16),AI_SI_Load_A_Registers);
		win_out((timer&0xffff),AI_SI_Load_A_Registers+1);

		/* AI_SI_Initial_Load_Source=A */
		mode2 |= AI_SI_Initial_Load_Source&0;
		win_out(mode2,AI_Mode_2_Register);

		/* load SI */
		win_out(AI_SI_Load,AI_Command_1_Register);

		/* stage freq. counter into SI B */
		win_out((timer>>16),AI_SI_Load_B_Registers);
		win_out((timer&0xffff),AI_SI_Load_B_Registers+1);

		break;
	case TRIG_EXT:
		win_out(AI_START_Edge|AI_START_Sync|
			AI_START_Select(1+cmd->scan_begin_arg)|
			AI_STOP_Select(19)|AI_STOP_Sync,
			AI_START_STOP_Select_Register);
		break;
	}

	switch(cmd->convert_src){
	case TRIG_TIMER:
		timer=ni_ns_to_timer(&cmd->convert_arg,TRIG_ROUND_NEAREST);
		win_out(timer,AI_SI2_Load_A_Register); /* 0,0 does not work. */
		win_out(timer,AI_SI2_Load_B_Register);

		/* AI_SI2_Reload_Mode = alternate */
		/* AI_SI2_Initial_Load_Source = A */
		win_out((AI_SI2_Initial_Load_Source&0)|
			(AI_SI2_Reload_Mode),
			AI_Mode_2_Register);

		/* AI_SI2_Load */
		win_out(AI_SI2_Load,AI_Command_1_Register);

		mode2 |= AI_SI_Reload_Mode(0);
		mode2 |= 0&AI_SI_Initial_Load_Source;
		mode2 |= AI_SI2_Reload_Mode; // alternate
		mode2 |= AI_SI2_Initial_Load_Source; // B

		win_out(mode2,AI_Mode_2_Register);

		break;
	case TRIG_EXT:
		mode1 |= AI_CONVERT_Source_Select(1+cmd->convert_arg) |
			AI_CONVERT_Source_Polarity |
			AI_Start_Stop;
		win_out(mode1,AI_Mode_1_Register);

		win_out(mode2 | AI_SI2_Reload_Mode,AI_Mode_2_Register);

		mode2 |= AI_SI_Reload_Mode(0);
		mode2 |= 0&AI_SI_Initial_Load_Source;
		mode2 |= AI_SI2_Reload_Mode; // alternate
		mode2 |= AI_SI2_Initial_Load_Source; // B

		win_out(mode2,AI_Mode_2_Register);

		break;
	}

	if(dev->irq){
		int bits;

		/* interrupt on FIFO, errors, SC_TC */
		bits=AI_FIFO_Interrupt_Enable|
			AI_Error_Interrupt_Enable|
			AI_SC_TC_Interrupt_Enable;

		if(s->cb_mask&COMEDI_CB_EOS){
			/* wake on end-of-scan */
			devpriv->aimode=AIMODE_SCAN;
		}else{
			devpriv->aimode=AIMODE_HALF_FULL;
		}
		switch(devpriv->aimode){
		case AIMODE_HALF_FULL:
			/*generate FIFO interrupts on half-full */
			win_out(AI_FIFO_Mode_HF|0x0000,AI_Mode_3_Register);
			break;
		case AIMODE_SAMPLE:
			/*generate FIFO interrupts on non-empty */
			win_out(AI_FIFO_Mode_NE|0x0000,AI_Mode_3_Register);
			break;
		case AIMODE_SCAN:
			/*generate FIFO interrupts on half-full */
			win_out(AI_FIFO_Mode_HF|0x0000,AI_Mode_3_Register);
			bits|=AI_STOP_Interrupt_Enable;
			break;
		default:
			break;
		}

		win_out(0x3f80,Interrupt_A_Ack_Register); /* clear interrupts */

		win_out(bits,Interrupt_A_Enable_Register) ;
	}else{
		/* interrupt on nothing */
		win_out(0x0000,Interrupt_A_Enable_Register) ;

		/* XXX start polling if necessary */
	}

	/* end configuration */
	win_out(AI_Configuration_End,Joint_Reset_Register);
	
	switch(cmd->scan_begin_src){
	case TRIG_TIMER:
		/* AI_SI2_Arm, AI_SI_Arm, AI_DIV_Arm, AI_SC_Arm */
		win_out(0x1540,AI_Command_1_Register);
		break;
	case TRIG_EXT:
		/* AI_SI2_Arm, AI_DIV_Arm, AI_SC_Arm */
		win_out(0x1500,AI_Command_1_Register);
		break;
	}

	/* AI_START1_Pulse */
	win_out(AI_START1_Pulse,AI_Command_2_Register);

	win_restore(wsave);
	return 0;
}

/*
	mode 2 is timed, multi-channel
*/
static int ni_ai_mode2(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int wsave;
	int trigvar;
	int trigvar1;

	wsave=win_save();

	win_out(1,ADC_FIFO_Clear);

	trigvar = ni_ns_to_timer(&it->trigvar,TRIG_ROUND_NEAREST);
	trigvar1 = ni_ns_to_timer(&it->trigvar1,TRIG_ROUND_NEAREST);

	ni_load_channelgain_list(dev,it->n_chan,it->chanlist,(it->flags&TRIG_DITHER)==TRIG_DITHER);
	
	/* start configuration */
	win_out(AI_Configuration_Start,Joint_Reset_Register);

	/* stage number of scans */
	if(it->n==0){
		/* hack for continuous acquisition */
		win_out(0,AI_SC_Load_A_Registers);
		win_out(0,AI_SC_Load_A_Registers+1);
		win_out(0x000e,AI_Mode_1_Register);
	}else{
		win_out((it->n-1)>>16,AI_SC_Load_A_Registers);
		win_out((it->n-1)&0xffff,AI_SC_Load_A_Registers+1);
		win_out(0x000d,AI_Mode_1_Register);
	}
	
	/* load SC (Scan Count) */
	win_out(0x20,AI_Command_1_Register);
	
	/*
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
	win_out(AI_START_Edge|AI_START_Sync|
		AI_STOP_Select(19)|AI_STOP_Sync,
		AI_START_STOP_Select_Register);

	win_out((trigvar>>16),AI_SI_Load_A_Registers);
	win_out((trigvar&0xffff),AI_SI_Load_A_Registers+1);
	/* AI_SI_Initial_Load_Source=A */
	win_out(0,AI_Mode_2_Register);
	/* load SI */
	win_out(0x200,AI_Command_1_Register);

	/* stage freq. counter into SI B */
	win_out((trigvar>>16),AI_SI_Load_B_Registers);
	win_out((trigvar&0xffff),AI_SI_Load_B_Registers+1);

	win_out(trigvar1,AI_SI2_Load_A_Register); /* 0,0 does not work. */
	win_out(trigvar1,AI_SI2_Load_B_Register);

	/* AI_SI2_Reload_Mode = alternate */
	/* AI_SI2_Initial_Load_Source = A */
	win_out(0x0100,AI_Mode_2_Register);

	/* AI_SI2_Load */
	win_out(0x0800,AI_Command_1_Register);

	/* AI_SI_Initial_Load_Source=0
	   AI_SI_Reload_Mode(0)
	   AI_SI2_Reload_Mode = alternate, AI_SI2_Initial_Load_Source = B */
	win_out(0x0300,AI_Mode_2_Register);

	if(dev->irq){
		int bits;

		/* interrupt on FIFO, errors, SC_TC */
		bits=AI_FIFO_Interrupt_Enable|AI_Error_Interrupt_Enable|
		     AI_SC_TC_Interrupt_Enable;
		//bits|=Pass_Thru_0_Interrupt_Enable;

		if(s->cb_mask&COMEDI_CB_EOS){
			/* wake on end-of-scan */
			devpriv->aimode=AIMODE_SCAN;
		}else{
			devpriv->aimode=AIMODE_HALF_FULL;
		}
		switch(devpriv->aimode){
		case AIMODE_HALF_FULL:
			/*generate FIFO interrupts on half-full */
			win_out(AI_FIFO_Mode_HF|0x0000,AI_Mode_3_Register);
			break;
		case AIMODE_SAMPLE:
			/*generate FIFO interrupts on non-empty */
			win_out(AI_FIFO_Mode_NE|0x0000,AI_Mode_3_Register);
			break;
		case AIMODE_SCAN:
			/*generate FIFO interrupts on half-full */
			win_out(AI_FIFO_Mode_HF|0x0000,AI_Mode_3_Register);
			bits|=AI_STOP_Interrupt_Enable;
			break;
		default:
			break;
		}

		win_out(0x3f80,Interrupt_A_Ack_Register); /* clear interrupts */

		win_out(bits,Interrupt_A_Enable_Register) ;
	}else{
		/* interrupt on nothing */
		win_out(0x0000,Interrupt_A_Enable_Register) ;

		/* XXX start polling if necessary */
	}

#ifdef PCIDMA
#if 0 
	Strobes_Register
	AI_AO_Select_Register
	Interrupt_B_Enable_Register MSC_Pass_thru
#endif

	ni_writeb(0x01,AI_AO_Select);
	mite_dma_prep(devpriv->mite, s);

	mite_dma_arm(devpriv->mite);
#endif

#ifdef DEBUG
rt_printk("end config\n");
#endif
	/* end configuration */
	win_out(AI_Configuration_End,Joint_Reset_Register);
	
	/* AI_SI2_Arm, AI_SI_Arm, AI_DIV_Arm, AI_SC_Arm */
	win_out(0x1540,AI_Command_1_Register);

	/* AI_START1_Pulse */
	win_out(AI_START1_Pulse,AI_Command_2_Register);
#ifdef DEBUG
rt_printk("START1 pulse\n");
#endif

	win_restore(wsave);
	return 0;
}

/*
	mode 4 is external trigger for scans, timer for samples
	in a scan
*/
static int ni_ai_mode4(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int wsave;
	int trigvar1;

	wsave=win_save();

	win_out(1,ADC_FIFO_Clear);

	trigvar1 = ni_ns_to_timer(&it->trigvar1,TRIG_ROUND_NEAREST);
	ni_load_channelgain_list(dev,it->n_chan,it->chanlist,(it->flags&TRIG_DITHER)==TRIG_DITHER);
	
	/* start configuration */
	win_out(AI_Configuration_Start,Joint_Reset_Register);

	/* stage number of scans */
	if(it->n==0){
		/* hack for continuous acquisition */
		win_out(0,AI_SC_Load_A_Registers);
		win_out(0,AI_SC_Load_A_Registers+1);
		win_out(0x000e,AI_Mode_1_Register);
	}else{
		win_out((it->n-1)>>16,AI_SC_Load_A_Registers);
		win_out((it->n-1)&0xffff,AI_SC_Load_A_Registers+1);
		win_out(0x000d,AI_Mode_1_Register);
	}
	
	/* load SC (Scan Count) */
	win_out(0x20,AI_Command_1_Register);
	
	/*
	    AI_SI_Special_Trigger_Delay=0
	    AI_Pre_Trigger=0
	  AI_START_STOP_Select_Register:
	    AI_START_Polarity=0 (?)	rising edge
	    AI_START_Edge=1		edge triggered
	    AI_START_Sync=1 (?)		
	    AI_START_Select=1		PFI0
	    AI_STOP_Polarity=0		rising edge
	    AI_STOP_Edge=0		level
	    AI_STOP_Sync=1		
	    AI_STOP_Select=19		external pin (configuration mem)
	*/
	win_out(AI_START_Edge|AI_START_Sync|AI_START_Select(1)|
		AI_STOP_Select(19)|AI_STOP_Sync,
		AI_START_STOP_Select_Register);
	
#if 0
	win_out((it->trigvar>>16),AI_SI_Load_A_Registers);
	win_out((it->trigvar&0xffff),AI_SI_Load_A_Registers+1);
	/* AI_SI_Initial_Load_Source=A */
	win_out(0,AI_Mode_2_Register);
	/* load SI */
	win_out(0x200,AI_Command_1_Register);

	/* stage freq. counter into SI B */
	win_out((it->trigvar>>16),AI_SI_Load_B_Registers);
	win_out((it->trigvar&0xffff),AI_SI_Load_B_Registers+1);
#endif

	win_out(trigvar1,AI_SI2_Load_A_Register); /* 0,0 does not work. */
	win_out(trigvar1,AI_SI2_Load_B_Register);

	/* AI_SI2_Reload_Mode = alternate */
	/* AI_SI2_Initial_Load_Source = A */
	win_out(0x0100,AI_Mode_2_Register);

	/* AI_SI2_Load */
	win_out(0x0800,AI_Command_1_Register);

	/* AI_SI_Initial_Load_Source=0
	   AI_SI_Reload_Mode(0)
	   AI_SI2_Reload_Mode = alternate, AI_SI2_Initial_Load_Source = B */
	win_out(0x0300,AI_Mode_2_Register);

	if(dev->irq){
		int bits;

		/* interrupt on FIFO, errors, SC_TC */
		bits=0x00a1;

		if(s->cb_mask&COMEDI_CB_EOS){
			/* wake on end-of-scan */
			devpriv->aimode=AIMODE_SCAN;
		}else{
			devpriv->aimode=AIMODE_HALF_FULL;
		}
		switch(devpriv->aimode){
		case AIMODE_HALF_FULL:
			/*generate FIFO interrupts on half-full */
			win_out(AI_SI2_Source_Select|AI_FIFO_Mode_HF|0x0000,AI_Mode_3_Register);
			break;
		case AIMODE_SAMPLE:
			/*generate FIFO interrupts on non-empty */
			win_out(AI_SI2_Source_Select|AI_FIFO_Mode_NE|0x0000,AI_Mode_3_Register);
			break;
		case AIMODE_SCAN:
			/*generate FIFO interrupts on half-full */
			win_out(AI_SI2_Source_Select|AI_FIFO_Mode_HF|0x0000,AI_Mode_3_Register);
			bits|=AI_STOP_Interrupt_Enable;
			break;
		default:
			break;
		}

		/* clear interrupts */
		win_out(0x3f80,Interrupt_A_Ack_Register);

		win_out(bits,Interrupt_A_Enable_Register) ;
	}else{
		/* interrupt on nothing */
		win_out(0x0000,Interrupt_A_Enable_Register) ;

		/* XXX start polling if necessary */
	}
	
	/* end configuration */
	win_out(AI_Configuration_End,Joint_Reset_Register);

	/* AI_SI2_Arm, AI_DIV_Arm, AI_SC_Arm */
	win_out(0x1500,AI_Command_1_Register);

	/* AI_START1_Pulse */
	win_out(AI_START1_Pulse,AI_Command_2_Register);

	win_restore(wsave);
	return 0;
}


static void ni_ao_fifo_load(comedi_device *dev,comedi_subdevice *s,
		sampl_t *data,int n)
{
	int i;

	for(i=0;i<n;i++){
		ni_writew(data[i],DAC_FIFO_Data);
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
	int n,m;

	n=(s->buf_int_count-s->buf_user_count)/sizeof(sampl_t);
	if(n==0)return 0;
	if(n>boardtype.ao_fifo_depth/2)
		n=boardtype.ao_fifo_depth/2;

	if(s->buf_int_ptr+n*sizeof(sampl_t)>s->cur_trig.data_len){
		m=(s->cur_trig.data_len-s->buf_int_ptr)/sizeof(sampl_t);
		ni_ao_fifo_load(dev,s,((void *)(s->cur_trig.data))+s->buf_int_ptr,m);
		s->buf_int_count+=m*sizeof(sampl_t);
		s->buf_int_ptr=0;
		n-=m;
	}
	ni_ao_fifo_load(dev,s,((void *)(s->cur_trig.data))+s->buf_int_ptr,n);
	s->buf_int_count+=n*sizeof(sampl_t);
	s->buf_int_ptr+=n*sizeof(sampl_t);

	comedi_bufcheck(dev,s);

	return 1;
}

static int ni_ao_prep_fifo(comedi_device *dev,comedi_subdevice *s)
{
	int n;

	/* reset fifo */
	win_out(0,DAC_FIFO_Clear);

	/* load some data */
	n=(s->buf_int_count-s->buf_user_count)/sizeof(sampl_t);
	if(n==0)return 0;
	if(n>boardtype.ao_fifo_depth)
		n=boardtype.ao_fifo_depth;

	ni_ao_fifo_load(dev,s,((void *)(s->cur_trig.data))+s->buf_int_ptr,n);
	s->buf_int_count+=n*sizeof(sampl_t);
	s->buf_int_ptr+=n*sizeof(sampl_t);

	return n;
}






static int ni_ao_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	unsigned int data;
	unsigned int conf;
	unsigned int chan;
	unsigned int range;
	int i;

	/* XXX this function could use a little cleanup to make sure that it
	 * gets the range settings correct for every board. */
	
for(i=0;i<it->n_chan;i++){
	data=it->data[i];
	chan=CR_CHAN(it->chanlist[i]);

	conf=chan<<8;
	
	/* XXX check range with current range in flaglist[chan] */
	/* should update calibration if range changes (ick) */

	range = CR_RANGE(it->chanlist[i]);
	if(boardtype.ao_unipolar){
		conf |= (range&1)^1;
	}
	conf |= (range&2)<<1;
	
	/* not all boards can deglitch, but this shouldn't hurt */
	if(it->flags & TRIG_DEGLITCH)
		conf |= 2;

	/* analog reference */
	/* AREF_OTHER connects AO ground to AI ground, i think */
	conf |= (CR_AREF(it->chanlist[i])==AREF_OTHER)? 8 : 0;

	ni_writew(conf,AO_Configuration);

	if(((range&1)==0) || !boardtype.ao_unipolar)
		data^=(1<<(boardtype.aobits-1));
	
	ni_writew(data,(chan)? DAC1_Direct_Data : DAC0_Direct_Data);
}
	
	return i;
}

static int ni_ao_mode2(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	unsigned int conf;
	unsigned int chan;
	unsigned int range;
	int trigvar;
	
	trigvar = ni_ns_to_timer(&it->trigvar,TRIG_ROUND_NEAREST);

	chan=CR_CHAN(it->chanlist[0]);

	conf=chan<<8;
	
	/* XXX check range with current range in flaglist[chan] */
	/* should update calibration if range changes (ick) */

	range = CR_RANGE(it->chanlist[0]);
	conf |= (range&1);
	conf |= (range&2)<<1;
	
	/* not all boards can deglitch, but this shouldn't hurt */
	if(it->flags & TRIG_DEGLITCH)
		conf |= 2;

	/* analog reference */
	/* AREF_OTHER connects AO ground to AI ground, i think */
	conf |= (CR_AREF(it->chanlist[0])==AREF_OTHER)? 8 : 0;

	win_out(AO_Disarm,AO_Command_1_Register);

	ni_writew(conf,AO_Configuration);

	/* user is supposed to write() to buffer before triggering */
	if(ni_ao_prep_fifo(dev,s)==0)
		return -EIO;

	win_out(AO_Configuration_Start,Joint_Reset_Register);

	devpriv->ao_mode1|=AO_Trigger_Once;
	win_out(devpriv->ao_mode1,AO_Mode_1_Register);
	devpriv->ao_trigger_select&=~(AO_START1_Polarity|AO_START1_Select(-1));
	devpriv->ao_trigger_select|=AO_START1_Edge|AO_START1_Sync;
	win_out(devpriv->ao_trigger_select,AO_Trigger_Select_Register);
	devpriv->ao_mode3&=~AO_Trigger_Length;
	win_out(devpriv->ao_mode3,AO_Mode_3_Register);

	if(it->n==0){
		devpriv->ao_mode1|=AO_Continuous;
	}else{
		devpriv->ao_mode1&=~AO_Continuous;
	}
	win_out(devpriv->ao_mode1,AO_Mode_1_Register);
	devpriv->ao_mode2&=~AO_BC_Initial_Load_Source;
	win_out(devpriv->ao_mode2,AO_Mode_2_Register);
	if(it->n==0){
		win_out(0xff,AO_BC_Load_A_Register_High);
		win_out(0xffff,AO_BC_Load_A_Register_Low);
	}else{
		win_out(0,AO_BC_Load_A_Register_High);
		win_out(0,AO_BC_Load_A_Register_Low);
	}
	win_out(AO_BC_Load,AO_Command_1_Register);
	devpriv->ao_mode2&=~AO_UC_Initial_Load_Source;
	win_out(devpriv->ao_mode2,AO_Mode_2_Register);
	if(it->n==0){
		win_out(0xff,AO_UC_Load_A_Register_High);
		win_out(0xffff,AO_UC_Load_A_Register_Low);
		win_out(AO_UC_Load,AO_Command_1_Register);
		win_out(0xff,AO_UC_Load_A_Register_High);
		win_out(0xffff,AO_UC_Load_A_Register_Low);
	}else{
		win_out(0,AO_UC_Load_A_Register_High);
		win_out(0,AO_UC_Load_A_Register_Low);
		win_out(AO_UC_Load,AO_Command_1_Register);
		win_out((it->n-1)>>16,AO_UC_Load_A_Register_High);
		win_out((it->n-1)&0xffff,AO_UC_Load_A_Register_Low);
	}

	devpriv->ao_cmd2&=~AO_BC_Gate_Enable;
	ni_writew(devpriv->ao_cmd2,AO_Command_2);
	devpriv->ao_mode1&=~(AO_UI_Source_Select(0x1f)|AO_UI_Source_Polarity);
	win_out(devpriv->ao_mode1,AO_Mode_1_Register);
	devpriv->ao_mode2&=~(AO_UI_Reload_Mode(3)|AO_UI_Initial_Load_Source);
	win_out(devpriv->ao_mode2,AO_Mode_2_Register);
	win_out(0,AO_UI_Load_A_Register_High);
	win_out(1,AO_UI_Load_A_Register_Low);
	win_out(AO_UI_Load,AO_Command_1_Register);
	win_out((trigvar>>16),AO_UI_Load_A_Register_High);
	win_out((trigvar&0xffff),AO_UI_Load_A_Register_Low);

	devpriv->ao_mode1&=~AO_Multiple_Channels;
	win_out(devpriv->ao_mode1,AO_Mode_1_Register);
	win_out(AO_UPDATE_Output_Select(1),AO_Output_Control_Register);

	win_out(AO_DAC0_Update_Mode|AO_DAC1_Update_Mode,AO_Command_1_Register);

	devpriv->ao_mode3|=AO_Stop_On_Overrun_Error;
	win_out(devpriv->ao_mode3,AO_Mode_3_Register);

devpriv->ao_mode2|=AO_FIFO_Mode(1);
	devpriv->ao_mode2&=~AO_FIFO_Retransmit_Enable;
	win_out(devpriv->ao_mode2,AO_Mode_2_Register);

	win_out(AO_Configuration_End,Joint_Reset_Register);

	win_out(devpriv->ao_mode3|AO_Not_An_UPDATE,AO_Mode_3_Register);
	win_out(devpriv->ao_mode3,AO_Mode_3_Register);

	/* wait for DACs to be loaded */
	udelay(100);

	win_out(devpriv->ao_cmd1|AO_UI_Arm|AO_UC_Arm|AO_BC_Arm|AO_DAC1_Update_Mode|AO_DAC0_Update_Mode,
		AO_Command_1_Register);

	win_out(AO_FIFO_Interrupt_Enable|AO_Error_Interrupt_Enable,Interrupt_B_Enable_Register);


	ni_writew(devpriv->ao_cmd2|AO_START1_Pulse,AO_Command_2);
	
	return 0;
}



/*
	digital i/o

*/
static int ni_dio(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int data,mask;
	int i;
	int temp;

	/* rt stuff */
	temp=win_save();

	if(it->flags & TRIG_CONFIG){
		data=s->io_bits;
		for(i=0;i<it->n_chan;i++){
			mask=1<<CR_CHAN(it->chanlist[i]);
			data&= ~mask;
			if(it->data[i])
				data|=mask;
		}
		s->io_bits=data;
		win_out(s->io_bits,DIO_Control_Register);
	}else{
		if(it->flags & TRIG_WRITE){
			do_pack(&s->state,it);
			win_out(s->state,DIO_Output_Register);
		}else{
			data=win_in(DIO_Input_Register);
			di_unpack(data,it);
		}
	}

	win_restore(temp);

	return it->n_chan;
}

static int ni_dio_insn_bits(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data)
{
	if(data[0]){
		s->state &= ~data[0];
		s->state |= (data[0]&data[1]);
		win_out(s->state,DIO_Output_Register);
	}
	data[1] = win_in(DIO_Input_Register);

	return 2;
}

/*
	HACK! 

        general purpose timer counter (CLO)
        This reads the value of the counter

*/
static int ni_gpct(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int data,mask;
	int i;
	int temp;

	/* rt stuff */
	temp=win_save();

	if(it->flags & TRIG_CONFIG){
		data=s->io_bits;
		for(i=0;i<it->n_chan;i++){
			mask=1<<CR_CHAN(it->chanlist[i]);
			data&= ~mask;
			if(it->data[i])
				data|=mask;
		}
		s->io_bits=data;
		win_out(s->io_bits,DIO_Control_Register);
	}else{
		if(it->flags & TRIG_WRITE){
			do_pack(&s->state,it);
			win_out(s->state,DIO_Output_Register);
		}else{
			data=win_in(DIO_Input_Register);
			di_unpack(data,it);
		}
	}

	win_restore(temp);

	return it->n_chan;
}


static int ni_E_init(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	
	dev->n_subdevices=7;
	
	if(alloc_subdevices(dev)<0)
		return -ENOMEM;
	
	/* analog input subdevice */

	s=dev->subdevices+0;
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE|SDF_RT|SDF_GROUND|SDF_COMMON|SDF_DIFF|SDF_OTHER;
	s->subdev_flags|=SDF_DITHER;
	s->n_chan=boardtype.n_adchan;
	s->len_chanlist=512;	/* XXX is this the same for PCI-MIO ? */
	s->maxdata=(1<<boardtype.adbits)-1;
	s->range_table=ni_range_lkup[boardtype.gainlkup];
	s->trig[0]=ni_ai_mode0;
	s->trig[2]=ni_ai_mode2;
	s->trig[4]=ni_ai_mode4;
	s->insn_read=ni_ai_insn_read;
	s->do_cmdtest=ni_ai_cmdtest;
	s->do_cmd=ni_ai_cmd;
	s->cancel=ni_ai_reset;
	s->do_lock=ni_ai_lock;
	s->do_unlock=ni_ai_unlock;
	
	/* analog output subdevice */
	/* XXX what about boards without ao? */

	s=dev->subdevices+1;
	if(boardtype.n_aochan){
		s->type=COMEDI_SUBD_AO;
		s->subdev_flags=SDF_WRITEABLE|SDF_RT|SDF_DEGLITCH|SDF_GROUND|SDF_OTHER;
		s->n_chan=boardtype.n_aochan;
		s->maxdata=(1<<boardtype.aobits)-1;
		if(boardtype.ao_unipolar){
			s->range_table=&range_ni_E_ao_ext;	/* XXX wrong for some boards */
		}else{
			s->range_table=&range_bipolar10;
		}
		s->trig[0]=ni_ao_mode0;
		s->len_chanlist = 2;
		if(boardtype.ao_fifo_depth)
			s->trig[2]=ni_ao_mode2;
	}else{
		s->type=COMEDI_SUBD_UNUSED;
	}
	
	/* digital i/o subdevice */
	
	s=dev->subdevices+2;
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_WRITEABLE|SDF_READABLE|SDF_RT;
	s->n_chan=8;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->io_bits=0;		/* all bits input */
	s->insn_read=ni_dio_insn_bits;
	s->trig[0]=ni_dio;

	/* dio setup */
	win_out(s->io_bits,DIO_Control_Register);
	
	/* 8255 device */
	s=dev->subdevices+3;
	if(boardtype.has_8255){
		subdev_8255_init(dev,s,ni_8255_callback,dev);
	}else{
		s->type=COMEDI_SUBD_UNUSED;
		s->trig[0]=NULL;
	}
	/* XXX */
	
	/* general purpose counter/timer device */
	s=dev->subdevices+4;
	gpct_setup(dev,s);
	s->type=COMEDI_SUBD_COUNTER;
	s->trig[0]=ni_gpct;
	/* XXX */
	
	/* calibration subdevice -- ai and ao */
	s=dev->subdevices+5;
	s->type=COMEDI_SUBD_CALIB;
	s->subdev_flags=SDF_WRITEABLE|SDF_INTERNAL;
	caldac_setup(dev,s);
	s->trig[0]=ni_calib;
	
	/* EEPROM */
	s=dev->subdevices+6;
	s->type=COMEDI_SUBD_MEMORY;
	s->subdev_flags=SDF_READABLE|SDF_INTERNAL;
	s->n_chan=512;
	s->maxdata=0xff;
	s->trig[0]=ni_eeprom;
	
	/* ai configuration */
	ni_ai_reset(dev,dev->subdevices+0);
	win_out(0x1ba0,Clock_and_FOUT_Register);


	/* analog output configuration */
	
	devpriv->ao0p=0x0000;
	ni_writew(devpriv->ao0p,AO_Configuration);
	devpriv->ao1p=0x0100;
	ni_writew(devpriv->ao1p,AO_Configuration);
	win_out(AO_Configuration_Start,Joint_Reset_Register);
	win_out(AO_Disarm,AO_Command_1_Register);
	win_out(0,Interrupt_B_Enable_Register);
	win_out(0x0010,AO_Personal_Register);
	ni_writew(0x3f98,Interrupt_B_Ack);
	win_out(0x1430,AO_Personal_Register);
	win_out(0,AO_Output_Control_Register);
	win_out(0,AO_Start_Select_Register);
	devpriv->ao_cmd1=0;
	win_out(devpriv->ao_cmd1,AO_Command_1_Register);
	devpriv->ao_cmd2=0;
	devpriv->ao_mode1=0;
	devpriv->ao_mode2=0;
	devpriv->ao_mode3=0;
	devpriv->ao_trigger_select=0;

        if(dev->irq){
                win_out((IRQ_POLARITY<<0) |  /* polarity : active high */
                        (0<<1) |  /* no interrupt on 3 pins */
                        (1<<11) |  /* enable int A */
                        (1<<15) |  /* enable int B */
                        (interrupt_pin(dev->irq)<<8) |  /* interrupt output pin A */
                        (interrupt_pin(dev->irq)<<12) ,  /* interrupt output pin B */
                        Interrupt_Control_Register
                );
        }

	printk("\n");

	return 0;
}




/*
	presents the EEPROM as a subdevice
*/

static int ni_eeprom(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int i;
	
	for(i=0;i<it->n_chan;i++)
		it->data[i]=ni_read_eeprom(dev,CR_CHAN(it->chanlist[i]));

	return i;
}

/*
	reads bytes out of eeprom
*/

static int ni_read_eeprom(comedi_device *dev,int addr)
{
	int bit;
	int bitstring;

	bitstring=0x0300|((addr&0x100)<<3)|(addr&0xff);
	ni_writeb_p(0x04,Serial_Command);
	for(bit=0x8000;bit;bit>>=1){
		ni_writeb_p(0x04|((bit&bitstring)?0x02:0),Serial_Command);
		ni_writeb_p(0x05|((bit&bitstring)?0x02:0),Serial_Command);
	}
	bitstring=0;
	for(bit=0x80;bit;bit>>=1){
		ni_writeb_p(0x04,Serial_Command);
		ni_writeb_p(0x05,Serial_Command);
		bitstring|=((ni_readb_p(XXX_Status)&0x01)?bit:0);
	}
	ni_writeb_p(0x00,Serial_Command);
	
	return bitstring;
}

static void ni_write_caldac(comedi_device *dev,int addr,int val);
/*
	calibration subdevice
*/
static int ni_calib(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	ni_write_caldac(dev,CR_CHAN(it->chanlist[0]),it->data[0]);

	return 1;
}

static int pack_mb88341(int addr,int val,int *bitstring);
static int pack_dac8800(int addr,int val,int *bitstring);
static int pack_dac8043(int addr,int val,int *bitstring);
#ifdef PCIMIO
static int pack_ad8522(int addr,int val,int *bitstring);
#endif

struct caldac_struct{
	int n_chans;
	int n_bits;
	int (*packbits)(int,int,int *);
};

static struct caldac_struct caldac_mb88341={ 12, 8, pack_mb88341 };
static struct caldac_struct caldac_dac8800={ 8, 8, pack_dac8800 };
static struct caldac_struct caldac_dac8043={ 1, 12, pack_dac8043 };
#ifdef PCIMIO
static struct caldac_struct caldac_ad8522={ 2, 12, pack_ad8522 };
#endif


static void caldac_setup(comedi_device *dev,comedi_subdevice *s)
{
	int i,j;
	int n_dacs;
	int n_chans=0;
	int n_bits;
	int diffbits=0;
	
	if(!boardtype.caldac[0])return;
	n_bits=boardtype.caldac[0]->n_bits;
	for(i=0;i<3;i++){
		if(!boardtype.caldac[i])break;
		if(boardtype.caldac[i]->n_bits!=n_bits)diffbits=1;
		n_chans+=boardtype.caldac[i]->n_chans;
	}
	n_dacs=i;
	s->n_chan=n_chans;

	if(diffbits){
		int chan;

		s->maxdata_list=kmalloc(sizeof(int)*n_chans,GFP_KERNEL);
		chan=0;
		for(i=0;i<n_dacs;i++){
			for(j=0;j<boardtype.caldac[i]->n_chans;j++){
				s->maxdata_list[chan]=
					(1<<boardtype.caldac[i]->n_bits)-1;
				chan++;
			}
		}
	}else{
		s->maxdata=(1<<boardtype.caldac[0]->n_bits)-1;
	}
}

static void ni_write_caldac(comedi_device *dev,int addr,int val)
{
	int loadbit=0,bits=0,bit,bitstring=0;
	int i;
	
	for(i=0;i<3;i++){
		if(!boardtype.caldac[i])return;
		if(addr<boardtype.caldac[i]->n_chans){
			bits=boardtype.caldac[i]->packbits(addr,val,&bitstring);
			loadbit=SerDacLd(i);
			break;
		}
		addr-=boardtype.caldac[i]->n_chans;
	}

	for(bit=1<<(bits-1);bit;bit>>=1){
		ni_writeb(((bit&bitstring)?0x02:0),Serial_Command);
		ni_writeb(1|((bit&bitstring)?0x02:0),Serial_Command);
	}
	ni_writeb(loadbit,Serial_Command);
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
	
#ifdef PCIMIO
static int pack_ad8522(int addr,int val,int *bitstring)
{
	*bitstring=(val&0xfff)|(addr ? 0xc000:0xa000);
	return 16;
}
#endif



/*
 *
 *  Programmable Function Inputs
 *
 */

#if 0

static void pfi_setup(comedi_device *dev)
{
	/* currently, we don't output any signals, thus, all
	   the PFI's are input */
	
	win_out(IO_Bidirection_Pin_Register,0);
}

#endif


/*
 *
 *  General Purpose Counter/Timer section
 *
 */


/* event counting */

/*
        Initialize the general purpose counter timers. (CLO)

        Everything after the word HACK is a HACK because I don't
        properly understand how to work within the comedi
        architecture!

 */
int gpct_setup(comedi_device *dev,comedi_subdevice *s)
{
	unsigned short tmpreg;	/* For handling strobe writes */
        unsigned short int msb, lsb;
	unsigned short counter_init = 10000;

        /* basic initialization of both counters, from section 4.6.1.3
           of the DAQ-STC manul */

	/* we track several write only register values in software */

	/* G(i)_Reset = 1 (strobe) */
	win_out(G0_Reset,Joint_Reset_Register);
	win_out(G1_Reset,Joint_Reset_Register);

	/* G(i)_Mode_Register = 0 */
	devpriv->gpct_mode0 = 0x0000;
	devpriv->gpct_mode1 = 0x0000;
	win_out(devpriv->gpct_mode0,G_Mode_Register(0));
	win_out(devpriv->gpct_mode1,G_Mode_Register(1));

	/* G(i)_Command_Register = 0 */
	devpriv->gpct_command0 = 0x0000;
        devpriv->gpct_command1 = 0x0000;
	win_out(devpriv->gpct_command0,G_Command_Register(0));
	win_out(devpriv->gpct_command1,G_Command_Register(1));

	/* G(i)_Input_Select_Register = 0 */
	devpriv->gpct_input_select0 = 0x0000;
	devpriv->gpct_input_select1 = 0x0000;
	win_out(devpriv->gpct_input_select0,G_Input_Select_Register(0));
	win_out(devpriv->gpct_input_select1,G_Input_Select_Register(1));

	/* G(i)_Autoincrement = 0 (write) */
	/* G(i)_Autoincrement_Register = 0 */
	win_out(G_Autoincrement(0x00),G_Autoincrement_Register(0));
	win_out(G_Autoincrement(0x00),G_Autoincrement_Register(1));

	/* XXX - for now we ignore interrupts */
	/* G(i)_TC_Interrupt_Enable = 0 (write)*/
	/* G(i)_Gate_Interrupt_Enable = 0 (write) */
	/* win_out(0x0000,Interrupt_A_Enable_Register); */
	/* win_out(0x0000,Interrupt_B_Enable_Register); */

	/* G(i)_Synchronized_Gate = 1 (write) */
	devpriv->gpct_command0 |= G0_Synchronized_Gate;
	devpriv->gpct_command1 |= G1_Synchronized_Gate;
	win_out(devpriv->gpct_command0,G_Command_Register(0));
	win_out(devpriv->gpct_command1,G_Command_Register(1));

	/* XXX - for now we leave this in, but perhaps we could do without
           if it causes problems elsewhere? */
	/* G(i)_Gate_Error_Confirm = 1 (strobe) */
	/* G(i)_TC_Error_Confirm = 1 (strobe) */
	/* G(i)_TC_Interrupt_Ack = 1 (strobe) */
	/* G(i)_Gate_Interrupt_Ack = 1 (strobe) */
	win_out(G0_Gate_Error_Confirm|G0_TC_Error_Confirm|G0_TC_Interrupt_Ack|
		G0_Gate_Interrupt_Ack,Interrupt_A_Ack_Register);
	win_out(G1_Gate_Error_Confirm|G1_TC_Error_Confirm|G1_TC_Interrupt_Ack|
		G1_Gate_Interrupt_Ack,Interrupt_B_Ack_Register);

	/********************************************************************/

	/* HACK - What follows is a hack.  This puts counter #0 in
           "relative position sensing" mode and then arms it */
  
	/* G(i)_Load_Source_Select = 0 (write) */
	devpriv->gpct_mode0 &= ~G0_Load_Source_Select;
	win_out(devpriv->gpct_mode0,G_Mode_Register(0));

	/* G(i)_Load_A = initial counter value (write) */
	msb = counter_init>>16;
	lsb = counter_init - msb;
	win_out(msb,G_Load_A_Register_High(0));
	win_out(lsb,G_Load_A_Register_Low(0));

	/* FLUSH */

	/* G(i)_Load = 1 (strobe) */
	tmpreg = devpriv->gpct_command0 | G0_Load;
	win_out(tmpreg,G_Command_Register(0));

	/* FLUSH */

	/* G(i)_Source_Select = PFI0 (write) */
	devpriv->gpct_input_select0 &= (0xffff & G_Source_Select(0x00));
	devpriv->gpct_input_select0 |= G_Source_Select(0x01);

	/* G(i)_Source_Polarity = 0 (count rising edges) (write) */
	devpriv->gpct_input_select0 &= ~G0_Source_Polarity;

	/* G(i)_Gate_select = 0 (write) */
	devpriv->gpct_input_select0 &= (0xffff & G_Gate_Select(0x00));
	devpriv->gpct_input_select0 |= G_Gate_Select(0x00);

	/* G(i)_OR_Gate = 0 (write) */
	devpriv->gpct_input_select0 &= ~G0_OR_Gate;

	/* G(i)_Output_Polarity = 0 (write) */
	devpriv->gpct_input_select0 &= ~G0_Output_Polarity;

	/* G(i)_Gate_Select_Load_Source = 0 (write) */
	devpriv->gpct_input_select0 &= ~G0_Gate_Select_Load_Source;

	/* G(i)_Gate_Polarity = 0 (write) */
	devpriv->gpct_mode0 &= ~G0_Gate_Polarity;

	/* G(i)_Output_Mode = 1 (one clock cycle output) (write) */
	devpriv->gpct_mode0 &= (0xffff & G_Output_Mode(0x00));
	devpriv->gpct_mode0 |= G_Output_Mode(0x01);

	/* G(i)_Reload_Source_Switching = 1 (write) */
	devpriv->gpct_mode0 |= G0_Reload_Source_Switching;

	/* G(i)_Loading_On_Gate = 0 (write) */
	devpriv->gpct_mode0 &= ~G0_Loading_On_Gate;

	/* G(i)_Gating_Mode = 2 (write) */
	devpriv->gpct_mode0 &= (0xffff & G_Gating_Mode(0x00));
	devpriv->gpct_mode0 |= G_Gating_Mode(0x02);

	/* G(i)_Gate_On_Both_Edges = 0 (write) */
	devpriv->gpct_mode0 &= ~G0_Gate_On_Both_Edges;

	/* G(i)_Trigger_Mode_For_Edge_Gate = 3 (write) */
	devpriv->gpct_mode0 &= (0xffff & G_Trigger_Mode_For_Edge_Gate(0x00));
	devpriv->gpct_mode0 |= G_Trigger_Mode_For_Edge_Gate(0x03);

	/* G(i)_Stop_Mode = 0 */
	devpriv->gpct_mode0 &= (0xffff & G0_Stop_Mode(0x00));
	devpriv->gpct_mode0 |= G0_Stop_Mode(0x00);

	/* G(i)_Counting_Once = 0 (write) */
	devpriv->gpct_mode0 &= (0xffff & G0_Counting_Once(0x00));
	devpriv->gpct_mode0 |= G0_Counting_Once(0x00);

	/* G(i)_Up_Down = 2 (hardware controlled) (write) */
	devpriv->gpct_command0 &= (0xffff & G_Up_Down(0x00));
	devpriv->gpct_command0 |= G_Up_Down(0x02);

	/* G(i)_Bank_Switch_Enable = 0 (write) */
	devpriv->gpct_command0 &= ~G0_Bank_Switch_Enable;

	/* G(i)_Bank_Switch_Mode = 0 (write) */
	devpriv->gpct_command0 &= ~G0_Bank_Switch_Mode;

	/* XXX - for now we ignore interrupts */
	/* G(i)_TC_Interrupt_Enable = 0 (write) */
	/* win_out(0x0000,Interrupt_A_Enable_Register); */

	/* XXX - for now we ignore interrupts */
	/* G(i)_Gate_Interrupt_Enable = 0 (write) */
	/* win_out(0x0000,Interrupt_A_Enable_Register); */

	/* actually write out the registers */
	win_out(devpriv->gpct_input_select0,G_Input_Select_Register(0));
	win_out(devpriv->gpct_mode0,G_Mode_Register(0));
	win_out(devpriv->gpct_command0,G_Command_Register(0));

	/********************************************************************/

	/* HACK - What follows continues my hack of configuring the
           counter in a specific mode.  Arming the counter instructs
           it to start running with our configuration */

	/* Arm the counter 0 (stobe) */
	tmpreg = devpriv->gpct_command0 | G0_Arm;
	win_out(tmpreg,G_Command_Register(0));

	return 0;
}

#if 0
int gpct_start(comedi_device *dev,int chan)
{
	/* select load source */
	Gi_Load_Source_Select = 0;
	
	/* load initial counter value */
	Gi_Load_A = 0;
	
	/* strobe */
	Gi_Load=1;

/* command reg */
	Gi_Up_Down = 1; /* up counting */
	Gi_Bank_Switch_Enable = 0;
	Gi_Bank_Switch_Mode = 0;

/* input select reg */
	Gi_Source_Select = PFIn;
	Gi_Source_Polarity = 0; /* rising edges */
	Gi_Gate_Select = 31; /* logic low */
	Gi_OR_Gate = 0;
	Gi_Output_Polarity = 1; /* active low */
	Gi_Gate_Select_Load_Source = 0;

/* mode reg */
	Gi_Gate_Polarity = 0; /* disable inversion */
	Gi_Loading_On_Gate = 0;
	Gi_Output_Mode = 1; /* one clock cycle output */
	Gi_Loading_On_TC = 0;
	Gi_Gating_Mode = 1;
	Gi_Gate_On_Both_Edges = 0;
	Gi_Trigger_Mode_For_Edge_Gate = 2;
	Gi_Stop_Mode = 0;
	Gi_Counting_Once = 0;
	
	
/* int enable reg -- ignore */
	
/*
	Gi_TC_Interrupt_Enable = 0;
	Gi_Gate_Interrupt_Enable = 0;
*/
}

static int gpct_sp(comedi_device *dev,comedi_param *it)
{
	switch(it->pnum){
	case COMEDI_CTR_INPUT:
	{
		/* which input source? */
		int pval=it->pval;
		
		if(pval<0 || pval>=10)return -EINVAL;
		changeparam(&dev->chinfo[it->chan].paramlist,COMEDI_CTR_INPUT,
			0,pval);
		/* XXX set it */
		return 0;
	}
	case COMEDI_CTR_POLARITY:
		/* count rising or falling edges? */
		if(it->pval<0 || it->pval>=4)return -EINVAL;
		changeparam(&dev->chinfo[it->chan].paramlist,COMEDI_AREF,
			0,it->pval);
		/* XXX set it */
		return 0;
	case COMEDI_COUNT:
		changeparam(&dev->chinfo[it->chan].paramlist,COMEDI_COUNT,
			0,it->pval);  /* irrelevant... */
		devpriv->gpct[it->chan-boardtype->counteroffset]=it->pval;
		return 0;
	}
	return -EINVAL;
}
#endif

static int ni_8255_callback(int dir,int port,int data,void *arg)
{
	comedi_device *dev=arg;

	if(dir){
		ni_writeb(25+2*port,data);
		return 0;
	}else{
		return ni_readb(25+2*port);
	}
}
