/*
    module/ni_F.c
    hardware driver for National Instruments AT-MIO-16F

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1995 Claus Schroeter <clausi@chemie.fu-berlin.de>
    Copyright (C) 1999 David A. Schleef <ds@stm.lbl.gov>

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
	This driver is an adaptation of one written by clausi
*/

#include <comedi_module.h>
#include <asm/io.h>
#include <linux/ioport.h>
#include <linux/delay.h>

#define ATMIO16F_SIZE 16

#define Command_Register_1		0x00	/* wo */
#define Command_Register_2		0x02	/* wo */
#define Command_Register_3		0x04	/* wo */
#define Command_Register_4		0x06	/* wo */
#define Status_Register_1		0x18	/* ro */
#define Status_Register_2		0x1a	/* ro */

#define ADC_FIFO_Register		0x00	/* ro */
#define CONFIGMEM_Register		0x08	/* wo */

#define DAC0_Register			0x10	/* wo */
#define DAC1_Register			0x12	/* wo */

#define CONFIGMEMCLR_Register		0x1b	/* ro 8 */
#define CONFIGMEMLD_Register		0x1b	/* wo 8 */
#define DAQ_Clear_Register		0x19	/* ro 8 */
#define DAQ_Start_Register		0x1d	/* ro 8 */
#define Single_Conversion_Register	0x1d	/* wo 8 */
#define ADC_Calibration_Register	0x1f	/* wo 8 */

#define TMRREQ_Clear_Register		0x1f	/* ro 8 */
#define DAC_Update_Register		0x18	/* wo */
#define DAC_Clear_Register		0x1e	/* ro 8 */

#define DMA_Channel_Clear_Register	0x0b	/* ro 8 */
#define DMATCA_Clear_Register		0x19	/* wo 8 */
#define DMATCB_Clear_Register		0x09	/* ro 8 */
#define External_Strobe_Register	0x1e	/* wo 8 */
#define Calibration_DAC_0_Load_Register	0x0a	/* wo 8 */
#define Calibration_DAC_1_Load_Register	0x1a	/* wo 8 */

#define Am9513A_Data_Register		0x14	/* rw */
#define Am9513A_Command_Register	0x16	/* wo */
#define Am9513A_Status_Register		0x16	/* ro */

#define Digital_Input_Register		0x1c	/* ro */
#define Digital_Output_Register		0x1c	/* wo */

#define RTSI_Switch_Shift_Register	0x0c	/* wo 8 */
#define RTSI_Switch_Strobe_Register	0x0e	/* wo 8 */


#define EEPROMCS	_bit15
#define SDATA		_bit14
#define SCLK		_bit13
#define SCANDIV		_bit12
#define INTGATE		_bit10
#define RETRIG_DIS	_bit9
#define DAQEN		_bit8
#define SCANEN		_bit7
#define SCN2		_bit6
#define CNT32		_bit5
#define RTSITRIG	_bit4

#define A4RCV		_bit15
#define A4DRV		_bit14
#define A2RCV		_bit13
#define A2DRV		_bit12
#define BIPDAC1		_bit11
#define BIPDAC0		_bit10
#define EXTREFDAC1	_bit9
#define EXTREFDAC0	_bit8
#define EISA_DMA	_bit7
#define DMACHBB		_bits543
#define DMACHAB		_bits210



/* stuff i don't know */
#define MIO_ADCLR 0
#define BIPOLAR 0
#define DIFF 0
#define MIO_SCONV 0
#define MIO_SR 0
#define FIFOEF 0
#define MIO_ADFIFO 0
#define mio_reset_bitcmd(a,b) /* */
#define mio_set_bitcmd(a,b) /* */



#define DB_ERROR      (1<<0)
#define DB_OVERFLOW   (1<<1)
#define DB_HALF       (1<<2)
#define DB_EMPTY      (1<<3)

#define DAQ_RUNNING (1<<0)
#define DAQ_DACUP   (1<<1)
#define DAQ_SCONV   (1<<2)
#define DAQ_BIPOLAR (1<<3)
#define DAQ_DIFF    (1<<4)
#define DAQ_SYNC    (1<<5)


typedef struct {
	int adc;
	int dac0;
} AdcVal;

typedef struct{
	int DaqMode;
	int CurrentDac;
	int WgenStat;
	int admode[16];
}ni_F_private;
#define devpriv ((ni_F_private *)dev->private)

int atmio16f_rem(comedi_device *dev);

#if 0
static void mio_SetDaqMode(int mode)
{
	devpriv->DaqMode = mode;
}
#endif

/**********************************************************************/
static int atmio16f_ai(comedi_device * dev, comedi_subdevice *s, comedi_trig * it)
{
	int chan,aref,range;
	int i;

	chan=CR_CHAN(it->chanlist[0]);
	range=CR_RANGE(it->chanlist[0]);
	aref=CR_AREF(it->chanlist[0]);

	outw(0x00, dev->iobase + MIO_ADCLR);

	/* set ADC UNIPOLAR/BIPOLAR mode */
	if (range)
		mio_reset_bitcmd(BIPOLAR, 0);
	else
		mio_set_bitcmd(BIPOLAR, 0);

	/* set ADC DIFF/RSE mode */
	if (aref)	/* XXX check */
		mio_set_bitcmd(DIFF, 0);
	else
		mio_reset_bitcmd(DIFF, 0);

	outw(0x00, dev->iobase + MIO_SCONV);

	/* poll status register */
	for(i=0;i<25;i++){
		if( inw(dev->iobase + MIO_SR) & FIFOEF )
			break;
		udelay(5);
	}
	if(i==25){
		comedi_error(dev,"timeout");
		return -ETIME;
	}

	/* read value */
	it->data[0] = inw(dev->iobase + MIO_ADFIFO);

	return 1;
}


#if 0
/***********************************************************************
 *
 *  Doublebuffer FIFO mechanism
 *
 */

#define MAX_DBUFSIZE 1024

struct wait_queue *DaqWait = NULL;
struct semaphore DaqSem = MUTEX;


unsigned int DblBuf[2][MAX_DBUFSIZE + 1];

unsigned int DblBufState = 0;

unsigned int ActualReadCount = 0;
unsigned int ActualWriteCount = 0;
unsigned int ActualResidue = 0;

unsigned int ActualWrite = 0;
unsigned int ActualRead = 0;

/***********************************************************************/
void mio_DaqInit(void)
{

	DblBufState = 0;
	ActualResidue = 0;
	ActualWriteCount = 0;
	ActualReadCount = 0;

	ActualRead = 0;
	ActualWrite = 0;
	DaqMode = 0;

	/*clear A/D circuit */
	outw(0x0000, MIO_ADCLR);

}


/**********************************************************************/
void mio_SwapWrite(void)
{
	ActualWrite = (ActualWrite ? 0 : 1);
	DBGprint(DBG_DATA, ("Swapped Write to %d", ActualWrite));
	ActualWriteCount = 0;
	if (ActualResidue >= (2 * MAX_DBUFSIZE)) {	/* overflow buffer bit set */
		DBGprint(DBG_DATA, ("Buffer Overflow"));
		DblBufState |= (DB_ERROR | DB_OVERFLOW);
	}
}

/**********************************************************************/
void mio_SwapRead(void)
{
	ActualRead = (ActualRead ? 0 : 1);
	DBGprint(DBG_DATA, ("Swapped Read to %d", ActualRead));
	ActualReadCount = 0;
}

/**********************************************************************/
void mio_AddDBuf(unsigned int val)
{
	if (ActualWriteCount < MAX_DBUFSIZE) {
		DblBuf[ActualWrite][ActualWriteCount] = val;
		ActualWriteCount++;
		ActualResidue++;
	} else {
		mio_SwapWrite();
	}
	if (ActualResidue >= MAX_DBUFSIZE) {
		DblBufState |= DB_HALF;
		if (DaqMode & DAQ_SYNC)
			wake_up_interruptible(&DaqWait);
	}
}

/**********************************************************************/
unsigned int mio_GetDBuf(void)
{
	unsigned int ret = 0;

	if (ActualResidue > 0) {
		if (ActualReadCount < MAX_DBUFSIZE) {
			ret = DblBuf[ActualRead][ActualReadCount];
			ActualReadCount++;
			ActualResidue--;
		} else {
			mio_SwapRead();
		}
		if (ActualResidue < MAX_DBUFSIZE)
			DblBufState &= ~DB_HALF;
	} else {
		ret = 0;
		DblBufState |= DB_EMPTY;
	}

	DBGprint(DBG_DATA, ("residue=%d", ActualResidue));
	return ret;
}

/**********************************************************************/
int mio_SyncRead(int *buf, int count)
{
	int retval, i;
	int dummy;
	char *tbuf;


	if (retval = verify_area(VERIFY_WRITE, buf, count))
		return retval;


	if (DblBufState & DB_EMPTY) {
		DBGprint(DBG_DATA, ("Buffer Empty"));
		return -EINTR;
	}
	if (!(DblBufState & DB_HALF) && (DaqMode & DAQ_RUNNING)) {
		DBGprint(DBG_DATA, ("\nWaiting for DB_HALF"));
		DaqMode |= DAQ_SYNC;
		interruptible_sleep_on(&DaqWait);
		DaqMode &= ~DAQ_SYNC;
		DBGprint(DBG_DATA, ("\nGot DB_HALF"));
	}
	if (DblBufState & DB_OVERFLOW) {
		DBGprint(DBG_DATA, ("Double Buffer Overflow"));
		return -EOVERFLOW;
	}
	/* copy block to FS */
	for (i = 0; i < count && i < ActualResidue; i++) {
		dummy = mio_GetDBuf();
		DBGprint(DBG_DATA, ("buf=%d", dummy));
		if (DblBufState & DB_EMPTY) {
			DBGprint(DBG_DATA, ("count=%d", i));
			return i;
		}
		put_fs_long(dummy, (int *) &(buf[i]));
		/*memcpy_tofs((int *) buf, &dummy ,sizeof(int)); */
	}

	return count;

}

/**********************************************************************/
void mio_DaqFinish(void)
{

	DaqMode &= ~DAQ_RUNNING;
	if (DaqMode & DAQ_SYNC)
		wake_up_interruptible(&DaqWait);
}

/**********************************************************************/
void mio_DaqStart(int mode)
{

	DBGprint(DBG_DATA, ("mode (0x%x)", mode));

	mio_DaqInit();

	DaqMode |= (DAQ_RUNNING | mode);

	/*clear A/D circuit */
	outw(0x0000, MIO_ADCLR);

	/* set ADC UNIPOLAR/BIPOLAR mode */
	if (DaqMode & DAQ_BIPOLAR)
		mio_reset_bitcmd(BIPOLAR, 0);
	else
		mio_set_bitcmd(BIPOLAR, 0);

	/* set ADC DIFF/RSE mode */
	if (DaqMode & DAQ_DIFF)
		mio_set_bitcmd(DIFF, 0);
	else
		mio_reset_bitcmd(DIFF, 0);

	if (DaqMode & DAQ_SCONV) {
		DBGprint(DBG_DATA, ("trigger conversion"));
		outw(0x00, MIO_SCONV);	/* trigger first value */
		while (!(inw(MIO_SR) & FIFOEF));	/* sync FIFOEF */
	}
	if (DaqMode & DAQ_DACUP)
		printk("WGEN MODE\n");

}

/***********************************************************************/
void mio_DaqIRQ(int status)
{
	comedi_sample s;

	if (DaqMode & DAQ_RUNNING) {

		/* read FIFO content into Buffer */
		s.chan = dev->adchan;
		s.job = dev->job;
		while (inw(MIO_SR) & FIFOEF) {

			it->data = inw(dev->iobase + MIO_ADFIFO);

		}

		/* trigger next conversion */
		if (DaqMode & DAQ_SCONV) {
			outw(0x0000, dev->iobase + MIO_SCONV);
		}
		if ((DaqMode & DAQ_DACUP) && !(WgenStat & WGEN_RUNNING)) {
#if DEBUG
			if (dbgMask)
				printk("Finish DACUP DAQ Cycle\n");
#endif
			DaqMode &= ~DAQ_RUNNING;
			while (!(inw(MIO_SR) & FIFOEF));	/* poll status */
			val = inw(MIO_ADFIFO) | (CurrentDac << 16);
			mio_AddDBuf(val);

			if (DaqMode & DAQ_SYNC)
				wake_up_interruptible(&DaqWait);
		}
	} else
		printk("DAQ-INTERRUPT unknown status (0x%x)", status);

}

#endif

int atmio16f_ao(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	return -EINVAL;
}

int atmio16f_di(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	return -EINVAL;
}

int atmio16f_do(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	return -EINVAL;
}


int atmio16f_init(comedi_device *dev,comedi_devconfig *it)
{
	int ret=0;
	comedi_subdevice *s;
	
	if(strcmp(it->board_name,"at-mio-16f-5"))
		return 0;
	
	dev->driver_name="atmio16f";
	dev->board_name="at-mio-16f";
	if(it->options[0])
		dev->iobase=it->options[0];
	else return -EINVAL;
	
	printk("comedi: atmio16f: 0x%04x\n",dev->iobase);
	if(check_region(dev->iobase,ATMIO16F_SIZE)<0){
		comedi_error(dev,"I/O port conflict");
		return -EIO;
	}
	request_region(dev->iobase,ATMIO16F_SIZE,"atmio16f");
	dev->iosize=ATMIO16F_SIZE;
	
	dev->n_subdevices=4;
	
	if((ret=alloc_subdevices(dev))<0)
		goto cleanup;
	if((ret=alloc_private(dev,sizeof(ni_F_private)))<0)
		goto cleanup;
	
	s=dev->subdevices+0;
	/* ai subdevice */
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=16;
	s->maxdata=0xfff;
	s->range_type=RANGE_unknown;
	s->trig[0]=atmio16f_ai;

	s++;
	/* ao subdevice */
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=2;
	s->maxdata=0xfff;
	s->range_type=RANGE_unknown;
	s->trig[0]=atmio16f_ao;

	s++;
	/* di subdevice */
	s->type=COMEDI_SUBD_DI;
	s->subdev_flags=SDF_READABLE;
	s->n_chan=8;
	s->maxdata=1;
	s->range_type=RANGE_digital;
	s->trig[0]=atmio16f_di;

	s++;
	/* do subdevice */
	s->type=COMEDI_SUBD_DO;
	s->subdev_flags=SDF_WRITEABLE;
	s->maxdata=1;
	s->n_chan=8;
	s->range_type=RANGE_digital;
	s->trig[0]=atmio16f_do;

	/* do some init */

	dev->rem=atmio16f_rem;

	return 1;
cleanup:
	atmio16f_rem(dev);

	return ret;
}


int atmio16f_rem(comedi_device *dev)
{

	if(dev->iobase)
		release_region(dev->iobase,dev->iosize);

	return 0;
}

