/*
 * Copyright (C) 1995,1996  Sam Moore, Warren Jasper
 *               1999       David Schleef
 * All rights reserved.
 *
 * This software may be freely copied, modified, and redistributed
 * provided that this copyright notice is preserved on all copies.
 *
 * There is no warranty or other guarantee of fitness of this software
 * for any purpose.  It is provided solely "as is".
 *
 */
/*
 * The comedi version of this driver was modified from the original
 * by David Schleef.
 */

#include <comedi_module.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/mm.h>
#include <linux/timer.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/malloc.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <asm/io.h>
#include <asm/system.h>

#define DEBUG

/*
 *   Note: Interrupts, when enabled, are generated differently depending
 *   upon the mode of the board:
 *   Compatibility mode: Interrupts generated every A/D conversion
 *                       from either pacer clock or soft trigger.
 *   Enhanced mode:      Interrupts generated when FIFO half full
 *                       or when Total_Count = 0.
 */

/* DT Connection is not supported */

#define DT_CONNECT FALSE


#define AD_CHANNELS         16	/* 16 Single Ended or 8 Diff.  */
#define DIO_CHANNEL         16	/* dio minor channel number    */

#define PORT_SIZE        0x14	/* uses 20 bytes of registers                 */


#define LSB_AND_CHNLS    0	/* A/D Data & Channel Register      */
#define MSB_DATA_BYTE    1
#define MUX_SCAN_LIMITS  2	/* Channel Mux Scan Limits Register */
#define DIO_REG          3	/* 4 Bit Digital I/O Register       */
#define STATUS_REG       8	/* Status Register                  */
#define CNTRL_REG        9	/* DMA, Interrupt & Trigger Control */
#define PACER_CLOCK_REG  10	/* Pacer Clock Control Register     */
#define MISC_REG         11	/* Analog Input Range Reg., ETC     */

#define COUNTERA_0_DATA  12	/* Pacer Clock A Counter 0 Register */
#define COUNTERA_1_DATA  13	/* Pacer Clock A Counter 1 Register */
#define COUNTERA_2_DATA  14	/* Pacer Clock A Counter 2 Register */
#define COUNTERA_CONTROL 15	/* Pacer Clock A Control  Register  */

#define COUNTERB_0_DATA  16	/* Pacer Clock B Counter 0 Register */
#define COUNTERB_1_DATA  17	/* Pacer Clock B Counter 1 Register */
#define COUNTERB_2_DATA  18	/* Pacer Clock B Counter 2 Register */
#define COUNTERB_CONTROL 19	/* Pacer Clock B Control  Register  */


/*************************************************************************
* STATUS_REG         base_reg+8             Status Register              *
**************************************************************************/
#define ADSR_INT        0x10	/* INT=1 External pulse received, INT=0 waiting */
#define ADSR_MUX        0x20	/* MUX=1 16 Single Ended, MUX=0 8 Differential  */
#define ADSR_UB         0x40	/* U/B=1 Unipolar mode, U/B=0 Bipolar mode      */
#define ADSR_EOC        0x80	/* EOC=1, A/D is busy, EOC=0 free               */

/*************************************************************************
* CNTRL_REG          base_reg+9        DMA, Interrupt & Trigger Control  *
**************************************************************************/
#define CNTL_TS0       0x1	/* TS0=0 External Trigger, TS0=1 Pacer Clock */
#define CNTL_TS1       0x2	/* TS1=0 Software triggered A/D only         */
#define CNTL_DMA       0x4	/* DMA Disabled = 0, DMA Enabled = 1         */
#define CNTL_INTE      0x80	/* Interrupts Disabled = 0, Enabled = 1      */

/*************************************************************************
* MISC_REG        base_reg+11         Range, and Exteded Features        *
**************************************************************************/

#define ENHANCED_BIT  (0x10)	/* Enhanced Mode Enabled = 1, Disabled = 0 */
#define MISC_OVERRUN  (0x20)	/* No overrun = 0, FIFP buffer full = 1    */
#define MISC_PRETRIG  (0x40)	/* Pretrigger Enabled = 1, Disable = 0     */
#define MISC_DT       (0x80)	/* DT-Connect Enable = 1, Disable = 0      */

/*************************************************************************
* Constants for dealing with the 8254 counters                           *
**************************************************************************/

#define MODE0 0x0
#define MODE1 0x2
#define MODE2 0x4
#define MODE3 0x6
#define MODE4 0x8
#define MODE5 0xa

#define C0 0x00
#define C1 0x40
#define C2 0x80

//#define LATCH    0x00
#define LSBONLY  0x10
#define MSBONLY  0x20
#define LSBFIRST 0x30

#define S0       0x00
#define S1       0x02

#define PACKETSIZE  512
#define MAX_COUNT   16384
/* Interrupt context data. Controls interrupt service handler */

typedef struct{
	unsigned int ir;	/* shifted bits for board 0-7, 10->0, 11->1 */
	unsigned int misc_reg;
} das16_private ;
#define devpriv ((das16_private *)dev->private)

/* Values kept for each channel. */

#if 0
typedef struct CHANNEL_REC {
	int open;		/* Is channel device open()                 */
	int adc_dio_pretrig;	/* TRUE = write dio before sampling         */
	LONG count;		/* Number of samples requested              */
	BYTE dio_value;		/* DIO value to be written before sampling  */
	BYTE mode;		/* 0 =  Soft Trigger                        */
	/* 2 = External Trigger                     */
	/* 4 = Pacer Clock                          */
	/* 6 = Pacer Clock External Trig Enable     */
	BYTE gain;		/* Voltage range                            */
	BYTE lowChan;		/* Starting Channel of MUX scan limit       */
	BYTE hiChan;		/* Ending Channel of MUX scan limit         */
} ChanRec;

#endif






#if 0
static WORD base_reg = PORT_MIN;	/* base register address */
static int WordsToRead;		/* number of conversions until done  */
static ChanRec *CurrChan = NULL;	/* pointer to Chan[minor]            */
static int MajorNumber = DEFAULT_MAJOR_DEV;	/* Major number compiled in          */
static ChanRec Chan[DIO_CHANNEL + 1];	/* Channel specific information      */
static BoardRec BoardData;	/* Board specific information        */
static WORD KernBuff[MAX_COUNT];	/* Kernel buffer where samples are   */
					    /* saved before write to user space  */
					    /* structure for timer data          */
static WORD *KernBuffPtr;	/* pointer to kernbuf                */
struct wait_queue *adc0_wait;	/* wait semaphore                    */
struct timer_list TimerList =
{NULL, NULL, 0, 0, adc0_TimerHandler};
#endif

int irq_list[]={ -1, -1, 2, 3, 4, 5, 6, 7, -1, -1, 0, 1, -1, -1, -1, -1 };

static int compatibility_read(comedi_device *dev,comedi_trig *it);
static void release_resources(comedi_device *dev);

static int das16_attach(comedi_device *dev,comedi_devconfig *it);
static int das16_detach(comedi_device *dev);
comedi_driver driver_das16={
	driver_name:	"das16",
	module:		&__this_module,
	attach:		das16_attach,
	detach:		das16_detach,
};


/*
   interrupt routine

 */

static void das16_interrupt(int irq,void *d,struct pt_regs *regs)
{

}


/*
   AD subsystem

 */

static void set_gain(comedi_device * dev, int gain)
{
	devpriv->misc_reg &= 0xf0;
	devpriv->misc_reg |= (gain << 4);
	outb_p(devpriv->misc_reg, dev->iobase + MISC_REG);
}

static int set_channel_mux(comedi_device * dev, int first_chan, int last_chan)
{
	int channel;

	channel = (last_chan << 4) | first_chan;
	outb_p(channel, dev->iobase + MUX_SCAN_LIMITS);

	while ((inb_p(dev->iobase + MISC_REG) & MISC_OVERRUN))
		/* spin */;

	return 0;
}

static int das16_ai_mode0(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int bReg;

	/* Put in Compatibility Mode here just to be safe */
	bReg = inb(dev->iobase + MISC_REG);
	outb(bReg & ~ENHANCED_BIT, dev->iobase + MISC_REG);

	/* Set modes for this channel */
	set_gain(dev, CR_RANGE(it->chanlist[0]));

	set_channel_mux(dev, CR_CHAN(it->chanlist[0]), CR_CHAN(it->chanlist[0]));

	compatibility_read(dev,it);

	return 1;
}


/*
   DIO subsystem

 */

static int das16_dio(comedi_device * dev, comedi_subdevice * s, comedi_trig * it)
{
	int data;

	data = inb(dev->iobase + DIO_REG);

	return di_unpack(data, it);
}

/*
   AO subsystem

 */

/***************************************************************************
 *
 * Loads driver. Called when "insmod adc.o" is invoked on the command line.
 *               The board is set to IRQ, 
 *
 ***************************************************************************/

static int das16_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int iobase,irq;
	int ret=0;
	int chan;

	iobase=it->options[0];
	if (check_region(iobase, PORT_SIZE) != 0) {	/* in use */
		printk("comedi%d: das16: Can't allocate region port address 0x%x\n",
		       dev->minor,iobase);
		return -EIO;
	}
	request_region(iobase, PORT_SIZE, "das16");
	dev->iobase=iobase;

#if 0
	/* we don't really want to search for boards... */
	/* Look to see if ADC is installed */
	for (base_reg = PORT_MIN; base_reg <= PORT_MAX; base_reg += PORT_STEP) {
		if (check_region(base_reg, PORT_SIZE) == 0) {	/* not in use */
			request_region(base_reg, PORT_SIZE, "adc");
			if (adc_find(base_reg)) {	/* found the board */
				BoardData.base = base_reg;
				break;
			} else {
				release_region(base_reg, PORT_SIZE);
			}
		}
	}

	if (base_reg > PORT_MAX) {
		printk("%s: No boards found from address %#x to %#x.\n",
		       ADAPTER_ID, PORT_MIN, PORT_MAX);
		if (unregister_chrdev(MajorNumber, "adc") != 0) {
			printk("%s: unregister_chrdev() failed.\n", ADAPTER_ID);
		}
		return -ENODEV;
	}
#endif

	/* Register interrupt handler. */

	irq=it->options[1];

	if(irq<0 || irq>=16 || irq_list[irq]<0){
		return -EINVAL;
	}
	if (request_irq(irq, das16_interrupt, SA_INTERRUPT, "das16", dev) == 0) {
		return -EIO;
	}
	dev->irq=irq;

#if 0
	printk("%s: address=%#x IRQ=%d.", ADAPTER_ID, BoardData.base, BoardData.irq);
	printk(" 4/27/95 wjasper@tx.ncsu.edu sam@tx.ncsu.edu\n");
#endif

	if((ret=alloc_private(dev,sizeof(das16_private)))<0)
		return ret;

	dev->n_subdevices=3;

	if((ret=alloc_subdevices(dev))<0)
		return ret;
	
	s=dev->subdevices;

	if (inb(dev->iobase+STATUS_REG) & ADSR_MUX) {
		chan = 16;
	} else {
		chan = 8;
	}
	/* ai subdevice */
	s->type=COMEDI_SUBD_AI;
	s->n_chan=8;
	s->trig[0]=das16_ai_mode0;
	s->maxdata=0xfff;

	s++;
	/* ao subdevice */
	s->type=COMEDI_SUBD_AO;
	s->n_chan=2;
#if 0
	s->trig[0]=das16_ao;
#endif
	s->maxdata=0xfff;

	s++;
	/* dio subdevice */
	s->type=COMEDI_SUBD_DIO;
	s->n_chan=8;
	s->trig[0]=das16_dio;
	s->maxdata=1;


	/* Set interrupt level on board  2-7, 10->0 and 11->1 */
	devpriv->ir = irq_list[irq] << 4;

	/* ... clearing INTE bit in control reg */
	outb(devpriv->ir, dev->iobase+CNTRL_REG);

	outb(0x0, dev->iobase+MISC_REG);	/* Put in compatibility mode */

	printk("\n");

	return 0;
}


#if 0
/***************************************************************************
 *
 * Test memory locations for exsistance of adc board.
 *
 ***************************************************************************/
static int adc_find(WORD base_reg)
{

/*
   To test if there is a DAS 16/330 board do the following:
   1. write MUX_SCAN_LIMITS register.
   2. read MUX_SCAN_LIMITS register. If fail return FALSE.
   3. read STATUS_REG (bits 0-3).  If fail return FALSE.
 */

	BYTE bReg = 0x0;

	bReg = 0xa3;		/* set mux from channel 3 to channel 10 */
	outb_p(bReg, MUX_SCAN_LIMITS);	/* write to register */
	if (inb_p(MUX_SCAN_LIMITS) != 0xa3)
		return FALSE;	/* not a 16/330 board */
	if ((inb_p(STATUS_REG) & 0xf) != 0x3)
		return FALSE;	/* not a 16/330 board */
	bReg = 0xd2;		/* set mux from channel 2 to channel 13 */
	outb_p(bReg, MUX_SCAN_LIMITS);	/* write to register */
	if (inb_p(MUX_SCAN_LIMITS) != 0xd2)
		return FALSE;	/* not a 16/330 board */
	if ((inb_p(STATUS_REG) & 0xf) != 0x2)
		return FALSE;	/* not a 16/330 board */
	return TRUE;		/* found it!          */
}
#endif

/***************************************************************************
 *
 * Remove driver. Called when "rmmod adc" is run on the command line.
 *
 ***************************************************************************/

static void release_resources(comedi_device *dev)
{
	if(dev->irq){
		free_irq(dev->irq,dev);
	}

	if(dev->iobase)
		release_region(dev->iobase, PORT_SIZE);

}

static int das16_detach(comedi_device *dev)
{
	release_resources(dev);

	return 0;
}

#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_das16);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_das16);
}
#endif

#if 0
static int adc_open(struct inode *iNode, struct file *filePtr)
{
	int minor = MINOR(iNode->i_rdev);

	MOD_INC_USE_COUNT;

	/* 
	   check if device is already open: only one process may read from a
	   port at a time.  There is still the possibility of two processes 
	   reading from two different channels messing things up. However,
	   the overhead to check for this may not be worth it.
	 */

	if (Chan[minor].open == TRUE) {
		return -EBUSY;
	}
	Chan[minor].open = TRUE;	/* The device is open */
	Chan[minor].gain = BP_10_00V;	/* +/- 10V */
	Chan[minor].mode = filePtr->f_flags;	/* set trigger mode */
	Chan[minor].lowChan = minor;	/* set MUX scan limits to current channel */
	Chan[minor].hiChan = minor;

	if (Chan[minor].mode == ADC_PACER_EXTERN_TRIG) {
		BoardData.pacerReg = 0x1;
	} else {
		BoardData.pacerReg = 0x0;
	}

	outb_p(0x00, STATUS_REG);	/* Clear INT bit */
	outb_p(0x0, MISC_REG);	/* Put in compatibility mode */

#ifdef DEBUG
	printk("%s: open(): minor %d mode %d.\n", ADAPTER_ID, minor, Chan[minor].mode);
#endif
	return 0;
}
#endif



#if 0
static int adc_read(struct inode *iNode, struct file *filePtr, char *buf, int count)
{
	int stat;
	BYTE bReg;
	int minor = MINOR(iNode->i_rdev);
	register int i;

	/* Read */

	switch (Chan[minor].mode) {

	case ADC_SOFT_TRIGGER:
#ifdef DEBUG
		printk("adc_read(): Entering ADC_SOFT_TRIGGER mode.\n");
#endif
		if (CompatibilityRead(KernBuff, &Chan[minor])) {
			printk("adc_read: SoftTrigRead() failed.\n");
			BoardData.busy = FALSE;
			return (-1);
		}
		break;

	case ADC_EXTERNAL_TRIGGER:
#ifdef DEBUG
		printk("adc_read(): Entering ADC_EXTERNAL_TRIGGER mode.\n");
#endif
		if (Chan[minor].count == 1) {	/* use compatibility mode */
			if (CompatibilityRead(KernBuff, &Chan[minor])) {
				printk("adc_read: CompatibilityRead() failed with ADC_EXTERNAL_TRIGGER.\n");
				BoardData.busy = FALSE;
				return (-1);
			}
		} else {
			StopPacer();	/* disable pacer if in pacer mode */
			if (EnhancedRead(KernBuff, &Chan[minor])) {
				printk("adc_read: ExternalRead() failed with ADC_EXTERNAL_TRIGGER.\n");
				BoardData.busy = FALSE;
				return (-1);
			}
		}
		break;

	case ADC_PACER_CLOCK:
#ifdef DEBUG
		printk("adc_read(): Entering ADC_PACER_CLOCK mode.\n");
#endif
		if (Chan[minor].count == 1) {	/* use compatibility mode */
			if (CompatibilityRead(KernBuff, &Chan[minor])) {
				printk("adc_read: CompatibilityRead() failed with pacer.\n");
				BoardData.busy = FALSE;
				return (-1);
			}
		} else {
			StopPacer();	/* disable pacer if in pacer mode */
			outb_p(0x1, PACER_CLOCK_REG);	/*IP0 controls counter gate */
			if (EnhancedRead(KernBuff, &Chan[minor])) {
				printk("adc_read: EnchancedRead() failed.\n");
				BoardData.busy = FALSE;
				return (-1);
			}
		}
		break;

	case ADC_PACER_EXTERN_TRIG:
#ifdef DEBUG
		printk("adc_read(): Entering ADC_PACER_EXTERN_TRIG mode.\n");
#endif
		StopPacer();	/* disable pacer if in pacer mode */
		outb_p(0x1, PACER_CLOCK_REG);	/*IP0 controls counter gate */
		if (EnhancedRead(KernBuff, &Chan[minor])) {
			printk("adc_read: EnchancedRead() with external tigger failed.\n");
			BoardData.busy = FALSE;
			return (-1);
		}
		break;
	}

	/* Check that data can be written to file. */

	if ((stat = verify_area(VERIFY_WRITE, buf, sizeof(WORD) * Chan[minor].count)) != 0) {
		printk("adc_read: Failed VERIFY_WRITE.\n");
		BoardData.busy = FALSE;
		return -1;
	}
	/* Write data to user space */

	for (i = 0; i < Chan[minor].count; i++) {
		KernBuff[i] >>= 4;
	}

	if (Chan[minor].count == 1) {
		put_fs_word(*KernBuff, (WORD *) buf);
	} else {
		memcpy_tofs(buf, KernBuff, Chan[minor].count * sizeof(WORD));
	}

	BoardData.busy = FALSE;
	return (Chan[minor].count);	/* return number of samples read */
}
#endif




#if 0
static int dio_write(struct inode *iNode, struct file *filePtr, char *buf, int count)
{
	int minor = MINOR(iNode->i_rdev);
	BYTE value;

	if (minor != DIO_CHANNEL) {
		printk("dio_write: bad minor number = %d\n", minor);
		return -1;
	}
	cli();
	value = get_fs_byte(buf) & 0xf;
	outb_p(value, DIO_REG);
	sti();
#ifdef DEBUG
	printk("DIO set to %#x\n", value & 0xf);
#endif
	return 1;
}
#endif


#if 0
static int adc_ioctl(struct inode *iNode, struct file *filePtr, unsigned int cmd, LONG arg)
{
	int minor = MINOR(iNode->i_rdev);

	switch (cmd) {
	case ADC_SET_GAINS:
		Chan[minor].gain = (BYTE) arg;
		break;
	case ADC_GET_GAINS:
		put_fs_long((long) Chan[minor].gain, (long *) arg);
		break;
	case ADC_SET_PACER_FREQ:
		if (BoardData.freq > MAX_FREQ && Chan[minor].mode == ADC_PACER_CLOCK) {
			printk("ioctl:  BoardData.freq = %ld out of range.\n", BoardData.freq);
			return -1;
		} else {
			BoardData.freq = (LONG) arg;
			SetPacerFreq(&BoardData);
			LoadPacer(&BoardData);	/* load the board frequency now */
			outb_p(BoardData.pacerReg, PACER_CLOCK_REG);
		}
		break;
	case ADC_GET_PACER_FREQ:
		put_fs_long(BoardData.freq, (long *) arg);
		break;
	case ADC_STOP_PACER:
		StopPacer();
		break;
	case CTR0:
		if (arg == 1) {
			BoardData.pacerReg |= 0x2;
		} else {
			BoardData.pacerReg &= ~(0x2);
		}
		outb_p(BoardData.pacerReg, PACER_CLOCK_REG);
		break;
	case COUNTER0:
		LoadCounter0((LONG) arg);
		break;
	case ADC_DIO_PRESET:
		if (arg == -1) {
			Chan[minor].adc_dio_pretrig = FALSE;
		} else {
			Chan[minor].adc_dio_pretrig = TRUE;
			Chan[minor].dio_value = (BYTE) (arg & 0xf);
		}

#ifdef DEBUG
		printk("ioctl(ADC_DIO_PRESET):  value = %#x\n", Chan[minor].dio_value);
#endif

		break;
	case ADC_GET_STATUS:
		/* return status register */
		put_fs_long(inb_p(STATUS_REG), (long *) arg);
		break;
	case SET_MUX_LOW:
		Chan[minor].lowChan = (BYTE) arg;
		break;
	case SET_MUX_HIGH:
		Chan[minor].hiChan = (BYTE) arg;
		break;
	case GET_MUX_SCAN_LIMITS:
		put_fs_long(inb_p(MUX_SCAN_LIMITS), (long *) arg);
		break;
	default:
		return (-EINVAL);
		break;
	}
	return 0;
}
#endif

/***************************************************************************
 *
 * Block copy of data samples from data register to kernel memory.
 *
 ***************************************************************************/

#if 0
inline char *
 RegCopy(char *dest, const int reg, int n)
{
	__asm__ __volatile__(
				    "cld\n\t"
				    "rep\n\t"
				    "insw"
				    :
				    :"D"(dest), "d"(reg), "c"(n)
				    :"di", "dx", "cx");
	return dest;
}
#endif

/***************************************************************************
 *
 *  Alarm handler to handle situations where an interrupt is missed.
 *
 ***************************************************************************/

#if 0
static void adc0_TimerHandler(LONG unused)
{
	/* 
	   we should only be here if an interrupt occured while 
	   we were reading half the FIFO 
	 */

#ifdef DEBUG
	printk("TimerHandler: WordsToRead = %d\n", WordsToRead);
#endif

	cli();
	outb_p(0x30, COUNTERB_CONTROL);		/* Set TOTALBAR to 1 */
	RegCopy((char *) KernBuffPtr, base_reg, WordsToRead);
	WordsToRead -= WordsToRead;
	del_timer(&TimerList);
	wake_up_interruptible(&adc0_wait);
}
#endif

/***************************************************************************
 *
 * Interrupt handler used to service enhanced mode(interrupt) read().
 *
 ***************************************************************************/

#if 0
static void adc0_ReadInterrupt(int irq)
{
	WORD msb, lsb;

#ifdef DEBUG
	printk("Entering adc0_ReadInterrupt().  mode = %d\n", CurrChan->mode);
#endif

	cli();
	switch (CurrChan->mode) {

	case ADC_SOFT_TRIGGER:
		/* Bang! Bang! Method: Force conversion, Conversion forces interrupt */
		lsb = (WORD) inb(base_reg);	/* Sample: ADC -> kernel buffer */
		msb = (WORD) inb(MSB_DATA_BYTE);
		*KernBuffPtr++ = (msb << 8) | lsb;
		WordsToRead--;	/* One sample at a time */
		if (!WordsToRead) {
			wake_up_interruptible(&adc0_wait);
		} else {
			outb_p(0x00, STATUS_REG);	/* Clear INT bit, allow Interrupt */
			outb_p(0x1, base_reg);	/* Force conversion */
			sti();
		}
		break;

	case ADC_PACER_CLOCK:
	case ADC_EXTERNAL_TRIGGER:
		if (CurrChan->count == 1) {
			lsb = (WORD) inb(base_reg);	/* Sample: ADC -> kernel buffer */
			msb = (WORD) inb(MSB_DATA_BYTE);
			*KernBuffPtr++ = (msb << 8) | lsb;
			wake_up_interruptible(&adc0_wait);
			break;
		}
	case ADC_PACER_EXTERN_TRIG:
		if (WordsToRead > PACKETSIZE) {
			RegCopy((char *) KernBuffPtr, base_reg, PACKETSIZE);
			WordsToRead -= PACKETSIZE;
			KernBuffPtr += PACKETSIZE;
			if (WordsToRead < PACKETSIZE) {
				TimerList.expires = WordsToRead * 100 / BoardData.freq + 2;
				add_timer(&TimerList);
			}
			outb_p(0x00, STATUS_REG);	/* Clear INT bit, allow Interrupt */
			sti();
		} else {
			outb_p(0x30, COUNTERB_CONTROL);		/* Set TOTALBAR to 1 */
			RegCopy((char *) KernBuffPtr, base_reg, WordsToRead);
			del_timer(&TimerList);
			WordsToRead -= WordsToRead;
			wake_up_interruptible(&adc0_wait);
			return;
		}
		break;
	default:
		break;
	}
}
#endif

/***************************************************************************
 *
 * Handles software/pacer triggered read().
 *
 * Bang! Bang! Method: 
 *    
 *    o Runs in Compatibility Mode
 *    o Force interrupt by forcing conversion.
 *    o Get one sample at a time and put it in the kernel buffer(kernBuff).
 *    o Get chan->count samples
 *
 ***************************************************************************/

static int compatibility_read(comedi_device *dev,comedi_trig *it)
{
	int bReg;

#ifdef DEBUG
	printk("Entering CompatibilityRead().\n");
#endif

	/* Set interrupt level on board, disable interrupts, and pacer  */

	outb(devpriv->ir, dev->iobase + CNTRL_REG);	/* & clear INTE bit in control reg */

	bReg = inb(dev->iobase+MISC_REG);	/* put in compatibility mode */
	outb(bReg & ~ENHANCED_BIT, MISC_REG);

	bReg = inb(dev->iobase + MUX_SCAN_LIMITS);	/* Read/Write clears FIFO */
	outb(bReg, dev->iobase + MUX_SCAN_LIMITS);

#if 0
	/* Prepare global data for parameterless interrupt handler */

	CurrChan = chan;	/* pass chanel number to global */
	KernBuffPtr = KernBuff;	/* same with KernBuff           */
	WordsToRead = chan->count;

	bReg = inb(CNTRL_REG);	/* Turns on ADC interrupts */

	switch (chan->mode) {

	case ADC_SOFT_TRIGGER:
		outb(bReg | CNTL_INTE, CNTRL_REG);
		outb(0x00, STATUS_REG);	/* Clears INT bit, allow Int. */
		outb(0x1, base_reg);	/* Force first conversion */
		break;

	case ADC_EXTERNAL_TRIGGER:
		outb(bReg | CNTL_INTE | 0x2, CNTRL_REG);
		outb(0x00, STATUS_REG);	/* Clears INT bit, allow Int. */
		break;

	case ADC_PACER_CLOCK:
		outb(bReg | CNTL_INTE | 0x3, CNTRL_REG);
		outb(0x00, STATUS_REG);	/* Clears INT bit, allow Int. */
		break;
	}

#endif
	outb(devpriv->ir, dev->iobase+CNTRL_REG);	/* & clear INTE bit in control reg */

	return 0;
}


/****************************************************************************
 *   Set Pacer Clock Frequency
 * 
 *   Description: 
 *       Set the counters so that the pacer clock runs at the
 *       desired frequency.  The frequency is generated by dividing
 *       down a 10 MHz clock, so all frequencies can not be generated.
 *       This routine calculated the divisor to generate a frequency
 *       as near as possible to the requested one.  It then calculates
 *       the real frequency and returns it .
 ****************************************************************************/

#if 0
static int SetPacerFreq(BoardRec * board)
{
	short unsigned ctr1, ctr2;
	LONG product, error;

	/* divide 10Mhz by frequency */
	product = 10000000.0 / board->freq + 0.5;

	/* Now the job is to find two 16 bit numbers, that when multiplied
	   together are approximately equal to product.  Start by setting
	   one of them, ctr1 to 2 (minimum settable value) and increment until
	   the error is minimized and ctr2 is less than 32768.

	   NOTE: In Mode 2, a value of 1 is illegal! Therefore, crt1 and crt2
	   can never be 1.

	 */

	ctr1 = product / 32768;
	if (ctr1 < 2)
		ctr1 = 2;
	ctr2 = product / ctr1;
	error = abs(product - (long) ctr2 * (long) ctr1);

	while (error && ctr1 < 32768 && ctr2 > 1) {
		ctr1++;
		ctr2 = product / ctr1;
		error = abs(product - (long) ctr2 * (long) ctr1);
	}

	/* the frequency is prime, add 1 to it */
	if (error) {
		product++;
		ctr1 = product / 32768;
		if (ctr1 < 2)
			ctr1 = 2;
		ctr2 = product / ctr1;
		error = abs(product - (long) ctr2 * (long) ctr1);

		while (error && ctr1 < 32768 && ctr2 > 1) {
			ctr1++;
			ctr2 = product / ctr1;
			error = abs(product - (long) ctr2 * (long) ctr1);
		}
	}
	/* we can't have ctr2 equal to 1, or system hangs */
	if (ctr2 == 1) {
		ctr2++;
		ctr1 /= 2;
	}
	board->ctr1 = ctr1;
	board->ctr2 = ctr2;
	board->freq = 10000000.0 / ((long) ctr1 * (long) ctr2) + 0.5;

#ifdef DEBUG
	printk("SetPacerFreq: Pacer Register set to %#x\n", BoardData.pacerReg);
#endif

	return 0;
}
#endif

/***************************************************************************
 *
 * Load two part frequency to pacer counter chip.
 *
 ***************************************************************************/

static void load_pacer(comedi_device *dev,int timer)
{
	unsigned int mask;
	unsigned int ctr1,ctr2;

	/* Write the values of ctr1 and ctr2 into counter A1 and A2 */

	ctr1=timer&0xffff;
	ctr2=(timer>>16)&0xffff;

	mask = C2 | MODE2 | LSBFIRST;
	outb(mask, dev->iobase+COUNTERA_CONTROL);

	outb(ctr2 & 0xff , dev->iobase + COUNTERA_2_DATA);
	outb(ctr2 >> 8, dev->iobase + COUNTERA_2_DATA);

	mask = C1 | MODE2 | LSBFIRST;
	outb(mask, dev->iobase+COUNTERA_CONTROL);

	outb(ctr1 & 0xff, dev->iobase + COUNTERA_1_DATA);
	outb(ctr1 >> 8, dev->iobase + COUNTERA_1_DATA);

}

/***************************************************************************
 *
 * Load value into Counter 0  XXXX    Mode    MSB     LSB
 *                            Byte 3  Byte 2  Byte 1  Byte 0
 *
 ***************************************************************************/

static void load_counter(comedi_device *dev,int mode,int value)
{
	unsigned int mask;

	/* Write the value into Counter 0 Mode 2 */

	/* the mode is in the thrid byte */
	mask = (0xff & mode) | C0 | LSBFIRST;
	outb(mask, dev->iobase + COUNTERA_CONTROL);

#if 0
	if (mode & 0xff00)	/* load control word only */
		return;
#endif
	outb(value & 0xff, dev->iobase+COUNTERA_0_DATA);
	outb((value>>8)&0xff, dev->iobase+COUNTERA_0_DATA);
}


/***************************************************************************
 *
 * Turn off pacer timer chip.
 *
 ***************************************************************************/

static void stop_pacer(comedi_device *dev)
{
	unsigned int mask;

	mask = C2 | MODE2 | LSBFIRST;
	outb(mask, dev->iobase + COUNTERA_CONTROL);
	mask = C1 | MODE2 | LSBFIRST;
	outb(mask, dev->iobase + COUNTERA_CONTROL);
}

/***************************************************************************
 *
 * Handles 
 *    o pacer/counter triggered read() with software start.
 *    o pacer/counter triggered read() with external start.
 *    o external triggered read().
 *
 * Pacer Method: 
 *
 *    o Runs in CIO-DAS 16/330 Enhanced Mode
 *    o Pacer counter controls frequency of sample conversions
 *    o Total counter controls number of samples taken
 *    o Interrupts occur every 512(PACKETSIZE) samples and at last sample
 *    o Get chan->count samples
 *    o Samples are saved in kernel buffer(kernBuff) in readInterrupt()
 *
 ***************************************************************************/

#if 0
static int EnhancedRead(WORD * kernBuff, ChanRec * chan)
{
	BYTE bReg = 0x0;
	WORD wReg = 0x0;

#ifdef DEBUG
	printk("Entering EnchacedRead().\n");
#endif

	cli();

	/* Set interrupt level on board, disable interrupts, and pacer  */

	outb_p(BoardData.ir, CNTRL_REG);	/* & clear INTE bit in control reg */

	/*
	   There is some confusion exactly what is going on, but if bit 5 in reg+11
	   is equal to 1, the FIFO is over run and we can not write to reg+2,
	   or what I call MUX_SCAN_LIMITS.  So I will try to clear bit 5
	   and keep reading until the FIFO is not full.  This should not
	   happen, but Robert Wilhelm (robert@haegar.physiol.med.tu-muenchen.de)
	   claims that it is.
	 */

	/* clear Overrun status bit (bit 5 in reg+11) */
	bReg = (BYTE) inb_p(MISC_REG) & (~MISC_OVERRUN);
	outb_p(bReg, MISC_REG);

	do {
		bReg = (BYTE) inb_p(MUX_SCAN_LIMITS);	/* Read/Write clears FIFO */
		outb_p(bReg, MUX_SCAN_LIMITS);
	}
	while (((BYTE) inb_p(MISC_REG) & MISC_OVERRUN));

	bReg = chan->gain | ENHANCED_BIT;	/* Set gain and enhanced mode */
	outb(bReg, MISC_REG);
	bReg = (BYTE) inb_p(MISC_REG);	/* check to make sure we are in */
	if (!(bReg & ENHANCED_BIT)) {	/* enhanced mode                */
		return (NO_ENHANCED_MODE);
	}
	do {
		bReg = (BYTE) inb_p(MUX_SCAN_LIMITS);	/* Read/Write clears FIFO */
		outb_p(bReg, MUX_SCAN_LIMITS);
	}
	while (((BYTE) inb_p(MISC_REG) & MISC_OVERRUN));

	if (DT_CONNECT) {
		bReg = (BYTE) inb_p(MISC_REG);
		bReg |= MISC_DT;
		outb_p(bReg, MISC_REG);
	}
	/* load the Total Counter  for the DAS 16/330 */

	bReg = C0 + MODE0 + LSBFIRST;	/* CounterB -> mode 0, DON'T load it */
	outb_p(bReg, COUNTERB_CONTROL);		/* Sets output HIGH? */

	bReg = (BYTE) inb_p(CNTRL_REG);		/* S1=0 and S0=0 from above */
	if (chan->mode == ADC_PACER_CLOCK || chan->mode == ADC_PACER_EXTERN_TRIG) {
		bReg |= (0x3);	/* Set S1=1 and S0=1 Internal Pacer */
	} else {
		bReg |= (0x2);	/* Set S1=1 and S0=0 External Pacer (A/D Triggering) */
	}
	outb_p(bReg, CNTRL_REG);

	wReg = (WORD) (chan->count >> 16);	/* get upper word of count */
	if (chan->count & 0xffff)
		wReg++;		/* increment if lower count is ! zero */

	bReg = C0 + MODE0 + LSBFIRST;	/* Counter -> mode 0, DON'T load it */
	outb_p(bReg, COUNTERB_CONTROL);		/* Sets output HIGH? */
	bReg = (BYTE) (wReg & 0xff);	/* load the upper word */
	outb_p(bReg, COUNTERB_0_DATA);
	bReg = (BYTE) (wReg >> 8);
	outb_p(bReg, COUNTERB_0_DATA);

	/* Set the trigger counters. Counter 0 is changed to counter 1 */

	bReg = C1 + MODE2 + LSBFIRST;	/* Ensure clock starts low */
	outb_p(bReg, COUNTERB_CONTROL);

	bReg = C1 + MODE0 + LSBFIRST;	/* Make C0 clock high */
	outb_p(bReg, COUNTERB_CONTROL);

	bReg = C1 + MODE2 + LSBFIRST;	/* Make C0 clock low */
	outb_p(bReg, COUNTERB_CONTROL);		/* complete 1st clock cycle */

	wReg = (WORD) (chan->count & 0xffff);

	if (wReg == 1) {
		wReg++;		/* wReg can not be 1 */
		bReg = (BYTE) (wReg & 0xff);	/* load the lower word */
		outb_p(bReg, COUNTERB_1_DATA);
		bReg = (BYTE) (wReg >> 8);
		outb_p(bReg, COUNTERB_1_DATA);
		outb_p(0x1, base_reg);	/* Force conversion to get count = 1 */
	} else {
		bReg = (BYTE) (wReg & 0xff);	/* load the lower word */
		outb_p(bReg, COUNTERB_1_DATA);
		bReg = (BYTE) (wReg >> 8);
		outb_p(bReg, COUNTERB_1_DATA);
	}
	outb_p(0x1, base_reg);	/* Force a conversion */
	outw_p(0x0, COUNTERB_1_DATA);	/* rollover mode ?? */

	if (chan->mode == ADC_PACER_CLOCK || chan->mode == ADC_PACER_EXTERN_TRIG) {
		LoadPacer(&BoardData);	/* Establish sample frequency */
	}
	do {
		bReg = (BYTE) inb_p(MUX_SCAN_LIMITS);	/* Read/Write clears FIFO */
		outb_p(bReg, MUX_SCAN_LIMITS);
	}
	while (((BYTE) inb_p(MISC_REG) & MISC_OVERRUN));

	outb_p(0x00, STATUS_REG);	/* Clear INT bit, allow Interrupt */

	/* Prepare global data for parameterless interrupt handler */

	CurrChan = chan;	/* pass chanel number to global */
	KernBuffPtr = KernBuff;	/* same with KernBuff           */
	WordsToRead = chan->count;

	bReg = (BYTE) inb_p(CNTRL_REG);		/* Enable interrupts */
	bReg |= CNTL_INTE;
	outb_p(bReg, CNTRL_REG);

	if (chan->mode == ADC_PACER_CLOCK || chan->mode == ADC_PACER_EXTERN_TRIG) {
		outb_p(BoardData.pacerReg, PACER_CLOCK_REG);	/* Start pacer begins sampling. */
	}
	if (chan->adc_dio_pretrig) {
		/* enable interrupts before setting dio bits */
		sti();
		outb_p(chan->dio_value, DIO_REG);
	}
	interruptible_sleep_on(&adc0_wait);	/* Block in wait state */

	/* turn off interrupts, and trigger control.  Leave pacer clock on if enabled */
	outb_p(0x0, CNTRL_REG);

	if (WordsToRead != 0) {
		printk("Timing error in EnchacedRead: WordsToRead = %d\n", WordsToRead);
		return -1;
	}
	outb_p(0x00, STATUS_REG);	/* Clear INT bit */
	return 0;
}
#endif
