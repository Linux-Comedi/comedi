/*
 * Device Driver for DataTranslation DT2801
 *
 */

#include <comedi_module.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <asm/io.h>

#define DT2801_TIMEOUT 1000


/* Hardware Configuration */
/* ====================== */

#define DT2801_MAX_DMA_SIZE (64 * 1024)

/* Ports */
#define DT2801_IOSIZE 2


/* define's & typedef's */
/* ====================== */

/* Commands */
#define DT_C_RESET       0x0
#define DT_C_CLEAR_ERR   0x1
#define DT_C_READ_ERRREG 0x2
#define DT_C_SET_CLOCK   0x3

#define DT_C_TEST        0xb
#define DT_C_STOP        0xf

#define DT_C_SET_DIGIN   0x4
#define DT_C_SET_DIGOUT  0x5
#define DT_C_READ_DIG    0x6
#define DT_C_WRITE_DIG   0x7

#define DT_C_WRITE_DAIM  0x8
#define DT_C_SET_DA      0x9
#define DT_C_WRITE_DA    0xa

#define DT_C_READ_ADIM   0xc
#define DT_C_SET_AD      0xd
#define DT_C_READ_AD     0xe

/* Command modifiers (only used with read/write), EXTTRIG can be
   used with some other commands.
*/
#define DT_MOD_DMA     (1<<4)
#define DT_MOD_CONT    (1<<5)
#define DT_MOD_EXTCLK  (1<<6)
#define DT_MOD_EXTTRIG (1<<7)

/* Bits in status register */
#define DT_S_DATA_OUT_READY   (1<<0)
#define DT_S_DATA_IN_FULL     (1<<1)
#define DT_S_READY            (1<<2)
#define DT_S_COMMAND          (1<<3)
#define DT_S_COMPOSITE_ERROR  (1<<7)


/* registers */
#define DT2801_DATA		0
#define DT2801_STATUS		1
#define DT2801_CMD		1

static int dt2801_attach(comedi_device *dev,comedi_devconfig *it);
static int dt2801_detach(comedi_device *dev);
comedi_driver driver_dt2801={
	driver_name:	"dt2801",
	module:		&__this_module,
	attach:		dt2801_attach,
	detach:		dt2801_detach,
};


typedef struct{
	char *name;
	int boardcode;
	int adbits;
	int adrangetype;
	int dabits;
} boardtype_t;

/* Typeid's for the different boards of the DT2801-series
   (taken from the test-software, that comes with the board)
   */
static boardtype_t boardtypes[] =
{
	{"dt2801",0x09,		12,RANGE_unknown,12},
	{"dt2801-a",0x52,	12,RANGE_unknown,12},
	{"dt2801/5716a",0x82,	16,RANGE_unknown,12},
	{"dt2805",0x12,		12,RANGE_unknown,12},
	{"dt2805/5716a",0x92,	16,RANGE_unknown,12},
	{"dt2808",0x20,		12,RANGE_unknown,8},
	{"dt2818",0xa2,		12,RANGE_unknown,12},
	{"dt2809",0xb0,		12,RANGE_unknown,12},
};
#define n_boardtypes ((sizeof(boardtypes))/(sizeof(boardtypes[0])))


typedef struct{
	boardtype_t *board;
	unsigned int dac_range_types[2];
}dt2801_private;
#define devpriv ((dt2801_private *)dev->private)
#define boardtype (*devpriv->board)


static int dt2801_ai_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int dt2801_ao_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);
static int dt2801_dio(comedi_device *dev,comedi_subdevice *s,comedi_trig *it);

/* These are the low-level routines:
   writecommand: write a command to the board
   writedata: write data byte
   readdata: read data byte
 */

/* Only checks DataOutReady-flag, not the Ready-flag as it is done
   in the examples of the manual. I don't see why this should be
   necessary. */
static int dt2801_readdata(comedi_device *dev, int* data)
{
	int stat = 0;
	int timeout = DT2801_TIMEOUT;

	do{
		stat = inb_p(dev->iobase+DT2801_STATUS);
		if (stat & DT_S_COMPOSITE_ERROR) {
       			printk("dt2801: composite-error in dt2801_readdata()\n");
           		return -EIO;
		}
		if(stat & DT_S_DATA_OUT_READY){
			*data = inb_p(dev->iobase+DT2801_DATA);
			return 0;
		}
		if(stat & DT_S_READY){
			printk("dt2801: no data to read in dt2801_readdata()\n");
			return -EIO;
		}
	}while(--timeout>0);
	printk("dt2801: timeout in dt2801_readdata()\n");
	return -ETIME;
}

static int dt2801_readdata2(comedi_device *dev, int *data)
{
	int lb, hb;
	int ret;

	ret=dt2801_readdata(dev, &lb);
	if(ret<0)return ret;
	ret=dt2801_readdata(dev, &hb);
	if(ret<0)return ret;

	*data = (hb<<8)+lb;
	return 0;
}

static int dt2801_writedata(comedi_device *dev, unsigned int data)
{
	int stat = 0;
	int timeout = DT2801_TIMEOUT;

	do{
		stat = inb_p(dev->iobase+DT2801_STATUS);

		if (stat & DT_S_COMPOSITE_ERROR) {
        		printk("dt2801: composite-error in dt2801_writedata()\n");
            		return -EIO;
		}
		if (!(stat & DT_S_DATA_IN_FULL)) {
			outb_p(data & 0xff, dev->iobase+DT2801_DATA);
			return 0;
		}
		if(stat & DT_S_READY){
			printk("dt2801: ready flag set (bad!) in dt2801_writedata()\n");
			return -EIO;
		}
	}while(--timeout>0);

	printk("dt2801: timeout in dt2801_writedata()\n");

	return -ETIME;
}

static int dt2801_writedata2(comedi_device *dev, unsigned int data)
{
	int ret;

	ret=dt2801_writedata(dev,  data & 0xff);
	if(ret<0)return ret;
	ret=dt2801_writedata(dev, (data >> 8) );
	if(ret<0)return ret;

	return 0;
}

static int dt2801_wait_for_ready(comedi_device *dev)
{
	int timeout = DT2801_TIMEOUT;
	int stat;

	printk("dt2801: dt2801_wait_for_ready()\n");
	stat = inb_p(dev->iobase+DT2801_STATUS);
	if(stat & DT_S_READY){
		printk("dt2801: board immediately ready\n");
		return 0;
	}
	do{
		stat = inb_p(dev->iobase+DT2801_STATUS);

		if (stat & DT_S_COMPOSITE_ERROR) {
        		printk("dt2801: composite-error in dt2801_wait_for_ready()\n");
            		return -EIO;
		}
		if(stat & DT_S_READY){
			printk("dt2801: waited %d cycles\n",DT2801_TIMEOUT-timeout);
			return 0;
		}
	}while(--timeout>0);

	printk("dt2801: timeout in dt2801_wait_for_ready() status=0x%02x\n",stat);

	return -ETIME;
}

static int dt2801_writecmd(comedi_device * dev, int command)
{
	int stat;

	dt2801_wait_for_ready(dev);

	stat = inb_p(dev->iobase+DT2801_STATUS);
	if (stat & DT_S_COMPOSITE_ERROR) {
        	printk("dt2801: composite-error in dt2801_writecmd()\n");
		return -EIO;
	}
	if (!(stat & DT_S_READY)) {
        	printk("dt2801: !ready in dt2801_writecmd(), ignoring\n");
	}
	outb_p(command, dev->iobase+DT2801_CMD);

	return 0;
}


static int dt2801_reset(comedi_device *dev)
{
	int board_code=0;

	printk("dt2801: resetting board...\n");

	printk("dt2801: stop\n");
	dt2801_writecmd(dev,DT_C_STOP);
	printk("dt2801: reading dummy\n");
	dt2801_readdata(dev,&board_code);

	printk("dt2801: reset\n");
	dt2801_writecmd(dev,DT_C_RESET);

	printk("dt2801: reading code\n");
	dt2801_readdata(dev,&board_code);

	printk("dt2801: ok.  code=0x%02x\n",board_code);


	return 0;
}

static int dac_range_lkup(int bip,int v10)
{
	if(bip){
		if(v10)return RANGE_unipolar10;
		return RANGE_unipolar5;
	}else{
		if(v10)return RANGE_bipolar10;
		return RANGE_bipolar5;
	}
}


/*
   options:
	[0] - i/o base
	[1] - unused
	[2] - a/d 0=differential, 1=single-ended
	[3] - dac0 unipolar=0, bipolar=1
	[4] - dac0 5 V reference =0, 10 V ref = 1
	[5] - dac1 unipolar=0, bipolar=1
	[6] - dac0 5 V reference =0, 10 V ref = 1
*/
static int dt2801_attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int iobase;
	int board_code,type;
	int ret=0;

	iobase=it->options[0];
	if(check_region(iobase,DT2801_IOSIZE)<0){
		comedi_error(dev,"I/O port conflict");
		return -EIO;
	}
	request_region(dev->iobase, DT2801_IOSIZE, "dt2801");
	dev->iobase=iobase;

	/* do some checking */

	board_code=dt2801_reset(dev);

	for(type=0;type<n_boardtypes;type++){
		if(boardtypes[type].boardcode==board_code)
			goto havetype;
	}
	printk("dt2801: unrecognized board code=0x%02x, contact author\n",board_code);
	type=0;

havetype:
	printk("dt2801: %s at port 0x%x\n",boardtypes[type].name,iobase);

	dev->n_subdevices=4;

	if((ret=alloc_subdevices(dev))<0)
		return ret;

	if((ret=alloc_private(dev,sizeof(dt2801_private)))<0)
		return ret;

	devpriv->board=boardtypes+type;

	s=dev->subdevices+0;
	/* ai subdevice */
	s->type=COMEDI_SUBD_AI;
	s->subdev_flags=SDF_READABLE;
	if(it->options[2])s->n_chan=16;
	else s->n_chan=8;
	s->maxdata=(1<<boardtype.adbits)-1;
	s->range_type=boardtype.adrangetype;
	s->trig[0]=dt2801_ai_mode0;

	s++;
	/* ao subdevice */
	s->type=COMEDI_SUBD_AO;
	s->subdev_flags=SDF_WRITEABLE;
	s->n_chan=2;
	s->maxdata=(1<<boardtype.dabits)-1;
	s->range_type_list=devpriv->dac_range_types;
	devpriv->dac_range_types[0]=dac_range_lkup(it->options[3],it->options[4]);
	devpriv->dac_range_types[1]=dac_range_lkup(it->options[5],it->options[6]);
	s->trig[0]=dt2801_ao_mode0;

	s++;
	/* 1st digital subdevice */
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_READABLE|SDF_WRITEABLE;
	s->n_chan=8;
	s->maxdata=1;
	s->range_type=RANGE_digital;
	s->trig[0]=dt2801_dio;

	s++;
	/* 2nd digital subdevice */
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_READABLE|SDF_WRITEABLE;
	s->n_chan=8;
	s->maxdata=1;
	s->range_type=RANGE_digital;
	s->trig[0]=dt2801_dio;

	return 0;
}

static int dt2801_detach(comedi_device *dev)
{
	if(dev->iobase)
		release_region(dev->iobase,DT2801_IOSIZE);

	return 0;
}

static int dt2801_ai_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
        int data;
        int stat;


        stat = dt2801_writecmd(dev, DT_C_READ_ADIM);
        dt2801_writedata(dev, CR_RANGE(it->chanlist[0]));
        dt2801_writedata(dev, CR_CHAN(it->chanlist[0]));
        stat = dt2801_readdata2(dev, &data);

        if (stat != 0) {
             printk("dt2801: stat = %x\n", stat);
             return -EIO;
        }

	it->data[0]=data;

        return 1;
}

static int dt2801_ao_mode0(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	int chan=CR_CHAN(it->chanlist[0]);

	dt2801_writecmd(dev, DT_C_WRITE_DAIM);
	dt2801_writedata(dev, chan);
	dt2801_writedata2(dev, it->data[0]);
	
	return 1;
}

static int dt2801_dio(comedi_device *dev,comedi_subdevice *s,comedi_trig *it)
{
	unsigned int bits;
	int which=0;
	
	if(s==dev->subdevices+4)which=1;
	
	if(it->flags & TRIG_CONFIG){
		/* configure */
		if(it->chanlist[it->n_chan-1]){
			s->io_bits=0xff;
			dt2801_writecmd(dev, DT_C_SET_DIGOUT);
		}else{
			s->io_bits=0;
			dt2801_writecmd(dev, DT_C_SET_DIGIN);
		}
		dt2801_writedata(dev, which);
	}else{
		if(s->io_bits){
			do_pack(&s->state,it);
			dt2801_writecmd(dev, DT_C_WRITE_DIG);
			dt2801_writedata(dev, which);
			dt2801_writedata(dev, s->state);
		}else{
			dt2801_writecmd(dev, DT_C_READ_DIG);
			dt2801_writedata(dev, which);
			dt2801_readdata(dev, &bits);
			di_unpack(bits,it);
		}
	}
	return it->n_chan;
}



#ifdef MODULE
int init_module(void)
{
	comedi_driver_register(&driver_dt2801);
	
	return 0;
}

void cleanup_module(void)
{
	comedi_driver_unregister(&driver_dt2801);
}
#endif
