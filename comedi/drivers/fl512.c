/*
    module/FL512.c
*/
/* Option for this board
 * [0] IO base addres
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
#include <asm/io.h>
#include <linux/comedidev.h>

#define FL512_SIZE 16               /* the size of the used memory */
typedef struct {
  char l_byte;
  char h_byte;
  int chan_ai;
  int chan_ao;
} FL512_private;
#define devpriv ((FL512_private *) dev->private)

static comedi_lrange range_FL512 =
{ 4, {
  BIP_RANGE(0.5),
  BIP_RANGE(1),
  BIP_RANGE(5),
  BIP_RANGE(10),
  UNI_RANGE(1),
  UNI_RANGE(5),
  UNI_RANGE(10),
}};

static int FL512_attach(comedi_device *dev,comedi_devconfig *it);
static int FL512_detach(comedi_device *dev);
static int FL512_recognize(char *name);

comedi_driver driver_FL512 = {
 driver_name: "FL512",
 module:  THIS_MODULE,
 attach:  FL512_attach,
 detach:  FL512_detach,
 recognize:      FL512_recognize,
};

static int FL512_ai(comedi_device *dev,
      comedi_subdevice *s,
      comedi_insn *insn,
      lsampl_t *data);
static int FL512_ao(comedi_device *dev,
      comedi_subdevice *s,
      comedi_insn *insn,
      lsampl_t *data);
/*
 * FL512_ai : this is the analog input funkton
 */
static int FL512_ai(comedi_device *dev,
      comedi_subdevice *s,
      comedi_insn *insn,
      lsampl_t *data)
{
  int n;
  unsigned int lo_byte, hi_byte;
  char chan = CR_CHAN(insn->chanspec);
  int iobase = dev->iobase;
  if ( chan < 0 || chan >=16 ) return -1;

  for(n=0; n<insn->n; n++) {          /* sample n times on selected
channel */
    outb(chan,iobase+2);              /* select chan */
    outb(0,iobase+3);                 /* start conversion */
    udelay(30);                       /* sleep 30 usec */
    lo_byte = inb(iobase+2);          /* low 8 byte */
    hi_byte = inb(iobase+3) & 0x8;    /* high 4 bit and mask */
    data[n] = lo_byte + (hi_byte << 8);
  }
  return 1; /*should be changed when interupt is test as timeout */
}

/*
 * FL512_ao : used to write to a DA port n times
 */
static int FL512_ao(comedi_device *dev,
      comedi_subdevice *s,
      comedi_insn *insn,
      lsampl_t *data)
{
  int n;
  char chan = CR_CHAN(insn->chanspec);                  /* get chan to
write */
  int iobase = dev->iobase;                             /* get base
addres   */

  if (chan!=0 && chan!=1) return -1;
  for (n=0; n<insn->n; n++) {                            /* write n data
set */
    outb_p(data[n] & 0x0ff, iobase+4+2*chan);            /* write low
byte   */
    outb_p((data[n] & 0x800) >> 8, iobase+4+2*chan);     /* write high
byte  */
    inb_p(iobase+4+2*chan);                              /* trig */
  }
  return 1; /*should be changed when interupt is test as timeout */
}

/*
 * start to recognize
 */
static int FL512_recognize(char *name) {
  if (!strcmp("FL512",name)) return 0;
  return -1;
}
/*
 * start attach
 */
static int FL512_attach(comedi_device *dev,comedi_devconfig *it)
{
  comedi_subdevice *s;      /* pointer to the subdevice:
          Analog in, Analog out, ( not made ->and Digital IO) */
  dev->iobase = it->options[0];
  printk("comedi:%d FL512: 0x%04x",dev->minor,dev->iobase);
  if (check_region(dev->iobase, FL512_SIZE) < 0) {
    printk("I/O port conflikt\n");
    return -EIO;
  }
  request_region(dev->iobase, FL512_SIZE, "FL512");
  dev->board_name = "FL512";
  dev->n_subdevices = 2;      /* Analog in/out */
  if(alloc_private(dev,sizeof(FL512_private)) < 0)
    return -ENOMEM;

#if DEBUG
  printk("malloc ok\n");
#endif

  if(alloc_subdevices(dev)<0)
    return -ENOMEM;

  /*
   * this if the definitions of the supdevices, 2 have been defined
   */
  /* Analog indput */
  s                = dev->subdevices+0;
  s->type          = COMEDI_SUBD_AI;         /* define subdevice as
Analog In   */
  s->subdev_flags  = SDF_READABLE;           /* you can read it from
userspace  */
  s->n_chan        = 16;                     /* Number of Analog input
channels */
  s->maxdata       = 0x0fff;                 /* accept only 12 bits of
data     */
  s->range_table   = &range_FL512;           /* device use one of the
ranges    */
  s->insn_read     = FL512_ai;               /* funktion to call when
read AD   */
  printk("comedi: subdevice 1 i FL512 lavet\n");

  /* Analog output */
  s                = dev->subdevices+1;
  s->type          = COMEDI_SUBD_AO;         /* define subdevice as
Analog OUT   */
  s->subdev_flags  = SDF_WRITEABLE;          /* you can write it from
userspace  */
  s->n_chan        = 2;                      /* Number of Analog output
channels */
  s->maxdata       = 0x0fff;                 /* accept only 12 bits of
data      */
  s->range_table   = &range_FL512;           /* device use one of the
ranges     */
  s->insn_write    = FL512_ao;               /* funktion to call when
write DA   */
  printk("comedi: subdevice 2 i FL512 lavet\n");
  return 1;
}

static int FL512_detach(comedi_device *dev) {
  release_region(dev->iobase,FL512_SIZE);
  printk("comedi%d:FL512: dummy i detach\n",dev->minor);
  return 0;
}

int init_module(void) {
  comedi_driver_register(&driver_FL512);
  return 0;
}

void cleanup_module(void) {
  comedi_driver_unregister(&driver_FL512);
}

