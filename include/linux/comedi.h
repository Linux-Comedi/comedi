/*
    include/comedi.h (installed as /usr/include/comedi.h)
    header file for comedi

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1998-2001 David A. Schleef <ds@schleef.org>

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


#ifndef _COMEDI_H
#define _COMEDI_H

#ifdef __cplusplus
extern "C" {
#endif

/* comedi's major device number */
#define COMEDI_MAJOR 98

/*
   maximum number of minor devices.  This can be increased, although
   kernel structures are currently statically allocated, thus you
   don't want this to be much more than you actually use.
 */
#define COMEDI_NDEVICES 4

/* number of config options in the config structure */
#define COMEDI_NDEVCONFOPTS 32

/* max length of device and driver names */
#define COMEDI_NAMELEN 20


typedef unsigned int lsampl_t;
typedef unsigned short sampl_t;

/* packs and unpacks a channel/range number */

#define CR_PACK(chan,rng,aref)		( (((aref)&0x3)<<24) | (((rng)&0xff)<<16) | ((chan)&0xffff) )

#define CR_CHAN(a)	((a)&0xffff)
#define CR_RANGE(a)	(((a)>>16)&0xff)
#define CR_AREF(a)	(((a)>>24)&0x03)

#define CR_ALT_FILTER	(1<<26)
#define CR_DITHER		CR_ALT_FILTER
#define CR_DEGLITCH		CR_ALT_FILTER
#define CR_ALT_SOURCE	(1<<27)
#define CR_INVERT	(1<<31)

#define AREF_GROUND	0x00		/* analog ref = analog ground */
#define AREF_COMMON	0x01		/* analog ref = analog common */
#define AREF_DIFF	0x02		/* analog ref = differential */
#define AREF_OTHER	0x03		/* analog ref = other (undefined) */

/* counters -- these are arbitrary values */
#define GPCT_RESET		0x0001
#define GPCT_SET_SOURCE		0x0002
#define GPCT_SET_GATE		0x0004
#define GPCT_SET_DIRECTION	0x0008
#define GPCT_SET_OPERATION	0x0010
#define GPCT_ARM		0x0020
#define GPCT_DISARM		0x0040
#define GPCT_GET_INT_CLK_FRQ	0x0080

#define GPCT_INT_CLOCK		0x0001
#define GPCT_EXT_PIN		0x0002
#define GPCT_NO_GATE		0x0004
#define GPCT_UP			0x0008
#define GPCT_DOWN		0x0010
#define GPCT_HWUD		0x0020
#define GPCT_SIMPLE_EVENT	0x0040
#define GPCT_SINGLE_PERIOD	0x0080
#define GPCT_SINGLE_PW		0x0100
#define GPCT_CONT_PULSE_OUT	0x0200
#define GPCT_SINGLE_PULSE_OUT	0x0400


/* instructions */

#define INSN_MASK_WRITE		0x8000000
#define INSN_MASK_READ		0x4000000
#define INSN_MASK_SPECIAL	0x2000000

#define INSN_READ		( 0 | INSN_MASK_READ)
#define INSN_WRITE		( 1 | INSN_MASK_WRITE)
#define INSN_BITS		( 2 | INSN_MASK_READ|INSN_MASK_WRITE)
#define INSN_CONFIG		( 3 | INSN_MASK_READ|INSN_MASK_WRITE)
#define INSN_GTOD		( 4 | INSN_MASK_READ|INSN_MASK_SPECIAL)
#define INSN_WAIT		( 5 | INSN_MASK_WRITE|INSN_MASK_SPECIAL)
#define INSN_INTTRIG		( 6 | INSN_MASK_WRITE|INSN_MASK_SPECIAL)

/* trigger flags */
/* These flags are used in comedi_trig structures */

#define TRIG_BOGUS	0x0001		/* do the motions */
#define TRIG_DITHER	0x0002		/* enable dithering */
#define TRIG_DEGLITCH	0x0004		/* enable deglitching */
//#define TRIG_RT	0x0008		/* perform op in real time */
#define TRIG_CONFIG	0x0010		/* perform configuration, not triggering */
//#define TRIG_WAKE_EOS	0x0020		/* wake up on end-of-scan events */
//#define TRIG_WRITE	0x0040		/* write to bidirectional devices */

/* command flags */
/* These flags are used in comedi_cmd structures */

#define CMDF_PRIORITY		0x00000008 /* try to use a real-time interrupt while performing command */

#define TRIG_RT		CMDF_PRIORITY /* compatibility definition */
#define TRIG_WAKE_EOS		0x00000020 /* legacy definition for COMEDI_EV_SCAN_END */

#define CMDF_WRITE		0x00000040
#define TRIG_WRITE	CMDF_WRITE /* compatibility definition */

#define CMDF_RAWDATA		0x00000080

#define COMEDI_EV_START		0x00040000
#define COMEDI_EV_SCAN_BEGIN	0x00080000
#define COMEDI_EV_CONVERT	0x00100000
#define COMEDI_EV_SCAN_END	0x00200000
#define COMEDI_EV_STOP		0x00400000

#define TRIG_ROUND_MASK		0x00030000
#define TRIG_ROUND_NEAREST	0x00000000
#define TRIG_ROUND_DOWN		0x00010000
#define TRIG_ROUND_UP		0x00020000
#define TRIG_ROUND_UP_NEXT	0x00030000

/* trigger sources */

#define TRIG_ANY	0xffffffff
#define TRIG_INVALID	0x00000000

#define TRIG_NONE	0x00000001	/* never trigger */
#define TRIG_NOW	0x00000002	/* trigger now + N ns */
#define TRIG_FOLLOW	0x00000004	/* trigger on next lower level trig */
#define TRIG_TIME	0x00000008	/* trigger at time N ns */
#define TRIG_TIMER	0x00000010	/* trigger at rate N ns */
#define TRIG_COUNT	0x00000020	/* trigger when count reaches N */
#define TRIG_EXT	0x00000040	/* trigger on external signal N */
#define TRIG_INT	0x00000080	/* trigger on comedi-internal signal N */
#define TRIG_OTHER	0x00000100	/* driver defined */

/* subdevice flags */

#define SDF_BUSY	0x0001		/* device is busy */
#define SDF_BUSY_OWNER	0x0002		/* device is busy with your job */
#define SDF_LOCKED	0x0004		/* subdevice is locked */
#define SDF_LOCK_OWNER	0x0008		/* you own lock */
#define SDF_MAXDATA	0x0010		/* maxdata depends on channel */
#define SDF_FLAGS	0x0020		/* flags depend on channel */
#define SDF_RANGETYPE	0x0040		/* range type depends on channel */
#define SDF_MODE0	0x0080		/* can do mode 0 */
#define SDF_MODE1	0x0100		/* can do mode 1 */
#define SDF_MODE2	0x0200		/* can do mode 2 */
#define SDF_MODE3	0x0400		/* can do mode 3 */
#define SDF_MODE4	0x0800		/* can do mode 4 */
#define SDF_CMD		0x1000		/* can do commands */

#define SDF_READABLE	0x00010000	/* subdevice can be read (e.g. analog input) */
#define SDF_WRITEABLE	0x00020000	/* subdevice can be written (e.g. analog output) */
#define SDF_INTERNAL	0x00040000	/* subdevice does not have externally visible lines */
#define SDF_RT		0x00080000	/* subdevice is RT capable */
#define SDF_GROUND	0x00100000	/* can do aref=ground */
#define SDF_COMMON	0x00200000	/* can do aref=common */
#define SDF_DIFF	0x00400000	/* can do aref=diff */
#define SDF_OTHER	0x00800000	/* can do aref=other */
#define SDF_DITHER	0x01000000	/* can do dithering */
#define SDF_DEGLITCH	0x02000000	/* can do deglitching */
#define SDF_MMAP	0x04000000	/* can do mmap() */
#define SDF_RUNNING	0x08000000	/* subdevice is acquiring data */
#define SDF_LSAMPL	0x10000000	/* subdevice uses 32-bit samples */
#define SDF_PACKED	0x20000000	/* subdevice can do packed DIO */

/* subdevice types */

#define COMEDI_SUBD_UNUSED              0	/* unused */
#define COMEDI_SUBD_AI                  1	/* analog input */
#define COMEDI_SUBD_AO                  2	/* analog output */
#define COMEDI_SUBD_DI                  3	/* digital input */
#define COMEDI_SUBD_DO                  4	/* digital output */
#define COMEDI_SUBD_DIO                 5	/* digital input/output */
#define COMEDI_SUBD_COUNTER             6	/* counter */
#define COMEDI_SUBD_TIMER               7	/* timer */
#define COMEDI_SUBD_MEMORY              8	/* memory, EEPROM, DPRAM */
#define COMEDI_SUBD_CALIB               9	/* calibration DACs */
#define COMEDI_SUBD_PROC                10	/* processor, DSP */

/* configuration instructions */

#define COMEDI_INPUT			0
#define COMEDI_OUTPUT			1
#define COMEDI_OPENDRAIN		2

#define INSN_CONFIG_ANALOG_TRIG		0x10
//#define INSN_CONFIG_WAVEFORM		0x11
//#define INSN_CONFIG_TRIG		0x12
//#define INSN_CONFIG_COUNTER		0x13
#define INSN_CONFIG_ALT_SOURCE		0x14

/* ioctls */

#define CIO 'd'
#define COMEDI_DEVCONFIG _IOW(CIO,0,comedi_devconfig)
#define COMEDI_DEVINFO _IOR(CIO,1,comedi_devinfo)
#define COMEDI_SUBDINFO _IOR(CIO,2,comedi_subdinfo)
#define COMEDI_CHANINFO _IOR(CIO,3,comedi_chaninfo)
#define COMEDI_TRIG _IOWR(CIO,4,comedi_trig)
#define COMEDI_LOCK _IO(CIO,5)
#define COMEDI_UNLOCK _IO(CIO,6)
#define COMEDI_CANCEL _IO(CIO,7)
#define COMEDI_RANGEINFO _IOR(CIO,8,comedi_rangeinfo)
#define COMEDI_CMD _IOR(CIO,9,comedi_cmd)
#define COMEDI_CMDTEST _IOR(CIO,10,comedi_cmd)
#define COMEDI_INSNLIST _IOR(CIO,11,comedi_insnlist)
#define COMEDI_INSN _IOR(CIO,12,comedi_insn)
#define COMEDI_BUFCONFIG _IOR(CIO,13,comedi_bufconfig)
#define COMEDI_BUFINFO _IOWR(CIO,14,comedi_bufinfo)
#define COMEDI_POLL _IO(CIO,15)


/* structures */

typedef struct comedi_trig_struct comedi_trig;
typedef struct comedi_cmd_struct comedi_cmd;
typedef struct comedi_insn_struct comedi_insn;
typedef struct comedi_insnlist_struct comedi_insnlist;
typedef struct comedi_chaninfo_struct comedi_chaninfo;
typedef struct comedi_subdinfo_struct comedi_subdinfo;
typedef struct comedi_devinfo_struct comedi_devinfo;
typedef struct comedi_devconfig_struct comedi_devconfig;
typedef struct comedi_rangeinfo_struct comedi_rangeinfo;
typedef struct comedi_krange_struct comedi_krange;
typedef struct comedi_bufconfig_struct comedi_bufconfig;
typedef struct comedi_bufinfo_struct comedi_bufinfo;

struct comedi_trig_struct{
	unsigned int subdev;		/* subdevice */
	unsigned int mode;		/* mode */
	unsigned int flags;
	unsigned int n_chan;		/* number of channels */
	unsigned int *chanlist; 	/* channel/range list */
	sampl_t *data;			/* data list, size depends on subd flags */
	unsigned int n;			/* number of scans */
	unsigned int trigsrc;
	unsigned int trigvar;
	unsigned int trigvar1;
	unsigned int data_len;
	unsigned int unused[3];
};

struct comedi_insn_struct{
	unsigned int insn;
	unsigned int n;
	lsampl_t *data;
	unsigned int subdev;
	unsigned int chanspec;
	unsigned int unused[3];
};

struct comedi_insnlist_struct{
	unsigned int n_insns;
	comedi_insn *insns;
};

struct comedi_cmd_struct{
	unsigned int subdev;
	unsigned int flags;

	unsigned int start_src;
	unsigned int start_arg;

	unsigned int scan_begin_src;
	unsigned int scan_begin_arg;

	unsigned int convert_src;
	unsigned int convert_arg;

	unsigned int scan_end_src;
	unsigned int scan_end_arg;

	unsigned int stop_src;
	unsigned int stop_arg;

	unsigned int *chanlist; 	/* channel/range list */
	unsigned int chanlist_len;

	sampl_t *data;			/* data list, size depends on subd flags */
	unsigned int data_len;
};

struct comedi_chaninfo_struct{
	unsigned int subdev;
	lsampl_t *maxdata_list;
	unsigned int *flaglist;
	unsigned int *rangelist;
	unsigned int unused[4];
};

struct comedi_rangeinfo_struct{
	unsigned int range_type;
	void *range_ptr;
};

struct comedi_krange_struct{
	int min;	/* fixed point, multiply by 1e-6 */
	int max;	/* fixed point, multiply by 1e-6 */
	unsigned int flags;
};

struct comedi_subdinfo_struct{
	unsigned int type;
	unsigned int n_chan;
	unsigned int subd_flags;
	unsigned int timer_type;
	unsigned int len_chanlist;
	lsampl_t	maxdata;
	unsigned int	flags;		/* channel flags */
	unsigned int	range_type;	/* lookup in kernel */
	unsigned int unused[10];
};

struct comedi_devinfo_struct{
	unsigned int version_code;
	unsigned int n_subdevs;
	char driver_name[COMEDI_NAMELEN];
	char board_name[COMEDI_NAMELEN];
	int read_subdevice;
	int write_subdevice;
	int unused[30];
};

struct comedi_devconfig_struct{
	char board_name[COMEDI_NAMELEN];
	int options[COMEDI_NDEVCONFOPTS];
};

struct comedi_bufconfig_struct{
	unsigned int subdevice;
	unsigned int flags;

	unsigned int maximum_size;
	unsigned int size;

	unsigned int unused[4];
};

struct comedi_bufinfo_struct{
	unsigned int subdevice;
	unsigned int bytes_read;

	unsigned int buf_int_ptr;
	unsigned int buf_user_ptr;
	unsigned int buf_int_count;
	unsigned int buf_user_count;

	unsigned int unused[5];
};

/* range stuff */

#define __RANGE(a,b)	((((a)&0xffff)<<16)|((b)&0xffff))

#define RANGE_OFFSET(a)		(((a)>>16)&0xffff)
#define RANGE_LENGTH(b)		((b)&0xffff)

#define RF_UNIT(flags)		((flags)&0xff)
#define RF_EXTERNAL		(1<<8)

#define UNIT_volt		0
#define UNIT_mA			1
#define UNIT_none		2

#define COMEDI_MIN_SPEED	((unsigned int)0xffffffff)

/* callback stuff */
/* only relevant to kernel modules. */

#define COMEDI_CB_EOS		1	/* end of scan */
#define COMEDI_CB_EOA		2	/* end of acquisition */
#define COMEDI_CB_BLOCK		4	/* convenient block size */
#define COMEDI_CB_EOBUF		8	/* end of buffer */
#define COMEDI_CB_ERROR		16	/* card error during acquisition */


#ifdef __cplusplus
}
#endif

#endif /* _COMEDI_H */

