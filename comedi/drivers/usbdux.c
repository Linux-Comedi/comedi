#define DRIVER_VERSION "v0.95"
#define DRIVER_AUTHOR "Bernd Porr, Bernd.Porr@cn.stir.ac.uk"
#define DRIVER_DESC "Stirling/ITL USB-DUX -- Bernd.Porr@cn.stir.ac.uk"
/*
   module/usbdux.c
   Copyright (C) 2003 Bernd Porr, Bernd.Porr@cn.stir.ac.uk

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
Driver: usbdux.c
Description: University of Stirling USB DAQ & INCITE Technology Limited
Devices: [ITL] USB-DUX (usbdux.o)
Author: Bernd Porr <Bernd.Porr@cn.stir.ac.uk>
Updated: Sun Oct 12
Status: testing

Configuration Options:
  -i /usr/share/usb/usbdux_firmware.hex
*/
/*
 * I must give credit here to Chris Baugher who
 * wrote the driver for AT-MIO-16d. I used some parts of this
 * driver. I also must give credits to David Brownell
 * who supported me with the USB development.
 *
 * Bernd Porr
 *
 *
 * Revision history:
 * 0.94: D/A output should work now with any channel list combinations
 * 0.95: .owner commented out for kernel vers below 2.4.19
 *       sanity checks in ai/ao_cmd
 *
 *
 * Todo:
 * - use gpif of the FX2 to transfer digital data to the host
 * - use EP1in/out for control messages and leave the "better ones" for
 *   digital I/O
 */


//#define  COMEDI_IN_KERNEL_PATH
//#define  CONFIG_COMEDI_DEBUG


#include <linux/version.h>

#if LINUX_VERSION_CODE < 0x020600
/////////////////////////////////////////////////////////////////////
// Linux 2.4.2x


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/usb.h>
#include <linux/smp_lock.h>
#include <linux/fcntl.h>

#ifdef COMEDI_IN_KERNEL_PATH
#include <comedi/comedidev.h>
#else
#include <linux/comedidev.h>
#endif

#define BOARDNAME "usbdux"

// timeout for the USB-transfer
#define EZTIMEOUT 10

// constants for "firmware" upload and download
#define USBDUXSUB_FIRMWARE 0xA0
#define VENDOR_DIR_IN  0xC0
#define VENDOR_DIR_OUT 0x40

// internal adresses of the 8051 processor
#define USBDUXSUB_CPUCS 0xE600

// the minor device number, major is 180
// only for debugging purposes and to
// upload special firmware (programming the
// eeprom etc) which is not compatible with
// the comedi framwork
#define USBDUXSUB_MINOR 32

// max lenghth of the transfer-buffer for software upload
#define TB_LEN 0x2000

// Input endpoint number: ISO/IRQ
#define ISOINEP           6

#define IRQINEP           6

// Output endpoint number: ISO/IRQ
#define ISOOUTEP          2

#define IRQOUTEP          2

// Endpoint for the A/D channellist: bulk OUT
#define CHANNELLISTEP     4

// Endpoint for a single A/D aquisition: bulk IN
#define ADSINGLEEP        8

// Number of channels
#define NUMCHANNELS       8

// Size of one A/D value
#define SIZEADIN          ((sizeof(int16_t)))

// Size of the input-buffer IN BYTES
// Always multiple of 8 for 8 microframes which is needed in the highspeed mode
#define SIZEINBUF         ((8*SIZEADIN))

// 16 bytes.
#define SIZEINSNBUF       16

// Number of DA channels
#define NUMOUTCHANNELS    8

// size of one value for the D/A converter: channel and value
#define SIZEDAOUT          ((sizeof(int8_t)+sizeof(int16_t)))

// Size of the output-buffer in bytes
// Actually only the first 4 triplets are used but for the
// high speed mode we need to pad it to 8 (microframes).
#define SIZEOUTBUF         ((8*SIZEDAOUT))

// Size of the buffer for the dux commands: just now max size is determined
// by the analogue out + command byte + panic byte...
#define SIZEOFDUXBUFFER    ((8*SIZEDAOUT+2))

// Number of in-URBs which receive the data: min=2
#define NUMOFINBUFFERSFULL     4

// Number of out-URBs which send the data: min=2
#define NUMOFOUTBUFFERSFULL    4

// Total number of usbdux devices
#define NUMUSBDUX             16


// Number of subdevices
#define N_SUBDEVICES          3

// Analogue in subdevice
#define SUBDEV_AD             0

// Analogue out subdevice
#define SUBDEV_DA             1

// Digital I/O
#define SUBDEV_DIO            2


/////////////////////////////////////////////
// comedi constants
static comedi_lrange range_usbdux_ai_range = { 4, {
        BIP_RANGE( 4.096 ),
        BIP_RANGE( 4.096/2 ),
        UNI_RANGE( 4.096 ),
        UNI_RANGE( 4.096/2 )
} };


static comedi_lrange range_usbdux_ao_range = { 2, {
        BIP_RANGE( 4.096 ),
        UNI_RANGE( 4.096 ),
} };




/*
 * private structure of one subdevice
 */

// This is the structure which holds all the data of this driver
// one sub device just now: A/D
typedef struct {
	// attached?
	int attached;
	// is it associated with a subdevice?
	int probed;
	// pointer to the usb-device
        struct usb_device *usbdev;
	// actual number of in-buffers
	int numOfInBuffers;
	// actual number of out-buffers
	int numOfOutBuffers;
        // ISO-transfer handling: buffers
        struct urb **urbIn;
	struct urb **urbOut;
	// input buffer for the ISO-transfer
        int16_t *inBuffer;
	// input buffer for single insn
	int16_t *insnBuffer;
	// output buffer for the ISO-transfer
	int16_t *outBuffer;
 	// interface number in the earlier versions
        int ifnum;
	// comedi device for the interrupt context
	comedi_device *comedidev;
	// is it USB_SPEED_HIGH or not?
	short int high_speed;
	// asynchronous command is running
	short int ai_cmd_running;
	short int ao_cmd_running;
	// continous aquisition
	short int ai_continous;
	short int ao_continous;
	// number of samples to aquire
	int ai_sample_count;
	int ao_sample_count;
	// time between samples in units of the timer
	unsigned int ai_timer;
	unsigned int ao_timer;
	// counter between aquisitions
	unsigned int ai_counter;
	unsigned int ao_counter;
	// A/D commands
	unsigned char *adc_commands;
	// D/A commands
	unsigned char *dac_commands;
	// commands
	unsigned char *dux_commands;
	short int ai_insn_running;
	short int ao_insn_running;
	short int dio_insn_running;
} usbduxsub_t;



// The pointer to the private usb-data of the driver
// is also the private data for the comedi-device.
// This has to be global as the usb subsystem needs
// global variables. The other reason is that this
// structure must be there _before_ any comedi
// command is issued. The usb subsystem must be
// initialised before comedi can access it.
// This global var will go away
// with kernel version 2.6.x.
static usbduxsub_t usbduxsub[NUMUSBDUX];




static int usbduxsub_unlink_InURBs(usbduxsub_t* usbduxsub_tmp) {
	int i,j;
	int err=0;

	for (i=0; i < usbduxsub_tmp->numOfInBuffers; i++) {
		j=usb_unlink_urb(usbduxsub_tmp->urbIn[i]);
		if (j<0) {
			err=j;
		}
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: usbdux: unlinked InURB %d: res=%d\n",
		       usbduxsub_tmp->comedidev->minor,
		       i,
		       j);
#endif
	}
	return err;
}



/* This will stop a running acquisition operation */
// Is called from within this driver from both the
// interrupt context and from comedi
static int usbdux_ai_stop(usbduxsub_t* this_usbduxsub,
			  int do_unlink)
{
	int ret=0;

	if (!this_usbduxsub) {
		printk("comedi?: usbdux_ai_stop: this_usbduxsub=NULL!\n");
		return -EFAULT;
	}


#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux_ai_stop\n",this_usbduxsub->comedidev->minor);
#endif

	if (do_unlink) {
		// stop aquistion
		ret=usbduxsub_unlink_InURBs(this_usbduxsub);
	}

	this_usbduxsub->ai_cmd_running=0;		

	return ret;
}



// This will cancel a running acquisition operation.
// This is called by comedi but never from inside the
// driver.
static int usbdux_ai_cancel(comedi_device *dev, 
			    comedi_subdevice *s)
{
	usbduxsub_t* this_usbduxsub;
	// force unlink of all urbs
#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux_ai_cancel\n",dev->minor);
#endif
	this_usbduxsub=dev->private;
	if (!this_usbduxsub) {
		printk("comedi%d: usbdux_ai_cancel: this_usbduxsub=NULL\n",
		       this_usbduxsub->comedidev->minor);
		return -EFAULT;
	}
	// unlink only if the urb really has been submitted
	return usbdux_ai_stop(this_usbduxsub,this_usbduxsub->ai_cmd_running);
}




// analogue IN
// interrupt service routine
static void usbduxsub_ai_IsocIrq(struct urb *urb)
{
	int i;
	usbduxsub_t* this_usbduxsub;
	comedi_device *this_comedidev;
	comedi_subdevice *s;

	// sanity checks
	// is the urb there?
	if (!urb) {
		printk("comedi_: usbdux_: ao int-handler called with urb=NULL!\n");
		return;
	}

	// the context variable points to the subdevice
	this_comedidev=urb->context;
	if (!this_comedidev) {
		printk("comedi_: usbdux_: urb context is a NULL pointer!\n");
		return;
	}

	// the private structure of the subdevice is usbduxsub_t
	this_usbduxsub=this_comedidev->private;
	if (!this_usbduxsub) {
		printk("comedi_: usbdux_: private of comedi subdev is a NULL pointer!\n");
		return;
	}

	s=this_comedidev->subdevices + SUBDEV_AD;

	// first we test if something unusual has just happend
	switch (urb->status) {
        case 0:
                // success
		// copy the result in the transfer buffer
		memcpy(this_usbduxsub->inBuffer,
		       urb->transfer_buffer,
		       SIZEINBUF);
		break;
 	case -EILSEQ:
		// error in the ISOchronous data
		// we don't copy the data into the transfer buffer
		// and recycle the last data byte
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: usbdux: CRC error in ISO IN stream.\n",
		       this_usbduxsub->comedidev->minor);
#endif
	
		break;

		// happens after an unlink command
        case -ECONNRESET:
        case -ENOENT:
        case -ESHUTDOWN:
	case -ECONNABORTED:
		if (this_usbduxsub->ai_cmd_running) {
			// we are still running a command
			// tell this comedi
			s->async->events |= COMEDI_CB_EOA;
			s->async->events |= COMEDI_CB_ERROR;
			comedi_event(this_usbduxsub->comedidev, 
				     s,
				     s->async->events);
			this_usbduxsub->ai_cmd_running=0;
			usbdux_ai_stop(this_usbduxsub,0);
		}
		return;

 
		// a real error on the bus
	default:
		// pass error to comedi if we are really running a command
		if (this_usbduxsub->ai_cmd_running) {
			printk("Non-zero urb status received in ai intr context: %d\n",
			       urb->status);
			s->async->events |= COMEDI_CB_EOA;
			s->async->events |= COMEDI_CB_ERROR;
			comedi_event(this_usbduxsub->comedidev, 
				     s, 
				     s->async->events);
			// do an unlink if it's IRQ transfer in high speed
			usbdux_ai_stop(this_usbduxsub,
				       this_usbduxsub->high_speed);
		}
		return;
        }

	// at this point we are reasonably sure that nothing dodgy has happened
	// are we running a command?
	if (!(this_usbduxsub->ai_cmd_running)) {
		// not running a command
		if (this_usbduxsub->high_speed) {
			printk("comedi%d: usbdux: stopping IRQ transfer in IRQ handler.\n",
			       this_usbduxsub->comedidev->minor);
			// force unlink of the IN URBs
			usbdux_ai_stop(this_usbduxsub,1);
			// signal the automagic code to abort
			urb->status = -ECONNRESET;
		}
		// do not continue execution if no asynchronous command is running
		return;
	}

	// really executing a command in this subdevice without USB errors
	this_usbduxsub->ai_counter--;
	if (this_usbduxsub->ai_counter<=0) {
		// timer zero, transfer measurements to comedi
		this_usbduxsub->ai_counter=this_usbduxsub->ai_timer;

		// test, if we are transmit only a fixed number of samples
		if (!(this_usbduxsub->ai_continous)) {
			// not continous, fixed number of samples
			this_usbduxsub->ai_sample_count--;
			if (this_usbduxsub->ai_sample_count<0){
				// all samples transmitted to comedi
				usbdux_ai_stop(this_usbduxsub,
					       0);
				if (this_usbduxsub->high_speed) {
					// signal the automagic code to abort
					// implicit unlinking: prevents resubmit
					urb->status = -ECONNRESET;
				}
				// say comedi the the acquistion is over
				s->async->events |= COMEDI_CB_EOA;
				comedi_event(this_usbduxsub->comedidev, 
					     s, 
					     s->async->events);
				return;
			}
		}

		// get the data from the USB bus and hand it over
		// to comedi
		for(i=0;i<s->async->cmd.chanlist_len;i++) {
			// transfer data
			if (CR_RANGE(s->async->cmd.chanlist[i])<=1) {
				comedi_buf_put
					(s->async, 
					 (this_usbduxsub->inBuffer[i])^0x800);
			} else {
				comedi_buf_put
					(s->async, 
					 this_usbduxsub->inBuffer[i]);
			}
		}
		// tell comedi that data is there
		comedi_event(this_usbduxsub->comedidev, 
			     s, 
			     s->async->events);
	}

	// prepare the next acquistion
	if (!this_usbduxsub->high_speed) {
		// it's an ISO transfer we have to resubmit
		// are we still running a command?
		if (this_usbduxsub->ai_cmd_running) {
			// command is still running
			// resubmit urb for ISO transfer
			urb->dev = this_usbduxsub->usbdev;
			if (usb_submit_urb(urb)<0) {
				printk("comedi_: usbdux_: urb resubmit failed in int-context!\n");
			}
		}
	} else {
		// it's an IRQ transfer. No need to resubmit in 2.4. It's automagic.
		/*printk("usbduxsub_ai_IsocIrq: urbIn[0]=%x, urbIn[0]->dev=%x\n",
		       (int)(this_usbduxsub->urbIn[0]),
		       (int)(this_usbduxsub->urbIn[0]->dev));*/

	}
}





static int usbduxsub_unlink_OutURBs(usbduxsub_t* usbduxsub_tmp) {
	int i,j;
	int err=0;

	for (i=0; i < usbduxsub_tmp->numOfOutBuffers; i++) {
		j=usb_unlink_urb(usbduxsub_tmp->urbOut[i]);
		if (j<err) {
			err=j;
		}
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: usbdux: unlinked OutURB %d: res=%d\n",
		       usbduxsub_tmp->comedidev->minor,
		       i,
		       j);
#endif
	}
	return err;
}





/* This will cancel a running acquisition operation
 * in any context.
 */
static int usbdux_ao_stop(usbduxsub_t* this_usbduxsub,
			  int do_unlink) {
	int ret=0;

	if (!this_usbduxsub) {
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi?: usbdux_ao_stop: this_usbduxsub=NULL!\n");
#endif
		return -EFAULT;
	}

#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux_ao_cancel\n",this_usbduxsub->comedidev->minor);
#endif
	if (do_unlink) {
		ret=usbduxsub_unlink_OutURBs(this_usbduxsub);
	}
	this_usbduxsub->ao_cmd_running=0;
	return ret;
}



// force unlink
// is called by comedi
static int usbdux_ao_cancel(comedi_device *dev,
			    comedi_subdevice *s) {
	usbduxsub_t* this_usbduxsub=dev->private;
	if (!this_usbduxsub) {
		printk("comedi%d: usbdux_ao_cancel: this_usbduxsub=NULL\n",
		       this_usbduxsub->comedidev->minor);
		return -EFAULT;
	}
	// unlink only if it is really running
	return usbdux_ao_stop(this_usbduxsub,this_usbduxsub->ao_cmd_running);
}





static void usbduxsub_ao_IsocIrq(struct urb *urb)
{
	int i,ret;
	unsigned char* datap;
	usbduxsub_t* this_usbduxsub;
	comedi_device *this_comedidev;
	comedi_subdevice *s;

	if (!urb) {
		printk("comedi_: usbdux_: ao urb handler called with NULL ptr.\n");
		return;
	}

	// the context variable points to the subdevice
	this_comedidev=urb->context;
	if (!this_comedidev) {
		printk("comedi_: usbdux_: ao urb int-context is a NULL pointer.\n");
		return;
	}

	// the private structure of the subdevice is usbduxsub_t
	this_usbduxsub=this_comedidev->private;
	if (!this_usbduxsub) {
		printk("comedi_: usbdux_: private data structure of ao subdev is NULL p.\n");
		return;
	}

	s=this_comedidev->subdevices + SUBDEV_DA;

	switch (urb->status) {
        case 0:
                /* success */
                break;

		// after an unlink command, unplug, ... etc
		// no unlink needed here. Already shutting down.
        case -ECONNRESET:
        case -ENOENT:
        case -ESHUTDOWN:
	case -ECONNABORTED:
		if (this_usbduxsub->ao_cmd_running) {
			s->async->events |= COMEDI_CB_EOA;
			comedi_event(this_usbduxsub->comedidev, 
				     s, 
				     s->async->events);
			usbdux_ao_stop(this_usbduxsub,0);
		}
		return;

		// a real error
        default:
		if (this_usbduxsub->ao_cmd_running) {
			printk("comedi_: usbdux_: Non-zero urb status received in ao intr context: %d\n",
			       urb->status);
			s->async->events |= COMEDI_CB_ERROR;
			s->async->events |= COMEDI_CB_EOA;
			comedi_event(this_usbduxsub->comedidev, 
				     s, 
				     s->async->events);
		        // we do an unlink if we are in the high speed mode
			usbdux_ao_stop(this_usbduxsub,this_usbduxsub->high_speed);
		}
		return;
        }

	if (!(this_usbduxsub->ao_cmd_running)) {
		if (this_usbduxsub->high_speed) {
			printk("comedi: usbdux: stopping IRQ transfer in int handler.\n");
			// force unlink of the OUT URBs
			usbdux_ao_stop(this_usbduxsub,1);
			// stop automagic
			urb->status = -ECONNRESET;
		}
		return;
	}

	// normal operation: executing a command in this subdevice
	this_usbduxsub->ao_counter--;
	if (this_usbduxsub->ao_counter<=0) { 
		// timer zero
		this_usbduxsub->ao_counter=this_usbduxsub->ao_timer;

		// handle non continous aquisition
		if (!(this_usbduxsub->ao_continous)) {
			// fixed number of samples
			this_usbduxsub->ao_sample_count--;
			if (this_usbduxsub->ao_sample_count<0){
				// all samples transmitted
				usbdux_ao_stop(this_usbduxsub,
					       0);
				s->async->events |= COMEDI_CB_EOA;
				comedi_event(this_usbduxsub->comedidev, 
					     s, 
					     s->async->events);
				if (this_usbduxsub->high_speed) {
					// prevents resubmit of the
					// urb for IRQ transfers
					urb->status = -ECONNRESET;
				}
				return;
			}
		}

		// transmit data to the USB bus
		((unsigned char*)(urb->transfer_buffer))[0]=
			s->async->cmd.chanlist_len;
		for(i=0;i<s->async->cmd.chanlist_len;i++) {
			if (i>=NUMOUTCHANNELS) {
				break;
			}
			datap=&(((unsigned char*)urb->transfer_buffer)[i*3+1]);
			ret=comedi_buf_get
				(s->async, 
				 ((sampl_t*)datap));
			datap[2]=this_usbduxsub->dac_commands[i];
			/*printk("data[0]=%x, data[1]=%x, data[2]=%x\n",
			  datap[0],datap[1],datap[2]);*/
			if (ret<0) {
				printk("comedi: usbdux: buffer underflow\n");
				s->async->events |= COMEDI_CB_EOA;
				s->async->events |= COMEDI_CB_OVERFLOW;
			}
		// transmit data to comedi
		s->async->events |= COMEDI_CB_BLOCK;
		comedi_event(this_usbduxsub->comedidev, 
			     s, 
			     s->async->events);
		}
	}
	urb->transfer_buffer_length = SIZEOUTBUF;
	urb->dev = this_usbduxsub->usbdev;
	urb->status = 0;
	if (!(this_usbduxsub->high_speed)) {
		if (this_usbduxsub->ao_cmd_running) {	
			urb->interval=1;
			urb->number_of_packets = 1;		
			urb->iso_frame_desc[0].offset = 0;
			urb->iso_frame_desc[0].length = SIZEOUTBUF;
			urb->iso_frame_desc[0].status = 0;
			if ((ret=usb_submit_urb(urb))<0) {
				printk("comedi_: usbdux_: ao urb resubm failed in int-cont.");
				printk("ret=%d\n",ret);
			}
		}
	} else {
		// printk("comedi: usbdux: debug: ao irq\n");
	}
}


static int usbduxsub_start(usbduxsub_t* usbduxsub) {
	int errcode=0;
	unsigned char local_transfer_buffer[16];

	if (usbduxsub->probed) {
		// 7f92 to zero
		local_transfer_buffer[0]=0; 
		errcode=usb_control_msg
			(usbduxsub->usbdev,
			 // create a pipe for a control transfer
			 usb_sndctrlpipe(usbduxsub->usbdev,0),
			 // bRequest, "Firmware"
			 USBDUXSUB_FIRMWARE, 
			 // bmRequestType
			 VENDOR_DIR_OUT, 
			 // Value
			 USBDUXSUB_CPUCS, 
			 // Index
			 0x0000,
			 // address of the transfer buffer
			 local_transfer_buffer,
			 // Length
			 1,
			 // Timeout
			 EZTIMEOUT 
			 );
		if (errcode<0) {
			printk("comedi_: usbdux_: control msg failed (start)\n");
			return errcode;
		}
	}
	return 0;
}




static int usbduxsub_stop(usbduxsub_t* usbduxsub) {
	int errcode=0;

	unsigned char local_transfer_buffer[16];
	if (usbduxsub->probed) {
		// 7f92 to one
		local_transfer_buffer[0]=1; 
		errcode=usb_control_msg
			(usbduxsub->usbdev,
			 usb_sndctrlpipe(usbduxsub->usbdev,0),
			 // bRequest, "Firmware"
			 USBDUXSUB_FIRMWARE, 
			 // bmRequestType
			 VENDOR_DIR_OUT,
			 // Value
			 USBDUXSUB_CPUCS,
			 // Index
			 0x0000, 
			 local_transfer_buffer,
			 // Length
			 1,
			 // Timeout
			 EZTIMEOUT 
			 );
		if (errcode<0) {
			printk("comedi_: usbdux: control msg failed (stop)\n");
			return errcode;
		}
	}
	return 0;
}
		




static int usbduxsub_upload(usbduxsub_t* usbduxsub,
			    unsigned char* local_transfer_buffer,
			    unsigned int startAddr,
			    unsigned int len) {
	int errcode;

	if (usbduxsub->probed) {
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: usbdux: uploading %d bytes",
		       usbduxsub->comedidev->minor,len);
		printk(" to addr %d, first byte=%d.\n",
		       startAddr,
		       local_transfer_buffer[0]);
#endif
		errcode=usb_control_msg
			(usbduxsub->usbdev,
			 usb_sndctrlpipe(usbduxsub->usbdev, 0),
			 // brequest, firmware
			 USBDUXSUB_FIRMWARE,
			 // bmRequestType
			 VENDOR_DIR_OUT,
			 // value
			 startAddr,
			 // index
			 0x0000,
			 // our local safe buffer
			 local_transfer_buffer,
			 // length
			 len,
			 // timeout
			 EZTIMEOUT
			 );
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi_: usbdux: result=%d\n",errcode);
#endif
		if (errcode<0) {
			printk("comedi_: usbdux: uppload failed\n");
			return errcode;
		}
	} else {
		// no device on the bus for this index
		return -EFAULT;
	}
return 0;
}





int firmwareUpload(usbduxsub_t* usbduxsub, 
		   unsigned char* firmwareBinary,
		   int sizeFirmware) {
	int ret;

	if (!firmwareBinary) {
		return 0;
	}
	ret=usbduxsub_stop(usbduxsub);
	if (ret<0) {
		printk("comedi_: usbdux: can not stop firmware\n"
		       );
		return ret;
	}
	ret=usbduxsub_upload(usbduxsub,
			     firmwareBinary,
			     0,
			     sizeFirmware);
	if (ret<0) {
		printk("comedi_: usbdux: firmware upload failed\n"
		       );
		return ret;
	}
	ret=usbduxsub_start(usbduxsub);
	if (ret<0) {
		printk("comedi_: usbdux: can not start firmware\n"
		       );
		return ret;
	}
	return 0;
}
	


int usbduxsub_submit_InURBs(usbduxsub_t* usbduxsub) {
	int i,errFlag;

	/* Submit all URBs and start the transfer on the bus */
	for (i=0; i < usbduxsub->numOfInBuffers; i++) {
		// in case of a resubmission after an unlink...
		usbduxsub->urbIn[i]->context=usbduxsub->comedidev;
		usbduxsub->urbIn[i]->dev = usbduxsub->usbdev;
		usbduxsub->urbIn[i]->status = 0;
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: usbdux: submitting in-urb[%d]: %x,%x\n",
		       usbduxsub->comedidev->minor,
		       i,
		       (int)(usbduxsub->urbIn[i]->context),
		       (int)(usbduxsub->urbIn[i]->dev));
#endif
		errFlag = usb_submit_urb
			(usbduxsub->urbIn[i]);
		if (errFlag) {
			printk("comedi_: usbdux: ai: ");
			printk("USB_SUBMIT_URB(%d)",i);
			printk(" error %d\n",errFlag);
			return errFlag;
		}
	}
	return 0;
}




int usbduxsub_submit_OutURBs(usbduxsub_t* usbduxsub) {
	int i,errFlag;

	for (i=0; i < usbduxsub->numOfOutBuffers; i++) {
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi_: usbdux: submitting out-urb[%d]\n",i);
#endif
		// in case of a resubmission after an unlink...
		usbduxsub->urbOut[i]->context=usbduxsub->comedidev;
		usbduxsub->urbOut[i]->dev = usbduxsub->usbdev;
		usbduxsub->urbOut[i]->status = 0;
		errFlag = usb_submit_urb
			(usbduxsub->urbOut[i]);
		if (errFlag) {
			printk("comedi_: usbdux: ao: ");
			printk("USB_SUBMIT_URB(%d)",i);
			printk(" error %d\n",errFlag);
			return errFlag;
		}
	}
	return 0;
}





// allocate memory for the urbs and initialise them
static void* usbduxsub_probe(struct usb_device *this_usbdev, 
			     unsigned int ifnum,
			     const struct usb_device_id *id) {
        int i;
	int index;

#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi_: usbdux_: finding a free structure for the usb-device\n");
#endif
	index=-1;
	for(i=0; i<NUMUSBDUX; i++) {
		if (!(usbduxsub[i].probed)) {
			index=i;
			break;
		}
	}

	if (index==-1) {
		printk("More than %d devices attached to the bus...\n",
		       NUMUSBDUX);
		return NULL;
	}

#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi_: usbdux: found a free usb device: index=%d\n",index);
#endif
	usbduxsub[index].usbdev = this_usbdev;
        usbduxsub[index].ifnum = ifnum;
	usbduxsub[index].probed=1;
	usbduxsub[index].inBuffer=kmalloc(SIZEINBUF,GFP_KERNEL);
	memset(usbduxsub[index].inBuffer,0,SIZEINBUF);
	usbduxsub[index].insnBuffer=kmalloc(SIZEINSNBUF,GFP_KERNEL);
	memset(usbduxsub[index].insnBuffer,0,SIZEINSNBUF);
	usbduxsub[index].outBuffer=kmalloc(SIZEOUTBUF,GFP_KERNEL);
	memset(usbduxsub[index].outBuffer,0,SIZEOUTBUF);
	usbduxsub[index].high_speed=(usbduxsub[index].usbdev->speed==USB_SPEED_HIGH);

	if (!(usbduxsub[index].inBuffer)) {
		printk("comedi_: usbdux%d: could not alloc space for inBuffer\n",index);

		return NULL;
	}

	if (!(usbduxsub[index].insnBuffer)) {
		printk("comedi_: usbdux%d: could not alloc space for insnBuffer\n",index);

		return NULL;
	}

	if (!(usbduxsub[index].outBuffer)) {
		printk("comedi_: usbdux%d: could not alloc space for outBuffer\n",index);
		return NULL;
	}

	memset(usbduxsub[index].outBuffer,0,SIZEOUTBUF);
                                                                               
	if (usbduxsub[index].high_speed) {
		// high speed: Interrupt transfer
		i=usb_set_interface(usbduxsub[index].usbdev,ifnum,2);
		if (i<0) {
			printk("comedi_: usbdux%d: could not set alternate setting 2 in high speed.\n",index);
			return NULL;
		}

#ifdef CONFIG_COMEDI_DEBUG
		else {
			printk("comedi_: usbdux%d: set alternate setting 2 in high speed.\n",index);
		}	
#endif		
		usbduxsub[index].numOfInBuffers=1;
		usbduxsub[index].urbIn=kmalloc(sizeof(struct urb*)*usbduxsub[index].numOfInBuffers,
						 GFP_KERNEL);


		usbduxsub[index].urbIn[0]=usb_alloc_urb(0);
		if (usbduxsub[index].urbIn[0]==NULL) {
			printk("comedi_: usbdux%d: Could not alloc. in IRQ-urb(%d)\n",index,i);
			return NULL;
		}
		usb_fill_int_urb (usbduxsub[index].urbIn[0],
				  usbduxsub[index].usbdev,
				  usb_rcvintpipe(usbduxsub[index].usbdev,IRQINEP),
				  usbduxsub[index].inBuffer,
				  SIZEINBUF,
				  usbduxsub_ai_IsocIrq,
				  NULL, // context
				  8     // interval in microframes
				  );
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi_: usbdux: probe: urbIn[0]=%x, urbIn[0]->dev=%x\n",
		       (int)(usbduxsub[index].urbIn[0]),
		       (int)(usbduxsub[index].urbIn[0]->dev));
#endif
	} else {
		// setting to alternate setting 3: enabling iso ep and bulk ep.
		i=usb_set_interface(usbduxsub[index].usbdev,ifnum,3);
		if (i<0) {
			printk("comedi_: usbdux%d: could not set alternate setting 3 in high speed.\n",index);
			return NULL;
		}
		usbduxsub[index].numOfInBuffers=NUMOFINBUFFERSFULL;
		usbduxsub[index].urbIn=kmalloc(sizeof(struct urb*)*usbduxsub[index].numOfInBuffers,
						 GFP_KERNEL);
		for (i=0; i < usbduxsub[index].numOfInBuffers; i++) {
			// one frame: 1ms
			usbduxsub[index].urbIn[i]=usb_alloc_urb(1);
			if (usbduxsub[index].urbIn[i]==NULL) {
				printk("comedi_: usbdux%d: Could not alloc. urb(%d)\n",index,i);
				return NULL;
			}		
			usbduxsub[index].urbIn[i]->dev = usbduxsub[index].usbdev;
			// will be filled later with a pointer to the comedi-device
			// and ONLY then the urb should be submitted
			usbduxsub[index].urbIn[i]->context = NULL;
			usbduxsub[index].urbIn[i]->pipe = 
				usb_rcvisocpipe(usbduxsub[index].usbdev, ISOINEP);
			usbduxsub[index].urbIn[i]->transfer_flags = USB_ISO_ASAP;
			usbduxsub[index].urbIn[i]->transfer_buffer=
				kmalloc(SIZEINBUF,GFP_KERNEL);
			if (!(usbduxsub[index].urbIn[i]->transfer_buffer)) {
				printk("comedi_: usbdux%d: could not alloc. transb.\n",index);
				return NULL;
			}
			usbduxsub[index].urbIn[i]->complete = usbduxsub_ai_IsocIrq;
			usbduxsub[index].urbIn[i]->interval=1;
			usbduxsub[index].urbIn[i]->number_of_packets = 1;
			usbduxsub[index].urbIn[i]->transfer_buffer_length = SIZEINBUF;
			usbduxsub[index].urbIn[i]->iso_frame_desc[0].offset = 0;
			usbduxsub[index].urbIn[i]->iso_frame_desc[0].length = SIZEINBUF;
		}
	}


	// ISO out transfer
	if (usbduxsub[index].high_speed) {
		// high speed: Interrupt transfer, buffering by the firmware
		usbduxsub[index].numOfOutBuffers=1;
		usbduxsub[index].urbOut=kmalloc(sizeof(struct urb*)*usbduxsub[index].numOfOutBuffers,
						  GFP_KERNEL);
		usbduxsub[index].urbOut[0]=usb_alloc_urb(0);
		if (usbduxsub[index].urbOut[0]==NULL) {
			printk("comedi_: usbdux%d: Could not alloc. out IRQ-urb(%d)\n",index,i);

			return NULL;
		}
		usb_fill_int_urb (usbduxsub[index].urbOut[0],
				  usbduxsub[index].usbdev,
				  usb_sndintpipe(usbduxsub[index].usbdev,IRQOUTEP),
				  usbduxsub[index].outBuffer,
				  SIZEOUTBUF,
				  usbduxsub_ao_IsocIrq,
				  NULL, // context
				  8     // interval in microframes
				  );
	} else {
		// USB 1.1 allows any number of buffers
		usbduxsub[index].numOfOutBuffers=NUMOFOUTBUFFERSFULL;	
		usbduxsub[index].urbOut=
			kmalloc(sizeof(struct urb*)*usbduxsub[index].numOfOutBuffers,
				GFP_KERNEL);
		for (i=0; i < usbduxsub[index].numOfOutBuffers; i++) {
			// one frame: 1ms
			usbduxsub[index].urbOut[i]=usb_alloc_urb(1);
			if (usbduxsub[index].urbOut[i]==NULL) {
				printk("comedi_: usbdux%d: Could not alloc. urb(%d)\n",index,i);
				return NULL;
			}				
			usbduxsub[index].urbOut[i]->dev = usbduxsub[index].usbdev;
			// will be filled later with a pointer to the comedi-device
			// and ONLY then the urb should be submitted
			usbduxsub[index].urbOut[i]->context = NULL;
			usbduxsub[index].urbOut[i]->pipe = 
				usb_sndisocpipe(usbduxsub[index].usbdev, ISOOUTEP);
			usbduxsub[index].urbOut[i]->transfer_flags = USB_ISO_ASAP;
			usbduxsub[index].urbOut[i]->transfer_buffer=
				kmalloc(SIZEOUTBUF,GFP_KERNEL);
			if (!(usbduxsub[index].urbOut[i]->transfer_buffer)) {
				printk("comedi_: usbdux%d: could not alloc. transb.\n",index);
				return NULL;
			}
			usbduxsub[index].urbOut[i]->complete = usbduxsub_ao_IsocIrq;
			usbduxsub[index].urbOut[i]->interval=1;
			usbduxsub[index].urbOut[i]->number_of_packets = 1;
			usbduxsub[index].urbOut[i]->transfer_buffer_length = 
				SIZEOUTBUF;
			usbduxsub[index].urbOut[i]->iso_frame_desc[0].offset = 0;
			usbduxsub[index].urbOut[i]->iso_frame_desc[0].length = 
				SIZEOUTBUF;
		}
	}

	printk("comedi_: usbdux%d has been successfully initialized.\n",index);
	return (void*)(&usbduxsub[index]);
}





static void usbduxsub_disconnect(struct usb_device *dev, void *ptr) {
	int i;
	// get a pointer from an allocated structure

	usbduxsub_t* usbduxsub_tmp=(usbduxsub_t*)ptr;

	if (usbduxsub_tmp->usbdev!=dev) {
		printk("comedi_: usbdux: BUG! called with wrong ptr!!!\n");
		return;
	}
#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux: disconnected from the usb\n",
	       usbduxsub_tmp->comedidev->minor);
#endif
	if (usbduxsub_tmp->attached) {
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: usbdux: stopping all transfers\n",
		       usbduxsub_tmp->comedidev->minor);
#endif
		if (usbduxsub_tmp->ai_cmd_running) {
			usbduxsub_tmp->ai_cmd_running=0;
			usbduxsub_unlink_InURBs(usbduxsub_tmp);
		}
		if (usbduxsub_tmp->ao_cmd_running) {
			usbduxsub_tmp->ao_cmd_running=0;
			usbduxsub_unlink_OutURBs(usbduxsub_tmp);
		}
	}
	for (i=0; i < usbduxsub_tmp->numOfInBuffers; i++) {
		if (usbduxsub_tmp->urbIn[i]->transfer_buffer) {
			kfree(usbduxsub_tmp->urbIn[i]->transfer_buffer);
			usbduxsub_tmp->urbIn[i]->transfer_buffer=NULL;
		}
		usb_free_urb (usbduxsub_tmp->urbIn[i]);
	}
	for (i=0; i < usbduxsub_tmp->numOfOutBuffers; i++) {
		if (usbduxsub_tmp->urbOut[i]->transfer_buffer) {
			kfree(usbduxsub_tmp->urbOut[i]->transfer_buffer);
			usbduxsub_tmp->urbOut[i]->transfer_buffer=NULL;
		}
		usb_free_urb (usbduxsub_tmp->urbOut[i]);
	}
	usbduxsub_tmp->numOfOutBuffers=0;
	usbduxsub_tmp->numOfInBuffers=0;
	if (usbduxsub_tmp->inBuffer) {
		kfree(usbduxsub_tmp->inBuffer);
		usbduxsub_tmp->inBuffer=NULL;
	}
	if (usbduxsub_tmp->insnBuffer) {
		kfree(usbduxsub_tmp->insnBuffer);
		usbduxsub_tmp->insnBuffer=NULL;
	}
	if (usbduxsub_tmp->inBuffer) {
		kfree(usbduxsub_tmp->inBuffer);
		usbduxsub_tmp->inBuffer=NULL;
	}
	if (usbduxsub_tmp->urbIn) {
		kfree(usbduxsub_tmp->urbIn);
		usbduxsub_tmp->urbIn=NULL;
	}
	if (usbduxsub_tmp->urbOut) {
		kfree(usbduxsub_tmp->urbOut);
		usbduxsub_tmp->urbOut=NULL;
	}	
	usbduxsub_tmp->probed=0;
}





///////////////////////////////////////////////////////////////////////////
// comedi stuff



static int usbdux_attach(comedi_device * dev, comedi_devconfig * it);
static int usbdux_detach(comedi_device * dev);




/* main driver struct */
static comedi_driver driver_usbdux={
        driver_name:    "usbdux",
        module:         THIS_MODULE,
        attach:         usbdux_attach,
        detach:         usbdux_detach,
};


static int usbdux_ai_cmdtest(comedi_device *dev, 
			     comedi_subdevice *s, 
			     comedi_cmd *cmd)
{
	int err=0, tmp;

#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux_ai_cmdtest\n",dev->minor);
#endif
	/* make sure triggers are valid */
	// Only immediate triggers are allowed
	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW|TRIG_INT;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	// trigger should happen timed
	tmp=cmd->scan_begin_src;
	// fixme
	// just now also in high speed we sample only every frame
	if (0) /* (this_usbduxsub->high_speed) */ {
		// start immidiately a new scan
		// the sampling rate is set by the coversion rate
		cmd->scan_begin_src &= TRIG_FOLLOW;
	} else {
		// start a new _scan_ with a timer
		cmd->scan_begin_src &= TRIG_TIMER;
	}
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	// scanning is continous
	tmp=cmd->convert_src;
	// for the host just now the sampling happens at once: every frame
	// (also for usb 2.0 due to ehci driver limitations)
	if (0) /* (this_usbduxsub->high_speed) */ {
		// in usb-2.0 only one conversion it tranmitted but with 8kHz/n
		cmd->convert_src &= TRIG_TIMER;
	} else {
		// all conversion events happen simultaneously with a rate of 1kHz/n
		cmd->convert_src &= TRIG_NOW;
	}		
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	// issue a trigger when scan is finished and start a new scan
	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	// trigger at the end of count events or not, stop condition or not
	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT|TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */
	/* note that mutual compatiblity is not an issue here */
	if(cmd->scan_begin_src!=TRIG_FOLLOW &&
	   cmd->scan_begin_src!=TRIG_EXT &&
	   cmd->scan_begin_src!=TRIG_TIMER)err++;
	if(cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_NONE)err++;

	if(err)return 2;


	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}

	if(cmd->scan_begin_src==TRIG_FOLLOW){
		/* internal trigger */
		if(cmd->scan_begin_arg!=0){
			cmd->scan_begin_arg=0;
			err++;
		}
	}

	// this is for 8kHz sampling rate (every microframe)
	// just now it's not supported by the ehci driver
	if (cmd->convert_src==TRIG_TIMER) {
		if(cmd->convert_arg<125000){
			cmd->convert_arg=125000;
			err++;
		}
	}

	// 1kHz scans every USB frame
	if(cmd->scan_begin_src==TRIG_TIMER){
		/* timer */
		if(cmd->scan_begin_arg<1000000){
			cmd->scan_begin_arg=1000000;
			err++;
		}
	}

	// the same argument
	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}

	
	if(cmd->stop_src==TRIG_COUNT){
		/* any count is allowed */
	}else{
		/* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}

	if(err)return 3;

	return 0;
}


// creates the ADC command for the MAX1271

static unsigned char create_adc_command(unsigned int chan, int polarity, int range) {
	return	(chan<<4)|
		((polarity==1)<<2)|
		((range==1)<<3);
}



// bulk transfers to usbdux

#define SENDADCOMMANDS            0
#define SENDDACOMMANDS            1
#define SENDDIOCONFIGCOMMAND      2
#define SENDDIOBITSCOMMAND        3

static int send_dux_commands(usbduxsub_t* this_usbduxsub,int cmd_type) {
	int result,nsent,i;
	comedi_subdevice *s;

	switch (cmd_type) {
	case SENDADCOMMANDS:
		// AD commands
		this_usbduxsub->dux_commands[0]=cmd_type;
		memcpy((this_usbduxsub->dux_commands)+1,
		       this_usbduxsub->adc_commands,
		       NUMCHANNELS);
		break;
	case SENDDACOMMANDS:
		// DA commands: send one channel to the USB board
		// Same format as in the synchronous case:
		// channel number + 16 bit value
		this_usbduxsub->dux_commands[0]=cmd_type;
		// number of channels: 1
		this_usbduxsub->dux_commands[1]=1;
		// one 16 bit value
		*((int16_t*)(this_usbduxsub->dux_commands+2))=this_usbduxsub->outBuffer[1];
		// channel number
		this_usbduxsub->dux_commands[4]=(this_usbduxsub->outBuffer[0]<<6);
		break;
	case SENDDIOCONFIGCOMMAND:
		// the firmware will ignore s->state and will set
		// only the direction
	case SENDDIOBITSCOMMAND:
		// sets the out bits at port B and write the data to it
		s=this_usbduxsub->comedidev->subdevices+SUBDEV_DIO;
		this_usbduxsub->dux_commands[0]=cmd_type;
		this_usbduxsub->dux_commands[1]=s->io_bits;
		this_usbduxsub->dux_commands[2]=s->state;
		break;
	default:
		printk("comedi%d: usbdux: illegal dux_command.\n",
		       this_usbduxsub->comedidev->minor);
		return -EFAULT;
	}
#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux: dux_commands: ",
	       this_usbduxsub->comedidev->minor);
	for(i=0;i<SIZEOFDUXBUFFER;i++) {
		printk(" %02x",this_usbduxsub->dux_commands[i]);
	}
	printk("\n");
#endif
	result = usb_bulk_msg(this_usbduxsub->usbdev,
			      usb_sndbulkpipe(this_usbduxsub->usbdev,
					      CHANNELLISTEP),
			      this_usbduxsub->dux_commands,
			      SIZEOFDUXBUFFER,
			      &nsent, 
			      10*HZ);
	if (result<0) {
		printk("comedi%d: could not transmit dux_commands to the usb-device, err=%d\n",
		       this_usbduxsub->comedidev->minor,result);
	}
	return result;
}



static int usbdux_ai_inttrig(comedi_device *dev, 
			     comedi_subdevice *s,
			     unsigned int trignum)
{
	int ret;
	usbduxsub_t* this_usbduxsub=dev->private;

#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux_ai_inttrig\n",dev->minor);
#endif

        if (trignum!=0) {
                printk("comedi%d: usbdux_ai_inttrig: invalid trignum\n",dev->minor);
		return -EINVAL;
	}
	if (!(this_usbduxsub->ai_cmd_running)) {
		this_usbduxsub->ai_cmd_running=1;
		ret=usbduxsub_submit_InURBs(this_usbduxsub);
		if (ret<0) {
			printk("comedi%d: usbdux_ai_inttrig: urbSubmit: err=%d\n",
			       dev->minor,ret);
			this_usbduxsub->ai_cmd_running=0;
			return ret;
		}
		s->async->inttrig = NULL;
	} else {
		printk("comedi%d: ai_inttrig but acqu is already running\n",
		       dev->minor);
	}
        return 1;
}









static int usbdux_ai_cmd(comedi_device *dev, comedi_subdevice *s)
{
	comedi_cmd *cmd = &s->async->cmd;
	unsigned int chan, gain;
	int i,ret;
	usbduxsub_t* this_usbduxsub=dev->private;
	int result;

#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux_ai_cmd\n",dev->minor);
#endif

	if (this_usbduxsub->ai_insn_running) {
		printk("comedi%d: ai_cmd not possible. Sync command is running.\n",
		       dev->minor);
		return -EBUSY;
	}	

	if (this_usbduxsub->ai_cmd_running) {
		printk("comedi%d: ai_cmd not possible. Another ai_cmd is running.\n",
		       dev->minor);
		return -EBUSY;
	}

	// set current channel of the running aquisition to zero
	s->async->cur_chan	= 0;
	
	for(i=0; i < cmd->chanlist_len; ++i ) {
		chan = CR_CHAN(cmd->chanlist[i]);
		gain = CR_RANGE(cmd->chanlist[i]);
		if (i>=NUMCHANNELS) {
			printk("comedi%d: channel list too long\n",dev->minor);
			break;
		}
		this_usbduxsub->adc_commands[i]=create_adc_command(chan,gain<=1,(gain%2)==0);
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: adc command for ch %d is %x\n",
		       dev->minor,
		       i,
		       this_usbduxsub->adc_commands[i]);
#endif
	}


#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi %d: sending commands to the usb device: ",
	       dev->minor);
	printk("size=%u\n",
	       NUMCHANNELS);
#endif
	// 0 means that the AD commands are sent
	result=send_dux_commands(this_usbduxsub,SENDADCOMMANDS);
	if (result<0) {
		printk("comedi%d: adc command could not be submitted. Aborting...\n",
		       dev->minor);
		return result;
	}		
	// we count in steps of 1ms (also in high speed mode)
	if (0) /* (this_usbduxsub->high_speed) */ {
		// 125us
		// timing of the conversion itself: every 125 us
		this_usbduxsub->ai_timer = cmd->convert_arg/125000;
      	} else {
		// 1ms
		// timing of the scan: we get all channels at once
		this_usbduxsub->ai_timer = cmd->scan_begin_arg/1000000;
                if (this_usbduxsub->ai_timer<1) { 
		  printk("comedi%d: usbdux: ai_cmd: timer=%d, scan_begin_arg=%d. Not properly tested by cmdtest?\n", 
			 dev->minor, 
			 this_usbduxsub->ai_timer,
			 cmd->scan_begin_arg); 
		  return -EINVAL;
                }
	}
	this_usbduxsub->ai_counter=this_usbduxsub->ai_timer;

	if(cmd->stop_src==TRIG_COUNT){
		// not continous
		// counter
		if (0) /* (this_usbduxsub->high_speed) */ {
			this_usbduxsub->ai_sample_count = 
				(cmd->stop_arg)*(cmd->scan_end_arg);
		} else {
			// there's no scan as the scan has been 
			// perf inside the FX2
			// data arrives as one packet
			this_usbduxsub->ai_sample_count = cmd->stop_arg;
		}		
		this_usbduxsub->ai_continous=0;		
	} else {
		// continous aquisition
		this_usbduxsub->ai_continous=1;
		this_usbduxsub->ai_sample_count=0;
	}


        if(cmd->start_src == TRIG_NOW){
		// enable this acquisition operation
		this_usbduxsub->ai_cmd_running=1;
		ret=usbduxsub_submit_InURBs(this_usbduxsub);
		if (ret<0) {
			this_usbduxsub->ai_cmd_running=0;
			// fixme: unlink here??
			return ret;
		}
                s->async->inttrig = NULL;
        }else{
                /* TRIG_INT */
		// don't enable the acquision operation
		// wait for an internal signal
                s->async->inttrig = usbdux_ai_inttrig;
        }
	return 0;
}



static int single_adc_conv(usbduxsub_t* this_usbduxsub,lsampl_t* value,int n) {
	int result=-EFAULT;
	int nrec;
	int i;
	
	for(i=0;i<n;i++) {
		result = usb_bulk_msg(this_usbduxsub->usbdev,
				      usb_rcvbulkpipe(this_usbduxsub->usbdev,
						      ADSINGLEEP),
				      this_usbduxsub->insnBuffer,
				      SIZEINSNBUF,
				      &nrec, 
				      1*HZ);
		if (result<0) {
			printk("comedi%d: insn: USB error %d while requesting AD data.\n",
			       this_usbduxsub->comedidev->minor,result);
			return result;
		}
		*value=((uint16_t*)(this_usbduxsub->insnBuffer))[0];
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: singleADC: i=%d, value=%d\n",
		       this_usbduxsub->comedidev->minor,
		       i,*value);
#endif
	}
	return result;
}






/* Mode 0 is used to get a single conversion on demand */
static int usbdux_ai_insn_read(comedi_device * dev, 
			       comedi_subdevice *s,
			       comedi_insn *insn, 
			       lsampl_t *data)
{
	int i;
	int twice;
	lsampl_t one=0;
	int chan,gain;
	int err;

	usbduxsub_t* this_usbduxsub=dev->private;

	if (!this_usbduxsub) {
		printk("comedi%d: ai_insn_read: no usb dev.\n",
		       dev->minor);
		return 0;
	}		

#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: ai_insn_read, insn->n=%d, insn->subdev=%d\n",
	       dev->minor,
	       insn->n,
	       insn->subdev);
#endif
	if (this_usbduxsub->ai_cmd_running) {
		printk("comedi%d: ai_insn_read not possible. Async Command is running.\n",
		       dev->minor);
		return 0;
	}

	if (this_usbduxsub->dio_insn_running) {
		printk("comedi%d:ai_insn_read not possible.dio is running.\n",
		       dev->minor);
		return 0;
	}

	// sample one channel
	chan = CR_CHAN(insn->chanspec);
	gain = CR_RANGE(insn->chanspec);
	// set command for the first channel
	this_usbduxsub->adc_commands[0]=create_adc_command(chan,gain<=1,(gain%2)==0);
#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: ai_insn_read, adc_command=%x\n",
	       dev->minor,
	       this_usbduxsub->adc_commands[0]);
#endif
	// adc commands
	err=send_dux_commands(this_usbduxsub,SENDADCOMMANDS);
	if (err<0) {
		printk("comedi%d: usb err =%d\n",
		       dev->minor,err);
		return 0;
	}

	// read the first byte two times
	twice=3;
	this_usbduxsub->ai_insn_running=1;
	for(i=0 ; i < insn->n ; i++) {
		err=single_adc_conv(this_usbduxsub,&one,twice);
		twice=1;
		if (err<0) {
			printk("comedi%d: insn. error: %d\n",dev->minor,err);
			this_usbduxsub->ai_insn_running=0;
			return 0;
		}
		if (CR_RANGE(insn->chanspec)<=1) {
			one=one^0x800;
		}
		data[i]=one;
	}
	this_usbduxsub->ai_insn_running=0;
	return i;
}




//////////////////
// analog out






static int usbdux_ao_insn_read(comedi_device *dev, comedi_subdevice *s,
				 comedi_insn *insn, lsampl_t *data)
{
  int i;
  int chan = CR_CHAN(insn->chanspec);
  usbduxsub_t* this_usbduxsub=dev->private;

  for(i=0;i<insn->n;i++) {
	  data[i]=this_usbduxsub->outBuffer[chan];
  }
  return i;
}



static int usbdux_ao_insn_write(comedi_device *dev, comedi_subdevice *s,
				  comedi_insn *insn, lsampl_t *data)
{
  int i;
  int chan = CR_CHAN(insn->chanspec);
  usbduxsub_t* this_usbduxsub=dev->private;

#ifdef CONFIG_COMEDI_DEBUG
  printk("comedi%d: ao_insn_write\n",dev->minor);
#endif
  if (this_usbduxsub->ao_cmd_running) {
	  printk("comedi%d: ao_insn_write: ERROR: asynchronous ao_cmd is running\n",
		 dev->minor
		 );
	  return 0;
  }

  this_usbduxsub->ao_insn_running=1;
  for(i=0;i<insn->n;i++){
#ifdef CONFIG_COMEDI_DEBUG
	  printk("comedi%d: ao_insn_write: data[chan=%d,i=%d]=%d\n",dev->minor,chan,i,data[i]);
#endif
	  // send one channel to the board
	  this_usbduxsub->outBuffer[0] = chan;	  
	  this_usbduxsub->outBuffer[1] = data[i];
	  send_dux_commands(this_usbduxsub,SENDDACOMMANDS);
  }
  this_usbduxsub->ao_insn_running=0;

  return i;
}









static int usbdux_ao_inttrig(comedi_device *dev, comedi_subdevice *s,
			     unsigned int trignum)
{
	int ret;
	usbduxsub_t* this_usbduxsub=dev->private;

        if (trignum!=0) {
                printk("comedi%d: usbdux_ao_inttrig: invalid trignum\n",dev->minor);
		return -EINVAL;
	}
	if (!(this_usbduxsub->ao_cmd_running)) {
		this_usbduxsub->ao_cmd_running=1;
		ret=usbduxsub_submit_OutURBs(this_usbduxsub);
		if (ret<0) {
			printk("comedi%d: usbdux_ao_inttrig: submitURB: err=%d\n",
			       dev->minor,ret);
			this_usbduxsub->ao_cmd_running=0;
			return ret;
		}
		s->async->inttrig = NULL;
	}  else {
		printk("comedi%d: ao_inttrig but acqu is already running.\n",
		       dev->minor);
	}
        return 1;
}





static int usbdux_ao_cmdtest(comedi_device *dev, 
			     comedi_subdevice *s, 
			     comedi_cmd *cmd)
{
	int err=0, tmp;
	//usbduxsub_t* this_usbduxsub=dev->private;

#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux_ao_cmdtest\n",dev->minor);
#endif
	/* make sure triggers are valid */
	// Only immediate triggers are allowed
	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW|TRIG_INT;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;

	// trigger should happen timed
	tmp=cmd->scan_begin_src;
	// just now we scan also in the high speed mode every frame
	// this is due to ehci driver limitations
	if (0) /* (this_usbduxsub->high_speed) */ {
		// start immidiately a new scan
		// the sampling rate is set by the coversion rate
		cmd->scan_begin_src &= TRIG_FOLLOW;
	} else {
		// start a new scan (output at once) with a timer
		cmd->scan_begin_src &= TRIG_TIMER;
	}
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;

	// scanning is continous
	tmp=cmd->convert_src;
	// we always output at 1kHz just now all channels at once
	if (0) /* (this_usbduxsub->high_speed) */ {
		// in usb-2.0 only one conversion it tranmitted but with 8kHz/n
		cmd->convert_src &= TRIG_TIMER;
	} else {
		// all conversion events happen simultaneously with a rate of 1kHz/n
		cmd->convert_src &= TRIG_NOW;
	}		
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;

	// issue a trigger when scan is finished and start a new scan
	tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;

	// trigger at the end of count events or not, stop condition or not
	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT|TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++;

	if(err)return 1;

	/* step 2: make sure trigger sources are unique and mutually compatible */
	/* note that mutual compatiblity is not an issue here */
	if(cmd->scan_begin_src!=TRIG_FOLLOW &&
	   cmd->scan_begin_src!=TRIG_EXT &&
	   cmd->scan_begin_src!=TRIG_TIMER)err++;
	if(cmd->stop_src!=TRIG_COUNT &&
	   cmd->stop_src!=TRIG_NONE)err++;

	if(err)return 2;


	/* step 3: make sure arguments are trivially compatible */

	if(cmd->start_arg!=0){
		cmd->start_arg=0;
		err++;
	}

	if(cmd->scan_begin_src==TRIG_FOLLOW){
		/* internal trigger */
		if(cmd->scan_begin_arg!=0){
			cmd->scan_begin_arg=0;
			err++;
		}
	}

	if(cmd->scan_begin_src==TRIG_TIMER){
		/* timer */
		if(cmd->scan_begin_arg<1000000){
			cmd->scan_begin_arg=1000000;
			err++;
		}
	}

	// not used now, is for later use
	if (cmd->convert_src==TRIG_TIMER) {
		if(cmd->convert_arg<125000){
			cmd->convert_arg=125000;
			err++;
		}
	}



	// the same argument
	if(cmd->scan_end_arg!=cmd->chanlist_len){
		cmd->scan_end_arg=cmd->chanlist_len;
		err++;
	}

	
	if(cmd->stop_src==TRIG_COUNT){
		/* any count is allowed */
	}else{
		/* TRIG_NONE */
		if(cmd->stop_arg!=0){
			cmd->stop_arg=0;
			err++;
		}
	}

#ifdef CONFIG_COMEDI_DEBUG 
	printk("comedi%d: err=%d, scan_begin_src=%d, scan_begin_arg=%d, convert_src=%d, convert_arg=%d\n", 
	       dev->minor, 
	       err,
	       cmd->scan_begin_src, 
	       cmd->scan_begin_arg, 
	       cmd->convert_src, 
	       cmd->convert_arg);  
#endif

	if(err)return 3;

	return 0;
}




static int usbdux_ao_cmd(comedi_device *dev, comedi_subdevice *s)
{
	comedi_cmd *cmd = &s->async->cmd;
	unsigned int chan, gain;
	int i,ret;
	usbduxsub_t* this_usbduxsub=dev->private;

	if (!this_usbduxsub) {
		printk("comedi%d: usbdux?: pointer to usb device is NULL!\n",
		       dev->minor);
		return -EFAULT;
	}

#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux_ao_cmd\n",dev->minor);
#endif

	if (this_usbduxsub->ao_insn_running) {
		printk("comedi%d: ao_cmd: ERROR: synchronous ao_insn is running\n",
		       dev->minor
		       );
		return 0;
	}



	// set current channel of the running aquisition to zero
	s->async->cur_chan	= 0;	
	for(i=0; i < cmd->chanlist_len; ++i ) {
		chan = CR_CHAN(cmd->chanlist[i]);
		gain = CR_RANGE(cmd->chanlist[i]);
		if (i>=NUMOUTCHANNELS) {
			printk("comedi%d: usbdux_ao_cmd: channel list too long\n",dev->minor);
			break;
		}
		this_usbduxsub->dac_commands[i]=(chan<<6);
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: dac command for ch %d is %x\n",
		       dev->minor,
		       i,
		       this_usbduxsub->dac_commands[i]);
#endif
	}


	// we count in steps of 1ms (125us)
	// 125us mode not used yet
	if (0) /* (this_usbduxsub->high_speed) */ {
		// 125us
		// timing of the conversion itself: every 125 us
		this_usbduxsub->ao_timer = cmd->convert_arg/125000;
      	} else {
		// 1ms
		// timing of the scan: we get all channels at once
		this_usbduxsub->ao_timer = cmd->scan_begin_arg/1000000;
#ifdef CONFIG_COMEDI_DEBUG
                printk("comedi%d: usbdux: scan_begin_src=%d, scan_begin_arg=%d, convert_src=%d, convert_arg=%d\n",
                       dev->minor,
		       cmd->scan_begin_src,
                       cmd->scan_begin_arg,
		       cmd->convert_src,
		       cmd->convert_arg); 
		printk("comedi%d: usbdux: ao_timer=%d (ms)\n",
		       dev->minor,
		       this_usbduxsub->ao_timer);
#endif
		if (this_usbduxsub->ao_timer<1) {
		         printk("comedi%d: usbdux: ao_timer=%d,  scan_begin_arg=%d. Not properly tested by cmdtest?\n",
				dev->minor,
				this_usbduxsub->ao_timer,
				cmd->scan_begin_arg);
			 return -EINVAL;
		}
	}
	this_usbduxsub->ao_counter=this_usbduxsub->ao_timer;

	if(cmd->stop_src==TRIG_COUNT){
		// not continous
		// counter
	        // high speed also scans everything at once
	        if (0) /* (this_usbduxsub->high_speed) */ {
			this_usbduxsub->ao_sample_count = 
				(cmd->stop_arg)*(cmd->scan_end_arg);
		} else {
			// there's no scan as the scan has been 
			// perf inside the FX2
			// data arrives as one packet
			this_usbduxsub->ao_sample_count = 
				cmd->stop_arg;
		}		
		this_usbduxsub->ao_continous=0;		
	} else {
		// continous aquisition
		this_usbduxsub->ao_continous=1;
		this_usbduxsub->ao_sample_count=0;
	}


        if(cmd->start_src == TRIG_NOW){
		// enable this acquisition operation
		this_usbduxsub->ao_cmd_running=1;
		ret=usbduxsub_submit_OutURBs(this_usbduxsub);
		if (ret<0) {
			this_usbduxsub->ao_cmd_running=0;
			// fixme: unlink here??
			return ret;
		}
                s->async->inttrig = NULL;
        } else {
                /* TRIG_INT */
		// submit the urbs later
		// wait for an internal signal
                s->async->inttrig = usbdux_ao_inttrig;
        }



	return 0;
}




static unsigned hex2unsigned(char *h) {
	unsigned hi,lo;
	if (h[0]>'9') {
		hi=h[0]-'A'+0x0a;
	} else {
		hi=h[0]-'0';
	}
	if (h[1]>'9') {
		lo=h[1]-'A'+0x0a;
	} else {
		lo=h[1]-'0';
	}
	return hi*0x10+lo;
}


// for FX2
#define FIRMWARE_MAX_LEN 0x2000

// taken from David Brownell's fxload and adjusted for this driver
static int read_firmware(usbduxsub_t* usbduxsub,int firmwarePtr,long size) {
	int i=0;
	unsigned char* fp=(char*)firmwarePtr;
	unsigned char* firmwareBinary=NULL;
	int res=0;
	int maxAddr=0;

	firmwareBinary = kmalloc(FIRMWARE_MAX_LEN,GFP_KERNEL);
	if(!firmwareBinary){
		printk("comedi_: usbdux: mem alloc for firmware failed\n"
		       );
		return -ENOMEM;
	}
	
	for (;;) {
		char 	buf[256],*cp;
		char	type;
		int	len;
		int	idx, off;
		int j=0;

		// get one line
		while ((i<size)&&(fp[i]!=13)&&(fp[i]!=10)) {
			buf[j]=fp[i];
			i++;
			j++;
			if (j>=sizeof(buf)) {
				printk("comedi_: usbdux: bogus firmware file!\n");
				return -1;
			}
		}
		// get rid of LF/CR/...
		while ((i<size)&&((fp[i]==13)||(fp[i]==10)||(fp[i]==0))) {
			i++;
		}

		buf[j]=0;
		//printk("comedi_: buf=%s\n",buf);

		/* EXTENSION: "# comment-till-end-of-line", for copyrights etc */
		if (buf[0] == '#')
			continue;

		if (buf[0] != ':') {
			printk("comedi_: usbdux: upload: not an ihex record: %s", buf);
			return -EFAULT;
		}

		/* Read the length field (up to 16 bytes) */
		len = hex2unsigned(buf+1);

		/* Read the target offset */
		off = (hex2unsigned(buf+3)*0x0100)+hex2unsigned(buf+5);

		if ((off+len)>maxAddr) {
			maxAddr=off+len;
		}

		if (maxAddr>=FIRMWARE_MAX_LEN) {
			printk("comedi_: usbdux: firmware upload goes beyond FX2 RAM boundaries.");
			return -EFAULT;
		}
		
		//printk("comedi_: usbdux: off=%x, len=%x:",off,len);

		/* Read the record type */
		type = hex2unsigned(buf+7);

		/* If this is an EOF record, then make it so. */
		if (type == 1) {
			break;
		}

		if (type != 0) {
			printk("comedi_: usbdux: unsupported record type: %u\n",type);
			return -EFAULT;
		}

		for (idx = 0, cp = buf+9 ;  idx < len ;  idx += 1, cp += 2) {
			firmwareBinary[idx+off] = hex2unsigned(cp);
			//printk("%02x ",firmwareBinary[idx+off]);
		}
		//printk("\n");

		if (i>=size) {
			printk("comedi_: usbdux: unexpected end of hex file\n");
			break;
		}

	}
	res=firmwareUpload(usbduxsub,firmwareBinary,maxAddr+1);
	kfree(firmwareBinary);
	return res;
}



static int usbdux_dio_insn_config (
    comedi_device *dev,
    comedi_subdevice *s,
    comedi_insn *insn,
    lsampl_t *data) {
    int chan=CR_CHAN(insn->chanspec);

    if (insn->n!=1) return -EINVAL;

    /* The input or output configuration of each digital line is
     * configured by a special insn_config instruction.  chanspec
     * contains the channel to be changed, and data[0] contains the 
     * value COMEDI_INPUT or COMEDI_OUTPUT. */
        
    if (data[0]==COMEDI_OUTPUT) {
        s->io_bits |= 1<<chan;          /* 1 means Out */
    } else {
        s->io_bits &= ~(1<<chan);
    }
    // we don't tell the firmware here as it would take 8 frames
    // to submit the information. We do it in the insn_bits.
    return 1;
}



static int single_dio_read(usbduxsub_t* this_usbduxsub,lsampl_t* value) {
	int result=-EFAULT;
	int nrec;
	int i;
	
	for(i=0;i<3;i++) {
		result = usb_bulk_msg(this_usbduxsub->usbdev,
				      usb_rcvbulkpipe(this_usbduxsub->usbdev,
						      ADSINGLEEP),
				      this_usbduxsub->insnBuffer,
				      SIZEINSNBUF,
				      &nrec,
				      1*HZ);
		if (result<0) {
			printk("comedi%d: insn: USB error %d while DIO data.\n",
			       this_usbduxsub->comedidev->minor,result);
			return result;
		}
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: usbdux: dio_read: buffer[0]=%d\n",
		       this_usbduxsub->comedidev->minor,
		       this_usbduxsub->insnBuffer[0]);
#endif
		*value=(lsampl_t)(this_usbduxsub->insnBuffer[0]);
	}
	return result;
}




static int usbdux_dio_insn_bits (
    comedi_device *dev,
    comedi_subdevice *s,
    comedi_insn *insn,
    lsampl_t *data) {

	usbduxsub_t* this_usbduxsub=dev->private;
	if (insn->n!=2) return -EINVAL;

	if (!this_usbduxsub) {
		printk("comedi%d: usbdux: dio_insn_bits: s->private=NULL\n",dev->minor);
		return -EFAULT;
	}

	if (this_usbduxsub->ai_insn_running) {
		printk("comedi%d: dio not possible. Sync command is running.\n",		       dev->minor);
		return -EBUSY;
	}	

	/* The insn data is a mask in data[0] and the new data
	 * in data[1], each channel cooresponding to a bit. */
	s->state &= ~data[0];
	s->state |= data[0]&data[1];
        
	this_usbduxsub->dio_insn_running=1;

	/* Write out the new digital output lines */
	// This command also tells the firmware to return
	// the digital input lines
	send_dux_commands(this_usbduxsub,
			  SENDDIOBITSCOMMAND);
	/* on return, data[1] contains the value of the digital
	 * input/output lines. */
	single_dio_read(this_usbduxsub,data+1);

	this_usbduxsub->dio_insn_running=0;
	
	return 2;
}






// is called when comedi-config is called
static int usbdux_attach(comedi_device * dev, comedi_devconfig * it)
{
	int ret;
	int index;
	int i;
	comedi_subdevice *s=NULL;
       	dev->private=NULL;

	// find a valid device which has been detected by the probe function of the usb
	index=-1;
	for(i=0;i<NUMUSBDUX;i++) {
		if ((usbduxsub[i].probed)&&
		    (!usbduxsub[i].attached)) {
			index=i;
			break;
		}
	}

	if (index<0) {
		printk("comedi%d: usbdux: error: attach failed, no usbdux devs connected to the usb bus.\n",
		       dev->minor);
		return -ENODEV;
	}

	// pointer back to the corresponding comedi device
	usbduxsub[index].comedidev=dev;

	// trying to upload the firmware into the chip
	if(it->options[COMEDI_DEVCONF_AUX_DATA] && 
	   it->options[COMEDI_DEVCONF_AUX_DATA_LENGTH]){
#ifdef CONFIG_COMEDI_DEBUG
		printk("comedi%d: Uploading the firmware.\n",
		       dev->minor);
#endif
		read_firmware(usbduxsub,
			      it->options[COMEDI_DEVCONF_AUX_DATA],
			      it->options[COMEDI_DEVCONF_AUX_DATA_LENGTH]);
	} 
#ifdef CONFIG_COMEDI_DEBUG
	else {
		printk("comedi%d: Need firmware to operate.",
		       dev->minor);
		printk("Use the -i option to provide it or use fxload.\n");
	}
#endif
	
	dev->board_name = BOARDNAME;
	
	/* set number of subdevices */
	dev->n_subdevices=N_SUBDEVICES;

	// allocate space for the subdevices
	if((ret=alloc_subdevices(dev,N_SUBDEVICES))<0) {
		printk("comedi%d: usbdux: error alloc space for subdev\n",
		       dev->minor);
		return ret;
	}

	usbduxsub[index].adc_commands=kmalloc(NUMCHANNELS,
					      GFP_KERNEL);
	if (!usbduxsub[index].adc_commands) {
		printk("comedi%d: usbdux: error alloc space for adc commands\n",
		       dev->minor);
		return -ENOMEM;
	}

	usbduxsub[index].dac_commands=kmalloc(NUMOUTCHANNELS,
					      GFP_KERNEL);
	if (!usbduxsub[index].dac_commands) {
		printk("comedi%d: usbdux: error alloc space for dac commands\n",
		       dev->minor);
		return -ENOMEM;
	}

	usbduxsub[index].dux_commands=kmalloc(SIZEOFDUXBUFFER,
					      GFP_KERNEL);
	if (!usbduxsub[index].dux_commands) {
		printk("comedi%d: usbdux: error alloc space for dac commands\n",
		       dev->minor);
		return -ENOMEM;
	}

	printk("comedi%d: usbdux: usb-device %d is attached to comedi.\n",
	       dev->minor,index);
	usbduxsub[index].attached=1;
	// private structure is also simply the usb-structure
	dev->private=usbduxsub+index;
	// the first subdevice is the A/D converter
	s = dev->subdevices + SUBDEV_AD;
	// the URBs get the comedi subdevice
	// which is responsible for reading
	// this is the subdevice which reads data
	dev->read_subdev = s;
	// the subdevice receives as private structure the
	// usb-structure
	s->private=NULL;
	// analog input
	s->type=COMEDI_SUBD_AI;
	// readable and ref is to ground
	s->subdev_flags=SDF_READABLE|SDF_GROUND;
	// 8 channels
	s->n_chan=8;
	// length of the channellist
	s->len_chanlist=8;
	// callback functions
	s->insn_read = usbdux_ai_insn_read;
	s->do_cmdtest=usbdux_ai_cmdtest;
	s->do_cmd=usbdux_ai_cmd;
	s->cancel=usbdux_ai_cancel;
	// max value from the A/D converter (12bit)
	s->maxdata=0xfff;
	// range table to convert to physical units
	s->range_table = &range_usbdux_ai_range;
	//

	// analog out
	s=dev->subdevices + SUBDEV_DA;
	// analog out
	s->type=COMEDI_SUBD_AO;
	// backward pointer
	dev->write_subdev=s;
	// the subdevice receives as private structure the
	// usb-structure
	s->private=NULL;
	// are writable
	s->subdev_flags=SDF_WRITABLE|SDF_GROUND;
	// 4 channels
	s->n_chan=4;
	// length of the channellist
	s->len_chanlist=4;
	// 12 bit resolution
	s->maxdata=0x0fff;
	// bipolar range
	s->range_table=&range_usbdux_ao_range;
	// callback
	s->do_cmdtest = usbdux_ao_cmdtest;
	s->do_cmd = usbdux_ao_cmd;
	s->cancel = usbdux_ao_cancel;
	s->insn_read = usbdux_ao_insn_read;
	s->insn_write = usbdux_ao_insn_write;

	// digital I/O
	s=dev->subdevices + SUBDEV_DIO;
	s->type=COMEDI_SUBD_DIO;
	s->subdev_flags=SDF_READABLE|SDF_WRITABLE;
	s->n_chan=8;
	s->maxdata=1;
	s->range_table=&range_digital;
	s->insn_bits = usbdux_dio_insn_bits;
	s->insn_config = usbdux_dio_insn_config;
	// the subdevice receives as private structure the
	// usb-structure
	s->private=NULL;

	printk("comedi%d: successfully attached to usbdux.\n",
	       dev->minor);

	return 0;
}



static int usbdux_detach(comedi_device * dev) {
	usbduxsub_t* usbduxsub_tmp;

	if (!dev) {
		printk("comedi?: usbdux: detach without dev variable...\n");
		return -EFAULT;
	}

	usbduxsub_tmp=dev->private;
	if (!usbduxsub_tmp) {
		return 0;
	}

#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux: detach usb device\n",
	       dev->minor);
#endif
	if (!(usbduxsub_tmp->probed)) {
		printk("comedi%d: usbdux: err: detach on invalid dev\n",
		       dev->minor);
		return 0;
	}
       	if (!(usbduxsub_tmp->attached)) {
		printk("comedi%d: usbdux: err: detach on detached dev\n",
		       dev->minor);
		return 0;
	}
	if (usbduxsub_tmp->ao_cmd_running) {
		usbduxsub_tmp->ao_cmd_running=0;
		usbduxsub_unlink_OutURBs(usbduxsub_tmp);
	}
	if (usbduxsub_tmp->ai_cmd_running) {
		usbduxsub_tmp->ai_cmd_running=0;
		usbduxsub_unlink_InURBs(usbduxsub_tmp);
	}
	if (usbduxsub_tmp->adc_commands) {
		kfree(usbduxsub_tmp->adc_commands);
		usbduxsub_tmp->adc_commands=NULL;
	}
	if (usbduxsub_tmp->dac_commands) {
		kfree(usbduxsub_tmp->dac_commands);
		usbduxsub_tmp->dac_commands=NULL;
	}
	if (usbduxsub_tmp->dux_commands) {
		kfree(usbduxsub_tmp->dux_commands);
		usbduxsub_tmp->dux_commands=NULL;
	}
	usbduxsub_tmp->attached=0;
	// Don't allow detach to free the private structure
	// Leave it to the usb subsystem when the module is removed
	dev->private=NULL;
#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi%d: usbdux: detach: successfully removed\n",
	       dev->minor);
#endif
	return 0;
}


static void init_usb_devices(void) {
	int index;
#ifdef CONFIG_COMEDI_DEBUG
	printk("comedi_: usbdux: setting all possible devs to invalid\n");
#endif
	// all devices entries are invalid to begin with
	// they will become valid by the probe function
	// and then finally by the attach-function
	for(index=0;index<NUMUSBDUX;index++) {
		usbduxsub[index].adc_commands=NULL;
		usbduxsub[index].dac_commands=NULL;
		usbduxsub[index].dux_commands=NULL;
		usbduxsub[index].probed=0;
		usbduxsub[index].attached=0;
		usbduxsub[index].comedidev=NULL;
		usbduxsub[index].usbdev=NULL;
		usbduxsub[index].inBuffer=NULL;		
		usbduxsub[index].insnBuffer=NULL;
		usbduxsub[index].outBuffer=NULL;
		usbduxsub[index].high_speed=0;
		usbduxsub[index].ai_cmd_running=0;
		usbduxsub[index].ao_cmd_running=0;
		usbduxsub[index].ai_insn_running=0;
		usbduxsub[index].ao_insn_running=0;
		usbduxsub[index].dio_insn_running=0;
		usbduxsub[index].urbIn=NULL;
		usbduxsub[index].urbOut=NULL;
		usbduxsub[index].numOfOutBuffers=0;
		usbduxsub[index].numOfInBuffers=0;
	}
}

// Table with the USB-devices: just now only testing IDs
static struct usb_device_id usbduxsub_table [] = {
        { USB_DEVICE(0x4b4, 0x8613),
        },
        { USB_DEVICE(0x4b4, 0x86ff) 
        },
	{ }	/* Terminating entry */
};


MODULE_DEVICE_TABLE (usb, usbduxsub_table);


// The usbduxsub-driver
static struct usb_driver usbduxsub_driver = {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,4,18)
        owner:          THIS_MODULE,
#endif
	name:		BOARDNAME,
	probe:		usbduxsub_probe,
	disconnect:	usbduxsub_disconnect,
	id_table:	usbduxsub_table,
};







// Can't use the nice macro as I have also to initialise the USB
// subsystem:
// registering the usb-system _and_ the comedi-driver
static int init_usbdux(void) {
	info(DRIVER_VERSION ":" DRIVER_DESC);
	init_usb_devices();
      	usb_register(&usbduxsub_driver);
	comedi_driver_register(&driver_usbdux);
	return 0;
}



// deregistering the comedi driver and the usb-subsystem
static void exit_usbdux(void) {
	comedi_driver_unregister(&driver_usbdux);
        usb_deregister(&usbduxsub_driver);
}


module_init(init_usbdux);
module_exit(exit_usbdux);                                      

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");




#endif
