/*
    module/atmio-16d.c
    hardware driver for National Instruments AT-MIO-16D

    COMEDI - Linux Control and Measurement Device Interface
    Copyright (C) 1995 Claus Schroeter <clausi@chemie.fu-berlin.de>
    Copyright (C) 1998 David A. Schleef <ds@stm.lbl.gov>

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
	
	specifications can be found in NI document 320489.pdf
*/

#define Command_Register_1		0x00	/* wo */
#define Status_Register			0x00	/* ro */
#define Command_Register_2		0x02	/* wo */

#define Start_Convert_Register		0x08	/* wo */
#define Start_DAQ_Register		0x0a	/* wo */
#define AD_Clear_Register		0x0c	/* wo */
#define External_Strobe_Register	0x0e	/* wo */

#define DAC0_Register			0x10	/* wo */
#define DAC1_Register			0x12	/* wo */
#define INT2CLR_Register		0x14	/* wo */

#define Mux_Counter_Register		0x04	/* wo */
#define Mux_Gain_Register		0x06	/* wo */
#define AD_FIFO_Register		0x16	/* ro */
#define DMA_TC_INT_Clear_Register	0x16	/* wo */

#define Am9513A_Data_Register		0x18	/* rw */
#define Am9513A_Command_Register	0x1a	/* wo */
#define Am9513A_Status_Register		0x1a	/* ro */

#define MIO_16_Digital_Input_Register	0x1c	/* ro */
#define MIO_16_Digital_Output_Register	0x1c	/* wo */

#define RTSI_Switch_Shift_Register	0x1e	/* wo 8 */
#define RTSI_Switch_Strobe_Register	0x1f	/* wo 8 */

#define DIO_24_PORTA_Register		0x00	/* rw 8 */
#define DIO_24_PORTB_Register		0x01	/* rw 8 */
#define DIO_24_PORTC_Register		0x02	/* rw 8 */
#define DIO_24_CNFG_Register		0x03	/* wo 8 */


#define _B(b)		((struct bitchan){_B_CHAN,(b)})

#define _B_CHAN		Command_Register_1
#define DAQSTOPINTEN	_bit9
#define	TCINTEN		_bit8
#define CONVINTEN	_bit7
#define DBDMA		_bit6
#define DMAEN		_bit5
#define DAQEN		_bit4
#define SCANEN		_bit3
#define SCANDIV		_bit2
#define CNT32		_bit1
#define TWOSCADC	_bit0
#undef _B_CHAN

#define _B_CHAN		Status_Register
#define GINT		_bit15
#define DAQSTOPINT	_bit14
#define CONVAVAIL	_bit13
#define OUT2INT		_bit12
#define DAQPROG		_bit11
#define DMATCINT	_bit10
#define OVERFLOW	_bit9
#define OVERRUN		_bit8
#define GAIN1		_bit7
#define GAIN0		_bit6
#define DMACH		_bit5
#define MUX1EN		_bit4
#define MUX0EN		_bit3
#define MA2		_bit2
#define MA1		_bit1
#define MA0		_bit0
#undef _B_CHAN

#define _B_CHAN		Command_Register_2
#define DOUTEN1		_bit9
#define DOUTEN0		_bit8
#define INTEN		_bit7
#define INT2EN		_bit6
#define LDAC		_bit5
#define SCN2		_bit4
#define A4RCV		_bit3
#define A4DRV		_bit2
#define A2RCV		_bit1
#define A2DRV		_bit0
#undef _B_CHAN






