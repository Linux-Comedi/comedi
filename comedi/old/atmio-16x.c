/*
    module/atmio-16x.c
    hardware driver for National Instruments AT-MIO-16X

    COMEDI - Linux Control and Measurement Device Interface
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
	specifications are found in NI document 320640b.pdf
*/

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

#define _B(b)		((struct bitchan){_B_CHAN,(b)})

#define _B_CHAN		Command_Register_1
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
#undef _B_CHAN

#define _B_CHAN		Command_Register_2
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
#undef _B_CHAN

#define _B_CHAN		Command_Register_3
#define ADCDSP		_bit15
#define DIOPBEN		_bit14
#define DIOPAEN		_bit13
#define DMATCINT	_bit12
#define DACCMPLINT	_bit11
#define DAQCMPLINT	_bit10
#define IO_INT		_bit9
#define DMACHA		_bit8
#define DMACHB		_bit7
#define ARCREQ		_bit6
#define DAC1REQ		_bit5
#define DAC0REQ		_bit4
#define DRVAIS		_bit3
#define INTCHB2		_bit2
#define INTCHB1		_bit1
#define INTCHB0		_bit0
#undef _B_CHAN

#define _B_CHAN		Command_Register_3
#define CLKMODEB1	_bit15
#define CLKMODEB0	_bit14
#define DAC1DSP		_bit13
#define DAC0DSP		_bit12
#define DACMB3		_bit11
#define DACMB2		_bit10
#define DACMB1		_bit9
#define DACMB0		_bit8
#define DACGATE		_bit7
#define DB_DIS		_bit6
#define CYCLICSTOP	_bit5
#define ADCFIFOREQ	_bit4
#define SRC3SEL		_bit3
#define GATE2SEL	_bit2
#define FIFODAC		_bit1
#define EXTTRIG_DIS	_bit0
#undef _B_CHAN

#define _B_CHAN		Status_Register_1
#define DAQCOMP		_bit15
#define DAQPROG		_bit14
#define ADCFIFOHF	_bit13
#define ADCFIFOEF	_bit12
#define DMATCA		_bit11
#define DMATCB		_bit10
#define OVERFLOW	_bit9
#define OVERRUN		_bit8
#define TMRREQ		_bit7
#define	DACCOMP		_bit6
#define DACFIFOFF	_bit5
#define DACFIFOHF	_bit4
#define DACFIFOEF	_bit3
#define EEPROMDATA	_bit2
#define EEPROMCD	_bit1
#define CFGMEMEF	_bit0
#undef _B_CHAN

#define _B_CHAN
#define ADC_BUSY	_bit0
#undef _B_CHAN

#define _B_CHAN		CONFIGMEM_Register
#define CHAN_SE		_bit15
#define CHAN_AIS	_bit14
#define CHAN_CAL	_bit13
#define CHAN_BIP	_bit12
#define CHANSEL3	_bit9
#define CHANSEL2	_bit8
#define CHANSEL1	_bit7
#define CHANSEL0	_bit6
#define CH_GAIN2	_bit5
#define CH_GAIN1	_bit4
#define CH_GAIN0	_bit3
#define CHAN_LAST	_bit2
#define CHAN_GHOST	_bit1
#define CHAN_DSP	_bit0
#endif _B_CHAN










int irqbits[]={
	-1, -1, -1, 0, 1, 2, -1, 3,
	-1, -1, 4, 5, 6, -1, -1, 7
}




