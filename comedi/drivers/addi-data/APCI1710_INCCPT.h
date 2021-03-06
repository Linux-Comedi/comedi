/**
@verbatim

Copyright (C) 2004,2005  ADDI-DATA GmbH for the source code of this module. 
        
        ADDI-DATA GmbH 
        Dieselstrasse 3 
        D-77833 Ottersweier 
        Tel: +19(0)7223/9493-0 
        Fax: +49(0)7223/9493-92 
        http://www.addi-data-com 
        info@addi-data.com 

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

You shoud also find the complete GPL in the COPYING file accompanying this source code.

@endverbatim
*/

#define APCI1710_16BIT_COUNTER   			0x10
#define APCI1710_32BIT_COUNTER   			0x0
#define APCI1710_QUADRUPLE_MODE  			0x0
#define APCI1710_DOUBLE_MODE     			0x3
#define APCI1710_SIMPLE_MODE     			0xF
#define APCI1710_DIRECT_MODE     			0x80
#define APCI1710_HYSTERESIS_ON   			0x60
#define APCI1710_HYSTERESIS_OFF  			0x0
#define APCI1710_INCREMENT       			0x60
#define APCI1710_DECREMENT       			0x0
#define APCI1710_LATCH_COUNTER   			0x1
#define APCI1710_CLEAR_COUNTER   			0x0
#define APCI1710_LOW             			0x0
#define APCI1710_HIGH            			0x1

/*********************/
/* Version 0600-0229 */
/*********************/

#define APCI1710_HIGH_EDGE_CLEAR_COUNTER 		0x0
#define APCI1710_HIGH_EDGE_LATCH_COUNTER 		0x1
#define APCI1710_LOW_EDGE_CLEAR_COUNTER  		0x2
#define APCI1710_LOW_EDGE_LATCH_COUNTER  		0x3
#define APCI1710_HIGH_EDGE_LATCH_AND_CLEAR_COUNTER 	0x4
#define APCI1710_LOW_EDGE_LATCH_AND_CLEAR_COUNTER 	0x5
#define APCI1710_SOURCE_0				0x0
#define APCI1710_SOURCE_1				0x1

#define APCI1710_30MHZ           30
#define APCI1710_33MHZ           33
#define APCI1710_40MHZ           40

#define APCI1710_ENABLE_LATCH_INT    		0x80
#define APCI1710_DISABLE_LATCH_INT   		(~APCI1710_ENABLE_LATCH_INT)

#define APCI1710_INDEX_LATCH_COUNTER 		0x10
#define APCI1710_INDEX_AUTO_MODE     		0x8
#define APCI1710_ENABLE_INDEX        		0x4
#define APCI1710_DISABLE_INDEX       		(~APCI1710_ENABLE_INDEX)
#define APCI1710_ENABLE_LATCH_AND_CLEAR 	0x8
#define APCI1710_DISABLE_LATCH_AND_CLEAR 	(~APCI1710_ENABLE_LATCH_AND_CLEAR)
#define APCI1710_SET_LOW_INDEX_LEVEL		0x4
#define APCI1710_SET_HIGH_INDEX_LEVEL           (~APCI1710_SET_LOW_INDEX_LEVEL)
#define APCI1710_INVERT_INDEX_RFERENCE		0x2
#define APCI1710_DEFAULT_INDEX_RFERENCE         (~APCI1710_INVERT_INDEX_RFERENCE)

#define APCI1710_ENABLE_INDEX_INT    		0x1
#define APCI1710_DISABLE_INDEX_INT   		(~APCI1710_ENABLE_INDEX_INT)

#define APCI1710_ENABLE_FREQUENCY    		0x4
#define APCI1710_DISABLE_FREQUENCY   		(~APCI1710_ENABLE_FREQUENCY)

#define APCI1710_ENABLE_FREQUENCY_INT   	0x8
#define APCI1710_DISABLE_FREQUENCY_INT  	(~APCI1710_ENABLE_FREQUENCY_INT)

#define APCI1710_ENABLE_40MHZ_FREQUENCY		0x40
#define APCI1710_DISABLE_40MHZ_FREQUENCY	(~APCI1710_ENABLE_40MHZ_FREQUENCY)

#define APCI1710_ENABLE_40MHZ_FILTER		0x80
#define APCI1710_DISABLE_40MHZ_FILTER		(~APCI1710_ENABLE_40MHZ_FILTER)

#define APCI1710_ENABLE_COMPARE_INT  		0x2
#define APCI1710_DISABLE_COMPARE_INT 		(~APCI1710_ENABLE_COMPARE_INT)

#define APCI1710_ENABLE_INDEX_ACTION  		0x20
#define APCI1710_DISABLE_INDEX_ACTION 		(~APCI1710_ENABLE_INDEX_ACTION)
#define APCI1710_REFERENCE_HIGH       		0x40
#define APCI1710_REFERENCE_LOW        		(~APCI1710_REFERENCE_HIGH)

#define APCI1710_TOR_GATE_LOW		0x40
#define APCI1710_TOR_GATE_HIGH		(~APCI1710_TOR_GATE_LOW)

//      INSN CONFIG 
#define	APCI1710_INCCPT_INITCOUNTER							100
#define APCI1710_INCCPT_COUNTERAUTOTEST						101
#define APCI1710_INCCPT_INITINDEX							102
#define APCI1710_INCCPT_INITREFERENCE						103
#define APCI1710_INCCPT_INITEXTERNALSTROBE					104
#define APCI1710_INCCPT_INITCOMPARELOGIC					105
#define APCI1710_INCCPT_INITFREQUENCYMEASUREMENT			106

// INSN READ
#define APCI1710_INCCPT_READLATCHREGISTERSTATUS				200
#define APCI1710_INCCPT_READLATCHREGISTERVALUE				201
#define APCI1710_INCCPT_READ16BITCOUNTERVALUE				202
#define APCI1710_INCCPT_READ32BITCOUNTERVALUE				203
#define APCI1710_INCCPT_GETINDEXSTATUS						204
#define APCI1710_INCCPT_GETREFERENCESTATUS					205
#define APCI1710_INCCPT_GETUASSTATUS						206
#define APCI1710_INCCPT_GETCBSTATUS							207
#define APCI1710_INCCPT_GET16BITCBSTATUS					208
#define APCI1710_INCCPT_GETUDSTATUS							209
#define APCI1710_INCCPT_GETINTERRUPTUDLATCHEDSTATUS			210
#define APCI1710_INCCPT_READFREQUENCYMEASUREMENT			211
#define APCI1710_INCCPT_READINTERRUPT                       212

//INSN BITS
#define APCI1710_INCCPT_CLEARCOUNTERVALUE					300
#define APCI1710_INCCPT_CLEARALLCOUNTERVALUE				301
#define APCI1710_INCCPT_SETINPUTFILTER						302
#define APCI1710_INCCPT_LATCHCOUNTER						303
#define APCI1710_INCCPT_SETINDEXANDREFERENCESOURCE			304
#define APCI1710_INCCPT_SETDIGITALCHLON						305
#define APCI1710_INCCPT_SETDIGITALCHLOFF					306

// INSN WRITE
#define APCI1710_INCCPT_ENABLELATCHINTERRUPT				400
#define APCI1710_INCCPT_DISABLELATCHINTERRUPT				401
#define APCI1710_INCCPT_WRITE16BITCOUNTERVALUE				402
#define APCI1710_INCCPT_WRITE32BITCOUNTERVALUE				403
#define APCI1710_INCCPT_ENABLEINDEX							404
#define APCI1710_INCCPT_DISABLEINDEX						405
#define APCI1710_INCCPT_ENABLECOMPARELOGIC					406
#define APCI1710_INCCPT_DISABLECOMPARELOGIC					407
#define APCI1710_INCCPT_ENABLEFREQUENCYMEASUREMENT			408
#define APCI1710_INCCPT_DISABLEFREQUENCYMEASUREMENT			409

/************ Main Functions *************/
INT i_APCI1710_InsnConfigINCCPT(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);

INT i_APCI1710_InsnBitsINCCPT(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);

INT i_APCI1710_InsnWriteINCCPT(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);

INT i_APCI1710_InsnReadINCCPT(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);

/*********** Supplementary Functions********/

// INSN CONFIG

INT i_APCI1710_InitCounter(comedi_device * dev,
	BYTE b_ModulNbr,
	BYTE b_CounterRange,
	BYTE b_FirstCounterModus,
	BYTE b_FirstCounterOption,
	BYTE b_SecondCounterModus, BYTE b_SecondCounterOption);

INT i_APCI1710_CounterAutoTest(comedi_device * dev, PBYTE pb_TestStatus);

INT i_APCI1710_InitIndex(comedi_device * dev,
	BYTE b_ModulNbr,
	BYTE b_ReferenceAction,
	BYTE b_IndexOperation, BYTE b_AutoMode, BYTE b_InterruptEnable);

INT i_APCI1710_InitReference(comedi_device * dev,
	BYTE b_ModulNbr, BYTE b_ReferenceLevel);

INT i_APCI1710_InitExternalStrobe(comedi_device * dev,
	BYTE b_ModulNbr, BYTE b_ExternalStrobe, BYTE b_ExternalStrobeLevel);

INT i_APCI1710_InitCompareLogic(comedi_device * dev,
	BYTE b_ModulNbr, UINT ui_CompareValue);

INT i_APCI1710_InitFrequencyMeasurement(comedi_device * dev,
	BYTE b_ModulNbr,
	BYTE b_PCIInputClock,
	BYTE b_TimingUnity,
	ULONG ul_TimingInterval, PULONG pul_RealTimingInterval);

//INSN BITS

INT i_APCI1710_ClearCounterValue(comedi_device * dev, BYTE b_ModulNbr);

INT i_APCI1710_ClearAllCounterValue(comedi_device * dev);

INT i_APCI1710_SetInputFilter(comedi_device * dev,
	BYTE b_ModulNbr, BYTE b_PCIInputClock, BYTE b_Filter);

INT i_APCI1710_LatchCounter(comedi_device * dev,
	BYTE b_ModulNbr, BYTE b_LatchReg);

INT i_APCI1710_SetIndexAndReferenceSource(comedi_device * dev,
	BYTE b_ModulNbr, BYTE b_SourceSelection);

INT i_APCI1710_SetDigitalChlOn(comedi_device * dev, BYTE b_ModulNbr);

INT i_APCI1710_SetDigitalChlOff(comedi_device * dev, BYTE b_ModulNbr);

// INSN WRITE
INT i_APCI1710_EnableLatchInterrupt(comedi_device * dev, BYTE b_ModulNbr);

INT i_APCI1710_DisableLatchInterrupt(comedi_device * dev, BYTE b_ModulNbr);

INT i_APCI1710_Write16BitCounterValue(comedi_device * dev,
	BYTE b_ModulNbr, BYTE b_SelectedCounter, UINT ui_WriteValue);

INT i_APCI1710_Write32BitCounterValue(comedi_device * dev,
	BYTE b_ModulNbr, ULONG ul_WriteValue);

INT i_APCI1710_EnableIndex(comedi_device * dev, BYTE b_ModulNbr);

INT i_APCI1710_DisableIndex(comedi_device * dev, BYTE b_ModulNbr);

INT i_APCI1710_EnableCompareLogic(comedi_device * dev, BYTE b_ModulNbr);

INT i_APCI1710_DisableCompareLogic(comedi_device * dev, BYTE b_ModulNbr);

INT i_APCI1710_EnableFrequencyMeasurement(comedi_device * dev,
	BYTE b_ModulNbr, BYTE b_InterruptEnable);

INT i_APCI1710_DisableFrequencyMeasurement(comedi_device * dev,
	BYTE b_ModulNbr);

// INSN READ

INT i_APCI1710_ReadLatchRegisterStatus(comedi_device * dev,
	BYTE b_ModulNbr, BYTE b_LatchReg, PBYTE pb_LatchStatus);

INT i_APCI1710_ReadLatchRegisterValue(comedi_device * dev,
	BYTE b_ModulNbr, BYTE b_LatchReg, PULONG pul_LatchValue);

INT i_APCI1710_Read16BitCounterValue(comedi_device * dev,
	BYTE b_ModulNbr, BYTE b_SelectedCounter, PUINT pui_CounterValue);

INT i_APCI1710_Read32BitCounterValue(comedi_device * dev,
	BYTE b_ModulNbr, PULONG pul_CounterValue);

INT i_APCI1710_GetIndexStatus(comedi_device * dev,
	BYTE b_ModulNbr, PBYTE pb_IndexStatus);

INT i_APCI1710_GetReferenceStatus(comedi_device * dev,
	BYTE b_ModulNbr, PBYTE pb_ReferenceStatus);

INT i_APCI1710_GetUASStatus(comedi_device * dev,
	BYTE b_ModulNbr, PBYTE pb_UASStatus);

INT i_APCI1710_GetCBStatus(comedi_device * dev,
	BYTE b_ModulNbr, PBYTE pb_CBStatus);

INT i_APCI1710_Get16BitCBStatus(comedi_device * dev,
	BYTE b_ModulNbr, PBYTE pb_CBStatusCounter0, PBYTE pb_CBStatusCounter1);

INT i_APCI1710_GetUDStatus(comedi_device * dev,
	BYTE b_ModulNbr, PBYTE pb_UDStatus);

INT i_APCI1710_GetInterruptUDLatchedStatus(comedi_device * dev,
	BYTE b_ModulNbr, PBYTE pb_UDStatus);

INT i_APCI1710_ReadFrequencyMeasurement(comedi_device * dev,
	BYTE b_ModulNbr,
	PBYTE pb_Status, PBYTE pb_UDStatus, PULONG pul_ReadValue);
