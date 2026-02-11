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
/*

  +-----------------------------------------------------------------------+
  | (C) ADDI-DATA GmbH          Dieselstrasse 3      D-77833 Ottersweier  |
  +-----------------------------------------------------------------------+
  | Tel : +49 (0) 7223/9493-0     | email    : info@addi-data.com         |
  | Fax : +49 (0) 7223/9493-92    | Internet : http://www.addi-data.com   |
  +-----------------------------------------------------------------------+
  | Project   : ADDI DATA         | Compiler : GCC 		          |
  | Modulname : addi_common.c     | Version  : 2.96                       |
  +-------------------------------+---------------------------------------+
  | Author    :           | Date     :                    		  |
  +-----------------------------------------------------------------------+
  | Description : ADDI COMMON Main Module                                 |
  +-----------------------------------------------------------------------+
  | CONFIG OPTIONS                                                        |
  |	option[0] - PCI bus number - if bus number and slot number are 0, |
  |			         then driver search for first unused card |
  |	option[1] - PCI slot number                                       |
  |							                  |
  |	option[2] = 0  - DMA ENABLE                                       |
  |               = 1  - DMA DISABLE                                      |
  +----------+-----------+------------------------------------------------+
*/

#include<linux/kernel.h>
#include<linux/module.h>
#include<linux/sched/signal.h>
#include<linux/mm.h>
#include<linux/slab.h>
#include<linux/errno.h>
#include<linux/ioport.h>
#include<linux/delay.h>
#include<linux/interrupt.h>
#include<linux/timex.h>
#include<linux/timer.h>
#include<linux/pci.h>
#include<linux/comedidev.h>
#include<asm/io.h>
#if defined(CONFIG_APCI_1710) || defined(CONFIG_APCI_3200) || defined(CONFIG_APCI_3300)
#include<asm/i387.h>
#endif

#include "addi_common.h"
#include "addi_amcc_s5933.h"

#ifndef ADDIDATA_DRIVER_NAME
#define ADDIDATA_DRIVER_NAME	"addi_common"
#endif

//Update-0.7.57->0.7.68MODULE_AUTHOR("ADDI-DATA GmbH <info@addi-data.com>");
//Update-0.7.57->0.7.68MODULE_DESCRIPTION("Comedi ADDI-DATA module");
//Update-0.7.57->0.7.68MODULE_LICENSE("GPL");

#define devpriv ((addi_private *)dev->private)
#define this_board ((const boardtype *)dev->board_ptr)

#if defined(CONFIG_APCI_1710) || defined(CONFIG_APCI_3200) || defined(CONFIG_APCI_3300)
//BYTE b_SaveFPUReg [94];

void fpu_begin(void)
{
	//asm ("fstenv b_SaveFPUReg");
	kernel_fpu_begin();
}

void fpu_end(void)
{
	// asm ("frstor b_SaveFPUReg");
	kernel_fpu_end();
}
#endif

#include "addi_eeprom.c"
#if (defined (CONFIG_APCI_3120) || defined (CONFIG_APCI_3001))
#include "hwdrv_apci3120.c"
#endif
#ifdef CONFIG_APCI_1032
#include "hwdrv_apci1032.c"
#endif
#ifdef CONFIG_APCI_1516
#include "hwdrv_apci1516.c"
#endif
#ifdef CONFIG_APCI_2016
#include "hwdrv_apci2016.c"
#endif
#ifdef CONFIG_APCI_2032
#include "hwdrv_apci2032.c"
#endif
#ifdef CONFIG_APCI_2200
#include "hwdrv_apci2200.c"
#endif
#ifdef CONFIG_APCI_1564
#include "hwdrv_apci1564.c"
#endif
#ifdef CONFIG_APCI_1500
#include "hwdrv_apci1500.c"
#endif
#ifdef CONFIG_APCI_3501
#include "hwdrv_apci3501.c"
#endif
#ifdef CONFIG_APCI_035
#include "hwdrv_apci035.c"
#endif
#if (defined (CONFIG_APCI_3200) || defined (CONFIG_APCI_3300))
#include "hwdrv_apci3200.c"
#endif
#ifdef CONFIG_APCI_1710
#include "hwdrv_APCI1710.c"
#endif
#ifdef CONFIG_APCI_16XX
#include "hwdrv_apci16xx.c"
#endif
#ifdef CONFIG_APCI_3XXX
#include "hwdrv_apci3xxx.c"
#endif

#ifndef COMEDI_SUBD_TTLIO
#define COMEDI_SUBD_TTLIO   11	/* Digital Input Output But TTL */
#endif

static DEFINE_PCI_DEVICE_TABLE(addi_apci_tbl) = {
#ifdef CONFIG_APCI_3120
	{APCI3120_BOARD_VENDOR_ID, 0x818D, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_1032
	{APCI1032_BOARD_VENDOR_ID, 0x1003, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_1516
	{APCI1516_BOARD_VENDOR_ID, 0x1001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_2016
	{APCI2016_BOARD_VENDOR_ID, 0x1002, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_2032
	{APCI2032_BOARD_VENDOR_ID, 0x1004, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_2200
	{APCI2200_BOARD_VENDOR_ID, 0x1005, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_1564
	{APCI1564_BOARD_VENDOR_ID, 0x1006, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_1500
	{APCI1500_BOARD_VENDOR_ID, 0x80fc, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_3001
	{APCI3120_BOARD_VENDOR_ID, 0x828D, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_3501
	{APCI3501_BOARD_VENDOR_ID, 0x3001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_035
	{APCI035_BOARD_VENDOR_ID, 0x0300, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_3200
	{APCI3200_BOARD_VENDOR_ID, 0x3000, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_3300
	{APCI3200_BOARD_VENDOR_ID, 0x3007, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_1710
	{APCI1710_BOARD_VENDOR_ID, APCI1710_BOARD_DEVICE_ID,
		PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_16XX
	{0x15B8, 0x1009, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x100A, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
#ifdef CONFIG_APCI_3XXX
	{0x15B8, 0x3010, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x300F, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x300E, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3013, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3014, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3015, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3016, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3017, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3018, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3019, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x301A, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x301B, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x301C, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x301D, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x301E, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x301F, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3020, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3021, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3022, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3023, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x300B, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3002, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3003, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3004, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0x15B8, 0x3024, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
#endif
	{0}
};

MODULE_DEVICE_TABLE(pci, addi_apci_tbl);

static const boardtype boardtypes[] = {
#ifdef CONFIG_APCI_3120
	{
		.pc_DriverName						= "apci3120",
		.i_VendorId							= APCI3120_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x818D,
		.i_IorangeBase0						= AMCC_OP_REG_SIZE,
		.i_IorangeBase1						= APCI3120_ADDRESS_RANGE,
		.i_IorangeBase2						= 8,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= NULL,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 8,
		.i_AiMaxdata						= 0xffff,
		.i_AoMaxdata						= 0x3fff,
		.pr_AiRangelist						= &range_apci3120_ai,
		.pr_AoRangelist						= &range_apci3120_ao,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 0x0f,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 1,
		.i_Timer							= 1,
		.b_AvailableConvertUnit				= 1,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 100000,
		.v_hwdrv_Interrupt					= v_APCI3120_Interrupt,
		.i_hwdrv_Reset						= i_APCI3120_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3120_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3120_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= i_APCI3120_CommandTestAnalogInput,
		.i_hwdrv_CommandAnalogInput			= i_APCI3120_CommandAnalogInput,
		.i_hwdrv_CancelAnalogInput			= i_APCI3120_StopCyclicAcquisition,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3120_InsnWriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3120_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3120_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI3120_InsnConfigDigitalOutput,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3120_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3120_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= i_APCI3120_InsnConfigTimer,
		.i_hwdrv_InsnWriteTimer				= i_APCI3120_InsnWriteTimer,
		.i_hwdrv_InsnReadTimer				= i_APCI3120_InsnReadTimer,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_1032
	{
		.pc_DriverName						= "apci1032",
		.i_VendorId							= APCI1032_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x1003,
		.i_IorangeBase0						= 4,
		.i_IorangeBase1						= APCI1032_ADDRESS_RANGE,
		.i_IorangeBase2						= 0,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_EEPROM,
		.pc_EepromChip						= ADDIDATA_93C76,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 32,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI1032_Interrupt,
		.i_hwdrv_Reset						= i_APCI1032_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= i_APCI1032_ConfigDigitalInput,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI1032_Read1DigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI1032_ReadMoreDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_1516
	{
		.pc_DriverName						= "apci1516",
		.i_VendorId							= APCI1516_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x1001,
		.i_IorangeBase0						= 128,
		.i_IorangeBase1						= APCI1516_ADDRESS_RANGE,
		.i_IorangeBase2						= 32,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_EEPROM,
		.pc_EepromChip						= ADDIDATA_S5920,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 8,
		.i_NbrDoChannel						= 8,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 1,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= NULL,
		.i_hwdrv_Reset						= i_APCI1516_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI1516_Read1DigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI1516_ReadMoreDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI1516_ConfigDigitalOutput,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI1516_WriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI1516_ReadDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= i_APCI1516_ConfigWatchdog,
		.i_hwdrv_InsnWriteTimer				= i_APCI1516_StartStopWriteWatchdog,
		.i_hwdrv_InsnReadTimer				= i_APCI1516_ReadWatchdog,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_2016
	{
		.pc_DriverName						= "apci2016",
		.i_VendorId							= APCI2016_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x1002,
		.i_IorangeBase0						= 128,
		.i_IorangeBase1						= APCI2016_ADDRESS_RANGE,
		.i_IorangeBase2						= 32,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_EEPROM,
		.pc_EepromChip						= ADDIDATA_S5920,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 16,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 1,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= NULL,
		.i_hwdrv_Reset						= i_APCI2016_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI2016_ConfigDigitalOutput,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI2016_WriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI2016_BitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= i_APCI2016_ConfigWatchdog,
		.i_hwdrv_InsnWriteTimer				= i_APCI2016_StartStopWriteWatchdog,
		.i_hwdrv_InsnReadTimer				= i_APCI2016_ReadWatchdog,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_2032
	{
		.pc_DriverName						= "apci2032",
		.i_VendorId							= APCI2032_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x1004,
		.i_IorangeBase0						= 4,
		.i_IorangeBase1						= APCI2032_ADDRESS_RANGE,
		.i_IorangeBase2						= 0,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_EEPROM,
		.pc_EepromChip						= ADDIDATA_93C76,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 32,
		.i_DoMaxdata						= 0xffffffff,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 1,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI2032_Interrupt,
		.i_hwdrv_Reset						= i_APCI2032_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI2032_ConfigDigitalOutput,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI2032_WriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI2032_ReadDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI2032_ReadInterruptStatus,
		.i_hwdrv_InsnConfigTimer			= i_APCI2032_ConfigWatchdog,
		.i_hwdrv_InsnWriteTimer				= i_APCI2032_StartStopWriteWatchdog,
		.i_hwdrv_InsnReadTimer				= i_APCI2032_ReadWatchdog,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_2200
	{
		.pc_DriverName						= "apci2200",
		.i_VendorId							= APCI2200_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x1005,
		.i_IorangeBase0						= 4,
		.i_IorangeBase1						= APCI2200_ADDRESS_RANGE,
		.i_IorangeBase2						= 0,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_EEPROM,
		.pc_EepromChip						= ADDIDATA_93C76,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 8,
		.i_NbrDoChannel						= 16,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 1,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= NULL,
		.i_hwdrv_Reset						= i_APCI2200_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI2200_Read1DigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI2200_ReadMoreDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI2200_ConfigDigitalOutput,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI2200_WriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI2200_ReadDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= i_APCI2200_ConfigWatchdog,
		.i_hwdrv_InsnWriteTimer				= i_APCI2200_StartStopWriteWatchdog,
		.i_hwdrv_InsnReadTimer				= i_APCI2200_ReadWatchdog,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_1564
	{
		.pc_DriverName						= "apci1564",
		.i_VendorId							= APCI1564_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x1006,
		.i_IorangeBase0						= 128,
		.i_IorangeBase1						= APCI1564_ADDRESS_RANGE,
		.i_IorangeBase2						= 0,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_EEPROM,
		.pc_EepromChip						= ADDIDATA_93C76,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 32,
		.i_NbrDoChannel						= 32,
		.i_DoMaxdata						= 0xffffffff,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 1,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI1564_Interrupt,
		.i_hwdrv_Reset						= i_APCI1564_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= i_APCI1564_ConfigDigitalInput,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI1564_Read1DigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI1564_ReadMoreDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI1564_ConfigDigitalOutput,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI1564_WriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI1564_ReadDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI1564_ReadInterruptStatus,
		.i_hwdrv_InsnConfigTimer			= i_APCI1564_ConfigTimerCounterWatchdog,
		.i_hwdrv_InsnWriteTimer				= i_APCI1564_StartStopWriteTimerCounterWatchdog,
		.i_hwdrv_InsnReadTimer				= i_APCI1564_ReadTimerCounterWatchdog,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_1500
	{
		.pc_DriverName						= "apci1500",
		.i_VendorId							= APCI1500_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x80fc,
		.i_IorangeBase0						= 128,
		.i_IorangeBase1						= APCI1500_ADDRESS_RANGE,
		.i_IorangeBase2						= 4,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= NULL,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 16,
		.i_NbrDoChannel						= 16,
		.i_DoMaxdata						= 0xffff,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 1,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI1500_Interrupt,
		.i_hwdrv_Reset						= i_APCI1500_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= i_APCI1500_ConfigDigitalInputEvent,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI1500_Initialisation,
		.i_hwdrv_InsnWriteDigitalInput		= i_APCI1500_StartStopInputEvent,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI1500_ReadMoreDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI1500_ConfigDigitalOutputErrorInterrupt,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI1500_WriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI1500_ConfigureInterrupt,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= i_APCI1500_ConfigCounterTimerWatchdog,
		.i_hwdrv_InsnWriteTimer				= i_APCI1500_StartStopTriggerTimerCounterWatchdog,
		.i_hwdrv_InsnReadTimer				= i_APCI1500_ReadInterruptMask,
		.i_hwdrv_InsnBitsTimer				= i_APCI1500_ReadCounterTimerWatchdog,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_3001
	{
		.pc_DriverName						= "apci3001",
		.i_VendorId							= APCI3120_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x828D,
		.i_IorangeBase0						= AMCC_OP_REG_SIZE,
		.i_IorangeBase1						= APCI3120_ADDRESS_RANGE,
		.i_IorangeBase2						= 8,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= NULL,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0xfff,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3120_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 0x0f,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 1,
		.i_Timer							= 1,
		.b_AvailableConvertUnit				= 1,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 100000,
		.v_hwdrv_Interrupt					= v_APCI3120_Interrupt,
		.i_hwdrv_Reset						= i_APCI3120_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3120_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3120_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= i_APCI3120_CommandTestAnalogInput,
		.i_hwdrv_CommandAnalogInput			= i_APCI3120_CommandAnalogInput,
		.i_hwdrv_CancelAnalogInput			= i_APCI3120_StopCyclicAcquisition,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3120_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3120_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI3120_InsnConfigDigitalOutput,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3120_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3120_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= i_APCI3120_InsnConfigTimer,
		.i_hwdrv_InsnWriteTimer				= i_APCI3120_InsnWriteTimer,
		.i_hwdrv_InsnReadTimer				= i_APCI3120_InsnReadTimer,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_3501
	{
		.pc_DriverName						= "apci3501",
		.i_VendorId							= APCI3501_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x3001,
		.i_IorangeBase0						= 64,
		.i_IorangeBase1						= APCI3501_ADDRESS_RANGE,
		.i_IorangeBase2						= 0,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_EEPROM,
		.pc_EepromChip						= ADDIDATA_S5933,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 8,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 16383,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= &range_apci3501_ao,
		.i_NbrDiChannel						= 2,
		.i_NbrDoChannel						= 2,
		.i_DoMaxdata						= 0x3,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 1,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3501_Interrupt,
		.i_hwdrv_Reset						= i_APCI3501_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= i_APCI3501_ConfigAnalogOutput,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3501_WriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3501_ReadDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI3501_ConfigDigitalOutput,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3501_WriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3501_ReadDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= i_APCI3501_ConfigTimerCounterWatchdog,
		.i_hwdrv_InsnWriteTimer				= i_APCI3501_StartStopWriteTimerCounterWatchdog,
		.i_hwdrv_InsnReadTimer				= i_APCI3501_ReadTimerCounterWatchdog,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_035
	{
		.pc_DriverName						= "apci035",
		.i_VendorId							= APCI035_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x0300,
		.i_IorangeBase0						= 127,
		.i_IorangeBase1						= APCI035_ADDRESS_RANGE,
		.i_IorangeBase2						= 0,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= 1,
		.pc_EepromChip						= ADDIDATA_S5920,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0xff,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci035_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 1,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 100000,
		.v_hwdrv_Interrupt					= v_APCI035_Interrupt,
		.i_hwdrv_Reset						= i_APCI035_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI035_ConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI035_ReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= i_APCI035_ConfigTimerWatchdog,
		.i_hwdrv_InsnWriteTimer				= i_APCI035_StartStopWriteTimerWatchdog,
		.i_hwdrv_InsnReadTimer				= i_APCI035_ReadTimerWatchdog,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_3200
	{
		.pc_DriverName						= "apci3200",
		.i_VendorId							= APCI3200_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x3000,
		.i_IorangeBase0						= 128,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 4,
		.i_IorangeBase3						= 4,
		.i_PCIEeprom						= ADDIDATA_EEPROM,
		.pc_EepromChip						= ADDIDATA_S5920,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0x3ffff,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3200_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 100000,
		.v_hwdrv_Interrupt					= v_APCI3200_Interrupt,
		.i_hwdrv_Reset						= i_APCI3200_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3200_ConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3200_ReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= i_APCI3200_InsnWriteReleaseAnalogInput,
		.i_hwdrv_InsnBitsAnalogInput		= i_APCI3200_InsnBits_AnalogInput_Test,
		.i_hwdrv_CommandTestAnalogInput		= i_APCI3200_CommandTestAnalogInput,
		.i_hwdrv_CommandAnalogInput			= i_APCI3200_CommandAnalogInput,
		.i_hwdrv_CancelAnalogInput			= i_APCI3200_StopCyclicAcquisition,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3200_ReadDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI3200_ConfigDigitalOutput,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3200_WriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3200_ReadDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_3300
	//Begin JK 20.10.2004: APCI-3300 integration
	{
		.pc_DriverName						= "apci3300",
		.i_VendorId							= APCI3200_BOARD_VENDOR_ID,
		.i_DeviceId							= 0x3007,
		.i_IorangeBase0						= 128,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 4,
		.i_IorangeBase3						= 4,
		.i_PCIEeprom						= ADDIDATA_EEPROM,
		.pc_EepromChip						= ADDIDATA_S5920,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 8,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0x3ffff,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3300_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 100000,
		.v_hwdrv_Interrupt					= v_APCI3200_Interrupt,
		.i_hwdrv_Reset						= i_APCI3200_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3200_ConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3200_ReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= i_APCI3200_InsnWriteReleaseAnalogInput,
		.i_hwdrv_InsnBitsAnalogInput		= i_APCI3200_InsnBits_AnalogInput_Test,
		.i_hwdrv_CommandTestAnalogInput		= i_APCI3200_CommandTestAnalogInput,
		.i_hwdrv_CommandAnalogInput			= i_APCI3200_CommandAnalogInput,
		.i_hwdrv_CancelAnalogInput			= i_APCI3200_StopCyclicAcquisition,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3200_ReadDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= i_APCI3200_ConfigDigitalOutput,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3200_WriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3200_ReadDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_1710
	{
		.pc_DriverName						= "apci1710",
		.i_VendorId							= APCI1710_BOARD_VENDOR_ID,
		.i_DeviceId							= APCI1710_BOARD_DEVICE_ID,
		.i_IorangeBase0						= 128,
		.i_IorangeBase1						= 8,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= NULL,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI1710_Interrupt,
		.i_hwdrv_Reset						= i_APCI1710_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
#endif
#ifdef CONFIG_APCI_16XX
	{
		.pc_DriverName						= "apci1648",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x1009,
		.i_IorangeBase0						= 128,
		.i_IorangeBase1						= 0,
		.i_IorangeBase2						= 0,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= NULL,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 48,
		.pr_TTLRangelist					= &range_apci16xx_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= NULL,
		.i_hwdrv_Reset						= i_APCI16XX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI16XX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI16XX_InsnBitsReadTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI16XX_InsnReadTTLIOAllPortValue,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI16XX_InsnBitsWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci1696",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x100A,
		.i_IorangeBase0						= 128,
		.i_IorangeBase1						= 0,
		.i_IorangeBase2						= 0,
		.i_IorangeBase3						= 0,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= NULL,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 96,
		.pr_TTLRangelist					= &range_apci16xx_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= NULL,
		.i_hwdrv_Reset						= i_APCI16XX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI16XX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI16XX_InsnBitsReadTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI16XX_InsnReadTTLIOAllPortValue,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI16XX_InsnBitsWriteTTLIO,
	},
#endif
#ifdef CONFIG_APCI_3XXX
	{
		.pc_DriverName						= "apci3000-16",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3010,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 4095,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3000-8",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x300F,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 8,
		.i_NbrAiChannelDiff					= 4,
		.i_AiChannelList					= 8,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 4095,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3000-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x300E,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 4,
		.i_NbrAiChannelDiff					= 2,
		.i_AiChannelList					= 4,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 4095,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3006-16",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3013,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3006-8",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3014,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 8,
		.i_NbrAiChannelDiff					= 4,
		.i_AiChannelList					= 8,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3006-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3015,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 4,
		.i_NbrAiChannelDiff					= 2,
		.i_AiChannelList					= 4,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3010-16",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3016,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 4095,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3010-8",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3017,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 8,
		.i_NbrAiChannelDiff					= 4,
		.i_AiChannelList					= 8,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 4095,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3010-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3018,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 4,
		.i_NbrAiChannelDiff					= 2,
		.i_AiChannelList					= 4,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 4095,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3016-16",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3019,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3016-8",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x301A,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 8,
		.i_NbrAiChannelDiff					= 4,
		.i_AiChannelList					= 8,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3016-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x301B,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 4,
		.i_NbrAiChannelDiff					= 2,
		.i_AiChannelList					= 4,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3100-16-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x301C,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 4,
		.i_AiMaxdata						= 4095,
		.i_AoMaxdata						= 4095,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= &range_apci3XXX_ao,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3XXX_InsnWriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3100-8-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x301D,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 8,
		.i_NbrAiChannelDiff					= 4,
		.i_AiChannelList					= 8,
		.i_NbrAoChannel						= 4,
		.i_AiMaxdata						= 4095,
		.i_AoMaxdata						= 4095,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= &range_apci3XXX_ao,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3XXX_InsnWriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3106-16-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x301E,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 4,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 4095,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= &range_apci3XXX_ao,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3XXX_InsnWriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3106-8-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x301F,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 8,
		.i_NbrAiChannelDiff					= 4,
		.i_AiChannelList					= 8,
		.i_NbrAoChannel						= 4,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 4095,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= &range_apci3XXX_ao,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 10000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3XXX_InsnWriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3110-16-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3020,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 4,
		.i_AiMaxdata						= 4095,
		.i_AoMaxdata						= 4095,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= &range_apci3XXX_ao,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3XXX_InsnWriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3110-8-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3021,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 8,
		.i_NbrAiChannelDiff					= 4,
		.i_AiChannelList					= 8,
		.i_NbrAoChannel						= 4,
		.i_AiMaxdata						= 4095,
		.i_AoMaxdata						= 4095,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= &range_apci3XXX_ao,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3XXX_InsnWriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3116-16-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3022,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 16,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 4,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 4095,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= &range_apci3XXX_ao,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3XXX_InsnWriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3116-8-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3023,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 8,
		.i_NbrAiChannelDiff					= 4,
		.i_AiChannelList					= 8,
		.i_NbrAoChannel						= 4,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 4095,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= &range_apci3XXX_ao,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3XXX_InsnWriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
	{
		.pc_DriverName						= "apci3003",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x300B,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 4,
		.i_AiChannelList					= 4,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 7,
		.ui_MinAcquisitiontimeNs			= 2500,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
	{
		.pc_DriverName						= "apci3002-16",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3002,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 16,
		.i_AiChannelList					= 16,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
	{
		.pc_DriverName						= "apci3002-8",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3003,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 8,
		.i_AiChannelList					= 8,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
	{
		.pc_DriverName						= "apci3002-4",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3004,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 4,
		.i_AiChannelList					= 4,
		.i_NbrAoChannel						= 0,
		.i_AiMaxdata						= 65535,
		.i_AoMaxdata						= 0,
		.pr_AiRangelist						= &range_apci3XXX_ai,
		.pr_AoRangelist						= NULL,
		.i_NbrDiChannel						= 4,
		.i_NbrDoChannel						= 4,
		.i_DoMaxdata						= 1,
		.i_NbrTTLChannel					= 0,
		.pr_TTLRangelist					= NULL,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 6,
		.ui_MinAcquisitiontimeNs			= 5000,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= i_APCI3XXX_InsnConfigAnalogInput,
		.i_hwdrv_InsnReadAnalogInput		= i_APCI3XXX_InsnReadAnalogInput,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= NULL,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= i_APCI3XXX_InsnReadDigitalInput,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= i_APCI3XXX_InsnBitsDigitalInput,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= i_APCI3XXX_InsnWriteDigitalOutput,
		.i_hwdrv_InsnBitsDigitalOutput		= i_APCI3XXX_InsnBitsDigitalOutput,
		.i_hwdrv_InsnReadDigitalOutput		= i_APCI3XXX_InsnReadDigitalOutput,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= NULL,
		.i_hwdr_ReadTTLIOBits				= NULL,
		.i_hwdr_ReadTTLIOAllPortValue		= NULL,
		.i_hwdr_WriteTTLIOChlOnOff			= NULL,
	},
	{
		.pc_DriverName						= "apci3500",
		.i_VendorId							= 0x15B8,
		.i_DeviceId							= 0x3024,
		.i_IorangeBase0						= 256,
		.i_IorangeBase1						= 256,
		.i_IorangeBase2						= 256,
		.i_IorangeBase3						= 256,
		.i_PCIEeprom						= ADDIDATA_NO_EEPROM,
		.pc_EepromChip						= ADDIDATA_9054,
		.i_NbrAiChannel						= 0,
		.i_NbrAiChannelDiff					= 0,
		.i_AiChannelList					= 0,
		.i_NbrAoChannel						= 4,
		.i_AiMaxdata						= 0,
		.i_AoMaxdata						= 4095,
		.pr_AiRangelist						= NULL,
		.pr_AoRangelist						= &range_apci3XXX_ao,
		.i_NbrDiChannel						= 0,
		.i_NbrDoChannel						= 0,
		.i_DoMaxdata						= 0,
		.i_NbrTTLChannel					= 24,
		.pr_TTLRangelist					= &range_apci3XXX_ttl,
		.i_Dma								= 0,
		.i_Timer							= 0,
		.b_AvailableConvertUnit				= 0,
		.ui_MinAcquisitiontimeNs			= 0,
		.ui_MinDelaytimeNs					= 0,
		.v_hwdrv_Interrupt					= v_APCI3XXX_Interrupt,
		.i_hwdrv_Reset						= i_APCI3XXX_Reset,
		.i_hwdrv_InsnConfigAnalogInput		= NULL,
		.i_hwdrv_InsnReadAnalogInput		= NULL,
		.i_hwdrv_InsnWriteAnalogInput		= NULL,
		.i_hwdrv_InsnBitsAnalogInput		= NULL,
		.i_hwdrv_CommandTestAnalogInput		= NULL,
		.i_hwdrv_CommandAnalogInput			= NULL,
		.i_hwdrv_CancelAnalogInput			= NULL,
		.i_hwdrv_InsnConfigAnalogOutput		= NULL,
		.i_hwdrv_InsnWriteAnalogOutput		= i_APCI3XXX_InsnWriteAnalogOutput,
		.i_hwdrv_InsnBitsAnalogOutput		= NULL,
		.i_hwdrv_InsnConfigDigitalInput		= NULL,
		.i_hwdrv_InsnReadDigitalInput		= NULL,
		.i_hwdrv_InsnWriteDigitalInput		= NULL,
		.i_hwdrv_InsnBitsDigitalInput		= NULL,
		.i_hwdrv_InsnConfigDigitalOutput	= NULL,
		.i_hwdrv_InsnWriteDigitalOutput		= NULL,
		.i_hwdrv_InsnBitsDigitalOutput		= NULL,
		.i_hwdrv_InsnReadDigitalOutput		= NULL,
		.i_hwdrv_InsnConfigTimer			= NULL,
		.i_hwdrv_InsnWriteTimer				= NULL,
		.i_hwdrv_InsnReadTimer				= NULL,
		.i_hwdrv_InsnBitsTimer				= NULL,
		.i_hwdr_ConfigInitTTLIO				= i_APCI3XXX_InsnConfigInitTTLIO,
		.i_hwdr_ReadTTLIOBits				= i_APCI3XXX_InsnBitsTTLIO,
		.i_hwdr_ReadTTLIOAllPortValue		= i_APCI3XXX_InsnReadTTLIO,
		.i_hwdr_WriteTTLIOChlOnOff			= i_APCI3XXX_InsnWriteTTLIO,
	},
#endif
};

#define n_boardtypes ARRAY_SIZE(boardtypes)

comedi_driver driver_addi = {
	.driver_name	= ADDIDATA_DRIVER_NAME,
	.module			= THIS_MODULE,
	.attach			= i_ADDI_Attach,
	.detach			= i_ADDI_Detach,
	.num_names		= n_boardtypes,
	.board_name		= &boardtypes[0].pc_DriverName,
	.offset			= sizeof(boardtype),
};

COMEDI_PCI_INITCLEANUP(driver_addi, addi_apci_tbl);

/*
+----------------------------------------------------------------------------+
| Function name     :static int i_ADDI_Attach(comedi_device *dev,            |
|										comedi_devconfig *it)        |
|                                        									 |
+----------------------------------------------------------------------------+
| Task              :Detects the card.                                       |
|  			 Configure the driver for a particular board.            |
|  			 This function does all the initializations and memory   |
|			 allocation of data structures for the driver.	         |
+----------------------------------------------------------------------------+
| Input Parameters  :comedi_device *dev										 |
|                    comedi_devconfig *it									 |
|                                                 					         |
+----------------------------------------------------------------------------+
| Return Value      :  0            					                     |
|                    													     |
+----------------------------------------------------------------------------+
*/

static int i_ADDI_Attach(comedi_device * dev, comedi_devconfig * it)
{
	comedi_subdevice *s;
	int ret, order, i, n_subdevices;
	DWORD dw_Dummy;
	resource_size_t io_addr[5];
	unsigned int irq;
	resource_size_t iobase_a, iobase_main, iobase_addon, iobase_reserved;
	struct pcilst_struct *card = NULL;
	unsigned char pci_bus, pci_slot, pci_func;
	int i_Dma = 0;

	if ((ret = alloc_private(dev, sizeof(addi_private))) < 0) {
	  	return -ENOMEM;
	}

	//rt_printk("comedi%d: "ADDIDATA_DRIVER_NAME": board=%s\n",dev->minor,this_board->pc_DriverName);

	if ((this_board->i_Dma) && (it->options[2] == 0)) {
		i_Dma = 1;
	}

	if ((card = ptr_select_and_alloc_pci_card(this_board->i_VendorId,
				this_board->i_DeviceId,
				it->options[0],
				it->options[1], i_Dma)) == NULL) {
		return -EIO;
	}
	devpriv->allocated = 1;

	if ((i_pci_card_data(card, &pci_bus, &pci_slot, &pci_func, &io_addr[0],
				&irq)) < 0) {
		i_pci_card_free(card);
		printk(" - Can't get AMCC data!\n");
		return -EIO;
	}

	iobase_a = io_addr[0];
	iobase_main = io_addr[1];
	iobase_addon = io_addr[2];
	iobase_reserved = io_addr[3];
	printk("Bus %d: Slot %d: Funct%d\nBase0: 0x%8llx\nBase1: 0x%8llx\nBase2: 0x%8llx\nBase3: 0x%8llx\n", pci_bus, pci_slot, pci_func, (unsigned long long)io_addr[0], (unsigned long long)io_addr[1], (unsigned long long)io_addr[2], (unsigned long long)io_addr[3]);

	if ((this_board->pc_EepromChip == NULL)
		|| (strcmp(this_board->pc_EepromChip, ADDIDATA_9054) != 0)) {
	   /************************************/
		/* Test if more that 1 address used */
	   /************************************/

		if (this_board->i_IorangeBase1 != 0) {
			dev->iobase = (unsigned long)iobase_main;	// DAQ base address...
		} else {
			dev->iobase = (unsigned long)iobase_a;	// DAQ base address...
		}

		dev->board_name = this_board->pc_DriverName;
		devpriv->amcc = card;
		devpriv->iobase = (INT) dev->iobase;
		devpriv->i_IobaseAmcc = (INT) iobase_a;	//AMCC base address...
		devpriv->i_IobaseAddon = (INT) iobase_addon;	//ADD ON base address....
		devpriv->i_IobaseReserved = (INT) iobase_reserved;
	} else {
		dev->board_name = this_board->pc_DriverName;
		dev->iobase = (unsigned long)io_addr[2];
		devpriv->amcc = card;
		devpriv->iobase = (INT) io_addr[2];
		devpriv->i_IobaseReserved = (INT) io_addr[3];
		printk("ioremap begin\n");
		devpriv->dw_AiBase =
			(ULONG_PTR) ioremap(io_addr[3],
			this_board->i_IorangeBase3);
		if (!devpriv->dw_AiBase) {
			printk(" - Can't remap MMIO region\n");
			return -ENOMEM;
		}
		printk("ioremap end\n");
	}

	/* Initialize parameters that can be overridden in EEPROM */
	devpriv->s_EeParameters.i_NbrAiChannel = this_board->i_NbrAiChannel;
	devpriv->s_EeParameters.i_NbrAoChannel = this_board->i_NbrAoChannel;
	devpriv->s_EeParameters.i_AiMaxdata = this_board->i_AiMaxdata;
	devpriv->s_EeParameters.i_AoMaxdata = this_board->i_AoMaxdata;
	devpriv->s_EeParameters.i_NbrDiChannel = this_board->i_NbrDiChannel;
	devpriv->s_EeParameters.i_NbrDoChannel = this_board->i_NbrDoChannel;
	devpriv->s_EeParameters.i_DoMaxdata = this_board->i_DoMaxdata;
	devpriv->s_EeParameters.i_Dma = this_board->i_Dma;
	devpriv->s_EeParameters.i_Timer = this_board->i_Timer;
	devpriv->s_EeParameters.ui_MinAcquisitiontimeNs =
		this_board->ui_MinAcquisitiontimeNs;
	devpriv->s_EeParameters.ui_MinDelaytimeNs =
		this_board->ui_MinDelaytimeNs;

	//##

	if (irq > 0) {
		if (comedi_request_irq(irq, v_ADDI_Interrupt, IRQF_SHARED,
				ADDIDATA_DRIVER_NAME, dev) < 0) {
			printk(", unable to allocate IRQ %u, DISABLING IT\n",
				irq);
			irq = 0;	/* Can't use IRQ */
		} else {
			rt_printk("\nirq=%u", irq);
		}
	} else {
		rt_printk(", IRQ disabled\n");
	}

	printk("Option %d %d %d\n", it->options[0], it->options[1],
		it->options[2]);
	dev->irq = irq;

	// Read eepeom and fill boardtype Structure

	if (this_board->i_PCIEeprom) {
		printk("PCI Eeprom used\n");
		if (!(strcmp(this_board->pc_EepromChip, "S5920"))) {
			// Set 3 wait stait
			if (!(strcmp(this_board->pc_DriverName, "apci035"))) {
				outl(0x80808082, devpriv->i_IobaseAmcc + 0x60);
			} else {
				outl(0x83838383, devpriv->i_IobaseAmcc + 0x60);
			}
			// Enable the interrupt for the controler
			dw_Dummy = inl(devpriv->i_IobaseAmcc + 0x38);
			outl(dw_Dummy | 0x2000, devpriv->i_IobaseAmcc + 0x38);
			printk("Enable the interrupt for the controler\n");
		}
		printk("Read Eeprom\n");
		i_EepromReadMainHeader(io_addr[0], this_board->pc_EepromChip,
			dev);
	} else {
		printk("PCI Eeprom unused\n");
	}

	if (it->options[2] > 0) {
		devpriv->us_UseDma = ADDI_DISABLE;
	} else {
		devpriv->us_UseDma = ADDI_ENABLE;
	}

	if (devpriv->s_EeParameters.i_Dma) {
		printk("DMA used\n");
		pci_set_master(devpriv->amcc->pcidev);
		if (dma_set_mask(&devpriv->amcc->pcidev->dev,
				((1ULL << 32) - 1)) != 0 ||
			dma_set_coherent_mask(&devpriv->amcc->pcidev->dev,
				((1ULL << 32) - 1)) != 0) {
			if (devpriv->us_UseDma == ADDI_ENABLE) {
				rt_printk
					(", Can't set DMA mask, DMA disabled!\n");
			}
			devpriv->us_UseDma = ADDI_DISABLE;
		}

		if (devpriv->us_UseDma == ADDI_ENABLE) {
			// alloc DMA buffers
			devpriv->b_DmaDoubleBuffer = 0;
			for (i = 0; i < 2; i++) {
				dma_addr_t hwaddr;

				for (order = 2; order >= 0; order--) {
					if ((devpriv->ul_DmaBufferVirtual[i] =
						dma_alloc_coherent(
							&devpriv->amcc->pcidev->dev,
							(PAGE_SIZE << order),
							&hwaddr, GFP_KERNEL))) {
						break;
					}
				}
				if (devpriv->ul_DmaBufferVirtual[i]) {
					devpriv->ui_DmaBufferPages[i] =
						(1 << order);
					devpriv->ui_DmaBufferSize[i] =
						devpriv->ui_DmaBufferPages[i] *
						PAGE_SIZE;
					devpriv->ui_DmaBufferSamples[i] =
						devpriv->
						ui_DmaBufferSize[i] >> 1;
					devpriv->ul_DmaBufferHw[i] = hwaddr;
				}
			}
			if (!devpriv->ul_DmaBufferVirtual[0]) {
				rt_printk
					(", Can't allocate DMA buffer, DMA disabled!\n");
				devpriv->us_UseDma = ADDI_DISABLE;
			}

			if (devpriv->ul_DmaBufferVirtual[1]) {
				devpriv->b_DmaDoubleBuffer = 1;
			}
		}

		if ((devpriv->us_UseDma == ADDI_ENABLE)) {
			rt_printk("DMA ENABLED\n");
		} else {
			printk("DMA DISABLED\n");
		}
	}

	if (!strcmp(this_board->pc_DriverName, "apci1710")) {
#ifdef CONFIG_APCI_1710
		i_ADDI_AttachPCI1710(dev);

		// save base address
		devpriv->s_BoardInfos.ui_Address = io_addr[2];
#endif
	} else {
		//Update-0.7.57->0.7.68dev->n_subdevices = 7;
		n_subdevices = 7;
		if ((ret = alloc_subdevices(dev, n_subdevices)) < 0)
			return ret;

		// Allocate and Initialise AI Subdevice Structures
		s = dev->subdevices + 0;
		if ((devpriv->s_EeParameters.i_NbrAiChannel)
			|| (this_board->i_NbrAiChannelDiff)) {
			dev->read_subdev = s;
			s->type = COMEDI_SUBD_AI;
			s->subdev_flags =
				SDF_READABLE | SDF_RT | SDF_COMMON | SDF_GROUND
				| SDF_DIFF;
			if (devpriv->s_EeParameters.i_NbrAiChannel) {
				s->n_chan = devpriv->s_EeParameters.i_NbrAiChannel;
				devpriv->b_SingelDiff = 0;
			} else {
				s->n_chan = this_board->i_NbrAiChannelDiff;
				devpriv->b_SingelDiff = 1;
			}
			s->maxdata = devpriv->s_EeParameters.i_AiMaxdata;
			s->len_chanlist = this_board->i_AiChannelList;
			s->range_table = this_board->pr_AiRangelist;

			/* Set the initialisation flag */
			devpriv->b_AiInitialisation = 1;

			s->insn_config =
				this_board->i_hwdrv_InsnConfigAnalogInput;
			s->insn_read = this_board->i_hwdrv_InsnReadAnalogInput;
			s->insn_write =
				this_board->i_hwdrv_InsnWriteAnalogInput;
			s->insn_bits = this_board->i_hwdrv_InsnBitsAnalogInput;
			s->do_cmdtest =
				this_board->i_hwdrv_CommandTestAnalogInput;
			s->do_cmd = this_board->i_hwdrv_CommandAnalogInput;
			s->cancel = this_board->i_hwdrv_CancelAnalogInput;

		} else {
			s->type = COMEDI_SUBD_UNUSED;
		}

		// Allocate and Initialise AO Subdevice Structures
		s = dev->subdevices + 1;
		if (devpriv->s_EeParameters.i_NbrAoChannel) {
			s->type = COMEDI_SUBD_AO;
			s->subdev_flags =
				SDF_WRITEABLE | SDF_GROUND | SDF_COMMON |
				SDF_RT;
			s->n_chan = devpriv->s_EeParameters.i_NbrAoChannel;
			s->maxdata = devpriv->s_EeParameters.i_AoMaxdata;
			s->len_chanlist = devpriv->s_EeParameters.i_NbrAoChannel;
			s->range_table = this_board->pr_AoRangelist;
			s->insn_config =
				this_board->i_hwdrv_InsnConfigAnalogOutput;
			s->insn_write =
				this_board->i_hwdrv_InsnWriteAnalogOutput;
		} else {
			s->type = COMEDI_SUBD_UNUSED;
		}
		// Allocate and Initialise DI Subdevice Structures
		s = dev->subdevices + 2;
		if (devpriv->s_EeParameters.i_NbrDiChannel) {
			s->type = COMEDI_SUBD_DI;
			s->subdev_flags =
				SDF_READABLE | SDF_RT | SDF_GROUND | SDF_COMMON;
			s->n_chan = devpriv->s_EeParameters.i_NbrDiChannel;
			s->maxdata = 1;
			s->len_chanlist = devpriv->s_EeParameters.i_NbrDiChannel;
			s->range_table = &range_digital;
			s->io_bits = 0;	/* all bits input */
			s->insn_config =
				this_board->i_hwdrv_InsnConfigDigitalInput;
			s->insn_read = this_board->i_hwdrv_InsnReadDigitalInput;
			s->insn_write =
				this_board->i_hwdrv_InsnWriteDigitalInput;
			s->insn_bits = this_board->i_hwdrv_InsnBitsDigitalInput;
		} else {
			s->type = COMEDI_SUBD_UNUSED;
		}
		// Allocate and Initialise DO Subdevice Structures
		s = dev->subdevices + 3;
		if (devpriv->s_EeParameters.i_NbrDoChannel) {
			s->type = COMEDI_SUBD_DO;
			s->subdev_flags =
				SDF_READABLE | SDF_WRITEABLE | SDF_RT |
				SDF_GROUND | SDF_COMMON;
			s->n_chan = devpriv->s_EeParameters.i_NbrDoChannel;
			s->maxdata = devpriv->s_EeParameters.i_DoMaxdata;
			s->len_chanlist = devpriv->s_EeParameters.i_NbrDoChannel;
			s->range_table = &range_digital;
			s->io_bits = 0xf;	/* all bits output */

			s->insn_config = this_board->i_hwdrv_InsnConfigDigitalOutput;	//for digital output memory..
			s->insn_write =
				this_board->i_hwdrv_InsnWriteDigitalOutput;
			s->insn_bits =
				this_board->i_hwdrv_InsnBitsDigitalOutput;
			s->insn_read =
				this_board->i_hwdrv_InsnReadDigitalOutput;
		} else {
			s->type = COMEDI_SUBD_UNUSED;
		}

		// Allocate and Initialise Timer Subdevice Structures
		s = dev->subdevices + 4;
		if (devpriv->s_EeParameters.i_Timer) {
			s->type = COMEDI_SUBD_TIMER;
			s->subdev_flags =
				SDF_WRITEABLE | SDF_RT | SDF_GROUND |
				SDF_COMMON;
			s->n_chan = 1;
			s->maxdata = 0;
			s->len_chanlist = 1;
			s->range_table = &range_digital;

			s->insn_write = this_board->i_hwdrv_InsnWriteTimer;
			s->insn_read = this_board->i_hwdrv_InsnReadTimer;
			s->insn_config = this_board->i_hwdrv_InsnConfigTimer;
			s->insn_bits = this_board->i_hwdrv_InsnBitsTimer;
		} else {
			s->type = COMEDI_SUBD_UNUSED;
		}

		// Allocate and Initialise TTL
		s = dev->subdevices + 5;
		if (this_board->i_NbrTTLChannel) {
			s->type = COMEDI_SUBD_TTLIO;
			s->subdev_flags =
				SDF_WRITEABLE | SDF_READABLE | SDF_RT |
				SDF_GROUND | SDF_COMMON;
			s->n_chan = this_board->i_NbrTTLChannel;
			s->maxdata = 1;
			s->io_bits = 0;	/* all bits input */
			s->len_chanlist = this_board->i_NbrTTLChannel;
			s->range_table = &range_digital;
			s->insn_config = this_board->i_hwdr_ConfigInitTTLIO;
			s->insn_bits = this_board->i_hwdr_ReadTTLIOBits;
			s->insn_read = this_board->i_hwdr_ReadTTLIOAllPortValue;
			s->insn_write = this_board->i_hwdr_WriteTTLIOChlOnOff;
		} else {
			s->type = COMEDI_SUBD_UNUSED;
		}

		/* EEPROM */
		s = dev->subdevices + 6;
		if (this_board->i_PCIEeprom) {
			s->type = COMEDI_SUBD_MEMORY;
			s->subdev_flags = SDF_READABLE | SDF_INTERNAL;
			s->n_chan = 256;
			s->maxdata = 0xffff;
			s->insn_read = i_ADDIDATA_InsnReadEeprom;
		} else {
			s->type = COMEDI_SUBD_UNUSED;
		}
	}

	printk("i_ADDI_Attach end\n");
	i_ADDI_Reset(dev);
	devpriv->b_ValidDriver = 1;
	return 0;
}

/*
+----------------------------------------------------------------------------+
| Function name     : static int i_ADDI_Detach(comedi_device *dev)           |
|                                        									 |
|                                            						         |
+----------------------------------------------------------------------------+
| Task              : Deallocates resources of the addi_common driver        |
|			  Free the DMA buffers, unregister irq.				     |
|                     										                 |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev									 |
|                     														 |
|                                                 					         |
+----------------------------------------------------------------------------+
| Return Value      : 0             					                     |
|                    													     |
+----------------------------------------------------------------------------+
*/

static int i_ADDI_Detach(comedi_device * dev)
{

	if (dev->private) {
		if (devpriv->b_ValidDriver) {
			i_ADDI_Reset(dev);
		}

		if (dev->irq) {
			comedi_free_irq(dev->irq, dev);
		}

		if ((this_board->pc_EepromChip == NULL)
			|| (strcmp(this_board->pc_EepromChip,
					ADDIDATA_9054) != 0)) {
			if (devpriv->allocated) {
				i_pci_card_free(devpriv->amcc);
			}

			if (devpriv->ul_DmaBufferVirtual[0]) {
				dma_free_coherent(&devpriv->amcc->pcidev->dev,
					devpriv->ui_DmaBufferSize[0],
					devpriv->ul_DmaBufferVirtual[0],
					devpriv->ul_DmaBufferHw[0]);
			}

			if (devpriv->ul_DmaBufferVirtual[1]) {
				dma_free_coherent(&devpriv->amcc->pcidev->dev,
					devpriv->ui_DmaBufferSize[1],
					devpriv->ul_DmaBufferVirtual[1],
					devpriv->ul_DmaBufferHw[1]);
			}
		} else {
			if (devpriv->dw_AiBase) {
				iounmap((void *)devpriv->dw_AiBase);
			}

			if (devpriv->allocated) {
				i_pci_card_free(devpriv->amcc);
			}
		}

		pci_card_free(devpriv->amcc);
	}

	return 0;
}

/*
+----------------------------------------------------------------------------+
| Function name     : static int i_ADDI_Reset(comedi_device *dev)			 |
|                                        									 |
+----------------------------------------------------------------------------+
| Task              : Disables all interrupts, Resets digital output to low, |
|				Set all analog output to low						 |
|                     										                 |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev									 |
|                     														 |
|                                                 					         |
+----------------------------------------------------------------------------+
| Return Value      : 0           					                         |
|                    													     |
+----------------------------------------------------------------------------+
*/

static int i_ADDI_Reset(comedi_device * dev)
{

	this_board->i_hwdrv_Reset(dev);
	return 0;
}

// Interrupt function
/*
+----------------------------------------------------------------------------+
| Function name     :                                                        |
|static void v_ADDI_Interrupt(int irq, void *d  PT_REGS_ARG)                 |
|                                        									 |
+----------------------------------------------------------------------------+
| Task              : Registerd interrupt routine						     |
|                     										                 |
+----------------------------------------------------------------------------+
| Input Parameters  : 	int irq												 |
|                     														 |
|                                                 					         |
+----------------------------------------------------------------------------+
| Return Value      :              					                         |
|                    													     |
+----------------------------------------------------------------------------+
*/

static irqreturn_t v_ADDI_Interrupt(int irq, void *d PT_REGS_ARG)
{
	comedi_device *dev = d;
	this_board->v_hwdrv_Interrupt(irq, d);
	return IRQ_RETVAL(1);
}

// EEPROM Read Function
/*
+----------------------------------------------------------------------------+
| Function name     :                                                        |
|INT i_ADDIDATA_InsnReadEeprom(comedi_device *dev,comedi_subdevice *s,
							comedi_insn *insn,lsampl_t *data)
|                                        									 |
+----------------------------------------------------------------------------+
| Task              : Read 256 words from EEPROM          				     |
|                     										                 |
+----------------------------------------------------------------------------+
| Input Parameters  :(comedi_device *dev,comedi_subdevice *s,
			comedi_insn *insn,lsampl_t *data) 						 |
|                     														 |
|                                                 					         |
+----------------------------------------------------------------------------+
| Return Value      :              					                         |
|                    													     |
+----------------------------------------------------------------------------+
*/

static int i_ADDIDATA_InsnReadEeprom(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data)
{
	WORD w_Data;
	WORD w_Address;
	w_Address = CR_CHAN(insn->chanspec);	// address to be read as 0,1,2,3...255

	if (insn->n == 0)
		return 0;

	w_Data = w_EepromReadWord(devpriv->i_IobaseAmcc,
		this_board->pc_EepromChip, 0x100 + (2 * w_Address));
	data[0] = w_Data;
	//multiplied by 2 bcozinput will be like 0,1,2...255
	return insn->n;

}
