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
#include<linux/sched.h>
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
#include<asm/i387.h>
#include "../comedi_fc.h"

#include "addi_common.h"
#include "addi_amcc_s5933.h"

//Update-0.7.57->0.7.68MODULE_AUTHOR("ADDI-DATA GmbH <info@addi-data.com>");
//Update-0.7.57->0.7.68MODULE_DESCRIPTION("Comedi ADDI-DATA module");
//Update-0.7.57->0.7.68MODULE_LICENSE("GPL");

#define devpriv ((addi_private *)dev->private)
#define this_board ((boardtype *)dev->board_ptr)

//BYTE b_SaveFPUReg [94];

void 	fpu_begin (void)
	{
	//asm ("fstenv b_SaveFPUReg");
	kernel_fpu_begin ();
	}

void 	fpu_end (void)
	{
	// asm ("frstor b_SaveFPUReg");
	kernel_fpu_end ();
	}

#include "addi_eeprom.c"
#include "hwdrv_apci3120.c"
#include "hwdrv_apci1032.c"
#include "hwdrv_apci1516.c"
#include "hwdrv_apci2016.c"
#include "hwdrv_apci2032.c"
#include "hwdrv_apci2200.c"
#include "hwdrv_apci1564.c"
#include "hwdrv_apci1500.c"
#include "hwdrv_apci3501.c"
#include "hwdrv_apci035.c"
#include "hwdrv_apci3200.c" 
#include "hwdrv_APCI1710.c"
#include "hwdrv_apci16xx.c"
#include "hwdrv_apci3xxx.c"


static boardtype boardtypes[] =
{
		{"apci3120", 
		APCI3120_BOARD_VENDOR_ID,
		0x818D,
	 	AMCC_OP_REG_SIZE,
		APCI3120_ADDRESS_RANGE,
		8,
		0,
                ADDIDATA_NO_EEPROM,
		NULL,
                16, 
		8, 
		16, 
		8, 
		0xffff, 
		0x3fff,
	 	&range_apci3120_ai, 
		&range_apci3120_ao,
          	4,
		4,
		0x0f,
		0,
		NULL,
		1,
		1,
		1,
		10000,
		100000,
                v_APCI3120_Interrupt,
                i_APCI3120_Reset,
                i_APCI3120_InsnConfigAnalogInput,
		i_APCI3120_InsnReadAnalogInput,
                NULL,
                NULL,
                i_APCI3120_CommandTestAnalogInput,
		i_APCI3120_CommandAnalogInput,
                i_APCI3120_StopCyclicAcquisition,
                NULL,
		i_APCI3120_InsnWriteAnalogOutput,
                NULL,
                NULL,
		i_APCI3120_InsnReadDigitalInput,
                NULL,
		i_APCI3120_InsnBitsDigitalInput,
		i_APCI3120_InsnConfigDigitalOutput,
                i_APCI3120_InsnWriteDigitalOutput,
                i_APCI3120_InsnBitsDigitalOutput,
                NULL,
		i_APCI3120_InsnConfigTimer,
		i_APCI3120_InsnWriteTimer,
		i_APCI3120_InsnReadTimer,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
		},
    
                {"apci1032",
		APCI1032_BOARD_VENDOR_ID,
		0x1003,
                4,
		APCI1032_ADDRESS_RANGE,
		0,
		0,
                ADDIDATA_EEPROM,
		ADDIDATA_93C76, 
		0,
		0,
		0,
		0,
		0,
		0,
		0,
		0,
                32,
		0,
		0,
		0,
		NULL,
		0,
		0,
		0,
		0,
		0,
                v_APCI1032_Interrupt,		 
                i_APCI1032_Reset,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                i_APCI1032_ConfigDigitalInput,
                i_APCI1032_Read1DigitalInput,
                NULL,
                i_APCI1032_ReadMoreDigitalInput, 
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
                },
		
                {"apci1516",
		APCI1516_BOARD_VENDOR_ID,
		0x1001,
		128,
		APCI1516_ADDRESS_RANGE,
		32,
		0,
                ADDIDATA_EEPROM,
		ADDIDATA_S5920,
                0,
		0,
		0,
		0,
		0,
		0,
		NULL,
		NULL,
		8,
                8,
		0,
		0,
		NULL,
		0,
		1,
		0,
		0,
		0,
                NULL,		 
                i_APCI1516_Reset,
                NULL,NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                i_APCI1516_Read1DigitalInput,
                NULL,
                i_APCI1516_ReadMoreDigitalInput,
                i_APCI1516_ConfigDigitalOutput,
                i_APCI1516_WriteDigitalOutput,
                i_APCI1516_ReadDigitalOutput,
                NULL,
                i_APCI1516_ConfigWatchdog,
                i_APCI1516_StartStopWriteWatchdog,
                i_APCI1516_ReadWatchdog,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
                },

                {"apci2016",
		APCI2016_BOARD_VENDOR_ID,
		0x1002,
                128,
		APCI2016_ADDRESS_RANGE,
		32,
		0,
		ADDIDATA_EEPROM,
		ADDIDATA_S5920,
		0,
		0,
		0,
		0,
		0,
		0,
                NULL,
		NULL,
                0,
		16,
		0,
		0,
		NULL,
		0,
		1,
		0,
		0,
		0,
                NULL, 
                i_APCI2016_Reset,
                NULL,
		NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                i_APCI2016_ConfigDigitalOutput,
                i_APCI2016_WriteDigitalOutput,
                i_APCI2016_BitsDigitalOutput,
                NULL,
                i_APCI2016_ConfigWatchdog,
                i_APCI2016_StartStopWriteWatchdog,
                i_APCI2016_ReadWatchdog,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
                },
 
                {"apci2032", 
		APCI2032_BOARD_VENDOR_ID, 
		0x1004,
	 	4,
		APCI2032_ADDRESS_RANGE,
		0,
		0,
		ADDIDATA_EEPROM,
		ADDIDATA_93C76,
	 	0, 
		0, 
		0, 
		0, 
		0, 
		0,
	 	NULL, 
		NULL,
          	0,
		32,
		0xffffffff,
		0,
		NULL,
		0,
		1,
		0,
		0,
		0,
                v_APCI2032_Interrupt,                          
                i_APCI2032_Reset,
                NULL,NULL,                                        
		NULL,                                          
                NULL,                                         
		NULL,                                           
                NULL,                                          
                NULL,                                          
		NULL,                                         
                NULL,                                          
		NULL,                                          
		NULL,                                          
                NULL,
                NULL,
                NULL,
                i_APCI2032_ConfigDigitalOutput,
		i_APCI2032_WriteDigitalOutput,
                i_APCI2032_ReadDigitalOutput,
                i_APCI2032_ReadInterruptStatus, 
                i_APCI2032_ConfigWatchdog,         
		i_APCI2032_StartStopWriteWatchdog, 
		i_APCI2032_ReadWatchdog,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
                },
		
                {"apci2200",
		APCI2200_BOARD_VENDOR_ID,
		0x1005,
		4,
		APCI2200_ADDRESS_RANGE,
		0,
		0,
                ADDIDATA_EEPROM,
		ADDIDATA_93C76,
                0,
		0,
		0,
		0,
		0,
		0,
		NULL,
		NULL,
		8,
                16,
		0,
		0,
		NULL,
		0,
		1,
		0,
		0,
		0,
                NULL,		 
                i_APCI2200_Reset,
                NULL,NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                i_APCI2200_Read1DigitalInput,
                NULL,
                i_APCI2200_ReadMoreDigitalInput,
                i_APCI2200_ConfigDigitalOutput,
                i_APCI2200_WriteDigitalOutput,
                i_APCI2200_ReadDigitalOutput,
                NULL,
                i_APCI2200_ConfigWatchdog,
                i_APCI2200_StartStopWriteWatchdog,
                i_APCI2200_ReadWatchdog,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
                },
		
                {"apci1564",
		APCI1564_BOARD_VENDOR_ID,
		0x1006,
                128,
		APCI1564_ADDRESS_RANGE,
		0,
		0,
                ADDIDATA_EEPROM,
		ADDIDATA_93C76,
		0,
		0,
		0,
		0,
		0,
		0,
                NULL,
		NULL,
                32,
		32,
		0xffffffff,
		0,
		NULL,
		0,
		1,
		0,
		0,
		0,
                v_APCI1564_Interrupt,		 
                i_APCI1564_Reset,
                NULL,
		NULL, 
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                NULL,
                i_APCI1564_ConfigDigitalInput,
                i_APCI1564_Read1DigitalInput,
                NULL,  
                i_APCI1564_ReadMoreDigitalInput, 
                i_APCI1564_ConfigDigitalOutput,
                i_APCI1564_WriteDigitalOutput,
                i_APCI1564_ReadDigitalOutput,
                i_APCI1564_ReadInterruptStatus,
                i_APCI1564_ConfigTimerCounterWatchdog,
                i_APCI1564_StartStopWriteTimerCounterWatchdog,
                i_APCI1564_ReadTimerCounterWatchdog,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
                },
		
                {"apci1500", 
		APCI1500_BOARD_VENDOR_ID, 
		0x80fc,
	 	128,
		APCI1500_ADDRESS_RANGE,
		4,
		0,
                ADDIDATA_NO_EEPROM,
		NULL,
	 	0, 
		0, 
		0, 
		0, 
		0, 
		0,
		NULL, 
		NULL,
        	16,
		16,
		0xffff,
		0,
		NULL,
		0,
		1,
		0,
		0,
		0,
                v_APCI1500_Interrupt,                          
                i_APCI1500_Reset, 
                NULL,
		NULL,                                       
		NULL,                                          
                NULL,                                         
		NULL,                                           
                NULL,                                          
                NULL,                                          
		NULL,                                         
                NULL,                                          
		NULL,                                          
		i_APCI1500_ConfigDigitalInputEvent,                                          
                i_APCI1500_Initialisation,
                i_APCI1500_StartStopInputEvent,
                i_APCI1500_ReadMoreDigitalInput,
                i_APCI1500_ConfigDigitalOutputErrorInterrupt,
		i_APCI1500_WriteDigitalOutput,
                i_APCI1500_ConfigureInterrupt,
                NULL, 
                i_APCI1500_ConfigCounterTimerWatchdog,         
		i_APCI1500_StartStopTriggerTimerCounterWatchdog, 
                i_APCI1500_ReadInterruptMask,
		i_APCI1500_ReadCounterTimerWatchdog,
		NULL,
		NULL,
		NULL,
		NULL
                },
                

               {"apci3001", 
	        APCI3120_BOARD_VENDOR_ID, 
		0x828D,
	 	AMCC_OP_REG_SIZE, 
		APCI3120_ADDRESS_RANGE,
		8,
		0,
                ADDIDATA_NO_EEPROM,
		NULL,
                16, 
		8, 
		16, 
		0, 
		0xfff, 
		0,
	 	&range_apci3120_ai, 
		NULL,
          	4,
		4,
		0x0f,
		0,
		NULL,
		1,
		1,
		1,
		10000,
		100000,
                v_APCI3120_Interrupt,
                i_APCI3120_Reset,
                i_APCI3120_InsnConfigAnalogInput,
		i_APCI3120_InsnReadAnalogInput,
                NULL,
		NULL,
                i_APCI3120_CommandTestAnalogInput,
		i_APCI3120_CommandAnalogInput,
                i_APCI3120_StopCyclicAcquisition,
                NULL,
		NULL,
                NULL,
                NULL,
		i_APCI3120_InsnReadDigitalInput,
                NULL,
		i_APCI3120_InsnBitsDigitalInput,
		i_APCI3120_InsnConfigDigitalOutput,
                i_APCI3120_InsnWriteDigitalOutput,
                i_APCI3120_InsnBitsDigitalOutput,
                NULL,
		i_APCI3120_InsnConfigTimer,
		i_APCI3120_InsnWriteTimer,
		i_APCI3120_InsnReadTimer,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
		},
		
                {"apci3501", 
		APCI3501_BOARD_VENDOR_ID, 
		0x3001,
	 	64,
		APCI3501_ADDRESS_RANGE,
		0,
		0,
                ADDIDATA_EEPROM,
		ADDIDATA_S5933,
                0, 
		0, 
		0,
		8, 
		0,
		16383,
	 	NULL,
		&range_apci3501_ao,
          	2,
		2,
		0x3,
		0,
		NULL,
		0,
		1,
		0,
		0,
		0,
                v_APCI3501_Interrupt,                         
                i_APCI3501_Reset,
                NULL,NULL,                                        
		NULL,                                          
                NULL,                                        
		NULL,                                           
                NULL,                                       
                NULL,                                        
		i_APCI3501_ConfigAnalogOutput,                                    
                i_APCI3501_WriteAnalogOutput,                                         
		NULL,                                      
		NULL, 
                NULL,
                NULL,                                       
                i_APCI3501_ReadDigitalInput,
                i_APCI3501_ConfigDigitalOutput,
		i_APCI3501_WriteDigitalOutput,
                i_APCI3501_ReadDigitalOutput,
                NULL,
                i_APCI3501_ConfigTimerCounterWatchdog,         
		i_APCI3501_StartStopWriteTimerCounterWatchdog, 
		i_APCI3501_ReadTimerCounterWatchdog,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
                },
		
                {"apci035", 
		APCI035_BOARD_VENDOR_ID, 
		0x0300,
	 	127,
		APCI035_ADDRESS_RANGE ,
		0,
		0,
		1,
		ADDIDATA_S5920,
	 	16,
		8,
		16,
		0, 
		0xff, 
		0,
	 	&range_apci035_ai, 
		NULL,
          	0,
		0,
		0,
		0,
		NULL,
		0,
		1,
		0,
		10000,
		100000,
                v_APCI035_Interrupt,
                i_APCI035_Reset, 
                i_APCI035_ConfigAnalogInput,
		i_APCI035_ReadAnalogInput ,
                NULL,
		NULL,
                NULL,
                NULL,
                NULL,
                NULL,
		NULL,
                NULL,
                NULL,
		NULL,
		NULL,
		NULL,
                NULL,
                NULL,
                NULL,
                NULL,
		i_APCI035_ConfigTimerWatchdog,
		i_APCI035_StartStopWriteTimerWatchdog,
		i_APCI035_ReadTimerWatchdog,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
		},
		
                {"apci3200", 
		APCI3200_BOARD_VENDOR_ID, 
		0x3000,
	 	128,
		256,
		4,
		4,
		ADDIDATA_EEPROM,
		ADDIDATA_S5920,
	 	16,
		8,
		16,
		0, 
		0x3ffff, 
		0,
	 	&range_apci3200_ai, 
		NULL,
          	4,
		4,
		0,
		0,
		NULL,
		0,
		0,
		0,
		10000,
		100000,
                v_APCI3200_Interrupt,
                i_APCI3200_Reset,    
                i_APCI3200_ConfigAnalogInput,
		i_APCI3200_ReadAnalogInput ,
                i_APCI3200_InsnWriteReleaseAnalogInput,
                i_APCI3200_InsnBits_AnalogInput_Test,
                i_APCI3200_CommandTestAnalogInput,
		i_APCI3200_CommandAnalogInput,
                i_APCI3200_StopCyclicAcquisition,
                NULL,
		NULL,
                NULL,
                NULL,
		NULL,
                NULL,
                i_APCI3200_ReadDigitalInput,
		i_APCI3200_ConfigDigitalOutput,
                i_APCI3200_WriteDigitalOutput,
                i_APCI3200_ReadDigitalOutput,
                NULL,
		NULL,
		NULL,
		NULL,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
		},
		
		//Begin JK 20.10.2004: APCI-3300 integration
                {"apci3300", 
		APCI3200_BOARD_VENDOR_ID, 
		0x3007,
	 	128,
		256,
		4,
		4,
		ADDIDATA_EEPROM,
		ADDIDATA_S5920,
	 	0,
		8,
		8,
		0, 
		0x3ffff, 
		0,
	 	&range_apci3300_ai, 
		NULL,
          	4,
		4,
		0,
		0,
		NULL,
		0,
		0,
		0,
		10000,
		100000,
                v_APCI3200_Interrupt,
                i_APCI3200_Reset,    
                i_APCI3200_ConfigAnalogInput,
		i_APCI3200_ReadAnalogInput ,
                i_APCI3200_InsnWriteReleaseAnalogInput,
                i_APCI3200_InsnBits_AnalogInput_Test,
                i_APCI3200_CommandTestAnalogInput,
		i_APCI3200_CommandAnalogInput,
                i_APCI3200_StopCyclicAcquisition,
                NULL,
		NULL,
                NULL,
                NULL,
		NULL,
                NULL,
                i_APCI3200_ReadDigitalInput,
		i_APCI3200_ConfigDigitalOutput,
                i_APCI3200_WriteDigitalOutput,
                i_APCI3200_ReadDigitalOutput,
                NULL,
		NULL,
		NULL,
		NULL,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
		},
		
		//End JK 20.10.2004: APCI-3300 integration				
		{"apci1710",APCI1710_BOARD_VENDOR_ID,APCI1710_BOARD_DEVICE_ID,
	 	128,
		8,
		256,
		0,
                ADDIDATA_NO_EEPROM,
		NULL,
                0, 
		0, 
		0, 
		0, 
		0, 
		0,
	 	NULL, 
		NULL,
          	0,
		0,
		0,
		0,
		NULL,
		0,
		0,
		0,
		0,
		0,
                v_APCI1710_Interrupt,		 
                i_APCI1710_Reset,
                NULL,
		NULL,
                NULL,
		NULL,
                NULL,
		NULL,
                NULL,
                NULL,
		NULL,
                NULL,
                NULL,
		NULL,
                NULL,
		NULL,
		NULL,
                NULL,
                NULL,
                NULL,
		NULL,
		NULL,
		NULL,
                NULL,
		NULL,
		NULL,
		NULL,
		NULL
		},
		
                {"apci1648", 
                0x15B8,
                0x1009,
	 	128,
	 	0,
	 	0,
	 	0,
                ADDIDATA_NO_EEPROM,
                NULL,
	 	0, 
	 	0, 
	 	0, 
	 	0, 
	 	0, 
	 	0,
	 	NULL,
	 	NULL,
          	0,
          	0,
          	0,
		48,
		&range_apci16xx_ttl,
          	0,
		0,
		0,
		0,
		0,
                NULL,
                i_APCI16XX_Reset,
                NULL,
                NULL,                                       
		NULL,                                          
                NULL,                                         
		NULL,                                           
                NULL,                                          
                NULL,                                          
		NULL,                                         
                NULL,                                          
		NULL,
		NULL,
                NULL,
                NULL,
                NULL,
                NULL,
		NULL,
                NULL,
                NULL, 
                NULL,
		NULL,
                NULL,
                NULL,
		i_APCI16XX_InsnConfigInitTTLIO,
		i_APCI16XX_InsnBitsReadTTLIO,
		i_APCI16XX_InsnReadTTLIOAllPortValue,
		i_APCI16XX_InsnBitsWriteTTLIO
                },


                {"apci1696", 
                0x15B8,
                0x100A,
	 	128,
	 	0,
	 	0,
	 	0,
                ADDIDATA_NO_EEPROM,
                NULL,
	 	0, 
	 	0, 
	 	0, 
	 	0, 
	 	0, 
	 	0,
	 	NULL,
	 	NULL,
          	0,
          	0,
          	0,
		96,
		&range_apci16xx_ttl,
          	0,
		0,
		0,
		0,
		0,
                NULL,
                i_APCI16XX_Reset,
                NULL,
                NULL,                                       
		NULL,                                          
                NULL,                                         
		NULL,                                           
                NULL,                                          
                NULL,                                          
		NULL,                                         
                NULL,                                          
		NULL,
		NULL,
                NULL,
                NULL,
                NULL,
                NULL,
		NULL,
                NULL,
                NULL, 
                NULL,
		NULL,
                NULL,
                NULL,
		i_APCI16XX_InsnConfigInitTTLIO,
		i_APCI16XX_InsnBitsReadTTLIO,
		i_APCI16XX_InsnReadTTLIOAllPortValue,
		i_APCI16XX_InsnBitsWriteTTLIO
                },

		{"apci3000-16", 
		0x15B8,
		0x3010,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		16, 
		8, 
		16, 
		0, 
		4095, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		0,
		0,
		0,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		10000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},
                
		{"apci3000-8", 
		0x15B8,
		0x300F,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		8, 
		4, 
		8, 
		0, 
		4095, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		0,
		0,
		0,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		10000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3000-4", 
		0x15B8,
		0x300E,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		4, 
		2, 
		4, 
		0, 
		4095, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		0,
		0,
		0,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		10000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3006-16", 
		0x15B8,
		0x3013,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		16, 
		8, 
		16, 
		0, 
		65535, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		0,
		0,
		0,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		10000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},
                
		{"apci3006-8", 
		0x15B8,
		0x3014,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		8, 
		4, 
		8, 
		0, 
		65535, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		0,
		0,
		0,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		10000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3006-4", 
		0x15B8,
		0x3015,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		4, 
		2, 
		4, 
		0, 
		65535, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		0,
		0,
		0,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		10000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3010-16", 
		0x15B8,
		0x3016,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		16, 
		8, 
		16, 
		0, 
		4095, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},
                
		{"apci3010-8", 
		0x15B8,
		0x3017,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		8, 
		4, 
		8, 
		0, 
		4095, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3010-4", 
		0x15B8,
		0x3018,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		4, 
		2, 
		4, 
		0, 
		4095, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3016-16", 
		0x15B8,
		0x3019,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		16, 
		8, 
		16, 
		0, 
		65535, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3016-8", 
		0x15B8,
		0x301A,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		8, 
		4, 
		8, 
		0, 
		65535, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3016-4", 
		0x15B8,
		0x301B,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		4, 
		2, 
		4, 
		0, 
		65535, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3100-16-4", 
		0x15B8,
		0x301C,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		16, 
		8, 
		16, 
		4, 
		4095, 
		4095,
		&range_apci3XXX_ai, 
		&range_apci3XXX_ao,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		10000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnWriteAnalogOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3100-8-4", 
		0x15B8,
		0x301D,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		8, 
		4, 
		8, 
		4, 
		4095, 
		4095,
		&range_apci3XXX_ai, 
		&range_apci3XXX_ao,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		10000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnWriteAnalogOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3106-16-4", 
		0x15B8,
		0x301E,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		16, 
		8, 
		16, 
		4, 
		65535, 
		4095,
		&range_apci3XXX_ai, 
		&range_apci3XXX_ao,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		10000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnWriteAnalogOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3106-8-4", 
		0x15B8,
		0x301F,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		8, 
		4, 
		8, 
		4, 
		65535, 
		4095,
		&range_apci3XXX_ai, 
		&range_apci3XXX_ao,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		10000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnWriteAnalogOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3110-16-4", 
		0x15B8,
		0x3020,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		16, 
		8, 
		16, 
		4, 
		4095, 
		4095,
		&range_apci3XXX_ai, 
		&range_apci3XXX_ao,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnWriteAnalogOutput,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3110-8-4", 
		0x15B8,
		0x3021,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		8, 
		4, 
		8, 
		4, 
		4095, 
		4095,
		&range_apci3XXX_ai, 
		&range_apci3XXX_ao,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnWriteAnalogOutput,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3116-16-4", 
		0x15B8,
		0x3022,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		16, 
		8, 
		16, 
		4, 
		65535, 
		4095,
		&range_apci3XXX_ai, 
		&range_apci3XXX_ao,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnWriteAnalogOutput,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3116-8-4", 
		0x15B8,
		0x3023,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		8, 
		4, 
		8, 
		4, 
		65535, 
		4095,
		&range_apci3XXX_ai, 
		&range_apci3XXX_ao,
		4,
		4,
		0xF,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnWriteAnalogOutput,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},

		{"apci3003", 
		0x15B8,
		0x300B,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		0, 
		4, 
		4, 
		0, 
		65535, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		4,
		4,
		0xF,
		0,
		NULL,
		0,
		0,
		7,
		2500,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
		},

		{"apci3002-16", 
		0x15B8,
		0x3002,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		0, 
		16, 
		16, 
		0, 
		65535, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		4,
		4,
		0xF,
		0,
		NULL,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
		},

		{"apci3002-8", 
		0x15B8,
		0x3003,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		0, 
		8, 
		8, 
		0, 
		65535, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		4,
		4,
		0xF,
		0,
		NULL,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
		},

		{"apci3002-4", 
		0x15B8,
		0x3004,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		0, 
		4, 
		4, 
		0, 
		65535, 
		0,
		&range_apci3XXX_ai, 
		NULL,
		4,
		4,
		0xF,
		0,
		NULL,
		0,
		0,
		6,
		5000,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		i_APCI3XXX_InsnConfigAnalogInput,
		i_APCI3XXX_InsnReadAnalogInput,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnReadDigitalInput,
		NULL,
		i_APCI3XXX_InsnBitsDigitalInput,
		NULL,
		i_APCI3XXX_InsnWriteDigitalOutput,
		i_APCI3XXX_InsnBitsDigitalOutput,
		i_APCI3XXX_InsnReadDigitalOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL
		},

		{"apci3500", 
		0x15B8,
		0x3024,
		256,
		256,
		256,
		256,
		ADDIDATA_NO_EEPROM,
		ADDIDATA_9054,
		0, 
		0, 
		0, 
		4, 
		0, 
		4095,
		NULL, 
		&range_apci3XXX_ao,
		0,
		0,
		0,
		24,
		&range_apci3XXX_ttl,
		0,
		0,
		0,
		0,
		0,
		v_APCI3XXX_Interrupt,
		i_APCI3XXX_Reset,
		NULL,
		NULL,                                       
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnWriteAnalogOutput,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		NULL,
		i_APCI3XXX_InsnConfigInitTTLIO,
		i_APCI3XXX_InsnBitsTTLIO,
		i_APCI3XXX_InsnReadTTLIO,
		i_APCI3XXX_InsnWriteTTLIO
		},
};

#define n_boardtypes (sizeof(boardtypes)/sizeof(boardtype))

comedi_driver driver_addi ={
		driver_name:	"addi_common",
		module:		THIS_MODULE,
		attach:		i_ADDI_Attach,
		detach:		i_ADDI_Detach, 
		num_names:	n_boardtypes,
		board_name:	boardtypes,
		offset:		sizeof(boardtype),
	};


//This macro is defined in comedidev.h
/*	#define COMEDI_INITCLEANUP(x)						\
	int init_module(void){return comedi_driver_register(&(x));}	\
	void cleanup_module(void){comedi_driver_unregister(&(x));}
*/

COMEDI_INITCLEANUP(driver_addi);



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


static int i_ADDI_Attach(comedi_device *dev,comedi_devconfig *it)
{
	comedi_subdevice *s;
	int ret,pages,i,n_subdevices;
        DWORD dw_Dummy; 
	unsigned long io_addr[5];
	unsigned short master,irq;//v_58
        unsigned long iobase_a,iobase_main,iobase_addon,iobase_reserved;
	struct pcilst_struct *card=NULL;
	unsigned char pci_bus,pci_slot,pci_func;
	int i_Dma = 0;
	static char c_Identifier [150];
	
	sprintf (c_Identifier, "Addi-Data GmbH Comedi %s", this_board->pc_DriverName);
 
	if (!pci_list_builded) 
	    {
	    v_pci_card_list_init(this_board->i_VendorId,1); //1 for displaying the list..
	    pci_list_builded=1;
	    }
	    
 	//rt_printk("comedi%d: addi_common: board=%s",dev->minor,this_board->pc_DriverName);
	
	if ((this_board->i_Dma) && (it->options[2] == 0))
	   {
	   i_Dma = 1;
	   }
	
	if ((card=ptr_select_and_alloc_pci_card(this_board->i_VendorId, 
	                                        this_board->i_DeviceId,
	                                        it->options[0],
	                                        it->options[1],
	                                        i_Dma))==NULL) 
	   {
	   return -EIO;
	   }
	   
	if ((i_pci_card_data(card,&pci_bus,&pci_slot,&pci_func,&io_addr[0],&irq,&master))<0) 
	   {
	   i_pci_card_free(card);
	   printk(" - Can't get AMCC data!\n");
	   return -EIO;
	   }
	   
	iobase_a=io_addr[0];
	iobase_main=io_addr[1];
	iobase_addon=io_addr[2];
        iobase_reserved=io_addr[3];
	printk("\nBus %d: Slot %d: Funct%d\nBase0: 0x%8lx\nBase1: 0x%8lx\nBase2: 0x%8lx\nBase3: 0x%8lx\n",
	        pci_bus,pci_slot,pci_func,io_addr[0],io_addr[1],io_addr[2],io_addr[3]);
	
	if ((this_board->pc_EepromChip == NULL) || (strcmp (this_board->pc_EepromChip, ADDIDATA_9054) != 0))
	   {
	   /************************************/
	   /* Test if more that 1 address used */
	   /************************************/
	
	   if (this_board->i_IorangeBase1 != 0)
	      {
	      dev->iobase=iobase_main;// DAQ base address...
	      printk("\nrequest_region i_IorangeBase1 - 1\n");
	      request_region(dev->iobase,  this_board->i_IorangeBase1, c_Identifier);
	      printk("\nrequest_region i_IorangeBase1 - 1 OK\n");
	      }
	   else
	      {
	      dev->iobase=iobase_a;// DAQ base address...
	      printk("\nrequest_region i_IorangeBase0 - 2");
	      request_region(dev->iobase,  this_board->i_IorangeBase0, c_Identifier);
	      printk("\nrequest_region i_IorangeBase0 - 2 %lX OK", dev->iobase);
	      }

	   dev->board_name =this_board->pc_DriverName;
	   if((ret=alloc_private(dev,sizeof(addi_private)))<0)
	      {
	      return -ENOMEM;
	      }
	   devpriv->amcc=card;
	   devpriv->master=master; //testing 
           devpriv->iobase=dev->iobase;
	   devpriv->i_IobaseAmcc=iobase_a;//AMCC base address...
	   devpriv->i_IobaseAddon=iobase_addon;//ADD ON base address....
           devpriv->i_IobaseReserved=iobase_reserved; 
	   devpriv->ps_BoardInfo = this_board;

           //if(this_board->i_Dma)
	   if((iobase_a) && (iobase_a != dev->iobase))
	      {
	      request_region(devpriv->i_IobaseAmcc,this_board->i_IorangeBase0, c_Identifier);
	      printk("\nrequest_region i_IorangeBase0 - 3 OK\n");
	      }
        
           //##
	   if(io_addr[2])
	      {
	      printk("request_region i_IorangeBase2\n");
	      request_region(io_addr[2],this_board->i_IorangeBase2, c_Identifier);
	      printk("request_region i_IorangeBase2 OK\n");
	      }
	   }
	else
	   {
	   if((ret=alloc_private(dev,sizeof(addi_private)))<0)
	      {
	      return -ENOMEM;
	      }
	   
	   if (pci_request_regions (card->pcidev, c_Identifier))
	      {
	      printk("\nRequest regions error\n");
	      return -EIO;
	      }
	      
	   dev->board_name =this_board->pc_DriverName;
	   dev->iobase=io_addr[2];
	   devpriv->amcc=card;
           devpriv->iobase=io_addr[2];
	   devpriv->ps_BoardInfo = this_board;
	   devpriv->i_IobaseReserved=io_addr[3];
	   printk ("\nioremap begin");
	   devpriv->dw_AiBase=(UINT) ioremap(io_addr[3],this_board->i_IorangeBase3);
	   printk ("\nioremap end");
	   }

        //##

	if (irq>0)  
	   {
	   if (comedi_request_irq(irq, v_ADDI_Interrupt, IRQF_SHARED, c_Identifier, dev) < 0) 
	      {
	      printk(", unable to allocate IRQ %d, DISABLING IT", irq);
	      irq=0; /* Can't use IRQ */
	      } 
	   else 
	      {
	      rt_printk("\nirq=%d", irq);
	      enable_irq (irq);
	      }    
	   } 
	else 
	   {
	   rt_printk(", IRQ disabled");
	   }
	   
	printk("\nOption %d %d %d\n",it->options[0],it->options[1],it->options[2]); 
	dev->irq = irq;
        
	// Read eepeom and fill boardtype Structure
        
	if(this_board->i_PCIEeprom)
	   { 
	   printk("\nPCI Eeprom used"); 
	   if (!(strcmp(this_board->pc_EepromChip, "S5920")))
	      {
	      // Set 3 wait stait 
	      if(!(strcmp(this_board->pc_DriverName,"apci035")))
	         {
	         outl(0x80808082,devpriv->i_IobaseAmcc+0x60); 
	         }
	      else
	         { 
	         outl(0x83838383,devpriv->i_IobaseAmcc+0x60);
	         }
	      // Enable the interrupt for the controler 
	      dw_Dummy =  inl(devpriv->i_IobaseAmcc+ 0x38);
	      outl(dw_Dummy | 0x2000,devpriv->i_IobaseAmcc+0x38);
	      printk ("\nEnable the interrupt for the controler");
	      }
	   printk("\nRead Eeprom"); 
	   i_EepromReadMainHeader(io_addr[0],this_board->pc_EepromChip,dev);
	   } 
	else
	   {
	   printk("\nPCI Eeprom unused"); 
	   }
	
	if (it->options[2]>0)	
	  {
	  devpriv->us_UseDma=ADDI_DISABLE;
	  }
	else
	  {
	  devpriv->us_UseDma=ADDI_ENABLE;
	  }
	
	if(this_board->i_Dma)
	   { 
	   printk("\nDMA used"); 
	   if (devpriv->us_UseDma==ADDI_ENABLE) 
	      {
	      // alloc DMA buffers
	      devpriv->b_DmaDoubleBuffer=0;
	      for (i=0; i<2; i++) 
	         {
	         for (pages=4; pages>=0; pages--)
	            {
	            if((devpriv->ul_DmaBufferVirtual[i]=(void *)__get_free_pages(GFP_KERNEL,pages)))
	               {
	               break;
	               }
	            }
	         if (devpriv->ul_DmaBufferVirtual[i]) 
	            {
	            devpriv->ui_DmaBufferPages[i]=pages;
	            devpriv->ui_DmaBufferSize[i]=PAGE_SIZE*pages;
	            devpriv->ui_DmaBufferSamples[i]=devpriv->ui_DmaBufferSize[i]>>1;
	            devpriv->ul_DmaBufferHw[i]=virt_to_bus((void*)devpriv->ul_DmaBufferVirtual[i]);
	            }
	         }
	      if (!devpriv->ul_DmaBufferVirtual[0]) 
	         {
	         rt_printk(", Can't allocate DMA buffer, DMA disabled!");
	         master=0;
	         devpriv->us_UseDma=ADDI_DISABLE;
	         }

	      if (devpriv->ul_DmaBufferVirtual[1])
	         {
	         devpriv->b_DmaDoubleBuffer=1;
	         }
	      }
       

	   if ((devpriv->us_UseDma==ADDI_ENABLE)) 
	      {
	      rt_printk("\nDMA ENABLED\n");
	      } 
	   else 
	      {
	      printk("\nDMA DISABLED\n");
	      }
	   }

	if (!strcmp(this_board->pc_DriverName,"apci1710"))
	   {
	   i_ADDI_AttachPCI1710 (dev);
    
	   // save base address
	   devpriv->s_BoardInfos.ui_Address=io_addr[2];
	   }       
	else
	{
	//Update-0.7.57->0.7.68dev->n_subdevices = 7;
	n_subdevices = 7;
	if((ret=alloc_subdevices(dev,n_subdevices))<0)
    	return ret;

     // Allocate and Initialise AI Subdevice Structures
	s = dev->subdevices + 0;
        if((this_board->i_NbrAiChannel) || (this_board->i_NbrAiChannelDiff))
        {
	dev->read_subdev = s;
	s->type = COMEDI_SUBD_AI;
	s->subdev_flags = SDF_READABLE|SDF_RT|SDF_COMMON|SDF_GROUND|SDF_DIFF;
	if (this_board->i_NbrAiChannel)
	   s->n_chan = this_board->i_NbrAiChannel;
	else
	   s->n_chan = this_board->i_NbrAiChannelDiff;
	s->maxdata = this_board->i_AiMaxdata;
	s->len_chanlist = this_board->i_AiChannelList;
	s->range_table = this_board->pr_AiRangelist;
       
        
        
        s->insn_config=this_board->i_hwdrv_InsnConfigAnalogInput;
	s->insn_read=this_board->i_hwdrv_InsnReadAnalogInput;
        s->insn_write=this_board->i_hwdrv_InsnWriteAnalogInput;  
        s->insn_bits=this_board->i_hwdrv_InsnBitsAnalogInput;  
	s->do_cmdtest=this_board->i_hwdrv_CommandTestAnalogInput;
	s->do_cmd=this_board->i_hwdrv_CommandAnalogInput;
        s->cancel=this_board->i_hwdrv_CancelAnalogInput;
 
        }
        else
	{
		s->type=COMEDI_SUBD_UNUSED;
	}

    // Allocate and Initialise AO Subdevice Structures
	s = dev->subdevices + 1;
        if(this_board->i_NbrAoChannel)
	{
	s->type = COMEDI_SUBD_AO;
	s->subdev_flags = SDF_WRITEABLE|SDF_GROUND|SDF_COMMON|SDF_RT;
	s->n_chan = this_board->i_NbrAoChannel;
	s->maxdata = this_board->i_AoMaxdata;
	s->len_chanlist = this_board->i_NbrAoChannel;
	s->range_table = this_board->pr_AoRangelist;
        s->insn_config=this_board->i_hwdrv_InsnConfigAnalogOutput;
	s->insn_write=this_board->i_hwdrv_InsnWriteAnalogOutput;
	}
        else
        {
		s->type=COMEDI_SUBD_UNUSED;
	}
    // Allocate and Initialise DI Subdevice Structures	
	s = dev->subdevices + 2;
        if(this_board->i_NbrDiChannel)
	{
	s->type = COMEDI_SUBD_DI;
	s->subdev_flags = SDF_READABLE|SDF_RT|SDF_GROUND|SDF_COMMON;
	s->n_chan = this_board->i_NbrDiChannel;
	s->maxdata = 1;
	s->len_chanlist = this_board->i_NbrDiChannel;
    	s->range_table = &range_digital;
	s->io_bits=0;		/* all bits input */
        s->insn_config=this_board->i_hwdrv_InsnConfigDigitalInput;
        s->insn_read=this_board->i_hwdrv_InsnReadDigitalInput;
        s->insn_write=this_board->i_hwdrv_InsnWriteDigitalInput;
	s->insn_bits=this_board->i_hwdrv_InsnBitsDigitalInput;
	}
	else
	{
		s->type=COMEDI_SUBD_UNUSED;
	}
    // Allocate and Initialise DO Subdevice Structures	
	s = dev->subdevices + 3;
        if(this_board->i_NbrDoChannel)
	{
	s->type = COMEDI_SUBD_DO;
	s->subdev_flags = SDF_READABLE|SDF_WRITEABLE|SDF_RT|SDF_GROUND|SDF_COMMON;
	s->n_chan= this_board->i_NbrDoChannel; 
	s->maxdata = this_board->i_DoMaxdata;
	s->len_chanlist =this_board->i_NbrDoChannel ;
	s->range_table = &range_digital;
	s->io_bits=0xf;		/* all bits output */
        
        s->insn_config=this_board->i_hwdrv_InsnConfigDigitalOutput;//for digital output memory.. 
        s->insn_write=this_board->i_hwdrv_InsnWriteDigitalOutput;
	s->insn_bits=this_board->i_hwdrv_InsnBitsDigitalOutput;
        s->insn_read=this_board->i_hwdrv_InsnReadDigitalOutput;
  	}
        else
	{
		s->type=COMEDI_SUBD_UNUSED;
	}
  
    // Allocate and Initialise Timer Subdevice Structures		
    	s = dev->subdevices + 4;
	if(this_board->i_Timer)
	{
	s->type = COMEDI_SUBD_TIMER;
	s->subdev_flags = SDF_WRITEABLE|SDF_RT|SDF_GROUND|SDF_COMMON; 
	s->n_chan = 1; 
	s->maxdata = 0; 
	s->len_chanlist = 1;
	s->range_table = &range_digital;
        
        s->insn_write=this_board->i_hwdrv_InsnWriteTimer;
	s->insn_read=this_board->i_hwdrv_InsnReadTimer;
	s->insn_config=this_board->i_hwdrv_InsnConfigTimer;
        s->insn_bits=this_board->i_hwdrv_InsnBitsTimer;
        }
        else
	{
		s->type=COMEDI_SUBD_UNUSED;
	}

    // Allocate and Initialise TTL
        s = dev->subdevices + 5;
	if(this_board->i_NbrTTLChannel)
	{
	s->type         = COMEDI_SUBD_TTLIO;
	s->subdev_flags = SDF_WRITEABLE|SDF_READABLE|SDF_RT|SDF_GROUND|SDF_COMMON;
	s->n_chan       = this_board->i_NbrTTLChannel;
	s->maxdata      = 0;
	s->io_bits=0;		/* all bits input */
	s->len_chanlist = this_board->i_NbrTTLChannel;
    	s->range_table  = this_board->pr_TTLRangelist; // to pass arguments in range	
        s->insn_config  = this_board->i_hwdr_ConfigInitTTLIO;
	s->insn_bits    = this_board->i_hwdr_ReadTTLIOBits;
        s->insn_read    = this_board->i_hwdr_ReadTTLIOAllPortValue;
 	s->insn_write   = this_board->i_hwdr_WriteTTLIOChlOnOff;
        }
        else
	{
		s->type=COMEDI_SUBD_UNUSED;
	}
	
        /* EEPROM */
	s=dev->subdevices+6;
        if(this_board->i_PCIEeprom)
	{
	s->type=COMEDI_SUBD_MEMORY;
	s->subdev_flags=SDF_READABLE|SDF_INTERNAL;
	s->n_chan=256;
	s->maxdata=0xffff;
	s->insn_read=i_ADDIDATA_InsnReadEeprom;
	}
        else
	{
		s->type=COMEDI_SUBD_UNUSED;
	}
     }
	
	printk("\ni_ADDI_Attach end\n"); 
	i_ADDI_Reset(dev);
	devpriv->b_ValidDriver=1;
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


static int i_ADDI_Detach(comedi_device *dev)
	{
   
	if (dev->private) 
	   {
	   if (devpriv->b_ValidDriver) 
	      {
	      i_ADDI_Reset(dev);
	      }
	   
	   if ((devpriv->ps_BoardInfo->pc_EepromChip == NULL) || (strcmp (devpriv->ps_BoardInfo->pc_EepromChip, ADDIDATA_9054) != 0))
	      {
	      if(devpriv->i_IobaseAmcc)
	         {
	         printk("\nrelease_region base address 0");
	         release_region(devpriv->i_IobaseAmcc,this_board->i_IorangeBase0);
	         printk("\nrelease_region base address 0 OK");
	         }
	         
	      if(this_board->i_IorangeBase1) 
	         {
	         printk("\nrelease_region base address 1");
	         release_region(dev->iobase,this_board->i_IorangeBase1);
	         printk("\nrelease_region base address 1 OK");
	         }

	      if(this_board->i_IorangeBase2) 
	         {
	         printk("\nrelease_region base address 2");
	         release_region(devpriv->i_IobaseAddon,this_board->i_IorangeBase2); 
	         printk("\nrelease_region base address 2 OK");
	         }
	         
	      if (devpriv->allocated)
	         {          
	         i_pci_card_free(devpriv->amcc);
	         }
	   
	      if (devpriv->ul_DmaBufferVirtual[0]) 
	         {          
	         free_pages((unsigned long)devpriv->ul_DmaBufferVirtual[0],devpriv->ui_DmaBufferPages[0]);
	         }
	   
	      if (devpriv->ul_DmaBufferVirtual[1]) 
	         {          
	         free_pages((unsigned long)devpriv->ul_DmaBufferVirtual[1],devpriv->ui_DmaBufferPages[1]);
	         }
	      }
	   else
	      {
	      iounmap ((void *) devpriv->dw_AiBase);
	      
	      pci_release_regions(devpriv->amcc->pcidev);

	      if (devpriv->allocated)
	         {          
	         i_pci_card_free(devpriv->amcc);
	         }
	      }
           
	   if(dev->irq)
	      {
	      free_irq(dev->irq,dev);
	      }
       
	   if (pci_list_builded) 
	      {
	      //v_pci_card_list_cleanup(PCI_VENDOR_ID_AMCC);
	      v_pci_card_list_cleanup(this_board->i_VendorId);
	      pci_list_builded=0;
	      }
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

static int i_ADDI_Reset(comedi_device *dev)
{
	
        this_board->i_hwdrv_Reset(dev);
	return 0;
}



// Interrupt function
/*
+----------------------------------------------------------------------------+
| Function name     :                                                        |
|static void v_ADDI_Interrupt(int irq, void *d, struct pt_regs *regs)        |
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

static irqreturn_t v_ADDI_Interrupt(int irq, void *d, struct pt_regs *regs)
{
comedi_device *dev = d;
this_board->v_hwdrv_Interrupt(irq,d,regs);
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


static int i_ADDIDATA_InsnReadEeprom(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{ 
   WORD w_Data;
   WORD w_Address;
   w_Address = CR_CHAN(insn->chanspec);// address to be read as 0,1,2,3...255  
 
   w_Data=w_EepromReadWord(devpriv->i_IobaseAmcc,this_board->pc_EepromChip,0x100+(2*w_Address));
   data[0]=w_Data;
   //multiplied by 2 bcozinput will be like 0,1,2...255
   return insn->n;

}
