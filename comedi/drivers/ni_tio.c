/*
  comedi/drivers/ni_tio.c
  Support for NI general purpose counters

  Copyright (C) 2006 Frank Mori Hess <fmhess@users.sourceforge.net>

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
Driver: ni_tio.o
Description: National Instruments general purpose counters
Devices:
Author: J.P. Mellor <jpmellor@rose-hulman.edu>,
	Herman.Bruyninckx@mech.kuleuven.ac.be,
	Wim.Meeussen@mech.kuleuven.ac.be,
	Klaas.Gadeyne@mech.kuleuven.ac.be,
	Frank Mori Hess <fmhess@users.sourceforge.net>
Updated: Thu Nov 16 09:50:32 EST 2006
Status: experimental

This module is not used directly by end-users.  Rather, it
is used by other drivers (for example ni_660x and ni_pcimio)
to provide support for NI's general purpose counters.  It was
originally based on the counter code from ni_660x.c and
ni_mio_common.c.

References:
DAQ 660x Register-Level Programmer Manual  (NI 370505A-01)
DAQ 6601/6602 User Manual (NI 322137B-01)
340934b.pdf  DAQ-STC reference manual

*/

#include "ni_tio.h"
#include "mite.h"

static uint64_t ni_tio_clock_period_ps(const struct ni_gpct *counter, unsigned generic_clock_source);
static unsigned ni_tio_generic_clock_src_select(const struct ni_gpct *counter);

MODULE_AUTHOR("Comedi <comedi@comedi.org>");
MODULE_DESCRIPTION("Comedi support for NI general-purpose counters");
MODULE_LICENSE("GPL");

static inline enum ni_gpct_register NITIO_Gi_Autoincrement_Reg(unsigned counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_Autoincrement_Reg;
		break;
	case 1:
		return NITIO_G1_Autoincrement_Reg;
		break;
	case 2:
		return NITIO_G2_Autoincrement_Reg;
		break;
	case 3:
		return NITIO_G3_Autoincrement_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gi_Command_Reg(unsigned counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_Command_Reg;
		break;
	case 1:
		return NITIO_G1_Command_Reg;
		break;
	case 2:
		return NITIO_G2_Command_Reg;
		break;
	case 3:
		return NITIO_G3_Command_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gi_Counting_Mode_Reg(unsigned counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_Counting_Mode_Reg;
		break;
	case 1:
		return NITIO_G1_Counting_Mode_Reg;
		break;
	case 2:
		return NITIO_G2_Counting_Mode_Reg;
		break;
	case 3:
		return NITIO_G3_Counting_Mode_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gi_Input_Select_Reg(unsigned counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_Input_Select_Reg;
		break;
	case 1:
		return NITIO_G1_Input_Select_Reg;
		break;
	case 2:
		return NITIO_G2_Input_Select_Reg;
		break;
	case 3:
		return NITIO_G3_Input_Select_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gxx_Joint_Reset_Reg(unsigned counter_index)
{
	switch(counter_index)
	{
	case 0:
	case 1:
		return NITIO_G01_Joint_Reset_Reg;
		break;
	case 2:
	case 3:
		return NITIO_G23_Joint_Reset_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gxx_Status_Reg(unsigned counter_index)
{
	switch(counter_index)
	{
	case 0:
	case 1:
		return NITIO_G01_Status_Reg;
		break;
	case 2:
	case 3:
		return NITIO_G23_Status_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gi_LoadA_Reg(unsigned counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_LoadA_Reg;
		break;
	case 1:
		return NITIO_G1_LoadA_Reg;
		break;
	case 2:
		return NITIO_G2_LoadA_Reg;
		break;
	case 3:
		return NITIO_G3_LoadA_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gi_LoadB_Reg(unsigned counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_LoadB_Reg;
		break;
	case 1:
		return NITIO_G1_LoadB_Reg;
		break;
	case 2:
		return NITIO_G2_LoadB_Reg;
		break;
	case 3:
		return NITIO_G3_LoadB_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gi_Mode_Reg(unsigned counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_Mode_Reg;
		break;
	case 1:
		return NITIO_G1_Mode_Reg;
		break;
	case 2:
		return NITIO_G2_Mode_Reg;
		break;
	case 3:
		return NITIO_G3_Mode_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gi_SW_Save_Reg(int counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_SW_Save_Reg;
		break;
	case 1:
		return NITIO_G1_SW_Save_Reg;
		break;
	case 2:
		return NITIO_G2_SW_Save_Reg;
		break;
	case 3:
		return NITIO_G3_SW_Save_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gi_Second_Gate_Reg(int counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_Second_Gate_Reg;
		break;
	case 1:
		return NITIO_G1_Second_Gate_Reg;
		break;
	case 2:
		return NITIO_G2_Second_Gate_Reg;
		break;
	case 3:
		return NITIO_G3_Second_Gate_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gi_DMA_Config_Reg(int counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_DMA_Config_Reg;
		break;
	case 1:
		return NITIO_G1_DMA_Config_Reg;
		break;
	case 2:
		return NITIO_G2_DMA_Config_Reg;
		break;
	case 3:
		return NITIO_G3_DMA_Config_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static inline enum ni_gpct_register NITIO_Gi_ABZ_Reg(int counter_index)
{
	switch(counter_index)
	{
	case 0:
		return NITIO_G0_ABZ_Reg;
		break;
	case 1:
		return NITIO_G1_ABZ_Reg;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

enum Gi_Auto_Increment_Reg_Bits
{
	Gi_Auto_Increment_Mask = 0xff
};

#define Gi_Up_Down_Shift 5
enum Gi_Command_Reg_Bits
{
	Gi_Arm_Bit = 0x1,
	Gi_Save_Trace_Bit = 0x2,
	Gi_Load_Bit = 0x4,
	Gi_Disarm_Bit = 0x10,
	Gi_Up_Down_Mask = 0x3 << Gi_Up_Down_Shift,
	Gi_Always_Down_Bits = 0x0 << Gi_Up_Down_Shift,
	Gi_Always_Up_Bits = 0x1 << Gi_Up_Down_Shift,
	Gi_Up_Down_Hardware_IO_Bits = 0x2 << Gi_Up_Down_Shift,
	Gi_Up_Down_Hardware_Gate_Bits = 0x3  << Gi_Up_Down_Shift,
	Gi_Write_Switch_Bit = 0x80,
	Gi_Synchronize_Gate_Bit = 0x100,
	Gi_Little_Big_Endian_Bit = 0x200,
	Gi_Bank_Switch_Start_Bit = 0x400,
	Gi_Bank_Switch_Mode_Bit = 0x800,
	Gi_Bank_Switch_Enable_Bit = 0x1000,
	Gi_Arm_Copy_Bit = 0x2000,
	Gi_Save_Trace_Copy_Bit = 0x4000,
	Gi_Disarm_Copy_Bit = 0x8000
};

#define Gi_Index_Phase_Bitshift 5
#define Gi_HW_Arm_Select_Shift 8
enum Gi_Counting_Mode_Reg_Bits
{
	Gi_Counting_Mode_Mask = 0x7,
	Gi_Counting_Mode_Normal_Bits = 0x0,
	Gi_Counting_Mode_QuadratureX1_Bits = 0x1,
	Gi_Counting_Mode_QuadratureX2_Bits = 0x2,
	Gi_Counting_Mode_QuadratureX4_Bits = 0x3,
	Gi_Counting_Mode_Two_Pulse_Bits = 0x4,
	Gi_Counting_Mode_Sync_Source_Bits = 0x6,
	Gi_Index_Mode_Bit = 0x10,
	Gi_Index_Phase_Mask = 0x3 << Gi_Index_Phase_Bitshift,
	Gi_Index_Phase_LowA_LowB = 0x0 << Gi_Index_Phase_Bitshift,
	Gi_Index_Phase_LowA_HighB = 0x1 << Gi_Index_Phase_Bitshift,
	Gi_Index_Phase_HighA_LowB = 0x2 << Gi_Index_Phase_Bitshift,
	Gi_Index_Phase_HighA_HighB = 0x3 << Gi_Index_Phase_Bitshift,
	Gi_HW_Arm_Enable_Bit = 0x80, /* from m-series example code, not documented in 660x register level manual */
	Gi_660x_HW_Arm_Select_Mask = 0x7 << Gi_HW_Arm_Select_Shift,  /* from m-series example code, not documented in 660x register level manual */
	Gi_660x_Prescale_X8_Bit = 0x1000,
	Gi_M_Series_Prescale_X8_Bit = 0x2000,
	Gi_M_Series_HW_Arm_Select_Mask = 0x1f << Gi_HW_Arm_Select_Shift,
	/* must be set for clocks over 40MHz, which includes synchronous counting and quadrature modes */
	Gi_660x_Alternate_Sync_Bit = 0x2000,
	Gi_M_Series_Alternate_Sync_Bit = 0x4000,
	Gi_660x_Prescale_X2_Bit = 0x4000,  /* from m-series example code, not documented in 660x register level manual */
	Gi_M_Series_Prescale_X2_Bit = 0x8000,
};
static inline unsigned Gi_Alternate_Sync_Bit(enum ni_gpct_variant variant)
{
	switch(variant)
	{
	case ni_gpct_variant_e_series:
		return 0;
		break;
	case ni_gpct_variant_m_series:
		return Gi_M_Series_Alternate_Sync_Bit;
		break;
	case ni_gpct_variant_660x:
		return Gi_660x_Alternate_Sync_Bit;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}
static inline unsigned Gi_Prescale_X2_Bit(enum ni_gpct_variant variant)
{
	switch(variant)
	{
	case ni_gpct_variant_e_series:
		return 0;
		break;
	case ni_gpct_variant_m_series:
		return Gi_M_Series_Prescale_X2_Bit;
		break;
	case ni_gpct_variant_660x:
		return Gi_660x_Prescale_X2_Bit;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}
static inline unsigned Gi_Prescale_X8_Bit(enum ni_gpct_variant variant)
{
	switch(variant)
	{
	case ni_gpct_variant_e_series:
		return 0;
		break;
	case ni_gpct_variant_m_series:
		return Gi_M_Series_Prescale_X8_Bit;
		break;
	case ni_gpct_variant_660x:
		return Gi_660x_Prescale_X8_Bit;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}
static inline unsigned Gi_HW_Arm_Select_Mask(enum ni_gpct_variant variant)
{
	switch(variant)
	{
	case ni_gpct_variant_e_series:
		return 0;
		break;
	case ni_gpct_variant_m_series:
		return Gi_M_Series_HW_Arm_Select_Mask;
		break;
	case ni_gpct_variant_660x:
		return Gi_660x_HW_Arm_Select_Mask;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

/* clock sources for ni_660x boards, get bits with Gi_Source_Select_Bits() */
enum ni_660x_clock_source
{
	NI_660x_Timebase_1_Clock = 0x0,	/* 20MHz */
	NI_660x_Source_Pin_i_Clock = 0x1,
	NI_660x_Next_Gate_Clock = 0xa,
	NI_660x_Timebase_2_Clock = 0x12,	/* 100KHz */
	NI_660x_Next_TC_Clock = 0x13,
	NI_660x_Timebase_3_Clock = 0x1e,	/* 80MHz */
	NI_660x_Logic_Low_Clock = 0x1f,
};
static const unsigned ni_660x_max_rtsi_channel = 6;
static inline unsigned NI_660x_RTSI_Clock(unsigned n)
{
	BUG_ON(n > ni_660x_max_rtsi_channel);
	return (0xb + n);
}
static const unsigned ni_660x_max_source_pin = 7;
static inline unsigned NI_660x_Source_Pin_Clock(unsigned n)
{
	BUG_ON(n > ni_660x_max_source_pin);
	return (0x2 + n);
}

/* clock sources for ni e and m series boards, get bits with Gi_Source_Select_Bits() */
enum ni_m_series_clock_source
{
	NI_M_Series_Timebase_1_Clock = 0x0,	/* 20MHz */
	NI_M_Series_Timebase_2_Clock = 0x12,	/* 100KHz */
	NI_M_Series_Next_TC_Clock = 0x13,
	NI_M_Series_Next_Gate_Clock = 0x14,	/* when Gi_Src_SubSelect = 0 */
	NI_M_Series_PXI_Star_Trigger_Clock = 0x14,	/* when Gi_Src_SubSelect = 1 */
	NI_M_Series_PXI10_Clock = 0x1d,
	NI_M_Series_Timebase_3_Clock = 0x1e,	/* 80MHz, when Gi_Src_SubSelect = 0 */
	NI_M_Series_Analog_Trigger_Out_Clock = 0x1e,	/* when Gi_Src_SubSelect = 1 */
	NI_M_Series_Logic_Low_Clock = 0x1f,
};
static const unsigned ni_m_series_max_pfi_channel = 15;
static inline unsigned NI_M_Series_PFI_Clock(unsigned n)
{
	BUG_ON(n > ni_m_series_max_pfi_channel);
	if(n < 10) return 1 + n;
	else return 0xb + n;
}
static const unsigned ni_m_series_max_rtsi_channel = 7;
static inline unsigned NI_M_Series_RTSI_Clock(unsigned n)
{
	BUG_ON(n > ni_m_series_max_rtsi_channel);
	if(n == 7) return 0x1b;
	else return 0xb + n;
}

enum ni_660x_gate_select
{
	NI_660x_Source_Pin_i_Gate_Select = 0x0,
	NI_660x_Gate_Pin_i_Gate_Select = 0x1,
	NI_660x_Next_SRC_Gate_Select = 0xa,
	NI_660x_Next_Out_Gate_Select = 0x14,
	NI_660x_Logic_Low_Gate_Select = 0x1f,
};
static const unsigned ni_660x_max_gate_pin = 7;
static inline unsigned NI_660x_Gate_Pin_Gate_Select(unsigned n)
{
	BUG_ON(n > ni_660x_max_gate_pin);
	return 0x2 + n;
}
static inline unsigned NI_660x_RTSI_Gate_Select(unsigned n)
{
	BUG_ON(n > ni_660x_max_rtsi_channel);
	return 0xb + n;
}

enum ni_m_series_gate_select
{
	NI_M_Series_Timestamp_Mux_Gate_Select = 0x0,
	NI_M_Series_AI_START2_Gate_Select = 0x12,
	NI_M_Series_PXI_Star_Trigger_Gate_Select = 0x13,
	NI_M_Series_Next_Out_Gate_Select = 0x14,
	NI_M_Series_AI_START1_Gate_Select = 0x1c,
	NI_M_Series_Next_SRC_Gate_Select = 0x1d,
	NI_M_Series_Analog_Trigger_Out_Gate_Select = 0x1e,
	NI_M_Series_Logic_Low_Gate_Select = 0x1f,
};
static inline unsigned NI_M_Series_RTSI_Gate_Select(unsigned n)
{
	BUG_ON(n > ni_m_series_max_rtsi_channel);
	if(n == 7)
		return 0x1b;
	return 0xb + n;
}
static inline unsigned NI_M_Series_PFI_Gate_Select(unsigned n)
{
	BUG_ON(n > ni_m_series_max_pfi_channel);
	if(n < 10)
		return 1 + n;
	return 0xb + n;
}

#define Gi_Source_Select_Shift 2
#define Gi_Gate_Select_Shift 7
enum Gi_Input_Select_Bits
{
	Gi_Source_Select_Mask = 0x7c,
	Gi_Gate_Select_Mask = 0x1f << Gi_Gate_Select_Shift,
	Gi_Gate_Select_Load_Source_Bit = 0x1000,
	Gi_Or_Gate_Bit = 0x2000,
	Gi_Output_Polarity_Bit = 0x4000,	/* set to invert */
	Gi_Source_Polarity_Bit = 0x8000	/* set to invert */
};
static inline unsigned Gi_Source_Select_Bits(unsigned source)
{
	return (source << Gi_Source_Select_Shift) & Gi_Source_Select_Mask;
}
static inline unsigned Gi_Gate_Select_Bits(unsigned gate_select)
{
	return (gate_select << Gi_Gate_Select_Shift) & Gi_Gate_Select_Mask;
}

enum Gi_Mode_Bits
{
	Gi_Gating_Mode_Mask = 0x3,
	Gi_Gating_Disabled_Bits = 0x0,
	Gi_Level_Gating_Bits = 0x1,
	Gi_Rising_Edge_Gating_Bits = 0x2,
	Gi_Falling_Edge_Gating_Bits = 0x2,
	Gi_Gate_On_Both_Edges_Bit = 0x4, /* used in conjunction with rising edge gating mode */
	Gi_Trigger_Mode_for_Edge_Gate_Mask = 0x18,
	Gi_Edge_Gate_Starts_Stops_Bits = 0x0,
	Gi_Edge_Gate_Stops_Starts_Bits = 0x8,
	Gi_Edge_Gate_Starts_Bits = 0x10,
	Gi_Edge_Gate_No_Starts_or_Stops_Bits = 0x18,
	Gi_Stop_Mode_Mask = 0x60,
	Gi_Stop_on_Gate_Bits = 0x00,
	Gi_Stop_on_Gate_or_TC_Bits = 0x20,
	Gi_Stop_on_Gate_or_Second_TC_Bits = 0x40,
	Gi_Load_Source_Select_Bit = 0x80,
	Gi_Output_Mode_Mask = 0x300,
	Gi_Output_TC_Pulse_Bits = 0x100,
	Gi_Output_TC_Toggle_Bits = 0x200,
	Gi_Output_TC_or_Gate_Toggle_Bits = 0x300,
	Gi_Counting_Once_Mask = 0xc00,
	Gi_No_Hardware_Disarm_Bits = 0x000,
	Gi_Disarm_at_TC_Bits = 0x400,
	Gi_Disarm_at_Gate_Bits = 0x800,
	Gi_Disarm_at_TC_or_Gate_Bits = 0xc00,
	Gi_Loading_On_TC_Bit = 0x1000,
	Gi_Gate_Polarity_Bit = 0x2000,
	Gi_Loading_On_Gate_Bit = 0x4000,
	Gi_Reload_Source_Switching_Bit = 0x8000
};

enum ni_660x_second_gate_select
{
	NI_660x_Source_Pin_i_Second_Gate_Select = 0x0,
	NI_660x_Up_Down_Pin_i_Second_Gate_Select = 0x1,
	NI_660x_Next_SRC_Second_Gate_Select = 0xa,
	NI_660x_Next_Out_Second_Gate_Select = 0x14,
	NI_660x_Selected_Gate_Second_Gate_Select = 0x1e,
	NI_660x_Logic_Low_Second_Gate_Select = 0x1f,
};
static const unsigned ni_660x_max_up_down_pin = 7;
static inline unsigned NI_660x_Up_Down_Pin_Second_Gate_Select(unsigned n)
{
	BUG_ON(n > ni_660x_max_up_down_pin);
	return 0x2 + n;
}
static inline unsigned NI_660x_RTSI_Second_Gate_Select(unsigned n)
{
	BUG_ON(n >  ni_660x_max_rtsi_channel);
	return 0xb + n;
}

#define Gi_Second_Gate_Select_Shift 7
/*FIXME: m-series has a second gate subselect bit */
/*FIXME: m-series second gate sources are undocumented (by NI)*/
enum Gi_Second_Gate_Bits
{
	Gi_Second_Gate_Mode_Bit = 0x1,
	Gi_Second_Gate_Select_Mask = 0x1f << Gi_Second_Gate_Select_Shift,
	Gi_Second_Gate_Polarity_Bit = 0x2000,
	Gi_Second_Gate_Subselect_Bit = 0x4000,	/* m-series only */
	Gi_Source_Subselect_Bit = 0x8000	/* m-series only */
};
static inline unsigned Gi_Second_Gate_Select_Bits(unsigned second_gate_select)
{
	return (second_gate_select << Gi_Second_Gate_Select_Shift) & Gi_Second_Gate_Select_Mask;
}

enum Gxx_Status_Bits
{
	G0_Save_Bit = 0x1,
	G1_Save_Bit = 0x2,
	G0_Counting_Bit = 0x4,
	G1_Counting_Bit = 0x8,
	G0_Next_Load_Source_Bit = 0x10,
	G1_Next_Load_Source_Bit = 0x20,
	G0_Stale_Data_Bit = 0x40,
	G1_Stale_Data_Bit = 0x80,
	G0_Armed_Bit = 0x100,
	G1_Armed_Bit = 0x200,
	G0_No_Load_Between_Gates_Bit = 0x400,
	G1_No_Load_Between_Gates_Bit = 0x800,
	G0_TC_Error_Bit = 0x1000,
	G1_TC_Error_Bit = 0x2000
};
static inline unsigned Gi_Counting_Bit(unsigned counter_index)
{
	if(counter_index % 2) return G1_Counting_Bit;
	return G0_Counting_Bit;
}
static inline unsigned Gi_Armed_Bit(unsigned counter_index)
{
	if(counter_index % 2) return G1_Armed_Bit;
	return G0_Armed_Bit;
}
static inline unsigned Gi_Next_Load_Source_Bit(unsigned counter_index)
{
	if(counter_index % 2) return G1_Next_Load_Source_Bit;
	return G0_Next_Load_Source_Bit;
}

/* joint reset register bits */
static inline unsigned Gi_Reset_Bit(unsigned counter_index)
{
	return 0x1 << (2 + (counter_index % 2));
}

enum Gi_DMA_Config_Reg_Bits
{
	Gi_DMA_Enable_Bit = 0x1,
	Gi_DMA_Write_Bit = 0x2,
	Gi_DMA_Int_Bit = 0x4
};

static const lsampl_t counter_status_mask = COMEDI_COUNTER_ARMED | COMEDI_COUNTER_COUNTING;

static int __init ni_tio_init_module(void)
{
	return 0;
}
module_init(ni_tio_init_module);

static void __exit ni_tio_cleanup_module(void)
{
}
module_exit(ni_tio_cleanup_module);

struct ni_gpct_device* ni_gpct_device_construct(comedi_device *dev,
	void (*write_register)(struct ni_gpct *counter, unsigned bits, enum ni_gpct_register reg),
	unsigned (*read_register)(struct ni_gpct *counter, enum ni_gpct_register reg),
	enum ni_gpct_variant variant, unsigned num_counters)
{
	unsigned i;

	struct ni_gpct_device *counter_dev = kmalloc(sizeof(struct ni_gpct_device), GFP_KERNEL);
	if(counter_dev == NULL) return NULL;
	counter_dev->dev = dev;
	counter_dev->write_register = write_register;
	counter_dev->read_register = read_register;
	counter_dev->variant = variant;
	BUG_ON(num_counters == 0);
	counter_dev->counters = kzalloc(sizeof(struct ni_gpct) * num_counters, GFP_KERNEL);
	if(counter_dev->counters == NULL)
	{
		kfree(counter_dev);
		return NULL;
	}
	for(i = 0; i < num_counters; ++i)
	{
		counter_dev->counters[i].counter_dev = counter_dev;
	}
	counter_dev->num_counters = num_counters;
	return counter_dev;
}

void ni_gpct_device_destroy(struct ni_gpct_device *counter_dev)
{
	if(counter_dev->counters == NULL) return;
	kfree(counter_dev->counters);
	kfree(counter_dev);
}

static int ni_tio_counting_mode_registers_present(const struct ni_gpct_device *counter_dev)
{
	switch(counter_dev->variant)
	{
	case ni_gpct_variant_e_series:
		return 0;
		break;
	case ni_gpct_variant_m_series:
	case ni_gpct_variant_660x:
		return 1;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static int ni_tio_second_gate_registers_present(const struct ni_gpct_device *counter_dev)
{
	switch(counter_dev->variant)
	{
	case ni_gpct_variant_e_series:
		return 0;
		break;
	case ni_gpct_variant_m_series:
	case ni_gpct_variant_660x:
		return 1;
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static void ni_tio_reset_count_and_disarm(struct ni_gpct *counter)
{
	counter->counter_dev->write_register(counter, Gi_Reset_Bit(counter->counter_index),
		NITIO_Gxx_Joint_Reset_Reg(counter->counter_index));
}

void ni_tio_init_counter(struct ni_gpct *counter)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;

	ni_tio_reset_count_and_disarm(counter);
	/* initialize counter registers */
	counter_dev->regs[NITIO_Gi_Autoincrement_Reg(counter->counter_index)] = 0x0;
	counter_dev->write_register(counter, counter_dev->regs[NITIO_Gi_Autoincrement_Reg(counter->counter_index)],
		NITIO_Gi_Autoincrement_Reg(counter->counter_index));
	counter_dev->regs[NITIO_Gi_Command_Reg(counter->counter_index)] = Gi_Synchronize_Gate_Bit;
	counter_dev->write_register(counter, counter_dev->regs[NITIO_Gi_Command_Reg(counter->counter_index)],
		NITIO_Gi_Command_Reg(counter->counter_index));
	counter_dev->regs[NITIO_Gi_Mode_Reg(counter->counter_index)] = 0x0;
	counter_dev->write_register(counter, counter_dev->regs[NITIO_Gi_Mode_Reg(counter->counter_index)],
		NITIO_Gi_Mode_Reg(counter->counter_index));
	counter_dev->regs[NITIO_Gi_LoadA_Reg(counter->counter_index)] = 0x0;
	counter_dev->write_register(counter, counter_dev->regs[NITIO_Gi_LoadA_Reg(counter->counter_index)],
		NITIO_Gi_LoadA_Reg(counter->counter_index));
	counter_dev->regs[NITIO_Gi_LoadB_Reg(counter->counter_index)] = 0x0;
	counter_dev->write_register(counter, counter_dev->regs[NITIO_Gi_LoadB_Reg(counter->counter_index)],
		NITIO_Gi_LoadB_Reg(counter->counter_index));
	counter_dev->regs[NITIO_Gi_Input_Select_Reg(counter->counter_index)] = 0x0;
	counter_dev->write_register(counter, counter_dev->regs[NITIO_Gi_Input_Select_Reg(counter->counter_index)],
		NITIO_Gi_Input_Select_Reg(counter->counter_index));
	if(ni_tio_counting_mode_registers_present(counter_dev))
	{
		counter_dev->regs[NITIO_Gi_Counting_Mode_Reg(counter->counter_index)] = 0x0;
		counter_dev->write_register(counter, counter_dev->regs[NITIO_Gi_Counting_Mode_Reg(counter->counter_index)],
			NITIO_Gi_Counting_Mode_Reg(counter->counter_index));
	}
	if(ni_tio_second_gate_registers_present(counter_dev))
	{
		counter_dev->regs[NITIO_Gi_Second_Gate_Reg(counter->counter_index)] = 0x0;
		counter_dev->write_register(counter, counter_dev->regs[NITIO_Gi_Second_Gate_Reg(counter->counter_index)],
			NITIO_Gi_Second_Gate_Reg(counter->counter_index));
	}
}

static lsampl_t ni_tio_counter_status(struct ni_gpct *counter)
{
	lsampl_t status = 0;
	const unsigned bits = counter->counter_dev->read_register(counter, NITIO_Gxx_Status_Reg(counter->counter_index));
	if(bits & Gi_Armed_Bit(counter->counter_index))
	{
		status |= COMEDI_COUNTER_ARMED;
		if(bits & Gi_Counting_Bit(counter->counter_index))
			status |= COMEDI_COUNTER_COUNTING;
	}
	return status;
}

static void ni_tio_set_sync_mode(struct ni_gpct *counter,
	int force_alt_sync)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned counting_mode_reg = NITIO_Gi_Counting_Mode_Reg(counter->counter_index);
	static const uint64_t min_normal_sync_period_ps = 25000;
	const uint64_t clock_period_ps = ni_tio_clock_period_ps(counter,
		ni_tio_generic_clock_src_select(counter));

	if(ni_tio_counting_mode_registers_present(counter_dev) == 0) return;

	switch(counter_dev->regs[counting_mode_reg] & Gi_Counting_Mode_Mask)
	{
	case Gi_Counting_Mode_QuadratureX1_Bits:
	case Gi_Counting_Mode_QuadratureX2_Bits:
	case Gi_Counting_Mode_QuadratureX4_Bits:
	case Gi_Counting_Mode_Sync_Source_Bits:
		force_alt_sync = 1;
		break;
	default:
		break;
	}
	/* It's not clear what we should do if clock_period is unknown, so we are not
	using the alt sync bit in that case, but allow the caller to decide by using the
	force_alt_sync parameter. */
	if(force_alt_sync ||
		(clock_period_ps && clock_period_ps < min_normal_sync_period_ps))
	{
		counter_dev->regs[counting_mode_reg] |= Gi_Alternate_Sync_Bit(counter_dev->variant);
	}else
	{
		counter_dev->regs[counting_mode_reg] &= ~Gi_Alternate_Sync_Bit(counter_dev->variant);
	}
	counter_dev->write_register(counter, counter_dev->regs[counting_mode_reg], counting_mode_reg);
}

static int ni_tio_set_counter_mode(struct ni_gpct *counter, unsigned mode)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;

	const unsigned counting_mode_reg = NITIO_Gi_Counting_Mode_Reg(counter->counter_index);
	const unsigned mode_reg = NITIO_Gi_Mode_Reg(counter->counter_index);
	const unsigned command_reg = NITIO_Gi_Command_Reg(counter->counter_index);
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	/* these bits map directly on to the mode register */
	static const unsigned mode_reg_direct_mask = NI_GPCT_GATE_ON_BOTH_EDGES_BIT |
		NI_GPCT_EDGE_GATE_MODE_MASK | NI_GPCT_STOP_MODE_MASK |
		NI_GPCT_OUTPUT_MODE_MASK | NI_GPCT_HARDWARE_DISARM_MASK |
		NI_GPCT_LOADING_ON_TC_BIT | NI_GPCT_LOADING_ON_GATE_BIT |
		NI_GPCT_LOAD_B_SELECT_BIT;

	switch(mode & NI_GPCT_RELOAD_SOURCE_MASK)
	{
	case NI_GPCT_RELOAD_SOURCE_FIXED_BITS:
		counter_dev->regs[mode_reg] &= ~Gi_Reload_Source_Switching_Bit;
		counter_dev->regs[input_select_reg] &= ~Gi_Gate_Select_Load_Source_Bit;
		break;
	case NI_GPCT_RELOAD_SOURCE_SWITCHING_BITS:
		counter_dev->regs[mode_reg] |= Gi_Reload_Source_Switching_Bit;
		counter_dev->regs[input_select_reg] &= ~Gi_Gate_Select_Load_Source_Bit;
		break;
	case NI_GPCT_RELOAD_SOURCE_GATE_SELECT_BITS:
		counter_dev->regs[input_select_reg] |= Gi_Gate_Select_Load_Source_Bit;
		counter_dev->regs[mode_reg] &= ~(Gi_Reload_Source_Switching_Bit | Gi_Gating_Mode_Mask);
		counter_dev->regs[mode_reg] |= Gi_Level_Gating_Bits;
		break;
	default:
		break;
	}

	counter_dev->regs[mode_reg] &= ~mode_reg_direct_mask;
	counter_dev->regs[mode_reg] |= mode & mode_reg_direct_mask;
	counter_dev->write_register(counter, counter_dev->regs[mode_reg], mode_reg);

	if(ni_tio_counting_mode_registers_present(counter_dev))
	{
		counter_dev->regs[counting_mode_reg] &= ~Gi_Counting_Mode_Mask;
		counter_dev->regs[counting_mode_reg] |= (mode >> NI_GPCT_COUNTING_MODE_SHIFT) & Gi_Counting_Mode_Mask;
		counter_dev->regs[counting_mode_reg] &= ~Gi_Index_Phase_Mask;
		counter_dev->regs[counting_mode_reg] |= ((mode >> NI_GPCT_INDEX_PHASE_BITSHIFT) << Gi_Index_Phase_Bitshift) & Gi_Index_Phase_Mask;
		if(mode & NI_GPCT_INDEX_ENABLE_BIT)
		{
			counter_dev->regs[counting_mode_reg] |= Gi_Index_Mode_Bit;
		}else
		{
			counter_dev->regs[counting_mode_reg] &= ~Gi_Index_Mode_Bit;
		}
		counter_dev->write_register(counter, counter_dev->regs[counting_mode_reg], counting_mode_reg);
		ni_tio_set_sync_mode(counter, 0);
	}

	counter_dev->regs[command_reg] &= ~Gi_Up_Down_Mask;
	counter_dev->regs[command_reg] |= ((mode >> NI_GPCT_COUNTING_DIRECTION_SHIFT) << Gi_Up_Down_Shift) & Gi_Up_Down_Mask;
	counter_dev->write_register(counter, counter_dev->regs[command_reg], command_reg);

	if(mode & NI_GPCT_OR_GATE_BIT)
	{
		counter_dev->regs[input_select_reg] |= Gi_Or_Gate_Bit;
	}else
	{
		counter_dev->regs[input_select_reg] &= ~Gi_Or_Gate_Bit;
	}
	if(mode & NI_GPCT_INVERT_OUTPUT_BIT)
	{
		counter_dev->regs[input_select_reg] |= Gi_Output_Polarity_Bit;
	}else
	{
		counter_dev->regs[input_select_reg] &= ~Gi_Output_Polarity_Bit;
	}
	counter_dev->write_register(counter, counter_dev->regs[input_select_reg], input_select_reg);

	return 0;
}

static int ni_tio_arm(struct ni_gpct *counter,
	int arm, unsigned start_trigger)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;

	unsigned command_bits = counter_dev->regs[NITIO_Gi_Command_Reg(counter->counter_index)];
	if(arm)
	{
		switch(start_trigger)
		{
		case NI_GPCT_ARM_IMMEDIATE:
			command_bits |= Gi_Arm_Bit;
			break;
		case NI_GPCT_ARM_PAIRED_IMMEDIATE:
			command_bits |= Gi_Arm_Bit | Gi_Arm_Copy_Bit;
			break;
		default:
			break;
		}
		if(ni_tio_counting_mode_registers_present(counter_dev))
		{
			const unsigned counting_mode_reg = NITIO_Gi_Counting_Mode_Reg(counter->counter_index);
			switch(start_trigger)
			{
			case NI_GPCT_ARM_IMMEDIATE:
			case NI_GPCT_ARM_PAIRED_IMMEDIATE:
				counter_dev->regs[counting_mode_reg] &= ~Gi_HW_Arm_Enable_Bit;
				break;
			default:
				if(start_trigger & NI_GPCT_ARM_UNKNOWN)
				{
					/* pass-through the least significant bits so we can figure out what select later */
					unsigned hw_arm_select_bits;

					counter_dev->regs[counting_mode_reg] &= ~Gi_HW_Arm_Select_Mask(counter_dev->variant);
					hw_arm_select_bits = (start_trigger << Gi_HW_Arm_Select_Shift) & Gi_HW_Arm_Select_Mask(counter_dev->variant);
					counter_dev->regs[counting_mode_reg] |= Gi_HW_Arm_Enable_Bit | hw_arm_select_bits;
					counter_dev->write_register(counter, counter_dev->regs[counting_mode_reg], counting_mode_reg);
				}else
				{
					return -EINVAL;
				}
				break;
			}
		}
	}else
	{
		command_bits |= Gi_Disarm_Bit;
	}
	counter_dev->write_register(counter, command_bits, NITIO_Gi_Command_Reg(counter->counter_index));
	return 0;
}

static unsigned ni_660x_source_select_bits(lsampl_t clock_source)
{
	unsigned ni_660x_clock;
	unsigned i;
	const unsigned clock_select_bits = clock_source & NI_GPCT_CLOCK_SRC_SELECT_MASK;

	switch(clock_select_bits)
	{
	case NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Timebase_1_Clock;
		break;
	case NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Timebase_2_Clock;
		break;
	case NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Timebase_3_Clock;
		break;
	case NI_GPCT_LOGIC_LOW_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Logic_Low_Clock;
		break;
	case NI_GPCT_SOURCE_PIN_i_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Source_Pin_i_Clock;
		break;
	case NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Next_Gate_Clock;
		break;
	case NI_GPCT_NEXT_TC_CLOCK_SRC_BITS:
		ni_660x_clock = NI_660x_Next_TC_Clock;
		break;
	default:
		for(i = 0; i <= ni_660x_max_rtsi_channel; ++i)
		{
			if(clock_select_bits == NI_GPCT_RTSI_CLOCK_SRC_BITS(i))
			{
				ni_660x_clock = NI_660x_RTSI_Clock(i);
				break;
			}
		}
		if(i <= ni_660x_max_rtsi_channel) break;
		for(i = 0; i <= ni_660x_max_source_pin; ++i)
		{
			if(clock_select_bits == NI_GPCT_SOURCE_PIN_CLOCK_SRC_BITS(i))
			{
				ni_660x_clock = NI_660x_Source_Pin_Clock(i);
				break;
			}
		}
		if(i <= ni_660x_max_source_pin) break;
		ni_660x_clock = 0;
		BUG();
		break;
	}
	return Gi_Source_Select_Bits(ni_660x_clock);
}

static unsigned ni_m_series_source_select_bits(lsampl_t clock_source)
{
	unsigned ni_m_series_clock;
	unsigned i;
	const unsigned clock_select_bits = clock_source & NI_GPCT_CLOCK_SRC_SELECT_MASK;
	switch(clock_select_bits)
	{
	case NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Timebase_1_Clock;
		break;
	case NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Timebase_2_Clock;
		break;
	case NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Timebase_3_Clock;
		break;
	case NI_GPCT_LOGIC_LOW_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Logic_Low_Clock;
		break;
	case NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Next_Gate_Clock;
		break;
	case NI_GPCT_NEXT_TC_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Next_TC_Clock;
		break;
	case NI_GPCT_PXI10_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_PXI10_Clock;
		break;
	case NI_GPCT_PXI_STAR_TRIGGER_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_PXI_Star_Trigger_Clock;
		break;
	case NI_GPCT_ANALOG_TRIGGER_OUT_CLOCK_SRC_BITS:
		ni_m_series_clock = NI_M_Series_Analog_Trigger_Out_Clock;
		break;
	default:
		for(i = 0; i <= ni_m_series_max_rtsi_channel; ++i)
		{
			if(clock_select_bits == NI_GPCT_RTSI_CLOCK_SRC_BITS(i))
			{
				ni_m_series_clock = NI_M_Series_RTSI_Clock(i);
				break;
			}
		}
		if(i <= ni_m_series_max_rtsi_channel) break;
		for(i = 0; i <= ni_m_series_max_pfi_channel; ++i)
		{
			if(clock_select_bits == NI_GPCT_PFI_CLOCK_SRC_BITS(i))
			{
				ni_m_series_clock = NI_M_Series_PFI_Clock(i);
				break;
			}
		}
		if(i <= ni_m_series_max_pfi_channel) break;
		rt_printk("invalid clock source 0x%lx\n", (unsigned long)clock_source);
		BUG();
		ni_m_series_clock = 0;
		break;
	}
	return Gi_Source_Select_Bits(ni_m_series_clock);
};

static void ni_tio_set_source_subselect(struct ni_gpct *counter, lsampl_t clock_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned second_gate_reg = NITIO_Gi_Second_Gate_Reg(counter->counter_index);

	if(counter_dev->variant != ni_gpct_variant_m_series) return;
	switch(clock_source & NI_GPCT_CLOCK_SRC_SELECT_MASK)
	{
	/* Gi_Source_Subselect is zero */
	case NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS:
	case NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS:
		counter_dev->regs[second_gate_reg] &= ~Gi_Source_Subselect_Bit;
		break;
	/* Gi_Source_Subselect is one */
	case NI_GPCT_ANALOG_TRIGGER_OUT_CLOCK_SRC_BITS:
	case NI_GPCT_PXI_STAR_TRIGGER_CLOCK_SRC_BITS:
		counter_dev->regs[second_gate_reg] |= Gi_Source_Subselect_Bit;
		break;
	/* Gi_Source_Subselect doesn't matter */
	default:
		return;
		break;
	}
	counter_dev->write_register(counter, counter_dev->regs[second_gate_reg], second_gate_reg);
}

static int ni_tio_set_clock_src(struct ni_gpct *counter,
	lsampl_t clock_source, lsampl_t period_ns)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	static const uint64_t pico_per_nano = 1000;

/*FIXME: validate clock source */
	counter_dev->regs[input_select_reg] &= ~Gi_Source_Select_Mask;
	switch(counter_dev->variant)
	{
	case ni_gpct_variant_660x:
		counter_dev->regs[input_select_reg] |= ni_660x_source_select_bits(clock_source);
		break;
	case ni_gpct_variant_e_series:
	case ni_gpct_variant_m_series:
		counter_dev->regs[input_select_reg] |= ni_m_series_source_select_bits(clock_source);
		break;
	default:
		BUG();
		break;
	}
	if(clock_source & NI_GPCT_INVERT_CLOCK_SRC_BIT)
		counter_dev->regs[input_select_reg] |= Gi_Source_Polarity_Bit;
	else
		counter_dev->regs[input_select_reg] &= ~Gi_Source_Polarity_Bit;
	counter_dev->write_register(counter, counter_dev->regs[input_select_reg], input_select_reg);
	ni_tio_set_source_subselect(counter, clock_source);
	if(ni_tio_counting_mode_registers_present(counter_dev))
	{
		const unsigned counting_mode_reg = NITIO_Gi_Counting_Mode_Reg(counter->counter_index);
		const unsigned prescaling_mode = clock_source & NI_GPCT_PRESCALE_MODE_CLOCK_SRC_MASK;

		switch(prescaling_mode)
		{
		case NI_GPCT_NO_PRESCALE_CLOCK_SRC_BITS:
			counter_dev->regs[counting_mode_reg] &= ~(Gi_Prescale_X2_Bit(counter_dev->variant) | Gi_Prescale_X8_Bit(counter_dev->variant));
			break;
		case NI_GPCT_PRESCALE_X2_CLOCK_SRC_BITS:
			counter_dev->regs[counting_mode_reg] |= Gi_Prescale_X2_Bit(counter_dev->variant);
			counter_dev->regs[counting_mode_reg] &= ~Gi_Prescale_X8_Bit(counter_dev->variant);
			break;
		case NI_GPCT_PRESCALE_X8_CLOCK_SRC_BITS:
			counter_dev->regs[counting_mode_reg] |= Gi_Prescale_X8_Bit(counter_dev->variant);
			counter_dev->regs[counting_mode_reg] &= ~Gi_Prescale_X2_Bit(counter_dev->variant);
			break;
		default:
			return -EINVAL;
			break;
		}
		counter_dev->write_register(counter, counter_dev->regs[counting_mode_reg], counting_mode_reg);
	}
	counter->clock_period_ps = pico_per_nano * period_ns;
	ni_tio_set_sync_mode(counter, 0);
	return 0;
}

static unsigned ni_tio_clock_src_modifiers(const struct ni_gpct *counter)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	const unsigned counting_mode_reg = NITIO_Gi_Counting_Mode_Reg(counter->counter_index);
	unsigned bits = 0;

	if(counter_dev->regs[input_select_reg] & Gi_Source_Polarity_Bit)
		bits |= NI_GPCT_INVERT_CLOCK_SRC_BIT;
	if(counter_dev->regs[counting_mode_reg] & Gi_Prescale_X2_Bit(counter_dev->variant))
		bits |= NI_GPCT_PRESCALE_X2_CLOCK_SRC_BITS;
	if(counter_dev->regs[counting_mode_reg] & Gi_Prescale_X8_Bit(counter_dev->variant))
		bits |= NI_GPCT_PRESCALE_X8_CLOCK_SRC_BITS;
	return bits;
}

static unsigned ni_m_series_clock_src_select(const struct ni_gpct *counter)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	const unsigned second_gate_reg = NITIO_Gi_Second_Gate_Reg(counter->counter_index);
	unsigned clock_source = 0;
	unsigned i;
	const unsigned input_select = (counter_dev->regs[input_select_reg] & Gi_Source_Select_Mask) >> Gi_Source_Select_Shift;

	switch(input_select)
	{
	case NI_M_Series_Timebase_1_Clock:
		clock_source = NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_Timebase_2_Clock:
		clock_source = NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_Timebase_3_Clock:
		if(counter_dev->regs[second_gate_reg] & Gi_Source_Subselect_Bit)
			clock_source = NI_GPCT_ANALOG_TRIGGER_OUT_CLOCK_SRC_BITS;
		else
			clock_source = NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_Logic_Low_Clock:
		clock_source = NI_GPCT_LOGIC_LOW_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_Next_Gate_Clock:
		if(counter_dev->regs[second_gate_reg] & Gi_Source_Subselect_Bit)
			clock_source = NI_GPCT_PXI_STAR_TRIGGER_CLOCK_SRC_BITS;
		else
			clock_source = NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_PXI10_Clock:
		clock_source = NI_GPCT_PXI10_CLOCK_SRC_BITS;
		break;
	case NI_M_Series_Next_TC_Clock:
		clock_source = NI_GPCT_NEXT_TC_CLOCK_SRC_BITS;
		break;
	default:
		for(i = 0; i <= ni_m_series_max_rtsi_channel; ++i)
		{
			if(input_select == NI_M_Series_RTSI_Clock(i))
			{
				clock_source = NI_GPCT_RTSI_CLOCK_SRC_BITS(i);
				break;
			}
		}
		if(i <= ni_m_series_max_rtsi_channel) break;
		for(i = 0; i <= ni_m_series_max_pfi_channel; ++i)
		{
			if(input_select == NI_M_Series_PFI_Clock(i))
			{
				clock_source = NI_GPCT_PFI_CLOCK_SRC_BITS(i);
				break;
			}
		}
		if(i <= ni_m_series_max_pfi_channel) break;
		BUG();
		break;
	}
	clock_source |= ni_tio_clock_src_modifiers(counter);
	return clock_source;
}

static unsigned ni_660x_clock_src_select(const struct ni_gpct *counter)
{
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	unsigned clock_source = 0;
	unsigned i;
	const unsigned input_select = (counter->counter_dev->regs[input_select_reg] & Gi_Source_Select_Mask) >> Gi_Source_Select_Shift;

	switch(input_select)
	{
	case NI_660x_Timebase_1_Clock:
		clock_source = NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS;
		break;
	case NI_660x_Timebase_2_Clock:
		clock_source = NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS;
		break;
	case NI_660x_Timebase_3_Clock:
		clock_source = NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS;
		break;
	case NI_660x_Logic_Low_Clock:
		clock_source = NI_GPCT_LOGIC_LOW_CLOCK_SRC_BITS;
		break;
	case NI_660x_Source_Pin_i_Clock:
		clock_source = NI_GPCT_SOURCE_PIN_i_CLOCK_SRC_BITS;
		break;
	case NI_660x_Next_Gate_Clock:
		clock_source = NI_GPCT_NEXT_GATE_CLOCK_SRC_BITS;
		break;
	case NI_660x_Next_TC_Clock:
		clock_source = NI_GPCT_NEXT_TC_CLOCK_SRC_BITS;
		break;
	default:
		for(i = 0; i <= ni_660x_max_rtsi_channel; ++i)
		{
			if(input_select == NI_660x_RTSI_Clock(i))
			{
				clock_source = NI_GPCT_RTSI_CLOCK_SRC_BITS(i);
				break;
			}
		}
		if(i <= ni_660x_max_rtsi_channel) break;
		for(i = 0; i <= ni_660x_max_source_pin; ++i)
		{
			if(input_select == NI_660x_Source_Pin_Clock(i))
			{
				clock_source = NI_GPCT_SOURCE_PIN_CLOCK_SRC_BITS(i);
				break;
			}
		}
		if(i <= ni_660x_max_source_pin) break;
		BUG();
		break;
	}
	clock_source |= ni_tio_clock_src_modifiers(counter);
	return clock_source;
}

static unsigned ni_tio_generic_clock_src_select(const struct ni_gpct *counter)
{
	switch(counter->counter_dev->variant)
	{
	case ni_gpct_variant_e_series:
	case ni_gpct_variant_m_series:
		return ni_m_series_clock_src_select(counter);
		break;
	case ni_gpct_variant_660x:
		return ni_660x_clock_src_select(counter);
		break;
	default:
		BUG();
		break;
	}
	return 0;
}

static uint64_t ni_tio_clock_period_ps(const struct ni_gpct *counter, unsigned generic_clock_source)
{
	uint64_t clock_period_ps;

	switch(generic_clock_source & NI_GPCT_CLOCK_SRC_SELECT_MASK)
	{
	case NI_GPCT_TIMEBASE_1_CLOCK_SRC_BITS:
		clock_period_ps = 50000;
		break;
	case NI_GPCT_TIMEBASE_2_CLOCK_SRC_BITS:
		clock_period_ps = 10000000;
		break;
	case NI_GPCT_TIMEBASE_3_CLOCK_SRC_BITS:
		clock_period_ps = 12500;
		break;
	case NI_GPCT_PXI10_CLOCK_SRC_BITS:
		clock_period_ps = 100000;
		break;
	default:
		/* clock period is specified by user with prescaling already taken into account. */
		return counter->clock_period_ps;
		break;
	}

	switch(generic_clock_source & NI_GPCT_PRESCALE_MODE_CLOCK_SRC_MASK)
	{
	case NI_GPCT_NO_PRESCALE_CLOCK_SRC_BITS:
		break;
	case NI_GPCT_PRESCALE_X2_CLOCK_SRC_BITS:
		clock_period_ps *= 2;
		break;
	case NI_GPCT_PRESCALE_X8_CLOCK_SRC_BITS:
		clock_period_ps *= 8;
		break;
	default:
		BUG();
		break;
	}
	return clock_period_ps;
}

static void ni_tio_get_clock_src(struct ni_gpct *counter,
	lsampl_t *clock_source, lsampl_t *period_ns)
{
	static const unsigned pico_per_nano = 1000;
	uint64_t temp64;
	*clock_source = ni_tio_generic_clock_src_select(counter);
	temp64 = ni_tio_clock_period_ps(counter, *clock_source);
	do_div(temp64, pico_per_nano);
	*period_ns = temp64;
}

static void ni_tio_set_first_gate_modifiers(struct ni_gpct *counter, lsampl_t gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned mode_reg = NITIO_Gi_Mode_Reg(counter->counter_index);

	if(gate_source & CR_INVERT)
	{
		counter_dev->regs[mode_reg] |= Gi_Gate_Polarity_Bit;
	}else
	{
		counter_dev->regs[mode_reg] &= ~Gi_Gate_Polarity_Bit;
	}
	counter_dev->regs[mode_reg] &= ~Gi_Gating_Mode_Mask;
	if(gate_source & CR_EDGE)
	{
		counter_dev->regs[mode_reg] |= Gi_Rising_Edge_Gating_Bits;
	}else
	{
		counter_dev->regs[mode_reg] |= Gi_Level_Gating_Bits;
	}
	counter_dev->write_register(counter, counter_dev->regs[mode_reg], mode_reg);
}

static int ni_660x_set_first_gate(struct ni_gpct *counter, lsampl_t gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	const unsigned selected_gate = CR_CHAN(gate_source);
	/* bits of selected_gate that may be meaningful to input select register */
	const unsigned selected_gate_mask = 0x1f;
	unsigned ni_660x_gate_select;
	unsigned i;

	switch(selected_gate)
	{
	case NI_GPCT_NEXT_SOURCE_GATE_SELECT:
		ni_660x_gate_select = NI_660x_Next_SRC_Gate_Select;
		break;
	case NI_GPCT_NEXT_OUT_GATE_SELECT:
	case NI_GPCT_LOGIC_LOW_GATE_SELECT:
	case NI_GPCT_SOURCE_PIN_i_GATE_SELECT:
	case NI_GPCT_GATE_PIN_i_GATE_SELECT:
		ni_660x_gate_select = selected_gate & selected_gate_mask;
		break;
	default:
		for(i = 0; i <= ni_660x_max_rtsi_channel; ++i)
		{
			if(selected_gate == NI_GPCT_RTSI_GATE_SELECT(i))
			{
				ni_660x_gate_select = selected_gate & selected_gate_mask;
				break;
			}
		}
		if(i <= ni_660x_max_rtsi_channel) break;
		for(i = 0; i <= ni_660x_max_gate_pin; ++i)
		{
			if(selected_gate == NI_GPCT_GATE_PIN_GATE_SELECT(i))
			{
				ni_660x_gate_select = selected_gate & selected_gate_mask;
				break;
			}
		}
		if(i <= ni_660x_max_gate_pin) break;
		return -EINVAL;
		break;
	}
	counter_dev->regs[input_select_reg] &= ~Gi_Gate_Select_Mask;
	counter_dev->regs[input_select_reg] |= Gi_Gate_Select_Bits(ni_660x_gate_select);
	counter_dev->write_register(counter, counter_dev->regs[input_select_reg], input_select_reg);
	return 0;
}

static int ni_m_series_set_first_gate(struct ni_gpct *counter, lsampl_t gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	const unsigned selected_gate = CR_CHAN(gate_source);
	/* bits of selected_gate that may be meaningful to input select register */
	const unsigned selected_gate_mask = 0x1f;
	unsigned ni_m_series_gate_select;
	unsigned i;

	switch(selected_gate)
	{
	case NI_GPCT_AI_START2_GATE_SELECT:
	case NI_GPCT_PXI_STAR_TRIGGER_GATE_SELECT:
	case NI_GPCT_NEXT_OUT_GATE_SELECT:
	case NI_GPCT_AI_START1_GATE_SELECT:
	case NI_GPCT_NEXT_SOURCE_GATE_SELECT:
	case NI_GPCT_ANALOG_TRIGGER_OUT_GATE_SELECT:
	case NI_GPCT_LOGIC_LOW_GATE_SELECT:
		ni_m_series_gate_select = selected_gate & selected_gate_mask;
		break;
	default:
		for(i = 0; i <= ni_m_series_max_rtsi_channel; ++i)
		{
			if(selected_gate == NI_GPCT_RTSI_GATE_SELECT(i))
			{
				ni_m_series_gate_select = selected_gate & selected_gate_mask;
				break;
			}
		}
		if(i <= ni_m_series_max_rtsi_channel) break;
		for(i = 0; i <= ni_m_series_max_pfi_channel; ++i)
		{
			if(selected_gate == NI_GPCT_PFI_GATE_SELECT(i))
			{
				ni_m_series_gate_select = selected_gate & selected_gate_mask;
				break;
			}
		}
		if(i <= ni_m_series_max_pfi_channel) break;
		return -EINVAL;
		break;
	}
	counter_dev->regs[input_select_reg] &= ~Gi_Gate_Select_Mask;
	counter_dev->regs[input_select_reg] |= Gi_Gate_Select_Bits(ni_m_series_gate_select);
	counter_dev->write_register(counter, counter_dev->regs[input_select_reg], input_select_reg);
	return 0;
}

static int ni_660x_set_second_gate(struct ni_gpct *counter, lsampl_t gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned second_gate_reg = NITIO_Gi_Second_Gate_Reg(counter->counter_index);
	const unsigned selected_second_gate = CR_CHAN(gate_source);
	/* bits of second_gate that may be meaningful to second gate register */
	static const unsigned selected_second_gate_mask = 0x1f;
	unsigned ni_660x_second_gate_select;
	unsigned i;

	switch(selected_second_gate)
	{
	case NI_GPCT_SOURCE_PIN_i_GATE_SELECT:
	case NI_GPCT_UP_DOWN_PIN_i_GATE_SELECT:
	case NI_GPCT_SELECTED_GATE_GATE_SELECT:
	case NI_GPCT_NEXT_OUT_GATE_SELECT:
	case NI_GPCT_LOGIC_LOW_GATE_SELECT:
		ni_660x_second_gate_select = selected_second_gate & selected_second_gate_mask;
		break;
	case NI_GPCT_NEXT_SOURCE_GATE_SELECT:
		ni_660x_second_gate_select = NI_660x_Next_SRC_Second_Gate_Select;
		break;
	default:
		for(i = 0; i <= ni_660x_max_rtsi_channel; ++i)
		{
			if(selected_second_gate == NI_GPCT_RTSI_GATE_SELECT(i))
			{
				ni_660x_second_gate_select = selected_second_gate & selected_second_gate_mask;
				break;
			}
		}
		if(i <= ni_660x_max_rtsi_channel) break;
		for(i = 0; i <= ni_660x_max_up_down_pin; ++i)
		{
			if(selected_second_gate == NI_GPCT_UP_DOWN_PIN_GATE_SELECT(i))
			{
				ni_660x_second_gate_select = selected_second_gate & selected_second_gate_mask;
				break;
			}
		}
		if(i <= ni_660x_max_up_down_pin) break;
		return -EINVAL;
		break;
	};
	counter_dev->regs[second_gate_reg] |= Gi_Second_Gate_Mode_Bit;
	counter_dev->regs[second_gate_reg] &= ~Gi_Second_Gate_Select_Mask;
	counter_dev->regs[second_gate_reg] |= Gi_Second_Gate_Select_Bits(ni_660x_second_gate_select);
	counter_dev->write_register(counter, counter_dev->regs[second_gate_reg], second_gate_reg);
	return 0;
}

static int ni_m_series_set_second_gate(struct ni_gpct *counter, lsampl_t gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned second_gate_reg = NITIO_Gi_Second_Gate_Reg(counter->counter_index);
	const unsigned selected_second_gate = CR_CHAN(gate_source);
	/* bits of second_gate that may be meaningful to second gate register */
	static const unsigned selected_second_gate_mask = 0x1f;
	unsigned ni_m_series_second_gate_select;

	/* FIXME: We don't know what the m-series second gate codes are, so we'll just pass
	the bits through for now. */
	switch(selected_second_gate)
	{
	default:
		ni_m_series_second_gate_select = selected_second_gate & selected_second_gate_mask;
		break;
	};
	counter_dev->regs[second_gate_reg] |= Gi_Second_Gate_Mode_Bit;
	counter_dev->regs[second_gate_reg] &= ~Gi_Second_Gate_Select_Mask;
	counter_dev->regs[second_gate_reg] |= Gi_Second_Gate_Select_Bits(ni_m_series_second_gate_select);
	counter_dev->write_register(counter, counter_dev->regs[second_gate_reg], second_gate_reg);
	return 0;
}

static int ni_tio_set_gate_src(struct ni_gpct *counter, unsigned gate_index, lsampl_t gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned mode_reg = NITIO_Gi_Mode_Reg(counter->counter_index);
	const unsigned second_gate_reg = NITIO_Gi_Second_Gate_Reg(counter->counter_index);

	switch(gate_index)
	{
	case 0:
		if(CR_CHAN(gate_source) == NI_GPCT_DISABLED_GATE_SELECT)
		{
			counter_dev->regs[mode_reg] &= ~Gi_Gating_Mode_Mask;
			counter_dev->regs[mode_reg] |= Gi_Gating_Disabled_Bits;
			counter_dev->write_register(counter, counter_dev->regs[mode_reg], mode_reg);
			return 0;
		}
		ni_tio_set_first_gate_modifiers(counter, gate_source);
		switch(counter_dev->variant)
		{
		case ni_gpct_variant_e_series:
		case ni_gpct_variant_m_series:
			return ni_m_series_set_first_gate(counter, gate_source);
			break;
		case ni_gpct_variant_660x:
			return ni_660x_set_first_gate(counter, gate_source);
			break;
		default:
			BUG();
			break;
		}
		break;
	case 1:
		if(ni_tio_second_gate_registers_present(counter_dev) == 0) return -EINVAL;
		if(CR_CHAN(gate_source) == NI_GPCT_DISABLED_GATE_SELECT)
		{
			counter_dev->regs[second_gate_reg] &= ~Gi_Second_Gate_Mode_Bit;
			counter_dev->write_register(counter, counter_dev->regs[second_gate_reg], second_gate_reg);
			return 0;
		}
		if(gate_source & CR_INVERT)
		{
			counter_dev->regs[second_gate_reg] |= Gi_Second_Gate_Polarity_Bit;
		}else
		{
			counter_dev->regs[second_gate_reg] &= ~Gi_Second_Gate_Polarity_Bit;
		}
		switch(counter_dev->variant)
		{
		case ni_gpct_variant_m_series:
			return ni_m_series_set_second_gate(counter, gate_source);
			break;
		case ni_gpct_variant_660x:
			return ni_660x_set_second_gate(counter, gate_source);
			break;
		default:
			BUG();
			break;
		}
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

static int ni_tio_set_other_src(struct ni_gpct *counter, unsigned index, lsampl_t source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;

	if(counter_dev->variant == ni_gpct_variant_m_series)
	{
		unsigned int abz_reg, shift, mask;

		abz_reg = NITIO_Gi_ABZ_Reg(counter->counter_index);
		switch(index) {
		case NI_GPCT_SOURCE_ENCODER_A:
			shift = 10;
			break;
		case NI_GPCT_SOURCE_ENCODER_B:
			shift = 5;
			break;
		case NI_GPCT_SOURCE_ENCODER_Z:
			shift = 0;
			break;
		default:
			return -EINVAL;
			break;
		}
		mask = 0x1f << shift;
		if(source > 0x1f)
		{
			/* Disable gate */
			source = 0x1f;
		}
		counter_dev->regs[abz_reg] &= ~mask;
		counter_dev->regs[abz_reg] |= (source << shift) & mask;
		counter_dev->write_register(counter, counter_dev->regs[abz_reg], abz_reg);
//		rt_printk("%s %x %d %d\n", __FUNCTION__, counter_dev->regs[abz_reg], index, source);
		return 0;
	}
	return -EINVAL;
}

static unsigned ni_660x_first_gate_to_generic_gate_source(unsigned ni_660x_gate_select)
{
	unsigned i;

	switch(ni_660x_gate_select)
	{
	case NI_660x_Source_Pin_i_Gate_Select:
		return NI_GPCT_SOURCE_PIN_i_GATE_SELECT;
		break;
	case NI_660x_Gate_Pin_i_Gate_Select:
		return NI_GPCT_GATE_PIN_i_GATE_SELECT;
		break;
	case NI_660x_Next_SRC_Gate_Select:
		return NI_GPCT_NEXT_SOURCE_GATE_SELECT;
		break;
	case NI_660x_Next_Out_Gate_Select:
		return NI_GPCT_NEXT_OUT_GATE_SELECT;
		break;
	case NI_660x_Logic_Low_Gate_Select:
		return NI_GPCT_LOGIC_LOW_GATE_SELECT;
		break;
	default:
		for(i = 0; i <= ni_660x_max_rtsi_channel; ++i)
		{
			if(ni_660x_gate_select == NI_660x_RTSI_Gate_Select(i))
			{
				return NI_GPCT_RTSI_GATE_SELECT(i);
				break;
			}
		}
		if(i <= ni_660x_max_rtsi_channel) break;
		for(i = 0; i <= ni_660x_max_gate_pin; ++i)
		{
			if(ni_660x_gate_select == NI_660x_Gate_Pin_Gate_Select(i))
			{
				return NI_GPCT_GATE_PIN_GATE_SELECT(i);
				break;
			}
		}
		if(i <= ni_660x_max_gate_pin) break;
		BUG();
		break;
	}
	return 0;
};

static unsigned ni_m_series_first_gate_to_generic_gate_source(unsigned ni_m_series_gate_select)
{
	unsigned i;

	switch(ni_m_series_gate_select)
	{
	case NI_660x_Source_Pin_i_Gate_Select:
		return NI_GPCT_SOURCE_PIN_i_GATE_SELECT;
		break;
	case NI_660x_Gate_Pin_i_Gate_Select:
		return NI_GPCT_GATE_PIN_i_GATE_SELECT;
		break;
	case NI_660x_Next_SRC_Gate_Select:
		return NI_GPCT_NEXT_SOURCE_GATE_SELECT;
		break;
	case NI_660x_Next_Out_Gate_Select:
		return NI_GPCT_NEXT_OUT_GATE_SELECT;
		break;
	case NI_660x_Logic_Low_Gate_Select:
		return NI_GPCT_LOGIC_LOW_GATE_SELECT;
		break;
	default:
		for(i = 0; i <= ni_m_series_max_rtsi_channel; ++i)
		{
			if(ni_m_series_gate_select == NI_M_Series_RTSI_Gate_Select(i))
			{
				return NI_GPCT_RTSI_GATE_SELECT(i);
				break;
			}
		}
		if(i <= ni_m_series_max_rtsi_channel) break;
		for(i = 0; i <= ni_m_series_max_pfi_channel; ++i)
		{
			if(ni_m_series_gate_select == NI_M_Series_PFI_Gate_Select(i))
			{
				return NI_GPCT_PFI_GATE_SELECT(i);
				break;
			}
		}
		if(i <= ni_m_series_max_pfi_channel) break;
		BUG();
		break;
	}
	return 0;
};

static unsigned ni_660x_second_gate_to_generic_gate_source(unsigned ni_660x_gate_select)
{
	unsigned i;

	switch(ni_660x_gate_select)
	{
	case NI_660x_Source_Pin_i_Second_Gate_Select:
		return NI_GPCT_SOURCE_PIN_i_GATE_SELECT;
		break;
	case NI_660x_Up_Down_Pin_i_Second_Gate_Select:
		return NI_GPCT_UP_DOWN_PIN_i_GATE_SELECT;
		break;
	case NI_660x_Next_SRC_Second_Gate_Select:
		return NI_GPCT_NEXT_SOURCE_GATE_SELECT;
		break;
	case NI_660x_Next_Out_Second_Gate_Select:
		return NI_GPCT_NEXT_OUT_GATE_SELECT;
		break;
	case NI_660x_Selected_Gate_Second_Gate_Select:
		return NI_GPCT_SELECTED_GATE_GATE_SELECT;
		break;
	case NI_660x_Logic_Low_Second_Gate_Select:
		return NI_GPCT_LOGIC_LOW_GATE_SELECT;
		break;
	default:
		for(i = 0; i <= ni_660x_max_rtsi_channel; ++i)
		{
			if(ni_660x_gate_select == NI_660x_RTSI_Second_Gate_Select(i))
			{
				return NI_GPCT_RTSI_GATE_SELECT(i);
				break;
			}
		}
		if(i <= ni_660x_max_rtsi_channel) break;
		for(i = 0; i <= ni_660x_max_up_down_pin; ++i)
		{
			if(ni_660x_gate_select == NI_660x_Up_Down_Pin_Second_Gate_Select(i))
			{
				return NI_GPCT_UP_DOWN_PIN_GATE_SELECT(i);
				break;
			}
		}
		if(i <= ni_660x_max_up_down_pin) break;
		BUG();
		break;
	}
	return 0;
};

static unsigned ni_m_series_second_gate_to_generic_gate_source(unsigned ni_m_series_gate_select)
{
	/*FIXME: the second gate sources for the m series are undocumented, so we just return
	* the raw bits for now. */
	switch(ni_m_series_gate_select)
	{
	default:
		return ni_m_series_gate_select;
		break;
	}
	return 0;
};

static int ni_tio_get_gate_src(struct ni_gpct *counter, unsigned gate_index, lsampl_t *gate_source)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	const unsigned mode_reg = NITIO_Gi_Mode_Reg(counter->counter_index);
	const unsigned second_gate_reg = NITIO_Gi_Second_Gate_Reg(counter->counter_index);
	unsigned gate_select_bits;

	switch(gate_index)
	{
	case 0:
		if((counter_dev->regs[mode_reg] & Gi_Gating_Mode_Mask) == Gi_Gating_Disabled_Bits)
		{
			*gate_source = NI_GPCT_DISABLED_GATE_SELECT;
			return 0;
		}else
		{
			gate_select_bits = (counter_dev->regs[input_select_reg] & Gi_Gate_Select_Mask) >> Gi_Gate_Select_Shift;
		}
		switch(counter_dev->variant)
		{
		case ni_gpct_variant_e_series:
		case ni_gpct_variant_m_series:
			*gate_source = ni_m_series_first_gate_to_generic_gate_source(gate_select_bits);
			break;
		case ni_gpct_variant_660x:
			*gate_source = ni_660x_first_gate_to_generic_gate_source(gate_select_bits);
			break;
		default:
			BUG();
			break;
		}
		if(counter_dev->regs[input_select_reg] & Gi_Gate_Polarity_Bit)
		{
			*gate_source |= CR_INVERT;
		}
		if((counter_dev->regs[mode_reg] & Gi_Gating_Mode_Mask) != Gi_Level_Gating_Bits)
		{
			*gate_source |= CR_EDGE;
		}
		break;
	case 1:
		if((counter_dev->regs[mode_reg] & Gi_Gating_Mode_Mask) == Gi_Gating_Disabled_Bits ||
			(counter_dev->regs[second_gate_reg] & Gi_Second_Gate_Mode_Bit) == 0)
		{
			*gate_source = NI_GPCT_DISABLED_GATE_SELECT;
			return 0;
		}else
		{
			gate_select_bits = (counter_dev->regs[second_gate_reg] & Gi_Second_Gate_Select_Mask) >> Gi_Second_Gate_Select_Shift;
		}
		switch(counter_dev->variant)
		{
		case ni_gpct_variant_e_series:
		case ni_gpct_variant_m_series:
			*gate_source = ni_m_series_second_gate_to_generic_gate_source(gate_select_bits);
			break;
		case ni_gpct_variant_660x:
			*gate_source = ni_660x_second_gate_to_generic_gate_source(gate_select_bits);
			break;
		default:
			BUG();
			break;
		}
		if(counter_dev->regs[second_gate_reg] & Gi_Second_Gate_Polarity_Bit)
		{
			*gate_source |= CR_INVERT;
		}
		/* second gate can't have edge/level mode set independently */
		if((counter_dev->regs[mode_reg] & Gi_Gating_Mode_Mask) != Gi_Level_Gating_Bits)
		{
			*gate_source |= CR_EDGE;
		}
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

int ni_tio_insn_config(struct ni_gpct *counter,
	comedi_insn *insn, lsampl_t *data)
{
	switch(data[0])
	{
	case INSN_CONFIG_SET_COUNTER_MODE:
		return ni_tio_set_counter_mode(counter, data[1]);
		break;
	case INSN_CONFIG_ARM:
		return ni_tio_arm(counter, 1, data[1]);
		break;
	case INSN_CONFIG_DISARM:
		ni_tio_arm(counter, 0, 0);
		return 0;
		break;
	case INSN_CONFIG_GET_COUNTER_STATUS:
		data[1] = ni_tio_counter_status(counter);
		data[2] = counter_status_mask;
		return 0;
		break;
	case INSN_CONFIG_SET_CLOCK_SRC:
		return ni_tio_set_clock_src(counter, data[1], data[2]);
		break;
	case INSN_CONFIG_GET_CLOCK_SRC:
		ni_tio_get_clock_src(counter, &data[1], &data[2]);
		return 0;
		break;
	case INSN_CONFIG_SET_GATE_SRC:
		return ni_tio_set_gate_src(counter, data[1], data[2]);
		break;
	case INSN_CONFIG_GET_GATE_SRC:
		return ni_tio_get_gate_src(counter, data[1], &data[2]);
		break;
	case INSN_CONFIG_SET_OTHER_SRC:
		return ni_tio_set_other_src(counter, data[1], data[2]);
		break;
	case INSN_CONFIG_RESET:
		ni_tio_reset_count_and_disarm(counter);
		return 0;
		break;
	default:
		break;
	}
	return -EINVAL;
}

int ni_tio_rinsn(struct ni_gpct *counter,
	comedi_insn *insn, lsampl_t *data)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned command_reg = NITIO_Gi_Command_Reg(counter->counter_index);
	const unsigned channel = CR_CHAN(insn->chanspec);
	unsigned first_read;
	unsigned second_read;
	unsigned correct_read;

	if(insn->n < 1) return 0;
	switch(channel)
	{
	case 0:
		counter_dev->regs[command_reg] &= ~Gi_Save_Trace_Bit;
		counter_dev->write_register(counter, counter_dev->regs[command_reg], command_reg);
		counter_dev->write_register(counter, counter_dev->regs[command_reg] | Gi_Save_Trace_Bit,
			command_reg);
		/* The count doesn't get latched until the next clock edge, so it is possible the count
		may change (once) while we are reading.  Since the read of the SW_Save_Reg isn't
		atomic (apparently even when it's a 32 bit register according to 660x docs),
		we need to read twice and make sure the reading hasn't changed.  If it has,
		a third read will be correct since the count value will definitely have latched by then. */
		first_read = counter_dev->read_register(counter, NITIO_Gi_SW_Save_Reg(counter->counter_index));
		second_read = counter_dev->read_register(counter, NITIO_Gi_SW_Save_Reg(counter->counter_index));
		if(first_read != second_read)
			correct_read = counter_dev->read_register(counter, NITIO_Gi_SW_Save_Reg(counter->counter_index));
		else
			correct_read = first_read;
		data[0] = correct_read;
		return 0;
		break;
	case 1:
		data[0] = counter_dev->regs[NITIO_Gi_LoadA_Reg(counter->counter_index)];
		break;
	case 2:
		data[0] = counter_dev->regs[NITIO_Gi_LoadB_Reg(counter->counter_index)];
		break;
	};
	return 0;
}

static unsigned ni_tio_next_load_register(struct ni_gpct *counter)
{
	const unsigned bits = counter->counter_dev->read_register(counter, NITIO_Gxx_Status_Reg(counter->counter_index));

	if(bits & Gi_Next_Load_Source_Bit(counter->counter_index))
	{
		return NITIO_Gi_LoadB_Reg(counter->counter_index);
	}else
	{
		return NITIO_Gi_LoadA_Reg(counter->counter_index);
	}
}

int ni_tio_winsn(struct ni_gpct *counter,
	comedi_insn *insn,
	lsampl_t * data)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	const unsigned channel = CR_CHAN(insn->chanspec);
	const unsigned command_reg = NITIO_Gi_Command_Reg(counter->counter_index);
	unsigned load_reg;

	if(insn->n < 1) return 0;
	switch(channel)
	{
	case 0:
		/* Unsafe if counter is armed.  Should probably check status and return -EBUSY if armed. */
		/* Don't disturb load source select, just use whichever load register is already selected. */
		load_reg = ni_tio_next_load_register(counter);
		counter_dev->write_register(counter, data[0], load_reg);
		counter_dev->write_register(counter, counter_dev->regs[command_reg] | Gi_Load_Bit, command_reg);
		/* restore state of load reg to whatever the user set last set it to */
		counter_dev->write_register(counter, counter_dev->regs[load_reg], load_reg);
		break;
	case 1:
		counter_dev->regs[NITIO_Gi_LoadA_Reg(counter->counter_index)] = data[0];
		counter_dev->write_register(counter, data[0], NITIO_Gi_LoadA_Reg(counter->counter_index));
		break;
	case 2:
		counter_dev->regs[NITIO_Gi_LoadB_Reg(counter->counter_index)] = data[0];
		counter_dev->write_register(counter, data[0], NITIO_Gi_LoadB_Reg(counter->counter_index));
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

static int ni_tio_input_cmd(struct ni_gpct *counter, comedi_async *async)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	comedi_cmd *cmd = &async->cmd;

	/* write alloc the entire buffer */
	comedi_buf_write_alloc(async, async->prealloc_bufsz);

	counter->mite_chan->dir = COMEDI_INPUT;
	mite_prep_dma(counter->mite_chan, 32, 32);
	if(counter_dev->variant == ni_gpct_variant_m_series ||
		counter_dev->variant == ni_gpct_variant_660x)
	{
		const unsigned gi_dma_config_reg = NITIO_Gi_DMA_Config_Reg(counter->counter_index);

		counter_dev->regs[gi_dma_config_reg] |= Gi_DMA_Enable_Bit;
		counter_dev->regs[gi_dma_config_reg] &= ~Gi_DMA_Write_Bit;
		counter_dev->write_register(counter, counter_dev->regs[gi_dma_config_reg], gi_dma_config_reg);
	}
	/*start the MITE*/
	mite_dma_arm(counter->mite_chan);
	return ni_tio_arm(counter, 1, NI_GPCT_ARM_IMMEDIATE);
}

static int ni_tio_output_cmd(struct ni_gpct *counter, comedi_async *async)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;

	rt_printk("ni_tio: output commands not yet implemented.\n");
	return -ENOTSUPP;

	counter->mite_chan->dir = COMEDI_OUTPUT;
	mite_prep_dma(counter->mite_chan, 32, 32);
	if(counter_dev->variant == ni_gpct_variant_m_series ||
		counter_dev->variant == ni_gpct_variant_660x)
	{
		const unsigned gi_dma_config_reg = NITIO_Gi_DMA_Config_Reg(counter->counter_index);

		counter_dev->regs[gi_dma_config_reg] |= Gi_DMA_Enable_Bit | Gi_DMA_Write_Bit;
		counter_dev->write_register(counter, counter_dev->regs[gi_dma_config_reg], gi_dma_config_reg);
	}
	return ni_tio_arm(counter, 1, NI_GPCT_ARM_IMMEDIATE);
}

int ni_tio_cmd(struct ni_gpct *counter, comedi_async *async)
{
	comedi_cmd *cmd = &async->cmd;
	int retval;

	if(counter->mite_chan == NULL)
	{
		rt_printk("ni_tio: commands only supported with DMA.  Interrupt-driven commands not yet implemented.\n");
		retval = -EIO;
	}else
	{
		ni_tio_reset_count_and_disarm(counter);
		if(cmd->flags & CMDF_WRITE)
		{
			retval = ni_tio_output_cmd(counter, async);
		}else
		{
			retval = ni_tio_input_cmd(counter, async);
		}
	}
	return retval;
}

int ni_tio_cmdtest(struct ni_gpct *counter)
{
	return 0;
}

int ni_tio_cancel(struct ni_gpct *counter)
{
	struct ni_gpct_device *counter_dev = counter->counter_dev;
	if(counter->mite_chan == NULL)
	{
		return 0;
	}
	ni_tio_arm(counter, 0, 0);
	if(counter_dev->variant == ni_gpct_variant_m_series ||
		counter_dev->variant == ni_gpct_variant_660x)
	{
		const unsigned gi_dma_config_reg = NITIO_Gi_DMA_Config_Reg(counter->counter_index);

		counter_dev->regs[gi_dma_config_reg] &= ~Gi_DMA_Enable_Bit;
		counter_dev->write_register(counter, counter_dev->regs[gi_dma_config_reg], gi_dma_config_reg);
	}
	mite_dma_disarm(counter->mite_chan);
	return 0;
}

EXPORT_SYMBOL_GPL(ni_tio_rinsn);
EXPORT_SYMBOL_GPL(ni_tio_winsn);
EXPORT_SYMBOL_GPL(ni_tio_cmd);
EXPORT_SYMBOL_GPL(ni_tio_cmdtest);
EXPORT_SYMBOL_GPL(ni_tio_cancel);
EXPORT_SYMBOL_GPL(ni_tio_insn_config);
EXPORT_SYMBOL_GPL(ni_tio_init_counter);
EXPORT_SYMBOL_GPL(ni_gpct_device_construct);
EXPORT_SYMBOL_GPL(ni_gpct_device_destroy);
