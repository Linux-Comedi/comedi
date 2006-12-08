/*
  comedi/drivers/ni_tio.c
  Support for NI general purpose counters

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

/* TODO:
need to provide init function(s) that will init hardware and
gpct_counter struct

 */

#include "ni_tio.h"

MODULE_AUTHOR("Comedi <comedi@comedi.org>");
MODULE_DESCRIPTION("Comedi support for NI general-purpose counters");
MODULE_LICENSE("GPL");

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
	Gi_Prescale_Bit = 0x1000,
	/* must be set for clocks over 40MHz, which includes synchronous counting and quadrature modes */
	Gi_Alternate_Sync_Bit = 0x2000
};

/*FIXME: these gates/sources are 660x specific.  See m-series example code for its gates/sources */
#define Gi_Source_Select_Shift 2
#define Gi_Gate_Select_Shift 7
enum Gi_Input_Select_Bits
{
	Gi_Source_Select_Mask = 0x7c,
	Gi_Source_Select_Timebase_1_Bits = 0x0 << Gi_Source_Select_Shift,	/* 20MHz */
	Gi_Source_Select_Source_Pin_i_Bits = 0x1 << Gi_Source_Select_Shift,
	Gi_Source_Select_Next_Gate_Bits = 0xa << Gi_Source_Select_Shift,
	Gi_Source_Select_Timebase_2_Bits = 0x12 << Gi_Source_Select_Shift,	/* 100KHz */
	Gi_Source_Select_Next_TC_Bits = 0x13 << Gi_Source_Select_Shift,
	Gi_Source_Select_Timebase_3_Bits = 0x1e << Gi_Source_Select_Shift,	/* 80MHz */
	Gi_Source_Select_Logic_Low_Bits = 0x1f << Gi_Source_Select_Shift,
	Gi_Gate_Select_Mask = 0xf80,
	Gi_Gate_Select_Source_Pin_Bits = 0x0 << Gi_Gate_Select_Shift,
	Gi_Gate_Select_Gate_Pin_i_Bits = 0x1 << Gi_Gate_Select_Shift,
	Gi_Gate_Select_Next_SRC_Bits = 0xa << Gi_Gate_Select_Shift,
	Gi_Gate_Select_Next_Out_Bits = 0x14 << Gi_Gate_Select_Shift,
	Gi_Gate_Select_Logic_Low_Bits = 0x1e << Gi_Gate_Select_Shift,
	Gi_Gate_Select_Load_Source_Bit = 0x1000,
	Gi_Or_Gate_Bit = 0x2000,
	Gi_Output_Polarity_Bit = 0x4000,	/* set to invert */
	Gi_Source_Polarity_Bit = 0x8000	/* set to invert */
};
static inline unsigned Gi_Source_Select_RTSI_Bits(unsigned n)
{
	BUG_ON(n > 6);
	return (0xb + n) << Gi_Source_Select_Shift;
}
static inline unsigned Gi_Source_Select_Source_Pin_Bits(unsigned n)
{
	BUG_ON(n > 7);
	return (0x2 + n) << Gi_Source_Select_Shift;
}
static inline unsigned Gi_Gate_Select_Gate_Pin_Bits(unsigned n)
{
	BUG_ON(n > 7);
	return (0x2 + n) << Gi_Gate_Select_Shift;
}
static inline unsigned Gi_Gate_Select_RTSI_Bits(unsigned n)
{
	BUG_ON(n > 6);
	return (0xb + n) << Gi_Gate_Select_Shift;
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
#define Gi_Second_Gate_Select_Shift 7
/*FIXME: these gates are 660x specific.  See m-series example code for its gates/sources*/
enum Gi_Second_Gate_Bits
{
	Gi_Second_Gate_Mode_Bit = 0x1,
	Gi_Second_Gate_Select_Mask = 0xf80,
	Gi_Second_Gate_Select_Source_Pin_i_Bits = 0x0 << Gi_Second_Gate_Select_Shift,
	Gi_Second_Gate_Select_Up_Down_Pin_i_Bits = 0x1 << Gi_Second_Gate_Select_Shift,
	Gi_Second_Gate_Select_Next_SRC_Bits = 0xa << Gi_Second_Gate_Select_Shift,
	Gi_Second_Gate_Select_Next_Out_Bits = 0x14 << Gi_Second_Gate_Select_Shift,
	Gi_Second_Gate_Select_Selected_Gate_Bits = 0x1e << Gi_Second_Gate_Select_Shift,
	Gi_Second_Gate_Select_Logic_Low_Bits = 0x1f << Gi_Second_Gate_Select_Shift,
	Gi_Second_Gate_Polarity_Bit = 0x2000
};
static inline unsigned Gi_Second_Gate_Select_Up_Down_Pin_Bits(unsigned n)
{
	BUG_ON(n > 7);
	return (0x2 + n) << 7;
}
static inline unsigned Gi_Second_Gate_Select_RTSI_Bits(unsigned n)
{
	BUG_ON(n > 6);
	return (0xb + n) << 7;
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
static inline unsigned G_Counting_Bit(unsigned counter_index)
{
	if(counter_index % 2) return G1_Counting_Bit;
	return G0_Counting_Bit;
}
static inline unsigned G_Armed_Bit(unsigned counter_index)
{
	if(counter_index % 2) return G1_Armed_Bit;
	return G0_Armed_Bit;
}

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

static lsampl_t ni_tio_counter_status(struct ni_gpct *counter)
{
	lsampl_t status = 0;
	const unsigned bits = counter->read_register(counter, NITIO_Gxx_Status_Reg(counter->counter_index));
	if(bits & G_Armed_Bit(counter->counter_index))
	{
		status |= COMEDI_COUNTER_ARMED;
		if(bits & G_Counting_Bit(counter->counter_index))
			status |= COMEDI_COUNTER_COUNTING;
	}
	return status;
}

static void ni_tio_set_sync_mode(struct ni_gpct *counter, int force_alt_sync)
{
	const unsigned counting_mode_reg = NITIO_Gi_Counting_Mode_Reg(counter->counter_index);
	static const unsigned min_normal_sync_period_ns = 25;

	if(counter->counting_mode_registers_present == 0) return;

	switch(counter->regs[counting_mode_reg] & Gi_Counting_Mode_Mask)
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
	/* It's not clear what we should do if clock_period_ns is set to zero, so we are not
	using the alt sync in that case, but allow the caller to decide by using the
	force_alt_sync parameter. */
	/* FIXME: take into account possibility of the divide-by-8 prescale*/
	if(force_alt_sync ||
		(counter->clock_period_ns && counter->clock_period_ns < min_normal_sync_period_ns))
	{
		counter->regs[counting_mode_reg] |= Gi_Alternate_Sync_Bit;
	}else
	{
		counter->regs[counting_mode_reg] &= ~Gi_Alternate_Sync_Bit;
	}
	counter->write_register(counter, counter->regs[counting_mode_reg], counting_mode_reg);
}

static int ni_tio_set_counter_mode(struct ni_gpct *counter, unsigned mode)
{
	const unsigned counting_mode_reg = NITIO_Gi_Counting_Mode_Reg(counter->counter_index);
	const unsigned mode_reg = NITIO_Gi_Mode_Reg(counter->counter_index);
	const unsigned command_reg = NITIO_Gi_Command_Reg(counter->counter_index);
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	/* these bits map directly on to the mode register */
	static const unsigned mode_reg_direct_mask = NI_GPCT_GATE_ON_BOTH_EDGES_BIT |
		NI_GPCT_EDGE_GATE_MODE_MASK | NI_GPCT_STOP_MODE_MASK |
		NI_GPCT_OUTPUT_MODE_MASK | NI_GPCT_HARDWARE_DISARM_MASK |
		NI_GPCT_LOADING_ON_TC_BIT | NI_GPCT_LOADING_ON_GATE_BIT;

	switch(mode & NI_GPCT_RELOAD_SOURCE_MASK)
	{
	case NI_GPCT_RELOAD_SOURCE_SWITCHING_BITS:
		counter->regs[mode_reg] |= Gi_Reload_Source_Switching_Bit;
		counter->regs[input_select_reg] &= ~Gi_Gate_Select_Load_Source_Bit;
		break;
	case NI_GPCT_RELOAD_SOURCE_GATE_SELECT_BITS:
		counter->regs[input_select_reg] |= Gi_Gate_Select_Load_Source_Bit;
		counter->regs[mode_reg] &= ~(Gi_Reload_Source_Switching_Bit | Gi_Gating_Mode_Mask);
		counter->regs[mode_reg] |= Gi_Level_Gating_Bits;
		break;
	default:
		break;
	}

	counter->regs[mode_reg] &= ~mode_reg_direct_mask;
	counter->regs[mode_reg] |= mode & mode_reg_direct_mask;
	counter->write_register(counter, counter->regs[mode_reg], mode_reg);

	if(counter->counting_mode_registers_present)
	{
		counter->regs[counting_mode_reg] &= ~Gi_Counting_Mode_Mask;
		counter->regs[counting_mode_reg] |= (mode >> NI_GPCT_COUNTING_MODE_SHIFT) & Gi_Counting_Mode_Mask;
		counter->regs[counting_mode_reg] &= ~Gi_Index_Phase_Mask;
		counter->regs[counting_mode_reg] |= ((mode >> NI_GPCT_INDEX_PHASE_BITSHIFT) << Gi_Index_Phase_Bitshift) & Gi_Index_Phase_Mask;
		if(mode & NI_GPCT_INDEX_ENABLE_BIT)
		{
			counter->regs[counting_mode_reg] |= Gi_Index_Mode_Bit;
		}else
		{
			counter->regs[counting_mode_reg] &= ~Gi_Index_Mode_Bit;
		}
		counter->write_register(counter, counter->regs[counting_mode_reg], counting_mode_reg);
		ni_tio_set_sync_mode(counter, 0);
	}

	counter->regs[command_reg] &= ~Gi_Up_Down_Mask;
	counter->regs[command_reg] |= ((mode >> NI_GPCT_COUNTING_DIRECTION_SHIFT) << Gi_Up_Down_Shift) & Gi_Up_Down_Mask;
	counter->write_register(counter, counter->regs[command_reg], command_reg);

	if(mode & NI_GPCT_OR_GATE_BIT)
	{
		counter->regs[input_select_reg] |= Gi_Or_Gate_Bit;
	}else
	{
		counter->regs[input_select_reg] &= ~Gi_Or_Gate_Bit;
	}
	if(mode & NI_GPCT_INVERT_OUTPUT_BIT)
	{
		counter->regs[input_select_reg] |= Gi_Output_Polarity_Bit;
	}else
	{
		counter->regs[input_select_reg] &= ~Gi_Output_Polarity_Bit;
	}
	counter->write_register(counter, counter->regs[input_select_reg], input_select_reg);

	return 0;
}

static void ni_tio_arm(struct ni_gpct *counter, int arm)
{
	unsigned bits = counter->regs[NITIO_Gi_Command_Reg(counter->counter_index)];
	if(arm)
		bits |= Gi_Arm_Bit;
	else
		bits |= Gi_Disarm_Bit;
	counter->write_register(counter, bits, NITIO_Gi_Command_Reg(counter->counter_index));
}

static int ni_tio_set_clock_src(struct ni_gpct *counter, lsampl_t clock_source, lsampl_t period_ns)
{
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	const unsigned source_bits = CR_CHAN(clock_source) << Gi_Source_Select_Shift;

/*FIXME: m-series has a couple extra gate sources using Gi_src_subselect bit */
/*FIXME: add support for prescale in counting mode register */
	if((source_bits & Gi_Source_Select_Mask) != source_bits) return -EINVAL;
	counter->regs[input_select_reg] &= ~Gi_Source_Select_Mask;
	counter->regs[input_select_reg] |= source_bits;
	if(clock_source & CR_INVERT)
		counter->regs[input_select_reg] |= Gi_Source_Polarity_Bit;
	else
		counter->regs[input_select_reg] &= ~Gi_Source_Polarity_Bit;
	counter->write_register(counter, counter->regs[input_select_reg], input_select_reg);
	counter->update_clock_period(counter, clock_source, period_ns);
	ni_tio_set_sync_mode(counter, 0);
	return 0;
}

static void ni_tio_get_clock_src(struct ni_gpct *counter, lsampl_t *clock_source, lsampl_t *period_ns)
{
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	*clock_source = (counter->regs[input_select_reg] & Gi_Source_Select_Mask) >> Gi_Source_Select_Shift;
	*period_ns = counter->clock_period_ns;
}

static int ni_tio_set_gate_src(struct ni_gpct *counter, unsigned gate_index, lsampl_t gate_source)
{
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	const unsigned mode_reg = NITIO_Gi_Mode_Reg(counter->counter_index);
	const unsigned second_gate_reg = NITIO_Gi_Second_Gate_Reg(counter->counter_index);
	unsigned gate_select_bits;
	unsigned second_gate_select_bits;

	switch(gate_index)
	{
	case 0:
		if(CR_CHAN(gate_source) == 0)
		{
			counter->regs[mode_reg] &= ~Gi_Gating_Mode_Mask;
			counter->regs[mode_reg] |= Gi_Gating_Disabled_Bits;
			counter->write_register(counter, counter->regs[mode_reg], mode_reg);
		}else
		{
			gate_select_bits = (CR_CHAN(gate_source) - 1) << Gi_Gate_Select_Shift;
			if((gate_select_bits & Gi_Gate_Select_Mask) != gate_select_bits)
				return -EINVAL;
			counter->regs[input_select_reg] &= ~Gi_Gate_Select_Mask;
			counter->regs[input_select_reg] |= gate_select_bits;
			counter->write_register(counter, counter->regs[input_select_reg], input_select_reg);
			if(gate_source & CR_INVERT)
			{
				counter->regs[mode_reg] |= Gi_Gate_Polarity_Bit;
			}else
			{
				counter->regs[mode_reg] &= ~Gi_Gate_Polarity_Bit;
			}
			counter->regs[mode_reg] &= ~Gi_Gating_Mode_Mask;
			if(gate_source & CR_EDGE)
			{
				counter->regs[mode_reg] |= Gi_Level_Gating_Bits;
			}else
			{
				counter->regs[mode_reg] |= Gi_Rising_Edge_Gating_Bits;
			}
			counter->write_register(counter, counter->regs[mode_reg], mode_reg);
		}
		break;
	case 1:
		if(counter->second_gate_registers_present == 0) return -EINVAL;
		if(CR_CHAN(gate_source) == 0)
		{
			counter->regs[second_gate_reg] &= ~Gi_Second_Gate_Mode_Bit;
		}else
		{
			counter->regs[second_gate_reg] |= Gi_Second_Gate_Mode_Bit;
			second_gate_select_bits = (CR_CHAN(gate_source) - 1) << Gi_Second_Gate_Select_Shift;
			if((second_gate_select_bits & Gi_Second_Gate_Select_Mask) != second_gate_select_bits)
			{
				return -EINVAL;
			}
			counter->regs[second_gate_reg] &= ~Gi_Second_Gate_Select_Mask;
			counter->regs[second_gate_reg] |= second_gate_select_bits;
			if(gate_source & CR_INVERT)
			{
				counter->regs[second_gate_reg] |= Gi_Second_Gate_Polarity_Bit;
			}
		}
		counter->write_register(counter, counter->regs[second_gate_reg], second_gate_reg);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int ni_tio_get_gate_src(struct ni_gpct *counter, unsigned gate_index, lsampl_t *gate_source)
{
	const unsigned input_select_reg = NITIO_Gi_Input_Select_Reg(counter->counter_index);
	const unsigned mode_reg = NITIO_Gi_Mode_Reg(counter->counter_index);
	const unsigned second_gate_reg = NITIO_Gi_Second_Gate_Reg(counter->counter_index);

	switch(gate_index)
	{
	case 0:
		if((counter->regs[mode_reg] & Gi_Gating_Mode_Mask) == Gi_Gating_Disabled_Bits)
		{
			*gate_source = 0;
		}else
		{
			*gate_source = (counter->regs[input_select_reg] & Gi_Gate_Select_Mask) >> Gi_Gate_Select_Shift;
			if(counter->regs[input_select_reg] & Gi_Gate_Polarity_Bit)
			{
				*gate_source |= CR_INVERT;
			}
			if((counter->regs[mode_reg] & Gi_Gating_Mode_Mask) != Gi_Level_Gating_Bits)
			{
				*gate_source |= CR_EDGE;
			}
		}
		break;
	case 1:
		if((counter->regs[mode_reg] & Gi_Gating_Mode_Mask) == Gi_Gating_Disabled_Bits ||
			(counter->regs[second_gate_reg] & Gi_Second_Gate_Mode_Bit) == 0)
		{
			*gate_source = 0;
		}else
		{
			*gate_source = (counter->regs[second_gate_reg] & Gi_Second_Gate_Select_Mask) >> Gi_Second_Gate_Select_Shift;
			if(counter->regs[second_gate_reg] & Gi_Second_Gate_Polarity_Bit)
			{
				*gate_source |= CR_INVERT;
			}
			/* second gate can't have edge/level mode set independently */
			if((counter->regs[mode_reg] & Gi_Gating_Mode_Mask) != Gi_Level_Gating_Bits)
			{
				*gate_source |= CR_EDGE;
			}
		}
		break;
	default:
		return -EINVAL;
		break;
	}
	return 0;
}

int ni_tio_insn_config(struct ni_gpct *counter,
	comedi_insn *insn,
	lsampl_t *data)
{
	switch(data[0])
	{
	case INSN_CONFIG_SET_COUNTER_MODE:
		return ni_tio_set_counter_mode(counter, data[1]);
		break;
	case INSN_CONFIG_ARM:
		ni_tio_arm(counter, 1);
		return 0;
		break;
	case INSN_CONFIG_DISARM:
		ni_tio_arm(counter, 0);
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
	default:
		break;
	}
	return -EINVAL;
}

int ni_tio_rinsn(struct ni_gpct *counter,
	comedi_insn *insn,
	lsampl_t *data)
{
	const unsigned command_reg = NITIO_Gi_Command_Reg(counter->counter_index);
	const unsigned channel = CR_CHAN(insn->chanspec);
	unsigned first_read;
	unsigned second_read;
	unsigned correct_read;

	if(insn->n < 1) return 0;
	switch(channel)
	{
	case 0:
		counter->regs[command_reg] &= ~Gi_Save_Trace_Bit;
		counter->write_register(counter, counter->regs[command_reg], command_reg);
		counter->write_register(counter, counter->regs[command_reg] | Gi_Save_Trace_Bit,
			command_reg);
		/* The count doesn't get latched until the next clock edge, so it is possible the count
		may change (once) while we are reading.  Since the read of the SW_Save_Reg isn't
		atomic, we need to read twice and make sure the reading hasn't changed.  If it has,
		a third read will be correct since the count value will definitely have latched by then. */
		first_read = counter->read_register(counter, NITIO_Gi_SW_Save_Reg(counter->counter_index));
		second_read = counter->read_register(counter, NITIO_Gi_SW_Save_Reg(counter->counter_index));
		if(first_read != second_read)
			correct_read = counter->read_register(counter, NITIO_Gi_SW_Save_Reg(counter->counter_index));
		else
			correct_read = first_read;
		data[0] = correct_read;
		return 0;
		break;
	case 1:
		data[0] = counter->regs[NITIO_Gi_LoadA_Reg(counter->counter_index)];
		break;
	case 2:
		data[0] = counter->regs[NITIO_Gi_LoadB_Reg(counter->counter_index)];
		break;
	};
	return 0;
}

int ni_tio_winsn(struct ni_gpct *counter,
	comedi_insn *insn,
	lsampl_t * data)
{
	const unsigned channel = CR_CHAN(insn->chanspec);
	const unsigned command_reg = NITIO_Gi_Command_Reg(counter->counter_index);
	const unsigned mode_reg = NITIO_Gi_Mode_Reg(counter->counter_index);

	if(insn->n < 1) return 0;
	switch(channel)
	{
	case 0:
		counter->regs[NITIO_Gi_LoadA_Reg(counter->counter_index)] = data[0];
		counter->write_register(counter, data[0], NITIO_Gi_LoadA_Reg(counter->counter_index));
		counter->regs[mode_reg] &= ~Gi_Load_Source_Select_Bit;
		counter->write_register(counter, counter->regs[mode_reg], mode_reg);
		counter->write_register(counter, counter->regs[command_reg] | Gi_Load_Bit, command_reg);
		break;
	case 1:
		counter->regs[NITIO_Gi_LoadA_Reg(counter->counter_index)] = data[0];
		counter->write_register(counter, data[0], NITIO_Gi_LoadA_Reg(counter->counter_index));
		break;
	case 2:
		counter->regs[NITIO_Gi_LoadB_Reg(counter->counter_index)] = data[0];
		counter->write_register(counter, data[0], NITIO_Gi_LoadB_Reg(counter->counter_index));
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
