

#define APCI1710_PCI_BUS_CLOCK 					0
#define APCI1710_FRONT_CONNECTOR_INPUT 			1
#define APCI1710_TIMER_READVALUE				0
#define APCI1710_TIMER_GETOUTPUTLEVEL			1
#define APCI1710_TIMER_GETPROGRESSSTATUS		2
#define APCI1710_TIMER_WRITEVALUE				3

#define APCI1710_TIMER_READINTERRUPT            1
#define APCI1710_TIMER_READALLTIMER             2

/*
+----------------------------------------------------------------------------+
|                       82X54 TIMER INISIALISATION FUNCTION                  |
+----------------------------------------------------------------------------+
*/



INT i_APCI1710_InsnConfigInitTimer(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);



INT i_APCI1710_InsnWriteEnableDisableTimer(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
/*
+----------------------------------------------------------------------------+
|                       82X54 READ FUNCTION                                  |
+----------------------------------------------------------------------------+
*/

INT i_APCI1710_InsnReadAllTimerValue(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);


INT	i_APCI1710_InsnBitsTimer(comedi_device *dev,comedi_subdevice *s,
comedi_insn *insn,lsampl_t *data);

/*
+----------------------------------------------------------------------------+
|                       82X54 READ & WRITE FUNCTION                          |
+----------------------------------------------------------------------------+
*/
INT i_APCI1710_ReadTimerValue       (comedi_device *dev,
					 BYTE     b_ModulNbr,
					 BYTE     b_TimerNbr,
					 PULONG   pul_TimerValue);

INT   i_APCI1710_GetTimerOutputLevel  (comedi_device *dev,
					 BYTE   b_ModulNbr,
					 BYTE   b_TimerNbr,
					 PBYTE  pb_OutputLevel);

INT   i_APCI1710_GetTimerProgressStatus       (comedi_device *dev,
						 BYTE   b_ModulNbr,
						 BYTE   b_TimerNbr,
						 PBYTE pb_TimerStatus);

/*
+----------------------------------------------------------------------------+
|                       82X54 WRITE FUNCTION                                 |
+----------------------------------------------------------------------------+
*/
INT   i_APCI1710_WriteTimerValue      (comedi_device *dev,
					 BYTE   b_ModulNbr,
					 BYTE   b_TimerNbr,
					 ULONG ul_WriteValue);