


   #define APCI1710_30MHZ           30
   #define APCI1710_33MHZ           33
   #define APCI1710_40MHZ           40



   #define APCI1710_GATE_INPUT 10


#define APCI1710_TOR_SIMPLE_MODE    2
#define APCI1710_TOR_DOUBLE_MODE    3
#define APCI1710_TOR_QUADRUPLE_MODE 4


#define APCI1710_SINGLE     0
#define APCI1710_CONTINUOUS 1


#define APCI1710_TOR_GETPROGRESSSTATUS	0
#define APCI1710_TOR_GETCOUNTERVALUE	1
#define APCI1710_TOR_READINTERRUPT      2


/*
+----------------------------------------------------------------------------+
|                       TOR_COUNTER INISIALISATION FUNCTION                  |
+----------------------------------------------------------------------------+
*/

INT   i_APCI1710_InsnConfigInitTorCounter(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);






INT  i_APCI1710_InsnWriteEnableDisableTorCounter (comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);


INT i_APCI1710_InsnReadGetTorCounterInitialisation(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
/*
+----------------------------------------------------------------------------+
|                       TOR_COUNTER READ FUNCTION                            |
+----------------------------------------------------------------------------+
*/



INT i_APCI1710_InsnBitsGetTorCounterProgressStatusAndValue(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
