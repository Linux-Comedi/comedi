



#define APCI1710_TTL_INIT			0
#define APCI1710_TTL_INITDIRECTION  1


#define APCI1710_TTL_READCHANNEL	0
#define APCI1710_TTL_READPORT		1


/*
+----------------------------------------------------------------------------+
|                       TTL INISIALISATION FUNCTION                          |
+----------------------------------------------------------------------------+
*/


	INT   i_APCI1710_InsnConfigInitTTLIO(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
/*
+----------------------------------------------------------------------------+
|                       TTL INPUT FUNCTION                                   |
+----------------------------------------------------------------------------+
*/

	INT  i_APCI1710_InsnBitsReadTTLIO(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
	INT i_APCI1710_InsnReadTTLIOAllPortValue(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
/*
+----------------------------------------------------------------------------+
|                            TTL OUTPUT FUNCTIONS                            |
+----------------------------------------------------------------------------+
*/

	INT i_APCI1710_InsnWriteSetTTLIOChlOnOff(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
