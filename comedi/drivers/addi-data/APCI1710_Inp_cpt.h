


   #define APCI1710_SINGLE     0
   #define APCI1710_CONTINUOUS 1


#define APCI1710_PULSEENCODER_READ		0   
#define APCI1710_PULSEENCODER_WRITE		1


INT  i_APCI1710_InsnConfigInitPulseEncoder(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);




INT i_APCI1710_InsnWriteEnableDisablePulseEncoder(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);
/*
+----------------------------------------------------------------------------+
|                       READ PULSE ENCODER FUNCTIONS                         |
+----------------------------------------------------------------------------+
*/

INT   i_APCI1710_InsnReadInterruptPulseEncoder(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

/*
+----------------------------------------------------------------------------+
|                       WRITE PULSE ENCODER FUNCTIONS                        |
+----------------------------------------------------------------------------+
*/

 INT   i_APCI1710_InsnBitsReadWritePulseEncoder(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);


