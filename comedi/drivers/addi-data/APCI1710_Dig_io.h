


#define		APCI1710_ON				1  			// Digital  Output ON or OFF
#define		APCI1710_OFF			0


#define		APCI1710_INPUT			0   		// Digital I/O
#define		APCI1710_OUTPUT			1

#define         APCI1710_DIGIO_MEMORYONOFF  0x10  	// 	
#define         APCI1710_DIGIO_INIT         0x11


/*
+----------------------------------------------------------------------------+
|                       DIGITAL I/O INISIALISATION FUNCTION                  |
+----------------------------------------------------------------------------+
*/


	INT i_APCI1710_InsnConfigDigitalIO(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

/*
+----------------------------------------------------------------------------+
|                            INPUT OUTPUT  FUNCTIONS                         |
+----------------------------------------------------------------------------+
*/
INT i_APCI1710_InsnReadDigitalIOChlValue(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

INT i_APCI1710_InsnWriteDigitalIOChlOnOff(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

INT i_APCI1710_InsnBitsDigitalIOPortOnOff(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);

