



#define APCI1710_30MHZ           30
#define APCI1710_33MHZ           33
#define APCI1710_40MHZ           40


#define APCI1710_BINARY_MODE     0x1
#define APCI1710_GRAY_MODE       0x0

#define APCI1710_SSI_READ1VALUE				1
#define APCI1710_SSI_READALLVALUE			2


#define APCI1710_SSI_SET_CHANNELON       0
#define APCI1710_SSI_SET_CHANNELOFF		1
#define APCI1710_SSI_READ_1CHANNEL		2
#define APCI1710_SSI_READ_ALLCHANNEL   	3

/*
+----------------------------------------------------------------------------+
|                       SSI INISIALISATION FUNCTION                          |
+----------------------------------------------------------------------------+
*/
INT   i_APCI1710_InsnConfigInitSSI(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);


INT i_APCI1710_InsnReadSSIValue(comedi_device *dev,comedi_subdevice *s,
	comedi_insn *insn,lsampl_t *data);



INT	i_APCI1710_InsnBitsSSIDigitalIO(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,
lsampl_t *data);



