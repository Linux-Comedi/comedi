/*********      Definitions for APCI-1516 card  *****/

// Card Specific information
#define APCI1516_BOARD_VENDOR_ID                 0x15B8
#define APCI1516_ADDRESS_RANGE                   8


//DIGITAL INPUT-OUTPUT DEFINE   

#define APCI1516_DIGITAL_OP                 	4 
#define APCI1516_DIGITAL_OP_RW                 	4 
#define APCI1516_DIGITAL_IP                     0  




// TIMER COUNTER WATCHDOG DEFINES  

#define ADDIDATA_WATCHDOG                          2
#define APCI1516_DIGITAL_OP_WATCHDOG               0
#define APCI1516_WATCHDOG_ENABLEDISABLE            12
#define APCI1516_WATCHDOG_RELOAD_VALUE             4
#define APCI1516_WATCHDOG_STATUS                   16


// Hardware Layer  functions for Apci1516


//Digital Input
INT i_APCI1516_ReadMoreDigitalInput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
INT i_APCI1516_Read1DigitalInput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);




//Digital Output
int i_APCI1516_ConfigDigitalOutput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
INT i_APCI1516_WriteDigitalOutput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
INT i_APCI1516_ReadDigitalOutput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data) ;


// TIMER  
// timer value is passed as u seconds
int i_APCI1516_ConfigWatchdog(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data); 
int i_APCI1516_StartStopWriteWatchdog(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data); 
int i_APCI1516_ReadWatchdog(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);


//reset
INT i_APCI1516_Reset(comedi_device *dev); 

