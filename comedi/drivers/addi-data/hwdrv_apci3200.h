// Card Specific information
#define APCI3200_BOARD_VENDOR_ID                 0x15B8
//#define APCI3200_ADDRESS_RANGE                   264



int MODULE_NO ;
 struct 
{			 	
 INT i_Gain ;
 INT i_Polarity;
 INT i_OffsetRange;
 INT i_Coupling;
 INT i_SingleDiff;
 INT i_AutoCalibration;
 UINT ui_ReloadValue;
 UINT ui_TimeUnitReloadVal;
 INT i_Interrupt;
 INT i_ModuleSelection;
}Config_Parameters_Module1,Config_Parameters_Module2,Config_Parameters_Module3,Config_Parameters_Module4;



//ANALOG INPUT RANGE 
comedi_lrange range_apci3200_ai={ 8, {
		BIP_RANGE(10),
		BIP_RANGE(5),
		BIP_RANGE(2),
		BIP_RANGE(1),
		UNI_RANGE(10),
		UNI_RANGE(5),
		UNI_RANGE(2),
		UNI_RANGE(1)
	}
};

//Analog Input related Defines
#define APCI3200_AI_OFFSET_GAIN                  0
#define APCI3200_AI_SC_TEST                      4
#define APCI3200_AI_IRQ                          8
#define APCI3200_AI_AUTOCAL                      12
#define APCI3200_RELOAD_CONV_TIME_VAL            32
#define APCI3200_CONV_TIME_TIME_BASE             36
#define APCI3200_RELOAD_DELAY_TIME_VAL           40
#define APCI3200_DELAY_TIME_TIME_BASE            44
#define APCI3200_AI_MODULE1                      0
#define APCI3200_AI_MODULE2                      64
#define APCI3200_AI_MODULE3                      128
#define APCI3200_AI_MODULE4                      192
#define TRUE                                     1
#define FALSE                                    0
#define APCI3200_AI_EOSIRQ                       16
#define APCI3200_AI_EOS                          20
#define APCI3200_AI_CHAN_ID                      24
#define APCI3200_AI_CHAN_VAL                     28
#define ANALOG_INPUT                             0
#define TEMPERATURE                              1
#define RESISTANCE                               2

#define ENABLE_EXT_TRIG                          1
#define ENABLE_EXT_GATE                          2
#define ENABLE_EXT_TRIG_GATE                     3


#define APCI3200_MAXVOLT                         2.5
#define ADDIDATA_GREATER_THAN_TEST               0
#define ADDIDATA_LESS_THAN_TEST                  1

#define ADDIDATA_UNIPOLAR                        1
#define ADDIDATA_BIPOLAR                         2

//ADDIDATA Enable Disable
#define ADDIDATA_ENABLE                            1
#define ADDIDATA_DISABLE                           0

typedef struct
   {
   ULONG ul_NumberOfValue;
   ULONG *pul_ResistanceValue;
   ULONG *pul_TemperatureValue;
   }str_ADDIDATA_RTDStruct,*pstr_ADDIDATA_RTDStruct;


// Hardware Layer  functions for Apci3200

//AI

INT i_APCI3200_ConfigAnalogInput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
INT i_APCI3200_ReadAnalogInput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
INT i_APCI3200_InsnWriteReleaseAnalogInput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
INT i_APCI3200_InsnBits_AnalogInput_Test(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data); 
INT i_APCI3200_StopCyclicAcquisition(comedi_device *dev,comedi_subdevice *s);
INT i_APCI3200_InterruptHandleEos(comedi_device *dev);
INT i_APCI3200_CommandTestAnalogInput(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd) ;
INT i_APCI3200_CommandAnalogInput(comedi_device *dev,comedi_subdevice *s);
INT i_APCI3200_ReadDigitalInput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
//Interrupt
void v_APCI3200_Interrupt(int irq, void *d, struct pt_regs *regs) ;
int i_APCI3200_InterruptHandleEos(comedi_device *dev);
//Reset functions
INT i_APCI3200_Reset(comedi_device *dev);	


int i_APCI3200_ReadCJCCalOffset(comedi_device *dev,lsampl_t *data);
int i_APCI3200_ReadCJCValue(comedi_device *dev,lsampl_t *data);
int i_APCI3200_ReadCalibrationGainValue(comedi_device *dev,UINT *data);
int i_APCI3200_ReadCalibrationOffsetValue(comedi_device *dev,UINT *data);
int i_APCI3200_Read1AnalogInputChannel(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data);
int i_APCI3200_ReadCJCCalGain(comedi_device *dev,lsampl_t *data);
