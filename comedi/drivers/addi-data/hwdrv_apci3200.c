/*
  +-----------------------------------------------------------------------+
  | (C) ADDI-DATA GmbH          Dieselstraße 3       D-77833 Ottersweier  |
  +-----------------------------------------------------------------------+
  | Tel : +49 (0) 7223/9493-0     | email    : info@addi-data.com         |
  | Fax : +49 (0) 7223/9493-92    | Internet : http://www.addi-data.com   |
  +-------------------------------+---------------------------------------+
  | Project     : 13Card Linux Driver | Compiler   : GCC                  |
  | Module name : hwdrv_apci3200.c|     Version    : 2.96                 |
  +-------------------------------+---------------------------------------+
  | Author : Shitalkumar S Chavan |     Date       : 10.12.2001           |
  +-------------------------------+---------------------------------------+
  | Description :   Hardware Layer Acces For APCI-3200                    |
  +-----------------------------------------------------------------------+
  |                             UPDATES                                   |
  +----------+-----------+------------------------------------------------+
  |   Date   |   Author  |          Description of updates                |
  +----------+-----------+------------------------------------------------+
  |          |           |                                                |
  |          |           |                                                |
  |          |           |                                                |
  +----------+-----------+------------------------------------------------+
*/

/*
+----------------------------------------------------------------------------+
|                               Included files                               |
+----------------------------------------------------------------------------+
*/
#include "hwdrv_apci3200.h"
INT i_CJCAvailable=1; 
INT i_CJCPolarity=0;
INT i_CJCGain=0;
INT i_InterruptFlag=0;
INT i_ADDIDATAPolarity;
INT i_ADDIDATAGain;                  
INT   i_AutoCalibration=0;   //: auto calibration  
INT i_ADDIDATAConversionTime;
INT i_ADDIDATAConversionTimeUnit; 
INT i_ADDIDATAType;
INT i_ChannelNo;
INT i_ChannelCount=0;
INT i_ScanType;
INT i_FirstChannel;
INT i_LastChannel;
INT i_Sum=0;
INT i_Offset;
UINT ui_Channel_num=0;
static int i_Count=0;
INT i_Initialised=0;
UINT ui_InterruptChannelValue[96]; //Buffer
/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_ReadDigitalInput                       |
|			  (comedi_device *dev,comedi_subdevice *s,               | 
|                      comedi_insn *insn,lsampl_t *data)                     |
+----------------------------------------------------------------------------+
| Task              : Read  value  of the selected channel or port           |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     UINT ui_NoOfChannels    : No Of Channels To read  for Port   
                                                Channel Numberfor single channel
|                     UINT data[0]            : 0: Read single channel
                                                1: Read port value   
                           data[1]              Port number       
+----------------------------------------------------------------------------+
| Output Parameters :	--	data[0] :Read status value
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/

INT i_APCI3200_ReadDigitalInput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data) 
{ 
   UINT ui_Temp=0;
   UINT ui_NoOfChannel=0;
   ui_NoOfChannel=CR_CHAN(insn->chanspec);
   ui_Temp=data[0];
   *data=inl(devpriv->i_IobaseReserved);
   
   if  (ui_Temp==0)
      {
      *data=(*data >> ui_NoOfChannel)&0x1; 
      } //if  (ui_Temp==0)
   else
      {
      if  (ui_Temp==1)
         {
         if(data[1] < 0 || data[1] >1)
           {
           printk("\nThe port number is in error\n");
           return -EINVAL;
           }//if(data[1] < 0 || data[1] >1)
         switch( ui_NoOfChannel)
	   {

	   case 2:
                 *data=(*data >>(2*data[1]))&0x3;  
	          break;
           case 3:
	          *data=(*data & 15 ); 
                  break;
           default:
	           comedi_error(dev," chan spec wrong");
		   return -EINVAL;   // "sorry channel spec wrong "  
                 
          }//switch(ui_NoOfChannels)    
         }//if  (ui_Temp==1)
  else
     {
     printk("\nSpecified channel not supported \n");
     }//elseif  (ui_Temp==1)
  }  
return insn->n;
}

/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_ConfigDigitalOutput                     |
|			  (comedi_device *dev,comedi_subdevice *s,				 | 
|                      comedi_insn *insn,lsampl_t *data)                     |
+----------------------------------------------------------------------------+
| Task              : Configures The Digital Output Subdevice.               |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev : Driver handle                     |
|			  data[0]  :1  Memory enable
                                0  Memory Disable														 
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error			 |
|																	 |
+----------------------------------------------------------------------------+
*/
int i_APCI3200_ConfigDigitalOutput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{ 

if  ( (data[0]!=0) && (data[0]!=1) ) 
     {   
     comedi_error(dev,"Not a valid Data !!! ,Data should be 1 or 0\n");  
     return -EINVAL;
     }//if  ( (data[0]!=0) && (data[0]!=1) ) 
  if  (data[0])
     {
     devpriv->b_OutputMemoryStatus = ADDIDATA_ENABLE ; 
     }// if  (data[0])
  else
     {
     devpriv->b_OutputMemoryStatus = ADDIDATA_DISABLE;
     }//else if  (data[0])
return insn->n;
}



/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_WriteDigitalOutput                      |
|			  (comedi_device *dev,comedi_subdevice *s,				 | 
|                      comedi_insn *insn,lsampl_t *data)                     |
+----------------------------------------------------------------------------+
| Task              : writes To the digital Output Subdevice                 |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     comedi_subdevice *s     : Subdevice Pointer            |  
|                     comedi_insn *insn       : Insn Structure Pointer       |  
|                     lsampl_t *data          : Data Pointer contains        |
|                                          configuration parameters as below |
|                     data[0]             :Value to output
                      data[1]             : 0 o/p single channel
                                            1 o/p port
                      data[2]             : port no
                      data[3]             :0 set the digital o/p on
                                           1 set the digital o/p off 	
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error	     	 |
|			                                                         |
+----------------------------------------------------------------------------+
*/
INT i_APCI3200_WriteDigitalOutput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data) 
{
UINT ui_Temp=0,ui_Temp1=0;
UINT ui_NoOfChannel = CR_CHAN(insn->chanspec); // get the channel
 if(devpriv->b_OutputMemoryStatus )
   {
    ui_Temp=inl(devpriv->i_IobaseAddon);
 
   }//if(devpriv->b_OutputMemoryStatus )
 else
   {
   ui_Temp=0;
    }//if(devpriv->b_OutputMemoryStatus )
if(data[3]==0)
  { 
  if(data[1]==0)
    {
     data[0]=(data[0] << ui_NoOfChannel)|ui_Temp;
     outl(data[0],devpriv->i_IobaseAddon);  
    }//if(data[1]==0) 
  else
    {
    if(data[1]==1)
       { 
       switch( ui_NoOfChannel)
	 {

	 case 2:data[0]=(data[0] << (2 * data[2]))|ui_Temp; 
	        break;
         case 3:data[0]=(data[0]|ui_Temp);
	        break;
         }//switch(ui_NoOfChannels)
   
     outl(data[0],devpriv->i_IobaseAddon);
     }// if(data[1]==1)
   else
     {
     printk("\nSpecified channel not supported\n");
     }//else if(data[1]==1)
   }//elseif(data[1]==0)
 }//if(data[3]==0)
else
   {
    if(data[3]==1)
       {
        if(data[1]==0)
          {
          data[0]=~data[0]&0x1;
          ui_Temp1=1;
          ui_Temp1=ui_Temp1<<ui_NoOfChannel;
          ui_Temp=ui_Temp|ui_Temp1;  
          data[0]=(data[0] << ui_NoOfChannel)^0xf;
          data[0]=data[0]& ui_Temp; 
          outl(data[0],devpriv->i_IobaseAddon);  
          }//if(data[1]==0) 
        else
          {
          if(data[1]==1)
             { 
             switch( ui_NoOfChannel)
	       {

	       case 2: data[0]=~data[0]&0x3; 
                       ui_Temp1=3;
                       ui_Temp1=ui_Temp1<<2 * data[2];
                       ui_Temp=ui_Temp|ui_Temp1;   
                       data[0]=((data[0] << (2 * data[2]))^0xf)& ui_Temp; 

	               break;
   	       case 3:
		           break;

               default:
 	              comedi_error(dev," chan spec wrong");
		      return -EINVAL;   // "sorry channel spec wrong "  
 }//switch(ui_NoOfChannels)
   
            outl(data[0],devpriv->i_IobaseAddon);
            }// if(data[1]==1)
         else
           {
            printk("\nSpecified channel not supported\n");
           }//else if(data[1]==1)
         }//elseif(data[1]==0)
      }//if(data[3]==1);
   else
      {
       printk("\nSpecified functionality does not exist\n");
       return -EINVAL;
      }//if else data[3]==1)
   }//if else data[3]==0) 
return insn->n;
}

/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_ReadDigitalOutput                       |
|			  (comedi_device *dev,comedi_subdevice *s,               | 
|                      comedi_insn *insn,lsampl_t *data)                     |
+----------------------------------------------------------------------------+
| Task              : Read  value  of the selected channel or port           |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     UINT ui_NoOfChannels    : No Of Channels To read       |
|                     UINT *data              : Data Pointer to read status  |
                      data[0]                 :0 read single channel
                                               1 read port value
                      data[1]                  port no
                     
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/
INT i_APCI3200_ReadDigitalOutput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data) 
{
  UINT ui_Temp;
  UINT ui_NoOfChannel;
  ui_NoOfChannel=CR_CHAN(insn->chanspec);
  ui_Temp=data[0];
  *data=inl(devpriv->i_IobaseAddon);
   if  (ui_Temp==0)
      {
      *data=(*data >> ui_NoOfChannel)&0x1; 
      } // if  (ui_Temp==0)
   else
      {
      if  (ui_Temp==1)
         {
         if(data[1] <0 ||data[1] >1)
            {
             printk("\nThe port selection is in error\n");
             return -EINVAL;
            }//if(data[1] <0 ||data[1] >1)   
         switch (ui_NoOfChannel)
	   {
           case 2: 
                    *data=(*data >>(2*data[1]))&3; 
                    break;

           case 3:break; 
                    

           default:
	            comedi_error(dev," chan spec wrong");
		    return -EINVAL;   // "sorry channel spec wrong "  
                    break;
           } // switch(ui_NoOfChannels)    
        } // if  (ui_Temp==1)
     else
        {
        printk("\nSpecified channel not supported \n");
        } // else if (ui_Temp==1)
     } // else if  (ui_Temp==0)
   return insn->n;
}

/*
 +----------------------------------------------------------------------------+
| Function   Name   : INT i_APCI3200_ConfigAnalogInput                       |
|			  (comedi_device *dev,comedi_subdevice *s,               | 
|                      comedi_insn *insn,lsampl_t *data)                     |
+----------------------------------------------------------------------------+
| Task              : Configures The Analog Input Subdevice                  |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     comedi_subdevice *s     : Subdevice Pointer            |  
|                     comedi_insn *insn       : Insn Structure Pointer       |  
|                     lsampl_t *data          : Data Pointer contains        |
|                                          configuration parameters as below |
|                                                                            | 
|					data[0]            
|                                               0:Normal AI                  |
|                                               1:RTD                        |  
|                                               2:THERMOCOUPLE               |
|				    data[1]            : Gain To Use                 |
|                                                                            |
|                           data[2]            : Polarity                             	
|                                                0:Bipolar                   |  	
|                                                1:Unipolar                  | 	
|															    	 | 	
|                           data[3]            : Offset Range                         	
|                                                                            | 	
|                           data[4]            : Coupling                            
|                                                0:DC Coupling               |
|                                                1:AC Coupling               |
|                                                                            |
|                           data[5]            :Differential/Single                   
|                                                0:Single                    |
|                                                1:Differential              |
|                                                                            |
|                           data[6]            :TimerReloadValue                     
|                                                                            |
|                           data[7]            :ConvertingTimeUnit                   
|                                                                            |
|                           data[8]             :0 Analog voltage measurement
                                                 1 Resistance measurement
                                                 2 Temperature measurement    
|                           data[9]            :Interrupt                            
|                                              0:Disable                         
|                                              1:Enable    
                           data[10]           :Type of Thermocouple                       
|                          data[11]           : 0: single channel
                                                Module Number                    
|                                        
|                          data[12]                            	
|                                             0:Single Read                        	
|                                             1:Read more channel  
                                              2:Single scan                       	
|                                             3:Continous Scan                
                           data[13]          :Number of channels to read      	
|                          data[14]          :RTD connection type
                                             :0:RTD not used
                                              1:RTD 2 wire connection
                                              2:RTD 3 wire connection
                                              3:RTD 4 wire connection                                          
|                                                                            | 
|                                                                            | 
|                                                                            | 	
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/
INT i_APCI3200_ConfigAnalogInput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data) 
{

   UINT  ul_Config = 0,ul_Temp=0 ; 
   UINT ui_ChannelNo=0; 
   UINT ui_Dummy=0;
   INT i_err=0;		 
   if(data[0]!=0 && data[0]!=1 && data[0]!=2)
      {
       printk("\nThe selection of acquisition type is in error\n");
       i_err++;
      }//if(data[0]!=0 && data[0]!=1 && data[0]!=2)
   if(data[0]==1)
      {
       if(data[14]!=0 && data[14]!=1 && data[14]!=2 && data[14]!=4)
         {
          printk("\n Error in selection of RTD connection type\n");
          i_err++; 
         }//if(data[14]!=0 && data[14]!=1 && data[14]!=2 && data[14]!=4)
       }//if(data[0]==1 )  
    if(data[1]<0 || data[1]>7)
      {
        printk("\nThe selection of gain is in error\n");
        i_err++;
      } // if(data[1]<0 || data[1]>7)
    if(data[2]!=0 &&  data[2]!=1)
       {
        printk("\nThe selection of polarity is in error\n"); 
        i_err++;
       }//if(data[2]!=0 &&  data[2]!=1)
    if(data[3]!=0)
      {
       printk("\nThe selection of offset range  is in error\n"); 
       i_err++;
      }// if(data[3]!=0)
     if(data[4]!=0 &&  data[4]!=1)
       {
        printk("\nThe selection of coupling is in error\n"); 
        i_err++;
       }//if(data[4]!=0 &&  data[4]!=1)
      if(data[5]!=0 &&  data[5]!=1)
       {
        printk("\nThe selection of single/differential mode is in error\n"); 
        i_err++;
       }//if(data[5]!=0 &&  data[5]!=1)
     if(data[8]!=0 && data[8]!=1 && data[2]!=2)
        {
        printk("\nError in selection of functionality\n");
        }//if(data[8]!=0 && data[8]!=1 && data[2]!=2)     
    if(data[12]==0 || data[12]==1)
      {
      if (data[6]!=20 && data[6]!=40 && data[6]!=80 && data[6]!=160 )
         {
         printk("\nThe selection of conversion time reload value is in error\n");
         i_err++;
         }// if (data[6]!=20 && data[6]!=40 && data[6]!=80 && data[6]!=160 )
      if(data[7]!=2)
        {
        printk("\nThe selection of conversion time unit  is in error\n");
        i_err++;
        }// if(data[7]!=2)
       }  
      if(data[9]!=0 &&  data[9]!=1)
       {
        printk("\nThe selection of interrupt enable is in error\n"); 
        i_err++;
       }//if(data[9]!=0 &&  data[9]!=1)
      if(data[11] < 0 || data[11] > 4)
        {
        printk("\nThe selection of module is in error\n"); 
        i_err++;
       }//if(data[11] <0 ||  data[11]>1)
      if(data[12] < 0 || data[12] > 3)
        {
        printk("\nThe selection of singlechannel/scan selection is in error\n"); 
        i_err++;
        }//if(data[12] < 0 ||  data[12]> 3) 
      if(data[13] <0 ||data[13] >16)
        {
        printk("\nThe selection of number of channels is in error\n"); 
        i_err++;
        }// if(data[13] <0 ||data[13] >15)
   
          
   i_ChannelCount=data[13];
   i_ScanType=data[12];
  i_ADDIDATAPolarity = data[2];
   i_ADDIDATAGain=data[1];
   i_ADDIDATAConversionTime=data[6];
   i_ADDIDATAConversionTimeUnit=data[7];
   i_ADDIDATAType=data[0]; 
   while(i_InterruptFlag==1)
     {
      udelay(1);
     }
    
   ui_ChannelNo = CR_CHAN(insn->chanspec); // get the channel
   i_ChannelNo=ui_ChannelNo; 
   ui_Channel_num =ui_ChannelNo;
 if(data[5]==0)
   {    
   if(ui_ChannelNo<0 || ui_ChannelNo>15)
      {
      printk("\nThe Selection of the channel is in error\n");  
      i_err++;
      }// if(ui_ChannelNo<0 || ui_ChannelNo>15)
    }//if(data[5]==0)
 else
    {
     if(data[14]==2)
       {
       if(ui_ChannelNo<0 || ui_ChannelNo>3) 
         {
         printk("\nThe Selection of the channel is in error\n");  
         i_err++;
         }// if(ui_ChannelNo<0 || ui_ChannelNo>3) 
       }//if(data[14]==2)
    else
       {  
       if(ui_ChannelNo<0 || ui_ChannelNo>7)
          {
          printk("\nThe Selection of the channel is in error\n");  
          i_err++;
          }// if(ui_ChannelNo<0 || ui_ChannelNo>7)
       }//elseif(data[14]==2)
     }//elseif(data[5]==0)
   if(data[12]==0 || data[12]==1)
      {
       switch(data[5])
          {
           case 0:   
              if(ui_ChannelNo >=0 && ui_ChannelNo <=3)
                {
                i_Offset=0;
                }//if(ui_ChannelNo >=0 && ui_ChannelNo <=3) 
              if(ui_ChannelNo >=4 && ui_ChannelNo <=7)
                {
                i_Offset=64;
                }//if(ui_ChannelNo >=4 && ui_ChannelNo <=7) 
              if(ui_ChannelNo >=8 && ui_ChannelNo <=11)
                {
                i_Offset=128;
                }//if(ui_ChannelNo >=8 && ui_ChannelNo <=11) 
              if(ui_ChannelNo >=12 && ui_ChannelNo <=15)
                {
                i_Offset=192;
                }//if(ui_ChannelNo >=12 && ui_ChannelNo <=15)
                break;
          case 1:
                if(data[14]==2)
                  {
                   if(ui_ChannelNo ==0 )
                   {
                   i_Offset=0;
                   }//if(ui_ChannelNo ==0 )
                 if(ui_ChannelNo ==1)
                   { 
                   i_Offset=64;
                   }// if(ui_ChannelNo ==1)
                 if(ui_ChannelNo ==2 )
                   {
                   i_Offset=128;
                   }//if(ui_ChannelNo ==2 )
                 if(ui_ChannelNo ==3)
                   {
                   i_Offset=192;
                   }//if(ui_ChannelNo ==3) 
                   i_ChannelNo=0;
                   ui_ChannelNo=0;
                   break;   
                  }//if(data[14]==2)
                if(ui_ChannelNo >=0 && ui_ChannelNo <=1)
                   {
                   i_Offset=0;
                   }//if(ui_ChannelNo >=0 && ui_ChannelNo <=1) 
                 if(ui_ChannelNo >=2 && ui_ChannelNo <=3)
                   {
                   i_ChannelNo=i_ChannelNo-2;
                   ui_ChannelNo=ui_ChannelNo-2; 
                   i_Offset=64;
                   }//if(ui_ChannelNo >=2 && ui_ChannelNo <=3) 
                 if(ui_ChannelNo >=4 && ui_ChannelNo <=5)
                   {
                   i_ChannelNo=i_ChannelNo-4;
                   ui_ChannelNo=ui_ChannelNo-4; 
                   i_Offset=128;
                   }//if(ui_ChannelNo >=4 && ui_ChannelNo <=5) 
                 if(ui_ChannelNo >=6 && ui_ChannelNo <=7)
                   {
                   i_ChannelNo=i_ChannelNo-6;
                   ui_ChannelNo=ui_ChannelNo-6; 
                   i_Offset=192;
                   }//if(ui_ChannelNo >=6 && ui_ChannelNo <=7) 
                 break;
       
        default: printk("\n This selection of polarity does not exist\n");
                 i_err++;
        }//switch(data[2])         
       }//if(data[12]==0 || data[12]==1)
   else
       {
       switch(data[11])
         {
         case 1:i_Offset=0;
                break;
         case 2:i_Offset=64;
                break;
         case 3:i_Offset=128;
                break;
         case 4:i_Offset=192;
                break;
         default:
                printk("\nError in module selection\n");
                i_err++;
         }// switch(data[11]) 
       }// elseif(data[12]==0 || data[12]==1)       
    if(i_err)
      {
      i_APCI3200_Reset(dev);
      return -EINVAL; 
      }
   if(i_ScanType!=1)
      {
      i_Count=0;
      i_Sum=0;
      }//if(i_ScanType!=1)
    
      
  
    ul_Config =  data[1] |(data[2] << 6) |(data[5]<< 7) |(data[3] << 8) |(data[4] << 9);
                              
                              
                              
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);                   
              /*********************************/
              /* Write the channel to configure*/
              /*********************************/
              outl(0 | ui_ChannelNo , devpriv->iobase+i_Offset + 0x4);
            
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1); 
              /**************************/                                                          
              /* Reset the configuration */                                                           
              /**************************/  
               outl(0 , devpriv->iobase+i_Offset + 0x0);  
            
               while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
                  
              /***************************/
              /* Write the configuration */
              /***************************/
              outl(ul_Config , devpriv->iobase+i_Offset + 0x0);

              /***************************/
              /*Reset the calibration bit*/
              /***************************/
              ul_Temp = inl(devpriv->iobase+i_Offset + 12);
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl((ul_Temp & 0xFFF9FFFF) , devpriv->iobase+i_Offset + 12);
              if(data[9]==1)
                 {
                 devpriv->tsk_Current=current;
                 i_InterruptFlag=1;
                 }// if(data[9]==1)
              else
                 {
                  i_InterruptFlag=0;
                 }//else  if(data[9]==1)
              i_Initialised=1;
              if(i_ScanType==1)
                 {
                 i_Sum=i_Sum+1;
                 insn->unused[0]=0; 
                 i_APCI3200_ReadAnalogInput(dev,s,insn,&ui_Dummy);
                 }         
return insn->n;
} 

/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_ReadAnalogInput                         |
|			          (comedi_device *dev,comedi_subdevice *s,       | 
|                     comedi_insn *insn,lsampl_t *data)                      |
+----------------------------------------------------------------------------+
| Task              : Read  value  of the selected channel			         |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     UINT ui_NoOfChannels    : No Of Channels To read       |
|                     UINT *data              : Data Pointer to read status  |
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
|			           data[0]  : Digital Value Of Input             |
|			           data[1]  : Calibration Offset Value           |
|			           data[2]  : Calibration Gain Value          
                               data[3]  : CJC value  
                               data[4]  : CJC offset value
                               data[5]  : CJC gain value 
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/
INT i_APCI3200_ReadAnalogInput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data) 
{ 
   UINT ui_DummyValue=0;
   int i_ConvertCJCCalibration;
   int i=0;
   
   
   if(i_Initialised==0)
     {
     printk("\nThe channel is not initialised\n");
     i_APCI3200_Reset(dev);
     return -EINVAL;
     }//if(i_Initialised==0);
     
         
   switch(insn->unused[0])
      {
      case 0:     
           
             i_APCI3200_Read1AnalogInputChannel(dev,s,insn,&ui_DummyValue);
             ui_InterruptChannelValue[i_Count+0]=ui_DummyValue; 

             if((i_ADDIDATAType==2) && (i_InterruptFlag == FALSE) && (i_CJCAvailable==1))
                {
                i_APCI3200_ReadCJCValue(dev,&ui_DummyValue);
                ui_InterruptChannelValue[i_Count + 3]=ui_DummyValue; 
                }//if((i_ADDIDATAType==2) && (i_InterruptFlag == FALSE))
             else
                {
                  ui_InterruptChannelValue[i_Count + 3]=0;
                }//elseif((i_ADDIDATAType==2) && (i_InterruptFlag == FALSE) && (i_CJCAvailable==1))  
             if (( i_AutoCalibration == FALSE) && (i_InterruptFlag == FALSE))    
                {  
                i_APCI3200_ReadCalibrationOffsetValue(dev,&ui_DummyValue);
                ui_InterruptChannelValue[i_Count + 1]=ui_DummyValue;  
                i_APCI3200_ReadCalibrationGainValue(dev,&ui_DummyValue);  
                ui_InterruptChannelValue[i_Count + 2]=ui_DummyValue; 
                }//if (( i_AutoCalibration == FALSE) && (i_InterruptFlag == FALSE)) 
             if((i_ADDIDATAType==2) && (i_InterruptFlag == FALSE)&& (i_CJCAvailable==1))
                {
                /**********************************************************/
                /*Test if the Calibration channel must be read for the CJC*/
                /**********************************************************/
                 /**********************************/
                 /*Test if the polarity is the same*/
                 /**********************************/                                     
                 if(i_CJCPolarity!=i_ADDIDATAPolarity)
                    {
                    i_ConvertCJCCalibration=1;
                    }//if(i_CJCPolarity!=i_ADDIDATAPolarity)
                 else 
                    {
                    if(i_CJCGain==i_ADDIDATAGain) 
                      {
                       i_ConvertCJCCalibration=0;
                      }//if(i_CJCGain==i_ADDIDATAGain) 
                    else
                      {
                      i_ConvertCJCCalibration=1;
                      }//elseif(i_CJCGain==i_ADDIDATAGain)  
                    }//elseif(i_CJCPolarity!=i_ADDIDATAPolarity)           
                if(i_ConvertCJCCalibration==1)
                   {
                   i_APCI3200_ReadCJCCalOffset(dev,&ui_DummyValue);
                   ui_InterruptChannelValue[i_Count+4]=ui_DummyValue;
                   i_APCI3200_ReadCJCCalGain(dev,&ui_DummyValue);
                   ui_InterruptChannelValue[i_Count+5]=ui_DummyValue;
                   }//if(i_ConvertCJCCalibration==1)
                else
                   {
                    ui_InterruptChannelValue[i_Count+4]=0;
                    ui_InterruptChannelValue[i_Count+5]=0;
                   }//elseif(i_ConvertCJCCalibration==1) 
                }//if((i_ADDIDATAType==2) && (i_InterruptFlag == FALSE)) 
                if(i_ScanType!=1)
                   {
                   i_Count=0;
                   }//if(i_ScanType!=1)
                else  
                   {
                   i_Count=i_Count +6;
                   }//else if(i_ScanType!=1)  
                if((i_ScanType==1) &&(i_InterruptFlag==1))
                   {
                   i_Count=i_Count-6;
                   } 
                if(i_ScanType==0)
                  {           
                  data[0]= ui_InterruptChannelValue[0];
                  data[1]= ui_InterruptChannelValue[1];
                  data[2]= ui_InterruptChannelValue[2];  
                  data[3]= ui_InterruptChannelValue[3];
                  data[4]= ui_InterruptChannelValue[4];
                  data[5]= ui_InterruptChannelValue[5];  
                  

                  }   
                 break;
         case 1 :
                for(i=0;i<insn->n;i++)
                  { 
                  data[i]=ui_InterruptChannelValue[i]; 
                  }

                  i_Count=0;
                  i_Sum=0;
                if(i_ScanType==1)
                  {           
                  i_Initialised=0; 
                  i_InterruptFlag=0; 
                  }   
                 break;  
         default:printk("\nThe parameters passed are in error\n");
                 i_APCI3200_Reset(dev);
                 return -EINVAL;
         }//switch(insn->unused[0])    
return insn->n;
}

/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_Read1AnalogInputChannel                 |
|			          (comedi_device *dev,comedi_subdevice *s,       | 
|                     comedi_insn *insn,lsampl_t *data)                      |
+----------------------------------------------------------------------------+
| Task              : Read  value  of the selected channel			         |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     UINT ui_NoOfChannel    : Channel No to read            |
|                     UINT *data              : Data Pointer to read status  |
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
|			          data[0]  : Digital Value read                   |
|			         			          
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/
INT i_APCI3200_Read1AnalogInputChannel(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data){
UINT ui_EOC=0;
UINT ui_ChannelNo=0;
UINT ui_CommandRegister=0;
     
 ui_ChannelNo=i_ChannelNo;
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);                   
              /*********************************/
              /* Write the channel to configure*/
              /*********************************/
              outl(0 | ui_Channel_num , devpriv->iobase+i_Offset + 0x4);
           
/*******************************/
/* Set the convert timing unit */
/*******************************/
while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
outl(i_ADDIDATAConversionTimeUnit , devpriv->iobase+i_Offset + 36);                             
/**************************/
/* Set the convert timing */
/**************************/
while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
outl(i_ADDIDATAConversionTime , devpriv->iobase+i_Offset + 32);
         
/**************************************************************************/
/* Set the start end stop index to the selected channel and set the start */
/**************************************************************************/ 

  
ui_CommandRegister = ui_ChannelNo | (ui_ChannelNo << 8) | 0x80000;  
    
   /*********************************/
   /*Test if the interrupt is enable*/
   /*********************************/
             
 if (i_InterruptFlag == ADDIDATA_ENABLE)
   { 
   /************************/
   /* Enable the interrupt */
   /************************/
   ui_CommandRegister = ui_CommandRegister | 0x00100000;
   }//if (i_InterruptFlag == ADDIDATA_ENABLE)
              
   /******************************/
   /* Write the command register */
   /******************************/
    while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
    outl(ui_CommandRegister, devpriv->iobase+i_Offset + 8);
   /*****************************/
   /*Test if interrupt is enable*/
   /*****************************/              
if (i_InterruptFlag == ADDIDATA_DISABLE)
   {
    do
       {
        /*************************/
        /*Read the EOC Status bit*/
        /*************************/
                    
        ui_EOC = inl(devpriv->iobase+i_Offset + 20) & 1;

       } while (ui_EOC != 1);
                

        /***************************************/
        /* Read the digital value of the input */
        /***************************************/
                

        data[0] = inl (devpriv->iobase+i_Offset + 28);

              
    }// if (i_InterruptFlag == ADDIDATA_DISABLE)
return 0;
} 

/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_ReadCalibrationOffsetValue              |
|			          (comedi_device *dev,comedi_subdevice *s,       | 
|                     comedi_insn *insn,lsampl_t *data)                      |
+----------------------------------------------------------------------------+
| Task              : Read calibration offset  value  of the selected channel|
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     UINT *data              : Data Pointer to read status  |
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
|			          data[0]  : Calibration offset Value   |
|			         			          
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/
int i_APCI3200_ReadCalibrationOffsetValue(comedi_device *dev,UINT *data)
{
   UINT ui_Temp=0 , ui_EOC=0;
   UINT ui_CommandRegister=0;
    while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);                   
              /*********************************/
              /* Write the channel to configure*/
              /*********************************/
              outl(0 | ui_Channel_num , devpriv->iobase+i_Offset + 0x4);
   
              /*******************************/
              /* Set the convert timing unit */
              /*******************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(i_ADDIDATAConversionTimeUnit , devpriv->iobase+i_Offset + 36);
              /**************************/
              /* Set the convert timing */
              /**************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(i_ADDIDATAConversionTime , devpriv->iobase+i_Offset + 32);
              /*****************************/
              /*Read the calibration offset*/
              /*****************************/
               ui_Temp = inl(devpriv->iobase+i_Offset + 12);

              /*********************************/
              /*Configure the Offset Conversion*/
              /*********************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl((ui_Temp | 0x00020000), devpriv->iobase+i_Offset + 12);
              /*******************************/
              /*Initialise ui_CommandRegister*/
              /*******************************/
             
              ui_CommandRegister = 0;

              /*********************************/
              /*Test if the interrupt is enable*/
              /*********************************/

              if (i_InterruptFlag == ADDIDATA_ENABLE)
                 {
        
                 /**********************/
                 /*Enable the interrupt*/
                 /**********************/

                 ui_CommandRegister = ui_CommandRegister | 0x00100000;

                 }//if (i_InterruptFlag == ADDIDATA_ENABLE)
             
              /**********************/
              /*Start the conversion*/
              /**********************/
              ui_CommandRegister = ui_CommandRegister | 0x00080000;


              /***************************/
              /*Write the command regiter*/
              /***************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(ui_CommandRegister, devpriv->iobase+i_Offset + 8);

              /*****************************/
              /*Test if interrupt is enable*/
              /*****************************/
              
              if (i_InterruptFlag == ADDIDATA_DISABLE)
                 {
            
                 do
                    {
                    /*******************/
                    /*Read the EOC flag*/
                    /*******************/
                    
                    ui_EOC = inl (devpriv->iobase+i_Offset + 20) & 1;

                    } while (ui_EOC != 1);
                

                 /**************************************************/
                 /*Read the digital value of the calibration Offset*/
                 /**************************************************/
                 

                 data[0] = inl(devpriv->iobase+i_Offset+ 28);
                 }//if (i_InterruptFlag == ADDIDATA_DISABLE)
return 0;
}

/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_ReadCalibrationGainValue                |
|			          (comedi_device *dev,comedi_subdevice *s,       | 
|                     comedi_insn *insn,lsampl_t *data)                      |
+----------------------------------------------------------------------------+
| Task              : Read calibration gain  value  of the selected channel  |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     UINT *data              : Data Pointer to read status  |
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
|			          data[0]  : Calibration gain Value Of Input     |
|			         			          
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/ 
int i_APCI3200_ReadCalibrationGainValue(comedi_device *dev,UINT *data)
{
  UINT ui_EOC=0;
  INT ui_CommandRegister=0;
 while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);                   
              /*********************************/
              /* Write the channel to configure*/
              /*********************************/
              outl(0 | ui_Channel_num , devpriv->iobase+i_Offset + 0x4);
    
              /***************************/
              /*Read the calibration gain*/
              /***************************/
              /*******************************/
              /* Set the convert timing unit */
              /*******************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(i_ADDIDATAConversionTimeUnit , devpriv->iobase+i_Offset + 36);
              /**************************/
              /* Set the convert timing */
              /**************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(i_ADDIDATAConversionTime , devpriv->iobase+i_Offset + 32);
              /*******************************/
              /*Configure the Gain Conversion*/
              /*******************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(0x00040000 , devpriv->iobase+i_Offset + 12);
             

              /*******************************/
              /*Initialise ui_CommandRegister*/
              /*******************************/

              ui_CommandRegister = 0;

              /*********************************/
              /*Test if the interrupt is enable*/
              /*********************************/
              
              if (i_InterruptFlag == ADDIDATA_ENABLE)
                 {
                 
                 /**********************/
                 /*Enable the interrupt*/
                 /**********************/
            
                 ui_CommandRegister = ui_CommandRegister | 0x00100000;

                 }//if (i_InterruptFlag == ADDIDATA_ENABLE)
              
              /**********************/
              /*Start the conversion*/
              /**********************/
              
              ui_CommandRegister = ui_CommandRegister | 0x00080000;
              /***************************/
              /*Write the command regiter*/
              /***************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(ui_CommandRegister , devpriv->iobase+i_Offset + 8);

              /*****************************/
              /*Test if interrupt is enable*/
              /*****************************/
              
              if (i_InterruptFlag == ADDIDATA_DISABLE)
                 {
               
                 do
                    {
                   
                    /*******************/
                    /*Read the EOC flag*/
                    /*******************/
                    
                    ui_EOC = inl(devpriv->iobase+i_Offset + 20) & 1;

                    } while (ui_EOC != 1);
                 
                 /************************************************/
                 /*Read the digital value of the calibration Gain*/
                 /************************************************/

                 data[0] = inl(devpriv->iobase+i_Offset + 28);

                 }//if (i_InterruptFlag == ADDIDATA_DISABLE)
return 0;
}

/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_ReadCJCValue                            |
|			          (comedi_device *dev,comedi_subdevice *s,       | 
|                     comedi_insn *insn,lsampl_t *data)                      |
+----------------------------------------------------------------------------+
| Task              : Read CJC  value  of the selected channel               |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     UINT *data              : Data Pointer to read status  |
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
|			          data[0]  : CJC Value                           |
|			         			          
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/ 

int i_APCI3200_ReadCJCValue(comedi_device *dev,lsampl_t *data)
{
 UINT  ui_EOC=0;
 INT ui_CommandRegister=0;
   
           /******************************/
           /*Set the converting time unit*/
           /******************************/
          
            while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
            outl(i_ADDIDATAConversionTimeUnit , devpriv->iobase+i_Offset + 36);
              /**************************/
              /* Set the convert timing */
              /**************************/
             while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
             outl(i_ADDIDATAConversionTime , devpriv->iobase+i_Offset + 32);  

              /******************************/
              /*Configure the CJC Conversion*/
              /******************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl( 0x00000400 , devpriv->iobase+i_Offset + 4);
              /*******************************/
              /*Initialise dw_CommandRegister*/
              /*******************************/
              ui_CommandRegister = 0;
             /*********************************/
             /*Test if the interrupt is enable*/
             /*********************************/  
             if (i_InterruptFlag == ADDIDATA_ENABLE)
                 { 
                  /**********************/
                  /*Enable the interrupt*/
                  /**********************/
                 ui_CommandRegister =ui_CommandRegister | 0x00100000; 
                  }
              
             /**********************/
              /*Start the conversion*/
              /**********************/
             
              ui_CommandRegister = ui_CommandRegister | 0x00080000;

              /***************************/
              /*Write the command regiter*/
              /***************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(ui_CommandRegister , devpriv->iobase+i_Offset + 8);
         
              /*****************************/
              /*Test if interrupt is enable*/
              /*****************************/
              
              if (i_InterruptFlag == ADDIDATA_DISABLE)
                 {
                 do
                    {
                   
                    /*******************/
                    /*Read the EOC flag*/
                    /*******************/
                    
                    ui_EOC = inl(devpriv->iobase+i_Offset + 20) & 1;

                    } while (ui_EOC != 1);
                 
                 /***********************************/
                 /*Read the digital value of the CJC*/
                 /***********************************/
                 
                 data[0] = inl(devpriv->iobase+i_Offset + 28);

                 }//if (i_InterruptFlag == ADDIDATA_DISABLE) 
return 0;
}      

/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_ReadCJCCalOffset                        |
|			          (comedi_device *dev,comedi_subdevice *s,       | 
|                     comedi_insn *insn,lsampl_t *data)                      |
+----------------------------------------------------------------------------+
| Task              : Read CJC calibration offset  value  of the selected channel  
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     UINT *data              : Data Pointer to read status  |
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
|			          data[0]  : CJC calibration offset Value                    
|			         			          
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/
int i_APCI3200_ReadCJCCalOffset(comedi_device *dev,lsampl_t *data)
{
 UINT  ui_EOC=0;
 INT ui_CommandRegister=0;
               /*******************************************/
               /*Read calibration offset value for the CJC*/
               /*******************************************/ 
               /*******************************/
               /* Set the convert timing unit */
               /*******************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(i_ADDIDATAConversionTimeUnit , devpriv->iobase+i_Offset + 36);
              /**************************/
              /* Set the convert timing */
              /**************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(i_ADDIDATAConversionTime , devpriv->iobase+i_Offset + 32);               
              /******************************/
              /*Configure the CJC Conversion*/
              /******************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(0x00000400 , devpriv->iobase+i_Offset + 4); 
              /*********************************/
              /*Configure the Offset Conversion*/
              /*********************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(0x00020000, devpriv->iobase+i_Offset + 12); 
              
              /*******************************/
              /*Initialise ui_CommandRegister*/
              /*******************************/
              ui_CommandRegister = 0;
              /*********************************/
              /*Test if the interrupt is enable*/
              /*********************************/
              
             if (i_InterruptFlag == ADDIDATA_ENABLE)
                 { 
                  /**********************/
                  /*Enable the interrupt*/
                  /**********************/
                 ui_CommandRegister =ui_CommandRegister | 0x00100000; 
               
                  }
              
             /**********************/
              /*Start the conversion*/
              /**********************/
             ui_CommandRegister = ui_CommandRegister | 0x00080000;
             /***************************/
              /*Write the command regiter*/
              /***************************/
              while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
              outl(ui_CommandRegister,devpriv->iobase+i_Offset + 8);  
              if (i_InterruptFlag == ADDIDATA_DISABLE)
                 {
                 do
                    {
                    /*******************/
                    /*Read the EOC flag*/
                    /*******************/
                    ui_EOC = inl(devpriv->iobase+i_Offset + 20) & 1;
                    } while (ui_EOC != 1);
                 
                 /**************************************************/
                 /*Read the digital value of the calibration Offset*/
                 /**************************************************/
                 data[0] = inl(devpriv->iobase+i_Offset + 28);
                 }//if (i_InterruptFlag == ADDIDATA_DISABLE) 
return 0;
} 

/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_ReadCJCGainValue                        |
|			          (comedi_device *dev,comedi_subdevice *s,       | 
|                     comedi_insn *insn,lsampl_t *data)                      |
+----------------------------------------------------------------------------+
| Task              : Read CJC calibration gain value  
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     UINT ui_NoOfChannels    : No Of Channels To read       |
|                     UINT *data              : Data Pointer to read status  |
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
|			          data[0]  : CJC calibration gain value                      
|			         			          
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/ 
int i_APCI3200_ReadCJCCalGain(comedi_device *dev,lsampl_t *data)  
{   
 UINT  ui_EOC=0;
 INT ui_CommandRegister=0;
 /*******************************/
 /* Set the convert timing unit */
 /*******************************/
 while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
 outl(i_ADDIDATAConversionTimeUnit , devpriv->iobase+i_Offset + 36);
 /**************************/
 /* Set the convert timing */
 /**************************/
  while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
   outl(i_ADDIDATAConversionTime , devpriv->iobase+i_Offset + 32);
 /******************************/
 /*Configure the CJC Conversion*/
 /******************************/
  while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
  outl(0x00000400,devpriv->iobase+i_Offset + 4); 
/*******************************/
/*Configure the Gain Conversion*/
/*******************************/
 while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
 outl(0x00040000,devpriv->iobase+i_Offset + 12);             

 /*******************************/
 /*Initialise dw_CommandRegister*/
 /*******************************/
   ui_CommandRegister = 0;            
/*********************************/
/*Test if the interrupt is enable*/
/*********************************/
 if (i_InterruptFlag == ADDIDATA_ENABLE)  
{          
/**********************/
/*Enable the interrupt*/
/**********************/
ui_CommandRegister = ui_CommandRegister | 0x00100000;           
}
/**********************/
/*Start the conversion*/
/**********************/             
ui_CommandRegister = ui_CommandRegister | 0x00080000;
/***************************/
/*Write the command regiter*/
/***************************/            
while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
outl(ui_CommandRegister ,devpriv->iobase+i_Offset + 8);
if (i_InterruptFlag == ADDIDATA_DISABLE)
    {
      do
         {            
         /*******************/
         /*Read the EOC flag*/
         /*******************/
          ui_EOC = inl(devpriv->iobase+i_Offset + 20) & 1;      
         } while (ui_EOC != 1);         
       /************************************************/
       /*Read the digital value of the calibration Gain*/
       /************************************************/
        data[0] = inl (devpriv->iobase+i_Offset + 28);           
    }//if (i_InterruptFlag == ADDIDATA_DISABLE)
return 0;
}              

                  
/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_InsnBits_AnalogInput_Test               |
|			  (comedi_device *dev,comedi_subdevice *s,               | 
|                      comedi_insn *insn,lsampl_t *data)                     |
+----------------------------------------------------------------------------+
| Task              : Tests the Selected Anlog Input Channel                 |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     comedi_subdevice *s     : Subdevice Pointer            |  
|                     comedi_insn *insn       : Insn Structure Pointer       |  
|                     lsampl_t *data          : Data Pointer contains        |
|                                          configuration parameters as below |
|                                                                           
|                         
|                           data[0]            : 0 TestAnalogInputShortCircuit			     
|									     1 TestAnalogInputConnection							 														                        | 

+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
|			        data[0]            : Digital value obtained      | 
|                           data[1]            : calibration offset          |
|                           data[2]            : calibration gain            |         
|			                                                         |
|			                                                         |
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/

INT i_APCI3200_InsnBits_AnalogInput_Test(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data) 
{
UINT 	ui_Configuration=0;
INT i_Temp,i_TimeUnit;
if(i_Initialised==0)
   {
   printk("\nThe channel is not initialised\n");
   i_APCI3200_Reset(dev);
   return -EINVAL;
   }//if(i_Initialised==0);   
if(data[0]!=0 && data[0]!=1)
  {
   printk("\nError in selection of functionality\n");
   i_APCI3200_Reset(dev);
   return -EINVAL;
  }//if(data[0]!=0 && data[0]!=1)

if(data[0]==1) //Perform Short Circuit TEST 
  {
  /**************************/
  /*Set the short-cicuit bit*/
  /**************************/
  while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
  outl((0x00001000 |i_ChannelNo) , devpriv->iobase+i_Offset + 4);
      /*************************/
      /*Set the time unit to ns*/
      /*************************/
     /* i_TimeUnit= i_ADDIDATAConversionTimeUnit;
      i_ADDIDATAConversionTimeUnit= 1;*/
       i_Temp= i_InterruptFlag ;  
      i_InterruptFlag = ADDIDATA_DISABLE;
      i_APCI3200_Read1AnalogInputChannel(dev,s,insn,data); 
       if(i_AutoCalibration == FALSE)
         {
          while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
          outl((0x00001000 |i_ChannelNo) , devpriv->iobase+i_Offset + 4);
          data++;
          i_APCI3200_ReadCalibrationOffsetValue(dev,data);  
          data++; 
          i_APCI3200_ReadCalibrationGainValue(dev,data); 
         }  
   }
else
   {
     while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
     outl((0x00000800|i_ChannelNo) , devpriv->iobase+i_Offset + 4); 
     ui_Configuration = inl(devpriv->iobase+i_Offset + 0);
      /*************************/
      /*Set the time unit to ns*/
      /*************************/
     /* i_TimeUnit= i_ADDIDATAConversionTimeUnit;
      i_ADDIDATAConversionTimeUnit= 1;*/
       i_Temp= i_InterruptFlag ;  
      i_InterruptFlag = ADDIDATA_DISABLE;
      i_APCI3200_Read1AnalogInputChannel(dev,s,insn,data); 
      if(i_AutoCalibration == FALSE)
         {
         while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
         outl((0x00000800|i_ChannelNo) , devpriv->iobase+i_Offset + 4);
          data++;
          i_APCI3200_ReadCalibrationOffsetValue(dev,data);
          data++;     
          i_APCI3200_ReadCalibrationGainValue(dev,data);    
         }
     }
          i_InterruptFlag=i_Temp ;
printk("\ni_InterruptFlag=%d\n",i_InterruptFlag);
return insn->n;
} 

/*
+----------------------------------------------------------------------------+
| Function   Name   : int i_APCI3200_InsnWriteReleaseAnalogInput             |
|			  (comedi_device *dev,comedi_subdevice *s,               | 
|                      comedi_insn *insn,lsampl_t *data)                     |
+----------------------------------------------------------------------------+
| Task              :  Resets the channels                                                      |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev      : Driver handle                |
|                     comedi_subdevice *s     : Subdevice Pointer            |  
|                     comedi_insn *insn       : Insn Structure Pointer       |  
|                     lsampl_t *data          : Data Pointer 
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |

+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error          |
|			                                                         |
+----------------------------------------------------------------------------+
*/

INT i_APCI3200_InsnWriteReleaseAnalogInput(comedi_device *dev,comedi_subdevice *s,comedi_insn *insn,lsampl_t *data)
{ 
  i_APCI3200_Reset(dev);   
  return insn->n;
    }    
          
                 
          
                 
     
                 
 
     
/*
+----------------------------------------------------------------------------+
| Function name     :int i_APCI3200_CommandTestAnalogInput(comedi_device *dev|
|			,comedi_subdevice *s,comedi_cmd *cmd)			         |
|                                        									 |
+----------------------------------------------------------------------------+
| Task              : Test validity for a command for cyclic anlog input     |
|                       acquisition  						     			 | 
|                     										                 |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev									 |
|                     comedi_subdevice *s									 |
|                     comedi_cmd *cmd              					         |
|                     										                 |
|           
|                     										                 |
|                     										                 |
|                     										                 |
+----------------------------------------------------------------------------+
| Return Value      :0              					                     |
|                    													     |
+----------------------------------------------------------------------------+
*/

int i_APCI3200_CommandTestAnalogInput(comedi_device *dev,comedi_subdevice *s,comedi_cmd *cmd)
{

        int err=0;
	int tmp;// divisor1,divisor2;
        UINT ui_ConvertTime=0;
        UINT ui_ConvertTimeBase=0;
        UINT ui_DelayTime=0;
        UINT ui_DelayTimeBase=0;
        INT i_Triggermode=0;
        INT i_TriggerEdge=0; 
        INT i_NbrOfChannel=0;
        INT i_Cpt=0;
        double d_ConversionTimeForAllChannels=0.0;
        double d_SCANTimeNewUnit=0.0;
	// step 1: make sure trigger sources are trivially valid 

	tmp=cmd->start_src;
	cmd->start_src &= TRIG_NOW|TRIG_EXT;
	if(!cmd->start_src || tmp!=cmd->start_src)err++;
	tmp=cmd->scan_begin_src;
	cmd->scan_begin_src &= TRIG_TIMER|TRIG_FOLLOW;
	if(!cmd->scan_begin_src || tmp!=cmd->scan_begin_src)err++;
        tmp=cmd->convert_src;
	cmd->convert_src &= TRIG_TIMER;
	if(!cmd->convert_src || tmp!=cmd->convert_src)err++;
        tmp=cmd->scan_end_src;
	cmd->scan_end_src &= TRIG_COUNT;
	if(!cmd->scan_end_src || tmp!=cmd->scan_end_src)err++;
	tmp=cmd->stop_src;
	cmd->stop_src &= TRIG_COUNT|TRIG_NONE;
	if(!cmd->stop_src || tmp!=cmd->stop_src)err++; 
        if(i_InterruptFlag==0)
          {
          err++;
//          printk("\nThe interrupt should be enabled\n");
          }
	if(err) 
           {
            i_APCI3200_Reset(dev);
            return 1;
           }  
        

	if(cmd->start_src!=TRIG_NOW && cmd->start_src!=TRIG_EXT) 
        {
		err++;
	}
        if(cmd->start_src==TRIG_EXT)
           {
            i_TriggerEdge=cmd->start_arg & 0xFFFF;
            i_Triggermode=cmd->start_arg >> 16;
            if(i_TriggerEdge < 1 || i_TriggerEdge >3 )
              {
               err++;
               printk("\nThe trigger edge selection is in error\n") ;
              }
             if(i_Triggermode!=2)
              {
               err++;
               printk("\nThe trigger mode selection is in error\n") ;
              }   
           }    

        if(cmd->scan_begin_src!=TRIG_TIMER &&
	     cmd->scan_begin_src!=TRIG_FOLLOW) err++;

        if(cmd->convert_src!=TRIG_TIMER ) err++;
        
        if(cmd->scan_end_src!=TRIG_COUNT) 
        {
		cmd->scan_end_src=TRIG_COUNT;
		err++;
	}
                  
        if(cmd->stop_src!=TRIG_NONE &&
	   cmd->stop_src!=TRIG_COUNT ) err++;

        if(err) 
           {
           i_APCI3200_Reset(dev); 
           return 2;
           } 
          
         i_FirstChannel=cmd->chanlist[0];
         i_LastChannel=cmd->chanlist[1];   
  
       
        if (cmd->convert_src==TRIG_TIMER)
            {
              ui_ConvertTime=cmd->convert_arg & 0xFFFF;
              ui_ConvertTimeBase=cmd->convert_arg >> 16; 
        if (ui_ConvertTime!=20 && ui_ConvertTime!=40 && ui_ConvertTime!=80 && ui_ConvertTime!=160 )

	{
         printk("\nThe selection of conversion time reload value is in error\n");
         err++;
         }// if (ui_ConvertTime!=20 && ui_ConvertTime!=40 && ui_ConvertTime!=80 && ui_ConvertTime!=160 ) 
      if(ui_ConvertTimeBase!=2)
        {
        printk("\nThe selection of conversion time unit  is in error\n");
        err++;
        }//if(ui_ConvertTimeBase!=2)  
            }
         else
            {
             ui_ConvertTime=0;
             ui_ConvertTimeBase=0; 
            }
        if(cmd->scan_begin_src==TRIG_FOLLOW)
	    { 
             ui_DelayTime=0;
             ui_DelayTimeBase=0;
            }//if(cmd->scan_begin_src==TRIG_FOLLOW)
        else
            {
            ui_DelayTime=cmd->scan_begin_arg & 0xFFFF;
            ui_DelayTimeBase=cmd->scan_begin_arg >> 16; 
            if(ui_DelayTimeBase!=2 && ui_DelayTimeBase!=3)
               {
               err++;
               printk("\nThe Delay time base selection is in error\n");
               }
             if(ui_DelayTime < 1 && ui_DelayTime >1023)
                {
                 err++;
                 printk("\nThe Delay time value is in error\n"); 
                }
               if(err)
                 {
                 i_APCI3200_Reset(dev);  
                 return 3;
                 }
            d_SCANTimeNewUnit = (double)ui_DelayTime;
            i_NbrOfChannel= i_LastChannel-i_FirstChannel + 4;
             /**********************************************************/
             /*calculate the total conversion time for all the channels*/
             /**********************************************************/
             d_ConversionTimeForAllChannels = (double)((double)ui_ConvertTime / (double)i_NbrOfChannel);
             
             /*******************************/
             /*Convert the frequence in time*/
             /*******************************/ 
             d_ConversionTimeForAllChannels =(double) 1.0/ d_ConversionTimeForAllChannels;
             ui_ConvertTimeBase=3; 
              /***********************************/
              /*Test if the time unit is the same*/
              /***********************************/
                   
                    if (ui_DelayTimeBase <= ui_ConvertTimeBase)
                       {
                      
                       for (i_Cpt = 0; i_Cpt < (ui_ConvertTimeBase-ui_DelayTimeBase);i_Cpt++)
                          {
                           
                          d_ConversionTimeForAllChannels = d_ConversionTimeForAllChannels * 1000;
                          d_ConversionTimeForAllChannels=d_ConversionTimeForAllChannels+1;
                          }
                       }
                    else
                       {
                       for (i_Cpt = 0; i_Cpt < (ui_DelayTimeBase-ui_ConvertTimeBase);i_Cpt++)
                          {
                          d_SCANTimeNewUnit = d_SCANTimeNewUnit * 1000;

                          }
                       }
                      
                       if (d_ConversionTimeForAllChannels >= d_SCANTimeNewUnit)
                         {
                       
                       printk("\nSCAN Delay value cannot be used\n");
                       /*********************************/
                       /*SCAN Delay value cannot be used*/
                       /*********************************/
                       err++;
                       }           
            }//else if(cmd->scan_begin_src==TRIG_FOLLOW) 

	
         
	if(err) 
           {
           i_APCI3200_Reset(dev);  
           return 4;
           }
        
  return 0;
  }

/*
+----------------------------------------------------------------------------+
| Function name     :int i_APCI3200_StopCyclicAcquisition(comedi_device *dev,|
| 											     comedi_subdevice *s)|
|                                        									 |
+----------------------------------------------------------------------------+
| Task              : Stop the  acquisition  						     |
|                     										                 |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev									 |
|                     comedi_subdevice *s									 |
|                                                 					         |
+----------------------------------------------------------------------------+
| Return Value      :0              					                     |
|                    													     |
+----------------------------------------------------------------------------+
*/

int i_APCI3200_StopCyclicAcquisition(comedi_device *dev,comedi_subdevice *s)
{
UINT ui_Configuration=0;
i_InterruptFlag=0;
i_Initialised==0;
i_Count=0;
i_Sum=0;
 /*******************/
 /*Read the register*/
 /*******************/
  ui_Configuration = inl(devpriv->iobase+i_Offset + 8);             
/*****************************/
/*Reset the START and IRQ bit*/
/*****************************/
 while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
 outl((ui_Configuration & 0xFFE7FFFF),devpriv->iobase+i_Offset + 8);
return 0;             
}
/*
+----------------------------------------------------------------------------+
| Function name     : int i_APCI3200_CommandAnalogInput(comedi_device *dev,  |
|												comedi_subdevice *s) |
|                                        									 |
+----------------------------------------------------------------------------+
| Task              : Does asynchronous acquisition                          |
|                     Determines the mode 1 or 2.						     |
|                     										                 |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev									 |
|                     comedi_subdevice *s									 |
|                     														 |
|                     														 |
+----------------------------------------------------------------------------+
| Return Value      :              					                         |
|                    													     |
+----------------------------------------------------------------------------+
*/

 int i_APCI3200_CommandAnalogInput(comedi_device *dev,comedi_subdevice *s)
{
comedi_cmd *cmd=&s->async->cmd; 
UINT ui_Configuration=0;
INT  i_CurrentSource = 0; 
UINT ui_Trigger=0;
UINT ui_TriggerEdge=0;
UINT ui_Triggermode=0; 
UINT ui_ScanMode=0;
UINT ui_ConvertTime=0;
UINT ui_ConvertTimeBase=0;
UINT ui_DelayTime=0;
UINT ui_DelayTimeBase=0;
UINT ui_DelayMode=0;
     i_FirstChannel=cmd->chanlist[0];
     i_LastChannel=cmd->chanlist[1];
     if(cmd->start_src==TRIG_EXT) 
        {
         ui_Trigger=1;
         ui_TriggerEdge=cmd->start_arg &0xFFFF;
         ui_Triggermode=cmd->start_arg >> 16;
        }//if(cmd->start_src==TRIG_EXT) 
     else
        {
         ui_Trigger=0;
        }//elseif(cmd->start_src==TRIG_EXT)    
     
     if (cmd->stop_src==TRIG_COUNT) 
	{ 
	     ui_ScanMode=0; 
	}// if (cmd->stop_src==TRIG_COUNT) 
     else   
	{ 
	    ui_ScanMode=2;
	}//else if (cmd->stop_src==TRIG_COUNT) 
         
	if(cmd->scan_begin_src==TRIG_FOLLOW)
	    { 
             ui_DelayTime=0;
             ui_DelayTimeBase=0;
             ui_DelayMode=0;
            }//if(cmd->scan_begin_src==TRIG_FOLLOW)
        else
            {
            ui_DelayTime=cmd->scan_begin_arg & 0xFFFF;
            ui_DelayTimeBase=cmd->scan_begin_arg >> 16;
            ui_DelayMode =1;  
            }//else if(cmd->scan_begin_src==TRIG_FOLLOW) 
//        printk("\nui_DelayTime=%u\n",ui_DelayTime);
//        printk("\nui_DelayTimeBase=%u\n",ui_DelayTimeBase);
        if (cmd->convert_src==TRIG_TIMER)
            {
              ui_ConvertTime=cmd->convert_arg & 0xFFFF;
              ui_ConvertTimeBase=cmd->convert_arg >> 16; 
            }
         else
            {
             ui_ConvertTime=0;
             ui_ConvertTimeBase=0; 
            }
       printk("\nui_ConvertTime=%u\n",ui_ConvertTime);
       printk("\nui_ConvertTimebase=%u\n",ui_ConvertTimeBase);
    // if(i_ADDIDATAType ==1 || ((i_ADDIDATAType==2)))
     //   {
     /**************************************************/
     /*Read the old configuration of the current source*/
     /**************************************************/   
     ui_Configuration = inl(devpriv->iobase+i_Offset + 12);
     /***********************************************/
     /*Write the configuration of the current source*/
     /***********************************************/
     while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
    outl((ui_Configuration & 0xFFC00000 ), devpriv->iobase+i_Offset +12);
       // } 
     ui_Configuration=0;
//     printk("\nfirstchannel=%u\n",i_FirstChannel);
//     printk("\nlastchannel=%u\n",i_LastChannel);
//     printk("\nui_Trigger=%u\n",ui_Trigger); 
//     printk("\nui_TriggerEdge=%u\n",ui_TriggerEdge);
//     printk("\nui_Triggermode=%u\n",ui_Triggermode);   
//      printk("\nui_DelayMode=%u\n",ui_DelayMode);  
//     printk("\nui_ScanMode=%u\n",ui_ScanMode);       
     ui_Configuration = i_FirstChannel |(i_LastChannel << 8)| 0x00100000 |
                                 (ui_Trigger << 24) |
                                 (ui_TriggerEdge << 25)|
                                 (ui_Triggermode << 27)| 
                                 (ui_DelayMode << 18) |
                                 (ui_ScanMode << 16);
    
 
      /*************************/
      /*Write the Configuration*/
      /*************************/
        while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
        outl( ui_Configuration, devpriv->iobase+i_Offset + 0x8);          
     /***********************/
     /*Write the Delay Value*/
     /***********************/
     while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
     outl(ui_DelayTime,devpriv->iobase+i_Offset + 40);                      
      /***************************/
      /*Write the Delay time base*/
      /***************************/
      while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
      outl(ui_DelayTimeBase,devpriv->iobase+i_Offset + 44);                      
      /*********************************/
      /*Write the conversion time value*/
      /*********************************/
       while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
       outl(ui_ConvertTime,devpriv->iobase+i_Offset + 32);                  
       /********************************/
       /*Write the conversion time base*/
       /********************************/
       while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
       outl(ui_ConvertTimeBase,devpriv->iobase+i_Offset + 36);          
        /*******************/
        /*Read the register*/
        /*******************/
         ui_Configuration = inl(devpriv->iobase+i_Offset + 4);       
       /******************/
       /*Set the SCAN bit*/
       /******************/
       while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);

       outl(((ui_Configuration & 0x1E0FF) | 0x00002000),devpriv->iobase+i_Offset + 4);                  
      /*******************/
      /*Read the register*/
      /*******************/
       ui_Configuration=0;       
       ui_Configuration = inl(devpriv->iobase+i_Offset + 8);

       /*******************/
        /*Set the START bit*/
        /*******************/
       while (((inl(devpriv->iobase+i_Offset+12)>>19) & 1) != 1);
       outl((ui_Configuration | 0x00080000),devpriv->iobase+i_Offset + 8);        
  return 0;
}





/*
+----------------------------------------------------------------------------+
| Function   Name   :  int i_APCI3200_Reset(comedi_device *dev)			     |
|							                                         |
+----------------------------------------------------------------------------+
| Task              :Resets the registers of the card                        |
+----------------------------------------------------------------------------+
| Input Parameters  :                                                        |
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
+----------------------------------------------------------------------------+
| Return Value      :                                                        |
|					                                                 |
+----------------------------------------------------------------------------+
*/

int i_APCI3200_Reset(comedi_device *dev)
{
INT i_Temp;
DWORD dw_Dummy;
i_InterruptFlag=0;
i_Initialised==0;
i_Count=0;
i_Sum=0;

outl(0x83838383,devpriv->i_IobaseAmcc+0x60);
                 
    		// Enable the interrupt for the controler 
		dw_Dummy =  inl(devpriv->i_IobaseAmcc+ 0x38);
 		outl(dw_Dummy | 0x2000,devpriv->i_IobaseAmcc+0x38);
outl(0,devpriv->i_IobaseAddon);//Resets the output
/***************/
/*Empty the buffer*/
/**************/
for(i_Temp=0;i_Temp<=95;i_Temp++)
   {
    ui_InterruptChannelValue[i_Temp]=0;
   }//for(i_Temp=0;i_Temp<=95;i_Temp++) 
/*****************************/
/*Reset the START and IRQ bit*/
/*****************************/
for(i_Temp=0;i_Temp<=192;)
  {
  while (((inl(devpriv->iobase+i_Temp+12)>>19) & 1) != 1);
  outl(0,devpriv->iobase+i_Temp + 8);
  i_Temp=i_Temp+64;
  }//for(i_Temp=0;i_Temp<=192;i_Temp+64) 
return 0;
}

/*
+----------------------------------------------------------------------------+
| Function   Name   : static void v_APCI3200_Interrupt					     |
|					  (int irq , void *d, struct pt_regs *regs)				 |
+----------------------------------------------------------------------------+
| Task              : Interrupt processing Routine                           |
+----------------------------------------------------------------------------+
| Input Parameters  : int irq                 : irq number                   |
|                     void *d                 : void pointer                 |
|                     struct pt_regs *regs    : structure pointer            |
+----------------------------------------------------------------------------+
| Output Parameters :	--													 |
+----------------------------------------------------------------------------+
| Return Value      : TRUE  : No error occur                                 |
|		            : FALSE : Error occur. Return the error					 |
|					                                                         |
+----------------------------------------------------------------------------+
*/
 void v_APCI3200_Interrupt(int irq, void *d, struct pt_regs *regs) 
{	
 comedi_device *dev = d;
 UINT ui_StatusRegister=0;
 UINT ui_ChannelNumber = 0;
 INT  i_CalibrationFlag = 0;
 INT i_CJCFlag=0;
 UINT  ui_DummyValue = 0;
 UINT ui_DigitalTemperature=0;
 UINT ui_DigitalInput =0;
 int i_ConvertCJCCalibration;
 int k;

switch(i_ScanType)
   {
    case 0: 
    case 1: 
          switch(i_ADDIDATAType)
             {        
              case 0:
              case 1: 
             
             /************************************/
             /*Read the interrupt status register*/
             /************************************/
             ui_StatusRegister = inl(devpriv->iobase+i_Offset + 16);
             if ((ui_StatusRegister & 0x2) == 0x2)
                {
                i_CalibrationFlag = ((inl(devpriv->iobase+i_Offset + 12) & 0x00060000) >> 17); 
                /*************************/
                /*Read the channel number*/
                /*************************/
                ui_ChannelNumber = inl(devpriv->iobase+i_Offset + 24);
                /*************************************/
                /*Read the digital analog input value*/
                /*************************************/
                ui_DigitalInput = inl(devpriv->iobase+i_Offset + 28);
                /***********************************************/
                /* Test if the value read is the channel value */
                /***********************************************/
                if (i_CalibrationFlag == 0)
                  {
                   ui_InterruptChannelValue[i_Count + 0] = ui_DigitalInput;

                   /******************************************************/
                   /*Start the conversion of the calibration offset value*/
                   /******************************************************/
                   i_APCI3200_ReadCalibrationOffsetValue(dev,&ui_DummyValue);
                  }//if (i_CalibrationFlag == 0)
                /**********************************************************/
                /* Test if the value read is the calibration offset value */
                /**********************************************************/

               if (i_CalibrationFlag == 1)
                 {
                
                 /******************/
                 /* Save the value */
                 /******************/
              
                 ui_InterruptChannelValue[i_Count + 1] = ui_DigitalInput;

                 /******************************************************/
                 /* Start the conversion of the calibration gain value */
                 /******************************************************/
                 i_APCI3200_ReadCalibrationGainValue(dev,&ui_DummyValue);
                 }//if (i_CalibrationFlag == 1)
              /******************************************************/
              /*Test if the value read is the calibration gain value*/
              /******************************************************/
            
              if (i_CalibrationFlag == 2)
                {
              
                /****************/
                /*Save the value*/
                /****************/ 
                ui_InterruptChannelValue[i_Count + 2] = ui_DigitalInput;
                if(i_ScanType==1)
                   { 

                   i_InterruptFlag=0;
                   i_Count=i_Count + 6;
                   }//if(i_ScanType==1)
                else
                  {
                  i_Count=0;
                  }//elseif(i_ScanType==1)
                if(i_ScanType!=1)
                  { 

                  send_sig(SIGIO,devpriv->tsk_Current,0); // send signal to the sample
                  }//if(i_ScanType!=1)
                else
                  {
                  if(i_ChannelCount==i_Sum)
                    {

                    send_sig(SIGIO,devpriv->tsk_Current,0); // send signal to the sample               
                    }
                  }//if(i_ScanType!=1)
                }//if (i_CalibrationFlag == 2)
               }// if ((ui_StatusRegister & 0x2) == 0x2)  
               break;
           
          case 2: 
                  /************************************/
                  /*Read the interrupt status register*/
                  /************************************/

                 ui_StatusRegister = inl(devpriv->iobase+i_Offset + 16);
                 /*************************/
                 /*Test if interrupt occur*/
                 /*************************/
     

                if ((ui_StatusRegister & 0x2) == 0x2)
                  {
           
                  i_CJCFlag = ((inl(devpriv->iobase+i_Offset + 4) & 0x00000400) >> 10);

                  i_CalibrationFlag = ((inl(devpriv->iobase+i_Offset + 12) & 0x00060000) >> 17);

                /*************************/
                /*Read the channel number*/
                /*************************/
           
                   ui_ChannelNumber = inl(devpriv->iobase+i_Offset + 24);

           
                   /************************************/
                   /*Read the digital temperature value*/
                   /************************************/
                   ui_DigitalTemperature = inl(devpriv->iobase+i_Offset + 28);

           

                   /*********************************************/
                   /*Test if the value read is the channel value*/
                   /*********************************************/
           

                 if ((i_CalibrationFlag == 0) && (i_CJCFlag == 0))
                    {
                     ui_InterruptChannelValue[i_Count + 0]=ui_DigitalTemperature;

                    /*********************************/
                    /*Start the conversion of the CJC*/
                    /*********************************/
                    i_APCI3200_ReadCJCValue(dev,&ui_DummyValue);

                    }//if ((i_CalibrationFlag == 0) && (i_CJCFlag == 0))
           

                 /*****************************************/
                 /*Test if the value read is the CJC value*/
                 /*****************************************/
           
                if ((i_CJCFlag == 1) && (i_CalibrationFlag == 0))
                   {
                   ui_InterruptChannelValue[i_Count + 3]=ui_DigitalTemperature;

              
                /******************************************************/
                /*Start the conversion of the calibration offset value*/
                /******************************************************/
                   i_APCI3200_ReadCalibrationOffsetValue(dev,&ui_DummyValue);
                   }// if ((i_CJCFlag == 1) && (i_CalibrationFlag == 0))
           

                 /********************************************************/
                 /*Test if the value read is the calibration offset value*/
                 /********************************************************/
           
                 if ((i_CalibrationFlag == 1) && (i_CJCFlag == 0))
                    {
                    ui_InterruptChannelValue[i_Count + 1]=ui_DigitalTemperature;

                 /****************************************************/
                 /*Start the conversion of the calibration gain value*/
                 /****************************************************/
                  i_APCI3200_ReadCalibrationGainValue(dev,&ui_DummyValue);

                     }//if ((i_CalibrationFlag == 1) && (i_CJCFlag == 0))
           

                    /******************************************************/
                    /*Test if the value read is the calibration gain value*/
                    /******************************************************/
           

                if ((i_CalibrationFlag == 2) && (i_CJCFlag == 0))
                    {
                    ui_InterruptChannelValue[i_Count + 2]=ui_DigitalTemperature;

                   /**********************************************************/
                   /*Test if the Calibration channel must be read for the CJC*/
                   /**********************************************************/
              
                    /*Test if the polarity is the same*/
                    /**********************************/                                     
                    if(i_CJCPolarity!=i_ADDIDATAPolarity)
                       {
                       i_ConvertCJCCalibration=1;
                       }//if(i_CJCPolarity!=i_ADDIDATAPolarity)
                    else 
                       {
                       if(i_CJCGain==i_ADDIDATAGain) 
                          {
                          i_ConvertCJCCalibration=0;
                          }//if(i_CJCGain==i_ADDIDATAGain)
                       else
                          {
                           i_ConvertCJCCalibration=1;
                          }//elseif(i_CJCGain==i_ADDIDATAGain)
                        }//elseif(i_CJCPolarity!=i_ADDIDATAPolarity)          
                    if(i_ConvertCJCCalibration==1)
                       {
                       /****************************************************************/
                       /*Start the conversion of the calibration gain value for the CJC*/
                       /****************************************************************/
                       i_APCI3200_ReadCJCCalOffset(dev,&ui_DummyValue);
 
                       }//if(i_ConvertCJCCalibration==1)
                    else
                       {
                       ui_InterruptChannelValue[i_Count + 4]=0;
                       ui_InterruptChannelValue[i_Count + 5]=0;

                     }//elseif(i_ConvertCJCCalibration==1)
                  }//else if ((i_CalibrationFlag == 2) && (i_CJCFlag == 0))

                 /********************************************************************/
                 /*Test if the value read is the calibration offset value for the CJC*/
                 /********************************************************************/
           

                if ((i_CalibrationFlag == 1) && (i_CJCFlag == 1))
                  {
                  ui_InterruptChannelValue[i_Count + 4]=ui_DigitalTemperature; 

                  /****************************************************************/
                  /*Start the conversion of the calibration gain value for the CJC*/
                  /****************************************************************/
                  i_APCI3200_ReadCJCCalGain(dev,&ui_DummyValue);
          
                  }//if ((i_CalibrationFlag == 1) && (i_CJCFlag == 1))
           

               /******************************************************************/
               /*Test if the value read is the calibration gain value for the CJC*/
               /******************************************************************/
          

               if ((i_CalibrationFlag == 2) && (i_CJCFlag == 1))
                  {
                   ui_InterruptChannelValue[i_Count + 5]=ui_DigitalTemperature;  

                  if(i_ScanType==1)
                     { 

                     i_InterruptFlag=0;
                     i_Count=i_Count + 6;
                     }//if(i_ScanType==1)
                  else
                     {
                     i_Count=0;
                     }//elseif(i_ScanType==1)
                  if(i_ScanType!=1)
                    { 
                    
                    send_sig(SIGIO,devpriv->tsk_Current,0); // send signal to the sample
                    }//if(i_ScanType!=1)
                  else
                    {
                    if(i_ChannelCount==i_Sum)
                      {
                      send_sig(SIGIO,devpriv->tsk_Current,0); // send signal to the sample
                                  
                      }//if(i_ChannelCount==i_Sum)
                    }//else if(i_ScanType!=1) 
                  }//if ((i_CalibrationFlag == 2) && (i_CJCFlag == 1))

                }//else if ((ui_StatusRegister & 0x2) == 0x2)
                break;
              }//switch(i_ADDIDATAType) 
              break;
     case 2:
     case 3:
             i_APCI3200_InterruptHandleEos(dev);
             break;
           }//switch(i_ScanType)        
return;
}



/*
+----------------------------------------------------------------------------+
| Function name     :int i_APCI3200_InterruptHandleEos(comedi_device *dev)   |
|                                        									 |
|                                            						         |
+----------------------------------------------------------------------------+
| Task              : .                   |
|                     This function copies the acquired data(from FIFO)      |
|				to Comedi buffer.		 							 |
|                     										                 |
+----------------------------------------------------------------------------+
| Input Parameters  : comedi_device *dev									 |
|                     														 |
|                                                 					         |
+----------------------------------------------------------------------------+
| Return Value      : 0            					                         |
|                    													     |
+----------------------------------------------------------------------------+
*/
int i_APCI3200_InterruptHandleEos(comedi_device *dev)
{  
       UINT ui_StatusRegister=0;
       UINT ui_ChannelNumber = 0;
       UINT *data;
       comedi_subdevice *s=dev->subdevices+0;
       comedi_async *async = s->async;
        data=async->data+async->buf_int_ptr;//new samples added from here onwards
       
           /************************************/
           /*Read the interrupt status register*/
           /************************************/
           ui_StatusRegister = inl(devpriv->iobase+i_Offset + 16);
           /*************************/
           /*Test if interrupt occur*/
           /*************************/
        
          if ((ui_StatusRegister & 0x2) == 0x2)
            {
           /*************************/
           /*Read the channel number*/
           /*************************/
            ui_ChannelNumber = inl(devpriv->iobase+i_Offset + 24);

           /*************************************/
           /*Read the digital Analog Input value*/
           /*************************************/

           data[i_Count] = inl(devpriv->iobase+i_Offset + 28);
//           printk("\ndata[%d]=%x\n",i_Count,data[i_Count]);
            if((i_Count == (i_LastChannel-i_FirstChannel+3)))
             {
               i_Count=-1; 
             async->buf_int_count+=(i_LastChannel-i_FirstChannel+4)*sizeof(UINT);
             async->buf_int_ptr+=(i_LastChannel-i_FirstChannel+4)*sizeof(UINT);
             comedi_eos(dev,s);
             if (s->async->buf_int_ptr>=s->async->data_len) //  for buffer rool over  
              { 
	        /* buffer rollover */
	        s->async->buf_int_ptr=0;
		comedi_eobuf(dev,s);
              }
            }  
           i_Count++;
            }
        i_InterruptFlag=0;
 	return 0;
}  


       
