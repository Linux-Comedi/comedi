 #include "hwdrv_APCI1710.h"
 #include "APCI1710_Inp_cpt.c" 
 #include "APCI1710_Ssi.c"
 #include "APCI1710_Tor.c"
 #include "APCI1710_Ttl.c"
 #include "APCI1710_Dig_io.c" 
 #include "APCI1710_82x54.c"
 #include "APCI1710_Chrono.c"
 #include "APCI1710_Pwm.c"
 #include "APCI1710_INCCPT.c"

int i_APCI1710_Reset(comedi_device *dev);
VOID v_APCI1710_Interrupt(int irq, void *d, struct pt_regs *regs) ;
//for 1710

 int i_APCI1710_Reset(comedi_device *dev)
{
    int ret;
    DWORD dw_Dummy;	
  
 	/*********************************/ 
	/* Read all module configuration */ 
	/*********************************/ 
        ret=inl(devpriv->s_BoardInfos.ui_Address+60);
	devpriv->s_BoardInfos.dw_MolduleConfiguration [0]=ret;
	     
        ret=inl(devpriv->s_BoardInfos.ui_Address+124);
	devpriv->s_BoardInfos.dw_MolduleConfiguration [1]=ret;
	     
        ret=inl(devpriv->s_BoardInfos.ui_Address+188);
	devpriv->s_BoardInfos.dw_MolduleConfiguration [2]=ret;
	     
        ret=inl(devpriv->s_BoardInfos.ui_Address+252);
	devpriv->s_BoardInfos.dw_MolduleConfiguration [3]=ret;

	// outl(0x80808082,devpriv->s_BoardInfos.ui_Address+0x60);
	outl(0x83838383,devpriv->s_BoardInfos.ui_Address+0x60);  

    	// Enable the interrupt for the controler 	
	dw_Dummy =  inl(devpriv->s_BoardInfos.ui_Address+ 0x38);
    	outl(dw_Dummy | 0x2000,devpriv->s_BoardInfos.ui_Address+0x38);

	return 0;
}


/*
+----------------------------------------------------------------------------+
| Function's Name   : __VOID__ v_APCI1710_InterruptFunction                  |
|				(BYTE b_Interrupt, __CPPARGS)                |
+----------------------------------------------------------------------------+
| Task              : APCI-1710 interrupt function                           |
+----------------------------------------------------------------------------+
| Input Parameters  : BYTE b_Interrupt : Interrupt number                    |
+----------------------------------------------------------------------------+
| Output Parameters : -                                                      |
+----------------------------------------------------------------------------+
| Return Value      : 0 : OK                                                 |
|                    -1 : Error                                              |
+----------------------------------------------------------------------------+
*/


	


VOID v_APCI1710_Interrupt(int irq, void *d, struct pt_regs *regs)  
{
	comedi_device *dev = d;
	comedi_subdevice *s = dev->subdevices + 0;	
	BYTE   b_ModuleCpt       = 0;
	BYTE   b_InterruptFlag   = 0;
	BYTE   b_PWMCpt          = 0;
	BYTE   b_ETMCpt          = 0;
	BYTE   b_TorCounterCpt   = 0;
	BYTE   b_PulseIncoderCpt = 0;
	UINT  ui_16BitValue;
	ULONG ul_InterruptLatchReg;
	ULONG ul_LatchRegisterValue;
	ULONG ul_82X54InterruptStatus;
	ULONG ul_StatusRegister;

	str_ModuleInfo  * ps_ModuleInfo;

        printk("APCI1710 Interrupt\n");
          for (b_ModuleCpt = 0; b_ModuleCpt < 4; b_ModuleCpt ++, ps_ModuleInfo ++)
             {

		 /**************************/
		 /* 1199/0225 to 0100/0226 */
		 /**************************/
		 ps_ModuleInfo = &devpriv->s_ModuleInfo [b_ModuleCpt];


                 /***********************/
		 /* Test if 82X54 timer */
		 /***********************/

		 if ((devpriv->
		      s_BoardInfos.
		      dw_MolduleConfiguration [b_ModuleCpt] & 0xFFFF0000UL) == APCI1710_82X54_TIMER)
		    {
		    
                     //printk("TIMER Interrupt Occurred\n");
                     ul_82X54InterruptStatus= inl(devpriv->s_BoardInfos.
			       ui_Address + 12 + (64 * b_ModuleCpt));

		    /***************************/
		    /* Test if interrupt occur */
		    /***************************/

		    if ((ul_82X54InterruptStatus & ps_ModuleInfo->
						   s_82X54ModuleInfo.
						   b_InterruptMask) != 0)
		       {
		       devpriv->
		       s_InterruptParameters.
		       s_FIFOInterruptParameters [devpriv->
						  s_InterruptParameters.
						  ui_Write].
		       ul_OldInterruptMask = (ul_82X54InterruptStatus & ps_ModuleInfo->
									s_82X54ModuleInfo.
									b_InterruptMask) << 4;

		       devpriv->
		       s_InterruptParameters.
		       s_FIFOInterruptParameters [devpriv->
						  s_InterruptParameters.
						  ui_Write].
		       b_OldModuleMask = 1 << b_ModuleCpt;

		       devpriv->
		       s_InterruptParameters.
		       s_FIFOInterruptParameters [devpriv->
						  s_InterruptParameters.
						  ui_Write].
		       ul_OldCounterLatchValue = 0;

		       devpriv->
		       s_InterruptParameters.
		       ul_InterruptOccur ++;

		       /****************************/
		       /* Increment the write FIFO */
		       /****************************/

		       devpriv->
		       s_InterruptParameters.
		       ui_Write = (devpriv->
				   s_InterruptParameters.
				   ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

		       b_InterruptFlag = 1;

		       
			    
			     /**********************/
			     /* Call user function */
			     /**********************/
                         //Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 
			    
		       } // if ((ul_82X54InterruptStatus & 0x7) != 0)
		    } // 82X54 timer





		 /***************************/
		 /* Test if increm. counter */
		 /***************************/

		 if ((devpriv->
		      s_BoardInfos.
		      dw_MolduleConfiguration [b_ModuleCpt] & 0xFFFF0000UL) == APCI1710_INCREMENTAL_COUNTER)
		    {

			 ul_InterruptLatchReg=inl(devpriv->s_BoardInfos.
			       ui_Address + (64 * b_ModuleCpt));

		    /*********************/
		    /* Test if interrupt */
		    /*********************/

		    if ((ul_InterruptLatchReg & 0x22) && (ps_ModuleInfo->
							  s_SiemensCounterInfo.
							  s_ModeRegister.
							  s_ByteModeRegister.
							  b_ModeRegister2 & 0x80))
		       {
		       /************************************/
		       /* Test if strobe latch I interrupt */
		       /************************************/

		       if (ul_InterruptLatchReg & 2)
			  {
				ul_LatchRegisterValue=inl(devpriv->s_BoardInfos.
				     ui_Address + 4 + (64 * b_ModuleCpt));

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldInterruptMask = 1UL;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  b_OldModuleMask = 1 << b_ModuleCpt;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldCounterLatchValue = ul_LatchRegisterValue;

			  devpriv->
			  s_InterruptParameters.
			  ul_InterruptOccur ++;

			  /****************************/
			  /* 0899/0224 to 1199/0225   */
			  /****************************/
			  /* Increment the write FIFO */
                      /****************************/

			  devpriv->
			  s_InterruptParameters.
			  ui_Write = (devpriv->
				      s_InterruptParameters.
				      ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

			  b_InterruptFlag = 1;

			 
			     
				/**********************/
				/* Call user function */
				/**********************/
                        //Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 
				
			 
			  }

		       /*************************************/
		       /* Test if strobe latch II interrupt */
		       /*************************************/

		       if (ul_InterruptLatchReg & 0x20)
			  {
			  
			  ul_LatchRegisterValue=inl(devpriv->s_BoardInfos.
				     ui_Address + 8 + (64 * b_ModuleCpt));

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldInterruptMask = 2UL;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  b_OldModuleMask = 1 << b_ModuleCpt;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldCounterLatchValue = ul_LatchRegisterValue;

			  devpriv->
			  s_InterruptParameters.
			  ul_InterruptOccur ++;

			  /****************************/
			  /* 0899/0224 to 1199/0225   */
			  /****************************/
			  /* Increment the write FIFO */
			  /****************************/

			  devpriv->
			  s_InterruptParameters.
			  ui_Write = (devpriv->
				      s_InterruptParameters.
				      ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

			  b_InterruptFlag = 1;

			  
			    /**********************/
				/* Call user function */
				/**********************/
                        //Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 
				
			  	  }
		       }

		  
			ul_InterruptLatchReg=inl(devpriv-> s_BoardInfos.
			       ui_Address + 24 + (64 * b_ModuleCpt));

		    /***************************/
		    /* Test if index interrupt */
		    /***************************/

		    if (ul_InterruptLatchReg & 0x8)
		       {
		       ps_ModuleInfo->
		       s_SiemensCounterInfo.
		       s_InitFlag.
		       b_IndexInterruptOccur = 1;

		       if (ps_ModuleInfo->
			   s_SiemensCounterInfo.
			   s_ModeRegister.
			   s_ByteModeRegister.
			   b_ModeRegister2 & APCI1710_INDEX_AUTO_MODE)
			  {

                       outl(ps_ModuleInfo->
				      s_SiemensCounterInfo.
				      s_ModeRegister.
				      dw_ModeRegister1_2_3_4,devpriv->
				      s_BoardInfos.
				      ui_Address + 20 + (64 * b_ModuleCpt));
			  }

		       /*****************************/
		       /* Test if interrupt enabled */
		       /*****************************/

		       if ((ps_ModuleInfo->
			    s_SiemensCounterInfo.
			    s_ModeRegister.
			    s_ByteModeRegister.
			    b_ModeRegister3 & APCI1710_ENABLE_INDEX_INT) == APCI1710_ENABLE_INDEX_INT)
			  {
			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldInterruptMask = 4UL;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  b_OldModuleMask = 1 << b_ModuleCpt;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldCounterLatchValue = ul_LatchRegisterValue;

			  devpriv->
			  s_InterruptParameters.
			  ul_InterruptOccur ++;

			  /****************************/
			  /* 0899/0224 to 1199/0225   */
			  /****************************/
			  /* Increment the write FIFO */
			  /****************************/

			  devpriv->
			  s_InterruptParameters.
			  ui_Write = (devpriv->
				      s_InterruptParameters.
				      ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

			  b_InterruptFlag = 1;

			  
			    
				/**********************/
				/* Call user function */
				/**********************/
                        //Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 
				
				}
		       }

		    /*****************************/
		    /* Test if compare interrupt */
		    /*****************************/

		    if (ul_InterruptLatchReg & 0x10)
		       {
		       /*****************************/
		       /* Test if interrupt enabled */
		       /*****************************/

		       if ((ps_ModuleInfo->
			    s_SiemensCounterInfo.
			    s_ModeRegister.
			    s_ByteModeRegister.
			    b_ModeRegister3 & APCI1710_ENABLE_COMPARE_INT) == APCI1710_ENABLE_COMPARE_INT)
			  {
			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldInterruptMask = 8UL;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  b_OldModuleMask = 1 << b_ModuleCpt;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldCounterLatchValue = ul_LatchRegisterValue;

			  devpriv->
			  s_InterruptParameters.
			  ul_InterruptOccur ++;

			  /****************************/
			  /* 0899/0224 to 1199/0225   */
			  /****************************/
			  /* Increment the write FIFO */
                      /****************************/

			  devpriv->
			  s_InterruptParameters.
			  ui_Write = (devpriv->
				      s_InterruptParameters.
				      ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

			  b_InterruptFlag = 1;

			  
			    
				/**********************/
				/* Call user function */
				/**********************/
                        //Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 
				
			  
			  }
		       }

		    /*******************************************/
		    /* Test if frequency measurement interrupt */
		    /*******************************************/

		    if (ul_InterruptLatchReg & 0x20)
		       {
		       /*******************/
		       /* Read the status */
		       /*******************/

                         ul_StatusRegister=inl(devpriv->s_BoardInfos.
				  ui_Address + 32 + (64 * b_ModuleCpt));

		       /******************/
		       /* Read the value */
		       /******************/


                         ul_LatchRegisterValue=inl(devpriv->s_BoardInfos.
				  ui_Address + 28 + (64 * b_ModuleCpt));

		       switch ((ul_StatusRegister >> 1) & 3)
			  {
			  case 0:
			       /*************************/
			       /* Test the counter mode */
			       /*************************/

			       if ((devpriv->
				    s_ModuleInfo [b_ModuleCpt].
				    s_SiemensCounterInfo.
				    s_ModeRegister.
				    s_ByteModeRegister.
				    b_ModeRegister1 & APCI1710_16BIT_COUNTER) == APCI1710_16BIT_COUNTER)
				  {
				  /****************************************/
				  /* Test if 16-bit counter 1 pulse occur */
				  /****************************************/

				  if ((ul_LatchRegisterValue & 0xFFFFU) != 0)
				     {
				     ui_16BitValue         = (UINT) ul_LatchRegisterValue & 0xFFFFU;
				     ul_LatchRegisterValue = (ul_LatchRegisterValue & 0xFFFF0000UL) | (0xFFFFU - ui_16BitValue);
				     }

				  /****************************************/
				  /* Test if 16-bit counter 2 pulse occur */
				  /****************************************/

				  if ((ul_LatchRegisterValue & 0xFFFF0000UL) != 0)
				     {
				     ui_16BitValue         = (UINT) ((ul_LatchRegisterValue >> 16) & 0xFFFFU);
				     ul_LatchRegisterValue = (ul_LatchRegisterValue & 0xFFFFUL) | ((0xFFFFU - ui_16BitValue) << 16);
				     }
				  }
			       else
				  {
				  if (ul_LatchRegisterValue != 0)
				     {
				     ul_LatchRegisterValue = 0xFFFFFFFFUL - ul_LatchRegisterValue;
				     }
				  }
			       break;

			  case 1:
			       /****************************************/
			       /* Test if 16-bit counter 2 pulse occur */
			       /****************************************/

			       if ((ul_LatchRegisterValue & 0xFFFF0000UL) != 0)
				  {
				  ui_16BitValue         = (UINT) ((ul_LatchRegisterValue >> 16) & 0xFFFFU);
				  ul_LatchRegisterValue = (ul_LatchRegisterValue & 0xFFFFUL) | ((0xFFFFU - ui_16BitValue) << 16);
				  }
			       break;

			  case 2:
			       /****************************************/
			       /* Test if 16-bit counter 1 pulse occur */
			       /****************************************/

			       if ((ul_LatchRegisterValue & 0xFFFFU) != 0)
				  {
				  ui_16BitValue         = (UINT) ul_LatchRegisterValue & 0xFFFFU;
				  ul_LatchRegisterValue = (ul_LatchRegisterValue & 0xFFFF0000UL) | (0xFFFFU - ui_16BitValue);
				  }
			       break;
			  }

		       devpriv->
		       s_InterruptParameters.
		       s_FIFOInterruptParameters [devpriv->
						  s_InterruptParameters.
						  ui_Write].
		       ul_OldInterruptMask = 0x10000UL;

		       devpriv->
		       s_InterruptParameters.
		       s_FIFOInterruptParameters [devpriv->
						  s_InterruptParameters.
						  ui_Write].
		       b_OldModuleMask = 1 << b_ModuleCpt;

		       devpriv->
		       s_InterruptParameters.
		       s_FIFOInterruptParameters [devpriv->
						  s_InterruptParameters.
						  ui_Write].
		       ul_OldCounterLatchValue = ul_LatchRegisterValue;

		       devpriv->
		       s_InterruptParameters.
		       ul_InterruptOccur ++;

		       /****************************/
		       /* 0899/0224 to 1199/0225   */
		       /****************************/
		       /* Increment the write FIFO */
		       /****************************/

		       devpriv->
		       s_InterruptParameters.
		       ui_Write = (devpriv->
				   s_InterruptParameters.
				   ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

		       b_InterruptFlag = 1;

		       
			    
			     /**********************/
			     /* Call user function */
			     /**********************/
                        //Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 
			    

		       
		       }
		    } // Incremental counter






 		 /***************/
		 /* Test if CDA */
		 /***************/

		 if ((devpriv->
		      s_BoardInfos.
		      dw_MolduleConfiguration [b_ModuleCpt] & 0xFFFF0000UL) == APCI1710_CDA)
		    {
		    /******************************************/
		    /* Test if CDA enable and functionality 0 */
		    /******************************************/

		    if ((devpriv->
			 s_ModuleInfo [b_ModuleCpt].
			 s_CDAModuleInfo.
			 b_CDAEnable == APCI1710_ENABLE) && (devpriv->
							     s_ModuleInfo [b_ModuleCpt].
							     s_CDAModuleInfo.
							     b_FctSelection == 0))
		       {
		       /****************************/
		       /* Get the interrupt status */
    		       /****************************/


                       ul_StatusRegister=inl(devpriv->s_BoardInfos.
				  ui_Address + 16 + (64 * b_ModuleCpt));
		       /***************************/
		       /* Test if interrupt occur */
		       /***************************/

		       if (ul_StatusRegister & 1)
			  {
			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldInterruptMask = 0x80000UL;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  b_OldModuleMask = 1 << b_ModuleCpt;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldCounterLatchValue = 0;

			  devpriv->
			  s_InterruptParameters.
			  ul_InterruptOccur ++;

			  
			  /****************************/
			  /* Increment the write FIFO */
			  /****************************/

			  devpriv->
			  s_InterruptParameters.
			  ui_Write = (devpriv->
				      s_InterruptParameters.
				      ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

			  b_InterruptFlag = 1;

			  
			   
				/**********************/
				/* Call user function */
				/**********************/

                		//Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 		

			  } // if (ul_StatusRegister & 1)
		      
			 }
		    } // CDA


		 /***********************/
		 /* Test if PWM counter */
		 /***********************/

		 if ((devpriv->
		      s_BoardInfos.
		      dw_MolduleConfiguration [b_ModuleCpt] & 0xFFFF0000UL) == APCI1710_PWM)
		    {
		    for (b_PWMCpt = 0; b_PWMCpt < 2; b_PWMCpt ++)
		       {
		       /*************************************/
		       /* Test if PWM interrupt initialised */
		       /*************************************/

		       if (devpriv->
			   s_ModuleInfo [b_ModuleCpt].
			   s_PWMModuleInfo.
			   s_PWMInfo [b_PWMCpt].
			   b_InterruptEnable == APCI1710_ENABLE)
			  {
			  /*****************************/
			  /* Read the interrupt status */
			  /*****************************/

                       ul_StatusRegister=inl(devpriv->s_BoardInfos.
				     ui_Address + 16 + (20 * b_PWMCpt) + (64 * b_ModuleCpt));


			  /***************************/
			  /* Test if interrupt occur */
			  /***************************/

			  if (ul_StatusRegister & 0x1)
			     {
			     devpriv->
			     s_InterruptParameters.
			     s_FIFOInterruptParameters [devpriv->
							s_InterruptParameters.
							ui_Write].
			     ul_OldInterruptMask = 0x4000UL << b_PWMCpt;

			     devpriv->
			     s_InterruptParameters.
			     s_FIFOInterruptParameters [devpriv->
							s_InterruptParameters.
							ui_Write].
			     b_OldModuleMask = 1 << b_ModuleCpt;

			     devpriv->
			     s_InterruptParameters.
			     ul_InterruptOccur ++;

			     /****************************/
			     /* Increment the write FIFO */
			     /****************************/

			     devpriv->
			     s_InterruptParameters.
			     ui_Write = (devpriv->
					 s_InterruptParameters.
					 ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

			     b_InterruptFlag = 1;

			     
				   /**********************/
				   /* Call user function */
				   /**********************/
				//Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 

			     
			     } // if (ul_StatusRegister & 0x1)
			  } // if (APCI1710_ENABLE)
		       } // for (b_PWMCpt == 0; b_PWMCpt < 0; b_PWMCpt ++)
		    } // PWM counter



		 /***********************/
		 /* Test if tor counter */
		 /***********************/

		 if ((devpriv->
		      s_BoardInfos.
		      dw_MolduleConfiguration [b_ModuleCpt] & 0xFFFF0000UL) == APCI1710_TOR_COUNTER)
		    {
		    for (b_TorCounterCpt = 0; b_TorCounterCpt < 2; b_TorCounterCpt ++)
		       {
		       /*************************************/
		       /* Test if tor interrupt initialised */
		       /*************************************/

		       if (devpriv->
			   s_ModuleInfo [b_ModuleCpt].
			   s_TorCounterModuleInfo.
			   s_TorCounterInfo [b_TorCounterCpt].
			   b_InterruptEnable == APCI1710_ENABLE)
			  {
			  /*****************************/
			  /* Read the interrupt status */
			  /*****************************/

			
				ul_StatusRegister=inl(devpriv->s_BoardInfos.
				     ui_Address + 12 + (16 * b_TorCounterCpt) + (64 * b_ModuleCpt));

			  /***************************/
			  /* Test if interrupt occur */
			  /***************************/

			  if (ul_StatusRegister & 0x1)
			     {
			     /******************************/
			     /* Read the tor counter value */
			     /******************************/

			     
                          ul_LatchRegisterValue=inl(devpriv->s_BoardInfos.
					ui_Address + 0 + (16 * b_TorCounterCpt) + (64 * b_ModuleCpt));

			     devpriv->
			     s_InterruptParameters.
			     s_FIFOInterruptParameters [devpriv->
							s_InterruptParameters.
							ui_Write].
			     ul_OldInterruptMask = 0x1000UL << b_TorCounterCpt;

			     devpriv->
			     s_InterruptParameters.
			     s_FIFOInterruptParameters [devpriv->
							s_InterruptParameters.
							ui_Write].
			     b_OldModuleMask = 1 << b_ModuleCpt;

			     devpriv->
			     s_InterruptParameters.
			     s_FIFOInterruptParameters [devpriv->
							s_InterruptParameters.
							ui_Write].
			     ul_OldCounterLatchValue = ul_LatchRegisterValue;

			     devpriv->
			     s_InterruptParameters.
			     ul_InterruptOccur ++;

			     
			     /****************************/
			     /* Increment the write FIFO */
			     /****************************/

			     devpriv->
			     s_InterruptParameters.
			     ui_Write = (devpriv->
					 s_InterruptParameters.
					 ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

			     b_InterruptFlag = 1;

	
				   /**********************/
				   /* Call user function */
				   /**********************/

			     //Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 
			     } // if (ul_StatusRegister & 0x1)
			  } // if (APCI1710_ENABLE)
		       } // for (b_TorCounterCpt == 0; b_TorCounterCpt < 0; b_TorCounterCpt ++)
		    } // Tor counter




		 /***********************/
		 /* Test if chronometer */
		 /***********************/

		 if ((devpriv->
		      s_BoardInfos.
		      dw_MolduleConfiguration [b_ModuleCpt] & 0xFFFF0000UL) == APCI1710_CHRONOMETER)
		    {
                    
         		//printk("APCI1710 Chrono Interrupt\n");
		    /*****************************/
		    /* Read the interrupt status */
		    /*****************************/

		    
                     ul_InterruptLatchReg=inl(devpriv->s_BoardInfos.
			       ui_Address + 12 + (64 * b_ModuleCpt));

		    /***************************/
		    /* Test if interrupt occur */
		    /***************************/

		    if ((ul_InterruptLatchReg & 0x8) == 0x8)
		       {
		       /****************************/
		       /* Clear the interrupt flag */
		       /****************************/

		      
 				outl(0,devpriv->s_BoardInfos.
				   ui_Address + 32 + (64 * b_ModuleCpt));

		       /***************************/
		       /* Test if continuous mode */
		       /***************************/

		       if (ps_ModuleInfo->
			   s_ChronoModuleInfo.
			   b_CycleMode == APCI1710_ENABLE)
			  {
			  /********************/
			  /* Clear the status */
			  /********************/

			  
                       outl(0,devpriv->s_BoardInfos.
				      ui_Address + 36 + (64 * b_ModuleCpt));
			  }

		       /*************************/
		       /* Read the timing value */
		       /*************************/

		       
				ul_LatchRegisterValue=inl(devpriv->s_BoardInfos.
				  ui_Address + 4 + (64 * b_ModuleCpt));

		       /*****************************/
		       /* Test if interrupt enabled */
		       /*****************************/

		       if (ps_ModuleInfo->
			   s_ChronoModuleInfo.
			   b_InterruptMask)
			  {
			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldInterruptMask = 0x80;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  b_OldModuleMask = 1 << b_ModuleCpt;

			  devpriv->
			  s_InterruptParameters.
			  s_FIFOInterruptParameters [devpriv->
						     s_InterruptParameters.
						     ui_Write].
			  ul_OldCounterLatchValue = ul_LatchRegisterValue;

			  devpriv->
			  s_InterruptParameters.
			  ul_InterruptOccur ++;

			  /****************************/
			  /* Increment the write FIFO */
                      /****************************/

			  devpriv->
			  s_InterruptParameters.
			  ui_Write = (devpriv->
				      s_InterruptParameters.
				      ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

			  b_InterruptFlag = 1;

			  
			    
				/**********************/
				/* Call user function */
				/**********************/
				//Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 
				
			  }
		   }
		 } // Chronometer



		 /*************************/
		 /* Test if pulse encoder */
		 /*************************/

		 if ((devpriv->
		      s_BoardInfos.
		      dw_MolduleConfiguration [b_ModuleCpt] & 0xFFFF0000UL) == APCI1710_PULSE_ENCODER)
		    {
		    /****************************/
		    /* Read the status register */
		    /****************************/

		    
                       ul_StatusRegister=inl(devpriv->s_BoardInfos.
			       ui_Address + 20 + (64 * b_ModuleCpt));

		    if (ul_StatusRegister & 0xF)
		       {
		       for (b_PulseIncoderCpt = 0; b_PulseIncoderCpt < 4; b_PulseIncoderCpt ++)
			  {
			  /*************************************/
			  /* Test if pulse encoder initialised */
			  /*************************************/

			  if ((ps_ModuleInfo->
			       s_PulseEncoderModuleInfo.
			       s_PulseEncoderInfo [b_PulseIncoderCpt].
			       b_PulseEncoderInit == 1) &&
			      (((ps_ModuleInfo->
				 s_PulseEncoderModuleInfo.
				 dw_SetRegister >> b_PulseIncoderCpt) & 1) == 1) &&
			      (((ul_StatusRegister >> (b_PulseIncoderCpt)) & 1) == 1))
			     {
			     devpriv->
			     s_InterruptParameters.
			     s_FIFOInterruptParameters [devpriv->
							s_InterruptParameters.
							ui_Write].
			     ul_OldInterruptMask = 0x100UL << b_PulseIncoderCpt;

			     devpriv->
			     s_InterruptParameters.
			     s_FIFOInterruptParameters [devpriv->
							s_InterruptParameters.
							ui_Write].
			     b_OldModuleMask = 1 << b_ModuleCpt;

			     devpriv->
			     s_InterruptParameters.
			     s_FIFOInterruptParameters [devpriv->
							s_InterruptParameters.
							ui_Write].
			     ul_OldCounterLatchValue = ul_LatchRegisterValue;

			     devpriv->
			     s_InterruptParameters.
			     ul_InterruptOccur ++;

			     /****************************/
			     /* 0899/0224 to 1199/0225   */
			     /****************************/
			     /* Increment the write FIFO */
			     /****************************/

			    devpriv->
			     s_InterruptParameters.
			     ui_Write = (devpriv->
					 s_InterruptParameters.
					 ui_Write + 1) % APCI1710_SAVE_INTERRUPT;

			     b_InterruptFlag = 1;
			    
				   /**********************/
				   /* Call user function */
				   /**********************/
			     //Send a signal to from kernel to user space
			     send_sig(SIGIO,devpriv->tsk_Current,0); 
				   
		 
			     }
				}
		       }
		    }//pulse encoder

	   }
  return ; 

}

