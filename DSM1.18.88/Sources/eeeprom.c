#include "includes.h"


/******************************************************************************/
void ErrorCheck(volatile ErrType status, volatile unsigned char statbits,
            volatile unsigned char ferstatbits)					
{
  //Compare the copied FSTAT and FERSTAT register results against the selected
  //error bits. A match indicates an error and loops forever.
  if((status.fstat_var & statbits) || (status.ferfstat_var & ferstatbits))
  {
    while(1); //Wait forever.
  }
}
/******************************************************************************/
void ReportError(unsigned char error_code)					
{
  //User reported errors are sent here.
  while(1);
}

/******************************************************************************
Function  : LaunchFlashCommand
Engineer  :	Matt Grant
Date      : 02/18/2008
Arguments : The following parameters are used as specified in the command list of the Flash block guide.
          :	params: Indicates # of word parameters used from ccob1 - ccob7, if any.
          : ccob0high: Indicates the high byte command portion of the ccob0 word.
          : ccob0low: Forms the low byte of the ccob0 word parameter, if needed.
          : ccob1: First word parameter, if needed.
          : ccob2: Second word parameter, if needed.
          : ccob3: Third word parameter, if needed.
          : ccob4: Fourth word parameter, if needed.
          : ccob5: Fifth word parameter, if needed.
          : ccob6: Sixth word parameter, if needed.
          : ccob7: Seventh word parameter, if needed.
Return    : Flash module status registers
Notes     : Clears any error flags and uses parameters to initiate an FTM command.
            Does not check if the Flash is erased, and does not explicitely
            verify that data has been sucessfully programmed.
******************************************************************************/
//unsigned char LaunchFlashCommand(char params, unsigned char command, unsigned char ccob0,
ErrType LaunchFlashCommand(char params, unsigned char ccob0high, unsigned char ccob0low, unsigned int ccob1,
unsigned int ccob2,unsigned int ccob3,unsigned int ccob4,unsigned int ccob5, unsigned int ccob6, unsigned int ccob7)
{
unsigned char temp;                 //Temporary variable.
volatile ErrType status;            //Used to copy and store Flash status registers.

  temp = FCLKDIV;                   //Read the FCLKDIV register
  if((temp & 0x80) != 0x80)         //If FDIVLD not set, then write FCLKDIV.
  {
    FCLKDIV = FCLK_DIV;             //Write FCLKDIV before launching FTM command.
    if(FCLKDIV != (FCLK_DIV | 0x80))//Check to make sure value is written.
      ReportError(0);
  }
  while(FSTAT_CCIF != 1);           //Wait a while for CCIF to be set
  if(FSTAT_CCIF == 1)               //Ensure CCIF set before using the FTM module.
	{																	
	  FSTAT = 0x30;                   //Use store instruction to clear ACCERR, FPVIOL.
    /**********SET UP THE FTM COMMAND AND PARAMETERS***********/
    FCCOBIX = 0;                    //Set CCOB index to 0 to begin command setup.
    FCCOBHI = ccob0high;            //Write ccob0 high-byte command value.
    FCCOBLO = ccob0low;             //Write ccob0 low-byte parameter, if used.
    if (params > 0)                 //Continue if more parameters to specify.
    {
      FCCOBIX++;
      FCCOB = ccob1;                //Write next word parameter to CCOB1 buffer.
      if (params > 1)               //Continue if more parameters to specify.
      {
        FCCOBIX++;
        FCCOB = ccob2;              //Write next word parameter to CCOB2 buffer.
        if (params > 2)             //Continue if more parameters to specify.
        {
          FCCOBIX++;
          FCCOB = ccob3;            //Write next word parameter to CCOB3 buffer.
          if (params > 3)           //Continue if more parameters to specify.
          {
            FCCOBIX++;
            FCCOB = ccob4;          //Write next word parameter to CCOB4 buffer.
            if (params > 4)         //Continue if more parameters to specify.
            {
              FCCOBIX++;
              FCCOB = ccob5;        //Write next word parameter to CCOB5 buffer.
              if (params > 5)       //Continue if more parameters to specify.
              {
                FCCOBIX++;
                FCCOB = ccob6;      //Write next word parameter to CCOB6 buffer.
                if (params > 6)     //Continue if more parameters to specify.
                {
                  FCCOBIX++;
                  FCCOB = ccob7;    //Write next word parameter to CCOB7 buffer.
                }
              }
            }
          }
        }
      }
    }
    /**********************************************************/
    FSTAT = 0x80;                   //Clear buffer-empty-flag to start FTM command.
    while (!FSTAT_CCIF);            //Now wait for the FTM command to complete.
    status.fstat_var = FSTAT;       //Copy FSTAT register state for later comparison.
    status.ferfstat_var = FERSTAT;  //Copy FERSTAT register state for later comparison.
    return(status);                 //After FTM command completed, return status.
	} 
	else 
    ReportError(COMMAND_BUSY);      //FTM is currently busy.
}

//................................................
void Eeeprom_intial(void)    //inital
{
    while (!FSTAT_CCIF);               //Wait for FTM to be ready
    FCLKDIV = FCLK_DIV;               //Now write FCLKDIV before using the FTM.
    
    #ifdef FORCE_PARTITON_FOR_DEBUG     //If defined, force EEE partition and eraser.
        //Run LaunchFlashCommand to partition and erase the EEE in debug mode.
        //Return the status of the FSTAT and FERSTAT Flash registers.
        LaunchFlashCommand(2 ,FULL_PARTITION_D_FLASH, 0, DFPART, EEE_RAM, 0, 0, 0, 0, 0);
        //This enables the FTM to take any revised data written to the EEE
        LaunchFlashCommand(0 ,ENABLE_EEPROM_EMULATION, 0, 0, 0, 0, 0, 0, 0, 0);         
        //inital bootcount
        BootCount=0; 
    #else
        //This enables the FTM to take any revised data written to the EEE
        LaunchFlashCommand(0 ,ENABLE_EEPROM_EMULATION, 0, 0, 0, 0, 0, 0, 0, 0);                                                                  
    #endif
    
}

void PartionEEPROM(void)
{
    LaunchFlashCommand(2 ,FULL_PARTITION_D_FLASH, 0, DFPART, EEE_RAM, 0, 0, 0, 0, 0);
    //This enables the FTM to take any revised data written to the EEE
    LaunchFlashCommand(0 ,ENABLE_EEPROM_EMULATION, 0, 0, 0, 0, 0, 0, 0, 0);             
}
