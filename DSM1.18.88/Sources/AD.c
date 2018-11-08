#include "includes.h"


void AD_Init(void)
{ 

/* Initialise the ATD to application requirements */  
  
   
    ATD0DIEN = 0x00;
    ATD0CTL0 = 0x04;            //Wrap around channel
    
    ATD0CTL1 = 0xD2;            // Set resolution , discharge, external triger2
        
    //ATD0CTL3 = 0xD0;              //每个序列10次转换
    ATD0CTL3 = 0x81;            // right justify ,1 conversion per sequence
    
    ATD0CTL4 = 0x05;             //Sample time    
    
    ATD0CTL2 = ATD0CTL2_AFFC_MASK|ATD0CTL2_ETRIGE_MASK|ATD0CTL2_ASCIE_MASK;            //为A/D转换CCF自动清0位 ,Enable external trigger ,enable Interrupt
    ATD0CTL5 = 0x10;
    
}

float ReadVoltage()
{

    word temp=SumFilter[0]>>6;
    return (float)temp*3.3*16.0/4096.0+0.7;    
}

float ReadCurrent()
{            
    word temp=SumFilter[2]>>6;
    return (float)temp*3.3*500/4096.0;        
  
}

float ReadInnerPre()
{
    word temp=SumFilter[3]>>9;
    //word temp=ADCvalue(3);     
    //return((float)temp*3.3/4096.0-2.048)*1000.0/50.0*(eepromDat.CRP2.P1_k)-eepromDat.CRP2.P1_b; 
    //float tempFloat=(float)temp*3.3/4096.0*1000.0/300.394*(eepromDat.CRP2.P1_k)+eepromDat.CRP2.P1_b;
    float tempFloat=(float)temp*eepromDat.CRP2.P1_k+eepromDat.CRP2.P1_b;
    if(tempFloat<=1)
    {
        tempFloat=1;
    }
    return tempFloat;
    
}

float ReadannPre()
{
    //word temp=ADCvalue(4);
    word temp=SumFilter[4]>>9;
    //return((float)temp*3.3/4096.0-2.048)*1000.0/50.0*(eepromDat.CRP2.P2_k)-eepromDat.CRP2.P2_b;
   
    //float tempFloat=(float)temp*3.3/4096.0*1000.0/300.394*(eepromDat.CRP2.P2_k)+eepromDat.CRP2.P2_b;
    float tempFloat=(float)temp*eepromDat.CRP2.P2_k+eepromDat.CRP2.P2_b;
    if(tempFloat<=1)
    {
        tempFloat=1;
    }
    return tempFloat;    
    
}
