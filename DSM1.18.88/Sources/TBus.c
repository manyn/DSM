
#include "includes.h"

static BusDAT tempBusDatEvent;


// TBus Init 
void  TBus_Init(void)
{
    INT_CFADDR=128;
    INT_CFDATA7=5;
    INT_CFADDR=192;
    INT_CFDATA6=4;
    INT_CFADDR=240;
    INT_CFDATA1=1;
  
    //define pk3 as output,defualt value 0  TBus off
    DDRK_DDRK3=1;
    //define pk5 as output,defualt value 0  LBus off
    DDRK_DDRK5=1;
    
    DDRA=0; //PORTA input
    DDRB=0; //PORTB input
        
    //ESC  PK0 input
    DDRK_DDRK0=0;
    //EE   PH1 ouput 0
    DDRH_DDRH1=1; 
    PTH_PTH1=0;
    //SS   PH0 ouput 0
    DDRH_DDRH0=1; 
    PTH_PTH0=0;
    //SD   PK4 input
    DDRK_DDRK4=0;    
    //SDI  PK2 output
    DDRK_DDRK2=1;
    PORTK_PK2=0; 
    //OI   PK1 output 0
    DDRK_DDRK1=1;
    PORTK_PK1=0;
    //RST  PH2 output 0
    DDRH_DDRH2=1; 
    PTH_PTH2=0;    
    //CDS  PH3 input 
    DDRH_DDRH3=0;  
    
    //1553 r/w
    DDRE_DDRE2=1; //define pe2 as output 
    PORTE_PE2=0;  
    
    //VW
    IRQCR_IRQEN=0;   //VW disable  
    IRQCR_IRQE=1;    //falling edge
    //IRQCR_IRQEN=1; //VW enable 
    
    //TD
    PIEP_PIEP3=0;   //disable TD interrupt 
    PIFP_PIFP3=1;   //clear flag   
    PPSP_PPSP3=1;   //raising edge
    PIEP_PIEP3=1;   //enable TD interrupt   
    
    //1553 Clock
    PWME_PWME5=0;  //PWM5 disable
    PWMCNT45=0; //Reset Counter
    PWMSDN=0x80;   
    PWMDTY45=16;   //Store initial value to the duty-compare register
    PWMPER45=32;   //....to the period register
    PWMPRCLK=0;    // Set prescaler register,equal bus clock    
    PWMCLK=0x00;   //Select clock source,Clock A
    PWME_PWME5=1;  //PWM5 enable
    
    //PRE VIBR DIAM  PT5 PT6 PP1 as output
    DDRT_DDRT5=1;
    DDRT_DDRT6=1;
    DDRP_DDRP1=1;
    
    
} 


void OpenPRE(void)
{
    PTT_PTT5=1;
    Status_PRE=1;
}
void ClosePRE(void)
{
  PTT_PTT5=0;
  Status_PRE=0;

}

void OpenVIBR(void)
{
    PTT_PTT6 = 1;
    Status_VIBR = 1;
}
void CloseVIBR(void)
{
  PTT_PTT6=0;
  Status_VIBR=0;

}


void OpenDIAM(void)
{
    PTP_PTP1=1;
    Status_DIAM=1;
}
void CloseDIAM(void)
{
  PTP_PTP1=0;
  Status_DIAM=0;

}

void SendC(word CMD)
{ 
     int i;      
     PIEP_PIEP3 = 0;    //disable TD
     IRQCR_IRQEN = 0;  //VW disable
     DisableInterrupts;          
     PORTK_PK1=1;   //OI=1     
     PTH_PTH2=0;    //RST=0     
     while(1!=PORTK_PK0);//ESC 1
     PTH_PTH1=1;        //EE set 1             
     while(0!=PORTK_PK0);        //ESC 0          
     PTH_PTH0=1;        //SS set 1  CMD  
     while(1!=PORTK_PK4);        //SD  1      
     for(i=0;i<16;i++)
     {
        while(0!=PORTK_PK0);        //ESC 0 
        PORTK_PK2=(CMD>>(15-i)) & 1;  //SDI
        while(1!=PORTK_PK0);        //ESC 1
     }  
     PTH_PTH1=0;      //EE  0        
     DelayESC(30); 
     PIEP_PIEP3 = 1;    //enable TD    
     EnableInterrupts;
}

void SendD(const word DAT)
{
     int i;
     PIEP_PIEP3 = 0;    //disable TD 
     IRQCR_IRQEN = 0;  //VW disable
     
     //DisableInterrupts;             
     PORTK_PK1=1;   //OI=1     
     PTH_PTH2=0;    //RST=0     
     while(1!=PORTK_PK0);//ESC 1
     PTH_PTH1=1;        //EE set 1             
     while(0!=PORTK_PK0);        //ESC 0          
     PTH_PTH0=0;        //SS set 0  DAT  
     while(1!=PORTK_PK4);        //SD  1      
     for(i=0;i<16;i++)
     {
        while(0!=PORTK_PK0);        //ESC 0 
        PORTK_PK2=(DAT>>(15-i)) & 1;  //SDI
        while(1!=PORTK_PK0);        //ESC 1
     }  
     PTH_PTH1=0;      //EE  0        
     DelayESC(30);
     PIEP_PIEP3 = 1;    //enable TD
     //EnableInterrupts;
         
}
void SendDlist(word* DatBuf,int length)
{     
     int i,j;
     PIEP_PIEP3 = 0;    //disable TD 
     IRQCR_IRQEN = 0;  //VW disable     
     DisableInterrupts;     
     PORTK_PK1=1;   //OI=1     
     PTH_PTH2=0;    //RST=0     
     while(1!=PORTK_PK0);//ESC 1
     PTH_PTH1=1;        //EE set 1             
     while(0!=PORTK_PK0);        //ESC 0 
     for(j=0;j<length;j++)
     {
        
         PTH_PTH0=0;        //SS set 0  DAT  
         while(1!=PORTK_PK4);        //SD  1      
         for(i=0;i<16;i++)
         {
            while(0!=PORTK_PK0);        //ESC 0 
            PORTK_PK2=(DatBuf[j]>>(15-i)) & 1;  //SDI
            while(1!=PORTK_PK0);        //ESC 1
         }
         while(0!=PORTK_PK0);        //ESC 0
         while(1!=PORTK_PK0);        //ESC 1
     }
     PTH_PTH1=0;      //EE  0        
     DelayESC(30);     
     PIEP_PIEP3 = 1;    //enable TD
     EnableInterrupts;   
}

void SendCDlist(word* CDBuf,int CDLen)  //CMD:CDBuf[0]  CDLen=DatLen+1
{
     int i,j;
    // OS_CPU_SR cpu_sr;       
    // OS_ENTER_CRITICAL();
     DisableInterrupts; 
     
     //PIEP_PIEP3 = 0;    //disable TD 
     //IRQCR_IRQEN = 0;  //VW disable               
     PORTK_PK1=1;   //OI=1     
     PTH_PTH2=0;    //RST=0     
     while(1!=PORTK_PK0);//ESC 1
     PTH_PTH1=1;        //EE set 1             
     while(0!=PORTK_PK0);        //ESC 0 
     for(j=0;j<CDLen;j++)
     {
         if(0==j)
            PTH_PTH0=1;        //SS set 1  CMD
         else
            PTH_PTH0=0;        //DAT   
         while(1!=PORTK_PK4);        //SD  1      
         for(i=0;i<16;i++)
         {
            while(0!=PORTK_PK0);        //ESC 0 
            PORTK_PK2=(CDBuf[j]>>(15-i)) & 1;  //SDI
            while(1!=PORTK_PK0);        //ESC 1
         }
         while(0!=PORTK_PK0);        //ESC 0
         while(1!=PORTK_PK0);        //ESC 1        
         
     }
     PTH_PTH1=0;      //EE  0           
     DelayESC(1);
     PORTK_PK1=0;   //OI 0  
     PIEP_PIEP3 = 1;    //enable TD        
     EnableInterrupts;   
     
   //  DelayESC(2);
   //  OS_EXIT_CRITICAL();  
}
void SetFreq(word Freq)     //set TBus frequency
{
    switch (Freq)
    {
        case 40:            
            PWME_PWME5=0;   //disable PWM5
            PWMDTY45=16;   //Store initial value to the duty-compare register
            PWMPER45=32;   //....to the period register            
            PWME_PWME5=1;  //PWM5 enable
            Status_FREQ=1;
            break;
        case 80:
            PWME_PWME5=0;  //disable PWM5
            PWMDTY45=8;    //Store initial value to the duty-compare register
            PWMPER45=16;   //....to the period register            
            PWME_PWME5=1;  //PWM5 enable
            Status_FREQ=0;
            break;
    }
}

//calculate CHKS

word CalcuCHKS(word *Buffer,word Length)
{
    word i, Result;
	Result=Buffer[0];
	for(i=1;i<Length;i++)
	{
		Result^=Buffer[i];
	}
	Result^=0xFFFF;
	return Result;
} 

word CalcuCHKS_RST(word *Buffer,word Length)	//COTAS  CHKS calculate
{
	word i, Result;
	Result=Buffer[0];
	for(i=1;i<Length;i++)
	{
		Result^=Buffer[i];
	}
	return Result;
}

word CalcuCHKS_ACPR(word *Buffer,word Length)
{
    word i, Result;
    Result=0;
	for(i=0;i<Length;i++)
	{
	    Result+=Buffer[i];
	}
	return Result;
}

byte CalcuCHKS_GRO(byte *pData,int nLength) 
{
	byte crc; 
	byte i; 
	crc = 0; 
	while(nLength--) 
	{ 
		crc ^= *pData++; 
		for(i = 0;i < 8;i++) 
		{ 
			if(crc & 0x80) 
			{ 
				crc = (crc >> 1) ^ 0x31; 
			}
			else
			{ 
				crc >>= 1; 
			}
		} 
	} 
	return crc; 
}
    

void DelayNop(word Num)
{
    word i=0;
    for(i=0;i<Num;i++)
    {
        _FEED_COP();   
        asm nop;
    }
    
}

void DelayESC(word Num)
{
    word i=0;
    for(i=0;i<Num;i++)
     {
        while(0!=PORTK_PK0);        //ESC 0
        while(1!=PORTK_PK0);        //ESC 1
     }    
}

#pragma CODE_SEG __NEAR_SEG NON_BANKED 
__interrupt  void TD_ISR(void)
{  
    
    int i=0;   
    DisableInterrupts;       //disable interrupt    
    //DelayNop(20);
    tempBusDatEvent.Type=PTH_PTH3;      
    
    IRQCR_IRQEN = 1;  //VW enable
    PIFP_PIFP3=1;   //clear the flag
    EnableInterrupts;   
}

__interrupt  void VW_ISR(void)
{ 
    
    IRQCR_IRQEN = 0;  //VW disable 
    DelayNop(20);
    tempBusDatEvent.Dat=PORTAB;     
    EnQueue(&TbusQueue,tempBusDatEvent); 
          

}
