#include "includes.h"


void SCIInit(void)
{ 
  /* SCI0CR1: LOOPS=0,SCISWAI=0,RSRC=0,M=0,WAKE=0,ILT=0,PE=0,PT=0 */
  SCI0CR1=0;                 
  /* SCI0SR2: AMAP=1,??=0,??=0,TXPOL=0,RXPOL=0,BRK13=0,TXDIR=0,RAF=0 */
  SCI0SR2=128;              /* Switch to the alternative register set */ 
  /* SCI0ASR1: RXEDGIF=1,??=0,??=0,??=0,??=0,BERRV=0,BERRIF=1,BKDIF=1 */
  SCI0ASR1=131U;             /* Clear alternative status flags */ 
  /* SCI0ACR1: RXEDGIE=0,??=0,??=0,??=0,??=0,??=0,BERRIE=0,BKDIE=0 */
  SCI0ACR1=0;                
  /* SCI0ACR2: ??=0,??=0,??=0,??=0,??=0,BERRM1=0,BERRM0=0,BKDFE=0 */
  SCI0ACR2=0;                
  /* SCI0SR2: AMAP=0,??=0,??=0,TXPOL=0,RXPOL=0,BRK13=0,TXDIR=0,RAF=0 */
  SCI0SR2=0;                /* Switch to the normal register set */ 
  (void) SCI0SR1;                      /* Reset interrupt request flags */
  /* SCI0CR2: TIE=0,TCIE=0,RIE=0,ILIE=0,TE=0,RE=0,RWU=0,SBK=0 */
  SCI0CR2 = 0;                        /* Disable error interrupts */
  SCI0BD = 103;                        /* Set prescaler bits */
  SCI0CR2 |= (SCI0CR2_TE_MASK | SCI0CR2_RE_MASK | SCI0CR2_RIE_MASK); /* Enable transmitter, Enable receiver, Enable receiver interrupt */
  //SCI0CR2 |= (SCI0CR2_TE_MASK | SCI0CR2_RE_MASK);
   
    
    DDRP_DDRP2=1;   //ptp2 as output
    PTP_PTP2=0;     //default receive
    
}


void SCIInitHigh(void)
{
 
  /* SCI0CR1: LOOPS=0,SCISWAI=0,RSRC=0,M=0,WAKE=0,ILT=0,PE=0,PT=0 */
  SCI0CR1=0;                 
  /* SCI0SR2: AMAP=1,??=0,??=0,TXPOL=0,RXPOL=0,BRK13=0,TXDIR=0,RAF=0 */
  SCI0SR2=128;              /* Switch to the alternative register set */ 
  /* SCI0ASR1: RXEDGIF=1,??=0,??=0,??=0,??=0,BERRV=0,BERRIF=1,BKDIF=1 */
  SCI0ASR1=131U;             /* Clear alternative status flags */ 
  /* SCI0ACR1: RXEDGIE=0,??=0,??=0,??=0,??=0,??=0,BERRIE=0,BKDIE=0 */
  SCI0ACR1=0;                
  /* SCI0ACR2: ??=0,??=0,??=0,??=0,??=0,BERRM1=0,BERRM0=0,BKDFE=0 */
  SCI0ACR2=0;                
  /* SCI0SR2: AMAP=0,??=0,??=0,TXPOL=0,RXPOL=0,BRK13=0,TXDIR=0,RAF=0 */
  SCI0SR2=0;                /* Switch to the normal register set */ 
  (void) SCI0SR1;                      /* Reset interrupt request flags */
  /* SCI0CR2: TIE=0,TCIE=0,RIE=0,ILIE=0,TE=0,RE=0,RWU=0,SBK=0 */
  SCI0CR2 = 0;                        /* Disable error interrupts */
  SCI0BD = 1;                        /* Set prescaler bits */
  //SCI0CR2 |= (SCI0CR2_TE_MASK | SCI0CR2_RE_MASK | SCI0CR2_RIE_MASK); /* Enable transmitter, Enable receiver, Enable receiver interrupt */
  SCI0CR2 |= (SCI0CR2_TE_MASK | SCI0CR2_RE_MASK);
   
    
    DDRP_DDRP2=1;   //ptp2 as output
    PTP_PTP2=0;     //default receive
    
}

void SCI_Send(byte *buffer,int lenth)
{
    int i;    
    //OS_CPU_SR cpu_sr;
    
    SCI_ClearBuf();
    
    //OS_ENTER_CRITICAL();                     
    PTP_PTP2=1;
    for(i=0;i<lenth;i++) 
    { 
        while(SCI0SR1_TC!=1);
        while(SCI0SR1_TDRE!=1);            
        SCI0DRL=buffer[i];        
    } 
    //OS_ENTER_CRITICAL();      
    while( SCI0SR1_TC!=1); 
    PTP_PTP2=0;         
} 

int SCI_Recv(byte *buffer,int lenth)
{
    int i=0;
    long count=0;
    for(i=0;i<lenth;i++)
    {
        while(!SCI0SR1_RDRF)
        {
            count++;
            if(count>0x1FFFF)
                return -1;    
        }
        buffer[i]= SCI0DRL;   
    }
    return 0;
}

void SCI_ClearBuf(void)
{
    byte temp;
    while(SCI0SR1_RDRF)
    {
        temp=SCI0DRL;
    }
}

__interrupt void SCI0_ISR(void)
{   
    byte tempByte;  
    DisableInterrupts;
    SCI_Recv(&tempByte,1);
    EnQueueByte(&LbusQueue,tempByte);
    EnableInterrupts;
}
