#include "includes.h"

#pragma DATA_SEG SPI_DATA
#pragma CODE_SEG SPI_CODE
//#pragma CODE_SEG DEFAULT
void InitSPI(void)
{
     /* SPI0CR1: SPIE=0,SPE=0,SPTIE=0,MSTR=0,CPOL=0,CPHA=1,SSOE=0,LSBFE=0 */
    SPI0CR1 = 4U;                        /* Reset the device register */
    (void)SPI0SR;                        /* Read the status register */
    (void)SPI0DRL;                       /* Read the device register */
    /* SPI0BR: ??=0,SPPR2=0,SPPR1=1,SPPR0=0,??=0,SPR2=0,SPR1=0,SPR0=0 */
    SPI0BR = 32U;                        /* Set the baud rate register */
    /* SPI0CR2: ??=0,XFRW=0,??=0,MODFEN=0,BIDIROE=0,??=0,SPISWAI=0,SPC0=0 */
    SPI0CR2 = 0U;                        /* Set control register 2 */
    /* SPI0CR1: SPIE=0,SPE=1,SPTIE=0,MSTR=1,CPOL=0,CPHA=0,SSOE=0,LSBFE=0 */
    SPI0CR1 = 80U;                       /* Set control register 1 */
    
    //CS1~4,tmp_CS  PT0~PT4 output  
    DDRT|=0x1F;
  //  RDRT|=0x1F;
    PTT=0xFF;
 
   
}

void CS_Enable(enum CCS chipSelect)
{    
   PTT &= (~(1<<chipSelect));   
}

void CS_Disable(enum CCS chipSelect)
{
    PTT |= (1<<chipSelect); 
    
}

//SPI read write data

byte SPI_RW_8(byte WD)
{
 //   OS_CPU_SR cpu_sr;
	byte result;
	dword count=0;
		
//	OS_ENTER_CRITICAL();    
    SPI0DRL = WD;   //send Char           
    while(SPI0SR_SPIF==0)
    {
        count++;
        if(count>10000000)
            break;
    }
    result=SPI0DRL;
    
 //   OS_EXIT_CRITICAL();    
    return result;    
}

//read temperator
float ReadTemp()    
{     
   word t;
   float tf=0.0;
   CS_Enable(TEMP); 
   t=SPI_RW_8(0);  
   t=(t<<8)+SPI_RW_8(0);   
   if(t>=0x8000)		//负温度
   {
   		t=~t+1;	
   		tf=(float)(t>>3)*0.0625;      				
   		tf=0.0-(tf);
   }
   else
   {
   		tf=(t>>3)*0.0625;       		
   }      
   CS_Disable(TEMP); 
   return tf;  
}

// Flash Operation
unsigned char ReadFSR(enum CCS Dev) //read flash status resgistor
{
    byte Result;
    word Snd;  	
	CS_Enable (Dev);
    SPI_RW_8(RDSR);
	Result=SPI_RW_8(0xFF);
	Result=SPI_RW_8(0xFF);
	CS_Disable (Dev);
	return Result;	
}
 
void WaitWIP(enum CCS Dev)  //wait WIP finished
{
    byte temp=ReadFSR(Dev);
    dword count=0;
    while(0!=(temp&1))
    {
         temp=ReadFSR(Dev);   
         asm(NOP); asm(NOP);asm(NOP); asm(NOP);
         asm(NOP); asm(NOP);asm(NOP); asm(NOP); 
         asm(NOP); asm(NOP);asm(NOP); asm(NOP);
         asm(NOP); asm(NOP);asm(NOP); asm(NOP);
         asm(NOP); asm(NOP);asm(NOP); asm(NOP); 
         count++;        
         if(count>30000)
            break;
         
         _FEED_COP(); /* feeds the dog */                        
    }
    
} 

void WriteEnable(enum CCS Dev)   //Dev Write enable
{
	byte temp;
	CS_Enable (Dev);	  
   	SPI_RW_8(WREN);
	CS_Disable (Dev);
}

void WaitWEL(enum CCS Dev)  //wait WEL finished
{
    byte temp=ReadFSR(Dev);
    dword count=0;
    while(2!=(temp&2))
    {         
         WriteEnable(Dev);         
         asm(NOP); asm(NOP);asm(NOP); asm(NOP);
         asm(NOP); asm(NOP);asm(NOP); asm(NOP); 
         asm(NOP); asm(NOP);asm(NOP); asm(NOP);
         asm(NOP); asm(NOP);asm(NOP); asm(NOP);
         asm(NOP); asm(NOP);asm(NOP); asm(NOP);
         temp=ReadFSR(Dev);
         count++;            
         if(count>30000)
            break;
    }
    
}

//Address three bytes avilable  range 0x000000~0x7FFFFF, length no restrict
void ReadFData(enum CCS Dev,dword Address,byte *Buf,word Length)
{
    
	byte temp;
	word i;

	WaitWIP(Dev);
	CS_Enable (Dev);	  
   	SPI_RW_8(READ);

	for(i=0;i<3;i++)
	{
		temp=(Address>>(2-i)*8);
		temp &=0xFF;
		SPI_RW_8(temp);
	}
	for(i=0;i<Length;i++)
	{
		Buf[i]=SPI_RW_8(0);	
	}
	CS_Disable (Dev);
}
//Address three bytes avilable  range 0x000000~0x7FFFFF
//length is no more than 0xFF-(Address &0xFF)+1
void WriteFData(enum CCS Dev,dword Address,byte *Buf,word Length)
{
	byte temp;
	word i;
//	OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL();
	if(Length>256)
	    Length=256;
	
	WaitWIP(Dev);
	WriteEnable(Dev);
	WaitWEL(Dev);

	CS_Enable (Dev);	  
   	SPI_RW_8(PP);

	for(i=0;i<3;i++)
	{
		temp=(Address>>(2-i)*8);
		temp &=0xFF;
		SPI_RW_8(temp);
	}

	for(i=0;i<Length;i++)
	{   
		SPI_RW_8(Buf[i]);			
	}
	CS_Disable (Dev);    
	WaitWIP(Dev);
//	OS_EXIT_CRITICAL();
}


void Flash_SectorErase(enum CCS Dev,dword Address)
{		
    byte temp;
	word i;
 //   OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL();
	WaitWIP(Dev);
	WriteEnable(Dev);
	WaitWEL(Dev);

	CS_Enable (Dev);	  
   	SPI_RW_8(SE);

	for(i=0;i<3;i++)
	{
		temp=(Address>>(2-i)*8);
		temp &=0x00FF;
		SPI_RW_8(temp);
	}    
	CS_Disable (Dev);
	WaitWIP(Dev);
   // OS_EXIT_CRITICAL();
}

void SectorEraseU(dword *adr)
{ 
	
	word Dev=((*adr)>>23)&3;
	dword addr=(*adr) & 0x07FFFFFF;
	Flash_SectorErase(Dev,addr);    
}
void WriteFDataU(dword *adr,byte *Buf,word Length)
{
    
    word Dev=((*adr)>>23)&3;
	dword addr=(*adr) & 0x07FFFFFF;
    WriteFData(Dev,addr,Buf,Length);
    *adr+=Length;    
}
void ReadFDataU(dword *adr,byte *Buf,word Length)
{
    word Dev=((*adr)>>23)&3;
	dword addr=(*adr) & 0x07FFFFFF;    
    ReadFData(Dev,addr,Buf,Length);
}

void LB_OffsetToAdr(dword LB,word Offset,dword *adr)
{
	*adr=LB;
	*adr=(*adr<<8)+Offset;
}
void SenToAdr(word SeN,dword *adr)  //SeN :0~511
{
    dword LB=SeN;
    LB=LB<<8;
    LB_OffsetToAdr(LB,0,adr);    
}

void SecPageToAdr(word Sec,word page,dword *adr)
{
    dword tempAdr=page;
    tempAdr=tempAdr<<8;
    SenToAdr(Sec,adr);
    *adr+=tempAdr;
}
word LB_ToSen(dword LB)
{
    return (LB>>8)&0xFFFF;   
}

word Adr_ToSen(dword adr)
{
    return (adr>>16);   
}

dword LookForBlkAdr(DMSTAT *dms)   //UnitLength <= 256/4
{

    byte *pb;    
    word i,j,Sum;
    word Sec,Page,Offset;
    dword adr;
    dword Result;
    word UnitLength;
    word tempWordBuf[260];
    UnitLength=dms->RecordLenth;
    
    //查找Sec
    if(dms->StarSec==dms->EndSec)
        Sec=dms->StarSec;
    else
    {   
        for(i=dms->StarSec;i<=dms->EndSec;i++)      
        {
            Sec=i;
            SenToAdr(i,&adr);
            ReadFDataU(&adr,tempWordBuf,UnitLength);
            pb=tempWordBuf;
            Sum=0;
            for(j=0;j<UnitLength;j++)
            {            
                if(*pb!=0xFF)
                    break;
                else
                {                       
                    Sum++;                
                    pb++;
                }
            }
            if(Sum==UnitLength)  //视为空白
            {
                break;
            }               
        }        
        if(Sec==dms->StarSec)  //全为空白
        {
            SenToAdr(dms->StarSec,&adr);
            Result=adr;        
    		return Result ;            
        }
    	else	
    		Sec-=1;	
    }
	//查找Sec内的Page
	for(i=1;i<256;i++)
	{
	    Page=i;
	    SecPageToAdr(Sec,i,&adr);
	    ReadFDataU(&adr,tempWordBuf,UnitLength);
        pb=tempWordBuf;
        Sum=0;
        for(j=0;j<UnitLength;j++)
        {            
            if(*pb!=0xFF)
                break;
            else
            {                   
                Sum++;                
                pb++;
            }
        }
        if(Sum==UnitLength)  //视为空白
        {
            break;
        }    
	}
	Page-=1;
	//查找Sec,Page  内偏移量
	for(i=UnitLength;i<256;i=i+UnitLength)
	{
	    Offset=i;
	    SecPageToAdr(Sec,Page,&adr);
	    adr+=i;
	    ReadFDataU(&adr,tempWordBuf,UnitLength);
        pb=tempWordBuf;
        Sum=0;
        for(j=0;j<UnitLength;j++)
        {            
            if(*pb!=0xFF)
                break;
            else
            {
                
                Sum++;                
                pb++;
            }
        }
        if(Sum==UnitLength)  //视为空白
        {
            break;
        }	    
	}
	SecPageToAdr(Sec,Page,&adr);
	Result=adr+Offset;	
	
	return Result;
		
}