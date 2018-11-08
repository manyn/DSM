#ifndef _TBus_H
#define _TBus_H

/* MODULE TBus. */

  /* Including shared modules, which are used in the whole project */
  
#pragma CODE_SEG DEFAULT

__interrupt  void TD_ISR(void);
__interrupt  void VW_ISR(void);


void TBus_Init(void);
void SendC(word CMD);
void SendD(const word DAT);
void SendCDlist(word* CDBuf,int CDLen);  //CMD:CDBuf[0]  CDLen=DatLen+1
void SendDlist(word* DatBuf,int length);
void SetFreq(word Freq);     //set TBus frequency
word CalcuCHKS(word *Buffer,word Length);
word CalcuCHKS_RST(word *Buffer,word Length);
word CalcuCHKS_ACPR(word *Buffer,word Length);
byte CalcuCHKS_GRO(byte *pData,int nLength);
 
void OpenPRE(void);
void ClosePRE(void);
void OpenVIBR(void);
void CloseVIBR(void);
void OpenDIAM(void);
void CloseDIAM(void);

void DelayNop(word Num);
void DelayESC(word Num); 

#pragma CODE_SEG DEFAULT

#endif