/*
*********************************************************************************************************
*                                                uC/OS-II
*                                          The Real-Time Kernel
*
*                              (c) Copyright 1992-2005, Micrium, Weston, FL
*                                           All Rights Reserved
*
*                                           MASTER INCLUDE FILE
*********************************************************************************************************
*/

#ifndef  INCLUDES_H
#define  INCLUDES_H

#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "xgate.h"

#include "eeeprom.h"
#include "UserVarDef.h"
#include "MCU.h"
#include "TBus.h"
#include "IIC_App.h"  
#include "SPI_App.h" 
#include "AD.h"  
#include "FIFO.h"
#include "Timer.h"  
#include "XgateMCU.h"
#include "SCI.h"


extern STATUSSTR CurStatus;
extern EEPROM_DAT eepromDat;
extern RTCCLOCK RtcClock;
extern word SysTime;   //系统时间
extern TBUSDAT CurDat;      //Current TBus Dat
extern QUEUE TbusQueue; //Tbus queue
extern byte GlobalOpState;
extern byte NextGlobalOpState;
extern QUEUEBYTE LbusQueue;   //Lbus queue
extern word VIP[12][4];
extern byte AD_Index;
extern float AvVIP[4]; 
extern byte BI_SV_State; 
extern byte FlagBigCurrentProcess;
extern byte FlagLowVoltageProcess;
extern byte LV_Count;
extern dword BatTimerCounter;
extern byte TimeBroState;

extern QUEUEBYTE LbusQueue;   //Lbus queue


#endif                                                                  /* End of file                                              */

