#include "includes.h"

//global variable defination
EEPROM_DAT eepromDat @0x0F00;
volatile STATUSSTR CurStatus; //Current status
volatile RTCCLOCK RtcClock;   //realtime clock
volatile word SysTime;        //system time

volatile TBUSDAT CurDat;      //Current TBus Dat
volatile dword DmBlankAdr[MaxDiskCount]; //data memory blank address 
volatile QUEUE TbusQueue; //Tbus queue 

volatile word DimStatus[3];    //Dim status

volatile DBCLOG DBCLogDat;
volatile float VI[2];

volatile dword BatTimerCounter;
byte TimeBroState=0;

const byte DBC_Adr=0x02; 
const byte BRA_Adr=0x00;
const byte UDT_Adr=0x42;
const byte VST_Adr=0x51;

TOOLMONITOR_ ToolMonitor; 

//2014.04.03 rewrite the firmware
//const char FW_Version[]="DBC_2.3.0#";
//add communication with T/L bus error process
//simplify the memory data
//const char FW_Version[]="DBC_2.3.1#";
//add read drive count cmd,rivise the quite state bug ,20140426
//const char FW_Version[]="DBC_2.3.2#";
//add reset Tbus tools function after four times response error
//const char FW_Version[]="DBC_2.3.3#";
//modify bat capacity caculate  bug
//const char FW_Version[]="DBC_2.3.4#";
//add Long time Reset Tbus tools
//const char FW_Version[]="DBC_2.3.6#";
//配合CGR1.0.8版本，存储 CPSC
//const char FW_Version[]="DBC_2.3.7#";
//简化读取CGR的CPS指令
//const char FW_Version[]="DBC_2.3.8#";
//修改广播时间，20140823
//const char FW_Version[]="DBC_2.3.9#";
//2014.11.06 修改与CGR通讯命令,加入TimeDelay到时广播时间命令
//const char FW_Version[]="DBC_2.3.10";
//修改配置参数相关命令，去掉实时模式初始状态字，环空压力改为PSI单位,2014.12.01
//const char FW_Version[]="DBC_2.3.11";
//修改环空压力读取命令,环空压力实时传输压缩，2015.1.21
//修改参数配置错误导致系统启动失败错误处理机制
//const char FW_Version[]="DBC_2.3.12";
//增加ACPR、DBC数据压缩相关指令，2015.04.09
//const char FW_Version[]="DBC_2.3.13";
//修正数据压缩传输bug，修正传送环空压力bug,2015.4.27
//const char FW_Version[]="DBC_2.3.14";
//增加随钻中子、密度、声波相关指令
//const char FW_Version[]="DBC_2.3.15";
//增加对GYRO的支持指令,2015.4.30                                       
//const char FW_Version[]="DBC_2.3.16";
//增加对超声井径的支持，2015.5.5
//const char FW_Version[]="DBC_2.3.17";  //去掉对随钻陀螺的支持    2016.3.15
//const char FW_Version[]="DBC_2.3.25";  //在2.3.17基础上，针对TMP+DSM+ACPR+CGR+Welleader仪器串组合，优化定型版本，2016.4.11
//const char FW_Version[]="DBC_2.3.26";  //针对丽水故障，采取临时应对措施 TMP状态字为0 1 8 不转发TGC状态字，当TMP为0 且有TbusError时，至TGC状态字 0并转发至TMP
//const char FW_Version[]="DBC_2.3.28";  // 对中子密度新仪器支持，
//const char FW_Version[]="DBC_2.3.29";   //
//const char FW_Version[]="DBC_2.3.30";   //去掉CGR前延时，感觉没啥作用
//const char FW_Version[]="DSM_1.3.30";   //随钻四条线+AGR方位伽玛+MAST声波+DSM+CDL指令下传
//const char FW_Version[]="DSM_1.4.01";   //随钻四条线+AGR方位伽玛+MAST声波+DSM+CDL指令下传
//const char FW_Version[]="DSM_1.5.01";   //修改SCI内总线相关逻辑结构
//const char FW_Version[]="DSM1.18.31";  //中控版本定型，2018年作业推广版本,支持MWD + 中子 + 密度 + DSM全功能 + ACPR + CGR + AGR + NBGR + Welleader + CDL(ASP),具有方位信息广播功能。
// const char FW_Version[]="DSM1.18.41";   //添加welleader 状态监控,修改实时内存区数据存储内容，2018.4.8
// const char FW_Version[]="DSM1.18.62";    //添加TBus error标记位,修改中控对近钻头静态井斜上次的支持
//const char FW_Version[]="DSM1.18.71";     //增加省电模式
//const char FW_Version[]="DSM1.18.72";    //增加上总线故障退出省电模式处理
//const char FW_Version[]="DSM1.18.73";    //解决VSM实时上传曲线总是带TBusError的问题
// const char FW_Version[]="DSM1.18.81";    //去掉省电模式，加入DWPR
// const char FW_Version[]="DSM1.18.82";      //修改方位信息同步转发处理
// const char FW_Version[]="DSM1.18.83";      //修改NBIG静态测量数据上传处理，2018.8.4
//  const char FW_Version[]="DSM1.18.85";      //修改了内部串行总线的处理，2018.8.14
//const char FW_Version[]="DSM1.18.86";      //修改了测试中发现的探管进入测斜模式后的假死问题，2018.8.15
// const char FW_Version[]="DSM1.18.87";       //限流提高到1.6A
  const char FW_Version[]="DSM1.18.88";       //优化开关总线时的内存存储,2018.11.08
  
//end global variable defination
//function declaration
#define ReSet() (COPCTL|=7, ARMCOP = 85, ARMCOP = 171)
static void VariableInit(void);

static void Delay_xms(word xms);
static void WriteLogDat(byte dataID,DBCLOG *logdat);
static void GetRTC(word *pBuf);
static void CompressData(word *Object,float *Source,float Xmin,float Xmax,byte N);

static void GetPreDat(int index,PRESAVEDAT *preDat);
static void WritePreDat(int DiskID,PRESAVEDAT *preDat);

void OpenLbus(void);      
void CloseLbus(void);
void OpenTbus(void);      
void CloseTbus(void);

//end function declaration

//临时变量外移，解决堆栈溢出问题
#define MAXBUFLEN   260
static word tempRecvBuf[MAXBUFLEN];
static word tempSendBuf[MAXBUFLEN];
static word tempWordBuf[30];
word timeBroBuf[7];
       
static byte tempRecvBufLbus[MAXBUFLEN];
static byte tempSendBufLbus[MAXBUFLEN];                              

OPRAT_DAT OperaBuf;

const COB_CPR_TABLE[8]={50,100,300,500,600,700,800,1000};
const VOB_CPR_TABLE[4]={28,29,30,35};
const BAT_CPR_TABLE[13]={1,3,5,8,11,15,20,25,30,35,40,45,55};
const TEMP_CPR_TABLE[16]={50,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200};

const word POWER_ON_DELAY = 30000;  //30 S
const word DAQ_DELAY=5000;  //5 S
const byte RETRY_TIMES=18;  //5*18=90S

volatile QUEUEBYTE LbusQueue;   //Lbus queue                  
const byte LBUS_CYCLE = 6;
volatile byte NBGR_SMDAT_CNT = 0;
volatile byte NBGR_SMDAT_FLG = 0;
                                                                       
void main(void) 
{
   //*****************************
   //variable defination
    byte VI_Check_State=0; 
    byte RT_State=0;    
    byte Next_BC_State=0;
    
    byte GlobalOpState=0;  //operation state
    byte NextGlobalOpState=0;  
    
    //end defination
    //*****************************
    //temp variable
    int i=0,j=0;
    
    BusDAT tempBusDat;
    word tempCMD=0;
    word DatRecvLen=0;
    word DatSendLen=0;
    word SciDatSendLen = 0;
    
    word SecS,SecE;
    word Length=0;
    dword LB_Start=0;
    
    byte *pByte,*pByte2,tempByte;
    word *pWord,tWord,tempWord;
    float tFloat=0.0;
    dword tempDWord,tempAdr,tempLB;
    
    dword BusTimeDelayCount=0;
    
    //******** Big Current low Voltage related variable
    byte  BigCur_Count=0;
    dword VI_CheckMillSecStart=0; 
    dword VI_CheckMillSecEnd=0;
    
    dword BigCurMillSecStart=0;
    dword BigCurMillSecEnd=0;    
    
    dword BigCurSecStart=0;
    dword BigCurSecEnd=0;
     
    byte FlagBigCurrentProcess=0; 
    byte FlagLowVoltageProcess=0;     
    byte SmallVol_Count=0;  
    
    byte BigCurrentProcesStep=0;
    byte BigCurrentJude=0;
    byte BigCurrentBeforJudge=0;
    
    byte LowVoltageProcessStep=0; 
    dword LowVolMillSecStart=0;
    dword LowVolMillSecEnd=0;     
    dword LowVolSecStart=0;
    dword LowVolSecEnd=0;
    byte LowVolBeforJudge=0;
  //******** broadcast time  
    byte NextTimeBroState=0;/////yao
    dword TimeBroMillSecStart=0;
    dword TimeBroMillSecEnd=0; 
    byte BroCount=0; 
  //******** Operation mode
    word SysTime0;  //计时起点
    word SysTime2;  //DBC log data save timer start 
    word tempCMD_Send=0; 
    Byte ReponseErrorBackState=0; 
    byte ReponseRightBackState=0; 
    
    dword OperMillSecStart=0;
    dword OperMillSecEnd=0; 
    byte RetryCount=0;
    byte OddEvenCount=0;
    
    
    dword CwprMillSecStart=0;
    dword CwprMillSecEnd=0; 
    OPRAT_SAVDAT OperSaveDat;
    
    byte ErrorCommCount=0;
    byte ErrorCommCountTbus=0;
    byte ErrorCommCountACPR=0;
    byte TempCountABC=0;
  //********* caculate battery capacity
    long long BatSum=0;
    long I_Count=0;
    dword StartBatCounter=BatTimerCounter;     
  //********* quiet mode
    dword QuietMillSecStart=0;
    dword QuietMillSecEnd=0;
    byte FlagInQuite=0;
  //********* TimeDelay
    dword TimeDelaySecStart=0;
    dword TimeDelaySecEnd=0;
  //********* DBC save log data 
    byte DBC_LogSaveStat=0;
  //********** save pre data
    byte DBC_PreSaveStat=0;   //环空压力
    word tempSysTime=0;
    int PreDatIndex=0;
    PRESAVEDAT PreSavDat;
  //********* Communication error process
    dword ComErrorSecStart=0;
    dword ComErrorSecEnd=0;
 
 //*********DIAM
    byte LbusState=0;
    byte NextLbusState=0; 
    byte LbusRightState = 0;
    byte LbusErrorState = 0;
    dword UDT_DelayMSecStart=0;
    dword UDT_DelayMSecEnd=0;  
    word TempLbusSysTime;        //system time
    word LbusLen=0;
    word LbusRecvLen=0;
    word LbusObjRecvLen=0; 
 //****************VST
    
    byte SCI_ReplyDatLen = 0;  
   
 //-------广播角差----------------
    byte DIMDTA_BRO_Flag = 0;
    //byte DIMDTA_Updata_Flag = 0;
    word GT_CHK;
    word DIM_Status;
 //------AGR-------------------
    dword CGRSDelaySecEnd = 0;
    dword CGRSDelaySecStart = 0;
    dword CGRRDelaySecEnd = 0;
    dword CGRRDelaySecStart = 0;
    
  //*********TGC
    byte TGC_TbusErrorCount=0;
    
  /*
  // pump off save energeny    
    word TIMES_PUMP_OFF = 0;     //Polling Times during Pump off
    word PowerSavingETS = 0;     //Minute change  seconds
    const word PollingDimStatus = 10000; // set DIM status polling time during Pump off, ms
    dword PUMPOFF_DelayMSecStart = 0;
    dword PUMPOFF_DelayMSecEnd = 0;
   */
 //*****************************  
    IVBR = 0x7F;    //for bootloader
      
    Eeeprom_intial();   
    if(BootCount==65535)       //refresh bootcount
        BootCount=0; 
    else
        BootCount++;
    
    PLL_Init();
    SetupXGATE();     //enable XGATE
    
    GPIO_Init();
    TBus_Init(); 
    InitRtc(); 
    TimerInit();        
    AD_Init();      
    InitSPI();
    SCIInit();

    //变量初始化
    VariableInit();     
    // _DISABLE_COP(); 
    _ENABLE_COP_X(); 
	  EnableInterrupts;
	  
    OperaBuf.FlagDSTATUS=1;
    OperaBuf.FlagUSM=1;
    OperaBuf.FlagVSM=1;
    OperaBuf.FlagACPRR=1;
    OperaBuf.FlagTGCC=1;
    OperaBuf.FlagSNK=1;

  	//for broadcasting time when reset in operation mode
  	if(3==EEP_MODE)
  	{
  	    TimeBroState=1;
  	}
  	else
  	{
  	    TimeBroState=0;
  	}
	
	
    while(TRUE) 
    {
    
        //******************************************
        //DBC work as BC
        if(3==EEP_MODE )
        {
             //broadcast time when set operation and after reset in operation
             switch(TimeBroState)
             {
                case 1:
                    TimeBroState = 2;
                    BroCount = 0;                    
                    break;     
                case 2:
                    TimeBroMillSecStart = MillSecCount;
                    TimeBroState = 3;
                    break;
                case 3:
                    TimeBroMillSecEnd = MillSecCount;                
                    if(TimeBroMillSecEnd < TimeBroMillSecStart)
                    {
                        TimeBroMillSecEnd |= 0x00010000;
                    }                    
                    //time delay 4 s
                    if(TimeBroMillSecEnd - TimeBroMillSecStart >= 4000)
                    {  
                        TimeBroState = 4; 
                    }
                    break;
                case 4:
                    tempWordBuf[0] = SDI_SYNCTIME_WRITE_BRO;     
                    GetRTC(tempWordBuf+1);
                    tempWordBuf[3] = CalcuCHKS(tempWordBuf+1,2);
                    SendCDlist(tempWordBuf,4);
                    DatRecvLen=0;
                    
                    //UDT 广播时间
                    SCI_ClearBuf();
                    InitQueueByte(&LbusQueue);                    
            				timeBroBuf[0] = SDI_SYNCTIME_WRITE_BRO;
            				timeBroBuf[1] = 3;  //length
            				for(i=0;i<2;i++)
            				{                   				    
            				    timeBroBuf[i+2] = tempWordBuf[i+1];
            				}
            				timeBroBuf[4] = CalcuCHKS(timeBroBuf+1,3);
            				SCI_Send(timeBroBuf,10);                      
    				
                    BroCount++;
                    if(BroCount >= 3)
                    {
                        TimeBroState=0;
                            
                    }
                    else
                    {
                        TimeBroState = 2;                            
                    }
                                      
                    break; 
                default:
                    break;
                
             }
             
             //Save DBC log data
            switch(DBC_LogSaveStat)
            {
                case 0: //default state
                    if(0!=eepromDat.CRP1.ParTimeDBC && 0==SysTime%eepromDat.CRP1.ParTimeDBC)
                    {
                        SysTime2=SysTime;
                        DBC_LogSaveStat=1;
                    }
                    break;
                case 1:  //save current data                                         
                    WriteLogDat(18,&DBCLogDat); 
                    DBC_LogSaveStat=2;
                    break;
                case 2:  //Idel
                    if(SysTime2!=SysTime)
                    {
                        DBC_LogSaveStat=0;  //go to default 
                    }
                    break;
                default:
                    DBC_LogSaveStat=0;  //go to default 
                    break;
                
            }
            
            
            //save Pressure realtime data
            switch(DBC_PreSaveStat)
            {
                case 0: //default state
                    if(0!=SysPar.ParTimeSavePre && 0==SysTime%SysPar.ParTimeSavePre)
                    {
                        tempSysTime=SysTime;
                        DBC_PreSaveStat=1;                        
                    }
                    break;
                case 1:  //save current data 
                    GetPreDat(PreDatIndex,&PreSavDat);                    
                    PreDatIndex++;                                                           
                    if(6==PreDatIndex)
                    {
                        WritePreDat(2,&PreSavDat);  
                        PreDatIndex=0;  
                    }
                    
                    DBC_PreSaveStat=2;
                    break;
                case 2:  //Idel
                    if(tempSysTime!=SysTime)
                    {
                        DBC_PreSaveStat=0;  //go to default 
                    }
                    break;
                default:
                    DBC_PreSaveStat=0;  //go to default 
                    break;
                
            }
            
            //save Pressure statistics data
            
            
  //***************Lbus data  process ***********//
            LbusState = NextLbusState;
            switch(LbusState)
            {
                case OP_STAT_LBUS_DEFAULT:
                    if(0 == SysTime%LBUS_CYCLE)         //4s
                    {
                        if(1 == FlagDIAM)               //UDT
                        {
                            NextLbusState = OP_STAT_LBUS_UDT;
                        }
                        else if(1 == FlagVST)
                        {
                            NextLbusState = OP_STAT_LBUS_VST;
                        }
                       
                        TempLbusSysTime = SysTime;        
                    }
                    break;
                case OP_STAT_LBUS_UDT:       //clear buf and send cmd
                    SCI_ClearBuf();
                    InitQueueByte(&LbusQueue);
                    
                    tempSendBufLbus[0] = 0x50;
                    tempSendBufLbus[1] = 0x42;
                    tempSendBufLbus[2] = 0x00;
                    tempSendBufLbus[3] = 0x00;                  
                    //SciDatSendLen = 4;
                    SCI_Send(tempSendBufLbus,4);   
                    NextLbusState = OP_STAT_LBUS_RECV_WAIT; 
                    SCI_ReplyDatLen = 4;       
                    LbusRightState = OP_STAT_LBUS_UDT_1;   
                    LbusErrorState = OP_STAT_LBUS_UDT_2;        
                    UDT_DelayMSecStart = MillSecCount;                   
                    break;
                case OP_STAT_LBUS_RECV_WAIT:    
                    UDT_DelayMSecEnd = MillSecCount;
                    if(UDT_DelayMSecEnd < UDT_DelayMSecStart)
                    {
                        UDT_DelayMSecEnd |= 0x00010000;
                    }                    
                    //time delay 200 ms
                    if(UDT_DelayMSecEnd-UDT_DelayMSecStart>=200)
                    {
                        NextLbusState = LbusErrorState; 
                    }
                    else
                    {
                        if(QueueLenthByte(&LbusQueue) >= SCI_ReplyDatLen)
                        {
                            NextLbusState = LbusRightState;    
                        }
                    }
                    break;
                case OP_STAT_LBUS_UDT_1://read data from fifo to tempRecvBufLbus buf                                         
                    LbusLen = 0;
                    while(QueueLenthByte(&LbusQueue)>0)
                    {
                        DeQueueByte(&LbusQueue,&tempByte);
                        tempRecvBufLbus[LbusLen]=tempByte;
                        LbusLen++;    
                    }
                    OperaBuf.FlagUSM = 0;
                    pWord = tempRecvBufLbus+2;
                    OperaBuf.USM_DAT = *pWord;
                    NextLbusState = OP_STAT_LBUS_UDT_2; 
                    break;
                case OP_STAT_LBUS_UDT_2:
                    if(1 == FlagVST)
                    {
                        
                        NextLbusState=OP_STAT_LBUS_VST;
                    } 
                    else
                        NextLbusState=OP_STAT_LBUS_IDLE;
                    break;
                case OP_STAT_LBUS_VST:       //clear buf and send cmd
                    SCI_ClearBuf();
                    InitQueueByte(&LbusQueue);
                    tempSendBufLbus[0] = 0x55;
                    tempSendBufLbus[1] = 0x51;
                    tempSendBufLbus[2] = 0x00;
                    tempSendBufLbus[3] = 0x00;                      
                    //SciDatSendLen = 4;
                    SCI_Send(tempSendBufLbus,4); 
                    //OperaBuf.FlagVSM = 1;   //reset the flag
                    NextLbusState = OP_STAT_LBUS_RECV_WAIT;  
                    SCI_ReplyDatLen = 18;       
                    LbusRightState = OP_STAT_LBUS_VST_1;  
                    LbusErrorState = OP_STAT_LBUS_ERROR;             
                    UDT_DelayMSecStart = MillSecCount;                   
                    break;

                case OP_STAT_LBUS_VST_1://read data from fifo to tempRecvBufLbus buf                                         
                    LbusLen=0;                            
                    while(QueueLenthByte(&LbusQueue)>0)
                    {
                        DeQueueByte(&LbusQueue,&tempByte);
                        tempRecvBufLbus[LbusLen] = tempByte;
                        LbusLen++;    
                    }
                    OperaBuf.FlagVSM = 0; 
                    pWord = tempRecvBufLbus+2;
                    for(i = 0; i < (LbusLen/2)-1; i++)
                    {
                        OperaBuf.VST_DAT[i] = *pWord;
                        pWord++;
                    }
                    NextLbusState=OP_STAT_LBUS_IDLE; 
                    break;     
                case OP_STAT_LBUS_ERROR: 
                    SCIInit();
                    //ClosePRE();
                    //Delay_xms(800);
                    //OpenVIBR();
                    NextLbusState = OP_STAT_LBUS_IDLE; 
                    break;   
                case OP_STAT_LBUS_IDLE:              
                    if(TempLbusSysTime!=SysTime)
                    {
                        NextLbusState = OP_STAT_LBUS_DEFAULT;    
                    }
                    break;
                default:
                    NextLbusState=OP_STAT_LBUS_DEFAULT;
                    break;
            }
            //***************Lbus data process end ***********//
    
             //operation mode DBC work as Bus controller
             //realtime process
            GlobalOpState=NextGlobalOpState;      //状态转换           
            switch(GlobalOpState)
            {
                case OP_STAT_DELAY:
                  
                    if(SysTime>SysPar.ParTimeDelay)  //time delay function
                    {                                                                              
                        OpenLbus();
                        OpenTbus();
                        if(1==FlagPRE)
                        {
                            OpenPRE();
                        }
                        else
                        {
                            ClosePRE();
                        }
                        if(1==FlagVST)
                        {
                            OpenVIBR();
                        }
                        else
                        {
                            CloseVIBR();
                        }
                        if(1==FlagDIAM)
                        {
                            OpenDIAM();
                        }
                        else
                        {
                            CloseDIAM();
                        } 
                        
                        //broadcast time switch
                        TimeBroState=1;  //broadcast time    
                        NextGlobalOpState=OP_STAT_DEFAULT;                       
                    }
                    break;
                   
                case OP_STAT_DEFAULT:                     
                    if(0!=SysPar.ParTimeToMWD && 0==SysTime%SysPar.ParTimeToMWD)
                    {
                        SysTime0 = SysTime;  //当前时间缓存,标记
                        NextGlobalOpState = OP_STAT_DIMCOM;        
                    } 
                    else
                        NextGlobalOpState = OP_STAT_DEFAULT;
                    break;
                   
                case OP_STAT_DIMCOM:
                    //准备数据
                    tempSendBuf[0] = DIM_STAT_READ;                     
                    DatSendLen = 1; 
                    OperaBuf.FlagDSTATUS = 1;                  
                    NextGlobalOpState = OP_STAT_STARTSEND;
                    ReponseRightBackState = OP_STAT_DIMCOMOK;
                    ReponseErrorBackState = OP_STAT_TRANSFER_REATIMEDATA; //for future rewrite
                    break;
                case OP_STAT_STARTSEND:
                    RetryCount=0;
                    NextGlobalOpState=OP_STAT_SENDCMD;
                    break;
                case OP_STAT_SENDCMD:
                    OperMillSecStart = MillSecCount;                                                                             
                    tempCMD_Send = tempSendBuf[0];                                     
                    SendCDlist(tempSendBuf,DatSendLen); 
                    DatRecvLen=0;                                         
                    NextGlobalOpState = OP_STAT_GETRESPONSE;                                       
                    break;
                case OP_STAT_GETRESPONSE:                                   
                    switch(tempCMD_Send)
                    {
                        
                        case  DIM_STAT_READ: 
                            //if(DatRecvLen>=1)  //for debug                                                   
                            if(DatRecvLen>=4)
                            {
                                if((tempRecvBuf[1]==tempRecvBuf[2])&&(tempRecvBuf[1]==tempRecvBuf[3]))                           
                                {
                                    DimStatus[2] = DimStatus[1];   
                                    DimStatus[1] = DimStatus[0];   
                            	    DimStatus[0] = tempRecvBuf[1];
                            	    //put data into cache    
                            	    OperaBuf.FlagDSTATUS = 0;
                            	    OperaBuf.DSTATUS = DimStatus[0];
                            	    ErrorCommCount = 0;
                            	    NextGlobalOpState = ReponseRightBackState;
                                }    
                                else
                                    NextGlobalOpState = ReponseErrorBackState;   
                            }                                                    
                            break; 
                        case DIM_DBC_SEND:    
                        case DIM_PMT_SEND: 
                        case DIM_AGR_SEND:
                        case DIM_ACPR_SEND: 
                        case DIM_ACPRCP_SEND:                        
                        case DIM_SAK_SEND:
                        case DIM_RSCDL_SEND:
                        case DIM_SCDL_SEND:
                        case DIM_SFUN_SEND:
                        case DIM_ABIN_SEND:
                        case DIM_TSTAT_SEND:            
                        case DIM_TVEL_SEND:
                        case DIM_GRO_SEND:
                        case DIM_PRE_SEND:
                        case DIM_NEUT_SEND:
                        case DIM_ACOU_SEND:
                        case DIM_DENS_SEND:
                        case DIM_UDT_SEND:
                        case DIM_VST_SEND:
                        case DIM_NBMD_SEND:
                        case DIM_NBGD_SEND:
                        case DIM_DWPR_SEND:
                            if(DatRecvLen>=2)
                            {
                                NextGlobalOpState = ReponseRightBackState;    
                            }
                            break;
                        
                        case DIM_MCM_READ:               //广播角差
                            if((DatRecvLen>=15) && (tempRecvBuf[14]==CalcuCHKS(tempRecvBuf+1,13)))
                            {
                                DIM_Status = tempRecvBuf[2];
                                GT_CHK = tempRecvBuf[11];
                                //if( (DIM_Status==9) && (GT_CHK>=2038) && (GT_CHK<=2058))
                                if((DIM_Status==9) && (GT_CHK>=4) && (GT_CHK<=13)) 
                                {
                                     //DIMDTA_Updata_Flag = 1;//广播角差 
                                     OperaBuf.FlagMWDD++;  //收到DIM测量数据次数累加
                                     for(i=0;i<12;i++) 
                                     {
                                        OperaBuf.MWD_DAT[i]=tempRecvBuf[i+2];
                                     }
                                     NextGlobalOpState = ReponseRightBackState;
                                } 
                                else 
                                     NextGlobalOpState = ReponseErrorBackState;                            
                            }
                            break;
                        /*    
                        case GRO_POWER_ON:
                        case GRO_START_DAQ: 
                        case GRO_POWER_OFF:                       
                            if(DatRecvLen>=5 && tempRecvBuf[4]==CalcuCHKS(tempRecvBuf+1,3) && 0x0001==(tempRecvBuf[2]&0x00FF))
                            {
                                OperaBuf.FlagGROO=0;                                
                                NextGlobalOpState = ReponseRightBackState; 
                            }
                            break;    
                        case GRO_STAT_READ:
                            if(DatRecvLen>=5 && tempRecvBuf[4]==CalcuCHKS(tempRecvBuf+1,3))
                            {
                                OperaBuf.FlagGROO=0;
                                OperaBuf.GRO_MEAS_STAT=tempRecvBuf[2]&0x00FF;                                                                
                                NextGlobalOpState = ReponseRightBackState; 
                            }
                            break;                               
                        case GRO_DATA_READ:
                            if(DatRecvLen>=7 && tempRecvBuf[6]==CalcuCHKS(tempRecvBuf+1,5))
                            {
                                OperaBuf.FlagGROO=0;
                                for(i=0;i<5;i++) 
                                {
                                    OperaBuf.GRO_DAT[i]=tempRecvBuf[i+1];
                                }
                                NextGlobalOpState = ReponseRightBackState; 
                            }
                            break;
                        case GRO_BATC_READ:
                            if(DatRecvLen>=3 && tempRecvBuf[2]==CalcuCHKS(tempRecvBuf+1,1))
                            {
                                OperaBuf.FlagGROO=0;
                                OperaBuf.GRO_DAT[4]=OperaBuf.GRO_DAT[4]&0xFF00; //clear low 8 bits                                
                                OperaBuf.GRO_DAT[4]+=(tempRecvBuf[4]&0x00FF);
                                NextGlobalOpState = ReponseRightBackState;                            
                            }
                            break;
                        */    
                        case ACOU_DAT_READ:         //声波
                            if(DatRecvLen>=5 &&tempRecvBuf[4]==CalcuCHKS(tempRecvBuf+1,3))    //zhuwh1016
                            {
                                OperaBuf.FlagMAST=0; //clear flag  
                                for(i=0;i<3;i++)
                                {
                                    OperaBuf.MAST_DAT[i]=tempRecvBuf[i+1];    
                                }                                
                                NextGlobalOpState = ReponseRightBackState;        
                            }
                            break;
                       
                        case NEUT_data_READ:       //2016.9.3针对中子新仪器修改
                            if(DatRecvLen>=4 &&tempRecvBuf[3]==CalcuCHKS(tempRecvBuf+1,2))   
                            {
                                OperaBuf.FlagINPT=0; //clear flag  
                                OperaBuf.Porosity=tempRecvBuf[1];  
                                OperaBuf.HoleSize=tempRecvBuf[2];                              
                                NextGlobalOpState = ReponseRightBackState;        
                            }
                            break;
                        case LDI_DAT_READ:
                            if(DatRecvLen>=5 &&tempRecvBuf[4]==CalcuCHKS(tempRecvBuf+1,3)) //2016.8.30针对密度新仪器修改  
                            {
                                OperaBuf.FlagLDIT=0; //clear flag  
                                for(i=0;i<3;i++)
                                {
                                    OperaBuf.LDI_DAT[i]=tempRecvBuf[i+1];    
                                }                                
                                NextGlobalOpState = ReponseRightBackState;        
                            }
                            break;                                                
                        case ACPR_GNDAT_READ:
                            if(DatRecvLen>=23)
                            {
                                OperaBuf.FlagACPRR=0;
                                
                      	        OperaBuf.ACPR_DAT[0]=tempRecvBuf[4];  //time
                      	        OperaBuf.ACPR_DAT[1]=tempRecvBuf[3];
                      	        OperaBuf.ACPR_DAT[2]=tempRecvBuf[5];  //trx                     	                                  	        
                      	        //for debug
                      	        
                      	        
                      	        //PDBHX ATBHX ~ PDBSLX ATBSX                       	        
                      	        for(i=0;i<8;i++)
                      	        {
                      	            //interchange upword and low word
                      	            tempWord=tempRecvBuf[6+2*i];
                      	            tempRecvBuf[6+2*i]=tempRecvBuf[6+2*i+1];
                      	            tempRecvBuf[6+2*i+1]=tempWord;
                      	            
                      	            //compress data
                              	    pByte=tempRecvBuf+6+2*i;                              	    
                              	    if(i%2==0)
	                                    CompressData(OperaBuf.ACPR_DAT+i+3,pByte,-10.0,250.0,16);	//[-10, 250] 	16  	                                    
	                                else
	                                    CompressData(OperaBuf.ACPR_DAT+i+3,pByte,0.0,50.0,16);	//[0, 50]	16           	    
                      	        }    
                                
                                NextGlobalOpState = ReponseRightBackState;         
                            }
                            break;
                        case DWPR_RTD_READ:
                            if(DatRecvLen >= 6 )
                            {
                                OperaBuf.FlagDWPR = 0;    
                      	        OperaBuf.DWPR[0] = tempRecvBuf[2];  //
                      	        OperaBuf.DWPR[1] = tempRecvBuf[3];
                      	        OperaBuf.DWPR[2] = tempRecvBuf[4]; 
                      	        NextGlobalOpState = ReponseRightBackState;      
                            }
                            break;
                        case ACPR_CPDAT_READ:
                            if(DatRecvLen>=8)
                            {
                                OperaBuf.FlagACPRR=0;       
                      	        for(i=0;i<4;i++)
                      	        {
                      	           OperaBuf.ACPR_DAT[i]=tempRecvBuf[i+3]; 
                      	        } 
                                NextGlobalOpState = ReponseRightBackState;         
                            }
                            break;
                        case PMT_API_READ:             //CGR
                            if(DatRecvLen>=4)
                            {
                                pWord=OperaBuf.API;
                                for(i=0;i<2;i++)
                                {
                                    *pWord=tempRecvBuf[i+1];
                                    pWord++;
                                }                                
                                OperaBuf.FlagPMTT =0;
                                NextGlobalOpState = ReponseRightBackState;                
                            }
                            break;
                        case AGR_API_READ:             //AGR
                            if(DatRecvLen>=10)
                            {
                                pByte=OperaBuf.AGR;
                                for(i=0;i<7;i++)
                                {
                                    *pByte=tempRecvBuf[i+2]&0xFF;
                                    pByte++;
                                }                                
                                OperaBuf.FlagAGRT =0;
                                NextGlobalOpState = ReponseRightBackState;                
                            }
                            break;  
                        case NBT_GDATA_READ:         //NBGR
                            if(DatRecvLen>=9)
                            {
                                pWord = OperaBuf.NBGR_GDAT;
                                for(i=0;i<6;i++)
                                {
                                    *pWord=tempRecvBuf[i+2];
                                    pWord++;
                                }                                
                                OperaBuf.FlagNBIG = 0;
                                NextGlobalOpState = ReponseRightBackState;                
                            }
                            break;  
                        case NBT_MDATA_READ:       //NBGR
                            if(DatRecvLen>=12)
                            {
                                pWord = OperaBuf.NBGR_MDAT;
                                for(i=0;i<9;i++)
                                {
                                    *pWord=tempRecvBuf[i+2];
                                    pWord++;
                                }                                
                                OperaBuf.FlagNBIG = 0;
                                if((OperaBuf.NBGR_MDAT[0]&0xFFFF) == 0x02)
                                {
                                    for(i=0; i<9; i++)
                                    {
                                        OperaBuf.NBGR_SMDAT[i] = OperaBuf.NBGR_MDAT[i];
                                       
                                    } 
                                    NBGR_SMDAT_CNT++;
                                    NBGR_SMDAT_FLG++;        
                                }
                                NextGlobalOpState = ReponseRightBackState;                
                            }
                            break;      
                        
                        case RST_SAK_READ:
                            if(DatRecvLen>=16)
                            {
                                if(tempRecvBuf[15]==CalcuCHKS_RST(tempRecvBuf+1,14))
                                {                                    
                                    OperaBuf.FlagSNK = 0;     //clear error flag
                                    NextGlobalOpState = ReponseRightBackState;
                                }
                                pByte=OperaBuf.SNK_DAT;
                                for(i=0;i<14;i++)
                                {
                                    *pByte=tempRecvBuf[i+1]&0xFF;
                                    pByte++;    
                                } 
                            }
                            break;
                        case RST_SCDL_READ:
                            if(DatRecvLen>=5 && tempRecvBuf[4]==CalcuCHKS_RST(tempRecvBuf+1,3))
                            {
                                OperaBuf.FlagSNK=0;     //clear error flag
                                pByte=&OperaBuf.Mode;
                                for(i=0;i<3;i++)
                                {
                                    *pByte=tempRecvBuf[i+1]&0xFF;    
                                    pByte++;
                                }
                                NextGlobalOpState = ReponseRightBackState;
                            }
                            break;
                         case READ_TGC_CDLDATD:
                            if(DatRecvLen>=5 && tempRecvBuf[4]==CalcuCHKS_RST(tempRecvBuf+1,3))
                            {
                                OperaBuf.FlagSNK=0;     //clear error flag
                                pByte=&OperaBuf.Mode;
                                for(i=0;i<3;i++)
                                {
                                    *pByte=tempRecvBuf[i+1]&0xFF;    
                                    pByte++;
                                }
                                if(OperaBuf.Mode == 0x0A)
                                {
                                     NextGlobalOpState = OP_STAT_TRANSFER_CDLDATA_2;
                                     
                                }
                                else
                                    NextGlobalOpState = ReponseRightBackState;
                                
                            }
                            break;
                         case RST_TVEL_READ:
                            if(DatRecvLen>=5 && tempRecvBuf[4]==CalcuCHKS_RST(tempRecvBuf+1,3))
                            {
                                OperaBuf.FlagTGCC=0;       //clear error flag
                                OperaBuf.TVEL=tempRecvBuf[1]&0xFF;
                                OperaBuf.RMAX=tempRecvBuf[2]&0xFF;
                                OperaBuf.RMIN=tempRecvBuf[3]&0xFF;
                                NextGlobalOpState = ReponseRightBackState;
                            }
                            break;
                        case RST_TSTAT_READ:
                            if(DatRecvLen>=4 &&tempRecvBuf[1]==tempRecvBuf[2] && tempRecvBuf[1]==tempRecvBuf[3] )   
                            {
                                OperaBuf.FlagTGCC = 0; //clear flag
                                OperaBuf.TSTATUS = tempRecvBuf[1]&0xFF;
                                NextGlobalOpState = ReponseRightBackState;        
                            }
                            break;
                        case RST_DSTAT_SEND:
                            if(DatRecvLen>=2 &&tempRecvBuf[1]==tempSendBuf[2])   
                            {
                                NextGlobalOpState = ReponseRightBackState;        
                            }
                            break;
                            
                        case MONITOR_RST_DAT:
                            if(DatRecvLen>=2)   
                            {
                                ToolMonitor.RST = tempRecvBuf[1]&0xFF;
                                NextGlobalOpState = ReponseRightBackState;        
                            }
                            break;
                        default:
                            break;
                    }
                    
                    OperMillSecEnd = MillSecCount;
                    if(OperMillSecEnd<OperMillSecStart)
                    {
                        OperMillSecEnd|=0x00010000;
                    }                    
                    //time delay 300 ms
                    if(OperMillSecEnd-OperMillSecStart>=500)
                    {
                          
                        NextGlobalOpState = OP_STAT_SENDCMD;
                        RetryCount++;
                        if(RetryCount>=3)
                        {
                            NextGlobalOpState = ReponseErrorBackState;    
                        }
                    }                    
                    break;
                
                case OP_STAT_DIMCOMOK:                     
                    switch(DimStatus[0]&0xFF)
                    {   
                        case PumpOFF:   
                            /*
                            TIMES_PUMP_OFF++;
                            PowerSavingETS = SysPar.PowerSavingET*60;                            
                            if((SysPar.PowerSavingET!=0)&&(SysPar.ParTimeToMWD>0)&&(TIMES_PUMP_OFF>(PowerSavingETS/SysPar.ParTimeToMWD)))
                            {                                 
                                NextGlobalOpState = OP_STAT_PUMP_OFF_SAVEBAT;    
                            }
                            else
                            {
                                NextGlobalOpState = OP_STAT_TRANSFER_REATIMEDATA;   
                            }
                            */
                            NextGlobalOpState = OP_STAT_TRANSFER_REATIMEDATA;
                            break;
                        case CDL_WaitData:
                            //TIMES_PUMP_OFF = 0;
                            NextGlobalOpState = OP_STAT_READ_CDLDATA;  //transfer cdl data
                            break;                       
                        case CDL_Waiting:
                            //TIMES_PUMP_OFF = 0;
                            NextGlobalOpState = OP_STAT_SYN_DIM_TGC_1;  //only syn status
                            break;
                        case DIM_Sleep:
                            //TIMES_PUMP_OFF = 0;
                            NextGlobalOpState=OP_STAT_IDEL;
                            break;
                        case DIM_Measure:            //测斜状态
                            //TIMES_PUMP_OFF = 0;
                            NextGlobalOpState = OP_STAT_GET_DIM_DATA;
                            break;
                        default:
                            //TIMES_PUMP_OFF = 0;
                            NextGlobalOpState=OP_STAT_TRANSFER_REATIMEDATA;                             
                            break;
                    } 
                    //broadcast time
                    if(((DIM_DataSend==(DimStatus[0]&0xFF))&&(DIM_DataSend!=(DimStatus[2]&0xFF)))
                       || ((DIM_DataSend!=(DimStatus[0]&0xFF))&&(DIM_DataSend==(DimStatus[2]&0xFF))) )
                    {
                        //turn on the switch of timebroadcast
                        TimeBroState=1;    
                    }                    
                    //enter into quiet                    
                    if(FlagInQuite==0 && (DimStatus[0]&0xFF) == DIM_Sleep&&(DimStatus[1]&0xFF) == DIM_Sleep&&(DimStatus[2]&0xFF) == DIM_Sleep)
                    {
                        NextGlobalOpState=OP_STAT_QUIET;        
                    }
                    //exit quiet
                    else if(FlagInQuite==1 && DimStatus[0]!=DIM_Sleep && DimStatus[1]!=DIM_Sleep &&DimStatus[2]!=DIM_Sleep)
                    {
                        NextGlobalOpState=OP_STAT_QUIT_QUIET;        
                    }                     
                    break;
                /*    
                case OP_STAT_PUMP_OFF_SAVEBAT:
                    WriteLogDat(37,&DBCLogDat);  //进入省电模式，记录日志数据
                    CloseTbus(); 
                    NextGlobalOpState=OP_STAT_PUMP_OFF_SAVEBAT_1;
                    break;
                case OP_STAT_PUMP_OFF_SAVEBAT_1:
                    //准备数据
                    tempSendBuf[0] = DIM_STAT_READ;                     
                    DatSendLen = 1; 
                    OperaBuf.FlagDSTATUS = 1;                  
                    NextGlobalOpState = OP_STAT_STARTSEND;
                    ReponseRightBackState = OP_STAT_PUMP_OFF_SAVEBAT_2;
                    ReponseErrorBackState = OP_STAT_PUMP_OFF_SAVEBAT_5; //DIM通信错误，退出省电模式                   
                    break;
                 case OP_STAT_PUMP_OFF_SAVEBAT_2:
                    if((DimStatus[0]&0xFF)!=PumpOFF && (DimStatus[1]&0xFF)!=PumpOFF && (DimStatus[2]&0xFF)!=PumpOFF)
                    {
                        PUMPOFF_DelayMSecStart = MillSecCount;
                        OpenTbus();
                        WriteLogDat(38,&DBCLogDat);  //退出省电模式，记录日志数据
                        NextGlobalOpState=OP_STAT_PUMP_OFF_SAVEBAT_3;    
                    }
                    else
                    {
                        PUMPOFF_DelayMSecStart = MillSecCount;
                        NextGlobalOpState = OP_STAT_PUMP_OFF_SAVEBAT_4;    
                    }
                    break;
                case OP_STAT_PUMP_OFF_SAVEBAT_3:
                    PUMPOFF_DelayMSecEnd = MillSecCount;
                    if(PUMPOFF_DelayMSecEnd < PUMPOFF_DelayMSecStart)
                    {
                        PUMPOFF_DelayMSecEnd |= 0x00010000;
                    } 
                    //Delay 10s, broadcast time;
                    if(PUMPOFF_DelayMSecEnd - PUMPOFF_DelayMSecStart >= PollingDimStatus)
                    {
                        tempWordBuf[0] = SDI_SYNCTIME_WRITE_BRO;     
                        GetRTC(tempWordBuf+1);
                        tempWordBuf[3] = CalcuCHKS(tempWordBuf+1,2);
                        SendCDlist(tempWordBuf,4);
                        
                        TimeBroState = 1;    //broadcast time
                        NextGlobalOpState = OP_STAT_IDEL;
                    }                    
                    break;
                case OP_STAT_PUMP_OFF_SAVEBAT_4:  //延时
                    PUMPOFF_DelayMSecEnd = MillSecCount;
                    if(PUMPOFF_DelayMSecEnd < PUMPOFF_DelayMSecStart)
                    {
                        PUMPOFF_DelayMSecEnd |= 0x00010000;
                    }
                     //time delay
                    //temp_Times = SysPar.ParTimeToMWD*1000;
                    if(PUMPOFF_DelayMSecEnd - PUMPOFF_DelayMSecStart >= PollingDimStatus)
                    {
                        NextGlobalOpState = OP_STAT_PUMP_OFF_SAVEBAT_1;
                    }                    
                    break;
                case OP_STAT_PUMP_OFF_SAVEBAT_5:
                    ErrorCommCount++;
                    if(ErrorCommCount>5)
                    {
                        OpenTbus();
                        WriteLogDat(38,&DBCLogDat);  //退出省电模式，记录日志数据
                        PUMPOFF_DelayMSecStart = MillSecCount;
                        NextGlobalOpState=OP_STAT_PUMP_OFF_SAVEBAT_3;       
                    }
                    else
                    {
                        PUMPOFF_DelayMSecStart = MillSecCount;
                        NextGlobalOpState = OP_STAT_PUMP_OFF_SAVEBAT_4;    
                    }
                    break;
                */
      //-------------------------------------------------------------                           
      //**********   广播角差   **************
      //-------------------------------------------------------------               
                case OP_STAT_GET_DIM_DATA:           //get dim measure data
                    //DIMDTA_Updata_Flag = 0;
                    DIMDTA_BRO_Flag = 0;
                    tempSendBuf[0] = DIM_MCM_READ;     //send cmd to MWD get AZI and INC
                    DatSendLen = 1; 
                    ReponseRightBackState = OP_STAT_DIMDTA_BRO;
                    ReponseErrorBackState = OP_STAT_IDEL; 
                    NextGlobalOpState = OP_STAT_STARTSEND;      
                    break;
                case OP_STAT_DIMDTA_BRO:
                    if(OperaBuf.FlagMWDD >= 2U)
                    {
                        OperaBuf.FlagDMBRO = 0; 
                        //DIMDTA_BRO_Flag++;
                        tempSendBuf[0] = DIRTOOL_DIMDTA_BRO;
                        tempSendBuf[1] = 0x000D;   //length 包括命令字和check
                        tempSendBuf[2] = OperaBuf.MWD_DAT[0]; //   DIM  status
                        tempSendBuf[3] = OperaBuf.MWD_DAT[5]-OperaBuf.MWD_DAT[4];//
                        tempSendBuf[4] = OperaBuf.MWD_DAT[3];
                        tempSendBuf[5] = OperaBuf.MWD_DAT[2];
                        tempSendBuf[6] = OperaBuf.MWD_DAT[8];
                        tempSendBuf[7] = OperaBuf.MWD_DAT[4];
                        tempSendBuf[8] = OperaBuf.MWD_DAT[5];
                        tempSendBuf[9] = OperaBuf.MWD_DAT[7];
                        tempSendBuf[10] = OperaBuf.MWD_DAT[9];
                        tempSendBuf[11] = 0x0000;
                        tempSendBuf[12] = CalcuCHKS(tempSendBuf,12);
                        DatSendLen = 13;
                        SendCDlist(tempSendBuf,13); 
                        Delay_xms(50); 
                    } 
                    NextGlobalOpState = OP_STAT_IDEL;                                
                    break; 
                
              /*    
                //get dim inc azi data
                case OP_STAT_GET_DIM_DATA:
                    //send cmd to MWD get AZI and INC
                    //准备数据
                    OperaBuf.FlagMWDD=1;   
                    tempSendBuf[0]=DIM_MCM_READ; 
                    tempSendBuf[1]=2;   //Len
                    tempSendBuf[2]=0;
                    tempSendBuf[3]=CalcuCHKS(tempSendBuf+1,2);
                    DatSendLen=4;
                    ReponseRightBackState=OP_STAT_TRANSFER_GRODATA;
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                                              
                    break;
                //---------------------------------                           
                //**********   GYRO   *************
                //---------------------------------  
               case OP_STAT_TRANSFER_GRODATA:
                     if(1==FlagGRO)
                        NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_1;
                    else
                        NextGlobalOpState = OP_STAT_IDEL;                    
                    break;     
                //*****   POWER ON               
                case OP_STAT_TRANSFER_GRODATA_1: 
                    OperaBuf.FlagGROO=1;
                    OperaBuf.ErrorCode=0;                   
                    tempSendBuf[0]=GRO_POWER_ON;          
                    tempSendBuf[1]=0x7E02;
                    tempSendBuf[2]=0x0300;
                    tempSendBuf[3]=0x0000;
                    tempByte=CalcuCHKS_GRO(tempSendBuf+1,5);
                    tempSendBuf[3]+=tempByte;
                    tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                                         
                    DatSendLen=5;
                    ReponseRightBackState=OP_STAT_TRANSFER_GRODATA_2; 
                    ReponseErrorBackState=OP_STAT_TRANSFER_GRO_ERR_1; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                    
                    break;
                //*****   Wait 30 second
                case OP_STAT_TRANSFER_GRODATA_2:
                
                    GYRO_DelaySecStart=MillSecCount;
                    GYRO_DelaySecEnd=MillSecCount;
                    NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_3;
                    break;                    
                case OP_STAT_TRANSFER_GRODATA_3:                    
                    GYRO_DelaySecEnd=MillSecCount;
                    if(GYRO_DelaySecEnd<GYRO_DelaySecStart)
                    {
                        GYRO_DelaySecEnd|=0x00010000;
                    }                    
                    //time delay 30 s
                    if(GYRO_DelaySecEnd-GYRO_DelaySecStart>=POWER_ON_DELAY)
                    {
                        
                        NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_4;
    
                    }                   
                    break;
                //*****   Start Measure
                case OP_STAT_TRANSFER_GRODATA_4:
                    OperaBuf.FlagGROO=1;   
                    //start daq
                    tempSendBuf[0]=GRO_START_DAQ;
                    tempSendBuf[1]=0x7E01;
                    tempSendBuf[2]=0x0300;
                    tempSendBuf[3]=0x0000;
                    tempByte=CalcuCHKS_GRO(tempSendBuf+1,5);
                    tempSendBuf[3]+=tempByte;
                    tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                                         
                    DatSendLen=5;
                    ReponseRightBackState=OP_STAT_TRANSFER_GRODATA_5; 
                    ReponseErrorBackState=OP_STAT_TRANSFER_GRO_ERR_2; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    retryTimes=0;
                    break;  
                 //*****   wait 5 second
                case OP_STAT_TRANSFER_GRODATA_5: 
                    GYRO_DelaySecStart=MillSecCount;
                    GYRO_DelaySecEnd=MillSecCount;
                    NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_6;                    
                    break;
                case OP_STAT_TRANSFER_GRODATA_6:                   
                    GYRO_DelaySecEnd=MillSecCount;
                    if(GYRO_DelaySecEnd<GYRO_DelaySecStart)
                    {
                        GYRO_DelaySecEnd|=0x00010000;
                    }                    
                    //time delay 5 s
                    if(GYRO_DelaySecEnd-GYRO_DelaySecStart>=DAQ_DELAY)
                    {
                        
                        NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_7;     
                    }                                    
                    break;
                //*****  read status
                case OP_STAT_TRANSFER_GRODATA_7: 
                    OperaBuf.FlagGROO=1;   
                    //read gyro status
                    tempSendBuf[0]=GRO_STAT_READ;
                    tempSendBuf[1]=0x7E01;
                    tempSendBuf[2]=0x0400;
                    tempSendBuf[3]=0x0000;
                    tempByte=CalcuCHKS_GRO(tempSendBuf+1,5);
                    tempSendBuf[3]+=tempByte;
                    tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                                         
                    DatSendLen=5;
                    ReponseRightBackState=OP_STAT_TRANSFER_GRODATA_8; 
                    ReponseErrorBackState=OP_STAT_TRANSFER_GRO_ERR_3; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                                     
                    break;
                
                //judge status word
                case OP_STAT_TRANSFER_GRODATA_8: 
                    if(0x01==OperaBuf.GRO_MEAS_STAT)    //寻北成功
                    {
                        NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_10;    
                    }
                    else if(0x02==OperaBuf.GRO_MEAS_STAT)  //寻北失败
                    {
                        NextGlobalOpState=OP_STAT_TRANSFER_GRO_ERR_4;     //记录错误标志                               
                    }
                    else
                    {
                        NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_9;     //判断是否超时
                        retryTimes++;  
                    }
                    break;
                case OP_STAT_TRANSFER_GRODATA_9:
                    if(retryTimes>=RETRY_TIMES)
                    {
                        NextGlobalOpState=OP_STAT_TRANSFER_GRO_ERR_5;     //记录错误标志    
                    }
                    else
                    {
                        NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_5;     //继续等待，重新查询
                    }
                    break;
                //*****  read data                        
                case OP_STAT_TRANSFER_GRODATA_10: 
                    OperaBuf.FlagGROO=1;   
                    //准备数据,get measure data from gyro
                    tempSendBuf[0]=GRO_DATA_READ;
                    tempSendBuf[1]=0x0004;
                    tempSendBuf[2]=0x0000;       
                    tempSendBuf[3]=CalcuCHKS(tempSendBuf+1,2);                                         
                    DatSendLen=4;
                    ReponseRightBackState=OP_STAT_TRANSFER_GRODATA_11; 
                    ReponseErrorBackState=OP_STAT_TRANSFER_GRO_ERR_6; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                    
                    break;
                case OP_STAT_TRANSFER_GRODATA_11: 
                    OperaBuf.FlagGROO=1;   
                    //准备数据,get Battery data from gro
                    tempSendBuf[0]=GRO_BATC_READ;
                    tempSendBuf[1]=0x0005;
                    tempSendBuf[2]=0x0000;
                    tempSendBuf[3]=0x0000;                    
                    tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                                         
                    DatSendLen=5;
                    ReponseRightBackState=OP_STAT_TRANSFER_GRODATA_12; 
                    ReponseErrorBackState=OP_STAT_TRANSFER_GRO_ERR_7; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                    
                    break;
                //**** POWER OFF
                case OP_STAT_TRANSFER_GRODATA_12:    
                    OperaBuf.FlagGROO=1;   
                    //power off
                    tempSendBuf[0]=GRO_POWER_OFF;
                    tempSendBuf[1]=0x7E02;
                    tempSendBuf[2]=0x0400;
                    tempSendBuf[3]=0x0000;
                    tempByte=CalcuCHKS_GRO(tempSendBuf+1,5);
                    tempSendBuf[3]+=tempByte;
                    tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                                         
                    DatSendLen=5;
                    ReponseRightBackState=OP_STAT_TRANSFER_GRODATA_13; 
                    ReponseErrorBackState=OP_STAT_TRANSFER_GRO_ERR_8; 
                    NextGlobalOpState=OP_STAT_STARTSEND; 
                    break;
                //**** send gyro data to mwd 
                case OP_STAT_TRANSFER_GRODATA_13:
                    //准备数据
                    tempSendBuf[0]=DIM_GRO_SEND; 
                    tempSendBuf[1]=5;   //Len
                    if(0!=OperaBuf.FlagGROO)
                    {
                        tempSendBuf[1] |=0x8000;
                        OperaBuf.GRO_DAT[0]=OperaBuf.ErrorCode;
                    }                        
                    for(i=0;i<5;i++) 
                    {   
                        tempSendBuf[i+2]=OperaBuf.GRO_DAT[i];
                    }
                	
                	tempSendBuf[7]=CalcuCHKS(tempSendBuf+1,6);                    
                    DatSendLen=8;
                    ReponseRightBackState=OP_STAT_IDEL;
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                     
                    break;
                 //*** power on fail error
                case OP_STAT_TRANSFER_GRO_ERR_1:
                    //准备数据
                    OperaBuf.FlagGROO=1;
                    OperaBuf.ErrorCode|=1;//error code                    
                    NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_13;     //send data              
                    break;                                                                   
               //*** start daq fail error
                case OP_STAT_TRANSFER_GRO_ERR_2:
                    //准备数据
                    OperaBuf.FlagGROO=1;
                    OperaBuf.ErrorCode|=2;//error code                     
                    NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_9;      //power off                   
                    break; 
               //*** read status error
                case OP_STAT_TRANSFER_GRO_ERR_3:
                    //准备数据
                    OperaBuf.FlagGROO=1;
                    OperaBuf.ErrorCode|=4;//error code                     
                    NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_9;      //power off                     
                    break;  
              //*** daq error
                case OP_STAT_TRANSFER_GRO_ERR_4:
                    //准备数据
                    OperaBuf.FlagGROO=1;
                    OperaBuf.ErrorCode|=8;//error code                     
                    NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_9;      //power off                  
                    break; 
              //*** time out error
                case OP_STAT_TRANSFER_GRO_ERR_5:
                    //准备数据
                    OperaBuf.FlagGROO=1;
                    OperaBuf.ErrorCode|=16;//error code                     
                    NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_9;      //power off                  
                    break; 
             //*** read data error
                case OP_STAT_TRANSFER_GRO_ERR_6:
                    //准备数据
                    OperaBuf.FlagGROO=1;
                    OperaBuf.ErrorCode|=32;//error code                     
                    NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_9;      //power off                   
                    break; 
              //*** read data error
                case OP_STAT_TRANSFER_GRO_ERR_7:
                    //准备数据
                    OperaBuf.FlagGROO=1;
                    OperaBuf.ErrorCode|=64;//error code                     
                    NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_9;      //power off  
                    break;
             //*** Power off error
                case OP_STAT_TRANSFER_GRO_ERR_8:
                    //准备数据
                    OperaBuf.FlagGROO=1;                             
                    OperaBuf.ErrorCode|=128;//error code                     
                    NextGlobalOpState=OP_STAT_TRANSFER_GRODATA_13;     //send data 
                    break;
                  //---------------------------------                           
                  //**********  END  GYRO   *********
                  //--------------------------------- 
                */        
                case OP_STAT_TRANSFER_REATIMEDATA:                 
                    if(1==FlagDBC)
                        NextGlobalOpState=OP_STAT_TRANSFER_DBCDATA;
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_ACPR;
                    //communication with DIM error process
                    if(1==OperaBuf.FlagDSTATUS)
                    {
                        ErrorCommCount++;
                        if(ErrorCommCount>20)
                        {
                            NextGlobalOpState = OP_STAT_ERROR_COMM;    
                        }
                    }  
                    OperaBuf.FlagDMBRO = 0;             //获取探管静态测量数据次数清零     
                    break;           
                //---------------------------------                           
                //**********   DBC   **************
                //---------------------------------
                case OP_STAT_TRANSFER_DBCDATA:
                    //准备数据
                    tempSendBuf[0]=DIM_DBC_SEND; 
                    tempSendBuf[1]=3;   //Len
                    
                    //电流
                  	tFloat=ReadCurrent();                	
                  	for(i=0;i<8;i++)
                  	{
                  	    if(tFloat<=COB_CPR_TABLE[i])
                  	        break;
                  	}
                  	if(i>7) i=7;  //数据超界
                  	tempWordBuf[0]=i;
                  	//CompressData(tempWordBuf,&tFloat,0.0,1000.0,8); 
                      //电压            	
                  	tFloat=ReadVoltage();
                  	for(i=0;i<4;i++)
                  	{
                  	     if(tFloat<=VOB_CPR_TABLE[i])
                  	        break;
                  	}
                	  if(i>3) i=3;  //数据超界
                	  tempWordBuf[1]=i;
                	  //使第3位动态变化
                	  OddEvenCount++;
                  	if(0==OddEvenCount%2)
                  	{
                  	   tempWordBuf[1]|=0B100; 
                  	} 
                  	else
                  	{
                  	    tempWordBuf[1]&=0B011;     
                  	}                	
                	  //CompressData(tempWordBuf+1,&tFloat,20.0,50.0,8); 
                	  //温度
                  	tFloat=ReadTemp();
                  	for(i=0;i<16;i++)
                  	{
                  	     if(tFloat<=TEMP_CPR_TABLE[i])
                  	        break;
                  	}
                  	if(i>15) i=15;  //数据超界
                  	tempWordBuf[2]=i;
                	                	
                  	//CompressData(tempWordBuf+2,&tFloat,-50.0,200.0,8); 
                  	//电池容量
                	
                  	for(i=0;i<13;i++)
                  	{
                  	     if(BatCap<=BAT_CPR_TABLE[i])
                  	        break;
                  	}
                  	if(i>12) i=12;  //数据超界
                  	tempWordBuf[3]=i;
                	                	
                  	//CompressData(tempWordBuf+3,&BatCap,0.0,60.0,8);
                    
                    pByte=tempSendBuf+2;                    	
                  	for(i=0;i<4;i++)
                  	{
                  	    *pByte=tempWordBuf[i] & 0xFF;
                  	    OperaBuf.DBC_DAT[i]=*pByte;
                  	    pByte++;    
                  	}                                        	
                  	tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                    
                    DatSendLen=5;
                    ReponseRightBackState=OP_STAT_TRANSFER_ACPR;
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;
                    
                
                //---------------------------------                           
                //**********   ACPR   **************
                //---------------------------------                                     
                case OP_STAT_TRANSFER_ACPR:
                    if(1==FlagACPR)
                        NextGlobalOpState=OP_STAT_TRANSFER_ACPR_1;
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_PMT;
                    break;
               
                case OP_STAT_TRANSFER_ACPR_1:
                    OperaBuf.FlagACPRR=1;   //Flag                     
                    //准备数据
                    tempSendBuf[0]=ACPR_GNDAT_READ;
                    tempSendBuf[1]=3;   //length 包括命令字和check
                    tempSendBuf[2]=CalcuCHKS_ACPR(tempSendBuf,2);                                        
                    DatSendLen=3;
                    ReponseErrorBackState=OP_STAT_TRANSFER_ACPR_2;
                    ReponseRightBackState=ReponseErrorBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;
                case OP_STAT_TRANSFER_ACPR_2:
                    //准备数据
                    tempSendBuf[0]=DIM_ACPR_SEND; 
                    tempSendBuf[1]=9;   //Len
                    if(0!=OperaBuf.FlagACPRR)
                        tempSendBuf[1] |=0x8000;                    
                	for(i=0;i<8;i++)
                	    tempSendBuf[i+2]=OperaBuf.ACPR_DAT[i+3];                	
                	tempSendBuf[10]=CalcuCHKS(tempSendBuf+1,9);                 	                   
                    DatSendLen=11;
                    ReponseErrorBackState=OP_STAT_TRANSFER_ACPR_3;
                    ReponseRightBackState=ReponseErrorBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                                        
                    break;
                case OP_STAT_TRANSFER_ACPR_3:
                    //准备数据
                    tempSendBuf[0]=ACPR_CPDAT_READ;
                    tempSendBuf[1]=3;   //length 包括命令字和check
                    tempSendBuf[2]=CalcuCHKS_ACPR(tempSendBuf,2);                                        
                    DatSendLen=3;
                    ReponseErrorBackState=OP_STAT_TRANSFER_ACPR_4;
                    ReponseRightBackState=ReponseErrorBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;
                case OP_STAT_TRANSFER_ACPR_4:
                    //准备数据
                    tempSendBuf[0]=DIM_ACPRCP_SEND; 
                    tempSendBuf[1]=5;   //Len
                    if(0!=OperaBuf.FlagACPRR)
                        tempSendBuf[1] |=0x8000;                    
                	for(i=0;i<4;i++)
                	    tempSendBuf[i+2]=OperaBuf.ACPR_DAT[i];                	
                	tempSendBuf[6]=CalcuCHKS(tempSendBuf+1,5);                 	                   
                    DatSendLen=7;
                    ReponseErrorBackState=OP_STAT_TRANSFER_PMT;
                    ReponseRightBackState=ReponseErrorBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND; 
                    
                    if(0==OperaBuf.FlagACPRR)
                    {
                        ErrorCommCountACPR = 0;
                        TempCountABC = 0;
                    }
                    else
                    {
                        ErrorCommCountACPR++;
                        if(ErrorCommCountACPR>30)
                        {
                            NextGlobalOpState = OP_STAT_ERROR_TBUS;    
                        }
                    }
                    break;
                //---------------------------------                           
                //**********   CGR   **************
                //---------------------------------                     
                case OP_STAT_TRANSFER_PMT:
                    if(1==FlagPMT)
                    {
                        NextGlobalOpState=OP_STAT_TRANSFER_PMT_2;
                    }    
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_AGR;
                    break;           
                case OP_STAT_TRANSFER_PMT_2:
                    OperaBuf.FlagPMTT=1;
                    //准备数据,get API data
                    tempSendBuf[0]=PMT_API_READ;                                        
                    DatSendLen=1;
                    ReponseRightBackState=OP_STAT_TRANSFER_PMT_3; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                    
                    break;                
                case OP_STAT_TRANSFER_PMT_3:    
                    //准备数据
                    tempSendBuf[0]=DIM_PMT_SEND; 
                    tempSendBuf[1]=3;   //Len
                    if(0!=OperaBuf.FlagPMTT)
                        tempSendBuf[1] |=0x8000;                   
                    
                    pByte=tempSendBuf+2;                    	
                  	for(i=0;i<3;i++)
                  	{
                  	    *pByte=OperaBuf.API[i];
                  	    pByte++;    
                  	    
                  	}                                     	
                	tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                    
                    DatSendLen=5;
                    ReponseRightBackState=OP_STAT_TRANSFER_AGR; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    
                    if(0==OperaBuf.FlagPMTT)
                    {
                        ErrorCommCountTbus=0;
                        TempCountABC=0;
                    }
                    else
                    {
                        ErrorCommCountTbus++;
                        if(ErrorCommCountTbus>30)
                        {
                            NextGlobalOpState = OP_STAT_ERROR_TBUS;    
                        }
                    }
                    break;
                //---------------------------------                           
                //**********   AGR   **************
                //---------------------------------      
                case OP_STAT_TRANSFER_AGR:
                    if(1==FlagAGR)
                        NextGlobalOpState = OP_STAT_TRANSFER_AGR_1;
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_DWPR;
                    break;           
                case OP_STAT_TRANSFER_AGR_1:
                    OperaBuf.FlagAGRT=1;
                    //准备数据,get API data
                    tempSendBuf[0]=AGR_API_READ; 
                    tempSendBuf[1]=0x0002;
                    tempSendBuf[2]=0x0004;
                    tempSendBuf[3]=0xFFF9;                                       
                    DatSendLen=4;
                    ReponseRightBackState=OP_STAT_TRANSFER_AGR_2; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                    
                    break;   
                case OP_STAT_TRANSFER_AGR_2: 
                    tempSendBuf[0] = DIM_AGR_SEND; 
                    tempSendBuf[1] = 6;   //Len
                    if(0!=OperaBuf.FlagAGRT)
                        tempSendBuf[1] |=0x8000;                   
                    
                    pByte=tempSendBuf+2;                    	
                  	for(i=0;i<3;i++)
                  	{
                  	    *pByte=OperaBuf.AGR[i];
                  	    pByte++;    
                  	    
                  	}  
                  	pByte=tempSendBuf+4;    
                  	for(i=0;i<4;i++)
                  	{
                  	    *pByte=OperaBuf.AGR[i+3];
                  	    pByte++;    
                  	    
                  	}                            	
                	tempSendBuf[6]=CalcuCHKS(tempSendBuf,6);                    
                    DatSendLen = 7;
                    ReponseRightBackState = OP_STAT_TRANSFER_DWPR; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;   
                    break;  
                    
                //-----------------------------------------
                //***********  DWPR  *********************
                //-----------------------------------------
                case OP_STAT_TRANSFER_DWPR:
                    if(1 == FlagDWPRT)          
                        NextGlobalOpState = OP_STAT_TRANSFER_DWPR_1;
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_NBT;                    
                    break;   
                case OP_STAT_TRANSFER_DWPR_1:
                    OperaBuf.FlagDWPR = 1;
                    //准备数据,get API data
                    tempSendBuf[0] = DWPR_RTD_READ; 
                    tempSendBuf[1] = 0x0005;
                    tempSendBuf[2] = 0x0001;
                    tempSendBuf[3] = 0x0007;
                    tempSendBuf[4] = 0x0009;
                    tempSendBuf[5] = 0xBCCF;                                       
                    DatSendLen=6;
                    ReponseRightBackState = OP_STAT_TRANSFER_DWPR_2; 
                    ReponseErrorBackState = ReponseRightBackState; 
                    NextGlobalOpState = OP_STAT_STARTSEND;                    
                    break;   
                case OP_STAT_TRANSFER_DWPR_2: 
                    tempSendBuf[0] = DIM_DWPR_SEND; //DIM_DWPR_SEND
                    tempSendBuf[1] = 7;   //Len
                    if(0!=OperaBuf.FlagDWPR)
                        tempSendBuf[1] |=0x8000;                   
                    
                    pWord = &tempSendBuf[2];                    	
                  	for(i=0;i<3;i++)
                  	{
                  	    *pWord = OperaBuf.DWPR[i];
                  	    pWord++;    
                  	    
                  	}  
                  		
                	tempSendBuf[7] = CalcuCHKS(tempSendBuf,7);                    
                    DatSendLen=8;
                    ReponseRightBackState=OP_STAT_TRANSFER_NBT; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;   
                    break;      
                //------------------------------------                           
                //**********   NBIG  **************
                //------------------------------------      
                case OP_STAT_TRANSFER_NBT:
                    if(1==FlagNBT)          
                        NextGlobalOpState = OP_STAT_TRANSFER_NBT_1;
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_PREDATA;                    
                    break;  
                case OP_STAT_TRANSFER_NBT_1:  
                    tempSendBuf[0] = NBT_MDATA_READ;
                    DatSendLen=1;
                    ReponseRightBackState=OP_STAT_TRANSFER_NBT_2; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;    
                    break;    
                case OP_STAT_TRANSFER_NBT_2:  
                    tempSendBuf[0] = DIM_NBMD_SEND;
                    tempSendBuf[1] = 0x000B;
                    if(0 != OperaBuf.FlagNBIG)
                        tempSendBuf[1] |=0x8000;   
                    if((RealTimeMode != (DimStatus[0]&0xFF)) && (NBGR_SMDAT_FLG > 0))
                    {
                        pWord = &OperaBuf.NBGR_SMDAT[0];
                        for(i=2;i<11;i++)
                        {
                            tempSendBuf[i] = *pWord;
                            pWord++;
                        }         
                    }
                    else
                    {
                        pWord = &OperaBuf.NBGR_MDAT[0];
                        for(i=2;i<11;i++)
                        {
                            tempSendBuf[i] = *pWord;
                            pWord++;
                        }
                        if(RealTimeMode == (DimStatus[0]&0xFF))
                            NBGR_SMDAT_FLG = 0;
                    } 
                    tempSendBuf[11] = 0x0000;//
                    tempSendBuf[12] = CalcuCHKS(tempSendBuf+1,11);//
                   
                    DatSendLen=13;
                    ReponseRightBackState=OP_STAT_TRANSFER_NBT_3; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;
                case OP_STAT_TRANSFER_NBT_3:  
                    tempSendBuf[0] = NBT_GDATA_READ;
                    OperaBuf.FlagNBIG = 1;
                    DatSendLen=1;
                    ReponseRightBackState=OP_STAT_TRANSFER_NBT_4; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;    
                    break;
                case OP_STAT_TRANSFER_NBT_4:  
                    tempSendBuf[0] = DIM_NBGD_SEND;
                    tempSendBuf[1] = 0x0009;
                    if(0 != OperaBuf.FlagNBIG)
                        tempSendBuf[1] |=0x8000;     
                    pWord = &OperaBuf.NBGR_GDAT[0];
                    for(i=2;i<8;i++)
                    {
                        tempSendBuf[i] = *pWord;
                        pWord++;
                    }
                    if((RealTimeMode != (DimStatus[0]&0xFF)) && (NBGR_SMDAT_FLG > 0))
                    {
                        tempSendBuf[6] = OperaBuf.NBGR_SMDAT[2];//
                        tempSendBuf[7] = OperaBuf.NBGR_SMDAT[3];//
                        
                    }
                    tempSendBuf[8] = 0x0000;//
                    tempSendBuf[9] = 0x0000;//
                    tempSendBuf[10] = CalcuCHKS(tempSendBuf+1,9);;
                    DatSendLen=11;
                    
                    ReponseRightBackState=OP_STAT_TRANSFER_PREDATA; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;        
                    break;
                
                         
                //---------------------------------                           
                //**********   PRE   **************
                //---------------------------------   
                case OP_STAT_TRANSFER_PREDATA:
                    if(1==FlagPRE)
                        NextGlobalOpState=OP_STAT_TRANSFER_PREDATA_1;
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_DIAMDATA;                    
                    break;
                case OP_STAT_TRANSFER_PREDATA_1:                     
                    //准备数据
                    tempSendBuf[0]=DIM_PRE_SEND; 
                    tempSendBuf[1]=3;   //Len
                    //内压
                  	tFloat=ReadInnerPre();
                  	tempSendBuf[2]=(word)(tFloat/5.0+0.5);                
                  	OperaBuf.InnPRE=tempSendBuf[2];
                    //外压            	
                  	tFloat=ReadannPre();
                  	tempSendBuf[3]=(word)(tFloat/5.0+0.5); 
                  	OperaBuf.AnnPRE=tempSendBuf[3];
                  	
                  	tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                    
                    DatSendLen=5;
                    ReponseRightBackState=OP_STAT_TRANSFER_DIAMDATA;
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                     
                    break; 
                    
                //---------------------------------                           
                //**********   USM   **************
                //--------------------------------- 
                    
                case OP_STAT_TRANSFER_DIAMDATA:
                    if(1==FlagDIAM)
                        NextGlobalOpState = OP_STAT_TRANSFER_DIAMDATA_1;
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_VSTDATA;                    
                    break;
                case OP_STAT_TRANSFER_DIAMDATA_1:                   
                    //准备数据
                    tempSendBuf[0]=DIM_UDT_SEND; 
                    tempSendBuf[1]=3;   //Len
                    if(0!=OperaBuf.FlagUSM)
                        tempSendBuf[1] |=0x8000;                    
                	  tempSendBuf[2] = OperaBuf.USM_DAT;                 	               	
                	  //3 备用
                	  tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                    
                    DatSendLen=5;
                    ReponseErrorBackState=OP_STAT_TRANSFER_VSTDATA;
                    ReponseRightBackState=ReponseErrorBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    
                    OperaBuf.FlagUSM=1;   //reset the flag
                    break; 
                    
                //------------------------------------------------------                           
                //**********   VST振动模块  **************
                //------------------------------------------------------     
                case OP_STAT_TRANSFER_VSTDATA:
                    if(1==FlagVST)
                        NextGlobalOpState=OP_STAT_TRANSFER_VSTDATA_1;
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_NEUT;                    
                    break;
                case OP_STAT_TRANSFER_VSTDATA_1:    ///  数据需要重新处理             
                    //准备数据
                    tempSendBuf[0] = DIM_VST_SEND; 
                    tempSendBuf[1] = 4;   //Len
                    if(0 != OperaBuf.FlagVSM) 
                    {
                         tempSendBuf[1] |=0x8000;     //标记无效
                    }
                    tempSendBuf[2] = (OperaBuf.VST_DAT[0] & 0x000F)<<12|(OperaBuf.VST_DAT[1] & 0x000F)<<8|(OperaBuf.VST_DAT[2] & 0x000F)<<4;
                    tempSendBuf[3] = (OperaBuf.VST_DAT[3] & 0x0001)<<8|OperaBuf.VST_DAT[4] & 0x00FF;
                    
                    tempSendBuf[4] = 0x0000;    // 备用
                    tempSendBuf[5] = CalcuCHKS(tempSendBuf+1,4);                    
                    DatSendLen=6;
                    ReponseRightBackState = OP_STAT_TRANSFER_NEUT;
                    ReponseErrorBackState = ReponseRightBackState; 
                    NextGlobalOpState = OP_STAT_STARTSEND;      
                    break;                               
                //---------------------------------                                                           
                //**********   中子   **************
                //---------------------------------       
                case OP_STAT_TRANSFER_NEUT:
                    if(1==FlagINP)
                        NextGlobalOpState=OP_STAT_TRANSFER_NEUT_1;
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_ACOU;                    
                    break;
                case OP_STAT_TRANSFER_NEUT_1:  
                    OperaBuf.FlagINPT=1;
                    //准备数据
                    tempSendBuf[0]=NEUT_data_READ;   //读孔隙度数据                                     
                    DatSendLen=1;
                    ReponseRightBackState=OP_STAT_TRANSFER_NEUT_3; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                     
                    break;
                case OP_STAT_TRANSFER_NEUT_3:
                    //准备数据
                    tempSendBuf[0]=DIM_NEUT_SEND; 
                    tempSendBuf[1]=4;   //Len
                    if(0!=OperaBuf.FlagINPT)
                        tempSendBuf[1] |=0x8000;
                    
                    tempSendBuf[2]=OperaBuf.Porosity;
                    tempSendBuf[3]=OperaBuf.HoleSize;
                    tempSendBuf[4]=0x0000;     //备用
                  	tempSendBuf[5]=CalcuCHKS(tempSendBuf+1,4);                    
                    DatSendLen=6;
                    ReponseRightBackState=OP_STAT_TRANSFER_ACOU; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;                    
                //---------------------------------                           
                //**********   声波   **************
                //---------------------------------         
                case OP_STAT_TRANSFER_ACOU:
                    if(1==FlagACOU)
                        NextGlobalOpState=OP_STAT_TRANSFER_ACOU_1;
                    else 
                        NextGlobalOpState = OP_STAT_TRANSFER_DENS;                                                
                    break;
                case OP_STAT_TRANSFER_ACOU_1:
                    OperaBuf.FlagMAST = 1;
                    //准备数据
                    tempSendBuf[0]=ACOU_DAT_READ;                                        
                    DatSendLen=1;
                    ReponseRightBackState=OP_STAT_TRANSFER_ACOU_2; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                     
                    break;
                case OP_STAT_TRANSFER_ACOU_2:
                    //准备数据
                    tempSendBuf[0]=DIM_ACOU_SEND; 
                    tempSendBuf[1]=4;   //Len
                    if(0!=OperaBuf.FlagMAST)
                        tempSendBuf[1] |=0x8000;
                    for(i=0;i<3;i++)
                    {
                        tempSendBuf[i+2]=OperaBuf.MAST_DAT[i];
                    }                    
                	  tempSendBuf[5]=CalcuCHKS(tempSendBuf+1,4);                    
                    DatSendLen=6;  
                    ReponseRightBackState= OP_STAT_TRANSFER_DENS; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;                    
                //---------------------------------                           
                //**********   密度   **************
                //---------------------------------  
                case OP_STAT_TRANSFER_DENS:
                    if(1==FlagLDI)
                        NextGlobalOpState = OP_STAT_TRANSFER_DENS_1;
                    else 
                        NextGlobalOpState = OP_STAT_TRANSFER_SAKDATA;                                                
                    break;
                case OP_STAT_TRANSFER_DENS_1:
                    OperaBuf.FlagLDIT=1;
                    //准备数据
                    tempSendBuf[0]=LDI_DAT_READ;                                        
                    DatSendLen=1;
                    ReponseRightBackState=OP_STAT_TRANSFER_DENS_2; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                     
                    break;
                case OP_STAT_TRANSFER_DENS_2:
                    //准备数据
                    tempSendBuf[0] = DIM_DENS_SEND; 
                    tempSendBuf[1] = 5;   //Len
                    if(0!=OperaBuf.FlagLDIT)
                        tempSendBuf[1] |=0x8000;
                    for(i=0;i<3;i++)
                    {
                        tempSendBuf[i+2]=OperaBuf.LDI_DAT[i];
                    }              
                    tempSendBuf[5]=0x0000;     //备用      
                	  tempSendBuf[6]=CalcuCHKS(tempSendBuf+1,5);                    
                    DatSendLen=7;  
                    ReponseRightBackState= OP_STAT_TRANSFER_SAKDATA; 
                    ReponseErrorBackState=ReponseRightBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;   
      
                //---------------------------------                           
                //**********   SFU   **************
                //---------------------------------                     
                case OP_STAT_TRANSFER_SAKDATA:
                    if(1==FlagSFU)
                        NextGlobalOpState = OP_STAT_SFU_MONITOR;
                    else
                        NextGlobalOpState = OP_STAT_TRANSFER_TVELDATA;
                    break;
               case OP_STAT_SFU_MONITOR:
                    tempSendBuf[0] = MONITOR_RST_DAT;
                    DatSendLen=1;
                    ReponseErrorBackState = OP_STAT_TRANSFER_SAKDATA_1; 
                    ReponseRightBackState = ReponseErrorBackState;
                    NextGlobalOpState = OP_STAT_STARTSEND;                    
                    break;
                case OP_STAT_TRANSFER_SAKDATA_1:
                    OperaBuf.FlagSNK=1;   //Flag
                    //准备数据
                    tempSendBuf[0]=RST_SAK_READ;     //READ SFU simplified parameters                                     
                    DatSendLen=1;
                    ReponseErrorBackState=OP_STAT_TRANSFER_SAKDATA_2; 
                    ReponseRightBackState=ReponseErrorBackState;
                    NextGlobalOpState=OP_STAT_STARTSEND;                    
                    break;
                case OP_STAT_TRANSFER_SAKDATA_2:
                    //准备数据
                    tempSendBuf[0]=DIM_SAK_SEND;   //DIM上传
                    tempSendBuf[1]=9;   //Len
                    if(0!=OperaBuf.FlagSNK)
                        tempSendBuf[1] |=0x8000;
                    pWord=OperaBuf.SNK_DAT;
                    for(i=0;i<7;i++)
                    {
                        tempSendBuf[i+2]=*pWord;
                        pWord++;
                    }
                    
                	  tempSendBuf[10]=CalcuCHKS(tempSendBuf+1,9);                    
                    DatSendLen=11;
                    ReponseErrorBackState=OP_STAT_TRANSFER_SCDLDATA_1; 
                    ReponseRightBackState=ReponseErrorBackState;
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;               
                case OP_STAT_TRANSFER_SCDLDATA_1:
                    OperaBuf.FlagSNK=1;   //Flag
                    //准备数据
                    tempSendBuf[0]=RST_SCDL_READ;       //F481,Read CDL Downlink Data                                 
                    DatSendLen=1;
                    ReponseErrorBackState=OP_STAT_TRANSFER_SCDLDATA_2; 
                    ReponseRightBackState=ReponseErrorBackState;
                    NextGlobalOpState=OP_STAT_STARTSEND; 
                    break;
                case OP_STAT_TRANSFER_SCDLDATA_2:
                    //准备数据
                    tempSendBuf[0]=DIM_RSCDL_SEND;    //AC11,实时状态下传送CDL数据
                    tempSendBuf[1]=3;   //Len
                    if(0!=OperaBuf.FlagSNK)
                        tempSendBuf[1] |=0x8000;
                    tempSendBuf[2]=OperaBuf.Mode;
                    tempSendBuf[3]=OperaBuf.CONFIG;
                                            
                	  tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                    
                    DatSendLen=5;
                    ReponseErrorBackState=OP_STAT_TRANSFER_TVELDATA; 
                    ReponseRightBackState=ReponseErrorBackState;
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;
                //---------------------------------                           
                //**********   TGC   **************
                //---------------------------------                 
                case OP_STAT_TRANSFER_TVELDATA:
                    if(1==FlagTGC)
                        NextGlobalOpState=OP_STAT_TRANSFER_TVELDATA_1;
                    else
                        NextGlobalOpState = OP_STAT_SAVE_DATA;
                    break;
                case OP_STAT_TRANSFER_TVELDATA_1:
                    OperaBuf.FlagTGCC=1;   //Flag                                                  
                    //准备数据
                    tempSendBuf[0]=RST_TVEL_READ;  //读转速                                      
                    DatSendLen=1;
                    ReponseErrorBackState=OP_STAT_TRANSFER_TVELDATA_2;
                    ReponseRightBackState=ReponseErrorBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;
                case OP_STAT_TRANSFER_TVELDATA_2:
                    //准备数据
                    tempSendBuf[0]=DIM_TVEL_SEND;  //DIM上传
                    tempSendBuf[1]=3;   //Len
                    if(0!=OperaBuf.FlagTGCC)
                        tempSendBuf[1] |=0x8000;
                    pWord=&OperaBuf.TVEL;
                  	tempSendBuf[2]=*pWord;
                  	pWord++;
                  	tempSendBuf[3]=*pWord;
                  	tempSendBuf[3] &=0xFF00;
                  	tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                    
                    DatSendLen=5;
                    ReponseErrorBackState=OP_STAT_SYN_DIM_TGC_1;
                    ReponseRightBackState=ReponseErrorBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break;                     
                
                case  OP_STAT_SYN_DIM_TGC_1:                    
                    OperaBuf.FlagTGCC=1;   //Flag
                    //准备数据
                    tempSendBuf[0]=RST_TSTAT_READ;   //E181,Get Downlink Status                                     
                    DatSendLen=1;
                    ReponseErrorBackState=OP_STAT_SYN_DIM_TGC_2;
                    ReponseRightBackState=ReponseErrorBackState; 
                    NextGlobalOpState=OP_STAT_STARTSEND;                     
                    break;
                case OP_STAT_SYN_DIM_TGC_2:
                    //准备数据
                    tempSendBuf[0]=DIM_TSTAT_SEND; 
                    tempSendBuf[1]=3;   //Len
                    if(0!=OperaBuf.FlagTGCC)
                    {
                        TGC_TbusErrorCount++;
                        tempSendBuf[1] |=0x8000;
                        if(TGC_TbusErrorCount>3)     //连续3次Error清TGC状态
                        {
                            tempSendBuf[1] =3;
                            OperaBuf.TSTATUS=0; 
                            TGC_TbusErrorCount=0;   
                        }  
                    }
                    else
                    {
                        TGC_TbusErrorCount=0;
                    }
                    tempSendBuf[2]=OperaBuf.TSTATUS;
                    tempSendBuf[3]=tempSendBuf[2];
                    tempSendBuf[4]=tempSendBuf[2];                         
                                       
                    DatSendLen=5;
                    
                    if((RealTimeMode==DimStatus[0])||(CDL_Upload==DimStatus[0])||(CDL_Waiting==DimStatus[0])||(CDL_WaitData==DimStatus[0]))
                    {
                        
                        ReponseErrorBackState=OP_STAT_SYN_DIM_TGC_3; 
                        ReponseRightBackState=ReponseErrorBackState;
                        NextGlobalOpState=OP_STAT_STARTSEND;    
                    }
                    else
                    {
                        NextGlobalOpState=OP_STAT_SYN_DIM_TGC_3;  
                    }                     
                    break;
                case  OP_STAT_SYN_DIM_TGC_3:
                    //准备数据
                    tempSendBuf[0]=RST_DSTAT_SEND;  //SEND DIM STATUS
                    tempSendBuf[1]=DimStatus[0];
                    tempSendBuf[2]=DimStatus[0];                    
                    DatSendLen=3;
                    ReponseErrorBackState=OP_STAT_SAVE_DATA; 
                    ReponseRightBackState=ReponseErrorBackState;
                    NextGlobalOpState=OP_STAT_STARTSEND; 
                    break;
                    
                //---------------------------------------------------------------------------                           
                //**********   Save Data related   *******
                //--------------------------------------------------------------------------- 
                case  OP_STAT_SAVE_DATA:                    
                    GetRTC(&OperSaveDat.RTC); //RTC
                    OperSaveDat.DataID = 128;
                    OperSaveDat.FlagDIM = OperaBuf.FlagDSTATUS;
                    OperSaveDat.DIM_STATUS = OperaBuf.DSTATUS;
                    OperSaveDat.Status = StatusAll; //DBC
                    OperSaveDat.FlagUSM = OperaBuf.FlagUSM;   //USM
                    OperSaveDat.FlagVSM = OperaBuf.FlagVSM; //VSM
                    OperSaveDat.FlagDMBRO = OperaBuf.FlagDMBRO;   //角差广播
                    OperSaveDat.FlagACPRR = OperaBuf.FlagACPRR;
                    OperSaveDat.FlagPMTT = OperaBuf.FlagPMTT;
                    OperSaveDat.CGR_APIC = OperaBuf.API[2];
                    OperSaveDat.FlagTGCC = OperaBuf.FlagTGCC;
                    OperSaveDat.TGC_STATUS = OperaBuf.TSTATUS;
                    OperSaveDat.FlagSFUU = OperaBuf.FlagSNK;
                    OperSaveDat.SFU_STATUS = OperaBuf.SNK_DAT[5];
                    OperSaveDat.RST_MONITOR = ToolMonitor.RST;
                    OperSaveDat.TGC_ECHO = OperaBuf.FlagTGCC;
                    OperSaveDat.FlagINPT = OperaBuf.FlagINPT;     //中子
                    OperSaveDat.FlagLDIT = OperaBuf.FlagLDIT;     //密度
                    
                    OperSaveDat.FlagAGRT = OperaBuf.FlagAGRT; 
                    OperSaveDat.FlagNBIG = OperaBuf.FlagNBIG;
                    OperSaveDat.FlagMAST = OperaBuf.FlagMAST;
                    OperSaveDat.FlagDART = OperaBuf.FlagDART;
                    OperSaveDat.FlagIFPT = OperaBuf.FlagIFPT;
                    OperSaveDat.FlagDWPR = OperaBuf.FlagDWPR;
                    WriteFDataU(&DmBlankAdr[1],&OperSaveDat,32);
                    NextGlobalOpState = OP_STAT_IDEL;
                    break;      
      
                //------------------------------------------------                           
                //**********   CDL related   *******
                //------------------------------------------------ 
                case OP_STAT_READ_CDLDATA:
                    tempSendBuf[0]=READ_TGC_CDLDATD;                                        
                    DatSendLen=1;
                    ReponseErrorBackState = OP_STAT_TRANSFER_CDLDATA; 
                    ReponseRightBackState = ReponseErrorBackState;
                    NextGlobalOpState=OP_STAT_STARTSEND; 
                    break;
                /*    
                case OP_STAT_TRANSFER_CDLDATA_DIM:
                    //准备数据
                    tempSendBuf[0]=DIM_SCDL_SEND; //A711
                    tempSendBuf[1]=3;   //Len
                    if(0!=OperaBuf.FlagSNK)
                        tempSendBuf[1] |=0x8000;
                    tempSendBuf[2]=OperaBuf.Mode;
                    tempSendBuf[3]=OperaBuf.CONFIG;
                                            
                  	tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                    
                    DatSendLen=5;
                    ReponseErrorBackState=OP_STAT_TRANSFER_CDLDATA; 
                    ReponseRightBackState=ReponseErrorBackState;
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break; 
                 */                       
                case OP_STAT_TRANSFER_CDLDATA:
                    if(1==FlagSFU)
                        NextGlobalOpState=OP_STAT_TRANSFER_CDLDATA_1;
                    else
                        NextGlobalOpState = OP_STAT_IDEL; 
                    break;                                   
                case OP_STAT_TRANSFER_CDLDATA_1:
                    OperaBuf.FlagSNK=1;   //Flag
                    //准备数据
                    tempSendBuf[0]=RST_SCDL_READ;   //F481                                     
                    DatSendLen=1;
                    ReponseErrorBackState=OP_STAT_TRANSFER_CDLDATA_2; 
                    ReponseRightBackState=ReponseErrorBackState;
                    NextGlobalOpState=OP_STAT_STARTSEND; 
                    break;
                case OP_STAT_TRANSFER_CDLDATA_2:
                    //准备数据
                    tempSendBuf[0]=DIM_SCDL_SEND; //A711
                    tempSendBuf[1]=3;   //Len
                    if(0!=OperaBuf.FlagSNK)
                        tempSendBuf[1] |=0x8000;
                    tempSendBuf[2]=OperaBuf.Mode;
                    tempSendBuf[3]=OperaBuf.CONFIG;
                                            
                  	tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);                    
                    DatSendLen=5;
                    ReponseErrorBackState=OP_STAT_IDEL; 
                    ReponseRightBackState=ReponseErrorBackState;
                    NextGlobalOpState=OP_STAT_STARTSEND;
                    break; 
                    
                //-----------------------------------------------------------                           
                //**********   QUIET related   ****
                //-----------------------------------------------------------                              
                case OP_STAT_QUIET:
                    WriteLogDat(9,&DBCLogDat);  //befor enter  into sleep mode
                    CloseTbus();                                  
                    QuietMillSecStart=MillSecCount;            
                    NextGlobalOpState=OP_STAT_QUIET_1;
                    break;
                case OP_STAT_QUIET_1:
                    QuietMillSecEnd=MillSecCount;                
                    if(QuietMillSecEnd<QuietMillSecStart)
                    {
                        QuietMillSecEnd|=0x00010000;
                    }                    
                    //time delay 200 ms
                    if(QuietMillSecEnd-QuietMillSecStart>=200)
                    {
                        WriteLogDat(10,&DBCLogDat);  //enter  into sleep mode  
                        FlagInQuite=1;
                        NextGlobalOpState=OP_STAT_IDEL;                             
                    }
                    break;                   
                case OP_STAT_QUIT_QUIET:
                    FlagInQuite=0;
                    WriteLogDat(11,&DBCLogDat);  //befor exit sleep mode
                    CloseLbus();
                    QuietMillSecStart=MillSecCount;                   
                    NextGlobalOpState=OP_STAT_QUIT_QUIET_1;
                    break;
                case OP_STAT_QUIT_QUIET_1:
                    QuietMillSecEnd=MillSecCount;                
                    if(QuietMillSecEnd<QuietMillSecStart)
                    {
                        QuietMillSecEnd|=0x00010000;
                    }                    
                    //time delay 60 s
                    if(QuietMillSecEnd-QuietMillSecStart>=60000)
                    {
                        WriteLogDat(12,&DBCLogDat);  //before open Lbus
                        OpenLbus();
                        QuietMillSecStart=MillSecCount; 
                        NextGlobalOpState=OP_STAT_QUIT_QUIET_2;     
                    }
                    break;
                case OP_STAT_QUIT_QUIET_2:
                    QuietMillSecEnd=MillSecCount;                
                    if(QuietMillSecEnd<QuietMillSecStart)
                    {
                        QuietMillSecEnd|=0x00010000;
                    }                    
                    //time delay 200 s
                    if(QuietMillSecEnd-QuietMillSecStart>=200)
                    {
                        WriteLogDat(13,&DBCLogDat);  
                        OpenTbus();
                        QuietMillSecStart=MillSecCount; 
                        NextGlobalOpState=OP_STAT_QUIT_QUIET_3;     
                    }
                    break;
                case  OP_STAT_QUIT_QUIET_3:
                    QuietMillSecEnd=MillSecCount;                
                    if(QuietMillSecEnd<QuietMillSecStart)
                    {
                        QuietMillSecEnd|=0x00010000;
                    }                    
                    //time delay 200 s
                    if(QuietMillSecEnd-QuietMillSecStart>=200)
                    {
                        WriteLogDat(14,&DBCLogDat); 
                        NextGlobalOpState=OP_STAT_IDEL;     
                    }
                    break;
                    
                //-------------------------------------------------------                           
                //**********   Communication error handling   *******
                //------------------------------------------------------- 
                
                //-------------Lbus ERROR--------------------------------                
                case OP_STAT_ERROR_COMM:
                    if(1!=FlagBigCurrentProcess && 1!=FlagLowVoltageProcess)
                    {
                        WriteLogDat(15,&DBCLogDat);  //befor process
                        CloseLbus();                                  
                        ComErrorSecStart=SysTime;            
                        NextGlobalOpState=OP_STAT_ERROR_COMM_1;   
                    }
                    else
                    {
                        NextGlobalOpState=OP_STAT_IDEL;   
                    }                    
                    break;
                case OP_STAT_ERROR_COMM_1:
                    ComErrorSecEnd=SysTime;                
                    if(ComErrorSecEnd<ComErrorSecStart)
                    {
                        ComErrorSecEnd|=0x00010000;
                    }                    
                    //time delay 60 s
                    if(ComErrorSecEnd-ComErrorSecStart>=60)
                    {
                        WriteLogDat(16,&DBCLogDat);  
                        OpenLbus();
                        ComErrorSecStart=SysTime; 
                        NextGlobalOpState=OP_STAT_ERROR_COMM_2;     
                    }
                    break;                    
                case  OP_STAT_ERROR_COMM_2:
                    ComErrorSecEnd=SysTime;                
                    if(ComErrorSecEnd<ComErrorSecStart)
                    {
                        ComErrorSecEnd|=0x00010000;
                    }                    
                    //time delay 60 s
                    if(ComErrorSecEnd-ComErrorSecStart>=2)
                    {
                        WriteLogDat(17,&DBCLogDat);  
                        ErrorCommCount=0;
                        NextGlobalOpState=OP_STAT_IDEL;     
                    }
                    break;
                                        
                //--------- Tbus Error ---------------------------------
                
                case OP_STAT_ERROR_TBUS:                                        
                    if(1!=FlagBigCurrentProcess && 1!=FlagLowVoltageProcess)
                    {
                        
                        TempCountABC++;
                        
                        WriteLogDat(19,&DBCLogDat);  //befor process
                        CloseTbus();                                  
                        ComErrorSecStart = SysTime;            
                        NextGlobalOpState = OP_STAT_ERROR_TBUS_1;
                        
                        //SendD(TempCountABC);
                        if(TempCountABC>4)
                        {
                            NextGlobalOpState=OP_STAT_ERROR_TBUS_3; 
                            TempCountABC=0;       
                        }   
                    }
                    else
                    {
                        NextGlobalOpState=OP_STAT_IDEL;   
                    }                                  
                    break;
                case OP_STAT_ERROR_TBUS_1:
                    ComErrorSecEnd=SysTime;                
                    if(ComErrorSecEnd<ComErrorSecStart)
                    {
                        ComErrorSecEnd|=0x00010000;
                    }                    
                    //time delay 30 s
                    if(ComErrorSecEnd-ComErrorSecStart>=30)
                    {
                        WriteLogDat(20,&DBCLogDat);  
                        OpenTbus();
                        ComErrorSecStart=SysTime; 
                        NextGlobalOpState=OP_STAT_ERROR_TBUS_2;     
                    }
                    break;
                case  OP_STAT_ERROR_TBUS_2:
                    ComErrorSecEnd=SysTime;                
                    if(ComErrorSecEnd<ComErrorSecStart)
                    {
                        ComErrorSecEnd|=0x00010000;
                    }                    
                    //time delay 60 s
                    if(ComErrorSecEnd-ComErrorSecStart>=60)
                    {
                        WriteLogDat(21,&DBCLogDat);  
                        ErrorCommCountTbus = 0;
                        ErrorCommCountACPR = 0;
                        //broadcast time switch
                        TimeBroState=1;                        
                        
                        NextGlobalOpState=OP_STAT_IDEL;     
                    }
                    break;
                case OP_STAT_ERROR_TBUS_3:
                    ComErrorSecEnd=SysTime;                
                    if(ComErrorSecEnd<ComErrorSecStart)
                    {
                        ComErrorSecEnd|=0x00010000;
                    }                    
                    //time delay 300 s
                    if(ComErrorSecEnd-ComErrorSecStart>=300)
                    {
                        WriteLogDat(22,&DBCLogDat);  
                        OpenTbus();
                        
                        ComErrorSecStart=SysTime; 
                        NextGlobalOpState=OP_STAT_ERROR_TBUS_2;     
                    }
                    break;
                    
                //---------------------------------                           
                //**********   IDEL   *************
                //---------------------------------                                             
                case OP_STAT_IDEL:
                    if(SysTime0 != SysTime)
                    {
                        NextGlobalOpState = OP_STAT_DEFAULT;     
                    }
                    break;                   
                    
                default:
                    NextGlobalOpState=OP_STAT_DEFAULT; 
                    break;
            } 
             
            
        }
        //****************************************** 
        //end DBC do work as BC 
        //****************************************** 
        
        //=====================================================================
        //*                                                                   *
        //* caculate the battery capacity                                     *
        //*                                                                   *
        //=====================================================================
        
        BatSum+=(SumFilter[2]>>6);
        I_Count++;
        if(BatSum>45000000)  //约0.1AH        
        {
            BatCap-=(float)(BatSum/I_Count*3.3*500/4096*(BatTimerCounter-StartBatCounter)/1000.0/3600.0);                    
            BatSum=0.0;
            StartBatCounter=BatTimerCounter;    //reset timer counter
            I_Count=0;
        }
        //end caculate
        //****************************************** 
        
        
        //******************************************
        //volatage current check mode         
        switch (VI_Check_State)
        {
            
            //step 0 get data
            case 0:                
                VI[1]=ReadCurrent();
                VI[0]=ReadVoltage();
                VI_Check_State=1;
                //for delay time count
                VI_CheckMillSecStart=MillSecCount;
                break;
            //step 1 compare with threshold 
            case 1:
                if(VI[1]>Threshold_BC)
                {
                    VI_Check_State=2;                    
                }
                else
                {
                    BigCur_Count=0;
                    
                    if(VI[0]<Threshold_LV)
                    {
                        VI_Check_State=3;
                    }
                    else
                    {
                        SmallVol_Count=0;
                        VI_Check_State=4; 
                           
                    }
                    
                }                
                break;
            //step 2 change the process flag for future process
            case 2:
                BigCur_Count++;
                if(BigCur_Count>10)      //100ms
                {
                    FlagBigCurrentProcess=1;    //enter into Big Current Process        
                }
                VI_Check_State=4;                
                break;
            case 3:
                SmallVol_Count++;
                if(SmallVol_Count>100)    //1s
                {
                    FlagLowVoltageProcess=1;
                }
                VI_Check_State=4;
                break;
                
            //step3 time Delay
            case 4:
                VI_CheckMillSecEnd=MillSecCount;                
                if(VI_CheckMillSecEnd<VI_CheckMillSecStart)
                {
                    VI_CheckMillSecEnd|=0x00010000;
                }
                
                //time delay 10 ms
                if(VI_CheckMillSecEnd-VI_CheckMillSecStart>=10)
                {
                    VI_Check_State=0;    
                }
                break;
            default:
                break;
        }
        
        //High Current Process
        if(1==FlagBigCurrentProcess)
        {
            switch(BigCurrentProcesStep)
            {
                case 0:
                    WriteLogDat(1,&DBCLogDat);  //记录大电流数据 
                    BigCurrentProcesStep=1;
                    
                    BigCurrentBeforJudge=0;
                    if(1==Status_LBUS)
                    {
                        BigCurrentBeforJudge|=1;    
                    }
                    if(1==Status_TBUS)
                    {
                        BigCurrentBeforJudge|=2;    
                    }
                    break;
                case 1:
                    CloseTbus();
                    CloseLbus();
                    BigCurrentProcesStep=2;
                    BigCurMillSecStart=MillSecCount;
                    break;
                //delay 1 s
                case 2:
                    BigCurMillSecEnd=MillSecCount;
                    if(BigCurMillSecEnd<BigCurMillSecStart)
                    {
                        BigCurMillSecEnd|=0x00010000;
                    }
                    //time delay 1 s
                    if(BigCurMillSecEnd-BigCurMillSecStart>=1000)
                    {
                        BigCurrentProcesStep=3;
                    }                                             
                    break;
                case 3:
                    WriteLogDat(2,&DBCLogDat); //大电流处理--上下全关时数据
                    BigCurrentProcesStep=4;
                    break;
                case 4:
                    OpenLbus();
                    BigCurrentProcesStep=5;
                    BigCurMillSecStart=MillSecCount;
                    break;
                //delay 50 ms
                case 5:
                    BigCurMillSecEnd=MillSecCount;
                    if(BigCurMillSecEnd<BigCurMillSecStart)
                    {
                        BigCurMillSecEnd|=0x00010000;
                    }
                    //time delay 10 ms
                    if(BigCurMillSecEnd-BigCurMillSecStart>=100)
                    {
                        BigCurrentProcesStep=6;
                    }
                    break;
                case 6:
                    WriteLogDat(3,&DBCLogDat); //大电流处理--上开下关时数据
                    if(DBCLogDat.Cur>Threshold_BC)
                    {
                        BigCurrentJude=0;
                        BigCurrentJude|=1;    
                    }
                    BigCurrentProcesStep=7;
                    break;
                case 7:
                    CloseLbus();
                    OpenTbus();
                    BigCurrentProcesStep=8;
                    BigCurMillSecStart=MillSecCount;
                    break;
                case 8:
                    BigCurMillSecEnd=MillSecCount;
                    if(BigCurMillSecEnd<BigCurMillSecStart)
                    {
                        BigCurMillSecEnd|=0x00010000;
                    }
                    //time delay 10 ms
                    if(BigCurMillSecEnd-BigCurMillSecStart>=100)
                    {
                        BigCurrentProcesStep=9;
                    }
                    break;
                case 9:
                    WriteLogDat(4,&DBCLogDat); //大电流处理--上关下开时数据
                    if(DBCLogDat.Cur>Threshold_BC)
                    {
                        BigCurrentJude|=2;    
                    }
                    BigCurrentProcesStep=10;
                    break;
                //final deal with big current
                case 10:
                    if(0x1==(BigCurrentJude &0x1))
                    {
                        CloseLbus();
                    }
                    if(0x2==(BigCurrentJude &0x2))
                    {
                        CloseTbus();
                    }
                    Status_CUR=1;
                    BigCurrentProcesStep=11;
                    BigCurMillSecStart=MillSecCount;
                    break;                                      
                case 11:
                    BigCurMillSecEnd=MillSecCount;
                    if(BigCurMillSecEnd<BigCurMillSecStart)
                    {
                        BigCurMillSecEnd|=0x00010000;
                    }
                    //time delay 10 ms
                    if(BigCurMillSecEnd-BigCurMillSecStart>=100)
                    {
                        BigCurrentProcesStep=12;
                    }
                    break;
                case 12:
                    WriteLogDat(5,&DBCLogDat); //大电流处理--最终处理
                    BigCurrentProcesStep=13;
                    BigCurSecStart=SysTime;
                    break;
                case 13:
                    BigCurSecEnd=SysTime;
                    if(BigCurSecEnd<BigCurSecStart)
                    {
                        BigCurSecEnd|=0x00010000;    
                    }
                    //delay 5 minte
                    if(BigCurSecEnd-BigCurSecStart>300)
                    {
                        if(0x01==(BigCurrentBeforJudge & 0x01))
                        {
                            OpenLbus();
                        }
                        if(0x02==(BigCurrentBeforJudge & 0x02))
                        {
                            OpenTbus();
                        } 
                        Status_CUR=0;
                        BigCurrentProcesStep=14;
                        BigCurMillSecStart=MillSecCount;
                    }
                    break;
                case 14:
                    BigCurMillSecEnd=MillSecCount;
                    if(BigCurMillSecEnd<BigCurMillSecStart)
                    {
                        BigCurMillSecEnd|=0x00010000;
                    }
                    //time delay 100 ms
                    if(BigCurMillSecEnd-BigCurMillSecStart>=100)
                    {
                        BigCurrentProcesStep=15;
                    }
                    break;
                //reset the bus state
                case 15:
                    WriteLogDat(6,&DBCLogDat);                    
                    BigCurrentProcesStep=0;
                    FlagBigCurrentProcess=0;
                default:                     
                    BigCurrentProcesStep=0;
                    FlagBigCurrentProcess=0;
                    break;
            }
        }
        
        //low voltage process
        if(1==FlagLowVoltageProcess)
        {
            switch(LowVoltageProcessStep)    
            {
                case 0:
                    WriteLogDat(6,&DBCLogDat);  //记录地低电压数据                     
                    LowVolBeforJudge=0;
                    if(1==Status_TBUS)
                    {
                        LowVolBeforJudge|=1;    
                    }
                    if(1==Status_LBUS)
                    {
                        LowVolBeforJudge|=2;    
                    } 
                    LowVoltageProcessStep=1;                   
                    break;
                case 1:
                    CloseTbus();
                    CloseLbus();
                    LowVoltageProcessStep=2;
                    LowVolMillSecStart=MillSecCount;
                    Status_VOL=1;
                    break;
                //delay 1 s
                case 2:                
                    LowVolMillSecEnd=MillSecCount;
                    if(LowVolMillSecEnd<LowVolMillSecStart)
                    {
                        LowVolMillSecEnd|=0x00010000;
                    }
                    //time delay 1 s
                    if(LowVolMillSecEnd-LowVolMillSecStart>=1000)
                    {
                        LowVoltageProcessStep=3;
                    }                                             
                    break;
                case 3:
                    WriteLogDat(7,&DBCLogDat);  //低电压处理--上下全关数据 
                    LowVoltageProcessStep=4;
                    LowVolSecStart=SysTime;
                case 4:                   
                    LowVolSecEnd=SysTime;
                    if(LowVolSecEnd<LowVolSecStart)
                    {
                        LowVolSecEnd|=0x00010000;    
                    }
                    //delay 5 minte
                    if(LowVolSecEnd-LowVolSecStart>300)
                    {
                        if(0x01==(LowVolBeforJudge & 0x01))
                        {
                            OpenTbus();
                        }
                        if(0x02==(LowVolBeforJudge & 0x02))
                        {
                            OpenLbus();
                        }                        
                        LowVoltageProcessStep=5;
                        
                        Status_VOL=0;
                        LowVolMillSecStart=MillSecCount;
                    }
                    break;
                case 5:
                    LowVolMillSecEnd=MillSecCount;
                    if(LowVolMillSecEnd<LowVolMillSecStart)
                    {
                        LowVolMillSecEnd|=0x00010000;
                    }
                    //time delay 100 ms
                    if(LowVolMillSecEnd-LowVolMillSecStart>=100)
                    {
                        LowVoltageProcessStep=6;
                    }                                             
                    break;
                case 6:
                    WriteLogDat(8,&DBCLogDat);  //低电压处理--尝试恢复数据
                    LowVoltageProcessStep=0;
                    FlagLowVoltageProcess=0;
                    break;
                default:
                    break;    
            }
        }
                        
        //end volatage current check mode 
        //******************************************
        
      
        
        //******************************************
        //DBC work as RT
        switch (RT_State)
        {
            
            case STAT_DEFAULT:
                if(OK==DeQueue(&TbusQueue,&tempBusDat))
                {
                    RT_State=STAT_CHECKTBUSCMD; 
                }                               
                break;
            case STAT_CHECKTBUSCMD:
                if(1==tempBusDat.Type && ( DBC_Adr==(tempBusDat.Dat&0xFF) || BRA_Adr==(tempBusDat.Dat&0xFF)||UDT_Adr==(tempBusDat.Dat&0xFF)||VST_Adr==(tempBusDat.Dat&0xFF)))
                {
                    tempCMD = tempBusDat.Dat;  
                    tempCMD_Send = 0;                  
                    DatRecvLen = 0;                                        
                } 
                else
                {
                    if(DatRecvLen<MAXBUFLEN)
                    {                           
                        tempRecvBuf[DatRecvLen]=tempBusDat.Dat;
                        DatRecvLen++;   
                    }
                    RT_State=STAT_DEFAULT;                                                    
                }               
                switch(tempCMD)
                {
                    //0 data
                    case  SDI_SET_STANDBY:
                    case  SDI_SET_OPERATION:
                    case  SDI_STATUS_READ:
                    case  SDI_TOOLID_READ:
                    case  SDI_FWREV_READ:
                    case  SDI_SYNCTIME_READ:                     
                    case  UDI_VOLTAGE_READ:
                    case  UDI_CURRENT_READ:
                    case  UDI_TEMPERATURE_READ:
                    case  UDI_BC_READ:
                    case  UDI_BC_RESET:
                    case  UDI_BATCAP_READ:
                    case  UDI_TEST_READ_ADR:
                    case  SDI_DISKNO_READ:
                    case  UDI_PRESS_READ:
                    case  UDI_PRESSAD_READ:
                        if(DatRecvLen>=0)
                        {
                            RT_State=STAT_CMDDAT_PROSESS;    
                        }                                
                        else
                        {
                            RT_State=STAT_DEFAULT;    
                        }
                        break;
                    //1 data
                    case  SDI_ECHO:
                    case  UDI_FREQUENCY_WRITE_BRO:
                    case  SDI_DMSTAT_READ:
                    case  UDI_TEST_READ_ALDATA:
                    case  UDI_TEST_WRITE_ADR:
                    //case  UDI_TEST_CHANGE_DIM:
                    case  SDI_TABLE_READ:
                        if(DatRecvLen>=1)
                        {
                            RT_State=STAT_CMDDAT_PROSESS;    
                        }                                
                        else
                        {
                            RT_State=STAT_DEFAULT;    
                        }
                        break;
                    //2 data    
                    case  SDI_RESET:
                    case  SDI_SET_SLEEP:
                    case  UDI_BOOTLOADER:
                    case  SDI_LB_READ:
                        if(DatRecvLen>=2)
                        {
                            RT_State=STAT_CMDDAT_PROSESS;    
                        }                                
                        else
                        {
                            RT_State=STAT_DEFAULT;    
                        }
                        break;
                    //3 data
                    case  SDI_SYNCTIME_WRITE:
                    case  SDI_SYNCTIME_WRITE_BRO:
                    case  UDI_TBUS_ON_OFF:
                    case  UDI_LBUS_ON_OFF:
                    case  UDI_BATCAP_WRITE:
                    case  SDI_DMS_ERASE:
                    case  SDI_DBCMEM_FORMAT: 
                    case  UDI_PVD_POWER: 
                    case  UDI_PASSWORD_WRITE:
                        if(DatRecvLen>=3)
                        {
                            RT_State=STAT_CMDDAT_PROSESS;    
                        }                                
                        else
                        {
                            RT_State=STAT_DEFAULT;    
                        }
                        break;
                    //4 data
                    case  SDI_NLB_READ:
                        if(DatRecvLen>=4)
                        {
                            RT_State=STAT_CMDDAT_PROSESS;    
                        }                                
                        else
                        {
                            RT_State=STAT_DEFAULT;    
                        }
                        break;
                    //5 data
                    case  UDI_DMSTAT_WRITE:
                        if(DatRecvLen>=5)
                        {
                            RT_State=STAT_CMDDAT_PROSESS;    
                        }                                
                        else
                        {
                            RT_State=STAT_DEFAULT;    
                        }
                        break;
                    //7 data
                    case  UDI_TOOLID_WRITE:
                        if(DatRecvLen>=7)
                        {
                            RT_State=STAT_CMDDAT_PROSESS;    
                        }                                
                        else
                        {
                            RT_State=STAT_DEFAULT;    
                        }
                        break;
                    // Config 
                    case  SDI_TABLE_WRITE:
                        if(DatRecvLen>=1)
                        {
                            switch(tempRecvBuf[0])
                            {
                                case 1:   //TID=1,length=8,DBC维保参数
                                    if(DatRecvLen>=8)
                                    {
                                        RT_State=STAT_CMDDAT_PROSESS;    
                                    }                                
                                    else
                                    {
                                        RT_State=STAT_DEFAULT;    
                                    }
                                    break;
                                case 2:   //TID=2，length=20,环空压力标定系数
                                    if(DatRecvLen>=20)
                                    {
                                        RT_State=STAT_CMDDAT_PROSESS;    
                                    }                                
                                    else
                                    {
                                        RT_State=STAT_DEFAULT;    
                                    }
                                    break;
                                case 3:   //TID=3，length=27,内存参数
                                    if(DatRecvLen>=27)
                                    {
                                        RT_State=STAT_CMDDAT_PROSESS;    
                                    }                                
                                    else
                                    {
                                        RT_State=STAT_DEFAULT;    
                                    }
                                    break;
                                case 6:   //TID=6，length=15,作业参数
                                    if(DatRecvLen>=15)
                                    {
                                        RT_State=STAT_CMDDAT_PROSESS;    
                                    }                                
                                    else
                                    {
                                        RT_State=STAT_DEFAULT;    
                                    }
                                    break;                                    
                                default:
                                    RT_State=STAT_DEFAULT; 
                                    break;
                            }
                        } 
                        else                        
                        {
                            RT_State=STAT_DEFAULT;      
                        }
                        break;
                    //n data 
                    case SDI_COMM_TEST:
                        if(DatRecvLen>=1 && DatRecvLen>=tempRecvBuf[0]+1)
                        {
                            RT_State=STAT_CMDDAT_PROSESS;      
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;    
                        }
                        break;
                    default:
                        //Lbus 转发命令
                        if( ((UDT_Adr==(tempCMD&0xFF))||(VST_Adr==(tempCMD&0xFF)))
                               &&(DatRecvLen>=1)
                               &&(DatRecvLen>=tempRecvBuf[0]+1 ))
                        {
                            pWord=tempSendBufLbus;
                            *pWord=tempCMD;    
                            pWord++;
                            for(i=0;i<DatRecvLen;i++)
                            {
                                *pWord=tempRecvBuf[i];
                                pWord++;
                            }
                            
                            SCI_ClearBuf();
                            InitQueueByte(&LbusQueue);
                            LbusRecvLen=0;
                            UDT_DelayMSecStart=MillSecCount; 
                            SCI_Send(tempSendBufLbus,tempRecvBuf[0]*2+4); 
                            RT_State=STAT_GET_UDT_RESPONSE; 
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;  
                        }
                        
                        break;
                }                
                break;                
            case STAT_GET_UDT_RESPONSE:
                UDT_DelayMSecEnd=MillSecCount;
                if(UDT_DelayMSecEnd<UDT_DelayMSecStart)
                {
                    UDT_DelayMSecEnd|=0x00010000;
                }
                 //time up 50 ms
                if(UDT_DelayMSecEnd-UDT_DelayMSecStart>=50)
                {
                    RT_State=STAT_DEFAULT;
                }
                else
                {
                    if(OK==DeQueueByte(&LbusQueue,&tempByte))
                    {
                        
                        tempRecvBufLbus[0]=tempByte;
                        LbusRecvLen++;                    
                        RT_State=STAT_GET_UDT_RESPONSE_1;
                        UDT_DelayMSecStart=MillSecCount; 
                    }
                    
                }                                                                    
                break;
            case STAT_GET_UDT_RESPONSE_1:
                UDT_DelayMSecEnd=MillSecCount;
                if(UDT_DelayMSecEnd<UDT_DelayMSecStart)
                {
                    UDT_DelayMSecEnd|=0x00010000;
                }
                 //time up 50 ms
                if(UDT_DelayMSecEnd-UDT_DelayMSecStart>=50)
                {
                    RT_State=STAT_DEFAULT;
                }
                else
                {
                    if(OK==DeQueueByte(&LbusQueue,&tempByte))
                    {
                        
                        tempRecvBufLbus[1]=tempByte;
                        LbusRecvLen++;
                        pWord=tempRecvBufLbus;                                                           
                        SendD(*pWord);
                        if(2==LbusRecvLen)
                        {
                            LbusObjRecvLen=*pWord; 
                            RT_State=STAT_GET_UDT_RESPONSE;                               
                        }
                        else if(2<LbusRecvLen)
                        {
                            if(LbusRecvLen>=LbusObjRecvLen*2+2)
                            {
                                RT_State=STAT_DEFAULT;    
                            }
                            else
                            {
                               RT_State=STAT_GET_UDT_RESPONSE; 
                            }
                        }
                        UDT_DelayMSecStart=MillSecCount;
                         
                    }
                    
                }
                break;                
            case  STAT_CMDDAT_PROSESS:
                switch(tempCMD)
                { 
                    // n data
                    case SDI_COMM_TEST:                          
                        tempSendBuf[0]=SDI_COMM_TEST;
                        for(i=0;i<DatRecvLen;i++)
                        {
                            tempSendBuf[i+1]=tempRecvBuf[i];        
                        }
                        DatSendLen=DatRecvLen+1;                                            			
                        RT_State=STAT_SENDRESP;                            
                        break;    
                    //0 DAT
                    case  SDI_SET_STANDBY:
                        EEP_MODE=2;     //save in eeprom                          
                        StatusAll= eepromDat.CRP1.StInitStatus; 
                        Status_MODE=2;  //change current status mode  
                        /*
                        if(1==Status_TBUS)
                        {
                            OpenTbus();
                        }
                        else
                        {
                            CloseTbus();
                        }
                        if(1==Status_LBUS)
                        {
                            OpenLbus();
                        }
                        else
                        {
                            CloseLbus();
                        }
                        if(1==Status_DIAM)
                        {
                            OpenDIAM();    
                        }
                        else
                        {
                            CloseDIAM();
                        }                        
                        if(1==Status_VIBR)
                        {
                            OpenVIBR();        
                        }
                        else
                        {
                            CloseVIBR();
                        }
                        if(1==Status_PRE)
                        {
                            OpenPRE();    
                        }
                        else
                        {
                            ClosePRE();
                        }  
                        */ 
                        CloseTbus();
                        CloseLbus();
                        CloseDIAM(); 
                        ClosePRE();  
                        CloseVIBR();   
                                                                
                        tempSendBuf[0]=StatusAll;
                        DatSendLen=1;                        
                        RT_State=STAT_SENDRESP;
                        break;
                    case  SDI_SET_OPERATION:
                        EEP_MODE=3;     //save in eeprom                               
                        Status_MODE=3;  //change current status mode
                        
                        //启动实时模式，时间清零，进入TimeDelay状态
                        SysTime=0; 
                        GlobalOpState=OP_STAT_DELAY;
                        NextGlobalOpState=OP_STAT_DELAY;	                    
                                              
                        tempSendBuf[0]=StatusAll;
                        DatSendLen=1;                        
                        RT_State=STAT_SENDRESP;                                            
                        break;
                    case  SDI_STATUS_READ:
                        tempSendBuf[0]=StatusAll;
                        DatSendLen=1;                        
                        RT_State=STAT_SENDRESP;
                        break;
                    case  SDI_TOOLID_READ:
                        for(i=0;i<5;i++)
                  			{
                  				tempSendBuf[i]=(ToolID[2*i]<<8)+ToolID[2*i+1];             			
                  			}
                  			DatSendLen=5;                        
                        RT_State=STAT_SENDRESP;              			
                        break;
                    case  SDI_FWREV_READ:
                        for(i=0;i<5;i++)
                  			{
                  				tempSendBuf[i]=(FW_Version[2*i]<<8)+FW_Version[2*i+1];             			
                  			}
                  			DatSendLen=5;                        
                        RT_State=STAT_SENDRESP; 
                        break;
                    case  SDI_SYNCTIME_READ:
                        ReadRtc();
                    		CacuSecondTime(&tempDWord);
                    		pWord=&tempDWord;
                    		for(i=0;i<2;i++)
                    		{
                    		    tempSendBuf[i]=*pWord;
                    		    pWord++;    
                    		}
                    		tempSendBuf[2]=CalcuCHKS(tempSendBuf,2);
                        DatSendLen=3;                        
                        RT_State=STAT_SENDRESP;                        
                        break;                                        
                    case  UDI_VOLTAGE_READ:
                        tFloat=ReadVoltage();                        
                        pWord=&tFloat;  			        			            			      
                        for(i=0;i<2;i++)
                        {
                            tempSendBuf[i]=*pWord;
                            pWord++;
                        }
                        tempSendBuf[2]=CalcuCHKS(tempSendBuf,2);
                        DatSendLen=3;             			
                        RT_State=STAT_SENDRESP;
                        break;
                    case  UDI_CURRENT_READ:
                        tFloat=ReadCurrent();                       
                        pWord=&tFloat;  			        			            			      
                        for(i=0;i<2;i++)
                        {
                            tempSendBuf[i]=*pWord;
                            pWord++;
                        }
                        tempSendBuf[2]=CalcuCHKS(tempSendBuf,2);
                        DatSendLen=3;             			
                        RT_State=STAT_SENDRESP;
                        break;
                    case  UDI_TEMPERATURE_READ:
                        tFloat=ReadTemp();
                        pWord=&tFloat;  			        			            			      
                        for(i=0;i<2;i++)
                        {
                            tempSendBuf[i]=*pWord;
                            pWord++;
                        }
                        tempSendBuf[2]=CalcuCHKS(tempSendBuf,2);
                        DatSendLen=3;             			
                        RT_State=STAT_SENDRESP;
                        break;                    
                    case  UDI_BC_READ: 
                        tempSendBuf[0]=BootCount;
                        DatSendLen=1;             			
                        RT_State=STAT_SENDRESP;
                        break;
                    case  UDI_BC_RESET: 
                        BootCount=0;                    
                        tempSendBuf[0]=BootCount;
                        DatSendLen=1;             			
                        RT_State=STAT_SENDRESP;
                        break;
                    case  UDI_BATCAP_READ:                                              
                        pWord=&BatCap;  			        			            			      
                        for(i=0;i<2;i++)
                        {
                            tempSendBuf[i]=*pWord;
                            pWord++;
                        }
                        tempSendBuf[2]=CalcuCHKS(tempSendBuf,2);
                        DatSendLen=3;             				
                        RT_State=STAT_SENDRESP;
                        break;
                    case UDI_TEST_READ_ADR:
                        
                        tempSendBuf[0]=SecondAdr;
                        DatSendLen=1;             				
                        RT_State=STAT_SENDRESP;
                        break;
                    case  SDI_DISKNO_READ:
                        tempSendBuf[0]=DiskCount;
                        DatSendLen=1;             			
                        RT_State=STAT_SENDRESP;
                        break;
                    case  UDI_PRESS_READ:
                        tFloat = ReadInnerPre();                        
                        tempSendBuf[0]=(word)tFloat;                         
                        tFloat = ReadannPre();
                        tempSendBuf[1]=(word)tFloat; 
                        tempSendBuf[2]=CalcuCHKS(tempSendBuf,2);
                        DatSendLen=3;             			
                        RT_State=STAT_SENDRESP;
                        break;
                    case  UDI_PRESSAD_READ:
                        tempSendBuf[0]=SumFilter[3]>>9;
                        tempSendBuf[1]=SumFilter[4]>>9;
                        tempSendBuf[2]=CalcuCHKS(tempSendBuf,2);
                        DatSendLen=3;             			
                        RT_State=STAT_SENDRESP;                        
                        break;
                    //1 DAT        
                    case SDI_ECHO: 
                        tempSendBuf[0]=tempRecvBuf[0];
                        //tempSendBuf[0]=GlobalOpState;
                        DatSendLen=1;             			
                        RT_State=STAT_SENDRESP;
                        break;
                    case UDI_FREQUENCY_WRITE_BRO:                        
                        SetFreq(tempRecvBuf[0]);                                              			
                        RT_State=STAT_DEFAULT;                          
                        break;            
                    case SDI_DMSTAT_READ:                         
                        tWord= tempRecvBuf[0]>MaxDiskCount ? MaxDiskCount:tempRecvBuf[0];
                        pWord=&DmStat[tWord];                                                                           
                        for(i=0;i<4;i++)
                        {
                            tempSendBuf[i]=*pWord;
                            pWord++;
                        }
                        pWord=&DmBlankAdr[tWord];
                        for(i=0;i<2;i++)
                        {
                            tempSendBuf[i+4]=*pWord;
                            pWord++;    
                        }
                        tempSendBuf[6]=CalcuCHKS(tempSendBuf,6);
                        DatSendLen=7;             			
                        RT_State=STAT_SENDRESP;                          
                        break;
                    case UDI_TEST_READ_ALDATA:
                        if(SecondAdr==tempRecvBuf[0])
                        {
                            ReadRtc();
                        		CacuSecondTime(&tempDWord);    //0 1
                        		pWord=&tempDWord;
                        		for(i=0;i<2;i++)
                        		{
                        		    tempSendBuf[i]=*pWord;
                        		    pWord++;    
                        		}
                        		tempSendBuf[2]= SecondAdr;  //2
                        		pWord++;
                    		
                        		tempSendBuf[3]= BootCount;  //3
                        		pWord++;
                    		
                        		tFloat=ReadVoltage();   //4 5                    
                            pWord=&tFloat;  			        			            			      
                            for(i=0;i<2;i++)
                            {
                                tempSendBuf[i+4]=*pWord;
                                pWord++;
                            }
                    		
                        		tFloat=ReadCurrent();   //6 7                    
                            pWord=&tFloat;  			        			            			      
                            for(i=0;i<2;i++)
                            {
                                tempSendBuf[i+6]=*pWord;
                                pWord++;
                            }
                            
                            tFloat=ReadTemp();  //8 9
                            pWord=&tFloat;  			        			            			      
                            for(i=0;i<2;i++)
                            {
                                tempSendBuf[i+8]=*pWord;
                                pWord++;
                            }
                        		
                        		pWord=&BatCap;  	//10 11		        			            			      
                            for(i=0;i<2;i++)
                            {
                                tempSendBuf[i+10]=*pWord;
                                pWord++;
                            }                 		
                        		tempSendBuf[24]=CalcuCHKS(tempSendBuf,24);
                            DatSendLen=25;                        
                            RT_State=STAT_SENDRESP;     
                        }
                        break;
                    case UDI_TEST_WRITE_ADR:
                        SecondAdr=tempRecvBuf[0];
                        tempSendBuf[0]=SecondAdr;
                        DatSendLen=1;                        
                        RT_State=STAT_SENDRESP; 
                        break;
                    /*     
                    case UDI_TEST_CHANGE_DIM:
                        DimStatus[0]=tempRecvBuf[0];
                        tempSendBuf[0]=DimStatus[0];                        
                        DatSendLen=1;                        
                        RT_State=STAT_SENDRESP; 
                        break;
                    */
                    case  SDI_TABLE_READ:
                        switch(tempRecvBuf[0])
                        {
                            case 1:
                                pWord=&eepromDat.CRP1;                                                                  
                                for(i=0;i<8;i++)
                                {
                                    tempSendBuf[i]=*pWord;
                                    pWord++;
                                }                                
                                DatSendLen=8;                    
                                break;
                            case 2:
                                pWord=&eepromDat.CRP2;                                                                  
                                for(i=0;i<20;i++)
                                {
                                    tempSendBuf[i]=*pWord;
                                    pWord++;
                                }                                
                                DatSendLen=20;                    
                                break;
                            case 3:
                                pWord=&eepromDat.CRP3;                                                                  
                                for(i=0;i<27;i++)
                                {
                                    tempSendBuf[i]=*pWord;
                                    pWord++;
                                }                                
                                DatSendLen=27;                    
                                break;
                            case 6:
                                pWord=&SysPar;                                                                  
                                for(i=0;i<15;i++)
                                {
                                    tempSendBuf[i]=*pWord;
                                    pWord++;
                                }                                
                                DatSendLen=15;                    
                                break;                                
                            default:
                                break;
                        }
                            
                        RT_State=STAT_SENDRESP;                            
                        break;                            
                    //2 data
                    case  SDI_RESET:                        
                        if(tempRecvBuf[0]==0x0404 && tempRecvBuf[1]==0x0202)
                        {
                            SendD(StatusAll&0xFFFC);                 			       												
            				        ReSet();             			
                            RT_State=STAT_DEFAULT;        
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }
                        break;
                    case SDI_SET_SLEEP:
                        if(tempRecvBuf[0]==0x0101 && tempRecvBuf[1]==0x0202)
                        {
                            eepromDat.BootSelect=1;
                            
                            CloseTbus();
                            CloseLbus();
                            SendD(0x3821); 
                            Delay_xms(50);
                            ReSet();                                                           			
                            RT_State=STAT_DEFAULT;        
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }
                        break;
                    case UDI_BOOTLOADER: 
                        if(tempRecvBuf[0]==PassWord && tempRecvBuf[1]==0x5E02)
                        {
                            eepromDat.BootSelect=0xFF; 
                            
                            SendD(StatusAll); 
                            Delay_xms(50);
                            
                            ReSet();                          			
                            RT_State=STAT_DEFAULT;        
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }
                        break;
                    case SDI_LB_READ:                            
                        tempLB=tempRecvBuf[0];
                        tempLB=tempLB<<16;
                        tempLB+=tempRecvBuf[1];
                        LB_OffsetToAdr(tempLB,0,&tempAdr);
                        pByte=tempSendBuf;
                        ReadFDataU(&tempAdr,pByte,256);    	                
        				tempSendBuf[128]=CalcuCHKS(tempSendBuf,128);
        				DatSendLen=129;             			
                        RT_State=STAT_SENDRESP;
                        break;  
                    //3 data
                    case SDI_SYNCTIME_WRITE: 
                    case SDI_SYNCTIME_WRITE_BRO:                        
                        if(tempRecvBuf[2]==CalcuCHKS(tempRecvBuf,2))
                        {
                            tempDWord=tempRecvBuf[0];
            				tempDWord=(tempDWord<<16)+tempRecvBuf[1];
            				CacuClockTime(tempDWord);
            				WriteRtc(); 
            				
            				//UDT 广播时间
                            SCI_ClearBuf();
                            InitQueueByte(&LbusQueue);                    
            				tempSendBuf[0]=SDI_SYNCTIME_WRITE_BRO;
            				tempSendBuf[1]= 3;  //length
            				for(i=0;i<2;i++)
            				{                   				    
            				    tempSendBuf[i+2]= tempRecvBuf[i];
            				}
            				tempSendBuf[4]=CalcuCHKS(tempSendBuf+1,3);
            				SCI_Send(tempSendBuf,10); 
            				
            				if(tempCMD == SDI_SYNCTIME_WRITE ) 
            				{
            				    tempSendBuf[0]=tempRecvBuf[2];
                				DatSendLen=1; 
                				RT_State = STAT_SENDRESP; 
            				}    
            				else
                                RT_State = STAT_DEFAULT;                                                                  
                        }
                        else
                        {
                            RT_State = STAT_DEFAULT;
                        }                        
                        break;                    
                    case UDI_TBUS_ON_OFF: 
                        if(tempRecvBuf[0]==tempRecvBuf[1] && tempRecvBuf[2]==UDI_TBUS_ON_OFF)
                        {
                            if(0x0200==tempRecvBuf[0]) 
                            {                                                  
            				    OpenTbus();
            				    RT_State = STAT_DELAY_300MS; 
                            }
            				else if((~0x0200)==tempRecvBuf[0]) 
            				{
              					CloseTbus();
              					tempSendBuf[0]=StatusAll;
            				    DatSendLen=1;
            				    RT_State=STAT_DELAY_300MS;  				    
            				}
            				else
                            {
                                RT_State=STAT_DEFAULT;
                            }
                            //reset big current process step
                            BigCurrentProcesStep=0;
                            FlagBigCurrentProcess=0; 
                            
                            //reset low voltage process step
                            LowVoltageProcessStep=0;
                            FlagLowVoltageProcess=0;
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }                        
                        break;
                    case UDI_LBUS_ON_OFF:                        
                        if(tempRecvBuf[0]==tempRecvBuf[1] && tempRecvBuf[2]==UDI_LBUS_ON_OFF)
                        {
                            if(0x0400==tempRecvBuf[0]) 
                            {                                       
            				    OpenLbus();
            				    RT_State=STAT_DELAY_300MS; 
                            }
            				else if((~0x0400)==tempRecvBuf[0]) 
            				{
              					CloseLbus();
              					tempSendBuf[0]=StatusAll;
                				DatSendLen=1;             			
                                RT_State=STAT_DELAY_300MS;		    
            				}
            				else
                            {
                                RT_State=STAT_DEFAULT;
                            }
                            //reset big current process step
                            BigCurrentProcesStep=0;
                            FlagBigCurrentProcess=0;                                                                    
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }                                               
                        break;                         
                    case UDI_BATCAP_WRITE:                         
                        if(tempRecvBuf[2]==CalcuCHKS(tempRecvBuf,2))
                        {
                            pWord=&BatCap;
            				for(i=0;i<2;i++)
            				{
            				    *pWord=tempRecvBuf[i];
            				    pWord++;
            				}                			              				    
                            tempSendBuf[0]=tempRecvBuf[2];
            			    DatSendLen=1;             			
                            RT_State=STAT_SENDRESP;                                       
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }                                                
                        break;      
                    
                    case SDI_DMS_ERASE:                        
                        if(tempRecvBuf[0]==tempRecvBuf[1] && tempRecvBuf[2]==SDI_DMS_ERASE)
                        {
                            tWord=tempRecvBuf[0];
                            
                            SecE= DmBlankAdr[tWord]>>16;
                            for(i=DmStat[tWord].StarSec;i<=SecE;i++)                                  
                            {
                                SenToAdr(i,&tempAdr); 
                                SectorEraseU(&tempAdr);
                                SendD(i);    
                            } 
                            DmStat[tWord].BrokenLB=0;                                                        
                            DmBlankAdr[tWord]= LookForBlkAdr(&DmStat[tWord]);
                                                                             				    
                            tempSendBuf[0]=SDI_DMS_ERASE;
            			    DatSendLen=1;             			
                            RT_State=STAT_SENDRESP;                                       
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }                                                
                        break;
                    case SDI_DBCMEM_FORMAT:                        
                        if(tempRecvBuf[0]==tempRecvBuf[1] && tempRecvBuf[2]==SDI_DBCMEM_FORMAT)
                        {
                            tWord=tempRecvBuf[0];
                            
                            SecE= DmStat[tWord].EndSec;
                            for(i=DmStat[tWord].StarSec;i<=SecE;i++)                                  
                            {
                                SenToAdr(i,&tempAdr); 
                                SectorEraseU(&tempAdr);
                                SendD(i);    
                            } 
                            DmStat[tWord].BrokenLB=0;                                                        
                            DmBlankAdr[tWord]= LookForBlkAdr(&DmStat[tWord]);
                                                                             				    
                            tempSendBuf[0]=SDI_DBCMEM_FORMAT;
            			        	DatSendLen=1;             			
                            RT_State=STAT_SENDRESP;                                       
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }                        
                        break;          
                     case UDI_PVD_POWER:
                        if(tempRecvBuf[0]==tempRecvBuf[1] && tempRecvBuf[2]==UDI_PVD_POWER)
                        {  
                            tWord=tempRecvBuf[0];
                            
                            switch(tWord)
                            {
                                case 0x0100:
                                    OpenPRE();
                                    break;
                                case 0xFEFF:
                                    ClosePRE();
                                    break;
                                case 0x0080:
                                    OpenVIBR();
                                    break;
                                case 0xFF7F:
                                    CloseVIBR();
                                    break;
                                case 0x0040:
                                    OpenDIAM();
                                    break;
                                case 0xFFBF:
                                    CloseDIAM();
                                    break;
                                default:
                                    break;
                            }                                                                             				    
                            tempSendBuf[0]=StatusAll;
            				DatSendLen=1;             			
                            RT_State=STAT_DELAY_300MS;                                        
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }                        
                        break;
                     case  UDI_PASSWORD_WRITE:
                        if(tempRecvBuf[0]==tempRecvBuf[1] && tempRecvBuf[2]==CalcuCHKS(tempRecvBuf,2))
                        {
                            PassWord=tempRecvBuf[0];                                                                                 				    
                            tempSendBuf[0]=PassWord;
            				DatSendLen=1;             			
                            RT_State=STAT_SENDRESP;
                        } 
                        else
                        {
                            RT_State=STAT_DEFAULT;    
                        }
                        break;
                     //4 data
                     case SDI_NLB_READ:                        
                        if(tempRecvBuf[3]==CalcuCHKS(tempRecvBuf,3))
                        {
                            tempLB=tempRecvBuf[0];
                            tempLB=tempLB<<16;
                            tempLB+=tempRecvBuf[1];
                            LB_Start=tempLB;
                            Length=tempRecvBuf[2];                                 
                            RT_State=STAT_SEND_NLB;
                                                           
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }                          
                        break;                      
                    //5 data
                    case  UDI_DMSTAT_WRITE:                        
                        if(tempRecvBuf[4]==CalcuCHKS(tempRecvBuf,4))
                        {                                
            				tWord=(tempRecvBuf[0]>>8)>10 ? 10:(tempRecvBuf[0]>>8);
            				pWord=&DmStat[tWord];
            				for(i=0;i<4;i++)
            				{
            				    *pWord=tempRecvBuf[i];
            				    pWord++;
            				}
            				DmBlankAdr[tWord]= LookForBlkAdr(&DmStat[tWord]);                                 
                                                                                                             				    
                            tempSendBuf[0]=tempRecvBuf[4];
            				DatSendLen=1;             			
                            RT_State=STAT_SENDRESP;                                       
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }                         
                        break;
                    // 7 data
                    case UDI_TOOLID_WRITE:                        
                        if(tempRecvBuf[6]==CalcuCHKS(tempRecvBuf,6) && PassWord==tempRecvBuf[0])
                        {                                
            				pByte=tempRecvBuf+1;
            				for(i=0;i<10;i++)
            				{
            				    ToolID[i]=*pByte;
            				    pByte++;
            				}                				
                                                                                                             				    
                            tempSendBuf[0]=tempRecvBuf[6];
            				DatSendLen=1;             			
                            RT_State=STAT_SENDRESP;                                       
                        }
                        else
                        {
                            RT_State=STAT_DEFAULT;
                        }                                                                        
                        break;
                    // Config 
                    case SDI_TABLE_WRITE:
                        switch(tempRecvBuf[0])
                        {
                            case 1:   //TID=1,length=8,DBC维保参数
                                if(tempRecvBuf[7]==CalcuCHKS(tempRecvBuf,7))
                                {                                
                    				pWord=&eepromDat.CRP1;
                    				for(i=0;i<8;i++)
                    				{
                    					*pWord=tempRecvBuf[i];
                    					pWord++;
                    				}                                                                                				    
                                    tempSendBuf[0]=tempRecvBuf[7];
                    				DatSendLen=1;             			
                                    RT_State=STAT_SENDRESP;                                       
                                }
                                else
                                {
                                    RT_State=STAT_DEFAULT;
                                } 
                                break;
                            case 2:   //TID=2，length=20,环空压力标定系数
                                if(tempRecvBuf[19]==CalcuCHKS(tempRecvBuf,19))
                                {                                
                    				pWord=&eepromDat.CRP2;
                    				for(i=0;i<20;i++)
                    				{
                    					*pWord=tempRecvBuf[i];
                    					pWord++;
                    				}                                                                                				    
                                    tempSendBuf[0]=tempRecvBuf[19];
                    				DatSendLen=1;             			
                                    RT_State=STAT_SENDRESP;                                       
                                }
                                else
                                {
                                    RT_State=STAT_DEFAULT;
                                }
                                break;
                            case 3:   //TID=3，length=27,内存参数
                                if(tempRecvBuf[26]==CalcuCHKS(tempRecvBuf,26))
                                {                                
                    				pWord=&eepromDat.CRP3;
                    				for(i=0;i<27;i++)
                    				{
                    					*pWord=tempRecvBuf[i];
                    					pWord++;
                    				}                                                                                				    
                                    tempSendBuf[0]=tempRecvBuf[26];
                    				DatSendLen=1;             			
                                    RT_State=STAT_SENDRESP;                                       
                                }
                                else
                                {
                                    RT_State=STAT_DEFAULT;
                                }
                                break;
                            case 6:   //TID=6，length=15,作业参数
                                if(tempRecvBuf[14]==CalcuCHKS(tempRecvBuf,14))
                                {                                
                    				pWord=&SysPar;
                    				for(i=0;i<15;i++)
                    				{
                    					*pWord=tempRecvBuf[i];
                    					pWord++;
                    				}                                                                                				    
                                    tempSendBuf[0]=tempRecvBuf[14];
                    				DatSendLen=1;             			
                                    RT_State=STAT_SENDRESP;                                       
                                }
                                else
                                {
                                    RT_State=STAT_DEFAULT;
                                }
                                break;                                    
                            default:
                                RT_State=STAT_DEFAULT; 
                                break;
                        }
                        break;                                                          
                    default:
                        RT_State=STAT_DEFAULT;
                        break;
                }
                break;
            case STAT_DELAY_300MS:
                BusTimeDelayCount=0;
                RT_State=STAT_DELAY_300MS_2;                
                break;
            case STAT_DELAY_300MS_2:
                BusTimeDelayCount++;
                if(BusTimeDelayCount>10000)    //100ms
                {
                    tempSendBuf[0]=StatusAll;
                	DatSendLen=1;    
                    RT_State=STAT_SENDRESP;        
                }
                break;
            case STAT_SENDRESP:
                SendDlist(tempSendBuf,DatSendLen);
                RT_State=STAT_DEFAULT;
                DatRecvLen=0;
                tempCMD=0;
                break;            
            case STAT_SEND_NLB:
                if(Length>0)
                {
                    LB_OffsetToAdr(LB_Start,0,&tempAdr);
                    tempSendBuf[0]=(LB_Start>>16);
                    tempSendBuf[1]=(LB_Start&0xFFFF);
                    pByte=tempSendBuf+2;
                    
                    ReadFDataU(&tempAdr,pByte,256);    	                
                    tempSendBuf[130]=CalcuCHKS(tempSendBuf+2,128);
                    DatSendLen=131;
                    SendDlist(tempSendBuf,DatSendLen);
                    
                    LB_Start++;
                    Length--; 
                }
                else
                {
                    
                    RT_State=STAT_DEFAULT;                   
                }
                break;
            default:
                RT_State=STAT_DEFAULT;
                break;
        }               
        //end DBC work as RT 
        //****************************************** 
        
        
        _FEED_COP(); /* feeds the dog */    
    }
    /* please make sure that you never leave main */
}

static void VariableInit(void)
{
  int i;   	
  
	if(2!=eepromDat.BootSelect)     //UserApp 模式
	    eepromDat.BootSelect=2;
	
	 eepromDat.CRP1.ParTimeBro=1800;

    //判断配置参数是否正确
    if(eepromDat.CRP1.CHK==0xFFFF||eepromDat.CRP1.CHK!=CalcuCHKS(&eepromDat.CRP1,7)) 
    {
        eepromDat.CRP1.TID=1;
        eepromDat.CRP1.ParTimeBro=1800;
        eepromDat.CRP1.ParTimeDBC=600;
        eepromDat.CRP1.StInitStatus=0x3802;
        eepromDat.CRP1.CHK=CalcuCHKS(&eepromDat.CRP1,7);
    }
    if(eepromDat.CRP2.CHK==0xFFFF||eepromDat.CRP2.CHK!=CalcuCHKS(&eepromDat.CRP2,19)) 
    {
        eepromDat.CRP2.TID=2;
        eepromDat.CRP2.P1_k=7.0;
        eepromDat.CRP2.P1_b=0.0;
        eepromDat.CRP2.P2_k=7.0;
        eepromDat.CRP2.P2_b=0.0;        
        eepromDat.CRP2.CHK=CalcuCHKS(&eepromDat.CRP2,19);
    }
    if(eepromDat.CRP3.CHK==0xFFFF||eepromDat.CRP3.CHK!=CalcuCHKS(&eepromDat.CRP3,26)) 
    {
        eepromDat.CRP3.TID=3;
        eepromDat.CRP3.DisCount=3;
        for(i=0;i<eepromDat.CRP3.DisCount;i++)
        {               
            DmStat[i].DiskNum=i;
            DmStat[i].BrokenLB=0;
        }          
        DmStat[0].RecordLenth=32;   //数据区1
        DmStat[0].StarSec = 0;
        DmStat[0].EndSec = 127; 
        
        DmStat[1].RecordLenth=32;   //数据区2
        DmStat[1].StarSec = 128;
        DmStat[1].EndSec = 383;  
        
        DmStat[2].RecordLenth=32;   //数据区3
        DmStat[2].StarSec = 384;
        DmStat[2].EndSec = 511; 
               
        eepromDat.CRP3.CHK=CalcuCHKS(&eepromDat.CRP3,26);
    }
    
    if(eepromDat.Config.CHK==0xFFFF||eepromDat.Config.CHK!=CalcuCHKS(&eepromDat.Config,14))        
    {           
        //配置默认参数        
        SysPar.TID=6;
        SysPar.ParTimeDelay=0;        
        SysPar.ParTimeToMWD=9;
        SysPar.ParTimeSavePre=0; 
        FlagRTD_All=0;
        SysPar.PowerSavingET = 15;  //default setting,  Power Saving Enter Time
        
        EEP_MODE=2; //standby
        
        eepromDat.Config.CHK=CalcuCHKS(&eepromDat.Config,14);
    }
    
    if (3==EEP_MODE)    //Operation模式      
    {
        Status_MODE=3;
    
        OpenLbus();        
        OpenTbus();        
        if(1==FlagPRE)
        {
            OpenPRE();
        }
        else
        {
            ClosePRE();
        }
        if(1==FlagVST)
        {
            OpenVIBR();
        }
        else
        {
            CloseVIBR();
        }
        if(1==FlagDIAM)
        {
            OpenDIAM();
        }
        else
        {
            CloseDIAM();
        }
    } 
    else
    {
       StatusAll= eepromDat.CRP1.StInitStatus;   //Standby mode
        // StatusAll= 0x38A2;   //Standby mode,open VST   
        if(1==Status_TBUS)
        {
            OpenTbus();
        }
        else
        {
            CloseTbus();
        }
        if(1==Status_LBUS)
        {
            OpenLbus();
        }
        else
        {
            CloseLbus();
        }
        if(1==Status_DIAM)
        {
            OpenDIAM();    
        }
        else
        {
            CloseDIAM();
        }                        
        if(1==Status_VIBR)
        {
            OpenVIBR();        
        }
        else
        {
            CloseVIBR();
        }
        if(1==Status_PRE)
        {
            OpenPRE();    
        }
        else
        {
            ClosePRE();
        }
    }
    SetFreq(40);
    
    if(eepromDat.CRP3.DisCount>5)
    {           
        eepromDat.CRP3.DisCount=1;
        for(i=0;i<eepromDat.CRP3.DisCount;i++)
        {               
            DmStat[i].DiskNum=i;
            DmStat[i].BrokenLB=0;
        }
        
        DmStat[0].RecordLenth=32;   //数据区1
        DmStat[0].StarSec = 0;
        DmStat[0].EndSec = 127; 
    }
    
    for(i=0;i<eepromDat.CRP3.DisCount;i++)
    {
        DmBlankAdr[i]= LookForBlkAdr(&DmStat[i]); 
    }
    
    Delay_xms(100);
    
    WriteLogDat(0,&DBCLogDat);  //记录初始启动数据
    
    BatTimerCounter=0;  //电量计算计时器  
    TimeBroState=0;
        
}

static void Delay_xms(word xms)
{
    word tempDelay;
     while(xms>0)
     {
        tempDelay=1600;
        while(tempDelay>0)
        {
            tempDelay--; 
            _FEED_COP(); /* feeds the dog */  
        }
        xms--;
        
     } 
}

static void GetPreDat(int index,PRESAVEDAT *preDat)
{
    if(0==index)
    {
      preDat->DataID=131;
      GetRTC(&preDat->RTC);        
    }    
    preDat->SavPre[index][0]=(word)ReadInnerPre();
    preDat->SavPre[index][1]=(word)ReadannPre();   
      
}

static void WritePreDat(int DiskID,PRESAVEDAT *preDat)
{
    byte *pByte,*pByte2;
    int FlagSave=0,i,j,recLen;
    dword tempAdr;
    word tempWordBuf[64];
    for(i=0;i<3;i++)
	{ 	    
	    //recLen=DmStat[DiskID].RecordLenth; 
	    recLen=32;          	         	
	    tempAdr=DmBlankAdr[DiskID];            	                	    
	    WriteFDataU(&DmBlankAdr[DiskID],preDat,recLen); 
	    ReadFDataU(&tempAdr,tempWordBuf,recLen);
	    FlagSave=0;
	    pByte=preDat;
	    pByte2=tempWordBuf;
	    
	    for(j=0;j<recLen;j++)
	    {
	        if(*pByte!=*pByte2)
	        {
	            FlagSave=1; 
	            break;   
	        }                 	        
	    }
	    if(FlagSave==0)
	            break;
	}
}

static void WriteLogDat(byte dataID,DBCLOG *logdat)
{
    
    byte *pByte,*pByte2;
    int FlagSave=0,i,j,recLen;
    dword tempAdr;
    word tempWordBuf[16];
    
    GetRTC(&logdat->RTC);
    logdat->DatID=dataID;     
    logdat->BootC=BootCount&0xFF;
    logdat->Status=StatusAll;  
    logdat->Vol=ReadVoltage();        
    logdat->Cur= ReadCurrent();      
    logdat->BatC=BatCap;        
    logdat->Temp=ReadTemp();
    logdat->InnPre=ReadInnerPre();
    logdat->AnnPre=ReadannPre();
    
    for(i=0;i<3;i++)
	{ 	    
	    recLen=32;	              	         	
	    tempAdr=DmBlankAdr[0];            	                	    
	    WriteFDataU(&DmBlankAdr[0],logdat,recLen); 
	    ReadFDataU(&tempAdr,tempWordBuf,recLen);
	    FlagSave=0;
	    pByte=logdat;
	    pByte2=tempWordBuf;
	    
	    for(j=0;j<recLen;j++)
	    {
	        if(*pByte!=*pByte2)
	        {
	            FlagSave=1; 
	            break;   
	        }                 	        
	    }
	    if(FlagSave==0)
	            break;
	}
}

static void GetRTC(word *pBuf)
{
    dword TimeSecd=0;    
    word *pWord,i=0;
    ReadRtc();
	CacuSecondTime(&TimeSecd);
	pWord=&TimeSecd;
	for(i=0;i<2;i++)	
	    *pBuf++=*pWord++; 
}

static void CompressData(word *Object,float *Source,float Xmin,float Xmax,byte N)
{	
	float temp=*Source;
	if(temp<Xmin)
		temp=Xmin;
	else if(temp>Xmax)
		temp=Xmax;

	temp=(temp-Xmin)/(Xmax-Xmin);	
	*Object=(word)((((unsigned long)1<<N)-1)*temp);
}

void OpenTbus()
{    
    if(PORTK_PK3!=1)
    {
        PORTK_PK3 = 1;  
        WriteLogDat(35,&DBCLogDat);  //打开下总线，记录日志数据
    }
    Status_TBUS=1;
    
}
void CloseTbus()
{
    if(PORTK_PK3 != 0)
    {
        PORTK_PK3 = 0;
        WriteLogDat(36,&DBCLogDat);  //关闭下总线，记录日志数据
    }
    Status_TBUS=0;
}

void OpenLbus(void)
{
    if(PORTK_PK5 != 1)
    {
        PORTK_PK5 = 1;
        WriteLogDat(33,&DBCLogDat);  //打开上总线，记录日志数据
    }    
    Status_LBUS=1;
    
}
void CloseLbus(void)
{
    if(PORTK_PK5 != 0)
    {
        PORTK_PK5 = 0;     
        WriteLogDat(34,&DBCLogDat);  //关闭上总线，记录日志数据
    }
    Status_LBUS=0;
}
