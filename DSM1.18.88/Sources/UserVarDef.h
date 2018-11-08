#ifndef __USERVARDEF_H_
#define __USERVARDEF_H_

//TBus 命令
#define SDI_SET_SLEEP   0x0102
#define SDI_SET_STANDBY	0x0202
#define SDI_SET_OPERATION	0x0302
#define SDI_RESET	0x0402
#define SDI_STATUS_READ	0x0502
#define SDI_ECHO	0x0602
#define SDI_TOOLID_READ	0x0702
#define SDI_FWREV_READ	0x0802
#define SDI_COMM_TEST   0x0902
	
#define SDI_SYNCTIME_WRITE	0x1102
#define SDI_SYNCTIME_WRITE_BRO 0x1100
#define SDI_SYNCTIME_READ	0x1202
#define SDI_TABLE_WRITE	0x1302
#define SDI_TABLE_READ	0x1402

#define SDI_DISKNO_READ 0x2002	
#define SDI_DMSTAT_READ	0x2102  
#define UDI_DMSTAT_WRITE	0x2302
	

#define SDI_LB_READ	    0x2702
#define SDI_NLB_READ    0x2802
	
#define SDI_DMS_ERASE	0x2A02
#define SDI_DBCMEM_FORMAT	0x2B02
	
#define UDI_VOLTAGE_READ	0x5002
#define UDI_CURRENT_READ	0x5102
#define UDI_TBUS_ON_OFF	0x5202

#define UDI_PRESS_READ  0x5302
#define UDI_FREQUENCY_WRITE_BRO	0x5400
#define UDI_TEMPERATURE_READ	0x5502
#define UDI_LBUS_ON_OFF	0x5602


#define UDI_TOOLID_WRITE	0x5802
#define UDI_BC_READ	0x5902
#define UDI_BC_RESET	0x5A02
#define UDI_BATCAP_READ	0x5B02
#define UDI_BATCAP_WRITE	0x5C02

#define UDI_BOOTLOADER	0x5E02

#define UDI_PVD_POWER	0x6002
#define UDI_PASSWORD_WRITE	0x6102

#define UDI_TEST_READ_ALDATA 0x6402
#define UDI_TEST_WRITE_ADR  0x6502
#define UDI_TEST_READ_ADR   0x6602
#define UDI_TEST_COMM  0x6702
#define UDI_TEST_CHANGE_DIM 0x6802

#define UDI_PRESSAD_READ  0x6902


//PMT 相关命令
#define PMT_API_READ   0x5522
#define AGR_API_READ   0x5422


//CWPR 相关命令
#define CWPR_CMD_SEND   0x4031
#define CWPR_FETCH_RESPONSE   0x4131
#define CWPR_STAT_READ   0x0531

//ACPR 相关命令
#define ACPR_GNDAT_READ 0x4232
#define ACPR_CPDAT_READ 0x4532
#define DWPR_RTD_READ 0x433A

//CDL指令下传控制
#define READ_TGC_CDLDATD  0xEB81

//COTAS 相关命令
#define RST_TSTAT_READ	0xE181      //GetDownlinkStatus
#define RST_TVEL_READ  	0xE281      //读转速
#define RST_DSTAT_SEND  0xEC81      //SEND DIM STATUS

#define RST_SAK_READ	0xF681      //读取SFU简化参数
#define RST_SCDL_READ	0xF481 

#define RST_ABIN_READ   0xFA81

//DIM 相关命令
#define DIM_STAT_READ	0x0511
#define DIM_DBC_SEND	0xA011
#define DIM_NGR_SEND	0xA111
#define DIM_CWPR_SEND	0xA211
#define DIM_MCM_READ    0xA311
#define DIM_TSTAT_SEND	0xA411
#define DIM_TVEL_SEND	0xA511
#define DIM_SAK_SEND	0xA611
#define DIM_SCDL_SEND	0xA711    //SEND CDL DOWNLINK DATA
#define DIM_SFUN_SEND	0xA811
#define DIM_ABIN_SEND	0xA911
#define DIM_RTCFG_SEND  0xAA11
#define DIM_RSCDL_SEND  0xAC11    //实时状态下传送CDL数据

#define DIM_AGR_SEND  0xC711


#define DIM_PRE_SEND    0xAD11
#define DIM_VST_SEND    0xAE11
#define DIM_ACPR_SEND   0xAF11

#define DIM_ACOU_SEND   0xB411
#define DIM_NEUT_SEND   0xB511
#define DIM_DENS_SEND   0xB611

#define DIM_PMT_SEND    0xB011
#define DIM_UDT_SEND    0xB111

#define DIM_ACPRCP_SEND 0xB811
#define DIM_GRO_SEND    0xB911

#define DIM_NBGD_SEND   0xC311
#define DIM_NBMD_SEND   0xC411

#define DIM_DWPR_SEND   0xCA11


#define DIRTOOL_DIMDTA_BRO  0xD100    //广播角差

#define PumpOFF	        0
#define RunIn           1 
#define RealTimeMode	2
#define CDL_DiagUpload  3
#define CDL_Upload      4
#define CDL_Waiting     5
#define DIM_Sleep       6
#define CDL_WaitData    7
#define DIM_DataSend    8
#define DIM_Measure     9

//NEUT 相关指令
#define NEUT_POR_READ   0x4326
#define NEUT_DIA_READ   0x4926
#define NEUT_data_READ  0x8326    // 2016.8.30 修改

//MAST 声波相关指令
#define ACOU_DAT_READ   0x4141 
//DENS 相关命令  
#define LDI_DAT_READ   0x7025   //针对密度新仪器2016.8.30 修改
#define DENS_RESET_CMD  0x2525

/*
//GYRO 相关指令
#define GRO_DATA_READ   0x2412
#define GRO_BATC_READ   0x2512
#define GRO_START_DAQ   0x8312
#define GRO_STAT_READ   0x8412
#define GRO_POWER_ON    0x0312
#define GRO_POWER_OFF   0x0412
*/
//UDT
#define UDT_DATA_READ   0x5042
//VST 振动量级测量
#define VST_DATA_READ   0x5551

#define NBT_GDATA_READ  0x5535
#define NBT_MDATA_READ  0x5635

// TOOL STUTUS MONITOR
#define MONITOR_RST_DAT   0xEF81


//#define Threshold_BC    1500
//#define	Threshold_LV    20
#define Threshold_BC    1600
#define	Threshold_LV    20


#define STAT_DEFAULT       0
#define STAT_CHECKTBUSCMD  1
#define STAT_CMDDAT_PROSESS  2


#define STAT_SENDRESP      3
#define STAT_SEND_LBUS_CMD 4
#define STAT_SEND_NLB      5
#define STAT_DELAY_300MS   6 
#define STAT_DELAY_300MS_2 7 
#define STAT_GET_UDT_RESPONSE  8
#define STAT_GET_UDT_RESPONSE_1  9
#define STAT_GET_UDT_RESPONSE_2  10
#define STAT_GET_UDT_RESPONSE_3  11

#define OP_STAT_DEFAULT     0
#define OP_STAT_DELAY       1
#define OP_STAT_DIMCOM      2
#define OP_STAT_SENDCMD     3
#define OP_STAT_GETRESPONSE 4
#define OP_STAT_IDEL        5
#define OP_STAT_STARTSEND   6
#define OP_STAT_DIMCOMOK    7
#define OP_STAT_TRANSFER_REATIMEDATA 8
#define OP_STAT_TRANSFER_DBCDATA 9
#define OP_STAT_TRANSFER_NGRDATA 10
#define OP_STAT_TRANSFER_NGRDATA_1 11
#define OP_STAT_TRANSFER_NGRDATA_2 12
#define OP_STAT_TRANSFER_NGRDATA_3 13
#define OP_STAT_TRANSFER_CWPRDATA 14 
#define OP_STAT_TRANSFER_CWPRDATA_1 15
#define OP_STAT_TRANSFER_CWPRDATA_2 16
#define OP_STAT_TRANSFER_CWPRDATA_3 17
#define OP_STAT_TRANSFER_CWPRDATA_4 18

#define OP_STAT_TRANSFER_PREDATA 19
#define OP_STAT_TRANSFER_PREDATA_1 20
#define OP_STAT_TRANSFER_VSTDATA 21
#define OP_STAT_TRANSFER_VSTDATA_1 22
#define OP_STAT_TRANSFER_VSTDATA_2 23
#define OP_STAT_TRANSFER_VSTDATA_3 24

#define OP_STAT_TRANSFER_ACPR 25
#define OP_STAT_TRANSFER_ACPR_1 26
#define OP_STAT_TRANSFER_ACPR_2 27
#define OP_STAT_TRANSFER_ACPR_3 28
#define OP_STAT_TRANSFER_ACPR_4 29

#define OP_STAT_TRANSFER_PMT 30
#define OP_STAT_TRANSFER_PMT_1 31
#define OP_STAT_TRANSFER_PMT_2 32
#define OP_STAT_TRANSFER_PMT_3 33
#define OP_STAT_TRANSFER_DIAM 34
#define OP_STAT_TRANSFER_DIAM_1 35
#define OP_STAT_TRANSFER_DIAM_2 36
#define OP_STAT_TRANSFER_DIAM_3 37

#define OP_STAT_TRANSFER_TVELDATA 38
#define OP_STAT_TRANSFER_TVELDATA_1 39
#define OP_STAT_TRANSFER_TVELDATA_2 40
#define OP_STAT_TRANSFER_SAKDATA 41
#define OP_STAT_TRANSFER_SAKDATA_1 42
#define OP_STAT_TRANSFER_SAKDATA_2 43
#define OP_STAT_TRANSFER_SAKDATA_3 44
#define OP_STAT_TRANSFER_SAKDATA_4 45

#define OP_STAT_TRANSFER_SCDLDATA 46
#define OP_STAT_TRANSFER_SCDLDATA_1 47
#define OP_STAT_TRANSFER_SCDLDATA_2 48
#define OP_STAT_DIAG_JUDGE  49
#define OP_STAT_TRANSFER_SFUNDATA 50
#define OP_STAT_TRANSFER_SFUNDATA_1 51
#define OP_STAT_TRANSFER_SFUNDATA_2 52
#define OP_STAT_TRANSFER_ABINDATA   53
#define OP_STAT_TRANSFER_ABINDATA_1   54
#define OP_STAT_TRANSFER_ABINDATA_2   55
#define OP_STAT_SYN_DIM_TGC 56
#define OP_STAT_SYN_DIM_TGC_1 57
#define OP_STAT_SYN_DIM_TGC_2 58
#define OP_STAT_SYN_DIM_TGC_3 59
#define OP_STAT_TRANSFER_CDLDATA    60
#define OP_STAT_TRANSFER_CDLDATA_1   61
#define OP_STAT_TRANSFER_CDLDATA_2   62
#define OP_STAT_TRANSFER_CDLDATA_3   63
#define OP_STAT_TRANSFER_CDLDATA_4   64
#define OP_STAT_TIMEBRO 65

#define OP_STAT_QUIET 66
#define OP_STAT_QUIET_1 67

#define OP_STAT_QUIT_QUIET 68
#define OP_STAT_QUIT_QUIET_1 69
#define OP_STAT_QUIT_QUIET_2 70
#define OP_STAT_QUIT_QUIET_3 71

#define OP_STAT_SAVE_DATA   72

#define OP_STAT_ERROR_COMM  73
#define OP_STAT_ERROR_COMM_1  74
#define OP_STAT_ERROR_COMM_2  75

#define OP_STAT_ERROR_TBUS  76
#define OP_STAT_ERROR_TBUS_1  77
#define OP_STAT_ERROR_TBUS_2  78
#define OP_STAT_ERROR_TBUS_3  79

/*
//随钻陀螺
#define OP_STAT_TRANSFER_GRODATA    80
#define OP_STAT_TRANSFER_GRODATA_1  81
#define OP_STAT_TRANSFER_GRODATA_2  82
#define OP_STAT_TRANSFER_GRODATA_3  83
#define OP_STAT_TRANSFER_GRODATA_4  84
#define OP_STAT_TRANSFER_GRODATA_5  85
#define OP_STAT_TRANSFER_GRODATA_6  86
#define OP_STAT_TRANSFER_GRODATA_7  87
#define OP_STAT_TRANSFER_GRODATA_8  88
#define OP_STAT_TRANSFER_GRODATA_9  89
#define OP_STAT_TRANSFER_GRODATA_10  90
#define OP_STAT_TRANSFER_GRODATA_11  91
#define OP_STAT_TRANSFER_GRODATA_12  92
#define OP_STAT_TRANSFER_GRODATA_13  93
#define OP_STAT_TRANSFER_GRODATA_14  94

#define OP_STAT_TRANSFER_GRO_ERR_1  95
#define OP_STAT_TRANSFER_GRO_ERR_2  96
#define OP_STAT_TRANSFER_GRO_ERR_3  97
#define OP_STAT_TRANSFER_GRO_ERR_4  98
#define OP_STAT_TRANSFER_GRO_ERR_5  99
#define OP_STAT_TRANSFER_GRO_ERR_6  100
#define OP_STAT_TRANSFER_GRO_ERR_7  101
#define OP_STAT_TRANSFER_GRO_ERR_8  102
*/



//随钻中子
#define OP_STAT_TRANSFER_NEUT 103
#define OP_STAT_TRANSFER_NEUT_1 104
#define OP_STAT_TRANSFER_NEUT_2 105
#define OP_STAT_TRANSFER_NEUT_3 106
//随钻声波
#define OP_STAT_TRANSFER_ACOU 107
#define OP_STAT_TRANSFER_ACOU_1 108
#define OP_STAT_TRANSFER_ACOU_2 109
//随钻密度
#define OP_STAT_TRANSFER_DENS 110
#define OP_STAT_TRANSFER_DENS_1 111
#define OP_STAT_TRANSFER_DENS_2 112

#define OP_STAT_GET_DIM_DATA  113

#define OP_STAT_TRANSFER_DIAMDATA   114
#define OP_STAT_TRANSFER_DIAMDATA_1 115

#define OP_STAT_DIMDTA_BRO      142    //广播DIM状态
#define OP_STAT_DIMDTA_BRO_1    143

#define OP_STAT_TRANSFER_AGR   130
#define OP_STAT_TRANSFER_AGR_1 131
#define OP_STAT_TRANSFER_AGR_2 132
#define OP_STAT_TRANSFER_AGR_3 133

#define OP_STAT_TRANSFER_NBT     135
#define OP_STAT_TRANSFER_NBT_1   136
#define OP_STAT_TRANSFER_NBT_2   137
#define OP_STAT_TRANSFER_NBT_3   138
#define OP_STAT_TRANSFER_NBT_4   139

#define OP_STAT_READ_CDLDATA   150  //FROM TGC GET CDL DATA
#define OP_STAT_TRANSFER_CDLDATA_DIM  151 //SEND CDL DATA TO DIM

#define OP_STAT_SFU_MONITOR   152 // SFU MONITOR

/*
//pump off save energeny
#define OP_STAT_PUMP_OFF_SAVEBAT  155
#define OP_STAT_PUMP_OFF_SAVEBAT_1  156
#define OP_STAT_PUMP_OFF_SAVEBAT_2  157
#define OP_STAT_PUMP_OFF_SAVEBAT_3  158
#define OP_STAT_PUMP_OFF_SAVEBAT_4  159
#define OP_STAT_PUMP_OFF_SAVEBAT_5  160
#define OP_STAT_PUMP_OFF_SAVEBAT_6  161
*/
//DWPR
#define OP_STAT_TRANSFER_DWPR       162
#define OP_STAT_TRANSFER_DWPR_1     163
#define OP_STAT_TRANSFER_DWPR_2     164

/*
#define GWD_DEFAULT     0
#define GWD_CLOSE_TBUS  1
#define GWD_CLOSE_LBUS  2
#define GWD_OPEN_TBUS   3
#define GWD_RESET       4
#define GWD_DELAY_1     5
#define GWD_DELAY_2     6
#define GWD_DELAY_3     7
*/
#define OP_STAT_LBUS_DEFAULT    0
#define OP_STAT_LBUS_UDT        1
#define OP_STAT_LBUS_UDT_1      2
#define OP_STAT_LBUS_UDT_2      3
#define OP_STAT_LBUS_RECV_WAIT  4

#define OP_STAT_LBUS_VST        7
#define OP_STAT_LBUS_VST_1      8
#define OP_STAT_LBUS_VST_2      9
#define OP_STAT_LBUS_ERROR      10

#define OP_STAT_LBUS_IDLE   40


typedef struct
{
    //dword RTC;
    byte FlagDSTATUS;
    word DSTATUS;
    byte FlagMWDD;
    word MWD_DAT[21];
    
    word Status;
    byte DBC_DAT[4];        //Cur,Vol,Temp,BatC    
    
    //USM
    byte FlagUSM;
    word USM_DAT;
    //VSM
    byte FlagVSM;
    word VST_DAT[7]; 
    
    byte FlagDMBRO;   //角差广播
    
    
    byte FlagTGCC;
    byte TSTATUS;
    byte TVEL;
    byte RMAX;
    byte RMIN;
    
    byte FlagSNK;
    byte SNK_DAT[14];        

    byte Mode;                                                                                        
    word CONFIG;   
    
    byte FlagACPRR;
    word ACPR_DAT[11];
    
    byte FlagPMTT;
    byte API[4];
    
    byte FlagAGRT;
    byte AGR[7];      //
    
    byte FlagPREE;
    word InnPRE;
    word AnnPRE;
    
    byte FlagINPT;
    word Porosity;
    byte HoleSize;
    
    byte FlagMAST;
    word MAST_DAT[3];
    
    byte FlagLDIT;
    word LDI_DAT[2];
    
    //byte FlagGROO;
    //word GRO_DAT[5];
    //byte GRO_MEAS_STAT;
    //word ErrorCode;   
   
    byte FlagNBIG;
    word NBGR_GDAT[6];   //GAMMA DATA
    word NBGR_MDAT[9];   //MEASURE DIMA DATA
    word NBGR_SMDAT[9];  //STATIC MEASURE DIMA DATA
    
    byte FlagDART;
    byte FlagIFPT;
    byte FlagDWPR;
    word DWPR[3];
}OPRAT_DAT;

typedef struct
{
    dword RTC;
    byte DataID;
    
    byte FlagDIM;   //DIM
    word DIM_STATUS;
    
    word Status;    //DBC   
    
    byte FlagUSM;  //USM       
    
    byte FlagVSM;    //VSM
    byte FlagDMBRO;  //方位信息广播   
    
    byte FlagACPRR; //ACPR  
    
    byte FlagPMTT;  //PMT  
    byte CGR_APIC;  //PMT APIC
    
    byte FlagTGCC;  //TGC
    byte TGC_STATUS;    
    
    byte FlagSFUU;  //SFU   
    byte SFU_STATUS;
    byte RST_MONITOR;  
    byte TGC_ECHO; //LH want to say hello to TGC
        
    byte FlagINPT;  //中子INP    
    byte FlagLDIT;  //密度LDI 
    
    byte FlagAGRT;
    byte FlagNBIG;
    byte FlagMAST;
    
    byte FlagDART;
    byte FlagIFPT;
    
    byte FlagDWPR;  
    byte Blank[2]; //blank data for future use
    
}OPRAT_SAVDAT;



typedef struct        //Bus data type
{
   	byte Type;
	  uint Dat;
}TBUSDAT;

typedef struct                 //System buffer
{    
    word Len; 
    word LenS;
    word Source;    //0x02 表示DBC发出的命令
    word CMD;
    word Dat[520]; 
}SYSBUF;

typedef struct
{
    word CMD;                       
    word Len;    
}SYSBUFFLG;

typedef struct
{
    byte DIM;
    byte LDI;
    byte INP;
    byte ACPR;
    byte AGR;
    byte RST;
}TOOLMONITOR_;

#define OS_CHKS_ERROR   144u

typedef union {
  word Word;
  struct {
    word PRE        :1;      //pressure power
    word TBUS       :1;      //High voltage  0:off  1:on 
    word LBUS       :1;      //undefine
    word TIME       :1;      //Synchronized or not
    word CONFIG     :1;      //undefine                                       
    word FREQ       :1;      //frequency 0:80K  1:40K                                 
    word            :2;      //undefine 
    
    word WORKMODE   :2;  //work mode =0, DBC is in Initializing or is being Reset.
                              //=1, Sleep mode
                              //=2, STANDBY mode. This is the default MODE after reset. 
                              //=3, OPERATION mode                                                          
    word DIAG        :1;
    word VOL         :1;   
    word CUR         :1;
    word TEMPP       :1;                                  
    word DIAM        :1;      //diameter power
    word VIBR        :1;      //vibration power
                                     
                                          
  } Bits;
} STATUSSTR;


typedef union 
{
    dword dword;
      struct 
      {
        dword         :8;
        
        dword TGC     :1;  //BIT16
        dword SFU     :1;  //BIT17
        dword NBT     :1;  //近钻头工具
        dword AGR     :1;  //方位伽玛
        dword         :4;
        
        dword FTWD  :1; //随钻测压  BIT8
        dword ACOU  :1; //随钻声波  BIT9
        dword INP  :1; //随钻中子
        dword LDI  :1; //随钻密度
        dword GRO   :1;  //随钻陀螺
        
        dword       :3;  //
        
        dword DBC   :1;   //LSB，BIT0
        dword NGR   :1;
        dword DWPR  :1;   //BIT2
        dword PRE   :1;       
        dword VST   :1;
        dword DIAM  :1;       
        dword ACPR  :1;
        dword PMT   :1;  //BIT7 CGR
        
      }Bits;
}FLAGRTDATA;

typedef struct 	    //rtc  CLOCK
{
	word Second;
	word Minute;
	word Hour;
	word Day;
	word Weekday;
	word Month;
	word Year;
}RTCCLOCK;

typedef struct
{ 
    word TID;
    word ParTimeBro;
    word ParTimeDBC;
    word StInitStatus;
    word BackUp[3];    
    word CHK;
}CONFIG_REPAIR_1;

typedef struct
{
    word TID; 
    float P1_k;
    float P1_b;
    float P2_k;
    float P2_b;     
    word Backup[10];
    word CHK;         
}CONFIG_REPAIR_2;

typedef struct
{
    byte DiskNum;
    byte RecordLenth;
    word StarSec;
    word EndSec;     
    word BrokenLB;        
}DMSTAT;

#define MaxDiskCount 6
typedef struct
{ 
    word TID;
    word DisCount;
    DMSTAT DMS[MaxDiskCount];   //data memory status
    word CHK;
            
}CONFIG_REPAIR_3;

typedef struct
{
    word TID;
    word ParTimeDelay;    
    word ParTimeToMWD;
    word ParTimeSavePre;     
    FLAGRTDATA FlagRTD; 
    word PowerSavingET;  //LH want save battery power
    
    word BackUp[7];
    word  CHK;     //parameter checksum
}CONFIGPAR;



typedef struct
{
    byte BootSelect;
    word BCT;                //bootCount
    word PWD;                //Password
    char TID[10];            //ToolID
    byte Mode;               //work mode
    CONFIGPAR Config;       //real time config data
    CONFIG_REPAIR_1 CRP1;   //repair config data
    CONFIG_REPAIR_2 CRP2;   //repair config data
    CONFIG_REPAIR_3 CRP3;   //repair config data
    
    float BATC;              //batary capcity
    byte SecondAdr;
}EEPROM_DAT;


#define BootCount    eepromDat.BCT
#define PassWord     eepromDat.PWD
#define BatCap       eepromDat.BATC
#define DiskCount    eepromDat.CRP3.DisCount
#define DmStat       eepromDat.CRP3.DMS
#define ToolID       eepromDat.TID
#define SecondAdr   eepromDat.SecondAdr

//存储于 RAM 中
#define EEP_MODE     eepromDat.Mode

#define StatusAll    CurStatus.Word
#define Status_MODE  CurStatus.Bits.WORKMODE
#define Status_DIAG  CurStatus.Bits.DIAG
#define Status_VOL   CurStatus.Bits.VOL
#define Status_CUR   CurStatus.Bits.CUR
#define Status_TEMP  CurStatus.Bits.TEMPP
#define Status_DIAM  CurStatus.Bits.DIAM
#define Status_VIBR  CurStatus.Bits.VIBR

#define Status_PRE   CurStatus.Bits.PRE
#define Status_TBUS  CurStatus.Bits.TBUS
#define Status_LBUS  CurStatus.Bits.LBUS

#define Status_TIME  CurStatus.Bits.TIME
#define Status_CONFIG  CurStatus.Bits.CONFIG
#define Status_FREQ  CurStatus.Bits.FREQ



#define SysPar  eepromDat.Config
#define FlagRTD_All eepromDat.Config.FlagRTD.dword
#define FlagDBC  eepromDat.Config.FlagRTD.Bits.DBC
#define FlagNGR  eepromDat.Config.FlagRTD.Bits.NGR
#define FlagCWPR  eepromDat.Config.FlagRTD.Bits.CWPR 
#define FlagACPR  eepromDat.Config.FlagRTD.Bits.ACPR
#define FlagPMT  eepromDat.Config.FlagRTD.Bits.PMT

#define FlagTGC  eepromDat.Config.FlagRTD.Bits.TGC
#define FlagSFU  eepromDat.Config.FlagRTD.Bits.SFU
#define FlagAGR  eepromDat.Config.FlagRTD.Bits.AGR

#define FlagPRE  eepromDat.Config.FlagRTD.Bits.PRE
#define FlagVST  eepromDat.Config.FlagRTD.Bits.VST
#define FlagDIAM  eepromDat.Config.FlagRTD.Bits.DIAM 

#define FlagFTWD  eepromDat.Config.FlagRTD.Bits.FTWD
#define FlagACOU  eepromDat.Config.FlagRTD.Bits.ACOU
#define FlagINP  eepromDat.Config.FlagRTD.Bits.INP
#define FlagLDI  eepromDat.Config.FlagRTD.Bits.LDI                        
#define FlagGRO   eepromDat.Config.FlagRTD.Bits.GRO
#define FlagNBT   eepromDat.Config.FlagRTD.Bits.NBT
#define FlagDWPRT   eepromDat.Config.FlagRTD.Bits.DWPR





enum CCS{FL0=0,FL1=1,FL2=2,FL3=3,TEMP=4};


enum DHS
{
    Default,
    Quiet,
    RealTime,
    Diagnostic,
    CDL_Up,
    CDL_Wait
};

typedef struct 
{
    byte CompleteTBusOFF;
    byte CompleteLBusOFF;    
}TLBUS_FLAG;

typedef struct
{
    dword RTC;
    word GA;
    word GB;  
}NGR_DAT;

typedef struct
{
    dword RTC;
    byte DatID;
    byte BootC;     
    word Status;        
    float Cur;
    float Vol;
    float Temp; 
    float BatC; 
    float InnPre;
    float AnnPre;
}DBCLOG;

typedef struct
{
    dword RTC;
    byte DataID;
    word SavPre[6][2];    
}PRESAVEDAT;

typedef struct        //Bus data type
{   	
    uchar Type;     //indicate CW or DW
	word Dat;		//actual data
}BusDAT;

#define MAXLEN 518
typedef struct                 //System buffer
{ 
  BusDAT Buf[MAXLEN];  //Data buffer  
  word front;
  word rear;
}QUEUE;

#define MAXLENBYTE 1050
typedef struct                 //System buffer
{ 
  byte Buf[MAXLENBYTE];  //Data buffer  
  word front;
  word rear;
}QUEUEBYTE;



#endif /* __GLOBALVARDEF_H_ */