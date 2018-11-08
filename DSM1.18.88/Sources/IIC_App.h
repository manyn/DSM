
#ifndef __IIC_APP
#define __IIC_APP


/*Include shared modules, which are used for whole project*/


#define SlaveAdr 0xA2 

#define SDA_1   PTJ_PTJ6=1
#define SDA_0   PTJ_PTJ6=0
#define DIR_IN  DDRJ_DDRJ6=0
#define DIR_OUT DDRJ_DDRJ6=1 
#define SCL_1   PTJ_PTJ7=1
#define SCL_0   PTJ_PTJ7=0
#define READ_SDA PTJ_PTJ6


void InitRtc();
void ReadRtc(void);
void WriteRtc();
void CacuSecondTime(dword *TimeSec);
void CacuClockTime(dword TimeSec);


__interrupt  void RTC_ISR(void);

#endif 
