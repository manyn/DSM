#ifndef _SCI_H
#define _SCI_H


//串行通信寄存器及标志位定义
#define ReSendStatusR SCI0SR1  		    //SCI状态寄存器
#define ReTestBit     5        		    //接收缓冲区满标志位
#define SendTestBit   7        		    //发送缓冲区空标志位
#define ReSendDataR   SCI0DRL  		    //数据寄存器


#pragma CODE_SEG DEFAULT


//串行通信相关函数声明
void SCIInit(void);            	      //串行口初始化函数声明
void SCIInitHigh(void);
void SCI_ClearBuf(void);
void SCI_Send(byte *buffer,int lenth);
int SCI_Recv(byte *buffer,int lenth);
__interrupt void  SCI0_ISR(void);







#pragma CODE_SEG DEFAULT

#endif