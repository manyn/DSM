#ifndef _SCI_H
#define _SCI_H


//����ͨ�żĴ�������־λ����
#define ReSendStatusR SCI0SR1  		    //SCI״̬�Ĵ���
#define ReTestBit     5        		    //���ջ���������־λ
#define SendTestBit   7        		    //���ͻ������ձ�־λ
#define ReSendDataR   SCI0DRL  		    //���ݼĴ���


#pragma CODE_SEG DEFAULT


//����ͨ����غ�������
void SCIInit(void);            	      //���пڳ�ʼ����������
void SCIInitHigh(void);
void SCI_ClearBuf(void);
void SCI_Send(byte *buffer,int lenth);
int SCI_Recv(byte *buffer,int lenth);
__interrupt void  SCI0_ISR(void);







#pragma CODE_SEG DEFAULT

#endif