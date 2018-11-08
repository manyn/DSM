#ifndef _FIFO_H
#define _FIFO_H  

#define    OK 1
#define ERROR 0



//初始化队列
int InitQueue(QUEUE *q);
int EnQueue(QUEUE *q,BusDAT e);
int DeQueue(QUEUE *q,BusDAT *e);
int QueueLenth(QUEUE *q);

int InitQueueByte(QUEUEBYTE *q);
int EnQueueByte(QUEUEBYTE *q,byte e);
int DeQueueByte(QUEUEBYTE *q,byte *e);
int QueueLenthByte(QUEUEBYTE *q);

#endif