#include "includes.h"


int InitQueue(QUEUE *q)
{
    q->front=q->rear=0;
    return OK;
    
}
int EnQueue(QUEUE *q,BusDAT e)
{
    if((q->rear+1)%MAXLEN==q->front)
    {     
        return(ERROR);
    }
    q->Buf[q->rear]=e;
    q->rear=(q->rear+1)%MAXLEN;
    return OK;    
}
int DeQueue(QUEUE *q,BusDAT *e)
{
    if(q->front==q->rear)
    {
        return ERROR;
    }
    *e=q->Buf[q->front];
    q->front=(q->front+1)%MAXLEN;
    return OK;
    
}
int QueueLenth(QUEUE *q)
{
    return (q->rear-q->front+MAXLEN)%MAXLEN;    
}


int InitQueueByte(QUEUEBYTE *q)
{
    q->front=q->rear=0;
    return OK;
    
}
int EnQueueByte(QUEUEBYTE *q,byte e)
{
    if((q->rear+1)%MAXLENBYTE==q->front)
    {     
        return(ERROR);
    }
    q->Buf[q->rear]=e;
    q->rear=(q->rear+1)%MAXLENBYTE;
    return OK;    
}
int DeQueueByte(QUEUEBYTE *q,byte *e)
{
    if(q->front==q->rear)
    {
        return ERROR;
    }
    *e=q->Buf[q->front];
    q->front=(q->front+1)%MAXLENBYTE;
    return OK;
    
}
int QueueLenthByte(QUEUEBYTE *q)
{
    return (q->rear-q->front+MAXLENBYTE)%MAXLENBYTE;    
}