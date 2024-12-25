#include "fml_queue.h"
#include "stdlib.h"
#include "string.h"

uint8_t CreateQueue(TRANS_QUEUE *Q,int maxsize,int framsize)
{
	Q->pBase = (uint8_t *)malloc(framsize*maxsize);
	if(NULL==Q->pBase)
	{
		return 0;       
	}
	Q->framsize = framsize;
	Q->front=0;        
	Q->rear=0;
	Q->maxsize=maxsize;
	Q->current_size = 0;
	
	return 1;
}

uint8_t FullQueue(TRANS_QUEUE *Q)
{
	if(Q->front==(Q->rear+1)%Q->maxsize)    
		return 1;
	else
		return 0;
}

uint8_t EmptyQueue(TRANS_QUEUE *Q)
{
	if(Q->front==Q->rear)    
		return 1;
	else
		return 0;
}

uint8_t Enqueue(TRANS_QUEUE *Q, void *val)
{
	if(FullQueue(Q))
		return 0;
	else
	{
    memcpy(Q->pBase+(Q->rear * Q->framsize),val,Q->framsize);
		Q->rear=(Q->rear+1)%Q->maxsize;
		Q->current_size++;
		return 1;
	}
}

uint8_t Dequeue(TRANS_QUEUE *Q, void *val)
{
	if(EmptyQueue(Q))
	{
		return 0;
	}
	else
	{
		memcpy(val,Q->pBase+(Q->front * Q->framsize),Q->framsize);
		Q->front=(Q->front+1)%Q->maxsize;
		Q->current_size--;
		return 1;
	}
}

int getLength(TRANS_QUEUE *Q)
{
	return (Q->rear - Q->front + Q->maxsize) % Q->maxsize;
}

void Clear_queue(TRANS_QUEUE *Q)
{
	Q->front = Q->rear;
}
