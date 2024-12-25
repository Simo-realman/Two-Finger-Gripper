#ifndef __FML_QUEUE_H
#define __FML_QUEUE_H

#include "main.h"

typedef struct{
	uint8_t len;
	uint8_t data[50];
}s_queue_uart_data;

typedef struct{
	uint32_t id;
	uint8_t len;
	uint8_t data[70];
}s_queue_can_data;

typedef struct {
	uint8_t *pBase;
	int front;    
	int rear;    
	int maxsize; 
	int framsize;
	int current_size;
}TRANS_QUEUE;

uint8_t CreateQueue(TRANS_QUEUE *Q,int maxsize,int framsize);
uint8_t FullQueue(TRANS_QUEUE *Q);
uint8_t EmptyQueue(TRANS_QUEUE *Q);
uint8_t Enqueue(TRANS_QUEUE *Q, void *val);
uint8_t Dequeue(TRANS_QUEUE *Q, void *val);
int getLength(TRANS_QUEUE *Q);
void Clear_queue(TRANS_QUEUE *Q);

#endif
