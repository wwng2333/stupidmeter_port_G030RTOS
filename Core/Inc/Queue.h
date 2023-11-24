#ifndef __QUEUE_H_
#define __QUEUE_H_

#include "main.h"

#define Queue_SIZE 60

typedef struct Queue{
	float arr[Queue_SIZE];
	int front, rear;
	float max, min, avg;
} Queue;

void Countqueue(struct Queue* queue);
void enqueue(struct Queue* queue, float item);

#endif