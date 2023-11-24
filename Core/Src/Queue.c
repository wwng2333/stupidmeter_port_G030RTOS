#include "main.h"
#include "cmsis_os2.h"
#include "Queue.h"
#include "math.h"

void Countqueue(struct Queue* queue)
{
	float max = -0xFFFF, min = 0xFFFF, avg = 0.0f;
	uint8_t i = queue->front;
	uint8_t count = 0;
	while (i != queue->rear) {
		if(queue->arr[i] > max) max = queue->arr[i];
		if(queue->arr[i] < min) min = queue->arr[i];
		if(queue->arr[i] != 0)
		{
			avg += queue->arr[i];
			count++;
		}
		i = (i + 1) % Queue_SIZE;
	}
	queue->max = max;
	queue->min = min;
	queue->avg = avg / count;
}

void enqueue(Queue* queue, float item) {
	#if __Crazy_DEBUG
	SEGGER_RTT_SetTerminal(3);
	SEGGER_RTT_printf(0, "enqueue=%f\n", item);
	SEGGER_RTT_SetTerminal(0);
	#endif
	Countqueue(queue);
	if ((queue->rear + 1) % Queue_SIZE == queue->front) {
		queue->front = (queue->front + 1) % Queue_SIZE;
	}
	queue->arr[queue->rear] = item;
	queue->rear = (queue->rear + 1) % Queue_SIZE;
}

//int dequeue(struct Queue* queue) {
//    if (queue->front == queue->rear) {
//        SEGGER_RTT_printf(0, "Queue is empty\n");
//        return -1;
//    }
//    int item = queue->arr[queue->front];
//    queue->front = (queue->front + 1) % SIZE;
//    return item;
//}