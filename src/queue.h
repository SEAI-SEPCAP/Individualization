#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

// Select servo data - Queue
#define QUEUE_SIZE 20
uint8_t queue[QUEUE_SIZE];

uint8_t elements = 0;

bool isQueueEmpty(void) { return (elements == 0); }
bool isQueueFull(void) { return (elements == QUEUE_SIZE); }

// Inset the destination code to the end of the queue
void queuePush(uint8_t dest_code) {
    if (isQueueFull())
        return;

    queue[elements++] = dest_code;
}

// Select the last code inserted and remove it frome the queue
uint8_t queuePop(void) {
    if (isQueueEmpty()) {
        return 0;
    } else {
        uint8_t ret = queue[0];

        for (uint8_t i = 0; i < elements - 1; i++) {
            queue[i] = queue[i + 1];
        }

        elements--;
        return ret;
    }
}

void queueClear() { elements = 0; }

#endif
