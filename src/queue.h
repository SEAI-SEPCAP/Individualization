#ifndef __QUEUE_H__
#define __QUEUE_H__

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

// Select servo data - Queue
#define SIZE 10
uint8_t selected_servo[SIZE];
uint8_t servo = 0;
uint8_t rear = -1;
uint8_t front = -1;
uint8_t elements = 0;

// Inset the destination code to the end of the queue
void insert_code(uint8_t dest_code) {
    if (rear == SIZE - 1)
        return;
    else {
        if (front == -1)
            front = 0;
        rear++;
        selected_servo[rear] = dest_code;
        elements++;
    }
}

// Select the laste code inserted and remove it frome the queue
void display_rem_code(void) {
    if (front == -1) {
        return;
    } else {
        servo = selected_servo[front];
        front++;
        elements--;
        if (front > rear) {
            front = rear = -1;
            elements = 0;
        }
    }
}

bool isempty(void) { return (elements == 0); }

#endif
