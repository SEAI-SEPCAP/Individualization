#ifndef __SERVO_CONTROL_H__
#define __SERVO_CONTROL_H__

//#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>

#define ZERO1 2800 // fechado
#define ZERO2 2800
#define ZERO3 2800
#define ZERO4 2800
#define ZERO5 3100
#define ZERO6 3700
#define ZERO7 3400

#define MAX1 1500 // aberto
#define MAX2 1600
#define MAX3 1400
#define MAX4 1700
#define MAX5 1900
#define MAX6 2700
#define MAX7 1800

#define SERVO_OCR_1 OCR3B
#define SERVO_OCR_2 OCR3C
#define SERVO_OCR_3 OCR1B
#define SERVO_OCR_4 OCR3A
#define SERVO_OCR_5 OCR4A
#define SERVO_OCR_6 OCR4B
#define SERVO_OCR_7 OCR4C

#define MAX(index) (MAX##index)
#define ZERO(index) (ZERO##index)
#define SERVO_OCR(index) SERVO_OCR_##index

void openServo(uint8_t index);
void closeServo(uint8_t index);

#endif
