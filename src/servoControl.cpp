#include "servoControl.h"

void openServo(uint8_t index) {
    switch (index) {
    case 1:
        SERVO_OCR(1) = MAX(1);
        break;
    case 2:
        SERVO_OCR(2) = MAX(2);
        break;
    case 3:
        SERVO_OCR(3) = MAX(3);
        break;
    case 4:
        SERVO_OCR(4) = MAX(4);
        break;
    case 5:
        SERVO_OCR(5) = MAX(5);
        break;
    case 6:
        SERVO_OCR(6) = MAX(6);
        break;
    case 7:
        SERVO_OCR(7) = MAX(7);
        break;
    }
}

void closeServo(uint8_t index) {
    switch (index) {
    case 1:
        SERVO_OCR(1) = ZERO(1);
        break;
    case 2:
        SERVO_OCR(2) = ZERO(2);
        break;
    case 3:
        SERVO_OCR(3) = ZERO(3);
        break;
    case 4:
        SERVO_OCR(4) = ZERO(4);
        break;
    case 5:
        SERVO_OCR(5) = ZERO(5);
        break;
    case 6:
        SERVO_OCR(6) = ZERO(6);
        break;
    case 7:
        SERVO_OCR(7) = ZERO(7);
        break;
    }
}