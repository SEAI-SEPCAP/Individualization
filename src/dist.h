#include "servoControl.h"
#include "timer.h"

#define DIST_STATE_IDLE 0
#define DIST_STATE_SERVO_1_OPEN 1
#define DIST_STATE_SERVO_1_CLOSED 2
#define DIST_STATE_SERVO_2_OPEN 3
#define DIST_STATE_SERVO_2_CLOSED 4
#define DIST_STATE_SERVO_3_OPEN 5
#define DIST_STATE_SERVO_3_CLOSED 6
#define DIST_STATE_SERVO_4_OPEN 7
#define DIST_STATE_SERVO_4_CLOSED 8
#define DIST_STATE_SERVO_5_OPEN 9
#define DIST_STATE_SERVO_5_CLOSED 10
#define DIST_STATE_SERVO_6_OPEN 11
#define DIST_STATE_SERVO_6_CLOSED 12
#define DIST_STATE_SERVO_7_OPEN 13
#define DIST_STATE_SERVO_7_CLOSED 14

#define SERVO_OPEN_TIME 30 // 800 ms

uint8_t distState = DIST_STATE_IDLE;

void distStateMachine(uint8_t selected) {

    switch (distState) {
    case DIST_STATE_IDLE:
        switch (selected) {
        case 1:
            distState = DIST_STATE_SERVO_1_CLOSED;
            break;

        case 2:
            distState = DIST_STATE_SERVO_2_CLOSED;
            break;

        case 3:
            distState = DIST_STATE_SERVO_3_CLOSED;
            break;

        case 4:
            distState = DIST_STATE_SERVO_4_CLOSED;
            break;

        case 5:
            distState = DIST_STATE_SERVO_5_CLOSED;
            break;

        case 6:
            distState = DIST_STATE_SERVO_6_CLOSED;
            break;

        case 7:
            distState = DIST_STATE_SERVO_7_CLOSED;
            break;

        default:
            distState = DIST_STATE_IDLE;
            break;
        }

        break;

    case DIST_STATE_SERVO_1_OPEN:
        if (timerIsDone(openServoTimer)) {
            closeServo(1);
            distState = DIST_STATE_IDLE;
        }
        break;

    case DIST_STATE_SERVO_1_CLOSED:
        openServo(1);
        setTimer(openServoTimer, SERVO_OPEN_TIME);
        distState = DIST_STATE_SERVO_1_OPEN;
        break;

    case DIST_STATE_SERVO_2_OPEN:
        if (timerIsDone(openServoTimer)) {
            closeServo(2);
            distState = DIST_STATE_IDLE;
        }
        break;

    case DIST_STATE_SERVO_2_CLOSED:
        openServo(2);
        setTimer(openServoTimer, SERVO_OPEN_TIME);
        distState = DIST_STATE_SERVO_2_OPEN;
        break;

    case DIST_STATE_SERVO_3_OPEN:
        if (timerIsDone(openServoTimer)) {
            closeServo(3);
            distState = DIST_STATE_IDLE;
        }
        break;

    case DIST_STATE_SERVO_3_CLOSED:
        openServo(3);
        setTimer(openServoTimer, SERVO_OPEN_TIME);
        distState = DIST_STATE_SERVO_3_OPEN;
        break;

    case DIST_STATE_SERVO_4_OPEN:
        if (timerIsDone(openServoTimer)) {
            closeServo(4);
            distState = DIST_STATE_IDLE;
        }
        break;

    case DIST_STATE_SERVO_4_CLOSED:
        openServo(4);
        setTimer(openServoTimer, SERVO_OPEN_TIME);
        distState = DIST_STATE_SERVO_4_OPEN;
        break;

    case DIST_STATE_SERVO_5_OPEN:
        if (timerIsDone(openServoTimer)) {
            closeServo(5);
            distState = DIST_STATE_IDLE;
        }
        break;

    case DIST_STATE_SERVO_5_CLOSED:
        openServo(5);
        setTimer(openServoTimer, SERVO_OPEN_TIME);
        distState = DIST_STATE_SERVO_5_OPEN;
        break;

    case DIST_STATE_SERVO_6_OPEN:
        if (timerIsDone(openServoTimer)) {
            closeServo(6);
            distState = DIST_STATE_IDLE;
        }
        break;

    case DIST_STATE_SERVO_6_CLOSED:
        openServo(6);
        setTimer(openServoTimer, SERVO_OPEN_TIME);
        distState = DIST_STATE_SERVO_6_OPEN;
        break;

    case DIST_STATE_SERVO_7_OPEN:
        if (timerIsDone(openServoTimer)) {
            closeServo(7);
            distState = DIST_STATE_IDLE;
        }
        break;

    case DIST_STATE_SERVO_7_CLOSED:
        openServo(7);
        setTimer(openServoTimer, SERVO_OPEN_TIME);
        distState = DIST_STATE_SERVO_7_OPEN;
        break;
    }
}

#define PWMTOP 39999 // TIMER TOP for PRESC=8

void initDistTimers() {

    TCNT1 = 0;     // Set timer1 count zero
    ICR1 = PWMTOP; // TOP count for timer1 -> FPWM = FOSC/(N*(1+TOP)) with
                   // FPWM=50 and N=8
    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | (0 << COM1A0); // Non inverter PWM
    TCCR1A |= _BV(WGM11) | (0 << WGM10);                // Fast PWM: TOP: ICR1
    TCCR1B = _BV(WGM13) | _BV(WGM12);                   // Fast PWM: TOP: ICR1
    TCCR1B |= (0 << CS12) | _BV(CS11) | (0 << CS10);    // Preesc = 8

    TCNT3 = 0;     // Set timer1 count zero
    ICR3 = PWMTOP; // TOP count for timer1 -> FPWM = FOSC/(N*(1+TOP)) with
                   // FPWM=50 and N=8
    TCCR3A = _BV(COM3B1) | _BV(COM3C1) | _BV(COM3A1) |
             (0 << COM3A0);                          // Non inverter PWM
    TCCR3A |= _BV(WGM31) | (0 << WGM30);             // Fast PWM: TOP: ICR1
    TCCR3B = _BV(WGM33) | _BV(WGM32);                // Fast PWM: TOP: ICR1
    TCCR3B |= (0 << CS32) | _BV(CS31) | (0 << CS30); // Preesc = 8

    TCNT4 = 0;     // Set timer1 count zero
    ICR4 = PWMTOP; // TOP count for timer1 -> FPWM = FOSC/(N*(1+TOP)) with
                   // FPWM=50 and N=8
    TCCR4A = _BV(COM4B1) | _BV(COM4C1) | _BV(COM4A1) |
             (0 << COM4A0);                          // Non inverter PWM
    TCCR4A |= _BV(WGM41) | (0 << WGM40);             // Fast PWM: TOP: ICR1
    TCCR4B = _BV(WGM43) | _BV(WGM42);                // Fast PWM: TOP: ICR1
    TCCR4B |= (0 << CS42) | _BV(CS41) | (0 << CS40); // Preesc = 8
}

/*********************************************
 * Distribution SERVO ports
 *********************************************/
void initDistPins(void) {
    // SERVO as OUT
    DDRE |= _BV(PINE4); // Digital Pin 2, OC3B, Servo1 - PORT PWM2
    DDRE |= _BV(PINE5); // Digital Pin 3, OC3C, Servo2 - PORT PWM3
    DDRB |= _BV(PINB6); // Digital Pin 12, OC1B, Servo3 - PORT PWM12
    DDRE |= _BV(PINE3); // Digital Pin 5, OC3A, Servo4 - PORT PWM5
    DDRH |= _BV(PINH3); // Digital Pin 6, OC4A, Servo5 - PORT PWM6
    DDRH |= _BV(PINH4); // Digital Pin 7, OC4B, Servo6 - PORT PWM7
    DDRH |= _BV(PINH5); // Digital Pin 8, OC4C, Servo7 - PORT PWM8
}

void resetServoPositions() {
    closeServo(1);
    closeServo(2);
    closeServo(3);
    closeServo(4);
    closeServo(5);
    closeServo(6);
    closeServo(7);
}