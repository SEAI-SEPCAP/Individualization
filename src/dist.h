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

#define SERVO_OPEN_TIME 40 // 800 ms

uint8_t distState = DIST_STATE_IDLE;

void distStateMachine(void) {

    switch (distState) {
    case DIST_STATE_IDLE:
        // Wait for packet
        if (timerIsDone(2)) { // debug code
            distState = DIST_STATE_SERVO_1_CLOSED;
        }
        break;

    case DIST_STATE_SERVO_1_OPEN:
        if (timerIsDone(openServoTimer)) {
            closeServo(1);
            distState = DIST_STATE_IDLE;
            setTimer(2, 250); // debug code
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