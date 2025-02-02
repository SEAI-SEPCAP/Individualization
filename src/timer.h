#ifndef __TIMER_H__
#define __TIMER_H__

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>

#include "debug.h"

#define numberOfTimerVars 12

typedef uint16_t timer;

timer timerVars[numberOfTimerVars] = {0}; // PWM F=50, therefore we need 500
                                          // interrupts to complete 10s

#define inverterTimer 0
#define holdDiscTimer 1
#define capsuleDetectionDebounceTimer 2
#define emergencyDebounceTimer 3
#define servoTimer_1 4
#define servoTimer_2 5
#define servoTimer_3 6
#define servoTimer_4 7
#define servoTimer_5 8
#define servoTimer_6 9
#define servoTimer_7 10

#define DEBOUNCE_TIME 6
#define Time_Invert 610
#define HOLD_DISC_TIME 55
#define OPEN_SERVO_TIME 45

void setTimer(uint8_t index, timer value) { timerVars[index] = value; }

timer getTimer(uint8_t index) { return timerVars[index]; }
bool timerIsDone(uint8_t index) { return !getTimer(index); }

void initTimersTimer() {
    TCNT2 = 0;                                      // Set timer5 count zero
    TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20); // Fast PWM: TOP: ICR5
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);     // Preesc = 8
    TIMSK2 |= _BV(TOIE2);                           // Overflow interrupt enable
}

// Timer interrupt
ISR(TIMER2_OVF_vect) {
    for (uint8_t i = 0; i < numberOfTimerVars; i++) {
        if (timerVars[i]) {
            timerVars[i]--;
        }
    }
}

#endif
