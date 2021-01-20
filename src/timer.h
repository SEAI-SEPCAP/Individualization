#ifndef __TIMER_H__
#define __TIMER_H__

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>

#define numberOfTimerVars 5

typedef uint16_t timer;

timer timerVars[numberOfTimerVars]; // PWM F=50, therefore we need 500
                                    // interrupts to complete 10s

#define inverterTimer 0
#define openServoTimer 1
#define capsuleDetectionDebounceTimer 2

void setTimer(uint8_t index, timer value) { timerVars[index] = value; }

timer getTimer(uint8_t index) { return timerVars[index]; }
bool timerIsDone(uint8_t index) { return !getTimer(index); }

// Timer interrupt
ISR(TIMER5_OVF_vect) {
    for (uint8_t i = 0; i < numberOfTimerVars; i++) {
        if (timerVars[i]) {
            timerVars[i]--;
        }
    }
}

#endif
