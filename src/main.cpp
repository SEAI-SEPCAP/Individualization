#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

#include "debug.h"
#include "dist.h"
#include "servoControl.h"
#include "sms.h"
#include "timer.h"

#define FCPU 16000000ul

#define Motor_indv PINL3 // Individualization Servo Output Port - PORT 46
#define IR_Sensor PIND0  // INT0 - IR Sensor Interrupt
#define EM_BUTTOM_EMERGENCY PIND1 // INT1 - EM Bottom Interrupt - Emergency
#define EM_BUTTOM_RESUME PIND2    // INT2 - EM Bottom Interrupt - Resume

#define Time_Invert 500
#define MAX_CCW 1.0 // Duty-cycle for MAX CCW speed
#define MAX_CW 2.0  // Dutry-cycle for MAX CW speed
#define ZERO_W 1.5  // Duty-cycle for STOP Disk
#define P_FCLK 2000 // (16M/1000)/8
#define DISC_SPEED 15

bool new_capsule = false;

void disc_speed_rot(double speed) {
    if (speed <= 0) {
        OCR5A = (ZERO_W - abs((speed / 100) * (MAX_CCW - ZERO_W))) * P_FCLK;
    } else {
        OCR5A = (ZERO_W + abs((speed / 100) * (MAX_CW - ZERO_W))) * P_FCLK;
    }
}

/*********************************************
 * Timer 1,2,3 Fast PWM mode
 * Timer 1 with interrupt
 *********************************************/
void initTimers(void) {
    initDistTimers();

    TCNT5 = 0;     // Set timer5 count zero
    ICR5 = PWMTOP; // TOP count for timer5 -> FPWM = FOSC/(N*(1+TOP)) with
                   // FPWM=50 and N=8
    TCCR5A = _BV(COM5A1) | (0 << COM5A0);            // Non inverter PWM
    TCCR5A |= _BV(WGM51) | (0 << WGM50);             // Fast PWM: TOP: ICR5
    TCCR5B = _BV(WGM53) | _BV(WGM52);                // Fast PWM: TOP: ICR5
    TCCR5B |= (0 << CS52) | _BV(CS51) | (0 << CS50); // Preesc = 8
}

/*********************************************
 * Timer1 init
 * Timer to trigger the inversion of the disk rotation
 *********************************************/

void IR_interrupt(void) {
    /* Set Interrupt pins as input and activate internal pull-ups */
    DDRD &= (0 << IR_Sensor);
    PORTD |= _BV(IR_Sensor);
    /* Interrupt request at falling edge for INT0 */
    EICRA |= _BV(ISC01);
    /* Enable INT0 */
    EIMSK |= _BV(INT0);
}

void EM_interrupt(void) {
    /* Set Interrupt pins as input */
    PORTD |= _BV(EM_BUTTOM_EMERGENCY);
    /* Interrupt request at any edge for INT1 */
    EICRA |= _BV(ISC11); //
    /* Enable INT1 */
    EIMSK |= _BV(INT1);

    /* Set Interrupt pins as input*/
    PORTD |= _BV(EM_BUTTOM_RESUME);
    /* Interrupt request at any edge for INT1 */
    EICRA |= _BV(ISC21) | _BV(ISC20); //
    /* Enable INT1 */
    EIMSK |= _BV(INT2);
}

/*********************************************
 * Individualization Setup
 *********************************************/
void setup_indv(void) {
    DDRL |= _BV(Motor_indv); // Digital Pin 11, OCR1A
}

/*********************************************
 * Individualization State Machine
 *********************************************/
void indv_control(void) {
    switch (state) {
    case 0:
        if (!inv) {
            state = 1;                            // Normal Rotation
            setTimer(inverterTimer, Time_Invert); // Reset Timer
        } else {
            state = 2;                            // Inverted Rotation
            setTimer(inverterTimer, Time_Invert); // Reset Timer
        }
        break;

    case 1:
        /*if (timerIsDone(inverterTimer)) {
            state = 0;
            inv = true; // Set Inverted rotation
        }*/
        break;

    case 2:
        /*if (timerIsDone(inverterTimer)) {
            state = 0;
            inv = false; // Set Normal Rotation
        }*/
        break;
    }

    switch (state) {
    case 0:
        disc_speed_rot(0);
        break;
    case 1:
        if (hold_disc)
            disc_speed_rot(0);
        else
            disc_speed_rot(DISC_SPEED);
        break;
    case 2:
        if (hold_disc)
            disc_speed_rot(0);
        else
            disc_speed_rot(-DISC_SPEED);
        break;
    }
}

void setup_uart(void) {

    /* Set baud rate */
    long BAUD = 9600;
    int UBBR_VAL = 16000000 / (BAUD * 16) - 1;
    UBRR0H = (uint8_t)(UBBR_VAL >> 8); // Define Baudrate
    UBRR0L = (uint8_t)UBBR_VAL;

    UCSR0C |= (3 << UCSZ10);               // 8 Data bits configuration
    UCSR0C |= (0 << USBS0);                // 1 stop bit
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0); // Activate Rx, Tx
}

void initialization(void) {
    initDistPins();
    initTimers();
    initTimersTimer();

    setup_indv();
    IR_interrupt();
    // EM_interrupt();

    initDebugLED();
    turnOffDebugLED();

    setup_uart();
    sei(); // Enable global int

    resetServoPositions();

    setTimer(inverterTimer, Time_Invert); // Reset Timer

    state = 0;
    inv = false;
}

/*********************************************
 * MAIN
 *********************************************/
int main(void) {
    // Initialization
    initialization();

    while (1) {
        receive_data();

        if (timerIsDone(openServoTimer)) {
            hold_disc = false;
        } /*else {
            turnOffDebugLED();
        }*/

        /* if (emergency) {
             disc_speed_rot(0);   // Stop the disk
             distStateMachine(0); // Idle state - Don't do anything
         } else */
        if (operation) {
            indv_control();
            distStateMachine(selected_servo, new_capsule);
            if (new_capsule) {
                new_capsule = false;
            }
        } else {
            disc_speed_rot(0);
            selected_servo = 0;
            distStateMachine(selected_servo);
            state = 0;
            inv = false;
            resetServoPositions();
        }
    }
}

// Sensor interrupt
ISR(INT0_vect) {
    setTimer(inverterTimer, Time_Invert); // Reset the timer
    if (timerIsDone(capsuleDetectionDebounceTimer)) {
        setTimer(capsuleDetectionDebounceTimer, DEBOUNCE_TIME);
        selected_servo = queuePop();
        new_capsule = true;
        sendNewCapsuleDetection();
    }
}

// Emergency interrupt - any edge generates an interrupt
// FE - Emergency is TRUE
// RE - Emergency is FALSE
ISR(INT1_vect) {
    if (timerIsDone(emergencyDebounceTimer)) {
        setTimer(emergencyDebounceTimer, DEBOUNCE_TIME);
        emergency = true;
        operation = false;
        // sendEmergency_Emergency();
    }
}

ISR(INT2_vect) {
    if (timerIsDone(emergencyDebounceTimer)) {
        setTimer(emergencyDebounceTimer, DEBOUNCE_TIME);
        emergency = false;
        // sendEmergency_Resume();
    }
}