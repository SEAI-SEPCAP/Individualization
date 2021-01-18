#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>

#include "dist.h"
#include "servoControl.h"
#include "timer.h"

#define FCPU 16000000ul

#define Motor_indv PINL3 // Individualization Servo Output Port - PORT 46
#define IR_Sensor PIND0  // INT0 - IR Sensor Interrupt
#define EM_BUTTOM PIND1  // INT1 - EM Bottom Interrupt

#define Time_Invert 500
#define MAX_CCW 1.0 // Duty-cycle for MAX CCW speed
#define MAX_CW 2.0  // Dutry-cycle for MAX CW speed
#define ZERO_W 1.5  // Duty-cycle for STOP Disk
#define P_FCLK 2000 // (16M/1000)/8

uint8_t state;
float dc; // Duty-cycle
bool inv;

uint16_t addr;
uint16_t r_data;
uint16_t message_type;

uint16_t selected_servo = 0;

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
    TIMSK5 |= _BV(TOIE5); // Overflow interrupt enable
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
    /* Set Interrupt pins as input and activate internal pull-ups */
    DDRD &= (0 << EM_BUTTOM);
    PORTD |= _BV(EM_BUTTOM);
    /* Interrupt request at falling edge for INT1 */
    EICRA |= _BV(ISC11);
    /* Enable INT0 */
    EIMSK |= _BV(INT1);
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
        if (false == inv) {
            state = 1;                            // Normal Rotation
            setTimer(inverterTimer, Time_Invert); // Reset Timer
        } else {
            state = 2;                            // Inverted Rotation
            setTimer(inverterTimer, Time_Invert); // Reset Timer
        }
        break;

    case 1:
        if (0 == timerVars[0]) {
            state = 0;
            inv = true; // Set Inverted rotation
        }
        break;

    case 2:
        if (0 == timerVars[0]) {
            state = 0;
            inv = false; // Set Normal Rotation
        }
        break;
    }

    switch (state) {
    case 0:
        disc_speed_rot(0); // STOP
        // OCR1A = 3000; // Pulse with 1.5ms (0 degrees / STOP)
        break;
    case 1:
        disc_speed_rot(70);
        // OCR1A = 2800; // Pulse with 1ms (90 degrees / CCW max speed)
        break;
    case 2:
        disc_speed_rot(-70);
        // OCR1A = 4000; // Pulse with 1ms (-90 degrees / CW max speed)
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

bool isData(void) { return (UCSR0A & (1 << RXC0)); }

void receive_Data(void) {
    /*
    while (!(UCSR0A & (1 << RXC0)))
        ;            // Waits until has new data to be read
    */
    if (isData()) {
        r_data = UDR0;                  // Saves the data to be analised below
        message_type = r_data & (0x0F); // Read the message type
        if (1 != message_type)
            // If ADDR is different from 0x01, it is an emergency message
            // or another message wihtout meaning for dirtibution
            return;
    } else {
        return;
    }

    while (!(UCSR0A & (1 << RXC0)))
        ;                  // Waits until has new data to be read
    selected_servo = UDR0; // Saves the data to be analised below
}

void send_Data(uint8_t Data) {

    while (!(UCSR0A & (1 << UDRE0)))
        ;        // Wait until the buffer is empty
    UDR0 = Data; // Copy to UDR0 the correspondent data
}

void initialization(void) {
    initDistPins();
    initTimers();

    setup_indv();
    IR_interrupt();

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
        receive_Data();
        indv_control();
        distStateMachine(selected_servo);
    }
}

// Sensor interrupt
ISR(INT0_vect) {
    setTimer(inverterTimer, Time_Invert); // Reset the timer
}

// Emergency interrupt
ISR(INT1_vect) {
    // TODO
}