#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

#define FCPU 16000000ul

#define T1TOP 625    // CTC mode OCR1A=625
#define PWMTOP 39999 // TIMER1 TOP for PRESC=8

#define Motor_indv PB5 // Individualization Servo Output Port
#define IR_Sensor PD0  // INT0 - IR Sensor Interrupt
#define MaxTime 500
#define MAX_CCW 1.0 // Duty-cycle for MAX CCW speed
#define MAX_CW 2.0  // Dutry-cycle for MAX CW speed
#define ZERO_W 1.5  // Duty-cycle for STOP Disk
#define P_FCLK 2000 // (16M/1000)/8

#define MAX1 1500 // aberto
#define ZERO1 2800
#define MAX3 1400 // aberto
#define ZERO3 2800
#define MAX2 1600
#define ZERO2 2800
#define MAX4 1700
#define ZERO4 2800
#define MAX5 1900
#define ZERO5 3100
#define MAX6 2700
#define ZERO6 3700
#define MAX7 1800
#define ZERO7 3400

int16_t time_p; // PWM F=50, therefore we need 500 interrupts to complete 10s
uint8_t state;
float dc; // Duty-cycle
bool inv;

void speed(double speed) {
    if (speed >= 0) {
        OCR1A = (ZERO_W - abs((speed / 100) * (MAX_CCW - ZERO_W))) * P_FCLK;
    } else {
        OCR1A = (ZERO_W + abs((speed / 100) * (MAX_CW - ZERO_W))) * P_FCLK;
    }
}

/*********************************************
 * Timer 1 Fast PWM mode
 *********************************************/
void timers(void) {
    TCNT1 = 0;     // Set timer1 count zero
    ICR1 = PWMTOP; // TOP count for timer1 -> FPWM = FOSC/(N*(1+TOP)) with
                   // FPWM=50 and N=8
    TCCR1A = _BV(COM1A1) | (0 << COM1A0);            // Non inverter PWM
    TCCR1A |= _BV(WGM11) | (0 << WGM10);             // Fast PWM: TOP: ICR1
    TCCR1B = _BV(WGM13) | _BV(WGM12);                // Fast PWM: TOP: ICR1
    TCCR1B |= (0 << CS12) | _BV(CS11) | (0 << CS10); // Preesc = 8
    TIMSK1 |= _BV(TOIE1); // Overflow interrupt enable

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

    OCR3B = ZERO1;
    OCR3C = ZERO2;
    OCR1A = ZERO3;
    OCR3A = ZERO4;
    OCR4A = ZERO5;
    OCR4B = ZERO6;
    OCR4C = ZERO7;
}

void hw_init_interrupt(void) {
    /* Set Interrupt pins as input and activate internal pull-ups */
    DDRD &= (0 << IR_Sensor);
    PORTD |= _BV(IR_Sensor);
    /* Interrupt request at falling edge for INT0 */
    EICRA |= _BV(ISC01);
    /* Enable INT0 */
    EIMSK |= _BV(INT0);
    /* Enable global interrupt flag */
    sei();
}

/*********************************************
 * Setup - Output's and Input's
 *********************************************/
void setup_indv(void) {
    DDRB |= _BV(Motor_indv); // Output Port (Port 9)
}

void setup_servos() {
    // SERVO as OUT
    DDRE |= 1 << PINE4; // Digital Pin 2, OC3B
    DDRE |= 1 << PINE5; // Digital Pin 3, OC3C
    DDRB |= 1 << PINB5; // Digital Pin 11, OC1A
    DDRE |= 1 << PINE3; // Digital Pin 5, OC3A
    DDRH |= 1 << PINH3; // Digital Pin 6, OC4A
    DDRH |= 1 << PINH4; // Digital Pin 7, OC4B
    DDRH |= 1 << PINH5; // Digital Pin 8, OC4C
}

void start_timer1(int timer) { time_p = timer; }

/*********************************************
 * MAIN
 *********************************************/
int main(void) {
    // Initialization
    timers();
    setup_indv();
    hw_init_interrupt();
    start_timer1(MaxTime); // Reset Timer
    sei();                 // Enable global int

    state = 0;
    inv = false;

    while (1) {
        switch (state) {
        case 0:
            _delay_ms(500);
            if (false == inv) {
                state = 1;             // Normal Rotation
                start_timer1(MaxTime); // Reset Timer
            } else {
                state = 2;             // Inverted Rotation
                start_timer1(MaxTime); // Reset Timer
            }
            break;

        case 1:
            if (0 == time_p) {
                state = 0;
                inv = true; // Set Inverted rotation
            }
            break;

        case 2:
            if (0 == time_p) {
                state = 0;
                inv = false; // Set Normal Rotation
            }
            break;
        }

        switch (state) {
        case 0:
            speed(0); // STOP
            // OCR1A = 3000; // Pulse with 1.5ms (0 degrees / STOP)
            break;
        case 1:
            speed(-30);
            // OCR1A = 2800; // Pulse with 1ms (90 degrees / CCW max speed)
            break;
        case 2:
            speed(30);
            // OCR1A = 4000; // Pulse with 1ms (-90 degrees / CW max speed)
            break;
        }
    }
}

// Timer interrupt
ISR(TIMER1_OVF_vect) {
    if (time_p) {
        time_p--;
    }
}

// Sensor interrupt
ISR(INT0_vect) {
    start_timer1(MaxTime); // Reset the timer
}