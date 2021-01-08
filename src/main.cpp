#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <util/delay.h>

#define T1TOP 625      // CTC mode OCR1A=625
#define PWMTOP 39999   // TIMER1 TOP for PRESC=8
#define Motor_indv PB1 // Individualization Servo Output Port
#define LED PB0        // Test Counter LED
#define IR_Sensor PD2  // PD2=INT0
#define MaxTime 500

int16_t time_p; // PWM F=50, therefore we need 500 interrupts to complete 10s
uint8_t state;
bool inv;

/*********************************************
 * Timer 1 Fast PWM mode
 *********************************************/
void timer1_init(void) {
    TCNT1 = 0;     // Set timer1 count zero
    ICR1 = PWMTOP; // TOP count for timer1 -> FPWM = FOSC/(N*(1+TOP)) with
                   // FPWM=50 and N=8
    TCCR1A = _BV(COM1A1) | _BV(0 << COM1A0); // Non inverter PWM
    TCCR1A |= _BV(WGM11) | _BV(0 << WGM10);  // Fast PWM: TOP: ICR1
    TCCR1B = _BV(WGM13) | _BV(WGM12);        // Fast PWM: TOP: ICR1
    TCCR1B |= _BV(0 << CS12) | _BV(CS11) | _BV(0 << CS10); // Preesc = 8
    TIMSK1 |= _BV(TOIE1); // Overflow interrupt enable
}

void hw_init_interrupt(void) {
    /* Set Interrupt pins as input and activate internal pull-ups */
    DDRD &= ~_BV(IR_Sensor);
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
    DDRB |= _BV(LED);        // LED test

    PORTB &= ~(_BV(LED)); // Clear Output Port
}

void start_timer1(int timer) { time_p = timer; }

/*********************************************
 * MAIN
 *********************************************/
int main(void) {
    // Initialization
    timer1_init();
    setup_indv();
    hw_init_interrupt();
    start_timer1(MaxTime); // Reset Timer
    sei();                 // Enable global int
    state = 0;
    inv = false;

    while (1) {
        switch (state) {
        case 0:
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
                _delay_ms(1000);
            }
            break;

        case 2:
            if (0 == time_p) {
                state = 0;
                inv = false; // Set Normal Rotation
                _delay_ms(1000);
            }
            break;
        }

        switch (state) {
        case 0:
            OCR1A = 3000; // Pulse with 1.5ms (0 degrees / STOP)
            break;
        case 1:
            OCR1A = 2000; // Pulse with 1ms (90 degrees / CCW max speed)
            break;
        case 2:
            OCR1A = 4000; // Pulse with 1ms (-90 degrees / CW max speed)
            break;
        }
    }
}

// Timer interrupt
ISR(TIMER1_OVF_vect) {
    if (time_p) {
        time_p--;
    } else {
        PORTB |= (_BV(LED)); // Toggle LED
    }
}

// Sensor interrupt
ISR(INT0_vect) {
    start_timer1(MaxTime); // Reset the timer
    PORTB ^= (_BV(LED));   // Toggle LED
}