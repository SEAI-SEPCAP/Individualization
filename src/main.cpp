#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>

#define FCPU 16000000ul

#define T1TOP 625    // CTC mode OCR1A=625
#define PWMTOP 39999 // TIMER1 TOP for PRESC=8

#define Motor_indv PINL3 // Individualization Servo Output Port - PORT 46
#define IR_Sensor PIND0  // INT0 - IR Sensor Interrupt

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
uint8_t selected_servo = 0;
uint8_t addr;
uint8_t aux;

void speed(double speed) {
    if (speed >= 0) {
        OCR5A = (ZERO_W - abs((speed / 100) * (MAX_CCW - ZERO_W))) * P_FCLK;
    } else {
        OCR5A = (ZERO_W + abs((speed / 100) * (MAX_CW - ZERO_W))) * P_FCLK;
    }
}

/*********************************************
 * Timer 1,2,3 Fast PWM mode
 * Timer 1 with interrupt
 *********************************************/
void timers(void) {
    /* TCNT1 = 0;     // Set timer1 count zero
     ICR1 = PWMTOP; // TOP count for timer1 -> FPWM = FOSC/(N*(1+TOP)) with
                    // FPWM=50 and N=8
     TCCR1A = _BV(COM1B1) | (0 << COM1A0);            // Non inverter PWM
     TCCR1A |= _BV(WGM11) | (0 << WGM10);             // Fast PWM: TOP: ICR1
     TCCR1B = _BV(WGM13) | _BV(WGM12);                // Fast PWM: TOP: ICR1
     TCCR1B |= (0 << CS12) | _BV(CS11) | (0 << CS10); // Preesc = 8

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
 */

    TCNT5 = 0;     // Set timer5 count zero
    ICR5 = PWMTOP; // TOP count for timer5 -> FPWM = FOSC/(N*(1+TOP)) with
                   // FPWM=50 and N=8
    TCCR5A = _BV(COM5A1) | (0 << COM5A0);            // Non inverter PWM
    TCCR5A |= _BV(WGM51) | (0 << WGM50);             // Fast PWM: TOP: ICR5
    TCCR5B = _BV(WGM53) | _BV(WGM52);                // Fast PWM: TOP: ICR5
    TCCR5B |= (0 << CS52) | _BV(CS51) | (0 << CS50); // Preesc = 8
    TIMSK5 |= _BV(TOIE5); // Overflow interrupt enable

    /* OCR3B = ZERO1;
     OCR3C = ZERO2;
     OCR1B = ZERO3;
     OCR3A = ZERO4;
     OCR4A = ZERO5;
     OCR4B = ZERO6;
     OCR4C = ZERO7;
     */
}

/*********************************************
 * Timer1 init
 * Timer to trigger the inversion of the disk rotation
 *********************************************/
void start_timer5(int timer) { time_p = timer; }

void IR_interrupt(void) {
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
 * Individualization Setup
 *********************************************/
void setup_indv(void) {
    DDRL |= _BV(Motor_indv); // Digital Pin 11, OCR1A
}

/*********************************************
 * Distribution SERVO ports
 *********************************************/
void setup_dist(void) {
    // SERVO as OUT
    DDRE |= 1 << PINE4; // Digital Pin 2, OC3B
    DDRE |= 1 << PINE5; // Digital Pin 3, OC3C
    DDRB |= 1 << PINB6; // Digital Pin 12, OC1B
    DDRE |= 1 << PINE3; // Digital Pin 5, OC3A
    DDRH |= 1 << PINH3; // Digital Pin 6, OC4A
    DDRH |= 1 << PINH4; // Digital Pin 7, OC4B
    DDRH |= 1 << PINH5; // Digital Pin 8, OC4C
}

/*********************************************
 * Individualization State Machine
 *********************************************/
void indv_control(void) {
    switch (state) {
    case 0:
        if (false == inv) {
            state = 1;             // Normal Rotation
            start_timer5(MaxTime); // Reset Timer
        } else {
            state = 2;             // Inverted Rotation
            start_timer5(MaxTime); // Reset Timer
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
        speed(-70);
        // OCR1A = 2800; // Pulse with 1ms (90 degrees / CCW max speed)
        break;
    case 2:
        speed(70);
        // OCR1A = 4000; // Pulse with 1ms (-90 degrees / CW max speed)
        break;
    }
}

void dist_control(void) {

    switch (selected_servo) {
    case 1:
        // delay(800);
        OCR3B = MAX1;
        // delay(800);
        OCR3B = ZERO1;
        break;

    case 2:
        // delay(800);
        OCR3C = MAX2;
        // delay(800);
        // now scan back from 180 to 0 degrees
        OCR3C = ZERO2;
        break;

    case 3:
        // delay(800);
        OCR1A = MAX3;
        // delay(800);
        // now scan back from 180 to 0 degrees
        OCR1A = ZERO3;
        break;

    case 4:
        // delay(800);
        OCR3A = MAX4;
        // delay(800);
        // now scan back from 180 to 0 degrees
        OCR3A = ZERO4;
        break;

    case 5:
        // delay(800);
        OCR4A = MAX5;
        // delay(800);
        // now scan back from 180 to 0 degrees
        OCR4A = ZERO5;
        break;

    case 6:
        // delay(800);
        OCR4B = MAX6;
        // delay(800);
        // now scan back from 180 to 0 degrees
        OCR4B = ZERO6;
        break;

    case 7:
        // delay(800);
        OCR4C = MAX7;
        // delay(800);
        // now scan back from 180 to 0 degrees
        OCR4C = ZERO7;
        break;

    default:
        break;
    }
}

void setup_uart(void) {

    /* Set baud rate */
    long BAUD = 9600;
    int UBBR_VAL = 16000000 / (BAUD * 16) - 1;
    UBRR0H = (uint8_t)UBBR_VAL >> 8; // Define Baudrate
    UBRR0L = (uint8_t)UBBR_VAL;

    UCSR0C |= (3 << UCSZ01);               // 8 Data bits configuration
    UCSR0C |= (0 << USBS0);                // 1 stop bit
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0); // Activate Rx, Tx
}

void receive_Data(void) {

    while (!(UCSR0A & (1 << RXC0)))
        ;            // Waits until has new data to be read
    aux = UDR0;      // Saves the data to be analised below
    addr = aux >> 4; // read MSB (ADDR) from serial buffer
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
    // setup_dist();
    setup_indv();

    timers();

    IR_interrupt();
    // setup_uart();

    start_timer5(MaxTime); // Reset Timer
    sei();                 // Enable global int

    OCR5A = 2800;

    state = 0;
    inv = false;
    selected_servo = 1;
}

/*********************************************
 * MAIN
 *********************************************/
int main(void) {
    // Initialization
    initialization();
    // selected_servo = 1;
    while (1) {
        // receive_Data();
        indv_control();
        // if (addr == 0x03) {
        // dist_control();
        // selected_servo++;
        //}
    }
}

// Timer interrupt
ISR(TIMER5_OVF_vect) {
    if (time_p) {
        time_p--;
    }
}

// Sensor interrupt
ISR(INT0_vect) {
    start_timer5(MaxTime); // Reset the timer
}