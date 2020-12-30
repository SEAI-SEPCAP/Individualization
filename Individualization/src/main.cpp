#include <avr/io.h>
#include <util/delay.h>
#include <Arduino.h>

int main (void){  
    DDRB |= (1 << PB1);                                           //Output Port
 
    //Timer1
    TCNT1 = 0;                                                    //Set timer1 count zero
    ICR1 = 39999;                                                 //TOP count for timer1 -> FPWM = FOSC/(N*(1+TOP)) with FPWM=50 and N=64
    TCCR1A =  (1 << COM1A1) | (0 << COM1A0);                      //Non inverter PWM
    TCCR1A |=  (1 << WGM11) | (0 << WGM10);                       // Fast PWM: TOP: ICR1
    TCCR1B = (1 << WGM13) | (1 << WGM12);                         // Fast PWM: TOP: ICR1
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);            //Preesc = 8

    while (1){
        OCR1A = 20000;                                              //Pulse with 1ms (90 degrees / CCW max speed)
        _delay_ms (1000);

        OCR1A = 3000;                                               //Pulse with 1.5ms (0 degrees / STOP)
        _delay_ms (1000);

        OCR1A = 4000;                                               //Pulse with 1ms (-90 degrees / CW max speed)
        _delay_ms (1000);
    }
}