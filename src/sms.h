#ifndef __SMS_H__
#define __SMS_H__

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

#include "queue.h"

/************* Address + Message Type ******************/
// To send
#define INTERFACE 0x41 // Interface + IR new capsule
#define EM 0x00        // Broadcast - EM
// To receive
#define INDV 0x02    // Individualization + operation
#define CAPSULE 0x01 // Distribution  + New capsule

// Data
#define EM_ON 0x00       // EM Stop on
#define EM_OFF 0x01      // EM Stop off
#define New_Capsule 0xFF // IR detects new capsule

#define START 0x01 // Start operations
#define STOP 0x00  // Stop operations

uint8_t state;
float dc; // Duty-cycle
bool inv;

bool emergency = false;
bool operation = false;

uint8_t r_data;
uint8_t msgType;
uint8_t addr;
uint8_t header;
uint8_t data;
uint8_t msgAddr;
uint8_t em_message;

bool isData(void) { return (UCSR0A & (1 << RXC0)); }

void receive_data(void) {

    if (isData()) {
        header = UDR0; // Saves the data to be analised below
        while (!isData())
            ; // Waits until has new data to be read
        data = UDR0;

        msgAddr = header & (0xF0); // Read the received addreess (First 4 bits)
        msgType = header & (0x0F); // Read the message type (Last 4 bits)

        if (CAPSULE == header) {

            insert_code(data); // Saves the morot code in the queue
        } else if (INDV == header) {
            while (!isData())
                ; // Waits until has new data to be read
            data = UDR0;
            if (START == data) {
                operation = true;
            } else {
                operation = false;
            }
        } else
            return;
    } else {
        return;
    }
}

void send_Data(uint8_t addr, uint8_t message) {

    while (!(UCSR0A & (1 << UDRE0)))
        ;        // Wait until the buffer is empty
    UDR0 = addr; // Copy to UDR0 the correspondent data
    while (!(UCSR0A & (1 << UDRE0)))
        ; // Wait until the buffer is empty
    UDR0 = message;
}

#endif
