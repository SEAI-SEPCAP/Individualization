#ifndef __SMS_H__
#define __SMS_H__

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/delay.h>

#include "queue.h"

#define SMS_ADDRESS_MASK 0xF0
#define SMS_MSGTYPE_MASK 0x0F

#define SMS_ADDRESS__BROADCAST 0
#define SMS_ADDRESS__INDIVIDUALIZATION 1
#define SMS_ADDRESS__CLASSIFICATION 2
#define SMS_ADDRESS__DISTRIBUTION 3
#define SMS_ADDRESS__INTERFACE 4

#define SMS_MSGTYPE__EMERGENCYSTOP 0
#define SMS_MSGTYPE__NEWCAPSULE 1
#define SMS_MSGTYPE__STARTSTOP 2

#define SMS_MSGTYPE__EMERGENCYSTOP__EMERGENCY 0
#define SMS_MSGTYPE__EMERGENCYSTOP__RESUME 1

#define SMS_MSGTYPE__NEWCAPSULE__IRSENSOR 0xFF

#define SMS_MSGTYPE__STARTSTOP__STOP 0
#define SMS_MSGTYPE__STARTSTOP__START 1

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

uint8_t getMsgAdress(uint8_t headerByte) {
    return (headerByte & SMS_ADDRESS_MASK) >> 4;
}

uint8_t getMsgType(uint8_t headerByte) {
    return (headerByte & SMS_MSGTYPE_MASK);
}

bool isMsgAddressedAtUs(uint8_t msgAddr) {
    if (msgAddr == SMS_ADDRESS__BROADCAST ||
        msgAddr == SMS_ADDRESS__INDIVIDUALIZATION ||
        msgAddr == SMS_ADDRESS__DISTRIBUTION) {

        return true;
    }

    return false;
}

bool isData(void) { return (UCSR0A & (1 << RXC0)); }

void receive_data(void) {

    if (isData()) {
        header = UDR0; // Saves the data to be analised below
        msgAddr = getMsgAdress(header);
        msgType = getMsgType(header);

        while (!isData())
            ; // Waits until has new data to be read
        data = UDR0;

        if (!isMsgAddressedAtUs(msgAddr))
            return;

        if (msgType == SMS_MSGTYPE__NEWCAPSULE) {
            insert_code(data); // Saves the morot code in the queue
        } else if (msgType == SMS_MSGTYPE__STARTSTOP) {
            if (data == SMS_MSGTYPE__STARTSTOP__START) {
                operation = true;
            } else {
                operation = false;
            }
        }
    } else {
        return;
    }
}

bool isClearToSend() { return UCSR0A & (1 << UDRE0); }

void send_Data(uint8_t header, uint8_t data) {

    while (!isClearToSend())
        ;          // Wait until the buffer is empty
    UDR0 = header; // Copy to UDR0 the correspondent data
    while (!isClearToSend())
        ; // Wait until the buffer is empty
    UDR0 = data;
}

uint8_t generateMsgHeader(uint8_t address, uint8_t msg_type) {
    return (address << 4) | msg_type;
}

void sendNewCapsuleDetection() {
    uint8_t header =
        generateMsgHeader(SMS_ADDRESS__INTERFACE, SMS_MSGTYPE__NEWCAPSULE);
    uint8_t data = SMS_MSGTYPE__NEWCAPSULE__IRSENSOR;
    send_Data(header, data);
}

void sendEmergency_Emergency() {
    uint8_t header =
        generateMsgHeader(SMS_ADDRESS__INTERFACE, SMS_MSGTYPE__EMERGENCYSTOP);
    uint8_t data = SMS_MSGTYPE__EMERGENCYSTOP__EMERGENCY;
    send_Data(header, data);
}

void sendEmergency_Resume() {
    uint8_t header =
        generateMsgHeader(SMS_ADDRESS__BROADCAST, SMS_MSGTYPE__EMERGENCYSTOP);
    uint8_t data = SMS_MSGTYPE__EMERGENCYSTOP__RESUME;
    send_Data(header, data);
}

#endif
