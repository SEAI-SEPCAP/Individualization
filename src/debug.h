#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef LED_BUILTIN
#undef LED_BUILTIN
#endif
#define LED_BUILTIN PINB7

#define initDebugLED() DDRB |= _BV(LED_BUILTIN);

#define turnOnDebugLED() PORTB |= _BV(LED_BUILTIN);
#define turnOffDebugLED() PORTB &= ~_BV(LED_BUILTIN);

#endif