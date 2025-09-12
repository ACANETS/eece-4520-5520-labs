; comment starts with ';'
; Assembly Code

; If you are using assembly with avr-libc headers, you must define __SFR_OFFSET as 0 
; before including avr/io.h to make the header use the I/O addresses, 
; which are required for IN and OUT instructions. 
#define __SFR_OFFSET 0x00

; include avr/io.h to access definitions for PORTB, SREG, etc
#include "avr/io.h"

; Adafruit Metro board uses ATmega328P
; the built-in LED is connected to PB5
#define LED_PIN 5


.global start
.global led
.global shift_add_in_assembly

.section .text    ; Defines a code section

start:
  SBI DDRB,LED_PIN
  RET

led:
  CPI R24, 0x00
  BREQ  ledOFF
  SBI PORTB, LED_PIN
  RET

ledOFF:
  CBI PORTB, LED_PIN
  RET

; function argument convention :-
; Parameters are passed via R25:8 (R25 to R8).
; A minimum of two registers (16 bits) are used for each parameter passed.
; Parameters are passed left to right.
; A minimum of two registers (16 bits) are used for each parameter passed.
; Return values are placed in registers beginning at R25.
;   An 8-bit value gets returned in R24.
;   An 16-bit value gets returned in R25:24.
;   An 32-bit value gets returned in R25:22.
;   An 64-bit value gets returned in R25:18.
;
; "shift_add_in_assembly" function takes two arguments : i and j
; s = (i >> 1) + j
;
; the first argument i is placed in R25 and R24, where R24 holds the least signficant 8 bits
; the second argument j is placed in R23 and R22, where R22 holds the least signficant 8 bits
; the return value is placed in R24

shift_add_in_assembly:
  LSR R24
  ADD R24, R22
  RET
