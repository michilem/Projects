#include <atmega32/asm/io.h>

.global main

main:


    // Configure PC0 - PC3 as output
    ldi r18, 0x0F;
    out DDRC, r18;

set_high: 
    sbis PINC, 4
    rjump set_low

    // wait for one second
    ldi r18, 0x3F
    ldi r24, 0x0D
    ldi r25, 0x03
    1: subi r18, 0x01
    sbci r24, 0x00
    sbci r25, 0x00
    brne 1b ; local label backward
    rjmp 1f ; local label forward
    1: nop

    sbi PORTC, 3
    rjump set_high

set_low:
    sbic PINC, 4
    rjump set_high

    // wait for one second
    ldi r18, 0x3F
    ldi r24, 0x0D
    ldi r25, 0x03
    1: subi r18, 0x01
    sbci r24, 0x00
    sbci r25, 0x00
    brne 1b ; local label backward
    rjmp 1f ; local label forward
    1: nop

    cbi PORTC, 3
    rjmp set_low


.end