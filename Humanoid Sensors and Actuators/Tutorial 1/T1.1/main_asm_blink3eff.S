#include <atmega32/asm/io.h>

.global main

main:


    // Configure PC0 as output
    ldi r18, 0x07;
    out DDRC, r18;

loop: 
    in r18, PORTC;
    andi r18, 0xFB
    ori r18, 0x01;
    out PORTC, r18;

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

    // Set pin to low
    in r18, PORTC;
    andi r18, 0xFE;
    ori r18, 0x02
    out PORTC, r18;

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

    // Set pin to low
    in r18, PORTC;
    andi r18, 0xFD;
    ori r18, 0x04
    out PORTC, r18;

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

    rjmp loop


.end