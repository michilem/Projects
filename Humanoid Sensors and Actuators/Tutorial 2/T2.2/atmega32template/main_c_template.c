// Get register definitions with auto complete in Qt Creator
#include <atmega32/io.h>
#include <atmega32/uart.h>
#include <atmega32/adc.h>
#include <avr/interrupt.h>

// For delay functions: F_CPU has to be defined
#include <util/delay.h>

int main (void)
{
    
    // adc_setStdConfig();
    // adc_enable();

    //initialization
    DDRC |= (1 << PC0) | (1 << PC1);     // pin C0 is output
    PORTC &= ~(1 << PC0) ; // turn off at start
    TCCR0 |= (1 << CS02 )| (1 << CS00); // prescaler 1024 , enable PWM, Phase Correct
    TIMSK |= (1 << TOIE0) | (1 << OCIE0); // Enable overflow and compare match interrupts

    sei(); // enable global interrupt

    OCR0 = 127; //duty cycle

    while (1) {
    }

    // Should never be reached
    return 0;
}

ISR(TIMER0_OVF_vect) {
  PORTC |= (1 << PC0); // Set PC0
}

ISR(TIMER0_OCF_vect) {
  PORTC &= ~(1 << PC0); // Unset PC0
}