// Get register definitions with auto complete in Qt Creator
#include <atmega32/io.h>

// For delay functions: F_CPU has to be defined
#define F_CPU 1000000UL
#include <util/delay.h>


int main (void)
{
    DDRC = 0x00;
    for( ; ; ) {
        PORTC &= ~(1 << PC2);
        PORTC |= (1 << PC0);

        _delay_ms(1000);

        PORTC &= ~(1 << PC0);
        PORTC |= (1 << PC1);

        _delay_ms(1000);

        PORTC &= ~(1 << PC1);
        PORTC |= (1 << PC2);

        _delay_ms(1000);
    }
    // Should never be reached
    return 0;
}
