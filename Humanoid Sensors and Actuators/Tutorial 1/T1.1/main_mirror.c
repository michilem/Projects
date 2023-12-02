// Get register definitions with auto complete in Qt Creator
#include <atmega32/io.h>

// For delay functions: F_CPU has to be defined
#define F_CPU 1000000UL
#include <util/delay.h>


int main (void)
{
    DDRC = 0x0F;
    while(1){
        if(PINC & (1 << PC4))
        {
            _delay_ms(1000);
            PORTC |= (1 << PC3)
        }
        else
        {
            _delay_ms(1000);
            PORTC &= ~(1 << PC3)
        }
    }
    
    // Should never be reached
    return 0;
}
