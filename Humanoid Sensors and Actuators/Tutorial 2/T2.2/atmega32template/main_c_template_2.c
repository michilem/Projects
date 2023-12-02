// Get register definitions with auto complete in Qt Creator
#include <atmega32/io.h>

#include <atmega32/uart.h>

// For delay functions: F_CPU has to be defined
#include <util/delay.h>

void ADC_Init() {

    // Setting all pins of port A as input
    DDRA = 0x0;
    
    // AVCC pin is VCC 5V
    ADMUX |= (1 << REFS0) | (1 << ADLAR);

    // Enabling ADC
    ADCSRA |= (1 << ADATE) | (1 << ADEN) | (1 << ADSC);
    
}

void adc_readBlocking(uint8_t* b, uint8_t channel) {
    
    // Read value
    ADMUX |= (channel & 0x07);     // we ignore the different channels with possible gains
    while(!(ADCSRA & (1<<ADSC)));
    *b = (uint8_t) ADCH;

}

int main (void)
{

    uart_setBaudrateReg(CALC_BAUD_VAL(62500));
    uart_setFormat();
    uart_enable();

    // Should never be reached
    ADC_Init();

    uint8_t b;
    while (1) {
        adc_readBlocking(&b, 2);
        uart_writeByteBlocking(b);
        _delay_ms(10);
    }
    return 0;
}