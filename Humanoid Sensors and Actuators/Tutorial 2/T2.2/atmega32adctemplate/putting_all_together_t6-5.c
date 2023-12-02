#include <atmega32/uart.h>
#include <atmega32/adc.h>
#include <avr/interrupt.h>

#include <util/delay.h>

const uint16_t ADC_LOW = 423;
const uint16_t ADC_HIGH = 597;

uint8_t scale_adc_value(uint16_t adc_value)
{
    if (adc_value < ADC_LOW) {
        adc_value = ADC_LOW;
    }
    if (adc_value > ADC_HIGH) {
        adc_value = ADC_HIGH;
    }

    uint16_t value = (adc_value - ADC_LOW) * 255 / (ADC_HIGH - ADC_LOW);
    return (uint8_t)value;
}

void ADC_Init()
{
    DDRA = 0x0;
    DDRC |= (1 << PC0) | (1 << PC1);

    ADMUX |= (1 << REFS0);
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);
}

void PWM_Init()
{
    DDRC |= (1 << PC0) | (1 << PC1);
    PORTC &= ~(1 << PC0);
    TCCR0 |= (1 << CS01) | (1 << CS00);
    TIMSK |= (1 << TOIE0) | (1 << OCIE0);
    OCR0 = 23;
}

uint16_t adc_result = 0; // Now 16 bit to hold the full 10-bit ADC result
uint8_t adc_result_scaled = 0;

int main(void)
{
    uart_setBaudrateReg(CALC_BAUD_VAL(62500));
    uart_setFormat();
    uart_enable();

    ADC_Init();
    PWM_Init();

    sei();

    while (1)
    {
        ADCSRA |= (1 << ADSC);
        while(ADCSRA & (1<<ADSC)); // Wait until ADC conversion is complete

    }
    return 0;
}

ISR(ADC_vect)
{
    adc_result = ADC;
    adc_result_scaled = scale_adc_value(adc_result);
    adc_result_scaled = (adc_result_scaled + 30)/6;
    uart_writeByteBlocking(adc_result_scaled);
    OCR0 = adc_result_scaled;
}

ISR(TIMER0_OVF_vect)
{
    PORTC |= (1 << PC0);
}

ISR(TIMER0_OCF_vect)
{
    PORTC &= ~(1 << PC0);
}