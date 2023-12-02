#include <atmega32/uart.h>
#include <atmega32/adc.h>
#include <avr/interrupt.h>

#include <util/delay.h>

const uint8_t SLOPE = 31;
const uint16_t OFFSET = 7 * 255;

uint8_t scale_adc_value(uint8_t adc_value)
{
    uint16_t value = adc_value * SLOPE + OFFSET;
    uint8_t value_uint = (uint8_t) (value / 255);
    return value_uint;
}

void ADC_Init()
{

    // Setting all pins of port A as input
    DDRA = 0x0;

    // Seting all pins 0 and 1 of port C as output
    DDRC |= (1 << PC0) | (1 << PC1);

    // AVCC pin is VCC 5V
    ADMUX |= (1 << REFS0) | (1 << ADLAR);

    // Enabling ADC and setting prescaler
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADIE);
}

void PWM_Init()
{

    // initialization
    DDRC |= (1 << PC0) | (1 << PC1); // pin C0 is output
    PORTC &= ~(1 << PC0);            // turn LED off at start

    // TCCR0 |= (1 << CS01); // 8
    // TCCR0 |= (1 << CS00); // No prescaling
    TCCR0 |= (1 << CS01) | (1 << CS00); // 64
    // TCCR0 |= (1 << CS02) | (1 << CS00); // 1024
    TIMSK |= (1 << TOIE0) | (1 << OCIE0); // Enable overflow and compare match interrupts
    OCR0 = 23;                           // Initial value for duty cycle
}

uint8_t adc_result = 0;
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
    }
    return 0;
}

ISR(ADC_vect)
{

    adc_result = ADCH;
    adc_result_scaled = scale_adc_value(adc_result);
    OCR0 = adc_result_scaled;
    //OCR0 = ADCH;
}

ISR(TIMER0_OVF_vect)
{
    PORTC |= (1 << PC0); // Set PC0
}

ISR(TIMER0_OCF_vect)
{
    PORTC &= ~(1 << PC0); // Unset PC0
}