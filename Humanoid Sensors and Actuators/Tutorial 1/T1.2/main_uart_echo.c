// Get register definitions with auto complete in Qt Creator
#include <atmega32/io.h>

// For delay functions: F_CPU has to be defined
#include <util/delay.h>

#define F_CPU 1000000UL
#define BAUD 62500
#define UBBR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

void USART_Init(unsigned int ubbr) {

    /* Set baud rate */
    UBRRH = (unsigned char) (ubbr>>8);
    UBRRL = (unsigned char) ubbr;

    /* Enable receiver and transmitter */
    UCSRB = (1<<RXEN)|(1<<TXEN);

    // Set frame format: 8data, 1stop bit. Asynchronous
    UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);

}

void InitDebugLEDS() {
    // Set debug leds in PORTC
    DDRC |= (1 << PC0);
    DDRC |= (1 << PC1);
    DDRC |= (1 << PC2);
}

void USART_TransmitByte( unsigned char data )
{
    while ( !( UCSRA & (1<<UDRE)) )
        ;
    UDR = data;

}

unsigned char USART_receive(void) {
    // PORTC |= (1 << PC0);   led on for debugging
    while (!(UCSRA & (1 << RXC))); // Wait for data to be received
    PORTC &= ~(1 << PC0);
    return UDR; // Get and return received data from buffer
}

int main (void)
{

    InitDebugLEDS();
    USART_Init(UBBR_VALUE);
    while (1) {
        unsigned char byte = USART_receive();
        USART_TransmitByte(byte);
    }

    return 0;
}
