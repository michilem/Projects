#include <atmega32/io.h>

#ifndef F_CPU
    #error "F_CPU not defined. You need to define a frequency for your MCU."
#endif


// Macros for adding a special section in ELF file for simavr.
#include <sim/avr_mcu_section.h>

/*!
 * \brief Configure the microcontroller and its frequency for simavr.
 */
AVR_MCU(F_CPU, "atmega32");

/*!
 * \brief Configure the voltage for simavr.
 */
AVR_MCU_VOLTAGES(5000, 5000, 5000);	// 5.0V at VCC, AVCC, VREF

/*!
 * \brief Config struct in special section for VCD traces in simavr.
 *
 * This small section tells simavr to generate a VCD trace dump with changes
 * to these registers. Opening it with gtkwave will show you the data being
 * pumped out into the registers.
 */
const struct avr_mmcu_vcd_trace_t _mytrace[]  _MMCU_ =
{

    { AVR_MCU_VCD_SYMBOL("DDRC"),   .what = (void*)&DDRC,   },
    { AVR_MCU_VCD_SYMBOL("DDRC0"),  .what = (void*)&DDRC,   .mask = (1 << PC0), },

    { AVR_MCU_VCD_SYMBOL("PORTC"),  .what = (void*)&PORTC,  },
    { AVR_MCU_VCD_SYMBOL("PC0"),    .what = (void*)&PORTC,  .mask = (1 << PC0), },
    { AVR_MCU_VCD_SYMBOL("PC1"),    .what = (void*)&PORTC,  .mask = (1 << PC1), },
    { AVR_MCU_VCD_SYMBOL("PC2"),    .what = (void*)&PORTC,  .mask = (1 << PC2), },

    { AVR_MCU_VCD_SYMBOL("PINC"),   .what = (void*)&PINC,   },

    { AVR_MCU_VCD_SYMBOL("PORTD"),  .what = (void*)&PORTD,  },


    { AVR_MCU_VCD_SYMBOL("ADMUX"),  .what = (void*)&ADMUX,  },
    { AVR_MCU_VCD_SYMBOL("ADCSRA"), .what = (void*)&ADCSRA, },
    { AVR_MCU_VCD_SYMBOL("SFIOR"),  .what = (void*)&SFIOR,  },

    { AVR_MCU_VCD_SYMBOL("ADEN"),   .what = (void*)&ADCSRA, .mask = (1 << ADEN), },
    { AVR_MCU_VCD_SYMBOL("ADSC"),   .what = (void*)&ADCSRA, .mask = (1 << ADSC), },


    { AVR_MCU_VCD_SYMBOL("ADCH"),   .what = (void*)&ADCH,   },
    { AVR_MCU_VCD_SYMBOL("ADCL"),   .what = (void*)&ADCL,   },


    { AVR_MCU_VCD_SYMBOL("UDR"),    .what = (void*)&UDR,  },

    { AVR_MCU_VCD_SYMBOL("UCSRA"),  .what = (void*)&UCSRA,  },
    { AVR_MCU_VCD_SYMBOL("UCSRB"),  .what = (void*)&UCSRB,  },
    { AVR_MCU_VCD_SYMBOL("UCSRC"),  .what = (void*)&UCSRC,  },

    { AVR_MCU_VCD_SYMBOL("UBRRH"),  .what = (void*)&UBRRH,  },
    { AVR_MCU_VCD_SYMBOL("UBRRL"),  .what = (void*)&UBRRL,  },

    { AVR_MCU_VCD_SYMBOL("RXC"),    .what = (void*)&UCSRA,  .mask = (1 << RXC),},
    { AVR_MCU_VCD_SYMBOL("UDRE"),   .what = (void*)&UCSRA,  .mask = (1 << UDRE),},

};


/*!
 * \brief Create a VCD trace in simavr for interrupts and pending interrupts.
 */
//AVR_MCU_VCD_IRQ(TIMER1_OVF);
//AVR_MCU_VCD_IRQ_PENDING(TIMER1_OVF);
