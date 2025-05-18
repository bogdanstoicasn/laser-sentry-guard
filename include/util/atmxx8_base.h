#ifndef ATM_XX8_BASE_H
#define ATM_XX8_BASE_H

#include <stdint.h>
#include "avratlib_common.h"

/* Status register */
#define SREG    (*(volatile uint8_t *)0x5F)

#define PORTD   (*(volatile uint8_t *)0x2B)
#define PORTD0  0
#define PORTD1  1
#define PORTD2  2
#define PORTD3  3
#define PORTD4  4
#define PORTD5  5
#define PORTD6  6
#define PORTD7  7

#define DDRD    (*(volatile uint8_t *)0x2A)
#define DDD0    0
#define DDD1    1
#define DDD2    2
#define DDD3    3
#define DDD4    4
#define DDD5    5
#define DDD6    6
#define DDD7    7

#define PIND    (*(volatile uint8_t *)0x29)
#define PIND0   0
#define PIND1   1
#define PIND2   2
#define PIND3   3
#define PIND4   4
#define PIND5   5
#define PIND6   6
#define PIND7   7


#define PORTC   (*(volatile uint8_t *)0x28)
#define PORTC0  0
#define PORTC1  1
#define PORTC2  2
#define PORTC3  3
#define PORTC4  4
#define PORTC5  5
#define PORTC6  6

#define DDRC    (*(volatile uint8_t *)0x27)
#define DDC0    0
#define DDC1    1
#define DDC2    2
#define DDC3    3
#define DDC4    4
#define DDC5    5
#define DDC6    6

#define PINC    (*(volatile uint8_t *)0x26)
#define PINC0   0
#define PINC1   1
#define PINC2   2
#define PINC3   3
#define PINC4   4
#define PINC5   5
#define PINC6   6



#define PORTB   (*(volatile uint8_t *)0x25)
#define PORTB0  0
#define PORTB1  1
#define PORTB2  2
#define PORTB3  3
#define PORTB4  4
#define PORTB5  5
#define PORTB6  6
#define PORTB7  7

#define DDRB    (*(volatile uint8_t *)0x24)
#define DDB0    0
#define DDB1    1
#define DDB2    2
#define DDB3    3
#define DDB4    4
#define DDB5    5
#define DDB6    6
#define DDB7    7

#define PINB    (*(volatile uint8_t *)0x23)
#define PINB0   0
#define PINB1   1
#define PINB2   2
#define PINB3   3
#define PINB4   4
#define PINB5   5
#define PINB6   6
#define PINB7   7


/* USART macros */
#define UDR0    (*(volatile uint8_t *)0xC6)
#define UBRR0H  (*(volatile uint8_t *)0xC5)
#define UBRR0L  (*(volatile uint8_t *)0xC4)

#define UCSR0C  (*(volatile uint8_t *)0xC2)
#define UCPOL0  0
#define UCSZ00  1
#define UCSZ01  2
#define USBS0   3
#define UPM00   4
#define UPM01   5
#define UMSEL00 6
#define UMSEL01 7

#define UCSR0B  (*(volatile uint8_t *)0xC1)
#define TXB80   0
#define RXB80   1
#define UCSZ02  2
#define TXEN0   3
#define RXEN0   4
#define UDRIE0  5
#define TXCIE0  6
#define RXCIE0  7

#define UCSR0A  (*(volatile uint8_t *)0xC0)
#define MPCM0   0
#define U2X0    1
#define UPE0    2
#define DOR0    3
#define FE0     4
#define UDRE0   5
#define TXC0    6
#define RXC0    7


/* Two wire interface */
#define TWAMR   (*(volatile uint8_t *)0xBD)
#define TWAM0   1
#define TWAM1   2
#define TWAM2   3
#define TWAM3   4
#define TWAM4   5
#define TWAM5   6
#define TWAM6   7

#define TWCR    (*(volatile uint8_t *)0xBC)
#define TWIE    0
#define TWEN    2
#define TWWC    3
#define TWSTO   4
#define TWSTA   5
#define TWEA    6
#define TWINT   7

#define TWDR    (*(volatile uint8_t *)0xBB)

#define TWAR    (*(volatile uint8_t *)0xBA)
#define TWGCE   0
#define TWA0    1
#define TWA1    2
#define TWA2    3
#define TWA3    4
#define TWA4    5
#define TWA5    6
#define TWA6    7

#define TWSR    (*(volatile uint8_t *)0xB9)
#define TWPS0   0
#define TWPS1   1
#define TWS3    3
#define TWS4    4
#define TWS5    5
#define TWS6    6
#define TWS7    7

#define TWBR    (*(volatile uint8_t *)0xB8)


/* SPI related registers */
#define SPCR    (*(volatile uint8_t *)0x4C)
#define SPR0    0
#define SPR1    1
#define CPHA    2
#define CPOL    3
#define MSTR    4
#define DORD    5
#define SPE     6
#define SPIE    7

#define SPSR    (*(volatile uint8_t *)0x4D)
#define SPI2X   0
#define WCOL    6
#define SPIF    7

#define SPDR    (*(volatile uint8_t *)0x4E)
#define LSB     0
#define MSB     7


/* Timer/Counter0 related registers */
#define TCCR0A  (*(volatile uint8_t *)0x44)
#define WGM00   0
#define WGM01   1
#define COM0B0  4
#define COM0B1  5
#define COM0A0  6
#define COM0A1  7

#define TCCR0B  (*(volatile uint8_t *)0x45)
#define CS00    0
#define CS01    1
#define CS02    2
#define WGM02   3
#define FOC0B   6
#define FOC0A   7

#define TCNT0   (*(volatile uint8_t *)0x46)
#define OCR0A   (*(volatile uint8_t *)0x47)
#define OCR0B   (*(volatile uint8_t *)0x48)

#define TIMSK0  (*(volatile uint8_t *)0x6E)
#define TOIE0   0
#define OCIE0A  1
#define OCIE0B  2

#define TIFR0   (*(volatile uint8_t *)0x35)
#define TOV0    0
#define OCF0A   1
#define OCF0B   2


/* Timer/Counter1 related registers */
#define TCCR1A  (*(volatile uint8_t *)0x80)
#define WGM10   0
#define WGM11   1
#define COM1B0  4
#define COM1B1  5
#define COM1A0  6
#define COM1A1  7

#define TCCR1B  (*(volatile uint8_t *)0x81)
#define CS10    0
#define CS11    1
#define CS12    2
#define WGM12   3
#define WGM13   4
#define ICES1   6
#define ICNC1   7

#define TCCR1C  (*(volatile uint8_t *)0x82)
#define FOC1B   6
#define FOC1A   7

#define TCNT1H  (*(volatile uint8_t *)0x85)
#define TCNT1L  (*(volatile uint8_t *)0x84)

#define OCR1AH  (*(volatile uint8_t *)0x89)
#define OCR1AL  (*(volatile uint8_t *)0x88)

#define OCR1BH  (*(volatile uint8_t *)0x8B)
#define OCR1BL  (*(volatile uint8_t *)0x8A)

#define ICR1H   (*(volatile uint8_t *)0x87)
#define ICR1L   (*(volatile uint8_t *)0x86)

#define TIMSK1  (*(volatile uint8_t *)0x6F)
#define TOIE1   0
#define OCIE1A  1
#define OCIE1B  2
#define ICIE1   5

#define TIFR1   (*(volatile uint8_t *)0x36)
#define TOV1    0
#define OCF1A   1
#define OCF1B   2
#define ICF1    5


/* Power reduction register */
#define PRR     (*(volatile uint8_t *)0x64)
#define PRADC   0
#define PRUSART0 1
#define PRSPI   2
#define PRTIM1  3
#define PRTIM0  5
#define PRTIM2  6
#define PRTWI   7


/* ADC related registers */
#define DIDR1   (*(volatile uint8_t *)0x7F)
#define AIN0D   0
#define AIN1D   1

#define DIDR0   (*(volatile uint8_t *)0x7E)
#define ADC0D   0
#define ADC1D   1
#define ADC2D   2
#define ADC3D   3
#define ADC4D   4
#define ADC5D   5

#define ADMUX   (*(volatile uint8_t *)0x7C)
#define MUX0    0
#define MUX1    1
#define MUX2    2
#define MUX3    3
#define ADLAR   5
#define REFS0   6
#define REFS1   7

#define ADCSRB  (*(volatile uint8_t *)0x7B)
#define ADTS0   0
#define ADTS1   1
#define ADTS2   2
#define ACME    6

#define ADCSRA  (*(volatile uint8_t *)0x7A)
#define ADPS0   0
#define ADPS1   1
#define ADPS2   2
#define ADIE    3
#define ADIF    4
#define ADATE   5
#define ADSC    6
#define ADEN    7

#define ADCH    (*(volatile uint8_t *)0x79)
#define ADCL    (*(volatile uint8_t *)0x78)

#define ACSR    (*(volatile uint8_t *)0x50)
#define ACIS0   0
#define ACIS1   1
#define ACIC    2
#define ACIE    3
#define ACI     4
#define ACO     5
#define ACBG    6
#define ACD     7

/* Store program memory registers */
#define SPMCSR  (*(volatile uint8_t *)0x57)
#define SELFPRGEN 0
#define PGERS   1
#define PGWRT   2
#define BLBSET  3
#define RWWSRE  4
#define RWWSB   6
#define SPMIE   7


/* EEPROM related registers */
#define EEARH   (*(volatile uint8_t *)0x42)
#define EEARL   (*(volatile uint8_t *)0x41)

#define EEDR    (*(volatile uint8_t *)0x40)

#define EECR    (*(volatile uint8_t *)0x3F)
#define EERE    0
#define EEPE    1
#define EEMPE   2
#define EERIE   3
#define EEPM0   4
#define EEPM1   5


/* MCU status register */
#define MCUSR   (*(volatile uint8_t *)0x54)
#define PORF    0
#define EXTRF   1
#define BORF    2
#define WDRF    3


/* WDT control register*/
#define WDTCSR  (*(volatile uint8_t *)0x60)
#define WDP0    0
#define WDP1    1
#define WDP2    2
#define WDE     3
#define WDCE    4
#define WDP3    5
#define WDIE    6
#define WDIF    7

/* Interrupt section */
#define PCICR (*(volatile uint8_t *)0x68)
#define PCIE0 0
#define PCIE1 1
#define PCIE2 2

#define PCMSK0 (*(volatile uint8_t *)0x6B)
#define PCINT0 0
#define PCINT1 1
#define PCINT2 2
#define PCINT3 3
#define PCINT4 4
#define PCINT5 5
#define PCINT6 6
#define PCINT7 7

#endif
