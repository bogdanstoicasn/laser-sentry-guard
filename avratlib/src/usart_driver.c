#include "../include/usart_driver.h"

void usart_init(usart_config *config)
{
	config->baudprescaler = (config->f_cpu / (config->baudrate * 16UL)) - 1;
	/* Load lower and upper bits of the baud rate */
	UBRR0L = (uint8_t)(config->baudprescaler);
	UBRR0H = (uint8_t)(config->baudprescaler >> 8);
	
	/* Clear the operation mode then set it up */
	UCSR0C &= ~((1 << UMSEL00) | (1 << UMSEL01));
	UCSR0C |= (config->operationmode << UMSEL00) | (config->paritymode << UPM00) 
			| (config->stopbits << USBS0) | config->datasize |  (config->edge << UCPOL0);

	UCSR0B |= config->interruptmode;

	/* Disable double speed */
	UCSR0A &= ~(1 << U2X0);

	/* Enable transmit or receive or both */
	UCSR0B |= (config->rxtx << TXEN0);
}

uint8_t  usart_receive_byte(void)
{
	while (!(UCSR0A & (1 << RXC0)));

	return UDR0;
}

void usart_transmit_byte(uint8_t ch)
{
	while (!(UCSR0A & (1 << UDRE0)));

	UDR0 = ch;
}

/* 
 * Use for critical operations
 * For everyday project is obsolete
 */
void usart_disable(void)
{
	/* Disable the transm and recv */
	UCSR0B &= ~((1 << TXEN0) | (1 << RXEN0));

	/* Clear flags*/
	UCSR0A = 0;

	/* Reset baud rate */
	UBRR0L = 0;
	UBRR0H = 0;

	/* Clear other stuff */
	UCSR0C = 0;
}
