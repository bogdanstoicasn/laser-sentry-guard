#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include "./util/atmxx8_base.h"

#define BAUDRATE       9600
#define F_CPU          16000000UL
#define BAUD_PRESCALER ((F_CPU / (BAUDRATE * 16UL)) - 1)

/*              General macros               */

/* Parity for usart init */
#define EVEN_PARITY    2
#define ODD_PARITY     3
#define DEFAULT_PARITY 0

/* Number of stop bits for usart init */
#define DEFAULT_STOP   0
#define COMPLEX_STOP   1

/* Clock Polarity */
#define RISING_EDGE 0
#define FALLING_EDGE 1

#define USART_DISABLE_TRANSMIT() (UCSR0B &= ~(1 << TXEN0))
#define USART_DISABLE_RECEIVE()  (UCSR0B &= ~(1 << RXEN0))

/* ENUMS used to set fields in the usart_config struct */

/* Used to set datasize */
typedef enum {
	DATA_SIZE_5BIT = 0,
	DATA_SIZE_6BIT = (1 << UCSZ00),
	DATA_SIZE_7BIT = (1 << UCSZ01),
	DATA_SIZE_8BIT = (1 << UCSZ00) | (1 << UCSZ01)
} data_size;

/* Used to set the interruptmode */
typedef enum {
	USART_INT_DISABLED = 0,        // All interrupts disabled
    USART_INT_RXCIE = (1 << RXCIE0), // RX Complete Interrupt Enabled
    USART_INT_TXCIE = (1 << TXCIE0), // TX Complete Interrupt Enabled
    USART_INT_UDRIE = (1 << UDRIE0), // USART Data Register Empty Interrupt Enabled
    USART_INT_RXCIE_TXCIE = (1 << RXCIE0) | (1 << TXCIE0), // RX and TX Complete Interrupts Enabled
    USART_INT_RXCIE_UDRIE = (1 << RXCIE0) | (1 << UDRIE0), // RX Complete and Data Register Empty Interrupts Enabled
    USART_INT_TXCIE_UDRIE = (1 << TXCIE0) | (1 << UDRIE0), // TX Complete and Data Register Empty Interrupts Enabled
    USART_INT_RXCIE_TXCIE_UDRIE = (1 << RXCIE0) | (1 << TXCIE0) | (1 << UDRIE0)
} interrupt_mode;

/* Used to set rxtx */
typedef enum {
	RXTX_DISABLED = 0,
	TX_ENABLE = 1,
	RX_ENABLE = 2,
	RXTX_ENABLE = 3
} rxtx_mode;

/* Used to set operationmode */
typedef enum {
	ASYNCH_MODE = 0,
	SYNCH_MODE = 1,
	MSPIM_MODE = 3
} operation_mode;

/* Used to set paritymode */
typedef enum {
	DISABLED_MODE = 0,
	EVEN_MODE = 2,
	ODD_MODE = 3
} parity_mode;

/* Structure used to config the usart */
typedef struct {
	uint32_t f_cpu;
	uint16_t baudrate;
	uint16_t baudprescaler;
	uint8_t datasize;
	uint8_t interruptmode;
	uint8_t operationmode: 2;
	uint8_t rxtx: 2;
	uint8_t paritymode: 2;
	uint8_t stopbits: 1;
	/* Only used in synch mode else write to 0 */
	uint8_t edge: 1;
} usart_config;

/* Function that initializes the usart
 *
 * @param parity - add parity bits for message correction
 * @param stop - number of stop bits
 */
void usart_init(usart_config *config);

/* Receive a byte
 * 
 * @return byte received
 * TODO: check for other flags in UCSR0A
 */
uint8_t usart_receive_byte(void);

/* Transmite byte 
 * 
 * @param ch - byte to be sent
 *
 * TODO: maybe check for errors in ch
 */
void  usart_transmit_byte(uint8_t ch);

/* Usart disable 
 *
 * Usage: just for critical op. 
 * */
void usart_disable(void);


#endif

