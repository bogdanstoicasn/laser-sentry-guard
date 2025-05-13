#include "../include/spi_driver.h"

/* Master section */
void spi_master_init()
{
	/* Set output to MOSI, SCK, SS */
	DDRB |= (1 << PINB3) | (1 << PINB5) | (1 << PINB2);
	
	/* Set MISO input */
	DDRB &= ~(1 << PINB4);

	/* Set SS high */
	PORTB |= (1 << PINB2);

	SPCR |= (1 << SPE) | (1 << MSTR);
}

void spi_master_transmit(uint8_t data)
{
	uint8_t flush;

	SPDR = data;

	while (!(SPSR & (1 << SPIF)));

	/* SPIF is cleared by reading SPSR and accesing SPDR */
	flush = SPDR;

	(void)flush;
}

uint8_t spi_master_receive()
{
	SPDR = 0xFF;

	while (!(SPSR & (1 << SPIF)));

	return SPDR;
}


/* Slave section */
void spi_slave_init()
{
	/* Set MOSI, SCK, SS as input */
	DDRB &= ~((1 << PINB3) | (1 << PINB5) | (1 << PINB2));

	/* MISO output */
	DDRB |= (1 << PINB4);

	/* Enable SPI */
	SPCR |= (1 << SPE);
}

/* Same function as master transmit */
void spi_slave_transmit(uint8_t data)
{
	uint8_t flush;

	SPDR = data;

	while (!(SPSR & (1 << SPIF)));

	flush = SPDR;

	(void)flush;
}

uint8_t spi_slave_receive()
{
	while (!(SPSR & (1 << SPIF)));

	return (SPDR);
}
