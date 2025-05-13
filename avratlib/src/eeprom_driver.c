#include "../include/eeprom_driver.h"

void eeprom_write(uint16_t addr, uint8_t data)
{
	DISABLE_INTERRUPTS();

	/* Wait for previous read/write */
	while (EECR & (1 << EEPE));

	/* Check for other operation that uses the charge pump 
	 * flash mem writing or other
	 */
	while (SPMCSR & (1 << SELFPRGEN));

	/* Set address */
	EEARH = (addr >> 8) & 0x01;
	EEARL = (uint8_t)addr;

	/* Put the data */
	EEDR = data;

	/* Prepare for the read and start the process */
	EECR |= (1 << EEMPE);

	EECR |= (1 << EEPE);

	/* Enable the interrupts again*/
	ENABLE_INTERRUPTS();
}

uint8_t eeprom_read(uint16_t addr)
{
	DISABLE_INTERRUPTS();

	/* Wait for previous read/write */
	while (EECR & (1 << EEPE));

	/*Set address*/
	EEARH = (addr >> 8) & 0x01;
	EEARL = (uint8_t)addr;

	/* Read */
	EECR |= (1 << EERE);

	/* Enable interrupts*/
	ENABLE_INTERRUPTS();

	return EEDR;
}
