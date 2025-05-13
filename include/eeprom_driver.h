#ifndef EEPROM_DRIVER_H
#define EEPROM_DRIVER_H

#include "./util/atmxx8_base.h"
#include "./util/atmxx8_asm.h"

/* Function that writes to eeprom 
 * @param addr - the addres you want to write to
 * @param data - the data
 * 
 * Caution: -stops interrupts
 */
void eeprom_write(uint16_t addr, uint8_t data);

/* Function that reads from specific addres from eeprom
 * @param addr - the address you want to read
 *
 * @return the data
 */
uint8_t eeprom_read(uint16_t addr);

#endif
