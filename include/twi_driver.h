#ifndef TWI_DRIVER_H
#define TWI_DRIVER_H

#include "./util/atmxx8_base.h"

#define TWI_READ        1
#define TWI_WRITE       0

#define TWI_READ_ACK    1
#define TWI_READ_NACK   0

#define TWI_SCL_PIN     PORTC5
#define TWI_SDA_PIN     PORTC4

#define TWI_SLA_W(addr) ((addr << 1) | TWI_WRITE)
#define TWI_SLA_R(addr) ((addr << 1) | TWI_READ)

#define TWI_STATUS      (TWSR & ((1 << TWS3) | (1 << TWS4) | (1 << TWS5) | (1 << TWS6) | (1 << TWS7))) 

typedef enum {
	twi_freq_100k,
	twi_freq_250k,
	twi_freq_400k
} twi_freq_mode;

/*
 * 	@brief Initializes the TWI / I2C hardware
 *
 * 	Configures the TWI peripheral to operate at the specified baud rate
 * 	and enables/disables the pullup resistor.
 *
 * 	@param freq   - The freq value used for the baud rate
 * 			      check the enum
 * 	@param pullup - Enable internal pullup
 * 				  if u don't have external pullup always use it(set to 1)
 *
 * @return Error code
 */
uint8_t twi_init(twi_freq_mode freq, uint8_t pullup);

/*
 * @brief Send data up to uint8_t max size
 *
 * @param data       - The data u want to send
 * @parma size       - The size of data
 * @param addr       - The addr of the peripheral
 * @param repeat_req - Send repeat start if set to 1 (multi-master environment)
 *
 * @return Error code 
 * */
uint8_t twi_master_tx(uint8_t *data, uint8_t size, uint8_t addr, uint8_t repeat_req);

/*
 * @brief Receive data up to uint8_t max size
 *
 * @param data - Put the data here
 * @param size - Data size
 * @param addr - Slave addr
 *
 * @return Error code
 * */
uint8_t twi_master_rx(uint8_t *data, uint8_t size, uint8_t addr);

#endif
