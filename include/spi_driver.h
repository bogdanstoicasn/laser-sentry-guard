#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "./util/atmxx8_base.h"

void spi_master_init();

void spi_master_transmit(uint8_t data);

uint8_t spi_master_receive();


void spi_slave_init();

void spi_slave_transmit(uint8_t data);

uint8_t spi_slave_receive();


#endif
