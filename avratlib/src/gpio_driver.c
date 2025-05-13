#include "../include/gpio_driver.h"

void gpio_setup(volatile uint8_t *ddr, uint8_t pin, uint8_t state)
{
    /* Clears the bit at that pin and then sets it how you want */
    *ddr = (*ddr & ~(1 << pin)) | ((state ? 1 : 0) << pin);  
}

void gpio_write(volatile uint8_t *port, uint8_t pin, uint8_t state)
{
    /* TODO: check for reset
     */
    *port = (*port & ~(1 << pin)) | ((state ? 1 : 0) << pin);
}

uint8_t gpio_read(volatile uint8_t *pin_r, uint8_t pin)
{
    return ((*pin_r & (1 << pin)) != 0);
}

void gpio_toggle(volatile uint8_t *port, uint8_t pin)
{
    *port ^= (1 << pin); 
}

