#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "./util/atmxx8_base.h"

/*                      <ARDUINO UNO>
 * Here we are mapping the <Arduino Uno> pins to the atmega328p:
 *
 * D2 - D13 = digital pins
 * D1, D0 = serial pins
 * D19, D18 = analog reference pins
 * A0 - A5 = analog pins
 *
 * The '_' represents the aditional functionality the pin can have.
 * (PWM, I2C etc)
 */
#define D19_SCL PORTC5
#define D18_SDA PORTC4
#define D13     PORTB5
#define D12     PORTB4
#define D11_PWM PORTB3
#define D10_PWM PORTB2
#define D9_PWM  PORTB1
#define D8      PORTB0
#define D7      PORTD7
#define D6_PWM  PORTD6
#define D5_PWM  PORTD5
#define D4      PORTD4
#define D3_PWM  PORTD3
#define D2      PORTD2
#define D1_TX   PORTD1
#define D0_RX   PORTD0

#define A0      PORTC0
#define A1      PORTC1
#define A2      PORTC2
#define A3      PORTC3
#define A4      PORTC4
#define A5      PORTC5



#define TX_LED  PORTD5
#define RX_LED  PORTD4
#define RESET   PORTC6 
#define LED_BUILTIN PORTB5

#define PORT_B 'B'
#define PORT_C 'C'
#define PORT_D 'D'

#define SET_PIN_STATE(port, pin, state) do { \
    if (state) { \
        port |= (1 << pin); \
    } else { \
        port &= ~(1 << pin); \
    } \
} while(0)

/* void gpio_init(volatile uint8_t *ddr, uint8_t pin_mask, uint8_t mode) */

/*
 * Function that changes the ddr register as required:
 *      - input
 *      - output
 * @param *ddr  - the data direction register
 * @param pin   - the port/pin we want to work with
 * @param state - input/output
 *
 * Usage: DDRD, PORTD1, OUT 
 */
void gpio_setup(volatile uint8_t *ddr, uint8_t pin, uint8_t state);

/* 
 * Function that changes the port register as required:
 *      - input/output
 *      - disable/enable pull-up resitor (if pin is set as an input)
 */
void gpio_write(volatile uint8_t *port, uint8_t pin, uint8_t state);

/*
 * Function that reads the value from the pin register
 * @param *pin_r - the pin register
 * @param pin    - the pin
 *
 * Usage: PINB, PIN7
 */
uint8_t gpio_read(volatile uint8_t *pin_r, uint8_t pin);

/*
 * Function that toggles the port register
 *
 * Usage: on output -> ON/OFF
 */
void gpio_toggle(volatile uint8_t *port, uint8_t pin);

#endif
