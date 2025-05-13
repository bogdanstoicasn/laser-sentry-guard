#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "./util/atmxx8_base.h"

/* Use for prescaler value */
#define PRESC_VAL                          0
#define PRESC_VAL_2                        1
#define PRESC_VAL_4                        2
#define PRESC_VAL_8                        3
#define PRESC_VAL_16                       4
#define PRESC_VAL_32                       5
#define PRESC_VAL_64                       6
#define PRESC_VAL_128                      7

/* Use for reference voltage
 * Arduino Uno uses the value 1 (the second define)
 */
#define AREF_INT_VREF_OFF                  0
#define AVCC_EXT_CAPACITOR_AREF_PIN        1
#define INT_1_V_1_EXT_CAPACITOR_AREF_PIN   3

/* ADC Pin to choose from.
 * Values from the atmega328p datasheet
 * Arduino Uno has ADC from 0 to 5 (use the values from 0 to 5 or GND or 1.1V)
 */
#define ADC0                               0
#define ADC1                               1
#define ADC2                               2
#define ADC3                               3
#define ADC4                               4
#define ADC5                               5
#define ADC6                               6
#define ADC7                               7
#define ADC8                               8
#define V1_1                               14
#define GND                                15

/* Structure used to config the adc */
typedef struct {
	uint8_t pin: 4;
	uint8_t prescaler: 3;
	uint8_t interruptmode: 1;

	uint8_t autotrigger_source: 3;
	uint8_t refvoltage: 2;
	uint8_t autotrigger: 1;
} adc_config;

/* resolution = (Vref_high - Vref_low)/2^b 
 * resolution = voltage between 2 adjacent quantization levels
 * Ex: 5 volts range, 2 bit representation, 1.25 resolution
 *
 * Ex: prescaler  = 128 => ADC_clk = 16MHz/128 = 125KHz
 * 	   conversion ~= 13 cycles => sampling_rate = 125KHz/13 = 9600Hz
 * */

void adc_init(adc_config *config);

/* 
 * Function that disables the adc
 */
void adc_disable(void);

/*
 * Hard reset the adc 
 * Clears the config 
 *
 * @param config - pointer to adc config struct
 */
void adc_reset(adc_config *config);

/* Read the adc from a pin
 * Use adc_init first
 * Atmega328p can use only 1 adc pin at a time 
 *
 * @return the value we read
 */
uint16_t adc_read(void);

#endif
