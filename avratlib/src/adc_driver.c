#include "../include/adc_driver.h"

void adc_init(adc_config *config)
{
	/* Power the adc */
	PRR &= ~(1 << PRADC);

	/* Set the auto trigger source if it exists */
	ADCSRB |= (config->autotrigger_source << ADTS0);

	/* Set the prescaler, interrupt if it exists and auto triggering conversion */
	ADCSRA |= (config->prescaler << ADPS0) | (config->interruptmode << ADIE) | (config->autotrigger << ADATE);

	/* Set the reference voltage and pin */
	ADMUX |= (config->refvoltage << REFS0) | (config->pin << MUX0);

	if (config->pin <= 5)
		DIDR0 |= (1 << config->pin);
	
	ADCSRA |= (1 << ADEN);
}

void adc_disable(void)
{
	/* 
	 * Disable ADC in the control register 
	 * Disclaimer: it stops the current conversion
	 */
	ADCSRA &= ~(1 << ADEN);

	/* Shutdown the ADC from the power reduction register */
	PRR |= (1 << PRADC);
}

void adc_reset(adc_config *config)
{
	ADCSRA &= ~(1 << ADEN);
	PRR &= ~(1 << PRADC);

	ADMUX = 0x00;
	ADCSRA = 0x00;
	ADCSRB = 0x00;
	DIDR0 = 0x00;

	*config =(adc_config){0};
}

uint16_t adc_read(void)
{
	/* Start the conversion */
	ADCSRA |= (1 << ADSC);
	
	while (ADCSRA & (1 << ADSC));

	/* 
	 * First we get the low value!
	 * If high is checked first the both low and high regs will be reset
	 */
	uint8_t low = ADCL;
	uint8_t high = ADCH;

	uint16_t adc_val = (high << 8) | low;

	return adc_val;
}
