#include <util/delay.h>

#define USER_ISR
#include "avratlib.h"

#define TRIG_R PIND2
#define ECHO_R PIND3

#define TRIG_L PIND6
#define ECHO_L PIND7

#define DEBUG 1

#define MAX_DETECT_DIST(adc_val) (20 + ((adc_val) * 60) / 1023)

/*
 * Structure that holds the movement related data
 * such as direction, aggression and if motion was detected*/
typedef struct {
	uint8_t left;
	uint8_t right;
	uint8_t motion;
	uint8_t flag;
	uint16_t aggression;
}movement;

movement my_data = {0};

/* Debug section */
void init_usart_func(void)
{
    usart_config testusart = {
        .f_cpu = F_CPU,
        .baudrate = BAUDRATE,
        .baudprescaler = 0,
        .datasize = DATA_SIZE_8BIT,
        .interruptmode = USART_INT_DISABLED,
        .operationmode = ASYNCH_MODE,
        .rxtx = RXTX_ENABLE,
        .paritymode = DISABLED_MODE,
        .stopbits = DEFAULT_STOP,
        .edge = RISING_EDGE
    };

    usart_init(&testusart);

    _delay_ms(200);
}

void usart_transmit_number(uint16_t number)
{
    char digits[5];
    uint8_t i = 0;

    if (number == 0) {
        usart_transmit_byte('0');
        return;
    }

    // Extract digits from the number (in reverse order)
    while (number > 0) {
        digits[i++] = (number % 10) + '0'; // Convert digit to ASCII
        number /= 10;
    }

    // Transmit digits in correct order
    while (i > 0) {
        usart_transmit_byte(digits[--i]);
    }
}

/* Sensor Section */
void timer0_init(void)
{
    // Set Timer0 in normal mode (no PWM, no CTC, no Fast PWM)
    TCCR0A = 0;
    TCCR0B |= (1 << CS00) | (1 << CS02);  // Prescaler 1024
    TCNT0 = 0;  // Clear timer count
}

/* Init the 3 sensors(2 distance and 1 movement) */
void sensors_init()
{
	// sensor 1 section (right)
	DDRD |= (1 << TRIG_R); 
	DDRD &= ~(1 << ECHO_R);

	// sensor 2 section (left)
	DDRD |= (1 << TRIG_L);
	DDRD &= ~(1 << ECHO_L);

	// motion sensor(little bit more complicated cuz we have to use interrupts)
	DDRB &= ~(1 << PORTB0);

	PCICR |= (1 << PCIE0);
	PCMSK0 |= (1 << PCINT0);
}

/* Trigger the distance sensors */
void distance_sensor_trigger(uint8_t trigger_pin)
{
    PORTD |= (1 << trigger_pin);  // Set trigger high
    _delay_us(20);                // Wait 20 us
    PORTD &= ~(1 << trigger_pin); // Set trigger low
}

/* 
 * Read the distance sensors
 * Used the highest prescaler(1024) so we don't need to transform the reading
 * using the formula
 */
uint16_t distance_sensor_read(uint8_t trigger_pin, uint8_t echo_pin)
{
    uint16_t duration = 0;

    // Trigger the distance sensor
    distance_sensor_trigger(trigger_pin);

    // Wait for the echo to go high
    while (!(PIND & (1 << echo_pin))); // Wait for rising edge

    TCNT0 = 0; // Start timer when the echo rises

    // Wait for the echo to go low
    while (PIND & (1 << echo_pin));   // Wait for falling edge

    // Store the timer count
    duration = TCNT0;

	// because the time is already measured in ms, the raw value of tcnt represents the actual value
    return duration; // Convert time to cm
}

/* Wrapper to read both distance  sensors */
void read_direction(movement *mystruct)
{
	if (mystruct->motion == 0) return;
	mystruct->right = distance_sensor_read(TRIG_R, ECHO_R);

#if DEBUG
	usart_transmit_number(mystruct->right & 0xFF);
	usart_transmit_byte('R');
	usart_transmit_byte('\n');
	usart_transmit_byte('\r');
#endif
	
	_delay_ms(100);

	mystruct->left = distance_sensor_read(TRIG_L, ECHO_L);

#if DEBUG
	usart_transmit_number(mystruct->left & 0xFF);
	usart_transmit_byte('L');
	usart_transmit_byte('\n');
	usart_transmit_byte('\r');
#endif
} 

/* Potentiometer section */
void potentiometer_init()
{
	adc_config my_adc = {
		.pin = ADC0,
		.prescaler = PRESC_VAL_32,
		.interruptmode = 0,
		.autotrigger_source = 0,
		.refvoltage = AVCC_EXT_CAPACITOR_AREF_PIN,
		.autotrigger = 0,
	};
	adc_init(&my_adc);
}

/* Interrupts */

/* Watchdog interrupt 
 * Resets the motion detection 
 */
void __vector_6(void)
{
	my_data.motion = 0; // reset distance sensors read capacity(no read)
	my_data.flag = 1; // this flag triggers an AnalogRead
#if DEBUG
	usart_transmit_byte('0');
#endif

}

void __vector_3(void)
{
	if (PINB & (1 << PINB0)) {
		my_data.motion = 1;
		usart_transmit_byte('D');
	}
}

int main(void)
{
	wdt_enable(WDT_PRESCALER_8S, WDT_MODE_INTERRUPT);

	sensors_init();
	potentiometer_init();
    init_usart_func();
    _delay_ms(1000);

    timer0_init(); // Initialize Timer0
	_delay_ms(1000);

    while (1) {
		read_direction(&my_data);
		if (my_data.flag) {
			my_data.flag = 0;
			my_data.aggression = adc_read();
			_delay_ms(200);
			usart_transmit_number(my_data.aggression);
		}
		_delay_ms(1000);
		
    }
}

