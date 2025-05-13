#include <util/delay.h>

#define USER_ISR
#include "avratlib.h"

#define TRIG_R PIND2
#define ECHO_R PIND3

#define TRIG_L PIND6
#define ECHO_L PIND7

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

void timer0_init(void)
{
    // Set Timer0 in normal mode (no PWM, no CTC, no Fast PWM)
    TCCR0A = 0;
    TCCR0B |= (1 << CS00) | (1 << CS02);  // Prescaler 1024
    TCNT0 = 0;  // Clear timer count
}

void distance_sensor_init()
{
	// sensor 1 section (right)
	DDRD |= (1 << TRIG_R); 
	DDRD &= ~(1 << ECHO_R);

	// sensor 2 section (left)
	DDRD |= (1 << TRIG_L);
	DDRD &= ~(1 << ECHO_L);
}
void distance_sensor_trigger(uint8_t trigger_pin)
{
    PORTD |= (1 << trigger_pin);  // Set trigger high
    _delay_us(20);                // Wait 20 us
    PORTD &= ~(1 << trigger_pin); // Set trigger low
}

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


int main(void)
{
	distance_sensor_init();
    init_usart_func();
    usart_transmit_byte('y');
    _delay_ms(4000);

    uint16_t distance = 0;
    timer0_init(); // Initialize Timer0

    while (1) {
        distance = distance_sensor_read(TRIG_R, ECHO_R);  // Read the distance
        usart_transmit_number(distance & 0xFF);  // Send distance to USART
		usart_transmit_byte('R');
        usart_transmit_byte('\n');  // Newline
        usart_transmit_byte('\r');  // Carriage return

        /*_delay_ms(100);  // Delay before the next measurement
		distance = distance_sensor_read(TRIG_L, ECHO_L);
		usart_transmit_number(distance & 0xFF);
		usart_transmit_byte('L');
		usart_transmit_byte('\n');
		usart_transmit_byte('\r');*/
		_delay_ms(1000);
		
    }
}

