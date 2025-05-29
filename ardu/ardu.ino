#define DEBUG 1  // Set to 0 to disable debug prints

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <Servo.h>

#define BAUDRATE       9600
#define DELAY_DEBOUNCE 75

#define RADAR_STEP 2
#define RADAR_DELAY 200

#define RADAR_MIN_ANGLE 83
#define RADAR_MAX_ANGLE 100

#define SENTRY_LEFT_ANGLE 80
#define SENTRY_RIGHT_ANGLE 100
#define SENTRY_CENTER_ANGLE 90

#define TRIG_R 2    // Right sensor trigger pin
#define ECHO_R 3    // Right sensor echo pin

#define TRIG_L 6    // Left sensor trigger pin
#define ECHO_L 7    // Left sensor echo pin

#define PIR_PIN 8   // Motion sensor input pin (poll selectively)
#define BUTTON_PIN 12

#define ENABLE_INTERRUPTS() __asm__ __volatile__ ("sei" ::: "memory")
#define DISABLE_INTERRUPTS() __asm__ __volatile__("cli" ::: "memory")
#define RETURN_INTERRUPT() __asm__ __volatile__ ("reti"::: "memory")

#define MODES_NUM 3
#define MODE 'S'  // or 'R' or 'P'

Servo base_servo;               // Servo object
const uint8_t SERVO_PIN = 9;   // Servo control pin (choose a PWM pin)

uint8_t modes_num[MODES_NUM];


/* WDT SECTION */
typedef enum {
  WDT_MODE_INTERRUPT = 0,
  WDT_MODE_RESET = 1,
  WDT_MODE_INTERRUPT_RESET = 2
} wdt_mode;

typedef enum {
  WDT_PRESCALER_16MS = 0,
  WDT_PRESCALER_32MS = (1 << WDP0),
  WDT_PRESCALER_64MS = (1 << WDP1),
  WDT_PRESCALER_125MS = (1 << WDP0) | (1 << WDP1),    // 0.125s
  WDT_PRESCALER_250MS = (1 << WDP2),                  // 0.25s
  WDT_PRESCALER_500MS = (1 << WDP0) | (1 << WDP2),    // 0.5s
  WDT_PRESCALER_1S = (1 << WDP1) | (1 << WDP2),       // 1s
  WDT_PRESCALER_2S = (1 << WDP0) | (1 << WDP1) | (1 << WDP2), // 2s
  WDT_PRESCALER_4S = (1 << WDP3),                      // 4s
  WDT_PRESCALER_8S = (1 << WDP0) | (1 << WDP3)        // 8s
} wdt_prescaler;

void manual_wdt_enable(wdt_prescaler prescaler, wdt_mode mode) {
  uint8_t wdt_config = prescaler;

  if (mode == WDT_MODE_INTERRUPT) {
    wdt_config |= (1 << WDIE);
  } else if (mode == WDT_MODE_RESET) {
    wdt_config |= (1 << WDE);
  } else if (mode == WDT_MODE_INTERRUPT_RESET) {
    wdt_config |= (1 << WDIE) | (1 << WDE);
  }

  cli();

  WDTCSR = (1 << WDCE) | (1 << WDE);
  WDTCSR = wdt_config;

  sei();
}
/* END OF WDT SECTION*/

typedef struct {
  uint8_t left;
  uint8_t right;
  uint8_t motion;
  uint8_t flag;
  uint8_t setting;
  uint8_t setting_index;
  uint16_t aggression;
} movement;

movement my_data = {200, 200, 0, 0, 0};
unsigned long out_of_sight_time = 0;   // total time target has been out of sight (ms)
const unsigned long delta_time = 500;  // loop delay in milliseconds

/* ADC SECTION */
void adc_init() {
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read() {
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}
/* END OF ADC SECTION */

/* EEPROM SECTION */
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
/* END OF EEPROM SECTION */

/* HC-SR04 SECTION */
uint16_t readDistance(uint8_t trigPin, uint8_t echoPin) {
  PORTD &= ~(1 << trigPin); 
  delayMicroseconds(2);

  PORTD |= (1 << trigPin);
  delayMicroseconds(10);
  PORTD &= ~(1 << trigPin);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) return 200;

  uint16_t distance = duration / 58;

  if (distance > 200) distance = 200;

  return distance;
}

bool check_in_sensor_sight(uint8_t distance) {
  uint8_t max_distance = 1 + (my_data.aggression * 39) / 1023;  // scales between 1 and 50
  return (distance >= 1 && distance < max_distance);
}

bool line_of_fire() {
  bool target_on_left = check_in_sensor_sight(my_data.left);
  bool target_on_right = check_in_sensor_sight(my_data.right);

  if (!target_on_left && !target_on_right) {
    base_servo.write(SENTRY_CENTER_ANGLE);
    return false;
  }

  if (target_on_left && target_on_right) {
    base_servo.write(SENTRY_CENTER_ANGLE);
    return true;
  }

  if (target_on_left) {
    base_servo.write(SENTRY_RIGHT_ANGLE);
  } else {
    base_servo.write(SENTRY_LEFT_ANGLE);
  }


  return true;
}
/* END OF HC-SR04 SECTION */

/* UART SECTION */
void usart_init_simple(void) {
    // assume:
    // F_CPU and BAUDRATE are defined macros
    uint16_t ubrr = (F_CPU / (16UL * BAUDRATE)) - 1;

    // set baud rate
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);

    // frame format: 8 data bits, no parity, 1 stop bit
    // UCSZ01:0 = 3 (8-bit data), UPM01:0 = 0 (parity disabled), USBS0 = 0 (1 stop bit)
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

    // enable transmitter only (TX_ENABLE)
    UCSR0B = (1 << TXEN0);

    // disable interrupts
    UCSR0B &= ~((1 << RXCIE0) | (1 << TXCIE0) | (1 << UDRIE0));
}


uint8_t usart_receive_byte(void)
{
  while (!(UCSR0A & (1 << RXC0)));

  return UDR0;
}

void usart_transmit_byte(uint8_t ch)
{
  while (!(UCSR0A & (1 << UDRE0)));

  UDR0 = ch;
}

void usart_disable(void)
{
  /* Disable the transm and recv */
  UCSR0B &= ~((1 << TXEN0) | (1 << RXEN0));

  /* Clear flags*/
  UCSR0A = 0;

  /* Reset baud rate */
  UBRR0L = 0;
  UBRR0H = 0;

  /* Clear other stuff */
  UCSR0C = 0;
}

// helper cuz why not
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
/* END OF UART SECTION */

/* ISR SECTION */

// Watchdog Timer interrupt to reset states every 8s
ISR(WDT_vect) {
  my_data.motion = 0;        // Reset motion state, allow polling again
  my_data.flag = 1;
  my_data.left = 200;
  my_data.right = 200;
  out_of_sight_time = 0;
}

ISR(PCINT0_vect) {
  static unsigned long last_interrupt_time = 0;
  unsigned long current_time = millis();
  if ((current_time - last_interrupt_time) > DELAY_DEBOUNCE) {
    if (!(PINB & (1 << PB4))) {
      my_data.setting_index = (my_data.setting_index + 1) % MODES_NUM;
      my_data.setting = modes_num[my_data.setting_index];
    }
    last_interrupt_time = current_time;
  }
}
/* END OF ISR SECTION */

// Debug function to print R, L, A and mode values
void debug_print()
{
  // send R
  usart_transmit_byte('R');
  usart_transmit_number(my_data.right);

  usart_transmit_byte(' ');

  // send L
  usart_transmit_byte('L');
  usart_transmit_number(my_data.left);

  usart_transmit_byte(' ');

  // send A
  usart_transmit_byte('A');
  usart_transmit_number(my_data.aggression);

  usart_transmit_byte(' ');

  // send mode
  usart_transmit_byte('M');
  usart_transmit_byte(my_data.setting);

  // newline + carriage return
  usart_transmit_byte('\n');
  usart_transmit_byte('\r');
}

void setup() {

  usart_init_simple();
  manual_wdt_enable(WDT_PRESCALER_8S, WDT_MODE_INTERRUPT);

  DDRD |= (1 << PD2) | (1 << PD6);   // TRIG_R and TRIG_L output
  DDRD &= ~((1 << PD3) | (1 << PD7)); // ECHO_R and ECHO_L output

  DDRB &= ~(1 << PB0); // pir

  DDRB &= ~(1 << PB4); // button

  adc_init();
  my_data.aggression = adc_read();

  base_servo.attach(SERVO_PIN);
  base_servo.write(SENTRY_CENTER_ANGLE); // Start centered
  
  for (uint16_t i = 0; i < MODES_NUM; i++) {
    modes_num[i] = eeprom_read(i);
  }

  my_data.setting_index = 0;
  my_data.setting = modes_num[my_data.setting_index];

  PCICR |= (1 << PCIE0);     // Enable PCINT group 0 (PCINT0..7)
  PCMSK0 |= (1 << PCINT4);   // Enable PCINT4 (PB4 / D12)
  sei();
}

void loop() {
  switch (my_data.setting) {
    case 'S': {  // Mode S (Sentinel mode)
      // Only poll PIR sensor if previous state was no motion
      if (my_data.motion == 0) {
        my_data.motion = (PINB & (1 << PB0)) ? 1 : 0;
      }

      if (my_data.motion == 1) {
        my_data.right = readDistance(PD2, PD3);
        my_data.left = readDistance(PD6, PD7);

        #if DEBUG
          debug_print();
        #endif

        bool target_in_sight = line_of_fire();

        if (target_in_sight) {
          out_of_sight_time = 0;
        } else {
          out_of_sight_time += delta_time;
        }
      }

      delay(delta_time);
      break;
    }

    case 'R': {
      static int8_t direction = 1;
      static uint8_t angle = RADAR_MIN_ANGLE;
    
      base_servo.write(angle);
    
      // Read sensors
      my_data.right = readDistance(PD2, PD3);
      my_data.left = readDistance(PD6, PD7);
    
      #if DEBUG
        debug_print();
      #endif
    
      // Update angle
      angle += RADAR_STEP * direction;
    
      // Reverse direction if limits reached
      if (angle >= RADAR_MAX_ANGLE) {
        angle = RADAR_MAX_ANGLE;
        direction = -1;
      } else if (angle <= RADAR_MIN_ANGLE) {
        angle = RADAR_MIN_ANGLE;
        direction = 1;
      }
    
      delay(RADAR_DELAY);
      break;
    }
    case 'P': {
      my_data.right = readDistance(PD2, PD3);
      my_data.left = readDistance(PD6, PD7);
      #if DEBUG
        debug_print();
      #endif
      delay(1000);
    }

    default:
      // Unknown mode fallback
      base_servo.write(SENTRY_CENTER_ANGLE);
      delay(500);
      break;
  }
}
