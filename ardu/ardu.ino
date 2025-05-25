#define DEBUG 1  // Set to 0 to disable debug prints

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <Servo.h>

#define TRIG_R 2    // Right sensor trigger pin
#define ECHO_R 3    // Right sensor echo pin

#define TRIG_L 6    // Left sensor trigger pin
#define ECHO_L 7    // Left sensor echo pin

#define PIR_PIN 8   // Motion sensor input pin (poll selectively)

#define ENABLE_INTERRUPTS() __asm__ __volatile__ ("sei" ::: "memory")
#define DISABLE_INTERRUPTS() __asm__ __volatile__("cli" ::: "memory")
#define RETURN_INTERRUPT() __asm__ __volatile__ ("reti"::: "memory")

#define MODE 'S'  // or 'R'

#if MODE == 'S'
  #define MODE_ADDR 0x00
#elif MODE == 'R'
  #define MODE_ADDR 0x01
#else
  #error "Invalid MODE. Use 'S' or 'R'."
#endif

Servo base_servo;               // Servo object
const uint8_t SERVO_PIN = 9;   // Servo control pin (choose a PWM pin)

const uint8_t CENTER_ANGLE = 90;
const uint8_t LEFT_ANGLE = 60;
const uint8_t RIGHT_ANGLE = 120;

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

typedef struct {
  uint8_t left;
  uint8_t right;
  uint8_t motion;
  uint8_t flag;
  uint8_t setting;
  uint16_t aggression;
} movement;

movement my_data = {200, 200, 0, 0, 0};
unsigned long out_of_sight_time = 0;   // total time target has been out of sight (ms)
const unsigned long delta_time = 500;  // loop delay in milliseconds

void adc_init() {
  ADMUX = (1 << REFS0);
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t adc_read() {
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC));
  return ADC;
}

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

void setup() {
  Serial.begin(9600);

  manual_wdt_enable(WDT_PRESCALER_8S, WDT_MODE_INTERRUPT);

  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);
  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);

  pinMode(PIR_PIN, INPUT);

  adc_init();
  my_data.aggression = adc_read();

  base_servo.attach(SERVO_PIN);
  base_servo.write(CENTER_ANGLE); // Start centered
  my_data.setting = eeprom_read(MODE_ADDR);
  Serial.print('M');
  Serial.print(my_data.setting);
  Serial.println();
  sei();
}

uint16_t readDistance(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000);

  if (duration == 0) return 200;

  uint16_t distance = duration / 58;

  if (distance > 200) distance = 200;

  return distance;
}

// Watchdog Timer interrupt to reset states every 8s
ISR(WDT_vect) {
  my_data.motion = 0;        // Reset motion state, allow polling again
  my_data.flag = 1;
  my_data.left = 200;
  my_data.right = 200;
  out_of_sight_time = 0;
}

bool check_in_sensor_sight(uint8_t distance) {
  uint8_t max_distance = 1 + (my_data.aggression * 39) / 1023;  // scales between 1 and 50
  return (distance >= 1 && distance < max_distance);
}

bool check_target_in_sight() {
  bool target_on_left = check_in_sensor_sight(my_data.left);
  bool target_on_right = check_in_sensor_sight(my_data.right);

  if (!target_on_left && !target_on_right) {
    base_servo.write(CENTER_ANGLE);
    return false;
  }

  if (target_on_left && target_on_right) {
    base_servo.write(CENTER_ANGLE);
    return true;
  }

  if (target_on_left) {
    base_servo.write(RIGHT_ANGLE);
  } else { // target_on_right
    base_servo.write(LEFT_ANGLE);
  }


  return true;
}

#if MODE == 'S'

void loop() {
  // Only poll PIR sensor if previous state was no motion (motion==0)
  if (my_data.motion == 0) {
    my_data.motion = (digitalRead(PIR_PIN) == HIGH) ? 1 : 0;
  }
  // If motion detected, read sensors and move servo accordingly
  if (my_data.motion == 1) {
    my_data.right = readDistance(TRIG_R, ECHO_R);
    my_data.left = readDistance(TRIG_L, ECHO_L);

  #if DEBUG
    Serial.print("R:");
    Serial.print(my_data.right);
    Serial.print(" L:");
    Serial.print(my_data.left);
    Serial.print(" A:");
    Serial.print(my_data.aggression);
    Serial.println();
  #endif

    bool target_in_sight = check_target_in_sight();

    if (target_in_sight) {
      out_of_sight_time = 0;
    } else {
      out_of_sight_time += delta_time;
    }
  }

  delay(delta_time);
}

#elif MODE == 'R'

void loop() {
  static int8_t direction = 1;               // 1 = right, -1 = left
  static uint8_t angle = CENTER_ANGLE;       // Start at center

  angle += 2 * direction;

  if (angle >= 105 || angle <= 75) {
    direction *= -1;
    angle += 2 * direction; // prevent edge sticking
  }

  base_servo.write(angle);

  // Read sensors each sweep step
  my_data.right = readDistance(TRIG_R, ECHO_R);
  my_data.left = readDistance(TRIG_L, ECHO_L);

  Serial.print("Radar angle: ");
  Serial.print(angle);
  Serial.print(" | R: ");
  Serial.print(my_data.right);
  Serial.print(" cm | L: ");
  Serial.print(my_data.left);
  Serial.println(" cm");

  delay(100);  // sweep speed
}

#endif
