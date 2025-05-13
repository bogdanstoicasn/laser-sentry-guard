#include "../include/pwm_driver.h"

uint8_t tc0_pwm_init(pwm_mode mode, pwm_prescaler prescaler, uint8_t duty_cycle)
{
    /* Set PD6 (OC0A) as output */
    DDRD |= (1 << PORTD6);

#ifdef CLEAN
    /* Reset TCCR0A and TCCR0B */
    TCCR0A &= ~((1 << WGM00) | (1 << WGM01) | (1 << COM0A0) | (1 << COM0A1));
    TCCR0B &= ~((1 << WGM02) | (1 << CS00) | (1 << CS01) | (1 << CS02));
#endif

    /* Set PWM mode */
    switch (mode) {
    case MODE_FAST:
        /* Fast PWM mode (TOP = 0xFF) */
        TCCR0A |= (1 << WGM00) | (1 << WGM01);
        break;
    case MODE_PHASE_CORRECT:
        /* Phase Correct PWM */
        TCCR0A |= (1 << WGM00);
        break;
    default:
        return STATUS_FAILURE;
    }

    /* Set duty cycle */
    OCR0A = duty_cycle;  // Adjust if using OC0B instead

    /* Enable PWM output on OC0A (PD6) */
    TCCR0A |= (1 << COM0A1);

    /* Set clock prescaler */
    TCCR0B |= prescaler;  

    return STATUS_SUCCESS;
}

