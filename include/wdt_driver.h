#ifndef WDT_DRIVER_H
#define WDT_DRIVER_H

#include "./util/atmxx8_base.h"
#include "./util/atmxx8_asm.h"

typedef enum {
	WDT_MODE_INTERRUPT = 0,
	WDT_MODE_RESET = 1,
	WDT_MODE_INTERRUPT_RESET = 2
} wdt_mode;

typedef enum {
	WDT_PRESCALER_16MS = 0,
	WDT_PRESCALER_32MS = (1 << WDP0),
	WDT_PRESCALER_64MS = (1 << WDP1),
	WDT_PRESCALER_125MS = (1 << WDP0) | (1 << WDP1), /* 0.125s */
	WDT_PRESCALER_250MS = (1 << WDP2), /* 0.25s*/
	WDT_PRESCALER_500MS = (1 << WDP0) | (1 << WDP2), /* 0.5s */
	WDT_PRESCALER_1S = (1 << WDP1) | (1 << WDP2),
	WDT_PRESCALER_2S = (1 << WDP0) | (1 << WDP1) | (1 << WDP2),
	WDT_PRESCALER_4S = (1 << WDP3),
	WDT_PRESCALER_8S = (1 << WDP0) | (1 << WDP3)
} wdt_prescaler;

/* 
 * Enable the watchdog timer
 * 
 * @param value - prescaler value
 * @param modes - mode of operation
 */
void wdt_enable(wdt_prescaler value, wdt_mode modes);

/* 
 * Disable the watchdog timer
 */
void wdt_disable(void);

#endif
