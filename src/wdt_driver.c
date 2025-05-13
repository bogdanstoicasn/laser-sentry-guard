#include "../include/wdt_driver.h"

void wdt_disable()
{
    DISABLE_INTERRUPTS();

    WDT_RESET();

    MCUSR &= ~(1 << WDRF);

    /* keep basic settings to avoid unintentional timeout */
    WDTCSR |= (1 << WDCE) | (1 << WDE);  /* Enable change mode and watchdog reset */

    WDTCSR = 0x00;  /* Disable watchdog */

    ENABLE_INTERRUPTS();
}

void wdt_enable(wdt_prescaler value, wdt_mode modes)
{
	/* start with prescaler*/
    uint8_t wdt_config = value; 

	/* add mode */
    if (modes == WDT_MODE_INTERRUPT)
        wdt_config |= (1 << WDIE);
    else if (modes == WDT_MODE_RESET)
        wdt_config |= (1 << WDE);
    else if (modes == WDT_MODE_INTERRUPT_RESET)
        wdt_config |= (1 << WDIE) | (1 << WDE);

    DISABLE_INTERRUPTS();

    /* 4 cycles to change WDTCSR */
    WDTCSR = (1 << WDCE) | (1 << WDE);
    
    /* set new config */
    WDTCSR = wdt_config;

    ENABLE_INTERRUPTS();
}

