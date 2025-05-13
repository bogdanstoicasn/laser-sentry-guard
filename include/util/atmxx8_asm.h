#ifndef ATM_XX8_ASM_H
#define ATM_XX8_ASM_H

#include <stdint.h>

/* Disables/Enables interrupts by setting the global interrupt mask.
 * Returns from interrupt routine, enabling global interrupts.
 */
#define ENABLE_INTERRUPTS() __asm__ __volatile__ ("sei" ::: "memory")
#define DISABLE_INTERRUPTS() __asm__ __volatile__("cli" ::: "memory")
#define RETURN_INTERRUPT() __asm__ __volatile__ ("reti"::: "memory")

#define WDT_RESET() __asm__ __volatile__ ("wdr")

#define F_CPU 16000000UL

#define INACC_DELAY_CYCLES(us) do  {                  \
	uint32_t cycles = (F_CPU / 1000000UL) * (us) / 4; \
	while(cycles--) __asm__ __volatile__("nop");      \
} while(0)

#define INACC_DELAY_MS(ms) INACC_DELAY_CYCLES((ms) * 1000)

#endif
