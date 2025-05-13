#ifndef INTERRUPT_DRIVER_H
#define INTERRUPT_DRIVER_H

#include "./util/atmxx8_base.h"

/* First define USER_ISR in main then include the 328p-d.h library */

#ifdef USER_ISR

void __vector_1(void) __attribute__ ((signal, used)); /* Ext interrupt request 0*/
void __vector_2(void) __attribute__ ((signal, used)); /* Ext interrupt request 1 */
void __vector_3(void) __attribute__ ((signal, used)); /* Pin change interrupt request 0*/
void __vector_4(void) __attribute__ ((signal, used)); /* Pin change interrupt request 1 */
void __vector_5(void) __attribute__ ((signal, used)); /* Pin change interrupt request 2*/
void __vector_6(void) __attribute__ ((signal, used)); /* Watchdog time-out interrupt */
void __vector_7(void) __attribute__ ((signal, used)); /* Timer/Counter2 compare match A */
void __vector_8(void) __attribute__ ((signal, used)); /* Timer/COunter2 compare match B */
void __vector_9(void) __attribute__ ((signal, used)); /* Timer/Counter2 overflow */
void __vector_10(void) __attribute__ ((signal, used)); /* Timer/Counter1 capture event */
void __vector_11(void) __attribute__ ((signal, used)); /* Timer/Counter1 compare match A */
void __vector_12(void) __attribute__ ((signal, used)); /* Timer/Counter1 compare match B */
void __vector_13(void) __attribute__ ((signal, used)); /* Timer/Counter1 overflow */
void __vector_14(void) __attribute__ ((signal, used)); /* Timer/Counter0 compare match A */
void __vector_15(void) __attribute__ ((signal, used)); /* Timer/Counter0 compare match B */
void __vector_16(void) __attribute__ ((signal, used)); /* Timer/Counter0 overflow */
void __vector_17(void) __attribute__ ((signal, used)); /* SPI serial transfer complete */
void __vector_18(void) __attribute__ ((signal, used)); /* USART RX complete */
void __vector_19(void) __attribute__ ((signal, used)); /* USART, data register empty */
void __vector_20(void) __attribute__ ((signal, used)); /* USRAT TX complete */
void __vector_21(void) __attribute__ ((signal, used)); /* ADC conversion complete */
void __vector_22(void) __attribute__ ((signal, used)); /* EEPROM ready */
void __vector_23(void) __attribute__ ((signal, used)); /* Analog comparator */
void __vector_24(void) __attribute__ ((signal, used)); /* 2-wire serial interface */
void __vector_25(void) __attribute__ ((signal, used)); /* Store program memory ready */

#else

#warning "USER_ISR is NOT defined"

#endif

#endif
