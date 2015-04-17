/**
 * @file lm4f120_interrupt_service_routines.s
 *
 * Platform-specific McRTOS kernel interrupt service routines for for the LM4F120 SoC
 *
 * Copyright (C) 2014 German Rivera
 *
 * @author German Rivera
 */

#include "McRTOS_arm_cortex_m_macros.s"

.text
.thumb
//.syntax unified

/*
 * Generate ISR functions:
 */

/**
 * UART0 ISR
 */
GEN_ISR_FUNCTION lm4f120_uart0_isr, g_rtos_interrupt_uart0_p, \
                 lm4f120_uart_interrupt_e_handler

#if 0 // ???
/**
 * ADC0 ISR
 */
GEN_ISR_FUNCTION lm4f120_adc0_isr, g_rtos_interrupt_adc0_p, \
                 lm4f120_adc_interrupt_e_handler

/**
 * I2C0 ISR
 */
GEN_ISR_FUNCTION lm4f120_i2c0_isr, g_rtos_interrupt_i2c0_p, \
                 lm4f120_i2c_interrupt_e_handler
#endif

.end


