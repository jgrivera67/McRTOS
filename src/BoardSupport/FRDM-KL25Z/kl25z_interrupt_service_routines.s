/**
 * @file kl25z_interrupt_service_routines.s
 *
 * Platform-specific McRTOS kernel interrupt service routines for for the KL25 SoC
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
GEN_ISR_FUNCTION kl25_uart0_isr, g_rtos_interrupt_uart0_p, \
                 kl25_uart_interrupt_e_handler

/**
 * ADC0 ISR
 */
GEN_ISR_FUNCTION kl25_adc0_isr, g_rtos_interrupt_adc0_p, \
                 kl25_adc_interrupt_e_handler

/**
 * TPM0 ISR
 */
GEN_ISR_FUNCTION kl25_tpm0_isr, g_rtos_interrupt_tpm0_p, \
                 kl25_tpm_interrupt_e_handler

/**
 * TPM1 ISR
 */
GEN_ISR_FUNCTION kl25_tpm1_isr, g_rtos_interrupt_tpm1_p, \
                 kl25_tpm_interrupt_e_handler

/**
 * I2C0 ISR
 */
GEN_ISR_FUNCTION kl25_i2c0_isr, g_rtos_interrupt_i2c0_p, \
                 kl25_i2c_interrupt_e_handler

.end


