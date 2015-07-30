/**
 * @file kl25z_interrupt_service_routines.s
 *
 * Platform-specific McRTOS kernel interrupt service routines for for the KL25 SoC
 *
 * Copyright (C) 2014 German Rivera
 *
 * @author German Rivera
 */

#include <McRTOS/McRTOS_arm_cortex_m_macros.s>

.text
.thumb
.syntax unified

/*
 * Generate ISR functions:
 */

/**
 * UART0 Receive/Transmit ISR
 */
GEN_ISR_FUNCTION lpc54102_uart0_irq_isr, g_rtos_interrupt_uart0_p, \
                 lpc54102_uart_interrupt_e_handler

.end


