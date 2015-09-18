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
GEN_ISR_FUNCTION k64f_uart0_rx_tx_isr, g_rtos_interrupt_uart0_rx_tx_p, \
                 k64f_uart_rx_tx_interrupt_e_handler

/**
 * UART0 Error ISR
 */
GEN_ISR_FUNCTION k64f_uart0_err_isr, g_rtos_interrupt_uart0_err_p, \
                 k64f_uart_err_interrupt_e_handler

/**
 * UART4 Receive/Transmit ISR
 */
GEN_ISR_FUNCTION k64f_uart4_rx_tx_isr, g_rtos_interrupt_uart4_rx_tx_p, \
                 k64f_uart_rx_tx_interrupt_e_handler

/**
 * UART4 Error ISR
 */
GEN_ISR_FUNCTION k64f_uart4_err_isr, g_rtos_interrupt_uart4_err_p, \
                 k64f_uart_err_interrupt_e_handler

#if 0 // ???
/**
 * ADC0 ISR
 */
GEN_ISR_FUNCTION k64f_adc0_isr, g_rtos_interrupt_adc0_p, \
                 k64f_adc_interrupt_e_handler

/**
 * FTM0 ISR
 */
GEN_ISR_FUNCTION k64f_ftm0_isr, g_rtos_interrupt_ftm0_p, \
                 k64f_ftm_interrupt_e_handler

/**
 * FTM1 ISR
 */
GEN_ISR_FUNCTION k64f_tpm1_isr, g_rtos_interrupt_ftm1_p, \
                 k64f_ftm_interrupt_e_handler
#endif // ???

/**
 * I2C0 ISR
 */
GEN_ISR_FUNCTION k64f_i2c0_isr, g_rtos_interrupt_i2c0_p, \
                 k64f_i2c_interrupt_e_handler

/**
 * GPIO PORT C ISR
 */
GEN_ISR_FUNCTION k64f_port_c_isr, g_rtos_interrupt_port_c_p, \
                 k64f_port_c_interrupt_e_handler

/**
 * ENET Tx ISR
 */
GEN_ISR_FUNCTION k64f_enet_transmit_isr, g_rtos_interrupt_enet_tx_p, \
                 k64f_enet_transmit_interrupt_e_handler

/**
 * ENET Rx ISR
 */
GEN_ISR_FUNCTION k64f_enet_receive_isr, g_rtos_interrupt_enet_rx_p, \
                 k64f_enet_receive_interrupt_e_handler

/**
 * ENET Error ISR
 */
GEN_ISR_FUNCTION k64f_enet_error_isr, g_rtos_interrupt_enet_error_p, \
                 k64f_enet_error_interrupt_e_handler

.end


