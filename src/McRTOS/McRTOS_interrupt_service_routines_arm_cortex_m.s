/**
 * @file McRTOS_interrupt_service_routines_arm_cortex_m.s
 *
 * Machine-specific McRTOS kernel interrupt service routines common to all ARM
 * Cortex-M processors
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */

#include "McRTOS_arm_cortex_m_macros.s"

.text
.thumb
.syntax unified

/*
 * Generate ISR functions:
 */

/**
 * Systick ISR
 */
GEN_ISR_FUNCTION cortex_m_systick_isr, g_rtos_interrupt_systick_p, \
                 rtos_tick_timer_interrupt_handler

.end


