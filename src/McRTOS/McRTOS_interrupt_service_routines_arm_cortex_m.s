/**
 * @file McRTOS_interrupt_service_routines_arm_cortex_m.s
 *
 * Machine-specific McRTOS kernel interrupt service routines for ARM Cortex-M processors
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 

#include "arm_defs.h"

/*
 * Imported symbols
 */
.extern rtos_k_enter_interrupt
.extern rtos_k_exit_interrupt
.extern cortex_m_save_other_registers

.text
.thumb
//.syntax unified
//.arch armv6-m

/*???
    .thumb_func
    .align 1
    .global  Reset_Handler
    .type    Reset_Handler, %function
*/


/**
 * This macro saves non-pre-saved registers upon entry to an exception
 * and calls rtos_k_enter_interrupt().
 *
 * @param   _g_rtos_interrupt_p_: global variable that is a pointer to the
 *          McRTOS interrupt object associated with the calling ISR.
 *
 * @pre     pre-saved registers already saved in the stack of the interrupted
 *          context and interrupts currently disabled.
 *
 * INPUT REGISTERS:
 * lr = exception return special value (one of:
 *      CPU_EXC_RETURN_TO_HANDLER_MODE, 
 *      CPU_EXC_RETURN_TO_THREAD_MODE_USING_MSP or
 *      CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP)
 *
 * OUTPUT REGISTERS:
 * r4 = \_g_rtos_interrupt_p_->int_arg_p
 *
 * CLOBBERED REGISTERS:
 * N/A
 *
 * NOTE: For Cortex-M processors when the processor enters an ISR,
 * it automatically removes the pending state from the interrupt. Thus,
 * for Cortex-M we cannot call FDC_ASSERT_INTERRUPT_SOURCE_IS_SET()
 * from an ISR.
 */ 
.macro RTOS_ENTER_ISR_COMMON _g_rtos_interrupt_p_
    /*
     * We can clobber r0-r3, r12, lr, pc and psr as they are pre-saved registers
     */

    /*
     * Set r2 to \_g_rtos_interrupt_p_
     */
    ldr     r2, =\_g_rtos_interrupt_p_
    ldr     r2, [r2]

    /*
     * Set r0 to &(\_g_rtos_interrupt_p_->int_cpu_controller_p->
     *               cpc_current_execution_context_p->ctx_cpu_registers[0])
     */
    ldr     r0, [r2, #RTOS_INT_CPU_CONTROLLER_P_OFFSET]
    ldr     r0, [r0, #RTOS_CPC_CURRENT_EXECUTION_CONTEXT_P_OFFSET]
    add     r0, r0, #RTOS_CTX_CPU_REGISTERS_OFFSET

    /*
     * Save non-pre-saved registers:
     * 
     * r0 == &(\_g_rtos_interrupt_p_->int_cpu_controller_p->
     *               cpc_current_execution_context_p->ctx_cpu_registers[0]) 
     */
    mov     r1, lr
    bl      cortex_m_save_other_registers

    /*
     * NOTE: cortex_m_save_other_registers() only clobbered r0 and r4-r7
     * (r4-r7, after saving them).
     */

    /*
     * Call rtos_k_enter_interrupt(\_g_rtos_interrupt_p_):
     *
     * r2 == \_g_rtos_interrupt_p_
     *
     * NOTE: For Cortex-M, we ignore the return value of rtos_k_enter_interrupt(),
     * as for Cortex-M, we are using a shared stack for all exceptions.
     */
    mov     r0, r2
    mov     r4, r2                      /* save r2 in a callee-save register */
    bl      rtos_k_enter_interrupt
    
    /*
     * Set r4 to \_g_rtos_interrupt_p_->int_arg_p
     *
     * r4 == \_g_rtos_interrupt_p_
     */
    ldr     r4, [r4, #RTOS_INT_ARG_P_OFFSET]
.endm


/**
 * This macro must be invoked at the beginning of McRTOS-aware ISRs
 * that are not split into two-half handlers, and that run with
 * interrupts enabled.
 *
 * @param   _g_rtos_interrupt_p_: global variable that is a pointer to the
 *          McRTOS interrupt object associated with the calling ISR.
 *
 * @pre     pre-saved registers already saved in the stack of the interrupted
 *          context and interrupts currently enabled.
 *
 * INPUT REGISTERS:
 * lr = exception return special value (one of:
 *      CPU_EXC_RETURN_TO_HANDLER_MODE, 
 *      CPU_EXC_RETURN_TO_THREAD_MODE_USING_MSP or
 *      CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP)
 *
 * OUTPUT REGISTERS:
 * r4 = \_g_rtos_interrupt_p_->int_arg_p
 *
 * CLOBBERED REGISTERS:
 * N/A
 *
 */ 
.macro RTOS_ENTER_ISR _g_rtos_interrupt_p_
    /*
     * Cortex-M process leave interrupts enabled on the CPU upon entering 
     * an exception, so we need to disable them here:
     */
    cpsid   i

    RTOS_ENTER_ISR_COMMON \_g_rtos_interrupt_p_

    /*
     * Re-enable interrupts
     *
     * NOTE: This is necessary so that higher priority interrupts can preempt this
     * ISR (nested interrupts).
     */
    cpsie   i
.endm


/**
 * This macro must be invoked at the beginning of McRTOS-aware split ISRs.
 *
 * @param   _g_rtos_interrupt_p_: global variable that is a pointer to the
 *          McRTOS interrupt object associated with the calling ISR.
 *
 * @param   _interrupt_d_half_handler_function_: function name of the
 *          interrupt handler disabled-half function, which must run with
 *          interrupts disabled.
 *
 * INPUT REGISTERS:
 * - sp = banked SP of IRQ mode. sp has the value of the stack pointer
 *   register from the last interrupt context executed. For the first
 *   interrupt context executed since reset, it has the value set by the
 *   ARM Reset handler (ResetHandler), as defined in McRTOS_crt_armv4.s.
 *
 * OUTPUT REGISTERS:
 * r4 = \_g_rtos_interrupt_p_->int_arg_p
 *
 * CLOBBERED REGISTERS:
 * N/A
 */
.macro RTOS_ENTER_SPLIT_ISR _g_rtos_interrupt_p_, _interrupt_d_half_handler_function_
    /*
     * Cortex-M process leave interrupts enabled on the CPU upon entering 
     * an exception, so we need to disable them here:
     */
    cpsid   i

    RTOS_ENTER_ISR_COMMON \_g_rtos_interrupt_p_

    /*
     * Call \_interrupt_d_half_handler_function_(\_g_rtos_interrupt_p_->int_arg_)
     */
    mov     r0, r4
    bl      \_interrupt_d_half_handler_function_

    /*
     * Re-enable interrupts
     *
     * NOTE: This is necessary so that higher priority interrupts can preempt this
     * ISR (nested interrupts).
     */
    cpsie   i
.endm


/**
 * Macro that restores CPU context upon exit from a McRTOS-aware ISR.
 *
 * This macro must be invoked at the end of every McRTOS-aware ISR.
 *
 * INPUT REGISTERS:
 * none
 *
 * OUTPUT REGISTERS:
 * none
 *
 * CLOBBERED REGISTERS:
 * N/A
 */
.macro RTOS_EXIT_ISR
    /*
     * Disable interrupts again:
     */
    cpsid   i

    /*
     * Call rtos_k_exit_interrupt()
     *
     * NOTE: This function only returns if we are in a nested exception.
     */
    bl      rtos_k_exit_interrupt

    /*
     * Return from exception nested exception:
     */
    ldr     r0, =CPU_EXC_RETURN_TO_HANDLER_MODE
    mov     lr, r0
    cpsie   i
    bx      lr
.endm


/**
 * Macro that generates a McRTOS-aware non-split ISR function
 *
 * @param   _rtos_isr_func_: ISR function name
 *
 * @param   _g_interrupt_p_: Name of global pointer variable that 
 *          points to a struct rtos_interrupt. 
 *
 * @param   _interrupt_e_handler_function_: Interrupt handler
 *          function name
 *
 * INPUT REGISTERS:
 * N/A
 *
 * OUTPUT REGISTERS:
 * N/A
 *
 * CLOBBERED REGISTERS:
 * N/A
 */
.macro GEN_ISR_FUNCTION _rtos_isr_func_, _g_rtos_interrupt_p_, \
                        _interrupt_e_handler_function_

.extern \_g_rtos_interrupt_p_
.extern \_interrupt_e_handler_function_

/**
 * void \_rtos_isr_func_(void)
 */
.global \_rtos_isr_func_

.func \_rtos_isr_func_

\_rtos_isr_func_:
    RTOS_ENTER_ISR \_g_rtos_interrupt_p_

    /*
     * r4 == \_g_rtos_interrupt_p_->int_arg_p
     */

    /*
     * Call \_interrupt_e_handler_function_(\_g_rtos_interrupt_p_->int_arg_p)
     */
    mov     r0, r4
    bl      \_interrupt_e_handler_function_

    RTOS_EXIT_ISR
.endfunc

.endm


/**
 * Macro to generate a McRTOS-aware split ISR function. A split ISR
 * has two half-handlers: a disabled-half handler (d_half) that runs
 * with interrupts disabled and an enabled-half handler (e_half) that
 * runs with interrupts enabled.
 *
 * @param   _rtos_isr_func_: ISR function name
 *
 * @param   _g_interrupt_p_: Name of global pointer variable that 
 *          points to a struct rtos_interrupt. 
 *
 * @param   _interrupt_d_handler_function_: disabled-half handler
 *          function name
 *
 * @param   _interrupt_e_handler_function_: enabled-half handler
 *          function name
 *
 * INPUT REGISTERS:
 * N/A
 *
 * OUTPUT REGISTERS:
 * N/A
 *
 * CLOBBERED REGISTERS:
 * N/A
 */
.macro GEN_SPLIT_ISR_FUNCTION _rtos_isr_func_, _g_rtos_interrupt_p_, \
                              _interrupt_d_handler_function_, \
                              _interrupt_e_handler_func_

.extern \_g_rtos_interrupt_p_
.extern \_interrupt_d_handler_function_
.extern \_interrupt_e_handler_function_

/**
 * void \_rtos_isr_func_(void)
 */
.global \_rtos_isr_func_

.func \_rtos_isr_func_

\_rtos_isr_func_:
    RTOS_ENTER_SPLIT_ISR \_g_rtos_interrupt_p_, \_interrupt_d_handler_function_

    /*
     * r4 == \_g_rtos_interrupt_p_->int_arg_p
     */

    /*
     * Call \_interrupt_e_handler_function_(\_g_rtos_interrupt_p_->int_arg_p)
     */
    mov     r0, r4
    bl      \_interrupt_e_handler_function_

    RTOS_EXIT_ISR
.endfunc

.endm

/*
 * Generate ISR functions:
 */

/**
 * Systick ISR
 */
GEN_ISR_FUNCTION cortex_m_systick_isr, g_rtos_interrupt_systick_p, \
                 rtos_tick_timer_interrupt_handler

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

.end

