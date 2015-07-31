/**
 * @file McRTOS_arm_cortex_m_macros.s
 *
 * Machine-specific macros invoked form assembly source files.
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */

#include "arm_defs.h"

/*
 * Imported symbols
 */
.extern g_McRTOS_p
.extern debug_dump_r0_to_r3
.extern debug_capture_registers
.extern rtos_k_enter_interrupt
.extern rtos_k_exit_interrupt
.extern cortex_m_save_other_registers


/**
 * Macro that prints registers r0 to r3
 *
 *
 * INPUT REGISTERS:
 * r0 - r3
 *
 * OUTPUT REGISTERS:
 * None
 *
 * CLOBBERED REGISTERS:
 * None
 */
.macro DEBUG_DUMP_R0_TO_R3
    push    {r0-r3}
    push    {lr}
    bl      debug_dump_r0_to_r3
    pop     {r0}
    mov     lr, r0
    pop     {r0-r3}
.endm


/**
 * Macro that captures registers r0 to r3
 *
 *
 * INPUT REGISTERS:
 * r0 - r3
 *
 * OUTPUT REGISTERS:
 * None
 *
 * CLOBBERED REGISTERS:
 * None
 */
.macro DEBUG_CAPTURE_R0_TO_R3
    push    {r0-r3}
    push    {lr}
    bl      debug_capture_registers
    pop     {r0}
    mov     lr, r0
    pop     {r0-r3}
.endm


/**
 * Macro that retrieves in \_reg_ the value of
 * g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()].cpc_current_execution_context_p
 *
 * TODO: Add support for multiple cores.
 *
 * INPUT REGISTERS:
 * None
 *
 * OUTPUT REGISTERS:
 * \_reg_
 *
 * CLOBBERED REGISTERS:
 * None
 */
.macro GET_MCRTOS_CURRENT_EXECUTION_CONTEXT _reg_
    ldr     \_reg_, =g_McRTOS_p /* \_reg_ = &g_McRTOS_p */
    ldr     \_reg_, [\_reg_]    /* \_reg_ = g_McRTOS_p */
    ldr     \_reg_, [\_reg_, #(RTOS_RTS_CPU_CONTROLLERS_OFFSET + RTOS_CPC_CURRENT_EXECUTION_CONTEXT_P_OFFSET)]
.endm


/**
 * Macro that stores the value of \_reg_value_ in
 * g_McRTOS_p->rts_cpu_controllers[0].cpc_current_execution_context_p
 *
 *
 * INPUT REGISTERS:
 * \_reg_value_
 * \_reg_scratch_
 *
 * OUTPUT REGISTERS:
 * None
 *
 * CLOBBERED REGISTERS:
 * \_reg_scratch_
 */
.macro SET_MCRTOS_CURRENT_EXECUTION_CONTEXT _reg_value_, _reg_scratch_
    ldr     \_reg_scratch_, =g_McRTOS_p         /* \_reg_scratch_ = &g_McRTOS_p */
    ldr     \_reg_scratch_, [\_reg_scratch_]    /* \_reg_scratch_ = g_McRTOS_p */
    str     \_reg_value_, [\_reg_scratch_, #(RTOS_RTS_CPU_CONTROLLERS_OFFSET + RTOS_CPC_CURRENT_EXECUTION_CONTEXT_P_OFFSET)]
.endm


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
 * r4 = \_g_rtos_interrupt_p_
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
     * We can clobber r0-r3, r12, pc and psr as they are pre-saved registers
     *
     * Cortex-M process leave interrupts enabled on the CPU upon entering
     * an exception, so we need to disable them here:
     *
     * NOTE: A nested interrupt may have occurred between the time the original
     * interrupt interrupted the current thread and this point (after the CPU
     * saved the pre-saved registers on the PSP stack). In that case,
     * lr will be CPU_EXC_RETURN_TO_HANDLER_MODE but the current context will be
     * of type RTOS_THREAD_CONTEXT.
     */
    cpsid   i
    isb

#ifdef _MEASURE_INTERRUPTS_DISABLED_TIME_
    mov	    r0, lr
    bl	    rtos_start_interrupts_disabled_time_measure
    mov	    lr, r0
#endif

    /*
     * Set r2 to \_g_rtos_interrupt_p_
     */
    ldr     r2, =\_g_rtos_interrupt_p_
    ldr     r2, [r2]

    /*
     * Set r0 to point to current execution context.
     */
    GET_MCRTOS_CURRENT_EXECUTION_CONTEXT r0

    add     r0, r0, #RTOS_CTX_CPU_REGISTERS_OFFSET

    /*
     * Save non-pre-saved registers:
     *
     * r0 == &(current_execution_context_p->ctx_cpu_saved_registers)
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
     * r4 == \_g_rtos_interrupt_p_
     */
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
 * r4 = \_g_rtos_interrupt_p_
 *
 * CLOBBERED REGISTERS:
 * N/A
 *
 */
.macro RTOS_ENTER_ISR _g_rtos_interrupt_p_
    RTOS_ENTER_ISR_COMMON \_g_rtos_interrupt_p_

    /*
     * Re-enable interrupts
     *
     * NOTE: This is necessary so that higher priority interrupts can preempt this
     * ISR (nested interrupts).
     */

#ifdef _MEASURE_INTERRUPTS_DISABLED_TIME_
    bl	    rtos_stop_interrupts_disabled_time_measure
#endif

    isb
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
 * r4 = \_g_rtos_interrupt_p_
 *
 * CLOBBERED REGISTERS:
 * N/A
 */
.macro RTOS_ENTER_SPLIT_ISR _g_rtos_interrupt_p_, _interrupt_d_half_handler_function_
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

#ifdef _MEASURE_INTERRUPTS_DISABLED_TIME_
    bl	    rtos_stop_interrupts_disabled_time_measure
#endif

    isb
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
    isb

#ifdef _MEASURE_INTERRUPTS_DISABLED_TIME_
    bl	    rtos_start_interrupts_disabled_time_measure
#endif

    /*
     * Call rtos_k_exit_interrupt()
     *
     * NOTE: This function never returns here.
     */
    bl      rtos_k_exit_interrupt

    /*
     * We should never come back here
     */
    bkpt    #0
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

.thumb_func
.func \_rtos_isr_func_

\_rtos_isr_func_:
    RTOS_ENTER_ISR \_g_rtos_interrupt_p_

    /*
     * r4 == \_g_rtos_interrupt_p_
     */

    /*
     * Call \_interrupt_e_handler_function_(\_g_rtos_interrupt_p_)
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

.thumb_func
.func \_rtos_isr_func_

\_rtos_isr_func_:
    RTOS_ENTER_SPLIT_ISR \_g_rtos_interrupt_p_, \_interrupt_d_handler_function_

    /*
     * r4 == \_g_rtos_interrupt_p_
     */

    /*
     * Call \_interrupt_e_handler_function_(\_g_rtos_interrupt_p_)
     */
    mov     r0, r4
    bl      \_interrupt_e_handler_function_

    RTOS_EXIT_ISR
.endfunc

.endm

