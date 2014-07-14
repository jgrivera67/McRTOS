/**
 * @file McRTOS_kernel_services_arm_cortex_m.s
 *
 * Machine-specific McRTOS kernel services for ARM Cortex-M processors
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */

#include "McRTOS_arm_cortex_m_macros.s"

.global rtos_k_restore_execution_context
.global rtos_k_synchronous_context_switch
.global rtos_k_enter_debugger_saving_current_context

.extern fdc_trace_rtos_context_switch
.extern rtos_start_interrupts_disabled_time_measure
.extern rtos_stop_interrupts_disabled_time_measure
.extern check_synchronous_context_switch_preconditions
.extern capture_assert_failure

.text
.thumb

/**
 * Restores the given McRTOS execution context CPU registers on the calling
 * CPU core and updates cpc_current_execution_context_p for the calling CPU.
 *
 * void
 * rtos_k_restore_execution_context(
 *      const struct rtos_execution_context *execution_context_p,
 *      rtos_context_switch_type_t ctx_switch_type)
 *
 * @param   execution_context_p (r0): Pointer to the execution context to be
 *          restored.
 *
 * @param   context_switch_type(r1): type of context switch that is calling
 *          this function
 * @return  none
 *
 * @pre     CPU interrupts are disabled and the current CPU mode is handler mode.
 *          These assumptions are checked in fdc_trace_rtos_context_switch().
 */
.thumb_func
.func rtos_k_restore_execution_context

rtos_k_restore_execution_context:
    /*
     * r0 == execution_context_p
     * r1 == context_switch_type
     *
     * NOTE: We can clobber all registers in this function, since they are going
     * to be restored from the target context.
     */

#ifdef _RELIABILITY_CHECKS_
     /*
      * Save r0 in a callee-saved register, as r0 will get clobbered by
      * fdc_trace_rtos_context_switch:
      */
     mov    r4, r0
     bl     fdc_trace_rtos_context_switch

     /*
      * Restore r0
      */
     mov    r0, r4
#endif /* _RELIABILITY_CHECKS_ */

    /*
     * Set r4 to &execution_context_p->ctx_cpu_saved_registers
     *
     * r0 == execution_context_p
     */
    mov r4, #RTOS_CTX_CPU_REGISTERS_OFFSET
    add r4, r0, r4

#ifdef _MEASURE_INTERRUPTS_DISABLED_TIME_
    /*
     * Call rtos_stop_interrupts_disabled_time_measure():
     *
     * NOTE:
     * - It is assumed that earlier in the code path that lead us here, we made
     *   a call to rtos_start_interrupts_disabled_time_measure().
     */
    bl      rtos_stop_interrupts_disabled_time_measure
#endif /* _MEASURE_INTERRUPTS_DISABLED_TIME_ */

    /*
     * Restore explicitly-saved registers r4-r11:
     *
     * r4 == &execution_context_p->ctx_cpu_saved_registers
     */
    mov     r0, r4
    mov     r1, #(CPU_REG_R8 * ARM_CPU_WORD_SIZE_IN_BYTES)
    add     r1, r0, r1
    ldmia   r1!, {r4-r7} /* Cortex-M0 only supports ldm for r0-r7 */
    mov     r8, r4
    mov     r9, r5
    mov     r10, r6
    mov     r11, r7
    mov     r1, r0
    ldmia   r1!, {r4-r7} /* Cortex-M0+ only supports ldm for r0-r7 */

    /*
     * NOTE: Above, we use "ldmia r1!" instead of "ldmia r1" because for
     * Cortex-M0+ r1 is incremented with or without the "!".
     */

    /*
     * Update cpc_current_execution_context_p for calling CPU:
     *
     * r0 == &execution_context_p->ctx_cpu_saved_registers
     */
    mov     r2, #RTOS_CTX_CPU_REGISTERS_OFFSET
    sub     r2, r0, r2
    SET_MCRTOS_CURRENT_EXECUTION_CONTEXT r2, r1

    /*
     * Determine if the context to be restored is an interrupt or a
     * thread:
     *
     * r0 == &execution_context_p->ctx_cpu_saved_registers
     */
    ldr     r1, [r0, #(CPU_REG_LR_ON_EXC_ENTRY*ARM_CPU_WORD_SIZE_IN_BYTES)]
    ldr     r2, =CPU_EXC_RETURN_TO_HANDLER_MODE
    cmp     r1, r2
    beq     L_target_context_is_interrupt

    /*
     * Target context is a thread, so we need to restore the PSP stack pointer:
     *
     * r0 == &execution_context_p->ctx_cpu_saved_registers
     */
    ldr     r2, [r0, #(CPU_REG_PSP * ARM_CPU_WORD_SIZE_IN_BYTES)]
    msr     psp, r2

    /*
     * We also need to reset the MSP stack pointer to the bottom of the
     * shared exception stack:
     *
     * TODO: instead of assuming that the interrupt vector table is
     * at address 0x0, read the VTOR register
     */
    mov     r1, #0x0
    ldr     r1, [r1]
    msr     msp, r1

    /*
     * Restore the nPRIV bit of the CONTROL register:
     *
     * r0 == &execution_context_p->ctx_cpu_saved_registers
     */
    ldr     r1, [r0, #(CPU_REG_CONTROL*ARM_CPU_WORD_SIZE_IN_BYTES)]
#if 0
    mov	    r2, #CPU_REG_CONTROL_nPRIV_MASK
    tst	    r1, r2
    beq	    L_return_to_thread_context
    mrs	    r1, control
    orr	    r1, r1, r2
#endif
    msr	    control, r1
    isb

L_return_to_thread_context:
    /*
     * Return from exception to thread:
     *
     * Note that we don't need to explicitly restore the SPSEL bit of
     * the CONTROL register. That is implicitly done by using
     * CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP to return from the
     * exception
     */
    ldr     r0, =CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP
    mov     lr, r0
    isb
    cpsie   i
    bx      lr

L_target_context_is_interrupt:
    /*
     * Target context is an interrupt context, so we need to restore the MSP
     * stack pointer:
     *
     * NOTE: The caller is rtos_k_exit_interrupt() or
     * cortex_m_hard_fault_exception_handler().
     *
     * r0 == &execution_context_p->ctx_cpu_saved_registers
     * r1 == #CPU_EXC_RETURN_TO_HANDLER_MODE
     */
    ldr     r2, [r0, #(CPU_REG_MSP * ARM_CPU_WORD_SIZE_IN_BYTES)]
    msr     msp, r2

    /*
     * Return from nested exception:
     */
    mov     lr, r1
    isb
    cpsie   i
    bx      lr

.endfunc


/**
 * Initiates a synchronous context switch, by triggering a pendSV exception.
 * This function is invoked from rtos_k_condvar_wait() and rtos_k_thread_yield,
 * from rtos_k_mutex_acquire() and rtos_k_condvar_wait_interrupt() if the
 * calling thread needs to be switched out, and from rtos_k_condvar_signal()
 * and rots_k_mutex_release() if there are waiters on the condvar or mutex.
 *
 * void
 * rtos_k_synchronous_context_switch(
 *  struct rtos_execution_context *current_execution_context_p)
 *
 * @pre     The current execution context is a thread other than the
 *          idle thread for the calling CPU.
 *
 * @pre     This routine is called with interrupts disabled.
 *
 * @param   current_execution_context_p (r0): Pointer to the current execution
 *          context.
 *
 * @return  none
 */
.thumb_func
.func rtos_k_synchronous_context_switch

rtos_k_synchronous_context_switch:
#ifdef DEBUG
    /*
     * Call check_synchronous_context_switch_preconditions(r0),
     * preserving r0 and lr
     *
     * r0 == current_execution_context_p
     */
    push    {r0, lr}

    bl      check_synchronous_context_switch_preconditions

    /*
     * Restore r0 and lr
     */
    pop     {r0, r1}    /* Cortex-M0 does not support lr for pop */
    mov     lr, r1
#endif /* DEBUG */

    /*
     * Set r0 to &current_execution_context_p->ctx_cpu_saved_registers:
     *
     * r0 == current_execution_context_p
     *
     * NOTE: Since this is a synchronous context switch, we don't need to
     * preserve r0-r3, as they are caller-saved registers.
     */
    add     r0, r0, #RTOS_CTX_CPU_REGISTERS_OFFSET

    /*
     * We are running in thread mode, so we need to trigger a PendSV exception
     * to properly restore the pre-saved registers of the target context, as the
     * target context could have been switched-out from an interrupt handler:
     *
     * NOTE: The exception will only get triggered when we enable interrupts
     * below.
     *
     * r0 == &current_execution_context_p->ctx_cpu_saved_registers
     */
    ldr     r1, =CPU_SCB_ICSR_REGISTER_ADDR
    ldr     r2, [r1]
    ldr     r3, =CPU_SCB_ICSR_PENDSVSET_MASK
    orr     r2, r2, r3
    str     r2, [r1]

#ifdef _MEASURE_INTERRUPTS_DISABLED_TIME_
    /*
     * Call rtos_stop_interrupts_disabled_time_measure():
     */
    push    {r0}
    mov	    r0, lr
    bl      rtos_stop_interrupts_disabled_time_measure
    mov	    lr, r0
    pop     {r0}
#endif /* _MEASURE_INTERRUPTS_DISABLED_TIME_ */

    /*
     * Enable interrupts to take the PendSV exception:
     *
     * NOTE: We need an ISB barrier before enabling interrupts, to ensure
     * that all instructions before this point have executed, so that we can be
     * certain that the registers saved upon PendSV exception entry have the
     * values we expect. Also, there is no danger of a race with another
     * interrupt handler that could get executed before the PendSV exception
     * handler, since the PendSV exception has higher priority than all
     * interrupts with configurable priority.
     *
     * r0 == &current_execution_context_p->ctx_cpu_saved_registers
     */
    isb
    cpsie    i

    /*
     * When the thread being switched out resumes, it will
     * resume here, and we need to disable interrupts again as that
     * is expected by the callers of this function:
     */
    cpsid   i
    isb

#ifdef _MEASURE_INTERRUPTS_DISABLED_TIME_
    /*
     * Call rtos_start_interrupts_disabled_time_measure():
     */
    mov	    r0, lr
    bl      rtos_start_interrupts_disabled_time_measure
    mov	    lr, r0
#endif /* _MEASURE_INTERRUPTS_DISABLED_TIME_ */

    bx      lr

.endfunc

.end


