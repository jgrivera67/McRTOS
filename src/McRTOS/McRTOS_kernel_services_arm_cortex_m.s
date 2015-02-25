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
.global rtos_enter_privileged_mode
.global rtos_exit_privileged_mode

.extern fdc_trace_rtos_context_switch
.extern rtos_start_interrupts_disabled_time_measure
.extern rtos_stop_interrupts_disabled_time_measure
.extern check_synchronous_context_switch_preconditions
.extern rtos_k_enter_privileged_mode
.extern rtos_k_exit_privileged_mode

.text
.thumb
.syntax unified

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
     * Restore CONTROL register:
     * TODO: is this really needed or is it redundant with the restore
     * of lr done below?
     *
     * r0 == &execution_context_p->ctx_cpu_saved_registers
     */
    ldr     r1, [r0, #(CPU_REG_CONTROL*ARM_CPU_WORD_SIZE_IN_BYTES)]
    msr	    control, r1
    isb

    /*
     * Return from exception to thread:
     *
     * Note that we don't need to explicitly restore the SPSEL bit of
     * the CONTROL register. That is implicitly done by using
     * execution_context_p->ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry
     * to return from the exception
     */
    ldr     r1, [r0, #(CPU_REG_LR_ON_EXC_ENTRY*ARM_CPU_WORD_SIZE_IN_BYTES)]
    mov     lr, r1
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


/**
 * Enter privilege mode. It returns true if the CPU was already
 * in privileged mode before, and false otherwise.
 *
 * bool
 * rtos_enter_privileged_mode(void)
 *
 * NOTE: This function is to be invoked from hardware driver functions
 * that are invoked from unprivileged threads
 */
.thumb_func
.func rtos_enter_privileged_mode

rtos_enter_privileged_mode:
    push    {r4-r5}
    /*
     * Check if caller is an ISR:
     */
    mrs	    r4, ipsr
    mov	    r5, #CPU_REG_IPSR_EXCEPTION_NUMBER_MASK
    tst	    r4, r5
    beq	    L_do_enter_privileged_mode

    /*
     * Check if caller is a thread running in privileged mode:
     */
    mrs	    r4, control
    mov	    r5, #CPU_REG_CONTROL_nPRIV_MASK
    tst	    r4, r5
    bne	    L_do_enter_privileged_mode

    pop     {r4-r5}
    mov     r0, #1
    bx      lr

L_do_enter_privileged_mode:
    pop     {r4-r5}

    /*
     * Enter privilege mode:
     */
    svc     #RTOS_ENTER_PRIVILEGED_MODE_SVC_CODE

    /*
     * We return here in privileged mode now.
     */
    push    {lr}

    /*
     * Call rtos_k_enter_privileged_mode(RTOS_ENTER_PRIVILEGED_MODE_SVC_CODE):
     */
    mov	    r0, #RTOS_ENTER_PRIVILEGED_MODE_SVC_CODE
    bl      rtos_k_enter_privileged_mode

    mov     r0, #0
    pop     {pc}

.endfunc


/**
 * Exit privilege mode
 *
 * void
 * rtos_exit_privileged_mode(void)
 *
 * NOTE: This function is to be invoked from hardware driver functions
 * that are invoked from unprivileged threads
 */
.thumb_func
.func rtos_exit_privileged_mode

rtos_exit_privileged_mode:
    push    {lr}

    /*
     * Call rtos_k_exit_privileged_mode():
     */
    bl      rtos_k_exit_privileged_mode

    /*
     * Restore saved lr
     */
    pop	    {r0}
    mov	    lr, r0

    /*
     * Set nPRIV bit in control register to return to unprivileged mode
     */
    mrs	    r0, control
    mov	    r1, #CPU_REG_CONTROL_nPRIV_MASK
    orr	    r0, r0, r1
    msr	    control, r0
    isb

    bx	    lr
.endfunc

.end


