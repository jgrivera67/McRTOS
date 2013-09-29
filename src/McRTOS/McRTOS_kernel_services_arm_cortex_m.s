/**
 * @file McRTOS_kernel_services_armv6_m.s
 *
 * Machine-specific McRTOS kernel services for ARM Cortex-M0+ processors
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 

#include "arm_defs.h"
#include "arm_cortex_m_macros.s"

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

/*???
    .align 1
    .global    Reset_Handler
    .type    Reset_Handler, %function
*/

/**
 * Restores the given McRTOS execution context CPU registers on the calling
 * CPU core and updates cpc_current_execution_context_p for the calling CPU.
 *
 * void
 * rtos_k_restore_execution_context(
 *      const struct rtos_execution_context *execution_context_p)
 *
 * @param   execution_context_p (r0): Pointer to the execution context to be
 *          restored.
 *
 * @return  none
 *
 * @pre     CPU interrupts are disabled and the current CPU mode is privileged
 *          thread mode or handler mode.
 *          The target mode can only be privileged or unprivileged thread mode.
 *          
 *          These assumptions are checked in fdc_trace_rtos_context_switch().
 */
.thumb_func
.func rtos_k_restore_execution_context

rtos_k_restore_execution_context:
    /*
     * r0 == execution_context_p
     *
     * NOTE: We can clobber all registers in this function, since they are going 
     * to be restored from the target context.
     */
    push {lr}

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
     * Set r4 to &execution_context_p->ctx_cpu_registers[0]
     *
     * r0 == execution_context_p
     */
    mov r4, #RTOS_CTX_CPU_REGISTERS_OFFSET
    add r4, r0, r4

#if 0 // ??? this code is broken: Need a reliable to determine if target context was 
      // ??? running with interrupts enabled or disabled (perhaps using LR on excpetion entry)
#ifdef _CPU_CYCLES_MEASURE_
    /*
     * r4 == &execution_context_p->ctx_cpu_registers[0]
     *
     * If target context's PRIMASK register has interrupts enabled, call
     * rtos_stop_interrupts_disabled_time_measure(), preserving r0:
     *
     * r0 = rtos_stop_interrupts_disabled_time_measure(r0);
     *
     * NOTE:
     * - It is assumed that earlier in the code path that lead us here, we made 
     *   a call to rtos_start_interrupts_disabled_time_measure() via the macro
     *   RTOS_START_INTERRUPTS_DISABLED_TIME_MEASURE().
     */
    mov     r1, #CPU_REG_PRIMASK_PM_MASK 
    tst     r0, r1
    bne     1f

    /*
     * Interrupts are enabled in the target context
     */
    bl      rtos_stop_interrupts_disabled_time_measure
1:
#endif /* _CPU_CYCLES_MEASURE_ */
#endif // #if 0

    /*
     * Restore explicitly-saved registers r4-r11:
     *
     * r4 == &execution_context_p->ctx_cpu_registers[0]
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
    ldmia   r1!, {r4-r7} /* Cortex-M0 only supports ldm for r0-r7 */

    /*
     * NOTE: Above, we use "ldmia r1!" instead of "ldmia r1" because for
     * Cortex-M r1 is incremented with or without the "!".
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
     *
     * Get execution_context_p->ctx_cpu_mode:
     *
     * r2 == execution_context_p
     */
    ldrb    r1, [r2, #RTOS_CTX_CPU_MODE_OFFSET]

    /*
     * Determine if the context to be restored is an interrupt or a
     * thread:
     *
     * r1 == execution_context_p->ctx_cpu_mode
     */
    cmp     r1, #RTOS_INTERRUPT_MODE
    beq     L_target_context_is_interrupt

    /*
     * Target context is a thread, so we need to restore the PSP stack pointer:
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

#ifdef DEBUG
    /*
     * This function is supposed to be invoked only from an exception handler
     * (an ISR or the PendSV exception handler).
     *
     * Check that the current CPU mode is handler mode
     */
    mrs     r1, ipsr
    mov     r0, #CPU_REG_IPSR_EXCEPTION_NUMBER_MASK
    tst     r1, r0
    bne     1f
    ldr     r0, =L_rtos_k_restore_execution_context_assert_cond_str
    mov     r2, #0
    bl      capture_assert_failure
1:
    /*
     * Current CPU mode is not thread mode (exception number field > 0)
     */
#endif

    /*
     * Return from exception to thread:
     */
    ldr     r0, =CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP
    mov     lr, r0
    cpsie   i
    bx      lr
   
    /*
     * Target context is an interrupt context, so return from nested exception:
     *
     * NOTE: We need to return to the caller which is either 
     * cortex_m_hard_fault_exception_handler() or rtos_k_exit_interrupt().
     * rtos_k_exit_interrupt() will return to the corresponding ISR, so that the
     * exception stack drains by itself. In that case, we re-enable interrupts in
     * RTOS_EXIT_ISR(), before returning from the exception.
     */
L_target_context_is_interrupt:
    pop     {pc}

L_rtos_k_restore_execution_context_assert_cond_str:
.asciz "(ipsr & CPU_REG_IPSR_EXCEPTION_NUMBER_MASK) != 0"

.balign 4

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
     * Set r0 to &current_execution_context_p->ctx_cpu_registers[0]:
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
     * r0 == &current_execution_context_p->ctx_cpu_registers[0]
     */
    ldr     r1, =CPU_SCB_ICSR_REGISTER_ADDR
    ldr     r2, [r1]
    ldr     r3, =CPU_SCB_ICSR_PENDSVSET_MASK
    orr     r2, r2, r3
    str     r2, [r1]

    /*
     * Enable interrupts to take the PendSV exception:
     */
    cpsie    i

    /*
     * When the thread being switched out resumes, it will 
     * resume here, and we need disable interrupts again as that
     * is expected by the callers of this function:
     */
    cpsid   i
    bx      lr 

.endfunc

.end


