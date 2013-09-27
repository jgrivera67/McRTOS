/**
 * @file McRTOS_interrupt_service_routines_arm_cortex_m.s
 *
 * Machine-specific McRTOS kernel runtime exception handlers for ARM Cortex-M processors
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
.extern rtos_hard_fault_exception_handler

/*
 * Exported functions
 */
.global cortex_m_pendsv_exception_handler
.global cortex_m_hard_fault_exception_handler
.global cortex_m_save_other_registers

.text
.thumb
//.syntax unified

/*???
    .thumb_func
    .align 1
    .global  Reset_Handler
    .type    Reset_Handler, %function
*/

/*
 * PendSV exception handler. It is used for synchronous context switch.
 *
 * void
 * cortex_m_pendsv_exception_handler(void)
 *
 * @pre     This exception was triggered from thread mode.
 *
 * @param   none.
 *
 * @return  none
 */
.thumb_func
.func cortex_m_pendsv_exception_handler

cortex_m_pendsv_exception_handler:
    cpsid   i

    /*
     * Pre-saved registers were automatically saved on the stack.
     * The pre-saved registers appear in the following order in
     * memory, in the thread's stack:
     * - r0  <-- sp register in use before exception
     * - ..
     * - r3
     * - r12
     * - r14 (lr) 
     * - r15 (pc)
     * - psr
     */

    /*
     * Get stack pointer in use:
     *
     * NOTE: If PSP == NULL, this is the first context switch done from the
     * reset handler, so the stack pointer in use is MSP. Otherwise, the stack
     * pointer in use is PSP.
     */
    mrs     r0, psp
    cmp     r0, #0
    bne     L_save_other_registers
    mrs     r0, msp

L_save_other_registers:
    /*
     * r0 == stack pointer in use
     * [r0] == pre-saved r0
     * pre-saved r0 == &current_execution_context_p->ctx_cpu_saved_registers[0]
     */
    ldr r0, [r0]

    /* 
     * call cortex_m_save_other_registers(
     *          &current_execution_context_p->ctx_cpu_registers[0],
     *          LR on exception entry)
     *
     * r0 == &current_execution_context_p->ctx_cpu_registers[0]
     */
    mov     r1, lr
    bl      cortex_m_save_other_registers

    /*
     * NOTE: cortex_m_save_other_registers() only clobbered r0 and r4-r7
     * (r4-r7, after saving them).
     */

    /*
     * Call rtos_thread_scheduler()
     */
    bl       rtos_thread_scheduler

.endfunc


/*
 * Hard fault exception handler.
 *
 * void
 * cortex_m_hard_fault_exception_handler(void)
 *
 * @param   none
 *
 * @return  none
 */
.thumb_func
.func cortex_m_hard_fault_exception_handler

cortex_m_hard_fault_exception_handler:
    cpsid   i

    /*
     * Pre-saved registers were automatically saved on the stack.
     * The pre-saved registers appear in the following order in
     * memory, in the thread's stack:
     * - r0  <-- sp register in use before exception
     * - ..
     * - r3
     * - r12
     * - r14 (lr) 
     * - r15 (pc)
     * - psr
     */

    /*
     * Set r3 to
     * g_McRTOS_p->rts_cpu_controllers[0].cpc_current_execution_context_p
     */
    ldr     r3, =g_McRTOS_p /* r3 = &g_McRTOS_p */
    ldr     r3, [r3]        /* r3 = g_McRTOS_p */
    add     r3, r3, #(RTOS_RTS_CPU_CONTROLLERS_OFFSET + RTOS_CPC_CURRENT_EXECUTION_CONTEXT_P_OFFSET)

    /* 
     * Call cortex_m_save_other_registers(
     *          current_execution_context_p->ctx_cpu_registers,
     *          lr_on_exception_entry):
     *
     * NOTE: Cortex-M0+ does not support this:
     *       add     r0, r3, #RTOS_CTX_CPU_REGISTERS_OFFSET
     */
    mov     r0, r3
    add     r0, r0, #RTOS_CTX_CPU_REGISTERS_OFFSET
    mov     r1, lr
    bl      cortex_m_save_other_registers

    /*
     * NOTE: cortex_m_save_other_registers() only clobbered r0 and r4-r7
     * (r4-r7, after saving them).
     */

    /*
     * call rtos_hard_fault_exception_handler(current_execution_context_p)
     *
     * r1 == lr on exception entry
     * r3 == current_execution_context_p
     */
    mov     r0, r3
    mov     r4, r1  /* saved r1 to r4 */
    mov     r5, r3  /* saved r3 to r5 */
    bl      rtos_hard_fault_exception_handler
  
    /*
     * Return from the exception:
     *
     * NOTE: rtos_hard_fault_exception_handler() only returns here if this exception
     * was caused by a bkpt instruction
     *
     * r4 == lr on exception entry
     * r5 == current_execution_context_p
     */
    push    {r4}
    mov     r0, r5
    bl      rtos_k_restore_execution_context

    /*
     * We only come here if the exception happened while executing another exception handler:
     */
    cpsie   i
    pop     {pc}
.endfunc


/**
 * Saves registers that are not pre-saved upon exception entry.
 *
 * void
 * cortex_m_save_other_registers(
 *      cpu_register_t saved_registers[CPU_NUM_SAVED_REGISTERS],
 *      cpu_register_t lr_on_exception_entry)
 *
 * @pre     This routine is called with interrupts disabled.
 *
 * @param   saved_registers(r0): Pointer to the area of memory 
 *          where registers are to be saved.

 * @param   lr_on_exception_entry(r1): lr on exception entry
 *
 * @return  none
 *
 * CLOBBERED REGISTERS:
 * r0, r4-r7 (r4-r7, after saving them)
 */
.thumb_func
.func cortex_m_save_other_registers

cortex_m_save_other_registers:
    /* 
     * Save the non-pre-saved registers so they appear in the following
     * order in memory:
     * - r4
     * - ..
     * - r11
     * - msp
     * - psp
     * - primask
     * - control
     * - lr on exception entry
     */
    stmia   r0!, {r4-r7} /* Cortex-M0 only supports stm for r0-r7 */
    mov     r4, r8
    mov     r5, r9
    mov     r6, r10
    mov     r7, r11
    stmia   r0!, {r4-r7} /* saved r8-r11 */
    mrs     r4, msp
    mrs     r5, psp
    mrs     r6, primask
    mrs     r7, control
    stmia   r0!, {r4-r7} /* saved msp, psp, primaks, control */
    str     r1, [r0]

    bx      lr
.endfunc

.end

