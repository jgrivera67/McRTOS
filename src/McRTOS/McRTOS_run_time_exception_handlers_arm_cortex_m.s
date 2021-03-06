/**
 * @file McRTOS_interrupt_service_routines_arm_cortex_m.s
 *
 * Machine-specific McRTOS kernel runtime exception handlers for ARM Cortex-M processors
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */

#include <McRTOS/McRTOS_arm_cortex_m_macros.s>

/*
 * Imported symbols
 */
.extern rtos_hard_fault_exception_handler
.extern rtos_start_interrupts_disabled_time_measure

/*
 * Exported functions
 */
.global cortex_m_pendsv_exception_handler
.global cortex_m_hard_fault_exception_handler
.global cortex_m_memory_management_exception_handler
.global cortex_m_bus_fault_exception_handler
.global cortex_m_usage_fault_exception_handler
.global cortex_m_debug_monitor_exception_handler
.global cortex_m_save_other_registers

.text
.thumb
#ifndef __ARM_ARCH_6M__
.syntax unified
#endif

/*
 * PendSV exception handler. It is used for synchronous context switch.
 *
 * void
 * cortex_m_pendsv_exception_handler(void)
 *
 * @pre     This exception was triggered from thread mode.
 * @pre     This exception is configured to have higher priority than all
 *          interrupts. So no nested exceptions caused by interrupts are
 *          expected here.
 *
 * @param   none.
 *
 * @return  none
 */
.thumb_func
.func cortex_m_pendsv_exception_handler

cortex_m_pendsv_exception_handler:
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
     * Disable interrupts in the CPU, to ensure atomic saving of the
     * rest of the current context
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

    /*
     * Get stack pointer in use:
     *
     * NOTE: If PSP == 0x0, this is the first context switch done from the
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
     * pre-saved r0 == &current_execution_context_p->ctx_cpu_saved_registers
     */
    ldr r0, [r0]

    /*
     * call cortex_m_save_other_registers(
     *          &current_execution_context_p->ctx_cpu_saved_registers,
     *          LR on exception entry)
     *
     * r0 == &current_execution_context_p->ctx_cpu_saved_registers
     */
    mov     r1, lr
    bl      cortex_m_save_other_registers

    /*
     * NOTE: cortex_m_save_other_registers() only clobbered r0 and r4-r7
     * (r4-r7, after saving them).
     */

    /*
     * Branch to rtos_thread_scheduler()
     *
     * NOTE: For Cortex-M0+ se have to use 'bl', as 'b' does not have enough
     * range
     */
    mrs     r0, psp
    cmp     r0, #0
    bne     1f
    mov     r0, #RTOS_CSW_RESET_TO_THREAD
    b       2f
1:
    mov     r0, #RTOS_CSW_THREAD_TO_THREAD

2:
    bl       rtos_thread_scheduler

    /*
     * We should never come back here:
     */
    bkpt    #0

.endfunc


/**
 * Macro that generates a fault exception handler
 *
 * @param   _exception_vector_: Exception vector number
 *
 * @param   _exception_handler_func_: Exception handler function name
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
.macro GEN_FAULT_EXCEPTION_HANDLER_FUNCTION _exception_handler_func_, \
					    _exception_vector_

/**
 * Fault exception handler for exception \_exception_vector_.
 *
 * void \_exception_handler_func_(void)
 *
 * @pre     This exception has higher priority than all interrupts.
 *          So no nested exceptions caused by interrupts are expected here.
 *
 * @param   none
 *
 * @return  none
 */
.global \_exception_handler_func_

.thumb_func
.func \_exception_handler_func_

\_exception_handler_func_:
    cpsid   i
    isb

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
     * Set r2 to
     * g_McRTOS_p->rts_cpu_controllers[0].cpc_current_execution_context_p
     */
    GET_MCRTOS_CURRENT_EXECUTION_CONTEXT r2

    /*
     * Call cortex_m_save_other_registers(
     *          &current_execution_context_p->ctx_cpu_saved_registers,
     *          lr_on_exception_entry):
     *
     * NOTE: Cortex-M0+ does not support this:
     *       add     r0, r2, #RTOS_CTX_CPU_REGISTERS_OFFSET
     */
    mov     r0, r2
    add     r0, r0, #RTOS_CTX_CPU_REGISTERS_OFFSET
    mov     r1, lr
    bl      cortex_m_save_other_registers

    /*
     * NOTE: cortex_m_save_other_registers() only clobbered r0 and r4-r7
     * (r4-r7, after saving them). From this point on we can clobber all
     * registers.
     */

    /*
     * call rtos_hard_fault_exception_handler(current_execution_context_p)
     *
     * r2 == current_execution_context_p
     */
    mov     r0, r2
    mov     r4, r2  /* saved r2 to r4 */
    mov     r5, r1  /* saved r1 to r5 */
    mov	    r1, #\_exception_vector_
    bl      rtos_common_fault_exception_handler

    /*
     * Return from the exception:
     *
     * NOTE: rtos_common_fault_exception_handler() only returns here if
     * fdc_exception_debugger_on is set.
     *
     * r4 == current_execution_context_p
     */
    mov     r0, r4
    mov     r1, #RTOS_CSW_EXITING_HARD_FAULT
    bl      rtos_k_restore_execution_context

    /*
     * We should never come back here
     */
    bkpt    #0

.endfunc

.endm


GEN_FAULT_EXCEPTION_HANDLER_FUNCTION cortex_m_hard_fault_exception_handler, \
				     INT_HARD_FAULT

GEN_FAULT_EXCEPTION_HANDLER_FUNCTION cortex_m_memory_management_exception_handler, \
				     INT_MEMORY_MANAGEMENT

GEN_FAULT_EXCEPTION_HANDLER_FUNCTION cortex_m_bus_fault_exception_handler, \
				     INT_BUS_FAULT

GEN_FAULT_EXCEPTION_HANDLER_FUNCTION cortex_m_usage_fault_exception_handler, \
				     INT_USAGE_FAULT

GEN_FAULT_EXCEPTION_HANDLER_FUNCTION cortex_m_debug_monitor_exception_handler, \
				     INT_DEBUG_MONITOR


/**
 * Saves registers that are not pre-saved upon exception entry.
 *
 * void
 * cortex_m_save_other_registers(
 *      struct cpu_saved_registers *saved_registers_p,
 *      cpu_register_t lr_on_exception_entry)
 *
 * @pre     This routine is called with interrupts disabled.
 *
 * @param   saved_registers_p(r0): Pointer to the area of memory
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
     * - lr on exception entry
     * - control
     */
    stmia   r0!, {r4-r7}    /* Cortex-M0 only supports stm for r0-r7 */
    mov     r4, r8
    mov     r5, r9
    mov     r6, r10
    mov     r7, r11
    stmia   r0!, {r4-r7}    /* saved r8-r11 */
    mrs     r4, msp
    mrs     r5, psp
    stmia   r0!, {r4-r5}    /* saved msp, psp */
    str     r1, [r0]        /* saved lr_on_exception_entry */
    mrs     r4, control
    str	    r4, [r0, #ARM_CPU_WORD_SIZE_IN_BYTES]

    bx      lr

.endfunc

.end

