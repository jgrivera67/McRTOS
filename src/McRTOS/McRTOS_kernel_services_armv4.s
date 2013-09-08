/**
 * @file McRTOS_kernel_services_arm_v4.s
 *
 * McRTOS CPU-specific kernel services for ARM v4 processors
 *
 * @author German Rivera 
 */ 

#include "arm_defs.h"

.global rtos_k_disable_cpu_interrupts
.global rtos_k_restore_cpu_interrupts
.global rtos_k_atomic_fetch_add_uint32
.global rtos_k_atomic_fetch_sub_uint32
.global rtos_k_find_highest_thread_priority
.global rtos_k_restore_execution_context
.global rtos_k_synchronous_context_switch

.extern g_rtos_atomic_ops_spinlock
.extern rtos_start_interrupts_disabled_time_measure
.extern rtos_stop_interrupts_disabled_time_measure
.extern capture_assert_failure
.extern capture_fdc_failure
.extern fdc_fatal_error

.text
.arm

/**
 * Macro that disables IRQ and FIQ interrupts and saves the previous CPSR value
 *
 * INPUT REGISTERS:
 * none
 *
 * OUTPUT REGISTERS:
 * - r0 = original value of the CPSR register
 *
 * CLOBBERED REGISTERS:
 * r1
 */
.macro DISABLE_CPU_INTERRUPTS
    mrs     r0, cpsr

1:
    /*
     * Set IRQ & FIQ bits in CPSR to DISABLE all interrupts
     */
    orr     r1, r0, #ARM_INTERRUPTS_DISABLED_MASK
    msr     cpsr_c, r1
    
    /*
     * Confirm that CPSR contains the proper interrupt disable flags
     */
    mrs     r1, cpsr
    and     r1, r1, #ARM_INTERRUPTS_DISABLED_MASK
    cmp     r1, #ARM_INTERRUPTS_DISABLED_MASK
    bne     1b
.endm


/**
 * Macro that performs a given atomic operation on a 32-bit memory location.
 *
 * @param _op_  Mnemonic of an arithmetic/logic ARM instruction
 *
 * INPUT REGISTERS:
 * - r0 = pointer to the variable to be atomically modified
 * - r1 = right operand of the atomic operation
 *
 * OUTPUT REGISTERS:
 * - r0 = original value of the variable that was atomically modified
 *
 * CLOBBERED REGISTERS:
 * r2, r3, r12
 */
.macro ATOMIC_FETCH_OP _op_
    /*
     * Save input args, as r0 and r1 are modified by DISABLE_CPU_INTERRUPTS:
     */
    mov     r2, r0
    mov     r3, r1

    DISABLE_CPU_INTERRUPTS

    /*
     * r0 = original CPSR
     */

#if SOC_NUM_CPU_CORES > 1
    /*
     * Acquire SWP-based spinlock, to ensure inter-CPU atomicity:
     */
    ldr     r12, =g_rtos_atomic_ops_spinlock
    mov     r1, #0x1
1:
    swp     r1, r1, [r12]
    cmp     r1, #0x1
    beq     1b
#endif

    /*
     * r2 = counter_p
     * r3 = operand value
     */
    ldr     r1, [r2]
    \_op_   r3, r1, r3
    str     r3, [r2]

    /*
     * r1 = original value of the 32-bit memory location
     */

#if SOC_NUM_CPU_CORES > 1
    /*
     * Release SWP-based spinlock:
     * r12 = address of g_atomic_ops_spinlock
     */
    mov     r2, #0
    str     r2, [r12]
#endif

    /*
     * Restore CPSR
     */
    msr     cpsr_c, r0

    mov     r0, r1
.endm


/**
 * Function that disables IRQ and FIQ interrupts and returns the previous IRQ/FIQ mask
 *
 * cpu_status_register_t rtos_k_disable_cpu_interrupts(void)
 *
 * @return  Original value of the CPU status register
 */
.func rtos_k_disable_cpu_interrupts

rtos_k_disable_cpu_interrupts:
    DISABLE_CPU_INTERRUPTS

#ifdef _CPU_CYCLES_MEASURE_
    /*
     * Call rtos_start_interrupts_disabled_time_measure as follows, to 
     * preserve r0:
     *
     * r0 = rtos_start_interrupts_disabled_time_measure(r0)
     */
    push    {lr}
    bl      rtos_start_interrupts_disabled_time_measure
    pop     {lr}
#endif /* _CPU_CYCLES_MEASURE_ */

    /*
     * Return original CSPR:
     *
     * r0 == original CPSR
     */
    bx      lr

.endfunc


/**
 * Function that restores (and possibly enables) IRQ and FIQ interrupts
 *
 * void
 * rtos_k_restore_cpu_interrupts(cpu_register_t cpu_status_register)
 *
 * @param   cpu_status_register (r0): Value of the CPU status register to be
 *          restored
 */
.func rtos_k_restore_cpu_interrupts

rtos_k_restore_cpu_interrupts:

#ifdef _CPU_CYCLES_MEASURE_
    /*
     * Call rtos_stop_interrupts_disabled_time_measure as follows, to 
     * preserve r0:
     *
     * r0 = rtos_stop_interrupts_disabled_time_measure(r0)
     */
    push    {lr}
    bl      rtos_stop_interrupts_disabled_time_measure
    pop     {lr}
#endif /* _CPU_CYCLES_MEASURE_ */

    /*
     * Restore saved CPSR
     */
    msr     cpsr_c, r0
    bx      lr

.endfunc


/**
 * Increments atomically the value stored in *counter_p, and returns the
 * original value.
 *
 * uint32_t
 * rtos_k_atomic_fetch_add_uint32(volatile uint32_t *counter_p, uint32_t value)
 *
 * @param   counter_p (r0): Pointer to the counter to be incremented.
 *
 * @param   counter_p (r1): Increment value.
 *
 * @return  value of the counter prior to the increment.
 *
 * For ARMv4:
 * Since  ARM v4 does not have LDRX/STRX, we have to disable/enable interrupts.
 * We cannot use SWP with a global flag spinlock for this, as that can only
 * work for threads not ISRs. If the spinlock is held by a thread and  an ISR
 * interrupts the thread, the ISR will deadlock spinning trying to get the
 * spinlock.
 *
 */
.func rtos_k_atomic_fetch_add_uint32

rtos_k_atomic_fetch_add_uint32:
    ATOMIC_FETCH_OP add

    /*
     * r0 = original value of *counter_p
     */
    bx      lr

.endfunc


/**
 * Decrements atomically the value stored in *counter_p, and returns the
 * original value.
 *
 * uint32_t
 * rtos_atomic_fetch_sub_uint32(volatile uint32_t *counter_p, uint32_t value)
 *
 * @param   counter_p (r0): Pointer to the counter to be decremented.
 *
 * @param   counter_p (r1): Decrement value.
 *
 * @return  value of the counter prior to the decrement.
 *
 * For ARMv4:
 * Since  ARM v4 does not have LDRX/STRX, we have to disable/enable interrupts.
 * We cannot use SWP with a global flag spinlock for this, as that can only
 * work for threads not ISRs. If the spinlock is held by a thread and  an ISR
 * interrupts the thread, the ISR will deadlock spinning trying to get the
 * spinlock.
 */
.func rtos_k_atomic_fetch_sub_uint32

rtos_k_atomic_fetch_sub_uint32:
    ATOMIC_FETCH_OP sub

    /*
     * r0 = original value of *counter_p
     */
    bx      lr
.endfunc


/**
 * Find the highest thread priority bit set in the passed in thread priority
 * bitmap.
 *
 * rtos_thread_prio_t
 * rtos_k_find_highest_thread_priority(
 *      rtos_thread_prio_bitmap_t rtos_thread_prio_bitmap)
 *
 * @param   rtos_thread_prio_bitmap (r0): Thread priority bitmap
 *
 * @pre     rtos_thread_prio_bitmap is 32-bits long.
 *
 * @return  Highest thread priority whose bit is set in rtos_thread_prio_bitmap,
 *          if at least one bit is set in rtos_thread_prio_bitmap.
 *          If bit 31 in rtos_thread_prio_bitmap is set, this function 
 *          returns 0.
 *          If bit 0 in rtos_thread_prio_bitmap is set, this function
 *          returns 31.
 *
 * @return  ARM_CPU_WORD_SIZE_IN_BITS, if no bit is set in
 *          rtos_thread_prio_bitmap.
 *
 * For ARMv4:
 * Since  ARM v4 does not have the CLZ instruction, we have to manually emulate
 * the behavior of CLZ. The algorithm used here was taken from section 7.2.2 of
 * the ARM System Developer's Guide book.
 */
.func rtos_k_find_highest_thread_priority

rtos_k_find_highest_thread_priority:
    
    /*
     * r0 == rtos_thread_prio_bitmap
     */

    /*
     * r1 will be used as the leading zeros counter
     */
    mov     r1, #0

    /*
     * if (r0 < (1 << 16))
     * {
     *     r0 <<= 16;
     *     r1 += 16;
     * }
     */
    cmp     r0, #(1 << 16)
    movcc   r0, r0, LSL #16
    addcc   r1, r1, #16

    /*
     * if (r0 < (1 << 24))
     * {
     *     r0 <<= 8;
     *     r1 += 8;
     * }
     */
    tst     r0, #0xFF000000
    moveq   r0, r0, LSL #8
    addeq   r1, r1, #8

    /*
     * if (r0 < (1 << 28))
     * {
     *     r0 <<= 4;
     *     r1 += 4;
     * }
     */
    tst     r0, #0xF0000000
    moveq   r0, r0, LSL #4
    addeq   r1, r1, #4

    /*
     * if (r0 < (1 << 30))
     * {
     *     r0 <<= 2;
     *     r1 += 2;
     * }
     */
    tst     r0, #0xC0000000
    moveq   r0, r0, LSL #2
    addeq   r1, r1, #2

    /*
     * if (r0 < (1 << 31))
     * {
     *     r1 += 1;
     *     r0 <<= 1; 
     *     if (r0 == 0)
     *     {
     *         r1 = ARM_CPU_WORD_SIZE_IN_BITS
     *     }
     * }
     */
    tst     r0, #0x80000000
    addeq   r1, r1, #1
    moveqs  r0, r0, LSL #1
    moveq   r1, #ARM_CPU_WORD_SIZE_IN_BITS

    mov     r0, r1
    bx      lr

.endfunc


/**
 * Synchronous context switch. This function saves the current execution
 * context's CPU registers and calls the thread scheduler. This function
 * is invoked from rtos_k_condvar_wait() and rtos_k_thread_yield, from
 * rtos_k_mutex_acquire() and rtos_k_condvar_wait_interrupt() if the
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
 * @pre     This routine is called from from System mode, and
 *          with interrupts disabled.
 *
 * @param   current_execution_context_p (r0): Pointer to the current execution
 *          context.
 *
 * @return  none
 */
.func rtos_k_synchronous_context_switch

rtos_k_synchronous_context_switch:
    /*
     * Set r0 to &current_execution_context_p->ctx_cpu_registers[0]:
     *
     * r0 == current_execution_context_p
     *
     * NOTE: Since this is a synchronous context switch, we don't need to
     * preserve r0-r3, as they are caller-saved registers.
     */
    add     r0, r0, #RTOS_CTX_CPU_REGISTERS_OFFSET

#ifdef _RELIABILITY_CHECKS_
    /*
     * Save r0 and lr on the stack
     */
    push    {r0, lr}

    /*
     * Check that we are called from a privileged thread and with interrupts
     * disabled:
     */
    mrs     r0, cpsr
    and     r0, r0, #(ARM_MODE_MASK | ARM_INTERRUPTS_DISABLED_MASK)
    teq     r0, #(ARM_MODE_SYS | ARM_INTERRUPTS_DISABLED_MASK)
    ldrne   r0, =L_rtos_k_synchronous_context_switch_assert_cond_str
    mrsne   r1, cpsr
    movne   r2, #0
    blne    capture_assert_failure

    /*
     * Restore r0 and lr
     */
    pop     {r0, lr}
#endif /* _RELIABILITY_CHECKS_ */

    /*
     * Since this is a synchronous context switch we don't need to save
     * r0-r3 (it is OK that they will be restored as garbage). Save CPU 
     * registers to current_execution_context_p->ctx_cpu_registers,
     * so the appear in the following order in memory:
     * - r0
     * - ..
     * - r12
     * - r13 (sp)
     * - r14 (lr) 
     * - r15 (pc) - return address
     * - cpsr
     *
     * r0 == &current_execution_context_p->ctx_cpu_registers[0]
     */

    /* 
     * Save r4-r14:
     */
    add     r0, r0, #(CPU_REG_R4 * ARM_CPU_WORD_SIZE_IN_BYTES)
    stmia   r0!, {r4-r14}

    /* 
     * Save cpsr and pc:
     *
     * r0 == &current_execution_context_p->ctx_cpu_registers[CPU_REG_PC]
     *
     * NOTE: We use the return address as the saved pc
     */
    mov     r1, lr
    mrs     r2, cpsr
    stmia   r0, {r1-r2}

    /*
     * Branch to rtos_thread_scheduler(), instead of calling it, as we
     * will not come back here.
     */
    b      rtos_thread_scheduler

L_rtos_k_synchronous_context_switch_assert_cond_str:
.asciz "(cpsr & (ARM_MODE_SYS | ARM_INTERRUPTS_DISABLED_MASK)) == (ARM_MODE_SYS | ARM_INTERRUPTS_DISABLED_MASK)"

.balign 4

.endfunc


/**
 * Restores the given McRTOS execution context CPU registers on the calling
 * CPU core.
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
 * @pre     CPU interrupts are disabled and the current CPU mode is one of
 *          the modes: SYS, IRQ or SVC.
 *          The target mode can only be SYS or USER.
 *          
 *          These assumptions are checked in fdc_trace_rtos_context_switch().
 */
.func rtos_k_restore_execution_context

rtos_k_restore_execution_context:
    /*
     * r0 == execution_context_p
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
     * Set r14 to &execution_context_p->ctx_cpu_registers[0]
     *
     * r0 == execution_context_p
     */
    add r14, r0, #RTOS_CTX_CPU_REGISTERS_OFFSET

    /*
     * Retrieve target context's CSPR 
     */
    ldr    r0, [r14, #(CPU_REG_CPSR * ARM_CPU_WORD_SIZE_IN_BYTES)]

    /*
     * r0 == execution_context_p->ctx_cpu_registers[CPU_REG_CPSR]
     */

#ifdef _CPU_CYCLES_MEASURE_
    /*
     * If target context's CPSR has interrupts enabled, call
     * rtos_stop_interrupts_disabled_time_measure(), preserving r0:
     *
     * r0 = rtos_stop_interrupts_disabled_time_measure(r0);
     *
     * NOTE: It is assumed that earlier in the code path that lead us here, we made 
     * a call to rtos_start_interrupts_disabled_time_measure() via the macro
     * RTOS_START_INTERRUPTS_DISABLED_TIME_MEASURE().
     *
     * NOTE: we need to save r14 in r4 because r14 will get clobbered by calling
     * rtos_stop_interrupts_disabled_time_measure(). We can clobber r4 and lr
     * because we are going to restore them below from the target context.
     */
    tst     r0, #ARM_INTERRUPTS_DISABLED_MASK
    moveq   r4, r14
    bleq    rtos_stop_interrupts_disabled_time_measure

    /*
     * The Z flag may have been clobbered by the call above, so we need to do
     * the tst again
     */
    tst     r0, #ARM_INTERRUPTS_DISABLED_MASK
    moveq   r14, r4
#endif /* _CPU_CYCLES_MEASURE_ */

    /*
     * if (current ARM mode == target ARM mode)
     * {
     *      goto L_restore_cpu_registers_same_mode
     * }
     * else
     * {
     *      if (current ARM mode == ARM_MODE_IRQ ||
     *          current ARM mode == ARM_MODE_SVC)
     *      {
     *          reset current ARM mode's SP   
     *      }
     * }
     *
     * r0 == execution_context_p->ctx_cpu_registers[CPU_REG_CPSR]
     * r14 == &execution_context_p->ctx_cpu_registers[CPU_REG_R0]
     */
    and     r2, r0, #ARM_MODE_MASK
    mrs     r3, cpsr
    and     r3, r3, #ARM_MODE_MASK
    teq     r2, r3                                  // flags changed
    beq     L_restore_cpu_registers_same_mode

    teq     r3, #ARM_MODE_IRQ                       // flags changed
    ldreq   sp, =__stack_irq_end

    teqne   r3, #ARM_MODE_SVC                       // flags changed
    ldreq   sp, =__stack_svc_end

    /*
     * If current mode is ARM_MODE_SYS we need to switch to SVC mode (keeping
     * interrupts disabled) to borrow its spsr:
     */
    teq     r3, #ARM_MODE_SYS
    moveq   r4, r14
    msreq   cpsr_c, #(ARM_MODE_SVC | ARM_INTERRUPTS_DISABLED_MASK)
    moveq   r14, r4

    /*
     * Restore target mode cpsr to current mode's spsr
     *
     * r0 == execution_context_p->ctx_cpu_registers[CPU_REG_CPSR]
     *
     * NOTE: Since target mode is different from current mode, current mode has
     * to be ARM_MODE_IRQ or  ARM_MODE_SVC
     */
    msr     spsr_cxsf, r0

    /*
     * Restore user/system mode registers r0-r14:
     *
     * r14 == &execution_context_p->ctx_cpu_registers[CPU_REG_R0]
     */
    ldmia   r14, {r0-r14}^

    /*
     * Restore target context's pc and cpsr atomically:
     *
     * r14 == &execution_context_p->ctx_cpu_registers[CPU_REG_R0]
     *
     * NOTE: the current mode's r14 was not modified by the ldmia^
     * instruction above. The target mode's r14 was the one that
     * got changed.
     */
    ldr     r14, [r14, #(CPU_REG_PC * ARM_CPU_WORD_SIZE_IN_BYTES)]
    movs    pc, r14

L_restore_cpu_registers_same_mode:
    /*
     * Restore CPU registers, when staying in the same mode:
     *
     * r0 == execution_context_p->ctx_cpu_registers[CPU_REG_CPSR]
     * r14 == &execution_context_p->ctx_cpu_registers[CPU_REG_R0]
     * r2 == target ARM mode
     *
     * NOTE: If the target context's cpsr has interrupts enabled, we
     * can get interrupted right after restoring the CPSR, but that
     * is not a problem as we are not changing ARM modes.
     */
    msr     cpsr_cxsf, r0
    ldmia   r14, {r0-r14, pc}

.endfunc

.end
