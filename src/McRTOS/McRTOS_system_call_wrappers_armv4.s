/**
 * @file McRTOS_system_call_wrappers_armv4.s
 *
 * McRTOS System call wrapper functions, to invoke system calls from C code.
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */

#include "arm_defs.h"

.extern capture_fdc_error
.extern g_rtos_system_call_dispatch_table
.extern rtos_k_enter_system_call
.extern rtos_k_exit_system_call

.text
.arm

/**
 * Macro that generates a system call wrapper function
 *
 * @param _system_call_number_ System call number
 *
 * @param _rtos_function_suffix_  function name suffix
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
.macro GEN_SYSTEM_CALL_WRAPPER_FUNCTION _system_call_number_, _rtos_function_suffix_

/**
 * System call wrapper rtos_\_rtos_function_suffix
 *
 * @param   r0-r3: system call input parameters
 *
 * @return  r0: system call return code
 */
.global rtos_\_rtos_function_suffix_

.func rtos_\_rtos_function_suffix_

rtos_\_rtos_function_suffix_:
    /*
     * Save r4, lr on the stack:
     */
    stmdb   sp!, {r4, lr}

    /*
     * If current CPU mode is unprivileged mode, execute an swi instruction
     * to perform the system call. Otherwise, if the mode is privileged thread
     * mode, interrupt mode or supervisor mode (caller is rtos_startup()),
     * call the corresponding kernel service directly.
     * Otherwise, reject the system call as it is being invoked in an unexpected
     * CPU mode or with interrupts disabled.
     */
    mrs     r4, cpsr
    and     r4, r4, #ARM_MODE_MASK
    teq     r4, #ARM_MODE_USER
    beq     L_system_call_do_swi_\_rtos_function_suffix_
    teq     r4, #ARM_MODE_SYS
    teqne   r4, #ARM_MODE_IRQ
    teqne   r4, #ARM_MODE_SVC
    bne     L_system_call_error_\_rtos_function_suffix_

    /*
     * Call the corresponding kernel service directly:
     */
    bl      rtos_k_\_rtos_function_suffix_

L_system_call_exit_\_rtos_function_suffix_:
    /*
     * r0 = system call return code, if any.
     */

    /*
     * Restore r4 and return to the saved lr:
     */
    ldmia   sp!, {r4, pc}

L_system_call_do_swi_\_rtos_function_suffix_:
   /*
    * Check that interrupts are enabled
    */
    mrs     r4, cpsr
    tst     r4, #ARM_INTERRUPTS_DISABLED_MASK
    bne     L_system_call_error_\_rtos_function_suffix_

    /*
     * Perform system call:
     */
    swi     #\_system_call_number_
    b       L_system_call_exit_\_rtos_function_suffix_

L_system_call_error_\_rtos_function_suffix_:
    ldr     r0, =L_system_call_error_msg_\_rtos_function_suffix_
    mov     r1, r4
    mov     r2, #0
    bl      capture_fdc_error
    b       L_system_call_exit_\_rtos_function_suffix_
.endfunc

L_system_call_error_msg_\_rtos_function_suffix_:
.asciz "System call rtos_\_rtos_function_suffix_ failed"
.balign 4

.endm


/**
 * SWI instruction exception handler
 *
 * @pre: CPU interrupts are disabled and current CPU mode is ARM_MODE_SVC
 */
.global rtos_swi_exception_handler

.func rtos_swi_exception_handler

rtos_swi_exception_handler:
    /*
     * - r0-r3 == system call parameters
     * - lr == address where we want to return after the exception
     */

    /*
     * Save r0 on the SVC mode's stack, so that we can use them as scratch
     * registers here:
     */
    push    {r0}

    /*
     * Save the user mode's sp on the SVC mode's stack, and then pop it
     * out of the stack into r0:
     *
     * NOTE: we don't need to update the SVC mode's sp.
     */
    stmdb   sp, {r13}^
    ldmdb   sp, {r0}

    /*
     * Save callee-saved registers on the user mode's stack:
     *
     * r0 == user mode's sp
     */
    stmdb   r0!, {r4-r12}

    /*
     * Save SVC mode's lr and spsr on the user mode's stack:
     *
     * r0 == user mode's sp
     *
     * NOTES:
     * - Both the lr and spsr from SVC mode are needed so that we can return
     *   correctly from the SWI exception.
     * - The SVC mode's spsr and lr could be modified under us, as we are enabling
     *   interrupts below, and thus another thread can preempt us and make another
     *   system call, causing a nested SWI exception.
     * - We can now clobber the callee-saved registers.
     */
    mov     r4, lr
    mrs     r5, spsr
    stmdb   r0!, {r4, r5}

    /*
     * Move user mode's sp from r0 to r5, so that we can restore r0:
     */
     mov    r5, r0

    /*
     * Restore r0 from the SVC-mode's stack:
     */
    pop     {r0}

    /*
     * Switch to System mode, keeping interrupts disabled:
     */
    msr    cpsr_c, #(ARM_MODE_SYS | ARM_INTERRUPTS_DISABLED_MASK)

    /*
     * Update the user mode's sp to account for data saved on the user mode's stack
     * while we were in SVC mode:
     */
    mov     sp, r5

    /*
     * Re-enable interrupts (by turning off I_BIT and F_BIT in the cpsr), staying in
     * system mode.
     *
     * NOTE: This is necessary so that higher priority threads and interrupts
     * can preempt us.
     *
     * r0-r3 == parameters of the invoked system call
     * r4 == SVC mode's lr
     */
    msr     cpsr_c, #ARM_MODE_SYS

    /*
     * Retrieve the system call number from the immediate operand of the
     * calling SWI instruction
     *
     * - r4 == SVC mode's lr
     * - r4 - 4 == address of calling SWI instruction
     */
    sub     r4, r4, #4

    ldr     r4, [r4]
    and     r4, r4, #ARM_SWI_IMMEDIATE_OPERAND_MASK

    /*
     * Call rtos_k_enter_system_call(r4)
     *
     * r4 == system call number
     *
     * NOTE: We need to save r0-r3 into r5-r8 before making this call,
     * as they contain the system call arguments
     */
    mov     r5, r0
    mov     r6, r1
    mov     r7, r2
    mov     r8, r3
    mov     r0, r4
    bl      rtos_k_enter_system_call
    cmp     r0, #0
    bne     L_swi_exception_handler_exit

    /*
     * Restore system call parameters from r5-r8 to r0-r3:
     */
    mov     r0, r5
    mov     r1, r6
    mov     r2, r7
    mov     r3, r8

    /*
     * Retrieve the function pointer for the kernel service to be invoked:
     *
     * r4 == system call number
     */
    ldr     r5, =g_rtos_system_call_dispatch_table
    ldr     r4, [r5, r4, LSL #2]

    /*
     * Call the function pointer in r4:
     */
    mov     lr, pc
    bx      r4

    /*
     * Call rtos_k_exit_system_call()
     *
     * NOTE: We need to save r0 before making this call,
     * as it may contain the value to return from the system call
     */

    mov     r4, r0
    bl      rtos_k_exit_system_call
    mov     r0, r4

L_swi_exception_handler_exit:
    /*
     * r0 == fdc_error_t value to return
     * r1-r3 can be used as scratch registers
     */

    /*
     * Disable interrupts:
     */
1:
    msr    cpsr_c, #(ARM_MODE_SYS | ARM_INTERRUPTS_DISABLED_MASK)
    mrs     r1, cpsr
    tst     r1, #ARM_INTERRUPTS_DISABLED_MASK
    beq     1b

    /*
     * Pop the SVC mode's lr and spsr from the user mode's stack into r1, r2:
     */
    pop    {r1, r2}

    /*
     * Restore callee-saved registers from the user mode's stack:
     */
    pop     {r4-r12}

    /*
     * Switch back to Supervisor mode (keeping interrupts disabled) to set
     * the SVC mode's lr and spsr, and return from the SWI exception:
     *
     * r0 == fdc_error_t value to return
     * r1 == SVC mode's lr
     * r2 == SVC mode's spsr
     * r4-r12 == restored to values prior to the system call
     */
    msr     cpsr_c, #(ARM_MODE_SVC | ARM_INTERRUPTS_DISABLED_MASK)
    mov     lr, r1
    msr     spsr_cxsf, r2
    movs    pc, lr

.endfunc

/*
 * System call wrapper functions
 */

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_INIT_SYSTEM_CALL, thread_init
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_MUTEX_INIT_SYSTEM_CALL, mutex_init
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONDVAR_INIT_SYSTEM_CALL, condvar_init
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_TIMER_INIT_SYSTEM_CALL, timer_init
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_DELAY_SYSTEM_CALL, thread_delay
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_ABORT_SYSTEM_CALL, thread_abort
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_CONDVAR_WAIT_SYSTEM_CALL, thread_condvar_wait
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_CONDVAR_SIGNAL_SYSTEM_CALL, thread_condvar_signal
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_MUTEX_ACQUIRE_SYSTEM_CALL, mutex_acquire
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_MUTEX_RELEASE_SYSTEM_CALL, mutex_release
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONDVAR_WAIT_SYSTEM_CALL, condvar_wait
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONDVAR_WAIT_INTERRUPT_SYSTEM_CALL, condvar_wait_interrupt
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONDVAR_SIGNAL_SYSTEM_CALL, condvar_signal
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONDVAR_BROADCAST_SYSTEM_CALL, condvar_broadcast
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_TIMER_START_SYSTEM_CALL, timer_start
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_TIMER_STOP_SYSTEM_CALL, timer_stop
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CAPTURE_FAILURE_DATA_SYSTEM_CALL, capture_failure_data
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONSOLE_PUTCHAR_SYSTEM_CALL, console_putchar
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONSOLE_GETCHAR_SYSTEM_CALL, console_getchar
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_LCD_PUTCHAR_SYSTEM_CALL, lcd_putchar
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_LCD_DRAW_TILE_SYSTEM_CALL, lcd_draw_tile
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_APP_SYSTEM_CALL, app_system_call
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_SELF_SYSTEM_CALL, thread_self
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_NAME_SYSTEM_CALL, thread_name
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_CONDVAR_WAIT_INTERRUPT_SYSTEM_CALL, thread_condvar_wait_interrupt
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_YIELD_SYSTEM_CALL, thread_yield

.end
