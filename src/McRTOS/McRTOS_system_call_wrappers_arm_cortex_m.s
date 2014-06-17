/**
 * @file McRTOS_system_call_wrappers_arm_cortex_m.s
 *
 * McRTOS System call wrapper functions, to invoke system calls from C code.
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */

#include "McRTOS_arm_cortex_m_macros.s"

.extern g_rtos_system_call_dispatch_table

.text
.thumb

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
.macro GEN_SYSTEM_CALL_WRAPPER_FUNCTION _system_call_number_, _rtos_function_

/**
 * System call wrapper \_rtos_function_
 *
 * @param   r0-r3: system call input parameters
 *
 * @return  r0: system call return code
 */
.global \_rtos_function_

.thumb_func
.func \_rtos_function_

\_rtos_function_:
    push    {r4-r5, lr}
    /*
     * Check if caller is an ISR:
     */
#if 1
    mrs	    r4, ipsr
    mov	    r5, #CPU_REG_IPSR_EXCEPTION_NUMBER_MASK
    tst	    r4, r5
    bne	    L_fast_path_\_rtos_function_
#else
    mrs	    r4, msp
    cmp	    r4, sp
    beq	    L_fast_path_\_rtos_function_
#endif

    /*
     * Check if caller is a thread running in privileged mode:
     */
    mrs	    r4, control
    mov	    r5, #CPU_REG_CONTROL_nPRIV_MASK
    tst	    r4, r5
    bne	    L_slow_path_\_rtos_function_

 L_fast_path_\_rtos_function_:
    /*
     * Caller is in privileged mode already, so just call the corresponding
     * kernel service directly:
     */
    ldr     r4, =g_rtos_system_call_dispatch_table
    ldr     r4, [r4, #(\_system_call_number_ * ARM_CPU_WORD_SIZE_IN_BYTES)]
    blx     r4
L_exit_\_rtos_function_:
    /*
     * r0 == return code from the system call
     */
    pop     {r4-r5, pc}

    /*
     * Caller is in unprivileged mode, so execute an svc instruction:
     */
L_slow_path_\_rtos_function_:
    svc     #\_system_call_number_

    /*
     * We are in privileged mode now.
     *
     * r0-r3 == parameters of the invoked system call
     */
    mov	    r4, #\_system_call_number_
    bl	    rtos_invoke_system_call

    /*
     * r0 == return code from the system call
     */
    b	    L_exit_\_rtos_function_
.endfunc

.endm


/**
 * Helper function that invokes a system call
 *
 * @param   r0-r3: system call input parameters
 *          r4: system call number
 *
 * @return  r0: system call return code
 *
 * NOTE: It clobbers r0-r3, r4 and r5
 */
.thumb_func
.func rtos_invoke_system_call

rtos_invoke_system_call:
    push    {lr}

    /*
     * Save r0-r3 on the stack:
     */
    push    {r0-r3}

    /*
     * Call rtos_k_enter_system_call():
     * r4 == system call number
     */
    mov	    r0, r4
    bl      rtos_k_enter_system_call

    /*
     * Save return value from rtos_k_enter_system_call() in r5 and
     * restore r0-r3 from the stack:
     */
    mov	    r5, r0
    pop     {r0-r3}
    cmp     r5, #0
    bne     L_exit_rtos_invoke_system_call

    /*
     * Call the corresponding kernel service:
     *
     * r0-r3 == parameters of the invoked system call
     */
    ldr     r5, =g_rtos_system_call_dispatch_table
    lsl	    r4, r4, #2
    add     r5, r5, r4
    ldr     r5, [r5]
    blx     r5

    /*
     * Save return value from the system call in r5 and
     * call rtos_k_exit_system_call()
     */
    mov     r5, r0
    bl      rtos_k_exit_system_call

L_exit_rtos_invoke_system_call:
    /*
     * r5 == return code
     *
     * Set nPRIV bit in control register to return to unprivileged mode
     */
    mrs	    r0, control
    mov	    r1, #CPU_REG_CONTROL_nPRIV_MASK
    orr	    r0, r0, r1
    msr	    control, r0
    isb

    mov     r0, r5
    pop     {pc}
.endfunc

/*
 * Generate System call wrapper functions
 */

#ifdef MCRTOS_DYNAMIC_OBJECT_CREATION

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CREATE_THREAD_SYSTEM_CALL, \
                                 rtos_create_thread

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CREATE_MUTEX_SYSTEM_CALL, \
                                 rtos_create_mutex

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CREATE_CONDVAR_SYSTEM_CALL, \
                                 rtos_create_condvar

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CREATE_TIMER_SYSTEM_CALL, \
                                 rtos_create_timer

#endif /* MCRTOS_DYNAMIC_OBJECT_CREATION */

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_DELAY_SYSTEM_CALL, \
                                 rtos_thread_delay

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_ABORT_SYSTEM_CALL, \
                                 rtos_thread_abort

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_CONDVAR_WAIT_SYSTEM_CALL, \
                                 rtos_thread_condvar_wait

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_CONDVAR_SIGNAL_SYSTEM_CALL, \
                                 rtos_thread_condvar_signal

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_MUTEX_ACQUIRE_SYSTEM_CALL, \
                                 rtos_mutex_acquire

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_MUTEX_RELEASE_SYSTEM_CALL, \
                                 rtos_mutex_release

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONDVAR_WAIT_SYSTEM_CALL, \
                                 rtos_condvar_wait

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONDVAR_WAIT_INTERRUPT_SYSTEM_CALL, \
                                 rtos_condvar_wait_interrupt

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONDVAR_SIGNAL_SYSTEM_CALL, \
                                 rtos_condvar_signal

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONDVAR_BROADCAST_SYSTEM_CALL, \
                                 rtos_condvar_broadcast

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_TIMER_START_SYSTEM_CALL, \
                                 rtos_timer_start

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_TIMER_STOP_SYSTEM_CALL, \
                                 rtos_timer_stop

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CAPTURE_FAILURE_DATA_SYSTEM_CALL, \
                                 rtos_capture_failure_data

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_SET_FDC_PARAMS_SYSTEM_CALL, \
                                 rtos_set_fdc_params

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONSOLE_PUTCHAR_SYSTEM_CALL, \
                                 rtos_console_putchar

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONSOLE_GETCHAR_SYSTEM_CALL, \
                                 rtos_console_getchar

#ifdef LCD_SUPPORTED
GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_LCD_PUTCHAR_SYSTEM_CALL, \
                                 rtos_lcd_putchar

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_LCD_DRAW_TILE_SYSTEM_CALL, \
                                 rtos_lcd_draw_tile
#endif

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_APP_SYSTEM_CALL, \
                                 rtos_app_system_call

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_SELF_SYSTEM_CALL, \
                                 rtos_thread_self

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_NAME_SYSTEM_CALL, \
                                 rtos_thread_name

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_CONDVAR_WAIT_INTERRUPT_SYSTEM_CALL, \
                                 rtos_thread_condvar_wait_interrupt

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_YIELD_SYSTEM_CALL, \
                                 rtos_thread_yield

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CALLER_IS_THREAD_SYSTEM_CALL, \
                                 rtos_caller_is_thread

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_MPU_RW_REGION_PUSH_SYSTEM_CALL, \
                                 rtos_mpu_rw_region_push

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_MPU_RW_REGION_POP_SYSTEM_CALL, \
                                 rtos_mpu_rw_region_pop

.end
