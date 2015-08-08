/**
 * @file McRTOS_system_call_wrappers_arm_cortex_m.s
 *
 * McRTOS System call wrapper functions, to invoke system calls from C code.
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */

#include <McRTOS/McRTOS_arm_cortex_m_macros.s>

.extern g_rtos_system_call_dispatch_table
.extern rtos_k_enter_privileged_mode
.extern rtos_k_exit_privileged_mode

.text
.thumb
#ifndef __ARM_ARCH_6M__
.syntax unified
#endif

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
    push    {r4-r5, r7, lr}
    add	    r7, sp, #0

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
    pop     {r4-r5, r7, pc}

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
    /*
     * Function prolog with frame pointer:
     */
    push    {r7, lr}
    add	    r7, sp, #0

    /*
     * Save r0-r3 on the stack:
     */
    push    {r0-r3}

    /*
     * Call rtos_k_enter_privileged_mode():
     * r4 == system call number
     */
    mov	    r0, r4
    bl      rtos_k_enter_privileged_mode

    /*
     * Save return value from rtos_k_enter_privileged_mode() in r5 and
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
     * call rtos_k_exit_privileged_mode()
     */
    mov     r5, r0
    bl      rtos_k_exit_privileged_mode

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
    pop     {r7, pc}
.endfunc

/*
 * Generate System call wrapper functions
 */

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_INIT_SYSTEM_CALL, \
                                 rtos_thread_init

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_MUTEX_INIT_SYSTEM_CALL, \
                                 rtos_mutex_init

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CONDVAR_INIT_SYSTEM_CALL, \
                                 rtos_condvar_init

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_TIMER_INIT_SYSTEM_CALL, \
                                 rtos_timer_init

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

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_SET_COMP_REGION_SYSTEM_CALL, \
                                 rtos_thread_set_comp_region

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_RESTORE_COMP_REGION_SYSTEM_CALL, \
                                 rtos_thread_restore_comp_region

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_SET_TMP_REGION_SYSTEM_CALL, \
                                 rtos_thread_set_tmp_region

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_UNSET_TMP_REGION_SYSTEM_CALL, \
                                 rtos_thread_unset_tmp_region

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_ENABLE_FPU_SYSTEM_CALL, \
                                 rtos_thread_enable_fpu

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_THREAD_DISABLE_FPU_SYSTEM_CALL, \
                                 rtos_thread_disable_fpu

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_POINTER_CIRCULAR_BUFFER_INIT_SYSTEM_CALL, \
                                 rtos_pointer_circular_buffer_init

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_POINTER_CIRCULAR_BUFFER_WRITE_SYSTEM_CALL, \
                                 rtos_pointer_circular_buffer_write

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_POINTER_CIRCULAR_BUFFER_READ_SYSTEM_CALL, \
                                 rtos_pointer_circular_buffer_read

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_BYTE_CIRCULAR_BUFFER_INIT_SYSTEM_CALL, \
                                 rtos_byte_circular_buffer_init

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_BYTE_CIRCULAR_BUFFER_WRITE_SYSTEM_CALL, \
                                 rtos_byte_circular_buffer_write

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_BYTE_CIRCULAR_BUFFER_READ_SYSTEM_CALL, \
                                 rtos_byte_circular_buffer_read

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CIRCULAR_BUFFER_IS_EMPTY_SYSTEM_CALL, \
                                 rtos_circular_buffer_is_empty

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_QUEUE_INIT_SYSTEM_CALL, \
                                 rtos_queue_init

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_QUEUE_ADD_SYSTEM_CALL, \
                                 rtos_queue_add

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_QUEUE_REMOVE_SYSTEM_CALL, \
                                 rtos_queue_remove

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_GET_TICKS_SYSTEM_CALL, \
                                 rtos_get_ticks

GEN_SYSTEM_CALL_WRAPPER_FUNCTION RTOS_CAPTURE_FDC_MSG_VPRINTF_SYSTEM_CALL, \
                                 rtos_capture_fdc_msg_vprintf

.end
