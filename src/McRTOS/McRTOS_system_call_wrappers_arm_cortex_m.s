/**
 * @file McRTOS_system_call_wrappers_armv6_m.s
 *
 * McRTOS System call wrapper functions, to invoke system calls from C code.
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 

#include "arm_defs.h"
#include "arm_cortex_m_macros.s"

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
    /*
     * For now just call the corresponding kernel service directly:
     *
     * TODO: Need to implement a real SVC handler
     */
    push    {r4, lr}
    ldr     r4, =g_rtos_system_call_dispatch_table
    ldr     r4, [r4, #(\_system_call_number_ * ARM_CPU_WORD_SIZE_IN_BYTES)]
    blx     r4
    pop     {r4, pc}
.endfunc

.endm

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

.end
