/**
 * @file McRTOS_kernel_services.c
 *
 * McRTOS kernel services. These functions can only be invoked from
 * privileged code.
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */

#include "McRTOS_kernel_services.h"
#include "McRTOS_internals.h"
#include "failure_data_capture.h"
#include "hardware_abstractions.h"
#include "arm_defs.h"
#include "utils.h"

TODO("Remove this pragma")
#pragma GCC diagnostic ignored "-Wunused-parameter"

#if DEFINED_ARM_CLASSIC_ARCH()
static void
rtos_invalid_interrupt_return(void);
#endif

static void
rtos_k_condvar_signal_internal(
    _INOUT_ struct rtos_condvar *rtos_condvar_p,
    _IN_ bool broadcast);

static void
check_circular_buffer_invariants(
    _IN_ struct rtos_circular_buffer *circ_buf_p,
    _IN_ uint32_t signature);

#if defined(LPC2478_SOC)
/**
 * global variable used by the spinlock to serialize atomic operations
 * across multiple CPU cores
 */
volatile uint32_t g_rtos_atomic_ops_spinlock = 0x0;

#endif

#define GEN_SYSTEM_CALL_DISPATCH_ENTRY(_system_call_number,             \
                                       _rtos_k_function)                \
        [_system_call_number] = _rtos_k_function

/**
 * System call dispatch table
 *
 * NOTE: This variable is accessed from assembly code
 */
const void *const g_rtos_system_call_dispatch_table[] =
{
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_INIT_SYSTEM_CALL, rtos_k_thread_init),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_DELAY_SYSTEM_CALL, rtos_k_thread_delay),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_ABORT_SYSTEM_CALL, rtos_k_thread_abort),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_CONDVAR_WAIT_SYSTEM_CALL, rtos_k_thread_condvar_wait),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_CONDVAR_SIGNAL_SYSTEM_CALL, rtos_k_thread_condvar_signal),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_MUTEX_INIT_SYSTEM_CALL, rtos_k_mutex_init),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_MUTEX_ACQUIRE_SYSTEM_CALL, rtos_k_mutex_acquire),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_MUTEX_RELEASE_SYSTEM_CALL, rtos_k_mutex_release),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_CONDVAR_INIT_SYSTEM_CALL, rtos_k_condvar_init),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_CONDVAR_WAIT_SYSTEM_CALL, rtos_k_condvar_wait),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_CONDVAR_SIGNAL_SYSTEM_CALL, rtos_k_condvar_signal),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_CONDVAR_BROADCAST_SYSTEM_CALL, rtos_k_condvar_broadcast),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_TIMER_INIT_SYSTEM_CALL, rtos_k_timer_init),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_TIMER_START_SYSTEM_CALL, rtos_k_timer_start),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_TIMER_STOP_SYSTEM_CALL, rtos_k_timer_stop),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_CAPTURE_FAILURE_DATA_SYSTEM_CALL, rtos_k_capture_failure_data),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_SET_FDC_PARAMS_SYSTEM_CALL, rtos_k_set_fdc_params),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_CONSOLE_PUTCHAR_SYSTEM_CALL, rtos_k_console_putchar),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_CONSOLE_GETCHAR_SYSTEM_CALL, rtos_k_console_getchar),
#ifdef LCD_SUPPORTED
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_LCD_PUTCHAR_SYSTEM_CALL, rtos_k_lcd_putchar),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_LCD_DRAW_TILE_SYSTEM_CALL, rtos_k_lcd_draw_tile),
#endif
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_APP_SYSTEM_CALL, rtos_k_app_system_call),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_SELF_SYSTEM_CALL, rtos_k_thread_self),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_NAME_SYSTEM_CALL, rtos_k_thread_name),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_CONDVAR_WAIT_INTERRUPT_SYSTEM_CALL, rtos_k_thread_condvar_wait_interrupt),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_YIELD_SYSTEM_CALL, rtos_k_thread_yield),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_CALLER_IS_THREAD_SYSTEM_CALL, rtos_k_caller_is_thread),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_SET_COMP_REGION_SYSTEM_CALL, rtos_k_thread_set_comp_region),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_RESTORE_COMP_REGION_SYSTEM_CALL, rtos_k_thread_restore_comp_region),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_SET_TMP_REGION_SYSTEM_CALL, rtos_k_thread_set_tmp_region),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_UNSET_TMP_REGION_SYSTEM_CALL, rtos_k_thread_unset_tmp_region),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_ENABLE_FPU_SYSTEM_CALL, rtos_k_thread_enable_fpu),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_THREAD_DISABLE_FPU_SYSTEM_CALL, rtos_k_thread_disable_fpu),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_POINTER_CIRCULAR_BUFFER_INIT_SYSTEM_CALL, rtos_k_pointer_circular_buffer_init),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_POINTER_CIRCULAR_BUFFER_WRITE_SYSTEM_CALL, rtos_k_pointer_circular_buffer_write),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_POINTER_CIRCULAR_BUFFER_READ_SYSTEM_CALL, rtos_k_pointer_circular_buffer_read),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_BYTE_CIRCULAR_BUFFER_INIT_SYSTEM_CALL, rtos_k_byte_circular_buffer_init),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_BYTE_CIRCULAR_BUFFER_WRITE_SYSTEM_CALL, rtos_k_byte_circular_buffer_write),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_BYTE_CIRCULAR_BUFFER_READ_SYSTEM_CALL, rtos_k_byte_circular_buffer_read),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_CIRCULAR_BUFFER_IS_EMPTY_SYSTEM_CALL, rtos_k_circular_buffer_is_empty),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_QUEUE_INIT_SYSTEM_CALL, rtos_k_queue_init),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_QUEUE_ADD_SYSTEM_CALL, rtos_k_queue_add),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_QUEUE_REMOVE_SYSTEM_CALL, rtos_k_queue_remove),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_GET_TICKS_SYSTEM_CALL, rtos_k_get_ticks),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(
        RTOS_CAPTURE_FDC_MSG_VPRINTF_SYSTEM_CALL, rtos_k_capture_fdc_msg_vprintf),
};

C_ASSERT(ARRAY_SIZE(g_rtos_system_call_dispatch_table) == RTOS_NUM_SYSTEM_CALLS);


/**
 * Initializes a McRTOS thread
 *
 * @param   params_p: Pointer to thread creation parameters
 *
 * @param   thread_stack_p: Pointer to the thread's execution stack.
 *
 * @param   rtos_thread_p: Pointer to the thread object
 *
 * @return  none
 *
 * NOTE: All McRTOS threads start running in unprivileged mode.
 */
void
rtos_k_thread_init(
    _IN_ const struct rtos_thread_creation_params *params_p,
    _IN_ struct rtos_thread_execution_stack *thread_stack_p,
    _OUT_ struct rtos_thread *rtos_thread_p)
{
    static uint8_t next_thread_context_id = 0;
    cpu_status_register_t cpu_status_register = 0;
    rtos_cpu_mode_t rtos_cpu_mode = RTOS_INVALID_CPU_MODE;

    FDC_ASSERT_VALID_RAM_POINTER(thread_stack_p, sizeof(uint32_t));
    FDC_ASSERT_VALID_RAM_POINTER(rtos_thread_p, sizeof(uint32_t));

    rtos_thread_prio_t thread_prio = params_p->p_priority;

    FDC_ASSERT(thread_prio < RTOS_NUM_THREAD_PRIORITIES,
        thread_prio, RTOS_NUM_THREAD_PRIORITIES);

    FDC_ASSERT_VALID_FUNCTION_POINTER(params_p->p_function_p);

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    rtos_thread_p->thr_signature = RTOS_THREAD_SIGNATURE;
    rtos_thread_p->thr_function_p = params_p->p_function_p;
    rtos_thread_p->thr_function_arg_p = params_p->p_function_arg_p;

#   ifdef LCD_SUPPORTED
    rtos_thread_p->thr_lcd_channel = params_p->p_lcd_channel;
#   endif

    rtos_thread_p->thr_abort_status = 0;
    rtos_thread_p->thr_base_priority = thread_prio;
    rtos_thread_p->thr_current_priority = thread_prio;
    rtos_thread_p->thr_state = RTOS_THREAD_CREATED;
    rtos_thread_p->thr_time_slice_ticks_left = RTOS_THREAD_TIME_SLICE_IN_TICKS;
    rtos_thread_p->thr_state_history = 0x0;
    rtos_thread_p->thr_priority_history = thread_prio;
    rtos_thread_p->thr_preempted_by_time_slice_count = 0;
    rtos_thread_p->thr_preempted_by_other_thread_count = 0;
    rtos_thread_p->thr_self_preempted_count = 0;
    rtos_thread_p->thr_owned_mutexes_count = 0;
    rtos_thread_p->thr_fpu_enable_count = 0;
    rtos_thread_p->thr_blocked_on_p = NULL;

    GLIST_NODE_INIT(&rtos_thread_p->thr_list_node);

    fdc_context_switch_trace_entry_t prefilled_trace_entry = 0;

    uint8_t thread_context_id = ATOMIC_POST_INCREMENT_UINT8(&next_thread_context_id);

    SET_BIT_FIELD(
        prefilled_trace_entry,
        FDC_CST_CONTEXT_ID_MASK,
        FDC_CST_CONTEXT_ID_SHIFT,
        thread_context_id);

    rtos_cpu_mode = RTOS_UNPRIVILEGED_THREAD_MODE;

    SET_BIT_FIELD(
        prefilled_trace_entry,
        FDC_CST_CONTEXT_TYPE_MASK,
        FDC_CST_CONTEXT_TYPE_SHIFT,
        FDC_CST_APPLICATION_THREAD);

    /*
     * Initialize thread's stack:
     */

    for (uint32_t i = 0;
         i < RTOS_STACK_OVERFLOW_BUFFER_SIZE_IN_ENTRIES; ++ i)
    {
        thread_stack_p->tes_stack_overflow_buffer[i] =
            RTOS_STACK_OVERFLOW_BUFFER_SIGNATURE;
    }

    thread_stack_p->tes_stack_overflow_marker = RTOS_STACK_OVERFLOW_MARKER;
    thread_stack_p->tes_stack_underflow_marker = RTOS_STACK_UNDERFLOW_MARKER;
    for (uint32_t i = 0; i < RTOS_THREAD_STACK_NUM_ENTRIES; ++ i)
    {
        thread_stack_p->tes_stack[i] = RTOS_STACK_UNUSED_SIGNATURE;
    }

    rtos_thread_p->thr_execution_stack_p = thread_stack_p;

    /*
     * Initialize thread's MPU data regions:
     * - thread's stack region
     * - component data region inactive
     * - temp data region inactive
     */

    rtos_thread_p->thr_stack_region.start_addr = thread_stack_p;
    rtos_thread_p->thr_stack_region.end_addr = thread_stack_p + 1;
    rtos_thread_p->thr_stack_region.flags = 0;

    rtos_thread_p->thr_comp_region.flags = MPU_REGION_INACTIVE;
    rtos_thread_p->thr_tmp_region.flags = MPU_REGION_INACTIVE;

    /*
     * Initialize the thread's execution context:
     */
    rtos_execution_context_init(
        &rtos_thread_p->thr_execution_context,
        params_p->p_name_p,
        cpu_id,
        RTOS_THREAD_CONTEXT,
        prefilled_trace_entry,
        rtos_cpu_mode,
        (cpu_register_t)params_p->p_function_p,
        (cpu_register_t)params_p->p_function_arg_p,
        &thread_stack_p->tes_stack[0],
        &thread_stack_p->tes_stack[RTOS_THREAD_STACK_NUM_ENTRIES]);

    /*
     * Initialize the thread's timer
     */
    rtos_k_timer_init(
        params_p->p_name_p,
        rtos_thread_timer_callback,
        &rtos_thread_p->thr_timer);

    /*
     * Initialize the thread's condvar:
     */
    rtos_k_condvar_init(
        params_p->p_name_p,
        &rtos_thread_p->thr_condvar);

    /*
     * Add new thread to the corresponding runnable thread queue:
     */

    FDC_ASSERT_RTOS_THREAD_INVARIANTS(rtos_thread_p);

    cpu_status_register = rtos_k_disable_cpu_interrupts();

    rtos_add_tail_runnable_thread(
        cpu_controller_p, rtos_thread_p);

    /*
     * If we are being called after the thread scheduler has been activated
     * (i.e. after McRTOS startup), and if the new thread has higher
     * priority than the calling thread, perform a synchronous context switch
     * to run the thread scheduler.
     */

    struct rtos_execution_context *current_execution_context_p =
	cpu_controller_p->cpc_current_execution_context_p;

    DBG_ASSERT(
	current_execution_context_p != NULL, cpu_controller_p, 0);

    if (current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
        struct rtos_thread *current_thread_p =
            RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

        if (thread_prio < current_thread_p->thr_current_priority)
        {
            /*
             * Add the calling thread at the beginning of the corresponding runnable
             * thread queue:
             */

            FDC_ASSERT(
                current_thread_p->thr_state == RTOS_THREAD_RUNNING,
                current_thread_p->thr_state, current_thread_p);

            rtos_add_head_runnable_thread(
                cpu_controller_p, current_thread_p);

            /*
             * Perform a synchronous context switch:
             *
             * NOTE: When this thread is switched in again, it will start
             * executing after this call
             */
            rtos_k_synchronous_context_switch(current_execution_context_p);
        }
    }

    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Delays the current thread a given number of milliseconds
 */
void
rtos_k_thread_delay(rtos_milliseconds_t num_milliseconds)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    FDC_ASSERT(
        current_thread_p->thr_owned_mutexes_count == 0,
        current_thread_p->thr_owned_mutexes_count, current_thread_p);

    FDC_ASSERT(num_milliseconds != 0, num_milliseconds, current_thread_p);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    /*
     * Start timer for the current thread:
     */
    rtos_k_timer_start(
        &current_thread_p->thr_timer,
        num_milliseconds);

    /*
     * Wait for the calling thread's delay timer to expire:
     */
    rtos_k_condvar_wait_intr_disabled(&current_thread_p->thr_condvar, NULL);

    FDC_ASSERT(
        current_thread_p->thr_timer.tmr_time_to_expire == 0,
        current_thread_p->thr_timer.tmr_time_to_expire, current_thread_p);

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Delay calling thread by making it spin for the given number of
 * microseconds.  For delays longer than one 1 tick period
 * (RTOS_MILLISECONDS_PER_TICK milliseconds), the millisecond delay
 * API must be used, instead of this function.
 */
void
rtos_k_thread_micro_delay(_IN_ rtos_microseconds_t num_microseconds)
{
#   ifdef _CPU_CYCLES_MEASURE_
    cpu_clock_cycles_t cycles_delta;
    cpu_clock_cycles_t start_cycles = get_cpu_clock_cycles();
    cpu_clock_cycles_t target_cpu_cycles_delta =
        MICROSECONDS_TO_CPU_CLOCK_CYCLES(num_microseconds);

    FDC_ASSERT(
        num_microseconds > 0 &&
        num_microseconds < RTOS_MILLISECONDS_PER_TICK * 1000,
        num_microseconds, RTOS_MILLISECONDS_PER_TICK);

    /*
     * NOTE: We don't need to check overflow for target_cpu_cycles_delta
     * since num_micro_seconds < TICK_TIMER_PERIOD_IN_MS * 1000
     */

    do {
        cycles_delta =
            CPU_CLOCK_CYCLES_DELTA(start_cycles, get_cpu_clock_cycles());
    } while (cycles_delta < target_cpu_cycles_delta);
#   else

    delay_loop(num_microseconds * (SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ / 5));

#   endif
}


/**
 * Return the current number of ticks for the calling CPU.
 * 1 tick == RTOS_MILLISECONDS_PER_TICK milliseconds
 */
rtos_ticks_t
rtos_k_get_ticks(void)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    return cpu_controller_p->cpc_ticks_since_boot_count;
}


/**
 * Abort the calling thread
 */
void
rtos_k_thread_abort(fdc_error_t fdc_error)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    FDC_ASSERT(
        current_thread_p->thr_owned_mutexes_count == 0,
        current_thread_p->thr_owned_mutexes_count,
        current_thread_p);

    FDC_ASSERT(fdc_error != 0, current_thread_p, 0);

    current_thread_p->thr_abort_status = fdc_error;

    DEBUG_PRINTF("thread %s (%#p) aborted (context: %#p, abort status: %#x)\n",
	         current_execution_context_p->ctx_name_p,
	         current_thread_p,
	         current_execution_context_p,
	         fdc_error);

    /*
     * Disable interrupts in the ARM core
     */
    (void)rtos_k_disable_cpu_interrupts();

    FDC_ASSERT(
        current_thread_p->thr_state == RTOS_THREAD_RUNNING,
        current_thread_p->thr_state, current_thread_p);

    FDC_ASSERT(
        GLIST_NODE_IS_UNLINKED(&current_thread_p->thr_list_node),
        &current_thread_p->thr_list_node, current_thread_p);

    RTOS_THREAD_CHANGE_STATE(current_thread_p, RTOS_THREAD_ABORTED);

    RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
        current_execution_context_p, CTX_SWITCHED_OUT_THREAD_TERMINATED,
        cpu_controller_p);

    /*
     * Perform a synchronous context switch:
     */
    rtos_k_synchronous_context_switch(current_execution_context_p);

    /*
     * We should never come back here:
     */
    FDC_ASSERT(false, 0, 0);
}


#if DEFINED_ARM_CLASSIC_ARCH()
static void
rtos_invalid_interrupt_return(void)
{
    /*
     * We should never come here:
     */
    FDC_ASSERT(false, 0, 0);
}
#endif


/**
 * Self-preempt the calling thread, adding it at the end
 * of the corresponding runnable thread queue
 */
void
rtos_k_thread_yield(void)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    FDC_ASSERT(
        current_thread_p->thr_owned_mutexes_count == 0,
        current_thread_p->thr_owned_mutexes_count, current_thread_p);

    FDC_ASSERT(
        current_thread_p->thr_current_priority ==
        current_thread_p->thr_base_priority,
        current_thread_p->thr_current_priority,
        current_thread_p->thr_base_priority);

    FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
        current_execution_context_p, CTX_SWITCHED_OUT_THREAD_YIELD,
        cpu_controller_p);

    /*
     * Add calling thread at the end of the corresponding runnable thread queue,
     * to give a chance to run to other threads that have the same
     * priority:
     */
    rtos_add_tail_runnable_thread(
        cpu_controller_p, current_thread_p);

    current_thread_p->thr_time_slice_ticks_left =
        RTOS_THREAD_TIME_SLICE_IN_TICKS;

    current_thread_p->thr_self_preempted_count ++;

    /*
     * Perform a synchronous context switch:
     *
     * NOTE: When this thread is switched in again, it will start
     * executing after this call
     */
    rtos_k_synchronous_context_switch(current_execution_context_p);

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Return pointer to the McRTOS thread object for the calling thread
 */
const struct rtos_thread *
rtos_k_thread_self(void)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    struct rtos_execution_context *current_execution_context_p =
        RTOS_GET_CURRENT_EXECUTION_CONTEXT();

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    return current_thread_p;
}


bool
rtos_k_caller_is_thread(void)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(false);

    struct rtos_execution_context *current_execution_context_p =
        RTOS_GET_CURRENT_EXECUTION_CONTEXT();

    return (current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT);
}


const char *
rtos_k_thread_name(
    _IN_ const struct rtos_thread *rtos_thread_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT_RTOS_THREAD_INVARIANTS(rtos_thread_p);

    return rtos_thread_p->thr_execution_context.ctx_name_p;
}


void
rtos_k_thread_enable_fpu(void)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    struct rtos_execution_context *current_execution_context_p =
        RTOS_GET_CURRENT_EXECUTION_CONTEXT();

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
         RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    FDC_ASSERT(current_thread_p->thr_fpu_enable_count < UINT8_MAX,
	       current_thread_p->thr_fpu_enable_count,
	       current_thread_p);

    struct rtos_cpu_controller *cpu_controller_p =
	&g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct rtos_thread *last_fpu_thread_p =
	    cpu_controller_p->cpc_last_fpu_thread_p;

    if (current_thread_p->thr_fpu_enable_count == 0) {
        cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

        cortex_m_enable_fpu();
	if (last_fpu_thread_p != NULL) {
            FDC_ASSERT(last_fpu_thread_p->thr_signature == RTOS_THREAD_SIGNATURE,
                       last_fpu_thread_p->thr_signature, last_fpu_thread_p);

            FDC_ASSERT(last_fpu_thread_p != current_thread_p,
		       last_fpu_thread_p, current_thread_p);

	    cpu_controller_p->cpc_fpu_context_switch_count ++;
	    cortex_m_save_fpu_context(&last_fpu_thread_p->thr_saved_fpu_context);
	}

        cpu_controller_p->cpc_last_fpu_thread_p = current_thread_p;
        rtos_k_restore_cpu_interrupts(cpu_status_register);
    } else {
	FDC_ASSERT(last_fpu_thread_p == current_thread_p,
		   last_fpu_thread_p, current_thread_p);
    }

    current_thread_p->thr_fpu_enable_count ++;
}


void
rtos_k_thread_disable_fpu(void)
{

    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    struct rtos_execution_context *current_execution_context_p =
        RTOS_GET_CURRENT_EXECUTION_CONTEXT();

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
         RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    FDC_ASSERT(current_thread_p->thr_fpu_enable_count > 0,
 	       current_thread_p->thr_fpu_enable_count,
 	       current_thread_p);

    struct rtos_cpu_controller *cpu_controller_p =
	&g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    FDC_ASSERT(cpu_controller_p->cpc_last_fpu_thread_p == current_thread_p,
               cpu_controller_p->cpc_last_fpu_thread_p,
               current_thread_p);

    current_thread_p->thr_fpu_enable_count --;
    if (current_thread_p->thr_fpu_enable_count == 0) {
        cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

	cortex_m_disable_fpu();
	cpu_controller_p->cpc_last_fpu_thread_p = NULL;
        rtos_k_restore_cpu_interrupts(cpu_status_register);
    }
}


void
rtos_k_thread_condvar_wait(
    _IN_ struct rtos_mutex *rtos_mutex_p)
{
   FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    struct rtos_execution_context *current_execution_context_p =
        RTOS_GET_CURRENT_EXECUTION_CONTEXT();

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    rtos_k_condvar_wait(&current_thread_p->thr_condvar, rtos_mutex_p, NULL);
}


void
rtos_k_thread_condvar_wait_interrupt(void)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    struct rtos_execution_context *current_execution_context_p =
        RTOS_GET_CURRENT_EXECUTION_CONTEXT();

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    rtos_k_condvar_wait_intr_disabled(&current_thread_p->thr_condvar, NULL);
}


void
rtos_k_thread_condvar_signal(
    _IN_ struct rtos_thread *rtos_thread_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(false);

    FDC_ASSERT_RTOS_THREAD_INVARIANTS(rtos_thread_p);

    rtos_k_condvar_signal_internal(&rtos_thread_p->thr_condvar, false);
}


void
rtos_k_mutex_init(
    _IN_  const char *mutex_name_p,
    _OUT_ struct rtos_mutex *rtos_mutex_p)
{
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(mutex_name_p, sizeof(char));
    FDC_ASSERT_VALID_RAM_POINTER(rtos_mutex_p, sizeof(uint32_t));
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    rtos_mutex_p->mtx_signature = RTOS_MUTEX_SIGNATURE;
    rtos_mutex_p->mtx_name_p = mutex_name_p;
    rtos_mutex_p->mtx_cpu_id = cpu_id;
    rtos_mutex_p->mtx_owner_p = NULL;
    GLIST_NODE_INIT(&rtos_mutex_p->mtx_waiting_thread_queue_anchor);
}


/**
 * Acquire a McRTOS mutex
 */
void
rtos_k_mutex_acquire(
    _IN_ struct rtos_mutex *rtos_mutex_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    FDC_ASSERT(
        rtos_mutex_p->mtx_signature == RTOS_MUTEX_SIGNATURE,
        rtos_mutex_p->mtx_signature, rtos_mutex_p);

    FDC_ASSERT(
        rtos_mutex_p->mtx_cpu_id == cpu_id,
        rtos_mutex_p->mtx_cpu_id, cpu_id);

    FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    if (rtos_mutex_p->mtx_owner_p != NULL)
    {
        /*
         * The mutex is currently owned by another thread
         */
        struct rtos_thread *mutex_owner_p = rtos_mutex_p->mtx_owner_p;

        FDC_ASSERT_RTOS_THREAD_INVARIANTS(mutex_owner_p);

        FDC_ASSERT(
            mutex_owner_p != current_thread_p,
            mutex_owner_p, current_thread_p);

        /*
         * If mutex owner's priority is lower than the current thread's
         * priority, boost the mutex owner's priority to inherit the
         * current thread's priority. Also, requeue the mutex owner to the
         * corresponding runnable queue, but only if it is not currently
         * blocked:
         */
        if (mutex_owner_p->thr_current_priority >
            current_thread_p->thr_current_priority)
        {
            RTOS_THREAD_CHANGE_PRIORITY(
                mutex_owner_p, current_thread_p->thr_current_priority);

            if (mutex_owner_p->thr_state == RTOS_THREAD_RUNNABLE)
            {
                rtos_remove_runnable_thread(
                    cpu_controller_p, mutex_owner_p, RTOS_THREAD_BEING_REQUEUED);

                rtos_add_tail_runnable_thread(
                    cpu_controller_p, mutex_owner_p);
            }
        }

        /*
         * Add current thread at the end of the mutex's waiting queue and
         * change its state from running to blocked:
         */
        glist_add_tail_elem(
            &rtos_mutex_p->mtx_waiting_thread_queue_anchor,
            &current_thread_p->thr_list_node);

        RTOS_THREAD_CHANGE_STATE(current_thread_p, RTOS_THREAD_BLOCKED_ON_MUTEX);
	FDC_ASSERT(
	    current_thread_p->thr_blocked_on_p == NULL,
	    current_thread_p->thr_blocked_on_p, rtos_mutex_p);

	current_thread_p->thr_blocked_on_mutex_p = rtos_mutex_p;

        RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
            current_execution_context_p,
            CTX_SWITCHED_OUT_THREAD_BLOCKED_ON_MUTEX,
            cpu_controller_p);

        /*
         * Perform a synchronous context switch:
         *
         * NOTE: When this thread is switched in again, it will start
         * executing after this call
         */
        rtos_k_synchronous_context_switch(current_execution_context_p);

	FDC_ASSERT(
	    current_thread_p->thr_blocked_on_mutex_p == rtos_mutex_p,
	    current_thread_p->thr_blocked_on_mutex_p, rtos_mutex_p);

	current_thread_p->thr_blocked_on_mutex_p = NULL;
    }
    else
    {
        rtos_mutex_p->mtx_owner_p = current_thread_p;
        current_thread_p->thr_owned_mutexes_count ++;
    }

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Internal function common to rtos_k_mutex_release() and rtos_k_condvar_wait()
 */
static void
rtos_k_mutex_release_internal(
    _INOUT_ struct rtos_mutex *rtos_mutex_p,
    _INOUT_ struct rtos_thread *current_thread_p,
    _IN_    struct rtos_cpu_controller *cpu_controller_p,
    _IN_    bool in_condvar_wait)
{
    cpu_status_register_t cpu_status_register;

    FDC_ASSERT(
        rtos_mutex_p->mtx_signature == RTOS_MUTEX_SIGNATURE,
        rtos_mutex_p->mtx_signature, rtos_mutex_p);

    FDC_ASSERT(
        rtos_mutex_p->mtx_cpu_id == cpu_controller_p->cpc_cpu_id,
        rtos_mutex_p->mtx_cpu_id,
        cpu_controller_p->cpc_cpu_id);

    FDC_ASSERT(
        current_thread_p->thr_owned_mutexes_count > 0,
        current_thread_p, 0);

    /*
     * The mutex is currently owned by the calling thread
     */
    struct rtos_thread *mutex_owner_p = rtos_mutex_p->mtx_owner_p;

    FDC_ASSERT(
        mutex_owner_p == current_thread_p,
        mutex_owner_p, current_thread_p);

    if (in_condvar_wait) {
        FDC_ASSERT_CPU_INTERRUPTS_DISABLED();
    } else {
        FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

        /*
         * Disable interrupts in the ARM core
         */
        cpu_status_register = rtos_k_disable_cpu_interrupts();
    }

    /*
     * If there are waiters for the mutex, remove the first waiter,
     * transfer the ownership of the mutex to it and add it at the end
     * of the appropriate runnable thread queue:
     */

     if (GLIST_IS_NOT_EMPTY(&rtos_mutex_p->mtx_waiting_thread_queue_anchor))
     {
        struct glist_node *first_waiter_node_p =
            GLIST_GET_FIRST(
                &rtos_mutex_p->mtx_waiting_thread_queue_anchor);

        glist_remove_elem(first_waiter_node_p);

        /*
         * Set new mutex owner:
         */
        mutex_owner_p =
            RTOS_THREAD_QUEUE_NODE_GET_THREAD(first_waiter_node_p);

        FDC_ASSERT_RTOS_THREAD_INVARIANTS(mutex_owner_p);
        FDC_ASSERT(
            mutex_owner_p->thr_state == RTOS_THREAD_BLOCKED_ON_MUTEX,
            mutex_owner_p->thr_state, mutex_owner_p);

        /*
         * Transfer mutex ownership:
         */
        rtos_mutex_p->mtx_owner_p = mutex_owner_p;
        mutex_owner_p->thr_owned_mutexes_count ++;

        /*
         * If the old mutex owner had its priority boosted and its
         * current priority is higher than the new mutex owner's
         * current priority, boost the new owner's priority:
         *
         * NOTE: This condition means that there are other waiters and
         * at least one of them with higher priority than the
         * new owner.
         */
        if (current_thread_p->thr_current_priority <
                current_thread_p->thr_base_priority &&
            current_thread_p->thr_current_priority <
                mutex_owner_p->thr_current_priority)
        {
            RTOS_THREAD_CHANGE_PRIORITY(
                mutex_owner_p, current_thread_p->thr_current_priority);
        }

        /*
         * Add the new mutex owner to the end of the corresponding runnable queue:
         */
        rtos_add_tail_runnable_thread(
            cpu_controller_p, mutex_owner_p);
    }
    else
    {
        /*
         * Make the mutex available:
         */
        rtos_mutex_p->mtx_owner_p = NULL;
    }

    current_thread_p->thr_owned_mutexes_count --;

    if (!in_condvar_wait)
    {
        /*
         * If current thread had its priority boosted, lower it back to its
         * base priority:
         */
        if (current_thread_p->thr_current_priority <
            current_thread_p->thr_base_priority)
        {
            RTOS_THREAD_CHANGE_PRIORITY(
                current_thread_p, current_thread_p->thr_base_priority);
        }

        /*
         * Add current thread at the beginning of the corresponding runnable
         * queue and change its state from running to runnable
         */
        rtos_add_head_runnable_thread(
            cpu_controller_p, current_thread_p);

        /*
         * Perform a synchronous context switch:
         *
         * NOTE: When this thread is switched in again, it will start
         * executing after this call
         */
        rtos_k_synchronous_context_switch(&current_thread_p->thr_execution_context);

        /*
         * Restore previous interrupt masking in the ARM core
         */
        rtos_k_restore_cpu_interrupts(cpu_status_register);
    }
}


/**
 * Release a McRTOS mutex
 */
void
rtos_k_mutex_release(
    _INOUT_ struct rtos_mutex *rtos_mutex_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    rtos_k_mutex_release_internal(
        rtos_mutex_p, current_thread_p, cpu_controller_p, false);
}


void
rtos_k_condvar_init(
    _IN_  const char *condvar_name_p,
    _OUT_ struct rtos_condvar *rtos_condvar_p)
{
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(condvar_name_p, sizeof(char));
    FDC_ASSERT_VALID_RAM_POINTER(rtos_condvar_p, sizeof(uint32_t));
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    rtos_condvar_p->cv_signature = RTOS_CONDVAR_SIGNATURE;
    rtos_condvar_p->cv_name_p = condvar_name_p;
    rtos_condvar_p->cv_cpu_id = cpu_id;
    rtos_condvar_p->cv_released_mutex_p = NULL;
    GLIST_NODE_INIT(&rtos_condvar_p->cv_waiting_thread_queue_anchor);
}


static void
rtos_k_condvar_wait_internal(
    _INOUT_ struct rtos_condvar *rtos_condvar_p,
    _IN_    struct rtos_mutex *rtos_mutex_p,
    _INOUT_ rtos_milliseconds_t *timeout_ms_p)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    FDC_ASSERT(
        rtos_condvar_p->cv_signature == RTOS_CONDVAR_SIGNATURE,
        rtos_condvar_p->cv_signature, rtos_condvar_p);

    FDC_ASSERT(
        rtos_condvar_p->cv_cpu_id == cpu_id,
        rtos_condvar_p->cv_cpu_id, cpu_id);

    if (rtos_mutex_p != NULL)
    {
        if (rtos_condvar_p->cv_released_mutex_p == NULL)
	{
	    rtos_condvar_p->cv_released_mutex_p = rtos_mutex_p;
	}
	else
	{
	    FDC_ASSERT(
		rtos_mutex_p == rtos_condvar_p->cv_released_mutex_p,
		rtos_mutex_p, rtos_condvar_p->cv_released_mutex_p);
	}

	/*
	 * Release the mutex while having interrupts disabled, so that no one else
	 * can acquire it until after we have enqueued the calling thread to the
	 * condvar's wait queue:
	 */
	rtos_k_mutex_release_internal(
	    rtos_mutex_p, current_thread_p, cpu_controller_p, true);
    }

    if (timeout_ms_p != NULL) {
	rtos_milliseconds_t timeout_ms = *timeout_ms_p;

	FDC_ASSERT(timeout_ms != 0, rtos_condvar_p, 0);

	/*
	 * Start timer for the current thread:
	 */
	rtos_k_timer_start(
	    &current_thread_p->thr_timer,
	    timeout_ms);
    }

    /*
     * Add current thread at the end of the condvar's waiting queue and
     * change its state from running to blocked:
     */
    glist_add_tail_elem(
        &rtos_condvar_p->cv_waiting_thread_queue_anchor,
        &current_thread_p->thr_list_node);

    RTOS_THREAD_CHANGE_STATE(current_thread_p, RTOS_THREAD_BLOCKED_ON_CONDVAR);
    FDC_ASSERT(
	current_thread_p->thr_blocked_on_p == NULL,
	current_thread_p->thr_blocked_on_p, rtos_condvar_p);

    current_thread_p->thr_blocked_on_condvar_p = rtos_condvar_p;

    RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
        current_execution_context_p,
        CTX_SWITCHED_OUT_THREAD_BLOCKED_ON_CONDVAR,
        cpu_controller_p);

    /*
     * Perform a synchronous context switch:
     *
     * NOTE: When this thread is switched in again, it will start
     * executing after this call
     */
    rtos_k_synchronous_context_switch(current_execution_context_p);

    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();
    if (timeout_ms_p != NULL) {
	FDC_ASSERT(
	    GLIST_NODE_IS_UNLINKED(&current_thread_p->thr_timer.tmr_list_node),
	    current_thread_p, rtos_condvar_p);
	*timeout_ms_p = current_thread_p->thr_timer.tmr_time_to_expire;
    }

    FDC_ASSERT(
	current_thread_p->thr_blocked_on_condvar_p == rtos_condvar_p,
	current_thread_p->thr_blocked_on_condvar_p, rtos_condvar_p);

    current_thread_p->thr_blocked_on_condvar_p = NULL;
}


/**
 * Waits on a McRTOS condition variable. If timeout_ms_p != NULL and
 * the condition variable is not signaled before *timeout_ms_p milliseconds,
 * the calling thread will be awaken and *timeout_ms_p will be set 0.
 * Otherwise, if the cond variable is signaled before the timeout expires,
 * *timeout_ms_p will be set to the amount of time left before the timeout
 * expiration.
 */
void
rtos_k_condvar_wait(
    _INOUT_ struct rtos_condvar *rtos_condvar_p,
    _IN_    struct rtos_mutex *rtos_mutex_p,
    _INOUT_ rtos_milliseconds_t *timeout_ms_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    rtos_k_condvar_wait_internal(rtos_condvar_p, rtos_mutex_p, timeout_ms_p);

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);

    if (rtos_mutex_p != NULL) {
	/*
	 * Reacquire the mutex:
	 */
	rtos_k_mutex_acquire(rtos_mutex_p);
    }
}


/**
 * Waits on a McRTOS condition variable when the calling thread has interrupts
 * disabled.
 */
void
rtos_k_condvar_wait_intr_disabled(
    _INOUT_ struct rtos_condvar *rtos_condvar_p,
    _INOUT_ rtos_milliseconds_t *timeout_ms_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();

    rtos_k_condvar_wait_internal(rtos_condvar_p, NULL, timeout_ms_p);
}


/**
 * Internal function shared between rtos_k_condvar_signal() and
 * rtos_k_condvar_broadcast()
 */
static void
rtos_k_condvar_signal_internal(
    _INOUT_ struct rtos_condvar *rtos_condvar_p,
    _IN_ bool broadcast)
{
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        rtos_condvar_p->cv_signature == RTOS_CONDVAR_SIGNATURE,
        rtos_condvar_p->cv_signature, rtos_condvar_p);

    FDC_ASSERT(
        rtos_condvar_p->cv_cpu_id == cpu_id,
        rtos_condvar_p->cv_cpu_id, cpu_id);

    if (current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT) {
        FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();
    }

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    bool waiters_awaken = false;

    while (GLIST_IS_NOT_EMPTY(&rtos_condvar_p->cv_waiting_thread_queue_anchor)) {
        waiters_awaken = true;

        /*
         * If there are waiters on the condvar, remove the first waiter,
         * and add it at the end of the appropriate runnable thread queue:
         */
        struct glist_node *first_waiter_node_p =
            GLIST_GET_FIRST(
                &rtos_condvar_p->cv_waiting_thread_queue_anchor);

        glist_remove_elem(first_waiter_node_p);

        struct rtos_thread *awaken_thread_p =
            RTOS_THREAD_QUEUE_NODE_GET_THREAD(first_waiter_node_p);

        FDC_ASSERT_RTOS_THREAD_INVARIANTS(awaken_thread_p);
        FDC_ASSERT(
            awaken_thread_p->thr_state == RTOS_THREAD_BLOCKED_ON_CONDVAR,
            awaken_thread_p->thr_state, awaken_thread_p);

        rtos_add_tail_runnable_thread(
            cpu_controller_p, awaken_thread_p);

	if (GLIST_NODE_IS_LINKED(&awaken_thread_p->thr_timer.tmr_list_node)) {
	    rtos_k_timer_stop(&awaken_thread_p->thr_timer);
	}

        if (!broadcast) {
            break;
        }
    }

    /*
     * If the caller is a thread, perform a context switch, to allow awaken
     * threads to run as soon as possible, if they have higher priority than
     * the calling thread.
     *
     * NOTE: If the caller is an interrupt handler and there were waiters awaken,
     * they will get the chance to run when the calling interrupt handler calls
     * rtos_k_exit_interrupt(), which calls rtos_thread_scheduler().
     */
    if (current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT) {
        if (waiters_awaken) {
            /*
             * Add current thread at the beginning of the corresponding runnable
             * queue and change its state from running to runnable
             */
            struct rtos_thread *current_thread_p =
                RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

            rtos_add_head_runnable_thread(
                cpu_controller_p, current_thread_p);

            /*
             * Perform a synchronous context switch:
             *
             * NOTE: When this thread is switched in again, it will start
             * executing after this call
             */
            rtos_k_synchronous_context_switch(&current_thread_p->thr_execution_context);
        }
    }

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Signal a McRTOS condition variable. It wakes up the oldest waiter.
 */
void
rtos_k_condvar_signal(
    _INOUT_ struct rtos_condvar *rtos_condvar_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(false);

    rtos_k_condvar_signal_internal(rtos_condvar_p, false);
}


/**
 * Broadcast a McRTOS condition variable. It wakes up all waiters
 */
void
rtos_k_condvar_broadcast(
    _INOUT_ struct rtos_condvar *rtos_condvar_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(false);

    rtos_k_condvar_signal_internal(rtos_condvar_p, true);
}


void
rtos_k_timer_init(
    _IN_  const char *timer_name_p,
    _IN_  rtos_timer_function_t *timer_function_p,
    _OUT_ struct rtos_timer *rtos_timer_p)
{
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(timer_name_p, sizeof(char));
    FDC_ASSERT_VALID_FUNCTION_POINTER(timer_function_p);
    FDC_ASSERT_VALID_RAM_POINTER(rtos_timer_p, sizeof(uint32_t));
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    rtos_timer_p->tmr_signature = RTOS_TIMER_SIGNATURE;
    rtos_timer_p->tmr_name_p = timer_name_p;
    rtos_timer_p->tmr_cpu_id = cpu_id;
    rtos_timer_p->tmr_time_to_expire = 0;
    rtos_timer_p->tmr_callback_function_p = timer_function_p;
    GLIST_NODE_INIT(&rtos_timer_p->tmr_list_node);
}


void
rtos_k_timer_start(
    _INOUT_ struct rtos_timer *rtos_timer_p,
    _IN_ rtos_milliseconds_t expiration_time_in_ms)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(false);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    FDC_ASSERT(
        rtos_timer_p->tmr_signature == RTOS_TIMER_SIGNATURE,
        rtos_timer_p->tmr_signature, rtos_timer_p);

    FDC_ASSERT(expiration_time_in_ms != 0, expiration_time_in_ms, rtos_timer_p);

    rtos_ticks_t num_ticks = MILLISECONDS_TO_TICKS(expiration_time_in_ms);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    rtos_timer_wheel_spoke_index_t spoke_index =
        (cpu_controller_p->cpc_current_timer_wheel_spoke_index +
         num_ticks % RTOS_TIMER_WHEEL_NUM_SPOKES) % RTOS_TIMER_WHEEL_NUM_SPOKES;

    rtos_timer_p->tmr_time_to_expire =
        (num_ticks / RTOS_TIMER_WHEEL_NUM_SPOKES) * RTOS_MILLISECONDS_PER_TICK;

    /*
     * Add timer at the end of the corresponding hash chain of the calling
     * CPU's timer wheel:
     */
    glist_add_tail_elem(
        &cpu_controller_p->cpc_timer_wheel_hash_chains_anchors[spoke_index],
        &rtos_timer_p->tmr_list_node);

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


void
rtos_k_timer_stop(
    _INOUT_ struct rtos_timer *rtos_timer_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(false);

    FDC_ASSERT(
        rtos_timer_p->tmr_signature == RTOS_TIMER_SIGNATURE,
        rtos_timer_p->tmr_signature, rtos_timer_p);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    glist_remove_elem(&rtos_timer_p->tmr_list_node);

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Register an interrupt with McRTOS and install the corresponding ISR
 * at the corresponding interrupt vector in the interrupt controller.
 *
 * @param   params_p: Pointer to the interrupt object's registration parameters.
 *
 * @param   rtos_interrupt_p: Pointer to the area where the pointer to a new
 *          McRTOS interrupt object is to be returned.
 *
 * @return  none
 */
void
rtos_k_register_interrupt(
    _IN_  const struct rtos_interrupt_registration_params *params_p,
    _OUT_ struct rtos_interrupt **rtos_interrupt_p)
{
    fdc_error_t fdc_error = 0;
    bool fatal_error = false;
    interrupt_channel_t channel = params_p->irp_channel;
    interrupt_prio_t interrupt_prio = params_p->irp_priority;
    cpu_id_t cpu_id = params_p->irp_cpu_id;
    isr_function_t *isr_function_p = params_p->irp_isr_function_p;

    *rtos_interrupt_p = NULL;

    FDC_ASSERT(cpu_id < SOC_NUM_CPU_CORES,
        cpu_id, SOC_NUM_CPU_CORES);

    FDC_ASSERT(channel < SOC_NUM_INTERRUPT_CHANNELS,
        channel, SOC_NUM_INTERRUPT_CHANNELS);

    FDC_ASSERT(interrupt_prio < SOC_NUM_INTERRUPT_PRIORITIES,
        interrupt_prio, SOC_NUM_INTERRUPT_PRIORITIES);

    FDC_ASSERT_VALID_FUNCTION_POINTER(isr_function_p);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    if (channel < 0) {
        FDC_ASSERT(
            RTOS_INTR_BIT_MAP_GET_BIT(
		cpu_controller_p->cpc_active_internal_interrupts, -channel) == 0,
            cpu_controller_p->cpc_active_internal_interrupts, -channel);
    } else {
        FDC_ASSERT(
            RTOS_INTR_BIT_MAP_GET_BIT(
		cpu_controller_p->cpc_active_external_interrupts, channel) == 0,
            cpu_controller_p->cpc_active_external_interrupts, channel);
    }

    /*
     * Allocate an interrupt object:
     */
    struct rtos_interrupt *interrupt_p =
        ATOMIC_POST_INCREMENT_POINTER(g_McRTOS_p->rts_next_free_interrupt_p);

    if (interrupt_p >= &g_McRTOS_p->rts_interrupts[RTOS_MAX_NUM_INTERRUPTS])
    {
        fdc_error = CAPTURE_FDC_ERROR(
                        "No more interrupts can be registered",
                        interrupt_p,
                        g_McRTOS_p->rts_next_free_interrupt_p);

        /*
         * This is considered a fatal error
         */
        fatal_error = true;
        goto Exit;
    }

    interrupt_p->int_signature = RTOS_INTERRUPT_SIGNATURE;
    interrupt_p->int_cpu_controller_p = cpu_controller_p;
    interrupt_p->int_isr_function_p = isr_function_p;
    interrupt_p->int_arg_p = params_p->irp_arg_p;
    interrupt_p->int_channel = channel;
    interrupt_p->int_priority = interrupt_prio;
    interrupt_p->int_cpu_id = cpu_id;

#if DEFINED_ARM_CLASSIC_ARCH()
    /*
     * Initialize the interrupt's stack:
     */

    for (uint32_t i = 0;
         i < RTOS_STACK_OVERFLOW_BUFFER_SIZE_IN_ENTRIES; ++ i)
    {
        interrupt_p->int_stack_overflow_buffer[i] =
            RTOS_STACK_OVERFLOW_BUFFER_SIGNATURE;
    }

    interrupt_p->int_stack_overflow_marker = RTOS_STACK_OVERFLOW_MARKER;
    interrupt_p->int_stack_underflow_marker = RTOS_STACK_UNDERFLOW_MARKER;
    for (uint32_t i = 0; i < RTOS_INTERRUPT_STACK_NUM_ENTRIES; ++ i)
    {
        interrupt_p->int_stack[i] = RTOS_STACK_UNUSED_SIGNATURE;
    }
#   endif

    /*
     * Initialize the interrupt's execution context:
     */

    fdc_context_switch_trace_entry_t prefilled_trace_entry = 0;

    SET_BIT_FIELD(
        prefilled_trace_entry,
        FDC_CST_CONTEXT_ID_MASK,
        FDC_CST_CONTEXT_ID_SHIFT,
        interrupt_p - g_McRTOS_p->rts_interrupts);

    SET_BIT_FIELD(
        prefilled_trace_entry,
        FDC_CST_CONTEXT_TYPE_MASK,
        FDC_CST_CONTEXT_TYPE_SHIFT,
        FDC_CST_INTERRUPT);

    SET_BIT_FIELD(
        prefilled_trace_entry,
        FDC_CST_CONTEXT_PRIORITY_MASK,
        FDC_CST_CONTEXT_PRIORITY_SHIFT,
        interrupt_prio);

    rtos_execution_stack_entry_t *stack_top_end_p;
    rtos_execution_stack_entry_t *stack_bottom_end_p;

#   if DEFINED_ARM_CLASSIC_ARCH()
    stack_top_end_p = &interrupt_p->int_stack[0];
    stack_bottom_end_p =
        &interrupt_p->int_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES];
#   elif DEFINED_ARM_CORTEX_M_ARCH()
    stack_top_end_p = &g_cortex_m_exception_stack.es_stack[0];
    stack_bottom_end_p =
        &g_cortex_m_exception_stack.es_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES];
#   else
#       error "CPU architrecture not supported"
#   endif

    rtos_execution_context_init(
        &interrupt_p->int_execution_context,
        params_p->irp_name_p,
        cpu_id,
        RTOS_INTERRUPT_CONTEXT,
        prefilled_trace_entry,
        RTOS_INTERRUPT_MODE,
        (cpu_register_t)isr_function_p,
        0,
        stack_top_end_p,
        stack_bottom_end_p);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    /*
     * Install ISR in the interrupt controller:
     */
    install_isr(
        channel,
        isr_function_p,
        interrupt_prio,
        cpu_id);

    *rtos_interrupt_p = interrupt_p;

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);

Exit:
    if (fatal_error)
    {
        fatal_error_handler(fdc_error);
    }
}

#pragma GCC diagnostic push

#ifndef _RELIABILITY_CHECKS_
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif

/**
 * Function to be invoked at the beginning of an assembly language ISR,
 * right after saving the current execution context's CPU registers.
 *
 * @pre     The current execution context was already saved.
 *
 * @pre     CPU interrupts are disabled.
 *
 * @param   rtos_interrupt_p Pointer to the corresponding McRTOS interrupt
 *          object.
 *
 * @return stack pointer to use for the calling ISR
 */
cpu_register_t
rtos_k_enter_interrupt(
    struct rtos_interrupt *rtos_interrupt_p)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();

    FDC_ASSERT(
        rtos_interrupt_p->int_signature == RTOS_INTERRUPT_SIGNATURE,
        rtos_interrupt_p->int_signature, rtos_interrupt_p);

#   ifdef DEBUG
    check_rtos_interrupt_entry_preconditions(rtos_interrupt_p);
#   endif

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    DBG_ASSERT(
        rtos_interrupt_p->int_cpu_controller_p == cpu_controller_p,
        rtos_interrupt_p, cpu_controller_p);

    interrupt_channel_t interrupt_channel = rtos_interrupt_p->int_channel;

    DBG_ASSERT(
        interrupt_channel >= -1 && interrupt_channel < SOC_NUM_INTERRUPT_CHANNELS,
        interrupt_channel, rtos_interrupt_p);

    /*
     * Mark interrupt as active:
     */
    if (interrupt_channel < 0) {
        FDC_ASSERT(
            RTOS_INTR_BIT_MAP_GET_BIT(
		cpu_controller_p->cpc_active_internal_interrupts,
		-interrupt_channel) == 0,
            cpu_controller_p->cpc_active_internal_interrupts, -interrupt_channel);

	RTOS_INTR_BIT_MAP_SET_BIT(
	     cpu_controller_p->cpc_active_internal_interrupts,
	     -interrupt_channel);
    } else {
        FDC_ASSERT(
            RTOS_INTR_BIT_MAP_GET_BIT(
		cpu_controller_p->cpc_active_external_interrupts,
		interrupt_channel) == 0,
            cpu_controller_p->cpc_active_external_interrupts[
		RTOS_INTR_BIT_MAP_ENTRY_INDEX(interrupt_channel)], interrupt_channel);

	RTOS_INTR_BIT_MAP_SET_BIT(
	     cpu_controller_p->cpc_active_external_interrupts,
	     interrupt_channel);
    }

    struct rtos_execution_context *new_interrupt_context_p =
        &rtos_interrupt_p->int_execution_context;

    struct rtos_execution_context *interrupted_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

#   ifdef _CPU_CYCLES_MEASURE_
    /*
     * Track CPU usage for interrupted execution context:
     */
    cpu_clock_cycles_t used_cpu_cycles;

    if (_INFREQUENTLY_TRUE_(
	    interrupted_context_p->ctx_last_switched_in_time_stamp == 0 &&
	    interrupted_context_p->ctx_switched_out_counter == 0)) {
	/*
	 * Context interrupted before it got the chance to start executing
	 * after being chosen to run for the very first time
	 */
	used_cpu_cycles = 0;
    } else {
	cpu_clock_cycles_t end_cycles = get_cpu_clock_cycles();

	used_cpu_cycles =
	    CPU_CLOCK_CYCLES_DELTA(
		interrupted_context_p->ctx_last_switched_in_time_stamp,
		end_cycles);
#	if 0
        FDC_ASSERT(
	    used_cpu_cycles <
	    MILLISECONDS_TO_CPU_CLOCK_CYCLES(RTOS_THREAD_TIME_SLICE_IN_TICKS *
					     RTOS_MILLISECONDS_PER_TICK),
	    end_cycles, interrupted_context_p->ctx_last_switched_in_time_stamp);
#	endif

	if (used_cpu_cycles > g_McRTOS_p->rts_cpu_cycles_measure_overhead) {
	    used_cpu_cycles -= g_McRTOS_p->rts_cpu_cycles_measure_overhead;
	} else {
	    /*
	     * Context interrupted before it got the chance to start executing
	     * after being chosen to run
	     */
	    used_cpu_cycles = 0;
	}
    }

    RTOS_EXECUTION_CONTEXT_UPDATE_CPU_USAGE(
        interrupted_context_p, used_cpu_cycles);

    new_interrupt_context_p->ctx_last_switched_in_time_stamp = get_cpu_clock_cycles();
#   endif

#   ifdef _RELIABILITY_CHECKS_
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    fdc_info_p->fdc_interrupt_channel_counters[interrupt_channel] ++;
#   endif

    /*
     * Neither the calling interrupt context nor the interrupted execution context are
     * in the preemption chain:
     */

    DBG_ASSERT(
        GLIST_NODE_IS_UNLINKED(&new_interrupt_context_p->ctx_preemption_chain_node),
        &new_interrupt_context_p->ctx_preemption_chain_node,
        new_interrupt_context_p);

    DBG_ASSERT(
        GLIST_NODE_IS_UNLINKED(&interrupted_context_p->ctx_preemption_chain_node),
        &interrupted_context_p->ctx_preemption_chain_node,
        interrupted_context_p);

    DBG_ASSERT(
        cpu_controller_p->cpc_nested_interrupts_count < SOC_NUM_INTERRUPT_PRIORITIES,
        cpu_controller_p->cpc_nested_interrupts_count, cpu_controller_p);

    rtos_context_switch_type_t ctx_switch_type;

    if (cpu_controller_p->cpc_nested_interrupts_count == 0)
    {
        /*
         * The interrupted context is a thread:
         */
        FDC_ASSERT(
            interrupted_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
            interrupted_context_p->ctx_context_type, interrupted_context_p);

        struct rtos_thread *current_thread_p =
            RTOS_EXECUTION_CONTEXT_GET_THREAD(interrupted_context_p);

        DBG_ASSERT_RUNNING_THREAD_INVARIANTS(current_thread_p, cpu_controller_p);

        if (interrupted_context_p->
                ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
            CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP ||
	    interrupted_context_p->
                ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
            CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP_FPU)
        {
            /*
             * Add current thread at the beginning of the corresponding runnable
             * queue and change its state from running to runnable:
             */
             rtos_add_head_runnable_thread(
                cpu_controller_p, current_thread_p);

            ctx_switch_type = RTOS_CSW_THREAD_TO_INTERRUPT;
        }
        else
        {
            ctx_switch_type = RTOS_CSW_THREAD_TO_EARLY_NESTED_INTERRUPT;
        }
    }
    else
    {
        /*
         * The interrupted context is another interrupt:
         */
        FDC_ASSERT(
            interrupted_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            interrupted_context_p->ctx_context_type, interrupted_context_p);

        FDC_ASSERT(
            cpu_controller_p->cpc_nested_interrupts_count < SOC_NUM_INTERRUPT_PRIORITIES,
            cpu_controller_p->cpc_nested_interrupts_count, cpu_controller_p);

        ctx_switch_type = RTOS_CSW_ENTERING_NESTED_INTERRUPT;
    }

    cpu_controller_p->cpc_nested_interrupts_count ++;

    RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
        interrupted_context_p,
        CTX_SWITCHED_OUT_PREEMPTED_BY_INTERRUPT,
        cpu_controller_p);

    interrupted_context_p->ctx_last_preempted_by_p = new_interrupt_context_p;
    interrupted_context_p->ctx_preempted_counter ++;

    /*
     * Add interrupted execution context at the top of the preemption chain:
     */
    rtos_preemption_chain_push_context(cpu_controller_p, interrupted_context_p);

    /*
     * Set current execution context to the calling interrupt context:
     */
    FDC_TRACE_RTOS_CONTEXT_SWITCH(new_interrupt_context_p, ctx_switch_type);
    cpu_controller_p->cpc_current_execution_context_p = new_interrupt_context_p;

#   if DEFINED_ARM_CLASSIC_ARCH()
    /*
     * The stack pointer for an interrupt context always starts at the bottom
     * of the interrupt context's stack:
     */
    return (cpu_register_t)new_interrupt_context_p->ctx_execution_stack_bottom_end_p;
#   elif DEFINED_ARM_CORTEX_M_ARCH()
    /*
     * The value returned by this function is ignored by the caller.
     * For Cortex-M, a shared stack is used for all interrupts and exceptions. This
     * stack corresponds to the MSP stack pointer, which already the active stack
     * pointer at this point. So, we do no need to change the stack pointer for
     * the interrupt.
     */
    return 0x0;
#   else
#       error "CPU architecture not supported"
#   endif
}

#pragma GCC diagnostic pop

/**
 * Function to be invoked at the end of an assembly language ISR.
 *
 * @param   none
 *
 * @return  none
 */
void
rtos_k_exit_interrupt(void)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(current_context_p);

    /*
     * The current context is an interrupt:
     */
    DBG_ASSERT(
        current_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
        current_context_p->ctx_context_type, current_context_p);

    struct rtos_interrupt *current_interrupt_p =
        RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(current_context_p);

    DBG_ASSERT(
        current_interrupt_p->int_signature == RTOS_INTERRUPT_SIGNATURE,
        current_interrupt_p->int_signature, current_interrupt_p);

    interrupt_channel_t interrupt_channel = current_interrupt_p->int_channel;

    /*
     * Mark interrupt as inactive:
     */
    if (interrupt_channel < 0) {
        FDC_ASSERT(
            RTOS_INTR_BIT_MAP_GET_BIT(
		cpu_controller_p->cpc_active_internal_interrupts,
		-interrupt_channel) != 0,
            cpu_controller_p->cpc_active_internal_interrupts, -interrupt_channel);

	RTOS_INTR_BIT_MAP_CLEAR_BIT(
	     cpu_controller_p->cpc_active_internal_interrupts,
	     -interrupt_channel);
    } else {
        FDC_ASSERT(
            RTOS_INTR_BIT_MAP_GET_BIT(
		cpu_controller_p->cpc_active_external_interrupts,
		interrupt_channel) != 0,
            cpu_controller_p->cpc_active_external_interrupts, interrupt_channel);

	RTOS_INTR_BIT_MAP_CLEAR_BIT(
	     cpu_controller_p->cpc_active_external_interrupts,
	     interrupt_channel);
    }

    /*
     * Notify the interrupt controller that processing for the last interrupt
     * received by the calling CPU core has been completed, so that another
     * interrupt of the same priority or lower can be received by this CPU core:
     */
    notify_interrupt_controller_isr_done(interrupt_channel);

#   ifdef _CPU_CYCLES_MEASURE_
    /*
     * Track CPU usage for current execution context:
     */
    cpu_clock_cycles_t used_cpu_cycles =
        CPU_CLOCK_CYCLES_DELTA(
            current_context_p->ctx_last_switched_in_time_stamp,
            get_cpu_clock_cycles()) -
        g_McRTOS_p->rts_cpu_cycles_measure_overhead;

    RTOS_EXECUTION_CONTEXT_UPDATE_CPU_USAGE(
        current_context_p, used_cpu_cycles);
#   endif

    RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
        current_context_p,
        CTX_SWITCHED_OUT_INTERRUPT_SERVICED,
        cpu_controller_p);

    /*
     * Remove the execution context from the top of the preemption chain:
     */
    struct rtos_execution_context *preempted_context_p =
        rtos_preemption_chain_pop_context(cpu_controller_p);

    DBG_ASSERT(
        preempted_context_p != current_context_p,
        preempted_context_p, current_context_p);

    /*
     * Decrement interrupt nesting level:
     */

    FDC_ASSERT(
        cpu_controller_p->cpc_nested_interrupts_count > 0 &&
        cpu_controller_p->cpc_nested_interrupts_count <= SOC_NUM_INTERRUPT_PRIORITIES,
        cpu_controller_p->cpc_nested_interrupts_count, cpu_controller_p);

    cpu_controller_p->cpc_nested_interrupts_count --;

    if (cpu_controller_p->cpc_nested_interrupts_count == 0)
    {
        /*
         * The last preempted context was the current thread:
         */
        DBG_ASSERT(
            preempted_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
            preempted_context_p->ctx_context_type, preempted_context_p);

#	ifdef DEBUG
        struct rtos_thread *current_thread_p =
            RTOS_EXECUTION_CONTEXT_GET_THREAD(preempted_context_p);

        DBG_ASSERT(
            current_thread_p == cpu_controller_p->cpc_current_thread_p,
            current_thread_p, cpu_controller_p->cpc_current_thread_p);

        DBG_ASSERT_RTOS_THREAD_INVARIANTS(current_thread_p);
#	endif

        if (preempted_context_p->
                ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
            CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP ||
	    preempted_context_p->
                ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
            CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP_FPU)
        {
            /*
             * Call thread scheduler to ensure that the highest-priority
             * runnable thread gets to run as the new current context:
             *
             * NOTE: cpu_controller_p->cpc_current_execution_context_p will
             * be updated by rtos_k_restore_execution_context(), which is
             * called by rtos_thread_scheduler().
             */
            rtos_thread_scheduler(RTOS_CSW_INTERRUPT_TO_THREAD);
        }
        else
        {
            /*
             * Restore execution context of the interrupted thread
             *
             * NOTE: cpu_controller_p->cpc_current_execution_context_p will be
             * updated by rtos_k_restore_execution_context()
             */
            rtos_k_restore_execution_context(
                preempted_context_p,
                RTOS_CSW_EXITING_EARLY_NESTED_INTERRUPT);
        }
    }
    else
    {
        /*
         * The new current context is another interrupt that was preempted
         * by this interrupt:
         */
        DBG_ASSERT(
            preempted_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            preempted_context_p->ctx_context_type, preempted_context_p);

        /*
         * Restore execution context of the previous interrupt
         *
         * NOTE: cpu_controller_p->cpc_current_execution_context_p will be updated
         * by rtos_k_restore_execution_context()
         */
        rtos_k_restore_execution_context(
            preempted_context_p,
            RTOS_CSW_EXITING_NESTED_INTERRUPT);
    }

    /*
     * We should never come back here:
     */
    FDC_ASSERT(false, 0, 0);
}


fdc_error_t
rtos_k_enter_privileged_mode(
    _IN_ rtos_system_call_number_t system_call_number)
{
    fdc_error_t fdc_error = 0;

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    /*
     * The current context is a thread:
     */
    DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(current_context_p);

    DBG_ASSERT(
        current_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_context_p->ctx_context_type, current_context_p);

    DBG_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    if (system_call_number < RTOS_NUM_SYSTEM_CALLS) {
	DBG_ASSERT_VALID_FUNCTION_POINTER(
	    g_rtos_system_call_dispatch_table[system_call_number]);

    } else if (system_call_number != RTOS_ENTER_PRIVILEGED_MODE_SVC_CODE) {
        fdc_error = CAPTURE_FDC_ERROR("Invalid system call number",
                        system_call_number, current_context_p);
        goto Exit;
    }

    FDC_ASSERT(
        current_context_p->ctx_cpu_mode == RTOS_UNPRIVILEGED_THREAD_MODE,
        current_context_p->ctx_cpu_mode, system_call_number);

    current_context_p->ctx_cpu_mode = RTOS_PRIVILEGED_THREAD_MODE;

Exit:
    return fdc_error;
}


void
rtos_k_exit_privileged_mode(void)
{
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    /*
     * The current context is a thread:
     */
    DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(current_context_p);

    DBG_ASSERT(
        current_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_context_p->ctx_context_type, current_context_p);

    DBG_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    FDC_ASSERT(
        current_context_p->ctx_cpu_mode == RTOS_PRIVILEGED_THREAD_MODE,
        current_context_p->ctx_cpu_mode, current_context_p);

    current_context_p->ctx_cpu_mode = RTOS_UNPRIVILEGED_THREAD_MODE;
}


fdc_error_t
rtos_k_app_system_call(
    _IN_ rtos_app_system_call_function_t *rtos_app_system_call_function_p,
    _INOUT_ void *arg_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    return rtos_app_system_call_function_p(arg_p);
}


/**
 * Initializes an execution context
 *
 * @param execution_context_p   Pointer to the execution context
 *
 * @param context_name_p        Pointer to the symbolic name of the execution context
 *
 * @param cpu_id                CPU core on which this thread runs
 *
 * @param context_type          Execution context type
 *
 * @param prefilled_trace_entry Trace entry with some fields populated
 *
 * @param rtos_cpu_mode         McRTOS-level CPU privilege mode
 *
 * @param cpu_pc_register       Initial value for the PC register. Only meaningful for
 *                              thread contexts. It is address of the thread function.
 *
 * @param cpu_r0_register       Initial value for the r0 register. Only meaningful for
 *                              thread contexts. It is the parameter passed to the
 *                              thread function.
 *
 * @param stack_top_end_p       Pointer to the top end of the execution stack.
 *                              It points to the lowest entry of the stack.
 *
 * @param stack_bottom_end_p    Pointer to the bottom end of the execution stack.
 *                              It points to one entry after the highest entry of
 *                              the stack.
 */
void
rtos_execution_context_init(
    _OUT_ struct rtos_execution_context *execution_context_p,
    _IN_  const char *context_name_p,
    _IN_  cpu_id_t cpu_id,
    _IN_  rtos_execution_context_type_t context_type,
    _IN_  fdc_context_switch_trace_entry_t prefilled_trace_entry,
    _IN_  rtos_cpu_mode_t rtos_cpu_mode,
    _IN_  cpu_register_t cpu_pc_register,
    _IN_  cpu_register_t cpu_r0_register,
    _IN_  rtos_execution_stack_entry_t *stack_top_end_p,
    _IN_  rtos_execution_stack_entry_t *stack_bottom_end_p)
{
    DBG_ASSERT(execution_context_p != NULL, 0, 0);
    DBG_ASSERT(cpu_id < SOC_NUM_CPU_CORES, cpu_id, 0);

    if (context_type == RTOS_THREAD_CONTEXT)
    {
        FDC_ASSERT(
            rtos_cpu_mode == RTOS_UNPRIVILEGED_THREAD_MODE ||
            rtos_cpu_mode == RTOS_PRIVILEGED_THREAD_MODE,
            rtos_cpu_mode, execution_context_p);
    }
    else if (context_type == RTOS_INTERRUPT_CONTEXT)
    {
        FDC_ASSERT(
            rtos_cpu_mode == RTOS_INTERRUPT_MODE,
            rtos_cpu_mode, execution_context_p);
    }
    else
    {
        FDC_ASSERT(
            context_type == RTOS_RESET_CONTEXT,
            context_type, execution_context_p);

        FDC_ASSERT(
            rtos_cpu_mode == RTOS_RESET_MODE,
            rtos_cpu_mode, execution_context_p);
    }

    execution_context_p->ctx_signature = RTOS_EXECUTION_CONTEXT_SIGNATURE;
    execution_context_p->ctx_name_p = context_name_p;
    execution_context_p->ctx_cpu_id = cpu_id;
    execution_context_p->ctx_context_type = context_type;
    execution_context_p->ctx_cpu_mode = rtos_cpu_mode;
    execution_context_p->ctx_execution_stack_top_end_p = stack_top_end_p;
    execution_context_p->ctx_execution_stack_bottom_end_p = stack_bottom_end_p;
    execution_context_p->ctx_stack_pointer_high_water_mark_p = stack_bottom_end_p;
    execution_context_p->ctx_preempted_counter = 0;
    execution_context_p->ctx_switched_out_counter = 0;
#   ifdef _CPU_CYCLES_MEASURE_
    execution_context_p->ctx_last_switched_in_time_stamp = 0;
    execution_context_p->ctx_accumulated_cpu_usage_milliseconds = 0;
    execution_context_p->ctx_accumulated_cpu_usage_cycles = 0;
#   endif
    execution_context_p->ctx_last_switched_out_time_stamp_in_ticks = 0;
    execution_context_p->ctx_last_switched_out_reason = CTX_SWITCHED_OUT_NEVER;
    execution_context_p->ctx_switched_out_reason_history = 0x0;
    execution_context_p->ctx_last_preempted_by_p = NULL;
    execution_context_p->ctx_prefilled_trace_entry = prefilled_trace_entry;
    GLIST_NODE_INIT(&execution_context_p->ctx_list_node);
    GLIST_NODE_INIT(&execution_context_p->ctx_preemption_chain_node);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    glist_add_tail_elem(
        &cpu_controller_p->cpc_execution_contexts_list_anchor,
        &execution_context_p->ctx_list_node);

#if DEFINED_ARM_CLASSIC_ARCH()
    cpu_status_register_t cpu_status_register = 0;

    if (context_type == RTOS_THREAD_CONTEXT)
    {
        /*
         * Initialize CPU registers for the first time that the thread is switched in:
         */

        execution_context_p->ctx_cpu_registers[CPU_REG_R0] = cpu_r0_register;

        for (uint8_t i = CPU_REG_R1; i <= CPU_REG_R12; i ++)
        {
            execution_context_p->ctx_cpu_registers[i] = 0xFACE0000 + i;
        }

        /*
         * Initial value of the CPU status register for a thread has
         * CPU interrupts fully enabled: I_BIT = 0, F_BIT = 0
         */
        cpu_status_register &= ~(I_BIT | F_BIT);

        if (rtos_cpu_mode == RTOS_UNPRIVILEGED_THREAD_MODE)
        {
            execution_context_p->ctx_cpu_registers[CPU_REG_LR] =
                (cpu_register_t)rtos_thread_abort;

            /*
             * Initial value of the CPU status register for an unprivileged thread
             * uses User mode:
             */
            cpu_status_register |= ARM_MODE_USER;
        }
        else
        {
            execution_context_p->ctx_cpu_registers[CPU_REG_LR] =
                (cpu_register_t)rtos_k_thread_abort;

            /*
             * Initial value of the CPU status register for a privileged thread
             * uses System mode:
             */
            cpu_status_register |= ARM_MODE_SYS;
        }
    }
    else
    {
        execution_context_p->ctx_cpu_registers[CPU_REG_LR] =
                (cpu_register_t)rtos_invalid_interrupt_return;

        /*
         * NOTE: For interrupts, the initial value of the status register is not
         * used.
         */
    }

    execution_context_p->ctx_cpu_registers[CPU_REG_CPSR] = cpu_status_register;
    execution_context_p->ctx_cpu_registers[CPU_REG_SP] =
        (cpu_register_t)execution_context_p->ctx_execution_stack_bottom_end_p;

    execution_context_p->ctx_cpu_registers[CPU_REG_PC] = cpu_pc_register;

#elif DEFINED_ARM_CORTEX_M_ARCH()

    if (context_type == RTOS_THREAD_CONTEXT)
    {
        cpu_status_register_t cpu_status_register = 0;
        cpu_register_t cpu_lr_register = 0;
        cpu_register_t cpu_control_register = 0;

        /*
         * Initialize explicitly saved CPU registers, for the first time that
         * the thread is switched in:
         */
        cpu_register_t *saved_registers_p =
            &execution_context_p->ctx_cpu_saved_registers.cpu_reg_r4;

        uint8_t i;
        for (i = CPU_REG_R4; i <= CPU_REG_R11; i ++)
        {
            saved_registers_p[i] = 0xFACE0004 + i;
        }

        DBG_ASSERT(
            &saved_registers_p[i] ==
            &execution_context_p->ctx_cpu_saved_registers.cpu_reg_r11 + 1,
            &saved_registers_p[i],
            &execution_context_p->ctx_cpu_saved_registers.cpu_reg_r11 + 1);

        /*
         * Initial value of the CPU status register for a thread
         * - Exception number field is 0 (thread mode)
         * - Thumb mode bit set
         */
        cpu_status_register &= ~CPU_REG_IPSR_EXCEPTION_NUMBER_MASK;
        cpu_status_register |= CPU_REG_EPSR_THUMB_STATE_MASK;

	/*
	 * The SPSEL bit of the CONTROL register needs to be set to 1,
	 * to select the PSP register as the stack pointer:
	 */
        cpu_control_register |= CPU_REG_CONTROL_SPSEL_MASK;

        if (rtos_cpu_mode == RTOS_UNPRIVILEGED_THREAD_MODE)
        {
            cpu_lr_register = (cpu_register_t)rtos_thread_abort;

            /*
             * The nPRIV bit of the CONTROL register also needs to be set to 1
             * (unprivileged)
             */
            cpu_control_register |= CPU_REG_CONTROL_nPRIV_MASK;
        }
        else
        {
            cpu_lr_register = (cpu_register_t)rtos_k_thread_abort;
        }

        /*
         * For code compiled for thumb mode, addresses of functions must
         * have bit 0 set.
         */
        DBG_ASSERT(cpu_lr_register & 0x1, cpu_lr_register, 0);
        DBG_ASSERT(cpu_pc_register & 0x1, cpu_pc_register, 0);

        capture_fdc_msg_printf(
            "Created execution context \'%s\' (%#p)\n",
            execution_context_p->ctx_name_p,
            execution_context_p);

        /*
         * Initialize pre-saved registers in the thread's stack:
         *
         * NOTE: Since the initial context switch is implemented by returning from an
         * exception, we need to populate the thread's stack with the registers that
         * are pre-saved by the CPU, on the stack, upon exception entry:
         */
        uint32_t *stack_pointer =
            execution_context_p->ctx_execution_stack_bottom_end_p -
            CPU_NUM_PRE_SAVED_REGISTERS;

        stack_pointer[CPU_REG_R0] = cpu_r0_register;
        stack_pointer[CPU_REG_R1] = 0xFACE0001;
        stack_pointer[CPU_REG_R2] = 0xFACE0002;
        stack_pointer[CPU_REG_R3] = 0xFACE0003;
        stack_pointer[CPU_REG_R12] = 0xFACE000C;
        stack_pointer[CPU_REG_LR] = cpu_lr_register;
        stack_pointer[CPU_REG_PC] = cpu_pc_register;
        stack_pointer[CPU_REG_PSR] = cpu_status_register;

        execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp =
            (cpu_register_t)stack_pointer;

        execution_context_p->ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry =
            CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP;

        execution_context_p->ctx_cpu_saved_registers.cpu_reg_control =
            cpu_control_register;
    }

    /*
     * NOTE: if context_type is RTOS_RESET_CONTEXT or RTOS_INTERRUPT_CONTEXT,
     * we don't need not initialize any CPU registers.
     */

#else
#   error "CPU architrecture not supported"
#endif
}


void
rtos_k_console_putchar(
    _UNUSED_ void *unused_arg_p,
    _IN_ uint8_t c)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT(unused_arg_p == NULL, unused_arg_p, 0);

    uart_putchar(g_console_serial_port_p, c);
}


void
rtos_k_console_putchar_with_polling(
    _UNUSED_ void *unused_arg_p,
    _IN_ uint8_t c)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT(unused_arg_p == NULL, unused_arg_p, 0);

    uart_putchar_with_polling(g_console_serial_port_p, c);
}


uint8_t
rtos_k_console_getchar(void)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    return uart_getchar(g_console_serial_port_p);
}


#ifdef LCD_SUPPORTED

void
rtos_k_lcd_putchar(
    _INOUT_ struct rtos_lcd_putchar_attributes *lcd_putchar_attr_p,
    _IN_ uint8_t c)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT_VALID_RAM_POINTER(lcd_putchar_attr_p, sizeof(uint32_t));

    const struct lcd_char_attributes *lcd_char_attributes_p =
        lcd_putchar_attr_p->lcd_char_attributes_p;

    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(
        lcd_char_attributes_p, sizeof(uint32_t));

    lcd_x_t char_width_in_pixels =
        LCD_GET_CHAR_WIDTH_IN_PIXELS(lcd_char_attributes_p->lcd_dot_size);

    lcd_x_t char_height_in_pixels =
        LCD_GET_CHAR_HEIGHT_IN_PIXELS(lcd_char_attributes_p->lcd_dot_size);

    bool send_physical_output = true;

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    if (current_thread_p->thr_lcd_channel != RTOS_LCD_CHANNEL_NONE &&
        current_thread_p->thr_lcd_channel != g_McRTOS_p->rts_current_lcd_channel)
    {
        send_physical_output = false;
    }

    switch (c)
    {
    case '\n':
        lcd_putchar_attr_p->lcd_y += char_height_in_pixels;
        if (lcd_putchar_attr_p->lcd_y >= LCD_Y_SIZE)
        {
            TODO("Scroll LCD screen one line up")

            lcd_putchar_attr_p->lcd_y = 0;
        }
        break;

    case '\r':
        lcd_putchar_attr_p->lcd_x = 0;
        break;

    default:
        if (send_physical_output)
        {
            lcd_draw_char(
                lcd_putchar_attr_p->lcd_x,
                lcd_putchar_attr_p->lcd_y,
                lcd_putchar_attr_p->lcd_char_attributes_p,
                c);
        }

        /*
         * Move forward LCD char position:
         */
        lcd_putchar_attr_p->lcd_x += char_width_in_pixels;
        if (lcd_putchar_attr_p->lcd_x >= LCD_X_SIZE)
        {
            lcd_putchar_attr_p->lcd_x = 0;
            lcd_putchar_attr_p->lcd_y += char_height_in_pixels;
            if (lcd_putchar_attr_p->lcd_y >= LCD_Y_SIZE)
            {
                lcd_putchar_attr_p->lcd_y = 0;
            }
        }
    }
}


lcd_color_t
rtos_k_lcd_draw_tile(
    _IN_ lcd_x_t x,
    _IN_ lcd_y_t y,
    _IN_ lcd_color_t fill_color)
{
    bool send_physical_output = true;

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    if (current_thread_p->thr_lcd_channel != RTOS_LCD_CHANNEL_NONE &&
        current_thread_p->thr_lcd_channel != g_McRTOS_p->rts_current_lcd_channel)
    {
        send_physical_output = false;
    }

    if (send_physical_output)
    {
        return lcd_draw_tile(x, y, fill_color);
    }
    else
    {
        return LCD_COLOR_BLACK;
    }
}
#endif  /*  LCD_SUPPORTED */


/**
 * Set the calling thread's component data region to the address range
 * specified by (start_addr, size).
 *
 * @param   start_addr: start address of the region. It must be aligned to
 *          an MPU region boundary
 *
 * @param   size: size of the region. start_addr + size must be aligned
 *          to an MPU region boundary.
 *
 * @param   flags: access control flags (e.g., MPU_REGION_READ_ONLY)
 *
 * @param   old_comp_region_p: Pointer to area for returning previous component
 *          data region, or NULL
 *
 * @return  none
 */
void
rtos_k_thread_set_comp_region(
    _IN_ void *start_addr,
    _IN_ size_t size,
    _IN_ uint32_t flags,
    _OUT_ struct mpu_region_range *old_comp_region_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    FDC_ASSERT_VALID_RAM_POINTER(start_addr,
                                 MIN_MPU_REGION_ALIGNMENT);

    FDC_ASSERT(size != 0 && size % MIN_MPU_REGION_ALIGNMENT == 0,
               size, MIN_MPU_REGION_ALIGNMENT);

    FDC_ASSERT((flags & MPU_REGION_INACTIVE) == 0,
               flags, 0);

    void *end_addr = (void *)((uintptr_t)start_addr + size);

    /* check for wrap-around */
    FDC_ASSERT(start_addr < end_addr, start_addr, end_addr);

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    struct mpu_region_range *comp_region_p = &current_thread_p->thr_comp_region;

    if (old_comp_region_p != NULL) {
        *old_comp_region_p = *comp_region_p;
    }

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    comp_region_p->start_addr = start_addr;
    comp_region_p->end_addr = end_addr;
    comp_region_p->flags = flags;

    mpu_set_thread_data_region(cpu_id,
                               RTOS_THREAD_COMP_MPU_REGION_INDEX,
                               start_addr,
                               end_addr,
                               flags);

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Restore the calling thread's component data region to a previously
 * saved range (from a previous call to rtos_k_thread_set_comp_data_region())
 */
void
rtos_k_thread_restore_comp_region(
    _IN_ const struct mpu_region_range *old_comp_region_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    DBG_ASSERT((old_comp_region_p->flags & MPU_REGION_INACTIVE) ||
               (old_comp_region_p->start_addr < old_comp_region_p->end_addr &&
                old_comp_region_p->start_addr != NULL &&
                (uintptr_t)old_comp_region_p->start_addr % MIN_MPU_REGION_ALIGNMENT == 0 &&
                (uintptr_t)old_comp_region_p->end_addr % MIN_MPU_REGION_ALIGNMENT == 0),
               old_comp_region_p->start_addr, old_comp_region_p->end_addr);

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    current_thread_p->thr_comp_region = *old_comp_region_p;

    if (old_comp_region_p->flags & MPU_REGION_INACTIVE) {
        mpu_unset_thread_data_region(RTOS_THREAD_COMP_MPU_REGION_INDEX);
    } else {
        mpu_set_thread_data_region(cpu_id,
                                   RTOS_THREAD_COMP_MPU_REGION_INDEX,
                                   old_comp_region_p->start_addr,
                                   old_comp_region_p->end_addr,
                                   old_comp_region_p->flags);
    }

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Set the calling thread's temporary data region to the address range
 * specified by (start_addr, size).
 *
 * @param   start_addr: start address. If it is not aligned to
 *          an MPU region boundary, it will get rounded-down to
 *          the nearest MPU region boundary.
 *
 * @param   size: size of the region. If start_addr + size is not aligned
 *          to an MPU region boundary, it will be rounded-up to the
 *          nearest MPU region boundary.
 *
 * @param   flags: access control flags (e.g., MPU_REGION_READ_ONLY)
 *
 * @return  none
 */
void
rtos_k_thread_set_tmp_region(
    void *start_addr,
    size_t size,
    uint32_t flags)
{
    void *aligned_start_addr;
    void *aligned_end_addr;

    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    FDC_ASSERT(start_addr != NULL && size != 0, start_addr, size);

    void *end_addr = (void *)((uintptr_t)start_addr + size);

    /* check for wrap-around */
    FDC_ASSERT(start_addr < end_addr, start_addr, end_addr);

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    mpu_get_enclosing_region_boundaries(start_addr, end_addr,
                                        &aligned_start_addr, &aligned_end_addr);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    current_thread_p->thr_tmp_region.start_addr = aligned_start_addr;
    current_thread_p->thr_tmp_region.end_addr = aligned_end_addr;
    current_thread_p->thr_tmp_region.flags = flags;
    mpu_set_thread_data_region(cpu_id,
			       RTOS_THREAD_TMP_MPU_REGION_INDEX,
			       aligned_start_addr,
			       aligned_end_addr,
			       flags);

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Makes the calling thread's temporary data region inactive and
 * makes the corresponding MPU region invalid
 */
void
rtos_k_thread_unset_tmp_region(void)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    current_thread_p->thr_tmp_region.flags = MPU_REGION_INACTIVE;
    mpu_unset_thread_data_region(RTOS_THREAD_TMP_MPU_REGION_INDEX);

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Generate the function that initialize a generic circular buffer of entries of
 * a given type
 */
#define GEN_FUNCTION_CIRCULAR_BUFFER_INIT(                                  \
            _circular_buffer_init_func, _entry_type, _signature)            \
        void                                                                \
        _circular_buffer_init_func(                                         \
            _IN_  const char *name_p,                                       \
            _IN_ uint16_t num_entries,                                      \
            _IN_ _entry_type *storage_array_p,                              \
            _IN_ struct rtos_mutex *mutex_p,                                \
            _OUT_ struct rtos_circular_buffer *circ_buf_p)                  \
        {                                                                   \
            FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(name_p, sizeof(char));      \
            FDC_ASSERT_VALID_RAM_POINTER(                                   \
                storage_array_p, sizeof(_entry_type));                      \
            FDC_ASSERT_VALID_RAM_POINTER(circ_buf_p, sizeof(uint32_t));     \
            FDC_ASSERT(num_entries != 0, 0, 0);                             \
            if (mutex_p != NULL) {                                          \
                FDC_ASSERT_VALID_RAM_POINTER(mutex_p, sizeof(uint32_t));    \
                FDC_ASSERT(                                                 \
                    mutex_p->mtx_signature == RTOS_MUTEX_SIGNATURE,         \
                    mutex_p->mtx_signature, mutex_p);                       \
            }                                                               \
                                                                            \
            *((uint32_t *)&circ_buf_p->cb_signature) = _signature;          \
            *((const char **)&circ_buf_p->cb_name_p) = name_p;              \
            *((uint16_t *)&circ_buf_p->cb_num_entries) = num_entries;       \
            *((void **)&circ_buf_p->cb_storage_array_p) = storage_array_p;  \
            *((struct rtos_mutex **)&circ_buf_p->cb_mutex_p) = mutex_p;     \
            circ_buf_p->cb_entries_filled = 0;                              \
            circ_buf_p->cb_read_cursor = 0;                                 \
            circ_buf_p->cb_write_cursor = 0;                                \
            circ_buf_p->cb_write_cursor = 0;                                \
                                                                            \
            rtos_k_condvar_init(                                            \
                name_p, &circ_buf_p->cb_not_empty_condvar);		    \
            rtos_k_condvar_init(                                            \
                name_p, &circ_buf_p->cb_not_full_condvar);		    \
                                                                            \
            check_circular_buffer_invariants(circ_buf_p, _signature);       \
        }


/**
 * Generates the function that writes an entry to a circular buffer of entries
 * of a given type, if the buffer is not full.
 * If the buffer is full there are two cases:
 * - if the wait_if_full flag is true, the caller gets blocked on a
 *   condvar until the buffer becomes not full.
 * - If wait_if_full is false, the function returns false
 *
 * NOTE: if the caller is an interrupt handler, it is illegal to pass true for
 * the wait_if_full flag.
 */
#define GEN_FUNCTION_CIRCULAR_BUFFER_WRITE(                                 \
            _circular_buffer_write_func, _entry_type, _signature)           \
        bool                                                                \
        _circular_buffer_write_func(                                        \
            _INOUT_ struct rtos_circular_buffer *circ_buf_p,                \
            _IN_ _entry_type entry_value,                                   \
            _IN_ bool wait_if_full)                                         \
        {                                                                   \
            cpu_status_register_t saved_cpu_intr_mask = 0;                  \
            bool entry_written = false;                                     \
                                                                            \
            if (circ_buf_p->cb_mutex_p == NULL) {                           \
                saved_cpu_intr_mask = rtos_k_disable_cpu_interrupts();      \
                if (circ_buf_p->cb_entries_filled ==                        \
                    circ_buf_p->cb_num_entries) {                           \
                    if (wait_if_full) {                                     \
                        do {                                                \
                            rtos_k_condvar_wait_intr_disabled(              \
                                &circ_buf_p->cb_not_full_condvar, NULL);    \
                        } while (circ_buf_p->cb_entries_filled ==           \
                                 circ_buf_p->cb_num_entries);               \
                    } else {                                                \
                        goto exit;                                          \
                    }                                                       \
                }                                                           \
            } else {                                                        \
                rtos_k_mutex_acquire(circ_buf_p->cb_mutex_p);               \
                if (circ_buf_p->cb_entries_filled ==                        \
                    circ_buf_p->cb_num_entries) {                           \
                    if (wait_if_full) {                                     \
                        do {                                                \
                            rtos_k_condvar_wait(                            \
                                &circ_buf_p->cb_not_full_condvar,           \
                                circ_buf_p->cb_mutex_p, NULL);              \
                        } while (circ_buf_p->cb_entries_filled ==           \
                                 circ_buf_p->cb_num_entries);               \
                    } else {                                                \
                        goto exit;                                          \
                    }                                                       \
                }                                                           \
            }                                                               \
                                                                            \
            check_circular_buffer_invariants(circ_buf_p, _signature);       \
            FDC_ASSERT(                                                     \
                circ_buf_p->cb_entries_filled < circ_buf_p->cb_num_entries, \
                circ_buf_p->cb_entries_filled, circ_buf_p);                 \
                                                                            \
            ((_entry_type *)circ_buf_p->cb_storage_array_p)[                \
                circ_buf_p->cb_write_cursor] = entry_value;                 \
                                                                            \
            circ_buf_p->cb_entries_filled ++;                               \
            circ_buf_p->cb_write_cursor ++;                                 \
            if (circ_buf_p->cb_write_cursor ==                              \
                circ_buf_p->cb_num_entries) {                               \
                circ_buf_p->cb_write_cursor = 0;                            \
            }                                                               \
                                                                            \
            entry_written = true;                                           \
        exit:                                                               \
            if (circ_buf_p->cb_mutex_p == NULL) {                           \
                rtos_k_restore_cpu_interrupts(saved_cpu_intr_mask);         \
            } else {                                                        \
                rtos_k_mutex_release(circ_buf_p->cb_mutex_p);               \
            }                                                               \
                                                                            \
            if (entry_written) {                                            \
                rtos_k_condvar_signal(&circ_buf_p->cb_not_empty_condvar);   \
            }                                                               \
                                                                            \
            return entry_written;                                           \
        }


/**
 * Generates the function that reads an entry from a circular buffer of entries
 * of a given type, if the buffer is not empty.
 * If the buffer is empty there are two cases:
 * - if the wait_if_empty flag is true, the caller gets blocked on a
 *   condvar until the buffer becomes not empty.
 * - If wait_if_empty is false, the function returns false
 *
 * NOTE: if the caller is an interrupt handler, it is illegal to pass 'true' for
 * the wait_if_empty flag.
 */
#define GEN_FUNCTION_CIRCULAR_BUFFER_READ(                                  \
            _circular_buffer_read_func, _entry_type, _signature)            \
        bool                                                                \
        _circular_buffer_read_func(                                         \
            _INOUT_ struct rtos_circular_buffer *circ_buf_p,                \
            _OUT_ _entry_type *entry_value_p,                               \
            _IN_ bool wait_if_empty,					    \
	    _INOUT_ rtos_milliseconds_t *timeout_ms_p)			    \
        {                                                                   \
            cpu_status_register_t saved_cpu_intr_mask = 0;                  \
            bool entry_read = false;                                        \
                                                                            \
            if (circ_buf_p->cb_mutex_p == NULL) {                           \
                saved_cpu_intr_mask = rtos_k_disable_cpu_interrupts();      \
                if (circ_buf_p->cb_entries_filled == 0) {                   \
                    if (wait_if_empty) {                                    \
                        do {                                                \
                            rtos_k_condvar_wait_intr_disabled(              \
                                &circ_buf_p->cb_not_empty_condvar,	    \
				timeout_ms_p);				    \
			    if (timeout_ms_p != NULL &&	*timeout_ms_p == 0) \
				goto exit;                                  \
                        } while (circ_buf_p->cb_entries_filled == 0);       \
                    } else {                                                \
                        goto exit;                                          \
                    }                                                       \
                }                                                           \
            } else {                                                        \
                rtos_k_mutex_acquire(circ_buf_p->cb_mutex_p);               \
                if (circ_buf_p->cb_entries_filled == 0) {                   \
                    if (wait_if_empty) {                                    \
                        do {                                                \
                            rtos_k_condvar_wait(                            \
                                &circ_buf_p->cb_not_empty_condvar,          \
                                circ_buf_p->cb_mutex_p,                     \
				timeout_ms_p);				    \
			    if (timeout_ms_p != NULL &&	*timeout_ms_p == 0) \
				goto exit;                                  \
                        } while (circ_buf_p->cb_entries_filled == 0);       \
                    } else {                                                \
                        goto exit;                                          \
                    }                                                       \
                }                                                           \
            }                                                               \
                                                                            \
            check_circular_buffer_invariants(circ_buf_p, _signature);       \
            FDC_ASSERT(                                                     \
                circ_buf_p->cb_entries_filled > 0,                          \
                circ_buf_p->cb_entries_filled, circ_buf_p);                 \
                                                                            \
            if (entry_value_p != NULL) {                                    \
                *entry_value_p =                                            \
                    ((_entry_type *)circ_buf_p->cb_storage_array_p)[        \
                    circ_buf_p->cb_read_cursor];                            \
            }                                                               \
                                                                            \
            circ_buf_p->cb_entries_filled --;                               \
            circ_buf_p->cb_read_cursor ++;                                  \
            if (circ_buf_p->cb_read_cursor == circ_buf_p->cb_num_entries) { \
                circ_buf_p->cb_read_cursor = 0;                             \
            }                                                               \
                                                                            \
            entry_read = true;                                              \
        exit:                                                               \
            if (circ_buf_p->cb_mutex_p == NULL) {                           \
                rtos_k_restore_cpu_interrupts(saved_cpu_intr_mask);         \
            } else {                                                        \
                rtos_k_mutex_release(circ_buf_p->cb_mutex_p);               \
            }                                                               \
                                                                            \
            if (entry_read) {                                               \
                rtos_k_condvar_signal(&circ_buf_p->cb_not_full_condvar);    \
            }                                                               \
                                                                            \
            return entry_read;                                              \
        }


GEN_FUNCTION_CIRCULAR_BUFFER_INIT(
    rtos_k_pointer_circular_buffer_init, void *, RTOS_POINTER_CIRCULAR_BUFFER_SIGNATURE)

GEN_FUNCTION_CIRCULAR_BUFFER_WRITE(
    rtos_k_pointer_circular_buffer_write, void *, RTOS_POINTER_CIRCULAR_BUFFER_SIGNATURE)

GEN_FUNCTION_CIRCULAR_BUFFER_READ(
    rtos_k_pointer_circular_buffer_read, void *, RTOS_POINTER_CIRCULAR_BUFFER_SIGNATURE)

GEN_FUNCTION_CIRCULAR_BUFFER_INIT(
    rtos_k_byte_circular_buffer_init, uint8_t, RTOS_BYTE_CIRCULAR_BUFFER_SIGNATURE)

GEN_FUNCTION_CIRCULAR_BUFFER_WRITE(
    rtos_k_byte_circular_buffer_write, uint8_t, RTOS_BYTE_CIRCULAR_BUFFER_SIGNATURE)

GEN_FUNCTION_CIRCULAR_BUFFER_READ(
    rtos_k_byte_circular_buffer_read, uint8_t, RTOS_BYTE_CIRCULAR_BUFFER_SIGNATURE)


/**
 * Checks if a circular buffer is empty
 */
bool
rtos_k_circular_buffer_is_empty(_IN_ struct rtos_circular_buffer *circ_buf_p)
{
    bool is_empty;

    FDC_ASSERT(circ_buf_p->cb_signature == RTOS_POINTER_CIRCULAR_BUFFER_SIGNATURE ||
	       circ_buf_p->cb_signature == RTOS_BYTE_CIRCULAR_BUFFER_SIGNATURE,
	       circ_buf_p->cb_signature, circ_buf_p);

    if (circ_buf_p->cb_mutex_p == NULL) {
	cpu_status_register_t saved_cpu_intr_mask =
				rtos_k_disable_cpu_interrupts();

	is_empty = (circ_buf_p->cb_entries_filled == 0);
	rtos_k_restore_cpu_interrupts(saved_cpu_intr_mask);
    } else {
	rtos_k_mutex_acquire(circ_buf_p->cb_mutex_p);
	is_empty = (circ_buf_p->cb_entries_filled == 0);
	rtos_k_mutex_release(circ_buf_p->cb_mutex_p);
    }

    return is_empty;
}


/**
 * Check invariants of a circular buffer
 */
static void
check_circular_buffer_invariants(
    _IN_ struct rtos_circular_buffer *circ_buf_p,
    _IN_ uint32_t signature)
{
    FDC_ASSERT(
        circ_buf_p->cb_signature == signature,
        circ_buf_p->cb_signature, circ_buf_p);

    FDC_ASSERT(
        circ_buf_p->cb_entries_filled <= circ_buf_p->cb_num_entries,
        circ_buf_p->cb_entries_filled, circ_buf_p);

    FDC_ASSERT(
        circ_buf_p->cb_write_cursor < circ_buf_p->cb_num_entries,
        circ_buf_p->cb_write_cursor, circ_buf_p);

    FDC_ASSERT(
        circ_buf_p->cb_read_cursor < circ_buf_p->cb_num_entries,
        circ_buf_p->cb_read_cursor, circ_buf_p);
}


/**
 * Initializes a generic queue
 */
void
rtos_k_queue_init(
    _IN_  const char *queue_name_p,
    _IN_ bool use_mutex,
    _OUT_ struct rtos_queue *queue_p)
{
    queue_p->signature = RTOS_QUEUE_SIGNATURE;
    GLIST_NODE_INIT(&queue_p->list_anchor);
    queue_p->use_mutex = use_mutex;
    if (use_mutex) {
	rtos_k_mutex_init(queue_name_p, &queue_p->mutex);
    }

    rtos_k_condvar_init(queue_name_p, &queue_p->non_empty_condvar);
}


/**
 * Adds an element at the end of a generic queue
 */
void
rtos_k_queue_add(
    _INOUT_ struct rtos_queue *queue_p,
    _INOUT_ struct glist_node *elem_p)
{
    FDC_ASSERT(queue_p->signature == RTOS_QUEUE_SIGNATURE,
	       queue_p->signature, queue_p);

    if (queue_p->use_mutex) {
	rtos_k_mutex_acquire(&queue_p->mutex);
	glist_add_tail_elem(&queue_p->list_anchor, elem_p);
	rtos_k_mutex_release(&queue_p->mutex);
    } else {
	cpu_status_register_t saved_cpu_intr_mask = rtos_k_disable_cpu_interrupts();
	glist_add_tail_elem(&queue_p->list_anchor, elem_p);
        rtos_k_restore_cpu_interrupts(saved_cpu_intr_mask);
    }

    rtos_k_condvar_signal(&queue_p->non_empty_condvar);
}


/**
 * Removes the element from the head of a generic queue, if the queue
 * is not empty. Otherwise, it waits until the queue becomes non-empty.
 * If timeout_ms is not 0, The wait will timeout at the specified
 * milliseconds value.
 *
 * @param queue_p	Pointer to the queue object
 * @param timeout_ms	0, or timeout for waiting for the queue to become
 *			non-empty
 *
 * @return pointer to element removed from the queue, or NULL if timeout
 */
struct glist_node *
rtos_k_queue_remove(
    _INOUT_ struct rtos_queue *queue_p,
    _IN_ rtos_milliseconds_t timeout_ms)
{
    struct glist_node *elem_p = NULL;
    cpu_status_register_t saved_cpu_intr_mask = 0;

    FDC_ASSERT(queue_p->signature == RTOS_QUEUE_SIGNATURE,
	       queue_p->signature, queue_p);

    if (queue_p->use_mutex) {
	rtos_k_mutex_acquire(&queue_p->mutex);
    } else {
	saved_cpu_intr_mask = rtos_k_disable_cpu_interrupts();
    }

    while (GLIST_IS_EMPTY(&queue_p->list_anchor)) {
	if (timeout_ms != 0) {
	    rtos_milliseconds_t tmp_timeout_ms = timeout_ms;

	    if (queue_p->use_mutex) {
		rtos_k_condvar_wait(
		    &queue_p->non_empty_condvar,
		    &queue_p->mutex,
		    &tmp_timeout_ms);
	    } else {
		rtos_k_condvar_wait_intr_disabled(
		    &queue_p->non_empty_condvar,
		    &tmp_timeout_ms);
	    }

	    if (tmp_timeout_ms == 0) {
		goto common_exit;
	    }
	} else {
	    if (queue_p->use_mutex) {
		rtos_k_condvar_wait(
		    &queue_p->non_empty_condvar,
		    &queue_p->mutex,
		    NULL);
	    } else {
		rtos_k_condvar_wait_intr_disabled(
		    &queue_p->non_empty_condvar,
		    NULL);
	    }
	}
    }

    elem_p = GLIST_GET_FIRST(&queue_p->list_anchor);
    glist_remove_elem(elem_p);

common_exit:
    if (queue_p->use_mutex) {
	rtos_k_mutex_release(&queue_p->mutex);
    } else {
        rtos_k_restore_cpu_interrupts(saved_cpu_intr_mask);
    }

    return elem_p;
}


#if DEFINED_ARM_CORTEX_M_ARCH()

/**
 * Function that disables interrupts and returns the previous interrupt mask
 *
 * @return  Original value of the CPU PRIMASK register
 */
cpu_status_register_t
rtos_k_disable_cpu_interrupts(void)
{
    cpu_status_register_t old_primask = __get_PRIMASK();

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
	__disable_irq();
	__ISB();
	RTOS_START_INTERRUPTS_DISABLED_TIME_MEASURE();
    }

    return old_primask;
}


/**
 * Function that restores (and possibly enables) interrupts
 *
 * @param   cpu_status_register: Value of the CPU status register to be
 *          restored
 */
 void
 rtos_k_restore_cpu_interrupts(cpu_register_t old_primask)
{
    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
	RTOS_STOP_INTERRUPTS_DISABLED_TIME_MEASURE();
        __ISB();
        __enable_irq();
    }
}


/**
 * Increments atomically the 32-bit value stored in *counter_p, and returns the
 * original value.
 *
 * @param   counter_p: Pointer to the counter to be incremented.
 *
 * @param   value: Increment value.
 *
 * @return  value of the counter prior to the increment.
 */
uint32_t
rtos_k_atomic_fetch_add_uint32(volatile uint32_t *counter_p, uint32_t value)
{
#if 0
    cpu_status_register_t old_primask = __get_PRIMASK();

    __disable_irq();

    uint32_t old_value = *counter_p;

    *counter_p += value;

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
        __enable_irq();
    }

    return old_value;
#else
    uint32_t old_value;

    do {
	old_value = __LDREXW(counter_p);
    } while (__STREXW(old_value + value, counter_p) != 0);

    return old_value;
#endif
}


/**
 * Decrements atomically the 32-bit value stored in *counter_p, and returns the
 * original value.
 *
 * @param   counter_p: Pointer to the counter to be decremented.
 *
 * @param   value: Decrement value.
 *
 * @return  value of the counter prior to the decrement.
 */
uint32_t
rtos_k_atomic_fetch_sub_uint32(volatile uint32_t *counter_p, uint32_t value)
{
#if 0
    cpu_status_register_t old_primask = __get_PRIMASK();

    __disable_irq();

    uint32_t old_value = *counter_p;

    *counter_p -= value;

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
        __enable_irq();
    }

    return old_value;
#else
    uint32_t old_value;

    do {
	old_value = __LDREXW(counter_p);
    } while (__STREXW(old_value - value, counter_p) != 0);

    return old_value;
#endif
}


/**
 * Increments atomically the 16-bit value stored in *counter_p, and returns the
 * original value.
 *
 * @param   counter_p: Pointer to the counter to be incremented.
 *
 * @param   value: Increment value.
 *
 * @return  value of the counter prior to the increment.
 */
uint16_t
rtos_k_atomic_fetch_add_uint16(volatile uint16_t *counter_p, uint16_t value)
{
#if 0
    cpu_status_register_t old_primask = __get_PRIMASK();

    __disable_irq();

    uint16_t old_value = *counter_p;

    *counter_p += value;

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
        __enable_irq();
    }

    return old_value;
#else
    uint16_t old_value;

    do {
	old_value = __LDREXH(counter_p);
    } while (__STREXH(old_value + value, counter_p) != 0);

    return old_value;
#endif
}


/**
 * Decrements atomically the 16-bit value stored in *counter_p, and returns the
 * original value.
 *
 * @param   counter_p: Pointer to the counter to be decremented.
 *
 * @param   value: Decrement value.
 *
 * @return  value of the counter prior to the decrement.
 */
uint16_t
rtos_k_atomic_fetch_sub_uint16(volatile uint16_t *counter_p, uint16_t value)
{
#if 0
    cpu_status_register_t old_primask = __get_PRIMASK();

    __disable_irq();

    uint16_t old_value = *counter_p;

    *counter_p -= value;

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
        __enable_irq();
    }

    return old_value;
#else
    uint16_t old_value;

    do {
	old_value = __LDREXH(counter_p);
    } while (__STREXH(old_value - value, counter_p) != 0);

    return old_value;
#endif
}


/**
 * Increments atomically the 8-bit value stored in *counter_p, and returns the
 * original value.
 *
 * @param   counter_p: Pointer to the counter to be incremented.
 *
 * @param   value: Increment value.
 *
 * @return  value of the counter prior to the increment.
 */
uint8_t
rtos_k_atomic_fetch_add_uint8(volatile uint8_t *counter_p, uint8_t value)
{
#if 0
    cpu_status_register_t old_primask = __get_PRIMASK();

    __disable_irq();

    uint8_t old_value = *counter_p;

    *counter_p += value;

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
        __enable_irq();
    }

    return old_value;
#else
    uint8_t old_value;

    do {
	old_value = __LDREXB(counter_p);
    } while (__STREXB(old_value + value, counter_p) != 0);

    return old_value;
#endif
}


/**
 * Decrements atomically the 8-bit value stored in *counter_p, and returns the
 * original value.
 *
 * @param   counter_p: Pointer to the counter to be decremented.
 *
 * @param   value: Decrement value.
 *
 * @return  value of the counter prior to the decrement.
 */
uint8_t
rtos_k_atomic_fetch_sub_uint8(volatile uint8_t *counter_p, uint8_t value)
{
#if 0
    cpu_status_register_t old_primask = __get_PRIMASK();

    __disable_irq();

    uint8_t old_value = *counter_p;

    *counter_p -= value;

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
        __enable_irq();
    }

    return old_value;
#else
    uint8_t old_value;

    do {
	old_value = __LDREXB(counter_p);
    } while (__STREXB(old_value - value, counter_p) != 0);

    return old_value;
#endif
}


/**
 * Find the highest thread priority bit set in the passed in thread priority
 * bitmap.
 *
 * @param   rtos_thread_prio_bitmap: Thread priority bitmap
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
 */
rtos_thread_prio_t
rtos_k_find_highest_thread_priority(
    rtos_thread_prio_bitmap_t rtos_thread_prio_bitmap)
{
#if __CORTEX_M >= 0x03
    uint32_t leading_zeros_count = __CLZ(rtos_thread_prio_bitmap);

#   if RTOS_NUM_THREAD_PRIORITIES < 32
    leading_zeros_count -= (ARM_CPU_WORD_SIZE_IN_BITS - RTOS_NUM_THREAD_PRIORITIES);
#   endif

#else
    /*
     * Since Cortex-M0+ does not have the CLZ instruction, we have to manually
     * emulate the behavior of CLZ. The algorithm used here was taken from
     * section 7.2.2 of the ARM System Developer's Guide book.
     */
    uint32_t leading_zeros_count = 0;

#   if RTOS_NUM_THREAD_PRIORITIES == 32
    if (rtos_thread_prio_bitmap < BIT(16)) {
        rtos_thread_prio_bitmap <<= 16;
        leading_zeros_count += 16;
    }

    if (rtos_thread_prio_bitmap < BIT(24)) {
         rtos_thread_prio_bitmap <<= 8;
         leading_zeros_count += 8;
    }

    if (rtos_thread_prio_bitmap < BIT(28)) {
        rtos_thread_prio_bitmap <<= 4;
        leading_zeros_count += 4;
    }

    if (rtos_thread_prio_bitmap < BIT(30)) {
        rtos_thread_prio_bitmap <<= 2;
        leading_zeros_count += 2;
    }

    if (rtos_thread_prio_bitmap < BIT(31)) {
        leading_zeros_count += 1;
        rtos_thread_prio_bitmap <<= 1;
        if (rtos_thread_prio_bitmap == 0) {
            leading_zeros_count = 32;
        }
    }

#   elif RTOS_NUM_THREAD_PRIORITIES == 16
    if (rtos_thread_prio_bitmap < BIT(8)) {
        rtos_thread_prio_bitmap <<= 8;
        leading_zeros_count += 8;
    }

    if (rtos_thread_prio_bitmap < BIT(12)) {
        rtos_thread_prio_bitmap <<= 4;
        leading_zeros_count += 4;
    }

    if (rtos_thread_prio_bitmap < BIT(14)) {
        rtos_thread_prio_bitmap <<= 2;
        leading_zeros_count += 2;
    }

    if (rtos_thread_prio_bitmap < BIT(15)) {
        leading_zeros_count += 1;
        rtos_thread_prio_bitmap <<= 1;
        if (rtos_thread_prio_bitmap == 0) {
            leading_zeros_count = 16;
        }
    }

#   elif RTOS_NUM_THREAD_PRIORITIES == 8

    if (rtos_thread_prio_bitmap < BIT(4)) {
        rtos_thread_prio_bitmap <<= 4;
        leading_zeros_count += 4;
    }

    if (rtos_thread_prio_bitmap < BIT(6)) {
        rtos_thread_prio_bitmap <<= 2;
        leading_zeros_count += 2;
    }

    if (rtos_thread_prio_bitmap < BIT(7)) {
        leading_zeros_count += 1;
        rtos_thread_prio_bitmap <<= 1;
        if (rtos_thread_prio_bitmap == 0) {
            leading_zeros_count = 8;
        }
    }
#   else
#       error "Invalid RTOS_NUM_THREAD_PRIORITIES"
#   endif

#endif /* !(__CORTEX_M >= 0x03) */

    return leading_zeros_count;
}

#endif /* DEFINED_ARM_CORTEX_M_ARCH() */
