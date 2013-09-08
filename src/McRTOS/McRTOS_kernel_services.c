/**
 * @file McRTOS_kernel_services.c
 *
 * McRTOS kernel services. These functions can only be invoked from
 * privileged code.
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

static void
rtos_execution_context_init(
    _OUT_ struct rtos_execution_context *execution_context_p,
    _IN_  const char *context_name_p,
    _IN_  cpu_id_t cpu_id,
    _IN_  rtos_execution_context_type_t context_type,
    _IN_  fdc_context_switch_trace_entry_t prefilled_trace_entry,
    _IN_  rtos_cpu_mode_t rtos_cpu_mode,
    _IN_  cpu_status_register_t cpu_status_register,
    _IN_  cpu_register_t cpu_pc_register,
    _IN_  cpu_register_t cpu_r0_register,
    _IN_  rtos_execution_stack_entry_t *stack_top_end_p,
    _IN_  rtos_execution_stack_entry_t *stack_bottom_end_p);

static void
rtos_invalid_interrupt_return(void);

static void
rtos_k_condvar_signal_internal(
    _INOUT_ struct rtos_condvar *rtos_condvar_p,
    _IN_ bool broadcast);

#if defined(LPC2478_SOC)
/**
 * global variable used by the spinlock to serialize atomic operations
 * across multiple CPU cores
 */
volatile uint32_t g_rtos_atomic_ops_spinlock = 0x0;

#endif

#define GEN_SYSTEM_CALL_DISPATCH_ENTRY(_system_call_name,               \
                                       _rtos_k_function_suffix)         \
        [RTOS_ ## _system_call_name ## _SYSTEM_CALL] =                  \
            rtos_k_ ## _rtos_k_function_suffix

/**
 * System call dispatch table
 *
 * NOTE: This variable is accessed from the fdc_swi_instruction_handler
 * assembly language routine
 */
const void *const g_rtos_system_call_dispatch_table[] =
{
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CREATE_THREAD, create_thread),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(THREAD_DELAY, thread_delay),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(THREAD_ABORT, thread_abort),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(THREAD_CONDVAR_WAIT, thread_condvar_wait),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(THREAD_CONDVAR_SIGNAL, thread_condvar_signal),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CREATE_MUTEX, create_mutex),         
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(MUTEX_ACQUIRE, mutex_acquire),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(MUTEX_RELEASE, mutex_release),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CREATE_CONDVAR, create_condvar),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CONDVAR_WAIT, condvar_wait),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CONDVAR_WAIT_INTERRUPT, condvar_wait_interrupt),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CONDVAR_SIGNAL, condvar_signal),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CONDVAR_BROADCAST, condvar_broadcast),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CREATE_TIMER, create_timer),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(TIMER_START, timer_start),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(TIMER_STOP, timer_stop),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CAPTURE_FAILURE_DATA, capture_failure_data),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CONSOLE_PUTCHAR, console_putchar),             
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(CONSOLE_GETCHAR, console_getchar),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(LCD_PUTCHAR, lcd_putchar),             
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(LCD_DRAW_TILE, lcd_draw_tile),             
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(APP, app_system_call),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(THREAD_SELF, thread_self),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(THREAD_NAME, thread_name),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(THREAD_CONDVAR_WAIT_INTERRUPT, thread_condvar_wait_interrupt),
    GEN_SYSTEM_CALL_DISPATCH_ENTRY(THREAD_YIELD, thread_yield),
};

C_ASSERT(ARRAY_SIZE(g_rtos_system_call_dispatch_table) == RTOS_NUM_SYSTEM_CALLS);

/**
 * Create a McRTOS application thread on the calling CPU
 *
 * @param params_p Pointer to the thread's creation parameters.
 *
 * @return 0, on success
 *         Non-zero error code, on failure
 */
fdc_error_t
rtos_k_create_thread(
    _IN_ const struct rtos_thread_creation_params *params_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(params_p, sizeof(void *));

    fdc_error_t fdc_error = 0;
    bool fatal_error = false;
    struct rtos_thread_execution_stack *thread_stack_p = NULL;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    /*
     * Allocate a thread object:
     */
    struct rtos_thread *new_thread_p =
        ATOMIC_POST_INCREMENT_POINTER(g_McRTOS_p->rts_next_free_app_thread_p);

    if (new_thread_p >= &g_McRTOS_p->rts_app_threads[RTOS_MAX_NUM_APP_THREADS])
    {
        fdc_error = CAPTURE_FDC_ERROR(
                        "No more application threads can be created",
                        new_thread_p,
                        g_McRTOS_p->rts_next_free_app_thread_p);

        /*
         * This is considered a fatal error
         */
        fatal_error = true;
        goto Exit;
    }

    /*
     * Allocate an execution stack for the thread:
     */
    thread_stack_p =
        ATOMIC_POST_INCREMENT_POINTER(
            g_McRTOS_p->rts_next_free_app_thread_stack_p);

    if (thread_stack_p >=
            &g_McRTOS_p->rts_app_threads_execution_stacks_p[
                RTOS_MAX_NUM_APP_THREADS])
    {
        fdc_error = CAPTURE_FDC_ERROR(
                        "No more stacks for application threads can be created",
                        thread_stack_p,
                        g_McRTOS_p->rts_next_free_app_thread_stack_p);

        /*
         * This is considered a fatal error
         */
        fatal_error = true;
        goto Exit;
    }

#ifdef RTOS_USE_DRAM_FOR_APP_THREAD_STACKS
    /*
     * Since application thread stacks are in DRAM and we do DRAM validation
     * on first use, we need to validate the DRAM block the thread's stack here:
     */
    fdc_error = check_dram_memory_block(thread_stack_p, sizeof(*thread_stack_p));
    if (fdc_error != 0)
    {
        fatal_error = true;
        goto Exit;
    }
#endif

    rtos_k_thread_init(
        params_p,
        thread_stack_p,
        cpu_id,
        false,
        new_thread_p - g_McRTOS_p->rts_app_threads,
        new_thread_p);

    if (params_p->p_thread_pp != NULL)
    {
        FDC_ASSERT_VALID_RAM_POINTER(params_p->p_thread_pp, sizeof(void *));
        *params_p->p_thread_pp = new_thread_p;
    }

Exit:
    if (fatal_error)
    {
        fatal_error_handler(fdc_error);
    }

    return fdc_error;
}


/**
 * Initializes a McRTOS thread
 *
 * @param   params_p: Pointer to the thread's creation parameters.
 * 
 * @param   thread_stack_p: Pointer to the thread's execution stack.
 * 
 * @param   cpu_id: CPU ID of the CPU core where this thread exists.
 * 
 * @param   thread_is_privileged: true if thread to initialize is
 *          a system thread. False otherwise.
 *
 * @param context_id: Execution context ID for tracing purposes
 *
 * @param   rtos_thread_p: Pointer to the thread object
 *
 * @return  none
 */
void
rtos_k_thread_init(
    _IN_ const struct rtos_thread_creation_params *params_p,
    _IN_ struct rtos_thread_execution_stack *thread_stack_p,
    _IN_ cpu_id_t cpu_id,
    _IN_ bool thread_is_privileged,
    _IN_ uint8_t context_id,
    _INOUT_ struct rtos_thread *rtos_thread_p)
{
    cpu_status_register_t cpu_status_register = 0;
    rtos_cpu_mode_t rtos_cpu_mode = RTOS_INVALID_CPU_MODE;

    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(params_p, sizeof(uint32_t));
    FDC_ASSERT_VALID_RAM_POINTER(thread_stack_p, sizeof(uint32_t));
    FDC_ASSERT_VALID_RAM_POINTER(rtos_thread_p, sizeof(uint32_t));

    rtos_thread_prio_t thread_prio = params_p->p_priority;

    FDC_ASSERT(cpu_id < SOC_NUM_CPU_CORES,
        cpu_id, SOC_NUM_CPU_CORES);

    FDC_ASSERT(thread_prio < RTOS_NUM_THREAD_PRIORITIES,
        thread_prio, RTOS_NUM_THREAD_PRIORITIES);

    FDC_ASSERT_VALID_FUNCTION_POINTER(params_p->p_function_p);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    rtos_thread_p->thr_signature = RTOS_THREAD_SIGNATURE;
    rtos_thread_p->thr_function_p = params_p->p_function_p;
    rtos_thread_p->thr_function_arg_p = params_p->p_function_arg_p;
    rtos_thread_p->thr_console_channel = params_p->p_console_channel;
    rtos_thread_p->thr_lcd_channel = params_p->p_lcd_channel;
    rtos_thread_p->thr_abort_status = 0;
    rtos_thread_p->thr_base_priority = thread_prio;
    rtos_thread_p->thr_current_priority = thread_prio;
    rtos_thread_p->thr_state = RTOS_THREAD_RUNNABLE;
    rtos_thread_p->thr_time_slice_ticks_left = RTOS_THREAD_TIME_SLICE_IN_TICKS;
    rtos_thread_p->thr_state_history = 0x0;
    rtos_thread_p->thr_priority_history = thread_prio;
    rtos_thread_p->thr_preempted_by_time_slice_count = 0;
    rtos_thread_p->thr_preempted_by_other_thread_count = 0;
    rtos_thread_p->thr_self_preempted_count = 0;
    rtos_thread_p->thr_owned_mutexes_count = 0;
    rtos_thread_p->thr_blocked_on_p = NULL;

    GLIST_NODE_INIT(&rtos_thread_p->thr_list_node);

    fdc_context_switch_trace_entry_t prefilled_trace_entry = 0;

    SET_BIT_FIELD(
        prefilled_trace_entry, 
        FDC_CST_CONTEXT_ID_MASK,
        FDC_CST_CONTEXT_ID_SHIFT,
        context_id);

    if (thread_is_privileged)
    {
        rtos_cpu_mode = RTOS_PRIVILEGED_THREAD_MODE;

        /*
         * Initial value for CPU status register:
         * - System mode
         * - CPU interrupts fully enabled: I_BIT = 0, F_BIT = 0
         */
        cpu_status_register = ARM_MODE_SYS;

        SET_BIT_FIELD(
            prefilled_trace_entry, 
            FDC_CST_CONTEXT_TYPE_MASK,
            FDC_CST_CONTEXT_TYPE_SHIFT,
            FDC_CST_SYSTEM_THREAD);
    }
    else
    {
        rtos_cpu_mode = RTOS_UNPRIVILEGED_THREAD_MODE;

        /*
         * Initial value for CPU status register:
         * - User mode
         * - CPU interrupts fully enabled: I_BIT = 0, F_BIT = 0
         */
        cpu_status_register = ARM_MODE_USER;

        SET_BIT_FIELD(
            prefilled_trace_entry, 
            FDC_CST_CONTEXT_TYPE_MASK,
            FDC_CST_CONTEXT_TYPE_SHIFT,
            FDC_CST_APPLICATION_THREAD);
    }

    /*
     * Initialize thread's stack:
     */

    for (uint32_t i = 0;
         i < RTOS_THREAD_STACK_OVERFLOW_BUFFER_SIZE_IN_ENTRIES; ++ i)
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
     * Initialize the thread's execution context:
     */
    rtos_execution_context_init(
        &rtos_thread_p->thr_execution_context,
        params_p->p_name_p,
        cpu_id,
        RTOS_THREAD_CONTEXT,
        prefilled_trace_entry,
        rtos_cpu_mode,
        cpu_status_register,
        (cpu_register_t)params_p->p_function_p,
        (cpu_register_t)params_p->p_function_arg_p,
        &thread_stack_p->tes_stack[0],
        &thread_stack_p->tes_stack[RTOS_THREAD_STACK_NUM_ENTRIES]);

    /*
     * Initialize the thread's delay timer
     */
    rtos_k_timer_init(
        params_p->p_name_p,
        cpu_id,
        rtos_delay_timer_callback,
        &rtos_thread_p->thr_delay_timer);

    /*
     * Initialize the thread's condvar:
     */
    rtos_k_condvar_init(
        params_p->p_name_p,
        cpu_id,
        &rtos_thread_p->thr_condvar);
                    
    /*
     * Add new thread to the corresponding runnable thread queue:
     */

    FDC_ASSERT_RTOS_THREAD_INVARIANTS(rtos_thread_p);

    cpu_status_register = rtos_k_disable_cpu_interrupts();

    rtos_add_tail_runnable_thread(
        cpu_controller_p, rtos_thread_p);

    FDC_ASSERT(cpu_id == SOC_GET_CURRENT_CPU_ID(),
        cpu_id, SOC_GET_CURRENT_CPU_ID());

    /*
     * If we are being called after the thread scheduler has been activated
     * (i.e. after McRTOS startup), and if the new thread has higher
     * priority than the calling thread, perform a synchronous context switch
     * to run the thread scheduler.
     */
    if (cpu_controller_p->cpc_startup_completed)
    {
        struct rtos_execution_context *current_execution_context_p =
            cpu_controller_p->cpc_current_execution_context_p;

        DBG_ASSERT(
            current_execution_context_p != NULL, cpu_controller_p, 0);

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
     * Start delay timer for the current thread:
     */
    rtos_k_timer_start(
        &current_thread_p->thr_delay_timer,
        num_milliseconds);

    /*
     * Wait for the calling thread's delay timer to expire:
     */
    rtos_k_condvar_wait_interrupt(&current_thread_p->thr_condvar);

    FDC_ASSERT(
        current_thread_p->thr_delay_timer.tmr_time_to_expire == 0,
        current_thread_p->thr_delay_timer.tmr_time_to_expire, current_thread_p);
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

    rtos_thread_scheduler();

    /*
     * We should never come back here:
     */
    FDC_ASSERT(false, 0, 0);
}


static void
rtos_invalid_interrupt_return(void)
{
    /*
     * We should never come here:
     */
    FDC_ASSERT(false, 0, 0);
}


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


const char *
rtos_k_thread_name(
    _IN_ const struct rtos_thread *rtos_thread_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT_RTOS_THREAD_INVARIANTS(rtos_thread_p);

    return rtos_thread_p->thr_execution_context.ctx_name_p;
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

    rtos_k_condvar_wait(&current_thread_p->thr_condvar, rtos_mutex_p);
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

    rtos_k_condvar_wait_interrupt(&current_thread_p->thr_condvar);
}


void
rtos_k_thread_condvar_signal(
    _IN_ struct rtos_thread *rtos_thread_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(false);

    FDC_ASSERT_RTOS_THREAD_INVARIANTS(rtos_thread_p);

    rtos_k_condvar_signal_internal(&rtos_thread_p->thr_condvar, false);
}


/**
 * Create a McRTOS mutex
 */
fdc_error_t
rtos_k_create_mutex(
    _IN_ const struct rtos_mutex_creation_params *params_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(params_p, sizeof(void *));
    FDC_ASSERT_VALID_RAM_POINTER(params_p->p_mutex_pp, sizeof(void *));

    fdc_error_t fdc_error = 0;
    bool fatal_error = false;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    /*
     * Allocate a mutex object:
     */
    struct rtos_mutex *new_mutex_p =
        ATOMIC_POST_INCREMENT_POINTER(g_McRTOS_p->rts_next_free_app_mutex_p);

    if (new_mutex_p >= &g_McRTOS_p->rts_app_mutexes[RTOS_MAX_NUM_APP_MUTEXES])
    {
        fdc_error = CAPTURE_FDC_ERROR(
                        "No more application mutexes can be created",
                        new_mutex_p,
                        g_McRTOS_p->rts_next_free_app_mutex_p);

        /*
         * This is considered a fatal error
         */
        fatal_error = true;
        goto Exit;
    }

    rtos_k_mutex_init(
        params_p->p_name_p,
        cpu_id,
        new_mutex_p);

    *params_p->p_mutex_pp = new_mutex_p;

Exit:
    if (fatal_error)
    {
        fatal_error_handler(fdc_error);
    }

    return fdc_error;
}


void
rtos_k_mutex_init(
    _IN_  const char *mutex_name_p,
    _IN_ cpu_id_t cpu_id,
    _OUT_ struct rtos_mutex *rtos_mutex_p)
{
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(mutex_name_p, sizeof(char));
    FDC_ASSERT_VALID_RAM_POINTER(rtos_mutex_p, sizeof(uint32_t));
    FDC_ASSERT(cpu_id < SOC_NUM_CPU_CORES, cpu_id, SOC_NUM_CPU_CORES);

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
                    cpu_controller_p, mutex_owner_p);

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
 * Internal function shared between rtos_k_mutex_release() and rtos_k_condvar_wait()
 */
static void
rtos_k_mutex_release_internal(
    _INOUT_ struct rtos_mutex *rtos_mutex_p,
    _INOUT_ struct rtos_thread *current_thread_p,
    _IN_    struct rtos_cpu_controller *cpu_controller_p,
    _IN_    bool in_condvar_wait)
{
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

    FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

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

    if (in_condvar_wait)
    {
        goto Exit;
    }

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
     * Add current thread at the beginning of the corresponding runnable queue
     * and change its state from running to runnable
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

Exit:
    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
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


/**
 * Create a McRTOS condition variable
 */
fdc_error_t
rtos_k_create_condvar(
    _IN_ const struct rtos_condvar_creation_params *params_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(params_p, sizeof(void *));
    FDC_ASSERT_VALID_RAM_POINTER(params_p->p_condvar_pp, sizeof(void *));

    fdc_error_t fdc_error = 0;
    bool fatal_error = false;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    /*
     * Allocate a condvar object:
     */
    struct rtos_condvar *new_condvar_p =
        ATOMIC_POST_INCREMENT_POINTER(g_McRTOS_p->rts_next_free_app_condvar_p);

    if (new_condvar_p >= &g_McRTOS_p->rts_app_condvars[RTOS_MAX_NUM_APP_CONDVARS])
    {
        fdc_error = CAPTURE_FDC_ERROR(
                        "No more application condvars can be created",
                        new_condvar_p,
                        g_McRTOS_p->rts_next_free_app_condvar_p);

        /*
         * This is considered a fatal error
         */
        fatal_error = true;
        goto Exit;
    }

    rtos_k_condvar_init(
        params_p->p_name_p,
        cpu_id,
        new_condvar_p);

    *params_p->p_condvar_pp = new_condvar_p;

Exit:
    if (fatal_error)
    {
        fatal_error_handler(fdc_error);
    }

    return fdc_error;
}


void
rtos_k_condvar_init(
    _IN_  const char *condvar_name_p,
    _IN_  cpu_id_t cpu_id,
    _OUT_ struct rtos_condvar *rtos_condvar_p)
{
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(condvar_name_p, sizeof(char));
    FDC_ASSERT_VALID_RAM_POINTER(rtos_condvar_p, sizeof(uint32_t));
    FDC_ASSERT(cpu_id < SOC_NUM_CPU_CORES, cpu_id, SOC_NUM_CPU_CORES);

    rtos_condvar_p->cv_signature = RTOS_CONDVAR_SIGNATURE;
    rtos_condvar_p->cv_name_p = condvar_name_p;
    rtos_condvar_p->cv_cpu_id = cpu_id;
    rtos_condvar_p->cv_pending_interrupt_wakeup = false;
    rtos_condvar_p->cv_released_mutex_p = NULL;
    GLIST_NODE_INIT(&rtos_condvar_p->cv_waiting_thread_queue_anchor);
}


/**
 * Waits on a McRTOS condition variable
 */
void
rtos_k_condvar_wait(
    _INOUT_ struct rtos_condvar *rtos_condvar_p,
    _IN_    struct rtos_mutex *rtos_mutex_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
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
        rtos_condvar_p->cv_signature == RTOS_CONDVAR_SIGNATURE,
        rtos_condvar_p->cv_signature, rtos_condvar_p);

    FDC_ASSERT(
        rtos_condvar_p->cv_cpu_id == cpu_id,
        rtos_condvar_p->cv_cpu_id, cpu_id);

    FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

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

    /*
     * Add current thread at the end of the condvar's waiting queue and
     * change its state from running to blocked:
     */
    
    glist_add_tail_elem(
        &rtos_condvar_p->cv_waiting_thread_queue_anchor,
        &current_thread_p->thr_list_node);

    RTOS_THREAD_CHANGE_STATE(current_thread_p, RTOS_THREAD_BLOCKED_ON_CONDVAR);

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

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Waits on a McRTOS condition variable to be signaled by an interrupt handler
 */
void
rtos_k_condvar_wait_interrupt(
    _INOUT_ struct rtos_condvar *rtos_condvar_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
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
        rtos_condvar_p->cv_signature == RTOS_CONDVAR_SIGNATURE,
        rtos_condvar_p->cv_signature, rtos_condvar_p);

    FDC_ASSERT(
        rtos_condvar_p->cv_cpu_id == cpu_id,
        rtos_condvar_p->cv_cpu_id, cpu_id);

    cpu_status_register_t cpu_status_register;
    bool restore_interrupts = false;
    
    CAPTURE_ARM_CPSR_REGISTER(cpu_status_register);

    /*
     * Disable interrupts in the ARM core:
     *
     * NOTE: Here we don't call rtos_k_disable_cpu_interrupts() if interrupts are
     * already disabled, as this code path calls rtos_k_restore_execution_context()
     */
    if (CPU_INTERRUPTS_ARE_ENABLED(cpu_status_register))
    {
        cpu_status_register = rtos_k_disable_cpu_interrupts();
        restore_interrupts = true;
    }

    /*
     * If there is a pending wakeup from an interrupt for this condvar, we don't
     * need to wait:
     */
    if (rtos_condvar_p->cv_pending_interrupt_wakeup)
    {
        goto Exit;
    }

    /*
     * Add current thread at the end of the condvar's waiting queue and
     * change its state from running to blocked:
     */
    
    glist_add_tail_elem(
        &rtos_condvar_p->cv_waiting_thread_queue_anchor,
        &current_thread_p->thr_list_node);

    RTOS_THREAD_CHANGE_STATE(current_thread_p, RTOS_THREAD_BLOCKED_ON_CONDVAR);

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

Exit:
    rtos_condvar_p->cv_pending_interrupt_wakeup = false;

    /*
     * Restore previous interrupt masking in the ARM core
     */
    if (restore_interrupts)
    {
        rtos_k_restore_cpu_interrupts(cpu_status_register);
    }
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

    if (current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
        FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();
    }

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    while (GLIST_IS_NOT_EMPTY(&rtos_condvar_p->cv_waiting_thread_queue_anchor))
    {
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

        if (!broadcast)
        {
            break;
        }
    }

    if (current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
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
    else
    {
        /*
         * The caller is an interrupt context:
         */
        rtos_condvar_p->cv_pending_interrupt_wakeup = true;
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


/**
 * Creates a McRTOS timer
 */
fdc_error_t
rtos_k_create_timer(
    _IN_ const struct rtos_timer_creation_params *params_p)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(params_p, sizeof(void *));
    FDC_ASSERT_VALID_RAM_POINTER(params_p->p_timer_pp, sizeof(void *));

    fdc_error_t fdc_error = 0;
    bool fatal_error = false;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    /*
     * Allocate a timer object:
     */
    struct rtos_timer *new_timer_p =
        ATOMIC_POST_INCREMENT_POINTER(g_McRTOS_p->rts_next_free_app_timer_p);

    if (new_timer_p >= &g_McRTOS_p->rts_app_timers[RTOS_MAX_NUM_APP_TIMERS])
    {
        fdc_error = CAPTURE_FDC_ERROR(
                        "No more application timers can be created",
                        new_timer_p,
                        g_McRTOS_p->rts_next_free_app_timer_p);

        /*
         * This is considered a fatal error
         */
        fatal_error = true;
        goto Exit;
    }

    rtos_k_timer_init(
        params_p->p_name_p,
        cpu_id,
        params_p->p_function_p,
        new_timer_p);

    *params_p->p_timer_pp = new_timer_p;

Exit:
    if (fatal_error)
    {
        fatal_error_handler(fdc_error);
    }

    return fdc_error;
}


void
rtos_k_timer_init(
    _IN_  const char *timer_name_p,
    _IN_  cpu_id_t cpu_id,
    _IN_  rtos_timer_function_t *timer_function_p,
    _OUT_ struct rtos_timer *rtos_timer_p)
{
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(timer_name_p, sizeof(char));
    FDC_ASSERT_VALID_FUNCTION_POINTER(timer_function_p);
    FDC_ASSERT_VALID_RAM_POINTER(rtos_timer_p, sizeof(uint32_t));
    FDC_ASSERT(cpu_id < SOC_NUM_CPU_CORES, cpu_id, SOC_NUM_CPU_CORES);

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

    FDC_ASSERT(
        rtos_timer_p->tmr_time_to_expire == 0,
        rtos_timer_p->tmr_time_to_expire, rtos_timer_p);

    FDC_ASSERT(expiration_time_in_ms != 0, expiration_time_in_ms, rtos_timer_p);

    rtos_ticks_t num_ticks =
        HOW_MANY(expiration_time_in_ms, RTOS_MILLISECONDS_PER_TICK);

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

    struct rtos_interrupt *interrupt_p =
        &cpu_controller_p->cpc_interrupts[channel];

    interrupt_p->int_signature = RTOS_INTERRUPT_SIGNATURE;
    interrupt_p->int_cpu_controller_p = cpu_controller_p;
    interrupt_p->int_isr_function_p = isr_function_p;
    interrupt_p->int_arg_p = params_p->irp_arg_p;
    interrupt_p->int_channel = channel;
    interrupt_p->int_priority = interrupt_prio;

    /*
     * Initialize the interrupt's stack:
     */

    for (uint32_t i = 0;
         i < RTOS_INTERRUPT_STACK_OVERFLOW_BUFFER_SIZE_IN_ENTRIES; ++ i)
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

    /*
     * Initialize the interrupt's execution context:
     */

    fdc_context_switch_trace_entry_t prefilled_trace_entry = 0;

    SET_BIT_FIELD(
        prefilled_trace_entry, 
        FDC_CST_CONTEXT_ID_MASK,
        FDC_CST_CONTEXT_ID_SHIFT,
        channel);

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

    rtos_execution_context_init(
        &interrupt_p->int_execution_context,
        params_p->irp_name_p,
        cpu_id,
        RTOS_INTERRUPT_CONTEXT,
        prefilled_trace_entry,
        RTOS_INTERRUPT_MODE,
        ARM_MODE_SYS,
        (cpu_register_t)isr_function_p,
        0,
        &interrupt_p->int_stack[0],
        &interrupt_p->int_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES]);

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
}


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

    RTOS_START_INTERRUPTS_DISABLED_TIME_MEASURE();

    FDC_ASSERT(
        rtos_interrupt_p->int_signature == RTOS_INTERRUPT_SIGNATURE,
        rtos_interrupt_p->int_signature, rtos_interrupt_p);

    struct rtos_execution_context *interrupt_context_p =
        &rtos_interrupt_p->int_execution_context;

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_context_p->ctx_signature == RTOS_EXECUTION_CONTEXT_SIGNATURE,
        current_context_p->ctx_signature, current_context_p);

    /*
     * Check saved CPU registers for the interrupted context:
     */
    FDC_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(current_context_p);

    /*
     * The current context must have been running in either user mode or system
     * mode, otherwise it could not have been interrupted, as we only allow
     * interrupts to be enabled in user mode or system mode.
     */

    cpu_status_register_t current_context_cpsr =
        current_context_p->ctx_cpu_registers[CPU_REG_CPSR];

    FDC_ASSERT(
        CPU_MODE_IS_UNPRIVILEGED(current_context_cpsr) ||
        CPU_MODE_IS_PRIVILEGED(current_context_cpsr),
        current_context_cpsr, current_context_p);

    /*
     * Current execution context must be different from the calling
     * interrupt context:
     */
    DBG_ASSERT(
        interrupt_context_p != current_context_p,
        interrupt_context_p, current_context_p);

    DBG_ASSERT(
        (cpu_controller_p->cpc_active_interrupts & 
            BIT(rtos_interrupt_p->int_channel)) == 0,
        cpu_controller_p->cpc_active_interrupts, cpu_controller_p);

    /*
     * Track CPU usage for current execution context:
     */
    cpu_clock_cycles_t used_cpu_cycles =
        CPU_CLOCK_CYCLES_DELTA(
            current_context_p->ctx_last_switched_in_time_stamp,
            get_cpu_clock_cycles()) -
        g_McRTOS_p->rts_cpu_cycles_measure_overhead;

    current_context_p->ctx_accumulated_cpu_usage += used_cpu_cycles;

    /*
     * Neither the calling interrupt context nor the current execution context are
     * in the preemption chain:
     */

    DBG_ASSERT(
        GLIST_NODE_IS_UNLINKED(&interrupt_context_p->ctx_preemption_chain_node),
        &interrupt_context_p->ctx_preemption_chain_node,
        interrupt_context_p);

    DBG_ASSERT(
        GLIST_NODE_IS_UNLINKED(&current_context_p->ctx_preemption_chain_node),
        &current_context_p->ctx_preemption_chain_node,
        current_context_p);

    rtos_nested_interrupts_count_t nested_interrupts_count =
        cpu_controller_p->cpc_nested_interrupts_count;

    FDC_ASSERT(
        nested_interrupts_count < SOC_NUM_INTERRUPT_PRIORITIES,
        nested_interrupts_count, cpu_controller_p);

    if (nested_interrupts_count == 0)
    {
        /*
         * The current context is a thread:
         */
        FDC_ASSERT(
            current_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
            current_context_p->ctx_context_type, current_context_p);

        /*
         * Current thread is in running state and it is not in any thread queue
         */

        struct rtos_thread *current_thread_p =
            RTOS_EXECUTION_CONTEXT_GET_THREAD(current_context_p);

        DBG_ASSERT(
            current_thread_p == cpu_controller_p->cpc_current_thread_p,
            current_thread_p, cpu_controller_p->cpc_current_thread_p);

        DBG_ASSERT_RUNNING_THREAD_INVARIANTS(current_thread_p);

        /*
         * Add current thread at the beginning of the corresponding runnable
         * queue and change its state from running to runnable:
         */
         rtos_add_head_runnable_thread(
            cpu_controller_p, current_thread_p);
    }
    else
    {
        /*
         * The current context is another interrupt:
         */
        DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(current_context_p);

        FDC_ASSERT(
            current_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            current_context_p->ctx_context_type, current_context_p);
    }

    /*
     * Increment interrupt nesting level:
     */
    cpu_controller_p->cpc_nested_interrupts_count ++;

    RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
        current_context_p,
        CTX_SWITCHED_OUT_PREEMPTED_BY_INTERRUPT,
        cpu_controller_p);

    current_context_p->ctx_last_preempted_by_p = interrupt_context_p;
    current_context_p->ctx_preempted_counter ++;

    /*
     * Add current execution context at the top of the preemption chain:
     */
    rtos_preemption_chain_push_context(cpu_controller_p, current_context_p);

    /*
     * Set current execution context to the calling interrupt context:
     */
    cpu_controller_p->cpc_current_execution_context_p = interrupt_context_p;
    interrupt_context_p->ctx_last_switched_in_time_stamp = get_cpu_clock_cycles();
    FDC_TRACE_RTOS_CONTEXT_SWITCH(interrupt_context_p);

    cpu_controller_p->cpc_active_interrupts |= BIT(rtos_interrupt_p->int_channel);

    RTOS_STOP_INTERRUPTS_DISABLED_TIME_MEASURE();

    /*
     * The stack pointer for an interrupt context always starts at the bottom
     * of the interrupt context's stack:
     */
    return (cpu_register_t)interrupt_context_p->ctx_execution_stack_bottom_end_p;
}


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

    RTOS_START_INTERRUPTS_DISABLED_TIME_MEASURE();

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
        cpu_controller_p->cpc_active_interrupts & BIT(current_interrupt_p->int_channel),
        cpu_controller_p->cpc_active_interrupts, cpu_controller_p);

    /* 
     * Notify the interrupt controller that processing for the last interrupt
     * received by the calling CPU core has been completed, so that another
     * interrupt of the same priority or lower can be received by this CPU core:
     */
    notify_interrupt_controller_isr_done(current_interrupt_p->int_channel);

    /*
     * Track CPU usage for current execution context:
     */
    cpu_clock_cycles_t used_cpu_cycles =
        CPU_CLOCK_CYCLES_DELTA(
            current_context_p->ctx_last_switched_in_time_stamp,
            get_cpu_clock_cycles()) -
        g_McRTOS_p->rts_cpu_cycles_measure_overhead;

    current_context_p->ctx_accumulated_cpu_usage += used_cpu_cycles;

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
     * Set current execution context to the context removed from the
     * preemption chain:
     */
    cpu_controller_p->cpc_current_execution_context_p = preempted_context_p;
    current_context_p = preempted_context_p;

    /*
     * Decrement interrupt nesting level:
     */

    rtos_nested_interrupts_count_t nested_interrupts_count =
        cpu_controller_p->cpc_nested_interrupts_count;

    FDC_ASSERT(
        nested_interrupts_count > 0 &&
        nested_interrupts_count <= SOC_NUM_INTERRUPT_PRIORITIES,
        nested_interrupts_count, cpu_controller_p);

    nested_interrupts_count --;
    cpu_controller_p->cpc_nested_interrupts_count = nested_interrupts_count;
    cpu_controller_p->cpc_active_interrupts &= ~BIT(current_interrupt_p->int_channel);

    if (nested_interrupts_count == 0)
    {
        /*
         * The new current context is a thread:
         */
        struct rtos_thread *current_thread_p =
            RTOS_EXECUTION_CONTEXT_GET_THREAD(current_context_p);

        DBG_ASSERT(
            current_thread_p == cpu_controller_p->cpc_current_thread_p,
            current_thread_p, cpu_controller_p->cpc_current_thread_p);

        DBG_ASSERT_RTOS_THREAD_INVARIANTS(current_thread_p);

        /*
         * Call thread scheduler to ensure that the highest-priority runnable
         * thread gets to run:
         */
        rtos_thread_scheduler(); 
    }
    else
    {
        /*
         * The new current context is another interrupt that was preempted
         * by this interrupt:
         */
        DBG_ASSERT(
            current_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            current_context_p->ctx_context_type, current_context_p);

        /*
         * Check saved CPU registers for the interrupt context to be restored:
         */
        FDC_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(current_context_p);

        /*
         * Restore execution context of the previous interrupt
         */ 
        rtos_k_restore_execution_context(current_context_p);
    }

    /*
     * We should never come back here:
     */
    FDC_ASSERT(false, 0, 0);
}


fdc_error_t
rtos_k_enter_system_call(
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

    //XXX DBG_ASSERT_PRIVILEGED_THREAD_CPU_MODE_AND_INTERRUPTS_ENABLED();

    if (system_call_number >= RTOS_NUM_SYSTEM_CALLS)
    {
        fdc_error = CAPTURE_FDC_ERROR("Invalid system call number",
                        system_call_number, current_context_p);
        goto Exit;                        
    }

    DBG_ASSERT_VALID_FUNCTION_POINTER(
        g_rtos_system_call_dispatch_table[system_call_number]);

    FDC_ASSERT(
        current_context_p->ctx_cpu_mode == RTOS_UNPRIVILEGED_THREAD_MODE,
        current_context_p->ctx_cpu_mode, current_context_p);

    current_context_p->ctx_cpu_mode = RTOS_PRIVILEGED_THREAD_MODE;

Exit:
    return fdc_error;
}


void
rtos_k_exit_system_call(void)
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

    fdc_error_t fdc_error = 0;

    /*
     * TODO: implement this
     */ 
    TODO_IMPLEMENT_THIS();

    return fdc_error;
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
 * @param cpu_status_register   Initial value for the CPU status register. Only
 *                              meaningful for thread contexts.
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
static void
rtos_execution_context_init(
    _OUT_ struct rtos_execution_context *execution_context_p,
    _IN_  const char *context_name_p,
    _IN_  cpu_id_t cpu_id,
    _IN_  rtos_execution_context_type_t context_type,
    _IN_  fdc_context_switch_trace_entry_t prefilled_trace_entry,
    _IN_  rtos_cpu_mode_t rtos_cpu_mode,
    _IN_  cpu_status_register_t cpu_status_register,
    _IN_  cpu_register_t cpu_pc_register,
    _IN_  cpu_register_t cpu_r0_register,
    _IN_  rtos_execution_stack_entry_t *stack_top_end_p,
    _IN_  rtos_execution_stack_entry_t *stack_bottom_end_p)
{
    DBG_ASSERT(execution_context_p != NULL, 0, 0);
    DBG_ASSERT(cpu_id < SOC_NUM_CPU_CORES, cpu_id, 0);

    if (context_type == RTOS_THREAD_CONTEXT)
    {
        uint32_t arm_cpu_mode = (cpu_status_register & ARM_MODE_MASK);

        FDC_ASSERT(
            rtos_cpu_mode == RTOS_UNPRIVILEGED_THREAD_MODE ||
            rtos_cpu_mode == RTOS_PRIVILEGED_THREAD_MODE,
            rtos_cpu_mode, execution_context_p);

#if DEFINED_ARM_CLASSIC_ARCH()
        FDC_ASSERT(
           arm_cpu_mode == ARM_MODE_SYS ||
           arm_cpu_mode == ARM_MODE_USER,
           cpu_status_register, execution_context_p);

        FDC_ASSERT(
            (cpu_status_register & ARM_INTERRUPTS_DISABLED_MASK) == 0,
            cpu_status_register, execution_context_p);
#else
#error "TODO: Add code here for ARMv7-M"
#endif
    }
    else
    {
        FDC_ASSERT(
            context_type == RTOS_INTERRUPT_CONTEXT,
            context_type, execution_context_p);

        FDC_ASSERT(
            rtos_cpu_mode == RTOS_INTERRUPT_MODE,
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
    execution_context_p->ctx_last_switched_in_time_stamp = 0;
    execution_context_p->ctx_last_switched_out_time_stamp_in_ticks = 0;
    execution_context_p->ctx_accumulated_cpu_usage = 0;
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

        if (rtos_cpu_mode == RTOS_UNPRIVILEGED_THREAD_MODE)
        {
            execution_context_p->ctx_cpu_registers[CPU_REG_LR] =
                (cpu_register_t)rtos_thread_abort;
        }
        else
        {
            execution_context_p->ctx_cpu_registers[CPU_REG_LR] =
                (cpu_register_t)rtos_k_thread_abort;
        }
    }
    else
    {
        execution_context_p->ctx_cpu_registers[CPU_REG_LR] =
                (cpu_register_t)rtos_invalid_interrupt_return;
    }

    execution_context_p->ctx_cpu_registers[CPU_REG_CPSR] = cpu_status_register;
    execution_context_p->ctx_cpu_registers[CPU_REG_PC] = cpu_pc_register;
    execution_context_p->ctx_cpu_registers[CPU_REG_SP] =
        (cpu_register_t)execution_context_p->ctx_execution_stack_bottom_end_p;
#else
#error "TODO: Add code here for ARMv7-M"
#endif
}


void
rtos_k_console_putchar(
    _UNUSED_ void *unused_arg_p,
    _IN_ uint8_t c)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);
    FDC_ASSERT(unused_arg_p == NULL, unused_arg_p, 0);
    bool send_physical_output = true;
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

    if (current_thread_p->thr_console_channel != RTOS_CONSOLE_CHANNEL_NONE &&
        current_thread_p->thr_console_channel != g_McRTOS_p->rts_current_console_channel)
    {
        send_physical_output = false;
    }

    if (send_physical_output)
    {
        if (RTOS_CURRENT_THREAD_IS_ROOT_SYSTEM_THREAD(cpu_controller_p))
        {
            uart_putchar_with_polling(g_console_serial_port_p, c);
        }
        else
        {
            uart_putchar(g_console_serial_port_p, c);
        }
    }
}


uint8_t
rtos_k_console_getchar(_IN_ bool wait)
{
    FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(true);

    return uart_getchar(g_console_serial_port_p, wait);
}


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
