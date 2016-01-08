/**
 * @file McRTOS_startup.c
 *
 * McRTOS startup module.
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */

#include <McRTOS/McRTOS.h>
#include <McRTOS/McRTOS_internals.h>
#include <McRTOS/McRTOS_command_processor.h>
#include <McRTOS/failure_data_capture.h>
#include <BoardSupport/hardware_abstractions.h>
#include <McRTOS/utils.h>

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-function"

#define RTOS_CPU_CONTROLLER_INITIALIZER(_cpu_id) \
        .rts_cpu_controllers[_cpu_id] =                                 \
        {                                                               \
            .cpc_signature = RTOS_CPU_CONTROLLER_SIGNATURE,             \
            .cpc_cpu_id = (_cpu_id),                                    \
            .cpc_current_execution_context_p = NULL,                    \
            .cpc_current_thread_p = NULL,                               \
            .cpc_last_fpu_thread_p = NULL,                              \
            .cpc_runnable_thread_priorities = 0,                        \
            .cpc_active_internal_interrupts =  { 0 },                   \
            .cpc_active_external_interrupts = { 0 },                    \
            .cpc_nested_interrupts_count = 0,                           \
            .cpc_thread_scheduler_calls = 0,                            \
            .cpc_fpu_context_switch_count = 0,                          \
            .cpc_accumulated_thread_scheduler_overhead = 0,             \
            .cpc_interrupts_disabled_being_measured_count = 0,          \
            .cpc_measure_interrupts_disabled_time = false,		\
            .cpc_pending_thread_time_slice_decrement = false,           \
            .cpc_current_timer_wheel_spoke_index = 0,                   \
            .cpc_ticks_since_boot_count = 0,                            \
            .cpc_interrupts_disabled_start_time_stamp = 0,              \
            .cpc_longest_time_interrupts_disabled = 0,                  \
            FDC_INFO_INITIALIZER(.cpc_failures_info),                   \
            .cpc_execution_contexts_list_anchor =                       \
                GLIST_NODE_INITIALIZER(                                 \
                    g_McRTOS.rts_cpu_controllers[_cpu_id].              \
                        cpc_execution_contexts_list_anchor),            \
            .cpc_preemption_chain_anchor =                              \
                GLIST_NODE_INITIALIZER(                                 \
                    g_McRTOS.rts_cpu_controllers[_cpu_id].              \
                        cpc_preemption_chain_anchor),                   \
	    .cpc_system_threads_execution_stacks_p =			\
		g_rtos_system_threads_execution_stacks[_cpu_id],	\
        }


static void rtos_init_reset_execution_context(
                struct rtos_cpu_controller *cpu_controller_p);

static fdc_error_t rtos_root_thread_f(void *arg);
static fdc_error_t rtos_idle_thread_f(void *arg);
static fdc_error_t rtos_touch_screen_reader_thread_f(void *arg);

const char g_McRTOS_version[] = "McRTOS v2.6 - " GIT_COMMIT " (" RTOS_BUILD_FLAVOR ")";

const char g_McRTOS_build_timestamp[] = "built "__DATE__ " " __TIME__;

/**
 * Per-cpu system threads:
 */
static const struct rtos_thread_creation_params g_rtos_system_threads[] =
{
    [RTOS_ROOT_SYSTEM_THREAD] =
    {
        .p_name_p = "McRTOS root thread",
        .p_function_p = rtos_root_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY,
    },

    [RTOS_IDLE_SYSTEM_THREAD] =
    {
        .p_name_p = "McRTOS idle thread",
        .p_function_p = rtos_idle_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = RTOS_LOWEST_THREAD_PRIORITY,
    },
};

C_ASSERT(ARRAY_SIZE(g_rtos_system_threads) <= RTOS_NUM_SYSTEM_THREADS_PER_CPU);

/**
 * Array of execution stacks for system threads for each CPU core
 */
static struct rtos_thread_execution_stack
    g_rtos_system_threads_execution_stacks[SOC_NUM_CPU_CORES][RTOS_NUM_SYSTEM_THREADS_PER_CPU];

/**
 * McRTOS global state variables
 */
static struct McRTOS g_McRTOS =
{{
    .rts_signature = MCRTOS_SIGNATURE,

    .rts_release_secondary_cores = false,

    .rts_console_input_cpu_id = 0,

    .rts_next_free_interrupt_p = &g_McRTOS.rts_interrupts[0],

#ifdef MCRTOS_PRIVATE_OBJECTS
    .rts_next_free_app_thread_p = &g_McRTOS.rts_app_threads[0],

    .rts_next_free_app_timer_p = &g_McRTOS.rts_app_timers[0],

    .rts_next_free_app_mutex_p = &g_McRTOS.rts_app_mutexes[0],

    .rts_next_free_app_condvar_p = &g_McRTOS.rts_app_condvars[0],

#if RTOS_MAX_NUM_APP_MSG_CHANNELS > 0
    .rts_next_free_app_32bit_msg_channel_p = &g_McRTOS.rts_app_32bit_msg_channels[0],
#else
    .rts_next_free_app_32bit_msg_channel_p = NULL,
#endif

#if RTOS_MAX_NUM_APP_OBJECT_POOLS > 0
    .rts_next_free_app_object_pool_p = &g_McRTOS.rts_app_object_pools[0],
#else
    .rts_next_free_app_object_pool_p = NULL,
#endif
#endif /* MCRTOS_PRIVATE_OBJECTS */

#ifdef LCD_SUPPORTED
    .rts_current_lcd_channel = RTOS_COMMAND_LINE_LCD_CHANNEL,
#endif

#ifdef LPC2478_SOC
    .rts_stop_idle_cpu = false,
#else
    .rts_stop_idle_cpu = true,
#endif

    .rts_app_hardware_init_called = false,

    RTOS_CPU_CONTROLLER_INITIALIZER(0),
}};

/**
 * Access to the McRTOS global structure should be done through this pointer,
 * in case we want to move the global structure to SDRAM
 */
struct McRTOS *const g_McRTOS_p = &g_McRTOS;


/*
 * Check some of the compile-time initializations common to all CPU cores
 */
static void
check_mcrtos_common_compile_time_initializations(void)
{
    /*
     * Check that initialized global data was copied correctly from
     * flash to RAM (to catch problems in the linker script)
     */
    FDC_ASSERT(
        g_McRTOS_p->rts_signature == MCRTOS_SIGNATURE,
        g_McRTOS_p->rts_signature, g_McRTOS_p);
}


#ifdef LCD_SUPPORTED
static void
lcd_display_greetings(void)
{
    static const struct lcd_char_attributes lcd_char_attributes1 =
    {
        .lcd_dot_size = 1,
        .lcd_foreground_color = LCD_COLOR_YELLOW,
        .lcd_background_color = LCD_COLOR_BLUE,
    };

    static const struct lcd_char_attributes lcd_char_attributes2 =
    {
        .lcd_dot_size = 2,
        .lcd_foreground_color = LCD_COLOR_YELLOW,
        .lcd_background_color = LCD_COLOR_BLUE,
    };

    /*
     * Display McRTOS LCD greeting:
     */
    lcd_clear(LCD_COLOR_BLUE);
    lcd_printf(
        64, 64, &lcd_char_attributes2, "%s", g_McRTOS_version);
    lcd_printf(
        0, 96, &lcd_char_attributes1,
        "             German Rivera\n"
        "       %s\n",
        g_McRTOS_build_timestamp);
}
#endif

/**
 * Main entry point of McRTOS. This function is to be invoked from the
 * reset exception handler or from the application's main() function, which
 * is assumed to be invoked from the reset exception handler, and therefore it
 * runs in privileged mode and with interrupts disabled.
 *
 * When this function is invoked, McRTOS takes control of the machine, and
 * this function only returns if there is an initialization error in McRTOS.
 *
 * @param   rtos_app_config_p Pointer to startup application configuration.
 *
 * @return  none
 */
void
rtos_startup(
    _IN_ const struct rtos_startup_app_configuration *rtos_app_config_p)
{
    FDC_ASSERT_COMING_FROM_RESET();
    FDC_ASSERT_CPU_IS_LITTLE_ENDIAN();
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(rtos_app_config_p, sizeof(uint32_t));
    FDC_ASSERT_VALID_FUNCTION_POINTER(rtos_app_config_p->stc_app_hardware_init_p);
    FDC_ASSERT_VALID_FUNCTION_POINTER(rtos_app_config_p->stc_app_hardware_stop_p);
    FDC_ASSERT_VALID_FUNCTION_POINTER(rtos_app_config_p->stc_app_software_init_p);
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(
        rtos_app_config_p->stc_app_console_commands_p, sizeof(uint32_t));

    for (int i = 0; i < rtos_app_config_p->stc_num_app_console_commands; i ++)
    {
        FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(
            rtos_app_config_p->stc_app_console_commands_p[i].cmd_name_p, sizeof(char));

        FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(
            rtos_app_config_p->stc_app_console_commands_p[i].cmd_description_p, sizeof(char));

        FDC_ASSERT_VALID_FUNCTION_POINTER(
            rtos_app_config_p->stc_app_console_commands_p[i].cmd_function_p);
    }

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
                                    &g_McRTOS_p->rts_cpu_controllers[cpu_id];

#   if DEFINED_ARM_CORTEX_M_ARCH()
    /*
     * The Cortex-M processor enters reset with interrupts enabled,
     * so we have to disable them here:
     */
    __disable_irq();
#   endif

    g_McRTOS_p->rts_app_hardware_init_p = rtos_app_config_p->stc_app_hardware_init_p;
    g_McRTOS_p->rts_app_hardware_stop_p = rtos_app_config_p->stc_app_hardware_stop_p;
    g_McRTOS_p->rts_app_software_init_p = rtos_app_config_p->stc_app_software_init_p;

    init_command_processor(
        rtos_app_config_p->stc_num_app_console_commands,
        rtos_app_config_p->stc_app_console_commands_p);

    rtos_init_reset_execution_context(cpu_controller_p);

    /*
     * Check the compile-time initializations common to all CPU cores:
     */
    check_mcrtos_common_compile_time_initializations();

    /*
     * Initialize board hardware on CPU 0:
     */
    if (cpu_id == 0)
    {
        g_McRTOS_p->rts_soc_reset_cause = soc_hardware_init();

#       ifdef _BRANCH_MICRO_TRACING_
        micro_trace_init();
#       endif

        console_printf_init();

#       ifdef LCD_SUPPORTED
        lcd_printf_init();
#       endif

#       ifdef _CPU_CYCLES_MEASURE_
        /*
         * Calculate approximate overhead for taking a measurement of time in
         * CPU clock cycles:
         */
        cpu_clock_cycles_t begin_cycles = get_cpu_clock_cycles();
        cpu_clock_cycles_t end_cycles = get_cpu_clock_cycles();
        g_McRTOS_p->rts_cpu_cycles_measure_overhead =
            CPU_CLOCK_CYCLES_DELTA(begin_cycles, end_cycles);

        DBG_ASSERT(
            g_McRTOS_p->rts_cpu_cycles_measure_overhead <
                SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ /* 1 microsecond */,
            g_McRTOS_p->rts_cpu_cycles_measure_overhead,
            SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ);
#       endif

	g_McRTOS_p->rts_release_secondary_cores = true;
	__DSB();
	__SEV();
    } else {
	while (!g_McRTOS_p->rts_release_secondary_cores) {
		__WFE();
	}
    }

    /*
     * Initialize runnable thread queues for this CPU controller:
     */
    for (rtos_thread_prio_t i = 0; i < RTOS_NUM_THREAD_PRIORITIES; ++ i)
    {
        GLIST_NODE_INIT(
            &cpu_controller_p->cpc_runnable_thread_queues_anchors[i]);
    }

    /*
     * Initialize hash chains of the timer wheel for this CPU controller:
     */
    for (rtos_timer_wheel_spoke_index_t i = 0; i < RTOS_TIMER_WHEEL_NUM_SPOKES; ++ i)
    {
        GLIST_NODE_INIT(
            &cpu_controller_p->cpc_timer_wheel_hash_chains_anchors[i]);
    }

    rtos_k_condvar_init("inter-processor-interrupt condvar",
		        &cpu_controller_p->cpc_inter_processor_interrupt_condvar);

    /*
     * Create root system thread to continue system initialization in that thread:
     */
    rtos_k_thread_init(
        &g_rtos_system_threads[RTOS_ROOT_SYSTEM_THREAD],
        &cpu_controller_p->cpc_system_threads_execution_stacks_p[RTOS_ROOT_SYSTEM_THREAD],
        &cpu_controller_p->cpc_system_threads[RTOS_ROOT_SYSTEM_THREAD]);

    struct rtos_execution_context *current_context_p =
	cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_context_p == &cpu_controller_p->cpc_reset_execution_context,
        current_context_p, cpu_controller_p);

    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();

#   ifdef _MEASURE_INTERRUPTS_DISABLED_TIME_
    /*
     * Start measuring interrupts disabled time from here:
     */

    FDC_ASSERT(
	cpu_controller_p->cpc_longest_time_interrupts_disabled == 0,
	cpu_controller_p->cpc_longest_time_interrupts_disabled, cpu_controller_p);

    cpu_controller_p->cpc_measure_interrupts_disabled_time = true;
    RTOS_START_INTERRUPTS_DISABLED_TIME_MEASURE();
#   endif

#   ifdef _CPU_CYCLES_MEASURE_
    cpu_clock_cycles_t used_cpu_cycles = get_cpu_clock_cycles();

    RTOS_EXECUTION_CONTEXT_UPDATE_CPU_USAGE(
        current_context_p, used_cpu_cycles);
#   endif

    RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
        current_context_p, CTX_SWITCHED_OUT_RESET_TO_ROOT_THREAD,
        cpu_controller_p);

    /*
     * Do a synchronous context switch, to switch from running the reset
     * handler to running the root thread:
     */

#   if DEFINED_ARM_CORTEX_M_ARCH()
    /*
     * Set psp to 0x0, to indicate that this is the first context switch
     */
    __set_PSP(0x0);

    /*
     * NOTE: rtos_k_synchronous_context_switch() calls rtos_thread_scheduler(),
     * which will select the root thread to run, as it is the only existing
     * thread at this point
     */
    rtos_k_synchronous_context_switch(
        cpu_controller_p->cpc_current_execution_context_p);

#   else
    rtos_thread_scheduler(RTOS_CSW_RESET_TO_THREAD); // TODO: change this to synch ctxt switch too
#   endif

    /*
     * We should never come here
     */
    FDC_ASSERT(false, 0, 0);
}


/**
 * Reboot McRTOS by causing a software-induced SoC reset.
 */
void
rtos_reboot(void)
{
    DBG_ASSERT_VALID_FUNCTION_POINTER(g_McRTOS_p->rts_app_hardware_stop_p);

    if (g_McRTOS_p->rts_app_hardware_init_called) {
        g_McRTOS_p->rts_app_hardware_stop_p();
    }

    soc_reset();
}


/**
 * Initialize reset handler's execution context for a given CPU
 */
static void
rtos_init_reset_execution_context(
    struct rtos_cpu_controller *cpu_controller_p)
{
    fdc_context_switch_trace_entry_t prefilled_trace_entry = 0;

    SET_BIT_FIELD(
        prefilled_trace_entry,
        FDC_CST_CONTEXT_ID_MASK,
        FDC_CST_CONTEXT_ID_SHIFT,
        0);

    SET_BIT_FIELD(
        prefilled_trace_entry,
        FDC_CST_CONTEXT_TYPE_MASK,
        FDC_CST_CONTEXT_TYPE_SHIFT,
        FDC_CST_RESET);

    SET_BIT_FIELD(
        prefilled_trace_entry,
        FDC_CST_CONTEXT_PRIORITY_MASK,
        FDC_CST_CONTEXT_PRIORITY_SHIFT,
        0);

    rtos_execution_stack_entry_t *stack_top_end_p;
    rtos_execution_stack_entry_t *stack_bottom_end_p;
    cpu_register_t entry_point_pc;

#if DEFINED_ARM_CLASSIC_ARCH()
    extern void *__stack_svc_start;
    extern void *__stack_svc_end;
    extern void *ResetHandler;

    stack_top_end_p = __stack_svc_start;
    stack_bottom_end_p = __stack_svc_end;
    entry_point_pc = (uintptr_t)ResetHandler;
#elif DEFINED_ARM_CORTEX_M_ARCH()
    entry_point_pc = (uintptr_t)cortex_m_reset_handler;
    stack_top_end_p = &g_cortex_m_exception_stack.es_stack[0];
    stack_bottom_end_p =
        &g_cortex_m_exception_stack.es_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES];
#else
    #error "unsupported CPU architecture"
#endif

    rtos_execution_context_init(
        &cpu_controller_p->cpc_reset_execution_context,
        "reset handler",
        cpu_controller_p->cpc_cpu_id,
        RTOS_RESET_CONTEXT,
        prefilled_trace_entry,
        RTOS_RESET_MODE,
        entry_point_pc,
        0,
        stack_top_end_p,
        stack_bottom_end_p);

    cpu_controller_p->cpc_current_execution_context_p =
        &cpu_controller_p->cpc_reset_execution_context;
}

#pragma GCC diagnostic push

#ifndef _RELIABILITY_CHECKS_
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

/**
 * McRTOS root thread function
 */
static fdc_error_t
rtos_root_thread_f(void *arg)
{
    FDC_ASSERT_UNPRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    rtos_thread_set_comp_region(&g_McRTOS,
                                sizeof g_McRTOS,
                                0,
                                NULL);

    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
                                    &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    FDC_ASSERT(arg == NULL, arg, cpu_id);
    struct rtos_thread *root_thread_p = (struct rtos_thread *)rtos_thread_self();

    /*
     * The root system thread is created with the highest priority thread, so
     * that it is not preempted by any other thread, while completing the
     * system initialization. After completing the system initialization,
     * its priority will be lowered to be just above the idle thread.
     */
    FDC_ASSERT(
	root_thread_p->thr_base_priority == RTOS_HIGHEST_THREAD_PRIORITY &&
	root_thread_p->thr_current_priority == root_thread_p->thr_base_priority,
	root_thread_p->thr_base_priority, root_thread_p->thr_current_priority);

    (void)rtos_enter_privileged_mode();
    if (cpu_id == 0)
    {
        /*
         * Display McRTOS console greeting:
         */
        if (!software_reset_happened())
        {
            console_clear();
        }

        console_printf("%s\n%s\n", g_McRTOS_version, g_McRTOS_build_timestamp);

#       ifdef LCD_SUPPORTED
            lcd_display_greetings();
#       endif
    }

    console_printf("CPU core %u: %s started\n", cpu_id,
        g_rtos_system_threads[RTOS_ROOT_SYSTEM_THREAD].p_name_p);

    /*
     * Initialize tick timer interrupts for this CPU core:
     */
    initialize_tick_timer();

    rtos_exit_privileged_mode();
    console_printf("CPU core %u: McRTOS tick timer started\n", cpu_id);

    /*
     * Create the other system threads for this CPU
     */
    for (uint8_t i = RTOS_IDLE_SYSTEM_THREAD; i < ARRAY_SIZE(g_rtos_system_threads); i ++)
    {
        rtos_thread_init(
            &g_rtos_system_threads[i],
            &cpu_controller_p->cpc_system_threads_execution_stacks_p[i],
            &cpu_controller_p->cpc_system_threads[i]);

        console_printf("CPU core %u: %s started\n", cpu_id,
            g_rtos_system_threads[i].p_name_p);
    }

    if (cpu_id == 0)
    {
        /*
         * Do application-specific hardware initialization:
         */
        g_McRTOS_p->rts_app_hardware_init_p();

        g_McRTOS_p->rts_app_hardware_init_called = true;
    }

    /*
     * Do application-specific software initialization:
     */
    g_McRTOS_p->rts_app_software_init_p();

    /*
     * Lower priority of the root system thread, so that the root system
     * thread does not interfere with more time-critical threads:
     */
    (void)rtos_enter_privileged_mode();
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();
    root_thread_p->thr_base_priority = RTOS_LOWEST_THREAD_PRIORITY - 1;
    root_thread_p->thr_current_priority = RTOS_LOWEST_THREAD_PRIORITY - 1;
    rtos_k_restore_cpu_interrupts(cpu_status_register);
    rtos_exit_privileged_mode();

    for ( ; ; )
    {
	while (g_McRTOS_p->rts_console_input_cpu_id != cpu_id) {
	   rtos_condvar_wait(&cpu_controller_p->cpc_inter_processor_interrupt_condvar,
			     NULL, NULL);
	}

	rtos_command_processor();
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "McRTOS root thread should not have terminated", cpu_id, 0);

    return fdc_error;
}


/**
 * McRTOS idle thread function
 */
static fdc_error_t
rtos_idle_thread_f(void *arg)
{
    FDC_ASSERT_UNPRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    rtos_thread_set_comp_region(&g_McRTOS,
                                sizeof g_McRTOS,
                                0,
                                NULL);

    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    for ( ; ; )
    {
        TODO("Implement this")
        /*
         * TODO: Stop generation of tick timer interrupts if there are no active
         * McRTOS timers
         */

        watchdog_restart();

        /*
         * Wait for interrupts:
         */
        if (g_McRTOS_p->rts_stop_idle_cpu)
        {
            wait_for_interrupts();
        }

        TODO("Implement this")
        /*
         * TODO: Re-start generation of tick timer interrupts if it was stopped
         */
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "McRTOS idle thread should not have terminated", cpu_id, 0);

    return fdc_error;
}

#pragma GCC diagnostic pop

