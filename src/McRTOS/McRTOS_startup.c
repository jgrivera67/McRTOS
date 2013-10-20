/**
 * @file McRTOS_startup.c
 *
 * McRTOS startup module.
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 

#include "McRTOS.h"
#include "McRTOS_internals.h"
#include "failure_data_capture.h"
#include "hardware_abstractions.h"
#include "utils.h"

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-function"

/**
 * System thread priorities
 */
enum system_thread_priorities
{
    RTOS_ROOT_THREAD_PRIORITY = 0,
    RTOS_COMMAND_LINE_THREAD_PRIORITY = 1,
    RTOS_IDLE_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY,
};

#define RTOS_CPU_CONTROLLER_INITIALIZER(_cpu_id) \
        .rts_cpu_controllers[_cpu_id] =                                 \
        {                                                               \
            .cpc_signature = RTOS_CPU_CONTROLLER_SIGNATURE,             \
            .cpc_cpu_id = (_cpu_id),                                    \
            .cpc_current_execution_context_p = NULL,                    \
            .cpc_current_thread_p = NULL,                               \
            .cpc_runnable_thread_priorities = 0,                        \
            .cpc_active_internal_interrupts = 0,                        \
            .cpc_active_external_interrupts = 0,                        \
            .cpc_nested_interrupts_count = 0,                           \
            .cpc_thread_scheduler_calls = 0,                            \
            .cpc_accumulated_thread_scheduler_overhead = 0,             \
            .cpc_interrupts_disabled_being_measured_count = 0,          \
            .cpc_startup_completed = false,                             \
            .cpc_pending_thread_time_slice_decrement = false,           \
            .cpc_current_timer_wheel_spoke_index = 0,                   \
            .cpc_ticks_since_boot_count = 0,                            \
            .cpc_interrupts_disabled_start_time_stamp = 0,              \
            .cpc_longest_time_interrupts_disabled = 0,                  \
            .cpc_latest_measurement_time_interrupts_disabled = 0,       \
            .cpc_app_config_p = NULL,                                   \
            FDC_INFO_INITIALIZER(.cpc_failures_info),                   \
            .cpc_execution_contexts_list_anchor =                       \
                GLIST_NODE_INITIALIZER(                                 \
                    g_McRTOS.rts_cpu_controllers[_cpu_id].              \
                        cpc_execution_contexts_list_anchor),            \
            .cpc_preemption_chain_anchor =                              \
                GLIST_NODE_INITIALIZER(                                 \
                    g_McRTOS.rts_cpu_controllers[_cpu_id].              \
                        cpc_preemption_chain_anchor),                   \
        }


static void rtos_init_reset_execution_context(
                struct rtos_cpu_controller *cpu_controller_p);

static fdc_error_t rtos_root_thread_f(void *arg);
static fdc_error_t rtos_idle_thread_f(void *arg);
static fdc_error_t rtos_touch_screen_reader_thread_f(void *arg);
static void rtos_parse_command_line(const char *cmd_line);
static void McRTOS_display_help(void);
static void McRTOS_display_stats(void);

static const char g_McRTOS_version[] = "McRTOS v0.04";

static const char g_McRTOS_build_timestamp[] = "built " __DATE__ " " __TIME__;

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
        .p_priority = RTOS_ROOT_THREAD_PRIORITY,
#       ifdef LCD_SUPPORTED
        .p_lcd_channel = RTOS_LCD_CHANNEL_NONE,
#       endif
        .p_thread_pp = NULL,
    },

    [RTOS_IDLE_SYSTEM_THREAD] =
    {
        .p_name_p = "McRTOS idle thread",
        .p_function_p = rtos_idle_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = RTOS_IDLE_THREAD_PRIORITY,
#       ifdef LCD_SUPPORTED
        .p_lcd_channel = RTOS_LCD_CHANNEL_NONE,
#       endif
        .p_thread_pp = NULL,
    },
};

C_ASSERT(ARRAY_SIZE(g_rtos_system_threads) <= RTOS_NUM_SYSTEM_THREADS_PER_CPU);

/**
 * McRTOS global state variables
 */ 
static struct McRTOS g_McRTOS =
{
    .rts_signature = MCRTOS_SIGNATURE,

    .rts_next_free_interrupt_p = &g_McRTOS.rts_interrupts[0],

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

#ifdef LCD_SUPPORTED
    .rts_current_lcd_channel = RTOS_COMMAND_LINE_LCD_CHANNEL,
#endif

#ifdef LPC2478_SOC
    .rts_stop_idle_cpu = false,
#else
    .rts_stop_idle_cpu = true,
#endif

#ifdef RTOS_USE_DRAM_FOR_APP_THREAD_STACKS
    .rts_next_free_app_thread_stack_p = 
        (struct rtos_thread_execution_stack *)RTOS_APP_THREAD_DRAM_STACKS_BASE_ADDR,
    
    .rts_app_threads_execution_stacks_p =
        (struct rtos_thread_execution_stack *)RTOS_APP_THREAD_DRAM_STACKS_BASE_ADDR,
#else
    .rts_next_free_app_thread_stack_p = &g_McRTOS.rts_app_threads_execution_stacks[0],
    
    .rts_app_threads_execution_stacks_p = g_McRTOS.rts_app_threads_execution_stacks,
#endif

    RTOS_CPU_CONTROLLER_INITIALIZER(0),
};

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

#ifdef RTOS_USE_DRAM_FOR_APP_THREAD_STACKS
    FDC_ASSERT(
        g_McRTOS_p->rts_next_free_app_thread_stack_p ==
        (struct rtos_thread_execution_stack *)
            &g_sdram_map_p->sdr_rtos_app_thread_stacks[0],
        g_McRTOS_p->rts_next_free_app_thread_stack_p,
        &g_sdram_map_p->sdr_rtos_app_thread_stacks[0]);
#endif
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
 * @param   rtos_app_config_p Pointer to startup application configuration for
 *          the calling CPU core.
 *
 * @return  none
 */
void
rtos_startup( 
    _IN_ const struct rtos_per_cpu_startup_app_configuration *rtos_app_config_p,
    _IN_ app_hardware_init_t *app_hardware_init_p,
    _IN_ app_hardware_stop_t *app_hardware_stop_p)
{
    FDC_ASSERT_COMING_FROM_RESET();
    FDC_ASSERT_CPU_IS_LITTLE_ENDIAN();
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(rtos_app_config_p, sizeof(uint32_t));
    FDC_ASSERT_VALID_FUNCTION_POINTER(app_hardware_init_p);
    FDC_ASSERT_VALID_FUNCTION_POINTER(app_hardware_stop_p);


    struct rtos_thread *rtos_thread_p;
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

    g_McRTOS_p->rts_app_hardware_init_p = app_hardware_init_p;
    g_McRTOS_p->rts_app_hardware_stop_p = app_hardware_stop_p;

    rtos_init_reset_execution_context(cpu_controller_p);

    cpu_controller_p->cpc_app_config_p = rtos_app_config_p;

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

        g_McRTOS_p->rts_app_hardware_init_p();

#       ifdef _RELIABILITY_CHECKS_
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

    /*
     * Create root system thread to continue system initialization in that thread:
     */
    rtos_thread_p = 
        &(cpu_controller_p->cpc_system_threads[RTOS_ROOT_SYSTEM_THREAD]);

    rtos_k_thread_init(
        &g_rtos_system_threads[RTOS_ROOT_SYSTEM_THREAD],
        &cpu_controller_p->cpc_system_threads_execution_stacks[RTOS_ROOT_SYSTEM_THREAD],
        cpu_id,
        true,
        RTOS_ROOT_SYSTEM_THREAD,
        rtos_thread_p);

    FDC_ASSERT(
        cpu_controller_p->cpc_current_execution_context_p == 
        &cpu_controller_p->cpc_reset_execution_context,
        cpu_controller_p->cpc_current_execution_context_p, 
        cpu_controller_p);

    cpu_controller_p->cpc_startup_completed = true;

    /*
     * Reset the "longest time interrupts disabled" to 0, as this 
     * is boot time, and should not be taken into account.
     *
     * NOTE: We need to reset this because, board_init() called device
     * initialization functions which may call
     * rtos_k_disable_cpu_interrupts()/rtos_k_restore_cpu_interrupts()
     */
    cpu_controller_p->cpc_longest_time_interrupts_disabled = 0;

    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();

    /*
     * Start measuring interrupts disabled time from here:
     */
    RTOS_START_INTERRUPTS_DISABLED_TIME_MEASURE();

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
    FDC_ASSERT_VALID_FUNCTION_POINTER(g_McRTOS_p->rts_app_hardware_stop_p);

    console_printf("\nMcRTOS rebooting ...\n");

    g_McRTOS_p->rts_app_hardware_stop_p();
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


/**
 * McRTOS root thread function
 */
static fdc_error_t
rtos_root_thread_f(void *arg)
{
    FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
                                    &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    const struct rtos_per_cpu_startup_app_configuration *rtos_app_config_p =
                                            cpu_controller_p->cpc_app_config_p;

    FDC_ASSERT(arg == NULL, arg, cpu_id);
    FDC_ASSERT(rtos_app_config_p != NULL, 0, 0);

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
    }

    console_printf("CPU core %u: %s started\n", cpu_id,
        g_rtos_system_threads[RTOS_ROOT_SYSTEM_THREAD].p_name_p);

    /*
     * Initialize tick timer interrupts for this CPU core:
     *
     * NOTE: Even if tick timer interrupts start coming right away,
     * we will not get interrupted yet, as we still have interrupts
     * disabled.
     */
    initialize_tick_timer(); 

    console_printf("CPU core %u: McRTOS tick timer started\n", cpu_id);

    /*
     * Create the other system threads for this CPU
     */ 
    for (uint8_t i = RTOS_IDLE_SYSTEM_THREAD; i < ARRAY_SIZE(g_rtos_system_threads); i ++)
    {
        struct rtos_thread *rtos_thread_p = &cpu_controller_p->cpc_system_threads[i];
        rtos_k_thread_init(
            &g_rtos_system_threads[i],
            &cpu_controller_p->cpc_system_threads_execution_stacks[i],
            cpu_id,
            true,
            i,
            rtos_thread_p);

        console_printf("CPU core %u: %s started\n", cpu_id,
            g_rtos_system_threads[i].p_name_p);
    }

    /*
     * Create pre-built application mutexes for this CPU core:
     */

    rtos_num_app_mutexes_t num_app_mutexes =
        rtos_app_config_p->stc_num_prebuilt_mutexes;

    for (rtos_num_app_mutexes_t i = 0; i < num_app_mutexes; i ++)
    {
        fdc_error = rtos_k_create_mutex(
                        &rtos_app_config_p->stc_prebuilt_mutexes_p[i]);

        if (fdc_error != 0)
        {
            console_printf(
                "CPU core %u: *** Error creating application mutex %u ***\n",
                cpu_id, i);

            fatal_error_handler(fdc_error);
        }

        console_printf("CPU core %u: %s created\n", cpu_id,
            rtos_app_config_p->stc_prebuilt_mutexes_p[i].p_name_p);

    }

    /*
     * Create pre-built application condvars for this CPU core:
     */

    rtos_num_app_condvars_t num_app_condvars =
        rtos_app_config_p->stc_num_prebuilt_condvars;

    for (rtos_num_app_condvars_t i = 0; i < num_app_condvars; i ++)
    {
        fdc_error = rtos_k_create_condvar(
                        &rtos_app_config_p->stc_prebuilt_condvars_p[i]);

        if (fdc_error != 0)
        {
            console_printf(
                "CPU core %u: *** Error creating application condvar %u ***\n",
                cpu_id, i);

            fatal_error_handler(fdc_error);
        }

        console_printf("CPU core %u: %s created\n", cpu_id,
            rtos_app_config_p->stc_prebuilt_condvars_p[i].p_name_p);
    }

    /*
     * Create auto-start application threads for this CPU core
     */

    rtos_num_app_threads_t num_app_threads =
        rtos_app_config_p->stc_num_autostart_threads;

    for (rtos_num_app_threads_t i = 0; i < num_app_threads; i ++)
    {
        fdc_error = rtos_k_create_thread(
                        &rtos_app_config_p->stc_autostart_threads_p[i]);

        if (fdc_error != 0)
        {
            console_printf(
                "CPU core %u: *** Error creating application thread %u ***\n",
                cpu_id, i);

            fatal_error_handler(fdc_error);
        }

        console_printf("CPU core %u: %s started\n", cpu_id,
            rtos_app_config_p->stc_autostart_threads_p[i].p_name_p);
    }
    
    if (cpu_id == 0)
    {
#       ifdef LCD_SUPPORTED
            lcd_display_greetings();
#       endif
    }

    for ( ; ; )
    {
        console_printf("McRTOS> ");
        read_command_line(
            (putchar_func_t *)rtos_console_putchar,
            (getchar_func_t *)rtos_console_getchar,
            NULL,
            g_McRTOS_p->rts_command_line_buffer,
            RTOS_COMMAND_LINE_BUFFER_SIZE);

        rtos_parse_command_line(
            g_McRTOS_p->rts_command_line_buffer);
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
    FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
                                    &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    FDC_ASSERT(arg == NULL, arg, cpu_id);
    FDC_ASSERT(cpu_controller_p->cpc_app_config_p != NULL, cpu_id, 0);

    rtos_idle_thread_hook_function_t *idle_thread_hook_function_p = 
        cpu_controller_p->cpc_app_config_p->stc_idle_thread_hook_function_p;

    for ( ; ; )
    {
        /*
         * Invoke application hook:
         */
        if (idle_thread_hook_function_p != NULL)
        {
            idle_thread_hook_function_p();
        }

        TODO("Implement this")
        /*
         * TODO: Stop generation of tick timer interrupts if there are no active
         * McRTOS timers
         */

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


static void
rtos_parse_command_line(
    const char *cmd_line)
{
    uint8_t c = cmd_line[0]; // ???

    switch (c) {
    case 'c':
        console_clear();
        break;

    case 'h':
        McRTOS_display_help();
        break;

    case 'i':
        g_McRTOS_p->rts_stop_idle_cpu ^= true;
        break;

    case 'r':
        rtos_reboot();
        /*UNREACHABLE*/
        break;

    case 's':
        McRTOS_display_stats();
        break;

    default:
        console_printf("Invalid command: \'%s\' (type h for help)\n", cmd_line);
    }
}


static void
McRTOS_display_help(void)
{
    console_printf(
        "\nMcRTOS commands\n"
        "\tc - clear screen\n"
        "\th - display this message\n"
        "\ti - toggle on/off stopping the CPU in the idle thread\n"
        "\tr - reset CPU\n"
        "\ts - display McRTOS stats (until any key is pressed)\n\n");
}


static void
McRTOS_display_stats(void)
{
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    console_clear();
    console_printf("McRTOS stats for CPU core %u\n\n", cpu_id);
    
    uint32_t seconds =
        (cpu_controller_p->cpc_ticks_since_boot_count / 1000) * RTOS_MILLISECONDS_PER_TICK;
    uint32_t minutes = seconds / 60;
    uint32_t hours = minutes / 60;

    minutes %= 60;
    seconds %= 60;
    
    console_printf("Up time: %u ticks (%u hours, %u minutes, %u seconds)\n",
        cpu_controller_p->cpc_ticks_since_boot_count, hours, minutes, seconds);

    console_printf("Longest interrupts disabled time: %u us\n",
         CPU_CLOCK_CYCLES_TO_MICROSECONDS(cpu_controller_p->cpc_longest_time_interrupts_disabled));

    console_printf("Latest interrupts disabled time: %u us\n",
         CPU_CLOCK_CYCLES_TO_MICROSECONDS(cpu_controller_p->cpc_latest_measurement_time_interrupts_disabled));

    console_printf("Stop CPU in idle thread: %s\n\n",
        g_McRTOS_p->rts_stop_idle_cpu ? "On" : "Off");


    struct glist_node *context_node_p;

    console_printf(
        "Context    Name                           Priority Switched-out Preempted  Tstamp last  Switched-out\n"
        "address                                            count        count      switched-out history\n"
        "========== ============================== ======== ============ ========== ============ ============\n");

    GLIST_FOR_EACH_NODE(
        context_node_p,
        &cpu_controller_p->cpc_execution_contexts_list_anchor)
    {
        struct rtos_execution_context *context_p = 
            GLIST_NODE_ENTRY(
                context_node_p, struct rtos_execution_context, ctx_list_node);

        uint8_t context_type_symbol;
        uint32_t priority;

        switch (context_p->ctx_context_type)
        {
        case RTOS_RESET_CONTEXT:
            context_type_symbol = 'R';
            priority = 0;
            break;

        case RTOS_THREAD_CONTEXT:
            context_type_symbol = 'T';
            priority =  RTOS_EXECUTION_CONTEXT_GET_THREAD(context_p)->thr_current_priority;
            break;

        case RTOS_INTERRUPT_CONTEXT:
            context_type_symbol = 'I';
            priority = RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(context_p)->int_priority;
            break;

        default:
            FDC_ASSERT(false, context_p->ctx_context_type, context_p);
        }

        console_printf(
            "%#8p %30s %c%7u %12u %10u %12u %#x%x\n",
            context_p,
            context_p->ctx_name_p,
            context_type_symbol,
            priority,
            context_p->ctx_switched_out_counter,
            context_p->ctx_preempted_counter,
            context_p->ctx_last_switched_out_time_stamp_in_ticks,
            context_p->ctx_switched_out_reason_history,
            context_p->ctx_last_switched_out_reason
        );
    }
}
