/**
 * @file main.c
 *
 * Application main module
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 

#include "McRTOS.h"
#include "McRTOS_kernel_services.h"
#include "failure_data_capture.h"
#include "utils.h"

TODO("Remove these pragmas")
//#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

/**
 * Application thread priorities
 */
enum app_thread_priorities
{
    BUTTONS_READER_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
    TRIMPOT_READER_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
};

static fdc_error_t buttons_reader_thread_f(void *arg);
static fdc_error_t trimpot_reader_thread_f(void *arg);

/**
 * Array of application threads for CPU core 0
 */
static const struct rtos_thread_creation_params g_app_threads_cpu0[] =
{
    [0] =
    {
        .p_name_p = "buttons reader thread",
        .p_function_p = buttons_reader_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = BUTTONS_READER_THREAD_PRIORITY,
        .p_thread_pp = NULL,
    },

    [1] =
    {
        .p_name_p = "trimpot reader thread",
        .p_function_p = trimpot_reader_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = TRIMPOT_READER_THREAD_PRIORITY,
        .p_thread_pp = NULL,
    },
};

C_ASSERT(
    ARRAY_SIZE(g_app_threads_cpu0) <= RTOS_MAX_NUM_APP_THREADS);

/**
 * McRTOS application startup configuration
 */ 
static const struct rtos_per_cpu_startup_app_configuration g_rtos_app_config[] =
{
    /*
     * CPU core 0
     */
    [0] =
    {
        .stc_idle_thread_hook_function_p = NULL,

        .stc_num_autostart_threads = ARRAY_SIZE(g_app_threads_cpu0),
        .stc_autostart_threads_p = g_app_threads_cpu0,

        .stc_num_prebuilt_mutexes = 0,
        .stc_prebuilt_mutexes_p = NULL,

        .stc_num_prebuilt_condvars = 0,
        .stc_prebuilt_condvars_p = NULL,

        .stc_num_prebuilt_timers = 0,
        .stc_prebuilt_timers_p = NULL,
    },
};

C_ASSERT(ARRAY_SIZE(g_rtos_app_config) == SOC_NUM_CPU_CORES);


/**
 * Application's main()
 *
 * This function is invoked from the Reset exception handler for each CPU core.
 */ 
int
main(void)
{
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    rtos_startup(&g_rtos_app_config[cpu_id]);
    
    FDC_ASSERT(false, 0, 0);

    return -1;
}


/**
 * Buttons reader thread
 */
static fdc_error_t
buttons_reader_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing McRTOS buttons reader thread ...\n");

#if 0 // ???
    uint32_t buttons_pressed;

    init_buttons(g_buttons_device_p);

    for ( ; ; )
    {
        buttons_pressed = read_buttons(g_buttons_device_p);

        buttons_pressed &= (BUTTON1_PRESSED_MASK | BUTTON2_PRESSED_MASK);
        
        if (buttons_pressed == BUTTON1_PRESSED_MASK)
        {
        }
        else if (buttons_pressed == BUTTON2_PRESSED_MASK)
        {
        }
        else if (buttons_pressed != 0)
        {
            /*
             * Both buttons are pressed at the same time
             */
            DBG_ASSERT(
                buttons_pressed == (BUTTON1_PRESSED_MASK | BUTTON2_PRESSED_MASK),
                buttons_pressed, 0);
        }

#ifndef USE_GPIO_INTERRUPTS
        rtos_thread_delay(200);
#endif
    } 
#else
   for ( ; ; )
       ;
#endif

    fdc_error = CAPTURE_FDC_ERROR(
        "McRTOS buttons reader thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}


volatile uint32_t g_last_trimpot_reading = 0; // XXX

/**
 * Trimpot reader thread
 */
static fdc_error_t
trimpot_reader_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing McRTOS trimpot sensing thread ...\n");

#if 0 // ???
    uint32_t trimpot_reading;
    uint32_t old_trimpot_reading;

    init_trimpot();

    old_trimpot_reading = 0;
    for ( ; ; )
    {
        trimpot_reading = read_trimpot() & ~0xf;
        if (trimpot_reading != old_trimpot_reading)
        {
            console_printf("Trimpot changed: 0x%x\n", trimpot_reading);

            g_last_trimpot_reading = trimpot_reading;
            old_trimpot_reading = trimpot_reading;
        }
        
        rtos_thread_delay(250);
    }   

#else
   for ( ; ; )
       ;
#endif

    fdc_error = CAPTURE_FDC_ERROR(
        "McRTOS trimpot sensing thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}

