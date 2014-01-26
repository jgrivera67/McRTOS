/**
 * @file main.c
 *
 * Application main module for TI Stellaris Launchad
 *
 * Copyright (C) 2014 German Rivera
 *
 * @author German Rivera
 */

#include "McRTOS.h"
#include "McRTOS_kernel_services.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "launchpad_board.h"

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

/**
 * Application thread priorities
 */
enum app_thread_priorities
{
    BUTTONS_READER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 1,
};

static void app_hardware_init(void);
static void app_hardware_stop(void);
static void app_software_init(void);
static void dummy_command(const char *cmd_line);
static fdc_error_t buttons_reader_thread_f(void *arg);

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
};

C_ASSERT(
    ARRAY_SIZE(g_app_threads_cpu0) <= RTOS_MAX_NUM_APP_THREADS);

/**
 * Array of application-specific console commands
 */
static const struct rtos_console_command g_app_console_commands[] =
{
    [0] =
    {
        .cmd_name_p = "d",
        .cmd_description_p = "Dummy command",
        .cmd_function_p = dummy_command,
    },
};


/**
 * McRTOS application startup configuration
 */
static const struct rtos_startup_app_configuration g_rtos_app_config =
{
    .stc_app_hardware_init_p = app_hardware_init,
    .stc_app_hardware_stop_p = app_hardware_stop,
    .stc_app_software_init_p = app_software_init,
    .stc_num_app_console_commands = ARRAY_SIZE(g_app_console_commands),
    .stc_app_console_commands_p = g_app_console_commands,
    .stc_per_cpu_config =
    {
        /*
         * CPU core 0
         */
        [0] =
        {
            .stc_idle_thread_hook_function_p = NULL,
            .stc_num_autostart_threads = ARRAY_SIZE(g_app_threads_cpu0),
            .stc_autostart_threads_p = g_app_threads_cpu0,
        },
    },
};

/**
 * Application state variables
 */
struct app_state_vars {
    /**
     * Push buttons state
     */
    volatile bool c_push_buttons[LAUNCHPAD_NUM_PUSH_BUTTONS];
};

static struct app_state_vars g_app;


/**
 * Application's main()
 *
 * This function is invoked from the Reset exception handler for each CPU core.
 */
int
main(void)
{
    rtos_startup(&g_rtos_app_config);

    FDC_ASSERT(false, 0, 0);

    return -1;
}


static
void app_hardware_init(void)
{
    launchpad_board_init();
}


static
void app_hardware_stop(void)
{
    launchpad_board_stop();
}


static
void app_software_init(void)
{
    static const char g_app_version[] = "Launchpad application v0.1";
    static const char g_app_build_timestamp[] = "built " __DATE__ " " __TIME__;
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_mutex_creation_params mutex_params;
    struct rtos_condvar_creation_params condvar_params;

    console_printf(
        "%s\n%s\n",
        g_app_version, g_app_build_timestamp);
}


static void
dummy_command(const char *cmd_line)
{
    console_printf("This is a dummy command\n");
}

static fdc_error_t
buttons_reader_thread_f(void *arg)
{
#   define BUTTONS_READER_THREAD_PERIOD_MS 200
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing buttons reader thread ...\n");

    bool push_buttons[LAUNCHPAD_NUM_PUSH_BUTTONS];

    for ( ; ; ) {
        /*
         * Read push buttons:
         */
        launchpad_push_buttons_read(push_buttons);

        for (natural_t i = 0; i < LAUNCHPAD_NUM_PUSH_BUTTONS; i++) {
            if (push_buttons[i] != g_app.c_push_buttons[i]) {
                g_app.c_push_buttons[i] = push_buttons[i];
		DEBUG_PRINTF("Pushed button %u\n", i);
            }
        }

        rtos_thread_delay(BUTTONS_READER_THREAD_PERIOD_MS);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}

