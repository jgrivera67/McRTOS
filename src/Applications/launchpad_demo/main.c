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
    LED_FLASHING_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 2,
};

static void app_hardware_init(void);
static void app_hardware_stop(void);
static void app_software_init(void);
static void dummy_command(void);
static fdc_error_t buttons_reader_thread_f(void *arg);
static fdc_error_t led_flashing_thread_f(void *arg);

#define RTOS_NUM_APP_THREADS 2

/**
 * Array of execution stacks for application threads
 */
static struct rtos_thread_execution_stack g_app_thread_execution_stacks[RTOS_NUM_APP_THREADS];

/**
 * Array of application threads
 */
static struct rtos_thread g_app_threads[RTOS_NUM_APP_THREADS];

/**
 * Array of application thread creation parameters
 */
static const struct rtos_thread_creation_params g_app_thread_creation_params[] =
{
    [0] =
    {
        .p_name_p = "buttons reader thread",
        .p_function_p = buttons_reader_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = BUTTONS_READER_THREAD_PRIORITY,
    },

    [1] =
    {
        .p_name_p = "LED flashing thread",
        .p_function_p = led_flashing_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = LED_FLASHING_THREAD_PRIORITY,
    },
};

C_ASSERT(ARRAY_SIZE(g_app_thread_creation_params) <=
         ARRAY_SIZE(g_app_thread_execution_stacks));

C_ASSERT(
    ARRAY_SIZE(g_app_thread_creation_params) <= RTOS_MAX_NUM_APP_THREADS);

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
};

/**
 * Application state variables
 */
struct __app_state_vars {
    /**
     * Push buttons state
     */
    volatile bool push_buttons[LPAD_NUM_PUSH_BUTTONS];

    /**
     * Current LED color mask
     */
    volatile uint32_t led_color_mask;
};

struct app_state_vars {
    struct __app_state_vars;
} __attribute__ ((aligned(SOC_MPU_REGION_ALIGNMENT(struct __app_state_vars))));

C_ASSERT(sizeof(struct app_state_vars) % SOC_MPU_REGION_ALIGNMENT(struct __app_state_vars) == 0);

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


static void
app_hardware_init(void)
{
    launchpad_board_init();
}


static void
app_hardware_stop(void)
{
    launchpad_board_stop();
}


static void
app_software_init(void)
{
    static const char g_app_version[] = "Launchpad application v0.1";
    static const char g_app_build_timestamp[] = "built " __DATE__ " " __TIME__;
    static const struct rtos_mpu_data_region g_app_region = {
        .start_addr = &g_app,
        .size = sizeof g_app,
        .read_only = false,
    };

    struct rtos_mpu_data_region old_data_region;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    console_printf(
        "%s\n%s\n",
        g_app_version, g_app_build_timestamp);

    rtos_thread_replace_top_mpu_data_region(&g_app,
                                            sizeof g_app,
                                            false,
                                            &old_data_region);

    g_app.led_color_mask = LED_COLOR_RED;

    /*
     * Create app threads for this CPU
     */
    for (uint8_t i = 0; i < ARRAY_SIZE(g_app_thread_creation_params); i ++)
    {
        rtos_thread_init(
            &g_app_thread_creation_params[i],
            &g_app_thread_execution_stacks[i],
            &g_app_region,
            &g_app_threads[i]);

        console_printf("CPU core %u: %s started\n", cpu_id,
            g_app_thread_creation_params[i].p_name_p);
    }

    rtos_thread_restore_top_mpu_data_region(&old_data_region);
}


static void
dummy_command(void)
{
    console_printf("This is a dummy command\n");
}

static fdc_error_t
buttons_reader_thread_f(void *arg)
{
#   define BUTTONS_READER_THREAD_PERIOD_MS 250
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing buttons reader thread ...\n");

    bool push_buttons[LPAD_NUM_PUSH_BUTTONS];

    for ( ; ; ) {
        /*
         * Read push buttons:
         */
        launchpad_push_buttons_read(push_buttons);

        for (natural_t i = 0; i < LPAD_NUM_PUSH_BUTTONS; i++) {
            if (push_buttons[i] != g_app.push_buttons[i]) {
		if (push_buttons[i]) {
		    console_printf("Pushed button %u\n", i);
		    if (i == LPAD_SW1_BUTTON) {
			g_app.led_color_mask = LED_COLOR_GREEN;
		    } else if (i == LPAD_SW2_BUTTON) {
			g_app.led_color_mask = LED_COLOR_RED;
		    }
		} else {
		    console_printf("Released button %u\n", i);
		}

                g_app.push_buttons[i] = push_buttons[i];
            }
        }

        rtos_thread_delay(BUTTONS_READER_THREAD_PERIOD_MS);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}

static fdc_error_t
led_flashing_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing led flashing thread ...\n");

    uint32_t led_color_mask = g_app.led_color_mask;

    for ( ; ; ) {
	if (led_color_mask != g_app.led_color_mask) {
	    led_color_mask = g_app.led_color_mask;
	    (void)set_rgb_led_color(led_color_mask);
	} else {
            toggle_rgb_led(led_color_mask);
        }

        rtos_thread_delay(100);
	toggle_rgb_led(led_color_mask);
        rtos_thread_delay(500);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}

