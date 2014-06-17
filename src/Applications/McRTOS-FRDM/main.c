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
#include "frdm_board.h"

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

/**
 * Application thread priorities
 */
enum app_thread_priorities
{
    ACCELEROMETER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 1,
};

static void app_hardware_init(void);
static void app_hardware_stop(void);
static void app_software_init(void);
static void dummy_command(const char *cmd_line);
static fdc_error_t hello_world_thread_thread_f(void *arg);

#if 0 // ???
static fdc_error_t accelerometer_thread_f(void *arg);
#endif

struct app_state_vars {
    /**
     * Current LED color mask
     */
    volatile uint32_t led_color_mask;

    /**
     * Latest X-axis acceleration reading
     */
    volatile int16_t x_acceleration;

    /**
     * Latest Y-axis acceleration reading
     */
    volatile int16_t y_acceleration;

    /**
     * Latest Z-axis acceleration reading
     */
    volatile int16_t z_acceleration;
} __attribute__ ((aligned(SOC_MPU_REGION_ALIGNMENT)));

C_ASSERT(sizeof(struct app_state_vars) % SOC_MPU_REGION_ALIGNMENT == 0);

static struct app_state_vars g_app;


/**
 * Array of application threads for CPU core 0
 */
static const struct rtos_thread_creation_params g_app_threads_cpu0[] =
{
    [0] =
    {
	.p_name_p = "Hello World thread 1",
        .p_function_p = hello_world_thread_thread_f,
        .p_function_arg_p = "first thread",
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
        .p_thread_pp = NULL,
    },

    [1] =
    {
	.p_name_p = "Hello World thread 2",
        .p_function_p = hello_world_thread_thread_f,
        .p_function_arg_p = "second thread",
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
        .p_thread_pp = NULL,
    },

#if 0 // ???
    [1] =
    {
        .p_name_p = "accelerometer thread",
        .p_function_p = accelerometer_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = ACCELEROMETER_THREAD_PRIORITY,
        .p_thread_pp = NULL,
    },
#endif
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
    frdm_board_init();
    DEBUG_PRINTF("FRDM board initialized\n");
}


static
void app_hardware_stop(void)
{
    frdm_board_stop();
    DEBUG_PRINTF("FRDM board stopped\n");
}


static
void app_software_init(void)
{
    static const char g_app_version[] = "FRDM board application v0.1";
    static const char g_app_build_timestamp[] = "built " __DATE__ " " __TIME__;
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_mutex_creation_params mutex_params;
    struct rtos_condvar_creation_params condvar_params;

    console_printf(
        "%s\n%s\n",
        g_app_version, g_app_build_timestamp);

    g_app.led_color_mask = LED_COLOR_RED;
}

static void
dummy_command(const char *cmd_line)
{
    console_printf("This is a dummy command\n");
}

static fdc_error_t
hello_world_thread_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg != NULL, arg, cpu_id);

    fdc_error = rtos_mpu_rw_region_push(&g_app, &g_app + 1);
    if (fdc_error != 0) {
	    goto exit;
    }

    for ( ; ; ) {
	console_printf("%s: %s\n", __func__, arg);
	rtos_thread_delay(2000);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

exit:
    return fdc_error;
}

#if 0 // ???
/**
 * Accelerometer thread
 */
static fdc_error_t
accelerometer_thread_f(void *arg)
{
#   define ACCELEROMETER_SAMPLING_PERIOD_MS  50
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing accelerometer thread ...\n");

    accelerometer_init();
    for ( ; ; )
    {
#if 1
        bool read_ok = accelerometer_read_status(
                        (int16_t *)&g_app.x_acceleration,
                        (int16_t *)&g_app.y_acceleration,
                        (int16_t *)&g_app.z_acceleration);

        if (read_ok) {
//#           if 0
            console_printf("x_accel: %#x, y_accel: %#x, z_accel: %#x\n",
                g_app.x_acceleration & ~0x3f,
                g_app.y_acceleration & ~0x3f,
                g_app.z_acceleration & ~0x3f);
//#           endif
        }
#else
        bool read_ok = accelerometer_detect_motion(
                        (int8_t *)&g_app.x_motion_detection,
                        (int8_t *)&g_app.y_motion_detection,
                        (int8_t *)&g_app.z_motion_detection);
        if (read_ok) {
#           if 0
            console_printf("x_motion: %d, y_motion: %d, z_motion: %d\n",
                g_app.x_motion_detection,
                g_app.y_motion_detection,
                g_app.z_motion_detection);
#           endif
        }
#endif

        rtos_thread_delay(ACCELEROMETER_SAMPLING_PERIOD_MS);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}
#endif // ???
