/**
 * @file main.c
 *
 * Application main module
 *
 * @author German Rivera 
 */ 

#include "McRTOS.h"
#include "McRTOS_kernel_services.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "print.h"

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"

struct globalDataStr
{
    uint32_t guard;
    uint32_t inc;
    uint32_t t1;
    uint32_t t2;
};

static struct globalDataStr shared = 
{
    .guard = 0,
    .inc = 0,
    .t1 = 0,
    .t2 = 0
};

static const struct lcd_char_attributes lcd_char_attributes_hello_world_thread1 = 
{
    .lcd_dot_size = 1,
    .lcd_foreground_color = LCD_COLOR_GREEN,
    .lcd_background_color = LCD_COLOR_BLACK,
};


static const struct rtos_lcd_putchar_attributes lcd_putchar_attr_hello_world_thread1 =
{
    .lcd_x = 0,
    .lcd_y = 144,
    .lcd_char_attributes_p = &lcd_char_attributes_hello_world_thread1,
};

static const struct lcd_char_attributes lcd_char_attributes_hello_world_thread2 = 
{
    .lcd_dot_size = 1,
    .lcd_foreground_color = LCD_COLOR_RED,
    .lcd_background_color = LCD_COLOR_BLACK,
};

static const struct rtos_lcd_putchar_attributes lcd_putchar_attr_hello_world_thread2 =
{
    .lcd_x = 0,
    .lcd_y = 160,
    .lcd_char_attributes_p = &lcd_char_attributes_hello_world_thread2,
};

/**
 * Hello World Thread
 */
static fdc_error_t
HelloWorldThread(void *p)
{
    FDC_ASSERT_VALID_ROM_POINTER(p, sizeof(void *));
    struct rtos_lcd_putchar_attributes lcd_putchar_attr = 
                *(struct rtos_lcd_putchar_attributes *)p;
       
    bool display_lcd_msg = true;

    for ( ; ; )
    {
        console_printf(
            "%s: thread ID %p\n",
            __func__, rtos_thread_self());

        if (display_lcd_msg) 
        {
            lcd_printf(
                lcd_putchar_attr.lcd_x,
                lcd_putchar_attr.lcd_y,
                lcd_putchar_attr.lcd_char_attributes_p, 
                "%s: thread ID %p",
                __func__, rtos_thread_self());
        }
        else
        {
            lcd_printf(
                lcd_putchar_attr.lcd_x,
                lcd_putchar_attr.lcd_y,
                lcd_putchar_attr.lcd_char_attributes_p, 
                "                                        ");
        }

        display_lcd_msg ^= true;

        rtos_thread_delay(1 * 1000);
        //rtos_thread_yield();
    }

    FDC_ASSERT(false, p, 0);

    return (0);
}


/**
 * Producer thread
 */
static fdc_error_t
entry1Thread(void *p)
{
    struct globalDataStr *s = (struct globalDataStr *)p;

    console_printf("I am thread %s\n", __func__);

    for ( ; ; )
    {
        s->t1 ++;
        if (s->guard == 0)
        {
            s->guard = 1;
            s->inc ++;
            s->guard = 0;
        }
    }

    return 0;
}


/**
 * Consumer thread
 */
static fdc_error_t
entry2Thread(void *p)
{
    struct globalDataStr *s = (struct globalDataStr *)p;

    console_printf("I am thread %s\n", __func__);

    for ( ; ; )
    {
        s->t2 ++;
        if (s->guard==0)
        {
            s->guard = 1;
            s->inc --;
            s->guard = 0;
        }
    }

    return 0;
}


/**
 * Monitor thread
 */
static fdc_error_t
entry3Thread(void *p)
{
    struct globalDataStr *s = ( struct globalDataStr *)p;

    console_printf("I am thread %s\n", __func__);

    for ( ; ; )
    {
        if (s->guard == 0)
        {
            s->guard = 1;
            if (s->inc > 3)
            {
                console_printf("%s: Error: s.inc > 3: %u\n", __func__, s->inc);
                s->inc = 0;
                //return -1;
            }

            s->guard = 0;
        }
    }

    return 0;
}


/**
 * Idle thread
 */
static fdc_error_t
idleThread(void *p)
{
    console_printf("I am thread %s\n", __func__);

    for ( ; ; )
    {
        rtos_thread_delay(10 * 1000);
    }

    return (0);
}


#define APP_THREAD_PRIORITY  (RTOS_LOWEST_THREAD_PRIORITY - 1)

/**
 * Array of application threads for CPU core 0
 * 
 * NOTE: This array cannot be const, because McRTOS 
 * returns pointers to created McRTOS threads in the
 * rtos_thread_creation_params structs themselves.
 */
static struct rtos_thread_creation_params g_app_threads_cpu0[] =
{
    [0] =
    {
        .p_name_p = "HelloWorldThread 1",
        .p_function_p = HelloWorldThread,
        .p_function_arg_p = (void *)&lcd_putchar_attr_hello_world_thread1,
        .p_priority = APP_THREAD_PRIORITY,
        .p_console_channel = 0,
        .p_lcd_channel = 0,
    },

    [1] =
    {
        .p_name_p = "HelloWorldThread 2",
        .p_function_p = HelloWorldThread,
        .p_function_arg_p = (void *)&lcd_putchar_attr_hello_world_thread2,
        .p_priority = APP_THREAD_PRIORITY,
        .p_console_channel = 0,
        .p_lcd_channel = 0,
    },

    [2] =
    {
        .p_name_p = "Producer thread",
        .p_function_p = entry1Thread,
        .p_function_arg_p = &shared,
        .p_priority = APP_THREAD_PRIORITY,
        .p_console_channel = 0,
        .p_lcd_channel = RTOS_LCD_CHANNEL_NONE,
    },

    [3] =
    {
        .p_name_p = "Consumer thread",
        .p_function_p = entry2Thread,
        .p_function_arg_p = &shared,
        .p_priority = APP_THREAD_PRIORITY,
        .p_console_channel = 0,
        .p_lcd_channel = RTOS_LCD_CHANNEL_NONE,
    },

    [4] =
    {
        .p_name_p = "Monitor thread",
        .p_function_p = entry3Thread,
        .p_function_arg_p = &shared,
        .p_priority = APP_THREAD_PRIORITY,
        .p_console_channel = 0,
        .p_lcd_channel = RTOS_LCD_CHANNEL_NONE,
    },

#if 0
    [5] =
    {
        .p_name_p = "Application idle thread",
        .p_function_p = idleThread,
        .p_function_arg_p = &shared,
        .p_priority = APP_THREAD_PRIORITY,
        .p_console_channel = 0,
        .p_lcd_channel = RTOS_LCD_CHANNEL_NONE,
    },
#endif
};

C_ASSERT(
    ARRAY_SIZE(g_app_threads_cpu0) <= RTOS_MAX_NUM_APP_THREADS);

/**
 * McRTOS startup configuration
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


