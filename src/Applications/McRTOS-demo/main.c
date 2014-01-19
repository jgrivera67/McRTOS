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

TODO("Remove these pragmas")
//#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

/**
 * Application thread priorities
 */
enum app_thread_priorities
{
    MP3_PLAYER_PAUSED_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 7,
    MP3_PLAYER_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 6,
    HELLO_WORLD_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 2,
    PROD_CONS_MONITOR_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 2,
    PROD_CONS_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
    TILE_DRAWING_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 2,
};

struct globalDataStr
{
    uint32_t guard;
    uint32_t inc;
    uint32_t t1;
    uint32_t t2;
};


static fdc_error_t hello_world_thread_f(void *arg);
static fdc_error_t tile_drawing_thread1_f(void *arg);
static fdc_error_t tile_drawing_thread2_f(void *arg);
static fdc_error_t tile_drawing_thread3_f(void *arg);
static fdc_error_t producer_thread_f(void *p);
static fdc_error_t consumer_thread_f(void *p);
static fdc_error_t monitor_thread_f(void *p);


static const struct lcd_char_attributes lcd_char_attributes_hello_world_thread1 = 
{
    .lcd_dot_size = 1,
    .lcd_foreground_color = LCD_COLOR_YELLOW,
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
    .lcd_foreground_color = LCD_COLOR_BLUE,
    .lcd_background_color = LCD_COLOR_BLACK,
};

static const struct rtos_lcd_putchar_attributes lcd_putchar_attr_hello_world_thread2 =
{
    .lcd_x = 0,
    .lcd_y = 160,
    .lcd_char_attributes_p = &lcd_char_attributes_hello_world_thread2,
};

static const struct lcd_char_attributes lcd_char_attributes_hello_world_thread3 = 
{
    .lcd_dot_size = 1,
    .lcd_foreground_color = LCD_COLOR_RED,
    .lcd_background_color = LCD_COLOR_BLACK,
};

static const struct rtos_lcd_putchar_attributes lcd_putchar_attr_hello_world_thread3 =
{
    .lcd_x = 0,
    .lcd_y = 176,
    .lcd_char_attributes_p = &lcd_char_attributes_hello_world_thread3,
};

static const struct lcd_char_attributes lcd_char_attributes_hello_world_thread4 = 
{
    .lcd_dot_size = 1,
    .lcd_foreground_color = LCD_COLOR_GREEN,
    .lcd_background_color = LCD_COLOR_BLACK,
};

static const struct rtos_lcd_putchar_attributes lcd_putchar_attr_hello_world_thread4 =
{
    .lcd_x = 0,
    .lcd_y = 192,
    .lcd_char_attributes_p = &lcd_char_attributes_hello_world_thread4,
};


static struct globalDataStr g_prod_cons_shared = 
{
    .guard = 0,
    .inc = 0,
    .t1 = 0,
    .t2 = 0
};

/**
 * Array of application threads for CPU core 0
 */
static const struct rtos_thread_creation_params g_app_threads_cpu0[] =
{
    [0] =
    {
        .p_name_p = "Hello world thread 1",
        .p_function_p = hello_world_thread_f,
        .p_function_arg_p = (void *)&lcd_putchar_attr_hello_world_thread1,
        .p_priority = HELLO_WORLD_THREAD_PRIORITY,
        .p_console_channel = RTOS_APP_CONSOLE_CHANNEL,
        .p_lcd_channel = RTOS_APP_LCD_CHANNEL,
        .p_thread_pp = NULL,
    },

    [1] =
    {
        .p_name_p = "Hello world thread 2",
        .p_function_p = hello_world_thread_f,
        .p_function_arg_p = (void *)&lcd_putchar_attr_hello_world_thread2,
        .p_priority = HELLO_WORLD_THREAD_PRIORITY,
        .p_console_channel = RTOS_APP_CONSOLE_CHANNEL,
        .p_lcd_channel = RTOS_APP_LCD_CHANNEL,
        .p_thread_pp = NULL,
    },

    [2] =
    {
        .p_name_p = "Hello world thread 3",
        .p_function_p = hello_world_thread_f,
        .p_function_arg_p = (void *)&lcd_putchar_attr_hello_world_thread3,
        .p_priority = HELLO_WORLD_THREAD_PRIORITY,
        .p_console_channel = RTOS_APP_CONSOLE_CHANNEL,
        .p_lcd_channel = RTOS_APP_LCD_CHANNEL,
        .p_thread_pp = NULL,
    },

    [3] =
    {
        .p_name_p = "Hello world thread 4",
        .p_function_p = hello_world_thread_f,
        .p_function_arg_p = (void *)&lcd_putchar_attr_hello_world_thread4,
        .p_priority = HELLO_WORLD_THREAD_PRIORITY,
        .p_console_channel = RTOS_APP_CONSOLE_CHANNEL,
        .p_lcd_channel = RTOS_APP_LCD_CHANNEL,
        .p_thread_pp = NULL,
    },

    [4] =
    {
        .p_name_p = "Producer thread",
        .p_function_p = producer_thread_f,
        .p_function_arg_p = &g_prod_cons_shared,
        .p_priority = PROD_CONS_THREAD_PRIORITY,
        .p_console_channel = RTOS_APP_CONSOLE_CHANNEL,
        .p_lcd_channel = RTOS_LCD_CHANNEL_NONE,
        .p_thread_pp = NULL,
    },

    [5] =
    {
        .p_name_p = "Consumer thread",
        .p_function_p = consumer_thread_f,
        .p_function_arg_p = &g_prod_cons_shared,
        .p_priority = PROD_CONS_THREAD_PRIORITY,
        .p_console_channel = RTOS_APP_CONSOLE_CHANNEL,
        .p_lcd_channel = RTOS_LCD_CHANNEL_NONE,
        .p_thread_pp = NULL,
    },

    [6] =
    {
        .p_name_p = "Monitor thread",
        .p_function_p = monitor_thread_f,
        .p_function_arg_p = &g_prod_cons_shared,
        .p_priority = PROD_CONS_MONITOR_THREAD_PRIORITY,
        .p_console_channel = RTOS_APP_CONSOLE_CHANNEL,
        .p_lcd_channel = RTOS_LCD_CHANNEL_NONE,
        .p_thread_pp = NULL,
    },

    [7] =
    {
        .p_name_p = "Tile drawing thread 1",
        .p_function_p = tile_drawing_thread1_f,
        .p_function_arg_p = NULL,
        .p_priority = TILE_DRAWING_THREAD_PRIORITY,
        .p_console_channel = RTOS_APP_CONSOLE_CHANNEL,
        .p_lcd_channel = RTOS_TILES_LCD_CHANNEL,
        .p_thread_pp = NULL,
    },

    [8] =
    {
        .p_name_p = "Tile drawing thread 2",
        .p_function_p = tile_drawing_thread2_f,
        .p_function_arg_p = NULL,
        .p_priority = TILE_DRAWING_THREAD_PRIORITY,
        .p_console_channel = RTOS_APP_CONSOLE_CHANNEL,
        .p_lcd_channel = RTOS_TILES_LCD_CHANNEL,
        .p_thread_pp = NULL,
    },

    [9] =
    {
        .p_name_p = "Tile drawing thread 3",
        .p_function_p = tile_drawing_thread3_f,
        .p_function_arg_p = NULL,
        .p_priority = TILE_DRAWING_THREAD_PRIORITY,
        .p_console_channel = RTOS_APP_CONSOLE_CHANNEL,
        .p_lcd_channel = RTOS_TILES_LCD_CHANNEL,
        .p_thread_pp = NULL,
    },
};

C_ASSERT(
    ARRAY_SIZE(g_app_threads_cpu0) <= RTOS_MAX_NUM_APP_THREADS);

/**
 * Array of application mutexes for CPU core 0
 */
#if 0
static const struct rtos_mutex_creation_params g_app_mutexes_cpu0[] =
{
    [0] =
    {
        .p_name_p = "console output mutex",
        .p_mutex_pp = &g_console_output_mutex_p,
    },

    [1] =
    {
        .p_name_p = "lcd output mutex",
        .p_mutex_pp = &g_lcd_output_mutex_p,
    },
};
#endif

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

#if 0
        .stc_num_prebuilt_mutexes = ARRAY_SIZE(g_app_mutexes_cpu0),
        .stc_prebuilt_mutexes_p = g_app_mutexes_cpu0,
#else
        .stc_num_prebuilt_mutexes = 0,
        .stc_prebuilt_mutexes_p = NULL,
#endif
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
 * Hello World Thread
 */
static fdc_error_t
hello_world_thread_f(void *arg)
{
    FDC_ASSERT_VALID_ROM_POINTER(arg, sizeof(void *));
    struct rtos_lcd_putchar_attributes lcd_putchar_attr = 
                *(struct rtos_lcd_putchar_attributes *)arg;
       
    bool display_lcd_msg = true;

    const struct rtos_thread *thread_p = rtos_thread_self();
    const char *thread_name_p = rtos_thread_name(thread_p);

    for ( ; ; )
    {
        console_printf(
            "%s: thread ID %p\n",
            thread_name_p, thread_p);

        if (display_lcd_msg) 
        {
            lcd_printf(
                lcd_putchar_attr.lcd_x,
                lcd_putchar_attr.lcd_y,
                lcd_putchar_attr.lcd_char_attributes_p, 
                "%s: thread ID %p",
                thread_name_p, thread_p);
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

    return CAPTURE_FDC_ERROR(
                "thread should not have terminated", __func__,
                rtos_thread_self());
}


/**
 * Thread that draws the second tile strip
 */
static fdc_error_t
tile_drawing_thread1_f(void *arg)
{
    bool display_tile = true;
    uint_fast16_t x = 0;

    FDC_ASSERT(arg == NULL, arg, 0);

    for ( ; ; )
    {
        if (display_tile)
        {
            (void)rtos_lcd_draw_tile(x, 0*TILE_WIDTH, LCD_COLOR_YELLOW);
        }
        else
        {
            (void)rtos_lcd_draw_tile(x, 0*TILE_WIDTH, LCD_COLOR_BLUE);
        }

        toggle_usb_device_link_led();

        x += TILE_WIDTH;
        if (x == LCD_X_SIZE)
        {
            x = 0;
            display_tile ^= true;
            if (display_tile)
            {
                rtos_thread_delay(500);
            }
            else
            {
                rtos_thread_delay(1000);
            }
        }
        else
        {
            rtos_thread_delay(250);
        }
    }

    return CAPTURE_FDC_ERROR(
            "thread should not have terminated", __func__,
            rtos_thread_self());
}


/**
 * Thread that draws the second tile strip
 */
static fdc_error_t
tile_drawing_thread2_f(void *arg)
{
    bool display_tile = true;
    uint_fast16_t x = 0;
    extern volatile uint32_t g_last_trimpot_reading; // XXX

    FDC_ASSERT(arg == NULL, arg, 0);
    
    for ( ; ; )
    {
        if (display_tile)
        {
            (void)rtos_lcd_draw_tile(x, 1*TILE_WIDTH, LCD_COLOR_RED);
        }
        else
        {
            (void)rtos_lcd_draw_tile(x, 1*TILE_WIDTH, LCD_COLOR_BLUE);
        }

        toggle_usb_host_link_led();

        x += TILE_WIDTH;
        if (x == LCD_X_SIZE)
        {
            x = 0;
            display_tile ^= true;
            if (display_tile)
            {
                rtos_thread_delay(500);
            }
            else
            {
                rtos_thread_delay(1000);
            }
        }
        else
        {
            uint32_t delay = g_last_trimpot_reading;
            
            if (delay != 0)
            {
                rtos_thread_delay(delay);
            }
        }
    }

    return CAPTURE_FDC_ERROR(
            "thread should not have terminated", __func__,
            rtos_thread_self());
}


/**
 * Thread that draws the third tile strip
 */
static fdc_error_t
tile_drawing_thread3_f(void *arg)
{
    bool display_tile = true;
    uint_fast16_t x = 0;

    FDC_ASSERT(arg == NULL, arg, 0);
    
    for ( ; ; )
    {
        if (display_tile)
        {
            (void)rtos_lcd_draw_tile(x, 2*TILE_WIDTH, LCD_COLOR_GREEN);
        }
        else
        {
            (void)rtos_lcd_draw_tile(x, 2*TILE_WIDTH, LCD_COLOR_BLUE);
        }

        x += TILE_WIDTH;
        if (x == LCD_X_SIZE)
        {
            x = 0;
            display_tile ^= true;
            if (display_tile)
            {
                rtos_thread_delay(500);
            }
            else
            {
                rtos_thread_delay(1000);
            }
        }
        else
        {
            rtos_thread_delay(250);
        }
    }

    return CAPTURE_FDC_ERROR(
            "thread should not have terminated", __func__,
            rtos_thread_self());
}


/**
 * Producer thread
 */
static fdc_error_t
producer_thread_f(void *p)
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

        rtos_thread_delay(10);
    }

    return CAPTURE_FDC_ERROR(
                "thread should not have terminated", __func__,
                rtos_thread_self());
}


/**
 * Consumer thread
 */
static fdc_error_t
consumer_thread_f(void *p)
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

        rtos_thread_delay(10);
    }

    return CAPTURE_FDC_ERROR(
                "thread should not have terminated", __func__,
                rtos_thread_self());
}


/**
 * Monitor thread
 */
static fdc_error_t
monitor_thread_f(void *p)
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
            }

            s->guard = 0;
        }

        rtos_thread_delay(5);
    }

    return CAPTURE_FDC_ERROR(
                "thread should not have terminated", __func__,
                rtos_thread_self());
}
