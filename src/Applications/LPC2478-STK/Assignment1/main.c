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
#include "print.h"

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


/**
 * Producer thread
 */
static fdc_error_t
entry1Thread(void *p)
{
    struct globalDataStr *s = (struct globalDataStr *)p;
    s->t1 ++;
    if (s->guard == 0)
    {
        s->guard = 1;
        s->inc ++;
        s->guard = 0;
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

    s->t2 ++;
    if (s->guard==0)
    {
        s->guard = 1;
        s->inc --;
        s->guard = 0;
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

    if (s->guard == 0)
    {
        s->guard = 1;
        if (s->inc > 3)
        {
            console_printf("%s: Error: s.inc > 3: %u\n", __func__, s->inc);
            s->inc = 0;
            return -1;
        }

        s->guard = 0;
    }

    return 0;
}


/**
 * Idle thread
 */
static fdc_error_t
idleThread(void *p)
{
    p=p;
    rtos_k_thread_delay(10 * 1000);
    return (0);
}


#define APP_THREAD_PRIORITY  (RTOS_LOWEST_THREAD_PRIORITY - 1)

/**
 * McRTOS startup configuration
 */ 
static const struct rtos_startup_configuration g_rtos_config =
{
    .stc_idle_thread_hook_function_p = NULL,

    .stc_num_autostart_app_threads = 4,

    .stc_autostart_app_threads = 
    {
        [0] =
        {
            .thr_name_p = "Producer thread",
            .thr_function_p = entry1Thread,
            .thr_function_arg_p = &shared,
            .thr_priority = APP_THREAD_PRIORITY,
            .thr_cpu_id = 0,
        },

        [1] =
        {
            .thr_name_p = "Consumer thread",
            .thr_function_p = entry2Thread,
            .thr_function_arg_p = &shared,
            .thr_priority = APP_THREAD_PRIORITY,
            .thr_cpu_id = 0,
        },

        [2] =
        {
            .thr_name_p = "Monitor thread",
            .thr_function_p = entry3Thread,
            .thr_function_arg_p = &shared,
            .thr_priority = APP_THREAD_PRIORITY,
            .thr_cpu_id = 0,
        },

        [3] =
        {
            .thr_name_p = "Application idle thread",
            .thr_function_p = idleThread,
            .thr_function_arg_p = &shared,
            .thr_priority = APP_THREAD_PRIORITY,
            .thr_cpu_id = 0,
        },
    },
};

/**
 * Application's main()
 *
 * This function is invoked from the Reset exception handler.
 */ 
int
main(void)
{
    rtos_startup(&g_rtos_config);

    FDC_ASSERT(false, 0, 0);

    return -1;
}


