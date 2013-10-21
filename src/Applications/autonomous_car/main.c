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
#include "tfc_board.h"

TODO("Remove these pragmas")
//#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

/**
 * Application thread priorities
 */
enum app_thread_priorities
{
    CAMERA_FRAME_READER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY,
    CAMERA_FRAME_NORMALIZER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 1,
    BUTTONS_READER_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
    TRIMPOT_READER_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
};

static void autonomous_car_hardware_init(void);
static void autonomous_car_hardware_stop(void);
static void autonomous_car_app_init(void);
static fdc_error_t camera_frame_reader_thread_f(void *arg);
static fdc_error_t camera_frame_normalizer_thread_f(void *arg);
static fdc_error_t buttons_reader_thread_f(void *arg);
static fdc_error_t trimpot_reader_thread_f(void *arg);

/**
 * Array of application threads for CPU core 0
 */
static const struct rtos_thread_creation_params g_app_threads_cpu0[] =
{
    [0] =
    {
        .p_name_p = "camera frame reader thread",
        .p_function_p = camera_frame_reader_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = CAMERA_FRAME_READER_THREAD_PRIORITY,
        .p_thread_pp = NULL,
    },

    [1] =
    {
        .p_name_p = "camera frame normalizer thread",
        .p_function_p = camera_frame_normalizer_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = CAMERA_FRAME_NORMALIZER_THREAD_PRIORITY,
        .p_thread_pp = NULL,
    },

#if 0
    [2] =
    {
        .p_name_p = "buttons reader thread",
        .p_function_p = buttons_reader_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = BUTTONS_READER_THREAD_PRIORITY,
        .p_thread_pp = NULL,
    },

    [3] =
    {
        .p_name_p = "trimpot reader thread",
        .p_function_p = trimpot_reader_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = TRIMPOT_READER_THREAD_PRIORITY,
        .p_thread_pp = NULL,
    },
#endif
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
        .stc_app_hardware_init_p = autonomous_car_hardware_init,
        .stc_app_hardware_stop_p = autonomous_car_hardware_stop,
        .stc_app_software_init_p = autonomous_car_app_init
    },
};

C_ASSERT(ARRAY_SIZE(g_rtos_app_config) == SOC_NUM_CPU_CORES);

/**
 * Number of camera frame buffers
 */ 
#define NUM_CAMERA_FRAME_BUFFERS   4

/**
 * Array of camera frame buffers
 */
static tfc_camera_raw_pixel_t
    g_camera_frame_buffers[NUM_CAMERA_FRAME_BUFFERS][TFC_NUM_CAMERA_PIXELS];

/**
 * Circular buffer of pointers where pointers to captured camera raw frames 
 * are stored by the camera_frame_reader_thread_f thread, and consumed by
 * the camera_frame_normalizer_thread_f
 */
static struct rtos_circular_buffer g_captured_camera_frames_circular_buffer;

/**
 * Application's main()
 *
 * This function is invoked from the Reset exception handler for each CPU core.
 */ 
int
main(void)
{
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    rtos_startup(
        &g_rtos_app_config[cpu_id]);
    
    FDC_ASSERT(false, 0, 0);

    return -1;
}


static
void autonomous_car_hardware_init(void)
{
    frdm_board_init();
    tfc_board_init();
}


static
void autonomous_car_hardware_stop(void)
{
    frdm_board_stop();
    tfc_board_stop();
}


static
void autonomous_car_app_init(void)
{
    /**
     * Array of entries for g_camera_frames_circular_buffer
     */
    static tfc_camera_raw_pixel_t
        *g_captured_camera_frames_circular_buffer_entries[NUM_CAMERA_FRAME_BUFFERS];

    /**
     * Pointer to mutex to serialize access to g_camera_frames_circular_buffer
     */
    static struct rtos_mutex *g_captured_camera_frames_circular_buffer_mutex_p;

    static const struct rtos_mutex_creation_params mutex_params = {
        .p_name_p = "Captured camera frames circular buffer mutex",
        .p_mutex_pp = &g_captured_camera_frames_circular_buffer_mutex_p
    };

    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    fdc_error = rtos_k_create_mutex(&mutex_params);
    if (fdc_error != 0) {
        fatal_error_handler(fdc_error);
    }

    /*
     * Initialize captured camera frames circular buffer:
     */
    rtos_k_pointer_circular_buffer_init(
        "Captured camera frames",
        NUM_CAMERA_FRAME_BUFFERS,
        (void **)g_captured_camera_frames_circular_buffer_entries,
        g_captured_camera_frames_circular_buffer_mutex_p,
        cpu_id,
        &g_captured_camera_frames_circular_buffer);
}


static void
dump_camera_frame(
    _IN_ tfc_camera_raw_pixel_t camera_frame_raw_pixels[])
{
    for (int i = 0; i < TFC_NUM_CAMERA_PIXELS; i ++) {
        console_printf("%#x ", camera_frame_raw_pixels[i]);
    }

    console_printf("\n");
}


static fdc_error_t
camera_frame_reader_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing camera frame reader thread ...\n");
    
    uint32_t next_frame_index = 0;
    uint32_t dropped_frames_count = 0;

    for ( ; ; ) {
        rtos_thread_delay(TFC_CAMERA_EXPOSURE_TIME_MS);

        tfc_camera_read_frame(
            g_camera_frame_buffers[next_frame_index]);

        bool write_ok =
            rtos_k_pointer_circular_buffer_write(
                &g_captured_camera_frames_circular_buffer,
                g_camera_frame_buffers[next_frame_index],
                false);

        if (write_ok) {
            next_frame_index =
                (next_frame_index + 1) % NUM_CAMERA_FRAME_BUFFERS;
        } else {
            /*
             * The circular buffer is full, so drop the oldest captured frame:
             */
            (void)rtos_k_pointer_circular_buffer_read(
                &g_captured_camera_frames_circular_buffer, NULL, false);

            dropped_frames_count ++;

            DEBUG_PRINTF("Frame was dropped (total frames dropped %u)\n",
                    dropped_frames_count);
        }
    }

    fdc_error =
        CAPTURE_FDC_ERROR(
            "thread should not have terminated",
            cpu_id, rtos_thread_self());

    return fdc_error;
}


static fdc_error_t
camera_frame_normalizer_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    uint32_t frame_count = 0;

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing camera frame normalizer thread ...\n");
    
    for ( ; ; ) {
        tfc_camera_raw_pixel_t *camera_frame_buffer_p;

        (void)rtos_k_pointer_circular_buffer_read(
            &g_captured_camera_frames_circular_buffer,
            (void **)&camera_frame_buffer_p,
            true);

        //???
        console_printf("Frame %u\n", frame_count);
        dump_camera_frame(camera_frame_buffer_p);
        //???
        frame_count ++;
    }

    fdc_error =
        CAPTURE_FDC_ERROR(
            "thread should not have terminated",
            cpu_id, rtos_thread_self());

    return fdc_error;
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
   for ( ; ; ) {
       //???console_printf("%s\n", __func__);
       rtos_thread_delay(2000);
   }
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
   for ( ; ; ) {
       //???console_printf("%s\n", __func__);
       rtos_thread_delay(3000);
   }
#endif

    fdc_error = CAPTURE_FDC_ERROR(
        "McRTOS trimpot sensing thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}

