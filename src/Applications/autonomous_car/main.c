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
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

#define BLACK_SPOT_MIN_WIDTH    8
#define BLACK_SPOT_MAX_WIDTH    24

/**
 * Application thread priorities
 */
enum app_thread_priorities
{
    CAMERA_FRAME_READER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY,
    CAR_DRIVER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 1,
    SWITCHES_AND_BUTTONS_READER_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
    TRIMPOT_READER_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
};

typedef _RANGE_(0, TFC_NUM_CAMERA_PIXELS - 1)
        uint8_t black_spot_position_t;

static void autonomous_car_hardware_init(void);
static void autonomous_car_hardware_stop(void);
static void autonomous_car_app_init(void);
static fdc_error_t camera_frame_reader_thread_f(void *arg);
static fdc_error_t car_driver_thread_f(void *arg);
static fdc_error_t switches_and_buttons_reader_thread_f(void *arg);
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
        .p_name_p = "car driver thread",
        .p_function_p = car_driver_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = CAR_DRIVER_THREAD_PRIORITY,
        .p_thread_pp = NULL,
    },

    [2] =
    {
        .p_name_p = "trimpot reader thread",
        .p_function_p = trimpot_reader_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = TRIMPOT_READER_THREAD_PRIORITY,
        .p_thread_pp = NULL,
    },

    [3] =
    {
        .p_name_p = "switches & buttons reader thread",
        .p_function_p = switches_and_buttons_reader_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = SWITCHES_AND_BUTTONS_READER_THREAD_PRIORITY,
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
        .stc_app_hardware_init_p = autonomous_car_hardware_init,
        .stc_app_hardware_stop_p = autonomous_car_hardware_stop,
        .stc_app_software_init_p = autonomous_car_app_init
    },
};

C_ASSERT(ARRAY_SIZE(g_rtos_app_config) == SOC_NUM_CPU_CORES);

/**
 * Macro to filter a raw pixel reading
 */
#define FILTER_PIXEL_READING(_x) \
        ((_x) >> PIXEL_READING_FILTER_SHIFT)

#define PIXEL_READING_FILTER_SHIFT  (ADC_RESOLUTION - 1)

/**
 * Number of raw camera frame buffers
 */ 
#define NUM_CAMERA_FRAMES    8

/**
 * Array of raw camera frame buffers
 */
static struct tfc_camera_frame g_camera_frames[NUM_CAMERA_FRAMES];

/**
 * Circular buffer of pointers used to keep track of available camera frame 
 * buffers.
 */
static struct rtos_circular_buffer g_camera_frame_buffer_pool;

/**
 * Circular buffer of pointers where pointers to captured camera raw frames 
 * are stored by the camera_frame_reader_thread_f thread, and consumed by
 * the car_driver_thread_f
 */
static struct rtos_circular_buffer g_captured_camera_frames_queue;

/**
 * Count of dropped camera frames
 */
static uint32_t g_dropped_camera_frames = 0;

/**
 * Flag to enable/disable dumping of camera frames to the console. It can
 * be changed through a DIP switch.
 */ 
static volatile bool g_dump_camera_frames_on = true;

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
     * Array of entries for g_camera_frame_buffer_pool
     */
    static struct tfc_camera_frame
        *g_camera_frame_buffer_pool_entries[NUM_CAMERA_FRAMES];

    /**
     * Array of entries for g_captured_camera_frames_queue
     */
    static struct tfc_camera_frame
        *g_captured_camera_frames_queue_entries[NUM_CAMERA_FRAMES];

    /**
     * Pointer to mutex to serialize access to g_camera_frame_buffer_pool
     */
    static struct rtos_mutex *g_camera_frame_buffer_pool_mutex_p;

    /**
     * Pointer to mutex to serialize access to g_camera_frames_queue
     */
    static struct rtos_mutex *g_captured_camera_frames_queue_mutex_p;

    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_mutex_creation_params mutex_params;

   /*
    * Initialize camera frame buffer pool:
    */

    mutex_params.p_name_p = "Camera frame buffer pool mutex";
    mutex_params.p_mutex_pp = &g_camera_frame_buffer_pool_mutex_p;
    fdc_error = rtos_k_create_mutex(&mutex_params);
    if (fdc_error != 0) {
        fatal_error_handler(fdc_error);
    }

    rtos_k_pointer_circular_buffer_init(
        "Raw camera frame buffer pool",
        NUM_CAMERA_FRAMES,
        (void **)g_captured_camera_frames_queue_entries,
        g_camera_frame_buffer_pool_mutex_p,
        cpu_id,
        &g_camera_frame_buffer_pool);

    for (int i = 0; i < NUM_CAMERA_FRAMES; i++) {
        bool write_ok = rtos_k_pointer_circular_buffer_write(
                            &g_camera_frame_buffer_pool,
                            &g_camera_frames[i],
                            false);

        DBG_ASSERT(write_ok, 0, 0);
    }

    /*
     * Initialize captured camera frames circular buffer:
     */

    mutex_params.p_name_p = "Captured camera frames circular buffer mutex";
    mutex_params.p_mutex_pp = &g_captured_camera_frames_queue_mutex_p;
    fdc_error = rtos_k_create_mutex(&mutex_params);
    if (fdc_error != 0) {
        fatal_error_handler(fdc_error);
    }

    rtos_k_pointer_circular_buffer_init(
        "Captured camera frames",
        NUM_CAMERA_FRAMES,
        (void **)g_captured_camera_frames_queue_entries,
        g_captured_camera_frames_queue_mutex_p,
        cpu_id,
        &g_captured_camera_frames_queue);
}


static fdc_error_t
camera_frame_reader_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing camera frame reader thread ...\n");
    
    for ( ; ; ) {
        rtos_thread_delay(TFC_CAMERA_EXPOSURE_TIME_MS);

        /*
         * Get next available buffer from the frame buffer pool:
         */
        bool read_ok;
        struct tfc_camera_frame *frame_buffer_p = NULL;

        do {
            read_ok = rtos_k_pointer_circular_buffer_read(
                        &g_camera_frame_buffer_pool,
                        (void **)&frame_buffer_p,
                        false);

            if (!read_ok) {
                /*
                 * The frame buffer pool is empty, so drop the oldest captured
                 * frame, to use its buffer to capture the next camera frame:
                 */
                read_ok = rtos_k_pointer_circular_buffer_read(
                            &g_captured_camera_frames_queue,
                            (void **)&frame_buffer_p,
                            false);

                if (read_ok) {
                    ATOMIC_POST_INCREMENT_UINT32(&g_dropped_camera_frames);
                }
            }
        } while (!read_ok);

        DBG_ASSERT(frame_buffer_p != NULL, 0, 0);

        /*
         * Capture next camera frame:
         */
        tfc_camera_read_frame(frame_buffer_p);

        bool write_ok =
            rtos_k_pointer_circular_buffer_write(
                &g_captured_camera_frames_queue,
                frame_buffer_p,
                false);

        FDC_ASSERT(write_ok, 0, 0);
    }

    fdc_error =
        CAPTURE_FDC_ERROR(
            "thread should not have terminated",
            cpu_id, rtos_thread_self());

    return fdc_error;
}


static black_spot_position_t
find_black_spot(
    _IN_ struct tfc_camera_frame *camera_frame_p)
{
    black_spot_position_t black_spot_position = 0;

    for (int i = 0; i < TFC_NUM_CAMERA_PIXELS; i ++) {
        if (g_dump_camera_frames_on) {
            console_printf("%x",
                FILTER_PIXEL_READING(camera_frame_p->cf_pixels[i]));
        }

        //XXX
    }

    if (g_dump_camera_frames_on) {
        console_printf("\n");
        console_printf("Dropped frames: %u\n", g_dropped_camera_frames);
    }

    return black_spot_position;
}


static void
drive_car(
    _IN_ black_spot_position_t black_spot_position)
{

}


static fdc_error_t
car_driver_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing car driver thread ...\n");
    
    for ( ; ; ) {
        struct tfc_camera_frame *camera_frame_p;

        /*
         * Get the next camera frame to process, or wait if there is none yet:
         */
        (void)rtos_k_pointer_circular_buffer_read(
            &g_captured_camera_frames_queue,
            (void **)&camera_frame_p,
            true);

        black_spot_position_t black_spot_position =
            find_black_spot(camera_frame_p);

        /*
         * Return frame buffer to the frame buffer pool:
         */
        bool write_ok = rtos_k_pointer_circular_buffer_write(
                            &g_camera_frame_buffer_pool,
                            camera_frame_p,
                            false);

        FDC_ASSERT(write_ok, 0, 0);

        drive_car(black_spot_position);
    }

    fdc_error =
        CAPTURE_FDC_ERROR(
            "thread should not have terminated",
            cpu_id, rtos_thread_self());

    return fdc_error;
}


static tfc_trimpot_reading_t g_last_trimpot_readings[TFC_NUM_TRIMPOTS] = {
    [0] = 0,
    [1] = 0 
};

/**
 * Trimpot reader thread
 */
static fdc_error_t
trimpot_reader_thread_f(void *arg)
{
#   define TRIMPOTS_SAMPLING_PERIOD_MS  250
#   define TRIMPOT_READING_SHIFT        (ADC_RESOLUTION - 5)
#   define FILTER_TRIMPOT_READING(_x) \
            ((_x) >> TRIMPOT_READING_SHIFT)

    static tfc_trimpot_reading_t trimpot_readings[TFC_NUM_TRIMPOTS];

    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing McRTOS trimpot sensing thread ...\n");

    for ( ; ; )
    {
        tfc_trimpots_read(trimpot_readings);
        for (int i = 0; i < TFC_NUM_TRIMPOTS; i ++) {
            tfc_trimpot_reading_t filtered_reading =
                FILTER_TRIMPOT_READING(trimpot_readings[i]);

            tfc_trimpot_reading_t filtered_last_reading =
                FILTER_TRIMPOT_READING(g_last_trimpot_readings[i]);

            if (filtered_reading != filtered_last_reading) {
                console_printf(
                    "Trimpot %u changed: %#x\n", i, filtered_reading);

                g_last_trimpot_readings[i] = trimpot_readings[i];
            }
        }
        
        rtos_thread_delay(TRIMPOTS_SAMPLING_PERIOD_MS);
    }   

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}


static fdc_error_t
switches_and_buttons_reader_thread_f(void *arg)
{
#   define SWITCHES_AND_BUTTONS_READER_THREAD_PERIOD_MS 200
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing switches & buttons reader thread ...\n");

    bool push_buttons[TFC_NUM_PUSH_BUTTONS];
    bool dip_switches[TFC_NUM_DIP_SWITCHES];


    for ( ; ; ) {
        tfc_dip_switches_read(dip_switches);

        //XXX

        tfc_push_buttons_read(push_buttons);

        //XXX

        rtos_thread_delay(SWITCHES_AND_BUTTONS_READER_THREAD_PERIOD_MS);
    } 

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}

