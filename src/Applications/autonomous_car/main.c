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

#define BLACK_SPOT_SET_POINT \
        ((TFC_NUM_CAMERA_PIXELS / 2) - (BLACK_SPOT_AVG_WIDTH / 2))

#define BLACK_SPOT_AVG_WIDTH  \
        ((BLACK_SPOT_MIN_WIDTH + BLACK_SPOT_MAX_WIDTH) / 2)

#define BLACK_SPOT_MIN_WIDTH    16 
#define BLACK_SPOT_MAX_WIDTH    32

/**
 * Application thread priorities
 */
enum app_thread_priorities
{
    CAMERA_FRAME_READER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY,
    CAR_DRIVER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 1,
    TRIMPOT_READER_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
    SWITCHES_AND_BUTTONS_READER_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
    BATTERY_MONITOR_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
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
static fdc_error_t battery_monitor_thread_f(void *arg);
static void toggle_dump_camera_frames(const char *cmd_line);

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

    [4] =
    {
        .p_name_p = "battery monitor thread",
        .p_function_p = battery_monitor_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = BATTERY_MONITOR_THREAD_PRIORITY,
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
        .cmd_description_p = "Toggle on/off dumping filtered camera frames",
        .cmd_function_p = toggle_dump_camera_frames,
    },
};


/**
 * McRTOS application startup configuration
 */ 
static const struct rtos_startup_app_configuration g_rtos_app_config =
{
    .stc_app_hardware_init_p = autonomous_car_hardware_init,
    .stc_app_hardware_stop_p = autonomous_car_hardware_stop,
    .stc_app_software_init_p = autonomous_car_app_init,
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
 * Macro to filter a raw pixel reading
 */
#define FILTER_PIXEL_READING(_x) \
        ((_x) >> PIXEL_READING_FILTER_SHIFT)

#define PIXEL_READING_FILTER_SHIFT  (ADC_RESOLUTION - PIXEL_MEANINGFUL_TOP_BITS)

#define PIXEL_MEANINGFUL_TOP_BITS    4

#define MAX_FILTERED_PIXEL_READING \
        ((UINT32_C(1) << PIXEL_MEANINGFUL_TOP_BITS) - 1)

/**
 * Macro that determines if a pixel is white.
 */
#define IS_PIXEL_WHITE(_filtered_pixel) \
        ((_filtered_pixel) > MAX_FILTERED_PIXEL_READING / 4)

/**
 * Number of raw camera frame buffers
 */ 
#define NUM_CAMERA_FRAMES    8

/**
 * Macro to filter a trimpot reading
 */
#define FILTER_TRIMPOT_READING(_x)  ((_x) >> TRIMPOT_READING_SHIFT)

#define TRIMPOT_READING_SHIFT \
        (ADC_RESOLUTION - TRIMPOT_MEANINGFUL_TOP_BITS)

#define TRIMPOT_MEANINGFUL_TOP_BITS    4

#define MAX_FILTERED_TRIMPOT_READING \
        ((UINT32_C(1) << TRIMPOT_MEANINGFUL_TOP_BITS) - 1)

/**
 * Index in g_last_trimpot_readings[] for the trimpot used to set
 * the initial forward speed for the car's wheels
 */
#define BASE_WHEEL_SPEED_TRIMPOT 0

/**
 * Macro to filter a battery reading
 */
#define FILTER_BATTERY_READING(_x)  ((_x) >> BATTERY_READING_SHIFT)

#define BATTERY_READING_SHIFT \
        (ADC_RESOLUTION - BATTERY_MEANINGFUL_TOP_BITS)

#define BATTERY_MEANINGFUL_TOP_BITS    4

#define MAX_FILTERED_BATTERY_READING \
        ((UINT32_C(1) << BATTERY_MEANINGFUL_TOP_BITS) - 1)

C_ASSERT(
    HOW_MANY(MAX_FILTERED_BATTERY_READING, TFC_NUM_BATTERY_LEDS) == 
    TFC_NUM_BATTERY_LEDS);

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
 * the car_driver_thread_fTRIMPOT_READING_SHIFT
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
static volatile bool g_dump_camera_frames_on = false;

/**
 * Last trimpot readings
 */
static tfc_trimpot_reading_t g_last_trimpot_readings[TFC_NUM_TRIMPOTS] = {
    [0] = 0,
    [1] = 0 
};

#define ASSERT_BREAKPOINT_DIP_SWITCH    0
#define ERROR_BREAKPOINT_DIP_SWITCH     1
#define EXCEPTION_DEBUGGER_DIP_SWITCH   2
#define DRIVE_FAST_DIP_SWITCH           3

static bool g_dip_switches_on[TFC_NUM_DIP_SWITCHES];

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
void autonomous_car_hardware_init(void)
{
    frdm_board_init();
    tfc_board_init();
    DEBUG_PRINTF("Car hardware initialized\n");
}


static
void autonomous_car_hardware_stop(void)
{
    frdm_board_stop();
    tfc_board_stop();
    DEBUG_PRINTF("Car hardware stopped\n");
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

    /*
     * Get initial trimpot reading:
     */ 
    tfc_trimpots_read(g_last_trimpot_readings);

    /*
     * Get initial dip switches settings:
     */
    tfc_dip_switches_read(g_dip_switches_on);

    rtos_set_fdc_params(
        g_dip_switches_on[ASSERT_BREAKPOINT_DIP_SWITCH],
        g_dip_switches_on[ERROR_BREAKPOINT_DIP_SWITCH],
        g_dip_switches_on[EXCEPTION_DEBUGGER_DIP_SWITCH]);

    console_printf(
        "DIP Switches:\n"
        "\tAssert breakpoints %s\n"
        "\tError breakpoints %s\n"
        "\tException debugger %s\n"
        "\tDrive fast %s\n",
        g_dip_switches_on[ASSERT_BREAKPOINT_DIP_SWITCH] ? "ON" : "OFF",
        g_dip_switches_on[ERROR_BREAKPOINT_DIP_SWITCH] ? "ON" : "OFF",
        g_dip_switches_on[EXCEPTION_DEBUGGER_DIP_SWITCH] ? "ON" : "OFF",
        g_dip_switches_on[DRIVE_FAST_DIP_SWITCH] ? "ON" : "OFF");
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
    _IN_ struct tfc_camera_frame *camera_frame_p,
    _OUT_ black_spot_position_t *black_spot_position_p)
{
#   define NUM_BW_CLUSTERS              (sizeof(bw_cluster_bit_map_t) * 8)
#   define BW_CLUSTER_SIZE              (TFC_NUM_CAMERA_PIXELS / NUM_BW_CLUSTERS)
#   define NUM_BW_SUPER_CLUSTERS        (NUM_BW_CLUSTERS / BW_CLUSTER_SIZE)

    typedef uint32_t bw_cluster_bit_map_t;
    
    bool black_spot_found = false;
    bw_cluster_bit_map_t bw_cluster_bit_map = 0x0;
    bw_cluster_bit_map_t bw_super_cluster_bit_map = 0x0;

    /* 
     * Form a bit map of "mostly" black and "mostly" white clusters of pixels:
     *
     * NOTE: Each bit in the bitmap represents a B/W cluster of BW_CLUSTER_SIZE
     * pixels:
     * 0 - "mostly" black cluster
     * 1 - "mostly" white cluster
     */
    natural_t cluster_bit_cursor_mask = BIT(NUM_BW_CLUSTERS - 1);
    natural_t black_count = 0;
    natural_t white_count = 0;
    natural_t black_cluster_count = 0;

    for (natural_t i = 0; i < TFC_NUM_CAMERA_PIXELS; i ++) {
        natural_t filtered_pixel = 
                    FILTER_PIXEL_READING(camera_frame_p->cf_pixels[i]);

        if (g_dump_camera_frames_on) {
            console_printf("%x", filtered_pixel);
        }

        bool is_pixel_white = IS_PIXEL_WHITE(filtered_pixel);

        if (is_pixel_white) {
            white_count ++;
        } else {
            black_count ++;
        }

        if ((i + 1) % BW_CLUSTER_SIZE == 0) {
            DBG_ASSERT(
                cluster_bit_cursor_mask != 0x0, 0, 0);

            if (white_count > black_count) {
                bw_cluster_bit_map |= cluster_bit_cursor_mask;
            } else {
                black_cluster_count ++;
            }

            white_count = 0;
            black_count = 0;
            cluster_bit_cursor_mask >>= 1;
        }
    }

    DBG_ASSERT(black_cluster_count <= NUM_BW_CLUSTERS,
        black_cluster_count, NUM_BW_CLUSTERS);

    if (black_cluster_count == 0) {
        DBG_ASSERT(
            bw_cluster_bit_map == (bw_cluster_bit_map_t)-1,
            bw_cluster_bit_map, 0);

        goto Exit;
    }

    if (black_cluster_count == NUM_BW_CLUSTERS) {
        DBG_ASSERT(bw_cluster_bit_map == 0x0, bw_cluster_bit_map, 0);
        goto Exit;
    }
    
    /* 
     * Form a bit map of "mostly" black and "mostly" white super-clusters of
     * clusters:
     *
     * NOTE: Each bit in the bitmap represents a B/W super-cluster of
     * BW_CLUSTER_SIZE clusters:
     * 0 - "mostly" black super-cluster
     * 1 - "mostly" white super-cluster
     */

    natural_t cluster_cursor = 0;
    natural_t black_super_cluster_count = 0;

    white_count = 0;
    black_count = 0;
    natural_t super_cluster_bit_cursor_mask = BIT(NUM_BW_SUPER_CLUSTERS - 1);
    for (cluster_bit_cursor_mask = BIT(NUM_BW_CLUSTERS - 1);
         cluster_bit_cursor_mask != 0x0;
         cluster_bit_cursor_mask >>= 1) {
        if (bw_cluster_bit_map & cluster_bit_cursor_mask) {
            white_count ++;
        } else {
            black_count ++;
        }

        if ((cluster_cursor + 1) % BW_CLUSTER_SIZE == 0) {
            if (white_count > black_count) {
                bw_super_cluster_bit_map |= super_cluster_bit_cursor_mask;
            } else {
                black_super_cluster_count ++;
            }

            white_count = 0;
            black_count = 0;
            super_cluster_bit_cursor_mask >>= 1;
        }

        cluster_cursor ++;
    }

    DBG_ASSERT(cluster_cursor == NUM_BW_CLUSTERS, 
        cluster_cursor, NUM_BW_CLUSTERS);

    DBG_ASSERT(
        (bw_super_cluster_bit_map & ~(BIT(NUM_BW_SUPER_CLUSTERS) - 1)) == 0x0,
         bw_super_cluster_bit_map, 0);

    DBG_ASSERT(
        black_super_cluster_count <= NUM_BW_SUPER_CLUSTERS,
        black_super_cluster_count, NUM_BW_SUPER_CLUSTERS);

    if (black_super_cluster_count == 0) {
        DBG_ASSERT(
            bw_super_cluster_bit_map == BIT(NUM_BW_SUPER_CLUSTERS) - 1,
            bw_super_cluster_bit_map, BIT(NUM_BW_SUPER_CLUSTERS) - 1);
        goto Exit;
    }

    if (black_super_cluster_count == NUM_BW_SUPER_CLUSTERS) {
        DBG_ASSERT(
            bw_super_cluster_bit_map == 0x0, bw_super_cluster_bit_map, 0);
        goto Exit;
    }
   
    /*
     * Find the super-cluster that is closest to the center of the processed
     * camera frame:
     */
    natural_t super_cluster_index;
#if 1
    for (natural_t i = 0; i < NUM_BW_SUPER_CLUSTERS / 2; i ++) {
        super_cluster_index = (NUM_BW_SUPER_CLUSTERS / 2) + i;
        natural_t bit_mask = 
            BIT((NUM_BW_SUPER_CLUSTERS - 1) - super_cluster_index);
        if ((bw_super_cluster_bit_map & bit_mask) == 0) {
            black_spot_found = true;
            break;
        }

        super_cluster_index = (NUM_BW_SUPER_CLUSTERS / 2) - i - 1;
        bit_mask = BIT((NUM_BW_SUPER_CLUSTERS - 1) - super_cluster_index);
        if ((bw_super_cluster_bit_map & bit_mask) == 0) {
            black_spot_found = true;
            break;
        }
    }
#else
    static natural_t last_super_cluster_index = 0;

    if (last_super_cluster_index < NUM_BW_SUPER_CLUSTERS / 2) {
        for (natural_t i = 0; i < NUM_BW_SUPER_CLUSTERS; i ++) {
            natural_t bit_mask = BIT((NUM_BW_SUPER_CLUSTERS - 1) - i);
            if ((bw_super_cluster_bit_map & bit_mask) == 0) {
                black_spot_found = true;
                super_cluster_index = i;
                break;
            }
        }
    } else {
        for (int i = NUM_BW_SUPER_CLUSTERS - 1; i >= 0; i --) {
            natural_t bit_mask = BIT((NUM_BW_SUPER_CLUSTERS - 1) - i);
            if ((bw_super_cluster_bit_map & bit_mask) == 0) {
                black_spot_found = true;
                super_cluster_index = i;
                break;
            }
        }
    }

    last_super_cluster_index = super_cluster_index;
#endif

    if (black_spot_found) {
        DBG_ASSERT(
            super_cluster_index < NUM_BW_SUPER_CLUSTERS,
            super_cluster_index, NUM_BW_SUPER_CLUSTERS);

        *black_spot_position_p =
            super_cluster_index * (BW_CLUSTER_SIZE * BW_CLUSTER_SIZE);
    }
 
Exit:
    if (g_dump_camera_frames_on) {
        console_printf("\n");
        console_printf(
            "Black spot position: %d, b/w cluster bitmap: %#x, "
            "b/w super cluster bitmap: %#x, dropped frames: %u\n",
                black_spot_found ? *black_spot_position_p : -1,
                bw_cluster_bit_map, bw_super_cluster_bit_map,
                g_dropped_camera_frames);
    }

    return black_spot_found;   
}


static void
toggle_dump_camera_frames(const char *cmd_line)
{
    g_dump_camera_frames_on ^= true;
}


static int
calculate_new_manipulated_var_value(
    int base_manipulated_var, 
    int offset_manipulated_var,
    int min_value,
    int max_value)
{
    int manipulated_var = base_manipulated_var + offset_manipulated_var;

    if (manipulated_var > max_value) {
        manipulated_var = max_value;
    } else if (manipulated_var < min_value) {
        manipulated_var = min_value;
    }

    return manipulated_var;
}


/**
 * Calculate base forward speed of wheels
 */
static pwm_duty_cycle_us_t
calculate_base_wheel_motor_pwm_duty_cycle_us(void)
{
    natural_t base_wheel_speed_delta =
        FILTER_TRIMPOT_READING(
            g_last_trimpot_readings[BASE_WHEEL_SPEED_TRIMPOT]) *
        UINT_DIV_APPROX(
            TFC_WHEEL_MOTOR_MAX_SPEED_DELTA, MAX_FILTERED_TRIMPOT_READING);

    pwm_duty_cycle_us_t base_wheel_motor_pwm_duty_cycle_us =
        TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US + base_wheel_speed_delta;

    if (base_wheel_motor_pwm_duty_cycle_us > TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US) {
        base_wheel_motor_pwm_duty_cycle_us = TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US;
    }

    return base_wheel_motor_pwm_duty_cycle_us;
}


/**
 * PID controller to drive the car
 */
static void
drive_car(
    _IN_ black_spot_position_t black_spot_position,
    _IN_ pwm_duty_cycle_us_t base_wheel_motor_pwm_duty_cycle_us)
{
#   define STEERING_SERVO_PROPORTIONAL_GAIN \
        UINT_DIV_APPROX(                                                \
            TFC_STEERING_SERVO_MAX_DUTY_CYCLE_US -                      \
                TFC_STEERING_SERVO_MIN_DUTY_CYCLE_US + 1,               \
            TFC_NUM_CAMERA_PIXELS)

#   define STEERING_SERVO_INTEGRAL_GAIN \
        UINT_DIV_APPROX(STEERING_SERVO_PROPORTIONAL_GAIN, 50)

#   define STEERING_SERVO_DERIVATIVE_GAIN \
        UINT_DIV_APPROX(STEERING_SERVO_PROPORTIONAL_GAIN, 200)

#   define WHEEL_MOTOR_PROPORTIONAL_GAIN \
        UINT_DIV_APPROX(                                                \
            TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US -                         \
                TFC_WHEEL_MOTOR_MIN_DUTY_CYCLE_US + 1,                  \
            TFC_NUM_CAMERA_PIXELS)

#   define WHEEL_MOTOR_INTEGRAL_GAIN        0
#   define WHEEL_MOTOR_DERIVATIVE_GAIN      0

#   define PID_ERROR_THRESHOLD_IN_PIXELS    (TFC_NUM_CAMERA_PIXELS / 16)

    static int previous_errors[2] = {0, 0};
    static pwm_duty_cycle_us_t steering_servo_pwm_duty_cycle_us =
                                    TFC_STEERING_SERVO_STRAIGHT_DUTY_CYCLE_US;

    static pwm_duty_cycle_us_t wheel_motor_pwm_duty_cycle_us =
                                    TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US;

    /*
     * Calculate PID values:
     */
    int error = (int)BLACK_SPOT_SET_POINT - (int)black_spot_position;
    int delta_error = error - previous_errors[0];
    int derivative_term =
        (previous_errors[1] * previous_errors[0]) + previous_errors[1];

    previous_errors[1] = previous_errors[0];
    previous_errors[0] = error;

    /*
     * Calculate new steering servo PWM duty cycle:
     */

    int offset_steering_servo_pwm_duty_cycle = 
             STEERING_SERVO_PROPORTIONAL_GAIN * delta_error + 
             STEERING_SERVO_INTEGRAL_GAIN * error +
             STEERING_SERVO_DERIVATIVE_GAIN * derivative_term;

    steering_servo_pwm_duty_cycle_us =
        (pwm_duty_cycle_us_t)calculate_new_manipulated_var_value(
                                steering_servo_pwm_duty_cycle_us,
                                offset_steering_servo_pwm_duty_cycle,
                                TFC_STEERING_SERVO_MIN_DUTY_CYCLE_US,
                                TFC_STEERING_SERVO_MAX_DUTY_CYCLE_US);

    /*
     * Calculate new wheel motors PWM duty cycle:
     *
     * Heuristics:
     * - The smaller ABS(error) the faster you can go. The larger ABS(error)
     *   the slower you should go.
     * - For ABS(error) close to zero, you can go to the maximum forward speed.
     * - For ABS(error) >= PID_ERROR_THRESHOLD_IN_PIXELS (i.e. sharp turns), you should
     *   push the breaks by making the wheels turn backwards and/or make the inner
     *   wheel go slower than the outer wheel.
     */

    int offset_wheel_motor_pwm_duty_cycle;

    if (g_dip_switches_on[DRIVE_FAST_DIP_SWITCH]) {
        if (wheel_motor_pwm_duty_cycle_us ==
                TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US) {
            wheel_motor_pwm_duty_cycle_us = base_wheel_motor_pwm_duty_cycle_us;
        }
        
        if (ABS(error) <= PID_ERROR_THRESHOLD_IN_PIXELS) {
            offset_wheel_motor_pwm_duty_cycle = 
                WHEEL_MOTOR_PROPORTIONAL_GAIN * ABS(error);
        } else {
            offset_wheel_motor_pwm_duty_cycle = 
                -WHEEL_MOTOR_PROPORTIONAL_GAIN * ABS(error);
        }
    } else {
        wheel_motor_pwm_duty_cycle_us = base_wheel_motor_pwm_duty_cycle_us;
        offset_wheel_motor_pwm_duty_cycle = 0;
    }

    wheel_motor_pwm_duty_cycle_us =
        (pwm_duty_cycle_us_t)calculate_new_manipulated_var_value(
                                wheel_motor_pwm_duty_cycle_us,
                                offset_wheel_motor_pwm_duty_cycle,
                                TFC_WHEEL_MOTOR_MIN_DUTY_CYCLE_US,
                                TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US);

    pwm_duty_cycle_us_t left_wheel_pwm_duty_cycle_us =
        wheel_motor_pwm_duty_cycle_us;

    pwm_duty_cycle_us_t right_wheel_pwm_duty_cycle_us =
        wheel_motor_pwm_duty_cycle_us;

    if (ABS(error) >= PID_ERROR_THRESHOLD_IN_PIXELS &&
        wheel_motor_pwm_duty_cycle_us != TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US) {
        if (error < 0) {
            /*
             * Help steer left by making left wheel spin slower
             */
            left_wheel_pwm_duty_cycle_us -= 
                wheel_motor_pwm_duty_cycle_us / 4;
        } else {
            /*
             * Help steer right by making right wheel spin slower
             */
            right_wheel_pwm_duty_cycle_us -=
                wheel_motor_pwm_duty_cycle_us / 4;
        }
    }
        
    /*
     * Send commands to actuators:
     */

    tfc_steering_servo_set(
        steering_servo_pwm_duty_cycle_us);

    tfc_wheel_motors_set(
        left_wheel_pwm_duty_cycle_us,
        right_wheel_pwm_duty_cycle_us);
}


static fdc_error_t
car_driver_thread_f(void *arg)
{
#   define MAX_NO_BLACK_SPOT_FOUND_COUNT   16 
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    uint32_t no_black_spot_found_count = 0;
    bool wheels_stopped = true;

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing car driver thread ...\n");

    tfc_steering_servo_set(
        TFC_STEERING_SERVO_STRAIGHT_DUTY_CYCLE_US);

    black_spot_position_t black_spot_position = BLACK_SPOT_SET_POINT;

    for ( ; ; ) {
        struct tfc_camera_frame *camera_frame_p;

        pwm_duty_cycle_us_t base_wheel_motor_pwm_duty_cycle_us =
            calculate_base_wheel_motor_pwm_duty_cycle_us();

        /*
         * Get the next camera frame to process, or wait if there is none yet:
         */
        (void)rtos_k_pointer_circular_buffer_read(
            &g_captured_camera_frames_queue,
            (void **)&camera_frame_p,
            true);

        bool black_spot_found =
            find_black_spot(camera_frame_p, &black_spot_position);

        /*
         * Return frame buffer to the frame buffer pool:
         */
        bool write_ok = rtos_k_pointer_circular_buffer_write(
                            &g_camera_frame_buffer_pool,
                            camera_frame_p,
                            false);

        FDC_ASSERT(write_ok, 0, 0);

        if (black_spot_found) {
            if (wheels_stopped) {
                tfc_wheel_motors_set(
                    base_wheel_motor_pwm_duty_cycle_us,
                    base_wheel_motor_pwm_duty_cycle_us);

                wheels_stopped = false;
            }

            drive_car(
                black_spot_position, base_wheel_motor_pwm_duty_cycle_us);
        } else {
            (void)CAPTURE_FDC_ERROR("Black spot not found", 0, 0);
            no_black_spot_found_count ++;
            if (no_black_spot_found_count == MAX_NO_BLACK_SPOT_FOUND_COUNT) {
                no_black_spot_found_count = 0;
                tfc_wheel_motors_set(
                    TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US,
                    TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US);

                wheels_stopped = true;
            }
        }
    }

    fdc_error =
        CAPTURE_FDC_ERROR(
            "thread should not have terminated",
            cpu_id, rtos_thread_self());

    return fdc_error;
}


/**
 * Trimpot reader thread
 */
static fdc_error_t
trimpot_reader_thread_f(void *arg)
{
#   define TRIMPOTS_SAMPLING_PERIOD_MS  250
    static tfc_trimpot_reading_t trimpot_readings[TFC_NUM_TRIMPOTS];

    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing trimpot sensing thread ...\n");

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
    bool fdc_setting_changed;

    for ( ; ; ) {
        /*
         * Read DIP switches:
         */ 
        tfc_dip_switches_read(dip_switches);

        fdc_setting_changed = false;
        for (natural_t i = 0; i < TFC_NUM_DIP_SWITCHES; i++) {
            if (dip_switches[i] != g_dip_switches_on[i]) {
                g_dip_switches_on[i] = dip_switches[i];
                switch (i) {
                case ASSERT_BREAKPOINT_DIP_SWITCH:
                    fdc_setting_changed = true;
                    console_printf(
                        "Assert breakpoints changed to %s\n",
                        dip_switches[i] ? "ON" : "OFF");
                    break;

                case ERROR_BREAKPOINT_DIP_SWITCH:
                    fdc_setting_changed = true;
                    console_printf(
                        "Error breakpoints changed to %s\n",
                        dip_switches[i] ? "ON" : "OFF");
                    break;

                case EXCEPTION_DEBUGGER_DIP_SWITCH:
                    fdc_setting_changed = true;
                    console_printf(
                        "Exception debugger changed to %s\n",
                        dip_switches[i] ? "ON" : "OFF");
                    break;

                case DRIVE_FAST_DIP_SWITCH:
                    console_printf(
                        "Drive fast changed to %s\n",
                         dip_switches[i] ? "ON" : "OFF");
                    break;
                }
            }
        }

        if (fdc_setting_changed) {
            rtos_set_fdc_params(
                dip_switches[ASSERT_BREAKPOINT_DIP_SWITCH],
                dip_switches[ERROR_BREAKPOINT_DIP_SWITCH],
                dip_switches[EXCEPTION_DEBUGGER_DIP_SWITCH]);
        }

        /*
         * Read push buttons:
         */ 
        tfc_push_buttons_read(push_buttons);

        //XXX

        rtos_thread_delay(SWITCHES_AND_BUTTONS_READER_THREAD_PERIOD_MS);
    } 

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}


/**
 * Battery monitor thread
 */
static fdc_error_t
battery_monitor_thread_f(void *arg)
{
#   define BATTERY_SAMPLING_PERIOD_MS  1500
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    static natural_t last_battery_reading = 0;

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing battery monitor thread ...\n");

    for ( ; ; )
    {
        tfc_battery_reading_t battery_reading = tfc_battery_sensor_read();
      
        if (battery_reading != last_battery_reading) {
            natural_t battery_level = 
                HOW_MANY(
                    FILTER_BATTERY_READING(battery_reading),
                    2);

            if (battery_level > TFC_NUM_BATTERY_LEDS) {
                battery_level = TFC_NUM_BATTERY_LEDS;
            }

            tfc_battery_leds_set(battery_level);

            last_battery_reading = battery_reading;
        }

        rtos_thread_delay(BATTERY_SAMPLING_PERIOD_MS);
    }   

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}
