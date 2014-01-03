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

#define CENTER_PIXEL_INDEX      ((TFC_NUM_CAMERA_PIXELS - 1) / 2)

#define CENTER_BLACK_SPOT_WIDTH 8

#define GET_BLACK_SPOT_CENTER(_black_spot_p) \
        ((_black_spot_p)->bs_position + (_black_spot_p)->bs_size / 2)

#define NUM_BW_CLUSTERS (sizeof(bw_cluster_bit_map_t) * 8)
#define BW_CLUSTER_SIZE (TFC_NUM_CAMERA_PIXELS / NUM_BW_CLUSTERS)

typedef uint32_t bw_cluster_bit_map_t;

/**
 * Application thread priorities
 */
enum app_thread_priorities
{
    CAMERA_FRAME_READER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY,
    ACCELEROMETER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 1,
    CAR_DRIVER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 2,
    TRIMPOT_READER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 1,
    SWITCHES_AND_BUTTONS_READER_THREAD_PRIORITY = RTOS_HIGHEST_THREAD_PRIORITY + 1,
    BATTERY_MONITOR_THREAD_PRIORITY = RTOS_LOWEST_THREAD_PRIORITY - 1,
};

typedef _RANGE_(0, TFC_NUM_CAMERA_PIXELS - 1)
        uint8_t black_spot_position_t;

struct black_spot {
    /**
     * Indicates if the black spot was detected
     */
    bool bs_detected;

    /**
     * Index of the right-most pixel of the black spot
     */
    black_spot_position_t bs_position;

    /**
     * Size of the black spot in pixels
     */
    uint8_t bs_size;
};

static void autonomous_car_hardware_init(void);
static void autonomous_car_hardware_stop(void);
static void autonomous_car_app_init(void);
static fdc_error_t camera_frame_reader_thread_f(void *arg);
static fdc_error_t car_driver_thread_f(void *arg);
static fdc_error_t switches_and_buttons_reader_thread_f(void *arg);
static fdc_error_t trimpot_reader_thread_f(void *arg);
static fdc_error_t battery_monitor_thread_f(void *arg);
static fdc_error_t accelerometer_thread_f(void *arg);
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

    [5] =
    {
        .p_name_p = "accelerometer thread",
        .p_function_p = accelerometer_thread_f,
        .p_function_arg_p = NULL,
        .p_priority = ACCELEROMETER_THREAD_PRIORITY,
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
#define IS_PIXEL_WHITE(_filtered_pixel, _white_threshold) \
        ((_filtered_pixel) >= (_white_threshold))

/**
 * Number of raw camera frame buffers
 */ 
#define NUM_CAMERA_FRAMES    2

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
 * Trimpots assignments
 */
#define BASE_WHEEL_SPEED_TRIMPOT    0
#define WHITE_THRESHOLD_TRIMPOT     1

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

#define ASSERT_BREAKPOINT_DIP_SWITCH    0
#define EXCEPTION_DEBUGGER_DIP_SWITCH   1
#define ACTIVE_DIFFERENTIAL_DIP_SWITCH  2
#define DRIVE_FAST_DIP_SWITCH           3

#define TURN_ON_CAR_PUSH_BUTTON     0
#define TURN_OFF_CAR_PUSH_BUTTON    1 

#define THROTTLE_INCREMENT_UNIT \
        UINT_DIV_APPROX(                                                \
            TFC_WHEEL_MOTOR_MAX_THROTTLE, MAX_FILTERED_TRIMPOT_READING)

/*
 * Threshold of wheel motor throttle after which we need to slow down on turns
 */
#define SLOW_DOWN_ON_TURN_THREASHOLD \
        (TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US + THROTTLE_INCREMENT_UNIT * 8)

/**
 * State variables of the autonomous car
 */
struct autonomous_car {
    /**
     * Flag to enable/disable driving the car
     */ 
    volatile bool c_turned_on;

    /**
     * Pointer to mutex to serialize access to c_turned_on
     */
    struct rtos_mutex *c_turned_on_mutex_p;

    /**
     * Pointer to condvar to signal threads to be activated when the 
     * car is turned on.
     */
    struct rtos_condvar *c_turned_on_condvar_p;

    /**
     * Array of entries for g_camera_frame_buffer_pool
     */
    struct tfc_camera_frame
        *c_camera_frame_buffer_pool_entries[NUM_CAMERA_FRAMES];

    /**
     * Array of entries for g_captured_camera_frames_queue
     */
    struct tfc_camera_frame
        *c_captured_camera_frames_queue_entries[NUM_CAMERA_FRAMES];

    /**
     * Pointer to mutex to serialize access to g_camera_frame_buffer_pool
     */
    struct rtos_mutex *c_camera_frame_buffer_pool_mutex_p;

    /**
     * Pointer to mutex to serialize access to g_camera_frames_queue
     */
    struct rtos_mutex *c_captured_camera_frames_queue_mutex_p;

    /**
     * Array of raw camera frame buffers
     */
    struct tfc_camera_frame c_camera_frames[NUM_CAMERA_FRAMES];

    /**
     * Circular buffer of pointers used to keep track of available camera frame 
     * buffers.
     */
    struct rtos_circular_buffer c_camera_frame_buffer_pool;

    /**
     * Circular buffer of pointers where pointers to captured camera raw frames 
     * are stored by the camera_frame_reader_thread_f thread, and consumed by
     * the car_driver_thread_f thread
     */
    struct rtos_circular_buffer c_captured_camera_frames_queue;

    /**
     * Flag to enable/disable dumping of camera frames to the console. It can
     * be changed through a DIP switch.
     */ 
    volatile bool c_dump_camera_frames_on;

    /**
     * Latest X-axis acceleration reading
     */
    volatile int16_t c_x_acceleration;

    /**
     * Latest Y-axis acceleration reading
     */
    volatile int16_t c_y_acceleration;

    /**
     * Latest Z-axis acceleration reading
     */
    volatile int16_t c_z_acceleration;

    /**
     * Last trimpot readings
     */
    volatile tfc_trimpot_reading_t c_last_trimpot_readings[TFC_NUM_TRIMPOTS];

    /**
     * DIP switches state
     */
    volatile bool c_dip_switches_on[TFC_NUM_DIP_SWITCHES];

    /**
     * Push buttons state
     */
    volatile bool c_push_buttons[TFC_NUM_PUSH_BUTTONS];
};

static struct autonomous_car g_car = {
    .c_turned_on = false,
    .c_turned_on_mutex_p = NULL,
    .c_turned_on_condvar_p = NULL,
    .c_camera_frame_buffer_pool_mutex_p = NULL,
    .c_captured_camera_frames_queue_mutex_p = NULL,
    .c_dump_camera_frames_on = false,
    .c_x_acceleration = 0,
    .c_y_acceleration = 0,
    .c_z_acceleration = 0,
    .c_last_trimpot_readings = {
        [0] = 0,
        [1] = 0,
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
    static const char g_app_version[] = "Autonomous car v0.1";
    static const char g_app_build_timestamp[] = "built " __DATE__ " " __TIME__;
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_mutex_creation_params mutex_params;
    struct rtos_condvar_creation_params condvar_params;

   /*
    * Initialize camera frame buffer pool:
    */

    mutex_params.p_name_p = "Camera frame buffer pool mutex";
    mutex_params.p_mutex_pp = &g_car.c_camera_frame_buffer_pool_mutex_p;
    fdc_error = rtos_k_create_mutex(&mutex_params);
    if (fdc_error != 0) {
        fatal_error_handler(fdc_error);
    }

    rtos_k_pointer_circular_buffer_init(
        "Raw camera frame buffer pool",
        NUM_CAMERA_FRAMES,
        (void **)g_car.c_captured_camera_frames_queue_entries,
        g_car.c_camera_frame_buffer_pool_mutex_p,
        cpu_id,
        &g_car.c_camera_frame_buffer_pool);

    for (int i = 0; i < NUM_CAMERA_FRAMES; i++) {
        bool write_ok = rtos_k_pointer_circular_buffer_write(
                            &g_car.c_camera_frame_buffer_pool,
                            &g_car.c_camera_frames[i],
                            false);

        DBG_ASSERT(write_ok, 0, 0);
    }

    /*
     * Initialize captured camera frames circular buffer:
     */

    mutex_params.p_name_p = "Captured camera frames circular buffer mutex";
    mutex_params.p_mutex_pp = &g_car.c_captured_camera_frames_queue_mutex_p;
    fdc_error = rtos_k_create_mutex(&mutex_params);
    if (fdc_error != 0) {
        fatal_error_handler(fdc_error);
    }

    rtos_k_pointer_circular_buffer_init(
        "Captured camera frames",
        NUM_CAMERA_FRAMES,
        (void **)g_car.c_captured_camera_frames_queue_entries,
        g_car.c_captured_camera_frames_queue_mutex_p,
        cpu_id,
        &g_car.c_captured_camera_frames_queue);

    mutex_params.p_name_p = "c_turned_on mutex";
    mutex_params.p_mutex_pp = &g_car.c_turned_on_mutex_p;
    fdc_error = rtos_k_create_mutex(&mutex_params);
    if (fdc_error != 0) {
        fatal_error_handler(fdc_error);
    }

    mutex_params.p_name_p = "c_turned_on mutex";
    mutex_params.p_mutex_pp = &g_car.c_turned_on_mutex_p;
    fdc_error = rtos_k_create_mutex(&mutex_params);
    if (fdc_error != 0) {
        fatal_error_handler(fdc_error);
    }

    condvar_params.p_name_p = "c_turned_on condvar";
    condvar_params.p_condvar_pp = &g_car.c_turned_on_condvar_p;
    fdc_error = rtos_k_create_condvar(&condvar_params);
    if (fdc_error != 0) {
        fatal_error_handler(fdc_error);
    }

    /*
     * Get initial trimpot reading:
     */ 
    tfc_trimpots_read((tfc_trimpot_reading_t *)g_car.c_last_trimpot_readings);

    /*
     * Get initial dip switches settings:
     */
    tfc_dip_switches_read((bool *)g_car.c_dip_switches_on);

    rtos_set_fdc_params(
        g_car.c_dip_switches_on[ASSERT_BREAKPOINT_DIP_SWITCH],
        g_car.c_dip_switches_on[EXCEPTION_DEBUGGER_DIP_SWITCH]);

    console_printf(
        "%s\n%s\n"
        "DIP Switches:\n"
        "\tAssert breakpoints %s\n"
        "\tException debugger %s\n"
        "\tActive differential %s\n"
        "\tDrive fast %s\n",
        g_app_version, g_app_build_timestamp,
        g_car.c_dip_switches_on[ASSERT_BREAKPOINT_DIP_SWITCH] ? "ON" : "OFF",
        g_car.c_dip_switches_on[EXCEPTION_DEBUGGER_DIP_SWITCH] ? "ON" : "OFF",
        g_car.c_dip_switches_on[ACTIVE_DIFFERENTIAL_DIP_SWITCH] ? "ON" : "OFF",
        g_car.c_dip_switches_on[DRIVE_FAST_DIP_SWITCH] ? "ON" : "OFF");
}


static void
set_wheels_straight(void) 
{
    tfc_steering_servo_set(
        TFC_STEERING_SERVO_NEUTRAL_DUTY_CYCLE_US);
}


/**
 * Calculate base forward speed of wheels
 */
static pwm_duty_cycle_us_t
calculate_base_wheel_motor_pwm_duty_cycle_us(void)
{
    natural_t base_wheel_speed_delta =
        FILTER_TRIMPOT_READING(
            g_car.c_last_trimpot_readings[BASE_WHEEL_SPEED_TRIMPOT]) *
        THROTTLE_INCREMENT_UNIT;

    pwm_duty_cycle_us_t base_wheel_motor_pwm_duty_cycle_us =
        TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US + base_wheel_speed_delta;

    if (base_wheel_motor_pwm_duty_cycle_us > TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US) {
        base_wheel_motor_pwm_duty_cycle_us = TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US;
    }

    return base_wheel_motor_pwm_duty_cycle_us;
}


static fdc_error_t
camera_frame_reader_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg == NULL, arg, cpu_id);

    console_printf("Initializing camera frame reader thread ...\n");
    
    for ( ; ; ) {
        rtos_mutex_acquire(g_car.c_turned_on_mutex_p);
        if (!g_car.c_turned_on) {
            do {
                rtos_condvar_wait(
                    g_car.c_turned_on_condvar_p,
                    g_car.c_turned_on_mutex_p);
            } while (!g_car.c_turned_on);

            /*
             * Discard first frame, as it may be garbage:
             */
            tfc_camera_read_frame(NULL);
        }
        rtos_mutex_release(g_car.c_turned_on_mutex_p);

        /*
         * Get next available buffer from the frame buffer pool:
         */
        struct tfc_camera_frame *frame_buffer_p = NULL;
        bool read_ok = rtos_k_pointer_circular_buffer_read(
                            &g_car.c_camera_frame_buffer_pool,
                            (void **)&frame_buffer_p,
                            false);

        if (!read_ok) {
            /*
             * The frame buffer pool is empty, so drop the oldest captured
             * frame, to use its buffer to capture the next camera frame:
             */
            read_ok = rtos_k_pointer_circular_buffer_read(
                        &g_car.c_captured_camera_frames_queue,
                        (void **)&frame_buffer_p,
                        false);

            FDC_ASSERT(read_ok, 0, 0);
        }

        DBG_ASSERT(frame_buffer_p != NULL, 0, 0);

        /*
         * Capture next camera frame:
         */
        tfc_camera_read_frame(frame_buffer_p);

        bool write_ok =
            rtos_k_pointer_circular_buffer_write(
                &g_car.c_captured_camera_frames_queue,
                frame_buffer_p,
                false);

        FDC_ASSERT(write_ok, 0, 0);

        rtos_thread_delay(TFC_CAMERA_SAMPLING_DELAY_MS);
    }

    fdc_error =
        CAPTURE_FDC_ERROR(
            "thread should not have terminated",
            cpu_id, rtos_thread_self());

    return fdc_error;
}

   
/**
 * Find the middle black spot in a camera frame. 
 * Pixel 0 is the right-most pixel
 * Pixel 'TFC_NUM_CAMERA_PIXELS - 1' is the left-most pixel.
 *
 * @return true if success, false otherwise
 */
static bool
find_middle_black_spot(
    _IN_ struct tfc_camera_frame *camera_frame_p,
    _IN_ tfc_trimpot_reading_t white_threshold,
    _OUT_ black_spot_position_t *middle_black_spot_pos_p)
{
    natural_t min_pixel_index = TFC_NUM_CAMERA_PIXELS;
    natural_t min_filtered_pixel = MAX_FILTERED_PIXEL_READING;
    bool black_spot_found = false;
    bw_cluster_bit_map_t bw_cluster_bit_map = 0x0;

    /* 
     * Form a bit map of "mostly" black and "mostly" white clusters of pixels:
     *
     * NOTE: Each bit in the bitmap represents a B/W cluster of BW_CLUSTER_SIZE
     * pixels:
     * 0 - "mostly" black cluster
     * 1 - "mostly" white cluster
     */
    bw_cluster_bit_map_t cluster_bit_cursor_mask = BIT(0);
    natural_t black_count = 0;
    natural_t white_count = 0;
    natural_t black_cluster_count = 0;

    for (natural_t i = 0; i < TFC_NUM_CAMERA_PIXELS; i ++) {
        natural_t filtered_pixel = 
                    FILTER_PIXEL_READING(camera_frame_p->cf_pixels[i]);

        if (g_car.c_dump_camera_frames_on) {
            console_printf("%x", filtered_pixel);
        }

        bool is_pixel_white = IS_PIXEL_WHITE(filtered_pixel, white_threshold);

        if (is_pixel_white) {
            white_count ++;
        } else {
            black_count ++;
        }

        if ((i + 1) % BW_CLUSTER_SIZE == 0) {
            DBG_ASSERT(
                cluster_bit_cursor_mask != 0x0 &&
                cluster_bit_cursor_mask <= BIT(NUM_BW_CLUSTERS - 1),
                cluster_bit_cursor_mask, 0);

            if (white_count > black_count) {
                bw_cluster_bit_map |= cluster_bit_cursor_mask;
            } else {
                black_cluster_count ++;
            }

            white_count = 0;
            black_count = 0;
            cluster_bit_cursor_mask <<= 1;
        }
    }

    if (g_car.c_dump_camera_frames_on) {
        console_printf("\n");
    }

    DBG_ASSERT(black_cluster_count <= NUM_BW_CLUSTERS,
        black_cluster_count, NUM_BW_CLUSTERS);

    /*
     * Find black cluster closest to the center of the camera frame:
     */
    if (black_cluster_count != 0) {
        natural_t bw_cluster_index = ((NUM_BW_CLUSTERS / 2) - 1);
        cluster_bit_cursor_mask = BIT(bw_cluster_index);
        if ((bw_cluster_bit_map & cluster_bit_cursor_mask) == 0) {
            *middle_black_spot_pos_p =
                (bw_cluster_index * BW_CLUSTER_SIZE) + (BW_CLUSTER_SIZE / 2);
            return true;
        }

        for (natural_t i = 1; i < NUM_BW_CLUSTERS / 2; i ++) {
            bw_cluster_index = ((NUM_BW_CLUSTERS / 2) - 1) - i;
            cluster_bit_cursor_mask = BIT(bw_cluster_index);
            if ((bw_cluster_bit_map & cluster_bit_cursor_mask) == 0) {
                *middle_black_spot_pos_p =
                    (bw_cluster_index * BW_CLUSTER_SIZE) + (BW_CLUSTER_SIZE / 2);
                return true;
            }

            bw_cluster_index = ((NUM_BW_CLUSTERS / 2) - 1) + i;
            cluster_bit_cursor_mask = BIT(bw_cluster_index);
            if ((bw_cluster_bit_map & cluster_bit_cursor_mask) == 0) {
                *middle_black_spot_pos_p =
                    (bw_cluster_index * BW_CLUSTER_SIZE) + (BW_CLUSTER_SIZE / 2);
                return true;
            }
        }

        DBG_ASSERT(false, 0, 0);
    }

    return false;
}


static void
toggle_dump_camera_frames(const char *cmd_line)
{
    g_car.c_dump_camera_frames_on ^= true;
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
 * PID controller to drive the car
 */
static void
steer_wheels(
    _IN_ black_spot_position_t black_spot_position,
    _IN_ pwm_duty_cycle_us_t base_wheel_motor_pwm_duty_cycle_us,
    _INOUT_ pwm_duty_cycle_us_t *current_steering_servo_pwm_duty_cycle_us_p)
{
#   define STEERING_SERVO_PROPORTIONAL_GAIN \
        UINT_DIV_APPROX(                                                \
            TFC_STEERING_SERVO_MAX_DUTY_CYCLE_US -                      \
                TFC_STEERING_SERVO_MIN_DUTY_CYCLE_US + 1,               \
            TFC_NUM_CAMERA_PIXELS)

#   define STEERING_SERVO_INTEGRAL_GAIN \
          UINT_DIV_APPROX(STEERING_SERVO_PROPORTIONAL_GAIN, 5000)

#   define STEERING_SERVO_DERIVATIVE_GAIN  0
           //(STEERING_SERVO_PROPORTIONAL_GAIN / 32)

#   define WHEEL_MOTOR_PROPORTIONAL_GAIN \
        UINT_DIV_APPROX(                                                \
            TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US -                         \
                TFC_WHEEL_MOTOR_MIN_DUTY_CYCLE_US + 1,                  \
            TFC_NUM_CAMERA_PIXELS)

#   define WHEEL_DIFFERENTIAL_PROPORTIONAL_GAIN \
        (WHEEL_MOTOR_PROPORTIONAL_GAIN / 4)

#   define PID_ERROR_THRESHOLD_IN_PIXELS    (TFC_NUM_CAMERA_PIXELS / 16)

    static int previous_error = 0;
    static int integral_term = 0;
    static pwm_duty_cycle_us_t wheel_motor_pwm_duty_cycle_us =
                                    TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US;

    /*
     * Calculate PID values:
     * - error < 0 means black spot is to the left, so need to steer left
     *   by setting servo PWM duty cycle <
     *      TFC_STEERING_SERVO_NEUTRAL_DUTY_CYCLE_US
     * - error > 0 means black spot is to the right, so need to steer right
     *   by setting servo PWM duty cycle > 
     *      TFC_STEERING_SERVO_NEUTRAL_DUTY_CYCLE_US
     *
     * NOTE: right-most pixel index is 0 and left-most pixel index is
     * TFC_NUM_CAMERA_PIXELS - 1.
     */
    int error = (int)CENTER_PIXEL_INDEX - (int)black_spot_position;

    int derivative_term = error - previous_error;
    previous_error = error;
    integral_term += error;
   
    /*
     * Calculate new steering servo PWM duty cycle:
     */

    int offset_steering_servo_pwm_duty_cycle = 
             STEERING_SERVO_PROPORTIONAL_GAIN * error + 
             STEERING_SERVO_INTEGRAL_GAIN * integral_term +
             STEERING_SERVO_DERIVATIVE_GAIN * derivative_term;

    pwm_duty_cycle_us_t steering_servo_pwm_duty_cycle_us =
        (pwm_duty_cycle_us_t)calculate_new_manipulated_var_value(
                                TFC_STEERING_SERVO_NEUTRAL_DUTY_CYCLE_US,
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

    if (g_car.c_dip_switches_on[DRIVE_FAST_DIP_SWITCH]) {
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

    pwm_duty_cycle_us_t left_wheel_pwm_duty_cycle_us;
    pwm_duty_cycle_us_t right_wheel_pwm_duty_cycle_us;

    if (g_car.c_dip_switches_on[ACTIVE_DIFFERENTIAL_DIP_SWITCH] &&
        ABS(error) > PID_ERROR_THRESHOLD_IN_PIXELS &&
        wheel_motor_pwm_duty_cycle_us != TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US) {
        int offset_wheel_differential = 
                 -(WHEEL_DIFFERENTIAL_PROPORTIONAL_GAIN * ABS(error));

        if (error < 0) {
            /*
             * Help steer left by making left wheel spin slower
             */
            left_wheel_pwm_duty_cycle_us = 
                (pwm_duty_cycle_us_t)calculate_new_manipulated_var_value(
                                        wheel_motor_pwm_duty_cycle_us,
                                        offset_wheel_differential,
                                        TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US,
                                        TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US);

            right_wheel_pwm_duty_cycle_us = wheel_motor_pwm_duty_cycle_us;
        } else {
            /*
             * Help steer right by making right wheel spin slower
             */
            right_wheel_pwm_duty_cycle_us =
                (pwm_duty_cycle_us_t)calculate_new_manipulated_var_value(
                                        wheel_motor_pwm_duty_cycle_us,
                                        offset_wheel_differential,
                                        TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US,
                                        TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US);

            left_wheel_pwm_duty_cycle_us = wheel_motor_pwm_duty_cycle_us;
        }
    } else {
        left_wheel_pwm_duty_cycle_us = wheel_motor_pwm_duty_cycle_us;
        right_wheel_pwm_duty_cycle_us = wheel_motor_pwm_duty_cycle_us;
    }
        
    /*
     * Send commands to actuators:
     */

    tfc_wheel_motors_set(
        left_wheel_pwm_duty_cycle_us,
        right_wheel_pwm_duty_cycle_us);

    tfc_steering_servo_set(
        steering_servo_pwm_duty_cycle_us);

    *current_steering_servo_pwm_duty_cycle_us_p = steering_servo_pwm_duty_cycle_us;
}


static void
turn_off_car(void)
{
    rtos_mutex_acquire(g_car.c_turned_on_mutex_p);
    g_car.c_turned_on = false;
    rtos_mutex_release(g_car.c_turned_on_mutex_p);

    tfc_battery_leds_set(0);
    console_printf("Car turned off\n");

    tfc_wheel_motors_set(
        TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US,
        TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US);

    tfc_steering_servo_set(
        TFC_STEERING_SERVO_NEUTRAL_DUTY_CYCLE_US);
    tfc_steering_servo_set(
        TFC_STEERING_SERVO_OFF_DUTY_CYCLE_US);
}


static void
turn_on_car(void)
{
    set_wheels_straight();
    tfc_battery_leds_set(TFC_NUM_BATTERY_LEDS);
    console_printf("Car turned on\n");

    rtos_mutex_acquire(g_car.c_turned_on_mutex_p);
    g_car.c_turned_on = true;
    rtos_mutex_release(g_car.c_turned_on_mutex_p);
    rtos_condvar_broadcast(g_car.c_turned_on_condvar_p);
}
        

static fdc_error_t
car_driver_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    black_spot_position_t previous_center_black_spot_position;
    black_spot_position_t center_black_spot_position;
    pwm_duty_cycle_us_t current_steering_servo_pwm_duty_cycle_us;

    FDC_ASSERT(arg == NULL, arg, cpu_id);
    FDC_ASSERT(!g_car.c_turned_on, 0, 0);

    console_printf("Initializing car driver thread ...\n");

    for ( ; ; ) {
        struct tfc_camera_frame *camera_frame_p;

        rtos_mutex_acquire(g_car.c_turned_on_mutex_p);
        if (!g_car.c_turned_on) {
            do {
                rtos_condvar_wait(
                    g_car.c_turned_on_condvar_p,
                    g_car.c_turned_on_mutex_p);
            } while (!g_car.c_turned_on);

            set_wheels_straight();
            center_black_spot_position = CENTER_PIXEL_INDEX;
            current_steering_servo_pwm_duty_cycle_us =
                TFC_STEERING_SERVO_NEUTRAL_DUTY_CYCLE_US;
        }
        rtos_mutex_release(g_car.c_turned_on_mutex_p);

        pwm_duty_cycle_us_t base_wheel_motor_pwm_duty_cycle_us =
            calculate_base_wheel_motor_pwm_duty_cycle_us();

        tfc_trimpot_reading_t white_threshold =
            FILTER_TRIMPOT_READING(
                g_car.c_last_trimpot_readings[WHITE_THRESHOLD_TRIMPOT]);

        /*
         * Get the next camera frame to process, or wait if there is none yet:
         */
        (void)rtos_k_pointer_circular_buffer_read(
            &g_car.c_captured_camera_frames_queue,
            (void **)&camera_frame_p,
            true);

        previous_center_black_spot_position = center_black_spot_position;
        bool black_spot_found = 
                find_middle_black_spot(
                    camera_frame_p,
                    white_threshold,
                    &center_black_spot_position);

        /*
         * Return frame buffer to the frame buffer pool:
         */
        bool write_ok = rtos_k_pointer_circular_buffer_write(
                            &g_car.c_camera_frame_buffer_pool,
                            camera_frame_p,
                            false);

        FDC_ASSERT(write_ok, 0, 0);

        if (black_spot_found) {
#if 0
            if (ABS(center_black_spot_position -
                previous_center_black_spot_position) > TFC_NUM_CAMERA_PIXELS / 4 &&
                g_car.c_x_motion_detection == 0) {
                base_wheel_motor_pwm_duty_cycle_us =
                    TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US + THROTTLE_INCREMENT_UNIT * 6;
            }
#endif
        } else {
            //center_black_spot_position = CENTER_PIXEL_INDEX;
            center_black_spot_position = 
                (previous_center_black_spot_position + CENTER_PIXEL_INDEX) / 2;
#if 0
            base_wheel_motor_pwm_duty_cycle_us =
                TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US - THROTTLE_INCREMENT_UNIT * 2;
#endif
        }

//#if 0 // ???
        if (g_car.c_x_acceleration == 0x40) {
            base_wheel_motor_pwm_duty_cycle_us += THROTTLE_INCREMENT_UNIT;
        }
//#endif

        steer_wheels(
            center_black_spot_position,
            base_wheel_motor_pwm_duty_cycle_us,
            &current_steering_servo_pwm_duty_cycle_us);
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
                FILTER_TRIMPOT_READING(g_car.c_last_trimpot_readings[i]);

            if (filtered_reading != filtered_last_reading) {
                console_printf(
                    "Trimpot %u changed: %#x\n", i, filtered_reading);

                g_car.c_last_trimpot_readings[i] = trimpot_readings[i];
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
            if (dip_switches[i] != g_car.c_dip_switches_on[i]) {
                g_car.c_dip_switches_on[i] = dip_switches[i];
                switch (i) {
                case ASSERT_BREAKPOINT_DIP_SWITCH:
                    fdc_setting_changed = true;
                    console_printf(
                        "Assert breakpoints changed to %s\n",
                        dip_switches[i] ? "ON" : "OFF");
                    break;

                case EXCEPTION_DEBUGGER_DIP_SWITCH:
                    fdc_setting_changed = true;
                    console_printf(
                        "Exception debugger changed to %s\n",
                        dip_switches[i] ? "ON" : "OFF");
                    break;

                case ACTIVE_DIFFERENTIAL_DIP_SWITCH:
                    console_printf(
                        "Active differential changed to %s\n",
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
                dip_switches[EXCEPTION_DEBUGGER_DIP_SWITCH]);
        }

        /*
         * Read push buttons:
         */ 
        tfc_push_buttons_read(push_buttons);

        for (natural_t i = 0; i < TFC_NUM_PUSH_BUTTONS; i++) {
            if (push_buttons[i] != g_car.c_push_buttons[i]) {
                g_car.c_push_buttons[i] = push_buttons[i];
                switch (i) {
                case TURN_ON_CAR_PUSH_BUTTON:
                    turn_on_car();
                    break;
                case TURN_OFF_CAR_PUSH_BUTTON:
                    turn_off_car();
                    break;
                }
            }
        }

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
                    4);

            if (battery_level > TFC_NUM_BATTERY_LEDS) {
                battery_level = TFC_NUM_BATTERY_LEDS;
            }

            if (g_car.c_turned_on) {
                tfc_battery_leds_set(battery_level);
            }

            last_battery_reading = battery_reading;
        } else {
            if (!g_car.c_turned_on) {
                tfc_battery_leds_set(0);
            }
        }

        rtos_thread_delay(BATTERY_SAMPLING_PERIOD_MS);
    }   

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    return fdc_error;
}


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
                        (int16_t *)&g_car.c_x_acceleration,
                        (int16_t *)&g_car.c_y_acceleration,
                        (int16_t *)&g_car.c_z_acceleration);

        if (read_ok) {
#           if 0
            console_printf("x_accel: %#x, y_accel: %#x, z_accel: %#x\n",
                g_car.c_x_acceleration & ~0x3f,
                g_car.c_y_acceleration & ~0x3f,
                g_car.c_z_acceleration & ~0x3f);
#           endif
        }
#else
        bool read_ok = accelerometer_detect_motion(
                        (int8_t *)&g_car.c_x_motion_detection,
                        (int8_t *)&g_car.c_y_motion_detection,
                        (int8_t *)&g_car.c_z_motion_detection);
        if (read_ok) {
#           if 0
            console_printf("x_motion: %d, y_motion: %d, z_motion: %d\n",
                g_car.c_x_motion_detection,
                g_car.c_y_motion_detection,
                g_car.c_z_motion_detection);
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
