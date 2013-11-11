/**
 * @file tfc_board.h
 *
 * Freescale TFC add-on board declarations
 *
 * @author German Rivera 
 */ 
#ifndef __TFC_BOARD_H
#define __TFC_BOARD_H

#include <stdint.h>

#define TFC_NUM_BATTERY_LEDS    4
#define TFC_NUM_DIP_SWITCHES    4
#define TFC_NUM_PUSH_BUTTONS    2
#define TFC_NUM_TRIMPOTS        2
#define TFC_NUM_CAMERA_PIXELS   128

/**
 * Camera sampling delay in milliseconds 
 *
 * NOTE: This delay + frame capture time must be <= 100ms, to prevent
 * saturating the pixel capacitors, before the next frame capture.
 * Also, this delay has to be >= 20ms, which is the steering servo response
 * time, and you want to give time to the steering servo to fully act before
 * capturing the next frame from the camera.
 */
#define TFC_CAMERA_SAMPLING_DELAY_MS    20

/**
 * Steering servo minimum duty cycle in microseconds
 * (limit for steering to the left)
 */
#define TFC_STEERING_SERVO_MIN_DUTY_CYCLE_US    UINT32_C(1000)

/**
 * Steering servo middle duty cycle in microseconds
 * (for center position - wheels straight)
 */
#define TFC_STEERING_SERVO_MIDDLE_DUTY_CYCLE_US   UINT32_C(1500)

/**
 * Steering servo maximum duty cycle in microseconds
 * (limit for steering to the right)
 */
#define TFC_STEERING_SERVO_MAX_DUTY_CYCLE_US    UINT32_C(2000)

/**
 * Steering servo off duty cycle in microseconds 
 */
#define TFC_STEERING_SERVO_OFF_DUTY_CYCLE_US    UINT32_C(0)

/**
 * Steering servo "neutral" duty cycle in microseconds
 */
#define TFC_STEERING_SERVO_NEUTRAL_DUTY_CYCLE_US \
        TFC_STEERING_SERVO_MIDDLE_DUTY_CYCLE_US

/**
 * Wheel motor minimum duty cycle in microseconds
 * (limit for backward wheel speed)
 */
#define TFC_WHEEL_MOTOR_MIN_DUTY_CYCLE_US   UINT32_C(0)

/**
 * Wheel motor middle duty cycle in microseconds
 * (for center position - wheel stopped)
 */
#define TFC_WHEEL_MOTOR_MIDDLE_DUTY_CYCLE_US    UINT32_C(100)

/**
 * Wheel motor maximum duty cycle in microseconds
 * (limit for wheel forward speed)
 */
#define TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US    UINT32_C(200)

/**
 * Wheel motor stopped duty cycle in microseconds
 */
#define TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US \
        TFC_WHEEL_MOTOR_MIDDLE_DUTY_CYCLE_US

/**
 * Maximum wheel speed delta relative to wheel motor PWM duty cycle
 * for stopped wheel
 */
#define TFC_WHEEL_MOTOR_MAX_SPEED_DELTA \
        (TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US -                            \
         TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US)

/*
 * Camera raw pixel range type
 */
typedef adc_result_t tfc_camera_raw_pixel_t;

/*
 * Trimpot reading range type
 */
typedef adc_result_t tfc_trimpot_reading_t;

/*
 * Battery sensor reading range type
 */
typedef adc_result_t tfc_battery_reading_t;

/**
 * Raw camera frame, where each pixel is presented by a sampled voltage.
 */
struct tfc_camera_frame {
    tfc_camera_raw_pixel_t cf_pixels[TFC_NUM_CAMERA_PIXELS];
};

void tfc_board_init(void);
void tfc_board_stop(void);

void
tfc_steering_servo_set(
    pwm_duty_cycle_us_t pwm_duty_cycle_us);

void
tfc_wheel_motors_set(
    pwm_duty_cycle_us_t left_wheel_pwm_duty_cycle_us,
    pwm_duty_cycle_us_t right_wheel_pwm_duty_cycle_us);

void
tfc_camera_read_frame(
    _OUT_ struct tfc_camera_frame *camera_frame_p);

tfc_battery_reading_t
tfc_battery_sensor_read(void);

void
tfc_battery_leds_set(natural_t battery_level);

void
tfc_trimpots_read(
        _OUT_ tfc_trimpot_reading_t trimpot_readings[]);

void
tfc_push_buttons_read(
        _OUT_ bool push_buttons[]);

void
tfc_dip_switches_read(
        _OUT_ bool dip_switches[]);

#endif /* __TFC_BOARD_H */
