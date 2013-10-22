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
 * Camera exposure time in milliseconds
 */
#define TFC_CAMERA_EXPOSURE_TIME_MS 50

/*
 * Camera raw pixel range type
 */
typedef _RANGE_(0, ADC_RESULT_MAX_VALUE)
        uint16_t tfc_camera_raw_pixel_t;

/*
 * Trimpot reading range type
 */
typedef _RANGE_(0, ADC_RESULT_MAX_VALUE)
        uint16_t tfc_trimpot_reading_t;

/*
 * Battery sensor reading range type
 */
typedef _RANGE_(0, ADC_RESULT_MAX_VALUE)
        uint16_t tfc_battery_reading_t;

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
    _OUT_ tfc_camera_raw_pixel_t camera_frame_raw_pixels[]);

tfc_battery_reading_t
tfc_battery_sensor_read(void);

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
