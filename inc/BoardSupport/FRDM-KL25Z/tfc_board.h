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

#define TFC_HBRIDGE_EN_LOC   BIT(21)
#define TFC_BAT_LED0_LOC     BIT(8)
#define TFC_BAT_LED1_LOC     BIT(9) 
#define TFC_BAT_LED2_LOC     BIT(10) 
#define TFC_BAT_LED3_LOC     BIT(11) 

/**
 * Steering servo PWM overflow frequency in Hz
 * (PWM period: 20 ms)
 */
#define TFC_STEERING_SERVO_TPM_OVERFLOW_FREQ_HZ UINT16_C(50)

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
 * (wheels turned left)
 */
#define TFC_STEERING_SERVO_OFF_DUTY_CYCLE_US    UINT32_C(0)

/**
 * Steering servo "wheels straight" duty cycle in microseconds
 */
#define TFC_STEERING_SERVO_STRAIGHT_DUTY_CYCLE_US \
        TFC_STEERING_SERVO_MIDDLE_DUTY_CYCLE_US

/**
 * Wheel motor PWM overflow frequency in Hz
 * (PWM period: 200 us)
 */
#define TFC_WHEEL_MOTOR_TPM_OVERFLOW_FREQ_HZ UINT16_C(5000)

/**
 * Wheel motor minimum duty cycle in microseconds
 * (limit for wheel speed going backwards)
 */
#define TFC_WHEEL_MOTOR_MIN_DUTY_CYCLE_US   UINT32_C(0)

/**
 * Wheel motor middle duty cycle in microseconds
 * (for center position - wheel stopped)
 */
#define TFC_WHEEL_MOTOR_MIDDLE_DUTY_CYCLE_US    UINT32_C(100)

/**
 * Wheel motor maximum duty cycle in microseconds
 * (limit for wheel speed going forward)
 */
#define TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US    UINT32_C(200)

/**
 * Wheel motor stopped duty cycle in microseconds
 */
#define TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US \
        TFC_WHEEL_MOTOR_MIDDLE_DUTY_CYCLE_US

void tfc_board_init(void);

void
tfc_steering_servo_set(
    pwm_duty_cycle_us_t pwm_duty_cycle_us);

void
tfc_wheel_motors_set(
    pwm_duty_cycle_us_t left_wheel_pwm_duty_cycle_us,
    pwm_duty_cycle_us_t right_wheel_pwm_duty_cycle_us);

#endif /* __TFC_BOARD_H */
