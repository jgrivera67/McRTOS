/**
 * @file frdm_board__hardware_abstractions.c
 *
 * Hardware abstraction layer for the FRDM (Freedom) main board
 *
 * @author German Rivera 
 */ 

#include "hardware_abstractions.h"
#include "kl25z_soc.h"
#include "McRTOS_arm_cortex_m.h"
#include "MKL25Z4.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"
#include "frdm_board.h"
#include "mma8451q_accelerometer.h"

static void rgb_led_init(void);

/**
 * FRDM board RGB LED pins
 */
static struct pin_config_info g_frdm_rgb_led_pins[] = {
    [FRDM_RED_LED] = PIN_COFIG_INFO_INITIALIZER(
            FRDM_RGB_LED_RED_PIN_INDEX,
            PORT_PCR_MUX(1),
            false,
            PORTB_BASE_PTR,
            PTB_BASE_PTR),

    [FRDM_GREEN_LED] = PIN_COFIG_INFO_INITIALIZER(
            FRDM_RGB_LED_GREEN_PIN_INDEX,
            PORT_PCR_MUX(1),
            false,
            PORTB_BASE_PTR,
            PTB_BASE_PTR),

    [FRDM_BLUE_LED] = PIN_COFIG_INFO_INITIALIZER(
            FRDM_RGB_LED_BLUE_PIN_INDEX,
            PORT_PCR_MUX(1),
            false,
            PORTD_BASE_PTR,
            PTD_BASE_PTR)
};

C_ASSERT(ARRAY_SIZE(g_frdm_rgb_led_pins) == FRDM_NUM_RGB_LED_PINS);

static uint32_t g_rgb_led_current_mask = 0x0;

void
frdm_board_init(void)
{
    rgb_led_init();

    #ifdef DEBUG
    if (software_reset_happened()) {
        DEBUG_BLINK_LED(LED_COLOR_YELLOW);
    } else {
        DEBUG_BLINK_LED(LED_COLOR_GREEN);
    }
#   else
    if (software_reset_happened()) {
        (void)set_rgb_led_color(LED_COLOR_YELLOW);
    } else {
        (void)set_rgb_led_color(LED_COLOR_GREEN);
    }
#   endif
}


void
frdm_board_stop(void)
{
    set_rgb_led_color(LED_COLOR_BLACK);
}



static void 
rgb_led_init(void)
{
    for (int i = 0; i < FRDM_NUM_RGB_LED_PINS; i++) {
        configure_pin(&g_frdm_rgb_led_pins[i], true);
        deactivate_output_pin(&g_frdm_rgb_led_pins[i]);
    }
}


void
toggle_rgb_led(uint32_t led_color_mask)
{
    g_rgb_led_current_mask ^= led_color_mask;

    for (int i = 0; i < FRDM_NUM_RGB_LED_PINS; i++) {
        if (g_frdm_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            toggle_output_pin(&g_frdm_rgb_led_pins[i]);
        }
    }
}


void
turn_on_rgb_led(uint32_t led_color_mask)
{
    g_rgb_led_current_mask |= led_color_mask;

    for (int i = 0; i < FRDM_NUM_RGB_LED_PINS; i++) {
        if (g_frdm_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            activate_output_pin(&g_frdm_rgb_led_pins[i]);
        }
    }
}


void
turn_off_rgb_led(uint32_t led_color_mask)
{
    g_rgb_led_current_mask &= ~led_color_mask;

    for (int i = 0; i < FRDM_NUM_RGB_LED_PINS; i++) {
        if (g_frdm_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            deactivate_output_pin(&g_frdm_rgb_led_pins[i]);
        }
    }
}


/*
 * Set the LED to the given color and returns the previous color
 */
uint32_t
set_rgb_led_color(uint32_t led_color_mask)
{
    uint32_t old_rgb_led_mask = g_rgb_led_current_mask;

    g_rgb_led_current_mask = led_color_mask;

    for (int i = 0; i < FRDM_NUM_RGB_LED_PINS; i++) {
        if (g_frdm_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            activate_output_pin(&g_frdm_rgb_led_pins[i]);
        } else {
            deactivate_output_pin(&g_frdm_rgb_led_pins[i]);
        }
    }

    return old_rgb_led_mask;
}


/**
 * Initializes MMA8451Q accelerometer
 *
 * NOTE: This function cannot be called with interrupts disabled, as it calls
 * functions that block on condition variables.
 */
void
accelerometer_init(void)
{
    struct i2c_device_var *const i2c_var_p = g_i2c0_device_p->i2c_var_p;
    FDC_ASSERT(i2c_var_p->i2c_initialized, g_i2c0_device_p, i2c_var_p);

    uint8_t accel_reg_value;

    i2c_read(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_WHO_AM_I,
        &accel_reg_value,
        1);

    FDC_ASSERT(accel_reg_value == ACCEL_DEVICE_ID, accel_reg_value, ACCEL_DEVICE_ID);

    i2c_read(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_F_SETUP,
        &accel_reg_value,
        1);
    FDC_ASSERT(accel_reg_value == 0, accel_reg_value, 0);

    i2c_read(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG1,
        &accel_reg_value,
        1);
    
    if ((accel_reg_value & ACCEL_CTRL_REG1_ACTIVE_MASK) != 0) {
        accelerometer_stop();
        i2c_read(
            g_i2c0_device_p,
            ACCELEROMETER_I2C_ADDR,
            ACCEL_CTRL_REG1,
            &accel_reg_value,
            1);

        FDC_ASSERT(
            (accel_reg_value & ACCEL_CTRL_REG1_ACTIVE_MASK) == 0,
            accel_reg_value, 0);
    }

    accel_reg_value |= ACCEL_CTRL_REG1_ACTIVE_MASK;
    i2c_write(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG1,
        &accel_reg_value,
        1);
}


/**
 * Stops the MMA8451Q accelerometer
 *
 * NOTE: This function cannot be called with interrupts disabled, as it calls
 * functions that block on condition variables.
 */
void
accelerometer_stop(void)
{
    struct i2c_device_var *const i2c_var_p = g_i2c0_device_p->i2c_var_p;
    FDC_ASSERT(i2c_var_p->i2c_initialized, g_i2c0_device_p, i2c_var_p);

    uint8_t accel_reg_value;

    i2c_read(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG1,
        &accel_reg_value,
        1);
    
    FDC_ASSERT(
        (accel_reg_value & ACCEL_CTRL_REG1_ACTIVE_MASK) != 0,
        accel_reg_value, 0);

    accel_reg_value &= ~ACCEL_CTRL_REG1_ACTIVE_MASK;
    i2c_write(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG1,
        &accel_reg_value,
        1);
}


/**
 * Read MMA8451Q accelerometer
 *
 * NOTE: This function cannot be called with interrupts disabled, as it calls
 * functions that block on condition variables.
 */
void
accelerometer_read(
    uint16_t *x_p,
    uint16_t *y_p,
    uint16_t *z_p)
{
    uint8_t accel_reg_value;
    uint8_t i2c_buf[6];

    i2c_read(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_STATUS,
        &accel_reg_value,
        1);
   
    if ((accel_reg_value & ACCEL_STATUS_ZYXDR_MASK) != 0) {
        FDC_ASSERT(
            (accel_reg_value & (ACCEL_STATUS_XDR_MASK|
                           ACCEL_STATUS_YDR_MASK|
                           ACCEL_STATUS_ZDR_MASK)) ==
            (ACCEL_STATUS_XDR_MASK|
             ACCEL_STATUS_YDR_MASK|
             ACCEL_STATUS_ZDR_MASK), accel_reg_value, 0);
#if 1
        i2c_read(
            g_i2c0_device_p,
            ACCELEROMETER_I2C_ADDR,
            ACCEL_OUT_X_MSB,
            i2c_buf,
            sizeof i2c_buf);

        *x_p = (i2c_buf[0] << 8) | i2c_buf[1];
        *y_p = (i2c_buf[2] << 8) | i2c_buf[3];
        *z_p = (i2c_buf[4] << 8) | i2c_buf[5];
#else
            i2c_read(
                g_i2c0_device_p,
                ACCELEROMETER_I2C_ADDR,
                ACCEL_OUT_X_MSB,
                i2c_buf,
                2);

            *x_p = (i2c_buf[0] << 8) | i2c_buf[1];

            i2c_read(
                g_i2c0_device_p,
                ACCELEROMETER_I2C_ADDR,
                ACCEL_OUT_Y_MSB,
                i2c_buf,
                2);

            *y_p = (i2c_buf[0] << 8) | i2c_buf[1];

            i2c_read(
                g_i2c0_device_p,
                ACCELEROMETER_I2C_ADDR,
                ACCEL_OUT_Z_MSB,
                i2c_buf,
                2);

            *z_p = (i2c_buf[0] << 8) | i2c_buf[1];
#endif
    
        DEBUG_PRINTF("x=%#x, y=%#x, z=%#x\n", *x_p, *y_p, *z_p);
    }
}

