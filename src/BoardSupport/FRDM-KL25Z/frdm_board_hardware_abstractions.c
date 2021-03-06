/**
 * @file frdm_board_hardware_abstractions.c
 *
 * Hardware abstraction layer for the FRDM (Freedom) main board
 *
 * @author German Rivera
 */

#include <BoardSupport/hardware_abstractions.h>
#include <BoardSupport/FRDM-KL25Z/kl25z_soc.h>
#include <McRTOS/McRTOS_arm_cortex_m.h>
#include <BoardSupport/FRDM-KL25Z/MKL25Z4.h>
#include <McRTOS/failure_data_capture.h>
#include <McRTOS/utils.h>
#include <McRTOS/McRTOS_config_parameters.h>
#include <McRTOS/McRTOS_kernel_services.h>
#include <BoardSupport/FRDM-KL25Z/frdm_board.h>
#include <BoardSupport/FRDM-KL25Z/mma8451q_accelerometer.h>

static void rgb_led_init(void);

/**
 * FRDM board RGB LED pins
 */
static struct gpio_pin g_frdm_rgb_led_pins[] = {
    [FRDM_RED_LED] = GPIO_PIN_INITIALIZER(
            PORT_B,
            FRDM_RGB_LED_RED_PIN_INDEX,
            1,
            false),

    [FRDM_GREEN_LED] =  GPIO_PIN_INITIALIZER(
            PORT_B,
            FRDM_RGB_LED_GREEN_PIN_INDEX,
            1,
            false),

    [FRDM_BLUE_LED] = GPIO_PIN_INITIALIZER(
            PORT_D,
            FRDM_RGB_LED_BLUE_PIN_INDEX,
            1,
            false),
};

C_ASSERT(ARRAY_SIZE(g_frdm_rgb_led_pins) == FRDM_NUM_RGB_LED_PINS);

static uint32_t g_rgb_led_current_mask = 0x0;

/**
 * FRDM board accelerometer INT1 pin
 */
static struct gpio_pin g_frdm_accelerometer_int1_pin =
        GPIO_PIN_INITIALIZER(
            PORT_A,
            FRDM_ACCELEROMETER_INT1_PIN_INDEX,
            1,
            false);

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
    fdc_error_t error;

    fdc_error = rtos_mpu_rw_region_push(g_i2c0_device_p, g_i2c0_device_p + 1);
    if (fdc_error != 0) {
        fatal_error_handler(fdc_error);
    }

    struct i2c_device_var *const i2c_var_p = g_i2c0_device_p->i2c_var_p;

    FDC_ASSERT(i2c_var_p->i2c_initialized, g_i2c0_device_p, i2c_var_p);

    uint8_t accel_reg_value;

    configure_pin(&g_frdm_accelerometer_int1_pin, false);

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

#if 0
    /*
     * Enable motion detection for X, Y, Z axes with no latch:
     */
    accel_reg_value = (ACCEL_FF_MT_CFG_OAE_MASK |
                       ACCEL_FF_MT_CFG_XEFE_MASK |
                       ACCEL_FF_MT_CFG_YEFE_MASK |
                       ACCEL_FF_MT_CFG_ZEFE_MASK);
    i2c_write(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_FF_MT_CFG,
        &accel_reg_value,
        1);

    /*
     * Set threshold value for motion detection of > 0.125g:
     * 0.125g/0.063g = 1.984 ~ 2.
     */
    accel_reg_value = 0;
    SET_BIT_FIELD(
        accel_reg_value, ACCEL_FF_MT_THS_THS_MASK, ACCEL_FF_MT_THS_THS_SHIFT,
        2);
    i2c_write(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_FF_MT_THS,
        &accel_reg_value,
        1);

    /*
     * Set the debounce counter to eliminate false readings for 100 Hz sample
     * rate with a requirement of 100 ms timer. See table 7 of AN4070
     */
    accel_reg_value = 10;
    i2c_write(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_FF_MT_COUNT,
        &accel_reg_value,
        1);
#endif

    /*
     * Set rate to sample acceleration every 10ms and activate
     * accelerometer:
     */
    accel_reg_value |= ACCEL_CTRL_REG1_ACTIVE_MASK;
    SET_BIT_FIELD(
        accel_reg_value,
        ACCEL_CTRL_REG1_DR_MASK,
        ACCEL_CTRL_REG1_DR_SHIFT,
        ACCEL_CTRL_REG1_DR_VALUE_100HZ);

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
 * Read MMA8451Q accelerometer status
 *
 * NOTE: This function cannot be called with interrupts disabled, as it calls
 * functions that block on condition variables.
 */
bool
accelerometer_read_status(
    int16_t *x_p,
    int16_t *y_p,
    int16_t *z_p)
{
#   define BUILD_14_BIT_SIGNED(_msb, _lsb) \
            (((int8_t)(_msb) << 6) | ((uint8_t)(_lsb) >> 2))

#   define IN_RANGE_14_BIT_SIGNED(_num) \
            ((_num) >= -(int16_t)BIT(13) && (_num) <= (int16_t)(BIT(13) - 1))

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

        i2c_read(
            g_i2c0_device_p,
            ACCELEROMETER_I2C_ADDR,
            ACCEL_OUT_X_MSB,
            i2c_buf,
            sizeof i2c_buf);

        *x_p = BUILD_14_BIT_SIGNED(i2c_buf[0], i2c_buf[1]);
        *y_p = BUILD_14_BIT_SIGNED(i2c_buf[2], i2c_buf[3]);
        *z_p = BUILD_14_BIT_SIGNED(i2c_buf[4], i2c_buf[5]);

        DBG_ASSERT(IN_RANGE_14_BIT_SIGNED(*x_p), *x_p, 0);
        DBG_ASSERT(IN_RANGE_14_BIT_SIGNED(*y_p), *y_p, 0);
        DBG_ASSERT(IN_RANGE_14_BIT_SIGNED(*z_p), *z_p, 0);
        return true;
    } else {
        return false;
    }
}


/**
 * Read MMA8451Q accelerometer motion detection
 *
 * NOTE: This function cannot be called with interrupts disabled, as it calls
 * functions that block on condition variables.
 */
bool
accelerometer_detect_motion(
    int8_t *x_p,
    int8_t *y_p,
    int8_t *z_p)
{
    uint8_t accel_reg_value;

    i2c_read(
        g_i2c0_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_FF_MT_SRC,
        &accel_reg_value,
        1);

    if ((accel_reg_value & ACCEL_FF_MT_SRC_EA_MASK) != 0) {
        if (accel_reg_value & ACCEL_FF_MT_SRC_XHE_MASK) {
            if (accel_reg_value & ACCEL_FF_MT_SRC_XHP_MASK) {
                *x_p = 1;
            } else {
                *x_p = -1;
            }
        } else {
            *x_p = 0;
        }

        if (accel_reg_value & ACCEL_FF_MT_SRC_YHE_MASK) {
            if (accel_reg_value & ACCEL_FF_MT_SRC_YHP_MASK) {
                *y_p = 1;
            } else {
                *y_p = -1;
            }
        } else {
            *y_p = 0;
        }

        if (accel_reg_value & ACCEL_FF_MT_SRC_ZHE_MASK) {
            if (accel_reg_value & ACCEL_FF_MT_SRC_ZHP_MASK) {
                *z_p = 1;
            } else {
                *z_p = -1;
            }
        } else {
            *z_p = 0;
        }
        return true;
    } else {
        return false;
    }
}

