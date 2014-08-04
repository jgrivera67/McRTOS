/**
 * @file frdm_board_hardware_abstractions.c
 *
 * Hardware abstraction layer for the FRDM (Freedom) main board
 *
 * @author German Rivera
 */

#include "hardware_abstractions.h"
#include "k64f_soc.h"
#include "McRTOS_arm_cortex_m.h"
#include "MK64F12.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"
#include "frdm_board.h"
#include "fxos8700cq_accelerometer.h"

/**
 * Const fields of the Accelerometer device (to be placed in flash)
 */
struct accelerometer_device {
#   define ACCEL_DEVICE_SIGNATURE  GEN_SIGNATURE('A', 'C', 'C', 'E')
    uint32_t acc_signature;
    struct accelerometer_device_var *acc_var_p;
    const struct i2c_device *acc_i2c_device_p;
    struct pin_config_info acc_int1_pin;
    struct pin_config_info acc_int2_pin;
    struct rtos_interrupt_registration_params acc_rtos_interrupt_params;
    struct rtos_interrupt **acc_rtos_interrupt_pp;
};

/**
 * Non-const fields of an I2C controller device (to be placed in SRAM)
 */
struct accelerometer_device_var {
    /**
     * Flag indicating if accelerometer_init() has been called for this device
     */
    bool acc_initialized;
};

static void rgb_led_init(void);

/**
 * FRDM board RGB LED pins
 */
static const struct pin_config_info g_frdm_rgb_led_pins[] = {
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
            PORTE_BASE_PTR,
            PTE_BASE_PTR),

    [FRDM_BLUE_LED] = PIN_COFIG_INFO_INITIALIZER(
            FRDM_RGB_LED_BLUE_PIN_INDEX,
            PORT_PCR_MUX(1),
            false,
            PORTB_BASE_PTR,
            PTB_BASE_PTR)
};

C_ASSERT(ARRAY_SIZE(g_frdm_rgb_led_pins) == FRDM_NUM_RGB_LED_PINS);

static uint32_t g_rgb_led_current_mask = 0x0;

/**
 * Global non-const structure for accelerometer device
 * (allocated in SRAM space)
 */
static struct accelerometer_device_var g_accelerometer_var = {
    .acc_initialized = false,
};

/**
 * McRTOS interrupt object for Port C interrupt
 */
struct rtos_interrupt *g_rtos_interrupt_port_c_p = NULL;

/**
 * Global const structure for the accelerometer device
 * (allocated in flash space)
 */
static const struct accelerometer_device g_accelerometer = {
        .acc_signature = ACCEL_DEVICE_SIGNATURE,
        .acc_var_p = &g_accelerometer_var,
	.acc_i2c_device_p = &g_i2c_devices[0],

	.acc_int1_pin =
	    PIN_COFIG_INFO_INITIALIZER(
		FRDM_ACCELEROMETER_INT1_PIN_INDEX,
		/* Pin Mux Control: Alternative 1, Interrupt when logical 1 */
		PORT_PCR_MUX(1) | PORT_PCR_IRQC(0xC),
		false,
		PORTC_BASE_PTR,
		PTC_BASE_PTR),

	.acc_int2_pin =
	    PIN_COFIG_INFO_INITIALIZER(
		FRDM_ACCELEROMETER_INT2_PIN_INDEX,
		/* Pin Mux Control: Alternative 1, Interrupt when logical 1 */
		PORT_PCR_MUX(1) | PORT_PCR_IRQC(0xC),
		false,
		PORTC_BASE_PTR,
		PTC_BASE_PTR),

	.acc_rtos_interrupt_params = {
            .irp_name_p = "Port C Interrupts (Accelerometer INT1, INT2)",
            .irp_isr_function_p = k64f_port_c_isr,
            .irp_arg_p = (void *)&g_accelerometer,
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PORTC),
            .irp_priority = PORT_C_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

        .acc_rtos_interrupt_pp = &g_rtos_interrupt_port_c_p,
};


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
        (void)set_rgb_led_color(LED_COLOR_BLACK);
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


static void
accelerometer_device_stop(const struct accelerometer_device *accel_device_p)
{
    FDC_ASSERT(
        accel_device_p->acc_signature == ACCEL_DEVICE_SIGNATURE,
        accel_device_p->acc_signature, 0);

    struct accelerometer_device_var *const accel_var_p = accel_device_p->acc_var_p;

    FDC_ASSERT(accel_var_p->acc_initialized, accel_device_p, accel_var_p);

    uint8_t accel_reg_value;

    i2c_read(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG1,
        &accel_reg_value,
        1);

    if ((accel_reg_value & ACCEL_CTRL_REG1_ACTIVE_MASK) != 0) {
	accel_reg_value &= ~ACCEL_CTRL_REG1_ACTIVE_MASK;
	i2c_write(
	    accel_device_p->acc_i2c_device_p,
	    ACCELEROMETER_I2C_ADDR,
	    ACCEL_CTRL_REG1,
	    &accel_reg_value,
	    1);

	i2c_read(
	    accel_device_p->acc_i2c_device_p,
	    ACCELEROMETER_I2C_ADDR,
	    ACCEL_CTRL_REG1,
	    &accel_reg_value,
	    1);

	FDC_ASSERT(
	    (accel_reg_value & ACCEL_CTRL_REG1_ACTIVE_MASK) == 0,
	    accel_reg_value, 0);
    }
}


static void
accelerometer_device_init(const struct accelerometer_device *accel_device_p)
{
    FDC_ASSERT(
        accel_device_p->acc_signature == ACCEL_DEVICE_SIGNATURE,
        accel_device_p->acc_signature, 0);

    struct accelerometer_device_var *const accel_var_p = accel_device_p->acc_var_p;

    FDC_ASSERT(!accel_var_p->acc_initialized, accel_device_p, accel_var_p);

    uint8_t accel_reg_value;

    i2c_read(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_WHO_AM_I,
        &accel_reg_value,
        1);

    FDC_ASSERT(accel_reg_value == ACCEL_DEVICE_ID, accel_reg_value, ACCEL_DEVICE_ID);

    /*
     * Reset FXOS chip:
     */

    accel_reg_value = ACCEL_CTRL_REG2_RST_MASK;
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG2,
        &accel_reg_value,
        1);

    rtos_thread_delay(50);

    /*
     * Go to standby mode:
     */
    accelerometer_device_stop(accel_device_p);

    /*
     * Disable the FIFO
     */
    accel_reg_value = 0;
     SET_BIT_FIELD(
        accel_reg_value,
        ACCEL_F_SETUP_MODE_MASK,
        ACCEL_F_SETUP_MODE_SHIFT,
        ACCEL_F_SETUP_MODE_DISABLED);
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_F_SETUP,
        &accel_reg_value,
        1);

    /*
     * Enable auto-sleep, low power in sleep, high res in wake
     */
    accel_reg_value = ACCEL_CTRL_REG2_SLPE_MASK;
    SET_BIT_FIELD(
        accel_reg_value,
        ACCEL_CTRL_REG2_SMODS_MASK,
        ACCEL_CTRL_REG2_SMODS_SHIFT,
        ACCEL_CTRL_REG2_SMOD_LOW_POWER);
    SET_BIT_FIELD(
        accel_reg_value,
        ACCEL_CTRL_REG2_MODS_MASK,
        ACCEL_CTRL_REG2_MODS_SHIFT,
        ACCEL_CTRL_REG2_MOD_HIGH_RES);
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG2,
        &accel_reg_value,
        1);

    /*
     * Set up Magnetometer OSR and Hybrid mode, use default for Acc
     */
    accel_reg_value = 0;
    SET_BIT_FIELD(
        accel_reg_value,
        MAGNET_CTRL_REG1_OSR_MASK,
        MAGNET_CTRL_REG1_OSR_SHIFT,
        0x7);
    SET_BIT_FIELD(
        accel_reg_value,
        MAGNET_CTRL_REG1_HMS_MASK,
        MAGNET_CTRL_REG1_HMS_SHIFT,
        0x3);
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        MAGNET_CTRL_REG1,
        &accel_reg_value,
        1);

    /*
     * Enable hyrid mode auto increment
     */
    accel_reg_value = MAGNET_CTRL_REG2_HYB_AUTOINC_MASK;
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        MAGNET_CTRL_REG2,
        &accel_reg_value,
        1);

    /*
     * Enable FFMT for motion detect for X, Y and Z axes, latch enable
     */
    accel_reg_value = (ACCEL_FF_MT_CFG_XEFE_MASK |
                       ACCEL_FF_MT_CFG_YEFE_MASK |
                       ACCEL_FF_MT_CFG_ZEFE_MASK |
		       ACCEL_FF_MT_CFG_ELE_MASK |
		       ACCEL_FF_MT_CFG_OAE_MASK);
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_FF_MT_CFG,
        &accel_reg_value,
        1);

    /*
     * Set threshold to about 0.25g
     */
    accel_reg_value = 0x4;
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_FF_MT_THS,
        &accel_reg_value,
        1);

    /*
     * Set debounce to zero
     */
    accel_reg_value = 0x0;
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_FF_MT_COUNT,
        &accel_reg_value,
        1);

    /*
     * Set auto-sleep wait period to 5s (=5/0.64=~8)
     */
    accel_reg_value = 8;
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_ASLP_COUNT,
        &accel_reg_value,
        1);

    /*
     * Register McRTOS interrupt handler
     */
    rtos_k_register_interrupt(
        &accel_device_p->acc_rtos_interrupt_params,
        accel_device_p->acc_rtos_interrupt_pp);

    DBG_ASSERT(
        *accel_device_p->acc_rtos_interrupt_pp != NULL,
        accel_device_p->acc_rtos_interrupt_pp, accel_device_p);

    /*
     * Configure INT1 and INT2 interrupt pins:
     */
    configure_pin(&accel_device_p->acc_int1_pin, false);
    configure_pin(&accel_device_p->acc_int2_pin, false);

    /*
     * Enable data-ready, auto-sleep and motion detection interrupts:
     */
#if 0
    accel_reg_value = (ACCEL_CTRL_REG4_INT_EN_DRDY_MASK |
		       ACCEL_CTRL_REG4_INT_EN_ASLP_MASK |
		       ACCEL_CTRL_REG4_INT_EN_FF_MT_MASK);
#else
    accel_reg_value = 0;
#endif
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG4,
        &accel_reg_value,
        1);

    /*
     * Route data-ready interrupts to INT1, others INT2 (default)
     */
    accel_reg_value = ACCEL_CTRL_REG5_INT_CFG_DRDY_MASK;
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG5,
        &accel_reg_value,
        1);

    /*
     * Enable ffmt as a wake-up source
     */
    accel_reg_value = ACCEL_CTRL_REG3_WAKE_FF_MT_MASK;
    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG3,
        &accel_reg_value,
        1);

    /*
     * Set sampling rates and activate accelerometer:
     * - ASLP rate: every 640ms (1.56 HZ)
     * - data rate: 200 HZ (every 5ms)
     * - FSR=2g
     */

    accel_reg_value = 0;
    SET_BIT_FIELD(
        accel_reg_value,
        ACCEL_CTRL_REG1_ASLP_RATE_MASK,
        ACCEL_CTRL_REG1_ASLP_RATE_SHIFT,
        ACCEL_CTRL_REG1_ASLP_RATE_640MS);

    SET_BIT_FIELD(
        accel_reg_value,
        ACCEL_CTRL_REG1_DR_MASK,
        ACCEL_CTRL_REG1_DR_SHIFT,
        ACCEL_CTRL_REG1_DR_200HZ);

    accel_reg_value |= ACCEL_CTRL_REG1_ACTIVE_MASK;

    i2c_write(
        accel_device_p->acc_i2c_device_p,
        ACCELEROMETER_I2C_ADDR,
        ACCEL_CTRL_REG1,
        &accel_reg_value,
        1);

    i2c_read(
	accel_device_p->acc_i2c_device_p,
	ACCELEROMETER_I2C_ADDR,
	ACCEL_CTRL_REG1,
	&accel_reg_value,
	1);

    FDC_ASSERT(
	(accel_reg_value & ACCEL_CTRL_REG1_ACTIVE_MASK) != 0,
	accel_reg_value, 0);
}


/**
 * Initializes FXOS8700CQ accelerometer
 *
 * NOTE: This function cannot be called with interrupts disabled, as it calls
 * functions that block on condition variables.
 */
void
accelerometer_init(void)
{
    rtos_enter_privileged_mode();
    accelerometer_device_init(&g_accelerometer);
    rtos_exit_privileged_mode();
}


/**
 * Stops the FXOS8700CQ accelerometer
 *
 * NOTE: This function cannot be called with interrupts disabled, as it calls
 * functions that block on condition variables.
 */
void
accelerometer_stop(void)
{
    rtos_enter_privileged_mode();
    accelerometer_device_stop(&g_accelerometer);
    rtos_exit_privileged_mode();
}


/**
 * Read FXOS8700CQ accelerometer status
 *
 * NOTE: This function cannot be called with interrupts disabled, as it calls
 * functions that block on condition variables.
 */
static bool
accelerometer_device_read_status(
    const struct accelerometer_device *accel_device_p,
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
    bool read_ok;

    i2c_read(
        accel_device_p->acc_i2c_device_p,
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
            accel_device_p->acc_i2c_device_p,
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
        read_ok = true;
    } else {
        read_ok = false;
    }

    return read_ok;
}


bool
accelerometer_read_status(
    int16_t *x_p,
    int16_t *y_p,
    int16_t *z_p)
{
    bool read_ok;

    rtos_enter_privileged_mode();
    read_ok = accelerometer_device_read_status(&g_accelerometer, x_p, y_p, z_p);
    rtos_exit_privileged_mode();
    return read_ok;
}


#if 0
/**
 * Read FXOS8700CQ accelerometer motion detection
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
    bool read_ok;

    rtos_enter_privileged_mode();
    i2c_read(
        accel_device_p->acc_i2c_device_p,
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

        read_ok = true;
    } else {
        read_ok = false;
    }

    rtos_exit_privileged_mode();
    return read_ok;
}
#endif

