/**
 * @file tfc_board_hardware_abstractions.c
 *
 * Hardware abstraction layer for the TFC add-on board
 *
 * @author German Rivera
 */
#include <BoardSupport/hardware_abstractions.h>
#include <BoardSupport/FRDM-KL25Z/kl25z_soc.h>
#include <McRTOS/McRTOS_arm_cortex_m.h>
#include <McRTOS/failure_data_capture.h>
#include <McRTOS/utils.h>
#include <McRTOS/McRTOS_config_parameters.h>
#include <McRTOS/McRTOS_kernel_services.h>
#include "tfc_board.h"

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#define TFC_CAMERA_CLK_PULSE_DELAY \
        ((SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ / 3) /* iterations in 1us */ * 24) // 50

/*
 * KL25 GPIO PORT B Pins
 */
#define TFC_BATTERY_LEDS_FIRST_PIN_INDEX    8

/*
 * KL25 GPIO PORT C Pins
 */
#define TFC_PUSH_BUTTON_SW1_PIN_INDEX 13
#define TFC_PUSH_BUTTON_SW2_PIN_INDEX 17

/*
 * KL25 GPIO PORT D Pins
 */

#define TFC_CAMERA_AO0_PIN_INDEX    5
#define TFC_CAMERA_AO1_PIN_INDEX    6
#define TFC_CAMERA_SI_PIN_INDEX     7

/*
 * KL25 GPIO PORT E Pins
 */

#define TFC_CAMERA_CLK_PIN_INDEX            1
#define TFC_DIP_SWITCHES_FIRST_PIN_INDEX    2
#define TFC_HBRIDGE_FAULT_PIN_INDEX         20
#define TFC_HBRIDGE_ENABLE_PIN_INDEX        21

/*
 * ADC channels
 */
#define TFC_BATTERY_SENSE_CHANNEL	4
#define TFC_LINESCAN_0_ADC_CHANNEL	6
#define TFC_LINESCAN_1_ADC_CHANNEL	7
#define TFC_POT_1_ADC_CHANNEL		12
#define TFC_POT_0_ADC_CHANNEL		13

static void tfc_steering_servo_init(void);
static void tfc_wheel_motors_init(void);
static void tfc_camera_init(void);
static void tfc_battery_sensor_init(void);
static void tfc_trimpots_init(void);
static void tfc_push_buttons_init(void);
static void tfc_dip_switches_init(void);

/**
 * Pointer to the McRTOS interrupt objects for the TPM interrupts.
 */
struct rtos_interrupt *g_rtos_interrupt_tpm0_p = NULL;
struct rtos_interrupt *g_rtos_interrupt_tpm1_p = NULL;

/**
 * Global array of non-const structures for TPM devices for the KL25Z SoC
 * (allocated in SRAM space)
 */
static struct tpm_device_var g_tpm_devices_var[] =
{
    [0] = {
        .tpm_initialized = false,
    },
    [1] = {
        .tpm_initialized = false,
    },
    [2] = {
        .tpm_initialized = false,
    },
};

/**
 * Global array of const structures for TPM devices for the KL25Z SoC
 * (allocated in flash space)
 */
const struct tpm_device g_tpm_devices[] =
{
    [0] = {
        .tpm_signature = TPM_DEVICE_SIGNATURE,
        .tpm_var_p = &g_tpm_devices_var[0],
        .tpm_name_p = "Wheel motors TPM",
        .tpm_mmio_p = TPM0_BASE_PTR,
        .tpm_wait_pwm_cycle_completion = false,
        .tpm_channels = {
            [0] = {
                .tpm_mmio_pcr_p = &PORTC_PCR1, /* Motor B (H-Bridge B - 1) */

                /*
                 * - Use (MSnB:MSnA = 1:0): Edge-aligned PWM mode.
                 * - Use (ELSnB:ELSnA = 1:0): Channel output is forced high
                 *   at the counter overflow (when the zero is loaded into the TPM
                 *   counter), and it is forced low at channel match
                 *   (TPM counter = CnV)
                 */
                .tpm_mmio_CnSC_value = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK
            },
            [1] = {
                .tpm_mmio_pcr_p = &PORTC_PCR2, /* Motor B (H-Bridge B - 2) */

                /*
                 * - Use (MSnB:MSnA = 1:0): Edge-aligned PWM mode.
                 * - Use (ELSnB:ELSnA = X:1): Channel output is forced low
                 *   at the counter overflow (when zero is loaded into the TPM
                 *   counter), and it is forced high at channel match
                 *   (TPM counter = CnV).
                 */
                .tpm_mmio_CnSC_value = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK
            },
            [2] = {
                .tpm_mmio_pcr_p = &PORTC_PCR3, /* Motor A (H-Bridge A - 1) */
                .tpm_mmio_CnSC_value = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK
            },
            [3] = {
                .tpm_mmio_pcr_p = &PORTC_PCR4, /* Motor A (H-Bridge A - 2)  */
                 /* invert the second PWM signal for a complimentary output */
                .tpm_mmio_CnSC_value = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK
            },
            [4] = {
                .tpm_mmio_pcr_p = NULL,
            },
            [5] = {
                .tpm_mmio_pcr_p = NULL,
            },

        },
        .tpm_mmio_pin_mux_selector_mask = PORT_PCR_MUX(0x4),
        .tpm_mmio_clock_gate_mask = SIM_SCGC6_TPM0_MASK,
        .tpm_clock_freq_hz = CPU_CLOCK_FREQ_IN_HZ / 2,
        .tpm_clock_prescale = 0, /* Divide by 1 */
        .tpm_overflow_freq_hz = TFC_WHEEL_MOTOR_TPM_OVERFLOW_FREQ_HZ,
        .tpm_initial_duty_cycle_us = TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US,
        .tpm_rtos_interrupt_params = {
            .irp_name_p = "TPM0 Interrupt",
            .irp_isr_function_p = kl25_tpm0_isr,
            .irp_arg_p =  (void *)&g_tpm_devices[0],
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_TPM0),
            .irp_priority = TPM_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

        .tpm_rtos_interrupt_pp = &g_rtos_interrupt_tpm0_p,
        .tpm_condvar_name = "TPM0 condvar",
    },

    [1] = {
        .tpm_signature = TPM_DEVICE_SIGNATURE,
        .tpm_var_p = &g_tpm_devices_var[1],
        .tpm_name_p = "Steering servo TPM",
        .tpm_mmio_p = TPM1_BASE_PTR,
        .tpm_wait_pwm_cycle_completion = true,
        .tpm_channels = {
            [0] = {
                .tpm_mmio_pcr_p = &PORTB_PCR0, /* Servo Channel 0 */
                .tpm_mmio_CnSC_value = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK
            },
#           if 0
            [1] = {
                .tpm_mmio_pcr_p = &PORTB_PCR1, /* Servo Channel 1 */
                .tpm_mmio_CnSC_value = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK
            },
#           else
            [1] = {
                .tpm_mmio_pcr_p = NULL,
            },
#           endif

            [2] = {
                .tpm_mmio_pcr_p = NULL,
            },
            [3] = {
                .tpm_mmio_pcr_p = NULL,
            },
            [4] = {
                .tpm_mmio_pcr_p = NULL,
            },
            [5] = {
                .tpm_mmio_pcr_p = NULL,
            },
        },
        .tpm_mmio_pin_mux_selector_mask = PORT_PCR_MUX(0x3),
        .tpm_mmio_clock_gate_mask = SIM_SCGC6_TPM1_MASK,
        .tpm_clock_freq_hz = CPU_CLOCK_FREQ_IN_HZ / 2,
        .tpm_clock_prescale = 6, /* Divide by 64 */
        .tpm_overflow_freq_hz = TFC_STEERING_SERVO_TPM_OVERFLOW_FREQ_HZ,
        .tpm_initial_duty_cycle_us = TFC_STEERING_SERVO_OFF_DUTY_CYCLE_US,
        .tpm_rtos_interrupt_params = {
            .irp_name_p = "TPM1 Interrupt",
            .irp_isr_function_p = kl25_tpm1_isr,
            .irp_arg_p =  (void *)&g_tpm_devices[1],
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_TPM1),
            .irp_priority = TPM_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

        .tpm_rtos_interrupt_pp = &g_rtos_interrupt_tpm1_p,
        .tpm_condvar_name = "TPM1 condvar",
    },

    [2] = {
        .tpm_signature = TPM_DEVICE_SIGNATURE,
        .tpm_var_p = &g_tpm_devices_var[2],
        .tpm_mmio_p = TPM2_BASE_PTR,
        .tpm_mmio_clock_gate_mask = SIM_SCGC6_TPM2_MASK,
    },
};

C_ASSERT(ARRAY_SIZE(g_tpm_devices) == ARRAY_SIZE(g_tpm_devices_var));

/**
 * PWM device for TFC wheel motors
 */
static const struct tpm_device *g_tfc_motors_pwm_device_p = &g_tpm_devices[0];

/**
 * PWM device for TFC steering servo
 */
static const struct tpm_device *g_tfc_steering_servo_pwm_device_p = &g_tpm_devices[1];

/*
 *  A/D converter
 */

/**
 * McRTOS interrupt object for the A/D converter interrupts
 */
struct rtos_interrupt *g_rtos_interrupt_adc0_p = NULL;

/**
 * Global non-const structure for A/D converter device
 * (allocated in SRAM space)
 */
static struct adc_device_var g_adc_device_var = {
    .ad_initialized = false,
    .ad_adc_channels = {
        [TFC_BATTERY_SENSE_CHANNEL] = {
            .adc_mux_selector = ADC_MUX_SIDE_B
        },

        [TFC_LINESCAN_0_ADC_CHANNEL] = {
            .adc_mux_selector = ADC_MUX_SIDE_B
        },

        [TFC_LINESCAN_1_ADC_CHANNEL] = {
            .adc_mux_selector = ADC_MUX_SIDE_B
        },

        [TFC_POT_1_ADC_CHANNEL] = {
            .adc_mux_selector = ADC_MUX_SIDE_A
        },

        [TFC_POT_0_ADC_CHANNEL] = {
            .adc_mux_selector = ADC_MUX_SIDE_A
        },
    }

};

/**
 * Global const structure for the A/D converter device
 * (allocated in flash space)
 */
static const struct adc_device g_adc0_device = {
    .ad_signature = ADC_DEVICE_SIGNATURE,
    .ad_var_p = &g_adc_device_var,
    .ad_mmio_registers_p = ADC0_BASE_PTR,
    .ad_rtos_interrupt_params = {
        .irp_name_p = "ADC Interrupt",
        .irp_isr_function_p = kl25_adc0_isr,
        .irp_arg_p = (void *)&g_adc0_device,
        .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ADC0),
        .irp_priority = ADC_INTERRUPT_PRIORITY,
        .irp_cpu_id = 0,
    },

    .ad_rtos_interrupt_pp = &g_rtos_interrupt_adc0_p,

    .ad_mutex_name = "ADC0 mutex",
    .ad_channel_condvar_name = "ADC0 channel condvar",
};

const struct adc_device *const g_adc0_device_p = &g_adc0_device;

/**
 * TFC H-Bridge Enable pin for wheel motors (KL25's pin PTE21)
 */
static struct gpio_pin g_tfc_hbridge_enable_pin =
    GPIO_PIN_INITIALIZER(
        PIN_PORT_E,
        TFC_HBRIDGE_ENABLE_PIN_INDEX,
        1,
        true);

/**
 * TFC H-Bridge Fault pin for wheel motors (KL25's pin PTE20)
 */
static struct gpio_pin g_tfc_hbridge_fault_pin =
    GPIO_PIN_INITIALIZER(
        PIN_PORT_E,
        TFC_HBRIDGE_FAULT_PIN_INDEX,
        1,
        true);

/**
 * TFC camera SI pin (KL25's pin PTD7)
 */
static struct gpio_pin g_tfc_camera_si_pin =
    GPIO_PIN_INITIALIZER(
        PIN_PORT_D,
        TFC_CAMERA_SI_PIN_INDEX,
        1,
        true);

/**
 * TFC camera CLK pin (KL25's pin PTE1)
 */
static struct gpio_pin g_tfc_camera_clk_pin =
    GPIO_PIN_INITIALIZER(
        PIN_PORT_E,
        TFC_CAMERA_CLK_PIN_INDEX,
        1,
        true);

/**
 * TFC camera AO0 pin (KL25's pin PTD5)
 */
static struct gpio_pin g_tfc_camera_ao0_pin =
    GPIO_PIN_INITIALIZER(
        PIN_PORT_D,
        TFC_CAMERA_AO0_PIN_INDEX,
        0,
        true);

#if 0
/**
 * TFC camera AO1 pin (KL25's pin PTD6)
 */
static struct gpio_pin g_tfc_camera_ao1_pin =
    GPIO_PIN_INITIALIZER(
        PIN_PORT_D,
        TFC_CAMERA_AO1_PIN_INDEX,
        0,
        true);
#endif

/**
 * TFC Battery LEDs pins (KL25's pins PTB8 - PTB11)
 */
static struct gpio_pin g_tfc_battery_led_pins[] = {
    [0] = GPIO_PIN_INITIALIZER(
            PIN_PORT_B,
            TFC_BATTERY_LEDS_FIRST_PIN_INDEX,
            1,
            true),
    [1] = GPIO_PIN_INITIALIZER(
            PIN_PORT_B,
            TFC_BATTERY_LEDS_FIRST_PIN_INDEX + 1,
            1,
            true),
    [2] = GPIO_PIN_INITIALIZER(
            PIN_PORT_B,
            TFC_BATTERY_LEDS_FIRST_PIN_INDEX + 2,
            1,
            true),
    [3] = GPIO_PIN_INITIALIZER(
            PIN_PORT_B,
            TFC_BATTERY_LEDS_FIRST_PIN_INDEX + 3,
            1,
            true),
};

C_ASSERT(ARRAY_SIZE(g_tfc_battery_led_pins) == TFC_NUM_BATTERY_LEDS);

/**
 * TFC DIP switches pins (KL25's pins PTE2 - PTE5)
 */
static struct gpio_pin g_tfc_dip_switch_pins[] = {
    [0] = GPIO_PIN_INITIALIZER(
            PIN_PORT_E,
            TFC_DIP_SWITCHES_FIRST_PIN_INDEX,
            1,
            true),
    [1] = GPIO_PIN_INITIALIZER(
            PIN_PORT_E,
            TFC_DIP_SWITCHES_FIRST_PIN_INDEX + 1,
            1,
            true),
    [2] = GPIO_PIN_INITIALIZER(
            PIN_PORT_E,
            TFC_DIP_SWITCHES_FIRST_PIN_INDEX + 2,
            1,
            true),
    [3] = GPIO_PIN_INITIALIZER(
            PIN_PORT_E,
            TFC_DIP_SWITCHES_FIRST_PIN_INDEX + 3,
            1,
            true),
};

C_ASSERT(ARRAY_SIZE(g_tfc_dip_switch_pins) == TFC_NUM_DIP_SWITCHES);

/**
 * TFC push buttons pins (KL25's pins PTC13, PTC17)
 */
static struct gpio_pin g_tfc_push_button_pins[] = {
    [0] = GPIO_PIN_INITIALIZER(
            PIN_PORT_C,
            TFC_PUSH_BUTTON_SW1_PIN_INDEX,
            1,
            true),
    [1] = GPIO_PIN_INITIALIZER(
            PIN_PORT_C,
            TFC_PUSH_BUTTON_SW2_PIN_INDEX,
            1,
            true),
};

C_ASSERT(ARRAY_SIZE(g_tfc_push_button_pins) == TFC_NUM_PUSH_BUTTONS);

void
tfc_board_init(void)
{
    init_adc(g_adc0_device_p);
    tfc_steering_servo_init();
    tfc_wheel_motors_init();
    tfc_camera_init();
    tfc_trimpots_init();
    tfc_push_buttons_init();
    tfc_dip_switches_init();
    tfc_battery_sensor_init();
}


void
tfc_board_stop(void)
{
    tfc_steering_servo_set(
        TFC_STEERING_SERVO_OFF_DUTY_CYCLE_US);
    tfc_wheel_motors_set(
        TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US,
        TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US);
}


static void
tfc_steering_servo_init(void)
{
    kl25_tpm_init(g_tfc_steering_servo_pwm_device_p);
}


void
tfc_steering_servo_set(
    pwm_duty_cycle_us_t pwm_duty_cycle_us)
{
    FDC_ASSERT(
        (pwm_duty_cycle_us >= TFC_STEERING_SERVO_MIN_DUTY_CYCLE_US &&
         pwm_duty_cycle_us <= TFC_STEERING_SERVO_MAX_DUTY_CYCLE_US) ||
        pwm_duty_cycle_us == TFC_STEERING_SERVO_OFF_DUTY_CYCLE_US,
        pwm_duty_cycle_us, 0);

    kl25_tpm_set_duty_cycle(
        g_tfc_steering_servo_pwm_device_p,
        0,
        pwm_duty_cycle_us);

#if 0 // ???
    if (pwm_duty_cycle_us != TFC_STEERING_SERVO_OFF_DUTY_CYCLE_US) {
            rtos_thread_delay(1000 / TFC_STEERING_SERVO_TPM_OVERFLOW_FREQ_HZ);
    }
#endif
}


static void
tfc_wheel_motors_init(void)
{
    /*
     * Setup GPIO pins for wheel motor signals:
     */
    configure_gpio_pin(&g_tfc_hbridge_enable_pin, PORT_PCR_DSE_MASK, true);
    configure_gpio_pin(&g_tfc_hbridge_fault_pin, 0, false);

    /*
     * Disable TFC H-Bridge:
     */
    deactivate_output_pin(&g_tfc_hbridge_enable_pin);

    kl25_tpm_init(g_tfc_motors_pwm_device_p);
}


void
tfc_wheel_motors_set(
    pwm_duty_cycle_us_t left_wheel_pwm_duty_cycle_us,
    pwm_duty_cycle_us_t right_wheel_pwm_duty_cycle_us)
{
    static bool h_bridge_enabled = false;

    FDC_ASSERT(
        left_wheel_pwm_duty_cycle_us <= TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US &&
        right_wheel_pwm_duty_cycle_us <= TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US,
        left_wheel_pwm_duty_cycle_us, right_wheel_pwm_duty_cycle_us);

    if (!h_bridge_enabled) {
        activate_output_pin(&g_tfc_hbridge_enable_pin);
        h_bridge_enabled = true;
    }

    kl25_tpm_set_duty_cycle(
        g_tfc_motors_pwm_device_p,
        0,
        right_wheel_pwm_duty_cycle_us);

    kl25_tpm_set_duty_cycle(
        g_tfc_motors_pwm_device_p,
        1,
        right_wheel_pwm_duty_cycle_us);

    kl25_tpm_set_duty_cycle(
        g_tfc_motors_pwm_device_p,
        2,
        left_wheel_pwm_duty_cycle_us);

    kl25_tpm_set_duty_cycle(
        g_tfc_motors_pwm_device_p,
        3,
        left_wheel_pwm_duty_cycle_us);

    if (left_wheel_pwm_duty_cycle_us == TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US &&
        right_wheel_pwm_duty_cycle_us == TFC_WHEEL_MOTOR_STOPPED_DUTY_CYCLE_US) {
        deactivate_output_pin(&g_tfc_hbridge_enable_pin);
        h_bridge_enabled = false;
    }
}


static void
tfc_camera_init(void)
{
    /*
     * Setup GPIO pins for camera signals:
     */
    configure_gpio_pin(&g_tfc_camera_si_pin, PORT_PCR_DSE_MASK, true);
    configure_gpio_pin(&g_tfc_camera_clk_pin, PORT_PCR_DSE_MASK, true);
    configure_gpio_pin(&g_tfc_camera_ao0_pin, PORT_PCR_DSE_MASK, false);

#   if 0
    configure_gpio_pin(&g_tfc_camera_ao1_pin, 0, false);
#   endif

    deactivate_output_pin(&g_tfc_camera_si_pin);
    deactivate_output_pin(&g_tfc_camera_clk_pin);
}


void
tfc_camera_read_frame(
    _OUT_ struct tfc_camera_frame *camera_frame_p)
{
    activate_output_pin(&g_tfc_camera_si_pin);
    delay_loop(TFC_CAMERA_CLK_PULSE_DELAY / 2);
    activate_output_pin(&g_tfc_camera_clk_pin);
    delay_loop(TFC_CAMERA_CLK_PULSE_DELAY / 2);
    deactivate_output_pin(&g_tfc_camera_si_pin);
    delay_loop(TFC_CAMERA_CLK_PULSE_DELAY / 2);

    for (int i = 0; i < TFC_NUM_CAMERA_PIXELS; i++) {
        tfc_camera_raw_pixel_t raw_pixel =
            read_adc_channel(g_adc0_device_p, TFC_LINESCAN_0_ADC_CHANNEL);

        if (camera_frame_p != NULL) {
            camera_frame_p->cf_pixels[i] = raw_pixel;
        }

        deactivate_output_pin(&g_tfc_camera_clk_pin);
        delay_loop(TFC_CAMERA_CLK_PULSE_DELAY);
        activate_output_pin(&g_tfc_camera_clk_pin);
        delay_loop(TFC_CAMERA_CLK_PULSE_DELAY);
    }

    /*
     * Complete N+1 clock cycle
     */
    delay_loop(TFC_CAMERA_CLK_PULSE_DELAY / 4);
    deactivate_output_pin(&g_tfc_camera_clk_pin);
    delay_loop(TFC_CAMERA_CLK_PULSE_DELAY);
}


static void
tfc_trimpots_init(void)
{

}


void
tfc_trimpots_read(
    _OUT_ tfc_trimpot_reading_t trimpot_readings[])
{
    trimpot_readings[0] = read_adc_channel(g_adc0_device_p, TFC_POT_0_ADC_CHANNEL);
    trimpot_readings[1] = read_adc_channel(g_adc0_device_p, TFC_POT_1_ADC_CHANNEL);
}


static void
tfc_battery_sensor_init(void)
{
    /*
     * Initialize battery level LEDs:
     */
    for (int i = 0; i < TFC_NUM_BATTERY_LEDS; i++) {
        configure_gpio_pin(&g_tfc_battery_led_pins[i], PORT_PCR_DSE_MASK, true);
        deactivate_output_pin(&g_tfc_battery_led_pins[i]);
    }

#if 0 // ???
    configure_gpio_pin(&g_tfc_battery_sensor_pin, 0, false);
#endif
}


tfc_battery_reading_t
tfc_battery_sensor_read(void)
{
    return read_adc_channel(g_adc0_device_p, TFC_BATTERY_SENSE_CHANNEL);
}


void
tfc_battery_leds_set(natural_t battery_level)
{
    natural_t i;

    FDC_ASSERT(battery_level <= TFC_NUM_BATTERY_LEDS, battery_level, 0);

    for (i = 0; i < battery_level; i++) {
        activate_output_pin(&g_tfc_battery_led_pins[i]);
    }

    for ( ; i < TFC_NUM_BATTERY_LEDS; i++) {
        deactivate_output_pin(&g_tfc_battery_led_pins[i]);
    }
}


static void
tfc_push_buttons_init(void)
{
    for (int i = 0; i < TFC_NUM_PUSH_BUTTONS; i++) {
        configure_gpio_pin(&g_tfc_push_button_pins[i], 0, false);
    }
}


void
tfc_push_buttons_read(
        _OUT_ bool push_buttons[])
{
    for (int i = 0; i < TFC_NUM_PUSH_BUTTONS; i++) {
        push_buttons[i] = read_input_pin(&g_tfc_push_button_pins[i]);
    }
}


static void
tfc_dip_switches_init(void)
{
    for (int i = 0; i < TFC_NUM_DIP_SWITCHES; i++) {
        configure_gpio_pin(&g_tfc_dip_switch_pins[i], 0, false);
    }
}


void
tfc_dip_switches_read(
        _OUT_ bool dip_switches[])
{
    for (int i = 0; i < TFC_NUM_DIP_SWITCHES; i++) {
        dip_switches[i] = read_input_pin(&g_tfc_dip_switch_pins[i]);
    }
}


