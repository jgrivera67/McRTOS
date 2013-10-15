/**
 * @file tfc_board__hardware_abstractions.c
 *
 * Hardware abstraction layer for the TFC add-on board
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
#include "tfc_board.h"

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-function"

static void tfc_gpio_init(void);
static void tfc_steering_servo_init(void);
static void tfc_wheel_motors_init(void);

/**
 * Pointer to the McRTOS interrupt objects for the TPM interrupts.
 */
struct rtos_interrupt *g_rtos_interrupt_tpm0_p = NULL;
struct rtos_interrupt *g_rtos_interrupt_tpm1_p = NULL;

/**
 * Global array of const structures for TPM devices for the KL25Z SoC
 * (allocated in flash space)
 */
const struct tpm_device g_tpm_devices[] =
{
    [0] = {
        .tpm_signature = TPM_DEVICE_SIGNATURE,
        .tpm_name_p = "Wheel motors TPM",
        .tpm_mmio_p = TPM0_BASE_PTR,
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
    },

    [1] = {
        .tpm_signature = TPM_DEVICE_SIGNATURE,
        .tpm_name_p = "Steering servo TPM",
        .tpm_mmio_p = TPM1_BASE_PTR,
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
    },

    [2] = {
        .tpm_signature = TPM_DEVICE_SIGNATURE,
        .tpm_mmio_p = TPM2_BASE_PTR,
        .tpm_mmio_clock_gate_mask = SIM_SCGC6_TPM2_MASK,
    },
};

/**
 * PWM device for TFC wheel motors
 */
static const struct tpm_device *g_tfc_motors_pwm_device_p = &g_tpm_devices[0];

/**
 * PWM device for TFC steering servo
 */
static const struct tpm_device *g_tfc_steering_servo_pwm_device_p = &g_tpm_devices[1];

void
tfc_board_init(void)
{
    tfc_gpio_init();
    tfc_steering_servo_init();
    tfc_wheel_motors_init();
}

/* 
 * Set I/O for H-BRIDGE enables, switches and LEDs for the TFC add-on board
 */ 
static void
tfc_gpio_init(void)
{
        uint32_t reg_value;

	/*
         * Setup Pins as GPIO
         */
	write_32bit_mmio_register(
            &PORTE_PCR21, PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);

        write_32bit_mmio_register(
	    &PORTE_PCR20, PORT_PCR_MUX(1));
	
	/*
         * Port for Pushbuttons
         */ 
        write_32bit_mmio_register(
	    &PORTC_PCR13, PORT_PCR_MUX(1));

        write_32bit_mmio_register(
            &PORTC_PCR17, PORT_PCR_MUX(1));
	
	/*
         * Ports for DIP Switches
         */ 
        write_32bit_mmio_register(
	    &PORTE_PCR2, PORT_PCR_MUX(1)); 

        write_32bit_mmio_register(
	    &PORTE_PCR3, PORT_PCR_MUX(1));

        write_32bit_mmio_register(
	    &PORTE_PCR4, PORT_PCR_MUX(1));

        write_32bit_mmio_register(
	    &PORTE_PCR5, PORT_PCR_MUX(1));
	
	/*
         * Ports for LEDs
         */ 
        write_32bit_mmio_register(
	    &PORTB_PCR8, PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);

        write_32bit_mmio_register(
	    &PORTB_PCR9, PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);

        write_32bit_mmio_register(
	    &PORTB_PCR10, PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);

        write_32bit_mmio_register(
	    &PORTB_PCR11, PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);   
	
	/*
         * Setup the output pins
         */
        reg_value = read_32bit_mmio_register(&GPIOE_PDDR);
	reg_value |= TFC_HBRIDGE_EN_LOC;
        write_32bit_mmio_register(&GPIOE_PDDR, reg_value);

        reg_value = read_32bit_mmio_register(&GPIOB_PDDR);
	reg_value |= TFC_BAT_LED0_LOC | TFC_BAT_LED1_LOC | TFC_BAT_LED2_LOC |
                     TFC_BAT_LED3_LOC;
        write_32bit_mmio_register(&GPIOB_PDDR, reg_value);

        /*
         * TFC_HBRIDGE_DISABLE:
         */ 
        write_32bit_mmio_register(
            &GPIOE_PCOR, TFC_HBRIDGE_EN_LOC);
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
        pwm_duty_cycle_us >= TFC_STEERING_SERVO_MIN_DUTY_CYCLE_US &&
        pwm_duty_cycle_us <= TFC_STEERING_SERVO_MAX_DUTY_CYCLE_US,
        pwm_duty_cycle_us, 0);

    kl25_tpm_set_duty_cycle(
        g_tfc_steering_servo_pwm_device_p,
        0,
        pwm_duty_cycle_us);
}


static void
tfc_wheel_motors_init(void)
{
    kl25_tpm_init(g_tfc_motors_pwm_device_p);
}


void
tfc_wheel_motors_set(
    pwm_duty_cycle_us_t left_wheel_pwm_duty_cycle_us,
    pwm_duty_cycle_us_t right_wheel_pwm_duty_cycle_us)
{
    FDC_ASSERT(
        left_wheel_pwm_duty_cycle_us <= TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US &&
        right_wheel_pwm_duty_cycle_us <= TFC_WHEEL_MOTOR_MAX_DUTY_CYCLE_US,
        left_wheel_pwm_duty_cycle_us, right_wheel_pwm_duty_cycle_us);

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
}





