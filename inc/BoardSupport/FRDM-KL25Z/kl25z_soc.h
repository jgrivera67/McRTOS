/**
 * @file kl25z_soc.h
 *
 * Freescale KL25Z SOC declarations
 *
 * @author German Rivera 
 */ 

#ifndef __KL25Z_SOC_H
#define __KL25Z_SOC_H

#include "kl25z_soc_public.h"
#include "McRTOS_kernel_services.h"

struct rtos_interrupt;  /* opaque type */

/**
 * Const fields of a KL25 TPM device (to be placed in flash)
 * 
 * NOTE: struct tpm_device implements the abstract struct pwm_device.
 */
struct tpm_device {
#   define TPM_DEVICE_SIGNATURE  GEN_SIGNATURE('T', 'P', 'M', 'x')
    uint32_t tpm_signature;
    struct tpm_device_var *tpm_var_p;
    const char *tpm_name_p;
    TPM_MemMapPtr tpm_mmio_p;
    bool tpm_wait_pwm_cycle_completion;
    struct tpm_channel {
        volatile uint32_t *tpm_mmio_pcr_p;
        uint32_t tpm_mmio_CnSC_value;
    } tpm_channels[PWM_MAX_NUM_CHANNELS];

    uint32_t tpm_mmio_pin_mux_selector_mask;
    uint32_t tpm_mmio_clock_gate_mask;
    uint32_t tpm_clock_freq_hz;
    uint16_t tpm_clock_prescale; 
    uint16_t tpm_overflow_freq_hz;
    uint32_t tpm_initial_duty_cycle_us;
    struct rtos_interrupt_registration_params tpm_rtos_interrupt_params;
    struct rtos_interrupt **tpm_rtos_interrupt_pp;
    const char *tpm_condvar_name;
};

/**
 * Non-const fields of a KL25 TPM device (to be placed in SRAM)
 */
struct tpm_device_var {
    /**
     * Flag indicating if kl25_tpm_init() has been called for this TPM device
     */
    bool tpm_initialized;

    /**
     * Flag set when a pwm cycle has completed
     */
    bool tpm_pwm_cycle_completed;

    /**
     * cached copy of the TPM_MOD reg value
     */
    uint32_t tpm_mod_reg_value;

    /**
     * Condvar to signal a thread waiting in kl25_tpm_set_duty_cycle()
     */
    struct rtos_condvar tpm_condvar;
};

/**
 * Number of A/D converter channels
 */
#define NUM_ADC_CHANNELS    14 

#define ADC_CHANNEL_NONE    NUM_ADC_CHANNELS   

/**
 * ADC_CFG1 register MODE field
 * 0x0 - single-ended 8-bit conversion 
 * 0x1 - single-ended 12-bit conversion 
 * 0x2 - single-ended 10-bit conversion 
 * 0x3 - single-ended 16-bit conversion 
 */
#if ADC_RESOLUTION == 8
#   define ADC_CFG1_MODE_VALUE   0x0
#elif ADC_RESOLUTION == 12
#   define ADC_CFG1_MODE_VALUE   0x1
#elif ADC_RESOLUTION == 10
#   define ADC_CFG1_MODE_VALUE   0x2
#elif ADC_RESOLUTION == 16
#   define ADC_CFG1_MODE_VALUE   0x3
#else
#   error "ADC_RESOLUTION value not supported"
#endif

#define ADC_SC1A_REG(_adc_mmio_registers_p) \
        ADC_SC1_REG(_adc_mmio_registers_p, 0)

#define ADC_RA_REG(_adc_mmio_registers_p) \
        ADC_R_REG(_adc_mmio_registers_p, 0)

/**
 * Const fields of the A/D converter device (to be placed in flash)
 */
struct adc_device {
#   define ADC_DEVICE_SIGNATURE  GEN_SIGNATURE('A', '/', 'D', 'C')
    uint32_t ad_signature;
    struct adc_device_var *ad_var_p;
    ADC_MemMapPtr ad_mmio_registers_p;
    struct rtos_interrupt_registration_params ad_rtos_interrupt_params;
    struct rtos_interrupt **ad_rtos_interrupt_pp;
    const char *ad_mutex_name;
    const char *ad_channel_condvar_name;
};

/**
 * Non-const fields of an A/D converter channel (to be placed in SRAM)
 */
struct adc_channel {
    /**
     * Last value read from the V/VREF field of the corresponding ADC channel's
     * reg_AD0DR[] register, by the A/D conversion completion interrupt handler.
     */
    volatile uint16_t adc_result;

    /**
     * Boolean flag that indicates if an outstanding A/D conversion has completed
     */
    volatile uint8_t adc_conversion_completed;

    /**
     * ADC Mux selector
     */ 
    const uint8_t adc_mux_selector;
#   define ADC_MUX_SIDE_A   0
#   define ADC_MUX_SIDE_B   1

    /**
     * Condvar to signal a thread waiting for an A/D conversion
     */
    struct rtos_condvar adc_condvar;
};

/**
 * Non-const fields of a ADC device (to be placed in SRAM)
 */
struct adc_device_var {
    /**
     * Flag idicating if init_adc() has been called for this ADC device
     */
    bool ad_initialized;

    /**
     * A/D converter channel on which the current conversion was started
     * or ADC_CHANNEL_NONE if none.
     */
    uint8_t ad_active_adc_channel;

    /**
     * Mutex to serialize software-triggered conversions.
     *
     * NOTE: For the KL25 ADC, only one software-triggered
     * conversion can be done at one time, regardless of 
     * using different channels.
     */
    struct rtos_mutex ad_mutex;

    /**
     * A/D converter channels
     */
    struct adc_channel ad_adc_channels[NUM_ADC_CHANNELS];
};


/**
 * Const fields of the I2C controller device (to be placed in flash)
 */
struct i2c_device {
#   define I2C_DEVICE_SIGNATURE  GEN_SIGNATURE('I', '2', 'C', 'C')
    uint32_t i2c_signature;
    struct i2c_device_var *i2c_var_p;
    I2C_MemMapPtr i2c_mmio_registers_p;
    volatile uint32_t *i2c_mmio_scl_port_pcr_p;
    volatile uint32_t *i2c_mmio_sda_port_pcr_p;
    uint32_t i2c_clock_gate_mask;
    uint32_t i2c_pin_mux_selector_mask;
    uint8_t i2c_icr_value;
    struct rtos_interrupt_registration_params i2c_rtos_interrupt_params;
    struct rtos_interrupt **i2c_rtos_interrupt_pp;
    const char *i2c_condvar_name;
};

/**
 * Non-const fields of an I2C controller device (to be placed in SRAM)
 */
struct i2c_device_var {
    /**
     * Flag indicating if init_i2c() has been called for this I2C device
     */
    bool i2c_initialized;

    /**
     * Flag set when a byte transfer has completed
     */
    volatile bool i2c_byte_transfer_completed;

    /**
     * Condvar to signal a thread waiting for an I2C byte transfer
     */
    struct rtos_condvar i2c_condvar;
};


void kl25_tpm_init(
    const struct tpm_device *tpm_device_p);

void kl25_tpm_set_duty_cycle(
    const struct tpm_device *tpm_device_p,
    pwm_channel_t pwm_channel,
    pwm_duty_cycle_us_t pwm_duty_cycle_us);

extern isr_function_t kl25_uart0_isr;

extern isr_function_t kl25_adc0_isr;

extern isr_function_t kl25_tpm0_isr;

extern isr_function_t kl25_tpm1_isr;

extern isr_function_t kl25_i2c0_isr;

void kl25_uart_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void kl25_adc_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void kl25_tpm_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void
kl25_i2c_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

#endif /* __KL25Z_SOC_H */
