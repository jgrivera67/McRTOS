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
    const char *tpm_name_p;
    TPM_MemMapPtr tpm_mmio_p;
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

void kl25_uart_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void kl25_adc_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void kl25_tpm_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

#endif /* __KL25Z_SOC_H */
