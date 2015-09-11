/**
 * @file k64f_soc.h
 *
 * Freescale K64F SOC declarations
 *
 * @author German Rivera
 */

#ifndef __K64F_SOC_H
#define __K64F_SOC_H

#include <BoardSupport/FRDM-K64F/k64f_soc_public.h>
#include <McRTOS/McRTOS_kernel_services.h>

struct rtos_interrupt;  /* opaque type */

/**
 * Const fields of a K64F TPM device (to be placed in flash)
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
 * Const fields of the CRC device (to be placed in flash)
 */
struct crc_device {
#   define CRC_DEVICE_SIGNATURE  GEN_SIGNATURE('C', 'R', 'C', ' ')
    uint32_t signature;
    volatile CRC_Type *mmio_regs_p;
    struct crc_device_var *var_p;
};

/**
 * Non-const fields of an CRC device (to be placed in SRAM)
 */
struct crc_device_var {
    bool initialized;
};

/**
 * Const fields of a UART device (to be placed in flash)
 */
struct uart_device {
#   define UART_DEVICE_SIGNATURE  GEN_SIGNATURE('U', 'A', 'R', 'T')
    uint32_t urt_signature;
    const char *urt_name;
    struct uart_device_var *urt_var_p;
    UART_MemMapPtr urt_mmio_uart_p;
    struct pin_info urt_tx_pin;
    struct pin_info urt_rx_pin;
    volatile uint32_t *urt_mmio_clock_gate_reg_p;
    uint32_t urt_mmio_clock_gate_mask;
    uint32_t urt_source_clock_freq_in_hz;

    struct rtos_interrupt_registration_params urt_rtos_interrupt_rx_tx_params;
    struct rtos_interrupt **urt_rtos_interrupt_rx_tx_pp;
    struct rtos_interrupt_registration_params urt_rtos_interrupt_err_params;
    struct rtos_interrupt **urt_rtos_interrupt_err_pp;
    const char *urt_transmit_queue_name_p;
    const char *urt_receive_queue_name_p;
    uint8_t *urt_transmit_queue_storage_p;
    uint8_t *urt_receive_queue_storage_p;
};

/**
 * Non-const fields of a UART device (to be placed in SRAM)
 */
struct uart_device_var {
    bool urt_initialized;
    uint32_t urt_received_bytes_dropped;
    uint32_t urt_transmit_bytes_dropped;
    struct rtos_circular_buffer urt_transmit_queue;
    struct rtos_circular_buffer urt_receive_queue;
    uint8_t urt_tx_fifo_size;
    uint8_t urt_rx_fifo_size;
    bool urt_fifos_enabled;
};

/**
 * Const fields of a K64F FTM device (to be placed in flash)
 */
struct ftm_device {
#   define FTM_DEVICE_SIGNATURE  GEN_SIGNATURE('F', 'T', 'M', 'x')
    uint32_t ftm_signature;
    struct ftm_device_var *ftm_var_p;
    const char *ftm_name_p;
    FTM_MemMapPtr ftm_mmio_p;
    bool ftm_wait_pwm_cycle_completion;
    struct ftm_channel {
        volatile uint32_t *ftm_mmio_pcr_p;
        uint32_t ftm_mmio_CnSC_value;
    } ftm_channels[PWM_MAX_NUM_CHANNELS];

    uint32_t ftm_mmio_pin_mux_selector_mask;
    uint32_t ftm_mmio_clock_gate_mask;
    uint32_t ftm_clock_freq_hz;
    uint16_t ftm_clock_prescale;
    uint16_t ftm_overflow_freq_hz;
    uint32_t ftm_initial_duty_cycle_us;
    struct rtos_interrupt_registration_params ftm_rtos_interrupt_params;
    struct rtos_interrupt **ftm_rtos_interrupt_pp;
    const char *ftm_condvar_name;
};

/**
 * Non-const fields of a K64F FTM device (to be placed in SRAM)
 */
struct ftm_device_var {
    /**
     * Flag indicating if k64f_ftm_init() has been called for this FTM device
     */
    bool ftm_initialized;

    /**
     * Flag set when a pwm cycle has completed
     */
    bool ftm_pwm_cycle_completed;

    /**
     * cached copy of the FTM_MOD reg value
     */
    uint32_t ftm_mod_reg_value;

    /**
     * Condvar to signal a thread waiting in k64f_ftm_set_duty_cycle()
     */
    struct rtos_condvar ftm_condvar;
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
     * NOTE: For the K64F ADC, only one software-triggered
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
    const char *i2c_name;
    struct i2c_device_var *i2c_var_p;
    I2C_MemMapPtr i2c_mmio_registers_p;
    struct pin_info i2c_scl_pin;
    struct pin_info i2c_sda_pin;
    uint32_t i2c_clock_gate_mask;
    uint8_t i2c_icr_value;
    struct rtos_interrupt_registration_params i2c_rtos_interrupt_params;
    struct rtos_interrupt **i2c_rtos_interrupt_pp;
    const char *i2c_condvar_name;
};

/**
 * States of an I2C data transfer transaction
 */
enum i2c_transaction_states {
    I2C_TRANSACTION_NOT_STARTED = 0,
    I2C_SENDING_SLAVE_ADDR,
    I2C_SENDING_SLAVE_REG_ADDR,
    I2C_SENDING_SLAVE_ADDR_FOR_RX,
    I2C_SENDING_DATA_BYTE,
    I2C_RECEIVING_DATA_BYTE,
    I2C_TRANSACTION_COMPLETED,
    I2C_TRANSACTION_ABORTED
};

/**
 * Data transfer transaction over an I2C bus
 */
struct i2c_transaction {
    enum i2c_transaction_states state;
    uint8_t flags;
#   define I2C_TRANSACTION_IS_READ_DATA     UINT8_C(0x1)
#   define I2C_TRANSACTION_HAS_REG_ADDR     UINT8_C(0x2)

    uint8_t slave_addr;
    uint8_t slave_reg_addr;
    uint8_t *next_data_byte_p;
    size_t num_data_bytes_left;
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
     * Current data transfer transaction
     */
    struct i2c_transaction transaction;

    /**
     * Condvar to signal a thread waiting for an I2C data transaction
     * to finish
     */
    struct rtos_condvar i2c_condvar;
};


void k64f_ftm_init(
    const struct ftm_device *ftm_device_p);

void k64f_ftm_set_duty_cycle(
    const struct ftm_device *ftm_device_p,
    pwm_channel_t pwm_channel,
    pwm_duty_cycle_us_t pwm_duty_cycle_us);

extern isr_function_t k64f_uart0_rx_tx_isr;

extern isr_function_t k64f_uart0_err_isr;

extern isr_function_t k64f_uart4_rx_tx_isr;

extern isr_function_t k64f_uart4_err_isr;

extern isr_function_t k64f_adc0_isr;

extern isr_function_t k64f_ftm0_isr;

extern isr_function_t k64f_ftm1_isr;

extern isr_function_t k64f_i2c0_isr;

extern isr_function_t k64f_port_c_isr;

void k64f_uart_rx_tx_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void k64f_uart_err_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void k64f_adc_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void k64f_ftm_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void
k64f_i2c_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void
k64f_port_c_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

extern const struct i2c_device g_i2c_devices[];

#endif /* __K64F_SOC_H */
