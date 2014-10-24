/**
 * @file k64f_soc.h
 *
 * Freescale K64F SOC declarations
 *
 * @author German Rivera
 */

#ifndef __K64F_SOC_H
#define __K64F_SOC_H

#include "k64f_soc_public.h"
#include "McRTOS_kernel_services.h"

struct rtos_interrupt;  /* opaque type */

/**
 * Const fields of a MPU device
 */
struct mpu_device {
#   define MPU_DEVICE_SIGNATURE  GEN_SIGNATURE('M', 'P', 'U', ' ')
    uint32_t signature;
    volatile MPU_Type *mmio_regs_p;
    struct mpu_device_var *var_p;
};

/**
 * Non-const fields of a MPU device
 */
struct mpu_device_var {
    bool initialized;
    uint8_t num_regions;
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

/*
 * Ethernet frame buffer alignment in bytes
 */
#define ENET_FRAME_BUFFER_ALIGNMENT UINT32_C(16)

#define ENET_MAX_RX_FRAME_BUFFERS   8

#define ENET_MAX_TX_FRAME_BUFFERS   8

/**
 * Const fields of an Ethernet MAC device (to be placed in flash)
 */
struct enet_device {
#   define ENET_DEVICE_SIGNATURE  GEN_SIGNATURE('E', 'N', 'E', 'T')
    uint32_t signature;
    const char *name;
    struct enet_device_var *var_p;
    volatile ENET_Type *mmio_registers_p;
    struct pin_info rmii_mdio_pin;
    struct pin_info rmii_mdc_pin;
    struct pin_info rmii_rxd0_pin;
    struct pin_info rmii_rxd1_pin;
    struct pin_info rmii_crs_dv_pin;
    struct pin_info rmii_rxer_pin;
    struct pin_info rmii_txen_pin;
    struct pin_info rmii_txd0_pin;
    struct pin_info rmii_txd1_pin;
    struct pin_info mii_txer_pin;
    struct pin_info mii_intr_pin;
    struct rtos_interrupt_registration_params tx_rtos_interrupt_params;
    struct rtos_interrupt **tx_rtos_interrupt_pp;
    struct rtos_interrupt_registration_params rx_rtos_interrupt_params;
    struct rtos_interrupt **rx_rtos_interrupt_pp;
    struct pin_info enet_1588_tmr_pins[4];
    uint32_t clock_gate_mask;
    /* MAC address in big-endian byte order */
    uint8_t mac_address[6];
};

struct enet_rx_buffer_descriptor {
    uint16_t data_length;
    uint16_t control;
#   define  ENET_RX_BD_EMPTY_MASK			BIT(15)
#   define  ENET_RX_BD_SOFTWARE_OWNERSHIP1_MASK		BIT(14)
#   define  ENET_RX_BD_WRAP_MASK			BIT(13)
#   define  ENET_RX_BD_SOFTWARE_OWNERSHIP2_MASK		BIT(12)
#   define  ENET_RX_BD_LAST_IN_FRAME_MASK		BIT(11)
#   define  ENET_RX_BD_MISS_MASK			BIT(8)
#   define  ENET_RX_BD_BROADCAST_MASK			BIT(7)
#   define  ENET_RX_BD_MULTICAST_MASK			BIT(6)
#   define  ENET_RX_BD_LENGTH_VIOLATION_MASK		BIT(5)
#   define  ENET_RX_BD_NON_OCTET_ALIGNED_FRAME_MASK	BIT(4)
#   define  ENET_RX_BD_CRC_ERROR_MASK			BIT(2)
#   define  ENET_RX_BD_FIFO_OVERRRUN_MASK		BIT(1)
#   define  ENET_RX_BD_FRAME_TRUNCATED_MASK		BIT(0)

    void *data_buffer;
    uint16_t control_extend0;
#   define  ENET_RX_BD_VLAN_PRIORITY_CODE_POINT_MASK	MULTI_BIT_MASK(15, 13)
#   define  ENET_RX_BD_VLAN_PRIORITY_CODE_POINT_SHIFT	13
#   define  ENET_RX_BD_IP_HEADER_CHECKSUM_ERROR_MASK	BIT(5)
#   define  ENET_RX_BD_PROTOCOL_CHECKSUM_ERROR_MASK	BIT(4)
#   define  ENET_RX_BD_VLAN_FRAME_MASK			BIT(2)
#   define  ENET_RX_BD_IPv6_FRAME_MASK			BIT(1)
#   define  ENET_RX_BD_IPv4_FRAGMENT_MASK		BIT(0)

    uint16_t control_extend1;
#   define  ENET_RX_BD_GENERATE_INTERRUPT_MASK		BIT(7)
#   define  ENET_RX_BD_UNICAST_FRAME_MASK		BIT(8)
#   define  ENET_RX_BD_COLLISION_MASK			BIT(9)
#   define  ENET_RX_BD_PHY_ERROR_MASK			BIT(10)
#   define  ENET_RX_BD_MAC_ERROR_MASK			BIT(15)

    uint16_t payload_checksum;
    uint8_t header_length;
    uint8_t protocol_type;
    uint16_t reserved0;
    uint16_t control_extend2;
#   define  ENET_RX_BD_LAST_DESCRIPTOR_UPDATE_DONE_MASK	BIT(15)

    uint32_t timestamp;
    uint16_t reserved1;
    uint16_t reserved2;
    uint16_t reserved3;
    uint16_t reserved4;
} __attribute__ ((aligned(ENET_FRAME_BUFFER_ALIGNMENT)));

struct enet_tx_buffer_descriptor {
    uint16_t data_length;
    uint16_t control;
#   define  ENET_TX_BD_READY_MASK			BIT(15)
#   define  ENET_TX_BD_SOFTWARE_OWNER1_MASK		BIT(14)
#   define  ENET_TX_BD_WRAP_MASK			BIT(13)
#   define  ENET_TX_BD_SOFTWARE_OWNER2_MASK		BIT(12)
#   define  ENET_TX_BD_LAST_IN_FRAME_MASK		BIT(11)
#   define  ENET_TX_BD_CRC_MASK				BIT(10)

    void *data_buffer;
    uint16_t control_extend0;
#   define  ENET_TX_BD_ERROR_MASK			BIT(15)
#   define  ENET_TX_BD_UNDERFLOW_MASK			BIT(13)
#   define  ENET_TX_BD_EXCESS_COLLISION_ERROR_MASK	BIT(12)
#   define  ENET_TX_BD_FRAME_ERROR_MASK			BIT(11)
#   define  ENET_TX_BD_LATE_COLLISION_ERROR_MASK	BIT(10)
#   define  ENET_TX_BD_FIFO_OVERFLOW_ERROR_MASK		BIT(9)
#   define  ENET_TX_BD_TMESTAMP_ERROR_MASK		BIT(8)

    uint16_t control_extend1;
#   define  ENET_TX_BD_INTERRUPT_MASK			BIT(14)
#   define  ENET_TX_BD_TIMESTAMP_MASK			BIT(13)
#   define  ENET_TX_BD_INSERT_PROTOCOL_CHECKSUM_MASK	BIT(12)
#   define  ENET_TX_BD_INSERT_IP_HEADER_CHECKSUM_MASK	BIT(11)

    uint16_t reserved0;
    uint16_t reserved1;
    uint16_t reserved2;
    uint16_t control_extend2;
#   define  ENET_TX_BD_LAST_DESCRIPTOR_UPDATE_DONE_MASK	BIT(15)

    uint32_t timestamp;
    uint16_t reserved3;
    uint16_t reserved4;
    uint16_t reserved5;
    uint16_t reserved6;
} __attribute__ ((aligned(ENET_FRAME_BUFFER_ALIGNMENT)));

/**
 * Non-const fields of an Ethernet MAC device (to be placed in SRAM)
 */
struct enet_device_var {
    bool initialized;

    /**
     * Tx buffer descriptor ring accessed by the Ethernet MAC
     */
    struct enet_tx_buffer_descriptor tx_buffer_descriptors[ENET_MAX_TX_FRAME_BUFFERS];

    /**
     * Rx buffer descriptor ring accessed by the Ethernet MAC
     */
    struct enet_rx_buffer_descriptor rx_buffer_descriptors[ENET_MAX_RX_FRAME_BUFFERS];

    /**
     * Circular buffer of pointers used to keep track of free Tx buffers
     */
    struct rtos_circular_buffer tx_buffer_pool;

    /**
     * Circular buffer of pointers used to represent the queue of received
     * frames (non-empty Rx buffers)
     */
    struct rtos_circular_buffer rx_buffer_queue;

    /**
     * Array of entries for tx_buffer_pool
     */
    struct enet_tx_buffer_descriptor *tx_buffer_pool_entries[ENET_MAX_TX_FRAME_BUFFERS];

    /**
     * Array of entries for rx_buffer_queue
     */
    struct enet_rx_buffer_descriptor *rx_buffer_queue_entries[ENET_MAX_RX_FRAME_BUFFERS];
};

#define ENET_PHY_ADDRESS    0x0

enum enet_mmfr_op_values {
    ENET_MMFR_OP_WRITE_NON_MII_COMPLIANT_FRAME = 0x0,
    ENET_MMFR_OP_WRITE_VALID_MII_MANAGEMENT_FRAME = 0x1,
    ENET_MMFR_OP_READ_VALID_MII_MANAGEMENT_FRAME = 0x2,
    ENET_MMFR_OP_READ_NON_MII_COMPLIANT_FRAME = 0x3,
};

void k64f_ftm_init(
    const struct ftm_device *ftm_device_p);

void k64f_ftm_set_duty_cycle(
    const struct ftm_device *ftm_device_p,
    pwm_channel_t pwm_channel,
    pwm_duty_cycle_us_t pwm_duty_cycle_us);

void enet_phy_write(const struct enet_device *enet_device_p, uint32_t phy_reg,
		    uint32_t data);

uint32_t enet_phy_read(const struct enet_device *enet_device_p,
		       uint32_t phy_reg);

extern isr_function_t k64f_uart0_rx_tx_isr;

extern isr_function_t k64f_uart0_err_isr;

extern isr_function_t k64f_adc0_isr;

extern isr_function_t k64f_ftm0_isr;

extern isr_function_t k64f_ftm1_isr;

extern isr_function_t k64f_i2c0_isr;

extern isr_function_t k64f_port_c_isr;

extern isr_function_t k64f_enet_transmit_isr;

extern isr_function_t k64f_enet_receive_isr;

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

void
k64f_enet_transmit_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void
k64f_enet_receive_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

extern const struct i2c_device g_i2c_devices[];

#endif /* __K64F_SOC_H */
