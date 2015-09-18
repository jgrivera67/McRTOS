/**
 * @file hardware_abstractions.h
 *
 * Hardware abstractions to be implemented for each supported board.
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */
#ifndef _HARDWARE_ABSTRACTIONS_H
#define _HARDWARE_ABSTRACTIONS_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <McRTOS/compile_time_checks.h>
#include <BoardSupport/lcd.h>
#include <BoardSupport/touch_screen.h>

/**
 * DRAM Page size in bytes (4KB)
 */
#define DRAM_PAGE_SIZE  UINT32_C(4096)

#if defined(LPC2478_SOC)

#   include <BoardSupport/LPC2478-STK/lpc2478_stk_board_public.h>

#   define LCD_SUPPORTED
#   define DRAM_SUPPORTED

#elif defined(KL25Z_SOC)

#   include <BoardSupport/FRDM-KL25Z/kl25z_soc_public.h>
#   include <BoardSupport/FRDM-KL25Z/frdm_board.h>

#elif defined(K20D5_SOC)

#   include <BoardSupport/FRDM-K20D50k20d5_soc_public.h>

#elif defined(K64F_SOC)

#   include <BoardSupport/FRDM-K64F/k64f_soc_public.h>
#   include <BoardSupport/FRDM-K64F/frdm_board.h>

#elif defined(LM4F120_SOC)

#   include <BoardSupport/LaunchPad-LM4F120/lm4f120_soc_public.h>
#   include <BoardSupport/LaunchPad-LM4F120/launchpad_board.h>

#elif defined(LPC54102_SOC)

#   include <BoardSupport/LPC-54102/lpc54102_soc_public.h>
#   include <BoardSupport/LPC-54102/lpcxpresso_board.h>

#else

#error "No system on chip specified"

#endif /* platform-specific */

/*
 * RGB LED colors bit masks
 */
#define LED_COLOR_BLACK     UINT32_C(0x0)
#define LED_COLOR_RED       LED_RED_PIN_MASK
#define LED_COLOR_GREEN     LED_GREEN_PIN_MASK
#define LED_COLOR_YELLOW    (LED_RED_PIN_MASK | LED_GREEN_PIN_MASK)
#define LED_COLOR_BLUE      LED_BLUE_PIN_MASK
#define LED_COLOR_MAGENTA   (LED_RED_PIN_MASK | LED_BLUE_PIN_MASK)
#define LED_COLOR_CYAN      (LED_GREEN_PIN_MASK | LED_BLUE_PIN_MASK)
#define LED_COLOR_WHITE     (LED_RED_PIN_MASK | LED_GREEN_PIN_MASK | LED_BLUE_PIN_MASK)

/*
 * Pressed buttons bit masks
 */
#define BUTTON1_PRESSED_MASK    BIT(0)
#define BUTTON2_PRESSED_MASK    BIT(1)

/**
 * CPU clock frequency in Hz
 */
#define CPU_CLOCK_FREQ_IN_HZ \
        (SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ * UINT32_C(1000000))

/**
 * Convert from CPU clock cycles to nanoseconds
 */
#define CPU_CLOCK_CYCLES_TO_NANOSECONDS(_cycles) \
        ((cpu_clock_cycles_t)(_cycles) / (SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ * 1000))

/**
 * Convert from CPU clock cycles to microseconds
 */
#define CPU_CLOCK_CYCLES_TO_MICROSECONDS(_cycles) \
        ((cpu_clock_cycles_t)(_cycles) / SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ)

/**
 * Convert from CPU clock cycles to milliseconds
 */
#define CPU_CLOCK_CYCLES_TO_MILLISECONDS(_cycles) \
        (((cpu_clock_cycles_t)(_cycles) / SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ) / 1000)

/**
 * Convert from nanoseconds to CPU clock cycles
 */
#define NANOSECONDS_TO_CPU_CLOCK_CYCLES(_nano_secs) \
        ((rtos_microseconds_t)((_nano_secs) / 1000) * SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ)

/**
 * Convert from microseconds to CPU clock cycles
 */
#define MICROSECONDS_TO_CPU_CLOCK_CYCLES(_micro_secs) \
        ((rtos_microseconds_t)(_micro_secs) * SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ)

/**
 * Convert from milliseconds to CPU clock cycles
 */
#define MILLISECONDS_TO_CPU_CLOCK_CYCLES(_milli_secs) \
        ((rtos_microseconds_t)((_milli_secs) * 1000) * SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ)

/**
 * Calculate difference between two CPU clock cycle values
 */
#define CPU_CLOCK_CYCLES_DELTA(_begin_cycles, _end_cycles) \
        ((cpu_clock_cycles_t)((int32_t)(_end_cycles) - \
                              (int32_t)(_begin_cycles)))

#if __MPU_PRESENT == 1

/**
 * Minimum MPU region alignment in bytes
 */
#   define MIN_MPU_REGION_ALIGNMENT	UINT32_C(32)

/**
 * MPU region alignment in bytes for a given data type, for the
 * ARMv7-m generic MPU
 */
#   define SOC_MPU_REGION_ALIGNMENT(_type) \
           (sizeof(_type) < 32 ? UINT32_C(32) :                         \
            (sizeof(_type) < 64 ? UINT32_C(64) :                        \
             (sizeof(_type) < 128 ? UINT32_C(128) :                     \
              (sizeof(_type) < 256 ? UINT32_C(256) :                    \
               (sizeof(_type) < 512 ? UINT32_C(512) :                   \
                (sizeof(_type) < 1024 ? UINT32_C(1024) :                \
                 (sizeof(_type) < 2048 ? UINT32_C(2048) :               \
                  (sizeof(_type) < 4096 ? UINT32_C(4096) :              \
                   (sizeof(_type) < 8192 ? UINT32_C(8192) :             \
                    (sizeof(_type) < 16384 ? UINT32_C(16384) :          \
                    UINT32_MAX))))))))))

/*
 * Bit masks for unprivileged permissions
 */
#   define UNPRIVILEGED_WRITE_MASK  BIT(0)
#   define UNPRIVILEGED_READ_MASK   BIT(1)
#   define UNPRIVILEGED_EXEC_MASK   BIT(2)

/**
 * MPU bus masters
 */
enum mpu_bus_masters {
    MPU_BUS_MASTER_CPU_CORE = 0,
};

/**
 * Number of global MPU regions (besides the background region)
 */
#   define RTOS_NUM_GLOBAL_MPU_REGIONS UINT8_C(1)

#elif defined(K64F_SOC)

/**
 * Number of global MPU regions
 */
#   define RTOS_NUM_GLOBAL_MPU_REGIONS UINT8_C(4)

#else
#   error "No MPU supported"
#endif /* __MPU_PRESENT == 1 */

/**
 * Number of thread-specific MPU data regions.
 */
#define RTOS_NUM_THREAD_MPU_REGIONS   UINT8_C(3)

/**
 * Number of MPU regions required by McRTOS, including one
 * region for the stack of the current thread
  */
#define RTOS_MAX_MPU_REGIONS \
	(RTOS_NUM_GLOBAL_MPU_REGIONS + RTOS_NUM_THREAD_MPU_REGIONS)

/**
 * index for the first thread-specific MPU region
 */
#define FIRST_THREAD_MPU_REGION_INDEX   RTOS_NUM_GLOBAL_MPU_REGIONS

/**
 * Indices of thread-specific MPU regions
 */
enum rtos_thread_mpu_region_indices {
    /**
     * Thread's execution stack region
     */
    RTOS_THREAD_STACK_MPU_REGION_INDEX = FIRST_THREAD_MPU_REGION_INDEX,

    /**
     * Thread's current component data region
     * (for global data of current component called by the thread)
     */
    RTOS_THREAD_COMP_MPU_REGION_INDEX,

    /**
     * Thread's current temporary data region
     * (for temporary use, to be able to dereference input or output
     *  arguments of functions)
     */
    RTOS_THREAD_TMP_MPU_REGION_INDEX
};

/**
 * CPU identifier type
 */
typedef _RANGE_(0, SOC_NUM_CPU_CORES - 1)
        uint8_t cpu_id_t;

/*
 * Generic (Machine-independent) reset cause codes
 */
enum cpu_reset_causes {
    GRC_INVALID_RESET_CAUSE =            0x0,
    GRC_POWER_ON_RESET =                 0x1,
    GRC_EXTERNAL_PIN_RESET =             0x2,
    GRC_WATCHDOG_RESET =                 0x3,
    GRC_SOFTWARE_RESET =                 0x4,
    GRC_LOCKUP_EVENT_RESET =             0x5,
    GRC_EXTERNAL_DEBUGGER_RESET =        0x6,
    GRC_OTHER_HW_REASON_RESET =          0x7,
    GRC_STOP_ACK_ERROR_RESET  =          0x8
};

/**
 * CPU reset cause type
 *
 * 32-bit word with the following fields:
 * - bits[3:0] encode the generic reset cause using enum cpu_reset_causes
 * - bits[7:0] reserved (0)
 * - bits[15:8] first machine-dependent reset cause value
 * - bits[23:16] second first machine-dependent reset cause value
 * - bits[31:24] reserved (0)
 */
typedef uint32_t cpu_reset_cause_t;

#define CPU_RESET_GENERIC_CAUSE_MASK       MULTI_BIT_MASK(3, 0)
#define CPU_RESET_GENERIC_CAUSE_SHIFT      0
#define CPU_RESET_MACHDEP_CAUSE1_MASK      MULTI_BIT_MASK(15, 8)
#define CPU_RESET_MACHDEP_CAUSE1_SHIFT     8
#define CPU_RESET_MACHDEP_CAUSE2_MASK      MULTI_BIT_MASK(23, 16)
#define CPU_RESET_MACHDEP_CAUSE2_SHIFT     16

#if DEFINED_ARM_CORTEX_M_ARCH()
    /**
     * Interrupt channel type
     * (IRQ number of systick interrupt is -1)
     */
    typedef _RANGE_(-1, SOC_NUM_INTERRUPT_CHANNELS - 1)
            int16_t interrupt_channel_t;

    /**
     * CPU Instruction type
     */
    typedef uint16_t cpu_instruction_t;
#else
    /**
     * Interrupt channel type
     */
    typedef _RANGE_(0, SOC_NUM_INTERRUPT_CHANNELS - 1)
            uint8_t interrupt_channel_t;

    /**
     * CPU Instruction type
     */
    typedef uint32_t cpu_instruction_t;
#endif

/**
 * Interrupt priority type (0 is the highest priority)
 */
typedef _RANGE_(0, SOC_NUM_INTERRUPT_PRIORITIES - 1)
        uint8_t interrupt_prio_t;

/**
 * Interrupt service routine (ISR) function type
 */
typedef void isr_function_t(void);

/**
 * CPU clock cycles type
 */
typedef uint32_t cpu_clock_cycles_t;

C_ASSERT(sizeof(cpu_clock_cycles_t) == sizeof(int32_t));

/**
 * PWM channel index type
 */
typedef _RANGE_(0, PWM_MAX_NUM_CHANNELS - 1)
        uint8_t pwm_channel_t;

/**
 * PWM duty cycle type (in microseconds)
 */
typedef _RANGE_(0, 1000000)
        uint32_t pwm_duty_cycle_us_t;

/**
 * function signature for an interrupt service routine
 */
typedef void isr_function_t(void);

C_ASSERT(sizeof(isr_function_t *) == sizeof(uint32_t));

typedef _RANGE_(0, RTOS_MAX_MPU_REGIONS - 1)
        uint8_t mpu_region_index_t;

/*
 * I2C transaction header fields
 */
#define I2C_SLAVE_ADDR_MASK         MULTI_BIT_MASK(7, 1)
#define I2C_SLAVE_ADDR_SHIFT        1
#define I2C_READ_TRANSACTION_MASK   BIT(0)

/*
 * Opaque Types
 */
struct pin_info;
struct gpio_pin;
struct timer_device;
struct uart_device;
struct ssp_controller;
struct buttons_device;
struct adc_device;
struct i2c_device;
struct enet_device;
struct local_l3_end_point;
struct network_packet;
struct ethernet_mac_address;

typedef void app_hardware_init_t(void);
typedef void app_hardware_stop_t(void);
typedef void app_software_init_t(void);

/*
 * Exported functions
 */

void soc_early_init(void);
cpu_reset_cause_t soc_hardware_init(void);

bool software_reset_happened(void);

_NEVER_RETURN_
void soc_reset(void);

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
    uint8_t num_defined_global_regions;
};

/**
 * MPU data region range
 *
 * NOTE: If MPU_READ_ONLY_REGION is set in flags, only read access is allowed.
 * Otherwise, read/write access is allowed.
 */
struct mpu_region_range {
    /**
     * Address of the first byte of the region
     */
    void *start_addr;

    /**
     * Address of the first byte after the region
     */
    void *end_addr;

    /**
     * Access flags for the region
     */
    uint32_t flags;
#   define  MPU_REGION_INACTIVE     UINT32_C(0x1)
#   define  MPU_REGION_READ_ONLY    UINT32_C(0x2)
};

void mpu_disable(void);

void mpu_enable(void);

void mpu_set_all_thread_data_regions(
    cpu_id_t cpu_id,
    struct mpu_region_range regions[RTOS_NUM_THREAD_MPU_REGIONS]);

void mpu_set_thread_data_region(
    cpu_id_t cpu_id,
    mpu_region_index_t region_index,
    void *start_addr,
    void *end_addr,
    uint32_t flags);

void mpu_unset_thread_data_region(mpu_region_index_t region_index);

void mpu_register_dma_region(
    enum mpu_bus_masters dma_bus_master,
    void *start_addr,
    size_t size);

void mpu_get_enclosing_region_boundaries(
        void *start_addr, void *end_addr,
        void **region_start_addr, void **region_end_addr);

uint32_t calc_crc_32(const void *data_buf_p, size_t num_bytes);

void install_isr(
    interrupt_channel_t channel,
    isr_function_t *interruptServiceRoutine,
    interrupt_prio_t priority,
    cpu_id_t cpu_id);

void assert_interrupt_source_is_set(
        interrupt_channel_t interrupt_channel);

void notify_interrupt_controller_isr_done(
        interrupt_channel_t interrupt_channel);

void initialize_tick_timer(void);

#ifdef _CPU_CYCLES_MEASURE_
uint32_t get_cpu_clock_cycles(void);
#endif

void turn_on_power(uint32_t pconp_mask);

void set_pin_function(const struct pin_info *pin_p, uint32_t pin_flags);

void configure_gpio_pin(const struct gpio_pin *gpio_pin_p, uint32_t pin_flags,
			bool is_output);

void activate_output_pin(const struct gpio_pin *gpio_pin_p);

void deactivate_output_pin(const struct gpio_pin *gpio_pin_p);

void toggle_output_pin(const struct gpio_pin *gpio_pin_p);

bool read_input_pin(const struct gpio_pin *gpio_pin_p);

void uart_init(
        _IN_ const struct uart_device *uart_device_p,
        _IN_ uint32_t baud,
        _IN_ uint8_t mode);

void uart_stop(
        _IN_ const struct uart_device *uart_device_p);

void uart_enable_tx_rx_fifos(
    _IN_ const struct uart_device *uart_device_p);

void uart_disable_tx_rx_fifos(
    _IN_ const struct uart_device *uart_device_p);


_REQUIRES_MUTUAL_EXCLUSION_
void uart_putchar(
    _IN_ const struct uart_device *uart_device_p,
    _IN_ uint8_t c);

_REQUIRES_MUTUAL_EXCLUSION_
void uart_putchar_with_polling(
    _IN_ const struct uart_device *uart_device_p,
    _IN_ uint8_t c);

_REQUIRES_MUTUAL_EXCLUSION_
uint8_t
uart_getchar(
    _IN_ const struct uart_device *uart_device_p);

_REQUIRES_MUTUAL_EXCLUSION_
uint8_t
uart_getchar_with_polling(
    _IN_ const struct uart_device *uart_device_p);

void uart_interrupt_pre_handler(
        _IN_ const struct uart_device *uart_device_p);

void uart_interrupt_post_handler(
        _IN_ const struct uart_device *uart_device_p);

void clear_timer_interrupt_source(
        _IN_ const struct timer_device *timer_device_p);

void timer_interrupt_handler(
        _IN_ const struct timer_device *timer_device_p);

void clear_adc_interrupt_source(
        _IN_ const struct adc_device *adc_device_p);

void adc_interrupt_handler(
    _IN_ const struct adc_device *adc_device_p);

void clear_gpio_ports_interrupt_source(
        _IN_ const struct buttons_device *buttons_device_p);

void gpio_ports_interrupt_handler(
        _IN_ const struct buttons_device *buttons_device_p);

void clear_ssp_controller_interrupt_source(
    _IN_ const struct ssp_controller *ssp_controller_p);

void ssp_controller_interrupt_handler(
    _IN_ const struct ssp_controller *ssp_controller_p);

void init_adc(
    _IN_ const struct adc_device *adc_device_p);

void init_adc_channel(
        _IN_ const struct adc_device *adc_device_p,
        _IN_ uint8_t adc_channel);

void select_input_pin_adc_channel(
        _IN_ uint8_t adc_channel);

uint32_t read_adc_channel(
        _IN_ const struct adc_device *adc_device_p,
        _IN_ uint8_t adc_channel);

void toggle_rgb_led(
	_IN_ uint32_t led_color_mask);

uint32_t set_rgb_led_color(
	_IN_ uint32_t led_color_mask);

void init_buttons(
        _IN_ const struct buttons_device *buttons_device_p);

void init_trimpot(void);

uint32_t read_trimpot(void);

uint32_t read_buttons(
            _IN_ const struct buttons_device *buttons_device_p);

void init_ssp(
        _IN_ const struct ssp_controller *ssp_controller_p,
        _IN_ bool master);

void ssp_flush_transmit_receive_fifos(
        _IN_ const struct ssp_controller *ssp_controller_p);

uint16_t ssp_transmit_receive_16bit_value(
        _IN_ const struct ssp_controller *ssp_controller_p,
        _IN_ uint16_t outgoing_value);

void ssp_transmit_receive_buffer(
        _IN_ const struct ssp_controller *ssp_controller_p,
        _IN_ const uint8_t *outgoing_buffer, uint8_t *incoming_buffer, size_t size);

void i2c_init(
    const struct i2c_device *i2c_device_p);

void i2c_shutdown(
    const struct i2c_device *i2c_device_p);

void i2c_read(
    const struct i2c_device *i2c_device_p,
    uint8_t i2c_slave_addr, uint8_t i2c_slave_reg_addr,
    uint8_t *buffer_p, size_t num_bytes);

void i2c_write(
    const struct i2c_device *i2c_device_p,
    uint8_t i2c_slave_addr, uint8_t i2c_slave_reg_addr,
    uint8_t *buffer_p, size_t num_bytes);

void enet_init(const struct enet_device *enet_device_p);

void enet_start(const struct enet_device *enet_device_p,
	        struct local_l3_end_point *local_l3_end_point_p);

void enet_add_multicast_mac_addr(const struct enet_device *enet_device_p,
                                 struct ethernet_mac_address *mac_addr_p);

void enet_remove_multicast_mac_addr(const struct enet_device *enet_device_p,
                                    struct ethernet_mac_address *mac_addr_p);

void enet_start_xmit(const struct enet_device *enet_device_p,
		     struct network_packet *tx_packet_p);

void enet_repost_rx_packet(const struct enet_device *enet_device_p,
			   struct network_packet *rx_packet_p);

void enet_phy_write(const struct enet_device *enet_device_p, uint32_t phy_reg,
		    uint32_t data);

uint32_t enet_phy_read(const struct enet_device *enet_device_p,
		       uint32_t phy_reg);

void enet_get_mac_addr(const struct enet_device *enet_device_p,
                       struct ethernet_mac_address *mac_addr_p);

void enet_register_dma_region(void *start_addr, size_t size);

void wait_for_interrupts(void);

void send_inter_processor_interrupt(cpu_id_t cpu_id);

uint32_t read_32bit_mmio_register(const volatile uint32_t *io_reg_p);

void write_32bit_mmio_register(volatile uint32_t *io_reg_p, uint32_t value);

uint16_t read_16bit_mmio_register(const volatile uint16_t *io_reg_p);

void write_16bit_mmio_register(volatile uint16_t *io_reg_p, uint16_t value);

uint8_t read_8bit_mmio_register(const volatile uint8_t *io_reg_p);

void write_8bit_mmio_register(volatile uint8_t *io_reg_p, uint8_t value);

/*
 * Exported global variables
 */

extern const struct uart_device *const g_console_serial_port_p;
extern const struct uart_device *const g_bluetooth_serial_port_p;
extern const struct buttons_device *const g_buttons_device_p;
extern const struct adc_device *const g_adc_device_p;

extern struct sdram_map *const g_sdram_map_p;

extern void *const g_available_sdram_p;

#endif /* _HARDWARE_ABSTRACTIONS_H */
