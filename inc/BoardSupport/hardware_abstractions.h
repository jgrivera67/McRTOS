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
#include "compile_time_checks.h"
#include "lcd.h"
#include "touch_screen.h"

/**
 * DRAM Page size in bytes (4KB)
 */
#define DRAM_PAGE_SIZE  UINT32_C(4096)

#if defined(LPC2478_SOC)

#   include "lpc2478_stk_board_public.h"

#   define LCD_SUPPORTED
#   define DRAM_SUPPORTED

#elif defined(KL25Z_SOC)

#   include "kl25z_soc_public.h"

#elif defined(K20D5_SOC)

#   include "k20d5_soc_public.h"

#elif defined(K64F_SOC)

#   include "k64f_soc_public.h"

#elif defined(LM4F120_SOC)

#   include "lm4f120_soc_public.h"

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

typedef _RANGE_(0, RTOS_MAX_MPU_THREAD_DATA_REGIONS - 1)
        uint8_t mpu_thread_data_region_index_t;

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
 * MPU data region range
 *
 * NOTE: If read_only is true, only read access is allowed. Otherwise,
 * read/write access is allowed.
 */
struct mpu_region_range {
    void *start_addr;
    void *end_addr;
    bool read_only;
};

void mpu_disable(void);

void mpu_enable(void);

void mpu_set_thread_data_regions(
    cpu_id_t cpu_id,
    struct mpu_region_range regions[],
    uint8_t num_regions);

void mpu_set_thread_data_region(
    cpu_id_t cpu_id,
    mpu_thread_data_region_index_t thread_region_index,
    void *start_addr,
    void *end_addr,
    bool read_only);

void mpu_unset_thread_data_region(mpu_thread_data_region_index_t thread_region_index);

void mpu_register_dma_device(
    enum mpu_bus_masters mpu_bus_master);

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

void turn_on_rgb_led(
	_IN_ uint32_t led_color_mask);

void turn_off_rgb_led(
	_IN_ uint32_t led_color_mask);

uint32_t set_rgb_led_color(
	_IN_ uint32_t led_color_mask);

void init_buttons(
        _IN_ const struct buttons_device *buttons_device_p);

void init_trimpot(void);

uint32_t read_trimpot(void);

void toggle_heartbeat_led(void);

void turn_on_debugger_led(void);

void turn_off_debugger_led(void);

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
extern const struct buttons_device *const g_buttons_device_p;
extern const struct adc_device *const g_adc_device_p;

extern struct sdram_map *const g_sdram_map_p;

extern void *const g_available_sdram_p;

#endif /* _HARDWARE_ABSTRACTIONS_H */
