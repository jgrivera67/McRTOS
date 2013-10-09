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

#   include "kl25z_soc.h"

#elif defined(LM4F120_SOC)

#   error "TODO: Add #include here for LM4F120_launchpad"

#else

#error "No system on chip specified"

#endif /* platform-specific */

/**
 * Initialize a configurable pin
 */
#define PIN_COFIG_INFO_INITIALIZER(_gpio_port_index, _pin_bit_index,    \
                                   _pin_function,                       \
                                   _pin_is_active_high)                 \
    {                                                                   \
        .gpio_port_index = (_gpio_port_index),                          \
        .pin_bit_index = (_pin_bit_index),                              \
        .pinsel_mode_mask = GET_PINSEL_MODE_MASK(_pin_bit_index),       \
        .pinsel_mode_shift = GET_PINSEL_MODE_SHIFT(_pin_bit_index),     \
        .pin_function = (_pin_function),                                \
        .pin_is_active_high = (_pin_is_active_high),                    \
    }

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
            int8_t interrupt_channel_t;

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
 * function signature for an interrupt service routine
 */
typedef void isr_function_t(void);

C_ASSERT(sizeof(isr_function_t *) == sizeof(uint32_t));

/**
 * Pin configuration parameters
 */
struct pin_config_info {
    uint32_t pinsel_mode_mask;
    uint8_t gpio_port_index;
    uint8_t pin_bit_index;
    uint8_t pinsel_mode_shift;
    uint8_t pin_function;

    /*
     * The following fields is only meaningful if pin_function is PINSEL_PRIMARY
     */ 
    uint8_t pin_is_active_high;         /*  false - low, true - high */
    uint16_t reserved2;
};

/*
 * Opaque Types
 */
struct timer_device;
struct uart_device;
struct ssp_controller;
struct buttons_device;
struct adc_device;

typedef void app_hardware_init_t(void);

/*
 * Exported functions
 */

cpu_reset_cause_t soc_hardware_init(void);

bool software_reset_happened(void);

void soc_reset(void);

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

uint32_t get_cpu_clock_cycles(void);

void turn_on_power(uint32_t pconp_mask);

void configure_pin(const struct pin_config_info *pin_info_p, bool is_output);

void activate_output_pin(const struct pin_config_info *pin_info_p);

void deactivate_output_pin(const struct pin_config_info *pin_info_p);

bool read_input_pin(const struct pin_config_info *pin_info_p);

void uart_init(
        _IN_ const struct uart_device *uart_device_p,
        _IN_ uint32_t baud,
        _IN_ uint8_t mode);

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

void init_adc_channel(
        _IN_ const struct adc_device *adc_device_p,
        _IN_ uint8_t adc_channel);

void select_input_pin_adc_channel(
        _IN_ uint8_t adc_channel);

uint32_t read_adc_channel(
        _IN_ const struct adc_device *adc_device_p,
        _IN_ uint8_t adc_channel);

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

void wait_for_interrupts(void);

uint32_t read_32bit_mmio_register(volatile uint32_t *io_reg_p);

void write_32bit_mmio_register(volatile uint32_t *io_reg_p, uint32_t value);
 
uint16_t read_16bit_mmio_register(volatile uint16_t *io_reg_p);

void write_16bit_mmio_register(volatile uint16_t *io_reg_p, uint16_t value);
 
uint8_t read_8bit_mmio_register(volatile uint8_t *io_reg_p);

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
