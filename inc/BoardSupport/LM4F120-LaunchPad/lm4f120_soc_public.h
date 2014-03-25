/**
 * @file lm4f120_soc_public.h
 *
 * TI Stellaris LM4F120 SOC public declarations
 *
 * @author German Rivera
 */

#ifndef __LM4F120_SOC_PUBLIC_H
#define __LM4F120_SOC_PUBLIC_H

#include "arm_defs.h"
#include "compile_time_checks.h"
#include "tivaware/hw_memmap.h"
#include "tivaware/tm4c123gh6pm.h"
#include <stdint.h>

/**
 * Number of interrupt channels supported by the interrupt
 * controller
 */
#define SOC_NUM_INTERRUPT_CHANNELS INT32_C(139)
#include "cortex_m_nvic.h"

/**
 * CPU clock frequency in MHz
 */
#define SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ  UINT32_C(50)

/**
 * Interrupt priority assignments (from lowest to highest)
 */
#define UART0_INTERRUPT_PRIORITY    SOC_LOWEST_INTERRUPT_PRIORITY
#define SYSTICK_INTERRUPT_PRIORITY  (SOC_LOWEST_INTERRUPT_PRIORITY - 1)
#define PENDSV_INTERRUPT_PRIORITY   SOC_HIGHEST_INTERRUPT_PRIORITY

/*
 * include CMSIS API header after declaration of IRQn_Type
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
#pragma GCC diagnostic ignored "-Wcpp"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#define __MPU_PRESENT   1
#define __FPU_PRESENT   1
#define __CHECK_DEVICE_DEFINES
#include "core_cm4.h"

#pragma GCC diagnostic pop

/**
 * Cache line size. The LM4F120 SoC does not have caches
 */
#define SOC_CACHE_LINE_SIZE_IN_BYTES  sizeof(uint32_t)

/**
 * Flash Memory Range
 */
#define SOC_FLASH_BASE     UINT32_C(0x00000000)
#define SOC_FLASH_SIZE     (UINT32_C(128) * 1024)

/**
 * Static RAM Memory Ranges
 */
#define SOC_SRAM_BASE      UINT32_C(0x1FFFF000)
#define SOC_SRAM_SIZE      (UINT32_C(16) * 1024)

/*
 * MMIO Ranges
 */
#define SOC_PERIPHERAL_BRIDGE_MIN_ADDR      UINT32_C(0x40000000)
#define SOC_PERIPHERAL_BRIDGE_MAX_ADDR      UINT32_C(0x400FFFFF)
#define SOC_PRIVATE_PERIPHERALS_MIN_ADDR    UINT32_C(0xE0000000)
#define SOC_PRIVATE_PERIPHERALS_MAX_ADDR    UINT32_C(0xE00FFFFF)
#define SOC_MTB_MIN_ADDR                    UINT32_C(0xF0000000)
#define SOC_MTB_MAX_ADDR                    UINT32_C(0xF0000FFF)

/**
 * Check that an mmio address is in the valid MMIO space
 */
#define BOARD_VALID_MMIO_ADDRESS(_io_addr) \
        (((uintptr_t)(_io_addr) >= SOC_PERIPHERAL_BRIDGE_MIN_ADDR &&    \
          (uintptr_t)(_io_addr) <= SOC_PERIPHERAL_BRIDGE_MAX_ADDR) ||   \
         ((uintptr_t)(_io_addr) >= SOC_PRIVATE_PERIPHERALS_MIN_ADDR &&  \
          (uintptr_t)(_io_addr) <= SOC_PRIVATE_PERIPHERALS_MAX_ADDR) || \
         ((uintptr_t)(_io_addr) >= SOC_MTB_MIN_ADDR &&                  \
          (uintptr_t)(_io_addr) <= SOC_MTB_MAX_ADDR))

/**
 * Check that an address is in flash memory and it is not address 0x0
 */
#define BOARD_VALID_FLASH_ADDRESS(_addr) \
        ((uintptr_t)(_addr) > SOC_FLASH_BASE &&                         \
         (uintptr_t)(_addr) < SOC_FLASH_BASE + SOC_FLASH_SIZE)

/**
 * Check that an address is in RAM memory
 */
#define BOARD_VALID_RAM_ADDRESS(_addr) \
        ((uintptr_t)(_addr) >= SOC_SRAM_BASE &&                        \
         (uintptr_t)(_addr) < SOC_SRAM_BASE + SOC_SRAM_SIZE)

/**
 * Value for the MASK field of the MTB_MASTER register. It determines the
 * size of the micro trace circular buffer:
 * 2^(MASK field + 4) == 2^(6 + 4) = 1024 bytes
 */
#define MTB_MASTER_MASK_VALUE   6

/**
 * Hardware micro trace buffer size in bytes
 */
#define MICRO_TRACE_BUFFER_SIZE_IN_BYTES \
        (UINT32_C(1) << (MTB_MASTER_MASK_VALUE + 4))

/**
 * Number of entries in the hardware micro trace buffer
 */
#define MICRO_TRACE_BUFFER_NUM_ENTRIES \
        (MICRO_TRACE_BUFFER_SIZE_IN_BYTES / sizeof(uint64_t))


/**
 * Initialize a configurable pin
 */
#define PIN_COFIG_INFO_INITIALIZER(                                     \
             _gpio_port_p, _pin_bit_index, _pin_is_active_high,		\
	     _pin_is_locked)						\
    {                                                                   \
        .pin_gpio_port_p = (volatile struct gpio_port *)(_gpio_port_p),	\
        .pin_bit_index = _pin_bit_index,                                \
        .pin_bit_mask = BIT(_pin_bit_index),                            \
        .pin_is_active_high = (_pin_is_active_high),                    \
        .pin_is_locked = (_pin_is_locked),				\
    }

/**
 * Pin configuration parameters
 */
struct pin_config_info {
    volatile struct gpio_port *pin_gpio_port_p;
    uint32_t pin_bit_mask;
    uint8_t pin_bit_index;
    uint8_t pin_is_active_high; /* false - low, true - high */
    uint8_t pin_is_locked;
    uint8_t reserved;
};

void micro_trace_init(void);

void micro_trace_stop(void);

void micro_trace_restart(void);

void micro_trace_get_cursor(uint64_t **mtb_cursor_pp, bool *mtb_cursor_wrapped_p);

/*
 * Variables defined in LM4F120_SOC-flash.ld
 */
extern uint64_t __micro_trace_buffer[];
extern uint64_t __micro_trace_buffer_end[];

extern const struct i2c_device *const g_i2c0_device_p;

#endif /* __LM4F120_SOC_PUBLIC_H */
