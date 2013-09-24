/**
 * @file kl25z_soc.h
 *
 * Freescale KL25Z SOC declarations
 *
 * @author German Rivera 
 */ 

#ifndef __KL25Z_SOC_H
#define __KL25Z_SOC_H

#include "MKL25Z4.h"
#include "arm_defs.h"
#include "compile_time_checks.h"
#include <stdint.h>

/**
 * CPU clock frequency in MHz
 */
#define SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ  UINT32_C(48)

/**
 * Number of interrupt channels supported by the interrupt
 * controller
 */
#define SOC_NUM_INTERRUPT_CHANNELS INT32_C(32)

#define INT_SVCall_IRQn     VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SVCall)
#define PendableSrvReq_IRQn VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PendableSrvReq)
#define SysTick_IRQn        VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SysTick)

/**
 * IRQ number range type
 *
 * (typedef needed by the CMSIS APIs)
 */
typedef _RANGE_(INT_SVCall_IRQn, SOC_NUM_INTERRUPT_CHANNELS - 1)
        int8_t IRQn_Type;

/*
 * include CMSIS API header after declaration of IRQn_Type 
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
//#pragma GCC diagnostic warning "-Wcpp"
#pragma GCC diagnostic ignored "-Wcpp"

#define __CHECK_DEVICE_DEFINES
#include "core_cm0plus.h"

#pragma GCC diagnostic pop

/** 
 * Cache line size. The KL25Z SoC does not have caches
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

/**
 * Check that an mmio address is in the valid MMIO space
 */
#define BOARD_VALID_MMIO_ADDRESS(_io_addr) \
        (((uintptr_t)(_io_addr) >= SOC_PERIPHERAL_BRIDGE_MIN_ADDR &&    \
          (uintptr_t)(_io_addr) <= SOC_PERIPHERAL_BRIDGE_MAX_ADDR) ||   \
         ((uintptr_t)(_io_addr) >= SOC_PRIVATE_PERIPHERALS_MIN_ADDR &&  \
          (uintptr_t)(_io_addr) <= SOC_PRIVATE_PERIPHERALS_MAX_ADDR))

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

enum led_color_masks {
    LED_RED_MASK = 1 << 18,
    LED_GREEN_MASK = 1 << 19,
    LED_BLUE_MASK = 1 << 1,
};

void toggle_rgb_led(enum led_color_masks led_color_mask);
void turn_on_rgb_led(enum led_color_masks led_color_mask);
void turn_off_rgb_led(enum led_color_masks led_color_mask);

struct rtos_interrupt; /* opaque type */

void kl25_uart_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void kl25_adc_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

#endif /* __KL25Z_SOC_H */