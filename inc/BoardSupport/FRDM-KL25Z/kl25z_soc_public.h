/**
 * @file kl25z_soc_public.h
 *
 * Freescale KL25Z SOC public declarations
 *
 * @author German Rivera 
 */ 

#ifndef __KL25Z_SOC_PUBLIC_H
#define __KL25Z_SOC_PUBLIC_H

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
 * Interrupt priority assignments (from lowest to highest)
 */
#define UART0_INTERRUPT_PRIORITY    SOC_LOWEST_INTERRUPT_PRIORITY
#define SYSTICK_INTERRUPT_PRIORITY  (SOC_LOWEST_INTERRUPT_PRIORITY - 1)
#define ADC_INTERRUPT_PRIORITY      (SOC_LOWEST_INTERRUPT_PRIORITY - 2)
#define TPM_INTERRUPT_PRIORITY      (SOC_LOWEST_INTERRUPT_PRIORITY - 2)

C_ASSERT(ADC_INTERRUPT_PRIORITY > SOC_HIGHEST_INTERRUPT_PRIORITY);

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
#define SOC_MTB_MIN_ADDR                    UINT32_C(0xF0000000)
#define SOC_MTB_MAX_ADDR                    UINT32_C(0xF0000FFF)

/**
 * Value for the MASK field of the MTB_MASTER register. It determines the
 * size of the micro trace circular buffer: 
 * 2^(MASK field + 4) == 2^(6 + 4) = 1024 bytes
 */
#define MTB_MASTER_MASK_VALUE   6

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
 * Max number of PWM channels in a PWM device
 */
#define PWM_MAX_NUM_CHANNELS    6

/**
 * Hardware micro trace buffer size in bytes
 */
#define MICRO_TRACE_BUFFER_SIZE_IN_BYTES    UINT32_C(1024)

C_ASSERT(
    MICRO_TRACE_BUFFER_SIZE_IN_BYTES ==
        UINT32_C(1) << (MTB_MASTER_MASK_VALUE + 4));

/**
 * Number of entries in the hardware micro trace buffer
 */
#define MICRO_TRACE_BUFFER_NUM_ENTRIES \
        (MICRO_TRACE_BUFFER_SIZE_IN_BYTES / sizeof(uint64_t))

void micro_trace_init(void);

void micro_trace_stop(void);

void micro_trace_restart(void);

void micro_trace_get_cursor(uint64_t **mtb_cursor_pp, bool *mtb_cursor_wrapped_p);

extern uint64_t g_micro_trace_buffer[];

#endif /* __KL25Z_SOC_PUBLIC_H */
