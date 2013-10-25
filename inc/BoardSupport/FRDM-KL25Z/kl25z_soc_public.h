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

#define __MPU_PRESENT   1
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
 * A/D conversion resolution in bits
 */
#define ADC_RESOLUTION  8

/**
 * Max value of the result of an A/D conversion
 */
#define ADC_RESULT_MAX_VALUE    ((UINT32_C(1) << ADC_RESOLUTION) - 1)

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
            _pin_bit_index, _pcr_value, _pin_is_active_high,            \
            _port_base_p, _gpio_base_p)                                 \
    {                                                                   \
        .pin_port_base_p = (_port_base_p),                              \
        .pin_gpio_base_p = (_gpio_base_p),                              \
        .pin_pcr_value = (_pcr_value),                                  \
        .pin_bit_index = _pin_bit_index,                                \
        .pin_bit_mask = BIT(_pin_bit_index),                            \
        .pin_is_active_high = (_pin_is_active_high),                    \
    }

/**
 * ADC result range type
 */
#if ADC_RESOLUTION <= 8
    typedef _RANGE_(0, ADC_RESULT_MAX_VALUE)
            uint8_t adc_result_t;
#elif ADC_RESOLUTION <= 16
    typedef _RANGE_(0, ADC_RESULT_MAX_VALUE)
            uint16_t adc_result_t;
#else
#   error "ADC_RESOLUTION value not supported"
#endif

C_ASSERT(sizeof(adc_result_t) * 8 >= ADC_RESOLUTION);

/**
 * Pin configuration parameters
 */
struct pin_config_info {
    PORT_MemMapPtr pin_port_base_p;
    GPIO_MemMapPtr pin_gpio_base_p;
    uint32_t pin_pcr_value;
    uint32_t pin_bit_mask;
    uint8_t pin_bit_index;
    uint8_t pin_is_active_high; /*  false - low, true - high */
    uint16_t reserved;
};

void micro_trace_init(void);

void micro_trace_stop(void);

void micro_trace_restart(void);

void micro_trace_get_cursor(uint64_t **mtb_cursor_pp, bool *mtb_cursor_wrapped_p);

/*
 * Variables defined in KL25Z_SOC-flash.ld 
 */
extern uint64_t __micro_trace_buffer[];
extern uint64_t __micro_trace_buffer_end[];

#endif /* __KL25Z_SOC_PUBLIC_H */
