/**
 * @file lpc2478_stk_board_public.h
 *
 * LPC2478-STK board-specific public declarations
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 
#ifndef _LPC2478_STK_BOARD_PUBLIC_H
#define _LPC2478_STK_BOARD_PUBLIC_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "compile_time_checks.h"

/**
 * CPU clock frequency in MHz
 */
#define SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ  UINT32_C(72)

/**
 * Number of interrupt channels supported by the interrupt
 * controller
 */
#define SOC_NUM_INTERRUPT_CHANNELS UINT32_C(32)

/**
 * Number of interrupt priorities in the interrupt controller
 */
#define SOC_NUM_INTERRUPT_PRIORITIES   UINT32_C(16)

/** 
 * Cache line size. The LPC2478 does not have caches
 */
#define SOC_CACHE_LINE_SIZE_IN_BYTES  sizeof(uint32_t)

/**
 * Flash Memory Range
 */
#define SOC_FLASH_BASE     UINT32_C(0x00000000)
#define SOC_FLASH_SIZE     UINT32_C(0x0007E000)          /* 512 KB */
                                                
/**
 * Static RAM Memory Ranges
 */
#define SOC_SRAM_BASE      UINT32_C(0x40000000)
#define SOC_SRAM_SIZE      UINT32_C(0x00010000)          /* 64 KB */

#define SOC_ETHERNET_SRAM_BASE  UINT32_C(0x7FE00000)
#define SOC_ETHERNET_SRAM_SIZE  UINT32_C(0x00004000)     /* 16 KB */

#define SOC_USB_SRAM_BASE       UINT32_C(0x7FD00000)
#define SOC_USB_SRAM_SIZE       UINT32_C(0x00004000)     /* 16 KB */

/* 
 * MMIO Ranges
 */
#define SOC_FAST_GPIO_MIN_ADDR          UINT32_C(0x3FFFC000)
#define SOC_FAST_GPIO_MAX_ADDR          UINT32_C(0x3FFFFFFF)
#define SOC_APB_PERIPHERALS_MAX_ADDR    UINT32_C(0xe01fffff)
#define SOC_APB_PERIPHERALS_MIN_ADDR    UINT32_C(0xe0000000)
#define SOC_APB_PERIPHERALS_MAX_ADDR    UINT32_C(0xe01fffff)
#define SOC_AHB_PERIPHERALS_MIN_ADDR    UINT32_C(0xffe00000)
#define SOC_AHB_PERIPHERALS_MAX_ADDR    UINT32_C(0xffffffff)


/*
 * Off-chip SDRAM Memory Range
 */
#define BOARD_SDRAM_BASE     UINT32_C(0xA0000000)
#define BOARD_SDRAM_SIZE     UINT32_C(0x04000000)        /* 64 MB */

/**
 * Check that an mmio address is in the valid MMIO space
 */
#define BOARD_VALID_MMIO_ADDRESS(_io_addr) \
        (((uintptr_t)(_io_addr) >= SOC_FAST_GPIO_MIN_ADDR &&                 \
          (uintptr_t)(_io_addr) <= SOC_FAST_GPIO_MAX_ADDR) ||                \
         ((uintptr_t)(_io_addr) >= SOC_AHB_PERIPHERALS_MIN_ADDR &&           \
          (uintptr_t)(_io_addr) <= SOC_AHB_PERIPHERALS_MAX_ADDR) ||          \
         ((uintptr_t)(_io_addr) >= SOC_APB_PERIPHERALS_MIN_ADDR &&           \
          (uintptr_t)(_io_addr) <= SOC_APB_PERIPHERALS_MAX_ADDR))

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
        (((uintptr_t)(_addr) >= SOC_SRAM_BASE &&                        \
          (uintptr_t)(_addr) < SOC_SRAM_BASE + SOC_SRAM_SIZE) ||        \
         ((uintptr_t)(_addr) >= SOC_ETHERNET_SRAM_BASE &&               \
          (uintptr_t)(_addr) < SOC_ETHERNET_SRAM_BASE +                 \
                               SOC_ETHERNET_SRAM_SIZE) ||               \
         ((uintptr_t)(_addr) >= SOC_USB_SRAM_BASE &&                    \
          (uintptr_t)(_addr) < SOC_USB_SRAM_BASE +                      \
                               SOC_USB_SRAM_SIZE) ||                    \
         ((uintptr_t)(_addr) >= BOARD_SDRAM_BASE &&                     \
          (uintptr_t)(_addr) < BOARD_SDRAM_BASE + BOARD_SDRAM_SIZE))

/**
 * LCD Frame buffer size in bytes (one 4K page) 
 */
#define LCD_FRAME_BUFFER_SIZE   (1 * DRAM_PAGE_SIZE)

C_ASSERT(LCD_FRAME_BUFFER_SIZE % sizeof(uint64_t) == 0);

/**
 * Size in bytes of the DRAM area reserved for the execution stacks of
 * application threads (256 4K pages or 1MB)
 */
#define RTOS_APP_THREAD_STACK_DRAM_POOL_SIZE   (UINT32_C(256) * DRAM_PAGE_SIZE)

/**
 * Size in bytes of the DRAM area reserved for Ethernet 
 * data fragment buffers (12288 4K pages or 48MB)
 */
#define ETHERNET_DATA_FRAGMENT_BUFFER_POOL_SIZE (UINT32_C(12288) * DRAM_PAGE_SIZE)

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

/**
 * SDRAM map
 */
struct sdram_map
{
    /**
     * LCD frame buffer. It has to be dword aligned. 
     */
    uint64_t sdr_lcd_frame_buffer[LCD_FRAME_BUFFER_SIZE / sizeof(uint64_t)];

    /**
     * Area reserved for the pool of execution stacks for application threads
     */
    uint32_t sdr_rtos_app_thread_stacks[
                RTOS_APP_THREAD_STACK_DRAM_POOL_SIZE / sizeof(uint32_t)];

    /**
     * Area reserved for the pool of Ethernet data fragment buffers
     */
    uint32_t sdr_ethernet_data_fragment_buffers[
                ETHERNET_DATA_FRAGMENT_BUFFER_POOL_SIZE / sizeof(uint32_t)];
};

C_ASSERT(sizeof(struct sdram_map) % sizeof(uint64_t) == 0);
C_ASSERT(sizeof(struct sdram_map) <= BOARD_SDRAM_SIZE);
C_ASSERT(
    offsetof(struct sdram_map, sdr_lcd_frame_buffer) % DRAM_PAGE_SIZE == 0);
C_ASSERT(
    offsetof(struct sdram_map, sdr_rtos_app_thread_stacks) % DRAM_PAGE_SIZE == 0);
C_ASSERT(
    offsetof(struct sdram_map, sdr_ethernet_data_fragment_buffers) % DRAM_PAGE_SIZE == 0);

/**
 * Base address for the LCD frame buffer in RAM
 */
#define LCD_FRAME_BUFFER_BASE_ADDR \
        (BOARD_SDRAM_BASE + offsetof(struct sdram_map, sdr_lcd_frame_buffer))

C_ASSERT(LCD_FRAME_BUFFER_BASE_ADDR % sizeof(uint64_t) == 0);

/**
 * Base address for the pool of execution stacks for application threads
 */
#define RTOS_APP_THREAD_DRAM_STACKS_BASE_ADDR \
        (BOARD_SDRAM_BASE + offsetof(struct sdram_map, sdr_rtos_app_thread_stacks))

/**
 * Base address for the pool of Ethernet data fragment buffers
 */
#define ETHERNET_DATA_FRAGMENT_BUFFERS_DRAM_BASE_ADDR \
        (BOARD_SDRAM_BASE + offsetof(struct sdram_map, sdr_ethernet_data_fragment_buffers))

/**
 * Start of available SDRAM
 */
#define AVAILABLE_SDRAM_START_ADDR  (BOARD_SDRAM_BASE + sizeof(struct sdram_map))

C_ASSERT(AVAILABLE_SDRAM_START_ADDR < BOARD_SDRAM_BASE + BOARD_SDRAM_SIZE);

void toggle_usb_host_link_led(void);

void toggle_usb_device_link_led(void);

#endif /* _LPC2478_STK_BOARD_PUBLIC_H */
