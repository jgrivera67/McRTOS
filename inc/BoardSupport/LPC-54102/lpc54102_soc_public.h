/**
 * @file lpc54102_soc_public.h
 *
 * NXP LPC54102 SOC public declarations
 *
 * @author German Rivera
 */

#ifndef __LPC54102_SOC_PUBLIC_H
#define __LPC54102_SOC_PUBLIC_H

#include <nxp_chip.h>
#include <arm_defs.h>
#include <compile_time_checks.h>
#include <stdint.h>

/**
 * Number of interrupt channels supported by the interrupt
 * controller
 */
#define SOC_NUM_INTERRUPT_CHANNELS INT32_C(45)
#include "cortex_m_nvic.h"

/**
 * CPU clock frequency in MHz
 */
#define SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ  UINT32_C(100)

/**
 * Interrupt priority assignments (from lowest to highest)
 *
 * NOTE: All interrupts must have priorities lower (higher value)
 * than SOC_HIGHEST_INTERRUPT_PRIORITY, which is reserved for the
 * PENDSV and SVC exceptions
 */
#define UART0_INTERRUPT_PRIORITY    SOC_LOWEST_INTERRUPT_PRIORITY
#define SYSTICK_INTERRUPT_PRIORITY  (SOC_HIGHEST_INTERRUPT_PRIORITY + 2)
#define ADC_INTERRUPT_PRIORITY      (SOC_LOWEST_INTERRUPT_PRIORITY - 2)
#define I2C_INTERRUPT_PRIORITY      (SOC_LOWEST_INTERRUPT_PRIORITY - 2)
#define FTM_INTERRUPT_PRIORITY      (SOC_LOWEST_INTERRUPT_PRIORITY - 2)
#define PORT_C_INTERRUPT_PRIORITY   (SOC_LOWEST_INTERRUPT_PRIORITY - 1)
#define ENET_INTERRUPT_PRIORITY	    (SOC_LOWEST_INTERRUPT_PRIORITY - 1)

C_ASSERT(ADC_INTERRUPT_PRIORITY > SOC_HIGHEST_INTERRUPT_PRIORITY);

/*
 * include CMSIS API header after declaration of IRQn_Type
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtype-limits"
#pragma GCC diagnostic ignored "-Wcpp"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#ifdef __CORTEX_M
#error "core_cm4.h should not be included before here"
#endif

#ifndef __VFP_FP__
#error "Hardware floating point code not being generated"
#endif

#define __MPU_PRESENT   1
#define __FPU_PRESENT   1
#define __CHECK_DEVICE_DEFINES
#include "core_cm4.h"

C_ASSERT(__FPU_USED == 1);

#pragma GCC diagnostic pop

/**
 * Cache line size. The K64F SoC does not have caches
 */
#define SOC_CACHE_LINE_SIZE_IN_BYTES  sizeof(uint32_t)

/**
 * Flash Memory Range
 */
#define SOC_FLASH_BASE     UINT32_C(0x00000000)
#define SOC_FLASH_SIZE     (UINT32_C(512) * 1024)

/**
 * Static RAM Memory Ranges
 */
#define SOC_SRAM_BASE      UINT32_C(0x20000000)
#define SOC_SRAM_SIZE      (UINT32_C(104) * 1024)

/*
 * MMIO Ranges
 */
#define SOC_PERIPHERAL_BRIDGE_MIN_ADDR      UINT32_C(0x40000000)
#define SOC_PERIPHERAL_BRIDGE_MAX_ADDR      UINT32_C(0x400FFFFF)
#define SOC_PRIVATE_PERIPHERALS_MIN_ADDR    UINT32_C(0xE0000000)
#define SOC_PRIVATE_PERIPHERALS_MAX_ADDR    UINT32_C(0xE0100000)

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

/**
 * Max number of PWM channels in a PWM device
 */
#define PWM_MAX_NUM_CHANNELS    6

/**
 * A/D conversion resolution in bits
 */
#define ADC_RESOLUTION 12

/**
 * Max value of the result of an A/D conversion
 */
#define ADC_RESULT_MAX_VALUE    ((UINT32_C(1) << ADC_RESOLUTION) - 1)

/**
 * Initializer for a pin_info structure
 */
#define PIN_INITIALIZER(						\
            _pin_port, _pin_index, _pin_function)            		\
    {                                                                   \
	.pin_port = (_pin_port),					\
	.pin_index = (_pin_index),					\
	.pin_function = (_pin_function),				\
    }

/**
 * Initializer for a gpio_pin structure
 */
#define GPIO_PIN_INITIALIZER(						\
            _pin_port, _pin_index, _pin_function,			\
	    _pin_is_active_high)					\
    {                                                                   \
        .pin_info = PIN_INITIALIZER(_pin_port, _pin_index,		\
				    _pin_function),			\
        .pin_bit_mask = BIT(_pin_index),				\
        .pin_is_active_high = (_pin_is_active_high),                    \
    }

/**
 * SoC-specific NVIC interrupt vector numbers
 */
enum soc_interrupt_vectors {
    INT_WDT_IRQ = CORTEX_M_IRQ_VECTOR_BASE, // Watchdog
    INT_BOD_IRQ,         // Brown Out Detect
    INT_Reserved0_IRQ,    // Reserved
    INT_DMA_IRQ,         // DMA Controller
    INT_GINT0_IRQ,       // GPIO Group0 Interrupt
    INT_PIN_INT0_IRQ,    // PIO INT0
    INT_PIN_INT1_IRQ,    // PIO INT1
    INT_PIN_INT2_IRQ,    // PIO INT2
    INT_PIN_INT3_IRQ,    // PIO INT3
    INT_UTICK_IRQ,       // UTICK timer
    INT_MRT_IRQ,         // Multi-Rate Timer
    INT_CT32B0_IRQ,      // Counter Timer 0
    INT_CT32B1_IRQ,      // Counter Timer 1
    INT_CT32B2_IRQ,      // Counter Timer 2
    INT_CT32B3_IRQ,      // Counter Timer 3
    INT_CT32B4_IRQ,      // Counter Timer 4
    INT_SCT0_IRQ,        // Smart Counter Timer
    INT_UART0_IRQ,       // UART0
    INT_UART1_IRQ,       // UART1
    INT_UART2_IRQ,       // UART2
    INT_UART3_IRQ,       // UART3
    INT_I2C0_IRQ,        // I2C0 controller
    INT_I2C1_IRQ,        // I2C1 controller
    INT_I2C2_IRQ,        // I2C2 controller
    INT_SPI0_IRQ,        // SPI0 controller
    INT_SPI1_IRQ,        // SPI1 controller
    INT_ADC_SEQA_IRQ,    // ADC SEQA
    INT_ADC_SEQB_IRQ,    // ADC SEQB
    INT_ADC_THCMP_IRQ,   // ADC THCMP and OVERRUN ORed
    INT_RTC_IRQ,         // RTC Timer
    INT_Reserved1_IRQ,   // Reserved
    INT_MAILBOX_IRQ,     // Mailbox

    // External Interrupts - For M4 only
    INT_GINT1_IRQ,       // GPIO Group1 Interrupt
    INT_PIN_INT4_IRQ,    // PIO INT4
    INT_PIN_INT5_IRQ,    // PIO INT5
    INT_PIN_INT6_IRQ,    // PIO INT6
    INT_PIN_INT7_IRQ,    // PIO INT7
    INT_SPI2_IRQ,        // SPI2 controller
    INT_SPI3_IRQ,        // SPI3 controller
    INT_Reserved2_IRQ,   // Reserved
    INT_RIT_IRQ,         // RIT Timer
    INT_Reserved3_IRQ,   // Reserved
    INT_Reserved4_IRQ,   // Reserved
    INT_Reserved5_IRQ,   // Reserved
    INT_Reserved6_IRQ,   // Reserved
};

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

#define NUM_PINS_PER_PORT   32

typedef _RANGE_(0, NUM_PINS_PER_PORT - 1)
        uint8_t pin_index_t;

/**
 * Pin ports
 */
enum pin_ports {
	PIN_PORT_0 = 0,
	PIN_PORT_1,
	NUM_PIN_PORTS
};

typedef enum pin_ports pin_port_t;

enum pin_functions {
	PIN_FUNCTION0 = 0,
	PIN_FUNCTION1,
	PIN_FUNCTION2,
	PIN_FUNCTION3,
	PIN_FUNCTION4,
	PIN_FUNCTION5,
	PIN_FUNCTION6,
	PIN_FUNCTION7,
};

typedef enum pin_functions pin_function_t;

/**
 * Pin configuration parameters
 */
struct pin_info {
    pin_port_t pin_port;
    pin_index_t pin_index;
    pin_function_t pin_function;
};

/**
 * GPIO pin
 */
struct gpio_pin {
    struct pin_info pin_info;
    uint32_t pin_bit_mask;
    uint8_t pin_is_active_high; /*  false - low, true - high */
};

#endif /* __LPC54102_SOC_PUBLIC_H */
