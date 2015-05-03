/**
 * @file k64f_soc_public.h
 *
 * Freescale K64F SOC public declarations
 *
 * @author German Rivera
 */

#ifndef __K64F_SOC_PUBLIC_H
#define __K64F_SOC_PUBLIC_H

#include "MK64F12.h"
#include <arm_defs.h>
#include <compile_time_checks.h>
#include <stdint.h>

/**
 * Number of interrupt channels supported by the interrupt
 * controller
 */
#define SOC_NUM_INTERRUPT_CHANNELS INT32_C(86)
#include "cortex_m_nvic.h"

/**
 * CPU clock frequency in MHz
 */
#define SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ  UINT32_C(120)

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

/*
 * NOTE: CMSIS parametric #defines were done in MK64F12.h
 */
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
#define SOC_FLASH_SIZE     (UINT32_C(1024) * 1024)

/**
 * Static RAM Memory Ranges
 */
#define SOC_SRAM_BASE      UINT32_C(0x1FFF0000)
#define SOC_SRAM_SIZE      (UINT32_C(256) * 1024)

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
 * MPU region alignment in bytes
 */
#define K64F_MPU_REGION_ALIGNMENT	UINT32_C(32)

/**
 * MPU region alignment mask
 */
#define K64F_MPU_REGION_ALIGNMENT_MASK	(~(K64F_MPU_REGION_ALIGNMENT - 1))

#if __MPU_PRESENT == 0
#   define MIN_MPU_REGION_ALIGNMENT     K64F_MPU_REGION_ALIGNMENT
    /*
     * For the K64F MPU, the MPU region alignment for a given data type
     * does not depend on the size of the data type
     */
#   define SOC_MPU_REGION_ALIGNMENT(_type)  K64F_MPU_REGION_ALIGNMENT
#endif

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
  INT_DMA0 = CORTEX_M_IRQ_VECTOR_BASE, /* DMA Channel 0 Transfer Complete */
  INT_DMA1,		    /* DMA Channel 1 Transfer Complete */
  INT_DMA2,		    /* DMA Channel 2 Transfer Complete */
  INT_DMA3,		    /* DMA Channel 3 Transfer Complete */
  INT_DMA4,		    /* DMA Channel 4 Transfer Complete */
  INT_DMA5,		    /* DMA Channel 5 Transfer Complete */
  INT_DMA6,		    /* DMA Channel 6 Transfer Complete */
  INT_DMA7,		    /* DMA Channel 7 Transfer Complete */
  INT_DMA8,		    /* DMA Channel 8 Transfer Complete */
  INT_DMA9,		    /* DMA Channel 9 Transfer Complete */
  INT_DMA10,		    /* DMA Channel 10 Transfer Complete */
  INT_DMA11,		    /* DMA Channel 11 Transfer Complete */
  INT_DMA12,		    /* DMA Channel 12 Transfer Complete */
  INT_DMA13,		    /* DMA Channel 13 Transfer Complete */
  INT_DMA14,		    /* DMA Channel 14 Transfer Complete */
  INT_DMA15,		    /* DMA Channel 15 Transfer Complete */
  INT_DMA_Error,	    /* DMA Error Interrupt */
  INT_MCM,		    /* Normal Interrupt */
  INT_FTFE,		    /* FTFE Command complete interrupt */
  INT_Read_Collision,       /* Read Collision Interrupt */
  INT_LVD_LVW,              /* Low Voltage Detect, Low Voltage Warning */
  INT_LLW,		    /* Low Leakage Wakeup */
  INT_Watchdog,             /* WDOG Interrupt */
  INT_RNG,		    /* RNG Interrupt */
  INT_I2C0,		    /* I2C0 interrupt */
  INT_I2C1,		    /* I2C1 interrupt */
  INT_SPI0,		    /* SPI0 Interrupt */
  INT_SPI1,		    /* SPI1 Interrupt */
  INT_I2S0_Tx,              /* I2S0 transmit interrupt */
  INT_I2S0_Rx,              /* I2S0 receive interrupt */
  INT_UART0_LON,            /* UART0 LON interrupt */
  INT_UART0_RX_TX,          /* UART0 Receive/Transmit interrupt */
  INT_UART0_ERR,            /* UART0 Error interrupt */
  INT_UART1_RX_TX,          /* UART1 Receive/Transmit interrupt */
  INT_UART1_ERR,            /* UART1 Error interrupt */
  INT_UART2_RX_TX,          /* UART2 Receive/Transmit interrupt */
  INT_UART2_ERR,            /* UART2 Error interrupt */
  INT_UART3_RX_TX,          /* UART3 Receive/Transmit interrupt */
  INT_UART3_ERR,            /* UART3 Error interrupt */
  INT_ADC0,		    /* ADC0 interrupt */
  INT_CMP0,		    /* CMP0 interrupt */
  INT_CMP1,		    /* CMP1 interrupt */
  INT_FTM0,		    /* FTM0 fault, overflow and channels interrupt */
  INT_FTM1,		    /* FTM1 fault, overflow and channels interrupt */
  INT_FTM2,		    /* FTM2 fault, overflow and channels interrupt */
  INT_CMT,		    /* CMT interrupt */
  INT_RTC,		    /* RTC interrupt */
  INT_RTC_Seconds,          /* RTC seconds interrupt */
  INT_PIT0,		    /* PIT timer channel 0 interrupt */
  INT_PIT1,		    /* PIT timer channel 1 interrupt */
  INT_PIT2,		    /* PIT timer channel 2 interrupt */
  INT_PIT3,		    /* PIT timer channel 3 interrupt */
  INT_PDB0,		    /* PDB0 Interrupt */
  INT_USB0,		    /* USB0 interrupt */
  INT_USBDCD,               /* USBDCD Interrupt */
  INT_Reserved71,           /* Reserved interrupt 71 */
  INT_DAC0,		    /* DAC0 interrupt */
  INT_MCG,		    /* MCG Interrupt */
  INT_LPTimer,              /* LPTimer interrupt */
  INT_PORTA,		    /* Port A interrupt */
  INT_PORTB,		    /* Port B interrupt */
  INT_PORTC,		    /* Port C interrupt */
  INT_PORTD,		    /* Port D interrupt */
  INT_PORTE,		    /* Port E interrupt */
  INT_SWI,		    /* Software interrupt */
  INT_SPI2,		    /* SPI2 Interrupt */
  INT_UART4_RX_TX,          /* UART4 Receive/Transmit interrupt */
  INT_UART4_ERR,            /* UART4 Error interrupt */
  INT_UART5_RX_TX,          /* UART5 Receive/Transmit interrupt */
  INT_UART5_ERR,            /* UART5 Error interrupt */
  INT_CMP2,		    /* CMP2 interrupt */
  INT_FTM3,		    /* FTM3 fault, overflow and channels interrupt */
  INT_DAC1,		    /* DAC1 interrupt */
  INT_ADC1,		    /* ADC1 interrupt */
  INT_I2C2,		    /* I2C2 interrupt */
  INT_CAN0_ORed_Message_buffer,	/* CAN0 OR'd message buffers interrupt */
  INT_CAN0_Bus_Off,		/* CAN0 bus off interrupt */
  INT_CAN0_Error,		/* CAN0 error interrupt */
  INT_CAN0_Tx_Warning,		/* CAN0 Tx warning interrupt */
  INT_CAN0_Rx_Warning,		/* CAN0 Rx warning interrupt */
  INT_CAN0_Wake_Up,             /* CAN0 wake up interrupt */
  INT_SDHC,			/* SDHC interrupt */
  INT_ENET_1588_Timer,          /* Ethernet MAC IEEE 1588 Timer Interrupt */
  INT_ENET_Transmit,            /* Ethernet MAC Transmit Interrupt */
  INT_ENET_Receive,             /* Ethernet MAC Receive Interrupt */
  INT_ENET_Error		/* Ethernet MAC Error and miscelaneous Interrupt */
};

/**
 * MPU bus masters (see section 3.3.7.2 of K64 Sub-Family Reference Manual)
 */
enum mpu_bus_masters {
	MPU_BUS_MASTER_CPU_CORE =   0,
	MPU_BUS_MASTER_DEBUGGER =   1,
	MPU_BUS_MASTER_DMA_EZPORT = 2,
	MPU_BUS_MASTER_ENET =	    3,
	MPU_BUS_MASTER_USB =	    4,
	MPU_BUS_MASTER_SDHC =	    5,
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
	PIN_PORT_A = 0,
	PIN_PORT_B,
	PIN_PORT_C,
	PIN_PORT_D,
	PIN_PORT_E,
	NUM_PIN_PORTS
};

typedef enum pin_ports pin_port_t;

/**
 * Pin configuration registers
 */
struct pin_config_registers {
    volatile PORT_Type *pin_port_regs_p;
    volatile GPIO_Type *pin_gpio_regs_p;
};


enum pin_functions {
	PIN_FUNCTION_ALT0 = 0,
	PIN_FUNCTION_ALT1,
	PIN_FUNCTION_ALT2,
	PIN_FUNCTION_ALT3,
	PIN_FUNCTION_ALT4,
	PIN_FUNCTION_ALT5,
	PIN_FUNCTION_ALT6,
	PIN_FUNCTION_ALT7,
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

#endif /* __K64F_SOC_PUBLIC_H */
