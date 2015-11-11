/**
 * @file k64f_soc_hardware_abstractions.c
 *
 * Hardware abstraction layer for the K64F SoC
 *
 * @author German Rivera
 */

#include <BoardSupport/hardware_abstractions.h>
#include <BoardSupport/FRDM-K64F/k64f_soc.h>
#include <BoardSupport/FRDM-K64F/k64f_soc_enet.h>
#include <BoardSupport/FRDM-K64F/frdm_board.h>
#include <McRTOS/McRTOS_arm_cortex_m.h>
#include <McRTOS/failure_data_capture.h>
#include <McRTOS/utils.h>
#include <McRTOS/McRTOS_config_parameters.h>
#include <McRTOS/McRTOS_kernel_services.h>

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wunused-parameter" // ???

/*
 * Disable compiler optimizations for this file, since it contains
 * timing sensitive functions
 */
#pragma GCC optimize "O0"

/**
 * Crystal frequency in HZ
 */
#define CRYSTAL_FREQUENCY_HZ    UINT32_C(8000000)

#define PLL_DIVIDER             4

#define PLL_MULTIPLIER          24

#define REF_FREQUENCY           (CRYSTAL_FREQUENCY_HZ / PLL_DIVIDER)

#define PLL_FREQUENCY           ((CRYSTAL_FREQUENCY_HZ / PLL_DIVIDER) * PLL_MULTIPLIER)

C_ASSERT(CRYSTAL_FREQUENCY_HZ > 5000000 &&
         CRYSTAL_FREQUENCY_HZ <= 10000000);

C_ASSERT(CRYSTAL_FREQUENCY_HZ < CPU_CLOCK_FREQ_IN_HZ);
C_ASSERT(PLL_DIVIDER >= 1 && PLL_DIVIDER <= 25);
C_ASSERT(PLL_MULTIPLIER >= 24 && PLL_MULTIPLIER <= 50);
C_ASSERT(
    REF_FREQUENCY >= 2000000 && REF_FREQUENCY <= 4000000);

C_ASSERT(
    PLL_FREQUENCY >= 48000000 && PLL_FREQUENCY <= 100000000);

/**
 * Macro to generate dummy NVIC ISR functions
 */
#define GENERATE_DUMMY_NVIC_ISR_FUNCTION(_isr_function_name,                \
                                         _nvic_irq_number)                  \
        void                                                                \
        _isr_function_name(void)                                            \
        {                                                                   \
            /* This function should not have been invoked */                \
            FDC_ASSERT(false, _nvic_irq_number, 0);                         \
            FDC_ASSERT_INTERRUPT_SOURCE_IS_SET(_nvic_irq_number);           \
        }

/**
 * Serial communication parameters for the serial port used as the console
 * port
 */
#define CONSOLE_SERIAL_PORT_BAUD_RATE   115200
#define CONSOLE_SERIAL_PORT_MODE        0 /* default: 8-bits, no-parity, 1 stop bit */

#define UART_TRANSMIT_QUEUE_SIZE_IN_BYTES   UINT16_C(128)
#define UART_RECEIVE_QUEUE_SIZE_IN_BYTES    UINT16_C(16)

static cpu_reset_cause_t find_reset_cause(void);

#ifdef _CPU_CYCLES_MEASURE_
static void init_cpu_clock_cycles_counter(void);
#endif

#if 0 // ???
static void
k64f_adc_calibrate(const struct adc_device *adc_device_p);
#endif // ???

/*
 * Function prototypes for dummy VIC interrupt ISRs
 */
static isr_function_t dummy_dma0_isr;
static isr_function_t dummy_dma1_isr;
static isr_function_t dummy_dma2_isr;
static isr_function_t dummy_dma3_isr;
static isr_function_t dummy_dma4_isr;
static isr_function_t dummy_dma5_isr;
static isr_function_t dummy_dma6_isr;
static isr_function_t dummy_dma7_isr;
static isr_function_t dummy_dma8_isr;
static isr_function_t dummy_dma9_isr;
static isr_function_t dummy_dma10_isr;
static isr_function_t dummy_dma11_isr;
static isr_function_t dummy_dma12_isr;
static isr_function_t dummy_dma13_isr;
static isr_function_t dummy_dma14_isr;
static isr_function_t dummy_dma15_isr;
static isr_function_t dummy_dma_error_isr;
static isr_function_t dummy_mcm_isr;
static isr_function_t dummy_ftfe_isr;
static isr_function_t dummy_read_collision_isr;
static isr_function_t dummy_lvd_lvw_isr;
static isr_function_t dummy_llw_isr;
static isr_function_t dummy_watchdog_isr;
static isr_function_t dummy_i2c1_isr;
static isr_function_t dummy_spi0_isr;
static isr_function_t dummy_spi1_isr;
static isr_function_t dummy_i2s0_tx_isr;
static isr_function_t dummy_i2s0_rx_isr;
static isr_function_t dummy_uart0_lon_isr;
static isr_function_t dummy_uart1_rx_tx_isr;
static isr_function_t dummy_uart1_err_isr;
static isr_function_t dummy_uart2_rx_tx_isr;
static isr_function_t dummy_uart2_err_isr;
static isr_function_t dummy_uart3_rx_tx_isr;
static isr_function_t dummy_uart3_err_isr;
static isr_function_t dummy_cmp0_isr;
static isr_function_t dummy_cmp1_isr;
static isr_function_t dummy_ftm0_isr;
static isr_function_t dummy_ftm1_isr;
static isr_function_t dummy_ftm2_isr;
static isr_function_t dummy_cmt_isr;
static isr_function_t dummy_rtc_alarm_isr;
static isr_function_t dummy_rtc_seconds_isr;
static isr_function_t dummy_pit0_isr;
static isr_function_t dummy_pit1_isr;
static isr_function_t dummy_pit2_isr;
static isr_function_t dummy_pdb0_isr;
static isr_function_t dummy_usb_otg_isr;
static isr_function_t dummy_usb_dcd_isr;
static isr_function_t dummy_reserved71_isr;
static isr_function_t dummy_dac0_isr;
static isr_function_t dummy_mcg_isr;
static isr_function_t dummy_lptimer_isr;
static isr_function_t dummy_port_a_isr;
static isr_function_t dummy_port_b_isr;
static isr_function_t dummy_port_d_isr;
static isr_function_t dummy_port_e_isr;
static isr_function_t dummy_swi_isr;
static isr_function_t dummy_spi2_isr;
static isr_function_t dummy_uart5_rx_tx_isr;
static isr_function_t dummy_uart5_err_isr;
static isr_function_t dummy_cmp2_isr;
static isr_function_t dummy_ftm3_isr;
static isr_function_t dummy_dac1_isr;
static isr_function_t dummy_adc0_isr;
static isr_function_t dummy_adc1_isr;
static isr_function_t dummy_i2c2_isr;
static isr_function_t dummy_can0_ored_message_buffer_isr;
static isr_function_t dummy_can0_bus_off_isr;
static isr_function_t dummy_can0_error_isr;
static isr_function_t dummy_can0_tx_warning_isr;
static isr_function_t dummy_can0_rx_warning_isr;
static isr_function_t dummy_can0_wake_up_isr;
static isr_function_t dummy_sdhc_isr;
static isr_function_t dummy_enet_1588_timer_isr;

//???static uint32_t g_pll_frequency_in_hz = 0;

/**
 * Interrupt Vector Table
 */
isr_function_t *const g_interrupt_vector_table[] __attribute__ ((section(".vectortable"))) = {
    [INT_Initial_Stack_Pointer] = (void *)&g_cortex_m_exception_stack.es_stack_underflow_marker,

    /*
     * Processor exceptions
     */
    [INT_Initial_Program_Counter] = cortex_m_reset_handler,
    [INT_NMI] = cortex_m_nmi_isr,
    [INT_Hard_Fault] = cortex_m_hard_fault_exception_handler,
    [INT_MemoryManagement] = cortex_m_memory_management_exception_handler,
    [INT_BusFault] = cortex_m_bus_fault_exception_handler,
    [INT_UsageFault] = cortex_m_usage_fault_exception_handler,
    [INT_SVCall] = cortex_m_svc_exception_handler,
    [INT_DebugMonitor] = cortex_m_debug_monitor_exception_handler,
    [INT_PendableSrvReq] = cortex_m_pendsv_exception_handler,
    [INT_SysTick] = cortex_m_systick_isr,

    /* Interrupts external to the Cortex-M core*/
    [INT_DMA0] = dummy_dma0_isr, /* DMA Channel 0 Transfer Complete */
    [INT_DMA1] = dummy_dma1_isr, /* DMA Channel 1 Transfer Complete */
    [INT_DMA2] = dummy_dma2_isr, /* DMA Channel 2 Transfer Complete */
    [INT_DMA3] = dummy_dma3_isr, /* DMA Channel 3 Transfer Complete */
    [INT_DMA4] = dummy_dma4_isr, /* DMA Channel 4 Transfer Complete */
    [INT_DMA5] = dummy_dma5_isr, /* DMA Channel 5 Transfer Complete */
    [INT_DMA6] = dummy_dma6_isr, /* DMA Channel 6 Transfer Complete */
    [INT_DMA7] = dummy_dma7_isr, /* DMA Channel 7 Transfer Complete */
    [INT_DMA8] = dummy_dma8_isr, /* DMA Channel 8 Transfer Complete */
    [INT_DMA9] = dummy_dma9_isr, /* DMA Channel 9 Transfer Complete */
    [INT_DMA10] = dummy_dma10_isr, /* DMA Channel 10 Transfer Complete */
    [INT_DMA11] = dummy_dma11_isr, /* DMA Channel 11 Transfer Complete */
    [INT_DMA12] = dummy_dma12_isr, /* DMA Channel 12 Transfer Complete */
    [INT_DMA13] = dummy_dma13_isr, /* DMA Channel 13 Transfer Complete */
    [INT_DMA14] = dummy_dma14_isr, /* DMA Channel 14 Transfer Complete */
    [INT_DMA15] = dummy_dma15_isr, /* DMA Channel 15 Transfer Complete */
    [INT_DMA_Error] = dummy_dma_error_isr, /* DMA Error Interrupt */
    [INT_MCM] = dummy_mcm_isr, /* Normal Interrupt */
    [INT_FTFE] = dummy_ftfe_isr, /* FTFE Command Complete Interrupt */
    [INT_Read_Collision] = dummy_read_collision_isr, /* Read Collision Interrupt */
    [INT_LVD_LVW] = dummy_lvd_lvw_isr, /* Low Voltage Detect, Low Voltage Warning */
    [INT_LLW] = dummy_llw_isr, /* Low Leakage Wake-up */
    [INT_Watchdog] = dummy_watchdog_isr, /*  WDOG Interrupt */
    [INT_I2C0] = k64f_i2c0_isr, /* I2C0 interrupt */
    [INT_I2C1] = dummy_i2c1_isr, /* I2C1 interrupt */
    [INT_SPI0] = dummy_spi0_isr, /* SPI0 Interrupt */
    [INT_SPI1] = dummy_spi1_isr, /* SPI1 Interrupt */
    [INT_I2S0_Tx] = dummy_i2s0_tx_isr, /* I2S0 transmit interrupt */
    [INT_I2S0_Rx] = dummy_i2s0_rx_isr, /* I2S0 receive interrupt */
    [INT_UART0_LON] = dummy_uart0_lon_isr, /* UART0 LON interrupt */
    [INT_UART0_RX_TX] = k64f_uart0_rx_tx_isr, /* UART0 Receive/Transmit interrupt */
    [INT_UART0_ERR] = k64f_uart0_err_isr, /* UART0 Error interrupt */
    [INT_UART1_RX_TX] = dummy_uart1_rx_tx_isr, /* UART1 Receive/Transmit interrupt */
    [INT_UART1_ERR] = dummy_uart1_err_isr, /* UART1 Error interrupt */
    [INT_UART2_RX_TX] = dummy_uart2_rx_tx_isr, /* UART2 Receive/Transmit interrupt */
    [INT_UART2_ERR] = dummy_uart2_err_isr, /* UART2 Error interrupt */
    [INT_UART3_RX_TX] = dummy_uart3_rx_tx_isr, /* UART3 Receive/Transmit interrupt */
    [INT_UART3_ERR] = dummy_uart3_err_isr, /* UART3 Error interrupt */
    [INT_ADC0] = dummy_adc0_isr, /* ADC0 interrupt */
    [INT_CMP0] = dummy_cmp0_isr, /* CMP0 interrupt */
    [INT_CMP1] = dummy_cmp1_isr, /* CMP1 interrupt */
    [INT_FTM0] = dummy_ftm0_isr, /* FTM0 fault, overflow and channels interrupt */
    [INT_FTM1] = dummy_ftm1_isr, /* FTM1 fault, overflow and channels interrupt */
    [INT_FTM2] = dummy_ftm2_isr, /* FTM2 fault, overflow and channels interrupt */
    [INT_CMT] = dummy_cmt_isr, /* CMT interrupt */
    [INT_RTC] = dummy_rtc_alarm_isr, /* RTC Alarm interrupt */
    [INT_RTC_Seconds] = dummy_rtc_seconds_isr, /* RTC Seconds interrupt */
    [INT_PIT0] = dummy_pit0_isr, /* PIT timer channel 0 interrupt */
    [INT_PIT1] = dummy_pit1_isr, /* PIT timer channel 1 interrupt */
    [INT_PIT2] = dummy_pit2_isr, /* PIT timer channel 2 interrupt */
    [INT_PDB0] = dummy_pdb0_isr,  /* PDB0 Interrupt */
    [INT_USB0] = dummy_usb_otg_isr, /* USB interrupt */
    [INT_USBDCD] = dummy_usb_dcd_isr, /* USBDCD interrupt */
    [INT_Reserved71] = dummy_reserved71_isr, /*  Reserved interrupt 71 */
    [INT_DAC0] = dummy_dac0_isr, /* DAC0 interrupt */
    [INT_MCG] = dummy_mcg_isr, /* MCG Interrupt */
    [INT_LPTimer] = dummy_lptimer_isr, /* LPTimer interrupt */
    [INT_PORTA] = dummy_port_a_isr, /* Port A interrupt */
    [INT_PORTB] = dummy_port_b_isr, /* Port B interrupt */
    [INT_PORTC] = k64f_port_c_isr, /* Port C interrupt */
    [INT_PORTD] = dummy_port_d_isr, /* Port D interrupt */
    [INT_PORTE] = dummy_port_e_isr, /* Port E interrupt */
    [INT_SWI] = dummy_swi_isr, /* Software interrupt */
    [INT_SPI2] = dummy_spi2_isr, /* SPI2 Interrupt */
    [INT_UART4_RX_TX] = k64f_uart4_rx_tx_isr, /* UART4 Receive/Transmit interrupt */
    [INT_UART4_ERR] = k64f_uart4_err_isr, /* UART4 Error interrupt */
    [INT_UART5_RX_TX] = dummy_uart5_rx_tx_isr, /* UART5 Receive/Transmit interrupt */
    [INT_UART5_ERR] = dummy_uart5_err_isr, /* UART5 Error interrupt */
    [INT_CMP2] = dummy_cmp2_isr, /* CMP2 interrupt */
    [INT_FTM3] = dummy_ftm3_isr, /* FTM3 fault, overflow and channels interrupt */
    [INT_DAC1] = dummy_dac1_isr, /* DAC1 interrupt */
    [INT_ADC1] = dummy_adc1_isr, /* ADC1 interrupt */
    [INT_I2C2] = dummy_i2c2_isr, /* I2C2 interrupt */
    [INT_CAN0_ORed_Message_buffer] = dummy_can0_ored_message_buffer_isr, /* CAN0 OR'd message buffers interrupt */
    [INT_CAN0_Bus_Off] = dummy_can0_bus_off_isr, /* CAN0 bus off interrupt */
    [INT_CAN0_Error] = dummy_can0_error_isr, /* CAN0 error interrupt */
    [INT_CAN0_Tx_Warning] = dummy_can0_tx_warning_isr, /* CAN0 Tx warning interrupt */
    [INT_CAN0_Rx_Warning] = dummy_can0_rx_warning_isr, /* CAN0 Rx warning interrupt */
    [INT_CAN0_Wake_Up] = dummy_can0_wake_up_isr, /* CAN0 wake up interrupt */
    [INT_SDHC] = dummy_sdhc_isr, /* SDHC interrupt */
    [INT_ENET_1588_Timer] = dummy_enet_1588_timer_isr, /* Ethernet MAC IEEE 1588 Timer Interrupt */
    [INT_ENET_Transmit] = k64f_enet_transmit_isr, /* Ethernet MAC Transmit Interrupt */
    [INT_ENET_Receive] = k64f_enet_receive_isr, /* Ethernet MAC Receive Interrupt */
    [INT_ENET_Error] = k64f_enet_error_isr, /* Ethernet MAC Error and miscelaneous Interrupt */
};

C_ASSERT(
    ARRAY_SIZE(g_interrupt_vector_table) ==
    CORTEX_M_IRQ_VECTOR_BASE + SOC_NUM_INTERRUPT_CHANNELS);

/**
 * SoC configuration in Flash
 */
static const NV_Type nv_cfmconfig __attribute__ ((section(".cfmconfig"))) = {
    .BACKKEY3 = 0xff,
    .BACKKEY2 = 0xff,
    .BACKKEY1 = 0xff,
    .BACKKEY0 = 0xff,
    .BACKKEY7 = 0xff,
    .BACKKEY6 = 0xff,
    .BACKKEY5 = 0xff,
    .BACKKEY4 = 0xff,
    .FPROT3 = 0xff,
    .FPROT2 = 0xff,
    .FPROT1 = 0xff,
    .FPROT0 = 0xff,
    .FSEC = 0xfe,
    .FOPT = 0xff,
    .FEPROT = 0xff,
    .FDPROT = 0xff
};

/**
 * Configuration registers for all pin ports
 */
static const struct pin_config_registers g_pin_config_registers[NUM_PIN_PORTS] = {
    [PIN_PORT_A] = {
	    .pin_port_regs_p = PORTA,
	    .pin_gpio_regs_p = PTA,
    },

    [PIN_PORT_B] = {
	    .pin_port_regs_p = PORTB,
	    .pin_gpio_regs_p = PTB,
    },

    [PIN_PORT_C] = {
	    .pin_port_regs_p = PORTC,
	    .pin_gpio_regs_p = PTC,
    },

    [PIN_PORT_D] = {
	    .pin_port_regs_p = PORTD,
	    .pin_gpio_regs_p = PTD,
    },

    [PIN_PORT_E] = {
	    .pin_port_regs_p = PORTE,
	    .pin_gpio_regs_p = PTE,
    },
};

/**
 * Matrix to keep track of what pins are currently in use. If a pin is not in
 * use (set_pin_function() has not been called for it), its entry is NULL.
 */
static const struct pin_info *g_pins_in_use_map[NUM_PIN_PORTS][NUM_PINS_PER_PORT];

static struct mpu_device_var g_mpu_var = {
    .initialized = false,
    .num_regions = 0,
    .num_defined_global_regions = 0,
};

static const struct mpu_device g_mpu = {
    .signature = MPU_DEVICE_SIGNATURE,
    .mmio_regs_p = (volatile MPU_Type *)MPU_BASE,
    .var_p = &g_mpu_var,
};

static struct crc_device_var g_crc_var = {
    .initialized = false,
};

static const struct crc_device g_crc = {
    .signature = CRC_DEVICE_SIGNATURE,
    .mmio_regs_p = (volatile CRC_Type *)CRC_BASE,
    .var_p = &g_crc_var,
};

/**
 * McRTOS interrupt object for the UART0 Receive/Transmit interrupts
 */
struct rtos_interrupt *g_rtos_interrupt_uart0_rx_tx_p = NULL;

/**
 * McRTOS interrupt object for the UART0 Error interrupts
 */
struct rtos_interrupt *g_rtos_interrupt_uart0_err_p = NULL;

/**
 * McRTOS interrupt object for the UART4 Receive/Transmit interrupts
 */
struct rtos_interrupt *g_rtos_interrupt_uart4_rx_tx_p = NULL;

/**
 * McRTOS interrupt object for the UART4 Error interrupts
 */
struct rtos_interrupt *g_rtos_interrupt_uart4_err_p = NULL;

/**
 * Global array of non-const structures for UART devices
 * (allocated in SRAM space)
 */
static struct uart_device_var g_uart_devices_var[] =
{
    [0] = {
        .urt_initialized = false,
        .urt_received_bytes_dropped = 0,
        .urt_transmit_bytes_dropped = 0,
        },

    [4] = {
        .urt_initialized = false,
        .urt_received_bytes_dropped = 0,
        .urt_transmit_bytes_dropped = 0,
        },
};

static uint8_t uart0_transmit_queue_storage[UART_TRANSMIT_QUEUE_SIZE_IN_BYTES];
static uint8_t uart0_receive_queue_storage[UART_RECEIVE_QUEUE_SIZE_IN_BYTES];
static uint8_t uart4_transmit_queue_storage[UART_TRANSMIT_QUEUE_SIZE_IN_BYTES];
static uint8_t uart4_receive_queue_storage[UART_RECEIVE_QUEUE_SIZE_IN_BYTES];

/**
 * Global array of const structures for UART devices for the K64F SoC
 * (allocated in flash space)
 */
static const struct uart_device g_uart_devices[] =
{
    [0] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
	.urt_name = "UART0",
        .urt_var_p = &g_uart_devices_var[0],
        .urt_mmio_uart_p = UART0_BASE_PTR,
        .urt_tx_pin = PIN_INITIALIZER(PIN_PORT_B, 17, PIN_FUNCTION_ALT3),
        .urt_rx_pin = PIN_INITIALIZER(PIN_PORT_B, 16, PIN_FUNCTION_ALT3),
        .urt_mmio_clock_gate_reg_p = &SIM_SCGC4,
        .urt_mmio_clock_gate_mask = SIM_SCGC4_UART0_MASK,
	.urt_source_clock_freq_in_hz = CPU_CLOCK_FREQ_IN_HZ,
        .urt_rtos_interrupt_rx_tx_params = {
            .irp_name_p = "UART0 Receive/Transmit Interrupt",
            .irp_isr_function_p = k64f_uart0_rx_tx_isr,
            .irp_arg_p =  (void *)&g_uart_devices[0],
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART0_RX_TX),
            .irp_priority = UART0_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

        .urt_rtos_interrupt_rx_tx_pp = &g_rtos_interrupt_uart0_rx_tx_p,

	.urt_rtos_interrupt_err_params = {
            .irp_name_p = "UART0 Error Interrupt",
            .irp_isr_function_p = k64f_uart0_err_isr,
            .irp_arg_p =  (void *)&g_uart_devices[0],
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART0_ERR),
            .irp_priority = UART0_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

        .urt_rtos_interrupt_err_pp = &g_rtos_interrupt_uart0_err_p,

        .urt_transmit_queue_name_p = "UART0 transmit queue",
        .urt_receive_queue_name_p = "UART0 receive queue",
        .urt_transmit_queue_storage_p = uart0_transmit_queue_storage,
        .urt_receive_queue_storage_p = uart0_receive_queue_storage,
    },

    [4] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
	.urt_name = "UART4",
        .urt_var_p = &g_uart_devices_var[4],
        .urt_mmio_uart_p = UART4_BASE_PTR,
        .urt_tx_pin = PIN_INITIALIZER(PIN_PORT_C, 15, PIN_FUNCTION_ALT3),
        .urt_rx_pin = PIN_INITIALIZER(PIN_PORT_C, 14, PIN_FUNCTION_ALT3),
        .urt_mmio_clock_gate_reg_p = &SIM_SCGC1,
        .urt_mmio_clock_gate_mask = SIM_SCGC1_UART4_MASK,
	.urt_source_clock_freq_in_hz = CPU_CLOCK_FREQ_IN_HZ / 2,
        .urt_rtos_interrupt_rx_tx_params = {
            .irp_name_p = "UART4 Receive/Transmit Interrupt",
            .irp_isr_function_p = k64f_uart4_rx_tx_isr,
            .irp_arg_p =  (void *)&g_uart_devices[4],
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART4_RX_TX),
            .irp_priority = UART4_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

        .urt_rtos_interrupt_rx_tx_pp = &g_rtos_interrupt_uart4_rx_tx_p,

	.urt_rtos_interrupt_err_params = {
            .irp_name_p = "UART4 Error Interrupt",
            .irp_isr_function_p = k64f_uart4_err_isr,
            .irp_arg_p =  (void *)&g_uart_devices[4],
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART4_ERR),
            .irp_priority = UART4_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

        .urt_rtos_interrupt_err_pp = &g_rtos_interrupt_uart4_err_p,

        .urt_transmit_queue_name_p = "UART4 transmit queue",
        .urt_receive_queue_name_p = "UART4 receive queue",
        .urt_transmit_queue_storage_p = uart4_transmit_queue_storage,
        .urt_receive_queue_storage_p = uart4_receive_queue_storage,
    },
};

C_ASSERT(
    ARRAY_SIZE(g_uart_devices) == ARRAY_SIZE(g_uart_devices_var));

const struct uart_device *const g_console_serial_port_p = &g_uart_devices[0];
const struct uart_device *const g_bluetooth_serial_port_p = &g_uart_devices[4];

/**
 * McRTOS interrupt object for the I2C0 controller interrupts
 */
struct rtos_interrupt *g_rtos_interrupt_i2c0_p = NULL;

/**
 * Global non-const structure for I2C controller device
 * (allocated in SRAM space)
 */
static struct i2c_device_var g_i2c_devices_var[] = {
    [0] = {
        .i2c_initialized = false,
        .transaction = {
            .state = I2C_TRANSACTION_NOT_STARTED,
        },
    },

    [1] = {
        .i2c_initialized = false,
    },
};

/**
 * Global const structure for the I2C controller devices
 * (allocated in flash space)
 */
const struct i2c_device g_i2c_devices[] = {
    [0] = {
        .i2c_signature = I2C_DEVICE_SIGNATURE,
	.i2c_name = "I2C0",
        .i2c_var_p = &g_i2c_devices_var[0],
        .i2c_mmio_registers_p = I2C0_BASE_PTR,
        .i2c_scl_pin = PIN_INITIALIZER(PIN_PORT_E, 24, PIN_FUNCTION_ALT5),
        .i2c_sda_pin = PIN_INITIALIZER(PIN_PORT_E, 25, PIN_FUNCTION_ALT5),
        .i2c_clock_gate_mask = SIM_SCGC4_I2C0_MASK,
        .i2c_icr_value = 0x2d, /* 100KHz for bus clock of 60 MHz */
        .i2c_rtos_interrupt_params = {
            .irp_name_p = "I2C0 Interrupt",
            .irp_isr_function_p = k64f_i2c0_isr,
            .irp_arg_p = (void *)&g_i2c_devices[0],
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2C0),
            .irp_priority = I2C_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },
        .i2c_rtos_interrupt_pp = &g_rtos_interrupt_i2c0_p,
        .i2c_condvar_name = "I2C0 condvar",
    },

    [1] = {
        .i2c_signature = I2C_DEVICE_SIGNATURE,
	.i2c_name = "I2C1",
        .i2c_var_p = &g_i2c_devices_var[1],
        .i2c_mmio_registers_p = I2C1_BASE_PTR,
        .i2c_scl_pin = PIN_INITIALIZER(PIN_PORT_C, 10, PIN_FUNCTION_ALT2),
        .i2c_sda_pin = PIN_INITIALIZER(PIN_PORT_C, 11, PIN_FUNCTION_ALT2),
        .i2c_clock_gate_mask = SIM_SCGC4_I2C1_MASK,
        .i2c_icr_value = 0x2d, /* 100KHz for bus clock of 60 MHz */
    }
};

C_ASSERT(ARRAY_SIZE(g_i2c_devices) == ARRAY_SIZE(g_i2c_devices_var));


static void
pll_init(void)
{
    uint32_t reg_value;

    /*
     * Release hold with ACKISO: Only has an effect if recovering from VLLSx.
     * if ACKISO is set you must clear ackiso before initializing the PLL
     * if osc enabled in low power modes - enable it first before ack
     */
    reg_value = read_8bit_mmio_register(&PMC_REGSC);
    if (reg_value & PMC_REGSC_ACKISO_MASK) {
        reg_value |= PMC_REGSC_ACKISO_MASK;
	write_8bit_mmio_register(&PMC_REGSC, reg_value);
    }

    /* SIM->SOPT2: PLLFLLSEL=1 */
    reg_value = read_32bit_mmio_register(&SIM_SOPT2);
    /* Select PLL as a clock source for various peripherals */
    reg_value |= SIM_SOPT2_PLLFLLSEL(0x01);
    write_32bit_mmio_register(&SIM_SOPT2, reg_value);

    /* SIM->SOPT1: OSC32KSEL=3 */
    reg_value = read_32bit_mmio_register(&SIM_SOPT1);
    /* LPO 1kHz oscillator drives 32 kHz clock for various peripherals */
    reg_value |= SIM_SOPT1_OSC32KSEL(0x03);
    write_32bit_mmio_register(&SIM_SOPT1, reg_value);

    /* PORTA->PCR[18]: ISF=0,MUX=0 */
    reg_value = read_32bit_mmio_register(&PORTA_PCR18);
    reg_value &= ~(PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07));
    write_32bit_mmio_register(&PORTA_PCR18, reg_value);

    /* Switch to FBE Mode */
    /* MCG->C2: LOCRE0=0,?=0,RANGE=2,HGO=0,EREFS=0,LP=0,IRCS=0 */
    write_8bit_mmio_register(&MCG_C2, MCG_C2_RANGE(0x02));

    /* OSC->CR: ERCLKEN=1,?=0,EREFSTEN=0,?=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
    write_8bit_mmio_register(&OSC_CR, OSC_CR_ERCLKEN_MASK);

    /* MCG->C7: OSCSEL=0 */
    reg_value = read_8bit_mmio_register(&MCG_C7);
    reg_value &= ~MCG_C7_OSCSEL_MASK;
    write_8bit_mmio_register(&MCG_C7, reg_value);

    /* MCG->C1: CLKS=2,FRDIV=5,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    reg_value = (MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x05) | MCG_C1_IRCLKEN_MASK);
    write_8bit_mmio_register(&MCG_C1, reg_value);

    /* MCG->C4: DMX32=0,DRST_DRS=0 */
    reg_value = read_8bit_mmio_register(&MCG_C4);
    reg_value &= ~(MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS(0x03));
    write_8bit_mmio_register(&MCG_C4, reg_value);

    /* MCG->C5: ?=0,PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=0x13 */
    write_8bit_mmio_register(&MCG_C5, MCG_C5_PRDIV0(0x13));

    /* MCG->C6: LOLIE0=0,PLLS=0,CME0=0,VDIV0=0x18 */
    write_8bit_mmio_register(&MCG_C6, MCG_C6_VDIV0(0x18));

    /* Check that the source of the FLL reference clock is the external reference clock. */
    do {
        reg_value = read_8bit_mmio_register(&MCG_S);
    } while ((reg_value & MCG_S_IREFST_MASK) != 0x00U);

    /* Wait until external reference clock is selected as MCG output */
    do {
        reg_value = read_8bit_mmio_register(&MCG_S);
    } while((reg_value & 0x0CU) != 0x08U);

    /* Switch to PBE Mode */
    /* MCG->C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=0x18 */
    reg_value = (MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0x18));
    write_8bit_mmio_register(&MCG_C6, reg_value);

    /* Wait until external reference clock is selected as MCG output */
    do {
        reg_value = read_8bit_mmio_register(&MCG_S);
    } while ((reg_value & 0x0CU) != 0x08U);

    /* Wait until locked */
    do {
        reg_value = read_8bit_mmio_register(&MCG_S);
    } while ((reg_value & MCG_S_LOCK0_MASK) == 0x00U);

    /* Switch to PEE Mode */
    /* MCG->C1: CLKS=0,FRDIV=5,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
    reg_value = (MCG_C1_CLKS(0x00) | MCG_C1_FRDIV(0x05) | MCG_C1_IRCLKEN_MASK);
    write_8bit_mmio_register(&MCG_C1, reg_value);

    /* Wait until output of the PLL is selected */
    do {
        reg_value = read_8bit_mmio_register(&MCG_S);
    } while ((reg_value & 0x0CU) != 0x0CU);

    /* Set USB input clock to 48MHz  */
    /* SIM->CLKDIV2: USBDIV=4,USBFRAC=1 */
    reg_value = read_32bit_mmio_register(&SIM_CLKDIV2);
    reg_value &= ~SIM_CLKDIV2_USBDIV(0x03);
    reg_value |= (SIM_CLKDIV2_USBDIV(0x04) |
                  SIM_CLKDIV2_USBFRAC_MASK);
    write_32bit_mmio_register(&SIM_CLKDIV2, reg_value);
}


static void
system_clocks_init(void)
{
    uint32_t reg_value;

    /*
     * Enable all of the GPIO port clocks. These have to be enabled to configure
     * pin muxing options, so most code will need all of these on anyway.
     */
    reg_value = read_32bit_mmio_register(&SIM_SCGC5);
    reg_value |= (SIM_SCGC5_PORTA_MASK
                    | SIM_SCGC5_PORTB_MASK
                    | SIM_SCGC5_PORTC_MASK
                    | SIM_SCGC5_PORTD_MASK
                    | SIM_SCGC5_PORTE_MASK);
    write_32bit_mmio_register(&SIM_SCGC5, reg_value);

    /*
     * Set the system dividers:
     * SIM->CLKDIV1: OUTDIV1=0 (divide by 1)
     *		     OUTDIV2=1 (divide by 2)
     *		     OUTDIV3=1 (divide by 2)
     *		     OUTDIV4=4 (divide by 5)
     *
     * Core clock: MCG CLKOUT divided by OUTDIV1
     * System clock: MCG CLKOUT divided by OUTDIV1
     * Bus clock: MCG CLKOUT divided by OUTDIV2
     *
     * NOTE: To see which clocks are used for which devices see table 5-2,
     * page 191, section 5.7, K64 Sub-Family Reference Manual
     */
    reg_value = SIM_CLKDIV1_OUTDIV1(0x00) |
	        SIM_CLKDIV1_OUTDIV2(0x01) |
                SIM_CLKDIV1_OUTDIV3(0x02) |
                SIM_CLKDIV1_OUTDIV4(0x04);

    write_32bit_mmio_register(&SIM_CLKDIV1, reg_value);

    /*
     * Initialize PLL:
     *
     * NOTE: PLL will be the source for MCG CLKOUT so the core, system, and flash
     * clocks are derived from it
     */
    pll_init();
}


/**
 *  SoC-specific early initialization to be invoked at the beginning of
 *  the Reset exception handler.
 */
void
soc_early_init(void)
{
    uint32_t reg_value;

    /*
     * Disable the Watchdog because it will cause reset unless we have
     * refresh logic in place for the watchdog
     *
     * NOTE: First, we need to unlock the Watchdog, and to do so, two
     * writes must be done on the 'WDOG->UNLOCK' register without using
     * I/O accessors, due to strict timing requirements.
     */
    WDOG->UNLOCK = WDOG_UNLOCK_WDOGUNLOCK(0xC520); /* Key 1 */
    WDOG->UNLOCK = WDOG_UNLOCK_WDOGUNLOCK(0xD928); /* Key 2 */

    /*
     * WDOG->STCTRLH: ?=0,DISTESTWDOG=0,BYTESEL=0,TESTSEL=0,TESTWDOG=0,?=0,?=1,
     *	WAITEN=1,STOPEN=1,DBGEN=0,ALLOWUPDATE=1,WINEN=0,IRQRSTEN=0,CLKSRC=1,
     *	WDOGEN=0
     */
    reg_value = WDOG_STCTRLH_BYTESEL(0x00) |
                WDOG_STCTRLH_WAITEN_MASK |
                WDOG_STCTRLH_STOPEN_MASK |
                WDOG_STCTRLH_ALLOWUPDATE_MASK |
                WDOG_STCTRLH_CLKSRC_MASK |
                0x0100U;
    write_16bit_mmio_register(&WDOG_STCTRLH, reg_value);
}


/**
 * Configure an MPU region (region index must be non 0, as region 0 is special)
 * to be accessed from unprivileged mode on the given CPU
 */
static void
k64f_set_mpu_region_for_cpu(
    struct mpu_device_var *mpu_var_p,
    volatile MPU_Type *mpu_regs_p,
    cpu_id_t cpu_id,
    uint8_t region_index,
    void *start_addr,
    void *end_addr,
    uint32_t unprivileged_permissions)
{
    struct permissions_bit_field {
	uint32_t mask;
	uint32_t shift;
    };

    static const struct permissions_bit_field unprivileged_permissions_fields[] = {
        [0] = {
	    .mask = MPU_WORD_M0UM_MASK,
	    .shift = MPU_WORD_M0UM_SHIFT
	},
    };

    uint32_t reg_value;

    DBG_ASSERT(cpu_id < ARRAY_SIZE(unprivileged_permissions_fields),
 	       cpu_id, ARRAY_SIZE(unprivileged_permissions_fields));

    DBG_ASSERT(region_index > 0 && region_index < mpu_var_p->num_regions,
 	       region_index, mpu_var_p->num_regions);

    DBG_ASSERT(start_addr < end_addr &&
	       (uintptr_t)start_addr % K64F_MPU_REGION_ALIGNMENT == 0 &&
	       (uintptr_t)end_addr + 1 % K64F_MPU_REGION_ALIGNMENT == 0,
	       start_addr, end_addr);

    /*
     * Configure region:
     *
     * NOTE: writing to registers WORD[region_index][0], WORD[region_index][1] and
     * WORD[region_index][2] of the region descriptor 'region_index' will disable
     * the region (turn off bit MPU_WORD_VLD_MASK in WORD[region_index][3]):
     */
    reg_value = ((uintptr_t)start_addr & K64F_MPU_REGION_ALIGNMENT_MASK);
    write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][0], reg_value);

    reg_value = (((uintptr_t)end_addr & K64F_MPU_REGION_ALIGNMENT_MASK) |
		 (K64F_MPU_REGION_ALIGNMENT - 1));
    write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][1], reg_value);

    reg_value = read_32bit_mmio_register(&mpu_regs_p->WORD[region_index][2]);

    SET_BIT_FIELD(reg_value,
		  unprivileged_permissions_fields[cpu_id].mask,
		  unprivileged_permissions_fields[cpu_id].shift,
		  unprivileged_permissions);

    write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][2], reg_value);

    /*
     * Re-enable region:
     */
    write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][3], MPU_WORD_VLD_MASK);
}


static void
k64f_mpu_set_dma_region(
    uint8_t region_index,
    enum mpu_bus_masters mpu_bus_master,
    void *start_addr,
    void *end_addr)
{
    struct bus_master_type1_permissions_bit_field {
	uint32_t mask;
	uint32_t shift;
    };

    struct bus_master_type2_permissions_bit_field {
	uint32_t write_mask;
	uint32_t read_mask;
    };

    static const struct bus_master_type1_permissions_bit_field privileged_permissions_fields[] = {
        [0] = {
	    .mask = MPU_WORD_M0SM_MASK,
	    .shift = MPU_WORD_M0SM_SHIFT
	},
        [1] = {
	    .mask = MPU_WORD_M1SM_MASK,
	    .shift = MPU_WORD_M1SM_SHIFT
	},
        [2] = {
	    .mask = MPU_WORD_M2SM_MASK,
	    .shift = MPU_WORD_M2SM_SHIFT
	},
        [3] = {
	    .mask = MPU_WORD_M3SM_MASK,
	    .shift = MPU_WORD_M3SM_SHIFT
	},
    };

    static const struct bus_master_type1_permissions_bit_field unprivileged_permissions_fields[] = {
        [0] = {
	    .mask = MPU_WORD_M0UM_MASK,
	    .shift = MPU_WORD_M0UM_SHIFT
	},
        [1] = {
	    .mask = MPU_WORD_M1UM_MASK,
	    .shift = MPU_WORD_M1UM_SHIFT
	},
        [2] = {
	    .mask = MPU_WORD_M2UM_MASK,
	    .shift = MPU_WORD_M2UM_SHIFT
	},
        [3] = {
	    .mask = MPU_WORD_M3UM_MASK,
	    .shift = MPU_WORD_M3UM_SHIFT
	},
    };

    C_ASSERT2(assert_permissions_fields_same_size,
	  ARRAY_SIZE(privileged_permissions_fields) ==
	  ARRAY_SIZE(unprivileged_permissions_fields));

    static const struct bus_master_type2_permissions_bit_field rw_permissions_fields[] = {
        [4] = {
	    .write_mask = MPU_WORD_M4WE_MASK,
	    .read_mask = MPU_WORD_M4RE_MASK
	},

        [5] = {
	    .write_mask = MPU_WORD_M5WE_MASK,
	    .read_mask = MPU_WORD_M5RE_MASK
	},
        [6] = {
	    .write_mask = MPU_WORD_M6WE_MASK,
	    .read_mask = MPU_WORD_M6RE_MASK
	},
        [7] = {
	    .write_mask = MPU_WORD_M7WE_MASK,
	    .read_mask = MPU_WORD_M7RE_MASK
	},
    };

    C_ASSERT2(assert_rw_permissions_fields_size,
	      ARRAY_SIZE(rw_permissions_fields) > MPU_BUS_MASTER_SDHC);

    uint32_t reg_value;
    volatile MPU_Type *mpu_regs_p = g_mpu.mmio_regs_p;

    DEBUG_PRINTF("MPU region %u assigned to DMA bus master %u\n", region_index,
                 mpu_bus_master);
    DBG_ASSERT(region_index < RTOS_NUM_GLOBAL_MPU_REGIONS,
               region_index, 0);
    DBG_ASSERT(mpu_bus_master >= MPU_BUS_MASTER_DEBUGGER &&
	       mpu_bus_master <= MPU_BUS_MASTER_SDHC,
 	       mpu_bus_master, 0);
    DBG_ASSERT(start_addr < end_addr &&
              (uintptr_t)start_addr % K64F_MPU_REGION_ALIGNMENT == 0,
               start_addr, end_addr);

    /*
     * The region must be currently disabled:
     */
    reg_value = read_32bit_mmio_register(&mpu_regs_p->WORD[region_index][3]);
    FDC_ASSERT((reg_value & MPU_WORD_VLD_MASK) == 0, reg_value, 0);

    /*
     * Set region address range:
     */
    reg_value = ((uintptr_t)start_addr & K64F_MPU_REGION_ALIGNMENT_MASK);
    write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][0], reg_value);

    reg_value = (((uintptr_t)end_addr & K64F_MPU_REGION_ALIGNMENT_MASK) |
		 (K64F_MPU_REGION_ALIGNMENT - 1));
    write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][1], reg_value);

    /*
     * Set region permissions:
     */
    reg_value = read_32bit_mmio_register(&mpu_regs_p->WORD[region_index][2]);
    if (mpu_bus_master < ARRAY_SIZE(privileged_permissions_fields)) {
	SET_BIT_FIELD(reg_value,
		      privileged_permissions_fields[mpu_bus_master].mask,
		      privileged_permissions_fields[mpu_bus_master].shift,
		      0x2  /* privileged rw- */);

	SET_BIT_FIELD(reg_value,
		      unprivileged_permissions_fields[mpu_bus_master].mask,
		      unprivileged_permissions_fields[mpu_bus_master].shift,
		      0x6 /* unprivileged rw- */);
    } else {
	DBG_ASSERT(mpu_bus_master < ARRAY_SIZE(rw_permissions_fields),
	           mpu_bus_master, ARRAY_SIZE(rw_permissions_fields));

	reg_value |= rw_permissions_fields[mpu_bus_master].write_mask |
		     rw_permissions_fields[mpu_bus_master].read_mask;
    }

    write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][2], reg_value);

    /*
     * Enable region:
     */
    write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][3], MPU_WORD_VLD_MASK);
}


static void
k64f_mpu_init(void)
{
    C_ASSERT2(assert_soc_flash_base_aligned, SOC_FLASH_BASE % 32 == 0);
    C_ASSERT2(assert_soc_sram_base_aligned, SOC_SRAM_BASE % 32 == 0);
    C_ASSERT2(assert_soc_mmio_base1_aligned, SOC_PERIPHERAL_BRIDGE_MIN_ADDR % 32 == 0);
    C_ASSERT2(assert_soc_mmio_base2_aligned, SOC_PRIVATE_PERIPHERALS_MIN_ADDR % 32 == 0);
    C_ASSERT2(assert_enough_mpu_regions, RTOS_NUM_GLOBAL_MPU_REGIONS >= 2);

    extern uint32_t __flash_text_start[];
    extern uint32_t __flash_text_end[];

    static const uint8_t num_mpu_regions_table[] = { 8, 12, 16 };
    uint32_t reg_value;

    FDC_ASSERT(
        g_mpu.signature == MPU_DEVICE_SIGNATURE,
        g_mpu.signature, 0);

    struct mpu_device_var *mpu_var_p = g_mpu.var_p;
    FDC_ASSERT(!mpu_var_p->initialized, mpu_var_p, 0);

    volatile MPU_Type *mpu_regs_p = g_mpu.mmio_regs_p;

    /*
     * Verify that the MPU has enough regions:
     */
    reg_value = read_32bit_mmio_register(&mpu_regs_p->CESR);
    uint8_t nrgd_field = GET_BIT_FIELD(reg_value, MPU_CESR_NRGD_MASK,
				       MPU_CESR_NRGD_SHIFT);
    FDC_ASSERT(nrgd_field <= 0x2, nrgd_field, 0);

    mpu_var_p->num_regions = num_mpu_regions_table[nrgd_field];

    FDC_ASSERT(mpu_var_p->num_regions >= RTOS_MAX_MPU_REGIONS,
	       mpu_var_p->num_regions, RTOS_MAX_MPU_REGIONS);

    /*
     * Disable MPU to configure it:
     */
    write_32bit_mmio_register(&mpu_regs_p->CESR, 0);

    /*
     * Make region 0 non-accessible for CPU unprivileged mode and for DMA.
     * Region 0 is used a background region accessible only for CPU
     * privileged mode.
     *
     * NOTE: Region 0 is defined by default as the whole address space.
     * Only its permissions can be changed. To avoid disabling region
     * region 0, while changing its access permissions, modify register
     * RGDAAC[0] instead of WORD[0][2]
     */
    write_32bit_mmio_register(&mpu_regs_p->RGDAAC[0], 0);
    mpu_var_p->num_defined_global_regions ++;

    /*
     * Make region 1, the code in flash to be executable in unprivileged mode
     */
    for (cpu_id_t cpu_id = 0; cpu_id < SOC_NUM_CPU_CORES; cpu_id ++) {
	k64f_set_mpu_region_for_cpu(mpu_var_p, mpu_regs_p, cpu_id, 1,
				    (void *)__flash_text_start,
				    (void *)__flash_text_end,
				    0x5); /* unprivileged r-x */
    }

    mpu_var_p->num_defined_global_regions ++;

    /*
     * Set remaining regions as invalid to save power
     */
    for (uint_fast8_t region_index = 2; region_index < mpu_var_p->num_regions;
         region_index ++) {
	write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][3],
				  ~MPU_WORD_VLD_MASK);
    }

    /*
     * Enable MPU:
     */
    write_32bit_mmio_register(&mpu_regs_p->CESR, MPU_CESR_VLD_MASK);
    mpu_var_p->initialized = true;
    capture_fdc_msg_printf("K64F MPU initialized (regions: %u)\n", g_mpu.var_p->num_regions);
}


/*
 * Disable MPU
 */
void
mpu_disable(void)
{
#   ifdef _RELIABILITY_CHECKS_
    struct mpu_device_var *mpu_var_p = g_mpu.var_p;
#   endif
    volatile MPU_Type *mpu_regs_p = g_mpu.mmio_regs_p;

    FDC_ASSERT(g_mpu.signature == MPU_DEVICE_SIGNATURE, g_mpu.signature, 0);
    FDC_ASSERT(mpu_var_p->initialized, mpu_var_p, 0);

    write_32bit_mmio_register(&mpu_regs_p->CESR, 0);
}


/*
 * Enable MPU
 */
void
mpu_enable(void)
{
#   ifdef _RELIABILITY_CHECKS_
    struct mpu_device_var *mpu_var_p = g_mpu.var_p;
#   endif
    volatile MPU_Type *mpu_regs_p = g_mpu.mmio_regs_p;

    FDC_ASSERT(g_mpu.signature == MPU_DEVICE_SIGNATURE, g_mpu.signature, 0);
    FDC_ASSERT(mpu_var_p->initialized, mpu_var_p, 0);

    write_32bit_mmio_register(&mpu_regs_p->CESR, MPU_CESR_VLD_MASK);
}


/*
 *
 * Set all the thread-specific data regions for the current thread, including
 * the stack region.
 *
 * NOTE: This function is to be invoked as part of a thread context switch
 */
void
mpu_set_all_thread_data_regions(
    cpu_id_t cpu_id,
    struct mpu_region_range regions[RTOS_NUM_THREAD_MPU_REGIONS])
{
    DBG_ASSERT(
        g_mpu.signature == MPU_DEVICE_SIGNATURE,
        g_mpu.signature, 0);

    struct mpu_device_var *mpu_var_p = g_mpu.var_p;
    DBG_ASSERT(mpu_var_p->initialized, mpu_var_p, 0);

    volatile MPU_Type *mpu_regs_p = g_mpu.mmio_regs_p;
    uint_fast8_t region_index = FIRST_THREAD_MPU_REGION_INDEX;

    for (uint_fast8_t i = 0; i < RTOS_NUM_THREAD_MPU_REGIONS; i++) {
        struct mpu_region_range *region_p = &regions[i];

        if (region_p->flags & MPU_REGION_INACTIVE) {
            write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][3],
                                      ~MPU_WORD_VLD_MASK);
        } else {
            uint32_t unprivileged_permissions;

            DBG_ASSERT(region_p->start_addr != NULL, i, 0);
            if (region_p->flags & MPU_REGION_READ_ONLY) {
                unprivileged_permissions = 0x4; /* r-- */
            } else {
                unprivileged_permissions = 0x6; /* rw- */
            }

            k64f_set_mpu_region_for_cpu(mpu_var_p, mpu_regs_p, cpu_id, region_index,
                                        region_p->start_addr, region_p->end_addr,
                                        unprivileged_permissions);
        }

	region_index ++;
    }
}


/**
 * Sets the MPU region descriptor specified by 'region_index' for the give CPU
 *
 * @param cpu_id: CPU ID
 * @param region_index: Index of the corresponding MPU region descriptor
 * @param start_addr: Address of fist byte of the memory region
 * @param end_addr: Address of the last byte of the memory region
 * @param flags: permission flags
 */
void
mpu_set_thread_data_region(
    cpu_id_t cpu_id,
    mpu_region_index_t region_index,
    void *start_addr,
    void *end_addr,
    uint32_t flags)
{
    uint32_t unprivileged_permissions;

    FDC_ASSERT(
        g_mpu.signature == MPU_DEVICE_SIGNATURE,
        g_mpu.signature, 0);

    struct mpu_device_var *mpu_var_p = g_mpu.var_p;
    FDC_ASSERT(mpu_var_p->initialized, mpu_var_p, 0);

    volatile MPU_Type *mpu_regs_p = g_mpu.mmio_regs_p;

    FDC_ASSERT(start_addr != NULL, start_addr, end_addr);
    FDC_ASSERT(start_addr < end_addr, start_addr, end_addr);
    FDC_ASSERT(region_index >= FIRST_THREAD_MPU_REGION_INDEX &&
               region_index < RTOS_MAX_MPU_REGIONS,
	       region_index, FIRST_THREAD_MPU_REGION_INDEX);

    if (flags & MPU_REGION_READ_ONLY) {
        unprivileged_permissions = 0x4; /* r-- */
    } else {
        unprivileged_permissions = 0x6; /* rw- */
    }

    k64f_set_mpu_region_for_cpu(mpu_var_p, mpu_regs_p, cpu_id, region_index,
				start_addr, end_addr,
				unprivileged_permissions);
}


void
mpu_unset_thread_data_region(mpu_region_index_t region_index)
{
    FDC_ASSERT(
        g_mpu.signature == MPU_DEVICE_SIGNATURE,
        g_mpu.signature, 0);

#   ifdef _RELIABILITY_CHECKS_
    struct mpu_device_var *mpu_var_p = g_mpu.var_p;
    FDC_ASSERT(mpu_var_p->initialized, mpu_var_p, 0);
#   endif

    volatile MPU_Type *mpu_regs_p = g_mpu.mmio_regs_p;

    FDC_ASSERT(region_index >= FIRST_THREAD_MPU_REGION_INDEX &&
               region_index < RTOS_MAX_MPU_REGIONS,
	       region_index, FIRST_THREAD_MPU_REGION_INDEX);

    /*
     * Set region as invalid
     */
    write_32bit_mmio_register(&mpu_regs_p->WORD[region_index][3],
			      ~MPU_WORD_VLD_MASK);
}


void
mpu_register_dma_region(
    enum mpu_bus_masters dma_bus_master,
    void *start_addr,
    size_t size)
{
    FDC_ASSERT(
        g_mpu.signature == MPU_DEVICE_SIGNATURE,
        g_mpu.signature, 0);

    struct mpu_device_var *mpu_var_p = g_mpu.var_p;

    bool caller_was_privileged = rtos_enter_privileged_mode();

    FDC_ASSERT(mpu_var_p->initialized, mpu_var_p, 0);
    FDC_ASSERT((uintptr_t)start_addr % K64F_MPU_REGION_ALIGNMENT == 0 &&
               size != 0, start_addr, size);

    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();
    uint8_t region_index = mpu_var_p->num_defined_global_regions;

    if (region_index == RTOS_NUM_GLOBAL_MPU_REGIONS) {
        fdc_error_t fdc_error =
            CAPTURE_FDC_ERROR("No global MPU regions left", 0, 0);

        fatal_error_handler(fdc_error);
    }

    mpu_var_p->num_defined_global_regions ++;
    rtos_k_restore_cpu_interrupts(cpu_status_register);

    void *end_addr = (void *)((uintptr_t)start_addr + size);

    k64f_mpu_set_dma_region(region_index, dma_bus_master,
                            start_addr, end_addr);

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


void
mpu_get_enclosing_region_boundaries(void *start_addr, void *end_addr,
                                    void **region_start_addr, void **region_end_addr)
{
    DBG_ASSERT(start_addr < end_addr, start_addr, end_addr);

    *region_start_addr = (void *)ROUND_DOWN((uintptr_t)start_addr, MIN_MPU_REGION_ALIGNMENT);
    *region_end_addr = (void *)ROUND_UP((uintptr_t)end_addr, MIN_MPU_REGION_ALIGNMENT);
}


static void
crc_init(void)
{
    uint32_t reg_value;

    FDC_ASSERT(
        g_crc.signature == CRC_DEVICE_SIGNATURE,
        g_crc.signature, 0);

    struct crc_device_var *crc_var_p = g_crc.var_p;
    FDC_ASSERT(!crc_var_p->initialized, crc_var_p, 0);

    volatile CRC_Type *crc_regs_p = g_crc.mmio_regs_p;

    /*
     * Enable the Clock to the CRC Module
     */
    reg_value = read_32bit_mmio_register(&SIM_SCGC6);
    reg_value |= SIM_SCGC6_CRC_MASK;
    write_32bit_mmio_register(&SIM_SCGC6, reg_value);

    write_32bit_mmio_register(&crc_regs_p->CTRL, 0);
    crc_var_p->initialized = true;
}


uint32_t
calc_crc_32(const void *data_buf_p, size_t num_bytes)
{
#   define CRC_32_POLYNOMIAL UINT32_C(0x04C11DB7)

    uint32_t reg_value;
    struct crc_device_var *crc_var_p = g_crc.var_p;
    bool privileged_caller = rtos_enter_privileged_mode();

    FDC_ASSERT(crc_var_p->initialized, crc_var_p, 0);

    volatile CRC_Type *crc_regs_p = g_crc.mmio_regs_p;

    /*
     * Configure CRC functionality:
     * - Select 32-bit CRC
     * - Don't do 1's complement of input data
     * - Bits in bytes are transposed but bytes are not transposed, when writing
     *   to DATA register.
     * - Both bits in bytes and bytes are transposed when reading DATA register
     */
    reg_value = read_32bit_mmio_register(&crc_regs_p->CTRL);
    reg_value |= CRC_CTRL_TCRC_MASK;
    reg_value &= ~CRC_CTRL_FXOR_MASK;
    SET_BIT_FIELD(reg_value, CRC_CTRL_TOT_MASK, CRC_CTRL_TOT_SHIFT, 0x1);
    SET_BIT_FIELD(reg_value, CRC_CTRL_TOTR_MASK, CRC_CTRL_TOTR_SHIFT, 0x2);
    write_32bit_mmio_register(&crc_regs_p->CTRL, reg_value);

    /*
     * Program CRC-32 polynomial
     */
    write_32bit_mmio_register(&crc_regs_p->GPOLY, CRC_32_POLYNOMIAL);

    /*
     * Program seed
     */
    reg_value = read_32bit_mmio_register(&crc_regs_p->CTRL);
    reg_value |= CRC_CTRL_WAS_MASK;
    write_32bit_mmio_register(&crc_regs_p->CTRL, reg_value);
    write_32bit_mmio_register(&crc_regs_p->DATA, UINT32_MAX);
    reg_value &= ~CRC_CTRL_WAS_MASK;
    write_32bit_mmio_register(&crc_regs_p->CTRL, reg_value);

    /*
     * Write data values (most significant byte first - Big endian):
     */
    size_t num_words = num_bytes / 4;
    size_t remaining_bytes = num_bytes % 4;
    const uint8_t *p = data_buf_p;
    const uint8_t *end_p = p + num_words * 4;

    for ( ; p != end_p; p += 4) {
#if 1
        reg_value = (((uint32_t)p[0] << 24) |
                     ((uint32_t)p[1] << 16) |
                     ((uint32_t)p[2] << 8) |
                     (uint32_t)p[3]);
        write_32bit_mmio_register(&crc_regs_p->DATA, reg_value);
#else
        write_8bit_mmio_register(&crc_regs_p->ACCESS8BIT.DATAHU, p[0]);
        write_8bit_mmio_register(&crc_regs_p->ACCESS8BIT.DATAHL, p[1]);
        write_8bit_mmio_register(&crc_regs_p->ACCESS8BIT.DATALU, p[2]);
        write_8bit_mmio_register(&crc_regs_p->ACCESS8BIT.DATALL, p[3]);
#endif
    }

    switch (remaining_bytes) {
    case 3:
        write_8bit_mmio_register(&crc_regs_p->ACCESS8BIT.DATAHU, p[0]);
        write_8bit_mmio_register(&crc_regs_p->ACCESS8BIT.DATAHL, p[1]);
        write_8bit_mmio_register(&crc_regs_p->ACCESS8BIT.DATALU, p[2]);
        break;
    case 2:
        write_8bit_mmio_register(&crc_regs_p->ACCESS8BIT.DATAHU, p[0]);
        write_8bit_mmio_register(&crc_regs_p->ACCESS8BIT.DATAHL, p[1]);
        break;
    case 1:
        write_8bit_mmio_register(&crc_regs_p->ACCESS8BIT.DATAHU, p[0]);
        break;
    }

    /*
     * Retrieve calculated CRC:
     */
    reg_value = read_32bit_mmio_register(&crc_regs_p->DATA);
    if (!privileged_caller) {
        rtos_exit_privileged_mode();
    }

    return reg_value;
}


/**
 *  Initializes board hardware.
 *
 *  @pre This function must be called with interrupts disabled.
 */
cpu_reset_cause_t
soc_hardware_init(void)
{
#   ifdef _RELIABILITY_CHECKS_
    cpu_status_register_t reg_primask = __get_PRIMASK();

    FDC_ASSERT(
        CPU_INTERRUPTS_ARE_DISABLED(reg_primask),
        reg_primask, 0);
#   endif

    cpu_reset_cause_t reset_cause = find_reset_cause();

    capture_fdc_msg_printf("Last reset cause: %#x\n", reset_cause);

    system_clocks_init();

#   ifdef _CPU_CYCLES_MEASURE_
    init_cpu_clock_cycles_counter();
#   endif

    capture_fdc_msg_printf("SoC model: %#x, SoC ID: %x-%x-%x\n",
                           SIM_SDID, SIM_UIDMH, SIM_UIDML, SIM_UIDL);

    cortex_m_nvic_init();

    uart_init(
        g_console_serial_port_p,
        CONSOLE_SERIAL_PORT_BAUD_RATE,
        CONSOLE_SERIAL_PORT_MODE);

    capture_fdc_msg_printf("%s initialized (Tx FIFO size: %u, Rx FIFO size: %u)\n",
		 g_console_serial_port_p->urt_name,
		 g_console_serial_port_p->urt_var_p->urt_tx_fifo_size,
		 g_console_serial_port_p->urt_var_p->urt_rx_fifo_size);

    k64f_mpu_init();
    i2c_init(&g_i2c_devices[0]);
    crc_init();
    enet_init(&g_enet_device0);
    return reset_cause;
}


void
soc_reset(void)
{
    __disable_irq();

    /*
     * Stop all peripherals:
     */
    i2c_shutdown(&g_i2c_devices[0]);
    uart_stop(g_console_serial_port_p);

    /*
     * Trigger software reset in the SoC:
     */
    NVIC_SystemReset();
}


static cpu_reset_cause_t
find_reset_cause(void)
{
    uint8_t generic_cause = GRC_INVALID_RESET_CAUSE;

     uint8_t reg_rcm_srs0 = read_8bit_mmio_register(&RCM_SRS0);
    if (reg_rcm_srs0 & RCM_SRS0_POR_MASK) {
        generic_cause = GRC_POWER_ON_RESET;
    } else if (reg_rcm_srs0 & RCM_SRS0_PIN_MASK) {
        generic_cause = GRC_EXTERNAL_PIN_RESET;
    } else if (reg_rcm_srs0 & RCM_SRS0_WDOG_MASK) {
        generic_cause = GRC_WATCHDOG_RESET;
    } else if (reg_rcm_srs0 != 0) {
        generic_cause = GRC_OTHER_HW_REASON_RESET;
    }

    uint8_t reg_rcm_srs1 = read_8bit_mmio_register(&RCM_SRS1);
    if (generic_cause == GRC_INVALID_RESET_CAUSE) {
        if (reg_rcm_srs1 & RCM_SRS1_SW_MASK) {
            generic_cause = GRC_SOFTWARE_RESET;
        } else if (reg_rcm_srs1 & RCM_SRS1_MDM_AP_MASK) {
            generic_cause = GRC_EXTERNAL_DEBUGGER_RESET;
        } else if (reg_rcm_srs1 & RCM_SRS1_LOCKUP_MASK) {
            generic_cause = GRC_LOCKUP_EVENT_RESET;
        } else if (reg_rcm_srs1 & RCM_SRS1_SACKERR_MASK) {
            generic_cause = GRC_STOP_ACK_ERROR_RESET;
        }
    }

    cpu_reset_cause_t reset_cause = 0x0;
    SET_BIT_FIELD(
        reset_cause, CPU_RESET_GENERIC_CAUSE_MASK,
        CPU_RESET_GENERIC_CAUSE_SHIFT, generic_cause);

    SET_BIT_FIELD(
        reset_cause, CPU_RESET_MACHDEP_CAUSE1_MASK,
        CPU_RESET_MACHDEP_CAUSE1_SHIFT, reg_rcm_srs0);

    SET_BIT_FIELD(
        reset_cause, CPU_RESET_MACHDEP_CAUSE2_MASK,
        CPU_RESET_MACHDEP_CAUSE2_SHIFT, reg_rcm_srs1);

    return reset_cause;
}


bool
software_reset_happened(void)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();

    bool result = ((RCM_SRS1 & RCM_SRS1_SW_MASK) != 0);

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }

    return result;
}


#ifdef _CPU_CYCLES_MEASURE_
static void
init_cpu_clock_cycles_counter(void)
{
    uint32_t reg_value;

    /*
     * Enable DWT and ITM blocks of the Cortex-M4 core:
     */
    reg_value = read_32bit_mmio_register(&CoreDebug->DEMCR);
    reg_value |= CoreDebug_DEMCR_TRCENA_Msk;
    write_32bit_mmio_register(&CoreDebug->DEMCR, reg_value);

    /*
     * Reset CPU clock cycle counter:
     */
    write_32bit_mmio_register(&DWT->CYCCNT, 0);

    /*
     * Enable DWT's CYCCNT:
     */

    reg_value = read_32bit_mmio_register(&DWT->CTRL);

    /*
     * Check that this cortex-M4 core does implement CYCCNT:
     */
    FDC_ASSERT((reg_value & DWT_CTRL_NOCYCCNT_Msk) == 0,
	       reg_value, 0);

    reg_value |= DWT_CTRL_CYCCNTENA_Msk;
    write_32bit_mmio_register(&DWT->CTRL, reg_value);
}


uint32_t
get_cpu_clock_cycles(void)
{
#if 0
    uint32_t low_reg_value = read_32bit_mmio_register(&PIT_LTMR64L);

    /*
     * TODO: PIT module frequency is half of the CPU frequency, so may
     * need to multiply by 2 to get the accurate number CPU clock cycles.
     */
    return low_reg_value * 2;
#else
    return DWT->CYCCNT;
#endif
}
#endif /* _CPU_CYCLES_MEASURE_ */

void set_pin_function(const struct pin_info *pin_p, uint32_t pin_flags)
{
    const struct pin_info **pins_in_use_entry_p =
	&g_pins_in_use_map[pin_p->pin_port][pin_p->pin_index];

    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();
    if (*pins_in_use_entry_p != NULL) {
	fdc_error_t fdc_error =
            CAPTURE_FDC_ERROR("Pin already allocated", pin_p->pin_port,
			      pin_p->pin_index);

        fatal_error_handler(fdc_error);
    }

    volatile PORT_Type *port_regs_p =
	g_pin_config_registers[pin_p->pin_port].pin_port_regs_p;

    write_32bit_mmio_register(
        &port_regs_p->PCR[pin_p->pin_index],
        PORT_PCR_MUX(pin_p->pin_function) | pin_flags);

    *pins_in_use_entry_p = pin_p;
}


/**
 * It configures a GPIO pin of the K64F SoC
 */
void
configure_gpio_pin(const struct gpio_pin *gpio_pin_p, uint32_t pin_flags,
		   bool is_output)
{
    uint32_t reg_value;

    bool caller_was_privileged = rtos_enter_privileged_mode();

    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    set_pin_function(&gpio_pin_p->pin_info, pin_flags);

    volatile GPIO_Type *gpio_regs_p =
	g_pin_config_registers[gpio_pin_p->pin_info.pin_port].pin_gpio_regs_p;

   if (is_output) {
        reg_value = read_32bit_mmio_register(&gpio_regs_p->PDDR);
        reg_value |= gpio_pin_p->pin_bit_mask;
        write_32bit_mmio_register(&gpio_regs_p->PDDR, reg_value);
    } else {
        reg_value = read_32bit_mmio_register(&gpio_regs_p->PDDR);
        reg_value &= ~gpio_pin_p->pin_bit_mask;
        write_32bit_mmio_register(&gpio_regs_p->PDDR, reg_value);
    }

    rtos_k_restore_cpu_interrupts(cpu_status_register);

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


void
activate_output_pin(const struct gpio_pin *gpio_pin_p)
{
    volatile GPIO_Type *gpio_regs_p =
	g_pin_config_registers[gpio_pin_p->pin_info.pin_port].pin_gpio_regs_p;

    bool caller_was_privileged = rtos_enter_privileged_mode();

#   ifdef DEBUG
    uint32_t reg_value = read_32bit_mmio_register(&gpio_regs_p->PDDR);

    FDC_ASSERT(
        reg_value & gpio_pin_p->pin_bit_mask,
        gpio_pin_p, reg_value);
#   endif

    if (gpio_pin_p->pin_is_active_high) {
        write_32bit_mmio_register(&gpio_regs_p->PSOR, gpio_pin_p->pin_bit_mask);
    } else {
        write_32bit_mmio_register(
            &gpio_regs_p->PCOR, gpio_pin_p->pin_bit_mask);
    }

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


void
deactivate_output_pin(const struct gpio_pin *gpio_pin_p)
{
    volatile GPIO_Type *gpio_regs_p =
	g_pin_config_registers[gpio_pin_p->pin_info.pin_port].pin_gpio_regs_p;

    bool caller_was_privileged = rtos_enter_privileged_mode();

#   ifdef DEBUG
    uint32_t reg_value = read_32bit_mmio_register(&gpio_regs_p->PDDR);

    FDC_ASSERT(
        reg_value & gpio_pin_p->pin_bit_mask,
        reg_value, gpio_pin_p->pin_bit_mask);
#   endif

    if (gpio_pin_p->pin_is_active_high) {
        write_32bit_mmio_register(&gpio_regs_p->PCOR, gpio_pin_p->pin_bit_mask);
    } else {
        write_32bit_mmio_register(&gpio_regs_p->PSOR, gpio_pin_p->pin_bit_mask);
    }

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


void
toggle_output_pin(const struct gpio_pin *gpio_pin_p)
{
    volatile GPIO_Type *gpio_regs_p =
	g_pin_config_registers[gpio_pin_p->pin_info.pin_port].pin_gpio_regs_p;

    bool caller_was_privileged = rtos_enter_privileged_mode();

#   ifdef DEBUG
    uint32_t reg_value = read_32bit_mmio_register(&gpio_regs_p->PDDR);

    FDC_ASSERT(
        reg_value & gpio_pin_p->pin_bit_mask,
        gpio_pin_p, reg_value);
#   endif

    write_32bit_mmio_register(
        &gpio_regs_p->PTOR, gpio_pin_p->pin_bit_mask);

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


bool
read_input_pin(const struct gpio_pin *gpio_pin_p)
{
    volatile GPIO_Type *gpio_regs_p =
	g_pin_config_registers[gpio_pin_p->pin_info.pin_port].pin_gpio_regs_p;

    bool caller_was_privileged = rtos_enter_privileged_mode();

    uint32_t reg_value = read_32bit_mmio_register(&gpio_regs_p->PDIR);

    bool result = ((reg_value & gpio_pin_p->pin_bit_mask) != 0);

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }

    return result;
}


void
uart_init(
    const struct uart_device *uart_device_p,
    uint32_t baud_rate,
    uint8_t mode)
{
    uint32_t reg_value;

    FDC_ASSERT(
        uart_device_p->urt_signature == UART_DEVICE_SIGNATURE,
        uart_device_p->urt_signature, uart_device_p);

    struct uart_device_var *uart_var_p = uart_device_p->urt_var_p;
    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    FDC_ASSERT(!uart_var_p->urt_initialized, uart_device_p, 0);
    FDC_ASSERT(mode == 0, mode, uart_device_p);
    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();

    /*
     * Enable clock for the UART:
     */
    reg_value = read_32bit_mmio_register(uart_device_p->urt_mmio_clock_gate_reg_p);
    reg_value |= uart_device_p->urt_mmio_clock_gate_mask;
    write_32bit_mmio_register(uart_device_p->urt_mmio_clock_gate_reg_p, reg_value);

    /*
     * Disable UART's transmitter and receiver, while UART is being
     * configured:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);

    /*
     * Configure the uart for 8-bit mode, no parity, 1 stop bit (mode is 0):
     */
    write_8bit_mmio_register(
        &UART_C1_REG(uart_mmio_registers_p), mode);

    /*
     * Get Tx FIFO size
     */
    reg_value = read_8bit_mmio_register(&UART_PFIFO_REG(uart_mmio_registers_p));
    uint8_t fifo_size_field = GET_BIT_FIELD(reg_value,
					    UART_PFIFO_TXFIFOSIZE_MASK,
					    UART_PFIFO_TXFIFOSIZE_SHIFT);
    if (fifo_size_field == 0x0) {
        uart_var_p->urt_tx_fifo_size = 1;
    } else {
        uart_var_p->urt_tx_fifo_size = 1 << (fifo_size_field + 1);
    }

    /*
     * Get Rx FIFO size:
     */
    fifo_size_field = GET_BIT_FIELD(reg_value,
				    UART_PFIFO_RXFIFOSIZE_MASK,
				    UART_PFIFO_RXFIFOSIZE_SHIFT);
    if (fifo_size_field == 0x0) {
        uart_var_p->urt_rx_fifo_size = 1;
    } else {
        uart_var_p->urt_rx_fifo_size = 1 << (fifo_size_field + 1);
    }

    /*
     * Configure Tx and RX FIFOs:
     * - Tx FIFO water mark = 0 (genarate interrupt when Tx FIFO is empty)
     * - Rx FIFO water mark = 1 (genarate interrupt when Rx FIFO is not empty)
     * - Enable Tx and Rx FIFOs
     * - Flush Tx and Rx FIFOs
     */
    write_8bit_mmio_register(
        &UART_TWFIFO_REG(uart_mmio_registers_p), 0);
    write_8bit_mmio_register(
        &UART_RWFIFO_REG(uart_mmio_registers_p), 1);
    write_8bit_mmio_register(
        &UART_PFIFO_REG(uart_mmio_registers_p),
	UART_PFIFO_TXFE_MASK | UART_PFIFO_RXFE_MASK);
    write_8bit_mmio_register(
        &UART_CFIFO_REG(uart_mmio_registers_p),
	UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK);

    uart_var_p->urt_fifos_enabled = true;

    /*
     * Configure Tx and Rx pins:
     */
    set_pin_function(&uart_device_p->urt_tx_pin, PORT_PCR_DSE_MASK);
    if (uart_device_p == g_bluetooth_serial_port_p) { //???
        set_pin_function(&uart_device_p->urt_rx_pin, PORT_PCR_DSE_MASK|PORT_PCR_PS_MASK|PORT_PCR_PE_MASK);
    } else {
        set_pin_function(&uart_device_p->urt_rx_pin, PORT_PCR_DSE_MASK);
    }

    /*
    * Calculate baud rate settings:
    */
    uint32_t uart_clk = uart_device_p->urt_source_clock_freq_in_hz;
    uint16_t sbr_val = uart_clk / (baud_rate * 16);

    FDC_ASSERT(sbr_val >= 1 && sbr_val <= 0x1FFF, uart_device_p, sbr_val);

     /*
      * Set SBR high-part field in the UART's BDH register:
      */
    reg_value = read_8bit_mmio_register(&UART_BDH_REG(uart_mmio_registers_p));
    SET_BIT_FIELD(
        reg_value, UART_BDH_SBR_MASK, UART_BDH_SBR_SHIFT,
        (sbr_val & 0x1F00) >> 8);
    write_8bit_mmio_register(
        &UART_BDH_REG(uart_mmio_registers_p), reg_value);

     /*
      * Set SBR low byte in the UART's BDL register:
      */
    write_8bit_mmio_register(
        &UART_BDL_REG(uart_mmio_registers_p),
        sbr_val & UART_BDL_SBR_MASK);

#   if 0 //???
    /*
     * Determine if a fractional divider is needed to fine tune closer to the
     * desired baud rate. Each value of brfa is in 1/32 increments, hence the
     * multiply-by-32
     */
    uint16_t brfa_val = (32*uart_clk / (baud_rate*16)) - 32*sbr_val;

    /*
     * Set BRFA field in the UART's C4 register:
     */
    reg_value = read_8bit_mmio_register(&UART_C4_REG(uart_mmio_registers_p));
    SET_BIT_FIELD(
        reg_value, UART_C4_BRFA_MASK, UART_C4_BRFA_SHIFT, brfa_val);
    write_8bit_mmio_register(&UART_C4_REG(uart_mmio_registers_p), reg_value); // TODO: check if this works????
#   endif

    /*
     * Enable UART's transmitter and receiver:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);

    /*
     * Initialize transmit queue:
     */
    rtos_k_byte_circular_buffer_init(
        uart_device_p->urt_transmit_queue_name_p,
        UART_TRANSMIT_QUEUE_SIZE_IN_BYTES,
        uart_device_p->urt_transmit_queue_storage_p,
        NULL,
        &uart_var_p->urt_transmit_queue);

    /*
     * Initialize receive queue:
     */
    rtos_k_byte_circular_buffer_init(
        uart_device_p->urt_receive_queue_name_p,
        UART_RECEIVE_QUEUE_SIZE_IN_BYTES,
        uart_device_p->urt_receive_queue_storage_p,
        NULL,
        &uart_var_p->urt_receive_queue);

    /*
     * Register McRTOS interrupt handler for UART Rx/Tx interrupt:
     */
    rtos_k_register_interrupt(
        &uart_device_p->urt_rtos_interrupt_rx_tx_params,
        uart_device_p->urt_rtos_interrupt_rx_tx_pp);

    DBG_ASSERT(
        *uart_device_p->urt_rtos_interrupt_rx_tx_pp != NULL,
        uart_device_p->urt_rtos_interrupt_rx_tx_pp, uart_device_p);

    /*
     * Register McRTOS interrupt handler for UART error interrupt:
     */

    rtos_k_register_interrupt(
        &uart_device_p->urt_rtos_interrupt_err_params,
        uart_device_p->urt_rtos_interrupt_err_pp);

    DBG_ASSERT(
        *uart_device_p->urt_rtos_interrupt_err_pp != NULL,
        uart_device_p->urt_rtos_interrupt_err_pp, uart_device_p);

    /*
     * NOTE: Generation of Tx/Rx FIFO interrupts is enabled in
     * uart_putchar() and uart_getchar() respectively
     */
    uart_var_p->urt_initialized = true;
}


void
uart_stop(
    const struct uart_device *uart_device_p)
{
    uint32_t reg_value;
    struct uart_device_var *uart_var_p = uart_device_p->urt_var_p;
    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    /*
     * Disable UART's transmitter and receiver:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);

    /*
     * Disable clock for the UART:
     */
    reg_value = read_32bit_mmio_register(&SIM_SCGC4);
    reg_value &= ~uart_device_p->urt_mmio_clock_gate_mask;
    write_32bit_mmio_register(&SIM_SCGC4, reg_value);

    uart_var_p->urt_initialized = false;
}


void
uart_enable_tx_rx_fifos(
    _IN_ const struct uart_device *uart_device_p)
{
    uint32_t reg_value;

    FDC_ASSERT(
        uart_device_p->urt_signature == UART_DEVICE_SIGNATURE,
        uart_device_p->urt_signature, uart_device_p);

    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

#   ifdef _RELIABILITY_CHECKS_
    struct uart_device_var *uart_var_p = uart_device_p->urt_var_p;

    FDC_ASSERT(uart_var_p->urt_initialized, uart_device_p, 0);
    FDC_ASSERT(!uart_var_p->urt_fifos_enabled, uart_device_p, 0);
#   endif

    /*
     * Disable UART's transmitter and receiver:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);

    /*
     * - Enable Tx and Rx FIFOs
     * - Flush Tx and Rx FIFOs
     */
    write_8bit_mmio_register(
        &UART_PFIFO_REG(uart_mmio_registers_p),
	UART_PFIFO_TXFE_MASK | UART_PFIFO_RXFE_MASK);
    write_8bit_mmio_register(
        &UART_CFIFO_REG(uart_mmio_registers_p),
	UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK);

    /*
     * Enable UART's transmitter and receiver:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);
}


void
uart_disable_tx_rx_fifos(
    _IN_ const struct uart_device *uart_device_p)
{
    uint32_t reg_value;

    FDC_ASSERT(
        uart_device_p->urt_signature == UART_DEVICE_SIGNATURE,
        uart_device_p->urt_signature, uart_device_p);

    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

#   ifdef _RELIABILITY_CHECKS_
    struct uart_device_var *uart_var_p = uart_device_p->urt_var_p;

    FDC_ASSERT(uart_var_p->urt_initialized, uart_device_p, 0);
    FDC_ASSERT(uart_var_p->urt_fifos_enabled, uart_device_p, 0);
#   endif

    /*
     * Disable UART's transmitter and receiver:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);

    /*
     * - Disable Tx and Rx FIFOs
     * - Flush Tx and Rx FIFOs
     */
    write_8bit_mmio_register(
        &UART_PFIFO_REG(uart_mmio_registers_p),
	(uint8_t)~(UART_PFIFO_TXFE_MASK | UART_PFIFO_RXFE_MASK));
    write_8bit_mmio_register(
        &UART_CFIFO_REG(uart_mmio_registers_p),
	UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK);

    /*
     * Enable UART's transmitter and receiver:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value |= (UART_C2_TE_MASK | UART_C2_RE_MASK);
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);
}


/**
 * Handling of "Tx FIFO empty" interrupt
 */
static void
k64f_uart_tx_interrupt_handling(UART_MemMapPtr uart_mmio_registers_p,
				struct uart_device_var *restrict uart_var_p)
{
    /*
     * NOTE: This interrupt source will be cleared when
     * the next character is transmitted
     */
    bool sw_transmit_queue_empty = false;

    /*
     * Disable interrupts, to prevent higher priority interrupts to write
     * data to the UART (i.e., printf's)
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    /*
     * Fill the Tx FIFO as much as possible:
     */
#   ifdef DEBUG
    uint_fast8_t tx_fifo_length =
		read_8bit_mmio_register(&UART_TCFIFO_REG(uart_mmio_registers_p));

    DBG_ASSERT(tx_fifo_length == 0, tx_fifo_length, 0);
#   endif

    for (uint_fast8_t i = 0; i < uart_var_p->urt_tx_fifo_size; ++i) {
	uint8_t byte_to_transmit;
	bool entry_read = rtos_k_byte_circular_buffer_read(
			    &uart_var_p->urt_transmit_queue,
			    &byte_to_transmit,
			    false, 0);

	if (!entry_read) {
	    sw_transmit_queue_empty = true;
	    break;
	}

	write_8bit_mmio_register(&UART_D_REG(uart_mmio_registers_p),
				 byte_to_transmit);
    }

    rtos_k_restore_cpu_interrupts(cpu_status_register);

    if (sw_transmit_queue_empty) {
	/*
	 * Disable "transmit data register empty" interrupt:
	 */
	uint32_t reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
	reg_value &= ~UART_C2_TIE_MASK;
	write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);
    }
}


/**
 * Handling of "Rx FIFO not empty" interrupt
 */
static void
k64f_uart_rx_interrupt_handling(UART_MemMapPtr uart_mmio_registers_p,
				struct uart_device_var *restrict uart_var_p)
{
    uint_fast8_t i;
    uint_fast8_t byte_received;
    bool entry_written;
    uint32_t reg_value;
    uint_fast8_t rx_fifo_length =
		read_8bit_mmio_register(&UART_RCFIFO_REG(uart_mmio_registers_p));

    do {
	DBG_ASSERT(
		rx_fifo_length > 0 && rx_fifo_length <= uart_var_p->urt_rx_fifo_size,
		rx_fifo_length, uart_var_p->urt_rx_fifo_size);
	/*
	 * Drain the Rx FIFO but leave one byte in it:
	 */
	for (i = 0; i < rx_fifo_length - 1; i++) {
            byte_received =
		    read_8bit_mmio_register(&UART_D_REG(uart_mmio_registers_p));

	    entry_written = rtos_k_byte_circular_buffer_write(
				&uart_var_p->urt_receive_queue,
				byte_received,
				false);

            if (!entry_written) {
		uart_var_p->urt_received_bytes_dropped ++;
		break;
            }
	}

	/*
	 * Drain the last byte from the FIFO in a special way to clear the
	 * RDRF flag in the S1 register:
	 * - read first the S1 register
	 * - then read the D register
	 */
	if (i == rx_fifo_length - 1) {
	    (void)read_8bit_mmio_register(&UART_S1_REG(uart_mmio_registers_p));

	    byte_received =
		    read_8bit_mmio_register(&UART_D_REG(uart_mmio_registers_p));

	    entry_written = rtos_k_byte_circular_buffer_write(
				&uart_var_p->urt_receive_queue,
				byte_received,
				false);

            if (!entry_written) {
		uart_var_p->urt_received_bytes_dropped ++;
	    }
	}

	rx_fifo_length =
	    read_8bit_mmio_register(&UART_RCFIFO_REG(uart_mmio_registers_p));
    } while (rx_fifo_length != 0);

    /*
     * Disable generation of receive interrupts:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value &= ~UART_C2_RIE_MASK;
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);
}


/**
 * UART Rx/Tx interrupt handler with interrupts enabled
 */
void
k64f_uart_rx_tx_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(rtos_interrupt_p);

    const struct uart_device *uart_device_p =
        (struct uart_device *)rtos_interrupt_p->int_arg_p;

    DBG_ASSERT(
        uart_device_p->urt_signature == UART_DEVICE_SIGNATURE,
        uart_device_p->urt_signature, uart_device_p);

    struct uart_device_var *const restrict uart_var_p = uart_device_p->urt_var_p;
    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    DBG_ASSERT(
        uart_mmio_registers_p == UART0_BASE_PTR ||
        uart_mmio_registers_p == UART4_BASE_PTR,
        uart_mmio_registers_p, uart_device_p);

    DBG_ASSERT(
        uart_var_p->urt_initialized, uart_device_p, 0);

    uint8_t s1_reg_value = read_8bit_mmio_register(&UART_S1_REG(uart_mmio_registers_p));

    /*
     * There is at least one interrupt pending
     * (interrupt status bit is asserted low)
     */
    FDC_ASSERT(
        (s1_reg_value & (UART_S1_RDRF_MASK | UART_S1_TDRE_MASK)) != 0,
        s1_reg_value, uart_device_p);

    /*
     * "Tx FIFO empty" interrupt:
     */
    if (s1_reg_value & UART_S1_TDRE_MASK) {
	k64f_uart_tx_interrupt_handling(uart_mmio_registers_p, uart_var_p);
    }

    /*
     * "Rx FIFO not empty" interrupt:
     */
    if (s1_reg_value & UART_S1_RDRF_MASK) {
	k64f_uart_rx_interrupt_handling(uart_mmio_registers_p, uart_var_p);
    }
}


/**
 * UART error interrupt handler with interrupts enabled
 */
void
k64f_uart_err_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(rtos_interrupt_p);
}


#pragma GCC diagnostic push

#ifndef _RELIABILITY_CHECKS_
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

/**
 * Send a character over a UART serial port, blocking the caller on a condvar
 * if the UART Tx fifo is full.
 */
void
uart_putchar(
    _IN_ const struct uart_device *uart_device_p,
    _IN_ uint8_t c)
{
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;
    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    DBG_ASSERT(uart_var_p->urt_initialized, uart_var_p, uart_device_p);

    DBG_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    if (rtos_k_caller_is_thread()) {
        bool entry_written =
            rtos_k_byte_circular_buffer_write(
                &uart_var_p->urt_transmit_queue, c, true);

        FDC_ASSERT(entry_written, uart_device_p, 0);
    } else {
        bool entry_written =
            rtos_k_byte_circular_buffer_write(
                &uart_var_p->urt_transmit_queue, c, false);

        if (!entry_written) {
            ATOMIC_POST_INCREMENT_UINT32(&uart_var_p->urt_transmit_bytes_dropped);
            return;
        }
    }

    /*
     * Enable generation of "transmit data register empty" interrupts if
     * necessary:
     *
     * NOTE: if the "transmit data register" is empty there will be a
     * pending "transmit data register empty" interrupt, regardless
     * of this interrupt being enabled or not in the UART. Thus, as soon
     * as we enable the generation of this interrupt, the interrupt will
     * fire, if the "transmit data register" was empty.
     */
    uint32_t reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    if ((reg_value & UART_C2_TIE_MASK) == 0) {
        reg_value |= UART_C2_TIE_MASK;
        write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);
    }
}

#pragma GCC diagnostic pop

/**
 * Send a character over a UART serial port, doing polling until the
 * character gets transmitted.
 */
void
uart_putchar_with_polling(
    _IN_ const struct uart_device *uart_device_p,
    _IN_ uint8_t c)
{
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;
    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;
    uint32_t reg_value;

    if (!uart_var_p->urt_initialized) {
        return;
    }

    /*
     * Disable "transmit data register empty" interrupt:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value &= ~UART_C2_TIE_MASK;
            write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);

    /*
     * Do polling until the UART's transmit buffer is empty:
     */
    for ( ; ; ) {
        reg_value = read_8bit_mmio_register(&UART_S1_REG(uart_mmio_registers_p));
        if ((reg_value & UART_S1_TDRE_MASK) != 0) {
            break;
        }
    }

    write_8bit_mmio_register(&UART_D_REG(uart_mmio_registers_p), c);
}

#pragma GCC diagnostic push

#ifndef _RELIABILITY_CHECKS_
#pragma GCC diagnostic ignored "-Wunused-variable"
#endif

/**
 * Receive a character from a UART serial port, blocking the caller
 * if there are no characters to read
 */
uint8_t
uart_getchar(
    _IN_ const struct uart_device *uart_device_p)
{
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;
    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;
    uint32_t reg_value;
    uint8_t char_received;

    DBG_ASSERT(
        uart_var_p->urt_initialized, uart_var_p, uart_device_p);

    DBG_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    /*
     * Enable generation of receive interrupts, if necessary:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    if ((reg_value & UART_C2_RIE_MASK) == 0) {
        reg_value |= UART_C2_RIE_MASK;
        write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);
    }

    bool entry_read = rtos_k_byte_circular_buffer_read(
			&uart_var_p->urt_receive_queue,
			&char_received,
			true, 0);

    FDC_ASSERT(entry_read, uart_device_p, 0);

    return char_received;
}

#pragma GCC diagnostic pop

/**
 * Reads the next character received from a UART serial port, doing polling
 * until the character is received.
 */
uint8_t
uart_getchar_with_polling(
    _IN_ const struct uart_device *uart_device_p)
{
    uint32_t reg_value;
    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

#   ifdef _RELIABILITY_CHECKS_
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;

    FDC_ASSERT(
        uart_var_p->urt_initialized, uart_var_p, uart_device_p);
#   endif

    /*
     * Disable generation of receive interrupts:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value &= ~UART_C2_RIE_MASK;
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);

    /*
     * Do polling until the UART's receive buffer is not empty:
     */
    for ( ; ; ) {
	reg_value = read_8bit_mmio_register(&UART_S1_REG(uart_mmio_registers_p));
        if ((reg_value & UART_S1_RDRF_MASK) != 0) {
            break;
        }
    }

    uint8_t byte_received = read_8bit_mmio_register(&UART_D_REG(uart_mmio_registers_p));

    return byte_received;
}


#if 0 // ???
void
init_adc(const struct adc_device *adc_device_p)
{
    uint32_t reg_value;

    FDC_ASSERT(
        adc_device_p->ad_signature == ADC_DEVICE_SIGNATURE,
        adc_device_p->ad_signature, adc_device_p);

    struct adc_device_var *adc_var_p = adc_device_p->ad_var_p;
    ADC_MemMapPtr adc_mmio_registers_p = adc_device_p->ad_mmio_registers_p;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(!adc_var_p->ad_initialized, adc_device_p, 0);
    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();

    /*
     * Enable the Clock to the ADC Module
     */
    reg_value = read_32bit_mmio_register(&SIM_SCGC6);
    reg_value |= SIM_SCGC6_ADC0_MASK;
    write_32bit_mmio_register(&SIM_SCGC6, reg_value);

    /*
     * NOTE: For the KL25 ADC to operate properly, it must be calibrated
     * first.
     */
    k64f_adc_calibrate(adc_device_p);

    /*
     * Configure ADC_CFG1 register:
     * - ADLPC bit = 0: Normal power configuration
     * - ADLSMP bit = 1: Long sample time
     * - ADIV = 0x2: clock rate is (input clock)/4
     * - MODE = ADC_CFG1_MODE_VALUE
     * - ADICLK = 0x0: Bus clock
     */
    reg_value = ADC_CFG1_ADLSMP_MASK;
    SET_BIT_FIELD(
        reg_value, ADC_CFG1_ADIV_MASK, ADC_CFG1_ADIV_SHIFT, 0x2);
    SET_BIT_FIELD(
        reg_value, ADC_CFG1_MODE_MASK, ADC_CFG1_MODE_SHIFT, ADC_CFG1_MODE_VALUE);
    SET_BIT_FIELD(
        reg_value, ADC_CFG1_ADICLK_MASK, ADC_CFG1_ADICLK_SHIFT, 0x0);
    write_32bit_mmio_register(
        &ADC_CFG1_REG(adc_mmio_registers_p), reg_value);

    /*
     * Configure ADC_CFG2 register:
     * - MUXSEL bit = 0: ADxxa channels are selected (side A of the mux)
     * - ADACKEN bit = 0: Asynchronous clock output disabled
     * - ADHSC bit = 1: High-speed conversion sequence selected with 2
     *   additional ADCK cycles to total conversion time
     * - ADLSTS = 0x0: Default longest sample time; 20 extra ADCK cycles;
     *   24 ADCK cycles total.
     */
    reg_value = ADC_CFG2_ADHSC_MASK;
    SET_BIT_FIELD(
        reg_value, ADC_CFG2_ADLSTS_MASK, ADC_CFG2_ADLSTS_SHIFT, 0x0);
    write_32bit_mmio_register(
        &ADC_CFG2_REG(adc_mmio_registers_p), reg_value);

    /*
     * Configure ADC_SC1A register:
     * - AIEN bit = 1: Conversion complete interrupt is enabled.
     * - DIFF bit = 0: Single-ended conversions and input channels are selected.
     * - ADCH = 0x1F: ADC conversion turned off
     */
    reg_value = ADC_SC1_AIEN_MASK;
    SET_BIT_FIELD(
        reg_value, ADC_SC1_ADCH_MASK, ADC_SC1_ADCH_SHIFT, 0x1F);
    write_32bit_mmio_register(
        &ADC_SC1A_REG(adc_mmio_registers_p), reg_value);

    /*
     * Configure ADC_SC2 register:
     * - ADTRG bit = 0: Software trigger selected.
     * - ACFE bit = 0: Compare function disabled.
     * - ACREN bit = 0: Range function disabled.
     * - DMAEN bit = 0: DMA is disabled.
     * - REFSEL = 0x0: Default voltage reference pin pair, that is, external
     *   pins VREFH and VREFL.
     */
    reg_value = 0x0;
    write_32bit_mmio_register(
        &ADC_SC2_REG(adc_mmio_registers_p), reg_value);

    /*
     * Configure ADC_SC3 register:
     * CAL bit = 0: calibration disabled
     * ADCO bit = 0: 0 One conversion
     * AVGE bit = 0: Hardware average function disabled.
     */
    reg_value = 0x0;
    write_32bit_mmio_register(
        &ADC_SC3_REG(adc_mmio_registers_p), reg_value);

    /*
     * McRTOS-related initialization of the ADC device:
     */

    rtos_k_mutex_init(
        adc_device_p->ad_mutex_name,
        cpu_id,
        &adc_var_p->ad_mutex);

    for (uint8_t i = 0; i < NUM_ADC_CHANNELS; i ++)
    {
        struct adc_channel *adc_channel_p = &adc_var_p->ad_adc_channels[i];

        rtos_k_condvar_init(
            adc_device_p->ad_channel_condvar_name,
            cpu_id,
            &adc_channel_p->adc_condvar);

        adc_channel_p->adc_conversion_completed = false;
    }

    adc_var_p->ad_active_adc_channel = ADC_CHANNEL_NONE;

    /*
     * Register McRTOS interrupt handler
     */
    rtos_k_register_interrupt(
        &adc_device_p->ad_rtos_interrupt_params,
        adc_device_p->ad_rtos_interrupt_pp);

    DBG_ASSERT(
        *adc_device_p->ad_rtos_interrupt_pp != NULL,
        adc_device_p->ad_rtos_interrupt_pp, cpu_id);

    adc_var_p->ad_initialized = true;
}


static void
k64f_adc_calibrate(const struct adc_device *adc_device_p)
{
    uint32_t reg_value;
    ADC_MemMapPtr adc_mmio_registers_p = adc_device_p->ad_mmio_registers_p;

    /*
     * Configure ADC_CFG1 register:
     * - ADLPC bit = 0: Normal power configuration
     * - ADLSMP bit = 1: Long sample time
     * - ADIV = 0x3: clock rate is (input clock)/8 = 3MHz (<= 4MHz)
     * - MODE = ADC_CFG1_MODE_VALUE
     * - ADICLK = 0x0: Bus clock
     */
    reg_value = ADC_CFG1_ADLSMP_MASK;
    SET_BIT_FIELD(
        reg_value, ADC_CFG1_ADIV_MASK, ADC_CFG1_ADIV_SHIFT, 0x3);
    SET_BIT_FIELD(
        reg_value, ADC_CFG1_MODE_MASK, ADC_CFG1_MODE_SHIFT, ADC_CFG1_MODE_VALUE);
    SET_BIT_FIELD(
        reg_value, ADC_CFG1_ADICLK_MASK, ADC_CFG1_ADICLK_SHIFT, 0x0);
    write_32bit_mmio_register(
        &ADC_CFG1_REG(adc_mmio_registers_p), reg_value);

    /*
     * Configure ADC_CFG2 register:
     * - MUXSEL bit = 0: ADxxa channels are selected
     * - ADACKEN bit = 0: Asynchronous clock output disabled
     * - ADHSC bit = 1: High-speed conversion sequence selected with 2
     *   additional ADCK cycles to total conversion time
     * - ADLSTS = 0x0: Default longest sample time; 20 extra ADCK cycles;
     *   24 ADCK cycles total.
     */
    reg_value = ADC_CFG2_ADHSC_MASK;
    SET_BIT_FIELD(
        reg_value, ADC_CFG2_ADLSTS_MASK, ADC_CFG2_ADLSTS_SHIFT, 0x0);
    write_32bit_mmio_register(
        &ADC_CFG2_REG(adc_mmio_registers_p), reg_value);

    /*
     * Configure ADC_SC1A register:
     * - AIEN bit = 0: Conversion complete interrupt is enabled.
     * - DIFF bit = 0: Single-ended conversions and input channels are selected.
     * - ADC_SC1_ADCH_MASK = 0x1F: ADC turned off
     */
    reg_value = 0x0;
    SET_BIT_FIELD(
        reg_value, ADC_SC1_ADCH_MASK, ADC_SC1_ADCH_SHIFT, 0x1F);
    write_32bit_mmio_register(
        &ADC_SC1A_REG(adc_mmio_registers_p), reg_value);

    /*
     * Configure ADC_SC2 register:
     * - ADTRG bit = 0: Software trigger selected.
     * - ACFE bit = 0: Compare function disabled.
     * - ACREN bit = 0: Range function disabled.
     * - DMAEN bit = 0: DMA is disabled.
     * - REFSEL = 0x0: Default voltage reference pin pair, that is, external
     *   pins VREFH and VREFL.
     */
    write_32bit_mmio_register(
        &ADC_SC2_REG(adc_mmio_registers_p), 0x0);

    /*
     * Configure ADC_SC3 register:
     * - CAL bit = 1: calibration enabled
     * - ADCO bit = 0: 0 One conversion
     * - AVGE bit = 1: Hardware average function enabled.
     * - AVGS = 0x3: 32 samples averaged.
     */
    reg_value = (ADC_SC3_CAL_MASK | ADC_SC3_AVGE_MASK);
    SET_BIT_FIELD(
        reg_value, ADC_SC3_AVGS_MASK, ADC_SC3_AVGS_SHIFT, 0x3);
    write_32bit_mmio_register(
        &ADC_SC3_REG(adc_mmio_registers_p), reg_value);

    /*
     * Wait for calibration to be completed:
     */
    do {
        reg_value = read_32bit_mmio_register(
                         &ADC_SC1A_REG(adc_mmio_registers_p));
    } while ((reg_value & ADC_SC1_COCO_MASK) == 0);

    reg_value = read_32bit_mmio_register(
                    &ADC_SC3_REG(adc_mmio_registers_p));

    if (reg_value & ADC_SC3_CALF_MASK) {
        fdc_error_t fdc_error =
            CAPTURE_FDC_ERROR("ADC calibration failed", 0, 0);

        fatal_error_handler(fdc_error);
    }

    /*
     * Calculate plus-side calibration:
     */
    uint16_t calibration_var = 0x0;
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLP0_REG(adc_mmio_registers_p));
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLP1_REG(adc_mmio_registers_p));
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLP2_REG(adc_mmio_registers_p));
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLP3_REG(adc_mmio_registers_p));
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLP4_REG(adc_mmio_registers_p));
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLPS_REG(adc_mmio_registers_p));
    calibration_var /= 2;
    calibration_var |= BIT(15); /* Set MSB */

    write_32bit_mmio_register(
        &ADC_PG_REG(adc_mmio_registers_p), (uint32_t)calibration_var);

    /*
     * Calculate minus-side calibration:
     */
    calibration_var = 0x0;
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLM0_REG(adc_mmio_registers_p));
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLM1_REG(adc_mmio_registers_p));
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLM2_REG(adc_mmio_registers_p));
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLM3_REG(adc_mmio_registers_p));
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLM4_REG(adc_mmio_registers_p));
    calibration_var +=
        read_32bit_mmio_register(&ADC_CLMS_REG(adc_mmio_registers_p));
    calibration_var /= 2;
    calibration_var |= BIT(15); /* Set MSB */

    write_32bit_mmio_register(
        &ADC_MG_REG(adc_mmio_registers_p), (uint32_t)calibration_var);
}


/**
 * ADC interrupt handler with interrupts enabled
 */
void
k64f_adc_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    uint32_t reg_value;

    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(rtos_interrupt_p);

    const struct adc_device *adc_device_p =
        (struct adc_device *)rtos_interrupt_p->int_arg_p;

    DBG_ASSERT(
        adc_device_p->ad_signature == ADC_DEVICE_SIGNATURE,
        adc_device_p->ad_signature, adc_device_p);

    struct adc_device_var *const adc_var_p = adc_device_p->ad_var_p;
    ADC_MemMapPtr adc_mmio_registers_p = adc_device_p->ad_mmio_registers_p;

    reg_value = read_32bit_mmio_register(
                    &ADC_SC1A_REG(adc_mmio_registers_p));

#   ifdef DEBUG
    uint8_t sc1_adch =
        GET_BIT_FIELD(
            reg_value, ADC_SC1_ADCH_MASK, ADC_SC1_ADCH_SHIFT);

    FDC_ASSERT(
        adc_var_p->ad_active_adc_channel == sc1_adch,
        adc_var_p->ad_active_adc_channel, sc1_adch);
#   endif

    struct adc_channel *adc_channel_p =
        &adc_var_p->ad_adc_channels[adc_var_p->ad_active_adc_channel];

    DBG_ASSERT(
        !adc_channel_p->adc_conversion_completed,
        adc_var_p->ad_active_adc_channel, adc_device_p);

    /*
     * Get A/D conversion result:
     * (this also clears the ADC interrupt)
     */
    reg_value = read_32bit_mmio_register(
                    &ADC_RA_REG(adc_mmio_registers_p));

    FDC_ASSERT(
        reg_value <= ADC_RESULT_MAX_VALUE,
        reg_value, ADC_RESULT_MAX_VALUE);

    adc_channel_p->adc_result = reg_value;
    adc_channel_p->adc_conversion_completed = true;

    rtos_k_condvar_signal(&adc_channel_p->adc_condvar);

    adc_var_p->ad_active_adc_channel = ADC_CHANNEL_NONE;
}


/*
 * Starts an A/D conversion for a given ADC channel and then it waits
 * until the A/D conversion is done.
 */
uint32_t
read_adc_channel(
    _IN_ const struct adc_device *adc_device_p,
    _IN_ uint8_t adc_channel)
{
    uint32_t reg_value;
    struct adc_channel *adc_channel_p;

    FDC_ASSERT(
        adc_device_p->ad_signature == ADC_DEVICE_SIGNATURE,
        adc_device_p->ad_signature, adc_device_p);

    FDC_ASSERT(
        adc_channel < NUM_ADC_CHANNELS, adc_channel, NUM_ADC_CHANNELS);

    struct adc_device_var *adc_var_p = adc_device_p->ad_var_p;
    ADC_MemMapPtr adc_mmio_registers_p = adc_device_p->ad_mmio_registers_p;

    FDC_ASSERT(adc_var_p->ad_initialized, adc_device_p, 0);

    adc_channel_p = &adc_var_p->ad_adc_channels[adc_channel];

    FDC_ASSERT(
        !adc_channel_p->adc_conversion_completed,
        adc_channel_p, adc_channel);

    /*
     * Acquire mutex to serialize software-triggered conversions:
     *
     * NOTE: For the KL25 ADC, only one software-triggered
     * conversion can be done at one time, regardless of
     * using different channels.
     */
    rtos_mutex_acquire(&adc_var_p->ad_mutex);

    FDC_ASSERT(
        adc_var_p->ad_active_adc_channel == ADC_CHANNEL_NONE,
        adc_var_p->ad_active_adc_channel, adc_var_p);

    adc_var_p->ad_active_adc_channel = adc_channel;

    /*
     * Set ADC_CFG2 register's MUXEL bit to select side A or side B of
     * the channel:
     */
    reg_value = read_32bit_mmio_register(&ADC_CFG2_REG(adc_mmio_registers_p));
    if (adc_var_p->ad_adc_channels[adc_channel].adc_mux_selector ==
            ADC_MUX_SIDE_A) {
        /*
         * Select channel side A:
         */
        reg_value &= ~ADC_CFG2_MUXSEL_MASK;
    } else {
        /*
         * Select channel side B:
         */
        reg_value |= ADC_CFG2_MUXSEL_MASK;
    }

    write_32bit_mmio_register(
        &ADC_CFG2_REG(adc_mmio_registers_p), reg_value);

    /*
     * Start ADC conversion on the given channel, by setting the channel number
     * on the ADCH field of the ADC_SC1A register:
     */
    reg_value = read_32bit_mmio_register(
                    &ADC_SC1A_REG(adc_mmio_registers_p));
    SET_BIT_FIELD(
        reg_value, ADC_SC1_ADCH_MASK, ADC_SC1_ADCH_SHIFT, adc_channel);
    write_32bit_mmio_register(
        &ADC_SC1A_REG(adc_mmio_registers_p), reg_value);

    /*
     * Wait until the conversion is complete:
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    while (!adc_channel_p->adc_conversion_completed)
    {
        rtos_k_condvar_wait_intr_disabled(&adc_channel_p->adc_condvar, NULL);
    }

    adc_channel_p->adc_conversion_completed = false;

    rtos_k_restore_cpu_interrupts(cpu_status_register);

    rtos_mutex_release(&adc_var_p->ad_mutex);

    return adc_channel_p->adc_result;
}


/**
 * Wait until the current PWM cycle completes
 */
static void
k64f_ftm_wait_pwm_cycle_completion(
    const struct ftm_device *ftm_device_p)
{
    struct ftm_device_var *const ftm_var_p = ftm_device_p->ftm_var_p;
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    while (!ftm_var_p->ftm_pwm_cycle_completed)
    {
        rtos_k_condvar_wait_intr_disabled(&ftm_var_p->ftm_condvar, NULL);
    }

    ftm_var_p->ftm_pwm_cycle_completed = false;

    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


static void
k64f_ftm_set_duty_cycle_internal(
    const struct ftm_device *ftm_device_p,
    pwm_channel_t pwm_channel,
    pwm_duty_cycle_us_t pwm_duty_cycle_us,
    bool wait_previous_cycle_completion)
{
    uint32_t reg_value;
    struct ftm_device_var *const ftm_var_p = ftm_device_p->ftm_var_p;
    FTM_MemMapPtr ftm_mmio_registers_p = ftm_device_p->ftm_mmio_p;

    FDC_ASSERT(
        ftm_device_p->ftm_signature == FTM_DEVICE_SIGNATURE,
        ftm_device_p->ftm_signature, ftm_device_p);
    FDC_ASSERT(
        ftm_var_p->ftm_initialized, ftm_device_p, ftm_var_p);
    FDC_ASSERT(
        pwm_channel < PWM_MAX_NUM_CHANNELS, pwm_channel, ftm_device_p);

    uint32_t pwm_period_us = UINT32_C(1000000) / ftm_device_p->ftm_overflow_freq_hz;

    FDC_ASSERT(
        pwm_duty_cycle_us <= pwm_period_us, pwm_duty_cycle_us, pwm_period_us);

#ifdef DEBUG
    reg_value = read_32bit_mmio_register(&FTM_MOD_REG(ftm_mmio_registers_p));
    FDC_ASSERT(
        reg_value == ftm_var_p->ftm_mod_reg_value,
        reg_value, ftm_var_p->ftm_mod_reg_value);
#endif

    if (wait_previous_cycle_completion) {
        k64f_ftm_wait_pwm_cycle_completion(ftm_device_p);
    }

    /*
     * Set CnV to duty cycle for the channel, as a fraction of ftm_mod_reg_value:
     *
     * new CnV value = ftm_mod_reg_value * (pwm_duty_cycle_us / pwm_period_us)
     *
     * NOTE: The actual CnV register is updated after a write is done on the CnV
     * register and the FTM counter changes from MOD to zero (counter overflow).
     * Thus there is a worst-case latency of pwm_period_us microseconds from
     * the timet this function is called to the time the CnV change takes
     * effect.
     */

    uint32_t tmp = ftm_var_p->ftm_mod_reg_value * pwm_duty_cycle_us;

    FDC_ASSERT(
        tmp >= ftm_var_p->ftm_mod_reg_value || pwm_duty_cycle_us == 0,
        tmp, ftm_var_p->ftm_mod_reg_value);

    reg_value = tmp / pwm_period_us;
    write_32bit_mmio_register(
        &FTM_CnV_REG(ftm_mmio_registers_p, pwm_channel), reg_value);
}


void
k64f_ftm_init(
    const struct ftm_device *ftm_device_p)
{
    uint32_t reg_value;

    FDC_ASSERT(
        ftm_device_p->ftm_signature == FTM_DEVICE_SIGNATURE,
        ftm_device_p->ftm_signature, ftm_device_p);

    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();

    /*
     * Enable the Clock to the FTM Module
     */
    reg_value = read_32bit_mmio_register(&SIM_SCGC6);
    reg_value |= ftm_device_p->ftm_mmio_clock_gate_mask;
    write_32bit_mmio_register(&SIM_SCGC6, reg_value);

    struct ftm_device_var *const ftm_var_p = ftm_device_p->ftm_var_p;
    FTM_MemMapPtr ftm_mmio_registers_p = ftm_device_p->ftm_mmio_p;

    FDC_ASSERT(!ftm_var_p->ftm_initialized, ftm_device_p, ftm_var_p);
    ftm_var_p->ftm_initialized = true;

    /*
     * Blow away the control registers to ensure that the counter is not running
     */
    write_32bit_mmio_register(
        &FTM_SC_REG(ftm_mmio_registers_p), 0x0);
    write_32bit_mmio_register(
        &FTM_CONF_REG(ftm_mmio_registers_p), 0x0);

    /*
     * Setup prescaler before enabling the FTM counter
     */
    reg_value = 0;
    SET_BIT_FIELD(
        reg_value, FTM_SC_PS_MASK, FTM_SC_PS_SHIFT,
        ftm_device_p->ftm_clock_prescale);
    write_32bit_mmio_register(
        &FTM_SC_REG(ftm_mmio_registers_p), reg_value);

    /*
     * Setup the MOD register to get the correct EPWM Period:
     *
     * NOTE: The EPWM period is determined by (MOD + 0x0001), for all channels.
     * The pulse width (duty cycle) for channel n is determined by CnV.
     * MOD must be less than 0xFFFF in order to get a 100% duty cycle EPWM signal.
     */
    reg_value = ((ftm_device_p->ftm_clock_freq_hz /
                    (UINT32_C(1) << ftm_device_p->ftm_clock_prescale)) /
                 ftm_device_p->ftm_overflow_freq_hz) - 1;
    DBG_ASSERT(
        reg_value < 0xffff, reg_value, ftm_device_p);
    write_32bit_mmio_register(
        &FTM_MOD_REG(ftm_mmio_registers_p), reg_value);

    ftm_var_p->ftm_mod_reg_value = reg_value;

    /*
     * Configure PWM channels:
     */
    for (uint32_t i = 0;
         i < ARRAY_SIZE(ftm_device_p->ftm_channels);
         i ++) {
        if (ftm_device_p->ftm_channels[i].ftm_mmio_pcr_p != NULL) {
            /*
             * Set the initial duty cycle for the channel
             */
            k64f_ftm_set_duty_cycle_internal(
                ftm_device_p, i, ftm_device_p->ftm_initial_duty_cycle_us, false);

            /*
             * Enable channel pin:
             */
            write_32bit_mmio_register(
                ftm_device_p->ftm_channels[i].ftm_mmio_pcr_p,
                ftm_device_p->ftm_mmio_pin_mux_selector_mask);

            /*
             * Setup PWM channel:
             * - MSnB:MSnA = 1:0, the edge-aligned PWM mode (EPWM) is selected
             *   for the channel.
             */
            write_32bit_mmio_register(
                &FTM_CnSC_REG(ftm_mmio_registers_p, i),
                ftm_device_p->ftm_channels[i].ftm_mmio_CnSC_value);
        }
    }

    rtos_k_condvar_init(
        ftm_device_p->ftm_condvar_name,
        SOC_GET_CURRENT_CPU_ID(),
        &ftm_var_p->ftm_condvar);

    ftm_var_p->ftm_pwm_cycle_completed = false;

    if (ftm_device_p->ftm_wait_pwm_cycle_completion) {
        /*
         * Register McRTOS interrupt handler
         */
        rtos_k_register_interrupt(
            &ftm_device_p->ftm_rtos_interrupt_params,
            ftm_device_p->ftm_rtos_interrupt_pp);

        DBG_ASSERT(
            *ftm_device_p->ftm_rtos_interrupt_pp != NULL,
            ftm_device_p->ftm_rtos_interrupt_pp, ftm_device_p);
    }

    /*
     * - Enable the FTM Counter:
     *   - CMOD = 01: LPFTM counter increments on every LPFTM counter clock.
     *   - CPWMS = 0: Up counting is selected.
     * - Enable Interrupts for the Timer Overflow if we are going to wait
     *   for PWM cycle completions.
     */

    reg_value = read_32bit_mmio_register(&FTM_SC_REG(ftm_mmio_registers_p));
    SET_BIT_FIELD(
        reg_value, FTM_SC_CMOD_MASK, FTM_SC_CMOD_SHIFT, 0x1);
    if (ftm_device_p->ftm_wait_pwm_cycle_completion) {
        reg_value |= FTM_SC_TOIE_MASK;
    }

    write_32bit_mmio_register(
        &FTM_SC_REG(ftm_mmio_registers_p), reg_value);
}


void
k64f_ftm_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(rtos_interrupt_p);

    const struct ftm_device *ftm_device_p =
        (struct ftm_device *)rtos_interrupt_p->int_arg_p;

    DBG_ASSERT(
        ftm_device_p->ftm_signature == FTM_DEVICE_SIGNATURE,
        ftm_device_p->ftm_signature, ftm_device_p);

    uint32_t reg_value;
    struct ftm_device_var *const ftm_var_p = ftm_device_p->ftm_var_p;
    FTM_MemMapPtr ftm_mmio_registers_p = ftm_device_p->ftm_mmio_p;

    DBG_ASSERT(
        ftm_var_p->ftm_initialized, ftm_device_p, ftm_var_p);

   /*
    * Clear the overflow mask if set, by writing a logic one in it:
    */
    reg_value = read_32bit_mmio_register(&FTM_SC_REG(ftm_mmio_registers_p));
    if (reg_value & FTM_SC_TOF_MASK) {
        write_32bit_mmio_register(
            &FTM_SC_REG(ftm_mmio_registers_p), reg_value);
    }

    ftm_var_p->ftm_pwm_cycle_completed = true;
    rtos_k_condvar_signal(&ftm_var_p->ftm_condvar);
}


void
k64f_ftm_set_duty_cycle(
    const struct ftm_device *ftm_device_p,
    pwm_channel_t pwm_channel,
    pwm_duty_cycle_us_t pwm_duty_cycle_us)
{
    k64f_ftm_set_duty_cycle_internal(
        ftm_device_p,
        pwm_channel,
        pwm_duty_cycle_us,
        ftm_device_p->ftm_wait_pwm_cycle_completion);
}
#endif // ???

/**
 * Wait until the current I2C transaction completes
 */
static void
i2c_wait_transaction_completion(const struct i2c_device *i2c_device_p)
{
    struct i2c_device_var *i2c_var_p = i2c_device_p->i2c_var_p;
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    while (i2c_var_p->transaction.state != I2C_TRANSACTION_COMPLETED &&
           i2c_var_p->transaction.state != I2C_TRANSACTION_ABORTED) {
        rtos_k_condvar_wait_intr_disabled(&i2c_var_p->i2c_condvar, NULL);
    }

    i2c_var_p->transaction.state = I2C_TRANSACTION_NOT_STARTED;
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


static void
i2c_end_transaction(const struct i2c_device *i2c_device_p,
                    enum i2c_transaction_states final_state)
{
    uint32_t reg_value;
    struct i2c_device_var *const i2c_var_p = i2c_device_p->i2c_var_p;
    I2C_MemMapPtr i2c_mmio_registers_p = i2c_device_p->i2c_mmio_registers_p;

    FDC_ASSERT(i2c_var_p->transaction.state != I2C_TRANSACTION_NOT_STARTED,
               i2c_var_p->transaction.state, i2c_device_p);
    reg_value =
        read_8bit_mmio_register(&I2C_S_REG(i2c_mmio_registers_p));
    FDC_ASSERT((reg_value & I2C_S_BUSY_MASK) != 0, reg_value, i2c_device_p);

    /*
     * Generate STOP signal by transitioning the MST bit from 1 to 0:
     */
    reg_value = read_8bit_mmio_register(&I2C_C1_REG(i2c_mmio_registers_p));
    FDC_ASSERT(reg_value & I2C_C1_MST_MASK, reg_value, i2c_device_p);
    reg_value &= ~(I2C_C1_MST_MASK | I2C_C1_TX_MASK | I2C_C1_RSTA_MASK);
    write_8bit_mmio_register(&I2C_C1_REG(i2c_mmio_registers_p), reg_value);
    //???delay_loop(100);
    i2c_var_p->transaction.state = final_state;
    rtos_k_condvar_signal(&i2c_var_p->i2c_condvar);
}


static void
i2c_start_transaction(
    const struct i2c_device *i2c_device_p,
    uint8_t i2c_slave_addr,
    uint8_t i2c_slave_reg_addr,
    uint8_t flags,
    uint8_t *data_buffer_p,
    size_t num_bytes)
{
    uint32_t reg_value;
    struct i2c_device_var *const i2c_var_p = i2c_device_p->i2c_var_p;
    I2C_MemMapPtr i2c_mmio_registers_p = i2c_device_p->i2c_mmio_registers_p;

    FDC_ASSERT(i2c_var_p->transaction.state == I2C_TRANSACTION_NOT_STARTED,
               i2c_var_p->transaction.state, i2c_device_p);

    i2c_var_p->transaction.flags = flags;
    i2c_var_p->transaction.slave_addr = i2c_slave_addr;
    if (flags & I2C_TRANSACTION_HAS_REG_ADDR) {
        i2c_var_p->transaction.slave_reg_addr = i2c_slave_reg_addr;
    }

    i2c_var_p->transaction.next_data_byte_p = data_buffer_p;
    i2c_var_p->transaction.num_data_bytes_left = num_bytes;

    reg_value = read_8bit_mmio_register(&I2C_C1_REG(i2c_mmio_registers_p));

#   ifdef _RELIABILITY_CHECKS_
    uint32_t reg_value2 =
        read_8bit_mmio_register(&I2C_S_REG(i2c_mmio_registers_p));
#   endif

    /*
     * Generate START signal by changing MST bit from 0 to 1:
     */
    FDC_ASSERT(
        (reg_value & (I2C_C1_MST_MASK|I2C_C1_RSTA_MASK|I2C_C1_TX_MASK)) == 0,
        reg_value, i2c_device_p);
    FDC_ASSERT(
        (reg_value2 & I2C_S_BUSY_MASK) == 0 ||
        (reg_value2 & I2C_S_TCF_MASK) != 0, reg_value2, i2c_device_p);

    reg_value |= (I2C_C1_MST_MASK | I2C_C1_TX_MASK);
    write_8bit_mmio_register(&I2C_C1_REG(i2c_mmio_registers_p), reg_value);

    uint8_t data_value = 0;
    SET_BIT_FIELD(
        data_value, I2C_SLAVE_ADDR_MASK, I2C_SLAVE_ADDR_SHIFT, i2c_slave_addr);

    i2c_var_p->transaction.state = I2C_SENDING_SLAVE_ADDR;

    /*
     * Initiate transmit of slave address on the I2C bus:
     */
    write_8bit_mmio_register(&I2C_D_REG(i2c_mmio_registers_p), data_value);
}


void
i2c_init(
    const struct i2c_device *i2c_device_p)
{
    uint32_t reg_value;

    FDC_ASSERT(
        i2c_device_p->i2c_signature == I2C_DEVICE_SIGNATURE,
        i2c_device_p->i2c_signature, i2c_device_p);

    struct i2c_device_var *const i2c_var_p = i2c_device_p->i2c_var_p;
    I2C_MemMapPtr i2c_mmio_registers_p = i2c_device_p->i2c_mmio_registers_p;

    FDC_ASSERT(!i2c_var_p->i2c_initialized, i2c_device_p, i2c_var_p);
    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();

    rtos_k_condvar_init(
        i2c_device_p->i2c_condvar_name,
        &i2c_var_p->i2c_condvar);

    /*
     * Enable the Clock to the I2C Module
     */
    reg_value = read_32bit_mmio_register(&SIM_SCGC4);
    reg_value |= i2c_device_p->i2c_clock_gate_mask;
    write_32bit_mmio_register(&SIM_SCGC4, reg_value);

    /*
     * Configure GPIO pins for I2C functions:
     * - Set "Open Drain Enabled" for both the SCL and SDA pins
     */
    set_pin_function(&i2c_device_p->i2c_scl_pin, PORT_PCR_ODE_MASK);
    set_pin_function(&i2c_device_p->i2c_sda_pin, PORT_PCR_ODE_MASK);

    /*
     * Set baud rate:
     */
    reg_value = 0;
    SET_BIT_FIELD(
        reg_value, I2C_F_ICR_MASK, I2C_F_ICR_SHIFT, i2c_device_p->i2c_icr_value);
    write_8bit_mmio_register(
        &I2C_F_REG(i2c_mmio_registers_p), reg_value);

    /*
     * Clear any pending interrupt:
     */
    write_8bit_mmio_register(
	&I2C_S_REG(i2c_mmio_registers_p), I2C_S_IICIF_MASK | I2C_S_ARBL_MASK);

    /*
     * Enable IIC mode and enable interrupt generation:
     */
    write_8bit_mmio_register(
        &I2C_C1_REG(i2c_mmio_registers_p),
        I2C_C1_IICEN_MASK | I2C_C1_IICIE_MASK);

    /*
     * Register McRTOS interrupt handler
     */
    rtos_k_register_interrupt(
        &i2c_device_p->i2c_rtos_interrupt_params,
        i2c_device_p->i2c_rtos_interrupt_pp);

    DBG_ASSERT(
        *i2c_device_p->i2c_rtos_interrupt_pp != NULL,
        i2c_device_p->i2c_rtos_interrupt_pp, i2c_device_p);

    i2c_var_p->i2c_initialized = true;
    DEBUG_PRINTF("Initialized device %s\n", i2c_device_p->i2c_name);
}

void
i2c_shutdown(
    const struct i2c_device *i2c_device_p)
{
    uint32_t reg_value;

    FDC_ASSERT(
        i2c_device_p->i2c_signature == I2C_DEVICE_SIGNATURE,
        i2c_device_p->i2c_signature, i2c_device_p);

    struct i2c_device_var *const i2c_var_p = i2c_device_p->i2c_var_p;
    I2C_MemMapPtr i2c_mmio_registers_p = i2c_device_p->i2c_mmio_registers_p;

    i2c_var_p->i2c_initialized = false;

    reg_value =
        read_8bit_mmio_register(&I2C_S_REG(i2c_mmio_registers_p));
    if ((reg_value & I2C_S_BUSY_MASK) != 0) {
        i2c_end_transaction(i2c_device_p, I2C_TRANSACTION_ABORTED);
    }

    /*
     * Disable IIC mode and interrupts:
     */
    write_8bit_mmio_register(
        &I2C_C1_REG(i2c_mmio_registers_p), 0x0);

    /*
     * Disable the Clock to the I2C Module
     */
    reg_value = read_32bit_mmio_register(&SIM_SCGC4);
    reg_value &= ~i2c_device_p->i2c_clock_gate_mask;
    write_32bit_mmio_register(&SIM_SCGC4, reg_value);

    DEBUG_PRINTF("Stopped device %s\n", i2c_device_p->i2c_name);
}

void i2c_read(
    const struct i2c_device *i2c_device_p,
    uint8_t i2c_slave_addr, uint8_t i2c_slave_reg_addr,
    uint8_t *buffer_p, size_t num_bytes)
{
    DBG_ASSERT(
        i2c_device_p->i2c_signature == I2C_DEVICE_SIGNATURE,
        i2c_device_p->i2c_signature, i2c_device_p);

#   ifdef DEBUG
    struct i2c_device_var *const i2c_var_p = i2c_device_p->i2c_var_p;

    DBG_ASSERT(i2c_var_p->i2c_initialized, i2c_device_p, i2c_var_p);
#   endif

    DBG_ASSERT_VALID_RAM_POINTER(buffer_p, 1);
    DBG_ASSERT(num_bytes != 0, 0, 0);

    i2c_start_transaction(i2c_device_p,
                          i2c_slave_addr,
                          i2c_slave_reg_addr,
                          I2C_TRANSACTION_HAS_REG_ADDR |
                          I2C_TRANSACTION_IS_READ_DATA,
                          buffer_p,
                          num_bytes);

    i2c_wait_transaction_completion(i2c_device_p);
}


void i2c_write(
    const struct i2c_device *i2c_device_p,
    uint8_t i2c_slave_addr, uint8_t i2c_slave_reg_addr,
    uint8_t *buffer_p, size_t num_bytes)
{
    i2c_start_transaction(i2c_device_p,
                          i2c_slave_addr,
                          i2c_slave_reg_addr,
                          I2C_TRANSACTION_HAS_REG_ADDR,
                          buffer_p,
                          num_bytes);

    i2c_wait_transaction_completion(i2c_device_p);
}


static void
i2c_switch_to_rx_mode(const struct i2c_device *i2c_device_p)
{
    uint32_t reg_value;
    struct i2c_device_var *const i2c_var_p = i2c_device_p->i2c_var_p;
    I2C_MemMapPtr i2c_mmio_registers_p = i2c_device_p->i2c_mmio_registers_p;

#   ifdef DEBUG
    if (i2c_var_p->transaction.flags & I2C_TRANSACTION_HAS_REG_ADDR) {
        DBG_ASSERT(i2c_var_p->transaction.state == I2C_SENDING_SLAVE_REG_ADDR,
                   i2c_var_p->transaction.state, i2c_device_p);
    } else {
        DBG_ASSERT(i2c_var_p->transaction.state == I2C_SENDING_SLAVE_ADDR,
                   i2c_var_p->transaction.state, i2c_device_p);
    }

    DBG_ASSERT(i2c_var_p->transaction.flags & I2C_TRANSACTION_IS_READ_DATA,
               i2c_var_p->transaction.flags, i2c_device_p);
#   endif

    /*
     * Generate a repeated START signal and also set TX mode to send
     * slave addr and transfer direction flag:
     */
    reg_value = read_8bit_mmio_register(&I2C_C1_REG(i2c_mmio_registers_p));
    FDC_ASSERT(
        (reg_value & I2C_C1_MST_MASK) != 0, reg_value, i2c_device_p);

#   ifdef _RELIABILITY_CHECKS_
    uint32_t reg_value2 = read_8bit_mmio_register(&I2C_S_REG(i2c_mmio_registers_p));

    FDC_ASSERT(
        (reg_value2 & I2C_S_BUSY_MASK) != 0, reg_value2, i2c_device_p);
#   endif

    reg_value |= (I2C_C1_RSTA_MASK | I2C_C1_TX_MASK);
    write_8bit_mmio_register(&I2C_C1_REG(i2c_mmio_registers_p), reg_value);

    /*
     * Send slave addr and direction flag set, to effectively switch
     * data transfer direction to Rx:
     */
    uint8_t data_value = 0;
    SET_BIT_FIELD(
        data_value, I2C_SLAVE_ADDR_MASK, I2C_SLAVE_ADDR_SHIFT,
        i2c_var_p->transaction.slave_addr);

    data_value |= I2C_READ_TRANSACTION_MASK;

    i2c_var_p->transaction.state = I2C_SENDING_SLAVE_ADDR_FOR_RX;

    /*
     * Initiate transmit of slave address and transfer direction flag on the
     * I2C bus:
     */
    write_8bit_mmio_register(&I2C_D_REG(i2c_mmio_registers_p), data_value);
}


void
k64f_i2c_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(rtos_interrupt_p);

    const struct i2c_device *i2c_device_p =
        (struct i2c_device *)rtos_interrupt_p->int_arg_p;

    DBG_ASSERT(
        i2c_device_p->i2c_signature == I2C_DEVICE_SIGNATURE,
        i2c_device_p->i2c_signature, i2c_device_p);

    struct i2c_device_var *const i2c_var_p = i2c_device_p->i2c_var_p;
    I2C_MemMapPtr i2c_mmio_registers_p = i2c_device_p->i2c_mmio_registers_p;

    uint32_t reg_value =
        read_8bit_mmio_register(&I2C_S_REG(i2c_mmio_registers_p));

    FDC_ASSERT(
        (reg_value & I2C_S_IICIF_MASK) != 0, reg_value, i2c_device_p);
    FDC_ASSERT(
        (reg_value & I2C_S_TCF_MASK) != 0, reg_value, i2c_device_p);

    /*
     * Clear the interrupt flag, by writing 1 to it:
     */
    if (reg_value & I2C_S_ARBL_MASK) {
	write_8bit_mmio_register(
	    &I2C_S_REG(i2c_mmio_registers_p), I2C_S_IICIF_MASK | I2C_S_ARBL_MASK);
    } else {
	write_8bit_mmio_register(
	    &I2C_S_REG(i2c_mmio_registers_p), I2C_S_IICIF_MASK);
    }

    reg_value =
        read_8bit_mmio_register(&I2C_S_REG(i2c_mmio_registers_p));

    DBG_ASSERT(
        (reg_value & I2C_S_IICIF_MASK) == 0, reg_value, i2c_device_p);
    DBG_ASSERT(
        (reg_value & I2C_S_ARBL_MASK) == 0, reg_value, i2c_device_p);

    if (i2c_var_p->transaction.state == I2C_SENDING_SLAVE_ADDR ||
        i2c_var_p->transaction.state == I2C_SENDING_SLAVE_REG_ADDR ||
        i2c_var_p->transaction.state == I2C_SENDING_DATA_BYTE) {
        if (reg_value & I2C_S_RXAK_MASK) {
            /*
             * RXAK == 1 means no ACK signal from the slave detected
             */
            CAPTURE_FDC_ERROR("ACK signal not received from i2C slave",
                              reg_value, i2c_device_p);

            i2c_end_transaction(i2c_device_p, I2C_TRANSACTION_ABORTED);
            return;
        }
    }

    switch (i2c_var_p->transaction.state) {
    case I2C_SENDING_SLAVE_ADDR:
        if (i2c_var_p->transaction.flags & I2C_TRANSACTION_HAS_REG_ADDR) {
            write_8bit_mmio_register(&I2C_D_REG(i2c_mmio_registers_p),
                                     i2c_var_p->transaction.slave_reg_addr);
            i2c_var_p->transaction.state = I2C_SENDING_SLAVE_REG_ADDR;
            break;
        }
        /* fall through */

    case I2C_SENDING_SLAVE_REG_ADDR:
        if (i2c_var_p->transaction.flags & I2C_TRANSACTION_IS_READ_DATA) {
            i2c_switch_to_rx_mode(i2c_device_p);
        } else {
            /*
             * Initiate send of first data byte to the slave:
             */
            FDC_ASSERT(i2c_var_p->transaction.num_data_bytes_left > 0,
                       i2c_device_p, 0);

            i2c_var_p->transaction.state = I2C_SENDING_DATA_BYTE;
            write_8bit_mmio_register(&I2C_D_REG(i2c_mmio_registers_p),
                                     *i2c_var_p->transaction.next_data_byte_p);
        }

        break;

    case I2C_SENDING_SLAVE_ADDR_FOR_RX:
        /*
         * Initiate receive of first data byte from the slave:
         */
        FDC_ASSERT(i2c_var_p->transaction.num_data_bytes_left > 0,
                   i2c_device_p, 0);

        reg_value = read_8bit_mmio_register(&I2C_C1_REG(i2c_mmio_registers_p));
        reg_value &= ~I2C_C1_TX_MASK;
        if (i2c_var_p->transaction.num_data_bytes_left == 1) {
            /*
             * Don't send ACK signal for next byte received from the slave
             */
            reg_value |= I2C_C1_TXAK_MASK;
        } else {
            /*
             * Send ACK signal for next byte received from the slave
             * (TXAK bit is active low)
             */
            reg_value &= ~I2C_C1_TXAK_MASK;
        }

        write_8bit_mmio_register(&I2C_C1_REG(i2c_mmio_registers_p), reg_value);

        /*
         * Do a dummy read of the D register to initiate receive of first data
         * byte from slave:
         */
        (void)read_8bit_mmio_register(&I2C_D_REG(i2c_mmio_registers_p));

        i2c_var_p->transaction.state = I2C_RECEIVING_DATA_BYTE;
        break;

    case I2C_SENDING_DATA_BYTE:
        FDC_ASSERT(i2c_var_p->transaction.num_data_bytes_left > 0,
                   i2c_device_p, 0);

        i2c_var_p->transaction.next_data_byte_p ++;
        i2c_var_p->transaction.num_data_bytes_left --;
        if (i2c_var_p->transaction.num_data_bytes_left == 0) {
            i2c_end_transaction(i2c_device_p, I2C_TRANSACTION_COMPLETED);
            return;
        }

        /*
         * Initiate send of next data byte:
         */
        write_8bit_mmio_register(&I2C_D_REG(i2c_mmio_registers_p),
                                 *i2c_var_p->transaction.next_data_byte_p);
        break;

    case I2C_RECEIVING_DATA_BYTE:
        FDC_ASSERT(i2c_var_p->transaction.num_data_bytes_left > 0,
                   i2c_device_p, 0);

        i2c_var_p->transaction.num_data_bytes_left --;
        if (i2c_var_p->transaction.num_data_bytes_left == 0) {
            /*
             * This is the last byte:
             */
            i2c_end_transaction(i2c_device_p, I2C_TRANSACTION_COMPLETED);
        } else {
            reg_value = read_8bit_mmio_register(&I2C_C1_REG(i2c_mmio_registers_p));
            if (i2c_var_p->transaction.num_data_bytes_left == 1) {
                /*
                 * Don't send ACK signal for next byte received from the slave
                 * (TXAK bit is active low)
                 */
                reg_value |= I2C_C1_TXAK_MASK;
            } else {
                /*
                 * Send ACK signal for next byte received from the slave
                 */
                reg_value &= ~I2C_C1_TXAK_MASK;
            }

            write_8bit_mmio_register(&I2C_C1_REG(i2c_mmio_registers_p), reg_value);
        }

        /*
         * Read D register to retrieve last data byte received and to start
         * receiving the next data byte from the slave
         *
         * NOTE: If i2c_end_transaction() is not called above, this will also
         * initiate another read transfer.
         */
        *i2c_var_p->transaction.next_data_byte_p =
            read_8bit_mmio_register(&I2C_D_REG(i2c_mmio_registers_p));

        i2c_var_p->transaction.next_data_byte_p ++;
        break;

    default:
        FDC_ASSERT(false, i2c_var_p->transaction.state, i2c_device_p);
    }
}


/**
 * GPIO Port C interrupt handler with interrupts enabled
 */
void
k64f_port_c_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(rtos_interrupt_p);
}


void
assert_interrupt_source_is_set(interrupt_channel_t interrupt_channel)
{
    FDC_ASSERT(
        interrupt_channel < SOC_NUM_INTERRUPT_CHANNELS,
        interrupt_channel, SOC_NUM_INTERRUPT_CHANNELS);

    FDC_ASSERT(
        NVIC_GetPendingIRQ(interrupt_channel) != 0,
        interrupt_channel, 0);
}


/**
 * Notify interrupt controller that processing for the last interrupt received
 * by the calling CPU core has been completed, so that another interrupt of
 * the same priority or lower can be received by this CPU core
 */
void
notify_interrupt_controller_isr_done(interrupt_channel_t interrupt_channel)
{
    DBG_ASSERT(
        interrupt_channel >= -1 && interrupt_channel < SOC_NUM_INTERRUPT_CHANNELS,
        interrupt_channel, SOC_NUM_INTERRUPT_CHANNELS);
}


/*
 * Generate dummy NVIC interrupt handler functions
 */
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma2_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA2))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma3_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA3))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma4_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA4))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma5_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA5))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma6_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA6))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma7_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA7))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma8_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA8))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma9_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA9))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma10_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA10))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma11_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA11))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma12_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA12))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma13_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA13))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma14_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA14))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma15_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA15))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma_error_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA_Error))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_mcm_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_MCM))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ftfe_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_FTFE))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_read_collision_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Read_Collision))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_lvd_lvw_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_LVD_LVW))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_llw_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_LLW))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_watchdog_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Watchdog))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_i2c1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2C1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_i2s0_tx_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2S0_Tx))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_i2s0_rx_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2S0_Rx))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart0_lon_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART0_LON))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart1_rx_tx_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART1_RX_TX))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart1_err_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART1_ERR))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart2_rx_tx_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART2_RX_TX))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart2_err_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART2_ERR))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart3_rx_tx_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART3_RX_TX))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart3_err_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART3_ERR))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_cmp0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CMP0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_cmp1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CMP1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ftm0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_FTM0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ftm1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_FTM1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ftm2_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_FTM2))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_cmt_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CMT))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_rtc_alarm_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_RTC))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_rtc_seconds_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_RTC_Seconds))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pit0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIT0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pit1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIT1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pit2_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIT2))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pdb0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PDB0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_usb_otg_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_USB0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_usb_dcd_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_USBDCD))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_reserved71_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved71))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dac0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DAC0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_mcg_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_MCG))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_lptimer_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_LPTimer))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_port_a_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PORTA))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_port_b_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PORTB))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_port_d_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PORTD))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_port_e_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PORTE))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_swi_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SWI))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi2_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI2))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart5_rx_tx_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART5_RX_TX))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart5_err_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART5_ERR))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_cmp2_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CMP2))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ftm3_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_FTM3))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dac1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DAC1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_adc0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ADC0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_adc1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ADC1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_i2c2_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2C2))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_can0_ored_message_buffer_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CAN0_ORed_Message_buffer))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_can0_bus_off_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CAN0_Bus_Off))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_can0_error_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CAN0_Error))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_can0_tx_warning_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CAN0_Tx_Warning))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_can0_rx_warning_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CAN0_Rx_Warning))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_can0_wake_up_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CAN0_Wake_Up))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_sdhc_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SDHC))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_enet_1588_timer_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ENET_1588_Timer))

/**
 * Generate function that reads from an MMIO register of the given bit width
 */
#define GEN_READ_MMIO_REGISTER_FUNCTION(_func_name, _reg_type, _value_type) \
    _value_type                                                         \
    _func_name(const volatile _reg_type *io_reg_p)                      \
    {                                                                   \
        FDC_ASSERT_VALID_MMIO_ADDRESS(io_reg_p);                        \
        return *io_reg_p;                                               \
    }

/**
 * Generate function that writes to an MMIO register of the given bit width
 */
#define GEN_WRITE_MMIO_REGISTER_FUNCTION(_func_name, _reg_type, _value_type) \
    void                                                                \
    _func_name(volatile _reg_type *io_reg_p, _value_type value)         \
    {                                                                   \
        FDC_ASSERT_VALID_MMIO_ADDRESS(io_reg_p);                        \
        *io_reg_p = value;                                              \
    }

GEN_READ_MMIO_REGISTER_FUNCTION(read_32bit_mmio_register, uint32_t, uint32_t)

GEN_WRITE_MMIO_REGISTER_FUNCTION(write_32bit_mmio_register, uint32_t, uint32_t)

GEN_READ_MMIO_REGISTER_FUNCTION(read_16bit_mmio_register, uint16_t, uint16_t)

GEN_WRITE_MMIO_REGISTER_FUNCTION(write_16bit_mmio_register, uint16_t, uint16_t)

GEN_READ_MMIO_REGISTER_FUNCTION(read_8bit_mmio_register, uint8_t, uint8_t)

GEN_WRITE_MMIO_REGISTER_FUNCTION(write_8bit_mmio_register, uint8_t, uint8_t)

