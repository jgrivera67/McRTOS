/**
 * @file kl25z_hardware_abstractions.c
 *
 * Hardware abstraction layer for the KL25Z SoC
 *
 * @author German Rivera 
 */ 

#include "hardware_abstractions.h"
#include "McRTOS_arm_cortex_m.h"
#include "MKL25Z4.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"

//TODO("Remove these pragmas")
//#pragma GCC diagnostic ignored "-Wunused-variable"

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

/**
 * Const fields of a UART device (to be placed in flash)
 */
struct uart_device {
#   define UART_DEVICE_SIGNATURE  GEN_SIGNATURE('U', 'A', 'R', 'T')
    uint32_t urt_signature;
    struct uart_device_var *urt_var_p;
    union {
        UART0_MemMapPtr urt_mmio_uart0_p;
        UART_MemMapPtr urt_mmio_uart_p;
    };

    volatile uint32_t *urt_mmio_tx_port_pcr_p;
    volatile uint32_t *urt_mmio_rx_port_pcr_p;
    uint32_t urt_mmio_pin_mux_selector_mask;
    uint32_t urt_mmio_clock_gate_mask;

    struct rtos_interrupt_registration_params urt_rtos_interrupt_params;
    struct rtos_interrupt **urt_rtos_interrupt_pp;
    const char *urt_transmit_queue_name_p;
    const char *urt_receive_queue_name_p;
    uint8_t *urt_transmit_queue_storage_p;
    uint8_t *urt_receive_queue_storage_p;
};

C_ASSERT(
    offsetof(struct UART0_MemMap, BDH) == offsetof(struct UART_MemMap, BDH));

C_ASSERT(
    offsetof(struct UART0_MemMap, BDL) == offsetof(struct UART_MemMap, BDL));

C_ASSERT(
    offsetof(struct UART0_MemMap, C1) == offsetof(struct UART_MemMap, C1));

C_ASSERT(
    offsetof(struct UART0_MemMap, C2) == offsetof(struct UART_MemMap, C2));

C_ASSERT(
    offsetof(struct UART0_MemMap, S1) == offsetof(struct UART_MemMap, S1));

C_ASSERT(
    offsetof(struct UART0_MemMap, S2) == offsetof(struct UART_MemMap, S2));

C_ASSERT(
    offsetof(struct UART0_MemMap, C3) == offsetof(struct UART_MemMap, C3));

C_ASSERT(
    offsetof(struct UART0_MemMap, D) == offsetof(struct UART_MemMap, D));

/**
 * Non-const fields of a UART device (to be placed in SRAM)
 */
struct uart_device_var {
    bool urt_initialized;
    uint32_t urt_received_bytes_dropped;
    uint32_t urt_transmit_bytes_dropped;
    struct rtos_circular_buffer urt_transmit_queue;
    struct rtos_circular_buffer urt_receive_queue;
};

#if 0 // ???
/**
 * Const fields of the A/D converter device (to be placed in flash)
 */
struct adc_device {
#   define ADC_DEVICE_SIGNATURE  GEN_SIGNATURE('A', '/', 'D', 'C')
    uint32_t ad_signature;
    lpc2478_adc_t *ad_mmio_registers_p;
    struct rtos_interrupt_registration_params ad_rtos_interrupt_params;
    struct rtos_interrupt **ad_rtos_interrupt_pp;
    const char *ad_channel_condvar_names[NUM_ADC_CHANNELS];
    struct adc_channel *ad_adc_channels_p;
};

/**
 * Non-const fields of an A/D converter channel (to be placed in SRAM)
 */
struct adc_channel {
    /**
     * Condvar to signal a thread waiting for an A/D conversion
     */
    struct rtos_condvar adc_condvar;
    
    /**
     * Last value read from the V/VREF field of the corresponding ADC channel's
     * reg_AD0DR[] register, by the A/D conversion completion interrupt handler.
     */
    volatile uint16_t adc_result;

    /**
     * Boolean flag that indicates if an outstanding A/D conversion has completed
     */
    volatile uint8_t adc_conversion_completed;

    uint8_t  adc_reserved;
};
#endif // ???

static cpu_reset_cause_t find_reset_cause(void);

static void system_clocks_init(void);

static void pll_init(void);

#if 0 // ???
static void tfc_gpio_init(void);
#endif

static void rgb_led_init(void);

static void uart_stop(
    const struct uart_device *uart_device_p);

static void
init_adc(const struct adc_device *adc_device_p);

/* 
 * Function prototypes for dummy VIC interrupt ISRs
 */
static isr_function_t dummy_dma0_isr;
static isr_function_t dummy_dma1_isr;
static isr_function_t dummy_dma2_isr;
static isr_function_t dummy_dma3_isr;
static isr_function_t dummy_mcm_isr;
static isr_function_t dummy_ftfl_isr;
static isr_function_t dummy_pmc_isr;
static isr_function_t dummy_llw_isr;
static isr_function_t dummy_i2c0_isr;
static isr_function_t dummy_i2c1_isr;
static isr_function_t dummy_spi0_isr;
static isr_function_t dummy_spi1_isr;
static isr_function_t dummy_uart1_isr;
static isr_function_t dummy_uart2_isr;
static isr_function_t dummy_cmp0_isr;
static isr_function_t dummy_ftm0_isr;
static isr_function_t dummy_ftm1_isr;
static isr_function_t dummy_ftm2_isr;
static isr_function_t dummy_rtc_alarm_isr;
static isr_function_t dummy_rtc_seconds_isr;
static isr_function_t dummy_pit_isr;
static isr_function_t dummy_reserved_irq23_isr;
static isr_function_t dummy_usb_otg_isr;
static isr_function_t dummy_dac0_isr;
static isr_function_t dummy_tsi0_isr;
static isr_function_t dummy_mcg_isr;
static isr_function_t dummy_lptimer_isr;
static isr_function_t dummy_reserved_irq29_isr;
static isr_function_t dummy_port_a_isr;
static isr_function_t dummy_port_d_isr;

extern isr_function_t cortex_m_systick_isr;
extern isr_function_t kl25_uart0_isr;
extern isr_function_t kl25_adc0_isr;

static uint32_t g_pll_frequency_in_hz = 0;

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
    [INT_SVCall] = cortex_m_svc_handler,
    [INT_PendableSrvReq] = cortex_m_pendsv_exception_handler,
    [INT_SysTick] = cortex_m_systick_isr,

    /* Interrupts external to the Cortex-M core*/
    [INT_DMA0] = dummy_dma0_isr, /* DMA Channel 0 Transfer Complete and Error */
    [INT_DMA1] = dummy_dma1_isr, /* DMA Channel 1 Transfer Complete and Error */
    [INT_DMA2] = dummy_dma2_isr, /* DMA Channel 2 Transfer Complete and Error */
    [INT_DMA3] = dummy_dma3_isr, /* DMA Channel 3 Transfer Complete and Error */
    [INT_Reserved20] = dummy_mcm_isr, /* Normal Interrupt */
    [INT_FTFA] = dummy_ftfl_isr, /* FTFL Interrupt */
    [INT_LVD_LVW] = dummy_pmc_isr, /* PMC Interrupt */
    [INT_LLW] = dummy_llw_isr, /* Low Leakage Wake-up */
    [INT_I2C0] = dummy_i2c0_isr, /* I2C0 interrupt */
    [INT_I2C1] = dummy_i2c1_isr, /* I2C1 interrupt */
    [INT_SPI0] = dummy_spi0_isr, /* SPI0 Interrupt */
    [INT_SPI1] = dummy_spi1_isr, /* SPI1 Interrupt */
    [INT_UART0] = kl25_uart0_isr,    /* UART0 Status and Error interrupt */
    [INT_UART1] = dummy_uart1_isr, /* UART1 Status and Error interrupt */
    [INT_UART2] = dummy_uart2_isr, /* UART2 Status and Error interrupt */
    [INT_ADC0] = kl25_adc0_isr, /* ADC0 interrupt */
    [INT_CMP0] = dummy_cmp0_isr, /* CMP0 interrupt */
    [INT_TPM0] = dummy_ftm0_isr, /* FTM0 fault, overflow and channels interrupt */
    [INT_TPM1] = dummy_ftm1_isr, /* FTM1 fault, overflow and channels interrupt */
    [INT_TPM2] = dummy_ftm2_isr, /* FTM2 fault, overflow and channels interrupt */
    [INT_RTC] = dummy_rtc_alarm_isr, /* RTC Alarm interrupt */
    [INT_RTC_Seconds] = dummy_rtc_seconds_isr, /* RTC Seconds interrupt */
    [INT_PIT] = dummy_pit_isr, /* PIT timer all channels interrupt */
    [INT_Reserved39] = dummy_reserved_irq23_isr,
    [INT_USB0] = dummy_usb_otg_isr, /* USB interrupt */
    [INT_DAC0] = dummy_dac0_isr, /* DAC0 interrupt */
    [INT_TSI0] = dummy_tsi0_isr, /* TSI0 Interrupt */
    [INT_MCG] = dummy_mcg_isr, /* MCG Interrupt */
    [INT_LPTimer] = dummy_lptimer_isr, /* LPTimer interrupt */
    [INT_Reserved45] = dummy_reserved_irq29_isr,
    [INT_PORTA] = dummy_port_a_isr, /* Port A interrupt */
    [INT_PORTD] = dummy_port_d_isr /* Port D interrupt */
};

C_ASSERT(
    ARRAY_SIZE(g_interrupt_vector_table) ==
    CORTEX_M_IRQ_VECTOR_BASE + SOC_NUM_INTERRUPT_CHANNELS);

/**
 * SoC configuration in Flash
 */
static const struct NV_MemMap nv_cfmconfig __attribute__ ((section(".cfmconfig"))) = {
#if 0
  .BACKKEY3 =
  .BACKKEY2
  .BACKKEY1
  .BACKKEY0
  .BACKKEY7
  .BACKKEY6
  .BACKKEY5
  .BACKKEY4
  .FPROT3
  .FPROT2
  .FPROT1
  .FPROT0
  .FSEC
#endif
  .FOPT = NV_FOPT_RESET_PIN_CFG_MASK
};

#if 0 // ???
/**
 * Global definition of const pointer to the array of fast_gpio_port structs,
 * one entry per fast GPIO port of the LPC2478.
 */ 
static volatile struct fast_gpio_port *const g_fast_gpio_ports_array =
    (struct fast_gpio_port *)FIO_BASE_ADDR;

/**
 * Global definition of const pointer to the memory-mapped registers of the
 * LPC2478 Pin Connect Block
 */
pin_connect_block_t *const g_pin_connect_block = (pin_connect_block_t *)PINCON_BASE_ADDR;

/**
 * Global definition of const pointer to the memory-mapped registers of the
 * LPC2478 SCB (System Control Block)
 */
lpc2478_scb_t *const g_scb_mmio_registers_p = (lpc2478_scb_t *)LPC2478_SCB_BASE_ADDR;

/**
 * Global array of const structures for timer devices for the LPC2478 
 * (allocated in flash space)
 */
static const struct timer_device g_timer_devices[] =
{
    [0] = {
        .tmr_signature = TIMER_DEVICE_SIGNATURE,
        .tmr_mmio_registers_p = (lpc2478_timer_t *)LPC2478_TIMER0_BASE_ADDR,
        .tmr_timer_pclk_shift = PCLK_TIMER0,
        .tmr_pclksel_index = 0,
        .tmr_pconp_mask = PCONP_PCTIM0,
        .tmr_rtos_interrupt_params = {
            .irp_name_p = "McRTOS Tick Timer Interrupt",
            .irp_isr_function_p = isr_timer0,
            .irp_arg_p =  (void *)&g_timer_devices[0],
            .irp_channel = VIC_CHANNEL_TIMER0,
            .irp_priority = VIC_VECT_PRIORITY2,
            .irp_cpu_id = 0,
        },
        .tmr_rtos_interrupt_pp = &g_rtos_interrupt_timer0,
    },
    [1] = {
        .tmr_signature = TIMER_DEVICE_SIGNATURE,
        .tmr_mmio_registers_p = (lpc2478_timer_t *)LPC2478_TIMER1_BASE_ADDR,
        .tmr_timer_pclk_shift = PCLK_TIMER1,
        .tmr_pclksel_index = 0,
        .tmr_pconp_mask = PCONP_PCTIM1,
    },
    [2] = {
        .tmr_signature = TIMER_DEVICE_SIGNATURE,
        .tmr_mmio_registers_p = (lpc2478_timer_t *)LPC2478_TIMER2_BASE_ADDR,
        .tmr_timer_pclk_shift = PCLK_TIMER2,
        .tmr_pclksel_index = 1,
        .tmr_pconp_mask = PCONP_PCTIM2,
    },
    [3] = {
        .tmr_signature = TIMER_DEVICE_SIGNATURE,
        .tmr_mmio_registers_p = (lpc2478_timer_t *)LPC2478_TIMER3_BASE_ADDR,
        .tmr_timer_pclk_shift = PCLK_TIMER3,
        .tmr_pclksel_index = 1,
        .tmr_pconp_mask = PCONP_PCTIM3,
    }
};
#endif // ???

/**
 * McRTOS interrupt object for the UART0 interrupts
 */
struct rtos_interrupt *g_rtos_interrupt_uart0_p = NULL;
 
/**
 * Global array of non-const structures for UART devices for the LPC2478
 * (allocated in SRAM space)
 */
static struct uart_device_var g_uart_devices_var[] =
{
    [0] = {
        .urt_initialized = false,
        .urt_received_bytes_dropped = 0,
        .urt_transmit_bytes_dropped = 0,
        },

    [1] = {
        .urt_initialized = false,
        .urt_received_bytes_dropped = 0,
        .urt_transmit_bytes_dropped = 0,
        },

    [2] = {
        .urt_initialized = false,
        .urt_received_bytes_dropped = 0,
        .urt_transmit_bytes_dropped = 0,
        },
};

static uint8_t uart0_transmit_queue_storage[UART_TRANSMIT_QUEUE_SIZE_IN_BYTES];
static uint8_t uart0_receive_queue_storage[UART_RECEIVE_QUEUE_SIZE_IN_BYTES];

/**
 * Global array of const structures for UART devices for the KL25Z SoC
 * (allocated in flash space)
 */
static const struct uart_device g_uart_devices[] =
{
    [0] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
        .urt_var_p = &g_uart_devices_var[0],
        .urt_mmio_uart0_p = UART0_BASE_PTR,
        .urt_mmio_tx_port_pcr_p = &PORTA_PCR1,
        .urt_mmio_rx_port_pcr_p = &PORTA_PCR2,
        .urt_mmio_pin_mux_selector_mask = PORT_PCR_MUX(0x2),
        .urt_mmio_clock_gate_mask = SIM_SCGC4_UART0_MASK,
        .urt_rtos_interrupt_params = {
            .irp_name_p = "UART0 Interrupt",
            .irp_isr_function_p = kl25_uart0_isr,
            .irp_arg_p =  (void *)&g_uart_devices[0],
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART0),
            .irp_priority =  SOC_HIGHEST_INTERRUPT_PRIORITY + 1, //SOC_LOWEST_INTERRUPT_PRIORITY - 1,
            .irp_cpu_id = 0,
        },

        .urt_rtos_interrupt_pp = &g_rtos_interrupt_uart0_p,
        .urt_transmit_queue_name_p = "UART0 transmit queue",
        .urt_receive_queue_name_p = "UART0 receive queue",
        .urt_transmit_queue_storage_p = uart0_transmit_queue_storage,
        .urt_receive_queue_storage_p = uart0_receive_queue_storage,
    },

    [1] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
        .urt_var_p = &g_uart_devices_var[1],
        .urt_mmio_uart_p = UART1_BASE_PTR,
        .urt_mmio_tx_port_pcr_p = &PORTC_PCR4,
        .urt_mmio_rx_port_pcr_p = &PORTC_PCR3,
        .urt_mmio_pin_mux_selector_mask = PORT_PCR_MUX(0x3),
        .urt_mmio_clock_gate_mask = SIM_SCGC4_UART1_MASK,
    },

    [2] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
        .urt_var_p = &g_uart_devices_var[2],
        .urt_mmio_uart_p = UART2_BASE_PTR,
        .urt_mmio_tx_port_pcr_p = &PORTD_PCR3,
        .urt_mmio_rx_port_pcr_p = &PORTD_PCR2,
        .urt_mmio_pin_mux_selector_mask = PORT_PCR_MUX(0x3),
        .urt_mmio_clock_gate_mask = SIM_SCGC4_UART2_MASK,
    },
};

C_ASSERT(
    ARRAY_SIZE(g_uart_devices) == ARRAY_SIZE(g_uart_devices_var));

const struct uart_device *const g_console_serial_port_p = &g_uart_devices[0];

/*
 *  A/D converter
 */

/**
 * McRTOS interrupt object for the A/D converter interrupts
 */
struct rtos_interrupt *g_rtos_interrupt_adc0_p = NULL;

#if 0 // ???
/**
 * Global const structure for the A/D converter device
 * (allocated in flash space)
 */
static const struct adc_device g_adc0_device = {
    .ad_signature = ADC_DEVICE_SIGNATURE,
    .ad_mmio_registers_p = (lpc2478_adc_t *)LPC2478_ADC_BASE_ADDR,
    .ad_rtos_interrupt_params = {
        .irp_name_p = "ADC Interrupt",
        .irp_isr_function_p = isr_adc,
        .irp_arg_p = (void *)&g_adc_device,
        .irp_channel = VIC_CHANNEL_AD0,
        .irp_priority = VIC_VECT_PRIORITY3,
        .irp_cpu_id = 0,
    },

    .ad_rtos_interrupt_pp = &g_rtos_interrupt_adc,

#   define ADC_CONDVAR_NAME_ENTRY(_channel) \
        [_channel] = "A/D channel " #_channel " condvar"

    .ad_channel_condvar_names = {
        ADC_CONDVAR_NAME_ENTRY(0),
        ADC_CONDVAR_NAME_ENTRY(1),
        ADC_CONDVAR_NAME_ENTRY(2),
        ADC_CONDVAR_NAME_ENTRY(3),
        ADC_CONDVAR_NAME_ENTRY(4),
        ADC_CONDVAR_NAME_ENTRY(5),
        ADC_CONDVAR_NAME_ENTRY(6),
        ADC_CONDVAR_NAME_ENTRY(7),
    },

    .ad_adc_channels_p = g_adc_channels,
};

const struct adc_device *const g_adc0_device_p = &g_adc0_device;
#else
const struct adc_device *const g_adc0_device_p = NULL; // TODO
#endif


/**
 *  Initializes board hardware. 
 *
 *  @pre This function must be called with interrupts disabled.
 */
cpu_reset_cause_t
board_init(void)
{
    cpu_status_register_t reg_primask = __get_PRIMASK();
   
    FDC_ASSERT(
        CPU_INTERRUPTS_ARE_DISABLED(reg_primask),
        reg_primask, 0);

    cpu_reset_cause_t reset_cause = find_reset_cause();

    //RCM_RPFC = RCM_RPFC_RSTFLTSS_MASK | RCM_RPFC_RSTFLTSRW(0x2); 
    //RCM_RPFC = RCM_RPFC_RSTFLTSRW(0x1); 
    system_clocks_init();

    rgb_led_init();

#ifdef DEBUG
    if (software_reset_happened()) {
        DEBUG_BLINK_LED(LED_COLOR_YELLOW);
    } else {
        DEBUG_BLINK_LED(LED_COLOR_GREEN);
    }
#   else
    if (software_reset_happened()) {
        (void)set_rgb_led_color(LED_COLOR_CYAN);
    } else {
        (void)set_rgb_led_color(LED_COLOR_GREEN);
    }
#   endif

    cortex_m_nvic_init();

    uart_init(
        g_console_serial_port_p,
        CONSOLE_SERIAL_PORT_BAUD_RATE,
        CONSOLE_SERIAL_PORT_MODE);

   
#   ifdef DEBUG
    uart_putchar_with_polling(g_console_serial_port_p, '\r');
    uart_putchar_with_polling(g_console_serial_port_p, '\n');
    //uart_putchar(g_console_serial_port_p, '\r');
    //uart_putchar(g_console_serial_port_p, '\n');
    DEBUG_PRINTF("UART0 initialized\n");
    DEBUG_PRINTF("Last reset cause: %#x\n", reset_cause);
#   else
    uart_putchar(g_console_serial_port_p, '\r');
    uart_putchar(g_console_serial_port_p, '\n');
#   endif

    init_adc(g_adc0_device_p);

    //??? tfc_gpio_init();

    return reset_cause;
}

void
board_reset(void)
{
    __disable_irq();

    /*
     * Stop all peripherals:
     */
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
    } if (reg_rcm_srs0 != 0) {
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
    return ((RCM_SRS1 & RCM_SRS1_SW_MASK) != 0);
}


static void
system_clocks_init(void)
{
    uint32_t reg_value;

    /*
     * Enable all of the port clocks. These have to be enabled to configure
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
     *
     * NOTE: This is not relaly needed, as these are the settings at reset time.
     */  
    write_32bit_mmio_register(
        &SIM_CLKDIV1,
        SIM_CLKDIV1_OUTDIV1(0) | SIM_CLKDIV1_OUTDIV4(1));

    /*
     * Initialize PLL:
     * 
     * NOTE: PLL will be the source for MCG CLKOUT so the core, system, and flash
     * clocks are derived from it
     */
    pll_init();

    /*
     * Set PLLFLLSEL to select the PLL, as the clock source for
     * peripherals:
     * (MCGPLLCLK clock with fixed divide by two)
     */
    reg_value = read_32bit_mmio_register(&SIM_SOPT2);
    reg_value |= SIM_SOPT2_PLLFLLSEL_MASK;
    write_32bit_mmio_register(&SIM_SOPT2, reg_value);
}


static void
pll_init(void)
{
  unsigned char frdiv_val;
  unsigned char temp_reg;
  unsigned char prdiv, vdiv;
  short i;
  fdc_error_t fdc_error;

  // check if in FEI mode
  if (!((((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x0) && // check CLKS mux has selcted FLL output
      (MCG_S & MCG_S_IREFST_MASK) &&                                  // check FLL ref is internal ref clk
      (!(MCG_S & MCG_S_PLLST_MASK))))                                 // check PLLS mux has selected FLL
  {
      fdc_error = CAPTURE_FDC_ERROR("Not in FEI mode", 0, 0);
      fatal_error_handler(fdc_error);
  }

  // configure the MCG_C2 register
  // the RANGE value is determined by the external frequency. Since the RANGE parameter affects the FRDIV divide value
  // it still needs to be set correctly even if the oscillator is not being used
      
  temp_reg = MCG_C2;
  temp_reg &= ~(MCG_C2_RANGE0_MASK | MCG_C2_HGO0_MASK | MCG_C2_EREFS0_MASK); // clear fields before writing new values
    
  temp_reg |= (MCG_C2_RANGE0(1) | (1 << MCG_C2_EREFS0_SHIFT));
  MCG_C2 = temp_reg;
  
  // determine FRDIV based on reference clock frequency
  frdiv_val = 3;

  // Select external oscillator and Reference Divider and clear IREFS to start ext osc
  // If IRCLK is required it must be enabled outside of this driver, existing state will be maintained
  // CLKS=2, FRDIV=frdiv_val, IREFS=0, IRCLKEN=0, IREFSTEN=0
  temp_reg = MCG_C1;
  temp_reg &= ~(MCG_C1_CLKS_MASK | MCG_C1_FRDIV_MASK | MCG_C1_IREFS_MASK); // Clear values in these fields
  temp_reg |= (MCG_C1_CLKS(2) | MCG_C1_FRDIV(frdiv_val)); // Set the required CLKS and FRDIV values
  MCG_C1 = temp_reg;

  // if the external oscillator is used need to wait for OSCINIT to set
    for (i = 0 ; i < 20000 ; i++)
    {
      if (MCG_S & MCG_S_OSCINIT0_MASK) break; // jump out early if OSCINIT sets before loop finishes
    }

  FDC_ASSERT(
    (MCG_S & MCG_S_OSCINIT0_MASK) != 0, 0, 0);

  // wait for Reference clock Status bit to clear
  for (i = 0 ; i < 2000 ; i++)
  {
    if (!(MCG_S & MCG_S_IREFST_MASK)) break; // jump out early if IREFST clears before loop finishes
  }

  FDC_ASSERT(
    (MCG_S & MCG_S_IREFST_MASK) == 0, 0, 0);

  // Wait for clock status bits to show clock source is ext ref clk
  for (i = 0 ; i < 2000 ; i++)
  {
    if (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x2) break; // jump out early if CLKST shows EXT CLK slected before loop finishes
  }

  FDC_ASSERT(
    ((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x2, 0, 0);

  // Now in FBE
  // It is recommended that the clock monitor is enabled when using an external clock as the clock source/reference.
  // It is enabled here but can be removed if this is not required.
  MCG_C6 |= MCG_C6_CME0_MASK;
  
  // Configure PLL
  // Configure MCG_C5
  // If the PLL is to run in STOP mode then the PLLSTEN bit needs to be OR'ed in here or in user code.
  temp_reg = MCG_C5;
  temp_reg &= ~MCG_C5_PRDIV0_MASK;
  temp_reg |= MCG_C5_PRDIV0(PLL_DIVIDER - 1);    //set PLL ref divider
  MCG_C5 = temp_reg;

  // Configure MCG_C6
  // The PLLS bit is set to enable the PLL, MCGOUT still sourced from ext ref clk
  // The loss of lock interrupt can be enabled by seperately OR'ing in the LOLIE bit in MCG_C6
  temp_reg = MCG_C6; // store present C6 value
  temp_reg &= ~MCG_C6_VDIV0_MASK; // clear VDIV settings
  temp_reg |= MCG_C6_PLLS_MASK | MCG_C6_VDIV0(PLL_MULTIPLIER - 24); // write new VDIV and enable PLL
  MCG_C6 = temp_reg; // update MCG_C6

  // wait for PLLST status bit to set
  for (i = 0 ; i < 2000 ; i++)
  {
    if (MCG_S & MCG_S_PLLST_MASK) break; // jump out early if PLLST sets before loop finishes
  }

  FDC_ASSERT(
    (MCG_S & MCG_S_PLLST_MASK) != 0, 0, 0);

  // Wait for LOCK bit to set
  for (i = 0 ; i < 4000 ; i++)
  {
    if (MCG_S & MCG_S_LOCK0_MASK) break; // jump out early if LOCK sets before loop finishes
  }

  FDC_ASSERT(
    (MCG_S & MCG_S_LOCK0_MASK) != 0, 0, 0);

  // Use actual PLL settings to calculate PLL frequency
  prdiv = ((MCG_C5 & MCG_C5_PRDIV0_MASK) + 1);
  vdiv = ((MCG_C6 & MCG_C6_VDIV0_MASK) + 24);

  // now in PBE

  MCG_C1 &= ~MCG_C1_CLKS_MASK; // clear CLKS to switch CLKS mux to select PLL as MCG_OUT

  // Wait for clock status bits to update
  for (i = 0 ; i < 2000 ; i++)
  {
    if (((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x3) break; // jump out early if CLKST = 3 before loop finishes
  }

  FDC_ASSERT(
    ((MCG_S & MCG_S_CLKST_MASK) >> MCG_S_CLKST_SHIFT) == 0x3, 0, 0);

  // Now in PEE

  g_pll_frequency_in_hz = (CRYSTAL_FREQUENCY_HZ / prdiv) * vdiv; //MCGOUT equals PLL output frequency
}

#if 0 // ???
//set I/O for H-BRIDGE enables, switches and LEDs
static void
tfc_gpio_init(void)
{
#   define TFC_HBRIDGE_EN_LOC   BIT(21)
#   define TFC_BAT_LED0_LOC     BIT(8)
#   define TFC_BAT_LED1_LOC     BIT(9) 
#   define TFC_BAT_LED2_LOC     BIT(10) 
#   define TFC_BAT_LED3_LOC     BIT(11) 

	//enable Clocks to all ports
	
	SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTB_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK | SIM_SCGC5_PORTE_MASK;

	//Setup Pins as GPIO
	PORTE_PCR21 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;   
	PORTE_PCR20 = PORT_PCR_MUX(1);    
	
	//Port for Pushbuttons
	PORTC_PCR13 = PORT_PCR_MUX(1);   
	PORTC_PCR17 = PORT_PCR_MUX(1);   
	
	
	//Ports for DIP Switches
	PORTE_PCR2 = PORT_PCR_MUX(1); 
	PORTE_PCR3 = PORT_PCR_MUX(1);
	PORTE_PCR4 = PORT_PCR_MUX(1); 
	PORTE_PCR5 = PORT_PCR_MUX(1);
	
	//Ports for LEDs
	PORTB_PCR8 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;   
	PORTB_PCR9 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;   
	PORTB_PCR10 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;   
	PORTB_PCR11 = PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK;   
	
	
	//Setup the output pins
    GPIOE_PDDR =  TFC_HBRIDGE_EN_LOC;  
    GPIOB_PDDR =  TFC_BAT_LED0_LOC	| TFC_BAT_LED1_LOC | TFC_BAT_LED2_LOC | TFC_BAT_LED3_LOC;

    //TFC_HBRIDGE_DISABLE:
    GPIOE_PCOR = TFC_HBRIDGE_EN_LOC;
}
#endif // # if 0

static uint32_t g_rgb_led_current_mask = 0x0;

static void 
rgb_led_init(void)
{
	/* Turn on clock to PortB and PortD module */
	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK|SIM_SCGC5_PORTD_MASK;
	
	/* Set the PTB18 pin multiplexer to GPIO mode */
	PORTB_PCR18 = PORT_PCR_MUX(1);
	
	/* Set the initial output state to high */
	GPIOB_PSOR |= LED_RED_PIN_MASK;
	
	/* Set the pins direction to output */
	GPIOB_PDDR |= LED_RED_PIN_MASK;
	
	/* Set the PTB19 pin multiplexer to GPIO mode */
	PORTB_PCR19 = PORT_PCR_MUX(1);
	
	/* Set the initial output state to high */
	GPIOB_PSOR |= LED_GREEN_PIN_MASK;
	
	/* Set the pins direction to output */
	GPIOB_PDDR |= LED_GREEN_PIN_MASK;
	
	/* Set the PTD1 pin multiplexer to GPIO mode */
	PORTD_PCR1 = PORT_PCR_MUX(1);
	
	/* Set the initial output state to high */
	GPIOD_PSOR = LED_BLUE_PIN_MASK;
	
	/* Set the pins direction to output */
	GPIOD_PDDR |= LED_BLUE_PIN_MASK;
}


void
toggle_heartbeat_led(void)
{
    toggle_rgb_led(LED_COLOR_BLUE);
}


void
toggle_rgb_led(uint32_t led_color_mask)
{
    g_rgb_led_current_mask ^= led_color_mask;

    if (led_color_mask & LED_BLUE_PIN_MASK) {
        GPIO_PTOR_REG(PTD_BASE_PTR) = LED_BLUE_PIN_MASK;
        led_color_mask &= ~LED_BLUE_PIN_MASK;
    }

    if (led_color_mask != 0x0) {
        GPIO_PTOR_REG(PTB_BASE_PTR) = led_color_mask;
    }
}


void
turn_on_rgb_led(uint32_t led_color_mask)
{
    g_rgb_led_current_mask |= led_color_mask;

    if (led_color_mask & LED_BLUE_PIN_MASK) {
        GPIO_PCOR_REG(PTD_BASE_PTR) = LED_BLUE_PIN_MASK;
        led_color_mask &= ~LED_BLUE_PIN_MASK;
    }

    if (led_color_mask != 0x0) {
        GPIO_PCOR_REG(PTB_BASE_PTR) = led_color_mask;
    }
}


void
turn_off_rgb_led(uint32_t led_color_mask)
{
    g_rgb_led_current_mask &= ~led_color_mask;

    if (led_color_mask & LED_BLUE_PIN_MASK) {
        GPIO_PSOR_REG(PTD_BASE_PTR) = LED_BLUE_PIN_MASK;
        led_color_mask &= ~LED_BLUE_PIN_MASK;
    }
    
    if (led_color_mask != 0x0) {
        GPIO_PSOR_REG(PTB_BASE_PTR) = led_color_mask;
    }
}


/*
 * Set the LED to the given color and returns the previous color
 */
uint32_t
set_rgb_led_color(uint32_t led_color_mask)
{
    uint32_t old_rgb_led_mask = g_rgb_led_current_mask;

    g_rgb_led_current_mask = led_color_mask;

    if (led_color_mask & LED_BLUE_PIN_MASK) {
        GPIO_PCOR_REG(PTD_BASE_PTR) = LED_BLUE_PIN_MASK;
    } else {
        GPIO_PSOR_REG(PTD_BASE_PTR) = LED_BLUE_PIN_MASK;
    }

    if (led_color_mask & LED_RED_PIN_MASK) {
        GPIO_PCOR_REG(PTB_BASE_PTR) = LED_RED_PIN_MASK;
    } else {
        GPIO_PSOR_REG(PTB_BASE_PTR) = LED_RED_PIN_MASK;
    }

    if (led_color_mask & LED_GREEN_PIN_MASK) {
        GPIO_PCOR_REG(PTB_BASE_PTR) = LED_GREEN_PIN_MASK;
    } else {
        GPIO_PSOR_REG(PTB_BASE_PTR) = LED_GREEN_PIN_MASK;
    }

    return old_rgb_led_mask;
}


void
uart_init(
    const struct uart_device *uart_device_p,
    uint32_t baud_rate,
    uint8_t mode)
{
    uint32_t reg_value;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct uart_device_var *uart_var_p = uart_device_p->urt_var_p;
    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    FDC_ASSERT(!uart_var_p->urt_initialized, 0, 0);
    FDC_ASSERT(mode == 0, mode, uart_device_p);
    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();

    /*
     * Enable clock for the UART:
     */
    reg_value = read_32bit_mmio_register(&SIM_SCGC4);
    reg_value |= uart_device_p->urt_mmio_clock_gate_mask;
    write_32bit_mmio_register(&SIM_SCGC4, reg_value);

    /*
     * Disable UART's transmitter and receiver, while UART is being
     * configured:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);

    /*
     * Configure the uart for 8-bit mode, no parity (mode is 0):
     */
    write_8bit_mmio_register(
        &UART_C1_REG(uart_mmio_registers_p), mode);

    /* 
     * Enable Tx pin:
     */
    write_32bit_mmio_register(
        uart_device_p->urt_mmio_tx_port_pcr_p,
        uart_device_p->urt_mmio_pin_mux_selector_mask | PORT_PCR_DSE_MASK);

    /* 
     * Enable Rx pin:
     */
    write_32bit_mmio_register(
        uart_device_p->urt_mmio_rx_port_pcr_p,
        uart_device_p->urt_mmio_pin_mux_selector_mask | PORT_PCR_DSE_MASK);

    // Calculate the first baud rate using the lowest OSR value possible.  
    //uint32_t uart0clk = CPU_CLOCK_FREQ_IN_HZ / 2;
    uint32_t uart0clk = g_pll_frequency_in_hz / 2;
    uint32_t i = 4;
    uint32_t sbr_val = uart0clk / (baud_rate * i);
    uint32_t calculated_baud = uart0clk / (i * sbr_val);
    uint32_t baud_diff;
        
    if (calculated_baud > baud_rate)
        baud_diff = calculated_baud - baud_rate;
    else
        baud_diff = baud_rate - calculated_baud;
    
    uint32_t osr_val = i;
       
    if (uart_device_p == &g_uart_devices[0]) 
    {
        /*
         * Select the clock source for the UART0 transmit and receive clock:
         * 01 =  MCGFLLCLK clock or MCGPLLCLK/2 clock
         */
        reg_value = read_32bit_mmio_register(&SIM_SOPT2);
        SET_BIT_FIELD(
            reg_value, SIM_SOPT2_UART0SRC_MASK, SIM_SOPT2_UART0SRC_SHIFT,
            0x1);
        write_32bit_mmio_register(&SIM_SOPT2, reg_value);

        UART0_MemMapPtr uart0_mmio_registers_p = uart_device_p->urt_mmio_uart0_p;

        // Select the best OSR value
        for (i = 5; i <= 32; i++)
        {
            uint32_t temp;

            sbr_val = uart0clk / (baud_rate * i);
            calculated_baud = uart0clk / (i * sbr_val);
            
            if (calculated_baud > baud_rate)
                temp = calculated_baud - baud_rate;
            else
                temp = baud_rate - calculated_baud;
            
            if (temp <= baud_diff)
            {
                baud_diff = temp;
                osr_val = i; 
            }
        }
        
        FDC_ASSERT(baud_diff < (baud_rate / 100) * 3, baud_diff, baud_rate);
    
        // If the OSR is between 4x and 8x then both
        // edge sampling MUST be turned on.  
        if (osr_val > 3 && osr_val < 9)
        {
            reg_value = read_8bit_mmio_register(&UART0_C5_REG(uart0_mmio_registers_p));
            reg_value |= UART0_C5_BOTHEDGE_MASK;
            write_8bit_mmio_register(&UART0_C5_REG(uart0_mmio_registers_p), reg_value);
        }
        
        // Setup OSR value 
        reg_value = read_8bit_mmio_register(&UART0_C4_REG(uart0_mmio_registers_p));
        SET_BIT_FIELD(reg_value, UART0_C4_OSR_MASK, UART0_C4_OSR_SHIFT, osr_val - 1);
        write_8bit_mmio_register(&UART0_C4_REG(uart0_mmio_registers_p), reg_value); 
    
        DBG_ASSERT(
            (reg_value & UART0_C4_OSR_MASK) + 1 == osr_val,
            reg_value, osr_val);

        sbr_val = uart0clk / (baud_rate * osr_val);
    }
    else
    {
        /* Calculate baud settings */
        sbr_val = uart0clk / (baud_rate * 16);
    }

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
        cpu_id,
        &uart_var_p->urt_transmit_queue);

    /*
     * Initialize receive queue:
     */
    rtos_k_byte_circular_buffer_init(
        uart_device_p->urt_receive_queue_name_p,
        UART_RECEIVE_QUEUE_SIZE_IN_BYTES,
        uart_device_p->urt_receive_queue_storage_p,
        NULL,
        cpu_id,
        &uart_var_p->urt_receive_queue);

    /* 
     * Register McRTOS interrupt handler
     */
    rtos_k_register_interrupt(
        &uart_device_p->urt_rtos_interrupt_params,
        uart_device_p->urt_rtos_interrupt_pp);

    DBG_ASSERT(
        *uart_device_p->urt_rtos_interrupt_pp != NULL, 
        uart_device_p->urt_rtos_interrupt_pp, uart_device_p);

    uart_var_p->urt_initialized = true;
}


static void
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


/**
 * UART common interrupt handler with interrupts enabled
 */
void
kl25_uart_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(rtos_interrupt_p);

    const struct uart_device *uart_device_p =
        (struct uart_device *)rtos_interrupt_p->int_arg_p;

    DBG_ASSERT(
        uart_device_p->urt_signature == UART_DEVICE_SIGNATURE,
        uart_device_p->urt_signature, uart_device_p);

    uint32_t reg_value;
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;
    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    DBG_ASSERT(
        (UART0_MemMapPtr)uart_mmio_registers_p == UART0_BASE_PTR ||
        uart_mmio_registers_p == UART1_BASE_PTR ||
        uart_mmio_registers_p == UART2_BASE_PTR,
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
     * "Transmit data register empty" interrupt:
     */ 
    if (s1_reg_value & UART_S1_TDRE_MASK) {
        /*
         * NOTE: This interrupt source will be cleared when
         * the next character is transmitted
         */

        uint8_t byte_to_transmit;

        bool entry_read = rtos_k_byte_circular_buffer_read(
                &uart_var_p->urt_transmit_queue,
                &byte_to_transmit,
                false);

        if (entry_read) {
            write_8bit_mmio_register(&UART_D_REG(uart_mmio_registers_p), byte_to_transmit);
        } else {
            /*
             * Disable "transmit data register empty" interrupt:
             */
            reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
            reg_value &= ~UART_C2_TIE_MASK;
            write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);
        }
    }

    /*
     * "Receive data register full" interrupt:
     */ 
    if (s1_reg_value & UART_S1_RDRF_MASK) {
        /*
         * Read the first byte received to clear the interrupt source. 
         */
        uint8_t byte_received = read_8bit_mmio_register(&UART_D_REG(uart_mmio_registers_p));

        bool entry_written = rtos_k_byte_circular_buffer_write(
                &uart_var_p->urt_receive_queue,
                byte_received,
                false);

        if (!entry_written) {
            uart_var_p->urt_received_bytes_dropped ++;
        }
    }
}


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
     * pending "transmit data register empty" interrupt empty, regardless
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
     * Enable generation of receive interrupts:
     */
    reg_value = read_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p));
    reg_value |= UART_C2_RIE_MASK;
    write_8bit_mmio_register(&UART_C2_REG(uart_mmio_registers_p), reg_value);

    bool entry_read = rtos_k_byte_circular_buffer_read(
        &uart_var_p->urt_receive_queue,
        &char_received,
        true);

    DBG_ASSERT(entry_read, uart_device_p, 0);

    return char_received;
}

/**
 * Reads the next character received from a UART serial port, doing polling
 * until the character is received.
 */
uint8_t
uart_getchar_with_polling(
    _IN_ const struct uart_device *uart_device_p)
{
    uint32_t reg_value;
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;
    UART_MemMapPtr uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    FDC_ASSERT(
        uart_var_p->urt_initialized, uart_var_p, uart_device_p);

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


static void
init_adc(const struct adc_device *adc_device_p)
{
#if 0 // ???
    uint32_t reg_value;

    FDC_ASSERT(
        adc_device_p->ad_signature == ADC_DEVICE_SIGNATURE,
        adc_device_p->ad_signature, adc_device_p);

    lpc2478_adc_t *adc_mmio_registers_p = adc_device_p->ad_mmio_registers_p;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
	
    /*
     * Turn on ADC power in the System Control Block
     */
    turn_on_power(PCONP_PCADC);

    uint32_t adc_pclksel_freq = get_peripheral_clock_freq(0, PCLK_ADC);
    
    FDC_ASSERT(adc_pclksel_freq != 0, 0, 0);

    /*
     *  Calculate value for the ADC clock divider:
     *
     * The APB clock (PCLK) is divided by this value plus one to produce the
     * clock for the A/D converter, which should be less than or equal to 4.5 MHz.
     *
     * adc_pclksel_freq / (adc_clock_divider + 1) = ADC_CLOCK_FREQUENCY
     *
     * So,
     * adc_clock_divider = (adc_pclk_freq / ADC_CLOCK_FREQUENCY) - 1
     */
    uint8_t adc_clock_divider = (adc_pclksel_freq / ADC_CLOCK_FREQUENCY) - 1;
  
    /*
     * Initialize of the AD0CR register:
     */

    reg_value = read_32bit_mmio_register(
                    &adc_mmio_registers_p->reg_AD0CR);

    reg_value &= ~AD0CR_BURST_MASK;
    CLEAR_BIT_FIELD(reg_value, AD0CR_SEL_MASK);
    CLEAR_BIT_FIELD(reg_value, AD0CR_START_MASK);
    SET_BIT_FIELD(reg_value, AD0CR_CLKDIV_MASK, AD0CR_CLKDIV_SHIFT,
                  adc_clock_divider);
    reg_value |= AD0CR_PDN_MASK;

    write_reg_AD0CR(&adc_mmio_registers_p->reg_AD0CR, reg_value);

    for (uint8_t i = 0; i < NUM_ADC_CHANNELS; i ++)
    {
        struct adc_channel *adc_channel_p = &adc_device_p->ad_adc_channels_p[i];

        rtos_k_condvar_init(
            adc_device_p->ad_channel_condvar_names[i],
            cpu_id,
            &adc_channel_p->adc_condvar);

        adc_channel_p->adc_conversion_completed = false;
    }

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    /*
     * Install ISR before enabling interrupt source:
     */
    rtos_k_register_interrupt(
        &adc_device_p->ad_rtos_interrupt_params,
        adc_device_p->ad_rtos_interrupt_pp);

    DBG_ASSERT(
        *adc_device_p->ad_rtos_interrupt_pp != NULL,
        adc_device_p->ad_rtos_interrupt_pp, cpu_id);

    /*
     * Enable a separate interrupt for each A/D channel in the A/D converter:
     */
    write_32bit_mmio_register(
        &adc_mmio_registers_p->reg_AD0INTEN,
        AD0INTEN_ADINTEN_MASK);

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);

#else
    FDC_ASSERT(adc_device_p == NULL, adc_device_p, 0); // TODO
#endif
}


void
kl25_adc_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    FDC_ASSERT(
        rtos_interrupt_p == g_rtos_interrupt_adc0_p,
        rtos_interrupt_p, g_rtos_interrupt_adc0_p);
}


#if 0 // ???
/**
 * Clear last interrupt for the given UART
 */ 
void
clear_adc_interrupt_source(
    _IN_ const struct adc_device *adc_device_p)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();
    DBG_ASSERT(
        adc_device_p->ad_signature == ADC_DEVICE_SIGNATURE,
        adc_device_p->ad_signature, adc_device_p);

    uint32_t reg_value;
    lpc2478_adc_t *adc_mmio_registers_p = adc_device_p->ad_mmio_registers_p;

    FDC_ASSERT_INTERRUPT_SOURCE_IS_SET(VIC_CHANNEL_AD0);

    reg_value = read_32bit_mmio_register(&adc_mmio_registers_p->reg_AD0STAT);
                
    FDC_ASSERT(
        (reg_value & AD0STAT_ADINT_MASK) != 0 &&
        (reg_value & AD0STAT_DONE_MASK) != 0,
        reg_value, 0);

    uint32_t adc_done_mask = (reg_value & AD0STAT_DONE_MASK);

    for (uint8_t i = 0; i < NUM_ADC_CHANNELS; i ++)
    {
        if (adc_done_mask & BIT(i))
        {
            struct adc_channel *adc_channel_p = &g_adc_channels[i];

            /*
             * Read the corresponding reg_AD0DR register, to clear the
             * interrupt source:
             */
            reg_value = read_32bit_mmio_register(&adc_mmio_registers_p->reg_AD0DR[i]);

            adc_channel_p->adc_result = GET_BIT_FIELD(
                reg_value, AD0DR_V_OVER_VREF_MASK, AD0DR_V_OVER_VREF_SHIFT);

            FDC_ASSERT(
                !adc_channel_p->adc_conversion_completed,
                adc_channel_p, i);

            adc_channel_p->adc_conversion_completed = true;
        }
    }
}


/** 
 * A/D converter interrupt handler
 */
void
adc_interrupt_handler(
    _IN_ const struct adc_device *adc_device_p)
{
    DBG_ASSERT(
        adc_device_p->ad_signature == ADC_DEVICE_SIGNATURE,
        adc_device_p->ad_signature, adc_device_p);

    struct rtos_interrupt *adc_interrupt_p =
        *adc_device_p->ad_rtos_interrupt_pp;

    FDC_ASSERT_RTOS_INTERRUPT_HANDLER_PRECONDITIONS(adc_interrupt_p);
 
    /*
     * Signal condition variables of the channels for an A/D conversion
     * has completed 
     */
    for (uint8_t i = 0; i < NUM_ADC_CHANNELS; i ++)
    {
        struct adc_channel *adc_channel_p = &g_adc_channels[i];
            
        if (adc_channel_p->adc_conversion_completed)
        {
            rtos_k_condvar_signal(&adc_channel_p->adc_condvar);
        }
    }
}


void
select_input_pin_adc_channel(
    _IN_ uint8_t adc_channel)
{
    uint32_t reg_value;

    FDC_ASSERT(adc_channel < NUM_ADC_CHANNELS, adc_channel, NUM_ADC_CHANNELS);

    switch (adc_channel)
    {
    case 0:
        reg_value = read_32bit_mmio_register(
                        &g_pin_connect_block->reg_PINSEL[1]);

        reg_value |= PINSEL_MASK(PINSEL_ALT_01, 7); /* P0.23 */

        write_32bit_mmio_register(
            &g_pin_connect_block->reg_PINSEL[1], reg_value);

        break;

    case 1:
        reg_value = read_32bit_mmio_register(
                        &g_pin_connect_block->reg_PINSEL[1]);

        reg_value |= PINSEL_MASK(PINSEL_ALT_01, 8); /* P0.24 */

        write_32bit_mmio_register(
            &g_pin_connect_block->reg_PINSEL[1], reg_value);

        break;

    case 2:
        reg_value = read_32bit_mmio_register(
                        &g_pin_connect_block->reg_PINSEL[1]);

        reg_value |= PINSEL_MASK(PINSEL_ALT_01, 9); /* P0.25 */

        write_32bit_mmio_register(
            &g_pin_connect_block->reg_PINSEL[1], reg_value);

        break;

    case 3:
        reg_value = read_32bit_mmio_register(
                        &g_pin_connect_block->reg_PINSEL[1]);

        reg_value |= PINSEL_MASK(PINSEL_ALT_01, 10); /* P0.26 */

        write_32bit_mmio_register(
            &g_pin_connect_block->reg_PINSEL[1], reg_value);

        break;
	
    case 4:
        reg_value = read_32bit_mmio_register(
                        &g_pin_connect_block->reg_PINSEL[3]);

        reg_value |= PINSEL_MASK(PINSEL_ALT_11, 14); /* P1.30 */

        write_32bit_mmio_register(
            &g_pin_connect_block->reg_PINSEL[3], reg_value);

        break;

    case 5:
        reg_value = read_32bit_mmio_register(
                        &g_pin_connect_block->reg_PINSEL[3]);

        reg_value |= PINSEL_MASK(PINSEL_ALT_11, 15); /* P1.31 */

        write_32bit_mmio_register(
            &g_pin_connect_block->reg_PINSEL[3], reg_value);

        break;

    case 6:
        reg_value = read_32bit_mmio_register(
                        &g_pin_connect_block->reg_PINSEL[0]);

        reg_value |= PINSEL_MASK(PINSEL_ALT_11, 12); /* P0.12 */

        write_32bit_mmio_register(
            &g_pin_connect_block->reg_PINSEL[0], reg_value);

        break;

    case 7:
        reg_value = read_32bit_mmio_register(
                        &g_pin_connect_block->reg_PINSEL[0]);

        reg_value |= PINSEL_MASK(PINSEL_ALT_11, 13); /* P0.13 */

        write_32bit_mmio_register(
            &g_pin_connect_block->reg_PINSEL[0], reg_value);

        break;
    }
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
    fdc_error_t fdc_error;
    struct adc_channel *adc_channel_p;

    FDC_ASSERT(
        adc_device_p->ad_signature == ADC_DEVICE_SIGNATURE,
        adc_device_p->ad_signature, adc_device_p);

    FDC_ASSERT(adc_channel < NUM_ADC_CHANNELS, adc_channel, NUM_ADC_CHANNELS);

    lpc2478_adc_t *adc_mmio_registers_p = adc_device_p->ad_mmio_registers_p;

    adc_channel_p = &g_adc_channels[adc_channel];
    FDC_ASSERT(
        !adc_channel_p->adc_conversion_completed,
        adc_channel_p, adc_channel);

    reg_value = read_32bit_mmio_register(
                    &adc_mmio_registers_p->reg_AD0CR);

    /*
     * Select the channel pin:
     */ 
    SET_BIT_FIELD(reg_value, AD0CR_SEL_MASK, AD0CR_SEL_SHIFT, BIT(adc_channel));
   
    /*
     * Indicate that we want to start the A/D conversion now:
     */
    
    SET_BIT_FIELD(reg_value, AD0CR_START_MASK, AD0CR_START_SHIFT, 0x1);
   
    write_reg_AD0CR(&adc_mmio_registers_p->reg_AD0CR, reg_value);

    /*
     * Wait until the conversion is complete:
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    while (!adc_channel_p->adc_conversion_completed)
    {
        rtos_k_condvar_wait_interrupt(&adc_channel_p->adc_condvar);
    }

    adc_channel_p->adc_conversion_completed = false;

#if 0 // XXX Remove this code
    uint32_t adc_done_mask;
    do {
        reg_value = read_32bit_mmio_register(&adc_mmio_registers_p->reg_AD0STAT);
                
        FDC_ASSERT(
            (reg_value & AD0STAT_ADINT_MASK) != 0 &&
            (reg_value & AD0STAT_DONE_MASK) != 0,
            reg_value, 0);

        adc_done_mask = (reg_value & AD0STAT_DONE_MASK);
    } while ((adc_done_mask & BIT(adc_channel)) == 0);

    reg_value = read_32bit_mmio_register(&adc_mmio_registers_p->reg_AD0DR[adc_channel]);

    adc_channel_p->adc_result = GET_BIT_FIELD(
        reg_value, AD0DR_V_OVER_VREF_MASK, AD0DR_V_OVER_VREF_SHIFT);
#endif // XXX

    rtos_k_restore_cpu_interrupts(cpu_status_register);

    return adc_channel_p->adc_result;
}


void
init_trimpot(void)
{
    select_input_pin_adc_channel(TRIMPOT_ADC_CHANNEL);
}
    
uint32_t
read_trimpot(void)
{
    return read_adc_channel(g_adc0_device_p, TRIMPOT_ADC_CHANNEL);
}
#endif


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
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_mcm_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved20))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ftfl_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_FTFA))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pmc_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_LVD_LVW))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_llw_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_LLW))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_i2c0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2C0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_i2c1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2C1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart2_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART2))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_cmp0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CMP0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ftm0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_TPM0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ftm1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_TPM1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ftm2_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_TPM2))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_rtc_alarm_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_RTC))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_rtc_seconds_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_RTC_Seconds))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pit_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIT))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_reserved_irq23_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved39))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_usb_otg_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_USB0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dac0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DAC0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_tsi0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_TSI0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_mcg_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_MCG))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_lptimer_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_LPTimer))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_reserved_irq29_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved45))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_port_a_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PORTA))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_port_d_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PORTD))

/**
 * Generate function that reads from an MMIO register of the given bit width
 */
#define GEN_READ_MMIO_REGISTER_FUNCTION(_func_name, _reg_type, _value_type) \
    _value_type                                                         \
    _func_name(volatile _reg_type *io_reg_p)                            \
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

