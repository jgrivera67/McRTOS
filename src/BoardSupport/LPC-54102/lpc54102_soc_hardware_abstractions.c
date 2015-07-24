/**
 * @file lpc54102_soc_hardware_abstractions.c
 *
 * Hardware abstraction layer for the LPC54102 SoC
 *
 * @author German Rivera
 */

#include "hardware_abstractions.h"
#include "lpc54102_soc.h"
#include "lpcxpresso_board.h"
#include "McRTOS_arm_cortex_m.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"

/*
 * Disable compiler optimizations for this file, since it contains
 * timing sensitive functions
 */
#pragma GCC optimize "O0"

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

static cpu_reset_cause_t find_reset_cause(void);

#ifdef _CPU_CYCLES_MEASURE_
static void init_cpu_clock_cycles_counter(void);
#endif

static void uart_stop(
    const struct uart_device *uart_device_p);

/*
 * Function prototypes for external interrupt ISRs
 */
static isr_function_t dummy_wdt_irq_isr;
static isr_function_t dummy_bod_irq_isr;
static isr_function_t dummy_reserved0_irq_isr;
static isr_function_t dummy_dma_irq_isr;
static isr_function_t dummy_gint0_irq_isr;
static isr_function_t dummy_pin_int0_irq_isr;
static isr_function_t dummy_pin_int1_irq_isr;
static isr_function_t dummy_pin_int2_irq_isr;
static isr_function_t dummy_pin_int3_irq_isr;
static isr_function_t dummy_utick_irq_isr;
static isr_function_t dummy_mrt_irq_isr;
static isr_function_t dummy_ct32b0_irq_isr;
static isr_function_t dummy_ct32b1_irq_isr;
static isr_function_t dummy_ct32b2_irq_isr;
static isr_function_t dummy_ct32b3_irq_isr;
static isr_function_t dummy_ct32b4_irq_isr;
static isr_function_t dummy_sct0_irq_isr;
static isr_function_t dummy_uart1_irq_isr;
static isr_function_t dummy_uart2_irq_isr;
static isr_function_t dummy_uart3_irq_isr;
static isr_function_t dummy_i2c0_irq_isr;
static isr_function_t dummy_i2c1_irq_isr;
static isr_function_t dummy_i2c2_irq_isr;
static isr_function_t dummy_spi0_irq_isr;
static isr_function_t dummy_spi1_irq_isr;
static isr_function_t dummy_adc_seqa_irq_isr;
static isr_function_t dummy_adc_seqb_irq_isr;
static isr_function_t dummy_adc_thcmp_irq_isr;
static isr_function_t dummy_rtc_irq_isr;
static isr_function_t dummy_reserved1_irq_isr;
static isr_function_t dummy_mailbox_irq_isr;
static isr_function_t dummy_int_gint1_irq_isr;
static isr_function_t dummy_pin_int4_irq_isr;
static isr_function_t dummy_pin_int5_irq_isr;
static isr_function_t dummy_pin_int6_irq_isr;
static isr_function_t dummy_pin_int7_irq_isr;
static isr_function_t dummy_spi2_irq_isr;
static isr_function_t dummy_spi3_irq_isr;
static isr_function_t dummy_reserved2_irq_isr;
static isr_function_t dummy_rit_irq_isr;
static isr_function_t dummy_reserved3_irq_isr;
static isr_function_t dummy_reserved4_irq_isr;
static isr_function_t dummy_reserved5_irq_isr;
static isr_function_t dummy_reserved6_irq_isr;

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
    [INT_WDT_IRQ] = dummy_wdt_irq_isr,
    [INT_BOD_IRQ] = dummy_bod_irq_isr,
    [INT_Reserved0_IRQ] = dummy_reserved0_irq_isr,
    [INT_DMA_IRQ] = dummy_dma_irq_isr,
    [INT_GINT0_IRQ] = dummy_gint0_irq_isr,
    [INT_PIN_INT0_IRQ] = dummy_pin_int0_irq_isr,
    [INT_PIN_INT1_IRQ] = dummy_pin_int1_irq_isr,
    [INT_PIN_INT2_IRQ] = dummy_pin_int2_irq_isr,
    [INT_PIN_INT3_IRQ] = dummy_pin_int3_irq_isr,
    [INT_UTICK_IRQ] = dummy_utick_irq_isr,
    [INT_MRT_IRQ] = dummy_mrt_irq_isr,
    [INT_CT32B0_IRQ] = dummy_ct32b0_irq_isr,
    [INT_CT32B1_IRQ] = dummy_ct32b1_irq_isr,
    [INT_CT32B2_IRQ] = dummy_ct32b2_irq_isr,
    [INT_CT32B3_IRQ] = dummy_ct32b3_irq_isr,
    [INT_CT32B4_IRQ] = dummy_ct32b4_irq_isr,
    [INT_SCT0_IRQ] = dummy_sct0_irq_isr,
    [INT_UART0_IRQ] = lpc54102_uart0_irq_isr,
    [INT_UART1_IRQ] = dummy_uart1_irq_isr,
    [INT_UART2_IRQ] = dummy_uart2_irq_isr,
    [INT_UART3_IRQ] = dummy_uart3_irq_isr,
    [INT_I2C0_IRQ] = dummy_i2c0_irq_isr,
    [INT_I2C1_IRQ] = dummy_i2c1_irq_isr,
    [INT_I2C2_IRQ] = dummy_i2c2_irq_isr,
    [INT_SPI0_IRQ] = dummy_spi0_irq_isr,
    [INT_SPI1_IRQ] = dummy_spi1_irq_isr,
    [INT_ADC_SEQA_IRQ] = dummy_adc_seqa_irq_isr,
    [INT_ADC_SEQB_IRQ] = dummy_adc_seqb_irq_isr,
    [INT_ADC_THCMP_IRQ] = dummy_adc_thcmp_irq_isr,
    [INT_RTC_IRQ] = dummy_rtc_irq_isr,
    [INT_Reserved1_IRQ] = dummy_reserved1_irq_isr,
    [INT_MAILBOX_IRQ] = dummy_mailbox_irq_isr,

    /* External Interrupts - For M4 only */
    [INT_GINT1_IRQ] = dummy_int_gint1_irq_isr,
    [INT_PIN_INT4_IRQ] = dummy_pin_int4_irq_isr,
    [INT_PIN_INT5_IRQ] = dummy_pin_int5_irq_isr,
    [INT_PIN_INT6_IRQ] = dummy_pin_int6_irq_isr,
    [INT_PIN_INT7_IRQ] = dummy_pin_int7_irq_isr,
    [INT_SPI2_IRQ] = dummy_spi2_irq_isr,
    [INT_SPI3_IRQ] = dummy_spi3_irq_isr,
    [INT_Reserved2_IRQ] = dummy_reserved2_irq_isr,
    [INT_RIT_IRQ] = dummy_rit_irq_isr,
    [INT_Reserved3_IRQ] = dummy_reserved3_irq_isr,
    [INT_Reserved4_IRQ] = dummy_reserved4_irq_isr,
    [INT_Reserved5_IRQ] = dummy_reserved5_irq_isr,
    [INT_Reserved6_IRQ] = dummy_reserved6_irq_isr,
};

C_ASSERT(
    ARRAY_SIZE(g_interrupt_vector_table) ==
    CORTEX_M_IRQ_VECTOR_BASE + SOC_NUM_INTERRUPT_CHANNELS);

/**
 * McRTOS interrupt object for the UART0 interrupt
 */
struct rtos_interrupt *g_rtos_interrupt_uart0_p = NULL;

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
};


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
        .urt_mmio_uart_p = LPC_USART0,
        .urt_tx_pin = PIN_INITIALIZER(PIN_PORT_0, 1, IOCON_FUNC1),
        .urt_rx_pin = PIN_INITIALIZER(PIN_PORT_0, 0, IOCON_FUNC1),
        .urt_async_apb_control_mask = BIT(1),
        .urt_rtos_interrupt_params = {
            .irp_name_p = "UART0 Interrupt",
            .irp_isr_function_p = lpc54102_uart0_irq_isr,
            .irp_arg_p =  (void *)&g_uart_devices[0],
            .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART0_IRQ),
            .irp_priority = UART0_INTERRUPT_PRIORITY,
            .irp_cpu_id = 0,
        },

        .urt_rtos_interrupt_pp = &g_rtos_interrupt_uart0_p,

        .urt_transmit_queue_name_p = "UART0 transmit queue",
        .urt_receive_queue_name_p = "UART0 receive queue",
    },
};

C_ASSERT(
    ARRAY_SIZE(g_uart_devices) == ARRAY_SIZE(g_uart_devices_var));

const struct uart_device *const g_console_serial_port_p = &g_uart_devices[0];

/**
 * Matrix to keep track of what pins are currently in use. If a pin is not in
 * use (set_pin_function() has not been called for it), its entry is NULL.
 */
static const struct pin_info *g_pins_in_use_map[NUM_PIN_PORTS][NUM_PINS_PER_PORT];

static void
pll_init(void)
{
    uint32_t reg_value;

    /*
     * Select the PLL input to the IRC
     */
    write_32bit_mmio_register(&LPC_SYSCON->SYSPLLCLKSEL, 0x0);

    /*
     * Power down PLL to change the PLL divider ratio
     */
    write_32bit_mmio_register(&LPC_SYSCON->PDRUNCFGSET, SYSCON_PDRUNCFG_PD_SYS_PLL);

    /*
     * Get System PLL input clock rate:
     */
    reg_value = read_32bit_mmio_register(&LPC_SYSCON->SYSPLLCLKSEL);
    uint_fast8_t clk_sel = GET_BIT_FIELD(reg_value, SYSPLLCLKSEL_SEL_MASK, SYSPLLCLKSEL_SEL_SHIFT);
    FDC_ASSERT(clk_sel == 0x0, clk_sel, 0);

    /*
     * Set PLL output based on desired CPU clock rate:
     */

    /* Baseline parameters (no input or output dividers) */
    uint32_t pllPreDivider = 3;	/* SYSCON_IRC_FREQ / GCD(SYSCON_IRC_FREQ, CPU_CLOCK_FREQ_IN_HZ) */
    uint32_t pllPostDivider = 0;	/* 0 implies post-divider will be disabled */
    uint32_t multFccoDiv = 1;

    /* Determine PLL multipler */
    uint32_t nDivOutHz = SYSCON_IRC_FREQ / pllPreDivider;
    uint32_t pllMultiplier = (CPU_CLOCK_FREQ_IN_HZ / nDivOutHz) / multFccoDiv;

    /* Will bumping up M by 1 get us closer to the desired CCO frequency? */
    if ((nDivOutHz * ((multFccoDiv * pllMultiplier * 2) + 1)) < (CPU_CLOCK_FREQ_IN_HZ * 2)) {
            pllMultiplier++;
    }

    /*
     * Setup filtering:
     */
    uint32_t pllSelP;
    uint32_t pllSelI;
    uint32_t pllSelR;

    pllFindSel(pllMultiplier, true, &pllSelP, &pllSelI, &pllSelR);

    /*
     * Power off PLL during setup changes
     */
    write_32bit_mmio_register(&LPC_SYSCON->PDRUNCFGSET,
                              SYSCON_PDRUNCFG_PD_SYS_PLL);

    /*
     * Setup PLL:
     */

    /*
     * Set PLL control:
     */
    reg_value =
            (pllSelR << SYSCON_SYSPLLCTRL_SELR_P) |	    /* Filter coefficient */
            (pllSelI << SYSCON_SYSPLLCTRL_SELI_P) |	    /* Filter coefficient */
            (pllSelP << SYSCON_SYSPLLCTRL_SELP_P) |	    /* Filter coefficient */
            (1 << SYSCON_SYSPLLCTRL_BYPASS_FBDIV2_P) |      /* Extra M / 2 divider? */
            (1 << SYSCON_SYSPLLCTRL_BANDSEL_SSCGREG_N_P) |  /* Manual bandwidth selection enabled */
            (1 << SYSCON_SYSPLLCTRL_DIRECTO_P);	            /* Bypass post-divider */

    write_32bit_mmio_register(&LPC_SYSCON->SYSPLLCTRL, reg_value);

    /*
     * Set N (prediv):
     */
    reg_value = PLL_NDEC_VAL_SET(pllEncodeN(pllPreDivider));

    write_32bit_mmio_register(&LPC_SYSCON->SYSPLLNDEC, reg_value);
    write_32bit_mmio_register(&LPC_SYSCON->SYSPLLNDEC,
                              reg_value | PLL_NDEC_NREQ /*latch*/
                             );

    /*
     * Set P (postdiv):
     */
    reg_value = PLL_PDEC_VAL_SET(pllEncodeP(pllPostDivider));
    write_32bit_mmio_register(&LPC_SYSCON->SYSPLLPDEC,
                              reg_value);
    write_32bit_mmio_register(&LPC_SYSCON->SYSPLLPDEC,
                              reg_value | PLL_PDEC_PREQ /*latch*/
                             );

    /*
     * Set (mult) and use manual filter, disable SS mode:
     */
    reg_value = PLL_SSCG0_MDEC_VAL_SET(pllEncodeM(pllMultiplier)) |
                (1 << PLL_SSCG0_SEL_EXT_SSCG_N_P);
    write_32bit_mmio_register(&LPC_SYSCON->SYSPLLSSCTRL[0],
                              reg_value);
    write_32bit_mmio_register(&LPC_SYSCON->SYSPLLSSCTRL[0],
                              reg_value | PLL_SSCG0_MREQ /*latch*/
                             );

    /*
     * Power down SSC, not used
     */
    write_32bit_mmio_register(&LPC_SYSCON->SYSPLLSSCTRL[1],
                              PLL_SSCG1_MOD_PD_SSCGCLK_N);
    write_32bit_mmio_register(&LPC_SYSCON->SYSPLLSSCTRL[1],
                              PLL_SSCG1_MOD_PD_SSCGCLK_N | PLL_SSCG1_MD_REQ /*latch*/
                             );

    /*
     * Power on PLL:
     */
    write_32bit_mmio_register(&LPC_SYSCON->PDRUNCFGCLR,
                              SYSCON_PDRUNCFG_PD_SYS_PLL);

    /*
     * Wait until PLL is locked:
     */
    do {
        reg_value = read_32bit_mmio_register(&LPC_SYSCON->SYSPLLSTAT);
    } while ((reg_value & 1) == 0);
}


static void
system_clocks_init(void)
{

    uint32_t reg_value;

    /*
     * Enable IOCON clock and all GPIO port clocks:
     */
    reg_value = SYSCON0_IOCON_MASK |
                SYSCON0_GPIO0_MASK |
                SYSCON0_GPIO1_MASK;
# if 0 //???
                SYSCON0_PINT_MASK |
                SYSCON0_GINT_MASK;
#endif

    write_32bit_mmio_register(&LPC_SYSCON->AHBCLKCTRLSET[0], reg_value);

    /*
     * Turn on the IRC by clearing the power down bit:
     */
    write_32bit_mmio_register(&LPC_SYSCON->PDRUNCFGCLR,
                              SYSCON_PDRUNCFG_PD_IRC_OSC | SYSCON_PDRUNCFG_PD_IRC);

    pll_init();

    /*
     * Set system clock divider to 1:
     */
    write_32bit_mmio_register(&LPC_SYSCON->AHBCLKDIV, 0x1);

    /*
     * Set main clock source to the system PLL. This will drive 24MHz
     * for the main clock and 24MHz for the system clock
     */
    write_32bit_mmio_register(&LPC_SYSCON->MAINCLKSELB, 0x2);

    /*
     * ASYSNC SYSCON needs to be on or all serial peripheral won't work.
     */
    write_32bit_mmio_register(&LPC_SYSCON->ASYNCAPBCTRL, 0x1);

    /*
     * Set asynchronous APB clock divider (for peripherals attached to the async
     * APB bridge):
     */
    write_32bit_mmio_register(&LPC_ASYNC_SYSCON->ASYNCCLKDIV, 0x1);

    /*
     * Set asynchronous APB clock source to IRC input:
     */
    write_32bit_mmio_register(&LPC_ASYNC_SYSCON->ASYNCAPBCLKSELA, 0x0);
    write_32bit_mmio_register(&LPC_ASYNC_SYSCON->ASYNCAPBCLKSELB, 0x3);

    /*
     * Enable clocks for system FIFO peripheral and clear its reset:
     * (used for UART and SPI interfaces)
     */
    write_32bit_mmio_register(&LPC_SYSCON->AHBCLKCTRLSET[1],
                              SYSCON1_FIFO_MASK);
    write_32bit_mmio_register(&LPC_SYSCON->PRESETCTRLCLR[1],
                              SYSCON1_FIFO_MASK);
}


/**
 *  SoC-specific early initialization to be invoked at the beginning of
 *  the Reset exception handler.
 */
void
soc_early_init(void)
{
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

    cortex_m_nvic_init();

    uart_init(
        g_console_serial_port_p,
        CONSOLE_SERIAL_PORT_BAUD_RATE,
        CONSOLE_SERIAL_PORT_MODE);

    //???    uart_putchar_with_polling(g_console_serial_port_p, '8'); //???

    capture_fdc_msg_printf("%s initialized (Tx FIFO size: %u, Rx FIFO size: %u)\n",
		 g_console_serial_port_p->urt_name,
		 g_console_serial_port_p->urt_var_p->urt_tx_fifo_size,
		 g_console_serial_port_p->urt_var_p->urt_rx_fifo_size);

    cortex_m_mpu_init();
    return reset_cause;
}


void
soc_reset(void)
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
    cpu_reset_cause_t generic_cause;
    uint32_t reg_value = read_32bit_mmio_register(&LPC_SYSCON->SYSRSTSTAT);

    if (reg_value & SYSCON_RST_POR) {
        generic_cause = GRC_POWER_ON_RESET;
    } else if (reg_value & SYSCON_RST_EXTRST) {
        generic_cause = GRC_EXTERNAL_PIN_RESET;
    } else if (reg_value & SYSCON_RST_WDT) {
        generic_cause = GRC_WATCHDOG_RESET;
    } else if (reg_value & SYSCON_RST_SYSRST) {
        generic_cause = GRC_SOFTWARE_RESET;
    } else if (reg_value != 0) {
        generic_cause = GRC_OTHER_HW_REASON_RESET;
    }

    return generic_cause;
}


bool
software_reset_happened(void)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();

    uint32_t reg_value = read_32bit_mmio_register(&LPC_SYSCON->SYSRSTSTAT);

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }

    return (reg_value & SYSCON_RST_SYSRST) != 0;
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

    write_32bit_mmio_register(
        &LPC_IOCON->PIO[pin_p->pin_port][pin_p->pin_index],
        pin_p->pin_function | pin_flags);

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

    __IO uint32_t *dir_reg_p = &LPC_GPIO->DIR[gpio_pin_p->pin_info.pin_port];

   if (is_output) {

        reg_value = read_32bit_mmio_register(dir_reg_p);
        reg_value |= gpio_pin_p->pin_bit_mask;
        write_32bit_mmio_register(dir_reg_p, reg_value);
    } else {
        reg_value = read_32bit_mmio_register(dir_reg_p);
        reg_value &= ~gpio_pin_p->pin_bit_mask;
        write_32bit_mmio_register(dir_reg_p, reg_value);
    }

    rtos_k_restore_cpu_interrupts(cpu_status_register);

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


void
activate_output_pin(const struct gpio_pin *gpio_pin_p)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();

#   ifdef DEBUG
    uint32_t reg_value =
        read_32bit_mmio_register(&LPC_GPIO->DIR[gpio_pin_p->pin_info.pin_port]);

    FDC_ASSERT(
        reg_value & gpio_pin_p->pin_bit_mask,
        gpio_pin_p, reg_value);
#   endif

    if (gpio_pin_p->pin_is_active_high) {
        write_32bit_mmio_register(&LPC_GPIO->SET[gpio_pin_p->pin_info.pin_port],
                                  gpio_pin_p->pin_bit_mask);
    } else {
        write_32bit_mmio_register(&LPC_GPIO->CLR[gpio_pin_p->pin_info.pin_port],
                                  gpio_pin_p->pin_bit_mask);
    }

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


void
deactivate_output_pin(const struct gpio_pin *gpio_pin_p)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();

#   ifdef DEBUG
    uint32_t reg_value =
        read_32bit_mmio_register(&LPC_GPIO->DIR[gpio_pin_p->pin_info.pin_port]);

    FDC_ASSERT(
        reg_value & gpio_pin_p->pin_bit_mask,
        reg_value, gpio_pin_p->pin_bit_mask);
#   endif

    if (gpio_pin_p->pin_is_active_high) {
        write_32bit_mmio_register(&LPC_GPIO->CLR[gpio_pin_p->pin_info.pin_port],
                                  gpio_pin_p->pin_bit_mask);
    } else {
        write_32bit_mmio_register(&LPC_GPIO->SET[gpio_pin_p->pin_info.pin_port],
                                  gpio_pin_p->pin_bit_mask);
    }

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


void
toggle_output_pin(const struct gpio_pin *gpio_pin_p)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();

#   ifdef DEBUG
    uint32_t reg_value =
        read_32bit_mmio_register(&LPC_GPIO->DIR[gpio_pin_p->pin_info.pin_port]);

    FDC_ASSERT(
        reg_value & gpio_pin_p->pin_bit_mask,
        gpio_pin_p, reg_value);
#   endif

    write_32bit_mmio_register(&LPC_GPIO->NOT[gpio_pin_p->pin_info.pin_port],
                              gpio_pin_p->pin_bit_mask);

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


bool
read_input_pin(const struct gpio_pin *gpio_pin_p)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();

    uint32_t reg_value =
        read_32bit_mmio_register(&LPC_GPIO->PIN[gpio_pin_p->pin_info.pin_port]);

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
    uint_fast8_t uart_index = uart_device_p - g_uart_devices;

    FDC_ASSERT(
        uart_device_p->urt_signature == UART_DEVICE_SIGNATURE,
        uart_device_p->urt_signature, uart_device_p);

    struct uart_device_var *uart_var_p = uart_device_p->urt_var_p;
    UART_REGS_T *uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    FDC_ASSERT(!uart_var_p->urt_initialized, uart_device_p, 0);
    FDC_ASSERT(mode == 0, mode, uart_device_p);
    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();

    /*
     * Enable clock for the UART:
     */
    write_32bit_mmio_register(&LPC_ASYNC_SYSCON->ASYNCAPBCLKCTRLSET,
                              uart_device_p->urt_async_apb_control_mask);

    /*
     * Clear reset for the UART:
     */
    write_32bit_mmio_register(&LPC_ASYNC_SYSCON->ASYNCPRESETCTRLCLR,
                              uart_device_p->urt_async_apb_control_mask);

    /*
     * Configure the uart for 8-bit mode, no parity, 1 stop bit (mode is 0):
     */
    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->CFG);
    SET_BIT_FIELD(reg_value, UART_CFG_DATALEN_MASK, UART_CFG_DATALEN_SHIFT, 0x1);
    SET_BIT_FIELD(reg_value, UART_CFG_PARITYSEL_MASK, UART_CFG_PARITYSEL_SHIFT, 0x0);
    reg_value &= ~UART_CFG_STOPLEN_MASK;
    write_32bit_mmio_register(&uart_mmio_registers_p->CFG, reg_value);

    /*
     * Get Tx and Rx FIFO space total sizes
     */
    reg_value = read_32bit_mmio_register(&LPC_FIFO->common.FIFOCTLUSART);
    uint_fast8_t total_tx_fifo_space_size = GET_BIT_FIELD(
                                                reg_value,
					        FIFOCTLUSART_TXFIFOTOTAL_MASK,
					        FIFOCTLUSART_TXFIFOTOTAL_SHIFT);

    FDC_ASSERT(total_tx_fifo_space_size != 0 &&
               total_tx_fifo_space_size % LPC_FIFO_USART_MAX == 0,
               total_tx_fifo_space_size, LPC_FIFO_USART_MAX);

    uint_fast8_t total_rx_fifo_space_size = GET_BIT_FIELD(
                                                reg_value,
					        FIFOCTLUSART_RXFIFOTOTAL_MASK,
					        FIFOCTLUSART_RXFIFOTOTAL_SHIFT);

    FDC_ASSERT(total_rx_fifo_space_size != 0 &&
               total_rx_fifo_space_size % LPC_FIFO_USART_MAX == 0,
               total_rx_fifo_space_size, LPC_FIFO_USART_MAX);

    /*
     * Configure Tx and RX FIFOs:
     * - Pause all UARTs Rx FIFO operations and Tx FIFO operations
     * - Allocate Rx and Tx FIFO space for this UART
     * - Tx FIFO threshold = total num entries (generate interrupt when Tx FIFO is empty)
     * - Rx FIFO threshold = 0 (generate interrupt when Rx FIFO is not empty)
     * - Flush Tx and Rx FIFOs
     * - Resume all UARTs Rx FIFO operations and Tx FIFO operations
     */
    write_32bit_mmio_register(
        &LPC_FIFO->common.FIFOCTLUSART,
        FIFOCTLUSART_RXPAUSE_MASK | FIFOCTLUSART_TXPAUSE_MASK);

    uart_var_p->urt_tx_fifo_size = total_tx_fifo_space_size / LPC_FIFO_USART_MAX;
    uart_var_p->urt_rx_fifo_size = total_rx_fifo_space_size / LPC_FIFO_USART_MAX;

    reg_value = 0;
    SET_BIT_FIELD(reg_value, FIFOCFGUSART_TXSIZE_MASK, FIFOCFGUSART_TXSIZE_SHIFT,
                  uart_var_p->urt_tx_fifo_size);
    SET_BIT_FIELD(reg_value, FIFOCFGUSART_RXSIZE_MASK, FIFOCFGUSART_RXSIZE_SHIFT,
                  uart_var_p->urt_rx_fifo_size);
    write_32bit_mmio_register(
        &LPC_FIFO->common.FIFOCFGUSART[uart_index], reg_value);

    write_32bit_mmio_register(
        &LPC_FIFO->common.FIFOUPDATEUSART,
        FIFOUPDATEUSART_RXSIZE_MASK(uart_index) |
        FIFOUPDATEUSART_TXSIZE_MASK(uart_index));

    reg_value = 0;
    SET_BIT_FIELD(reg_value, FIFO_UART_CFGUSART_TX_THRESHOLD_MASK,
                  FIFO_UART_CFGUSART_TX_THRESHOLD_SHIFT,
                  uart_var_p->urt_tx_fifo_size);
    SET_BIT_FIELD(reg_value, FIFO_UART_CFGUSART_RX_THRESHOLD_MASK,
                  FIFO_UART_CFGUSART_RX_THRESHOLD_SHIFT,
                  0);
    write_32bit_mmio_register(
        &LPC_FIFO->usart[uart_index].CFG, reg_value);

#if 0
    write_32bit_mmio_register(
        &LPC_FIFO->usart[uart_index].CTLSET,
        FIFO_UART_CTL_RXFLUSH_MASK |
        FIFO_UART_CTL_TXFLUSH_MASK);
#endif

     /* Resume all UART FIFOs: */
    write_32bit_mmio_register(&LPC_FIFO->common.FIFOCTLUSART, 0x0);

    uart_var_p->urt_fifos_enabled = true;

    /*
     * Configure Tx and Rx pins:
     */
    set_pin_function(&uart_device_p->urt_tx_pin, IOCON_DIGITAL_EN);
    set_pin_function(&uart_device_p->urt_rx_pin, IOCON_DIGITAL_EN);

   /*
    * Calculate baud rate settings (assuming OSRVAL = 16):
    */
    reg_value = SYSCON_IRC_FREQ / (baud_rate * 16);

    FDC_ASSERT(reg_value >= 1 && reg_value <= UINT16_MAX + 1,
               uart_device_p, reg_value);

#if 1 //???
    /*
     * Set Baud rat generator (BRG) register (BRG):
     */
    write_32bit_mmio_register(&uart_mmio_registers_p->BRG, reg_value - 1);
#else
    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->CTL);
    reg_value |= UART_CTL_AUTOBAUD_MASK;
    write_32bit_mmio_register(&uart_mmio_registers_p->CTL, reg_value);
#endif

    /*
     * Enable UART:
     */
    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->CFG);
    reg_value |= UART_CFG_ENABLE_MASK;
    write_32bit_mmio_register(&uart_mmio_registers_p->CFG, reg_value);

    /*
     * Initialize transmit queue:
     */
    rtos_k_byte_circular_buffer_init(
        uart_device_p->urt_transmit_queue_name_p,
        UART_TRANSMIT_QUEUE_SIZE_IN_BYTES,
        uart_var_p->urt_transmit_queue_storage,
        NULL,
        &uart_var_p->urt_transmit_queue);

    /*
     * Initialize receive queue:
     */
    rtos_k_byte_circular_buffer_init(
        uart_device_p->urt_receive_queue_name_p,
        UART_RECEIVE_QUEUE_SIZE_IN_BYTES,
        uart_var_p->urt_receive_queue_storage,
        NULL,
        &uart_var_p->urt_receive_queue);

    /*
     * Register McRTOS interrupt handler for UART Rx/Tx interrupt:
     */
    rtos_k_register_interrupt(
        &uart_device_p->urt_rtos_interrupt_params,
        uart_device_p->urt_rtos_interrupt_pp);

    DBG_ASSERT(
        *uart_device_p->urt_rtos_interrupt_pp != NULL,
        uart_device_p->urt_rtos_interrupt_pp, uart_device_p);

    /*
     * NOTE: Generation of Tx/Rx FIFO interrupts is enabled in
     * uart_putchar() and uart_getchar() respectively
     */
    uart_var_p->urt_initialized = true;
}


static void
uart_stop(
    const struct uart_device *uart_device_p)
{
    uint32_t reg_value;
    struct uart_device_var *uart_var_p = uart_device_p->urt_var_p;
    UART_REGS_T *uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    /*
     * Disable UART:
     */
    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->CFG);
    reg_value &= ~UART_CFG_ENABLE_MASK;
    write_32bit_mmio_register(&uart_mmio_registers_p->CFG, reg_value);

    /*
     * Disable clock for the UART:
     */
    write_32bit_mmio_register(&LPC_ASYNC_SYSCON->ASYNCAPBCLKCTRLCLR,
                              uart_device_p->urt_async_apb_control_mask);

    uart_var_p->urt_initialized = false;
}


/**
 * Handling of "Tx FIFO empty" interrupt
 */
static void
lpc54102_fifo_uart_tx_interrupt_handling(uint_fast8_t uart_index,
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

	write_32bit_mmio_register(&LPC_FIFO->usart[uart_index].TXDAT,
				 byte_to_transmit);
    }

    rtos_k_restore_cpu_interrupts(cpu_status_register);

    if (sw_transmit_queue_empty) {
	/*
	 * Disable "transmit FIFO empty" interrupt:
	 */
        write_32bit_mmio_register(
            &LPC_FIFO->usart[uart_index].CTLCLR,
            FIFO_UART_CTL_TXTHINTEN_MASK);
    }
}


/**
 * Handling of "Rx FIFO not empty" interrupt
 */
static void
lpc54102_fifo_uart_rx_interrupt_handling(uint_fast8_t uart_index,
                                         uint_fast8_t rx_fifo_length,
				         struct uart_device_var *restrict uart_var_p)
{
    uint_fast8_t i;
    uint_fast8_t byte_received;
    bool entry_written;
    uint32_t reg_value;

    DBG_ASSERT(
        rx_fifo_length > 0 && rx_fifo_length <= uart_var_p->urt_rx_fifo_size,
        rx_fifo_length, uart_var_p->urt_rx_fifo_size);

    /*
     * Drain the Rx FIFO:
     */
    for (i = 0; i < rx_fifo_length; i++) {
        reg_value =
            read_32bit_mmio_register(&LPC_FIFO->usart[uart_index].RXDATSTAT);

        if (reg_value & (FIFO_UART_RXDATSTAT_FRAMERR_MASK |
                         FIFO_UART_RXDATSTAT_PARITYERR_MASK |
                         FIFO_UART_RXDATSTAT_RXNOISE_MASK)) {
            CAPTURE_FDC_ERROR("UART receive error", reg_value, uart_index);
            continue;
        }

        byte_received = GET_BIT_FIELD(reg_value, FIFO_UART_RXDATSTAT_RXDAT_MASK,
                                      FIFO_UART_RXDATSTAT_RXDAT_SHIFT);

        entry_written = rtos_k_byte_circular_buffer_write(
                            &uart_var_p->urt_receive_queue,
                            byte_received,
                            false);

        if (!entry_written) {
            uart_var_p->urt_received_bytes_dropped ++;
            break;
        }
    }
}


/**
 * UART interrupt handler with interrupts enabled
 */
void
lpc54102_uart_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p)
{
    uint32_t reg_value;

    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(rtos_interrupt_p);

    const struct uart_device *uart_device_p =
        (struct uart_device *)rtos_interrupt_p->int_arg_p;

    DBG_ASSERT(
        uart_device_p->urt_signature == UART_DEVICE_SIGNATURE,
        uart_device_p->urt_signature, uart_device_p);

    struct uart_device_var *const restrict uart_var_p = uart_device_p->urt_var_p;
    uint_fast8_t uart_index = uart_device_p - g_uart_devices;

    DBG_ASSERT(uart_index < LPC_FIFO_USART_MAX, uart_index, LPC_FIFO_USART_MAX);

    DBG_ASSERT(
        uart_var_p->urt_initialized, uart_device_p, 0);

    reg_value = read_32bit_mmio_register(&LPC_FIFO->usart[uart_index].INTSTAT);

    /*
     * There is at least one Rx or Tx FIFO interrupt pending
     */
    FDC_ASSERT(
        (reg_value & (FIFO_UART_STAT_RXTH_MASK | FIFO_UART_STAT_TXTH_MASK)) != 0,
        reg_value, uart_device_p);

    /*
     * "Tx FIFO empty" interrupt:
     */
    if (reg_value & FIFO_UART_STAT_TXTH_MASK) {
#       ifdef DEBUG
        /*
         * NOTE: These asserts holds even if a higher prior interrupt handler
         * preempt us and calls uart_putchar(), since physical writes to the
         * UART FIFO only happen in this interrupt handler.
         */
        DBG_ASSERT(reg_value & FIFO_UART_STAT_TXEMPTY_MASK,
                   reg_value, uart_index);

        uint_fast8_t tx_fifo_free_count =
            GET_BIT_FIELD(reg_value,
                          FIFO_UART_STAT_TXCOUNT_MASK,
                          FIFO_UART_STAT_TXCOUNT_SHIFT);

        DBG_ASSERT(tx_fifo_free_count == uart_var_p->urt_tx_fifo_size,
                   tx_fifo_free_count, uart_var_p->urt_tx_fifo_size);
#       endif

	lpc54102_fifo_uart_tx_interrupt_handling(uart_index, uart_var_p);
    }

    /*
     * "Rx FIFO not empty" interrupt:
     */
    if (reg_value & FIFO_UART_STAT_RXTH_MASK) {
        DBG_ASSERT((reg_value & FIFO_UART_STAT_RXEMPTY_MASK) == 0,
                   reg_value, uart_index);

        uint_fast8_t rx_fifo_length =
            GET_BIT_FIELD(reg_value,
                          FIFO_UART_STAT_RXCOUNT_MASK,
                          FIFO_UART_STAT_RXCOUNT_SHIFT);

        DBG_ASSERT(rx_fifo_length != 0, uart_device_p, uart_index);

	lpc54102_fifo_uart_rx_interrupt_handling(uart_index, rx_fifo_length,
                                                 uart_var_p);
    }
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
    uint_fast8_t uart_index = uart_device_p - g_uart_devices;

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
     * Enable generation of "transmit FIFO empty" interrupts if
     * necessary:
     *
     * NOTE: if the "transmit FIFO" is empty there will be a
     * pending "transmit FIFO empty" interrupt, regardless
     * of this interrupt being enabled or not in the UART. Thus, as soon
     * as we enable the generation of this interrupt, the interrupt will
     * fire, if the "transmit FIFO" was empty.
     */
    uint32_t reg_value =
        read_32bit_mmio_register(&LPC_FIFO->usart[uart_index].CTLSET);

    if ((reg_value & FIFO_UART_CTL_TXTHINTEN_MASK) == 0) {
        reg_value |= FIFO_UART_CTL_TXTHINTEN_MASK;
        write_32bit_mmio_register(&LPC_FIFO->usart[uart_index].CTLSET, reg_value);
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
    uint_fast8_t uart_index = uart_device_p - g_uart_devices;
    uint32_t reg_value;

    if (!uart_var_p->urt_initialized) {
        return;
    }

    /*
     * Disable "transmit FIFO empty" interrupt:
     */
    write_32bit_mmio_register(
        &LPC_FIFO->usart[uart_index].CTLCLR,
        FIFO_UART_CTL_TXTHINTEN_MASK);

    /*
     * Do polling until the UART's transmit FIFO is empty:
     */
    for ( ; ; ) {
        reg_value = read_32bit_mmio_register(&LPC_FIFO->usart[uart_index].STAT);
        if (reg_value & FIFO_UART_STAT_TXEMPTY_MASK) {
            break;
        }
    }

#   ifdef DEBUG
    uint_fast8_t tx_fifo_free_count =
        GET_BIT_FIELD(reg_value,
                      FIFO_UART_STAT_TXCOUNT_MASK,
                      FIFO_UART_STAT_TXCOUNT_SHIFT);

        DBG_ASSERT(tx_fifo_free_count == uart_var_p->urt_tx_fifo_size,
                   tx_fifo_free_count, uart_var_p->urt_tx_fifo_size);
#   endif

    write_32bit_mmio_register(&LPC_FIFO->usart[uart_index].TXDAT, c);
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
    uint_fast8_t uart_index = uart_device_p - g_uart_devices;
    uint32_t reg_value;
    uint8_t char_received;

    DBG_ASSERT(
        uart_var_p->urt_initialized, uart_var_p, uart_device_p);

    DBG_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    /*
     * Enable generation of receive interrupts, if necessary:
     */
    reg_value = read_32bit_mmio_register(&LPC_FIFO->usart[uart_index].CTLSET);
    if ((reg_value & FIFO_UART_CTL_RXTHINTEN_MASK) == 0) {
        reg_value |= FIFO_UART_CTL_RXTHINTEN_MASK;
        write_32bit_mmio_register(&LPC_FIFO->usart[uart_index].CTLSET, reg_value);
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
    uint_fast8_t uart_index = uart_device_p - g_uart_devices;

#   ifdef _RELIABILITY_CHECKS_
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;

    FDC_ASSERT(
        uart_var_p->urt_initialized, uart_var_p, uart_device_p);
#   endif

    /*
     * Disable generation of receive interrupts:
     */
    write_32bit_mmio_register(
        &LPC_FIFO->usart[uart_index].CTLCLR,
        FIFO_UART_CTL_RXTHINTEN_MASK);

    /*
     * Do polling until the UART's receive FIFO is not empty:
     */
try_again:
    for ( ; ; ) {
        reg_value = read_32bit_mmio_register(&LPC_FIFO->usart[uart_index].STAT);
        if ((reg_value & FIFO_UART_STAT_RXEMPTY_MASK) == 0) {
            break;
        }
    }

#   ifdef DEBUG
    uint_fast8_t rx_fifo_length =
        GET_BIT_FIELD(reg_value,
                      FIFO_UART_STAT_RXCOUNT_MASK,
                      FIFO_UART_STAT_RXCOUNT_SHIFT);

        DBG_ASSERT(rx_fifo_length > 0 &&
                   rx_fifo_length <= uart_var_p->urt_rx_fifo_size,
                   rx_fifo_length, uart_var_p->urt_rx_fifo_size);
#   endif

    reg_value =
        read_32bit_mmio_register(&LPC_FIFO->usart[uart_index].RXDATSTAT);

    if (reg_value & (FIFO_UART_RXDATSTAT_FRAMERR_MASK |
                     FIFO_UART_RXDATSTAT_PARITYERR_MASK |
                     FIFO_UART_RXDATSTAT_RXNOISE_MASK)) {
        CAPTURE_FDC_ERROR("UART receive error", reg_value, uart_index);
        goto try_again;
    }

    uint_fast8_t byte_received =
        GET_BIT_FIELD(reg_value, FIFO_UART_RXDATSTAT_RXDAT_MASK,
                      FIFO_UART_RXDATSTAT_RXDAT_SHIFT);

    return byte_received;
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
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_wdt_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_WDT_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_bod_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_BOD_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_reserved0_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved0_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_dma_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_DMA_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_gint0_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_GINT0_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pin_int0_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIN_INT0_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pin_int1_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIN_INT1_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pin_int2_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIN_INT2_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pin_int3_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIN_INT3_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_utick_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UTICK_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_mrt_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_MRT_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ct32b0_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CT32B0_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ct32b1_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CT32B1_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ct32b2_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CT32B2_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ct32b3_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CT32B3_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_ct32b4_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CT32B4_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_sct0_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SCT0_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart1_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART1_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart2_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART2_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart3_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART3_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_i2c0_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2C0_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_i2c1_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2C1_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_i2c2_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2C2_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi0_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI0_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi1_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI1_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_adc_seqa_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ADC_SEQA_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_adc_seqb_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ADC_SEQB_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_adc_thcmp_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_ADC_THCMP_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_rtc_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_RTC_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_reserved1_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved1_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_mailbox_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_MAILBOX_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_int_gint1_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_GINT1_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pin_int4_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIN_INT4_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pin_int5_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIN_INT5_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pin_int6_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIN_INT6_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_pin_int7_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PIN_INT7_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi2_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI2_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi3_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI3_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_reserved2_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved2_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_rit_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_RIT_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_reserved3_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved3_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_reserved4_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved4_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_reserved5_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved5_IRQ))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_reserved6_irq_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_Reserved6_IRQ))

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

