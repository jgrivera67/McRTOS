/**
 * @file lm4f120_soc_hardware_abstractions.c
 *
 * Hardware abstraction layer for the TI Stellaris LM24F120 SoC
 *
 * @author German Rivera
 */

#include "hardware_abstractions.h"
#include "lm4f120_soc.h"
#include "tivaware/rom.h"
#include "tivaware/sysctl.h"
#include "launchpad_board.h"
#include "McRTOS_arm_cortex_m.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"

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


static cpu_reset_cause_t find_reset_cause(void);

static void system_clocks_init(void);

#ifdef _CPU_CYCLES_MEASURE_
static void init_cpu_clock_cycles_counter(void);
#endif

#if 0 // ???
static void uart_stop(
    const struct uart_device *uart_device_p);
#endif

/*
 * Function prototypes for dummy VIC interrupt ISRs
 */
static isr_function_t dummy_isr;

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

    /* Interrupts external to the Cortex-M core */
    [INT_GPIOA] = dummy_isr,          // GPIO Port A
    [INT_GPIOB] = dummy_isr,          // GPIO Port B
    [INT_GPIOC] = dummy_isr,          // GPIO Port C
    [INT_GPIOD] = dummy_isr,          // GPIO Port D
    [INT_GPIOE] = dummy_isr,          // GPIO Port E
    [INT_UART0] = dummy_isr,          // UART0
    [INT_UART1] = dummy_isr,          // UART1
    [INT_SSI0] = dummy_isr,          // SSI0
    [INT_I2C0] = dummy_isr,          // I2C0
    [INT_PWM0_FAULT] = dummy_isr,          // PWM0 Fault
    [INT_PWM0_0] = dummy_isr,          // PWM0 Generator 0
    [INT_PWM0_1] = dummy_isr,          // PWM0 Generator 1
    [INT_PWM0_2] = dummy_isr,          // PWM0 Generator 2
    [INT_QEI0] = dummy_isr,          // QEI0
    [INT_ADC0SS0] = dummy_isr,          // ADC0 Sequence 0
    [INT_ADC0SS1] = dummy_isr,          // ADC0 Sequence 1
    [INT_ADC0SS2] = dummy_isr,          // ADC0 Sequence 2
    [INT_ADC0SS3] = dummy_isr,          // ADC0 Sequence 3
    [INT_WATCHDOG] = dummy_isr,          // Watchdog Timers 0 and 1
    [INT_TIMER0A] = dummy_isr,          // 16/32-Bit Timer 0A
    [INT_TIMER0B] = dummy_isr,          // 16/32-Bit Timer 0B
    [INT_TIMER1A] = dummy_isr,          // 16/32-Bit Timer 1A
    [INT_TIMER1B] = dummy_isr,          // 16/32-Bit Timer 1B
    [INT_TIMER2A] = dummy_isr,          // 16/32-Bit Timer 2A
    [INT_TIMER2B] = dummy_isr,          // 16/32-Bit Timer 2B
    [INT_COMP0] = dummy_isr,          // Analog Comparator 0
    [INT_COMP1] = dummy_isr,          // Analog Comparator 1
    [INT_SYSCTL] = dummy_isr,          // System Control
    [INT_FLASH] = dummy_isr,          // Flash Memory Control and EEPROM
                                            // Control
    [INT_GPIOF] = dummy_isr,          // GPIO Port F
    [INT_UART2] = dummy_isr,          // UART2
    [INT_SSI1] = dummy_isr,          // SSI1
    [INT_TIMER3A] = dummy_isr,          // Timer 3A
    [INT_TIMER3B] = dummy_isr,          // Timer 3B
    [INT_I2C1] = dummy_isr,          // I2C1
    [INT_QEI1] = dummy_isr,          // QEI1
    [INT_CAN0] = dummy_isr,          // CAN0
    [INT_CAN1] = dummy_isr,          // CAN1
    [INT_HIBERNATE] = dummy_isr,          // Hibernation Module
    [INT_USB0] = dummy_isr,          // USB
    [INT_PWM0_3] = dummy_isr,          // PWM Generator 3
    [INT_UDMA] = dummy_isr,          // uDMA Software
    [INT_UDMAERR] = dummy_isr,          // uDMA Error
    [INT_ADC1SS0] = dummy_isr,          // ADC1 Sequence 0
    [INT_ADC1SS1] = dummy_isr,          // ADC1 Sequence 1
    [INT_ADC1SS2] = dummy_isr,          // ADC1 Sequence 2
    [INT_ADC1SS3] = dummy_isr,          // ADC1 Sequence 3
    [INT_SSI2] = dummy_isr,          // SSI2
    [INT_SSI3] = dummy_isr,          // SSI3
    [INT_UART3] = dummy_isr,          // UART3
    [INT_UART4] = dummy_isr,          // UART4
    [INT_UART5] = dummy_isr,          // UART5
    [INT_UART6] = dummy_isr,          // UART6
    [INT_UART7] = dummy_isr,          // UART7
    [INT_I2C2] = dummy_isr,          // I2C2
    [INT_I2C3] = dummy_isr,          // I2C3
    [INT_TIMER4A] = dummy_isr,          // 16/32-Bit Timer 4A
    [INT_TIMER4B] = dummy_isr,          // 16/32-Bit Timer 4B
    [INT_TIMER5A] = dummy_isr,         // 16/32-Bit Timer 5A
    [INT_TIMER5B] = dummy_isr,         // 16/32-Bit Timer 5B
    [INT_WTIMER0A] = dummy_isr,         // 32/64-Bit Timer 0A
    [INT_WTIMER0B] = dummy_isr,         // 32/64-Bit Timer 0B
    [INT_WTIMER1A] = dummy_isr,         // 32/64-Bit Timer 1A
    [INT_WTIMER1B] = dummy_isr,         // 32/64-Bit Timer 1B
    [INT_WTIMER2A] = dummy_isr,         // 32/64-Bit Timer 2A
    [INT_WTIMER2B] = dummy_isr,         // 32/64-Bit Timer 2B
    [INT_WTIMER3A] = dummy_isr,         // 32/64-Bit Timer 3A
    [INT_WTIMER3B] = dummy_isr,         // 32/64-Bit Timer 3B
    [INT_WTIMER4A] = dummy_isr,         // 32/64-Bit Timer 4A
    [INT_WTIMER4B] = dummy_isr,         // 32/64-Bit Timer 4B
    [INT_WTIMER5A] = dummy_isr,         // 32/64-Bit Timer 5A
    [INT_WTIMER5B] = dummy_isr,         // 32/64-Bit Timer 5B
    [INT_SYSEXC] = dummy_isr,         // System Exception (imprecise)
    [INT_PWM1_0] = dummy_isr,         // PWM1 Generator 0
    [INT_PWM1_1] = dummy_isr,         // PWM1 Generator 1
    [INT_PWM1_2] = dummy_isr,         // PWM1 Generator 2
    [INT_PWM1_3] = dummy_isr,         // PWM1 Generator 3
    [INT_PWM1_FAULT] = dummy_isr,         // PWM1 Fault
};

C_ASSERT(
    ARRAY_SIZE(g_interrupt_vector_table) ==
    CORTEX_M_IRQ_VECTOR_BASE + SOC_NUM_INTERRUPT_CHANNELS);

#if 0 // ???
/**
 * McRTOS interrupt object for the UART0 interrupts
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
            .irp_priority = UART0_INTERRUPT_PRIORITY,
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

#else
const struct uart_device *const g_console_serial_port_p = NULL;
#endif

/**
 * Hardware Micro trace buffer (MTB)
 */
#if 0
uint64_t __attribute__ ((section(".mtb_buf")))
    g_micro_trace_buffer[MICRO_TRACE_BUFFER_NUM_ENTRIES];
#endif

/**
 *  Initializes board hardware.
 *
 *  @pre This function must be called with interrupts disabled.
 */
cpu_reset_cause_t
soc_hardware_init(void)
{
    cpu_status_register_t reg_primask = __get_PRIMASK();

    FDC_ASSERT(
        CPU_INTERRUPTS_ARE_DISABLED(reg_primask),
        reg_primask, 0);

    cpu_reset_cause_t reset_cause = find_reset_cause();

    system_clocks_init();

#   ifdef _CPU_CYCLES_MEASURE_
    init_cpu_clock_cycles_counter();
#   endif

    bool mpu_present = cortex_m_mpu_init();

    cortex_m_nvic_init();

#if 0
    uart_init(
        g_console_serial_port_p,
        CONSOLE_SERIAL_PORT_BAUD_RATE,
        CONSOLE_SERIAL_PORT_MODE);
#else
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
#endif

#   ifdef DEBUG
    uart_putchar_with_polling(g_console_serial_port_p, '\r');
    uart_putchar_with_polling(g_console_serial_port_p, '\n');
    DEBUG_PRINTF("UART0 initialized\n");
    DEBUG_PRINTF("Last reset cause: %#x\n", reset_cause);
    DEBUG_PRINTF("MPU %s present\n", mpu_present ? "" : "not");
#   else
    uart_putchar(g_console_serial_port_p, '\r');
    uart_putchar(g_console_serial_port_p, '\n');
#   endif

    i2c_init(g_i2c0_device_p);
    return reset_cause;
}


void
soc_reset(void)
{
    __disable_irq();

    /*
     * Stop all peripherals:
     */
#if 0 // ???
    uart_stop(g_console_serial_port_p);
#endif

    /*
     * Trigger software reset in the SoC:
     */
    NVIC_SystemReset();
}


static cpu_reset_cause_t
find_reset_cause(void)
{
    return GRC_INVALID_RESET_CAUSE; // TODO
}


bool
software_reset_happened(void)
{
    return false; // TODO
}


static void
system_clocks_init(void)
{
    /*
     * Set the clocking to run at 50 MHz from the PLL.
     */
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);
}

static bool g_micro_trace_initialized = false;

void
micro_trace_init(void)
{
    uint32_t reg_value;

    reg_value = read_32bit_mmio_register(&MTB_BASE);

    FDC_ASSERT(
        reg_value == SOC_SRAM_BASE, reg_value, SOC_SRAM_BASE);

    DBG_ASSERT(
        (uintptr_t)__micro_trace_buffer % sizeof(uint64_t) == 0 &&
        (uintptr_t)__micro_trace_buffer == SOC_SRAM_BASE,
        __micro_trace_buffer, SOC_SRAM_BASE);

    DBG_ASSERT(
        (uintptr_t)__micro_trace_buffer_end % sizeof(uint64_t) == 0,
        __micro_trace_buffer_end, 0);

    DBG_ASSERT(
        __micro_trace_buffer_end - __micro_trace_buffer ==
            MICRO_TRACE_BUFFER_NUM_ENTRIES,
        __micro_trace_buffer, __micro_trace_buffer_end);

    /*
     * Zero-fill trace buffer:
     */
    for (uint64_t *entry_p = __micro_trace_buffer;
         entry_p != __micro_trace_buffer_end; entry_p ++) {
        *entry_p = 0x0;
    }

    /*
     * Initialize MTB_POSITION register:
     * - POINTER field (bits 31:3) = encoding of __micro_trace_buffer
     * - WRAP bit = 0: No wrap has happened yet
     *
     * NOTE: POSITION register bits greater than or equal to 15 are RAZ/WI.
     */
    reg_value = 0;
    SET_BIT_FIELD(
        reg_value, MTB_POSITION_POINTER_MASK, MTB_POSITION_POINTER_SHIFT,
        (uintptr_t)__micro_trace_buffer >> MTB_POSITION_POINTER_SHIFT);
    write_32bit_mmio_register(&MTB_POSITION, reg_value);

    /*
     * Initialize MTB_FLOW register:
     * - WATERMARK field (bits 31:3) = __micro_trace_buffer_end
     * - AUTOHALT bit = 0
     * - AUTOSTOP bit = 0
     */
    reg_value = 0;
    SET_BIT_FIELD(
        reg_value, MTB_FLOW_WATERMARK_MASK, MTB_FLOW_WATERMARK_SHIFT,
        (uintptr_t)__micro_trace_buffer_end >> MTB_FLOW_WATERMARK_SHIFT);
    write_32bit_mmio_register(&MTB_FLOW, reg_value);

    /*
     * Initialize MTB_MASTER register:
     * - EN bit = 0: Enable micro tracing
     * - Mask field (bits 4:0) = mask to implicitly set the trace buffer size
     */
    reg_value = read_32bit_mmio_register(&MTB_MASTER);
    reg_value |= MTB_MASTER_EN_MASK;
    SET_BIT_FIELD(
        reg_value, MTB_MASTER_MASK_MASK, MTB_MASTER_MASK_SHIFT,
        MTB_MASTER_MASK_VALUE);
    write_32bit_mmio_register(&MTB_MASTER, reg_value);

    g_micro_trace_initialized = true;
}


void
micro_trace_stop(void)
{
    if (! g_micro_trace_initialized) {
        return;
    }

    /*
     * Disable micro tracing
     */
    uint32_t reg_value = read_32bit_mmio_register(&MTB_MASTER);
    reg_value &= ~MTB_MASTER_EN_MASK;
    write_32bit_mmio_register(&MTB_MASTER, reg_value);
}


void
micro_trace_restart(void)
{
    if (! g_micro_trace_initialized) {
        return;
    }

    /*
     * Re-enable micro tracing
     */
    uint32_t reg_value = read_32bit_mmio_register(&MTB_MASTER);
    reg_value |= MTB_MASTER_EN_MASK;
    write_32bit_mmio_register(&MTB_MASTER, reg_value);
}


void
micro_trace_get_cursor(uint64_t **mtb_cursor_pp, bool *mtb_cursor_wrapped_p)
{
    if (! g_micro_trace_initialized) {
        *mtb_cursor_pp = __micro_trace_buffer;
        *mtb_cursor_wrapped_p = false;
        return;
    }

    uint32_t reg_value = read_32bit_mmio_register(&MTB_POSITION);

    uintptr_t mtb_position_pointer_field =
        GET_BIT_FIELD(
            reg_value, MTB_POSITION_POINTER_MASK, MTB_POSITION_POINTER_SHIFT);

    if (reg_value & MTB_POSITION_WRAP_MASK) {
        *mtb_cursor_wrapped_p = true;
    } else {
        *mtb_cursor_wrapped_p = false;
    }

    /*
     * NOTE: POSITION register bits greater than or equal to 15 are RAZ/WI.
     */
    *mtb_cursor_pp = (uint64_t *)(
        (uintptr_t)__micro_trace_buffer |
        (mtb_position_pointer_field << MTB_POSITION_POINTER_SHIFT));
}


#ifdef _CPU_CYCLES_MEASURE_
static void
init_cpu_clock_cycles_counter(void)
{
    /*
     * Enable the clock to the PIT Module
     */
    uint32_t reg_value = read_32bit_mmio_register(&SIM_SCGC6);
    reg_value |= SIM_SCGC6_PIT_MASK;
    write_32bit_mmio_register(&SIM_SCGC6, reg_value);

    /*
     * Turn on PIT
     *
     * Bit MIDS = 0: Clock for standard PIT timers is enabled
     * Bit FRZ = 1: Timers are stopped in Debug mode.
     */
    write_32bit_mmio_register(
        &PIT_MCR,
        PIT_MCR_FRZ_MASK);

    /*
     * Configure the lifetimer timer, by chaining timer 1 to timer 0,
     * and by setting the LDVAL register of each timer to the maximum value.
     * Each timer is a down counter. Interrupts are left disabled for both
     * timers.
     */
    write_32bit_mmio_register(&PIT_LDVAL0, 0xFFFFFFFF);
    write_32bit_mmio_register(&PIT_LDVAL1, 0xFFFFFFFF);

    /*
     * Bit CHN = 1: Timer 1 is chained to previous timer (timer 0).
     * Bit TIE = 0: Interrupt requests from Timer 1 are disabled.
     * Bit TEN = 1: Timer 1 is enabled.
     */
    write_32bit_mmio_register(
        &PIT_TCTRL1,
        PIT_TCTRL_CHN_MASK | PIT_TCTRL_TEN_MASK);

    /*
     * Bit CHN = 0: Timer 0 is not chained.
     * Bit TIE = 0: Interrupt requests from Timer 0 are disabled.
     * Bit TEN = 1: Timer 0 is enabled.
     */
    write_32bit_mmio_register(
        &PIT_TCTRL0,
        PIT_TCTRL_TEN_MASK);
}


uint64_t
get_cpu_clock_cycles64(void)
{
    uint32_t low_reg_value = read_32bit_mmio_register(&PIT_LTMR64L);
    uint32_t high_reg_value = read_32bit_mmio_register(&PIT_LTMR64H);

    return ((uint64_t)high_reg_value << 32) + low_reg_value;
}


uint32_t
get_cpu_clock_cycles(void)
{
    uint32_t low_reg_value = read_32bit_mmio_register(&PIT_LTMR64L);

    /*
     * TODO: PIT module frequency is half of the CPU frequency, so may
     * need to multiply by 2 to get the accurate number CPU clock cycles.
     */
    return low_reg_value * 2;
}
#endif /* _CPU_CYCLES_MEASURE_ */

#if 0 // ???
/**
 * It configures a GPIO pin of the KL25 SoC
 */
void
configure_pin(const struct pin_config_info *pin_info_p, bool is_output)
{
    uint32_t reg_value;

    write_32bit_mmio_register(
        &PORT_PCR_REG(pin_info_p->pin_port_base_p, pin_info_p->pin_bit_index),
        pin_info_p->pin_pcr_value);

    if (is_output) {
        reg_value = read_32bit_mmio_register(
                        &GPIO_PDDR_REG(pin_info_p->pin_gpio_base_p));
        reg_value |= pin_info_p->pin_bit_mask;
        write_32bit_mmio_register(
            &GPIO_PDDR_REG(pin_info_p->pin_gpio_base_p), reg_value);
    }
    else
    {
        reg_value = read_32bit_mmio_register(
                        &GPIO_PDDR_REG(pin_info_p->pin_gpio_base_p));
        reg_value &= ~pin_info_p->pin_bit_mask;
        write_32bit_mmio_register(
            &GPIO_PDDR_REG(pin_info_p->pin_gpio_base_p), reg_value);
    }
}


void
activate_output_pin(const struct pin_config_info *pin_info_p)
{
#   ifdef DEBUG
    uint32_t reg_value = read_32bit_mmio_register(
                            &GPIO_PDDR_REG(pin_info_p->pin_gpio_base_p));

    FDC_ASSERT(
        reg_value & pin_info_p->pin_bit_mask,
        pin_info_p, reg_value);
#   endif

    if (pin_info_p->pin_is_active_high) {
        write_32bit_mmio_register(
            &GPIO_PSOR_REG(pin_info_p->pin_gpio_base_p),
            pin_info_p->pin_bit_mask);
    } else {
        write_32bit_mmio_register(
            &GPIO_PCOR_REG(pin_info_p->pin_gpio_base_p),
            pin_info_p->pin_bit_mask);
    }
}


void
deactivate_output_pin(const struct pin_config_info *pin_info_p)
{
#   ifdef DEBUG
    uint32_t reg_value = read_32bit_mmio_register(
                            &GPIO_PDDR_REG(pin_info_p->pin_gpio_base_p));

    FDC_ASSERT(
        reg_value & pin_info_p->pin_bit_mask,
        reg_value, pin_info_p->pin_bit_mask);
#   endif

    if (pin_info_p->pin_is_active_high) {
        write_32bit_mmio_register(
            &GPIO_PCOR_REG(pin_info_p->pin_gpio_base_p),
            pin_info_p->pin_bit_mask);
    } else {
        write_32bit_mmio_register(
            &GPIO_PSOR_REG(pin_info_p->pin_gpio_base_p),
            pin_info_p->pin_bit_mask);
    }
}


void
toggle_output_pin(const struct pin_config_info *pin_info_p)
{
#   ifdef DEBUG
    uint32_t reg_value = read_32bit_mmio_register(
                            &GPIO_PDDR_REG(pin_info_p->pin_gpio_base_p));

    FDC_ASSERT(
        reg_value & pin_info_p->pin_bit_mask,
        pin_info_p, reg_value);
#   endif

    write_32bit_mmio_register(
        &GPIO_PTOR_REG(pin_info_p->pin_gpio_base_p),
        pin_info_p->pin_bit_mask);
}


bool
read_input_pin(const struct pin_config_info *pin_info_p)
{
    uint32_t reg_value = read_32bit_mmio_register(
        &GPIO_PDIR_REG(pin_info_p->pin_gpio_base_p));

    return (reg_value & pin_info_p->pin_bit_mask) != 0;
}


void
toggle_heartbeat_led(void)
{
    toggle_rgb_led(LED_COLOR_BLUE);
}


static uint32_t g_before_debugger_led_color = LED_COLOR_BLACK;

void
turn_on_debugger_led(void)
{
    g_before_debugger_led_color = set_rgb_led_color(LED_COLOR_RED);
}


void
turn_off_debugger_led(void)
{
    set_rgb_led_color(g_before_debugger_led_color);
}


void
uart_init(
    const struct uart_device *uart_device_p,
    uint32_t baud_rate,
    uint8_t mode)
{
    uint32_t reg_value;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

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
#endif //???


/**
 * Send a character over a UART serial port, blocking the caller on a condvar
 * if the UART Tx fifo is full.
 */
void
uart_putchar(
    _IN_ const struct uart_device *uart_device_p,
    _IN_ uint8_t c)
{
#if 0
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
#else
    DBG_ASSERT(uart_device_p == NULL, uart_device_p, 0);
    ROM_UARTCharPut(UART0_BASE, c);
#endif
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
#if 0 // ???
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
#else
    DBG_ASSERT(uart_device_p == NULL, uart_device_p, 0);
    ROM_UARTCharPut(UART0_BASE, c);
#endif
}


/**
 * Receive a character from a UART serial port, blocking the caller
 * if there are no characters to read
 */
uint8_t
uart_getchar(
    _IN_ const struct uart_device *uart_device_p)
{
#if 0 // ???
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
#else
    DBG_ASSERT(uart_device_p == NULL, uart_device_p, 0);
    return ROM_UARTCharGet(UART0_BASE);
#endif
}

/**
 * Reads the next character received from a UART serial port, doing polling
 * until the character is received.
 */
uint8_t
uart_getchar_with_polling(
    _IN_ const struct uart_device *uart_device_p)
{
#if 0 // ???
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
#else
    DBG_ASSERT(uart_device_p == NULL, uart_device_p, 0);
    return ROM_UARTCharGet(UART0_BASE);
#endif
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


#if 0 // ???
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
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_i2c1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_I2C1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_spi1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SPI1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart1_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART1))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_uart2_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_UART2))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_cmp0_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_CMP0))
GENERATE_DUMMY_NVIC_ISR_FUNCTION(dummy_tpm2_isr, VECTOR_NUMBER_TO_IRQ_NUMBER(INT_TPM2))
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
#else
void
dummy_isr(void)
{
    /* This function should not have been invoked */
    FDC_ASSERT(false, 0, 0);
}
#endif

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

