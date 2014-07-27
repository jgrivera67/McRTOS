/**
 * @file lm4f120_soc_hardware_abstractions.c
 *
 * Hardware abstraction layer for the TI Stellaris LM24F120 SoC
 *
 * @author German Rivera
 */

#include "hardware_abstractions.h"
#include "lm4f120_soc.h"
#include "launchpad_board.h"
#include "McRTOS_arm_cortex_m.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"

#pragma GCC diagnostic ignored "-Wunused-parameter" //???
#pragma GCC diagnostic ignored "-Wunused-variable" //???

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
#define CONSOLE_SERIAL_PORT_MODE        0   /* 8-bits, no-parity, 1 stop bit */

#define UART_TRANSMIT_QUEUE_SIZE_IN_BYTES   UINT16_C(128)
#define UART_RECEIVE_QUEUE_SIZE_IN_BYTES    UINT16_C(16)

#define CALC_BAUD_RATE_DIVISOR_INT_PART(_cpu_clock_freq_hz, _baud_rate) \
	((_cpu_clock_freq_hz) / (16 * (_baud_rate)))

#define CALC_BAUD_RATE_DIVISOR_FRAC_PART(_cpu_clock_freq_hz, _baud_rate) \
	UINT_DIV_APPROX(						\
		((_cpu_clock_freq_hz) % (16 * (_baud_rate))) * 64,	\
		16 * (_baud_rate))

/**
 * Const fields of a UART device (to be placed in flash)
 */
struct uart_device {
#   define UART_DEVICE_SIGNATURE  GEN_SIGNATURE('U', 'A', 'R', 'T')
    uint32_t urt_signature;
    struct uart_device_var *urt_var_p;
    volatile struct uart *urt_mmio_uart_p;
    uint32_t urt_mmio_clock_gate_mask;
    volatile struct gpio_port *urt_gpio_port_p; /* for Tx and Rx pins */
    uint32_t urt_tx_pin_bit_index;
    uint32_t urt_rx_pin_bit_index;
    uint8_t urt_mux_function_selector;
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

static void uart_stop(
    const struct uart_device *uart_device_p);

/*
 * Function prototypes for dummy VIC interrupt ISRs
 */
static isr_function_t dummy_isr;

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
    [INT_SVCall] = cortex_m_svc_exception_handler,
    [INT_PendableSrvReq] = cortex_m_pendsv_exception_handler,
    [INT_SysTick] = cortex_m_systick_isr,

    /* Interrupts external to the Cortex-M core */
    [INT_GPIOA] = dummy_isr,          // GPIO Port A
    [INT_GPIOB] = dummy_isr,          // GPIO Port B
    [INT_GPIOC] = dummy_isr,          // GPIO Port C
    [INT_GPIOD] = dummy_isr,          // GPIO Port D
    [INT_GPIOE] = dummy_isr,          // GPIO Port E
    [INT_UART0] = lm4f120_uart0_isr,  // UART0
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
 * Global array of const structures for UART devices for the LM4F120 SoC
 * (allocated in flash space)
 */
static const struct uart_device g_uart_devices[] =
{
    [0] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
        .urt_var_p = &g_uart_devices_var[0],
        .urt_mmio_uart_p = (volatile struct uart *)UART0_BASE,
        .urt_mmio_clock_gate_mask = SYSCTL_RCGC1_UART0,
	.urt_gpio_port_p = (volatile struct gpio_port *)GPIO_PORTA_BASE,
	.urt_tx_pin_bit_index = LPAD_UART0_TX_PIN_INDEX,
	.urt_rx_pin_bit_index = LPAD_UART0_RX_PIN_INDEX,
	.urt_mux_function_selector = 0x1,
        .urt_rtos_interrupt_params = {
            .irp_name_p = "UART0 Interrupt",
            .irp_isr_function_p = lm4f120_uart0_isr,
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
        .urt_mmio_uart_p = (volatile struct uart *)UART1_BASE,
        .urt_mmio_clock_gate_mask = SYSCTL_RCGC1_UART1,
	.urt_gpio_port_p = (volatile struct gpio_port *)GPIO_PORTC_BASE,
	.urt_tx_pin_bit_index = LPAD_UART1_TX_PIN_INDEX,
	.urt_rx_pin_bit_index = LPAD_UART1_RX_PIN_INDEX,
	.urt_mux_function_selector = 0x2,
    },

    [2] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
        .urt_var_p = &g_uart_devices_var[2],
        .urt_mmio_uart_p = (volatile struct uart *)UART2_BASE,
        .urt_mmio_clock_gate_mask = SYSCTL_RCGC1_UART2,
    },
};

C_ASSERT(
    ARRAY_SIZE(g_uart_devices) == ARRAY_SIZE(g_uart_devices_var));

const struct uart_device *const g_console_serial_port_p = &g_uart_devices[0];

/**
 * Hardware Micro trace buffer (MTB)
 */
#if 0
uint64_t __attribute__ ((section(".mtb_buf")))
    g_micro_trace_buffer[MICRO_TRACE_BUFFER_NUM_ENTRIES];
#endif

/**
 *  Initializes SoC hardware.
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

    capture_fdc_msg_printf("Last reset cause: %#x\n", reset_cause);

    system_clocks_init();

#   ifdef _CPU_CYCLES_MEASURE_
    init_cpu_clock_cycles_counter();
#   endif

    bool mpu_present = cortex_m_mpu_init();

    capture_fdc_msg_printf("MPU %s present\n", mpu_present ? "" : "not");

    cortex_m_nvic_init();

    uart_init(
        g_console_serial_port_p,
        CONSOLE_SERIAL_PORT_BAUD_RATE,
        CONSOLE_SERIAL_PORT_MODE);

#if 0
    i2c_init(g_i2c0_device_p);
#endif
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
    return GRC_INVALID_RESET_CAUSE; // TODO
}


bool
software_reset_happened(void)
{
    return false; // TODO
}


/**
 * Configure the system to get its clock from the PLL
 */
static void
pll_init(void)
{
    /*
     * TODO: re-write this function
     */
#   define SYSDIV2 4
    // bus frequency is 400MHz/(SYSDIV2+1) = 400MHz/(4+1) = 80 MHz

  // 0) configure the system to use RCC2 for advanced features
  //    such as 400 MHz PLL and non-integer System Clock Divisor
  SYSCTL_RCC2_R |= SYSCTL_RCC2_USERCC2;
  // 1) bypass PLL while initializing
  SYSCTL_RCC2_R |= SYSCTL_RCC2_BYPASS2;
  // 2) select the crystal value and oscillator source
  SYSCTL_RCC_R &= ~SYSCTL_RCC_XTAL_M;   // clear XTAL field
  SYSCTL_RCC_R += SYSCTL_RCC_XTAL_16MHZ;// configure for 16 MHz crystal
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_OSCSRC2_M;// clear oscillator source field
  SYSCTL_RCC2_R += SYSCTL_RCC2_OSCSRC2_MO;// configure for main oscillator source
  // 3) activate PLL by clearing PWRDN
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_PWRDN2;
  // 4) set the desired system divider and the system divider least significant bit
  SYSCTL_RCC2_R |= SYSCTL_RCC2_DIV400;  // use 400 MHz PLL
  SYSCTL_RCC2_R = (SYSCTL_RCC2_R&~0x1FC00000) // clear system clock divider field
                  + (SYSDIV2<<22);      // configure for 80 MHz clock
  // 5) wait for the PLL to lock by polling PLLLRIS
  while((SYSCTL_RIS_R&SYSCTL_RIS_PLLLRIS)==0){};
  // 6) enable use of PLL by clearing BYPASS
  SYSCTL_RCC2_R &= ~SYSCTL_RCC2_BYPASS2;
}

static void
system_clocks_init(void)
{
    uint32_t reg_value;

    pll_init();

    /*
     * Enable all of the GPIO port clocks. These have to be enabled to configure
     * pin muxing options, so most code will need all of these on anyway.
     */
    reg_value = read_32bit_mmio_register(&SYSCTL_RCGC2_R);
    reg_value |= (SYSCTL_RCGC2_GPIOA |
		  SYSCTL_RCGC2_GPIOB |
		  SYSCTL_RCGC2_GPIOC |
		  SYSCTL_RCGC2_GPIOD |
		  SYSCTL_RCGC2_GPIOE |
		  SYSCTL_RCGC2_GPIOF);
    write_32bit_mmio_register(&SYSCTL_RCGC2_R, reg_value);
}

static bool g_micro_trace_initialized = false;

void
micro_trace_init(void)
{
}


void
micro_trace_stop(void)
{
    if (! g_micro_trace_initialized) {
        return;
    }
}


void
micro_trace_restart(void)
{
    if (! g_micro_trace_initialized) {
        return;
    }
}


void
micro_trace_get_cursor(uint64_t **mtb_cursor_pp, bool *mtb_cursor_wrapped_p)
{
    if (! g_micro_trace_initialized) {
        *mtb_cursor_pp = __micro_trace_buffer;
        *mtb_cursor_wrapped_p = false;
        return;
    }
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

/**
 * It configures a GPIO pin of the LM4F120 SoC
 */
void
configure_pin(const struct pin_config_info *pin_info_p, bool is_output)
{
    uint32_t reg_value;
    volatile struct gpio_port *pin_gpio_port_p = pin_info_p->pin_gpio_port_p;

#   ifdef DEBUG
    FDC_ASSERT(
        pin_info_p->pin_bit_mask <= 0xff,
        pin_info_p->pin_bit_mask, pin_info_p);
#   endif

    if (pin_info_p->pin_is_locked) {
	/*
	 * Unlock pin:
	 */
        write_32bit_mmio_register(
	    &pin_gpio_port_p->reg_LOCK, GPIO_LOCK_KEY);

        reg_value = read_32bit_mmio_register(
			    &pin_gpio_port_p->reg_CR);
        reg_value |= pin_info_p->pin_bit_mask;
        write_32bit_mmio_register(
	    &pin_gpio_port_p->reg_CR, reg_value);
    }

    /*
     * Set pin direction:
     */
    reg_value = read_32bit_mmio_register(&pin_gpio_port_p->reg_DIR);
    if (is_output) {
        reg_value |= pin_info_p->pin_bit_mask;
    } else {
        reg_value &= ~pin_info_p->pin_bit_mask;
    }
    write_32bit_mmio_register(&pin_gpio_port_p->reg_DIR, reg_value);

    /*
     * Disable alternate function for pin:
     */
    reg_value = read_32bit_mmio_register(&pin_gpio_port_p->reg_AFSEL);
    reg_value &= ~pin_info_p->pin_bit_mask;
    write_32bit_mmio_register(&pin_gpio_port_p->reg_AFSEL, reg_value);
    ///???
    reg_value = read_32bit_mmio_register(&pin_gpio_port_p->reg_PCTL);
    reg_value &= ~pin_info_p->pin_bit_mask;
    write_32bit_mmio_register(&pin_gpio_port_p->reg_PCTL, reg_value);
    ///???

    switch (pin_info_p->pin_pull_mode) {
    case PIN_PULL_DOWN:
        reg_value = read_32bit_mmio_register(&pin_gpio_port_p->reg_PDR);
        reg_value |= pin_info_p->pin_bit_mask;
        write_32bit_mmio_register(&pin_gpio_port_p->reg_PDR, reg_value);
	break;

    case PIN_PULL_UP:
	reg_value = read_32bit_mmio_register(&pin_gpio_port_p->reg_PUR);
	reg_value |= pin_info_p->pin_bit_mask;
	write_32bit_mmio_register(&pin_gpio_port_p->reg_PUR, reg_value);
	break;

    case PIN_OPEN_DRAIN:
	reg_value = read_32bit_mmio_register(&pin_gpio_port_p->reg_ODR);
	reg_value |= pin_info_p->pin_bit_mask;
	write_32bit_mmio_register(&pin_gpio_port_p->reg_ODR, reg_value);
	break;

    default:
	FDC_ASSERT(pin_info_p->pin_pull_mode == PIN_PULL_NONE,
		   pin_info_p->pin_pull_mode, pin_info_p);
    }

    /*
     * Set initial value of pin to "deasserted" if output pin:
     */
    if (is_output) {
	volatile uint32_t *data_reg_addr =
		&pin_gpio_port_p->reg_DATA_bits[pin_info_p->pin_bit_mask];

        reg_value = read_32bit_mmio_register(data_reg_addr);
	if (pin_info_p->pin_is_active_high) {
	    reg_value &= ~pin_info_p->pin_bit_mask;
        } else {
	    reg_value |= pin_info_p->pin_bit_mask;
	}
	write_32bit_mmio_register(data_reg_addr, reg_value);
    }

    /*
     * Disable analog mode for pin:
     */
    reg_value = read_32bit_mmio_register(&pin_gpio_port_p->reg_AMSEL);
    reg_value &= ~pin_info_p->pin_bit_mask;
    write_32bit_mmio_register(&pin_gpio_port_p->reg_AMSEL, reg_value);

#if 0
    /*
     * Set pin strength:
     */
    reg_value = read_32bit_mmio_register(&pin_gpio_port_p->reg_DR8R);
    reg_value &= ~pin_info_p->pin_bit_mask;
    write_32bit_mmio_register(&pin_gpio_port_p->reg_DR8R, reg_value);
    reg_value = read_32bit_mmio_register(&pin_gpio_port_p->reg_SLR);
    reg_value &= ~pin_info_p->pin_bit_mask;
    write_32bit_mmio_register(&pin_gpio_port_p->reg_SLR, reg_value);
#endif

    /*
     * Enable pin as a digital input or output signal:
     */
    reg_value = read_32bit_mmio_register(&pin_gpio_port_p->reg_DEN);
    reg_value |= pin_info_p->pin_bit_mask;
    write_32bit_mmio_register(&pin_gpio_port_p->reg_DEN, reg_value);
}


void
activate_output_pin(const struct pin_config_info *pin_info_p)
{
    uint32_t reg_value;
    volatile struct gpio_port *pin_gpio_port_p = pin_info_p->pin_gpio_port_p;

#   ifdef DEBUG
    reg_value = read_32bit_mmio_register(
                            &pin_gpio_port_p->reg_DIR);

    FDC_ASSERT(
        reg_value & pin_info_p->pin_bit_mask,
        pin_info_p, reg_value);

    FDC_ASSERT(
        pin_info_p->pin_bit_mask <= 0xff,
        pin_info_p->pin_bit_mask, pin_info_p);
#   endif

    if (pin_info_p->pin_is_active_high) {
        reg_value = pin_info_p->pin_bit_mask;
    } else {
        reg_value = ~pin_info_p->pin_bit_mask;
    }

    write_32bit_mmio_register(
	&pin_gpio_port_p->reg_DATA_bits[pin_info_p->pin_bit_mask],
	reg_value);
}


void
deactivate_output_pin(const struct pin_config_info *pin_info_p)
{
    uint32_t reg_value;
    volatile struct gpio_port *pin_gpio_port_p = pin_info_p->pin_gpio_port_p;

#   ifdef DEBUG
    reg_value = read_32bit_mmio_register(
                            &pin_gpio_port_p->reg_DIR);

    FDC_ASSERT(
        reg_value & pin_info_p->pin_bit_mask,
        reg_value, pin_info_p->pin_bit_mask);

    FDC_ASSERT(
        pin_info_p->pin_bit_mask <= 0xff,
        pin_info_p->pin_bit_mask, pin_info_p);
#   endif

    if (pin_info_p->pin_is_active_high) {
        reg_value = ~pin_info_p->pin_bit_mask;
    } else {
        reg_value = pin_info_p->pin_bit_mask;
    }

    write_32bit_mmio_register(
	&pin_gpio_port_p->reg_DATA_bits[pin_info_p->pin_bit_mask],
	reg_value);
}


void
toggle_output_pin(const struct pin_config_info *pin_info_p)
{
    uint32_t reg_value;
    volatile struct gpio_port *pin_gpio_port_p = pin_info_p->pin_gpio_port_p;

#   ifdef DEBUG
    reg_value = read_32bit_mmio_register(
                            &pin_gpio_port_p->reg_DIR);

    FDC_ASSERT(
        reg_value & pin_info_p->pin_bit_mask,
        pin_info_p, reg_value);

    FDC_ASSERT(
        pin_info_p->pin_bit_mask <= 0xff,
        pin_info_p->pin_bit_mask, pin_info_p);
#   endif

    volatile uint32_t *data_reg_addr =
	&pin_gpio_port_p->reg_DATA_bits[pin_info_p->pin_bit_mask];

    reg_value = read_32bit_mmio_register(data_reg_addr);
    reg_value ^= pin_info_p->pin_bit_mask;
    write_32bit_mmio_register(data_reg_addr, reg_value);
}


bool
read_input_pin(const struct pin_config_info *pin_info_p)
{
    uint32_t reg_value;
    volatile struct gpio_port *pin_gpio_port_p = pin_info_p->pin_gpio_port_p;

#   ifdef DEBUG
    reg_value = read_32bit_mmio_register(
                            &pin_gpio_port_p->reg_DIR);

    FDC_ASSERT(
        (reg_value & pin_info_p->pin_bit_mask) == 0,
        pin_info_p, reg_value);

    FDC_ASSERT(
        pin_info_p->pin_bit_mask <= 0xff,
        pin_info_p->pin_bit_mask, pin_info_p);
#   endif

    reg_value = read_32bit_mmio_register(
			&pin_gpio_port_p->reg_DATA_bits[pin_info_p->pin_bit_mask]);

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
    volatile struct uart *uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;
    volatile struct gpio_port *uart_gpio_port_p = uart_device_p->urt_gpio_port_p;

    FDC_ASSERT(!uart_var_p->urt_initialized, uart_device_p, 0);
    FDC_ASSERT(mode == 0, mode, uart_device_p);
    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();

    /*
     * Enable clock for the UART:
     */
    reg_value = read_32bit_mmio_register(&SYSCTL_RCGC1_R);
    reg_value |= uart_device_p->urt_mmio_clock_gate_mask;
    write_32bit_mmio_register(&SYSCTL_RCGC1_R, reg_value);

    /*
     * Disable UART, while it is being configured:
     */
    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_CTL);
    reg_value &= ~UART_CTL_UARTEN;
    write_32bit_mmio_register(&uart_mmio_registers_p->reg_CTL, reg_value);

    /*
     * Set baud rate divisor integer part:
     */
    reg_value = CALC_BAUD_RATE_DIVISOR_INT_PART(
		    CPU_CLOCK_FREQ_IN_HZ, baud_rate);
    write_32bit_mmio_register(&uart_mmio_registers_p->reg_IBRD, reg_value);

    /*
     * Set baud rate divisor fractional part:
     */
    reg_value = CALC_BAUD_RATE_DIVISOR_FRAC_PART(
		    CPU_CLOCK_FREQ_IN_HZ, baud_rate);
    write_32bit_mmio_register(&uart_mmio_registers_p->reg_FBRD, reg_value);

    /*
     * Configure the uart for 8-bit mode, no parity, 1 stop bit, FIFOs:
     */
    write_32bit_mmio_register(
	&uart_mmio_registers_p->reg_LCRH,
	UART_LCRH_WLEN_8 | UART_LCRH_FEN);

    /*
     * Configure Tx and Rx pins:
     * - Enable alternate function for corresponding GPIO port pins
     * - Select UART functions for pins
     * - Disable analog functionality for pins
     * - Enable digital I/O for pins
     */

    uint32_t tx_rx_pins_bit_mask =
	(BIT(uart_device_p->urt_tx_pin_bit_index) |
	 BIT(uart_device_p->urt_rx_pin_bit_index));

    reg_value = read_32bit_mmio_register(&uart_gpio_port_p->reg_AFSEL);
    reg_value |= tx_rx_pins_bit_mask;
    write_32bit_mmio_register(&uart_gpio_port_p->reg_AFSEL, reg_value);

    reg_value = read_32bit_mmio_register(&uart_gpio_port_p->reg_PCTL);
    SET_BIT_FIELD(
        reg_value,
        GPIO_PCTL_PIN_MUX_MASK(uart_device_p->urt_tx_pin_bit_index),
        GPIO_PCTL_PIN_MUX_SHIFT(uart_device_p->urt_tx_pin_bit_index),
        uart_device_p->urt_mux_function_selector);
    SET_BIT_FIELD(
        reg_value,
        GPIO_PCTL_PIN_MUX_MASK(uart_device_p->urt_rx_pin_bit_index),
        GPIO_PCTL_PIN_MUX_SHIFT(uart_device_p->urt_rx_pin_bit_index),
        uart_device_p->urt_mux_function_selector);
    write_32bit_mmio_register(&uart_gpio_port_p->reg_PCTL, reg_value);

    reg_value = read_32bit_mmio_register(&uart_gpio_port_p->reg_AMSEL);
    reg_value &= ~tx_rx_pins_bit_mask;
    write_32bit_mmio_register(&uart_gpio_port_p->reg_AMSEL, reg_value);

    reg_value = read_32bit_mmio_register(&uart_gpio_port_p->reg_DEN);
    reg_value |= tx_rx_pins_bit_mask;
    write_32bit_mmio_register(&uart_gpio_port_p->reg_DEN, reg_value);

    /*
     * Enable UART:
     */
    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_CTL);
    reg_value |= UART_CTL_UARTEN;
    write_32bit_mmio_register(&uart_mmio_registers_p->reg_CTL, reg_value);

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
    volatile struct uart *uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    /*
     * Disable UART:
     */
    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_CTL);
    reg_value &= ~UART_CTL_UARTEN;
    write_32bit_mmio_register(&uart_mmio_registers_p->reg_CTL, reg_value);

    /*
     * Disable clock for the UART:
     */
    reg_value = read_32bit_mmio_register(&SYSCTL_RCGC1_R);
    reg_value &= ~uart_device_p->urt_mmio_clock_gate_mask;
    write_32bit_mmio_register(&SYSCTL_RCGC1_R, reg_value);

    uart_var_p->urt_initialized = false;
}

/**
 * UART common interrupt handler with interrupts enabled
 */
void
lm4f120_uart_interrupt_e_handler(
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
    volatile struct uart *uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    DBG_ASSERT(
        uart_mmio_registers_p == (volatile struct uart *)UART0_BASE ||
        uart_mmio_registers_p == (volatile struct uart *)UART1_BASE ||
        uart_mmio_registers_p == (volatile struct uart *)UART2_BASE,
        uart_mmio_registers_p, uart_device_p);

    DBG_ASSERT(
        uart_var_p->urt_initialized, uart_device_p, 0);

    uint32_t reg_mis_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_MIS);
    uint32_t reg_fr_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_FR);

    /*
     * Only one or more of the following UART interrupts is expected:
     */
    FDC_ASSERT(
        (reg_mis_value &
	 ~(UART_MIS_TXMIS | UART_MIS_RXMIS | UART_MIS_RTMIS)) == 0,
        reg_mis_value, uart_device_p);

    if ((reg_mis_value & UART_MIS_TXMIS) != 0) {
	/*
	 * "Tx FIFO not full" interrupt:
	 */
	unsigned int bytes_transmitted_count = 0;

	FDC_ASSERT((reg_fr_value & UART_FR_TXFF) == 0,
		   reg_fr_value, uart_device_p);

	/*
	 * Clear (acknowledge) the interrupt source:
	 */
	write_32bit_mmio_register(&uart_mmio_registers_p->reg_ICR, UART_ICR_TXIC);

	/*
	 * Fill the Tx FIFO as much as possible:
	 */
	do {
	    uint8_t byte_to_transmit;

	    bool entry_read = rtos_k_byte_circular_buffer_read(
				&uart_var_p->urt_transmit_queue,
				&byte_to_transmit,
				false);

	    if (!entry_read) {
		break;
	    }

	    /*
	     * Write the next byte to transmit, to clear the interrupt source:
	     */
	    write_32bit_mmio_register(&uart_mmio_registers_p->reg_DR,
				      byte_to_transmit);
	    bytes_transmitted_count++;
	    reg_fr_value = read_32bit_mmio_register(
				&uart_mmio_registers_p->reg_FR);
	} while ((reg_fr_value & UART_FR_TXFF) == 0);

	DBG_ASSERT(
	    bytes_transmitted_count <= UART_TX_FIFO_SIZE,
	    bytes_transmitted_count, uart_device_p);

	if (bytes_transmitted_count == 0) {
	    /*
	     * If nothing was transmitted, disable "Tx FIFO not full" interrupt:
	     */
	    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_IM);
	    reg_value &= ~UART_IM_TXIM;
	    write_32bit_mmio_register(&uart_mmio_registers_p->reg_IM, reg_value);
	}
    }

    if ((reg_mis_value & (UART_MIS_RXMIS | UART_MIS_RTMIS)) != 0) {
	/*
	 * "Rx FIFO not empty" interrupts:
	 */
	unsigned int bytes_received_count = 0;

	FDC_ASSERT((reg_fr_value & UART_FR_RXFE) == 0,
		   reg_fr_value, uart_device_p);

	/*
	 * Clear (acknowledge) the interrupt source:
	 */
	write_32bit_mmio_register(&uart_mmio_registers_p->reg_ICR,
				  UART_ICR_RXIC | UART_ICR_RTIC);

	/*
	 * Drain the Rx FIFO as much as possible:
	 */
	do {
	    /*
	     * Read the next byte received, to clear the interrupt source:
	     */
	    uint8_t byte_received = read_32bit_mmio_register(
					&uart_mmio_registers_p->reg_DR);

	    bool entry_written = rtos_k_byte_circular_buffer_write(
				    &uart_var_p->urt_receive_queue,
				    byte_received,
				    false);

	    if (!entry_written) {
		uart_var_p->urt_received_bytes_dropped ++;
		break;
	    }

	    bytes_received_count++;
	    reg_fr_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_FR);
	} while ((reg_fr_value & UART_FR_RXFE) == 0);

	DBG_ASSERT(
	    bytes_received_count <= UART_TX_FIFO_SIZE,
	    bytes_received_count, uart_device_p);

	if (bytes_received_count == 0) {
	    /*
	     * If the software receive queue is full, disable the
	     * "Rx FIFO not empty" interrupts:
	     *
	     * NOTE: We need to do this as the interrupt is still asserted.
	     * These interrupts will be re-enabled when there is room
	     * in the receive queue.
	     */
	    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_IM);
	    reg_value &= ~UART_IM_RXIM;
	    write_32bit_mmio_register(&uart_mmio_registers_p->reg_IM, reg_value);
	}
    }
}

/**
 * Enqueue a character for transmission over a UART serial port, blocking the
 * caller on a condvar if the transmit queue is full, if the caller is a thread.
 */
void
uart_putchar(
    _IN_ const struct uart_device *uart_device_p,
    _IN_ uint8_t c)
{
    uint32_t reg_value;
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;
    volatile struct uart *uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

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
     * Disable generation of "Tx FIFO not full" interrupts, if not disabled,
     * to avoid a race with the UART ISR:
     */
    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_IM);
    if ((reg_value & UART_IM_TXIM) == 0) {
	reg_value &= ~UART_IM_TXIM;
	write_32bit_mmio_register(&uart_mmio_registers_p->reg_IM, reg_value);
    }

    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_FR);
    if ((reg_value & UART_FR_TXFF) == 0) {
	/*
	 * Tx FIFO is not full, so transmit the first character from
	 * uart_var_p->urt_transmit_queue:
	 */
	uint8_t first_c;
	bool entry_read;

	entry_read = rtos_k_byte_circular_buffer_read(
			&uart_var_p->urt_transmit_queue, &first_c, false);
	if (entry_read) {
	    write_32bit_mmio_register(&uart_mmio_registers_p->reg_DR, first_c);
	}
    }

    /*
     * Enable generation of "Tx FIFO not full" interrupts:
     */
    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_IM);
    reg_value |= UART_IM_TXIM;
    write_32bit_mmio_register(&uart_mmio_registers_p->reg_IM, reg_value);
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
    volatile struct uart *uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;
    uint32_t reg_value;

    if (!uart_var_p->urt_initialized) {
        return;
    }

    /*
     * Do polling until the UART's transmit register becomes empty:
     */
    do {
        reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_FR);
    } while ((reg_value & UART_FR_TXFF) != 0);

    write_32bit_mmio_register(&uart_mmio_registers_p->reg_DR, c);
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
    volatile struct uart *uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;
    uint32_t reg_value;
    uint8_t char_received;

    DBG_ASSERT(
        uart_var_p->urt_initialized, uart_var_p, uart_device_p);

    DBG_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

    bool entry_read = rtos_k_byte_circular_buffer_read(
			&uart_var_p->urt_receive_queue,
			&char_received,
			false);

    if (!entry_read) {
	/*
	 * Enable generation of receive interrupts, if necessary:
	 */
	reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_IM);
	if ((reg_value & UART_IM_RXIM) == 0) {
	    reg_value |= UART_IM_RXIM | UART_IM_RTIM;
	    write_32bit_mmio_register(&uart_mmio_registers_p->reg_IM, reg_value);
	}

	entry_read = rtos_k_byte_circular_buffer_read(
			&uart_var_p->urt_receive_queue,
			&char_received,
			true);

	DBG_ASSERT(entry_read, uart_device_p, 0);
    }

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
    volatile struct uart *uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    FDC_ASSERT(
        uart_var_p->urt_initialized, uart_var_p, uart_device_p);

    /*
     * Do polling until the UART's Rx FIFO becomes not empty:
     */
    do {
        reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_FR);
    } while ((reg_value & UART_FR_RXFE) != 0);

    uint8_t byte_received = read_32bit_mmio_register(&uart_mmio_registers_p->reg_DR);
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

