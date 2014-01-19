/**
 * @file lpc2478_hardware_abstractions.c
 *
 * Hardware abstraction layer for the lpc2478-stk board
 *
 * @author German Rivera 
 */ 

#include "hardware_abstractions.h"
#include "lpc2478_stk_board.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-variable"

/*
 * Number of times the timer counter register (reg_TC) is incremented before
 * the timer generates an interrupt
 * Since we want MR0 to be the smallest non-zero value (1) and the value for MR0 is
 * given by TARGET_NUM_TC_INCREMENTS - 1, TARGET_NUM_TC_INCREMENTS must be 2.
 */
#define TARGET_NUM_TC_INCREMENTS    UINT32_C(2)

/**
 * CPU clock frequency in Hz
 */
#define CPU_CLOCK_FREQ_IN_HZ \
        (SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ * UINT32_C(1000000))

/**
 * CPU clock cycle counter timer frequency in Hz
 * (CPU clock frequency divided by CPU_CLOCK_FREQ_DIVIDER)
 */
#define CPU_CLOCK_CYCLE_COUNTER_TIMER_FREQUENCY \
        (CPU_CLOCK_FREQ_IN_HZ / CPU_CLOCK_FREQ_DIVIDER)

#define CPU_CLOCK_FREQ_DIVIDER  1

/**
 * A/D converter clock frequency in Hz
 */
#define ADC_CLOCK_FREQUENCY (UINT32_C(2) * 1000 * 1000)   /* 2 MHz */

C_ASSERT(ADC_CLOCK_FREQUENCY <= MAX_ADC_CLOCK_FREQUENCY);

/**
 * SSP clock frequency in Hz
 */
#define SSP_CLOCK_FREQUENCY (UINT32_C(2) * 1000 * 1000)   /* 2 MHz */

C_ASSERT(SSP_CLOCK_FREQUENCY <= MAX_SSP_CLOCK_FREQUENCY);

/**
 * Macro to generate dummy VIC ISR functions
 */
#define GENERATE_DUMMY_VIC_ISR_FUNCTION(_peripheral_name,                   \
                                        _vic_interrupt_channel)             \
        __attribute__((interrupt ("IRQ")))                                  \
        void                                                                \
        dummy_ ## _peripheral_name ## _isr(void)                            \
        {                                                                   \
            /* This function should not have been invoked */                \
            FDC_ASSERT(false, _vic_interrupt_channel, 0);                   \
            FDC_ASSERT_INTERRUPT_SOURCE_IS_SET(_vic_interrupt_channel);     \
                                                                            \
            /* Tell VIC that interrupt has been serviced */                 \
            g_vic_mmio_registers_p->reg_VICAddress = 0x0;                   \
        }

/**
 * Serial communication parameters for the serial port used as the console
 * port
 */
#define DEBUG_SERIAL_PORT_BAUD_RATE   38400
#define DEBUG_SERIAL_PORT_MODE        UART_8N1

struct scbParams {
    uint16_t PLL_M_Mul; /* PLL Multiplier. Valid values 6 through 512*/
    uint8_t  PLL_N_Div; /* PLL Divider. Valid values 1 through 32 */
    uint32_t PLL_Fcco;  /* Frequency (Hz) of PLL output */
    uint8_t  CCLK_Div;  /* CPU Clock divider, cclk */
    uint8_t  MAMMode;   /* */
    uint8_t  MAMTim;
};

struct timer_device {
#   define TIMER_DEVICE_SIGNATURE  GEN_SIGNATURE('T', 'I', 'M', 'R')
    uint32_t tmr_signature;
    lpc2478_timer_t *tmr_mmio_registers_p;
    uint32_t tmr_timer_pclk_shift;
    uint32_t tmr_pclksel_index;
    uint32_t tmr_pconp_mask;
    struct rtos_interrupt_registration_params tmr_rtos_interrupt_params;
    struct rtos_interrupt **tmr_rtos_interrupt_pp;
};

/**
 * Const fields of a UART device (to be placed in flash)
 */
struct uart_device {
#   define UART_DEVICE_SIGNATURE  GEN_SIGNATURE('U', 'A', 'R', 'T')
    uint32_t urt_signature;
    struct uart_device_var *urt_var_p;
    lpc2478_uart_t *urt_mmio_uart_p;
    uint32_t urt_uart_pclk;
    uint32_t urt_pclksel_index;
    uint32_t urt_pconp_mask;
    struct rtos_interrupt_registration_params urt_rtos_interrupt_params;
    struct rtos_interrupt **urt_rtos_interrupt_pp;
    const char *urt_thre_condvar_name_p;
    const char *urt_rdr_condvar_name_p;
};

/**
 * Non-const fields of a UART device (to be placed in SRAM)
 */
struct uart_device_var {
    bool urt_initialized;
    bool urt_signal_thre_condvar;
    bool urt_signal_rdr_condvar;
    volatile bool urt_byte_received_pending;
    uint8_t urt_first_byte_received;
    struct rtos_condvar urt_thre_condvar;
    struct rtos_condvar urt_rdr_condvar;
};

/**
 * Const fields of the buttons device (to be placed in flash)
 */
struct buttons_device {
#   define BUTTONS_DEVICE_SIGNATURE  GEN_SIGNATURE('B', 'U', 'T', 'T')
    uint32_t bd_signature;
    struct buttons_device_var *bd_var_p;
    uint8_t bd_gpio_port_index;
    volatile struct gpio_interrupt_registers *bd_gpio_mmio_interrupt_registers_p;
    volatile struct gpio_port_interrupt_registers *bd_gpio_port_mmio_interrupt_registers_p;
    struct pin_config_info bd_button1_pin;
    struct pin_config_info bd_button2_pin;
    struct rtos_interrupt_registration_params bd_rtos_interrupt_params;
    struct rtos_interrupt **bd_rtos_interrupt_pp;
    const char *bd_condvar_name_p;
};

/**
 * Non-const fields of the buttons device (to be placed in SRAM)
 */
struct buttons_device_var {
    volatile uint32_t bd_buttons_pressed_bitmap;
    struct rtos_condvar bd_condvar;
};

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

/**
 * Const fields of an SSP controller (to be placed in flash)
 */
struct ssp_controller {
#   define   SSP_CONTROLLER_SIGNATURE  GEN_SIGNATURE('S', 'S', 'P', 'C')
    uint32_t sc_signature;
    struct ssp_controller_var *sc_var_p;
    lpc2478_ssp_t *sc_mmio_registers_p;
    vic_interrupt_channel_t sc_vic_channel;
    uint32_t sc_ssp_pclk_shift;
    uint32_t sc_pclksel_index;
    uint32_t sc_pconp_mask;
    struct pin_config_info sc_pin_sck;
    struct pin_config_info sc_pin_ssel;
    struct pin_config_info sc_pin_miso;
    struct pin_config_info sc_pin_mosi;
    struct rtos_interrupt_registration_params sc_rtos_interrupt_params;
    struct rtos_interrupt **sc_rtos_interrupt_pp;
    const char *sc_condvar_name_p;
};

/**
 * Non-const fields of an SSP controller (to be placed in SRAM)
 */
struct ssp_controller_var
{
    struct rtos_condvar sc_condvar;
    volatile bool sc_interrupt_pending;
};


/* local functions */
static uint32_t getFcclk(void);

static void initPLL(const struct scbParams *pSCBParams);
static void initPCLK(void);
static void initGPIO(void);
static void initMAM(uint32_t cclk, uint8_t mamcr, uint8_t mamtim_override);
static void initVIC(void);
static void initSDRAM(uint32_t cclk);
static void disableSDRAM(void);

static void init_uart_internal(
    const struct uart_device *uart_device_p,
    uint32_t baud,
    uint8_t mode,
    uint8_t fmode,
    uint32_t cclk);

static void init_timer(
    const struct timer_device *timer_device_p,
    uint32_t timer_frequency_hz,
    bool generate_interrupts);

static void
init_leds(void);

static void
init_adc(const struct adc_device *adc_device_p);

#if 0
static void
init_spi(bool master);
#endif


/* 
 * Added function prototypes for dummy VIC interrupt ISRs
 */
static isr_function_t dummy_wdt_isr;
static isr_function_t dummy_softint_isr;
static isr_function_t dummy_armcore0_isr;
static isr_function_t dummy_armcore1_isr;
static isr_function_t dummy_timer0_isr;
static isr_function_t dummy_timer1_isr;
static isr_function_t dummy_uart0_isr;
static isr_function_t dummy_uart1_isr;
static isr_function_t dummy_pwm0_1_isr;
static isr_function_t dummy_i2c0_isr;
static isr_function_t dummy_spi_ssp0_isr;
static isr_function_t dummy_ssp1_isr;
static isr_function_t dummy_pll_isr;
static isr_function_t dummy_rtc_isr;
static isr_function_t dummy_eint0_isr;
static isr_function_t dummy_eint1_isr;
static isr_function_t dummy_eint2_isr;
static isr_function_t dummy_eint3_isr;
static isr_function_t dummy_ad0_isr;
static isr_function_t dummy_i2c1_isr;
static isr_function_t dummy_bod_isr;
static isr_function_t dummy_ethernet_isr;
static isr_function_t dummy_usb_isr;
static isr_function_t dummy_can_isr;
static isr_function_t dummy_sd_mmc_isr;
static isr_function_t dummy_gpdma_isr;
static isr_function_t dummy_timer2_isr;
static isr_function_t dummy_timer3_isr;
static isr_function_t dummy_uart2_isr;
static isr_function_t dummy_uart3_isr;
static isr_function_t dummy_i2c2_isr;
static isr_function_t dummy_i2s_isr;

/*
 * ISR assembly routines defined in lpc2478_interrupt_handlers.s
 */
extern isr_function_t isr_timer0;
extern isr_function_t isr_gpio_ports;
extern isr_function_t isr_uart0;
extern isr_function_t isr_ssp_controller0;
extern isr_function_t isr_ssp_controller1;
extern isr_function_t isr_adc;

/**
 * Global pointer to the SDRAM map
 */ 
struct sdram_map *const g_sdram_map_p = (struct sdram_map *)BOARD_SDRAM_BASE;

/**
 * Global pointer to the beginning of unused SDRAM
 */ 
void *const g_available_sdram_p = (void *)AVAILABLE_SDRAM_START_ADDR;

/*
 * Added definition of global pointers that point to memory-mapped
 * I/O registers
 */

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
 * Global definition of const pointer to the memory-mapped registers of the
 * LPC2478 EMC (External Memory Controller)
 */
lpc2478_emc_t *const g_emc_mmio_registers_p = (lpc2478_emc_t *)LPC2478_EMC_BASE_ADDR;

/**
 * Global definition of const pointer to the memory-mapped registers of the
 * LPC2478 VIC (Vectored Interrupt Controller)
 */
static lpc2478_vic_t *const g_vic_mmio_registers_p = (lpc2478_vic_t *)LPC2478_VIC_BASE_ADDR;

/**
 * Pointer to the McRTOS interrupt object for the tick timer interrupt of CPU
 * core 0
 *
 * NOTE: This variable is accessed from lpc2478_interrupt_handlers.s
 */
struct rtos_interrupt *g_rtos_interrupt_timer0 = NULL;

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

/**
 * CPU clock cycle counter timer
 */
static const struct timer_device *const g_cpu_clock_cycle_counter_timer_p =
                                            &g_timer_devices[1];

/**
 * McRTOS interrupt object for the UART0 interrupts
 *
 * NOTE: It is used in lpc2478_interrupt_handlers.s.
 */
struct rtos_interrupt *g_rtos_interrupt_uart0 = NULL;
 
/**
 * Global array of non-const structures for UART devices for the LPC2478
 * (allocated in SRAM space)
 */
static struct uart_device_var g_uart_devices_var[] =
{
    [0] = {
        .urt_initialized = false,
        .urt_signal_thre_condvar = false,
        .urt_signal_rdr_condvar = false,
        .urt_byte_received_pending = false,
        .urt_first_byte_received = '\0',
        },

    [1] = {
        .urt_initialized = false,
        .urt_signal_thre_condvar = false,
        .urt_signal_rdr_condvar = false,
        .urt_byte_received_pending = false,
        .urt_first_byte_received = '\0',
        },

    [2] = {
        .urt_initialized = false,
        .urt_signal_thre_condvar = false,
        .urt_signal_rdr_condvar = false,
        .urt_byte_received_pending = false,
        .urt_first_byte_received = '\0',
        },

    [3] = {
        .urt_initialized = false,
        .urt_signal_thre_condvar = false,
        .urt_signal_rdr_condvar = false,
        .urt_byte_received_pending = false,
        .urt_first_byte_received = '\0',
        },
};

/**
 * Global array of const structures for UART devices for the LPC2478 
 * (allocated in flash space)
 */
static const struct uart_device g_uart_devices[] =
{
    [0] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
        .urt_var_p = &g_uart_devices_var[0],
        .urt_mmio_uart_p = (lpc2478_uart_t *)LPC2478_UART0_BASE_ADDR,
        .urt_uart_pclk = PCLK_UART0,
        .urt_pclksel_index = 0,
        .urt_pconp_mask = PCONP_PCUART0,
        .urt_rtos_interrupt_params = {
            .irp_name_p = "UART0 Interrupt",
            .irp_isr_function_p = isr_uart0,
            .irp_arg_p =  (void *)&g_uart_devices[0],
            .irp_channel = VIC_CHANNEL_UART0,
            .irp_priority = VIC_VECT_PRIORITY6,
            .irp_cpu_id = 0,
        },

        .urt_rtos_interrupt_pp = &g_rtos_interrupt_uart0,
        .urt_thre_condvar_name_p = "UART0 THRE condvar",
        .urt_rdr_condvar_name_p = "UART0 RDR condvar",
    },

    [1] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
        .urt_var_p = &g_uart_devices_var[1],
        .urt_mmio_uart_p = (lpc2478_uart_t *)LPC2478_UART1_BASE_ADDR,
        .urt_uart_pclk = PCLK_UART1,
        .urt_pclksel_index = 0,
        .urt_pconp_mask = PCONP_PCUART1,
    },

    [2] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
        .urt_var_p = &g_uart_devices_var[2],
        .urt_mmio_uart_p = (lpc2478_uart_t *)LPC2478_UART2_BASE_ADDR,
        .urt_uart_pclk = PCLK_UART2,
        .urt_pclksel_index = 1,
        .urt_pconp_mask = PCONP_PCUART2,
    },

    [3] = {
        .urt_signature = UART_DEVICE_SIGNATURE,
        .urt_var_p = &g_uart_devices_var[3],
        .urt_mmio_uart_p = (lpc2478_uart_t *)LPC2478_UART3_BASE_ADDR,
        .urt_uart_pclk = PCLK_UART3,
        .urt_pclksel_index = 1,
        .urt_pconp_mask = PCONP_PCUART3,
    }
};

C_ASSERT(
    ARRAY_SIZE(g_uart_devices) == ARRAY_SIZE(g_uart_devices_var));

const struct uart_device *const g_console_serial_port_p = &g_uart_devices[0];

/*
 *  A/D converter
 */

/**
 * McRTOS interrupt object for the A/D converter interrupts
 *
 * NOTE: It is used in lpc2478_interrupt_handlers.s.
 */
struct rtos_interrupt *g_rtos_interrupt_adc = NULL;
 
/**
 * Global array of non-const structures for A/D converter channels
 * (allocated in SRAM space)
 */
static struct adc_channel g_adc_channels[NUM_ADC_CHANNELS];

/**
 * Global const structure for the A/D converter device
 * (allocated in flash space)
 */
static const struct adc_device g_adc_device = {
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

const struct adc_device *const g_adc_device_p = &g_adc_device;

/*
 * Pin definitions for LEDs
 */

static const struct pin_config_info g_sd_mmc_led_pin =
    PIN_COFIG_INFO_INITIALIZER(
        LED_PINS_GPIO_PORT, SD_MMC_LED_PIN_BIT_INDEX, PINSEL_PRIMARY, true);

static const struct pin_config_info g_usb_host_link_led_pin =
    PIN_COFIG_INFO_INITIALIZER(
        LED_PINS_GPIO_PORT, USB_HOST_LINK_LED_PIN_BIT_INDEX, PINSEL_PRIMARY, false);

static const struct pin_config_info g_usb_device_link_led_pin =
    PIN_COFIG_INFO_INITIALIZER(
        LED_PINS_GPIO_PORT, USB_DEVICE_LINK_LED_PIN_BIT_INDEX, PINSEL_PRIMARY, false);

/**
 * McRTOS interrupt object for the GPIO ports interrupt
 *
 * NOTE: It is used in lpc2478_interrupt_handlers.s.
 */
struct rtos_interrupt *g_rtos_interrupt_gpio_ports = NULL;

/** 
 * Condvar signaled when any of the buttons of the buttons device is pressed
 */
static struct rtos_condvar g_buttons_pressed_condvar;

/**
 * Buttons device
 */

static struct buttons_device_var g_buttons_device_var =
{
    .bd_buttons_pressed_bitmap = 0x0,
};

static const struct buttons_device g_buttons_device =
{
    .bd_signature = BUTTONS_DEVICE_SIGNATURE,
    .bd_var_p = &g_buttons_device_var,
    .bd_gpio_port_index = GPIO_PORT_P2,

    .bd_gpio_mmio_interrupt_registers_p =
        (struct gpio_interrupt_registers *)GPIO_INTERRUPT_REGISTERS_BASE_ADDR,

    .bd_gpio_port_mmio_interrupt_registers_p =
        &((struct gpio_interrupt_registers *)
            GPIO_INTERRUPT_REGISTERS_BASE_ADDR)->gpio_port_p2,

    .bd_button1_pin = 
        PIN_COFIG_INFO_INITIALIZER(
            BUTTON_PINS_GPIO_PORT, BUTTON1_PIN_BIT_INDEX, PINSEL_PRIMARY, false),

    .bd_button2_pin = 
        PIN_COFIG_INFO_INITIALIZER(
            BUTTON_PINS_GPIO_PORT, BUTTON2_PIN_BIT_INDEX, PINSEL_PRIMARY, false),

    .bd_rtos_interrupt_params = {
        .irp_name_p = "GPIO Ports 0 and 2 Interrupt",
        .irp_isr_function_p = isr_gpio_ports,
        .irp_arg_p =  (void *)&g_buttons_device,
        .irp_channel = VIC_CHANNEL_EINT3,
        .irp_priority = VIC_VECT_PRIORITY4,
        .irp_cpu_id = 0,
    },

    .bd_rtos_interrupt_pp = &g_rtos_interrupt_gpio_ports,
    .bd_condvar_name_p = "Buttons pressed condvar",
};

const struct buttons_device *const g_buttons_device_p = &g_buttons_device;

/**
 * Global array of non-const structures for the LPC2478 SSP controllers
 * (allocated in SRAM space)
 */
static struct ssp_controller_var g_ssp_controllers_var[] =
{
    [0] = {
        .sc_interrupt_pending = false,
    },
    [1] = {
        .sc_interrupt_pending = false,
    },
};

/**
 * McRTOS interrupt objects for the SSP controllers
 *
 * NOTE: It is used in lpc2478_interrupt_handlers.s.
 */
struct rtos_interrupt *g_rtos_interrupt_ssp_controller0 = NULL;
struct rtos_interrupt *g_rtos_interrupt_ssp_controller1 = NULL;

/**
 * Global array of const structures for the LPC2478 SSP controllers
 * (allocated in flash space)
 */
static const struct ssp_controller g_ssp_controllers[] =
{
    [0] = {
        .sc_signature = SSP_CONTROLLER_SIGNATURE,
        .sc_var_p = &g_ssp_controllers_var[0],
        .sc_mmio_registers_p = (lpc2478_ssp_t *)LPC2478_SSP0_BASE_ADDR,
        .sc_ssp_pclk_shift = PCLK_SSP0,
        .sc_pclksel_index = 1,
        .sc_pconp_mask = PCONP_PCSSP0,
        .sc_pin_sck =  PIN_COFIG_INFO_INITIALIZER(
            SSP0_PINS_GPIO_PORT, SSP0_SCK_PIN_BIT_INDEX, SSP0_PINS_GPIO_FUNCTION, false),
        .sc_pin_ssel = PIN_COFIG_INFO_INITIALIZER(
            SSP0_PINS_GPIO_PORT, SSP0_SSEL_PIN_BIT_INDEX, SSP0_PINS_GPIO_FUNCTION, false),
        .sc_pin_miso = PIN_COFIG_INFO_INITIALIZER(
            SSP0_PINS_GPIO_PORT, SSP0_MISO_PIN_BIT_INDEX, SSP0_PINS_GPIO_FUNCTION, false),
        .sc_pin_mosi = PIN_COFIG_INFO_INITIALIZER(
            SSP0_PINS_GPIO_PORT, SSP0_MOSI_PIN_BIT_INDEX, SSP0_PINS_GPIO_FUNCTION, false),
        .sc_rtos_interrupt_params = {
            .irp_name_p = "McRTOS SSP controller 0 Interrupt",
            .irp_isr_function_p = isr_ssp_controller0,
            .irp_arg_p =  (void *)&g_ssp_controllers[0],
            .irp_channel = VIC_CHANNEL_SPI_SSP0,
            .irp_priority = VIC_VECT_PRIORITY5,
            .irp_cpu_id = 0,
        },
        .sc_rtos_interrupt_pp = &g_rtos_interrupt_ssp_controller0,
        .sc_condvar_name_p = "SSP controller 0 condvar",
    },
    [1] = {
        .sc_signature = SSP_CONTROLLER_SIGNATURE,
        .sc_var_p = &g_ssp_controllers_var[1],
        .sc_mmio_registers_p = (lpc2478_ssp_t *)LPC2478_SSP1_BASE_ADDR,
        .sc_ssp_pclk_shift = PCLK_SSP1,
        .sc_pclksel_index = 0,
        .sc_pconp_mask = PCONP_PCSSP1,
        .sc_pin_sck =  PIN_COFIG_INFO_INITIALIZER(
            SSP1_PINS_GPIO_PORT, SSP1_SCK_PIN_BIT_INDEX, SSP1_PINS_GPIO_FUNCTION, false),
        .sc_pin_ssel = PIN_COFIG_INFO_INITIALIZER(
            SSP1_PINS_GPIO_PORT, SSP1_SSEL_PIN_BIT_INDEX, SSP1_PINS_GPIO_FUNCTION, false),
        .sc_pin_miso = PIN_COFIG_INFO_INITIALIZER(
            SSP1_PINS_GPIO_PORT, SSP1_MISO_PIN_BIT_INDEX, SSP1_PINS_GPIO_FUNCTION, false),
        .sc_pin_mosi = PIN_COFIG_INFO_INITIALIZER(
            SSP1_PINS_GPIO_PORT, SSP1_MOSI_PIN_BIT_INDEX, SSP1_PINS_GPIO_FUNCTION, false),
        .sc_rtos_interrupt_params = {
            .irp_name_p = "McRTOS SSP controller 1 Interrupt",
            .irp_isr_function_p = isr_ssp_controller1,
            .irp_arg_p =  (void *)&g_ssp_controllers[1],
            .irp_channel = VIC_CHANNEL_SSP1,
            .irp_priority = VIC_VECT_PRIORITY5,
            .irp_cpu_id = 0,
        },
        .sc_rtos_interrupt_pp = &g_rtos_interrupt_ssp_controller1,
        .sc_condvar_name_p = "SSP controller 1 condvar",
    }
};

const struct ssp_controller * const g_ssp0_controller_p = &g_ssp_controllers[0];
const struct ssp_controller * const g_ssp1_controller_p = &g_ssp_controllers[1];

static bool spi_connected = false;
static bool ssp0_connected = false;


/**
 * Turn on power for a give peripheral or set of peripherals
 */
void
turn_on_power(uint32_t pconp_mask)
{
    uint32_t reg_value = read_32bit_mmio_register(&g_scb_mmio_registers_p->reg_PCONP);

    /*
     * Clear reserved bits. Reading reserved bits is not defined, so the
     * register value read can contain garbage 1's in these bits, and we are
     * not supposed to write 1's to reserved bits.
     */ 
    CLEAR_BIT_FIELD(reg_value, PCONP_RESERVED_MASK);

    reg_value |= pconp_mask;

    write_32bit_mmio_register(
        &g_scb_mmio_registers_p->reg_PCONP, reg_value);
}


static void
write_reg_AD0CR(volatile uint32_t *reg_AD0CR_p, uint32_t reg_value)
{
    /*
     * Clear reserved bits. Reading reserved bits is not defined, so the
     * register value read can contain garbage 1's in these bits, and we are
     * not supposed to write 1's to reserved bits.
     */
    reg_value &= ~AD0CR_RESERVED1_MASK;
    CLEAR_BIT_FIELD(reg_value, AD0CR_RESERVED2_MASK);
    CLEAR_BIT_FIELD(reg_value, AD0CR_RESERVED3_MASK);

    write_32bit_mmio_register(reg_AD0CR_p, reg_value);
}


/**
 * It configures a pin of the LPC2478
 *
 * the 'is_output' parameter is only meaningful if the selected
 * pin function is the primary function (PINSEL_PRIMARY)
 */
void
configure_pin(const struct pin_config_info *pin_info_p, bool is_output)
{
    uint32_t reg_value;
    uint8_t pinsel_mode_index =
            GET_PINSEL_MODE_INDEX(pin_info_p->gpio_port_index,
                                  pin_info_p->pin_bit_index);
    volatile uint32_t *reg_PINSEL_p =
        &g_pin_connect_block->reg_PINSEL[pinsel_mode_index];
    volatile uint32_t *reg_PINMODE_p =
        &g_pin_connect_block->reg_PINMODE[pinsel_mode_index];
    volatile uint32_t *reg_FIODIR_p =
        &g_fast_gpio_ports_array[pin_info_p->gpio_port_index].reg_FIODIR;

    /*
     * Set pin function:
     */
    reg_value = read_32bit_mmio_register(reg_PINSEL_p);
    SET_BIT_FIELD(reg_value,
        pin_info_p->pinsel_mode_mask,
        pin_info_p->pinsel_mode_shift,
        pin_info_p->pin_function);
    write_32bit_mmio_register(reg_PINSEL_p, reg_value);

    /*
     * Set pin mode to neither pull-up nor pull-down enabled:
     */
    reg_value = read_32bit_mmio_register(reg_PINMODE_p);
    SET_BIT_FIELD(reg_value,
        pin_info_p->pinsel_mode_mask,
        pin_info_p->pinsel_mode_shift,
        PINMODE_PULL_NONE);
    write_32bit_mmio_register(reg_PINMODE_p, reg_value);

    if (pin_info_p->pin_function == PINSEL_PRIMARY)
    {
        /*
         * Set pin I/O direction:
         */
        if (is_output)
        {
            /*
             * Initialize pin to be de-asserted, before setting it as an
             * output pin:
             */ 
            deactivate_output_pin(pin_info_p);

            reg_value = read_32bit_mmio_register(reg_FIODIR_p);
            reg_value |= BIT(pin_info_p->pin_bit_index);
            write_32bit_mmio_register(reg_FIODIR_p, reg_value);
        }
        else
        {
            reg_value = read_32bit_mmio_register(reg_FIODIR_p);
            reg_value &= ~BIT(pin_info_p->pin_bit_index);
            write_32bit_mmio_register(reg_FIODIR_p, reg_value);
        }
    }
}


void
activate_output_pin(const struct pin_config_info *pin_info_p)
{
    uint32_t reg_value;
    volatile uint32_t *reg_FIODIR_p =
        &g_fast_gpio_ports_array[pin_info_p->gpio_port_index].reg_FIODIR;

    FDC_ASSERT(
        pin_info_p->pin_function == PINSEL_PRIMARY,
        pin_info_p->pin_function, PINSEL_PRIMARY);

    reg_value = read_32bit_mmio_register(reg_FIODIR_p);
    FDC_ASSERT(
        reg_value & BIT(pin_info_p->pin_bit_index),
        pin_info_p, reg_value);

    if (pin_info_p->pin_is_active_high)
    {
        volatile uint32_t *reg_FIOSET_p =
            &g_fast_gpio_ports_array[pin_info_p->gpio_port_index].reg_FIOSET;
                
        write_32bit_mmio_register(
            reg_FIOSET_p,
            BIT(pin_info_p->pin_bit_index));
    }
    else
    {
        volatile uint32_t *reg_FIOCLR_p =
            &g_fast_gpio_ports_array[pin_info_p->gpio_port_index].reg_FIOCLR;

        write_32bit_mmio_register(
            reg_FIOCLR_p,
            BIT(pin_info_p->pin_bit_index));
    }
}


void
deactivate_output_pin(const struct pin_config_info *pin_info_p)
{
    FDC_ASSERT(
        pin_info_p->pin_function == PINSEL_PRIMARY,
        pin_info_p->pin_function, PINSEL_PRIMARY);

    if (pin_info_p->pin_is_active_high)
    {
        volatile uint32_t *reg_FIOCLR_p =
            &g_fast_gpio_ports_array[pin_info_p->gpio_port_index].reg_FIOCLR;

        write_32bit_mmio_register(
            reg_FIOCLR_p,
            BIT(pin_info_p->pin_bit_index));
    }
    else
    {
        volatile uint32_t *reg_FIOSET_p =
            &g_fast_gpio_ports_array[pin_info_p->gpio_port_index].reg_FIOSET;
                
        write_32bit_mmio_register(
            reg_FIOSET_p,
            BIT(pin_info_p->pin_bit_index));
    }
}


bool
read_input_pin(const struct pin_config_info *pin_info_p)
{
    volatile uint32_t *reg_FIOPIN_p =
        &g_fast_gpio_ports_array[pin_info_p->gpio_port_index].reg_FIOPIN;

    uint32_t reg_value = read_32bit_mmio_register(reg_FIOPIN_p);

    return (reg_value & BIT(pin_info_p->pin_bit_index)) != 0;
}


/**
 *  Initializes SoC hardware
 */
void soc_hardware_init(void)
{
    /* Initial SCB Parameters (CCLK 72MHz) */
    static const struct scbParams sCBParams = {
        .PLL_M_Mul = 12,           /* PLL Multiplier. Valid values 6 through 512*/
        .PLL_N_Div = 1,            /* PLL Divider. Valid values 1 through 32 */
        .PLL_Fcco = 288000000,     /* Frequency (Hz) of PLL output */
        .CCLK_Div = 4,             /* CPU Clock divider, cclk */
        .MAMMode = MAMCR_FULL,     /* MAM mode Partial is the preferred setting for Rev -,A parts */
        .MAMTim = MAMTIM_5_CLK,    /* Override optimal MAM timing due to LPC2478 Rev.C MAM errata */
    };

    uint32_t cclk = sCBParams.PLL_Fcco / sCBParams.CCLK_Div;

    FDC_ASSERT(cclk == CPU_CLOCK_FREQ_IN_HZ, cclk, CPU_CLOCK_FREQ_IN_HZ);

    /*
     * initialize main hardware components
     */
    /* Must turn off MAM before increasing frequency */
    initMAM(cclk, MAMCR_OFF, MAMTIM_7_CLK);

    /* Disable SDRAM before changing frequency (all data in SDRAM will be lost */
    disableSDRAM();

    initPLL(&sCBParams);
    initMAM(cclk, sCBParams.MAMMode, sCBParams.MAMTim);
    initPCLK();
    initGPIO();

    /* MEMMAP Choices are:
     * BOOTLOADERMODE      0x00
     * USERFLASHMODE       0x01
     * USERRAMMODE         0x02
     * EXTERNALMEMORYMODE  0x03
     */
    g_scb_mmio_registers_p->reg_MEMMAP = USERFLASHMODE;

    initVIC();

    uart_init(
        g_console_serial_port_p,
        DEBUG_SERIAL_PORT_BAUD_RATE,
        DEBUG_SERIAL_PORT_MODE);

    init_leds();

    init_adc(g_adc_device_p);

    /*
     * Initialize timer used for counting CPU clock cycles:
     */
    init_timer(
        g_cpu_clock_cycle_counter_timer_p,
        CPU_CLOCK_CYCLE_COUNTER_TIMER_FREQUENCY,
        false);

    /*
     * Initialize SDRAM:
     */
    initSDRAM(cclk);

    /*
     * Initialize peripherals that use DRAM:
     */
    
    init_lcd();

    TODO("Enable init_ethernet")
    //TODO init_ethernet();
}


/**
 * Initializes the RTOS tick timer for the calling CPU core
 */ 
void
initialize_tick_timer(void)
{
    /**
     * McRTOS tick timers for each CPU core
     */
    static const struct timer_device *const rtos_tick_timers_p[] =
    {
        [0] = &g_timer_devices[0],
    };

    C_ASSERT2(rtos_tick_timers_p_array_size,
        ARRAY_SIZE(rtos_tick_timers_p) == SOC_NUM_CPU_CORES);

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    rtos_k_register_interrupt(
        &rtos_tick_timers_p[cpu_id]->tmr_rtos_interrupt_params,
        rtos_tick_timers_p[cpu_id]->tmr_rtos_interrupt_pp);

    DBG_ASSERT(
        *rtos_tick_timers_p[cpu_id]->tmr_rtos_interrupt_pp != NULL,
        rtos_tick_timers_p[cpu_id]->tmr_rtos_interrupt_pp, cpu_id);

    init_timer(
        rtos_tick_timers_p[cpu_id],
        RTOS_TICK_TIMER_FREQUENCY,
        true);
}


uint32_t 
get_cpu_clock_cycles(void)
{
    lpc2478_timer_t *timer_mmio_registers_p =
        g_cpu_clock_cycle_counter_timer_p->tmr_mmio_registers_p;

    return read_32bit_mmio_register(&timer_mmio_registers_p->reg_TC) *
           CPU_CLOCK_FREQ_DIVIDER;
}


/******************************************************************************
 *
 * DESCRIPTION
 *
 *  This init is specific to the 2400 family of devices.
 *
 * NOTES
 *
 *  Taken from the LPC2478 User Manual:
 *
 *   5.14 PLL setup sequence
 *
 *   The following sequence must be followed step by step in order to
 *   have the PLL initialized and running:
 *
 *   1. Disconnect the PLL with one feed sequence if PLL is already
 *      connected.
 *   2. Disable the PLL with one feed sequence.
 *   3. Change the CPU Clock Divider setting to speed up operation without
 *      the PLL, if desired.
 *   4. Write to the Clock Source Selection Control register to change the
 *      clock source.
 *   5. Write to the PLLCFG and make it effective with one feed sequence.
 *      The PLLCFG can only be updated when the PLL is disabled.
 *   6. Enable the PLL with one feed sequence.
 *   7. Change the CPU Clock Divider setting for the operation with the
 *      PLL. It's critical to do this before connecting the PLL.
 *   8. Wait for the PLL to achieve lock by monitoring the PLOCK bit in the
 *      PLLSTAT register, or using the PLOCK interrupt, or wait for a fixed
 *      time when the input clock to PLL is slow (i.e. 32 kHz). The value of
 *      PLOCK may not be stable when the PLL reference frequency (FREF, the
 *      frequency of REFCLK, which is equal to the PLL input frequency divided
 *      by the pre-divider value) is less than 100 kHz or greater than 20
 *      MHz. In these cases, the PLL may be assumed to be stable after a
 *      start-up time has passed. This time is 500 us when FREF is
 *      greater than 400 kHz and 200 / FREF seconds when FREF is less than
 *      400kHz.
 *   9. Connect the PLL with one feed sequence.
 *
 *   It's very important not to merge any steps above. For example, don't
 *   update the PLLCFG and enable the PLL simultaneously with the same feed
 *   sequence.
 *
 *
 *  This implementation is fixed as follows:
 * 1. Uses the main oscillator
 * 2. PLL Fcco is fixed at compile time at 288 MHz using external defines
 * 3. It also sets the CCLK to a known stable 48 MHz
 */
static void initPLL(const struct scbParams *pSCBParams)
{
    uint32_t pllcfg;
    fdc_error_t fdc_error;

    /* Validate the incoming parameters based on supported values for the Main OSC */
    if ( pSCBParams->PLL_M_Mul < 6 ||
       pSCBParams->PLL_M_Mul > 512 ||
       pSCBParams->PLL_N_Div < 1 ||
       pSCBParams->PLL_N_Div > 32 ||
       pSCBParams->PLL_Fcco < 275000000 ||
       pSCBParams->PLL_Fcco > 550000000 ||
       pSCBParams->CCLK_Div == 0
        )
    {
      /* invalid value: M, N, Fcco, CCLK_Div */
      fdc_error = CAPTURE_FDC_ERROR("Invalid value: M, N, Fcco, CCLK_Div", pSCBParams, 0);
      fatal_error_handler(fdc_error);
    }

    /* Validate the M and N matches the provided Fcco */
    if ( pSCBParams->PLL_Fcco != ((2 * pSCBParams->PLL_M_Mul * FOSC_MAIN)/pSCBParams->PLL_N_Div) )
    {
      /* invalid M and N or Fcco combinations */
      fdc_error = CAPTURE_FDC_ERROR("Invalid M and N or Fcco combinations", pSCBParams, 0);
      fatal_error_handler(fdc_error);
    }

    /* [1] Check if PLL connected, disconnect if yes. */
    if ((g_scb_mmio_registers_p->reg_PLLSTAT) & PLLSTAT_PLLC)
    {
        g_scb_mmio_registers_p->reg_PLLCON = PLLCON_PLLE;
        /* Enable PLL, disconnected ( PLLC = 0)*/
        g_scb_mmio_registers_p->reg_PLLFEED = 0xAA;
        g_scb_mmio_registers_p->reg_PLLFEED = 0x55;
    }

    /* [2] Disable the PLL once it has been disconnected. */
    g_scb_mmio_registers_p->reg_PLLCON  = 0;
    g_scb_mmio_registers_p->reg_PLLFEED = 0xAA;
    g_scb_mmio_registers_p->reg_PLLFEED = 0x55;

    /* [3] Change the CPU Clock Divider setting
     * to speed up operation without the PLL, if desired.
     * We're going to divide by 1 for maximum non-PLL clock speed.
     * NOTE: CCLKCFG adds one internally so we must subtract one */
    g_scb_mmio_registers_p->reg_CCLKCFG = (CCLK_DIV_1 - 1);

    /* [4] Enable the main oscillator, select clock source  */
    if( FOSC_MAIN > 20000000 )
    {
        g_scb_mmio_registers_p->reg_SCS |= SCS_OSCRANGE;
    }
    else
    {
        g_scb_mmio_registers_p->reg_SCS &= ~SCS_OSCRANGE;
    }

    g_scb_mmio_registers_p->reg_SCS |= SCS_OSCEN;

    /* Wait until main OSC is usable */
    do {} while ((g_scb_mmio_registers_p->reg_SCS & SCS_OSCSTAT) == 0);

    /* Select the main osc as the PLL clk source. */
    g_scb_mmio_registers_p->reg_CLKSRCSEL = CLKSRC_MAIN_OSC;

    /*
     * NOTES:
     *
     * Set multiplier and divider values.
     *
     *  PLLCFG = ((N - 1) << 16) + (M - 1)
     *
     *  F_cco = (2 * M *F_in)/N
     *      M = 12, N = 1, F_in = 12.000 MHz -> F_cco = 288.000 MHz
     *
     */
    /* [5] Write to the PLLCFG and make it effective with feed sequence */
    pllcfg = (((pSCBParams->PLL_N_Div - 1) << 16) + ((pSCBParams->PLL_M_Mul - 1) << 0));
    g_scb_mmio_registers_p->reg_PLLCFG = pllcfg;
    g_scb_mmio_registers_p->reg_PLLFEED = 0xAA;
    g_scb_mmio_registers_p->reg_PLLFEED = 0x55;

    /* [6] Enable the PLL and make it effective with feed sequence */
    g_scb_mmio_registers_p->reg_PLLCON  = PLLCON_PLLE;
    g_scb_mmio_registers_p->reg_PLLFEED = 0xAA;
    g_scb_mmio_registers_p->reg_PLLFEED = 0x55;

    /* [7] Change the CPU Clock Divider setting for the operation with the PLL.
     *     It's critical to do this before connecting the PLL.
     * NOTE: CCLKCFG adds one internally so we must subtract one
     * Divide F_cco down to get the CCLK output. */
    g_scb_mmio_registers_p->reg_CCLKCFG = (pSCBParams->CCLK_Div - 1);

    /* [8] Wait for the PLL to lock to set frequency
     * TODO extra credit: Add timeout for slow FREF */
    do {} while(((g_scb_mmio_registers_p->reg_PLLSTAT) & PLLSTAT_PLOCK) == 0);
    do {} while(((g_scb_mmio_registers_p->reg_PLLSTAT) & 0x00FF7FFF) != pllcfg);

    /* [9] Enable and connect the PLL as the clock source */
    g_scb_mmio_registers_p->reg_PLLCON = (PLLCON_PLLE | PLLCON_PLLC);
    g_scb_mmio_registers_p->reg_PLLFEED = 0xAA;
    g_scb_mmio_registers_p->reg_PLLFEED = 0x55;

    /* Check connect bit status and wait for connection. */
    do {} while(((g_scb_mmio_registers_p->reg_PLLSTAT) & PLLSTAT_PLLC) == 0);

    return;
}

/*
 * DESCRIPTION
 *
 * enable the MAM accelerator and sets number of clocks used for
 * flash memory fetch.
 *
 * PARAMETERS
 * cclk : CPU Clock Frequency
 * mamcr: MAM mode MAMCR_OFF, MAMCR_PARTIAL, or MAMCR_FULL
 *
 * EXAMPLE
 *
 *   initMAM(12000000, MAMCR_PARTIAL);
 */

static void initMAM(uint32_t cclk, uint8_t mamcr, uint8_t mamtim_override)
{
    /* disable MAM */
    g_scb_mmio_registers_p->reg_MAMCR = MAMCR_OFF;

    /* Either set according to optimally calculated value or override parameter */
    if(mamtim_override == MAMTIM_AUTOCFG)
    {
        g_scb_mmio_registers_p->reg_MAMTIM  = (((cclk)+19999999)/20000000); /* stable setting */
    }
    else if( (mamtim_override > MAMTIM_AUTOCFG) && (mamtim_override <= MAMTIM_MAX_CLK) )
    {
        g_scb_mmio_registers_p->reg_MAMTIM  = mamtim_override;
    }
    else
    {
        /* 
         * invalid value
         * setting to reset value for now 
         */
        (void)CAPTURE_FDC_ERROR(
            "Invalid mamtim_override", mamtim_override, 0);

        g_scb_mmio_registers_p->reg_MAMTIM  = MAMTIM_MAX_CLK;
    }

    g_scb_mmio_registers_p->reg_MAMCR   = mamcr;
}


static uint32_t
getFcclk(void)
{
    uint32_t Fosc;
    uint32_t Fpllclk;
    uint32_t Fcclk;
    uint8_t  clksrcsel;
    uint32_t pllstat;
    uint16_t M_mul;
    uint8_t  N_div;
    uint8_t  plle;
    uint8_t  pllc;

    /* Determine Fosc from configured clock source */
    clksrcsel = g_scb_mmio_registers_p->reg_CLKSRCSEL;
    if( (clksrcsel & CLKSRCSEL_MASK) == CLKSRC_INT_RC )
    {
        Fosc = FOSC_INT_RC;
    }
    else if( (clksrcsel & CLKSRCSEL_MASK) == CLKSRC_MAIN_OSC )
    {
        Fosc = FOSC_MAIN;
    }
    else if( (clksrcsel & CLKSRCSEL_MASK) == CLKSRC_MIN_RTC )
    {
        Fosc = FOSC_RTC;
    }
    else /* Invalid clock source */
    {
        return -1;
    }

    /* Determine Fpllclk based on PLL configuration */
    pllstat = g_scb_mmio_registers_p->reg_PLLSTAT;
    M_mul = (pllstat & PLLSTAT_MSEL_MASK) + 1;
    N_div = ((pllstat & PLLSTAT_NSEL_MASK) >> 16) + 1;
    plle  = (pllstat & PLLSTAT_PLLE) >> 24;
    pllc  = (pllstat & PLLSTAT_PLLC) >> 25;

    if( plle && pllc )
    {
        Fpllclk = (2 * M_mul * Fosc) / N_div;
    }
    else
    {
        Fpllclk = Fosc;
    }

    /* Find out cclk frequency based on CCLKCFG */
    Fcclk = Fpllclk / (g_scb_mmio_registers_p->reg_CCLKCFG + 1);

    return Fcclk;
}

/*
 * NAME
 *
 * initPCLK
 *
 * DESCRIPTION
 *
 * Set the pclk for each peripheral. The default PDIV_4, but we do that
 * explicitly here to make it easier to change them later.
 *
 * PARAMETER
 *
 * none...
 *
 * EXAMPLE
 *
 *   initPCLK();
 *
 * NOTES
 *
 *  none...
 *
 */

static void initPCLK(void)
{
    g_scb_mmio_registers_p->reg_PCLKSEL[0] =
    (
          SET_PCLK (PCLK_WDT,     PDIV_4)
        | SET_PCLK (PCLK_TIMER0,  PDIV_1)
        | SET_PCLK (PCLK_TIMER1,  PDIV_1)
        | SET_PCLK (PCLK_UART0,   PDIV_1)
        | SET_PCLK (PCLK_UART1,   PDIV_1)
        | SET_PCLK (PCLK_PWM1,    PDIV_4)
        | SET_PCLK (PCLK_I2C0,    PDIV_4)
        | SET_PCLK (PCLK_SPI,     PDIV_4)
        | SET_PCLK (PCLK_RTC,     PDIV_4)
        | SET_PCLK (PCLK_SSP1,    PDIV_4)
        | SET_PCLK (PCLK_DAC,     PDIV_4)
        | SET_PCLK (PCLK_ADC,     PDIV_4)
        | SET_PCLK (PCLK_CAN1,    PDIV_4)
        | SET_PCLK (PCLK_CAN2,    PDIV_4)
        | SET_PCLK (PCLK_ACF,     PDIV_4)
    );

    g_scb_mmio_registers_p->reg_PCLKSEL[1] =
    (
          SET_PCLK (PCLK_BAT_RAM, PDIV_4)
        | SET_PCLK (PCLK_GPIO,    PDIV_4)
        | SET_PCLK (PCLK_PCB,     PDIV_4)
        | SET_PCLK (PCLK_I2C1,    PDIV_4)
        | SET_PCLK (PCLK_SSP0,    PDIV_4)
        | SET_PCLK (PCLK_TIMER2,  PDIV_4)
        | SET_PCLK (PCLK_TIMER3,  PDIV_4)
        | SET_PCLK (PCLK_UART2,   PDIV_1)
        | SET_PCLK (PCLK_UART3,   PDIV_1)
        | SET_PCLK (PCLK_I2C2,    PDIV_4)
        | SET_PCLK (PCLK_I2S,     PDIV_4)
        | SET_PCLK (PCLK_MCI,     PDIV_4)
        | SET_PCLK (PCLK_SYSCON,  PDIV_4)
    );
}


static void initGPIO(void)
{
    DBG_ASSERT_EQUAL(
        &g_fast_gpio_ports_array[GPIO_PORT_P0], FIO_BASE_ADDR);
    DBG_ASSERT_EQUAL(
        &g_fast_gpio_ports_array[GPIO_PORT_P1], FIO_BASE_ADDR + FAST_GPIO_PORT_SIZE);
    DBG_ASSERT_EQUAL(
        &g_fast_gpio_ports_array[GPIO_PORT_P2], FIO_BASE_ADDR + FAST_GPIO_PORT_SIZE*2);
    DBG_ASSERT_EQUAL(
        &g_fast_gpio_ports_array[GPIO_PORT_P3], FIO_BASE_ADDR + FAST_GPIO_PORT_SIZE*3);
    DBG_ASSERT_EQUAL(
        &g_fast_gpio_ports_array[GPIO_PORT_P4], FIO_BASE_ADDR + FAST_GPIO_PORT_SIZE*4);

    //TODO:
    TODO("Use PINSEL_PRIMARY for all pins and move P0.2, P0.3 to uart_init for UART0 and use a loop here")

    g_pin_connect_block->reg_PINSEL[0] =
    (
          PINSEL_MASK( PINSEL_PRIMARY, 0 )  /* P0.0  */
        | PINSEL_MASK( PINSEL_PRIMARY, 1 )  /* P0.1  */
        | PINSEL_MASK( PINSEL_ALT_01 , 2 )  /* P0.2 TXD0 - UART0 */
        | PINSEL_MASK( PINSEL_ALT_01 , 3 )  /* P0.3 RXD0 - UART0 */
        | PINSEL_MASK( PINSEL_PRIMARY, 4 )  /* P0.4  */
        | PINSEL_MASK( PINSEL_PRIMARY, 5 )  /* P0.5  */
        | PINSEL_MASK( PINSEL_PRIMARY, 6 )  /* P0.6  */
        | PINSEL_MASK( PINSEL_PRIMARY, 7 )  /* P0.7  */
        | PINSEL_MASK( PINSEL_PRIMARY, 8 )  /* P0.8  */
        | PINSEL_MASK( PINSEL_PRIMARY, 9 )  /* P0.9  */
        | PINSEL_MASK( PINSEL_PRIMARY,10 )  /* P0.10 */
        | PINSEL_MASK( PINSEL_PRIMARY,11 )  /* P0.11 */
        | PINSEL_MASK( PINSEL_PRIMARY,12 )  /* P0.12 */
        | PINSEL_MASK( PINSEL_PRIMARY,13 )  /* P0.13 */
        | PINSEL_MASK( PINSEL_PRIMARY,14 )  /* P0.14 */
        | PINSEL_MASK( PINSEL_PRIMARY,15 )  /* P0.15 */
    );

    g_pin_connect_block->reg_PINSEL[1] =
    (
          PINSEL_MASK( PINSEL_PRIMARY, 0 )  /* P0.16 */
        | PINSEL_MASK( PINSEL_PRIMARY, 1 )  /* P0.17 */
        | PINSEL_MASK( PINSEL_PRIMARY, 2 )  /* P0.18 */
        | PINSEL_MASK( PINSEL_PRIMARY, 3 )  /* P0.19 */
        | PINSEL_MASK( PINSEL_PRIMARY, 4 )  /* P0.20 */
        | PINSEL_MASK( PINSEL_PRIMARY, 5 )  /* P0.21 */
        | PINSEL_MASK( PINSEL_PRIMARY, 6 )  /* P0.22 */
        | PINSEL_MASK( PINSEL_PRIMARY, 7 )  /* P0.23 */
        | PINSEL_MASK( PINSEL_PRIMARY, 8 )  /* P0.24 */
        | PINSEL_MASK( PINSEL_PRIMARY, 9 )  /* P0.25 */
        | PINSEL_MASK( PINSEL_PRIMARY,10 )  /* P0.26 */
        | PINSEL_MASK( PINSEL_PRIMARY,11 )  /* P0.27 */
        | PINSEL_MASK( PINSEL_PRIMARY,12 )  /* P0.28 */
        | PINSEL_MASK( PINSEL_PRIMARY,13 )  /* P0.29 */
        | PINSEL_MASK( PINSEL_PRIMARY,14 )  /* P0.30 */
        | PINSEL_MASK( PINSEL_PRIMARY,15 )  /* P0.31 */
    );

    /* PORT1 */
    g_pin_connect_block->reg_PINSEL[2] =
    (
          PINSEL_MASK( PINSEL_PRIMARY, 0 )  /* P1.0  */
        | PINSEL_MASK( PINSEL_PRIMARY, 1 )  /* P1.1  */
        | PINSEL_MASK( PINSEL_PRIMARY, 2 )  /* P1.2  */
        | PINSEL_MASK( PINSEL_PRIMARY, 3 )  /* P1.3  */
        | PINSEL_MASK( PINSEL_PRIMARY, 4 )  /* P1.4  */
        | PINSEL_MASK( PINSEL_PRIMARY, 5 )  /* P1.5  */
        | PINSEL_MASK( PINSEL_PRIMARY, 6 )  /* P1.6  */
        | PINSEL_MASK( PINSEL_PRIMARY, 7 )  /* P1.7  */
        | PINSEL_MASK( PINSEL_PRIMARY, 8 )  /* P1.8  */
        | PINSEL_MASK( PINSEL_PRIMARY, 9 )  /* P1.9  */
        | PINSEL_MASK( PINSEL_PRIMARY,10 )  /* P1.10 */
        | PINSEL_MASK( PINSEL_PRIMARY,11 )  /* P1.11 */
        | PINSEL_MASK( PINSEL_PRIMARY,12 )  /* P1.12 */
        | PINSEL_MASK( PINSEL_PRIMARY,13 )  /* P1.13 */
        | PINSEL_MASK( PINSEL_PRIMARY,14 )  /* P1.14 */
        | PINSEL_MASK( PINSEL_PRIMARY,15 )  /* P1.15 */
    );

    g_pin_connect_block->reg_PINSEL[3] =
    (
          PINSEL_MASK( PINSEL_PRIMARY, 0 )  /* P1.16 */
        | PINSEL_MASK( PINSEL_PRIMARY, 1 )  /* P1.17 */
        | PINSEL_MASK( PINSEL_PRIMARY, 2 )  /* P1.18 */
        | PINSEL_MASK( PINSEL_PRIMARY, 3 )  /* P1.19 */
        | PINSEL_MASK( PINSEL_PRIMARY, 4 )  /* P1.20 */
        | PINSEL_MASK( PINSEL_PRIMARY, 5 )  /* P1.21 */
        | PINSEL_MASK( PINSEL_PRIMARY, 6 )  /* P1.22 */
        | PINSEL_MASK( PINSEL_PRIMARY, 7 )  /* P1.23 */
        | PINSEL_MASK( PINSEL_PRIMARY, 8 )  /* P1.24 */
        | PINSEL_MASK( PINSEL_PRIMARY, 9 )  /* P1.25 */
        | PINSEL_MASK( PINSEL_PRIMARY,10 )  /* P1.26 */
        | PINSEL_MASK( PINSEL_PRIMARY,11 )  /* P1.27 */
        | PINSEL_MASK( PINSEL_PRIMARY,12 )  /* P1.28 */
        | PINSEL_MASK( PINSEL_PRIMARY,13 )  /* P1.29 */
        | PINSEL_MASK( PINSEL_PRIMARY,14 )  /* P1.30 */
        | PINSEL_MASK( PINSEL_PRIMARY,15 )  /* P1.31 */
    );

    /* PORT2 */
    g_pin_connect_block->reg_PINSEL[4] =
    (
          PINSEL_MASK( PINSEL_PRIMARY, 0 )  /* P2.0  */
        | PINSEL_MASK( PINSEL_PRIMARY, 1 )  /* P2.1  */
        | PINSEL_MASK( PINSEL_PRIMARY, 2 )  /* P2.2  */
        | PINSEL_MASK( PINSEL_PRIMARY, 3 )  /* P2.3  */
        | PINSEL_MASK( PINSEL_PRIMARY, 4 )  /* P2.4  */
        | PINSEL_MASK( PINSEL_PRIMARY, 5 )  /* P2.5  */
        | PINSEL_MASK( PINSEL_PRIMARY, 6 )  /* P2.6  */
        | PINSEL_MASK( PINSEL_PRIMARY, 7 )  /* P2.7  */
        | PINSEL_MASK( PINSEL_PRIMARY, 8 )  /* P2.8  */
        | PINSEL_MASK( PINSEL_PRIMARY, 9 )  /* P2.9  */
        | PINSEL_MASK( PINSEL_PRIMARY,10 )  /* P2.10 */
        | PINSEL_MASK( PINSEL_PRIMARY,11 )  /* P2.11 */
        | PINSEL_MASK( PINSEL_PRIMARY,12 )  /* P2.12 */
        | PINSEL_MASK( PINSEL_PRIMARY,13 )  /* P2.13 */
        | PINSEL_MASK( PINSEL_PRIMARY,14 )  /* P2.14 */
        | PINSEL_MASK( PINSEL_PRIMARY,15 )  /* P2.15 */
    );

    g_pin_connect_block->reg_PINSEL[5] =
    (
          PINSEL_MASK( PINSEL_PRIMARY, 0 )  /* P2.16 */
        | PINSEL_MASK( PINSEL_PRIMARY, 1 )  /* P2.17 */
        | PINSEL_MASK( PINSEL_PRIMARY, 2 )  /* P2.18 */
        | PINSEL_MASK( PINSEL_PRIMARY, 3 )  /* P2.19 */
        | PINSEL_MASK( PINSEL_PRIMARY, 4 )  /* P2.20 */
        | PINSEL_MASK( PINSEL_PRIMARY, 5 )  /* P2.21 */
        | PINSEL_MASK( PINSEL_PRIMARY, 6 )  /* P2.22 */
        | PINSEL_MASK( PINSEL_PRIMARY, 7 )  /* P2.23 */
        | PINSEL_MASK( PINSEL_PRIMARY, 8 )  /* P2.24 */
        | PINSEL_MASK( PINSEL_PRIMARY, 9 )  /* P2.25 */
        | PINSEL_MASK( PINSEL_PRIMARY,10 )  /* P2.26 */
        | PINSEL_MASK( PINSEL_PRIMARY,11 )  /* P2.27 */
        | PINSEL_MASK( PINSEL_PRIMARY,12 )  /* P2.28 */
        | PINSEL_MASK( PINSEL_PRIMARY,13 )  /* P2.29 */
        | PINSEL_MASK( PINSEL_PRIMARY,14 )  /* P2.30 */
        | PINSEL_MASK( PINSEL_PRIMARY,15 )  /* P2.31 */
    );

    /* PORT3 */
    g_pin_connect_block->reg_PINSEL[6] =
    (
          PINSEL_MASK( PINSEL_PRIMARY, 0 )  /* P3.0  */
        | PINSEL_MASK( PINSEL_PRIMARY, 1 )  /* P3.1  */
        | PINSEL_MASK( PINSEL_PRIMARY, 2 )  /* P3.2  */
        | PINSEL_MASK( PINSEL_PRIMARY, 3 )  /* P3.3  */
        | PINSEL_MASK( PINSEL_PRIMARY, 4 )  /* P3.4  */
        | PINSEL_MASK( PINSEL_PRIMARY, 5 )  /* P3.5  */
        | PINSEL_MASK( PINSEL_PRIMARY, 6 )  /* P3.6  */
        | PINSEL_MASK( PINSEL_PRIMARY, 7 )  /* P3.7  */
        | PINSEL_MASK( PINSEL_PRIMARY, 8 )  /* P3.8  */
        | PINSEL_MASK( PINSEL_PRIMARY, 9 )  /* P3.9  */
        | PINSEL_MASK( PINSEL_PRIMARY,10 )  /* P3.10 */
        | PINSEL_MASK( PINSEL_PRIMARY,11 )  /* P3.11 */
        | PINSEL_MASK( PINSEL_PRIMARY,12 )  /* P3.12 */
        | PINSEL_MASK( PINSEL_PRIMARY,13 )  /* P3.13 */
        | PINSEL_MASK( PINSEL_PRIMARY,14 )  /* P3.14 */
        | PINSEL_MASK( PINSEL_PRIMARY,15 )  /* P3.15 */
    );

    g_pin_connect_block->reg_PINSEL[7] =
    (
          PINSEL_MASK( PINSEL_PRIMARY, 0 )  /* P3.16 */
        | PINSEL_MASK( PINSEL_PRIMARY, 1 )  /* P3.17 */
        | PINSEL_MASK( PINSEL_PRIMARY, 2 )  /* P3.18 */
        | PINSEL_MASK( PINSEL_PRIMARY, 3 )  /* P3.19 */
        | PINSEL_MASK( PINSEL_PRIMARY, 4 )  /* P3.20 */
        | PINSEL_MASK( PINSEL_PRIMARY, 5 )  /* P3.21 */
        | PINSEL_MASK( PINSEL_PRIMARY, 6 )  /* P3.22 */
        | PINSEL_MASK( PINSEL_PRIMARY, 7 )  /* P3.23 */
        | PINSEL_MASK( PINSEL_PRIMARY, 8 )  /* P3.24 */
        | PINSEL_MASK( PINSEL_PRIMARY, 9 )  /* P3.25 */
        | PINSEL_MASK( PINSEL_PRIMARY,10 )  /* P3.26 */
        | PINSEL_MASK( PINSEL_PRIMARY,11 )  /* P3.27 */
        | PINSEL_MASK( PINSEL_PRIMARY,12 )  /* P3.28 */
        | PINSEL_MASK( PINSEL_PRIMARY,13 )  /* P3.29 */
        | PINSEL_MASK( PINSEL_PRIMARY,14 )  /* P3.30 */
        | PINSEL_MASK( PINSEL_PRIMARY,15 )  /* P3.31 */
    );

    /* PORT4 */
    g_pin_connect_block->reg_PINSEL[8] =
    (
          PINSEL_MASK( PINSEL_PRIMARY, 0 )  /* P4.0  */
        | PINSEL_MASK( PINSEL_PRIMARY, 1 )  /* P4.1  */
        | PINSEL_MASK( PINSEL_PRIMARY, 2 )  /* P4.2  */
        | PINSEL_MASK( PINSEL_PRIMARY, 3 )  /* P4.3  */
        | PINSEL_MASK( PINSEL_PRIMARY, 4 )  /* P4.4  */
        | PINSEL_MASK( PINSEL_PRIMARY, 5 )  /* P4.5  */
        | PINSEL_MASK( PINSEL_PRIMARY, 6 )  /* P4.6  */
        | PINSEL_MASK( PINSEL_PRIMARY, 7 )  /* P4.7  */
        | PINSEL_MASK( PINSEL_PRIMARY, 8 )  /* P4.8  */
        | PINSEL_MASK( PINSEL_PRIMARY, 9 )  /* P4.9  */
        | PINSEL_MASK( PINSEL_PRIMARY,10 )  /* P4.10 */
        | PINSEL_MASK( PINSEL_PRIMARY,11 )  /* P4.11 */
        | PINSEL_MASK( PINSEL_PRIMARY,12 )  /* P4.12 */
        | PINSEL_MASK( PINSEL_PRIMARY,13 )  /* P4.13 */
        | PINSEL_MASK( PINSEL_PRIMARY,14 )  /* P4.14 */
        | PINSEL_MASK( PINSEL_PRIMARY,15 )  /* P4.15 */
    );

    g_pin_connect_block->reg_PINSEL[9] =
    (
          PINSEL_MASK( PINSEL_PRIMARY, 0 )  /* P4.16 */
        | PINSEL_MASK( PINSEL_PRIMARY, 1 )  /* P4.17 */
        | PINSEL_MASK( PINSEL_PRIMARY, 2 )  /* P4.18 */
        | PINSEL_MASK( PINSEL_PRIMARY, 3 )  /* P4.19 */
        | PINSEL_MASK( PINSEL_PRIMARY, 4 )  /* P4.20 */
        | PINSEL_MASK( PINSEL_PRIMARY, 5 )  /* P4.21 */
        | PINSEL_MASK( PINSEL_PRIMARY, 6 )  /* P4.22 */
        | PINSEL_MASK( PINSEL_PRIMARY, 7 )  /* P4.23 */
        | PINSEL_MASK( PINSEL_PRIMARY, 8 )  /* P4.24 */
        | PINSEL_MASK( PINSEL_PRIMARY, 9 )  /* P4.25 */
        | PINSEL_MASK( PINSEL_PRIMARY,10 )  /* P4.26 */
        | PINSEL_MASK( PINSEL_PRIMARY,11 )  /* P4.27 */
        | PINSEL_MASK( PINSEL_PRIMARY,12 )  /* P4.28 */
        | PINSEL_MASK( PINSEL_PRIMARY,13 )  /* P4.29 */
        | PINSEL_MASK( PINSEL_PRIMARY,14 )  /* P4.30 */
        | PINSEL_MASK( PINSEL_PRIMARY,15 )  /* P4.31 */
    );

    g_pin_connect_block->reg_PINSEL[10] = ~(BIT(3)) & PINSEL10_MASK;   /* disable ETM, mask reserved bit to 0 */

    /*
     * NOTE:
     *
     *  SCS controls whether the LPC2478 is set to use the legacy registers or the new
     *  fast GPIO control registers.
     *
     */

    g_scb_mmio_registers_p->reg_SCS |= BIT(0);  /* Fast GPIO / new registers */

    /*
     * Disable generation of all falling edge and rising edge interrupts for
     * GPIO ports P0 and P2:
     */
    struct gpio_interrupt_registers *gpio_interrupt_registers_p =
        (struct gpio_interrupt_registers *)GPIO_INTERRUPT_REGISTERS_BASE_ADDR;

    write_32bit_mmio_register(
        &gpio_interrupt_registers_p->gpio_port_p0.reg_IntEnF, 0x0);

    write_32bit_mmio_register(
        &gpio_interrupt_registers_p->gpio_port_p0.reg_IntEnR, 0x0);

    write_32bit_mmio_register(
        &gpio_interrupt_registers_p->gpio_port_p2.reg_IntEnF, 0x0);

    write_32bit_mmio_register(
        &gpio_interrupt_registers_p->gpio_port_p2.reg_IntEnR, 0x0);

    /*
     * Clear any leftover interrupts:
     */
    write_32bit_mmio_register(
        &gpio_interrupt_registers_p->gpio_port_p0.reg_IntClr, UINT32_MAX);
    write_32bit_mmio_register(
        &gpio_interrupt_registers_p->gpio_port_p2.reg_IntClr, UINT32_MAX);

}


void
uart_init(
    const struct uart_device *uart_p,
    uint32_t baud,
    uint8_t mode)
{
    init_uart_internal(
        (struct uart_device *)uart_p, baud, mode, UART_FIFO_OFF,
        CPU_CLOCK_FREQ_IN_HZ);
}


static void
init_uart_internal(
    const struct uart_device *uart_device_p,
    uint32_t baud,
    uint8_t mode,
    uint8_t fmode,
    uint32_t cclk)
{
    uint8_t pclk_div;
    uint8_t pclk_sel;
    uint32_t uart_divisor_latch;
    uint8_t udl_roundbit;
    fdc_error_t fdc_error;
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;
    lpc2478_uart_t *const uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;
    uint32_t uart_pclk = uart_device_p->urt_uart_pclk;
    uint32_t pclksel_index = uart_device_p->urt_pclksel_index;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(!uart_var_p->urt_initialized, 0, 0);

    turn_on_power(uart_device_p->urt_pconp_mask);

    /* find out the UART's pclk divider */
    pclk_sel = GET_PCLK_SEL(g_scb_mmio_registers_p->reg_PCLKSEL[pclksel_index], uart_pclk);
    pclk_div = ( pclk_sel == 0 ? 4 : \
                 pclk_sel == 1 ? 1 : \
                 pclk_sel == 2 ? 2 : \
                 pclk_sel == 3 ? 8 : \
                 0 ); /* this evaluation should never happen */
    if( pclk_div == 0 )
    {
        fdc_error = CAPTURE_FDC_ERROR("Invalid UART's pclk_sel", pclk_sel, 0);
        fatal_error_handler(fdc_error);
    }

    /* Calculate divisor latch value based on cclk and the UART's pclk divider
     * uart_divisor_latch = cclk/(16 * pclk_div * baud)
     * We effectively multiply by 2 to determine if it is odd or even
     * If it is even, then there is no round bit
     * If it is odd, then there is a round up
     * Shift it back */
    uart_divisor_latch = cclk/(pclk_div * baud * 8);
    udl_roundbit = ( (uart_divisor_latch & 0x1) == 0 ? 0 : 1 );
    uart_divisor_latch /= 2;
    /* TODO extra credit: use fractional dividers */

    /* stop any transmissions */
    uart_mmio_registers_p->reg_TER = 0;

    uart_mmio_registers_p->reg_LCR = ULCR_DLAB_ENABLE;
    uart_mmio_registers_p->reg_DLL = (uint8_t) uart_divisor_latch + udl_roundbit;
    uart_mmio_registers_p->reg_DLM = (uint8_t)(uart_divisor_latch >> 8);
    uart_mmio_registers_p->reg_LCR = (mode & ~ULCR_DLAB_ENABLE);

    /* Set FIFO modes and Reset */
    uart_mmio_registers_p->reg_FCR = fmode | UFCR_TXFIFO_RESET | UFCR_RXFIFO_RESET;

    /*
     * Initialize condition variables:
     */

    rtos_k_condvar_init(
        uart_device_p->urt_thre_condvar_name_p,
        cpu_id,
        &uart_device_p->urt_var_p->urt_thre_condvar);

    rtos_k_condvar_init(
        uart_device_p->urt_rdr_condvar_name_p,
        cpu_id,
        &uart_device_p->urt_var_p->urt_rdr_condvar);

    /* 
     * Register McRTOS interrupt handler
     */
    rtos_k_register_interrupt(
        &uart_device_p->urt_rtos_interrupt_params,
        uart_device_p->urt_rtos_interrupt_pp);

    DBG_ASSERT(
        *uart_device_p->urt_rtos_interrupt_pp != NULL, 
        uart_device_p->urt_rtos_interrupt_pp, uart_device_p);

    /*
     * Enable generation of UART THRE interrupts:
     */
    uint32_t reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_IER);
    reg_value |= UART_IER_THRE_INTERRUPT_ENABLE_MASK;
    write_32bit_mmio_register(&uart_mmio_registers_p->reg_IER, reg_value);

    /* 
     * Enable transmissions:
     */
    uart_mmio_registers_p->reg_TER = UTER_TXEN;

    uart_var_p->urt_initialized = true;
}


/**
 * UART interrupt pre-handler
 */ 
void
uart_interrupt_pre_handler(
    _IN_ const struct uart_device *uart_device_p)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();
    DBG_ASSERT(
        uart_device_p->urt_signature == UART_DEVICE_SIGNATURE,
        uart_device_p->urt_signature, uart_device_p);

    uint32_t reg_value;
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;
    lpc2478_uart_t *const uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;

    DBG_ASSERT(
        uart_var_p->urt_initialized, uart_device_p, 0);

    reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_IIR);

    /*
     * There is at least one interrupt pending
     * (interrupt status bit is asserted low)
     */
    FDC_ASSERT(
        (reg_value & UART_IIR_INT_STATUS_MASK) == 0,
        reg_value, uart_device_p);
    
    uart_interrupt_id_t interrupt_id =
        GET_BIT_FIELD(
            reg_value, UART_IIR_INT_ID_MASK, UART_IIR_INT_ID_SHIFT);

    uint8_t reg_lsr_value = read_8bit_mmio_register(&uart_mmio_registers_p->reg_LSR);

    switch(interrupt_id)
    {
        case UART_INT_TRANSMIT_HOLDING_REGISTER_EMPTY:
            FDC_ASSERT(
                reg_lsr_value & UART_LSR_THRE_MASK,
                reg_lsr_value, uart_device_p);

            /*
             * NOTE: Nothing explicit needs to be done to clear this interrupt,
             * as just having read the IIR above cleared this interrupt:
             */

            DBG_ASSERT(
                !uart_var_p->urt_signal_thre_condvar,
                uart_device_p, 0);

            uart_var_p->urt_signal_thre_condvar = true;
            break;

        case UART_INT_RECEIVE_DATA_AVAILABLE:
  
            FDC_ASSERT(
                reg_lsr_value & UART_LSR_RDR_MASK,
                reg_lsr_value, uart_device_p);

           /*
             * Read the first byte from the Receive FIFO to clear the
             * interrupt source. 
             */
            uart_var_p->urt_first_byte_received =
                read_8bit_mmio_register(&uart_mmio_registers_p->reg_RBR);
           
            DBG_ASSERT(
                !uart_var_p->urt_byte_received_pending,
                uart_var_p, 0);

            uart_var_p->urt_byte_received_pending = true;

            /*
             * Disable further generation of RDA interrupts, until uart_getchar() gets
             * the chance to drain the UART Rx fifo.
             */
            reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_IER);
            reg_value &= ~UART_IER_RDA_INTERRUPT_ENABLE_MASK;
            write_32bit_mmio_register(&uart_mmio_registers_p->reg_IER, reg_value);

            DBG_ASSERT(
                !uart_var_p->urt_signal_rdr_condvar,
                uart_device_p, 0);

            uart_var_p->urt_signal_rdr_condvar = true;
            break;

        case UART_INT_RECEIVE_LINE_STATUS:
            break;

        case UART_INT_CHARACTER_TIMEOUT_INDICATOR:
            break;

        default:
            FDC_ASSERT(false, interrupt_id, uart_device_p);
    }
}


/**
 * UART interrupt post-handler
 */
void
uart_interrupt_post_handler(
    _IN_ const struct uart_device *uart_device_p)
{
    DBG_ASSERT(
        uart_device_p->urt_signature == UART_DEVICE_SIGNATURE,
        uart_device_p->urt_signature, uart_device_p);

    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;

    FDC_ASSERT_RTOS_INTERRUPT_HANDLER_PRECONDITIONS(
        *uart_device_p->urt_rtos_interrupt_pp);

    if (uart_var_p->urt_signal_thre_condvar)
    {
        uart_var_p->urt_signal_thre_condvar = false;

        rtos_k_condvar_signal(&uart_var_p->urt_thre_condvar);
    }
    else if (uart_var_p->urt_signal_rdr_condvar)
    {
        uart_var_p->urt_signal_rdr_condvar = false;

        rtos_k_condvar_signal(&uart_var_p->urt_rdr_condvar);
    }
}


/* initVIC is used to
 * 1. initialize the VIC to a known state by writing all registers with their
 *    reset values
 * 2. configures system-wide
 * This is useful if only a soft-reset occurred that does not
 * perform a full reset of the hardware
 */
static void initVIC(void)
{
#   define GENERATE_ARRAY_ENTRY(_peripheral_name,                           \
                                _vic_interrupt_channel)                     \
            [_vic_interrupt_channel] =                                      \
                dummy_ ## _peripheral_name ## _isr,

    uint32_t i;

    static isr_function_t *const g_dummy_isr_functions[] =
    {
        GENERATE_ARRAY_ENTRY(wdt, VIC_CHANNEL_WDT)
        GENERATE_ARRAY_ENTRY(softint, VIC_CHANNEL_SOFTINT)
        GENERATE_ARRAY_ENTRY(armcore0, VIC_CHANNEL_ARMCORE0)
        GENERATE_ARRAY_ENTRY(armcore1, VIC_CHANNEL_ARMCORE1)
        GENERATE_ARRAY_ENTRY(timer0, VIC_CHANNEL_TIMER0)
        GENERATE_ARRAY_ENTRY(timer1, VIC_CHANNEL_TIMER1)
        GENERATE_ARRAY_ENTRY(uart0, VIC_CHANNEL_UART0)
        GENERATE_ARRAY_ENTRY(uart1, VIC_CHANNEL_UART1)
        GENERATE_ARRAY_ENTRY(pwm0_1, VIC_CHANNEL_PWM0_1)
        GENERATE_ARRAY_ENTRY(i2c0, VIC_CHANNEL_I2C0)
        GENERATE_ARRAY_ENTRY(spi_ssp0, VIC_CHANNEL_SPI_SSP0)
        GENERATE_ARRAY_ENTRY(ssp1, VIC_CHANNEL_SSP1)
        GENERATE_ARRAY_ENTRY(pll, VIC_CHANNEL_PLL)
        GENERATE_ARRAY_ENTRY(rtc, VIC_CHANNEL_RTC)
        GENERATE_ARRAY_ENTRY(eint0, VIC_CHANNEL_EINT0)
        GENERATE_ARRAY_ENTRY(eint1, VIC_CHANNEL_EINT1)
        GENERATE_ARRAY_ENTRY(eint2, VIC_CHANNEL_EINT2)
        GENERATE_ARRAY_ENTRY(eint3, VIC_CHANNEL_EINT3)
        GENERATE_ARRAY_ENTRY(ad0, VIC_CHANNEL_AD0)
        GENERATE_ARRAY_ENTRY(i2c1, VIC_CHANNEL_I2C1)
        GENERATE_ARRAY_ENTRY(bod, VIC_CHANNEL_BOD)
        GENERATE_ARRAY_ENTRY(ethernet, VIC_CHANNEL_ETHERNET)
        GENERATE_ARRAY_ENTRY(usb, VIC_CHANNEL_USB)
        GENERATE_ARRAY_ENTRY(can, VIC_CHANNEL_CAN)
        GENERATE_ARRAY_ENTRY(sd_mmc, VIC_CHANNEL_SD_MMC)
        GENERATE_ARRAY_ENTRY(gpdma, VIC_CHANNEL_GPDMA)
        GENERATE_ARRAY_ENTRY(timer2, VIC_CHANNEL_TIMER2)
        GENERATE_ARRAY_ENTRY(timer3, VIC_CHANNEL_TIMER3)
        GENERATE_ARRAY_ENTRY(uart2, VIC_CHANNEL_UART2)
        GENERATE_ARRAY_ENTRY(uart3, VIC_CHANNEL_UART3)
        GENERATE_ARRAY_ENTRY(i2c2, VIC_CHANNEL_I2C2)
        GENERATE_ARRAY_ENTRY(i2s, VIC_CHANNEL_I2S)
    };

#   undef GENERATE_ARRAY_ENTRY

    C_ASSERT2(check_g_dummy_isr_functions_array_size,
              ARRAY_SIZE(g_dummy_isr_functions) == SOC_NUM_INTERRUPT_CHANNELS);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    /* Clear any leftover pending software generated interrupts */
    g_vic_mmio_registers_p->reg_VICSoftIntClear = ALL_VIC_CHANNELS_MASK;

    /* Clear all interrupt enable bits, mask all interrupt sources */
    g_vic_mmio_registers_p->reg_VICIntEnClear = ALL_VIC_CHANNELS_MASK;

    /* Set all interrupt sources to map to IRQ */
    g_vic_mmio_registers_p->reg_VICIntSelect = ~ALL_VIC_CHANNELS_MASK;

    /* Initialize all VectAddr0..31 registers to point to dummy interrupt handlers */
    for (i = 0; i < SOC_NUM_INTERRUPT_CHANNELS; i ++)
    {
         g_vic_mmio_registers_p->reg_VICVectAddr[i] = g_dummy_isr_functions[i];

         /*
          * Ensure that the VIC priority for each interrupt channel is the
          * lowest by default
          */
         g_vic_mmio_registers_p->reg_VICVectPriority[i] = VIC_VECT_PRIORITY_LOWEST;
    }

    /* Unmask all priority levels */
    g_vic_mmio_registers_p->reg_VICSWPriorityMask = ALL_VIC_PRIORITIES_MASK;

    /* Configure VIC register access mode
     *
     * NOTE: This is a one-bit register and we are not supposed to read
     * bits 1 to 31, which are reserved
     */
    g_vic_mmio_registers_p->reg_VICProtection = VIC_ACCESS_MASK;

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/** 
 * Install an ISR into the VIC
 */
void
install_isr(
        interrupt_channel_t channel,
        isr_function_t *interruptServiceRoutine,
        interrupt_prio_t priority,
        cpu_id_t cpu_id)
{
    FDC_ASSERT(channel < SOC_NUM_INTERRUPT_CHANNELS,
        channel, SOC_NUM_INTERRUPT_CHANNELS);

    FDC_ASSERT_VALID_FUNCTION_POINTER(interruptServiceRoutine);

    FDC_ASSERT(priority < SOC_NUM_INTERRUPT_PRIORITIES,
        priority, SOC_NUM_INTERRUPT_PRIORITIES);

    FDC_ASSERT(cpu_id < SOC_NUM_CPU_CORES, cpu_id, 0);

    /*
     * Interrupt channel is currently disabled in the VIC
     */
    FDC_ASSERT_EQUAL(
        g_vic_mmio_registers_p->reg_VICIntEnable & VIC_CHANNEL_MASK(channel),
        0);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    /*
     * Select IRQ interrupt for the given channel:
     * (0 means IRQ, 1 means FIQ)
     */
    g_vic_mmio_registers_p->reg_VICIntSelect &= ~VIC_CHANNEL_MASK(channel);

    /* Install ISR into VICVectAddr# slot */
    g_vic_mmio_registers_p->reg_VICVectAddr[channel] = interruptServiceRoutine;

    /* Set Vector Priorities */
    g_vic_mmio_registers_p->reg_VICVectPriority[channel] = priority;

    /*  
     *  Enable the interrupt channel in the VIC:
     *
     *  NOTE: There is no need to read the VICIntEnable register
     *  before modifying it, as writing 0s has no effect.
     */
    write_32bit_mmio_register(
        &g_vic_mmio_registers_p->reg_VICIntEnable,
        VIC_CHANNEL_MASK(channel));

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


void
assert_interrupt_source_is_set(interrupt_channel_t interrupt_channel)
{
    FDC_ASSERT(
        interrupt_channel < SOC_NUM_INTERRUPT_CHANNELS,
        interrupt_channel, SOC_NUM_INTERRUPT_CHANNELS);

    FDC_ASSERT_EQUAL(                             
        g_vic_mmio_registers_p->reg_VICIRQStatus & VIC_CHANNEL_MASK(interrupt_channel),
        VIC_CHANNEL_MASK(interrupt_channel)); 
                                                  
    FDC_ASSERT_EQUAL(                             
        g_vic_mmio_registers_p->reg_VICRawIntr & VIC_CHANNEL_MASK(interrupt_channel),
        VIC_CHANNEL_MASK(interrupt_channel)); 
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
        interrupt_channel < SOC_NUM_INTERRUPT_CHANNELS,
        interrupt_channel, SOC_NUM_INTERRUPT_CHANNELS);

    write_32bit_mmio_register(        
        &g_vic_mmio_registers_p->reg_VICAddress, 0x0); 
} 


/* 
 * Generate dummy VIC interrupt handler functions
 */
GENERATE_DUMMY_VIC_ISR_FUNCTION(wdt, VIC_CHANNEL_WDT)
GENERATE_DUMMY_VIC_ISR_FUNCTION(softint, VIC_CHANNEL_SOFTINT)
GENERATE_DUMMY_VIC_ISR_FUNCTION(armcore0, VIC_CHANNEL_ARMCORE0)
GENERATE_DUMMY_VIC_ISR_FUNCTION(armcore1, VIC_CHANNEL_ARMCORE1)
GENERATE_DUMMY_VIC_ISR_FUNCTION(timer0, VIC_CHANNEL_TIMER0)
GENERATE_DUMMY_VIC_ISR_FUNCTION(timer1, VIC_CHANNEL_TIMER1)
GENERATE_DUMMY_VIC_ISR_FUNCTION(uart0, VIC_CHANNEL_UART0)
GENERATE_DUMMY_VIC_ISR_FUNCTION(uart1, VIC_CHANNEL_UART1)
GENERATE_DUMMY_VIC_ISR_FUNCTION(pwm0_1, VIC_CHANNEL_PWM0_1)
GENERATE_DUMMY_VIC_ISR_FUNCTION(i2c0, VIC_CHANNEL_I2C0)
GENERATE_DUMMY_VIC_ISR_FUNCTION(spi_ssp0, VIC_CHANNEL_SPI_SSP0)
GENERATE_DUMMY_VIC_ISR_FUNCTION(ssp1, VIC_CHANNEL_SSP1)
GENERATE_DUMMY_VIC_ISR_FUNCTION(pll, VIC_CHANNEL_PLL)
GENERATE_DUMMY_VIC_ISR_FUNCTION(rtc, VIC_CHANNEL_RTC)
GENERATE_DUMMY_VIC_ISR_FUNCTION(eint0, VIC_CHANNEL_EINT0)
GENERATE_DUMMY_VIC_ISR_FUNCTION(eint1, VIC_CHANNEL_EINT1)
GENERATE_DUMMY_VIC_ISR_FUNCTION(eint2, VIC_CHANNEL_EINT2)
GENERATE_DUMMY_VIC_ISR_FUNCTION(eint3, VIC_CHANNEL_EINT3)
GENERATE_DUMMY_VIC_ISR_FUNCTION(ad0, VIC_CHANNEL_AD0)
GENERATE_DUMMY_VIC_ISR_FUNCTION(i2c1, VIC_CHANNEL_I2C1)
GENERATE_DUMMY_VIC_ISR_FUNCTION(bod, VIC_CHANNEL_BOD)
GENERATE_DUMMY_VIC_ISR_FUNCTION(ethernet, VIC_CHANNEL_ETHERNET)
GENERATE_DUMMY_VIC_ISR_FUNCTION(usb, VIC_CHANNEL_USB)
GENERATE_DUMMY_VIC_ISR_FUNCTION(can, VIC_CHANNEL_CAN)
GENERATE_DUMMY_VIC_ISR_FUNCTION(sd_mmc, VIC_CHANNEL_SD_MMC)
GENERATE_DUMMY_VIC_ISR_FUNCTION(gpdma, VIC_CHANNEL_GPDMA)
GENERATE_DUMMY_VIC_ISR_FUNCTION(timer2, VIC_CHANNEL_TIMER2)
GENERATE_DUMMY_VIC_ISR_FUNCTION(timer3, VIC_CHANNEL_TIMER3)
GENERATE_DUMMY_VIC_ISR_FUNCTION(uart2, VIC_CHANNEL_UART2)
GENERATE_DUMMY_VIC_ISR_FUNCTION(uart3, VIC_CHANNEL_UART3)
GENERATE_DUMMY_VIC_ISR_FUNCTION(i2c2, VIC_CHANNEL_I2C2)
GENERATE_DUMMY_VIC_ISR_FUNCTION(i2s, VIC_CHANNEL_I2S)


/* This macro turns periods in CCLKs */
#define P2C(sdram_period_ns,emc_period_ps)      ((( (sdram_period_ns*1000) < emc_period_ps)? \
                                        0:(uint32_t)((sdram_period_ns*1000)/emc_period_ps))+1)

#define SDRAM_TRRD          15      // Row active to row active delay (min)
#define SDRAM_TRCD          20      //  RAS# to CAS# delay (min)
#define SDRAM_TRP           20      // Row Precharge time (min)
#define SDRAM_TRAS          45      // Row active time (max)
#define SDRAM_TRAS_MAX      100000  //  Row active time (max)
#define SDRAM_TRC           65      // Row cycle time (min)
#define SDRAM_TRFC          65      // tRC=tRFC (min)
#define SDRAM_TRDL_CLK      2       //  Last data in to row precharge (min)
#define SDRAM_TWR_CLK       2       // tRDL=tWR (min)
#define SDRAM_TDAL_CLK      2       // Last data in to Active delay (min)
#define SDRAM_TCDL_CLK      1       //  Last data in to new col address delay (min)
#define SDRAM_TBDL_CLK      1       //  Last data in to burst stop (min)
#define SDRAM_TCCD_CLK      1       //  Col address to col address delay (min)

#define SDRAM_REFRESH       7813
#define SDRAM_TAPR_CLK      1       // Last data out to active command time
#define SDRAM_TXSR          67      // tSREX=tSXR Self-refresh exit time
#define SDRAM_TMRD_CLK      3       // Load mode register to active command time


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-variable"
static void initSDRAM(uint32_t cclk)
{
    FDC_ASSERT_EQUAL(g_pin_connect_block, PINCON_BASE_ADDR);

    /* Reference: AN10771 Using the LPC24xx EMC peripheral to drive SDRAM
     *
     * Step 1: Configure the EMC Pin function and Pin mode
     * preserving the other bits where appropriate
     */
    g_pin_connect_block->reg_PINSEL[5] &=
    (
          PINSEL_CLR_MASK(  0 )  /* P2.16 CAS#    */
        & PINSEL_CLR_MASK(  1 )  /* P2.17 RAS#    */
        & PINSEL_CLR_MASK(  2 )  /* P2.18 CLKOUT0 */
        & PINSEL_CLR_MASK(  4 )  /* P2.20 DYCS0#  */
        & PINSEL_CLR_MASK(  8 )  /* P2.24 CKEOUT0 */
        & PINSEL_CLR_MASK( 12 )  /* P2.28 DQMOUT0 */
        & PINSEL_CLR_MASK( 13 )  /* P2.29 DQMOUT1 */
        & PINSEL_CLR_MASK( 14 )  /* P2.30 DQMOUT2 */
        & PINSEL_CLR_MASK( 15 )  /* P2.31 DQMOUT3 */
    );

    g_pin_connect_block->reg_PINSEL[5] |=
    (
          PINSEL_MASK( PINSEL_ALT_01 , 0 )  /* P2.16 CAS#    */
        | PINSEL_MASK( PINSEL_ALT_01 , 1 )  /* P2.17 RAS#    */
        | PINSEL_MASK( PINSEL_ALT_01 , 2 )  /* P2.18 CLKOUT0 */
        | PINSEL_MASK( PINSEL_ALT_01 , 4 )  /* P2.20 DYCS0#  */
        | PINSEL_MASK( PINSEL_ALT_01 , 8 )  /* P2.24 CKEOUT0 */
        | PINSEL_MASK( PINSEL_ALT_01 ,12 )  /* P2.28 DQMOUT0 */
        | PINSEL_MASK( PINSEL_ALT_01 ,13 )  /* P2.29 DQMOUT1 */
        | PINSEL_MASK( PINSEL_ALT_01 ,14 )  /* P2.30 DQMOUT2 */
        | PINSEL_MASK( PINSEL_ALT_01 ,15 )  /* P2.31 DQMOUT3 */
    );

    g_pin_connect_block->reg_PINMODE[5] &=
    (
          PINSEL_CLR_MASK(  0 )  /* P2.16 CAS#    */
        & PINSEL_CLR_MASK(  1 )  /* P2.17 RAS#    */
        & PINSEL_CLR_MASK(  2 )  /* P2.18 CLKOUT0 */
        & PINSEL_CLR_MASK(  4 )  /* P2.20 DYCS0#  */
        & PINSEL_CLR_MASK(  8 )  /* P2.24 CKEOUT0 */
        & PINSEL_CLR_MASK( 12 )  /* P2.28 DQMOUT0 */
        & PINSEL_CLR_MASK( 13 )  /* P2.29 DQMOUT1 */
        & PINSEL_CLR_MASK( 14 )  /* P2.30 DQMOUT2 */
        & PINSEL_CLR_MASK( 15 )  /* P2.31 DQMOUT3 */
    );

    g_pin_connect_block->reg_PINMODE[5] |=
    (
          PINSEL_MASK( PINMODE_PULL_NONE , 0 )  /* P2.16 CAS#    */
        | PINSEL_MASK( PINMODE_PULL_NONE , 1 )  /* P2.17 RAS#    */
        | PINSEL_MASK( PINMODE_PULL_NONE , 2 )  /* P2.18 CLKOUT0 */
        | PINSEL_MASK( PINMODE_PULL_NONE , 4 )  /* P2.20 DYCS0#  */
        | PINSEL_MASK( PINMODE_PULL_NONE , 8 )  /* P2.24 CKEOUT0 */
        | PINSEL_MASK( PINMODE_PULL_NONE ,12 )  /* P2.28 DQMOUT0 */
        | PINSEL_MASK( PINMODE_PULL_NONE ,13 )  /* P2.29 DQMOUT1 */
        | PINSEL_MASK( PINMODE_PULL_NONE ,14 )  /* P2.30 DQMOUT2 */
        | PINSEL_MASK( PINMODE_PULL_NONE ,15 )  /* P2.31 DQMOUT3 */
    );

    /* For Port3, it is only the data bus so we can ignore any previous setting */
    g_pin_connect_block->reg_PINSEL[6] =
    (
          PINSEL_MASK( PINSEL_ALT_01 , 0 )  /* P3.0  D0  */
        | PINSEL_MASK( PINSEL_ALT_01 , 1 )  /* P3.1  D1  */
        | PINSEL_MASK( PINSEL_ALT_01 , 2 )  /* P3.2  D2  */
        | PINSEL_MASK( PINSEL_ALT_01 , 3 )  /* P3.3  D3  */
        | PINSEL_MASK( PINSEL_ALT_01 , 4 )  /* P3.4  D4  */
        | PINSEL_MASK( PINSEL_ALT_01 , 5 )  /* P3.5  D5  */
        | PINSEL_MASK( PINSEL_ALT_01 , 6 )  /* P3.6  D6  */
        | PINSEL_MASK( PINSEL_ALT_01 , 7 )  /* P3.7  D7  */
        | PINSEL_MASK( PINSEL_ALT_01 , 8 )  /* P3.8  D8  */
        | PINSEL_MASK( PINSEL_ALT_01 , 9 )  /* P3.9  D9  */
        | PINSEL_MASK( PINSEL_ALT_01 ,10 )  /* P3.10 D10 */
        | PINSEL_MASK( PINSEL_ALT_01 ,11 )  /* P3.11 D11 */
        | PINSEL_MASK( PINSEL_ALT_01 ,12 )  /* P3.12 D12 */
        | PINSEL_MASK( PINSEL_ALT_01 ,13 )  /* P3.13 D13 */
        | PINSEL_MASK( PINSEL_ALT_01 ,14 )  /* P3.14 D14 */
        | PINSEL_MASK( PINSEL_ALT_01 ,15 )  /* P3.15 D15 */
    );

    g_pin_connect_block->reg_PINMODE[6] =
    (
          PINSEL_MASK( PINMODE_PULL_NONE, 0 )  /* P3.0  D0  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 1 )  /* P3.1  D1  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 2 )  /* P3.2  D2  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 3 )  /* P3.3  D3  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 4 )  /* P3.4  D4  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 5 )  /* P3.5  D5  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 6 )  /* P3.6  D6  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 7 )  /* P3.7  D7  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 8 )  /* P3.8  D8  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 9 )  /* P3.9  D9  */
        | PINSEL_MASK( PINMODE_PULL_NONE,10 )  /* P3.10 D10 */
        | PINSEL_MASK( PINMODE_PULL_NONE,11 )  /* P3.11 D11 */
        | PINSEL_MASK( PINMODE_PULL_NONE,12 )  /* P3.12 D12 */
        | PINSEL_MASK( PINMODE_PULL_NONE,13 )  /* P3.13 D13 */
        | PINSEL_MASK( PINMODE_PULL_NONE,14 )  /* P3.14 D14 */
        | PINSEL_MASK( PINMODE_PULL_NONE,15 )  /* P3.15 D15 */
    );

    g_pin_connect_block->reg_PINSEL[7] =
    (
          PINSEL_MASK( PINSEL_ALT_01 , 0 )  /* P3.16 D16 */
        | PINSEL_MASK( PINSEL_ALT_01 , 1 )  /* P3.17 D17 */
        | PINSEL_MASK( PINSEL_ALT_01 , 2 )  /* P3.18 D18 */
        | PINSEL_MASK( PINSEL_ALT_01 , 3 )  /* P3.19 D19 */
        | PINSEL_MASK( PINSEL_ALT_01 , 4 )  /* P3.20 D20 */
        | PINSEL_MASK( PINSEL_ALT_01 , 5 )  /* P3.21 D21 */
        | PINSEL_MASK( PINSEL_ALT_01 , 6 )  /* P3.22 D22 */
        | PINSEL_MASK( PINSEL_ALT_01 , 7 )  /* P3.23 D23 */
        | PINSEL_MASK( PINSEL_ALT_01 , 8 )  /* P3.24 D24 */
        | PINSEL_MASK( PINSEL_ALT_01 , 9 )  /* P3.25 D25 */
        | PINSEL_MASK( PINSEL_ALT_01 ,10 )  /* P3.26 D26 */
        | PINSEL_MASK( PINSEL_ALT_01 ,11 )  /* P3.27 D27 */
        | PINSEL_MASK( PINSEL_ALT_01 ,12 )  /* P3.28 D28 */
        | PINSEL_MASK( PINSEL_ALT_01 ,13 )  /* P3.29 D29 */
        | PINSEL_MASK( PINSEL_ALT_01 ,14 )  /* P3.30 D30 */
        | PINSEL_MASK( PINSEL_ALT_01 ,15 )  /* P3.31 D31 */
    );

    g_pin_connect_block->reg_PINMODE[7] =
    (
          PINSEL_MASK( PINMODE_PULL_NONE, 0 )  /* P3.16 D16 */
        | PINSEL_MASK( PINMODE_PULL_NONE, 1 )  /* P3.17 D17 */
        | PINSEL_MASK( PINMODE_PULL_NONE, 2 )  /* P3.18 D18 */
        | PINSEL_MASK( PINMODE_PULL_NONE, 3 )  /* P3.19 D19 */
        | PINSEL_MASK( PINMODE_PULL_NONE, 4 )  /* P3.20 D20 */
        | PINSEL_MASK( PINMODE_PULL_NONE, 5 )  /* P3.21 D21 */
        | PINSEL_MASK( PINMODE_PULL_NONE, 6 )  /* P3.22 D22 */
        | PINSEL_MASK( PINMODE_PULL_NONE, 7 )  /* P3.23 D23 */
        | PINSEL_MASK( PINMODE_PULL_NONE, 8 )  /* P3.24 D24 */
        | PINSEL_MASK( PINMODE_PULL_NONE, 9 )  /* P3.25 D25 */
        | PINSEL_MASK( PINMODE_PULL_NONE,10 )  /* P3.26 D26 */
        | PINSEL_MASK( PINMODE_PULL_NONE,11 )  /* P3.27 D27 */
        | PINSEL_MASK( PINMODE_PULL_NONE,12 )  /* P3.28 D28 */
        | PINSEL_MASK( PINMODE_PULL_NONE,13 )  /* P3.29 D29 */
        | PINSEL_MASK( PINMODE_PULL_NONE,14 )  /* P3.30 D30 */
        | PINSEL_MASK( PINMODE_PULL_NONE,15 )  /* P3.31 D31 */
    );

    g_pin_connect_block->reg_PINSEL[8] &=
    (
          PINSEL_CLR_MASK(  0 )  /* P4.0  A0  */
        & PINSEL_CLR_MASK(  1 )  /* P4.1  A1  */
        & PINSEL_CLR_MASK(  2 )  /* P4.2  A2  */
        & PINSEL_CLR_MASK(  3 )  /* P4.3  A3  */
        & PINSEL_CLR_MASK(  4 )  /* P4.4  A4  */
        & PINSEL_CLR_MASK(  5 )  /* P4.5  A5  */
        & PINSEL_CLR_MASK(  6 )  /* P4.6  A6  */
        & PINSEL_CLR_MASK(  7 )  /* P4.7  A7  */
        & PINSEL_CLR_MASK(  8 )  /* P4.8  A8  */
        & PINSEL_CLR_MASK(  9 )  /* P4.9  A9  */
        & PINSEL_CLR_MASK( 10 )  /* P4.10 A10 */
        & PINSEL_CLR_MASK( 11 )  /* P4.11 A11 */
        & PINSEL_CLR_MASK( 12 )  /* P4.12 A12 */
        & PINSEL_CLR_MASK( 13 )  /* P4.13 A13 */
        & PINSEL_CLR_MASK( 14 )  /* P4.14 A14 */
    );

    g_pin_connect_block->reg_PINSEL[8] |=
    (
          PINSEL_MASK( PINSEL_ALT_01 , 0 )  /* P4.0  A0  */
        | PINSEL_MASK( PINSEL_ALT_01 , 1 )  /* P4.1  A1  */
        | PINSEL_MASK( PINSEL_ALT_01 , 2 )  /* P4.2  A2  */
        | PINSEL_MASK( PINSEL_ALT_01 , 3 )  /* P4.3  A3  */
        | PINSEL_MASK( PINSEL_ALT_01 , 4 )  /* P4.4  A4  */
        | PINSEL_MASK( PINSEL_ALT_01 , 5 )  /* P4.5  A5  */
        | PINSEL_MASK( PINSEL_ALT_01 , 6 )  /* P4.6  A6  */
        | PINSEL_MASK( PINSEL_ALT_01 , 7 )  /* P4.7  A7  */
        | PINSEL_MASK( PINSEL_ALT_01 , 8 )  /* P4.8  A8  */
        | PINSEL_MASK( PINSEL_ALT_01 , 9 )  /* P4.9  A9  */
        | PINSEL_MASK( PINSEL_ALT_01 ,10 )  /* P4.10 A10 */
        | PINSEL_MASK( PINSEL_ALT_01 ,11 )  /* P4.11 A11 */
        | PINSEL_MASK( PINSEL_ALT_01 ,12 )  /* P4.12 A12 */
        | PINSEL_MASK( PINSEL_ALT_01 ,13 )  /* P4.13 A13 */
        | PINSEL_MASK( PINSEL_ALT_01 ,14 )  /* P4.14 A14 */
    );

    g_pin_connect_block->reg_PINMODE[8] &=
    (
          PINSEL_CLR_MASK(  0 )  /* P4.0  A0  */
        & PINSEL_CLR_MASK(  1 )  /* P4.1  A1  */
        & PINSEL_CLR_MASK(  2 )  /* P4.2  A2  */
        & PINSEL_CLR_MASK(  3 )  /* P4.3  A3  */
        & PINSEL_CLR_MASK(  4 )  /* P4.4  A4  */
        & PINSEL_CLR_MASK(  5 )  /* P4.5  A5  */
        & PINSEL_CLR_MASK(  6 )  /* P4.6  A6  */
        & PINSEL_CLR_MASK(  7 )  /* P4.7  A7  */
        & PINSEL_CLR_MASK(  8 )  /* P4.8  A8  */
        & PINSEL_CLR_MASK(  9 )  /* P4.9  A9  */
        & PINSEL_CLR_MASK( 10 )  /* P4.10 A10 */
        & PINSEL_CLR_MASK( 11 )  /* P4.11 A11 */
        & PINSEL_CLR_MASK( 12 )  /* P4.12 A12 */
        & PINSEL_CLR_MASK( 13 )  /* P4.13 A13 */
        & PINSEL_CLR_MASK( 14 )  /* P4.14 A14 */
    );

    g_pin_connect_block->reg_PINMODE[8] |=
    (
          PINSEL_MASK( PINMODE_PULL_NONE, 0 )  /* P4.0  A0  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 1 )  /* P4.1  A1  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 2 )  /* P4.2  A2  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 3 )  /* P4.3  A3  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 4 )  /* P4.4  A4  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 5 )  /* P4.5  A5  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 6 )  /* P4.6  A6  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 7 )  /* P4.7  A7  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 8 )  /* P4.8  A8  */
        | PINSEL_MASK( PINMODE_PULL_NONE, 9 )  /* P4.9  A9  */
        | PINSEL_MASK( PINMODE_PULL_NONE,10 )  /* P4.10 A10 */
        | PINSEL_MASK( PINMODE_PULL_NONE,11 )  /* P4.11 A11 */
        | PINSEL_MASK( PINMODE_PULL_NONE,12 )  /* P4.12 A12 */
        | PINSEL_MASK( PINMODE_PULL_NONE,13 )  /* P4.13 A13 */
        | PINSEL_MASK( PINMODE_PULL_NONE,14 )  /* P4.14 A14 */
    );

    g_pin_connect_block->reg_PINSEL[9] &=
    (
          PINSEL_CLR_MASK( 9 )  /* P4.25 WE# */
    );

    g_pin_connect_block->reg_PINSEL[9] |=
    (
          PINSEL_MASK( PINSEL_ALT_01 , 9 )  /* P4.25 WE# */
    );

    g_pin_connect_block->reg_PINMODE[9] &=
    (
          PINSEL_CLR_MASK( 9 ) /* P4.25 WE# */
    );

    g_pin_connect_block->reg_PINMODE[9] |=
    (
          PINSEL_MASK( PINMODE_PULL_NONE, 9 ) /* P4.25 WE# */
    );

    /*
     * Step 2: enable EMC and set EMC parameters
     */

    /* Turn on EMC power in the System Control Block */
    turn_on_power(PCONP_PCEMC);

    /* enable EMC */
    g_emc_mmio_registers_p->reg_CONTROL = EMCCONTROL_ENABLE;

    /* Configure the dynamic memory read strategy (Command delayed strategy) */
    g_emc_mmio_registers_p->reg_DYNAMICREADCONFIG = EMCDYNAMICREADCONFIG_RD_CMDDELAY;

    /* CAS latency = 3, RAS latency (active to read/write delay) = 3 */
    // TODO extra credit: is this or'd or blow away?
    g_emc_mmio_registers_p->dynamic_config[0].reg_DYNAMICRASCAS =
        EMCDYNAMICRASCAS_CAS3 | EMCDYNAMICRASCAS_RAS3;

    /* calc SDRAM period in pico-seconds) */
    uint32_t emc_period_ps = 1000000/(cclk/1000000);

    g_emc_mmio_registers_p->reg_DYNAMICRP   = P2C(SDRAM_TRP,emc_period_ps);
    g_emc_mmio_registers_p->reg_DYNAMICRAS  = P2C(SDRAM_TRAS,emc_period_ps);
    g_emc_mmio_registers_p->reg_DYNAMICSREX = P2C(SDRAM_TXSR,emc_period_ps);
    g_emc_mmio_registers_p->reg_DYNAMICAPR  = SDRAM_TAPR_CLK;
    g_emc_mmio_registers_p->reg_DYNAMICDAL  = SDRAM_TDAL_CLK+P2C(SDRAM_TRP,emc_period_ps);
    g_emc_mmio_registers_p->reg_DYNAMICWR   = SDRAM_TWR_CLK;
    g_emc_mmio_registers_p->reg_DYNAMICRC   = P2C(SDRAM_TRC,emc_period_ps);
    g_emc_mmio_registers_p->reg_DYNAMICRFC  = P2C(SDRAM_TRFC,emc_period_ps);
    g_emc_mmio_registers_p->reg_DYNAMICXSR  = P2C(SDRAM_TXSR,emc_period_ps);
    g_emc_mmio_registers_p->reg_DYNAMICRRD  = P2C(SDRAM_TRRD,emc_period_ps);
    g_emc_mmio_registers_p->reg_DYNAMICMRD  = SDRAM_TMRD_CLK;

    /* 32-bit external bus high-performance address mapping
     *  256 Mb (16Mx16bit), 4 banks, row length = 13, column length = 9 */
    g_emc_mmio_registers_p->dynamic_config[0].reg_DYNAMICCONFIG =
        EMCDYNAMICCONFIG_AM_32BIT_HP_256MB_4BANKS_R13_C9;

    /* Step 3: perform SDRAM initialization
     * JEDEC General SDRAM Initialization Sequence
       Delay to allow power and clocks to stabilize ~100 us
     */

    /* Issue SDRAM NOP (no operation) command,
     * CLKOUT runs continuously,
     * all clock enables are driven HIGH continuously */
    g_emc_mmio_registers_p->reg_DYNAMICCONTROL = EMCDYNAMICCONTROL_I_NOP | EMCDYNAMICCONTROL_CS | EMCDYNAMICCONTROL_CE; // 0x0183

    volatile uint32_t i;
    for(i = 200*30; i;i--);

    /* Issue SDRAM PALL (precharge all) command */
    g_emc_mmio_registers_p->reg_DYNAMICCONTROL = (g_emc_mmio_registers_p->reg_DYNAMICCONTROL & ~EMCDYNAMICCONTROL_I_MASK) | EMCDYNAMICCONTROL_I_PALL;

    /* n x 16 CCLKs between SDRAM refresh cycles */
    g_emc_mmio_registers_p->reg_DYNAMICREFRESH = 1; // n=1; 1x16 CCLKs between SDRAM refresh cycles

    for(i= 128; i; --i); // > 128 clk

    /* Calculate and configure SDRAM refresh time based on CCLK
     * must divide by 16 since the value is multiplied by 16 (n x 16 CCLKs)
     */
    g_emc_mmio_registers_p->reg_DYNAMICREFRESH = P2C(SDRAM_REFRESH,emc_period_ps) >> 4;

    /* Issue SDRAM Mode Command */
    g_emc_mmio_registers_p->reg_DYNAMICCONTROL =
        (g_emc_mmio_registers_p->reg_DYNAMICCONTROL & ~EMCDYNAMICCONTROL_I_MASK) |
        EMCDYNAMICCONTROL_I_MODE;

    //TODO extra credit: remove, what is this?
    //Burst 8, Sequential, CAS 2
    //volatile uint32_t Dummy = *(volatile unsigned short *)
    //                                ((uint32_t)&SDRAM_BASE + (0x33UL << (12)));

    //SHIFT_HIGH_PERFORMANCE = COL + TBW + BANK, where
    //COL=# of column bits (here: 9)
    //TBW=total bus width, 1=16 bits, 2=32 bits (here: 2)
    //BANK=# of bank select bits (here: 2)
    // --Formula courtesy of Rolf Meeser
    // Burst 4, Sequential, CAS-3

    /*
     * This is necessary for the LCD to work properly, since we will be
     * placing the LCD frame buffer in SDRAM.
     */
    volatile uint32_t Dummy = *(volatile uint32_t *)
                                    ((uint32_t)BOARD_SDRAM_BASE + (0x32UL << (13)));

    /* Issue SDRAM normal command, CLKOUT enabled, CLKOUT stop, all clock enables low */
    g_emc_mmio_registers_p->reg_DYNAMICCONTROL = 0x00000000;

    /* Buffer enabled for accesses to DCS0 chip */
    g_emc_mmio_registers_p->dynamic_config[0].reg_DYNAMICCONFIG |= EMCDYNAMICCONFIG_BUFFER_ENABLE;

    delay_loop(10000);
}
#pragma GCC diagnostic pop


static void disableSDRAM(void)
{
    /* Step 1: perform SDRAM disabled mode */

    /* Issue
     * SR=1: Enter self-refresh mode (POR reset value)
     */
    if(!(g_emc_mmio_registers_p->reg_STATUS & EMCSTATUS_SA))
    {
      g_emc_mmio_registers_p->reg_DYNAMICCONTROL = EMCDYNAMICCONTROL_SR;
    }

    /* Wait until Self-Refresh Acknowledged */
    while(!(g_emc_mmio_registers_p->reg_STATUS & EMCSTATUS_SA));

}


static uint32_t
get_peripheral_clock_freq(
    uint32_t peripheral_clock_selector_index,
    uint32_t peripheral_clock_shift)
{
    uint8_t peripheral_clock_divider;
    uint8_t peripheral_clock_selector;
    uint32_t cpu_clock_freq;
    fdc_error_t fdc_error;

    peripheral_clock_selector =
        GET_PCLK_SEL(
            g_scb_mmio_registers_p->reg_PCLKSEL[peripheral_clock_selector_index],
            peripheral_clock_shift);

    switch (peripheral_clock_selector)
    {
    case 0:
        peripheral_clock_divider = 4;
        break;

    case 1:
        peripheral_clock_divider = 1;
        break;

    case 2:
        peripheral_clock_divider = 2;
        break;

    case 3:
        peripheral_clock_divider = 8;
        break;

    default:
        fdc_error = CAPTURE_FDC_ERROR(
            "Invalid peripheral_clock_selector", peripheral_clock_selector, 0);
        fatal_error_handler(fdc_error);
    }

    cpu_clock_freq = getFcclk();
    return cpu_clock_freq / peripheral_clock_divider;
}


static void
init_timer(
    const struct timer_device *timer_device_p,
    uint32_t timer_frequency_hz,
    bool generate_interrupts
    )
{
    uint32_t prescale;
    uint32_t reg_value;
    lpc2478_timer_t *timer_mmio_registers_p = timer_device_p->tmr_mmio_registers_p;
    uint32_t timer_pclk_shift = timer_device_p->tmr_timer_pclk_shift;
    uint32_t timer_pclksel_index = timer_device_p->tmr_pclksel_index;
    vic_interrupt_channel_t vic_channel = 
        timer_device_p->tmr_rtos_interrupt_params.irp_channel;

    turn_on_power(timer_device_p->tmr_pconp_mask);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

#if 0 // XXX Remove this code
    /*  
     *  Disable timer interrupt in the VIC
     *
     *  NOTE: There is no need to read the VICIntEnClear register
     *  before modifying it, as writing 0s has no effect.
     */
    g_vic_mmio_registers_p->reg_VICIntEnClear = VIC_CHANNEL_MASK(vic_channel);
#endif

    uint32_t timer_pclksel_freq =
        get_peripheral_clock_freq(timer_pclksel_index, timer_pclk_shift);
    
    FDC_ASSERT(timer_pclksel_freq != 0, 0, 0);

    /*
     * Determine prescale amount:
     *
     * NOTE: we calculate the largest prescale, so that we can set MR0
     * to the smallest non-zero value: 1, to save a little power by having
     * the timer circuits switch fewer times.
     *
     * For example, if the timer peripheral clock frequency is 72 MHz,
     * timer_frequency_hz is 100 and TARGET_NUM_TC_INCREMENTS is 2, then
     * prescale would be:
     * 
     *      (72,000,000 / 100) / 2 = 360,000
     */
    prescale = (timer_pclksel_freq / timer_frequency_hz) / TARGET_NUM_TC_INCREMENTS;

    if (prescale == 0)
    {
        /*
         * The minimum prescale value is 1
         */
        prescale = 1;
    }

    /*
     * Program the Timer registers.
     * make sure we are not running at half the speed due to the way the
     * counters increment and match
     */

    /*
     * Disable and reset Timer and Prescale counters
     */
    reg_value = timer_mmio_registers_p->reg_TCR;
    CLEAR_BIT_FIELD(reg_value, TCR_RESERVED_MASK);
    reg_value &= ~TCR_COUNTER_ENABLE_MASK;
    reg_value |= TCR_COUNTER_RESET_MASK;
    timer_mmio_registers_p->reg_TCR = reg_value;

    /*
     * Set the Prescale register (PR)
     *
     * NOTE: We use prescale - 1 because the first time that reg_PC is
     * incremented, it is actually set to 0, not to 1. So the count 
     * kept in reg_PC is one unit behind.
     */
    
    FDC_ASSERT(prescale >= 1, prescale, 0);

    timer_mmio_registers_p->reg_PR = prescale - 1;

    /*
     * Set the timer/counter mode to Timer Mode. The prescale counter
     * register (PC) will get incremented on every rising PCLK edge, 
     * until it matches the PR register. On the next PCLK cycle after
     * reg_PC matches reg_PR, reg_PC is reset back to 0 and reg_TC is
     * incremented. Note also that the first time that reg_TC is
     * incremented, it is actually set to 0, not 1.
     */
    reg_value = timer_mmio_registers_p->reg_CTCR;
    reg_value &= ~CTCR_COUNTER_TIMER_MODE_MASK;
    reg_value |= CTCR_TIMER_MODE;
    timer_mmio_registers_p->reg_CTCR = reg_value;

    if (generate_interrupts)
    {
        /*
         * Set the value of the MR0 register and configure generation of periodic
         * interrupts for the timer when the TC register matches the value in MR0,
         * and disable all the other control functions in the MCR register
         *
         * NOTE: We use TARGET_NUM_TC_INCREMENTS - 1 because the first time that
         * reg_TC is incremented, it is actually set to 0, not to 1. So the count
         * kept in reg_TC is one unit behind.
         */
        timer_mmio_registers_p->reg_MR[0] = TARGET_NUM_TC_INCREMENTS - 1;
        timer_mmio_registers_p->reg_MCR = MCR_MR0I_MASK | MCR_MR0R_MASK;

        /*
         * Clear any old interrupt asserted by the peripheral by writing a logic one to the
         * corresponding IR bit
         */
        timer_mmio_registers_p->reg_IR |= IR_MR0_INTERRUPT_MASK;

#if 0 // XXX Remove this code
        /*  
         *  Enable the corresponding timer interrupt channel in the VIC
         *
         *  NOTE: There is no need to read the VICIntEnable register
         *  before modifying it, as writing 0s has no effect.
         */
        g_vic_mmio_registers_p->reg_VICIntEnable = VIC_CHANNEL_MASK(vic_channel);
#endif //XXX

        /*
         * Enable Timer and Prescale counters so that interrupts start coming
         */
        reg_value = timer_mmio_registers_p->reg_TCR;
        reg_value &= ~TCR_COUNTER_RESET_MASK;
        reg_value |= TCR_COUNTER_ENABLE_MASK; 
        timer_mmio_registers_p->reg_TCR = reg_value;
    }
    else
    {
        /*
         * This timer is just used as a running counter. The 32-bit Timer
         * Counter register is incremented when the prescale counter reaches
         * its terminal count. The current counter value can be read from
         * register timer_mmio_registers_p->reg_TC.
         */
        reg_value = timer_mmio_registers_p->reg_TCR;
        CLEAR_BIT_FIELD(reg_value, TCR_RESERVED_MASK);
        reg_value &= ~TCR_COUNTER_RESET_MASK;
        reg_value |= TCR_COUNTER_ENABLE_MASK; 
        timer_mmio_registers_p->reg_TCR = reg_value;
    }

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


void
clear_timer_interrupt_source(
    _IN_ const struct timer_device *timer_device_p)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();
    DBG_ASSERT(
        timer_device_p->tmr_signature == TIMER_DEVICE_SIGNATURE,
        timer_device_p->tmr_signature, timer_device_p);

    lpc2478_timer_t *timer_mmio_registers_p = timer_device_p->tmr_mmio_registers_p;
    vic_interrupt_channel_t vic_channel = 
        timer_device_p->tmr_rtos_interrupt_params.irp_channel;

    FDC_ASSERT_INTERRUPT_SOURCE_IS_SET(vic_channel);

    FDC_ASSERT_EQUAL(                             
        timer_mmio_registers_p->reg_IR & IR_MR0_INTERRUPT_MASK,
        IR_MR0_INTERRUPT_MASK); 

    /*
     * Clear interrupt condition in the timer device by writing a logic one to the
     * corresponding IR bit
     */
    timer_mmio_registers_p->reg_IR |= IR_MR0_INTERRUPT_MASK;

    FDC_ASSERT_EQUAL(                             
        g_vic_mmio_registers_p->reg_VICRawIntr & VIC_CHANNEL_MASK(vic_channel),
        0); 
}


void
timer_interrupt_handler(
    _IN_ const struct timer_device *timer_device_p)
{
    struct rtos_interrupt *timer_interrupt_p =
        *timer_device_p->tmr_rtos_interrupt_pp;

    FDC_ASSERT_RTOS_INTERRUPT_HANDLER_PRECONDITIONS(timer_interrupt_p);
    rtos_tick_timer_interrupt_handler(timer_interrupt_p);
}


static inline void
uart_putchar_internal(
    _IN_ const struct uart_device *uart_device_p, 
    _IN_ uint8_t c,
    _IN_ bool use_condvar)
{
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;
    lpc2478_uart_t *const uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;
    uint8_t reg_lsr_value;

    if (!uart_var_p->urt_initialized)
    {
        /*
         * Any call to this function from a "printf()" placed before the
         * serial port gets initialized has to be ignored.
         *
         * We can't call FDC_ASSERT or CAPTURE_APP_ERROR() here as that can
         * cause an infinite recursion, because they call this function
         * from FAILURE_PRINTF().
         */
        return;
    }

    for ( ; ; )
    {
        reg_lsr_value = read_8bit_mmio_register(&uart_mmio_registers_p->reg_LSR);
        if ((reg_lsr_value & UART_LSR_THRE_MASK) != 0)
        {
            break;
        }

        if (use_condvar)
        {
            rtos_k_condvar_wait_interrupt(&uart_var_p->urt_thre_condvar);
        }
    }

    write_8bit_mmio_register(&uart_mmio_registers_p->reg_THR, c);
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
    uart_putchar_internal(uart_device_p, c, true);
}


/**
 * Send a character over a UART serial port, doing polling until the UART Tx
 * fifo is not full. This is inefficient and should only be used only for
 * sending failure data on the serial port.
 */
void
uart_putchar_with_polling(
    _IN_ const struct uart_device *uart_device_p, 
    _IN_ uint8_t c)
{
    uart_putchar_internal(uart_device_p, c, false);
}


/**
 * Receive a character from a UART serial port
 */
uint8_t
uart_getchar(
    _IN_ const struct uart_device *uart_device_p)
{
    struct uart_device_var *const uart_var_p = uart_device_p->urt_var_p;
    lpc2478_uart_t *const uart_mmio_registers_p = uart_device_p->urt_mmio_uart_p;
    uint8_t reg_lsr_value;
    uint8_t char_received;

    FDC_ASSERT(
        uart_var_p->urt_initialized, uart_var_p, uart_device_p);

    reg_lsr_value = read_8bit_mmio_register(&uart_mmio_registers_p->reg_LSR);
    if (reg_lsr_value & UART_LSR_RDR_MASK)
    {
        char_received =
            read_8bit_mmio_register(&uart_mmio_registers_p->reg_RBR);
    }
    else
    {
        /*
         * Disable interrupts in the ARM core
         */
        cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

        /*
         * Enable or re-enable generation of UART RDA interrupts:
         */
        uint32_t reg_value = read_32bit_mmio_register(&uart_mmio_registers_p->reg_IER);
        reg_value |= UART_IER_RDA_INTERRUPT_ENABLE_MASK;
        write_32bit_mmio_register(&uart_mmio_registers_p->reg_IER, reg_value);

        /*
         * Wait to be signaled from UART interrupt:
         */
        rtos_k_condvar_wait_interrupt(&uart_var_p->urt_rdr_condvar);

        FDC_ASSERT(uart_var_p->urt_byte_received_pending,
            uart_var_p, 0);

        char_received = uart_var_p->urt_first_byte_received;
        uart_var_p->urt_byte_received_pending = false;

        /*
         * Restore previous interrupt masking in the ARM core
         */
        rtos_k_restore_cpu_interrupts(cpu_status_register);
    }

    return char_received;
}


static void
init_leds(void)
{
    configure_pin(&g_sd_mmc_led_pin, true);
    configure_pin(&g_usb_host_link_led_pin, true);
    configure_pin(&g_usb_device_link_led_pin, true);

    /*
     * Turn off LEDs
     */
    deactivate_output_pin(&g_sd_mmc_led_pin);
    deactivate_output_pin(&g_usb_host_link_led_pin);
    deactivate_output_pin(&g_usb_device_link_led_pin);
}


static inline void
toggle_led(const struct pin_config_info *pin_info_p, bool *led_is_on_p)
{
    if (*led_is_on_p)
    {
        /*
         * Turn led off
         */
        deactivate_output_pin(pin_info_p);
    }
    else
    {
        /*
         * Turn led on
         */
        activate_output_pin(pin_info_p);
    }

    *led_is_on_p ^= true;
}


void
toggle_heartbeat_led(void)
{
    static bool led_is_on = false;

    toggle_led(&g_sd_mmc_led_pin, &led_is_on);
}


void
toggle_usb_host_link_led(void)
{
    static bool led_is_on = false;

    toggle_led(&g_usb_host_link_led_pin, &led_is_on);
}


void
toggle_usb_device_link_led(void)
{
    static bool led_is_on = false;

    toggle_led(&g_usb_device_link_led_pin, &led_is_on);
}


void
init_buttons(const struct buttons_device *buttons_device_p)
{
    FDC_ASSERT(
        buttons_device_p->bd_signature == BUTTONS_DEVICE_SIGNATURE,
        buttons_device_p->bd_signature, BUTTONS_DEVICE_SIGNATURE);

    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    volatile struct gpio_port_interrupt_registers
        *gpio_port_mmio_interrupt_registers_p =
            buttons_device_p->bd_gpio_port_mmio_interrupt_registers_p;

    rtos_k_condvar_init(
        buttons_device_p->bd_condvar_name_p,
        cpu_id,
        &buttons_device_p->bd_var_p->bd_condvar);

    configure_pin(&buttons_device_p->bd_button1_pin, false);
    configure_pin(&buttons_device_p->bd_button2_pin, false);

#ifdef USE_GPIO_INTERRUPTS // a storm of EINT3 interrupts is generated.
                           // Not clear yet to fix this.
    /*
     * Clear interrupt source, before installing ISR:
     */

    write_32bit_mmio_register(
        &gpio_port_mmio_interrupt_registers_p->reg_IntClr,
        BUTTON1_PIN_MASK | BUTTON2_PIN_MASK);

    /*
     * Install ISR:
     */
    rtos_k_register_interrupt(
        &buttons_device_p->bd_rtos_interrupt_params,
        buttons_device_p->bd_rtos_interrupt_pp);

    DBG_ASSERT(
        *buttons_device_p->bd_rtos_interrupt_pp, buttons_device_p, 0);

    /*
     * Enable generation of interrupts for port GPIO_PORT_P2,
     * for detecting falling edge on pins BUTTON1_PIN_BIT_INDEX
     * and BUTTON2_PIN_BIT_INDEX:
     */

    uint32_t reg_value = read_32bit_mmio_register(
        &gpio_port_mmio_interrupt_registers_p->reg_IntEnF);

    reg_value |= (BUTTON1_PIN_MASK | BUTTON2_PIN_MASK);

    write_32bit_mmio_register(
        &gpio_port_mmio_interrupt_registers_p->reg_IntEnF, reg_value);
#endif
}


/**
 * Clear last GPIO ports interrupt
 */ 
void
clear_gpio_ports_interrupt_source(
    const struct buttons_device *buttons_device_p)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();
    DBG_ASSERT(
        buttons_device_p->bd_signature == BUTTONS_DEVICE_SIGNATURE,
        buttons_device_p->bd_signature, buttons_device_p);

    uint32_t reg_value;
    struct buttons_device_var *buttons_device_var_p =
        buttons_device_p->bd_var_p;

    volatile struct gpio_interrupt_registers
        *gpio_mmio_interrupt_registers_p =
            buttons_device_p->bd_gpio_mmio_interrupt_registers_p;

    volatile struct gpio_port_interrupt_registers
        *gpio_port_mmio_interrupt_registers_p =
            buttons_device_p->bd_gpio_port_mmio_interrupt_registers_p;

    reg_value = read_32bit_mmio_register(
                    &gpio_mmio_interrupt_registers_p->reg_IntStatus);

#if 0    
    FDC_ASSERT(
        reg_value & GPIO_INT_STATUS_P2_INT_MASK,
        reg_value, 0);
#endif
    
    reg_value = read_32bit_mmio_register(
                    &gpio_port_mmio_interrupt_registers_p->reg_IntStatF);

    uint32_t buttons_pressed = 0x0;

    if (reg_value & BUTTON1_PIN_MASK)
    {
        buttons_pressed |= BUTTON1_PRESSED_MASK;
    }

    if (reg_value & BUTTON2_PIN_MASK)
    {
        buttons_pressed |= BUTTON2_PRESSED_MASK;
    }

#if 0
    FDC_ASSERT(
        buttons_pressed != 0x0, buttons_device_p, 0);
#endif

    buttons_device_var_p->bd_buttons_pressed_bitmap = buttons_pressed;

    /*
     * Clear interrupt source:
     */
    write_32bit_mmio_register(
        &gpio_port_mmio_interrupt_registers_p->reg_IntClr, reg_value);

    /*
     * We cannot receive further interrupts of this type until read_buttons() gets
     * a chance to process the state of the buttons. So, here we just disable
     * the corresponding VIC channel, so that we don't get interrupted again
     * for this interrupt that are leaving pending.
     */
    write_32bit_mmio_register(
        &g_vic_mmio_registers_p->reg_VICIntEnClear,
        VIC_CHANNEL_MASK(buttons_device_p->bd_rtos_interrupt_params.irp_channel));
}


/**
 * GPIO ports interrupt handler
 */
void
gpio_ports_interrupt_handler(
    const struct buttons_device *buttons_device_p)
{
    DBG_ASSERT(
        buttons_device_p->bd_signature == BUTTONS_DEVICE_SIGNATURE,
        buttons_device_p->bd_signature, buttons_device_p);
   
    struct rtos_interrupt *buttons_interrupt_p =
        *buttons_device_p->bd_rtos_interrupt_pp;

    FDC_ASSERT_RTOS_INTERRUPT_HANDLER_PRECONDITIONS(buttons_interrupt_p);

    struct buttons_device_var *buttons_device_var_p =
        buttons_device_p->bd_var_p;

#if 0 // XXX
    DBG_ASSERT(
        buttons_device_var_p->bd_buttons_pressed_bitmap != 0x0,
        buttons_device_var_p, buttons_device_p);
#endif

    rtos_k_condvar_signal(
        &buttons_device_var_p->bd_condvar);
}


/**
 * Read the buttons device. It blocks the calling thread until a button is
 * pressed.
 * 
 * @return  bit map of the buttons last pressed
 */ 
uint32_t
read_buttons(
    _IN_ const struct buttons_device *buttons_device_p)
{
    FDC_ASSERT(
        buttons_device_p->bd_signature == BUTTONS_DEVICE_SIGNATURE,
        buttons_device_p->bd_signature, buttons_device_p);

    uint32_t buttons_pressed_bitmap = 0x0;

#ifdef USE_GPIO_INTERRUPTS
    do {
        /*
         * Wait for a GPIO interrupt on port GPIO_PORT_P2:
         */
        rtos_k_condvar_wait_interrupt(
            &buttons_device_p->bd_var_p->bd_condvar);

        buttons_pressed_bitmap = 
            buttons_device_p->bd_var_p->bd_buttons_pressed_bitmap;

        /*
         * Re-enable the corresponding VIC channel:
         */
        write_32bit_mmio_register(
            &g_vic_mmio_registers_p->reg_VICIntEnable,
            VIC_CHANNEL_MASK(buttons_device_p->bd_rtos_interrupt_params.irp_channel));
    } while (buttons_pressed_bitmap == 0x0);
#else

    volatile struct fast_gpio_port *fast_gpio_port_registers_p =
        &g_fast_gpio_ports_array[buttons_device_p->bd_gpio_port_index];

    uint32_t reg_value =
        read_32bit_mmio_register(&fast_gpio_port_registers_p->reg_FIOPIN);

    if ((reg_value & BUTTON1_PIN_MASK) == 0)
    {
        buttons_pressed_bitmap |= BUTTON1_PRESSED_MASK;
    }

    if ((reg_value & BUTTON2_PIN_MASK) == 0)
    {
        buttons_pressed_bitmap |= BUTTON2_PRESSED_MASK;
    }

#endif

    return buttons_pressed_bitmap;
}


static void
init_adc(const struct adc_device *adc_device_p)
{
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
}


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
    return read_adc_channel(g_adc_device_p, TRIMPOT_ADC_CHANNEL);
}


#if 0
static void
init_spi(bool master)
{
    uint32_t reg_value;

    /*
     * The SSP0 and SPI ports are mutually exclusive
     */
    FDC_ASSERT(!ssp0_connected, 0, 0);
    spi_connected = true;

    /*
     * XXX Not implemented yet
     */ 
    FDC_ASSERT(false, 0, 0);

    /*
     * Turn on SPI power in the System Control Block
     */
    turn_on_power(PCONP_PCSPI);

    uint32_t spi_pclksel_freq = get_peripheral_clock_freq(0, PCLK_SPI);
    
    if (spi_pclksel_freq == 0)
    {
        FDC_ASSERT(false, 0, 0);
        return;
    }

    /*
     * Configure pins:
     *
     * XXX Not implemented yet
     */ 
    FDC_ASSERT(false, 0, 0);
}
#endif


void
init_ssp(
    const struct ssp_controller *ssp_controller_p,
    bool master)
{
    uint32_t reg_value;

    FDC_ASSERT(
        ssp_controller_p->sc_signature == SSP_CONTROLLER_SIGNATURE,
        ssp_controller_p->sc_signature, ssp_controller_p);

    lpc2478_ssp_t *ssp_mmio_registers_p = ssp_controller_p->sc_mmio_registers_p;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    if (ssp_controller_p == &g_ssp_controllers[0])
    {
        /*
         * The SSP0 and SPI ports are mutually exclusive
         */
        FDC_ASSERT(!spi_connected, 0, 0);
        ssp0_connected = true;
    }

    struct ssp_controller_var *ssp_controller_var_p =
        ssp_controller_p->sc_var_p;

    rtos_k_condvar_init(
        ssp_controller_p->sc_condvar_name_p,
        cpu_id,
        &ssp_controller_var_p->sc_condvar);

    /*
     * Configure the SCK pin:
     */
    configure_pin(&ssp_controller_p->sc_pin_sck, true);

    if (!master)
    {
        /*
         * Configure the SSEL pin as input if the LPC2478 is an SPI slave of another chip:
         */
        configure_pin(&ssp_controller_p->sc_pin_ssel, false);
    }

    /*
     * Configure the MISO pin as input pin:
     */
    configure_pin(&ssp_controller_p->sc_pin_miso, false);

    /*
     * Configure the MOSI pin as output pin:
     */
    configure_pin(&ssp_controller_p->sc_pin_mosi, true);

    /*
     * Turn on SSP power in the System Control Block
     */
    turn_on_power(ssp_controller_p->sc_pconp_mask);

    /*
     * Clear the CR1 register before configuring other SSP registers:
     * (to ensure that the SSP controller is disabled while being configured - SSE bit is 0)
     */
    reg_value = 0;
    write_8bit_mmio_register(
        &ssp_mmio_registers_p->reg_CR1, reg_value);

    /*
     * Initialize of the CR0 register:
     * - data size 8 bits
     * - frame format SPI
     * - Clock out polarity: 0 SSP controller maintains the bus clock low between frames
     * - Clock Out Phase: 0 SSP controller captures serial data on the first clock
     *   transition of the frame
     * - Serial Clock Rate: 0 (number of prescaler-output clocks per bit on the bus, minus one)
     */
    reg_value = 0;
    SET_BIT_FIELD(reg_value, SSP_CR0_DSS_MASK, SSP_CR0_DSS_SHIFT, 8 - 1);
    SET_BIT_FIELD(reg_value, SSP_CR0_FRF_MASK, SSP_CR0_FRF_SHIFT, SSP_FRF_SPI);
    write_16bit_mmio_register(
        &ssp_mmio_registers_p->reg_CR0, reg_value);

    /*
     * For now, we don't enable generation of SSP interrupts
     */
    reg_value = 0;
    write_8bit_mmio_register(
        &ssp_mmio_registers_p->reg_IMSC, reg_value);

    /*
     * For now, we don't enable DMA for the SSP controller
     */
    reg_value = 0;
    write_16bit_mmio_register(
        &ssp_mmio_registers_p->reg_DMACR, reg_value);

    /*
     * Calculate value for the SSP clock divider:
     *
     * The APB clock (PCLK) is divided by this value plus one to produce the
     * clock for SSP port, which should be less than or equal to 4 MHz.
     *
     * ssp_pclksel_freq / (ssp_clock_divider + 1) = SSP_CLOCK_FREQUENCY
     *
     * So,
     * ssp_clock_divider = (ssp_pclk_freq / SSP_CLOCK_FREQUENCY) - 1
     */

    uint32_t ssp_pclksel_freq = get_peripheral_clock_freq(
        ssp_controller_p->sc_pclksel_index,
        ssp_controller_p->sc_ssp_pclk_shift);
    
    FDC_ASSERT(ssp_pclksel_freq != 0, ssp_controller_p, 0);

    uint8_t ssp_clock_divider = (ssp_pclksel_freq / SSP_CLOCK_FREQUENCY) - 1;
  
    /*
     * Set SSP clock frequency:
     */
    write_8bit_mmio_register(
        &ssp_mmio_registers_p->reg_CPSR_CPSDVSR, ssp_clock_divider);

    /*
     * Enable SSP controller:
     */
    reg_value = read_8bit_mmio_register(
                    &ssp_mmio_registers_p->reg_CR1);

    reg_value |= SSP_CR1_SSE_MASK;

    write_8bit_mmio_register(
        &ssp_mmio_registers_p->reg_CR1, reg_value);

    ssp_flush_transmit_receive_fifos(ssp_controller_p);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    /*
     * Install ISR before enabling interrupt source:
     */
    rtos_k_register_interrupt(
        &ssp_controller_p->sc_rtos_interrupt_params,
        ssp_controller_p->sc_rtos_interrupt_pp);

    DBG_ASSERT(
        *ssp_controller_p->sc_rtos_interrupt_pp != NULL,
        ssp_controller_p->sc_rtos_interrupt_pp, ssp_controller_p);

    /*
     * Install ISR before enabling interrupt source:
     */
    rtos_k_register_interrupt(
        &ssp_controller_p->sc_rtos_interrupt_params,
        ssp_controller_p->sc_rtos_interrupt_pp);

    DBG_ASSERT(
        *ssp_controller_p->sc_rtos_interrupt_pp != NULL,
        ssp_controller_p->sc_rtos_interrupt_pp, ssp_controller_p);

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


void
ssp_flush_transmit_receive_fifos(
    const struct ssp_controller *ssp_controller_p)
{
    uint32_t reg_value;
    lpc2478_ssp_t *ssp_mmio_registers_p = ssp_controller_p->sc_mmio_registers_p;

    /*
     * Flush transmit FIFO:
     */
    do {
        reg_value = read_8bit_mmio_register(
            &ssp_mmio_registers_p->reg_SR);
    } while (!(reg_value & SSP_SR_TFE_MASK));

    /*
     * Flush receive FIFO:
     */
    while (reg_value & SSP_SR_RNE_MASK)
    {
        (void)read_16bit_mmio_register(&ssp_mmio_registers_p->reg_DR_data);
        reg_value = read_8bit_mmio_register(&ssp_mmio_registers_p->reg_SR);
    }
}


/** 
 * Clear SSP controller interrupt source
 */
void
clear_ssp_controller_interrupt_source(
    _IN_ const struct ssp_controller *ssp_controller_p)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();
    DBG_ASSERT(
        ssp_controller_p->sc_signature == SSP_CONTROLLER_SIGNATURE,
        ssp_controller_p->sc_signature, ssp_controller_p);

        uint32_t reg_value;
    uint8_t interrupt_source;

    FDC_ASSERT(
        ssp_controller_p->sc_signature == SSP_CONTROLLER_SIGNATURE,
        ssp_controller_p->sc_signature, ssp_controller_p);

    lpc2478_ssp_t *ssp_mmio_registers_p = ssp_controller_p->sc_mmio_registers_p;

    FDC_ASSERT_INTERRUPT_SOURCE_IS_SET(ssp_controller_p->sc_vic_channel);

    /*
     * Check reason for interrupt:
     */
    reg_value = read_8bit_mmio_register(&ssp_mmio_registers_p->reg_MIS);
    if ((reg_value & (SSP_MIS_TXMIS_MASK | SSP_MIS_RXMIS_MASK)) == 0)
    {
       (void)CAPTURE_FDC_ERROR("Unexpected SSP interrupt", reg_value, ssp_controller_p);

        /*
         * Clear unexpected interrupt source:
         */
        write_8bit_mmio_register(
            &ssp_mmio_registers_p->reg_ICR, 
            SSP_ICR_RORIC_MASK | SSP_ICR_RTIC_MASK);

       return;
    }

    if (reg_value & SSP_MIS_TXMIS_MASK)
    {
        FDC_ASSERT(
            (reg_value & SSP_MIS_RXMIS_MASK) == 0,
            reg_value, ssp_controller_p);

        interrupt_source = SSP_IMSC_TXIM_MASK;
    }
    else
    {
        FDC_ASSERT(
            (reg_value & SSP_MIS_TXMIS_MASK) == 0,
            reg_value, ssp_controller_p);

        interrupt_source = SSP_IMSC_RXIM_MASK;
    }

    reg_value = read_8bit_mmio_register(&ssp_mmio_registers_p->reg_RIS);
    FDC_ASSERT(
        (reg_value & (SSP_RIS_TXRIS_MASK | SSP_RIS_RXRIS_MASK)) != 0,
        reg_value, ssp_controller_p);

    /*
     * Since we cannot clear the interrupt source at the SSP here, we disable
     * further interrupts of this type in the SSP controller. Also, we need to
     * disable the corresponding VIC channel, so that we don't get interrupted
     * again for this interrupt that are leaving pending.
     *
     * NOTE:
     * - To clear the TXIM interrupt we need to write to the Tx fifo.
     * - To clear the RXIM interrupt we need to read from the Rx fifo.
     */

    reg_value = read_8bit_mmio_register(&ssp_mmio_registers_p->reg_IMSC);

    /*
     * Clear reserved bits. Reading reserved bits is not defined, so the
     * register value read can contain garbage 1's in these bits, and we are
     * not supposed to write 1's to reserved bits.
     */ 
    CLEAR_BIT_FIELD(reg_value, SSP_IMSC_RESERVED_MASK);

    FDC_ASSERT(
        (reg_value & (SSP_IMSC_TXIM_MASK | SSP_IMSC_RXIM_MASK)) == interrupt_source,
        reg_value, interrupt_source);

    reg_value &= ~interrupt_source;
  
    write_8bit_mmio_register(
        &ssp_mmio_registers_p->reg_IMSC, reg_value);

    /*
     * Disable VIC channel:
     */
    write_32bit_mmio_register(
        &g_vic_mmio_registers_p->reg_VICIntEnClear,
        VIC_CHANNEL_MASK(ssp_controller_p->sc_rtos_interrupt_params.irp_channel));

    struct ssp_controller_var *ssp_controller_var_p =
        ssp_controller_p->sc_var_p;

    FDC_ASSERT(
        !ssp_controller_var_p->sc_interrupt_pending,
        ssp_controller_var_p, ssp_controller_p);

    ssp_controller_var_p->sc_interrupt_pending = true;
}


/** 
 * SSP controller interrupt handler
 */
void
ssp_controller_interrupt_handler(
    _IN_ const struct ssp_controller *ssp_controller_p)
{
    uint32_t reg_value;
    uint8_t interrupt_source;

    FDC_ASSERT(
        ssp_controller_p->sc_signature == SSP_CONTROLLER_SIGNATURE,
        ssp_controller_p->sc_signature, ssp_controller_p);

    struct rtos_interrupt *ssp_interrupt_p = 
        *ssp_controller_p->sc_rtos_interrupt_pp;

    FDC_ASSERT_RTOS_INTERRUPT_HANDLER_PRECONDITIONS(ssp_interrupt_p);

    /*
     * Signal this SSP's condvar:
     */

   struct ssp_controller_var *ssp_controller_var_p =
        ssp_controller_p->sc_var_p;

    FDC_ASSERT(
        ssp_controller_var_p->sc_interrupt_pending,
        ssp_controller_var_p, ssp_controller_p);

    rtos_k_condvar_signal(&ssp_controller_var_p->sc_condvar);
}


static inline void
ssp_wait_for_interrupt(
    const struct ssp_controller *ssp_controller_p, uint8_t ssp_interrupt_mask)
{
    uint32_t reg_value;
    lpc2478_ssp_t *ssp_mmio_registers_p = ssp_controller_p->sc_mmio_registers_p;

    FDC_ASSERT(
        ssp_controller_p->sc_signature == SSP_CONTROLLER_SIGNATURE,
        ssp_controller_p->sc_signature, ssp_controller_p);

    FDC_ASSERT(
        (ssp_interrupt_mask & (SSP_IMSC_TXIM_MASK | SSP_IMSC_RXIM_MASK)) == 
            SSP_IMSC_TXIM_MASK ||
        (ssp_interrupt_mask & (SSP_IMSC_TXIM_MASK | SSP_IMSC_RXIM_MASK)) == 
            SSP_IMSC_RXIM_MASK,
        ssp_interrupt_mask, ssp_controller_p);

#ifdef DEBUG
    reg_value = read_32bit_mmio_register(&g_vic_mmio_registers_p->reg_VICIntEnable);
    DBG_ASSERT(
        (reg_value & VIC_CHANNEL_MASK(ssp_controller_p->sc_vic_channel)) != 0,
        reg_value, ssp_controller_p);
#endif

    /*
     * Enable specified interrupt in the SSP controller:
     * - TXIM interrupt: asserted when the Tx FIFO is at least half empty and
     *   cleared upon the next write to the Tx FIFO.
     * - RXIM interrupt: asserted when the Rx FIFO is at least half full and
     *   cleared upon the next read form the Rx FIFO.
     */

    reg_value = read_8bit_mmio_register(&ssp_mmio_registers_p->reg_IMSC);

    /*
     * Clear reserved bits. Reading reserved bits is not defined, so the
     * register value read can contain garbage 1's in these bits, and we are
     * not supposed to write 1's to reserved bits.
     */ 
    CLEAR_BIT_FIELD(reg_value, SSP_IMSC_RESERVED_MASK);

    FDC_ASSERT(reg_value == 0, reg_value, 0);

    reg_value |= ssp_interrupt_mask;
  
    write_8bit_mmio_register(
        &ssp_mmio_registers_p->reg_IMSC, reg_value);

    /*
     * Re-enable VIC channel:
     */
    write_32bit_mmio_register(
        &g_vic_mmio_registers_p->reg_VICIntEnable,
        VIC_CHANNEL_MASK(ssp_controller_p->sc_rtos_interrupt_params.irp_channel));

    /*
     * Wait until interrupt is received:
     */

    struct ssp_controller_var *ssp_controller_var_p =
        ssp_controller_p->sc_var_p;

    while (!ssp_controller_var_p->sc_interrupt_pending)
    {
        rtos_k_condvar_wait_interrupt(&ssp_controller_var_p->sc_condvar);
    }

    ssp_controller_var_p->sc_interrupt_pending = false;
}


/**
 * Transmits a 16-bit value as two 8-bit transfers, with the high byte first.
 */
uint16_t
ssp_transmit_receive_16bit_value(
    const struct ssp_controller *ssp_controller_p,
    uint16_t outgoing_value)
{
    uint8_t outgoing_buffer[2];
    uint8_t incoming_buffer[2];
    uint16_t incoming_value;
   
    outgoing_buffer[0] = outgoing_value >> 8;
    outgoing_buffer[1] = outgoing_value & 0xff;

    ssp_transmit_receive_buffer(
        ssp_controller_p, outgoing_buffer, incoming_buffer, 2);
    
    incoming_value = incoming_buffer[0] << 8;
    incoming_value |= incoming_buffer[1];

    return incoming_value;
}


void
ssp_transmit_receive_buffer(
    const struct ssp_controller *ssp_controller_p,
    const uint8_t *outgoing_buffer, uint8_t *incoming_buffer, size_t size)
{
    uint8_t status_reg_value;
    uint16_t data_reg_value;
    lpc2478_ssp_t *ssp_mmio_registers_p = ssp_controller_p->sc_mmio_registers_p;

    FDC_ASSERT(
        outgoing_buffer != NULL && size != 0 &&
        outgoing_buffer + size > outgoing_buffer,
        outgoing_buffer, size);

    FDC_ASSERT(
        incoming_buffer == NULL ||
        incoming_buffer + size > incoming_buffer,
        incoming_buffer, size);

    status_reg_value = read_8bit_mmio_register(&ssp_mmio_registers_p->reg_SR);
    FDC_ASSERT(
        (status_reg_value & SSP_SR_TFE_MASK) != 0 && (status_reg_value & SSP_SR_RNE_MASK) == 0,
        status_reg_value , 0);

    const uint8_t *const out_buffer_end_p = outgoing_buffer + size;
    uint8_t *in_buffer_cursor_p = incoming_buffer; 

    for (const uint8_t *out_buffer_cursor_p = outgoing_buffer; 
         out_buffer_cursor_p != out_buffer_end_p; out_buffer_cursor_p ++)
    {
        /*
         * Transmit next byte:
         */ 

        for ( ; ; )
        {
            status_reg_value = read_8bit_mmio_register(&ssp_mmio_registers_p->reg_SR);
            if ((status_reg_value & SSP_SR_TNF_MASK) != 0)
            {
                break;
            }

            ssp_wait_for_interrupt(ssp_controller_p, SSP_IMSC_TXIM_MASK);
        }

        data_reg_value = *out_buffer_cursor_p;
        write_16bit_mmio_register(
            &ssp_mmio_registers_p->reg_DR_data, data_reg_value);

        /*
         * Receive next byte:
         */

        for ( ; ; )
        {
            status_reg_value = read_8bit_mmio_register(&ssp_mmio_registers_p->reg_SR);
            if ((status_reg_value & SSP_SR_RNE_MASK) != 0)
            {
                break;
            }
        
            //ssp_wait_for_interrupt(ssp_controller_p, SSP_IMSC_RXIM_MASK);
        }
   
        data_reg_value = read_16bit_mmio_register(&ssp_mmio_registers_p->reg_DR_data);

        FDC_ASSERT(data_reg_value <= UINT8_MAX, data_reg_value, 0);

        if (in_buffer_cursor_p != NULL)
        {
            *in_buffer_cursor_p = data_reg_value;
            in_buffer_cursor_p ++;
        }
    }
}


/**
 * Puts the processor in idle mode to wait for interrupts
 */
void
wait_for_interrupts(void)
{
    uint8_t reg_PCON = read_8bit_mmio_register(&g_scb_mmio_registers_p->reg_PCON);
   
    SET_SCB_PCON_IDLE_MODE(reg_PCON);

    write_8bit_mmio_register(&g_scb_mmio_registers_p->reg_PCON, reg_PCON);
}


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

