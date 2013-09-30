/**
 * @file McRTOS_startup_arm_cortex_m.c
 *
 * Startup code for ARM Cortex-M processors
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */  
#include "McRTOS_arm_cortex_m.h"
#include "hardware_abstractions.h"
#include <stdint.h>

#define SYSTICK_COUNTER_RELOAD_VALUE \
        (CPU_CLOCK_FREQ_IN_HZ / RTOS_TICK_TIMER_FREQUENCY)

static void copy_data_section_initializers_to_ram(void);
static void zero_fill_bss(void);

extern void main(void);

/**
 * Pointer to the McRTOS interrupt object for the tick timer interrupt.
 */
struct rtos_interrupt *g_rtos_interrupt_systick_p = NULL;

/**
 * CPU cycles counter
 */
static uint32_t g_cpu_cycles_count = 0;

/**
 * Exception handlers stack shared among all nested exception handlers
 */
struct cortex_m_exception_stack 
    g_cortex_m_exception_stack __attribute__ ((section(".resetstack")));

/**
 * Reset exception handler
 */
void
cortex_m_reset_handler(void)
{
    /*
     * Disable the Watchdog because it will cause reset unless we have
     * refresh logic in place for the watchdog
     */
    write_32bit_mmio_register(&SIM_COPC, 0x0);

    copy_data_section_initializers_to_ram();
    zero_fill_bss();

    /*
     * NOTE: the reset stack markers cannot be initialized at
     * compile-time as it is not in the .data section
     */
    g_cortex_m_exception_stack.es_stack_overflow_marker = RTOS_STACK_OVERFLOW_MARKER;
    g_cortex_m_exception_stack.es_stack_underflow_marker = RTOS_STACK_UNDERFLOW_MARKER;

    main();

    /*
     * should never get here
     */ 
    __BKPT(0);
}


/* 
 * Copy data section initializers from Flash to SRAM
 */ 
static void
copy_data_section_initializers_to_ram(void)
{
	extern uint32_t _etext[];
	extern uint32_t _sdata[];
	extern uint32_t _edata[];
        
        uint32_t *src_word_p = _etext;
        uint32_t *dest_word_p = _sdata;

        do {
            *dest_word_p++ = *src_word_p++;
        } while (dest_word_p != _edata);
}

/**
 * Zero out uninitialized global and static variables
 */
static void
zero_fill_bss(void)
{
        extern uint32_t __start_bss[];
	extern uint32_t __end_bss[];
        
        uint32_t *word_p = __start_bss;

        do {
            *word_p++ = 0;
        } while (word_p != __end_bss);
}


/**
 * Initialize NVIC
 */
void cortex_m_nvic_init(void)
{
    /*
     * Check that the vector table pointer registers points to address 0x0
     */ 
    uint32_t reg_value = SCB_VTOR;
    FDC_ASSERT(reg_value == 0x0, reg_value, 0);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    /* Clear any leftover pending interrupts */
    write_32bit_mmio_register(&NVIC_ICPR, UINT32_MAX); 

    /* Clear all interrupt enable bits, mask all interrupt sources */
    write_32bit_mmio_register(&NVIC_ICER, UINT32_MAX); 

    /*
     * Set to the lowest priority the priority of the PendSV exception, which is
     * used for context switches:
     */
    install_isr(
        VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PendableSrvReq),
        cortex_m_pendsv_exception_handler,
        SOC_LOWEST_INTERRUPT_PRIORITY,
        SOC_GET_CURRENT_CPU_ID());

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/** 
 * Configure an ISR in the NVIC, for an external interrupt, and in the SCB
 * for internal interrupts.
 */
void
install_isr(
        interrupt_channel_t channel,
        isr_function_t *interruptServiceRoutine,
        interrupt_prio_t priority,
        cpu_id_t cpu_id)
{
    extern const isr_function_t *const g_interrupt_vector_table[];
    IRQn_Type irq_number = (IRQn_Type)channel;
    uint32_t vector_number = IRQ_NUMBER_TO_VECTOR_NUMBER(irq_number);

    /*
     * Since for the Cortex-M all ISRs are installed at compile-time,
     * we don't install the ISR here, but we can check it
     */
    isr_function_t *installed_isr_p = g_interrupt_vector_table[vector_number];

    FDC_ASSERT(
        installed_isr_p == interruptServiceRoutine,
        installed_isr_p, interruptServiceRoutine);

    FDC_ASSERT_VALID_FUNCTION_POINTER(interruptServiceRoutine);

    FDC_ASSERT(priority < SOC_NUM_INTERRUPT_PRIORITIES,
        priority, SOC_NUM_INTERRUPT_PRIORITIES);

    FDC_ASSERT(cpu_id < SOC_NUM_CPU_CORES, cpu_id, 0);

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    if (irq_number < 0) {
        /*
         * Internal interrupt
         *
         * NOTE: this function only supports INT_SysTick and INT_PendableSrvReq
         */
        FDC_ASSERT(
            vector_number == INT_SysTick ||
            vector_number == INT_PendableSrvReq,
            vector_number, interruptServiceRoutine);

        /*
         * Clear any pending interrupt:
         * 
         * NOTE: Writing 0s to other bits of SCB_ICSR has no effect
         */
        if (vector_number == INT_SysTick) {
            write_32bit_mmio_register(
                &SCB_ICSR, SCB_ICSR_PENDSTCLR_Msk);
        } else {
            write_32bit_mmio_register(
                &SCB_ICSR, SCB_ICSR_PENDSVCLR_MASK);
        }

        /*
         * NOTE: NVIC_SetPriority() handles negative IRQ numbers
         */
        NVIC_SetPriority(irq_number, priority);
    } else {
        /*
         * External interrupt
         */
        FDC_ASSERT(channel < SOC_NUM_INTERRUPT_CHANNELS,
            channel, SOC_NUM_INTERRUPT_CHANNELS);

        NVIC_SetPriority(irq_number, priority);
        NVIC_ClearPendingIRQ(irq_number);
        NVIC_EnableIRQ(irq_number);
    }

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}


/**
 * Initializes the RTOS tick timer for the calling CPU core
 */ 
void
initialize_tick_timer(void)
{
    static const struct rtos_interrupt_registration_params rtos_interrupt_params = 
    {
        .irp_name_p = "McRTOS Tick Timer Interrupt",
        .irp_isr_function_p = cortex_m_systick_isr,
        .irp_arg_p =  NULL,
        .irp_channel = VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SysTick),
        .irp_priority = SOC_HIGHEST_INTERRUPT_PRIORITY + 1,
        .irp_cpu_id = 0,
    };

    rtos_k_register_interrupt(
        &rtos_interrupt_params,
        &g_rtos_interrupt_systick_p);

    /**
     * Initialize system tick timer to start generating interrupts:
     *
     * NOTE: We cannot use SysTick_Config() because it changes again
     * the priority of the interrupt, which we already set in the 
     * rtos_k_register_interrupt() call above.
     */
    write_32bit_mmio_register(
        &SYST_RVR, SYSTICK_COUNTER_RELOAD_VALUE - 1);

    write_32bit_mmio_register(
        &SYST_CVR, 0);

    write_32bit_mmio_register(
        &SYST_CSR,
        SysTick_CSR_ENABLE_MASK |
        SysTick_CSR_TICKINT_MASK |
        SysTick_CSR_CLKSOURCE_MASK);
}


uint32_t 
get_cpu_clock_cycles(void)
{
    uint32_t reg_value = read_32bit_mmio_register(&SYST_CVR);
    uint32_t delta_cycles = SYSTICK_COUNTER_RELOAD_VALUE - reg_value;

    reg_value = read_32bit_mmio_register(&SYST_CSR);
    if (reg_value & SysTick_CSR_COUNTFLAG_MASK)
    {
        delta_cycles += SYSTICK_COUNTER_RELOAD_VALUE;
    }

    g_cpu_cycles_count += delta_cycles;
    return g_cpu_cycles_count;
}


/**
 * Puts the processor in idle mode to wait for interrupts
 */
void
wait_for_interrupts(void)
{
    __WFI();
}


void cortex_m_nmi_isr(void)
{
    TODO("Not implemented yet");
    __BKPT(0); 
}


void cortex_m_svc_handler(void)
{
    TODO("Not implemented yet");
    __BKPT(0); 
}

