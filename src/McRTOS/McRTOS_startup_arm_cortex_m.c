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

/*
 * Coprocessor CP10 and CP11 fields in the SCB->CPACR register:
 */
#define CPACR_CP10_MASK		MULTI_BIT_MASK(21, 20)
#define CPACR_CP10_SHIFT	20
#define CPACR_CP11_MASK		MULTI_BIT_MASK(23, 22)
#define CPACR_CP11_SHIFT	22

static void copy_initialized_data_section_from_flash_to_ram(void);
static void zero_fill_uninitialized_data_section(void);

extern void main(void);

/**
 * Pointer to the McRTOS interrupt object for the tick timer interrupt.
 */
struct rtos_interrupt *g_rtos_interrupt_systick_p = NULL;

/**
 * Exception handlers stack shared among all nested exception handlers
 */
struct cortex_m_exception_stack
    g_cortex_m_exception_stack __attribute__ ((section(".resetstack")));


static void
cortex_m_set_ccr(void)
{
    uint32_t reg_value = read_32bit_mmio_register(&SCB->CCR);

    reg_value |= (SCB_CCR_UNALIGN_TRP_Msk |
		  SCB_CCR_DIV_0_TRP_Msk);

    write_32bit_mmio_register(&SCB->CCR, reg_value);
}


void
cortex_m_enable_fpu(void)
{
    uint32_t reg_value = read_32bit_mmio_register(&SCB->CPACR);

    /* set CP10, CP11 Full Access */
    SET_BIT_FIELD(reg_value, CPACR_CP10_MASK, CPACR_CP10_SHIFT, 0x3);
    SET_BIT_FIELD(reg_value, CPACR_CP11_MASK, CPACR_CP11_SHIFT, 0x3);

    write_32bit_mmio_register(&SCB->CPACR, reg_value);
    __DSB();
    __ISB();
}


void
cortex_m_disable_fpu(void)
{
    __DSB();
    __ISB();
    uint32_t reg_value = read_32bit_mmio_register(&SCB->CPACR);

    SET_BIT_FIELD(reg_value, CPACR_CP10_MASK, CPACR_CP10_SHIFT, 0x0);
    SET_BIT_FIELD(reg_value, CPACR_CP11_MASK, CPACR_CP11_SHIFT, 0x0);

    write_32bit_mmio_register(&SCB->CPACR, reg_value);
}


/**
 * Initializes floating point unit
 */
static void
cortex_m_fpu_init(void)
{
    uint32_t reg_value;

    /**
     * FPU is initialized as disabled upon reset
     */
    reg_value = read_32bit_mmio_register(&SCB->CPACR);

    FDC_ASSERT((reg_value & (CPACR_CP10_MASK|CPACR_CP10_MASK)) == 0,
	       reg_value, 0);

    /*
     * Configure FPCCR:
     * - Disable automatic saving of floating point registers on exception entry
     * - Disable automatic lazy stacking of floating point registers
     */
    reg_value = read_32bit_mmio_register(&FPU->FPCCR);

    reg_value &= ~(FPU_FPCCR_ASPEN_Msk | FPU_FPCCR_LSPEN_Msk);
    write_32bit_mmio_register(&FPU->FPCCR, reg_value);
}


void
cortex_m_save_fpu_context(struct fpu_context *fpu_context_p)
{
    asm volatile (
	"mov		r0, %[fpu_context_p]\t\n"
	"vstmia.32	r0!, {s0-s15}\n\t"
	"vstmia.32	r0!, {s16-s31}\n\t"
	"vmrs		r1, fpscr\t\n"
	"str		r1, [r0]\t\n"
	:
	: [fpu_context_p] "r" (fpu_context_p)
	: "r0", "r1"
    );
}


void
cortex_m_restore_fpu_context(const struct fpu_context *fpu_context_p)
{
    asm volatile (
	"mov		r0, %[fpu_context_p]\t\n"
	"vldmia.32	r0!, {s0-s15}\n\t"
	"vldmia.32	r0!, {s16-s31}\n\t"
	"ldr		r1, [r0]\t\n"
	"vmsr		fpscr, r1\t\n"
	:
	: [fpu_context_p] "r" (fpu_context_p)
	: "r0", "r1"
    );
}


/**
 * Reset exception handler
 */
void
cortex_m_reset_handler(void)
{
    soc_early_init();
    copy_initialized_data_section_from_flash_to_ram();
    zero_fill_uninitialized_data_section();

    /*
     * NOTE: the stack markers cannot be initialized at
     * compile-time as the stack is not in the .data section
     */
    g_cortex_m_exception_stack.es_stack_overflow_marker = RTOS_STACK_OVERFLOW_MARKER;
    g_cortex_m_exception_stack.es_stack_underflow_marker = RTOS_STACK_UNDERFLOW_MARKER;
    for (uint32_t i = 0; i < RTOS_INTERRUPT_STACK_NUM_ENTRIES; ++ i) {
        g_cortex_m_exception_stack.es_stack[i] = RTOS_STACK_UNUSED_SIGNATURE;
    }

    cortex_m_set_ccr();
    cortex_m_fpu_init();

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
copy_initialized_data_section_from_flash_to_ram(void)
{
	extern uint32_t __flash_initialized_data_start[];
	extern uint32_t __ram_initialized_data_start[];
	extern uint32_t __ram_initialized_data_end[];

        uint32_t *src_word_p = __flash_initialized_data_start;
        uint32_t *dest_word_p = __ram_initialized_data_start;

        do {
            *dest_word_p++ = *src_word_p++;
        } while (dest_word_p != __ram_initialized_data_end);
}

/**
 * Zero out uninitialized global and static variables
 */
static void
zero_fill_uninitialized_data_section(void)
{
        extern uint32_t __uninitialized_data_start[];
	extern uint32_t __uninitialized_data_end[];

        uint32_t *word_p = __uninitialized_data_start;

        do {
            *word_p++ = 0;
        } while (word_p != __uninitialized_data_end);
}

/**
 * Initializes the Cortex-M Memory Protection Unit (MPU) if available.
 * It returns true if MPU is present
 */
bool
cortex_m_mpu_init(void)
{
#if __MPU_PRESENT == 1
    uint32_t reg_value =
        read_32bit_mmio_register((volatile uint32_t *)&MPU->TYPE);

    uint32_t num_data_regions =
        GET_BIT_FIELD(reg_value, MPU_TYPE_DREGION_Msk, MPU_TYPE_DREGION_Pos);

    FDC_ASSERT(
        num_data_regions == 0x0 || num_data_regions == 0x8,
        num_data_regions, 0);

    return (num_data_regions != 0x0);
#else
    return false;
#endif
}


/**
 * Initialize NVIC
 */
void
cortex_m_nvic_init(void)
{
#   ifdef _RELIABILITY_CHECKS_
    uint32_t reg_value;

    /*
     * Check that the vector table pointer registers points to address 0x0
     */
    reg_value = read_32bit_mmio_register(&SCB->VTOR);
    FDC_ASSERT(reg_value == 0x0, reg_value, 0);
#   endif

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    for (IRQn_Type irq_num = 0; irq_num < SOC_NUM_INTERRUPT_CHANNELS; irq_num ++) {
	    /* Clear any leftover pending interrupts */
	    NVIC_ClearPendingIRQ(irq_num);

	    /* Clear all interrupt enable bits to disable all interrupt sources */
	    NVIC_DisableIRQ(irq_num);
    }

    /*
     * Set the priority of the PendSV exception to the highest priority, since
     * PendSV is used for synchronous context switches, and we want to prevent
     * all interrupts from preempting us during a synchronous context switch:
     */
    install_isr(
        VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PendableSrvReq),
        cortex_m_pendsv_exception_handler,
        SOC_HIGHEST_INTERRUPT_PRIORITY,
        SOC_GET_CURRENT_CPU_ID());

    /*
     * Set the priority of the SVC exception to the highest priority also,
     * to ensure the SVC exception handler is not interrupted.
     */
    install_isr(
        VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SVCall),
        cortex_m_svc_exception_handler,
        SOC_HIGHEST_INTERRUPT_PRIORITY,
        SOC_GET_CURRENT_CPU_ID());

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);
}

#pragma GCC diagnostic push

#ifndef _RELIABILITY_CHECKS_
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

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
    IRQn_Type irq_number = (IRQn_Type)channel;
    uint32_t vector_number = IRQ_NUMBER_TO_VECTOR_NUMBER(irq_number);

#   ifdef _RELIABILITY_CHECKS_
    /*
     * Since for the Cortex-M all ISRs are installed at compile-time,
     * we don't install the ISR here, but we can check it
     */
    extern const isr_function_t *const g_interrupt_vector_table[];
    isr_function_t *installed_isr_p = g_interrupt_vector_table[vector_number];

    FDC_ASSERT(
        installed_isr_p == interruptServiceRoutine,
        installed_isr_p, interruptServiceRoutine);

    FDC_ASSERT_VALID_FUNCTION_POINTER(interruptServiceRoutine);

    FDC_ASSERT(priority < SOC_NUM_INTERRUPT_PRIORITIES,
        priority, SOC_NUM_INTERRUPT_PRIORITIES);

    FDC_ASSERT(cpu_id < SOC_NUM_CPU_CORES, cpu_id, 0);
#   endif

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
            vector_number == INT_PendableSrvReq ||
            vector_number == INT_SVCall,
            vector_number, interruptServiceRoutine);

        /*
         * Clear any pending interrupt:
         *
         * NOTE: Writing 0s to other bits of SCB_ICSR has no effect
         */
        if (vector_number == INT_SysTick) {
            write_32bit_mmio_register(
                &SCB->ICSR, SCB_ICSR_PENDSTCLR_Msk);
        } else if (vector_number == INT_PendableSrvReq) {
            write_32bit_mmio_register(
                &SCB->ICSR, SCB_ICSR_PENDSVCLR_Msk);
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

#pragma GCC diagnostic pop

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
        .irp_priority = SYSTICK_INTERRUPT_PRIORITY,
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
        &SysTick->LOAD, SYSTICK_COUNTER_RELOAD_VALUE - 1);

    write_32bit_mmio_register(
        &SysTick->VAL, 0);

    write_32bit_mmio_register(
        &SysTick->CTRL,
        SysTick_CTRL_ENABLE_Msk |
        SysTick_CTRL_TICKINT_Msk |
        SysTick_CTRL_CLKSOURCE_Msk);
}


/**
 * Puts the processor in idle mode to wait for interrupts
 */
void
wait_for_interrupts(void)
{
    __WFI();
}


/*
 * SVC exception handler. It is used to transition from unprivileged
 * thread mode to privileged thread mode, as part of implementing system
 * calls.
 *
 * void
 * cortex_m_svc_exception_handler(void)
 *
 * @pre     This exception was triggered from thread mode.
 *
 * @param   none.
 *
 * @return  none
 *
 * NOTE: The SVC exception has the highest priority, so its handler cannot be
 * preempted by any other interrupt with configurable priority.
 */
void
cortex_m_svc_exception_handler(void)
{
    cpu_register_t reg_value;

    /*
     * Clear nPRIV bit in the CPU control register to stay in privileged mode
     * upon return from the exception
     */
    reg_value = __get_CONTROL();
    reg_value &= ~CPU_REG_CONTROL_nPRIV_MASK;
    __set_CONTROL(reg_value);
    __ISB();
}


void cortex_m_nmi_isr(void)
{
    TODO("Not implemented yet");
    __BKPT(0);
}


