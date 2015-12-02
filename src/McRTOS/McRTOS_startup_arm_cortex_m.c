/**
 * @file McRTOS_startup_arm_cortex_m.c
 *
 * Startup code for ARM Cortex-M processors
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */
#include <McRTOS/McRTOS_arm_cortex_m.h>
#include <BoardSupport/hardware_abstractions.h>
#include <stdint.h>

#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wunused-parameter"

#define SYSTICK_COUNTER_RELOAD_VALUE \
        (CPU_CLOCK_FREQ_IN_HZ / RTOS_TICK_TIMER_FREQUENCY)

#if __MPU_PRESENT == 1
/*
 * Minimum MPU region alignment in bytes
 */
#define MIN_MPU_REGION_ALIGNMENT	UINT32_C(32)

/**
 * Minimum MPU region alignment mask
 */
#define MIN_MPU_REGION_ALIGNMENT_MASK	(~(MIN_MPU_REGION_ALIGNMENT - 1))

#endif /* __MPU_PRESENT == 1 */

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
struct cortex_m_exception_stack g_cortex_m_exception_stack __attribute__ ((section(".resetstack")));

#if __MPU_PRESENT == 1
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
#endif

static void
cortex_m_set_ccr(void)
{
    uint32_t reg_value = read_32bit_mmio_register(&SCB->CCR);

    reg_value |= SCB_CCR_UNALIGN_TRP_Msk;

#   if __CORTEX_M >= 0x03
    reg_value |= SCB_CCR_DIV_0_TRP_Msk;
#   endif

    write_32bit_mmio_register(&SCB->CCR, reg_value);
}


void
cortex_m_enable_fpu(void)
{
#ifdef __FPU_PRESENT
    uint32_t reg_value;

    /*
     * Enable full access (privileged and unprivileged access) for
     * CP10, CP11 coprocessors
     */
    reg_value = read_32bit_mmio_register(&SCB->CPACR);
    SET_BIT_FIELD(reg_value, CPACR_CP10_MASK, CPACR_CP10_SHIFT, 0x3);
    SET_BIT_FIELD(reg_value, CPACR_CP11_MASK, CPACR_CP11_SHIFT, 0x3);
    write_32bit_mmio_register(&SCB->CPACR, reg_value);

    /*
     * Set CONTROL.FPCA bit
     */
    reg_value = __get_CONTROL();
    reg_value |= CPU_REG_CONTROL_FPCA_MASK;
    __set_CONTROL(reg_value);

    __DSB();
    __ISB();
#endif
}


void
cortex_m_disable_fpu(void)
{
#ifdef __FPU_PRESENT
    uint32_t reg_value;

    __DSB();
    __ISB();

    /*
     * Clear CONTROL.FPCA bit
     */
    reg_value = __get_CONTROL();
    reg_value &= ~CPU_REG_CONTROL_FPCA_MASK;
    __set_CONTROL(reg_value);

    /*
     * Disable access to CP10, CP11 coprocessors
     */
    reg_value = read_32bit_mmio_register(&SCB->CPACR);
    SET_BIT_FIELD(reg_value, CPACR_CP10_MASK, CPACR_CP10_SHIFT, 0x0);
    SET_BIT_FIELD(reg_value, CPACR_CP11_MASK, CPACR_CP11_SHIFT, 0x0);
    write_32bit_mmio_register(&SCB->CPACR, reg_value);
#endif
}


/**
 * Initializes floating point unit
 */
static void
cortex_m_fpu_init(void)
{
#ifdef __FPU_PRESENT
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

#if 0
    /*
     * Disables automatic update of CONTROL.FPCA:
     */
    reg_value = read_32bit_mmio_register(&SCnSCB->ACTLR);
    reg_value |= SCnSCB_ACTLR_DISFPCA_Msk;
    write_32bit_mmio_register(&SCnSCB->ACTLR, reg_value);
#endif
#endif /* __FPU_PRESENT */
}


void
cortex_m_save_fpu_context(struct fpu_context *fpu_context_p)
{
#ifdef __FPU_PRESENT
    asm volatile (
	"mov		r1, %[fpu_context_p]\t\n"
	"vstmia.32	r1!, {s0-s15}\n\t"
	"vstmia.32	r1!, {s16-s31}\n\t"
	"vmrs		r2, fpscr\t\n"
	"str		r2, [r1]\t\n"
	:
	: [fpu_context_p] "r" (fpu_context_p)
	: "r1", "r2"
    );
#endif
}


void
cortex_m_restore_fpu_context(const struct fpu_context *fpu_context_p)
{
#ifdef __FPU_PRESENT
    asm volatile (
	"mov		r1, %[fpu_context_p]\t\n"
	"vldmia.32	r1!, {s0-s15}\n\t"
	"vldmia.32	r1!, {s16-s31}\n\t"
	"ldr		r2, [r1]\t\n"
	"vmsr		fpscr, r2\t\n"
	:
	: [fpu_context_p] "r" (fpu_context_p)
	: "r1", "r2"
    );
#endif
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


#if __MPU_PRESENT == 1

static bool
is_power_of_2(uint32_t value)
{
#   if __CORTEX_M >= 0x03
    uint32_t log_value = 31 - __CLZ(value);

    return (value == BIT(log_value));
#   else
    for (uint32_t log_value = 31; log_value != 0; log_value ++) {
        if (value == BIT(log_value)) {
            return true;
        }
    }

    return (value == 1);
#   endif
}


static inline uint8_t
int_log_base_2(uint32_t value)
{
    uint32_t log_value;

    FDC_ASSERT(value != 0, value, 0);
#   if __CORTEX_M >= 0x03
    log_value = 31 - __CLZ(value);
#   else
    for (log_value = 31; (value & BIT(log_value)) == 0; log_value --)
        ;
#   endif
    return (uint8_t)log_value;
}


static void
cortex_m_set_mpu_region(
    struct mpu_device_var *mpu_var_p,
    volatile MPU_Type *mpu_regs_p,
    uint8_t region_index,
    void *start_addr,
    void *end_addr,
    uint32_t unprivileged_permissions)
{
    uint32_t reg_value;

    DBG_ASSERT(region_index < mpu_var_p->num_regions,
 	       region_index, mpu_var_p->num_regions);

    DBG_ASSERT(start_addr < end_addr, start_addr, end_addr);

    size_t region_size = ((uintptr_t)end_addr - (uintptr_t)start_addr) + 1;

    DBG_ASSERT((uintptr_t)start_addr % MIN_MPU_REGION_ALIGNMENT == 0 &&
               ((uintptr_t)end_addr  + 1) % MIN_MPU_REGION_ALIGNMENT == 0,
	       start_addr, end_addr);

    DBG_ASSERT(is_power_of_2(region_size), region_size, 0);

    uint8_t encoded_region_size = int_log_base_2(region_size) - 1;

    /*
     * Configure region:
     */
    write_32bit_mmio_register(&mpu_regs_p->RNR, region_index);
    write_32bit_mmio_register(&mpu_regs_p->RBAR, (uintptr_t)start_addr);
    reg_value = MPU_RASR_ENABLE_Msk;
    SET_BIT_FIELD(reg_value, MPU_RASR_SIZE_Msk, MPU_RASR_SIZE_Pos, encoded_region_size);
    SET_BIT_FIELD(reg_value, MPU_RASR_AP_Msk, MPU_RASR_AP_Pos,
                  unprivileged_permissions &
                    (UNPRIVILEGED_READ_MASK | UNPRIVILEGED_WRITE_MASK));

    if (!(unprivileged_permissions & UNPRIVILEGED_EXEC_MASK)) {
        reg_value |= MPU_RASR_XN_Msk;
    }

    write_32bit_mmio_register(&mpu_regs_p->RASR, reg_value);
}


/*
 *
 * Set all the data regions for the current thread, including the stack region.
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
            write_32bit_mmio_register(&mpu_regs_p->RNR, region_index);
            write_32bit_mmio_register(&mpu_regs_p->RASR, 0x0);
        } else {
            uint32_t unprivileged_permissions;

            DBG_ASSERT(region_p->start_addr != NULL, i, 0);
            if (region_p->flags & MPU_REGION_READ_ONLY) {
                unprivileged_permissions = UNPRIVILEGED_READ_MASK;
            } else {
                unprivileged_permissions = UNPRIVILEGED_READ_MASK | UNPRIVILEGED_WRITE_MASK;
            }

            cortex_m_set_mpu_region(mpu_var_p, mpu_regs_p, region_index,
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
        unprivileged_permissions = UNPRIVILEGED_READ_MASK;
    } else {
        unprivileged_permissions = UNPRIVILEGED_READ_MASK | UNPRIVILEGED_WRITE_MASK;
    }

    cortex_m_set_mpu_region(mpu_var_p, mpu_regs_p, region_index,
			    start_addr, end_addr, unprivileged_permissions);
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
    write_32bit_mmio_register(&mpu_regs_p->RNR, region_index);
    write_32bit_mmio_register(&mpu_regs_p->RASR, 0x0);
}


void
mpu_register_dma_region(
    enum mpu_bus_masters dma_bus_master,
    void *start_addr,
    size_t size)
{
    DEBUG_PRINTF("DMA regions not supported for Cortex-M MPU "
                 "(dma_bus_master: %u, start_add: %p, size: %u)\n",
                 dma_bus_master, start_addr, size);
}


void
mpu_get_enclosing_region_boundaries(void *start_addr, void *end_addr,
                                    void **region_start_addr, void **region_end_addr)
{
    DBG_ASSERT(start_addr < end_addr, start_addr, end_addr);

    size_t range_size = (uintptr_t)end_addr - (uintptr_t)start_addr;
    size_t region_size = UINT32_C(1) << int_log_base_2(range_size);

    if (region_size < range_size) {
        region_size *= 2;
    }

    *region_start_addr = (void *)ROUND_DOWN((uintptr_t)start_addr, region_size);
    *region_end_addr = (void *)ROUND_UP((uintptr_t)end_addr, region_size);
}


void
cortex_m_mpu_init(void)
{
    C_ASSERT2(assert_soc_flash_base_aligned, SOC_FLASH_BASE % 32 == 0);
    C_ASSERT2(assert_soc_sram_base_aligned, SOC_SRAM_BASE % 32 == 0);
    C_ASSERT2(assert_enough_mpu_regions, RTOS_NUM_GLOBAL_MPU_REGIONS >= 1);

    extern uint32_t __flash_text_start[];
    extern uint32_t __flash_text_end[];

    uint32_t reg_value;
    void *aligned_start_addr;
    void *aligned_end_addr;

    FDC_ASSERT(
        g_mpu.signature == MPU_DEVICE_SIGNATURE,
        g_mpu.signature, 0);

    struct mpu_device_var *mpu_var_p = g_mpu.var_p;
    FDC_ASSERT(!mpu_var_p->initialized, mpu_var_p, 0);

    volatile MPU_Type *mpu_regs_p = g_mpu.mmio_regs_p;

    /*
     * Verify that the MPU has enough regions:
     */
    reg_value = read_32bit_mmio_register(&mpu_regs_p->TYPE);

    mpu_var_p->num_regions =
        GET_BIT_FIELD(reg_value, MPU_TYPE_DREGION_Msk, MPU_TYPE_DREGION_Pos);

    FDC_ASSERT(mpu_var_p->num_regions >= RTOS_MAX_MPU_REGIONS,
	       mpu_var_p->num_regions, RTOS_MAX_MPU_REGIONS);

    /*
     * Disable MPU to configure it:
     */
    reg_value = read_32bit_mmio_register(&mpu_regs_p->CTRL);
    reg_value &= ~MPU_CTRL_ENABLE_Msk;
    write_32bit_mmio_register(&mpu_regs_p->CTRL, reg_value);

    /*
     * Enable the default memory map as a background region for privileged
     * access. The background region acts as region number -1
     */
    reg_value = read_32bit_mmio_register(&mpu_regs_p->CTRL);
    reg_value |= MPU_CTRL_PRIVDEFENA_Msk;
    write_32bit_mmio_register(&mpu_regs_p->CTRL, reg_value);

    /*
     * Make region 0, the code in flash to be executable in unprivileged mode
     */
    mpu_get_enclosing_region_boundaries(__flash_text_start, __flash_text_end,
                                        &aligned_start_addr, &aligned_end_addr);

    for (cpu_id_t cpu_id = 0; cpu_id < SOC_NUM_CPU_CORES; cpu_id ++) {
	cortex_m_set_mpu_region(mpu_var_p, mpu_regs_p, 0,
			        aligned_start_addr,
			        aligned_end_addr,
			        UNPRIVILEGED_READ_MASK |
                                UNPRIVILEGED_EXEC_MASK);
    }

    mpu_var_p->num_defined_global_regions ++;

    /*
     * Set remaining regions as invalid to save power
     */
    for (uint_fast8_t region_index = 2; region_index < mpu_var_p->num_regions;
         region_index ++) {
        write_32bit_mmio_register(&mpu_regs_p->RNR, region_index);
        write_32bit_mmio_register(&mpu_regs_p->RASR, 0x0);
    }

    /*
     * Enable MPU:
     */
    reg_value = read_32bit_mmio_register(&mpu_regs_p->CTRL);
    reg_value |= MPU_CTRL_ENABLE_Msk;
    write_32bit_mmio_register(&mpu_regs_p->CTRL, reg_value);
    mpu_var_p->initialized = true;
    capture_fdc_msg_printf("Cortex-M MPU initialized (regions: %u)\n", g_mpu.var_p->num_regions);
}

#endif /* __MPU_PRESENT == 1 */

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


/**
 * Sends an inter-processor interrupt to the given CPU
 */
void
send_inter_processor_interrupt(cpu_id_t cpu_id)
{
    DEBUG_PRINTF("Not implemented yet\n");
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


