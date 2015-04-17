/**
 * @file McRTOS_arm_cortex_m.h
 *
 * ARM Cortex-M declarations
 *
 * @author German Rivera
 */

#ifndef __McRTOS_ARM_CORTEX_M_H
#define __McRTOS_ARM_CORTEX_M_H

#include "McRTOS_internals.h"
#include "McRTOS_kernel_services.h"

/**
 * Exception handlers stack
 */
struct __cortex_m_exception_stack
{
    /**
     * Stack overflow buffer, to be initialized to RTOS_STACK_OVERFLOW_BUFFER_SIGNATURE
     */
    rtos_execution_stack_entry_t es_stack_overflow_buffer
        [RTOS_STACK_OVERFLOW_BUFFER_SIZE_IN_ENTRIES];

    /**
     * Stack overflow sentinel, to be initialized to RTOS_STACK_OVERFLOW_MARKER
     */
    rtos_execution_stack_entry_t es_stack_overflow_marker;

    /**
     * Actual stack
     */
    rtos_execution_stack_entry_t es_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES];

    /**
     * Stack underflow sentinel, to be initialized to RTOS_STACK_UNDERFLOW_MARKER
     */
    rtos_execution_stack_entry_t es_stack_underflow_marker;
};

struct cortex_m_exception_stack {
    struct __cortex_m_exception_stack;
} __attribute__ ((aligned(SOC_MPU_REGION_ALIGNMENT(struct __cortex_m_exception_stack))));

C_ASSERT(sizeof(struct cortex_m_exception_stack) % SOC_MPU_REGION_ALIGNMENT(struct __cortex_m_exception_stack) == 0);

void cortex_m_reset_handler(void);
bool cortex_m_mpu_present(void);
void cortex_m_mpu_init(void);
void cortex_m_enable_fpu(void);
void cortex_m_disable_fpu(void);
void cortex_m_save_fpu_context(struct fpu_context *fpu_context_p);
void cortex_m_restore_fpu_context(const struct fpu_context *fpu_context_p);
void cortex_m_nvic_init(void);
void cortex_m_systick_init(void);
void cortex_m_nmi_isr(void);
void cortex_m_hard_fault_exception_handler(void);
void cortex_m_memory_management_exception_handler(void);
void cortex_m_bus_fault_exception_handler(void);
void cortex_m_usage_fault_exception_handler(void);
void cortex_m_svc_exception_handler(void);
void cortex_m_debug_monitor_exception_handler(void);
void cortex_m_pendsv_exception_handler(void);
void cortex_m_systick_isr(void);
void cortex_m_trigger_pendsv_exception(void);

extern struct cortex_m_exception_stack g_cortex_m_exception_stack;

#endif /* __McRTOS_ARM_CORTEX_M_H */
