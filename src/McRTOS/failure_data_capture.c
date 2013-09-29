/**
 * @file failure_data_capture.c
 *
 * Failure data capture support functions
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 

#include <stdint.h>
#include "failure_data_capture.h"
#include "hardware_abstractions.h"
#include "arm_defs.h"
#include "McRTOS_kernel_services.h"
#include "McRTOS_internals.h"
#include "utils.h"

/**
 * Macro that prints a failure message on the serial port in product code,
 * or to a CppUtest log for unit test code.
 */

#ifdef CPPUTEST_COMPILATION  // from CppUTest

#   define FAILURE_PRINTF(_fmt, ...) \
            cpputest_printf(                                                \
                "FDC: " _fmt, ##__VA_ARGS__)
#else
    /**
     * NOTE: we cannot use console_printf() here because FAILURE_PRINTF()
     * needs to be able to always send output to serial port.
     */ 
#   define FAILURE_PRINTF(_fmt, ...) \
            embedded_printf(                                                \
                (putchar_func_t *)uart_putchar_with_polling,                \
                (void *)g_console_serial_port_p,                            \
                "FDC: " _fmt, ##__VA_ARGS__)

#endif /* CPPUTEST_COMPILATION */


static void capture_unexpected_exception_failure(
    enum unexpected_exception_types exception_type,
    void *location,
    uintptr_t arg,
    cpu_status_register_t cpu_status_register);

/**
 * Check if the CPU is running in little-endian mode
 */
bool 
is_cpu_little_endian(void)
{ 
    static const union
    { 
        uint16_t value; 
        uint8_t bytes[2]; 
    } half_word =
    { 
        .value = 0x00ff 
    }; 

    if (half_word.bytes[0] == 0xff)
    { 
        /*
         * little endian 
         */ 
        FDC_ASSERT(half_word.bytes[1] == 0x00, half_word.bytes[1], 0); 
        return true; 
    }
    else
    { 
        /*
         * big endian 
         */ 
        FDC_ASSERT(half_word.bytes[0] == 0x00, half_word.bytes[0], 0); 
        FDC_ASSERT(half_word.bytes[1] == 0xff, half_word.bytes[1], 0); 
        return false; 
    } 
} 
 

/**
 * Function that actually captures a failure data record. This function is
 * to be invoked only from the rtos_capture_failure_data() system call.
 */
void
rtos_k_capture_failure_data(
    const char *failure_str,
    uintptr_t arg1,
    uintptr_t arg2,
    void *failure_location)
{
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    cpu_status_register_t cpu_sr;
    bool interrupts_enabled;
    bool restore_interrupts = false;

#   if DEFINED_ARM_CLASSIC_ARCH()
        CAPTURE_ARM_CPSR_REGISTER(cpu_sr);
        interrupts_enabled =
            CPU_INTERRUPTS_ARE_ENABLED(cpu_sr);
#   elif DEFINED_ARM_CORTEX_M_ARCH()
        interrupts_enabled =
            CPU_INTERRUPTS_ARE_ENABLED(__get_PRIMASK());
#   endif

    /*
     * Only call rtos_k_disable_cpu_interrupts() if interrupts are currently
     * enabled:
     *
     * NOTE: This check is necessary to avoid an unwanted recursion if an assert
     * fails in rtos_stop_interrupts_disabled_time_measure()
     */
    if (interrupts_enabled)
    {
        cpu_sr = rtos_k_disable_cpu_interrupts();
        restore_interrupts = true;
    }

    struct failure_record *failure =
        fdc_info_p->fdc_failures +
        fdc_info_p->fdc_failure_cursor;

    failure->fr_failure_location = failure_location;

    failure->fr_seq_number =
        fdc_info_p->fdc_failures_count;

    failure->fr_failure_descrption_str = failure_str;
    failure->fr_failure_args[0] = arg1;
    failure->fr_failure_args[1] = arg2;

    failure->fr_cpu_status_register = cpu_sr;
    failure->fr_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    fdc_info_p->fdc_failure_cursor ++;
    if (fdc_info_p->fdc_failure_cursor == RTOS_MAX_NUM_FAILURE_RECORDS)
    {
        fdc_info_p->fdc_failure_cursor = 0;
    }

    fdc_info_p->fdc_failures_count ++;

    /*
     * Print failure message to serial port:
     */ 
    {
        static bool failure_being_printed = false;

        /*
         * Avoid recursive printing:
         */
        if (!failure_being_printed)
        {
            failure_being_printed = true;

            FAILURE_PRINTF(
                "Failure captured: %s (code location: %#x, "
                "arg1: %#x, arg2: %#x, execution context: %#p)\n",
                failure_str, failure_location, arg1, arg2,
                cpu_controller_p->cpc_current_execution_context_p);

            failure_being_printed = false;
        }
    }

    if (restore_interrupts)
    {
        rtos_k_restore_cpu_interrupts(cpu_sr);
    }
}


/**
 * Captures failure data for an assertion failure
 */ 
void
capture_assert_failure(
    const char *cond_str,
    uintptr_t arg1,
    uintptr_t arg2)
{
    uint32_t *return_address;
    uint32_t *assert_address;

    /*
     * Capture ARM LR register on entry
     */ 
    CAPTURE_ARM_LR_REGISTER(return_address);

    /*
     * The exact location of the assertion is the place where this
     * function was invoked
     */ 
    assert_address = return_address - 1;

    rtos_capture_failure_data(
        cond_str,
        arg1,
        arg2,
        assert_address);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    if (fdc_info_p->fdc_asserts_failures_breakpoint_on)
    {
        ARTIFICIAL_BREAK_POINT();
    } 
}


/**
 * Capture failure data for a runtime error
 */
fdc_error_t
capture_fdc_error(
    const char *error_description,
    uintptr_t arg1,
    uintptr_t arg2)
{
    uint32_t *return_address;
    uint32_t *error_address;

    /*
     * Capture ARM LR register on entry
     */ 
    CAPTURE_ARM_LR_REGISTER(return_address);
    
    error_address = return_address - 1;

    rtos_capture_failure_data(
        error_description,
        arg1,
        arg2,
        error_address);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    if (fdc_info_p->fdc_error_breakpoint_on)
    {
        ARTIFICIAL_BREAK_POINT();
    }

    return (fdc_error_t)error_address;
}


void
check_isr_reset_entry_asserts(void)
{
#if DEFINED_ARM_CLASSIC_ARCH()
    uint32_t currentCpsr;
    
    CAPTURE_ARM_CPSR_REGISTER(currentCpsr);

    FDC_ASSERT_EQUAL(
        currentCpsr & ARM_MODE_MASK, ARM_MODE_SVC);

    FDC_ASSERT_EQUAL(
        currentCpsr & ARM_INTERRUPTS_DISABLED_MASK,
        ARM_INTERRUPTS_DISABLED_MASK);

#elif DEFINED_ARM_CORTEX_M_ARCH()
    uint32_t current_ipsr = __get_IPSR();
    uint32_t current_control = __get_CONTROL();

    FDC_ASSERT(
        CPU_MODE_IS_THREAD(current_ipsr), current_ipsr, 0);

    FDC_ASSERT(
        CPU_USING_MSP_STACK_POINTER(current_control), 0, 0);

    FDC_ASSERT(
        CPU_MODE_IS_PRIVILEGED(current_control), 0, 0);

    FDC_ASSERT(
        CPU_INTERRUPTS_ARE_ENABLED(__get_PRIMASK()), 0, 0);
#else
#   error "CPU architecture not supported"
#endif
}


#ifdef _RELIABILITY_CHECKS_

/**
 * Check the invariants of a McRTOS execution context
 */
void
check_rtos_execution_context_invariants(
    _IN_ const struct rtos_execution_context *rtos_execution_context_p)
{
    FDC_ASSERT(
        rtos_execution_context_p->ctx_signature == RTOS_EXECUTION_CONTEXT_SIGNATURE,
        rtos_execution_context_p->ctx_signature, rtos_execution_context_p);
  
    rtos_execution_stack_entry_t *stack_top_end_p =
        rtos_execution_context_p->ctx_execution_stack_top_end_p;

    rtos_execution_stack_entry_t *stack_bottom_end_p =
        rtos_execution_context_p->ctx_execution_stack_bottom_end_p;

    if (rtos_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
        struct rtos_thread *rtos_thread_p = 
            RTOS_EXECUTION_CONTEXT_GET_THREAD(rtos_execution_context_p);
        
        FDC_ASSERT(
            rtos_thread_p->thr_signature == RTOS_THREAD_SIGNATURE,
            rtos_thread_p->thr_signature, rtos_thread_p);
       
        FDC_ASSERT(
            rtos_thread_p->thr_current_priority < RTOS_NUM_THREAD_PRIORITIES &&
            rtos_thread_p->thr_current_priority <= rtos_thread_p->thr_base_priority,
            rtos_thread_p->thr_current_priority, rtos_thread_p->thr_base_priority);
  
        FDC_ASSERT(
            rtos_thread_p->thr_time_slice_ticks_left <= RTOS_THREAD_TIME_SLICE_IN_TICKS,
            rtos_thread_p->thr_time_slice_ticks_left, rtos_thread_p);

        FDC_ASSERT(
            rtos_execution_context_p->ctx_cpu_mode == RTOS_UNPRIVILEGED_THREAD_MODE
            ||
            rtos_execution_context_p->ctx_cpu_mode == RTOS_PRIVILEGED_THREAD_MODE,
            rtos_execution_context_p->ctx_cpu_mode, rtos_execution_context_p);

        if (rtos_thread_p->thr_state == RTOS_THREAD_RUNNING)
        {
            struct rtos_cpu_controller *cpu_controller_p =
                &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

            DBG_ASSERT(
                rtos_thread_p == cpu_controller_p->cpc_current_thread_p,
                rtos_thread_p, cpu_controller_p->cpc_current_thread_p);
        }

#ifdef DEBUG
        struct rtos_thread_execution_stack *thread_execution_stack_p =
            rtos_thread_p->thr_execution_stack_p;

        DBG_ASSERT(
            thread_execution_stack_p != NULL, 0, 0);

        DBG_ASSERT(
            stack_top_end_p == thread_execution_stack_p->tes_stack,
            stack_top_end_p, thread_execution_stack_p->tes_stack);

        DBG_ASSERT(
            stack_bottom_end_p == 
                &thread_execution_stack_p->tes_stack[RTOS_THREAD_STACK_NUM_ENTRIES],
            stack_bottom_end_p, 
            &thread_execution_stack_p->tes_stack[RTOS_THREAD_STACK_NUM_ENTRIES]);
#endif
    }
    else if (rtos_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT)
    {
        struct rtos_interrupt *rtos_interrupt_p = 
            RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(rtos_execution_context_p);

        FDC_ASSERT(
            rtos_interrupt_p->int_signature == RTOS_INTERRUPT_SIGNATURE,
            rtos_interrupt_p->int_signature, rtos_interrupt_p);
       
        FDC_ASSERT(
            rtos_execution_context_p->ctx_cpu_mode == RTOS_INTERRUPT_MODE,
            rtos_execution_context_p->ctx_cpu_mode, rtos_execution_context_p);

#       if DEFINED_ARM_CLASSIC_ARCH()
        DBG_ASSERT(
            stack_top_end_p == rtos_interrupt_p->int_stack,
            stack_top_end_p, rtos_interrupt_p->int_stack);

        DBG_ASSERT(
            stack_bottom_end_p ==
                &rtos_interrupt_p->int_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES],
            stack_bottom_end_p, 
            &rtos_interrupt_p->int_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES]);

#       elif DEFINED_ARM_CORTEX_M_ARCH()

        DBG_ASSERT(
            stack_top_end_p == g_cortex_m_exception_stack.es_stack,
            stack_top_end_p, g_cortex_m_exception_stack.es_stack);

        DBG_ASSERT(
            stack_bottom_end_p ==
                &g_cortex_m_exception_stack.es_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES],
            stack_bottom_end_p, 
            &g_cortex_m_exception_stack.es_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES]);

#       else
#           error "CPU architecture not supported"
#       endif
    }
    else
    {
        FDC_ASSERT(
            rtos_execution_context_p->ctx_context_type == RTOS_RESET_CONTEXT,
            rtos_execution_context_p->ctx_context_type,
            rtos_execution_context_p);

        FDC_ASSERT(
            rtos_execution_context_p->ctx_cpu_mode == RTOS_RESET_MODE,
            rtos_execution_context_p->ctx_cpu_mode, rtos_execution_context_p);

#       if DEFINED_ARM_CORTEX_M_ARCH()
        DBG_ASSERT(
            stack_top_end_p == g_cortex_m_exception_stack.es_stack,
            stack_top_end_p, g_cortex_m_exception_stack.es_stack);

        DBG_ASSERT(
            stack_bottom_end_p ==
                &g_cortex_m_exception_stack.es_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES],
            stack_bottom_end_p, 
            &g_cortex_m_exception_stack.es_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES]);

#       endif
    }

    FDC_ASSERT(
        *(stack_top_end_p - 1) == RTOS_STACK_OVERFLOW_MARKER,
        *(stack_top_end_p - 1), rtos_execution_context_p);

    FDC_ASSERT(
        *stack_bottom_end_p == RTOS_STACK_UNDERFLOW_MARKER,
        *stack_bottom_end_p, rtos_execution_context_p);
}


/**
 * Check last saved CPU registers for this execution context:
 */
#if DEFINED_ARM_CLASSIC_ARCH()
void
check_rtos_execution_context_cpu_registers(
    _IN_ const struct rtos_execution_context *rtos_execution_context_p)
{
    uint32_t *return_address;
    uint32_t *caller_address;

    CAPTURE_ARM_LR_REGISTER(return_address);
    caller_address = return_address - 1;
    
    FDC_ASSERT(
        rtos_execution_context_p->ctx_signature == RTOS_EXECUTION_CONTEXT_SIGNATURE,
        rtos_execution_context_p->ctx_signature, rtos_execution_context_p);
  
    rtos_execution_stack_entry_t *stack_top_end_p =
        rtos_execution_context_p->ctx_execution_stack_top_end_p;

    rtos_execution_stack_entry_t *stack_bottom_end_p =
        rtos_execution_context_p->ctx_execution_stack_bottom_end_p;

    cpu_register_t cpu_pc_register =
        rtos_execution_context_p->ctx_cpu_registers[CPU_REG_PC];

    uint32_t arm_cpu_mode = (cpu_status_register & ARM_MODE_MASK);

    cpu_register_t cpu_sp_register =
        rtos_execution_context_p->ctx_cpu_registers[CPU_REG_SP];

    cpu_status_register_t cpu_status_register =
        rtos_execution_context_p->ctx_cpu_registers[CPU_REG_CPSR];

    /*
     * For classic ARM, McRTOS does not support applications using Thumb mode
     */
        FDC_ASSERT(
            (cpu_status_register & T_BIT) == 0,
            cpu_status_register, T_BIT);

    if (rtos_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
        FDC_ASSERT(
            arm_cpu_mode == ARM_MODE_USER || arm_cpu_mode == ARM_MODE_SYS,
            arm_cpu_mode, rtos_execution_context_p);
        
        if (arm_cpu_mode == ARM_MODE_USER)
        {
            FDC_ASSERT(
                CPU_INTERRUPTS_ARE_ENABLED(cpu_status_register),
                cpu_status_register, rtos_execution_context_p);
        }
    }
    else
    {
        FDC_ASSERT(
            arm_cpu_mode == ARM_MODE_SYS,
            arm_cpu_mode, rtos_execution_context_p);
    }

    FDC_ASSERT(
        cpu_sp_register % ARM_CPU_WORD_SIZE_IN_BYTES == 0,
        cpu_sp_register, rtos_execution_context_p);

    FDC_ASSERT(
        (rtos_execution_stack_entry_t *)cpu_sp_register >= stack_top_end_p,
        cpu_sp_register, stack_top_end_p);

    FDC_ASSERT(
        (rtos_execution_stack_entry_t *)cpu_sp_register <= stack_bottom_end_p,
        cpu_sp_register, stack_bottom_end_p);

    FDC_ASSERT_VALID_CODE_ADDRESS(cpu_pc_register, rtos_execution_context_p);

    /*
     * NOTE: We cannot assert anything about LR, as the compiler may have generated
     * code to use LR as a scratch register in the middle of a routine
     */
}

#elif DEFINED_ARM_CORTEX_M_ARCH()

void
check_rtos_execution_context_cpu_registers(
    _IN_ const struct rtos_execution_context *rtos_execution_context_p)
{
    cpu_register_t cpu_sp_register;

    FDC_ASSERT(
        rtos_execution_context_p->ctx_signature == RTOS_EXECUTION_CONTEXT_SIGNATURE,
        rtos_execution_context_p->ctx_signature, rtos_execution_context_p);
  
    rtos_execution_stack_entry_t *stack_top_end_p =
        rtos_execution_context_p->ctx_execution_stack_top_end_p;

    rtos_execution_stack_entry_t *stack_bottom_end_p =
        rtos_execution_context_p->ctx_execution_stack_bottom_end_p;

    if (rtos_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
        cpu_sp_register =
            rtos_execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp;
    }
    else
    {
        cpu_sp_register =
            rtos_execution_context_p->ctx_cpu_saved_registers.cpu_reg_msp;
    }

    uint32_t *stack_p = (uint32_t *)cpu_sp_register;
    cpu_status_register_t cpu_status_register = stack_p[CPU_REG_PSR];
    cpu_register_t cpu_pc_register = stack_p[CPU_REG_PC];

    if (rtos_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
        FDC_ASSERT(
            (cpu_status_register & CPU_REG_IPSR_EXCEPTION_NUMBER_MASK) == 0,
            cpu_status_register, rtos_execution_context_p);
    }

    FDC_ASSERT(
        cpu_sp_register % ARM_CPU_WORD_SIZE_IN_BYTES == 0,
        cpu_sp_register, rtos_execution_context_p);

    FDC_ASSERT(
        (rtos_execution_stack_entry_t *)cpu_sp_register >= stack_top_end_p,
        cpu_sp_register, stack_top_end_p);

    FDC_ASSERT(
        (rtos_execution_stack_entry_t *)cpu_sp_register <= stack_bottom_end_p,
        cpu_sp_register, stack_bottom_end_p);

    FDC_ASSERT_VALID_CODE_ADDRESS(cpu_pc_register, rtos_execution_context_p);

    /*
     * NOTE: We cannot assert anything about LR, as the compiler may have generated
     * code to use LR as a scratch register in the middle of a routine
     */
}

#else
#   error "CPU architecture not supported"
#endif

#endif /* _RELIABILITY_CHECKS_ */


/**
 * Captures a trace for a context switch and checks the preconditions for
 * moving to a new execution context
 */ 
void
fdc_trace_rtos_context_switch(
    _IN_ const struct rtos_execution_context *target_execution_context_p)
{
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(current_execution_context_p);
    DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(target_execution_context_p);

#   if DEFINED_ARM_CLASSIC_ARCH()
    /*
     * Validate Context switch assumptions:
     */
    cpu_status_register_t actual_cpsr;                                       
    CAPTURE_ARM_CPSR_REGISTER(actual_cpsr);

    uint32_t actual_arm_mode = (actual_cpsr & ARM_MODE_MASK);

    DBG_ASSERT(
        CPU_INTERRUPTS_ARE_DISABLED(actual_cpsr),
        actual_cpsr, target_execution_context_p);

    DBG_ASSERT(
        actual_arm_mode == ARM_MODE_SVC ||
        actual_arm_mode == ARM_MODE_IRQ ||
        actual_arm_mode == ARM_MODE_SYS,
        actual_arm_mode, target_execution_context_p);

    cpu_status_register_t target_cpsr =
        target_execution_context_p->ctx_cpu_registers[CPU_REG_CPSR];

    uint32_t target_arm_mode = (target_cpsr & ARM_MODE_MASK);

    DBG_ASSERT(
        target_arm_mode == ARM_MODE_USER ||
        target_arm_mode == ARM_MODE_SYS,
        target_arm_mode, target_execution_context_p);

    /*
     * If the target cpu mode is user mode, interrupts are enabled in the
     * target context. However, if the target cpu mode is system mode,
     * interrupts may or may not be enabled. They disabled in case of
     * synchronous context switches (i.e. threads getting blocked on a mutex
     * or condvar).
     */
    if (target_arm_mode == ARM_MODE_USER)
    {
        DBG_ASSERT(
            CPU_INTERRUPTS_ARE_ENABLED(target_cpsr),
            target_cpsr, target_execution_context_p);
    }

#   elif DEFINED_ARM_CORTEX_M_ARCH()

    cpu_status_register_t actual_primask = __get_PRIMASK();
    
    cpu_status_register_t actual_ipsr = __get_IPSR();

    cpu_status_register_t actual_control_reg = __get_CONTROL();

    DBG_ASSERT(
        CPU_INTERRUPTS_ARE_DISABLED(actual_primask),
        actual_primask, target_execution_context_p);

    DBG_ASSERT(
        (CPU_MODE_IS_INTERRUPT(actual_ipsr) ||
         CPU_MODE_IS_PENDSV_EXCEPTION(actual_ipsr) ||
         CPU_MODE_IS_HARD_FAULT_EXCEPTION(actual_ipsr)) &&
         CPU_MODE_IS_PRIVILEGED(actual_control_reg) &&
         CPU_USING_MSP_STACK_POINTER(actual_control_reg),
        actual_ipsr, actual_control_reg);

    uint32_t *target_stack_p;

    if (target_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT) {
        target_stack_p =
            (uint32_t *)target_execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp;
    } else {
        target_stack_p =
            (uint32_t *)target_execution_context_p->ctx_cpu_saved_registers.cpu_reg_msp;
    }
        
    cpu_status_register_t target_psr = target_stack_p[CPU_REG_PSR];
    
    DBG_ASSERT(
        CPU_MODE_IS_THREAD(target_psr) ||
        CPU_MODE_IS_INTERRUPT(target_psr),
        target_psr, target_execution_context_p);

    /*
     * If the target cpu mode is unprivileged thread mode, interrupts are
     * enabled in the target context. However, if the target cpu mode is
     * privileged thread mode, interrupts may or may not be enabled. They 
     * disabled in case of synchronous context switches (i.e. threads getting
     * blocked on a mutex or condvar).
     */

#   else
#       error "CPU architecture not supported"
#   endif

    /*
     * Capture context switch trace:
     */

    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    fdc_context_switch_trace_entry_t trace_entry =
        target_execution_context_p->ctx_prefilled_trace_entry;

    SET_BIT_FIELD(
        trace_entry, 
        FDC_CST_LAST_SWITCHED_OUT_REASON_MASK,
        FDC_CST_LAST_SWITCHED_OUT_REASON_SHIFT,
        target_execution_context_p->ctx_last_switched_out_reason);

    if (target_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
        struct rtos_thread *target_thread_p =
            RTOS_EXECUTION_CONTEXT_GET_THREAD(target_execution_context_p);

        DBG_ASSERT(
            target_thread_p->thr_state == RTOS_THREAD_RUNNING,
            target_thread_p->thr_state, target_thread_p);

        SET_BIT_FIELD(
            trace_entry, 
            FDC_CST_CONTEXT_PRIORITY_MASK,
            FDC_CST_CONTEXT_PRIORITY_SHIFT,
            target_thread_p->thr_current_priority);
    }
    else
    {
        DBG_ASSERT(
            target_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            target_execution_context_p->ctx_context_type, target_execution_context_p);
          
        struct rtos_interrupt *target_interrupt_p =
            RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(target_execution_context_p);

        interrupt_channel_t interrupt_channel = target_interrupt_p->int_channel;

        fdc_info_p->fdc_interrupt_channel_counters[interrupt_channel] ++;
    }

#   if DEFINED_ARM_CLASSIC_ARCH()
    cpu_status_register_t current_cpsr =
        current_execution_context_p->ctx_cpu_registers[CPU_REG_CPSR];

    SET_BIT_FIELD(
        trace_entry, 
        FDC_CST_CURRENT_CPU_MODE_MASK,
        FDC_CST_CURRENT_CPU_MODE_SHIFT,
        current_cpsr & 0xf);

    SET_BIT_FIELD(
        trace_entry, 
        FDC_CST_TARGET_CPU_MODE_MASK,
        FDC_CST_TARGET_CPU_MODE_SHIFT,
        target_cpsr & 0xf);

#   elif DEFINED_ARM_CORTEX_M_ARCH()

    uint32_t *current_stack_p;

    if (current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT) {
        current_stack_p =
            (uint32_t *)current_execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp;
    } else {
        current_stack_p =
            (uint32_t *)current_execution_context_p->ctx_cpu_saved_registers.cpu_reg_msp;
    }
        
    cpu_status_register_t current_psr = current_stack_p[CPU_REG_PSR];

    if (CPU_MODE_IS_THREAD(current_psr)) {      
        trace_entry &= ~FDC_CST_CURRENT_CPU_MODE_MASK;
    } else {
        trace_entry |= FDC_CST_CURRENT_CPU_MODE_MASK;
    }

    trace_entry &= ~FDC_CST_CURRENT_LR_EXC_SP_MODE_MASK;

    if (CPU_MODE_IS_THREAD(target_psr)) {      
        trace_entry &= ~FDC_CST_TARGET_CPU_MODE_MASK;
    } else {
        trace_entry |= FDC_CST_TARGET_CPU_MODE_MASK;
    }

    if (target_execution_context_p->
            ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry !=
        CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP) {
        trace_entry &= ~FDC_CST_TARGET_LR_EXC_SP_MODE_MASK;
    } else {
        trace_entry |= FDC_CST_TARGET_LR_EXC_SP_MODE_MASK;
    }

#   else
#       error "CPU architecture not supported"
#   endif

    fdc_info_p->fdc_context_switch_trace_buffer[
        fdc_info_p->fdc_context_switch_trace_cursor] = trace_entry;
                                                     
    fdc_info_p->fdc_context_switch_trace_cursor ++;
    if (fdc_info_p->fdc_context_switch_trace_cursor ==  
        RTOS_NUM_CONTEXT_SWITCH_TRACE_BUFFER_ENTRIES) {          
        fdc_info_p->fdc_context_switch_trace_cursor = 0; 
    }                                              
                                                  
    fdc_info_p->fdc_context_switch_count ++;
}


#if DEFINED_ARM_CLASSIC_ARCH()

void capture_unexpected_undefined_instruction(
    void *location, uintptr_t arg, cpu_status_register_t cpsr)
{
    capture_unexpected_exception_failure(
        UET_UNDEFINED_INSTRUCTION, location, arg, cpsr);
}


void capture_unexpected_data_abort(
    void *location, uintptr_t arg, cpu_status_register_t cpsr)
{
    capture_unexpected_exception_failure(
        UET_DATA_ABORT, location, arg, cpsr);
}


void capture_unexpected_prefetch_abort(
    void *location, uintptr_t arg, cpu_status_register_t cpsr)
{
    capture_unexpected_exception_failure(
        UET_PREFETCH_ABORT, location, arg, cpsr);
}

#elif DEFINED_ARM_CORTEX_M_ARCH()

void capture_unexpected_hard_fault(
    void *location, uintptr_t arg, uint32_t psr)
{
    capture_unexpected_exception_failure(
        UET_HARD_FAULT, location, arg, psr);
}

#else
#   error "CPU architecture not supported"
#endif

static void
capture_unexpected_exception_failure(
    enum unexpected_exception_types exception_type,
    void *location,
    uintptr_t arg,
    cpu_status_register_t cpu_status_register)
{
    static const char *exception_type_strings[] =
    {
#   if DEFINED_ARM_CLASSIC_ARCH()
      [UET_UNDEFINED_INSTRUCTION] = "Undefined Instruction",
      [UET_DATA_ABORT] =            "Data Abort",
      [UET_PREFETCH_ABORT] =        "Prefetch Abort"

#   elif DEFINED_ARM_CORTEX_M_ARCH()
      [UET_HARD_FAULT] = "Hard Fault"

#else
#   error "CPU architecture not supported"
#   endif
    };

    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();

#   if DEFINED_ARM_CLASSIC_ARCH()
    DBG_ASSERT(
        exception_type >= UET_UNDEFINED_INSTRUCTION &&
        exception_type <= UET_PREFETCH_ABORT, exception_type, 0);
#   endif

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    struct unexpected_exception_record *exception_failure =
        fdc_info_p->fdc_unexpected_exceptions +
        fdc_info_p->fdc_unexpected_exceptions_cursor;

    exception_failure->uer_exception_type = exception_type;
    exception_failure->uer_location = location;
    exception_failure->uer_arg = arg;
    exception_failure->uer_cpu_status_register = cpu_status_register;
    exception_failure->uer_execution_context_p = 
        cpu_controller_p->cpc_current_execution_context_p;

    exception_failure->uer_seq_number =
        fdc_info_p->fdc_unexpected_exceptions_count;

    fdc_info_p->fdc_unexpected_exceptions_cursor ++;

    if (fdc_info_p->fdc_unexpected_exceptions_cursor == 
        RTOS_MAX_NUM_UNEXPECTED_EXCEPTION_RECORDS)
    {
        fdc_info_p->fdc_unexpected_exceptions_cursor = 0;
    }

    fdc_info_p->fdc_unexpected_exceptions_count ++;

    FAILURE_PRINTF(
        "Unexpected exception: %s "
        "(code location: %#x, arg: %#x, cpu status register: %#x, "
        "execution context: %#p)\n",
        exception_type_strings[exception_type],
        location, arg, cpu_status_register,
        cpu_controller_p->cpc_current_execution_context_p);
}


/**
 * Fatal error handler
 */
void fatal_error_handler(fdc_error_t fdc_error)
{
    cpu_status_register_t cpu_sr = rtos_k_disable_cpu_interrupts();

    FAILURE_PRINTF("\n*** FATAL ERROR: %x ***\n", fdc_error);

    /*
     * Hold the processor in an infinite tight loop.
     * Keep interrupts disabled to block execution of all
     * threads and interrupt handlers.
     */
    for ( ; ; )
    {
        // XXX make the LEDs blink in a distinctive pattern
    }

    rtos_k_restore_cpu_interrupts(cpu_sr);
}


void
check_rtos_public_kernel_service_preconditions(bool thread_callers_only)
{
#   if DEFINED_ARM_CLASSIC_ARCH()
    cpu_status_register_t cpu_status_register;
    cpu_register_t cpu_lr_register;
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    CAPTURE_ARM_CPSR_REGISTER(cpu_status_register);
    CAPTURE_ARM_LR_REGISTER(cpu_lr_register);

    if (cpu_controller_p->cpc_startup_completed)
    {
        struct rtos_execution_context *current_context_p =
            cpu_controller_p->cpc_current_execution_context_p;

        FDC_ASSERT(
            current_context_p->ctx_signature == RTOS_EXECUTION_CONTEXT_SIGNATURE,
            current_context_p->ctx_signature, current_context_p);

        /*
         * Current execution context is a thread running in privileged mode
         * or is an interrupt context:
         */
        if (current_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
        {
            FDC_ASSERT(
                current_context_p->ctx_cpu_mode == RTOS_PRIVILEGED_THREAD_MODE,
                current_context_p->ctx_cpu_mode, cpu_lr_register);

            FDC_ASSERT(
                CPU_MODE_IS_PRIVILEGED(cpu_status_register),
                cpu_status_register, current_context_p);
    
            FDC_ASSERT_RUNNING_THREAD_INVARIANTS(
                RTOS_EXECUTION_CONTEXT_GET_THREAD(current_context_p));
        }
        else
        {
            FDC_ASSERT(
                current_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
                current_context_p->ctx_context_type, current_context_p);

            FDC_ASSERT(
                current_context_p->ctx_cpu_mode == RTOS_INTERRUPT_MODE,
                current_context_p->ctx_cpu_mode, current_context_p);

            FDC_ASSERT(
                CPU_MODE_IS_PRIVILEGED(cpu_status_register),
                cpu_status_register, current_context_p);

            FDC_ASSERT(!thread_callers_only, 0, 0);
        }
    }
    else
    {
        FDC_ASSERT(
            cpu_controller_p->cpc_current_execution_context_p == NULL,
            cpu_controller_p->cpc_current_execution_context_p,
            cpu_controller_p);

        FDC_ASSERT(
            CPU_MODE_IS_SUPERVISOR(cpu_status_register),
            cpu_status_register, cpu_controller_p);
    }

#   elif DEFINED_ARM_CORTEX_M_ARCH()
    cpu_register_t cpu_lr_register;
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    CAPTURE_ARM_LR_REGISTER(cpu_lr_register);
    cpu_register_t cpu_control_register = __get_CONTROL();

    if (cpu_controller_p->cpc_startup_completed)
    {
        struct rtos_execution_context *current_context_p =
            cpu_controller_p->cpc_current_execution_context_p;

        FDC_ASSERT(
            current_context_p->ctx_signature == RTOS_EXECUTION_CONTEXT_SIGNATURE,
            current_context_p->ctx_signature, current_context_p);

        /*
         * Current execution context is a thread running in privileged mode
         * or is an interrupt context:
         */
        if (current_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
        {
#           if 0 // TODO: Enable this check when SVC logic is implemented
            FDC_ASSERT(
                current_context_p->ctx_cpu_mode == RTOS_PRIVILEGED_THREAD_MODE,
                current_context_p->ctx_cpu_mode, cpu_lr_register);
#           endif

            FDC_ASSERT(
                CPU_MODE_IS_PRIVILEGED(cpu_control_register),
                cpu_control_register, current_context_p);
    
            FDC_ASSERT_RUNNING_THREAD_INVARIANTS(
                RTOS_EXECUTION_CONTEXT_GET_THREAD(current_context_p));
        }
        else
        {
            FDC_ASSERT(
                current_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
                current_context_p->ctx_context_type, current_context_p);

            FDC_ASSERT(
                current_context_p->ctx_cpu_mode == RTOS_INTERRUPT_MODE,
                current_context_p->ctx_cpu_mode, current_context_p);

            FDC_ASSERT(
                CPU_MODE_IS_PRIVILEGED(cpu_control_register),
                cpu_control_register, current_context_p);

            FDC_ASSERT(!thread_callers_only, 0, 0);
        }
    }
    else
    {
        FDC_ASSERT(
            cpu_controller_p->cpc_current_execution_context_p == NULL,
            cpu_controller_p->cpc_current_execution_context_p,
            cpu_controller_p);

        TODO("Enable this when SVC excpetion handler is implemented");
#       if 0
        cpu_status_register_t cpu_status_register = __get_IPSR();

        FDC_ASSERT(
            CPU_MODE_IS_SUPERVISOR(cpu_status_register),
            cpu_status_register, cpu_controller_p);
#       endif
    }

#   else
#        error "CPU architecture not supported"
#   endif
}


void
check_rtos_interrupt_entry_preconditions(
    _IN_ struct rtos_interrupt *rtos_interrupt_p)
{
    struct rtos_execution_context *interrupt_context_p =
        &rtos_interrupt_p->int_execution_context;

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_context_p->ctx_signature == RTOS_EXECUTION_CONTEXT_SIGNATURE,
        current_context_p->ctx_signature, current_context_p);

    /*
     * Check saved CPU registers for the interrupted context:
     */
    FDC_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(current_context_p);

    /*
     * The current context must have been running in either user mode or system
     * mode, otherwise it could not have been interrupted, as we only allow
     * interrupts to be enabled in user mode or system mode.
     */

#   if DEFINED_ARM_CLASSIC_ARCH()
    cpu_status_register_t current_context_cpsr =
        current_context_p->ctx_cpu_registers[CPU_REG_CPSR];

    FDC_ASSERT(
        CPU_MODE_IS_UNPRIVILEGED(current_context_cpsr) ||
        CPU_MODE_IS_PRIVILEGED(current_context_cpsr),
        current_context_cpsr, current_context_p);
#   endif

    /*
     * Current execution context must be different from the calling
     * interrupt context:
     */
    FDC_ASSERT(
        interrupt_context_p != current_context_p,
        interrupt_context_p, current_context_p);

#   if DEBUG   
    if (rtos_interrupt_p->int_channel < 0) {
        FDC_ASSERT(
            (cpu_controller_p->cpc_active_internal_interrupts & 
                BIT(-rtos_interrupt_p->int_channel)) == 0,
            cpu_controller_p->cpc_active_internal_interrupts, cpu_controller_p);
    } else {
        FDC_ASSERT(
            (cpu_controller_p->cpc_active_external_interrupts & 
                BIT(rtos_interrupt_p->int_channel)) == 0,
            cpu_controller_p->cpc_active_external_interrupts, cpu_controller_p);
    }
#   endif
}


/**
 * Check preconditions of an interrupt handler that runs with CPU interrupts
 * enabled.
 */
void
check_rtos_interrupt_e_handler_preconditions(
    _IN_ const struct rtos_interrupt *rtos_interrupt_p)
{
    /*
     * We are running in interrupt mode but interrupts are currently enabled: 
     */
    DBG_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();

#if defined(DEBUG) && DEFINED_ARM_CORTEX_M_ARCH()
    uint32_t reg_ipsr = __get_IPSR();

    FDC_ASSERT(
        CPU_MODE_IS_INTERRUPT(reg_ipsr), reg_ipsr, rtos_interrupt_p);
#endif

    FDC_ASSERT(
        rtos_interrupt_p->int_signature == RTOS_INTERRUPT_SIGNATURE,
        rtos_interrupt_p->int_signature, rtos_interrupt_p);

    const struct rtos_execution_context *current_context_p =
        RTOS_GET_CURRENT_EXECUTION_CONTEXT();

    DBG_ASSERT(
        current_context_p == &rtos_interrupt_p->int_execution_context,
        current_context_p,
        &rtos_interrupt_p->int_execution_context);

    /*
     * Current execution context is an interrupt and is not in preemption chain:
     */

    FDC_ASSERT(
        current_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
        current_context_p->ctx_context_type, current_context_p);

    FDC_ASSERT(
        !GLIST_NODE_IS_LINKED(&current_context_p->ctx_preemption_chain_node),
        &current_context_p->ctx_preemption_chain_node,
        current_context_p);

    /*
     * The current thread is in the preemption chain, but not necessarily
     * at the top, since the calling tick timer interrupt may not have preempted
     * the current but a lower priority interrupt, and in that case the
     * current thread was preempted by one of the lower priority interrupts
     * currently in the preemption chain.
     */

#ifdef DEBUG
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_thread *current_thread_p =
        cpu_controller_p->cpc_current_thread_p;

    DBG_ASSERT_RTOS_THREAD_INVARIANTS(current_thread_p);

    struct glist_node *preemption_chain_anchor_p = 
        &cpu_controller_p->cpc_preemption_chain_anchor;

    DBG_ASSERT(
        !GLIST_IS_EMPTY(preemption_chain_anchor_p),
        preemption_chain_anchor_p, cpu_controller_p);

    struct glist_node *top_preemption_node_p = 
        GLIST_GET_FIRST(preemption_chain_anchor_p);

    struct rtos_execution_context *preempted_context_p =
        RTOS_PREEMPTION_CHAIN_NODE_GET_EXECUTION_CONTEXT(
        top_preemption_node_p);

    DBG_ASSERT(
        cpu_controller_p->cpc_nested_interrupts_count > 0,
        cpu_controller_p->cpc_nested_interrupts_count,
        cpu_controller_p);

    if (cpu_controller_p->cpc_nested_interrupts_count == 1)
    {
        /*
         * The top node of the preemption chain is the current thread's context:
         */
        DBG_ASSERT(
            preempted_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
            preempted_context_p->ctx_context_type, preempted_context_p);

        struct rtos_thread *preempted_thread_p =
            RTOS_EXECUTION_CONTEXT_GET_THREAD(preempted_context_p);

        DBG_ASSERT(
            preempted_thread_p == current_thread_p,
            preempted_thread_p, current_thread_p);
    }
    else
    {
        DBG_ASSERT(
            preempted_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            preempted_context_p->ctx_context_type, preempted_context_p);
    }
#endif /* DEBUG */
}


#if DEFINED_ARM_CORTEX_M_ARCH()
/*
 * Check preconditions of synchronous context switch for Cortex-M
 * processors:
 * - Interrupts are disabled
 * - The CPU is in thread mode
 * - The CPU is in privileged thread
 * - If PSP stack pointer != NULL, the CPU is using the PSP stack
 *   pointer
 */
void
check_synchronous_context_switch_preconditions(
    _IN_ const struct rtos_execution_context *current_execution_context_p)
{
    uint32_t reg_value;

    check_rtos_execution_context_invariants(current_execution_context_p);
    FDC_ASSERT(
        current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT ||
        current_execution_context_p->ctx_context_type == RTOS_RESET_CONTEXT,
        current_execution_context_p->ctx_context_type,
        current_execution_context_p);

    /*
     * Interrupts are disabled at the CPU:
     */
    reg_value = __get_PRIMASK();
    FDC_ASSERT(
        (reg_value & CPU_REG_PRIMASK_PM_MASK) != 0,
        reg_value, current_execution_context_p); 

    /*
     * The caller is running in thread mode:
     */
    reg_value = __get_IPSR();
    FDC_ASSERT(
        (reg_value & CPU_REG_IPSR_EXCEPTION_NUMBER_MASK) == 0,
        reg_value, current_execution_context_p); 

    /*
     * The caller is running in privileged mode:
     */
    reg_value = __get_CONTROL();
    FDC_ASSERT(
        (reg_value & CPU_REG_CONTROL_nPRIV_MASK) == 0,
        reg_value, current_execution_context_p);

    if (__get_PSP() == 0x0) {
        /*
         *  This is the first context switch, done from the resrt handler.
         *  Thus, the SP in use must be MSP.
         */
        FDC_ASSERT(
            (reg_value & CPU_REG_CONTROL_SPSEL_MASK) == 0,
            reg_value, current_execution_context_p);
    } else {
        /*
         *  The SP in use must be PSP.
         */
        FDC_ASSERT(
            (reg_value & CPU_REG_CONTROL_SPSEL_MASK) != 0,
            reg_value, current_execution_context_p);
    }
}
#endif
