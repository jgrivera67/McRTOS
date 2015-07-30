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
#include <McRTOS/failure_data_capture.h>
#include <BoardSupport/hardware_abstractions.h>
#include <McRTOS/arm_defs.h>
#include <McRTOS/McRTOS_kernel_services.h>
#include <McRTOS/McRTOS_internals.h>
#include <McRTOS/utils.h>

/**
 * Macro that prints a failure message on the serial port in product code,
 * or to a CppUtest log for unit test code.
 */

#ifdef CPPUTEST_COMPILATION  // from CppUTest

#   define FAILURE_PRINTF(_fmt, ...) \
            cpputest_printf(                                                \
                "FDC: " _fmt, ##__VA_ARGS__)

#else
#   define FAILURE_PRINTF(_fmt, ...) \
    do {								    \
            capture_fdc_msg_printf(					    \
                "FDC: " _fmt, ##__VA_ARGS__);				    \
    } while (0)

#endif /* CPPUTEST_COMPILATION */

#ifdef _RELIABILITY_CHECKS_
static void capture_unexpected_exception_failure(
    enum unexpected_exception_types exception_type,
    void *location,
    uintptr_t arg,
    cpu_status_register_t cpu_status_register);
#endif

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

#pragma GCC diagnostic push

#ifndef _RELIABILITY_CHECKS_
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

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
#   ifdef _RELIABILITY_CHECKS_

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    /*
     * NOTE: We cannot call rtos_k_disable_cpu_interrupts(), since unwanted
     * recursion can happen if an assert fails in
     * rtos_stop_interrupts_disabled_time_measure()
     */

    cpu_status_register_t old_primask = __get_PRIMASK();

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
	__disable_irq();
	__ISB();
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

    failure->fr_cpu_status_register = old_primask;
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

            capture_fdc_stack_trace(4);
            failure_being_printed = false;
        }
    }

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
        __ISB();
        __enable_irq();
    }

#   endif /* _RELIABILITY_CHECKS_ */
}


void
rtos_k_set_fdc_params(
    _IN_ bool assert_break_point_on,
    _IN_ bool exception_debugger_on)
{
#   ifdef _RELIABILITY_CHECKS_
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    fdc_info_p->fdc_asserts_failures_breakpoint_on = assert_break_point_on;
    fdc_info_p->fdc_exception_debugger_on = exception_debugger_on;
#   endif /* _RELIABILITY_CHECKS_ */
}


#ifdef _RELIABILITY_CHECKS_
/**
 * Captures failure data for an assertion failure
 */
fdc_error_t
capture_assert_failure(
    const char *cond_str,
    uintptr_t arg1,
    uintptr_t arg2)
{
    uintptr_t return_address;
    uintptr_t assert_address;

    /*
     * Capture ARM LR register on entry
     */
    CAPTURE_ARM_LR_REGISTER(return_address);

#   ifdef _BRANCH_MICRO_TRACING_
    micro_trace_stop();
#   endif

    /*
     * The exact location of the assertion is the place where this
     * function was invoked
     */
    assert_address = GET_CALL_ADDRESS(return_address);

    rtos_capture_failure_data(
        cond_str,
        arg1,
        arg2,
        (void *)assert_address);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    if (!fdc_info_p->fdc_handling_exception) {
        if (fdc_info_p->fdc_asserts_failures_breakpoint_on) {
            ARTIFICIAL_BREAK_POINT();
        } else {
#           ifdef _BRANCH_MICRO_TRACING_
            micro_trace_restart();
#           endif
        }
    }

    return (fdc_error_t)assert_address;
}
#endif /* _RELIABILITY_CHECKS_ */


/**
 * Capture failure data for a runtime error
 */
fdc_error_t
capture_fdc_error(
    const char *error_description,
    uintptr_t arg1,
    uintptr_t arg2)
{
#   ifdef _RELIABILITY_CHECKS_
    uintptr_t return_address;
    uintptr_t error_address;

    /*
     * Capture ARM LR register on entry
     */
    CAPTURE_ARM_LR_REGISTER(return_address);

#   ifdef _BRANCH_MICRO_TRACING_
    micro_trace_stop();
#   endif

    error_address = GET_CALL_ADDRESS(return_address);

    rtos_capture_failure_data(
        error_description,
        arg1,
        arg2,
        (void *)error_address);

    bool caller_was_privileged = rtos_enter_privileged_mode();

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    if (fdc_info_p->fdc_error_breakpoint_on) {
        ARTIFICIAL_BREAK_POINT();
    }

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }

#   ifdef _BRANCH_MICRO_TRACING_
    micro_trace_restart();
#   endif

    return (fdc_error_t)error_address;
#   else

    return 0;

#   endif /* _RELIABILITY_CHECKS_ */
}

/**
 * Captures a trace for a context switch and checks the preconditions for
 * moving to a new execution context
 */
void
fdc_trace_rtos_context_switch(
    _IN_ const struct rtos_execution_context *target_execution_context_p,
    _IN_ rtos_context_switch_type_t ctx_switch_type)
{
#   ifdef _RELIABILITY_CHECKS_
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

#   ifdef DEBUG
    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;
#   endif

    DBG_ASSERT(
        target_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT ||
        target_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
        target_execution_context_p->ctx_context_type, target_execution_context_p);

    DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(target_execution_context_p);

#   ifdef DEBUG
    switch (ctx_switch_type) {
    case RTOS_CSW_THREAD_TO_INTERRUPT:
    case RTOS_CSW_THREAD_TO_EARLY_NESTED_INTERRUPT:
        DBG_ASSERT(
            current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
            current_execution_context_p->ctx_context_type,
            current_execution_context_p);

        DBG_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(
            current_execution_context_p, ctx_switch_type);

        DBG_ASSERT(
            target_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            target_execution_context_p->ctx_context_type,
            target_execution_context_p);
        break;

    case RTOS_CSW_ENTERING_NESTED_INTERRUPT:
        DBG_ASSERT(
            current_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            current_execution_context_p->ctx_context_type,
            current_execution_context_p);

        DBG_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(
            current_execution_context_p, ctx_switch_type);

        DBG_ASSERT(
            target_execution_context_p != current_execution_context_p,
            target_execution_context_p, current_execution_context_p);

        DBG_ASSERT(
            target_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            target_execution_context_p->ctx_context_type,
            target_execution_context_p);
        break;

    case RTOS_CSW_EXITING_NESTED_INTERRUPT:
        DBG_ASSERT(
            current_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            current_execution_context_p->ctx_context_type,
            current_execution_context_p);

        DBG_ASSERT(
            target_execution_context_p != current_execution_context_p,
            target_execution_context_p, current_execution_context_p);

        DBG_ASSERT(
            target_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            target_execution_context_p->ctx_context_type,
            target_execution_context_p);

        DBG_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(
            target_execution_context_p, ctx_switch_type);
        break;

    case RTOS_CSW_INTERRUPT_TO_THREAD:
    case RTOS_CSW_EXITING_EARLY_NESTED_INTERRUPT:
        DBG_ASSERT(
            current_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            current_execution_context_p->ctx_context_type,
            current_execution_context_p);

        DBG_ASSERT(
            target_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
            target_execution_context_p->ctx_context_type,
            target_execution_context_p);

        DBG_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(
            target_execution_context_p, ctx_switch_type);
        break;

    case RTOS_CSW_THREAD_TO_THREAD:
        DBG_ASSERT(
            current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
            current_execution_context_p->ctx_context_type,
            current_execution_context_p);

	DBG_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(
	    current_execution_context_p, ctx_switch_type);

        if (target_execution_context_p != current_execution_context_p) {
            DBG_ASSERT(
                target_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
                target_execution_context_p->ctx_context_type,
                target_execution_context_p);

            DBG_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(
                target_execution_context_p, ctx_switch_type);
        } else {
	    DBG_ASSERT(current_execution_context_p->ctx_last_switched_out_reason !=
	               CTX_SWITCHED_OUT_THREAD_TERMINATED,
		       current_execution_context_p, 0);
	}
        break;

    case RTOS_CSW_RESET_TO_THREAD:
        DBG_ASSERT(
            current_execution_context_p->ctx_context_type == RTOS_RESET_CONTEXT,
            current_execution_context_p->ctx_context_type,
            current_execution_context_p);

        DBG_ASSERT(
            target_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
            target_execution_context_p->ctx_context_type,
            target_execution_context_p);

        DBG_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(
            target_execution_context_p, ctx_switch_type);
        break;

    default:
        FDC_ASSERT(false, ctx_switch_type, target_execution_context_p);
    }

    if (current_execution_context_p != target_execution_context_p) {
        DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(current_execution_context_p);

        DBG_ASSERT(
            current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT ||
            current_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT ||
            current_execution_context_p->ctx_context_type == RTOS_RESET_CONTEXT,
            current_execution_context_p->ctx_context_type, current_execution_context_p);
    } else {
        DBG_ASSERT(
            target_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
            target_execution_context_p->ctx_context_type, target_execution_context_p);
    }
#   endif /* DEBUG */

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

#   elif DEFINED_ARM_CORTEX_M_ARCH()

#   ifdef DEBUG
    cpu_status_register_t actual_primask = __get_PRIMASK();
    cpu_status_register_t actual_ipsr = __get_IPSR();
    cpu_status_register_t actual_control_reg = __get_CONTROL();
#   endif

    DBG_ASSERT(
        CPU_INTERRUPTS_ARE_DISABLED(actual_primask),
        actual_primask, target_execution_context_p);

    DBG_ASSERT(
        (CPU_MODE_IS_INTERRUPT(actual_ipsr) ||
         CPU_MODE_IS_PENDSV_EXCEPTION(actual_ipsr) ||
         CPU_MODE_IS_HARD_FAULT_EXCEPTION(actual_ipsr)) &&
         CPU_MODE_IS_PRIVILEGED(actual_control_reg, actual_ipsr) &&
         CPU_USING_MSP_STACK_POINTER(actual_control_reg),
        actual_ipsr, actual_control_reg);
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

    SET_BIT_FIELD(
        trace_entry,
        FDC_CST_CONTEXT_SWITCH_TYPE_MASK,
        FDC_CST_CONTEXT_SWITCH_TYPE_SHIFT,
        ctx_switch_type);

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
#   endif

    fdc_info_p->fdc_context_switch_trace_buffer[
        fdc_info_p->fdc_context_switch_trace_cursor] = trace_entry;

    fdc_info_p->fdc_context_switch_trace_cursor ++;
    if (fdc_info_p->fdc_context_switch_trace_cursor ==
        RTOS_NUM_CONTEXT_SWITCH_TRACE_BUFFER_ENTRIES) {
        fdc_info_p->fdc_context_switch_trace_cursor = 0;
    }

    fdc_info_p->fdc_context_switch_count ++;
#   endif /* _RELIABILITY_CHECKS_ */
}

#pragma GCC diagnostic pop

#ifdef _RELIABILITY_CHECKS_

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
        CPU_MODE_IS_PRIVILEGED(current_control, current_ipsr), 0, 0);

    FDC_ASSERT(
        CPU_INTERRUPTS_ARE_ENABLED(__get_PRIMASK()), 0, 0);
#else
#   error "CPU architecture not supported"
#endif
}


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

#	ifdef DEBUG
        if (rtos_thread_p->thr_state == RTOS_THREAD_RUNNING)
        {
            struct rtos_cpu_controller *cpu_controller_p =
                &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

            DBG_ASSERT(
                rtos_thread_p == cpu_controller_p->cpc_current_thread_p,
                rtos_thread_p, cpu_controller_p->cpc_current_thread_p);
        }

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
#	endif
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
    _IN_ const struct rtos_execution_context *rtos_execution_context_p,
    _IN_ rtos_context_switch_type_t ctx_switch_type)
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

uint32_t *g_check_rtos_execution_context_cpu_registers_last_caller = NULL;

void
check_rtos_execution_context_cpu_registers(
    _IN_ const struct rtos_execution_context *rtos_execution_context_p,
    _IN_ rtos_context_switch_type_t ctx_switch_type)
{
    /*
     * Capture ARM LR register on entry
     */
    uint32_t *return_address;
    CAPTURE_ARM_LR_REGISTER(return_address);

    g_check_rtos_execution_context_cpu_registers_last_caller = return_address - 1;

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
            CPU_MODE_IS_THREAD(cpu_status_register),
            cpu_status_register, rtos_execution_context_p);

        if (ctx_switch_type == RTOS_CSW_THREAD_TO_EARLY_NESTED_INTERRUPT ||
            ctx_switch_type == RTOS_CSW_EXITING_EARLY_NESTED_INTERRUPT)
        {
	    FDC_ASSERT(
		rtos_execution_context_p->
		    ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
		CPU_EXC_RETURN_TO_HANDLER_MODE,
		rtos_execution_context_p->
		    ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry,
		rtos_execution_context_p);
        }
        else
        {
	    FDC_ASSERT(
		rtos_execution_context_p->
		    ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
		CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP ||
		rtos_execution_context_p->
		    ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
		CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP_FPU,
		rtos_execution_context_p->
		    ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry,
		rtos_execution_context_p);
	}
    }
    else
    {
        FDC_ASSERT(
            CPU_MODE_IS_INTERRUPT(cpu_status_register),
            cpu_status_register, rtos_execution_context_p);

        FDC_ASSERT(
            rtos_execution_context_p->
                ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
            CPU_EXC_RETURN_TO_HANDLER_MODE,
            rtos_execution_context_p->
                ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry,
            rtos_execution_context_p);
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

void capture_unexpected_fault(
    void *location, uintptr_t arg, uint32_t psr,
    enum cpu_core_internal_interrupt_vectors exception_vector)
{
    enum unexpected_exception_types exception_type;

    switch (exception_vector) {
    case INT_Hard_Fault:
	    exception_type = UET_HARD_FAULT;
	    break;
    case INT_MemoryManagement:
	    exception_type = UET_MEMORY_MANAGEMENT_FAULT;
	    break;
    case INT_BusFault:
	    exception_type = UET_BUS_FAULT;
	    break;
    case INT_UsageFault:
	    exception_type = UET_USAGE_FAULT;
	    break;
    case INT_DebugMonitor:
	    exception_type = UET_DEBUG_MONITOR_FAULT;
	    break;
    default:
	    exception_type = UET_INVALID_EXCEPTION_TYPE;
    }

    capture_unexpected_exception_failure(
        exception_type, location, arg, psr);
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
      [UET_HARD_FAULT] = "Hard Fault",
      [UET_MEMORY_MANAGEMENT_FAULT] = "Memory Management Fault",
      [UET_BUS_FAULT] = "Bus Fault",
      [UET_USAGE_FAULT] = "Usage Fault",
      [UET_DEBUG_MONITOR_FAULT] = "Debug Monitor Fault",

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

#endif /* _RELIABILITY_CHECKS_ */


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


#ifdef _RELIABILITY_CHECKS_
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

    struct rtos_execution_context *current_context_p =
	cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
	current_context_p->ctx_signature == RTOS_EXECUTION_CONTEXT_SIGNATURE,
	current_context_p->ctx_signature, current_context_p);

    /*
     * Current execution context is a thread running in privileged mode
     * or is an interrupt context or it is the reset context:
     */
    if (current_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
	struct rtos_thread *current_thread =
	    RTOS_EXECUTION_CONTEXT_GET_THREAD(current_context_p);

	FDC_ASSERT_RTOS_THREAD_INVARIANTS(current_thread_p);

	FDC_ASSERT_RUNNING_THREAD_INVARIANTS(current_thread_p, cpu_controller_p);

	FDC_ASSERT(
	    current_context_p->ctx_cpu_mode == RTOS_PRIVILEGED_THREAD_MODE,
	    current_context_p->ctx_cpu_mode, cpu_lr_register);

	FDC_ASSERT(
	    CPU_MODE_IS_PRIVILEGED(cpu_status_register),
	    cpu_status_register, current_context_p);

    }
    else if (current_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT)
    {
	FDC_ASSERT(
	    current_context_p->ctx_cpu_mode == RTOS_INTERRUPT_MODE,
	    current_context_p->ctx_cpu_mode, current_context_p);

	FDC_ASSERT(
	    CPU_MODE_IS_PRIVILEGED(cpu_status_register),
	    cpu_status_register, current_context_p);

	FDC_ASSERT(!thread_callers_only, 0, 0);
    }
    else
    {
	FDC_ASSERT(
	    current_context_p->ctx_context_type == RTOS_RESET_CONTEXT,
	    current_context_p->ctx_context_type, current_context_p);

	FDC_ASSERT(
	    current_context_p->ctx_cpu_mode == RTOS_RESET_MODE,
	    current_context_p->ctx_cpu_mode, current_context_p);

        FDC_ASSERT(
            CPU_MODE_IS_SUPERVISOR(cpu_status_register),
            cpu_status_register, cpu_controller_p);

	FDC_ASSERT(!thread_callers_only, 0, 0);
    }

#   elif DEFINED_ARM_CORTEX_M_ARCH()
    cpu_register_t cpu_lr_register;
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    CAPTURE_ARM_LR_REGISTER(cpu_lr_register);
    cpu_register_t cpu_control_register = __get_CONTROL();
    cpu_register_t cpu_ipsr_register = __get_IPSR();

    struct rtos_execution_context *current_context_p =
	cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
	current_context_p->ctx_signature == RTOS_EXECUTION_CONTEXT_SIGNATURE,
	current_context_p->ctx_signature, current_context_p);

    /*
     * Current execution context is a thread running in privileged mode
     * or is an interrupt context or the reset context:
     */
    if (current_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
	struct rtos_thread *current_thread_p =
	    RTOS_EXECUTION_CONTEXT_GET_THREAD(current_context_p);

    FDC_ASSERT_RTOS_THREAD_INVARIANTS(current_thread_p);

	FDC_ASSERT_RUNNING_THREAD_INVARIANTS(current_thread_p, cpu_controller_p);

	FDC_ASSERT(
	    current_context_p->ctx_cpu_mode == RTOS_PRIVILEGED_THREAD_MODE,
	    current_context_p->ctx_cpu_mode, cpu_lr_register);

	FDC_ASSERT(
	    CPU_MODE_IS_PRIVILEGED(cpu_control_register, cpu_ipsr_register),
	cpu_control_register, cpu_ipsr_register);
    }
    else
    {
	FDC_ASSERT(
	    current_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT ||
	    current_context_p->ctx_context_type == RTOS_RESET_CONTEXT,
	    current_context_p->ctx_context_type, current_context_p);

	if (current_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT) {
	    FDC_ASSERT(
		current_context_p->ctx_cpu_mode == RTOS_INTERRUPT_MODE,
		current_context_p->ctx_cpu_mode, current_context_p);
	} else {
	    FDC_ASSERT(
		current_context_p->ctx_cpu_mode == RTOS_RESET_MODE,
		current_context_p->ctx_cpu_mode, current_context_p);
	}

	FDC_ASSERT(
	    CPU_MODE_IS_PRIVILEGED(cpu_control_register, cpu_ipsr_register),
	    cpu_control_register, cpu_ipsr_register);

	FDC_ASSERT(!thread_callers_only, 0, 0);
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

#   ifdef DEBUG
    if (rtos_interrupt_p->int_channel < 0) {
        FDC_ASSERT(
            RTOS_INTR_BIT_MAP_GET_BIT(
		cpu_controller_p->cpc_active_internal_interrupts,
		-rtos_interrupt_p->int_channel) == 0,
            cpu_controller_p->cpc_active_internal_interrupts,
	    -rtos_interrupt_p->int_channel);
    } else {
	FDC_ASSERT(
            RTOS_INTR_BIT_MAP_GET_BIT(
		cpu_controller_p->cpc_active_external_interrupts,
		rtos_interrupt_p->int_channel) == 0,
            cpu_controller_p->cpc_active_external_interrupts,
	    rtos_interrupt_p->int_channel);
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

#endif /* _RELIABILITY_CHECKS_ */

static void
fdc_capture_msg_putchar(
    void *putchar_arg_p,
    _IN_ uint8_t c)
{
    struct fdc_info *fdc_info_p = putchar_arg_p;
    uint16_t cursor = fdc_info_p->fdc_msg_cursor;

    fdc_info_p->fdc_msg_buffer[cursor] = c;
    cursor ++;
    if (cursor == RTOS_FDC_MSG_BUFFER_SIZE - 1) {
	cursor = 0;
    }

    fdc_info_p->fdc_msg_cursor = cursor;
}

/**
 * Simplified printf that write messages to the FDC message buffer.
 * It only supports the format specifiers supported by embedded_vprintf().
 *
 * @param fmt               format string
 *
 * @param ...               variable arguments
 *
 * @return None
 */
void
capture_fdc_msg_printf(const char *fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    rtos_capture_fdc_msg_vprintf(fmt, va);
    va_end(va);
}


/**
 * Simplified vprintf that write messages to the FDC message buffer.
 * It only supports the format specifiers supported by embedded_vprintf().
 *
 * @param fmt               format string
 *
 * @param ...               variable arguments
 *
 * @return None
 */
void
rtos_k_capture_fdc_msg_vprintf(const char *fmt, va_list va)
{
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;
    cpu_status_register_t old_primask = __get_PRIMASK();

    __disable_irq();

    embedded_printf(fdc_capture_msg_putchar, fdc_info_p, "%u ",
		    fdc_info_p->fdc_msg_seq_num);

    embedded_vprintf(fdc_capture_msg_putchar, fdc_info_p, fmt, va);
    fdc_info_p->fdc_msg_seq_num ++;

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
        __enable_irq();
    }
}


void
capture_fdc_stack_trace(uint_fast8_t num_entries_to_skip)
{
    uintptr_t trace_buff[RTOS_MAX_STACK_TRACE_ENTRIES];
    uint8_t num_trace_entries;

    struct rtos_execution_context *current_execution_context_p =
        RTOS_GET_CURRENT_EXECUTION_CONTEXT();

    FAILURE_PRINTF("Stack trace for %s:\n",
                   current_execution_context_p->ctx_name_p);

    num_trace_entries = sizeof(trace_buff) / sizeof(trace_buff[0]);
    get_stack_trace(current_execution_context_p, num_entries_to_skip,
                    trace_buff, &num_trace_entries);

    for (uint_fast8_t i = 0; i < num_trace_entries; i ++) {
	FAILURE_PRINTF("\t%#p\n", trace_buff[i]);
    }
}
