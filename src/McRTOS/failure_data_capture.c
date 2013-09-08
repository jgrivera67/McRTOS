/**
 * @file failure_data_capture.c
 *
 * Failure data capture support functions
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
                _fmt, ##__VA_ARGS__)
#else
    /**
     * NOTE: we cannot use console_printf() here because FAILURE_PRINTF()
     * needs to be able to always send output to serial port.
     */ 
#   define FAILURE_PRINTF(_fmt, ...) \
            embedded_printf(                                                \
                (putchar_func_t *)uart_putchar_with_polling,                \
                (void *)g_console_serial_port_p,                            \
                _fmt, ##__VA_ARGS__)

#endif /* CPPUTEST_COMPILATION */


static void capture_unexpected_exception_failure(
    enum unexpected_exception_types exception_type,
    void *location,
    uintptr_t arg,
    cpu_status_register_t cpu_sr);

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
    uintptr_t arg3,
    void *failure_location)
{
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    cpu_status_register_t cpu_sr;
    bool restore_interrupts = false;
    
    CAPTURE_ARM_CPSR_REGISTER(cpu_sr);
    
    /*
     * Only call rtos_k_disable_cpu_interrupts() if interrupts are currently
     * enabled:
     *
     * NOTE: This check is necessary to avoid an unwanted recursion if an assert
     * fails in rtos_stop_interrupts_disabled_time_measure()
     */
    if (CPU_INTERRUPTS_ARE_ENABLED(cpu_sr))
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
    failure->fr_failure_args[2] = arg3;

    failure->fr_cpu_status_register = cpu_sr;
    failure->fr_execution_context_p = RTOS_GET_CURRENT_EXECUTION_CONTEXT();

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
                "Failure captured: %s (code location: %x, "
                "arg1: 0x%x, arg2: 0x%x, arg3: 0x%x)\n",
                failure_str, failure_location,
                arg1, arg2, arg3);

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
    uintptr_t arg2,
    uintptr_t arg3)
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
        arg3,
        assert_address);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    if (fdc_info_p->fdc_asserts_failures_breakpoint_on)
    {
        debug_break_point();
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
        0,
        error_address);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    if (fdc_info_p->fdc_error_breakpoint_on)
    {
        debug_break_point();
    }

    return (fdc_error_t)error_address;
}


void
check_isr_reset_entry_asserts(void)
{
    uint32_t currentCpsr;
    
    CAPTURE_ARM_CPSR_REGISTER(currentCpsr);

    FDC_ASSERT_EQUAL(
        currentCpsr & ARM_MODE_MASK, ARM_MODE_SVC);

    FDC_ASSERT_EQUAL(
        currentCpsr & ARM_INTERRUPTS_DISABLED_MASK,
        ARM_INTERRUPTS_DISABLED_MASK);
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

            FDC_ASSERT(
                RTOS_EXECUTION_CONTEXT_GET_THREAD(
                    cpu_controller_p->cpc_current_execution_context_p) ==
                cpu_controller_p->cpc_current_thread_p,
                RTOS_EXECUTION_CONTEXT_GET_THREAD(
                    cpu_controller_p->cpc_current_execution_context_p),
                cpu_controller_p->cpc_current_thread_p);

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
    else
    {
        FDC_ASSERT(
            rtos_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            rtos_execution_context_p->ctx_context_type,
            rtos_execution_context_p);
    
        struct rtos_interrupt *rtos_interrupt_p = 
            RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(rtos_execution_context_p);

        FDC_ASSERT(
            rtos_interrupt_p->int_signature == RTOS_INTERRUPT_SIGNATURE,
            rtos_interrupt_p->int_signature, rtos_interrupt_p);
       
        FDC_ASSERT(
            rtos_execution_context_p->ctx_cpu_mode == RTOS_INTERRUPT_MODE,
            rtos_execution_context_p->ctx_cpu_mode, rtos_execution_context_p);

        DBG_ASSERT(
            stack_top_end_p == rtos_interrupt_p->int_stack,
            stack_top_end_p, rtos_interrupt_p->int_stack);

        DBG_ASSERT(
            stack_bottom_end_p ==
                &rtos_interrupt_p->int_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES],
            stack_bottom_end_p, 
            &rtos_interrupt_p->int_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES]);
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

    cpu_register_t cpu_sp_register =
        rtos_execution_context_p->ctx_cpu_registers[CPU_REG_SP];

    cpu_status_register_t cpu_status_register =
        rtos_execution_context_p->ctx_cpu_registers[CPU_REG_CPSR];

    uint32_t arm_cpu_mode = (cpu_status_register & ARM_MODE_MASK);

    /*
     * McRTOS does not support application using Thumb mode
     */
#   if DEFINED_ARM_CLASSIC_ARCH()
        FDC_ASSERT(
            (cpu_status_register & T_BIT) == 0,
            cpu_status_register, T_BIT);
#   endif

    if (rtos_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
        FDC_ASSERT3(
            arm_cpu_mode == ARM_MODE_USER || arm_cpu_mode == ARM_MODE_SYS,
            arm_cpu_mode, rtos_execution_context_p, caller_address);
        
        if (arm_cpu_mode == ARM_MODE_USER)
        {
            FDC_ASSERT(
                CPU_INTERRUPTS_ARE_ENABLED(cpu_status_register),
                cpu_status_register, rtos_execution_context_p);
        }
    }
    else
    {
        FDC_ASSERT3(
            arm_cpu_mode == ARM_MODE_SYS,
            arm_cpu_mode, rtos_execution_context_p, caller_address);
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

#endif /* _RELIABILITY_CHECKS_ */


/**
 * Captures a trace for a context switch and checks the preconditions for
 * moving to a new execution context
 */ 
void
fdc_trace_rtos_context_switch(
    _IN_ const struct rtos_execution_context *rtos_execution_context_p)
{
#ifdef DEBUG
    struct rtos_execution_context *current_execution_context_p =
        RTOS_GET_CURRENT_EXECUTION_CONTEXT();

    DBG_ASSERT(
       rtos_execution_context_p == current_execution_context_p,
       rtos_execution_context_p, current_execution_context_p);
    
    DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(rtos_execution_context_p);
#endif /* DEBUG */

    /*
     * Validate Context switch assumptions:
     */
    cpu_status_register_t current_cpsr;                                       
    CAPTURE_ARM_CPSR_REGISTER(current_cpsr);

    uint32_t current_arm_mode = (current_cpsr & ARM_MODE_MASK);

    cpu_status_register_t target_cpsr =
        rtos_execution_context_p->ctx_cpu_registers[CPU_REG_CPSR];

    uint32_t target_arm_mode = (target_cpsr & ARM_MODE_MASK);

    FDC_ASSERT(
        CPU_INTERRUPTS_ARE_DISABLED(current_cpsr),
        current_cpsr, rtos_execution_context_p);

    FDC_ASSERT(
        target_arm_mode == ARM_MODE_USER ||
        target_arm_mode == ARM_MODE_SYS,
        target_arm_mode, rtos_execution_context_p);

    FDC_ASSERT(
        current_arm_mode == ARM_MODE_SVC ||
        current_arm_mode == ARM_MODE_IRQ ||
        current_arm_mode == ARM_MODE_SYS,
        current_arm_mode, rtos_execution_context_p);

    if (current_arm_mode != ARM_MODE_SYS)
    {
        FDC_ASSERT(
            CPU_INTERRUPTS_ARE_DISABLED(current_cpsr),
            current_cpsr, rtos_execution_context_p);
    }

    if (target_arm_mode == ARM_MODE_USER)
    {
        FDC_ASSERT(
            CPU_INTERRUPTS_ARE_ENABLED(target_cpsr),
            target_cpsr, rtos_execution_context_p);
    }

    /*
     * Capture context switch trace:
     */
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    fdc_context_switch_trace_entry_t trace_entry =
        rtos_execution_context_p->ctx_prefilled_trace_entry;

    SET_BIT_FIELD(
        trace_entry, 
        FDC_CST_LAST_SWITCHED_OUT_REASON_MASK,
        FDC_CST_LAST_SWITCHED_OUT_REASON_SHIFT,
        rtos_execution_context_p->ctx_last_switched_out_reason);

    if (rtos_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
        struct rtos_thread *current_thread_p =
            RTOS_EXECUTION_CONTEXT_GET_THREAD(rtos_execution_context_p);

        DBG_ASSERT(
            current_thread_p->thr_state == RTOS_THREAD_RUNNING,
            current_thread_p->thr_state, current_thread_p);

        SET_BIT_FIELD(
            trace_entry, 
            FDC_CST_CONTEXT_PRIORITY_MASK,
            FDC_CST_CONTEXT_PRIORITY_SHIFT,
            current_thread_p->thr_current_priority);
    }
    else
    {
        DBG_ASSERT(
            rtos_execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
            rtos_execution_context_p->ctx_context_type, rtos_execution_context_p);
          
        struct rtos_interrupt *current_interrupt_p =
            RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(rtos_execution_context_p);

        interrupt_channel_t interrupt_channel = current_interrupt_p->int_channel;

        fdc_info_p->fdc_interrupt_channel_counters[interrupt_channel] ++;
    }

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

    fdc_info_p->fdc_context_switch_trace_buffer[
        fdc_info_p->fdc_context_switch_trace_cursor] = trace_entry;
                                                     
    fdc_info_p->fdc_context_switch_trace_cursor ++;
    if (fdc_info_p->fdc_context_switch_trace_cursor ==  
        RTOS_NUM_CONTEXT_SWITCH_TRACE_BUFFER_ENTRIES) {          
        fdc_info_p->fdc_context_switch_trace_cursor = 0; 
    }                                              
                                                  
    fdc_info_p->fdc_context_switch_count ++;
}


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


static void
capture_unexpected_exception_failure(
    enum unexpected_exception_types exception_type,
    void *location,
    uintptr_t arg,
    cpu_status_register_t cpsr)
{
    static const char *exception_type_strings[] =
    {
      [UET_UNDEFINED_INSTRUCTION] =  "Undefined Instruction",
      [UET_DATA_ABORT] = "Data Abort",
      [UET_PREFETCH_ABORT] =  "Prefetch Abort"
    };

    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();
    DBG_ASSERT(
        exception_type >= UET_UNDEFINED_INSTRUCTION &&
        exception_type <= UET_PREFETCH_ABORT, exception_type, 0);

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    struct unexpected_exception_record *exception_failure =
        fdc_info_p->fdc_unexpected_exceptions +
        fdc_info_p->fdc_unexpected_exceptions_cursor;

    exception_failure->uer_exception_type = exception_type;
    exception_failure->uer_location = location;
    exception_failure->uer_arg = arg;
    exception_failure->uer_cpu_status_register = cpsr;

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
        "Unexpected exception: %s (code location: %x, arg: %x, cpsr: %x)\n",
        exception_type_strings[exception_type], location, arg, cpsr);
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


void debug_break_point(void)
{
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    if (!fdc_info_p->fdc_breakpoints_on)
    {
        return;
    }

    cpu_status_register_t cpu_sr = rtos_k_disable_cpu_interrupts();

    FAILURE_PRINTF("\n*** DEBUG BREAK POINT HIT ***\n");

    /*
     * Hold the processor in a tight loop, until the
     * fdc_breakpoints_on flag is turned off with the debugger.
     * Keep interrupts disabled to block execution of all
     * threads and interrupt handlers.
     */
    do {
    } while (fdc_info_p->fdc_breakpoints_on);

    /*
     * Re-enable flag for next break point:
     */ 
    fdc_info_p->fdc_breakpoints_on = true;

    rtos_k_restore_cpu_interrupts(cpu_sr);
}


void
check_rtos_public_kernel_service_preconditions(bool thread_callers_only)
{
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
}


void
check_rtos_interrupt_handler_preconditions(
    _IN_ const struct rtos_interrupt *rtos_interrupt_p)
{
    /*
     * We are running in interrupt mode but interrupts are currently enabled: 
     */
#if 0 // TODO: Enable this check
    FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();
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
