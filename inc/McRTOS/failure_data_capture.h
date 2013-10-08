/**
 * @file failure_data_capture.h
 *
 * Failure data capture declarations
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 
#ifndef _FAILURE_DATA_CAPTURE_H
#define _FAILURE_DATA_CAPTURE_H

#include <stdint.h>
#include <stdbool.h>
#include "compile_time_checks.h"
#include "arm_defs.h"
#include "hardware_abstractions.h"
#include "McRTOS_config_parameters.h"

/**
 * Compile-time assertion macros
 */ 

#define C_ASSERT(_cond) \
        extern const char c_assert_dummy_decl[(_cond) ? 1 : -1]

#define C_ASSERT2(_assertName, _cond) \
        typedef char c_assert__ ## _assertName[(_cond) ? 1 : -1]

/*
 * Run-time assertion macros
 */

#ifdef _RELIABILITY_CHECKS_

#define FDC_ASSERT_EQUAL(_value1, _value2) \
    do {                                                        \
        uintptr_t value1 = (uintptr_t)(_value1);                \
        uintptr_t value2 = (uintptr_t)(_value2);                \
        FDC_ASSERT(value1 == value2, value1, value2);           \
    } while (0)

#define FDC_ASSERT_NOT_EQUAL(_value1, _value2) \
    do {                                                        \
        uintptr_t value1 = (uintptr_t)(_value1);                \
        uintptr_t value2 = (uintptr_t)(_value2);                \
        FDC_ASSERT(value1 != value2, value1, value2);           \
    } while (0)

#define FDC_ASSERT_LESS_THAN(_value1, _value2) \
    do {                                                        \
        uintptr_t value1 = (uintptr_t)(_value1);                \
        uintptr_t value2 = (uintptr_t)(_value2);                \
        FDC_ASSERT(value1 < value2, value1, value2);            \
    } while (0)


#ifdef CPPUTEST_COMPILATION  // from CppUTest

void cpputest_fail_test_fdc_assert(const char *fmt, ...);

#define FDC_ASSERT(_cond, _arg1, _arg2) \
    do {					                \
        if (_INFREQUENTLY_TRUE_(!(_cond))) {                    \
            cpputest_fail_test_fdc_assert(                      \
                "*** FDC_ASSERT failed: " __FILE__ ":%d "       \
                #_cond " (%#x, %#x)\n",                         \
                __LINE__,                                       \
                (uintptr_t)(_arg1),                             \
                (uintptr_t)(_arg2));                            \
        }						        \
    } while (0)

#else

#if DEFINED_ARM_CLASSIC_ARCH()

#   define FDC_ASSERT(_cond, _arg1, _arg2) \
    do {                                                                \
        asm volatile (                                                  \
            "teq    %[cond], #0\n\t"                                    \
            "moveq  r0, %[cond_str]\n\t"                                \
            "moveq  r1, %[arg1]\n\t"                                    \
            "moveq  r2, %[arg2]\n\t"                                    \
            "bleq   capture_assert_failure"                             \
            :                                                           \
            : [cond] "r" (_cond),                                       \
              [cond_str] "r" ("Assert: " #_cond),                       \
              [arg1] "r" (_arg1),                                       \
              [arg2] "r" (_arg2)                                        \
            : "cc", "r0", "r1", "r2"                                    \
        );                                                              \
    } while (0)

#elif DEFINED_ARM_CORTEX_M_ARCH()

#   define FDC_ASSERT(_cond, _arg1, _arg2) \
    do {					                \
        if (_INFREQUENTLY_TRUE_(!(_cond))) {                    \
            capture_assert_failure(                             \
                "Assert: " #_cond,                              \
                (uintptr_t)(_arg1),                             \
                (uintptr_t)(_arg2));                            \
        }						        \
    } while (0)

#endif

#endif /* CPPUTEST_COMPILATION */


#if DEFINED_ARM_CLASSIC_ARCH()

#   define FDC_ASSERT_CPU_INTERRUPTS_DISABLED() \
           __FDC_ASSERT_ARM_INTERRUPTS_DISABLED(ARM_INTERRUPTS_DISABLED_MASK)

#   define __FDC_ASSERT_ARM_INTERRUPTS_DISABLED(_interrupt_mask) \
    do {                                                                    \
        uint32_t currentCpsr;                                               \
        CAPTURE_ARM_CPSR_REGISTER(currentCpsr);                             \
        FDC_ASSERT(                                                         \
            (currentCpsr & (_interrupt_mask)) != 0 &&                       \
            !CPU_MODE_IS_UNPRIVILEGED(currentCpsr),                         \
            currentCpsr, _interrupt_mask);                                  \
    } while (0)

#   define FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED() \
    do {                                                                    \
        uint32_t currentCpsr;                                               \
        CAPTURE_ARM_CPSR_REGISTER(currentCpsr);                             \
        FDC_ASSERT(                                                         \
            CPU_MODE_IS_PRIVILEGED(currentCpsr), currentCpsr, 0);           \
        FDC_ASSERT(                                                         \
            CPU_INTERRUPTS_ARE_ENABLED(currentCpsr), currentCpsr, 0);       \
    } while (0)

#elif DEFINED_ARM_CORTEX_M_ARCH()

#   define FDC_ASSERT_CPU_INTERRUPTS_DISABLED() \
    FDC_ASSERT(CPU_INTERRUPTS_ARE_DISABLED(__get_PRIMASK()), 0, 0)
   
#   define FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED() \
    do {                                                                    \
        FDC_ASSERT(                                                         \
            CPU_MODE_IS_PRIVILEGED(__get_CONTROL()), 0, 0);                 \
        FDC_ASSERT(                                                         \
            CPU_INTERRUPTS_ARE_ENABLED(__get_PRIMASK()), 0, 0);             \
    } while (0)

#endif

#define FDC_ASSERT_COMING_FROM_RESET() \
    check_isr_reset_entry_asserts()

#define FDC_ASSERT_CPU_IS_LITTLE_ENDIAN() \
    FDC_ASSERT(is_cpu_little_endian(), 0 , 0)

/**
 * Check that an mmio address is in the valid MMIO space
 */
#define FDC_ASSERT_VALID_MMIO_ADDRESS(_io_addr) \
    FDC_ASSERT(BOARD_VALID_MMIO_ADDRESS(_io_addr), _io_addr, 0)

#if DEFINED_ARM_CLASSIC_ARCH()
    /**
     * Check that a code address is in flash memory and word aligned
     */
#   define FDC_ASSERT_VALID_CODE_ADDRESS(_code_addr, _context_p) \
            FDC_ASSERT(                                                 \
                BOARD_VALID_FLASH_ADDRESS(_code_addr) &&                \
                (uintptr_t)(_code_addr) % sizeof(uint32_t) == 0,        \
                _code_addr, _context_p)

    /**
     * Check that a function pointer points to flash memory
     */
#   define FDC_ASSERT_VALID_FUNCTION_POINTER(_func_ptr) \
           FDC_ASSERT_VALID_CODE_ADDRESS(_func_ptr, NULL)
                
#elif DEFINED_ARM_CORTEX_M_ARCH()
    /**
     * Check that a code address is in flash memory and half-word aligned
     */
#   define FDC_ASSERT_VALID_CODE_ADDRESS(_code_addr, _context_p) \
            FDC_ASSERT(                                                 \
                BOARD_VALID_FLASH_ADDRESS(_code_addr),                  \
                _code_addr, _context_p)

    /**
     * Check that lowest bit of function pointer is set (code compiled for
     * thumb mode) and points to flash memory
     */
#   define FDC_ASSERT_VALID_FUNCTION_POINTER(_func_ptr) \
            do {                                                        \
                FDC_ASSERT(                                             \
                    ((uintptr_t)(_func_ptr) & 0x1) != 0, _func_ptr, 0); \
                FDC_ASSERT_VALID_CODE_ADDRESS(_func_ptr, NULL);         \
            } while (0)

#else
#   error "CPU architecture not supported"
#endif /* !DEFINED_ARM_CORTEX_M_ARCH() */

/**
 * Check that a data pointer points to RAM memory
 */
#define FDC_ASSERT_VALID_RAM_POINTER(_data_ptr, _alignment) \
    FDC_ASSERT(                                                         \
        BOARD_VALID_RAM_ADDRESS(_data_ptr) &&                           \
        (uintptr_t)(_data_ptr) % (_alignment) == 0,                     \
        _data_ptr, _alignment)

/**
 * Check that a data pointer points to ROM memory
 */
#define FDC_ASSERT_VALID_ROM_POINTER(_data_ptr, _alignment) \
    FDC_ASSERT(                                                         \
        BOARD_VALID_FLASH_ADDRESS(_data_ptr) &&                         \
        (uintptr_t)(_data_ptr) % (_alignment) == 0,                     \
        _data_ptr, _alignment)

/**
 * Check that a data pointer points to RAM memory or Flash memory
 */
#define FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(_data_ptr, _alignment) \
    FDC_ASSERT(                                                         \
        (BOARD_VALID_RAM_ADDRESS(_data_ptr) ||                          \
         BOARD_VALID_FLASH_ADDRESS(_data_ptr)) &&                       \
        (uintptr_t)(_data_ptr) % (_alignment) == 0,                     \
        _data_ptr, _alignment)


#define FDC_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(                   \
            _rtos_execution_context_p)                                  \
        check_rtos_execution_context_invariants(                        \
            _rtos_execution_context_p)

#define FDC_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(                \
            _rtos_execution_context_p, _ctx_switch_type)                \
        check_rtos_execution_context_cpu_registers(                     \
            _rtos_execution_context_p, _ctx_switch_type)

#define FDC_ASSERT_RTOS_THREAD_INVARIANTS(_rtos_thread_p) \
        do {                                                                \
            FDC_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(                   \
                &(_rtos_thread_p)->thr_execution_context);                  \
            FDC_ASSERT(                                                     \
                (_rtos_thread_p)->thr_execution_context.ctx_context_type == \
                RTOS_THREAD_CONTEXT,                                        \
                (_rtos_thread_p)->thr_execution_context.ctx_context_type,   \
                _rtos_thread_p);                                            \
        } while (0)

#define FDC_ASSERT_RUNNING_THREAD_INVARIANTS(_thread_p, _cpu_controller_p) \
        do {                                                                \
            FDC_ASSERT(                                                     \
                (_thread_p)->thr_state == RTOS_THREAD_RUNNING,              \
                (_thread_p)->thr_state, _thread_p);                         \
            FDC_ASSERT(                                                     \
                GLIST_NODE_IS_UNLINKED(&(_thread_p)->thr_list_node),        \
                &(_thread_p)->thr_list_node, _thread_p);                    \
            FDC_ASSERT(                                                     \
                _thread_p == _cpu_controller_p->cpc_current_thread_p,       \
                _thread_p, _cpu_controller_p->cpc_current_thread_p);        \
            FDC_ASSERT(                                                     \
                &(_thread_p)->thr_execution_context ==                      \
                    _cpu_controller_p->cpc_current_execution_context_p,     \
                &(_thread_p)->thr_execution_context,                        \
                _cpu_controller_p->cpc_current_execution_context_p);        \
        } while (0)

#define FDC_ASSERT_GLIST_ANCHOR_INVARIANTS(_list_p) \
        glist_anchor_check_invariants(_list_p)

#define FDC_ASSERT_GLIST_ELEM_INVARIANTS(_elem_p) \
        glist_elem_check_invariants(_elem_p)

#define FDC_ASSERT_INTERRUPT_SOURCE_IS_SET(_interrupt_channel) \
        assert_interrupt_source_is_set(_interrupt_channel)

#define FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(_thread_callers_only) \
        check_rtos_public_kernel_service_preconditions(_thread_callers_only)

#define FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(_rtos_interrupt_p) \
        check_rtos_interrupt_e_handler_preconditions(_rtos_interrupt_p)

#define FDC_TRACE_RTOS_CONTEXT_SWITCH( \
            _rtos_execution_context_p, _context_switch_type)            \
        fdc_trace_rtos_context_switch(                                  \
            _rtos_execution_context_p, _context_switch_type)

#else

#define FDC_ASSERT_EQUAL(_value1, _value2)
#define FDC_ASSERT_NOT_EQUAL(_value1, _value2)
#define FDC_ASSERT_LESS_THAN(_value1, _value2)
#define FDC_ASSERT(_cond, _arg1, _arg2)
#define FDC_ASSERT_CPU_INTERRUPTS_DISABLED()
#define FDC_ASSERT_UNPRIVILEGED_CPU_MODE()
#define FDC_ASSERT_COMING_FROM_RESET()
#define FDC_ASSERT_CPU_IS_LITTLE_ENDIAN()
#define FDC_ASSERT_VALID_MMIO_ADDRESS(_io_addr)
#define FDC_ASSERT_VALID_FUNCTION_POINTER(_func_ptr)
#define FDC_ASSERT_VALID_RAM_POINTER(_data_ptr, _alignment)
#define FDC_ASSERT_VALID_ROM_POINTER(_data_ptr, _alignment)
#define FDC_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS( \
            _rtos_execution_context_p)

#define FDC_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS( \
            _rtos_execution_context_p, _ctx_switch_type)

#define FDC_ASSERT_RTOS_THREAD_INVARIANTS(_rtos_thread_p)
#define FDC_ASSERT_RUNNING_THREAD_INVARIANTS(_current_thread_p, _cpu_controller_p)
#define FDC_ASSERT_GLIST_ANCHOR_INVARIANTS(_list_p)
#define FDC_ASSERT_GLIST_ELEM_INVARIANTS(_elem_p)
#define FDC_ASSERT_INTERRUPT_SOURCE_IS_SET(_interrupt_channel)
#define FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(_thread_callers_only)
#define FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(_rtos_interrupt_p)
#define FDC_TRACE_RTOS_CONTEXT_SWITCH( \
            _rtos_execution_context_p, _context_switch_type)

#endif /* _RELIABILITY_CHECKS_ */


#if DEFINED_ARM_CLASSIC_ARCH()
/**
 * Capture current value of the ARM core CPSR register
 */
#   define CAPTURE_ARM_CPSR_REGISTER(_cpsr_value) \
    asm volatile ("mrs %[cpsr_value], cpsr" : [cpsr_value] "=r" (_cpsr_value))

#elif DEFINED_ARM_CORTEX_M_ARCH()

#   define CAPTURE_ARM_PSR_REGISTER(_psr_value) \
    asm volatile ("mrs %[psr_value], psr" : [psr_value] "=r" (_psr_value))

#endif

/**
 * Capture current value of the ARM core LR (r14) register
 */
#define CAPTURE_ARM_LR_REGISTER(_lr_value) \
    asm volatile ("mov %[lr_value], lr" : [lr_value] "=r" (_lr_value))

/**
 * Capture current value of the ARM core SP (r13) register
 */
#define CAPTURE_ARM_SP_REGISTER(_sp_value) \
    asm volatile ("mov %[sp_value], sp" : [sp_value] "=r" (_sp_value))


#define GEN_SIGNATURE(_a, _b, _c, _d) \
        (((uint32_t)(_d) << 24) | \
         ((uint32_t)(_c) << 16) | \
         ((uint32_t)(_b) << 8)  | \
         (uint32_t)(_a))


#define CAPTURE_FDC_ERROR(_error_description, _arg1, _arg2) \
        capture_fdc_error(_error_description, \
                          (uintptr_t)_arg1, \
                          (uintptr_t)_arg2)


/**
 * Debug-only macros
 */

#ifdef DEBUG

#define DBG_ASSERT_EQUAL(_value1, _value2) \
        FDC_ASSERT_EQUAL(_value1, _value2)

#define DBG_ASSERT_NOT_EQUAL(_value1, _value2) \
        FDC_ASSERT_NOT_EQUAL(_value1, _value2)

#define DBG_ASSERT_LESS_THAN(_value1, _value2) \
        FDC_ASSERT_LESS_THAN(_value1, _value2)

#define DBG_ASSERT(_cond, _arg1, _arg2) \
        FDC_ASSERT(_cond, _arg1, _arg2)

#define DBG_ASSERT_CPU_INTERRUPTS_DISABLED() \
        FDC_ASSERT_CPU_INTERRUPTS_DISABLED()

#define DBG_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED() \
        FDC_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED()

#define DBG_ASSERT_VALID_MMIO_ADDRESS(_io_addr) \
        FDC_ASSERT_VALID_MMIO_ADDRESS(_io_addr)

#define DBG_ASSERT_VALID_FUNCTION_POINTER(_func_ptr) \
        FDC_ASSERT_VALID_FUNCTION_POINTER(_func_ptr)

#define DBG_ASSERT_VALID_RAM_POINTER(_data_ptr, _alignment) \
        FDC_ASSERT_VALID_RAM_POINTER(_data_ptr, _alignment)

#define DBG_ASSERT_VALID_ROM_POINTER(_data_ptr, _alignment) \
        FDC_ASSERT_VALID_ROM_POINTER(_data_ptr, _alignment)

#define DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(                   \
            _rtos_execution_context_p)                                  \
        FDC_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS(                   \
            _rtos_execution_context_p)

#define DBG_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(                \
            _rtos_execution_context_p, _ctx_switch_type)                \
        FDC_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(                \
            _rtos_execution_context_p, _ctx_switch_type)

#define DBG_ASSERT_RTOS_THREAD_INVARIANTS(_rtos_thread_p) \
        FDC_ASSERT_RTOS_THREAD_INVARIANTS(_rtos_thread_p)

#define DBG_ASSERT_RUNNING_THREAD_INVARIANTS(_current_thread_p, _cpu_controller_p) \
        FDC_ASSERT_RUNNING_THREAD_INVARIANTS(_current_thread_p, _cpu_controller_p)

#define DBG_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(_thread_callers_only) \
        FDC_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(_thread_callers_only)

#else

#define DBG_ASSERT_EQUAL(_value1, _value2)
#define DBG_ASSERT_NOT_EQUAL(_value1, _value2)
#define DBG_ASSERT_LESS_THAN(_value1, _value2)
#define DBG_ASSERT(_cond, _arg1, _arg2)
#define DBG_ASSERT_CPU_INTERRUPTS_DISABLED()
#define DBG_ASSERT_VALID_MMIO_ADDRESS(_io_addr)
#define DBG_ASSERT_VALID_FUNCTION_POINTER(_func_ptr)
#define DBG_ASSERT_VALID_RAM_POINTER(_data_ptr, _alignment)
#define DBG_ASSERT_VALID_ROM_POINTER(_data_ptr, _alignment)
#define DBG_ASSERT_RTOS_EXECUTION_CONTEXT_INVARIANTS( \
            _rtos_execution_context_p)

#define DBG_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS( \
            _rtos_execution_context_p, _ctx_switch_type)

#define DBG_ASSERT_RTOS_THREAD_INVARIANTS(_rtos_thread_p)
#define DBG_ASSERT_RUNNING_THREAD_INVARIANTS(_current_thread_p, _cpu_controller_p)
#define DBG_ASSERT_RTOS_PUBLIC_KERNEL_SERVICE_PRECONDITIONS(_thread_callers_only)

#endif /* DEBUG */

struct rtos_execution_context;
struct rtos_interrupt;

typedef uintptr_t fdc_error_t;

/**
 * Context switch type. Values defined in arm_defs.h (RTOS_CSW_...)
 */
typedef _RANGE_(RTOS_CSW_THREAD_TO_INTERRUPT, RTOS_CSW_RESET_TO_THREAD)
        uint8_t rtos_context_switch_type_t;

/**
 * Context switch trace entry type
 *
 * Fields listed from most significant bit to least significant bit:
 * <target CPU mode>: 4 bits
 * <from CPU mode> : 4 bits
 * <context switch type>: 4 bits
 * <switched-out reason>: 4 bits
 * <priority>: 6 bits
 * <context type>: 2 bits
 * <relative ID>: 8 bits
 */
typedef uint32_t fdc_context_switch_trace_entry_t;

/**
 * - For unprivileged threads, the context Id is an index in
 *   g_McRTOS_p->rts_app_threads[] 
 * - For privileged threads, the context Id is an index in
 *   g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()].cpc_system_threads[] 
 * - For interrupts, the context Id is the interrupt channel
 */
#define FDC_CST_CONTEXT_ID_MASK  MULTI_BIT_MASK(7, 0)
#define FDC_CST_CONTEXT_ID_SHIFT 0

/**
 * Context type. Values from enum fdc_cst_context_type below.
 */
#define FDC_CST_CONTEXT_TYPE_MASK  MULTI_BIT_MASK(9, 8)
#define FDC_CST_CONTEXT_TYPE_SHIFT 8

enum fdc_cst_context_type
{
    FDC_CST_RESET =                 0x0,
    FDC_CST_APPLICATION_THREAD =    0x1,
    FDC_CST_SYSTEM_THREAD =         0x2,
    FDC_CST_INTERRUPT =             0x3,
};

/**
 * - For threads, it is the current thread priority
 * - For interrupts, it is the interrupt priority
 */
#define FDC_CST_CONTEXT_PRIORITY_MASK   MULTI_BIT_MASK(15, 10)
#define FDC_CST_CONTEXT_PRIORITY_SHIFT  10

/**
 * Last switched out reason. Value taken from the execution context's field
 * ctx_last_switched_out_reason. 
 * (Value from enum rtos_execution_context_switched_out_reasons)
 */
#define FDC_CST_LAST_SWITCHED_OUT_REASON_MASK   MULTI_BIT_MASK(19, 16)
#define FDC_CST_LAST_SWITCHED_OUT_REASON_SHIFT  16

/**
 * Context switch type. Values defined in arm_defs.h (RTOS_CSW_...)
 */ 
#define FDC_CST_CONTEXT_SWITCH_TYPE_MASK        MULTI_BIT_MASK(23, 20)
#define FDC_CST_CONTEXT_SWITCH_TYPE_SHIFT       20

#if DEFINED_ARM_CLASSIC_ARCH()
    /**
     * Lowest 4 bits of the current ARM CPU mode (the actual mode is 1bbbb)
     */
#   define FDC_CST_CURRENT_CPU_MODE_MASK          MULTI_BIT_MASK(27, 24)
#   define FDC_CST_CURRENT_CPU_MODE_SHIFT         24

    /**
     * Lowest 4 bits of the target ARM CPU mode (the actual mode is 1bbbb)
     */
#   define FDC_CST_TARGET_CPU_MODE_MASK          MULTI_BIT_MASK(31, 28)
#   define FDC_CST_TARGET_CPU_MODE_SHIFT         28
#endif


/**
 * Captured failure data record
 */
struct failure_record { 
    /**
     * sequence number
     */
    uint32_t    fr_seq_number; 

    /**
     * Address in the code where the failure happened
     */ 
    uint32_t    *fr_failure_location;

    /**
     * Failure data arguments
     */
    uintptr_t   fr_failure_args[2];

    /**
     * Pointer to failure description string literal
     */
    const char  *fr_failure_descrption_str;

    /**
     * CPU status register
     */ 
    uint32_t    fr_cpu_status_register;

    /**
     * Pointer to the execution context where the error happened
     */
    struct rtos_execution_context      *fr_execution_context_p;
};

// total size of failure_record must be at most 32 bytes 
C_ASSERT(sizeof(struct failure_record) <= 32);

/* 
 * Types of unexpected exceptions
 */
enum unexpected_exception_types
{
    UET_INVALID_EXCEPTION_TYPE =    0x0,
#if DEFINED_ARM_CLASSIC_ARCH()
    UET_UNDEFINED_INSTRUCTION =     0x1,
    UET_DATA_ABORT =                0x2,
    UET_PREFETCH_ABORT =            0x3
#elif DEFINED_ARM_CORTEX_M_ARCH()
    UET_HARD_FAULT =                0x1,
#endif
};

/**
 * Unexpected exception failure data capture record
 */
struct unexpected_exception_record { 
    /**
     * sequence number
     */
    uint32_t    uer_seq_number; 

    /**
     * Type of exception (value from enum exception_types)
     */ 
    uint8_t     uer_exception_type;

    /**
     * Reserved fields filling alignment holes for now
     */
    uint8_t     uer_reserved1;
    uint16_t    uer_reserved2;

    /**
     * Address of the instruction that caused the exception
     */ 
    uint32_t    *uer_location;

    /**
     * Data argument for the exception
     */
    uint32_t    uer_arg;

    /**
     * CPU status register
     */
    uint32_t    uer_cpu_status_register;

    /**
     * Current execution context when the failure happened
     */ 
    struct rtos_execution_context    *uer_execution_context_p;
};

/**
 * Failure data capture information block
 */ 
struct fdc_info
{
#   define      FDC_INFO_SIGNATURE  UINT32_C(0xFACEFACE)
    uint32_t    fdc_signature1;

    /**
     * Debug flags
     */
    volatile uint32_t fdc_asserts_failures_breakpoint_on : 1;
    volatile uint32_t fdc_error_breakpoint_on : 1;
    volatile uint32_t fdc_reserved_flags : 30;

    /**
     * Number of failures captured since last reset. Failure data is captured only
     * for the last RTOS_MAX_NUM_FAILURE_RECORDS failures.
     */
    uint32_t fdc_failures_count;

    /**
     * Number of unexpected exception failures since last reset. Failure data is captured only
     * for the last RTOS_MAX_NUM_UNEXPECTED_EXCEPTION_RECORDS exceptions.
     */
    uint32_t fdc_unexpected_exceptions_count;

    /**
     * Index of next entry to fill in fdc_failures
     */ 
    uint8_t fdc_failure_cursor;

    /**
     * Index of next entry to fill in fdc_unexpected_exceptions
     */ 
    uint8_t fdc_unexpected_exceptions_cursor;

    /**
     * Index of next entry to fill in fdc_context_switch_trace_buffer
     */
    uint16_t fdc_context_switch_trace_cursor;

    /** 
     * Total number of context switches, since boot
     */ 
    uint32_t fdc_context_switch_count; 

    /**
     * Captured failures circular buffer
     */ 
    struct failure_record fdc_failures[RTOS_MAX_NUM_FAILURE_RECORDS];

    /**
     * Unexpected exceptions failures circular buffer
     */ 
    struct unexpected_exception_record fdc_unexpected_exceptions
                                    [RTOS_MAX_NUM_UNEXPECTED_EXCEPTION_RECORDS];

    /**
     * Counters of interrupts received for each interrupt channel
     */
#   if DEFINED_ARM_CORTEX_M_ARCH()
    /*
     * One entry before the array, to allow access to 
     * fdc_interrupt_channel_counters[-1] for the systick interrupt.
     */
    uint32_t fdc_systick_interrupt_counter;
#   endif
    uint32_t fdc_interrupt_channel_counters[SOC_NUM_INTERRUPT_CHANNELS];

    /**
     * Interrupts & task context switches trace circular buffer
     */ 
    fdc_context_switch_trace_entry_t fdc_context_switch_trace_buffer
                                        [RTOS_NUM_CONTEXT_SWITCH_TRACE_BUFFER_ENTRIES];

    uint32_t    fdc_signature2;
};

C_ASSERT(sizeof(struct fdc_info) <= RTOS_MAX_FDC_INFO_SIZE);
C_ASSERT(RTOS_NUM_CONTEXT_SWITCH_TRACE_BUFFER_ENTRIES <= UINT16_MAX);

#ifdef _RELIABILITY_CHECKS_

#define FDC_INFO_INITIALIZER(_fdc_info) \
    _fdc_info = {                                                       \
        .fdc_signature1 = FDC_INFO_SIGNATURE,                           \
        .fdc_asserts_failures_breakpoint_on  = true,                    \
        .fdc_error_breakpoint_on = true,                                \
        .fdc_failures_count = 0,                                        \
        .fdc_unexpected_exceptions_count = 0,                           \
        .fdc_failure_cursor = 0,                                        \
        .fdc_unexpected_exceptions_cursor = 0,                          \
        .fdc_context_switch_trace_cursor = 0,                           \
        .fdc_context_switch_count = 0,                                  \
        .fdc_signature2 = FDC_INFO_SIGNATURE                            \
    }

#else

#define FDC_INFO_INITIALIZER(_fdc_info) 

#endif /* _RELIABILITY_CHECKS_ */

bool is_cpu_little_endian(void);

void capture_assert_failure(
        const char *cond_str,
        uintptr_t arg1,
        uintptr_t arg2);

#if DEFINED_ARM_CORTEX_M_ARCH()
void capture_unexpected_hard_fault(
    void *location, uintptr_t arg, uint32_t psr);
#endif

fdc_error_t capture_fdc_error(
        const char *error_description,
        uintptr_t arg1,
        uintptr_t arg2);

void check_isr_reset_entry_asserts(void);

void check_rtos_execution_context_invariants(
    _IN_ const struct rtos_execution_context *rtos_execution_context_p);

void
check_rtos_execution_context_cpu_registers(
    _IN_ const struct rtos_execution_context *rtos_execution_context_p,
    _IN_ rtos_context_switch_type_t ctx_switch_type);

void fdc_trace_rtos_context_switch(
    _IN_ const struct rtos_execution_context *rtos_execution_context_p,
    _IN_ rtos_context_switch_type_t ctx_switch_type);

void capture_unexpected_undefined_instruction(
     void *location, uintptr_t arg, uint32_t cpsr);

void capture_unexpected_data_abort(
     void *location, uintptr_t arg, uint32_t cpsr);

void capture_unexpected_prefetch_abort(
     void *location, uintptr_t arg, uint32_t cpsr);

_NEVER_RETURN_
void fatal_error_handler(_IN_ fdc_error_t fdc_error);

void
check_rtos_public_kernel_service_preconditions(bool thread_callers_only);

void
check_rtos_interrupt_entry_preconditions(
    _IN_ struct rtos_interrupt *rtos_interrupt_p);

void
check_rtos_interrupt_e_handler_preconditions(
    _IN_ const struct rtos_interrupt *rtos_interrupt_p);

void
check_synchronous_context_switch_preconditions(
    _IN_ const struct rtos_execution_context *current_execution_context_p);

#endif /* _FAILURE_DATA_CAPTURE_H */

