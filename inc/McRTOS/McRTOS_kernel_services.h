/**
 * @file McRTOS_kernel_services.h
 *
 * McRTOS kernel services
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */
#ifndef _McRTOS_KERNEL_SERVICES_H
#define _McRTOS_KERNEL_SERVICES_H

#include <stdint.h>
#include "McRTOS.h"
#include "arm_defs.h"
#include "generic_list.h"
#include "hardware_abstractions.h"
#include "compile_time_checks.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Calculate difference between two values of McRTOS clock ticks
 */
#define RTOS_TICKS_DELTA(_begin_ticks, _end_ticks) \
        ((rtos_ticks_t)((int32_t)(_end_ticks) - \
                         (int32_t)(_begin_ticks)))

#define MILLISECONDS_TO_TICKS(_milli_secs) \
        ((rtos_ticks_t)HOW_MANY(_milli_secs, RTOS_MILLISECONDS_PER_TICK))

struct rtos_interrupt;
struct rtos_thread_execution_stack;

/**
 * CPU integer register type
 */
typedef uint32_t cpu_register_t;

/**
 * CPU status register
 */
typedef uint32_t cpu_status_register_t;

/**
 * Time in microseconds type
 */
typedef _RANGE_(0, (RTOS_MILLISECONDS_PER_TICK * 1000) - 1)
        uint32_t rtos_microseconds_t;

/**
 * Range type for system call numbers
 */
typedef _RANGE_(0, RTOS_NUM_SYSTEM_CALLS - 1)
        uint8_t rtos_system_call_number_t;

C_ASSERT(RTOS_NUM_SYSTEM_CALLS - 1 <= UINT8_MAX);

/**
 * CPU mode range type
 */
typedef _RANGE_(RTOS_UNPRIVILEGED_THREAD_MODE, RTOS_INTERRUPT_MODE)
        uint8_t rtos_cpu_mode_t;

/**
 * Execution context type
 */
typedef _RANGE_(RTOS_RESET_CONTEXT, RTOS_INTERRUPT_CONTEXT)
        uint8_t rtos_execution_context_type_t;

/**
 * Thread states
 */
enum rtos_thread_states
{
    RTOS_INVALID_THREAD_STATE =             0x0,
    RTOS_THREAD_CREATED =                   0x1,
    RTOS_THREAD_RUNNABLE =                  0x2,
    RTOS_THREAD_RUNNING =                   0x3,
    RTOS_THREAD_BLOCKED_ON_MUTEX =          0x4,
    RTOS_THREAD_BLOCKED_ON_CONDVAR =        0x5,
    RTOS_THREAD_ABORTED =                   0x6,
    RTOS_THREAD_BEING_REQUEUED =            0x7,

    /*
     * Last dummy value. New state must be added above here:
     */
    RTOS_THREAD_LAST_INVALID_VALUE
};

/*
 * Greatest valid state value cannot be larger than 0xf, as all valid states
 * must fit in 4 bits
 */
C_ASSERT(RTOS_THREAD_LAST_INVALID_VALUE <= 0x10);

typedef _RANGE_(RTOS_THREAD_RUNNABLE, RTOS_THREAD_TERMINATED)
        uint8_t rtos_thread_state_t;

/**
 * Range type for thread time slice in timer ticks
 */
typedef _RANGE_(0, RTOS_THREAD_TIME_SLICE_IN_TICKS)
        uint8_t rtos_thread_time_slice_in_ticks_t;

/**
 * Deadline time in milliseconds type
 */
typedef uint16_t rtos_deadline_ms_t;

/**
 * Block of parameters for registering an interrupt with McRTOS
 */
struct rtos_interrupt_registration_params
{
    /**
     * Pointer to interrupt name string for debugging purposes
     */
    const char *irp_name_p;

    /**
     * Pointer to the interrupt service routine (ISR)
     */
    isr_function_t *irp_isr_function_p;

    /**
     * Argument to be passed to the interrupt's clear_interrupt_xxx() and
     * interrupt_handler_xxx() functions from the interrupt's ISR function.
     */
    void *irp_arg_p;

    /**
     * Interrupt channel in the Interrupt controller
     */
    interrupt_channel_t irp_channel;

    /**
     * Interrupt priority at the Interrupt controller
     */
    interrupt_prio_t irp_priority;

    /**
     * CPU ID of the CPU core where this interrupt is to fire.
     */
    cpu_id_t irp_cpu_id;
};

/**
 * McRTOS execution context
 */
struct rtos_execution_context
{
#   define      RTOS_EXECUTION_CONTEXT_SIGNATURE  GEN_SIGNATURE('C', 'T', 'X', 'T')
    uint32_t    ctx_signature;

    /**
     * Pointer to the symbolic name of the execution context for debugging purposes
     */
    const char *ctx_name_p;

    /**
     * CPU execution mode
     */
    rtos_cpu_mode_t ctx_cpu_mode;

    /**
     * Context type
     */
    rtos_execution_context_type_t ctx_context_type;

    /**
     * CPU core on which this thread runs
     */
    cpu_id_t ctx_cpu_id;

    /**
     * Reason this context got switched out last time
     * (Value from  enum rtos_execution_context_switched_out_reasons)
     */
    uint8_t ctx_last_switched_out_reason;

#if DEFINED_ARM_CLASSIC_ARCH()
    /**
     * CPU general registers, including stack pointer and program counter
     *
     * NOTE: The RTOS_ENTER_ISR() assembly language macro uses this field.
     */
    cpu_register_t ctx_cpu_registers[CPU_NUM_REGISTERS];

#elif DEFINED_ARM_CORTEX_M_ARCH()
    /**
     * For Cortex-M, registers R0-R3, R12, LR, PC and PSR are automatically saved
     * on the stack by the CPU, upon entering an exception. So, only the remaining
     * registers need to be saved here.
     */
    struct cpu_saved_registers {
        cpu_register_t cpu_reg_r4;
        cpu_register_t cpu_reg_r5;
        cpu_register_t cpu_reg_r6;
        cpu_register_t cpu_reg_r7;
        cpu_register_t cpu_reg_r8;
        cpu_register_t cpu_reg_r9;
        cpu_register_t cpu_reg_r10;
        cpu_register_t cpu_reg_r11;
        cpu_register_t cpu_reg_msp;
        cpu_register_t cpu_reg_psp;
        cpu_register_t cpu_reg_lr_on_exc_entry;
        cpu_register_t cpu_reg_control;
    } ctx_cpu_saved_registers;

#else
#   error "CPU architrecture not supported"
#endif

    /**
     * Switched-out reason history
     */
    uint32_t ctx_switched_out_reason_history;

    /**
     * Pointer to the execution context that last preempted this context
     */
    struct rtos_execution_context *ctx_last_preempted_by_p;

    /**
     * Pointer to the top end of the execution stack for this context.
     * Since the stack is full descending, the context's stack pointer register
     * must always be >= ctx_execution_stack_top_end_p.
     */
    rtos_execution_stack_entry_t *ctx_execution_stack_top_end_p;

    /**
     * Pointer to the bottom end of the execution stack for this context
     * Since the stack is full descending, the context's stack pointer register
     * must always be <= ctx_execution_stack_bottom_end_p.
     */
    rtos_execution_stack_entry_t *ctx_execution_stack_bottom_end_p;

    /**
     * High water mark of the context's stack pointer at context switch
     */
    rtos_execution_stack_entry_t *ctx_stack_pointer_high_water_mark_p;

    /**
     * Number of times that this execution context has been preempted
     * by a higher priority execution context.
     */
    uint32_t ctx_preempted_counter;

    /**
     * Number of times that this execution context has been switched out
     */
    uint32_t ctx_switched_out_counter;

#   ifdef _CPU_CYCLES_MEASURE_
    /**
     * Time stamp (in CPU clock cycles) of the last time this context was
     * switched-in
     */
    cpu_clock_cycles_t ctx_last_switched_in_time_stamp;

    /**
     * Accumulated CPU usage for this context in milliseconds
     */
    uint32_t ctx_accumulated_cpu_usage_milliseconds;

    /**
     * Accumulated CPU usage for this context in CPU cycles under 1 millisecond
     */
    uint32_t ctx_accumulated_cpu_usage_cycles;
#   endif

    /**
     * Time stamp (in ticks since boot) of the last time this context was
     * switched-out
     */
    rtos_ticks_t ctx_last_switched_out_time_stamp_in_ticks;

    /**
     * Pre-filled trace entry for context switch tracing
     */
    fdc_context_switch_trace_entry_t ctx_prefilled_trace_entry;

    /**
     * Embedded list node used to insert this execution context in the list
     * of all execution contexts for the corresponding CPU.
     */
    struct glist_node ctx_list_node;

    /**
     * Embedded list node used to insert this execution context in the preemption
     * chain maintained for each CPU.
     */
    struct glist_node ctx_preemption_chain_node;
};

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_mode) ==
    RTOS_CTX_CPU_MODE_OFFSET);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_context_type) ==
    RTOS_CTX_CONTEXT_TYPE_OFFSET);

#if DEFINED_ARM_CLASSIC_ARCH()
C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_registers) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET);

#elif DEFINED_ARM_CORTEX_M_ARCH()
C_ASSERT(
    sizeof(struct cpu_saved_registers) ==
    CPU_NUM_SAVED_REGISTERS * sizeof(cpu_register_t));

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_r4) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_R4*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_r5) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_R5*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_r6) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_R6*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_r7) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_R7*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_r8) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_R8*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_r9) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_R9*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_r10) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_R10*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_r11) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_R11*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_msp) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_MSP*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_psp) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_PSP*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_LR_ON_EXC_ENTRY*ARM_CPU_WORD_SIZE_IN_BYTES);

C_ASSERT(
    offsetof(struct rtos_execution_context, ctx_cpu_saved_registers.cpu_reg_control) ==
    RTOS_CTX_CPU_REGISTERS_OFFSET + CPU_REG_CONTROL*ARM_CPU_WORD_SIZE_IN_BYTES);
#endif

/**
 * McRTOS timer object
 */
struct rtos_timer
{
#   define      RTOS_TIMER_SIGNATURE  GEN_SIGNATURE('T', 'I', 'M', 'R')
    uint32_t    tmr_signature;

    /**
     * Pointer to the symbolic name of the timer for debugging purposes
     */
    const char *tmr_name_p;

    /**
     * CPU core on which this timer fires
     */
    cpu_id_t tmr_cpu_id;

    /**
     * Milliseconds left before the timer expires
     */
    rtos_milliseconds_t tmr_time_to_expire;

    /**
     * Pointer to timer callback function
     */
    rtos_timer_function_t *tmr_callback_function_p;

    /**
     * Embedded list node used to insert this timer in a timer wheel
     * hash chain
     */
    struct glist_node tmr_list_node;

} __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_timer) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);

/**
 * McRTOS mutex object
 */
struct rtos_mutex
{
#   define      RTOS_MUTEX_SIGNATURE  GEN_SIGNATURE('M', 'U', 'T', 'X')
    uint32_t    mtx_signature;

    /**
     * Pointer to the symbolic name of the mutex for debugging purposes
     */
    const char *mtx_name_p;

    /**
     * CPU core on which this mutex can be acquired
     */
    cpu_id_t    mtx_cpu_id;

    /**
     * Head node of the thread queue for threads waiting on this mutex
     */
    struct glist_node mtx_waiting_thread_queue_anchor;

    /**
     * Pointer to OS thread that currently owns the mutex or NULL
     */
    struct rtos_thread *mtx_owner_p;
}  __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_mutex) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);

/**
 * McRTOS condition variable object
 */
struct rtos_condvar
{
#   define      RTOS_CONDVAR_SIGNATURE  GEN_SIGNATURE('C', 'O', 'N', 'D')
    uint32_t    cv_signature;

    /**
     * Pointer to the symbolic name of the condvar for debugging purposes
     */
    const char *cv_name_p;

    /**
     * CPU core on which this mutex can be acquired
     */
    cpu_id_t    cv_cpu_id;

    /**
     * Pointer to the mutex that was released when rtos_condvar_wait()
     * call was entered for this condvar, and which needs to be reacquired
     * before rtos_condvar_wait() returns.
     *
     * NOTE: All threads calling  rtos_condvar_wait() for the same condvar
     * must always use the same mutex.
     */
    struct rtos_mutex *cv_released_mutex_p;

    /**
     * Head node of the thread queue for threads waiting on this condvar
     */
    struct glist_node cv_waiting_thread_queue_anchor;

} __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_condvar) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);

/**
 * McRTOS generic circular buffer
 */
struct rtos_circular_buffer
{
#   define RTOS_POINTER_CIRCULAR_BUFFER_SIGNATURE   GEN_SIGNATURE('C', 'B', 'P', 'T')
#   define RTOS_BYTE_CIRCULAR_BUFFER_SIGNATURE      GEN_SIGNATURE('C', 'B', 'B', 'Y')
    const uint32_t cb_signature;

    /**
     * Pointer to the symbolic name of the circular buffer for debugging purposes
     */
    const char *const cb_name_p;

    /**
     * Total number of entries of the circular buffer
     */
    const uint16_t cb_num_entries;

    /**
     * Number of entries currently filled
     */
    volatile uint16_t cb_entries_filled;

    /**
     * Next entry filled to read
     */
    uint16_t cb_read_cursor;

    /**
     * Next entry unfilled to write
     */
    uint16_t cb_write_cursor;

    /**
     * Pointer to the storage array for the circular buffer entries
     */
    void *const cb_storage_array_p;

    /**
     * Pointer to mutex to serialize access to the circular buffer, or
     * NULL if serialization is to be done by disabling interrupts.
     */
    struct rtos_mutex *const cb_mutex_p;

    /**
     * Condvar signaled by writers when the circular buffer becomes not empty
     */
    struct rtos_condvar cb_not_empty_condvar;

    /**
     * Condvar signaled by readers when the circular buffer becomes not full.
     */
    struct rtos_condvar cb_not_full_condvar;
};

/**
 * McRTOS 8-bit message channel object
 */
struct rtos_8bit_msg_channel
{
#   define      RTOS_8BIT_MSG_CHANNEL_SIGNATURE  GEN_SIGNATURE('C', 'H', 'N', 'B')
    uint32_t    mch_signature;

    /**
     * Pointer to array of message entries
     */
    uint8_t     *mch_entries_p;

    struct rtos_circular_buffer mch_circular_buffer;

} __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_8bit_msg_channel) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);

/**
 * McRTOS 16-bit message channel object
 */
struct rtos_16bit_msg_channel
{
#   define      RTOS_16BIT_MSG_CHANNEL_SIGNATURE  GEN_SIGNATURE('C', 'H', 'N', 'H')
    uint32_t    mch_signature;

    /**
     * Pointer to array of message entries
     */
    uint16_t     *mch_entries_p;

    struct rtos_circular_buffer mch_circular_buffer;

} __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_16bit_msg_channel) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);

/**
 * McRTOS 32-bit message channel object
 */
struct rtos_32bit_msg_channel
{
#   define      RTOS_32BIT_MSG_CHANNEL_SIGNATURE  GEN_SIGNATURE('C', 'H', 'N', 'W')
    uint32_t    mch_signature;

    /**
     * Pointer to array of message entries
     */
    uint32_t     *mch_entries_p;

    struct rtos_circular_buffer mch_circular_buffer;

} __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_32bit_msg_channel) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);

/**
 * McRTOS object pool object
 */
struct rtos_object_pool
{
#   define      RTOS_OBJECT_POOL_SIGNATURE  GEN_SIGNATURE('P', 'O', 'O', 'L')
    uint32_t    opl_signature;

    /**
     * Pointer to array of object pointers
     */
    void        **opl_object_pointers_p;

    struct rtos_circular_buffer opl_free_objects;

}  __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_object_pool) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);

/**
  * FPU context saved for a thread that is using the FPU when it is
  * switched out
  */
 struct fpu_context {
     /**
      * Saved FPU single-precision registers
      */
     uint32_t fpu_registers[FPU_NUM_SINGLE_REGISTERS];

     /**
      * Saved FPU status register
      */
     uint32_t fpscr;
 };

/**
 * McRTOS thread object
 */
struct rtos_thread
{
#   define      RTOS_THREAD_SIGNATURE  GEN_SIGNATURE('T', 'H', 'R', 'D')
    uint32_t    thr_signature;

    /**
     * Embedded list node used to insert this thread in a thread queue
     */
    struct glist_node thr_list_node;

    /**
     * Execution context for the thread
     */
    struct rtos_execution_context thr_execution_context;

    /**
     * Pointer to the thread function (top-level function executed by the
     * thread)
     */
    rtos_thread_function_t *thr_function_p;

    /**
     * Argument to be passed in to the thread function
     */
    void *thr_function_arg_p;

    /**
     * Pointer to the execution stack for the thread
     */
    struct rtos_thread_execution_stack *thr_execution_stack_p;

    /**
     * Abort status passed in to rtos_thread_abort(), if ever called
     * by the thread
     */
    fdc_error_t thr_abort_status;

    /**
     * Thread state
     */
    rtos_thread_state_t thr_state;

    /**
     * Number of timer ticks left for the thread's time slice
     */
    rtos_thread_time_slice_in_ticks_t   thr_time_slice_ticks_left;

    /**
     * Base thread priority
     */
    rtos_thread_prio_t thr_base_priority;

    /**
     * Current thread priority
     */
    rtos_thread_prio_t thr_current_priority;

    /**
     * Deadline to run in milliseconds once it becomes runnable.
     * 0 means no deadline.
     */
    rtos_deadline_ms_t thr_deadline_to_run;

#   ifdef LCD_SUPPORTED
    /**
     * LCD channel assigned to the thread
     */
    rtos_lcd_channels_t thr_lcd_channel;
#   endif

    /**
     * Number of MPU data regions currently defined for the thread
     */
    uint8_t thr_num_mpu_data_regions;

    /**
     * Thread state history
     */
    uint32_t thr_state_history;

    /**
     * Thread priority history
     */
    uint32_t thr_priority_history;

    /**
     * Number of times that this thread has been preempted because it used all
     * of its time slice
     */
    uint32_t thr_preempted_by_time_slice_count;

    /**
     * Number of times that this thread has been preempted by a higher priority
     * thread.
     */
    uint32_t thr_preempted_by_other_thread_count;

    /**
     * Number of times that this thread has been self-preempted
     */
    uint32_t thr_self_preempted_count;

    /**
     * Number of mutexes currently owned by the thread
     */
    uint32_t thr_owned_mutexes_count;

    /**
     * Number of rtos_k_thread_enable_fpu() calls unpaired with
     * corresponding rtos_k_thread_disable_fpu()
     *
     * NOTE: A count is used instead of a boolean flag, to allow
     * nested calls to rtos_k_thread_enable_fpu()/rtos_k_thread_disable_fpu()
     */
    uint8_t thr_fpu_enable_count;

    uint8_t thr_reserved1;
    uint16_t thr_reserved2;

    union
    {
        /**
         * Pointer to the the synchronization object (mutex or condvar)
         * the thread is blocked on, if any.
         */
        void *thr_blocked_on_p;

        /**
         * Mutex the thread is blocked on, if any.
         */
        struct rtos_mutex *thr_blocked_on_mutex_p;

        /**
         * Condvar the thread is blocked on, if any.
         */
        struct rtos_condvar *thr_blocked_on_condvar_p;
    };

    /**
     * Timer for this thread. It is started by when
     * rtos_k_thread_delay() is called for this thread.
     * It is also started when rtos_k_condvar_wait*() is
     * called with a timeout.
     */
    struct rtos_timer __attribute__ ((aligned(sizeof(uint32_t)))) thr_timer;

    /**
     * Built-in condvar for this thread. It is signaled from the
     * tick timer interrupt when this thread's delay timer expires.
     * It can also be signaled from other threads or interrupts
     * by calling rtos_k_thread_condvar_signal().
     */
    struct rtos_condvar  __attribute__ ((aligned(sizeof(uint32_t)))) thr_condvar;

    /**
     * MPU data regions accessible from the thread, including one region
     * for the thread's stack
     */
    struct mpu_region_range
	    thr_mpu_data_regions[RTOS_MAX_MPU_THREAD_DATA_REGIONS];

    /**
     * Saved FPU context on last context switch, if the thread was using the FPU
     */
    struct fpu_context thr_saved_fpu_context;

} __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_thread) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);


/**
 * Generic queue
 */
struct rtos_queue {
#   define RTOS_QUEUE_SIGNATURE  GEN_SIGNATURE('Q', 'U', 'E', 'U')
    uint32_t signature;

    /**
     * Flag indicating if serialization to access the queue is to be done
     * using a mutex (true), or by disabling interrupts (false)
     */
    bool use_mutex;

    /**
     * Anchor node of the linked list that represents the queue
     */
    struct glist_node list_anchor;

    /**
     * Mutex to serialize access to the queue. It is only meaningful if
     * 'use_mutex' is true.
     */
    struct rtos_mutex mutex;

    /**
     * Condition variable to be signaled when the queue becomes non-empty
     */
    struct rtos_condvar non_empty_condvar;
};


_MAY_NOT_RETURN_
void
rtos_k_thread_init(
    _IN_ const struct rtos_thread_creation_params *params_p,    
    _IN_ struct rtos_thread_execution_stack *thread_stack_p,
    _OUT_ struct rtos_thread *rtos_thread_p);

_THREAD_CALLERS_ONLY_
void
rtos_k_thread_delay(_IN_ rtos_milliseconds_t num_milliseconds);

_THREAD_CALLERS_ONLY_
void
rtos_k_thread_micro_delay(_IN_ rtos_microseconds_t num_microseconds);

rtos_ticks_t
rtos_k_get_ticks(void);

_NEVER_RETURN_
_THREAD_CALLERS_ONLY_
void
rtos_k_thread_abort(_IN_ fdc_error_t fdc_error);

_THREAD_CALLERS_ONLY_
const struct rtos_thread *
rtos_k_thread_self(void);

bool
rtos_k_caller_is_thread(void);

const char *
rtos_k_thread_name(
    _IN_ const struct rtos_thread *rtos_thread_p);

_THREAD_CALLERS_ONLY_
void
rtos_k_thread_yield(void);

_THREAD_CALLERS_ONLY_
void
rtos_k_thread_enable_fpu(void);

_THREAD_CALLERS_ONLY_
void
rtos_k_thread_disable_fpu(void);

_THREAD_CALLERS_ONLY_
void
rtos_k_thread_condvar_wait(
    _IN_ struct rtos_mutex *rtos_mutex_p);

_THREAD_CALLERS_ONLY_
void
rtos_k_thread_condvar_wait_interrupt(void);

void
rtos_k_thread_condvar_signal(
    _IN_ struct rtos_thread *rtos_thread_p);

void
rtos_k_mutex_init(
    _IN_  const char *mutex_name_p,
    _OUT_ struct rtos_mutex *rtos_mutex_p);

_THREAD_CALLERS_ONLY_
void
rtos_k_mutex_acquire(
    _IN_ struct rtos_mutex *rtos_mutex_p);

_THREAD_CALLERS_ONLY_
void
rtos_k_mutex_release(
    _IN_ struct rtos_mutex *rtos_mutex_p);

void
rtos_k_condvar_init(
    _IN_  const char *condvar_name_p,
    _OUT_ struct rtos_condvar *rtos_condvar_p);

_THREAD_CALLERS_ONLY_
void
rtos_k_condvar_wait(
    _INOUT_ struct rtos_condvar *rtos_condvar_p,
    _IN_    struct rtos_mutex *rtos_mutex_p,
    _INOUT_ rtos_milliseconds_t *timeout_ms_p);

_THREAD_CALLERS_ONLY_
void
rtos_k_condvar_wait_intr_disabled(
    _INOUT_ struct rtos_condvar *rtos_condvar_p,
    _INOUT_ rtos_milliseconds_t *timeout_ms_p);

void
rtos_k_condvar_signal(
    _INOUT_ struct rtos_condvar *rtos_condvar_p);

void
rtos_k_condvar_broadcast(
    _INOUT_ struct rtos_condvar *rtos_condvar_p);

void
rtos_k_timer_init(
    _IN_  const char *timer_name_p,
    _IN_  rtos_timer_function_t *timer_function_p,
    _OUT_ struct rtos_timer *rtos_timer_p);

void
rtos_k_timer_start(
    _INOUT_ struct rtos_timer *rtos_timer_p,
    _IN_ rtos_milliseconds_t expiration_time_in_ms);

void
rtos_k_timer_stop(
    _INOUT_ struct rtos_timer *rtos_timer_p);

cpu_status_register_t
rtos_k_disable_cpu_interrupts(void);

void
rtos_k_restore_cpu_interrupts(cpu_status_register_t cpu_status_register);

uint32_t
rtos_k_atomic_fetch_add_uint32(
    volatile uint32_t *counter_p, uint32_t value);

uint32_t
rtos_k_atomic_fetch_sub_uint32(
    volatile uint32_t *counter_p, uint32_t value);

uint16_t
rtos_k_atomic_fetch_add_uint16(
    volatile uint16_t *counter_p, uint16_t value);

uint16_t
rtos_k_atomic_fetch_sub_uint16(
    volatile uint16_t *counter_p, uint16_t value);

uint8_t
rtos_k_atomic_fetch_add_uint8(
    volatile uint8_t *counter_p, uint8_t value);

uint8_t
rtos_k_atomic_fetch_sub_uint8(
    volatile uint8_t *counter_p, uint8_t value);

rtos_thread_prio_t
rtos_k_find_highest_thread_priority(
    rtos_thread_prio_bitmap_t rtos_thread_prio_bitmap);

void
rtos_k_register_interrupt(
    _IN_  const struct rtos_interrupt_registration_params *params_p,
    _OUT_ struct rtos_interrupt **rtos_interrupt_p);

cpu_register_t
rtos_k_enter_interrupt(
    struct rtos_interrupt *rtos_interrupt_p);

_NEVER_RETURN_
void
rtos_k_exit_interrupt(void);

fdc_error_t
rtos_k_enter_privileged_mode(
    _IN_ rtos_system_call_number_t system_call_number);

void
rtos_k_exit_privileged_mode(void);

_MAY_NOT_RETURN_
_THREAD_CALLERS_ONLY_
void
rtos_k_synchronous_context_switch(
    struct rtos_execution_context *current_execution_context_p);

_NEVER_RETURN_
void rtos_k_restore_execution_context(
        _IN_ const struct rtos_execution_context *execution_context_p,
        _IN_ rtos_context_switch_type_t ctx_switch_type);

void
rtos_k_capture_failure_data(
    _IN_ const char *failure_str,
    _IN_ uintptr_t arg1,
    _IN_ uintptr_t arg2,
    _IN_ void *failure_location);

void
rtos_k_set_fdc_params(
    _IN_ bool assert_break_point_on,
    _IN_ bool exception_debugger_on);

_THREAD_CALLERS_ONLY_
void
rtos_k_console_putchar(
    _UNUSED_ void *unused_arg_p,
    _IN_ uint8_t c);

void
rtos_k_console_putchar_with_polling(
    _UNUSED_ void *unused_arg_p,
    _IN_ uint8_t c);

_THREAD_CALLERS_ONLY_
uint8_t
rtos_k_console_getchar(void);

void
rtos_k_lcd_putchar(
    _INOUT_ struct rtos_lcd_putchar_attributes *lcd_putchar_attr_p,
    _IN_ uint8_t c);

lcd_color_t
rtos_k_lcd_draw_tile(
    _IN_ lcd_x_t x,
    _IN_ lcd_y_t y,
    _IN_ lcd_color_t fill_color);

fdc_error_t
rtos_k_mpu_add_thread_data_region(
    void *start_addr,
    size_t size,
    bool read_only);

void
rtos_k_mpu_remove_thread_data_region(void);

fdc_error_t
rtos_k_app_system_call(
    _IN_ rtos_app_system_call_function_t *rtos_app_system_call_function_p,
    _INOUT_ void *arg_p);

void rtos_tick_timer_interrupt_handler(
    struct rtos_interrupt *timer_interrupt_p);

void rtos_k_pointer_circular_buffer_init(
        _IN_  const char *name_p,
        _IN_ uint16_t num_entries,
        _IN_ void **storage_array_p,
        _IN_ struct rtos_mutex *cb_mutex_p,
        _OUT_ struct rtos_circular_buffer *circ_buf_p);

bool rtos_k_pointer_circular_buffer_write(
        _INOUT_ struct rtos_circular_buffer *circ_buf_p,
        _IN_ void *entry_value,
        _IN_ bool wait_if_full);

bool rtos_k_pointer_circular_buffer_read(
        _INOUT_ struct rtos_circular_buffer *circ_buf_p,
        _OUT_ void **entry_value_p,
        _IN_ bool wait_if_empty,
	_INOUT_ rtos_milliseconds_t *timeout_ms_p);

void rtos_k_byte_circular_buffer_init(
        _IN_  const char *name_p,
        _IN_ uint16_t num_entries,
        _IN_ uint8_t *storage_array_p,
        _IN_ struct rtos_mutex *cb_mutex_p,
        _OUT_ struct rtos_circular_buffer *circ_buf_p);

bool rtos_k_byte_circular_buffer_write(
        _INOUT_ struct rtos_circular_buffer *circ_buf_p,
        _IN_ uint8_t entry_value,
        _IN_ bool wait_if_full);

bool rtos_k_byte_circular_buffer_read(
        _INOUT_ struct rtos_circular_buffer *circ_buf_p,
        _OUT_ uint8_t *entry_value_p,
        _IN_ bool wait_if_empty,
	_INOUT_ rtos_milliseconds_t *timeout_ms_p);

bool rtos_k_circular_buffer_is_empty(
	_IN_ struct rtos_circular_buffer *circ_buf_p);

void rtos_k_queue_init(
	_IN_  const char *queue_name_p,
	_IN_ bool use_mutex,
	_OUT_ struct rtos_queue *queue_p);

void
rtos_k_queue_add(
    _INOUT_ struct rtos_queue *queue_p,
    _INOUT_ struct glist_node *elem_p);

struct glist_node *
rtos_k_queue_remove(
    _INOUT_ struct rtos_queue *queue_p,
    _IN_ rtos_milliseconds_t timeout_ms);

void
rtos_k_capture_fdc_msg_vprintf(const char *fmt, va_list va);

#define ATOMIC_POST_INCREMENT_UINT32(_counter_p) \
        rtos_k_atomic_fetch_add_uint32(_counter_p, 1)

#define ATOMIC_POST_DECREMENT_UINT32(_counter_p) \
        rtos_k_atomic_fetch_sub_uint32(_counter_p, 1)

#define ATOMIC_POST_INCREMENT_UINT16(_counter_p) \
        rtos_k_atomic_fetch_add_uint16(_counter_p, 1)

#define ATOMIC_POST_DECREMENT_UINT16(_counter_p) \
        rtos_k_atomic_fetch_sub_uint16(_counter_p, 1)

#define ATOMIC_POST_INCREMENT_UINT8(_counter_p) \
        rtos_k_atomic_fetch_add_uint8(_counter_p, 1)

#define ATOMIC_POST_DECREMENT_UINT8(_counter_p) \
        rtos_k_atomic_fetch_sub_uint8(_counter_p, 1)

#define ATOMIC_POST_INCREMENT_POINTER(_pointer_p)                       \
        ((void *)rtos_k_atomic_fetch_add_uint32(                        \
                    (uint32_t *)&(_pointer_p), sizeof(*(_pointer_p))))

C_ASSERT(sizeof(void *) == sizeof(uint32_t));

#ifdef __cplusplus
}
#endif

#endif /* _McRTOS_KERNEL_SERVICES_H */
