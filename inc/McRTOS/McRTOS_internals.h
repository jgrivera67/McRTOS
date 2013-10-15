/**
 * @file McRTOS_internals.h
 *
 * McRTOS internal data structures and function prototypes
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 
#ifndef _McRTOS_INTERNALS_H
#define _McRTOS_INTERNALS_H

#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"

#if DEFINED_ARM_CORTEX_M_ARCH()
#   include "McRTOS_arm_cortex_m.h"
#endif

#include "generic_list.h"
#include "utils.h"
#include "compile_time_checks.h"

/**
 * Number of system threads per CPU
 */
#define RTOS_NUM_SYSTEM_THREADS_PER_CPU UINT8_C(2)

/**
 * Number of spokes of the per-cpu timer wheel. It must be a power of 2.
 */
//#define RTOS_TIMER_WHEEL_NUM_SPOKES UINT16_C(256)
#define RTOS_TIMER_WHEEL_NUM_SPOKES UINT16_C(8)

/**
 * Timer wheel spoke index range type
 */
typedef _RANGE_(0, RTOS_TIMER_WHEEL_NUM_SPOKES - 1)
        uint16_t rtos_timer_wheel_spoke_index_t;

/**
 * Range type for number of nested interrupts
 */
typedef _RANGE_(0, SOC_NUM_INTERRUPT_PRIORITIES)
        uint8_t rtos_nested_interrupts_count_t;

/**
 * Covert a thread priority to the corresponding bit mask in a thread priority
 * bit map
 */
#define RTOS_THREAD_PRIO_BIT_MASK(_thread_prio) \
        BIT((RTOS_NUM_THREAD_PRIORITIES - 1) - (_thread_prio))

/**
 * Reasons a McRTOS execution context can get switched out
 */
enum rtos_execution_context_switched_out_reasons
{
    CTX_SWITCHED_OUT_NEVER                  =       0x0,
    CTX_SWITCHED_OUT_PREEMPTED_BY_INTERRUPT =       0x1,

    /*
     * The following value reasons only apply to threads
     */
    CTX_SWITCHED_OUT_THREAD_PREEMPTED_BY_THREAD =   0x2,
    CTX_SWITCHED_OUT_THREAD_BLOCKED_ON_MUTEX =      0x3,
    CTX_SWITCHED_OUT_THREAD_BLOCKED_ON_CONDVAR =    0x4,
    CTX_SWITCHED_OUT_THREAD_YIELD =                 0x5,
    CTX_SWITCHED_OUT_THREAD_TIME_SLICE_EXHAUSTED =  0x6,
    CTX_SWITCHED_OUT_THREAD_TERMINATED =            0x7,

    /*
     * The following value reasons only apply to interrupts
     */
    CTX_SWITCHED_OUT_INTERRUPT_SERVICED =           0x8,

    /*
     * Last dummy value. New reasons must be added above here:
     */
    CTX_SWITCHED_OUT_LAST_INVALID_VALUE
};

/*
 * Greatest valid reason value cannot be larger than 0xf, as all valid reasons
 * must fit in 4 bits
 */
C_ASSERT(CTX_SWITCHED_OUT_LAST_INVALID_VALUE <= 0x10);


#define RTOS_STACK_OVERFLOW_BUFFER_SIGNATURE    UINT32_C(0xAABBAABB)
#define RTOS_STACK_OVERFLOW_MARKER              UINT32_C(0xFACEFFFF)
#define RTOS_STACK_UNDERFLOW_MARKER             UINT32_C(0xFACEBBBB)
#define RTOS_STACK_UNUSED_SIGNATURE             UINT32_C(0xFACECCCC)

/**
 * Execution stack area for a McRTOS thread
 */
struct rtos_thread_execution_stack
{
    /**
     * Stack overflow buffer, to be initialized to RTOS_STACK_OVERFLOW_BUFFER_SIGNATURE
     */
    rtos_execution_stack_entry_t tes_stack_overflow_buffer
        [RTOS_THREAD_STACK_OVERFLOW_BUFFER_SIZE_IN_ENTRIES];

    /**
     * Stack overflow sentinel, to be initialized to RTOS_STACK_OVERFLOW_MARKER
     */
    rtos_execution_stack_entry_t tes_stack_overflow_marker;

    /**
     * Actual stack
     */
    rtos_execution_stack_entry_t tes_stack[RTOS_THREAD_STACK_NUM_ENTRIES];

    /**
     * Stack underflow sentinel, to be initialized to RTOS_STACK_UNDERFLOW_MARKER
     */
    rtos_execution_stack_entry_t tes_stack_underflow_marker; 
}  __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_thread_execution_stack) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);

/**
 * McRTOS interrupt object
 */
struct rtos_interrupt
{
#   define      RTOS_INTERRUPT_SIGNATURE  GEN_SIGNATURE('I', 'N', 'T', 'R')
    uint32_t    int_signature;

    /**
     * Pointer to the CPU controller for the CPU core associated with this interrupt
     * object. 
     *
     * NOTE: The RTOS_ENTER_ISR() assembly language macro uses this field to retrieve
     * the pointer to the current execution context.
     */
    struct rtos_cpu_controller *int_cpu_controller_p;

    /**
     * Argument to be passed to the interrupt's clear_interrupt_xxx() and
     * interrupt_handler_xxx() functions from the interrupt's ISR function.
     *
     * NOTE: The RTOS_ENTER_ISR() assembly language macro uses this field.
     */
    void *int_arg_p;

    /**
     * Execution context for the Interrupt handler
     */
    struct rtos_execution_context int_execution_context;

    /**
     * Pointer to the interrupt service routine (ISR)
     */
    isr_function_t *int_isr_function_p;

    /**
     * Interrupt channel in the interrupt controller
     */
    interrupt_channel_t int_channel;

    /**
     * Interrupt priority at the Interrupt controller
     */
    interrupt_prio_t int_priority;

    /**
     * CPU core on which this interrupt should be raised
     */
    cpu_id_t int_cpu_id;

#if DEFINED_ARM_CLASSIC_ARCH()
    /**
     * Execution stack
     */

    /**
     * Stack overflow buffer, to be initialized to RTOS_STACK_OVERFLOW_BUFFER_SIGNATURE
     */
    rtos_execution_stack_entry_t int_stack_overflow_buffer
        [RTOS_INTERRUPT_STACK_OVERFLOW_BUFFER_SIZE_IN_ENTRIES];

    /**
     * Stack overflow sentinel, to be initialized to RTOS_STACK_OVERFLOW_MARKER
     */
    rtos_execution_stack_entry_t int_stack_overflow_marker;

    /**
     * Actual stack
     */
    rtos_execution_stack_entry_t int_stack[RTOS_INTERRUPT_STACK_NUM_ENTRIES];

    /**
     * Stack underflow sentinel, to be initialized to RTOS_STACK_UNDERFLOW_MARKER
     */
    rtos_execution_stack_entry_t int_stack_underflow_marker; 
#   endif

} __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_interrupt) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);

C_ASSERT(
    offsetof(struct rtos_interrupt, int_cpu_controller_p) == RTOS_INT_CPU_CONTROLLER_P_OFFSET);

/**
 * McRTOS Per-CPU Execution Controller 
 */
struct rtos_cpu_controller
{
#   define      RTOS_CPU_CONTROLLER_SIGNATURE  GEN_SIGNATURE('C', 'P', 'U', 'C')
    uint32_t    cpc_signature;
  
    /**
     * Pointer to the control block of the current execution context.
     *
     * NOTE: The RTOS_ENTER_ISR() assembly language macro uses this field.
     */
    struct rtos_execution_context *cpc_current_execution_context_p;

    /**
     * CPU ID of a given CPU
     */
    cpu_id_t    cpc_cpu_id;

    /**
     * Counter indicating if measurement of interrupts disabled time is
     * currently in progressing.
     */
    uint8_t cpc_interrupts_disabled_being_measured_count;

    /*
     * Boolean flag indicating that rtos_startup() has been completed for this
     * CPU controller.
     */
    bool cpc_startup_completed;

    /**
     * Boolean flag indicating that there is a pending time slice decrement
     * for the current thread. This flag is set by the rtos_tick_timer_handler()
     * and cleared by rtos_thread_scheduler() when invoked from rtos_k_exit_interrupt().
     */
    volatile bool cpc_pending_thread_time_slice_decrement;

    /**
     * Index of the current timer wheel spoke (hash table bucket). This is the
     * index of the hash table bucket that was processed on the last timer tick.
     */
    rtos_timer_wheel_spoke_index_t cpc_current_timer_wheel_spoke_index;

    /**
     * Number of ticks since boot for this CPU
     */
    rtos_ticks_t cpc_ticks_since_boot_count;

    /**
     * Pointer to the thread currently running or currently preempted by an
     * interrupt.
     */
    struct rtos_thread *cpc_current_thread_p;

    /**
     * Runnable thread priorities bit map
     */
    rtos_thread_prio_bitmap_t cpc_runnable_thread_priorities;

    /**
     * Active internal interrupts bit map
     */
    rtos_per_cpu_interrupts_bitmap_t cpc_active_internal_interrupts;

    /**
     * Active external interrupts bit map
     */
    rtos_per_cpu_interrupts_bitmap_t cpc_active_external_interrupts;

    /**
     * Current number of nested interrupts
     */
    rtos_nested_interrupts_count_t  cpc_nested_interrupts_count;

    /**
     * Total number of calls to this CPU's thread scheduler 
     */
    uint32_t cpc_thread_scheduler_calls;

    /**
     * Accumulated overhead of the thread scheduler in CPU clock cycles
     */
    cpu_clock_cycles_t cpc_accumulated_thread_scheduler_overhead;

    /**
     * Start time stamp for the last time that interrupts were disabled
     * on this CPU core.
     */
    cpu_clock_cycles_t cpc_interrupts_disabled_start_time_stamp;

    /**
     * Longest time interrupts have been disabled
     */
    cpu_clock_cycles_t cpc_longest_time_interrupts_disabled;

    /**
     * Latest measurement of time interrupts have been disabled
     */
    cpu_clock_cycles_t cpc_latest_measurement_time_interrupts_disabled;

    /**
     * Pointer to the per-CPU application startup configuration
     */
    const struct rtos_per_cpu_startup_app_configuration *cpc_app_config_p;

#ifdef _RELIABILITY_CHECKS_
    /**
     * Failures captured for this CPU core
     */
    struct fdc_info cpc_failures_info;
#endif

    /**
     * Anchor node node of the list of execution contexts that exist in the systems
     */
    struct glist_node cpc_execution_contexts_list_anchor;

    /**
     * Anchor node of the preemption chain, in which all currently preempted 
     * execution contexts are chained together. The preemption chain behaves
     * like a stack. Nodes are inserted and removed only at the head of the 
     * chain. The preemption chain represents the "Preempted by" relationship
     * between execution contexts.
     */
    struct glist_node cpc_preemption_chain_anchor;

    /**
     * McRTOS executon context for the reset handler
     */
    struct rtos_execution_context cpc_reset_execution_context;

    /**
     * Array of system thread objects for this CPU core
     */
    struct rtos_thread cpc_system_threads[RTOS_NUM_SYSTEM_THREADS_PER_CPU];

    /**
     * Anchor nodes of the runnable thread queues, one queue per thread priority
     */
    struct glist_node cpc_runnable_thread_queues_anchors[RTOS_NUM_THREAD_PRIORITIES];

    /**
     * Anchor nodes of the hash chains of timers for the timer wheel hash table
     */
    struct glist_node cpc_timer_wheel_hash_chains_anchors[RTOS_TIMER_WHEEL_NUM_SPOKES];

    /**
     * Array of execution stacks for system threads
     */
    struct rtos_thread_execution_stack cpc_system_threads_execution_stacks
                                            [RTOS_NUM_SYSTEM_THREADS_PER_CPU];

} __attribute__ ((aligned(SOC_CACHE_LINE_SIZE_IN_BYTES)));

C_ASSERT(sizeof(struct rtos_cpu_controller) <= RTOS_MAX_McRTOS_CPU_CONTROLLER_SIZE);

C_ASSERT(sizeof(struct rtos_cpu_controller) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);
C_ASSERT(
    offsetof(struct rtos_cpu_controller, cpc_current_execution_context_p) ==
    RTOS_CPC_CURRENT_EXECUTION_CONTEXT_P_OFFSET);

/**
 * Indexes of per-cpu system thread objects in the cpc_system_threads array.
 */
enum rtos_system_thread_indexes
{
    RTOS_ROOT_SYSTEM_THREAD = 0,
    RTOS_IDLE_SYSTEM_THREAD,

    /*
     * New enum entries must be added above this entry
     */
    RTOS_NUM_SYSTEM_THREAD_INDEXES
};

C_ASSERT(RTOS_NUM_SYSTEM_THREAD_INDEXES <= RTOS_NUM_SYSTEM_THREADS_PER_CPU);

/**
 * McRTOS global state variables
 */
struct McRTOS
{
#   define      MCRTOS_SIGNATURE  GEN_SIGNATURE('R', 'T', 'O', 'S')
    uint32_t    rts_signature;

    /**
     * Per-CPU execution controllers, one for each CPU core
     */
    struct rtos_cpu_controller rts_cpu_controllers[SOC_NUM_CPU_CORES];

    /**
     * Cause of last SoC reset
     */
    cpu_reset_cause_t rts_soc_reset_cause;

    /**
     * Pointer to the next free interrupt object
     */
    struct rtos_interrupt *rts_next_free_interrupt_p;

    /**
     * Pointer to the next free application thread object
     */
    struct rtos_thread *rts_next_free_app_thread_p;

    /**
     * Pointer to the next free application timer object
     */
    struct rtos_timer *rts_next_free_app_timer_p;

    /**
     * Pointer to the next free application mutex object
     */
    struct rtos_mutex *rts_next_free_app_mutex_p;

    /**
     * Pointer to the next free application condvar object
     */
    struct rtos_condvar *rts_next_free_app_condvar_p;

    /**
     * Pointer to the next free application message channel object
     */
    struct rtos_32bit_msg_channel *rts_next_free_app_32bit_msg_channel_p;

    /**
     * Pointer to the next free application object pool object
     */
    struct rtos_object_pool *rts_next_free_app_object_pool_p;

    /**
     * Pointer to the next free execution stack for application threads
     */
    struct rtos_thread_execution_stack *rts_next_free_app_thread_stack_p;

    /**
     * Pointer to the array of execution stacks for application threads
     * (Execution stacks for application threads can be in DRAM or SRAM,
     *  depending on how much memory they use)
     */
    struct rtos_thread_execution_stack *rts_app_threads_execution_stacks_p;

#   ifdef _CPU_CYCLES_MEASURE_
    /**
     * Approximate overhead for taking a measurement of time in CPU clock
     * cycles, using the BEGIN_CPU_CYCLES_MEASURE()/END_CPU_CYCLES_MEASURE()
     * macros.
     */
    cpu_clock_cycles_t rts_cpu_cycles_measure_overhead;
#   endif

#   ifdef LCD_SUPPORTED
    /**
     * Currently active LCD channel used by rtos_lcd_putchar().
     *
     * LCD Channel Rules:
     * - Each LCD channel has its own frame buffer. The output displayed
     *   on the LCD corresponds to the frame buffer of the current LCD
     *   channel.
     * - Threads running on different CPU cores are not allowed to use the same
     *   LCD channel.
     * - If a thread sends LCD output and its LCD channel is not the
     *   currently active LCD channel, the output is not physically displayed in
     *   the LCD, but it is saved in the corresponding frame buffer.
     *   If more than one thread sends output to the same LCD channel, output
     *   interference may happen among the threads, unless they use a mutex to 
     *   serialize their LCD outputs, or they write to disjoint areas of the
     *   frame buffer.
     * - Input receive on the current LCD channel is passed only to the thread
     *   whose LCD channel matches the current LCD channel. There cannot be
     *   more than one thread reading from the same LCD channel.
     *
     * The current LCD channel is meant to be changeable using a dedicated
     * input device, for example up/down buttons in the target embedded board.
     */
    rtos_lcd_channels_t   rts_current_lcd_channel;
#   endif

    /**
     * Flag indicating that the idle thread is to stop the CPU in the idle thread
     * and wait for interrupts, instead of busy-waiting.
     */
    volatile bool rts_stop_idle_cpu;

    /**
     * Array of interrupt objects, one for each interrupt registered with McRTOS.
     */
    struct rtos_interrupt rts_interrupts[RTOS_MAX_NUM_INTERRUPTS];

    /**
     * Array of application thread objects (thread control blocks) that can
     * exist in the system
     */
    struct rtos_thread rts_app_threads[RTOS_MAX_NUM_APP_THREADS];
   
#ifndef RTOS_USE_DRAM_FOR_APP_THREAD_STACKS
    /**
     * Array of execution stacks for application threads
     */
    struct rtos_thread_execution_stack rts_app_threads_execution_stacks
                                            [RTOS_MAX_NUM_APP_THREADS];
#endif

    /**
     * Array of McRTOS timer objects that can exist in the system
     */
    struct rtos_timer rts_app_timers[RTOS_MAX_NUM_APP_TIMERS];

    /**
     * Array of McRTOS mutexes that can exist in the system
     */
    struct rtos_mutex rts_app_mutexes[RTOS_MAX_NUM_APP_MUTEXES];

    /**
     * Array of McRTOS condition variables that can exist in the system
     */
    struct rtos_condvar rts_app_condvars[RTOS_MAX_NUM_APP_CONDVARS];

#if RTOS_MAX_NUM_APP_MSG_CHANNELS > 0
    /**
     * Array of McRTOS 32-bit message channels that can exist in the system
     */
    struct rtos_32bit_msg_channel rts_app_32bit_msg_channels[RTOS_MAX_NUM_APP_MSG_CHANNELS];
#endif

#if RTOS_MAX_NUM_APP_OBJECT_POOLS > 0
    /**
     * Array of McRTOS object pools that can exist in the system
     */
    struct rtos_object_pool rts_app_object_pools[RTOS_MAX_NUM_APP_OBJECT_POOLS];
#endif

    /**
     * Command line buffer used by the McRTOS console and the McRTOS debugger
     */
    char rts_command_line_buffer[RTOS_COMMAND_LINE_BUFFER_SIZE];
};

C_ASSERT(
    sizeof(struct McRTOS) - offsetof(struct McRTOS, rts_app_threads) <=
    RTOS_MAX_McRTOS_DATA_SIZE);

C_ASSERT(
    offsetof(struct McRTOS, rts_cpu_controllers) == RTOS_RTS_CPU_CONTROLLERS_OFFSET);

extern struct McRTOS *const g_McRTOS_p;


/**
 * Get the current execution context for the calling CPU
 */
#define RTOS_GET_CURRENT_EXECUTION_CONTEXT() \
        (g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()].     \
            cpc_current_execution_context_p)

#define RTOS_EXECUTION_CONTEXT_GET_THREAD(_execution_context_p)         \
            ENCLOSING_STRUCT(                                           \
                _execution_context_p, struct rtos_thread,               \
                thr_execution_context)

#define RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(_execution_context_p)  \
            ENCLOSING_STRUCT(                                           \
                _execution_context_p, struct rtos_interrupt,            \
                int_execution_context)

#define RTOS_EXECUTION_CONTEXT_GET_STACK_POINTER(_execution_context_p)  \
            ((_execution_context_p)->ctx_cpu_registers[CPU_REG_SP])

#ifdef _CPU_CYCLES_MEASURE_
#   define RTOS_EXECUTION_CONTEXT_UPDATE_CPU_USAGE( \
            _execution_context_p, _used_cpu_cycles)                         \
        do {                                                                \
            DBG_ASSERT(                                                     \
                (int32_t)(_used_cpu_cycles) > 0,                            \
                _used_cpu_cycles, _execution_context_p);                    \
            (_execution_context_p)->                                        \
                ctx_accumulated_cpu_usage_cycles += (_used_cpu_cycles);     \
            if ((_execution_context_p)->                                    \
                    ctx_accumulated_cpu_usage_cycles >=                     \
                SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ * 1000) {                     \
                uint32_t whole_milliseconds =                               \
                    CPU_CLOCK_CYCLES_TO_MILLISECONDS(                       \
                        (_execution_context_p)->                            \
                            ctx_accumulated_cpu_usage_cycles);              \
                (_execution_context_p)->                                    \
                    ctx_accumulated_cpu_usage_milliseconds +=               \
                    whole_milliseconds;                                     \
                (_execution_context_p)->                                    \
                    ctx_accumulated_cpu_usage_cycles -=                     \
                        whole_milliseconds *                                \
                        (SOC_CPU_CLOCK_FREQ_IN_MEGA_HZ * 1000);             \
            }                                                               \
        } while (0)

#else
#   define RTOS_EXECUTION_CONTEXT_UPDATE_CPU_USAGE( \
            _execution_context_p, _used_cpu_cycles)
#endif

#define RTOS_THREAD_QUEUE_NODE_GET_THREAD(_glist_node_p) \
        GLIST_NODE_ENTRY(_glist_node_p, struct rtos_thread, thr_list_node)

#define RTOS_PREEMPTION_CHAIN_NODE_GET_EXECUTION_CONTEXT(_glist_node_p) \
        GLIST_NODE_ENTRY(_glist_node_p, struct rtos_execution_context,  \
                         ctx_preemption_chain_node)

#define RTOS_TIMER_HASH_CHAIN_NODE_GET_TIMER(_glist_node_p) \
        GLIST_NODE_ENTRY(_glist_node_p, struct rtos_timer, tmr_list_node)

#define RTOS_THREAD_CHANGE_STATE(_thread_p, _new_state) \
        do {                                                                \
            /* if ((_thread_p)->thr_state != (_new_state)) */                    \
            FDC_ASSERT((_thread_p)->thr_state != (_new_state), (_thread_p)->thr_state, _new_state);                     \
            {                                                               \
                (_thread_p)->thr_state_history <<= 4;                       \
                (_thread_p)->thr_state_history |= (_thread_p)->thr_state;   \
                (_thread_p)->thr_state = (_new_state);                      \
            }                                                               \
        } while (0)

#define RTOS_THREAD_CHANGE_PRIORITY(_thread_p, _new_priority) \
        do {                                                                \
            (_thread_p)->thr_priority_history <<= 8;                        \
            (_thread_p)->thr_priority_history |=                            \
                (_thread_p)->thr_current_priority;                          \
            (_thread_p)->thr_current_priority = (_new_priority);            \
        } while (0)

#define RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON( \
            _execution_context_p, _switched_out_reason, _cpu_controller_p)  \
        do {                                                                \
            (_execution_context_p)->ctx_switched_out_reason_history <<= 4;  \
            (_execution_context_p)->ctx_switched_out_reason_history |=      \
                (_execution_context_p)->ctx_last_switched_out_reason;       \
            (_execution_context_p)->ctx_last_switched_out_reason =          \
                (_switched_out_reason);                                     \
            (_execution_context_p)->                                        \
                ctx_last_switched_out_time_stamp_in_ticks =                 \
                (_cpu_controller_p)->cpc_ticks_since_boot_count;            \
            (_execution_context_p)->ctx_switched_out_counter ++;            \
        } while (0)

#define RTOS_CURRENT_THREAD_IS_ROOT_SYSTEM_THREAD(_cpu_controller_p) \
        ((_cpu_controller_p)->cpc_current_thread_p ==                       \
         &(_cpu_controller_p)->cpc_system_threads[RTOS_ROOT_SYSTEM_THREAD])

/**
 * Add at a thread to a runnable queue, invoking add_elem_function_p()
 */
static inline void
rtos_add_runnable_thread(
    struct rtos_cpu_controller *cpu_controller_p,
    struct rtos_thread *thread_p,
    glist_add_elem_function_t *add_elem_function_p)
{
    DBG_ASSERT_RTOS_THREAD_INVARIANTS(thread_p);

    struct glist_node *runnable_queue_anchor_p = 
        &cpu_controller_p->cpc_runnable_thread_queues_anchors
                                        [thread_p->thr_current_priority];

    DBG_ASSERT(
        add_elem_function_p == glist_add_tail_elem ||
        add_elem_function_p == glist_add_head_elem,
        add_elem_function_p, 0);

    add_elem_function_p(
        runnable_queue_anchor_p, &thread_p->thr_list_node);

    cpu_controller_p->cpc_runnable_thread_priorities |=
        RTOS_THREAD_PRIO_BIT_MASK(thread_p->thr_current_priority);

    RTOS_THREAD_CHANGE_STATE(thread_p, RTOS_THREAD_RUNNABLE);
}


/**
 * Add a thread at the end of a runnable queue
 */
static inline void
rtos_add_tail_runnable_thread(
    struct rtos_cpu_controller *cpu_controller_p,
    struct rtos_thread *thread_p)
{
    rtos_add_runnable_thread(
        cpu_controller_p,
        thread_p,
        glist_add_tail_elem);
}


/**
 * Add a thread at the beginning of a runnable queue
 */
static inline void
rtos_add_head_runnable_thread(
    struct rtos_cpu_controller *cpu_controller_p,
    struct rtos_thread *thread_p)
{
    rtos_add_runnable_thread(
        cpu_controller_p,
        thread_p,
        glist_add_head_elem);
}


/**
 * Remove a thread from the runnable queue it is currently in.
 *
 * NOTE: Depending on when this macro is invoked, thr_current_priority may
 * not match the runnable queue the thread is in.
 */
static inline void
rtos_remove_runnable_thread(
    struct rtos_cpu_controller *cpu_controller_p,
    struct rtos_thread *thread_p,
    rtos_thread_state_t new_thread_state)
{
    struct glist_node *runnable_queue_anchor_p =           
        GLIST_NODE_GET_LIST(&thread_p->thr_list_node);    
                                                            
    FDC_ASSERT(
        thread_p->thr_state == RTOS_THREAD_RUNNABLE,
        thread_p->thr_state, thread_p);

    rtos_thread_prio_t thread_prio =                          
        runnable_queue_anchor_p -                              
        cpu_controller_p->cpc_runnable_thread_queues_anchors;
                                                     
    FDC_ASSERT(                                       
        thread_prio < RTOS_NUM_THREAD_PRIORITIES,      
        runnable_queue_anchor_p, cpu_controller_p);    
                                                         
    glist_remove_elem(&thread_p->thr_list_node);       
                                                           
    if (GLIST_IS_EMPTY(runnable_queue_anchor_p))            
    {                                                        
        cpu_controller_p->cpc_runnable_thread_priorities &= 
            ~RTOS_THREAD_PRIO_BIT_MASK(thread_prio);
    }

    RTOS_THREAD_CHANGE_STATE(thread_p, new_thread_state); 
}


/**
 * Calculate difference between two CPU clock cycle values
 */
#define CPU_CLOCK_CYCLES_DELTA(_begin_cycles, _end_cycles) \
        ((cpu_clock_cycles_t)((int32_t)(_end_cycles) - \
                              (int32_t)(_begin_cycles)))

#ifdef _CPU_CYCLES_MEASURE_

/**
 * Declare local variables used by BEGIN_CPU_CYCLES_MEASURE() and 
 * END_CPU_CYCLES_MEASURE()
 */
#define DECLARE_CPU_CYCLES_MEASURE_VARS() \
        cpu_clock_cycles_t begin_cycles_; \
        cpu_status_register_t saved_cpu_status_register_

/**
 * Begin taking a time measure in CPU clock cycles 
 */
#define BEGIN_CPU_CYCLES_MEASURE() \
        do {                                                                \
            saved_cpu_status_register_ = rtos_k_disable_cpu_interrupts();   \
            begin_cycles_ = get_cpu_clock_cycles();                         \
        } while (0)

/**
 * End taking a time measure in CPU clock cycles 
 */
#define END_CPU_CYCLES_MEASURE(_measured_cycles_var_) \
        do {                                                                \
            (_measured_cycles_var_) =                                       \
            CPU_CLOCK_CYCLES_DELTA(begin_cycles_, get_cpu_clock_cycles()) - \
                g_McRTOS_p->rts_cpu_cycles_measure_overhead;                \
            rtos_k_restore_cpu_interrupts(saved_cpu_status_register_);      \
        } while (0)

#ifdef _MEASURE_INTERRUPTS_DISABLED_TIME_
/**
 * Macro to be invoked at the beginning of a disabled interrupts measurement,
 * when interrupts have already been disabled without a prior call to
 * rtos_k_disable_cpu_interrupts(), for example when entering an ISR.
 */
#define RTOS_START_INTERRUPTS_DISABLED_TIME_MEASURE() \
        (void)rtos_start_interrupts_disabled_time_measure(0)

/**
 * Macro to be invoked at the end of a disabled interrupts measurement,
 * when interrupts have already been disabled without a prior call to
 * rtos_k_disable_cpu_interrupts(), for example when entering an ISR.
 */
#define RTOS_STOP_INTERRUPTS_DISABLED_TIME_MEASURE() \
        (void)rtos_stop_interrupts_disabled_time_measure(0)

#else
#define RTOS_START_INTERRUPTS_DISABLED_TIME_MEASURE()
#define RTOS_STOP_INTERRUPTS_DISABLED_TIME_MEASURE() 

#endif /* _MEASURE_INTERRUPTS_DISABLED_TIME_ */

#else

#define DECLARE_CPU_CYCLES_MEASURE_VARS()

#define BEGIN_CPU_CYCLES_MEASURE() 

#define END_CPU_CYCLES_MEASURE(_measured_cycles_var_) \
        (_measured_cycles_var_) = 0      

#define RTOS_START_INTERRUPTS_DISABLED_TIME_MEASURE()

#define RTOS_STOP_INTERRUPTS_DISABLED_TIME_MEASURE() 

#endif /* _CPU_CYCLES_MEASURE_ */


_NEVER_RETURN_
void rtos_thread_scheduler(rtos_context_switch_type_t ctx_switch_type);

extern rtos_timer_function_t rtos_delay_timer_callback;

cpu_status_register_t rtos_start_interrupts_disabled_time_measure(
    _IN_ cpu_status_register_t saved_cpsr);

cpu_status_register_t rtos_stop_interrupts_disabled_time_measure(
    _IN_ cpu_status_register_t saved_cpsr);

void rtos_execution_context_init(
    _OUT_ struct rtos_execution_context *execution_context_p,
    _IN_  const char *context_name_p,
    _IN_  cpu_id_t cpu_id,
    _IN_  rtos_execution_context_type_t context_type,
    _IN_  fdc_context_switch_trace_entry_t prefilled_trace_entry,
    _IN_  rtos_cpu_mode_t rtos_cpu_mode,
    _IN_  cpu_register_t cpu_pc_register,
    _IN_  cpu_register_t cpu_r0_register,
    _IN_  rtos_execution_stack_entry_t *stack_top_end_p,
    _IN_  rtos_execution_stack_entry_t *stack_bottom_end_p);

void rtos_hard_fault_exception_handler(
        _IN_ const struct rtos_execution_context *current_execution_context_p);

void rtos_run_debugger(
        _IN_ const struct rtos_execution_context *current_execution_context_p,
        _IN_ const uint32_t *stack_p);

/**
 * Add an execution context at the top of the preemption chain
 */
static inline void
rtos_preemption_chain_push_context(
    _INOUT_ struct rtos_cpu_controller *cpu_controller_p,
    _INOUT_ struct rtos_execution_context *preempted_context_p)
{
    struct glist_node *preemption_chain_anchor_p = 
        &cpu_controller_p->cpc_preemption_chain_anchor;

#ifdef _RELIABILITY_CHECKS_
    if (GLIST_IS_NOT_EMPTY(preemption_chain_anchor_p))
    {
        struct glist_node *top_preemption_node_p = 
            GLIST_GET_FIRST(preemption_chain_anchor_p);

        struct rtos_execution_context *last_preempted_context_p =
            RTOS_PREEMPTION_CHAIN_NODE_GET_EXECUTION_CONTEXT(top_preemption_node_p);

        if (last_preempted_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT)
        {
            /*
             * If the last context preempted is an interrupt, the context being
             * preempted has to be another interrupt. It cannot a thread, because
             * a thread cannot preempt an interrupt
             */
            FDC_ASSERT(
                preempted_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT,
                preempted_context_p->ctx_context_type, preempted_context_p);

           struct rtos_interrupt *last_preempted_interrupt_p =
               RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(last_preempted_context_p);

           struct rtos_interrupt *preempted_interrupt_p =
               RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(preempted_context_p);

           FDC_ASSERT(
                preempted_interrupt_p->int_priority <
                last_preempted_interrupt_p->int_priority,
                preempted_interrupt_p,
                last_preempted_interrupt_p);
        }
        else
        {
            /*
             * The last context preempted is a thread. So, the context being
             * preempted can be either another thread or an interrupt.
             *
             * NOTE: We cannot assert anything about the current priorities
             * of the two threads, as they may been boosted due to mutex
             * priority inheritance, after they were preempted.
             */
            DBG_ASSERT(
                last_preempted_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
                last_preempted_context_p->ctx_context_type,
                last_preempted_context_p);
        }
    }
#endif

    glist_add_head_elem(
        preemption_chain_anchor_p,
        &preempted_context_p->ctx_preemption_chain_node);
}


/**
 * Remove the execution context from the top of the preemption chain
 */
static inline struct rtos_execution_context *
rtos_preemption_chain_pop_context(
    _INOUT_ struct rtos_cpu_controller *cpu_controller_p)
{
    struct glist_node *preemption_chain_anchor_p = 
        &cpu_controller_p->cpc_preemption_chain_anchor;

    FDC_ASSERT(
        GLIST_IS_NOT_EMPTY(preemption_chain_anchor_p),
        preemption_chain_anchor_p, cpu_controller_p);

    struct glist_node *top_preemption_node_p = 
        GLIST_GET_FIRST(preemption_chain_anchor_p);

    glist_remove_elem(top_preemption_node_p);

    struct rtos_execution_context *last_preempted_context_p =
        RTOS_PREEMPTION_CHAIN_NODE_GET_EXECUTION_CONTEXT(top_preemption_node_p);

    return last_preempted_context_p;
}


/**
 * Remove the given execution context from the preemption chain.
 */
static inline void
rtos_preemption_chain_remove_context(
    _INOUT_ struct rtos_cpu_controller *cpu_controller_p,
    _INOUT_ struct rtos_execution_context *context_p)
{
    struct glist_node *preemption_chain_anchor_p = 
        &cpu_controller_p->cpc_preemption_chain_anchor;

    FDC_ASSERT(
        GLIST_IS_NOT_EMPTY(preemption_chain_anchor_p),
        preemption_chain_anchor_p, cpu_controller_p);

    FDC_ASSERT(
        GLIST_NODE_GET_LIST(&context_p->ctx_preemption_chain_node) ==
        preemption_chain_anchor_p,
        context_p, preemption_chain_anchor_p);

    glist_remove_elem(&context_p->ctx_preemption_chain_node);
}

#endif /* _McRTOS_INTERNALS_H */
