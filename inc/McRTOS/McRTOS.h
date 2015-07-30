/**
 * @file McRTOS.h
 *
 * McRTOS public interfaces
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */
#ifndef _McRTOS_H
#define _McRTOS_H

#include <McRTOS/McRTOS_config_parameters.h>
#include <BoardSupport/hardware_abstractions.h>
#include <McRTOS/failure_data_capture.h>
#include <McRTOS/utils.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Opaque types
 */

struct rtos_thread;
struct rtos_interrupt_handler;
struct rtos_timer;
struct rtos_mutex;
struct rtos_condvar;
struct rtos_msg_channel;
struct rtos_object_pool;
struct rtos_circular_buffer;
struct rtos_queue;
struct glist_node;

#define RTOS_HIGHEST_THREAD_PRIORITY    UINT8_C(0)
#define RTOS_LOWEST_THREAD_PRIORITY     (RTOS_NUM_THREAD_PRIORITIES - 1)

#ifdef LCD_SUPPORTED
/**
 * LCD channels
 */
enum rtos_lcd_channels
{
    RTOS_COMMAND_LINE_LCD_CHANNEL = 0,
    RTOS_APP_LCD_CHANNEL,
    RTOS_TILES_LCD_CHANNEL,

    /*
     * Add new entries above here
     */
    RTOS_LCD_CHANNEL_NONE
};

C_ASSERT(RTOS_LCD_CHANNEL_NONE == RTOS_NUM_LCD_CHANNELS);
#endif /* LCD_SUPPORTED */

/**
 * Thread priority type (0 is the highest priority)
 */
typedef _RANGE_(RTOS_HIGHEST_THREAD_PRIORITY, RTOS_LOWEST_THREAD_PRIORITY)
        uint8_t rtos_thread_prio_t;

/**
 * Number of application threads range type
 */
typedef _RANGE_(0, RTOS_MAX_NUM_APP_THREADS)
        uint8_t rtos_num_app_threads_t;

/**
 * Number of application mutexes range type
 */
typedef _RANGE_(0, RTOS_MAX_NUM_APP_MUTEXES)
        uint8_t rtos_num_app_mutexes_t;

/**
 * Number of application condvars range type
 */
typedef _RANGE_(0, RTOS_MAX_NUM_APP_CONDVARS)
        uint8_t rtos_num_app_condvars_t;

/**
 * Number of application timers range type
 */
typedef _RANGE_(0, RTOS_MAX_NUM_APP_TIMERS)
        uint8_t rtos_num_app_timers_t;

/**
 * Number of application message channels range type
 */
typedef _RANGE_(0, RTOS_MAX_NUM_APP_MSG_CHANNELS)
        uint8_t rtos_num_app_msg_channels_t;

/**
 * Number of application object pools range type
 */
typedef _RANGE_(0, RTOS_MAX_NUM_APP_OBJECT_POOLS)
        uint8_t rtos_num_app_object_pools_t;

/**
 * Thread Priorities bit map type
 *
 * The most significant bit (bit 31) corresponds to the highest priority
 * (priority 0).
 * The least significant bit (bit 0) corresponds to the lowest priority
 * (priority RTOS_NUM_THREAD_PRIORITIES - 1).
 */
#if RTOS_NUM_THREAD_PRIORITIES == 8
typedef uint8_t rtos_thread_prio_bitmap_t;
#elif RTOS_NUM_THREAD_PRIORITIES == 16
typedef uint16_t rtos_thread_prio_bitmap_t;
#elif RTOS_NUM_THREAD_PRIORITIES == 32
typedef uint32_t rtos_thread_prio_bitmap_t;
#else
#error "Unsupported value for RTOS_NUM_THREAD_PRIORITIES"
#endif

C_ASSERT(
    sizeof(rtos_thread_prio_bitmap_t) * 8 == RTOS_NUM_THREAD_PRIORITIES);

#define RTOS_INTERRUPTS_BITMAP_NUM_WORDS \
	HOW_MANY(SOC_NUM_INTERRUPT_CHANNELS, sizeof(uint32_t) * 8)

/**
 * Per CPU interrupts bit map type
 */
typedef uint32_t rtos_per_cpu_interrupts_bitmap_t[RTOS_INTERRUPTS_BITMAP_NUM_WORDS];

C_ASSERT(
    sizeof(rtos_per_cpu_interrupts_bitmap_t) * 8 >= SOC_NUM_INTERRUPT_CHANNELS);

#define RTOS_INTR_BIT_MAP_ENTRY_INDEX(_intr_channel) \
	((_intr_channel) / (sizeof(uint32_t) * 8))

C_ASSERT(
    RTOS_INTR_BIT_MAP_ENTRY_INDEX(SOC_NUM_INTERRUPT_CHANNELS - 1) ==
    RTOS_INTERRUPTS_BITMAP_NUM_WORDS - 1);

#define RTOS_INTR_BIT_MAP_ENTRY_MASK(_intr_channel) \
	BIT((_intr_channel) % (sizeof(uint32_t) * 8))

#define RTOS_INTR_BIT_MAP_SET_BIT(_intr_bitmap_p, _intr_channel) \
	do {								    \
		(_intr_bitmap_p)[					    \
			RTOS_INTR_BIT_MAP_ENTRY_INDEX(_intr_channel)]	    \
			|= RTOS_INTR_BIT_MAP_ENTRY_MASK(_intr_channel);	    \
	} while (0)

#define RTOS_INTR_BIT_MAP_CLEAR_BIT(_intr_bitmap_p, _intr_channel) \
	do {								    \
		(_intr_bitmap_p)[					    \
			RTOS_INTR_BIT_MAP_ENTRY_INDEX(_intr_channel)]	    \
			&= ~RTOS_INTR_BIT_MAP_ENTRY_MASK(_intr_channel);    \
	} while (0)

#define RTOS_INTR_BIT_MAP_GET_BIT(_intr_bitmap_p, _intr_channel) \
		(((_intr_bitmap_p)[					    \
			RTOS_INTR_BIT_MAP_ENTRY_INDEX(_intr_channel)]	    \
			& RTOS_INTR_BIT_MAP_ENTRY_MASK(_intr_channel)) ?    \
			1 : 0)

#ifdef LCD_SUPPORTED
/**
 * Range type for LCD channels
 */
typedef _RANGE_(0, RTOS_NUM_LCD_CHANNELS)
        uint8_t rtos_lcd_channels_t;
#endif

/**
 * Thread function type
 */
typedef fdc_error_t rtos_thread_function_t(void *arg_p);

/**
 * Timer callback function type
 */
typedef void rtos_timer_function_t(struct rtos_timer *rtos_timer_p);

/**
 * Idle thread hook function type
 */
typedef void rtos_idle_thread_hook_function_t(void);

/**
 * Application-specific system call function type
 */
typedef fdc_error_t rtos_app_system_call_function_t(void *arg_p);

/**
 * Time in milliseconds type
 */
typedef uint32_t rtos_milliseconds_t;

/**
 * Number of timer ticks range type
 */
typedef uint32_t rtos_ticks_t;

/**
 *
 * command-line command processor function type
 */
typedef void cmd_function_t(void);

/**
 * Console command
 */
struct rtos_console_command
{
    const char *cmd_name_p;
    const char *cmd_description_p;
    cmd_function_t *cmd_function_p;
};

/**
 * Execution stack entry type
 */
typedef uint32_t rtos_execution_stack_entry_t;

C_ASSERT(
    sizeof(rtos_execution_stack_entry_t) == ARM_CPU_WORD_SIZE_IN_BYTES);

/**
 * Execution stack area for a McRTOS thread
 */
struct __rtos_thread_execution_stack
{
    /**
     * Stack overflow buffer, to be initialized to RTOS_STACK_OVERFLOW_BUFFER_SIGNATURE
     */
    rtos_execution_stack_entry_t tes_stack_overflow_buffer
        [RTOS_STACK_OVERFLOW_BUFFER_SIZE_IN_ENTRIES];
#   define RTOS_STACK_OVERFLOW_BUFFER_SIGNATURE    UINT32_C(0xAABBAABB)

    /**
     * Stack overflow sentinel, to be initialized to RTOS_STACK_OVERFLOW_MARKER
     */
    rtos_execution_stack_entry_t tes_stack_overflow_marker;
#   define RTOS_STACK_OVERFLOW_MARKER              UINT32_C(0xFACEFFFF)

    /**
     * Actual stack, to be initialized with RTOS_STACK_UNUSED_SIGNATURE
     */
    rtos_execution_stack_entry_t tes_stack[RTOS_THREAD_STACK_NUM_ENTRIES];
#   define RTOS_STACK_UNUSED_SIGNATURE             UINT32_C(0xFACECCCC)

    /**
     * Stack underflow sentinel, to be initialized to RTOS_STACK_UNDERFLOW_MARKER
     */
    rtos_execution_stack_entry_t tes_stack_underflow_marker;
#   define RTOS_STACK_UNDERFLOW_MARKER             UINT32_C(0xFACEBBBB)
};

struct rtos_thread_execution_stack {
    struct __rtos_thread_execution_stack;
} __attribute__ ((aligned(SOC_MPU_REGION_ALIGNMENT(struct __rtos_thread_execution_stack))));

C_ASSERT(sizeof(struct __rtos_thread_execution_stack) % SOC_CACHE_LINE_SIZE_IN_BYTES == 0);

C_ASSERT(sizeof(struct rtos_thread_execution_stack) %
         SOC_MPU_REGION_ALIGNMENT(struct __rtos_thread_execution_stack) == 0);

/**
 * Block of parameters for creating a thread
 */
struct rtos_thread_creation_params {
    /**
     * Pointer to thread name string for debugging purposes
     */
    const char *p_name_p;

    /**
     * Pointer to the thread function (top-level function executed by the
     * thread)
     */
    rtos_thread_function_t *p_function_p;

    /**
     * Argument to be passed in to the thread function
     */
    void *p_function_arg_p;

    /**
     * Thread priority
     */
    rtos_thread_prio_t p_priority;
};


/**
 * McRTOS startup application configuration structure
 */
struct rtos_startup_app_configuration
{
    /**
     * Pointer to function that initializes application-specific hardware
     */
    app_hardware_init_t *const stc_app_hardware_init_p;

    /**
     * Pointer to function that stops application-specific hardware
     */
    app_hardware_stop_t *const stc_app_hardware_stop_p;

    /**
     * Pointer to function that does application-specific initialization
     */
    app_software_init_t *const stc_app_software_init_p;

    /**
     * Number of application-specific console commands
     */
    const uint8_t stc_num_app_console_commands;

    /**
     * Pointer to array of application-specific console command
     */
    const struct rtos_console_command *const stc_app_console_commands_p;
};

/**
 * Attributes argument for rtos_lcd_putchar
 */
struct rtos_lcd_putchar_attributes
{
    lcd_x_t lcd_x;
    lcd_y_t lcd_y;
    const struct lcd_char_attributes *lcd_char_attributes_p;
};

/*
 * Application interface functions
 */

/*
 * Function to be invoked from the Application's main() function
 * to start McRTOS.
 */
_NEVER_RETURN_
void
rtos_startup(
    _IN_ const struct rtos_startup_app_configuration *rtos_app_config_p);

_NEVER_RETURN_
void
rtos_reboot(void);

/*
 * System calls to be invoked from Application threads, which run in
 * unprivileged mode:
 *
 * NOTE: These functions cannot have more than 4 parameters.
 */

_MAY_NOT_RETURN_
void
rtos_thread_init(
    _IN_ const struct rtos_thread_creation_params *params_p,
    _IN_ struct rtos_thread_execution_stack *thread_stack_p,
    _OUT_ struct rtos_thread *rtos_thread_p);

_THREAD_CALLERS_ONLY_
void
rtos_thread_delay(
    rtos_milliseconds_t num_milliseconds);

_THREAD_CALLERS_ONLY_
void
rtos_thread_abort(fdc_error_t fdc_error);

_THREAD_CALLERS_ONLY_
const struct rtos_thread *
rtos_thread_self(void);

const char *
rtos_thread_name(
    _IN_ const struct rtos_thread *rtos_thread_p);

_THREAD_CALLERS_ONLY_
void
rtos_thread_yield(void);

bool
rtos_caller_is_thread(void);

_THREAD_CALLERS_ONLY_
void
rtos_thread_condvar_wait(
    _IN_ struct rtos_mutex *rtos_mutex_p);

_THREAD_CALLERS_ONLY_
void
rtos_thread_condvar_wait_interrupt(void);

void
rtos_thread_condvar_signal(
    _IN_ struct rtos_thread *rtos_thread_p);

rtos_ticks_t
rtos_get_ticks(void);

void
rtos_mutex_init(
    _IN_  const char *mutex_name_p,
    _OUT_ struct rtos_mutex *rtos_mutex_p);

_THREAD_CALLERS_ONLY_
void
rtos_mutex_acquire(
    _IN_ struct rtos_mutex *rtos_mutex_p);

_THREAD_CALLERS_ONLY_
void
rtos_mutex_release(
    _IN_ struct rtos_mutex *rtos_mutex_p);

void
rtos_condvar_init(
    _IN_  const char *condvar_name_p,
    _OUT_ struct rtos_condvar *rtos_condvar_p);

_THREAD_CALLERS_ONLY_
void
rtos_condvar_wait(
    _IN_ struct rtos_condvar *rtos_condvar_p,
    _IN_  struct rtos_mutex *rtos_mutex_p,
    _INOUT_ rtos_milliseconds_t *timeout_ms_p);

void
rtos_condvar_signal(
    struct rtos_condvar *rtos_condvar_p);

void
rtos_condvar_broadcast(
    struct rtos_condvar *rtos_condvar_p);

void
rtos_timer_init(
    _IN_  const char *timer_name_p,
    _IN_  rtos_timer_function_t *timer_function_p,
    _OUT_ struct rtos_timer *rtos_timer_p);

void
rtos_timer_start(
    _INOUT_ struct rtos_timer *rtos_timer_p,
    _IN_ rtos_milliseconds_t expiration_time_in_ms);

void
rtos_timer_stop(
    _INOUT_ struct rtos_timer *rtos_timer_p);

void rtos_queue_init(
	_IN_  const char *queue_name_p,
	_IN_ bool use_mutex,
	_OUT_ struct rtos_queue *queue_p);

void
rtos_queue_add(
    _INOUT_ struct rtos_queue *queue_p,
    _INOUT_ struct glist_node *elem_p);

struct glist_node *
rtos_queue_remove(
    _INOUT_ struct rtos_queue *queue_p,
    _IN_ rtos_milliseconds_t timeout_ms);

void
rtos_capture_failure_data(
    _IN_ const char *failure_str,
    _IN_ uintptr_t arg1,
    _IN_ uintptr_t arg2,
    _IN_ void *failure_location);

void
rtos_set_fdc_params(
    _IN_ bool assert_break_point_on,
    _IN_ bool exception_debugger_on);

_THREAD_CALLERS_ONLY_
void
rtos_console_putchar(
    _UNUSED_ void *unused_arg_p,
    _IN_ uint8_t c);

_THREAD_CALLERS_ONLY_
uint8_t
rtos_console_getchar(void);

void
rtos_lcd_putchar(
    _INOUT_ struct rtos_lcd_putchar_attributes *lcd_putchar_attr_p,
    _IN_ uint8_t c);

lcd_color_t
rtos_lcd_draw_tile(
    _IN_ lcd_x_t x,
    _IN_ lcd_y_t y,
    _IN_ lcd_color_t fill_color);

_THREAD_CALLERS_ONLY_
void
rtos_thread_set_comp_region(
    _IN_ void *start_addr,
    _IN_ size_t size,
    _IN_ uint32_t flags,
    _OUT_ struct mpu_region_range *old_comp_region_p);

_THREAD_CALLERS_ONLY_
void
rtos_thread_restore_comp_region(
    _IN_ const struct mpu_region_range *old_comp_region_p);

_THREAD_CALLERS_ONLY_
void
rtos_thread_set_tmp_region(
    _IN_ void *start_addr,
    _IN_ size_t size,
    _IN_ uint32_t flags);

_THREAD_CALLERS_ONLY_
void
rtos_thread_unset_tmp_region(void);

void rtos_pointer_circular_buffer_init(
        _IN_  const char *name_p,
        _IN_ uint16_t num_entries,
        _IN_ void **storage_array_p,
        _IN_ struct rtos_mutex *cb_mutex_p,
        _OUT_ struct rtos_circular_buffer *circ_buf_p);

bool rtos_pointer_circular_buffer_write(
        _INOUT_ struct rtos_circular_buffer *circ_buf_p,
        _IN_ void *entry_value,
        _IN_ bool wait_if_full);

bool rtos_pointer_circular_buffer_read(
        _INOUT_ struct rtos_circular_buffer *circ_buf_p,
        _OUT_ void **entry_value_p,
        _IN_ bool wait_if_empty,
	_INOUT_ rtos_milliseconds_t *timeout_ms_p);

void rtos_byte_circular_buffer_init(
        _IN_  const char *name_p,
        _IN_ uint16_t num_entries,
        _IN_ uint8_t *storage_array_p,
        _IN_ struct rtos_mutex *cb_mutex_p,
        _OUT_ struct rtos_circular_buffer *circ_buf_p);

bool rtos_byte_circular_buffer_write(
        _INOUT_ struct rtos_circular_buffer *circ_buf_p,
        _IN_ uint8_t entry_value,
        _IN_ bool wait_if_full);

bool rtos_byte_circular_buffer_read(
        _INOUT_ struct rtos_circular_buffer *circ_buf_p,
        _OUT_ uint8_t *entry_value_p,
        _IN_ bool wait_if_empty,
	_INOUT_ rtos_milliseconds_t *timeout_ms_p);

bool rtos_circular_buffer_is_empty(
	_IN_ struct rtos_circular_buffer *circ_buf_p);

void rtos_queue_init(
	_IN_  const char *queue_name_p,
	_IN_ bool use_mutex,
	_OUT_ struct rtos_queue *queue_p);

void
rtos_queue_add(
    _INOUT_ struct rtos_queue *queue_p,
    _INOUT_ struct glist_node *elem_p);

struct glist_node *
rtos_queue_remove(
    _INOUT_ struct rtos_queue *queue_p,
    _IN_ rtos_milliseconds_t timeout_ms);

fdc_error_t
rtos_app_system_call(
    _IN_ rtos_app_system_call_function_t *rtos_app_system_call_function_p,
    _INOUT_ void *arg_p);

bool
rtos_enter_privileged_mode(void);

_THREAD_CALLERS_ONLY_
void
rtos_exit_privileged_mode(void);

bool
rtos_in_privileged_mode(void);

static inline void
rtos_restore_privilege_mode(bool privilege_mode_before)
{
    if (!privilege_mode_before) {
        rtos_exit_privileged_mode();
    }
}

_THREAD_CALLERS_ONLY_
bool
rtos_thread_enable_fpu(void);

_THREAD_CALLERS_ONLY_
void
rtos_thread_disable_fpu(void);

static inline void
rtos_thread_restore_fpu_mode(bool enabled_before)
{
    if (!enabled_before) {
        rtos_thread_disable_fpu();
    }
}

void
rtos_capture_fdc_msg_vprintf(const char *fmt, va_list va);

void
rtos_capture_stack_trace(_IN_ uint_fast8_t num_entries_to_skip,
                         _OUT_ uintptr_t trace_buff[],
		         _INOUT_ uint8_t *num_entries_p);

bool
rtos_disable_preemption(void);

void
rtos_restore_preemption_state(bool preemption_state_before);

#ifdef __cplusplus
}
#endif

#endif /* _McRTOS */
