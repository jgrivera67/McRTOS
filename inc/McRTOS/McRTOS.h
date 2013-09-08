/**
 * @file McRTOS.h
 *
 * McRTOS public interfaces
 *
 * @author German Rivera 
 */ 
#ifndef _McRTOS_H
#define _McRTOS_H

#include "McRTOS_config_parameters.h"
#include "hardware_abstractions.h"
#include "failure_data_capture.h"

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

#define RTOS_HIGHEST_THREAD_PRIORITY    UINT8_C(0)
#define RTOS_LOWEST_THREAD_PRIORITY     (RTOS_NUM_THREAD_PRIORITIES - 1)

/**
 * Console channels
 */
enum rtos_console_channels
{
    RTOS_COMMAND_LINE_CONSOLE_CHANNEL = 0,
    RTOS_APP_CONSOLE_CHANNEL,

    /*
     * Add new entries above here
     */ 
    RTOS_CONSOLE_CHANNEL_NONE
};

C_ASSERT(RTOS_CONSOLE_CHANNEL_NONE == RTOS_NUM_CONSOLE_CHANNELS);

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
typedef uint32_t rtos_thread_prio_bitmap_t;

C_ASSERT(
    sizeof(rtos_thread_prio_bitmap_t) * 8 == RTOS_NUM_THREAD_PRIORITIES);

/**
 * Per CPU interrupts bit map type
 */
typedef uint32_t rtos_per_cpu_interrupts_bitmap_t;

C_ASSERT(
    sizeof(rtos_per_cpu_interrupts_bitmap_t) * 8 >= SOC_NUM_INTERRUPT_CHANNELS);

/**
 * Range type for console channels
 */
typedef _RANGE_(0, RTOS_NUM_CONSOLE_CHANNELS)
        uint8_t rtos_console_channels_t;

/**
 * Range type for LCD channels
 */
typedef _RANGE_(0, RTOS_NUM_LCD_CHANNELS)
        uint8_t rtos_lcd_channels_t;

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
 * Block of parameters for creating a thread
 */ 
struct rtos_thread_creation_params
{ 
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

    /**
     * Stdio console channel assigned to the thread
     */
    rtos_console_channels_t p_console_channel;

    /**
     * Stdio LCD channel assigned to the thread
     */
    rtos_lcd_channels_t p_lcd_channel;

    /**
     * Pointer to area to store the pointer to the created McRTOS thread object
     */
    struct rtos_thread **const p_thread_pp;
}; 

/**
 * Block of parameters for creating a mutex
 */ 
struct rtos_mutex_creation_params
{ 
    /**
     * Pointer to mutex name string for debugging purposes
     */ 
    const char *p_name_p;

    /**
     * Pointer to created mutex object returned by McRTOS
     */
    struct rtos_mutex **const p_mutex_pp;
}; 


/**
 * Block of parameters for creating a condvar
 */ 
struct rtos_condvar_creation_params
{ 
    /**
     * Pointer to condvar name string for debugging purposes
     */ 
    const char *p_name_p;

    /**
     * Pointer to created condvar object returned by McRTOS
     */
    struct rtos_condvar **const p_condvar_pp;
};

/**
 * Block of parameters for creating a timer
 */ 
struct rtos_timer_creation_params
{ 
    /**
     * Pointer to timer name string for debugging purposes
     */ 
    const char *p_name_p;

    /**
     * Pointer to the callback function to be invoked when the timer expires
     */
    rtos_timer_function_t *p_function_p;

    /**
     * Pointer to created timer object returned by McRTOS
     */
    struct rtos_timer **const p_timer_pp;
}; 

/**
 * Block of parameters for creating a msg_channel
 */ 
struct rtos_msg_channel_creation_params
{ 
    /**
     * Pointer to msg_channel name string for debugging purposes
     */ 
    const char *p_name_p;

    /**
     * Pointer to created msg_channel object returned by McRTOS
     */
    struct rtos_msg_channel **const p_msg_channel_pp;
};

/**
 * Block of parameters for creating a object_pool
 */ 
struct rtos_object_pool_creation_params
{ 
    /**
     * Pointer to object_pool name string for debugging purposes
     */ 
    const char *p_name_p;

    /**
     * Pointer to created object_pool object returned by McRTOS
     */
    struct rtos_object_pool **const p_object_pool_pp;
};

/**
 * McRTOS per-cpu startup application configuration structure
 */ 
struct rtos_per_cpu_startup_app_configuration
{
    /**
     * Pointer to the application-specific idle thread hook function
     */
    rtos_idle_thread_hook_function_t *const stc_idle_thread_hook_function_p;

    /**
     * Number of application threads to be started on this CPU when
     * McRTOS starts running. These threads can create other threads
     * if necessary by calling rtos_create_thread().
     */ 
    const rtos_num_app_threads_t stc_num_autostart_threads;

    /**
     * Pointer to array of thread creation structures, for the application
     * threads to be started on this CPU core, when McRTOS starts running.
     */ 
    const struct rtos_thread_creation_params *const stc_autostart_threads_p;

    /**
     * Number of application mutexes to be created for this CPU core,
     * when McRTOS starts running. 
     */ 
    const rtos_num_app_mutexes_t stc_num_prebuilt_mutexes;

    /**
     * Pointer to array of mutex creation structures.
     */ 
    const struct rtos_mutex_creation_params *const stc_prebuilt_mutexes_p;

    /**
     * Number of application condvars to be created for this CPU core,
     * when McRTOS starts running. 
     */ 
    const rtos_num_app_condvars_t stc_num_prebuilt_condvars;

    /**
     * Pointer to array of condvar creation structures.
     */ 
    const struct rtos_condvar_creation_params *const stc_prebuilt_condvars_p;

    /**
     * Number of application timers to be created for this CPU core,
     * when McRTOS starts running. 
     */ 
    const rtos_num_app_timers_t stc_num_prebuilt_timers;

    /**
     * Pointer to array of timer creation structures.
     */ 
    const struct rtos_timer_creation_params *const stc_prebuilt_timers_p;

    /**
     * Number of application message channels to be created for this CPU core,
     * when McRTOS starts running. 
     */ 
    const rtos_num_app_msg_channels_t stc_num_prebuilt_msg_channels;

    /**
     * Pointer to array of timer creation structures.
     */ 
    const struct rtos_msg_channel_creation_params *const stc_prebuilt_msg_channels_p;

    /**
     * Number of application object pools to be created for this CPU core,
     * when McRTOS starts running. 
     */ 
    const rtos_num_app_object_pools_t stc_num_prebuilt_object_pools;

    /**
     * Pointer to array of object pool creation structures.
     */ 
    const struct rtos_object_pool_creation_params *const stc_prebuilt_object_pools_p;
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
    _IN_ const struct rtos_per_cpu_startup_app_configuration *rtos_app_config_p);

/*
 * Functions to be invoked from Application threads, which run in
 * unprivileged mode:
 */

#ifdef MCRTOS_DYNAMIC_OBJECT_CREATION

fdc_error_t
rtos_create_thread(
    _IN_ const struct rtos_thread_creation_params *params_p);

fdc_error_t
rtos_create_mutex(
    _IN_ const struct rtos_mutex_creation_params *params_p);

fdc_error_t
rtos_create_condvar(
    _IN_ const struct rtos_condvar_creation_params *params_p);
 
fdc_error_t
rtos_create_timer(
    _IN_ const struct rtos_timer_creation_params *params_p);

#endif /* MCRTOS_DYNAMIC_OBJECT_CREATION */

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

_THREAD_CALLERS_ONLY_
void
rtos_mutex_acquire(
    _IN_ struct rtos_mutex *rtos_mutex_p);
 
_THREAD_CALLERS_ONLY_
void
rtos_mutex_release(
    _IN_ struct rtos_mutex *rtos_mutex_p);

_THREAD_CALLERS_ONLY_
fdc_error_t
rtos_condvar_wait(
    _IN_ struct rtos_condvar *rtos_condvar_p,
    _IN_  struct rtos_mutex *rtos_mutex_p);

_THREAD_CALLERS_ONLY_
fdc_error_t
rtos_condvar_wait_interrupt(
    _IN_ struct rtos_condvar *rtos_condvar_p);

void
rtos_condvar_signal(
    struct rtos_condvar *rtos_condvar_p);

void
rtos_condvar_broadcast(
    struct rtos_condvar *rtos_condvar_p);

void
rtos_timer_start(
    _INOUT_ struct rtos_timer *rtos_timer_p,
    _IN_ rtos_milliseconds_t expiration_time_in_ms);

void
rtos_timer_stop(
    _INOUT_ struct rtos_timer *rtos_timer_p);

void
rtos_capture_failure_data(
    _IN_ const char *failure_str,
    _IN_ uintptr_t arg1,
    _IN_ uintptr_t arg2,
    _IN_ uintptr_t arg3,
    _IN_ void *failure_location);

_THREAD_CALLERS_ONLY_
void
rtos_console_putchar(
    _UNUSED_ void *unused_arg_p,
    _IN_ uint8_t c);

_THREAD_CALLERS_ONLY_
uint8_t
rtos_console_getchar(_IN_ bool wait);

void
rtos_lcd_putchar(
    _INOUT_ struct rtos_lcd_putchar_attributes *lcd_putchar_attr_p,
    _IN_ uint8_t c);

lcd_color_t
rtos_lcd_draw_tile(
    _IN_ lcd_x_t x,
    _IN_ lcd_y_t y,
    _IN_ lcd_color_t fill_color);

fdc_error_t
rtos_app_system_call(
    _IN_ rtos_app_system_call_function_t *rtos_app_system_call_function_p,
    _INOUT_ void *arg_p);

#endif /* _McRTOS */
