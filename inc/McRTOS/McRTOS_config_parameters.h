/**
 * @file McRTOS_config_parameters.h
 *
 * McRTOS compile-time configuration parameters
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */
#ifndef _McRTOS_CONFIG_PARAMS_H
#define _McRTOS_CONFIG_PARAMS_H

#include <stdint.h>
#include "hardware_abstractions.h"
#include "arm_defs.h"

/**
 * Tick Timer frequency in Hz
 */
#define RTOS_TICK_TIMER_FREQUENCY    UINT32_C(500)  /* 500 Hz or every 2 ms */

/**
 * Tick timer period in milliseconds
 */
#define RTOS_MILLISECONDS_PER_TICK (1000 / RTOS_TICK_TIMER_FREQUENCY)

/**
 * Number of ticks for a system heartbeat
 */
#define RTOS_HEARTBEAT_TICKS    UINT32_C(256)

/**
 * Execution stack size for threads (in number of stack entries)
 */
#define RTOS_THREAD_STACK_NUM_ENTRIES   UINT32_C(256)

/**
 * Thread stack size in bytes
 */
#define RTOS_THREAD_STACK_SIZE_IN_BYTES \
        (RTOS_THREAD_STACK_NUM_ENTRIES * ARM_CPU_WORD_SIZE_IN_BYTES)

/**
 * Stack overflow buffer size (in number of stack entries) for
 * thread and interrupt stacks
 */
#define RTOS_STACK_OVERFLOW_BUFFER_SIZE_IN_ENTRIES 6

/**
 * Execution stack size for interrupts (in number of stack entries)
 */
#if DEFINED_ARM_CLASSIC_ARCH()
#   define RTOS_INTERRUPT_STACK_NUM_ENTRIES    UINT32_C(128)

#elif DEFINED_ARM_CORTEX_M_ARCH()
    /*
     * For Cortex-M there is only one interrupt stack shared among all
     * nested exceptions
     */
#   define RTOS_INTERRUPT_STACK_NUM_ENTRIES \
            (UINT32_C(96) * SOC_NUM_INTERRUPT_PRIORITIES)

#endif

/**
 * Number of thread priorities supported
 */
//#define RTOS_NUM_THREAD_PRIORITIES  ARM_CPU_WORD_SIZE_IN_BITS
#define RTOS_NUM_THREAD_PRIORITIES  8

/*
 * Maximum number of McRTOS interrupt objects in the system
 */
#define RTOS_MAX_NUM_INTERRUPTS 8

C_ASSERT(RTOS_MAX_NUM_INTERRUPTS < SOC_NUM_INTERRUPT_CHANNELS);

/**
 * Time slice in number of timer ticks for each thread of the same priority.
 * Threads with the same priority are scheduled in a round-robin fashion,
 * giving each thread the CPU for this number of ticks.
 */
#define RTOS_THREAD_TIME_SLICE_IN_TICKS UINT8_C(8)

/*
 * Command line size in bytes including null terminator
 */
#define RTOS_COMMAND_LINE_BUFFER_SIZE    UINT32_C(128)

/**
 * Maximum size of the McRTOS struct in bytes
 */
#define RTOS_MAX_McRTOS_DATA_SIZE   (SOC_SRAM_SIZE / 2)

/**
 * Maximum size of the rtos_cpu_controller struct in bytes
 */
#define RTOS_MAX_McRTOS_CPU_CONTROLLER_SIZE \
        ((SOC_SRAM_SIZE / 2) / SOC_NUM_CPU_CORES)

/**
 * Maximum size of the fdc_info struct in bytes
 */
#if defined(LPC2478_SOC)
#   define RTOS_MAX_FDC_INFO_SIZE  UINT32_C(4*1024)
#elif defined(KL25Z_SOC) || defined(K20D5_SOC)
#   define RTOS_MAX_FDC_INFO_SIZE  UINT32_C(1*1024)
#elif defined(K64F_SOC)
#   define RTOS_MAX_FDC_INFO_SIZE  UINT32_C(4*1024)
#elif defined(LM4F120_SOC)
#   define RTOS_MAX_FDC_INFO_SIZE  UINT32_C(4*1024)
#else
#   error "No system on chip specified"
#endif

/**
 * Size of the buffer that holds the most recent output from capture_fdc_msg_printf()
 */
#if defined(LPC2478_SOC)
#   define RTOS_FDC_MSG_BUFFER_SIZE  UINT16_C(1024)
#elif defined(KL25Z_SOC) || defined(K20D5_SOC)
#   define RTOS_FDC_MSG_BUFFER_SIZE  UINT16_C(256)
#elif defined(K64F_SOC)
#   define RTOS_FDC_MSG_BUFFER_SIZE  UINT16_C(2048)
#elif defined(LM4F120_SOC)
#   define RTOS_FDC_MSG_BUFFER_SIZE  UINT16_C(2048)
#else
#   error "No system on chip specified"
#endif

/**
 * Maximum number of failure records that can be captured
 */
#define RTOS_MAX_NUM_FAILURE_RECORDS    UINT32_C(8)

/**
 * Maximum number of unexpected exception records that can be captured
 */
#define RTOS_MAX_NUM_UNEXPECTED_EXCEPTION_RECORDS   UINT32_C(4)

/**
 * Size of the context switch trace buffer in number of entries
 */
#define RTOS_NUM_CONTEXT_SWITCH_TRACE_BUFFER_ENTRIES    UINT16_C(64)

#ifdef LCD_SUPPORTED
    /**
     * Number of LCD channels
     */
#   define RTOS_NUM_LCD_CHANNELS UINT8_C(3)
#endif

#if __MPU_PRESENT == 1
    /**
     * Number of global MPU regions (besides the background region)
     */
#   define RTOS_NUM_GLOBAL_MPU_REGIONS UINT8_C(1)

    /**
     * Maximum number of MPU data regions used by the current
     * thread, including the thread's stack region.
     */
#   define RTOS_MAX_MPU_THREAD_DATA_REGIONS   UINT8_C(7)

#elif defined(K64F_SOC)
    /**
     * Number of global MPU regions
     */
#   define RTOS_NUM_GLOBAL_MPU_REGIONS UINT8_C(4)

    /**
     * Maximum number of MPU data regions used by the current
     * thread, including the thread's stack region.
     */
#   define RTOS_MAX_MPU_THREAD_DATA_REGIONS   UINT8_C(8)

#else
#   error "No MPU supported"
#endif /* __MPU_PRESENT == 1 */

/**
 * Number of MPU regions required by McRTOS, including one
 * region for the stack of the current thread
  */
#define RTOS_MAX_MPU_REGIONS \
	(RTOS_NUM_GLOBAL_MPU_REGIONS + RTOS_MAX_MPU_THREAD_DATA_REGIONS)

/**
 * Maximum time (microseconds) that interrupts are expected to be disabled
 */
#ifdef DEBUG
#   define RTOS_MAX_TIME_INTERRUPTS_DISABLED UINT32_C(128)
#   define RTOS_BUILD_FLAVOR   "debug"
#elif defined(_RELIABILITY_CHECKS_)
#   define RTOS_MAX_TIME_INTERRUPTS_DISABLED UINT32_C(64)
#   define RTOS_BUILD_FLAVOR   "reliability"
#else
#   define RTOS_MAX_TIME_INTERRUPTS_DISABLED UINT32_C(32)
#   define RTOS_BUILD_FLAVOR   "performance"
#endif

/*
 * Application-dependent parameters
 */

/**
 * Maximum number of application threads that can exist in the system
 */
#if defined(LPC2478_SOC)
#   define RTOS_MAX_NUM_APP_THREADS UINT8_C(16)
#elif defined(KL25Z_SOC) || defined(K20D5_SOC)
#   define RTOS_MAX_NUM_APP_THREADS UINT8_C(6)
#elif defined(K64F_SOC)
#   define RTOS_MAX_NUM_APP_THREADS UINT8_C(64)
#elif defined(LM4F120_SOC)
#   define RTOS_MAX_NUM_APP_THREADS UINT8_C(8)
#else
#   error "No system on chip specified"
#endif


/*
 * If the total size of application thread stacks is larger than 25% of
 * the SoC SRAM, use DRAM for the application thread stacks
 */
#if (RTOS_MAX_NUM_APP_THREADS * RTOS_THREAD_STACK_SIZE_IN_BYTES) > \
    (SOC_SRAM_SIZE / 4)
#   if defined(RTOS_APP_THREAD_STACK_DRAM_POOL_SIZE)
#       define RTOS_USE_DRAM_FOR_APP_THREAD_STACKS

        C_ASSERT(
            RTOS_MAX_TOTAL_APP_THREAD_STACKS_SIZE <=
            RTOS_APP_THREAD_STACK_DRAM_POOL_SIZE);
#   else
#       error "Cannot allocate app thread stacks"
#   endif
#else
#   undef RTOS_USE_DRAM_FOR_APP_THREAD_STACKS
#endif

/**
 * Maximum number of software timers that can exist in the system
 */
#define RTOS_MAX_NUM_APP_TIMERS             UINT8_C(4)

/**
 * Maximum number of mutexes that can exist in the system
 */
#define RTOS_MAX_NUM_APP_MUTEXES            UINT8_C(4)

/**
 * Maximum number of condition variables that can exist in the system
 */
#define RTOS_MAX_NUM_APP_CONDVARS           UINT8_C(4)

/**
 * Maximum number of message channels that can exist in the system
 */
#define RTOS_MAX_NUM_APP_MSG_CHANNELS       UINT8_C(0)

/**
 * Maximum number of object pools that can exist in the system
 */
#define RTOS_MAX_NUM_APP_OBJECT_POOLS       UINT8_C(0)

#endif /* _McRTOS_CONFIG_PARAMS_H */
