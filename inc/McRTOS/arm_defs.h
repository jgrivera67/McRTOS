/**
 * @file arm_defs.h
 *
 * SoC-independent ARM declarations to be included in both assembly and C code
 *
 * NOTE: Only preprocessor constructs should be used here.
 *
 * @author German Rivera 
 */ 

#ifndef _ARM_DEFS_H
#define _ARM_DEFS_H

#define DEFINED_ARM_CLASSIC_ARCH() \
        (defined(__ARM_ARCH_4T__) || \
         defined(__ARM_ARCH_7A__) || \
         defined(__ARM_ARCH_7R__))

#define DEFINED_ARM_V7_ARCH() \
        (defined(__ARM_ARCH_7A__) || \
         defined(__ARM_ARCH_7R__) || \
         defined(__ARM_ARCH_7M__))

#ifndef CPPUTEST_COMPILATION
#   if !DEFINED_ARM_CLASSIC_ARCH() && !defined(__ARM_ARCH_7M__)
#       error "CPU architecture not supported"
#   endif
#endif

#if DEFINED_ARM_CLASSIC_ARCH()

#   define ARM_MODE_USER  0x10      /* Normal User Mode                             */
#   define ARM_MODE_FIQ   0x11      /* FIQ Fast Interrupts Mode                     */
#   define ARM_MODE_IRQ   0x12      /* IRQ Standard Interrupts Mode                 */
#   define ARM_MODE_SVC   0x13      /* Supervisor Interrupts Mode                   */
#   define ARM_MODE_ABORT 0x17      /* Abort Processing memory Faults Mode          */
#   define ARM_MODE_UNDEF 0x1B      /* Undefined Instructions Mode                  */
#   define ARM_MODE_SYS   0x1F      /* System Running in Privileged Operating Mode  */

#   define ARM_MODE_MASK  0x1F
   
#   define I_BIT          0x80      /* disable IRQ when I bit is set */
#   define F_BIT          0x40      /* disable FIQ when F bit is set */
#   define T_BIT          0x20      /* switch to Thumb mode when T bit is set */

#   define N_FLAG_BIT     0x80000000   /* Negative flag is set */
#   define Z_FLAG_BIT     0x40000000   /* Zero flag is set */
#   define C_FLAG_BIT     0x20000000   /* Carry flag is set */
#   define V_FLAG_BIT     0x10000000   /* Overflow flag is set */

#   define ARM_INTERRUPTS_DISABLED_MASK    (I_BIT|F_BIT)

    /**
     * Address of the ARM IRQ vector
     */
#   define ARM_IRQ_VECTOR_ADDRESS      0x00000018

    /**
     * Bit mask for the SWI instruction immediate operand (lower 24 bits)
     */
#   define ARM_SWI_IMMEDIATE_OPERAND_MASK   0x00FFFFFF

    /**
     * Symbolic names of the CPU registers, defined in the order in which they
     * are to be saved in memory in a context switch
     *
     * NOTE: sp and lr are out of order here, as they cannot be restored
     * in the same "ldmia ^" as r0-r12 and pc, because they are banked.
     */
#   define CPU_REG_R0      0x0
#   define CPU_REG_R1      0x1
#   define CPU_REG_R2      0x2
#   define CPU_REG_R3      0x3
#   define CPU_REG_R4      0x4
#   define CPU_REG_R5      0x5
#   define CPU_REG_R6      0x6
#   define CPU_REG_R7      0x7
#   define CPU_REG_R8      0x8
#   define CPU_REG_R9      0x9
#   define CPU_REG_R10     0xa
#   define CPU_REG_R11     0xb
#   define CPU_REG_R12     0xc
#   define CPU_REG_SP      0xd
#   define CPU_REG_LR      0xe
#   define CPU_REG_PC      0xf
#   define CPU_REG_CPSR    0x10

    /**
     * Number of CPU registers that need to be saved in a context switch
     */
#   define CPU_NUM_REGISTERS    (1 + 16)

#   define CPU_MODE_IS_UNPRIVILEGED(_cpsr) \
        (((_cpsr) & ARM_MODE_MASK) == ARM_MODE_USER)

#   define CPU_MODE_IS_PRIVILEGED(_cpsr) \
        (((_cpsr) & ARM_MODE_MASK) == ARM_MODE_SYS)

#   define CPU_MODE_IS_SUPERVISOR(_cpsr) \
        (((_cpsr) & ARM_MODE_MASK) == ARM_MODE_SVC)

#   define CPU_MODE_IS_INTERRUPT(_cpsr) \
        (((_cpsr) & ARM_MODE_MASK) == ARM_MODE_IRQ)

#   define CPU_INTERRUPTS_ARE_ENABLED(_cpsr) \
        (((_cpsr) & ARM_INTERRUPTS_DISABLED_MASK) == 0)

#   define CPU_INTERRUPTS_ARE_DISABLED(_cpsr) \
        (((_cpsr) & ARM_INTERRUPTS_DISABLED_MASK) != 0)

#else  /* __ARM_ARCH_7M__ */

#   error "TODO: Add defines here for ARMv7-M"

#endif /* #if DEFINED_ARM_CLASSIC_ARCH() */

/**
 * Get current CPU ID
 */
#if SOC_NUM_CPU_CORES > 1 && DEFINED_ARM_V7_ARCH()
#   error "TODO: Define SOC_GET_CURRENT_CPU_ID() using CP15 ID register"
#else
#   define SOC_GET_CURRENT_CPU_ID() ((cpu_id_t)0)
#endif

/**
 * ARM CPU word size in bytes
 */
#define ARM_CPU_WORD_SIZE_IN_BYTES  4

/**
 * ARM CPU word size in bits
 */
#define ARM_CPU_WORD_SIZE_IN_BITS   (ARM_CPU_WORD_SIZE_IN_BYTES * 8)

/*
 * System call numbers that can be specified as the immediate operand
 * of the SWI instruction for ARMv4 or the SVC instruction for ARMv7:
 */
#define  RTOS_CREATE_THREAD_SYSTEM_CALL                 0x0
#define  RTOS_THREAD_DELAY_SYSTEM_CALL                  0x1
#define  RTOS_THREAD_ABORT_SYSTEM_CALL                  0x2
#define  RTOS_THREAD_CONDVAR_WAIT_SYSTEM_CALL           0x3
#define  RTOS_THREAD_CONDVAR_SIGNAL_SYSTEM_CALL         0x4
#define  RTOS_CREATE_MUTEX_SYSTEM_CALL                  0x5
#define  RTOS_MUTEX_ACQUIRE_SYSTEM_CALL                 0x6
#define  RTOS_MUTEX_RELEASE_SYSTEM_CALL                 0x7
#define  RTOS_CREATE_CONDVAR_SYSTEM_CALL                0x8
#define  RTOS_CONDVAR_WAIT_SYSTEM_CALL                  0x9
#define  RTOS_CONDVAR_WAIT_INTERRUPT_SYSTEM_CALL        0xa
#define  RTOS_CONDVAR_SIGNAL_SYSTEM_CALL                0xb
#define  RTOS_CONDVAR_BROADCAST_SYSTEM_CALL             0xc
#define  RTOS_CREATE_TIMER_SYSTEM_CALL                  0xd
#define  RTOS_TIMER_START_SYSTEM_CALL                   0xe
#define  RTOS_TIMER_STOP_SYSTEM_CALL                    0xf
#define  RTOS_CAPTURE_FAILURE_DATA_SYSTEM_CALL          0x10
#define  RTOS_CONSOLE_PUTCHAR_SYSTEM_CALL               0x11
#define  RTOS_CONSOLE_GETCHAR_SYSTEM_CALL               0x12
#define  RTOS_LCD_PUTCHAR_SYSTEM_CALL                   0x13
#define  RTOS_LCD_DRAW_TILE_SYSTEM_CALL                 0x14
#define  RTOS_APP_SYSTEM_CALL                           0x15
#define  RTOS_THREAD_SELF_SYSTEM_CALL                   0x16
#define  RTOS_THREAD_NAME_SYSTEM_CALL                   0x17
#define  RTOS_THREAD_CONDVAR_WAIT_INTERRUPT_SYSTEM_CALL 0x18
#define  RTOS_THREAD_YIELD_SYSTEM_CALL                  0x19

/*
 * CAUTION: This macro needs to be updated when new system
 * calls are added.
 */
#define __LAST_RTOS_SYSTEM_CALL  RTOS_THREAD_YIELD_SYSTEM_CALL

/**
 * Number of system calls supported:
 */ 
#define RTOS_NUM_SYSTEM_CALLS   (__LAST_RTOS_SYSTEM_CALL + 1)

/*
 * McRTOS CPU execution modes
 */
#define RTOS_INVALID_CPU_MODE           0x0
#define RTOS_UNPRIVILEGED_THREAD_MODE   0x1
#define RTOS_PRIVILEGED_THREAD_MODE     0x2
#define RTOS_INTERRUPT_MODE             0x3

/*
 * Offsets of struct rtos_execution_context fields accessed from
 * assembly code
 */
#define RTOS_CTX_CPU_REGISTERS_OFFSET   (ARM_CPU_WORD_SIZE_IN_BYTES * 2)

/*
 * Offsets of struct rtos_interrupt fields accessed from
 * assembly code
 */
#define RTOS_INT_CPU_CONTROLLER_P_OFFSET    ARM_CPU_WORD_SIZE_IN_BYTES
#define RTOS_INT_ARG_P_OFFSET               (ARM_CPU_WORD_SIZE_IN_BYTES * 2)

/*
 * Offsets of struct rtos_cpu_controller fields accessed from
 * assembly code
 */
#define RTOS_CPC_CURRENT_EXECUTION_CONTEXT_P_OFFSET \
        ARM_CPU_WORD_SIZE_IN_BYTES

/**
 * Number of CPU cores
 */
#if defined(LPC2478_SOC) || defined(LM4F120_SOC)

#   define SOC_NUM_CPU_CORES   1
#else
#   error "No system on chip specified"
#endif /* SoC-specific */

#endif /* _ARM_DEFS_H */
