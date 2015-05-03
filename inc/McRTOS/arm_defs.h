/**
 * @file arm_defs.h
 *
 * SoC-independent ARM declarations to be included in both assembly and C code
 *
 * Copyright (C) 2013 German Rivera
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

#define DEFINED_ARM_CORTEX_M_ARCH() \
        (defined(__ARM_ARCH_6M__) || \
         defined(__ARM_ARCH_7EM__))

#ifndef CPPUTEST_COMPILATION
#   if !DEFINED_ARM_CLASSIC_ARCH() && !DEFINED_ARM_CORTEX_M_ARCH()
#       error "CPU architecture not supported"
#   endif
#endif

#ifdef _CPU_CYCLES_MEASURE_
#   define  _MEASURE_INTERRUPTS_DISABLED_TIME_
#else
#   undef   _MEASURE_INTERRUPTS_DISABLED_TIME_
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

    /**
     * Address of the LPC2478 VIC's VICAddress register
     */
#   define LPC2478_VIC_ADDRESS_ADDR    0xFFFFFF00

    /**
     * Value of the PC when the processor executes the instruction at
     * address ARM_IRQ_VECTOR_ADDRESS
     */
#   define ARM_IRQ_VECTOR_INSTR_PC     (ARM_IRQ_VECTOR_ADDRESS + 8)

    /**
     * Value that needs to be subtracted from the PC when executing the
     * instruction at address ARM_IRQ_VECTOR_ADDRESS, if that instruction is a
     * a PC-relative indirect branch to jump to the address contained in the
     * VIC's VICAddress register
     */
#   define LPC2478_VIC_ADDRESS_PC_DELTA \
        (ARM_IRQ_VECTOR_INSTR_PC - LPC2478_VIC_ADDRESS_ADDR)


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

#define GET_FUNCTION_ADDRESS(_func_p)   ((uintptr_t)(_func_p)

#elif DEFINED_ARM_CORTEX_M_ARCH()

    /**
     * Bit mask for the SVC instruction immediate operand (lower 8 bits)
     */
#   define ARM_SVC_IMMEDIATE_OPERAND_MASK   0xFF

    /**
     * Symbolic names of the CPU registers, defined in the order in which they
     * are saved automatically by the processor, in the current stack, upon
     * exception entry.
     */
#   define CPU_REG_R0      0x0
#   define CPU_REG_R1      0x1
#   define CPU_REG_R2      0x2
#   define CPU_REG_R3      0x3
#   define CPU_REG_R12     0x4
#   define CPU_REG_LR      0x5
#   define CPU_REG_PC      0x6
#   define CPU_REG_PSR     0x7

#   define CPU_NUM_PRE_SAVED_REGISTERS  8

   /**
     * Symbolic names of remaining CPU registers, defined in the order in which
     * they are to be explicitly saved in memory in a context switch
     */
#   define CPU_REG_R4               0x0
#   define CPU_REG_R5               0x1
#   define CPU_REG_R6               0x2
#   define CPU_REG_R7               0x3
#   define CPU_REG_R8               0x4
#   define CPU_REG_R9               0x5
#   define CPU_REG_R10              0x6
#   define CPU_REG_R11              0x7
#   define CPU_REG_MSP              0x8
#   define CPU_REG_PSP              0x9
#   define CPU_REG_LR_ON_EXC_ENTRY  0xA
#   define CPU_REG_CONTROL	    0xB

    /**
     * Number of CPU registers that need to be explicitly saved in a context
     * switch, as they are not automatically saved in the stack by the CPU.
     */
#   define CPU_NUM_SAVED_REGISTERS  (8 + 4)

    /**
     * Number of 32-bit FPU registers
     */
#   define FPU_NUM_SINGLE_REGISTERS 32

    /**
     * Values that LR can be set to, to return from an exception:
     */
#   define CPU_EXC_RETURN_TO_HANDLER_MODE		0xFFFFFFF1
#   define CPU_EXC_RETURN_TO_THREAD_MODE_USING_MSP	0xFFFFFFF9
#   define CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP	0xFFFFFFFD
#   define CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP_FPU	0xFFFFFFED

    /**
     * Bitmasks to decode the op-code and operand fields of a THUMB instruction
     */
#   define THUMB_INSTR_OP_CODE_MASK 0xFF00
#   define THUMB_INSTR_OPEEAND_MASK 0x00FF

    /**
     * bkpt instruction opcode mask
     */
#   define  BKPT_OP_CODE_MASK       0xBE00

    /**
     * svc instruction opcode mask
     */
#   define  SVC_OP_CODE_MASK        0xDF00

    /**
     * bl instruction opcode mask
     */
#   define  BL_OP_CODE_MASK         0xF000

    /**
     * bx instruction opcode mask
     */
#   define  BX_OP_CODE_MASK         0x4700

    /**
     * pop instruction opcode mask
     */
#   define  POP_OP_CODE_MASK        0xBD00

    /**
     * "bx lr" instruction
     */
#   define  BX_LR_INSTRUCTION       0x4770

    /*
     * Bit masks for the IPSR register
     */
#   define CPU_REG_IPSR_EXCEPTION_NUMBER_MASK   0x3F

    /*
     * Bit masks for the EPSR register
     */
#   define CPU_REG_EPSR_THUMB_STATE_MASK    (0x1 << 24)

    /*
     * Bit masks for the PRIMASK register
     */
#   define CPU_REG_PRIMASK_PM_MASK   0x1

    /*
     * Bit masks for the CONTROL register
     */
#   define CPU_REG_CONTROL_nPRIV_MASK   (0x1 << 0)
#   define CPU_REG_CONTROL_SPSEL_MASK   (0x1 << 1)
#   define CPU_REG_CONTROL_FPCA_MASK	(0x1 << 2)

    /*
     * Cortex-M SCB ICSR register address
     */
#   define CPU_SCB_ICSR_REGISTER_ADDR   0xE000ED04

    /*
     * Mask for the PendSV set-pending bit in the SCB ICSR register
     */
#   define CPU_SCB_ICSR_PENDSVSET_MASK  (0x1 << 28)

    /**
     * Number of interrupt priorities for interrupts with configurable
     * priority, with 0 being the highest priority and lower priorities
     * begin > 0.
     * The Reset, NMI, and fault exceptions have fixed priorities, with
     * negative priority values, always have higher priority than any
     * other exception.
     */
#   if defined(KL25Z_SOC)
#	define SOC_NUM_INTERRUPT_PRIORITIES 4
#   elif defined(K64F_SOC)
#	define SOC_NUM_INTERRUPT_PRIORITIES 16
#   elif defined(LM4F120_SOC)
#	define SOC_NUM_INTERRUPT_PRIORITIES 8 
#   else
#	error "SOC not supported"
#   endif

    /**
     * Base interrupt vector number for external IRQs
     */
#   define CORTEX_M_IRQ_VECTOR_BASE     16

    /**
     * Interrupt vector numbers for Fault exceptions
     */
#   define INT_HARD_FAULT	    3
#   define INT_MEMORY_MANAGEMENT    4
#   define INT_BUS_FAULT	    5
#   define INT_USAGE_FAULT          6
#   define INT_DEBUG_MONITOR        12

    /**
     * Convert an IRQ number to an interrupt vector number (vector table index)
     * IRQ number can be negative (for internal interrupts)
     */
#   define IRQ_NUMBER_TO_VECTOR_NUMBER(_irq_number) \
            (((IRQn_Type)(_irq_number)) + CORTEX_M_IRQ_VECTOR_BASE)

    /**
     * Convert an interrupt vector number (vector table index) to an IRQ number.
     */
#   define VECTOR_NUMBER_TO_IRQ_NUMBER(_vector_number) \
            ((IRQn_Type)((_vector_number) - CORTEX_M_IRQ_VECTOR_BASE))

#   define SOC_HIGHEST_INTERRUPT_PRIORITY  0
#   define SOC_LOWEST_INTERRUPT_PRIORITY   (SOC_NUM_INTERRUPT_PRIORITIES - 1)

#   define CPU_MODE_IS_UNPRIVILEGED(_reg_control_value) \
        (((_reg_control_value) & CPU_REG_CONTROL_nPRIV_MASK) != 0)

#   define CPU_MODE_IS_PRIVILEGED(_reg_control_value, _reg_ipsr_value) \
        (CPU_MODE_IS_HANDLER(_reg_ipsr_value) || \
	 ((_reg_control_value) & CPU_REG_CONTROL_nPRIV_MASK) == 0)

#   define CPU_USING_MSP_STACK_POINTER(_reg_control_value) \
        (((_reg_control_value) & CPU_REG_CONTROL_SPSEL_MASK) == 0)

#   define CPU_USING_PSP_STACK_POINTER(_reg_control_value) \
        (((_reg_control_value) & CPU_REG_CONTROL_SPSEL_MASK) != 0)

#   define CPU_MODE_IS_SUPERVISOR(_reg_ipsr_value) \
        (((_reg_ipsr_value) & CPU_REG_IPSR_EXCEPTION_NUMBER_MASK) == 11)

#   define CPU_MODE_IS_THREAD(_reg_ipsr_value) \
        (((_reg_ipsr_value) & CPU_REG_IPSR_EXCEPTION_NUMBER_MASK) == 0)

#   define CPU_MODE_IS_HANDLER(_reg_ipsr_value) \
        (((_reg_ipsr_value) & CPU_REG_IPSR_EXCEPTION_NUMBER_MASK) != 0)

#   define CPU_MODE_IS_INTERRUPT(_reg_ipsr_value) \
        (((_reg_ipsr_value) & CPU_REG_IPSR_EXCEPTION_NUMBER_MASK) >= 15)

#   define CPU_MODE_IS_PENDSV_EXCEPTION(_reg_ipsr_value) \
        (((_reg_ipsr_value) & CPU_REG_IPSR_EXCEPTION_NUMBER_MASK) == INT_PendableSrvReq)

#   define CPU_MODE_IS_HARD_FAULT_EXCEPTION(_reg_ipsr_value) \
        (((_reg_ipsr_value) & CPU_REG_IPSR_EXCEPTION_NUMBER_MASK) == INT_Hard_Fault)

#   define CPU_INTERRUPTS_ARE_ENABLED(_reg_primask_value) \
        (((_reg_primask_value) & CPU_REG_PRIMASK_PM_MASK) == 0)

#   define CPU_INTERRUPTS_ARE_DISABLED(_reg_primask_value) \
        (((_reg_primask_value) & CPU_REG_PRIMASK_PM_MASK) != 0)

#define GET_FUNCTION_ADDRESS(_func_p)   ((uintptr_t)(_func_p) & ~0x1)

#else

#   error "CPU architecture not supported"

#endif /* #if DEFINED_ARM_CLASSIC_ARCH() */

/**
 * Context switch types for context switch tracing (must fit in 4 bits)
 */
#define RTOS_CSW_THREAD_TO_INTERRUPT        0x1
#define RTOS_CSW_ENTERING_NESTED_INTERRUPT  0x2
#define RTOS_CSW_EXITING_NESTED_INTERRUPT   0x3
#define RTOS_CSW_INTERRUPT_TO_THREAD        0x4
#define RTOS_CSW_THREAD_TO_THREAD           0x5
#define RTOS_CSW_RESET_TO_THREAD            0x6
#define RTOS_CSW_EXITING_HARD_FAULT         0x7
#define RTOS_CSW_THREAD_TO_EARLY_NESTED_INTERRUPT   0x8
#define RTOS_CSW_EXITING_EARLY_NESTED_INTERRUPT     0x9

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
#define  RTOS_THREAD_INIT_SYSTEM_CALL                           0x0
#define  RTOS_THREAD_DELAY_SYSTEM_CALL                          0x1
#define  RTOS_THREAD_ABORT_SYSTEM_CALL                          0x2
#define  RTOS_THREAD_CONDVAR_WAIT_SYSTEM_CALL                   0x3
#define  RTOS_THREAD_CONDVAR_SIGNAL_SYSTEM_CALL                 0x4
#define  RTOS_MUTEX_INIT_SYSTEM_CALL                            0x5
#define  RTOS_MUTEX_ACQUIRE_SYSTEM_CALL                         0x6
#define  RTOS_MUTEX_RELEASE_SYSTEM_CALL                         0x7
#define  RTOS_CONDVAR_INIT_SYSTEM_CALL                          0x8
#define  RTOS_CONDVAR_WAIT_SYSTEM_CALL                          0x9
#define  RTOS_CONDVAR_SIGNAL_SYSTEM_CALL                        0xb
#define  RTOS_CONDVAR_BROADCAST_SYSTEM_CALL                     0xc
#define  RTOS_TIMER_INIT_SYSTEM_CALL                            0xd
#define  RTOS_TIMER_START_SYSTEM_CALL                           0xe
#define  RTOS_TIMER_STOP_SYSTEM_CALL                            0xf
#define  RTOS_CAPTURE_FAILURE_DATA_SYSTEM_CALL                  0x10
#define  RTOS_SET_FDC_PARAMS_SYSTEM_CALL                        0x11
#define  RTOS_CONSOLE_PUTCHAR_SYSTEM_CALL                       0x12
#define  RTOS_CONSOLE_GETCHAR_SYSTEM_CALL                       0x13
#define  RTOS_LCD_PUTCHAR_SYSTEM_CALL                           0x14
#define  RTOS_LCD_DRAW_TILE_SYSTEM_CALL                         0x15
#define  RTOS_APP_SYSTEM_CALL                                   0x16
#define  RTOS_THREAD_SELF_SYSTEM_CALL                           0x17
#define  RTOS_THREAD_NAME_SYSTEM_CALL                           0x18
#define  RTOS_THREAD_CONDVAR_WAIT_INTERRUPT_SYSTEM_CALL         0x19
#define  RTOS_THREAD_YIELD_SYSTEM_CALL                          0x1a
#define  RTOS_CALLER_IS_THREAD_SYSTEM_CALL		        0x1b
#define  RTOS_THREAD_ADD_MPU_DATA_REGION_SYSTEM_CALL	        0x1c
#define  RTOS_THREAD_REMOVE_TOP_MPU_DATA_REGION_SYSTEM_CALL	0x1d
#define  RTOS_THREAD_REPLACE_TOP_MPU_DATA_REGION_SYSTEM_CALL    0x1e
#define  RTOS_THREAD_RESTORE_TOP_MPU_DATA_REGION_SYSTEM_CALL    0x1f
#define  RTOS_THREAD_ENABLE_FPU_SYSTEM_CALL		        0x20
#define  RTOS_THREAD_DISABLE_FPU_SYSTEM_CALL		        0x21
#define  RTOS_POINTER_CIRCULAR_BUFFER_INIT_SYSTEM_CALL          0x22
#define  RTOS_POINTER_CIRCULAR_BUFFER_WRITE_SYSTEM_CALL         0x23
#define  RTOS_POINTER_CIRCULAR_BUFFER_READ_SYSTEM_CALL          0x24
#define  RTOS_BYTE_CIRCULAR_BUFFER_INIT_SYSTEM_CALL             0x25
#define  RTOS_BYTE_CIRCULAR_BUFFER_WRITE_SYSTEM_CALL            0x26
#define  RTOS_BYTE_CIRCULAR_BUFFER_READ_SYSTEM_CALL             0x27
#define  RTOS_CIRCULAR_BUFFER_IS_EMPTY_SYSTEM_CALL              0x28
#define  RTOS_QUEUE_INIT_SYSTEM_CALL                            0x29
#define  RTOS_QUEUE_ADD_SYSTEM_CALL                             0x2a
#define  RTOS_QUEUE_REMOVE_SYSTEM_CALL                          0x2b
#define  RTOS_GET_TICKS_SYSTEM_CALL                             0x2c
#define  RTOS_CAPTURE_FDC_MSG_VPRINTF_SYSTEM_CALL               0x2d

/*
 * CAUTION: This macro needs to be updated when new system
 * calls are added.
 */
#define __LAST_RTOS_SYSTEM_CALL	RTOS_CAPTURE_FDC_MSG_VPRINTF_SYSTEM_CALL

/**
 * Number of system calls supported:
 */
#define RTOS_NUM_SYSTEM_CALLS   (__LAST_RTOS_SYSTEM_CALL + 1)

/**
 * SVC operand value used by rtos_enter_privileged_mode()
 */
#define RTOS_ENTER_PRIVILEGED_MODE_SVC_CODE		0xff

/*
 * McRTOS CPU execution modes
 */
#define RTOS_INVALID_CPU_MODE           0x0
#define RTOS_RESET_MODE                 0x1
#define RTOS_UNPRIVILEGED_THREAD_MODE   0x2
#define RTOS_PRIVILEGED_THREAD_MODE     0x4
#define RTOS_INTERRUPT_MODE             0x8

/**
 * Execution context types
 */
#define RTOS_INVALID_CONTEXT    0x0
#define RTOS_RESET_CONTEXT      0x1
#define RTOS_THREAD_CONTEXT     0x2
#define RTOS_INTERRUPT_CONTEXT  0x8

/*
 * Offsets of struct McRTOS fields accessed from assembly code
 */
#define RTOS_RTS_CPU_CONTROLLERS_OFFSET 0

/*
 * Offsets of struct rtos_execution_context fields accessed from
 * assembly code
 */
#define RTOS_CTX_CPU_MODE_OFFSET        (ARM_CPU_WORD_SIZE_IN_BYTES * 2)
#define RTOS_CTX_CONTEXT_TYPE_OFFSET    (RTOS_CTX_CPU_MODE_OFFSET + 1)
#define RTOS_CTX_CPU_REGISTERS_OFFSET   (ARM_CPU_WORD_SIZE_IN_BYTES * 3)

/*
 * Offsets of struct rtos_interrupt fields accessed from
 * assembly code
 */
#define RTOS_INT_CPU_CONTROLLER_P_OFFSET    (ARM_CPU_WORD_SIZE_IN_BYTES * 1)

/*
 * Offsets of struct rtos_cpu_controller fields accessed from
 * assembly code
 */
#define RTOS_CPC_CURRENT_EXECUTION_CONTEXT_P_OFFSET \
        (ARM_CPU_WORD_SIZE_IN_BYTES * 1)

/**
 * Number of CPU cores
 */
#if defined(LPC2478_SOC) || defined(LM4F120_SOC) || defined(KL25Z_SOC) || defined(K64F_SOC)

#   define SOC_NUM_CPU_CORES   1
#else
#   error "No SoC specified"
#endif /* SoC-specific */

/**
 * Get current CPU ID
 */
#if SOC_NUM_CPU_CORES > 1
#   error "TODO: Define SOC_GET_CURRENT_CPU_ID() using CP15 ID register"
#else
#   define SOC_GET_CURRENT_CPU_ID() ((cpu_id_t)0)
#endif

#endif /* _ARM_DEFS_H */
