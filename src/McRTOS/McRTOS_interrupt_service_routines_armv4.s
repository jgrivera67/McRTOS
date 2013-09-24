/**
 * @file McRTOS_interrupt_service_routines_armv4.s
 *
 * McRTOS interrupt service routines
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */

#include "arm_defs.h"

/*
 * Imported functions
 */

.extern rtos_k_enter_interrupt
.extern rtos_k_exit_interrupt

.text
.arm
.code   32


/**
 * Macro that saves the CPU registers of the current McRTOS execution
 * context, upon entry to a McRTOS-aware ISR.
 *
 * This macro must be invoked at the beginning of every McRTOS-aware ISR.
 *
 * @param   _g_rtos_interrupt_p_: global variable that is a pointer to the
 *          McRTOS interrupt object associated with the calling ISR.
 *
 * @param   _clear_interrupt_source_function_: function name of the
 *          function that clears the interrupt source that cause
 *          the calling ISR to run.
 *
 * INPUT REGISTERS:
 * - sp = banked SP of IRQ mode. sp has the value of the stack pointer
 *   register from the last interrupt context executed. For the first
 *   interrupt context executed since reset, it has the value set by the
 *   ARM Reset handler (ResetHandler), as defined in McRTOS_crt_armv4.s.
 *
 * OUTPUT REGISTERS:
 * r4 = \_g_rtos_interrupt_p_->int_arg_p
 *
 * CLOBBERED REGISTERS:
 * N/A
 */
.macro RTOS_ENTER_ISR _g_rtos_interrupt_p_, _clear_interrupt_source_function_
    /*
     * Save r0 on the IRQ mode's stack, so that we can use them as scratch
     * registers here:
     */
    push    {r0}

    /*
     * Capture in r0 the interrupted context's return-from-interrupt address
     * (which is in lr-4), to free r14
     */
    sub     r0, lr, #4

    /*
     * Set r14 to &(\_g_rtos_interrupt_p_->int_cpu_controller_p->
     *               cpc_current_execution_context_p->ctx_cpu_registers[0])
     */
    ldr     r14, =\_g_rtos_interrupt_p_
    ldr     r14, [r14]
    ldr     r14, [r14, #RTOS_INT_CPU_CONTROLLER_P_OFFSET]
    ldr     r14, [r14, #RTOS_CPC_CURRENT_EXECUTION_CONTEXT_P_OFFSET]
    add     r14,  r14, #RTOS_CTX_CPU_REGISTERS_OFFSET

    /*
     * Save interrupted context's registers, so that they appear in the following
     * order in memory:
     * - r0
     * - ..
     * - r12
     * - r13 (sp) 
     * - r14 (lr) 
     * - r15 (pc) - return-from-interrupt address
     * - cpsr
     *
     * At this point:
     * - r0 == interrupted context's pc
     * - IRQ mode's r14 == &execution_context_p->ctx_cpu_registers[0]
     * - Other registers are intact.
     */

    /* 
     * Save interrupted context's pc, to free r0:
     */
    str     r0, [r14, #(CPU_REG_PC * ARM_CPU_WORD_SIZE_IN_BYTES)] 

    /*
     * Restore r0
     */
    pop     {r0}
    
    /*
     * Save interrupted context's r0-r12, sp and lr:
     *
     * NOTE: The interrupted context's ARM mode was User mode or System
     * mode, as those are the only two modes that we allow to run with
     * interrupts enabled. 
     */
    stmia   r14, {r0-r14}^

    /* 
     * Save interrupted context's cspsr:
     */
    mrs     r0, spsr
    str     r0, [r14, #(CPU_REG_CPSR * ARM_CPU_WORD_SIZE_IN_BYTES)] 

    /*
     * Call rtos_k_enter_interrupt(\_g_rtos_interrupt_p_):
     */
    ldr     r4, =\_g_rtos_interrupt_p_
    ldr     r4, [r4]
    mov     r0, r4
    bl      rtos_k_enter_interrupt

    /*
     * Save r0 into r5
     *
     * r0 == stack pointer register to use for this ISR
     */
    mov     r5, r0

    /*
     * Call _clear_interrupt_source_function_(\_g_rtos_interrupt_p_->int_arg_p)
     *
     * r4 == \_g_rtos_interrupt_p_
     */
    ldr     r4, [r4, #RTOS_INT_ARG_P_OFFSET]
    mov     r0, r4
    bl      \_clear_interrupt_source_function_

    /*
     * Switch to System mode, keeping interrupts disabled:
     */
    msr    cpsr_c, #(ARM_MODE_SYS | ARM_INTERRUPTS_DISABLED_MASK)

    /*
     * Set the system mode's sp to the sp to use for the part of the ISR that runs with
     * interrupts enabled:
     *
     * r5 == stack pointer register to use for this ISR
     */
    mov     sp, r5

    /*
     * Re-enable interrupts (by turning off I_BIT and
     * F_BIT in the cpsr),  staying in sydtrm mode:
     *
     * NOTE: This is necessary so that higher priority interrupts can preempt this
     * ISR (nested interrupts).
     */
    msr     cpsr_c, #ARM_MODE_SYS

.endm


/**
 * Macro that restores CPU context upon exit from a McRTOS-aware ISR.
 *
 * This macro must be invoked at the end of every McRTOS-aware ISR.
 *
 * INPUT REGISTERS:
 * none
 *
 * OUTPUT REGISTERS:
 * none
 *
 * CLOBBERED REGISTERS:
 * N/A
 */
.macro RTOS_EXIT_ISR
    /*
     * Disable interrupts again:
     */
1:
    msr     cpsr_c, #(ARM_MODE_SYS | ARM_INTERRUPTS_DISABLED_MASK)
    mrs     r1, cpsr
    tst     r1, #ARM_INTERRUPTS_DISABLED_MASK
    beq     1b

    /*
     * Switch back to IRQ mode keeping interrupts disabled, to return correctly
     * from the interrupt:
     */
    msr     cpsr_c, #(ARM_MODE_IRQ | ARM_INTERRUPTS_DISABLED_MASK)

    /*
     * Branch to rtos_k_exit_interrupt()
     */
    b       rtos_k_exit_interrupt
.endm


/**
 * Macro that generates an ISR function
 *
 * @param   _interrupt_name_suffix_: Interrupt name suffix
 *
 * @param   _clear_interrupt_source_function_: clear interrupt source
 *          function name
 *
 * @param   _interrupt_handler_function_: Interrupt handler
 *          function name
 *
 * @param   _g_interrupt_handler_arg_p_ (optional): Name of global pointer 
 *          variable that points to device object to be passed in as argument
 *          to _interrupt_handler_function_
 *
 * INPUT REGISTERS:
 * N/A
 *
 * OUTPUT REGISTERS:
 * N/A
 *
 * CLOBBERED REGISTERS:
 * N/A
 */
.macro GEN_ISR_FUNCTION _interrupt_name_suffix_, \
                        _clear_interrupt_source_function_, \
                        _interrupt_handler_function_

.extern g_rtos_interrupt_\_interrupt_name_suffix_
.extern \_clear_interrupt_source_function_
.extern \_interrupt_handler_function_

/**
 * void isr\_interrupt_name_suffix_(void)
 */
.global isr_\_interrupt_name_suffix_

.func isr_\_interrupt_name_suffix_

isr_\_interrupt_name_suffix_:
    RTOS_ENTER_ISR g_rtos_interrupt_\_interrupt_name_suffix_, \
                   \_clear_interrupt_source_function_

    /*
     * r4 == g_rtos_interrupt_\_interrupt_name_suffix_->int_arg_p
     */

    /*
     * Call \_interrupt_handler_function_(
     *          g_rtos_interrupt_\_interrupt_name_suffix_->int_arg_p)
     */
    mov     r0, r4
    bl      \_interrupt_handler_function_

    RTOS_EXIT_ISR
.endfunc

.endm

/*
 * ISR functions:
 */

GEN_ISR_FUNCTION timer0, \
                 clear_timer_interrupt_source, \
                 timer_interrupt_handler

GEN_ISR_FUNCTION uart0, \
                 uart_interrupt_pre_handler, \
                 uart_interrupt_post_handler

GEN_ISR_FUNCTION gpio_ports, \
                 clear_gpio_ports_interrupt_source, \
                 gpio_ports_interrupt_handler

GEN_ISR_FUNCTION ssp_controller0, \
                 clear_ssp_controller_interrupt_source, \
                 ssp_controller_interrupt_handler

GEN_ISR_FUNCTION ssp_controller1, \
                 clear_ssp_controller_interrupt_source, \
                 ssp_controller_interrupt_handler

GEN_ISR_FUNCTION adc, \
                 clear_adc_interrupt_source, \
                 adc_interrupt_handler

.end
