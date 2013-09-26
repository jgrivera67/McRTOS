/**
 * @file McRTOS_debugger.c
 *
 * McRTOS command-line debugger
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */
#include "McRTOS_kernel_services.h"
#include "McRTOS_internals.h"
#include "arm_defs.h"
#include <stdint.h>

static void rtos_dbg_display_help(void);

static void rtos_dbg_display_cpu_registers(
    _IN_ const struct rtos_execution_context *current_execution_context_p,
    _IN_ const uint32_t *stack_p);


/**
 * Enters McRTOS debugger from an exception handler.
 *
 * @param   current_execution_context_p: Pointer to current execution context
 *
 * @pre     interrupts are disabled
 * @pre     current_execution_context_p->ctx_cpu_saved_registers[] contains the
 *          latest non-pre-saved CPU registers of the current context
 */
void
rtos_enter_debugger(
        _IN_ const struct rtos_execution_context *current_execution_context_p)
{
    rtos_execution_stack_entry_t *stack_p;

    /*
     * Determine what stack pointer was in use before the exception
     */
    if (current_execution_context_p->
            ctx_cpu_saved_registers[CPU_REG_LR_ON_EXC_ENTRY] ==
        CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP) {
        stack_p = (rtos_execution_stack_entry_t *)
                    current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_PSP];
    } else {
        stack_p = (rtos_execution_stack_entry_t *)
                    current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_MSP];
    }

    capture_unexpected_hard_fault(
            (void *)stack_p[CPU_REG_PC], 0, stack_p[CPU_REG_PSR]);

    rtos_run_debugger(current_execution_context_p, stack_p);

    uint32_t *instruction_p = (uint32_t *)(stack_p[CPU_REG_PC]);

    /*
     * If the instruction that caused the exception is not breakpoint,
     * trigger a software reset when exiting the McRTOS debugger:
     */
    if ((*instruction_p & THUMB_INSTR_OP_CODE_MASK) != BKPT_OP_CODE_MASK) {
        NVIC_SystemReset();
    }
}


/**
 * Runs a command-line low-level debugger on the console serial port
 *
 * @param   current_execution_context_p: Pointer to current execution context
 * @param   stack_p: stack pointer before the exception that causes the 
 *          debugger to run.
 *
 * @pre     interrupts are disabled
 */
void
rtos_run_debugger(
    _IN_ const struct rtos_execution_context *current_execution_context_p,
    _IN_ const uint32_t *stack_p)
{
    DEBUG_BLINK_LED(LED_RED_MASK); // ???
    turn_on_rgb_led(LED_RED_MASK);

    for ( ; ; )
    {
        debug_printf("\nMcRTOS debugger> ");

        uint8_t c = uart_getchar_with_polling(g_console_serial_port_p);

        debug_printf("%c\n", c);
        switch (c)
        {
            case 'h':
                rtos_dbg_display_help();
                break;

            case 'r':
                rtos_dbg_display_cpu_registers(
                    current_execution_context_p, stack_p);
                break;

            case 'q':
                goto Exit;

            default:
                debug_printf("Invalid command: \'%c\' (type h for help)\n", c);
                break;
        }
    }

Exit:
    turn_off_rgb_led(LED_RED_MASK);
}


static void 
rtos_dbg_display_help(void)
{
    debug_printf("\nMcRTOS debugger commands\n");
    debug_printf("\tr - display CPU registers\n");
    debug_printf("\tq - quit McRTOS command mode\n");
}


static void 
rtos_dbg_display_cpu_registers(
    _IN_ const struct rtos_execution_context *current_execution_context_p,
    _IN_ const uint32_t *stack_p)
{
    debug_printf("\nCPU registers:\n");

#if DEFINED_ARM_CLASSIC_ARCH()
    debug_printf(
        "r0:   0x%x\n"
        "r1:   0x%x\n"
        "r2:   0x%x\n"
        "r3:   0x%x\n"
        "r4:   0x%x\n"
        "r5:   0x%x\n"
        "r6:   0x%x\n"
        "r7:   0x%x\n"
        "r8:   0x%x\n"
        "r9:   0x%x\n"
        "r10:  0x%x\n"
        "r11:  0x%x\n"
        "r12:  0x%x\n"
        "sp:   0x%x\n"
        "lr:   0x%x\n"
        "pc:   0x%x\n"
        "cpsr: 0x%x\n",
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R0],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R1],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R2],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R3],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R4],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R5],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R6],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R7],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R8],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R9],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R10],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R11],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R12],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_LR],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_PC],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_CPSR]);

#elif DEFINED_ARM_CORTEX_M_ARCH()
    debug_printf(
        "r0:        0x%x\n"
        "r1:        0x%x\n"
        "r2:        0x%x\n"
        "r3:        0x%x\n"
        "r4:        0x%x\n"
        "r5:        0x%x\n"
        "r6:        0x%x\n"
        "r7:        0x%x\n"
        "r8:        0x%x\n"
        "r9:        0x%x\n"
        "r10:       0x%x\n"
        "r11:       0x%x\n"
        "r12:       0x%x\n"
        "sp:        0x%x\n"
        "lr:        0x%x\n"
        "pc:        0x%x\n"
        "psr:       0x%x\n"
        "lre:       0x%x\n"
        "msp:       0x%x\n"
        "psp:       0x%x\n"
        "primask:   0x%x\n"
        "control:   0x%x\n",
        stack_p[CPU_REG_R0],
        stack_p[CPU_REG_R1],
        stack_p[CPU_REG_R2],
        stack_p[CPU_REG_R3],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R4],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R5],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R6],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R7],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R8],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R9],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R10],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_R11],
        stack_p[CPU_REG_R12],
        stack_p,
        stack_p[CPU_REG_LR],
        stack_p[CPU_REG_PC],
        stack_p[CPU_REG_PSR],
        current_execution_context_p->
            ctx_cpu_saved_registers[CPU_REG_LR_ON_EXC_ENTRY],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_MSP],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_PSP],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_PRIMASK],
        current_execution_context_p->ctx_cpu_saved_registers[CPU_REG_CONTROL]);
#else
    #error "unsupported CPU architecture"
#endif
}

