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

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-function"

static bool rtos_dbg_parse_command(
    const char *cmd_line,
    _IN_ const struct rtos_execution_context *current_execution_context_p,
    _IN_ const uint32_t *stack_p);

static void rtos_dbg_display_help(void);

static void rtos_dbg_display_cpu_registers(
    _IN_ const struct rtos_execution_context *current_execution_context_p,
    _IN_ const uint32_t *stack_p);

static void 
rtos_dbg_dump_execution_context(
    _IN_ const struct rtos_execution_context *current_execution_context_p);

static void 
rtos_dbg_dump_memory(
    _IN_ const void *addr,
    _IN_ size_t size);

static void
rtos_dbg_signature_to_string(
    _IN_ uint32_t signature,
    _OUT_ char *str_buffer);

/**
 * Enters McRTOS debugger from the hard fault exception handler.
 *
 * @param   current_execution_context_p: Pointer to current execution context
 *
 * @pre     interrupts are disabled
 * @pre     current_execution_context_p->ctx_cpu_saved_registers[] contains the
 *          latest non-pre-saved CPU registers of the current context
 */
void
rtos_hard_fault_exception_handler(
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

#   if 0
    cpu_instruction_t *instruction_p = (cpu_instruction_t *)(stack_p[CPU_REG_PC]);

    /*
     * If the instruction that caused the exception is not breakpoint,
     * trigger a software reset when exiting the McRTOS debugger:
     */
    if ((*instruction_p & THUMB_INSTR_OP_CODE_MASK) != BKPT_OP_CODE_MASK) {
        NVIC_SystemReset();
    }
#   endif

    /*
     * Change the saved PC to point to the instruction after the faulting
     * instruction, otherwise we will fall into an infinite loop:
     */
    stack_p[CPU_REG_PC] += sizeof(cpu_instruction_t); 
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
    bool quit;

    DEBUG_BLINK_LED(LED_RED_MASK); // ???
    turn_on_rgb_led(LED_RED_MASK);
   
    do {
        debug_printf("\nMcRTOS debugger> ");
        read_command_line(
            (putchar_func_t *)uart_putchar_with_polling,
            (getchar_func_t *)uart_getchar_with_polling,
            (void *)g_console_serial_port_p,
            g_McRTOS_p->rts_command_line_buffer,
            RTOS_COMMAND_LINE_BUFFER_SIZE);

        quit = rtos_dbg_parse_command(
                    g_McRTOS_p->rts_command_line_buffer,
                    current_execution_context_p,
                    stack_p);

    } while (!quit);

    turn_off_rgb_led(LED_RED_MASK);
}

static bool
rtos_dbg_parse_command(
    const char *cmd_line,
    _IN_ const struct rtos_execution_context *current_execution_context_p,
    _IN_ const uint32_t *stack_p)
{
    uint8_t c = cmd_line[0]; // ???
    bool quit = false;

    switch (c)
    {
        case 'h':
            rtos_dbg_display_help();
            break;

        case 'r':
            rtos_dbg_display_cpu_registers(
                current_execution_context_p, stack_p);
            break;

        case 'c':
            rtos_dbg_dump_execution_context(
                current_execution_context_p);
            break;

        case 'q':
            quit = true;
            break;

        case 'k':
            board_reset();
            /*UNREACHABLE*/
            break;

        default:
            debug_printf("Invalid command: \'%s\' (type h for help)\n", cmd_line);
            break;
    }

    return quit;
}


static void 
rtos_dbg_display_help(void)
{
    debug_printf("\nMcRTOS debugger commands\n");
    debug_printf("\tr - display CPU registers\n");
    debug_printf("\tc - dump current execution context\n");
    debug_printf("\tq - quit McRTOS command mode\n");
    debug_printf("\tk - reset CPU\n");
}


static void 
rtos_dbg_display_cpu_registers(
    _IN_ const struct rtos_execution_context *current_execution_context_p,
    _IN_ const uint32_t *stack_p)
{
    debug_printf("\nCPU registers:\n");

#if DEFINED_ARM_CLASSIC_ARCH()
    debug_printf(
        "r0:   %#x\n"
        "r1:   %#x\n"
        "r2:   %#x\n"
        "r3:   %#x\n"
        "r4:   %#x\n"
        "r5:   %#x\n"
        "r6:   %#x\n"
        "r7:   %#x\n"
        "r8:   %#x\n"
        "r9:   %#x\n"
        "r10:  %#x\n"
        "r11:  %#x\n"
        "r12:  %#x\n"
        "sp:   %#x\n"
        "lr:   %#x\n"
        "pc:   %#x\n"
        "cpsr: %#x\n",
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
        "r0:        %#x\n"
        "r1:        %#x\n"
        "r2:        %#x\n"
        "r3:        %#x\n"
        "r4:        %#x\n"
        "r5:        %#x\n"
        "r6:        %#x\n"
        "r7:        %#x\n"
        "r8:        %#x\n"
        "r9:        %#x\n"
        "r10:       %#x\n"
        "r11:       %#x\n"
        "r12:       %#x\n"
        "sp:        %#x\n"
        "lr:        %#x\n"
        "pc:        %#x\n"
        "psr:       %#x\n"
        "lre:       %#x\n"
        "msp:       %#x\n"
        "psp:       %#x\n"
        "primask:   %#x\n"
        "control:   %#x\n",
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


static void 
rtos_dbg_dump_execution_context(
    _IN_ const struct rtos_execution_context *execution_context_p)
{
    debug_printf("\nCurrent Execution Context:\n");
    if (execution_context_p->ctx_signature != RTOS_EXECUTION_CONTEXT_SIGNATURE) {
        char signature_str[sizeof(uint32_t) + 1];

        rtos_dbg_signature_to_string(
            execution_context_p->ctx_signature,
            signature_str);

        debug_printf("*** Error: Invalid signature for context %#p: %#x (%s)\n",
            execution_context_p, 
            execution_context_p->ctx_signature,
            signature_str);
    }

    debug_printf("Name: %s\n", execution_context_p->ctx_name_p);
}


static void 
rtos_dbg_dump_memory(
    _IN_ const void *addr,
    _IN_ size_t size)
{
}


static void
rtos_dbg_signature_to_string(
    _IN_ uint32_t signature,
    _OUT_ char *str_buffer)
{
    char *signature_as_chars = (char *)&signature;
    uint32_t i;

    for (i = 0; i < sizeof(uint32_t); i++) {
        if (IS_PRINT(signature_as_chars[i])) { 
            str_buffer[i] = signature_as_chars[i];
        } else {
            str_buffer[i] = '.';
        }
    }

    str_buffer[i] = '\0';
}
