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

static void 
rtos_dbg_dump_registers_saved_on_exception(
    _IN_ const uint32_t *before_exception_stack_p);

static void 
rtos_dbg_dump_execution_context(
    _IN_ const struct rtos_execution_context *execution_context_p);

static void 
rtos_dbg_dump_memory(
    _IN_ const uint32_t *addr,
    _IN_ uint32_t num_words,
    _IN_ const char *title);

static void
rtos_dbg_dump_context_switch_traces(void);


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
    rtos_execution_stack_entry_t *before_exception_stack_p;

    /*
     * Determine what stack pointer was in use before the exception
     */
    if (current_execution_context_p->
            ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
        CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP) {
        before_exception_stack_p = (rtos_execution_stack_entry_t *)
                    current_execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp;
    } else {
        before_exception_stack_p = (rtos_execution_stack_entry_t *)
                    current_execution_context_p->ctx_cpu_saved_registers.cpu_reg_msp;
    }

    capture_unexpected_hard_fault(
            (void *)before_exception_stack_p[CPU_REG_PC], 0,
            before_exception_stack_p[CPU_REG_PSR]);

    rtos_run_debugger(current_execution_context_p, before_exception_stack_p);

#   if 0
    cpu_instruction_t *instruction_p =
        (cpu_instruction_t *)(before_exception_stack_p[CPU_REG_PC]);

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
    before_exception_stack_p[CPU_REG_PC] += sizeof(cpu_instruction_t); 
}


/**
 * Runs a command-line low-level debugger on the console serial port
 *
 * @param   current_execution_context_p: Pointer to current execution context
 * @param   before_exception_stack_p: stack pointer before the exception that
 *          causes the debugger to run.
 *
 * @pre     interrupts are disabled
 */
void
rtos_run_debugger(
    _IN_ const struct rtos_execution_context *current_execution_context_p,
    _IN_ const uint32_t *before_exception_stack_p)
{
    bool quit;

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
                    before_exception_stack_p);

    } while (!quit);

    turn_off_rgb_led(LED_RED_MASK);
}

static bool
rtos_dbg_parse_command(
    const char *cmd_line,
    _IN_ const struct rtos_execution_context *current_execution_context_p,
    _IN_ const uint32_t *before_exception_stack_p)
{
    bool quit = false;

    uint8_t c = cmd_line[0]; // ???
    switch (c)
    {
        case '\0':
            break;

        case 'c':
            rtos_dbg_dump_execution_context(
                current_execution_context_p);
            break;

        case 'e':
            rtos_dbg_dump_registers_saved_on_exception(
                before_exception_stack_p);
            rtos_dbg_dump_memory(
                before_exception_stack_p, 32, "Stack before exception");
            break;

        case 'h':
            rtos_dbg_display_help();
            break;

        case 'q':
            quit = true;
            break;

        case 'r':
            board_reset();
            /*UNREACHABLE*/
            break;

        case 't':
            rtos_dbg_dump_context_switch_traces();
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
    debug_printf("McRTOS debugger commands\n");
    debug_printf("\tc - dump current execution context\n");
    debug_printf("\te - dump exception info\n");
    debug_printf("\tq - quit McRTOS command mode\n");
    debug_printf("\tr - reset CPU\n");
    debug_printf("\tt - dump context switch traces\n");
}


static void 
rtos_dbg_dump_registers_saved_on_exception(
    _IN_ const uint32_t *before_exception_stack_p)
{
    debug_printf(
        "sp before exception: %#x\n"
        "Registers pre-saved on exception entry:\n"
        "\tr0:  %#x\n"
        "\tr1:  %#x\n"
        "\tr2:  %#x\n"
        "\tr3:  %#x\n"
        "\tr12: %#x\n"
        "\tlr:  %#x\n"
        "\tpc:  %#x\n"
        "\tpsr: %#x\n",
        before_exception_stack_p,
        before_exception_stack_p[CPU_REG_R0],
        before_exception_stack_p[CPU_REG_R1],
        before_exception_stack_p[CPU_REG_R2],
        before_exception_stack_p[CPU_REG_R3],
        before_exception_stack_p[CPU_REG_R12],
        before_exception_stack_p[CPU_REG_LR],
        before_exception_stack_p[CPU_REG_PC],
        before_exception_stack_p[CPU_REG_PSR]);
}


static void 
rtos_dbg_dump_execution_context(
    _IN_ const struct rtos_execution_context *execution_context_p)
{
    static const char * const context_types[] = {
           [RTOS_INVALID_CONTEXT] = "RTOS_INVALID_CONTEXT",
           [RTOS_RESET_CONTEXT] = "RTOS_RESET_CONTEXT",
           [RTOS_THREAD_CONTEXT] =  "RTOS_THREAD_CONTEXT",
           [RTOS_INTERRUPT_CONTEXT] = "RTOS_INTERRUPT_CONTEXT"
    };

    debug_printf("Current Execution Context: %#p\n",
        execution_context_p);

    if (execution_context_p->ctx_signature != RTOS_EXECUTION_CONTEXT_SIGNATURE) {


        debug_printf("*** Error: Invalid signature for context %#p: %#x (%s)\n",
            execution_context_p, 
            execution_context_p->ctx_signature,
            signature_to_string(execution_context_p->ctx_signature));
    }

    debug_printf("\tName: %s\n", execution_context_p->ctx_name_p);
    debug_printf("\tType: %s\n", context_types[execution_context_p->ctx_context_type]);
    debug_printf("\tSaved CPU registers:\n");

#if DEFINED_ARM_CLASSIC_ARCH()
    debug_printf(
        "\t\tr0:   %#x\n"
        "\t\tr1:   %#x\n"
        "\t\tr2:   %#x\n"
        "\t\tr3:   %#x\n"
        "\t\tr4:   %#x\n"
        "\t\tr5:   %#x\n"
        "\t\tr6:   %#x\n"
        "\t\tr7:   %#x\n"
        "\t\tr8:   %#x\n"
        "\t\tr9:   %#x\n"
        "\t\tr10:  %#x\n"
        "\t\tr11:  %#x\n"
        "\t\tr12:  %#x\n"
        "\t\tsp:   %#x\n"
        "\t\tlr:   %#x\n"
        "\t\tpc:   %#x\n"
        "\t\tcpsr: %#x\n",
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R0],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R1],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R2],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R3],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R4],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R5],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R6],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R7],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R8],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R9],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R10],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R11],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R12],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_LR],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_PC],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_CPSR]);

#elif DEFINED_ARM_CORTEX_M_ARCH()

    const uint32_t *context_stack_p;

    if (execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT) {
        context_stack_p = (uint32_t *)execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp;
    } else {
        context_stack_p = (uint32_t *)execution_context_p->ctx_cpu_saved_registers.cpu_reg_msp;
    }

    debug_printf(
        "\t\tr0:        %#x\n"
        "\t\tr1:        %#x\n"
        "\t\tr2:        %#x\n"
        "\t\tr3:        %#x\n"
        "\t\tr4:        %#x\n"
        "\t\tr5:        %#x\n"
        "\t\tr6:        %#x\n"
        "\t\tr7:        %#x\n"
        "\t\tr8:        %#x\n"
        "\t\tr9:        %#x\n"
        "\t\tr10:       %#x\n"
        "\t\tr11:       %#x\n"
        "\t\tr12:       %#x\n"
        "\t\tsp:        %#x\n"
        "\t\tlr:        %#x\n"
        "\t\tpc:        %#x\n"
        "\t\tpsr:       %#x\n"
        "\t\tmsp:       %#x\n"
        "\t\tpsp:       %#x\n"
        "\t\tlre_on_exc:%#x\n",
        context_stack_p[CPU_REG_R0],
        context_stack_p[CPU_REG_R1],
        context_stack_p[CPU_REG_R2],
        context_stack_p[CPU_REG_R3],
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r4,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r5,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r6,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r7,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r8,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r9,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r10,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r11,
        context_stack_p[CPU_REG_R12],
        context_stack_p,
        context_stack_p[CPU_REG_LR],
        context_stack_p[CPU_REG_PC],
        context_stack_p[CPU_REG_PSR],
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_msp,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp,
        execution_context_p->
            ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry);
#else
    #error "unsupported CPU architecture"
#endif
}


static void 
rtos_dbg_dump_memory(
    _IN_ const uint32_t *addr,
    _IN_ uint32_t num_words,
    _IN_ const char *title)
{
    debug_printf("%s: Memory at %#p (%u words):\n",
        title, addr, num_words);

    for (uint32_t i = 0; i < num_words; i ++) {
        debug_printf("\t%3u: [%#p]: %#x\n", i, addr + i, addr[i]);
    }
}


static void
rtos_dbg_dump_context_switch_traces(void)
{
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;
   
    debug_printf(
        "Number of context switches: %u\n"
        "Next entry to fill: %u\n",
        fdc_info_p->fdc_context_switch_count,
        fdc_info_p->fdc_context_switch_trace_cursor);

    uint32_t num_trace_entries;

    if (fdc_info_p->fdc_context_switch_count < RTOS_NUM_CONTEXT_SWITCH_TRACE_BUFFER_ENTRIES) {
        num_trace_entries = fdc_info_p->fdc_context_switch_count;
    } else {
        num_trace_entries = RTOS_NUM_CONTEXT_SWITCH_TRACE_BUFFER_ENTRIES;
    }

    for (uint32_t i = 0; i < num_trace_entries;  i ++) {
        fdc_context_switch_trace_entry_t trace_entry = 
            fdc_info_p->fdc_context_switch_trace_buffer[i];

        uint32_t context_id = 
            GET_BIT_FIELD(trace_entry, FDC_CST_CONTEXT_ID_MASK, FDC_CST_CONTEXT_ID_SHIFT);

        uint32_t trace_context_type = 
            GET_BIT_FIELD(trace_entry, FDC_CST_CONTEXT_TYPE_MASK, FDC_CST_CONTEXT_TYPE_SHIFT);

        uint32_t last_switched_out_reason =
            GET_BIT_FIELD(trace_entry, FDC_CST_LAST_SWITCHED_OUT_REASON_MASK,
                          FDC_CST_LAST_SWITCHED_OUT_REASON_SHIFT);

        uint32_t priority =
            GET_BIT_FIELD(trace_entry, FDC_CST_CONTEXT_PRIORITY_MASK,
            FDC_CST_CONTEXT_PRIORITY_SHIFT);

        struct rtos_thread *thread_p = NULL;
        struct rtos_interrupt *interrupt_p = NULL;
        struct rtos_execution_context *execution_context_p;

        switch (trace_context_type) {
        case FDC_CST_RESET:
            execution_context_p = &cpu_controller_p->cpc_reset_execution_context;
            break;
        case FDC_CST_APPLICATION_THREAD:
            thread_p = &g_McRTOS_p->rts_app_threads[context_id];
            execution_context_p = &thread_p->thr_execution_context;
            break;
        case FDC_CST_SYSTEM_THREAD:
            thread_p = &cpu_controller_p->cpc_system_threads[context_id];
            execution_context_p = &thread_p->thr_execution_context;
            break;
        case FDC_CST_INTERRUPT:
            interrupt_p = &g_McRTOS_p->rts_interrupts[context_id];
            execution_context_p = &interrupt_p->int_execution_context;
            break;
        default:
            debug_printf("*** Error: Invalid trace entry (trace context type: %u)\n",
                trace_context_type);
        }

        debug_printf(
            "%3u: %s (context: %#p), last switched-out reason: %u, priority %u\n",
            i, execution_context_p->ctx_name_p, execution_context_p,
            last_switched_out_reason, priority
            );
        if (thread_p != NULL) {
            debug_printf(
                "\tstate history: %#x\n", thread_p->thr_state_history);
        }
    }
}
