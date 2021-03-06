/**
 * @file McRTOS_debugger.c
 *
 * McRTOS command-line debugger
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */
#include <stdint.h>
#include <McRTOS/McRTOS_kernel_services.h>
#include <McRTOS/McRTOS_internals.h>
#include <McRTOS/arm_defs.h>
#include <McRTOS/utils.h>
#include <McRTOS/failure_data_capture.h>

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-function"

struct __debugger {
    /**
     * Command-line buffer
     */
    char command_line_buffer[RTOS_COMMAND_LINE_BUFFER_SIZE];
};

struct debugger {
    struct __debugger;
}  __attribute__ ((aligned(SOC_MPU_REGION_ALIGNMENT(struct __debugger))));

C_ASSERT(sizeof(struct debugger) % SOC_MPU_REGION_ALIGNMENT(struct __debugger) == 0);

static bool rtos_dbg_parse_command(
    const char *cmd_line,
    _IN_ const struct rtos_execution_context *current_execution_context_p,
    _IN_ const uint32_t *stack_p);

static void rtos_dbg_display_help(void);

static void
rtos_dbg_dump_exception_info(
    _IN_ const uint32_t *before_exception_stack_p);

static void
rtos_dbg_dump_all_execution_contexts(void);

static void
rtos_dbg_dump_fdc_msg_buffer(void);

static void
rtos_dbg_dump_execution_context(
    _IN_ const struct rtos_execution_context *execution_context_p);

static void
rtos_dbg_dump_memory(
    _IN_ const uint32_t *addr,
    _IN_ uint32_t num_words,
    _IN_ const char *title);

static void
rtos_dbg_dump_stack_trace(
    _IN_ const struct rtos_execution_context *execution_context_p);

static void
rtos_dbg_dump_context_switch_traces(void);

static void
rtos_dbg_dump_cpu_controller(void);

#ifdef _BRANCH_MICRO_TRACING_
static void
debug_dump_micro_trace_buffer(void);
#endif

static void
debug_dump_captured_registers(void);

static struct debugger g_debugger;


/**
 * Enters McRTOS debugger from the hard fault exception handler.
 *
 * @param   current_execution_context_p: Pointer to current execution context
 * @param   exception_vector: Exception vector number
 *
 * @pre     interrupts are disabled
 * @pre     current_execution_context_p->ctx_cpu_saved_registers[] contains the
 *          latest non-pre-saved CPU registers of the current context
 */
void
rtos_common_fault_exception_handler(
        _IN_ const struct rtos_execution_context *current_execution_context_p,
	_IN_ enum cpu_core_internal_interrupt_vectors exception_vector)
{
#   ifdef _BRANCH_MICRO_TRACING_
    micro_trace_stop();
#   endif

    rtos_execution_stack_entry_t *before_exception_stack_p;

#   ifdef _RELIABILITY_CHECKS_
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    fdc_info_p->fdc_handling_exception = true;
#   endif

    capture_fdc_msg_printf(
        "Fault %u exception caught on context %#p\n",
	exception_vector,
        current_execution_context_p);

    if (g_McRTOS_p->rts_app_hardware_init_called) {
        g_McRTOS_p->rts_app_hardware_stop_p();
    }

    /*
     * Determine what stack pointer was in use before the exception
     */
    if (current_execution_context_p->
            ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
        CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP ||
	current_execution_context_p->
            ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry ==
        CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP_FPU) {
        before_exception_stack_p = (rtos_execution_stack_entry_t *)
                    current_execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp;
    } else {
        before_exception_stack_p = (rtos_execution_stack_entry_t *)
                    current_execution_context_p->ctx_cpu_saved_registers.cpu_reg_msp;
    }

#   ifdef _RELIABILITY_CHECKS_
    capture_unexpected_fault(
            (void *)before_exception_stack_p[CPU_REG_PC], 0,
            before_exception_stack_p[CPU_REG_PSR],
	    exception_vector);

    if (fdc_info_p->fdc_exception_debugger_on) {
        rtos_run_debugger(current_execution_context_p, before_exception_stack_p);
    } else {
        rtos_reboot();
    }
#   else
    rtos_run_debugger(current_execution_context_p, before_exception_stack_p);
#   endif

    /*
     * Change the saved PC to point to the instruction after the faulting
     * instruction, otherwise we will fall into an infinite loop:
     */
    before_exception_stack_p[CPU_REG_PC] += sizeof(cpu_instruction_t);

#   ifdef _RELIABILITY_CHECKS_
    fdc_info_p->fdc_handling_exception = false;
#   endif

#   ifdef _BRANCH_MICRO_TRACING_
    micro_trace_restart();
#   endif

    debugger_printf("*** Exiting debugger ***\n");
}


static uint32_t g_before_debugger_led_color = LED_COLOR_BLACK;

static void
turn_on_debugger_led(void)
{
    g_before_debugger_led_color = set_rgb_led_color(LED_COLOR_RED);
}


static void
turn_off_debugger_led(void)
{
    set_rgb_led_color(g_before_debugger_led_color);
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

    turn_on_debugger_led();

    do {
        debugger_printf("\nMcRTOS debugger> ");
        read_command_line(
            (putchar_func_t *)uart_putchar_with_polling,
            (getchar_func_t *)uart_getchar_with_polling,
            (void *)g_console_serial_port_p,
            g_debugger.command_line_buffer,
            RTOS_COMMAND_LINE_BUFFER_SIZE);

        quit = rtos_dbg_parse_command(
                    g_debugger.command_line_buffer,
                    current_execution_context_p,
                    before_exception_stack_p);

    } while (!quit);

    turn_off_debugger_led();
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

        case 'a':
            rtos_dbg_dump_all_execution_contexts();
            break;

        case 'c':
	    if (current_execution_context_p != NULL) {
		    rtos_dbg_dump_execution_context(
			current_execution_context_p);
	    }
            break;

        case 'd':
            rtos_dbg_dump_fdc_msg_buffer();
            break;

        case 'e':
	    if (before_exception_stack_p != NULL) {
		    rtos_dbg_dump_exception_info(before_exception_stack_p);
		    rtos_dbg_dump_stack_trace(current_execution_context_p);
	    }

            break;

        case 'h':
            rtos_dbg_display_help();
            break;

        case 'p':
            rtos_dbg_dump_cpu_controller();
            break;

        case 'q':
            quit = true;
            break;

        case 'r':
            rtos_reboot();
            /*UNREACHABLE*/
            break;

        case 't':
            rtos_dbg_dump_context_switch_traces();
            break;

        default:
            debugger_printf("Invalid command: \'%s\' (type h for help)\n", cmd_line);
    }

    return quit;
}


static void
rtos_dbg_display_help(void)
{
    debugger_printf(
        "McRTOS debugger commands\n"
        "\ta - dump all execution contexts\n"
        "\tc - dump current execution context\n"
        "\td - dump debug message buffer\n"
        "\te - dump exception info\n"
        "\th - display this message\n"
        "\tp - dump CPU controller of CPU core that took the exception\n"
        "\tq - quit McRTOS debugger\n"
        "\tr - reset CPU\n"
        "\tt - dump context switch traces\n");
}


static void
rtos_dbg_dump_exception_info(
    _IN_ const uint32_t *before_exception_stack_p)
{
    debugger_printf(
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

#   if __CORTEX_M >= 0x03
    debugger_printf(
	"Fault status registers (see section 4.3 of DUI0553A_cortex_m4_dgug.pdf):\n"
	"\tSCB CFSR: %#x\n"
	"\tSCB HFSR: %#x\n"
	"\tSCB DFSR: %#x\n"
	"\tSCB MMFAR: %#x\n"
	"\tSCB BFAR: %#x\n"
	"\tSCB AFSR: %#x\n",
	SCB->CFSR,
	SCB->HFSR,
	SCB->DFSR,
	SCB->MMFAR,
	SCB->BFAR,
	SCB->AFSR);

    debugger_printf(
	"Control registers:\n"
	"\tSCB CPACR: %#x\n"
	"\tControl: %#x\n",
	SCB->CPACR,
	__get_CONTROL());
#else
    debugger_printf(
	"Control registers:\n"
	"\tControl: %#x\n",
	__get_CONTROL());
#endif

    debug_dump_captured_registers();

#   ifdef _BRANCH_MICRO_TRACING_
    debug_dump_micro_trace_buffer();
#   endif
}


static void
rtos_dbg_dump_all_execution_contexts(void)
{
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    struct glist_node *context_node_p;

    debugger_printf("Execution Contexts for CPU core %u\n", cpu_id);

    GLIST_FOR_EACH_NODE(
        context_node_p,
        &cpu_controller_p->cpc_execution_contexts_list_anchor)
    {
        struct rtos_execution_context *context_p =
            GLIST_NODE_ENTRY(
                context_node_p, struct rtos_execution_context, ctx_list_node);

        rtos_dbg_dump_execution_context(context_p);
    }
}

static void
rtos_dbg_dump_fdc_msg_buffer(void)
{
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    fdc_info_p->fdc_msg_buffer[RTOS_FDC_MSG_BUFFER_SIZE - 1] = '\0';
    debugger_printf("FDC message buffer for CPU: %u\n\n", cpu_id);

    for (char *s = fdc_info_p->fdc_msg_buffer; *s != '\0'; s ++) {
        uart_putchar_with_polling(g_console_serial_port_p, *s);
    }
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

    debugger_printf("Execution Context: %#p\n", execution_context_p);

    if (execution_context_p->ctx_signature != RTOS_EXECUTION_CONTEXT_SIGNATURE) {


        debugger_printf("*** Error: Invalid signature for context %#p: %#x (%s)\n",
            execution_context_p,
            execution_context_p->ctx_signature,
            signature_to_string(execution_context_p->ctx_signature));
    }

    debugger_printf("\tName: %s\n", execution_context_p->ctx_name_p);
    debugger_printf("\tType: %s\n", context_types[execution_context_p->ctx_context_type]);
    if (execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT) {
        struct rtos_thread *thread_p =
            RTOS_EXECUTION_CONTEXT_GET_THREAD(execution_context_p);

        debugger_printf("\tthread: %#p, state: %#x, state history: %#x\n",
            thread_p, thread_p->thr_state, thread_p->thr_state_history);

    } else if (execution_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT) {
        struct rtos_interrupt *interrupt_p =
            RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(execution_context_p);

        debugger_printf("\tinterrupt: %#p, IRQ channel: %d\n",
            interrupt_p, interrupt_p->int_channel);
    }

    debugger_printf("\tSaved CPU registers:\n");

#if DEFINED_ARM_CLASSIC_ARCH()
    debugger_printf(
        "\t\tr0:   %#x\n"
        "\t\tr1:   %#x\n"
        "\t\tr2:   %#x\n"
        "\t\tr3:   %#x\n"
        "\t\tr4:   %#x\n"
        "\t\tr5:   %#x\n"
        "\t\tr6:   %#x\n"
        "\t\tr7:   %#x\n"
        "\t\tr8:   %#x\n"
        "\t\tr9:   %#x\n",
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R0],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R1],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R2],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R3],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R4],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R5],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R6],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R7],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R8],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R9]);

    debugger_printf(
        "\t\tr10:  %#x\n"
        "\t\tr11:  %#x\n"
        "\t\tr12:  %#x\n"
        "\t\tsp:   %#x\n"
        "\t\tlr:   %#x\n"
        "\t\tpc:   %#x\n"
        "\t\tcpsr: %#x\n",
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R10],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R11],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_R12],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_LR],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_PC],
        execution_context_p->ctx_cpu_saved_registers[CPU_REG_CPSR]);
#elif DEFINED_ARM_CORTEX_M_ARCH()

    const rtos_execution_stack_entry_t *context_stack_p;

    if (execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT) {
        context_stack_p =
            (rtos_execution_stack_entry_t *)execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp;
    } else {
        context_stack_p =
            (rtos_execution_stack_entry_t *)execution_context_p->ctx_cpu_saved_registers.cpu_reg_msp;
    }

    debugger_printf(
        "\t\tr0:        %#x\n"
        "\t\tr1:        %#x\n"
        "\t\tr2:        %#x\n"
        "\t\tr3:        %#x\n"
        "\t\tr4:        %#x\n"
        "\t\tr5:        %#x\n"
        "\t\tr6:        %#x\n"
        "\t\tr7:        %#x\n"
        "\t\tr8:        %#x\n"
        "\t\tr9:        %#x\n",
        context_stack_p[CPU_REG_R0],
        context_stack_p[CPU_REG_R1],
        context_stack_p[CPU_REG_R2],
        context_stack_p[CPU_REG_R3],
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r4,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r5,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r6,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r7,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r8,
        execution_context_p->ctx_cpu_saved_registers.cpu_reg_r9);

    debugger_printf(
        "\t\tr10:       %#x\n"
        "\t\tr11:       %#x\n"
        "\t\tr12:       %#x\n"
        "\t\tsp:        %#x\n"
        "\t\tlr:        %#x\n"
        "\t\tpc:        %#x\n"
        "\t\tpsr:       %#x\n"
        "\t\tmsp:       %#x\n"
        "\t\tpsp:       %#x\n"
        "\t\tcontrol:   %#x\n"
        "\t\tlr_on_exc:	%#x\n",
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
            ctx_cpu_saved_registers.cpu_reg_control,
        execution_context_p->
            ctx_cpu_saved_registers.cpu_reg_lr_on_exc_entry);
#else
    #error "unsupported CPU architecture"
#endif

    if (execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT) {
        if (context_stack_p < execution_context_p->ctx_execution_stack_top_end_p) {
            debugger_printf(
                "*** Error: stack overflow (stack pointer: %#p, stack top limit: %#p)\n",
                context_stack_p, execution_context_p->ctx_execution_stack_top_end_p);
        }

        if (context_stack_p > execution_context_p->ctx_execution_stack_bottom_end_p) {
            debugger_printf(
                "*** Error: stack underflow (stack pointer: %#p, stack bottom limit: %#p)\n",
                context_stack_p, execution_context_p->ctx_execution_stack_bottom_end_p);
        }

	rtos_dbg_dump_stack_trace(execution_context_p);
    }
}

static uint_fast8_t get_pushed_r7_stack_index(cpu_instruction_t push_instruction)
{
    uint_fast8_t reg_list = (push_instruction & PUSH_OPERAND_REG_LIST_MASK);
    uint_fast8_t index = 0;

    /*
     * Check if registers r0 .. r6 are saved on the stack by the push instruction
     */
    for (uint_fast8_t i = 0; i < 7; i ++) {
        if (reg_list & BIT(i)) {
	    index ++;
        }
    }

    return index;
}


/*
 * This function assumes that function prologs have the following code pattern
 * ([] means optional):
 *
 *  push {[r4,] [r5,] [r6,] r7 [, lr]}   ([] means optional)
 *  [sub sp, #imm7]
 *  add	r7, sp, #imm8
 */
static fdc_error_t
find_previous_stack_frame(_IN_ const cpu_instruction_t *program_counter,
			  _INOUT_ const rtos_execution_stack_entry_t **frame_pointer_p,
			  _OUT_ uintptr_t *prev_return_address_p)
{
    fdc_error_t fdc_error;
    cpu_instruction_t instruction;
    uint_fast8_t stack_index;
    const rtos_execution_stack_entry_t *prev_frame_pointer;
    uintptr_t prev_return_address;
    const rtos_execution_stack_entry_t *frame_pointer = *frame_pointer_p;
    uint_fast16_t stop_count = UINT16_MAX;

    if (program_counter == NULL) {
        uintptr_t return_address;

        CAPTURE_ARM_LR_REGISTER(return_address);
        program_counter = (cpu_instruction_t *)GET_CALL_ADDRESS(return_address);
    }

    DBG_ASSERT(((uintptr_t)program_counter & 0x1) == 0 &&
	       VALID_CODE_ADDRESS(program_counter),
	       program_counter, 0);

    DBG_ASSERT_VALID_RAM_POINTER(frame_pointer,
				 sizeof(rtos_execution_stack_entry_t));

    /*
     * Scan instructions backwards looking for one of the 3 instructions in the
     * function prolog pattern:
     */
    while (stop_count != 0) {
	instruction = *program_counter;
	if (IS_ADD_R7_SP_IMMEDITATE(instruction) ||
	    IS_SUB_SP_IMMEDITATE(instruction) ||
	    IS_PUSH_R7(instruction)) {
	    break;
	}

	program_counter--;
	stop_count--;
    }

    if (stop_count == 0) {
        fdc_error = CAPTURE_FDC_ERROR("Could not find previous stack frame",
				      0, 0);
	return fdc_error;
    }

    if (IS_ADD_R7_SP_IMMEDITATE(instruction)) {
	/*
	 * The instruction to be executed is the 'add r7, ...' in the function
	 * prolog.
	 *
	 * NOTE: the decoded operand of the add instruction is
	 *	(instruction & ADD_SP_IMMEDITATE_OPERAND_MASK) << 2
	 *
	 * which is a byte offset that was added to the frame pointer. To
	 * convert it to an stack entry index we need to divide it by 4,
	 * so, the '/ 4' and the '<< 2' cancel each other.
	 */
	frame_pointer -= (instruction & ADD_SP_IMMEDITATE_OPERAND_MASK);
	program_counter --;

	/*
	 * Scan instructions backwards looking for the preceding 'sub sp, ...'
	 * or 'push {...r7}':
	 */
	while (stop_count != 0) {
	    instruction = *program_counter;
	    if (IS_SUB_SP_IMMEDITATE(instruction) ||
		IS_PUSH_R7(instruction)) {
		break;
	    }

	    program_counter--;
	    stop_count--;
	}

	if (stop_count == 0) {
	    fdc_error = CAPTURE_FDC_ERROR("Could not find previous 'sub sp, ...' or "
					  "push {... r7} instruction", 0, 0);
	    return fdc_error;
	}
    }

    if (IS_SUB_SP_IMMEDITATE(instruction)) {
	/*
	 * The preceding instruction to be executed is the 'sub sp, ...' in the
	 * function prolog.
	 *
	 * NOTE: the decoded operand of the sub instruction is
	 *	(instruction & SUB_SP_IMMEDITATE_OPERAND_MASK) << 2
	 *
	 * which is a byte offset that was subtracted from the stack pointer. To
	 * convert it to an stack entry index we need to divide it by 4,
	 * so, the '/ 4' and the '<< 2' cancel each other.
	 */
	frame_pointer += (instruction & SUB_SP_IMMEDITATE_OPERAND_MASK);
	program_counter --;

	/*
	 * Scan instructions backwards looking for the preceding 'push {...r7}'
	 */
	while (stop_count != 0) {
	    instruction = *program_counter;
	    if (IS_PUSH_R7(instruction)) {
		break;
	    }

	    program_counter--;
	    stop_count--;
	}

	if (stop_count == 0) {
	    fdc_error = CAPTURE_FDC_ERROR("Could not find previous push {... r7} instruction",
					  0, 0);
	    return fdc_error;
	}
    }

    /*
     * The preceding instruction is the 'push {...r7}'
     * at the beginning of the prolog:
     */
    DBG_ASSERT(IS_PUSH_R7(instruction), instruction, program_counter);

    stack_index = get_pushed_r7_stack_index(instruction);
    prev_frame_pointer = (rtos_execution_stack_entry_t *)frame_pointer[stack_index];
    FDC_ASSERT(prev_frame_pointer > frame_pointer,
	       prev_frame_pointer, frame_pointer);

    FDC_ASSERT_VALID_RAM_POINTER(prev_frame_pointer,
				 sizeof(rtos_execution_stack_entry_t));

    if (instruction & PUSH_OPERAND_INCLUDES_LR_MASK) {
	prev_return_address = frame_pointer[stack_index + 1];
	FDC_ASSERT((prev_return_address & 0x1) != 0 &&
	       VALID_CODE_ADDRESS(prev_return_address),
	       prev_return_address, frame_pointer);
    } else {
	/*
	 * Inline function with stack frame case
	 */
	prev_return_address = (uintptr_t)(program_counter - 1);
    }

    *frame_pointer_p = prev_frame_pointer;
    *prev_return_address_p = prev_return_address;
    return 0;
}


/*
 * Unwinds a given execution stack
 */
static void
unwind_execution_stack(_IN_ uint_fast8_t num_entries_to_skip,
                       _IN_ uintptr_t top_return_address,
		       _IN_ const rtos_execution_stack_entry_t *frame_pointer,
		       _IN_ const rtos_execution_stack_entry_t *stack_bottom_p,
		       _OUT_ uintptr_t trace_buff[],
		       _INOUT_ uint8_t *num_entries_p)
{
    fdc_error_t fdc_error;
    uint_fast8_t max_num_entries = *num_entries_p;

    FDC_ASSERT(frame_pointer <= stack_bottom_p,
	       frame_pointer, stack_bottom_p);

    FDC_ASSERT_VALID_RAM_POINTER(frame_pointer,
				 sizeof(rtos_execution_stack_entry_t));

    FDC_ASSERT((top_return_address & 0x1) != 0 &&
	       VALID_CODE_ADDRESS(top_return_address),
	       top_return_address, frame_pointer);

    uintptr_t return_address = top_return_address;
    uint_fast8_t i = 0;

    while (i < max_num_entries && frame_pointer < stack_bottom_p) {
	/*
	 * The next stack trace entry is the address of the instruction
	 * preceding the instruction at the return address, unless we
	 * have reached the bottom of the call chain.
	 */
	cpu_instruction_t *instruction_p =
	    (cpu_instruction_t *)GET_CODE_ADDRESS(return_address);

	if (IS_BLX(*(instruction_p - 1))) {
	    instruction_p --;
	} else if (IS_BL32_FIRST_HALF(*(instruction_p - 2)) &&
		   IS_BL32_SECOND_HALF(*(instruction_p - 1))) {
	    instruction_p -= 2;
	}

        if (num_entries_to_skip != 0) {
            num_entries_to_skip --;
        } else {
            trace_buff[i] = (uintptr_t)instruction_p;
            i ++;
        }

	fdc_error = find_previous_stack_frame(instruction_p - 1,
					      &frame_pointer,
					      &return_address);
	if (fdc_error != 0) {
	    break;
	}

	/*
	 * TODO: For the first iteration, check if return address saved
	 * in the previous stack frame is the same as top_return_address
	 * (LR register), to avoid wasting one trace entry with a duplicate.
	 */
    }

    *num_entries_p = i;
}


/*
 * Captures the call trace of a given execution context.
 */
void
get_stack_trace(_IN_ const struct rtos_execution_context *execution_context_p,
                _IN_ uint_fast8_t num_entries_to_skip,
	        _OUT_ uintptr_t trace_buff[],
	        _INOUT_ uint8_t *num_entries_p)
{
    const rtos_execution_stack_entry_t *frame_pointer;
    uintptr_t return_address;
    uintptr_t in_stack_return_address;
    fdc_error_t fdc_error;
    uint8_t num_entries = *num_entries_p;
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];
    struct rtos_execution_context *current_execution_context_p =
	cpu_controller_p->cpc_current_execution_context_p;
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    FDC_ASSERT(num_entries >= 1, num_entries, 0);

    if (execution_context_p == current_execution_context_p &&
        !fdc_info_p->fdc_handling_exception) {
	/*
	 * Latest context state has not been saved:
	 */
        *num_entries_p = 0;
        CAPTURE_ARM_LR_REGISTER(return_address);
	CAPTURE_ARM_FRAME_POINTER_REGISTER(frame_pointer);
	fdc_error = find_previous_stack_frame(NULL,
					      &frame_pointer,
					      &in_stack_return_address);
	if (fdc_error != 0) {
	    return;
	}

        if (in_stack_return_address != return_address) {
            return;
        }

	unwind_execution_stack(num_entries_to_skip,
                               return_address,
			       frame_pointer,
			       execution_context_p->ctx_execution_stack_bottom_end_p,
			       trace_buff,
			       &num_entries);

	DBG_ASSERT(num_entries <= *num_entries_p, num_entries, *num_entries_p);
	*num_entries_p = num_entries;
    } else {
	/*
	 * Latest context state has been saved:
	 */
	const rtos_execution_stack_entry_t *stack_pointer;
        const cpu_instruction_t *program_counter;
	bool old_preemption_state;
	bool restore_preemption_state = false;
        bool one_more_entry = false;

	if (execution_context_p->ctx_context_type != RTOS_THREAD_CONTEXT) {
	    CAPTURE_FDC_ERROR(
		"Cannot get stack trace for non-thread context",
		execution_context_p, 0);

	    return;
	}

	/*
	 * Disable preemption if the calling thread has lower priority than
	 * the thread being examined examined, so we don't get preempted by
	 * that thread, while getting its stack trace:
	 */
	if (current_execution_context_p->ctx_context_type == RTOS_THREAD_CONTEXT) {
	    struct rtos_thread *current_thread_p =
		RTOS_EXECUTION_CONTEXT_GET_THREAD(current_execution_context_p);

	    struct rtos_thread *target_thread_p =
		RTOS_EXECUTION_CONTEXT_GET_THREAD(execution_context_p);

	    if (current_thread_p->thr_current_priority >=
		target_thread_p->thr_current_priority) {
		old_preemption_state = rtos_disable_preemption();
		restore_preemption_state = true;
	    }
	}

#	if DEFINED_ARM_CLASSIC_ARCH()
#	    error "unsupported CPU architecture"

#	elif DEFINED_ARM_CORTEX_M_ARCH()
	    stack_pointer = (rtos_execution_stack_entry_t *)
		execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp;

	    FDC_ASSERT(stack_pointer + CPU_NUM_PRE_SAVED_REGISTERS <=
		       execution_context_p->ctx_execution_stack_bottom_end_p,
		       stack_pointer,
		       execution_context_p->ctx_execution_stack_bottom_end_p);

	    FDC_ASSERT(stack_pointer >=
		       execution_context_p->ctx_execution_stack_top_end_p,
		       stack_pointer,
		       execution_context_p->ctx_execution_stack_top_end_p);

	    program_counter = (cpu_instruction_t *)stack_pointer[CPU_REG_PC];
	    return_address = stack_pointer[CPU_REG_LR];
	    frame_pointer = (rtos_execution_stack_entry_t *)
		execution_context_p->ctx_cpu_saved_registers.cpu_reg_r7;

	    FDC_ASSERT(frame_pointer >= stack_pointer + CPU_NUM_PRE_SAVED_REGISTERS,
		       frame_pointer, stack_pointer);

#	else
#	    error "unsupported CPU architecture"
#	endif

	FDC_ASSERT(frame_pointer <=
		   execution_context_p->ctx_execution_stack_bottom_end_p &&
		   frame_pointer >=
		   execution_context_p->ctx_execution_stack_top_end_p,
		   frame_pointer, execution_context_p);

	FDC_ASSERT(VALID_CODE_ADDRESS(program_counter),
		   program_counter, execution_context_p);

        if (num_entries_to_skip != 0) {
            num_entries_to_skip --;
        } else {
            trace_buff[0] = (uintptr_t)program_counter;
            trace_buff ++;
            num_entries --;
            one_more_entry = true;
        }

	unwind_execution_stack(num_entries_to_skip,
                               return_address,
			       frame_pointer,
			       execution_context_p->ctx_execution_stack_bottom_end_p,
			       trace_buff,
			       &num_entries);

	DBG_ASSERT(num_entries < *num_entries_p, num_entries, *num_entries_p);
        if (one_more_entry) {
            *num_entries_p = num_entries + 1;
        }

	if (restore_preemption_state) {
	    rtos_restore_preemption_state(old_preemption_state);
	}
    }
}


static void
rtos_dbg_dump_memory(
    _IN_ const uint32_t *addr,
    _IN_ uint32_t num_words,
    _IN_ const char *title)
{
    debugger_printf("%s: Memory at %#p (%u words):\n",
        title, addr, num_words);

    for (uint32_t i = 0; i < num_words; i ++) {
        debugger_printf("\t%3u: [%#p]: %#x\n", i, addr + i, addr[i]);
    }
}


static void
rtos_dbg_dump_stack_trace(
    _IN_ const struct rtos_execution_context *execution_context_p)
{
    uintptr_t trace_buff[RTOS_MAX_STACK_TRACE_ENTRIES];
    uint8_t num_trace_entries;

    debugger_printf("Stack trace for %s:\n",
                    execution_context_p->ctx_name_p);

    num_trace_entries = sizeof(trace_buff) / sizeof(trace_buff[0]);
    get_stack_trace(execution_context_p, 0, trace_buff, &num_trace_entries);

    for (uint_fast8_t i = 0; i < num_trace_entries; i ++) {
	debugger_printf("\t%#p\n", trace_buff[i]);
    }
}

static void
rtos_dbg_dump_context_switch_traces(void)
{
#   ifdef _RELIABILITY_CHECKS_
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    debugger_printf(
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
            GET_BIT_FIELD(
                trace_entry,
                FDC_CST_CONTEXT_TYPE_MASK,
                FDC_CST_CONTEXT_TYPE_SHIFT);

        uint32_t trace_context_switch_type =
            GET_BIT_FIELD(
                trace_entry,
                FDC_CST_CONTEXT_SWITCH_TYPE_MASK,
                FDC_CST_CONTEXT_SWITCH_TYPE_SHIFT);

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
#if 0 //???
        case FDC_CST_APPLICATION_THREAD:
            thread_p = &g_McRTOS_p->rts_app_threads[context_id];
            execution_context_p = &thread_p->thr_execution_context;
            break;
#endif

        case FDC_CST_SYSTEM_THREAD:
            thread_p = &cpu_controller_p->cpc_system_threads[context_id];
            execution_context_p = &thread_p->thr_execution_context;
            break;

        case FDC_CST_INTERRUPT:
            interrupt_p = &g_McRTOS_p->rts_interrupts[context_id];
            execution_context_p = &interrupt_p->int_execution_context;
            break;

        default:
            debugger_printf("*** Error: Invalid trace entry (trace context type: %u)\n",
                trace_context_type);
            continue;
        }

        debugger_printf(
            "%3u: %s (context: %#p), context switch type: %#x, "
            "last switched-out reason: %u, priority %u\n",
            i, execution_context_p->ctx_name_p, execution_context_p,
            trace_context_switch_type, last_switched_out_reason, priority);

#       if 0 // ???
        if (thread_p != NULL) {
            debugger_printf(
                "\tthread: %#p, state: %#x, state history: %#x\n",
                thread_p, thread_p->thr_state, thread_p->thr_state_history);
        }

        if (interrupt_p != NULL) {
            debugger_printf(
                "\tinterrupt: %#p\n", interrupt_p);

        }
#       endif
    }
#   endif
}


static void
rtos_dbg_dump_cpu_controller(void)
{
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    debugger_printf(
        "CPU controller for core %u (%#p):\n"
        "\tcpc_ticks_since_boot_count: %u\n"
        "\tcpc_current_execution_context_p: %#p\n"
        "\tcpc_current_thread_p: %#p\n"
        "\tcpc_runnable_thread_priorities bitmap: %#x\n"
        "\tcpc_active_internal_interrupts bitmap: %#x\n"
        "\tcpc_active_external_interrupts bitmap: %#x\n"
        "\tcpc_nested_interrupts_count: %u\n"
        "\tcpc_thread_scheduler_calls: %u\n"
        "\tcpc_longest_time_interrupts_disabled: %u\n",
        SOC_GET_CURRENT_CPU_ID(),
        cpu_controller_p,
        cpu_controller_p->cpc_ticks_since_boot_count,
        cpu_controller_p->cpc_current_execution_context_p,
        cpu_controller_p->cpc_current_thread_p,
        cpu_controller_p->cpc_runnable_thread_priorities,
        cpu_controller_p->cpc_active_internal_interrupts,
        cpu_controller_p->cpc_active_external_interrupts,
        cpu_controller_p->cpc_nested_interrupts_count,
        cpu_controller_p->cpc_thread_scheduler_calls,
        cpu_controller_p->cpc_longest_time_interrupts_disabled);
}


#ifdef _BRANCH_MICRO_TRACING_
static void
debug_dump_micro_trace_buffer(void)
{
    uint64_t *mtb_cursor_p;
    bool mtb_cursor_wrapped;

    micro_trace_get_cursor(&mtb_cursor_p, &mtb_cursor_wrapped);

    if (mtb_cursor_p < __micro_trace_buffer ||
        mtb_cursor_p >= __micro_trace_buffer_end ||
        (uintptr_t)mtb_cursor_p % sizeof(uint64_t) != 0) {
        debugger_printf("*** Error: Invalid mtb_cursor_p: %#p\n", mtb_cursor_p);
    }

    uint32_t next_entry_to_fill = mtb_cursor_p - __micro_trace_buffer;

    uint32_t num_entries;
    uint64_t *entry_p;

    if (mtb_cursor_wrapped) {
        num_entries = MICRO_TRACE_BUFFER_NUM_ENTRIES;
        entry_p = &__micro_trace_buffer[next_entry_to_fill];
    } else {
        num_entries = next_entry_to_fill;
        entry_p = __micro_trace_buffer;
    }

    /*
     * Print control flow trace:
     */

    debugger_printf("Micro Trace Buffer Entries: %u\n", num_entries);

    while (num_entries != 0) {
        const char *prefix = "        ";
        const char *dest_func_name;
        uintptr_t source_addr = (uint32_t)*entry_p;
        uintptr_t dest_addr = (uint32_t)(*entry_p >> 32);

        source_addr &= ~0x1;
        if (VALID_CODE_ADDRESS(source_addr)) {
            cpu_instruction_t cpu_instruction = *(cpu_instruction_t *)source_addr;

            switch (cpu_instruction & THUMB_INSTR_OP_CODE_MASK) {
            case BL_OP_CODE_MASK:
                prefix = "bl   at:";
                break;

            case POP_OP_CODE_MASK:
                prefix = "pop  at:";
                break;

            case BX_OP_CODE_MASK:
                if (cpu_instruction == BX_LR_INSTRUCTION) {
                    prefix = "bxlr at:";
                } else {
                    prefix = "bx   at:";
                }
                break;
            }
        } else {
            source_addr |= 0x1;
        }

        dest_addr &= ~0x1;
        if (VALID_CODE_ADDRESS(dest_addr)) {
            if (dest_addr ==
                    GET_FUNCTION_ADDRESS(cortex_m_hard_fault_exception_handler)) {
                dest_func_name = "cortex_m_hard_fault_exception_handler";
            } else if (dest_addr ==
                        GET_FUNCTION_ADDRESS(capture_assert_failure)) {
                dest_func_name = "capture_assert_failure";
            } else {
                dest_func_name = "";
            }
        } else {
            dest_addr |= 0x1;
            if (dest_addr == CPU_EXC_RETURN_TO_HANDLER_MODE) {
                dest_func_name = "Exception return to handler mode";
            } else if (dest_addr == CPU_EXC_RETURN_TO_THREAD_MODE_USING_MSP) {
                dest_func_name = "Exception return to thread mode using MSP";
            } else if (dest_addr == CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP ||
                       dest_addr == CPU_EXC_RETURN_TO_THREAD_MODE_USING_PSP_FPU) {
                dest_func_name = "Exception return to thread mode using PSP";
            } else {
                dest_func_name = "Unexpected destination address";
            }
        }

        debugger_printf(
            "\t%3u: %s %#p -> %#p %s\n",
            entry_p - __micro_trace_buffer, prefix,
            source_addr, dest_addr, dest_func_name);

        num_entries --;

        entry_p ++;
        if (entry_p == __micro_trace_buffer_end) {
            entry_p = __micro_trace_buffer;
        }
    }
}
#endif /* _BRANCH_MICRO_TRACING_ */


/**
 * Simplified printf that sends debugger output to the console.
 * It only supports the format specifiers supported by embedded_vprintf().
 * It uses uart_putchar_with_polling() for transmitting each character.
 * It does polling on the serial port until all characters are transmitted
 * and no serialization for concurrent calls is provided.
 *
 * @param fmt               format string
 *
 * @param ...               variable arguments
 *
 * @return None
 */
void
debugger_printf(const char *fmt, ...)
{
    va_list va;

    va_start(va, fmt);
    embedded_vprintf(
        (putchar_func_t *)uart_putchar_with_polling,
        (void *)g_console_serial_port_p, fmt, va);

    va_end(va);
}

/*
 * function to be invoked from assembly code
 */
void
debug_dump_r0_to_r3(
    uint32_t r0,
    uint32_t r1,
    uint32_t r2,
    uint32_t r3)
{
    debugger_printf(
        "REGISTERS: r0: %#x, r1: %#x, r2: %#x, r3: %#x\n",
        r0, r1, r2, r3);
}


struct debug_captured_registers_buffer {
    struct rtos_execution_context *cap_execution_context_p;
    uint32_t *cap_location_addr;
    cpu_register_t cap_registers[CPU_NUM_PRE_SAVED_REGISTERS];
};

static struct debug_captured_registers_buffer g_debug_captured_registers_buffers[8];

static uint32_t g_debug_captured_registers_buffer_cursor = 0;

/*
 * function to be invoked from assembly code
 */
void
debug_capture_registers(
    uint32_t r0,
    uint32_t r1,
    uint32_t r2,
    uint32_t r3)
{
    /*
     * Capture ARM LR register on entry
     */
    uint32_t *return_address;
    CAPTURE_ARM_LR_REGISTER(return_address);

    cpu_status_register_t old_primask = __get_PRIMASK();
    __disable_irq();

    struct debug_captured_registers_buffer *captured_registers_buffer_p =
        &g_debug_captured_registers_buffers[g_debug_captured_registers_buffer_cursor];

    captured_registers_buffer_p->cap_execution_context_p =
        RTOS_GET_CURRENT_EXECUTION_CONTEXT();

    captured_registers_buffer_p->cap_location_addr = return_address - 1;
    captured_registers_buffer_p->cap_registers[0] = r0;
    captured_registers_buffer_p->cap_registers[1] = r1;
    captured_registers_buffer_p->cap_registers[2] = r2;
    captured_registers_buffer_p->cap_registers[3] = r3;

    g_debug_captured_registers_buffer_cursor ++;
    if (g_debug_captured_registers_buffer_cursor ==
        ARRAY_SIZE(g_debug_captured_registers_buffers)) {
        g_debug_captured_registers_buffer_cursor = 0;
    }

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
        __enable_irq();
    }
}


static void
debug_dump_captured_registers(void)
{
    debugger_printf(
        "Registers captured buffers (next to fill %u):\n",
        g_debug_captured_registers_buffer_cursor);

    for (uint32_t i = 0; i < ARRAY_SIZE(g_debug_captured_registers_buffers); i ++) {
        struct debug_captured_registers_buffer *captured_registers_buffer_p =
            &g_debug_captured_registers_buffers[i];

        if (captured_registers_buffer_p->cap_location_addr != NULL) {
            debugger_printf(
                "%u: Registers captured at %#p (context: %#p): r0: %#x, r1: %#x, r2: %#x, r3: %#x\n",
                i,
                captured_registers_buffer_p->cap_location_addr,
                captured_registers_buffer_p->cap_execution_context_p,
                captured_registers_buffer_p->cap_registers[0],
                captured_registers_buffer_p->cap_registers[1],
                captured_registers_buffer_p->cap_registers[2],
                captured_registers_buffer_p->cap_registers[3]);
        }
    }
}

