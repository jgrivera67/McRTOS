/**
 * @file McRTOS_command_processor.c
 *
 * McRTOS command-line processor module.
 *
 * Copyright (C) 2014 German Rivera
 *
 * @author German Rivera
 */

#include "McRTOS.h"
#include "McRTOS_internals.h"
#include "failure_data_capture.h"
#include "utils.h"

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"

static void rtos_parse_command_line(const char *cmd_line);
static void McRTOS_display_help(void);
static void McRTOS_display_stats(void);
static void McRTOS_change_console_cpu(cpu_id_t cpu_id);


void
rtos_command_processor(void)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    console_printf("McRTOS-cpu%u> ", cpu_id);
    read_command_line(
	(putchar_func_t *)rtos_console_putchar,
	(getchar_func_t *)rtos_console_getchar,
	NULL,
	g_McRTOS_p->rts_command_line_buffer,
	RTOS_COMMAND_LINE_BUFFER_SIZE);

    rtos_parse_command_line(
	g_McRTOS_p->rts_command_line_buffer);
}


void
rtos_parse_command_line(const char *cmd_line)
{
    int i;
    uint8_t c = cmd_line[0]; // ???

    switch (c) {
    case 'c':
        console_clear();
        break;

    case 'h':
        McRTOS_display_help();
        break;

    case 'i':
        g_McRTOS_p->rts_stop_idle_cpu ^= true;
        break;

    case 'r':
        rtos_reboot();
        /*UNREACHABLE*/
        break;

    case 's':
        McRTOS_display_stats();
        break;

    case CTRL_C:
    case 'D':
	__disable_irq();
	rtos_run_debugger(NULL, NULL);
	__enable_irq();
        break;

    case 'C': //???
	McRTOS_change_console_cpu((SOC_GET_CURRENT_CPU_ID() + 1) % SOC_NUM_CPU_CORES);
	break;

    default:
        for (i = 0; i < g_McRTOS_p->rts_num_app_console_commands; i++) {
            if (c == g_McRTOS_p->rts_app_console_commands_p[i].cmd_name_p[0]) {
                g_McRTOS_p->rts_app_console_commands_p[i].cmd_function_p(cmd_line);
                break;
            }
        }

        if (i == g_McRTOS_p->rts_num_app_console_commands) {
            console_printf("Invalid command: \'%s\' (type h for help)\n", cmd_line);
        }
    }
}


static void
McRTOS_display_help(void)
{
    console_printf(
        "\nMcRTOS commands\n"
        "\tc - clear screen\n"
        "\th - display this message\n"
        "\ti - toggle on/off stopping the CPU in the idle thread\n"
        "\tr - reset CPU\n"
        "\ts - display McRTOS stats (until any key is pressed)\n");

    for (int i = 0; i < g_McRTOS_p->rts_num_app_console_commands; i++) {
        console_printf("\t%s - %s\n",
            g_McRTOS_p->rts_app_console_commands_p[i].cmd_name_p,
            g_McRTOS_p->rts_app_console_commands_p[i].cmd_description_p);
    }

    console_printf("\n");
}


static void
McRTOS_display_stats(void)
{
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

    console_clear();
    console_printf("McRTOS stats for CPU core %u\n\n", cpu_id);

    uint32_t seconds =
        (cpu_controller_p->cpc_ticks_since_boot_count / 1000) * RTOS_MILLISECONDS_PER_TICK;
    uint32_t minutes = seconds / 60;
    uint32_t hours = minutes / 60;

    minutes %= 60;
    seconds %= 60;

    console_printf("Up time: %u ticks (%u hours, %u minutes, %u seconds)\n",
        cpu_controller_p->cpc_ticks_since_boot_count, hours, minutes, seconds);

    console_printf("Longest interrupts disabled time: %u us\n",
         CPU_CLOCK_CYCLES_TO_MICROSECONDS(cpu_controller_p->cpc_longest_time_interrupts_disabled));

    console_printf("FPU lazy context switches: %u\n",
        cpu_controller_p->cpc_fpu_context_switch_count);

    console_printf("Stop CPU in idle thread: %s\n\n",
        g_McRTOS_p->rts_stop_idle_cpu ? "On" : "Off");

    struct glist_node *context_node_p;

    console_printf(
	"Context    Name                           Priority Switched-out Preempted  CPU        CPU under 1ms  Tstamp last  FPU     Switched-out\n"
	"address                                            count        count      usage (ms) usage (cycles) switched-out enabled history     \n"
	"========== ============================== ======== ============ ========== ========== ============== ============ ======= ============\n");

    GLIST_FOR_EACH_NODE(
        context_node_p,
        &cpu_controller_p->cpc_execution_contexts_list_anchor)
    {
        struct rtos_execution_context *context_p =
            GLIST_NODE_ENTRY(
                context_node_p, struct rtos_execution_context, ctx_list_node);

        uint8_t context_type_symbol = '?';
        uint32_t priority = UINT32_MAX;
	char *fpu_enabled = "no";

        switch (context_p->ctx_context_type)
        {
        case RTOS_RESET_CONTEXT:
            context_type_symbol = 'R';
            priority = 0;
            break;

        case RTOS_THREAD_CONTEXT:
            context_type_symbol = 'T';
            priority =  RTOS_EXECUTION_CONTEXT_GET_THREAD(context_p)->thr_current_priority;
	    if (RTOS_EXECUTION_CONTEXT_GET_THREAD(context_p)->thr_fpu_enable_count != 0) {
		    fpu_enabled = "yes";
	    }
            break;

        case RTOS_INTERRUPT_CONTEXT:
            context_type_symbol = 'I';
            priority = RTOS_EXECUTION_CONTEXT_GET_INTERRUPT(context_p)->int_priority;
            break;

        default:
            FDC_ASSERT(false, context_p->ctx_context_type, context_p);
        }

        console_printf(
	    "%#8p %30s %c%7u %12u %10u %10u %14u %12u %7s %#x%x\n",
            context_p,
            context_p->ctx_name_p,
            context_type_symbol,
            priority,
            context_p->ctx_switched_out_counter,
            context_p->ctx_preempted_counter,
	    context_p->ctx_accumulated_cpu_usage_milliseconds,
	    context_p->ctx_accumulated_cpu_usage_cycles,
            context_p->ctx_last_switched_out_time_stamp_in_ticks,
	    fpu_enabled,
            context_p->ctx_switched_out_reason_history,
            context_p->ctx_last_switched_out_reason
        );
    }
}


static void
McRTOS_change_console_cpu(cpu_id_t cpu_id)
{
    g_McRTOS_p->rts_console_input_cpu_id = cpu_id;
    __DSB();
    send_inter_processor_interrupt(cpu_id);
}
