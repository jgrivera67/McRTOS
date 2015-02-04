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
#include "McRTOS_command_processor.h"
#include <networking.h>

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

static void rtos_parse_command_line(const char *cmd_line);
static void McRTOS_display_stats(void);
static void McRTOS_change_console_cpu(cpu_id_t cpu_id);

enum tokens {
    ADDR = FIRST_KEYWORD_TOKEN,
    CLEAR,
    CPU,
    DMESG,
    GATEWAY,
    HELP,
    IP4,
    IP6,
    PING,
    REBOOT,
    SET,
    STACK,
    TOP,
    VERSION,
};

static const char *const keyword_table[] = {
    "addr",
    "clear",
    "cpu",
    "dmesg",
    "gateway",
    "help",
    "ip4",
    "ip6",
    "ping",
    "reboot",
    "set",
    "stack",
    "top",
    "version",
};


static struct tokenizer g_tokenizer;

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
init_tokenizer(struct tokenizer *tokenizer_p,
	       const char *const *keyword_table_p,
	       size_t num_keywords,
	       const char *cmd_line)
{
    tokenizer_p->keyword_table_p = keyword_table_p;
    tokenizer_p->num_keywords = num_keywords;
    tokenizer_p->cmd_line_cursor = cmd_line;
}


static token_t
lookup_keyword_token(struct tokenizer *tokenizer_p)
{
    const char *keyword = tokenizer_p->last_lexical_unit;

    for (unsigned int i = 0; i < tokenizer_p->num_keywords; i ++) {
	const char *keyword_p = tokenizer_p->keyword_table_p[i];
	if (keyword_p != NULL && strcmp(keyword, keyword_p) == 0) {
	    return FIRST_KEYWORD_TOKEN + i;
	}
    }

    return APP_COMMAND;
}


token_t
get_next_token(struct tokenizer *tokenizer_p)
{
    static token_t token = INVALID_TOKEN;
    int c = *tokenizer_p->cmd_line_cursor ++;
    char *lex_cursor = tokenizer_p->last_lexical_unit;
    char *end_lex_cursor = &tokenizer_p->last_lexical_unit[LEXICAL_UNIT_MAX_SIZE - 1];

    /*
     * Skip token separators
     */
    while (is_space(c)) {
	c = *tokenizer_p->cmd_line_cursor ++;
    }

    if (c == '\0') {
	token = END_OF_INPUT;
    } else if (is_alpha(c)) {
	do {
	    if (lex_cursor == end_lex_cursor) {
		console_printf("lexical error at %s\n", tokenizer_p->cmd_line_cursor - 1);
		goto out;
	    }

	    *lex_cursor++ = c;
	    c = *tokenizer_p->cmd_line_cursor ++;
	} while (is_alpha(c) || is_digit(c));

	*lex_cursor = '\0';
	token = lookup_keyword_token(tokenizer_p);
	tokenizer_p->cmd_line_cursor --;
    } else if (c == '0') {
	*lex_cursor++ = c;
	c = *tokenizer_p->cmd_line_cursor ++;
	if (c == 'x') {
	    *lex_cursor++ = c;
	    c = *tokenizer_p->cmd_line_cursor ++;
	    while (is_xdigit(c)) {
		if (lex_cursor == end_lex_cursor) {
		    console_printf("lexical error at %s\n", tokenizer_p->cmd_line_cursor - 1);
		    goto out;
		}

		*lex_cursor++ = c;
		c = *tokenizer_p->cmd_line_cursor ++;
	    }

	    if (lex_cursor == tokenizer_p->last_lexical_unit + 2) {
		console_printf("lexical error at %s\n", tokenizer_p->cmd_line_cursor - 1);
		goto out;
	    }

	    *lex_cursor = '\0';
	    token = HEXADECIMAL_NUMBER;
	    tokenizer_p->cmd_line_cursor --;
	} else if (is_space(c)) {
	    *lex_cursor = '\0';
	    token = DECIMAL_NUMBER; /* 0 */
	} else {
	    console_printf("lexical error at %s\n", tokenizer_p->cmd_line_cursor - 1);
	}
    } else if (is_xdigit(c)) {
	bool non_decimal_digits_count = 0;

	do {
	    if (!is_digit(c)) {
		non_decimal_digits_count ++;
	    }

	    if (lex_cursor == end_lex_cursor) {
		console_printf("lexical error at %s\n", tokenizer_p->cmd_line_cursor - 1);
		goto out;
	    }

	    *lex_cursor++ = c;
	    c = *tokenizer_p->cmd_line_cursor ++;
	} while (is_xdigit(c));

	*lex_cursor = '\0';
	if (non_decimal_digits_count == 0) {
	    token = DECIMAL_NUMBER;
	} else {
	    token = HEXADECIMAL_NUMBER;
	}

	tokenizer_p->cmd_line_cursor --;
    } else if (c == '.') {
	token = DOT_TOKEN;
    } else if (c == ':') {
	token = COLON_TOKEN;
    } else if (c == '/') {
	token = SLASH_TOKEN;
    } else if (c == CTRL_C) {
	token = BREAK_INPUT;
    } else {
	console_printf("lexical error at %s\n", tokenizer_p->cmd_line_cursor - 1);
    }

out:
    return token;
}


static bool
parse_ip4_address(struct ipv4_address *ip_addr_p)
{
    token_t token;
    uint32_t value;

    value = convert_string_to_decimal(g_tokenizer.last_lexical_unit);
    if (value > UINT8_MAX) {
	console_printf("Invalid value \'%u\'\n", value);
	console_printf("line: %d\n", __LINE__);//???
	return false;
    }

    ip_addr_p->bytes[0] = value;
    token = get_next_token(&g_tokenizer);
    if (token != DOT_TOKEN) {
	console_printf("Invalid token \'%s\'\n", g_tokenizer.last_lexical_unit);
	console_printf("line: %d\n", __LINE__);//???
	return false;
    }

    token = get_next_token(&g_tokenizer);
    if (token != DECIMAL_NUMBER) {
	console_printf("Invalid token \'%s\'\n", g_tokenizer.last_lexical_unit);
	console_printf("line: %d\n", __LINE__);//???
	return false;
    }

    value = convert_string_to_decimal(g_tokenizer.last_lexical_unit);
    if (value > UINT8_MAX) {
	console_printf("Invalid value \'%u\'\n", value);
	console_printf("line: %d\n", __LINE__);//???
	return false;
    }

    ip_addr_p->bytes[1] = value;
    token = get_next_token(&g_tokenizer);
    if (token != DOT_TOKEN) {
	console_printf("Invalid token \'%s\'\n", g_tokenizer.last_lexical_unit);
	console_printf("line: %d\n", __LINE__);//???
	return false;
    }

    token = get_next_token(&g_tokenizer);
    if (token != DECIMAL_NUMBER) {
	console_printf("Invalid token \'%s\'\n", g_tokenizer.last_lexical_unit);
	console_printf("line: %d\n", __LINE__);//???
	return false;
    }

    value = convert_string_to_decimal(g_tokenizer.last_lexical_unit);
    if (value > UINT8_MAX) {
	console_printf("Invalid value \'%u\'\n", value);
	console_printf("line: %d\n", __LINE__);//???
	return false;
    }

    ip_addr_p->bytes[2] = value;
    token = get_next_token(&g_tokenizer);
    if (token != DOT_TOKEN) {
	console_printf("Invalid token \'%s\'\n", g_tokenizer.last_lexical_unit);
	console_printf("line: %d\n", __LINE__);//???
	return false;
    }

    token = get_next_token(&g_tokenizer);
    if (token != DECIMAL_NUMBER) {
	console_printf("Invalid token \'%s\'\n", g_tokenizer.last_lexical_unit);
	console_printf("line: %d\n", __LINE__);//???
	return false;
    }

    value = convert_string_to_decimal(g_tokenizer.last_lexical_unit);
    if (value > UINT8_MAX) {
	console_printf("Invalid value \'%u\'\n", value);
	console_printf("line: %d\n", __LINE__);//???
	return false;
    }

    ip_addr_p->bytes[3] = value;

    if (ip_addr_p->value == IPV4_NULL_ADDR ||
        ip_addr_p->value == IPV4_BORADCAST_ADDR) {
	console_printf("Invalid IP address %u.%u.%u.%u\n",
		       ip_addr_p->bytes[0],
		       ip_addr_p->bytes[1],
		       ip_addr_p->bytes[2],
		       ip_addr_p->bytes[3]);
	return false;
    }

    return true;
}


static bool
parse_subnet_prefix(uint8_t *subnet_prefix)
{
    token_t token;
    uint32_t value;

    token = get_next_token(&g_tokenizer);
    if (token != SLASH_TOKEN) {
	console_printf("Missing subnet prefix (expected token \'/\')\n");
	return false;
    }

    token = get_next_token(&g_tokenizer);
    if (token != DECIMAL_NUMBER) {
	goto error;
    }

    value = convert_string_to_decimal(g_tokenizer.last_lexical_unit);
    if (value >= 32) {
	goto error;
    }

    *subnet_prefix = value;
    return true;

error:
    console_printf("Invalid subnet mask \'%s\'\n", g_tokenizer.last_lexical_unit);
    return false;
}


static bool
parse_ip4_address_and_subnet_prefix(void)
{
    struct ipv4_address ip_addr;
    uint8_t subnet_prefix;
    token_t token = get_next_token(&g_tokenizer);

    switch (token) {
    case INVALID_TOKEN:
	return false;

    case END_OF_INPUT:
	console_printf("Missing arguments for \'set ip4 addr\' command\n");
	return false;

    case DECIMAL_NUMBER:
	if (!parse_ip4_address(&ip_addr)) {
	    return false;
	}

	if (!parse_subnet_prefix(&subnet_prefix)) {
	    return false;
	}

	net_set_local_ipv4_address(0, &ip_addr, subnet_prefix);
	break;

    default:
	console_printf("Unexpected token \'%s\'\n", g_tokenizer.last_lexical_unit);
	return false;
    }

    return true;
}


static void
cmd_set_default_ip4_gateway(const struct ipv4_address *ip_addr_p)
{
    console_printf("ERROR: %s not implemented yet\n", __func__);
}


static bool
parse_ip4_gateway(void)
{
    struct ipv4_address ip_addr;
    token_t token = get_next_token(&g_tokenizer);

    switch (token) {
    case INVALID_TOKEN:
	return false;

    case END_OF_INPUT:
	console_printf("Missing arguments for \'set ip4 gateway\' command\n");
	return false;

    case DECIMAL_NUMBER:
	if (!parse_ip4_address(&ip_addr)) {
	    return false;
	}

	cmd_set_default_ip4_gateway(&ip_addr);
	break;

    default:
	console_printf("Unexpected token \'%s\'\n", g_tokenizer.last_lexical_unit);
	return false;
    }

    return true;
}


static bool
parse_set_ip4_command(void)
{
    token_t token = get_next_token(&g_tokenizer);

    switch (token) {
    case INVALID_TOKEN:
	return false;

    case END_OF_INPUT:
	console_printf("Missing arguments for \'set ip4\' command\n");
	return false;

    case ADDR:
	return parse_ip4_address_and_subnet_prefix();

    case GATEWAY:
	return parse_ip4_gateway();

    default:
	console_printf("Unexpected token \'%s\'\n", g_tokenizer.last_lexical_unit);
	return false;
    }
}


static bool
parse_ip6_address(struct ipv6_address *ip_addr_p)
{
    console_printf("ERROR: %s not implemented yet\n", __func__);
    return false;
}


static void
cmd_set_local_ip6_address(const struct ipv6_address *dest_ip_addr_p,
			  uint8_t subnet_prefix)
{
    console_printf("ERROR: %s not implemented yet\n", __func__);
}


static bool
parse_ip6_address_and_subnet_prefix(void)
{
    struct ipv6_address ip_addr;
    uint8_t subnet_prefix;
    token_t token = get_next_token(&g_tokenizer);

    switch (token) {
    case INVALID_TOKEN:
	return false;

    case END_OF_INPUT:
	console_printf("Missing arguments for \'set ip6 addr\' command\n");
	return false;

    case DECIMAL_NUMBER:
    case HEXADECIMAL_NUMBER:
	/*
	 * NOTE: here we interpret string of decimal digits as a hexadecimal
	 * number
	 */
	if (!parse_ip6_address(&ip_addr)) {
	    return false;
	}

	if (!parse_subnet_prefix(&subnet_prefix)) {
	    return false;
	}

	cmd_set_local_ip6_address(&ip_addr, subnet_prefix);
	break;

    default:
	console_printf("Unexpected token \'%s\'\n", g_tokenizer.last_lexical_unit);
	return false;
    }

    return true;
}


static bool
parse_set_ip6_command(void)
{
    token_t token = get_next_token(&g_tokenizer);

    switch (token) {
    case INVALID_TOKEN:
	return false;

    case END_OF_INPUT:
	console_printf("Missing arguments for \'set ip6\' command\n");
	return false;

    case ADDR:
	return parse_ip6_address_and_subnet_prefix();

    default:
	console_printf("Unexpected token \'%s\'\n", g_tokenizer.last_lexical_unit);
	return false;
    }
}


static bool
parse_set(void)
{
    token_t token = get_next_token(&g_tokenizer);

    switch (token) {
    case INVALID_TOKEN:
	return false;

    case END_OF_INPUT:
    case HELP:
	console_printf(
	    "\tset ip4 addr <IP addr>/<subnet prefix size>\n"
	    "\tset ip6 addr <IP addr>/<subnet prefix size>\n"
	    "\tset ip4 gateway <IP addr>\n"
	    "\n");

	return true;

    case IP4:
	return parse_set_ip4_command();

    case IP6:
	return parse_set_ip6_command();

    default:
	console_printf("Unexpected token \'%s\'\n", g_tokenizer.last_lexical_unit);
	return false;
    }
}

static void
cmd_ping_remote_ip4_addr(const struct ipv4_address *dest_ip_addr_p)
{
    fdc_error_t fdc_error;
    struct ipv4_address remote_ip_addr;
    static uint16_t req_seq_num = 0;
    uint16_t reply_seq_num;
    uint16_t reply_identifier;
    uint16_t identifier = (uintptr_t)rtos_thread_self();

    for (int i = 0; i < 8; i ++) {
	fdc_error = net_send_ipv4_ping_request(dest_ip_addr_p, identifier,
					       req_seq_num);
	if (fdc_error != 0) {
	    console_printf("net_send_ipv4_ping_request() failed with error %#x\n",
		           fdc_error);
	    return;
	}

	fdc_error = net_receive_ipv4_ping_reply(3000,
						&remote_ip_addr,
						&reply_identifier,
						&reply_seq_num);

	if (fdc_error != 0) {
	    CONSOLE_POS_PRINTF(34,1, "Ping %d timedout for %u.%u.%u.%u\n",
			       req_seq_num,
			       dest_ip_addr_p->bytes[0],
			       dest_ip_addr_p->bytes[1],
			       dest_ip_addr_p->bytes[2],
			       dest_ip_addr_p->bytes[3]);
	    return;
	}

	FDC_ASSERT(remote_ip_addr.value == dest_ip_addr_p->value,
		   remote_ip_addr.value, dest_ip_addr_p->value);
	FDC_ASSERT(reply_identifier == identifier,
		   reply_identifier, identifier);
	FDC_ASSERT(reply_seq_num == req_seq_num,
		   reply_seq_num, req_seq_num);

	CONSOLE_POS_PRINTF(34,1, "Ping %d for %u.%u.%u.%u\n",
			   reply_seq_num,
			   remote_ip_addr.bytes[0],
			   remote_ip_addr.bytes[1],
			   remote_ip_addr.bytes[2],
			   remote_ip_addr.bytes[3]);

	req_seq_num ++;
    }
}


static bool
parse_ping(void)
{
    struct ipv4_address ip_addr;
    token_t token = get_next_token(&g_tokenizer);

    switch (token) {
    case INVALID_TOKEN:
	return false;

    case END_OF_INPUT:
    case HELP:
	console_printf("\tping <IPv4 address>\n\n");
	break;

    case DECIMAL_NUMBER:
	if (!parse_ip4_address(&ip_addr)) {
	    return false;
	}

	cmd_ping_remote_ip4_addr(&ip_addr);
	break;

    default:
	console_printf("Unexpected token (%d) \'%s\'\n", token, g_tokenizer.last_lexical_unit);
	return false;
    }

    return true;
}


static void
cmd_display_help(void)
{
    console_printf(
        "\nMcRTOS commands\n"
        "\tclear - clear screen\n"
        "\tcpu - Switch to the given CPU\n"
        "\tdmesg - Display message log\n"
        "\thelp - display this message\n"
	"\tping - send ping to a given IP address\n"
        "\treset - reset CPU\n"
        "\tset - Set config option\n"
        "\tstack - display stack trace of a given thread/ISR\n"
        "\ttop - display McRTOS stats\n"
        "\tversion - display McRTOS version\n");

    for (int i = 0; i < g_McRTOS_p->rts_num_app_console_commands; i++) {
        console_printf("\t%s - %s\n",
            g_McRTOS_p->rts_app_console_commands_p[i].cmd_name_p,
            g_McRTOS_p->rts_app_console_commands_p[i].cmd_description_p);
    }

    console_printf("\n");
}


static bool parse_help(void)
{
    token_t token = get_next_token(&g_tokenizer);

    switch (token) {
    case INVALID_TOKEN:
	return false;

    case END_OF_INPUT:
	cmd_display_help();
	break;

    default:
	console_printf("Unexpected token (%d) \'%s\'\n", token, g_tokenizer.last_lexical_unit);
	return false;
    }

    return true;
}


static void
cmd_display_msg_log(void)
{
    console_printf("ERROR: %s not implemented yet\n", __func__);
}


static bool parse_dmesg(void)
{
    token_t token = get_next_token(&g_tokenizer);

    switch (token) {
    case INVALID_TOKEN:
	return false;

    case END_OF_INPUT:
	cmd_display_msg_log();
	break;

    default:
	console_printf("Unexpected token (%d) \'%s\'\n", token, g_tokenizer.last_lexical_unit);
	return false;
    }

    return true;
}


static void
cmd_display_stack_trace(uintptr_t mem_addr)
{
    console_printf("ERROR: %s not implemented yet\n", __func__);
}


static bool
parse_mem_address(uintptr_t *mem_address_p)
{
    console_printf("ERROR: %s not implemented yet\n", __func__);
    return false;
}


static bool parse_stack(void)
{
    token_t token = get_next_token(&g_tokenizer);
    uintptr_t mem_addr;

    switch (token) {
    case INVALID_TOKEN:
	return false;

    case END_OF_INPUT:
    case HELP:
	console_printf("\tstack <execution context address>\n\n");
	break;

    case HEXADECIMAL_NUMBER:
	if (!parse_mem_address(&mem_addr)) {
	    return false;
	}

	cmd_display_stack_trace(mem_addr);
	break;

    default:
	console_printf("Unexpected token (%d) \'%s\'\n", token, g_tokenizer.last_lexical_unit);
	return false;
    }

    return true;
}

void
rtos_parse_command_line(const char *cmd_line)
{
    int i;

    init_tokenizer(&g_tokenizer,
		   keyword_table,
		   ARRAY_SIZE(keyword_table),
		   cmd_line);

    for ( ; ; ) {
	token_t token = get_next_token(&g_tokenizer);

	if (token == INVALID_TOKEN || token == END_OF_INPUT) {
	    return;
	}

	switch (token) {
	case CLEAR:
	    console_clear();
	    break;

	case HELP:
	    parse_help();
	    break;

	case REBOOT:
	    rtos_reboot();
	    /*UNREACHABLE*/
	    break;

	case TOP:
	    McRTOS_display_stats();
	    break;

	case BREAK_INPUT:
	    __disable_irq();
	    rtos_run_debugger(NULL, NULL);
	    __enable_irq();
	    break;

	case CPU:
	    McRTOS_change_console_cpu((SOC_GET_CURRENT_CPU_ID() + 1) % SOC_NUM_CPU_CORES);
	    break;

	case SET:
	    if (!parse_set()) {
		return;
	    }

	    break;

	case VERSION:
	    console_printf("%s\n%s\n", g_McRTOS_version, g_McRTOS_build_timestamp);
	    break;

	case PING:
	    if (!parse_ping()) {
		return;
	    }

	    break;

	case APP_COMMAND:
	    for (i = 0; i < g_McRTOS_p->rts_num_app_console_commands; i++) {
		if (strcmp(g_tokenizer.last_lexical_unit,
			   g_McRTOS_p->rts_app_console_commands_p[i].cmd_name_p) == 0) {
		    g_McRTOS_p->rts_app_console_commands_p[i].cmd_function_p();
		    break;
		}
	    }

	    if (i == g_McRTOS_p->rts_num_app_console_commands) {
		console_printf("Invalid command: \'%s\' (type help)\n", cmd_line);
		return;
	    }

	    break;

	default:
	    console_printf("Invalid command: %s\n", g_tokenizer.last_lexical_unit);
	}
    }
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
