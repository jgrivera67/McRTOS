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
#ifdef _NETWORKING_
#include <networking.h>
#endif

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"

static void rtos_parse_command_line(const char *cmd_line);
static void McRTOS_display_stats(void);
static void McRTOS_change_console_cpu(cpu_id_t cpu_id);

static const char *const keyword_table[] = {
    "addr",
    "clear",
    "cpu",
    "dmesg",
    "get",
    "help",
#ifdef _NETWORKING_
    "gateway",
    "ip4",
    "ip6",
    "ping",
    "ping6",
#endif
    "reset",
    "set",
    "stack",
    "top",
    "version",
};

struct command_processor g_command_processor = {{
    .keyword_table_p = keyword_table,
    .num_keywords = ARRAY_SIZE(keyword_table),
}};


void
rtos_command_processor(void)
{
    struct mpu_region_range old_comp_region;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    rtos_thread_set_comp_region(&g_command_processor,
                                sizeof g_command_processor,
                                0,
                                &old_comp_region);

    console_printf("McRTOS-cpu%u> ", cpu_id);

    read_command_line(
	(putchar_func_t *)rtos_console_putchar,
	(getchar_func_t *)rtos_console_getchar,
	NULL,
	g_command_processor.command_line_buffer,
	RTOS_COMMAND_LINE_BUFFER_SIZE);

    rtos_parse_command_line(
	g_command_processor.command_line_buffer);

    rtos_thread_restore_comp_region(&old_comp_region);
}


void
init_command_processor(
    uint8_t num_app_console_commands,
    const struct rtos_console_command *app_console_commands_p)
{
    g_command_processor.num_app_console_commands = num_app_console_commands;
    g_command_processor.app_console_commands_p = app_console_commands_p;
}


static token_t
lookup_keyword_token(struct command_processor *command_processor_p)
{
    const char *keyword = command_processor_p->last_lexical_unit;

    for (unsigned int i = 0; i < command_processor_p->num_keywords; i ++) {
	const char *keyword_p = command_processor_p->keyword_table_p[i];
	if (keyword_p != NULL && strcmp(keyword, keyword_p) == 0) {
	    return FIRST_KEYWORD_TOKEN + i;
	}
    }

    /*
     * Check if token is a hexadecimal numeric string:
     */
    for (unsigned int i = 0; keyword[i] != '\0'; i ++) {
	if (!is_xdigit(keyword[i])) {
		return APP_COMMAND;
	}
    }

    return HEXADECIMAL_NUMBER;
}


token_t
get_next_token(struct command_processor *command_processor_p)
{
    token_t token = INVALID_TOKEN;
    int c = *command_processor_p->cmd_line_cursor ++;
    char *lex_cursor = command_processor_p->last_lexical_unit;
    char *end_lex_cursor = &command_processor_p->last_lexical_unit[LEXICAL_UNIT_MAX_SIZE - 1];

    command_processor_p->last_lexical_unit[0] = '\0';

    /*
     * Skip token separators
     */
    while (is_space(c)) {
	c = *command_processor_p->cmd_line_cursor ++;
    }

restart_parsing:
    if (c == '\0') {
	token = END_OF_INPUT;
    } else if (is_alpha(c)) {
	do {
	    if (lex_cursor == end_lex_cursor) {
		console_printf("lexical error at %s\n", command_processor_p->cmd_line_cursor - 1);
		goto out;
	    }

	    *lex_cursor++ = c;
	    c = *command_processor_p->cmd_line_cursor ++;
	} while (is_alpha(c) || is_digit(c) || c == '-');

	*lex_cursor = '\0';
	token = lookup_keyword_token(command_processor_p);
	command_processor_p->cmd_line_cursor --;
    } else if (c == '0') {
	*lex_cursor++ = c;
	c = *command_processor_p->cmd_line_cursor ++;
	if (c == 'x') {
	    *lex_cursor++ = c;
	    c = *command_processor_p->cmd_line_cursor ++;
	    while (is_xdigit(c)) {
		if (lex_cursor == end_lex_cursor) {
		    console_printf("lexical error at %s\n", command_processor_p->cmd_line_cursor - 1);
		    goto out;
		}

		*lex_cursor++ = c;
		c = *command_processor_p->cmd_line_cursor ++;
	    }

	    if (lex_cursor == command_processor_p->last_lexical_unit + 2) {
		console_printf("lexical error at %s\n", command_processor_p->cmd_line_cursor - 1);
		goto out;
	    }

	    *lex_cursor = '\0';
	    token = HEXADECIMAL_NUMBER;
	    command_processor_p->cmd_line_cursor --;
	} else if (is_xdigit(c)) {
	    goto restart_parsing;
	} else {
	    *lex_cursor = '\0';
	    token = DECIMAL_NUMBER; /* 0 */
	    command_processor_p->cmd_line_cursor --;
	}
    } else if (is_xdigit(c)) {
	int non_decimal_digits_count = 0;

	do {
	    if (!is_digit(c)) {
		non_decimal_digits_count ++;
	    }

	    if (lex_cursor == end_lex_cursor) {
		console_printf("lexical error at %s\n", command_processor_p->cmd_line_cursor - 1);
		goto out;
	    }

	    *lex_cursor++ = c;
	    c = *command_processor_p->cmd_line_cursor ++;
	} while (is_xdigit(c));

	*lex_cursor = '\0';
	if (non_decimal_digits_count == 0) {
	    token = DECIMAL_NUMBER;
	} else {
	    token = HEXADECIMAL_NUMBER;
	}

	command_processor_p->cmd_line_cursor --;
    } else if (c == '.') {
	token = DOT_TOKEN;
    } else if (c == ':') {
	token = COLON_TOKEN;
    } else if (c == '/') {
	token = SLASH_TOKEN;
    } else if (c == CTRL_C) {
	token = BREAK_INPUT;
    } else {
	console_printf("lexical error at %s\n", command_processor_p->cmd_line_cursor - 1);
    }

out:
    return token;
}


#ifdef _NETWORKING_
bool
parse_ip4_address(struct ipv4_address *ip_addr_p)
{
    token_t token;
    uint32_t value;

    value = convert_string_to_decimal(g_command_processor.last_lexical_unit);
    if (value > UINT8_MAX) {
	console_printf("IPv4 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ip_addr_p->bytes[0] = value;
    token = get_next_token(&g_command_processor);
    if (token != DOT_TOKEN) {
	console_printf("IPv4 address parsing error:  Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != DECIMAL_NUMBER) {
	console_printf("IPv4 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    value = convert_string_to_decimal(g_command_processor.last_lexical_unit);
    if (value > UINT8_MAX) {
	console_printf("IPv4 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ip_addr_p->bytes[1] = value;
    token = get_next_token(&g_command_processor);
    if (token != DOT_TOKEN) {
	console_printf("IPv4 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);

	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != DECIMAL_NUMBER) {
	console_printf("IPv4 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    value = convert_string_to_decimal(g_command_processor.last_lexical_unit);
    if (value > UINT8_MAX) {
	console_printf("IPv4 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ip_addr_p->bytes[2] = value;
    token = get_next_token(&g_command_processor);
    if (token != DOT_TOKEN) {
	console_printf("IPv4 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != DECIMAL_NUMBER) {
	console_printf("IPv4 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    value = convert_string_to_decimal(g_command_processor.last_lexical_unit);
    if (value > UINT8_MAX) {
	console_printf("IPv4 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ip_addr_p->bytes[3] = value;

    if (ip_addr_p->value == IPV4_NULL_ADDR ||
        ip_addr_p->value == IPV4_BROADCAST_ADDR) {
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

    token = get_next_token(&g_command_processor);
    if (token != SLASH_TOKEN) {
	console_printf("Missing subnet prefix (expected token \'/\')\n");
	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != DECIMAL_NUMBER) {
	goto error;
    }

    value = convert_string_to_decimal(g_command_processor.last_lexical_unit);
    if (value >= 32) {
	goto error;
    }

    *subnet_prefix = value;
    return true;

error:
    console_printf("Invalid subnet mask \'%s\'\n", g_command_processor.last_lexical_unit);
    return false;
}


static void
parse_set_ip4_address_and_subnet_prefix(void)
{
    struct ipv4_address ip_addr;
    uint8_t subnet_prefix;
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;

    case DECIMAL_NUMBER:
	if (!parse_ip4_address(&ip_addr)) {
	    break;
	}

	if (!parse_subnet_prefix(&subnet_prefix)) {
	    break;
	}

	net_set_local_ipv4_address(&ip_addr, subnet_prefix);
	break;

    case END_OF_INPUT:
    default:
	console_printf("\'set ip4 addr\' command syntax error (type \'set help\')\n");
    }
}


static void
cmd_set_default_ip4_gateway(const struct ipv4_address *ip_addr_p)
{
    console_printf("ERROR: %s not implemented yet\n", __func__);
}


static void
parse_set_ip4_gateway(void)
{
    struct ipv4_address ip_addr;
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;


    case DECIMAL_NUMBER:
	if (!parse_ip4_address(&ip_addr)) {
            break;
        }

        cmd_set_default_ip4_gateway(&ip_addr);
	break;

    case END_OF_INPUT:
    default:
	console_printf("\'set ip4 gateway\' command syntax error (type \'set help\')\n");
    }
}


static void
parse_set_ip4_command(void)
{
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;

    case ADDR:
	parse_set_ip4_address_and_subnet_prefix();
        break;

    case GATEWAY:
	parse_set_ip4_gateway();
        break;

    case END_OF_INPUT:
    default:
	console_printf("\'set ip4\' command syntax error (type \'set help\')\n");
    }
}


static void
cmd_get_local_ip4_addr(void)
{
    struct ipv4_address ip_addr;

    net_get_local_ipv4_address(&ip_addr);

    console_printf("Local IPv4 address: %u.%u.%u.%u\n",
                  ip_addr.bytes[0],
                  ip_addr.bytes[1],
                  ip_addr.bytes[2],
                  ip_addr.bytes[3]);
}


static void
parse_get_ip4_command(void)
{
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;

    case ADDR:
        cmd_get_local_ip4_addr();
        break;

    case END_OF_INPUT:
    default:
	console_printf("\'get ip4\' command syntax error (type \'get help\')\n");
    }
}


bool
parse_ip6_address(struct ipv6_address *ipv6_addr_p)
{
    token_t token;
    uint32_t value;

    value = convert_string_to_hexadecimal(g_command_processor.last_lexical_unit);
    if (value > UINT16_MAX) {
	console_printf("IPv6 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ipv6_addr_p->hwords[0] = hton16(value);
    token = get_next_token(&g_command_processor);
    if (token != COLON_TOKEN) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != HEXADECIMAL_NUMBER && token != DECIMAL_NUMBER) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    value = convert_string_to_hexadecimal(g_command_processor.last_lexical_unit);
    if (value > UINT16_MAX) {
	console_printf("IPv6 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ipv6_addr_p->hwords[1] = hton16(value);
    token = get_next_token(&g_command_processor);
    if (token != COLON_TOKEN) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);

	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != HEXADECIMAL_NUMBER && token != DECIMAL_NUMBER) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    value = convert_string_to_hexadecimal(g_command_processor.last_lexical_unit);
    if (value > UINT16_MAX) {
	console_printf("IPv6 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ipv6_addr_p->hwords[2] = hton16(value);
    token = get_next_token(&g_command_processor);
    if (token != COLON_TOKEN) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != HEXADECIMAL_NUMBER && token != DECIMAL_NUMBER) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    value = convert_string_to_hexadecimal(g_command_processor.last_lexical_unit);
    if (value > UINT16_MAX) {
	console_printf("IPv6 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ipv6_addr_p->hwords[3] = hton16(value);
    token = get_next_token(&g_command_processor);
    if (token != COLON_TOKEN) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != HEXADECIMAL_NUMBER && token != DECIMAL_NUMBER) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    value = convert_string_to_hexadecimal(g_command_processor.last_lexical_unit);
    if (value > UINT16_MAX) {
	console_printf("IPv6 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ipv6_addr_p->hwords[4] = hton16(value);
    token = get_next_token(&g_command_processor);
    if (token != COLON_TOKEN) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != HEXADECIMAL_NUMBER && token != DECIMAL_NUMBER) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    value = convert_string_to_hexadecimal(g_command_processor.last_lexical_unit);
    if (value > UINT16_MAX) {
	console_printf("IPv6 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ipv6_addr_p->hwords[5] = hton16(value);
    token = get_next_token(&g_command_processor);
    if (token != COLON_TOKEN) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != HEXADECIMAL_NUMBER && token != DECIMAL_NUMBER) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    value = convert_string_to_hexadecimal(g_command_processor.last_lexical_unit);
    if (value > UINT16_MAX) {
	console_printf("IPv6 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ipv6_addr_p->hwords[6] = hton16(value);
    token = get_next_token(&g_command_processor);
    if (token != COLON_TOKEN) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    token = get_next_token(&g_command_processor);
    if (token != HEXADECIMAL_NUMBER && token != DECIMAL_NUMBER) {
	console_printf("IPv6 address parsing error: Invalid token \'%s\'\n",
                       g_command_processor.last_lexical_unit);
	return false;
    }

    value = convert_string_to_hexadecimal(g_command_processor.last_lexical_unit);
    if (value > UINT16_MAX) {
	console_printf("IPv6 address parsing error: Invalid value \'%u\'\n", value);
	return false;
    }

    ipv6_addr_p->hwords[7] = hton16(value);

    if (IPV6_ADDRESSES_EQUAL(ipv6_addr_p, &ipv6_unspecified_address)) {
	console_printf("Invalid IPv6 address %x:%x:%x:%x:%x:%x:%x:%x",
                       ntoh16(ipv6_addr_p->hwords[0]),
                       ntoh16(ipv6_addr_p->hwords[1]),
                       ntoh16(ipv6_addr_p->hwords[2]),
                       ntoh16(ipv6_addr_p->hwords[3]),
                       ntoh16(ipv6_addr_p->hwords[4]),
                       ntoh16(ipv6_addr_p->hwords[5]),
                       ntoh16(ipv6_addr_p->hwords[6]),
                       ntoh16(ipv6_addr_p->hwords[7]));
	return false;
    }

    return true;
}


static void
cmd_set_local_ip6_address(const struct ipv6_address *dest_ip_addr_p,
			  uint8_t subnet_prefix)
{
    console_printf("ERROR: %s not implemented yet\n", __func__);
}


static void
parse_set_ip6_address_and_subnet_prefix(void)
{
    struct ipv6_address ip_addr;
    uint8_t subnet_prefix;
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;

    case HEXADECIMAL_NUMBER:
    case DECIMAL_NUMBER:
	/*
	 * NOTE: here we interpret string of decimal digits as a hexadecimal
	 * number
	 */
	if (!parse_ip6_address(&ip_addr)) {
	    break;
	}

	if (!parse_subnet_prefix(&subnet_prefix)) {
	    break;
	}

	cmd_set_local_ip6_address(&ip_addr, subnet_prefix);
	break;

    case END_OF_INPUT:
    default:
	console_printf("\'set ip6 addr\' command syntax error (type \'set help\')\n");
    }
}


static void
parse_set_ip6_command(void)
{
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;

    case ADDR:
	parse_set_ip6_address_and_subnet_prefix();
        break;

    case END_OF_INPUT:
    default:
	console_printf("\'set ip6\' command syntax error (type \'set help\')\n");
    }
}


static void
cmd_get_local_ip6_addr(void)
{
    struct ipv6_address ip_addr;

    net_get_local_ipv6_address(&ip_addr);

    console_printf("Local IPv6 address: %x:%x:%x:%x:%x:%x:%x:%x\n",
                   ntoh16(ip_addr.hwords[0]),
                   ntoh16(ip_addr.hwords[1]),
                   ntoh16(ip_addr.hwords[2]),
                   ntoh16(ip_addr.hwords[3]),
                   ntoh16(ip_addr.hwords[4]),
                   ntoh16(ip_addr.hwords[5]),
                   ntoh16(ip_addr.hwords[6]),
                   ntoh16(ip_addr.hwords[7]));
}


static void
parse_get_ip6_command(void)
{
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;

    case ADDR:
        cmd_get_local_ip6_addr();
        break;

    case END_OF_INPUT:
    default:
	console_printf("\'get ip6\' command syntax error (type \'get help\')\n");
    }
}


static void
cmd_ping_remote_ip4_addr(const struct ipv4_address *dest_ip_addr_p)
{
    fdc_error_t fdc_error;
    struct ipv4_address remote_ip_addr;
    uint16_t req_seq_num = 0;
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
	    console_printf("Ping %d for %u.%u.%u.%u timed-out\n",
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

	console_printf("Ping %d replied by %u.%u.%u.%u\n",
		       reply_seq_num,
		       remote_ip_addr.bytes[0],
		       remote_ip_addr.bytes[1],
		       remote_ip_addr.bytes[2],
		       remote_ip_addr.bytes[3]);

	req_seq_num ++;
        rtos_thread_delay(500);
    }
}


static void
parse_ping(void)
{
    struct ipv4_address ip_addr;
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;

    case HELP:
	console_printf("\tping <IPv4 address>\n\n");
	break;

    case DECIMAL_NUMBER:
	if (!parse_ip4_address(&ip_addr)) {
	    break;
	}

	cmd_ping_remote_ip4_addr(&ip_addr);
	break;

    case END_OF_INPUT:
    default:
	console_printf("\'ping\' command syntax error (type \'ping help\')\n");
    }
}


static void
cmd_ping_remote_ip6_addr(const struct ipv6_address *dest_ipv6_addr_p)
{
    fdc_error_t fdc_error;
    struct ipv6_address remote_ipv6_addr;
    uint16_t req_seq_num = 0;
    uint16_t reply_seq_num;
    uint16_t reply_identifier;
    uint16_t identifier = (uintptr_t)rtos_thread_self();

    for (int i = 0; i < 8; i ++) {
	fdc_error = net_send_ipv6_ping_request(dest_ipv6_addr_p, identifier,
					       req_seq_num);
	if (fdc_error != 0) {
	    console_printf("net_send_ipv6_ping_request() failed with error %#x\n",
		           fdc_error);
	    return;
	}

	fdc_error = net_receive_ipv6_ping_reply(3000,
						&remote_ipv6_addr,
						&reply_identifier,
						&reply_seq_num);

	if (fdc_error != 0) {
	    console_printf("Ping IPv6 %d for %x:%x:%x:%x:%x:%x:%x:%x timed-out\n",
			   req_seq_num,
			   ntoh16(dest_ipv6_addr_p->hwords[0]),
			   ntoh16(dest_ipv6_addr_p->hwords[1]),
			   ntoh16(dest_ipv6_addr_p->hwords[2]),
			   ntoh16(dest_ipv6_addr_p->hwords[3]),
			   ntoh16(dest_ipv6_addr_p->hwords[4]),
			   ntoh16(dest_ipv6_addr_p->hwords[5]),
			   ntoh16(dest_ipv6_addr_p->hwords[6]),
			   ntoh16(dest_ipv6_addr_p->hwords[7]));
	    return;
	}

//???
console_printf("*** remote addr %x:%x:%x:%x:%x:%x:%x:%x\n",
		       ntoh16(remote_ipv6_addr.hwords[0]),
		       ntoh16(remote_ipv6_addr.hwords[1]),
		       ntoh16(remote_ipv6_addr.hwords[2]),
		       ntoh16(remote_ipv6_addr.hwords[3]),
		       ntoh16(remote_ipv6_addr.hwords[4]),
		       ntoh16(remote_ipv6_addr.hwords[5]),
		       ntoh16(remote_ipv6_addr.hwords[6]),
		       ntoh16(remote_ipv6_addr.hwords[7]));

console_printf("*** dest_ipv6_addr %x:%x:%x:%x:%x:%x:%x:%x\n",
		       ntoh16(dest_ipv6_addr_p->hwords[0]),
		       ntoh16(dest_ipv6_addr_p->hwords[1]),
		       ntoh16(dest_ipv6_addr_p->hwords[2]),
		       ntoh16(dest_ipv6_addr_p->hwords[3]),
		       ntoh16(dest_ipv6_addr_p->hwords[4]),
		       ntoh16(dest_ipv6_addr_p->hwords[5]),
		       ntoh16(dest_ipv6_addr_p->hwords[6]),
		       ntoh16(dest_ipv6_addr_p->hwords[7]));

//???
	FDC_ASSERT(IPV6_ADDRESSES_EQUAL(&remote_ipv6_addr, dest_ipv6_addr_p),
		   remote_ipv6_addr.words[2], remote_ipv6_addr.words[3]);
	FDC_ASSERT(reply_identifier == identifier,
		   reply_identifier, identifier);
	FDC_ASSERT(reply_seq_num == req_seq_num,
		   reply_seq_num, req_seq_num);

	console_printf("Ping IPv6 %d replied by %x:%x:%x:%x:%x:%x:%x:%x\n",
		       reply_seq_num,
		       ntoh16(remote_ipv6_addr.hwords[0]),
		       ntoh16(remote_ipv6_addr.hwords[1]),
		       ntoh16(remote_ipv6_addr.hwords[2]),
		       ntoh16(remote_ipv6_addr.hwords[3]),
		       ntoh16(remote_ipv6_addr.hwords[4]),
		       ntoh16(remote_ipv6_addr.hwords[5]),
		       ntoh16(remote_ipv6_addr.hwords[6]),
		       ntoh16(remote_ipv6_addr.hwords[7]));

	req_seq_num ++;
        rtos_thread_delay(500);
    }
}


static void
parse_ping6(void)
{
    struct ipv6_address ipv6_addr;
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;

    case HELP:
	console_printf("\tping6 <IPv6 address>\n\n");
	break;

    case HEXADECIMAL_NUMBER:
    case DECIMAL_NUMBER:
	/*
	 * NOTE: here we interpret string of decimal digits as a hexadecimal
	 * number
	 */
	if (!parse_ip6_address(&ipv6_addr)) {
	    break;
	}

	cmd_ping_remote_ip6_addr(&ipv6_addr);
	break;

    case END_OF_INPUT:
    default:
	console_printf("\'ping6\' command syntax error (type \'ping6 help\')\n");
    }
}
#endif /* _NETWORKING_ */


static void
parse_set(void)
{
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;

    case HELP:
	console_printf(
	    "\tset ip4 addr <IP addr>/<subnet prefix size>\n"
	    "\tset ip6 addr <IP addr>/<subnet prefix size>\n"
	    "\tset ip4 gateway <IP addr>\n"
	    "\n");

	break;

#ifdef _NETWORKING_
    case IP4:
	parse_set_ip4_command();
        break;

    case IP6:
	parse_set_ip6_command();
        break;
#endif

    case END_OF_INPUT:
    default:
	console_printf("\'set\' command syntax error (type \'set help\')\n");
	break;
    }
}


static void
parse_get(void)
{
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
	break;

    case HELP:
	console_printf(
	    "\tget ip4 addr\n"
	    "\tget ip6 addr\n"
	    "\n");
	break;

#ifdef _NETWORKING_
    case IP4:
	parse_get_ip4_command();
        break;

    case IP6:
	parse_get_ip6_command();
        break;
#endif

    case END_OF_INPUT:
    default:
	console_printf("\'get\' command syntax error (type \'get help\')\n");
	break;
    }
}


static void
cmd_display_help(void)
{
    console_printf(
        "\nMcRTOS commands\n"
        "\tclear - clear screen\n"
        "\tcpu - Switch to the given CPU\n"
        "\tdmesg - Display message log\n"
        "\tget - Get config option\n"
        "\thelp - display this message\n"
#ifdef _NETWORKING_
	"\tping - send ping to a given IP address\n"
	"\tping6 - send ping to a given IPv6 address\n"
#endif /* _NETWORKING_ */
        "\treset - reset CPU\n"
        "\tset - Set config option\n"
        "\tstack - display stack trace of a given thread/ISR\n"
        "\ttop - display McRTOS stats\n"
        "\tversion - display McRTOS version\n");

    for (int i = 0; i < g_command_processor.num_app_console_commands; i++) {
        console_printf("\t%s - %s\n",
            g_command_processor.app_console_commands_p[i].cmd_name_p,
            g_command_processor.app_console_commands_p[i].cmd_description_p);
    }

    console_printf("\n");
}


static void
cmd_display_msg_log(void)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];
    struct fdc_info *fdc_info_p = &cpu_controller_p->cpc_failures_info;

    fdc_info_p->fdc_msg_buffer[RTOS_FDC_MSG_BUFFER_SIZE - 1] = '\0';
    console_printf("FDC message buffer for CPU: %u\n\n", cpu_id);

    for (char *s = fdc_info_p->fdc_msg_buffer; *s != '\0'; s ++) {
        uart_putchar(g_console_serial_port_p, *s);
    }

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}

static void
cmd_display_stack_trace(uintptr_t execution_context_addr, bool only_call_entries)
{
    if (!BOARD_VALID_RAM_ADDRESS(execution_context_addr) || execution_context_addr % 4 != 0) {
        console_printf("ERROR: Invalid address %p\n", execution_context_addr);
        return;
    }

    struct rtos_execution_context *execution_context_p =
        (struct rtos_execution_context *)execution_context_addr;

    if (execution_context_p->ctx_signature !=
            RTOS_EXECUTION_CONTEXT_SIGNATURE) {
        console_printf("ERROR: Invalid execution context address %p\n",
                       execution_context_addr);
        return;
    }

    if (execution_context_p->ctx_context_type != RTOS_THREAD_CONTEXT) {
        console_printf("ERROR: Execution context %p is not a thread\n",
                       execution_context_p);
        return;
    }

    const uint32_t *context_stack_p;

#   if DEFINED_ARM_CLASSIC_ARCH()
    context_stack_p = execution_context_p->ctx_cpu_registers[CPU_REG_SP];
#   elif DEFINED_ARM_CORTEX_M_ARCH()
    context_stack_p =
        (rtos_execution_stack_entry_t *)execution_context_p->ctx_cpu_saved_registers.cpu_reg_psp;
#   else
        #error "unsupported CPU architecture"
#   endif

    console_printf("Stack trace for %s:\n",
                   execution_context_p->ctx_name_p);

    for ( ; context_stack_p < execution_context_p->ctx_execution_stack_bottom_end_p;
         context_stack_p ++) {
        uint32_t stack_entry = *context_stack_p;
        if ((stack_entry & 0x1) != 0 && VALID_CODE_ADDRESS(stack_entry)) {
            cpu_instruction_t *call_stack_entry =
                 (cpu_instruction_t *)((stack_entry & ~0x1) -
                                       sizeof(cpu_instruction_t));

            console_printf("\t%#p: %#x (call stack entry: %#p)\n",
                context_stack_p, stack_entry, call_stack_entry);
        } else if (!only_call_entries) {
            console_printf("\t%#p: %#x\n", context_stack_p, stack_entry);
        }
    }
}


static void
parse_stack(void)
{
    token_t token = get_next_token(&g_command_processor);
    uintptr_t mem_addr;

    switch (token) {
    case INVALID_TOKEN:
	break;

    case HELP:
	console_printf("\tstack <execution context address>\n\n");
	break;

    case HEXADECIMAL_NUMBER:
        mem_addr = convert_string_to_hexadecimal(g_command_processor.last_lexical_unit);

        bool caller_was_privileged = rtos_enter_privileged_mode();

	cmd_display_stack_trace(mem_addr, true);
        if (!caller_was_privileged) {
            rtos_exit_privileged_mode();
        }

	break;

    case END_OF_INPUT:
    default:
	console_printf("\'stack\' command syntax error (type \'stack help\')\n");
    }
}

void
rtos_parse_command_line(const char *cmd_line)
{
    int i;

    g_command_processor.cmd_line_cursor = g_command_processor.command_line_buffer;
    token_t token = get_next_token(&g_command_processor);

    switch (token) {
    case INVALID_TOKEN:
    case END_OF_INPUT:
        break;

    case BREAK_INPUT:
        (void)rtos_enter_privileged_mode();
        __disable_irq();
        rtos_run_debugger(NULL, NULL);
        __enable_irq();
        rtos_exit_privileged_mode();
        break;

    case CLEAR:
        console_clear();
        break;

    case CPU:
        McRTOS_change_console_cpu((SOC_GET_CURRENT_CPU_ID() + 1) % SOC_NUM_CPU_CORES);
        break;

    case DMESG:
        cmd_display_msg_log();
        break;

    case GET:
        parse_get();
        break;

    case HELP:
        cmd_display_help();
        break;

    case RESET:
        (void)rtos_enter_privileged_mode();
        rtos_reboot();
        /*UNREACHABLE*/

        FDC_ASSERT(false, 0, 0);
        break;

    case SET:
        parse_set();
        break;

    case STACK:
        parse_stack();
        break;

    case TOP:
        McRTOS_display_stats();
        break;

    case VERSION:
        console_printf("%s\n%s\n", g_McRTOS_version, g_McRTOS_build_timestamp);
        break;

#ifdef _NETWORKING_
    case PING:
        parse_ping();
        break;

    case PING6:
        parse_ping6();
        break;
#endif /* _NETWORKING_ */

    case APP_COMMAND:
        for (i = 0; i < g_command_processor.num_app_console_commands; i++) {
            if (strcmp(g_command_processor.last_lexical_unit,
                       g_command_processor.app_console_commands_p[i].cmd_name_p) == 0) {
                g_command_processor.app_console_commands_p[i].cmd_function_p();
                break;
            }
        }

        if (i == g_command_processor.num_app_console_commands) {
            console_printf("Invalid command: \'%s\' (type help)\n",
                           g_command_processor.last_lexical_unit);
        }

        break;

    default:
	console_printf("Invalid command: \'%s\' (type help)\n",
		       g_command_processor.last_lexical_unit);
    }
}


static void
McRTOS_display_stats(void)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();

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

#   ifdef _CPU_CYCLES_MEASURE_
    console_printf(
	"Context    Name                           Priority Switched-out Preempted  CPU        CPU under 1ms  Tstamp last  FPU     Switched-out\n"
	"address                                            count        count      usage (ms) usage (cycles) switched-out enabled history     \n"
	"========== ============================== ======== ============ ========== ========== ============== ============ ======= ============\n");
#   else
    console_printf(
	"Context    Name                           Priority Switched-out Preempted  Tstamp last  FPU     Switched-out\n"
	"address                                            count        count      switched-out enabled history     \n"
	"========== ============================== ======== ============ ========== ============ ======= ============\n");
#   endif

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

#   ifdef _CPU_CYCLES_MEASURE_
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
#   else
        console_printf(
	    "%#8p %30s %c%7u %12u %10u %12u %7s %#x%x\n",
            context_p,
            context_p->ctx_name_p,
            context_type_symbol,
            priority,
            context_p->ctx_switched_out_counter,
            context_p->ctx_preempted_counter,
            context_p->ctx_last_switched_out_time_stamp_in_ticks,
	    fpu_enabled,
            context_p->ctx_switched_out_reason_history,
            context_p->ctx_last_switched_out_reason
        );
#   endif
    }

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


static void
McRTOS_change_console_cpu(cpu_id_t cpu_id)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();

    g_McRTOS_p->rts_console_input_cpu_id = cpu_id;
    __DSB();
    send_inter_processor_interrupt(cpu_id);

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}
