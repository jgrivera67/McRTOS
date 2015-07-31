/**
 * @file McRTOS_command_processor.h
 *
 * McRTOS command-line processor module.
 *
 * Copyright (C) 2014 German Rivera
 *
 * @author German Rivera
 */
#ifndef _McRTOS_CMD_PROCESSOR_H
#define _McRTOS_CMD_PROCESSOR_H

struct ipv4_address;
struct ipv6_address;

enum common_tokens {
    INVALID_TOKEN = 0,
    END_OF_INPUT,
    BREAK_INPUT,
    HEXADECIMAL_NUMBER,
    DECIMAL_NUMBER,
    DOT_TOKEN,
    COLON_TOKEN,
    SLASH_TOKEN,
    APP_COMMAND,
    FIRST_KEYWORD_TOKEN,
};

enum tokens {
    ADDR = FIRST_KEYWORD_TOKEN,
    CLEAR,
    CPU,
    DMESG,
    GET,
    HELP,
#ifdef _NETWORKING_
    GATEWAY,
    IP4,
    IP6,
    PING,
    PING6,
#endif
    RESET,
    SET,
    STACK,
    TOP,
    VERSION,
};

struct __command_processor {
    /**
     * Pointer to table of keywords
     */
    const char *const *keyword_table_p;

    /**
     * Number of entries of the keyword table
     */
    uint8_t num_keywords;

    /**
     * Pointer to next character to process from the command line
     */
    const char *cmd_line_cursor;

   /**
     * Number of application-specific console commands
     */
    uint8_t num_app_console_commands;

    /**
     * Pointer to array of application-specific console command
     */
    const struct rtos_console_command *app_console_commands_p;

    /**
     * Maximum size of a lexical unit, including null terminator
     */
#   define LEXICAL_UNIT_MAX_SIZE 32

    /**
     * Buffer to hold the last lexical unit found in the command line
     */
    char last_lexical_unit[LEXICAL_UNIT_MAX_SIZE];

    /**
     * Command line buffer
     */
    char command_line_buffer[RTOS_COMMAND_LINE_BUFFER_SIZE];
};

struct command_processor {
    struct __command_processor;
}  __attribute__ ((aligned(SOC_MPU_REGION_ALIGNMENT(struct __command_processor))));

C_ASSERT(sizeof(struct command_processor) % SOC_MPU_REGION_ALIGNMENT(struct __command_processor) == 0);

typedef int token_t;

void init_command_processor(
    uint8_t num_app_console_commands,
    const struct rtos_console_command *app_console_commands_p);

token_t get_next_token(struct command_processor *command_processor_p);

bool parse_ip4_address(struct ipv4_address *ip_addr_p);

bool parse_ip6_address(struct ipv6_address *ip_addr_p);

void rtos_command_processor(void);

extern struct command_processor g_command_processor;

#endif /* _McRTOS_CMD_PROCESSOR_H */
