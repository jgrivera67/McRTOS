/**
 * @file utils.c
 *
 * General utilities
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 
#include "utils.h"
#include "failure_data_capture.h"
#include "hardware_abstractions.h"
#include "McRTOS.h"
#include "McRTOS_kernel_services.h"
#include "McRTOS_internals.h"
#include <inttypes.h>
#include <stdarg.h>

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-function"

static void
embedded_vprintf(
    putchar_func_t *putchar_func_p, void *putchar_arg_p, const char *fmt, va_list va);

static void 
print_uint32_hexadecimal(
    putchar_func_t *putchar_func_p, void *putchar_arg_p, uint32_t value,
    uint8_t padding_count);

static void
print_uint32_decimal(
    putchar_func_t *putchar_func_p, void *putchar_arg_p, uint32_t value,
    uint8_t padding_count);

static void
print_int32_decimal(
    putchar_func_t *putchar_func_p, void *putchar_arg_p, int32_t value,
    uint8_t padding_count);

static void
print_string(
    putchar_func_t *putchar_func_p, void *putchar_arg_p, const char *str,
    uint8_t padding_count);

C_ASSERT(sizeof(uintptr_t) == sizeof(uint32_t));

/**
 * Clear screen control sequence for VT100 terminals
 */
const char g_clear_console_control_string[] = "\x1b[2J\x1b[H";

/**
 * Mutex to serialize printf output to the console
 */
static struct rtos_mutex g_console_printf_mutex;

#ifdef LCD_SUPPORTED
/**
 * Mutex to serialize printf output to the LCD
 */
static struct rtos_mutex g_lcd_printf_mutex;
#endif

/**
 * Copies a word-aligned memory block
 *
 * @param dest  Pointer to destination area
 *
 * @param src   Pointer to source area
 *
 * @param num_bytes  Size of the block to copy in bytes
 *
 * @return  none
 */
void
copy_memory_block(
    void *dest,
    const void *src,
    size_t num_bytes
    )
{
    /**
     * Number of memory words to transfer in a single ldmia/stmia pair
     */ 
#   define MEM_TRANSFER_BLOCK_SIZE   8

    /**
     * Number of memory words to transfer in a single iteration
     */ 
#   define COPY_ITERATION_BLOCK_SIZE (8 * MEM_TRANSFER_BLOCK_SIZE)

    struct CopyIterationBlock {
        uint32_t words[COPY_ITERATION_BLOCK_SIZE];
    };

    struct MemTransferBlock {
        uint32_t words[MEM_TRANSFER_BLOCK_SIZE];
    };

    FDC_ASSERT(
        num_bytes != 0 && num_bytes % sizeof(uint32_t) == 0,
        num_bytes, 0);

    uint32_t numWords = num_bytes / sizeof(uint32_t);
    struct MemTransferBlock* destTransferBlock = (struct MemTransferBlock *)dest;    
    struct MemTransferBlock* srcTransferBlock = (struct MemTransferBlock *)src;
    uint32_t remainingWords = numWords;
    uint32_t numTransfers = remainingWords / COPY_ITERATION_BLOCK_SIZE;
    
    FDC_ASSERT_VALID_RAM_POINTER(dest, sizeof(uint32_t));
    FDC_ASSERT_VALID_RAM_POINTER(
        (uint32_t *)dest + (numWords - 1), sizeof(uint32_t));

    FDC_ASSERT_VALID_RAM_POINTER(src, sizeof(uint32_t));
    FDC_ASSERT_VALID_RAM_POINTER(
        (uint32_t *)src + (numWords - 1), sizeof(uint32_t));

    for ( ; numTransfers != 0; numTransfers --)
    {
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
    }

    remainingWords %= COPY_ITERATION_BLOCK_SIZE;
    numTransfers = remainingWords / MEM_TRANSFER_BLOCK_SIZE;
   
    DBG_ASSERT(numTransfers < 8, numTransfers, 0);

    // 
    // Computed goto:
    //
    switch (numTransfers)
    {
    case 7:
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
    case 6:
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
    case 5:
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
    case 4:
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
    case 3:
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
    case 2:
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
    case 1:
        *destTransferBlock ++ = *srcTransferBlock ++;   // ldmia/stmia
    }

    remainingWords %= MEM_TRANSFER_BLOCK_SIZE;
    
    uint32_t *dest_word_p = (uint32_t *)destTransferBlock;
    uint32_t *src_word_p = (uint32_t *)srcTransferBlock;
    
    DBG_ASSERT(remainingWords < 8, remainingWords, 0);

    // 
    // Computed goto:
    //
    switch (remainingWords)
    {
    case 7:
        *dest_word_p ++ = *src_word_p ++;                             // ldr/str
    case 6:
        *dest_word_p ++ = *src_word_p ++;                             // ldr/str
    case 5:
        *dest_word_p ++ = *src_word_p ++;                             // ldr/str
    case 4:
        *dest_word_p ++ = *src_word_p ++;                             // ldr/str
    case 3:
        *dest_word_p ++ = *src_word_p ++;                             // ldr/str
    case 2:
        *dest_word_p ++ = *src_word_p ++;                             // ldr/str
    case 1:
        *dest_word_p ++ = *src_word_p ++;                             // ldr/str
    }
}


#ifdef DRAM_SUPPORTED
/**
 * Checks that all the bits of a word-aligned DRAM memory block can be written
 * as 0's and as 1's.
 *
 * @param block_p   Pointer to RDAM memory block
 *
 * @param num_bytes Size of the block to copy in bytes
 *
 * @return  0, on success
 * @return  non-zero error code, on failure
 */
fdc_error_t
check_dram_memory_block(
    void *block_p,
    size_t num_bytes)
{
    /**
     * Number of unrolled loop iterations
     */ 
#   define CHECK_DRAM_UNROLLED_ITERATIONS   8

    /**
     * Macro that checks a single word, by writings 0s and 1s to all bits
     */
#   define CHECK_WORD(_word_p) \
    do {                                                                \
        CHECK_WORD_WRITE_READ_BACK(_word_p, 0x0);                       \
        CHECK_WORD_WRITE_READ_BACK(_word_p, UINT32_MAX);                \
    } while (0)

    /**
     * Macro that checks a single word, by writing a given value to it and
     * reading it back
     */
#   define CHECK_WORD_WRITE_READ_BACK(_word_p, _value) \
    do {                                                                \
        *(_word_p) = (_value);                                          \
        if (*(_word_p) != (_value)) {                                   \
            fdc_error = CAPTURE_FDC_ERROR(                              \
                    "Could not write given value to DRAM word",         \
                    _word_p, _value);                                   \
            goto Exit;                                                  \
        }                                                               \
    } while (0)

    fdc_error_t fdc_error;

    FDC_ASSERT(
        num_bytes != 0 && num_bytes % sizeof(uint32_t) == 0,
        num_bytes, 0);

    FDC_ASSERT
        ((uintptr_t)block_p % sizeof(uint32_t) == 0,
         block_p, num_bytes);

    FDC_ASSERT(
        (uintptr_t)block_p >= BOARD_SDRAM_BASE &&
        (uintptr_t)block_p + num_bytes <= BOARD_SDRAM_BASE + BOARD_SDRAM_SIZE,
        block_p, num_bytes);

    uint32_t numWords = num_bytes / sizeof(uint32_t);
    uint32_t numIterations = numWords / CHECK_DRAM_UNROLLED_ITERATIONS;
    uint32_t remainingWords = numWords % CHECK_DRAM_UNROLLED_ITERATIONS;
    volatile uint32_t *word_p = block_p;

    for ( ; numIterations != 0; numIterations --)
    {
        CHECK_WORD(word_p);
        word_p ++;
        CHECK_WORD(word_p);
        word_p ++;
        CHECK_WORD(word_p);
        word_p ++;
        CHECK_WORD(word_p);
        word_p ++;
        CHECK_WORD(word_p);
        word_p ++;
        CHECK_WORD(word_p);
        word_p ++;
        CHECK_WORD(word_p);
        word_p ++;
        CHECK_WORD(word_p);
        word_p ++;
    }

    DBG_ASSERT(remainingWords < 8, remainingWords, 0);

    // 
    // Computed goto:
    //
    switch (remainingWords)
    {
    case 7:
        CHECK_WORD(word_p);
        word_p ++;
    case 6:
        CHECK_WORD(word_p);
        word_p ++;
    case 5:
        CHECK_WORD(word_p);
        word_p ++;
    case 4:
        CHECK_WORD(word_p);
        word_p ++;
    case 3:
        CHECK_WORD(word_p);
        word_p ++;
    case 2:
        CHECK_WORD(word_p);
        word_p ++;
    case 1:
        CHECK_WORD(word_p);
        word_p ++;
    }

    fdc_error = 0;

Exit:
    return fdc_error;
}
#endif /* DRAM_SUPPORTED */

void
delay_loop(uint32_t times)
{
    /*
     * Approx. 5*times CPU clock cycles. If The CPU clock frequency is 72MHz,
     * there are 72000 CPU clock cycles in 1 ms and 72 CPU clock cycles in 1 us.
     */
    for (volatile uint32_t i = 0; i < times; i ++)
        ;
}


/**
 * Simplified printf that sends debug messages to the console. It only supports the
 * format specifiers supported by embedded_vprintf(). It disables interrupts while
 * transmitting and does polling on the serial port until all characters are
 * transmitted.
 *
 * @param fmt               format string
 *
 * @param ...               variable arguments
 *
 * @return None
 */
void
debug_printf(const char *fmt, ...)
{
    va_list va;

    cpu_status_register_t old_primask = __get_PRIMASK();

    __disable_irq();

    va_start(va, fmt);

    embedded_vprintf(
        (putchar_func_t *)uart_putchar_with_polling,
        (void *)g_console_serial_port_p, fmt, va);

    va_end(va);

    if (CPU_INTERRUPTS_ARE_ENABLED(old_primask)) {
        __enable_irq();
    }

}


/**
 * Simplified printf that send output to the console. It only supports the
 * format specifiers supported by embedded_vprintf()
 *
 * @param fmt               format string
 *
 * @param ...               variable arguments
 *
 * @return None
 */
void
console_printf(const char *fmt, ...)
{
    va_list va;
    putchar_func_t *putchar_func_p;
    cpu_status_register_t cpu_status_register;

    //???bool caller_is_thread = rtos_k_caller_is_thread();
    bool caller_is_thread = false;

    if (caller_is_thread) {
        putchar_func_p = rtos_console_putchar;
        rtos_mutex_acquire(&g_console_printf_mutex);
    } else {
        putchar_func_p = rtos_k_console_putchar_with_polling;
        cpu_status_register = rtos_k_disable_cpu_interrupts();
    }

    va_start(va, fmt);

    embedded_vprintf(putchar_func_p, NULL, fmt, va);

    va_end(va);

    if (caller_is_thread) {
        rtos_mutex_release(&g_console_printf_mutex);
    } else {
        rtos_k_restore_cpu_interrupts(cpu_status_register);
    }

}


/**
 * Initializes console printf mutex
 */
void
console_printf_init(void)
{
    rtos_k_mutex_init(
        "console printf mutex",
        0,
        &g_console_printf_mutex);
}


/**
 * Clears the console screen and moves cursor to home
 */
void
console_clear(void)
{
    console_printf(g_clear_console_control_string);
}


#ifdef LCD_SUPPORTED

/**
 * Initializes LCD printf mutex
 */
void
lcd_printf_init(void)
{
    rtos_k_mutex_init(
        "LCD printf mutex",
        0,
        &g_lcd_printf_mutex);
}

/**
 * Simplified printf that send output to the LCD. It only supports the
 * format specifiers supported by embedded_vprintf()
 *
 * @param fmt               format string
 *
 * @param ...               variable arguments
 *
 * @return None
 */
void
lcd_printf(
    lcd_x_t x, lcd_y_t y,
    const struct lcd_char_attributes *lcd_char_attributes_p,
    const char *fmt, ...)
{
    va_list va;
    struct rtos_lcd_putchar_attributes lcd_putchar_attributes = {
        .lcd_x = x,
        .lcd_y = y,
        .lcd_char_attributes_p = lcd_char_attributes_p
    };

    rtos_mutex_acquire(&g_lcd_printf_mutex);

    va_start(va, fmt);

    embedded_vprintf(
        (putchar_func_t *)rtos_lcd_putchar, &lcd_putchar_attributes, fmt, va);

    va_end(va);

    rtos_mutex_release(&g_lcd_printf_mutex);

}

#endif /* LCD_SUPPORTED */


#ifdef CPPUTEST_COMPILATION  // from CppUTest
void
cpputest_printf(const char *fmt, ...)
{
    void cpputest_putchar(void *putchar_arg_p, uint8_t c);
    va_list va;

    va_start(va, fmt);
    embedded_vprintf(cpputest_putchar, NULL, fmt, va);
    va_end(va);
}
#endif /* CPPUTEST_COMPILATION */


/**
 * Simplified printf function that calls the function pointed to by putchar_func_p
 * one character at a time, with the string resulting of processing the given
 * printf format string. It only supports the following format specifiers:
 * %x, %p, %u, %s.
 *
 * @param putchar_func_p    character output function
 *
 * @param putchar_arg_p     Argument to be passed to putchar_func_p() on every 
 *                          invocation
 *
 * @param fmt               format string
 *
 * @param ...               variable arguments
 *
 * @return None
 */
void
embedded_printf(
    putchar_func_t *putchar_func_p, void *putchar_arg_p, const char *fmt, ...)

{
    va_list va;

    va_start(va, fmt);
    embedded_vprintf(putchar_func_p, putchar_arg_p, fmt, va);
    va_end(va);
}


/**
 * Simplified vprintf function that calls the function pointed to by putchar_func_p
 * one character at a time, with the string resulting of processing the given
 * printf format string. It only supports the following format specifiers:
 * %x, %p, %u, %s.
 *
 * @param putchar_func_p    character output function
 *
 * @param putchar_arg_p     Argument to be passed to putchar_func_p() on every 
 *                          invocation
 *
 * @param fmt               format string
 *
 * @param va                variable argument list
 *
 * @return None
 */
static void
embedded_vprintf(
    putchar_func_t *putchar_func_p, void *putchar_arg_p, const char *fmt, va_list va) 
{
    const char *cursor_p = fmt;
    bool parsing_format_specifier = false;
    bool print_numeric_base_prefix = false;
    uint8_t padding_count;

    FDC_ASSERT_VALID_FUNCTION_POINTER(putchar_func_p);
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(fmt, sizeof(char));

    while (*cursor_p != '\0')
    {
        uint8_t c = *cursor_p ++;

        if (parsing_format_specifier)
        {
            switch (c)
            {
            case 'x':
            case 'X':
            case 'p':
                if (print_numeric_base_prefix)
                {
                    print_string(putchar_func_p, putchar_arg_p, "0x", 0);
                    print_numeric_base_prefix = false;
                    if (padding_count >= 2)
                    {
                        padding_count -= 2;
                    }
                }

                print_uint32_hexadecimal(putchar_func_p, putchar_arg_p, va_arg(va, uint32_t), 
                    padding_count);
                break;

            case 'u':
                print_uint32_decimal(putchar_func_p, putchar_arg_p, va_arg(va, uint32_t),
                    padding_count);
                break;

            case 'd':
                print_int32_decimal(putchar_func_p, putchar_arg_p, va_arg(va, int32_t),
                    padding_count);
                break;

            case 'c':
                putchar_func_p(putchar_arg_p, va_arg(va, uint32_t));
                break;

            case 's':
                print_string(putchar_func_p, putchar_arg_p, va_arg(va, char *),
                    padding_count);
                break;

            case '#':
                print_numeric_base_prefix = true;
                continue;

            default:
                if (c >= '0' && c <= '9')
                {
                    padding_count *= 10;
                    padding_count += (c - '0');
                    continue;
                }
                else
                {
                    putchar_func_p(putchar_arg_p, c);
                }
            }

            parsing_format_specifier = false;
        }
        else if (c == '%')
        {
            parsing_format_specifier = true;
            padding_count = 0;
        }
        else  if (c == '\n')
        {
            putchar_func_p(putchar_arg_p, '\r');
            putchar_func_p(putchar_arg_p, '\n');
        }
        else
        {
            putchar_func_p(putchar_arg_p, c);
        }
    }
}


static void
print_uint32_hexadecimal(putchar_func_t *putchar_func_p, void *putchar_arg_p, uint32_t value,
    uint8_t padding_count)
{
    uint32_t   u32Mask  = 0xF0000000;
    uint8_t    u32Shift = 32;
    uint32_t   u32Char;
    uint8_t    char_printed_count = 0;
    uint8_t    first_non_zero_nibble_found = false;

    do {
        u32Shift -= 4;
        u32Char = (value & u32Mask) >> u32Shift;
        u32Mask >>= 4;

        if (!first_non_zero_nibble_found)
        {
            if (u32Char == 0)
            {
                continue;
            }

            first_non_zero_nibble_found = true;
        }

        if (u32Char >= 0xA)
        {
            putchar_func_p(putchar_arg_p, 'A' + (u32Char - 10));
        }
        else
        {
            putchar_func_p(putchar_arg_p, '0' + u32Char);
        }

        char_printed_count ++;
    } while (u32Shift > 0);

    if (char_printed_count == 0)
    {
        putchar_func_p(putchar_arg_p, '0');
        char_printed_count ++;
    }

    while (char_printed_count < padding_count)
    {
        putchar_func_p(putchar_arg_p, ' ');
        char_printed_count ++;
    }
}


static void
print_uint32_decimal(
    putchar_func_t *putchar_func_p, void *putchar_arg_p, uint32_t value,
    uint8_t padding_count)
{
    char      buffer[16];
    char      *p = &buffer[sizeof(buffer) - 1];

    *p = '\0';
    do {
        p--;
        if (p < buffer) {
            p++;
            *p = 'T'; /* for truncated */
            break;
        }

        // FDC_ASSERT(p >= buffer, p, buffer);

        *p = (value % 10) + '0';
        value /= 10;
    } while (value > 0);

    print_string(putchar_func_p, putchar_arg_p, p, padding_count);
}


static void
print_int32_decimal(
    putchar_func_t *putchar_func_p, void *putchar_arg_p, int32_t value,
    uint8_t padding_count)
{
    char      buffer[16];
    char      *p = &buffer[sizeof(buffer) - 1];
    uint32_t  abs_value;

    if (value < 0) {
        abs_value = (uint32_t)(-value);
    } else {
        abs_value = (uint32_t)value;
    }

    *p = '\0';
    do {
        p--;
        if (p < buffer) {
            p++;
            *p = 'T'; /* for truncated */
            goto Exit;
        }

        // FDC_ASSERT(p >= buffer, p, buffer);

        *p = (abs_value % 10) + '0';
        abs_value /= 10;
    } while (abs_value > 0);

    if (value < 0) {
        p --;
        *p = '-';
    }

Exit:
    print_string(putchar_func_p, putchar_arg_p, p, padding_count);
}


static void
print_string(putchar_func_t *putchar_func_p, void *putchar_arg_p, const char *str,
    uint8_t padding_count)
{
    FDC_ASSERT(str != NULL, 0 , 0);

    const char *cursor_p = str;
    uint8_t char_printed_count = 0;

    while (*cursor_p != '\0')
    {
        uint8_t c = *cursor_p ++;
    
        putchar_func_p(putchar_arg_p, c);
        char_printed_count ++;
        if (char_printed_count == padding_count)
        {
            break;
        }
    }

    while (char_printed_count < padding_count)
    {
        putchar_func_p(putchar_arg_p, ' ');
        char_printed_count ++;
    }
}


void
read_command_line(
    _IN_  putchar_func_t *putchar_func_p,
    _IN_  getchar_func_t *getchar_func_p,
    _IN_  void *char_io_arg_p,
    _OUT_ char *cmd_line_buffer,
    _IN_  size_t buffer_size)
{
    char *cmd_line_cursor = cmd_line_buffer;
    char *cmd_line_end = cmd_line_buffer + buffer_size - 1;

    for  ( ; ; ) {
        uint8_t c = getchar_func_p(char_io_arg_p);
        
        switch (c) {
        case '\r':
            putchar_func_p(char_io_arg_p, '\r');
            putchar_func_p(char_io_arg_p, '\n');
            *cmd_line_cursor = '\0';
            return;

        case '\b':
            if (cmd_line_cursor > cmd_line_buffer)
            {
                putchar_func_p(char_io_arg_p, '\b');
                putchar_func_p(char_io_arg_p, ' ');
                putchar_func_p(char_io_arg_p, '\b');
                cmd_line_cursor --;
            }
            break;

        default:
            if (cmd_line_cursor < cmd_line_end && IS_PRINT(c))
            {
                putchar_func_p(char_io_arg_p, c);
                *cmd_line_cursor++ = c;
            }
        }
    }
}


char *
signature_to_string(
    _IN_ uint32_t signature)
{
    static char str_buffer[sizeof(uint32_t) + 1];
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
    return str_buffer;
}


uint32_t 
convert_string_to_hexadecimal(
    _IN_ const char *str)
{
    return 0;
}


uint32_t 
convert_string_to_decimal(
    _IN_ const char *str)
{
    return 0;
}


void
debug_dump_r0_to_r3(
    uint32_t r0, 
    uint32_t r1, 
    uint32_t r2, 
    uint32_t r3)
{
    debug_printf(
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


void
debug_dump_captured_registers(void)
{
    debug_printf(
        "Registers captured buffers (next to fill %u):\n",
        g_debug_captured_registers_buffer_cursor);

    for (uint32_t i = 0; i < ARRAY_SIZE(g_debug_captured_registers_buffers); i ++) {
        struct debug_captured_registers_buffer *captured_registers_buffer_p =
            &g_debug_captured_registers_buffers[i];

        if (captured_registers_buffer_p->cap_location_addr != NULL) {
            debug_printf(
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
