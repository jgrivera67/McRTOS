/**
 * @file utils.h
 *
 * General utilities
 *
 * @author German Rivera
 */

#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>     // C99 int types
#include <stdbool.h>    // bool
#include <stddef.h>     // offsetof()
#include <stdarg.h>
#include <stdalign.h>
#include "hardware_abstractions.h"
#include "compile_time_checks.h"
#include "failure_data_capture.h"

#define ARRAY_SIZE(_array) \
        (sizeof(_array) / sizeof((_array)[0]))

#define BIT(_bit_index)     (UINT32_C(0x1) << (_bit_index))

#define BIT64(_bit_index)   (UINT64_C(0x1) << (_bit_index))

#define MULTI_BIT_MASK(_most_significant_bit_index,                     \
                       _least_significant_bit_index)                    \
        (BIT(_most_significant_bit_index) |                             \
         ((BIT(_most_significant_bit_index) - 1) &                      \
          (UINT32_MAX << (_least_significant_bit_index))))

#define MULTI_BIT_MASK64(_most_significant_bit_index,                   \
                       _least_significant_bit_index)                    \
        (BIT64(_most_significant_bit_index) |                           \
         ((BIT64(_most_significant_bit_index) - 1) &                    \
          (UINT64_MAX << (_least_significant_bit_index))))

#define GET_BIT_FIELD(_container, _bit_mask, _bit_shift) \
    (((_container) & (_bit_mask)) >> (_bit_shift))

#define CLEAR_BIT_FIELD(_container, _bit_mask) \
    do {                                                                \
        (_container) &= ~(_bit_mask);                                   \
    } while (0)

#define SET_BIT_FIELD(_container, _bit_mask, _bit_shift, _value) \
    do {                                                                \
        (_container) &= ~(_bit_mask);                                   \
        FDC_ASSERT(((uint32_t)(_value) << (_bit_shift)) <= (_bit_mask), \
               _value, _bit_mask);                                      \
        (_container) |=                                                 \
            ((uint32_t)(_value) << (_bit_shift)) & (_bit_mask);         \
    } while (0)

#define BIT_MAP8(_bit7, _bit6, _bit5, _bit4, _bit3, _bit2, _bit1, _bit0) \
                ((_bit7) ? BIT(7) : UINT8_C(0x0) | \
                 (_bit6) ? BIT(6) : UINT8_C(0x0) | \
                 (_bit5) ? BIT(5) : UINT8_C(0x0) | \
                 (_bit4) ? BIT(4) : UINT8_C(0x0) | \
                 (_bit3) ? BIT(3) : UINT8_C(0x0) | \
                 (_bit2) ? BIT(2) : UINT8_C(0x0) | \
                 (_bit1) ? BIT(1) : UINT8_C(0x0) | \
                 (_bit0) ? BIT(0) : UINT8_C(0x0))

#define HOW_MANY(_m, _n)    (((size_t)(_m) - 1) / (_n) + 1)

#define ROUND_UP(_m, _n)    (HOW_MANY(_m, _n) * (_n))

#define ROUND_DOWN(_m, _n)  (((size_t)(_m) / (_n)) * (_n))

#define UINT_DIV_APPROX(_m, _n) \
        ((natural_t)(_m) / (natural_t)(_n) +                            \
         ((natural_t)(_m) % (natural_t)(_n) > (natural_t)(_n) / 2 ?     \
          1 : 0))

#define ENCLOSING_STRUCT(                                                           \
            _enclosed_struct_p, _enclosing_struct_type,  _enclosing_struct_field)   \
	((_enclosing_struct_type *)(					            \
		(uintptr_t)(_enclosed_struct_p) - 				    \
		offsetof(_enclosing_struct_type, _enclosing_struct_field)))

#define STRINGIFY_LITERAL(_num_literal) \
        __STRINGIFY_EXPANDED_LITERAL(_num_literal)

#define __STRINGIFY_EXPANDED_LITERAL(_expanded_num_literal) \
        #_expanded_num_literal

#define IS_PRINT(_c)    ((_c) >= ' ' && (_c) <= '~')

#define ABS(_value)     ((_value) < 0 ? -(_value) : (_value))

#define MAX(_x, _y)     ((_x) >= (_y) ? (_x) : (_y))

#ifdef DEBUG
#   define DEBUG_PRINTF(_fmt, ...) \
            capture_fdc_msg_printf("DBG %s:" STRINGIFY_LITERAL(__LINE__) " " _fmt, \
				     __func__, ##__VA_ARGS__)

#   define DEBUG_BREAK_POINT()  ARTIFICIAL_BREAK_POINT()

#   define DEBUG_BLINK_LED(_led_mask) \
            do {                                                            \
                uint32_t old_color = set_rgb_led_color(_led_mask);          \
                delay_loop(500000);                                         \
                set_rgb_led_color(old_color);                               \
            } while (0)

#   define DEBUG_PRINT_CALLER(_msg) \
            do {                                                            \
                uint32_t *return_address;                                   \
                uint32_t *call_address;                                     \
                CAPTURE_ARM_LR_REGISTER(return_address);                    \
                call_address = return_address - 1;                          \
                DEBUG_PRINTF("%s called at: %#p %s\n", __func__,            \
                             call_address, _msg);                           \
            } while (0)

#else
#   define DEBUG_PRINTF(_fmt, ...)
#   define DEBUG_BREAK_POINT()
#   define DEBUG_BLINK_LED(_led_mask)
#   define DEBUG_PRINT_CALLER(_msg)

#endif

#define TODO_IMPLEMENT_THIS() \
        do {                                                            \
            TODO("Implement this");                                     \
            DEBUG_PRINTF("%s not implemented yet\n", __func__);         \
        } while (0);


/**
 * Artificial break point by triggering a "divide by 0" exception.
 *
 * NOTE:
 * Ideally, we should be able to use break instruction (e.g., __BKPT()).
 * However, the processor goes to lockup state if a debugger is not
 * attached, rather than generating an exception.
 * So, we need to force an "artificial fault" here,
 * by doing an unaligned memory access.
 */
#define ARTIFICIAL_BREAK_POINT() \
    do {                                                                \
        asm volatile (                                                  \
            "mov    r0, #0x0\n\t"                                       \
            "udiv   r0, r0, r0"                                         \
            : : : "r0"                                                  \
        );                                                              \
    } while (0)


#define CONSOLE_POS_PRINTF(_row, _col, _fmt, ...) \
	console_printf("\x1b[s\x1b[%u;%uH" _fmt "\x1b[u", _row, _col, \
		       ##__VA_ARGS__)

/**
 * ASCII codes of common control characters
 */
#define CTRL_C  UINT8_C(0x03)

typedef unsigned int natural_t;

void copy_memory_block(
    _OUT_ void *dest,
    _IN_ const void *src,
    _IN_ size_t num_bytes);

fdc_error_t
check_dram_memory_block(
    _OUT_ void *block_p,
    _IN_ size_t num_bytes);

void delay_loop(_IN_ uint32_t times);

struct lcd_args;

void console_printf_init(void);

void console_clear(void);

void console_printf(const char *fmt, ...);

#ifdef LCD_SUPPORTED
void lcd_printf_init(void);

void lcd_printf(
    lcd_x_t x, lcd_y_t y,
    const struct lcd_char_attributes *lcd_char_attributes_p,
    const char *fmt, ...);
#endif

#ifdef CPPUTEST_COMPILATION  // from CppUTest

void cpputest_printf(const char *fmt, ...);

#endif /* CPPUTEST_COMPILATION */

typedef void putchar_func_t(void *putchar_arg_p, uint8_t c);

typedef uint8_t getchar_func_t(void *getchar_arg_p);

void embedded_printf(
        putchar_func_t *putchar_func_p, void *putchar_arg_p,
        const char *fmt, ...);

void embedded_vprintf(
        putchar_func_t *putchar_func_p, void *putchar_arg_p,
        const char *fmt, va_list va);

void read_command_line(
        _IN_  putchar_func_t *putchar_func_p,
        _IN_  getchar_func_t *getchar_func_p,
        _IN_  void *char_io_arg_p,
        _OUT_ char *cmd_line_buffer,
        _IN_  size_t buffer_size);

uint32_t convert_string_to_hexadecimal(
        _IN_ const char *str);

uint32_t convert_string_to_decimal(
        _IN_ const char *str);

char * signature_to_string(
        _IN_ uint32_t signature);

int strcmp(_IN_ const char *s1, _IN_ const char *s2);
int strlen(_IN_ const char *s);
void bzero(_INOUT_ uint8_t *buf, size_t len);

cpu_instruction_t *get_program_counter(void);

static inline int to_lower(int c)
{
    return (c >= 'A' && c <= 'Z') ? 'a' + (c - 'A')
	                          : c;
}

static inline bool is_digit(int c)
{
    return c >= '0' && c <= '9';
}

static inline bool is_xdigit(int c)
{
    if (is_digit(c)) {
	return true;
    }

    return (c >= 'a' && c <= 'f') ||
           (c >= 'A' && c <= 'F');
}

static inline bool is_space(int c)
{
    return c == ' ' || c == '\t';
}

static inline bool is_alpha(int c)
{
    return (c >= 'A' && c <= 'Z') ||
           (c >= 'a' && c <= 'z');
}

#endif /* __UTILS_H */
