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
#include "hardware_abstractions.h"
#include "compile_time_checks.h"
#include "failure_data_capture.h"

#define ARRAY_SIZE(_array) \
        (sizeof(_array) / sizeof((_array)[0]))

#define BIT(_bit_index)  (UINT32_C(0x1) << (_bit_index))

#define MULTI_BIT_MASK(_most_significant_bit_index,                     \
                       _least_significant_bit_index)                    \
        (BIT(_most_significant_bit_index) |                             \
         ((BIT(_most_significant_bit_index) - 1) &                      \
          (UINT32_MAX << _least_significant_bit_index)))

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

#define HOW_MANY(_m, _n)  (((size_t)(_m) - 1) / (_n) + 1)

#define ENCLOSING_STRUCT(                                                           \
            _enclosed_struct_p, _enclosing_struct_type,  _enclosing_struct_field)   \
	((_enclosing_struct_type *)(					            \
		(uintptr_t)(_enclosed_struct_p) - 				    \
		offsetof(_enclosing_struct_type, _enclosing_struct_field)))

#define STRINGIFY_LITERAL(_x)   #_x

#ifdef DEBUG
#   define DEBUG_PRINTF(_fmt, ...) \
            debug_printf(                                                   \
                "DBG: " __FILE__ ":" STRINGIFY_LITERAL(__LINE__) " " _fmt,  \
                ##__VA_ARGS__)

#   define DEBUG_BREAK_POINT(_fmt, ...) \
            debug_break_point(                                              \
                "DBG BKPT: " __FILE__ ":" STRINGIFY_LITERAL(__LINE__) " "   \
                _fmt, ##__VA_ARGS__)

#else
#   define DEBUG_PRINTF(_fmt, ...)
#   define DEBUG_BREAK_POINT(_fmt, ...)

#endif

#define TODO_IMPLEMENT_THIS() \
        do {                                                            \
            TODO("Implement this");                                     \
            DEBUG_PRINTF("%s not implemented yet\n", __func__);         \
        } while (0);


/**
 * ASCII codes of common control characters
 */ 
#define CTRL_C  UINT8_C(0x03)

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

void debug_printf(const char *fmt, ...);

void debug_break_point(const char *fmt, ...);

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

void
embedded_printf(
    putchar_func_t *putchar_func_p, void *putchar_arg_p, const char *fmt, ...);

extern const char g_clear_console_control_string[];

#endif /* __UTILS_H */
