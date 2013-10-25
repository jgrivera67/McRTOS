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

#ifdef DEBUG
#   define DEBUG_PRINTF(_fmt, ...) \
            debug_printf("DBG: %s:" STRINGIFY_LITERAL(__LINE__) " " _fmt, \
                         __func__, ##__VA_ARGS__)
            
#   define DEBUG_BREAK_POINT()  ARTIFICIAL_BREAK_POINT()

#   define DEBUG_BLINK_LED(_led_mask) \
            do {                                                            \
                uint32_t old_color = set_rgb_led_color(_led_mask);          \
                delay_loop(500000);                                         \
                set_rgb_led_color(old_color);                               \
            } while (0)

#else
#   define DEBUG_PRINTF(_fmt, ...)
#   define DEBUG_BREAK_POINT()
#   define DEBUG_BLINK_LED(_led_mask)

#endif

#define TODO_IMPLEMENT_THIS() \
        do {                                                            \
            TODO("Implement this");                                     \
            DEBUG_PRINTF("%s not implemented yet\n", __func__);         \
        } while (0);


/**
 * Unaligned access to force a data abort on classic ARM and a
 * hard fault exception on Cortex-M.
 *
 * NOTE:
 * Ideally, we should be able to use break instruction (e.g., __BKPT()).
 * However, for the Cortex-M0+, the processor goes to lockup state if
 * a debugger is not attached, rather than generating a hard fault
 * exception. So, we need to force an "artificial fault" here,
 * by doing an unaligned memory access.
 */
#define ARTIFICIAL_BREAK_POINT() \
    do {                                                                \
        asm volatile (                                                  \
            "mov    r0, #0x1\n\t"                                       \
            "ldr    r0, [r0]"                                           \
            : : : "r0"                                                  \
        );                                                              \
    } while (0)

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

void debug_printf(const char *fmt, ...);

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

void
debug_dump_r0_to_r3(
    uint32_t r0, 
    uint32_t r1, 
    uint32_t r2, 
    uint32_t r3);

void
debug_capture_registers(
    uint32_t r0, 
    uint32_t r1, 
    uint32_t r2, 
    uint32_t r3);

void
debug_dump_captured_registers(void);

extern const char g_clear_console_control_string[];

#endif /* __UTILS_H */
