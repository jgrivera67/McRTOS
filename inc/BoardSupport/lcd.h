/**
 * @file lcd.h
 *
 * Hardware abstraction layer for the LCD
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 
#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include "lcd_fonts.h"
#include "compile_time_checks.h"

/*
 * LCD X-axis size (one of 320, 640, 800, 1024)
 */
#define LCD_X_SIZE              UINT32_C(320)

/*
 * LCD Y-axis size (one of 240, 480, 600, 768)
 */
#define LCD_Y_SIZE              UINT32_C(240)

/*
 * Number of bits per pixel: 16 or 24
 */
#define LCD_BITS_PER_PIXEL      UINT32_C(16)

/**
 * Macro to generate 24-bit RGB-encoded colors:
 * bits[23:16]  Blue component
 * bits[15:8]   Green component
 * bits[7:0]    Red component
 */
#define LCD_24_BIT_RGB_COLOR(_red, _green, _blue) \
        ((UINT32_C(_blue) << 16) | (UINT32_C(_green) << 8) | UINT32_C(_red))

/**
 * Macros to generate 16-bit RGB-encoded colors (1:5:5:5):
 * bit[15]      Intensity
 * bit[14:10]   Blue component
 * bits[9:5]    Green component
 * bits[4:0]    Red component
 */
#define LCD_16_BIT_RGB_COLOR(_red, _green, _blue) \
        ((UINT16_C(_blue) << 10) |                                      \
         (UINT16_C(_green) << 5) |                                      \
         UINT16_C(_red))

#define LCD_16_BIT_RGB_COLOR_BRIGHT(_16bpp_rgb_color) \
        (BIT(15) | (_16bpp_rgb_color))


#if LCD_BITS_PER_PIXEL == 16

#   define LCD_COLOR_BLACK          LCD_16_BIT_RGB_COLOR(0x00, 0x00, 0x00)
#   define LCD_COLOR_RED            LCD_16_BIT_RGB_COLOR(0x1F, 0x00, 0x00)
#   define LCD_COLOR_GREEN          LCD_16_BIT_RGB_COLOR(0x00, 0x1F, 0x00)
#   define LCD_COLOR_YELLOW         LCD_16_BIT_RGB_COLOR(0x1F, 0x1F, 0x00)
#   define LCD_COLOR_BLUE           LCD_16_BIT_RGB_COLOR(0x00, 0x00, 0x1F)
#   define LCD_COLOR_MAGENTA        LCD_16_BIT_RGB_COLOR(0x1F, 0x00, 0x1F)
#   define LCD_COLOR_CYAN           LCD_16_BIT_RGB_COLOR(0x00, 0x1F, 0x1F)
#   define LCD_COLOR_WHITE          LCD_16_BIT_RGB_COLOR(0x1F, 0x1F, 0x1F)

#   define LCD_COLOR_BRIGHT(_color) \
    LCD_16_BIT_RGB_COLOR_BRIGHT(_color)

/**
 * LCD pixel color type
 */
typedef uint16_t lcd_color_t;

#elif LCD_BITS_PER_PIXEL == 24

#   define LCD_COLOR_BLACK          LCD_24_BIT_RGB_COLOR(0x00, 0x00, 0x00)
#   define LCD_COLOR_RED            LCD_24_BIT_RGB_COLOR(0xFF, 0x00, 0x00)
#   define LCD_COLOR_GREEN          LCD_24_BIT_RGB_COLOR(0x00, 0xFF, 0x00)
#   define LCD_COLOR_YELLOW         LCD_24_BIT_RGB_COLOR(0xFF, 0xFF, 0x00)
#   define LCD_COLOR_BLUE           LCD_24_BIT_RGB_COLOR(0x00, 0x00, 0xFF)
#   define LCD_COLOR_MAGENTA        LCD_24_BIT_RGB_COLOR(0xFF, 0x00, 0xFF)
#   define LCD_COLOR_CYAN           LCD_24_BIT_RGB_COLOR(0x00, 0xFF, 0xFF)
#   define LCD_COLOR_GREY           LCD_24_BIT_RGB_COLOR(0xAA, 0xAA, 0xAA)
#   define LCD_COLOR_WHITE          LCD_24_BIT_RGB_COLOR(0xFF, 0xFF, 0xFF)
#   define LCD_COLOR_LIGHT_YELLOW   LCD_24_BIT_RGB_COLOR(255, 255, 204)

/**
 * LCD pixel color type
 */
typedef uint32_t lcd_color_t;

#else
#   error "LCD_BITS_PER_PIXEL not defined"
#endif

/**
 * Tile width in pixels
 */
#define TILE_WIDTH          80  //40

#define NUM_TILE_ROWS       (LCD_Y_SIZE / TILE_WIDTH)
#define NUM_TILE_COLUMNS    (LCD_X_SIZE / TILE_WIDTH)

#define LCD_GET_CHAR_WIDTH_IN_PIXELS(_lcd_dot_size_in_pixels) \
        (LCD_FONT_WIDTH * (_lcd_dot_size_in_pixels))

#define LCD_GET_CHAR_HEIGHT_IN_PIXELS(_lcd_dot_size_in_pixels) \
        (LCD_FONT_HEIGHT * (_lcd_dot_size_in_pixels))

/**
 * LCD X coordinate range type
 */
typedef _RANGE_(0, LCD_X_SIZE - 1)
        uint16_t lcd_x_t;

/**
 * LCD Y coordinate range type
 */
typedef _RANGE_(0, LCD_Y_SIZE - 1)
        uint16_t lcd_y_t;

/**
 * LCD dot size in pixels range type
 */
typedef uint8_t lcd_dot_size_in_pixels_t;

/**
 * LCD character attributes
 */
struct lcd_char_attributes
{
    lcd_dot_size_in_pixels_t lcd_dot_size;
    lcd_color_t lcd_foreground_color;
    lcd_color_t lcd_background_color;
};

_NEVER_RETURN_ON_ERROR_
void init_lcd(void);

void lcd_clear(lcd_color_t color);

void lcd_draw_tile_grid(void);

lcd_color_t lcd_draw_tile(
    lcd_x_t x,
    lcd_y_t y,
    lcd_color_t fill_color);

void lcd_draw_char(
    lcd_x_t x,
    lcd_y_t y,
    const struct lcd_char_attributes *char_attributes_p,
    uint8_t char_code);

#endif /* LCD_H */
