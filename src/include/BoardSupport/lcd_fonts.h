#ifndef _LCD_FONTS_
#define _LCD_FONTS_

#include <stdint.h>

#define LCD_FONT_CHARS	256
#define LCD_FONT_WIDTH	8
#define LCD_FONT_HEIGHT	16

typedef uint8_t lcd_font_bit_matrix_t[LCD_FONT_HEIGHT];

extern const lcd_font_bit_matrix_t lcd_fonts[LCD_FONT_CHARS];

#endif /*  _LCD_FONTS_ */
