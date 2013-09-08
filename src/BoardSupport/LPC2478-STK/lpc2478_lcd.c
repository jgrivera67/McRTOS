/**
 * @file lcd.c
 *
 * Hardware abstraction layer for the lpc2478-stk LCD
 *
 * @author German Rivera 
 */ 

#include "lcd.h"
#include "utils.h"
#include "lpc2478_stk_board.h"
#include "hardware_abstractions.h"
#include "failure_data_capture.h"

/*
 * LCD configuration parameters
 */
#define	LCD_HORIZ_FRONT_PORCH   UINT32_C(20)
#define	LCD_HORIZ_BACK_PORCH    UINT32_C(38)
#define	LCD_HORIZ_SYNC_PULSE    UINT32_C(30)
#define	LCD_VERT_FRONT_PORCH    UINT32_C(5)
#define	LCD_VERT_BACK_PORCH     UINT32_C(15)
#define LCD_VERT_SYNC_PULSE     UINT32_C(3)
#define LCD_CLK_DIVIDER         UINT32_C(14)

#define LCD_SIZE_IN_PIXELS      (LCD_X_SIZE * LCD_Y_SIZE) 
#define LCD_SIZE_IN_CHARS       (LCD_NUM_CHAR_ROWS * LCD_NUM_CHAR_COLUMNS) 

#define LCD_PIXELS_PER_LINE_ENCODED \
                                (((LCD_X_SIZE / 16) - 1) << LCD_TIMH_PPL_SHIFT)

#define	LCD_HORIZ_FRONT_PORCH_ENCODED \
                                ((LCD_HORIZ_FRONT_PORCH - 1) << LCD_TIMH_HFP_SHIFT)

#define	LCD_HORIZ_BACK_PORCH_ENCODED \
                                ((LCD_HORIZ_BACK_PORCH - 1) << LCD_TIMH_HBP_SHIFT)

#define	LCD_HORIZ_SYNC_PULSE_ENCODED \
                                ((LCD_HORIZ_SYNC_PULSE - 1) << LCD_TIMH_HSW_SHIFT)

#define	LCD_LINES_PER_PANEL_ENCODED \
                                (LCD_Y_SIZE - 1)

#define	LCD_VERT_FRONT_PORCH_ENCODED \
                                (LCD_VERT_FRONT_PORCH << LCD_TIMV_VFP_SHIFT)

#define	LCD_VERT_BACK_PORCH_ENCODED \
                                (LCD_VERT_BACK_PORCH << LCD_TIMV_VBP_SHIFT)

#define LCD_VERT_SYNC_PULSE_ENCODED \
                                (LCD_VERT_SYNC_PULSE << LCD_TIMV_VSW_SHIFT)

#define LCD_CLOCKS_PER_LINE_ENCODED \
                                ((LCD_X_SIZE - 1) << LCD_POL_CPL_SHIFT)

#define LCD_CLK_DIVIDER_ENCODED LCD_CLK_DIVIDER

#if LCD_BITS_PER_PIXEL == 16

#   define LCD_BITS_PER_PIXEL_ENCODED \
                                    (LCD_16_BITS_PER_PIXEL << LCD_CTRL_LCDBPP_SHIFT)

#   define LCD_SET_BACKGROUND(_color) \
                                    lcd_16bpp_set_background(_color)

#   define LCD_DRAW_RECTANGLE(_x, _y, _width, _height,                  \
                              _border_thickness, _border_color,         \
                              _fill_color)                              \
    lcd_16bpp_draw_rectangle(                                           \
        _x, _y, _width, _height, _border_thickness, _border_color,      \
        _fill_color)

#   define GET_LCD_FRAME_BUFFER_PIXEL_LOCATION(_x, _y)  \
    (&g_lcd_16bpp_frame_buffer[(_y) * LCD_X_SIZE + (_x)])

#elif LCD_BITS_PER_PIXEL == 24

#   define LCD_BITS_PER_PIXEL_ENCODED \
                                    (LCD_24_BITS_PER_PIXEL << LCD_CTRL_LCDBPP_SHIFT)

#   define LCD_SET_BACKGROUND(_color) \
                                    lcd_24bpp_set_background(_color)

#   define LCD_DRAW_RECTANGLE(_x, _y, _width, _height,                  \
                              _border_thickness, _border_color,         \
                              _fill_color)                              \
    lcd_24bpp_draw_rectangle(                                           \
        _x, _y, _width, _height, _border_thickness, _border_color,      \
        _fill_color)

#   define GET_LCD_FRAME_BUFFER_PIXEL_LOCATION(_x, _y)  \
    (&g_lcd_24bpp_frame_buffer[(_y) * LCD_X_SIZE + (_x)])

#else
#   error "LCD_BITS_PER_PIXEL not defined"
#endif


#define LCD_DRAW_TILE(_x, _y, _fill_color) \
    LCD_DRAW_RECTANGLE(                                                 \
        _x, _y,                                                         \
        TILE_WIDTH, TILE_WIDTH, 2,                                      \
        TILE_BORDER_COLOR, _fill_color)

#define LCD_DRAW_DOT(_x, _y, _size_in_pixels, _color) \
    LCD_DRAW_RECTANGLE(                                                 \
        _x, _y,                                                         \
        _size_in_pixels, _size_in_pixels, 0,                            \
        0, _color)

#define TILE_BORDER_COLOR   LCD_COLOR_BLACK
#define TILE_FILL_COLOR     LCD_COLOR_BLUE

C_ASSERT(LCD_HORIZ_FRONT_PORCH_ENCODED <= LCD_TIMH_HFP_MASK);
C_ASSERT(LCD_HORIZ_BACK_PORCH_ENCODED <= LCD_TIMH_HBP_MASK);
C_ASSERT(LCD_HORIZ_SYNC_PULSE_ENCODED <= LCD_TIMH_HSW_MASK);
C_ASSERT(LCD_PIXELS_PER_LINE_ENCODED <= LCD_TIMH_PPL_MASK);
C_ASSERT(LCD_LINES_PER_PANEL_ENCODED <= LCD_TIMV_LPP_MASK);
C_ASSERT(LCD_VERT_FRONT_PORCH_ENCODED <= LCD_TIMV_VFP_MASK);
C_ASSERT(LCD_VERT_SYNC_PULSE_ENCODED <= LCD_TIMV_VSW_MASK);
C_ASSERT(LCD_CLOCKS_PER_LINE_ENCODED <= LCD_POL_CPL_MASK);
C_ASSERT(LCD_CLK_DIVIDER_ENCODED <= LCD_CLKDIV_MASK);
C_ASSERT(LCD_BITS_PER_PIXEL_ENCODED <= LCD_CTRL_LCDBPP_MASK);

/**
 * Layout of a 24-bit rgb pixel (assuming little endianess) in a local variable
 * (not in the LCD frame buffer)
 */
typedef volatile struct rgb_24bit_pixel
{
    uint8_t rgb_red_component;
    uint8_t rgb_green_component;
    uint8_t rgb_blue_component;
} __attribute__((__packed__)) rgb_24bit_pixel_t;

C_ASSERT(sizeof(rgb_24bit_pixel_t) == 3);
C_ASSERT(offsetof(rgb_24bit_pixel_t, rgb_red_component) == 0x0);
C_ASSERT(offsetof(rgb_24bit_pixel_t, rgb_green_component) == 0x1);
C_ASSERT(offsetof(rgb_24bit_pixel_t, rgb_blue_component) == 0x2);

/**
 * 16-bit rgb pixel
 */
typedef volatile uint16_t rgb_16bit_pixel_t;

/**
 * Memory-mapped I/O registers of the LPC2478 LCD controller
 */
static lpc2478_lcd_controller_t *const g_lpc2478_lcd_controller =
    (lpc2478_lcd_controller_t *)LPC2478_LCD_CONTROLLER_BASE_ADDR;

/**
 * LCD RAM frame buffer pointers
 */

static volatile rgb_24bit_pixel_t *const g_lcd_24bpp_frame_buffer =
        (rgb_24bit_pixel_t *)LCD_FRAME_BUFFER_BASE_ADDR;

static volatile rgb_16bit_pixel_t *const g_lcd_16bpp_frame_buffer =
        (rgb_16bit_pixel_t *)LCD_FRAME_BUFFER_BASE_ADDR;


/**
 * Current color of each tile in the tile grid
 */
static lcd_color_t current_tile_colors[NUM_TILE_ROWS][NUM_TILE_COLUMNS];

/**
 * Initialize the LCD controller
 */        
void
init_lcd(void)
{
    uint32_t reg_value;
    fdc_error_t fdc_error;

    /*
     * Check DRAM block to be used for the LCD frame buffer, before using it:
     */
    fdc_error = check_dram_memory_block
                    ((void *)LCD_FRAME_BUFFER_BASE_ADDR, LCD_FRAME_BUFFER_SIZE);
    if (fdc_error != 0)
    {
        fatal_error_handler(fdc_error);
    }

    reg_value = read_32bit_mmio_register(
                    &g_pin_connect_block->reg_PINSEL[0]);
    reg_value &= 0xfffc00ff;  //BIN32(11111111,11111100,00000000,11111111);
    reg_value |= 0x00015500;  //BIN32(00000000,00000001,01010101,00000000);
    write_32bit_mmio_register(
        &g_pin_connect_block->reg_PINSEL[0], reg_value);

    reg_value = read_32bit_mmio_register(
                    &g_pin_connect_block->reg_PINMODE[0]);
    reg_value &= 0xfffc00ff; //BIN32(11111111,11111100,00000000,11111111);
    reg_value |= 0x0002aa00; //BIN32(00000000,00000010,10101010,00000000);
    write_32bit_mmio_register(
        &g_pin_connect_block->reg_PINMODE[0], reg_value);

    reg_value = read_32bit_mmio_register(
                    &g_pin_connect_block->reg_PINSEL[3]);
    reg_value &= 0xf00000ff; //BIN32(11110000,00000000,00000000,11111111);
    reg_value |= 0x05555500; //BIN32(00000101,01010101,01010101,00000000);
    write_32bit_mmio_register(
        &g_pin_connect_block->reg_PINSEL[3], reg_value);

    reg_value = read_32bit_mmio_register(
                    &g_pin_connect_block->reg_PINMODE[3]);
    reg_value &= 0xf00000ff; //BIN32(11110000,00000000,00000000,11111111);
    reg_value |= 0x0aaaaa00; //BIN32(00001010,10101010,10101010,00000000);
    write_32bit_mmio_register(
        &g_pin_connect_block->reg_PINMODE[3], reg_value);

    reg_value = read_32bit_mmio_register(
                    &g_pin_connect_block->reg_PINSEL[4]);
    reg_value &= 0xf0300000; //BIN32(11110000,00110000,00000000,00000000);
    reg_value |= 0x058fffff; //BIN32(00000101,01001111,11111111,11111111);
    write_32bit_mmio_register(
        &g_pin_connect_block->reg_PINSEL[4], reg_value);

    reg_value = read_32bit_mmio_register(
                    &g_pin_connect_block->reg_PINMODE[4]);
    reg_value &= 0xf0300000; //BIN32(11110000,00110000,00000000,00000000);
    reg_value |= 0x0a8aaaaa; //BIN32(00001010,10001010,10101010,10101010);
    write_32bit_mmio_register(
        &g_pin_connect_block->reg_PINMODE[4], reg_value);

    reg_value = read_32bit_mmio_register(
                    &g_pin_connect_block->reg_PINSEL[9]);
    reg_value &= 0xf0ffffff; //BIN32(11110000,11111111,11111111,11111111);
    reg_value |= 0x0a000000; //BIN32(00001010,00000000,00000000,00000000);
    write_32bit_mmio_register(
        &g_pin_connect_block->reg_PINSEL[9], reg_value);

    reg_value = read_32bit_mmio_register(
                    &g_pin_connect_block->reg_PINMODE[9]);
    reg_value &= 0xf0ffffff; //BIN32(11110000,11111111,11111111,11111111);
    reg_value |= 0x0a000000; //BIN32(00001010,00000000,00000000,00000000);
    write_32bit_mmio_register(
        &g_pin_connect_block->reg_PINMODE[9], reg_value);

    reg_value = read_32bit_mmio_register(
                    &g_pin_connect_block->reg_PINSEL[11]);
    reg_value &= 0xfffffff0; //BIN32(11111111,11111111,11111111,11110000);
    reg_value |= 0x0000000f; //BIN32(00000000,00000000,00000000,00001111);
    write_32bit_mmio_register(
        &g_pin_connect_block->reg_PINSEL[11], reg_value);

    /*
     * Turn on LCD power in the System Control Block
     */
    reg_value = read_32bit_mmio_register(
                    &g_scb_mmio_registers_p->reg_PCONP);
    reg_value |= PCONP_PCLCD;
    write_32bit_mmio_register(
        &g_scb_mmio_registers_p->reg_PCONP, reg_value);

    write_32bit_mmio_register(
        &g_lpc2478_lcd_controller->reg_LCD_CFG, LCD_CLK_DIVIDER_ENCODED);

    write_32bit_mmio_register(
        &g_lpc2478_lcd_controller->reg_LCD_TIMH,
        LCD_HORIZ_BACK_PORCH_ENCODED |
        LCD_HORIZ_FRONT_PORCH_ENCODED |
        LCD_HORIZ_SYNC_PULSE_ENCODED |
        LCD_PIXELS_PER_LINE_ENCODED);

    write_32bit_mmio_register(
        &g_lpc2478_lcd_controller->reg_LCD_TIMV,
        LCD_VERT_BACK_PORCH_ENCODED |
        LCD_VERT_FRONT_PORCH_ENCODED |
        LCD_VERT_SYNC_PULSE_ENCODED |
        LCD_VERT_SYNC_PULSE_ENCODED |
        LCD_LINES_PER_PANEL_ENCODED);

    write_32bit_mmio_register(
        &g_lpc2478_lcd_controller->reg_LCD_POL,
        LCD_POL_BCD_MASK |
        LCD_CLOCKS_PER_LINE_ENCODED |
        LCD_POL_IPC_MASK |
        LCD_POL_IHS_MASK |
        LCD_POL_IVS_MASK);
			
    write_32bit_mmio_register(
        &g_lpc2478_lcd_controller->reg_LCD_LE, 0);

    write_32bit_mmio_register(
        &g_lpc2478_lcd_controller->reg_LCD_UPBASE, LCD_FRAME_BUFFER_BASE_ADDR);
    write_32bit_mmio_register(
        &g_lpc2478_lcd_controller->reg_LCD_LPBASE, LCD_FRAME_BUFFER_BASE_ADDR);

    write_32bit_mmio_register(
        &g_lpc2478_lcd_controller->reg_LCD_CTRL,
        LCD_CTRL_LCDPWR_MASK |
        LCD_CTRL_LCDTFT_MASK |
        LCD_BITS_PER_PIXEL_ENCODED);

    /*
     * XXX Do we need a delay loop before enabling the LCD?
     */
#if 0
    for (int i = 0; i < 10000; i ++)
        ;
#endif

    /*
     * Enable LCD
     */
    reg_value = read_32bit_mmio_register(
                    &g_lpc2478_lcd_controller->reg_LCD_CTRL);
    reg_value |= LCD_CTRL_LCDEN_MASK;

    CLEAR_BIT_FIELD(reg_value, LCD_CTRL_RESERVED1_MASK);
    CLEAR_BIT_FIELD(reg_value, LCD_CTRL_RESERVED2_MASK);

    write_32bit_mmio_register(
        &g_lpc2478_lcd_controller->reg_LCD_CTRL, reg_value);

    lcd_clear(LCD_COLOR_BLACK);
}


static inline void
lcd_24bpp_set_background(lcd_color_t lcd_background_color)
{
    uint8_t *const rgb_color_components = (uint8_t *)&lcd_background_color;
    rgb_24bit_pixel_t *const frame_buffer_end = g_lcd_24bpp_frame_buffer + LCD_SIZE_IN_PIXELS;

    FDC_ASSERT(rgb_color_components[3] == 0,
        rgb_color_components[3], lcd_background_color);

    for (rgb_24bit_pixel_t *frame_buffer_pixel = g_lcd_24bpp_frame_buffer;
         frame_buffer_pixel != frame_buffer_end;
         frame_buffer_pixel ++)
    {
        frame_buffer_pixel->rgb_red_component = rgb_color_components[0];
        frame_buffer_pixel->rgb_green_component = rgb_color_components[1];
        frame_buffer_pixel->rgb_blue_component = rgb_color_components[2];
    }
}


static inline void
lcd_16bpp_set_background(lcd_color_t lcd_background_color)
{
    rgb_16bit_pixel_t *const frame_buffer_end = g_lcd_16bpp_frame_buffer + LCD_SIZE_IN_PIXELS;

    for (rgb_16bit_pixel_t *frame_buffer_pixel = g_lcd_16bpp_frame_buffer;
         frame_buffer_pixel != frame_buffer_end;
         frame_buffer_pixel ++)
    {
        *frame_buffer_pixel = lcd_background_color;
    }
}


void
lcd_clear(lcd_color_t color)
{
    LCD_SET_BACKGROUND(color);
}


static void
lcd_16bpp_draw_rectangle(
    lcd_x_t x,
    lcd_y_t y,
    uint16_t width,
    uint16_t height,
    uint8_t border_thickness, // could be 0
    lcd_color_t border_color,
    lcd_color_t fill_color)
{
    FDC_ASSERT(width != 0 && height != 0, width, height);
    FDC_ASSERT(border_thickness <= width, border_thickness, width);
    FDC_ASSERT(border_thickness <= height, border_thickness, height);

    uint16_t pixel_x_end = x + width;
    uint16_t pixel_y_end = y + border_thickness;

    FDC_ASSERT(pixel_x_end <= LCD_X_SIZE, pixel_x_end, LCD_X_SIZE);
    FDC_ASSERT(pixel_y_end <= LCD_Y_SIZE, pixel_y_end, LCD_Y_SIZE);

    uint16_t pixel_y;

    /* 
     * Draw top border
     */
    for (pixel_y = y; pixel_y < pixel_y_end; pixel_y ++)
    {
        rgb_16bit_pixel_t *frame_buffer_pixel =
            GET_LCD_FRAME_BUFFER_PIXEL_LOCATION(x, pixel_y);

        for (uint16_t pixel_x = x; pixel_x < pixel_x_end; pixel_x ++)
        {
            *frame_buffer_pixel ++ = border_color;
        }
    }

    /*
     * Draw middle part
     */

    pixel_y_end = y + height - border_thickness;

    FDC_ASSERT(pixel_y_end <= LCD_Y_SIZE, pixel_y_end, LCD_Y_SIZE);

    for ( ; pixel_y < pixel_y_end; pixel_y ++)
    {
        uint16_t pixel_x;
        rgb_16bit_pixel_t *frame_buffer_pixel =
            GET_LCD_FRAME_BUFFER_PIXEL_LOCATION(x, pixel_y);

        pixel_x_end = x + border_thickness;
        for (pixel_x = x; pixel_x < pixel_x_end; pixel_x ++)
        {
            *frame_buffer_pixel ++ = border_color;
        }
            
        pixel_x_end = x + width - border_thickness;
        for ( ; pixel_x < pixel_x_end; pixel_x ++)
        {
            *frame_buffer_pixel ++ = fill_color;
        }

        pixel_x_end = x + width;
        for ( ; pixel_x < pixel_x_end; pixel_x ++)
        {
            *frame_buffer_pixel ++ = border_color;
        }
    }

    /* 
     * Draw bottom border
     */

    pixel_x_end = x + width;
    pixel_y_end = y + height;
    
    FDC_ASSERT(pixel_y_end <= LCD_Y_SIZE, pixel_y_end, LCD_Y_SIZE);

    for ( ; pixel_y < pixel_y_end; pixel_y ++)
    {
        rgb_16bit_pixel_t *frame_buffer_pixel =
            GET_LCD_FRAME_BUFFER_PIXEL_LOCATION(x, pixel_y);

        for (uint16_t pixel_x = x; pixel_x < pixel_x_end; pixel_x ++)
        {
            *frame_buffer_pixel ++ = border_color;
        }
    }
}


lcd_color_t
lcd_draw_tile(
    lcd_x_t x,
    lcd_y_t y,
    lcd_color_t fill_color)
{
    uint16_t row = y / TILE_WIDTH;
    uint16_t col = x / TILE_WIDTH;
    
    FDC_ASSERT(row < NUM_TILE_ROWS, row, NUM_TILE_ROWS);
    FDC_ASSERT(col < NUM_TILE_COLUMNS, col, NUM_TILE_COLUMNS);

    lcd_color_t *current_tile_color_p = &current_tile_colors[row][col];
    lcd_color_t old_color = *current_tile_color_p;

    LCD_DRAW_TILE(x, y, fill_color);
    *current_tile_color_p = fill_color;
    return old_color;
}


void
lcd_draw_tile_grid(void)
{
    lcd_y_t y = 0;

    for (uint16_t row = 0; row < NUM_TILE_ROWS; row ++)
    {
        lcd_x_t x = 0;

        for (uint16_t column = 0; column < NUM_TILE_COLUMNS; column ++)
        {
            LCD_DRAW_TILE(x, y, TILE_FILL_COLOR);
            current_tile_colors[row][column] = TILE_FILL_COLOR;

            x += TILE_WIDTH;
        }

        y += TILE_WIDTH;
    }
}


void
lcd_draw_char(
    lcd_x_t x,
    lcd_y_t y,
    const struct lcd_char_attributes *char_attributes_p,
    uint8_t char_code)
{
    FDC_ASSERT_VALID_RAM_OR_ROM_POINTER(char_attributes_p, sizeof(uint32_t));

    lcd_dot_size_in_pixels_t dot_size_in_pixels = char_attributes_p->lcd_dot_size;
    lcd_color_t foreground_color = char_attributes_p->lcd_foreground_color;
    lcd_color_t background_color = char_attributes_p->lcd_background_color;
    uint16_t pixel_y_end = y + LCD_FONT_HEIGHT*dot_size_in_pixels;
    uint16_t pixel_x_end = x + LCD_FONT_WIDTH*dot_size_in_pixels;

    FDC_ASSERT(pixel_x_end <= LCD_X_SIZE, pixel_x_end, LCD_X_SIZE);
    FDC_ASSERT(pixel_y_end <= LCD_Y_SIZE, pixel_y_end, LCD_Y_SIZE);

    uint8_t font_bit_row_index = 0;
    for (uint16_t pixel_y = y; pixel_y != pixel_y_end; pixel_y += dot_size_in_pixels)
    {
        FDC_ASSERT(font_bit_row_index < LCD_FONT_HEIGHT, font_bit_row_index, LCD_FONT_HEIGHT);

        uint8_t font_bit_row = lcd_fonts[char_code][font_bit_row_index];
        uint8_t font_bit_column_mask = BIT(LCD_FONT_WIDTH - 1);
        for (uint16_t pixel_x = x; pixel_x != pixel_x_end; pixel_x += dot_size_in_pixels)
        {
            FDC_ASSERT(font_bit_column_mask != 0, 0, 0);

            if (font_bit_row & font_bit_column_mask)
            {
                LCD_DRAW_DOT(pixel_x, pixel_y, dot_size_in_pixels, foreground_color);
            }
            else
            {
                LCD_DRAW_DOT(pixel_x, pixel_y, dot_size_in_pixels, background_color);
            }

            font_bit_column_mask >>= 1;
        }

        font_bit_row_index ++;
    }
}


#if 0
void
lcd_print_string(
    lcd_x_t x,
    lcd_y_t y,
    const char *string,
    lcd_color_t foreground_color,
    lcd_color_t background_color)
{

#   define MAX_STRING_LENGTH        256
#   define CHAR_WIDTH_IN_PIXELS     ((LCD_FONT_WIDTH + 1) * LCD_FONT_DOT_SIZE_IN_PIXELS)
    size_t length;
    size_t length_in_pixels;
    lcd_x_t x_cursor;

    /*
     * Calculate length of NULL terminated string
     */
    for (length = 0; length < MAX_STRING_LENGTH && string[length] != '\0'; length ++)
        ;

    length_in_pixels =  length * CHAR_WIDTH_IN_PIXELS;

    /*
     * Truncate string if necessary
     */
    if (x + length_in_pixels > LCD_X_SIZE)
    {
        length_in_pixels -= (x + length_in_pixels) - LCD_X_SIZE;
        length = length_in_pixels / CHAR_WIDTH_IN_PIXELS;
    }

    x_cursor = x;
    for (size_t i = 0; i < length; i ++)
    {
        lcd_print_char(
            x_cursor,
            y,
            (uint8_t)string[i],
            foreground_color,
            background_color);

        x_cursor += CHAR_WIDTH_IN_PIXELS;
    }
}
#endif
