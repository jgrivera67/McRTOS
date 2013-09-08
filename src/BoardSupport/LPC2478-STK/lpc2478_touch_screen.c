/**
 * @file touch_screen.c
 *
 * Touch screen abstraction layer implemented for the lpc2478-stk board
 *
 * @author German Rivera 
 */ 

#include "touch_screen.h"
#include "utils.h"
#include "lpc2478_stk_board.h"
#include "hardware_abstractions.h"
#include "failure_data_capture.h"
#include "touch_screen_xy_reading_to_tile_map.h"
#include "McRTOS_kernel_services.h"

static const struct pin_config_info g_pin_X1 =
    PIN_COFIG_INFO_INITIALIZER(
        TOUCH_SCREEN_PINS_GPIO_PORT, TOUCH_SCREEN_X1_PIN_BIT_INDEX, PINSEL_PRIMARY, true);

static const struct pin_config_info g_pin_X2 =
    PIN_COFIG_INFO_INITIALIZER(
        TOUCH_SCREEN_PINS_GPIO_PORT, TOUCH_SCREEN_X2_PIN_BIT_INDEX, PINSEL_PRIMARY, true);

static const struct pin_config_info g_pin_Y1 =
    PIN_COFIG_INFO_INITIALIZER(
        TOUCH_SCREEN_PINS_GPIO_PORT, TOUCH_SCREEN_Y1_PIN_BIT_INDEX, PINSEL_PRIMARY, true);

static const struct pin_config_info g_pin_Y2 =
    PIN_COFIG_INFO_INITIALIZER(
        TOUCH_SCREEN_PINS_GPIO_PORT, TOUCH_SCREEN_Y2_PIN_BIT_INDEX, PINSEL_PRIMARY, true);

void
init_touch_screen(void)
{
    /*
     * Configure X1, X2, Y1 and Y2 output pins:
     */
    configure_pin(&g_pin_X1, true);
    configure_pin(&g_pin_X2, true);
    configure_pin(&g_pin_Y1, true);
    configure_pin(&g_pin_Y2, true);
}


static void 
select_X1_X2_output_pins(void)
{
    /*
     * Re-configure X1 and X2 as output pins:
     */
    configure_pin(&g_pin_X1, true);
    configure_pin(&g_pin_X2, true);

    /*
     * Re-configure Y1 and Y2 as input pins:
     */
    configure_pin(&g_pin_Y1, false);
//#if 0
    configure_pin(&g_pin_Y2, false);
//#endif
}


static void 
touch_screen_polarize_X1_X2(void)
{
    select_X1_X2_output_pins();

    /*
     * Drive output to pin P0.22 (X1) high
     */
    activate_output_pin(&g_pin_X1);

    /*
     * Drive output to pin P0.24 (X2) low
     */
    deactivate_output_pin(&g_pin_X2);
}


static void 
touch_screen_polarize_X2_X1(void)
{
    select_X1_X2_output_pins();

    /*
     * Drive output to pin P0.22 (X1) low
     */
    deactivate_output_pin(&g_pin_X1);

    /*
     * Drive output to pin P0.24 (X2) high
     */
    activate_output_pin(&g_pin_X2);
}


static void 
select_Y1_Y2_output_pins(void)
{
    /*
     * Re-configure Y1 and Y2 as output pins:
     */
    configure_pin(&g_pin_Y1, true);
    configure_pin(&g_pin_Y2, true);

    /*
     * Re-configure X1 and X2 as input pins:
     */
    configure_pin(&g_pin_X1, false);
//#if 0
    configure_pin(&g_pin_X2, false);
//#endif
}


static void 
touch_screen_polarize_Y1_Y2(void)
{
    select_Y1_Y2_output_pins();

    /*
     * Drive output to pin P0.23 (Y1) high
     */
    activate_output_pin(&g_pin_Y1);

    /*
     * Drive output to pin P0.21 (Y2) low
     */
    deactivate_output_pin(&g_pin_Y2);
}


static void
touch_screen_polarize_Y2_Y1(void)
{
    select_Y1_Y2_output_pins();

    /*
     * Drive output to pin P0.23 (Y1) low
     */
    deactivate_output_pin(&g_pin_Y1);

    /*
     * Drive output to pin P0.21 (Y2) high
     */
    activate_output_pin(&g_pin_Y2);
}


#if 0 // XXX
static bool
adjust_xy_readings(
    uint_fast16_t x_reading,
    uint_fast16_t y_reading,
    uint_fast16_t *x_result_p,
    uint_fast16_t *y_result_p)
{
    uint_fast16_t tile_col;
    uint_fast16_t tile_row;

    if (x_reading >= 501 && x_reading <= 720)
    {
        tile_col = 0;
    }
    else if (x_reading >= 721 && x_reading <= 850)
    {
        tile_col = 1;
    }
    else if (x_reading >= 851 && x_reading <= 950)
    {
        tile_col = 2;
    }
    else if (x_reading >= 951 && x_reading <= 1021)
    {
        tile_col = 3;
    }
    else
    {
        return false;
    }

    if (y_reading >= 2 && y_reading <= 199)
    {
        tile_row = 2;
    }
    else if (y_reading >= 200 && y_reading <= 349)
    {
        tile_row = 1;
    }
    else if (y_reading >= 350 && y_reading <= 600)
    {
        tile_row = 0;
    }
    else
    {
        return false;
    }

    *x_result_p = tile_col * TILE_WIDTH;
    *y_result_p = tile_row * TILE_WIDTH;
    return true;
}
#endif // XXX


static void
map_xy_reading_to_tile(
    uint_fast16_t x_reading,
    uint_fast16_t y_reading,
    uint_fast8_t tile_width,
    uint_fast8_t *tile_row_p,
    uint_fast8_t *tile_column_p)
{
    const struct xy_reading_to_tile_entry *map_p;
    const struct xy_reading_to_tile_entry *end_map_p;

#if 0
    FDC_ASSERT(tile_width == 80 || tile_width == 40, tile_width, 0);

    if (tile_width == 80)
    {
        map_p = xy_reading_to_80x80_tile_map;
        end_map_p = 
            xy_reading_to_80x80_tile_map + num_entries_xy_reading_to_80x80_tile_map;
    }
    else if (tile_width == 40)
    {
        map_p = xy_reading_to_40x40_tile_map;
        end_map_p = 
            xy_reading_to_40x40_tile_map + num_entries_xy_reading_to_40x40_tile_map;
    }
#else
    FDC_ASSERT(tile_width == 80, tile_width, 0);

    map_p = xy_reading_to_80x80_tile_map;
    end_map_p = 
        xy_reading_to_80x80_tile_map + num_entries_xy_reading_to_80x80_tile_map;
#endif

    const struct xy_reading_to_tile_entry *closest_entry_p = NULL;
    uint_fast32_t min_square_distance = UINT32_MAX;
    uint_fast32_t square_distance;
    int32_t x_delta;
    int32_t y_delta;
    
    for (const struct xy_reading_to_tile_entry *entry_p = map_p;
         entry_p != end_map_p;
         entry_p ++)
    {
        x_delta = (int32_t)x_reading - (int32_t)entry_p->x_reading;
        x_delta *= x_delta;
        FDC_ASSERT(x_delta >= 0, x_delta, x_reading);

        y_delta = (int32_t)y_reading - (int32_t)entry_p->y_reading;
        y_delta *= y_delta;
        FDC_ASSERT(y_delta >= 0, y_delta, y_reading);

        square_distance = x_delta + y_delta;
        if (square_distance < min_square_distance)
        {
            min_square_distance = square_distance;
            closest_entry_p = entry_p;
            if (min_square_distance == 0)
            {
                break;
            }
        }
    }

    FDC_ASSERT(closest_entry_p != NULL, 0, 0);

    *tile_row_p = closest_entry_p->tile_row;
    *tile_column_p = closest_entry_p->tile_column;
}


/**
 * Returns true if the touchscreen was touched
 */
bool 
sense_touch_screen(
    uint_fast8_t tile_width,
    uint_fast8_t *tile_row_p,
    uint_fast8_t *tile_column_p)
{
#   define XDELTA_MAX	    16 //32 //100
#   define YDELTA_MAX	    12 //24 //100
#   define NUM_SAMPLES	    4 //4 //2
#   define RESOLUTION_BITS  10
 
#   define SETTLING_DELAY() delay_loop(10000)
    /*
     * Approx. 50000 CPU clock cycles. If The CPU clock frequency is 72MHz,
     * there are 72000 CPU clock cycles in 1 ms. So the settling delay is less
     * than 1 ms.
     */

    uint_fast16_t x_samples[NUM_SAMPLES*2];
    uint_fast16_t y_samples[NUM_SAMPLES*2];
    uint_fast16_t x_reading;
    uint_fast16_t y_reading;
    uint_fast8_t i;
    uint_fast16_t yvalue_min = 1023;
    uint_fast16_t yvalue_max = 0;
    uint_fast16_t xvalue_min = 1023;
    uint_fast16_t xvalue_max = 0;
    uint_fast16_t xdelta, ydelta;
    uint_fast16_t ts_max_value = 2 <<(RESOLUTION_BITS-1);

    for (i = 0; i < NUM_SAMPLES; i++)
    {
        /*
         * Power the "x-axis" layer of the touch panel, and use the "y-axis"
         * layer to measure voltage corresponding to the x-axis position:
         */

        touch_screen_polarize_X1_X2();
        select_input_pin_adc_channel(TOUCH_SCREEN_Y_ADC_CHANNEL);
        SETTLING_DELAY();

        x_samples[i*2] =
            read_adc_channel(g_adc_device_p, TOUCH_SCREEN_Y_ADC_CHANNEL);
        
        /*
         * Power the "y-axis" layer of the touch panel, and use the "x-axis"
         * layer to measure voltage corresponding to the y-axis position:
         */

        touch_screen_polarize_Y1_Y2();
        select_input_pin_adc_channel(TOUCH_SCREEN_X_ADC_CHANNEL);
        SETTLING_DELAY();

        y_samples[i*2] = 
            read_adc_channel(g_adc_device_p, TOUCH_SCREEN_X_ADC_CHANNEL);
        
        /*
         * Power the "x-axis" layer of the touch panel, and use the "y-axis"
         * layer to measure voltage corresponding to the x-axis position:
         */

        touch_screen_polarize_X2_X1();
        select_input_pin_adc_channel(TOUCH_SCREEN_Y_ADC_CHANNEL);
        SETTLING_DELAY();

        x_samples[(i*2)+1] = 
            read_adc_channel(g_adc_device_p, TOUCH_SCREEN_Y_ADC_CHANNEL);
        
        /*
         * Power the "y-axis" layer of the touch panel, and use the "x-axis"
         * layer to measure voltage corresponding to the y-axis position:
         */

        touch_screen_polarize_Y2_Y1();
        select_input_pin_adc_channel(TOUCH_SCREEN_X_ADC_CHANNEL);
        SETTLING_DELAY();

        y_samples[(i*2)+1] = 
            read_adc_channel(g_adc_device_p, TOUCH_SCREEN_X_ADC_CHANNEL);
    }
	
    x_reading = 0;
    y_reading = 0;
		
    for (i = 0; i < NUM_SAMPLES; i ++)
    {
        uint_fast16_t tempyval = ts_max_value - y_samples[i*2+1];
        uint_fast16_t tempxval = ts_max_value - x_samples[i*2+1];
        
        x_reading += x_samples[i*2];
        y_reading += y_samples[i*2];
            
        x_reading += (ts_max_value - x_samples[(i*2)+1]);
        y_reading += (ts_max_value - y_samples[(i*2)+1]);
        
        
        if (yvalue_min > y_samples[i*2])
        {
            yvalue_min = y_samples[i*2];
        }

        if (yvalue_max < y_samples[i*2])
        {
            yvalue_max = y_samples[i*2];
        }

        if (yvalue_min > tempyval)
        {
            yvalue_min = tempyval;
        }

        if (yvalue_min < tempyval)
        {
            yvalue_max = tempyval;
        }
        
        if (xvalue_min > x_samples[i*2])
        {
            xvalue_min = x_samples[i*2];
        }

        if (xvalue_max < x_samples[i*2])
        {
            xvalue_max = x_samples[i*2];
        }
        
        if (xvalue_min > tempxval)
        {
            xvalue_min = tempxval;
        }

        if (xvalue_min < tempxval)
        {
            xvalue_max = tempxval;
        }
    }
	
    x_reading /= NUM_SAMPLES*2;
    y_reading /= NUM_SAMPLES*2;

    xdelta = xvalue_max - xvalue_min;
    ydelta = yvalue_max - yvalue_min;
    
    if (x_reading < 1022 && y_reading > 1 &&
        xdelta < XDELTA_MAX && ydelta < YDELTA_MAX)
    {
#if 0 // XXX
        extern volatile uint32_t tile_cursor_index;
        extern volatile uint32_t same_tile_count;

        console_printf(
            "tile_cursor_index: %u, same_tile_count: %u, x_reading: %u, y_reading: %u\n",
            tile_cursor_index, same_tile_count, x_reading, y_reading);

        ATOMIC_POST_INCREMENT_UINT32(&same_tile_count);
#endif // XXX

        map_xy_reading_to_tile(
            x_reading, y_reading, tile_width, tile_row_p, tile_column_p);

        return true;
    }
    
    return false;
}


