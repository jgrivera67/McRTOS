/**
 * @file touch_screen.h
 *
 * Touch screen abstraction layer interface
 *
 * @author German Rivera 
 */ 
#ifndef TOUCH_SCREEN_H
#define TOUCH_SCREEN_H

#include <stdint.h>
#include <stdbool.h>

void init_touch_screen(void);

bool sense_touch_screen(
    uint_fast8_t tile_width,
    uint_fast8_t *tile_row_p,
    uint_fast8_t *tile_column_p);

#endif /* TOUCH_SCREEN_H */
