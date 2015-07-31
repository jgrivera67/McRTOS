/**
 * @file touch_screen_xy_reading_to_tile_map.h
 *
 * Mapping of touch screen (x,y) readings to tiles
 *
 * @author German Rivera 
 */ 
#ifndef TOUCH_SCREEN_XY_READING_TO_TILE_MAP_H
#define TOUCH_SCREEN_XY_READING_TO_TILE_MAP_H

#include <stdint.h>

#define GEN_XY_READING_TO_TILE_ENTRY(                                   \
            _x_reading, _y_reading,                                     \
            _tile_row, _tile_column)                                    \
    {                                                                   \
        .x_reading = _x_reading,                                        \
        .y_reading = _y_reading,                                        \
        .tile_row = _tile_row,                                          \
        .tile_column = _tile_column                                     \
    }

struct xy_reading_to_tile_entry
{
    uint16_t x_reading;
    uint16_t y_reading;
    uint8_t tile_row;
    uint8_t tile_column;
};

extern const struct xy_reading_to_tile_entry xy_reading_to_80x80_tile_map[];
extern const uint32_t num_entries_xy_reading_to_80x80_tile_map;

#if 0
extern const struct xy_reading_to_tile_entry xy_reading_to_40x40_tile_map[];
extern const uint32_t num_entries_xy_reading_to_40x40_tile_map;
#endif

#endif /* TOUCH_SCREEN_XY_READING_TO_TILE_MAP_H */
