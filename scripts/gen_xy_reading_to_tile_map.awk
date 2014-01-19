#
# @file gen_xy_reading_to_tile_map.awk
#
# Script that generates a xy_reading_to_*_tile_map.c file
#
# Invocation syntax fom the command line:
# awk -f .\gen_xy_reading_to_tile_map.awk -v TILE_WIDTH=80 <input file> |
# out-file <output file> -encoding ascii 
#
# Invocation syntax from vim (to populate an empty file):
# :r!awk -f .\gen_xy_reading_to_tile_map.awk -v TILE_WIDTH=80 <input file full path>
# 
# @author German Rivera
#
BEGIN {
    LCD_X_SIZE = 320
    NUM_TILE_COLUMNS = (LCD_X_SIZE / TILE_WIDTH)

    if (TILE_WIDTH == "") {
        exit(1)
    }

    printf "/**\n"
    printf " * @file touch_screen_xy_reading_to_%sx%s_tile_map.c\n", TILE_WIDTH, TILE_WIDTH
    printf " *\n"
    printf " * Auto-generated mapping of (x,y) reading to tiles\n"
    printf " *\n"
    printf " * @author script gen_xy_reading_to_tile_map.awk\n"
    printf " */\n" 
    printf "\n" 
    printf "#include \"touch_screen_xy_reading_to_tile_map.h\"\n"
    printf "#include \"utils.h\"\n"
    printf "\n" 
    printf "const struct xy_reading_to_tile_entry xy_reading_to_%sx%s_tile_map[] =\n", TILE_WIDTH, TILE_WIDTH
    printf "{\n"
}
{
    tile_index = $1

    if (counters[tile_index] == "") {
        counters[tile_index] = 0
    } else if (counters[tile_index] < 8) {
        counters[tile_index] ++

        split($3, a, "=")
        x = a[2]

        split($4, a, "=")
        y = a[2]

        tile_row = int(tile_index / NUM_TILE_COLUMNS)
        tile_col = tile_index % NUM_TILE_COLUMNS
        printf("    GEN_XY_READING_TO_TILE_ENTRY(%s, %s, %s, %s),\n",
               x, y, tile_row, tile_col)
    }
}
END {
    printf "};\n";
    printf "\n" 
    printf "const uint32_t num_entries_xy_reading_to_%sx%s_tile_map =\n", TILE_WIDTH, TILE_WIDTH
    printf "    ARRAY_SIZE(xy_reading_to_%sx%s_tile_map);\n", TILE_WIDTH, TILE_WIDTH
}
