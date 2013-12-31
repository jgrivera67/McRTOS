/**
 * @file frdm_board.h
 *
 * Freescale Freedom (FRDM) board declarations
 *
 * @author German Rivera 
 */ 
#ifndef __FRDM_BOARD_H
#define __FRDM_BOARD_H

#include <stdint.h>

/**
 * Indexes in g_frdm_rgb_led_pins[] 
 */
enum {
    FRDM_RED_LED =      0x0,
    FRDM_GREEN_LED =    0x1,
    FRDM_BLUE_LED =     0x2,
    FRDM_NUM_RGB_LED_PINS
};

/*
 * KL25 GPIO PORT A Pins
 */
#define FRDM_ACCELEROMETER_INT1_PIN_INDEX   14
#define FRDM_ACCELEROMETER_INT2_PIN_INDEX   15


/*
 * KL25 GPIO PORT B Pins
 */
#define FRDM_RGB_LED_RED_PIN_INDEX      18
#define FRDM_RGB_LED_GREEN_PIN_INDEX    19

/*
 * KL25 GPIO PORT D Pins
 */
#define FRDM_RGB_LED_BLUE_PIN_INDEX     1

#define LED_RED_PIN_MASK    BIT(FRDM_RGB_LED_RED_PIN_INDEX)
#define LED_GREEN_PIN_MASK  BIT(FRDM_RGB_LED_GREEN_PIN_INDEX)
#define LED_BLUE_PIN_MASK   BIT(FRDM_RGB_LED_BLUE_PIN_INDEX)

#define LED_COLOR_BLACK     UINT32_C(0x0)
#define LED_COLOR_RED       LED_RED_PIN_MASK
#define LED_COLOR_GREEN     LED_GREEN_PIN_MASK
#define LED_COLOR_YELLOW    (LED_RED_PIN_MASK | LED_GREEN_PIN_MASK)
#define LED_COLOR_BLUE      LED_BLUE_PIN_MASK
#define LED_COLOR_MAGENTA   (LED_RED_PIN_MASK | LED_BLUE_PIN_MASK)
#define LED_COLOR_CYAN      (LED_GREEN_PIN_MASK | LED_BLUE_PIN_MASK)
#define LED_COLOR_WHITE     (LED_RED_PIN_MASK | LED_GREEN_PIN_MASK | LED_BLUE_PIN_MASK)

/*
 * KL25 GPIO PORT E Pins
 */
#define FRDM_I2C0_SCL_PIN_INDEX      24
#define FRDM_I2C0_SDA_PIN_INDEX      25

void frdm_board_init(void);
void frdm_board_stop(void);

void toggle_rgb_led(uint32_t led_color_mask);
void turn_on_rgb_led(uint32_t led_color_mask);
void turn_off_rgb_led(uint32_t led_color_mask);
uint32_t set_rgb_led_color(uint32_t led_color_mask);

void accelerometer_init(void);
void accelerometer_stop(void);
bool accelerometer_read(
    int16_t *x_p,
    int16_t *y_p,
    int16_t *z_p);

#endif /* __FRDM_BOARD_H */
