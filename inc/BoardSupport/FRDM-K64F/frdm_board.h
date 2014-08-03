/**
 * @file frdm_board.h
 *
 * Freescale Freedom (FRDM-K64F) board declarations
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
 * GPIO PORT A Pins
 */
#define FRDM_ACCELEROMETER_INT1_PIN_INDEX   6
#define FRDM_ACCELEROMETER_INT2_PIN_INDEX   13


/*
 * GPIO PORT B Pins
 */
#define FRDM_RGB_LED_RED_PIN_INDEX      22
#define FRDM_RGB_LED_BLUE_PIN_INDEX     21

/*
 * GPIO PORT E Pins
 */
#define FRDM_RGB_LED_GREEN_PIN_INDEX   26

#define LED_RED_PIN_MASK    BIT(FRDM_RGB_LED_RED_PIN_INDEX)
#define LED_GREEN_PIN_MASK  BIT(FRDM_RGB_LED_GREEN_PIN_INDEX)
#define LED_BLUE_PIN_MASK   BIT(FRDM_RGB_LED_BLUE_PIN_INDEX)

/*
 * GPIO PORT E Pins
 */
#define FRDM_I2C0_SCL_PIN_INDEX      24
#define FRDM_I2C0_SDA_PIN_INDEX      25

void frdm_board_init(void);
void frdm_board_stop(void);

void accelerometer_init(void);

void accelerometer_stop(void);

bool accelerometer_read_status(
    int16_t *x_p,
    int16_t *y_p,
    int16_t *z_p);

bool accelerometer_detect_motion(
    int8_t *x_p,
    int8_t *y_p,
    int8_t *z_p);


#endif /* __FRDM_BOARD_H */
