/**
 * @file lpcxpresso_board.h
 *
 * NXP LPCXpresso board declarations
 *
 * @author German Rivera
 */
#ifndef __LPCX_BOARD_H
#define __LPCX_BOARD_H

#include <stdint.h>

/**
 * Indexes in g_frdm_rgb_led_pins[]
 */
enum {
    LPCX_RED_LED =      0x0,
    LPCX_GREEN_LED,
    LPCX_BLUE_LED,
    LPCX_NUM_RGB_LED_PINS
};

/**
 * Indexes in g_lpad_button_pins[]
 */
enum {
    LPCX_SW1_BUTTON = 0x0,
#if 0 /*  We cannot use it, as it uses the same pin as LPCX_BLUE_LED */
    LPCX_SW2_BUTTON,
#endif
    LPCX_NUM_PUSH_BUTTONS
};

/*
 * GPIO PORT 0 Pins
 */
#define LPCX_SW1_PIN_INDEX  24
#define LPCX_SW2_PIN_INDEX  31
#define LPCX_RGB_LED_RED_PIN_INDEX      29
#define LPCX_RGB_LED_GREEN_PIN_INDEX    30
#define LPCX_RGB_LED_BLUE_PIN_INDEX     31

#define LED_RED_PIN_MASK    BIT(LPCX_RGB_LED_RED_PIN_INDEX)
#define LED_GREEN_PIN_MASK  BIT(LPCX_RGB_LED_GREEN_PIN_INDEX)
#define LED_BLUE_PIN_MASK   BIT(LPCX_RGB_LED_BLUE_PIN_INDEX)

void lpcxpresso_board_init(void);
void lpcxpresso_board_stop(void);

void lpcxpresso_push_buttons_read(
        _OUT_ bool push_buttons[]);

#endif /* __LPCX_BOARD_H */
