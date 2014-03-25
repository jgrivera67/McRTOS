/**
 * @file launchpad_board.h
 *
 * TI Stellaris Launchpad board declarations
 *
 * @author German Rivera
 */
#ifndef __LAUNCHPAD_BOARD_H
#define __LAUNCHPAD_BOARD_H

#include <stdint.h>

/**
 * Indexes in g_lpad_rgb_led_pins[]
 */
enum {
    LPAD_RED_LED = 0x0,
    LPAD_GREEN_LED,
    LPAD_BLUE_LED,
    LPAD_NUM_RGB_LED_PINS
};

/**
 * Indexes in g_lpad_button_pins[]
 */
enum {
    LPAD_SW1_BUTTON = 0x0,
    LPAD_SW2_BUTTON,
    LPAD_NUM_PUSH_BUTTONS
};

/*
 * LM4F120 GPIO PORT F Pins
 */

#define LPAD_SW2_PIN_INDEX		0
#define LPAD_RGB_LED_RED_PIN_INDEX	1
#define LPAD_RGB_LED_BLUE_PIN_INDEX     2
#define LPAD_RGB_LED_GREEN_PIN_INDEX    3
#define LPAD_SW1_PIN_INDEX		4

#define LED_RED_PIN_MASK    BIT(LPAD_RGB_LED_RED_PIN_INDEX)
#define LED_GREEN_PIN_MASK  BIT(LPAD_RGB_LED_GREEN_PIN_INDEX)
#define LED_BLUE_PIN_MASK   BIT(LPAD_RGB_LED_BLUE_PIN_INDEX)

void launchpad_board_init(void);
void launchpad_board_stop(void);

void launchpad_push_buttons_read(
        _OUT_ bool push_buttons[]);

#endif /* __LAUNCHPAD_BOARD_H */
