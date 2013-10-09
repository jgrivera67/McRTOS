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

#define LED_RED_PIN_MASK    BIT(18)
#define LED_GREEN_PIN_MASK  BIT(19)
#define LED_BLUE_PIN_MASK   BIT(1)

#define LED_COLOR_BLACK     UINT32_C(0x0)
#define LED_COLOR_RED       LED_RED_PIN_MASK
#define LED_COLOR_GREEN     LED_GREEN_PIN_MASK
#define LED_COLOR_YELLOW    (LED_RED_PIN_MASK | LED_GREEN_PIN_MASK)
#define LED_COLOR_BLUE      LED_BLUE_PIN_MASK
#define LED_COLOR_MAGENTA   (LED_RED_PIN_MASK | LED_BLUE_PIN_MASK)
#define LED_COLOR_CYAN      (LED_GREEN_PIN_MASK | LED_GREEN_PIN_MASK)
#define LED_COLOR_WHITE     (LED_RED_PIN_MASK | LED_GREEN_PIN_MASK | LED_GREEN_PIN_MASK)

void frdm_board_init(void);

void toggle_rgb_led(uint32_t led_color_mask);
void turn_on_rgb_led(uint32_t led_color_mask);
void turn_off_rgb_led(uint32_t led_color_mask);
uint32_t set_rgb_led_color(uint32_t led_color_mask);

#endif /* __FRDM_BOARD_H */
