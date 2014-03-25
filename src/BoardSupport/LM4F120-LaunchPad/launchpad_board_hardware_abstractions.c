/**
 * @file launchpad_board_hardware_abstractions.c
 *
 * Hardware abstraction layer for the TI Stellaris Launchpad main board
 *
 * @author German Rivera
 */

#include "hardware_abstractions.h"
#include "lm4f120_soc.h"
#include "McRTOS_arm_cortex_m.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"
#include "launchpad_board.h"

#pragma GCC diagnostic ignored "-Wunused-parameter" //???

static void rgb_led_init(void);
static void push_buttons_init(void);

/**
 * Launchpad board RGB LED pins
 */
static struct pin_config_info g_lpad_rgb_led_pins[] = {
    [LPAD_RED_LED] = PIN_COFIG_INFO_INITIALIZER(
	    GPIO_PORTF_BASE,
            LPAD_RGB_LED_RED_PIN_INDEX,
            false,
            false),

    [LPAD_GREEN_LED] = PIN_COFIG_INFO_INITIALIZER(
	    GPIO_PORTF_BASE,
            LPAD_RGB_LED_GREEN_PIN_INDEX,
            false,
            false),

    [LPAD_BLUE_LED] = PIN_COFIG_INFO_INITIALIZER(
	    GPIO_PORTF_BASE,
            LPAD_RGB_LED_BLUE_PIN_INDEX,
            false,
            false)
};

C_ASSERT(ARRAY_SIZE(g_lpad_rgb_led_pins) == LPAD_NUM_RGB_LED_PINS);

/**
 * Launchpad board button pins
 */
static struct pin_config_info g_lpad_push_button_pins[] = {
    [LPAD_SW1_BUTTON] = PIN_COFIG_INFO_INITIALIZER(
	    GPIO_PORTF_BASE,
            LPAD_SW1_PIN_INDEX,
            false,
            false),

    [LPAD_SW2_BUTTON] = PIN_COFIG_INFO_INITIALIZER(
	    GPIO_PORTF_BASE,
            LPAD_SW2_PIN_INDEX,
            false,
            true),
};

C_ASSERT(ARRAY_SIZE(g_lpad_push_button_pins) == LPAD_NUM_PUSH_BUTTONS);

static uint32_t g_rgb_led_current_mask = 0x0;

void
launchpad_board_init(void)
{
    rgb_led_init();

    #ifdef DEBUG
    if (software_reset_happened()) {
        DEBUG_BLINK_LED(LED_COLOR_YELLOW);
    } else {
        DEBUG_BLINK_LED(LED_COLOR_GREEN);
    }
#   else
    if (software_reset_happened()) {
        (void)set_rgb_led_color(LED_COLOR_YELLOW);
    } else {
        (void)set_rgb_led_color(LED_COLOR_GREEN);
    }
#   endif

    push_buttons_init();
}


void
launchpad_board_stop(void)
{
    set_rgb_led_color(LED_COLOR_BLACK);
}

static void
rgb_led_init(void)
{
    for (int i = 0; i < LPAD_NUM_RGB_LED_PINS; i++) {
        configure_pin(&g_lpad_rgb_led_pins[i], true);
        deactivate_output_pin(&g_lpad_rgb_led_pins[i]);
    }
}

void
toggle_rgb_led(uint32_t led_color_mask)
{
    g_rgb_led_current_mask ^= led_color_mask;

    for (int i = 0; i < LPAD_NUM_RGB_LED_PINS; i++) {
        if (g_lpad_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            toggle_output_pin(&g_lpad_rgb_led_pins[i]);
        }
    }
}


void
turn_on_rgb_led(uint32_t led_color_mask)
{
    g_rgb_led_current_mask |= led_color_mask;

    for (int i = 0; i < LPAD_NUM_RGB_LED_PINS; i++) {
        if (g_lpad_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            activate_output_pin(&g_lpad_rgb_led_pins[i]);
        }
    }
}


void
turn_off_rgb_led(uint32_t led_color_mask)
{
    g_rgb_led_current_mask &= ~led_color_mask;

    for (int i = 0; i < LPAD_NUM_RGB_LED_PINS; i++) {
        if (g_lpad_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            deactivate_output_pin(&g_lpad_rgb_led_pins[i]);
        }
    }
}

/*
 * Set the LED to the given color and returns the previous color
 */
uint32_t
set_rgb_led_color(uint32_t led_color_mask)
{
    uint32_t old_rgb_led_mask = g_rgb_led_current_mask;

    g_rgb_led_current_mask = led_color_mask;

    for (int i = 0; i < LPAD_NUM_RGB_LED_PINS; i++) {
        if (g_lpad_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            activate_output_pin(&g_lpad_rgb_led_pins[i]);
        } else {
            deactivate_output_pin(&g_lpad_rgb_led_pins[i]);
        }
    }

    return old_rgb_led_mask;
}

static void
push_buttons_init(void)
{
    for (int i = 0; i < LPAD_NUM_PUSH_BUTTONS; i++) {
        configure_pin(&g_lpad_push_button_pins[i], false);
    }
}

void
launchpad_push_buttons_read(
        _OUT_ bool push_buttons[])
{
    for (int i = 0; i < LPAD_NUM_PUSH_BUTTONS; i++) {
        push_buttons[i] = read_input_pin(&g_lpad_push_button_pins[i]);
    }
}

