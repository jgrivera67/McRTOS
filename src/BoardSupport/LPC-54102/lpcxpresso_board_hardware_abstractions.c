/**
 * @file lpcxpresso_board_hardware_abstractions.c
 *
 * Hardware abstraction layer for the LPCXpresso main board
 *
 * @author German Rivera
 */

#include "hardware_abstractions.h"
#include "lpc54102_soc.h"
#include "McRTOS_arm_cortex_m.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"
#include "lpcxpresso_board.h"

static void rgb_led_init(void);
static void push_buttons_init(void);

/**
 * board RGB LED pins
 */
static const struct gpio_pin g_lpcx_rgb_led_pins[] = {
    [LPCX_RED_LED] = GPIO_PIN_INITIALIZER(
	    PIN_PORT_0,
            LPCX_RGB_LED_RED_PIN_INDEX,
            PIN_FUNCTION0,
	    false),

    [LPCX_GREEN_LED] = GPIO_PIN_INITIALIZER(
	    PIN_PORT_0,
            LPCX_RGB_LED_GREEN_PIN_INDEX,
            PIN_FUNCTION0,
	    false),

    [LPCX_BLUE_LED] = GPIO_PIN_INITIALIZER(
	    PIN_PORT_0,
            LPCX_RGB_LED_BLUE_PIN_INDEX,
            PIN_FUNCTION0,
            false),
};

C_ASSERT(ARRAY_SIZE(g_lpcx_rgb_led_pins) == LPCX_NUM_RGB_LED_PINS);

/**
 * board button pins
 */
static const struct gpio_pin g_lpcx_push_button_pins[] = {
    [LPCX_SW1_BUTTON] = GPIO_PIN_INITIALIZER(
	    PIN_PORT_0,
            LPCX_SW1_PIN_INDEX,
            PIN_FUNCTION0,
	    false),

    [LPCX_SW2_BUTTON] = GPIO_PIN_INITIALIZER(
	    PIN_PORT_0,
            LPCX_SW2_PIN_INDEX,
            PIN_FUNCTION0,
            false),
};

C_ASSERT(ARRAY_SIZE(g_lpcx_push_button_pins) == LPCX_NUM_PUSH_BUTTONS);

/**
 * Board state variables:
 */
struct __lpcxpresso_board {
    bool rgb_led_initialized;
    uint32_t rgb_led_current_mask;
};

struct lpcxpresso_board {
    struct __lpcxpresso_board;
} __attribute__ ((aligned(SOC_MPU_REGION_ALIGNMENT(struct __lpcxpresso_board))));

C_ASSERT(sizeof(struct lpcxpresso_board) % SOC_MPU_REGION_ALIGNMENT(struct __lpcxpresso_board) == 0);

static struct lpcxpresso_board g_lpcxpresso_board = {{
    .rgb_led_initialized = false,
    .rgb_led_current_mask = 0x0,
}};


void
lpcxpresso_board_init(void)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();

    rgb_led_init();

#   ifdef DEBUG
    if (software_reset_happened()) {
        DEBUG_BLINK_LED(LED_COLOR_YELLOW);
    } else {
        DEBUG_BLINK_LED(LED_COLOR_GREEN);
    }
#   else
    if (software_reset_happened()) {
        (void)set_rgb_led_color(LED_COLOR_YELLOW);
    } else {
        (void)set_rgb_led_color(LED_COLOR_BLACK);
    }
#   endif

    push_buttons_init();

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


void
lpcxpresso_board_stop(void)
{
    set_rgb_led_color(LED_COLOR_BLACK);
}


static void
rgb_led_init(void)
{
    FDC_ASSERT(!g_lpcxpresso_board.rgb_led_initialized, 0, 0);

    for (int i = 0; i < LPCX_NUM_RGB_LED_PINS; i++) {
        configure_gpio_pin(&g_lpcx_rgb_led_pins[i], 0, true);
        deactivate_output_pin(&g_lpcx_rgb_led_pins[i]);
    }

    g_lpcxpresso_board.rgb_led_initialized = true;
}


void
toggle_rgb_led(uint32_t led_color_mask)
{
    struct mpu_region_range old_comp_region;
    bool comp_region_changed = false;

    if (!rtos_in_privileged_mode()) {
        rtos_thread_set_comp_region(&g_lpcxpresso_board,
                                    sizeof g_lpcxpresso_board,
                                    0,
                                    &old_comp_region);
        comp_region_changed = true;
    }

    if (!g_lpcxpresso_board.rgb_led_initialized) {
        goto common_exit;
    }

    g_lpcxpresso_board.rgb_led_current_mask ^= led_color_mask;

    for (int i = 0; i < LPCX_NUM_RGB_LED_PINS; i++) {
        if (g_lpcx_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            toggle_output_pin(&g_lpcx_rgb_led_pins[i]);
        }
    }

common_exit:
    if (comp_region_changed) {
        rtos_thread_restore_comp_region(&old_comp_region);
    }
}


/*
 * Set the LED to the given color and returns the previous color
 */
uint32_t
set_rgb_led_color(uint32_t led_color_mask)
{
    struct mpu_region_range old_comp_region;
    bool comp_region_changed = false;

    if (!rtos_in_privileged_mode()) {
        rtos_thread_set_comp_region(&g_lpcxpresso_board,
                                    sizeof g_lpcxpresso_board,
                                    0,
                                    &old_comp_region);

        comp_region_changed = true;
    }

    uint32_t old_rgb_led_mask = g_lpcxpresso_board.rgb_led_current_mask;

    if (!g_lpcxpresso_board.rgb_led_initialized) {
        goto common_exit;
    }

    g_lpcxpresso_board.rgb_led_current_mask = led_color_mask;

    for (int i = 0; i < LPCX_NUM_RGB_LED_PINS; i++) {
        if (g_lpcx_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            activate_output_pin(&g_lpcx_rgb_led_pins[i]);
        } else {
            deactivate_output_pin(&g_lpcx_rgb_led_pins[i]);
        }
    }

common_exit:
    if (comp_region_changed) {
        rtos_thread_restore_comp_region(&old_comp_region);
    }

    return old_rgb_led_mask;
}

static void
push_buttons_init(void)
{
    for (int i = 0; i < LPCX_NUM_PUSH_BUTTONS; i++) {
        configure_gpio_pin(&g_lpcx_push_button_pins[i], 0, false);
    }
}

void
lpcxpresso_push_buttons_read(
        _OUT_ bool push_buttons[])
{
    /*
     * In the Launchpad board, a button is pressed when the
     * corresponding input pin reads as 0.
     */
    for (int i = 0; i < LPCX_NUM_PUSH_BUTTONS; i++) {
        push_buttons[i] = !read_input_pin(&g_lpcx_push_button_pins[i]);
    }
}

