/**
 * @file launchpad_board_hardware_abstractions.c
 *
 * Hardware abstraction layer for the TI Stellaris Launchpad main board
 *
 * @author German Rivera
 */

#include <BoardSupport/hardware_abstractions.h>
#include <BoardSupport/LaunchPad-LM4F120/lm4f120_soc.h>
#include <McRTOS/McRTOS_arm_cortex_m.h>
#include <McRTOS/failure_data_capture.h>
#include <McRTOS/utils.h>
#include <McRTOS/McRTOS_config_parameters.h>
#include <McRTOS/McRTOS_kernel_services.h>
#include <BoardSupport/LaunchPad-LM4F120/launchpad_board.h>

#pragma GCC diagnostic ignored "-Wunused-parameter" //???

static void rgb_led_init(void);
static void push_buttons_init(void);

/**
 * Launchpad board RGB LED pins
 */
static const struct gpio_pin g_lpad_rgb_led_pins[] = {
    [LPAD_RED_LED] = PIN_COFIG_INFO_INITIALIZER(
	    GPIO_PORTF_BASE,
            LPAD_RGB_LED_RED_PIN_INDEX,
            true,
            false,
	    PIN_PULL_NONE),

    [LPAD_GREEN_LED] = PIN_COFIG_INFO_INITIALIZER(
	    GPIO_PORTF_BASE,
            LPAD_RGB_LED_GREEN_PIN_INDEX,
            true,
            false,
	    PIN_PULL_NONE),

    [LPAD_BLUE_LED] = PIN_COFIG_INFO_INITIALIZER(
	    GPIO_PORTF_BASE,
            LPAD_RGB_LED_BLUE_PIN_INDEX,
            true,
            false,
	    PIN_PULL_NONE)
};

C_ASSERT(ARRAY_SIZE(g_lpad_rgb_led_pins) == LPAD_NUM_RGB_LED_PINS);

/**
 * Launchpad board button pins
 */
static const struct gpio_pin g_lpad_push_button_pins[] = {
    [LPAD_SW1_BUTTON] = PIN_COFIG_INFO_INITIALIZER(
	    GPIO_PORTF_BASE,
            LPAD_SW1_PIN_INDEX,
            false,
            false,
	    PIN_PULL_UP),

    [LPAD_SW2_BUTTON] = PIN_COFIG_INFO_INITIALIZER(
	    GPIO_PORTF_BASE,
            LPAD_SW2_PIN_INDEX,
            false,
            true,
	    PIN_PULL_UP),
};

C_ASSERT(ARRAY_SIZE(g_lpad_push_button_pins) == LPAD_NUM_PUSH_BUTTONS);

struct __launchpad_board {
    bool rgb_led_initialized;
    uint32_t rgb_led_current_mask;
};

struct launchpad_board {
    struct __launchpad_board;
} __attribute__ ((aligned(SOC_MPU_REGION_ALIGNMENT(struct __launchpad_board))));

C_ASSERT(sizeof(struct launchpad_board) % SOC_MPU_REGION_ALIGNMENT(struct __launchpad_board) == 0);

static struct launchpad_board g_launchpad_board = {{
    .rgb_led_initialized = false,
    .rgb_led_current_mask = 0x0,
}};


void
launchpad_board_init(void)
{
    bool caller_was_privileged = rtos_enter_privileged_mode();

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

    if (!caller_was_privileged) {
        rtos_exit_privileged_mode();
    }
}


void
launchpad_board_stop(void)
{
    set_rgb_led_color(LED_COLOR_BLACK);
}

static void
rgb_led_init(void)
{
    FDC_ASSERT(!g_launchpad_board.rgb_led_initialized, 0, 0);

    for (int i = 0; i < LPAD_NUM_RGB_LED_PINS; i++) {
        configure_gpio_pin(&g_lpad_rgb_led_pins[i], 0, true);
        deactivate_output_pin(&g_lpad_rgb_led_pins[i]);
    }

    g_launchpad_board.rgb_led_initialized = true;
}

void
toggle_rgb_led(uint32_t led_color_mask)
{
    struct mpu_region_range old_comp_region;
    bool comp_region_changed = false;

    if (!rtos_in_privileged_mode()) {
        rtos_thread_set_comp_region(&g_launchpad_board,
                                    sizeof g_launchpad_board,
                                    0,
                                    &old_comp_region);
        comp_region_changed = true;
    }

    if (!g_launchpad_board.rgb_led_initialized) {
        goto exit;
    }

    g_launchpad_board.rgb_led_current_mask ^= led_color_mask;

    for (int i = 0; i < LPAD_NUM_RGB_LED_PINS; i++) {
        if (g_lpad_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            toggle_output_pin(&g_lpad_rgb_led_pins[i]);
        }
    }

exit:
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
        rtos_thread_set_comp_region(&g_launchpad_board,
                                    sizeof g_launchpad_board,
                                    0,
                                    &old_comp_region);
        comp_region_changed = true;
    }

    uint32_t old_rgb_led_mask = g_launchpad_board.rgb_led_current_mask;

    if (!g_launchpad_board.rgb_led_initialized) {
        goto common_exit;
    }

    g_launchpad_board.rgb_led_current_mask = led_color_mask;

    for (int i = 0; i < LPAD_NUM_RGB_LED_PINS; i++) {
        if (g_lpad_rgb_led_pins[i].pin_bit_mask & led_color_mask) {
            activate_output_pin(&g_lpad_rgb_led_pins[i]);
        } else {
            deactivate_output_pin(&g_lpad_rgb_led_pins[i]);
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
    for (int i = 0; i < LPAD_NUM_PUSH_BUTTONS; i++) {
        configure_gpio_pin(&g_lpad_push_button_pins[i], 0, false);
    }
}

void
launchpad_push_buttons_read(
        _OUT_ bool push_buttons[])
{
    /*
     * In the Launchpad board, a button is pressed when the
     * corresponding input pin reads as 0.
     */
    for (int i = 0; i < LPAD_NUM_PUSH_BUTTONS; i++) {
        push_buttons[i] = !read_input_pin(&g_lpad_push_button_pins[i]);
    }
}

