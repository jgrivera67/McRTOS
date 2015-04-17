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
static struct gpio_pin g_lpad_rgb_led_pins[] = {
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
static struct gpio_pin g_lpad_push_button_pins[] = {
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

static uint32_t g_rgb_led_current_mask = 0x0;

#if 1 // ???
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

#else // ????

#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-function"
/*
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
*/

// 2. Declarations Section
//   Global Variables
unsigned long In;  // input from PF4
unsigned long Out; // outputs to PF3,PF2,PF1 (multicolor LED)

// Subroutine to initialize port F pins for input and output
// PF4 and PF0 are input SW1 and SW2 respectively
// PF3,PF2,PF1 are outputs to the LED
// Inputs: None
// Outputs: None
// Notes: These five pins are connected to hardware on the LaunchPad
static void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  //delay = SYSCTL_RCGC2_R;           // delay
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0
}
// Color    LED(s) PortF
// dark     ---    0
// red      R--    0x02
// blue     --B    0x04
// green    -G-    0x08
// yellow   RG-    0x0A
// sky blue -GB    0x0C
// white    RGB    0x0E
// pink     R-B    0x06

// Subroutine to wait 0.1 sec
// Inputs: None
// Outputs: None
// Notes: ...
static void Delay(void){unsigned long volatile time;
  time = 727240*200/91;  // 0.1sec
  while(time){
		time--;
  }
}

void
launchpad_board_init(void)
{
#if 0
  PortF_Init();        // Call initialization of port PF4 PF2
#else
    rgb_led_init();
    push_buttons_init();
#endif

  bool push_buttons[LPAD_NUM_PUSH_BUTTONS];
  while(1){
        launchpad_push_buttons_read(push_buttons);
    if (push_buttons[LPAD_SW1_BUTTON]){              // zero means SW1 is pressed
	    set_rgb_led_color(LED_COLOR_RED);
    } else {                      // 0x10 means SW1 is not pressed
	    set_rgb_led_color(LED_COLOR_GREEN);
    }
    Delay();                     // wait 0.1 sec
    set_rgb_led_color(LED_COLOR_BLUE);
    Delay();                     // wait 0.1 sec
  }
}

#endif //???


void
launchpad_board_stop(void)
{
    set_rgb_led_color(LED_COLOR_BLACK);
}

static void
rgb_led_init(void)
{
    for (int i = 0; i < LPAD_NUM_RGB_LED_PINS; i++) {
        configure_gpio_pin(&g_lpad_rgb_led_pins[i], 0, true);
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

