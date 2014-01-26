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

void
launchpad_board_init(void)
{

}


void
launchpad_board_stop(void)
{

}

void
launchpad_push_buttons_read(
        _OUT_ bool push_buttons[])
{
    for (int i = 0; i < LAUNCHPAD_NUM_PUSH_BUTTONS; i++) {
        //???push_buttons[i] = read_input_pin(&g_tfc_push_button_pins[i]);
    }
}

