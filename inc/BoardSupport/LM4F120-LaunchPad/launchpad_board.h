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

#define LAUNCHPAD_NUM_PUSH_BUTTONS    3

void launchpad_board_init(void);
void launchpad_board_stop(void);

void launchpad_push_buttons_read(
        _OUT_ bool push_buttons[]);

#endif /* __LAUNCHPAD_BOARD_H */
