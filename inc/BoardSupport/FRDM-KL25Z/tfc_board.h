/**
 * @file tfc_board.h
 *
 * Freescale TFC add-on board declarations
 *
 * @author German Rivera 
 */ 
#ifndef __TFC_BOARD_H
#define __TFC_BOARD_H

#include <stdint.h>

#define TFC_HBRIDGE_EN_LOC   BIT(21)
#define TFC_BAT_LED0_LOC     BIT(8)
#define TFC_BAT_LED1_LOC     BIT(9) 
#define TFC_BAT_LED2_LOC     BIT(10) 
#define TFC_BAT_LED3_LOC     BIT(11) 

void tfc_board_init(void);

#endif /* __TFC_BOARD_H */
