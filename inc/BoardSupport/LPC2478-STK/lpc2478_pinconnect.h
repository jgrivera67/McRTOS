#ifndef __PINCONNECT_H
#define __PINCONNECT_H

#include <inttypes.h>   /* C99 int types */
#include "compile_time_checks.h"

/*******************************************************************************
 * Pin function select register values
 */

#define PINSEL_PRIMARY  UINT32_C(0x0) /* Primary (default) function, typically GPIO port */
#define PINSEL_ALT_01   UINT32_C(0x1) /* First  alternate function */
#define PINSEL_ALT_10   UINT32_C(0x2) /*`Second alternate function */
#define PINSEL_ALT_11   UINT32_C(0x3) /* Third  alternate function */

#define PINSEL_MASK(function, pos)  ((function & 0x00000003) << (pos * 2))
#define PINSEL_CLR_MASK(pos) (~(3 << (pos * 2)))

/*******************************************************************************
 * Pin mode select register bits - Pull-up, Pull-down, or no resistor
 */
#define PINMODE_PULL_UP   UINT32_C(0x0) /* Pin has on-chip pull-up enabled (default) */
#define PINMODE_PULL_NONE UINT32_C(0x2) /* Pin has neither pull-up nor pull-down enabled */
#define PINMODE_PULL_DOWN UINT32_C(0x3) /* Pin has on-chip pull-down enabled */

#define PINMODE_MASK(mode, pos) ((mode & 0x00000003) << (pos * 2))

#define PINCON_BASE_ADDR  UINT32_C(0xE002C000)

#define PINSEL10_MASK  0x00000008   /* mask reserved bits */

#define PINSEL1_DACR       BIT(21)

/**
 * grivera: Register space size for the pin connect block
 */
#define PIN_CONNECT_BLOCK_SIZE	0x68

/**
 * grivera: Number of PINSEL registers
 */
#define	PINCON_NUM_PIN_SEL_REGISTERS    12

/**
 * grivera: Number of PINMODE registers
 */
#define	PINCON_NUM_PIN_MODE_REGISTERS   10

/**
 * grivera: Memory-mapped I/O registers of the LPC2478 Pin connect block
 */
struct pin_connect_block
{
    /**
     * Pin function select registers
     */ 
    uint32_t reg_PINSEL[PINCON_NUM_PIN_SEL_REGISTERS];

    /**
     * 16 bytes of Padding
     */
    uint8_t reg_padding[0x10];

    /**
     * Pin mode select registers
     */ 
    uint32_t reg_PINMODE[PINCON_NUM_PIN_MODE_REGISTERS];
};

typedef volatile struct pin_connect_block pin_connect_block_t;

/*
 * grivera - Compile-time asserts to verify that the compiler generates the expected offsets
 * for structs that represent groups of memory-mapped I/O registers
 */

C_ASSERT(sizeof(pin_connect_block_t) == PIN_CONNECT_BLOCK_SIZE);

C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[0]) == 0x00);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[1]) == 0x04);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[2]) == 0x08);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[3]) == 0x0c);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[4]) == 0x10);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[5]) == 0x14);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[6]) == 0x18);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[7]) == 0x1C);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[8]) == 0x20);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[9]) == 0x24);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINSEL[10]) == 0x28);

C_ASSERT(offsetof(pin_connect_block_t, reg_PINMODE[0]) == 0x40);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINMODE[1]) == 0x44);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINMODE[2]) == 0x48);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINMODE[3]) == 0x4C);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINMODE[4]) == 0x50);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINMODE[5]) == 0x54);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINMODE[6]) == 0x58);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINMODE[7]) == 0x5C);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINMODE[8]) == 0x60);
C_ASSERT(offsetof(pin_connect_block_t, reg_PINMODE[9]) == 0x64);


/**
 * Given the index of a GPIO port (0 .. 4) and the bit index
 * within that port (0 .. 31), return the corresponding index in
 * reg_PINSEL[] and reg_PINMODE[] (0 .. 9).
 */
#define GET_PINSEL_MODE_INDEX(_gpio_port_index, _pin_bit_index) \
    (((_pin_bit_index) / 16 == 0) ? (_gpio_port_index) * 2      \
                                  : (_gpio_port_index) * 2  + 1)

#define GET_PINSEL_MODE_SHIFT(_pin_bit_index) \
    (((_pin_bit_index) % 16) * 2)

#define GET_PINSEL_MODE_MASK(_pin_bit_index) \
    MULTI_BIT_MASK(GET_PINSEL_MODE_SHIFT(_pin_bit_index) + 1,   \
                   GET_PINSEL_MODE_SHIFT(_pin_bit_index))

#if 0 // XXX
/**
 * PIN bit masks for PINSEL and PINMODE registers
 */
#define PIN_SEL_MODE_0_P0_0_MASK	MULTI_BIT_MASK(1, 0)
#define PIN_SEL_MODE_0_P0_0_SHIFT	0
#define PIN_SEL_MODE_0_P0_1_MASK	MULTI_BIT_MASK(3, 2)
#define PIN_SEL_MODE_0_P0_1_SHIFT	2
#define PIN_SEL_MODE_0_P0_2_MASK	MULTI_BIT_MASK(5, 4)
#define PIN_SEL_MODE_0_P0_2_SHIFT	4
#define PIN_SEL_MODE_0_P0_3_MASK	MULTI_BIT_MASK(7, 6)
#define PIN_SEL_MODE_0_P0_3_SHIFT	6
#define PIN_SEL_MODE_0_P0_4_MASK	MULTI_BIT_MASK(9, 8)
#define PIN_SEL_MODE_0_P0_4_SHIFT	8
#define PIN_SEL_MODE_0_P0_5_MASK	MULTI_BIT_MASK(11, 10)
#define PIN_SEL_MODE_0_P0_5_SHIFT	10
#define PIN_SEL_MODE_0_P0_6_MASK	MULTI_BIT_MASK(13, 12)
#define PIN_SEL_MODE_0_P0_6_SHIFT	12
#define PIN_SEL_MODE_0_P0_7_MASK	MULTI_BIT_MASK(15, 14)
#define PIN_SEL_MODE_0_P0_7_SHIFT	14
#define PIN_SEL_MODE_0_P0_8_MASK	MULTI_BIT_MASK(17, 16)
#define PIN_SEL_MODE_0_P0_8_SHIFT	16
#define PIN_SEL_MODE_0_P0_9_MASK	MULTI_BIT_MASK(19, 18)
#define PIN_SEL_MODE_0_P0_9_SHIFT	18
#define PIN_SEL_MODE_0_P0_10_MASK	MULTI_BIT_MASK(21, 20)
#define PIN_SEL_MODE_0_P0_10_SHIFT	20
#define PIN_SEL_MODE_0_P0_11_MASK	MULTI_BIT_MASK(23, 22)
#define PIN_SEL_MODE_0_P0_11_SHIFT	22
#define PIN_SEL_MODE_0_P0_12_MASK	MULTI_BIT_MASK(25, 24)
#define PIN_SEL_MODE_0_P0_12_SHIFT	24
#define PIN_SEL_MODE_0_P0_13_MASK	MULTI_BIT_MASK(27, 26)
#define PIN_SEL_MODE_0_P0_13_SHIFT	26
#define PIN_SEL_MODE_0_P0_14_MASK	MULTI_BIT_MASK(29, 28)
#define PIN_SEL_MODE_0_P0_14_SHIFT	28
#define PIN_SEL_MODE_0_P0_15_MASK	MULTI_BIT_MASK(31, 30)
#define PIN_SEL_MODE_0_P0_15_SHIFT	30
#define PIN_SEL_MODE_1_P0_16_MASK	MULTI_BIT_MASK(1, 0)
#define PIN_SEL_MODE_1_P0_16_SHIFT	0
#define PIN_SEL_MODE_1_P0_17_MASK	MULTI_BIT_MASK(3, 2)
#define PIN_SEL_MODE_1_P0_17_SHIFT	2
#define PIN_SEL_MODE_1_P0_18_MASK	MULTI_BIT_MASK(5, 4)
#define PIN_SEL_MODE_1_P0_18_SHIFT	4
#define PIN_SEL_MODE_1_P0_19_MASK	MULTI_BIT_MASK(7, 6)
#define PIN_SEL_MODE_1_P0_19_SHIFT	6
#define PIN_SEL_MODE_1_P0_20_MASK	MULTI_BIT_MASK(9, 8)
#define PIN_SEL_MODE_1_P0_20_SHIFT	8
#define PIN_SEL_MODE_1_P0_21_MASK	MULTI_BIT_MASK(11, 10)
#define PIN_SEL_MODE_1_P0_21_SHIFT	10
#define PIN_SEL_MODE_1_P0_22_MASK	MULTI_BIT_MASK(13, 12)
#define PIN_SEL_MODE_1_P0_22_SHIFT	12
#define PIN_SEL_MODE_1_P0_23_MASK	MULTI_BIT_MASK(15, 14)
#define PIN_SEL_MODE_1_P0_23_SHIFT	14
#define PIN_SEL_MODE_1_P0_24_MASK	MULTI_BIT_MASK(17, 16)
#define PIN_SEL_MODE_1_P0_24_SHIFT	16
#define PIN_SEL_MODE_1_P0_25_MASK	MULTI_BIT_MASK(19, 18)
#define PIN_SEL_MODE_1_P0_25_SHIFT	18
#define PIN_SEL_MODE_1_P0_26_MASK	MULTI_BIT_MASK(21, 20)
#define PIN_SEL_MODE_1_P0_26_SHIFT	20
#define PIN_SEL_MODE_1_P0_27_MASK	MULTI_BIT_MASK(23, 22)
#define PIN_SEL_MODE_1_P0_27_SHIFT	22
#define PIN_SEL_MODE_1_P0_28_MASK	MULTI_BIT_MASK(25, 24)
#define PIN_SEL_MODE_1_P0_28_SHIFT	24
#define PIN_SEL_MODE_1_P0_29_MASK	MULTI_BIT_MASK(27, 26)
#define PIN_SEL_MODE_1_P0_29_SHIFT	26
#define PIN_SEL_MODE_1_P0_30_MASK	MULTI_BIT_MASK(29, 28)
#define PIN_SEL_MODE_1_P0_30_SHIFT	28
#define PIN_SEL_MODE_1_P0_31_MASK	MULTI_BIT_MASK(31, 30)
#define PIN_SEL_MODE_1_P0_31_SHIFT	30
#define PIN_SEL_MODE_2_P1_0_MASK	MULTI_BIT_MASK(1, 0)
#define PIN_SEL_MODE_2_P1_0_SHIFT	0
#define PIN_SEL_MODE_2_P1_1_MASK	MULTI_BIT_MASK(3, 2)
#define PIN_SEL_MODE_2_P1_1_SHIFT	2
#define PIN_SEL_MODE_2_P1_2_MASK	MULTI_BIT_MASK(5, 4)
#define PIN_SEL_MODE_2_P1_2_SHIFT	4
#define PIN_SEL_MODE_2_P1_3_MASK	MULTI_BIT_MASK(7, 6)
#define PIN_SEL_MODE_2_P1_3_SHIFT	6
#define PIN_SEL_MODE_2_P1_4_MASK	MULTI_BIT_MASK(9, 8)
#define PIN_SEL_MODE_2_P1_4_SHIFT	8
#define PIN_SEL_MODE_2_P1_5_MASK	MULTI_BIT_MASK(11, 10)
#define PIN_SEL_MODE_2_P1_5_SHIFT	10
#define PIN_SEL_MODE_2_P1_6_MASK	MULTI_BIT_MASK(13, 12)
#define PIN_SEL_MODE_2_P1_6_SHIFT	12
#define PIN_SEL_MODE_2_P1_7_MASK	MULTI_BIT_MASK(15, 14)
#define PIN_SEL_MODE_2_P1_7_SHIFT	14
#define PIN_SEL_MODE_2_P1_8_MASK	MULTI_BIT_MASK(17, 16)
#define PIN_SEL_MODE_2_P1_8_SHIFT	16
#define PIN_SEL_MODE_2_P1_9_MASK	MULTI_BIT_MASK(19, 18)
#define PIN_SEL_MODE_2_P1_9_SHIFT	18
#define PIN_SEL_MODE_2_P1_10_MASK	MULTI_BIT_MASK(21, 20)
#define PIN_SEL_MODE_2_P1_10_SHIFT	20
#define PIN_SEL_MODE_2_P1_11_MASK	MULTI_BIT_MASK(23, 22)
#define PIN_SEL_MODE_2_P1_11_SHIFT	22
#define PIN_SEL_MODE_2_P1_12_MASK	MULTI_BIT_MASK(25, 24)
#define PIN_SEL_MODE_2_P1_12_SHIFT	24
#define PIN_SEL_MODE_2_P1_13_MASK	MULTI_BIT_MASK(27, 26)
#define PIN_SEL_MODE_2_P1_13_SHIFT	26
#define PIN_SEL_MODE_2_P1_14_MASK	MULTI_BIT_MASK(29, 28)
#define PIN_SEL_MODE_2_P1_14_SHIFT	28
#define PIN_SEL_MODE_2_P1_15_MASK	MULTI_BIT_MASK(31, 30)
#define PIN_SEL_MODE_2_P1_15_SHIFT	30
#define PIN_SEL_MODE_3_P1_16_MASK	MULTI_BIT_MASK(1, 0)
#define PIN_SEL_MODE_3_P1_16_SHIFT	0
#define PIN_SEL_MODE_3_P1_17_MASK	MULTI_BIT_MASK(3, 2)
#define PIN_SEL_MODE_3_P1_17_SHIFT	2
#define PIN_SEL_MODE_3_P1_18_MASK	MULTI_BIT_MASK(5, 4)
#define PIN_SEL_MODE_3_P1_18_SHIFT	4
#define PIN_SEL_MODE_3_P1_19_MASK	MULTI_BIT_MASK(7, 6)
#define PIN_SEL_MODE_3_P1_19_SHIFT	6
#define PIN_SEL_MODE_3_P1_20_MASK	MULTI_BIT_MASK(9, 8)
#define PIN_SEL_MODE_3_P1_20_SHIFT	8
#define PIN_SEL_MODE_3_P1_21_MASK	MULTI_BIT_MASK(11, 10)
#define PIN_SEL_MODE_3_P1_21_SHIFT	10
#define PIN_SEL_MODE_3_P1_22_MASK	MULTI_BIT_MASK(13, 12)
#define PIN_SEL_MODE_3_P1_22_SHIFT	12
#define PIN_SEL_MODE_3_P1_23_MASK	MULTI_BIT_MASK(15, 14)
#define PIN_SEL_MODE_3_P1_23_SHIFT	14
#define PIN_SEL_MODE_3_P1_24_MASK	MULTI_BIT_MASK(17, 16)
#define PIN_SEL_MODE_3_P1_24_SHIFT	16
#define PIN_SEL_MODE_3_P1_25_MASK	MULTI_BIT_MASK(19, 18)
#define PIN_SEL_MODE_3_P1_25_SHIFT	18
#define PIN_SEL_MODE_3_P1_26_MASK	MULTI_BIT_MASK(21, 20)
#define PIN_SEL_MODE_3_P1_26_SHIFT	20
#define PIN_SEL_MODE_3_P1_27_MASK	MULTI_BIT_MASK(23, 22)
#define PIN_SEL_MODE_3_P1_27_SHIFT	22
#define PIN_SEL_MODE_3_P1_28_MASK	MULTI_BIT_MASK(25, 24)
#define PIN_SEL_MODE_3_P1_28_SHIFT	24
#define PIN_SEL_MODE_3_P1_29_MASK	MULTI_BIT_MASK(27, 26)
#define PIN_SEL_MODE_3_P1_29_SHIFT	26
#define PIN_SEL_MODE_3_P1_30_MASK	MULTI_BIT_MASK(29, 28)
#define PIN_SEL_MODE_3_P1_30_SHIFT	28
#define PIN_SEL_MODE_3_P1_31_MASK	MULTI_BIT_MASK(31, 30)
#define PIN_SEL_MODE_3_P1_31_SHIFT	30
#define PIN_SEL_MODE_4_P2_0_MASK	MULTI_BIT_MASK(1, 0)
#define PIN_SEL_MODE_4_P2_0_SHIFT	0
#define PIN_SEL_MODE_4_P2_1_MASK	MULTI_BIT_MASK(3, 2)
#define PIN_SEL_MODE_4_P2_1_SHIFT	2
#define PIN_SEL_MODE_4_P2_2_MASK	MULTI_BIT_MASK(5, 4)
#define PIN_SEL_MODE_4_P2_2_SHIFT	4
#define PIN_SEL_MODE_4_P2_3_MASK	MULTI_BIT_MASK(7, 6)
#define PIN_SEL_MODE_4_P2_3_SHIFT	6
#define PIN_SEL_MODE_4_P2_4_MASK	MULTI_BIT_MASK(9, 8)
#define PIN_SEL_MODE_4_P2_4_SHIFT	8
#define PIN_SEL_MODE_4_P2_5_MASK	MULTI_BIT_MASK(11, 10)
#define PIN_SEL_MODE_4_P2_5_SHIFT	10
#define PIN_SEL_MODE_4_P2_6_MASK	MULTI_BIT_MASK(13, 12)
#define PIN_SEL_MODE_4_P2_6_SHIFT	12
#define PIN_SEL_MODE_4_P2_7_MASK	MULTI_BIT_MASK(15, 14)
#define PIN_SEL_MODE_4_P2_7_SHIFT	14
#define PIN_SEL_MODE_4_P2_8_MASK	MULTI_BIT_MASK(17, 16)
#define PIN_SEL_MODE_4_P2_8_SHIFT	16
#define PIN_SEL_MODE_4_P2_9_MASK	MULTI_BIT_MASK(19, 18)
#define PIN_SEL_MODE_4_P2_9_SHIFT	18
#define PIN_SEL_MODE_4_P2_10_MASK	MULTI_BIT_MASK(21, 20)
#define PIN_SEL_MODE_4_P2_10_SHIFT	20
#define PIN_SEL_MODE_4_P2_11_MASK	MULTI_BIT_MASK(23, 22)
#define PIN_SEL_MODE_4_P2_11_SHIFT	22
#define PIN_SEL_MODE_4_P2_12_MASK	MULTI_BIT_MASK(25, 24)
#define PIN_SEL_MODE_4_P2_12_SHIFT	24
#define PIN_SEL_MODE_4_P2_13_MASK	MULTI_BIT_MASK(27, 26)
#define PIN_SEL_MODE_4_P2_13_SHIFT	26
#define PIN_SEL_MODE_4_P2_14_MASK	MULTI_BIT_MASK(29, 28)
#define PIN_SEL_MODE_4_P2_14_SHIFT	28
#define PIN_SEL_MODE_4_P2_15_MASK	MULTI_BIT_MASK(31, 30)
#define PIN_SEL_MODE_4_P2_15_SHIFT	30
#define PIN_SEL_MODE_5_P2_16_MASK	MULTI_BIT_MASK(1, 0)
#define PIN_SEL_MODE_5_P2_16_SHIFT	0
#define PIN_SEL_MODE_5_P2_17_MASK	MULTI_BIT_MASK(3, 2)
#define PIN_SEL_MODE_5_P2_17_SHIFT	2
#define PIN_SEL_MODE_5_P2_18_MASK	MULTI_BIT_MASK(5, 4)
#define PIN_SEL_MODE_5_P2_18_SHIFT	4
#define PIN_SEL_MODE_5_P2_19_MASK	MULTI_BIT_MASK(7, 6)
#define PIN_SEL_MODE_5_P2_19_SHIFT	6
#define PIN_SEL_MODE_5_P2_20_MASK	MULTI_BIT_MASK(9, 8)
#define PIN_SEL_MODE_5_P2_20_SHIFT	8
#define PIN_SEL_MODE_5_P2_21_MASK	MULTI_BIT_MASK(11, 10)
#define PIN_SEL_MODE_5_P2_21_SHIFT	10
#define PIN_SEL_MODE_5_P2_22_MASK	MULTI_BIT_MASK(13, 12)
#define PIN_SEL_MODE_5_P2_22_SHIFT	12
#define PIN_SEL_MODE_5_P2_23_MASK	MULTI_BIT_MASK(15, 14)
#define PIN_SEL_MODE_5_P2_23_SHIFT	14
#define PIN_SEL_MODE_5_P2_24_MASK	MULTI_BIT_MASK(17, 16)
#define PIN_SEL_MODE_5_P2_24_SHIFT	16
#define PIN_SEL_MODE_5_P2_25_MASK	MULTI_BIT_MASK(19, 18)
#define PIN_SEL_MODE_5_P2_25_SHIFT	18
#define PIN_SEL_MODE_5_P2_26_MASK	MULTI_BIT_MASK(21, 20)
#define PIN_SEL_MODE_5_P2_26_SHIFT	20
#define PIN_SEL_MODE_5_P2_27_MASK	MULTI_BIT_MASK(23, 22)
#define PIN_SEL_MODE_5_P2_27_SHIFT	22
#define PIN_SEL_MODE_5_P2_28_MASK	MULTI_BIT_MASK(25, 24)
#define PIN_SEL_MODE_5_P2_28_SHIFT	24
#define PIN_SEL_MODE_5_P2_29_MASK	MULTI_BIT_MASK(27, 26)
#define PIN_SEL_MODE_5_P2_29_SHIFT	26
#define PIN_SEL_MODE_5_P2_30_MASK	MULTI_BIT_MASK(29, 28)
#define PIN_SEL_MODE_5_P2_30_SHIFT	28
#define PIN_SEL_MODE_5_P2_31_MASK	MULTI_BIT_MASK(31, 30)
#define PIN_SEL_MODE_5_P2_31_SHIFT	30
#define PIN_SEL_MODE_6_P3_0_MASK	MULTI_BIT_MASK(1, 0)
#define PIN_SEL_MODE_6_P3_0_SHIFT	0
#define PIN_SEL_MODE_6_P3_1_MASK	MULTI_BIT_MASK(3, 2)
#define PIN_SEL_MODE_6_P3_1_SHIFT	2
#define PIN_SEL_MODE_6_P3_2_MASK	MULTI_BIT_MASK(5, 4)
#define PIN_SEL_MODE_6_P3_2_SHIFT	4
#define PIN_SEL_MODE_6_P3_3_MASK	MULTI_BIT_MASK(7, 6)
#define PIN_SEL_MODE_6_P3_3_SHIFT	6
#define PIN_SEL_MODE_6_P3_4_MASK	MULTI_BIT_MASK(9, 8)
#define PIN_SEL_MODE_6_P3_4_SHIFT	8
#define PIN_SEL_MODE_6_P3_5_MASK	MULTI_BIT_MASK(11, 10)
#define PIN_SEL_MODE_6_P3_5_SHIFT	10
#define PIN_SEL_MODE_6_P3_6_MASK	MULTI_BIT_MASK(13, 12)
#define PIN_SEL_MODE_6_P3_6_SHIFT	12
#define PIN_SEL_MODE_6_P3_7_MASK	MULTI_BIT_MASK(15, 14)
#define PIN_SEL_MODE_6_P3_7_SHIFT	14
#define PIN_SEL_MODE_6_P3_8_MASK	MULTI_BIT_MASK(17, 16)
#define PIN_SEL_MODE_6_P3_8_SHIFT	16
#define PIN_SEL_MODE_6_P3_9_MASK	MULTI_BIT_MASK(19, 18)
#define PIN_SEL_MODE_6_P3_9_SHIFT	18
#define PIN_SEL_MODE_6_P3_10_MASK	MULTI_BIT_MASK(21, 20)
#define PIN_SEL_MODE_6_P3_10_SHIFT	20
#define PIN_SEL_MODE_6_P3_11_MASK	MULTI_BIT_MASK(23, 22)
#define PIN_SEL_MODE_6_P3_11_SHIFT	22
#define PIN_SEL_MODE_6_P3_12_MASK	MULTI_BIT_MASK(25, 24)
#define PIN_SEL_MODE_6_P3_12_SHIFT	24
#define PIN_SEL_MODE_6_P3_13_MASK	MULTI_BIT_MASK(27, 26)
#define PIN_SEL_MODE_6_P3_13_SHIFT	26
#define PIN_SEL_MODE_6_P3_14_MASK	MULTI_BIT_MASK(29, 28)
#define PIN_SEL_MODE_6_P3_14_SHIFT	28
#define PIN_SEL_MODE_6_P3_15_MASK	MULTI_BIT_MASK(31, 30)
#define PIN_SEL_MODE_6_P3_15_SHIFT	30
#define PIN_SEL_MODE_7_P3_16_MASK	MULTI_BIT_MASK(1, 0)
#define PIN_SEL_MODE_7_P3_16_SHIFT	0
#define PIN_SEL_MODE_7_P3_17_MASK	MULTI_BIT_MASK(3, 2)
#define PIN_SEL_MODE_7_P3_17_SHIFT	2
#define PIN_SEL_MODE_7_P3_18_MASK	MULTI_BIT_MASK(5, 4)
#define PIN_SEL_MODE_7_P3_18_SHIFT	4
#define PIN_SEL_MODE_7_P3_19_MASK	MULTI_BIT_MASK(7, 6)
#define PIN_SEL_MODE_7_P3_19_SHIFT	6
#define PIN_SEL_MODE_7_P3_20_MASK	MULTI_BIT_MASK(9, 8)
#define PIN_SEL_MODE_7_P3_20_SHIFT	8
#define PIN_SEL_MODE_7_P3_21_MASK	MULTI_BIT_MASK(11, 10)
#define PIN_SEL_MODE_7_P3_21_SHIFT	10
#define PIN_SEL_MODE_7_P3_22_MASK	MULTI_BIT_MASK(13, 12)
#define PIN_SEL_MODE_7_P3_22_SHIFT	12
#define PIN_SEL_MODE_7_P3_23_MASK	MULTI_BIT_MASK(15, 14)
#define PIN_SEL_MODE_7_P3_23_SHIFT	14
#define PIN_SEL_MODE_7_P3_24_MASK	MULTI_BIT_MASK(17, 16)
#define PIN_SEL_MODE_7_P3_24_SHIFT	16
#define PIN_SEL_MODE_7_P3_25_MASK	MULTI_BIT_MASK(19, 18)
#define PIN_SEL_MODE_7_P3_25_SHIFT	18
#define PIN_SEL_MODE_7_P3_26_MASK	MULTI_BIT_MASK(21, 20)
#define PIN_SEL_MODE_7_P3_26_SHIFT	20
#define PIN_SEL_MODE_7_P3_27_MASK	MULTI_BIT_MASK(23, 22)
#define PIN_SEL_MODE_7_P3_27_SHIFT	22
#define PIN_SEL_MODE_7_P3_28_MASK	MULTI_BIT_MASK(25, 24)
#define PIN_SEL_MODE_7_P3_28_SHIFT	24
#define PIN_SEL_MODE_7_P3_29_MASK	MULTI_BIT_MASK(27, 26)
#define PIN_SEL_MODE_7_P3_29_SHIFT	26
#define PIN_SEL_MODE_7_P3_30_MASK	MULTI_BIT_MASK(29, 28)
#define PIN_SEL_MODE_7_P3_30_SHIFT	28
#define PIN_SEL_MODE_7_P3_31_MASK	MULTI_BIT_MASK(31, 30)
#define PIN_SEL_MODE_7_P3_31_SHIFT	30
#define PIN_SEL_MODE_8_P4_0_MASK	MULTI_BIT_MASK(1, 0)
#define PIN_SEL_MODE_8_P4_0_SHIFT	0
#define PIN_SEL_MODE_8_P4_1_MASK	MULTI_BIT_MASK(3, 2)
#define PIN_SEL_MODE_8_P4_1_SHIFT	2
#define PIN_SEL_MODE_8_P4_2_MASK	MULTI_BIT_MASK(5, 4)
#define PIN_SEL_MODE_8_P4_2_SHIFT	4
#define PIN_SEL_MODE_8_P4_3_MASK	MULTI_BIT_MASK(7, 6)
#define PIN_SEL_MODE_8_P4_3_SHIFT	6
#define PIN_SEL_MODE_8_P4_4_MASK	MULTI_BIT_MASK(9, 8)
#define PIN_SEL_MODE_8_P4_4_SHIFT	8
#define PIN_SEL_MODE_8_P4_5_MASK	MULTI_BIT_MASK(11, 10)
#define PIN_SEL_MODE_8_P4_5_SHIFT	10
#define PIN_SEL_MODE_8_P4_6_MASK	MULTI_BIT_MASK(13, 12)
#define PIN_SEL_MODE_8_P4_6_SHIFT	12
#define PIN_SEL_MODE_8_P4_7_MASK	MULTI_BIT_MASK(15, 14)
#define PIN_SEL_MODE_8_P4_7_SHIFT	14
#define PIN_SEL_MODE_8_P4_8_MASK	MULTI_BIT_MASK(17, 16)
#define PIN_SEL_MODE_8_P4_8_SHIFT	16
#define PIN_SEL_MODE_8_P4_9_MASK	MULTI_BIT_MASK(19, 18)
#define PIN_SEL_MODE_8_P4_9_SHIFT	18
#define PIN_SEL_MODE_8_P4_10_MASK	MULTI_BIT_MASK(21, 20)
#define PIN_SEL_MODE_8_P4_10_SHIFT	20
#define PIN_SEL_MODE_8_P4_11_MASK	MULTI_BIT_MASK(23, 22)
#define PIN_SEL_MODE_8_P4_11_SHIFT	22
#define PIN_SEL_MODE_8_P4_12_MASK	MULTI_BIT_MASK(25, 24)
#define PIN_SEL_MODE_8_P4_12_SHIFT	24
#define PIN_SEL_MODE_8_P4_13_MASK	MULTI_BIT_MASK(27, 26)
#define PIN_SEL_MODE_8_P4_13_SHIFT	26
#define PIN_SEL_MODE_8_P4_14_MASK	MULTI_BIT_MASK(29, 28)
#define PIN_SEL_MODE_8_P4_14_SHIFT	28
#define PIN_SEL_MODE_8_P4_15_MASK	MULTI_BIT_MASK(31, 30)
#define PIN_SEL_MODE_8_P4_15_SHIFT	30
#define PIN_SEL_MODE_9_P4_16_MASK	MULTI_BIT_MASK(1, 0)
#define PIN_SEL_MODE_9_P4_16_SHIFT	0
#define PIN_SEL_MODE_9_P4_17_MASK	MULTI_BIT_MASK(3, 2)
#define PIN_SEL_MODE_9_P4_17_SHIFT	2
#define PIN_SEL_MODE_9_P4_18_MASK	MULTI_BIT_MASK(5, 4)
#define PIN_SEL_MODE_9_P4_18_SHIFT	4
#define PIN_SEL_MODE_9_P4_19_MASK	MULTI_BIT_MASK(7, 6)
#define PIN_SEL_MODE_9_P4_19_SHIFT	6
#define PIN_SEL_MODE_9_P4_20_MASK	MULTI_BIT_MASK(9, 8)
#define PIN_SEL_MODE_9_P4_20_SHIFT	8
#define PIN_SEL_MODE_9_P4_21_MASK	MULTI_BIT_MASK(11, 10)
#define PIN_SEL_MODE_9_P4_21_SHIFT	10
#define PIN_SEL_MODE_9_P4_22_MASK	MULTI_BIT_MASK(13, 12)
#define PIN_SEL_MODE_9_P4_22_SHIFT	12
#define PIN_SEL_MODE_9_P4_23_MASK	MULTI_BIT_MASK(15, 14)
#define PIN_SEL_MODE_9_P4_23_SHIFT	14
#define PIN_SEL_MODE_9_P4_24_MASK	MULTI_BIT_MASK(17, 16)
#define PIN_SEL_MODE_9_P4_24_SHIFT	16
#define PIN_SEL_MODE_9_P4_25_MASK	MULTI_BIT_MASK(19, 18)
#define PIN_SEL_MODE_9_P4_25_SHIFT	18
#define PIN_SEL_MODE_9_P4_26_MASK	MULTI_BIT_MASK(21, 20)
#define PIN_SEL_MODE_9_P4_26_SHIFT	20
#define PIN_SEL_MODE_9_P4_27_MASK	MULTI_BIT_MASK(23, 22)
#define PIN_SEL_MODE_9_P4_27_SHIFT	22
#define PIN_SEL_MODE_9_P4_28_MASK	MULTI_BIT_MASK(25, 24)
#define PIN_SEL_MODE_9_P4_28_SHIFT	24
#define PIN_SEL_MODE_9_P4_29_MASK	MULTI_BIT_MASK(27, 26)
#define PIN_SEL_MODE_9_P4_29_SHIFT	26
#define PIN_SEL_MODE_9_P4_30_MASK	MULTI_BIT_MASK(29, 28)
#define PIN_SEL_MODE_9_P4_30_SHIFT	28
#define PIN_SEL_MODE_9_P4_31_MASK	MULTI_BIT_MASK(31, 30)
#define PIN_SEL_MODE_9_P4_31_SHIFT	30

#endif // XXX

/**
 * grivera: Global declaration of const pointer to the memory-mapped registers of the
 * LPC2478 Pin Connect Block
 */
extern pin_connect_block_t *const g_pin_connect_block;

#endif /* __PINCONNECT_H */
