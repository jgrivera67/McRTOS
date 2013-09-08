#ifndef __TIMERS_H
#define __TIMERS_H

#include "utils.h"
#include "compile_time_checks.h"

/**
 * grivera: Base addresses for the LPC2478's timers
 */
#define LPC2478_TIMER0_BASE_ADDR    UINT32_C(0xE0004000)
#define LPC2478_TIMER1_BASE_ADDR    UINT32_C(0xE0008000)
#define LPC2478_TIMER2_BASE_ADDR    UINT32_C(0xE0070000)
#define LPC2478_TIMER3_BASE_ADDR    UINT32_C(0xE0074000)

/**
 * grivera: Register space size for an individual timer
 */
#define LPC2478_TIMER_SIZE    UINT32_C(0x74)

/**
 * grivera: Memory-mapped I/O registers of an individual timer
 */
typedef volatile struct lpc2478_timer {
    uint8_t   reg_IR;               /* RW     0x00 */
    uint8_t   reg_padding0[3];
    uint8_t   reg_TCR;              /* RW     0x04 */
    uint8_t   reg_padding1[3];
    uint32_t  reg_TC;               /* RW     0x08 */
    uint32_t  reg_PR;               /* RW     0x0C */
    uint32_t  reg_PC;               /* RW     0x10 */
    uint16_t  reg_MCR;              /* RW     0x14 */
    uint8_t   reg_padding2[2];
    uint32_t  reg_MR[4];            /* RW     0x18 .. */
                                    /* RW     0x24 */
    uint16_t  reg_CCR;              /* RW     0x28 */
    uint8_t   reg_padding3[2];
    uint32_t  reg_CR[4];            /* RO     0x2C */
                                    /* R0     0x38 */
    uint16_t  reg_EMR;              /* RW     0x3C */
    uint8_t   reg_padding4[50];
    uint8_t   reg_CTCR;             /* RW     0x70 */
    uint8_t   reg_padding5[3];
} lpc2478_timer_t;

/*
 * grivera - Compile-time asserts to verify that the compiler generates
 * the expected offsets for structs that represent groups of memory-mapped
 * I/O registers
 */

C_ASSERT(sizeof(lpc2478_timer_t) == LPC2478_TIMER_SIZE);

C_ASSERT(offsetof(lpc2478_timer_t, reg_IR) == 0x00);
C_ASSERT(offsetof(lpc2478_timer_t, reg_TCR) == 0x04);
C_ASSERT(offsetof(lpc2478_timer_t, reg_TC) == 0x08);
C_ASSERT(offsetof(lpc2478_timer_t, reg_PR) == 0x0C);
C_ASSERT(offsetof(lpc2478_timer_t, reg_PC) == 0x10);
C_ASSERT(offsetof(lpc2478_timer_t, reg_MCR) == 0x14);
C_ASSERT(offsetof(lpc2478_timer_t, reg_MR[0]) == 0x18);
C_ASSERT(offsetof(lpc2478_timer_t, reg_MR[1]) == 0x1C);
C_ASSERT(offsetof(lpc2478_timer_t, reg_MR[2]) == 0x20);
C_ASSERT(offsetof(lpc2478_timer_t, reg_MR[3]) == 0x24);
C_ASSERT(offsetof(lpc2478_timer_t, reg_CCR) == 0x28);
C_ASSERT(offsetof(lpc2478_timer_t, reg_CR[0]) == 0x2C);
C_ASSERT(offsetof(lpc2478_timer_t, reg_CR[1]) == 0x30);
C_ASSERT(offsetof(lpc2478_timer_t, reg_CR[2]) == 0x34);
C_ASSERT(offsetof(lpc2478_timer_t, reg_CR[3]) == 0x38);
C_ASSERT(offsetof(lpc2478_timer_t, reg_EMR) == 0x3C);
C_ASSERT(offsetof(lpc2478_timer_t, reg_CTCR) == 0x70);

/*
 * Bit masks for TCR register
 */ 
#define TCR_COUNTER_ENABLE_MASK BIT(0)
#define TCR_COUNTER_RESET_MASK  BIT(1)
#define TCR_RESERVED_MASK       MULTI_BIT_MASK(7, 2)
#define TCR_RESERVED_SHIFT      2
    
/*
 * Bit masks for CTCR register
 */ 
#define CTCR_COUNTER_TIMER_MODE_MASK  MULTI_BIT_MASK(1, 0)
#define CTCR_COUNT_INPUT_SELECT_MASK  MULTI_BIT_MASK(3, 2)

/* 
 * Values for the CTCR counter/timer mode field
 */
enum ctcr_timer_mode_values
{
    /*
     * Timer mode: every rising PCLK edge
     */
    CTCR_TIMER_MODE = UINT8_C(0x0),

    /*
     * Counter Mode: TC is incremented on rising edges on the
     * CAP input selected by bits 3:2.
     */
    CTCR_COUNTER_MODE_RISING_EDGE = UINT8_C(0x1),

    /*
     * Counter Mode: TC is incremented on falling edges on the
     * CAP input selected by bits 3:2.
     */
    CTCR_COUNTER_MODE_FALLING_EDGE = UINT8_C(0x2),

    /*
     * Counter Mode: TC is incremented on both edges on the
     * CAP input selected by bits 3:2.
     */
    CTCR_COUNTER_MODE_BOTH_EDGES = UINT8_C(0x3),
};

/* 
 * Values for the CTCR count input select field
 */
enum ctcr_count_input_select_values
{
    /*
     * Select CAPn.0 pin for TIMERn
     */
    CTCR_COUNT_INPUT_SELECT_CAP0 = UINT8_C(0x0),

    /*
     * Select CAPn.1 pin for TIMERn
     */
    CTCR_COUNT_INPUT_SELECT_CAP1 = UINT8_C(0x1),
};

/*
 * Bit masks for IR register
 */ 
#define IR_MR0_INTERRUPT_MASK    BIT(0)
#define IR_MR1_INTERRUPT_MASK    BIT(1)
#define IR_MR2_INTERRUPT_MASK    BIT(2)
#define IR_MR3_INTERRUPT_MASK    BIT(3)
#define IR_CR0_INTERRUPT_MASK    BIT(4)
#define IR_CR1_INTERRUPT_MASK    BIT(5)
#define IR_CR2_INTERRUPT_MASK    BIT(6)
#define IR_CR3_INTERRUPT_MASK    BIT(7)

/*
 * Bit masks for MCR register
 */ 
#define MCR_MR0I_MASK   BIT(0)
#define MCR_MR0R_MASK   BIT(1)
#define MCR_MR0S_MASK   BIT(2)
#define MCR_MR1I_MASK   BIT(3)
#define MCR_MR1R_MASK   BIT(4)
#define MCR_MR1S_MASK   BIT(5)
#define MCR_MR2I_MASK   BIT(6)
#define MCR_MR2R_MASK   BIT(7)
#define MCR_MR2S_MASK   BIT(8)
#define MCR_MR3I_MASK   BIT(9)
#define MCR_MR3R_MASK   BIT(10)
#define MCR_MR3S_MASK   BIT(11)

#endif /* __TIMERS_H */
