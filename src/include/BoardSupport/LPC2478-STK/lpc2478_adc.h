#ifndef _ADC_H
#define _ADC_H

/**
 * Base address for the LPC2478 A/D Converter (ADC)
 */
#define LPC2478_ADC_BASE_ADDR    UINT32_C(0xE0034000)

/**
 * Register space size for the ADC
 */
#define LPC2478_ADC_SIZE    UINT32_C(0x34)

/**
 * Number of A/D converter channels
 */
#define NUM_ADC_CHANNELS 8

/**
 * Memory-mapped I/O registers of the ADC
 */
typedef volatile struct lpc2478_adc
{
    uint32_t reg_AD0CR;
#   define  AD0CR_SEL_MASK	    MULTI_BIT_MASK(7, 0)
#   define  AD0CR_SEL_SHIFT         0
#   define  AD0CR_CLKDIV_MASK	    MULTI_BIT_MASK(15, 8)
#   define  AD0CR_CLKDIV_SHIFT      8
#   define  AD0CR_BURST_MASK	    BIT(16)
#   define  AD0CR_CLKS_MASK	    MULTI_BIT_MASK(19, 17)
#   define  AD0CR_CLKS_SHIFT        17
#   define  AD0CR_RESERVED1_MASK    BIT(20)    
#   define  AD0CR_PDN_MASK	    BIT(21)
#   define  AD0CR_RESERVED2_MASK    MULTI_BIT_MASK(23, 22)
#   define  AD0CR_RESERVED2_SHIFT   22
#   define  AD0CR_START_MASK	    MULTI_BIT_MASK(26, 24)
#   define  AD0CR_START_SHIFT       24
#   define  AD0CR_EDGE_MASK	    BIT(27)
#   define  AD0CR_RESERVED3_MASK    MULTI_BIT_MASK(31, 28)
#   define  AD0CR_RESERVED3_SHIFT   28

    uint32_t reg_AD0GDR;
#   define  AD0GDR_V_OVER_VREF_MASK    MULTI_BIT_MASK(15, 6)
#   define  AD0GDR_V_OVER_VREF_SHIFT   6
#   define  AD0GDR_CHANNEL_MASK        MULTI_BIT_MASK(26, 24)
#   define  AD0GDR_CHANNEL_SHIFT       24
#   define  AD0GDR_OVERRUN_MASK        BIT(30)
#   define  AD0GDR_DONE_MASK           BIT(31)

    uint32_t padding1;

    uint32_t reg_AD0INTEN;
#   define  AD0INTEN_ADINTEN_MASK   MULTI_BIT_MASK(7, 0)
#   define  AD0INTEN_ADINTEN_SHIFT  0
#   define  AD0INTEN_ADGINTEN_MASK  BIT(8)

    uint32_t reg_AD0DR[NUM_ADC_CHANNELS];
#   define  AD0DR_V_OVER_VREF_MASK    MULTI_BIT_MASK(15, 6)
#   define  AD0DR_V_OVER_VREF_SHIFT   6
#   define  AD0DR_OVERRUN_MASK        BIT(30)
#   define  AD0DR_DONE_MASK           BIT(31)

    uint32_t reg_AD0STAT;
#   define  AD0STAT_DONE_MASK       MULTI_BIT_MASK(7, 0)
#   define  AD0STAT_DONE_SHIFT      0
#   define  AD0STAT_OVERRRUN_MASK   MULTI_BIT_MASK(15, 8)
#   define  AD0STAT_OVERRUN_SHIFT   8
#   define  AD0STAT_ADINT_MASK      BIT(16)
} lpc2478_adc_t;

/*
 * Compile-time asserts to verify that the compiler generates
 * the expected offsets for structs that represent groups of memory-mapped
 * I/O registers
 */

C_ASSERT(sizeof(lpc2478_adc_t) == LPC2478_ADC_SIZE);

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0CR) == 
    UINT32_C(0xE0034000));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0GDR) == 
    UINT32_C(0xE0034004));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0INTEN) == 
    UINT32_C(0xE003400C));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0DR[0]) == 
    UINT32_C(0xE0034010));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0DR[1]) == 
    UINT32_C(0xE0034014));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0DR[2]) == 
    UINT32_C(0xE0034018));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0DR[3]) == 
    UINT32_C(0xE003401C));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0DR[4]) == 
    UINT32_C(0xE0034020));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0DR[5]) == 
    UINT32_C(0xE0034024));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0DR[6]) == 
    UINT32_C(0xE0034028));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0DR[7]) == 
    UINT32_C(0xE003402C));

C_ASSERT(
    LPC2478_ADC_BASE_ADDR + offsetof(lpc2478_adc_t, reg_AD0STAT) == 
    UINT32_C(0xE0034030));

/**
 * Maximum A/D converter clock frequency in Hz
 */
#define MAX_ADC_CLOCK_FREQUENCY \
        ((UINT32_C(4) * 1000 * 1000) + (500 * 1000))    /* 4.5 MHz */

#endif /* _ADC_H */
