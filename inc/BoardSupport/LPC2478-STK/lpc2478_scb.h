#ifndef __SCB_H
#define __SCB_H

#include <inttypes.h>   /* C99 int types */
#include "compile_time_checks.h"

/**
 * grivera: Base address for the LPC2478 SCB
 */
#define LPC2478_SCB_BASE_ADDR   UINT32_C(0xE01FC000)

/**
 * grivera: Register space size for the SCB
 */
#define LPC2478_SCB_SIZE    UINT32_C(0x1b0)

/**
 * grivera: Memory-mapped I/O registers of the LPC2478 System Control Block (SCB).
 */
typedef volatile struct lpc2478_scb {
    uint8_t     reg_MAMCR;            /* RW     0x000 */
    uint8_t     reg_padding1[3];
    uint8_t     reg_MAMTIM;           /* RW     0x004 */
    uint8_t     reg_padding2[59];
    uint8_t     reg_MEMMAP;           /* RW     0x040 */
    uint8_t     reg_padding3[63];
    uint8_t     reg_PLLCON;           /* RW     0x080 */
    uint8_t     reg_padding4[3];
    uint32_t    reg_PLLCFG;           /* RW     0x084 */
    uint32_t    reg_PLLSTAT;          /* R      0x088 */
    uint8_t     reg_PLLFEED;          /* W      0x08C */
    uint8_t     reg_padding5[51];
    uint8_t     reg_PCON;             /* RW     0x0C0 */
#   define      SCB_PCON_PM0_MASK       BIT(0)
#   define      SCB_PCON_PM1_MASK       BIT(1)
#   define      SCB_PCON_BODPDM_MASK    BIT(2)
#   define      SCB_PCON_BOGD_MASK      BIT(3)
#   define      SCB_PCON_BORD_MASK      BIT(4)
#   define      SCB_PCON_RESERVED_MASK  MULTI_BIT_MASK(6, 5)
#   define      SCB_PCON_PM2_MASK       BIT(7)

    uint8_t     reg_padding6[3];
    uint32_t    reg_PCONP;            /* RW     0x0C4 */
    uint8_t     reg_padding7[60];
    uint8_t     reg_CCLKCFG;          /* RW     0x104 */
    uint8_t     reg_padding8[3];
    uint8_t     reg_USBCLKCFG;        /* RW     0x108 */
    uint8_t     reg_padding9[3];
    uint8_t     reg_CLKSRCSEL;        /* RW     0x10C */
    uint8_t     reg_padding10[51];
    uint8_t     reg_EXTINT;           /* RW     0x140 */
    uint8_t     reg_padding11[3];
    uint16_t    reg_INTWAKE;          /* RW     0x144 */
    uint8_t     reg_padding12[2];
    uint8_t     reg_EXTMODE;          /* RW     0x148 */
    uint8_t     reg_padding13[3];
    uint8_t     reg_EXTPOLAR;         /* RW     0x14C */
    uint8_t     reg_padding14[51];
    uint8_t     reg_RSID;             /* RW     0x180 */
    uint8_t     reg_padding15[3];
    uint32_t    reg_CSPR;             /* WO     0x184 */
    uint32_t    reg_AHBCFG1;          /* RW     0x188 */
    uint32_t    reg_AHBCFG2;          /* RW     0x18C */
    uint8_t     reg_padding16[16];
    uint32_t    reg_SCS;              /* RW     0x1A0 */
    uint16_t    reg_IRCTRIM;          /* RW     0x1A4 */
    uint8_t     reg_padding17[2];
    uint32_t    reg_PCLKSEL[2];       /* RW     0x1A8  */
                                      /* RW     0x1AC */
} lpc2478_scb_t;

/*
 * grivera - Compile-time asserts to verify that the compiler generates
 * the expected offsets for structs that represent groups of memory-mapped
 * I/O registers
 */

C_ASSERT(sizeof(lpc2478_scb_t) == LPC2478_SCB_SIZE);

C_ASSERT(offsetof(lpc2478_scb_t, reg_MAMCR) == 0x00);
C_ASSERT(offsetof(lpc2478_scb_t, reg_MAMTIM) == 0x04);
C_ASSERT(offsetof(lpc2478_scb_t, reg_MEMMAP) == 0x040);
C_ASSERT(offsetof(lpc2478_scb_t, reg_PLLCON) == 0x080);
C_ASSERT(offsetof(lpc2478_scb_t, reg_PLLCFG) == 0x084);
C_ASSERT(offsetof(lpc2478_scb_t, reg_PLLSTAT) == 0x088);
C_ASSERT(offsetof(lpc2478_scb_t, reg_PLLFEED) == 0x08C);
C_ASSERT(offsetof(lpc2478_scb_t, reg_PCON) == 0x0C0);
C_ASSERT(offsetof(lpc2478_scb_t, reg_PCONP) == 0x0C4);
C_ASSERT(offsetof(lpc2478_scb_t, reg_CCLKCFG) == 0x104);
C_ASSERT(offsetof(lpc2478_scb_t, reg_USBCLKCFG) == 0x108);
C_ASSERT(offsetof(lpc2478_scb_t, reg_CLKSRCSEL) == 0x10C);
C_ASSERT(offsetof(lpc2478_scb_t, reg_EXTINT) == 0x140);
C_ASSERT(offsetof(lpc2478_scb_t, reg_INTWAKE) == 0x144);
C_ASSERT(offsetof(lpc2478_scb_t, reg_EXTMODE) == 0x148);
C_ASSERT(offsetof(lpc2478_scb_t, reg_EXTPOLAR) == 0x14C);
C_ASSERT(offsetof(lpc2478_scb_t, reg_RSID) == 0x180);
C_ASSERT(offsetof(lpc2478_scb_t, reg_CSPR) == 0x184);
C_ASSERT(offsetof(lpc2478_scb_t, reg_AHBCFG1) == 0x188);
C_ASSERT(offsetof(lpc2478_scb_t, reg_AHBCFG2) == 0x18C);
C_ASSERT(offsetof(lpc2478_scb_t, reg_SCS) == 0x1A0);
C_ASSERT(offsetof(lpc2478_scb_t, reg_IRCTRIM) == 0x1A4);
C_ASSERT(offsetof(lpc2478_scb_t, reg_PCLKSEL[0]) == 0x1A8);
C_ASSERT(offsetof(lpc2478_scb_t, reg_PCLKSEL[1]) == 0x1AC);

/**
 * grivera: Global declaration of const pointer to the LPC2478 SCB registers
 */
extern lpc2478_scb_t *const g_scb_mmio_registers_p;

/*
 * MEMMAP
 */
#define BOOTLOADERMODE     0x0
#define USERFLASHMODE      0x1
#define USERRAMMODE        0x2
#define EXTERNALMEMORYMODE 0x3

/* MAM */
#define MAMCR_OFF          0
#define MAMCR_PARTIAL      1
#define MAMCR_FULL         2
//#define MAMTIM_CYCLES      (((CCLK)+19999999)/20000000)
#define MAMTIM_AUTOCFG     0
#define MAMTIM_1_CLK       1
#define MAMTIM_2_CLK       2
#define MAMTIM_3_CLK       3
#define MAMTIM_4_CLK       4
#define MAMTIM_5_CLK       5
#define MAMTIM_6_CLK       6
#define MAMTIM_7_CLK       7
#define MAMTIM_MAX_CLK     7



#define CCLK_DIV_1         (1)
#define CCLK_DIV_2         (2)
#define CCLK_DIV_4         (4)
#define CCLK_DIV_6         (6)
#define CCLK_DIV_8         (8)
#define CCLK_DIV_10        (10)
#define CCLK_DIV_12        (12)
#define CCLK_DIV_14        (14)
#define CCLK_DIV_16        (16)
#define CCLK_DIV_18        (18)
#define CCLK_DIV_20        (20)
#define CCLK_DIV_22        (22)
#define CCLK_DIV_24        (24)
#define CCLK_DIV_26        (26)
#define CCLK_DIV_28        (28)
//#define CCLK               (FCCO/CCLK_DIV)
//#define CCLKCFG_VALUE      (CCLK_DIV-1)
//#define PBSD               1
//#define PCLK               (CCLK / PBSD)


#define FOSC_INT_RC        (4000000)

#define CLKSRC_INT_RC      (0x00)
#define CLKSRC_MAIN_OSC    (0x01)
#define CLKSRC_MIN_RTC     (0x02)
#define CLKSRCSEL_MASK     (0x03)

#define SCS_GPIOM          (1<<0)
#define SCS_EMC_RST_DIS    (1<<1)
#define SCS_MCIPWR         (1<<3)
#define SCS_OSCRANGE       (1<<4)
#define SCS_OSCEN          (1<<5)
#define SCS_OSCSTAT        (1<<6)

//#define FCCO               (FOSC*PLL_MUL*2/PLL_DIV)
#define PLLCON_PLLE        (1<<0)
#define PLLCON_PLLC        (1<<1)
#define PLLSTAT_PLLE       (1<<24)
#define PLLSTAT_PLLC       (1<<25)
#define PLLSTAT_PLOCK      (1<<26)
#define PLLSTAT_MSEL_MASK      (0x00007FFF)
#define PLLSTAT_NSEL_MASK      (0x00FF0000)

#define PDIV_1             0x1
#define PDIV_2             0x2
#define PDIV_4             0x0
#define PDIV_6             0x3
#define PDIV_8             0x3

#define CLR_PCLK(per) ((0x3) << (per))
#define SET_PCLK(per,div) ((div) << (per))
#define GET_PCLK_SEL(PCLKSELn,per)     ( ( (PCLKSELn) >> (per) ) & 0x00000003 )

#define PCLK_WDT           0
#define PCLK_TIMER0        2
#define PCLK_TIMER1        4
#define PCLK_UART0         6
#define PCLK_UART1         8
#define PCLK_PWM1          12
#define PCLK_I2C0          14
#define PCLK_SPI           16
#define PCLK_RTC           18
#define PCLK_SSP1          20
#define PCLK_DAC           22
#define PCLK_ADC           24
#define PCLK_CAN1          26
#define PCLK_CAN2          28
#define PCLK_ACF           30

#define PCLK_BAT_RAM       0
#define PCLK_GPIO          2
#define PCLK_PCB           4
#define PCLK_I2C1          6
#define PCLK_SSP0          10
#define PCLK_TIMER2        12
#define PCLK_TIMER3        14
#define PCLK_UART2         16
#define PCLK_UART3         18
#define PCLK_I2C2          20
#define PCLK_I2S           22
#define PCLK_MCI           24
#define PCLK_SYSCON        28

/*
 * Macros to set power modes for the LPC2478
 * (See table 4-61 in the LPC2478 manual)
 */

#define SET_SCB_PCON_NORMAL_MODE(_pcon_variable) \
    do {                                                                \
        (_pcon_variable) &=                                             \
        ~(SCB_PCON_PM0_MASK | SCB_PCON_PM1_MASK | SCB_PCON_PM2_MASK);   \
    } while (0)

#define SET_SCB_PCON_IDLE_MODE(_pcon_variable) \
    do {                                                                \
        (_pcon_variable) |= SCB_PCON_PM0_MASK;                          \
        (_pcon_variable) &= ~(SCB_PCON_PM1_MASK | SCB_PCON_PM2_MASK);   \
    } while (0)

#define SET_SCB_PCON_SLEEP_MODE(_pcon_variable) \
    do {                                                                \
        (_pcon_variable) |= (SCB_PCON_PM0_MASK | SCB_PCON_PM2_MASK);    \
        (_pcon_variable) &= ~SCB_PCON_PM1_MASK;                         \
    } while (0)

#define SET_SCB_PCON_POWER_DOWN_MODE(_pcon_variable) \
    do {                                                                \
        (_pcon_variable) |= SCB_PCON_PM1_MASK;                          \
        (_pcon_variable) &= ~(SCB_PCON_PM0_MASK | SCB_PCON_PM2_MASK);   \
    } while (0)

#define SET_SCB_PCON_DEEP_POWER_DOWN_MODE(_pcon_variable) \
    do {                                                                \
        (_pcon_variable) |= (SCB_PCON_PM1_MASK | SCB_PCON_PM2_MASK);    \
        (_pcon_variable) &= ~SCB_PCON_PM0_MASK;                         \
    } while (0)


#define PCONP_PCTIM0    BIT(1)
#define PCONP_PCTIM1    BIT(2)
#define PCONP_PCUART0   BIT(3)
#define PCONP_PCUART1   BIT(4)
#define PCONP_PCPWM0    BIT(5)
#define PCONP_PCPWM1    BIT(6)
#define PCONP_PCI2C0    BIT(7)
#define PCONP_PCSPI     BIT(8)
#define PCONP_PCRTC     BIT(9)
#define PCONP_PCSSP1    BIT(10)
#define PCONP_PCEMC     BIT(11)
#define PCONP_PCADC     BIT(12)
#define PCONP_PCCAN1    BIT(13)
#define PCONP_PCCAN2    BIT(14)
#define PCONP_RESERVED_MASK   MULTI_BIT_MASK(18, 15)
#define PCONP_RESERVED_SHIFT  15
#define PCONP_PCI2C1    BIT(19)
#define PCONP_PCLCD     BIT(20)
#define PCONP_PCSSP0    BIT(21)
#define PCONP_PCTIM2    BIT(22)
#define PCONP_PCTIM3    BIT(23)
#define PCONP_PCUART2   BIT(24)
#define PCONP_PCUART3   BIT(25)
#define PCONP_PCI2C2    BIT(26)
#define PCONP_PCI2S     BIT(27)
#define PCONP_PCSDC     BIT(28)
#define PCONP_PCGPDMA   BIT(29)
#define PCONP_PCENET    BIT(30)
#define PCONP_PCUSB     BIT(31)

#endif /* __SCB_H */
