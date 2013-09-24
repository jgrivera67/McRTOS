#ifndef __VIC_H
#define __VIC_H

#include "utils.h"
#include "compile_time_checks.h"
#include "hardware_abstractions.h"
#include "arm_defs.h"

/**
 * grivera: Base address for the LPC2478 VIC
 */
#define LPC2478_VIC_BASE_ADDR    UINT32_C(0xFFFFF000)

/**
 * grivera: Register space size for the VIC
 */
#define LPC2478_VIC_SIZE    UINT32_C(0xF04)

/**
 * grivera: Memory-mapped I/O registers of the LPC2478 VIC
 */
typedef volatile struct lpc2478_vic {
    uint32_t        reg_VICIRQStatus;           // 0x000 RO
    uint32_t        reg_VICFIQStatus;           // 0x004 RO
    uint32_t        reg_VICRawIntr;             // 0x008 RO
    uint32_t        reg_VICIntSelect;           // 0x00C RW
    uint32_t        reg_VICIntEnable;           // 0x010 RW
    uint32_t        reg_VICIntEnClear;          // 0x014 WO
    uint32_t        reg_VICSoftInt;             // 0x018 RW
    uint32_t        reg_VICSoftIntClear;        // 0x01C WO
    uint32_t        reg_VICProtection;          // 0x020 RW
    uint32_t        reg_VICSWPriorityMask;      // 0x024 RW
    uint8_t         reg_padding1[216];     
    isr_function_t  *reg_VICVectAddr[SOC_NUM_INTERRUPT_CHANNELS];
                                                 // 0x100 RW ..
                                                 // 0X17C RW
    uint8_t         reg_padding2[128];     
    uint32_t        reg_VICVectPriority[SOC_NUM_INTERRUPT_CHANNELS];
                                                // 0x200 RW ..
                                                // 0X27C RW
    uint8_t         reg_padding3[3200];     
    uint32_t        reg_VICAddress;             // 0xF00 RW
} lpc2478_vic_t;

/*
 * grivera - Compile-time asserts to verify that the compiler generates
 * the expected offsets for structs that represent groups of memory-mapped
 * I/O registers
 */

C_ASSERT(sizeof(lpc2478_vic_t) == LPC2478_VIC_SIZE);

C_ASSERT(offsetof(lpc2478_vic_t, reg_VICIRQStatus) == 0x000);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICFIQStatus) == 0x004);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICRawIntr) == 0x008);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICIntSelect) == 0x00C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICIntEnable) == 0x010);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICIntEnClear) == 0x014);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICSoftInt) == 0x018);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICSoftIntClear) == 0x01C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICProtection) == 0x020);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICSWPriorityMask) == 0x024);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[0]) == 0x100);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[1]) == 0x104);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[2]) == 0x108);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[3]) == 0x10C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[4]) == 0x110);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[5]) == 0x114);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[6]) == 0x118);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[7]) == 0x11C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[8]) == 0x120);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[9]) == 0x124);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[10]) == 0x128);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[11]) == 0x12C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[12]) == 0x130);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[13]) == 0x134);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[14]) == 0x138);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[15]) == 0x13C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[16]) == 0x140);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[17]) == 0x144);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[18]) == 0x148);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[19]) == 0x14C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[20]) == 0x150);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[21]) == 0x154);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[22]) == 0x158);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[23]) == 0x15C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[24]) == 0x160);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[25]) == 0x164);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[26]) == 0x168);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[27]) == 0x16C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[28]) == 0x170);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[29]) == 0x174);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[30]) == 0x178);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectAddr[31]) == 0x17C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[0]) == 0x200);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[1]) == 0x204);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[2]) == 0x208);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[3]) == 0x20C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[4]) == 0x210);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[5]) == 0x214);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[6]) == 0x218);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[7]) == 0x21C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[8]) == 0x220);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[9]) == 0x224);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[10]) == 0x228);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[11]) == 0x22C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[12]) == 0x230);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[13]) == 0x234);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[14]) == 0x238);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[15]) == 0x23C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[16]) == 0x240);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[17]) == 0x244);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[18]) == 0x248);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[19]) == 0x24C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[20]) == 0x250);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[21]) == 0x254);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[22]) == 0x258);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[23]) == 0x25C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[24]) == 0x260);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[25]) == 0x264);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[26]) == 0x268);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[27]) == 0x26C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[28]) == 0x270);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[29]) == 0x274);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[30]) == 0x278);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICVectPriority[31]) == 0x27C);
C_ASSERT(offsetof(lpc2478_vic_t, reg_VICAddress) == 0xF00);

C_ASSERT(
    LPC2478_VIC_BASE_ADDR + offsetof(lpc2478_vic_t, reg_VICAddress) ==
    LPC2478_VIC_ADDRESS_ADDR);

/**
 * VIC interrupt channel indexes
 */
typedef enum vic_interrupt_channel
{
    VIC_CHANNEL_WDT = 0,        // 0x00
    VIC_CHANNEL_SOFTINT,        // 0x01
    VIC_CHANNEL_ARMCORE0,       // 0x02
    VIC_CHANNEL_ARMCORE1,       // 0x03
    VIC_CHANNEL_TIMER0,         // 0x04
    VIC_CHANNEL_TIMER1,         // 0x05
    VIC_CHANNEL_UART0,          // 0x06
    VIC_CHANNEL_UART1,          // 0x07
    VIC_CHANNEL_PWM0_1,         // 0x08
    VIC_CHANNEL_I2C0,           // 0x09
    VIC_CHANNEL_SPI_SSP0,       // 0x0a
    VIC_CHANNEL_SSP1,           // 0x0b
    VIC_CHANNEL_PLL,            // 0x0c
    VIC_CHANNEL_RTC,            // 0x0d
    VIC_CHANNEL_EINT0,          // 0x0e
    VIC_CHANNEL_EINT1,          // 0x0f
    VIC_CHANNEL_EINT2,          // 0x10
    VIC_CHANNEL_EINT3,          // 0x11
    VIC_CHANNEL_AD0,            // 0x12
    VIC_CHANNEL_I2C1,           // 0x13
    VIC_CHANNEL_BOD,            // 0x14
    VIC_CHANNEL_ETHERNET,       // 0x15
    VIC_CHANNEL_USB,            // 0x16
    VIC_CHANNEL_CAN,            // 0x17
    VIC_CHANNEL_SD_MMC,         // 0x18
    VIC_CHANNEL_GPDMA,          // 0x19
    VIC_CHANNEL_TIMER2,         // 0x1a
    VIC_CHANNEL_TIMER3,         // 0x1b
    VIC_CHANNEL_UART2,          // 0x1c
    VIC_CHANNEL_UART3,          // 0x1d
    VIC_CHANNEL_I2C2,           // 0x1e
    VIC_CHANNEL_I2S             // 0x1f
} vic_interrupt_channel_t;

C_ASSERT(VIC_CHANNEL_I2S == SOC_NUM_INTERRUPT_CHANNELS - 1);

/*
 * Bit masks for the VICIntEnable and VICIntEnClear registers
 */ 

#define VIC_CHANNEL_MASK(_channel)  BIT(_channel)

#define ALL_VIC_CHANNELS_MASK       UINT32_MAX

C_ASSERT(ALL_VIC_CHANNELS_MASK == 0xFFFFFFFF);

/*
 * Bit masks for the VICProtection registers
 */ 
#define VIC_ACCESS_MASK             BIT(0)

/**
 * VIC interrupt channel priorities
 */

typedef enum vic_vector_priority
{
    VIC_VECT_PRIORITY0 = 0,     // 0x0 - highest priority
    VIC_VECT_PRIORITY1,         // 0x1
    VIC_VECT_PRIORITY2,         // 0x2
    VIC_VECT_PRIORITY3,         // 0x3
    VIC_VECT_PRIORITY4,         // 0x4
    VIC_VECT_PRIORITY5,         // 0x5
    VIC_VECT_PRIORITY6,         // 0x6
    VIC_VECT_PRIORITY7,         // 0x7
    VIC_VECT_PRIORITY8,         // 0x8
    VIC_VECT_PRIORITY9,         // 0x9
    VIC_VECT_PRIORITY10,        // 0xa
    VIC_VECT_PRIORITY11,        // 0xb
    VIC_VECT_PRIORITY12,        // 0xc
    VIC_VECT_PRIORITY13,        // 0xd
    VIC_VECT_PRIORITY14,        // 0xe
    VIC_VECT_PRIORITY15         // 0xf - lowest priority
} vic_interrupt_priority_t;

C_ASSERT(VIC_VECT_PRIORITY15 == 15);

#define VIC_VECT_PRIORITY_HIGHEST               VIC_VECT_PRIORITY0
#define VIC_VECT_PRIORITY_MEDIUM                VIC_VECT_PRIORITY7
#define VIC_VECT_PRIORITY_LOWEST                VIC_VECT_PRIORITY15 

/*
 * Bit masks for the VICSWPriorityMask register
 */

#define VIC_VECT_PRIORITY_MASK(_vic_priority)   BIT(_vic_priority)

#define ALL_VIC_PRIORITIES_MASK                 UINT16_MAX

C_ASSERT(ALL_VIC_PRIORITIES_MASK == 0x0000FFFF);


typedef enum arminterrupt 
{
    IntSelectIRQ = 0,
    IntSelectFIQ
} arminterrupt_t;

#endif /* __VIC_H */
