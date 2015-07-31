#ifndef _SSP_H
#define _SSP_H

/**
 * Base address for the SSP interface
 */
#define LPC2478_SSP0_BASE_ADDR    UINT32_C(0xE0068000)
#define LPC2478_SSP1_BASE_ADDR    UINT32_C(0xE0030000)

/**
 * Register space size for the SSP
 */
#define LPC2478_SSP_SIZE    UINT32_C(0x28)

/**
 * Memory-mapped I/O registers of the SSP
 */
typedef volatile struct lpc2478_ssp
{
    /*
     * SSP Control Register 0
     */
    uint16_t reg_CR0;
#   define  SSP_CR0_DSS_MASK     MULTI_BIT_MASK(3, 0)
#   define  SSP_CR0_DSS_SHIFT    0
#   define  SSP_CR0_FRF_MASK     MULTI_BIT_MASK(5, 4)
#   define  SSP_CR0_FRF_SHIFT    4
#   define  SSP_CR0_CPOL_MASK    BIT(6)
#   define  SSP_CR0_CPHA_MASK    BIT(7)
#   define  SSP_CR0_SCR_MASK     MULTI_BIT_MASK(15, 8)

    uint16_t padding1;

    /*
     * SSP Control Register 1
     */
    uint8_t reg_CR1;
#   define  SSP_CR1_LBM_MASK        BIT(0)
#   define  SSP_CR1_SSE_MASK        BIT(1)
#   define  SSP_CR1_MS_MASK         BIT(2)
#   define  SSP_CR1_SOD_MASK        BIT(3)
#   define  SSP_CR1_RESERVED_MASK   MULTI_BIT_MASK(7, 4)
#   define  SSP_CR1_RESERVED_SHIFT  4

    uint8_t padding2;
    uint16_t padding3;

    /*
     * SSP Data Register
     */
    uint16_t reg_DR_data;

    uint16_t padding4;

    /*
     * SSP Status Register
     */
    uint8_t reg_SR;
#   define  SSP_SR_TFE_MASK         BIT(0)
#   define  SSP_SR_TNF_MASK         BIT(1)
#   define  SSP_SR_RNE_MASK         BIT(2)
#   define  SSP_SR_RFF_MASK         BIT(3)
#   define  SSP_SR_BSY_MASK         BIT(4)
#   define  SSP_SR_RESERVED_MASK    MULTI_BIT_MASK(7, 5)
#   define  SSP_SR_RESERVED_SHIFT   5

    uint8_t padding5;
    uint16_t padding6;

    /*
     * SSP Clock Prescale Register
     */
    uint8_t reg_CPSR_CPSDVSR;

    uint8_t padding7;
    uint16_t padding8;

    /*
     * SSP Interrupt Mask Set and Clear Register
     */
    uint8_t reg_IMSC;
#   define  SSP_IMSC_RORIM_MASK     BIT(0)
#   define  SSP_IMSC_RTIM_MASK      BIT(1)
#   define  SSP_IMSC_RXIM_MASK      BIT(2)
#   define  SSP_IMSC_TXIM_MASK      BIT(3)
#   define  SSP_IMSC_RESERVED_MASK  MULTI_BIT_MASK(7, 4)
#   define  SSP_IMSC_RESERVED_SHIFT 4

    uint8_t padding9;
    uint16_t padding10;

    /*
     * SSP Raw Interrupt Status Register
     */
    uint8_t reg_RIS;
#   define  SSP_RIS_RORRIS_MASK     BIT(0)
#   define  SSP_RIS_RTRIS_MASK      BIT(1)
#   define  SSP_RIS_RXRIS_MASK      BIT(2)
#   define  SSP_RIS_TXRIS_MASK      BIT(3)
#   define  SSP_RIS_RESERVED_MASK   MULTI_BIT_MASK(7, 4)
#   define  SSP_RIS_RESERVED_SHIFT  4

    uint8_t padding11;
    uint16_t padding12;

    /*
     * SSP Masked Interrupt Status Register
     */
    uint8_t reg_MIS;
#   define  SSP_MIS_RORMIS_MASK     BIT(0)
#   define  SSP_MIS_RTMIS_MASK      BIT(1)
#   define  SSP_MIS_RXMIS_MASK      BIT(2)
#   define  SSP_MIS_TXMIS_MASK      BIT(3)
#   define  SSP_MIS_RESERVED_MASK   MULTI_BIT_MASK(7, 4)
#   define  SSP_MIS_RESERVED_SHIFT  4

    uint8_t padding13;
    uint16_t padding14;

    /*
     * SSP SSPICR Interrupt Clear Register
     */
    uint8_t reg_ICR;
#   define  SSP_ICR_RORIC_MASK      BIT(0)
#   define  SSP_ICR_RTIC_MASK       BIT(1)
#   define  SSP_ICR_RESERVED_MASK   MULTI_BIT_MASK(7, 2)
#   define  SSP_ICR_RESERVED_SHIFT  2

    uint8_t padding15;
    uint16_t padding16;

    /*
     * SSP DMA Control Register
     */
    uint16_t reg_DMACR;
#   define  SSP_DMACR_RXDMAE_MASK       BIT(0)
#   define  SSP_DMACR_TXDMAE_MASK       BIT(1)
#   define  SSP_DMACR_RESERVED_MASK     MULTI_BIT_MASK(15, 2)
#   define  SSP_DMACR_RESERVED_SHIFT    2

    uint16_t padding17;
} lpc2478_ssp_t;

/*
 * Compile-time asserts to verify that the compiler generates
 * the expected offsets for structs that represent groups of memory-mapped
 * I/O registers
 */

C_ASSERT(sizeof(lpc2478_ssp_t) == LPC2478_SSP_SIZE);

C_ASSERT(offsetof(lpc2478_ssp_t, reg_CR0) == 0x00);
C_ASSERT(offsetof(lpc2478_ssp_t, reg_CR1) == 0x04);
C_ASSERT(offsetof(lpc2478_ssp_t, reg_DR_data) == 0x08);
C_ASSERT(offsetof(lpc2478_ssp_t, reg_SR) == 0x0C);
C_ASSERT(offsetof(lpc2478_ssp_t, reg_CPSR_CPSDVSR) == 0x10);
C_ASSERT(offsetof(lpc2478_ssp_t, reg_IMSC) == 0x14);
C_ASSERT(offsetof(lpc2478_ssp_t, reg_RIS) == 0x18);
C_ASSERT(offsetof(lpc2478_ssp_t, reg_MIS) == 0x1C);
C_ASSERT(offsetof(lpc2478_ssp_t, reg_ICR) == 0x20);
C_ASSERT(offsetof(lpc2478_ssp_t, reg_DMACR) == 0x24);

/**
 * Values for the SSP_CR0_FRF field of reg_CR0
 */
enum ssp_frame_format
{
    SSP_FRF_SPI =       0x0,
    SSP_FRF_TI =        0x1,
    SSP_FRF_Microwire = 0x2,
    SSP_FRF_Invalid =   0x3
};

#define SSP_TRANSMIT_FIFO_SIZE_IN_ENTRIES   8
#define SSP_RECEIVE_FIFO_SIZE_IN_ENTRIES    8

/**
 * Maximum SSP clock frequency in Hz
 */
#define MAX_SSP_CLOCK_FREQUENCY (UINT32_C(4) * 1000 * 1000)   /* 4 MHz */

#endif /* _SSP_H */
