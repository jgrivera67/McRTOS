#ifndef _SPI_H
#define _SPI_H

/**
 * Base address for the SPI interface
 */
#define LPC2478_SPI_BASE_ADDR    UINT32_C(0xE0020000)

/**
 * Register space size for the SPI
 */
#define LPC2478_SPI_SIZE    UINT32_C(0x20)

/**
 * Memory-mapped I/O registers of the SPI
 */
typedef volatile struct lpc2478_spi
{
    /*
     * SPI Control Register
     */
    uint16_t reg_S0SPCR;
#   define  S0SPCR_RESERVED1_MASK   MULTI_BIT_MASK(1, 0)
#   define  S0SPCR_RESERVED1_SHIFT  0
#   define  S0SPCR_BIT_ENABLE_MASK  BIT(2)
#   define  S0SPCR_CPHA_MASK        BIT(3)
#   define  S0SPCR_CPOL_MASK        BIT(4)
#   define  S0SPCR_MSTR_MASK        BIT(5)
#   define  S0SPCR_LSBF_MASK        BIT(6)
#   define  S0SPCR_SPIE_MASK        BIT(7)
#   define  S0SPCR_BITS_MASK        MULTI_BIT_MASK(11, 8)
#   define  S0SPCR_BITS_SHIFT       8
#   define  S0SPCR_RESERVED2_MASK   MULTI_BIT_MASK(15, 12)
#   define  S0SPCR_RESERVED2_SHIFT  12

    uint16_t padding1;

    /*
     * SPI Status Register
     */
    uint8_t reg_S0SPSR;
#   define  S0SPSR_RESERVED1_MASK   MULTI_BIT_MASK(2, 0)
#   define  S0SPSR_RESERVED1_SHIFT  0
#   define  S0SPSR_ABRT_MASK        BIT(3)
#   define  S0SPSR_MODF_MASK        BIT(4)
#   define  S0SPSR_ROVR_MASK        BIT(5)
#   define  S0SPSR_WCOL_MASK        BIT(6)
#   define  S0SPSR_SPIF_MASK        BIT(7)

    uint8_t padding2;
    uint16_t padding3;

    /*
     * SPI Data Register
     */
    uint8_t reg_S0SPDR_DataLow;
    uint8_t reg_S0SPDR_DataHigh;

    uint16_t padding4;

    /*
     * SPI Clock Counter Register
     */
    uint8_t reg_S0SPCCR_Counter;
    
    uint8_t padding5;
    uint16_t padding6;

    /*
     * SPI Test Control Register
     */
    uint8_t reg_SPTCR;
#   define  SPTCR_RESERVED_MASK BIT(0)
#   define  SPTCR_TEST_MASK     MULTI_BIT_MASK(7, 1)
#   define  SPTCR_TEST_SHIFT    1

    uint8_t padding7;
    uint16_t padding8;

    /*
     * SPI Test Status Register
     */
    uint8_t reg_SPTSR;
#   define  SPTSR_RESERVED_MASK     MULTI_BIT_MASK(2, 0)
#   define  SPTSR_RESERVED_SHIFT    0
#   define  SPTSR_ABRT_MASK         BIT(3)
#   define  SPTSR_MODF_MASK         BIT(4)
#   define  SPTSR_ROVR_MASK         BIT(5)
#   define  SPTSR_WCOL_MASK         BIT(6)
#   define  SPTSR_SPIF_MASK         BIT(7)

    uint8_t padding9;
    uint16_t padding10;
    uint32_t padding11;

    /*
     * SPI Interrupt Register
     */
    uint8_t reg_S0SPINT;
#   define  S0SPINT_SPI_INTERRUPT_FLAG  BIT(0)
#   define  S0SPINT_RESERVED_MASK       MULTI_BIT_MASK(7, 1)
#   define  S0SPINT_RESERVED_SHIFT      1

    uint8_t padding12;
    uint16_t padding13;
} lpc2478_spi_t;

/*
 * Compile-time asserts to verify that the compiler generates
 * the expected offsets for structs that represent groups of memory-mapped
 * I/O registers
 */

C_ASSERT(sizeof(lpc2478_spi_t) == LPC2478_SPI_SIZE);

C_ASSERT(
    LPC2478_SPI_BASE_ADDR + offsetof(lpc2478_spi_t, reg_S0SPCR) == 
    UINT32_C(0xE0020000));

C_ASSERT(
    LPC2478_SPI_BASE_ADDR + offsetof(lpc2478_spi_t, reg_S0SPSR) == 
    UINT32_C(0xE0020004));

C_ASSERT(
    LPC2478_SPI_BASE_ADDR + offsetof(lpc2478_spi_t, reg_S0SPDR_DataLow) == 
    UINT32_C(0xE0020008));

C_ASSERT(
    LPC2478_SPI_BASE_ADDR + offsetof(lpc2478_spi_t, reg_S0SPDR_DataHigh) == 
    UINT32_C(0xE0020009));

C_ASSERT(
    LPC2478_SPI_BASE_ADDR + offsetof(lpc2478_spi_t, reg_S0SPCCR_Counter) == 
    UINT32_C(0xE002000C));

C_ASSERT(
    LPC2478_SPI_BASE_ADDR + offsetof(lpc2478_spi_t, reg_SPTCR) == 
    UINT32_C(0xE0020010));

C_ASSERT(
    LPC2478_SPI_BASE_ADDR + offsetof(lpc2478_spi_t, reg_SPTSR) == 
    UINT32_C(0xE0020014));

C_ASSERT(
    LPC2478_SPI_BASE_ADDR + offsetof(lpc2478_spi_t, reg_S0SPINT) == 
    UINT32_C(0xE002001C));

#endif /* _SPI_H */
