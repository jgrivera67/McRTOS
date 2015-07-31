#ifndef __UARTS_H
#define __UARTS_H

#include "utils.h"
#include "failure_data_capture.h"

/**
 * grivera: Base addresses for the LPC2478 UARTs
 */
#define LPC2478_UART0_BASE_ADDR    UINT32_C(0xE000C000)
#define LPC2478_UART1_BASE_ADDR    UINT32_C(0xE0010000)
#define LPC2478_UART2_BASE_ADDR    UINT32_C(0xE0078000)
#define LPC2478_UART3_BASE_ADDR    UINT32_C(0xE007C000)

/**
 * grivera: Register space size for a single UART
 */
#define LPC2478_UART_SIZE    UINT32_C(0x34)

/**
 * grivera: Memory-mapped I/O registers of a single UART
 */
typedef volatile struct lpc2478_uart {
    union {                         /* Offset 0x00 */
        /**
         * Receive Buffer Register (read-only)
         *
         * This register must be read only if the RDR bit in the LSR is set.
         * This register returns the oldest character in the receive FIFO.
         * The RDR bit in the LSR is set when this register holds an unread
         * character and is cleared when the BR FIFO is empty.
         *
         * NOTE: The Divisor Latch Access Bit (DLAB) in LCR must be zero in
         * order to access this register.
         */
        uint8_t reg_RBR;            /* RO */

        /**
         * Transmit Holding Register (write-only)
         *
         * This register must be written only if the THRE bit in the LSR is set.
         */
        uint8_t reg_THR;            /* WO */

        /**
         * Divisor Latch LSB 
         */
        uint8_t reg_DLL;            /* RW if DLAB = 1 */
    };

    /* 
     * NOTE: No padding is necessary after a union or struct, as padding
     * at the end is automatically added by the compiler
     */

    union {                         /* Offset 0x04 */
        struct {
            /**
             * Divisor Latch MSB 
             */
            uint8_t reg_DLM;
            uint8_t reg_padding1[3];
        };

        /**
         * Interrupt Enable Register
         * bit i set means interrupt i enabled
         * bit i clear means interrupt i disabled
         */
        uint32_t reg_IER;

        /**
         * Receive Data Available Interrupt
         */
#       define  UART_IER_RDA_INTERRUPT_ENABLE_MASK      BIT(0)

        /**
         * Transmit Holding Register Empty Interrupt
         */
#       define  UART_IER_THRE_INTERRUPT_ENABLE_MASK     BIT(1)

        /**
         * Receive Line Status Interrupt
         */
#       define  UART_IER_RX_LINE_INTERRUPT_ENABLE_MASK  BIT(2)

        /* 
         * Reserved
         */
#       define  UART_IER_RESERVED1_MASK                 MULTI_BIT_MASK(7, 3)
#       define  UART_IER_RESERVED1_SHIFT                3

        /**
         * End of auto-baud interrupt
         */
#       define  UART_IER_ABEO_INTERRUPT_ENABLE_MASK     BIT(8)

        /**
         * Auto-baud Time-out Interrupt
         */
#       define  UART_IER_ABTO_INTERRUPT_ENABLE_MASK     BIT(9)

        /* 
         * Reserved
         */
#       define  UART_IER_RESERVED2_MASK                 MULTI_BIT_MASK(31, 10)
#       define  UART_IER_RESERVED2_SHIFT                10
    };

    union {                         /* Offset 0x08 */
        struct {
            /**
             * FIFO Control Register
             */
            uint8_t reg_FCR;
            uint8_t reg_padding2[3];
        };

        /**
         * Interrupt Identification Register
         */
        uint32_t reg_IIR;
        /**
         * Interrupt status (active low):
         * 0 - At least one interrupt is pending.
         * 1 - No interrupt is pending.
         */
#       define  UART_IIR_INT_STATUS_MASK    BIT(0)

        /**
         * Interrupt identification.
         * 011 - Receive Line Status (RLS).
         * 010 - Receive Data Available (RDA).
         * 110 - Character Time-out Indicator (CTI).
         * 001 - THRE Interrupt
         */
#       define  UART_IIR_INT_ID_MASK        MULTI_BIT_MASK(3, 1)
#       define  UART_IIR_INT_ID_SHIFT       1

        /* 
         * Reserved
         */
#       define  UART_IIR_RESERVED1_MASK     MULTI_BIT_MASK(5, 4)
#       define  UART_IIR_RESERVED1_SHIFT    4

        /**
         * FIFO Enable
         */ 
#       define  UART_IIR_FIFO_ENABLE_MASK   MULTI_BIT_MASK(7, 6)
#       define  UART_IIR_FIFO_ENABLE_SHIFT  6

        /**
         * End of auto-baud interrupt. True if auto-baud has finished
         * successfully and interrupt is enabled.
         */
#       define  UART_IIR_ABEO_INT_MASK      BIT(8)

        /**
         * Auto-baud time-out interrupt. True if auto-baud has timed
         * out and interrupt is enabled.
         */ 
#       define  UART_IIR_ABTO_INT_MASK      BIT(9)

        /* 
         * Reserved
         */
#       define  UART_IIR_RESERVED2_MASK     MULTI_BIT_MASK(31, 10)
#       define  UART_IIR_RESERVED2_SHIFT    10
    };

    /**
     * Line Control Register
     */
    uint8_t   reg_LCR;              /* Offset 0x0C */
    uint8_t   reg_padding3[7];

    /**
     * Line Status Register
     */
    uint8_t   reg_LSR;              /* Offset 0x14 */

    /**
     * Receiver Data Ready
     * 0 - RBR is empty
     * 1 - RBR contains unread received data
     */ 
#   define UART_LSR_RDR_MASK        BIT(0)

    /**
     * Overrun Error
     */
#   define UART_LSR_OE_MASK         BIT(1)

    /**
     * Transmit Holding Register Empty
     * 0 - THR contains data being transmitted
     * 1 - THR is empty
     */
#   define UART_LSR_THRE_MASK       BIT(5)

    uint8_t   reg_padding4[3];

    /**
     * Modem Status Register (only for UART1)
     */
    uint8_t   reg_MSR;              /* Offset 0x18 */
    uint8_t   reg_padding5[3];

    /** 
     * Scratch Pad Register
     */
    uint8_t   reg_SCR;              /* Offset 0x1C */
    uint8_t   reg_padding6[3];

    /**
     * Autobaud Control Register
     */
    uint32_t  reg_ACR;              /* Offset 0x20 */

    /**
     * IrDA Control Register
     */
    uint8_t   reg_ICR;              /* Offset 0x24 */
    uint8_t   reg_padding7[3];

    /**
     * Fractional Divider Register
     */
    uint8_t   reg_FDR;              /* Offset 0x28 */
    uint8_t   reg_padding8[7];

    /**
     * Transmit Enable Register
     */
    uint8_t   reg_TER;              /* Offset 0x30 */
    uint8_t   reg_padding9[3];
} lpc2478_uart_t;

/*
 * grivera - Compile-time asserts to verify that the compiler generates
 * the expected offsets for structs that represent groups of memory-mapped
 * I/O registers
 */

C_ASSERT(sizeof(lpc2478_uart_t) == LPC2478_UART_SIZE);

C_ASSERT(offsetof(lpc2478_uart_t, reg_RBR) == 0x00);
C_ASSERT(offsetof(lpc2478_uart_t, reg_THR) == 0x00);
C_ASSERT(offsetof(lpc2478_uart_t, reg_DLL) == 0x00);
C_ASSERT(offsetof(lpc2478_uart_t, reg_DLM) == 0x04);
C_ASSERT(offsetof(lpc2478_uart_t, reg_IER) == 0x04);
C_ASSERT(offsetof(lpc2478_uart_t, reg_IIR) == 0x08);
C_ASSERT(offsetof(lpc2478_uart_t, reg_FCR) == 0x08);
C_ASSERT(offsetof(lpc2478_uart_t, reg_LCR) == 0x0C);
C_ASSERT(offsetof(lpc2478_uart_t, reg_LSR) == 0x14);
C_ASSERT(offsetof(lpc2478_uart_t, reg_MSR) == 0x18);
C_ASSERT(offsetof(lpc2478_uart_t, reg_SCR) == 0x1C);
C_ASSERT(offsetof(lpc2478_uart_t, reg_ACR) == 0x20);
C_ASSERT(offsetof(lpc2478_uart_t, reg_ICR) == 0x24);
C_ASSERT(offsetof(lpc2478_uart_t, reg_FDR) == 0x28);
C_ASSERT(offsetof(lpc2478_uart_t, reg_TER) == 0x30);


#define U0_TX_PINSEL_REG   PINSEL0
#define U0_TX_PINSEL       (1UL<<4)
#define U0_TX_PINMASK      (3UL<<4)
#define U0_RX_PINSEL_REG   PINSEL0
#define U0_RX_PINSEL       (1UL<<6)
#define U0_RX_PINMASK      (3UL<<6)

#define ULCR_DLAB_ENABLE   (1<<7)

#define ULSR_PE            (1<<2)
#define ULSR_FE            (1<<3)
#define ULSR_BI            (1<<4)
#define ULSR_THRE          (1<<5)
#define ULSR_TEMT          (1<<6)
#define ULSR_RXFE          (1<<7)

#define UIER_THRE          (1<<1)
#define UTER_TXEN          (1<<7)
#define UART_8N1           (uint8_t)(3<<0)
#define UART_FIFO_OFF      (0x00)

#define UFCR_FIFO_ENABLE   (1<<0)
#define UFCR_RXFIFO_RESET  (1<<1)
#define UFCR_TXFIFO_RESET  (1<<2)
#define UFCR_RX_TRIGGER_1  (0<<6)
#define UFCR_RX_TRIGGER_4  (1<<6)
#define UFCR_RX_TRIGGER_8  (2<<6)
#define UFCR_RX_TRIGGER_14 (3<<6)

/**
 * Size in bytes of the UART Transmit and Receive FIFOs 
 */
#define UART_FIFO_SIZE  UINT32_C(16)

/**
 * UART interrupt Ids
 */
enum uart_interrupt_id
{
    UART_INT_TRANSMIT_HOLDING_REGISTER_EMPTY =  0x1,  /* 0b001 */
    UART_INT_RECEIVE_DATA_AVAILABLE =           0x2,  /* 0b010 */
    UART_INT_RECEIVE_LINE_STATUS =              0x3,  /* 0b011 */
    UART_INT_CHARACTER_TIMEOUT_INDICATOR =      0x6,  /* 0b110 */
};

typedef enum uart_interrupt_id uart_interrupt_id_t;

#endif /* __UARTS_H */
