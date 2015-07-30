/**
 * @file lpc54102_soc.h
 *
 * NXP LPC54102 SOC declarations
 *
 * @author German Rivera
 */

#ifndef __LPC54102_SOC_H
#define __LPC54102_SOC_H

#include "lpc54102_soc_public.h"
#include <McRTOS_kernel_services.h>

struct rtos_interrupt;  /* opaque type */

/*
 * AHBCLKCTRL0, PRESET0 register fields
 */
#define SYSCON0_SRAM2_MASK      BIT(4)
#define SYSCON0_INPUTMUX_MASK   BIT(11)
#define SYSCON0_IOCON_MASK      BIT(13)
#define SYSCON0_GPIO0_MASK      BIT(14)
#define SYSCON0_GPIO1_MASK      BIT(15)
#define SYSCON0_PINT_MASK       BIT(18)
#define SYSCON0_GINT_MASK       BIT(19)

/*
 * ASYNCAPBCLKCTRLSET register fields
 */
#define APBCLKCTRL_UART0_MASK   BIT(1)
#define APBCLKCTRL_FRACDIV_MASK BIT(15)

/*
 * ASYNCAPBCTR register fields
 */
#define ASYNCAPBCTR_ENABLE_MASK   BIT(0)

/*
 * SYSPLLCLKSEL register fields
 */
#define SYSPLLCLKSEL_SEL_MASK    MULTI_BIT_MASK(1, 0)
#define SYSPLLCLKSEL_SEL_SHIFT   0

/*
 * SYSPLLSSCTRL register fields
 */
#define SYSPLLSSCTRL_MDEC_MASK     MULTI_BIT_MASK(16, 0)
#define SYSPLLSSCTRL_MDEC_SHIFT    0
#define SYSPLLSSCTRL_MREQ_MASK     BIT(17)
#define SYSPLLSSCTRL_SEL_EXT_MASK  BIT(18)

/*
 * SYSPLLSTAT register fields
 */
#define SYSPLLSTAT_LOCK_MASK    BIT(0)

/*
 * MAINCLKSELB register fields
 */
#define MAINCLKSELB_SEL_MASK    MULTI_BIT_MASK(1, 0)
#define MAINCLKSELB_SEL_SHIFT   0

/*
 * AHBCLKCTRL1, PRESET1 register fields
 */
#define SYSCON1_FIFO_MASK    BIT(9)
#define SYSCON1_TIMER2_MASK  BIT(22)
#define SYSCON1_TIMER3_MASK  BIT(26)
#define SYSCON1_TIMER4_MASK  BIT(27)

/*
 * Fields of UART CFG register
 */
#define UART_CFG_ENABLE_MASK        BIT(0)
#define UART_CFG_DATALEN_MASK       MULTI_BIT_MASK(3, 2)
#define UART_CFG_DATALEN_SHIFT      2
#define UART_CFG_PARITYSEL_MASK     MULTI_BIT_MASK(5, 4)
#define UART_CFG_PARITYSEL_SHIFT    4
#define UART_CFG_STOPLEN_MASK       BIT(6)

/*
 * Fields of UART CTL register
 */
#define UART_CTL_AUTOBAUD_MASK        BIT(16)

/*
 * FIFOCTLUSART register fields
 */
#define FIFOCTLUSART_RXPAUSE_MASK        BIT(0)
#define FIFOCTLUSART_TXPAUSE_MASK        BIT(8)
#define FIFOCTLUSART_RXFIFOTOTAL_MASK    MULTI_BIT_MASK(23, 16)
#define FIFOCTLUSART_RXFIFOTOTAL_SHIFT   16
#define FIFOCTLUSART_TXFIFOTOTAL_MASK    MULTI_BIT_MASK(31, 24)
#define FIFOCTLUSART_TXFIFOTOTAL_SHIFT   24

/*
 * FIFOUPDATEUSART register fields
 */
#define FIFOUPDATEUSART_RXSIZE_MASK(_uart_index)    BIT(_uart_index)
#define FIFOUPDATEUSART_TXSIZE_MASK(_uart_index)    BIT((_uart_index) + 16)

/*
 * FIFOCFGUSART[] registers fields
 */
#define FIFOCFGUSART_RXSIZE_MASK    MULTI_BIT_MASK(7, 0)
#define FIFOCFGUSART_RXSIZE_SHIFT   0
#define FIFOCFGUSART_TXSIZE_MASK    MULTI_BIT_MASK(15, 8)
#define FIFOCFGUSART_TXSIZE_SHIFT   8

/*
 * FIFO UART-specific CFGUSART register fields
 */
#define FIFO_UART_CFGUSART_RX_THRESHOLD_MASK    MULTI_BIT_MASK(23, 16)
#define FIFO_UART_CFGUSART_RX_THRESHOLD_SHIFT   16
#define FIFO_UART_CFGUSART_TX_THRESHOLD_MASK    MULTI_BIT_MASK(31, 24)
#define FIFO_UART_CFGUSART_TX_THRESHOLD_SHIFT   24

/*
 * FIFO UART-specific CTLSETUSART/CTLCLRUSART register fields
 */
#define FIFO_UART_CTL_RXTHINTEN_MASK        BIT(0)
#define FIFO_UART_CTL_TXTHINTEN_MASK        BIT(1)
#define FIFO_UART_CTL_RXTIMEOUTINTEN_MASK   BIT(4)
#define FIFO_UART_CTL_RXFLUSH_MASK          BIT(8)
#define FIFO_UART_CTL_TXFLUSH_MASK          BIT(9)

/*
 * FIFO UART-specific STATUSART/INSTATUSART register fields
 */
#define FIFO_UART_STAT_RXTH_MASK         BIT(0)
#define FIFO_UART_STAT_TXTH_MASK         BIT(1)
#define FIFO_UART_STAT_RXTIMEOUT_MASK    BIT(4)
#define FIFO_UART_STAT_BUSERR_MASK       BIT(7)
#define FIFO_UART_STAT_RXEMPTY_MASK      BIT(8)
#define FIFO_UART_STAT_TXEMPTY_MASK      BIT(9)
#define FIFO_UART_STAT_RXCOUNT_MASK      MULTI_BIT_MASK(23, 16)
#define FIFO_UART_STAT_RXCOUNT_SHIFT     16
#define FIFO_UART_STAT_TXCOUNT_MASK      MULTI_BIT_MASK(31, 24)
#define FIFO_UART_STAT_TXCOUNT_SHIFT     24

/*
 * FIFO UART-specific RXDATSTAT register fields
 */
#define FIFO_UART_RXDATSTAT_RXDAT_MASK      MULTI_BIT_MASK(8, 0)
#define FIFO_UART_RXDATSTAT_RXDAT_SHIFT     0
#define FIFO_UART_RXDATSTAT_FRAMERR_MASK    BIT(13)
#define FIFO_UART_RXDATSTAT_PARITYERR_MASK  BIT(14)
#define FIFO_UART_RXDATSTAT_RXNOISE_MASK    BIT(15)

/**
 * Const fields of a UART device (to be placed in flash)
 */
struct uart_device {
#   define UART_DEVICE_SIGNATURE  GEN_SIGNATURE('U', 'A', 'R', 'T')
    uint32_t urt_signature;
    const char *urt_name;
    struct uart_device_var *urt_var_p;
    UART_REGS_T *urt_mmio_uart_p;
    struct pin_info urt_tx_pin;
    struct pin_info urt_rx_pin;
    uint32_t urt_async_apb_control_mask;
    struct rtos_interrupt_registration_params urt_rtos_interrupt_params;
    struct rtos_interrupt **urt_rtos_interrupt_pp;
    const char *urt_transmit_queue_name_p;
    const char *urt_receive_queue_name_p;
};

#define UART_TRANSMIT_QUEUE_SIZE_IN_BYTES   UINT16_C(128)
#define UART_RECEIVE_QUEUE_SIZE_IN_BYTES    UINT16_C(16)

/**
 * Non-const fields of a UART device (to be placed in SRAM)
 */
struct uart_device_var {
    bool urt_initialized;
    uint32_t urt_received_bytes_dropped;
    uint32_t urt_transmit_bytes_dropped;
    struct rtos_circular_buffer urt_transmit_queue;
    struct rtos_circular_buffer urt_receive_queue;
    uint8_t urt_tx_fifo_size;
    uint8_t urt_rx_fifo_size;
    bool urt_fifos_enabled;
    uint8_t urt_transmit_queue_storage[UART_TRANSMIT_QUEUE_SIZE_IN_BYTES];
    uint8_t urt_receive_queue_storage[UART_RECEIVE_QUEUE_SIZE_IN_BYTES];
};

extern isr_function_t lpc54102_uart0_irq_isr;

void lpc54102_uart_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

#endif /* __LPC54102_SOC_H */
