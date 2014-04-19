/**
 * @file lm4f120_soc.h
 *
 * TI Stellaris LM4F120 SOC private declarations
 *
 * @author German Rivera
 */

#ifndef __LM4F120_SOC_H
#define __LM4F120_SOC_H

#include "lm4f120_soc_public.h"
#include "McRTOS_kernel_services.h"

/**
 * GPIO port MMIO registers
 */
struct gpio_port {
	uint32_t reg_DATA_bits[256];	// Bit masks for data bits
	uint32_t reg_DIR;		// Direction
	uint32_t reg_IS;		// Interrupt Sense
	uint32_t reg_IBE;		// Interrupt Both Edges
	uint32_t reg_IEV;		// Interrupt Event
	uint32_t reg_IM;		// Interrupt Mask
	uint32_t reg_RIS;		// Raw Interrupt Status
	uint32_t reg_MIS;		// Masked Interrupt Status
	uint32_t reg_ICR;		// Interrupt Clear
	uint32_t reg_AFSEL;		// Alternate Function Select
	uint8_t reg_padding1[0xDC];
	uint32_t reg_DR2R;		// 2-mA Drive Select
	uint32_t reg_DR4R;		// 4-mA Drive Select
	uint32_t reg_DR8R;		// 8-mA Drive Select
	uint32_t reg_ODR;		// Open Drain Select
	uint32_t reg_PUR;		// Pull-Up Select
	uint32_t reg_PDR;		// Pull-Down Select
	uint32_t reg_SLR;		// Slew Rate Control Select
	uint32_t reg_DEN;		// Digital Enable
	uint32_t reg_LOCK;		// Lock mask
	uint32_t reg_CR;		// Commit Register
	uint32_t reg_AMSEL;		// Analog Mode Select
	uint32_t reg_PCTL;		// Port Control
	uint32_t reg_ADCCTL;		// ADC Control
	uint32_t reg_DMACTL;		// DMA Control
	uint32_t reg_SI;		// Select Interrupt
	uint32_t reg_DR12R;		// 12-mA Drive Select
	uint32_t reg_WAKEPEN;		// Wake Pin Enable
	uint32_t reg_WAKELVL;		// Wake Level
	uint32_t reg_WAKESTAT;		// Wake Status
};

C_ASSERT(offsetof(struct gpio_port, reg_DATA_bits) == 0x0);
C_ASSERT(offsetof(struct gpio_port, reg_DIR) == 0x400);
C_ASSERT(offsetof(struct gpio_port, reg_IS) == 0x404);
C_ASSERT(offsetof(struct gpio_port, reg_IBE) == 0x408);
C_ASSERT(offsetof(struct gpio_port, reg_IEV) == 0x40C);
C_ASSERT(offsetof(struct gpio_port, reg_IM) == 0x410);
C_ASSERT(offsetof(struct gpio_port, reg_RIS) == 0x414);
C_ASSERT(offsetof(struct gpio_port, reg_MIS) == 0x418);
C_ASSERT(offsetof(struct gpio_port, reg_ICR) == 0x41C);
C_ASSERT(offsetof(struct gpio_port, reg_AFSEL) == 0x420);
C_ASSERT(offsetof(struct gpio_port, reg_DR2R) == 0x500);
C_ASSERT(offsetof(struct gpio_port, reg_DR4R) == 0x504);
C_ASSERT(offsetof(struct gpio_port, reg_DR8R) == 0x508);
C_ASSERT(offsetof(struct gpio_port, reg_ODR) == 0x50C);
C_ASSERT(offsetof(struct gpio_port, reg_PUR) == 0x510);
C_ASSERT(offsetof(struct gpio_port, reg_PDR) == 0x514);
C_ASSERT(offsetof(struct gpio_port, reg_SLR) == 0x518);
C_ASSERT(offsetof(struct gpio_port, reg_DEN) == 0x51C);
C_ASSERT(offsetof(struct gpio_port, reg_LOCK) == 0x520);
C_ASSERT(offsetof(struct gpio_port, reg_CR) == 0x524);
C_ASSERT(offsetof(struct gpio_port, reg_AMSEL) == 0x528);
C_ASSERT(offsetof(struct gpio_port, reg_PCTL) == 0x52C);
C_ASSERT(offsetof(struct gpio_port, reg_ADCCTL) == 0x530);
C_ASSERT(offsetof(struct gpio_port, reg_DMACTL) == 0x534);
C_ASSERT(offsetof(struct gpio_port, reg_SI) == 0x538);
C_ASSERT(offsetof(struct gpio_port, reg_DR12R) == 0x53C);
C_ASSERT(offsetof(struct gpio_port, reg_WAKEPEN) == 0x540);
C_ASSERT(offsetof(struct gpio_port, reg_WAKELVL) == 0x544);
C_ASSERT(offsetof(struct gpio_port, reg_WAKESTAT) == 0x548);

#define GPIO_PCTL_PIN_MUX_MASK(_pin_index) \
	MULTI_BIT_MASK(							\
		GPIO_PCTL_PIN_MUX_SHIFT(_pin_index) + 3,		\
		GPIO_PCTL_PIN_MUX_SHIFT(_pin_index))

#define GPIO_PCTL_PIN_MUX_SHIFT(_pin_index)	((_pin_index) * 4)

#define UART_TX_FIFO_SIZE   16

/**
 * UART MMIO registers
 */
struct uart {
	uint32_t reg_DR;	    // Data
	union {
		uint32_t reg_RSR;   // Receive Status
		uint32_t reg_ECR;   // Error Clear
	};
	uint8_t reg_padding1[0x10];
	uint32_t reg_FR;	    // Flag
	uint8_t reg_padding2[0x4];
	uint32_t reg_ILPR;	    // IrDA Low-Power Register
	uint32_t reg_IBRD;          // Integer Baud-Rate Divisor
	uint32_t reg_FBRD;          // Fractional Baud-Rate Divisor
	uint32_t reg_LCRH;          // Line Control
	uint32_t reg_CTL;           // Control
	uint32_t reg_IFLS;          // Interrupt FIFO Level Select
	uint32_t reg_IM;            // Interrupt Mask
	uint32_t reg_RIS;           // Raw Interrupt Status
	uint32_t reg_MIS;           // Masked Interrupt Status
	uint32_t reg_ICR;           // Interrupt Clear
	uint32_t reg_DMACTL;        // UART DMA Control
};

C_ASSERT(offsetof(struct uart, reg_DR) == 0x0);
C_ASSERT(offsetof(struct uart, reg_RSR) == 0x4);
C_ASSERT(offsetof(struct uart, reg_ECR) == 0x4);
C_ASSERT(offsetof(struct uart, reg_FR) == 0x18);
C_ASSERT(offsetof(struct uart, reg_ILPR) == 0x20);
C_ASSERT(offsetof(struct uart, reg_IBRD) == 0x24);
C_ASSERT(offsetof(struct uart, reg_FBRD) == 0x28);
C_ASSERT(offsetof(struct uart, reg_LCRH) == 0x2C);
C_ASSERT(offsetof(struct uart, reg_CTL) == 0x30);
C_ASSERT(offsetof(struct uart, reg_IFLS) == 0x34);
C_ASSERT(offsetof(struct uart, reg_IM) == 0x38);
C_ASSERT(offsetof(struct uart, reg_RIS) == 0x3C);
C_ASSERT(offsetof(struct uart, reg_MIS) == 0x40);
C_ASSERT(offsetof(struct uart, reg_ICR) == 0x44);
C_ASSERT(offsetof(struct uart, reg_DMACTL) == 0x48);

void lm4f120_uart_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

extern isr_function_t lm4f120_uart0_isr;

#endif /* __LM4F120_SOC_H */
