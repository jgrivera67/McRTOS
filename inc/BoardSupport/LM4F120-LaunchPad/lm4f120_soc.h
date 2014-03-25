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
#include "tivaware/hw_gpio.h"

struct gpio_port {
	uint32_t reg_GPIO_O_DATA;
	uint8_t reg_padding1[0x3FC];
	uint32_t reg_GPIO_O_DIR;
	uint32_t reg_GPIO_O_IS;
	uint32_t reg_GPIO_O_IBE;
	uint32_t reg_GPIO_O_IEV;
	uint32_t reg_GPIO_O_IM;
	uint32_t reg_GPIO_O_RIS;
	uint32_t reg_GPIO_O_MIS;
	uint32_t reg_GPIO_O_ICR;
	uint32_t reg_GPIO_O_AFSEL;
	uint8_t reg_padding2[0xDC];
	uint32_t reg_GPIO_O_DR2R;
	uint32_t reg_GPIO_O_DR4R;
	uint32_t reg_GPIO_O_DR8R;
	uint32_t reg_GPIO_O_ODR;
	uint32_t reg_GPIO_O_PUR;
	uint32_t reg_GPIO_O_PDR;
	uint32_t reg_GPIO_O_SLR;
	uint32_t reg_GPIO_O_DEN;
	uint32_t reg_GPIO_O_LOCK;
	uint32_t reg_GPIO_O_CR;
	uint32_t reg_GPIO_O_AMSEL;
	uint32_t reg_GPIO_O_PCTL;
	uint32_t reg_GPIO_O_ADCCTL;
	uint32_t reg_GPIO_O_DMACTL;
	uint32_t reg_GPIO_O_SI;
	uint32_t reg_GPIO_O_DR12R;
	uint32_t reg_GPIO_O_WAKEPEN;
	uint32_t reg_GPIO_O_WAKELVL;
	uint32_t reg_GPIO_O_WAKESTAT;
	uint8_t reg_padding3[0xA74];
	uint32_t reg_GPIO_O_PP;
	uint32_t reg_GPIO_O_PC;
};

C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_DATA) == GPIO_O_DATA);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_DIR) == GPIO_O_DIR);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_IS) == GPIO_O_IS);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_IBE) == GPIO_O_IBE);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_IEV) == GPIO_O_IEV);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_IM) == GPIO_O_IM);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_RIS) == GPIO_O_RIS);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_MIS) == GPIO_O_MIS);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_ICR) == GPIO_O_ICR);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_AFSEL) == GPIO_O_AFSEL);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_DR2R) == GPIO_O_DR2R);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_DR4R) == GPIO_O_DR4R);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_DR8R) == GPIO_O_DR8R);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_ODR) == GPIO_O_ODR);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_PUR) == GPIO_O_PUR);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_PDR) == GPIO_O_PDR);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_SLR) == GPIO_O_SLR);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_DEN) == GPIO_O_DEN);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_LOCK) == GPIO_O_LOCK);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_CR) == GPIO_O_CR);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_AMSEL) == GPIO_O_AMSEL);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_PCTL) == GPIO_O_PCTL);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_ADCCTL) == GPIO_O_ADCCTL);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_DMACTL) == GPIO_O_DMACTL);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_SI) == GPIO_O_SI);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_DR12R) == GPIO_O_DR12R);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_WAKEPEN) == GPIO_O_WAKEPEN);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_WAKELVL) == GPIO_O_WAKELVL);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_WAKESTAT) == GPIO_O_WAKESTAT);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_PP) == GPIO_O_PP);
C_ASSERT(offsetof(struct gpio_port, reg_GPIO_O_PC) == GPIO_O_PC);

#define GPIO_PORTF_DATA_BITS_R  ((volatile uint32_t *)0x40025000)
#define GPIO_PORTF_DATA_R       (*((volatile uint32_t *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile uint32_t *)0x40025400))

#endif /* __LM4F120_SOC_H */
