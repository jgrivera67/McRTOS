/*
 * @brief LPC5410x basic chip inclusion file
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#ifndef __CHIP_H_
#define __CHIP_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Main memory addresses */
#define LPC_FLASHMEM_BASE          0x00000000UL
#define LPC_SRAM0_BASE             0x02000000UL
#define LPC_SRAM1_BASE             0x02010000UL
#define LPC_ROM_BASE               0x03000000UL
#define LPC_SRAM2_BASE             0x03400000UL
#define LPC_GPIO_PORT_BASE         0x1C000000UL
#define LPC_DMA_BASE               0x1C004000UL
#define LPC_CRC_BASE               0x1C010000UL
#define LPC_SCT_BASE               0x1C018000UL
#define LPC_MBOX_BASE              0x1C02C000UL
#define LPC_ADC_BASE               0x1C034000UL
#define LPC_FIFO_BASE              0x1C038000UL

/* APB0 peripheral group addresses */
#define LPC_SYSCON_BASE            0x40000000UL
#define LPC_TIMER2_BASE            0x40004000UL
#define LPC_TIMER3_BASE            0x40008000UL
#define LPC_TIMER4_BASE            0x4000C000UL
#define LPC_GPIO_GROUPINT0_BASE    0x40010000UL
#define LPC_GPIO_GROUPINT1_BASE    0x40014000UL
#define LPC_PIN_INT_BASE           0x40018000UL
#define LPC_IOCON_BASE             0x4001C000UL
#define LPC_UTICK_BASE             0x40020000UL
#define LPC_FMC_BASE               0x40024000UL
#define LPC_PMU_BASE               0x4002C000UL
#define LPC_WWDT_BASE              0x40038000UL
#define LPC_RTC_BASE               0x4003C000UL

/* APB1 peripheral group addresses */
#define LPC_ASYNC_SYSCON_BASE      0x40080000UL
#define LPC_USART0_BASE            0x40084000UL
#define LPC_USART1_BASE            0x40088000UL
#define LPC_USART2_BASE            0x4008C000UL
#define LPC_USART3_BASE            0x40090000UL
#define LPC_I2C0_BASE              0x40094000UL
#define LPC_I2C1_BASE              0x40098000UL
#define LPC_I2C2_BASE              0x4009C000UL
#define LPC_SPI0_BASE              0x400A4000UL
#define LPC_SPI1_BASE              0x400A8000UL
#define LPC_TIMER0_BASE            0x400B4000UL
#define LPC_TIMER1_BASE            0x400B8000UL
#define LPC_INMUX_BASE             0x40050000UL
#define LPC_RITIMER_BASE           0x40070000UL
#define LPC_MRT_BASE               0x40074000UL

/* Main memory register access */
#define LPC_GPIO           ((LPC_GPIO_T            *) LPC_GPIO_PORT_BASE)
#define LPC_DMA            ((LPC_DMA_T             *) LPC_DMA_BASE)
#define LPC_CRC            ((LPC_CRC_T             *) LPC_CRC_BASE)
#define LPC_SCT            ((LPC_SCT_T             *) LPC_SCT_BASE)
#define LPC_MBOX           ((LPC_MBOX_T            *) LPC_MBOX_BASE)
#define LPC_ADC            ((LPC_ADC_T             *) LPC_ADC_BASE)
#define LPC_FIFO           ((LPC_FIFO_T            *) LPC_FIFO_BASE)

/* APB0 peripheral group register access */
#define LPC_SYSCON         ((LPC_SYSCON_T          *) LPC_SYSCON_BASE)
#define LPC_TIMER2         ((LPC_TIMER_T           *) LPC_TIMER2_BASE)
#define LPC_TIMER3         ((LPC_TIMER_T           *) LPC_TIMER3_BASE)
#define LPC_TIMER4         ((LPC_TIMER_T           *) LPC_TIMER4_BASE)
#define LPC_GINT           ((LPC_GPIOGROUPINT_T    *) LPC_GPIO_GROUPINT0_BASE)
#define LPC_PININT         ((LPC_PIN_INT_T         *) LPC_PIN_INT_BASE)
#define LPC_IOCON          ((LPC_IOCON_T           *) LPC_IOCON_BASE)
#define LPC_UTICK          ((LPC_UTICK_T           *) LPC_UTICK_BASE)
#define LPC_WWDT           ((LPC_WWDT_T            *) LPC_WWDT_BASE)
#define LPC_RTC            ((LPC_RTC_T             *) LPC_RTC_BASE)

/* APB1 peripheral group register access */
#define LPC_ASYNC_SYSCON   ((LPC_ASYNC_SYSCON_T    *) LPC_ASYNC_SYSCON_BASE)
#define LPC_USART0         ((LPC_USART_T           *) LPC_USART0_BASE)
#define LPC_USART1         ((LPC_USART_T           *) LPC_USART1_BASE)
#define LPC_USART2         ((LPC_USART_T           *) LPC_USART2_BASE)
#define LPC_USART3         ((LPC_USART_T           *) LPC_USART3_BASE)
#define LPC_I2C0           ((LPC_I2C_T             *) LPC_I2C0_BASE)
#define LPC_I2C1           ((LPC_I2C_T             *) LPC_I2C1_BASE)
#define LPC_I2C2           ((LPC_I2C_T             *) LPC_I2C2_BASE)
#define LPC_SCT0           LPC_SCT
#define LPC_SPI0           ((LPC_SPI_T             *) LPC_SPI0_BASE)
#define LPC_SPI1           ((LPC_SPI_T             *) LPC_SPI1_BASE)
#define LPC_TIMER0         ((LPC_TIMER_T           *) LPC_TIMER0_BASE)
#define LPC_TIMER1         ((LPC_TIMER_T           *) LPC_TIMER1_BASE)
#define LPC_INMUX          ((LPC_INMUX_T           *) LPC_INMUX_BASE)
#define LPC_RITIMER        ((LPC_RITIMER_T         *) LPC_RITIMER_BASE)
#define LPC_MRT            ((LPC_MRT_T             *) LPC_MRT_BASE)
#define LPC_PMU            ((LPC_PMU_T             *) LPC_PMU_BASE)

/**
 * Power control definition bits (0 = powered, 1 = powered down)
 */
#define SYSCON_PDRUNCFG_PD_IRC_OSC       (1 << 3)		/*!< IRC oscillator output */
#define SYSCON_PDRUNCFG_PD_IRC           (1 << 4)		/*!< IRC oscillator */
#define SYSCON_PDRUNCFG_PD_FLASH         (1 << 5)		/*!< Flash memory */
#define SYSCON_PDRUNCFG_PD_BOD_RST       (1 << 7)		/*!< Brown-out Detect reset */
#define SYSCON_PDRUNCFG_PD_BOD_INTR      (1 << 8)		/*!< Brown-out Detect interrupt */
#define SYSCON_PDRUNCFG_PD_ADC0          (1 << 10)		/*!< ADC0 */
#define SYSCON_PDRUNCFG_PD_SRAM0A        (1 << 13)		/*!< First 8 kB of SRAM0 */
#define SYSCON_PDRUNCFG_PD_SRAM0B        (1 << 14)		/*!< Remaining portion of SRAM0 */
#define SYSCON_PDRUNCFG_PD_SRAM1         (1 << 15)		/*!< SRAM1 */
#define SYSCON_PDRUNCFG_PD_SRAM2         (1 << 16)		/*!< SRAM2 */
#define SYSCON_PDRUNCFG_PD_ROM           (1 << 17)		/*!< ROM */
#define SYSCON_PDRUNCFG_PD_VDDA_ENA      (1 << 19)		/*!< Vdda to the ADC, must be enabled for the ADC to work */
#define SYSCON_PDRUNCFG_PD_WDT_OSC       (1 << 20)		/*!< Watchdog oscillator */
#define SYSCON_PDRUNCFG_PD_SYS_PLL       (1 << 22)		/*!< PLL0 */
#define SYSCON_PDRUNCFG_PD_VREFP         (1 << 23)		/*!< Vrefp to the ADC, must be enabled for the ADC to work */
#define SYSCON_PDRUNCFG_PD_32K_OSC       (1 << 24)		/*!< 32 kHz RTC oscillator */

#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions                 */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions                 */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions                */
#define     __IO    volatile             /*!< Defines 'read / write' permissions              */

/**
 * @brief LPC5410X Main system configuration register block structure
 */
typedef struct {
	__IO uint32_t SYSMEMREMAP;			/*!< System Remap register */
	__I  uint32_t RESERVED0[4];
	__IO uint32_t SYSTCKCAL;			/*!< System Tick Calibration register */
	__I  uint32_t RESERVED1[1];
	__IO uint32_t NMISRC;				/*!< NMI Source select register */
	__IO uint32_t ASYNCAPBCTRL;			/*!< Asynch APB chiplet control register */
	__I  uint32_t RESERVED2[7];
	__IO uint32_t SYSRSTSTAT;			/*!< System Reset Stat register */
	__IO uint32_t PRESETCTRL[2];		/*!< Peripheral Reset Ctrl register */
	__IO uint32_t PRESETCTRLSET[2];		/*!< Peripheral Reset Ctrl Set register */
	__IO uint32_t PRESETCTRLCLR[2];		/*!< Peripheral Reset Ctrl Clr register */
	__IO uint32_t PIOPORCAP[2];			/*!< PIO Power-On Reset Capture register */
	__I  uint32_t RESERVED3[1];
	__IO uint32_t PIORESCAP[2];			/*!< PIO Pad Reset Capture register */
	__I  uint32_t RESERVED4[4];
	__IO uint32_t MAINCLKSELA;			/*!< Main Clk sel Source Sel A register */
	__IO uint32_t MAINCLKSELB;			/*!< Main Clk sel Source Sel B register */
	__I  uint32_t RESERVED5;
	__IO uint32_t ADCCLKSEL;			/*!< ADC Async Clk Sel register */
	__I  uint32_t RESERVED6;
	__IO uint32_t CLKOUTSELA;			/*!< Clk Out Sel Source A register */
	__IO uint32_t CLKOUTSELB;			/*!< Clk Out Sel Source B register */
	__I  uint32_t RESERVED7;
	__IO uint32_t SYSPLLCLKSEL;			/*!< System PLL Clk Selregister */
	__I  uint32_t RESERVED8[7];
	__IO uint32_t AHBCLKCTRL[2];		/*!< AHB Peripheral Clk Enable register */
	__IO uint32_t AHBCLKCTRLSET[2];		/*!< AHB Peripheral Clk Enable Set register */
	__IO uint32_t AHBCLKCTRLCLR[2];		/*!< AHB Peripheral Clk Enable Clr register */
	__I  uint32_t RESERVED9[2];
	__IO uint32_t SYSTICKCLKDIV;		/*!< Systick Clock divider register */
	__I  uint32_t RESERVED10[7];
	__IO uint32_t AHBCLKDIV;			/*!< Main Clk Divider register */
	__IO uint32_t RESERVED11;
	__IO uint32_t ADCCLKDIV;			/*!< ADC Async Clk Divider register */
	__IO uint32_t CLKOUTDIV;			/*!< Clk Out Divider register */
	__I  uint32_t RESERVED12[4];
	__IO uint32_t FREQMECTRL;			/*!< Frequency Measure Control register */
	__IO uint32_t FLASHCFG;				/*!< Flash Config register */
	__I  uint32_t RESERVED13[8];
	__IO uint32_t FIFOCTRL;				/*!< VFIFO control register */
	__I  uint32_t RESERVED14[14];
	__I  uint32_t RESERVED15[1];
	__I  uint32_t RESERVED16[2];
	__IO uint32_t RTCOSCCTRL;			/*!< RTC Oscillator Control register */
	__I  uint32_t RESERVED17[7];
	__IO uint32_t SYSPLLCTRL;			/*!< System PLL control register */
	__IO uint32_t SYSPLLSTAT;			/*!< PLL status register */
	__IO uint32_t SYSPLLNDEC;			/*!< PLL N decoder register */
	__IO uint32_t SYSPLLPDEC;			/*!< PLL P decoder register */
	__IO uint32_t SYSPLLSSCTRL[2];	/*!< Spread Spectrum control registers */
	__I  uint32_t RESERVED18[18];
	__IO uint32_t PDRUNCFG;				/*!< Power Down Run Config register */
	__IO uint32_t PDRUNCFGSET;			/*!< Power Down Run Config Set register */
	__IO uint32_t PDRUNCFGCLR;			/*!< Power Down Run Config Clr register */
	__I  uint32_t RESERVED19[9];
	__IO uint32_t STARTERP[2];			/*!< Start Signal Enable Register */
	__IO uint32_t STARTERSET[2];		/*!< Start Signal Enable Set Register */
	__IO uint32_t STARTERCLR[2];		/*!< Start Signal Enable Clr Register */
	__I  uint32_t RESERVED20[42];
	__I  uint32_t RESERVED20A[4];
	__I  uint32_t RESERVED21[57];
	__IO uint32_t JTAG_IDCODE;
	__IO uint32_t DEVICE_ID0;			/*!< Boot ROM and die revision register */
	__IO uint32_t DEVICE_ID1;			/*!< Boot ROM and die revision register */
} LPC_SYSCON_T;

/**
 * System reset status values
 */
#define SYSCON_RST_POR    (1 << 0)	/*!< POR reset status */
#define SYSCON_RST_EXTRST (1 << 1)	/*!< External reset status */
#define SYSCON_RST_WDT    (1 << 2)	/*!< Watchdog reset status */
#define SYSCON_RST_BOD    (1 << 3)	/*!< Brown-out detect reset status */
#define SYSCON_RST_SYSRST (1 << 4)	/*!< software system reset status */

/**
 * @brief LPC5410X Asynchronous system configuration register block structure
 */
typedef struct {
	__IO uint32_t AYSNCPRESETCTRL;		/*!< peripheral reset register */
	__IO uint32_t ASYNCPRESETCTRLSET;	/*!< peripheral reset Set register */
	__IO uint32_t ASYNCPRESETCTRLCLR;	/*!< peripheral reset Clr register */
	__I  uint32_t RESERVED0;
	__IO uint32_t ASYNCAPBCLKCTRL;		/*!< clk enable register */
	__IO uint32_t ASYNCAPBCLKCTRLSET;	/*!< clk enable Set register */
	__IO uint32_t ASYNCAPBCLKCTRLCLR;	/*!< clk enable Clr register */
	__I  uint32_t RESERVED1;
	__IO uint32_t ASYNCAPBCLKSELA;		/*!< clk source mux A register */
	__IO uint32_t ASYNCAPBCLKSELB;		/*!< clk source mux B register */
	__IO uint32_t ASYNCCLKDIV;			/*!< clk div register */
	__I  uint32_t RESERVED2;
	__IO uint32_t FRGCTRL;				/*!< Fraction Rate Generator Ctrl register */
} LPC_ASYNC_SYSCON_T;


typedef struct {		/* UART registers Structure          */
	volatile uint32_t CFG;				/*!< Offset: 0x000 Configuration register  */
	volatile uint32_t CTL;				/*!< Offset: 0x004 Control register */
	volatile uint32_t STAT;				/*!< Offset: 0x008 Status register */
	volatile uint32_t INTENSET;			/*!< Offset: 0x00C Interrupt Enable Read and Set register */
	volatile uint32_t INTENCLR;			/*!< Offset: 0x010 Interrupt Enable Clear register */
	const volatile uint32_t RXDAT;		/*!< Offset: 0x014 Receiver Data register */
	const volatile uint32_t RXDATSTAT;	/*!< Offset: 0x018 Rx Data with status */
	volatile uint32_t TXDAT;			/*!< Offset: 0x01C Transmitter Data Register */
	volatile uint32_t BRG;				/*!< Offset: 0x020 Baud Rate Generator register */
	const volatile uint32_t INTSTAT;	/*!< Offset: 0x024 Interrupt Status register */
	volatile uint32_t OSR;				/*!< Offset: 0x028 Oversampling register */
	volatile uint32_t ADR;				/*!< Offset: 0x02C Address register (for automatic address matching) */
} UART_REGS_T;

typedef UART_REGS_T LPC_USART_T;

/** Maximum USART peripherals */
#define LPC_FIFO_USART_MAX      (4)

/** Maximum SPI peripherals */
#define LPC_FIFO_SPI_MAX        (2)

/**
 * @brief LPC5410X System FIFO USART register block structure
 */
typedef struct {
	__IO uint32_t CFG;			/*!< USART configuration Register */
	__IO uint32_t STAT;			/*!< USART status Register */
	__IO uint32_t INTSTAT;		/*!< USART interrupt status Register */
	__IO uint32_t CTLSET;		/*!< USART control read and set Register */
	__IO uint32_t CTLCLR;		/*!< USART control clear Register */
	__IO uint32_t RXDAT;		/*!< USART received data Register */
	__IO uint32_t RXDATSTAT;	/*!< USART received data with status Register */
	__IO uint32_t TXDAT;		/*!< USART transmit data Register */
	__I uint32_t  RESERVED[0x38];
} LPC_FIFO_USART_T;

/**
 * @brief LPC5410X System FIFO SPI register block structure
 */
typedef struct {
	__IO uint32_t CFG;			/*!< SPI configuration Register */
	__IO uint32_t STAT;			/*!< SPI status Register */
	__IO uint32_t INTSTAT;		/*!< SPI interrupt status Register */
	__IO uint32_t CTLSET;		/*!< SPI control read and set Register */
	__IO uint32_t CTLCLR;		/*!< SPI control clear Register */
	__I  uint32_t RXDAT;		/*!< SPI received data Register */
	union {
		__O uint32_t TXDATSPI;	/*!< SPI transmit data and control Register */
		struct {
			__O uint16_t TXDATSPI_DATA;	/*!< SPI transmit data Register */
			__O uint16_t TXDATSPI_CTRL;	/*!< SPI transmit control Register */
		};

	};

	__I  uint32_t RESERVED[0x39];
} LPC_FIFO_SPI_T;

/**
 * @brief LPC5410X System FIFO common register block structure
 */
typedef struct {
	__I  uint32_t reserved0[0x40];
	__IO uint32_t FIFOCTLUSART;			/*!< USART FIFO global control Register */
	__O  uint32_t FIFOUPDATEUSART;		/*!< USART FIFO global update Register */
	__I  uint32_t reserved1[0x2];
	__IO uint32_t FIFOCFGUSART[LPC_FIFO_USART_MAX];	/*!< USART FIFO configuration Registers */
	__I  uint32_t reserved2[0x38];
	__IO uint32_t FIFOCTLSPI;			/*!< SPI FIFO global control Register */
	__O  uint32_t FIFOUPDATESPI;		/*!< SPI FIFO global update Register */
	__I  uint32_t reserved3[0x2];
	__IO uint32_t FIFOCFGSPI[LPC_FIFO_SPI_MAX];		/*!< SPI FIFO configuration Registers */
	__I  uint32_t reserved4[0x3A];
	__I  uint32_t reserved5[((0x1000 - 0x300) / sizeof(uint32_t))];
} LPC_FIFO_CMN_T;

/**
 * @brief LPC5410X Complete system FIFO register block structure
 */
typedef struct {
	LPC_FIFO_CMN_T      common;
	LPC_FIFO_USART_T    usart[LPC_FIFO_USART_MAX];
	__I uint32_t        reserved0[((0x2000 - 0x1400) / sizeof(uint32_t))];
	LPC_FIFO_SPI_T      spi[LPC_FIFO_SPI_MAX];
} LPC_FIFO_T;

/* Internal oscillator frequency */
#define SYSCON_IRC_FREQ     12000000
#define SYSCON_WDTOSC_FREQ  500000
#define SYSCON_RTC_FREQ     32768

#define NVALMAX             0x100
#define PVALMAX             0x20
#define MVALMAX             0x8000

#define PLL_MAX_N_DIV       0x100

#define SYSCON_SYSPLLCTRL_SELI_P    4
#define SYSCON_SYSPLLCTRL_SELI_M   (0x3FUL << SYSCON_SYSPLLCTRL_SELI_P)

#define SYSCON_SYSPLLCTRL_SELR_P    0
#define SYSCON_SYSPLLCTRL_SELR_M    (0xFUL << SYSCON_SYSPLLCTRL_SELR_P)

#define SYSCON_SYSPLLCTRL_SELP_P    10
#define SYSCON_SYSPLLCTRL_SELP_M    (0x1FUL << SYSCON_SYSPLLCTRL_SELP_P)

#define SYSCON_SYSPLLCTRL_BYPASS_FBDIV2_P   16
#define SYSCON_SYSPLLCTRL_BYPASS_FBDIV2     (1UL << SYSCON_SYSPLLCTRL_BYPASS_FBDIV2_P)

#define SYSCON_SYSPLLCTRL_BANDSEL_SSCGREG_N_P   18
#define SYSCON_SYSPLLCTRL_BANDSEL_SSCGREG_N     (1UL << SYSCON_SYSPLLCTRL_BANDSEL_SSCGREG_N_P)

#define SYSCON_SYSPLLCTRL_DIRECTO_P 20
#define SYSCON_SYSPLLCTRL_DIRECTO   (1UL << SYSCON_SYSPLLCTRL_DIRECTO_P)

/* SSCG control[0]: */
#define PLL_SSCG0_MDEC_VAL_SET(_value) \
        (((uint32_t) (_value) << PLL_SSCG0_MDEC_VAL_P) & PLL_SSCG0_MDEC_VAL_M)
#define PLL_SSCG0_SEL_EXT_SSCG_N_P      18
#define PLL_SSCG0_SEL_EXT_SSCG_N        (1 << PLL_SSCG0_SEL_EXT_SSCG_N_P)
#define PLL_SSCG0_MDEC_VAL_P            0 /* MDEC is in bits  16 downto 0 */
#define PLL_SSCG0_MDEC_VAL_M            (0x1FFFFUL << PLL_SSCG0_MDEC_VAL_P)
#define PLL_SSCG0_MREQ_P                17
#define PLL_SSCG0_MREQ                  (1 << PLL_SSCG0_MREQ_P)

/* SSCG control[1]: */
#define PLL_SSCG1_MOD_PD_SSCGCLK_N      (1 << PLL_SSCG1_MOD_PD_SSCGCLK_N_P)
#define PLL_SSCG1_MOD_PD_SSCGCLK_N_P    28
#define PLL_SSCG1_MD_REQ_P              19
#define PLL_SSCG1_MD_REQ                (1 << PLL_SSCG1_MD_REQ_P)
#define PLL_NDEC_VAL_P                  0 /* NDEC is in bits  9:0 */
#define PLL_NDEC_VAL_M                  (0x3FFUL << PLL_NDEC_VAL_P)
#define PLL_NDEC_VAL_SET(_value) \
        (((uint32_t)(_value) << PLL_NDEC_VAL_P) & PLL_NDEC_VAL_M)

#define PLL_PDEC_VAL_SET(_value) \
        (((uint32_t)(_value) << PLL_PDEC_VAL_P) & PLL_PDEC_VAL_M)

#define PLL_PDEC_VAL_P          0   /* PDEC is in bits 6:0 */
#define PLL_PDEC_VAL_M          (0x3FFUL << PLL_PDEC_VAL_P)

/* PLL0 */
#define SYSCON_PDRUNCFG_PD_SYS_PLL  (1 << 22)

#define PLL_NDEC_NREQ_P     10
#define PLL_NDEC_NREQ       (1 << PLL_NDEC_NREQ_P)

#define PLL_PDEC_PREQ_P     7
#define PLL_PDEC_PREQ       (1 << PLL_PDEC_PREQ_P)

/* Find SELP, SELI, and SELR values for raw M value, max M = MVALMAX */
static inline void pllFindSel(uint32_t M, bool bypassFBDIV2,
                              uint32_t *pSelP, uint32_t *pSelI, uint32_t *pSelR)
{
	/* Bypass divider? */
	if (bypassFBDIV2) {
		M = M / 2;
	}

	/* bandwidth: compute selP from Multiplier */
	if (M < 60) {
		*pSelP = (M >> 1) + 1;
	}
	else {
		*pSelP = PVALMAX - 1;
	}

	/* bandwidth: compute selI from Multiplier */
	if (M > 16384) {
		*pSelI = 1;
	}
	else if (M > 8192) {
		*pSelI = 2;
	}
	else if (M > 2048) {
		*pSelI = 4;
	}
	else if (M >= 501) {
		*pSelI = 8;
	}
	else if (M >= 60) {
		*pSelI = 4 * (1024 / (M + 9));
	}
	else {
		*pSelI = (M & 0x3C) + 4;
	}

	if (*pSelI > (SYSCON_SYSPLLCTRL_SELI_M >> SYSCON_SYSPLLCTRL_SELI_P)) {
		*pSelI = (SYSCON_SYSPLLCTRL_SELI_M >> SYSCON_SYSPLLCTRL_SELI_P);
	}

	*pSelR = 0;
}

/* Find encoded MDEC value for raw M value, max M = MVALMAX */
static inline uint32_t pllEncodeM(uint32_t M)
{
	uint32_t i, x;

	/* Find MDec */
	switch (M) {
	case 0:
		x = 0xFFFFF;
		break;

	case 1:
		x = 0x18003;
		break;

	case 2:
		x = 0x10003;
		break;

	default:
		x = 0x04000;
		for (i = M; i <= MVALMAX; i++) {
			x = (((x ^ (x >> 1)) & 1) << 14) | ((x >> 1) & 0x3FFF);
		}
		break;
	}

	return x & (PLL_SSCG0_MDEC_VAL_M >> PLL_SSCG0_MDEC_VAL_P);
}

/* Find encoded NDEC value for raw N value, max N = NVALMAX */
static inline uint32_t pllEncodeN(uint32_t N)
{
	uint32_t x, i;

	/* Find NDec */
	switch (N) {
	case 0:
		x = 0xFFF;
		break;

	case 1:
		x = 0x302;
		break;

	case 2:
		x = 0x202;
		break;

	default:
		x = 0x080;
		for (i = N; i <= NVALMAX; i++) {
			x = (((x ^ (x >> 2) ^ (x >> 3) ^ (x >> 4)) & 1) << 7) | ((x >> 1) & 0x7F);
		}
		break;
	}

	return x & (PLL_NDEC_VAL_M >> PLL_NDEC_VAL_P);
}

/* Find encoded PDEC value for raw P value, max P = PVALMAX */
static inline uint32_t pllEncodeP(uint32_t P)
{
	uint32_t x, i;

	/* Find PDec */
	switch (P) {
	case 0:
		x = 0xFF;
		break;

	case 1:
		x = 0x62;
		break;

	case 2:
		x = 0x42;
		break;

	default:
		x = 0x10;
		for (i = P; i <= PVALMAX; i++) {
			x = (((x ^ (x >> 2)) & 1) << 4) | ((x >> 1) & 0xF);
		}
		break;
	}

	return x & (PLL_PDEC_VAL_M >> PLL_PDEC_VAL_P);
}

/**
 * @brief  GPIO port register block structure
 */
typedef struct {				/*!< GPIO_PORT Structure */
	__IO uint8_t B[128][32];	/*!< Offset 0x0000: Byte pin registers ports 0 to n; pins PIOn_0 to PIOn_31 */
	__IO uint32_t W[32][32];	/*!< Offset 0x1000: Word pin registers port 0 to n */
	__IO uint32_t DIR[32];		/*!< Offset 0x2000: Direction registers port n */
	__IO uint32_t MASK[32];		/*!< Offset 0x2080: Mask register port n */
	__IO uint32_t PIN[32];		/*!< Offset 0x2100: Portpin register port n */
	__IO uint32_t MPIN[32];		/*!< Offset 0x2180: Masked port register port n */
	__IO uint32_t SET[32];		/*!< Offset 0x2200: Write: Set register for port n Read: output bits for port n */
	__O  uint32_t CLR[32];		/*!< Offset 0x2280: Clear port n */
	__O  uint32_t NOT[32];		/*!< Offset 0x2300: Toggle port n */
} LPC_GPIO_T;

/**
 * @brief LPC5410X Pin Interrupt and Pattern Match register block structure
 */
typedef struct {			/*!< PIN_INT Structure */
	__IO uint32_t ISEL;		/*!< Pin Interrupt Mode register */
	__IO uint32_t IENR;		/*!< Pin Interrupt Enable (Rising) register */
	__IO uint32_t SIENR;	/*!< Set Pin Interrupt Enable (Rising) register */
	__IO uint32_t CIENR;	/*!< Clear Pin Interrupt Enable (Rising) register */
	__IO uint32_t IENF;		/*!< Pin Interrupt Enable Falling Edge / Active Level register */
	__IO uint32_t SIENF;	/*!< Set Pin Interrupt Enable Falling Edge / Active Level register */
	__IO uint32_t CIENF;	/*!< Clear Pin Interrupt Enable Falling Edge / Active Level address */
	__IO uint32_t RISE;		/*!< Pin Interrupt Rising Edge register */
	__IO uint32_t FALL;		/*!< Pin Interrupt Falling Edge register */
	__IO uint32_t IST;		/*!< Pin Interrupt Status register */
	__IO uint32_t PMCTRL;	/*!< GPIO pattern match interrupt control register          */
	__IO uint32_t PMSRC;	/*!< GPIO pattern match interrupt bit-slice source register */
	__IO uint32_t PMCFG;	/*!< GPIO pattern match interrupt bit slice configuration register */
} LPC_PIN_INT_T;

/**
 * @brief LPC5410X IO Configuration Unit register block structure
 */
typedef struct {			/*!< LPC5410X IOCON Structure */
	__IO uint32_t  PIO[2][32];
} LPC_IOCON_T;

/**
 * IOCON function and mode selection definitions
 * See the User Manual for specific modes and functions supported by the
 * various LPC15XX pins.
 */
#define IOCON_FUNC0             0x0				/*!< Selects pin function 0 */
#define IOCON_FUNC1             0x1				/*!< Selects pin function 1 */
#define IOCON_FUNC2             0x2				/*!< Selects pin function 2 */
#define IOCON_FUNC3             0x3				/*!< Selects pin function 3 */
#define IOCON_FUNC4             0x4				/*!< Selects pin function 4 */
#define IOCON_FUNC5             0x5				/*!< Selects pin function 5 */
#define IOCON_FUNC6             0x6				/*!< Selects pin function 6 */
#define IOCON_FUNC7             0x7				/*!< Selects pin function 7 */
#define IOCON_MODE_INACT        (0x0 << 3)		/*!< No addition pin function */
#define IOCON_MODE_PULLDOWN     (0x1 << 3)		/*!< Selects pull-down function */
#define IOCON_MODE_PULLUP       (0x2 << 3)		/*!< Selects pull-up function */
#define IOCON_MODE_REPEATER     (0x3 << 3)		/*!< Selects pin repeater function */
#define IOCON_HYS_EN            (0x1 << 5)		/*!< Enables hysteresis */
#define IOCON_GPIO_MODE         (0x1 << 5)		/*!< GPIO Mode */
#define IOCON_I2C_SLEW          (0x1 << 5)		/*!< I2C Slew Rate Control */
#define IOCON_INV_EN            (0x1 << 6)		/*!< Enables invert function on input */
#define IOCON_ANALOG_EN         (0x0 << 7)		/*!< Enables analog function by setting 0 to bit 7 */
#define IOCON_DIGITAL_EN        (0x1 << 7)		/*!< Enables digital function by setting 1 to bit 7(default) */
#define IOCON_STDI2C_EN         (0x1 << 8)		/*!< I2C standard mode/fast-mode */
#define IOCON_FASTI2C_EN        (0x3 << 8)		/*!< I2C Fast-mode Plus and high-speed slave */
#define IOCON_INPFILT_OFF       (0x1 << 8)		/*!< Input filter Off for GPIO pins */
#define IOCON_INPFILT_ON        (0x0 << 8)		/*!< Input filter On for GPIO pins */
#define IOCON_OPENDRAIN_EN      (0x1 << 10)		/*!< Enables open-drain function */
#define IOCON_S_MODE_0CLK       (0x0 << 11)		/*!< Bypass input filter */
#define IOCON_S_MODE_1CLK       (0x1 << 11)		/*!< Input pulses shorter than 1 filter clock are rejected */
#define IOCON_S_MODE_2CLK       (0x2 << 11)		/*!< Input pulses shorter than 2 filter clock2 are rejected */
#define IOCON_S_MODE_3CLK       (0x3 << 11)		/*!< Input pulses shorter than 3 filter clock2 are rejected */
#define IOCON_S_MODE(clks)      ((clks) << 11)	/*!< Select clocks for digital input filter mode */
#define IOCON_CLKDIV(div)       ((div) << 13)	/*!< Select peripheral clock divider for input filter sampling clock, 2^n, n=0-6 */

/**
 * @brief High level ROM API structure
 */
typedef struct {
	const uint32_t reserved_usb;				/*!< Reserved */
	const uint32_t reserved_clib;				/*!< Reserved */
	const uint32_t reserved_can;				/*!< Reserved */
	const struct pwrd_api *pPWRD;				/*!< Power API function table base address */
	const uint32_t reserved_div;				/*!< Reserved */
	const uint32_t reserved_i2cd;				/*!< Reserved */
	const uint32_t reserved_dmad;				/*!< Reserved */
	const uint32_t reserved_spid;				/*!< Reserved */
	const uint32_t reserved_adcd;				/*!< Reserved */
	const uint32_t reserved_uartd;				/*!< Reserved */
	const uint32_t reserved_vfifo;				/*!< Reserved */
	const uint32_t reserved_usart;				/*!< Reserved */
	/* v2 drivers - only present in some LPC5410x devices */
	const struct rom_i2cmd_api *pI2CMD;			/*!< v2 I2C master only driver API function table base address */
	const struct rom_i2csd_api *pI2CSD;			/*!< v2 I2C slave only driver API function table base address */
	const struct rom_i2cmond_api *pI2CMOND;			/*!< v2 I2C bus monitor driver API function table base address */
	const struct rom_spimd_api *pSPIMD;			/*!< v2 SPI master only driver API function table base address */
	const struct rom_spisd_api *pSPISD;			/*!< v2 SPI slave only driver API function table base address */
	const struct rom_dmaaltd_api *pDMAALT;			/*!< v2 abstract DMA driver API function table base address */
	const struct rom_adc_api *pADCALT;			/*!< v2 ADC driver API function table base address */
	const struct rom_uart_api *pUARTALT;			/*!< v2 UART driver API function table base address */
} LPC_ROM_API_T;

/* Pointer to ROM API function address */
#define LPC_ROM_API_BASE_LOC    0x03000200UL
#define LPC_ROM_API     (*(LPC_ROM_API_T * *) LPC_ROM_API_BASE_LOC)

/** @defgroup PWRD_5410X CHIP: LPC5410X Power ROM API declarations and functions
 * @ingroup ROMAPI_5410X
 * @{
 */

/* 'mode' input values to set_voltage ROM function */
typedef enum {
	POWER_LOW_POWER_MODE = 0,
	POWER_BALANCED_MODE,
	POWER_HIGH_PERFORMANCE
} PERF_MODE_T;

/* 'mode' input values to power_mode_configure ROM function */
typedef enum {
	POWER_SLEEP = 0,
	POWER_DEEP_SLEEP,
	POWER_POWER_DOWN,
	POWER_DEEP_POWER_DOWN
} POWER_MODE_T;

/** @brief Power ROM indirect function structure
 * Do not use these functions as direct calls to ROM. Instead, use the
 * wrapper functions provided by the Power library (power_lib_5410x.h)
 */
struct pwrd_api {
	uint32_t (*set_pll)(uint32_t multiply_by, uint32_t input_freq);
	uint32_t (*set_voltage)(uint32_t mode, uint32_t desired_freq);
	void (*power_mode_configure)(uint32_t mode, uint32_t peripheral_ctrl);
};

/** Error code returned by Boot ROM drivers/library functions
 *
 *  Error codes are a 32-bit value with :
 *      - The 16 MSB contains the peripheral code number
 *      - The 16 LSB contains an error code number associated to that peripheral
 *
 */
typedef enum
{
  /**\b 0x00000000*/ LPC_OK=0, /**< enum value returned on Success */
  /**\b 0xFFFFFFFF*/ ERR_FAILED = -1, /**< enum value returned on general failure */
  /**\b 0xFFFFFFFE*/ ERR_TIME_OUT = -2, /**< enum value returned on general timeout */
  /**\b 0xFFFFFFFD*/ ERR_BUSY = -3,	/**< enum value returned when resource is busy */

  /* ISP related errors */
  ERR_ISP_BASE = 0x00000000,
  /*0x00000001*/ ERR_ISP_INVALID_COMMAND = ERR_ISP_BASE + 1,
  /*0x00000002*/ ERR_ISP_SRC_ADDR_ERROR, /* Source address not on word boundary */
  /*0x00000003*/ ERR_ISP_DST_ADDR_ERROR, /* Destination address not on word or 256 byte boundary */
  /*0x00000004*/ ERR_ISP_SRC_ADDR_NOT_MAPPED,
  /*0x00000005*/ ERR_ISP_DST_ADDR_NOT_MAPPED,
  /*0x00000006*/ ERR_ISP_COUNT_ERROR, /* Byte count is not multiple of 4 or is not a permitted value */
  /*0x00000007*/ ERR_ISP_INVALID_SECTOR,
  /*0x00000008*/ ERR_ISP_SECTOR_NOT_BLANK,
  /*0x00000009*/ ERR_ISP_SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,
  /*0x0000000A*/ ERR_ISP_COMPARE_ERROR,
  /*0x0000000B*/ ERR_ISP_BUSY, /* Flash programming hardware interface is busy */
  /*0x0000000C*/ ERR_ISP_PARAM_ERROR, /* Insufficient number of parameters */
  /*0x0000000D*/ ERR_ISP_ADDR_ERROR, /* Address not on word boundary */
  /*0x0000000E*/ ERR_ISP_ADDR_NOT_MAPPED,
  /*0x0000000F*/ ERR_ISP_CMD_LOCKED, /* Command is locked */
  /*0x00000010*/ ERR_ISP_INVALID_CODE, /* Unlock code is invalid */
  /*0x00000011*/ ERR_ISP_INVALID_BAUD_RATE,
  /*0x00000012*/ ERR_ISP_INVALID_STOP_BIT,
  /*0x00000013*/ ERR_ISP_CODE_READ_PROTECTION_ENABLED,
  /*0x00000014*/ ERR_ISP_INVALID_FLASH_UNIT,
  /*0x00000015*/ ERR_ISP_USER_CODE_CHECKSUM,
  /*0x00000016*/ ERR_ISP_SETTING_ACTIVE_PARTITION,
  /*0x00000017*/ ERR_ISP_IRC_NO_POWER,
  /*0x00000018*/ ERR_ISP_FLASH_NO_POWER,
  /*0x00000019*/ ERR_ISP_EEPROM_NO_POWER,
  /*0x0000001A*/ ERR_ISP_EEPROM_NO_CLOCK,
  /*0x0000001B*/ ERR_ISP_FLASH_NO_CLOCK,
  /*0x0000001C*/ ERR_ISP_REINVOKE_ISP_CONFIG,

  /* ROM API related errors */
  ERR_API_BASE = 0x00010000,
  /**\b 0x00010001*/ ERR_API_INVALID_PARAMS = ERR_API_BASE + 1, /**< Invalid parameters*/
  /**\b 0x00010002*/ ERR_API_INVALID_PARAM1, /**< PARAM1 is invalid */
  /**\b 0x00010003*/ ERR_API_INVALID_PARAM2, /**< PARAM2 is invalid */
  /**\b 0x00010004*/ ERR_API_INVALID_PARAM3, /**< PARAM3 is invalid */
  /**\b 0x00010005*/ ERR_API_MOD_INIT, /**< API is called before module init */

  /* SPIFI API related errors */
  ERR_SPIFI_BASE = 0x00020000,
  /*0x00020001*/ ERR_SPIFI_DEVICE_ERROR =ERR_SPIFI_BASE+1,
  /*0x00020002*/ ERR_SPIFI_INTERNAL_ERROR,
  /*0x00020003*/ ERR_SPIFI_TIMEOUT,
  /*0x00020004*/ ERR_SPIFI_OPERAND_ERROR,
  /*0x00020005*/ ERR_SPIFI_STATUS_PROBLEM,
  /*0x00020006*/ ERR_SPIFI_UNKNOWN_EXT,
  /*0x00020007*/ ERR_SPIFI_UNKNOWN_ID,
  /*0x00020008*/ ERR_SPIFI_UNKNOWN_TYPE,
  /*0x00020009*/ ERR_SPIFI_UNKNOWN_MFG,
  /*0x0002000A*/ ERR_SPIFI_NO_DEVICE,
  /*0x0002000B*/ ERR_SPIFI_ERASE_NEEDED,

  SEC_AES_NO_ERROR=0,
  /* Security API related errors */
  ERR_SEC_AES_BASE = 0x00030000,
  /*0x00030001*/ ERR_SEC_AES_WRONG_CMD=ERR_SEC_AES_BASE+1,
  /*0x00030002*/ ERR_SEC_AES_NOT_SUPPORTED,
  /*0x00030003*/ ERR_SEC_AES_KEY_ALREADY_PROGRAMMED,
  /*0x00030004*/ ERR_SEC_AES_DMA_CHANNEL_CFG,
  /*0x00030005*/ ERR_SEC_AES_DMA_MUX_CFG,
  /*0x00030006*/ SEC_AES_DMA_BUSY,

  /* USB device stack related errors */
  ERR_USBD_BASE = 0x00040000,
  /**\b 0x00040001*/ ERR_USBD_INVALID_REQ = ERR_USBD_BASE + 1, /**< invalid request */
  /**\b 0x00040002*/ ERR_USBD_UNHANDLED, /**< Callback did not process the event */
  /**\b 0x00040003*/ ERR_USBD_STALL,     /**< Stall the endpoint on which the call back is called */
  /**\b 0x00040004*/ ERR_USBD_SEND_ZLP,  /**< Send ZLP packet on the endpoint on which the call back is called */
  /**\b 0x00040005*/ ERR_USBD_SEND_DATA, /**< Send data packet on the endpoint on which the call back is called */
  /**\b 0x00040006*/ ERR_USBD_BAD_DESC,  /**< Bad descriptor*/
  /**\b 0x00040007*/ ERR_USBD_BAD_CFG_DESC,/**< Bad config descriptor*/
  /**\b 0x00040008*/ ERR_USBD_BAD_INTF_DESC,/**< Bad interface descriptor*/
  /**\b 0x00040009*/ ERR_USBD_BAD_EP_DESC,/**< Bad endpoint descriptor*/
  /**\b 0x0004000a*/ ERR_USBD_BAD_MEM_BUF, /**< Bad alignment of buffer passed. */
  /**\b 0x0004000b*/ ERR_USBD_TOO_MANY_CLASS_HDLR, /**< Too many class handlers. */

  /* CGU  related errors */
  ERR_CGU_BASE = 0x00050000,
  /*0x00050001*/ ERR_CGU_NOT_IMPL=ERR_CGU_BASE+1,
  /*0x00050002*/ ERR_CGU_INVALID_PARAM,
  /*0x00050003*/ ERR_CGU_INVALID_SLICE,
  /*0x00050004*/ ERR_CGU_OUTPUT_GEN,
  /*0x00050005*/ ERR_CGU_DIV_SRC,
  /*0x00050006*/ ERR_CGU_DIV_VAL,
  /*0x00050007*/ ERR_CGU_SRC,

  /*  I2C related errors   */
  ERR_I2C_BASE = 0x00060000,
  /*0x00060000*/ ERR_I2C_BUSY = ERR_I2C_BASE,
  /*0x00060001*/ ERR_I2C_NAK,
  /*0x00060002*/ ERR_I2C_BUFFER_OVERFLOW,
  /*0x00060003*/ ERR_I2C_BYTE_COUNT_ERR,
  /*0x00060004*/ ERR_I2C_LOSS_OF_ARBRITRATION,
  /*0x00060005*/ ERR_I2C_SLAVE_NOT_ADDRESSED,
  /*0x00060006*/ ERR_I2C_LOSS_OF_ARBRITRATION_NAK_BIT,
  /*0x00060007*/ ERR_I2C_GENERAL_FAILURE,
  /*0x00060008*/ ERR_I2C_REGS_SET_TO_DEFAULT,
  /*0x00060009*/ ERR_I2C_TIMEOUT,
  /*0x0006000A*/ ERR_I2C_BUFFER_UNDERFLOW,
  /*0x0006000B*/ ERR_I2C_PARAM,

   /* OTP  related errors */
  ERR_OTP_BASE = 0x00070000,
  /*0x00070001*/ ERR_OTP_WR_ENABLE_INVALID = ERR_OTP_BASE+1,
  /*0x00070002*/ ERR_OTP_SOME_BITS_ALREADY_PROGRAMMED,
  /*0x00070003*/ ERR_OTP_ALL_DATA_OR_MASK_ZERO,
  /*0x00070004*/ ERR_OTP_WRITE_ACCESS_LOCKED,
  /*0x00070005*/ ERR_OTP_READ_DATA_MISMATCH,
  /*0x00070006*/ ERR_OTP_USB_ID_ENABLED,
  /*0x00070007*/ ERR_OTP_ETH_MAC_ENABLED,
  /*0x00070008*/ ERR_OTP_AES_KEYS_ENABLED,
  /*0x00070009*/ ERR_OTP_ILLEGAL_BANK,

  /*  UART related errors   */
  ERR_UART_BASE = 0x00080000,
  /*0x00080001*/ ERR_UART_RXD_BUSY = ERR_UART_BASE+1,   //UART rxd is busy
  /*0x00080002*/ ERR_UART_TXD_BUSY,   //UART txd is busy
  /*0x00080003*/ ERR_UART_OVERRUN_FRAME_PARITY_NOISE, //overrun err, frame err, parity err, RxNoise err
  /*0x00080004*/ ERR_UART_UNDERRUN,    //underrun err
  /*0x00080005*/ ERR_UART_PARAM,       //parameter is error
  /*0x00080006*/ ERR_UART_BAUDRATE,    //baudrate setting is error

  /*  CAN related errors   */
  ERR_CAN_BASE = 0x00090000,
  /*0x00090001*/ ERR_CAN_BAD_MEM_BUF = ERR_CAN_BASE+1,
  /*0x00090002*/ ERR_CAN_INIT_FAIL,
  /*0x00090003*/ ERR_CANOPEN_INIT_FAIL,

  /* SPIFI Lite API related errors */
  ERR_SPIFI_LITE_BASE = 0x000A0000,
  /*0x000A0001*/ ERR_SPIFI_LITE_INVALID_ARGUMENTS = ERR_SPIFI_LITE_BASE+1,
  /*0x000A0002*/ ERR_SPIFI_LITE_BUSY,
  /*0x000A0003*/ ERR_SPIFI_LITE_MEMORY_MODE_ON,
  /*0x000A0004*/ ERR_SPIFI_LITE_MEMORY_MODE_OFF,
  /*0x000A0005*/ ERR_SPIFI_LITE_IN_DMA,
  /*0x000A0006*/ ERR_SPIFI_LITE_NOT_IN_DMA,
  /*0x000A0100*/ PENDING_SPIFI_LITE,

  /* CLK related errors */
  ERR_CLK_BASE = 0x000B0000,
  /*0x000B0001*/ ERR_CLK_NOT_IMPL=ERR_CLK_BASE+1,
  /*0x000B0002*/ ERR_CLK_INVALID_PARAM,
  /*0x000B0003*/ ERR_CLK_INVALID_SLICE,
  /*0x000B0004*/ ERR_CLK_OUTPUT_GEN,
  /*0x000B0005*/ ERR_CLK_DIV_SRC,
  /*0x000B0006*/ ERR_CLK_DIV_VAL,
  /*0x000B0007*/ ERR_CLK_SRC,
  /*0x000B0008*/ ERR_CLK_PLL_FIN_TOO_SMALL,
  /*0x000B0009*/ ERR_CLK_PLL_FIN_TOO_LARGE,
  /*0x000B000A*/ ERR_CLK_PLL_FOUT_TOO_SMALL,
  /*0x000B000B*/ ERR_CLK_PLL_FOUT_TOO_LARGE,
  /*0x000B000C*/ ERR_CLK_PLL_NO_SOLUTION,
  /*0x000B000D*/ ERR_CLK_PLL_MIN_PCT,
  /*0x000B000E*/ ERR_CLK_PLL_MAX_PCT,
  /*0x000B000F*/ ERR_CLK_OSC_FREQ,
  /*0x000B0010*/ ERR_CLK_CFG,
  /*0x000B0011*/ ERR_CLK_TIMEOUT,
  /*0x000B0012*/ ERR_CLK_BASE_OFF,
  /*0x000B0013*/ ERR_CLK_OFF_DEADLOCK,

  /*Power API*/
  ERR_PWR_BASE = 0x000C0000,
  /*0x000C0001*/  PWR_ERROR_ILLEGAL_MODE=ERR_PWR_BASE+1,
  /*0x000C0002*/  PWR_ERROR_CLOCK_FREQ_TOO_HIGH,
  /*0x000C0003*/  PWR_ERROR_INVALID_STATE,
  /*0x000C0004*/  PWR_ERROR_INVALID_CFG,
  /*0x000C0005*/  PWR_ERROR_PVT_DETECT,

  /* DMA related errors */
  ERR_DMA_BASE = 0x000D0000,
  /*0x000D0001*/	 ERR_DMA_ERROR_INT=ERR_DMA_BASE+1,
  /*0x000D0002*/	 ERR_DMA_CHANNEL_NUMBER,
  /*0x000D0003*/	 ERR_DMA_CHANNEL_DISABLED,
  /*0x000D0004*/	 ERR_DMA_BUSY,
  /*0x000D0005*/	 ERR_DMA_NOT_ALIGNMENT,
  /*0x000D0006*/	 ERR_DMA_PING_PONG_EN,
  /*0x000D0007*/	 ERR_DMA_CHANNEL_VALID_PENDING,
  /*0x000D0008*/	 ERR_DMA_PARAM,
  /*0x000D0009*/	 ERR_DMA_QUEUE_EMPTY,
  /*0x000D000A*/	 ERR_DMA_GENERAL,

  /* SPI related errors */
  ERR_SPI_BASE = 0x000E0000,
  /*0x000E0000*/	 ERR_SPI_BUSY=ERR_SPI_BASE,
  /*0x000E0001*/	 ERR_SPI_RXOVERRUN,
  /*0x000E0002*/	 ERR_SPI_TXUNDERRUN,
  /*0x000E0003*/	 ERR_SPI_SELNASSERT,
  /*0x000E0004*/	 ERR_SPI_SELNDEASSERT,
  /*0x000E0005*/	 ERR_SPI_CLKSTALL,
  /*0x000E0006*/	 ERR_SPI_PARAM,
  /*0x000E0007*/	 ERR_SPI_INVALID_LENGTH,

  /* ADC related errors */
  ERR_ADC_BASE = 0x000F0000,
  /*0x000F0001*/	 ERR_ADC_OVERRUN=ERR_ADC_BASE+1,
  /*0x000F0002*/	 ERR_ADC_INVALID_CHANNEL,
  /*0x000F0003*/	 ERR_ADC_INVALID_SEQUENCE,
  /*0x000F0004*/	 ERR_ADC_INVALID_SETUP,
  /*0x000F0005*/	 ERR_ADC_PARAM,
  /*0x000F0006*/	 ERR_ADC_INVALID_LENGTH,
  /*0x000F0007*/	 ERR_ADC_NO_POWER,

  /* Debugger Mailbox related errors */
  ERR_DM_BASE = 0x00100000,
  /*0x00100001*/	 ERR_DM_NOT_ENTERED=ERR_DM_BASE+1,
  /*0x00100002*/	 ERR_DM_UNKNOWN_CMD,
  /*0x00100003*/	 ERR_DM_COMM_FAIL

} ErrorCode_t;

/**
 * @brief	UART Baud rate calculation structure
 * @note
 * Use oversampling (@a ovr) value other than 16, only if the difference
 * between the actual baud and desired baud has an unacceptable error percentage.
 * Smaller @a ovr values can cause the sampling position within the data-bit
 * less accurate an may potentially cause more noise errors or incorrect data
 * set ovr to < 10 only when there is no other higher values suitable.
 */
typedef struct {
	uint32_t clk;	/*!< IN: Base clock to fractional divider; OUT: "Base clock rate for UART" */
	uint32_t baud;	/*!< IN: Required baud rate; OUT: Actual baud rate */
	uint8_t ovr;	/*!< IN: Number of desired over samples [0-auto detect or values 5 to 16]; OUT: Auto detected over samples [unchanged if IN is not 0] */
	uint8_t mul;	/*!< IN: 0 - calculate MUL, 1 - do't calculate (@a clk) has UART base clock; OUT: MUL value to be set in FRG register */
	uint16_t div;	/*!< OUT: Integer divider to divide the "Base clock rate for UART" */
} UART_BAUD_T;

/* PRIVATE: Division logic to divide without integer overflow */
static inline uint32_t _UART_DivClk(uint32_t pclk, uint32_t m)
{
	uint32_t q, r, u = pclk >> 24, l = pclk << 8;
	m = m + 256;
	q = (1 << 24) / m;
	r = (1 << 24) - (q * m);
	return ((q * u) << 8) + (((r * u) << 8) + l) / m;
}

/* PRIVATE: Get highest Over sampling value */
static inline uint32_t _UART_GetHighDiv(uint32_t val, uint8_t strict)
{
	int32_t i, max = strict ? 16 : 5;
	for (i = 16; i >= max; i--) {
		if (!(val % i)) {
			return i;
		}
	}
	return 0;
}

/* Calculate error difference */
static inline int32_t _CalcErr(uint32_t n, uint32_t d, uint32_t *prev)
{
	uint32_t err = n - (n / d) * d;
	uint32_t herr = ((n / d) + 1) * d - n;
	if (herr < err) {
		err = herr;
	}

	if (*prev <= err) {
		return 0;
	}
	*prev = err;
	return (herr == err) + 1;
}

/* Calculate the best MUL value */
static inline void _UART_CalcMul(UART_BAUD_T *ub)
{
	uint32_t m, perr = ~0UL, pclk = ub->clk, ovr = ub->ovr;

	/* If clock is UART's base clock calculate only the divider */
	for (m = 0; m < 256; m++) {
		uint32_t ov = ovr, x, v, tmp;

		/* Get clock and calculate error */
		x = _UART_DivClk(pclk, m);
		tmp = _CalcErr(x, ub->baud, &perr);
		v = (x / ub->baud) + tmp - 1;

		/* Update if new error is better than previous best */
		if (!tmp || (ovr && (v % ovr)) ||
			(!ovr && ((ov = _UART_GetHighDiv(v, ovr)) == 0))) {
			continue;
		}

		ub->ovr = ov;
		ub->mul = m;
		ub->clk = x;
		ub->div = tmp - 1;
	}
}

/* Calculate the base DIV value */
static inline ErrorCode_t _UART_CalcDiv(UART_BAUD_T *ub)
{
	int32_t i = 0;
	uint32_t perr = ~0UL;

	if (!ub->div) {
		i = ub->ovr ? ub->ovr : 16;
	}

	for (; i > 4; i--) {
		int32_t tmp = _CalcErr(ub->clk, ub->baud * i, &perr);

		/* Continue when no improvement seen in err value */
		if (!tmp) {
			continue;
		}

		ub->div = tmp - 1;
		if (ub->ovr == i) {
			break;
		}
		ub->ovr = i;
	}

	if (!ub->ovr) {
		return ERR_UART_BAUDRATE;
	}

	ub->div += ub->clk / (ub->baud * ub->ovr);
	if (!ub->div) {
		return ERR_UART_BAUDRATE;
	}

	ub->baud = ub->clk / (ub->div * ub->ovr);
	return LPC_OK;
}

/* EXPORTED API: Calculate UART Baudrate divisors */
static inline ErrorCode_t UART_CalculateBaud(UART_BAUD_T *ub)
{
	if (!ub->mul) {
		_UART_CalcMul(ub);
	}

	return _UART_CalcDiv(ub);
}

#ifdef __cplusplus
}
#endif

#endif /* __CHIP_H_ */
