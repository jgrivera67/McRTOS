/**
 *  @file lpc2478_stk_board.h
 *
 * Olimex LPC-2478-STK board-specific private definitions
 *
 * Copyright (C) 2013 German Rivera
 */ 

#ifndef __LPC_2478_STK_BOARD__
#define __LPC_2478_STK_BOARD__

#include "lpc2478.h"
#include "utils.h"
#include "hardware_abstractions.h"

/* Off-chip System Specific Parameters */
#define FOSC_MAIN            UINT32_C(12000000)
#define FOSC_RTC             UINT32_C(32768)

/*
 * LPC2478 output pins for LEDs (on GPIO port 1)
 */
#define LED_PINS_GPIO_PORT                  GPIO_PORT_P1
#define SD_MMC_LED_PIN_BIT_INDEX            5   /* red SD/MMC LED */
#define USB_HOST_LINK_LED_PIN_BIT_INDEX     13  /* yellow USB host LED */
#define USB_DEVICE_LINK_LED_PIN_BIT_INDEX   18  /* yellow USB device LED */

/*
 * LPC2478 input pins for buttons Buttons (on GPIO port 2)
 */
#define BUTTON_PINS_GPIO_PORT           GPIO_PORT_P2
#define BUTTON1_PIN_BIT_INDEX           19
#define BUTTON2_PIN_BIT_INDEX           21
#define BUTTON1_PIN_MASK                BIT(BUTTON1_PIN_BIT_INDEX)
#define BUTTON2_PIN_MASK                BIT(BUTTON2_PIN_BIT_INDEX)

/*
 * LPC2478 pins for UART0 (on GPIO port 0)
 */
#define UART0_PINS_GPIO_PORT            GPIO_PORT_P0
#define UART0_TXD0_PIN_BIT_INDEX        2
#define UART0_RXD0_PIN_BIT_INDEX        3

/*
 * LPC2478 output pins for Touch screen
 *
 * NOTE:
 * - P0.24 is used as the X1 output pin and also as the 
 *   TOUCH_SCREEN_X_CHANNEL ADC channel input pin.
 * - P0.23 is used as the Y1 output pin and also as the 
 *   TOUCH_SCREEN_Y_CHANNEL ADC channel input pin.
 */
#define TOUCH_SCREEN_PINS_GPIO_PORT     GPIO_PORT_P0
#define TOUCH_SCREEN_X2_PIN_BIT_INDEX   22
#define TOUCH_SCREEN_X1_PIN_BIT_INDEX   24
#define TOUCH_SCREEN_Y2_PIN_BIT_INDEX   21
#define TOUCH_SCREEN_Y1_PIN_BIT_INDEX   23

/*
 * ADC channels
 */
#define	TOUCH_SCREEN_Y_ADC_CHANNEL  0
#define	TOUCH_SCREEN_X_ADC_CHANNEL  1
#define	TRIMPOT_ADC_CHANNEL         7

/*
 * LPC2478 input/output pins for the SSP0 port
 */
#if 0
#define SSP0_PINS_GPIO_PORT         GPIO_PORT_P0
#define SSP0_PINS_GPIO_FUNCTION     PINSEL_ALT_10
#define SSP0_SCK_PIN_BIT_INDEX      15
#define SSP0_SSEL_PIN_BIT_INDEX     16
#define SSP0_MISO_PIN_BIT_INDEX     17
#define SSP0_MOSI_PIN_BIT_INDEX     18
#else
#define SSP0_PINS_GPIO_PORT         GPIO_PORT_P2
#define SSP0_PINS_GPIO_FUNCTION     PINSEL_ALT_11
#define SSP0_SCK_PIN_BIT_INDEX      22
#define SSP0_SSEL_PIN_BIT_INDEX     23
#define SSP0_MISO_PIN_BIT_INDEX     26
#define SSP0_MOSI_PIN_BIT_INDEX     27
#endif

/*
 * LPC2478 input/output pins for the SSP1 port
 */
#define SSP1_PINS_GPIO_PORT         GPIO_PORT_P0
#define SSP1_PINS_GPIO_FUNCTION     PINSEL_ALT_10
#define SSP1_SCK_PIN_BIT_INDEX      7
#define SSP1_SSEL_PIN_BIT_INDEX     6
#define SSP1_MISO_PIN_BIT_INDEX     8
#define SSP1_MOSI_PIN_BIT_INDEX     9

/*
 * LPC2478 input/output pins connected to the VS1002 audio chip
 */
#define VS1002_PINS_GPIO_PORT       GPIO_PORT_P4
#define VS1002_CS_AU_PIN_BIT_INDEX  15
#define VS1002_DREQ_PIN_BIT_INDEX   16
#define VS1002_DCS_PIN_GPIO_PORT    SSP0_PINS_GPIO_PORT
#define VS1002_DCS_PIN_BIT_INDEX    SSP0_SSEL_PIN_BIT_INDEX

/*
 * LPC2478 input/output pins for the Ethernet controller
 */

#define ETHERNET_PINS_GPIO_PORT             GPIO_PORT_P1
#define ETHERNET_PINS_GPIO_FUNCTION         PINSEL_ALT_01

/** 
 * Transmit data enable output pin
 */ 
#define ETHERNET_ENET_TX_EN_PIN_BIT_INDEX   4    

/** 
 * Transmit data output pins in RMII
 */ 
#define ETHERNET_ENET_TXD0_PIN_BIT_INDEX    0      
#define ETHERNET_ENET_TXD1_PIN_BIT_INDEX    1 

/** 
 * Receive data input pins in RMII mode
 */ 
#define ETHERNET_ENET_RXD0_PIN_BIT_INDEX    9     
#define ETHERNET_ENET_RXD1_PIN_BIT_INDEX    10  

/** 
 * Receive error input pin
 */ 
#define ETHERNET_ENET_RX_ER_PIN_BIT_INDEX   14      

/** 
 * Carrier sense input pin
 */ 
#define ETHERNET_ENET_CRS_PIN_BIT_INDEX     8      

/** 
 * Receive clock input pin
 */ 
#define ETHERNET_ENET_REF_CLK_PIN_BIT_INDEX  15      

/** 
 * MIIM (Media Independent Interface Management) clock output pin
 */ 
#define ETHERNET_ENET_MDC_PIN_BIT_INDEX 16      

/** 
 * MIIM data input/output pin
 */ 
#define ETHERNET_ENET_MDIO_PIN_BIT_INDEX 17      

/**
 * PHY ID for the Micrel KS 8721 PHY chip
 */
#define ETHERNET_PHY_ID UINT32_C(0x00221619)

extern const struct uart_device *const g_uart0_p;
extern const struct uart_device *const g_uart1_p;
extern const struct uart_device *const g_uart2_p;
extern const struct uart_device *const g_uart3_p;

extern const struct ssp_controller *const g_ssp0_controller_p;
extern const struct ssp_controller *const g_ssp1_controller_p;

#endif /* __LPC_2478_STK_BOARD__ */
