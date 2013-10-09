/**
 * @file tfc_board__hardware_abstractions.c
 *
 * Hardware abstraction layer for the TFC add-on board
 *
 * @author German Rivera 
 */ 

#include "hardware_abstractions.h"
#include "McRTOS_arm_cortex_m.h"
#include "MKL25Z4.h"
#include "failure_data_capture.h"
#include "utils.h"
#include "McRTOS_config_parameters.h"
#include "McRTOS_kernel_services.h"
#include "tfc_board.h"

static void tfc_gpio_init(void);
static void tfc_servos_init(void);

void
tfc_board_init(void)
{
    tfc_gpio_init();
    tfc_servos_init();
}

/* 
 * Set I/O for H-BRIDGE enables, switches and LEDs for the TFC add-on board
 */ 
static void
tfc_gpio_init(void)
{
        uint32_t reg_value;

	/*
         * Setup Pins as GPIO
         */
	write_32bit_mmio_register(
            &PORTE_PCR21, PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);

        write_32bit_mmio_register(
	    &PORTE_PCR20, PORT_PCR_MUX(1));
	
	/*
         * Port for Pushbuttons
         */ 
        write_32bit_mmio_register(
	    &PORTC_PCR13, PORT_PCR_MUX(1));

        write_32bit_mmio_register(
            &PORTC_PCR17, PORT_PCR_MUX(1));
	
	/*
         * Ports for DIP Switches
         */ 
        write_32bit_mmio_register(
	    &PORTE_PCR2, PORT_PCR_MUX(1)); 

        write_32bit_mmio_register(
	    &PORTE_PCR3, PORT_PCR_MUX(1));

        write_32bit_mmio_register(
	    &PORTE_PCR4, PORT_PCR_MUX(1));

        write_32bit_mmio_register(
	    &PORTE_PCR5, PORT_PCR_MUX(1));
	
	/*
         * Ports for LEDs
         */ 
        write_32bit_mmio_register(
	    &PORTB_PCR8, PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);

        write_32bit_mmio_register(
	    &PORTB_PCR9, PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);

        write_32bit_mmio_register(
	    &PORTB_PCR10, PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);

        write_32bit_mmio_register(
	    &PORTB_PCR11, PORT_PCR_MUX(1) | PORT_PCR_DSE_MASK);   
	
	/*
         * Setup the output pins
         */
        reg_value = read_32bit_mmio_register(&GPIOE_PDDR);
	reg_value |= TFC_HBRIDGE_EN_LOC;
        write_32bit_mmio_register(&GPIOE_PDDR, reg_value);

        reg_value = read_32bit_mmio_register(&GPIOB_PDDR);
	reg_value |= TFC_BAT_LED0_LOC | TFC_BAT_LED1_LOC | TFC_BAT_LED2_LOC |
                     TFC_BAT_LED3_LOC;
        write_32bit_mmio_register(&GPIOB_PDDR, reg_value);

        /*
         * TFC_HBRIDGE_DISABLE:
         */ 
        write_32bit_mmio_register(
            &GPIOE_PCOR, TFC_HBRIDGE_EN_LOC);
}


static void
tfc_servos_init(void)
{
    kl25_tpm_init();

#if 0 // ???
               //Blow away the control registers to ensure that the counter is not running
               TPM1_SC = 0;
               TPM1_CONF = 0;
               
               //While the counter is disabled we can setup the prescaler
               
               TPM1_SC = TPM_SC_PS(FTM1_CLK_PRESCALE);
               TPM1_SC |= TPM_SC_TOIE_MASK; //Enable Interrupts for the Timer Overflow
               
               //Setup the mod register to get the correct PWM Period
               
               TPM1_MOD = FTM1_CLOCK/(1<<(FTM1_CLK_PRESCALE+1))/FTM1_OVERFLOW_FREQUENCY;
               
               //Setup Channels 0 and 1
               
               TPM1_C0SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
               TPM1_C1SC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK;
               
               //Enable the Counter
               
               //Set the Default duty cycle to servo neutral
               TFC_SetServo(0, 0.0);
               TFC_SetServo(1, 0.0);
               
               //Enable the TPM COunter
               TPM1_SC |= TPM_SC_CMOD(1);
               
               //Enable TPM1 IRQ on the NVIC
               //???enable_irq (INT_TPM1-16); already done in tpm_init
              
               //Enable the FTM functions on the the port
               
               PORTB_PCR0 = PORT_PCR_MUX(3);
               PORTB_PCR1 = PORT_PCR_MUX(3);

#endif // ???
}



