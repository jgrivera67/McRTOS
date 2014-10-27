/**
 * @file k64f_soc_enet.h
 *
 * Freescale K64F SOC ENET-MAC declarations
 *
 * @author German Rivera
 */

#ifndef __K64F_SOC_ENET_H
#define __K64F_SOC_ENET_H

#include <stdint.h>
#include <McRTOS_kernel_services.h>
#include <Networking/networking.h>

/**
 * Const fields of an Ethernet MAC device (to be placed in flash)
 */
struct enet_device {
#   define ENET_DEVICE_SIGNATURE  GEN_SIGNATURE('E', 'N', 'E', 'T')
    uint32_t signature;
    const char *name;
    struct enet_device_var *var_p;
    volatile ENET_Type *mmio_registers_p;
    struct pin_info rmii_mdio_pin;
    struct pin_info rmii_mdc_pin;
    struct pin_info rmii_rxd0_pin;
    struct pin_info rmii_rxd1_pin;
    struct pin_info rmii_crs_dv_pin;
    struct pin_info rmii_rxer_pin;
    struct pin_info rmii_txen_pin;
    struct pin_info rmii_txd0_pin;
    struct pin_info rmii_txd1_pin;
    struct pin_info mii_txer_pin;
    struct pin_info mii_intr_pin;
    struct rtos_interrupt_registration_params tx_rtos_interrupt_params;
    struct rtos_interrupt **tx_rtos_interrupt_pp;
    struct rtos_interrupt_registration_params rx_rtos_interrupt_params;
    struct rtos_interrupt **rx_rtos_interrupt_pp;
    struct pin_info enet_1588_tmr_pins[4];
    uint32_t clock_gate_mask;
    struct ethernet_mac_address mac_address;
};

extern const struct enet_device g_enet_device0;

#endif /* __K64F_SOC_ENET_H */
