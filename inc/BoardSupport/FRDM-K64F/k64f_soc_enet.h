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
 * Ethernet frame buffer descriptor alignment in bytes
 */
#define ENET_FRAME_BUFFER_DESCRIPTOR_ALIGNMENT UINT32_C(16)

/**
 * Rx buffer descriptor (type of entries of the ENET Rx ring)
 */
struct enet_rx_buffer_descriptor {
    /**
     * If the ENET_RX_BD_LAST_IN_FRAME bit is set in 'control', this is the
     * frame length in bytes including CRC
     */
    uint16_t data_length;

    uint16_t control;
#   define  ENET_RX_BD_EMPTY_MASK			BIT(15)
#   define  ENET_RX_BD_SOFTWARE_OWNERSHIP1_MASK		BIT(14)
#   define  ENET_RX_BD_WRAP_MASK			BIT(13)
#   define  ENET_RX_BD_SOFTWARE_OWNERSHIP2_MASK		BIT(12)
#   define  ENET_RX_BD_LAST_IN_FRAME_MASK		BIT(11)
#   define  ENET_RX_BD_MISS_MASK			BIT(8)
#   define  ENET_RX_BD_BROADCAST_MASK			BIT(7)
#   define  ENET_RX_BD_MULTICAST_MASK			BIT(6)
#   define  ENET_RX_BD_LENGTH_VIOLATION_MASK		BIT(5)
#   define  ENET_RX_BD_NON_OCTET_ALIGNED_FRAME_MASK	BIT(4)
#   define  ENET_RX_BD_CRC_ERROR_MASK			BIT(2)
#   define  ENET_RX_BD_FIFO_OVERRRUN_MASK		BIT(1)
#   define  ENET_RX_BD_FRAME_TRUNCATED_MASK		BIT(0)

    void *data_buffer;
    uint16_t control_extend0;
#   define  ENET_RX_BD_VLAN_PRIORITY_CODE_POINT_MASK	MULTI_BIT_MASK(15, 13)
#   define  ENET_RX_BD_VLAN_PRIORITY_CODE_POINT_SHIFT	13
#   define  ENET_RX_BD_IP_HEADER_CHECKSUM_ERROR_MASK	BIT(5)
#   define  ENET_RX_BD_PROTOCOL_CHECKSUM_ERROR_MASK	BIT(4)
#   define  ENET_RX_BD_VLAN_FRAME_MASK			BIT(2)
#   define  ENET_RX_BD_IPv6_FRAME_MASK			BIT(1)
#   define  ENET_RX_BD_IPv4_FRAGMENT_MASK		BIT(0)

    uint16_t control_extend1;
#   define  ENET_RX_BD_GENERATE_INTERRUPT_MASK		BIT(7)
#   define  ENET_RX_BD_UNICAST_FRAME_MASK		BIT(8)
#   define  ENET_RX_BD_COLLISION_MASK			BIT(9)
#   define  ENET_RX_BD_PHY_ERROR_MASK			BIT(10)
#   define  ENET_RX_BD_MAC_ERROR_MASK			BIT(15)

    uint16_t payload_checksum;
    uint8_t header_length;
    uint8_t protocol_type;
    uint16_t reserved0;
    uint16_t control_extend2;
#   define  ENET_RX_BD_LAST_DESCRIPTOR_UPDATE_DONE_MASK	BIT(15)

    uint32_t timestamp;
    uint16_t reserved1;
    uint16_t reserved2;
    uint16_t reserved3;
    uint16_t reserved4;
} __attribute__ ((aligned(ENET_FRAME_BUFFER_DESCRIPTOR_ALIGNMENT)));

/**
 * Tx buffer descriptor (type of entries of the ENET Tx ring)
 */
struct enet_tx_buffer_descriptor {
    uint16_t data_length;
    uint16_t control;
#   define  ENET_TX_BD_READY_MASK			BIT(15)
#   define  ENET_TX_BD_SOFTWARE_OWNER1_MASK		BIT(14)
#   define  ENET_TX_BD_WRAP_MASK			BIT(13)
#   define  ENET_TX_BD_SOFTWARE_OWNER2_MASK		BIT(12)
#   define  ENET_TX_BD_LAST_IN_FRAME_MASK		BIT(11)
#   define  ENET_TX_BD_CRC_MASK				BIT(10)

    void *data_buffer;
    uint16_t control_extend0;
#   define  ENET_TX_BD_ERROR_MASK			BIT(15)
#   define  ENET_TX_BD_UNDERFLOW_MASK			BIT(13)
#   define  ENET_TX_BD_EXCESS_COLLISION_ERROR_MASK	BIT(12)
#   define  ENET_TX_BD_FRAME_ERROR_MASK			BIT(11)
#   define  ENET_TX_BD_LATE_COLLISION_ERROR_MASK	BIT(10)
#   define  ENET_TX_BD_FIFO_OVERFLOW_ERROR_MASK		BIT(9)
#   define  ENET_TX_BD_TMESTAMP_ERROR_MASK		BIT(8)

    uint16_t control_extend1;
#   define  ENET_TX_BD_INTERRUPT_MASK			BIT(14)
#   define  ENET_TX_BD_TIMESTAMP_MASK			BIT(13)
#   define  ENET_TX_BD_INSERT_PROTOCOL_CHECKSUM_MASK	BIT(12)
#   define  ENET_TX_BD_INSERT_IP_HEADER_CHECKSUM_MASK	BIT(11)

    uint16_t reserved0;
    uint16_t reserved1;
    uint16_t reserved2;
    uint16_t control_extend2;
#   define  ENET_TX_BD_LAST_DESCRIPTOR_UPDATE_DONE_MASK	BIT(15)

    uint32_t timestamp;
    uint16_t reserved3;
    uint16_t reserved4;
    uint16_t reserved5;
    uint16_t reserved6;
} __attribute__ ((aligned(ENET_FRAME_BUFFER_DESCRIPTOR_ALIGNMENT)));

C_ASSERT(sizeof(struct enet_rx_buffer_descriptor) % ENET_FRAME_BUFFER_DESCRIPTOR_ALIGNMENT == 0);

/**
 * Non-const fields of an Ethernet MAC device (to be placed in SRAM)
 */
struct enet_device_var {
    bool initialized;

    /**
     * Total number of Tx/Rx errors:
     */
    uint32_t tx_rx_error_count;

    /**
     * Pointer to local layer-3 end point associated with this ENET device
     */
    struct local_l3_end_point *local_l3_end_point_p;

    /**
     * Number of Tx buffer descriptors currently filled in the ENET Tx ring
     */
    uint16_t tx_ring_entries_filled;

    /**
     * Number of Rx buffer descriptors currently filled in the ENET Rx ring
     */
    uint16_t rx_ring_entries_filled;

    /**
     * ENET Tx ring write cursor (pointer to next Tx buffer descriptor
     * that can be filled by enet_start_xmit())
     */
    volatile struct enet_tx_buffer_descriptor *tx_ring_write_cursor;

    /**
     * ENET Tx ring read cursor (pointer to the first Tx buffer descriptor
     * that can be read by k64f_enet_transmit_interrupt_e_handler())
     */
    volatile struct enet_tx_buffer_descriptor *tx_ring_read_cursor;

    /**
     * ENET Rx ring write cursor (pointer to next Rx buffer descriptor
     * that can be filled by enet_repost_rx_packet())
     */
    volatile struct enet_rx_buffer_descriptor *rx_ring_write_cursor;

    /**
     * ENET Rx ring read cursor (pointer to the first Rx buffer descriptor
     * that can be read by k64f_enet_receive_interrupt_e_handler())
     */
    volatile struct enet_rx_buffer_descriptor *rx_ring_read_cursor;

    /**
     * Tx buffer descriptor ring accessed by the Ethernet MAC (ENET device)
     */
    volatile struct enet_tx_buffer_descriptor tx_buffer_descriptors[NET_MAX_TX_PACKETS];

    /**
     * Rx buffer descriptor ring accessed by the Ethernet MAC
     */
    volatile struct enet_rx_buffer_descriptor rx_buffer_descriptors[NET_MAX_RX_PACKETS];
};

#define ENET_PHY_ADDRESS    0x0

enum enet_mmfr_op_values {
    ENET_MMFR_OP_WRITE_NON_MII_COMPLIANT_FRAME = 0x0,
    ENET_MMFR_OP_WRITE_VALID_MII_MANAGEMENT_FRAME = 0x1,
    ENET_MMFR_OP_READ_VALID_MII_MANAGEMENT_FRAME = 0x2,
    ENET_MMFR_OP_READ_NON_MII_COMPLIANT_FRAME = 0x3,
};

/**
 * Const fields of an Ethernet MAC device (to be placed in flash)
 */
struct enet_device {
#   define ENET_DEVICE_SIGNATURE  GEN_SIGNATURE('E', 'N', 'E', 'T')
    uint32_t signature;
    const char *name;
    struct enet_device_var *var_p;
    volatile ENET_Type *mmio_registers_p;
    struct pin_info rmii_rxd0_pin;
    struct pin_info rmii_rxd1_pin;
    struct pin_info rmii_crs_dv_pin;
    struct pin_info rmii_rxer_pin;
    struct pin_info rmii_txen_pin;
    struct pin_info rmii_txd0_pin;
    struct pin_info rmii_txd1_pin;
    struct pin_info mii_txer_pin;
    struct pin_info mii_intr_pin;
    struct pin_info rmii_mdio_pin;
    struct pin_info rmii_mdc_pin;
    struct pin_info enet_1588_tmr_pins[4];
    struct rtos_interrupt_registration_params tx_rtos_interrupt_params;
    struct rtos_interrupt **tx_rtos_interrupt_pp;
    struct rtos_interrupt_registration_params rx_rtos_interrupt_params;
    struct rtos_interrupt **rx_rtos_interrupt_pp;
    struct rtos_interrupt_registration_params error_rtos_interrupt_params;
    struct rtos_interrupt **error_rtos_interrupt_pp;
    uint32_t clock_gate_mask;
    struct ethernet_mac_address mac_address;
    uint8_t mpu_region_index;
};

void enet_init(const struct enet_device *enet_device_p,
	       struct local_l3_end_point *local_l3_end_point_p);

void enet_start_xmit(const struct enet_device *enet_device_p,
		     struct network_packet *tx_packet_p);

void enet_repost_rx_packet(const struct enet_device *enet_device_p,
			   struct network_packet *rx_packet_p);

void enet_phy_write(const struct enet_device *enet_device_p, uint32_t phy_reg,
		    uint32_t data);

uint32_t enet_phy_read(const struct enet_device *enet_device_p,
		       uint32_t phy_reg);

extern isr_function_t k64f_enet_transmit_isr;

extern isr_function_t k64f_enet_receive_isr;

extern isr_function_t k64f_enet_error_isr;

void
k64f_enet_transmit_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void
k64f_enet_receive_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

void
k64f_enet_error_interrupt_e_handler(
    struct rtos_interrupt *rtos_interrupt_p);

extern const struct enet_device g_enet_device0;

#endif /* __K64F_SOC_ENET_H */
