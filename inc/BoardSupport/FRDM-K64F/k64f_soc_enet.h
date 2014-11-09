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

C_ASSERT(NETWORK_MTU == 1500);

/**
 * Maximum Ethernet frame size (in bytes) including CRC
 */
#define ENET_MAX_FRAME_SIZE	    (NETWORK_MTU + 18)

/**
 * Maximum Ethernet VLAN frame size (in bytes)
 */
#define ENET_MAX_FRAME_VLAN_SIZE    (NETWORK_MTU + 22)

/*
 * Ethernet frame buffer descriptor alignment in bytes
 */
#define ENET_FRAME_BUFFER_DESCRIPTOR_ALIGNMENT UINT32_C(16)

/*
 * Ethernet frame data buffer alignment in bytes
 */
#define ENET_FRAME_DATA_BUFFER_ALIGNMENT UINT32_C(16)

#define ENET_MAX_RX_PACKETS   8

#define ENET_MAX_TX_PACKETS   8

/**
 * Maximum Ethernet frame size rounded-up to the required alignment
 */
#define ENET_ALIGNED_MAX_FRAME_SIZE \
	ROUND_UP(ENET_MAX_FRAME_SIZE, ENET_FRAME_DATA_BUFFER_ALIGNMENT)

/**
 * Network packet object
 */
struct network_packet {
    uint32_t signature;
#   define ENET_TX_PACKET_SIGNATURE  GEN_SIGNATURE('T', 'X', 'B', 'U')
#   define ENET_RX_PACKET_SIGNATURE  GEN_SIGNATURE('R', 'X', 'B', 'U')

    union {
	volatile struct enet_tx_buffer_descriptor *tx_buf_desc_p;
	volatile struct enet_rx_buffer_descriptor *rx_buf_desc_p;
    };

    uint16_t state_flags;
#   define ENET_FRAME_IN_TX_TRANSIT		BIT(0)
#   define ENET_FRAME_IN_RX_TRANSIT		BIT(1)
#   define ENET_FRAME_IN_TX_USE_BY_APP		BIT(2)
#   define ENET_FRAME_IN_RX_USE_BY_APP		BIT(3)
#   define ENET_FRAME_FREE_AFTER_TX_COMPLETE	BIT(4)
#   define ENET_FRAME_IN_RX_QUEUE		BIT(5)
#   define ENET_FRAME_RX_DROPPED		BIT(6)
#   define ENET_FRAME_IN_TX_POOL		BIT(7)

    /**
     * Total packet length, including L2 L3 and L4 headers
     */
    uint16_t total_length;

    /**
     * Embedded node used to insert this packet in a list of packets
     * as it traverses the networking stack.
     */
    struct glist_node node;

    /*
     * Packet payload data buffer
     */
    uint8_t data_buffer[ENET_ALIGNED_MAX_FRAME_SIZE]
	__attribute__ ((aligned(ENET_FRAME_DATA_BUFFER_ALIGNMENT)));
};

C_ASSERT(ENET_ALIGNED_MAX_FRAME_SIZE >= 256 &&
	 (ENET_ALIGNED_MAX_FRAME_SIZE & ~ENET_MRBR_R_BUF_SIZE_MASK) == 0);

C_ASSERT(sizeof(bool) == sizeof(uint8_t));
C_ASSERT(offsetof(struct network_packet, data_buffer) %
	 ENET_FRAME_DATA_BUFFER_ALIGNMENT == 0);

#define TO_NETWORK_PACKET(_data_buf) \
	((struct network_packet *) \
	 ((uintptr_t)(_data_buf) - offsetof(struct network_packet, data_buffer)))

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
     * Tx buffer descriptor ring accessed by the Ethernet MAC
     */
    volatile struct enet_tx_buffer_descriptor tx_buffer_descriptors[ENET_MAX_TX_PACKETS];

    /**
     * Rx buffer descriptor ring accessed by the Ethernet MAC
     */
    volatile struct enet_rx_buffer_descriptor rx_buffer_descriptors[ENET_MAX_RX_PACKETS];

    /**
     * Circular buffer of pointers used to keep track of free Tx packets
     */
    struct rtos_circular_buffer tx_packet_pool;

    /**
     * Circular buffer of pointers used to represent the queue of received
     * Rx packets (non-empty Rx buffers)
     */
    struct rtos_circular_buffer rx_packet_queue;

    /**
     * Array of entries for tx_packet_pool
     */
    void *tx_packet_pool_entries[ENET_MAX_TX_PACKETS];

    /**
     * Array of entries for rx_packet_queue
     */
    void *rx_packet_queue_entries[ENET_MAX_RX_PACKETS];

    /**
     * Ethernet Tx packets (for outgoing Ethernet frames)
     */
    struct network_packet tx_packets[ENET_MAX_TX_PACKETS];

    /**
     * Ethernet Rx packets (for incoming Ethernet frames)
     */
    struct network_packet rx_packets[ENET_MAX_RX_PACKETS];
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
