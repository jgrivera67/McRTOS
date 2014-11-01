/**
 * @file networking.h
 *
 * Copyright (C) 2014 German Rivera
 *
 * @author German Rivera
 */
#ifndef _NETWORKING_H
#define _NETWORKING_H

#include <McRTOS/compile_time_checks.h>

/**
 * Maximum transfer unit for Ethernet (frame size without CRC)
 */
#define NETWORK_MTU 1500

/**
 * Convert a 16-bit value from host byte order to network byte order
 * (Do byte swap since Cortex-M is little endian)
 */
#define hton16(_x)  byte_swap16(_x)

/**
 * Convert a 16-bit value from network byte order to host byte order
 * (Do byte swap since Cortex-M is little endian)
 */
#define ntoh16(_x)  byte_swap16(_x)

/**
 * Convert a 32-bit value from host byte order to network byte order
 * (Do byte swap since Cortex-M is little endian)
 */
#define hton32(_x)  byte_swap32(_x)

/**
 * Convert a 32-bit value from network byte order to host byte order
 * (Do byte swap since Cortex-M is little endian)
 */
#define ntoh32(_x)  byte_swap32(_x)

#define ENET_COPY_MAC_ADDRESS(_dest, _src) \
	do {								\
	    if ((uintptr_t)(_dest) % 4 == 0 &&				\
	        (uintptr_t)(_src) % 4 == 0) {				\
		*(uint32_t *)((_dest)->bytes) =				\
		    *(uint32_t *)((_src)->bytes);			\
		*(uint16_t *)((_dest)->bytes + 4) =			\
		    *(uint16_t *)((_src)->bytes + 4);			\
	    } else {							\
		*(uint16_t *)((_dest)->bytes) =				\
		    *(uint16_t *)((_src)->bytes);			\
		*(uint16_t *)((_dest)->bytes + 2) =			\
		    *(uint16_t *)((_src)->bytes + 2);			\
		*(uint16_t *)((_dest)->bytes + 4) =			\
		    *(uint16_t *)((_src)->bytes + 4);			\
	    }								\
	} while (0)

#define ENET_COPY_IP_ADDRESS(_dest, _src) \
	do {								\
	    if ((uintptr_t)(_dest) % 4 == 0 &&				\
	        (uintptr_t)(_src) % 4 == 0) {				\
		*(uint32_t *)((_dest)->bytes) =				\
		    *(uint32_t *)((_src)->bytes);			\
	    } else {							\
		*(uint16_t *)((_dest)->bytes) =				\
		    *(uint16_t *)((_src)->bytes);			\
		*(uint16_t *)((_dest)->bytes + 2) =			\
		    *(uint16_t *)((_src)->bytes + 2);			\
	    }								\
	} while (0)

/**
 * Ethernet MAC address in network byte order
 */
struct ethernet_mac_address {
    /**
     * bytes[0] = most significant byte
     * bytes[5] = less significant byte
     */
    uint8_t bytes[6];
}; // __attribute__((packed));

C_ASSERT(sizeof(struct ethernet_mac_address) == 6);

/**
 * Ethernet header in network byte order
 */
struct ethernet_header {
    struct ethernet_mac_address dest_mac_addr;
    struct ethernet_mac_address source_mac_addr;

    /**
     * Ethernet frame type
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t frame_type;
#   define ENET_ARP_PACKET	    0x806
#   define ENET_IPv4_PACKET	    0x800
#   define ENET_VLAN_TAGGED_FRAME   0x8100
#   define ENET_IPv6_PACKET	    0x86dd
}; // __attribute__((packed));

C_ASSERT(sizeof(struct ethernet_header) == 14);
C_ASSERT(offsetof(struct ethernet_header, dest_mac_addr) == 0);
C_ASSERT(offsetof(struct ethernet_header, source_mac_addr) == 6);
C_ASSERT(offsetof(struct ethernet_header, frame_type) == 12);

/**
 * IPv4 address in network byte order
 */
struct ipv4_address {
    /**
     * bytes[0] = most significant byte
     * bytes[3] = less significant byte
     */
    uint8_t bytes[4];
};

C_ASSERT(sizeof(struct ipv4_address) == 4);

/**
 * ARP packet layout in network byte order
 * (An ARP packet is encapsulated in an Ethernet frame)
 */
struct arp_packet {
    /**
     * Link address type is 0x1 for Ethernet
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t link_addr_type;

    /**
     * Network address type is 0x800 for IPv4
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t network_addr_type;

    /**
     * link address size is 6 for MAC addresses
     */
    uint8_t link_addr_size;

    /**
     * Network address size is 4 for IPv4 addresses
     */
    uint8_t network_addr_size;

    /**
     * ARP operation (ARP_REQUEST or ARP_REPLY)
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t operation;
#   define ARP_REQUEST	0x1
#   define ARP_REPLY	0x2

    /**
     * Source (sender) MAC address
     */
    struct ethernet_mac_address source_mac_addr;

    /**
     * Source (sender) IPv4 address
     */
    struct ipv4_address source_ip_addr;

    /**
     * Destination (target) MAC address
     */
    struct ethernet_mac_address dest_mac_addr;

    /**
     * Destination (target) IPv4 address
     */
    struct ipv4_address dest_ip_addr;
}; // __attribute__((packed));

C_ASSERT(offsetof(struct arp_packet, source_mac_addr) == 8);
C_ASSERT(offsetof(struct arp_packet, source_ip_addr) == 14);
C_ASSERT(offsetof(struct arp_packet, dest_mac_addr) == 18);
C_ASSERT(offsetof(struct arp_packet, dest_ip_addr) == 24);

/**
 * Header of an IPv4 packet in network byte order
 * (An IPv4 packet is encapsulated in an Ethernet frame)
 */
struct ipv4_header {
    /**
     * IP version and header length
     */
    uint8_t version_and_header_length;
#   define IP_VERSION_MASK	    MULTI_BIT_MASK(3, 0)
#   define IP_VERSION_SHIFT	    0
#   define IP_HEADER_LENGTH_MASK    MULTI_BIT_MASK(7, 4)
#   define IP_HEADER_LENGTH_SHIFT   0

    /**
     * type of service
     */
    uint8_t type_of_service;
#   define TOS_PRECEDENCE_MASK		    MULTI_BIT_MASK(2, 0)
#   define TOS_PRECEDENCE_SHIFT		    0
#   define TOS_MINIMIZE_DELAY_MASK	    BIT(3)
#   define TOS_MAXIMIZE_THROUGHPUT_MASK	    BIT(4)
#   define TOS_MAXIMIZE_RELIABILITY_MASK    BIT(5)
#   define TOS_MINIMIZE_MONETARY_COST_MASK  BIT(6)

    /**
     * total packet length (header + data payload) in bytes
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t total_length;

    /**
     * total packet length (header + data payload) in bytes
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t identification;

    /**
     * Flags and fragment_offset
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t flags_and_fragment_offset;
#   define IP_FLAGS_MASK	    MULTI_BIT_MASK(2, 0)
#   define IP_FLAGS_SHIFT	    0
#   define IP_FRAGMENT_OFFSET_MASK  MULTI_BIT_MASK(15, 3)
#   define IP_FRAGMENT_OFFSET_SHIFT 3

    /**
     * Packet time to live
     */
    uint8_t time_to_live;

    /**
     * Transport protocol type
     */
    uint8_t protocol_type;

    /**
     * Header checksum
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t header_checksum;

    /**
     * Source (sender) IPv4 address
     */
    struct ipv4_address source_ip_addr;

    /**
     * Destination (receiver) IPv4 address
     */
    struct ipv4_address dest_ip_addr;
}; //  __attribute__((packed));

/**
 * Ethernet frame layout
 */
struct ethernet_frame {
    struct ethernet_header enet_header;
    union {
	struct arp_packet arp_packet;
	struct ipv4_header ipv4_header;
    };
}; // __attribute__((packed));

C_ASSERT(offsetof(struct ethernet_frame, arp_packet) ==
	 sizeof(struct ethernet_header));
C_ASSERT(offsetof(struct ethernet_frame, ipv4_header) ==
	 sizeof(struct ethernet_header));

/**
 * ICMP message layout
 * (An ICMP message is encapsulated in an IP packet)
 */
struct icmp_message {
    /**
     * Message type
     */
    uint8_t msg_type;

    /**
     * Message code
     */
    uint8_t msg_code;

    /**
     * message checksum
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t msg_checksum;

    /**
     * Data payload (if any)
     */
    uint8_t data[];
}; //  __attribute__((packed));

/**
 * UDP datagram layout
 * (A UDP datagram is encapsulated in an IP packet)
 */
struct udp_datagram {
    /**
     * Source port number
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t source_port;

    /**
     * Destination port number
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t dest_port;

    /**
     * UDP datagram length (header + data payload) in bytes
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t datagram_length;

    /**
     * UDP datagram checksum
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t datagram_checksum;

    /**
     * Data payload (if any)
     */
    uint8_t data[];
}; //  __attribute__((packed));


/**
 * Transport protocol end point
 */
struct transport_end_point {
    /**
     * Protocol-specific port number
     */
    uint16_t port;
};

/**
 * Invert byte order of a 16-bit value
 */
static inline uint16_t byte_swap16(uint16_t value)
{
    uint16_t swapped_val;

    asm volatile (
            "rev16 %[swapped_val], %[value]\n\t"
            : [swapped_val] "=r" (swapped_val)
            : [value] "r" (value)
    );

    return swapped_val;
}


/**
 * Invert byte order of a 32-bit value
 */
static inline uint32_t byte_swap32(uint32_t value)
{
    uint32_t swapped_val;

    asm volatile (
            "rev %[swapped_val], %[value]\n\t"
            : [swapped_val] "=r" (swapped_val)
            : [value] "r" (value)
    );

    return swapped_val;
}

void networking_init(void);

struct enet_device;

void
net_apr_send_request(const struct enet_device *enet_device_p,
		     const struct ipv4_address *source_ip_addr_p,
		     const struct ipv4_address *dest_ip_addr_p);

#endif /* _NETWORKING_H */
