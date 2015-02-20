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
#include <McRTOS/McRTOS_kernel_services.h>

#ifndef BOARD_INSTANCE
#define BOARD_INSTANCE	1
#endif

C_ASSERT(BOARD_INSTANCE == 1 || BOARD_INSTANCE == 2);

/*
 * Networking subsystem configuration options
 */
#define ENET_DATA_PAYLOAD_32_BIT_ALIGNED
#define	ENET_CHECKSUM_OFFLOAD
#undef	NET_TRACE

/**
 * Maximum transfer unit for Ethernet (frame size without CRC)
 */
#define ETHERNET_MAX_FRAME_DATA_SIZE	1500

/**
 * Maximum Ethernet frame size (in bytes) including CRC
 */
#define ETHERNET_MAX_FRAME_SIZE	    (ETHERNET_MAX_FRAME_DATA_SIZE + 18)

#if 0
/**
 * Maximum Ethernet VLAN frame size (in bytes)
 */
#define ENET_MAX_FRAME_VLAN_SIZE    (ETHERNET_MAX_FRAME_DATA_SIZE + 22)
#endif

/**
 * Network packet data buffer alignment in bytes
 * (required by the ENET device)
 */
#define NET_PACKET_DATA_BUFFER_ALIGNMENT UINT32_C(16)

/**
 * Network packet data buffer size rounded-up to the required alignment
 */
#define NET_PACKET_DATA_BUFFER_SIZE \
	ROUND_UP(ETHERNET_MAX_FRAME_SIZE, NET_PACKET_DATA_BUFFER_ALIGNMENT)

#define NET_MAX_IPV4_PACKET_PAYLOAD_SIZE \
        (NET_PACKET_DATA_BUFFER_SIZE - \
         (sizeof(struct ethernet_header) + sizeof(struct ipv4_header)))

#define NET_MAX_UDP_PACKET_PAYLOAD_SIZE \
        (NET_MAX_IPV4_PACKET_PAYLOAD_SIZE - sizeof(struct udp_header))

/**
 * Null IPv4 address (0.0.0.0)
 */
#define IPV4_NULL_ADDR	UINT32_C(0x0)

/**
 * Broadcast IPv4 address (255.255.255.255)
 */
#define IPV4_BROADCAST_ADDR UINT32_C(0xffffffff)

/**
 * Number of entries for the IPv4 ARP cache table
 */
#define ARP_CACHE_NUM_ENTRIES	16

/**
 * Number of entries for the IPv6 Neighbor cache table
 */
#define NEIGHBOR_CACHE_NUM_ENTRIES  16

/**
 * ARP cache entry lifetime in ticks (20 minutes)
 */
#define ARP_CACHE_ENTRY_LIFETIME_IN_TICKS \
	MILLISECONDS_TO_TICKS(20u * 60 * 1000)

/**
 * Timeout in milliseconds to wait for an ARP reply after sending a
 * non-gratuitous ARP request (3 minutes)
 */
#define ARP_REPLY_WAIT_TIMEOUT_IN_MS	(3u * 60 * 1000)

/**
 * Maximum number of ARP requests to be sent for a given destination
 * IP address, before failing with "unreachable destination".
 */
#define ARP_REQUEST_MAX_RETRIES	64

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

/**
 * Copies a MAC address. The source and destination must be at least
 * 2-byte aligned
 */
#define COPY_MAC_ADDRESS(_dest_p, _src_p) \
	do {								\
	    (_dest_p)->hwords[0] = (_src_p)->hwords[0];			\
	    (_dest_p)->hwords[1] = (_src_p)->hwords[1];			\
	    (_dest_p)->hwords[2] = (_src_p)->hwords[2];			\
	} while (0)

/**
 * Compares two MAC addresses. Their storage must be at least 2-byte aligned
 */
#define MAC_ADDRESSES_EQUAL(_mac_addr1_p, _mac_addr2_p) \
	 ((_mac_addr1_p)->hwords[0] == (_mac_addr2_p)->hwords[0] &&	\
	  (_mac_addr1_p)->hwords[1] == (_mac_addr2_p)->hwords[1] &&	\
	  (_mac_addr1_p)->hwords[2] == (_mac_addr2_p)->hwords[2])

/**
 * Copies an IPv4 address, where the source or destination are not 4-byte
 * aligned, but they must be at least 2-byte aligned.
 */
#define COPY_UNALIGNED_IPv4_ADDRESS(_dest_p, _src_p) \
	do {								\
	    (_dest_p)->hwords[0] = (_src_p)->hwords[0];			\
	    (_dest_p)->hwords[1] = (_src_p)->hwords[1];			\
	} while (0)

/**
 * Compares two IPv4 addresses stored at locations that are not 4-byte
 * aligned, but they must be at least 2-byte aligned.
 */
#define UNALIGNED_IPv4_ADDRESSES_EQUAL(_ip_addr1_p, _ip_addr2_p) \
	((_ip_addr1_p)->hwords[0] == (_ip_addr2_p)->hwords[0] &&	\
	 (_ip_addr1_p)->hwords[1] == (_ip_addr2_p)->hwords[1])

/**
 * Build an IPv4 subnet mask in network byte order (big endian),
 * assuming that the target CPU runs in little endian.
 *
 * NOTE: This macro can be used in static initializers, if
 * '_num_bits' is a constant,
 */
#define IPv4_SUBNET_MASK(_num_bits) \
	((_num_bits) % 8 == 0 ?						\
	     MULTI_BIT_MASK((_num_bits) - 1, 0) :			\
	     (MULTI_BIT_MASK((((_num_bits) / 8) * 8) - 1, 0) |		\
	      (MULTI_BIT_MASK(7, 7 - ((_num_bits) % 8)) <<		\
	       (((_num_bits) / 8) * 8))))

/**
 * Check if two IPv4 addresses are in the same subnet, for a given
 * subnet mask
 */
#define SAME_IPv4_SUBNET(_local_ip_addr_p, _dest_ip_addr_p, _subnet_mask) \
	(((_local_ip_addr_p)->value & (_subnet_mask)) == \
	 ((_dest_ip_addr_p)->value & (_subnet_mask)))

/**
 * Returns pointer to the data payload area of an IPv4 packet
 */
#define GET_IPV4_HEADER(_net_packet_p) \
        ((struct ipv4_header *)((_net_packet_p)->data_buffer +	\
				sizeof(struct ethernet_header)))

/**
 * Returns pointer to the data payload area of an IPv4 packet
 */
#define GET_IPV4_DATA_PAYLOAD_AREA(_net_packet_p)   \
        ((void *)((_net_packet_p)->data_buffer +    \
		  (sizeof(struct ethernet_header) + \
                   sizeof(struct ipv4_header))))


/**
 * Maximum number of local layer-4 end points
 */
#define NET_MAX_LOCAL_L4_END_POINTS 8

/**
 * Maximum number of Tx packet buffers
 */
#define NET_MAX_TX_PACKETS   8

/**
 * Maximum number of Rx packet buffers per layer-3 end point
 */
#define NET_MAX_RX_PACKETS   8

/**
 * First ephemeral port. All port number greater or equal
 * to this value are ephemeral ports.
 */
#define NET_FIRST_EPHEMERAL_PORT 49152

/**
 * DHCP client port
 */
#define DHCP_UDP_CLIENT_PORT	68

/**
 * DHCP server port
 */
#define DHCP_UDP_SERVER_PORT	67

/**
 * Ethernet MAC address in network byte order
 */
struct ethernet_mac_address {
    union {
	/**
	 * bytes[0] = most significant byte
	 * bytes[5] = less significant byte
	 */
	uint8_t bytes[6];
	uint16_t hwords[3];
    };
};

C_ASSERT(sizeof(struct ethernet_mac_address) == 6);

/**
 * Ethernet header in network byte order
 */
struct ethernet_header {
#   ifdef ENET_DATA_PAYLOAD_32_BIT_ALIGNED
    /**
     * Alignment padding so that data payload of the Ethernet frame
     * starts at a 32-bit boundary.
     */
    uint16_t alignment_padding;
#   endif

    /**
     * Destination MAC address
     */
    struct ethernet_mac_address dest_mac_addr;

    /**
     * Source MAC address
     */
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

#ifdef ENET_DATA_PAYLOAD_32_BIT_ALIGNED
C_ASSERT(sizeof(struct ethernet_header) == 16);
C_ASSERT(offsetof(struct ethernet_header, dest_mac_addr) == 2);
C_ASSERT(offsetof(struct ethernet_header, source_mac_addr) == 8);
C_ASSERT(offsetof(struct ethernet_header, frame_type) == 14);
#else
C_ASSERT(sizeof(struct ethernet_header) == 14);
C_ASSERT(offsetof(struct ethernet_header, dest_mac_addr) == 0);
C_ASSERT(offsetof(struct ethernet_header, source_mac_addr) == 6);
C_ASSERT(offsetof(struct ethernet_header, frame_type) == 12);
#endif


/**
 * IPv4 address in network byte order
 */
struct ipv4_address {
    union {
	/**
	 * bytes[0] = most significant byte
	 * bytes[3] = less significant byte
	 */
	uint8_t bytes[4];

	uint16_t hwords[2];

	/**
	 * IP address seen as a 32-bit value in big endian
	 */
	uint32_t value;
    } __attribute__((packed));
};
C_ASSERT(sizeof(struct ipv4_address) == sizeof(uint32_t));

/**
 * IPv6 address in network byte order
 */
struct ipv6_address {
    union {
	/**
	 * bytes[0] = most significant byte
	 * bytes[15] = less significant byte
	 */
	uint8_t bytes[16];

	/**
	 * Address seen as two 64-bit double words in big endian
	 */
	uint64_t dwords[2];
    } __attribute__((packed));
};
C_ASSERT(sizeof(struct ipv6_address) == sizeof(uint64_t) * 2);



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
 * Transport protocols encapsulated in IP packets
 */
enum l4_protocols {
    TRANSPORT_PROTO_ICMP = 0x1,
    TRANSPORT_PROTO_TCP =  0x6,
    TRANSPORT_PROTO_UDP =  0x11
};

/**
 * Header of an IPv4 packet in network byte order
 * (An IPv4 packet is encapsulated in an Ethernet frame)
 */
struct ipv4_header {
    /**
     * IP version and header length
     * - Version is 4 for IPv4
     * - Header length is in 32-bit words and if there are
     *   no options, its value is 5
     */
    uint8_t version_and_header_length;
#   define IP_VERSION_MASK	    MULTI_BIT_MASK(7, 4)
#   define IP_VERSION_SHIFT	    4
#   define IP_HEADER_LENGTH_MASK    MULTI_BIT_MASK(3, 0)
#   define IP_HEADER_LENGTH_SHIFT   0

    /**
     * type of service
     */
    uint8_t type_of_service;
#   define TOS_PRECEDENCE_MASK		    MULTI_BIT_MASK(7, 5)
#   define TOS_PRECEDENCE_SHIFT		    5
#   define TOS_MINIMIZE_DELAY_MASK	    BIT(4)
#   define TOS_MAXIMIZE_THROUGHPUT_MASK	    BIT(3)
#   define TOS_MAXIMIZE_RELIABILITY_MASK    BIT(2)
#   define TOS_MINIMIZE_MONETARY_COST_MASK  BIT(1)

    /**
     * total packet length (header + data payload) in bytes
     * (hton16() must be invoked before writing this field.
     *  ntoh16() must be invoked after reading this field.)
     */
    uint16_t total_length;

    /**
     * Identifcation number for the IP packet
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
#   define IP_FLAG_RESERVED_MASK	BIT(15)
#   define IP_FLAG_DONT_FRAGMENT_MASK	BIT(14)
#   define IP_FLAG_MORE_FRAGMENTS_MASK	BIT(13)
#   define IP_FRAGMENT_OFFSET_MASK	MULTI_BIT_MASK(12, 0)
#   define IP_FRAGMENT_OFFSET_SHIFT	0

    /**
     * Packet time to live
     */
    uint8_t time_to_live;

    /**
     * Transport protocol type (values from enum l4_protocols)
     */
    uint8_t protocol_type;

    /**
     * Header checksum
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

C_ASSERT(sizeof(struct ipv4_header) == 20);

/**
 * Header of an IPv6 packet in network byte order
 * (An IPv6 packet is encapsulated in an Ethernet frame)
 */
struct ipv6_header {
    /**
     * First 32-bit word:
     * - Version is 6 for IPv6
     * - Header length is in 32-bit words and if there are
     *   no options, its value is 5
     */
    uint32_t first_word;
#   define IPv6_VERSION_MASK	    MULTI_BIT_MASK(3, 0)
#   define IPv6_VERSION_SHIFT	    0
#   define IPv6_TRAFFIC_CLASS_MASK  MULTI_BIT_MASK(11, 4)
#   define IPv6_TRAFFIC_CLASS_SHIFT 4
#   define IPv6_FLOW_LABEL_MASK	    MULTI_BIT_MASK(31, 12)
#   define IPv6_FLOW_LABEL_SHIFT    12

    /**
     * Payload length
     */
    uint16_t payload_length;

    /**
     * Next header type (values from enum l4_protocols)
     */
    uint8_t next_header;

    /**
     * Hop limit
     */
    uint8_t hop_limit;

    /**
     * Source (sender) IPv6 address
     */
    struct ipv6_address source_ipv6_addr;

    /**
     * Destination (receiver) IPv6 address
     */
    struct ipv6_address dest_ipv6_addr;
};

C_ASSERT(sizeof(struct ipv6_header) == 40);

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
 * IPv4 ICMPv4 header layout
 * (An ICMPv4 message is encapsulated in an IPv4 packet)
 */
struct icmpv4_header {
    /**
     * Message type
     */
    uint8_t msg_type;
#   define ICMP_TYPE_PING_REPLY	    0
#   define ICMP_TYPE_PING_REQUEST   8
    /**
     * Message code
     */
    uint8_t msg_code;
#   define ICMP_CODE_PING_REPLY	    0
#   define ICMP_CODE_PING_REQUEST   0

    /**
     * message checksum
     */
    uint16_t msg_checksum;
}; //  __attribute__((packed));

C_ASSERT(sizeof(struct icmpv4_header) == 4);

/**
 * ICMPv4 echo request/reply message layout
 */
struct icmpv4_echo_message {
	struct icmpv4_header header;
	uint16_t identifier;
	uint16_t seq_num;
};

C_ASSERT(offsetof(struct icmpv4_echo_message, identifier) ==
	 sizeof(struct icmpv4_header));

/**
 * IPv4 DHCP message layout
 */
struct dhcp_message {
	uint8_t op;
	uint8_t hardware_type;
	uint8_t hw_addr_len;
	uint8_t hops;
	uint32_t transaction_id;
	uint16_t seconds;
	uint16_t flags;
	struct ipv4_address client_ip_addr;
	struct ipv4_address your_ip_addr;
	struct ipv4_address next_server_ip_addr;
	struct ipv4_address relay_agent_ip_addr;
	struct ethernet_mac_address client_mac_addr;
	uint8_t zero_filled[10 + 192];
	uint32_t magic_cookie;
	uint8_t options[];
};

/**
 * IPv6 ICMPv6 header layout
 * (An ICMPv6 message is encapsulated in an IPv6 packet)
 */
struct icmpv6_header {
    /**
     * Message type
     */
    uint8_t msg_type;

    /**
     * Message code
     */
    uint8_t msg_code;

    /**
     * Message checksum
     */
    uint16_t msg_checksum;

    /**
     * Message data
     */
    union {
	uint32_t data32[1];
	uint16_t data16[2];
	uint8_t  data8[4];
    };
}; //  __attribute__((packed));

C_ASSERT(sizeof(struct icmpv6_header) == 8);

/**
 * UDP header layout
 * (A UDP datagram is encapsulated in an IP packet)
 */
struct udp_header {
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
     */
    uint16_t datagram_checksum;
}; //  __attribute__((packed));

C_ASSERT(sizeof(struct udp_header) == 8);

enum arp_cache_entry_states {
    ARP_ENTRY_INVALID = 0,
    ARP_ENTRY_HALF_FILLED, /* arp reques sent but no reply received yet */
    ARP_ENTRY_FILLED,
};

/**
 * IPv4 ARP cache entry
 */
struct arp_cache_entry {
    struct ipv4_address dest_ip_addr;
    struct ethernet_mac_address dest_mac_addr;
    enum arp_cache_entry_states state;

    /**
     * Timestamp in ticks when the last ARP request for this entry was sent.
     * It is used to determine if we have waited too long for the ARP reply,
     * and need to send another ARP request.
     */
    rtos_ticks_t arp_request_time_stamp;

    /**
     * Timestamp in ticks when this entry was last filled.
     * It is used to determine when the entry has expired and a new ARP request
     * must be sent.
     */
    rtos_ticks_t entry_filled_time_stamp;

    /**
     * Timestamp in ticks when the last lookup was done for this entry. It is
     * used to determine the least recently used entry, for cache entry
     * replacement.
     */
    rtos_ticks_t last_lookup_time_stamp;
};

/**
 * IPv4 ARP cache
 */
struct arp_cache {
    /**
     * Mutex to serialize access to the ARP cache
     */
    struct rtos_mutex mutex;

    /**
     * Condvar signaled when the ARP cache is updated
     */
    struct rtos_condvar cache_updated_condvar;

    /**
     * Array of cache entries
     */
    struct arp_cache_entry entries[ARP_CACHE_NUM_ENTRIES];
};

enum neighbor_cache_entry_states {
    NEIGHBOR_ENTRY_INVALID = 0,
};

/**
 * IPv6 neighbor cache entry
 */
struct neighbor_cache_entry {
    struct ipv6_address dest_ipv6_addr;
    struct ethernet_mac_address dest_mac_addr;
    enum neighbor_cache_entry_states state;
};

/**
 * IPv6 Neighbor cache
 */
struct neighbor_cache {
    /**
     * Mutex to serialize access to the Neighbor cache
     */
    struct rtos_mutex mutex;

    /**
     * Array of cache entries
     */
    struct neighbor_cache_entry entries[NEIGHBOR_CACHE_NUM_ENTRIES];
};

/**
 * Llocal layer-3 end point configuration
 */
struct local_l3_end_point_config {
    /**
     * Local IPv4 address
     */
    struct ipv4_address ipv4_addr;

    /**
     * Subnet mask in network byte order
     */
    uint32_t ipv4_subnet_mask;

    /**
     * Local IPv4 address
     */
    struct ipv4_address default_gateway_ipv4_addr;

    /**
     * Local IPv6 address
     */
    struct ipv6_address ipv6_addr;

    /**
     * Local IPv6 address
     */
    struct ipv6_address default_gateway_ipv6_addr;
};

/**
 * IPv4 network end point
 */
struct ipv4_end_point {
    /**
     * Local IPv4 address
     */
    struct ipv4_address local_ip_addr;

    /**
     * Subnet mask in network byte order
     */
    uint32_t subnet_mask;

    /**
     * Local IPv4 address
     */
    struct ipv4_address default_gateway_ip_addr;

    /**
     * DHCP lease time in seconds
     */
    uint32_t dhcp_lease_time;

    /**
     * Sequence number to use as the 'identification' field of the next
     * IP packet transmitted out of this network end-point
     */
    uint16_t next_tx_ip_packet_seq_num;

    /**
     * Queue of received ICMPv4 packets
     */
    struct rtos_queue rx_icmpv4_packet_queue;

    /**
     * ARP cache
     */
    struct arp_cache arp_cache;
};

/**
 * IPv6 network end point
 */

struct ipv6_end_point {
    /**
     * Local IPv6 address
     */
    struct ipv6_address local_ip_addr;

    /**
     * Local IPv6 address
     */
    struct ipv6_address default_gateway_ip_addr;

    /**
     * Neighbor cache
     */
    struct neighbor_cache neighbor_cache;
};

/**
 * Network packet object
 */
struct network_packet {
    uint32_t signature;
#   define NET_TX_PACKET_SIGNATURE  GEN_SIGNATURE('T', 'X', 'B', 'U')
#   define NET_RX_PACKET_SIGNATURE  GEN_SIGNATURE('R', 'X', 'B', 'U')

    /**
     * ENET-MAC device buffer descriptor associated with the packet
     */
    union {
	volatile struct enet_tx_buffer_descriptor *tx_buf_desc_p;
	volatile struct enet_rx_buffer_descriptor *rx_buf_desc_p;
    };

    /**
     * Pointer to the local layer-3 end point that owns this network packet
     * object. Only meaningful for Rx packets.
     */
    struct local_l3_end_point *local_l3_end_point_p;

    uint16_t state_flags;
#   define NET_PACKET_IN_TX_TRANSIT		BIT(0)
#   define NET_PACKET_IN_RX_TRANSIT		BIT(1)
#   define NET_PACKET_IN_TX_USE_BY_APP		BIT(2)
#   define NET_PACKET_IN_RX_USE_BY_APP		BIT(3)
#   define NET_PACKET_FREE_AFTER_TX_COMPLETE	BIT(4)
#   define NET_PACKET_IN_RX_QUEUE		BIT(5)
#   define NET_PACKET_RX_FAILED			BIT(6)
#   define NET_PACKET_IN_TX_POOL		BIT(7)
#   define NET_PACKET_IN_ICMP_QUEUE		BIT(8)

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
    uint8_t data_buffer[NET_PACKET_DATA_BUFFER_SIZE]
	__attribute__ ((aligned(NET_PACKET_DATA_BUFFER_ALIGNMENT)));
};

C_ASSERT(sizeof(bool) == sizeof(uint8_t));
C_ASSERT(offsetof(struct network_packet, data_buffer) %
	 NET_PACKET_DATA_BUFFER_ALIGNMENT == 0);

#define BUFFER_TO_NETWORK_PACKET(_data_buf) \
	((struct network_packet *) \
	 ((uintptr_t)(_data_buf) - offsetof(struct network_packet, data_buffer)))

#define GLIST_NODE_TO_NETWORK_PACKET(_node_p) \
	GLIST_NODE_ENTRY(_node_p, struct network_packet, node)

/**
 * Local layer-3 (network layer) end point
 */
struct local_l3_end_point {
#   define LOCAL_L3_END_POINT_SIGNATURE	GEN_SIGNATURE('L', '3', 'E', 'P')
    uint32_t signature;

    /**
     * Ethernet network interface (NIC)
     */
    const struct enet_device *enet_device_p;

    /**
     * Queue of received Rx packets (non-empty Rx buffers)
     */
    struct rtos_queue rx_packet_queue;

    /**
     * Rx packet buffers
     */
    struct network_packet rx_packets[NET_MAX_RX_PACKETS];

    /**
     * IPv4 end point (used if layer3_packet_type == ENET_IPv4_PACKET)
     */
    struct ipv4_end_point ipv4;

    /**
     * IPv6 end point (used if layer3_packet_type == ENET_IPv6_PACKET)
     */
    struct ipv6_end_point ipv6;
};

/**
 * Local Layer-4 (transport layer) end point
 *
 * NOTE: There is no explicit IP address associated with a local l4
 * end-point. The local IP address is implicitly "INADDR_ANY".
 */
struct local_l4_end_point {
    /**
     * Transport protocol type
     */
    enum l4_protocols l4_protocol;

    /**
     * Protocol-specific port number (must be different from 0)
     */
    uint16_t l4_port;

    /**
     * Queue of incoming network packets received for this
     * layer-4 end point
     */
    struct rtos_queue l4_rx_packet_queue;
};

/**
 * Networking state variables
 */
struct networking {
    /**
     * Flag indicating if the networking subsystem has been initialized
     */
    bool initialized;

    /**
     * Flag indicating if there is an outstanding ping request for which no reply
     * has been received yet
     */
    bool expecting_ping_reply;

    /**
     * Next ephemeral port to assign to local UDP end point.
     */
    uint16_t next_udp_ephemeral_port;

    /**
     * Next ephemeral port to assign to local TCP end point.
     */
    uint16_t next_tcp_ephemeral_port;

    /**
     * Queue of received IPPv4 ping replies
     */
    struct rtos_queue rx_ipv4_ping_reply_packet_queue;

    /**
     * Pointer to the next free entry in local_l4_end_points[]
     */
    struct local_l4_end_point *next_free_l4_end_point_p;

    /**
     * Mutex to serialize access to expecting_ping_reply
     */
    struct rtos_mutex expecting_ping_reply_mutex;

    /**
     * Condvar to be signal when the ping reply for an outstanding ping request
     * has been received.
     */
    struct rtos_condvar ping_reply_recceived_condvar;

    /**
     * Mutex to serialize access calls to table of local layer-4 end points
     */
    struct rtos_mutex local_l4_end_points_mutex;

    /**
     * Pool of free Tx packets
     */
    struct rtos_queue free_tx_packet_pool;

    /**
     * Tx packet buffers (shared among all local layer-3 end points)
     */
    struct network_packet tx_packets[NET_MAX_TX_PACKETS];

    /**
     * Local layer-3 end point
     */
    struct local_l3_end_point local_l3_end_point;

    /**
     * Local layer-4 end points
     */
    struct local_l4_end_point local_l4_end_points[NET_MAX_LOCAL_L4_END_POINTS];
};

C_ASSERT(NET_MAX_IPV4_PACKET_PAYLOAD_SIZE <=
         UINT16_MAX - sizeof(struct ipv4_header));

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

/**
 * Returns pointer to the data payload area of a UDP datagram
 */
static inline void *net_get_udp_data_payload_area(struct network_packet *net_packet_p)
{
    if (net_packet_p->signature == NET_RX_PACKET_SIGNATURE) {
#   ifdef _RELIABILITY_CHECKS_
	struct ipv4_header *ipv4_header_p = GET_IPV4_HEADER(net_packet_p);
#   endif
	FDC_ASSERT(ipv4_header_p->protocol_type == TRANSPORT_PROTO_UDP,
		   ipv4_header_p->protocol_type, net_packet_p);
    } else {
	FDC_ASSERT(net_packet_p->signature == NET_TX_PACKET_SIGNATURE,
		   net_packet_p->signature, net_packet_p);
    }

    return (void *)((uint8_t *)GET_IPV4_DATA_PAYLOAD_AREA(net_packet_p) +
		    sizeof(struct udp_header));
}


/**
 * Returns the data payload length of an incoming UDP datagram
 */
static inline size_t net_get_udp_data_payload_length(struct network_packet *net_packet_p)
{
    FDC_ASSERT(net_packet_p->signature == NET_RX_PACKET_SIGNATURE,
	       net_packet_p->signature, net_packet_p);

#ifdef _RELIABILITY_CHECKS_
    struct ipv4_header *ipv4_header_p = GET_IPV4_HEADER(net_packet_p);
#endif

    FDC_ASSERT(ipv4_header_p->protocol_type == TRANSPORT_PROTO_UDP,
	       ipv4_header_p->protocol_type, net_packet_p);

    struct udp_header *udp_header_p =
	(struct udp_header *)GET_IPV4_DATA_PAYLOAD_AREA(net_packet_p);

    return ntoh16(udp_header_p->datagram_length) - sizeof(struct udp_header);
}


void networking_init(void);

void
net_set_local_ipv4_address(const struct ipv4_address *ip_addr_p,
			   uint8_t subnet_prefix);

struct network_packet *net_allocate_tx_packet(bool free_after_tx_complete);

void net_free_tx_packet(struct network_packet *tx_packet_p);

void net_dequeue_rx_packet(struct local_l3_end_point *local_l3_end_point_p,
			   struct network_packet **rx_packet_pp);

void net_enqueue_rx_packet(struct local_l3_end_point *local_l3_end_point_p,
			   struct network_packet *rx_packet_p);

void
net_recycle_rx_packet(struct network_packet *rx_packet_p);

fdc_error_t
net_send_ipv4_packet(const struct ipv4_address *dest_ip_addr_p,
		     struct network_packet *tx_packet_p,
		     size_t data_payload_length,
		     enum l4_protocols l4_protocol);

fdc_error_t
net_create_local_l4_end_point(enum l4_protocols l4_protocol,
	                      uint16_t l4_port,
			      struct local_l4_end_point **local_l4_end_point_pp);

fdc_error_t
net_send_ipv4_udp_datagram(struct local_l4_end_point *local_l4_end_point_p,
		           const struct ipv4_address *dest_ip_addr_p,
			   uint16_t dest_port,
		           struct network_packet *tx_packet_p,
		           size_t data_payload_length);

fdc_error_t
net_receive_ipv4_udp_datagram(struct local_l4_end_point *local_l4_end_point_p,
			      rtos_milliseconds_t timeout_ms,
		              struct ipv4_address *source_ip_addr_p,
			      uint16_t *source_port_p,
			      struct network_packet **rx_packet_pp);

void
net_send_ipv4_tcp_segment(struct local_l4_end_point *local_l4_end_point_p,
		          const struct ipv4_address *dest_ip_addr_p,
			  uint16_t dest_port,
		          struct network_packet *tx_packet_p,
		          size_t data_payload_length);

fdc_error_t
net_send_ipv4_icmp_message(const struct ipv4_address *dest_ip_addr_p,
		           struct network_packet *tx_packet_p,
			   uint8_t msg_type,
		           uint8_t msg_code,
		           size_t data_payload_length);

fdc_error_t
net_send_ipv4_ping_request(const struct ipv4_address *dest_ip_addr_p,
			   uint16_t identifier,
		           uint16_t seq_num);

fdc_error_t
net_receive_ipv4_ping_reply(rtos_milliseconds_t timeout_ms,
			    struct ipv4_address *remote_ip_addr_p,
			    uint16_t *identifier_p,
			    uint16_t *seq_num_p);

#endif /* _NETWORKING_H */
