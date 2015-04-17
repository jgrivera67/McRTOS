/**
 * @file networking.c
 *
 * Networking initialization
 *
 * Copyright (C) 2014 German Rivera
 *
 * @author German Rivera
 */

#include <Networking/networking.h>
#include <McRTOS/McRTOS.h>
#include <McRTOS/failure_data_capture.h>
#include <McRTOS/utils.h>

#pragma GCC diagnostic ignored "-Wunused-variable" // ???
#pragma GCC diagnostic ignored "-Wunused-parameter" // ???
#pragma GCC diagnostic ignored "-Wunused-function" // ???

#ifdef NET_TRACE
#define NET_TRACE_RECEIVED_ARP_PACKET(_packet_p) \
	net_trace_received_arp_packet(_packet_p)
#else
#define NET_TRACE_RECEIVED_ARP_PACKET(_packet_p)
#endif

/**
 * Ethernet link speed: 100 Mbps
 */
#define ETHERNET_LINK_SPEED_IN_BPS   UINT32_C(100000000)

static fdc_error_t net_receive_thread_f(void *arg);

static fdc_error_t net_icmpv4_receive_thread_f(void *arg);

static fdc_error_t net_dhcpv4_client_thread_f(void *arg);

static const struct ethernet_mac_address enet_broadcast_mac_addr = {
    .bytes = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};

static const struct ethernet_mac_address enet_null_mac_addr = {
    .bytes = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

/**
 * IPv4 loopback address: 127.0.0.1
 */
static const struct ipv4_address ipv4_loopback_address = {
    .bytes = { [0] = 127, [3] = 1 }
};

/**
 * IPv6 loopback address: ::1
 */
static const struct ipv6_address ipv6_loopback_address = {
    .bytes = { [15] = 0x1 }
};

/**
 * IPv6 unspecified address: ::
 */
static const struct ipv6_address ipv6_unspecified_address = {
    .bytes = { 0x0 }
};

/**
 * IPv6 link-local unicast address prefix: fe80::/10
 */
static const struct ipv6_address ipv6_link_local_unicast_prefix = {
    .bytes = { [0] = 0xfe, [1] = 0x80 }
};

/**
 * IPv6 site-local unicast address prefix: fec0::/10
 */
static const struct ipv6_address ipv6_site_local_unicast_prefix = {
    .bytes = { [0] = 0xfe, [1] = 0xc0 }
};

/**
 * IPv6 link-local solicited-node multicast address prefix:
 * ff02:0:0:0:0:1:ff00::/104
 */
static const struct ipv6_address ipv6_link_local_solicited_node_prefix = {
    .bytes = { [0] = 0xff, [1] = 0x02, [11] = 0x1, [12] = 0xff }
};

/**
 * IPv6 link-local all-nodes multicast address: ff02::1
 */
static const struct ipv6_address ipv6_link_local_all_nodes_address = {
    .bytes = { [0] = 0xff, [1] = 0x02, [15] = 0x1 }
};

/**
 * IPv6 link-local all-routers multicast address: ff02::2
 */
static const struct ipv6_address ipv6_link_local_all_routers_address = {
    .bytes = { [0] = 0xff, [1] = 0x02, [15] = 0x2 }
};

/**
 * Execution stacks for networking stack threads
 */
static struct rtos_thread_execution_stack g_thread_execution_stacks[NET_NUM_THREADS];

/**
 * Networking stack module global variables
 */
static struct networking g_networking = {{
    .initialized = false,
    .next_udp_ephemeral_port = NET_FIRST_EPHEMERAL_PORT,
    .next_tcp_ephemeral_port = NET_FIRST_EPHEMERAL_PORT,
    .next_free_l4_end_point_p = &g_networking.local_l4_end_points[0],
    .local_l3_end_point = {
        .signature = LOCAL_L3_END_POINT_SIGNATURE,
        .enet_device_p = NULL,
        .ipv4 = {
            .local_ip_addr.value = IPV4_NULL_ADDR,
            .subnet_mask = 0x0,
            .default_gateway_ip_addr.value = IPV4_NULL_ADDR,
            .next_tx_ip_packet_seq_num = 0,
        },
        .ipv6 = {
            .link_local_ip_addr = {
                .state = IPV6_ADDR_NOT_CONFIGURED,
            }
        }
    },
}};


static const struct rtos_thread_creation_params g_thread_creation_params[] = {
    [0] = {
        .p_name_p = "Network Packet Receive thread",
        .p_function_p = net_receive_thread_f,
        .p_function_arg_p = &g_networking.local_l3_end_point,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
    },

    [1] = {
        .p_name_p = "ICMPv4 packet receive thread",
        .p_function_p = net_icmpv4_receive_thread_f,
        .p_function_arg_p = &g_networking.local_l3_end_point,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
    },

    [2] = {
        .p_name_p = "DHCPv4 client thread",
        .p_function_p = net_dhcpv4_client_thread_f,
        .p_function_arg_p = &g_networking.local_l3_end_point,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
    },
};

C_ASSERT(ARRAY_SIZE(g_thread_creation_params) == NET_NUM_THREADS);

static void
arp_cache_init(struct arp_cache *arp_cache_p)
{
    rtos_mutex_init(
	"ARP cache mutex",
	&arp_cache_p->mutex);

    rtos_condvar_init(
	"ARP cache updated condvar",
	&arp_cache_p->cache_updated_condvar);


    for (unsigned int i = 0; i < ARP_CACHE_NUM_ENTRIES; i++) {
	struct arp_cache_entry *entry_p = &arp_cache_p->entries[i];

	entry_p->state = ARP_ENTRY_INVALID;
    }
}


/**
 * Send an ARP request message
 */
static void
net_send_arp_request(const struct enet_device *enet_device_p,
		     const struct ipv4_address *source_ip_addr_p,
		     const struct ipv4_address *dest_ip_addr_p)
{
    struct network_packet *tx_packet_p = net_allocate_tx_packet(true);

    DBG_ASSERT(tx_packet_p != NULL, enet_device_p, 0);

    struct ethernet_frame *enet_frame =
	(struct ethernet_frame *)tx_packet_p->data_buffer;

    struct ethernet_mac_address local_mac_address;

    enet_get_mac_addr(enet_device_p, &local_mac_address);

#if 0 // Hardware does this
    COPY_MAC_ADDRESS(&enet_frame->enet_header.source_mac_addr,
		     &local_mac_address);
#endif

    COPY_MAC_ADDRESS(&enet_frame->enet_header.dest_mac_addr,
		     &enet_broadcast_mac_addr);
    enet_frame->enet_header.frame_type = hton16(ENET_ARP_PACKET);
    enet_frame->arp_packet.link_addr_type = hton16(0x1);
    enet_frame->arp_packet.network_addr_type = hton16(ENET_IPv4_PACKET);
    enet_frame->arp_packet.link_addr_size = sizeof(struct ethernet_mac_address);
    enet_frame->arp_packet.network_addr_size = sizeof(struct ipv4_address);
    enet_frame->arp_packet.operation = hton16(ARP_REQUEST);

    COPY_MAC_ADDRESS(&enet_frame->arp_packet.source_mac_addr, &local_mac_address);
    COPY_UNALIGNED_IPv4_ADDRESS(&enet_frame->arp_packet.source_ip_addr,
				source_ip_addr_p);
    COPY_MAC_ADDRESS(&enet_frame->arp_packet.dest_mac_addr,
		     &enet_null_mac_addr);
    COPY_UNALIGNED_IPv4_ADDRESS(&enet_frame->arp_packet.dest_ip_addr,
		                dest_ip_addr_p);

    tx_packet_p->total_length =
	sizeof(struct ethernet_header) + sizeof(struct arp_packet);

    enet_start_xmit(enet_device_p, tx_packet_p);
}

/**
 * Send an ARP reply message
 */
static void
net_send_arp_reply(const struct enet_device *enet_device_p,
		   const struct ipv4_address *source_ip_addr_p,
		   const struct ethernet_mac_address *dest_mac_addr_p,
		   const struct ipv4_address *dest_ip_addr_p)
{
    struct network_packet *tx_packet_p = net_allocate_tx_packet(true);

    DBG_ASSERT(tx_packet_p != NULL, enet_device_p, 0);

    struct ethernet_frame *enet_frame =
	(struct ethernet_frame *)tx_packet_p->data_buffer;

    struct ethernet_mac_address local_mac_address;

    enet_get_mac_addr(enet_device_p, &local_mac_address);

#if 0 // Hardware does this
    COPY_MAC_ADDRESS(&enet_frame->enet_header.source_mac_addr,
		     &local_mac_address);
#endif

    COPY_MAC_ADDRESS(&enet_frame->enet_header.dest_mac_addr,
		     dest_mac_addr_p);
    enet_frame->enet_header.frame_type = hton16(ENET_ARP_PACKET);
    enet_frame->arp_packet.link_addr_type = hton16(0x1);
    enet_frame->arp_packet.network_addr_type = hton16(ENET_IPv4_PACKET);
    enet_frame->arp_packet.link_addr_size = sizeof(struct ethernet_mac_address);
    enet_frame->arp_packet.network_addr_size = sizeof(struct ipv4_address);
    enet_frame->arp_packet.operation = hton16(ARP_REPLY);
    COPY_MAC_ADDRESS(&enet_frame->arp_packet.source_mac_addr,
		     &local_mac_address);
    COPY_UNALIGNED_IPv4_ADDRESS(&enet_frame->arp_packet.source_ip_addr,
				source_ip_addr_p);
    COPY_MAC_ADDRESS(&enet_frame->arp_packet.dest_mac_addr,
		     dest_mac_addr_p);
    COPY_UNALIGNED_IPv4_ADDRESS(&enet_frame->arp_packet.dest_ip_addr,
		                dest_ip_addr_p);

    tx_packet_p->total_length =
	sizeof(struct ethernet_header) + sizeof(struct arp_packet);

    enet_start_xmit(enet_device_p, tx_packet_p);
}


static void
net_tx_packet_pool_init(void)
{
    rtos_queue_init(
	"Networking free Tx packet pool",
	false,
        &g_networking.free_tx_packet_pool);

    for (unsigned int i = 0; i < ARRAY_SIZE(g_networking.tx_packets); i ++) {
	struct network_packet *tx_packet_p = &g_networking.tx_packets[i];

	tx_packet_p->signature = NET_TX_PACKET_SIGNATURE;
	tx_packet_p->state_flags = NET_PACKET_IN_TX_POOL;
	tx_packet_p->tx_buf_desc_p = NULL;
	tx_packet_p->local_l3_end_point_p = NULL;
	GLIST_NODE_INIT(&tx_packet_p->node);

        rtos_queue_add(&g_networking.free_tx_packet_pool,
		       &tx_packet_p->node);
    }
}


static void
net_rx_packet_queue_init(struct local_l3_end_point *local_l3_end_point_p)
{
    rtos_queue_init(
	"Networking Rx packet queue",
	false,
	&local_l3_end_point_p->rx_packet_queue);

    for (unsigned int i = 0; i < ARRAY_SIZE(local_l3_end_point_p->rx_packets); i ++) {
	struct network_packet *rx_packet_p = &local_l3_end_point_p->rx_packets[i];

	rx_packet_p->signature = NET_RX_PACKET_SIGNATURE;
	rx_packet_p->state_flags = 0;
	rx_packet_p->rx_buf_desc_p = NULL;
	rx_packet_p->local_l3_end_point_p = local_l3_end_point_p;
	GLIST_NODE_INIT(&rx_packet_p->node);
    }
}


static void
net_send_ipv4_dhcp_discovery(struct local_l3_end_point *local_l3_end_point_p,
                             struct local_l4_end_point *client_end_point_p)
{
    fdc_error_t fdc_error;
    struct network_packet *tx_packet_p = net_allocate_tx_packet(true);

    struct dhcp_message *dhcp_discovery_msg_p =
	net_get_udp_data_payload_area(tx_packet_p);

    struct ethernet_mac_address local_mac_address;

    enet_get_mac_addr(local_l3_end_point_p->enet_device_p, &local_mac_address);

    dhcp_discovery_msg_p->op = 0x1;
    dhcp_discovery_msg_p->hardware_type = 0x1; /* Ethernet */
    dhcp_discovery_msg_p->hw_addr_len = 0x6;
    dhcp_discovery_msg_p->hops = 0x0;
    dhcp_discovery_msg_p->transaction_id = rtos_get_ticks();
    dhcp_discovery_msg_p->seconds = 0x0;
    dhcp_discovery_msg_p->flags = 0x0;
    dhcp_discovery_msg_p->client_ip_addr.value = IPV4_NULL_ADDR;
    dhcp_discovery_msg_p->your_ip_addr.value = IPV4_NULL_ADDR;
    dhcp_discovery_msg_p->next_server_ip_addr.value = IPV4_NULL_ADDR;
    dhcp_discovery_msg_p->relay_agent_ip_addr.value = IPV4_NULL_ADDR;
    COPY_MAC_ADDRESS(&dhcp_discovery_msg_p->client_mac_addr,
		     &local_mac_address);
    
    bzero(dhcp_discovery_msg_p->zero_filled,
	  sizeof dhcp_discovery_msg_p->zero_filled);

    dhcp_discovery_msg_p->magic_cookie = 0x63825363; /* DHCP */
    dhcp_discovery_msg_p->options[0] = 53; /* option 1 type: DHCP message type */
    dhcp_discovery_msg_p->options[1] = 1; /* option length */
    dhcp_discovery_msg_p->options[2] = 0x01; /* option value: DHCPDISCOVER */

    dhcp_discovery_msg_p->options[3] = 55; /* option 2 type: parameter request list */
    dhcp_discovery_msg_p->options[4] = 11; /* option length */
    dhcp_discovery_msg_p->options[5] = 0x01; /* option value[0]: subnet mask */
    dhcp_discovery_msg_p->options[6] = 0x1c; /* option value[1]: broadcast address */
    dhcp_discovery_msg_p->options[7] = 0x02; /* option value[2]: time offset */
    dhcp_discovery_msg_p->options[8] = 0x03; /* option value[3]: router */
    dhcp_discovery_msg_p->options[9] = 0x0f; /* option value[4]: domain name */
    dhcp_discovery_msg_p->options[10] = 0x06; /* option value[5]: domain name server */
    dhcp_discovery_msg_p->options[11] = 0x77; /* option value[6]: domain search */
    dhcp_discovery_msg_p->options[12] = 0x0c; /* option value[7]: host name */
    dhcp_discovery_msg_p->options[13] = 0x1a; /* option value[8]: interface MTU */
    dhcp_discovery_msg_p->options[14] = 0x79; /* option value[9]: classless static route */
    dhcp_discovery_msg_p->options[15] = 0x2a; /* option value[10]: network time protocol servers */

    struct ipv4_address dest_ip_addr = { .value = IPV4_BROADCAST_ADDR };

    net_send_ipv4_udp_datagram(client_end_point_p, &dest_ip_addr,
			       DHCP_UDP_SERVER_PORT,
			       tx_packet_p,
			       sizeof(struct dhcp_message) + 16);

    DEBUG_PRINTF("DHCP client sent discovery message\n");
}


static void
build_ipv6_interface_id(struct local_l3_end_point *local_l3_end_point_p)
{
    struct ethernet_mac_address local_mac_address;

    enet_get_mac_addr(local_l3_end_point_p->enet_device_p, &local_mac_address);

    /*
     * Build modified EUI-64 id from the local MAC address:
     * (Bit 1 of first byte is inverted)
     */
    uint8_t *interface_id_p = (uint8_t *)&local_l3_end_point_p->ipv6.interface_id;

    interface_id_p[0] = local_mac_address.bytes[0] ^ BIT(1);
    interface_id_p[1] = local_mac_address.bytes[1];
    interface_id_p[2] = local_mac_address.bytes[2];
    interface_id_p[3] = 0xff;
    interface_id_p[4] = 0xfe;
    interface_id_p[5] = local_mac_address.bytes[3];
    interface_id_p[6] = local_mac_address.bytes[4];
    interface_id_p[7] = local_mac_address.bytes[5];
   
    DEBUG_PRINTF("IPv6 interface Id: %x:%x:%x:%x:%x:%x:%x:%x\n",
                 interface_id_p[0],
                 interface_id_p[1],
                 interface_id_p[2],
                 interface_id_p[3],
                 interface_id_p[4],
                 interface_id_p[5],
                 interface_id_p[6],
                 interface_id_p[7]);
}


/**
 * Maps multicast IPv6 address to multicast Ethernet MAC address
 */
static inline void
map_ipv6_multicast_addr_to_ethernet_multicast_addr(
    const struct ipv6_address *ipv6_multicast_addr_p,
    struct ethernet_mac_address *enet_multicast_addr_p)
{
    FDC_ASSERT(ipv6_multicast_addr_p->bytes[0] == 0xff,
               ipv6_multicast_addr_p->bytes[0], 0);

    enet_multicast_addr_p->hwords[0] = 0x3333;
    enet_multicast_addr_p->hwords[1] = ipv6_multicast_addr_p->hwords[6];
    enet_multicast_addr_p->hwords[2] = ipv6_multicast_addr_p->hwords[7];
}


static void
join_ipv6_multicast_group(struct local_l3_end_point *local_l3_end_point_p,
                          const struct ipv6_address *multicast_addr_p)
{
    struct ethernet_mac_address enet_multicast_addr;

    map_ipv6_multicast_addr_to_ethernet_multicast_addr(multicast_addr_p,
                                                       &enet_multicast_addr);

    enet_add_multicast_mac_addr(local_l3_end_point_p->enet_device_p,
                                &enet_multicast_addr);
}


/**
 * Build solicited-node multicast address for a given unicast address
 */ 
static inline void
build_ipv6_solicited_node_mulicast_addr(
        const struct ipv6_address *unicast_addr_p,
        struct ipv6_address *solicited_node_multicast_addr_p)
{
    *solicited_node_multicast_addr_p = ipv6_link_local_solicited_node_prefix;
    solicited_node_multicast_addr_p->bytes[13] = unicast_addr_p->bytes[13];
    solicited_node_multicast_addr_p->bytes[14] = unicast_addr_p->bytes[14];
    solicited_node_multicast_addr_p->bytes[15] = unicast_addr_p->bytes[15];
}


/**
 * Send an ARP request message
 */
static void
net_send_neighbor_solicitation(struct local_l3_end_point *local_l3_end_point_p,
                               const struct ipv6_address *source_ip_addr_p,
		               const struct ipv6_address *dest_ip_addr_p)
{
    size_t data_payload_length;
    struct network_packet *tx_packet_p = net_allocate_tx_packet(true);

    DBG_ASSERT(tx_packet_p != NULL, dest_ip_addr_p, 0);
    struct icmpv6_neighbor_solicitation *ns_msg_p =
	(struct icmpv6_neighbor_solicitation *)GET_IPV6_DATA_PAYLOAD_AREA(tx_packet_p);

    ns_msg_p->target_ip_addr = *dest_ip_addr_p;
    if (IPV6_ADDRESSES_EQUAL(source_ip_addr_p, &ipv6_unspecified_address)) {
        /*
         * Sender is performing DAD as part of local IPv6 address configuration,
         * so source layer-2 address is not to be included in the message
         */
        data_payload_length = sizeof(struct icmpv6_neighbor_solicitation) -
			      sizeof(struct icmpv6_header);
    } else {
        enet_get_mac_addr(local_l3_end_point_p->enet_device_p, 
                          (struct ethernet_mac_address *)ns_msg_p->options);

       data_payload_length = (sizeof(struct icmpv6_neighbor_solicitation) -
			      sizeof(struct icmpv6_header)) +
                             sizeof(struct ethernet_mac_address);
    }

    struct ipv6_address solicited_node_multicast_addr;

    /*
     * Build solicited-node multicast address for the destination IPv6 address:
     */
    build_ipv6_solicited_node_mulicast_addr(
        dest_ip_addr_p,
        &solicited_node_multicast_addr);

    (void)net_send_ipv6_icmp_message(&solicited_node_multicast_addr,
		                     tx_packet_p,
				     ICMPV6_TYPE_NEIGHBOR_SOLICITATION,
				     0, /* code */
                                     0, /* data */
                                     data_payload_length);
}


static void 
autoconfigure_ipv6_link_local_addr(struct local_l3_end_point *local_l3_end_point_p)
{
    FDC_ASSERT(local_l3_end_point_p->ipv6.link_local_ip_addr.state ==
               IPV6_ADDR_NOT_CONFIGURED,
               local_l3_end_point_p->ipv6.link_local_ip_addr.state,
               local_l3_end_point_p);

    /*
     * Build link-local unicast address:
     */ 
    local_l3_end_point_p->ipv6.link_local_ip_addr.addr = 
        ipv6_link_local_unicast_prefix;
    local_l3_end_point_p->ipv6.link_local_ip_addr.addr.dwords[1] =
        local_l3_end_point_p->ipv6.interface_id;

    local_l3_end_point_p->ipv6.link_local_ip_addr.state = IPV6_ADDR_TENTATIVE;

    /*
     * Join solicited-node multicast group for the configured unicast address:
     */
    struct ipv6_address solicited_node_multicast_addr;

    build_ipv6_solicited_node_mulicast_addr(
        &local_l3_end_point_p->ipv6.link_local_ip_addr.addr,
        &solicited_node_multicast_addr);

    join_ipv6_multicast_group(local_l3_end_point_p,
                              &solicited_node_multicast_addr);

    /*
     * Start Duplicate Address Detection (DAD) by sending neighbor-solicited message
     * for configured local IPv6 address:
     */
    net_send_neighbor_solicitation(local_l3_end_point_p,
                                   &ipv6_unspecified_address,
		                   &local_l3_end_point_p->ipv6.link_local_ip_addr.addr);
}


static void 
net_start_ip6_address_autoconfiguration(struct local_l3_end_point *local_l3_end_point_p)
{
    build_ipv6_interface_id(local_l3_end_point_p);

    /*
     * Join link-local all-nodes multicast group:
     */
    join_ipv6_multicast_group(local_l3_end_point_p, &ipv6_link_local_all_nodes_address);

    autoconfigure_ipv6_link_local_addr(local_l3_end_point_p);
}

static void
net_send_ipv4_dhcp_request(struct local_l3_end_point *local_l3_end_point_p,
                           struct local_l4_end_point *client_end_point_p,
                           struct dhcp_message *dhcp_offer_msg_p)
{
    fdc_error_t fdc_error;
    struct network_packet *tx_packet_p = net_allocate_tx_packet(true);

    struct dhcp_message *dhcp_request_msg_p =
	net_get_udp_data_payload_area(tx_packet_p);

    *dhcp_request_msg_p = *dhcp_offer_msg_p;
    dhcp_request_msg_p->op = 0x1;
    dhcp_request_msg_p->options[0] = 53; /* option 1 type: DHCP message type */
    dhcp_request_msg_p->options[1] = 1; /* option length */
    dhcp_request_msg_p->options[2] = 0x03; /* option value: DHCPREQUEST */

    dhcp_request_msg_p->options[3] = 54; /* option 2 type: DHCP server identifier */
    dhcp_request_msg_p->options[4] = 4; /* option length */
    dhcp_request_msg_p->options[5] = dhcp_offer_msg_p->next_server_ip_addr.bytes[0]; /* option value[0] */
    dhcp_request_msg_p->options[6] = dhcp_offer_msg_p->next_server_ip_addr.bytes[1]; /* option value[1] */
    dhcp_request_msg_p->options[7] = dhcp_offer_msg_p->next_server_ip_addr.bytes[2]; /* option value[2] */
    dhcp_request_msg_p->options[8] = dhcp_offer_msg_p->next_server_ip_addr.bytes[3]; /* option value[3] */

    dhcp_request_msg_p->options[9] = 50; /* option 3 type: Requested IP address */
    dhcp_request_msg_p->options[10] = 4; /* option length */
    dhcp_request_msg_p->options[11] = dhcp_offer_msg_p->your_ip_addr.bytes[0]; /* option value[0] */
    dhcp_request_msg_p->options[12] = dhcp_offer_msg_p->your_ip_addr.bytes[1]; /* option value[1] */
    dhcp_request_msg_p->options[13] = dhcp_offer_msg_p->your_ip_addr.bytes[2]; /* option value[2] */
    dhcp_request_msg_p->options[14] = dhcp_offer_msg_p->your_ip_addr.bytes[3]; /* option value[3] */

    dhcp_request_msg_p->options[15] = 55; /* option 2 type: parameter request list */
    dhcp_request_msg_p->options[16] = 11; /* option length */
    dhcp_request_msg_p->options[17] = 0x01; /* option value[0]: subnet mask */
    dhcp_request_msg_p->options[18] = 0x1c; /* option value[1]: broadcast address */
    dhcp_request_msg_p->options[19] = 0x02; /* option value[2]: time offset */
    dhcp_request_msg_p->options[20] = 0x03; /* option value[3]: router */
    dhcp_request_msg_p->options[21] = 0x0f; /* option value[4]: domain name */
    dhcp_request_msg_p->options[22] = 0x06; /* option value[5]: domain name server */
    dhcp_request_msg_p->options[23] = 0x77; /* option value[6]: domain search */
    dhcp_request_msg_p->options[24] = 0x0c; /* option value[7]: host name */
    dhcp_request_msg_p->options[25] = 0x1a; /* option value[8]: interface MTU */
    dhcp_request_msg_p->options[26] = 0x79; /* option value[9]: classless static route */
    dhcp_request_msg_p->options[27] = 0x2a; /* option value[10]: network time protocol servers */

    struct ipv4_address dest_ip_addr = { .value = IPV4_BROADCAST_ADDR };

    net_send_ipv4_udp_datagram(client_end_point_p, &dest_ip_addr,
			       DHCP_UDP_SERVER_PORT,
			       tx_packet_p,
			       sizeof(struct dhcp_message) + 28);

    DEBUG_PRINTF("DHCP client sent request message\n");
}


static void
neighbor_cache_init(struct neighbor_cache *neighbor_cache_p)
{
    rtos_mutex_init(
	"IPv6 neighbor cache mutex",
	&neighbor_cache_p->mutex);

    rtos_condvar_init(
	"IPv6 neighbor cache updated condvar",
	&neighbor_cache_p->cache_updated_condvar);


    for (unsigned int i = 0; i < NEIGHBOR_CACHE_NUM_ENTRIES; i++) {
	struct neighbor_cache_entry *entry_p = &neighbor_cache_p->entries[i];

	entry_p->state = NEIGHBOR_ENTRY_INVALID;
    }
}


/**
 * Initialize networking subsystem
 */
void
networking_init(const struct enet_device *enet_device_p)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                sizeof g_networking,
                                                false);
    FDC_ASSERT(fdc_error == 0, fdc_error, cpu_id);

    FDC_ASSERT(!g_networking.initialized, 0, 0);

    /*
     * Initialize global Tx buffer pool:
     */
    net_tx_packet_pool_init();

    /*
     * Initialize local layer-3 end point:
     */
    struct local_l3_end_point *local_l3_end_point_p =
	    &g_networking.local_l3_end_point;

    local_l3_end_point_p->enet_device_p = enet_device_p;
    net_rx_packet_queue_init(local_l3_end_point_p);

    rtos_queue_init(
        "ICMPv4 incoming packet queue",
        true,
        &local_l3_end_point_p->ipv4.rx_icmpv4_packet_queue);

    rtos_queue_init(
        "ICMPv6 incoming packet queue",
        true,
        &local_l3_end_point_p->ipv6.rx_icmpv6_packet_queue);

    /*
     * Initialize IPv4 ARP cache for the layer-3 end point
     */
    arp_cache_init(&local_l3_end_point_p->ipv4.arp_cache);

    /*
     * Initialize IPv6 neighbor cache for the layer-3 end point
     */
    neighbor_cache_init(&local_l3_end_point_p->ipv6.neighbor_cache);

    /*
     * Enable access to Rx/Tx buffers memory for the ENET DMA engine:
     */
    enet_register_dma_region(&g_networking, sizeof g_networking);

    /*
     * Activate network interface (ENET device):
     */
    enet_start(local_l3_end_point_p->enet_device_p, local_l3_end_point_p);

    rtos_queue_init(
	"Incoming IPv4 ping reply packet queue",
	true,
	&g_networking.rx_ipv4_ping_reply_packet_queue);

    rtos_mutex_init("local_l4_end_points mutex", &g_networking.local_l4_end_points_mutex);
    rtos_mutex_init("expecting_ping_reply mutex", &g_networking.expecting_ping_reply_mutex);
    rtos_condvar_init("ping_reply_received condvar", &g_networking.ping_reply_recceived_condvar);
    g_networking.initialized = true;

    /*
     * Create threads for this layer-3 end point:
     */
    for (unsigned int i = 0; i < NET_NUM_THREADS; i ++) {
        rtos_thread_init(
            &g_thread_creation_params[i],
            &g_thread_execution_stacks[i],
            &g_networking.threads[i]);

        console_printf("CPU core %u: %s started\n", cpu_id,
            g_thread_creation_params[i].p_name_p);
    }

    /*
     * Start IPv6 address autoconfiguration:
     */
    net_start_ip6_address_autoconfiguration(local_l3_end_point_p);

    rtos_mpu_remove_thread_data_region();   /* g_networking */
}


/**
 * Set IPv4 address for the local layer-3 end point
 */
void
net_set_local_ipv4_address(const struct ipv4_address *ip_addr_p,
			   uint8_t subnet_prefix)
{
    fdc_error_t fdc_error;

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                sizeof g_networking,
                                                false);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    FDC_ASSERT(g_networking.initialized, 0, 0);
    FDC_ASSERT(ip_addr_p->value != IPV4_NULL_ADDR, ip_addr_p->value, 0);
    FDC_ASSERT(subnet_prefix < 32, subnet_prefix, 0);

    struct local_l3_end_point *local_l3_end_point_p =
	 &g_networking.local_l3_end_point;

    local_l3_end_point_p->ipv4.local_ip_addr = *ip_addr_p;
    local_l3_end_point_p->ipv4.subnet_mask = IPv4_SUBNET_MASK(subnet_prefix);

    capture_fdc_msg_printf("Set local IPv4 address to %u.%u.%u.%u/%u\n",
                           ip_addr_p->bytes[0],
                           ip_addr_p->bytes[1],
                           ip_addr_p->bytes[2],
                           ip_addr_p->bytes[3],
                           subnet_prefix);

    /*
     * Send gratuitous ARP request (to catch if someone else is using the same
     * IP address):
     */
    net_send_arp_request(local_l3_end_point_p->enet_device_p,
			 &local_l3_end_point_p->ipv4.local_ip_addr,
			 &local_l3_end_point_p->ipv4.local_ip_addr);

    rtos_mpu_remove_thread_data_region();   /* g_networking */
}


void
net_get_local_ipv4_address(struct ipv4_address *ip_addr_p)
{
    fdc_error_t fdc_error;

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                sizeof g_networking,
                                                false);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    FDC_ASSERT(g_networking.initialized, 0, 0);

    struct local_l3_end_point *local_l3_end_point_p =
	 &g_networking.local_l3_end_point;

    *ip_addr_p = local_l3_end_point_p->ipv4.local_ip_addr;
    rtos_mpu_remove_thread_data_region();   /* g_networking */
}


/**
 * Allocates a Tx packet from the global Tx packet pool.
 * If there are no free Tx packets, it waits until one becomes available
 */
struct network_packet *
net_allocate_tx_packet(bool free_after_tx_complete)
{
    fdc_error_t fdc_error;
    struct network_packet *tx_packet_p = NULL;

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                sizeof g_networking,
                                                false);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    FDC_ASSERT(g_networking.initialized, 0, 0);

    tx_packet_p = GLIST_NODE_TO_NETWORK_PACKET(
		     rtos_queue_remove(&g_networking.free_tx_packet_pool, 0));

    FDC_ASSERT(tx_packet_p->signature == NET_TX_PACKET_SIGNATURE,
	       tx_packet_p->signature, tx_packet_p);
    FDC_ASSERT(tx_packet_p->state_flags == NET_PACKET_IN_TX_POOL,
	       tx_packet_p->state_flags, tx_packet_p);
    FDC_ASSERT(GLIST_NODE_IS_UNLINKED(&tx_packet_p->node),
	       &tx_packet_p->node, tx_packet_p);
    FDC_ASSERT(tx_packet_p->tx_buf_desc_p == NULL,
	       tx_packet_p->tx_buf_desc_p, tx_packet_p);

    tx_packet_p->state_flags = NET_PACKET_IN_TX_USE_BY_APP;
    if (free_after_tx_complete) {
	tx_packet_p->state_flags |= NET_PACKET_FREE_AFTER_TX_COMPLETE;
    }

    rtos_mpu_remove_thread_data_region(); /* g_networking */
    return tx_packet_p;
}


/**
 * Frees a Tx packet back to the global Tx packet pool
 */
void
net_free_tx_packet(struct network_packet *tx_packet_p)
{
    fdc_error_t fdc_error;
    bool region_added = false;
   
    if (rtos_caller_is_thread()) {
        fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                    sizeof g_networking,
                                                    false);
        FDC_ASSERT(fdc_error == 0, fdc_error, 0);
        region_added = true;
    }

    FDC_ASSERT(g_networking.initialized, 0, 0);

    FDC_ASSERT(tx_packet_p->signature == NET_TX_PACKET_SIGNATURE,
	       tx_packet_p->signature, tx_packet_p);
    FDC_ASSERT(tx_packet_p->state_flags == NET_PACKET_IN_TX_USE_BY_APP,
	       tx_packet_p->state_flags, tx_packet_p);
    FDC_ASSERT(GLIST_NODE_IS_UNLINKED(&tx_packet_p->node),
	       &tx_packet_p->node, tx_packet_p);
    FDC_ASSERT(tx_packet_p->tx_buf_desc_p == NULL,
	       tx_packet_p->tx_buf_desc_p, tx_packet_p);

    tx_packet_p->state_flags = NET_PACKET_IN_TX_POOL;
    rtos_queue_add(&g_networking.free_tx_packet_pool, &tx_packet_p->node);

    if (region_added) {
        rtos_mpu_remove_thread_data_region(); /* g_networking */
    }
}


/**
 * Dequeues an Rx packet from the Rx packet queue of the given layer-3 local end point
 */
void
net_dequeue_rx_packet(struct local_l3_end_point *local_l3_end_point_p,
		       struct network_packet **rx_packet_pp)
{
    struct network_packet *rx_packet_p = NULL;

    FDC_ASSERT(
        local_l3_end_point_p->signature == LOCAL_L3_END_POINT_SIGNATURE,
        local_l3_end_point_p->signature, local_l3_end_point_p);

    rx_packet_p = GLIST_NODE_TO_NETWORK_PACKET(
		    rtos_queue_remove(&local_l3_end_point_p->rx_packet_queue, 0));

    FDC_ASSERT(rx_packet_p->signature == NET_RX_PACKET_SIGNATURE,
	       rx_packet_p->signature, rx_packet_p);
    FDC_ASSERT(rx_packet_p->state_flags & NET_PACKET_IN_RX_QUEUE,
	       rx_packet_p->state_flags, rx_packet_p);

    FDC_ASSERT(rx_packet_p->rx_buf_desc_p == NULL,
               rx_packet_p->rx_buf_desc_p, rx_packet_p);

    rx_packet_p->state_flags &= ~NET_PACKET_IN_RX_QUEUE;
    rx_packet_p->state_flags |= NET_PACKET_IN_RX_USE_BY_APP;
    *rx_packet_pp = rx_packet_p;
}


/**
 * Enqueue a Rx packet into the Rx packet queue of the given layer-3 local end point
 */
void
net_enqueue_rx_packet(struct local_l3_end_point *local_l3_end_point_p,
		      struct network_packet *rx_packet_p)
{
    FDC_ASSERT(
        local_l3_end_point_p->signature == LOCAL_L3_END_POINT_SIGNATURE,
        local_l3_end_point_p->signature, local_l3_end_point_p);

    FDC_ASSERT(rx_packet_p->signature == NET_RX_PACKET_SIGNATURE,
	       rx_packet_p->signature, rx_packet_p);
    FDC_ASSERT(rx_packet_p->state_flags == 0,
	       rx_packet_p->state_flags, rx_packet_p);

    FDC_ASSERT(rx_packet_p->rx_buf_desc_p == NULL,
	       rx_packet_p->rx_buf_desc_p, rx_packet_p);

    rx_packet_p->state_flags |= NET_PACKET_IN_RX_QUEUE;
    rtos_queue_add(&local_l3_end_point_p->rx_packet_queue, &rx_packet_p->node);
}


/**
 * Recycle a Rx packet for receiving another packet
 */
void
net_recycle_rx_packet(struct network_packet *rx_packet_p)
{
    fdc_error_t fdc_error;
    
    fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                sizeof g_networking,
                                                false);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    struct local_l3_end_point *local_l3_end_point_p = rx_packet_p->local_l3_end_point_p;

    FDC_ASSERT(
        local_l3_end_point_p->signature == LOCAL_L3_END_POINT_SIGNATURE,
        local_l3_end_point_p->signature, local_l3_end_point_p);

    FDC_ASSERT(rx_packet_p->signature == NET_RX_PACKET_SIGNATURE,
	       rx_packet_p->signature, rx_packet_p);
    FDC_ASSERT(rx_packet_p->state_flags == NET_PACKET_IN_RX_USE_BY_APP,
	       rx_packet_p->state_flags, rx_packet_p);

    FDC_ASSERT(rx_packet_p->rx_buf_desc_p == NULL,
	       rx_packet_p->rx_buf_desc_p, rx_packet_p);

    enet_repost_rx_packet(local_l3_end_point_p->enet_device_p, rx_packet_p);
    rtos_mpu_remove_thread_data_region(); /* g_networking */
}


static struct arp_cache_entry *
arp_cache_lookup_or_allocate(struct arp_cache *arp_cache_p,
			     const struct ipv4_address *dest_ip_addr_p,
			     struct arp_cache_entry **free_entry_pp)
{
    struct arp_cache_entry *first_free_entry_p = NULL;
    struct arp_cache_entry *least_recently_used_entry_p = NULL;
    rtos_ticks_t least_recently_used_ticks_delta = 0;
    struct arp_cache_entry *matching_entry_p = NULL;

    *free_entry_pp = NULL;
    for (unsigned int i = 0; i < ARP_CACHE_NUM_ENTRIES; i++) {
	struct arp_cache_entry *entry_p = &arp_cache_p->entries[i];
	rtos_ticks_t current_ticks = rtos_get_ticks();

	if (entry_p->state == ARP_ENTRY_INVALID) {
	    if (first_free_entry_p == NULL) {
		first_free_entry_p = entry_p;
	    }
	} else {
	    FDC_ASSERT(entry_p->state == ARP_ENTRY_FILLED ||
		       entry_p->state == ARP_ENTRY_HALF_FILLED,
		       entry_p->state, entry_p);

	    if (entry_p->dest_ip_addr.value == dest_ip_addr_p->value) {
		matching_entry_p = entry_p;
		break;
	    }

	    if (least_recently_used_entry_p == NULL ||
		RTOS_TICKS_DELTA(entry_p->last_lookup_time_stamp, current_ticks) >
		       least_recently_used_ticks_delta) {
		least_recently_used_entry_p = entry_p;
		least_recently_used_ticks_delta =
		    RTOS_TICKS_DELTA(entry_p->last_lookup_time_stamp,
				     current_ticks);
	    }
	}
    }

    if (matching_entry_p == NULL) {
	if (first_free_entry_p != NULL) {
	    *free_entry_pp = first_free_entry_p;
	} else {
	    /*
	     * Overwrite the least recently used entry:
	     */
	    FDC_ASSERT(least_recently_used_entry_p != NULL, 0, 0);
	    least_recently_used_entry_p->state = ARP_ENTRY_INVALID;
	    *free_entry_pp = least_recently_used_entry_p;
	}
    }

    return matching_entry_p;
}


static fdc_error_t
resolve_dest_ipv4_addr(struct local_l3_end_point *local_l3_end_point_p,
		       const struct ipv4_address *dest_ip_addr_p,
		       struct ethernet_mac_address *dest_mac_addr_p)
{
    unsigned int arp_request_retries = 0;
    struct arp_cache_entry *matching_entry_p = NULL;
    struct arp_cache_entry *free_entry_p = NULL;
    struct arp_cache *arp_cache_p = &local_l3_end_point_p->ipv4.arp_cache;
    fdc_error_t fdc_error;

    rtos_mutex_acquire(&arp_cache_p->mutex);
    for ( ; ; ) {
	bool send_arp_request = false;
	matching_entry_p = arp_cache_lookup_or_allocate(arp_cache_p, dest_ip_addr_p,
							&free_entry_p);
	rtos_ticks_t current_ticks = rtos_get_ticks();

	if (matching_entry_p == NULL) {
	    send_arp_request = true;
	} else if (matching_entry_p->state == ARP_ENTRY_FILLED) {
	    if (RTOS_TICKS_DELTA(matching_entry_p->entry_filled_time_stamp,
				 current_ticks) <
		ARP_CACHE_ENTRY_LIFETIME_IN_TICKS) {
		/*
		 * ARP cache hit
		 */
		*dest_mac_addr_p = matching_entry_p->dest_mac_addr;
		break;
	    } else {
		/*
		 * ARP entry expired, send a new ARP request:
		 */
		DEBUG_PRINTF("Expired ARP cache entry for IP address %u.%u.%u.%u\n",
			     dest_ip_addr_p->bytes[0],
			     dest_ip_addr_p->bytes[1],
			     dest_ip_addr_p->bytes[2],
			     dest_ip_addr_p->bytes[3]);

		matching_entry_p->state = ARP_ENTRY_INVALID;
		send_arp_request = true;
	    }
	} else {
	    FDC_ASSERT(matching_entry_p->state == ARP_ENTRY_HALF_FILLED,
		       matching_entry_p->state, matching_entry_p);

	    if (RTOS_TICKS_DELTA(matching_entry_p->arp_request_time_stamp, current_ticks) >=
		MILLISECONDS_TO_TICKS(ARP_REPLY_WAIT_TIMEOUT_IN_MS)) {
		/*
		 * Re-send ARP request:
		 */
		capture_fdc_msg_printf("Outstanding ARP request re-sent for IP address: %u.%u.%u.%u\n",
				       dest_ip_addr_p->bytes[0],
				       dest_ip_addr_p->bytes[1],
				       dest_ip_addr_p->bytes[2],
				       dest_ip_addr_p->bytes[3]);

		send_arp_request = true;
	    }
	}

	/*
	 * Send ARP request if necessary:
	 */
	if (send_arp_request) {
	    if (arp_request_retries == ARP_REQUEST_MAX_RETRIES) {
		fdc_error = CAPTURE_FDC_ERROR("Unreachable IP address",
					      dest_ip_addr_p->value, 0);

		capture_fdc_msg_printf("Unreachable IP address: %u.%u.%u.%u\n",
				       dest_ip_addr_p->bytes[0],
				       dest_ip_addr_p->bytes[1],
				       dest_ip_addr_p->bytes[2],
				       dest_ip_addr_p->bytes[3]);

		goto common_exit;
	    }

	    arp_request_retries ++;
	    if (matching_entry_p != NULL) {
		matching_entry_p->arp_request_time_stamp = rtos_get_ticks();
		matching_entry_p->state = ARP_ENTRY_HALF_FILLED;
	    } else {
		FDC_ASSERT(free_entry_p != NULL, dest_ip_addr_p, arp_cache_p);
		free_entry_p->dest_ip_addr.value = dest_ip_addr_p->value;
		free_entry_p->arp_request_time_stamp = rtos_get_ticks();
		free_entry_p->state = ARP_ENTRY_HALF_FILLED;
	    }

	    net_send_arp_request(local_l3_end_point_p->enet_device_p,
				 &local_l3_end_point_p->ipv4.local_ip_addr,
				 dest_ip_addr_p);
	}

	/*
	 * Wait for ARP cache update:
	 */
	rtos_milliseconds_t timeout = ARP_REPLY_WAIT_TIMEOUT_IN_MS;
	rtos_condvar_wait(&arp_cache_p->cache_updated_condvar,
			  &arp_cache_p->mutex,
			  &timeout);
    }

    fdc_error = 0;

common_exit:
    rtos_mutex_release(&arp_cache_p->mutex);
    return fdc_error;
}


static void
arp_cache_update(struct arp_cache *arp_cache_p,
		 const struct ipv4_address *dest_ip_addr_p,
		 struct ethernet_mac_address *dest_mac_addr_p)
{
    struct arp_cache_entry *chosen_entry_p = NULL;
    struct arp_cache_entry *free_entry_p = NULL;

    rtos_mutex_acquire(&arp_cache_p->mutex);
    chosen_entry_p = arp_cache_lookup_or_allocate(arp_cache_p, dest_ip_addr_p,
					          &free_entry_p);

    if (chosen_entry_p == NULL) {
	FDC_ASSERT(free_entry_p != NULL, dest_ip_addr_p, arp_cache_p);
	chosen_entry_p = free_entry_p;
	chosen_entry_p->dest_ip_addr.value = dest_ip_addr_p->value;
    }

    COPY_MAC_ADDRESS(&chosen_entry_p->dest_mac_addr, dest_mac_addr_p);
    chosen_entry_p->state = ARP_ENTRY_FILLED;
    chosen_entry_p->entry_filled_time_stamp = rtos_get_ticks();
    rtos_mutex_release(&arp_cache_p->mutex);
    rtos_condvar_signal(&arp_cache_p->cache_updated_condvar);
}


/**
 * Chooses the local network end-point to be used for sending a packet,
 * based on the destination IPv4 address
 */
static struct local_l3_end_point *
choose_ipv4_local_l3_end_point(const struct ipv4_address *dest_ip_addr_p)
{
    /*
     * There is only one local network end-point
     */
    return &g_networking.local_l3_end_point;
}


/**
 * Chooses the local network end-point to be used for sending a packet,
 * based on the destination IPv6 address
 */
static struct local_l3_end_point *
choose_ipv6_local_l3_end_point(const struct ipv6_address *dest_ip_addr_p)
{
    /*
     * There is only one local network end-point
     */
    return &g_networking.local_l3_end_point;
}


#ifndef ENET_CHECKSUM_OFFLOAD
static uint16_t
net_compute_checksum(void *data_p, size_t data_length)
{
    FDC_ASSERT((uintptr_t)data_p % sizeof(uint16_t) == 0 &&
	       data_length != 0 && data_length % sizeof(uint16_t) == 0,
	       data_p, data_length);

    uint16_t *end_data_p = (uint16_t *)((uint8_t *)data_p + data_length);
    uint32_t one_complement_sum = 0;
    uint16_t checksum = 0;

    for (uint16_t *val_p = data_p; val_p != end_data_p; val_p ++) {
	one_complement_sum += *val_p;
	if (one_complement_sum & BIT(16)) {
	    one_complement_sum = (uint16_t)one_complement_sum + 1;
	}

	FDC_ASSERT(one_complement_sum <= UINT16_MAX,
		   one_complement_sum, 0);
    }

    checksum = ~one_complement_sum;
    return checksum;
}
#endif


/**
 * Sends an IPv4 packet over Ethernet
 */
fdc_error_t
net_send_ipv4_packet(const struct ipv4_address *dest_ip_addr_p,
		     struct network_packet *tx_packet_p,
		     size_t data_payload_length,
		     enum l4_protocols l4_protocol)
{
    struct arp_cache_entry *arp_entry_p = NULL;
    struct ethernet_mac_address dest_mac_addr;
    fdc_error_t fdc_error;

    FDC_ASSERT(tx_packet_p->signature == NET_TX_PACKET_SIGNATURE,
	       tx_packet_p->signature, tx_packet_p);
    FDC_ASSERT(data_payload_length <= NET_MAX_IPV4_PACKET_PAYLOAD_SIZE,
	       data_payload_length, tx_packet_p);

    struct ethernet_frame *tx_frame_p =
       (struct ethernet_frame *)tx_packet_p->data_buffer;

    struct local_l3_end_point *local_l3_end_point_p =
	choose_ipv4_local_l3_end_point(dest_ip_addr_p);

    const struct enet_device *enet_device_p = local_l3_end_point_p->enet_device_p;

    /*
     * Populate IP header
     */
    tx_frame_p->ipv4_header.version_and_header_length = 0;
    SET_BIT_FIELD(tx_frame_p->ipv4_header.version_and_header_length,
		  IP_VERSION_MASK, IP_VERSION_SHIFT, 4);
    SET_BIT_FIELD(tx_frame_p->ipv4_header.version_and_header_length,
		  IP_HEADER_LENGTH_MASK, IP_HEADER_LENGTH_SHIFT, 5);

    tx_frame_p->ipv4_header.type_of_service = 0; /* normal service */
    tx_frame_p->ipv4_header.total_length =
	hton16(sizeof(struct ipv4_header) + data_payload_length);

    tx_frame_p->ipv4_header.identification = hton16(
	ATOMIC_POST_INCREMENT_UINT16(
	    &local_l3_end_point_p->ipv4.next_tx_ip_packet_seq_num));

    /*
     * No IP packet fragmentation is supported:
     */
    tx_frame_p->ipv4_header.flags_and_fragment_offset =
	hton16(IP_FLAG_DONT_FRAGMENT_MASK);

    tx_frame_p->ipv4_header.time_to_live = 64; /* max routing hops */
    tx_frame_p->ipv4_header.protocol_type = l4_protocol;

#   ifdef ENET_DATA_PAYLOAD_32_BIT_ALIGNED
    tx_frame_p->ipv4_header.source_ip_addr.value =
	local_l3_end_point_p->ipv4.local_ip_addr.value;

    tx_frame_p->ipv4_header.dest_ip_addr.value = dest_ip_addr_p->value;
#   else
    COPY_UNALIGNED_IPv4_ADDRESS(&tx_frame_p->ipv4_header.source_ip_addr,
				&local_l3_end_point_p->ipv4.local_ip_addr);

    COPY_UNALIGNED_IPv4_ADDRESS(&tx_frame_p->ipv4_header.dest_ip_addr,
				dest_ip_addr_p);
#   endif

    /*
     * NOTE: enet_frame->ipv4_header.header_checksum is computed by hardware.
     * We just need to initialize the checksum field to 0
     */
    tx_frame_p->ipv4_header.header_checksum = 0;

#   ifndef ENET_CHECKSUM_OFFLOAD
    tx_frame_p->ipv4_header.header_checksum =
	net_compute_checksum(&tx_frame_p->ipv4_header,
			     sizeof(struct ipv4_header));
#   endif

    /*
     * Populate Ethernet header
     */

#if 0 /* ENET hardware does this automatically */
    ENET_COPY_MAC_ADDRESS(&tx_frame_p->enet_header.source_mac_addr,
			  &enet_device_p->mac_address);
#endif

    tx_frame_p->enet_header.frame_type = hton16(ENET_IPv4_PACKET);

    tx_packet_p->total_length = sizeof(struct ethernet_header) +
				sizeof(struct ipv4_header) +
				data_payload_length;

    /*
     * Get destination MAC address:
     */
    if (local_l3_end_point_p->ipv4.local_ip_addr.value == dest_ip_addr_p->value) {
	fdc_error = CAPTURE_FDC_ERROR("IPv4 Loopback not supported", 0, 0);
    } else if (dest_ip_addr_p->value == IPV4_BROADCAST_ADDR) {
	dest_mac_addr = enet_broadcast_mac_addr;
    } else if (SAME_IPv4_SUBNET(&local_l3_end_point_p->ipv4.local_ip_addr,
		         dest_ip_addr_p,
			 local_l3_end_point_p->ipv4.subnet_mask)) {
	fdc_error = resolve_dest_ipv4_addr(local_l3_end_point_p,
				           dest_ip_addr_p,
				           &dest_mac_addr);
    } else if (local_l3_end_point_p->ipv4.default_gateway_ip_addr.value == IPV4_NULL_ADDR) {
	fdc_error = CAPTURE_FDC_ERROR("No default IPv4 gateway defined", 0, 0);
    } else {
	 fdc_error = resolve_dest_ipv4_addr(local_l3_end_point_p,
					    &local_l3_end_point_p->ipv4.default_gateway_ip_addr,
					    &dest_mac_addr);
    }

    if (fdc_error != 0) {
	return fdc_error;
    }

    COPY_MAC_ADDRESS(&tx_frame_p->enet_header.dest_mac_addr, &dest_mac_addr);

    /*
     * Transmit packet:
     */
    enet_start_xmit(enet_device_p, tx_packet_p);
    return 0;
}


fdc_error_t
net_create_local_l4_end_point(enum l4_protocols l4_protocol,
	                      uint16_t l4_port,
			      struct local_l4_end_point **local_l4_end_point_pp)
{
    fdc_error_t fdc_error;
    struct local_l4_end_point *l4_end_point_p;

    if (l4_protocol != TRANSPORT_PROTO_UDP &&
	l4_protocol != TRANSPORT_PROTO_TCP) {
        fdc_error = CAPTURE_FDC_ERROR(
                        "Invalid layer-4 protocol",
                        l4_protocol, l4_port);
	return fdc_error;
    }

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                sizeof g_networking,
                                                false);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    rtos_mutex_acquire(&g_networking.local_l4_end_points_mutex);
    if (g_networking.next_free_l4_end_point_p ==
        &g_networking.local_l4_end_points[NET_MAX_LOCAL_L4_END_POINTS]) {
        fdc_error = CAPTURE_FDC_ERROR(
                        "No more local_l4_end_points can be created",
                        l4_protocol, l4_port);
	goto error_release_mutex;
    }

    if (l4_port == 0) {
	if (l4_protocol == TRANSPORT_PROTO_UDP) {
	    if (g_networking.next_udp_ephemeral_port == 0) {
		fdc_error = CAPTURE_FDC_ERROR(
				"No more UDP ephemeral ports available",
				0, 0);
		goto error_release_mutex;
	    }

	    l4_port = hton16(g_networking.next_udp_ephemeral_port);
	    g_networking.next_udp_ephemeral_port ++;
	} else {
	    DBG_ASSERT(l4_protocol == TRANSPORT_PROTO_TCP, l4_protocol, 0);
	    if (g_networking.next_tcp_ephemeral_port == 0) {
		fdc_error = CAPTURE_FDC_ERROR(
				"No more TCP ephemeral ports available",
				0, 0);
		goto error_release_mutex;
	    }

	    l4_port = hton16(g_networking.next_tcp_ephemeral_port);
	    g_networking.next_tcp_ephemeral_port ++;
	}
    }

    l4_end_point_p = g_networking.next_free_l4_end_point_p;
    g_networking.next_free_l4_end_point_p ++;
    rtos_mutex_release(&g_networking.local_l4_end_points_mutex);

    l4_end_point_p->l4_protocol = l4_protocol;
    l4_end_point_p->l4_port = l4_port;
    rtos_queue_init("l4_end_point Rx packet queue", true,
	            &l4_end_point_p->l4_rx_packet_queue);

    *local_l4_end_point_pp = l4_end_point_p;
    rtos_mpu_remove_thread_data_region(); /* g_networking */
    return 0;

error_release_mutex:
    rtos_mutex_release(&g_networking.local_l4_end_points_mutex);
    rtos_mpu_remove_thread_data_region(); /* g_networking */
    return fdc_error;
}


fdc_error_t
net_send_ipv4_udp_datagram(struct local_l4_end_point *local_l4_end_point_p,
		           const struct ipv4_address *dest_ip_addr_p,
			   uint16_t dest_port,
		           struct network_packet *tx_packet_p,
		           size_t data_payload_length)
{
    fdc_error_t fdc_error;

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                sizeof g_networking,
                                                false);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    /*
     * Populate UPD header:
     */
    struct udp_header *udp_header_p =
	(struct udp_header *)GET_IPV4_DATA_PAYLOAD_AREA(tx_packet_p);

    FDC_ASSERT(local_l4_end_point_p->l4_protocol == TRANSPORT_PROTO_UDP,
	       local_l4_end_point_p->l4_protocol, local_l4_end_point_p);

    FDC_ASSERT(data_payload_length <= NET_MAX_UDP_PACKET_PAYLOAD_SIZE,
               data_payload_length, tx_packet_p);

    udp_header_p->source_port = local_l4_end_point_p->l4_port;
    udp_header_p->dest_port = dest_port;
    udp_header_p->datagram_length = hton16(sizeof(struct udp_header) +
	                                   data_payload_length);

    /*
     * NOTE: udp_header_p->datagram_checksum is filled by hardware
     * We just need to initialize it to 0
     */
    udp_header_p->datagram_checksum = 0;

    /*
     * Send IP packet:
     */
    fdc_error = net_send_ipv4_packet(dest_ip_addr_p,
			             tx_packet_p,
			             sizeof(struct udp_header) + data_payload_length,
			             TRANSPORT_PROTO_UDP);

    rtos_mpu_remove_thread_data_region(); /* g_networking */
    return fdc_error;
}


fdc_error_t
net_receive_ipv4_udp_datagram(struct local_l4_end_point *local_l4_end_point_p,
			      rtos_milliseconds_t timeout_ms,
		              struct ipv4_address *source_ip_addr_p,
			      uint16_t *source_port_p,
			      struct network_packet **rx_packet_pp)
{
    fdc_error_t fdc_error;

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                sizeof g_networking,
                                                false);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    FDC_ASSERT(local_l4_end_point_p->l4_protocol == TRANSPORT_PROTO_UDP,
	       local_l4_end_point_p->l4_protocol, local_l4_end_point_p);

    struct glist_node *rx_packet_node_p =
	    rtos_queue_remove(&local_l4_end_point_p->l4_rx_packet_queue,
			      timeout_ms);

    if (rx_packet_node_p == NULL) {
	*rx_packet_pp = NULL;
        fdc_error = CAPTURE_FDC_ERROR("No Rx packet available", timeout_ms, 0);
        rtos_mpu_remove_thread_data_region(); /* g_networking */
	return fdc_error;
    }

    struct network_packet *rx_packet_p =
	GLIST_NODE_TO_NETWORK_PACKET(rx_packet_node_p);

    struct ipv4_header *ipv4_header_p = GET_IPV4_HEADER(rx_packet_p);

    FDC_ASSERT(ipv4_header_p->protocol_type == TRANSPORT_PROTO_UDP,
	       ipv4_header_p->protocol_type, rx_packet_p);

    struct udp_header *udp_header_p =
	(struct udp_header *)GET_IPV4_DATA_PAYLOAD_AREA(rx_packet_p);

    FDC_ASSERT(udp_header_p->dest_port == local_l4_end_point_p->l4_port,
               udp_header_p->dest_port, local_l4_end_point_p->l4_port);

#   ifdef ENET_DATA_PAYLOAD_32_BIT_ALIGNED
    source_ip_addr_p->value = ipv4_header_p->source_ip_addr.value;
#   else
    COPY_UNALIGNED_IPv4_ADDRESS(source_ip_addr_p, &ipv4_header_p->source_ip_addr);
#   endif

    *source_port_p = udp_header_p->source_port;
    *rx_packet_pp = rx_packet_p;
    rtos_mpu_remove_thread_data_region(); /* g_networking */
    return 0;
}


void
net_send_ipv4_tcp_segment(struct local_l4_end_point *local_l4_end_point_p,
		          const struct ipv4_address *dest_ip_addr_p,
			  uint16_t dest_port,
		          struct network_packet *tx_packet_p,
		          size_t data_payload_length)
{
#if 0 // TODO: Finish implementing this
    /*
     * Populate TCP header:
     */
    struct tcp_header *tcp_header_p =
	(struct tcp_header *)GET_IPV4_DATA_PAYLOAD_AREA(tx_packet_p);

    FDC_ASSERT(local_l4_end_point_p->protocol == TRANSPORT_PROTO_TCP,
	       local_l4_end_point_p->protocol, local_l4_end_point_p);

    tcp_header_p->source_port = local_l4_end_point_p->port;
    tcp_header_p->dest_port = dest_port;
    tcp_header_p->datagram_length = hton16(sizeof(struct tcp_header) +
	                                   data_payload_length);

    /*
     * NOTE: tcp_header_p->segment_checksum is filled by hardware
     */
    tcp_header_p->segment_checksum = 0;

    /*
     * Send IP packet:
     */
    net_send_ipv4_packet(dest_ip_addr_p,
		         tx_packet_p,
		         sizeof(struct tcp_header) + data_payload_length,
		         TRANSPORT_PROTO_TCP);
#else
    DEBUG_PRINTF("Not implemented yet\n");
#endif
}


fdc_error_t
net_send_ipv4_icmp_message(const struct ipv4_address *dest_ip_addr_p,
		           struct network_packet *tx_packet_p,
			   uint8_t msg_type,
		           uint8_t msg_code,
		           size_t data_payload_length)
{
    /*
     * Populate ICMP header:
     */
    struct icmpv4_header *icmp_header_p =
	(struct icmpv4_header *)GET_IPV4_DATA_PAYLOAD_AREA(tx_packet_p);

    icmp_header_p->msg_type = msg_type;
    icmp_header_p->msg_code = msg_code;

    /*
     * NOTE: icmp_header_p->msg_checksum is computed by hardware.
     * We just need to initialize the checksum field to 0
     */
    icmp_header_p->msg_checksum = 0;

#   ifndef ENET_CHECKSUM_OFFLOAD
    icmp_header_p->msg_checksum =
	net_compute_checksum(icmp_header_p,
			     sizeof(struct icmpv4_header) + data_payload_length);
#   endif

    /*
     * Send IP packet:
     */
    return net_send_ipv4_packet(dest_ip_addr_p,
				tx_packet_p,
				sizeof(struct icmpv4_header) + data_payload_length,
				TRANSPORT_PROTO_ICMP);
}


fdc_error_t
net_send_ipv4_ping_request(const struct ipv4_address *dest_ip_addr_p,
			   uint16_t identifier,
	                   uint16_t seq_num)
{
    fdc_error_t fdc_error;

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                sizeof g_networking,
                                                false);
    if (fdc_error != 0) {
        return fdc_error;
    }

    rtos_mutex_acquire(&g_networking.expecting_ping_reply_mutex);
    while (g_networking.expecting_ping_reply) {
	rtos_condvar_wait(&g_networking.ping_reply_recceived_condvar,
			  &g_networking.expecting_ping_reply_mutex,
			  NULL);
    }

    g_networking.expecting_ping_reply = true;
    rtos_mutex_release(&g_networking.expecting_ping_reply_mutex);

    struct network_packet *tx_packet_p = net_allocate_tx_packet(true);
    
    DBG_ASSERT(tx_packet_p != NULL, dest_ip_addr_p, seq_num);
    struct icmpv4_echo_message *echo_msg_p =
	(struct icmpv4_echo_message *)GET_IPV4_DATA_PAYLOAD_AREA(tx_packet_p);

    echo_msg_p->identifier = identifier;
    echo_msg_p->seq_num = seq_num;
    fdc_error = net_send_ipv4_icmp_message(dest_ip_addr_p,
					   tx_packet_p,
					   ICMP_TYPE_PING_REQUEST,
					   ICMP_CODE_PING_REQUEST,
					   sizeof(struct icmpv4_echo_message) -
					   sizeof(struct icmpv4_header));
    if (fdc_error != 0) {
	g_networking.expecting_ping_reply = false;
    }

    rtos_mpu_remove_thread_data_region();   /* g_networking */
    return fdc_error;
}


fdc_error_t
net_receive_ipv4_ping_reply(rtos_milliseconds_t timeout_ms,
			    struct ipv4_address *remote_ip_addr_p,
			    uint16_t *identifier_p,
			    uint16_t *seq_num_p)
{
    fdc_error_t fdc_error;
    fdc_error = rtos_mpu_add_thread_data_region(&g_networking,
                                                sizeof g_networking,
                                                false);
    if (fdc_error != 0) {
        return fdc_error;
    }

    struct glist_node *rx_packet_node_p =
	rtos_queue_remove(&g_networking.rx_ipv4_ping_reply_packet_queue, timeout_ms);

    if (rx_packet_node_p == NULL) {
        fdc_error = CAPTURE_FDC_ERROR("No Rx packet available", timeout_ms, 0);
	goto exit_remove_region;
    }

    struct network_packet *rx_packet_p = GLIST_NODE_TO_NETWORK_PACKET(rx_packet_node_p);

    DBG_ASSERT(rx_packet_p->signature == NET_RX_PACKET_SIGNATURE,
	       rx_packet_p, 0);

    struct ipv4_header *ipv4_header_p = GET_IPV4_HEADER(rx_packet_p);
    struct icmpv4_header *icmpv4_header_p = GET_IPV4_DATA_PAYLOAD_AREA(rx_packet_p);
    struct icmpv4_echo_message *echo_msg_p = (struct icmpv4_echo_message *)(icmpv4_header_p);

    remote_ip_addr_p->value = ipv4_header_p->source_ip_addr.value;
    *identifier_p = echo_msg_p->identifier;
    *seq_num_p = echo_msg_p->seq_num;
    net_recycle_rx_packet(rx_packet_p);
    fdc_error = 0;

exit_remove_region:
    rtos_mpu_remove_thread_data_region();   /* g_networking */
    return fdc_error;

}


static fdc_error_t
select_ipv6_router(struct local_l3_end_point *local_l3_end_point_p,
                   const struct ipv6_address *dest_ip_addr_p,
                   struct ipv6_address *router_ip_addr_p)
{
    return CAPTURE_FDC_ERROR("select_ipv6_router() not implemented yet", 0, 0);
}


static fdc_error_t
resolve_dest_ipv6_addr(struct local_l3_end_point *local_l3_end_point_p,
		       const struct ipv6_address *dest_ip_addr_p,
		       struct ethernet_mac_address *dest_mac_addr_p)
{
    fdc_error_t fdc_error;
#if 0 //???
    unsigned int arp_request_retries = 0;
    struct arp_cache_entry *matching_entry_p = NULL;
    struct arp_cache_entry *free_entry_p = NULL;
    struct arp_cache *arp_cache_p = &local_l3_end_point_p->ipv4.arp_cache;

    rtos_mutex_acquire(&arp_cache_p->mutex);
    for ( ; ; ) {
	bool send_arp_request = false;
	matching_entry_p = arp_cache_lookup_or_allocate(arp_cache_p, dest_ip_addr_p,
							&free_entry_p);
	rtos_ticks_t current_ticks = rtos_get_ticks();

	if (matching_entry_p == NULL) {
	    send_arp_request = true;
	} else if (matching_entry_p->state == ARP_ENTRY_FILLED) {
	    if (RTOS_TICKS_DELTA(matching_entry_p->entry_filled_time_stamp,
				 current_ticks) <
		ARP_CACHE_ENTRY_LIFETIME_IN_TICKS) {
		/*
		 * ARP cache hit
		 */
		*dest_mac_addr_p = matching_entry_p->dest_mac_addr;
		break;
	    } else {
		/*
		 * ARP entry expired, send a new ARP request:
		 */
		DEBUG_PRINTF("Expired ARP cache entry for IP address %u.%u.%u.%u\n",
			     dest_ip_addr_p->bytes[0],
			     dest_ip_addr_p->bytes[1],
			     dest_ip_addr_p->bytes[2],
			     dest_ip_addr_p->bytes[3]);

		matching_entry_p->state = ARP_ENTRY_INVALID;
		send_arp_request = true;
	    }
	} else {
	    FDC_ASSERT(matching_entry_p->state == ARP_ENTRY_HALF_FILLED,
		       matching_entry_p->state, matching_entry_p);

	    if (RTOS_TICKS_DELTA(matching_entry_p->arp_request_time_stamp, current_ticks) >=
		MILLISECONDS_TO_TICKS(ARP_REPLY_WAIT_TIMEOUT_IN_MS)) {
		/*
		 * Re-send ARP request:
		 */
		capture_fdc_msg_printf("Outstanding ARP request re-sent for IP address: %u.%u.%u.%u\n",
				       dest_ip_addr_p->bytes[0],
				       dest_ip_addr_p->bytes[1],
				       dest_ip_addr_p->bytes[2],
				       dest_ip_addr_p->bytes[3]);

		send_arp_request = true;
	    }
	}

	/*
	 * Send ARP request if necessary:
	 */
	if (send_arp_request) {
	    if (arp_request_retries == ARP_REQUEST_MAX_RETRIES) {
		fdc_error = CAPTURE_FDC_ERROR("Unreachable IP address",
					      dest_ip_addr_p->value, 0);

		capture_fdc_msg_printf("Unreachable IP address: %u.%u.%u.%u\n",
				       dest_ip_addr_p->bytes[0],
				       dest_ip_addr_p->bytes[1],
				       dest_ip_addr_p->bytes[2],
				       dest_ip_addr_p->bytes[3]);

		goto common_exit;
	    }

	    arp_request_retries ++;
	    if (matching_entry_p != NULL) {
		matching_entry_p->arp_request_time_stamp = rtos_get_ticks();
		matching_entry_p->state = ARP_ENTRY_HALF_FILLED;
	    } else {
		FDC_ASSERT(free_entry_p != NULL, dest_ip_addr_p, arp_cache_p);
		free_entry_p->dest_ip_addr.value = dest_ip_addr_p->value;
		free_entry_p->arp_request_time_stamp = rtos_get_ticks();
		free_entry_p->state = ARP_ENTRY_HALF_FILLED;
	    }

	    net_send_arp_request(local_l3_end_point_p->enet_device_p,
				 &local_l3_end_point_p->ipv4.local_ip_addr,
				 dest_ip_addr_p);
	}

	/*
	 * Wait for ARP cache update:
	 */
	rtos_milliseconds_t timeout = ARP_REPLY_WAIT_TIMEOUT_IN_MS;
	rtos_condvar_wait(&arp_cache_p->cache_updated_condvar,
			  &arp_cache_p->mutex,
			  &timeout);
    }

    fdc_error = 0;

common_exit:
    rtos_mutex_release(&arp_cache_p->mutex);
#else
    fdc_error = CAPTURE_FDC_ERROR("Not implemented yet,", 0, 0);
#endif

    return fdc_error;
}


fdc_error_t
net_send_ipv6_packet(const struct ipv6_address *dest_ip_addr_p,
		     struct network_packet *tx_packet_p,
		     size_t data_payload_length,
		     enum l4_protocols l4_protocol)
{
    struct neighbor_cache_entry *neighbor_cache_entry_p = NULL;
    struct ethernet_mac_address dest_mac_addr;
    fdc_error_t fdc_error;

    FDC_ASSERT(tx_packet_p->signature == NET_TX_PACKET_SIGNATURE,
	       tx_packet_p->signature, tx_packet_p);
    FDC_ASSERT(data_payload_length <= NET_MAX_IPV6_PACKET_PAYLOAD_SIZE,
	       data_payload_length, tx_packet_p);

    struct ethernet_frame *tx_frame_p =
       (struct ethernet_frame *)tx_packet_p->data_buffer;

    struct local_l3_end_point *local_l3_end_point_p =
	choose_ipv6_local_l3_end_point(dest_ip_addr_p);

    const struct ipv6_address *source_ip_addr_p = 
        &local_l3_end_point_p->ipv6.link_local_ip_addr.addr;

    const struct enet_device *enet_device_p = local_l3_end_point_p->enet_device_p;

    FDC_ASSERT(!IPV6_ADDRESSES_EQUAL(dest_ip_addr_p, &ipv6_unspecified_address),
               dest_ip_addr_p, tx_packet_p);

    if (local_l3_end_point_p->ipv6.link_local_ip_addr.state == IPV6_ADDR_TENTATIVE &&
        l4_protocol != TRANSPORT_PROTO_ICMPV6) {
        fdc_error = CAPTURE_FDC_ERROR(
                        "Packet cannot be sent, as local IPv6 address is still tentative",
                        l4_protocol, local_l3_end_point_p);
        return fdc_error;
    }

    /*
     * Populate IPv6 header
     */
    tx_frame_p->ipv6_header.first_word = 0;
    SET_BIT_FIELD(tx_frame_p->ipv6_header.first_word,
		  IPv6_VERSION_MASK, IPv6_VERSION_SHIFT, 6);

    tx_frame_p->ipv6_header.payload_length = hton16(data_payload_length);
    tx_frame_p->ipv6_header.next_header = l4_protocol;
    tx_frame_p->ipv6_header.hop_limit = 64; /* max routing hops */

#   ifdef ENET_DATA_PAYLOAD_32_BIT_ALIGNED
    COPY_IPv6_ADDRESS(&tx_frame_p->ipv6_header.source_ipv6_addr,
				&local_l3_end_point_p->ipv6.link_local_ip_addr.addr);

    COPY_IPv6_ADDRESS(&tx_frame_p->ipv6_header.dest_ipv6_addr,
				dest_ip_addr_p);
#   else
    COPY_UNALIGNED_IPv6_ADDRESS(&tx_frame_p->ipv6_header.source_ipv6_addr,
				&local_l3_end_point_p->ipv6.link_local_ip_addr.addr);

    COPY_UNALIGNED_IPv6_ADDRESS(&tx_frame_p->ipv6_header.dest_ipv6_addr,
				dest_ip_addr_p);
#   endif

    /*
     * Populate Ethernet header
     */

#if 0 /* ENET hardware does this automatically */
    ENET_COPY_MAC_ADDRESS(&tx_frame_p->enet_header.source_mac_addr,
			  &enet_device_p->mac_address);
#endif

    tx_frame_p->enet_header.frame_type = hton16(ENET_IPv6_PACKET);

    tx_packet_p->total_length = sizeof(struct ethernet_header) +
				sizeof(struct ipv6_header) +
				data_payload_length;

    /*
     * Get destination MAC address:
     */
    if (IPV6_ADDR_IS_MULTICAST(dest_ip_addr_p)) {
        map_ipv6_multicast_addr_to_ethernet_multicast_addr(dest_ip_addr_p,
                                                           &dest_mac_addr);
        fdc_error = 0;
    } else {
        if (IPV6_ADDRESSES_EQUAL(&local_l3_end_point_p->ipv6.link_local_ip_addr.addr,
                                 dest_ip_addr_p)) {
            fdc_error = CAPTURE_FDC_ERROR("IPv6 Loopback not supported", 0, 0);
        } else if (IPV6_ADDR_IS_LINK_LOCAL(dest_ip_addr_p)) {
            fdc_error = resolve_dest_ipv6_addr(local_l3_end_point_p,
                                               dest_ip_addr_p,
                                               &dest_mac_addr);
        } else {
            struct ipv6_address router_ip_addr;

            fdc_error = select_ipv6_router(local_l3_end_point_p, dest_ip_addr_p,
                                           &router_ip_addr);
            if (fdc_error != 0) {
                return fdc_error;
            }

            fdc_error = resolve_dest_ipv6_addr(local_l3_end_point_p,
                                               &router_ip_addr,
                                               &dest_mac_addr);
        }
    }

    if (fdc_error != 0) {
	return fdc_error;
    }

    COPY_MAC_ADDRESS(&tx_frame_p->enet_header.dest_mac_addr, &dest_mac_addr);

    /*
     * Transmit packet:
     */
    enet_start_xmit(enet_device_p, tx_packet_p);
    return 0;
}


fdc_error_t
net_send_ipv6_udp_datagram(struct local_l4_end_point *local_l4_end_point_p,
		           const struct ipv6_address *dest_ip_addr_p,
			   uint16_t dest_port,
		           struct network_packet *tx_packet_p,
		           size_t data_payload_length)
{
    DEBUG_PRINTF("Not implemented yet\n");
    return 0;
}


fdc_error_t
net_receive_ipv6_udp_datagram(struct local_l4_end_point *local_l4_end_point_p,
			      rtos_milliseconds_t timeout_ms,
		              struct ipv6_address *source_ip_addr_p,
			      uint16_t *source_port_p,
			      struct network_packet **rx_packet_pp)
{
    DEBUG_PRINTF("Not implemented yet\n");
    return 0;
}


void
net_send_ipv6_tcp_segment(struct local_l4_end_point *local_l4_end_point_p,
		          const struct ipv6_address *dest_ip_addr_p,
			  uint16_t dest_port,
		          struct network_packet *tx_packet_p,
		          size_t data_payload_length)
{
    DEBUG_PRINTF("Not implemented yet\n");
}


fdc_error_t
net_send_ipv6_icmp_message(const struct ipv6_address *dest_ip_addr_p,
		           struct network_packet *tx_packet_p,
			   uint8_t msg_type,
		           uint8_t msg_code,
                           uint32_t header_data,
		           size_t data_payload_length)
{
    /*
     * Populate ICMPv6 header:
     */
    struct icmpv6_header *icmp_header_p =
	(struct icmpv6_header *)GET_IPV6_DATA_PAYLOAD_AREA(tx_packet_p);

    icmp_header_p->msg_type = msg_type;
    icmp_header_p->msg_code = msg_code;
    icmp_header_p->data = header_data;

    /*
     * NOTE: icmp_header_p->msg_checksum is computed by hardware.
     * We just need to initialize the checksum field to 0
     */
    icmp_header_p->msg_checksum = 0;

#   ifndef ENET_CHECKSUM_OFFLOAD
    icmp_header_p->msg_checksum =
	net_compute_checksum(icmp_header_p,
			     sizeof(struct icmpv6_header) + data_payload_length);
#   endif

    /*
     * Send IPv6 packet:
     */
    return net_send_ipv6_packet(dest_ip_addr_p,
				tx_packet_p,
				sizeof(struct icmpv6_header) + data_payload_length,
				TRANSPORT_PROTO_ICMPV6);
}


fdc_error_t
net_send_ipv6_ping_request(const struct ipv6_address *dest_ip_addr_p,
			   uint16_t identifier,
		           uint16_t seq_num)
{
    return 0;
}


fdc_error_t
net_receive_ipv6_ping_reply(rtos_milliseconds_t timeout_ms,
			    struct ipv6_address *remote_ip_addr_p,
			    uint16_t *identifier_p,
			    uint16_t *seq_num_p)
{
    return 0;
}


#ifdef NET_TRACE
static void
net_trace_received_arp_packet(struct network_packet *packet_p)
{
    struct ethernet_frame *frame_p =
	(struct ethernet_frame *)packet_p->data_buffer;

    uint16_t arp_operation = ntoh16(frame_p->arp_packet.operation);

    DEBUG_PRINTF("Received ARP packet:\n"
		"\toperation: %s (%#x)\n"
		"\tsource mac addr: %x:%x:%x:%x:%x:%x\n"
		"\tsource IP address: %u.%u.%u.%u\n"
		"\tdest mac addr: %x:%x:%x:%x:%x:%x\n"
		"\tdest IP address: %u.%u.%u.%u\n",
		arp_operation == ARP_REQUEST ? "ARP request" : "ARP reply",
		arp_operation,
		frame_p->arp_packet.source_mac_addr.bytes[0],
		frame_p->arp_packet.source_mac_addr.bytes[1],
		frame_p->arp_packet.source_mac_addr.bytes[2],
		frame_p->arp_packet.source_mac_addr.bytes[3],
		frame_p->arp_packet.source_mac_addr.bytes[4],
		frame_p->arp_packet.source_mac_addr.bytes[5],
		frame_p->arp_packet.source_ip_addr.bytes[0],
		frame_p->arp_packet.source_ip_addr.bytes[1],
		frame_p->arp_packet.source_ip_addr.bytes[2],
		frame_p->arp_packet.source_ip_addr.bytes[3],
		frame_p->arp_packet.dest_mac_addr.bytes[0],
		frame_p->arp_packet.dest_mac_addr.bytes[1],
		frame_p->arp_packet.dest_mac_addr.bytes[2],
		frame_p->arp_packet.dest_mac_addr.bytes[3],
		frame_p->arp_packet.dest_mac_addr.bytes[4],
		frame_p->arp_packet.dest_mac_addr.bytes[5],
		frame_p->arp_packet.dest_ip_addr.bytes[0],
		frame_p->arp_packet.dest_ip_addr.bytes[1],
		frame_p->arp_packet.dest_ip_addr.bytes[2],
		frame_p->arp_packet.dest_ip_addr.bytes[3]);
}
#endif


static void
net_receive_arp_packet(struct network_packet *rx_packet_p)
{
    FDC_ASSERT(rx_packet_p->total_length >=
	       sizeof(struct ethernet_header) + sizeof(struct arp_packet),
	       rx_packet_p->total_length, rx_packet_p);

    NET_TRACE_RECEIVED_ARP_PACKET(rx_packet_p);

    struct local_l3_end_point *local_l3_end_point_p = rx_packet_p->local_l3_end_point_p;
    const struct enet_device *enet_device_p = local_l3_end_point_p->enet_device_p;
    struct ethernet_frame *rx_frame_p =
	(struct ethernet_frame *)rx_packet_p->data_buffer;

    uint16_t arp_operation = ntoh16(rx_frame_p->arp_packet.operation);
    struct ipv4_address source_ip_addr;

    if (arp_operation == ARP_REQUEST || arp_operation == ARP_REPLY) {
	FDC_ASSERT(rx_frame_p->arp_packet.link_addr_type == hton16(0x1),
	           rx_frame_p->arp_packet.link_addr_type, rx_frame_p);
	FDC_ASSERT(rx_frame_p->arp_packet.network_addr_type == hton16(ENET_IPv4_PACKET),
		   rx_frame_p->arp_packet.network_addr_type, rx_frame_p);
	FDC_ASSERT(rx_frame_p->arp_packet.link_addr_size == sizeof(struct ethernet_mac_address),
		   rx_frame_p->arp_packet.network_addr_size, rx_frame_p);
	FDC_ASSERT(rx_frame_p->arp_packet.network_addr_size == sizeof(struct ipv4_address),
		   rx_frame_p->arp_packet.network_addr_size, rx_frame_p);
    }

    switch(arp_operation) {
    case ARP_REQUEST:
	if (UNALIGNED_IPv4_ADDRESSES_EQUAL(&rx_frame_p->arp_packet.dest_ip_addr,
				           &local_l3_end_point_p->ipv4.local_ip_addr)) {
	    /*
	     * Send ARP reply
	     */
	    net_send_arp_reply(enet_device_p,
			       &local_l3_end_point_p->ipv4.local_ip_addr,
			       &rx_frame_p->arp_packet.source_mac_addr,
			       &rx_frame_p->arp_packet.source_ip_addr);
	}

	if (source_ip_addr.value != IPV4_NULL_ADDR) {
	    /*
	     * Update ARP cache with (source IP addr, source MAC addr)
	     */
	    COPY_UNALIGNED_IPv4_ADDRESS(&source_ip_addr,
					&rx_frame_p->arp_packet.source_ip_addr);
	    arp_cache_update(&local_l3_end_point_p->ipv4.arp_cache,
			     &source_ip_addr,
			     &rx_frame_p->arp_packet.source_mac_addr);
	}

	break;

    case ARP_REPLY:
	FDC_ASSERT(
	    UNALIGNED_IPv4_ADDRESSES_EQUAL(&rx_frame_p->arp_packet.dest_ip_addr,
					   &local_l3_end_point_p->ipv4.local_ip_addr),
	    &rx_frame_p->arp_packet.dest_ip_addr,
	    &local_l3_end_point_p->ipv4.local_ip_addr);

#       ifdef _RELIABILITY_CHECKS_
        struct ethernet_mac_address local_mac_address;
        
        enet_get_mac_addr(local_l3_end_point_p->enet_device_p, &local_mac_address);

	FDC_ASSERT(
	    MAC_ADDRESSES_EQUAL(&rx_frame_p->arp_packet.dest_mac_addr, &local_mac_address),
	    rx_frame_p->arp_packet.dest_mac_addr.hwords[0],
	    rx_frame_p->arp_packet.dest_mac_addr.hwords[1]);
#       endif

	if (UNALIGNED_IPv4_ADDRESSES_EQUAL(&rx_frame_p->arp_packet.source_ip_addr,
					   &local_l3_end_point_p->ipv4.local_ip_addr)) {
	    capture_fdc_msg_printf(
		"Another Layer-3 end point (%x:%x:%x:%x:%x:%x) has the same IP address (%u.%u.%u.%u):\n",
		rx_frame_p->arp_packet.source_mac_addr.bytes[0],
		rx_frame_p->arp_packet.source_mac_addr.bytes[1],
		rx_frame_p->arp_packet.source_mac_addr.bytes[2],
		rx_frame_p->arp_packet.source_mac_addr.bytes[3],
		rx_frame_p->arp_packet.source_mac_addr.bytes[4],
		rx_frame_p->arp_packet.source_mac_addr.bytes[5],
		rx_frame_p->arp_packet.source_ip_addr.bytes[0],
		rx_frame_p->arp_packet.source_ip_addr.bytes[1],
		rx_frame_p->arp_packet.source_ip_addr.bytes[2],
		rx_frame_p->arp_packet.source_ip_addr.bytes[3]);
	} else {
	    /*
	     * Update ARP cache with (source IP addr, source MAC addr)
	     */
	    COPY_UNALIGNED_IPv4_ADDRESS(&source_ip_addr,
					&rx_frame_p->arp_packet.source_ip_addr);
	    arp_cache_update(&local_l3_end_point_p->ipv4.arp_cache,
			     &source_ip_addr,
			     &rx_frame_p->arp_packet.source_mac_addr);
	}
	break;

    default:
	capture_fdc_msg_printf("Received ARP packet with unsupported operation (%#x)\n",
			       arp_operation);
    }

    net_recycle_rx_packet(rx_packet_p);
}


static void
net_send_ipv4_ping_reply(const struct ipv4_address *dest_ip_addr_p,
			 const struct icmpv4_echo_message *ping_request_msg_p)
{
    struct network_packet *tx_packet_p = net_allocate_tx_packet(true);
    struct icmpv4_echo_message *echo_msg_p =
	(struct icmpv4_echo_message *)GET_IPV4_DATA_PAYLOAD_AREA(tx_packet_p);

    echo_msg_p->identifier = ping_request_msg_p->identifier;
    echo_msg_p->seq_num = ping_request_msg_p->seq_num;
    net_send_ipv4_icmp_message(dest_ip_addr_p,
			       tx_packet_p,
			       ICMP_TYPE_PING_REPLY,
		               ICMP_CODE_PING_REPLY,
		               sizeof(struct icmpv4_echo_message) -
			       sizeof(struct icmpv4_header));
}


static void
net_process_incoming_icmpv4_message(struct network_packet *rx_packet_p)
{
    bool signal_ping_reply_received = false;

    FDC_ASSERT(rx_packet_p->total_length >=
	       sizeof(struct ethernet_header) + sizeof(struct ipv4_header) +
	       sizeof(struct icmpv4_header),
	       rx_packet_p->total_length, rx_packet_p);

    struct ipv4_header *ipv4_header_p = GET_IPV4_HEADER(rx_packet_p);
    struct icmpv4_header *icmpv4_header_p = GET_IPV4_DATA_PAYLOAD_AREA(rx_packet_p);

    switch (icmpv4_header_p->msg_type) {
    case ICMP_TYPE_PING_REPLY:
	FDC_ASSERT(icmpv4_header_p->msg_code == ICMP_CODE_PING_REPLY,
		   icmpv4_header_p->msg_code, rx_packet_p);

	rtos_mutex_acquire(&g_networking.expecting_ping_reply_mutex);
	if (g_networking.expecting_ping_reply) {
	    rtos_queue_add(&g_networking.rx_ipv4_ping_reply_packet_queue,
		           &rx_packet_p->node);
	    g_networking.expecting_ping_reply = false;
	    signal_ping_reply_received = true;
	} else {
	    /*
	     * Drop unmatched ping reply
	     */
	    net_recycle_rx_packet(rx_packet_p);
	}

	rtos_mutex_release(&g_networking.expecting_ping_reply_mutex);
	if (signal_ping_reply_received) {
	    rtos_condvar_signal(&g_networking.ping_reply_recceived_condvar);
	}

	break;

    case ICMP_TYPE_PING_REQUEST:
	FDC_ASSERT(icmpv4_header_p->msg_code == ICMP_CODE_PING_REQUEST,
		   icmpv4_header_p->msg_code, rx_packet_p);

	struct ipv4_address dest_ip_addr;

#	ifdef ENET_DATA_PAYLOAD_32_BIT_ALIGNED
	dest_ip_addr.value = ipv4_header_p->source_ip_addr.value;
#	else
	COPY_UNALIGNED_IPv4_ADDRESS(&dest_ip_addr,
				    &ipv4_header_p->source_ip_addr);
#	endif

	net_send_ipv4_ping_reply(
	    &dest_ip_addr,
	    (struct icmpv4_echo_message *)(icmpv4_header_p));

	net_recycle_rx_packet(rx_packet_p);
	break;

    default:
	capture_fdc_msg_printf("Received ICMP message with unsupported type: %#x\n",
			       icmpv4_header_p->msg_type);
	net_recycle_rx_packet(rx_packet_p);
    }
}


static void
net_process_incoming_tcp_segment(struct network_packet *rx_packet_p)
{
#if 0 // TODO: Finish implementing this:
    FDC_ASSERT(rx_packet_p->total_length >=
	       sizeof(struct ethernet_header) + sizeof(struct ipv4_header) +
	       sizeof(struct tcp_header),
	       rx_packet_p->total_length, rx_packet_p);

    struct tcp_header *tcp_header_p = GET_IPV4_DATA_PAYLOAD_AREA(rx_packet_p);

#else
    DEBUG_PRINTF("Received TCP segment ignored - not supported yet\n");
    net_recycle_rx_packet(rx_packet_p);
#endif
}


static void
net_process_incoming_udp_datagram(struct network_packet *rx_packet_p)
{

    FDC_ASSERT(rx_packet_p->total_length >=
	       sizeof(struct ethernet_header) + sizeof(struct ipv4_header) +
	       sizeof(struct udp_header),
	       rx_packet_p->total_length, rx_packet_p);

    struct udp_header *udp_header_p = GET_IPV4_DATA_PAYLOAD_AREA(rx_packet_p);

    /*
     * Lookup local Layer-4 end point by destination port:
     */
    struct local_l4_end_point *local_l4_end_point_p = NULL;

    rtos_mutex_acquire(&g_networking.local_l4_end_points_mutex);
    for (struct local_l4_end_point *p = g_networking.local_l4_end_points;
	 p != g_networking.next_free_l4_end_point_p; p ++) {
	if (p->l4_protocol == TRANSPORT_PROTO_UDP &&
	    p->l4_port == udp_header_p->dest_port) {
	    local_l4_end_point_p = p;
	    break;
	}
    }

    rtos_mutex_release(&g_networking.local_l4_end_points_mutex);

    if (local_l4_end_point_p != NULL) {
	rtos_queue_add(&local_l4_end_point_p->l4_rx_packet_queue,
		         &rx_packet_p->node);
    } else {
	capture_fdc_msg_printf("Received UDP datagram ignored: unknown port %u\n",
			       udp_header_p->dest_port);
	net_recycle_rx_packet(rx_packet_p);
    }
}


static void
net_receive_ipv4_packet(struct network_packet *rx_packet_p)
{
    FDC_ASSERT(rx_packet_p->total_length >=
	       sizeof(struct ethernet_header) + sizeof(struct ipv4_header),
	       rx_packet_p->total_length, rx_packet_p);
        
    struct local_l3_end_point *local_l3_end_point_p = rx_packet_p->local_l3_end_point_p;
    struct ipv4_header *ipv4_header_p = GET_IPV4_HEADER(rx_packet_p);

    ATOMIC_POST_INCREMENT_UINT32(&g_networking.received_ipv4_packets_count);
    switch (ipv4_header_p->protocol_type) {
    case TRANSPORT_PROTO_ICMP:
	rx_packet_p->state_flags |= NET_PACKET_IN_ICMP_QUEUE;
	rtos_queue_add(&local_l3_end_point_p->ipv4.rx_icmpv4_packet_queue,
		       &rx_packet_p->node);
	break;

    case TRANSPORT_PROTO_TCP:
        net_process_incoming_tcp_segment(rx_packet_p);
	break;

    case TRANSPORT_PROTO_UDP:
	net_process_incoming_udp_datagram(rx_packet_p);
	break;

    default:
	capture_fdc_msg_printf("Received IPv4 packet with unknown protocol type: %#x\n",
			       ipv4_header_p->protocol_type);

	net_recycle_rx_packet(rx_packet_p);
    }
}


static void
net_receive_ipv6_packet(struct network_packet *rx_packet_p)
{

    FDC_ASSERT(rx_packet_p->total_length >=
	       sizeof(struct ethernet_header) + sizeof(struct ipv6_header),
	       rx_packet_p->total_length, rx_packet_p);

    ATOMIC_POST_INCREMENT_UINT32(&g_networking.received_ipv6_packets_count);

#if 0 //???
    struct ethernet_frame *rx_frame =
	(struct ethernet_frame *)rx_packet_p->data_buffer;
#endif //???

    /*
     * TODO: Only repost packet if not enqueued in a local transport end point
     */
    net_recycle_rx_packet(rx_packet_p);
}


/**
 * Packet receive processing thread for a given Layer-3 end point
 */
static fdc_error_t
net_receive_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    bool mpu_region_added = false;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct local_l3_end_point *local_l3_end_point_p =
	(struct local_l3_end_point *)arg;

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking, sizeof g_networking, false);
    if (fdc_error != 0) {
	    goto exit;
    }

    mpu_region_added = true;

    FDC_ASSERT(local_l3_end_point_p->signature == LOCAL_L3_END_POINT_SIGNATURE,
	       local_l3_end_point_p->signature, local_l3_end_point_p);

    const struct enet_device *enet_device_p = local_l3_end_point_p->enet_device_p;

    /*
     * Send gratuitous ARP request (to catch if someone else is using the same
     * IP address):
     */
    if (local_l3_end_point_p->ipv4.local_ip_addr.value != IPV4_NULL_ADDR) {
	net_send_arp_request(enet_device_p,
			     &local_l3_end_point_p->ipv4.local_ip_addr,
			     &local_l3_end_point_p->ipv4.local_ip_addr);
    }

    for ( ; ; ) {
	struct network_packet *rx_packet_p = NULL;

	net_dequeue_rx_packet(local_l3_end_point_p, &rx_packet_p);

	DBG_ASSERT(rx_packet_p != NULL, enet_device_p, cpu_id);
	DBG_ASSERT(rx_packet_p->local_l3_end_point_p == local_l3_end_point_p,
		   rx_packet_p->local_l3_end_point_p, local_l3_end_point_p);

	if (rx_packet_p->state_flags == NET_PACKET_RX_FAILED) {
	    net_recycle_rx_packet(rx_packet_p);
	    continue;
	}

        ATOMIC_POST_INCREMENT_UINT32(&g_networking.received_packets_count);

	struct ethernet_frame *rx_frame =
	    (struct ethernet_frame *)rx_packet_p->data_buffer;

	switch (ntoh16(rx_frame->enet_header.frame_type)) {
	case ENET_ARP_PACKET:
	    net_receive_arp_packet(rx_packet_p);
	    break;
	case ENET_IPv4_PACKET:
	    net_receive_ipv4_packet(rx_packet_p);
	    break;
	case ENET_IPv6_PACKET:
	    net_receive_ipv6_packet(rx_packet_p);
	    break;
	default:
	    capture_fdc_msg_printf("Received frame of unknown type: %#x\n",
				   ntoh16(rx_frame->enet_header.frame_type));

	    net_recycle_rx_packet(rx_packet_p);
	}
    }

exit:
    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    if (mpu_region_added) {
	rtos_mpu_remove_thread_data_region();
    }

    return fdc_error;
}


/**
 * ICMPv4 receive thread for a given Layer-3 end point
 */
static fdc_error_t
net_icmpv4_receive_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    bool mpu_region_added = false;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct local_l3_end_point *local_l3_end_point_p =
	(struct local_l3_end_point *)arg;

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking, sizeof g_networking, false);
    if (fdc_error != 0) {
	    goto exit;
    }

    mpu_region_added = true;

    FDC_ASSERT(local_l3_end_point_p->signature == LOCAL_L3_END_POINT_SIGNATURE,
	       local_l3_end_point_p->signature, local_l3_end_point_p);

    const struct enet_device *enet_device_p = local_l3_end_point_p->enet_device_p;

    for ( ; ; ) {
	struct network_packet *rx_packet_p = NULL;

	rx_packet_p = GLIST_NODE_TO_NETWORK_PACKET(
		rtos_queue_remove(&local_l3_end_point_p->ipv4.rx_icmpv4_packet_queue, 0));

	DBG_ASSERT(rx_packet_p->signature == NET_RX_PACKET_SIGNATURE &&
		   (rx_packet_p->state_flags & NET_PACKET_IN_ICMP_QUEUE),
		   rx_packet_p, local_l3_end_point_p);

	rx_packet_p->state_flags &= ~NET_PACKET_IN_ICMP_QUEUE;
	net_process_incoming_icmpv4_message(rx_packet_p);
    }

exit:
    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    if (mpu_region_added) {
	rtos_mpu_remove_thread_data_region();
    }

    return fdc_error;
}


static void
net_set_local_ipv4_addr_from_dhcp(struct local_l3_end_point *local_l3_end_point_p,
			          struct dhcp_message *dhcp_ack_msg_p,
                                  size_t dhcp_ack_msg_size)
{
    uint32_t subnet_mask = 0;
    uint32_t lease_time = 0;
    struct ipv4_address router_ip_addr = { .value = IPV4_NULL_ADDR };
    struct ipv4_address *local_ip_addr_p = &dhcp_ack_msg_p->your_ip_addr;
    size_t options_len = dhcp_ack_msg_size - sizeof(struct dhcp_message);
    unsigned int expected_options_found = 0;

    FDC_ASSERT(g_networking.initialized, 0, 0);
    FDC_ASSERT(local_ip_addr_p->value != IPV4_NULL_ADDR, local_ip_addr_p->value, 0);

    /*
     * Extract DHCP options:
     */
    for (unsigned int i = 0; i < options_len; ) {
        if (dhcp_ack_msg_p->options[i] == 1) {          /* subnet mask option */
            FDC_ASSERT(dhcp_ack_msg_p->options[i + 1] == 4,
                       dhcp_ack_msg_p->options[i + 1],
                       dhcp_ack_msg_p->options[i]);

            uint8_t *subnet_mask_bytes_p = (uint8_t *)&subnet_mask;

            subnet_mask_bytes_p[0] = dhcp_ack_msg_p->options[i + 2];
            subnet_mask_bytes_p[1] = dhcp_ack_msg_p->options[i + 3];
            subnet_mask_bytes_p[2] = dhcp_ack_msg_p->options[i + 4];
            subnet_mask_bytes_p[3] = dhcp_ack_msg_p->options[i + 5];

            expected_options_found ++;
        } else if (dhcp_ack_msg_p->options[i] == 3) {   /* router option */
            FDC_ASSERT(dhcp_ack_msg_p->options[i + 1] == 4,
                       dhcp_ack_msg_p->options[i + 1],
                       dhcp_ack_msg_p->options[i]);

            router_ip_addr.bytes[0] = dhcp_ack_msg_p->options[i + 2];
            router_ip_addr.bytes[1] = dhcp_ack_msg_p->options[i + 3];
            router_ip_addr.bytes[2] = dhcp_ack_msg_p->options[i + 4];
            router_ip_addr.bytes[3] = dhcp_ack_msg_p->options[i + 5];

            expected_options_found ++;
        } else if (dhcp_ack_msg_p->options[i] == 51) {  /* lease time option */
            FDC_ASSERT(dhcp_ack_msg_p->options[i + 1] == 4,
                       dhcp_ack_msg_p->options[i + 1],
                       dhcp_ack_msg_p->options[i]);

            uint8_t *lease_time_bytes_p = (uint8_t *)&lease_time;

            lease_time_bytes_p[0] = dhcp_ack_msg_p->options[i + 2];
            lease_time_bytes_p[1] = dhcp_ack_msg_p->options[i + 3];
            lease_time_bytes_p[2] = dhcp_ack_msg_p->options[i + 4];
            lease_time_bytes_p[3] = dhcp_ack_msg_p->options[i + 5];

            lease_time = ntoh32(lease_time);
            expected_options_found ++;
        } 

        /*
         * Skip to next option:
         */
        i += dhcp_ack_msg_p->options[i + 1];
    }

    FDC_ASSERT(expected_options_found == 3, expected_options_found, 0);

    local_l3_end_point_p->ipv4.local_ip_addr = *local_ip_addr_p;
    local_l3_end_point_p->ipv4.subnet_mask = subnet_mask;
    local_l3_end_point_p->ipv4.default_gateway_ip_addr = router_ip_addr;
    local_l3_end_point_p->ipv4.dhcp_lease_time = lease_time;

    DEBUG_PRINTF("Local IP address: %u.%u.%u.%u\n",
                 local_ip_addr_p->bytes[0],
                 local_ip_addr_p->bytes[1],
                 local_ip_addr_p->bytes[2],
                 local_ip_addr_p->bytes[3]);

    /*
     * Send gratuitous ARP request (to catch if someone else is using the same
     * IP address):
     */
    net_send_arp_request(local_l3_end_point_p->enet_device_p,
			 &local_l3_end_point_p->ipv4.local_ip_addr,
			 &local_l3_end_point_p->ipv4.local_ip_addr);
}


/**
 * DHCPv4 client thread
 */
static fdc_error_t
net_dhcpv4_client_thread_f(void *arg)
{
    enum dhcp_client_states {
        DHCP_OFFER_EXPECTED,
        DHCP_ACKNOWLEDGE_EXPECTED,
        DHCP_LEASE_GRANTED,
    } state;

    fdc_error_t fdc_error;
    struct network_packet *rx_packet_p = NULL;
    struct ipv4_address server_ip_addr;
    uint16_t server_port;
    struct dhcp_message *dhcp_msg_p;
    size_t dhcp_msg_size;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    rtos_milliseconds_t timeout_ms = 0;
    bool mpu_region_added = false;
    struct local_l3_end_point *local_l3_end_point_p =
	(struct local_l3_end_point *)arg;

    struct local_l4_end_point *client_end_point_p;

    fdc_error = rtos_mpu_add_thread_data_region(&g_networking, sizeof g_networking, false);
    if (fdc_error != 0) {
	    goto exit;
    }

    mpu_region_added = true;
    fdc_error = net_create_local_l4_end_point(TRANSPORT_PROTO_UDP,
					      DHCP_UDP_CLIENT_PORT,
					      &client_end_point_p);
    FDC_ASSERT(fdc_error == 0, fdc_error, 0);

    net_send_ipv4_dhcp_discovery(local_l3_end_point_p, client_end_point_p);

    state = DHCP_OFFER_EXPECTED;

    for ( ; ; ) {
        /*
         * Receive DHCP message:
         */
        net_receive_ipv4_udp_datagram(client_end_point_p,
                                      timeout_ms,
                                      &server_ip_addr,
                                      &server_port,
                                      &rx_packet_p);

        FDC_ASSERT(rx_packet_p != NULL, 0, 0);
        FDC_ASSERT(server_port == DHCP_UDP_SERVER_PORT,
                   server_port, 0);

        DEBUG_PRINTF("DHCP client received message %#x from %u.%u.%u.%u\n",
                     dhcp_msg_p->options[2],
                     server_ip_addr.bytes[0],
                     server_ip_addr.bytes[1],
                     server_ip_addr.bytes[2],
                     server_ip_addr.bytes[3]);

        dhcp_msg_size = net_get_udp_data_payload_length(rx_packet_p);
        FDC_ASSERT(dhcp_msg_size >= sizeof(struct dhcp_message), dhcp_msg_size, 0);
        switch (state) {
        case DHCP_OFFER_EXPECTED:
            dhcp_msg_p = net_get_udp_data_payload_area(rx_packet_p);
            FDC_ASSERT(dhcp_msg_p->op == 0x2, dhcp_msg_p->op, 0);
            FDC_ASSERT(dhcp_msg_p->options[2] == 0x2, dhcp_msg_p->options[2], 0);
            net_send_ipv4_dhcp_request(local_l3_end_point_p, client_end_point_p,
                                       dhcp_msg_p);
            state = DHCP_ACKNOWLEDGE_EXPECTED;
            break;

        case DHCP_ACKNOWLEDGE_EXPECTED:
            dhcp_msg_p = net_get_udp_data_payload_area(rx_packet_p);
            FDC_ASSERT(dhcp_msg_p->op == 0x2, dhcp_msg_p->op, 0);
            FDC_ASSERT(dhcp_msg_p->options[2] == 0x5, dhcp_msg_p->options[2], 0);
            net_set_local_ipv4_addr_from_dhcp(local_l3_end_point_p, dhcp_msg_p,
                                              dhcp_msg_size);
            state = DHCP_LEASE_GRANTED;
            break;

        default:
            /* Drop packet */
            DEBUG_PRINTF("Dropped DHCP message (DHCP client state: %d)\n",
                         state);
        }

        net_recycle_rx_packet(rx_packet_p);
    }

exit:
    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    if (mpu_region_added) {
	rtos_mpu_remove_thread_data_region();
    }

    return fdc_error;
}


