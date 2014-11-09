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
#include <k64f_soc_enet.h>
#include <McRTOS/McRTOS.h>
#include <McRTOS/failure_data_capture.h>
#include <McRTOS/utils.h>

#pragma GCC diagnostic ignored "-Wunused-variable" // ???

/**
 * Ethernet link speed: 100 Mbps
 */
#define ETHERNET_LINK_SPEED_IN_BPS   UINT32_C(100000000)

static fdc_error_t net_receive_thread_f(void *arg);

static const struct ethernet_mac_address enet_broadcast_mac_addr = {
    .bytes = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};

static const struct ethernet_mac_address enet_null_mac_addr = {
    .bytes = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

static struct local_l3_end_point g_local_l3_end_points[] = {
    [0] = {
	.signature = LOCAL_L3_END_POINT_SIGNATURE,
	.enet_device_p = &g_enet_device0,
	.ipv4 = {
	    .local_ip_addr = { .bytes = { 192, 168, 8, 2 } },
	    .subnet_mask = IPv4_SUBNET_MASK(24),
	    .default_gateway_ip_addr = { .bytes = { 192, 168, 8, 1 } },
	    .next_tx_ip_packet_seq_num = 0,
	},
    },
};

static const struct rtos_thread_creation_params net_threads[] = {
    [0] = {
	.p_name_p = "ENET Receive thread",
        .p_function_p = net_receive_thread_f,
        .p_function_arg_p = (void *)&g_local_l3_end_points[0],
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
        .p_thread_pp = NULL,
    },
};


static void
arp_cache_init(struct arp_cache_entry arp_cache[])
{
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    for (unsigned int i = 0; i < ARP_CACHE_NUM_ENTRIES; i++) {
	struct arp_cache_entry *entry_p = &arp_cache[i];

	entry_p->state = ARP_ENTRY_INVALID;
	GLIST_NODE_INIT(&entry_p->pending_tx_packet_list_anchor);
	rtos_k_mutex_init(
	    "ARP cache entry mutex",
	    cpu_id,
	    &entry_p->pending_tx_packet_list_mutex);
    }
}


/**
 * Send an ARP request message
 */
void
net_send_arp_request(const struct enet_device *enet_device_p,
		     const struct ipv4_address *source_ip_addr_p,
		     const struct ipv4_address *dest_ip_addr_p)
{
    struct network_packet *tx_packet_p = enet_allocate_tx_packet(enet_device_p, true);

    DBG_ASSERT(tx_packet_p != NULL, enet_device_p, 0);

    struct ethernet_frame *enet_frame =
	(struct ethernet_frame *)tx_packet_p->data_buffer;

#if 0 // Hw does this
    COPY_MAC_ADDRESS(&enet_frame->enet_header.source_mac_addr,
		     &enet_device_p->mac_address);
#endif

    COPY_MAC_ADDRESS(&enet_frame->enet_header.dest_mac_addr,
		     &enet_broadcast_mac_addr);
    enet_frame->enet_header.frame_type = hton16(ENET_ARP_PACKET);
    enet_frame->arp_packet.link_addr_type = hton16(0x1);
    enet_frame->arp_packet.network_addr_type = hton16(ENET_IPv4_PACKET);
    enet_frame->arp_packet.link_addr_size = sizeof(struct ethernet_mac_address);
    enet_frame->arp_packet.network_addr_size = sizeof(struct ipv4_address);
    enet_frame->arp_packet.operation = hton16(ARP_REQUEST);
    COPY_MAC_ADDRESS(&enet_frame->arp_packet.source_mac_addr,
		     &enet_device_p->mac_address);
    COPY_IPv4_ADDRESS(&enet_frame->arp_packet.source_ip_addr,
		      source_ip_addr_p);
    COPY_MAC_ADDRESS(&enet_frame->arp_packet.dest_mac_addr,
		     &enet_null_mac_addr);
    COPY_IPv4_ADDRESS(&enet_frame->arp_packet.dest_ip_addr,
		      dest_ip_addr_p);

    tx_packet_p->total_length =
	sizeof(struct ethernet_header) + sizeof(struct arp_packet);

    enet_start_xmit(enet_device_p, tx_packet_p);
}


/**
 * Initialize networking subsystem
 */
void
networking_init(void)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    /*
     * Initialize ARP cache for each local network end point
     */
    for (unsigned int i = 0; i < ARRAY_SIZE(g_local_l3_end_points); i ++) {
	struct local_l3_end_point *l3_end_point_p = &g_local_l3_end_points[i];

	arp_cache_init(l3_end_point_p->ipv4.arp_cache);
	//TODO: Init IPv6 neighbor cache
    }

    /*
     * Create networking threads:
     */
    for (unsigned int i = 0; i < ARRAY_SIZE(net_threads); i ++) {
	fdc_error = rtos_k_create_thread(&net_threads[i]);
	if (fdc_error != 0) {
	    console_printf(
		"CPU core %u: *** Error creating application thread '%s' ***\n",
		cpu_id, net_threads[i].p_name_p);

	    fatal_error_handler(fdc_error);
	}

	console_printf("CPU core %u: %s started\n", cpu_id, net_threads[i].p_name_p);
    }
}


static bool
arp_cache_lookup(struct local_l3_end_point *local_l3_end_point_p,
		 const struct ipv4_address *dest_ip_addr_p,
		 struct arp_cache_entry **arp_entry_pp)
{
    unsigned int i;
    struct arp_cache_entry *first_free_entry_p = NULL;
    struct arp_cache_entry *least_recently_used_entry_p = NULL;
    rtos_ticks_t least_recently_used_ticks_delta = 0;
    bool dest_mac_addr_found = false;
    bool send_arp_request = false;
    struct arp_cache_entry *chosen_entry_p = NULL;

    for (i = 0; i < ARP_CACHE_NUM_ENTRIES; i++) {
	struct arp_cache_entry *entry_p = &local_l3_end_point_p->ipv4.arp_cache[i];
	rtos_ticks_t current_ticks = rtos_k_get_ticks();

	if (entry_p->state == ARP_ENTRY_INVALID) {
	    if (first_free_entry_p == NULL) {
		first_free_entry_p = entry_p;
	    }

	    continue;
	}

	if (entry_p->dest_ip_addr.value == dest_ip_addr_p->value) {
	    entry_p->last_lookup_time_stamp = current_ticks;
	    chosen_entry_p = entry_p;
	    if (entry_p->state == ARP_ENTRY_ARP_REPLY_RECEIVED) {
		if (RTOS_TICKS_DELTA(entry_p->arp_reply_time_stamp, current_ticks) <
		    ARP_CACHE_ENTRY_LIFETIME_IN_TICKS) {
		    /*
		     * ARP cache hit
		     */
		    dest_mac_addr_found = true;
		} else {
		    /*
		     * ARP entry expired, send a new ARP request:
		     */
		    send_arp_request = true;
		}
	    } else {
		FDC_ASSERT(entry_p->state == ARP_ENTRY_ARP_REQUEST_SENT,
			   entry_p->state, entry_p);

		if (RTOS_TICKS_DELTA(entry_p->arp_request_time_stamp, current_ticks) >=
		    ARP_REPLY_WAIT_TIMEOUT_IN_TICKS) {
		    /*
		     * Re-send ARP request:
		     */
		    capture_fdc_msg_printf("Outstanding ARP request re-sent for IP address: %u%u%u%u\n",
			                   dest_ip_addr_p->bytes[0],
			                   dest_ip_addr_p->bytes[1],
			                   dest_ip_addr_p->bytes[2],
			                   dest_ip_addr_p->bytes[3]);

		    send_arp_request = true;
		}
	    }

	    break;
	}

	if (first_free_entry_p == NULL) {
	    /*
	     * If we have not found a free entry, keep track of the
	     * least recently used entry
	     */
	    if (least_recently_used_entry_p == NULL) {
		least_recently_used_entry_p = entry_p;
		least_recently_used_ticks_delta =
		    RTOS_TICKS_DELTA(entry_p->last_lookup_time_stamp,
				     current_ticks);
	    } else if (RTOS_TICKS_DELTA(entry_p->last_lookup_time_stamp, current_ticks) >
		       least_recently_used_ticks_delta) {
		least_recently_used_entry_p = entry_p;
		least_recently_used_ticks_delta =
		    RTOS_TICKS_DELTA(entry_p->last_lookup_time_stamp,
				     current_ticks);
	    }
	}
    }

    if (i == ARP_CACHE_NUM_ENTRIES) {
	/*
	 * Allocate a free entry or recycle the least recently used entry in the
	 * ARP cache, and send ARP request:
	 */
	if (first_free_entry_p != NULL) {
	    chosen_entry_p = first_free_entry_p;
	} else {
	    FDC_ASSERT(least_recently_used_entry_p != NULL, 0, 0);
	    chosen_entry_p = least_recently_used_entry_p;
	}

	chosen_entry_p->last_lookup_time_stamp = rtos_k_get_ticks();
	send_arp_request = true;
    }

    /*
     * Send ARP request if necessary:
     */
    if (send_arp_request) {
	net_send_arp_request(local_l3_end_point_p->enet_device_p,
			     &local_l3_end_point_p->ipv4.local_ip_addr,
			     dest_ip_addr_p);

	chosen_entry_p->arp_request_time_stamp = rtos_k_get_ticks();
	chosen_entry_p->state = ARP_ENTRY_ARP_REQUEST_SENT;
    }

    FDC_ASSERT(chosen_entry_p != NULL, 0, 0);
    *arp_entry_pp = chosen_entry_p;
    return dest_mac_addr_found;
}

/**
 * Sends an IPv4 packet over Ethernet
 */
void
net_send_ipv4_packet(struct local_l3_end_point *local_l3_end_point_p,
		     const struct ipv4_address *dest_ip_addr_p,
		     struct network_packet *tx_packet_p,
		     size_t data_payload_length,
		     enum l4_protocols l4_protocol)
{
    struct arp_cache_entry *arp_entry_p = NULL;
    bool arp_cache_hit;

    FDC_ASSERT(tx_packet_p->signature == ENET_TX_PACKET_SIGNATURE,
	       tx_packet_p->signature, tx_packet_p);
    FDC_ASSERT(data_payload_length < UINT16_MAX + sizeof(struct ipv4_header),
	       data_payload_length, tx_packet_p);

    struct ethernet_frame *enet_frame =
       (struct ethernet_frame *)tx_packet_p->data_buffer;

   const struct enet_device *enet_device_p = local_l3_end_point_p->enet_device_p;

   FDC_ASSERT(enet_device_p->signature == ENET_DEVICE_SIGNATURE,
	      enet_device_p->signature, enet_device_p);

    /*
     * Populate IP header
     */
    enet_frame->ipv4_header.version_and_header_length = 0;
    SET_BIT_FIELD(enet_frame->ipv4_header.version_and_header_length,
		  IP_VERSION_MASK, IP_VERSION_SHIFT, 4);
    SET_BIT_FIELD(enet_frame->ipv4_header.version_and_header_length,
		  IP_HEADER_LENGTH_MASK, IP_HEADER_LENGTH_SHIFT, 5);

    enet_frame->ipv4_header.type_of_service = 0; /* normal service */
    enet_frame->ipv4_header.total_length =
	hton16(sizeof(struct ipv4_header) + data_payload_length);

    enet_frame->ipv4_header.identification = hton16(
	ATOMIC_POST_INCREMENT_UINT16(
	    &local_l3_end_point_p->ipv4.next_tx_ip_packet_seq_num));

    /*
     * No IP packet fragmentation is supported:
     */
    enet_frame->ipv4_header.flags_and_fragment_offset = 0;

    enet_frame->ipv4_header.time_to_live = 64; /* max routing hops */
    enet_frame->ipv4_header.protocol_type = l4_protocol;

    /*
     * NOTE: enet_frame->ipv4_header.header_checksum is filled by hardware
     */

    COPY_IPv4_ADDRESS(&enet_frame->ipv4_header.source_ip_addr,
		      &local_l3_end_point_p->ipv4.local_ip_addr);
    COPY_IPv4_ADDRESS(&enet_frame->ipv4_header.dest_ip_addr,
		      dest_ip_addr_p);

   /*
    * Populate Ethernet header
    */

#if 0 // Hw does this
    ENET_COPY_MAC_ADDRESS(&enet_frame->enet_header.source_mac_addr,
			  &enet_device_p->mac_address);
#endif

    enet_frame->enet_header.frame_type = hton16(ENET_IPv4_PACKET);

    tx_packet_p->total_length = sizeof(struct ethernet_header) +
				sizeof(struct ipv4_header) +
				data_payload_length;

    /*
     * Get destination MAC address:
     */
    if (SAME_IPv4_SUBNET(&local_l3_end_point_p->ipv4.local_ip_addr,
		         dest_ip_addr_p,
			 local_l3_end_point_p->ipv4.subnet_mask)) {
	arp_cache_hit = arp_cache_lookup(local_l3_end_point_p,
					 dest_ip_addr_p,
		                         &arp_entry_p);
    } else {
	arp_cache_hit = arp_cache_lookup(local_l3_end_point_p,
					 &local_l3_end_point_p->
					     ipv4.default_gateway_ip_addr,
		                         &arp_entry_p);
    }

    if (!arp_cache_hit) {
	rtos_mutex_acquire(&arp_entry_p->pending_tx_packet_list_mutex);
	glist_add_tail_elem(&arp_entry_p->pending_tx_packet_list_anchor,
			    &tx_packet_p->node);
	rtos_mutex_release(&arp_entry_p->pending_tx_packet_list_mutex);
	return;
    }

    COPY_MAC_ADDRESS(&enet_frame->enet_header.dest_mac_addr,
		     &arp_entry_p->dest_mac_addr);

    enet_start_xmit(enet_device_p, tx_packet_p);
}


/**
 * Chooses the local network end-point to be used for sending a packet,
 * based on the destination IPv4 address
 */
static struct local_l3_end_point *
choose_ipv4_local_l3_end_point(const struct ipv4_address *dest_ip_addr_p)
{
    struct local_l3_end_point *chosen_l3_end_point_p = NULL;

    /*
     * Find local network endpoint that is in the same IP subnet as
     * the destination IP address. If none, then just choose the first
     * local network end point.
     */
    for (unsigned int i = 0; i < ARRAY_SIZE(g_local_l3_end_points); i ++) {
	struct local_l3_end_point *l3_end_point_p = &g_local_l3_end_points[i];

	if (SAME_IPv4_SUBNET(&l3_end_point_p->ipv4.local_ip_addr,
		             dest_ip_addr_p,
			     l3_end_point_p->ipv4.subnet_mask)) {
	    chosen_l3_end_point_p = l3_end_point_p;
	    break;
	}
    }

    if (chosen_l3_end_point_p == NULL) {
	chosen_l3_end_point_p = &g_local_l3_end_points[0];
    }

    return chosen_l3_end_point_p;
}


void
net_send_ipv4_udp_packet(struct local_l4_end_point *local_l4_end_point_p,
			 struct local_l3_end_point *local_l3_end_point_p,
		         const struct ipv4_address *dest_ip_addr_p,
			 uint16_t dest_port,
		         struct network_packet *tx_packet_p,
		         size_t data_payload_length)
{
    /*
     * Populate UPD header:
     */
    struct udp_header *udp_header_p =
	(struct udp_header *)GET_IPV4_DATA_PAYLOAD_AREA(tx_packet_p);

    FDC_ASSERT(local_l4_end_point_p->protocol == TRANSPORT_PROTO_UDP,
	       local_l4_end_point_p->protocol, local_l4_end_point_p);

    udp_header_p->source_port = local_l4_end_point_p->port;
    udp_header_p->dest_port = dest_port;
    udp_header_p->datagram_length = hton16(sizeof(struct udp_header) +
	                                   data_payload_length);

    /*
     * NOTE: udp_header_p->datagram_checksum is filled by hardware
     */

    /*
     * Send IP packet:
     */
    net_send_ipv4_packet(local_l3_end_point_p,
		         dest_ip_addr_p,
		         tx_packet_p,
		         sizeof(struct udp_header) + data_payload_length,
		         TRANSPORT_PROTO_UDP);
}


void
net_send_ipv4_icmp_message(struct local_l3_end_point *local_l3_end_point_p,
			   const struct ipv4_address *dest_ip_addr_p,
		           struct network_packet *tx_packet_p,
			   uint8_t msg_type,
		           uint8_t msg_code,
		           size_t data_payload_length)
{
    /*
     * Populate ICMP header:
     */
    struct ipv4_icmp_header *icmp_header_p =
	(struct ipv4_icmp_header *)GET_IPV4_DATA_PAYLOAD_AREA(tx_packet_p);

    icmp_header_p->msg_type = msg_type;
    icmp_header_p->msg_code = msg_code;

    /*
     * NOTE: icmp_header_p->msg_checksum is filled by hardware
     */

    /*
     * Send IP packet:
     */
    net_send_ipv4_packet(local_l3_end_point_p,
		         dest_ip_addr_p,
		         tx_packet_p,
		         sizeof(struct ipv4_icmp_header) + data_payload_length,
		         TRANSPORT_PROTO_ICMP);
}


void
net_send_ipv4_ping_request(const struct ipv4_address *dest_ip_addr_p)
{
    struct local_l3_end_point *local_l3_end_point_p =
	choose_ipv4_local_l3_end_point(dest_ip_addr_p);

    FDC_ASSERT(local_l3_end_point_p->signature == LOCAL_L3_END_POINT_SIGNATURE,
	       local_l3_end_point_p->signature, local_l3_end_point_p);

    const struct enet_device *enet_device_p = local_l3_end_point_p->enet_device_p;

    struct network_packet *tx_packet_p =
	enet_allocate_tx_packet(local_l3_end_point_p->enet_device_p, true);

    net_send_ipv4_icmp_message(local_l3_end_point_p,
			       dest_ip_addr_p,
			       tx_packet_p,
			       ICMP_TYPE_PING_REQUEST,
		               ICMP_CODE_PING_REQUEST,
		               0);
}


static void
net_receive_arp_packet(struct network_packet *rx_packet_p)
{

    FDC_ASSERT(rx_packet_p->total_length >=
	       sizeof(struct ethernet_header) + sizeof(struct arp_packet),
	       rx_packet_p->total_length, rx_packet_p);

    struct ethernet_frame *rx_frame =
	(struct ethernet_frame *)rx_packet_p->data_buffer;

    uint16_t arp_operation = ntoh16(rx_frame->arp_packet.operation);

    //???
    CONSOLE_POS_PRINTF(31,1,
		"Received ARP packet: "
		"operation: %x "
		"source mac addr: %x:%x:%x:%x:%x:%x "
		"dest mac addr: %x:%x:%x:%x:%x:%x\n",
		arp_operation,
		rx_frame->arp_packet.source_mac_addr.bytes[0],
		rx_frame->arp_packet.source_mac_addr.bytes[1],
		rx_frame->arp_packet.source_mac_addr.bytes[2],
		rx_frame->arp_packet.source_mac_addr.bytes[3],
		rx_frame->arp_packet.source_mac_addr.bytes[4],
		rx_frame->arp_packet.source_mac_addr.bytes[5],
		rx_frame->arp_packet.dest_mac_addr.bytes[0],
		rx_frame->arp_packet.dest_mac_addr.bytes[1],
		rx_frame->arp_packet.dest_mac_addr.bytes[2],
		rx_frame->arp_packet.dest_mac_addr.bytes[3],
		rx_frame->arp_packet.dest_mac_addr.bytes[4],
		rx_frame->arp_packet.dest_mac_addr.bytes[5]);
    //???

    switch(arp_operation) {
	case ARP_REQUEST:
	    break;
	case ARP_REPLY:
#if 0 //???
	    apr_replies_received_count++;
	    CONSOLE_POS_PRINTF(30, 1, "APR replies received %8u", apr_replies_received_count);
#endif
	    break;
	default:
	    capture_fdc_msg_printf("Recieved ARP packet with unsupported operation (%#x)\n",
		                   arp_operation);

    }
}


static void
net_receive_ipv4_packet(struct network_packet *rx_packet_p)
{

    FDC_ASSERT(rx_packet_p->total_length >=
	       sizeof(struct ethernet_header) + sizeof(struct ipv4_header),
	       rx_packet_p->total_length, rx_packet_p);

#if 0 //???
    struct ethernet_frame *rx_frame =
	(struct ethernet_frame *)rx_packet_p->data_buffer;
#endif //???
}


static void
net_receive_ipv6_packet(struct network_packet *rx_packet_p)
{

    FDC_ASSERT(rx_packet_p->total_length >=
	       sizeof(struct ethernet_header) + sizeof(struct ipv6_header),
	       rx_packet_p->total_length, rx_packet_p);

#if 0 //???
    struct ethernet_frame *rx_frame =
	(struct ethernet_frame *)rx_packet_p->data_buffer;
#endif //???
}


/**
 * Packet receive processing thread for a given Ethernet interface
 */
static fdc_error_t
net_receive_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct local_l3_end_point *local_l3_end_point_p =
	(struct local_l3_end_point *)arg;

    rtos_enter_privileged_mode();

    FDC_ASSERT(local_l3_end_point_p->signature == LOCAL_L3_END_POINT_SIGNATURE,
	       local_l3_end_point_p->signature, local_l3_end_point_p);

    const struct enet_device *enet_device_p = local_l3_end_point_p->enet_device_p;

    FDC_ASSERT(enet_device_p->signature == ENET_DEVICE_SIGNATURE,
	       enet_device_p->signature, enet_device_p);

    /*
     * Send gratuitous ARP request (to catch if someone else is using the same
     * IP address):
     */
    net_send_arp_request(enet_device_p,
		         &local_l3_end_point_p->ipv4.local_ip_addr,
		         &local_l3_end_point_p->ipv4.local_ip_addr);

    for ( ; ; ) {
	struct network_packet *rx_packet_p = NULL;

	enet_dequeue_rx_packet(enet_device_p, &rx_packet_p);
	FDC_ASSERT(rx_packet_p != NULL, enet_device_p, cpu_id);

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
	}

	enet_recycle_rx_packet(enet_device_p, rx_packet_p);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    rtos_exit_privileged_mode();
    return fdc_error;
}

