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

static const struct rtos_thread_creation_params net_threads[] = {
    [0] = {
	.p_name_p = "ENET Receive thread",
        .p_function_p = net_receive_thread_f,
        .p_function_arg_p = (void *)&g_enet_device0,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
        .p_thread_pp = NULL,
    },
};

static const struct ethernet_mac_address enet_broadcast_mac_addr = {
    .bytes = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};

static const struct ethernet_mac_address enet_null_mac_addr = {
    .bytes = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

static struct network_end_point g_local_network_end_point = {
    .local_ip_addr = { .bytes = { 192, 168, 8, 2 } },
    .subnet_mask = { .bytes = { 255, 255, 255, 0 } },
    .default_gateway_ip_addr = { .bytes = { 192, 168, 8, 1 } },
    .enet_device_p = &g_enet_device0
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

    struct ethernet_frame *enet_frame_buf =
	(struct ethernet_frame *)tx_packet_p->data_buffer;

#if 0 // Hw does this
    ENET_COPY_MAC_ADDRESS(&enet_frame_buf->enet_header.source_mac_addr,
			  &enet_device_p->mac_address);
#endif

    ENET_COPY_MAC_ADDRESS(&enet_frame_buf->enet_header.dest_mac_addr,
		          &enet_broadcast_mac_addr);
    enet_frame_buf->enet_header.frame_type = hton16(ENET_ARP_PACKET);
    enet_frame_buf->arp_packet.link_addr_type = hton16(0x1);
    enet_frame_buf->arp_packet.network_addr_type = hton16(ENET_IPv4_PACKET);
    enet_frame_buf->arp_packet.link_addr_size = sizeof(struct ethernet_mac_address);
    enet_frame_buf->arp_packet.network_addr_size = sizeof(struct ipv4_address);
    enet_frame_buf->arp_packet.operation = hton16(ARP_REQUEST);
    ENET_COPY_MAC_ADDRESS(&enet_frame_buf->arp_packet.source_mac_addr,
			  &enet_device_p->mac_address);
    ENET_COPY_IP_ADDRESS(&enet_frame_buf->arp_packet.source_ip_addr,
			 source_ip_addr_p);
    ENET_COPY_MAC_ADDRESS(&enet_frame_buf->arp_packet.dest_mac_addr,
			  &enet_null_mac_addr);
    ENET_COPY_IP_ADDRESS(&enet_frame_buf->arp_packet.dest_ip_addr,
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

    arp_cache_init(g_local_network_end_point.arp_cache);

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
arp_cache_lookup(struct network_end_point *network_end_point_p,
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
	struct arp_cache_entry *entry_p = &network_end_point_p->arp_cache[i];
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
	net_send_arp_request(network_end_point_p->enet_device_p,
			     &network_end_point_p->local_ip_addr,
			     dest_ip_addr_p);

	chosen_entry_p->arp_request_time_stamp = rtos_k_get_ticks();
	chosen_entry_p->state = ARP_ENTRY_ARP_REQUEST_SENT;
    }

    FDC_ASSERT(chosen_entry_p != NULL, 0, 0);
    *arp_entry_pp = chosen_entry_p;
    return dest_mac_addr_found;
}


void
net_send_ip_packet(struct network_end_point *network_end_point_p,
		   const struct ipv4_address *dest_ip_addr_p,
		   struct network_packet *tx_packet_p,
		   size_t data_payload_length)
{
    struct arp_cache_entry *arp_entry_p = NULL;

    FDC_ASSERT(tx_packet_p->signature == ENET_TX_PACKET_SIGNATURE,
	       tx_packet_p->signature, tx_packet_p);

    if (!arp_cache_lookup(network_end_point_p, dest_ip_addr_p, &arp_entry_p)) {
	rtos_mutex_acquire(&arp_entry_p->pending_tx_packet_list_mutex);
	glist_add_tail_elem(&arp_entry_p->pending_tx_packet_list_anchor,
		            &tx_packet_p->node);
	rtos_mutex_release(&arp_entry_p->pending_tx_packet_list_mutex);
	return;
    }

   struct ethernet_frame *enet_frame_buf =
       (struct ethernet_frame *)tx_packet_p->data_buffer;

   const struct enet_device *enet_device_p = network_end_point_p->enet_device_p;

   FDC_ASSERT(enet_device_p->signature == ENET_DEVICE_SIGNATURE,
	      enet_device_p->signature, enet_device_p);

#if 0 // Hw does this
    ENET_COPY_MAC_ADDRESS(&enet_frame_buf->enet_header.source_mac_addr,
			  &enet_device_p->mac_address);
#endif

    ENET_COPY_MAC_ADDRESS(&enet_frame_buf->enet_header.dest_mac_addr,
		          &arp_entry_p->dest_mac_addr);
    enet_frame_buf->enet_header.frame_type = hton16(ENET_IPv4_PACKET);
#if 0 //XXX TODO
    enet_frame_buf->ipv4_header.... = ...;
#endif
    ENET_COPY_IP_ADDRESS(&enet_frame_buf->ipv4_header.source_ip_addr,
			 &network_end_point_p->local_ip_addr);
    ENET_COPY_IP_ADDRESS(&enet_frame_buf->ipv4_header.dest_ip_addr,
		         dest_ip_addr_p);

    tx_packet_p->total_length = sizeof(struct ethernet_header) +
				sizeof(struct ipv4_header) +
				data_payload_length;

    enet_start_xmit(enet_device_p, tx_packet_p);
}

static void
net_receive_arp_packet(struct network_packet *rx_packet_p)
{

    FDC_ASSERT(rx_packet_p->total_length ==
	       sizeof(struct ethernet_header) + sizeof(struct arp_packet),
	       rx_packet_p->total_length, rx_packet_p);

    struct ethernet_frame *rx_frame_buf =
	(struct ethernet_frame *)rx_packet_p->data_buffer;

    uint16_t arp_operation = ntoh16(rx_frame_buf->arp_packet.operation);

    //???
    CONSOLE_POS_PRINTF(31,1,
		"Received ARP packet: "
		"operation: %x "
		"source mac addr: %x:%x:%x:%x:%x:%x "
		"dest mac addr: %x:%x:%x:%x:%x:%x\n",
		arp_operation,
		rx_frame_buf->arp_packet.source_mac_addr.bytes[0],
		rx_frame_buf->arp_packet.source_mac_addr.bytes[1],
		rx_frame_buf->arp_packet.source_mac_addr.bytes[2],
		rx_frame_buf->arp_packet.source_mac_addr.bytes[3],
		rx_frame_buf->arp_packet.source_mac_addr.bytes[4],
		rx_frame_buf->arp_packet.source_mac_addr.bytes[5],
		rx_frame_buf->arp_packet.dest_mac_addr.bytes[0],
		rx_frame_buf->arp_packet.dest_mac_addr.bytes[1],
		rx_frame_buf->arp_packet.dest_mac_addr.bytes[2],
		rx_frame_buf->arp_packet.dest_mac_addr.bytes[3],
		rx_frame_buf->arp_packet.dest_mac_addr.bytes[4],
		rx_frame_buf->arp_packet.dest_mac_addr.bytes[5]);
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


/**
 * Packet receive processing thread for a given Ethernet interface
 */
static fdc_error_t
net_receive_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    const struct enet_device *enet_device_p = (const struct enet_device *)arg;

    rtos_enter_privileged_mode();

    FDC_ASSERT(enet_device_p->signature == ENET_DEVICE_SIGNATURE,
	       enet_device_p->signature, enet_device_p);

    /*
     * Send gratuitous ARP request (to catch if someone else is using the same
     * IP address):
     */
    net_send_arp_request(enet_device_p,
		         &g_local_network_end_point.local_ip_addr,
		         &g_local_network_end_point.local_ip_addr);

    for ( ; ; ) {
	struct network_packet *rx_packet_p = NULL;

	enet_dequeue_rx_packet(enet_device_p, &rx_packet_p);
	FDC_ASSERT(rx_packet_p != NULL, enet_device_p, cpu_id);

	struct ethernet_frame *rx_frame_buf =
	    (struct ethernet_frame *)rx_packet_p->data_buffer;

	switch (ntoh16(rx_frame_buf->enet_header.frame_type)) {
	case ENET_ARP_PACKET:
	    net_receive_arp_packet(rx_packet_p);
	    break;
	default:
	    CONSOLE_POS_PRINTF(31,1,
		"Received frame of type %#x                                                       \n",
		ntoh16(rx_frame_buf->enet_header.frame_type));
	}

	enet_recycle_rx_packet(enet_device_p, rx_packet_p);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    rtos_exit_privileged_mode();
    return fdc_error;
}

