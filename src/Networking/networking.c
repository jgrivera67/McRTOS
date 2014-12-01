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
#pragma GCC diagnostic ignored "-Wunused-parameter" // ???

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

static const struct ethernet_mac_address enet_broadcast_mac_addr = {
    .bytes = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }
};

static const struct ethernet_mac_address enet_null_mac_addr = {
    .bytes = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

static struct networking g_networking = {
    .initialized = false,
    .next_udp_ephemeral_port = NET_FIRST_EPHEMERAL_PORT,
    .next_tcp_ephemeral_port = NET_FIRST_EPHEMERAL_PORT,
    .next_free_l4_end_point_p = &g_networking.local_l4_end_points[0],
    .local_l3_end_points = {
	[0] = {
	    .signature = LOCAL_L3_END_POINT_SIGNATURE,
	    .enet_device_p = &g_enet_device0,
	    .ipv4 = {
		.local_ip_addr = { .bytes = { 192, 168, 8, 1 + BOARD_INSTANCE } },
		.subnet_mask = IPv4_SUBNET_MASK(24),
		.default_gateway_ip_addr = { .bytes = { 192, 168, 8, 1 } },
		.next_tx_ip_packet_seq_num = 0,
	    },
	},
    },
};


static void
arp_cache_init(struct arp_cache *arp_cache_p)
{
    rtos_k_mutex_init(
	"ARP cache mutex",
	&arp_cache_p->mutex);

    rtos_k_condvar_init(
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

#if 0 // Hardware does this
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

#if 0 // Hardware does this
    COPY_MAC_ADDRESS(&enet_frame->enet_header.source_mac_addr,
		     &enet_device_p->mac_address);
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
		     &enet_device_p->mac_address);
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
    rtos_k_queue_init(
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

        rtos_k_queue_add(&g_networking.free_tx_packet_pool,
			 &tx_packet_p->node);
    }
}


static void
net_rx_packet_queue_init(struct local_l3_end_point *local_l3_end_point_p)
{
    rtos_k_queue_init(
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


/**
 * Initialize networking subsystem
 */
void
networking_init(void)
{
    struct rtos_thread_creation_params threads[] = {
	[0] = {
	    .p_name_p = "Network Packet Receive thread",
	    .p_function_p = net_receive_thread_f,
	    .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
	    .p_thread_pp = NULL,
	},

	[1] = {
	    .p_name_p = "ICMPv4 packet receive thread",
	    .p_function_p = net_icmpv4_receive_thread_f,
	    .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 2,
	    .p_thread_pp = NULL,
	},
    };

    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(!g_networking.initialized, 0, 0);

    /*
     * Initialize global Tx buffer pool:
     */
    net_tx_packet_pool_init();

    /*
     * Initialize local network end points:
     */
    for (unsigned int i = 0; i < ARRAY_SIZE(g_networking.local_l3_end_points); i ++) {
	struct local_l3_end_point *local_l3_end_point_p =
	    &g_networking.local_l3_end_points[i];

	net_rx_packet_queue_init(local_l3_end_point_p);

	rtos_k_queue_init(
	    "ICMPv4 incoming packet queue",
	    true,
	    &local_l3_end_point_p->ipv4.rx_icmpv4_packet_queue);

	/*
	 * Activate network interface (ENET device):
	 */
	enet_start(local_l3_end_point_p->enet_device_p, local_l3_end_point_p);

	/*
	 * Initialize ARP cache for each local network end point
	 */
	arp_cache_init(&local_l3_end_point_p->ipv4.arp_cache);
	//TODO: Init IPv6 neighbor cache

	/*
	 * Create threads for this layer-3 end point:
	 */
	for (unsigned int i = 0; i < ARRAY_SIZE(threads); i ++) {
	    threads[i].p_function_arg_p = (void *)local_l3_end_point_p;
	    fdc_error = rtos_k_create_thread(&threads[i]);
	    if (fdc_error != 0) {
		console_printf(
		    "CPU core %u: *** Error creating application thread '%s' ***\n",
		    cpu_id, threads[i].p_name_p);

		fatal_error_handler(fdc_error);
	    }

	    console_printf("CPU core %u: %s started\n", cpu_id, threads[i].p_name_p);
	}
    }

    rtos_k_queue_init(
	"Incoming IPv4 ping reply packet queue",
	true,
	&g_networking.rx_ipv4_ping_reply_packet_queue);

    rtos_k_mutex_init("local_l4_end_points mutex", &g_networking.local_l4_end_points_mutex);
    rtos_k_mutex_init("expecting_ping_reply mutex", &g_networking.expecting_ping_reply_mutex);
    rtos_k_condvar_init("ping_reply_recceived condvar", &g_networking.ping_reply_recceived_condvar);
    g_networking.initialized = true;
}


/**
 * Allocates a Tx packet from the global Tx packet pool
 */
struct network_packet *
net_allocate_tx_packet(bool free_after_tx_complete)
{
    struct network_packet *tx_packet_p = NULL;

    FDC_ASSERT(g_networking.initialized, 0, 0);

    tx_packet_p = GLIST_NODE_TO_NETWORK_PACKET(
		     rtos_k_queue_remove(&g_networking.free_tx_packet_pool, 0));

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

    return tx_packet_p;
}


/**
 * Frees a Tx packet back to the global Tx packet pool
 */
void
net_free_tx_packet(struct network_packet *tx_packet_p)
{
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
    rtos_k_queue_add(&g_networking.free_tx_packet_pool, &tx_packet_p->node);
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
		    rtos_k_queue_remove(&local_l3_end_point_p->rx_packet_queue, 0));

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
    rtos_k_queue_add(&local_l3_end_point_p->rx_packet_queue, &rx_packet_p->node);
}


/**
 * Recycle a Rx packet for receiving another packet
 */
void
net_recycle_rx_packet(struct network_packet *rx_packet_p)
{
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
	rtos_ticks_t current_ticks = rtos_k_get_ticks();

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
find_dest_mac_addr(struct local_l3_end_point *local_l3_end_point_p,
		   const struct ipv4_address *dest_ip_addr_p,
		   struct ethernet_mac_address *dest_mac_addr_p)
{
    unsigned int arp_request_retries = 0;
    struct arp_cache_entry *matching_entry_p = NULL;
    struct arp_cache_entry *free_entry_p = NULL;
    struct arp_cache *arp_cache_p = &local_l3_end_point_p->ipv4.arp_cache;
    fdc_error_t fdc_error;

    rtos_k_mutex_acquire(&arp_cache_p->mutex);
    for ( ; ; ) {
	bool send_arp_request = false;
	matching_entry_p = arp_cache_lookup_or_allocate(arp_cache_p, dest_ip_addr_p,
							&free_entry_p);
	rtos_ticks_t current_ticks = rtos_k_get_ticks();

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
		matching_entry_p->arp_request_time_stamp = rtos_k_get_ticks();
		matching_entry_p->state = ARP_ENTRY_HALF_FILLED;
	    } else {
		FDC_ASSERT(free_entry_p != NULL, dest_ip_addr_p, arp_cache_p);
		free_entry_p->dest_ip_addr.value = dest_ip_addr_p->value;
		free_entry_p->arp_request_time_stamp = rtos_k_get_ticks();
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
    rtos_k_mutex_release(&arp_cache_p->mutex);
    return fdc_error;
}


static void
arp_cache_update(struct arp_cache *arp_cache_p,
		 const struct ipv4_address *dest_ip_addr_p,
		 struct ethernet_mac_address *dest_mac_addr_p)
{
    struct arp_cache_entry *chosen_entry_p = NULL;
    struct arp_cache_entry *free_entry_p = NULL;

    rtos_k_mutex_acquire(&arp_cache_p->mutex);
    chosen_entry_p = arp_cache_lookup_or_allocate(arp_cache_p, dest_ip_addr_p,
					          &free_entry_p);

    if (chosen_entry_p == NULL) {
	FDC_ASSERT(free_entry_p != NULL, dest_ip_addr_p, arp_cache_p);
	chosen_entry_p = free_entry_p;
	chosen_entry_p->dest_ip_addr.value = dest_ip_addr_p->value;
    }

    COPY_MAC_ADDRESS(&chosen_entry_p->dest_mac_addr, dest_mac_addr_p);
    chosen_entry_p->state = ARP_ENTRY_FILLED;
    chosen_entry_p->entry_filled_time_stamp = rtos_k_get_ticks();
    rtos_k_mutex_release(&arp_cache_p->mutex);
    rtos_k_condvar_signal(&arp_cache_p->cache_updated_condvar);
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
    for (unsigned int i = 0; i < ARRAY_SIZE(g_networking.local_l3_end_points); i ++) {
	struct local_l3_end_point *l3_end_point_p = &g_networking.local_l3_end_points[i];

	if (SAME_IPv4_SUBNET(&l3_end_point_p->ipv4.local_ip_addr,
		             dest_ip_addr_p,
			     l3_end_point_p->ipv4.subnet_mask)) {
	    chosen_l3_end_point_p = l3_end_point_p;
	    break;
	}
    }

    if (chosen_l3_end_point_p == NULL) {
	chosen_l3_end_point_p = &g_networking.local_l3_end_points[0];
    }

    return chosen_l3_end_point_p;
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
    FDC_ASSERT(data_payload_length < UINT16_MAX + sizeof(struct ipv4_header),
	       data_payload_length, tx_packet_p);

    struct ethernet_frame *tx_frame_p =
       (struct ethernet_frame *)tx_packet_p->data_buffer;

   struct local_l3_end_point *local_l3_end_point_p =
	   choose_ipv4_local_l3_end_point(dest_ip_addr_p);

   const struct enet_device *enet_device_p = local_l3_end_point_p->enet_device_p;

   FDC_ASSERT(enet_device_p->signature == ENET_DEVICE_SIGNATURE,
	      enet_device_p->signature, enet_device_p);

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
    if (SAME_IPv4_SUBNET(&local_l3_end_point_p->ipv4.local_ip_addr,
		         dest_ip_addr_p,
			 local_l3_end_point_p->ipv4.subnet_mask)) {
	 fdc_error = find_dest_mac_addr(local_l3_end_point_p,
					dest_ip_addr_p,
					&dest_mac_addr);
    } else {
	 fdc_error = find_dest_mac_addr(local_l3_end_point_p,
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

    rtos_k_mutex_acquire(&g_networking.local_l4_end_points_mutex);
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
    rtos_k_mutex_release(&g_networking.local_l4_end_points_mutex);

    l4_end_point_p->l4_protocol = l4_protocol;
    l4_end_point_p->l4_port = l4_port;
    rtos_k_queue_init("l4_end_point Rx packet queue", true,
	              &l4_end_point_p->l4_rx_packet_queue);

    *local_l4_end_point_pp = l4_end_point_p;
    return 0;

error_release_mutex:
    rtos_k_mutex_release(&g_networking.local_l4_end_points_mutex);
    return fdc_error;
}


void
net_send_ipv4_udp_datagram(struct local_l4_end_point *local_l4_end_point_p,
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

    FDC_ASSERT(local_l4_end_point_p->l4_protocol == TRANSPORT_PROTO_UDP,
	       local_l4_end_point_p->l4_protocol, local_l4_end_point_p);

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
    net_send_ipv4_packet(dest_ip_addr_p,
		         tx_packet_p,
		         sizeof(struct udp_header) + data_payload_length,
		         TRANSPORT_PROTO_UDP);
}


void
net_receive_ipv4_udp_datagram(struct local_l4_end_point *local_l4_end_point_p,
			      rtos_milliseconds_t timeout_ms,
		              struct ipv4_address *source_ip_addr_p,
			      uint16_t *source_port_p,
			      struct network_packet **rx_packet_pp)
{
    FDC_ASSERT(local_l4_end_point_p->l4_protocol == TRANSPORT_PROTO_UDP,
	       local_l4_end_point_p->l4_protocol, local_l4_end_point_p);

    struct glist_node *rx_packet_node_p =
	    rtos_k_queue_remove(&local_l4_end_point_p->l4_rx_packet_queue,
			        timeout_ms);

    if (rx_packet_node_p == NULL) {
	*rx_packet_pp = NULL;
	return;
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


void
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
     * NOTE: icmp_header_p->msg_checksum is computed by hardware
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
    net_send_ipv4_packet(dest_ip_addr_p,
		         tx_packet_p,
		         sizeof(struct icmpv4_header) + data_payload_length,
		         TRANSPORT_PROTO_ICMP);
}


void
net_send_ipv4_ping_request(const struct ipv4_address *dest_ip_addr_p,
			   uint16_t identifier,
	                   uint16_t seq_num)
{
    rtos_k_mutex_acquire(&g_networking.expecting_ping_reply_mutex);
    while (g_networking.expecting_ping_reply) {
	rtos_k_condvar_wait(&g_networking.ping_reply_recceived_condvar,
			    &g_networking.expecting_ping_reply_mutex,
			    NULL);
    }

    g_networking.expecting_ping_reply = true;
    rtos_k_mutex_release(&g_networking.expecting_ping_reply_mutex);

    struct network_packet *tx_packet_p = net_allocate_tx_packet(true);
    struct icmpv4_echo_message *echo_msg_p =
	(struct icmpv4_echo_message *)GET_IPV4_DATA_PAYLOAD_AREA(tx_packet_p);

    echo_msg_p->identifier = identifier;
    echo_msg_p->seq_num = seq_num;
    net_send_ipv4_icmp_message(dest_ip_addr_p,
			       tx_packet_p,
			       ICMP_TYPE_PING_REQUEST,
		               ICMP_CODE_PING_REQUEST,
		               sizeof(struct icmpv4_echo_message) -
			       sizeof(struct icmpv4_header));
}


bool
net_receive_ipv4_ping_reply(rtos_milliseconds_t timeout_ms,
			    struct ipv4_address *remote_ip_addr_p,
			    uint16_t *identifier_p,
			    uint16_t *seq_num_p)
{
    struct glist_node *rx_packet_node_p =
	rtos_k_queue_remove(&g_networking.rx_ipv4_ping_reply_packet_queue, timeout_ms);

    if (rx_packet_node_p == NULL) {
	return false;
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
    return true;
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

	/*
	 * Update ARP cache with (source IP addr, source MAC addr)
	 */
	COPY_UNALIGNED_IPv4_ADDRESS(&source_ip_addr,
				    &rx_frame_p->arp_packet.source_ip_addr);
	arp_cache_update(&local_l3_end_point_p->ipv4.arp_cache,
			 &source_ip_addr,
			 &rx_frame_p->arp_packet.source_mac_addr);
	break;

    case ARP_REPLY:
	FDC_ASSERT(
	    UNALIGNED_IPv4_ADDRESSES_EQUAL(&rx_frame_p->arp_packet.dest_ip_addr,
					   &local_l3_end_point_p->ipv4.local_ip_addr),
	    &rx_frame_p->arp_packet.dest_ip_addr,
	    &local_l3_end_point_p->ipv4.local_ip_addr);

	FDC_ASSERT(
	    MAC_ADDRESSES_EQUAL(&rx_frame_p->arp_packet.dest_mac_addr,
				&local_l3_end_point_p->enet_device_p->mac_address),
	    &rx_frame_p->arp_packet.dest_mac_addr,
	    &local_l3_end_point_p->enet_device_p->mac_address);

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

	rtos_k_mutex_acquire(&g_networking.expecting_ping_reply_mutex);
	if (g_networking.expecting_ping_reply) {
	    rtos_k_queue_add(&g_networking.rx_ipv4_ping_reply_packet_queue,
		             &rx_packet_p->node);
	    g_networking.expecting_ping_reply = false;
	    signal_ping_reply_received = true;
	} else {
	    /*
	     * Drop unmatched ping reply
	     */
	    net_recycle_rx_packet(rx_packet_p);
	}

	rtos_k_mutex_release(&g_networking.expecting_ping_reply_mutex);
	if (signal_ping_reply_received) {
	    rtos_k_condvar_signal(&g_networking.ping_reply_recceived_condvar);
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

    rtos_k_mutex_acquire(&g_networking.local_l4_end_points_mutex);
    for (struct local_l4_end_point *p = g_networking.local_l4_end_points;
	 p != g_networking.next_free_l4_end_point_p; p ++) {
	if (p->l4_protocol == TRANSPORT_PROTO_UDP &&
	    p->l4_port == udp_header_p->dest_port) {
	    local_l4_end_point_p = p;
	    break;
	}
    }

    rtos_k_mutex_release(&g_networking.local_l4_end_points_mutex);

    if (local_l4_end_point_p != NULL) {
	rtos_k_queue_add(&local_l4_end_point_p->l4_rx_packet_queue,
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
//???
    static int count = 0;

    count++;
    CONSOLE_POS_PRINTF(35,1, "Received IPv4 packets: %d\n", count);
//???

    struct local_l3_end_point *local_l3_end_point_p = rx_packet_p->local_l3_end_point_p;
    struct ipv4_header *ipv4_header_p = GET_IPV4_HEADER(rx_packet_p);

    switch (ipv4_header_p->protocol_type) {
    case TRANSPORT_PROTO_ICMP:
	rx_packet_p->state_flags |= NET_PACKET_IN_ICMP_QUEUE;
	rtos_k_queue_add(&local_l3_end_point_p->ipv4.rx_icmpv4_packet_queue,
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

//???
    static int count = 0;

    count++;
    CONSOLE_POS_PRINTF(35,60, "Received IPv6 packets: %d\n", count);
//???

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

	net_dequeue_rx_packet(local_l3_end_point_p, &rx_packet_p);

	DBG_ASSERT(rx_packet_p != NULL, enet_device_p, cpu_id);
	DBG_ASSERT(rx_packet_p->local_l3_end_point_p == local_l3_end_point_p,
		   rx_packet_p->local_l3_end_point_p, local_l3_end_point_p);

	if (rx_packet_p->state_flags == NET_PACKET_RX_FAILED) {
	    net_recycle_rx_packet(rx_packet_p);
	    continue;
	}

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

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    rtos_exit_privileged_mode();
    return fdc_error;
}


/**
 * ICMPv4 receive thread for a given Layer-3 end point
 */
static fdc_error_t
net_icmpv4_receive_thread_f(void *arg)
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

    for ( ; ; ) {
	struct network_packet *rx_packet_p = NULL;

	rx_packet_p = GLIST_NODE_TO_NETWORK_PACKET(
		rtos_k_queue_remove(&local_l3_end_point_p->ipv4.rx_icmpv4_packet_queue, 0));

	DBG_ASSERT(rx_packet_p->signature == NET_RX_PACKET_SIGNATURE &&
		   (rx_packet_p->state_flags & NET_PACKET_IN_ICMP_QUEUE),
		   rx_packet_p, local_l3_end_point_p);

	rx_packet_p->state_flags &= ~NET_PACKET_IN_ICMP_QUEUE;
	net_process_incoming_icmpv4_message(rx_packet_p);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    rtos_exit_privileged_mode();
    return fdc_error;
}

