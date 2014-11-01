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

/**
 * Send an ARP request message
 */
void
net_apr_send_request(const struct enet_device *enet_device_p,
		     const struct ipv4_address *source_ip_addr_p,
		     const struct ipv4_address *dest_ip_addr_p)
{
    struct ethernet_frame *enet_frame_buf = enet_allocate_tx_buffer(enet_device_p);

    DBG_ASSERT(enet_frame_buf != NULL, enet_device_p, 0);

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

    enet_start_xmit(enet_device_p, enet_frame_buf,
		    sizeof(struct ethernet_header) + sizeof(struct arp_packet));
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


/**
 * Packet receive processing thread for a given Ethernet interface
 */
static fdc_error_t
net_receive_thread_f(void *arg)
{
    static const struct ipv4_address local_ipaddr = { .bytes = { 192, 168, 8, 2 } };
    static const struct ipv4_address dest_ipaddr = { .bytes = { 192, 168, 8, 1 } };

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
    net_apr_send_request(enet_device_p, &local_ipaddr, &local_ipaddr);

    uint32_t apr_reqs_sent_count  = 0; //???
    uint32_t apr_replies_received_count  = 0; //???
    for ( ; ; ) {
	struct ethernet_frame *rx_frame_buf = NULL;
	size_t rx_frame_length;

	net_apr_send_request(enet_device_p, &local_ipaddr, &dest_ipaddr); //???
	//???
	apr_reqs_sent_count++;
	CONSOLE_POS_PRINTF(29, 1, "APR requests sent %8u", apr_reqs_sent_count);
	//???
	enet_dequeue_rx_buffer(enet_device_p, (void **)&rx_frame_buf,
			       &rx_frame_length);
	FDC_ASSERT(rx_frame_buf != NULL, enet_device_p, cpu_id);
	//???
	switch (ntoh16(rx_frame_buf->enet_header.frame_type)) {
	case ENET_ARP_PACKET:
	    apr_replies_received_count++;
	    CONSOLE_POS_PRINTF(30, 1, "APR replies received %8u", apr_replies_received_count);
	    CONSOLE_POS_PRINTF(31,1,
		"Received ARP packet: "
		"operation: %x "
		"source mac addr: %x:%x:%x:%x:%x:%x "
		"dest mac addr: %x:%x:%x:%x:%x:%x\n",
		ntoh16(rx_frame_buf->arp_packet.operation),
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
	    break;
	default:
	    CONSOLE_POS_PRINTF(31,1,
		"Received frame of type %#x                                                       \n",
		ntoh16(rx_frame_buf->enet_header.frame_type));
	}
	//???

	enet_recycle_rx_buffer(enet_device_p, rx_frame_buf);
	rtos_thread_delay(500); //????
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    rtos_exit_privileged_mode();
    return fdc_error;
}

