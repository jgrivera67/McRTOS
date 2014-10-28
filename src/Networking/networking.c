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
    enet_frame_buf->arp_packet.network_addr_type = hton16(ENET_IP_PACKET);
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

//???
static void
poll_rx_frames(const struct enet_device *enet_device_p)
{
    DBG_ASSERT(
        enet_device_p->signature == ENET_DEVICE_SIGNATURE,
        enet_device_p->signature, enet_device_p);

    struct enet_device_var *const enet_var_p = enet_device_p->var_p;

    volatile ENET_Type *enet_regs_p = enet_device_p->mmio_registers_p;

    uint32_t reg_value = read_32bit_mmio_register(&enet_regs_p->RDAR);

    //FDC_ASSERT(reg_value & ENET_RDAR_RDAR_MASK, reg_value, 0);

	for (unsigned int i = 0; i < ENET_MAX_RX_FRAME_BUFFERS; i ++) {
	    volatile struct enet_rx_buffer_descriptor *buffer_desc_p =
		&enet_var_p->rx_buffer_descriptors[i];

	    if (i == ENET_MAX_RX_FRAME_BUFFERS - 1) {
		FDC_ASSERT(buffer_desc_p->control & ENET_RX_BD_WRAP_MASK,
		           buffer_desc_p->control, buffer_desc_p);
	    }

	    if (buffer_desc_p->control & ENET_RX_BD_EMPTY_MASK) {
		continue;
	    }

	    FDC_ASSERT(buffer_desc_p->control & ENET_RX_BD_LAST_IN_FRAME_MASK,
		       buffer_desc_p->control, buffer_desc_p);

	    struct enet_frame_buffer *frame_buf_p =
		TO_FRAME_BUFFER(buffer_desc_p->data_buffer);

	    DBG_ASSERT(frame_buf_p->signature == ENET_RX_BUFFER_SIGNATURE,
		       frame_buf_p->signature, frame_buf_p);

	    if (!frame_buf_p->in_transit) {
		continue;
	    }

	    buffer_desc_p->control_extend1 &= ~ENET_RX_BD_GENERATE_INTERRUPT_MASK;
	    if (buffer_desc_p->control &
	        (ENET_RX_BD_LENGTH_VIOLATION_MASK |
		 ENET_RX_BD_NON_OCTET_ALIGNED_FRAME_MASK |
		 ENET_RX_BD_CRC_ERROR_MASK |
		 ENET_RX_BD_FIFO_OVERRRUN_MASK |
		 ENET_RX_BD_FRAME_TRUNCATED_MASK)) {
		(void)CAPTURE_FDC_ERROR("Received bad frame (Rx packet dropped)",
					buffer_desc_p->control, buffer_desc_p);
		continue;
	    }

            debugger_printf("*** Received frame: %#p\n", buffer_desc_p->data_buffer);
	}
}
//???

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
    for ( ; ; ) {
	struct ethernet_frame *enet_rx_frame_buf = NULL;
	size_t rx_frame_length;

	net_apr_send_request(enet_device_p, &local_ipaddr, &dest_ipaddr); //???
	//???
	apr_reqs_sent_count++;
	CONSOLE_POS_PRINTF(29, 60, "APR requests sent %8u", apr_reqs_sent_count);
	poll_rx_frames(enet_device_p);
	//???
#if 0
	enet_dequeue_rx_buffer(enet_device_p, (void **)&enet_rx_frame_buf,
			       &rx_frame_length);
	FDC_ASSERT(enet_rx_frame_buf != NULL, enet_device_p, cpu_id);
#endif

	rtos_thread_delay(500); //????
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    rtos_exit_privileged_mode();
    return fdc_error;
}

