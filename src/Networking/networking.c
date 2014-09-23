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
#include <lwip/tcpip.h>
#include <lwip/ipaddr.h>
#include <lwip/err.h>
#include <lwip/netif.h>
#if 0 //???
#include <lwip/udp.h>
#include <netif/etharp.h>
#include <lwip/init.h>
#endif

TODO("Remove these pragmas")
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-function"

/**
 * Ethernet link speed: 100 Mbps
 */
#define ETHERNET_LINK_SPEED_IN_BPS   UINT32_C(100000000)

static struct netif g_netif0;

static err_t
ethernet_output(struct netif *netif_p, struct pbuf *pbuf_p, struct ip_addr *ipaddr)
{
    /*
     * TODO: Check if link is up before transmitting the packet
     */

    return etharp_output(netif_p, pbuf_p, ipaddr_p);
}


static err_t
ethernet_link_output(struct netif *netif_p, struct pbuf *pbuf_p)
{
    uint8_t *tx_payload_buf;
    struct enet_device *enet_device_p = (struct enet_device *)netif_p->state;

    /*
     * TODO: Change this for zero-copy
     */
    tx_payload_buf = enet_alloc_tx_buffer(enet_device_p);

    FDC_ASSERT(tx_payload_buf != NULL, enet_device_p, netif_p);
    FDC_ASSERT(pbuf_p->tot_len <= NETWORK_MTU,
	       pbuf_p->tot_len, pbuf_p);
    FDC_ASSERT(pbuf_p->tot_len == pbuf_p->len,
	       pbuf_p->tot_len, pbuf_p->len);
    FDC_ASSERT(pbuf_p->next == NULL,
	       pbuf_p->next, pbuf_p);

    /*
     * TODO: remove this memory copying for Tx zero-copy
     */
    for (i = 0; i < pbuf_p->tot_len; i ++) {
	tx_payload_buf[i] = ((uint8_t *)pbuf_p->payload)[i];
    }

    enet_start_xmit(enet_device_p, tx_payload_buf);
    return 0;
}


static err_t
ethernet_netif_init(struct netif *netif)
{
    err_t result;

    DBG_ASSERT(netif != NULL, 0, 0);
    DBG_ASSERT(netif->state == &g_enet_device0,
		netif->state, netif);
    DBG_ASSERT(netif->init == ethernet_netif_init,
	       netif->init, netif);
    DBG_ASSERT(netif->input == tcpip_input,
	       netif->input, netif);

    NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, ETHERNET_LINK_SPEED_IN_BPS);
    netif->name[0] = 'e';
    netif->name[1] = 'n';
    netif->output = ethernet_output;
    netif->linkoutput = ethernet_link_output;
    return 0;
}


void
networking_init(void)
{
    ip_addr_t netif0_ipaddr;
    ip_addr_t netif0_netmask;
    ip_addr_t netif0_gw;
    struct netif *netif_p;
    fdc_error_t fdc_error;

    tcpip_init(NULL,NULL);

    IP4_ADDR(&netif0_ipaddr, 192,168,2,102);
    IP4_ADDR(&netif0_netmask, 255,255,255,0);
    IP4_ADDR(&netif0_gw, 192,168,2,100);

    netif_p = netif_add(&g_netif0,
			&netif0_ipaddr,
			&netif0_netmask,
		        &netif0_gw,
			&g_enet_device0,
			ethernet_netif_init,
			tcpip_input);

    if (netif_p != &g_netif0) {
	fdc_error = CAPTURE_FDC_ERROR("netif_add() failed", &g_netif0, 0);
        fatal_error_handler(fdc_error);
    }

    netif_set_default(&g_netif0);

    DEBUG_PRINTF("lwIP initialized\n");
}


fdc_error_t
ethernet_link_input(struct netif *netif_p)
{
    fdc_error_t fdc_error;
    err_t lwip_err;
    void *rx_payload_buf;
    size_t rx_payload_length;
    struct pbuf *pbuf_p;
    bool packet_dropped = false;
    struct enet_device *enet_device_p = (struct enet_device *)netif_p->state;

    enet_dequeue_rx_buffer(enet_device_p, &rx_payload_buf, &rx_payload_length);

    /*
     * TODO for zero-copy: use PBUF_REF, instead of PBUF_POOL
     */
    pbuf_p = pbuf_alloc(PBUF_RAW, rx_payload_length, PBUF_POOL);
    if (pbuf_p == NULL) {
	fdc_error =
            CAPTURE_FDC_ERROR("pbuf_alloc() failed (Rx packet dropped)",
			      netif_p, rx_payload_length);
	packet_dropped = true;
	goto error;
    }

    /*
     * TODO for zero-copy: remove this memory copying and set
     * pbuf_p->payload to poin to rx_payload_buf
     */
    for (i = 0; i < rx_payload_length; i ++) {
	((uint8_t *)pbuf_p->payload)[i] = rx_payload_buf[i];
    }

    pbuf_p->tot_len = rx_payload_length;

    /*
     * TODO for zero-copy: don't recycle here. Instead, have an API that
     * the application calls when it is done processing the received packet
     */
    enet_recycle_rx_buffer(enet_device_p, rx_payload_buf);

    /*
     * Pass received packet to the TCP/IP stack:
     */
    lwip_err = tcpip_input(pbuf_p, netif_p);
    if (lwip_err != 0) {
	    fdc_error =
            CAPTURE_FDC_ERROR("tcpip_input() failed (Rx packet dropped)",
			      netif_p, pbuf_p);

	    /*
	     * TODO: For zero-copy, set packet_dropped to true
	     */
	    goto error;
    }

    return 0;

error:
    if (packet_dropped) {
	enet_recycle_rx_buffer(enet_device_p, rx_payload_buf);
    }

    return fdc_error;
}


fdc_error_t
networking_receive_thread_f(void *arg)
{
    fdc_error_t fdc_error;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();

    FDC_ASSERT(arg != NULL, arg, cpu_id);

    struct netif *netif_p = (struct netif *)arg;

    rtos_enter_privileged_mode();
    for ( ; ; ) {
	ethernet_link_input(netif_p);
    }

    fdc_error = CAPTURE_FDC_ERROR(
        "thread should not have terminated",
        cpu_id, rtos_thread_self());

    rtos_exit_privileged_mode();
    return fdc_error;
}


