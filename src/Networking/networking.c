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

/**
 * Ethernet link speed: 100 Mbps
 */
#define ETHERNET_LINK_SPEED_IN_BPS   UINT32_C(100000000)


void
networking_init(void)
{
#if 0 //???
    ip_addr_t netif0_ipaddr;
    ip_addr_t netif0_netmask;
    ip_addr_t netif0_gw;
    struct netif *netif_p;
    fdc_error_t fdc_error;


    IP4_ADDR(&netif0_ipaddr, 192,168,2,102);
    IP4_ADDR(&netif0_netmask, 255,255,255,0);
    IP4_ADDR(&netif0_gw, 192,168,2,100);
#endif
}

