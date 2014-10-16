/**
 * @file networking.h
 *
 * Copyright (C) 2014 German Rivera
 *
 * @author German Rivera
 */
#ifndef _NETWORKING_H
#define _NETWORKING_H

#include <McRTOS/failure_data_capture.h>

/**
 * Maximum transfer unit for Ethernet (frame size without CRC)
 */
#define NETWORK_MTU 1500

void networking_init(void);

void ping_init(void);

extern struct netif g_netif0;

#endif /* _NETWORKING_H */
