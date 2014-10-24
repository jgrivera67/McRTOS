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

struct transport_end_point {
	uint16_t port;
};

void networking_init(void);

fdc_error_t networking_receive_thread_f(void *arg);

#endif /* _NETWORKING_H */
