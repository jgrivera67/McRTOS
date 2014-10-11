/*
 * Copyright (c) 2013 - 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __SYS_ARCH_H__
#define __SYS_ARCH_H__

#include <McRTOS_kernel_services.h>

#define LWIP_COMPAT_MUTEX   0

/**
 * Mailbox object
 */
struct sys_mbox {
    /**
     * Circular buffer of pointers used to represent the queue of a mailbox
     */
    struct rtos_circular_buffer mbox_queue;

    /**
     * Array of entries for mbox_queue
     */
#   define MBOX_QUEUE_SIZE  8
    void *mbox_queue_entries[MBOX_QUEUE_SIZE];
};

typedef struct sys_mbox sys_mbox_t;

/**
 * Semaphore object
 */
struct sys_sem {
    /**
     * Counter for the semaphore
     */
    volatile uint8_t sem_count;

    /**
     * Condition variable for the semaphore
     */
    struct rtos_condvar sem_condvar;
};

typedef struct sys_sem sys_sem_t;

typedef struct rtos_mutex sys_mutex_t;

typedef struct rtos_thread *sys_thread_t;


#endif /* __SYS_ARCH_H__ */
