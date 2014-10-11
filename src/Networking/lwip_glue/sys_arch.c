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

#include "arch/sys_arch.h"
#include <lwip/sys.h>
#include <hardware_abstractions.h>

/*-----------------------------------------------------------------------------------*/
//  Creates an empty mailbox.
err_t sys_mbox_new(sys_mbox_t *mbox_p, int size)
{
    FDC_ASSERT(mbox_p->mbox_queue.cb_signature == 0,
	       mbox_p->mbox_queue.cb_signature, mbox_p);
    FDC_ASSERT(size <= MBOX_QUEUE_SIZE, size, MBOX_QUEUE_SIZE);

    rtos_k_pointer_circular_buffer_init(
        "lwIP mailbox",
        MBOX_QUEUE_SIZE,
        mbox_p->mbox_queue_entries,
        NULL,
        SOC_GET_CURRENT_CPU_ID(),
        &mbox_p->mbox_queue);

    return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
/*
  Deallocates a mailbox. If there are messages still present in the
  mailbox when the mailbox is deallocated, it is an indication of a
  programming error in lwIP and the developer should be notified.
*/
void
sys_mbox_free(sys_mbox_t *mbox_p)
{
    FDC_ASSERT(mbox_p->mbox_queue.cb_signature == RTOS_POINTER_CIRCULAR_BUFFER_SIGNATURE,
	       mbox_p->mbox_queue.cb_signature, mbox_p);

    FDC_ASSERT(rtos_k_circular_buffer_is_empty(&mbox_p->mbox_queue),
	       &mbox_p->mbox_queue, mbox_p);

    *((uint32_t *)&mbox_p->mbox_queue.cb_signature) = 0;
}


/*-----------------------------------------------------------------------------------*/
//   Posts the "msg" to the mailbox.
void
sys_mbox_post(sys_mbox_t *mbox_p, void *msg)
{
    bool write_ok =
	rtos_k_pointer_circular_buffer_write(&mbox_p->mbox_queue, msg, true);

    FDC_ASSERT(write_ok, &mbox_p->mbox_queue, mbox_p);
}


/*
 *Try to post the "msg" to the mailbox. Returns ERR_MEM if this one
 *is full, else, ERR_OK if the "msg" is posted.
 */
err_t
sys_mbox_trypost(sys_mbox_t *mbox_p, void *msg)
{
    bool write_ok =
	rtos_k_pointer_circular_buffer_write(&mbox_p->mbox_queue, msg, false);

    if (write_ok)
        return ERR_OK;
    else
        return ERR_MEM;
}


/*-----------------------------------------------------------------------------------*/
/*
  Blocks the thread until a message arrives in the mailbox, but does
  not block the thread longer than "timeout" milliseconds (similar to
  the sys_arch_sem_wait() function). The "msg" argument is a result
  parameter that is set by the function (i.e., by doing "*msg =
  ptr"). The "msg" parameter maybe NULL to indicate that the message
  should be dropped.

  The return values are the same as for the sys_arch_sem_wait() function:
  Number of milliseconds spent waiting or SYS_ARCH_TIMEOUT if there was a
  timeout.

  Note that a function with a similar name, sys_mbox_fetch(), is
  implemented by lwIP.
*/
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox_p, void **msg, u32_t timeout)
{
    void *msg_read;
    rtos_milliseconds_t read_timeout = timeout;

    bool read_ok =
	rtos_k_pointer_circular_buffer_read(&mbox_p->mbox_queue, &msg_read, true,
					    &read_timeout);

    if (read_ok && msg != NULL) {
	*msg = msg_read;
    }

    FDC_ASSERT(read_timeout <= timeout, read_timeout, timeout);
    return timeout - read_timeout;
}


/*
 *Try to fetch the "msg" from the mailbox. Returns ERR_MEM if this one
 *is empty, else, ERR_OK if the "msg" is fetched.
 */
u32_t
sys_arch_mbox_tryfetch( sys_mbox_t *mbox_p, void **msg)
{
    bool read_ok =
	rtos_k_pointer_circular_buffer_read(&mbox_p->mbox_queue, msg, false, NULL);

    if (read_ok)
        return 0;
    else
        return SYS_MBOX_EMPTY;
}


/*-----------------------------------------------------------------------------------*/
//  Creates and returns a new semaphore. The "count" argument specifies
//  the initial state of the semaphore. TBD finish and test
err_t
sys_sem_new(sys_sem_t *sem_p, u8_t count)
{
    FDC_ASSERT(sem_p->sem_condvar.cv_signature == 0,
	       sem_p->sem_condvar.cv_signature, sem_p);

    sem_p->sem_count = count;
    rtos_k_condvar_init("lwIP semaphore",
			SOC_GET_CURRENT_CPU_ID(),
			&sem_p->sem_condvar);

    return ERR_OK;
}


/*-----------------------------------------------------------------------------------*/
/*
  Blocks the thread while waiting for the semaphore to be
  signaled. If the "timeout" argument is non-zero, the thread should
  only be blocked for the specified time (measured in
  milliseconds).

  If the timeout argument is non-zero, the return value is the number of
  milliseconds spent waiting for the semaphore to be signaled. If the
  semaphore wasn't signaled within the specified time, the return value is
  SYS_ARCH_TIMEOUT. If the thread didn't have to wait for the semaphore
  (i.e., it was already signaled), the function may return zero.

  Notice that lwIP implements a function with a similar name,
  sys_sem_wait(), that uses the sys_arch_sem_wait() function.
*/
u32_t
sys_arch_sem_wait(sys_sem_t *sem_p, u32_t timeout)
{
    rtos_milliseconds_t cv_timeout = timeout;
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    while (sem_p->sem_count == 0) {
        rtos_k_condvar_wait_intr_disabled(&sem_p->sem_condvar, &cv_timeout);
    }

    sem_p->sem_count --;
    rtos_k_restore_cpu_interrupts(cpu_status_register);

    FDC_ASSERT(cv_timeout <= timeout, cv_timeout, timeout);
    return timeout - cv_timeout;
}


/*-----------------------------------------------------------------------------------*/
// Signals a semaphore
void
sys_sem_signal(sys_sem_t *sem_p)
{
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();

    FDC_ASSERT(sem_p->sem_count != UINT8_MAX, sem_p, 0);
    sem_p->sem_count ++;
    FDC_ASSERT(sem_p->sem_count != 0, sem_p, 0);
    rtos_k_restore_cpu_interrupts(cpu_status_register);
    rtos_k_condvar_signal(&sem_p->sem_condvar);
}

/*-----------------------------------------------------------------------------------*/
// Deallocates a semaphore
void
sys_sem_free(sys_sem_t *sem_p)
{
    FDC_ASSERT(sem_p->sem_condvar.cv_signature == RTOS_CONDVAR_SIGNATURE,
	       sem_p->sem_condvar.cv_signature, sem_p);

    *((uint32_t *)&sem_p->sem_condvar.cv_signature) = 0;
}


/** Create a new mutex
 * @param mutex pointer to the mutex to create
 * @return a new mutex */
err_t sys_mutex_new(sys_mutex_t *mutex_p)
{
    FDC_ASSERT(mutex_p->mtx_signature == 0,
	       mutex_p->mtx_signature, mutex_p);

    rtos_k_mutex_init(
        "lwIP mutex",
        SOC_GET_CURRENT_CPU_ID(),
        mutex_p);

    return ERR_OK;
}


/** Lock a mutex
 * @param mutex the mutex to lock */
void sys_mutex_lock(sys_mutex_t *mutex_p)
{
    rtos_k_mutex_acquire(mutex_p);
}


/** Unlock a mutex
 * @param mutex the mutex to unlock */
void sys_mutex_unlock(sys_mutex_t *mutex_p)
{
    rtos_k_mutex_release(mutex_p);
}


/** Delete a semaphore
 * @param mutex the mutex to delete */
void sys_mutex_free(sys_mutex_t *mutex_p)
{
    FDC_ASSERT(mutex_p->mtx_signature == RTOS_MUTEX_SIGNATURE,
	       mutex_p->mtx_signature, mutex_p);

    *((uint32_t *)&mutex_p->mtx_signature) = 0;
}

/*-----------------------------------------------------------------------------------*/
// Initialize sys arch
void
sys_init(void)
{
}


/*
 * Starts a new thread with priority "prio" that will begin its execution in the
 * function "thread()". The "arg" argument will be passed as an argument to the
 * thread() function. The argument "ssize" is the requested stack size for the
 * new thread. The id of the new thread is returned. Both the id and the
 * priority are system dependent.
 */
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
    fdc_error_t fdc_error;
    struct rtos_thread *thread_p = NULL;
    cpu_id_t cpu_id = SOC_GET_CURRENT_CPU_ID();
    struct rtos_thread_creation_params thread_params = {
	.p_name_p = name,
        .p_function_p = (rtos_thread_function_t *)thread,
        .p_function_arg_p = arg,
        .p_priority = RTOS_HIGHEST_THREAD_PRIORITY + 3,
        .p_thread_pp = &thread_p,
    };

    FDC_ASSERT(stacksize == 0, stacksize, 0);
    FDC_ASSERT(prio == 1, prio, 0);

    fdc_error = rtos_k_create_thread(&thread_params);
    if (fdc_error != 0) {
	console_printf(
	    "CPU core %u: *** Error creating application thread '%s' ***\n",
	    cpu_id, name);

	fatal_error_handler(fdc_error);
    }

    console_printf("CPU core %u: %s started\n", cpu_id, name);
    return thread_p;
}


/*
  This optional function does a "fast" critical region protection and returns
  the previous protection level. This function is only called during very short
  critical regions. An embedded system which supports ISR-based drivers might
  want to implement this function by disabling interrupts. Task-based systems
  might want to implement this by using a mutex or disabling tasking. This
  function should support recursive calls from the same task or interrupt. In
  other words, sys_arch_protect() could be called while already protected. In
  that case the return value indicates that it is already protected.

  sys_arch_protect() is only required if your port is supporting an operating
  system.
*/
sys_prot_t sys_arch_protect(void)
{
    return (sys_prot_t)rtos_k_disable_cpu_interrupts();
}

/*
  This optional function does a "fast" set of critical region protection to the
  value specified by pval. See the documentation for sys_arch_protect() for
  more information. This function is only required if your port is supporting
  an operating system.
*/
void sys_arch_unprotect(sys_prot_t pval)
{
    rtos_k_restore_cpu_interrupts((cpu_status_register_t)pval);
}

int sys_sem_valid(sys_sem_t *sem)
{
    return sem->sem_condvar.cv_signature == RTOS_CONDVAR_SIGNATURE;
}
//  set the sem invalid
void sys_sem_set_invalid(sys_sem_t *sem_p)
{
    *((uint32_t *)&sem_p->sem_condvar.cv_signature) = 0;
}
int sys_mbox_valid(sys_mbox_t *mbox)
{
    return mbox->mbox_queue.cb_signature == RTOS_BYTE_CIRCULAR_BUFFER_SIGNATURE;
}
// set the mailbox invalid
void sys_mbox_set_invalid(sys_mbox_t *mbox_p)
{
    *((uint32_t *)&mbox_p->mbox_queue.cb_signature) = 0;
}

/*
This optional function returns the current time in milliseconds (don't care
  for wraparound, this is only used for time diffs).
  Not implementing this function means you cannot use some modules (e.g. TCP
  timestamps, internal timeouts for NO_SYS==1).
  */

u32_t sys_now(void)
{
    return CPU_CLOCK_CYCLES_TO_MILLISECONDS(get_cpu_clock_cycles());
}
