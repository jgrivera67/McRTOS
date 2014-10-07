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
err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
    FDC_ASSERT(mbox->mbox_queue.cb_signature == 0,
	       mbox->mbox_queue.cb_signature, mbox);
    FDC_ASSERT(size <= MBOX_QUEUE_SIZE, size, MBOX_QUEUE_SIZE);

    rtos_k_pointer_circular_buffer_init(
        "lwIP mailbox",
        MBOX_QUEUE_SIZE,
        mbox->mbox_queue_entries,
        NULL,
        SOC_GET_CURRENT_CPU_ID(),
        &mbox->mbox_queue);

    return ERR_OK;
}

/*-----------------------------------------------------------------------------------*/
/*
  Deallocates a mailbox. If there are messages still present in the
  mailbox when the mailbox is deallocated, it is an indication of a
  programming error in lwIP and the developer should be notified.
*/
void
sys_mbox_free(sys_mbox_t *mbox)
{
    FDC_ASSERT(mbox->mbox_queue.cb_signature == RTOS_POINTER_CIRCULAR_BUFFER_SIGNATURE,
	       mbox->mbox_queue.cb_signature, mbox);

    FDC_ASSERT(rtos_k_circular_buffer_is_empty(&mbox->mbox_queue),
	       &mbox->mbox_queue, mbox);

    *((uint32_t *)mbox->mbox_queue.cb_signature) = 0;
}


/*-----------------------------------------------------------------------------------*/
//   Posts the "msg" to the mailbox.
void
sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
    bool write_ok =
	rtos_k_pointer_circular_buffer_write(&mbox->mbox_queue, msg, true);

    FDC_ASSERT(write_ok, &mbox->mbox_queue, mbox);
}


/*
 *Try to post the "msg" to the mailbox. Returns ERR_MEM if this one
 *is full, else, ERR_OK if the "msg" is posted.
 */
err_t
sys_mbox_trypost( sys_mbox_t *mbox, void *msg)
{
    bool write_ok =
	rtos_k_pointer_circular_buffer_write(&mbox->mbox_queue, msg, false);

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
u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
    void *msg_read;
    rtos_milliseconds_t read_timeout = timeout;

    bool read_ok =
	rtos_k_pointer_circular_buffer_read(&mbox->mbox_queue, &msg_read, true,
					    &read_timeout);

    if (read_ok && msg != NULL) {
	*msg = msg_read;
    }

    return timeout - read_timeout;
}

/*-----------------------------------------------------------------------------------*/
//  Creates and returns a new semaphore. The "count" argument specifies
//  the initial state of the semaphore. TBD finish and test
err_t
sys_sem_new(sys_sem_t *sem,u8_t count)
{
    osa_status_t outcome;
    outcome =  OSA_SemaCreate(sem,(uint8_t)count);
    if(outcome == kStatus_OSA_Success)
        return ERR_OK;
    else
	return ERR_VAL;

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
sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
    osa_status_t error;
    uint32_t timeStart , timeEnd;
    if(timeout == (u32_t)0)
        timeout = OSA_WAIT_FOREVER;
    timeStart = OSA_TimeGetMsec();
    error = OSA_SemaWait(sem,(uint32_t)timeout);
    timeEnd = OSA_TimeGetMsec();
    switch(error)
    {
        case kStatus_OSA_Timeout:return SYS_ARCH_TIMEOUT;
	case kStatus_OSA_Success:return (u32_t)(timeEnd-timeStart);
	default : return (u32_t)0;
    }
}


/*-----------------------------------------------------------------------------------*/
// Signals a semaphore
void
sys_sem_signal(sys_sem_t *sem)
{
    OSA_SemaPost(sem);
}

/*-----------------------------------------------------------------------------------*/
// Deallocates a semaphore
void
sys_sem_free(sys_sem_t *sem)
{
    OSA_SemaDestroy(sem);
}


/** Create a new mutex
 * @param mutex pointer to the mutex to create
 * @return a new mutex */
err_t sys_mutex_new(sys_mutex_t *mutex)
{
    osa_status_t error;
    error = OSA_MutexCreate(mutex);
    return (error == kStatus_OSA_Success) ? ERR_OK :  ERR_MEM ;
}


/** Lock a mutex
 * @param mutex the mutex to lock */
void sys_mutex_lock(sys_mutex_t *mutex)
{
    osa_status_t error;
    error = OSA_MutexLock(mutex,OSA_WAIT_FOREVER);
    assert(error == kStatus_OSA_Success) ;
}


/** Unlock a mutex
 * @param mutex the mutex to unlock */
void sys_mutex_unlock(sys_mutex_t *mutex)
{
    osa_status_t error;
    error = OSA_MutexUnlock(mutex);
    assert(error == kStatus_OSA_Success) ;
}


/** Delete a semaphore
 * @param mutex the mutex to delete */
void sys_mutex_free(sys_mutex_t *mutex)
{
    osa_status_t error;
    error = OSA_MutexDestroy(mutex);
    assert(error == kStatus_OSA_Success) ;
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
     task_handler_t taskHandler;
     osa_status_t error;
     task_stack_t * stackMem ;
#ifdef FSL_RTOS_UCOSIII
	 taskHandler = (task_handler_t)OSA_MemAlloc(sizeof(OS_TCB));
#endif

#if (defined FSL_RTOS_UCOSII) || (defined FSL_RTOS_UCOSIII)
    stackMem =  (task_stack_t *)OSA_MemAlloc((size_t)stacksize);
#else
    stackMem = NULL;
#endif
     error = OSA_TaskCreate((task_t)thread ,(uint8_t*) name,(uint16_t) stacksize, stackMem,prio,(task_param_t)arg,false,&taskHandler);
     if(error == kStatus_OSA_Success)
         return taskHandler;
     else
         return (sys_thread_t)0;
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
void sys_sem_set_invalid(sys_sem_t *sem)
{
    sem->sem_condvar.cv_signature = 0;
}
int sys_mbox_valid(sys_mbox_t *mbox)
{
    return mbox->mbox_queue.cb_signature == RTOS_BYTE_CIRCULAR_BUFFER_SIGNATURE;
}
// set the mailbox invalid
void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
    mbox->mbox_queue.cb_signature = 0;
}

/*
This optional function returns the current time in milliseconds (don't care
  for wraparound, this is only used for time diffs).
  Not implementing this function means you cannot use some modules (e.g. TCP
  timestamps, internal timeouts for NO_SYS==1).
  */

u32_t sys_now(void)
{
    return get_cpu_clock_cycles();
}
