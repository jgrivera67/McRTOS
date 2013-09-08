/**
 * @file McRTOS_execution_controller.c
 *
 * McRTOS execution controller
 *
 * @author German Rivera 
 */ 

#include "McRTOS_internals.h"
#include "failure_data_capture.h"
#include "utils.h"

/**
 * Per-CPU McRTOS thread scheduler
 */
void rtos_thread_scheduler(void)
{
    fdc_error_t fdc_error;
    cpu_clock_cycles_t measured_cycles;
    DECLARE_CPU_CYCLES_MEASURE_VARS();

    FDC_ASSERT_CPU_INTERRUPTS_DISABLED();

    BEGIN_CPU_CYCLES_MEASURE();

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    /*
     * Find highest runnable thread priority:
     */
    rtos_thread_prio_t chosen_thread_prio =
        rtos_k_find_highest_thread_priority(
            cpu_controller_p->cpc_runnable_thread_priorities);

    if (chosen_thread_prio == RTOS_NUM_THREAD_PRIORITIES)
    {
        fdc_error = CAPTURE_FDC_ERROR(
            "At least the idle thread should be runnable",
            cpu_controller_p->cpc_runnable_thread_priorities,
            cpu_controller_p);

        fatal_error_handler(fdc_error);
    }

    struct glist_node *chosen_thread_queue_anchor_p = 
        &cpu_controller_p->cpc_runnable_thread_queues_anchors
                                                    [chosen_thread_prio];

    FDC_ASSERT(
        GLIST_IS_NOT_EMPTY(chosen_thread_queue_anchor_p),
        chosen_thread_queue_anchor_p, chosen_thread_prio);

    /*
     * Current execution context is a thread and is not in preemption chain:
     */

    struct rtos_execution_context *current_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    FDC_ASSERT(
        current_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        current_context_p->ctx_context_type, current_context_p);

    FDC_ASSERT(
        GLIST_NODE_IS_UNLINKED(&current_context_p->ctx_preemption_chain_node),
        &current_context_p->ctx_preemption_chain_node,
        current_context_p);

    /*
     * Current thread is not in running state anymore and it is in
     * a thread queue (a runnable thread queue or a waiting thread queue
     * depending on its state)
     */

    struct rtos_thread *current_thread_p =
        RTOS_EXECUTION_CONTEXT_GET_THREAD(current_context_p);

    DBG_ASSERT(
        current_thread_p == cpu_controller_p->cpc_current_thread_p,
        current_thread_p, cpu_controller_p->cpc_current_thread_p);

    FDC_ASSERT(
        current_thread_p->thr_state != RTOS_THREAD_RUNNING,
        current_thread_p->thr_state, current_thread_p);

    FDC_ASSERT(
        GLIST_NODE_IS_LINKED(&current_thread_p->thr_list_node),
        &current_thread_p->thr_list_node, current_thread_p);

    /*
     * Check last saved CPU registers for the current thread:
     */
    FDC_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(current_context_p);

    /*
     * Remove first thread from the chosen thread queue:
     */
    
    struct glist_node *chosen_thread_node_p =
        GLIST_GET_FIRST(chosen_thread_queue_anchor_p);

    struct rtos_thread *chosen_thread_p =
        RTOS_THREAD_QUEUE_NODE_GET_THREAD(chosen_thread_node_p);

    rtos_remove_runnable_thread(cpu_controller_p, chosen_thread_p);

    /*
     * If the chosen thread is the same as the current thread,
     * decrement its time slice ticks left. If its time slice
     * is exhausted, we need to pick the next thread from the
     * chosen thread queue:
     * - Add the chosen thread back to the chosen thread queue, but 
     *   this time at the end of the queue.
     * - Remove again the first thread again from the chosen thread
     *   queue
     */
    if (chosen_thread_p == current_thread_p)
    {
        rtos_thread_time_slice_in_ticks_t time_slice_ticks_left = 
            current_thread_p->thr_time_slice_ticks_left;
        
        FDC_ASSERT(
            time_slice_ticks_left > 0 &&
            time_slice_ticks_left <= RTOS_THREAD_TIME_SLICE_IN_TICKS, 
            time_slice_ticks_left, current_thread_p);

        if (cpu_controller_p->cpc_pending_thread_time_slice_decrement)
        {
            cpu_controller_p->cpc_pending_thread_time_slice_decrement = false;
            time_slice_ticks_left --;
            if (time_slice_ticks_left == 0)
            {
                FDC_ASSERT(
                    current_thread_p->thr_state == RTOS_THREAD_RUNNABLE,
                    current_thread_p->thr_state, current_thread_p);

                RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
                    current_context_p,
                    CTX_SWITCHED_OUT_THREAD_TIME_SLICE_EXHAUSTED,
                    cpu_controller_p);

                current_thread_p->thr_preempted_by_time_slice_count ++;

                current_thread_p->thr_time_slice_ticks_left =
                    RTOS_THREAD_TIME_SLICE_IN_TICKS;

                /*
                 * Add the current thread back to the runnable queue but only if
                 * there are other threads in that queue:
                 */
                if (GLIST_IS_NOT_EMPTY(chosen_thread_queue_anchor_p))
                {
                    rtos_add_tail_runnable_thread(
                        cpu_controller_p, current_thread_p);
                       
                    chosen_thread_node_p =
                        GLIST_GET_FIRST(chosen_thread_queue_anchor_p);

                    chosen_thread_p =
                        RTOS_THREAD_QUEUE_NODE_GET_THREAD(chosen_thread_node_p);

                    rtos_remove_runnable_thread(cpu_controller_p, chosen_thread_p);
                }
            }
            else
            {
                current_thread_p->thr_time_slice_ticks_left = time_slice_ticks_left;
            }
        }
    }

    /*
     * If the chosen thread is in the preemption chain, it must have been
     * preempted by another thread. So, remove it from the preemption chain:
     *
     * NOTE: If the chosen thread is in the preemption chain, it is not
     * necessarily at the top of the chain, as its priority may been boosted.
     */
    struct rtos_execution_context *chosen_context_p =
            &chosen_thread_p->thr_execution_context;

    if (GLIST_NODE_IS_LINKED(&chosen_context_p->ctx_preemption_chain_node))
    {
        DBG_ASSERT(
            chosen_context_p->ctx_last_preempted_by_p != NULL &&
            chosen_context_p->ctx_last_preempted_by_p->ctx_context_type ==
            RTOS_THREAD_CONTEXT,
            chosen_context_p->ctx_last_preempted_by_p, chosen_context_p); 

        rtos_preemption_chain_remove_context(
            cpu_controller_p, chosen_context_p);
    }

    /*
     * Set the new current execution context and thread:
     */
    if (chosen_thread_p != current_thread_p)
    {
        if (current_thread_p->thr_time_slice_ticks_left <
                RTOS_THREAD_TIME_SLICE_IN_TICKS &&
            current_thread_p->thr_state == RTOS_THREAD_RUNNABLE)
        {
            /*
             * Current thread was preempted by a higher priority
             * thread.
             */
            RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
                current_context_p,
                CTX_SWITCHED_OUT_THREAD_PREEMPTED_BY_THREAD,
                cpu_controller_p);

            current_context_p->ctx_last_preempted_by_p =
                &chosen_thread_p->thr_execution_context;
    
            current_context_p->ctx_preempted_counter ++;

            current_thread_p->thr_preempted_by_other_thread_count ++;

            /*
             * Add old current execution context at the top of the preemption
             * chain:
             */
            rtos_preemption_chain_push_context(
                cpu_controller_p, current_context_p);
        }

        current_thread_p = chosen_thread_p;
        current_context_p = &chosen_thread_p->thr_execution_context;
        cpu_controller_p->cpc_current_thread_p = current_thread_p;
        cpu_controller_p->cpc_current_execution_context_p = current_context_p;
    }

    /*
     * The new current thread is in runnable state and is not in any
     * thread queue at this point
     */

    FDC_ASSERT(
        current_thread_p->thr_state == RTOS_THREAD_RUNNABLE,
        current_thread_p->thr_state, current_thread_p);

    FDC_ASSERT(
        GLIST_NODE_IS_UNLINKED(&current_thread_p->thr_list_node),
        &current_thread_p->thr_list_node, current_thread_p);

    RTOS_THREAD_CHANGE_STATE(current_thread_p, RTOS_THREAD_RUNNING); 

    END_CPU_CYCLES_MEASURE(measured_cycles);

    cpu_controller_p->cpc_thread_scheduler_calls ++; 
    cpu_controller_p->cpc_accumulated_thread_scheduler_overhead += measured_cycles;

    /*
     * Check saved CPU registers for the thread context to be restored:
     */
    FDC_ASSERT_RTOS_EXECUTION_CONTEXT_CPU_REGISTERS(current_context_p);

    /*
     * Switch to execute the current thread context:
     */

    current_context_p->ctx_last_switched_in_time_stamp = get_cpu_clock_cycles();

    rtos_k_restore_execution_context(current_context_p);

    /*
     * We should never come back here:
     */
    FDC_ASSERT(false, 0, 0);
}


/**
 * Per-CPU tick timer handler. This function is invoked from the
 * isr_rtos_tick_timer() ISRs.
 */
void
rtos_tick_timer_interrupt_handler(
    struct rtos_interrupt *timer_interrupt_p)
{
    cpu_id_t cpu_id = timer_interrupt_p->int_cpu_id;

    DBG_ASSERT(cpu_id == SOC_GET_CURRENT_CPU_ID(),
        cpu_id, SOC_GET_CURRENT_CPU_ID());

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

#if 0 // TODO
    DBG_ASSERT_PRIVILEGED_CPU_MODE_AND_INTERRUPTS_ENABLED();
#endif

    /*
     * Disable interrupts in the ARM core
     */
    cpu_status_register_t cpu_status_register = rtos_k_disable_cpu_interrupts();
    
    cpu_controller_p->cpc_ticks_since_boot_count ++;
    
    if (cpu_controller_p->cpc_ticks_since_boot_count % RTOS_HEARTBEAT_TICKS == 0)
    {
        toggle_heartbeat_led(); 
    }

    /*
     * Do timer wheel processing:
     */

    cpu_controller_p->cpc_current_timer_wheel_spoke_index ++;
    if (cpu_controller_p->cpc_current_timer_wheel_spoke_index == 
        RTOS_TIMER_WHEEL_NUM_SPOKES)
    {
        cpu_controller_p->cpc_current_timer_wheel_spoke_index = 0; 
    }

    struct glist_node *timer_hash_chain_anchor_p = 
        &cpu_controller_p->cpc_timer_wheel_hash_chains_anchors[
            cpu_controller_p->cpc_current_timer_wheel_spoke_index];

    struct glist_node *timer_node_p;
    struct glist_node *next_p;

    GLIST_FOR_EACH_NODE_REMOVING(timer_node_p, next_p, timer_hash_chain_anchor_p)
    {
        struct rtos_timer *rtos_timer_p = 
            RTOS_TIMER_HASH_CHAIN_NODE_GET_TIMER(timer_node_p);

        DBG_ASSERT(
            rtos_timer_p->tmr_signature == RTOS_TIMER_SIGNATURE,
            rtos_timer_p->tmr_signature, rtos_timer_p);

        if (rtos_timer_p->tmr_time_to_expire == 0)
        {
            glist_remove_elem(timer_node_p);

            /*
             * Call timer callback:
             */
            rtos_timer_p->tmr_callback_function_p(rtos_timer_p);
        }
        else
        {
            FDC_ASSERT(
                rtos_timer_p->tmr_time_to_expire >= RTOS_MILLISECONDS_PER_TICK,
                rtos_timer_p->tmr_time_to_expire, rtos_timer_p);

            rtos_timer_p->tmr_time_to_expire -= RTOS_MILLISECONDS_PER_TICK;
        }
    }

    /*
     * Restore previous interrupt masking in the ARM core
     */
    rtos_k_restore_cpu_interrupts(cpu_status_register);

    /*
     * Thread time slice tick bookkeeping is done in the thread scheduler,
     * which will be called by rtos_k_exit_interrupt() if there an no more nested
     * interrupts in the preemption chain. We just need to set the flag
     * cpc_pending_thread_time_slice_decrement here:
     */
    cpu_controller_p->cpc_pending_thread_time_slice_decrement = true;
}


#ifdef _CPU_CYCLES_MEASURE_

cpu_status_register_t
rtos_start_interrupts_disabled_time_measure(
    cpu_status_register_t saved_cpsr)
{

    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    /*
     * Skip measure if the thread scheduler has not been turned on yet:
     */
    if (_INFREQUENTLY_TRUE_(
            cpu_controller_p->cpc_current_execution_context_p == NULL))
    {
        goto Exit;
    }

    uint8_t interrupts_disabled_being_measured_count =
        cpu_controller_p->cpc_interrupts_disabled_being_measured_count;

    if (interrupts_disabled_being_measured_count == 0)
    {
        cpu_controller_p->cpc_interrupts_disabled_start_time_stamp =
            get_cpu_clock_cycles();
    }

    DBG_ASSERT(
        interrupts_disabled_being_measured_count < UINT8_MAX,
        interrupts_disabled_being_measured_count, cpu_controller_p);

    cpu_controller_p->cpc_interrupts_disabled_being_measured_count ++;

Exit:
    return saved_cpsr;
}


cpu_status_register_t
rtos_stop_interrupts_disabled_time_measure(
    cpu_status_register_t saved_cpsr)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    struct rtos_execution_context *current_execution_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    /*
     * Skip measure if the thread scheduler has not been turned on yet:
     */
    if (_INFREQUENTLY_TRUE_(current_execution_context_p == NULL))
    {
        goto Exit;
    }

    DBG_ASSERT(
        cpu_controller_p->cpc_interrupts_disabled_being_measured_count > 0,
        cpu_controller_p, 0);

    cpu_controller_p->cpc_interrupts_disabled_being_measured_count --;

    if (cpu_controller_p->cpc_interrupts_disabled_being_measured_count == 0)
    {
        cpu_clock_cycles_t end_time_stamp = get_cpu_clock_cycles();
        cpu_clock_cycles_t delta_cpu_cycles =
            CPU_CLOCK_CYCLES_DELTA(
                cpu_controller_p->cpc_interrupts_disabled_start_time_stamp,
                end_time_stamp);
        
        FDC_ASSERT(
            delta_cpu_cycles >= g_McRTOS_p->rts_cpu_cycles_measure_overhead,
            delta_cpu_cycles, g_McRTOS_p->rts_cpu_cycles_measure_overhead);

        delta_cpu_cycles -= g_McRTOS_p->rts_cpu_cycles_measure_overhead;

        FDC_ASSERT3(
            CPU_CLOCK_CYCLES_TO_MICROSECONDS(delta_cpu_cycles) <= 500,
            CPU_CLOCK_CYCLES_TO_MICROSECONDS(delta_cpu_cycles),
            end_time_stamp,
            cpu_controller_p->cpc_interrupts_disabled_start_time_stamp);

        if (delta_cpu_cycles > cpu_controller_p->cpc_longest_time_interrupts_disabled)
        {
            cpu_controller_p->cpc_longest_time_interrupts_disabled = delta_cpu_cycles;
        }

        cpu_controller_p->cpc_latest_measurement_time_interrupts_disabled =
            delta_cpu_cycles;
    }
    

Exit:
    return saved_cpsr;
}

#endif /* _CPU_CYCLES_MEASURE_ */


/**
 * Thread delay timer callback
 */
void rtos_delay_timer_callback(struct rtos_timer *rtos_timer_p)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();
    DBG_ASSERT(
        rtos_timer_p->tmr_signature == RTOS_TIMER_SIGNATURE,
        rtos_timer_p->tmr_signature, rtos_timer_p);

    DBG_ASSERT(
        rtos_timer_p->tmr_time_to_expire == 0,
        rtos_timer_p->tmr_time_to_expire, rtos_timer_p);

    DBG_ASSERT(
        GLIST_NODE_IS_UNLINKED(&rtos_timer_p->tmr_list_node),
        &rtos_timer_p->tmr_list_node, rtos_timer_p);

    struct rtos_thread *rtos_thread_p =
        ENCLOSING_STRUCT(rtos_timer_p, struct rtos_thread, thr_delay_timer);

    DBG_ASSERT_RTOS_THREAD_INVARIANTS(rtos_thread_p);

    rtos_k_condvar_signal(&rtos_thread_p->thr_condvar);
}


