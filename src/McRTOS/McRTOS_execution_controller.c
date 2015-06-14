/**
 * @file McRTOS_execution_controller.c
 *
 * McRTOS execution controller
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera
 */

#include "McRTOS_internals.h"
#include "failure_data_capture.h"
#include "utils.h"

/**
 * Per-CPU McRTOS thread scheduler
 */
void rtos_thread_scheduler(
        rtos_context_switch_type_t ctx_switch_type)
{
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

    /*
     * At least the idle thread should be runnable
     */
    FDC_ASSERT(
        chosen_thread_prio < RTOS_NUM_THREAD_PRIORITIES,
        cpu_controller_p->cpc_runnable_thread_priorities,
        cpu_controller_p);

    struct glist_node *chosen_thread_queue_anchor_p =
        &cpu_controller_p->cpc_runnable_thread_queues_anchors
                                                    [chosen_thread_prio];

    FDC_ASSERT(
        GLIST_IS_NOT_EMPTY(chosen_thread_queue_anchor_p),
        chosen_thread_queue_anchor_p, chosen_thread_prio);

    struct rtos_execution_context *current_context_p =
        cpu_controller_p->cpc_current_execution_context_p;

    /*
     * Current execution context is a thread, an interrupt handler or the
     * reset handler and it is not in a preemption chain:
     *
     * NOTE: If this function is invoked from rtos_k_exit_interrupt(),
     * current_context_p is an interrupt. Otherwise, current_context_p
     * is a thread or the reset handler.
     */
    FDC_ASSERT(
        current_context_p->ctx_context_type == RTOS_THREAD_CONTEXT ||
        current_context_p->ctx_context_type == RTOS_INTERRUPT_CONTEXT ||
        current_context_p->ctx_context_type == RTOS_RESET_CONTEXT,
        current_context_p->ctx_context_type, current_context_p);

    FDC_ASSERT(
        GLIST_NODE_IS_UNLINKED(&current_context_p->ctx_preemption_chain_node),
        &current_context_p->ctx_preemption_chain_node,
        current_context_p);

    struct rtos_thread *current_thread_p;

    if (current_context_p->ctx_context_type == RTOS_THREAD_CONTEXT)
    {
        /*
         * This is a synchronous context switch from another thread.
         * Current thread is not in running state anymore and it is in a thread
         * queue (a runnable thread queue or a waiting thread queue depending on
         * its state)
         */
        current_thread_p =
            RTOS_EXECUTION_CONTEXT_GET_THREAD(current_context_p);

        DBG_ASSERT(
            current_thread_p == cpu_controller_p->cpc_current_thread_p,
            current_thread_p, cpu_controller_p->cpc_current_thread_p);
    } else {
        /*
         * Asynchronous context switch from an interrupt handler or
         * first synchronous context switch from the reset handler
         */
        current_thread_p = cpu_controller_p->cpc_current_thread_p;
        if (current_thread_p == NULL) {
            FDC_ASSERT(
                current_context_p->ctx_context_type == RTOS_RESET_CONTEXT,
                current_context_p->ctx_context_type, current_context_p);
        } else {
            FDC_ASSERT(
                &current_thread_p->thr_execution_context != current_context_p,
                &current_thread_p->thr_execution_context, current_thread_p);
        }
    }

    if (current_thread_p != NULL) {
        FDC_ASSERT(
            current_thread_p->thr_state != RTOS_THREAD_RUNNING,
            current_thread_p->thr_state, current_thread_p);

        FDC_ASSERT(
            GLIST_NODE_IS_LINKED(&current_thread_p->thr_list_node) ||
	    current_thread_p->thr_state == RTOS_THREAD_ABORTED,
            current_thread_p->thr_state, current_thread_p);

        if (current_thread_p->thr_fpu_enable_count != 0) {
            FDC_ASSERT(current_thread_p == cpu_controller_p->cpc_last_fpu_thread_p,
                       current_thread_p, cpu_controller_p->cpc_last_fpu_thread_p);

            cortex_m_disable_fpu();
        }
    }

    /*
     * Remove first thread from the chosen thread queue:
     */

    struct glist_node *chosen_thread_node_p =
        GLIST_GET_FIRST(chosen_thread_queue_anchor_p);

    struct rtos_thread *chosen_thread_p =
        RTOS_THREAD_QUEUE_NODE_GET_THREAD(chosen_thread_node_p);

    rtos_remove_runnable_thread(
        cpu_controller_p, chosen_thread_p, RTOS_THREAD_RUNNING);

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
                RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
                    &current_thread_p->thr_execution_context,
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

                    rtos_remove_runnable_thread(
                        cpu_controller_p, chosen_thread_p, RTOS_THREAD_RUNNING);
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
     * necessarily at the top of the chain, as its priority may have been
     * boosted.
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

    if (chosen_thread_p != current_thread_p)
    {
        if (current_thread_p != NULL &&
            current_thread_p->thr_time_slice_ticks_left <
                RTOS_THREAD_TIME_SLICE_IN_TICKS &&
            current_thread_p->thr_state == RTOS_THREAD_RUNNABLE)
        {
            /*
             * Current thread was preempted by a higher priority
             * thread.
             */
            RTOS_EXECUTION_CONTEXT_SET_SWITCHED_OUT_REASON(
                &current_thread_p->thr_execution_context,
                CTX_SWITCHED_OUT_THREAD_PREEMPTED_BY_THREAD,
                cpu_controller_p);

            current_thread_p->thr_execution_context.ctx_last_preempted_by_p =
                &chosen_thread_p->thr_execution_context;

            current_thread_p->thr_execution_context.ctx_preempted_counter ++;

            current_thread_p->thr_preempted_by_other_thread_count ++;

            /*
             * Add current thread's execution context at the top of the preemption
             * chain:
             */
            rtos_preemption_chain_push_context(
                cpu_controller_p, &current_thread_p->thr_execution_context);
        }

        /*
         * Update the current thread running on the calling CPU:
         */
        cpu_controller_p->cpc_current_thread_p = chosen_thread_p;

        /*
         * NOTE: cpu_controller_p->cpc_current_execution_context_p
         * will be updated by rtos_k_restore_execution_context()
         */
    }

    /*
     * The chosen thread is in running state and is not in any
     * thread queue at this point
     */

    FDC_ASSERT(
        chosen_thread_p->thr_state == RTOS_THREAD_RUNNING,
        chosen_thread_p->thr_state, chosen_thread_p);

    FDC_ASSERT(
        GLIST_NODE_IS_UNLINKED(&chosen_thread_p->thr_list_node),
        &chosen_thread_p->thr_list_node, chosen_thread_p);

    END_CPU_CYCLES_MEASURE(measured_cycles);

    cpu_controller_p->cpc_thread_scheduler_calls ++;
    cpu_controller_p->cpc_accumulated_thread_scheduler_overhead += measured_cycles;

    DBG_ASSERT(
        chosen_context_p->ctx_context_type == RTOS_THREAD_CONTEXT,
        chosen_context_p->ctx_context_type, chosen_context_p);

    /*
     * Switch to execute the chosen thread context:
     */

#   ifdef _CPU_CYCLES_MEASURE_
    chosen_context_p->ctx_last_switched_in_time_stamp = get_cpu_clock_cycles();
#   endif

    /*
     * Set MPU regions for the chosen thread:
     */
    mpu_set_all_thread_data_regions(SOC_GET_CURRENT_CPU_ID(),
			            chosen_thread_p->thr_mpu_regions);

    /*
     * Do PFU context switch if necessary
     */
    if (chosen_thread_p->thr_fpu_enable_count != 0) {
	struct rtos_thread *last_fpu_thread_p =
		cpu_controller_p->cpc_last_fpu_thread_p;

	cortex_m_enable_fpu();
	if (chosen_thread_p != last_fpu_thread_p) {
	    if (last_fpu_thread_p != NULL) {
                FDC_ASSERT(last_fpu_thread_p->thr_signature == RTOS_THREAD_SIGNATURE,
                           last_fpu_thread_p->thr_signature, last_fpu_thread_p);

		cpu_controller_p->cpc_fpu_context_switch_count ++;
                cortex_m_save_fpu_context(&last_fpu_thread_p->thr_saved_fpu_context);
            }

            cortex_m_restore_fpu_context(&chosen_thread_p->thr_saved_fpu_context);
            cpu_controller_p->cpc_last_fpu_thread_p = chosen_thread_p;
	}
    }

    rtos_k_restore_execution_context(chosen_context_p, ctx_switch_type);

    /*
     * We should never come back here:
     */
    FDC_ASSERT(false, 0, 0);
}


static inline void
toggle_heartbeat_led(void)
{
    toggle_rgb_led(LED_COLOR_BLUE);
}


/**
 * Per-CPU tick timer handler. This function is invoked from the
 * tick timer ISR.
 */
void
rtos_tick_timer_interrupt_handler(
    struct rtos_interrupt *timer_interrupt_p)
{
    cpu_id_t cpu_id = timer_interrupt_p->int_cpu_id;

    FDC_ASSERT_RTOS_INTERRUPT_E_HANDLER_PRECONDITIONS(timer_interrupt_p);

    DBG_ASSERT(cpu_id == SOC_GET_CURRENT_CPU_ID(),
        cpu_id, SOC_GET_CURRENT_CPU_ID());

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[cpu_id];

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
     * which will be called by rtos_k_exit_interrupt() if there are no more
     * nested interrupts in the preemption chain. We just need to set the flag
     * cpc_pending_thread_time_slice_decrement here:
     */
    cpu_controller_p->cpc_pending_thread_time_slice_decrement = true;
}


#ifdef _MEASURE_INTERRUPTS_DISABLED_TIME_

cpu_status_register_t
rtos_start_interrupts_disabled_time_measure(
    cpu_register_t saved_reg)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    if (_INFREQUENTLY_TRUE_(
            !cpu_controller_p->cpc_measure_interrupts_disabled_time))
    {
	DBG_ASSERT(
	    cpu_controller_p->cpc_interrupts_disabled_being_measured_count == 0,
	    cpu_controller_p, 0);

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
    return saved_reg;
}


cpu_status_register_t
rtos_stop_interrupts_disabled_time_measure(
    cpu_register_t saved_reg)
{
    DBG_ASSERT_CPU_INTERRUPTS_DISABLED();

    struct rtos_cpu_controller *cpu_controller_p =
        &g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()];

    if (_INFREQUENTLY_TRUE_(
            !cpu_controller_p->cpc_measure_interrupts_disabled_time))
    {
	DBG_ASSERT(
	    cpu_controller_p->cpc_interrupts_disabled_being_measured_count == 0,
	    cpu_controller_p, 0);

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

        FDC_ASSERT(
            CPU_CLOCK_CYCLES_TO_MICROSECONDS(delta_cpu_cycles) <= RTOS_MAX_TIME_INTERRUPTS_DISABLED,
            CPU_CLOCK_CYCLES_TO_MICROSECONDS(delta_cpu_cycles),
            end_time_stamp);

        if (delta_cpu_cycles > cpu_controller_p->cpc_longest_time_interrupts_disabled)
        {
            cpu_controller_p->cpc_longest_time_interrupts_disabled = delta_cpu_cycles;
        }
    }


Exit:
    return saved_reg;
}

#endif /* _MEASURE_INTERRUPTS_DISABLED_TIME_ */


/**
 * Thread timer callback
 */
void rtos_thread_timer_callback(struct rtos_timer *rtos_timer_p)
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
        ENCLOSING_STRUCT(rtos_timer_p, struct rtos_thread, thr_timer);

    DBG_ASSERT_RTOS_THREAD_INVARIANTS(rtos_thread_p);

    if (rtos_thread_p->thr_state == RTOS_THREAD_BLOCKED_ON_CONDVAR) {
	FDC_ASSERT(rtos_thread_p->thr_blocked_on_condvar_p != NULL,
		   rtos_thread_p, 0);

	rtos_k_condvar_signal(rtos_thread_p->thr_blocked_on_condvar_p);
    }
}


