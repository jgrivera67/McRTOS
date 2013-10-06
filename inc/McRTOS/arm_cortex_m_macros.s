/**
 * @file McRTOS_interrupt_service_routines_arm_cortex_m.s
 *
 * Machine-specific macros invoked form assembly source files.
 *
 * Copyright (C) 2013 German Rivera
 *
 * @author German Rivera 
 */ 

.extern g_McRTOS_p
.extern debug_dump_r0_to_r3
.extern debug_capture_registers

/**
 * Macro that prints registers r0 to r3
 *
 *
 * INPUT REGISTERS:
 * r0 - r3
 *
 * OUTPUT REGISTERS:
 * None
 *
 * CLOBBERED REGISTERS:
 * None
 */
.macro DEBUG_DUMP_R0_TO_R3
    push    {r0-r3}
    push    {lr}
    bl      debug_dump_r0_to_r3
    pop     {r0}
    mov     lr, r0
    pop     {r0-r3}
.endm


/**
 * Macro that captures registers r0 to r3
 *
 *
 * INPUT REGISTERS:
 * r0 - r3
 *
 * OUTPUT REGISTERS:
 * None
 *
 * CLOBBERED REGISTERS:
 * None
 */
.macro DEBUG_CAPTURE_R0_TO_R3
    push    {r0-r3}
    push    {lr}
    bl      debug_capture_registers
    pop     {r0}
    mov     lr, r0
    pop     {r0-r3}
.endm


/**
 * Macro that retrieves in \_reg_ the value of
 * g_McRTOS_p->rts_cpu_controllers[SOC_GET_CURRENT_CPU_ID()].cpc_current_execution_context_p
 *
 * TODO: Add support for multiple cores.
 *
 * INPUT REGISTERS:
 * None
 *
 * OUTPUT REGISTERS:
 * \_reg_
 *
 * CLOBBERED REGISTERS:
 * None
 */
.macro GET_MCRTOS_CURRENT_EXECUTION_CONTEXT _reg_
    ldr     \_reg_, =g_McRTOS_p /* \_reg_ = &g_McRTOS_p */
    ldr     \_reg_, [\_reg_]    /* \_reg_ = g_McRTOS_p */
    ldr     \_reg_, [\_reg_, #(RTOS_RTS_CPU_CONTROLLERS_OFFSET + RTOS_CPC_CURRENT_EXECUTION_CONTEXT_P_OFFSET)]
.endm


/**
 * Macro that stores the value of \_reg_value_ in
 * g_McRTOS_p->rts_cpu_controllers[0].cpc_current_execution_context_p
 *
 *
 * INPUT REGISTERS:
 * \_reg_value_
 * \_reg_scratch_
 *
 * OUTPUT REGISTERS:
 * None
 *
 * CLOBBERED REGISTERS:
 * \_reg_scratch_
 */
.macro SET_MCRTOS_CURRENT_EXECUTION_CONTEXT _reg_value_, _reg_scratch_
    ldr     \_reg_scratch_, =g_McRTOS_p         /* \_reg_scratch_ = &g_McRTOS_p */
    ldr     \_reg_scratch_, [\_reg_scratch_]    /* \_reg_scratch_ = g_McRTOS_p */
    str     \_reg_value_, [\_reg_scratch_, #(RTOS_RTS_CPU_CONTROLLERS_OFFSET + RTOS_CPC_CURRENT_EXECUTION_CONTEXT_P_OFFSET)]
.endm

