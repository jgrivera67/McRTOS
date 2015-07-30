/**
 * @file cortex_m_nvic.h
 *
 * ARM Cortex-M NVIC common declarations
 *
 * Copyright (C) 2014 German Rivera
 *
 * @author German Rivera
 */
#ifndef _CORTEX_M_NVIC_H
#define _CORTEX_M_NVIC_H

#ifndef SOC_NUM_INTERRUPT_CHANNELS
#   error "SOC_NUM_INTERRUPT_CHANNELS must be defined before including this file"
#endif

#include <McRTOS/arm_defs.h>

#define INT_SVCall_IRQn     VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SVCall)
#define PendableSrvReq_IRQn VECTOR_NUMBER_TO_IRQ_NUMBER(INT_PendableSrvReq)
#define SysTick_IRQn        VECTOR_NUMBER_TO_IRQ_NUMBER(INT_SysTick)

enum cpu_core_internal_interrupt_vectors {
  INT_Initial_Stack_Pointer    = 0,                /* Initial stack pointer */
  INT_Initial_Program_Counter  = 1,                /* Initial program counter */
  INT_NMI                      = 2,                /* Non-maskable interrupt */
  INT_Hard_Fault               = INT_HARD_FAULT,
  INT_MemoryManagement	       = INT_MEMORY_MANAGEMENT,
  INT_BusFault                 = INT_BUS_FAULT,
  INT_UsageFault               = INT_USAGE_FAULT,
  INT_SVCall                   = 11,               /* A supervisor call exception */
  INT_DebugMonitor             = INT_DEBUG_MONITOR,
  INT_PendableSrvReq           = 14,               /* PendSV exception - request for system level service */
  INT_SysTick                  = 15,               /* SysTick interrupt */
};

/**
* IRQ number range type.
* (typedef needed by the CMSIS APIs)
*
* NOTE:
* - INT_SVCall_IRQn is negative
* - The ARMv7-M NVIC architecture supports up to
*   496 interrupts. Actual number of IRQs can be determined from the NVIC's
*   read-only Interrupt Controller Type Register (ICTR)
*/
typedef _RANGE_(INT_SVCall_IRQn, SOC_NUM_INTERRUPT_CHANNELS - 1)
    int16_t IRQn_Type;

#endif /* _CORTEX_M_NVIC_H */
