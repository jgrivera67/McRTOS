#include "arm_defs.h"

.extern capture_unexpected_undefined_instruction
.extern capture_unexpected_data_abort
.extern capture_unexpected_prefetch_abort

.global fdc_undefined_instruction_handler
.global fdc_data_abort_handler
.global fdc_prefetch_abort_handler
  
.text
.arm

.func fdc_undefined_instruction_handler
/* 
 * Undefined instruction exception handler
 */
fdc_undefined_instruction_handler:
    /*
     * Undefined mode's lr - 4 == address of instruction that caused the
     * exception.
     */

    /*
     * Call capture_unexpected_undefined_instruction():
     */
    sub     r0, lr, #4
    ldr     r1, [r0]
    mrs     r2, spsr
    bl      capture_unexpected_undefined_instruction

L_fdc_undefined_instruction_wait_for_reset:
    b  L_fdc_undefined_instruction_wait_for_reset 

.endfunc


.func fdc_prefetch_abort_handler
/* 
 * Prefetch abort exception handler
 */
fdc_prefetch_abort_handler:
    /*
     * Abort-mode's lr - 4 is the address where the prefetch abort
     * happened.
     */

    /*
     * Ensure that the T bit in the abort-mode spsr is not set, but
     * save the unmodified value in r2
     */
    mrs     r2, spsr
    mov     r0, r2
    bic     r0, #T_BIT
    msr     spsr_c, r0

    /*
     * Switch to previous mode but keeping interrupts disabled, to capture that
     * mode's lr:
     *
     * NOTE: If previous mode is user mode, we switch to system mode instead,
     * as we need to stay privileged.
     */
    teq     r0, #ARM_MODE_USER
    moveq   r0, #ARM_MODE_SYS
    orr     r0, r0, #ARM_INTERRUPTS_DISABLED_MASK
    msr     cpsr_c, r0

    /*
     * Capture previous mode's lr in r1:
     */
    mov     r1, lr
    
    /*
     * Switch back to abort mode (keeping interrupts disabled) to continue 
     * handling the exception:
     */
    msr     cpsr_c, #(ARM_MODE_ABORT | ARM_INTERRUPTS_DISABLED_MASK)

    /*
     * Call capture_unexpected_prefetch_abort():
     *
     * r1 == previous mode's lr
     * r2 == previous mode's cpsr
     * 
     * NOTE: previous mode's lr - 4 == address of bx instruction that branched
     * to an invalid address.
     */
    sub     r0, lr, #4
    sub     r1, r1, #4
    bl      capture_unexpected_prefetch_abort

L_fdc_prefetch_abort_wait_for_reset:
    b  L_fdc_prefetch_abort_wait_for_reset

.endfunc


.func fdc_data_abort_handler
/* 
 * Data abort exception handler
 */
fdc_data_abort_handler:
    /*
     * Abort-mode's lr - 8 is the address of the instruction that caused the
     * data abort.
     */

    /*
     * Call capture_unexpected_data_abort():
     * - lr - 4 == address of instruction that caused the data abort
     */
    sub     r0, lr, #8
    mov     r1, #0
    mrs     r2, spsr
    bl      capture_unexpected_data_abort

L_fdc_data_abort_wait_for_reset:
    b  L_fdc_data_abort_wait_for_reset

.endfunc

.end

