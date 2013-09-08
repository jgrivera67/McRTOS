/**
 * @file lpc2478_arm_defs.h
 *
 * LPC2478-specific ARM declarations to be included in both assembly and C code
 *
 * NOTE: Only preprocessor constructs should be used here.
 *
 * @author German Rivera 
 */ 

#ifndef _LPC2478_ARM_DEFS_H
#define _LPC2478_ARM_DEFS_H

#include "arm_defs.h"

/**
 * Address of the LPC2478 VIC's VICAddress register
 */
#define LPC2478_VIC_ADDRESS_ADDR    0xFFFFFF00 

/**
 * Value of the PC when the processor executes the instruction at
 * address ARM_IRQ_VECTOR_ADDRESS
 */
#define ARM_IRQ_VECTOR_INSTR_PC     (ARM_IRQ_VECTOR_ADDRESS + 8)

/**
 * Value that needs to be subtracted from the PC when executing the
 * instruction at address ARM_IRQ_VECTOR_ADDRESS, if that instruction is a
 * a PC-relative indirect branch to jump to the address contained in the
 * VIC's VICAddress register
 */
#define LPC2478_VIC_ADDRESS_PC_DELTA \
        (ARM_IRQ_VECTOR_INSTR_PC - LPC2478_VIC_ADDRESS_ADDR)

#endif /* _LPC2478_ARM_DEFS_H */
