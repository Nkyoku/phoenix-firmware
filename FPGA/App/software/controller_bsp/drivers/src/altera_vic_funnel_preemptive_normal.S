/******************************************************************************
*                                                                             *
* License Agreement                                                           *
*                                                                             *
* Copyright (c) 2015 Altera Corporation, San Jose, California, USA.           *
* All rights reserved.                                                        *
*                                                                             *
* Permission is hereby granted, free of charge, to any person obtaining a     *
* copy of this software and associated documentation files (the "Software"),  *
* to deal in the Software without restriction, including without limitation   *
* the rights to use, copy, modify, merge, publish, distribute, sublicense,    *
* and/or sell copies of the Software, and to permit persons to whom the       *
* Software is furnished to do so, subject to the following conditions:        *
*                                                                             *
* The above copyright notice and this permission notice shall be included in  *
* all copies or substantial portions of the Software.                         *
*                                                                             *
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR  *
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,    *
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE *
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER      *
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING     *
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER         *
* DEALINGS IN THE SOFTWARE.                                                   *
*                                                                             *
* This agreement shall be governed in all respects by the laws of the State   *
* of California and by the laws of the United States of America.              *
*                                                                             *
******************************************************************************/
/* Setup header files to work with assembler code. */
#define ALT_ASM_SRC

/* 
 * This code implemennts the interrupt funnel for interrupts assigned to the
 * normal register set whose preemption setting within the regsiter set is enabled. 
 * This funnel is selected via macro in the generated VIC vector table. 
 *
 * This code allows an interrupt to start execution in the normal register set
 * and to then be preempted by a higher priority interrupt also assigned to the normal
 * register set. All context must be saved to the stack. 
 *
 * Note: the runtime stack checking feature is not supported by this funnel.
 * The stack pointer is not checked against its lower bound for any stack 
 * operations in this funnel. This is due to a shortage of registers. Other
 * Altera VIC funnels do not save context or control registers to the stack,
 * and therefore there is no limitation against use of the runtime stack 
 * checking feature.
 */
 
#include "system.h"
#include "os/alt_hooks.h"
#include "nios2.h"
#include "altera_vic_macros.h"

    .section ALTERA_VIC_DRIVER_LINKER_SECTION

    /*
     * Explicitly allow the use of r1 (the assembler temporary register)
     * within this code. This register is normally reserved for the use of
     * the assembler.
     */
    .set noat

    .globl alt_normal_preemptive_interrupt
alt_normal_preemptive_interrupt:

    addi sp, sp, -80                /* Allocate space on stack */
    stw ra, 0(sp)                   /* Save RA */
    rdctl ra, status                /* Read STATUS early to avoid usage stall */
    PUSH_R1_TO_R15
    rdctl r1, estatus               /* Read STATUS early to avoid usage stall */
    stw ra, 76(sp)                  /* Save STATUS */
    stw ea, 72(sp)                  /* Save EA */
    stw r1, 68(sp)                  /* Save ESTATUS */

    /* 
     * Notify the operating system that we are at interrupt level. 
     * If present it is C code so it can trash R1 to R15.
     */
    ALT_OS_INT_ENTER_ASM

    /* Turn back on interrupts to allow preemption. Don't assume ra still contains status. */
    rdctl r5, status
    ori r5, r5, NIOS2_STATUS_PIE_MSK                     /* Set STATUS.PIE = 1 */
    orhi r5, r5, ((NIOS2_STATUS_RSIE_MSK >> 16) & 0xffff)/* Set STATUS.RSIE = 1 */
    wrctl status, r5

    /* Get ready to call ISR. */
    ldw r15, 4(et)                  /* Load ISR function pointer */
    ldw r4, 8(et)                   /* Load isr_context */
    callr r15                       /* Call ISR */

    /* Notify the operating system that interrupt processing is complete. */
    ALT_OS_INT_EXIT_ASM

    ldw r5, 76(sp)                  /* Load STATUS */
    ldw r6, 68(sp)                  /* Load ESTATUS */
    ldw ea, 72(sp)                  /* Restore EA */
    ldw ra, 0(sp)                   /* Restore RA */
    wrctl status, r5                /* Restore STATUS */
    wrctl estatus, r6               /* Restore ESTATUS */
    addi ea, ea, -4                 /* Want to re-execute instruction that was interrupted */
    POP_R1_TO_R15
    addi sp, sp, 80                 /* Restore SP */
    eret                            /* Re-execute instruction that was interrupted */
.end
