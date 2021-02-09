/******************************************************************************
*                                                                             *
* License Agreement                                                           *
*                                                                             *
* Copyright (c) 2009 Altera Corporation, San Jose, California, USA.           *
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
 * This code implemennts the interrupt funnel for interrupts assigned to a 
 * shadow register set whose preemtion setting is enabled. This funnel is 
 * selected via macro in the generated VIC vector table. 
 *
 * This code allows an interrupt to start execution in a particular shadow set,
 * and to then be preempted by a higher priority interrupt assigned to the same
 * shadow set. Because of this, this funnel is substantially more complex and
 * has a higher execution latency than other Altera VIC funnels. Interrupts 
 * which preempt another in the same shadow set require context to be saved 
 * to the stack. All interrupts using this shadow set require that certain 
 * control registers be saved to the stack.
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

/*----------------------------------------------------------
 *   Shadow register set and preemption enabled
 *---------------------------------------------------------*/
    .set nobreak
    .globl alt_shadow_preemptive_interrupt
alt_shadow_preemptive_interrupt:

    /*
     * If (SSTATUS.SRS==1), get SP, else sp is already set up for us.
     */

    blt sstatus, r0, .Lget_sp
    addi sp, sp, -80

    /*
     * If (SSTATUS.SRS==0), pre-emption occurred; need to save registers.
     */

    bge sstatus, r0, .Lpush_all

.Lreturn_from_branch:

    stw   sstatus,  68(sp)          /* sstatus */
    stw   ea,  72(sp)               /* ea */
    rdctl r5, status
    stw   r5,  76(sp)               /* status */

    /*
     * Notify the operating system that we are at interrupt level.
     */
    ALT_OS_INT_ENTER_ASM

    ldw r15, 4(et)                  /* load ISR pointer */
    ldw r4, 8(et)                   /* load isr_context */

    rdctl r5, status
    ori r5, r5, NIOS2_STATUS_PIE_MSK                     /* Set STATUS.PIE = 1 */
    orhi r5, r5, ((NIOS2_STATUS_RSIE_MSK >> 16) & 0xffff)/* Set STATUS.RSIE = 1 */
    wrctl status, r5

    callr r15                       /* call ISR */

    /*
     * Notify the operating system that interrupt processing is complete.
     */
    ALT_OS_INT_EXIT_ASM

    ldw r5, 76(sp)
    wrctl status, r5                /* restore status, disable pre-emption */

    ldw   ea,  72(sp)
    addi ea, ea, -4                 /* instruction that caused exception */
    ldw   sstatus,  68(sp)
    bge sstatus, r0, .Lpop_all      /* examine SSTATUS.SRS */

.Lreturn_from_pop_all:
    addi sp, sp, 80
    eret

.Lpush_all:
    stw   ra,  0(sp)
    PUSH_R1_TO_R15
    br .Lreturn_from_branch

.Lpop_all:
    ldw   ra,   0(sp)
    POP_R1_TO_R15
    br .Lreturn_from_pop_all

.Lget_sp:
#ifdef ALT_INTERRUPT_STACK

    /*
     * We are now in a new shadow set, does not need to push it to the stack.
     */

    andi r5, sstatus, NIOS2_STATUS_IH_MSK
    beq  r5, r0, .Lskip_get_sp_shadow_preemptive
#endif

    /*
     * Get SP from previous register set.
     */

    rdprs sp, sp, -80
    br .Lreturn_from_branch

.Lskip_get_sp_shadow_preemptive:
    addi sp, sp, -80
    br .Lreturn_from_branch


.end

