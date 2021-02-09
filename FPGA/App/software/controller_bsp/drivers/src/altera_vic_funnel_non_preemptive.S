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

#include "system.h"
#include "os/alt_hooks.h"
#include "nios2.h"

/*----------------------------------------------------------
 *   Shadow register set and preemption disabled
 *---------------------------------------------------------*/
    .section ALTERA_VIC_DRIVER_LINKER_SECTION
    .set nobreak
    .globl alt_shadow_non_preemptive_interrupt
alt_shadow_non_preemptive_interrupt:

    ldw r16, 4(et)               /* load ISR pointer */

    /*
     * If user stack is enabled and no other interrupts run in this
     * register set. SP already set up at boot time.
     */

#ifdef ALT_INTERRUPT_STACK
    /*
     * Get STATUS.IH from before the interrupt
     */

    andi r5, sstatus, NIOS2_STATUS_IH_MSK

    /*
     * If it wasn't an interrupt before, SP is already set up for us.
     */

    beq r5, r0, .Lskip_get_sp_shadow_non_preemptive
#endif

     /*
      * Get SP from previous register set.
      */

    rdprs sp, sp, 0

.Lskip_get_sp_shadow_non_preemptive:

  /*
   * Notify the operating system that we are at interrupt level.
   */
    ALT_OS_INT_ENTER_ASM

    /* Re-enable interrupts if preemption into a new register set is enabled */
#ifdef ALTERA_VIC_DRIVER_PREEMPTION_INTO_NEW_REGISTER_SET_ENABLED
    rdctl r5, status
    ori   r5, r5, NIOS2_STATUS_PIE_MSK
    wrctl status, r5
#endif

    ldw r4, 8(et)               /* load isr_context */
    callr r16                   /* call ISR */

    /*
     * Notify the operating system that interrupt processing is complete.
     */
    ALT_OS_INT_EXIT_ASM

    addi ea, ea, -4             /* instruction that caused exception */
    eret

