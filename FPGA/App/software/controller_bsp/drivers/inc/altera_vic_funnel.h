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
#ifndef __ALTERA_VIC_FUNNEL_H__
#define __ALTERA_VIC_FUNNEL_H__

/***************************************************
    Macros for each funnel, used by vector table
****************************************************/


#define ALT_VIC_VEC_SIZE_DEFAULT                    16  /* bytes */

/*
 * This macro fills unused spaces with zero. It only adds padding if vector
 * entry size is greater than ALT_VIC_VEC_SIZE_DEFAULT
 */
 .macro ALT_FUNNEL_ADD_PADDING  vec_size

.if ((\vec_size-ALT_VIC_VEC_SIZE_DEFAULT) > 0)
    .zero  (\vec_size-ALT_VIC_VEC_SIZE_DEFAULT)  /* number of bytes to fill */
.endif

.endm

 /*-----------------------------------------------------------
 * Funnel macro for interrupts run in shadow register set and
 * preemption disabled.
 * [in] vec_size: Vector entry size (in bytes)
 * ---------------------------------------------------------*/
 .macro ALT_SHADOW_NON_PREEMPTIVE_INTERRUPT  vec_size
    nextpc et
    jmpi alt_shadow_non_preemptive_interrupt
    .word   0x0                                /* ISR */
    .word   0x0                                /* ISR context */

    ALT_FUNNEL_ADD_PADDING \vec_size

.endm


/*-----------------------------------------------------------
 * Funnel macro for interrupts run in shadow register set and
 * preemption enabled.
 * [in] vec_size: Vector entry size (in bytes)
 * ---------------------------------------------------------*/
.macro ALT_SHADOW_PREEMPTIVE_INTERRUPT vec_size
    nextpc et
    jmpi alt_shadow_preemptive_interrupt
    .word   0x0                                 /* ISR */
    .word   0x0                                 /* ISR context */

    ALT_FUNNEL_ADD_PADDING \vec_size

.endm

 /*-----------------------------------------------------------
 * Funnel macro for interrupts run in normal register set and
 * preemption disabled.
 * [in] vec_size: Vector entry size (in bytes)
 * ---------------------------------------------------------*/
 .macro ALT_NORMAL_NON_PREEMPTIVE_INTERRUPT  vec_size
    nextpc et
    jmpi alt_normal_non_preemptive_interrupt
    .word   0x0                                /* ISR */
    .word   0x0                                /* ISR context */

    ALT_FUNNEL_ADD_PADDING \vec_size

.endm


/*-----------------------------------------------------------
 * Funnel macro for interrupts run in normal register set and
 * preemption enabled.
 * [in] vec_size: Vector entry size (in bytes)
 * ---------------------------------------------------------*/
.macro ALT_NORMAL_PREEMPTIVE_INTERRUPT vec_size
    nextpc et
    jmpi alt_normal_preemptive_interrupt
    .word   0x0                                 /* ISR */
    .word   0x0                                 /* ISR context */

    ALT_FUNNEL_ADD_PADDING \vec_size

.endm

 /*-----------------------------------------------------------
 * Funnel macro for non-maskable interrupts run in shadow
 * register set and preemption disabled.
 * [in] vec_size: Vector entry size (in bytes)
 * ---------------------------------------------------------*/
 .macro ALT_SHADOW_NON_PREEMPTIVE_NMI_INTERRUPT  vec_size
    nextpc et
    jmpi alt_shadow_non_preemptive_nmi_interrupt
    .word   0x0                                /* ISR */
    .word   0x0                                /* ISR context */

    ALT_FUNNEL_ADD_PADDING \vec_size

.endm


#endif /* __ALTERA_VIC_FUNNEL_H__ */
