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
#ifndef __ALTERA_VIC_MACROS_H__
#define __ALTERA_VIC_MACROS_H__

/*----------------------------------------------------------
 *   Macro to store R1 to R15 to stack
 *---------------------------------------------------------*/
.macro PUSH_R1_TO_R15
    stw   r1,   8(sp)
    stw   r2,  12(sp)
    stw   r3,  16(sp)
    stw   r4,  20(sp)
    stw   r5,  24(sp)
    stw   r6,  28(sp)
    stw   r7,  32(sp)
    stw   r8,  36(sp)
    stw   r9,  40(sp)
    stw   r10, 44(sp)
    stw   r11, 48(sp)
    stw   r12, 52(sp)
    stw   r13, 56(sp)
    stw   r14, 60(sp)
    stw   r15, 64(sp)
.endm

/*----------------------------------------------------------
 *   Macro to restore R1 to R15 from stack
 *---------------------------------------------------------*/
.macro POP_R1_TO_R15
    ldw   r1,   8(sp)
    ldw   r2,  12(sp)
    ldw   r3,  16(sp)
    ldw   r4,  20(sp)
    ldw   r5,  24(sp)
    ldw   r6,  28(sp)
    ldw   r7,  32(sp)
    ldw   r8,  36(sp)
    ldw   r9,  40(sp)
    ldw   r10, 44(sp)
    ldw   r11, 48(sp)
    ldw   r12, 52(sp)
    ldw   r13, 56(sp)
    ldw   r14, 60(sp)
    ldw   r15, 64(sp)
.endm

#endif /* __ALTERA_VIC_MACROS_H__ */
