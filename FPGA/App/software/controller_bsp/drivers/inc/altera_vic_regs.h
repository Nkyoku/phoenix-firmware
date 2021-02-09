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

#ifndef __ALTERA_VIC_REGS_H__
#define __ALTERA_VIC_REGS_H__

#include <io.h>

#define ALTERA_VIC_INT_CONFIG_REG                     0
#define IOADDR_ALTERA_VIC_INT_CONFIG(base, irq)       \
            __IO_CALC_ADDRESS_NATIVE(base, irq)
#define IORD_ALTERA_VIC_INT_CONFIG(base, irq)         IORD(base, irq)
#define IOWR_ALTERA_VIC_INT_CONFIG(base, irq, data)   IOWR(base, irq, data)

#define ALTERA_VIC_INT_CONFIG_RIL_MSK                       (0x3f)
#define ALTERA_VIC_INT_CONFIG_RIL_OFST                      (0)
#define ALTERA_VIC_INT_CONFIG_RNMI_MSK                      (0x40)
#define ALTERA_VIC_INT_CONFIG_RNMI_OFST                     (6)
#define ALTERA_VIC_INT_CONFIG_RRS_MSK                       (0x1f80)
#define ALTERA_VIC_INT_CONFIG_RRS_OFST                      (7)

#define ALTERA_VIC_BUILD_INT_CONFIG(RRS, RIL, NMI)          \
 ( (RRS <<  ALTERA_VIC_INT_CONFIG_RRS_OFST) |               \
   (RIL << ALTERA_VIC_INT_CONFIG_RIL_OFST)  |               \
   (NMI << ALTERA_VIC_INT_CONFIG_RNMI_OFST))

#define ALTERA_VIC_GET_RIL(base, irq)                       \
    ((IORD_ALTERA_VIC_INT_CONFIG(base, irq) & ALTERA_VIC_INT_CONFIG_RIL_MSK) \
        >> ALTERA_VIC_INT_CONFIG_RIL_OFST)

#define ALTERA_VIC_GET_RNMI(base, irq)                      \
    ((IORD_ALTERA_VIC_INT_CONFIG(base, irq) & ALTERA_VIC_INT_CONFIG_RNMI_MSK) \
        >> ALTERA_VIC_INT_CONFIG_RNMI_OFST)

#define ALTERA_VIC_GET_RRS(base, irq)                       \
    ((IORD_ALTERA_VIC_INT_CONFIG(base, irq) & ALTERA_VIC_INT_CONFIG_RRS_MSK) \
        >> ALTERA_VIC_INT_CONFIG_RRS_OFST)


#define ALTERA_VIC_INT_ENABLE_REG                           32
#define IOADDR_ALTERA_VIC_INT_ENABLE(base)                  \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_INT_ENABLE_REG)
#define IORD_ALTERA_VIC_INT_ENABLE(base)                    \
        IORD(base, ALTERA_VIC_INT_ENABLE_REG)
#define IOWR_ALTERA_VIC_INT_ENABLE(base, data)              \
        IOWR(base, ALTERA_VIC_INT_ENABLE_REG, data)


#define ALTERA_VIC_INT_ENABLE_SET_REG                       33
#define IOADDR_ALTERA_VIC_INT_ENABLE_SET(base)              \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_INT_ENABLE_SET_REG)
#define IORD_ALTERA_VIC_INT_ENABLE_SET(base)                \
        IORD(base, ALTERA_VIC_INT_ENABLE_SET_REG)
#define IOWR_ALTERA_VIC_INT_ENABLE_SET(base, data)          \
        IOWR(base, ALTERA_VIC_INT_ENABLE_SET_REG, data)


#define ALTERA_VIC_INT_ENABLE_CLR_REG                       34
#define IOADDR_ALTERA_VIC_INT_ENABLE_CLR(base)              \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_INT_ENABLE_CLR_REG)
#define IORD_ALTERA_VIC_INT_ENABLE_CLR(base)                \
        IORD(base, ALTERA_VIC_INT_ENABLE_CLR_REG)
#define IOWR_ALTERA_VIC_INT_ENABLE_CLR(base, data)          \
        IOWR(base, ALTERA_VIC_INT_ENABLE_CLR_REG, data)


#define ALTERA_VIC_INT_PENDING_REG                          35
#define IOADDR_ALTERA_VIC_INT_PENDING(base)                 \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_INT_PENDING_REG)
#define IORD_ALTERA_VIC_INT_PENDING(base)                   \
        IORD(base, ALTERA_VIC_INT_PENDING_REG)
#define IOWR_ALTERA_VIC_INT_PENDING(base, data)             \
        IOWR(base, ALTERA_VIC_INT_PENDING_REG, data)


#define ALTERA_VIC_INT_RAW_STATUS_REG                       36
#define IOADDR_ALTERA_VIC_INT_RAW_STATUS(base)              \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_INT_RAW_STATUS_REG)
#define IORD_ALTERA_VIC_INT_RAW_STATUS(base)                \
        IORD(base, ALTERA_VIC_INT_RAW_STATUS_REG)
#define IOWR_ALTERA_VIC_INT_RAW_STATUS(base, data)          \
        IOWR(base, ALTERA_VIC_INT_RAW_STATUS_REG, data)



#define ALTERA_VIC_SW_INTERRUPT_REG                         37
#define IOADDR_ALTERA_VIC_SW_INTERRUPT(base)                \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_SW_INTERRUPT_REG)
#define IORD_ALTERA_VIC_SW_INTERRUPT(base)                  \
        IORD(base, ALTERA_VIC_SW_INTERRUPT_REG)
#define IOWR_ALTERA_VIC_SW_INTERRUPT(base, data)            \
        IOWR(base, ALTERA_VIC_SW_INTERRUPT_REG, data)


#define ALTERA_VIC_SW_INTERRUPT_SET_REG                     38
#define IOADDR_ALTERA_VIC_SW_INTERRUPT_SET(base)            \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_SW_INTERRUPT_SET_REG)
#define IORD_ALTERA_VIC_SW_INTERRUPT_SET(base)              \
        IORD(base, ALTERA_VIC_SW_INTERRUPT_SET_REG)
#define IOWR_ALTERA_VIC_SW_INTERRUPT_SET(base, data)        \
        IOWR(base, ALTERA_VIC_SW_INTERRUPT_SET_REG, data)


#define ALTERA_VIC_SW_INTERRUPT_CLR_REG                     39
#define IOADDR_ALTERA_VIC_SW_INTERRUPT_CLR(base)            \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_SW_INTERRUPT_CLR_REG)
#define IORD_ALTERA_VIC_SW_INTERRUPT_CLR(base)              \
        IORD(base, ALTERA_VIC_SW_INTERRUPT_CLR_REG)
#define IOWR_ALTERA_VIC_SW_INTERRUPT_CLR(base, data)        \
        IOWR(base, ALTERA_VIC_SW_INTERRUPT_CLR_REG, data)


#define ALTERA_VIC_CONFIG_REG                               40
#define IOADDR_ALTERA_VIC_CONFIG(base)                      \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_CONFIG_REG)
#define IORD_ALTERA_VIC_CONFIG(base)                        \
        IORD(base, ALTERA_VIC_CONFIG_REG)
#define IOWR_ALTERA_VIC_CONFIG(base, data)                  \
        IOWR(base, ALTERA_VIC_CONFIG_REG, data)
#define ALTERA_VIC_CONFIG_VEC_SIZE_MSK                      (0x7)
#define ALTERA_VIC_CONFIG_VEC_SIZE_OFST                     (0)
#define ALTERA_VIC_CONFIG_DC_MSK                            (0x8)
#define ALTERA_VIC_CONFIG_DC_OFST                           (3)

#define ALTERA_VIC_STATUS_REG                               41
#define IOADDR_ALTERA_VIC_STATUS(base)                      \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_STATUS_REG)
#define IORD_ALTERA_VIC_STATUS(base)                        \
        IORD(base, ALTERA_VIC_STATUS_REG)
#define IOWR_ALTERA_VIC_STATUS(base, data)                  \
        IOWR(base, ALTERA_VIC_STATUS_REG, data)
#define ALTERA_VIC_STATUS_HI_PRI_IRQ_MSK                    (0x3f)
#define ALTERA_VIC_STATUS_HI_PRI_IRQ_OFST                   (0)
#define ALTERA_VIC_STATUS_IP_MSK                            (0x80000000)
#define ALTERA_VIC_STATUS_IP_OFST                           (31)



#define ALTERA_VIC_VEC_TBL_BASE_REG                         42
#define IOADDR_ALTERA_VIC_VEC_TBL_BASE(base)                \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_VEC_TBL_BASE_REG)
#define IORD_ALTERA_VIC_VEC_TBL_BASE(base)                  \
        IORD(base, ALTERA_VIC_VEC_TBL_BASE_REG)
#define IOWR_ALTERA_VIC_VEC_TBL_BASE(base, data)            \
        IOWR(base, ALTERA_VIC_VEC_TBL_BASE_REG, data)


#define ALTERA_VIC_VEC_TBL_ADDR_REG                         43
#define IOADDR_ALTERA_VIC_VEC_TBL_ADDR(base)                \
        __IO_CALC_ADDRESS_NATIVE(base, ALTERA_VIC_VEC_TBL_ADDR_REG)
#define IORD_ALTERA_VIC_VEC_TBL_ADDR(base)                  \
        IORD(base, ALTERA_VIC_VEC_TBL_ADDR_REG)
#define IOWR_ALTERA_VIC_VEC_TBL_ADDR(base, data)            \
        IOWR(base, ALTERA_VIC_VEC_TBL_ADDR_REG, data)

#endif /* __ALTERA_VIC_REGS_H__ */

