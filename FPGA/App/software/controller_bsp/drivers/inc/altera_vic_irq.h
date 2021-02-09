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
#ifndef __ALTERA_VIC_IRQ_H__
#define __ALTERA_VIC_IRQ_H__

#include "alt_types.h"
#include "system.h"
#include "sys/alt_irq.h"
#include "altera_vic_regs.h"

#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */

#define ALT_VIC_MAX_INTR_PORTS          (32)

typedef struct
{
  void          *base;                  /* Base address of VIC */
  alt_u32       intr_controller_id;     /* Interrupt controller ID */
  alt_u32       num_of_intr_ports;      /* Number of interrupt ports */
  alt_u32       ril_width;              /* RIL width */
  alt_u32       daisy_chain_present;    /* Daisy-chain input present */
  alt_u32       vec_size;               /* Vector size */
  void          *vec_addr;              /* Vector table base address */
  alt_u32       int_config[ALT_VIC_MAX_INTR_PORTS];   /* INT_CONFIG settings
                                                         for each interrupt */
} alt_vic_dev;


#define GET_INT_CONFIG(name, irq)                                              \
    (ALTERA_VIC_BUILD_INT_CONFIG(ALTERA_VIC_DRIVER_##name##_##IRQ##irq##_RRS,  \
                                 ALTERA_VIC_DRIVER_##name##_##IRQ##irq##_RIL,  \
                                 ALTERA_VIC_DRIVER_##name##_##IRQ##irq##_RNMI))

/*-------------------------------------------------------*/
/*                  Macros                               */
/*-------------------------------------------------------*/

/*
 * The macro ALTERA_VIC_IRQ_INSTANCE is used by the alt_irq_init()
 * function in the auto-generated file alt_sys_init.c to create an
 * instance of this interrupt controller device driver state.
 */
#define ALTERA_VIC_IRQ_INSTANCE(name, dev)          \
  extern void name##_VECTOR_TABLE (void);       \
  static alt_vic_dev dev =                      \
  {                                             \
    (void*) name##_BASE,                        \
    name##_INTERRUPT_CONTROLLER_ID,             \
    name##_NUMBER_OF_INT_PORTS,                 \
    name##_RIL_WIDTH,                           \
    name##_DAISY_CHAIN_ENABLE,                  \
    name##_VEC_SIZE,                            \
    (void*)name##_VEC_TBL_BASE,                 \
    {                                           \
        GET_INT_CONFIG(name, 0),   /*IRQ 0*/    \
        GET_INT_CONFIG(name, 1),   /*IRQ 1*/    \
        GET_INT_CONFIG(name, 2),   /*IRQ 2*/    \
        GET_INT_CONFIG(name, 3),   /*IRQ 3*/    \
        GET_INT_CONFIG(name, 4),   /*IRQ 4*/    \
        GET_INT_CONFIG(name, 5),   /*IRQ 5*/    \
        GET_INT_CONFIG(name, 6),   /*IRQ 6*/    \
        GET_INT_CONFIG(name, 7),   /*IRQ 7*/    \
        GET_INT_CONFIG(name, 8),   /*IRQ 8*/    \
        GET_INT_CONFIG(name, 9),   /*IRQ 9*/    \
        GET_INT_CONFIG(name, 10),  /*IRQ 10*/   \
        GET_INT_CONFIG(name, 11),  /*IRQ 11*/   \
        GET_INT_CONFIG(name, 12),  /*IRQ 12*/   \
        GET_INT_CONFIG(name, 13),  /*IRQ 13*/   \
        GET_INT_CONFIG(name, 14),  /*IRQ 14*/   \
        GET_INT_CONFIG(name, 15),  /*IRQ 15*/   \
        GET_INT_CONFIG(name, 16),  /*IRQ 16*/   \
        GET_INT_CONFIG(name, 17),  /*IRQ 17*/   \
        GET_INT_CONFIG(name, 18),  /*IRQ 18*/   \
        GET_INT_CONFIG(name, 19),  /*IRQ 19*/   \
        GET_INT_CONFIG(name, 20),  /*IRQ 20*/   \
        GET_INT_CONFIG(name, 21),  /*IRQ 21*/   \
        GET_INT_CONFIG(name, 22),  /*IRQ 22*/   \
        GET_INT_CONFIG(name, 23),  /*IRQ 23*/   \
        GET_INT_CONFIG(name, 24),  /*IRQ 24*/   \
        GET_INT_CONFIG(name, 25),  /*IRQ 25*/   \
        GET_INT_CONFIG(name, 26),  /*IRQ 26*/   \
        GET_INT_CONFIG(name, 27),  /*IRQ 27*/   \
        GET_INT_CONFIG(name, 28),  /*IRQ 28*/   \
        GET_INT_CONFIG(name, 29),  /*IRQ 29*/   \
        GET_INT_CONFIG(name, 30),  /*IRQ 30*/   \
        GET_INT_CONFIG(name, 31),  /*IRQ 31*/   \
    }                                           \
  }

/*
 * The macro ALTERA_VIC_IRQ_INIT is used by the alt_irq_init() routine
 * in the auto-generated file alt_sys_init.c to initialize an instance
 * of the interrupt controller device driver state.
 */
#define ALTERA_VIC_IRQ_INIT(name, dev)            \
{                                                 \
    alt_vic_irq_init(&dev);                       \
}

/*
 * The Altera HAL requires that all external device drivers be the
 * same module class type.  So, the number of external interrupt controllers
 * is equal to the number of VIC instances.
 */
extern alt_vic_dev* alt_vic_dev_list[ALT_NUM_EXTERNAL_INTERRUPT_CONTROLLERS];

/*-------------------------------------------------------*/
/*                  Public Functions                     */
/*-------------------------------------------------------*/
extern int alt_ic_isr_register(alt_u32 ic_id,
                        alt_u32 irq,
                        alt_isr_func isr,
                        void *isr_context,
                        void *flags);
extern int alt_ic_irq_enable (alt_u32 ic_id, alt_u32 irq);
extern int alt_ic_irq_disable(alt_u32 ic_id, alt_u32 irq);
extern alt_u32 alt_ic_irq_enabled(alt_u32 ic_id, alt_u32 irq);
extern int alt_vic_irq_set_level(alt_u32 ic_id, alt_u32 irq, alt_u32 level);
extern int alt_vic_sw_interrupt_set(alt_u32 ic_id, alt_u32 irq);
extern int alt_vic_sw_interrupt_clear(alt_u32 ic_id, alt_u32 irq);
extern alt_u32 alt_vic_sw_interrupt_status(alt_u32 ic_id, alt_u32 irq);

/*-------------------------------------------------------*/
/*                  Private Function                     */
/*-------------------------------------------------------*/

/*
 * alt_vic_irq_init() is called by the auto-generated function
 * alt_irq_init() once for each VIC instance.
 * The alt_vic_irq_init() routine is called using the ALTERA_VIC_IRQ_INIT macro.
 */
extern void alt_vic_irq_init (alt_vic_dev* vic_dev);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __ALTERA_VIC_IRQ_H__ */
