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
#include "altera_vic_irq.h"
#include "altera_vic_regs.h"

/* This macro write the data to address addr. */
#define ALT_VECTOR_TABLE_WRITE(addr, data)          \
    *(alt_u32* const)(addr) = (alt_u32)data

/** @Function Description:  This function registers an ISR. If the function is
  *                         successful, the requested interrupt is enabled, isr
  *                         and isr_context parameters updated to vector table
  *                         upon return. Calls to this function replace
  *                         previously registered handler for interrupt irq.
  *                         If isr is set to null, the interrupt is disabled.
  * @API Type:              External
  * @param ic_id            The interrupt controller ID.
  * @param irq              IRQ number
  * @param isr              isr is the function that is called  when the 
  *                         interrupt is active, isr_context is the input 
  *                         arguments to isr.
  * @param isr_context      pointer of type void*. The isr_context points to a 
  *                         per-instance data structure associate with the 
  *                         device driver instance. The isr_context is passed  
  *                         as the input argument to the isr function.
  * @param flags            TBD
  * @return                 0 if successful, else error (-1)
  *
  * Note:  Users who locate their ISR codes directly to vector table must not
  *        call to this function.
  */
int alt_ic_isr_register(alt_u32 ic_id,
                        alt_u32 irq,
                        alt_isr_func isr,
                        void *isr_context,
                        void* flags)
{
    alt_vic_dev *pdev;
    alt_u32 *offset;
    alt_irq_context status;

    if(ic_id >= ALT_NUM_EXTERNAL_INTERRUPT_CONTROLLERS)
        return -1;

    /* Get VIC handle */
    pdev = alt_vic_dev_list[ic_id];

    if(irq < pdev->num_of_intr_ports)
    {
        /* Disable all interrupts */
        status = alt_irq_disable_all ();

        /* Get table offset */
        offset = pdev->vec_addr + (irq * pdev->vec_size);

        /* Write ISR and ISR context to vector table */
        ALT_VECTOR_TABLE_WRITE(offset+2, isr);
        ALT_VECTOR_TABLE_WRITE(offset+3, isr_context);

        if(isr)
        {
            /* Enable this IRQ on the VIC */
            IOWR_ALTERA_VIC_INT_ENABLE_SET(pdev->base, 1 << irq);
        }
        else
        {
            /* Disable this IRQ on the VIC */
            IOWR_ALTERA_VIC_INT_ENABLE_CLR(pdev->base, 1 << irq);
        }

        /* Re-enable all interrupts */
        alt_irq_enable_all(status);

        return 0;
    }

    return -1;
}
