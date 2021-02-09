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
#include "nios2.h"
#include "altera_vic_irq.h"
#include "altera_vic_regs.h"

/* This pointer array stores the pointer of the VIC instances */
alt_vic_dev* alt_vic_dev_list[ALT_NUM_EXTERNAL_INTERRUPT_CONTROLLERS];


/** @Function Description:  This function called by macro
  *                         ALTERA_VIC_IRQ_INIT( ) to setup the VIC by
  *                         alt_irq_init() in alt_sys_init.c during startup.
  * @API Type:              Internal
  * @param pdev             pointer of type alt_vic_dev, main Device Structure.
  * @return                 None
  */
void alt_vic_irq_init (alt_vic_dev* pdev)
{
    alt_u32 i;
    static const int VEC_SIZE_ARRAY[]={4, 8, 16, 32, 64, 128, 256, 512};

    /* Stores the VIC instance to list */
    alt_vic_dev_list[pdev->intr_controller_id] = pdev;

    /* Disables all interrupts */
    IOWR_ALTERA_VIC_INT_ENABLE_CLR(pdev->base, 0xFFFFFFFF);

    /* Initialize INT_CONFIG for all interrupts in this VIC */
    for(i = 0; i < pdev->num_of_intr_ports; i++)
    {
        IOWR_ALTERA_VIC_INT_CONFIG(pdev->base, i, pdev->int_config[i]);
    }

    /* Set vector size and disable/enable daisy-chained input */
    for(i = 0; i < sizeof(VEC_SIZE_ARRAY)/sizeof(VEC_SIZE_ARRAY[0]); i++)
    {
        if(VEC_SIZE_ARRAY[i] == pdev->vec_size)
        {
            IOWR_ALTERA_VIC_CONFIG(pdev->base,
                (i | (pdev->daisy_chain_present << ALTERA_VIC_CONFIG_DC_OFST)));
            break;
        }
    }

    /* Set vector table base address */
    IOWR_ALTERA_VIC_VEC_TBL_BASE(pdev->base,((alt_u32)pdev->vec_addr));
}

