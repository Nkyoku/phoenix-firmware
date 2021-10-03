/*
 * system.h - SOPC Builder system and BSP software package information
 *
 * Machine generated for CPU 'nios_0' in SOPC Builder design 'controller'
 * SOPC Builder design path: ../../controller.sopcinfo
 *
 * Generated: Sat Oct 02 23:49:44 JST 2021
 */

/*
 * DO NOT MODIFY THIS FILE
 *
 * Changing this file will have subtle consequences
 * which will almost certainly lead to a nonfunctioning
 * system. If you do modify this file, be aware that your
 * changes will be overwritten and lost when this file
 * is generated again.
 *
 * DO NOT MODIFY THIS FILE
 */

/*
 * License Agreement
 *
 * Copyright (c) 2008
 * Altera Corporation, San Jose, California, USA.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * This agreement shall be governed in all respects by the laws of the State
 * of California and by the laws of the United States of America.
 */

#ifndef __SYSTEM_H_
#define __SYSTEM_H_

/* Include definitions from linker script generator */
#include "linker.h"


/*
 * CPU configuration
 *
 */

#define ALT_CPU_ARCHITECTURE "altera_nios2_gen2"
#define ALT_CPU_BIG_ENDIAN 0
#define ALT_CPU_BREAK_ADDR 0x0000b820
#define ALT_CPU_CPU_ARCH_NIOS2_R1
#define ALT_CPU_CPU_FREQ 75000000u
#define ALT_CPU_CPU_ID_SIZE 1
#define ALT_CPU_CPU_ID_VALUE 0x00000000
#define ALT_CPU_CPU_IMPLEMENTATION "fast"
#define ALT_CPU_DATA_ADDR_WIDTH 0x10
#define ALT_CPU_DCACHE_LINE_SIZE 0
#define ALT_CPU_DCACHE_LINE_SIZE_LOG2 0
#define ALT_CPU_DCACHE_SIZE 0
#define ALT_CPU_EIC_PRESENT
#define ALT_CPU_EXCEPTION_ADDR 0x00000020
#define ALT_CPU_FLASH_ACCELERATOR_LINES 0
#define ALT_CPU_FLASH_ACCELERATOR_LINE_SIZE 0
#define ALT_CPU_FLUSHDA_SUPPORTED
#define ALT_CPU_FREQ 75000000
#define ALT_CPU_HARDWARE_DIVIDE_PRESENT 1
#define ALT_CPU_HARDWARE_MULTIPLY_PRESENT 1
#define ALT_CPU_HARDWARE_MULX_PRESENT 0
#define ALT_CPU_HAS_DEBUG_CORE 1
#define ALT_CPU_HAS_DEBUG_STUB
#define ALT_CPU_HAS_DIVISION_ERROR_EXCEPTION
#define ALT_CPU_HAS_EXTRA_EXCEPTION_INFO
#define ALT_CPU_HAS_ILLEGAL_INSTRUCTION_EXCEPTION
#define ALT_CPU_HAS_JMPI_INSTRUCTION
#define ALT_CPU_ICACHE_LINE_SIZE 0
#define ALT_CPU_ICACHE_LINE_SIZE_LOG2 0
#define ALT_CPU_ICACHE_SIZE 0
#define ALT_CPU_INST_ADDR_WIDTH 0x10
#define ALT_CPU_NAME "nios_0"
#define ALT_CPU_NUM_OF_SHADOW_REG_SETS 3
#define ALT_CPU_OCI_VERSION 1
#define ALT_CPU_RESET_ADDR 0x00000000


/*
 * CPU configuration (with legacy prefix - don't use these anymore)
 *
 */

#define NIOS2_BIG_ENDIAN 0
#define NIOS2_BREAK_ADDR 0x0000b820
#define NIOS2_CPU_ARCH_NIOS2_R1
#define NIOS2_CPU_FREQ 75000000u
#define NIOS2_CPU_ID_SIZE 1
#define NIOS2_CPU_ID_VALUE 0x00000000
#define NIOS2_CPU_IMPLEMENTATION "fast"
#define NIOS2_DATA_ADDR_WIDTH 0x10
#define NIOS2_DCACHE_LINE_SIZE 0
#define NIOS2_DCACHE_LINE_SIZE_LOG2 0
#define NIOS2_DCACHE_SIZE 0
#define NIOS2_EIC_PRESENT
#define NIOS2_EXCEPTION_ADDR 0x00000020
#define NIOS2_FLASH_ACCELERATOR_LINES 0
#define NIOS2_FLASH_ACCELERATOR_LINE_SIZE 0
#define NIOS2_FLUSHDA_SUPPORTED
#define NIOS2_HARDWARE_DIVIDE_PRESENT 1
#define NIOS2_HARDWARE_MULTIPLY_PRESENT 1
#define NIOS2_HARDWARE_MULX_PRESENT 0
#define NIOS2_HAS_DEBUG_CORE 1
#define NIOS2_HAS_DEBUG_STUB
#define NIOS2_HAS_DIVISION_ERROR_EXCEPTION
#define NIOS2_HAS_EXTRA_EXCEPTION_INFO
#define NIOS2_HAS_ILLEGAL_INSTRUCTION_EXCEPTION
#define NIOS2_HAS_JMPI_INSTRUCTION
#define NIOS2_ICACHE_LINE_SIZE 0
#define NIOS2_ICACHE_LINE_SIZE_LOG2 0
#define NIOS2_ICACHE_SIZE 0
#define NIOS2_INST_ADDR_WIDTH 0x10
#define NIOS2_NUM_OF_SHADOW_REG_SETS 3
#define NIOS2_OCI_VERSION 1
#define NIOS2_RESET_ADDR 0x00000000


/*
 * Custom instruction macros
 *
 */

#define ALT_CI_FLOAT32TO16_0(A) __builtin_custom_ini(ALT_CI_FLOAT32TO16_0_N,(A))
#define ALT_CI_FLOAT32TO16_0_N 0x0
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0(n,A,B) __builtin_custom_inii(ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+(n&ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N_MASK),(A),(B))
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1(n,A,B) __builtin_custom_inii(ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N+(n&ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N_MASK),(A),(B))
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_FADDS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N+5
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_FDIVS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N+7
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_FIXSI_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N+1
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_FLOATIS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N+2
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_FMULS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N+4
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_FSQRTS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N+3
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_FSUBS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N+6
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N 0xf8
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N_MASK ((1<<3)-1)
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_ROUND_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_N+0
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FABSS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+0
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FCMPEQS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+3
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FCMPGES_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+4
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FCMPGTS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+5
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FCMPLES_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+6
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FCMPLTS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+7
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FCMPNES_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+2
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FMAXS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+8
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FMINS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+9
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FNEGS_N ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N+1
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N 0xe0
#define ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_N_MASK ((1<<4)-1)
#define fmaxf(A,B) __builtin_custom_fnff(ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FMAXS_N,(A),(B))
#define fminf(A,B) __builtin_custom_fnff(ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_FMINS_N,(A),(B))
#define lroundf(A) __builtin_custom_inf(ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_ROUND_N,(A))
#define sqrtf(A) __builtin_custom_fnf(ALT_CI_NIOS_CUSTOM_INSTR_FLOATING_POINT_2_0_1_FSQRTS_N,(A))


/*
 * Define for each module class mastered by the CPU
 *
 */

#define __ALTERA_AVALON_JTAG_UART
#define __ALTERA_AVALON_ONCHIP_MEMORY2
#define __ALTERA_AVALON_PERFORMANCE_COUNTER
#define __ALTERA_AVALON_PIO
#define __ALTERA_AVALON_SPI
#define __ALTERA_AVALON_SYSID_QSYS
#define __ALTERA_AVALON_TIMER
#define __ALTERA_MSGDMA
#define __ALTERA_NIOS2_GEN2
#define __ALTERA_NIOS_CUSTOM_INSTR_FLOATING_POINT_2
#define __ALTERA_VIC
#define __FLOAT32TO16
#define __I2C_MASTER
#define __IMU_SPIM
#define __MOTOR_CONTROLLER
#define __VECTOR_CONTROLLER_MASTER


/*
 * System configuration
 *
 */

#define ALT_DEVICE_FAMILY "Cyclone 10 LP"
#define ALT_ENHANCED_INTERRUPT_API_PRESENT
#define ALT_IRQ_BASE NULL
#define ALT_LOG_PORT "/dev/null"
#define ALT_LOG_PORT_BASE 0x0
#define ALT_LOG_PORT_DEV null
#define ALT_LOG_PORT_TYPE ""
#define ALT_NUM_EXTERNAL_INTERRUPT_CONTROLLERS 1
#define ALT_NUM_INTERNAL_INTERRUPT_CONTROLLERS 0
#define ALT_NUM_INTERRUPT_CONTROLLERS 1
#define ALT_STDERR "/dev/jtag_uart_0"
#define ALT_STDERR_BASE 0xc900
#define ALT_STDERR_DEV jtag_uart_0
#define ALT_STDERR_IS_JTAG_UART
#define ALT_STDERR_PRESENT
#define ALT_STDERR_TYPE "altera_avalon_jtag_uart"
#define ALT_STDIN "/dev/jtag_uart_0"
#define ALT_STDIN_BASE 0xc900
#define ALT_STDIN_DEV jtag_uart_0
#define ALT_STDIN_IS_JTAG_UART
#define ALT_STDIN_PRESENT
#define ALT_STDIN_TYPE "altera_avalon_jtag_uart"
#define ALT_STDOUT "/dev/jtag_uart_0"
#define ALT_STDOUT_BASE 0xc900
#define ALT_STDOUT_DEV jtag_uart_0
#define ALT_STDOUT_IS_JTAG_UART
#define ALT_STDOUT_PRESENT
#define ALT_STDOUT_TYPE "altera_avalon_jtag_uart"
#define ALT_SYSTEM_NAME "controller"


/*
 * altera_vic_driver configuration
 *
 */

#define ALTERA_VIC_DRIVER_ENABLE_PREEMPTION_RS_0 0
#define ALTERA_VIC_DRIVER_ENABLE_PREEMPTION_RS_1 0
#define ALTERA_VIC_DRIVER_ENABLE_PREEMPTION_RS_2 0
#define ALTERA_VIC_DRIVER_ENABLE_PREEMPTION_RS_3 0
#define ALTERA_VIC_DRIVER_ISR_PREEMPTION_ENABLED
#define ALTERA_VIC_DRIVER_LINKER_SECTION .text
#define ALTERA_VIC_DRIVER_PREEMPTION_INTO_NEW_REGISTER_SET_ENABLED
#define ALTERA_VIC_DRIVER_VIC_0_IRQ0_RIL 1
#define ALTERA_VIC_DRIVER_VIC_0_IRQ0_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ0_RRS 1
#define ALTERA_VIC_DRIVER_VIC_0_IRQ10_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ10_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ10_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ11_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ11_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ11_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ12_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ12_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ12_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ13_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ13_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ13_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ14_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ14_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ14_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ15_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ15_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ15_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ16_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ16_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ16_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ17_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ17_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ17_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ18_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ18_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ18_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ19_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ19_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ19_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ1_RIL 3
#define ALTERA_VIC_DRIVER_VIC_0_IRQ1_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ1_RRS 3
#define ALTERA_VIC_DRIVER_VIC_0_IRQ20_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ20_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ20_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ21_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ21_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ21_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ22_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ22_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ22_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ23_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ23_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ23_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ24_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ24_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ24_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ25_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ25_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ25_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ26_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ26_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ26_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ27_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ27_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ27_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ28_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ28_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ28_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ29_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ29_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ29_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ2_RIL 2
#define ALTERA_VIC_DRIVER_VIC_0_IRQ2_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ2_RRS 2
#define ALTERA_VIC_DRIVER_VIC_0_IRQ30_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ30_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ30_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ31_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ31_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ31_RRS 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ3_RIL 3
#define ALTERA_VIC_DRIVER_VIC_0_IRQ3_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ3_RRS 3
#define ALTERA_VIC_DRIVER_VIC_0_IRQ4_RIL 1
#define ALTERA_VIC_DRIVER_VIC_0_IRQ4_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ4_RRS 1
#define ALTERA_VIC_DRIVER_VIC_0_IRQ5_RIL 1
#define ALTERA_VIC_DRIVER_VIC_0_IRQ5_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ5_RRS 1
#define ALTERA_VIC_DRIVER_VIC_0_IRQ6_RIL 1
#define ALTERA_VIC_DRIVER_VIC_0_IRQ6_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ6_RRS 1
#define ALTERA_VIC_DRIVER_VIC_0_IRQ7_RIL 3
#define ALTERA_VIC_DRIVER_VIC_0_IRQ7_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ7_RRS 3
#define ALTERA_VIC_DRIVER_VIC_0_IRQ8_RIL 3
#define ALTERA_VIC_DRIVER_VIC_0_IRQ8_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ8_RRS 3
#define ALTERA_VIC_DRIVER_VIC_0_IRQ9_RIL 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ9_RNMI 0
#define ALTERA_VIC_DRIVER_VIC_0_IRQ9_RRS 0
#define VIC_0_VEC_SIZE 16
#define VIC_0_VEC_TBL_BASE VIC_0_VECTOR_TABLE


/*
 * data_ram_0 configuration
 *
 */

#define ALT_MODULE_CLASS_data_ram_0 altera_avalon_onchip_memory2
#define DATA_RAM_0_ALLOW_IN_SYSTEM_MEMORY_CONTENT_EDITOR 0
#define DATA_RAM_0_ALLOW_MRAM_SIM_CONTENTS_ONLY_FILE 0
#define DATA_RAM_0_BASE 0x8000
#define DATA_RAM_0_CONTENTS_INFO ""
#define DATA_RAM_0_DUAL_PORT 1
#define DATA_RAM_0_GUI_RAM_BLOCK_TYPE "AUTO"
#define DATA_RAM_0_INIT_CONTENTS_FILE "controller_data_ram_0"
#define DATA_RAM_0_INIT_MEM_CONTENT 1
#define DATA_RAM_0_INSTANCE_ID "NONE"
#define DATA_RAM_0_IRQ -1
#define DATA_RAM_0_IRQ_INTERRUPT_CONTROLLER_ID -1
#define DATA_RAM_0_NAME "/dev/data_ram_0"
#define DATA_RAM_0_NON_DEFAULT_INIT_FILE_ENABLED 0
#define DATA_RAM_0_RAM_BLOCK_TYPE "AUTO"
#define DATA_RAM_0_READ_DURING_WRITE_MODE "OLD_DATA"
#define DATA_RAM_0_SINGLE_CLOCK_OP 1
#define DATA_RAM_0_SIZE_MULTIPLE 1
#define DATA_RAM_0_SIZE_VALUE 8192
#define DATA_RAM_0_SPAN 8192
#define DATA_RAM_0_TYPE "altera_avalon_onchip_memory2"
#define DATA_RAM_0_WRITABLE 1


/*
 * data_ram_1 configuration
 *
 */

#define ALT_MODULE_CLASS_data_ram_1 altera_avalon_onchip_memory2
#define DATA_RAM_1_ALLOW_IN_SYSTEM_MEMORY_CONTENT_EDITOR 0
#define DATA_RAM_1_ALLOW_MRAM_SIM_CONTENTS_ONLY_FILE 0
#define DATA_RAM_1_BASE 0xb000
#define DATA_RAM_1_CONTENTS_INFO ""
#define DATA_RAM_1_DUAL_PORT 1
#define DATA_RAM_1_GUI_RAM_BLOCK_TYPE "AUTO"
#define DATA_RAM_1_INIT_CONTENTS_FILE "controller_data_ram_1"
#define DATA_RAM_1_INIT_MEM_CONTENT 1
#define DATA_RAM_1_INSTANCE_ID "NONE"
#define DATA_RAM_1_IRQ -1
#define DATA_RAM_1_IRQ_INTERRUPT_CONTROLLER_ID -1
#define DATA_RAM_1_NAME "/dev/data_ram_1"
#define DATA_RAM_1_NON_DEFAULT_INIT_FILE_ENABLED 0
#define DATA_RAM_1_RAM_BLOCK_TYPE "AUTO"
#define DATA_RAM_1_READ_DURING_WRITE_MODE "OLD_DATA"
#define DATA_RAM_1_SINGLE_CLOCK_OP 1
#define DATA_RAM_1_SIZE_MULTIPLE 1
#define DATA_RAM_1_SIZE_VALUE 1024
#define DATA_RAM_1_SPAN 1024
#define DATA_RAM_1_TYPE "altera_avalon_onchip_memory2"
#define DATA_RAM_1_WRITABLE 1


/*
 * hal configuration
 *
 */

#define ALT_INCLUDE_INSTRUCTION_RELATED_EXCEPTION_API
#define ALT_MAX_FD 4
#define ALT_SYS_CLK none
#define ALT_TIMESTAMP_CLK none


/*
 * i2c_master_0 configuration
 *
 */

#define ALT_MODULE_CLASS_i2c_master_0 i2c_master
#define I2C_MASTER_0_BASE 0xe400
#define I2C_MASTER_0_IRQ 5
#define I2C_MASTER_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define I2C_MASTER_0_NAME "/dev/i2c_master_0"
#define I2C_MASTER_0_SPAN 16
#define I2C_MASTER_0_TYPE "i2c_master"


/*
 * imu_spim configuration
 *
 */

#define ALT_MODULE_CLASS_imu_spim imu_spim
#define IMU_SPIM_BASE 0xe600
#define IMU_SPIM_IRQ -1
#define IMU_SPIM_IRQ_INTERRUPT_CONTROLLER_ID -1
#define IMU_SPIM_NAME "/dev/imu_spim"
#define IMU_SPIM_SPAN 16
#define IMU_SPIM_TYPE "imu_spim"


/*
 * instruction_rom_0 configuration
 *
 */

#define ALT_MODULE_CLASS_instruction_rom_0 altera_avalon_onchip_memory2
#define INSTRUCTION_ROM_0_ALLOW_IN_SYSTEM_MEMORY_CONTENT_EDITOR 0
#define INSTRUCTION_ROM_0_ALLOW_MRAM_SIM_CONTENTS_ONLY_FILE 0
#define INSTRUCTION_ROM_0_BASE 0x0
#define INSTRUCTION_ROM_0_CONTENTS_INFO ""
#define INSTRUCTION_ROM_0_DUAL_PORT 1
#define INSTRUCTION_ROM_0_GUI_RAM_BLOCK_TYPE "AUTO"
#define INSTRUCTION_ROM_0_INIT_CONTENTS_FILE "controller_instruction_rom_0"
#define INSTRUCTION_ROM_0_INIT_MEM_CONTENT 1
#define INSTRUCTION_ROM_0_INSTANCE_ID "NONE"
#define INSTRUCTION_ROM_0_IRQ -1
#define INSTRUCTION_ROM_0_IRQ_INTERRUPT_CONTROLLER_ID -1
#define INSTRUCTION_ROM_0_NAME "/dev/instruction_rom_0"
#define INSTRUCTION_ROM_0_NON_DEFAULT_INIT_FILE_ENABLED 0
#define INSTRUCTION_ROM_0_RAM_BLOCK_TYPE "AUTO"
#define INSTRUCTION_ROM_0_READ_DURING_WRITE_MODE "OLD_DATA"
#define INSTRUCTION_ROM_0_SINGLE_CLOCK_OP 1
#define INSTRUCTION_ROM_0_SIZE_MULTIPLE 1
#define INSTRUCTION_ROM_0_SIZE_VALUE 32768
#define INSTRUCTION_ROM_0_SPAN 32768
#define INSTRUCTION_ROM_0_TYPE "altera_avalon_onchip_memory2"
#define INSTRUCTION_ROM_0_WRITABLE 1


/*
 * jtag_uart_0 configuration
 *
 */

#define ALT_MODULE_CLASS_jtag_uart_0 altera_avalon_jtag_uart
#define JTAG_UART_0_BASE 0xc900
#define JTAG_UART_0_IRQ 4
#define JTAG_UART_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define JTAG_UART_0_NAME "/dev/jtag_uart_0"
#define JTAG_UART_0_READ_DEPTH 512
#define JTAG_UART_0_READ_THRESHOLD 8
#define JTAG_UART_0_SPAN 8
#define JTAG_UART_0_TYPE "altera_avalon_jtag_uart"
#define JTAG_UART_0_WRITE_DEPTH 512
#define JTAG_UART_0_WRITE_THRESHOLD 8


/*
 * motor_controller_5 configuration
 *
 */

#define ALT_MODULE_CLASS_motor_controller_5 motor_controller
#define MOTOR_CONTROLLER_5_BASE 0xf100
#define MOTOR_CONTROLLER_5_IRQ 8
#define MOTOR_CONTROLLER_5_IRQ_INTERRUPT_CONTROLLER_ID 0
#define MOTOR_CONTROLLER_5_NAME "/dev/motor_controller_5"
#define MOTOR_CONTROLLER_5_SPAN 8
#define MOTOR_CONTROLLER_5_TYPE "motor_controller"


/*
 * msgdma_0_csr configuration
 *
 */

#define ALT_MODULE_CLASS_msgdma_0_csr altera_msgdma
#define MSGDMA_0_CSR_BASE 0xc400
#define MSGDMA_0_CSR_BURST_ENABLE 0
#define MSGDMA_0_CSR_BURST_WRAPPING_SUPPORT 0
#define MSGDMA_0_CSR_CHANNEL_ENABLE 1
#define MSGDMA_0_CSR_CHANNEL_ENABLE_DERIVED 1
#define MSGDMA_0_CSR_CHANNEL_WIDTH 8
#define MSGDMA_0_CSR_DATA_FIFO_DEPTH 128
#define MSGDMA_0_CSR_DATA_WIDTH 8
#define MSGDMA_0_CSR_DESCRIPTOR_FIFO_DEPTH 128
#define MSGDMA_0_CSR_DMA_MODE 1
#define MSGDMA_0_CSR_ENHANCED_FEATURES 0
#define MSGDMA_0_CSR_ERROR_ENABLE 0
#define MSGDMA_0_CSR_ERROR_ENABLE_DERIVED 0
#define MSGDMA_0_CSR_ERROR_WIDTH 8
#define MSGDMA_0_CSR_IRQ 0
#define MSGDMA_0_CSR_IRQ_INTERRUPT_CONTROLLER_ID 0
#define MSGDMA_0_CSR_MAX_BURST_COUNT 2
#define MSGDMA_0_CSR_MAX_BYTE 1024
#define MSGDMA_0_CSR_MAX_STRIDE 1
#define MSGDMA_0_CSR_NAME "/dev/msgdma_0_csr"
#define MSGDMA_0_CSR_PACKET_ENABLE 1
#define MSGDMA_0_CSR_PACKET_ENABLE_DERIVED 1
#define MSGDMA_0_CSR_PREFETCHER_ENABLE 0
#define MSGDMA_0_CSR_PROGRAMMABLE_BURST_ENABLE 0
#define MSGDMA_0_CSR_RESPONSE_PORT 2
#define MSGDMA_0_CSR_SPAN 32
#define MSGDMA_0_CSR_STRIDE_ENABLE 0
#define MSGDMA_0_CSR_STRIDE_ENABLE_DERIVED 0
#define MSGDMA_0_CSR_TRANSFER_TYPE "Aligned Accesses"
#define MSGDMA_0_CSR_TYPE "altera_msgdma"


/*
 * msgdma_0_descriptor_slave configuration
 *
 */

#define ALT_MODULE_CLASS_msgdma_0_descriptor_slave altera_msgdma
#define MSGDMA_0_DESCRIPTOR_SLAVE_BASE 0xc500
#define MSGDMA_0_DESCRIPTOR_SLAVE_BURST_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_BURST_WRAPPING_SUPPORT 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_CHANNEL_ENABLE 1
#define MSGDMA_0_DESCRIPTOR_SLAVE_CHANNEL_ENABLE_DERIVED 1
#define MSGDMA_0_DESCRIPTOR_SLAVE_CHANNEL_WIDTH 8
#define MSGDMA_0_DESCRIPTOR_SLAVE_DATA_FIFO_DEPTH 128
#define MSGDMA_0_DESCRIPTOR_SLAVE_DATA_WIDTH 8
#define MSGDMA_0_DESCRIPTOR_SLAVE_DESCRIPTOR_FIFO_DEPTH 128
#define MSGDMA_0_DESCRIPTOR_SLAVE_DMA_MODE 1
#define MSGDMA_0_DESCRIPTOR_SLAVE_ENHANCED_FEATURES 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_ERROR_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_ERROR_ENABLE_DERIVED 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_ERROR_WIDTH 8
#define MSGDMA_0_DESCRIPTOR_SLAVE_IRQ -1
#define MSGDMA_0_DESCRIPTOR_SLAVE_IRQ_INTERRUPT_CONTROLLER_ID -1
#define MSGDMA_0_DESCRIPTOR_SLAVE_MAX_BURST_COUNT 2
#define MSGDMA_0_DESCRIPTOR_SLAVE_MAX_BYTE 1024
#define MSGDMA_0_DESCRIPTOR_SLAVE_MAX_STRIDE 1
#define MSGDMA_0_DESCRIPTOR_SLAVE_NAME "/dev/msgdma_0_descriptor_slave"
#define MSGDMA_0_DESCRIPTOR_SLAVE_PACKET_ENABLE 1
#define MSGDMA_0_DESCRIPTOR_SLAVE_PACKET_ENABLE_DERIVED 1
#define MSGDMA_0_DESCRIPTOR_SLAVE_PREFETCHER_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_PROGRAMMABLE_BURST_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_RESPONSE_PORT 2
#define MSGDMA_0_DESCRIPTOR_SLAVE_SPAN 16
#define MSGDMA_0_DESCRIPTOR_SLAVE_STRIDE_ENABLE 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_STRIDE_ENABLE_DERIVED 0
#define MSGDMA_0_DESCRIPTOR_SLAVE_TRANSFER_TYPE "Aligned Accesses"
#define MSGDMA_0_DESCRIPTOR_SLAVE_TYPE "altera_msgdma"


/*
 * performance_counter_0 configuration
 *
 */

#define ALT_MODULE_CLASS_performance_counter_0 altera_avalon_performance_counter
#define PERFORMANCE_COUNTER_0_BASE 0xc700
#define PERFORMANCE_COUNTER_0_HOW_MANY_SECTIONS 1
#define PERFORMANCE_COUNTER_0_IRQ -1
#define PERFORMANCE_COUNTER_0_IRQ_INTERRUPT_CONTROLLER_ID -1
#define PERFORMANCE_COUNTER_0_NAME "/dev/performance_counter_0"
#define PERFORMANCE_COUNTER_0_SPAN 32
#define PERFORMANCE_COUNTER_0_TYPE "altera_avalon_performance_counter"


/*
 * pio_0 configuration
 *
 */

#define ALT_MODULE_CLASS_pio_0 altera_avalon_pio
#define PIO_0_BASE 0xe000
#define PIO_0_BIT_CLEARING_EDGE_REGISTER 0
#define PIO_0_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PIO_0_CAPTURE 1
#define PIO_0_DATA_WIDTH 1
#define PIO_0_DO_TEST_BENCH_WIRING 1
#define PIO_0_DRIVEN_SIM_VALUE 0
#define PIO_0_EDGE_TYPE "RISING"
#define PIO_0_FREQ 75000000
#define PIO_0_HAS_IN 1
#define PIO_0_HAS_OUT 0
#define PIO_0_HAS_TRI 0
#define PIO_0_IRQ 2
#define PIO_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define PIO_0_IRQ_TYPE "EDGE"
#define PIO_0_NAME "/dev/pio_0"
#define PIO_0_RESET_VALUE 0
#define PIO_0_SPAN 16
#define PIO_0_TYPE "altera_avalon_pio"


/*
 * pio_1 configuration
 *
 */

#define ALT_MODULE_CLASS_pio_1 altera_avalon_pio
#define PIO_1_BASE 0xe100
#define PIO_1_BIT_CLEARING_EDGE_REGISTER 0
#define PIO_1_BIT_MODIFYING_OUTPUT_REGISTER 0
#define PIO_1_CAPTURE 0
#define PIO_1_DATA_WIDTH 32
#define PIO_1_DO_TEST_BENCH_WIRING 1
#define PIO_1_DRIVEN_SIM_VALUE -1
#define PIO_1_EDGE_TYPE "NONE"
#define PIO_1_FREQ 75000000
#define PIO_1_HAS_IN 1
#define PIO_1_HAS_OUT 0
#define PIO_1_HAS_TRI 0
#define PIO_1_IRQ 3
#define PIO_1_IRQ_INTERRUPT_CONTROLLER_ID 0
#define PIO_1_IRQ_TYPE "LEVEL"
#define PIO_1_NAME "/dev/pio_1"
#define PIO_1_RESET_VALUE 0
#define PIO_1_SPAN 16
#define PIO_1_TYPE "altera_avalon_pio"


/*
 * pio_2 configuration
 *
 */

#define ALT_MODULE_CLASS_pio_2 altera_avalon_pio
#define PIO_2_BASE 0xe200
#define PIO_2_BIT_CLEARING_EDGE_REGISTER 0
#define PIO_2_BIT_MODIFYING_OUTPUT_REGISTER 1
#define PIO_2_CAPTURE 0
#define PIO_2_DATA_WIDTH 10
#define PIO_2_DO_TEST_BENCH_WIRING 0
#define PIO_2_DRIVEN_SIM_VALUE 0
#define PIO_2_EDGE_TYPE "NONE"
#define PIO_2_FREQ 75000000
#define PIO_2_HAS_IN 0
#define PIO_2_HAS_OUT 1
#define PIO_2_HAS_TRI 0
#define PIO_2_IRQ -1
#define PIO_2_IRQ_INTERRUPT_CONTROLLER_ID -1
#define PIO_2_IRQ_TYPE "NONE"
#define PIO_2_NAME "/dev/pio_2"
#define PIO_2_RESET_VALUE 0
#define PIO_2_SPAN 32
#define PIO_2_TYPE "altera_avalon_pio"


/*
 * spim_0 configuration
 *
 */

#define ALT_MODULE_CLASS_spim_0 altera_avalon_spi
#define SPIM_0_BASE 0xe500
#define SPIM_0_CLOCKMULT 1
#define SPIM_0_CLOCKPHASE 0
#define SPIM_0_CLOCKPOLARITY 0
#define SPIM_0_CLOCKUNITS "Hz"
#define SPIM_0_DATABITS 8
#define SPIM_0_DATAWIDTH 16
#define SPIM_0_DELAYMULT "1.0E-9"
#define SPIM_0_DELAYUNITS "ns"
#define SPIM_0_EXTRADELAY 0
#define SPIM_0_INSERT_SYNC 0
#define SPIM_0_IRQ 6
#define SPIM_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define SPIM_0_ISMASTER 1
#define SPIM_0_LSBFIRST 0
#define SPIM_0_NAME "/dev/spim_0"
#define SPIM_0_NUMSLAVES 1
#define SPIM_0_PREFIX "spi_"
#define SPIM_0_SPAN 32
#define SPIM_0_SYNC_REG_DEPTH 2
#define SPIM_0_TARGETCLOCK 20000000u
#define SPIM_0_TARGETSSDELAY "0.0"
#define SPIM_0_TYPE "altera_avalon_spi"


/*
 * sysid_qsys_0 configuration
 *
 */

#define ALT_MODULE_CLASS_sysid_qsys_0 altera_avalon_sysid_qsys
#define SYSID_QSYS_0_BASE 0xc600
#define SYSID_QSYS_0_ID -889275714
#define SYSID_QSYS_0_IRQ -1
#define SYSID_QSYS_0_IRQ_INTERRUPT_CONTROLLER_ID -1
#define SYSID_QSYS_0_NAME "/dev/sysid_qsys_0"
#define SYSID_QSYS_0_SPAN 8
#define SYSID_QSYS_0_TIMESTAMP 1633185741
#define SYSID_QSYS_0_TYPE "altera_avalon_sysid_qsys"


/*
 * timer_0 configuration
 *
 */

#define ALT_MODULE_CLASS_timer_0 altera_avalon_timer
#define TIMER_0_ALWAYS_RUN 0
#define TIMER_0_BASE 0xc800
#define TIMER_0_COUNTER_SIZE 32
#define TIMER_0_FIXED_PERIOD 1
#define TIMER_0_FREQ 75000000
#define TIMER_0_IRQ 1
#define TIMER_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define TIMER_0_LOAD_VALUE 149999
#define TIMER_0_MULT 0.001
#define TIMER_0_NAME "/dev/timer_0"
#define TIMER_0_PERIOD 2
#define TIMER_0_PERIOD_UNITS "ms"
#define TIMER_0_RESET_OUTPUT 0
#define TIMER_0_SNAPSHOT 0
#define TIMER_0_SPAN 32
#define TIMER_0_TICKS_PER_SEC 500
#define TIMER_0_TIMEOUT_PULSE_OUTPUT 0
#define TIMER_0_TYPE "altera_avalon_timer"


/*
 * vector_controller_master_0 configuration
 *
 */

#define ALT_MODULE_CLASS_vector_controller_master_0 vector_controller_master
#define VECTOR_CONTROLLER_MASTER_0_BASE 0xf000
#define VECTOR_CONTROLLER_MASTER_0_IRQ 7
#define VECTOR_CONTROLLER_MASTER_0_IRQ_INTERRUPT_CONTROLLER_ID 0
#define VECTOR_CONTROLLER_MASTER_0_NAME "/dev/vector_controller_master_0"
#define VECTOR_CONTROLLER_MASTER_0_SPAN 64
#define VECTOR_CONTROLLER_MASTER_0_TYPE "vector_controller_master"


/*
 * vic_0 configuration
 *
 */

#define ALT_MODULE_CLASS_vic_0 altera_vic
#define VIC_0_BASE 0xc000
#define VIC_0_DAISY_CHAIN_ENABLE 0
#define VIC_0_INTERRUPT_CONTROLLER_ID 0
#define VIC_0_IRQ -1
#define VIC_0_IRQ_INTERRUPT_CONTROLLER_ID -1
#define VIC_0_NAME "/dev/vic_0"
#define VIC_0_NUMBER_OF_INT_PORTS 9
#define VIC_0_RIL_WIDTH 2
#define VIC_0_SPAN 1024
#define VIC_0_TYPE "altera_vic"

#endif /* __SYSTEM_H_ */
