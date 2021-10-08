/**
 * @file i2c_master.hpp
 * @author Fujii Naomichi
 * @copyright (c) 2021 Fujii Naomichi
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <system.h>

struct I2CMasterRegisters {
    volatile uint16_t CONTROL;
    volatile uint16_t STATUS;
    volatile uint16_t INTFLAG;
    volatile uint16_t DADDR;
    volatile uint16_t IADDR;
    volatile uint16_t TXDATA;
    volatile uint16_t RXDATA;
    volatile uint16_t _reserved0;
};

static inline void I2CM_BusReset(uint32_t base) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->CONTROL, 0x0001);
}

static inline void I2CM_GetStatus(uint32_t base, bool *busy, bool *acked) {
    int status = __builtin_ldhuio(&((I2CMasterRegisters*)base)->STATUS);
    if (busy) *busy = (status & 0x1) ? true : false;
    if (acked) *acked = (status & 0x2) ? true : false;
}

static inline bool I2CM_IsBusy(uint32_t base) {
    int intflag = __builtin_ldhuio(&((I2CMasterRegisters*)base)->STATUS);
    return (intflag & 0x1) ? true : false;
}

static inline bool I2CM_IsAcked(uint32_t base) {
    int intflag = __builtin_ldhuio(&((I2CMasterRegisters*)base)->STATUS);
    return (intflag & 0x2) ? true : false;
}

static inline bool I2CM_IsComplete(uint32_t base) {
    int intflag = __builtin_ldhuio(&((I2CMasterRegisters*)base)->INTFLAG);
    return (intflag & 0x1) ? true : false;
}

static inline void I2CM_ClearComplete(uint32_t base) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->INTFLAG, 0);
}

static inline void I2CM_SetDeviceAddress(uint32_t base, int address) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->DADDR, address);
}

static inline void I2CM_Write1Byte(uint32_t base, int data) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->TXDATA, data);
    __builtin_sthio(&((I2CMasterRegisters*)base)->CONTROL, 0x0002);
}

static inline void I2CM_Write2Byte(uint32_t base, int data) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->TXDATA, data);
    __builtin_sthio(&((I2CMasterRegisters*)base)->CONTROL, 0x0102);
}

static inline void I2CM_WriteRegister1Byte(uint32_t base, int address, int data) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->IADDR, address);
    __builtin_sthio(&((I2CMasterRegisters*)base)->TXDATA, data);
    __builtin_sthio(&((I2CMasterRegisters*)base)->CONTROL, 0x0042);
}

static inline void I2CM_WriteRegister2Byte(uint32_t base, int address, int data) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->IADDR, address);
    __builtin_sthio(&((I2CMasterRegisters*)base)->TXDATA, data);
    __builtin_sthio(&((I2CMasterRegisters*)base)->CONTROL, 0x0142);
}

static inline void I2CM_Read1Byte(uint32_t base) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->CONTROL, 0x0006);
}

static inline void I2CM_Read2Byte(uint32_t base) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->CONTROL, 0x0106);
}

static inline void I2CM_ReadRegister1Byte(uint32_t base, int address) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->IADDR, address);
    __builtin_sthio(&((I2CMasterRegisters*)base)->CONTROL, 0x0046);
}

static inline void I2CM_ReadRegister2Byte(uint32_t base, int address) {
    __builtin_sthio(&((I2CMasterRegisters*)base)->IADDR, address);
    __builtin_sthio(&((I2CMasterRegisters*)base)->CONTROL, 0x0146);
}

static inline int I2CM_GetReadResult1Byte(uint32_t base) {
    return __builtin_ldbuio(&((I2CMasterRegisters*)base)->RXDATA);
}

static inline int I2CM_GetReadResult2Byte(uint32_t base) {
    return __builtin_ldhuio(&((I2CMasterRegisters*)base)->RXDATA);
}
