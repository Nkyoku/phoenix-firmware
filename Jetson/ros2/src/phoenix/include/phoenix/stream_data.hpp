#pragma once

#include <stdint.h>

enum StreamId_t {
    StreamIdStatus = 1,
    StreamIdAdc2 = 2,
    StreamIdMotion = 3
};

struct StreamDataStatus_t {
    uint32_t error_flags;
    uint32_t fault_flags;
};

struct StreamDataAdc2_t {
    __fp16 dc48v_voltage;
    __fp16 dribble_voltage;
    __fp16 dribble_current;
};

struct StreamDataMotion_t {
    uint16_t performance_counter;
    __fp16 accelerometer[3];
    __fp16 gyroscope[3];
    __fp16 wheel_velocity[4];
    __fp16 wheel_current_meas_d[4];
    __fp16 wheel_current_meas_q[4];
    __fp16 wheel_current_ref_q[4];
};
