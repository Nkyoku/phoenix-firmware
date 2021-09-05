#pragma once

#include <stdint.h>

enum StreamId {
    StreamIdStatus = 1,
    StreamIdAdc2 = 2,
    StreamIdMotion = 3
};

struct StreamDataStatus {
    uint32_t error_flags;
    uint32_t fault_flags;
};

struct StreamDataAdc2 {
    __fp16 dc48v_voltage;
    __fp16 dribble_voltage;
    __fp16 dribble_current;
};

struct StreamDataMotion {
    __fp16 accelerometer[3];
    __fp16 gyroscope[3];
    __fp16 gravity[3];
    __fp16 body_acceleration[3];
    __fp16 body_velocity[3];
    __fp16 wheel_velocity_meas[4];
    __fp16 wheel_current_meas_d[4];
    __fp16 wheel_current_meas_q[4];
    __fp16 wheel_current_ref[4];
    __fp16 body_ref_accel_unlimit[4];
    __fp16 body_ref_accel[4];
    __fp16 rotation_torque;
    __fp16 omega_weight;
    uint16_t performance_counter;
};
