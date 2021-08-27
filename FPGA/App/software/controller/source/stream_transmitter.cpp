#include "stream_transmitter.hpp"
#include "centralized_monitor.hpp"
#include <fpu.hpp>
#include <stream_data.hpp>
#include <peripheral/msgdma.hpp>

static StreamDataStatus_t StreamDataStatus;
static constexpr MsgdmaTransmitDescriptor StreamDataDesciptorStatus(StreamDataStatus, StreamIdStatus);

static StreamDataAdc2_t StreamDataAdc2;
static constexpr MsgdmaTransmitDescriptor StreamDataDesciptorAdc2(StreamDataAdc2, StreamIdAdc2);

static StreamDataMotion_t StreamDataMotion;
static constexpr MsgdmaTransmitDescriptor StreamDataDesciptorMotion(StreamDataMotion, StreamIdMotion);

void StreamTransmitter::TransmitStatus(void) {
    // データキャッシュが有効になっている場合に備えてデータの格納には__builtin_st〇io()という系列のビルトイン関数を使用する
    __builtin_stwio(&StreamDataStatus.error_flags, CentralizedMonitor::GetErrorFlags());
    __builtin_stwio(&StreamDataStatus.fault_flags, CentralizedMonitor::GetFaultFlags());
    StreamDataDesciptorStatus.TransmitAsync(_Device);
}

void StreamTransmitter::TransmitAdc2(const Adc2Data_t &adc2_data) {
    __builtin_sthio(&StreamDataAdc2.dc48v_voltage, fpu::to_fp16(adc2_data.dc48v_voltage));
    __builtin_sthio(&StreamDataAdc2.dribble_voltage, fpu::to_fp16(adc2_data.dribble_voltage));
    __builtin_sthio(&StreamDataAdc2.dribble_current, fpu::to_fp16(adc2_data.dribble_current));
    StreamDataDesciptorAdc2.TransmitAsync(_Device);
}

void StreamTransmitter::TransmitMotion(const MotionData_t &motion_data, const ControlData_t &control_data, int performance_counter) {
    __builtin_sthio(&StreamDataMotion.accelerometer[0], fpu::to_fp16(motion_data.accelerometer(0)));
    __builtin_sthio(&StreamDataMotion.accelerometer[1], fpu::to_fp16(motion_data.accelerometer(1)));
    __builtin_sthio(&StreamDataMotion.accelerometer[2], fpu::to_fp16(motion_data.accelerometer(2)));
    __builtin_sthio(&StreamDataMotion.gyroscope[0], fpu::to_fp16(motion_data.gyroscope(0)));
    __builtin_sthio(&StreamDataMotion.gyroscope[1], fpu::to_fp16(motion_data.gyroscope(1)));
    __builtin_sthio(&StreamDataMotion.gyroscope[2], fpu::to_fp16(motion_data.gyroscope(2)));
    __builtin_sthio(&StreamDataMotion.gravity[0], fpu::to_fp16(motion_data.gravity(0)));
    __builtin_sthio(&StreamDataMotion.gravity[1], fpu::to_fp16(motion_data.gravity(1)));
    __builtin_sthio(&StreamDataMotion.gravity[2], fpu::to_fp16(motion_data.gravity(2)));
    __builtin_sthio(&StreamDataMotion.body_acceleration[0], fpu::to_fp16(motion_data.body_acceleration(0)));
    __builtin_sthio(&StreamDataMotion.body_acceleration[1], fpu::to_fp16(motion_data.body_acceleration(1)));
    __builtin_sthio(&StreamDataMotion.body_acceleration[2], fpu::to_fp16(motion_data.body_acceleration(2)));
    __builtin_sthio(&StreamDataMotion.body_velocity[0], fpu::to_fp16(motion_data.body_velocity(0)));
    __builtin_sthio(&StreamDataMotion.body_velocity[1], fpu::to_fp16(motion_data.body_velocity(1)));
    __builtin_sthio(&StreamDataMotion.body_velocity[2], fpu::to_fp16(motion_data.body_velocity(2)));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_meas[0], fpu::to_fp16(motion_data.wheel_velocity(0)));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_meas[1], fpu::to_fp16(motion_data.wheel_velocity(1)));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_meas[2], fpu::to_fp16(motion_data.wheel_velocity(2)));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_meas[3], fpu::to_fp16(motion_data.wheel_velocity(3)));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_d[0], fpu::to_fp16(motion_data.wheel_current_d(0)));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_d[1], fpu::to_fp16(motion_data.wheel_current_d(1)));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_d[2], fpu::to_fp16(motion_data.wheel_current_d(2)));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_d[3], fpu::to_fp16(motion_data.wheel_current_d(3)));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_q[0], fpu::to_fp16(motion_data.wheel_current_q(0)));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_q[1], fpu::to_fp16(motion_data.wheel_current_q(1)));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_q[2], fpu::to_fp16(motion_data.wheel_current_q(2)));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_q[3], fpu::to_fp16(motion_data.wheel_current_q(3)));
    __builtin_sthio(&StreamDataMotion.wheel_current_ref[0], fpu::to_fp16(control_data.current_ref(0)));
    __builtin_sthio(&StreamDataMotion.wheel_current_ref[1], fpu::to_fp16(control_data.current_ref(1)));
    __builtin_sthio(&StreamDataMotion.wheel_current_ref[2], fpu::to_fp16(control_data.current_ref(2)));
    __builtin_sthio(&StreamDataMotion.wheel_current_ref[3], fpu::to_fp16(control_data.current_ref(3)));
    __builtin_sthio(&StreamDataMotion.body_ref_accel_unlimit[0], fpu::to_fp16(control_data.body_ref_accel_unlimit(0)));
    __builtin_sthio(&StreamDataMotion.body_ref_accel_unlimit[1], fpu::to_fp16(control_data.body_ref_accel_unlimit(1)));
    __builtin_sthio(&StreamDataMotion.body_ref_accel_unlimit[2], fpu::to_fp16(control_data.body_ref_accel_unlimit(2)));
    __builtin_sthio(&StreamDataMotion.body_ref_accel_unlimit[3], fpu::to_fp16(control_data.body_ref_accel_unlimit(3)));
    __builtin_sthio(&StreamDataMotion.body_ref_accel[0], fpu::to_fp16(control_data.body_ref_accel(0)));
    __builtin_sthio(&StreamDataMotion.body_ref_accel[1], fpu::to_fp16(control_data.body_ref_accel(1)));
    __builtin_sthio(&StreamDataMotion.body_ref_accel[2], fpu::to_fp16(control_data.body_ref_accel(2)));
    __builtin_sthio(&StreamDataMotion.body_ref_accel[3], fpu::to_fp16(control_data.body_ref_accel(3)));
    __builtin_sthio(&StreamDataMotion.rotation_torque, fpu::to_fp16(control_data.rotation_torque));
    __builtin_sthio(&StreamDataMotion.omega_weight, fpu::to_fp16(control_data.omega_weight));
    __builtin_sthio(&StreamDataMotion.performance_counter, static_cast<uint16_t>(performance_counter));
    StreamDataDesciptorMotion.TransmitAsync(_Device);
}

alt_msgdma_dev *StreamTransmitter::_Device;
