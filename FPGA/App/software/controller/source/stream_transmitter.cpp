#include "stream_transmitter.hpp"
#include "centralized_monitor.hpp"
#include "board.hpp"
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
    __builtin_sthio(&StreamDataAdc2.dc48v_voltage, Fp32ToFp16(adc2_data.Dc48vVoltage));
    __builtin_sthio(&StreamDataAdc2.dribble_voltage, Fp32ToFp16(adc2_data.DribbleVoltage));
    __builtin_sthio(&StreamDataAdc2.dribble_current, Fp32ToFp16(adc2_data.DribbleCurrent));
    StreamDataDesciptorAdc2.TransmitAsync(_Device);
}

void StreamTransmitter::TransmitMotion(const MotionData_t &motion_data, const ControlData_t &control_data, int performance_counter) {
    __builtin_sthio(&StreamDataMotion.accelerometer[0], Fp32ToFp16(motion_data.Imu.AccelX));
    __builtin_sthio(&StreamDataMotion.accelerometer[1], Fp32ToFp16(motion_data.Imu.AccelY));
    __builtin_sthio(&StreamDataMotion.accelerometer[2], Fp32ToFp16(motion_data.Imu.AccelZ));
    __builtin_sthio(&StreamDataMotion.gyroscope[0], Fp32ToFp16(motion_data.Imu.GyroX));
    __builtin_sthio(&StreamDataMotion.gyroscope[1], Fp32ToFp16(motion_data.Imu.GyroY));
    __builtin_sthio(&StreamDataMotion.gyroscope[2], Fp32ToFp16(motion_data.Imu.GyroZ));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_meas[0], Fp32ToFp16(motion_data.Wheels[0].Velocity));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_meas[1], Fp32ToFp16(motion_data.Wheels[1].Velocity));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_meas[2], Fp32ToFp16(motion_data.Wheels[2].Velocity));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_meas[3], Fp32ToFp16(motion_data.Wheels[3].Velocity));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_d[0], Fp32ToFp16(motion_data.Wheels[0].CurrentD));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_d[1], Fp32ToFp16(motion_data.Wheels[1].CurrentD));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_d[2], Fp32ToFp16(motion_data.Wheels[2].CurrentD));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_d[3], Fp32ToFp16(motion_data.Wheels[3].CurrentD));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_q[0], Fp32ToFp16(motion_data.Wheels[0].CurrentQ));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_q[1], Fp32ToFp16(motion_data.Wheels[1].CurrentQ));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_q[2], Fp32ToFp16(motion_data.Wheels[2].CurrentQ));
    __builtin_sthio(&StreamDataMotion.wheel_current_meas_q[3], Fp32ToFp16(motion_data.Wheels[3].CurrentQ));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_ref[0], Fp32ToFp16(control_data.Wheels[0].VelocityRef));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_ref[1], Fp32ToFp16(control_data.Wheels[1].VelocityRef));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_ref[2], Fp32ToFp16(control_data.Wheels[2].VelocityRef));
    __builtin_sthio(&StreamDataMotion.wheel_velocity_ref[3], Fp32ToFp16(control_data.Wheels[3].VelocityRef));
    __builtin_sthio(&StreamDataMotion.wheel_current_ref[0], Fp32ToFp16(control_data.Wheels[0].CurrentRef));
    __builtin_sthio(&StreamDataMotion.wheel_current_ref[1], Fp32ToFp16(control_data.Wheels[1].CurrentRef));
    __builtin_sthio(&StreamDataMotion.wheel_current_ref[2], Fp32ToFp16(control_data.Wheels[2].CurrentRef));
    __builtin_sthio(&StreamDataMotion.wheel_current_ref[3], Fp32ToFp16(control_data.Wheels[3].CurrentRef));
    __builtin_sthio(&StreamDataMotion.wheel_current_limit[0], Fp32ToFp16(control_data.Wheels[0].CurrentLimit));
    __builtin_sthio(&StreamDataMotion.wheel_current_limit[1], Fp32ToFp16(control_data.Wheels[1].CurrentLimit));
    __builtin_sthio(&StreamDataMotion.wheel_current_limit[2], Fp32ToFp16(control_data.Wheels[2].CurrentLimit));
    __builtin_sthio(&StreamDataMotion.wheel_current_limit[3], Fp32ToFp16(control_data.Wheels[3].CurrentLimit));
    __builtin_sthio(&StreamDataMotion.machine_velocity[0], Fp32ToFp16(control_data.MachineVelocity[0]));
    __builtin_sthio(&StreamDataMotion.machine_velocity[1], Fp32ToFp16(control_data.MachineVelocity[1]));
    __builtin_sthio(&StreamDataMotion.machine_velocity[2], Fp32ToFp16(control_data.MachineVelocity[2]));
    __builtin_sthio(&StreamDataMotion.slip_flags, control_data.SlipFlags);
    __builtin_sthio(&StreamDataMotion.performance_counter, static_cast<uint16_t>(performance_counter));
    StreamDataDesciptorMotion.TransmitAsync(_Device);
}

alt_msgdma_dev *StreamTransmitter::_Device;
