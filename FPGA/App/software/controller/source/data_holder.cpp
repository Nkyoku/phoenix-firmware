#include "data_holder.hpp"
#include <board.hpp>
#include <driver/adc2.hpp>
#include <peripheral/imu_spim.hpp>
#include <peripheral/vector_controller.hpp>
#include <peripheral/motor_controller.hpp>
#include "wheel_controller.hpp"

void DataHolder::FetchRegistersOnPreControlLoop(void){
    _MotionData.Imu.AccelX = IMU_SPIM_GetAccelDataX(IMU_SPIM_BASE) * IMU_ACCELEROMETER_SCALE;
    _MotionData.Imu.AccelY = IMU_SPIM_GetAccelDataY(IMU_SPIM_BASE) * IMU_ACCELEROMETER_SCALE;
    _MotionData.Imu.AccelZ = IMU_SPIM_GetAccelDataZ(IMU_SPIM_BASE) * IMU_ACCELEROMETER_SCALE;
    _MotionData.Imu.GyroX = IMU_SPIM_GetGyroDataX(IMU_SPIM_BASE) * IMU_GYROSCOPE_SCALE;
    _MotionData.Imu.GyroY = IMU_SPIM_GetGyroDataY(IMU_SPIM_BASE) * IMU_GYROSCOPE_SCALE;
    _MotionData.Imu.GyroZ = IMU_SPIM_GetGyroDataZ(IMU_SPIM_BASE) * IMU_GYROSCOPE_SCALE;

    static constexpr float ENCODER_SCALE = IMU_OUTPUT_RATE / ENCODER_PPR * WHEEL_CIRCUMFERENCE;
    _MotionData.Wheels[0].Velocity = VectorController::GetEncoderValue(1) * ENCODER_SCALE;
    _MotionData.Wheels[1].Velocity = VectorController::GetEncoderValue(2) * ENCODER_SCALE;
    _MotionData.Wheels[2].Velocity = VectorController::GetEncoderValue(3) * ENCODER_SCALE;
    _MotionData.Wheels[3].Velocity = VectorController::GetEncoderValue(4) * ENCODER_SCALE;
    _MotionData.Wheels[0].CurrentD = VectorController::GetCurrentMeasurementD(1) * ADC1_CURRENT_SCALE;
    _MotionData.Wheels[1].CurrentD = VectorController::GetCurrentMeasurementD(2) * ADC1_CURRENT_SCALE;
    _MotionData.Wheels[2].CurrentD = VectorController::GetCurrentMeasurementD(3) * ADC1_CURRENT_SCALE;
    _MotionData.Wheels[3].CurrentD = VectorController::GetCurrentMeasurementD(4) * ADC1_CURRENT_SCALE;
    _MotionData.Wheels[0].CurrentQ = VectorController::GetCurrentMeasurementQ(1) * ADC1_CURRENT_SCALE;
    _MotionData.Wheels[1].CurrentQ = VectorController::GetCurrentMeasurementQ(2) * ADC1_CURRENT_SCALE;
    _MotionData.Wheels[2].CurrentQ = VectorController::GetCurrentMeasurementQ(3) * ADC1_CURRENT_SCALE;
    _MotionData.Wheels[3].CurrentQ = VectorController::GetCurrentMeasurementQ(4) * ADC1_CURRENT_SCALE;
    _ControlData.Wheels[0].VelocityRef = WheelController::GetWheelVelocityReference()[0];
    _ControlData.Wheels[1].VelocityRef = WheelController::GetWheelVelocityReference()[1];
    _ControlData.Wheels[2].VelocityRef = WheelController::GetWheelVelocityReference()[2];
    _ControlData.Wheels[3].VelocityRef = WheelController::GetWheelVelocityReference()[3];
    _ControlData.Wheels[0].CurrentRef = VectorController::GetCurrentReferenceQ(1) * ADC1_CURRENT_SCALE;
    _ControlData.Wheels[1].CurrentRef = VectorController::GetCurrentReferenceQ(2) * ADC1_CURRENT_SCALE;
    _ControlData.Wheels[2].CurrentRef = VectorController::GetCurrentReferenceQ(3) * ADC1_CURRENT_SCALE;
    _ControlData.Wheels[3].CurrentRef = VectorController::GetCurrentReferenceQ(4) * ADC1_CURRENT_SCALE;
    _ControlData.Wheels[0].Energy = WheelController::GetWheelRenegerationEnergy()[0];
    _ControlData.Wheels[1].Energy = WheelController::GetWheelRenegerationEnergy()[1];
    _ControlData.Wheels[2].Energy = WheelController::GetWheelRenegerationEnergy()[2];
    _ControlData.Wheels[3].Energy = WheelController::GetWheelRenegerationEnergy()[3];
    _ControlData.MachineVelocity[0] = WheelController::GetEstimatedMachineVelocity()[0];
    _ControlData.MachineVelocity[1] = WheelController::GetEstimatedMachineVelocity()[1];
    _ControlData.MachineVelocity[2] = WheelController::GetEstimatedMachineVelocity()[2];
}

void DataHolder::FetchAdc2Result(void){
    float dc48v_voltage = Adc2::GetDc48v();
    float dribble_current = Adc2::GetDribbleCurrent();
    float dribble_voltage = dc48v_voltage * MotorController::GetPower() * (1.0f / MotorController::FULL_SCALE_OF_POWER);
    _Adc2Data.Dc48vVoltage = dc48v_voltage;
    _Adc2Data.DribbleVoltage = dribble_voltage;
    _Adc2Data.DribbleCurrent = dribble_current;
}

Adc2Data_t DataHolder::_Adc2Data;
MotionData_t DataHolder::_MotionData;
ControlData_t DataHolder::_ControlData;
