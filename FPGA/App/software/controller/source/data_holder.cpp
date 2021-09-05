#include "data_holder.hpp"
#include "board.hpp"
#include <driver/adc2.hpp>
#include <peripheral/imu_spim.hpp>
#include <peripheral/vector_controller.hpp>
#include <peripheral/motor_controller.hpp>
#include "wheel_controller.hpp"

void DataHolder::fetchOnPreControlLoop(void) {
    static constexpr float ENCODER_SCALE = IMU_OUTPUT_RATE / ENCODER_PPR * WHEEL_CIRCUMFERENCE;
    _motion_data.accelerometer.x() = IMU_SPIM_GetAccelDataX(IMU_SPIM_BASE) * IMU_ACCELEROMETER_SCALE;
    _motion_data.accelerometer.y() = IMU_SPIM_GetAccelDataY(IMU_SPIM_BASE) * IMU_ACCELEROMETER_SCALE;
    _motion_data.accelerometer.z() = IMU_SPIM_GetAccelDataZ(IMU_SPIM_BASE) * IMU_ACCELEROMETER_SCALE;
    _motion_data.gyroscope.x() = IMU_SPIM_GetGyroDataX(IMU_SPIM_BASE) * IMU_GYROSCOPE_SCALE;
    _motion_data.gyroscope.y() = IMU_SPIM_GetGyroDataY(IMU_SPIM_BASE) * IMU_GYROSCOPE_SCALE;
    _motion_data.gyroscope.z() = IMU_SPIM_GetGyroDataZ(IMU_SPIM_BASE) * IMU_GYROSCOPE_SCALE;
    _motion_data.wheel_velocity(0) = VectorController::getEncoderValue(1) * ENCODER_SCALE;
    _motion_data.wheel_velocity(1) = VectorController::getEncoderValue(2) * ENCODER_SCALE;
    _motion_data.wheel_velocity(2) = VectorController::getEncoderValue(3) * ENCODER_SCALE;
    _motion_data.wheel_velocity(3) = VectorController::getEncoderValue(4) * ENCODER_SCALE;
    _motion_data.wheel_current_d(0) = VectorController::getCurrentMeasurementD(1) * ADC1_CURRENT_SCALE;
    _motion_data.wheel_current_d(1) = VectorController::getCurrentMeasurementD(2) * ADC1_CURRENT_SCALE;
    _motion_data.wheel_current_d(2) = VectorController::getCurrentMeasurementD(3) * ADC1_CURRENT_SCALE;
    _motion_data.wheel_current_d(3) = VectorController::getCurrentMeasurementD(4) * ADC1_CURRENT_SCALE;
    _motion_data.wheel_current_q(0) = VectorController::getCurrentMeasurementQ(1) * ADC1_CURRENT_SCALE;
    _motion_data.wheel_current_q(1) = VectorController::getCurrentMeasurementQ(2) * ADC1_CURRENT_SCALE;
    _motion_data.wheel_current_q(2) = VectorController::getCurrentMeasurementQ(3) * ADC1_CURRENT_SCALE;
    _motion_data.wheel_current_q(3) = VectorController::getCurrentMeasurementQ(4) * ADC1_CURRENT_SCALE;
}

void DataHolder::fetchOnPostControlLoop(void) {
    _motion_data.gravity = WheelController::gravityFilter().gravity();
    _motion_data.body_acceleration = WheelController::gravityFilter().acceleration();
    _motion_data.body_velocity = WheelController::velocityFilter().bodyVelocity();
    _control_data.current_ref(0) = VectorController::getCurrentReferenceQ(1) * ADC1_CURRENT_SCALE;
    _control_data.current_ref(1) = VectorController::getCurrentReferenceQ(2) * ADC1_CURRENT_SCALE;
    _control_data.current_ref(2) = VectorController::getCurrentReferenceQ(3) * ADC1_CURRENT_SCALE;
    _control_data.current_ref(3) = VectorController::getCurrentReferenceQ(4) * ADC1_CURRENT_SCALE;
    _control_data.rotation_torque = WheelController::absBodyTorque();
    _control_data.omega_weight = WheelController::omegaWeight();
    _control_data.body_ref_accel_unlimit = WheelController::referenceAccelerationUnlimited();
    _control_data.body_ref_accel = WheelController::referenceAcceleration();
}

void DataHolder::fetchAdc2Result(void) {
    float dc48v_voltage = Adc2::getDc48v();
    float dribble_current = Adc2::getDribbleCurrent();
    float dribble_voltage = dc48v_voltage * MotorController::getPower() * (1.0f / MotorController::FULL_SCALE_OF_POWER);
    _adc2_data.dc48v_voltage = dc48v_voltage;
    _adc2_data.dribble_voltage = dribble_voltage;
    _adc2_data.dribble_current = dribble_current;
}

Adc2Data_t DataHolder::_adc2_data;
MotionData_t DataHolder::_motion_data;
ControlData_t DataHolder::_control_data;
